//----------------------------------------------------------------------------
// TMS9900 Trace - GDB Remote Serial Protocol stub
//
// gdb_stub.cpp - GDB RSP server for interactive debugging
//
// Implements a subset of the GDB Remote Serial Protocol over TCP.
// Supports: register read/write, memory read/write, breakpoints,
// single-step, continue, target description XML.
//
// This is a machine-level compatibility shim. Stock GDB has no TMS9900
// architecture, so the stub exposes an MSP430-shaped register packet for
// register inspection, memory access, stepping, and breakpoints. It does not
// provide correct TMS9900 disassembly or compiler-DWARF register mapping.
//----------------------------------------------------------------------------

#include "gdb_stub.hpp"
#include "cpu.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <set>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <poll.h>

//----------------------------------------------------------------------------
// Target description XML (served to GDB via qXfer:features:read)
//----------------------------------------------------------------------------

// Target description XML for TMS9900.
//
// GDB requires a known <architecture> to accept any target description.
// We use "msp430" (another TI 16-bit CPU) as the architecture hint --
// this just tells GDB to use 16-bit register handling internally.
// MSP430's required feature "org.gnu.gdb.msp430.core" expects r0-r15
// as 32-bit values.  We map them as:
//   r0  (pc)  = TMS9900 PC      r1 (sp) = TMS9900 WP
//   r2  (sr)  = TMS9900 ST      r3-r15  = TMS9900 R0-R12
//
// TMS9900 R13-R15 are accessible via memory: x/3xh <WP+26>
static const char* targetXML =
    "<?xml version=\"1.0\"?>\n"
    "<!DOCTYPE target SYSTEM \"gdb-target.dtd\">\n"
    "<target version=\"1.0\">\n"
    "  <architecture>msp430</architecture>\n"
    "  <feature name=\"org.gnu.gdb.msp430.core\">\n"
    "    <reg name=\"pc\"  bitsize=\"32\" type=\"code_ptr\" regnum=\"0\"/>\n"
    "    <reg name=\"wp\"  bitsize=\"32\" type=\"data_ptr\" regnum=\"1\"/>\n"
    "    <reg name=\"st\"  bitsize=\"32\" regnum=\"2\"/>\n"
    "    <reg name=\"r0\"  bitsize=\"32\" regnum=\"3\"/>\n"
    "    <reg name=\"r1\"  bitsize=\"32\" regnum=\"4\"/>\n"
    "    <reg name=\"r2\"  bitsize=\"32\" regnum=\"5\"/>\n"
    "    <reg name=\"r3\"  bitsize=\"32\" regnum=\"6\"/>\n"
    "    <reg name=\"r4\"  bitsize=\"32\" regnum=\"7\"/>\n"
    "    <reg name=\"r5\"  bitsize=\"32\" regnum=\"8\"/>\n"
    "    <reg name=\"r6\"  bitsize=\"32\" regnum=\"9\"/>\n"
    "    <reg name=\"r7\"  bitsize=\"32\" regnum=\"10\"/>\n"
    "    <reg name=\"r8\"  bitsize=\"32\" regnum=\"11\"/>\n"
    "    <reg name=\"r9\"  bitsize=\"32\" regnum=\"12\"/>\n"
    "    <reg name=\"r10\" bitsize=\"32\" regnum=\"13\"/>\n"
    "    <reg name=\"r11\" bitsize=\"32\" regnum=\"14\"/>\n"
    "    <reg name=\"r12\" bitsize=\"32\" regnum=\"15\"/>\n"
    "  </feature>\n"
    "</target>\n";

//----------------------------------------------------------------------------
// Hex helpers
//----------------------------------------------------------------------------

static const char hexchars[] = "0123456789abcdef";

static int hex_val(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void hex_byte(char* buf, UINT8 val)
{
    buf[0] = hexchars[(val >> 4) & 0xF];
    buf[1] = hexchars[val & 0xF];
}

// Little-endian 32-bit register encoding (GDB MSP430 protocol expects this)
static void hex_reg(char* buf, UINT16 val)
{
    hex_byte(buf,     val & 0xFF);
    hex_byte(buf + 2, (val >> 8) & 0xFF);
    hex_byte(buf + 4, 0);
    hex_byte(buf + 6, 0);
}

static bool parse_hex_range(const char* begin, const char* end,
                            unsigned long& value)
{
    if (begin == end)
    {
        return false;
    }

    value = 0;
    for (const char* current = begin; current != end; ++current)
    {
        const int digit = hex_val(*current);
        if (digit < 0 ||
            value > (std::numeric_limits<unsigned long>::max() - digit) / 16)
        {
            return false;
        }
        value = value * 16 + static_cast<unsigned long>(digit);
    }
    return true;
}

static bool parse_register_value(const char* data, size_t length,
                                 UINT16& value)
{
    if (length != 8)
    {
        return false;
    }
    for (size_t i = 0; i < length; ++i)
    {
        if (hex_val(data[i]) < 0)
        {
            return false;
        }
    }

    const UINT8 low = static_cast<UINT8>((hex_val(data[0]) << 4) |
                                         hex_val(data[1]));
    const UINT8 high = static_cast<UINT8>((hex_val(data[2]) << 4) |
                                          hex_val(data[3]));
    value = static_cast<UINT16>((high << 8) | low);
    return true;
}

static bool set_optional_program_counter(const std::string& packet)
{
    if (packet.size() == 1)
    {
        return true;
    }

    unsigned long address = 0;
    if (!parse_hex_range(packet.data() + 1,
                         packet.data() + packet.size(), address) ||
        address > 0xFFFF)
    {
        return false;
    }
    SetProgramCounter(static_cast<UINT16>(address));
    return true;
}

//----------------------------------------------------------------------------
// RSP packet I/O
//----------------------------------------------------------------------------

static constexpr size_t MaxPacketSize = 4096;

static bool write_all(int fd, const char* data, size_t size)
{
    while (size != 0)
    {
        const ssize_t written = write(fd, data, size);
        if (written < 0 && errno == EINTR)
        {
            continue;
        }
        if (written <= 0)
        {
            return false;
        }
        data += written;
        size -= static_cast<size_t>(written);
    }
    return true;
}

static bool read_byte(int fd, char& value)
{
    for (;;)
    {
        const ssize_t count = read(fd, &value, 1);
        if (count == 1)
        {
            return true;
        }
        if (count < 0 && errno == EINTR)
        {
            continue;
        }
        return false;
    }
}

static bool send_packet(int fd, const std::string& data)
{
    UINT8 cksum = 0;
    for (char c : data)
        cksum += (UINT8)c;

    std::string packet;
    packet.reserve(data.size() + 4);
    packet.push_back('$');
    packet += data;
    packet.push_back('#');
    packet.push_back(hexchars[(cksum >> 4) & 0x0F]);
    packet.push_back(hexchars[cksum & 0x0F]);

    for (;;)
    {
        if (!write_all(fd, packet.data(), packet.size()))
        {
            return false;
        }

        char ack;
        if (!read_byte(fd, ack))
        {
            return false;
        }
        if (ack == '+')
        {
            return true;
        }
        if (ack != '-')
        {
            return false;
        }
    }
}

enum class ReceiveStatus
{
    Packet,
    Break,
    Disconnected
};

struct ReceivedPacket
{
    ReceiveStatus status;
    std::string data;
};

static ReceivedPacket recv_packet(int fd)
{
    for (;;)
    {
        char c;
        do
        {
            if (!read_byte(fd, c))
            {
                return { ReceiveStatus::Disconnected, {} };
            }
            if (c == 0x03)
            {
                return { ReceiveStatus::Break, {} };
            }
        }
        while (c != '$');

        std::string data;
        UINT8 actualChecksum = 0;
        bool oversized = false;
        for (;;)
        {
            if (!read_byte(fd, c))
            {
                return { ReceiveStatus::Disconnected, {} };
            }
            if (c == '#')
            {
                break;
            }
            actualChecksum = static_cast<UINT8>(actualChecksum +
                                                static_cast<UINT8>(c));
            if (data.size() < MaxPacketSize)
            {
                data.push_back(c);
            }
            else
            {
                oversized = true;
            }
        }

        char checksumHigh;
        char checksumLow;
        if (!read_byte(fd, checksumHigh) || !read_byte(fd, checksumLow))
        {
            return { ReceiveStatus::Disconnected, {} };
        }
        const int high = hex_val(checksumHigh);
        const int low = hex_val(checksumLow);
        const bool valid = !oversized && high >= 0 && low >= 0 &&
                           actualChecksum == ((high << 4) | low);

        const char ack = valid ? '+' : '-';
        if (!write_all(fd, &ack, 1))
        {
            return { ReceiveStatus::Disconnected, {} };
        }
        if (valid)
        {
            return { ReceiveStatus::Packet, std::move(data) };
        }

        // Invalid packets are NACKed and discarded; wait for retransmission.
    }
}

//----------------------------------------------------------------------------
// Fire periodic timers if due
//----------------------------------------------------------------------------

static void fire_timers(std::vector<PeriodicTimer>& timers)
{
    for (auto& t : timers)
    {
        while (ClockCycleCounter >= t.nextTriggerCycle)
        {
            TriggerInterrupt(t.irqLevel);
            t.nextTriggerCycle += t.periodCycles;
        }
    }
}

static void advance_idle_to_timer(std::vector<PeriodicTimer>& timers)
{
    if (!IsHalted() || timers.empty()) return;

    UINT64 nextCycle = timers.front().nextTriggerCycle;
    for (const auto& timer : timers)
    {
        nextCycle = std::min(nextCycle, timer.nextTriggerCycle);
    }
    if (ClockCycleCounter < nextCycle)
    {
        ClockCycleCounter = nextCycle;
    }
    fire_timers(timers);
}

static CPUStepResult step_instruction(std::vector<PeriodicTimer>& timers)
{
    for (;;)
    {
        fire_timers(timers);
        advance_idle_to_timer(timers);
        const CPUStepResult result = StepCPU();
        fire_timers(timers);
        if (result.event != CPUEvent::Interrupt)
        {
            return result;
        }
    }
}

//----------------------------------------------------------------------------
// Register access (no clock side effects)
//
// GDB register mapping (16 registers, MSP430 compatible):
//   r0  (pc)  = TMS9900 Program Counter
//   r1  (wp)  = TMS9900 Workspace Pointer
//   r2  (st)  = TMS9900 Status Register
//   r3  (r0)  = TMS9900 workspace R0
//   r4  (r1)  = TMS9900 workspace R1
//   ...
//   r15 (r12) = TMS9900 workspace R12
//
// TMS9900 R13-R15 are accessible via memory: x/3xh <WP+26>
//----------------------------------------------------------------------------

#define NUM_REGS 16

static UINT16 read_reg(int regnum)
{
    if (regnum == 0) return ProgramCounter;
    if (regnum == 1) return WorkspacePtr;
    if (regnum == 2) return Status;
    if (regnum >= 3 && regnum <= 15)
    {
        int wreg = regnum - 3;
        UINT16 addr = (WorkspacePtr + wreg * 2) & 0xFFFE;
        return (Memory[addr] << 8) | Memory[addr + 1];
    }
    return 0;
}

static void write_reg(int regnum, UINT16 val)
{
    if (regnum == 0) SetProgramCounter(val);
    else if (regnum == 1) SetWorkspacePointer(val);
    else if (regnum == 2) Status = val;
    else if (regnum >= 3 && regnum <= 15)
    {
        int wreg = regnum - 3;
        UINT16 addr = (WorkspacePtr + wreg * 2) & 0xFFFE;
        Memory[addr] = (val >> 8) & 0xFF;
        Memory[addr + 1] = val & 0xFF;
    }
}

//----------------------------------------------------------------------------
// GDB main loop
//----------------------------------------------------------------------------

int gdb_main(int port, std::vector<PeriodicTimer>& timers)
{
    signal(SIGPIPE, SIG_IGN);

    // Create listening socket
    int listenFd = socket(AF_INET, SOCK_STREAM, 0);
    if (listenFd < 0)
    {
        perror("gdb: socket");
        return 1;
    }

    int opt = 1;
    setsockopt(listenFd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);

    if (bind(listenFd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        perror("gdb: bind");
        close(listenFd);
        return 1;
    }

    if (listen(listenFd, 1) < 0)
    {
        perror("gdb: listen");
        close(listenFd);
        return 1;
    }

    fprintf(stderr, "GDB stub listening on localhost:%d\n", port);
    fprintf(stderr, "Connect with: target remote localhost:%d\n", port);

    int clientFd = accept(listenFd, nullptr, nullptr);
    if (clientFd < 0)
    {
        perror("gdb: accept");
        close(listenFd);
        return 1;
    }
    close(listenFd);
    fprintf(stderr, "GDB connected.\n");

    std::set<UINT16> breakpoints;
    bool running = true;

    while (running)
    {
        ReceivedPacket received = recv_packet(clientFd);
        if (received.status == ReceiveStatus::Disconnected)
        {
            fprintf(stderr, "GDB disconnected.\n");
            break;
        }
        std::string& pkt = received.data;

        std::string reply;

        //--- Async break (Ctrl-C) ---
        if (received.status == ReceiveStatus::Break)
        {
            reply = "S05";
        }
        else if (pkt.empty())
        {
            reply = "";
        }
        //--- Halt reason ---
        else if (pkt[0] == '?')
        {
            reply = "S05";
        }
        //--- Read all registers ---
        else if (pkt[0] == 'g')
        {
            char buf[NUM_REGS * 8 + 1];
            for (int i = 0; i < NUM_REGS; i++)
                hex_reg(buf + i * 8, read_reg(i));
            buf[NUM_REGS * 8] = '\0';
            reply = buf;
        }
        //--- Write all registers ---
        else if (pkt[0] == 'G')
        {
            std::array<UINT16, NUM_REGS> values{};
            bool valid = pkt.size() == 1 + NUM_REGS * 8;
            for (int i = 0; valid && i < NUM_REGS; i++)
            {
                valid = parse_register_value(pkt.data() + 1 + i * 8, 8,
                                             values[i]);
            }
            if (valid)
            {
                for (int i = 0; i < NUM_REGS; i++)
                {
                    write_reg(i, values[i]);
                }
                reply = "OK";
            }
            else
            {
                reply = "E01";
            }
        }
        //--- Read single register ---
        else if (pkt[0] == 'p')
        {
            unsigned long parsedReg = 0;
            const bool valid = parse_hex_range(pkt.data() + 1,
                                               pkt.data() + pkt.size(),
                                               parsedReg);
            if (valid && parsedReg < NUM_REGS)
            {
                char buf[9];
                hex_reg(buf, read_reg(static_cast<int>(parsedReg)));
                buf[8] = '\0';
                reply = buf;
            }
            else
            {
                reply = "E00";
            }
        }
        //--- Write single register ---
        else if (pkt[0] == 'P')
        {
            const char* eq = strchr(pkt.c_str(), '=');
            unsigned long parsedReg = 0;
            UINT16 value = 0;
            const bool valid = eq != nullptr && eq > pkt.data() + 1 &&
                parse_hex_range(pkt.data() + 1, eq, parsedReg) &&
                parsedReg < NUM_REGS &&
                parse_register_value(eq + 1,
                                     pkt.data() + pkt.size() - (eq + 1),
                                     value);
            if (valid)
            {
                write_reg(static_cast<int>(parsedReg), value);
                reply = "OK";
            }
            else
            {
                reply = "E01";
            }
        }
        //--- Read memory ---
        else if (pkt[0] == 'm')
        {
            const char* comma = strchr(pkt.c_str(), ',');
            unsigned long address = 0;
            unsigned long length = 0;
            const bool valid = comma != nullptr && comma > pkt.data() + 1 &&
                parse_hex_range(pkt.data() + 1, comma, address) &&
                parse_hex_range(comma + 1, pkt.data() + pkt.size(), length) &&
                address <= 0xFFFF && length <= MaxPacketSize / 2 &&
                address + length <= 0x10000;
            if (valid)
            {
                std::string hex;
                hex.reserve(length * 2);
                char hb[2];
                for (unsigned long i = 0; i < length; i++)
                {
                    hex_byte(hb, Memory[address + i]);
                    hex += hb[0];
                    hex += hb[1];
                }
                reply = hex;
            }
            else
            {
                reply = "E01";
            }
        }
        //--- Write memory ---
        else if (pkt[0] == 'M')
        {
            const char* comma = strchr(pkt.c_str(), ',');
            const char* colon = strchr(pkt.c_str(), ':');
            unsigned long address = 0;
            unsigned long length = 0;
            bool valid = comma != nullptr && colon != nullptr &&
                comma > pkt.data() + 1 && comma < colon &&
                parse_hex_range(pkt.data() + 1, comma, address) &&
                parse_hex_range(comma + 1, colon, length) &&
                address <= 0xFFFF && length <= 0x10000 &&
                address + length <= 0x10000 &&
                static_cast<size_t>(pkt.data() + pkt.size() - (colon + 1)) ==
                    length * 2;
            std::vector<UINT8> bytes;
            if (valid)
            {
                bytes.reserve(length);
                const char* hexdata = colon + 1;
                for (unsigned long i = 0; i < length; i++)
                {
                    const int high = hex_val(hexdata[0]);
                    const int low = hex_val(hexdata[1]);
                    if (high < 0 || low < 0)
                    {
                        valid = false;
                        break;
                    }
                    bytes.push_back(static_cast<UINT8>((high << 4) | low));
                    hexdata += 2;
                }
            }
            if (valid)
            {
                for (unsigned long i = 0; i < length; i++)
                {
                    Memory[address + i] = bytes[i];
                }
                reply = "OK";
            }
            else
            {
                reply = "E01";
            }
        }
        //--- Single step ---
        else if (pkt[0] == 's')
        {
            if (!set_optional_program_counter(pkt))
            {
                reply = "E01";
            }
            else
            {
                step_instruction(timers);
                reply = "S05";
            }
        }
        //--- Continue ---
        else if (pkt[0] == 'c')
        {
            if (!set_optional_program_counter(pkt))
            {
                reply = "E01";
            }
            else
            {
                // Step once past current PC if it's a breakpoint.
                if (breakpoints.count(ProgramCounter))
                {
                    step_instruction(timers);
                }

                int pollCounter = 0;
                for (;;)
                {
                    fire_timers(timers);
                    advance_idle_to_timer(timers);
                    const CPUStepResult result = StepCPU();

                    if (breakpoints.count(ProgramCounter))
                    {
                        reply = "S05";
                        break;
                    }

                    if (result.event == CPUEvent::Idle ||
                        (result.halted && timers.empty()))
                    {
                        reply = "S05";
                        break;
                    }

                    if (++pollCounter >= 1024)
                    {
                        pollCounter = 0;
                        struct pollfd pfd;
                        pfd.fd = clientFd;
                        pfd.events = POLLIN;
                        if (poll(&pfd, 1, 0) > 0)
                        {
                            char brk;
                            if (read(clientFd, &brk, 1) == 1 && brk == 0x03)
                            {
                                reply = "S02";
                                break;
                            }
                        }
                    }
                }
            }
        }
        //--- Set/clear software breakpoint ---
        else if ((pkt[0] == 'Z' || pkt[0] == 'z') &&
                 pkt.size() >= 3 && pkt[1] == '0' && pkt[2] == ',')
        {
            const char* secondComma = strchr(pkt.c_str() + 3, ',');
            unsigned long address = 0;
            unsigned long kind = 0;
            const bool valid = secondComma != nullptr &&
                parse_hex_range(pkt.data() + 3, secondComma, address) &&
                parse_hex_range(secondComma + 1,
                                pkt.data() + pkt.size(), kind) &&
                address <= 0xFFFF;
            if (!valid)
            {
                reply = "E01";
            }
            else if (pkt[0] == 'Z')
            {
                breakpoints.insert(static_cast<UINT16>(address & 0xFFFE));
                reply = "OK";
            }
            else
            {
                breakpoints.erase(static_cast<UINT16>(address & 0xFFFE));
                reply = "OK";
            }
        }
        //--- qSupported ---
        else if (pkt.compare(0, 10, "qSupported") == 0)
        {
            reply = "PacketSize=4096;qXfer:features:read+";
        }
        //--- Target description XML ---
        else if (pkt.compare(0, 31, "qXfer:features:read:target.xml:") == 0)
        {
            const char* args = pkt.c_str() + 31;
            const char* comma = strchr(args, ',');
            unsigned long offset = 0;
            unsigned long length = 0;
            const bool valid = comma != nullptr && comma > args &&
                parse_hex_range(args, comma, offset) &&
                parse_hex_range(comma + 1, pkt.data() + pkt.size(), length);
            if (valid)
            {
                size_t xmlLen = strlen(targetXML);

                if (offset >= xmlLen)
                {
                    reply = "l";
                }
                else
                {
                    size_t remaining = xmlLen - offset;
                    size_t requested = std::min<size_t>(length,
                                                        MaxPacketSize - 1);
                    size_t chunk = std::min(remaining, requested);
                    bool last = (offset + chunk >= xmlLen);
                    reply = last ? "l" : "m";
                    reply += std::string(targetXML + offset, chunk);
                }
            }
            else
            {
                reply = "E00";
            }
        }
        //--- Thread queries (single-thread stubs) ---
        else if (pkt == "qfThreadInfo")
        {
            reply = "m1";
        }
        else if (pkt == "qsThreadInfo")
        {
            reply = "l";
        }
        else if (pkt == "qC")
        {
            reply = "QC1";
        }
        else if (pkt == "qAttached")
        {
            reply = "1";
        }
        //--- Thread selection (always OK) ---
        else if (pkt[0] == 'H')
        {
            reply = "OK";
        }
        //--- Kill ---
        else if (pkt[0] == 'k')
        {
            fprintf(stderr, "GDB kill request.\n");
            close(clientFd);
            return 0;
        }
        //--- Detach ---
        else if (pkt[0] == 'D')
        {
            fprintf(stderr, "GDB detached.\n");
            (void)send_packet(clientFd, "OK");
            close(clientFd);
            return 0;
        }
        //--- Unsupported packet ---
        else
        {
            reply = "";
        }

        if (!send_packet(clientFd, reply))
        {
            fprintf(stderr, "GDB disconnected while sending reply.\n");
            break;
        }
    }

    close(clientFd);
    return 0;
}
