//----------------------------------------------------------------------------
// TMS9900 Trace - GDB Remote Serial Protocol stub
//
// gdb_stub.cpp - GDB RSP server for interactive debugging
//
// Implements a subset of the GDB Remote Serial Protocol over TCP.
// Supports: register read/write, memory read/write, breakpoints,
// single-step, continue, target description XML.
//
// Register model (19 registers, all 16-bit):
//   0-15: R0-R15 (workspace registers, read from Memory[WP+reg*2])
//   16:   PC (Program Counter)
//   17:   WP (Workspace Pointer)
//   18:   ST (Status Register)
//----------------------------------------------------------------------------

#include "gdb_stub.hpp"
#include "cpu.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
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

static const char* targetXML =
    "<?xml version=\"1.0\"?>\n"
    "<!DOCTYPE target SYSTEM \"gdb-target.dtd\">\n"
    "<target version=\"1.0\">\n"
    "  <architecture>tms9900</architecture>\n"
    "  <feature name=\"org.ti.tms9900.core\">\n"
    "    <reg name=\"r0\"  bitsize=\"16\" regnum=\"0\"/>\n"
    "    <reg name=\"r1\"  bitsize=\"16\" regnum=\"1\"/>\n"
    "    <reg name=\"r2\"  bitsize=\"16\" regnum=\"2\"/>\n"
    "    <reg name=\"r3\"  bitsize=\"16\" regnum=\"3\"/>\n"
    "    <reg name=\"r4\"  bitsize=\"16\" regnum=\"4\"/>\n"
    "    <reg name=\"r5\"  bitsize=\"16\" regnum=\"5\"/>\n"
    "    <reg name=\"r6\"  bitsize=\"16\" regnum=\"6\"/>\n"
    "    <reg name=\"r7\"  bitsize=\"16\" regnum=\"7\"/>\n"
    "    <reg name=\"r8\"  bitsize=\"16\" regnum=\"8\"/>\n"
    "    <reg name=\"r9\"  bitsize=\"16\" regnum=\"9\"/>\n"
    "    <reg name=\"r10\" bitsize=\"16\" regnum=\"10\"/>\n"
    "    <reg name=\"r11\" bitsize=\"16\" regnum=\"11\"/>\n"
    "    <reg name=\"r12\" bitsize=\"16\" regnum=\"12\"/>\n"
    "    <reg name=\"r13\" bitsize=\"16\" regnum=\"13\"/>\n"
    "    <reg name=\"r14\" bitsize=\"16\" regnum=\"14\"/>\n"
    "    <reg name=\"r15\" bitsize=\"16\" regnum=\"15\"/>\n"
    "    <reg name=\"pc\"  bitsize=\"16\" type=\"code_ptr\" regnum=\"16\"/>\n"
    "    <reg name=\"wp\"  bitsize=\"16\" type=\"data_ptr\" regnum=\"17\"/>\n"
    "    <reg name=\"st\"  bitsize=\"16\" regnum=\"18\"/>\n"
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

static void hex_word(char* buf, UINT16 val)
{
    hex_byte(buf, (val >> 8) & 0xFF);
    hex_byte(buf + 2, val & 0xFF);
}

static unsigned long parse_hex(const char* s, const char** endp)
{
    unsigned long val = 0;
    while (*s)
    {
        int d = hex_val(*s);
        if (d < 0) break;
        val = (val << 4) | d;
        s++;
    }
    if (endp) *endp = s;
    return val;
}

//----------------------------------------------------------------------------
// RSP packet I/O
//----------------------------------------------------------------------------

static bool send_packet(int fd, const std::string& data)
{
    UINT8 cksum = 0;
    for (char c : data)
        cksum += (UINT8)c;

    char hdr = '$';
    char trailer[3];
    trailer[0] = '#';
    hex_byte(trailer + 1, cksum);

    if (write(fd, &hdr, 1) != 1) return false;
    if (!data.empty())
    {
        ssize_t n = write(fd, data.c_str(), data.size());
        if (n != (ssize_t)data.size()) return false;
    }
    if (write(fd, trailer, 3) != 3) return false;

    // Wait for ACK
    char ack;
    if (read(fd, &ack, 1) != 1) return false;
    return (ack == '+');
}

static std::string recv_packet(int fd)
{
    char c;
    for (;;)
    {
        if (read(fd, &c, 1) != 1) return "";
        if (c == '$') break;
        if (c == 0x03)
        {
            // Ctrl-C async break
            return "\x03";
        }
    }

    std::string data;
    for (;;)
    {
        if (read(fd, &c, 1) != 1) return "";
        if (c == '#') break;
        data += c;
    }

    // Consume 2-char checksum (we trust it)
    char ckbuf[2];
    if (read(fd, ckbuf, 2) != 2) return "";

    // Send ACK
    char ack = '+';
    (void)write(fd, &ack, 1);

    return data;
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

//----------------------------------------------------------------------------
// Register access (no clock side effects)
//----------------------------------------------------------------------------

static UINT16 read_reg(int regnum)
{
    if (regnum >= 0 && regnum <= 15)
    {
        UINT16 addr = (WorkspacePtr + regnum * 2) & 0xFFFE;
        return (Memory[addr] << 8) | Memory[addr + 1];
    }
    if (regnum == 16) return ProgramCounter;
    if (regnum == 17) return WorkspacePtr;
    if (regnum == 18) return Status;
    return 0;
}

static void write_reg(int regnum, UINT16 val)
{
    if (regnum >= 0 && regnum <= 15)
    {
        UINT16 addr = (WorkspacePtr + regnum * 2) & 0xFFFE;
        Memory[addr] = (val >> 8) & 0xFF;
        Memory[addr + 1] = val & 0xFF;
    }
    else if (regnum == 16) ProgramCounter = val;
    else if (regnum == 17) WorkspacePtr = val;
    else if (regnum == 18) Status = val;
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
        std::string pkt = recv_packet(clientFd);
        if (pkt.empty())
        {
            fprintf(stderr, "GDB disconnected.\n");
            break;
        }

        std::string reply;

        //--- Async break (Ctrl-C) ---
        if (pkt[0] == 0x03)
        {
            reply = "S05";
        }
        //--- Halt reason ---
        else if (pkt[0] == '?')
        {
            reply = "S05";
        }
        //--- Read all registers ---
        else if (pkt[0] == 'g')
        {
            char buf[19 * 4 + 1];
            for (int i = 0; i < 19; i++)
                hex_word(buf + i * 4, read_reg(i));
            buf[19 * 4] = '\0';
            reply = buf;
        }
        //--- Write all registers ---
        else if (pkt[0] == 'G')
        {
            const char* p = pkt.c_str() + 1;
            for (int i = 0; i < 19 && p[0] && p[1] && p[2] && p[3]; i++)
            {
                int h0 = hex_val(p[0]), h1 = hex_val(p[1]);
                int h2 = hex_val(p[2]), h3 = hex_val(p[3]);
                if (h0 < 0 || h1 < 0 || h2 < 0 || h3 < 0) break;
                write_reg(i, (h0 << 12) | (h1 << 8) | (h2 << 4) | h3);
                p += 4;
            }
            reply = "OK";
        }
        //--- Read single register ---
        else if (pkt[0] == 'p')
        {
            int regnum = (int)parse_hex(pkt.c_str() + 1, nullptr);
            if (regnum >= 0 && regnum <= 18)
            {
                char buf[5];
                hex_word(buf, read_reg(regnum));
                buf[4] = '\0';
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
            if (eq)
            {
                int regnum = (int)parse_hex(pkt.c_str() + 1, nullptr);
                UINT16 val = (UINT16)parse_hex(eq + 1, nullptr);
                if (regnum >= 0 && regnum <= 18)
                {
                    write_reg(regnum, val);
                    reply = "OK";
                }
                else
                {
                    reply = "E00";
                }
            }
            else
            {
                reply = "E00";
            }
        }
        //--- Read memory ---
        else if (pkt[0] == 'm')
        {
            const char* comma = strchr(pkt.c_str(), ',');
            if (comma)
            {
                UINT16 maddr = (UINT16)parse_hex(pkt.c_str() + 1, nullptr);
                unsigned long len = parse_hex(comma + 1, nullptr);
                std::string hex;
                hex.reserve(len * 2);
                char hb[2];
                for (unsigned long i = 0; i < len; i++)
                {
                    hex_byte(hb, Memory[(maddr + i) & 0xFFFF]);
                    hex += hb[0];
                    hex += hb[1];
                }
                reply = hex;
            }
            else
            {
                reply = "E00";
            }
        }
        //--- Write memory ---
        else if (pkt[0] == 'M')
        {
            const char* comma = strchr(pkt.c_str(), ',');
            const char* colon = strchr(pkt.c_str(), ':');
            if (comma && colon)
            {
                UINT16 maddr = (UINT16)parse_hex(pkt.c_str() + 1, nullptr);
                unsigned long len = parse_hex(comma + 1, nullptr);
                const char* hexdata = colon + 1;
                for (unsigned long i = 0; i < len && hexdata[0] && hexdata[1]; i++)
                {
                    int h0 = hex_val(hexdata[0]);
                    int h1 = hex_val(hexdata[1]);
                    if (h0 < 0 || h1 < 0) break;
                    Memory[(maddr + i) & 0xFFFF] = (h0 << 4) | h1;
                    hexdata += 2;
                }
                reply = "OK";
            }
            else
            {
                reply = "E00";
            }
        }
        //--- Single step ---
        else if (pkt[0] == 's')
        {
            if (pkt.size() > 1)
                ProgramCounter = (UINT16)parse_hex(pkt.c_str() + 1, nullptr);

            fire_timers(timers);
            UINT32 prevInstr = InstructionCounter;
            bool idle = false;
            do {
                idle = StepCPU();
                fire_timers(timers);
            } while (InstructionCounter == prevInstr && !idle);

            reply = "S05";
        }
        //--- Continue ---
        else if (pkt[0] == 'c')
        {
            if (pkt.size() > 1)
                ProgramCounter = (UINT16)parse_hex(pkt.c_str() + 1, nullptr);

            // Step once past current PC if it's a breakpoint
            if (breakpoints.count(ProgramCounter))
            {
                fire_timers(timers);
                StepCPU();
            }

            int pollCounter = 0;
            for (;;)
            {
                fire_timers(timers);
                bool idle = StepCPU();

                if (breakpoints.count(ProgramCounter))
                {
                    reply = "S05";
                    break;
                }

                if (idle && timers.empty())
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
        //--- Set/clear software breakpoint ---
        else if ((pkt[0] == 'Z' || pkt[0] == 'z') &&
                 pkt.size() >= 3 && pkt[1] == '0' && pkt[2] == ',')
        {
            UINT16 bpAddr = (UINT16)parse_hex(pkt.c_str() + 3, nullptr);
            if (pkt[0] == 'Z')
                breakpoints.insert(bpAddr);
            else
                breakpoints.erase(bpAddr);
            reply = "OK";
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
            if (comma)
            {
                unsigned long offset = parse_hex(args, nullptr);
                unsigned long length = parse_hex(comma + 1, nullptr);
                size_t xmlLen = strlen(targetXML);

                if (offset >= xmlLen)
                {
                    reply = "l";
                }
                else
                {
                    size_t remaining = xmlLen - offset;
                    size_t chunk = remaining < length ? remaining : length;
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
            send_packet(clientFd, "OK");
            close(clientFd);
            return 0;
        }
        //--- Unsupported packet ---
        else
        {
            reply = "";
        }

        send_packet(clientFd, reply);
    }

    close(clientFd);
    return 0;
}
