//----------------------------------------------------------------------------
// TMS9900 Trace - Standalone TMS9900 CPU Simulator
//
// main.cpp - CLI entry point and trace loop
//
// Copyright (c) 2024
// Licensed under GPL v2 (due to ti99sim heritage)
//----------------------------------------------------------------------------

#include "cpu.hpp"
#include "disasm.hpp"
#include "gdb_stub.hpp"
#include <array>
#include <cctype>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <getopt.h>
#include <vector>
#include <algorithm>

//----------------------------------------------------------------------------
// Scheduled interrupt
//----------------------------------------------------------------------------

struct ScheduledIRQ
{
    int level;
    UINT64 atStep;
};

//----------------------------------------------------------------------------
// Configuration
//----------------------------------------------------------------------------

struct MemoryDump
{
    UINT16 start;
    UINT32 length;
};

struct Tracepoint
{
    UINT16 pc;
    UINT64 hits = 0;
};

struct BinaryLoadSpec
{
    UINT8 base = 0;
    UINT16 addr = 0;
    std::string path;
};

struct Config
{
    const char* inputFile = nullptr;
    const char* outputFile = nullptr;
    UINT16 loadAddr = 0x0000;
    UINT16 entryAddr = 0x0000;
    UINT16 wpAddr = 0x8300;
    UINT64 maxSteps = 1000000;
    std::vector<UINT16> stopAddrs;
    std::vector<ScheduledIRQ> irqs;
    std::vector<PeriodicTimer> timers;
    std::vector<MemoryDump> dumps;
    std::vector<MemoryDump> vramDumps;
    std::vector<BinaryLoadSpec> gromLoads;
    std::vector<BinaryLoadSpec> vramLoads;
    std::vector<BinaryLoadSpec> samsLoads;
    std::vector<BinaryLoadSpec> cartBankLoads;
    std::vector<BinaryLoadSpec> memLoads;
    std::vector<Tracepoint> tracepoints;
    UINT64 tracepointMax = 0;
    UINT32 waitStates = 0;
    bool ti99IO = false;
    bool tracepointStop = false;
    bool loopDetect = true;
    int gdbPort = 0;
    bool quiet = false;
    bool summary = false;
    bool entrySet = false;
};

//----------------------------------------------------------------------------
// Usage
//----------------------------------------------------------------------------

static void PrintUsage(const char* prog)
{
    fprintf(stderr, "TMS9900 Trace - Standalone TMS9900 CPU Simulator\n\n");
    fprintf(stderr, "Usage: %s [options] <binary.bin>\n\n", prog);
    fprintf(stderr, "Memory Options:\n");
    fprintf(stderr, "  -l, --load=ADDR       Load address (hex, default: 0x0000)\n");
    fprintf(stderr, "  -e, --entry=ADDR      Entry point (hex, default: same as load)\n");
    fprintf(stderr, "  -w, --wp=ADDR         Workspace pointer (hex, default: 0x8300)\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "Execution Options:\n");
    fprintf(stderr, "  -n, --max-steps=N     Max instructions (default: 1000000)\n");
    fprintf(stderr, "  -s, --stop-at=ADDR    Stop at address (hex, can repeat)\n");
    fprintf(stderr, "  --irq=LEVEL@STEP      Trigger interrupt at step (can repeat)\n");
    fprintf(stderr, "  --timer=LEVEL:PERIOD  Periodic interrupt every PERIOD cycles (can repeat)\n");
    fprintf(stderr, "  --wait-states=N       Add N clocks per modeled memory access\n");
    fprintf(stderr, "  --no-loop-detect      Do not stop on a repeated instruction PC\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "Output Options:\n");
    fprintf(stderr, "  -o, --output=FILE     Output file (default: stdout)\n");
    fprintf(stderr, "  -q, --quiet           Only output trace, no status messages\n");
    fprintf(stderr, "  -S, --summary         Output only final state as JSON (no per-step trace)\n");
    fprintf(stderr, "  -d, --dump=START:LEN  Dump memory range at exit (hex, can repeat)\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "Tracepoint Options (require -S):\n");
    fprintf(stderr, "  -t, --tracepoint=ADDR     Trace only this address (hex, can repeat)\n");
    fprintf(stderr, "  -T, --tracepoint-file=FILE  Load tracepoint addresses from file\n");
    fprintf(stderr, "  --tracepoint-max=N        Stop after N total tracepoint hits\n");
    fprintf(stderr, "  --tracepoint-stop         Stop when all tracepoints hit at least once\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "GDB Debugging:\n");
    fprintf(stderr, "  --gdb=PORT            Start GDB RSP server on PORT (e.g., --gdb=1234)\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "Other:\n");
    fprintf(stderr, "  -h, --help            Show this help\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "Output: NDJSON trace of each instruction:\n");
    fprintf(stderr, "  {\"step\":N,\"pc\":\"XXXX\",\"wp\":\"XXXX\",\"st\":\"XXXX\",\"clk\":N,\n");
    fprintf(stderr, "   \"op\":\"XXX\",\"asm\":\"...\",\"r\":[\"XXXX\",...,\"XXXX\"]}\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "Examples:\n");
    fprintf(stderr, "  %s test.bin\n", prog);
    fprintf(stderr, "  %s -l 0x6000 -e 0x6000 program.bin\n", prog);
    fprintf(stderr, "  %s -n 100 -o trace.ndjson test.bin\n", prog);
    fprintf(stderr, "  %s --irq=1@500 -s 0x6100 program.bin\n", prog);
    fprintf(stderr, "  %s --timer=1:50000 -n 500000 program.bin\n", prog);
}

//----------------------------------------------------------------------------
// Parse hex value (requires 0x prefix)
//----------------------------------------------------------------------------

static bool ParseHex(const char* str, UINT16* value)
{
    // Require 0x or 0X prefix
    if (str[0] != '0' || (str[1] != 'x' && str[1] != 'X'))
    {
        return false;
    }

    char* end;
    unsigned long v = strtoul(str, &end, 16);
    if (*end != '\0' || v > 0xFFFF)
    {
        return false;
    }
    *value = (UINT16)v;
    return true;
}

//----------------------------------------------------------------------------
// Parse interrupt spec: LEVEL@STEP (e.g., "1@500")
//----------------------------------------------------------------------------

static bool ParseIRQ(const char* str, ScheduledIRQ* irq)
{
    char* at = (char*)strchr(str, '@');
    if (!at)
    {
        return false;
    }

    *at = '\0';
    int level = atoi(str);
    *at = '@';

    if (level < 0 || level > 15)
    {
        return false;
    }

    char* end;
    UINT64 step = strtoull(at + 1, &end, 10);
    if (*end != '\0')
    {
        return false;
    }

    irq->level = level;
    irq->atStep = step;
    return true;
}

//----------------------------------------------------------------------------
// Parse periodic timer spec: LEVEL:PERIOD (e.g., "1:50000")
//----------------------------------------------------------------------------

static bool ParseTimer(const char* str, PeriodicTimer* timer)
{
    char* colon = (char*)strchr(str, ':');
    if (!colon)
    {
        return false;
    }

    *colon = '\0';
    int level = atoi(str);
    *colon = ':';

    if (level < 0 || level > 15)
    {
        return false;
    }

    char* end;
    UINT64 period = strtoull(colon + 1, &end, 10);
    if (*end != '\0' || period == 0)
    {
        return false;
    }

    timer->irqLevel = level;
    timer->periodCycles = period;
    timer->nextTriggerCycle = period;  // First trigger after one period
    return true;
}

//----------------------------------------------------------------------------
// Add tracepoint (with deduplication)
//----------------------------------------------------------------------------

static void AddTracepoint(std::vector<Tracepoint>& tracepoints, UINT16 addr)
{
    for (const auto& tp : tracepoints)
    {
        if (tp.pc == addr) return;  // Already exists
    }
    Tracepoint tp;
    tp.pc = addr;
    tracepoints.push_back(tp);
}

//----------------------------------------------------------------------------
// Parse tracepoint file (one hex address per line, # comments, blank lines ok)
//----------------------------------------------------------------------------

static bool ParseTracepointFile(const char* path, std::vector<Tracepoint>& tracepoints)
{
    FILE* f = fopen(path, "r");
    if (!f) return false;

    char line[256];
    while (fgets(line, sizeof(line), f))
    {
        // Skip leading whitespace
        char* p = line;
        while (*p && isspace(*p)) p++;

        // Skip comments and blank lines
        if (*p == '#' || *p == '\0' || *p == '\n') continue;

        // Parse hex address (with or without 0x prefix)
        char* end;
        unsigned long val;
        if (strncmp(p, "0x", 2) == 0 || strncmp(p, "0X", 2) == 0)
        {
            val = strtoul(p + 2, &end, 16);
        }
        else
        {
            val = strtoul(p, &end, 16);
        }

        if (val > 0xFFFF)
        {
            fclose(f);
            return false;
        }

        AddTracepoint(tracepoints, (UINT16)val);
    }
    fclose(f);
    return true;
}

//----------------------------------------------------------------------------
// Parse memory dump spec: START:LENGTH (e.g., "0x8300:32")
//----------------------------------------------------------------------------

static bool ParseDump(const char* str, MemoryDump* dump)
{
    char* colon = (char*)strchr(str, ':');
    if (!colon)
    {
        return false;
    }

    // Temporarily terminate at colon to parse start address
    *colon = '\0';
    UINT16 start;
    bool ok = ParseHex(str, &start);
    *colon = ':';

    if (!ok)
    {
        return false;
    }

    // Parse length (decimal or hex)
    char* end;
    unsigned long length;
    if (colon[1] == '0' && (colon[2] == 'x' || colon[2] == 'X'))
    {
        length = strtoul(colon + 1, &end, 16);
    }
    else
    {
        length = strtoul(colon + 1, &end, 10);
    }

    if (*end != '\0' || length == 0 || length > 0x10000 ||
        static_cast<unsigned long>(start) + length > 0x10000)
    {
        return false;
    }

    dump->start = start;
    dump->length = static_cast<UINT32>(length);
    return true;
}

//----------------------------------------------------------------------------
// Load binary file
//----------------------------------------------------------------------------

static bool LoadBinary(const char* filename, UINT16 loadAddr)
{
    FILE* f = fopen(filename, "rb");
    if (!f)
    {
        fprintf(stderr, "Error: Cannot open file '%s'\n", filename);
        return false;
    }

    // Get file size
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size < 0)
    {
        fprintf(stderr, "Error: Cannot determine file size\n");
        fclose(f);
        return false;
    }

    // Check if it fits
    if (loadAddr + size > 0x10000)
    {
        fprintf(stderr, "Error: File too large to load at 0x%04X (%ld bytes)\n", loadAddr, size);
        fclose(f);
        return false;
    }

    // Load into memory
    size_t read = fread(&Memory[loadAddr], 1, size, f);
    fclose(f);

    if ((long)read != size)
    {
        fprintf(stderr, "Error: Short read (%zu of %ld bytes)\n", read, size);
        return false;
    }

    return true;
}

//----------------------------------------------------------------------------
// Trace output state (for extensibility)
//----------------------------------------------------------------------------

struct TraceState
{
    UINT64 step;
    UINT16 pc;
    UINT16 wp;
    UINT16 st;
    UINT64 clocks;
    const char* mnemonic;
    const char* disasm;
    std::array<UINT16, 16> registers;
};

static std::array<UINT16, 16> CaptureRegisters(UINT16 wp)
{
    std::array<UINT16, 16> registers{};
    for (int i = 0; i < 16; i++)
    {
        UINT16 addr = static_cast<UINT16>((wp + i * 2) & 0xFFFE);
        registers[i] = static_cast<UINT16>((Memory[addr] << 8) |
                                           Memory[addr + 1]);
    }
    return registers;
}

//----------------------------------------------------------------------------
// Output JSON trace line (all hex for values)
//----------------------------------------------------------------------------

static void OutputTrace(FILE* out, const TraceState& t)
{
    fprintf(out, "{\"step\":%" PRIu64 ",\"pc\":\"%04X\",\"wp\":\"%04X\",\"st\":\"%04X\",\"clk\":%" PRIu64 ",",
            t.step, t.pc, t.wp, t.st, t.clocks);
    fprintf(out, "\"op\":\"%s\",\"asm\":\"%s\",\"r\":[", t.mnemonic, t.disasm);

    // Registers are captured before execution with the rest of the trace.
    for (int i = 0; i < 16; i++)
    {
        if (i > 0) fprintf(out, ",");
        fprintf(out, "\"%04X\"", t.registers[i]);
    }

    fprintf(out, "]}\n");
}

//----------------------------------------------------------------------------
// Check if PC matches any stop address
//----------------------------------------------------------------------------

static bool IsStopAddress(UINT16 pc, const std::vector<UINT16>& stopAddrs)
{
    return std::find(stopAddrs.begin(), stopAddrs.end(), pc) != stopAddrs.end();
}

//----------------------------------------------------------------------------
// Main
//----------------------------------------------------------------------------

int main(int argc, char* argv[])
{
    Config cfg;

    enum
    {
        OPT_WAIT_STATES = 256,
        OPT_NO_LOOP_DETECT,
        OPT_TI99_IO,
        OPT_LOAD_GROM,
        OPT_LOAD_VRAM,
        OPT_LOAD_SAMS,
        OPT_LOAD_CART_BANK,
        OPT_LOAD_MEM,
        OPT_DUMP_VRAM
    };

    static struct option longOpts[] = {
        {"load",           required_argument, nullptr, 'l'},
        {"entry",          required_argument, nullptr, 'e'},
        {"wp",             required_argument, nullptr, 'w'},
        {"max-steps",      required_argument, nullptr, 'n'},
        {"stop-at",        required_argument, nullptr, 's'},
        {"irq",            required_argument, nullptr, 'i'},
        {"timer",          required_argument, nullptr, 'R'},
        {"wait-states",    required_argument, nullptr, OPT_WAIT_STATES},
        {"no-loop-detect", no_argument,       nullptr, OPT_NO_LOOP_DETECT},
        {"ti99-io",        no_argument,       nullptr, OPT_TI99_IO},
        {"load-grom",      required_argument, nullptr, OPT_LOAD_GROM},
        {"load-vram",      required_argument, nullptr, OPT_LOAD_VRAM},
        {"load-sams",      required_argument, nullptr, OPT_LOAD_SAMS},
        {"load-cart-bank", required_argument, nullptr, OPT_LOAD_CART_BANK},
        {"load-mem",       required_argument, nullptr, OPT_LOAD_MEM},
        {"dump-vram",      required_argument, nullptr, OPT_DUMP_VRAM},
        {"output",         required_argument, nullptr, 'o'},
        {"dump",           required_argument, nullptr, 'd'},
        {"tracepoint",     required_argument, nullptr, 't'},
        {"tracepoint-file", required_argument, nullptr, 'T'},
        {"tracepoint-max", required_argument, nullptr, 'M'},
        {"tracepoint-stop", no_argument,      nullptr, 'P'},
        {"quiet",          no_argument,       nullptr, 'q'},
        {"summary",        no_argument,       nullptr, 'S'},
        {"gdb",            required_argument, nullptr, 'G'},
        {"help",           no_argument,       nullptr, 'h'},
        {nullptr,          0,                 nullptr, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "l:e:w:n:s:o:d:t:T:M:PqSh", longOpts, nullptr)) != -1)
    {
        switch (opt)
        {
            case 'l':
            {
                if (!ParseHex(optarg, &cfg.loadAddr))
                {
                    fprintf(stderr, "Error: Invalid load address '%s'\n", optarg);
                    return 1;
                }
                break;
            }
            case 'e':
            {
                if (!ParseHex(optarg, &cfg.entryAddr))
                {
                    fprintf(stderr, "Error: Invalid entry address '%s'\n", optarg);
                    return 1;
                }
                cfg.entrySet = true;
                break;
            }
            case 'w':
            {
                if (!ParseHex(optarg, &cfg.wpAddr))
                {
                    fprintf(stderr, "Error: Invalid workspace pointer '%s'\n", optarg);
                    return 1;
                }
                break;
            }
            case 'n':
                cfg.maxSteps = strtoull(optarg, nullptr, 10);
                break;
            case 's':
            {
                UINT16 addr;
                if (!ParseHex(optarg, &addr))
                {
                    fprintf(stderr, "Error: Invalid stop address '%s'\n", optarg);
                    return 1;
                }
                cfg.stopAddrs.push_back(addr);
                break;
            }
            case 'i':
            {
                ScheduledIRQ irq;
                if (!ParseIRQ(optarg, &irq))
                {
                    fprintf(stderr, "Error: Invalid IRQ spec '%s' (use LEVEL@STEP, e.g., 1@500)\n", optarg);
                    return 1;
                }
                cfg.irqs.push_back(irq);
                break;
            }
            case 'R':
            {
                PeriodicTimer timer;
                if (!ParseTimer(optarg, &timer))
                {
                    fprintf(stderr, "Error: Invalid timer spec '%s' (use LEVEL:PERIOD, e.g., 1:50000)\n", optarg);
                    return 1;
                }
                cfg.timers.push_back(timer);
                break;
            }
            case OPT_WAIT_STATES:
            {
                char* end = nullptr;
                unsigned long value = strtoul(optarg, &end, 10);
                if (*optarg == '\0' || *end != '\0' || value > UINT32_MAX)
                {
                    fprintf(stderr, "Error: Invalid wait-state count '%s'\n",
                            optarg);
                    return 1;
                }
                cfg.waitStates = static_cast<UINT32>(value);
                break;
            }
            case OPT_NO_LOOP_DETECT:
                cfg.loopDetect = false;
                break;
            case OPT_TI99_IO:
                cfg.ti99IO = true;
                break;
            case OPT_LOAD_GROM:
            {
                const char* c1 = strchr(optarg, ':');
                if (!c1)
                {
                    fprintf(stderr, "Error: Expected [BASE:]ADDR:FILE in '%s'\n", optarg);
                    return 1;
                }
                const char* c2 = strchr(c1 + 1, ':');
                UINT8 base = 0;
                UINT16 addr = 0;
                std::string filePath;
                if (c2)
                {
                    int b = atoi(std::string(optarg, c1 - optarg).c_str());
                    if (b < 0 || b > 15 || !ParseHex(std::string(c1 + 1, c2 - (c1 + 1)).c_str(), &addr))
                    {
                        fprintf(stderr, "Error: Invalid BASE:ADDR:FILE in '%s'\n", optarg);
                        return 1;
                    }
                    base = static_cast<UINT8>(b);
                    filePath = c2 + 1;
                }
                else
                {
                    if (!ParseHex(std::string(optarg, c1 - optarg).c_str(), &addr))
                    {
                        fprintf(stderr, "Error: Invalid ADDR:FILE in '%s'\n", optarg);
                        return 1;
                    }
                    filePath = c1 + 1;
                }
                cfg.ti99IO = true;
                cfg.gromLoads.push_back({ base, addr, filePath });
                break;
            }
            case OPT_LOAD_VRAM:
            case OPT_LOAD_MEM:
            {
                const char* colon = strchr(optarg, ':');
                if (!colon)
                {
                    fprintf(stderr, "Error: Expected ADDR:FILE in '%s'\n", optarg);
                    return 1;
                }
                std::string addrStr(optarg, colon - optarg);
                UINT16 addr = 0;
                if (!ParseHex(addrStr.c_str(), &addr))
                {
                    fprintf(stderr, "Error: Invalid address in '%s'\n", optarg);
                    return 1;
                }
                if (opt == OPT_LOAD_VRAM)
                {
                    cfg.ti99IO = true;
                    cfg.vramLoads.push_back({ 0, addr, colon + 1 });
                }
                else
                {
                    cfg.memLoads.push_back({ 0, addr, colon + 1 });
                }
                break;
            }
            case OPT_LOAD_SAMS:
            {
                const char* colon = strchr(optarg, ':');
                if (!colon)
                {
                    fprintf(stderr, "Error: Expected PAGE:FILE in '%s'\n", optarg);
                    return 1;
                }
                UINT16 page = 0;
                if (!ParseHex(std::string(optarg, colon - optarg).c_str(), &page) || page > 0xFF)
                {
                    fprintf(stderr, "Error: Invalid SAMS page (0x00..0xFF) in '%s'\n", optarg);
                    return 1;
                }
                cfg.ti99IO = true;
                cfg.samsLoads.push_back({ static_cast<UINT8>(page), 0, colon + 1 });
                break;
            }
            case OPT_LOAD_CART_BANK:
            {
                const char* colon = strchr(optarg, ':');
                if (!colon)
                {
                    fprintf(stderr, "Error: Expected BANK:FILE in '%s'\n", optarg);
                    return 1;
                }
                int b = atoi(std::string(optarg, colon - optarg).c_str());
                if (b < 0 || b > 15)
                {
                    fprintf(stderr, "Error: Invalid cart bank 0..15 in '%s'\n", optarg);
                    return 1;
                }
                cfg.ti99IO = true;
                cfg.cartBankLoads.push_back({ static_cast<UINT8>(b), 0, colon + 1 });
                break;
            }
            case OPT_DUMP_VRAM:
            {
                MemoryDump dump;
                if (!ParseDump(optarg, &dump) ||
                    static_cast<UINT32>(dump.start) + dump.length > 0x4000)
                {
                    fprintf(stderr, "Error: Invalid VRAM dump spec '%s' (max 0x4000)\n", optarg);
                    return 1;
                }
                cfg.ti99IO = true;
                cfg.vramDumps.push_back(dump);
                break;
            }
            case 'o':
                cfg.outputFile = optarg;
                break;
            case 'd':
            {
                MemoryDump dump;
                if (!ParseDump(optarg, &dump))
                {
                    fprintf(stderr, "Error: Invalid dump spec '%s' (use START:LENGTH, e.g., 0x8300:32)\n", optarg);
                    return 1;
                }
                cfg.dumps.push_back(dump);
                break;
            }
            case 't':
            {
                UINT16 addr;
                if (!ParseHex(optarg, &addr))
                {
                    fprintf(stderr, "Error: Invalid tracepoint address '%s'\n", optarg);
                    return 1;
                }
                AddTracepoint(cfg.tracepoints, addr);
                break;
            }
            case 'T':
            {
                if (!ParseTracepointFile(optarg, cfg.tracepoints))
                {
                    fprintf(stderr, "Error: Failed to read tracepoint file '%s'\n", optarg);
                    return 1;
                }
                break;
            }
            case 'M':
                cfg.tracepointMax = strtoull(optarg, nullptr, 10);
                break;
            case 'P':
                cfg.tracepointStop = true;
                break;
            case 'q':
                cfg.quiet = true;
                break;
            case 'S':
                cfg.summary = true;
                break;
            case 'G':
                cfg.gdbPort = atoi(optarg);
                if (cfg.gdbPort <= 0 || cfg.gdbPort > 65535)
                {
                    fprintf(stderr, "Error: Invalid GDB port '%s'\n", optarg);
                    return 1;
                }
                break;
            case 'h':
                PrintUsage(argv[0]);
                return 0;
            default:
                PrintUsage(argv[0]);
                return 1;
        }
    }

    // Get input file
    if (optind >= argc)
    {
        fprintf(stderr, "Error: No input file specified\n\n");
        PrintUsage(argv[0]);
        return 1;
    }
    cfg.inputFile = argv[optind];

    // Default entry to load address
    if (!cfg.entrySet)
    {
        cfg.entryAddr = cfg.loadAddr;
    }

    // Tracepoints require summary mode (otherwise all instructions are traced anyway)
    if (!cfg.tracepoints.empty() && !cfg.summary)
    {
        fprintf(stderr, "Error: --tracepoint requires --summary (-S) mode\n");
        fprintf(stderr, "       (without -S, all instructions are traced anyway)\n");
        return 1;
    }

    // Sort IRQs by step
    std::sort(cfg.irqs.begin(), cfg.irqs.end(),
              [](const ScheduledIRQ& a, const ScheduledIRQ& b) { return a.atStep < b.atStep; });

    // Open output file
    FILE* out = stdout;
    if (cfg.outputFile)
    {
        out = fopen(cfg.outputFile, "w");
        if (!out)
        {
            fprintf(stderr, "Error: Cannot open output file '%s'\n", cfg.outputFile);
            return 1;
        }
    }

    // Initialize CPU
    InitCPU();
    ResetTI99IO();
    SetTI99IOEnabled(cfg.ti99IO);

    for (const auto& gl : cfg.gromLoads)
    {
        FILE* gf = fopen(gl.path.c_str(), "rb");
        if (!gf)
        {
            fprintf(stderr, "Error: Cannot open GROM file '%s'\n", gl.path.c_str());
            if (cfg.outputFile) fclose(out);
            return 1;
        }
        if (gl.base == 0 || gl.addr < 0x6000)
        {
            size_t maxBytes = 0x10000 - gl.addr;
            size_t unused = fread(&GROM_Mem[gl.addr], 1, maxBytes, gf);
            (void)unused;
        }
        else
        {
            size_t maxBytes = 0x10000 - gl.addr;
            size_t unused = fread(&GROM_CartBases[gl.base][gl.addr - 0x6000], 1, maxBytes, gf);
            (void)unused;
        }
        fclose(gf);
    }

    for (const auto& vl : cfg.vramLoads)
    {
        FILE* vf = fopen(vl.path.c_str(), "rb");
        if (!vf || vl.addr >= 0x4000)
        {
            if (vf) fclose(vf);
            fprintf(stderr, "Error: Cannot load VRAM file '%s' at 0x%04X\n", vl.path.c_str(), vl.addr);
            if (cfg.outputFile) fclose(out);
            return 1;
        }
        size_t maxBytes = 0x4000 - vl.addr;
        size_t unused = fread(&VDP_VRAM[vl.addr], 1, maxBytes, vf);
        (void)unused;
        fclose(vf);
    }

    // Load binary and optional extra memory images
    if (!LoadBinary(cfg.inputFile, cfg.loadAddr))
    {
        if (cfg.outputFile) fclose(out);
        return 1;
    }
    for (const auto& ml : cfg.memLoads)
    {
        if (!LoadBinary(ml.path.c_str(), ml.addr))
        {
            if (cfg.outputFile) fclose(out);
            return 1;
        }
    }

    // Sync initial 64K CPU RAM windows into default SAMS pages 0..15, then apply --load-sams
    for (int p = 0; p < 16; p++)
    {
        memcpy(SAMS_Mem[p], &Memory[p << 12], 0x1000);
    }
    for (const auto& sl : cfg.samsLoads)
    {
        FILE* sf = fopen(sl.path.c_str(), "rb");
        if (!sf)
        {
            fprintf(stderr, "Error: Cannot open SAMS page file '%s'\n", sl.path.c_str());
            if (cfg.outputFile) fclose(out);
            return 1;
        }
        size_t unused = fread(SAMS_Mem[sl.base], 1, 0x1000, sf);
        (void)unused;
        fclose(sf);
    }
    for (const auto& cb : cfg.cartBankLoads)
    {
        FILE* cf = fopen(cb.path.c_str(), "rb");
        if (!cf)
        {
            fprintf(stderr, "Error: Cannot open cart bank file '%s'\n", cb.path.c_str());
            if (cfg.outputFile) fclose(out);
            return 1;
        }
        size_t unused = fread(CART_Banks[cb.base], 1, 0x2000, cf);
        (void)unused;
        fclose(cf);
        CART_BankingEnabled = true;
    }

    // Set up initial state
    SetWorkspacePointer(cfg.wpAddr);
    SetProgramCounter(cfg.entryAddr);
    Status = 0;
    ClockCycleCounter = 0;
    InstructionCounter = 0;
    SetMemoryWaitStates(cfg.waitStates);

    // GDB mode: start RSP server instead of batch execution
    if (cfg.gdbPort > 0)
    {
        if (cfg.outputFile) fclose(out);
        return gdb_main(cfg.gdbPort, cfg.timers);
    }

    if (!cfg.quiet && !cfg.summary)
    {
        fprintf(stderr, "TMS9900 Trace\n");
        fprintf(stderr, "  Input:  %s\n", cfg.inputFile);
        fprintf(stderr, "  Load:   0x%04X\n", cfg.loadAddr);
        fprintf(stderr, "  Entry:  0x%04X\n", cfg.entryAddr);
        fprintf(stderr, "  WP:     0x%04X\n", cfg.wpAddr);
        fprintf(stderr, "  Max:    %" PRIu64 " steps\n", cfg.maxSteps);
        fprintf(stderr, "  Wait:   %" PRIu32 " state(s) per memory access\n",
                cfg.waitStates);
        if (!cfg.stopAddrs.empty())
        {
            fprintf(stderr, "  Stop:");
            for (UINT16 addr : cfg.stopAddrs)
            {
                fprintf(stderr, " 0x%04X", addr);
            }
            fprintf(stderr, "\n");
        }
        if (!cfg.irqs.empty())
        {
            fprintf(stderr, "  IRQs:");
            for (const auto& irq : cfg.irqs)
            {
                fprintf(stderr, " %d@%" PRIu64, irq.level, irq.atStep);
            }
            fprintf(stderr, "\n");
        }
        if (!cfg.timers.empty())
        {
            fprintf(stderr, "  Timers:");
            for (const auto& t : cfg.timers)
            {
                fprintf(stderr, " level %d every %" PRIu64 " cycles", t.irqLevel, t.periodCycles);
            }
            fprintf(stderr, "\n");
        }
        if (!cfg.tracepoints.empty())
        {
            fprintf(stderr, "  Tracepoints:");
            for (const auto& tp : cfg.tracepoints)
            {
                fprintf(stderr, " 0x%04X", tp.pc);
            }
            fprintf(stderr, "\n");
            if (cfg.tracepointMax > 0)
            {
                fprintf(stderr, "  Tracepoint max: %" PRIu64 "\n", cfg.tracepointMax);
            }
            if (cfg.tracepointStop)
            {
                fprintf(stderr, "  Tracepoint stop: enabled\n");
            }
        }
        fprintf(stderr, "\n");
    }

    // Main trace loop
    char disasmBuf[80];
    UINT16 lastInstructionPC = 0;
    bool haveLastInstructionPC = false;
    size_t nextIRQ = 0;
    const char* haltReason = "max";
    UINT64 totalTracepointHits = 0;

    while (InstructionCounter < cfg.maxSteps)
    {
        const UINT64 step = InstructionCounter;
        bool interruptRequested = false;

        // Check for scheduled interrupts
        while (nextIRQ < cfg.irqs.size() && cfg.irqs[nextIRQ].atStep <= step)
        {
            TriggerInterrupt(cfg.irqs[nextIRQ].level);
            interruptRequested = true;
            if (!cfg.quiet && !cfg.summary)
            {
                fprintf(stderr, "[Step %" PRIu64 "] Triggered IRQ level %d\n",
                        step, cfg.irqs[nextIRQ].level);
            }
            nextIRQ++;
        }

        // Check for periodic timer interrupts
        for (auto& t : cfg.timers)
        {
            while (ClockCycleCounter >= t.nextTriggerCycle)
            {
                TriggerInterrupt(t.irqLevel);
                t.nextTriggerCycle += t.periodCycles;
                interruptRequested = true;
            }
        }

        // IDLE stops instruction execution but not the external clock. Advance
        // directly to the next timer edge so a periodic interrupt can wake it.
        if (IsHalted() && !interruptRequested && !cfg.timers.empty())
        {
            UINT64 nextTimerCycle = cfg.timers.front().nextTriggerCycle;
            for (const auto& t : cfg.timers)
            {
                nextTimerCycle = std::min(nextTimerCycle,
                                          t.nextTriggerCycle);
            }
            if (ClockCycleCounter < nextTimerCycle)
            {
                ClockCycleCounter = nextTimerCycle;
            }
            for (auto& t : cfg.timers)
            {
                while (ClockCycleCounter >= t.nextTriggerCycle)
                {
                    TriggerInterrupt(t.irqLevel);
                    t.nextTriggerCycle += t.periodCycles;
                    interruptRequested = true;
                }
            }
        }

        // A future instruction-count IRQ cannot wake IDLE because the count no
        // longer advances. Only an interrupt requested at this boundary can.
        if (IsHalted() && !interruptRequested)
        {
            haltReason = "idle";
            break;
        }

        // Capture pre-state
        UINT16 pc = ProgramCounter;
        UINT16 wp = WorkspacePtr;
        UINT16 st = Status;
        UINT64 clocks = ClockCycleCounter;
        const auto registers = CaptureRegisters(wp);

        // Disassemble the candidate instruction. If StepCPU reports interrupt
        // entry, this snapshot is discarded and never emitted as a trace.
        DisassembleASM(pc, disasmBuf, sizeof(disasmBuf));

        // Check stop address
        if (!cfg.stopAddrs.empty() && IsStopAddress(pc, cfg.stopAddrs))
        {
            if (!cfg.quiet && !cfg.summary)
            {
                fprintf(stderr, "Stopped at address 0x%04X\n", pc);
            }
            haltReason = "stop";
            break;
        }

        const CPUStepResult result = StepCPU();
        if (result.event == CPUEvent::Interrupt)
        {
            continue;
        }
        if (result.event == CPUEvent::Idle)
        {
            haltReason = "idle";
            break;
        }

        // Emit tracepoints only for instructions that actually executed.
        for (auto& tp : cfg.tracepoints)
        {
            if (pc != tp.pc)
            {
                continue;
            }

            tp.hits++;
            totalTracepointHits++;

            const char* disasm = strlen(disasmBuf) > 5
                                     ? disasmBuf + 5
                                     : disasmBuf;
            const char* mnemonic = result.opcode
                                       ? result.opcode->mnemonic
                                       : "???";
            TraceState trace = { step, pc, wp, st, clocks, mnemonic,
                                 disasm, registers };
            OutputTrace(out, trace);
            break;
        }

        // Output trace in normal mode.
        if (!cfg.summary)
        {
            const char* mnemonic = result.opcode
                                       ? result.opcode->mnemonic
                                       : "???";

            // Skip the "XXXX " prefix from disassembly for cleaner output
            const char* disasm = strlen(disasmBuf) > 5
                                     ? disasmBuf + 5
                                     : disasmBuf;

            TraceState trace = { step, pc, wp, st, clocks, mnemonic,
                                 disasm, registers };
            OutputTrace(out, trace);
        }

        if (cfg.tracepointMax > 0 &&
            totalTracepointHits >= cfg.tracepointMax)
        {
            if (!cfg.quiet)
            {
                fprintf(stderr, "Tracepoint max hit (%" PRIu64
                                " total hits)\n", totalTracepointHits);
            }
            haltReason = "tracepoint-max";
            break;
        }

        if (cfg.tracepointStop && !cfg.tracepoints.empty())
        {
            const bool allHit = std::all_of(
                cfg.tracepoints.begin(), cfg.tracepoints.end(),
                [](const Tracepoint& tp) { return tp.hits != 0; });
            if (allHit)
            {
                if (!cfg.quiet)
                {
                    fprintf(stderr, "All tracepoints hit at least once\n");
                }
                haltReason = "tracepoint-stop";
                break;
            }
        }

        // A repeated executed PC is a true self-loop. Keep running when a
        // timer or future scheduled interrupt can still alter control flow.
        const bool canReceiveFutureInterrupt =
            !cfg.timers.empty() || nextIRQ < cfg.irqs.size();
        if (cfg.loopDetect && haveLastInstructionPC &&
            pc == lastInstructionPC && !canReceiveFutureInterrupt)
        {
            if (!cfg.quiet && !cfg.summary)
            {
                fprintf(stderr, "Infinite loop detected at 0x%04X\n", pc);
            }
            haltReason = "loop";
            break;
        }
        lastInstructionPC = pc;
        haveLastInstructionPC = true;
    }

    // Final stats (human-readable)
    if (!cfg.quiet && !cfg.summary)
    {
        fprintf(stderr, "\nExecution complete:\n");
        fprintf(stderr, "  Instructions: %" PRIu64 "\n", InstructionCounter);
        fprintf(stderr, "  Clocks:       %" PRIu64 "\n", ClockCycleCounter);
        fprintf(stderr, "  Final PC:     0x%04X\n", ProgramCounter);
        fprintf(stderr, "  Final ST:     0x%04X\n", Status);
        if (IsHalted())
        {
            fprintf(stderr, "  Status:       HALTED (IDLE instruction)\n");
        }
        else if (InstructionCounter >= cfg.maxSteps)
        {
            fprintf(stderr, "  Status:       MAX STEPS REACHED\n");
        }
    }

    // Summary JSON output
    if (cfg.summary)
    {
        fprintf(out, "{\"pc\":\"%04X\",\"wp\":\"%04X\",\"st\":\"%04X\",\"clk\":%" PRIu64 ",\"steps\":%" PRIu64 ",\"halt\":\"%s\",\"r\":[",
                ProgramCounter, WorkspacePtr, Status, ClockCycleCounter,
                InstructionCounter, haltReason);
        for (int i = 0; i < 16; i++)
        {
            UINT16 addr = (WorkspacePtr + i * 2) & 0xFFFE;
            UINT16 reg = (Memory[addr] << 8) | Memory[addr + 1];
            if (i > 0) fprintf(out, ",");
            fprintf(out, "\"%04X\"", reg);
        }
        fprintf(out, "]");
        if (!cfg.tracepoints.empty())
        {
            fprintf(out, ",\"tp\":{");
            for (size_t i = 0; i < cfg.tracepoints.size(); i++)
            {
                if (i > 0) fprintf(out, ",");
                fprintf(out, "\"%04X\":%" PRIu64,
                        cfg.tracepoints[i].pc, cfg.tracepoints[i].hits);
            }
            fprintf(out, "}");
        }
        if (cfg.ti99IO)
        {
            fprintf(out, ",\"vdp_latched\":%s,\"vdp_regs\":[",
                    VDP_AddressLatched ? "true" : "false");
            for (int i = 0; i < 8; i++)
            {
                if (i > 0) fprintf(out, ",");
                fprintf(out, "\"%02X\"", VDP_Regs[i]);
            }
            fprintf(out, "],\"psg\":[");
            for (size_t i = 0; i < PSG_Writes.size(); i++)
            {
                if (i > 0) fprintf(out, ",");
                fprintf(out, "\"%02X\"", PSG_Writes[i]);
            }
            fprintf(out, "],\"sams_vis\":%s,\"sams_map\":%s,\"cart_bank\":%u,\"sams_regs\":[",
                    SAMS_RegsVisible ? "true" : "false",
                    SAMS_MapEnabled ? "true" : "false",
                    static_cast<unsigned>(CART_ActiveBank));
            for (int i = 0; i < 16; i++)
            {
                if (i > 0) fprintf(out, ",");
                fprintf(out, "\"%02X\"", SAMS_Regs[i]);
            }
            fprintf(out, "]");
        }
        fprintf(out, "}\n");
    }

    // VRAM dumps
    for (const auto& dump : cfg.vramDumps)
    {
        const UINT32 end = static_cast<UINT32>(dump.start) + dump.length - 1;
        fprintf(stderr, "\nVRAM dump 0x%04X - 0x%04X (%" PRIu32 " bytes):\n",
                dump.start, end, dump.length);
        for (UINT32 offset = 0; offset < dump.length; offset += 16)
        {
            const UINT32 lineAddress = static_cast<UINT32>(dump.start) + offset;
            fprintf(stderr, "  V%04X:", lineAddress);
            for (UINT32 i = 0; i < 16 && offset + i < dump.length; i++)
            {
                fprintf(stderr, " %02X", VDP_VRAM[lineAddress + i]);
            }
            fprintf(stderr, "\n");
        }
    }

    // Memory dumps
    for (const auto& dump : cfg.dumps)
    {
        const UINT32 end = static_cast<UINT32>(dump.start) +
                           dump.length - 1;
        fprintf(stderr, "\nMemory dump 0x%04X - 0x%04X (%" PRIu32
                        " bytes):\n", dump.start, end, dump.length);

        for (UINT32 offset = 0; offset < dump.length; offset += 16)
        {
            const UINT32 lineAddress = static_cast<UINT32>(dump.start) +
                                       offset;
            fprintf(stderr, "  %04X:", lineAddress);

            // Hex bytes avoid an aligned word read beyond an odd-length dump.
            for (UINT32 i = 0; i < 16 && offset + i < dump.length; i++)
            {
                const UINT32 addr = lineAddress + i;
                fprintf(stderr, " %02X", Memory[addr]);
            }

            // ASCII representation
            fprintf(stderr, "  |");
            for (UINT32 i = 0; i < 16 && offset + i < dump.length; i++)
            {
                UINT8 byte = Memory[lineAddress + i];
                fprintf(stderr, "%c", (byte >= 32 && byte < 127) ? byte : '.');
            }
            fprintf(stderr, "|\n");
        }
    }

    if (cfg.outputFile)
    {
        fclose(out);
    }

    return 0;
}
