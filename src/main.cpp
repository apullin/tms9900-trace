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
    UINT16 length;
};

struct Tracepoint
{
    UINT16 pc;
    UINT32 hits = 0;
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
    std::vector<MemoryDump> dumps;
    std::vector<Tracepoint> tracepoints;
    UINT64 tracepointMax = 0;
    bool tracepointStop = false;
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

    if (*end != '\0' || length == 0 || length > 0x10000)
    {
        return false;
    }

    dump->start = start;
    dump->length = (UINT16)length;
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
    UINT32 clocks;
    const char* mnemonic;
    const char* disasm;
};

//----------------------------------------------------------------------------
// Output JSON trace line (all hex for values)
//----------------------------------------------------------------------------

static void OutputTrace(FILE* out, const TraceState& t)
{
    fprintf(out, "{\"step\":%" PRIu64 ",\"pc\":\"%04X\",\"wp\":\"%04X\",\"st\":\"%04X\",\"clk\":%" PRIu32 ",",
            t.step, t.pc, t.wp, t.st, t.clocks);
    fprintf(out, "\"op\":\"%s\",\"asm\":\"%s\",\"r\":[", t.mnemonic, t.disasm);

    // Output registers as hex strings (read directly, don't affect clock)
    for (int i = 0; i < 16; i++)
    {
        UINT16 addr = (t.wp + i * 2) & 0xFFFE;
        UINT16 reg = (Memory[addr] << 8) | Memory[addr + 1];
        if (i > 0) fprintf(out, ",");
        fprintf(out, "\"%04X\"", reg);
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

    static struct option longOpts[] = {
        {"load",           required_argument, nullptr, 'l'},
        {"entry",          required_argument, nullptr, 'e'},
        {"wp",             required_argument, nullptr, 'w'},
        {"max-steps",      required_argument, nullptr, 'n'},
        {"stop-at",        required_argument, nullptr, 's'},
        {"irq",            required_argument, nullptr, 'i'},
        {"output",         required_argument, nullptr, 'o'},
        {"dump",           required_argument, nullptr, 'd'},
        {"tracepoint",     required_argument, nullptr, 't'},
        {"tracepoint-file", required_argument, nullptr, 'T'},
        {"tracepoint-max", required_argument, nullptr, 'M'},
        {"tracepoint-stop", no_argument,      nullptr, 'P'},
        {"quiet",          no_argument,       nullptr, 'q'},
        {"summary",        no_argument,       nullptr, 'S'},
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

    // Load binary
    if (!LoadBinary(cfg.inputFile, cfg.loadAddr))
    {
        if (cfg.outputFile) fclose(out);
        return 1;
    }

    // Set up initial state
    WorkspacePtr = cfg.wpAddr;
    ProgramCounter = cfg.entryAddr;
    Status = 0;
    ClockCycleCounter = 0;
    InstructionCounter = 0;

    if (!cfg.quiet && !cfg.summary)
    {
        fprintf(stderr, "TMS9900 Trace\n");
        fprintf(stderr, "  Input:  %s\n", cfg.inputFile);
        fprintf(stderr, "  Load:   0x%04X\n", cfg.loadAddr);
        fprintf(stderr, "  Entry:  0x%04X\n", cfg.entryAddr);
        fprintf(stderr, "  WP:     0x%04X\n", cfg.wpAddr);
        fprintf(stderr, "  Max:    %" PRIu64 " steps\n", cfg.maxSteps);
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
    UINT64 step = 0;
    bool halted = false;
    UINT16 lastPC = 0xFFFF;
    size_t nextIRQ = 0;
    const char* haltReason = "max";
    UINT64 totalTracepointHits = 0;

    while (step < cfg.maxSteps && !halted)
    {
        // Check for scheduled interrupts
        while (nextIRQ < cfg.irqs.size() && cfg.irqs[nextIRQ].atStep <= step)
        {
            TriggerInterrupt(cfg.irqs[nextIRQ].level);
            if (!cfg.quiet && !cfg.summary)
            {
                fprintf(stderr, "[Step %" PRIu64 "] Triggered IRQ level %d\n",
                        step, cfg.irqs[nextIRQ].level);
            }
            nextIRQ++;
        }

        // Capture pre-state
        UINT16 pc = ProgramCounter;
        UINT16 wp = WorkspacePtr;
        UINT16 st = Status;
        UINT32 clocks = ClockCycleCounter;

        // Disassemble early (needed for tracepoint output before stop checks)
        DisassembleASM(pc, &Memory[pc], disasmBuf);

        // Check tracepoints - output trace line immediately if hit
        for (auto& tp : cfg.tracepoints)
        {
            if (pc == tp.pc)
            {
                tp.hits++;
                totalTracepointHits++;

                // Output tracepoint trace immediately (before stop checks)
                const char* disasm = disasmBuf;
                if (strlen(disasmBuf) > 5)
                {
                    disasm = disasmBuf + 5;  // Skip "XXXX "
                }
                // Extract mnemonic from disasm (first word)
                static char mnemonicBuf[16];
                const char* space = strchr(disasm, ' ');
                if (space)
                {
                    size_t len = space - disasm;
                    if (len >= sizeof(mnemonicBuf)) len = sizeof(mnemonicBuf) - 1;
                    strncpy(mnemonicBuf, disasm, len);
                    mnemonicBuf[len] = '\0';
                }
                else
                {
                    strncpy(mnemonicBuf, disasm, sizeof(mnemonicBuf) - 1);
                    mnemonicBuf[sizeof(mnemonicBuf) - 1] = '\0';
                }

                TraceState trace = { step, pc, wp, st, clocks, mnemonicBuf, disasm };
                OutputTrace(out, trace);
                break;  // Only match one tracepoint per PC
            }
        }

        // Check tracepoint-max
        if (cfg.tracepointMax > 0 && totalTracepointHits >= cfg.tracepointMax)
        {
            if (!cfg.quiet)
            {
                fprintf(stderr, "Tracepoint max hit (%" PRIu64 " total hits)\n",
                        totalTracepointHits);
            }
            haltReason = "tracepoint-max";
            break;
        }

        // Check tracepoint-stop (all tracepoints hit at least once)
        if (cfg.tracepointStop && !cfg.tracepoints.empty())
        {
            bool allHit = true;
            for (const auto& tp : cfg.tracepoints)
            {
                if (tp.hits == 0)
                {
                    allHit = false;
                    break;
                }
            }
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

        // Check for infinite loop (JMP $)
        if (pc == lastPC)
        {
            if (!cfg.quiet && !cfg.summary)
            {
                fprintf(stderr, "Infinite loop detected at 0x%04X\n", pc);
            }
            haltReason = "loop";
            break;
        }
        lastPC = pc;

        // Execute
        halted = StepCPU();
        if (halted)
        {
            haltReason = "idle";
        }

        // Output trace in normal mode (tracepoint trace already output above)
        if (!cfg.summary)
        {
            sOpCode* op = GetLastOpcode();
            const char* mnemonic = op ? op->mnemonic : "???";

            // Skip the "XXXX " prefix from disassembly for cleaner output
            const char* disasm = disasmBuf;
            if (strlen(disasmBuf) > 5)
            {
                disasm = disasmBuf + 5;  // Skip "XXXX "
            }

            TraceState trace = { step, pc, wp, st, clocks, mnemonic, disasm };
            OutputTrace(out, trace);
        }

        step++;
    }

    // Final stats (human-readable)
    if (!cfg.quiet && !cfg.summary)
    {
        fprintf(stderr, "\nExecution complete:\n");
        fprintf(stderr, "  Instructions: %" PRIu64 "\n", step);
        fprintf(stderr, "  Clocks:       %u (rough estimate)\n", ClockCycleCounter);
        fprintf(stderr, "  Final PC:     0x%04X\n", ProgramCounter);
        fprintf(stderr, "  Final ST:     0x%04X\n", Status);
        if (halted)
        {
            fprintf(stderr, "  Status:       HALTED (IDLE instruction)\n");
        }
        else if (step >= cfg.maxSteps)
        {
            fprintf(stderr, "  Status:       MAX STEPS REACHED\n");
        }
    }

    // Summary JSON output
    if (cfg.summary)
    {
        fprintf(out, "{\"pc\":\"%04X\",\"wp\":\"%04X\",\"st\":\"%04X\",\"clk\":%" PRIu32 ",\"steps\":%" PRIu64 ",\"halt\":\"%s\",\"r\":[",
                ProgramCounter, WorkspacePtr, Status, ClockCycleCounter,
                step, haltReason);
        for (int i = 0; i < 16; i++)
        {
            UINT16 addr = (WorkspacePtr + i * 2) & 0xFFFE;
            UINT16 reg = (Memory[addr] << 8) | Memory[addr + 1];
            if (i > 0) fprintf(out, ",");
            fprintf(out, "\"%04X\"", reg);
        }
        fprintf(out, "]}\n");
    }

    // Memory dumps
    for (const auto& dump : cfg.dumps)
    {
        fprintf(stderr, "\nMemory dump 0x%04X - 0x%04X (%u bytes):\n",
                dump.start, dump.start + dump.length - 1, dump.length);

        for (UINT16 offset = 0; offset < dump.length; offset += 16)
        {
            fprintf(stderr, "  %04X:", dump.start + offset);

            // Hex words
            for (UINT16 i = 0; i < 16 && offset + i < dump.length; i += 2)
            {
                UINT16 addr = dump.start + offset + i;
                UINT16 word = ReadWord(addr);
                fprintf(stderr, " %04X", word);
            }

            // ASCII representation
            fprintf(stderr, "  |");
            for (UINT16 i = 0; i < 16 && offset + i < dump.length; i++)
            {
                UINT8 byte = Memory[dump.start + offset + i];
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
