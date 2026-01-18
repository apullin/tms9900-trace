# TMS9900 Trace

Standalone TMS9900 CPU simulator with per-instruction trace output.

## Purpose

This tool runs TMS9900 machine code binaries and outputs a detailed trace of each instruction executed in NDJSON format. It's designed for testing compilers and assemblers targeting the TMS9900 processor.

Unlike full TI-99/4a emulators, this tool:
- Runs programs from any address (no ROM requirements)
- Uses flat 64K memory (no banking or memory-mapped I/O)
- Outputs machine-readable trace data
- Supports scheduled interrupt injection for testing ISR code

## Building

```bash
mkdir build && cd build
cmake ..
make
```

## Usage

```
tms9900-trace [options] <binary.bin>

Memory Options:
  -l, --load=ADDR       Load address (hex, default: 0x0000)
  -e, --entry=ADDR      Entry point (hex, default: same as load)
  -w, --wp=ADDR         Workspace pointer (hex, default: 0x8300)

Execution Options:
  -n, --max-steps=N     Max instructions (default: 1000000)
  -s, --stop-at=ADDR    Stop at address (hex, can repeat)
  --irq=LEVEL@STEP      Trigger interrupt at step (can repeat)

Output Options:
  -o, --output=FILE     Output file (default: stdout)
  -q, --quiet           Only output trace, no status messages
  -S, --summary         Output only final state as JSON (no per-step trace)
  -d, --dump=START:LEN  Dump memory range at exit (hex, can repeat)

Tracepoint Options (require -S):
  -t, --tracepoint=ADDR       Trace only this address (hex, can repeat)
  -T, --tracepoint-file=FILE  Load tracepoint addresses from file
  --tracepoint-max=N          Stop after N total tracepoint hits
  --tracepoint-stop           Stop when all tracepoints hit at least once

Other:
  -h, --help            Show help
```

## Output Format

Each line is a JSON object with all values in hex:

```json
{"step":0,"pc":"6000","wp":"8300","st":"0000","clk":0,"op":"LWPI","asm":"LWPI >8300","r":["0000",...]}
```

Fields:
- `step`: Instruction number (0-indexed)
- `pc`: Program counter before execution (hex)
- `wp`: Workspace pointer before execution (hex)
- `st`: Status register before execution (hex)
- `clk`: Clock cycles (rough estimate, see below)
- `op`: Instruction mnemonic
- `asm`: Full disassembly
- `r`: Array of R0-R15 values (hex strings)

## Examples

Run a program loaded at 0x6000:
```bash
./tms9900-trace -l 0x6000 program.bin
```

Run 100 instructions and save trace:
```bash
./tms9900-trace -n 100 -o trace.ndjson test.bin
```

Stop when reaching address 0x6100:
```bash
./tms9900-trace -l 0x6000 -s 0x6100 program.bin
```

Multiple stop addresses:
```bash
./tms9900-trace -s 0x6100 -s 0x6200 -s 0x6300 program.bin
```

Test interrupt handling (trigger IRQ level 1 after 500 instructions):
```bash
./tms9900-trace --irq=1@500 program.bin
```

Dump memory after execution to verify results:
```bash
./tms9900-trace -l 0x8000 -d 0x8100:32 -d 0x8200:16 program.bin
```

Run silently, output only final state (for test harnesses):
```bash
./tms9900-trace -S test.bin
```

Profile a function (trace only entry/exit points):
```bash
./tms9900-trace -q -S --tracepoint 0x6000 --tracepoint 0x6100 --tracepoint-stop program.bin
```

Trace specific addresses from a file:
```bash
./tms9900-trace -q -S --tracepoint-file tracepoints.txt program.bin
```

## Summary Mode (Test Harness)

The `-S, --summary` option runs silently and outputs a single JSON line with final CPU state:

```json
{"pc":"0010","wp":"8300","st":"C000","clk":76,"steps":5,"halt":"idle","r":["1235","0001",...]}
```

Fields:
- `pc`, `wp`, `st`: Final CPU registers (hex)
- `clk`: Clock cycles (rough estimate)
- `steps`: Instructions executed
- `halt`: Stop reason (`"idle"`, `"stop"`, `"loop"`, `"max"`, `"tracepoint-max"`, `"tracepoint-stop"`)
- `r`: R0-R15 values (hex)

## Tracepoints (Filtered Tracing)

Tracepoints provide selective tracing - instead of tracing every instruction, only trace when PC matches a tracepoint address. This is useful for profiling specific functions without the overhead of full instruction tracing.

Tracepoints require `-S` mode. Without `-S`, all instructions are traced anyway.

Example output with `-q -S --tracepoint 0x6000 --tracepoint 0x6100 --tracepoint-stop`:
```
{"step":50,"pc":"6000","wp":"8300","st":"0000","clk":1000,"op":"LI","asm":"LI   R0,>0001","r":["0001",...]}
{"step":120,"pc":"6100","wp":"8300","st":"C000","clk":2500,"op":"RT","asm":"RT","r":["0005",...]}
{"pc":"6100","wp":"8300","st":"C000","clk":2500,"steps":120,"halt":"tracepoint-stop","r":["0005",...]}
```

Each tracepoint hit outputs a full trace line. The final line is the summary JSON. To measure function cycles, compute `clk` deltas between entry/exit trace lines.

## Memory Dumps

The `-d, --dump=START:LEN` option dumps memory ranges at exit, useful for verifying program results without implementing I/O. Output goes to stderr in hex+ASCII format:

```
Memory dump 0x8100 - 0x811F (32 bytes):
  8100: 0037 0056 008D 0000 0001 0002 0005 0008  |.7.V............|
  8110: 0009 0000 0000 0000 0000 0000 0000 0000  |................|
```

Multiple dump ranges can be specified by repeating the `-d` option.

## Interrupt Simulation

The `--irq=LEVEL@STEP` option triggers an interrupt at a specific instruction step:
- LEVEL: 0-15 (0 = highest priority/RESET, 15 = lowest)
- STEP: Instruction count at which to trigger

The interrupt is processed according to TMS9900 behavior:
1. Interrupt recognized if level <= mask in ST bits 0-3
2. Context switch: fetch WP/PC from vector (level * 4), save old state to R13/R14/R15
3. New interrupt mask = level - 1 (except level 0 stays 0)

Vector addresses:
- Level 0 (RESET): 0x0000
- Level 1: 0x0004
- Level 2: 0x0008
- ...
- Level 15: 0x003C

To test interrupt handling, ensure your program:
1. Sets up an interrupt vector at the appropriate address
2. Has an interrupt mask that enables the level (use LIMI)
3. Has an ISR that returns with RTWP

## Termination Conditions

The simulator stops when:
1. Maximum steps reached (default 1M)
2. Stop address is reached (`-s` option)
3. IDLE instruction executed (unless an interrupt wakes it)
4. Infinite loop detected (PC unchanged after instruction)
5. Tracepoint max hits reached (`--tracepoint-max`)
6. All tracepoints hit at least once (`--tracepoint-stop`)

## Cycle Timing (Rough Estimate)

The `clk` field provides a rough cycle count based on base instruction timings from the TMS9900 Data Manual. This does NOT account for addressing mode overhead or memory wait states - actual hardware timing would be higher.

Base timings (register-to-register only):
- Arithmetic (A, S, etc): 14 cycles
- Load Immediate (LI): 12 cycles
- Branch (B): 8 cycles
- Branch and Link (BL): 12 cycles
- BLWP: 26 cycles
- Jump (taken): 10 cycles
- Jump (not taken): 8 cycles
- Shift: 12 + 2*count cycles
- Multiply: 52 cycles
- Divide: 16 (overflow) or 92-124 cycles

## Limitations

- CRU instructions (SBO, SBZ, TB, LDCR, STCR) are accepted but have no hardware effect
  - `LDCR`: Reads source, updates flags, CRU write is no-op
  - `STCR`: CRU read returns 0, writes to destination
  - `TB`: Always returns 0 (bit not set)
- No memory-mapped I/O (VDP, GROM, sound, etc.)
- Memory is flat 64K RAM (no ROM/write protection)

## Status Register Bits

```
Bit 15 (0x8000): L>  - Logical greater than
Bit 14 (0x4000): A>  - Arithmetic greater than
Bit 13 (0x2000): EQ  - Equal
Bit 12 (0x1000): C   - Carry
Bit 11 (0x0800): OV  - Overflow
Bit 10 (0x0400): OP  - Odd parity
Bit  9 (0x0200): X   - XOP in progress
Bits 0-3 (0x000F): Interrupt mask
```

## Heritage

CPU emulation core adapted from [ti99sim](http://www.mrousseau.org/programs/ti99sim/) by Marc Rousseau.

Cycle timing reference: TMS9900 Microprocessor Data Manual, Texas Instruments, May 1976.

Licensed under GPL v2.
