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
  --timer=LEVEL:PERIOD  Periodic interrupt every PERIOD cycles (can repeat)
  --wait-states=N       Add N clocks per modeled memory access
  --no-loop-detect      Do not stop on a repeated instruction PC

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

GDB Debugging:
  --gdb=PORT            Start the machine-level GDB RSP compatibility server

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
- `clk`: Clock cycles before execution
- `op`: Instruction mnemonic
- `asm`: Full disassembly
- `r`: R0-R15 before execution (hex strings)

Every field in an instruction record is captured before that instruction
executes. Interrupt entry is an explicit CPU event and does not produce a fake
instruction record or increment `step`.

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
- `clk`: Clock cycles at exit
- `steps`: Instructions executed
- `halt`: Stop reason (`"idle"`, `"stop"`, `"loop"`, `"max"`, `"tracepoint-max"`, `"tracepoint-stop"`)
- `r`: R0-R15 values (hex)

## Tracepoints (Filtered Tracing)

Tracepoints provide selective tracing - instead of tracing every instruction, only trace when PC matches a tracepoint address. This is useful for profiling specific functions without the overhead of full instruction tracing.

Tracepoints require `-S` mode. Without `-S`, all instructions are traced anyway.

Example output with `-q -S --tracepoint 0x6000 --tracepoint 0x6100 --tracepoint-stop`:
```
{"step":50,"pc":"6000","wp":"8300","st":"0000","clk":1000,"op":"LI","asm":"LI   R0,>0001","r":["0000",...]}
{"step":120,"pc":"6100","wp":"8300","st":"C000","clk":2500,"op":"RT","asm":"RT","r":["0005",...]}
{"pc":"6100","wp":"8300","st":"C000","clk":2500,"steps":120,"halt":"tracepoint-stop","r":["0005",...]}
```

Each tracepoint hit outputs the same pre-instruction record used by unfiltered
tracing. The final line is the summary JSON. To measure function cycles,
compute `clk` deltas between entry/exit trace lines.

## Memory Dumps

The `-d, --dump=START:LEN` option dumps memory ranges at exit, useful for verifying program results without implementing I/O. Output goes to stderr in hex+ASCII format:

```
Memory dump 0x8100 - 0x811F (32 bytes):
  8100: 00 37 00 56 00 8D 00 00 00 01 00 02 00 05 00 08  |.7.V............|
  8110: 00 09 00 00 00 00 00 00 00 00 00 00 00 00 00 00  |................|
```

Multiple dump ranges can be specified by repeating the `-d` option.
Ranges must stay within the flat 64K address space. A complete
`0x0000:65536` dump is valid; wrapping ranges are rejected.

## Interrupt Simulation

The `--irq=LEVEL@STEP` option triggers an interrupt at a specific instruction step:
- LEVEL: 0-15 (0 = highest priority/RESET, 15 = lowest)
- STEP: Instruction count at which to trigger

The interrupt is processed according to TMS9900 behavior:
1. Interrupt recognized if level <= mask in ST bits 0-3
2. Context switch: fetch WP/PC from vector (level * 4), save old state to R13/R14/R15
3. New interrupt mask = level - 1 (except level 0 stays 0)

RESET (level 0) clears ST before saving the new context and bypasses the
one-instruction maskable-interrupt inhibition. BLWP, XOP, and interrupt entry
inhibit maskable interrupt recognition until the first target instruction has
executed.

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

## Cycle Timing

The `clk` field follows the TMS9900 Data Manual timing equation:

```
clocks = C + W * M
```

`C` and `M` include the instruction's Table 3 base clocks and memory accesses
plus the applicable Table A/B source and destination addressing-mode
additions. `W` is configured with `--wait-states=N` and defaults to zero.
Counters are 64-bit.

The wait-state setting is deliberately uniform. It is suitable for a flat
memory/compiler harness, but does not model a TI-99/4A's mix of fast 16-bit and
wait-stated 8-bit regions.

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

TI documents successful DIV as data-dependent on internal partial quotients
but does not publish the divider microcode. The simulator uses MAME's bounded
92-124-cycle model rather than claiming undocumented cycle-exact DIV timing.

## Testing

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

CTest covers CPU control and interrupt sequencing, all opcode decode words,
addressing modes, all branch/status combinations, exhaustive ALU/byte/shift
semantics, timing/wait-state cases, disassembly wraparound, CLI trace/summary
behavior, IDLE timer wakeup, memory dumps, and GDB packet validation.

## GDB Compatibility Server

`--gdb=PORT` exposes register inspection, memory access, stepping, continue,
and software breakpoints through the GDB Remote Serial Protocol. Stock GDB has
no TMS9900 architecture, so the server advertises an MSP430-shaped register
set: PC, WP, ST, and TMS9900 R0-R12. TMS9900 R13-R15 remain accessible through
workspace memory.

This mode is useful for machine-level inspection only. MSP430 disassembly and
the register numbering do not match TMS9900 compiler DWARF, so source-level
debugging requires a real TMS9900 GDB architecture port.

## Limitations

- CRU instructions (SBO, SBZ, TB, LDCR, STCR) are accepted but have no hardware effect
  - `LDCR`: Reads source, updates flags, CRU write is no-op
  - `STCR`: CRU read returns 0, writes to destination
  - `TB`: Always returns 0 (bit not set)
- No memory-mapped I/O (VDP, GROM, sound, etc.)
- Memory is flat 64K RAM (no ROM/write protection)
- External-operation pins for CKON, CKOF, LREX, RSET, and IDLE are not exposed
- LOAD and bus-visible discarded reads are not modeled

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
