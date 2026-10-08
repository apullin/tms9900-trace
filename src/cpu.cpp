//----------------------------------------------------------------------------
// TMS9900 Trace - Standalone TMS9900 CPU Simulator
//
// cpu.cpp - TMS9900 CPU emulation core
//
// Adapted from ti99sim by Marc Rousseau
// Original: Copyright (c) 1998-2004 Marc Rousseau, All Rights Reserved.
// Licensed under GPL v2
//
// Modifications for standalone use:
// - Removed TI-99/4a specific dependencies (CRU, TMS9901, memory manager)
// - Simplified to flat 64K memory model
// - CRU pins are not modeled; instructions retain their CPU-visible effects
// - Interrupt handling via scheduled interrupts (--irq option)
//
// Cycle timing reference: TMS9900 Data Manual (May 1976), Table 3
//----------------------------------------------------------------------------

#include "cpu.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>

//----------------------------------------------------------------------------
// Macros for register access (matches original)
//----------------------------------------------------------------------------

#define WP  WorkspacePtr
#define PC  fetchPtr
#define ST  Status

//----------------------------------------------------------------------------
// CPU State
//----------------------------------------------------------------------------

UINT8  Memory[0x10000];          // 64K flat memory
UINT16 WorkspacePtr;             // WP
UINT16 ProgramCounter;           // PC
UINT16 Status;                   // ST
UINT64 ClockCycleCounter;        // Clock cycles
UINT64 InstructionCounter;       // Instructions executed

static UINT16  fetchPtr;         // Current fetch pointer
static UINT16  curOpCode;        // Current opcode being executed
static UINT16  parity[256];      // Parity lookup table
static bool    cpuHalted;        // IDLE instruction executed
static sOpCode* lastOpcode;      // Last executed opcode
static int     pendingInterrupt; // Pending interrupt level (-1 = none)
static bool    interruptDefer;   // Defer interrupts for one instruction
static bool    lookupInitialized;
static UINT32  memoryWaitStates;
static UINT64  instructionCycles;
static UINT64  instructionMemoryAccesses;

//----------------------------------------------------------------------------
// Opcode lookup tables
//----------------------------------------------------------------------------

struct sLookUp
{
    sLookUp    *next;
    sOpCode    *opCode;
};

static sLookUp lookUp[16];

//----------------------------------------------------------------------------
// Forward declarations for opcode functions
//----------------------------------------------------------------------------

static void opcode_A();
static void opcode_AB();
static void opcode_ABS();
static void opcode_AI();
static void opcode_ANDI();
static void opcode_B();
static void opcode_BL();
static void opcode_BLWP();
static void opcode_C();
static void opcode_CB();
static void opcode_CI();
static void opcode_CKOF();
static void opcode_CKON();
static void opcode_CLR();
static void opcode_COC();
static void opcode_CZC();
static void opcode_DEC();
static void opcode_DECT();
static void opcode_DIV();
static void opcode_IDLE();
static void opcode_INC();
static void opcode_INCT();
static void opcode_INV();
static void opcode_JEQ();
static void opcode_JGT();
static void opcode_JH();
static void opcode_JHE();
static void opcode_JL();
static void opcode_JLE();
static void opcode_JLT();
static void opcode_JMP();
static void opcode_JNC();
static void opcode_JNE();
static void opcode_JNO();
static void opcode_JOC();
static void opcode_JOP();
static void opcode_LDCR();
static void opcode_LI();
static void opcode_LIMI();
static void opcode_LREX();
static void opcode_LWPI();
static void opcode_MOV();
static void opcode_MOVB();
static void opcode_MPY();
static void opcode_NEG();
static void opcode_ORI();
static void opcode_RSET();
static void opcode_RTWP();
static void opcode_S();
static void opcode_SB();
static void opcode_SBO();
static void opcode_SBZ();
static void opcode_SETO();
static void opcode_SLA();
static void opcode_SOC();
static void opcode_SOCB();
static void opcode_SRA();
static void opcode_SRC();
static void opcode_SRL();
static void opcode_STCR();
static void opcode_STST();
static void opcode_STWP();
static void opcode_SWPB();
static void opcode_SZC();
static void opcode_SZCB();
static void opcode_TB();
static void opcode_X();
static void opcode_XOP();
static void opcode_XOR();
static void InvalidOpcode();

//----------------------------------------------------------------------------
// Opcode table
//----------------------------------------------------------------------------

static sOpCode InvalidOp = {
    "INVL", 0x0000, 0x0000, 0, InvalidOpcode, 6, 1
};

// N fields are don't-care bits (Data Manual 3.5.9-3.5.13). The upper
// eleven bits select immediate, internal-register, and external operations.
static sOpCode OpCodes[69] =
{
    { "A",    0xA000, 0xF000, 1, opcode_A,    14, 4 },
    { "AB",   0xB000, 0xF000, 1, opcode_AB,   14, 4 },
    { "ABS",  0x0740, 0xFFC0, 6, opcode_ABS,  12, 2 },
    { "AI",   0x0220, 0xFFE0, 8, opcode_AI,   14, 4 },
    { "ANDI", 0x0240, 0xFFE0, 8, opcode_ANDI, 14, 4 },
    { "B",    0x0440, 0xFFC0, 6, opcode_B,     8, 2 },
    { "BL",   0x0680, 0xFFC0, 6, opcode_BL,   12, 3 },
    { "BLWP", 0x0400, 0xFFC0, 6, opcode_BLWP, 26, 6 },
    { "C",    0x8000, 0xF000, 1, opcode_C,    14, 3 },
    { "CB",   0x9000, 0xF000, 1, opcode_CB,   14, 3 },
    { "CI",   0x0280, 0xFFE0, 8, opcode_CI,   14, 3 },
    { "CKOF", 0x03C0, 0xFFE0, 7, opcode_CKOF, 12, 1 },
    { "CKON", 0x03A0, 0xFFE0, 7, opcode_CKON, 12, 1 },
    { "CLR",  0x04C0, 0xFFC0, 6, opcode_CLR,  10, 3 },
    { "COC",  0x2000, 0xFC00, 3, opcode_COC,  14, 3 },
    { "CZC",  0x2400, 0xFC00, 3, opcode_CZC,  14, 3 },
    { "DEC",  0x0600, 0xFFC0, 6, opcode_DEC,  10, 3 },
    { "DECT", 0x0640, 0xFFC0, 6, opcode_DECT, 10, 3 },
    { "DIV",  0x3C00, 0xFC00, 9, opcode_DIV,  16, 3 },
    { "IDLE", 0x0340, 0xFFE0, 7, opcode_IDLE, 12, 1 },
    { "INC",  0x0580, 0xFFC0, 6, opcode_INC,  10, 3 },
    { "INCT", 0x05C0, 0xFFC0, 6, opcode_INCT, 10, 3 },
    { "INV",  0x0540, 0xFFC0, 6, opcode_INV,  10, 3 },
    { "JEQ",  0x1300, 0xFF00, 2, opcode_JEQ,   8, 1 },
    { "JGT",  0x1500, 0xFF00, 2, opcode_JGT,   8, 1 },
    { "JH",   0x1B00, 0xFF00, 2, opcode_JH,    8, 1 },
    { "JHE",  0x1400, 0xFF00, 2, opcode_JHE,   8, 1 },
    { "JL",   0x1A00, 0xFF00, 2, opcode_JL,    8, 1 },
    { "JLE",  0x1200, 0xFF00, 2, opcode_JLE,   8, 1 },
    { "JLT",  0x1100, 0xFF00, 2, opcode_JLT,   8, 1 },
    { "JMP",  0x1000, 0xFF00, 2, opcode_JMP,   8, 1 },
    { "JNC",  0x1700, 0xFF00, 2, opcode_JNC,   8, 1 },
    { "JNE",  0x1600, 0xFF00, 2, opcode_JNE,   8, 1 },
    { "JNO",  0x1900, 0xFF00, 2, opcode_JNO,   8, 1 },
    { "JOC",  0x1800, 0xFF00, 2, opcode_JOC,   8, 1 },
    { "JOP",  0x1C00, 0xFF00, 2, opcode_JOP,   8, 1 },
    { "LDCR", 0x3000, 0xFC00, 4, opcode_LDCR, 20, 3 },
    { "LI",   0x0200, 0xFFE0, 8, opcode_LI,   12, 3 },
    { "LIMI", 0x0300, 0xFFE0, 8, opcode_LIMI, 16, 2 },
    { "LREX", 0x03E0, 0xFFE0, 7, opcode_LREX, 12, 1 },
    { "LWPI", 0x02E0, 0xFFE0, 8, opcode_LWPI, 10, 2 },
    { "MOV",  0xC000, 0xF000, 1, opcode_MOV,  14, 4 },
    { "MOVB", 0xD000, 0xF000, 1, opcode_MOVB, 14, 4 },
    { "MPY",  0x3800, 0xFC00, 9, opcode_MPY,  52, 5 },
    { "NEG",  0x0500, 0xFFC0, 6, opcode_NEG,  12, 3 },
    { "ORI",  0x0260, 0xFFE0, 8, opcode_ORI,  14, 4 },
    { "RSET", 0x0360, 0xFFE0, 7, opcode_RSET, 12, 1 },
    { "RTWP", 0x0380, 0xFFE0, 7, opcode_RTWP, 14, 4 },
    { "S",    0x6000, 0xF000, 1, opcode_S,    14, 4 },
    { "SB",   0x7000, 0xF000, 1, opcode_SB,   14, 4 },
    { "SBO",  0x1D00, 0xFF00, 2, opcode_SBO,  12, 2 },
    { "SBZ",  0x1E00, 0xFF00, 2, opcode_SBZ,  12, 2 },
    { "SETO", 0x0700, 0xFFC0, 6, opcode_SETO, 10, 3 },
    { "SLA",  0x0A00, 0xFF00, 5, opcode_SLA,  12, 3 },
    { "SOC",  0xE000, 0xF000, 1, opcode_SOC,  14, 4 },
    { "SOCB", 0xF000, 0xF000, 1, opcode_SOCB, 14, 4 },
    { "SRA",  0x0800, 0xFF00, 5, opcode_SRA,  12, 3 },
    { "SRC",  0x0B00, 0xFF00, 5, opcode_SRC,  12, 3 },
    { "SRL",  0x0900, 0xFF00, 5, opcode_SRL,  12, 3 },
    { "STCR", 0x3400, 0xFC00, 4, opcode_STCR, 42, 4 },
    { "STST", 0x02C0, 0xFFE0, 8, opcode_STST,  8, 2 },
    { "STWP", 0x02A0, 0xFFE0, 8, opcode_STWP,  8, 2 },
    { "SWPB", 0x06C0, 0xFFC0, 6, opcode_SWPB, 10, 3 },
    { "SZC",  0x4000, 0xF000, 1, opcode_SZC,  14, 4 },
    { "SZCB", 0x5000, 0xF000, 1, opcode_SZCB, 14, 4 },
    { "TB",   0x1F00, 0xFF00, 2, opcode_TB,   12, 2 },
    { "X",    0x0480, 0xFFC0, 6, opcode_X,     8, 2 },
    { "XOP",  0x2C00, 0xFC00, 9, opcode_XOP,  36, 8 },
    { "XOR",  0x2800, 0xFC00, 3, opcode_XOR,  14, 4 }
};

//----------------------------------------------------------------------------
// Timing and memory access
//----------------------------------------------------------------------------

static void AddInstructionTiming(UINT32 clocks, UINT32 memoryAccesses = 0)
{
    instructionCycles += clocks;
    instructionMemoryAccesses += memoryAccesses;
}

static void CommitInstructionTiming()
{
    ClockCycleCounter += instructionCycles;
    ClockCycleCounter +=
        static_cast<UINT64>(memoryWaitStates) * instructionMemoryAccesses;
}

UINT8 ReadByte(UINT16 address)
{
    return Memory[address];
}

UINT16 ReadWord(UINT16 address)
{
    address &= 0xFFFE;  // Word align
    return (Memory[address] << 8) | Memory[address + 1];
}

void WriteByte(UINT16 address, UINT8 value)
{
    Memory[address] = value;
}

void WriteWord(UINT16 address, UINT16 value)
{
    address &= 0xFFFE;  // Word align
    Memory[address] = value >> 8;
    Memory[address + 1] = value & 0xFF;
}

// Internal data accesses are untimed here. Instruction timing is accounted
// from the manual's C/M metadata and addressing-mode additions.
static UINT16 ReadMemoryW(UINT16 address)
{
    return ReadWord(address);
}

static UINT8 ReadMemoryB(UINT16 address)
{
    return ReadByte(address);
}

static void WriteMemoryW(UINT16 address, UINT16 value)
{
    WriteWord(address, value);
}

static void WriteMemoryB(UINT16 address, UINT8 value)
{
    WriteByte(address, value);
}

static UINT16 Fetch()
{
    UINT16 retVal = ReadMemoryW(PC);
    PC += 2;
    return retVal;
}

// Table 3 addressing-mode additions. Word operands use Table A and byte
// operands use Table B. The base instruction entry already includes its
// opcode fetch and operand accesses.
static void AddAddressTiming(UINT16 opCode, size_t size)
{
    switch (opCode & 0x0030)
    {
        case 0x0000:  // Register direct
            break;

        case 0x0010:  // Register indirect
            AddInstructionTiming(4, 1);
            break;

        case 0x0030:  // Auto-increment
            AddInstructionTiming(size == 1 ? 6 : 8, 2);
            break;

        case 0x0020:  // Symbolic/indexed
            AddInstructionTiming(8, (opCode & 0x000F) ? 2 : 1);
            break;
    }
}

//----------------------------------------------------------------------------
// Opcode lookup initialization
//----------------------------------------------------------------------------

static void InitOpCodeLookup()
{
    // Fill in the parity table
    for (size_t i = 0; i < 256; i++)
    {
        size_t value = i;
        value ^= value >> 1;
        value ^= value >> 4;
        value ^= value >> 2;
        parity[i] = (value & 1) ? TMS_PARITY : 0;
    }

    // Initialize lookup table entries to invalid
    for (int i = 0; i < 16; i++)
    {
        lookUp[i].next = nullptr;
        lookUp[i].opCode = &InvalidOp;
    }

    // Create the LookUp tables
    for (auto &opcode : OpCodes)
    {
        int code = opcode.opCode;
        int mask = opcode.mask;

        sLookUp *table = lookUp;

        while (mask & 0x0FFF)
        {
            sLookUp *&next = table[code >> 12].next;
            if (next == nullptr)
            {
                next = static_cast<sLookUp *>(calloc(16, sizeof(sLookUp)));
                for (int i = 0; i < 16; i++)
                {
                    next[i].opCode = &InvalidOp;
                }
            }
            table = next;
            code = (code << 4) & 0xFFFF;
            mask = (mask << 4) & 0xFFFF;
        }

        code >>= 12;
        mask >>= 12;

        for (int i = 0; (mask & i) == 0; i++)
        {
            table[code + i].opCode = &opcode;
        }
    }
}

sOpCode* LookupOpCode(UINT16 opcode)
{
    sLookUp *table = &lookUp[opcode >> 12];

    if (table->next == nullptr) return table->opCode;

    table = &table->next[(opcode >> 8) & 0x0F];

    if (table->next == nullptr) return table->opCode;

    table = &table->next[(opcode >> 4) & 0x0F];

    if (table->next == nullptr) return table->opCode;

    table = &table->next[opcode & 0x0F];

    return table->opCode;
}

//----------------------------------------------------------------------------
// Address mode decoding
//----------------------------------------------------------------------------

static UINT16 GetAddress(UINT16 opCode, size_t size)
{
    UINT16 address;
    int reg = opCode & 0x0F;

    AddAddressTiming(opCode, size);

    switch (opCode & 0x0030)
    {
        case 0x0000:  // Register direct
            address = (UINT16)(WP + 2 * reg);
            break;
        case 0x0010:  // Register indirect
            address = ReadMemoryW(WP + 2 * reg);
            break;
        case 0x0030:  // Auto-increment
            address = ReadMemoryW(WP + 2 * reg);
            WriteMemoryW(WP + 2 * reg, (UINT16)(address + size));
            break;
        case 0x0020:  // Symbolic/Indexed
            address = Fetch();
            if (reg)
            {
                address += ReadMemoryW(WP + 2 * reg);
            }
            break;
    }

    if (size == 2)
    {
        address &= 0xFFFE;
    }

    return address;
}

//----------------------------------------------------------------------------
// Context switch (for BLWP, XOP, interrupts)
//----------------------------------------------------------------------------

static void ContextSwitch(UINT16 address)
{
    UINT16 newWP = ReadMemoryW(address);
    UINT16 newPC = ReadMemoryW(address + 2);

    UINT16 oldWP = WP;
    UINT16 oldPC = PC;
    WP = newWP & 0xFFFE;
    PC = newPC & 0xFFFE;
    ProgramCounter = PC;

    WriteMemoryW(WP + 2 * 13, oldWP);
    WriteMemoryW(WP + 2 * 14, oldPC);
    WriteMemoryW(WP + 2 * 15, ST);
}

//----------------------------------------------------------------------------
// Interrupt handling
//
// Per TMS9900 manual:
// - 16 interrupt levels (0 = highest priority, 15 = lowest)
// - Level 0 is RESET (cannot be disabled)
// - Interrupt recognized when level <= mask in ST bits 0-3
// - Context switch: fetch WP/PC from vector, save old WP/PC/ST to R13/R14/R15
// - New interrupt mask = level - 1 (except level 0 stays 0)
// - Interrupts inhibited for one instruction after context switch
//----------------------------------------------------------------------------

static bool CheckAndHandleInterrupt()
{
    if (pendingInterrupt < 0)
    {
        interruptDefer = false;
        return false;
    }

    int level = pendingInterrupt;

    // RESET is not subject to the one-instruction interrupt inhibition.
    if (interruptDefer && level != 0)
    {
        interruptDefer = false;
        return false;
    }

    int mask = ST & 0x000F;

    // Interrupt recognized if level <= mask (lower number = higher priority)
    // Level 0 (RESET) is always recognized
    if (level > mask && level != 0)
    {
        return false;  // Not enabled
    }

    // Clear pending interrupt
    pendingInterrupt = -1;

    // RESET clears ST before it is saved in the new workspace.
    if (level == 0)
    {
        ST = 0;
    }

    // Vector address = level * 4
    UINT16 vectorAddr = level * 4;
    ContextSwitch(vectorAddr);

    // Set new interrupt mask = level - 1 (level 0 stays 0)
    ST = (ST & 0xFFF0) | ((level > 0) ? (level - 1) : 0);

    // Defer interrupts for one instruction
    interruptDefer = true;

    // Wake up from IDLE if halted
    cpuHalted = false;

    // RESET function: 26 clocks, 5 accesses. Other interrupt entries:
    // 22 clocks, 5 accesses.
    ClockCycleCounter += (level == 0) ? 26 : 22;
    ClockCycleCounter += static_cast<UINT64>(memoryWaitStates) * 5;

    return true;
}

//----------------------------------------------------------------------------
// Status flag helpers
//----------------------------------------------------------------------------

static void SetFlags_LAE(UINT16 val)
{
    if ((INT16)val > 0)
    {
        ST |= TMS_LOGICAL | TMS_ARITHMETIC;
    }
    else if ((INT16)val < 0)
    {
        ST |= TMS_LOGICAL;
    }
    else
    {
        ST |= TMS_EQUAL;
    }
}

static void SetFlags_LAE(UINT16 val1, UINT16 val2)
{
    if (val1 == val2)
    {
        ST |= TMS_EQUAL;
    }
    else
    {
        if ((INT16)val1 > (INT16)val2)
        {
            ST |= TMS_ARITHMETIC;
        }
        if (val1 > val2)
        {
            ST |= TMS_LOGICAL;
        }
    }
}

static void SetFlags_difW(UINT16 val1, UINT16 val2, UINT32 res)
{
    if (!(res & 0x00010000))
    {
        ST |= TMS_CARRY;
    }
    if ((val1 ^ val2) & (val2 ^ res) & 0x8000)
    {
        ST |= TMS_OVERFLOW;
    }
    SetFlags_LAE((UINT16)res);
}

static void SetFlags_difB(UINT8 val1, UINT8 val2, UINT32 res)
{
    if (!(res & 0x0100))
    {
        ST |= TMS_CARRY;
    }
    if ((val1 ^ val2) & (val2 ^ res) & 0x80)
    {
        ST |= TMS_OVERFLOW;
    }
    SetFlags_LAE((INT8)res);
    ST |= parity[(UINT8)res];
}

static void SetFlags_sumW(UINT16 val1, UINT16 val2, UINT32 res)
{
    if (res & 0x00010000)
    {
        ST |= TMS_CARRY;
    }
    if ((res ^ val1) & (res ^ val2) & 0x8000)
    {
        ST |= TMS_OVERFLOW;
    }
    SetFlags_LAE((UINT16)res);
}

static void SetFlags_sumB(UINT8 val1, UINT8 val2, UINT32 res)
{
    if (res & 0x0100)
    {
        ST |= TMS_CARRY;
    }
    if ((res ^ val1) & (res ^ val2) & 0x80)
    {
        ST |= TMS_OVERFLOW;
    }
    SetFlags_LAE((INT8)res);
    ST |= parity[(UINT8)res];
}

//----------------------------------------------------------------------------
// Invalid opcode handler
//----------------------------------------------------------------------------

static void InvalidOpcode()
{
    // Just consume cycles, don't crash
}

//----------------------------------------------------------------------------
// Opcode implementations
//----------------------------------------------------------------------------

static void opcode_LI()
{
    UINT16 value = Fetch();

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
    SetFlags_LAE(value);

    WriteMemoryW(WP + 2 * (curOpCode & 0x000F), value);
}

static void opcode_AI()
{
    int reg = curOpCode & 0x000F;

    UINT32 src = ReadMemoryW(WP + 2 * reg);
    UINT32 dst = Fetch();
    UINT32 sum = src + dst;

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY | TMS_OVERFLOW);
    SetFlags_sumW((UINT16)src, (UINT16)dst, sum);

    WriteMemoryW(WP + 2 * reg, (UINT16)sum);
}

static void opcode_ANDI()
{
    int reg = (UINT16)(curOpCode & 0x000F);
    UINT16 value = ReadMemoryW(WP + 2 * reg);
    value &= Fetch();

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
    SetFlags_LAE(value);

    WriteMemoryW(WP + 2 * reg, value);
}

static void opcode_ORI()
{
    int reg = (UINT16)(curOpCode & 0x000F);
    UINT16 value = ReadMemoryW(WP + 2 * reg);
    value |= Fetch();

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
    SetFlags_LAE(value);

    WriteMemoryW(WP + 2 * reg, value);
}

static void opcode_CI()
{
    UINT16 src = ReadMemoryW(WP + 2 * (curOpCode & 0x000F));
    UINT16 dst = Fetch();

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
    SetFlags_LAE(src, dst);
}

static void opcode_STWP()
{
    WriteMemoryW(WP + 2 * (curOpCode & 0x000F), WP);
}

static void opcode_STST()
{
    WriteMemoryW(WP + 2 * (curOpCode & 0x000F), ST);
}

static void opcode_LWPI()
{
    WP = Fetch() & 0xFFFE;
}

static void opcode_LIMI()
{
    ST = (UINT16)((ST & 0xFFF0) | (Fetch() & 0x0F));
}

static void opcode_IDLE()
{
    // Remain halted until an enabled interrupt is accepted.
    cpuHalted = true;
}

static void opcode_RSET()
{
    ST &= 0xFFF0;
}

static void opcode_RTWP()
{
    UINT16 restoredST = ReadMemoryW(WP + 2 * 15);
    UINT16 restoredPC = ReadMemoryW(WP + 2 * 14);
    UINT16 restoredWP = ReadMemoryW(WP + 2 * 13);

    ST = restoredST;
    PC = restoredPC & 0xFFFE;
    WP = restoredWP & 0xFFFE;
}

static void opcode_CKON()
{
    // No-op in bare-metal mode
}

static void opcode_CKOF()
{
    // No-op in bare-metal mode
}

static void opcode_LREX()
{
    // No-op in bare-metal mode
}

static void opcode_BLWP()
{
    UINT16 address = GetAddress(curOpCode, 2);
    ContextSwitch(address);
    interruptDefer = true;
}

static void opcode_B()
{
    PC = GetAddress(curOpCode, 2);
    PC &= 0xFFFE;
}

static void opcode_X()
{
    curOpCode = ReadMemoryW(GetAddress(curOpCode, 2));
    sOpCode *op = LookupOpCode(curOpCode);
    AddInstructionTiming(op->clocks - 4,
                         op->memoryAccesses ? op->memoryAccesses - 1 : 0);
    ((void (*)())op->function)();
}

static void opcode_CLR()
{
    UINT16 address = GetAddress(curOpCode, 2);
    ReadMemoryW(address);  // Hidden memory access
    WriteMemoryW(address, (UINT16)0);
}

static void opcode_NEG()
{
    UINT16 address = GetAddress(curOpCode, 2);
    UINT32 src = ReadMemoryW(address);

    UINT32 dst = 0 - src;

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY | TMS_OVERFLOW);
    SetFlags_LAE((UINT16)dst);
    if (src == 0x8000)
    {
        ST |= TMS_OVERFLOW;
    }
    if (src == 0x0000)
    {
        ST |= TMS_CARRY;
    }

    WriteMemoryW(address, (UINT16)dst);
}

static void opcode_INV()
{
    UINT16 address = GetAddress(curOpCode, 2);
    UINT16 value = ~ReadMemoryW(address);

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
    SetFlags_LAE(value);

    WriteMemoryW(address, value);
}

static void opcode_INC()
{
    UINT16 address = GetAddress(curOpCode, 2);
    UINT32 src = ReadMemoryW(address);

    UINT32 sum = src + 1;

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY | TMS_OVERFLOW);
    SetFlags_sumW((UINT16)src, 1, sum);

    WriteMemoryW(address, (UINT16)sum);
}

static void opcode_INCT()
{
    UINT16 address = GetAddress(curOpCode, 2);
    UINT32 src = ReadMemoryW(address);

    UINT32 sum = src + 2;

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY | TMS_OVERFLOW);
    SetFlags_sumW((UINT16)src, 2, sum);

    WriteMemoryW(address, (UINT16)sum);
}

static void opcode_DEC()
{
    UINT16 address = GetAddress(curOpCode, 2);
    UINT32 src = ReadMemoryW(address);

    UINT32 dif = src - 1;

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY | TMS_OVERFLOW);
    SetFlags_difW(1, (UINT16)src, dif);

    WriteMemoryW(address, (UINT16)dif);
}

static void opcode_DECT()
{
    UINT16 address = GetAddress(curOpCode, 2);
    UINT32 src = ReadMemoryW(address);

    UINT32 dif = src - 2;

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY | TMS_OVERFLOW);
    SetFlags_difW(2, (UINT16)src, dif);

    WriteMemoryW(address, (UINT16)dif);
}

static void opcode_BL()
{
    UINT16 address = GetAddress(curOpCode, 2);

    WriteMemoryW(WP + 2 * 11, PC);

    PC = address;
}

static void opcode_SWPB()
{
    UINT16 address = GetAddress(curOpCode, 2);
    UINT16 value = ReadMemoryW(address);

    value = (UINT16)((value << 8) | (value >> 8));

    WriteMemoryW(address, (UINT16)value);
}

static void opcode_SETO()
{
    UINT16 address = GetAddress(curOpCode, 2);
    ReadMemoryW(address);  // Hidden memory access
    WriteMemoryW(address, (UINT16)-1);
}

static void opcode_ABS()
{
    UINT16 address = GetAddress(curOpCode, 2);
    UINT16 dst = ReadMemoryW(address);

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY | TMS_OVERFLOW);
    SetFlags_LAE(dst);

    if (dst & 0x8000)
    {
        AddInstructionTiming(2, 1);
        WriteMemoryW(address, -dst);
        if (dst == 0x8000)
        {
            ST |= TMS_OVERFLOW;
        }
    }
}

static void opcode_SRA()
{
    int reg = curOpCode & 0x000F;
    unsigned int count = (curOpCode >> 4) & 0x000F;
    if (count == 0)
    {
        AddInstructionTiming(8, 1);
        count = ReadMemoryW(WP + 2 * 0) & 0x000F;
        if (count == 0)
        {
            count = 16;
        }
    }

    AddInstructionTiming(2 * count);

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY);

    INT16 value = (INT16)(((INT16)ReadMemoryW(WP + 2 * reg)) >> --count);
    if (value & 1)
    {
        ST |= TMS_CARRY;
    }
    value >>= 1;
    SetFlags_LAE(value);

    WriteMemoryW(WP + 2 * reg, (UINT16)value);
}

static void opcode_SRL()
{
    int reg = curOpCode & 0x000F;
    unsigned int count = (curOpCode >> 4) & 0x000F;
    if (count == 0)
    {
        AddInstructionTiming(8, 1);
        count = ReadMemoryW(WP + 2 * 0) & 0x000F;
        if (count == 0)
        {
            count = 16;
        }
    }

    AddInstructionTiming(2 * count);

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY);

    UINT16 value = (UINT16)(ReadMemoryW(WP + 2 * reg) >> --count);
    if (value & 1)
    {
        ST |= TMS_CARRY;
    }
    value >>= 1;
    SetFlags_LAE(value);

    WriteMemoryW(WP + 2 * reg, value);
}

static void opcode_SLA()
{
    int reg = curOpCode & 0x000F;
    unsigned int count = (curOpCode >> 4) & 0x000F;
    if (count == 0)
    {
        AddInstructionTiming(8, 1);
        count = ReadMemoryW(WP + 2 * 0) & 0x000F;
        if (count == 0)
        {
            count = 16;
        }
    }

    AddInstructionTiming(2 * count);

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY | TMS_OVERFLOW);

    UINT32 value = static_cast<UINT32>(ReadMemoryW(WP + 2 * reg)) << count;

    UINT32 mask = (static_cast<UINT32>(0xFFFF) << count) & 0xFFFF8000U;
    UINT32 bits = value & mask;

    if (value & 0x00010000)
    {
        ST |= TMS_CARRY;
    }
    if (bits && ((bits ^ mask) || (count == 16)))
    {
        ST |= TMS_OVERFLOW;
    }
    SetFlags_LAE((UINT16)value);

    WriteMemoryW(WP + 2 * reg, (UINT16)value);
}

static void opcode_SRC()
{
    int reg = curOpCode & 0x000F;
    unsigned int count = (curOpCode >> 4) & 0x000F;
    if (count == 0)
    {
        AddInstructionTiming(8, 1);
        count = ReadMemoryW(WP + 2 * 0) & 0x000F;
        if (count == 0)
        {
            count = 16;
        }
    }

    AddInstructionTiming(2 * count);

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY);

    UINT32 value = ReadMemoryW(WP + 2 * reg);
    value = ((value << 16) | value) >> count;
    if (value & 0x8000)
    {
        ST |= TMS_CARRY;
    }
    SetFlags_LAE((UINT16)value);

    WriteMemoryW(WP + 2 * reg, (UINT16)value);
}

static void opcode_JMP()
{
    AddInstructionTiming(2);
    PC += 2 * (INT8)curOpCode;
}

static void opcode_JLT()
{
    if (!(ST & (TMS_ARITHMETIC | TMS_EQUAL)))
    {
        opcode_JMP();
    }
}

static void opcode_JLE()
{
    if ((!(ST & TMS_LOGICAL)) | (ST & TMS_EQUAL))
    {
        opcode_JMP();
    }
}

static void opcode_JEQ()
{
    if (ST & TMS_EQUAL)
    {
        opcode_JMP();
    }
}

static void opcode_JHE()
{
    if (ST & (TMS_LOGICAL | TMS_EQUAL))
    {
        opcode_JMP();
    }
}

static void opcode_JGT()
{
    if (ST & TMS_ARITHMETIC)
    {
        opcode_JMP();
    }
}

static void opcode_JNE()
{
    if (!(ST & TMS_EQUAL))
    {
        opcode_JMP();
    }
}

static void opcode_JNC()
{
    if (!(ST & TMS_CARRY))
    {
        opcode_JMP();
    }
}

static void opcode_JOC()
{
    if (ST & TMS_CARRY)
    {
        opcode_JMP();
    }
}

static void opcode_JNO()
{
    if (!(ST & TMS_OVERFLOW))
    {
        opcode_JMP();
    }
}

static void opcode_JL()
{
    if (!(ST & (TMS_LOGICAL | TMS_EQUAL)))
    {
        opcode_JMP();
    }
}

static void opcode_JH()
{
    if ((ST & TMS_LOGICAL) && !(ST & TMS_EQUAL))
    {
        opcode_JMP();
    }
}

static void opcode_JOP()
{
    if (ST & TMS_PARITY)
    {
        opcode_JMP();
    }
}

// CRU instructions - no-op in bare-metal mode
static void opcode_SBO()
{
    // No CRU hardware
}

static void opcode_SBZ()
{
    // No CRU hardware
}

static void opcode_TB()
{
    ST &= ~TMS_EQUAL;  // Always returns 0 (bit not set)
}

static void opcode_COC()
{
    UINT16 src = ReadMemoryW(WP + 2 * ((curOpCode >> 6) & 0x000F));
    UINT16 dst = ReadMemoryW(GetAddress(curOpCode, 2));
    if ((src & dst) == dst)
    {
        ST |= TMS_EQUAL;
    }
    else
    {
        ST &= ~TMS_EQUAL;
    }
}

static void opcode_CZC()
{
    UINT16 src = ReadMemoryW(WP + 2 * ((curOpCode >> 6) & 0x000F));
    UINT16 dst = ReadMemoryW(GetAddress(curOpCode, 2));
    if ((~src & dst) == dst)
    {
        ST |= TMS_EQUAL;
    }
    else
    {
        ST &= ~TMS_EQUAL;
    }
}

static void opcode_XOR()
{
    int reg = (curOpCode >> 6) & 0x000F;
    UINT16 address = GetAddress(curOpCode, 2);
    UINT16 value = ReadMemoryW(WP + 2 * reg);
    value ^= ReadMemoryW(address);

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
    SetFlags_LAE(value);

    WriteMemoryW(WP + 2 * reg, value);
}

static void opcode_XOP()
{
    UINT16 address = GetAddress(curOpCode, 2);
    int level = ((curOpCode >> 4) & 0x003C) + 64;
    ContextSwitch(level);
    WriteMemoryW(WP + 2 * 11, address);
    ST |= TMS_XOP;
    interruptDefer = true;
}

// CRU instructions - no-op/stub in bare-metal mode
static void opcode_LDCR()
{
    unsigned int count = (curOpCode >> 6) & 0x000F;
    if (count == 0)
    {
        count = 16;
    }

    AddInstructionTiming(2 * count);

    // LDCR affects LGT, AGT, and EQ. Byte transfers also affect parity;
    // carry and overflow are preserved.
    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);

    if (count < 9)
    {
        ST &= ~TMS_PARITY;
        UINT16 address = GetAddress(curOpCode, 1);
        UINT16 value = ReadMemoryB(address);
        ST |= parity[(UINT8)value];
        SetFlags_LAE((INT8)value);
    }
    else
    {
        UINT16 address = GetAddress(curOpCode, 2);
        UINT16 value = ReadMemoryW(address);
        SetFlags_LAE(value);
    }
    // CRU write is a no-op
}

static void opcode_STCR()
{
    unsigned int count = (curOpCode >> 6) & 0x000F;
    if (count == 0)
    {
        count = 16;
    }

    // STCR has the same status effects as LDCR.
    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);

    AddInstructionTiming((count & 0x07) ? 0 : 2);

    // CRU read returns 0
    UINT16 value = 0;

    if (count < 9)
    {
        ST &= ~TMS_PARITY;
        ST |= parity[(UINT8)value];
        SetFlags_LAE((INT8)value);
        UINT16 address = GetAddress(curOpCode, 1);
        ReadMemoryB(address);  // Hidden memory access
        WriteMemoryB(address, (UINT8)value);
    }
    else
    {
        AddInstructionTiming(58 - 42);
        SetFlags_LAE(value);
        UINT16 address = GetAddress(curOpCode, 2);
        ReadMemoryW(address);  // Hidden memory access
        WriteMemoryW(address, value);
    }
}

static void opcode_MPY()
{
    UINT16 srcAddress = GetAddress(curOpCode, 2);
    UINT32 src = ReadMemoryW(srcAddress);
    UINT16 dstAddress = GetAddress((curOpCode >> 6) & 0x0F, 2);
    UINT32 dst = ReadMemoryW(dstAddress);

    dst *= src;

    WriteMemoryW(dstAddress, (UINT16)(dst >> 16));
    WriteMemoryW(dstAddress + 2, (UINT16)dst);
}

static void opcode_DIV()
{
    UINT16 srcAddress = GetAddress(curOpCode, 2);
    UINT32 src = ReadMemoryW(srcAddress);
    UINT16 dstAddress = GetAddress((curOpCode >> 6) & 0x0F, 2);
    UINT32 dst = ReadMemoryW(dstAddress);

    if (dst < src)
    {
        ST &= ~TMS_OVERFLOW;
        dst = (dst << 16) | ReadMemoryW(dstAddress + 2);
        const UINT16 quotient = static_cast<UINT16>(dst / src);
        const UINT16 remainder = static_cast<UINT16>(dst % src);
        WriteMemoryW(dstAddress, quotient);
        WriteMemoryW(dstAddress + 2, remainder);

        // TI documents 92-124 clocks depending on the partial quotient but
        // does not publish the divider microcode. Match MAME's bounded model:
        // 92 clocks plus two for each significant bit in the remainder.
        UINT16 value = remainder;
        UINT32 significantBits = 0;
        while (value != 0)
        {
            value >>= 1;
            significantBits++;
        }
        AddInstructionTiming(92 + 2 * significantBits - 16, 3);
    }
    else
    {
        ST |= TMS_OVERFLOW;
    }
}

static void opcode_SZC()
{
    UINT16 srcAddress = GetAddress(curOpCode, 2);
    UINT16 src = ReadMemoryW(srcAddress);
    UINT16 dstAddress = GetAddress(curOpCode >> 6, 2);
    UINT16 dst = ReadMemoryW(dstAddress);

    src = ~src & dst;

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
    SetFlags_LAE(src);

    WriteMemoryW(dstAddress, src);
}

static void opcode_SZCB()
{
    UINT16 srcAddress = GetAddress(curOpCode, 1);
    UINT8 src = ReadMemoryB(srcAddress);
    UINT16 dstAddress = GetAddress(curOpCode >> 6, 1);
    UINT8 dst = ReadMemoryB(dstAddress);

    src = ~src & dst;

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_PARITY);
    ST |= parity[src];
    SetFlags_LAE((INT8)src);

    WriteMemoryB(dstAddress, src);
}

static void opcode_S()
{
    UINT16 srcAddress = GetAddress(curOpCode, 2);
    UINT32 src = ReadMemoryW(srcAddress);
    UINT16 dstAddress = GetAddress(curOpCode >> 6, 2);
    UINT32 dst = ReadMemoryW(dstAddress);

    UINT32 sum = dst - src;

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY | TMS_OVERFLOW);
    SetFlags_difW((UINT16)src, (UINT16)dst, sum);

    WriteMemoryW(dstAddress, (UINT16)sum);
}

static void opcode_SB()
{
    UINT16 srcAddress = GetAddress(curOpCode, 1);
    UINT32 src = ReadMemoryB(srcAddress);
    UINT16 dstAddress = GetAddress(curOpCode >> 6, 1);
    UINT32 dst = ReadMemoryB(dstAddress);

    UINT32 sum = dst - src;

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY | TMS_OVERFLOW | TMS_PARITY);
    SetFlags_difB((UINT8)src, (UINT8)dst, sum);

    WriteMemoryB(dstAddress, (UINT8)sum);
}

static void opcode_C()
{
    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);

    UINT16 src = ReadMemoryW(GetAddress(curOpCode, 2));
    UINT16 dst = ReadMemoryW(GetAddress(curOpCode >> 6, 2));

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
    SetFlags_LAE(src, dst);
}

static void opcode_CB()
{
    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_PARITY);

    UINT8 src = ReadMemoryB(GetAddress(curOpCode, 1));
    UINT8 dst = ReadMemoryB(GetAddress(curOpCode >> 6, 1));

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_PARITY);
    ST |= parity[src];
    SetFlags_LAE((INT8)src, (INT8)dst);
}

static void opcode_A()
{
    UINT16 srcAddress = GetAddress(curOpCode, 2);
    UINT32 src = ReadMemoryW(srcAddress);
    UINT16 dstAddress = GetAddress(curOpCode >> 6, 2);
    UINT32 dst = ReadMemoryW(dstAddress);

    UINT32 sum = src + dst;

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY | TMS_OVERFLOW);
    SetFlags_sumW((UINT16)src, (UINT16)dst, sum);

    WriteMemoryW(dstAddress, (UINT16)sum);
}

static void opcode_AB()
{
    UINT16 srcAddress = GetAddress(curOpCode, 1);
    UINT32 src = ReadMemoryB(srcAddress);
    UINT16 dstAddress = GetAddress(curOpCode >> 6, 1);
    UINT32 dst = ReadMemoryB(dstAddress);

    UINT32 sum = src + dst;

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY | TMS_OVERFLOW | TMS_PARITY);
    ST |= parity[(UINT8)sum];
    SetFlags_sumB((UINT8)src, (UINT8)dst, sum);

    WriteMemoryB(dstAddress, (UINT8)sum);
}

static void opcode_MOV()
{
    UINT16 srcAddress = GetAddress(curOpCode, 2);
    UINT16 src = ReadMemoryW(srcAddress);
    UINT16 dstAddress = GetAddress(curOpCode >> 6, 2);

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
    SetFlags_LAE(src);

    ReadMemoryW(dstAddress);  // Hidden memory access

    WriteMemoryW(dstAddress, src);
}

static void opcode_MOVB()
{
    UINT16 srcAddress = GetAddress(curOpCode, 1);
    UINT8 src = ReadMemoryB(srcAddress);
    UINT16 dstAddress = GetAddress(curOpCode >> 6, 1);

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_PARITY);
    ST |= parity[src];
    SetFlags_LAE((INT8)src);

    ReadMemoryB(dstAddress);  // Hidden memory access

    WriteMemoryB(dstAddress, src);
}

static void opcode_SOC()
{
    UINT16 srcAddress = GetAddress(curOpCode, 2);
    UINT16 src = ReadMemoryW(srcAddress);
    UINT16 dstAddress = GetAddress(curOpCode >> 6, 2);
    UINT16 dst = ReadMemoryW(dstAddress);

    src = src | dst;

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
    SetFlags_LAE(src);

    WriteMemoryW(dstAddress, src);
}

static void opcode_SOCB()
{
    UINT16 srcAddress = GetAddress(curOpCode, 1);
    UINT8 src = ReadMemoryB(srcAddress);
    UINT16 dstAddress = GetAddress(curOpCode >> 6, 1);
    UINT8 dst = ReadMemoryB(dstAddress);

    src = src | dst;

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_PARITY);
    ST |= parity[src];
    SetFlags_LAE((INT8)src);

    WriteMemoryB(dstAddress, src);
}

//----------------------------------------------------------------------------
// Public API
//----------------------------------------------------------------------------

void InitCPU()
{
    if (!lookupInitialized)
    {
        InitOpCodeLookup();
        lookupInitialized = true;
    }
    ClearMemory();
    ResetCPU();
}

void ClearMemory()
{
    memset(Memory, 0, sizeof(Memory));
}

void ResetCPU()
{
    WorkspacePtr = 0;
    ProgramCounter = 0;
    fetchPtr = 0;
    Status = 0;
    ClockCycleCounter = 0;
    InstructionCounter = 0;
    instructionCycles = 0;
    instructionMemoryAccesses = 0;
    cpuHalted = false;
    lastOpcode = nullptr;
    pendingInterrupt = -1;
    interruptDefer = false;
}

void SetProgramCounter(UINT16 address)
{
    ProgramCounter = address & 0xFFFE;
    fetchPtr = ProgramCounter;
}

void SetWorkspacePointer(UINT16 address)
{
    WorkspacePtr = address & 0xFFFE;
}

void SetMemoryWaitStates(UINT32 waitStates)
{
    memoryWaitStates = waitStates;
}

UINT32 GetMemoryWaitStates()
{
    return memoryWaitStates;
}

sOpCode* GetLastOpcode()
{
    return lastOpcode;
}

CPUStepResult StepCPU()
{
    // External clients may write ProgramCounter directly, so normalize and
    // synchronize the private fetch pointer before interrupt recognition.
    SetProgramCounter(ProgramCounter);

    // Check for pending interrupt before executing instruction
    if (CheckAndHandleInterrupt())
    {
        // Interrupt was handled, don't count as instruction
        // The next StepCPU will execute the first instruction of the ISR
        return { CPUEvent::Interrupt, false, nullptr };
    }

    if (cpuHalted)
    {
        return { CPUEvent::Idle, true, nullptr };
    }

    curOpCode = Fetch();
    sOpCode *op = LookupOpCode(curOpCode);
    lastOpcode = op;

    instructionCycles = op->clocks;
    instructionMemoryAccesses = op->memoryAccesses;
    ((void (*)())op->function)();
    CommitInstructionTiming();

    InstructionCounter++;
    SetProgramCounter(fetchPtr);

    return { CPUEvent::Instruction, cpuHalted, op };
}

void TriggerInterrupt(int level)
{
    if (level >= 0 && level <= 15)
    {
        // If a higher priority interrupt is pending, keep it
        if (pendingInterrupt < 0 || level < pendingInterrupt)
        {
            pendingInterrupt = level;
        }
    }
}

bool IsHalted()
{
    return cpuHalted;
}
