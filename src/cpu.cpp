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
// - CRU instructions (SBO, SBZ, TB, LDCR, STCR) are no-ops
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
UINT32 ClockCycleCounter;        // Clock cycles
UINT32 InstructionCounter;       // Instructions executed

static UINT16  fetchPtr;         // Current fetch pointer
static UINT16  curOpCode;        // Current opcode being executed
static UINT16  parity[256];      // Parity lookup table
static bool    cpuHalted;        // IDLE instruction executed
static sOpCode* lastOpcode;      // Last executed opcode
static int     pendingInterrupt; // Pending interrupt level (-1 = none)
static bool    interruptDefer;   // Defer interrupts for one instruction

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

static sOpCode InvalidOp = { "INVL", 0x0000, 0x0000, 0, InvalidOpcode, 6 };

static sOpCode OpCodes[69] =
{
    { "A",    0xA000, 0xF000, 1, opcode_A,    14 },
    { "AB",   0xB000, 0xF000, 1, opcode_AB,   14 },
    { "ABS",  0x0740, 0xFFC0, 6, opcode_ABS,  12 },
    { "AI",   0x0220, 0xFFE0, 8, opcode_AI,   14 },
    { "ANDI", 0x0240, 0xFFE0, 8, opcode_ANDI, 14 },
    { "B",    0x0440, 0xFFC0, 6, opcode_B,     8 },
    { "BL",   0x0680, 0xFFC0, 6, opcode_BL,   12 },
    { "BLWP", 0x0400, 0xFFC0, 6, opcode_BLWP, 26 },
    { "C",    0x8000, 0xF000, 1, opcode_C,    14 },
    { "CB",   0x9000, 0xF000, 1, opcode_CB,   14 },
    { "CI",   0x0280, 0xFFE0, 8, opcode_CI,   14 },
    { "CKOF", 0x03C0, 0xFFFF, 7, opcode_CKOF, 12 },
    { "CKON", 0x03A0, 0xFFFF, 7, opcode_CKON, 12 },
    { "CLR",  0x04C0, 0xFFC0, 6, opcode_CLR,  10 },
    { "COC",  0x2000, 0xFC00, 3, opcode_COC,  14 },
    { "CZC",  0x2400, 0xFC00, 3, opcode_CZC,  14 },
    { "DEC",  0x0600, 0xFFC0, 6, opcode_DEC,  10 },
    { "DECT", 0x0640, 0xFFC0, 6, opcode_DECT, 10 },
    { "DIV",  0x3C00, 0xFC00, 9, opcode_DIV,  16 },
    { "IDLE", 0x0340, 0xFFFF, 7, opcode_IDLE, 12 },
    { "INC",  0x0580, 0xFFC0, 6, opcode_INC,  10 },
    { "INCT", 0x05C0, 0xFFC0, 6, opcode_INCT, 10 },
    { "INV",  0x0540, 0xFFC0, 6, opcode_INV,  10 },
    { "JEQ",  0x1300, 0xFF00, 2, opcode_JEQ,   8 },
    { "JGT",  0x1500, 0xFF00, 2, opcode_JGT,   8 },
    { "JH",   0x1B00, 0xFF00, 2, opcode_JH,    8 },
    { "JHE",  0x1400, 0xFF00, 2, opcode_JHE,   8 },
    { "JL",   0x1A00, 0xFF00, 2, opcode_JL,    8 },
    { "JLE",  0x1200, 0xFF00, 2, opcode_JLE,   8 },
    { "JLT",  0x1100, 0xFF00, 2, opcode_JLT,   8 },
    { "JMP",  0x1000, 0xFF00, 2, opcode_JMP,   8 },
    { "JNC",  0x1700, 0xFF00, 2, opcode_JNC,   8 },
    { "JNE",  0x1600, 0xFF00, 2, opcode_JNE,   8 },
    { "JNO",  0x1900, 0xFF00, 2, opcode_JNO,   8 },
    { "JOC",  0x1800, 0xFF00, 2, opcode_JOC,   8 },
    { "JOP",  0x1C00, 0xFF00, 2, opcode_JOP,   8 },
    { "LDCR", 0x3000, 0xFC00, 4, opcode_LDCR, 20 },
    { "LI",   0x0200, 0xFFE0, 8, opcode_LI,   12 },
    { "LIMI", 0x0300, 0xFFE0, 8, opcode_LIMI, 16 },
    { "LREX", 0x03E0, 0xFFFF, 7, opcode_LREX, 12 },
    { "LWPI", 0x02E0, 0xFFE0, 8, opcode_LWPI, 10 },
    { "MOV",  0xC000, 0xF000, 1, opcode_MOV,  14 },
    { "MOVB", 0xD000, 0xF000, 1, opcode_MOVB, 14 },
    { "MPY",  0x3800, 0xFC00, 9, opcode_MPY,  52 },
    { "NEG",  0x0500, 0xFFC0, 6, opcode_NEG,  12 },
    { "ORI",  0x0260, 0xFFE0, 8, opcode_ORI,  14 },
    { "RSET", 0x0360, 0xFFFF, 7, opcode_RSET, 12 },
    { "RTWP", 0x0380, 0xFFFF, 7, opcode_RTWP, 14 },
    { "S",    0x6000, 0xF000, 1, opcode_S,    14 },
    { "SB",   0x7000, 0xF000, 1, opcode_SB,   14 },
    { "SBO",  0x1D00, 0xFF00, 2, opcode_SBO,  12 },
    { "SBZ",  0x1E00, 0xFF00, 2, opcode_SBZ,  12 },
    { "SETO", 0x0700, 0xFFC0, 6, opcode_SETO, 10 },
    { "SLA",  0x0A00, 0xFF00, 5, opcode_SLA,  12 },
    { "SOC",  0xE000, 0xF000, 1, opcode_SOC,  14 },
    { "SOCB", 0xF000, 0xF000, 1, opcode_SOCB, 14 },
    { "SRA",  0x0800, 0xFF00, 5, opcode_SRA,  12 },
    { "SRC",  0x0B00, 0xFF00, 5, opcode_SRC,  12 },
    { "SRL",  0x0900, 0xFF00, 5, opcode_SRL,  12 },
    { "STCR", 0x3400, 0xFC00, 4, opcode_STCR, 42 },
    { "STST", 0x02C0, 0xFFE0, 8, opcode_STST,  8 },
    { "STWP", 0x02A0, 0xFFE0, 8, opcode_STWP,  8 },
    { "SWPB", 0x06C0, 0xFFC0, 6, opcode_SWPB, 10 },
    { "SZC",  0x4000, 0xF000, 1, opcode_SZC,  14 },
    { "SZCB", 0x5000, 0xF000, 1, opcode_SZCB, 14 },
    { "TB",   0x1F00, 0xFF00, 2, opcode_TB,   12 },
    { "X",    0x0480, 0xFFC0, 6, opcode_X,     8 },
    { "XOP",  0x2C00, 0xFC00, 9, opcode_XOP,  36 },
    { "XOR",  0x2800, 0xFC00, 3, opcode_XOR,  14 }
};

//----------------------------------------------------------------------------
// Memory access (simplified - no traps, no 8-bit penalty)
//----------------------------------------------------------------------------

UINT8 ReadByte(UINT16 address)
{
    ClockCycleCounter += 2;
    return Memory[address];
}

UINT16 ReadWord(UINT16 address)
{
    address &= 0xFFFE;  // Word align
    ClockCycleCounter += 2;
    return (Memory[address] << 8) | Memory[address + 1];
}

void WriteByte(UINT16 address, UINT8 value)
{
    ClockCycleCounter += 2;
    Memory[address] = value;
}

void WriteWord(UINT16 address, UINT16 value)
{
    address &= 0xFFFE;  // Word align
    ClockCycleCounter += 2;
    Memory[address] = value >> 8;
    Memory[address + 1] = value & 0xFF;
}

// Internal memory access (same as public for now)
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
    WP = newWP;
    PC = newPC;
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
    if (pendingInterrupt < 0 || interruptDefer)
    {
        interruptDefer = false;
        return false;
    }

    int level = pendingInterrupt;
    int mask = ST & 0x000F;

    // Interrupt recognized if level <= mask (lower number = higher priority)
    // Level 0 (RESET) is always recognized
    if (level > mask && level != 0)
    {
        return false;  // Not enabled
    }

    // Clear pending interrupt
    pendingInterrupt = -1;

    // Interrupt context switch takes 22 cycles, 5 memory accesses
    ClockCycleCounter += 22;

    // Vector address = level * 4
    UINT16 vectorAddr = level * 4;
    ContextSwitch(vectorAddr);

    // Set new interrupt mask = level - 1 (level 0 stays 0)
    ST = (ST & 0xFFF0) | ((level > 0) ? (level - 1) : 0);

    // Defer interrupts for one instruction
    interruptDefer = true;

    // Wake up from IDLE if halted
    cpuHalted = false;

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
    WP = Fetch();
}

static void opcode_LIMI()
{
    ST = (UINT16)((ST & 0xFFF0) | (Fetch() & 0x0F));
}

static void opcode_IDLE()
{
    // In bare-metal mode, IDLE halts the CPU since there are no interrupts
    cpuHalted = true;
}

static void opcode_RSET()
{
    ST &= 0xFFF0;
}

static void opcode_RTWP()
{
    ST = ReadMemoryW(WP + 2 * 15);
    PC = ReadMemoryW(WP + 2 * 14);
    WP = ReadMemoryW(WP + 2 * 13);
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
    ClockCycleCounter += op->clocks - 2;
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
        ClockCycleCounter += 2;
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
        ClockCycleCounter += 8;
        count = ReadMemoryW(WP + 2 * 0) & 0x000F;
        if (count == 0)
        {
            count = 16;
        }
    }

    ClockCycleCounter += 2 * count;

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
        ClockCycleCounter += 8;
        count = ReadMemoryW(WP + 2 * 0) & 0x000F;
        if (count == 0)
        {
            count = 16;
        }
    }

    ClockCycleCounter += 2 * count;

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
        ClockCycleCounter += 8;
        count = ReadMemoryW(WP + 2 * 0) & 0x000F;
        if (count == 0)
        {
            count = 16;
        }
    }

    ClockCycleCounter += 2 * count;

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY | TMS_OVERFLOW);

    UINT32 value = ReadMemoryW(WP + 2 * reg) << count;

    UINT32 mask = ((UINT16)-1 << count) & 0xFFFF8000;
    int bits = value & mask;

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
        ClockCycleCounter += 8;
        count = ReadMemoryW(WP + 2 * 0) & 0x000F;
        if (count == 0)
        {
            count = 16;
        }
    }

    ClockCycleCounter += 2 * count;

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_CARRY);

    int value = ReadMemoryW(WP + 2 * reg);
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
    ClockCycleCounter += 2;
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
    ClockCycleCounter += 2;
    // No CRU hardware
}

static void opcode_SBZ()
{
    ClockCycleCounter += 2;
    // No CRU hardware
}

static void opcode_TB()
{
    ClockCycleCounter += 2;
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
}

// CRU instructions - no-op/stub in bare-metal mode
static void opcode_LDCR()
{
    unsigned int count = (curOpCode >> 6) & 0x000F;
    if (count == 0)
    {
        count = 16;
    }

    ClockCycleCounter += 2 * count;

    // Still need to read the value (affects flags)
    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_OVERFLOW | TMS_PARITY);

    if (count < 9)
    {
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

    ST &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_OVERFLOW | TMS_PARITY);

    ClockCycleCounter += (count & 0x07) ? 0 : 2;

    // CRU read returns 0
    UINT16 value = 0;

    if (count < 9)
    {
        ST |= parity[(UINT8)value];
        SetFlags_LAE((INT8)value);
        UINT16 address = GetAddress(curOpCode, 1);
        ReadMemoryB(address);  // Hidden memory access
        WriteMemoryB(address, (UINT8)value);
    }
    else
    {
        ClockCycleCounter += 58 - 42;
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
        WriteMemoryW(dstAddress, (UINT16)(dst / src));
        WriteMemoryW(dstAddress + 2, (UINT16)(dst % src));
        ClockCycleCounter += (92 + 124) / 2 - 16;
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
    InitOpCodeLookup();
    ResetCPU();
}

void ResetCPU()
{
    memset(Memory, 0, sizeof(Memory));
    WorkspacePtr = 0;
    ProgramCounter = 0;
    Status = 0;
    ClockCycleCounter = 0;
    InstructionCounter = 0;
    cpuHalted = false;
    lastOpcode = nullptr;
    pendingInterrupt = -1;
    interruptDefer = false;
}

sOpCode* GetLastOpcode()
{
    return lastOpcode;
}

bool StepCPU()
{
    // Check for pending interrupt before executing instruction
    if (CheckAndHandleInterrupt())
    {
        // Interrupt was handled, don't count as instruction
        // The next StepCPU will execute the first instruction of the ISR
        return false;
    }

    if (cpuHalted)
    {
        return true;
    }

    fetchPtr = ProgramCounter;

    curOpCode = Fetch();
    sOpCode *op = LookupOpCode(curOpCode);
    lastOpcode = op;

    ClockCycleCounter += op->clocks - 2;
    ((void (*)())op->function)();

    InstructionCounter++;
    ProgramCounter = fetchPtr;

    return cpuHalted;
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
