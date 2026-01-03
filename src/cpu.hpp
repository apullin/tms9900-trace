//----------------------------------------------------------------------------
// TMS9900 Trace - Standalone TMS9900 CPU Simulator
//
// cpu.hpp - TMS9900 CPU emulation interface
//
// Adapted from ti99sim by Marc Rousseau
// Original: Copyright (c) 1998-2004 Marc Rousseau, All Rights Reserved.
// Licensed under GPL v2
//----------------------------------------------------------------------------

#ifndef TMS9900_CPU_HPP_
#define TMS9900_CPU_HPP_

#include "types.hpp"

//----------------------------------------------------------------------------
// Opcode structure
//----------------------------------------------------------------------------

struct sOpCode
{
    char        mnemonic[5];
    UINT16      opCode;
    UINT16      mask;
    UINT16      format;
    void      (*function)();
    UINT32      clocks;
};

//----------------------------------------------------------------------------
// CPU State (exposed for tracing)
//----------------------------------------------------------------------------

extern UINT16 WorkspacePtr;      // WP - Workspace Pointer
extern UINT16 ProgramCounter;    // PC - Program Counter
extern UINT16 Status;            // ST - Status Register
extern UINT32 ClockCycleCounter; // Clock cycles
extern UINT32 InstructionCounter;// Instructions executed

//----------------------------------------------------------------------------
// Memory (64K flat)
//----------------------------------------------------------------------------

extern UINT8 Memory[0x10000];

// Memory access functions
UINT8  ReadByte(UINT16 address);
UINT16 ReadWord(UINT16 address);
void   WriteByte(UINT16 address, UINT8 value);
void   WriteWord(UINT16 address, UINT16 value);

//----------------------------------------------------------------------------
// CPU Control
//----------------------------------------------------------------------------

// Initialize the CPU (call once at startup)
void InitCPU();

// Reset the CPU state
void ResetCPU();

// Execute one instruction, returns true if CPU halted (IDLE)
bool StepCPU();

// Check if CPU is halted (IDLE with no pending interrupt)
bool IsHalted();

// Trigger an interrupt at the specified level (0-15)
// Level 0 = highest priority (RESET), Level 15 = lowest
// Interrupt will be processed on next StepCPU if enabled
void TriggerInterrupt(int level);

// Get the current opcode info (after StepCPU)
sOpCode* GetLastOpcode();

// Lookup opcode by value
sOpCode* LookupOpCode(UINT16 opcode);

// Disassemble instruction at address, returns next address
UINT16 Disassemble(UINT16 address, char* buffer, size_t bufferSize);

//----------------------------------------------------------------------------
// Register access helpers
//----------------------------------------------------------------------------

// Read a workspace register (R0-R15)
inline UINT16 GetRegister(int reg)
{
    return ReadWord(WorkspacePtr + reg * 2);
}

// Write a workspace register
inline void SetRegister(int reg, UINT16 value)
{
    WriteWord(WorkspacePtr + reg * 2, value);
}

#endif // TMS9900_CPU_HPP_
