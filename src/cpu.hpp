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
    UINT32      memoryAccesses;
};

//----------------------------------------------------------------------------
// CPU State (exposed for tracing)
//----------------------------------------------------------------------------

extern UINT16 WorkspacePtr;      // WP - Workspace Pointer
extern UINT16 ProgramCounter;    // PC - Program Counter
extern UINT16 Status;            // ST - Status Register
extern UINT64 ClockCycleCounter; // Clock cycles
extern UINT64 InstructionCounter;// Instructions executed

//----------------------------------------------------------------------------
// Memory (64K flat)
//----------------------------------------------------------------------------

extern UINT8 Memory[0x10000];

// Timing-neutral memory inspection functions
UINT8  ReadByte(UINT16 address);
UINT16 ReadWord(UINT16 address);
void   WriteByte(UINT16 address, UINT8 value);
void   WriteWord(UINT16 address, UINT16 value);

//----------------------------------------------------------------------------
// CPU Control
//----------------------------------------------------------------------------

enum class CPUEvent
{
    Instruction,
    Interrupt,
    Idle
};

struct CPUStepResult
{
    CPUEvent event;
    bool halted;
    sOpCode* opcode;
};

// Initialize the CPU (call once at startup)
void InitCPU();

// Clear the flat 64K memory image.
void ClearMemory();

// Reset CPU state without altering memory.
void ResetCPU();

// Architectural PC and WP are always word-aligned.
void SetProgramCounter(UINT16 address);
void SetWorkspacePointer(UINT16 address);

// Add this many wait-state clocks to every modeled memory access.
void SetMemoryWaitStates(UINT32 waitStates);
UINT32 GetMemoryWaitStates();

// Advance to the next CPU event. Interrupt entry is reported separately and
// does not increment InstructionCounter.
CPUStepResult StepCPU();

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
