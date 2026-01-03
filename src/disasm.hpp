//----------------------------------------------------------------------------
// TMS9900 Trace - Standalone TMS9900 CPU Simulator
//
// disasm.hpp - TMS9900 Disassembler interface
//
// Adapted from ti99sim by Marc Rousseau
// Original: Copyright (c) 1998-2004 Marc Rousseau, All Rights Reserved.
// Licensed under GPL v2
//----------------------------------------------------------------------------

#ifndef TMS9900_DISASM_HPP_
#define TMS9900_DISASM_HPP_

#include "types.hpp"

// Disassemble instruction at address
// Returns the address of the next instruction
// Buffer should be at least 40 bytes
UINT16 DisassembleASM(UINT16 address, const UINT8* memory, char* buffer);

#endif // TMS9900_DISASM_HPP_
