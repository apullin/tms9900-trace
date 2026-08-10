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

#include <cstddef>

// Disassemble one instruction from the simulator's flat memory. Reads wrap at
// the 16-bit address boundary. Returns the next instruction address.
UINT16 DisassembleASM(UINT16 address, char* buffer, size_t bufferSize);

#endif // TMS9900_DISASM_HPP_
