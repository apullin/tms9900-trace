//----------------------------------------------------------------------------
// TMS9900 Trace - Standalone TMS9900 CPU Simulator
//
// types.hpp - Basic type definitions
//
// Adapted from ti99sim by Marc Rousseau
// Original: Copyright (c) 1994-2004 Marc Rousseau, All Rights Reserved.
// Licensed under GPL v2
//----------------------------------------------------------------------------

#ifndef TMS9900_TYPES_HPP_
#define TMS9900_TYPES_HPP_

#include <cstdint>
#include <cstddef>

// Basic integer types
using INT8   = int8_t;
using INT16  = int16_t;
using INT32  = int32_t;
using INT64  = int64_t;
using UINT8  = uint8_t;
using UINT16 = uint16_t;
using UINT32 = uint32_t;
using UINT64 = uint64_t;

// Address type
using ADDRESS = UINT16;

// Utility macros
#define EVER ;;

template <typename T, size_t N>
constexpr size_t SIZE(T(&)[N]) { return N; }

// TMS9900 Status Register Flags
constexpr int TMS_LOGICAL   = 0x8000;  // L>  - Logical greater than
constexpr int TMS_ARITHMETIC = 0x4000; // A>  - Arithmetic greater than
constexpr int TMS_EQUAL     = 0x2000;  // EQ  - Equal
constexpr int TMS_CARRY     = 0x1000;  // C   - Carry
constexpr int TMS_OVERFLOW  = 0x0800;  // OV  - Overflow
constexpr int TMS_PARITY    = 0x0400;  // OP  - Odd parity
constexpr int TMS_XOP       = 0x0200;  // X   - XOP in progress

#endif // TMS9900_TYPES_HPP_
