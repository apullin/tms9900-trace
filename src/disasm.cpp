//----------------------------------------------------------------------------
// TMS9900 Trace - Standalone TMS9900 CPU Simulator
//
// disasm.cpp - TMS9900 Disassembler
//
// Adapted from ti99sim by Marc Rousseau
// Original: Copyright (c) 1998-2004 Marc Rousseau, All Rights Reserved.
// Licensed under GPL v2
//----------------------------------------------------------------------------

#include "disasm.hpp"
#include "cpu.hpp"

#include <cstdio>
#include <cstring>

namespace {

class BufferWriter
{
public:
    BufferWriter(char* buffer, size_t size)
        : buffer_(buffer), size_(size), length_(0)
    {
        if (size_ != 0)
        {
            buffer_[0] = '\0';
        }
    }

    void Append(char value)
    {
        if (length_ + 1 < size_)
        {
            buffer_[length_] = value;
        }
        length_++;
        Terminate();
    }

    void Append(const char* value)
    {
        while (*value != '\0')
        {
            Append(*value++);
        }
    }

    void AppendHexByte(UINT8 value)
    {
        static constexpr char Hex[] = "0123456789ABCDEF";
        Append(Hex[value >> 4]);
        Append(Hex[value & 0x0F]);
    }

    void AppendHexWord(UINT16 value)
    {
        AppendHexByte(static_cast<UINT8>(value >> 8));
        AppendHexByte(static_cast<UINT8>(value));
    }

    void AppendUnsigned(unsigned value)
    {
        char digits[12];
        std::snprintf(digits, sizeof(digits), "%u", value);
        Append(digits);
    }

    void PadTo(size_t column)
    {
        while (length_ < column)
        {
            Append(' ');
        }
    }

private:
    void Terminate()
    {
        if (size_ == 0)
        {
            return;
        }
        const size_t terminator = length_ < size_ ? length_ : size_ - 1;
        buffer_[terminator] = '\0';
    }

    char* buffer_;
    size_t size_;
    size_t length_;
};

UINT16 ReadRawWord(UINT16 address)
{
    const UINT16 next = static_cast<UINT16>(address + 1);
    return static_cast<UINT16>((static_cast<UINT16>(Memory[address]) << 8) |
                               Memory[next]);
}

UINT16 FetchExtension(UINT16& nextAddress)
{
    const UINT16 value = ReadRawWord(nextAddress);
    nextAddress = static_cast<UINT16>(nextAddress + 2);
    return value;
}

void AppendRegister(BufferWriter& out, unsigned reg)
{
    out.Append('R');
    out.AppendUnsigned(reg & 0x0F);
}

void AppendGeneralAddress(BufferWriter& out, UINT16 specification,
                          UINT16& nextAddress)
{
    const unsigned reg = specification & 0x0F;
    const unsigned mode = (specification >> 4) & 0x03;

    switch (mode)
    {
        case 0:
            AppendRegister(out, reg);
            break;

        case 1:
            out.Append('*');
            AppendRegister(out, reg);
            break;

        case 2:
        {
            const UINT16 extension = FetchExtension(nextAddress);
            out.Append("@>");
            out.AppendHexWord(extension);
            if (reg != 0)
            {
                out.Append('(');
                AppendRegister(out, reg);
                out.Append(')');
            }
            break;
        }

        case 3:
            out.Append('*');
            AppendRegister(out, reg);
            out.Append('+');
            break;
    }
}

void AppendOperands(BufferWriter& out, const sOpCode& operation,
                    UINT16 opcode, UINT16& nextAddress)
{
    switch (operation.format)
    {
        case 1: // Two general addresses.
            AppendGeneralAddress(out, opcode, nextAddress);
            out.Append(',');
            AppendGeneralAddress(out, static_cast<UINT16>(opcode >> 6),
                                 nextAddress);
            break;

        case 2: // Relative branch or CRU bit displacement.
            out.Append('>');
            if (opcode >= 0x1D00)
            {
                out.AppendHexByte(static_cast<UINT8>(opcode));
            }
            else
            {
                const INT8 displacement = static_cast<INT8>(opcode);
                out.AppendHexWord(static_cast<UINT16>(
                    nextAddress + static_cast<INT16>(displacement) * 2));
            }
            break;

        case 3:
            AppendGeneralAddress(out, opcode, nextAddress);
            out.Append(',');
            AppendRegister(out, (opcode >> 6) & 0x0F);
            break;

        case 4:
            AppendGeneralAddress(out, opcode, nextAddress);
            out.Append(',');
            out.AppendUnsigned(((opcode >> 6) & 0x0F) != 0
                                   ? (opcode >> 6) & 0x0F
                                   : 16);
            break;

        case 5:
            AppendRegister(out, opcode & 0x0F);
            out.Append(',');
            out.AppendUnsigned((opcode >> 4) & 0x0F);
            break;

        case 6:
            AppendGeneralAddress(out, opcode, nextAddress);
            break;

        case 7:
            break;

        case 8:
            if (opcode < 0x02A0)
            {
                AppendRegister(out, opcode & 0x0F);
                out.Append(",>");
                out.AppendHexWord(FetchExtension(nextAddress));
            }
            else if (opcode >= 0x02E0)
            {
                out.Append('>');
                out.AppendHexWord(FetchExtension(nextAddress));
            }
            else
            {
                AppendRegister(out, opcode & 0x0F);
            }
            break;

        case 9:
            AppendGeneralAddress(out, opcode, nextAddress);
            out.Append(',');
            if (std::strcmp(operation.mnemonic, "XOP") == 0)
            {
                out.AppendUnsigned((opcode >> 6) & 0x0F);
            }
            else
            {
                AppendRegister(out, (opcode >> 6) & 0x0F);
            }
            break;

        default:
            break;
    }
}

} // namespace

UINT16 DisassembleASM(UINT16 address, char* buffer, size_t bufferSize)
{
    BufferWriter out(buffer, bufferSize);
    out.AppendHexWord(address);
    out.Append(' ');

    if ((address & 1) != 0)
    {
        out.Append("<-- Illegal value in PC");
        return static_cast<UINT16>(address + 1);
    }

    const UINT16 opcode = ReadRawWord(address);
    UINT16 nextAddress = static_cast<UINT16>(address + 2);
    const sOpCode* operation = LookupOpCode(opcode);

    const bool isNop = opcode == 0x1000;
    out.Append(isNop ? "NOP" : operation->mnemonic);

    if (!isNop && operation->format != 7 && operation->format != 0)
    {
        out.PadTo(10);
        AppendOperands(out, *operation, opcode, nextAddress);
    }

    return nextAddress;
}
