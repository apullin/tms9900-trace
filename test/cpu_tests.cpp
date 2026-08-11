#include "cpu.hpp"
#include "disasm.hpp"

#include <cstdio>
#include <cstring>
#include <cstdint>

namespace {

int Failures;

void RawWriteWord(UINT16 address, UINT16 value) {
  Memory[address] = static_cast<UINT8>(value >> 8);
  Memory[static_cast<UINT16>(address + 1)] = static_cast<UINT8>(value);
}

UINT16 RawReadWord(UINT16 address) {
  return static_cast<UINT16>((static_cast<UINT16>(Memory[address]) << 8) |
                             Memory[static_cast<UINT16>(address + 1)]);
}

void Begin(UINT16 wp = 0x1000, UINT16 pc = 0x0200, UINT16 st = 0) {
  ClearMemory();
  ResetCPU();
  SetMemoryWaitStates(0);
  SetWorkspacePointer(wp);
  SetProgramCounter(pc);
  Status = st;
}

void Check(const char *name, UINT64 actual, UINT64 expected) {
  if (actual == expected) {
    std::printf("PASS %-42s actual=%llu\n", name,
                static_cast<unsigned long long>(actual));
    return;
  }

  std::printf("FAIL %-42s actual=%llu expected=%llu\n", name,
              static_cast<unsigned long long>(actual),
              static_cast<unsigned long long>(expected));
  ++Failures;
}

void CheckHex(const char *name, UINT16 actual, UINT16 expected) {
  if (actual == expected) {
    std::printf("PASS %-42s actual=>%04X\n", name, actual);
    return;
  }

  std::printf("FAIL %-42s actual=>%04X expected=>%04X\n", name, actual,
              expected);
  ++Failures;
}

void TestInterruptEntryUsesArchitecturalPC() {
  // Run this first: InitCPU() leaves the private fetch pointer at its static
  // zero initialization while the architectural PC is set independently.
  Begin(0x1000, 0x0200, 0x0001);
  RawWriteWord(0x0004, 0x1100);
  RawWriteWord(0x0006, 0x0300);

  TriggerInterrupt(1);
  StepCPU();

  CheckHex("interrupt loads WP", WorkspacePtr, 0x1100);
  CheckHex("interrupt loads PC", ProgramCounter, 0x0300);
  CheckHex("interrupt saves old WP in R13", RawReadWord(0x111A), 0x1000);
  CheckHex("interrupt saves architectural PC in R14", RawReadWord(0x111C),
           0x0200);
  CheckHex("interrupt saves old ST in R15", RawReadWord(0x111E), 0x0001);
  Check("interrupt does not count as instruction", InstructionCounter, 0);
  Check("interrupt context-switch cycles", ClockCycleCounter, 22);
}

void TestRegisterMovTiming() {
  Begin();
  RawWriteWord(0x1000, 0x1234); // R0
  RawWriteWord(0x0200, 0xC040); // MOV R0,R1

  StepCPU();

  CheckHex("MOV R0,R1 result", RawReadWord(0x1002), 0x1234);
  CheckHex("MOV R0,R1 next PC", ProgramCounter, 0x0202);
  Check("MOV R0,R1 Table 3 cycles", ClockCycleCounter, 14);
}

void TestAddressingModeTiming() {
  Begin();
  RawWriteWord(0x0200, 0xC820); // MOV @>3000,@>4000
  RawWriteWord(0x0202, 0x3000);
  RawWriteWord(0x0204, 0x4000);
  RawWriteWord(0x3000, 0xA55A);
  StepCPU();
  Check("MOV symbolic,symbolic Table 3 cycles", ClockCycleCounter, 30);

  Begin();
  RawWriteWord(0x0200, 0x0460); // B @>3000
  RawWriteWord(0x0202, 0x3000);
  StepCPU();
  Check("B symbolic Table 3 cycles", ClockCycleCounter, 16);
}

void TestStepEvents() {
  Begin();
  RawWriteWord(0x0200, 0x0200); // LI R0,>1234
  RawWriteWord(0x0202, 0x1234);
  CPUStepResult result = StepCPU();
  Check("ordinary opcode reports instruction event",
        static_cast<UINT64>(result.event == CPUEvent::Instruction), 1);
  Check("instruction event exposes decoded opcode",
        static_cast<UINT64>(result.opcode != nullptr &&
                            std::strcmp(result.opcode->mnemonic, "LI") == 0),
        1);

  Begin(0x1000, 0x0200, 0x0001);
  RawWriteWord(0x0004, 0x1100);
  RawWriteWord(0x0006, 0x0300);
  RawWriteWord(0x0300, 0x0200); // LI R0,>5678
  RawWriteWord(0x0302, 0x5678);
  TriggerInterrupt(1);
  result = StepCPU();
  Check("interrupt entry reports interrupt event",
        static_cast<UINT64>(result.event == CPUEvent::Interrupt), 1);
  Check("interrupt event has no stale opcode",
        static_cast<UINT64>(result.opcode == nullptr), 1);
  Check("interrupt event does not increment instructions",
        InstructionCounter, 0);
  result = StepCPU();
  Check("post-interrupt target reports instruction event",
        static_cast<UINT64>(result.event == CPUEvent::Instruction), 1);
  Check("post-interrupt target increments instructions",
        InstructionCounter, 1);

  Begin();
  RawWriteWord(0x0200, 0x0340); // IDLE
  result = StepCPU();
  Check("IDLE opcode itself is an instruction event",
        static_cast<UINT64>(result.event == CPUEvent::Instruction), 1);
  Check("IDLE instruction reports halted state",
        static_cast<UINT64>(result.halted), 1);
  result = StepCPU();
  Check("subsequent halted step reports idle event",
        static_cast<UINT64>(result.event == CPUEvent::Idle), 1);
}

void TestWaitStateTiming() {
  Begin();
  SetMemoryWaitStates(3);
  RawWriteWord(0x1000, 0x1234);
  RawWriteWord(0x0200, 0xC040); // MOV R0,R1: C=14, M=4
  StepCPU();
  Check("MOV register wait-state timing", ClockCycleCounter, 26);

  Begin();
  SetMemoryWaitStates(2);
  RawWriteWord(0x0200, 0xC820); // MOV @>3000,@>4000: C=30, M=6
  RawWriteWord(0x0202, 0x3000);
  RawWriteWord(0x0204, 0x4000);
  RawWriteWord(0x3000, 0xA55A);
  StepCPU();
  Check("MOV symbolic wait-state timing", ClockCycleCounter, 42);

  Begin();
  SetMemoryWaitStates(2);
  RawWriteWord(0x1000, 0x3000);
  RawWriteWord(0x3000, 0xA55A);
  RawWriteWord(0x0200, 0xC050); // MOV *R0,R1: C=18, M=5
  StepCPU();
  Check("MOV indirect wait-state timing", ClockCycleCounter, 28);

  Begin();
  SetMemoryWaitStates(2);
  RawWriteWord(0x1000, 0x3000);
  RawWriteWord(0x3000, 0xA55A);
  RawWriteWord(0x0200, 0xC070); // MOV *R0+,R1: C=22, M=6
  StepCPU();
  Check("MOV word autoincrement timing", ClockCycleCounter, 34);

  Begin();
  SetMemoryWaitStates(2);
  RawWriteWord(0x1000, 0x3000);
  Memory[0x3000] = 0xA5;
  RawWriteWord(0x0200, 0xD070); // MOVB *R0+,R1: C=20, M=6
  StepCPU();
  Check("MOVB byte autoincrement timing", ClockCycleCounter, 32);

  Begin();
  SetMemoryWaitStates(1);
  RawWriteWord(0x0200, 0x0460); // B @>3000: C=16, M=3
  RawWriteWord(0x0202, 0x3000);
  StepCPU();
  Check("B symbolic wait-state timing", ClockCycleCounter, 19);

  Begin(0x1000, 0x0200, 0x0001);
  SetMemoryWaitStates(2);
  RawWriteWord(0x0004, 0x1100);
  RawWriteWord(0x0006, 0x0300);
  TriggerInterrupt(1); // C=22, M=5
  StepCPU();
  Check("interrupt wait-state timing", ClockCycleCounter, 32);
}

void TestInspectionAndResetAPI() {
  Begin();
  RawWriteWord(0x3000, 0xA55A);
  ClockCycleCounter = 123;
  CheckHex("public ReadWord returns raw memory", ReadWord(0x3000), 0xA55A);
  WriteWord(0x3000, 0x5AA5);
  Check("public inspection accesses are timing-neutral", ClockCycleCounter,
        123);

  Memory[0x4000] = 0xA5;
  ResetCPU();
  Check("architectural reset preserves loaded memory", Memory[0x4000], 0xA5);
  CheckHex("architectural reset clears PC", ProgramCounter, 0x0000);
  CheckHex("architectural reset clears WP", WorkspacePtr, 0x0000);
  ClearMemory();
  Check("explicit memory clear erases memory", Memory[0x4000], 0);
}

void TestVariableTiming() {
  Begin();
  RawWriteWord(0x1002, 0x0001);
  RawWriteWord(0x0200, 0x0751); // ABS *R1, positive: C=16, M=3
  RawWriteWord(0x1002, 0x3000);
  RawWriteWord(0x3000, 0x0001);
  StepCPU();
  Check("ABS positive indirect timing", ClockCycleCounter, 16);

  Begin();
  RawWriteWord(0x1002, 0x3000);
  RawWriteWord(0x3000, 0xFFFF);
  RawWriteWord(0x0200, 0x0751); // ABS *R1, negative: +2C,+1M
  StepCPU();
  Check("ABS negative indirect timing", ClockCycleCounter, 18);

  Begin();
  RawWriteWord(0x1000, 0x0001); // R0 supplies count one.
  RawWriteWord(0x1002, 0x8000);
  RawWriteWord(0x0200, 0x0801); // SRA R1,0: 12+8+2 clocks
  StepCPU();
  Check("R0-count shift timing", ClockCycleCounter, 22);

  Begin();
  RawWriteWord(0x1000, 0x0000); // Zero selects a count of 16.
  RawWriteWord(0x1002, 0x8000);
  RawWriteWord(0x0200, 0x0801); // 12+8+2*16 clocks, M=4.
  StepCPU();
  Check("R0-zero shift timing", ClockCycleCounter, 52);

  Begin();
  SetMemoryWaitStates(2);
  RawWriteWord(0x1000, 0x0201); // X R0 executes LI R1.
  RawWriteWord(0x0200, 0x0480);
  RawWriteWord(0x0202, 0xA55A);
  StepCPU(); // X+LI: C=16, M=4
  Check("X inner-instruction timing", ClockCycleCounter, 24);
  CheckHex("X inner instruction result", RawReadWord(0x1002), 0xA55A);

  Begin();
  RawWriteWord(0x1000, 0x0003); // Divisor R0.
  RawWriteWord(0x1002, 0x0003); // High dividend R1: overflow.
  RawWriteWord(0x1004, 0x1234);
  RawWriteWord(0x0200, 0x3C40); // DIV R0,R1.
  StepCPU();
  Check("DIV overflow timing", ClockCycleCounter, 16);

  Begin();
  RawWriteWord(0x1000, 0x0002);
  RawWriteWord(0x1002, 0x0000);
  RawWriteWord(0x1004, 0x0004); // Remainder zero: low endpoint.
  RawWriteWord(0x0200, 0x3C40);
  StepCPU();
  Check("DIV successful minimum timing", ClockCycleCounter, 92);

  Begin();
  SetMemoryWaitStates(1);
  RawWriteWord(0x1000, 0xFFFF);
  RawWriteWord(0x1002, 0x0000);
  RawWriteWord(0x1004, 0x8000); // 16-bit remainder: high endpoint.
  RawWriteWord(0x0200, 0x3C40);
  StepCPU();
  Check("DIV successful maximum plus waits", ClockCycleCounter, 130);

  Begin();
  RawWriteWord(0x1000, 0xA55A);
  RawWriteWord(0x0200, 0x3040); // LDCR R0,1: 20+2*1.
  StepCPU();
  Check("LDCR one-bit timing", ClockCycleCounter, 22);

  Begin();
  RawWriteWord(0x1000, 0xA55A);
  RawWriteWord(0x0200, 0x3000); // LDCR R0,16.
  StepCPU();
  Check("LDCR sixteen-bit timing", ClockCycleCounter, 52);

  struct CruTimingCase {
    UINT16 opcode;
    UINT32 cycles;
  } cruCases[] = {
      {0x35C0, 42}, // STCR R0,7.
      {0x3600, 44}, // STCR R0,8.
      {0x3640, 58}, // STCR R0,9.
      {0x3400, 60}, // STCR R0,16.
  };
  for (const CruTimingCase &test : cruCases) {
    Begin();
    RawWriteWord(0x0200, test.opcode);
    StepCPU();
    Check("STCR variable-count timing", ClockCycleCounter, test.cycles);
  }
}

void TestCRUStatusEffects() {
  constexpr UINT16 Preserved =
      TMS_CARRY | TMS_OVERFLOW | TMS_XOP | TMS_PARITY | 0x000A;

  Begin(0x1000, 0x0200, Preserved);
  RawWriteWord(0x1000, 0x8000);
  RawWriteWord(0x0200, 0x3000); // LDCR R0,16.
  StepCPU();
  CheckHex("LDCR word preserves unrelated status",
           Status, Preserved | TMS_LOGICAL);

  Begin(0x1000, 0x0200, Preserved);
  Memory[0x1000] = 0x03;
  RawWriteWord(0x0200, 0x3200); // LDCR R0,8.
  StepCPU();
  CheckHex("LDCR byte recomputes parity", Status,
           static_cast<UINT16>((Preserved & ~TMS_PARITY) |
                               TMS_LOGICAL | TMS_ARITHMETIC));

  Begin(0x1000, 0x0200, Preserved);
  RawWriteWord(0x1000, 0xFFFF);
  RawWriteWord(0x0200, 0x3400); // STCR R0,16; bare CRU reads zero.
  StepCPU();
  CheckHex("STCR word preserves unrelated status",
           Status, Preserved | TMS_EQUAL);
  CheckHex("STCR word writes CRU input", RawReadWord(0x1000), 0x0000);

  Begin(0x1000, 0x0200, Preserved);
  Memory[0x1000] = 0xFF;
  RawWriteWord(0x0200, 0x3600); // STCR R0,8; bare CRU reads zero.
  StepCPU();
  CheckHex("STCR byte recomputes parity", Status,
           static_cast<UINT16>((Preserved & ~TMS_PARITY) | TMS_EQUAL));
  Check("STCR byte writes CRU input", Memory[0x1000], 0x00);
}

void TestResetInterruptClearsStatus() {
  Begin(0x1000, 0x0200, 0xFFFF);
  RawWriteWord(0x0000, 0x1100);
  RawWriteWord(0x0002, 0x0300);

  TriggerInterrupt(0);
  StepCPU();

  CheckHex("RESET clears the complete status register", Status, 0x0000);
  CheckHex("RESET saves cleared status in new R15", RawReadWord(0x111E),
           0x0000);
  Check("RESET function cycles", ClockCycleCounter, 26);
}

void TestResetBypassesInterruptDeferral() {
  Begin(0x1000, 0x0200, 0x0001);
  RawWriteWord(0x0004, 0x1100); // Level-1 vector.
  RawWriteWord(0x0006, 0x3000);
  RawWriteWord(0x3000, 0x0200); // Would run if RESET were deferred.
  RawWriteWord(0x3002, 0xA55A);
  RawWriteWord(0x0000, 0x1200); // RESET vector.
  RawWriteWord(0x0002, 0x4000);

  TriggerInterrupt(1);
  StepCPU();
  TriggerInterrupt(0);
  StepCPU();

  CheckHex("RESET bypasses post-interrupt inhibition", WorkspacePtr, 0x1200);
  CheckHex("RESET enters its vector without executing ISR", ProgramCounter,
           0x4000);
  CheckHex("RESET preemption leaves ISR target untouched",
           RawReadWord(0x1100), 0x0000);
}

void TestInterruptMaskingAndPriority() {
  Begin(0x1000, 0x0200, 0x0000);
  RawWriteWord(0x0200, 0x0200); // LI R0,>1234.
  RawWriteWord(0x0202, 0x1234);
  RawWriteWord(0x0004, 0x1100);
  RawWriteWord(0x0006, 0x0300);
  TriggerInterrupt(1);
  CPUStepResult result = StepCPU();
  Check("masked interrupt leaves instruction executable",
        static_cast<UINT64>(result.event == CPUEvent::Instruction), 1);
  CheckHex("masked interrupt does not switch workspace", WorkspacePtr,
           0x1000);
  Status = static_cast<UINT16>((Status & 0xFFF0) | 1);
  result = StepCPU();
  Check("pending interrupt enters after mask enables it",
        static_cast<UINT64>(result.event == CPUEvent::Interrupt), 1);
  CheckHex("newly enabled interrupt switches workspace", WorkspacePtr,
           0x1100);

  Begin(0x1000, 0x0200, 0x0004);
  RawWriteWord(0x0008, 0x1200); // Level 2.
  RawWriteWord(0x000A, 0x0400);
  RawWriteWord(0x0010, 0x1400); // Level 4.
  RawWriteWord(0x0012, 0x0500);
  TriggerInterrupt(4);
  TriggerInterrupt(2);
  StepCPU();
  CheckHex("higher-priority pending level wins", WorkspacePtr, 0x1200);
  CheckHex("higher-priority vector supplies PC", ProgramCounter, 0x0400);

  Begin(0x1000, 0x0200, 0x0002);
  RawWriteWord(0x0008, 0x1100); // Level 2 vector.
  RawWriteWord(0x000A, 0x0300);
  RawWriteWord(0x0300, 0x0200); // First level-2 target instruction.
  RawWriteWord(0x0302, 0xA55A);
  RawWriteWord(0x0004, 0x1200); // Level 1 vector.
  RawWriteWord(0x0006, 0x0400);
  TriggerInterrupt(2);
  StepCPU();
  TriggerInterrupt(1);
  result = StepCPU();
  Check("nested IRQ waits for first target instruction",
        static_cast<UINT64>(result.event == CPUEvent::Instruction), 1);
  CheckHex("nested IRQ target instruction completed", RawReadWord(0x1100),
           0xA55A);
  result = StepCPU();
  Check("nested higher-priority IRQ then enters",
        static_cast<UINT64>(result.event == CPUEvent::Interrupt), 1);
  CheckHex("nested IRQ loads level-1 workspace", WorkspacePtr, 0x1200);
}

void TestBLWPDefersInterrupt() {
  Begin(0x1000, 0x0200, 0x0001);
  RawWriteWord(0x0200, 0x0420); // BLWP @>4000
  RawWriteWord(0x0202, 0x4000);
  RawWriteWord(0x4000, 0x1100);
  RawWriteWord(0x4002, 0x0300);
  RawWriteWord(0x0300, 0x0200); // LI R0,>1234
  RawWriteWord(0x0302, 0x1234);
  RawWriteWord(0x0004, 0x1200); // Level-1 interrupt vector.
  RawWriteWord(0x0006, 0x0500);

  StepCPU();
  TriggerInterrupt(1);
  StepCPU();

  CheckHex("BLWP target first instruction executes", RawReadWord(0x1100),
           0x1234);
  CheckHex("BLWP defers interrupt for target instruction", WorkspacePtr,
           0x1100);
  Check("BLWP target instruction counted", InstructionCounter, 2);
}

void TestXOPDefersInterrupt() {
  Begin(0x1000, 0x0200, 0x0001);
  RawWriteWord(0x0200, 0x2C00); // XOP R0,0
  RawWriteWord(0x0040, 0x1100); // XOP 0 vector.
  RawWriteWord(0x0042, 0x0300);
  RawWriteWord(0x0300, 0x0200); // LI R0,>5678
  RawWriteWord(0x0302, 0x5678);
  RawWriteWord(0x0004, 0x1200); // Level-1 interrupt vector.
  RawWriteWord(0x0006, 0x0500);

  StepCPU();
  TriggerInterrupt(1);
  StepCPU();

  CheckHex("XOP target first instruction executes", RawReadWord(0x1100),
           0x5678);
  CheckHex("XOP defers interrupt for target instruction", WorkspacePtr,
           0x1100);
  Check("XOP target instruction counted", InstructionCounter, 2);
}

void TestContextSwitchState() {
  Begin(0x1000, 0x0200, 0xA1A5);
  RawWriteWord(0x0200, 0x2CE0); // XOP @>4000,3
  RawWriteWord(0x0202, 0x4000);
  RawWriteWord(0x004C, 0x1100);
  RawWriteWord(0x004E, 0x3000);

  StepCPU();

  CheckHex("XOP loads vector workspace", WorkspacePtr, 0x1100);
  CheckHex("XOP loads vector PC", ProgramCounter, 0x3000);
  CheckHex("XOP saves source effective address in R11",
           RawReadWord(0x1116), 0x4000);
  CheckHex("XOP saves old WP in R13", RawReadWord(0x111A), 0x1000);
  CheckHex("XOP saves return PC in R14", RawReadWord(0x111C), 0x0204);
  CheckHex("XOP saves pre-XOP status in R15", RawReadWord(0x111E), 0xA1A5);
  CheckHex("XOP sets status X bit after saving ST", Status,
           static_cast<UINT16>(0xA1A5 | TMS_XOP));

  RawWriteWord(0x3000, 0x0380); // RTWP
  StepCPU();
  CheckHex("RTWP restores old workspace", WorkspacePtr, 0x1000);
  CheckHex("RTWP restores return PC", ProgramCounter, 0x0204);
  CheckHex("RTWP restores pre-XOP status", Status, 0xA1A5);
}

void TestArchitecturalAddressAlignment() {
  Begin();
  RawWriteWord(0x0200, 0x02E0); // LWPI >1001
  RawWriteWord(0x0202, 0x1001);
  StepCPU();
  CheckHex("LWPI discards WP bit 0", WorkspacePtr, 0x1000);

  Begin();
  RawWriteWord(0x0200, 0x0420); // BLWP @>4000
  RawWriteWord(0x0202, 0x4000);
  RawWriteWord(0x4000, 0x1101);
  RawWriteWord(0x4002, 0x3001);
  StepCPU();
  CheckHex("BLWP discards new WP bit 0", WorkspacePtr, 0x1100);
  CheckHex("BLWP discards new PC bit 0", ProgramCounter, 0x3000);

  Begin(0x1100);
  RawWriteWord(0x0200, 0x0380); // RTWP
  RawWriteWord(0x111A, 0x1001);
  RawWriteWord(0x111C, 0x3001);
  RawWriteWord(0x111E, 0xA55A);
  StepCPU();
  CheckHex("RTWP discards restored WP bit 0", WorkspacePtr, 0x1000);
  CheckHex("RTWP discards restored PC bit 0", ProgramCounter, 0x3000);

  Begin(0x1000, 0x0201);
  RawWriteWord(0x0200, 0x0201); // LI R1,>A55A
  RawWriteWord(0x0202, 0xA55A);
  StepCPU();
  CheckHex("instruction fetch discards PC bit 0", ProgramCounter, 0x0204);
  CheckHex("odd supplied PC still decodes aligned LI", RawReadWord(0x1002),
           0xA55A);
}

void TestDecodeAndDisassemblyEdges() {
  Check("reserved >0210 does not decode as LI",
        std::strcmp(LookupOpCode(0x0210)->mnemonic, "INVL") == 0, 1);
  Check("reserved >0230 does not decode as AI",
        std::strcmp(LookupOpCode(0x0230)->mnemonic, "INVL") == 0, 1);
  Check("reserved >02B0 does not decode as STWP",
        std::strcmp(LookupOpCode(0x02B0)->mnemonic, "INVL") == 0, 1);

  Begin();
  RawWriteWord(0x0200, 0x2CC1); // XOP R1,3
  char buffer[80] = {};
  DisassembleASM(0x0200, buffer, sizeof(buffer));
  Check("XOP disassembly prints numeric operation level",
        std::strstr(buffer, "R1,3") != nullptr, 1);

  Begin(0x1000, 0xFFFC);
  RawWriteWord(0xFFFC, 0xC820); // MOV @>3000,@>4000 across wrap.
  RawWriteWord(0xFFFE, 0x3000);
  RawWriteWord(0x0000, 0x4000);
  const UINT16 wrappedNext = DisassembleASM(0xFFFC, buffer, sizeof(buffer));
  CheckHex("three-word disassembly wraps next address", wrappedNext, 0x0002);
  Check("three-word disassembly wraps both extensions",
        std::strstr(buffer, "MOV  @>3000,@>4000") != nullptr, 1);

  Begin();
  RawWriteWord(0x0200, 0x1000); // JMP +0 alias.
  DisassembleASM(0x0200, buffer, sizeof(buffer));
  Check("JMP +0 disassembles as NOP",
        std::strcmp(buffer, "0200 NOP") == 0, 1);

  RawWriteWord(0x0200, 0x1DFF); // SBO -1.
  DisassembleASM(0x0200, buffer, sizeof(buffer));
  Check("CRU displacement prints encoded byte",
        std::strstr(buffer, "SBO  >FF") != nullptr, 1);

  struct BoundedBuffer {
    char text[8];
    char guard[4];
  } bounded;
  std::memset(&bounded, 0x5A, sizeof(bounded));
  RawWriteWord(0x0200, 0x0201);
  RawWriteWord(0x0202, 0xA55A);
  DisassembleASM(0x0200, bounded.text, sizeof(bounded.text));
  Check("bounded disassembly always terminates",
        static_cast<UINT64>(bounded.text[sizeof(bounded.text) - 1] == '\0'), 1);
  Check("bounded disassembly preserves following storage",
        static_cast<UINT64>(bounded.guard[0] == 0x5A &&
                            bounded.guard[1] == 0x5A &&
                            bounded.guard[2] == 0x5A &&
                            bounded.guard[3] == 0x5A), 1);
}

void RunDecodeSweep() {
  struct Pattern {
    UINT16 opcode;
    UINT16 mask;
    const char *mnemonic;
  } patterns[] = {
      {0x0200, 0xFFF0, "LI"},   {0x0220, 0xFFF0, "AI"},
      {0x0240, 0xFFF0, "ANDI"}, {0x0260, 0xFFF0, "ORI"},
      {0x0280, 0xFFF0, "CI"},   {0x02A0, 0xFFF0, "STWP"},
      {0x02C0, 0xFFF0, "STST"}, {0x02E0, 0xFFF0, "LWPI"},
      {0x0300, 0xFFF0, "LIMI"}, {0x0340, 0xFFFF, "IDLE"},
      {0x0360, 0xFFFF, "RSET"}, {0x0380, 0xFFFF, "RTWP"},
      {0x03A0, 0xFFFF, "CKON"}, {0x03C0, 0xFFFF, "CKOF"},
      {0x03E0, 0xFFFF, "LREX"}, {0x0400, 0xFFC0, "BLWP"},
      {0x0440, 0xFFC0, "B"},    {0x0480, 0xFFC0, "X"},
      {0x04C0, 0xFFC0, "CLR"},  {0x0500, 0xFFC0, "NEG"},
      {0x0540, 0xFFC0, "INV"},  {0x0580, 0xFFC0, "INC"},
      {0x05C0, 0xFFC0, "INCT"}, {0x0600, 0xFFC0, "DEC"},
      {0x0640, 0xFFC0, "DECT"}, {0x0680, 0xFFC0, "BL"},
      {0x06C0, 0xFFC0, "SWPB"}, {0x0700, 0xFFC0, "SETO"},
      {0x0740, 0xFFC0, "ABS"},  {0x0800, 0xFF00, "SRA"},
      {0x0900, 0xFF00, "SRL"},  {0x0A00, 0xFF00, "SLA"},
      {0x0B00, 0xFF00, "SRC"},  {0x1000, 0xFF00, "JMP"},
      {0x1100, 0xFF00, "JLT"},  {0x1200, 0xFF00, "JLE"},
      {0x1300, 0xFF00, "JEQ"},  {0x1400, 0xFF00, "JHE"},
      {0x1500, 0xFF00, "JGT"},  {0x1600, 0xFF00, "JNE"},
      {0x1700, 0xFF00, "JNC"},  {0x1800, 0xFF00, "JOC"},
      {0x1900, 0xFF00, "JNO"},  {0x1A00, 0xFF00, "JL"},
      {0x1B00, 0xFF00, "JH"},   {0x1C00, 0xFF00, "JOP"},
      {0x1D00, 0xFF00, "SBO"},  {0x1E00, 0xFF00, "SBZ"},
      {0x1F00, 0xFF00, "TB"},   {0x2000, 0xFC00, "COC"},
      {0x2400, 0xFC00, "CZC"},  {0x2800, 0xFC00, "XOR"},
      {0x2C00, 0xFC00, "XOP"},  {0x3000, 0xFC00, "LDCR"},
      {0x3400, 0xFC00, "STCR"}, {0x3800, 0xFC00, "MPY"},
      {0x3C00, 0xFC00, "DIV"},  {0x4000, 0xF000, "SZC"},
      {0x5000, 0xF000, "SZCB"}, {0x6000, 0xF000, "S"},
      {0x7000, 0xF000, "SB"},   {0x8000, 0xF000, "C"},
      {0x9000, 0xF000, "CB"},   {0xA000, 0xF000, "A"},
      {0xB000, 0xF000, "AB"},   {0xC000, 0xF000, "MOV"},
      {0xD000, 0xF000, "MOVB"}, {0xE000, 0xF000, "SOC"},
      {0xF000, 0xF000, "SOCB"},
  };

  UINT64 mismatches = 0;
  for (unsigned rawOpcode = 0; rawOpcode <= 0xFFFF; ++rawOpcode) {
    const UINT16 opcode = static_cast<UINT16>(rawOpcode);
    const char *expected = "INVL";
    for (const Pattern &pattern : patterns) {
      if ((opcode & pattern.mask) == pattern.opcode) {
        expected = pattern.mnemonic;
        break;
      }
    }

    const char *actual = LookupOpCode(opcode)->mnemonic;
    if (std::strcmp(actual, expected) != 0) {
      if (mismatches < 20) {
        std::printf("DECODE FAIL opcode=>%04X actual=%s expected=%s\n",
                    opcode, actual, expected);
      }
      ++mismatches;
    }
  }

  std::printf("Opcode decode sweep: 65536 cases, %llu mismatches\n",
              static_cast<unsigned long long>(mismatches));
  if (mismatches != 0)
    ++Failures;
}

void RunUBSRC() {
  Begin();
  RawWriteWord(0x1002, 0x8000); // R1
  RawWriteWord(0x0200, 0x0B11); // SRC R1,1
  StepCPU();
}

void RunUBSLA() {
  Begin();
  RawWriteWord(0x1000, 0x0000); // R0 count: zero means 16.
  RawWriteWord(0x1002, 0x8000); // R1
  RawWriteWord(0x0200, 0x0A01); // SLA R1,0
  StepCPU();
}

void RunDisasmWrap() {
  Begin(0x1000, 0xFFFE);
  RawWriteWord(0xFFFE, 0x0201); // LI R1, immediate wraps to address zero.
  RawWriteWord(0x0000, 0xA55A);
  char buffer[80];
  const UINT16 next = DisassembleASM(0xFFFE, buffer, sizeof(buffer));
  std::printf("%s next=>%04X\n", buffer, next);
}

UINT16 ExpectedLAE(UINT16 value) {
  if (value == 0)
    return TMS_EQUAL;
  if ((value & 0x8000) != 0)
    return TMS_LOGICAL;
  return TMS_LOGICAL | TMS_ARITHMETIC;
}

UINT16 ExpectedLAEByte(UINT8 value) {
  if (value == 0)
    return TMS_EQUAL;
  if (static_cast<int8_t>(value) < 0)
    return TMS_LOGICAL;
  return TMS_LOGICAL | TMS_ARITHMETIC;
}

UINT16 ExpectedCompare(UINT16 source, UINT16 destination) {
  UINT16 status = 0;
  if (source == destination)
    status |= TMS_EQUAL;
  else {
    if (source > destination)
      status |= TMS_LOGICAL;
    if (static_cast<int16_t>(source) > static_cast<int16_t>(destination))
      status |= TMS_ARITHMETIC;
  }
  return status;
}

UINT16 ExpectedCompareByte(UINT8 source, UINT8 destination) {
  UINT16 status = 0;
  if (source == destination)
    status |= TMS_EQUAL;
  else {
    if (source > destination)
      status |= TMS_LOGICAL;
    if (static_cast<int8_t>(source) > static_cast<int8_t>(destination))
      status |= TMS_ARITHMETIC;
  }
  return status;
}

UINT16 ExpectedParity(UINT8 value) {
  unsigned bits = 0;
  for (unsigned bit = 0; bit != 8; ++bit)
    bits += (value >> bit) & 1;
  return (bits & 1) ? TMS_PARITY : 0;
}

void ReportSemanticMismatch(const char *group, const char *op, UINT16 lhs,
                            UINT16 rhs, UINT16 actualValue,
                            UINT16 expectedValue, UINT16 actualStatus,
                            UINT16 expectedStatus, UINT64 &mismatches) {
  if (mismatches < 20) {
    std::printf(
        "%s FAIL %s lhs=>%04X rhs=>%04X result=>%04X/%04X "
        "st=>%04X/%04X\n",
        group, op, lhs, rhs, actualValue, expectedValue, actualStatus,
        expectedStatus);
  }
  ++mismatches;
}

void PrepareDirect(UINT16 opcode, UINT16 r0, UINT16 r1,
                   UINT16 status = 0xFFFF) {
  WorkspacePtr = 0x1000;
  ProgramCounter = 0x0200;
  Status = status;
  ClockCycleCounter = 0;
  InstructionCounter = 0;
  RawWriteWord(0x0200, opcode);
  RawWriteWord(0x1000, r0);
  RawWriteWord(0x1002, r1);
}

void RunExhaustiveUnary() {
  enum Kind { Abs, Neg, Inv, Inc, Inct, Dec, Dect, Swpb, Clr, Seto };
  struct UnaryCase {
    const char *name;
    UINT16 opcode;
    Kind kind;
  } cases[] = {
      {"ABS", 0x0741, Abs},   {"NEG", 0x0501, Neg},
      {"INV", 0x0541, Inv},   {"INC", 0x0581, Inc},
      {"INCT", 0x05C1, Inct}, {"DEC", 0x0601, Dec},
      {"DECT", 0x0641, Dect}, {"SWPB", 0x06C1, Swpb},
      {"CLR", 0x04C1, Clr},   {"SETO", 0x0701, Seto},
  };

  UINT64 tested = 0;
  UINT64 mismatches = 0;
  Begin();

  for (const UnaryCase &test : cases) {
    for (unsigned input = 0; input <= 0xFFFF; ++input) {
      const UINT16 value = static_cast<UINT16>(input);
      UINT16 expected = value;
      UINT16 expectedStatus = 0xFFFF;

      switch (test.kind) {
      case Abs:
        expectedStatus &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL |
                            TMS_CARRY | TMS_OVERFLOW);
        expectedStatus |= ExpectedLAE(value);
        if (value == 0x8000)
          expectedStatus |= TMS_OVERFLOW;
        if (value & 0x8000)
          expected = static_cast<UINT16>(0 - value);
        break;
      case Neg:
        expected = static_cast<UINT16>(0 - value);
        expectedStatus &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL |
                            TMS_CARRY | TMS_OVERFLOW);
        expectedStatus |= ExpectedLAE(expected);
        if (value == 0)
          expectedStatus |= TMS_CARRY;
        if (value == 0x8000)
          expectedStatus |= TMS_OVERFLOW;
        break;
      case Inv:
        expected = static_cast<UINT16>(~value);
        expectedStatus &=
            ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
        expectedStatus |= ExpectedLAE(expected);
        break;
      case Inc:
      case Inct: {
        const UINT32 amount = test.kind == Inc ? 1 : 2;
        const UINT32 result = static_cast<UINT32>(value) + amount;
        expected = static_cast<UINT16>(result);
        expectedStatus &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL |
                            TMS_CARRY | TMS_OVERFLOW);
        expectedStatus |= ExpectedLAE(expected);
        if (result & 0x10000)
          expectedStatus |= TMS_CARRY;
        if (((result ^ value) & (result ^ amount) & 0x8000) != 0)
          expectedStatus |= TMS_OVERFLOW;
        break;
      }
      case Dec:
      case Dect: {
        const UINT32 amount = test.kind == Dec ? 1 : 2;
        const UINT32 result = static_cast<UINT32>(value) - amount;
        expected = static_cast<UINT16>(result);
        expectedStatus &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL |
                            TMS_CARRY | TMS_OVERFLOW);
        expectedStatus |= ExpectedLAE(expected);
        if ((result & 0x10000) == 0)
          expectedStatus |= TMS_CARRY;
        if (((amount ^ value) & (value ^ result) & 0x8000) != 0)
          expectedStatus |= TMS_OVERFLOW;
        break;
      }
      case Swpb:
        expected = static_cast<UINT16>((value << 8) | (value >> 8));
        break;
      case Clr:
        expected = 0;
        break;
      case Seto:
        expected = 0xFFFF;
        break;
      }

      PrepareDirect(test.opcode, 0, value);
      StepCPU();
      const UINT16 actual = RawReadWord(0x1002);
      if (actual != expected || Status != expectedStatus)
        ReportSemanticMismatch("UNARY", test.name, value, 0, actual,
                               expected, Status, expectedStatus, mismatches);
      ++tested;
    }
  }

  std::printf("Exhaustive unary: %llu cases, %llu mismatches\n",
              static_cast<unsigned long long>(tested),
              static_cast<unsigned long long>(mismatches));
  if (mismatches != 0)
    ++Failures;
}

void RunExhaustiveByteOps() {
  enum Kind { Add, Sub, Compare, Move, Soc, Szc };
  struct ByteCase {
    const char *name;
    UINT16 opcode;
    Kind kind;
  } cases[] = {
      {"AB", 0xB040, Add},   {"SB", 0x7040, Sub},
      {"CB", 0x9040, Compare}, {"MOVB", 0xD040, Move},
      {"SOCB", 0xF040, Soc}, {"SZCB", 0x5040, Szc},
  };

  UINT64 tested = 0;
  UINT64 mismatches = 0;
  Begin();

  for (const ByteCase &test : cases) {
    for (unsigned source = 0; source <= 0xFF; ++source) {
      for (unsigned destination = 0; destination <= 0xFF; ++destination) {
        const UINT8 src = static_cast<UINT8>(source);
        const UINT8 dst = static_cast<UINT8>(destination);
        UINT8 expected = dst;
        UINT16 expectedStatus = 0xFFFF;

        switch (test.kind) {
        case Add: {
          const UINT32 result = source + destination;
          expected = static_cast<UINT8>(result);
          expectedStatus &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL |
                              TMS_CARRY | TMS_OVERFLOW | TMS_PARITY);
          expectedStatus |= ExpectedLAEByte(expected) | ExpectedParity(expected);
          if (result & 0x100)
            expectedStatus |= TMS_CARRY;
          if (((result ^ source) & (result ^ destination) & 0x80) != 0)
            expectedStatus |= TMS_OVERFLOW;
          break;
        }
        case Sub: {
          const UINT32 result = destination - source;
          expected = static_cast<UINT8>(result);
          expectedStatus &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL |
                              TMS_CARRY | TMS_OVERFLOW | TMS_PARITY);
          expectedStatus |= ExpectedLAEByte(expected) | ExpectedParity(expected);
          if ((result & 0x100) == 0)
            expectedStatus |= TMS_CARRY;
          if (((source ^ destination) & (destination ^ result) & 0x80) != 0)
            expectedStatus |= TMS_OVERFLOW;
          break;
        }
        case Compare:
          expectedStatus &=
              ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_PARITY);
          expectedStatus |= ExpectedCompareByte(src, dst) | ExpectedParity(src);
          break;
        case Move:
          expected = src;
          expectedStatus &=
              ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_PARITY);
          expectedStatus |= ExpectedLAEByte(expected) | ExpectedParity(expected);
          break;
        case Soc:
          expected = src | dst;
          expectedStatus &=
              ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_PARITY);
          expectedStatus |= ExpectedLAEByte(expected) | ExpectedParity(expected);
          break;
        case Szc:
          expected = static_cast<UINT8>(~src & dst);
          expectedStatus &=
              ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL | TMS_PARITY);
          expectedStatus |= ExpectedLAEByte(expected) | ExpectedParity(expected);
          break;
        }

        PrepareDirect(test.opcode,
                      static_cast<UINT16>((source << 8) | 0x003C),
                      static_cast<UINT16>((destination << 8) | 0x005A));
        StepCPU();
        const UINT16 actualWord = RawReadWord(0x1002);
        const UINT16 expectedWord = test.kind == Compare
                                        ? static_cast<UINT16>((destination << 8) |
                                                              0x005A)
                                        : static_cast<UINT16>((expected << 8) |
                                                              0x005A);
        if (actualWord != expectedWord || Status != expectedStatus)
          ReportSemanticMismatch("BYTE", test.name, src, dst, actualWord,
                                 expectedWord, Status, expectedStatus,
                                 mismatches);
        ++tested;
      }
    }
  }

  std::printf("Exhaustive byte ops: %llu cases, %llu mismatches\n",
              static_cast<unsigned long long>(tested),
              static_cast<unsigned long long>(mismatches));
  if (mismatches != 0)
    ++Failures;
}

UINT32 NextRandom(UINT32 &state) {
  state ^= state << 13;
  state ^= state >> 17;
  state ^= state << 5;
  return state;
}

void RunRandomWordOps() {
  enum Kind { Add, Sub, Compare, Move, Soc, Szc, Xor, Coc, Czc };
  struct WordCase {
    const char *name;
    UINT16 opcode;
    Kind kind;
  } cases[] = {
      {"A", 0xA040, Add},     {"S", 0x6040, Sub},
      {"C", 0x8040, Compare}, {"MOV", 0xC040, Move},
      {"SOC", 0xE040, Soc},   {"SZC", 0x4040, Szc},
      {"XOR", 0x2840, Xor},   {"COC", 0x2040, Coc},
      {"CZC", 0x2440, Czc},
  };

  UINT64 tested = 0;
  UINT64 mismatches = 0;
  UINT32 random = 0x9900C0DE;
  Begin();

  for (const WordCase &test : cases) {
    for (unsigned iteration = 0; iteration < 500000; ++iteration) {
      const UINT16 source = static_cast<UINT16>(NextRandom(random));
      const UINT16 destination = static_cast<UINT16>(NextRandom(random));
      UINT16 expected = destination;
      UINT16 expectedStatus = 0xFFFF;

      switch (test.kind) {
      case Add: {
        const UINT32 result = static_cast<UINT32>(source) + destination;
        expected = static_cast<UINT16>(result);
        expectedStatus &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL |
                            TMS_CARRY | TMS_OVERFLOW);
        expectedStatus |= ExpectedLAE(expected);
        if (result & 0x10000)
          expectedStatus |= TMS_CARRY;
        if (((result ^ source) & (result ^ destination) & 0x8000) != 0)
          expectedStatus |= TMS_OVERFLOW;
        break;
      }
      case Sub: {
        const UINT32 result =
            static_cast<UINT32>(destination) - source;
        expected = static_cast<UINT16>(result);
        expectedStatus &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL |
                            TMS_CARRY | TMS_OVERFLOW);
        expectedStatus |= ExpectedLAE(expected);
        if ((result & 0x10000) == 0)
          expectedStatus |= TMS_CARRY;
        if (((source ^ destination) & (destination ^ result) & 0x8000) != 0)
          expectedStatus |= TMS_OVERFLOW;
        break;
      }
      case Compare:
        expectedStatus &=
            ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
        expectedStatus |= ExpectedCompare(source, destination);
        break;
      case Move:
        expected = source;
        expectedStatus &=
            ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
        expectedStatus |= ExpectedLAE(expected);
        break;
      case Soc:
        expected = source | destination;
        expectedStatus &=
            ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
        expectedStatus |= ExpectedLAE(expected);
        break;
      case Szc:
        expected = static_cast<UINT16>(~source & destination);
        expectedStatus &=
            ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
        expectedStatus |= ExpectedLAE(expected);
        break;
      case Xor:
        expected = source ^ destination;
        expectedStatus &=
            ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
        expectedStatus |= ExpectedLAE(expected);
        break;
      case Coc:
        if ((source & destination) == source)
          expectedStatus |= TMS_EQUAL;
        else
          expectedStatus &= ~TMS_EQUAL;
        break;
      case Czc:
        if ((source & static_cast<UINT16>(~destination)) == source)
          expectedStatus |= TMS_EQUAL;
        else
          expectedStatus &= ~TMS_EQUAL;
        break;
      }

      PrepareDirect(test.opcode, source, destination);
      StepCPU();
      const UINT16 actual = RawReadWord(0x1002);
      const UINT16 expectedWord =
          (test.kind == Compare || test.kind == Coc || test.kind == Czc)
              ? destination
              : expected;
      if (actual != expectedWord || Status != expectedStatus)
        ReportSemanticMismatch("WORD", test.name, source, destination,
                               actual, expectedWord, Status, expectedStatus,
                               mismatches);
      ++tested;
    }
  }

  std::printf("Random word ops: %llu cases, %llu mismatches\n",
              static_cast<unsigned long long>(tested),
              static_cast<unsigned long long>(mismatches));
  if (mismatches != 0)
    ++Failures;
}

void RunRandomImmediateOps() {
  enum Kind { Load, Add, And, Or, Compare };
  struct ImmediateCase {
    const char *name;
    UINT16 opcode;
    Kind kind;
  } cases[] = {
      {"LI", 0x0201, Load},   {"AI", 0x0221, Add},
      {"ANDI", 0x0241, And}, {"ORI", 0x0261, Or},
      {"CI", 0x0281, Compare},
  };

  UINT64 tested = 0;
  UINT64 mismatches = 0;
  UINT32 random = 0x1A9900ED;
  Begin();

  for (const ImmediateCase &test : cases) {
    for (unsigned iteration = 0; iteration < 500000; ++iteration) {
      const UINT16 input = static_cast<UINT16>(NextRandom(random));
      const UINT16 immediate = static_cast<UINT16>(NextRandom(random));
      UINT16 expected = input;
      UINT16 expectedStatus = 0xFFFF;

      switch (test.kind) {
      case Load:
        expected = immediate;
        expectedStatus &=
            ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
        expectedStatus |= ExpectedLAE(expected);
        break;
      case Add: {
        const UINT32 result = static_cast<UINT32>(input) + immediate;
        expected = static_cast<UINT16>(result);
        expectedStatus &= ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL |
                            TMS_CARRY | TMS_OVERFLOW);
        expectedStatus |= ExpectedLAE(expected);
        if (result & 0x10000)
          expectedStatus |= TMS_CARRY;
        if (((result ^ input) & (result ^ immediate) & 0x8000) != 0)
          expectedStatus |= TMS_OVERFLOW;
        break;
      }
      case And:
        expected = input & immediate;
        expectedStatus &=
            ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
        expectedStatus |= ExpectedLAE(expected);
        break;
      case Or:
        expected = input | immediate;
        expectedStatus &=
            ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
        expectedStatus |= ExpectedLAE(expected);
        break;
      case Compare:
        expectedStatus &=
            ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL);
        expectedStatus |= ExpectedCompare(input, immediate);
        break;
      }

      PrepareDirect(test.opcode, 0, input);
      RawWriteWord(0x0202, immediate);
      StepCPU();
      const UINT16 actual = RawReadWord(0x1002);
      const UINT16 expectedWord =
          test.kind == Compare ? input : expected;
      if (actual != expectedWord || Status != expectedStatus)
        ReportSemanticMismatch("IMMED", test.name, input, immediate, actual,
                               expectedWord, Status, expectedStatus,
                               mismatches);
      ++tested;
    }
  }

  std::printf("Random immediate ops: %llu cases, %llu mismatches\n",
              static_cast<unsigned long long>(tested),
              static_cast<unsigned long long>(mismatches));
  if (mismatches != 0)
    ++Failures;
}

void RunRandomMultiplyDivide() {
  UINT64 tested = 0;
  UINT64 mismatches = 0;
  UINT32 random = 0xD1F1DE42;
  Begin();

  for (unsigned iteration = 0; iteration < 500000; ++iteration) {
    const UINT16 source = static_cast<UINT16>(NextRandom(random));
    const UINT16 destination = static_cast<UINT16>(NextRandom(random));
    const UINT32 expected = static_cast<UINT32>(source) * destination;

    PrepareDirect(0x3840, source, destination); // MPY R0,R1
    RawWriteWord(0x1004, 0xA55A);
    StepCPU();
    const UINT32 actual =
        (static_cast<UINT32>(RawReadWord(0x1002)) << 16) |
        RawReadWord(0x1004);
    if (actual != expected || Status != 0xFFFF) {
      ReportSemanticMismatch("MULDIV", "MPY", source, destination,
                             static_cast<UINT16>(actual >> 16),
                             static_cast<UINT16>(expected >> 16), Status,
                             0xFFFF, mismatches);
    }
    ++tested;
  }

  for (unsigned iteration = 0; iteration < 500000; ++iteration) {
    const UINT16 divisor = static_cast<UINT16>(NextRandom(random));
    const UINT16 high = static_cast<UINT16>(NextRandom(random));
    const UINT16 low = static_cast<UINT16>(NextRandom(random));
    const UINT16 initialStatus = (iteration & 1) ? 0 : 0xFFFF;
    const bool overflow = divisor == 0 || high >= divisor;
    const UINT32 dividend = (static_cast<UINT32>(high) << 16) | low;
    UINT16 expectedHigh = high;
    UINT16 expectedLow = low;
    UINT16 expectedStatus = initialStatus;
    if (overflow) {
      expectedStatus |= TMS_OVERFLOW;
    } else {
      expectedHigh = static_cast<UINT16>(dividend / divisor);
      expectedLow = static_cast<UINT16>(dividend % divisor);
      expectedStatus &= ~TMS_OVERFLOW;
    }

    PrepareDirect(0x3C40, divisor, high, initialStatus); // DIV R0,R1
    RawWriteWord(0x1004, low);
    StepCPU();
    const UINT16 actualHigh = RawReadWord(0x1002);
    const UINT16 actualLow = RawReadWord(0x1004);
    if (actualHigh != expectedHigh || actualLow != expectedLow ||
        Status != expectedStatus) {
      if (mismatches < 20) {
        std::printf(
            "MULDIV FAIL DIV divisor=>%04X dividend=>%04X%04X "
            "result=>%04X:%04X/%04X:%04X st=>%04X/%04X\n",
            divisor, high, low, actualHigh, actualLow, expectedHigh,
            expectedLow, Status, expectedStatus);
      }
      ++mismatches;
    }
    ++tested;
  }

  std::printf("Random multiply/divide: %llu cases, %llu mismatches\n",
              static_cast<unsigned long long>(tested),
              static_cast<unsigned long long>(mismatches));
  if (mismatches != 0)
    ++Failures;
}

void RunAddressingBattery() {
  Begin();
  RawWriteWord(0x1000, 0x3000); // R0
  RawWriteWord(0x3000, 0xA55A);
  RawWriteWord(0x0200, 0xC050); // MOV *R0,R1
  StepCPU();
  CheckHex("indirect source value", RawReadWord(0x1002), 0xA55A);
  CheckHex("indirect source leaves register", RawReadWord(0x1000), 0x3000);

  Begin();
  RawWriteWord(0x1000, 0x3000);
  RawWriteWord(0x3000, 0xA55A);
  RawWriteWord(0x0200, 0xC070); // MOV *R0+,R1
  StepCPU();
  CheckHex("word autoincrement source value", RawReadWord(0x1002), 0xA55A);
  CheckHex("word autoincrement advances by two", RawReadWord(0x1000),
           0x3002);

  Begin();
  RawWriteWord(0x1000, 0x3001);
  Memory[0x3001] = 0xA5;
  RawWriteWord(0x1002, 0x005A);
  RawWriteWord(0x0200, 0xD070); // MOVB *R0+,R1
  StepCPU();
  CheckHex("byte autoincrement source value", RawReadWord(0x1002), 0xA55A);
  CheckHex("byte autoincrement advances by one", RawReadWord(0x1000),
           0x3002);

  Begin();
  RawWriteWord(0x3000, 0xA55A);
  RawWriteWord(0x0200, 0xC060); // MOV @>3000,R1
  RawWriteWord(0x0202, 0x3000);
  StepCPU();
  CheckHex("symbolic source value", RawReadWord(0x1002), 0xA55A);
  CheckHex("symbolic source consumes extension", ProgramCounter, 0x0204);

  Begin();
  RawWriteWord(0x1004, 0x3000); // R2
  RawWriteWord(0x3010, 0xA55A);
  RawWriteWord(0x0200, 0xC062); // MOV @>0010(R2),R1
  RawWriteWord(0x0202, 0x0010);
  StepCPU();
  CheckHex("indexed source value", RawReadWord(0x1002), 0xA55A);

  Begin();
  RawWriteWord(0x1002, 0xA55A);
  RawWriteWord(0x0200, 0xC801); // MOV R1,@>3000
  RawWriteWord(0x0202, 0x3000);
  StepCPU();
  CheckHex("symbolic destination value", RawReadWord(0x3000), 0xA55A);

  Begin();
  RawWriteWord(0x3000, 0xA55A);
  RawWriteWord(0x0200, 0xC820); // MOV @>3000,@>4000
  RawWriteWord(0x0202, 0x3000);
  RawWriteWord(0x0204, 0x4000);
  StepCPU();
  CheckHex("dual extension source-before-destination", RawReadWord(0x4000),
           0xA55A);
  CheckHex("dual extension consumes both words", ProgramCounter, 0x0206);

  Begin();
  RawWriteWord(0x1000, 0x3000);
  RawWriteWord(0x3000, 0xA55A);
  RawWriteWord(0x3002, 0x5AA5);
  RawWriteWord(0x0200, 0xCC30); // MOV *R0+,*R0+
  StepCPU();
  CheckHex("shared autoincrement copies to next word", RawReadWord(0x3002),
           0xA55A);
  CheckHex("shared word autoincrement advances twice", RawReadWord(0x1000),
           0x3004);

  Begin();
  RawWriteWord(0x1000, 0x3000);
  Memory[0x3000] = 0xA5;
  Memory[0x3001] = 0x5A;
  RawWriteWord(0x0200, 0xDC30); // MOVB *R0+,*R0+
  StepCPU();
  CheckHex("shared byte autoincrement copies to next byte",
           static_cast<UINT16>((Memory[0x3000] << 8) | Memory[0x3001]),
           0xA5A5);
  CheckHex("shared byte autoincrement advances twice", RawReadWord(0x1000),
           0x3002);

  Begin();
  RawWriteWord(0x1002, 0x3000);
  RawWriteWord(0x0200, 0x0441); // B R1
  StepCPU();
  CheckHex("B register-direct uses register address", ProgramCounter, 0x1002);

  Begin();
  RawWriteWord(0x1002, 0x3001);
  RawWriteWord(0x0200, 0x0451); // B *R1
  StepCPU();
  CheckHex("B indirect uses value held in register", ProgramCounter, 0x3000);

  Begin();
  RawWriteWord(0x1002, 0x3000);
  RawWriteWord(0x0200, 0x0691); // BL *R1
  StepCPU();
  CheckHex("BL indirect target", ProgramCounter, 0x3000);
  CheckHex("BL saves return PC in R11", RawReadWord(0x1016), 0x0202);

  Begin();
  RawWriteWord(0x0200, 0x04A0); // X @>3000
  RawWriteWord(0x0202, 0x3000);
  RawWriteWord(0x0204, 0xA55A); // Immediate belongs to the outer stream.
  RawWriteWord(0x3000, 0x0201); // LI R1,>A55A
  StepCPU();
  CheckHex("X executes target instruction", RawReadWord(0x1002), 0xA55A);
  CheckHex("X immediate comes from outer instruction stream", ProgramCounter,
           0x0206);

  std::printf("Addressing battery: %d mismatch(es)\n", Failures);
}

void RunExhaustiveBranches() {
  enum Condition {
    Always,
    Lt,
    Le,
    Eq,
    He,
    Gt,
    Ne,
    Nc,
    Oc,
    No,
    Lo,
    Hi,
    Op,
  };
  struct BranchCase {
    const char *name;
    UINT16 opcode;
    Condition condition;
  } cases[] = {
      {"JMP", 0x1001, Always}, {"JLT", 0x1101, Lt},
      {"JLE", 0x1201, Le},     {"JEQ", 0x1301, Eq},
      {"JHE", 0x1401, He},     {"JGT", 0x1501, Gt},
      {"JNE", 0x1601, Ne},     {"JNC", 0x1701, Nc},
      {"JOC", 0x1801, Oc},     {"JNO", 0x1901, No},
      {"JL", 0x1A01, Lo},      {"JH", 0x1B01, Hi},
      {"JOP", 0x1C01, Op},
  };

  UINT64 tested = 0;
  UINT64 mismatches = 0;
  Begin();

  for (const BranchCase &test : cases) {
    for (unsigned rawStatus = 0; rawStatus <= 0xFFFF; ++rawStatus) {
      const UINT16 st = static_cast<UINT16>(rawStatus);
      bool taken = false;
      switch (test.condition) {
      case Always: taken = true; break;
      case Lt: taken = !(st & (TMS_ARITHMETIC | TMS_EQUAL)); break;
      case Le: taken = !(st & TMS_LOGICAL) || (st & TMS_EQUAL); break;
      case Eq: taken = st & TMS_EQUAL; break;
      case He: taken = st & (TMS_LOGICAL | TMS_EQUAL); break;
      case Gt: taken = st & TMS_ARITHMETIC; break;
      case Ne: taken = !(st & TMS_EQUAL); break;
      case Nc: taken = !(st & TMS_CARRY); break;
      case Oc: taken = st & TMS_CARRY; break;
      case No: taken = !(st & TMS_OVERFLOW); break;
      case Lo: taken = !(st & (TMS_LOGICAL | TMS_EQUAL)); break;
      case Hi: taken = (st & TMS_LOGICAL) && !(st & TMS_EQUAL); break;
      case Op: taken = st & TMS_PARITY; break;
      }

      PrepareDirect(test.opcode, 0, 0, st);
      StepCPU();
      const UINT16 expectedPC = taken ? 0x0204 : 0x0202;
      const UINT32 expectedCycles = taken ? 10 : 8;
      if (ProgramCounter != expectedPC || Status != st ||
          ClockCycleCounter != expectedCycles) {
        if (mismatches < 20) {
          std::printf(
              "BRANCH FAIL %s st=>%04X pc=>%04X/%04X cycles=%llu/%u\n",
              test.name, st, ProgramCounter, expectedPC,
              static_cast<unsigned long long>(ClockCycleCounter),
              expectedCycles);
        }
        ++mismatches;
      }
      ++tested;
    }
  }

  std::printf("Exhaustive branches: %llu cases, %llu mismatches\n",
              static_cast<unsigned long long>(tested),
              static_cast<unsigned long long>(mismatches));
  if (mismatches != 0)
    ++Failures;
}

void RunExhaustiveShifts();

void RunCoreSemanticBattery() {
  RunExhaustiveUnary();
  RunExhaustiveByteOps();
  RunExhaustiveShifts();
  RunRandomWordOps();
  RunRandomImmediateOps();
  RunRandomMultiplyDivide();
}

void RunExhaustiveShifts() {
  struct ShiftCase {
    const char *name;
    UINT16 opcode;
  } cases[] = {
      {"SRA", 0x0801},
      {"SRL", 0x0901},
      {"SLA", 0x0A01},
      {"SRC", 0x0B01},
  };

  UINT64 tested = 0;
  UINT64 mismatches = 0;
  Begin();

  for (const ShiftCase &test : cases) {
    for (unsigned count = 1; count <= 16; ++count) {
      const UINT16 encodedCount = count == 16 ? 0 : count;
      const UINT16 opcode = test.opcode | (encodedCount << 4);

      for (unsigned input = 0; input <= 0xFFFF; ++input) {
        UINT16 expected = static_cast<UINT16>(input);
        bool carry = false;
        bool overflow = false;

        for (unsigned bit = 0; bit < count; ++bit) {
          switch (test.opcode & 0x0F00) {
          case 0x0800: {
            carry = (expected & 1) != 0;
            const UINT16 sign = expected & 0x8000;
            expected = static_cast<UINT16>((expected >> 1) | sign);
            break;
          }
          case 0x0900:
            carry = (expected & 1) != 0;
            expected = static_cast<UINT16>(expected >> 1);
            break;
          case 0x0A00: {
            carry = (expected & 0x8000) != 0;
            expected = static_cast<UINT16>(expected << 1);
            if (carry != ((expected & 0x8000) != 0))
              overflow = true;
            break;
          }
          case 0x0B00:
            carry = (expected & 1) != 0;
            expected = static_cast<UINT16>((expected >> 1) |
                                           (carry ? 0x8000 : 0));
            break;
          }
        }

        const UINT16 preserved = static_cast<UINT16>(
            0xFFFF & ~(TMS_LOGICAL | TMS_ARITHMETIC | TMS_EQUAL |
                       TMS_CARRY |
                       ((test.opcode & 0x0F00) == 0x0A00 ? TMS_OVERFLOW
                                                        : 0)));
        const UINT16 expectedStatus = static_cast<UINT16>(
            preserved | ExpectedLAE(expected) |
            (carry ? TMS_CARRY : 0) | (overflow ? TMS_OVERFLOW : 0));

        WorkspacePtr = 0x1000;
        ProgramCounter = 0x0200;
        Status = 0xFFFF;
        ClockCycleCounter = 0;
        InstructionCounter = 0;
        RawWriteWord(0x0200, opcode);
        RawWriteWord(0x1000, 0); // R0=0 selects a count of 16 when needed.
        RawWriteWord(0x1002, static_cast<UINT16>(input));
        StepCPU();

        const UINT16 actual = RawReadWord(0x1002);
        if (actual != expected || Status != expectedStatus) {
          if (mismatches < 20) {
            std::printf(
                "SHIFT FAIL %s input=>%04X count=%u result=>%04X/%04X "
                "st=>%04X/%04X\n",
                test.name, input, count, actual, expected, Status,
                expectedStatus);
          }
          ++mismatches;
        }
        ++tested;
      }
    }
  }

  std::printf("Exhaustive shifts: %llu cases, %llu mismatches\n",
              static_cast<unsigned long long>(tested),
              static_cast<unsigned long long>(mismatches));
  if (mismatches != 0)
    ++Failures;
}

} // namespace

int main(int argc, char **argv) {
  InitCPU();

  if (argc == 2 && std::strcmp(argv[1], "ub-src") == 0) {
    RunUBSRC();
    return 0;
  }
  if (argc == 2 && std::strcmp(argv[1], "ub-sla") == 0) {
    RunUBSLA();
    return 0;
  }
  if (argc == 2 && std::strcmp(argv[1], "disasm-wrap") == 0) {
    RunDisasmWrap();
    return 0;
  }
  if (argc == 2 && std::strcmp(argv[1], "exhaustive-shifts") == 0) {
    RunExhaustiveShifts();
    return Failures == 0 ? 0 : 1;
  }
  if (argc == 2 && std::strcmp(argv[1], "semantic-core") == 0) {
    RunCoreSemanticBattery();
    return Failures == 0 ? 0 : 1;
  }
  if (argc == 2 && std::strcmp(argv[1], "addressing") == 0) {
    RunAddressingBattery();
    return Failures == 0 ? 0 : 1;
  }
  if (argc == 2 && std::strcmp(argv[1], "branches") == 0) {
    RunExhaustiveBranches();
    return Failures == 0 ? 0 : 1;
  }
  if (argc == 2 && std::strcmp(argv[1], "decode") == 0) {
    RunDecodeSweep();
    return Failures == 0 ? 0 : 1;
  }

  TestInterruptEntryUsesArchitecturalPC();
  TestResetInterruptClearsStatus();
  TestResetBypassesInterruptDeferral();
  TestInterruptMaskingAndPriority();
  TestRegisterMovTiming();
  TestAddressingModeTiming();
  TestStepEvents();
  TestWaitStateTiming();
  TestInspectionAndResetAPI();
  TestVariableTiming();
  TestCRUStatusEffects();
  TestBLWPDefersInterrupt();
  TestXOPDefersInterrupt();
  TestContextSwitchState();
  TestArchitecturalAddressAlignment();
  TestDecodeAndDisassemblyEdges();

  std::printf("\n%d probe assertion(s) failed\n", Failures);
  return Failures == 0 ? 0 : 1;
}
