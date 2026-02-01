//----------------------------------------------------------------------------
// TMS9900 Trace - GDB Remote Serial Protocol stub
//
// gdb_stub.hpp - Interface for GDB RSP server
//----------------------------------------------------------------------------

#ifndef GDB_STUB_HPP_
#define GDB_STUB_HPP_

#include "types.hpp"
#include <vector>

//----------------------------------------------------------------------------
// Periodic timer interrupt (shared between main.cpp and gdb_stub.cpp)
//----------------------------------------------------------------------------

struct PeriodicTimer
{
    int irqLevel;
    UINT64 periodCycles;
    UINT64 nextTriggerCycle;
};

//----------------------------------------------------------------------------
// Start GDB RSP server on the given TCP port.
// Blocks until GDB disconnects or sends kill/detach.
// Returns 0 on clean exit, 1 on error.
//----------------------------------------------------------------------------

int gdb_main(int port, std::vector<PeriodicTimer>& timers);

#endif // GDB_STUB_HPP_
