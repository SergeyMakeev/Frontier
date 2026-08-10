#pragma once
// Compile-time configuration, diagnostics policy, and the host integration
// context (allocation + task parallelism).
//
// Everything here is designed to be overridden by the embedding project by
// defining the macro BEFORE including any frontier header:
//
//   #define FRONTIER_FATAL(msg) MyEngine::Panic(msg)   // exceptions-off builds
//   #include "frontier/spatial_database.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>

#define FRONTIER_VERSION_MAJOR 0
#define FRONTIER_VERSION_MINOR 6
#define FRONTIER_VERSION_PATCH 0
#define FRONTIER_VERSION_STRING "0.6.0"

// ---------------------------------------------------------------------------
// Diagnostics
//
// FRONTIER_FATAL fires on a TRUE CONTRACT VIOLATION only — a caller bug that no
// correct program can trigger (mounting on a node that is not an expansion
// point or submitting non-finite bounds). It must not
// return.
//
// Expected runtime races are NOT fatal and never route here: a handle that
// went stale because its subtree was collected mid-stream makes mutating calls
// quiet no-ops and queries report absent. That distinction is the whole
// reason streaming callers can be written without locks.
//
// The default throws, which keeps the contract tests expressive. Projects
// building without exceptions should define FRONTIER_FATAL to abort or to their
// own panic handler.
// ---------------------------------------------------------------------------

#ifndef FRONTIER_FATAL
  #include <stdexcept>
  #include <string>
  #define FRONTIER_FATAL(msg) throw ::std::logic_error(::std::string("frontier: ") + (msg))
#endif

#define FRONTIER_CHECK(cond, msg)                                                  \
    do                                                                         \
    {                                                                          \
        if (!(cond)) FRONTIER_FATAL(msg);                                          \
    } while (false)

// Invariants that are expensive to check and cannot fail for a correct
// library: compiled out in release builds.
#ifndef FRONTIER_ASSERT
  #ifdef NDEBUG
    #define FRONTIER_ASSERT(cond, msg) ((void)0)
  #else
    #define FRONTIER_ASSERT(cond, msg) FRONTIER_CHECK(cond, msg)
  #endif
#endif

namespace frontier {

// ---------------------------------------------------------------------------
// Host context: subtree-blob allocation and optional task parallelism.
//
// SubtreeBuilder and owned Subtree factories use alloc/free.
// SpatialDatabase uses the blocking parallelFor callback and workerCount for an enabled
// uncached parallel selection. Other retained runtime storage is library-owned.
// ---------------------------------------------------------------------------

using AllocFn = void* (*)(size_t bytes, size_t alignment, void* user);
using FreeFn  = void (*)(void* ptr, void* user);

// Invoked as fn(i, payload) for i in [0, count). An implementation may run
// the calls on any thread and in any order, but must not return until all of
// them have completed. The default runs them serially on the calling thread.
using ParallelForFn = void (*)(uint32_t count, void (*fn)(uint32_t i, void* payload),
                               void* payload, void* user);

void* defaultAlloc(size_t bytes, size_t alignment, void* user);
void  defaultFree(void* ptr, void* user);
void  defaultParallelFor(uint32_t count, void (*fn)(uint32_t i, void* payload),
                         void* payload, void* user);

struct FrontierContext
{
    AllocFn       alloc       = &defaultAlloc;
    FreeFn        free        = &defaultFree;
    ParallelForFn parallelFor = &defaultParallelFor;

    // Upper bound on how many of parallelFor's calls can run concurrently.
    // Used to size per-worker output buffers; 1 keeps selectFrontier serial no
    // matter what parallelFor does.
    uint32_t workerCount = 1;

    void* user = nullptr;
};

// Process-wide default (malloc/free, serial). Returned by reference so it can
// be handed to anything expecting a context.
const FrontierContext& defaultContext();

} // namespace frontier
