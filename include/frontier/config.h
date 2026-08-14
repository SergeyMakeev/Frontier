#pragma once
// Compile-time configuration, diagnostics policy, and the host integration
// context (allocation + task parallelism).
//
// Everything here is designed to be overridden by the embedding project by
// defining the macro BEFORE including any frontier header:
//
//   #define FRONTIER_FATAL(msg) MyEngine::Panic(msg)   // optional host panic
//   #include "frontier/spatial_database.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>

#define FRONTIER_VERSION_MAJOR 0
#define FRONTIER_VERSION_MINOR 7
#define FRONTIER_VERSION_PATCH 0
#define FRONTIER_VERSION_STRING "0.7.0"

// Complete serialized-subtree validation is enabled by default. Defining this
// to 0 removes the linear topology/data scan from registration; constant-time
// header, layout, and root-range checks remain. Disabled builds must register
// only trusted bytes produced by a compatible Frontier builder.
#ifndef FRONTIER_VALIDATE_SUBTREES
  #define FRONTIER_VALIDATE_SUBTREES 1
#endif

#if FRONTIER_VALIDATE_SUBTREES != 0 && FRONTIER_VALIDATE_SUBTREES != 1
  #error "FRONTIER_VALIDATE_SUBTREES must be 0 or 1"
#endif

// Caller-contract checks are enabled by default in every build. Trusted-input
// performance builds may disable them explicitly; doing so makes violating a
// documented precondition undefined behavior. Debug-only internal assertions
// remain controlled independently by NDEBUG.
#ifndef FRONTIER_CONTRACT_CHECKS
  #define FRONTIER_CONTRACT_CHECKS 1
#endif

#if FRONTIER_CONTRACT_CHECKS != 0 && FRONTIER_CONTRACT_CHECKS != 1
  #error "FRONTIER_CONTRACT_CHECKS must be 0 or 1"
#endif

// Build-wide branching factor for both BLAS fragments and the dynamic TLAS.
// Four maps to one 128-bit SIMD vector; eight maps to one AVX2 vector or two
// SSE2/NEON vectors. Serialized subtree bytes record this value and cannot be
// registered by a build using the other width. Without an explicit definition,
// AVX2 selects eight and four-wide SIMD/scalar targets select four.
#ifndef FRONTIER_BVH_WIDTH
  #if defined(__AVX2__) && !defined(FRONTIER_FORCE_SCALAR)
    #define FRONTIER_BVH_WIDTH 8
  #else
    #define FRONTIER_BVH_WIDTH 4
  #endif
#endif

#if FRONTIER_BVH_WIDTH != 4 && FRONTIER_BVH_WIDTH != 8
  #error "FRONTIER_BVH_WIDTH must be 4 or 8"
#endif

// ---------------------------------------------------------------------------
// Diagnostics
//
// FRONTIER_FATAL fires on a TRUE CONTRACT VIOLATION only — a caller bug that no
// correct program can trigger (mounting on a node that is not mountable or
// submitting non-finite bounds). It must not
// return.
//
// Expected runtime races are NOT fatal and never route here: a handle that
// went stale because its subtree was collected mid-stream makes mutating calls
// quiet no-ops and queries report absent. That distinction is the whole
// reason streaming callers can be written without locks.
//
// With C++ exceptions enabled, the default throws, which keeps the contract
// tests expressive. With compiler exception support disabled, the default
// aborts. Projects may define FRONTIER_FATAL to their own non-returning panic
// handler in either build mode.
// ---------------------------------------------------------------------------

#ifndef FRONTIER_FATAL
  #if defined(__cpp_exceptions) || defined(__EXCEPTIONS) || defined(_CPPUNWIND)
    #include <stdexcept>
    #include <string>
    #define FRONTIER_FATAL(msg) throw ::std::logic_error(::std::string("frontier: ") + (msg))
  #else
    #define FRONTIER_FATAL(msg)                                               \
        do                                                                    \
        {                                                                     \
            (void)sizeof(msg);                                                \
            ::std::abort();                                                   \
        } while (false)
  #endif
#endif

#if FRONTIER_CONTRACT_CHECKS
  #define FRONTIER_CHECK(cond, msg)                                             \
      do                                                                        \
      {                                                                         \
          if (!(cond)) FRONTIER_FATAL(msg);                                     \
      } while (false)
#else
  // Keep the condition syntactically checked and suppress variables used only
  // by contracts, but generate no executable code.
  #define FRONTIER_CHECK(cond, msg)                                             \
      do                                                                        \
      {                                                                         \
          (void)sizeof(cond);                                                    \
      } while (false)
#endif

// Invariants that are expensive to check and cannot fail for a correct
// library: compiled out in release builds.
#ifndef FRONTIER_ASSERT
  #ifdef NDEBUG
    #define FRONTIER_ASSERT(cond, msg) ((void)0)
  #else
    #define FRONTIER_ASSERT(cond, msg)                                          \
        do                                                                      \
        {                                                                       \
            if (!(cond)) FRONTIER_FATAL(msg);                                   \
        } while (false)
  #endif
#endif

namespace frontier {

// ---------------------------------------------------------------------------
// Host context: subtree-blob allocation and optional task parallelism.
//
// SubtreeBuilder and SubtreeBytes use alloc/free.
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
