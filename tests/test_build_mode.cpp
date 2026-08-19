#include <gtest/gtest.h>

#include "frontier/math.h"

#ifdef NDEBUG
  #error "Frontier unit tests must be compiled as Debug so internal assertions execute"
#endif

#if !FRONTIER_CONTRACT_CHECKS
  #error "Frontier unit tests require FRONTIER_CONTRACT_CHECKS=1"
#endif

#if FRONTIER_SSE2_ONLY
  #if !defined(FRONTIER_SIMD_SSE2)
    #error "FRONTIER_SSE2_ONLY must select the SSE2 intrinsic backend"
  #endif
  #if defined(FRONTIER_SIMD_SSE41) || defined(FRONTIER_SIMD_AVX2)
    #error "FRONTIER_SSE2_ONLY selected a higher-ISA intrinsic backend"
  #endif
  #if defined(__SSE3__) || defined(__SSSE3__) || defined(__SSE4_1__) || \
      defined(__SSE4_2__) || defined(__AVX__) || defined(__AVX2__) || \
      defined(__FMA__)
    #error "FRONTIER_SSE2_ONLY compiler flags expose a higher x86 ISA"
  #endif
#endif

TEST(BuildProfile, DebugAssertionsAndContractChecksAreEnabled)
{
    SUCCEED();
}

#if FRONTIER_SSE2_ONLY
TEST(BuildProfile, Sse2OnlySelectsTheBaselineIntrinsicBackend)
{
    EXPECT_EQ(FRONTIER_SIMD_SSE2, 1);
}
#endif
