#include <gtest/gtest.h>

#include "frontier/config.h"

#ifdef NDEBUG
  #error "Frontier unit tests must be compiled as Debug so internal assertions execute"
#endif

#if !FRONTIER_CONTRACT_CHECKS
  #error "Frontier unit tests require FRONTIER_CONTRACT_CHECKS=1"
#endif

TEST(BuildProfile, DebugAssertionsAndContractChecksAreEnabled)
{
    SUCCEED();
}
