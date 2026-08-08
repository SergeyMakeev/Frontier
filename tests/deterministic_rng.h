#pragma once

// Cross-platform deterministic random numbers for tests and benchmarks.
//
// C++ standard engines have defined integer output, but distributions and
// std::shuffle are allowed to vary between standard-library implementations.
// These small helpers own the complete mapping so a seed produces the same
// sequence with MSVC, libc++, and libstdc++.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <type_traits>

namespace hlodtest {

class DeterministicRng
{
public:
    explicit DeterministicRng(uint32_t seed) : state_(seed ? seed : kZeroSeed) {}

    uint32_t next()
    {
        state_ ^= state_ << 13;
        state_ ^= state_ >> 17;
        state_ ^= state_ << 5;
        return state_;
    }

    uint32_t operator()() { return next(); }

    // [0, upperExclusive). Rejection removes modulo bias while keeping the
    // algorithm completely specified.
    uint32_t index(uint32_t upperExclusive)
    {
        if (upperExclusive == 0) return 0;
        const uint32_t threshold = uint32_t(0u - upperExclusive) % upperExclusive;
        for (;;)
        {
            const uint32_t value = next();
            if (value >= threshold) return value % upperExclusive;
        }
    }

    // [lo, hi), using the high 24 bits so conversion to float is exact. Keep
    // multiplication and addition in separate statements to prevent a target
    // with FMA from changing the last bit of the generated coordinate.
    float uniform(float lo, float hi)
    {
        const float unit = float(next() >> 8) * (1.0f / 16777216.0f);
        const float span = hi - lo;
        const float scaled = span * unit;
        return lo + scaled;
    }

private:
    static constexpr uint32_t kZeroSeed = 0x6d2b79f5u;
    uint32_t state_;
};

class DeterministicUniformFloat
{
public:
    DeterministicUniformFloat(float lo, float hi) : lo_(lo), hi_(hi) {}
    float operator()(DeterministicRng& rng) const { return rng.uniform(lo_, hi_); }

private:
    float lo_;
    float hi_;
};

template <class Int = int>
class DeterministicUniformInt
{
    static_assert(std::is_integral_v<Int>);

public:
    DeterministicUniformInt(Int lo, Int hi) : lo_(lo), span_(uint32_t(hi - lo) + 1u)
    {}

    Int operator()(DeterministicRng& rng) const
    {
        return Int(lo_ + Int(rng.index(span_)));
    }

private:
    Int      lo_;
    uint32_t span_;
};

template <class RandomIt>
void deterministicShuffle(RandomIt first, RandomIt last, DeterministicRng& rng)
{
    using Difference = typename std::iterator_traits<RandomIt>::difference_type;
    const Difference count = last - first;
    for (Difference remaining = count; remaining > 1; --remaining)
    {
        const uint32_t choice = rng.index(uint32_t(remaining));
        std::iter_swap(first + (remaining - 1), first + Difference(choice));
    }
}

} // namespace hlodtest
