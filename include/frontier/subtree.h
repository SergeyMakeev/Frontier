#pragma once

#include <cstddef>
#include <span>
#include <utility>

#include "config.h"

namespace frontier {

inline constexpr size_t kSubtreeByteAlignment = 64;

// Owning, traversal-ready serialized subtree bytes. This is deliberately
// only a byte array: building, saving, loading, and database registration all
// use the exact same representation. The copied allocation context makes the
// buffer self-contained and keeps custom allocator state valid after moves.
class SubtreeBytes
{
public:
    SubtreeBytes() = default;
    explicit SubtreeBytes(
        size_t size, const FrontierContext& context = defaultContext());
    ~SubtreeBytes() { release(); }

    SubtreeBytes(const SubtreeBytes& other);
    SubtreeBytes& operator=(const SubtreeBytes& other);
    SubtreeBytes(SubtreeBytes&& other) noexcept { moveFrom(other); }
    SubtreeBytes& operator=(SubtreeBytes&& other) noexcept;

    std::byte* data() { return data_; }
    const std::byte* data() const { return data_; }
    size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }

    std::span<std::byte> bytes() { return {data_, size_}; }
    std::span<const std::byte> bytes() const { return {data_, size_}; }

private:
    void release();
    void moveFrom(SubtreeBytes& other) noexcept;

    std::byte* data_ = nullptr;
    size_t size_ = 0;
    FrontierContext context_ = defaultContext();
};

} // namespace frontier
