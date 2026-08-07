#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace hlod {

// Retained-capacity, append-only storage for trivially copyable hot-path
// output. It deliberately omits middle insertion, erasure, and shrinking.
template <class T>
class AppendBuffer
{
    static_assert(std::is_trivially_copyable_v<T>);
    static_assert(alignof(T) <= alignof(std::max_align_t));

public:
    using value_type = T;
    using iterator = T*;
    using const_iterator = const T*;

    AppendBuffer() = default;
    AppendBuffer(const AppendBuffer& other) { append(other.data_, other.size_); }

    AppendBuffer(AppendBuffer&& other) noexcept
        : data_(std::exchange(other.data_, nullptr)),
          size_(std::exchange(other.size_, 0)),
          capacity_(std::exchange(other.capacity_, 0))
    {}

    AppendBuffer& operator=(const AppendBuffer& other)
    {
        if (this != &other)
        {
            reserve(other.size_);
            if (other.size_)
                std::memcpy(data_, other.data_, size_t(other.size_) * sizeof(T));
            size_ = other.size_;
        }
        return *this;
    }

    AppendBuffer& operator=(AppendBuffer&& other) noexcept
    {
        if (this != &other)
        {
            std::free(data_);
            data_ = std::exchange(other.data_, nullptr);
            size_ = std::exchange(other.size_, 0);
            capacity_ = std::exchange(other.capacity_, 0);
        }
        return *this;
    }

    ~AppendBuffer() { std::free(data_); }

    void clear() noexcept { size_ = 0; }

    void reserve(size_t requested)
    {
        if (requested > maxCapacity())
            throw std::length_error("AppendBuffer capacity exceeded");
        if (requested > capacity_)
            reallocate(uint32_t(requested));
    }

    void push_back(T value)
    {
        ensureCapacity(checkedSize(1));
        std::construct_at(data_ + size_, value);
        ++size_;
    }

    template <class... Args>
    T& emplace_back(Args&&... args)
    {
        ensureCapacity(checkedSize(1));
        T* out = std::construct_at(data_ + size_,
                                   std::forward<Args>(args)...);
        ++size_;
        return *out;
    }

    // Source must not point into this buffer: growth may relocate data before
    // it is copied. Cache runs satisfy that contract.
    void append(const T* source, size_t count)
    {
        if (count == 0) return;
        const uint32_t required = checkedSize(count);
        ensureCapacity(required);

        T* out = data_ + size_;
        if (count <= 4)
        {
            switch (count)
            {
            case 4: std::construct_at(out + 3, source[3]); [[fallthrough]];
            case 3: std::construct_at(out + 2, source[2]); [[fallthrough]];
            case 2: std::construct_at(out + 1, source[1]); [[fallthrough]];
            case 1: std::construct_at(out, source[0]);
            }
        }
        else
        {
            std::memcpy(out, source, count * sizeof(T));
        }
        size_ = required;
    }

    T* data() noexcept { return data_; }
    const T* data() const noexcept { return data_; }
    T* begin() noexcept { return data_; }
    const T* begin() const noexcept { return data_; }
    const T* cbegin() const noexcept { return data_; }
    T* end() noexcept { return size_ ? data_ + size_ : data_; }
    const T* end() const noexcept { return size_ ? data_ + size_ : data_; }
    const T* cend() const noexcept { return size_ ? data_ + size_ : data_; }
    T& operator[](size_t i) noexcept { return data_[i]; }
    const T& operator[](size_t i) const noexcept { return data_[i]; }
    T& front() noexcept { return data_[0]; }
    const T& front() const noexcept { return data_[0]; }
    T& back() noexcept { return data_[size_ - 1]; }
    const T& back() const noexcept { return data_[size_ - 1]; }
    size_t size() const noexcept { return size_; }
    size_t capacity() const noexcept { return capacity_; }
    bool empty() const noexcept { return size_ == 0; }

private:
    static constexpr uint32_t maxCapacity() noexcept
    {
        constexpr size_t byAddressSpace =
            std::numeric_limits<size_t>::max() / sizeof(T);
        constexpr size_t bySizeField =
            std::numeric_limits<uint32_t>::max();
        return uint32_t(byAddressSpace < bySizeField ? byAddressSpace : bySizeField);
    }

    uint32_t checkedSize(size_t added) const
    {
        if (added > size_t(maxCapacity() - size_))
            throw std::length_error("AppendBuffer capacity exceeded");
        return size_ + uint32_t(added);
    }

    void ensureCapacity(uint32_t required)
    {
        if (required <= capacity_) return;
        uint64_t grown = capacity_ ? uint64_t(capacity_) * 3 / 2 : 8;
        if (grown < required) grown = required;
        if (grown > maxCapacity()) grown = maxCapacity();
        reallocate(uint32_t(grown));
    }

    void reallocate(uint32_t newCapacity)
    {
        void* next = std::realloc(data_, size_t(newCapacity) * sizeof(T));
        if (!next) throw std::bad_alloc();
        data_ = static_cast<T*>(next);
        capacity_ = newCapacity;
    }

    T* data_ = nullptr;
    uint32_t size_ = 0;
    uint32_t capacity_ = 0;
};

} // namespace hlod
