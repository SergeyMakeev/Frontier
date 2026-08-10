#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "config.h"
#include "detail/subtree_data.h"
#include "node.h"

namespace frontier {

class SpatialDatabase;
class SubtreeBuilder;

// A complete immutable, reusable hierarchy fragment. Its implicit parent is
// not a node in this object: at runtime it is the renderable TLAS root or
// expansion node passed to mountSubtree(). The same type represents owned and
// explicitly borrowed serialized storage.
class Subtree
{
public:
    Subtree() = default;
    ~Subtree() { release(); }

    Subtree(Subtree&& other) noexcept { moveFrom(other); }
    Subtree& operator=(Subtree&& other) noexcept
    {
        if (this != &other)
        {
            release();
            moveFrom(other);
        }
        return *this;
    }
    Subtree(const Subtree&) = delete;
    Subtree& operator=(const Subtree&) = delete;

    // Validate and copy serialized storage.
    static Subtree fromBytes(
        const void* blob, size_t bytes,
        const FrontierContext& context = defaultContext());

    // Validate and borrow serialized storage without copying. The storage must
    // remain alive and suitably aligned until this object, or the database it
    // is moved into, releases it.
    static Subtree borrow(const void* blob, size_t bytes);

    Subtree clone(const FrontierContext& context = defaultContext()) const;

    bool valid() const { return parent_ != nullptr; }
    SubtreeKey key() const { return key_; }
    uint32_t nodeCount() const { return packedNodeCount_ ? packedNodeCount_ - 1 : 0; }
    AABB bounds() const { return valid() ? bbox_[0] : AABB::empty(); }
    const void* data() const { return base_; }
    size_t byteSize() const { return byteSize_; }
    std::span<const SubtreeKey> dependencies() const
    {
        return {dependencies_, dependencyCount_};
    }

private:
    friend class SpatialDatabase;
    friend class SubtreeBuilder;

    static Subtree adopt(void* blob, size_t bytes,
                         const FrontierContext& context);
    static Subtree fromValidatedBytes(const void* blob, size_t bytes,
                                      const FrontierContext* owner);
    static void validate(const void* blob, size_t bytes);
    void bind(const void* blob, size_t bytes);
    void release();
    void moveFrom(Subtree& other) noexcept;

    uint32_t packedNodeCount() const { return packedNodeCount_; }
    uint32_t wideCount() const { return wideCount_; }
    uint32_t childCount(uint32_t i) const
    {
        return detail::metaChildCount(meta_[i]);
    }
    bool isExpansion(uint32_t i) const
    {
        return detail::metaIsExpansion(meta_[i]);
    }
    uint32_t wideOffset(uint32_t i) const
    {
        return detail::metaWideOffset(meta_[i]);
    }
    uint32_t wideBlockCount(uint32_t i) const
    {
        return (childCount(i) + kWide - 1) / kWide;
    }
    uint32_t validLanes(uint32_t block) const
    {
        return detail::blockValidLanes(blockMask_[block]);
    }
    uint32_t leafLanes(uint32_t block) const
    {
        return detail::blockLeafLanes(blockMask_[block]);
    }
    detail::WideBoundsRef wideBounds() const
    {
        return detail::WideBoundsRef::interleaved(wide_);
    }
    std::span<const detail::SubtreeExpansion> expansions() const
    {
        return {expansions_, expansionCount_};
    }

    const uint32_t* parent_ = nullptr;
    const uint32_t* subtreeSize_ = nullptr;
    const uint32_t* meta_ = nullptr;
    const detail::WideBlock* wide_ = nullptr;
    const uint32_t* blockMask_ = nullptr;
    const UserPayload* payload_ = nullptr;
    const AABB* bbox_ = nullptr;
    const float* geometricError_ = nullptr;
    const SubtreeKey* dependencies_ = nullptr;
    const detail::SubtreeExpansion* expansions_ = nullptr;

    const std::byte* base_ = nullptr;
    uint32_t packedNodeCount_ = 0;
    uint32_t wideCount_ = 0;
    uint32_t dependencyCount_ = 0;
    uint32_t expansionCount_ = 0;
    size_t byteSize_ = 0;
    SubtreeKey key_{};
    const FrontierContext* context_ = nullptr;
};

} // namespace frontier
