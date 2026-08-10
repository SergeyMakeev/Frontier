#pragma once

#include <cstdint>

#include "math.h"

namespace frontier {

// Opaque application data carried by one renderable LOD node. Values are not
// identities and need not be unique.
using UserPayload = uint64_t;
inline constexpr UserPayload kSentinelPayload = ~0ull;
inline constexpr uint32_t kInvalidIndex = 0xFFFFFFFFu;

// Stable package identity for a reusable subtree. Runtime handles are
// deliberately separate because they are database-local and generation
// stamped.
struct SubtreeKey
{
    uint64_t value = 0;

    bool valid() const { return value != 0; }
    friend bool operator==(SubtreeKey, SubtreeKey) = default;
};
static_assert(sizeof(SubtreeKey) == 8, "subtree key must stay 64 bits");

// Translation plus positive uniform scale, the transform contract shared by
// top-level instances and mounted subtrees.
struct Transform
{
    float4 pos = float4::point(0.0f, 0.0f, 0.0f);
    float scale = 1.0f;
};
static_assert(sizeof(Transform) == 32, "transform layout changed");

// One renderable LOD choice. The same descriptor is used for a TLAS-owned root
// and for nodes authored into a Subtree. A valid childSubtree turns the node
// into an expansion site; such a node cannot also have local children.
struct NodeDesc
{
    UserPayload payload = 0;
    float geometricError = 0.0f;
    AABB bounds = AABB::empty();
    SubtreeKey childSubtree{};
    Transform childTransform{};
};

} // namespace frontier
