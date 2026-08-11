#pragma once

#include <cstdint>

#include "math.h"

namespace frontier {

// Opaque application data carried by one renderable LOD node. Values are not
// identities and need not be unique.
using UserPayload = uint64_t;
inline constexpr UserPayload kSentinelPayload = ~0ull;
inline constexpr uint32_t kInvalidIndex = 0xFFFFFFFFu;

// Translation plus positive uniform scale, the transform contract shared by
// top-level instances and mounted subtrees.
struct Transform
{
    float4 pos = float4::point(0.0f, 0.0f, 0.0f);
    float scale = 1.0f;
};
static_assert(sizeof(Transform) == 32, "transform layout changed");

// One renderable LOD choice. The same descriptor is used for a TLAS-owned root
// and for nodes authored into serialized subtree bytes. A mountable authored
// node must remain a leaf; its child definition and placement are supplied
// explicitly to SpatialDatabase::mountSubtree().
struct NodeDesc
{
    UserPayload payload = 0;
    float geometricError = 0.0f;
    bool mountable = false;
    AABB bounds = AABB::empty();
};
static_assert(sizeof(NodeDesc) == 48,
              "NodeDesc must use the padding before its aligned bounds");

} // namespace frontier
