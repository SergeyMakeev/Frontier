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

// Exact six-float bounds storage for cold authoring data. Runtime traversal
// keeps using the SIMD-friendly AABB and WideBounds representations.
struct ScalarAABB
{
    struct XYZ
    {
        float x;
        float y;
        float z;
    };

    XYZ mn = {FLT_MAX, FLT_MAX, FLT_MAX};
    XYZ mx = {-FLT_MAX, -FLT_MAX, -FLT_MAX};

    ScalarAABB() = default;
    ScalarAABB(const AABB& bounds) noexcept { *this = bounds; }

    ScalarAABB& operator=(const AABB& bounds) noexcept
    {
        mn = {bounds.mn.x, bounds.mn.y, bounds.mn.z};
        mx = {bounds.mx.x, bounds.mx.y, bounds.mx.z};
        return *this;
    }

    AABB toAABB() const noexcept
    {
        return AABB::fromMinMax(float4::point(mn.x, mn.y, mn.z),
                                float4::point(mx.x, mx.y, mx.z));
    }

    operator AABB() const noexcept { return toAABB(); }
    bool isEmpty() const noexcept { return mn.x > mx.x; }
};
static_assert(sizeof(ScalarAABB) == 24,
              "ScalarAABB must contain exactly six floats");

// One renderable LOD choice. The same descriptor is used for a TLAS-owned root
// and for nodes authored into serialized subtree bytes. A mountable authored
// node must remain a leaf; its child definition and placement are supplied
// explicitly to SpatialDatabase::mountSubtree().
struct NodeDesc
{
    UserPayload payload = 0;
    float geometricError = 0.0f;
    bool mountable = false;
    ScalarAABB bounds = AABB::empty();
};
static_assert(sizeof(NodeDesc) == 40,
              "NodeDesc must stay at five eight-byte words");

} // namespace frontier
