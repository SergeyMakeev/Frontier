#pragma once

#include <concepts>
#include <cstdint>
#include <type_traits>

#include "math.h"

namespace frontier {

// Opaque application render-payload identifier carried by one LOD node. Equal
// values may identify the same render resources, but readiness is tracked by
// registered definition node rather than by payload value. Applications may
// override both macros build-wide; for example:
//
//   FRONTIER_USER_PAYLOAD=void*
//   FRONTIER_INVALID_PAYLOAD=nullptr
//
// The fixed eight-byte contract preserves NodeDesc and serialized layouts.
#ifndef FRONTIER_USER_PAYLOAD
  #define FRONTIER_USER_PAYLOAD ::std::uint64_t
#endif
#ifndef FRONTIER_INVALID_PAYLOAD
  #define FRONTIER_INVALID_PAYLOAD (~::std::uint64_t{0})
#endif

using UserPayload = FRONTIER_USER_PAYLOAD;
inline constexpr UserPayload kInvalidPayload = FRONTIER_INVALID_PAYLOAD;
static_assert(std::is_trivially_copyable_v<UserPayload>,
              "UserPayload must be trivially copyable");
static_assert(std::is_default_constructible_v<UserPayload>,
              "UserPayload must be default constructible");
static_assert(requires(UserPayload a, UserPayload b) {
                  { a == b } -> std::convertible_to<bool>;
              }, "UserPayload must be equality comparable");
static_assert(sizeof(UserPayload) == 8,
              "UserPayload must be exactly eight bytes");
static_assert(alignof(UserPayload) <= 8,
              "UserPayload alignment must not exceed eight bytes");
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
    bool isEmpty() const noexcept
    {
        return mn.x > mx.x || mn.y > mx.y || mn.z > mx.z;
    }
};
static_assert(sizeof(ScalarAABB) == 24,
              "ScalarAABB must contain exactly six floats");

// One renderable LOD choice. The same descriptor is used for a TLAS-owned root
// and for nodes authored into serialized subtree bytes. A mountable authored
// node must remain a leaf; its child definition and placement are supplied
// explicitly to SpatialDatabase::mountSubtree().
struct NodeDesc
{
    enum Flag : uint32_t
    {
        FlagMountable = 1u << 0,
    };
    // The remaining 31 bits are reserved for future node properties.

    UserPayload payload{};
    float geometricError = 0.0f;
    uint32_t flags = 0;
    ScalarAABB bounds = AABB::empty();

    bool isMountable() const noexcept
    {
        return (flags & FlagMountable) != 0;
    }
};
static_assert(sizeof(NodeDesc) == 40,
              "NodeDesc must stay at five eight-byte words");

} // namespace frontier
