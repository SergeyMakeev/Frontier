#pragma once

#include <bit>
#include <concepts>
#include <cstddef>
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
// The payload must occupy one native 32- or 64-bit word. Public descriptors
// retain the application type; internal and serialized storage use the
// corresponding unsigned word through the bit-preserving codec below.
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
static_assert(std::has_unique_object_representations_v<UserPayload>,
              "UserPayload must have one object representation per value");
static_assert(std::is_default_constructible_v<UserPayload>,
              "UserPayload must be default constructible");
static_assert(requires(UserPayload a, UserPayload b) {
                  { a == b } -> std::convertible_to<bool>;
              }, "UserPayload must be equality comparable");
static_assert(sizeof(UserPayload) == 4 || sizeof(UserPayload) == 8,
              "UserPayload must be exactly four or eight bytes");

namespace detail {

template <size_t Bytes>
struct PayloadWordFor;

template <>
struct PayloadWordFor<4>
{
    using Type = uint32_t;
};

template <>
struct PayloadWordFor<8>
{
    using Type = uint64_t;
};

template <class Payload>
struct PayloadCodec
{
    static_assert(std::is_trivially_copyable_v<Payload>);
    static_assert(sizeof(Payload) == 4 || sizeof(Payload) == 8);

    using Word = typename PayloadWordFor<sizeof(Payload)>::Type;

    static constexpr Word encode(Payload payload) noexcept
    {
        return std::bit_cast<Word>(payload);
    }

    static constexpr Payload decode(Word word) noexcept
    {
        return std::bit_cast<Payload>(word);
    }
};

using PayloadWord = typename PayloadCodec<UserPayload>::Word;

inline constexpr PayloadWord encodePayload(UserPayload payload) noexcept
{
    return PayloadCodec<UserPayload>::encode(payload);
}

inline constexpr UserPayload decodePayload(PayloadWord word) noexcept
{
    return PayloadCodec<UserPayload>::decode(word);
}

inline PayloadWord invalidPayloadWord() noexcept
{
    // std::bit_cast of a pointer is not a constant expression in C++20, but
    // this inline conversion still folds to the configured bit pattern.
    return encodePayload(kInvalidPayload);
}

} // namespace detail

inline constexpr uint32_t kInvalidIndex = 0xFFFFFFFFu;

// Translation plus positive uniform scale for mounted subtrees.
struct Transform
{
    float4 pos = float4::point(0.0f, 0.0f, 0.0f);
    float scale = 1.0f;
};
static_assert(sizeof(Transform) == 32, "transform layout changed");

// Top-level placement transform. Yaw is stored as a unit cosine/sine pair so
// animation systems can submit their already-computed forward vector without
// paying for atan2 followed by sin/cos in the database. Planar yaw covers the
// dominant vehicle/crowd case while keeping this descriptor at 32 bytes.
struct InstanceTransform
{
    float4 pos = float4::point(0.0f, 0.0f, 0.0f);
    float scale = 1.0f;
    YawRotation yaw{};
};
static_assert(sizeof(InstanceTransform) == 32,
              "instance transform layout changed");

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
        // Top-level-only promise: bounds already contain the node's content
        // at every planar yaw around the local origin. This lets a rotating
        // actor keep one translation-only TLAS envelope. SubtreeBuilder
        // rejects this flag because mounted placements do not carry yaw.
        FlagYawInvariantBounds = 1u << 1,
    };
    // The remaining 30 bits are reserved for future node properties.

    UserPayload payload{};
    float geometricError = 0.0f;
    uint32_t flags = 0;
    ScalarAABB bounds = AABB::empty();

    bool isMountable() const noexcept
    {
        return (flags & FlagMountable) != 0;
    }
    bool hasYawInvariantBounds() const noexcept
    {
        return (flags & FlagYawInvariantBounds) != 0;
    }
};
static_assert(sizeof(NodeDesc) == sizeof(UserPayload) + 32,
              "NodeDesc layout changed");

} // namespace frontier
