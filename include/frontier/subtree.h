#pragma once
// Logical reusable hierarchy fragments. A Subtree's packed Page contains the
// real nodes below an implicit anchor. At runtime that anchor is either a TLAS
// instance or an expansion node in another Subtree.

#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include "page.h"

namespace frontier {

class SpatialDatabase;
class SubtreeBuilder;

// Stable application/package identity. Runtime handles are deliberately
// separate: this key survives serialization and database registration.
struct SubtreeKey
{
    uint64_t value = 0;

    bool valid() const { return value != 0; }
    friend bool operator==(SubtreeKey, SubtreeKey) = default;
};
static_assert(sizeof(SubtreeKey) == 8, "subtree key must stay 64 bits");

// Placement of a mounted child in its owner's coordinate system. Frontier's
// existing transform contract is translation plus positive uniform scale.
struct SubtreeTransform
{
    float4 pos = float4::point(0.0f, 0.0f, 0.0f);
    float  scale = 1.0f;
};
static_assert(sizeof(SubtreeTransform) == 32,
              "subtree transform layout changed");

// One expansion site in the packed root page. `dependency` indexes the
// Subtree's deduplicated dependency table, so a million identical placements
// store the full SubtreeKey only once.
struct SubtreeExpansion
{
    float4   pos = float4::point(0.0f, 0.0f, 0.0f);
    float    scale = 1.0f;
    uint32_t nodeIndex = kInvalidIndex;
    uint32_t dependency = kInvalidIndex;

    SubtreeTransform transform() const { return {pos, scale}; }
};
static_assert(sizeof(SubtreeExpansion) == 32,
              "one authored expansion site must stay 32 bytes");

// Move-only authored component. The first implementation intentionally keeps
// the physical representation to one packed page; composition supplies
// arbitrary depth, while Hierarchy/splitBelow remains available for legacy
// intra-tree paging.
class Subtree
{
public:
    Subtree() = default;
    Subtree(Subtree&&) noexcept = default;
    Subtree& operator=(Subtree&&) noexcept = default;
    Subtree(const Subtree&) = delete;
    Subtree& operator=(const Subtree&) = delete;

    bool valid() const { return key_.valid() && page_.valid(); }
    SubtreeKey key() const { return key_; }
    PageView page() const { return static_cast<const PageView&>(page_); }

    std::span<const SubtreeKey> dependencies() const { return dependencies_; }
    std::span<const SubtreeExpansion> expansions() const { return expansions_; }

private:
    friend class SpatialDatabase;
    friend class SubtreeBuilder;

    SubtreeKey key_{};
    Page page_;
    std::vector<SubtreeKey> dependencies_;
    std::vector<SubtreeExpansion> expansions_; // sorted by packed node index
};

} // namespace frontier
