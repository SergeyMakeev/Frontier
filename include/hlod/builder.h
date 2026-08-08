#pragma once
// Authoring-time builder. Insertion order arbitrary, nothing perf-sensitive.
// Builds ONE page; the content pipeline decides where to split the full tree
// into pages and marks expansion points. See docs/hlod_design.md §3.

#include <cstdint>
#include <vector>

#include "config.h"
#include "page.h"

namespace hlod {

class HLodBuilder
{
public:
    using NodeId = uint32_t;

    // A page is a forest fragment: root pages typically have one root; a page
    // attached under an expansion point holds that node's children as roots.
    NodeId createRoot(UserPayload payload, float geometricError,
                      const AABB& bbox = AABB::empty());
    NodeId createNode(NodeId parent, UserPayload payload, float geometricError,
                      const AABB& bbox = AABB::empty());

    // Marks a node whose children live in another page. Must remain a leaf.
    void markExpansion(NodeId node);

    // Consumes the builder. Establishes invariants (A)-(E), emits wide child
    // blocks, verifies the contract, and packs everything into one blob
    // allocated through `ctx`. Fires HLOD_FATAL on contract violations.
    //
    // The returned Page owns that blob; `ctx` must outlive it. Write
    // page.data()/page.byteSize() straight to disk to cache the result —
    // that byte range is the on-disk format.
    Page build(const HlodContext& ctx = defaultContext());

private:
    struct BuildNode
    {
        AABB     bbox;
        float    geometricError = 0.0f;
        uint32_t    parent = kInvalidIndex;   // kInvalidIndex == page root
        UserPayload payload = 0;
        bool        expansion = false;
        std::vector<uint32_t> children;
    };

    std::vector<BuildNode> nodes_;
    std::vector<uint32_t>  roots_;
    bool                   built_ = false;
};

} // namespace hlod
