#pragma once
// Authoring-time builder. Insertion order arbitrary, nothing perf-sensitive.
// Builds ONE page; the content pipeline decides where to split the full tree
// into pages and marks expansion points. See hlod_design.md §9.

#include <cstdint>
#include <vector>

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

    // Consumes the builder. Establishes invariants (A)-(D), emits wide child
    // blocks, verifies the contract. Throws std::logic_error on violations.
    Page build();

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
