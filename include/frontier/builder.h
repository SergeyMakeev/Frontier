#pragma once

#include <cstdint>
#include <vector>

#include "node.h"
#include "subtree.h"

namespace frontier {

// Authors one reusable hierarchy fragment. A node created without a parent is
// a direct child of the renderable node on which the resulting serialized
// bytes are mounted; no sentinel or non-renderable root is exposed.
class SubtreeBuilder
{
public:
    using NodeId = uint32_t;

    SubtreeBuilder() = default;

    // Optional authoring hint; useful for large generated definitions.
    void reserve(uint32_t nodeCount);

    NodeId createNode(const NodeDesc& node);
    NodeId createNode(NodeId parent, const NodeDesc& node);

    // Consumes the builder, establishes hierarchy invariants, and emits the
    // complete traversal-ready serialized byte array.
    SubtreeBytes build(const FrontierContext& context = defaultContext());

private:
    struct BuildNode
    {
        AABB bounds = AABB::empty();
        UserPayload payload = 0;
        float geometricError = 0.0f;
        NodeId parent = kInvalidIndex;
        NodeId firstChild = kInvalidIndex;
        NodeId lastChild = kInvalidIndex;
        NodeId nextSibling = kInvalidIndex;
        uint32_t childCount = 0;
        bool mountable = false;
    };

    std::vector<BuildNode> nodes_;
    std::vector<NodeId> roots_;
    bool built_ = false;
};

} // namespace frontier
