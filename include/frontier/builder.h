#pragma once

#include <cstdint>
#include <vector>

#include "subtree.h"

namespace frontier {

// Authors one reusable hierarchy fragment. A node created without a parent is
// a direct child of the renderable node on which the resulting Subtree is
// mounted; no sentinel or non-renderable root is exposed to the caller.
class SubtreeBuilder
{
public:
    using NodeId = uint32_t;

    explicit SubtreeBuilder(SubtreeKey key) : key_(key) {}

    // Optional authoring hint; useful for large generated definitions.
    void reserve(uint32_t nodeCount, uint32_t expansionCount = 0);

    NodeId createNode(const NodeDesc& node);
    NodeId createNode(NodeId parent, const NodeDesc& node);

    // Consumes the builder, establishes hierarchy invariants, and emits one
    // complete serialized Subtree allocated through context.
    Subtree build(const FrontierContext& context = defaultContext());

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
        uint32_t expansion = kInvalidIndex;
    };

    struct BuildExpansion
    {
        SubtreeKey target{};
        Transform transform{};
    };

    SubtreeKey key_{};
    std::vector<BuildNode> nodes_;
    std::vector<BuildExpansion> expansions_;
    std::vector<NodeId> roots_;
    bool built_ = false;
};

} // namespace frontier
