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
        static constexpr uint32_t kChildCountMask = (1u << 9) - 1;
        static constexpr uint32_t kMountableBit = 1u << 31;

        AABB bounds = AABB::empty();
        UserPayload payload = 0;
        float geometricError = 0.0f;
        NodeId parent = kInvalidIndex;
        NodeId firstChild = kInvalidIndex;
        NodeId lastChild = kInvalidIndex;
        NodeId nextSibling = kInvalidIndex;
        uint32_t childCountAndFlags = 0;

        uint32_t childCount() const
        {
            return childCountAndFlags & kChildCountMask;
        }
        void setChildCount(uint32_t count)
        {
            childCountAndFlags =
                (childCountAndFlags & ~kChildCountMask) | count;
        }
        bool mountable() const
        {
            return (childCountAndFlags & kMountableBit) != 0;
        }
        void setMountable(bool value)
        {
            if (value) childCountAndFlags |= kMountableBit;
            else childCountAndFlags &= ~kMountableBit;
        }
    };
    static_assert(sizeof(BuildNode) == 64,
                  "BuildNode flags must fit without alignment padding");

    std::vector<BuildNode> nodes_;
    std::vector<NodeId> roots_;
    bool built_ = false;
};

} // namespace frontier
