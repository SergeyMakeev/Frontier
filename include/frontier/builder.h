#pragma once
// Authoring-time builders. SubtreeBuilder assembles reusable mount-only
// components; HierarchyBuilder authors one monolithic logical tree with
// splitBelow() boundaries. PageBuilder is the
// low-level physical-page escape hatch. See docs/frontier_design.md §3.

#include <cstdint>
#include <vector>

#include "config.h"
#include "page.h"
#include "subtree.h"

namespace frontier {

class PageBuilder
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
    void markExpansion(NodeId node,
                       HierarchyPageId detailPage = kInvalidHierarchyPage);

    // Consumes the builder. Establishes invariants (A)-(E), emits wide child
    // blocks, verifies the contract, and packs everything into one blob
    // allocated through `ctx`. Fires FRONTIER_FATAL on contract violations.
    //
    // The returned Page owns that blob; `ctx` must outlive it. Write
    // page.data()/page.byteSize() straight to disk to cache the result —
    // that byte range is the on-disk format.
    Page build(const FrontierContext& ctx = defaultContext());

private:
    struct BuildNode
    {
        AABB     bbox;
        float    geometricError = 0.0f;
        uint32_t    parent = kInvalidIndex;   // kInvalidIndex == page root
        UserPayload payload = 0;
        bool        expansion = false;
        HierarchyPageId detailPage = kInvalidHierarchyPage;
        std::vector<uint32_t> children;
    };

    std::vector<BuildNode> nodes_;
    std::vector<uint32_t>  roots_;
    bool                   built_ = false;
};

// A complete authored hierarchy split into logical single-root pages.
//
// Page zero contains the hierarchy root. Every other page contains the
// refinements below exactly one node marked with HierarchyBuilder::splitBelow.
// The packed blob may begin with several children of that logical root; that
// continuation detail is hidden from the authoring model.
class Hierarchy
{
public:
    using PageId = HierarchyPageId;
    static constexpr PageId kInvalidPage = kInvalidHierarchyPage;

    Hierarchy() = default;
    Hierarchy(Hierarchy&&) noexcept = default;
    Hierarchy& operator=(Hierarchy&&) noexcept = default;
    Hierarchy(const Hierarchy&) = delete;
    Hierarchy& operator=(const Hierarchy&) = delete;

    size_t pageCount() const { return pages_.size(); }
    PageId rootPage() const { return pages_.empty() ? kInvalidPage : 0; }

    PageView page(PageId id) const;
    Page     clonePage(PageId id,
                       const FrontierContext& ctx = defaultContext()) const;

    // Transfers one generated blob to the caller. page(id) and clonePage(id)
    // are invalid for that id afterward.
    Page takePage(PageId id);

private:
    friend class HierarchyBuilder;

    std::vector<Page> pages_;
};

// High-level authoring API. Build one ordinary logical tree, mark natural
// entity boundaries with splitBelow(), and let build() generate the root page,
// detail pages and expansion points.
class HierarchyBuilder
{
public:
    // Authoring-only id. build() consumes it; runtime code uses NodeHandle.
    using NodeId = uint32_t;

    NodeId createRoot(UserPayload payload, float geometricError,
                      const AABB& bbox = AABB::empty());
    NodeId createNode(NodeId parent, UserPayload payload, float geometricError,
                      const AABB& bbox = AABB::empty());

    // Keep `node` as the renderable fallback in its current page and place all
    // of its descendants in a logical detail page rooted at `node`. Children
    // may be authored before or after this call.
    void splitBelow(NodeId node);

    // Consumes the builder and generates deterministically indexed pages.
    Hierarchy build(const FrontierContext& ctx = defaultContext());

private:
    struct BuildNode
    {
        AABB        bbox;
        float       geometricError = 0.0f;
        NodeId      parent = kInvalidIndex;
        UserPayload payload = 0;
        bool        splitBelow = false;
        std::vector<NodeId> children;
    };

    std::vector<BuildNode> nodes_;
    NodeId                 root_ = kInvalidIndex;
    bool                   built_ = false;
};

// Assembly-first authoring API. root() names the fragment's mount sentinel,
// not a hierarchy root. Real descendant nodes may be added directly below it;
// at runtime they acquire the renderable TLAS/expansion node passed to mount()
// as their parent. An expansion leaf can permanently reference another key.
class SubtreeBuilder
{
public:
    using NodeId = uint32_t;

    explicit SubtreeBuilder(SubtreeKey key) : key_(key) {}

    static constexpr NodeId root() { return kInvalidIndex; }

    NodeId createNode(NodeId parent, UserPayload payload, float geometricError,
                      const AABB& bbox = AABB::empty());

    // Turns a real leaf into a composition site. The child Subtree is not
    // copied or required to be present while this parent is authored.
    void setExpansion(NodeId node, SubtreeKey target,
                      const SubtreeTransform& transform = {});

    // Consumes the builder and emits one packed mount-sentinel page plus the
    // deduplicated external dependency/placement sidecar.
    Subtree build(const FrontierContext& ctx = defaultContext());

private:
    struct BuildNode
    {
        AABB bbox;
        float geometricError = 0.0f;
        NodeId parent = kInvalidIndex;
        UserPayload payload = 0;
        bool expansion = false;
        SubtreeKey target{};
        SubtreeTransform transform{};
        std::vector<NodeId> children;
    };

    SubtreeKey key_{};
    std::vector<BuildNode> nodes_;
    std::vector<NodeId> roots_;
    bool built_ = false;
};

} // namespace frontier
