#pragma once
// Authoring-time builders. HierarchyBuilder is the normal API: author one
// logical tree and mark natural splitBelow() boundaries. HLodBuilder is the
// low-level physical-page escape hatch. See docs/hlod_design.md §3.

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
    void markExpansion(NodeId node,
                       HierarchyPageId detailPage = kInvalidHierarchyPage);

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
                       const HlodContext& ctx = defaultContext()) const;

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
    Hierarchy build(const HlodContext& ctx = defaultContext());

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

} // namespace hlod
