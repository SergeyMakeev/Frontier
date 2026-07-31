#pragma once
// Runtime side of HLodTree (hlod_design.md §1, §3-§8):
//  - instances of immutable page trees over a wide dynamic top-level BVH
//  - topology streaming (attach/detach pages under expansion points)
//  - payload residency with O(1) all-or-nothing refinement tests
//  - single-pass pruned cut selection with epoch-stamped per-view scratch
//  - LRU garbage collection of cold pages
//  - sublinear conservative bounds refit for moving nodes and instances
//
// THE API IS FULLY HANDLE-BASED — the World keeps no id index at all (no
// hash maps anywhere). Handles are the only currency:
//  - addInstance / attachPage return a PageHandle {slot, generation};
//    node handles are composed from it plus the node's page-local index,
//    which is immutable authored data (nodeAt below).
//  - selectCut outputs carry a NodeHandle wherever the caller might act on
//    the entry (load requests, expansion requests).
//  - the 64-bit UserPayload is opaque: echoed in outputs, never interpreted.
// A handle dies with its page (detachPage / collect). Stale handles are
// detected by the generation stamp: mutating calls ignore them, queries
// report them absent — the normal race between streaming completion and GC.
//
// Single-threaded by design in this version; one World per thread, one
// ViewScratch per view.

#include <cstdint>
#include <vector>

#include "math.h"
#include "page.h"

namespace hlod {

// Attached page: returned by addInstance/attachPage. Compose node handles
// with nodeAt(); node indices are page-local, fixed at build time.
struct PageHandle
{
    uint32_t slot = kInvalidIndex;
    uint32_t generation = 0;
    bool valid() const { return slot != kInvalidIndex; }
};

// Resolved node reference: page slot + node index + generation stamp.
struct NodeHandle
{
    uint32_t slot = kInvalidIndex;
    uint32_t index = 0;
    uint32_t generation = 0;
    bool valid() const { return slot != kInvalidIndex; }
};

inline NodeHandle nodeAt(PageHandle page, uint32_t index)
{
    return NodeHandle{page.slot, index, page.generation};
}

struct CutEntry
{
    UserPayload payload;
    float       err;     // projected screen error in pixels
};

enum class IdealTag : uint8_t
{
    Direct,          // render as-is; the error test is satisfied
    NeedsExpansion,  // collapsed and too coarse; this tag IS the expansion request
};

struct IdealEntry
{
    UserPayload payload;
    NodeHandle  node;    // pass to attachPage() for NeedsExpansion entries
    float       err;
    IdealTag    tag;
};

struct LoadRequest
{
    UserPayload payload;   // load this node's payload (content is keyed by it)
    NodeHandle  node;      // pass to markResident() when the load completes
    float       priority;  // parent's screen error: how badly it is needed
};

struct CutParams
{
    float threshold  = 4.0f;   // refine when screen error exceeds this (px)
    float hysteresis = 0.0f;   // (1 +- h) split of the threshold
    float minPix     = 0.0f;   // contribution culling at the top level; 0 = off
};

using InstanceId = uint32_t;

// Per-view working set, used only for hysteresis history. Epoch-stamped: no
// per-frame clears, memory is one word per materialized interior node,
// reclaimed as pages detach. All other traversal state (screen error,
// undecided planes, refine-chain liveness) is carried on the walk's explicit
// stack instead of being scattered through memory.
class ViewScratch
{
public:
    void reset()
    {
        pages_.clear();
        frame_ = 0;
    }

private:
    friend class World;
    struct PageScratch
    {
        uint32_t generation = 0;
        // (frame of last visit << 1) | last ideal decision. The frame stamp
        // validates freshness, so nothing is ever cleared per frame.
        std::vector<uint32_t> seenSticky;
    };
    std::vector<PageScratch> pages_;   // by page slot
    uint32_t                 frame_ = 0;
};

class World
{
public:
    // ---- world assembly ----------------------------------------------------
    // The instance's root page is pinned: attached forever, roots' payloads
    // implicitly resident, never garbage-collected.
    //
    // InstanceRef carries a generation stamp, like NodeHandle: instance slots
    // are recycled, and a ref that outlives its instance (remove + later add
    // reusing the slot) must not act on the newcomer. Stale refs make
    // moveInstance / removeInstance safe no-ops.
    struct InstanceRef
    {
        InstanceId id = kInvalidIndex;
        uint32_t   generation = 0;
        PageHandle rootPage;
        bool valid() const { return id != kInvalidIndex; }
    };
    InstanceRef addInstance(Page rootPage, float4 pos, float scale = 1.0f);
    void        removeInstance(InstanceRef ref);                          // no-op if stale
    void        moveInstance(InstanceRef ref, float4 pos, float scale = 1.0f);   // no-op if stale

    // ---- topology streaming -------------------------------------------------
    // The expansion handle comes from an IdealEntry{NeedsExpansion} (or is
    // composed via nodeAt). attachPage returns an invalid PageHandle if the
    // expansion handle went stale while the page was being built (its parent
    // page was detached/collected) — drop the page in that case. It throws
    // only on true contract violations (not an expansion point, already
    // attached, empty page).
    PageHandle attachPage(NodeHandle expansionNode, Page page);
    void       detachPage(NodeHandle expansionNode);   // no-op if stale
    bool       isAttached(NodeHandle expansionNode) const;

    // ---- payload residency --------------------------------------------------
    // Handles come from LoadRequest::node. Stale handles are ignored (the
    // page was collected while the payload was loading); isResident reports
    // false for them.
    void markResident(NodeHandle h);
    void markNonResident(NodeHandle h);
    bool isResident(NodeHandle h) const;

    // ---- motion --------------------------------------------------------------
    // Record new local-space bounds for a node: ~16 ns, a bounds check and a
    // queue push — call it as often as you like, whenever you like. The
    // refit runs LAZILY: the tree is guaranteed up to date only where that
    // matters — at the next selectCut (which flushes internally) or an
    // explicit flushBounds(). A node moved many times between cuts ends at
    // exactly the last submitted box; the repeat applications hit hot cache
    // lines and early-out at the already-grown parent, so they cost a
    // fraction of a first move (~30 ns measured at 1M submissions/frame).
    // Stale handles are dropped at flush time via the generation stamp.
    void setNodeBounds(NodeHandle h, const AABB& localBounds);

    // Force pending bounds refits now (conservative grow-only propagation up
    // the tree, across page boundaries, and into the top-level BVH). Only
    // needed by callers that read bounds between cuts — tools, tests.
    void flushBounds();

    // ---- per frame ------------------------------------------------------------
    void beginFrame();   // advances the world clock (GC aging, LRU touch)

    // outIdealCut / outRequests are optional: pass nullptr to skip them and
    // their emission cost entirely. A static fully-resident scene (no
    // streaming) then pays for exactly one output: the actual cut.
    void selectCut(const CullView& view, const CutParams& params, ViewScratch& scratch,
                   std::vector<CutEntry>&    outCut,
                   std::vector<IdealEntry>*  outIdealCut = nullptr,
                   std::vector<LoadRequest>* outRequests = nullptr);

    void selectCut(const CullView& view, const CutParams& params, ViewScratch& scratch,
                   std::vector<CutEntry>&    outCut,
                   std::vector<IdealEntry>&  outIdealCut,
                   std::vector<LoadRequest>& outRequests)
    {
        selectCut(view, params, scratch, outCut, &outIdealCut, &outRequests);
    }

    // ---- garbage collection ----------------------------------------------------
    // Detaches cold pages from the LRU tail until streamedPageCount() <=
    // maxAttachedPages (pinned root pages are not counted: they can never be
    // collected). Only pages untouched for >= minAge frames, with no attached
    // child pages, and not pinned are eligible. Returns the number of pages
    // detached; freedPayloads (if given) receives the payloads whose content
    // became unreachable (they were resident).
    size_t collect(size_t maxAttachedPages, uint32_t minAge,
                   std::vector<UserPayload>* freedPayloads = nullptr);

    // ---- introspection -----------------------------------------------------------
    size_t   attachedPageCount() const { return attachedPages_; }
    size_t   streamedPageCount() const { return attachedPages_ - pinnedPages_; }
    uint32_t frame() const { return frame_; }

    struct TestAccess;   // defined by tests; full access to internals

private:
    friend struct TestAccess;

    struct NodeRef
    {
        uint32_t slot  = kInvalidIndex;
        uint32_t index = kInvalidIndex;
        bool valid() const { return slot != kInvalidIndex; }
    };

    struct PageRt
    {
        Page                  page;
        std::vector<uint8_t>  resident;       // payload loaded
        std::vector<uint32_t> readyChildren;  // resident children per node
        uint32_t   generation = 0;            // bumped per attach; invalidates handles/scratch
        uint32_t   lastTouched = 0;           // world frame of last walk touch
        uint32_t   lruPrev = kInvalidIndex, lruNext = kInvalidIndex;
        uint32_t   attachedChildPages = 0;
        bool       pinned = false;
        bool       inUse  = false;
        InstanceId instance = kInvalidIndex;
        NodeRef    owner;                     // expansion node above; invalid for root pages
        // Attached child slot per expansion node, kInvalidIndex when
        // collapsed. Allocated on the page's first child attach; this IS the
        // expansion link — there is no by-id index.
        std::vector<uint32_t> expSlot;

    };

    struct Instance
    {
        float4   pos{};
        float    scale = 1.0f;
        uint32_t rootSlot = kInvalidIndex;
        AABB     worldBox;
        float    maxErrWorld = 0.0f;
        bool     alive = false;
        uint32_t generation = 0;   // stamps InstanceRefs; bumped per reuse
        uint32_t tlasNode = kInvalidIndex;
        uint32_t tlasLane = 0;
    };

    // nullptr when the ref is stale (slot recycled) or invalid.
    Instance* resolveInstance(InstanceRef ref);

    // Wide top-level BVH node; lanes are children (inner nodes or instances).
    struct TlasNode
    {
        WideBounds bounds;
        float8     maxErr{};
        int32_t    child[kWide];   // >= 0: node index; < 0: instance ~child
        uint32_t   validMask = 0;
        int32_t    parent = -1;
    };

    struct WorkItem
    {
        uint32_t slot;
        uint8_t  alive;
        uint8_t  mask;
    };

    // Node visit carried on the walk's explicit DFS stack; err and planes are
    // computed by the parent's wide test, alive by the parent's visit.
    struct NodeItem
    {
        uint32_t node;
        float    err;
        uint8_t  planes;
        uint8_t  alive;
    };

    // ---- helpers ----
    // Live PageRt for a handle, or nullptr if the handle is stale/foreign.
    const PageRt* resolve(NodeHandle h) const;
    PageRt*       resolve(NodeHandle h)
    {
        return const_cast<PageRt*>(static_cast<const World*>(this)->resolve(h));
    }

    uint32_t allocSlot();
    uint32_t registerPage(Page&& page, InstanceId instance, NodeRef owner, bool pinned);
    void     detachSlot(uint32_t slot, std::vector<UserPayload>* freedPayloads);

    void lruUnlink(uint32_t slot);
    void lruPushFront(uint32_t slot);
    void lruTouch(uint32_t slot);

    void applyBoundsChange(uint32_t slot, uint32_t index, const AABB& box);
    void patchParentLane(PageRt& rt, uint32_t index);
    void refreshInstanceBounds(InstanceId id);

    void markTlasStructuralChange();
    void tlasRebuild();
    int32_t tlasBuildRange(std::vector<uint32_t>& items, int lo, int hi, int32_t parent);
    void tlasQuery(const CullView& view, float minPix,
                   std::vector<std::pair<uint32_t, uint8_t>>& outVisible);
    void tlasOnInstanceMoved(InstanceId id);

    ViewScratch::PageScratch& ensureScratch(ViewScratch& scratch, uint32_t slot,
                                            const PageRt& rt) const;
    void runPage(const WorkItem& item, const CullView& local, const CutParams& params,
                 ViewScratch& scratch,
                 std::vector<CutEntry>& outCut, std::vector<IdealEntry>* outIdeal,
                 std::vector<LoadRequest>* outRequests);
    void wideVisit(const Page& pg, uint32_t slot, uint32_t gen, uint32_t node,
                   uint8_t mask, uint8_t aliveKids, const CullView& local,
                   std::vector<CutEntry>& outCut, std::vector<IdealEntry>* outIdeal);
    void pushLoadRequests(const PageRt& rt, uint32_t slot, uint32_t node, float priority,
                          std::vector<LoadRequest>& out) const;

    // ---- state ----
    std::vector<PageRt>   slots_;
    std::vector<uint32_t> freeSlots_;
    size_t                attachedPages_ = 0;
    size_t                pinnedPages_ = 0;
    uint32_t              generationCounter_ = 0;

    std::vector<Instance> instances_;
    std::vector<uint32_t> freeInstances_;

    std::vector<TlasNode> tlasNodes_;
    int32_t               tlasRoot_ = -1;
    bool                  tlasDirty_ = true;
    bool                  tlasQualityBuild_ = true;   // median split vs Morton
    uint32_t              tlasEscapes_ = 0;
    uint32_t              tlasLeafCount_ = 0;
    uint32_t              tlasQualityCount_ = 0;   // leaves at last quality build

    uint32_t lruHead_ = kInvalidIndex, lruTail_ = kInvalidIndex;
    uint32_t frame_ = 0;

    // Submitted bounds in submission order. The generation stamp makes
    // entries self-invalidating: if the page detached (or the slot was
    // reused) before the flush, the entry is skipped instead of applying to
    // the wrong page.
    struct PendingMove
    {
        uint32_t slot, index, generation;
        AABB     box;
    };
    std::vector<PendingMove> pendingMoves_;

    // per-call scratch (kept to avoid reallocation)
    struct TlasItem { int32_t node; uint8_t mask; };
    std::vector<WorkItem>                      work_;
    std::vector<NodeItem>                      nodeStack_;
    std::vector<std::pair<uint32_t, uint8_t>>  visibleTmp_;
    std::vector<TlasItem>                      tlasStack_;
    std::vector<std::pair<uint64_t, uint32_t>> tlasKeys_;
    std::vector<int32_t>                       tlasLevelTmp_;
};

} // namespace hlod
