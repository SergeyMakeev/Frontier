#pragma once
// Runtime side of HLodTree (see hlod_design.md):
//  - assets: immutable page trees, registered once, shared by every instance
//    and every attachment that names them
//  - instances of those assets over a wide dynamic top-level BVH
//  - topology streaming (attach/detach pages under expansion points)
//  - payload residency with incrementally propagated subtree coverage
//  - single-pass pruned cut selection, with no required per-node view scratch
//  - LRU garbage collection of cold pages
//  - sublinear conservative bounds refit for moving nodes and instances
//
// EVERYTHING IS SHARED
// --------------------
// A page is registered once and instanced many times. Every instance of an
// asset walks the same bytes AND the same runtime state: one residency array,
// one attachment graph, one residency state. Ten thousand copies of
// a tree cost one tree, and a streamer that attaches a page under a shared
// expansion point attaches it for all ten thousand at once.
//
// Deformation does not break that. Bounds are the only part of a page the
// runtime rewrites, so an instance that is deformed with setNodeBounds gets a
// private copy of just its bounds (a copy-on-write overlay, roughly 60% of a
// page) and keeps sharing topology, payloads, errors, residency and streaming
// with every other instance. There is no "private page" mode: the thing that
// would actually hurt at scale is ten thousand duplicated *streaming graphs*,
// not ten thousand duplicated boxes, and duplicating the page duplicates both.
//
// THE API IS FULLY HANDLE-BASED — the World keeps no id index at all (no
// hash maps anywhere). Handles are the only currency:
//  - registerAsset returns an AssetHandle; addInstance returns an InstanceRef
//    containing its root PageHandle, and attachPage returns a PageHandle.
//    Node handles are composed from a page handle plus the packed page-local
//    index, which is immutable after build.
//  - selectCut outputs carry a NodeHandle wherever the caller might act on
//    the entry (payload residency or topology expansion).
//  - the 64-bit UserPayload is opaque: echoed in outputs, never interpreted.
// A handle dies with its page (detachPage / collect). Stale handles are
// detected by the generation stamp: mutating calls ignore them, queries
// report them absent — the normal race between streaming completion and GC.
//
// World updates are single-writer. applyUpdates() publishes a stable snapshot;
// contextual selectCut overloads then read only that snapshot and may run
// concurrently when each call has its own contexts and outputs. LOD damping
// lives in the CullView (ViewDamper) or in an optional per-view
// SelectionContext, never as per-node World state.

#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <memory>
#include <vector>

#include "config.h"
#include "math.h"
#include "page.h"

namespace hlod {

class World;
struct SelectionScratch;

// Registered page asset: the unit of sharing. Returned by registerAsset().
struct AssetHandle
{
    uint32_t slot = kInvalidIndex;
    uint32_t generation = 0;
    bool valid() const { return slot != kInvalidIndex; }
};

// Attached page (a "mount"): one placement of an asset in the world. Returned
// by addInstance/attachPage. Compose node handles with nodeAt(); node indices
// are page-local, fixed at build time.
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

// Every entry belongs to the current render cut, the fully-resident ideal cut,
// or both. Shared entries are emitted once, so filtering CurrentOnly|Shared
// yields the current cut and IdealOnly|Shared yields the ideal cut.
enum class CutMembership : uint8_t
{
    CurrentOnly,
    IdealOnly,
    Shared,
};

enum class CutTag : uint8_t
{
    Direct,          // concrete node in known topology
    NeedsExpansion,  // collapsed and too coarse; this tag IS the expansion request
};

// `instance` is the caller's tag from InstanceDesc. A result is associated
// with both its shared node and its instance placement; payloads and caller
// tags are opaque values and may be duplicated.
struct CutEntry
{
    UserPayload payload;
    NodeHandle  node;
    float       err;
    uint32_t    instance;
    CutMembership membership;
    CutTag        tag;
};

inline bool inCurrentCut(const CutEntry& entry)
{
    return entry.membership != CutMembership::IdealOnly;
}

inline bool inIdealCut(const CutEntry& entry)
{
    return entry.membership != CutMembership::CurrentOnly;
}

struct CutParams
{
    float threshold = 4.0f;   // refine when screen error exceeds this (px)
    float minPix    = 0.0f;   // contribution culling at the top level; 0 = off
};

using InstanceId = uint32_t;

// InstanceDesc::tag sentinel: echo the World's own InstanceId in CutEntry.
inline constexpr uint32_t kAutoTag = kInvalidIndex;

struct InstanceDesc
{
    float4   pos{};
    float    scale = 1.0f;

    // Echoed in CutEntry::instance. Whatever the
    // caller wants it to mean: a transform index, an entity id, a draw-list
    // slot. kAutoTag echoes the InstanceId instead.
    uint32_t tag = kAutoTag;

    // ANDed against CullView::viewMask; a zero result culls the instance at
    // the top level. Cheap layer visibility: shadow-only props, editor-only
    // gizmos, per-view opt-outs.
    uint32_t mask = ~0u;
};

// ---------------------------------------------------------------------------
// Output sinks
//
// selectCut writes through a Sink so the same out-of-line traversal can fill
// either a std::vector (grows, never drops) or caller memory (fixed, reports
// what did not fit). Engines that write straight into a draw list or a mapped
// instance buffer want the second one.
// ---------------------------------------------------------------------------

template <class T>
class Sink
{
public:
    Sink() = default;

    // Growable: appends to `v`, which is cleared first. Never drops.
    explicit Sink(std::vector<T>& v) : vec_(&v) { v.clear(); }

    // Fixed: writes into caller memory, counting (not writing) the overflow.
    Sink(T* data, uint32_t capacity) : data_(data), capacity_(capacity) {}

    void push(const T& v)
    {
        if (vec_)
        {
            vec_->push_back(v);
            return;
        }
        if (count_ < capacity_)
            data_[count_++] = v;
        else
            ++dropped_;
    }

    // Append n contiguous values. Same result as n pushes, one bulk copy: this
    // is how a SelectionContext hands back an instance's recorded cut.
    void pushRange(const T* p, uint32_t n)
    {
        if (n == 0) return;
        if (vec_)
        {
            // Contextual cuts are usually one entry per distant instance.
            // Avoid std::vector's range-insert machinery for that dominant
            // case; capacity is already retained across selectCut calls.
            if (n == 1)
            {
                vec_->push_back(*p);
                return;
            }
            vec_->insert(vec_->end(), p, p + n);
            return;
        }
        const uint32_t fits = count_ < capacity_ ? capacity_ - count_ : 0;
        const uint32_t take = n < fits ? n : fits;
        if (take) std::memcpy(data_ + count_, p, size_t(take) * sizeof(T));
        count_ += take;
        dropped_ += n - take;
    }

    uint32_t count() const { return vec_ ? uint32_t(vec_->size()) : count_; }
    uint32_t dropped() const { return dropped_; }
    bool     overflowed() const { return dropped_ != 0; }

private:
    std::vector<T>* vec_ = nullptr;
    T*              data_ = nullptr;
    uint32_t        capacity_ = 0;
    uint32_t        count_ = 0;
    uint32_t        dropped_ = 0;
};

using CutSink     = Sink<CutEntry>;

// Traversal counters, filled only when the library is built with HLOD_STATS.
struct CutStats
{
    uint64_t instancesVisited = 0;
    uint64_t pagesVisited = 0;
    uint64_t nodesVisited = 0;
    uint64_t wideBlocksTested = 0;
    uint64_t lanesSurvived = 0;
};

// Optional per-camera page-use feedback. Passing one to selectCut records the
// pages that camera needed; omitting it makes selection pay no page-use
// recording cost. The World consumes feedback only when collect() is called,
// so callers choose which cameras influence page retention (for example, the
// primary camera but not shadow cascades).
//
// A context may accumulate observations across many update epochs. Like
// SelectionContext, it belongs to one view and must not be used concurrently
// by two calls; distinct contexts are independent.
class PageUsageContext
{
public:
    PageUsageContext() = default;
    PageUsageContext(PageUsageContext&&) noexcept = default;
    PageUsageContext& operator=(PageUsageContext&&) noexcept = default;
    PageUsageContext(const PageUsageContext&) = delete;
    PageUsageContext& operator=(const PageUsageContext&) = delete;

    void   reset();
    size_t bytes() const;

private:
    friend class World;

    struct Rec
    {
        uint32_t generation = 0;
        uint32_t lastUsed = 0;
        bool     pending = false;
    };

    const World*     world_ = nullptr;
    std::vector<Rec> rec_;
    std::vector<uint32_t> dirty_;
};

// ---------------------------------------------------------------------------
// SelectionContext — everything selection remembers about one view
//
// A world that is mostly static, seen by a camera that moves continuously,
// produces a cut that is nearly identical frame to frame. This exploits that:
// an instance whose cut provably cannot have changed is not walked at all, and
// its entries are handed back where they already lie.
//
// THE STATE IS ALL HERE, NOT IN THE WORLD. Querying several cameras per frame
// (main view, shadow cascades, a reflection probe) means several contexts, and
// they cannot interfere: the World stays a pure read during selection, exactly
// as it is without one.
//
// It also OWNS THE VIEW DAMPER for its view, and that is not merely tidy. The
// two are one mechanism read two ways: the damper turns a camera position into
// a query envelope, and the reuse test below is driven by how far that same
// envelope has travelled, not by the camera. Held separately, the caller has to
// pair the right damper with the right context by hand every frame, and pairing
// the main view's damper with a shadow cascade's cache compiles perfectly well.
// So selectCut takes the RAW view and damps it internally, one reset() covers
// the discontinuity for both halves, and the pairing cannot be got wrong.
//
// It also makes one coupling visible instead of hidden, which is the other
// reason to keep them together. The damper relaxes projection scale k as well
// as position. A flip point lies at `geometricError * k / threshold`, so the
// maximum error observed while recording a cut gives a conservative slope for
// how far any of its decisions can move per unit k. The context accumulates
// absolute k travel and charges that per-record slope against the same validity
// margin used for camera travel.
//
// This matters most for damped zoom-out: k changes a little for roughly 24
// half-lives. The old all-or-nothing epoch invalidated the whole cache on each
// of those frames. The slope budget keeps unaffected records reusable while
// preserving the exact node set. Measured by BM_SelectionContext_Zoom at 20k
// instances, halfLife 8 and a zoom step every 120 frames, it removes 39% of the
// cut time without growing the 48-byte record. Threshold changes still bump an
// epoch because they change every stored slope at once.
//
// Damping and reuse each stay optional and free when unused: halfLife 0 makes
// the arithmetic bit-identical to an undamped query, and the storage below is
// empty until the first call. A fully dynamic world wants the damper and not
// the reuse, and pays ~8% for the reuse it cannot use; a shadow cascade often
// wants the reverse.
//
// WHY IT IS SOUND, AND WHY IT WINS
// --------------------------------
// Inside one instance the only camera-dependent decision is the screen-error
// test, `geomError * k / distance > threshold`, which flips when the distance
// reaches `geomError * k / threshold`. During a walk this records the smallest
// gap between the two over every node that was tested -- the VALIDITY MARGIN,
// a distance. Moving the camera by less than that margin cannot flip any
// decision, because a translation of d changes every distance by at most d.
//
// Frustum decisions would spoil that argument, since rotating a camera moves
// the planes much further than it moves the eye. So only instances that were
// ENTIRELY INSIDE the frustum are cached: no plane was tested anywhere inside
// them, and their cut is therefore a pure function of camera position. An
// instance straddling the frustum edge is always re-walked, and there are few
// of those -- they are a shell, not a volume.
//
// The margin is large exactly where it needs to be. A distant instance sits
// far past every flip point, so its margin is roughly its own distance and it
// stays valid for many frames; a near instance with a deep cut has some node
// sitting right at the threshold, so its margin is tiny and it is re-walked
// every frame. That is the correct division of labour rather than a
// limitation: the population is dominated by distant instances, which is where
// the per-instance fixed cost (resolve the instance, transform the view, touch
// the root page) was being paid over and over for an answer that never moved.
//
// FLAT OUTPUT
// -----------
// This overload writes the same contiguous CutEntry sequence as every other
// one, so the only difference at the call site is the context argument.
// Reused entries are copied from context-owned storage into the caller's flat
// output; no World-owned mutable query state is involved.
//
// WHAT IS EXACT AND WHAT IS NOT
// -----------------------------
// The set of emitted nodes is exactly what a stateless selectCut would emit.
// CutEntry::err is not: it is the value from the recorded camera position, so
// it is stale by at most the margin expressed in error units. Use it for
// prioritisation and dithering, which is what it is for; do not expect it to
// be bit-identical to the stateless path.
//
// Streaming and residency are part of the recorded unified cut. Page content
// versions invalidate a record when any page it depended on changes. Instances
// that cross more page dependencies than the compact record can hold are
// simply re-walked.
// ---------------------------------------------------------------------------

class SelectionContext
{
public:
    SelectionContext();
    explicit SelectionContext(float halfLifeFrames);
    ~SelectionContext();

    SelectionContext(SelectionContext&&) noexcept;
    SelectionContext& operator=(SelectionContext&&) noexcept;
    SelectionContext(const SelectionContext&) = delete;
    SelectionContext& operator=(const SelectionContext&) = delete;

    // LOD hysteresis for this view, in frames; 0 disables it exactly. See
    // ViewDamper in math.h for what the envelope does.
    float halfLife() const { return damper_.halfLife(); }
    void  setHalfLife(float frames) { damper_.setHalfLife(frames); }

    // Instances served from the cache, and re-walked, in the last call.
    uint32_t reused() const { return reused_; }
    uint32_t walked() const { return walked_; }

    // Counters from this context's last call. Per-context ownership keeps
    // concurrent views from racing over a global "last selection" value.
    const CutStats& lastCutStats() const { return stats_; }

    // Forget everything: the damping window and every record. Call it on a
    // teleport or camera cut. Reuse never *requires* this -- every record
    // carries the stamps and the travel budget that invalidate it -- but
    // damping does, or the envelope stretches across the discontinuity and
    // over-refines everything between the two positions. One call so that
    // cannot be half-done.
    void reset();

    size_t bytes() const;

private:
    friend class World;

    // Pages whose state this instance's cut depended on. Two covers an
    // instance root plus one attachment; a walk that touches more is simply
    // not cached, which costs nothing, because an instance that deep is
    // usually still streaming and being re-walked anyway.
    static constexpr uint32_t kMaxDeps = 2;

    // EXACTLY 48 BYTES, and that is the whole design constraint.
    //
    // This record is read for every visible instance, indexed by instance id,
    // so it is a random access per instance -- the one new memory stream the
    // cache introduces. It has to be cheaper than the walk it replaces, and
    // the walk it usually replaces is cheap: with a shared asset the page is
    // the same bytes for every instance and sits in L1. A first version of
    // this record was 128 bytes and lost 1.4x at 80k instances while reusing
    // 93% of them, purely on its own footprint. Everything here is therefore
    // either a scalar or absent:
    //  - no camera envelope: validity is a single scalar budget against the
    //    view's accumulated travel (see travel_);
    //  - no per-record k / threshold copies: threshold changes bump epoch_,
    //    while k motion consumes the scalar slope budget below;
    //  - no page generations: contentVersion is bumped on attach too, so it
    //    alone distinguishes a recycled slot.
    struct Rec
    {
        // Reusable while positionTravel + kSlope * kTravel has not passed
        // this. Set from that same expression plus the measured margin.
        float    validUntil = 0.0f;
        float    kSlope = 0.0f;      // max world-space flip travel per unit k
        uint32_t epoch = 0;          // cache epoch (threshold generation)
        uint32_t cutVersion = 0;     // unique instance transform/deform version
        uint32_t begin = 0;          // block in store_
        uint32_t count = 0;
        uint32_t capacity = 0;
        uint32_t depSlot[kMaxDeps]{};
        uint32_t depVersion[kMaxDeps]{};
        uint32_t depCount = 0;
    };
    static_assert(sizeof(Rec) == 48, "SelectionContext::Rec must stay 48 bytes");

    void compact();

    // This view's hysteresis. selectCut damps through it, so the odometer
    // below measures the envelope it produces.
    ViewDamper damper_;

    std::vector<Rec>      rec_;      // by InstanceId
    std::vector<CutEntry> store_;    // slab of recorded runs
    uint32_t              used_ = 0;
    uint32_t              garbage_ = 0;

    // Distance the damped query envelope has travelled since this context was
    // created, accumulated per call. kTravel_ similarly accumulates absolute
    // projection-scale motion. A record taken with margin m remains valid
    // while positionTravel + kSlope * kTravel stays below its saved budget:
    // both path lengths conservatively bound movement of every LOD flip point.
    // This replaces storing a camera and projection value per instance with
    // two scalar odometers and one per-record slope, keeping Rec at 48 bytes.
    // Doubling back is conservative; a teleport consumes the budget at once.
    float    travel_ = 0.0f;
    float    kTravel_ = 0.0f;
    float4   lastQmn_{}, lastQmx_{};
    bool     primed_ = false;
    // Bumped when the error threshold changes, invalidating every record in
    // O(1). Projection-scale changes consume kTravel_ instead.
    uint32_t epoch_ = 1;
    float    k_ = 0.0f, bar_ = 0.0f;
    uint32_t reused_ = 0;
    uint32_t walked_ = 0;
    CutStats stats_{};

    // Mutable query scratch. Opaque so traversal
    // implementation details do not become part of the public API.
    std::unique_ptr<SelectionScratch> scratch_;
};

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

enum class TlasQuality : uint8_t
{
    Morton,     // one sort, contiguous groups of kWide — cheapest, loosest
    Median,     // recursive longest-axis median split
    BinnedSAH,  // binned surface-area-heuristic split; best traversal cost
};

struct WorldConfig
{
    // Allocation and task parallelism. Copied into the World; callback code
    // and anything referenced by `user` must remain valid for its lifetime.
    HlodContext context{};

    // Quality tier of the top-level BVH. Initial builds, large population
    // drift, and area-drift repair use this tier. Escape/edit-budget repairs
    // use the cheaper Morton path unless another trigger promotes the build.
    TlasQuality tlasQuality = TlasQuality::BinnedSAH;

    // SAH cost constants (BinnedSAH only): cost of visiting a node vs testing
    // an instance. Raising intersect cost builds deeper, tighter trees.
    float tlasTraversalCost = 1.0f;
    float tlasIntersectCost = 1.0f;

    // Fraction of the instance population that must appear or disappear
    // before the next rebuild is promoted to the quality tier.
    float tlasCountDrift = 0.2f;

    // Fraction of the top-level BVH's original lane area that motion may add
    // through grow-only refit before a quality rebuild is forced. Catches the
    // bloat that a steady population hides from tlasCountDrift.
    float tlasAreaDrift = 0.5f;

    // Fraction of distinct leaves that may escape their build-time lane before
    // a rebuild. Repeated growth of the same moving leaf is charged once.
    float tlasEscapeFraction = 0.25f;

    // Fraction of the population that may be spawned or removed INCREMENTALLY
    // before the tree is rebuilt. An incremental insert descends to the leaf
    // whose box grows least and either takes a free lane or splits the leaf,
    // which deepens that subtree by one; a removal invalidates a lane and
    // leaves its box loose. Both are O(depth) and both cost a little quality,
    // so this bounds how much of it accumulates.
    //
    // Do not read this as a tuning knob of last resort: before it existed,
    // every add or remove marked the whole TLAS dirty, so ONE spawn cost a full
    // rebuild -- 2.1 ms at 20k instances and 9.5 ms at 80k, the same as five
    // hundred spawns. See ARCHITECTURE.md, experiment L.
    float tlasEditFraction = 0.05f;

    // Minimum number of visible instances before selectCut fans out across
    // context.parallelFor. 0 disables parallel cut selection entirely.
    // Also requires context.workerCount > 1.
    //
    // There is no correctness caveat attached to this: selection reads the
    // World and writes only per-worker buffers, which are concatenated in
    // instance order, so a parallel cut is bit-identical to a serial one.
    uint32_t parallelInstanceThreshold = 0;
};

class World
{
public:
    explicit World(const WorldConfig& config = WorldConfig{});
    ~World();

    // Non-copyable and non-movable: pages the World owns hold a pointer to
    // config_.context, and instances/mounts refer to each other by slot.
    World(const World&) = delete;
    World& operator=(const World&) = delete;
    World(World&&) = delete;
    World& operator=(World&&) = delete;

    const WorldConfig& config() const { return config_; }

    // ---- assets ------------------------------------------------------------
    // Publish a page for sharing. The World allocates ONE runtime state for
    // it (residency, attachment links) no matter how many instances name it,
    // and its root mount stays alive until releaseAsset().
    //
    // The borrowed overload does not copy: point it at a memory-mapped bundle
    // and the page costs nothing but address space. The bytes must outlive
    // the asset.
    AssetHandle registerAsset(Page&& page);
    AssetHandle registerAsset(PageView borrowedPage);

    // Drops the World's own reference. Contract violation if instances still
    // reference the asset. Detaches its whole page tree.
    void   releaseAsset(AssetHandle asset);
    bool   isAsset(AssetHandle asset) const;
    size_t assetCount() const { return liveAssets_; }

    // The asset's shared instance-root mount, for composing node handles
    // (marking residency, attaching children) without a selectCut round-trip.
    // Invalid until addInstance has materialized that root. Mounts created by
    // attaching the asset elsewhere are independent and returned by attachPage.
    PageHandle assetRootPage(AssetHandle asset) const;

    // ---- world assembly ----------------------------------------------------
    // An instance's root page is pinned: attached forever, roots' payloads
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

    // During initial assembly, additions are accumulated for the first build.
    // Once a TLAS exists, insertion is O(depth) and may allocate a TLAS node
    // when a full leaf splits; it never scans the whole instance population.
    InstanceRef addInstance(AssetHandle asset, const InstanceDesc& desc);
    InstanceRef addInstance(AssetHandle asset, float4 pos, float scale = 1.0f);

    // Convenience for one-off content: registers the page as an anonymous
    // asset owned by the World and instances it. The asset is freed when its
    // last instance goes away. Instancing the same tree more than once should
    // go through registerAsset so the two instances share it.
    InstanceRef addInstance(Page&& page, const InstanceDesc& desc);
    InstanceRef addInstance(Page&& page, float4 pos, float scale = 1.0f);

    void removeInstance(InstanceRef ref);                                    // no-op if stale
    void moveInstance(InstanceRef ref, float4 pos, float scale = 1.0f);      // no-op if stale

    // ---- topology streaming -------------------------------------------------
    // The expansion handle comes from a CutEntry{NeedsExpansion} (or is
    // composed via nodeAt). attachPage returns an invalid PageHandle if the
    // expansion handle went stale while the page was being built (its parent
    // page was detached/collected) — drop the page in that case. It fires
    // HLOD_FATAL only on true contract violations (not an expansion point,
    // already attached, empty page).
    //
    // Attaching under an asset attaches for every instance of it at once —
    // that is the point, and it is why one streamer no longer does the same
    // work ten thousand times.
    //
    // The child page must fit inside the expansion node's authored bounds.
    // Growing the owner here would ripple into every instance of the owning
    // asset, so it is a contract violation rather than a silent refit: write
    // conservative expansion bounds at build time.
    PageHandle attachPage(NodeHandle expansionNode, AssetHandle asset);
    PageHandle attachPage(NodeHandle expansionNode, Page&& page);
    void       detachPage(NodeHandle expansionNode);   // no-op if stale
    bool       isAttached(NodeHandle expansionNode) const;

    // ---- payload residency --------------------------------------------------
    // Handles come from ideal-side CutEntry values. Stale handles are ignored
    // (the page was collected while the payload was loading); isResident
    // reports false for them. Residency changes incrementally propagate
    // complete-subtree coverage toward the root.
    void markResident(NodeHandle h);
    void markNonResident(NodeHandle h);
    bool isResident(NodeHandle h) const;

    // ---- motion --------------------------------------------------------------
    // Record new local-space bounds for one node OF ONE INSTANCE: a bounds
    // check and a queue push. Call it as often as needed; the refit runs
    // LAZILY, so the tree is guaranteed up to date after applyUpdates() (or an
    // explicit flushBounds() for tools and tests). A node moved many times
    // between update barriers ends at
    // exactly the last submitted box; the repeat applications hit hot cache
    // lines and early-out at the already-grown parent, so they cost a fraction
    // of a first move. Stale handles are dropped at flush time via the
    // generation stamps.
    //
    // The first deformation of a given (instance, page) pair copies that
    // page's bounds into a per-instance overlay; everything else about the
    // page stays shared. Subsequent moves write straight into the overlay.
    // Propagation up the ancestor chain promotes overlays as it crosses page
    // boundaries, so only the pages on the path from the moved node to the
    // instance root are ever privatised.
    void setNodeBounds(InstanceRef inst, NodeHandle h, const AABB& localBounds);

    // Force pending bounds refits now (conservative grow-only propagation up
    // the tree, across page boundaries, and into the top-level BVH). Only
    // needed by callers that read bounds between cuts — tools, tests.
    void flushBounds();

    // Bounds of one node as this instance sees them: the overlay if it has
    // one, the shared page otherwise. Flushes pending moves first.
    AABB nodeBounds(InstanceRef inst, NodeHandle h);

    // Publish queued changes and prepare a stable read-only selection
    // snapshot. Call once before a group of selections, even when no world
    // objects changed; it also advances the epoch used by collection aging.
    // No World mutation (including collect) may overlap or occur before every
    // contextual selection using this snapshot has completed.
    void applyUpdates();

    // The single output is the union of the current and ideal cuts. Shared
    // entries appear once; CutMembership identifies the two additive deltas.
    // NeedsExpansion entries expose topology that is not attached yet. The
    // caller owns residency policy: inspect ideal-side entries, query
    // isResident(), deduplicate shared node ids as desired, and schedule IO.
    //
    // For LOD damping, run the view through a ViewDamper (math.h) first.
    // Stateless overloads retain World-owned scratch and aggregate stats; do
    // not call them concurrently. Use SelectionContext overloads for
    // concurrent views.
    void selectCut(const CullView& view, const CutParams& params,
                   std::vector<CutEntry>& outCut);

    void selectCut(const CullView& view, const CutParams& params,
                   PageUsageContext& usage, std::vector<CutEntry>& outCut);

    // Sink form: write straight into caller memory. Check Sink::dropped() to
    // find out whether the buffers were big enough.
    void selectCut(const CullView& view, const CutParams& params, CutSink& outCut);

    void selectCut(const CullView& view, const CutParams& params,
                   PageUsageContext& usage, CutSink& outCut);

    // Contextual form: same output as the overloads above, but an instance whose
    // cut provably cannot have changed is not walked -- its recorded entries are
    // copied out instead. One context per view.
    //
    // Two things differ from the stateless overloads, both deliberate:
    //  - pass the RAW view. This is the only overload that damps internally,
    //    through the context's own ViewDamper.
    //  - CutEntry::err on a reused entry is stale within the proven margin. The
    //    node set is exact; see the notes on SelectionContext.
    //
    // These overloads are const and may run concurrently after applyUpdates().
    // Each in-flight call must use distinct SelectionContext, optional
    // PageUsageContext, and output objects. The same context is not internally
    // synchronized.
    void selectCut(const CullView& view, const CutParams& params,
                   SelectionContext& ctx, CutSink& outCut) const;

    void selectCut(const CullView& view, const CutParams& params,
                   SelectionContext& ctx, PageUsageContext& usage,
                   CutSink& outCut) const;

    void selectCut(const CullView& view, const CutParams& params,
                   SelectionContext& ctx,
                   std::vector<CutEntry>& outCut) const;

    void selectCut(const CullView& view, const CutParams& params,
                   SelectionContext& ctx, PageUsageContext& usage,
                   std::vector<CutEntry>& outCut) const;

    // ---- garbage collection ----------------------------------------------------
    // Detaches cold pages from the LRU tail until streamedPageCount() <=
    // maxAttachedPages (pinned root pages are not counted: they can never be
    // collected). Only pages untouched for >= minAge frames, with no attached
    // child pages, and not pinned are eligible. Returns the number of pages
    // detached; freedPayloads (if given) receives the payloads whose content
    // became unreachable (they were resident). Overloads taking page-usage
    // contexts consume their accumulated feedback before examining the tail;
    // omitted views do not influence page retention.
    size_t collect(size_t maxAttachedPages, uint32_t minAge,
                   std::vector<UserPayload>* freedPayloads = nullptr);
    size_t collect(PageUsageContext& usage, size_t maxAttachedPages,
                   uint32_t minAge,
                   std::vector<UserPayload>* freedPayloads = nullptr);
    size_t collect(std::initializer_list<PageUsageContext*> usage,
                   size_t maxAttachedPages, uint32_t minAge,
                   std::vector<UserPayload>* freedPayloads = nullptr);

    // ---- introspection -----------------------------------------------------------
    size_t   attachedPageCount() const { return attachedPages_; }
    size_t   streamedPageCount() const { return attachedPages_ - pinnedPages_; }
    uint32_t frame() const { return frame_; }   // published update epoch

    // Number of live copy-on-write bounds overlays, and the bytes they hold.
    // Both stay zero for a scene that never calls setNodeBounds.
    size_t overlayCount() const { return liveOverlays_; }
    size_t overlayBytes() const;

    // Filled by the last stateless selectCut. Contextual calls report through
    // SelectionContext::lastCutStats(). All zero unless built with HLOD_STATS.
    const CutStats& lastCutStats() const { return stats_; }

    struct TestAccess;   // defined by tests; full access to internals

private:
    friend struct TestAccess;
    friend struct SelectionScratch;

    struct NodeRef
    {
        uint32_t slot  = kInvalidIndex;
        uint32_t index = kInvalidIndex;
        bool valid() const { return slot != kInvalidIndex; }
    };

    // A registered page: the immutable bytes plus the bookkeeping that keeps
    // them alive. `owned` is empty for borrowed pages.
    struct AssetRt
    {
        PageView view;
        Page     owned;
        uint32_t rootMount = kInvalidIndex;
        uint32_t generation = 0;
        uint32_t mountRefs = 0;      // PageRt entries referencing these bytes
        uint32_t instanceRefs = 0;   // Instances rooted at rootMount
        bool     registered = false; // user holds an AssetHandle
        bool     inUse = false;
    };

    // A mount: one placement of an asset's bytes, with the mutable state that
    // placement needs. There is exactly one mount per (asset, attachment
    // point) no matter how many instances walk it.
    struct PageRt
    {
        PageView              page;           // borrowed from the asset
        uint32_t              asset = kInvalidIndex;
        // Effective error ceiling for every node in this page: the owning
        // expansion node's effective error, or FLT_MAX for a root page.
        //
        // Invariant (D) across a page boundary used to be established by
        // REWRITING the child page's error array at attach time. It is a
        // per-attachment scalar instead, folded into the wide test with one
        // min8. That is what lets a single page back many attachments — and
        // it turns attach from an O(nodeCount) write pass into O(1).
        float                 errClamp = FLT_MAX;
        std::vector<uint8_t>  resident;        // this node's payload is loaded
        std::vector<uint8_t>  covered;         // self or a complete resident descendant cut
        std::vector<uint32_t> coveredChildren; // covered immediate children
        // Bumped by anything that can change what a walk of this page emits:
        // a child attaching or detaching, a payload becoming resident or
        // ceasing to be, a new error clamp. A SelectionContext records this per
        // page it walked, which is what lets one shared asset streaming in
        // invalidate every instance of it without any per-instance work.
        uint32_t   contentVersion = 0;
        uint32_t   generation = 0;            // bumped per attach; invalidates handles
        uint32_t   lastTouched = 0;           // world frame of last walk touch
        uint32_t   lruPrev = kInvalidIndex, lruNext = kInvalidIndex;
        uint32_t   attachedChildPages = 0;
        bool       pinned = false;
        bool       inUse  = false;
        NodeRef    owner;                     // expansion node above; invalid for root pages
        // Attached child slot per expansion node, kInvalidIndex when
        // collapsed. Allocated on the page's first child attach; this IS the
        // expansion link — there is no by-id index.
        std::vector<uint32_t> expSlot;
    };

    // Copy-on-write bounds for one (instance, mount) pair: the only part of a
    // page a deformed instance stops sharing. Allocated on that instance's
    // first setNodeBounds touching the mount, and on any ancestor mount that
    // the resulting growth propagates into.
    struct Overlay
    {
        uint32_t slot = kInvalidIndex;   // mount this shadows
        uint32_t generation = 0;         // that mount's generation when taken
#ifdef HLOD_OVERLAY_FULL_PAGE
        // Experiment: privatise the WHOLE page on first deform rather than
        // only its bounds. Topology and boxes land back in one contiguous
        // blob, so refit walks a single region again — at the cost of the
        // instance dropping out of sharing entirely. A/B it with
        // BM_LeafRefit_* and BM_DeformedCutCost.
        Page page;
#else
        std::vector<AABB>       bbox;    // nodeCount
        std::vector<WideBounds> wide;    // wideCount (bounds only; the rest of
                                         // each WideBlock stays shared)
#endif
        bool inUse = false;
    };

    // (mount slot -> overlay index), kept sorted by slot on each instance so
    // the traversal can binary search it. Empty for undeformed instances,
    // which is the case the hot path is tuned for.
    struct OverlayRef
    {
        uint32_t slot;
        uint32_t index;
    };

    // Exactly one cache line containing everything the cut walk and
    // SelectionContext read per visible instance. TLAS-only state lives in the
    // parallel InstanceTlas array below, so the instance prefetch no longer
    // drags a second line into the latency-bound walk.
    struct Instance
    {
        float4   pos{};
        std::vector<OverlayRef> overlays;   // sorted by slot; usually empty
        float    scale = 1.0f;
        uint32_t rootSlot = kInvalidIndex;
        uint32_t tag = 0;
        uint32_t generation = 0;   // stamps InstanceRefs; bumped per reuse
        // Bumped when this instance moves, is rescaled, or is deformed. A
        // deform always privatises bounds into this instance's own overlay, so
        // geometry changes never reach past the instance that caused them and
        // this one counter covers all of them.
        uint32_t cutVersion = 0;
        bool     alive = false;
    };
    static_assert(sizeof(Instance) == 64, "cut-path Instance must stay one cache line");

    // State used only while building, editing, or refitting the TLAS. Keeping
    // it parallel preserves dense indexing without charging the cut loop for
    // bytes it never reads.
    struct InstanceTlas
    {
        AABB     worldBox;
        float    maxErrWorld = 0.0f;
        uint32_t asset = kInvalidIndex;
        uint32_t mask = ~0u;
        uint32_t tlasNode = kInvalidIndex;
        uint32_t tlasLane = 0;
        uint32_t liveIndex = kInvalidIndex;
        // Escape budgeting is population-based: once this instance has grown
        // beyond its build-time lane, later growth before the next rebuild
        // must not charge the same leaf again.
        bool     escapedSinceBuild = false;
    };
    static_assert(sizeof(InstanceTlas) == 64,
                  "TLAS instance state should stay one cache line");

    // nullptr when the ref is stale (slot recycled) or invalid.
    Instance* resolveInstance(InstanceRef ref);

    // Wide top-level BVH node; lanes are children (inner nodes or instances).
    struct TlasNode
    {
        WideBounds bounds;
        float8     maxErr{};
        int32_t    child[kWide];   // >= 0: node index; < 0: instance ~child
        uint32_t   laneMask[kWide];// union of the lane subtree's instance masks
        uint32_t   validMask = 0;
        int32_t    parent = -1;
    };

    struct TlasItem;

    // One page queued on an instance walk. Which bounds this instance sees is
    // resolved once, when the page is pushed, so the inner loop never asks
    // whether there is an overlay — it just reads through a stride.
    struct WorkItem
    {
        uint32_t      slot;
        WideBoundsRef wide;   // page's (interleaved) or overlay's (packed)
        uint8_t       current;
        uint8_t       ideal;
        uint8_t       mask;
    };

    // Node visit carried on the walk's explicit DFS stack; err and planes are
    // computed by the parent's wide test; current/ideal identify which walks
    // still contain the node.
    struct NodeItem
    {
        uint32_t node;
        float    err;
        uint8_t  planes;
        uint8_t  current;
        uint8_t  ideal;
    };

    // Everything one instance walk needs. Serial selection uses slot 0; a
    // parallel selection gives each worker its own, then concatenates in
    // instance order so results stay bit-identical either way.
    struct Worker
    {
        std::vector<WorkItem> work;
        std::vector<NodeItem> nodeStack;

        // Backing storage for the parallel path, where each worker collects
        // into its own vectors; the serial path points the sinks straight at
        // the caller's output instead.
        std::vector<CutEntry> cutBuf;

        CutSink cut;

        std::vector<uint32_t> touched;      // page dependencies / optional usage
        bool trackTouches = false;
        bool uniqueTouches = false;         // SelectionContext dependency list
        CutStats stats;

        // Validity-margin tracking for SelectionContext. Off for every ordinary
        // selection, and the branch is invariant for a whole call.
        bool  trackMargin = false;
        float margin = FLT_MAX;   // min distance to a decision flip so far
        float maxError = 0.0f;    // max effective error among decided nodes
        float bar = 0.0f;         // params.threshold, for the margin arithmetic
    };

    // ---- helpers ----
    // Live PageRt for a handle, or nullptr if the handle is stale/foreign.
    const PageRt* resolve(NodeHandle h) const;
    PageRt*       resolve(NodeHandle h)
    {
        return const_cast<PageRt*>(static_cast<const World*>(this)->resolve(h));
    }

    uint32_t allocAsset();
    uint32_t createAsset(Page&& owned, PageView borrowed, bool registered);
    void     destroyAssetIfUnused(uint32_t asset);
    const AssetRt* resolveAsset(AssetHandle h) const;

    uint32_t allocSlot();
    uint32_t registerPage(uint32_t asset, NodeRef owner, bool pinned);
    void     detachSlot(uint32_t slot, std::vector<UserPayload>* freedPayloads);
    void     detachMountTree(uint32_t rootSlot, std::vector<UserPayload>* freedPayloads);
    void     pinRootPayloads(uint32_t slot);
    bool     descendantsCovered(uint32_t slot, uint32_t node) const;
    bool     computeCovered(uint32_t slot, uint32_t node) const;
    void     propagateCoverage(uint32_t slot, uint32_t node);

    void lruUnlink(uint32_t slot);
    void lruPushFront(uint32_t slot);
    void lruTouch(uint32_t slot, uint32_t epoch);
    void consumePageUsage(PageUsageContext& usage);
    void consumePageUsage(std::initializer_list<PageUsageContext*> usage);

    // ---- copy-on-write bounds ----
    // Live overlay for (instance, slot), or nullptr. Stale overlays (the
    // mount was detached and its slot reused) report as absent.
    const Overlay* findOverlay(const Instance& inst, uint32_t slot) const;
    // Same, but takes the copy if it is not there yet. Returns an INDEX, not a
    // reference: taking one overlay can reallocate the pool and invalidate a
    // reference to another, and refit holds two at a time as it crosses a page
    // boundary.
    uint32_t       ensureOverlay(Instance& inst, uint32_t slot);
    void           initOverlay(Overlay& ov, const PageRt& rt);
    void           freeOverlays(Instance& inst);
    // Effective bounds for a walk of `slot` by `inst`.
    const AABB*    boundsFor(const Instance& inst, uint32_t slot, const PageRt& rt) const;
    WideBoundsRef  wideBoundsFor(const Instance& inst, uint32_t slot, const PageRt& rt) const;
    // Debug-only: is `slot` reachable from this instance's root mount?
    bool mountBelongsTo(const Instance& inst, uint32_t slot) const;

    void applyBoundsChange(InstanceId id, uint32_t slot, uint32_t index, const AABB& box);
    void patchParentLane(const PageView& pg, AABB* bbox, MutWideBoundsRef wide,
                         uint32_t index);
    void refreshInstanceBounds(InstanceId id);

    InstanceRef addInstanceInternal(uint32_t asset, const InstanceDesc& desc);

    void markTlasStructuralChange();
    void tlasRebuild();
    int32_t tlasBuildRange(std::vector<uint32_t>& items, int lo, int hi, int32_t parent);
    int  tlasSplit(std::vector<uint32_t>& items, int lo, int hi);
    void tlasQuery(const CullView& view, float minPix,
                   std::vector<std::pair<uint32_t, uint8_t>>& outVisible,
                   std::vector<TlasItem>& stack) const;
    void recomputeInstanceBounds(InstanceId id);
    void tlasOnInstanceMoved(InstanceId id);
    void tlasNoteGrowth(float addedArea);

    // Incremental structural edits, so that spawning or removing one instance
    // costs O(tree depth) instead of a full rebuild. Both no-op when a rebuild
    // is already pending, since it will see the change anyway.
    void tlasInsert(InstanceId id);
    void tlasRemove(InstanceId id);
    // Grow a lane box up the parent chain, stopping as soon as an ancestor
    // already covers it. Returns the lane area added, for the drift trigger.
    float tlasGrowUp(uint32_t nodeIdx, const AABB& box, float maxErr, uint32_t laneMask);
    int32_t tlasAllocNode();
    // Union of a node's valid lanes, which is what its parent's lane must hold.
    AABB tlasNodeExtent(const TlasNode& n, float& maxErr, uint32_t& laneMask) const;
    void tlasNoteEdit();

    bool visibleDescendantsCovered(uint32_t slot, uint32_t node, uint8_t mask,
                                   const Instance& inst,
                                   const CullView& local,
                                   std::vector<uint32_t>* touched = nullptr,
                                   bool uniqueTouches = true) const;
    void runInstance(uint32_t instIdx, const CullView& view, const CutParams& params,
                     uint8_t mask, Worker& w) const;
    void runPage(const WorkItem& item, const Instance& inst, const CullView& local,
                 const CutParams& params, Worker& w) const;
    void wideVisit(const WorkItem& item, const PageView& pg, float errClamp,
                   uint32_t gen, uint32_t tag, uint32_t node, uint8_t mask,
                   uint8_t currentKids, uint8_t idealKids,
                   const CullView& local, Worker& w) const;
    void selectCutContext(const CullView& view, const CutParams& params,
                          SelectionContext& ctx, PageUsageContext* usage,
                          CutSink& outCut) const;
    void selectCutStateless(const CullView& view, const CutParams& params,
                            PageUsageContext* usage, CutSink& outCut);
    void recordPageUsage(PageUsageContext& usage, uint32_t slot) const;

    // ---- state ----
    WorldConfig config_;

    std::vector<AssetRt>  assets_;
    std::vector<uint32_t> freeAssets_;
    size_t                liveAssets_ = 0;

    std::vector<PageRt>   slots_;
    std::vector<uint32_t> freeSlots_;
    size_t                attachedPages_ = 0;
    size_t                pinnedPages_ = 0;
    uint32_t              generationCounter_ = 0;

    std::vector<Instance> instances_;
    std::vector<InstanceTlas> instanceTlas_;
    std::vector<InstanceId> liveInstances_;
    std::vector<uint32_t> freeInstances_;

    std::vector<Overlay>  overlays_;
    std::vector<uint32_t> freeOverlays_;
    size_t                liveOverlays_ = 0;

    std::vector<TlasNode> tlasNodes_;
    int32_t               tlasRoot_ = -1;
    bool                  tlasDirty_ = true;
    bool                  tlasQualityBuild_ = true;   // quality tier vs Morton
    uint32_t              tlasEscapes_ = 0;
    uint32_t              tlasEdits_ = 0;           // incremental inserts + removes
    std::vector<int32_t>  tlasFreeNodes_;           // nodes emptied by removal
    uint32_t              tlasLeafCount_ = 0;
    uint32_t              tlasQualityCount_ = 0;   // leaves at last quality build
    float                 tlasBaseArea_ = 0.0f;    // lane area at last quality build
    float                 tlasGrownArea_ = 0.0f;   // area added by grow-only refit

    uint32_t lruHead_ = kInvalidIndex, lruTail_ = kInvalidIndex;
    uint32_t frame_ = 0;
    CutStats stats_;

    // Submitted bounds in submission order. The generation stamps make
    // entries self-invalidating: if the page detached (or either slot was
    // reused) before the flush, the entry is skipped instead of applying to
    // the wrong page or the wrong instance.
    struct PendingMove
    {
        uint32_t instance, instGeneration;
        uint32_t slot, generation, index;
        AABB     box;
    };
    std::vector<PendingMove> pendingMoves_;

    // per-call scratch (kept to avoid reallocation)
    struct TlasItem { int32_t node; uint8_t mask; };
    std::vector<Worker>                        workers_;
    std::vector<std::pair<uint32_t, uint8_t>>  visibleTmp_;
    std::vector<TlasItem>                      tlasStack_;
    std::vector<std::pair<uint64_t, uint32_t>> tlasKeys_;
    std::vector<std::pair<uint64_t, uint32_t>> tlasKeysTmp_;
    std::vector<int32_t>                       tlasLevelTmp_;
    std::vector<uint32_t>                      tlasItemsTmp_;
};

} // namespace hlod
