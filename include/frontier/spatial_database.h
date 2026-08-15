#pragma once
// Permanent renderable roots live directly in a build-configured BVH4/BVH8
// TLAS. Reusable
// immutable subtree definitions mount only beneath those roots or beneath
// renderable mountable nodes in another mounted subtree. Single-node objects
// therefore allocate no hierarchy storage, while deep assemblies share their
// immutable topology, payload, error, and authored-bound data.
//
// Runtime operations are generation-stamped and handle-based. NodeHandle
// values come from frontier entries or InstanceHandle::rootNode();
// SubtreeHandle names registered bytes; SubtreeInstanceHandle names one
// mounted placement; and InstanceHandle names one permanent TLAS root.
//
// SpatialDatabase updates are single-writer. applyUpdates() publishes a stable snapshot;
// SpatialQuery::selectFrontier then reads only that snapshot and may run concurrently when
// each call has its own SpatialQuery and outputs. LOD damping, frontier reuse, query scratch,
// and optional selection statistics live in the SpatialQuery, never as mutable
// SpatialDatabase state.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <memory>
#include <span>
#include <type_traits>
#include <vector>

#include "config.h"
#include "detail/append_buffer.h"
#include "detail/subtree_data.h"
#include "math.h"
#include "node.h"
#include "subtree.h"

namespace frontier {

class SpatialDatabase;
struct QueryScratch;

// One registered immutable subtree definition.
struct SubtreeHandle
{
    uint32_t slot = kInvalidIndex;
    uint32_t generation = 0;
    constexpr bool valid() const noexcept { return slot != kInvalidIndex; }
    friend constexpr bool operator==(SubtreeHandle, SubtreeHandle) = default;
};
static_assert(sizeof(SubtreeHandle) == 8, "SubtreeHandle must stay 64 bits");

// One mounted placement of a registered subtree definition.
struct SubtreeInstanceHandle
{
    uint32_t slot = kInvalidIndex;
    uint32_t generation = 0;
    constexpr bool valid() const noexcept { return slot != kInvalidIndex; }
    friend constexpr bool operator==(SubtreeInstanceHandle,
                                     SubtreeInstanceHandle) = default;
};
static_assert(sizeof(SubtreeInstanceHandle) == 8,
              "SubtreeInstanceHandle must stay 64 bits");

// Resolved node reference, packed into 64 bits. Mounted-subtree nodes use a
// 20-bit placement slot, a 20-bit local index, and a 24-bit generation. The
// reserved slot code tags a TLAS-owned renderable root instead: its other
// 44 bits carry a stable 24-bit InstanceId and a 20-bit generation. This keeps
// FrontierEntry at 12 bytes while making top-level and mounted nodes uniform.
struct NodeHandle
{
    static constexpr uint32_t kSlotBits = 20;
    static constexpr uint32_t kIndexBits = 20;
    static constexpr uint32_t kGenerationBits = 24;
    static constexpr uint32_t kSlotMask = (1u << kSlotBits) - 1u;
    static constexpr uint32_t kIndexMask = (1u << kIndexBits) - 1u;
    static constexpr uint32_t kGenerationMask = (1u << kGenerationBits) - 1u;
    static constexpr uint32_t kInvalidSlot = kSlotMask;
    static constexpr uint32_t kTlasGenerationBits = 20;
    static constexpr uint32_t kTlasGenerationMask =
        (1u << kTlasGenerationBits) - 1u;

    uint32_t lo = kInvalidSlot;
    uint32_t hi = 0;

    constexpr NodeHandle() = default;
    constexpr NodeHandle(uint32_t slot, uint32_t index, uint32_t generation)
        : lo((slot & kSlotMask) | ((index & 0xfffu) << kSlotBits)),
          hi(((index >> 12) & 0xffu) |
             ((generation & kGenerationMask) << 8))
    {}

    constexpr uint32_t slot() const { return lo & kSlotMask; }
    constexpr uint32_t index() const
    {
        return ((lo >> kSlotBits) & 0xfffu) | ((hi & 0xffu) << 12);
    }
    constexpr uint32_t generation() const { return hi >> 8; }
    constexpr bool isTlasRoot() const
    {
        return slot() == kInvalidSlot &&
               (generation() & kTlasGenerationMask) != 0;
    }
    constexpr bool valid() const
    {
        return slot() != kInvalidSlot || isTlasRoot();
    }
    constexpr uint32_t tlasInstance() const
    {
        return index() | ((generation() >> kTlasGenerationBits) << kIndexBits);
    }
    constexpr uint32_t tlasGeneration() const
    {
        return generation() & kTlasGenerationMask;
    }
    static constexpr NodeHandle tlasRoot(uint32_t instance,
                                         uint32_t instanceGeneration)
    {
        return NodeHandle{kInvalidSlot, instance & kIndexMask,
                          ((instance >> kIndexBits) << kTlasGenerationBits) |
                              (instanceGeneration & kTlasGenerationMask)};
    }

    friend constexpr bool operator==(NodeHandle a, NodeHandle b)
    {
        return a.lo == b.lo && a.hi == b.hi;
    }
};
static_assert(sizeof(NodeHandle) == 8, "NodeHandle must stay 64 bits");

using InstanceId = uint32_t;
inline constexpr uint32_t kInstanceIdBits = 24;
inline constexpr InstanceId kInstanceIdMask = (1u << kInstanceIdBits) - 1u;
inline constexpr InstanceId kInvalidInstanceId = kInstanceIdMask;
inline constexpr uint8_t kFrontierErrorThreshold = 128;

// One live top-level placement. Every instance owns exactly one permanent,
// renderable TLAS root, so rootNode() is always valid while the handle is live.
struct InstanceHandle
{
    InstanceId id = kInvalidInstanceId;
    uint32_t generation = 0;

    constexpr bool valid() const noexcept { return id != kInvalidInstanceId; }
    constexpr NodeHandle rootNode() const noexcept
    {
        return valid() ? NodeHandle::tlasRoot(id, generation) : NodeHandle{};
    }
    friend constexpr bool operator==(InstanceHandle, InstanceHandle) = default;
};
static_assert(sizeof(InstanceHandle) == 8,
              "InstanceHandle must stay 64 bits");

// Threshold-relative logarithmic error. Codes [0, 127] are at or below the
// selection threshold and [128, 255] are above it. The boundary decision is
// exact even though magnitude is quantized; roughly eight codes cover each
// power-of-two step away from the threshold.
uint8_t encodeFrontierError(float error, float threshold);
float   decodeFrontierError(uint8_t code, float threshold);

// A frontier entry is deliberately 12 bytes: one logical 64-bit node handle plus
// a packed stable 24-bit InstanceId and 8-bit screen-error code. User payloads
// and application entity data stay in caller-side tables indexed by the
// instance id, rather than being repeated in every query result.
struct FrontierEntry
{
    NodeHandle nodeHandle;
    uint32_t   instanceAndError = kInvalidInstanceId;

    FrontierEntry() = default;
    FrontierEntry(NodeHandle node, float error, float threshold, InstanceId instance)
        : nodeHandle(node),
          instanceAndError((instance & kInstanceIdMask) |
                           (uint32_t(encodeFrontierError(error, threshold)) <<
                            kInstanceIdBits))
    {}
    FrontierEntry(NodeHandle node, uint8_t encodedError, InstanceId instance)
        : nodeHandle(node),
          instanceAndError((instance & kInstanceIdMask) |
                           (uint32_t(encodedError) << kInstanceIdBits))
    {}

    InstanceId instance() const { return instanceAndError & kInstanceIdMask; }
    uint8_t errorCode() const
    {
        return uint8_t(instanceAndError >> kInstanceIdBits);
    }
    bool overThreshold() const { return errorCode() >= kFrontierErrorThreshold; }
    float approximateError(float threshold) const
    {
        return decodeFrontierError(errorCode(), threshold);
    }
};
static_assert(sizeof(FrontierEntry) == 12, "FrontierEntry must stay 12 bytes");

// Zero-copy forward range over two result buckets. Iteration exhausts `first`
// before continuing with `second`; neither bucket is copied or made contiguous.
class FrontierCutView
{
public:
    class iterator
    {
    public:
        using iterator_category = std::forward_iterator_tag;
        using iterator_concept = std::forward_iterator_tag;
        using value_type = FrontierEntry;
        using difference_type = std::ptrdiff_t;
        using pointer = const FrontierEntry*;
        using reference = const FrontierEntry&;

        iterator() = default;

        reference operator*() const { return *current_; }
        pointer operator->() const { return current_; }

        iterator& operator++()
        {
            ++current_;
            if (firstEnd_)
            {
                if (current_ != firstEnd_) return *this;
                current_ = secondBegin_;
                firstEnd_ = nullptr;
                if (current_ != secondEnd_) return *this;
            }
            else if (current_ != secondEnd_)
            {
                return *this;
            }

            *this = iterator{};
            return *this;
        }

        iterator operator++(int)
        {
            iterator previous = *this;
            ++*this;
            return previous;
        }

        friend bool operator==(const iterator&, const iterator&) = default;

    private:
        friend class FrontierCutView;

        iterator(std::span<const FrontierEntry> first,
                 std::span<const FrontierEntry> second)
            : secondBegin_(second.data()),
              secondEnd_(second.empty() ? second.data()
                                        : second.data() + second.size())
        {
            if (!first.empty())
            {
                current_ = first.data();
                firstEnd_ = first.data() + first.size();
            }
            else if (!second.empty())
            {
                current_ = second.data();
            }
            else
            {
                *this = iterator{};
            }
        }

        const FrontierEntry* current_ = nullptr;
        const FrontierEntry* firstEnd_ = nullptr;
        const FrontierEntry* secondBegin_ = nullptr;
        const FrontierEntry* secondEnd_ = nullptr;
    };

    FrontierCutView() = default;
    FrontierCutView(std::span<const FrontierEntry> first,
                    std::span<const FrontierEntry> second)
        : first_(first), second_(second)
    {}

    iterator begin() const { return iterator(first_, second_); }
    iterator end() const { return {}; }
    size_t size() const { return first_.size() + second_.size(); }
    bool empty() const { return first_.empty() && second_.empty(); }

private:
    std::span<const FrontierEntry> first_;
    std::span<const FrontierEntry> second_;
};
static_assert(sizeof(FrontierCutView) == 4 * sizeof(void*),
              "two-span frontier cut view grew");

// One traversal produces both realities without per-entry tags. The current
// render frontier is shared + currentOnly. The fully-ready ideal frontier is
// shared + idealOnly. A high-error ideal-side mountable node can be used
// directly as the parent in mountSubtree().
// Non-owning result of one SpatialQuery selection. The spans remain valid until the
// next selection or reset on that SpatialQuery, or until the SpatialQuery is destroyed.
struct FrontierResultView
{
    std::span<const FrontierEntry> shared;
    std::span<const FrontierEntry> currentOnly;
    std::span<const FrontierEntry> idealOnly;

    FrontierCutView current() const { return {shared, currentOnly}; }
    FrontierCutView ideal() const { return {shared, idealOnly}; }
    size_t currentSize() const { return shared.size() + currentOnly.size(); }
    size_t idealSize() const { return shared.size() + idealOnly.size(); }
    size_t size() const
    {
        return shared.size() + currentOnly.size() + idealOnly.size();
    }
    bool empty() const { return size() == 0; }
};

namespace detail {

struct FrontierBuffers
{
    AppendBuffer<FrontierEntry> shared;
    AppendBuffer<FrontierEntry> currentOnly;
    AppendBuffer<FrontierEntry> idealOnly;

    void clear()
    {
        shared.clear();
        currentOnly.clear();
        idealOnly.clear();
    }

    FrontierResultView view() const
    {
        return {{shared.data(), shared.size()},
                {currentOnly.data(), currentOnly.size()},
                {idealOnly.data(), idealOnly.size()}};
    }
};

} // namespace detail

// Explicit owning result for callers that need to retain a frontier across the
// next selection. The normal API returns FrontierResultView and keeps ownership in SpatialQuery;
// this storage type exposes only spans, not its container.
class FrontierResult : public FrontierResultView
{
public:
    FrontierResult() = default;
    FrontierResult(const FrontierResult& other) : buffers_(other.buffers_) { sync(); }
    FrontierResult(FrontierResult&& other) noexcept : buffers_(std::move(other.buffers_))
    {
        sync();
        other.sync();
    }
    FrontierResult& operator=(const FrontierResult& other)
    {
        if (this != &other)
        {
            buffers_ = other.buffers_;
            sync();
        }
        return *this;
    }
    FrontierResult& operator=(FrontierResult&& other) noexcept
    {
        if (this != &other)
        {
            buffers_ = std::move(other.buffers_);
            sync();
            other.sync();
        }
        return *this;
    }

private:
    friend class SpatialQuery;
    friend class SpatialDatabase;

    void sync() { static_cast<FrontierResultView&>(*this) = buffers_.view(); }

    detail::FrontierBuffers buffers_;
};

// How the renderable current cut replaces an unavailable ideal-cut node.
enum class CurrentCutPolicy : uint8_t
{
    // Prefer a complete ready descendant cut. Fall back to a ready ancestor
    // only when the descendant cover is incomplete. This is usually the most
    // detailed renderable result and is the default.
    PreferReadyDescendants,

    // Do not search below an unavailable ideal-cut node. Replace it with the
    // nearest ready ancestor that can cover the affected branches. This
    // normally emits fewer, coarser entries.
    PreferReadyAncestors,
};

struct SelectionParams
{
    float threshold = 4.0f;   // refine when screen error exceeds this (px)
    float minPix    = 0.0f;   // contribution culling at the top level; 0 = off
    CurrentCutPolicy currentCutPolicy =
        CurrentCutPolicy::PreferReadyDescendants;
};

struct InstanceDesc
{
    float4   pos{};
    float    scale = 1.0f;

    // ANDed against Camera::viewMask; a zero result culls the instance at
    // the top level. Cheap layer visibility: shadow-only props, editor-only
    // gizmos, per-view opt-outs.
    uint32_t mask = ~0u;
};

// ---------------------------------------------------------------------------
// Output sinks
//
// selectFrontier writes through a Sink so the same out-of-line traversal can fill
// either retained internal storage or fixed caller memory that reports what
// did not fit. Engines that write straight into a draw list or mapped instance
// buffer use the public span constructor.
// ---------------------------------------------------------------------------

template <class T>
class Sink
{
    static_assert(std::is_trivially_copyable_v<T>,
                  "Sink requires trivially copyable values");

public:
    Sink() = default;

    // Fixed: writes into caller memory, counting (not writing) the overflow.
    explicit Sink(std::span<T> storage) : data_(storage.data())
    {
        FRONTIER_CHECK(storage.size() <= UINT32_MAX,
                   "Sink capacity exceeds 32-bit result count");
        capacity_ = uint32_t(storage.size());
    }

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
    // is how a SpatialQuery hands back an instance's recorded frontier.
    void pushRange(const T* p, uint32_t n)
    {
        if (n == 0) return;
        if (vec_)
        {
            vec_->append(p, n);
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

    void clear()
    {
        if (vec_) vec_->clear();
        count_ = 0;
        dropped_ = 0;
    }

private:
    friend class SpatialQuery;
    friend class SpatialDatabase;

    // Internal growable sink. Public callers see only fixed spans.
    explicit Sink(detail::AppendBuffer<T>& v) : vec_(&v) {}

    detail::AppendBuffer<T>* vec_ = nullptr;
    T*              data_ = nullptr;
    uint32_t        capacity_ = 0;
    uint32_t        count_ = 0;
    uint32_t        dropped_ = 0;
};

struct FrontierResultSink
{
    Sink<FrontierEntry> shared;
    Sink<FrontierEntry> currentOnly;
    Sink<FrontierEntry> idealOnly;

    FrontierResultSink() = default;
    FrontierResultSink(Sink<FrontierEntry> sharedSink,
                  Sink<FrontierEntry> currentOnlySink,
                  Sink<FrontierEntry> idealOnlySink)
        : shared(sharedSink), currentOnly(currentOnlySink), idealOnly(idealOnlySink)
    {}

private:
    friend class SpatialDatabase;
    bool retainsExisting_ = false;
};

// Traversal counters, filled only when the library is built with FRONTIER_STATS.
struct SelectionStats
{
    uint64_t instancesVisited = 0;
    uint64_t subtreesVisited = 0;
    uint64_t nodesVisited = 0;
    uint64_t wideBlocksTested = 0;
    uint64_t lanesSurvived = 0;
};

// Result of one mounted-topology collection pass. Definition-node readiness is
// retained when a placement is removed.
struct CollectResult
{
    size_t unmountedSubtrees = 0;
};

// ---------------------------------------------------------------------------
// SpatialQuery — reusable state for one logical spatial query
//
// A database that is mostly static, seen by a camera that moves continuously,
// produces a frontier that is nearly identical frame to frame. This exploits that:
// an instance whose frontier provably cannot have changed is not walked at all, and
// its entries are handed back where they already lie.
//
// THE STATE IS ALL HERE, NOT IN THE DATABASE. Querying several cameras per
// frame (main view, shadow cascades, a reflection probe) means several
// SpatialQuery objects, and they cannot interfere: SpatialDatabase stays a
// pure read during selection.
//
// It also owns the camera damper, and that is not merely tidy. The
// two are one mechanism read two ways: the damper turns a camera position into
// a query envelope, and the reuse test below is driven by how far that same
// envelope has travelled, not by the camera. Held separately, the caller has to
// pair the right damper with the right SpatialQuery by hand every frame, and pairing
// the main camera's damper with a shadow cascade's cache compiles perfectly well.
// So selectFrontier takes the raw Camera and damps it internally, one reset() covers
// the discontinuity for both halves, and the pairing cannot be got wrong.
//
// It also makes one coupling visible instead of hidden, which is the other
// reason to keep them together. The damper relaxes projection scale k as well
// as position. A flip point lies at `geometricError * k / threshold`, so the
// maximum error observed while recording a frontier gives a conservative slope for
// how far any of its decisions can move per unit k. The SpatialQuery accumulates
// absolute k travel and charges that per-record slope against the same validity
// margin used for camera travel.
//
// During damped zoom-out, k changes a little over many frames. The slope
// budget keeps unaffected records reusable while preserving the exact node
// set. Threshold changes still bump an epoch because they change every stored
// slope at once.
//
// Damping and reuse are independent. A half-life of 0 makes the arithmetic
// bit-identical to an undamped query; setReuseEnabled(false) selects the
// uncached traversal and avoids allocating records. A fully dynamic database may
// want damping without reuse, while a shadow cascade often wants the reverse.
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
// them, and their frontier is therefore a pure function of camera position. An
// instance straddling the frustum edge is always re-walked, and there are few
// of those -- they are a shell, not a volume.
//
// The margin is large exactly where it needs to be. A distant instance sits
// far past every flip point, so its margin is roughly its own distance and it
// stays valid for many frames; a near instance with a deep frontier has some node
// sitting right at the threshold, so its margin is tiny and it is re-walked
// every frame. That is the correct division of labour rather than a
// limitation: the population is dominated by distant instances, which is where
// the per-instance fixed cost (resolve the instance, transform the view, touch
// the mounted root) was being paid over and over for an answer that never moved.
//
// BUCKETED OUTPUT
// ---------------
// Cached and uncached modes write the same three FrontierEntry sequences. Reused
// entries are copied from SpatialQuery-owned storage into the matching caller bucket;
// no SpatialDatabase-owned mutable query state is involved.
//
// WHAT IS EXACT AND WHAT IS NOT
// -----------------------------
// The set of emitted nodes is exactly what an uncached selectFrontier would emit.
// The compact error code is recorded with that frontier and may therefore be stale
// within the proven margin. It remains useful for coarse prioritisation and
// threshold classification; do not expect a cached result to reproduce a
// freshly measured pixel error.
//
// Streaming demand and readiness are part of the recorded three-bucket result. Mount
// content versions invalidate a record when any dependency changes. Instances
// that cross more dependencies than the compact record can hold are
// simply re-walked.
// ---------------------------------------------------------------------------

class SpatialQuery
{
public:
    SpatialQuery();
    explicit SpatialQuery(float halfLifeFrames);
    ~SpatialQuery();

    SpatialQuery(SpatialQuery&&) noexcept;
    SpatialQuery& operator=(SpatialQuery&&) noexcept;
    SpatialQuery(const SpatialQuery&) = delete;
    SpatialQuery& operator=(const SpatialQuery&) = delete;

    // LOD hysteresis for this query, in frames; 0 disables it exactly. See
    // CameraDamper in math.h for what the envelope does.
    float halfLife() const { return damper_.halfLife(); }
    void  setHalfLife(float frames) { damper_.setHalfLife(frames); }

    // Instances served from the cache, and re-walked, in the last call.
    uint32_t reused() const { return reused_; }
    uint32_t walked() const { return walked_; }

    // Counters from this SpatialQuery's last call. Per-query ownership keeps
    // concurrent instrumented queries from racing over a global value.
    // Non-instrumented builds return one immutable zero value and store no
    // otherwise-dead counters in every query.
    const SelectionStats& lastSelectionStats() const
    {
#ifdef FRONTIER_STATS
        return stats_;
#else
        static const SelectionStats empty{};
        return empty;
#endif
    }

    // Cached hierarchical selection is the default. All-flat TLAS snapshots
    // use the direct path automatically because they have no hierarchy walk
    // to cache. Disable reuse for highly dynamic hierarchical queries that
    // prefer the internally parallel uncached traversal. Changing modes resets
    // accumulated damping and reuse state but retains allocations.
    bool reuseEnabled() const { return reuseEnabled_; }
    void setReuseEnabled(bool enabled);

    // Track mounted-subtree touches for garbage collection. Disabled by
    // default, so queries that do not drive retention pay no recording cost.
    bool mountUsageEnabled() const { return mountUsageEnabled_; }
    void setMountUsageEnabled(bool enabled);
    void resetMountUsage();

    // Query a snapshot published by SpatialDatabase::applyUpdates(). The SpatialQuery
    // is mutable and must not be used concurrently; any number of distinct queries
    // may read the same const SpatialDatabase concurrently. A query binds to the
    // first database it reads, and reset() releases that binding.
    FrontierResultView selectFrontier(const SpatialDatabase& database, const Camera& camera,
                            const SelectionParams& params);

    // Advanced zero-copy path: write directly to fixed caller storage.
    void selectFrontier(const SpatialDatabase& database, const Camera& camera,
                   const SelectionParams& params, FrontierResultSink& outResult);

    // Explicit owning-result path for callers that must retain a frontier after the
    // next selection on this SpatialQuery.
    void selectFrontier(const SpatialDatabase& database, const Camera& camera,
                   const SelectionParams& params, FrontierResult& outResult);

    // Forget everything: the damping window and every record. Call it on a
    // teleport or camera cut. Reuse never *requires* this -- every record
    // carries the stamps and the travel budget that invalidate it -- but
    // damping does, or the envelope stretches across the discontinuity and
    // over-refines everything between the two positions. One call so that
    // cannot be half-done.
    void reset();

    size_t bytes() const;

private:
    friend class SpatialDatabase;

    // Mounted trees whose state this instance's frontier depended on. Every
    // descendant mutation bumps the tree root, so a city assembled from any
    // number of placements normally has one dependency.
    static constexpr uint32_t kMaxDeps = 2;

    // EXACTLY 32 HOT BYTES, and that is the whole design constraint.
    //
    // This record is read for every visible instance, indexed by instance id,
    // so it is a random access per instance -- the one new memory stream the
    // cache introduces. It has to be cheaper than the walk it replaces, which
    // is often already cheap because shared subtree bytes stay hot. Everything
    // here is therefore either a scalar or absent:
    //  - no camera envelope: validity is a single scalar budget against the
    //    query's accumulated travel (see travel_);
    //  - no per-record k / threshold copies: threshold changes bump epoch_,
    //    while k motion consumes the scalar slope budget below;
    //  - no placement generations: the mounted-tree root contentVersion is bumped
    //    on descendant mutation and mount, and distinguishes a recycled slot.
    struct Rec
    {
        // Reusable while positionTravel + kSlope * kTravel has not passed
        // this. Set from that same expression plus the measured margin.
        float    validUntil = 0.0f;
        float    kSlope = 0.0f;      // max world-space flip travel per unit k
        uint32_t epoch = 0;          // cache epoch (threshold generation)
        uint32_t frontierVersion = 0;     // unique instance transform/deform version
        uint32_t begin = 0;          // block in store_
        // Three 10-bit counts for [shared, currentOnly, idealOnly], plus the
        // two-bit dependency count. Dependency count 3 is an escape marker;
        // the low 30 bits then index sparse full-width counts.
        uint32_t counts = 0;
        // The overwhelmingly common one-mount dependency stays inline. A
        // second dependency lives in the cold spill array below.
        uint32_t depSlot = 0;
        uint32_t depVersion = 0;
    };
    static_assert(sizeof(Rec) == 32, "SpatialQuery::Rec hot state must stay 32 bytes");

    // Allocation state is needed only when a record is replaced or the slab
    // is compacted. Keeping it out of Rec prevents every hit from fetching it.
    struct RecCold
    {
        uint32_t capacity = 0;
    };
    static_assert(sizeof(RecCold) == 4, "SpatialQuery cold record must stay 4 bytes");

    struct SecondDep
    {
        uint32_t slot = 0;
        uint32_t version = 0;
    };
    static_assert(sizeof(SecondDep) == 8, "second dependency must stay 8 bytes");

    struct OverflowCounts
    {
        uint32_t shared = 0;
        uint32_t current = 0;
        uint32_t ideal = 0;
        uint32_t dependencies = 0;
    };
    static_assert(sizeof(OverflowCounts) == 16,
                  "large-frontier count spill must stay 16 bytes");

    struct MountUseRec
    {
        static constexpr uint32_t kPending = 1u << 31;
        uint32_t generationAndPending = 0;
        uint32_t lastUsed = 0;

        uint32_t generation() const
        {
            return generationAndPending & NodeHandle::kGenerationMask;
        }
        bool pending() const
        {
            return (generationAndPending & kPending) != 0;
        }
        void setGeneration(uint32_t generation)
        {
            generationAndPending = generation & NodeHandle::kGenerationMask;
        }
        void setPending(bool pending)
        {
            if (pending) generationAndPending |= kPending;
            else generationAndPending &= ~kPending;
        }
    };
    static_assert(sizeof(MountUseRec) == 8,
                  "mount-use record must stay 8 bytes");

    void compact();

    // This query's hysteresis. selectFrontier damps through it, so the odometer
    // below measures the envelope it produces.
    CameraDamper damper_;

    std::vector<Rec>      rec_;      // hit-path state, by InstanceId
    std::vector<RecCold>  recCold_;  // miss/compaction state, by InstanceId
    // Allocated only after a cacheable walk actually needs two dependencies.
    // Ordinary selection coalesces a whole mounted tree to its root stamp.
    detail::AppendBuffer<SecondDep> secondDep_;
    // Sparse: allocated only for cacheable frontiers whose bucket counts do
    // not fit the inline 10-bit fields.
    detail::AppendBuffer<OverflowCounts> overflowCounts_;
    detail::AppendBuffer<uint32_t> freeOverflowCounts_;
    detail::AppendBuffer<FrontierEntry> store_;   // slab of recorded runs
    // Distance the damped query envelope has travelled since this SpatialQuery was
    // created, accumulated per call. kTravel_ similarly accumulates absolute
    // projection-scale motion. A record taken with margin m remains valid
    // while positionTravel + kSlope * kTravel stays below its saved budget:
    // both path lengths conservatively bound movement of every LOD flip point.
    // This replaces storing a camera and projection value per instance with
    // two scalar odometers and one per-record slope, keeping hot Rec at 32 bytes.
    // Doubling back is conservative; a teleport consumes the budget at once.
    float4   lastQmn_{}, lastQmx_{};
    float    travel_ = 0.0f;
    float    kTravel_ = 0.0f;
    float    k_ = 0.0f;
    float    bar_ = 0.0f;
    float    wholeMargin_ = 0.0f;
    float    wholeTravel_ = 0.0f;
    float    wholeKTravel_ = 0.0f;
    float    wholeMaxSlope_ = 0.0f;
    uint32_t used_ = 0;
    uint32_t garbage_ = 0;
    uint32_t instanceLayoutVersion_ = 0;
    uint32_t databaseGeneration_ = 0;
    uint32_t wholeEpoch_ = 0;
    // Bumped when the error threshold or current-cut policy changes,
    // invalidating every record in O(1). Projection-scale changes consume
    // kTravel_ instead.
    uint32_t epoch_ = 1;
    uint32_t reused_ = 0;
    uint32_t walked_ = 0;
#ifdef FRONTIER_STATS
    SelectionStats stats_{};
#endif
    const SpatialDatabase* database_ = nullptr;
    std::vector<MountUseRec> mountUse_;
    std::vector<uint32_t> dirtyMounts_;

    // Mutable query scratch. Opaque so traversal
    // implementation details do not become part of the public API.
    std::unique_ptr<QueryScratch> scratch_;
    CurrentCutPolicy currentCutPolicy_ =
        CurrentCutPolicy::PreferReadyDescendants;
    bool primed_ = false;
    bool wholeReusable_ = false;
    bool reuseEnabled_ = true;
    bool mountUsageEnabled_ = false;
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

struct SpatialDatabaseConfig
{
    // Allocation and task parallelism. Copied into the SpatialDatabase; callback code
    // and anything referenced by `user` must remain valid for its lifetime.
    FrontierContext context{};

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

    // Fraction of the population that may be spawned or removed INCREMENTALLY
    // before the tree is rebuilt. An incremental insert descends to the leaf
    // whose box grows least and either takes a free lane or splits the leaf,
    // which deepens that subtree by one; a removal invalidates a lane and
    // leaves its box loose. Both are O(depth) and both cost a little quality,
    // so this bounds how much of it accumulates.
    float tlasEditFraction = 0.05f;

    // Minimum number of visible instances before an uncached SpatialQuery fans
    // selection out across context.parallelFor. 0 disables parallel
    // selection entirely. Also requires context.workerCount > 1.
    //
    // There is no correctness caveat attached to this: selection reads the
    // SpatialDatabase and writes only per-worker buffers, which are concatenated in
    // instance order, so parallel and serial selection are bit-identical.
    uint32_t parallelInstanceThreshold = 0;
};

class SpatialDatabase
{
public:
    explicit SpatialDatabase(const SpatialDatabaseConfig& config = SpatialDatabaseConfig{});
    ~SpatialDatabase();

    // Non-copyable and non-movable: instances/mounts refer to each other by
    // slot and registered byte arrays retain their allocation contexts.
    SpatialDatabase(const SpatialDatabase&) = delete;
    SpatialDatabase& operator=(const SpatialDatabase&) = delete;
    SpatialDatabase(SpatialDatabase&&) = delete;
    SpatialDatabase& operator=(SpatialDatabase&&) = delete;

    const SpatialDatabaseConfig& config() const { return config_; }

    // ---- reusable definitions ---------------------------------------------
    // Consume one traversal-ready serialized byte array for reuse by any
    // number of mounted placements. Registration moves the allocation without
    // deserializing or copying it. It performs complete structural validation
    // by default; FRONTIER_VALIDATE_SUBTREES=0 retains constant-time format
    // envelope checks for trusted, builder-produced bytes.
    SubtreeHandle registerSubtree(SubtreeBytes&& bytes);
    void          releaseSubtree(SubtreeHandle subtree);
    bool          isSubtree(SubtreeHandle subtree) const;
    size_t        subtreeCount() const { return liveSubtrees_; }

    // ---- database assembly -------------------------------------------------
    // Every instance owns a permanent, always-ready renderable TLAS root.
    // InstanceHandle carries a generation stamp, like NodeHandle: instance slots
    // are recycled, and a ref that outlives its instance (remove + later instantiate
    // reusing the slot) must not act on the newcomer. Stale refs make
    // moveInstance / removeInstance safe no-ops.
    // A persistent cohort whose caller-visible order stays fixed while
    // SpatialDatabase caches the corresponding physical instance order. This
    // is intended for groups that move together every frame: dense instance
    // writes become sequential and nearby TLAS paths retain cache locality.
    // The one-time resolve/sort is amortized until optimize() or another
    // physical layout change automatically invalidates the cache.
    class MotionGroup
    {
    public:
        MotionGroup() = default;
        explicit MotionGroup(std::span<const InstanceHandle> instances);
        void reset(std::span<const InstanceHandle> instances);
        size_t size() const { return instances_.size(); }

    private:
        friend class SpatialDatabase;
        struct Slot
        {
            InstanceId dense;
            uint32_t source;

            bool operator<(const Slot& other) const
            {
                return dense < other.dense ||
                       (dense == other.dense && source < other.source);
            }
        };
        static_assert(sizeof(Slot) == 8, "motion-group slot must stay 8 bytes");

        detail::AppendBuffer<InstanceHandle> instances_;
        detail::AppendBuffer<Slot> physicalOrder_;
        uint32_t layoutVersion_ = 0;
        bool physicalOrderValid_ = false;
    };

    // During initial assembly, additions are accumulated for the first build.
    // Once a TLAS exists, insertion is O(depth) and may allocate a TLAS node
    // when a full leaf splits; it never scans the whole instance population.
    InstanceHandle instantiate(const NodeDesc& root,
                               const InstanceDesc& desc = {});

    void removeInstance(InstanceHandle instance); // no-op if stale
    void moveInstance(InstanceHandle instance, const Transform& transform);

    // positions[i] belongs to the InstanceHandle at i in the MotionGroup.
    // Stale refs are ignored and duplicate refs use the final position.
    void moveInstances(MotionGroup& group,
                       std::span<const float4> positions,
                       float scale = 1.0f);

    // ---- topology streaming -------------------------------------------------
    // A mount-point handle normally comes from a high-error ideal-side
    // FrontierEntry. A stale handle returns an invalid placement, covering the
    // normal race with collection. True contract violations still fail: the
    // parent must be mountable, empty, and contain the transformed child
    // bounds.
    SubtreeInstanceHandle mountSubtree(NodeHandle parent,
                                       SubtreeHandle subtree,
                                       const Transform& transform = {});
    void unmountSubtree(SubtreeInstanceHandle instance);
    bool isMounted(SubtreeInstanceHandle instance) const;
    bool hasMountedSubtree(NodeHandle parent) const;

    // Local-to-top-level-instance transform for the mount containing `node`.
    // The caller composes this with its transform table at entry.instance().
    bool tryGetNodeTransform(NodeHandle node, Transform& outTransform) const;

    // ---- shared definition-node readiness ----------------------------------
    // Readiness means the renderer can dispatch this renderable node. A
    // mounted NodeHandle resolves to one node in a registered definition; its
    // readiness is shared by that node across every placement of the same
    // definition. Equal UserPayload values in other nodes remain independent.
    // TLAS roots are permanently ready. Stale mounted handles are ignored.
    void markNodeReady(NodeHandle node);
    void markNodeUnavailable(NodeHandle node);
    bool isNodeReady(NodeHandle node) const;

    // Resolve immutable application payload data for a live frontier handle.
    // Returns kInvalidPayload for the normal async race with
    // unmount/collection. The reserved value can never be authored.
    UserPayload tryGetPayload(NodeHandle h) const
    {
        return detail::decodePayload(tryGetPayloadWord(h));
    }

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
    // The first deformation of a given (instance, mount) pair takes a private
    // bounds overlay; large subtrees keep wide bounds sparse until edits
    // justify a dense copy. Everything else remains shared.
    // Propagation up the ancestor chain promotes overlays across mount
    // boundaries, so only the placements on the path from the moved node to the
    // instance root are ever privatised.
    //
    // Performance: submission order is preserved. For large batches, group
    // calls by instance and mount when practical so consecutive refits reuse
    // nearby overlay data. Ordering is irrelevant to correctness, and SpatialDatabase
    // intentionally does not sort the queue because sorting can cost more
    // than the saved cache misses.
    // Bounds must also remain finite after every containing mount and instance
    // transform; deferred refit checks that before publishing a parent bound.
    void setNodeBounds(InstanceHandle instance, NodeHandle node,
                       const AABB& localBounds);

    // Force pending bounds refits now (conservative grow-only propagation up
    // the tree, across mount boundaries, and into the top-level BVH). Only
    // needed by callers that read bounds between selections — tools, tests.
    void flushBounds();

    // Bounds of one node as this instance sees them: the overlay if it has
    // one, the shared authored bounds otherwise. Flushes pending moves first.
    AABB nodeBounds(InstanceHandle instance, NodeHandle node);

    // Publish queued changes and prepare a stable read-only selection
    // snapshot. Call once before a group of selections, even when no database
    // contents changed; it also advances the epoch used by collection aging.
    // No SpatialDatabase mutation (including collect) may overlap or occur before every
    // SpatialQuery selection using this snapshot has completed.
    void applyUpdates();

    // Force a full quality TLAS rebuild, compact dead dense slots, and restore
    // physical instance/SpatialQuery-record order to TLAS traversal order. Public
    // InstanceHandles and FrontierEntry::instance() ids remain stable. This is
    // intentionally heavier than applyUpdates(): call it at an occasional
    // synchronization point after disruptive database changes (level transition,
    // teleport, loading screen), not as routine per-frame maintenance. It
    // flushes pending bounds edits but does not advance collection age.
    void optimize();

    // ---- garbage collection ----------------------------------------------------
    // Unmounts cold leaf placements from the LRU tail until the mounted count
    // is at most maxMountedSubtrees. Only mounts untouched for >= minAge
    // epochs and without mounted children are eligible. Overloads taking
    // SpatialQuery consume opt-in mount-usage feedback before collecting.
    CollectResult collect(size_t maxMountedSubtrees, uint32_t minAge);
    CollectResult collect(SpatialQuery& query, size_t maxMountedSubtrees,
                          uint32_t minAge);
    CollectResult collect(std::span<SpatialQuery* const> queries,
                          size_t maxMountedSubtrees, uint32_t minAge);

    // ---- introspection -----------------------------------------------------------
    size_t mountedSubtreeCount() const { return mountedSubtrees_; }
    uint32_t frame() const { return frame_; }   // published update epoch

    // Number of live copy-on-write bounds overlays, and the bytes they hold.
    // Both stay zero for a scene that never calls setNodeBounds.
    size_t overlayCount() const { return liveOverlays_; }
    size_t overlayBytes() const;
    // Retained capacity for mount records, transforms, shared/COW node state,
    // stamps, and mount links. Immutable subtree bytes are excluded.
    size_t subtreeInstanceStateBytes() const;

    struct TestAccess;   // defined by tests; full access to internals

private:
    friend struct TestAccess;
    friend struct QueryScratch;
    friend class SpatialQuery;

    struct NodeRef
    {
        uint32_t slot  = kInvalidIndex;
        uint32_t index = kInvalidIndex;
        bool valid() const { return slot != kInvalidIndex; }
        bool isTlasRoot() const
        {
            return slot == kInvalidIndex && index != kInvalidIndex;
        }
    };
    static_assert(sizeof(NodeRef) == 8, "internal node reference must stay 8 bytes");

    struct SubtreeDefinitionRt
    {
        SubtreeBytes bytes;
        detail::SubtreeView view;
        // The definition-shared node-state block owns readiness as well as
        // intrinsic coverage. Index zero is the implicit parent and is never
        // ready. The block is allocated lazily on the first mount.
        static constexpr uint16_t kNodeReady = 1u << 15;
        uint16_t* sharedNodeState = nullptr;
        uint32_t generation = 0;
        uint32_t readyNodes = 0;
        uint32_t firstMount = kInvalidIndex;
        bool rootLeavesOnly = false;

        bool inUse() const { return !bytes.empty(); }
        bool isNodeReady(uint32_t node) const
        {
            return sharedNodeState &&
                   (sharedNodeState[node] & kNodeReady) != 0;
        }
        void setNodeReady(uint32_t node, bool ready)
        {
            FRONTIER_ASSERT(sharedNodeState != nullptr,
                            "definition node state is not initialized");
            if (ready) sharedNodeState[node] |= kNodeReady;
            else sharedNodeState[node] &= ~kNodeReady;
        }
        bool allNodesReady() const
        {
            return readyNodes + 1 == view.packedNodeCount();
        }
    };
    static_assert(sizeof(SubtreeDefinitionRt) <= 160,
                  "subtree definition runtime state grew");

    // Definition-local coverage state. Placements without mounted descendants
    // point at one shared block; attaching a child takes a private copy on
    // demand. Geometric slabs keep those uncommon private copies allocation
    // free after warm-up.
    struct NodeStatePoolRt
    {
        struct Slab
        {
            std::unique_ptr<uint16_t[]> words;
            uint32_t usedBlocks = 0;
            uint32_t blockCount = 0;
        };

        std::vector<Slab> slabs;
        std::vector<uint16_t*> freeBlocks;
        uint16_t* sharedState = nullptr;
        uint32_t wordsPerBlock = 0;
        uint32_t nextSlabBlocks = 1;

        uint16_t* acquire(uint32_t nodeCount);
        uint16_t* shared(uint32_t nodeCount);
        void release(uint16_t* state);
        size_t bytes() const;
    };

    // Accumulated subtree-local -> top-level-instance-local placement, parallel
    // to mount slots. Root mounts are identity.
    struct MountTransformRt
    {
        static constexpr uint32_t kRootLeavesOnly = 1u << 31;
        static constexpr uint32_t kDefinitionMask = ~kRootLeavesOnly;

        float4 pos = float4::point(0.0f, 0.0f, 0.0f);
        float scale = 1.0f;
        float errClamp = FLT_MAX;
        uint32_t generation = 0;
        uint32_t definitionAndFlags = kDefinitionMask;

        uint32_t definition() const
        {
            return definitionAndFlags & kDefinitionMask;
        }
        bool rootLeavesOnly() const
        {
            return (definitionAndFlags & kRootLeavesOnly) != 0;
        }
    };
    static_assert(sizeof(MountTransformRt) == 32,
                  "hot mount record must stay 32 bytes");

    // The two words queried independently of the rest of a mount. Cached
    // SpatialQuery hits validate the mounted-tree root through contentVersion;
    // descendant mutations propagate there. Handles and overlays validate
    // through generation. Keeping these stamps in a compact parallel stream
    // avoids fetching a 56-byte SubtreeInstanceRt for every visible instance.
    struct MountStamp
    {
        uint32_t contentVersion = 0;
        // Mount generations use 24 bits in NodeHandle. One spare high bit is
        // the live-mount flag, so dependency validation remains two compact
        // loads and never has to touch SubtreeInstanceRt::definition.
        static constexpr uint32_t kInUse = 1u << 31;
        uint32_t generationAndInUse = 0;

        uint32_t generation() const
        {
            return generationAndInUse & NodeHandle::kGenerationMask;
        }
        bool inUse() const { return (generationAndInUse & kInUse) != 0; }
        void setGeneration(uint32_t generation)
        {
            generationAndInUse =
                (generationAndInUse & kInUse) |
                (generation & NodeHandle::kGenerationMask);
        }
        void setInUse(bool inUse)
        {
            if (inUse) generationAndInUse |= kInUse;
            else generationAndInUse &= ~kInUse;
        }
    };
    static_assert(sizeof(MountStamp) == 8, "mount stamp must stay 8 bytes");

    // One mounted placement of a definition, with its cold mutable state.
    struct SubtreeInstanceRt
    {
        uint32_t definition = kInvalidIndex;
        // Effective error ceiling for every node in this subtree: the owning
        // mount point's effective error, or FLT_MAX before mounting.
        //
        // The per-mount scalar is folded into the wide error test. It
        // lets one immutable definition back mounts with different parent error
        // ceilings without rewriting authored node data.
        float                 errClamp = FLT_MAX;
        // The derived covered flag and covered-child count share one word.
        // Childless placements point at the definition's shared state; adding
        // a mounted child switches this pointer to a private COW block.
        static constexpr uint16_t kCovered = 1u << 0;
        static constexpr uint16_t kCoveredChildShift = 1;
        uint16_t* nodeState = nullptr;
        // LRU links need 20 bits each and the handle generation needs 24.
        // Packing all three into one word retains generation beside the mount
        // data used by an actual walk without growing this cold record.
        static constexpr uint32_t kLruBits = NodeHandle::kSlotBits;
        static constexpr uint64_t kLruMask = NodeHandle::kSlotMask;
        static constexpr uint32_t kLruNextShift = kLruBits;
        static constexpr uint32_t kGenerationShift = 2 * kLruBits;
        uint64_t lruLinksAndGeneration =
            uint64_t(NodeHandle::kInvalidSlot) |
            (uint64_t(NodeHandle::kInvalidSlot) << kLruNextShift);
        uint32_t   lastTouched = 0;           // database frame of last walk touch
        uint32_t mountedChildren = 0;
        NodeRef owner;
        // Sparse-pool index; leaf placements never allocate a child array.
        uint32_t mountLinks = kInvalidIndex;
        // Root of this mounted subtree tree. Occupies what was tail padding and
        // lets cached traversal coalesce every descendant to one exact stamp.
        uint32_t rootSlot = kInvalidIndex;
        // Intrusive list of all live placements sharing one definition. This
        // lets a node-readiness transition visit affected placements without
        // a second allocation or stale generation-stamped reference array.
        uint32_t definitionPrev = kInvalidIndex;
        uint32_t definitionNext = kInvalidIndex;

        bool isCovered(uint32_t node) const
        {
            return (nodeState[node] & kCovered) != 0;
        }
        void setCovered(uint32_t node, bool covered)
        {
            if (covered) nodeState[node] |= kCovered;
            else nodeState[node] &= ~kCovered;
        }
        uint16_t coveredChildCount(uint32_t node) const
        {
            return uint16_t((nodeState[node] >> kCoveredChildShift) &
                            detail::kMaxChildren);
        }
        void addCoveredChild(uint32_t node)
        {
            FRONTIER_ASSERT(coveredChildCount(node) < detail::kMaxChildren,
                            "covered-child count overflow");
            nodeState[node] += 1u << kCoveredChildShift;
        }
        void removeCoveredChild(uint32_t node)
        {
            FRONTIER_ASSERT(coveredChildCount(node) != 0,
                            "covered-child count underflow");
            nodeState[node] -= 1u << kCoveredChildShift;
        }
        bool inUse() const { return definition != kInvalidIndex; }
        uint32_t generation() const
        {
            return uint32_t(lruLinksAndGeneration >> kGenerationShift) &
                   NodeHandle::kGenerationMask;
        }
        void setGeneration(uint32_t generation)
        {
            const uint64_t low = lruLinksAndGeneration &
                                 ((uint64_t(1) << kGenerationShift) - 1);
            lruLinksAndGeneration =
                low | (uint64_t(generation & NodeHandle::kGenerationMask) <<
                       kGenerationShift);
        }
        uint32_t lruPrev() const
        {
            const uint32_t slot = uint32_t(lruLinksAndGeneration & kLruMask);
            return slot == NodeHandle::kInvalidSlot ? kInvalidIndex : slot;
        }
        uint32_t lruNext() const
        {
            const uint32_t slot =
                uint32_t((lruLinksAndGeneration >> kLruNextShift) & kLruMask);
            return slot == NodeHandle::kInvalidSlot ? kInvalidIndex : slot;
        }
        void setLruPrev(uint32_t slot)
        {
            const uint64_t encoded =
                slot == kInvalidIndex ? NodeHandle::kInvalidSlot
                                      : (slot & NodeHandle::kSlotMask);
            lruLinksAndGeneration =
                (lruLinksAndGeneration & ~kLruMask) | encoded;
        }
        void setLruNext(uint32_t slot)
        {
            const uint64_t mask = kLruMask << kLruNextShift;
            const uint64_t encoded =
                uint64_t(slot == kInvalidIndex ? NodeHandle::kInvalidSlot
                                               : (slot & NodeHandle::kSlotMask))
                << kLruNextShift;
            lruLinksAndGeneration =
                (lruLinksAndGeneration & ~mask) | encoded;
        }
        uint32_t mountedChildSubtrees() const { return mountedChildren; }
        void addMountedChild()
        {
            FRONTIER_ASSERT(mountedChildren != UINT32_MAX,
                            "mounted-child count overflow");
            ++mountedChildren;
        }
        void removeMountedChild()
        {
            FRONTIER_ASSERT(mountedChildren != 0,
                            "mounted-child count underflow");
            --mountedChildren;
        }
    };
    static_assert(sizeof(SubtreeInstanceRt) == 56,
                  "subtree-instance state must stay 56 bytes");

    struct MountLinksRt
    {
        std::vector<uint32_t> slots;
    };

    // Compact placement-local summary derived from shared definition-node
    // readiness and mounted children. The high bit makes the fully-ready test
    // one load; the remaining bits count recursively incomplete children.
    struct MountReadiness
    {
        static constexpr uint32_t kFullyReady = 1u << 31;
        static constexpr uint32_t kIncompleteMask = ~kFullyReady;

        uint32_t incompleteChildrenAndReady = 0;

        uint32_t incompleteChildren() const
        {
            return incompleteChildrenAndReady & kIncompleteMask;
        }
        bool fullyReady() const
        {
            return (incompleteChildrenAndReady & kFullyReady) != 0;
        }
        void setFullyReady(bool ready)
        {
            if (ready) incompleteChildrenAndReady |= kFullyReady;
            else incompleteChildrenAndReady &= kIncompleteMask;
        }
        void addIncompleteChild()
        {
            FRONTIER_ASSERT(incompleteChildren() != kIncompleteMask,
                            "mount readiness child count overflow");
            ++incompleteChildrenAndReady;
            setFullyReady(false);
        }
        void removeIncompleteChild()
        {
            FRONTIER_ASSERT(incompleteChildren() != 0,
                            "mount readiness child count underflow");
            --incompleteChildrenAndReady;
        }
    };
    static_assert(sizeof(MountReadiness) == 4,
                  "mount readiness summary must stay 4 bytes");

    // Copy-on-write bounds for one (instance, mount) pair: the only part of a
    // subtree a deformed instance stops sharing. Allocated on that instance's
    // first setNodeBounds touching the mount, and on any ancestor mount that
    // the resulting growth propagates into.
    struct Overlay
    {
        static constexpr uint32_t kSparseWideMinBlocks = 64;
        static constexpr uint32_t kSparsePromotionDenominator = 16;

        uint32_t slot = kInvalidIndex;   // mount this shadows
        uint32_t generation = 0;         // that mount's generation when taken
        ScalarAABB rootBounds = AABB::empty();
        // Large-subtree wide bounds begin sparse: a dense block->patch table plus
        // only the modified blocks. Once a sixteenth of the blocks change,
        // `wide` is materialized and the sparse storage is released. Smaller
        // subtrees start dense because setup/lookup cost outweighs savings.
        std::vector<WideBounds> wide;          // dense after promotion
        std::vector<uint32_t>   widePatch;     // block -> patchedWide index
        std::vector<WideBounds> patchedWide;   // sparse modified blocks
        bool inUse() const { return slot != kInvalidIndex; }
        bool sparseWide() const { return !widePatch.empty(); }
    };
#ifdef NDEBUG
    // MSVC's checked Debug iterators intentionally enlarge std::vector.
    static_assert(sizeof(Overlay) == 104,
                  "bounds overlay header must stay 104 bytes");
#endif

    // (mount slot -> overlay index), kept sorted by slot on each instance so
    // the traversal can binary search it. Empty for undeformed instances,
    // which is the case the hot path is tuned for.
    struct OverlayRef
    {
        uint32_t slot;
        uint32_t index;
    };
    static_assert(sizeof(OverlayRef) == 8, "overlay reference must stay 8 bytes");

    struct OverlayList
    {
        std::vector<OverlayRef> refs;   // sorted by mount slot
    };
#ifdef NDEBUG
    static_assert(sizeof(OverlayList) == 24,
                  "cold overlay-list header must stay 24 bytes");
#endif

    // Instance transform and its derived exact world bound are updated
    // together, so selection and TLAS-edit state share one spatially ordered
    // record. Overlay-list headers remain in a cold pool allocated only for
    // deformed instances.
    struct Instance
    {
        static constexpr uint32_t kAlive = 1u << 31;
        static constexpr uint32_t kMountableRoot = 1u << 30;
        static constexpr uint32_t kZeroErrorRoot = 1u << 29;
        static constexpr uint32_t kOverlayListMask = kInvalidInstanceId;

        float4   pos{};
        AABB     worldBox;
        float    scale = 1.0f;
        float    maxErrWorld = 0.0f;
        uint32_t mask = ~0u;
        uint32_t rootSlot = kInvalidIndex;
        uint32_t generation = 0;   // stamps InstanceHandles; bumped per reuse
        uint32_t overlayListAndAlive = kOverlayListMask;
        // TLAS node[24] | lane[3]. Node count is bounded by the same 24-bit
        // population limit as InstanceId.
        uint32_t tlasPlacement = kInvalidInstanceId;
        uint32_t liveIndex = kInvalidIndex;

        bool alive() const { return (overlayListAndAlive & kAlive) != 0; }
        bool hasMountableRoot() const
        {
            return (overlayListAndAlive & kMountableRoot) != 0;
        }
        void setMountableRoot(bool value)
        {
            if (value) overlayListAndAlive |= kMountableRoot;
            else overlayListAndAlive &= ~kMountableRoot;
        }
        bool hasZeroErrorRoot() const
        {
            return (overlayListAndAlive & kZeroErrorRoot) != 0;
        }
        void setZeroErrorRoot(bool value)
        {
            if (value) overlayListAndAlive |= kZeroErrorRoot;
            else overlayListAndAlive &= ~kZeroErrorRoot;
        }
        void setAlive(bool value)
        {
            if (value) overlayListAndAlive |= kAlive;
            else overlayListAndAlive &= ~kAlive;
        }
        bool hasOverlayList() const
        {
            return (overlayListAndAlive & kOverlayListMask) != kOverlayListMask;
        }
        uint32_t overlayList() const
        {
            return overlayListAndAlive & kOverlayListMask;
        }
        void setOverlayList(uint32_t index)
        {
            FRONTIER_ASSERT(index < kOverlayListMask,
                        "SpatialDatabase: exhausted overlay-list index space");
            overlayListAndAlive =
                (overlayListAndAlive &
                 (kAlive | kMountableRoot | kZeroErrorRoot)) |
                index;
        }
        void clearOverlayList()
        {
            overlayListAndAlive =
                (overlayListAndAlive &
                 (kAlive | kMountableRoot | kZeroErrorRoot)) |
                kOverlayListMask;
        }

        uint32_t tlasNode() const { return tlasPlacement & kInstanceIdMask; }
        uint32_t tlasLane() const
        {
            return (tlasPlacement >> 24) & (kWide - 1u);
        }
        void setTlasPlacement(uint32_t node, uint32_t lane)
        {
            tlasPlacement = (node & kInstanceIdMask) |
                            ((lane & (kWide - 1u)) << 24);
        }
        void clearTlasPlacement() { tlasPlacement = kInvalidInstanceId; }
    };
    static_assert(sizeof(Instance) == 80,
                  "coupled instance state must stay 80 bytes");

    // nullptr when the ref is stale (slot recycled) or invalid.
    Instance* resolveInstance(InstanceHandle instance);
    InstanceId denseInstanceId(InstanceHandle instance) const;
    InstanceId publicInstanceId(InstanceId dense) const;
    void moveInstanceDense(InstanceId dense, float4 pos, float scale);
    void refreshMotionGroup(MotionGroup& group) const;
    void reorderInstancesByTlas();

    // Wide top-level BVH node; lanes are children (inner nodes or instances).
    // The default query touches only this stream: two cache lines in BVH4 and
    // four in BVH8. Optional contribution/layer data lives in TlasMeta so it
    // does not inflate the stride of the common frustum-only traversal.
    struct alignas(64) TlasNode
    {
        static constexpr uint32_t kValidLaneMask = (1u << kWide) - 1u;
        WideBounds bounds;
        int32_t    child[kWide];   // >= 0: node index; < 0: instance ~child
        // Low kWide bits mark valid lanes.
        uint32_t   validMask = 0;
        int32_t    parent = -1;

        uint32_t validLanes() const { return validMask & kValidLaneMask; }
        void setLeafLane(uint32_t lane)
        {
            validMask |= 1u << lane;
        }
        void clearLane(uint32_t lane)
        {
            validMask &= ~(1u << lane);
        }
    };
    static_assert(sizeof(TlasNode) == (kWide == 8 ? 256 : 128),
                  "hot SIMD TLAS node has unexpected padding");

    struct TlasMeta
    {
        float8   maxErr{};
        uint32_t laneMask[kWide]{}; // union of subtree instance masks
    };
    static_assert(sizeof(TlasMeta) == (kWide == 8 ? 64 : 32),
                  "cold TLAS metadata has unexpected padding");

    struct TlasItem;

    // A TLAS hit is a 24-bit dense instance id plus its six-bit plane mask.
    struct VisibleItem
    {
        uint32_t packed = 0;

        VisibleItem() = default;
        VisibleItem(InstanceId instance, uint8_t mask)
            : packed((instance & kInstanceIdMask) |
                     (uint32_t(mask & kAllPlanes) << kInstanceIdBits))
        {}
        InstanceId instance() const { return packed & kInstanceIdMask; }
        uint8_t mask() const
        {
            return uint8_t((packed >> kInstanceIdBits) & kAllPlanes);
        }
    };
    static_assert(sizeof(VisibleItem) == 4, "visible TLAS hit must stay 4 bytes");

    // One subtree queued on an instance walk. Dense bounds are resolved when
    // it is pushed; the two possible strides use one state bit instead of a
    // padded word. Sparse overlays use a separately instantiated inner loop.
    struct WorkItem
    {
        const std::byte* wideBase = nullptr;
        // slot[20] | plane mask[6] | current[1] | ideal[1] | packed bounds[1]
        uint32_t      state = 0;
        // Sparse overlay index, or kInvalidIndex. This occupies WorkItem's old
        // tail padding, so sparse support does not grow the traversal stack.
        uint32_t      sparseOverlay = kInvalidIndex;

        WorkItem() = default;
        WorkItem(uint32_t slot, detail::WideBoundsRef bounds, uint8_t current,
                 uint8_t ideal, uint8_t mask,
                 uint32_t sparse = kInvalidIndex)
            : wideBase(bounds.base),
              state((slot & NodeHandle::kSlotMask) |
                    (uint32_t(mask & 0x3fu) << NodeHandle::kSlotBits) |
                    (uint32_t(current != 0) << 26) |
                    (uint32_t(ideal != 0) << 27) |
                    (uint32_t(bounds.stride == sizeof(WideBounds)) << 28)),
              sparseOverlay(sparse)
        {}
        const WideBounds& bounds(uint32_t block) const
        {
            const size_t stride = (state & (1u << 28)) != 0
                                      ? sizeof(WideBounds)
                                      : sizeof(detail::WideBlock);
            return *reinterpret_cast<const WideBounds*>(
                wideBase + size_t(block) * stride);
        }
        uint32_t slot() const { return state & NodeHandle::kSlotMask; }
        uint8_t mask() const
        {
            return uint8_t((state >> NodeHandle::kSlotBits) & 0x3fu);
        }
        bool current() const { return (state & (1u << 26)) != 0; }
        bool ideal() const { return (state & (1u << 27)) != 0; }
    };
    static_assert(sizeof(WorkItem) == 16, "subtree work item must stay 16 bytes");

    // Node visit carried on the walk's explicit DFS stack; err and planes are
    // computed by the parent's wide test; current/ideal identify which walks
    // still contain the node.
    struct NodeItem
    {
        float    err;
        // node[20] | plane mask[6] | current[1] | ideal[1]
        uint32_t state = 0;

        NodeItem() = default;
        NodeItem(uint32_t node, float error, uint8_t planes, uint8_t current,
                 uint8_t ideal)
            : err(error),
              state((node & NodeHandle::kIndexMask) |
                    (uint32_t(planes & 0x3fu) << NodeHandle::kIndexBits) |
                    (uint32_t(current != 0) << 26) |
                    (uint32_t(ideal != 0) << 27))
        {}
        uint32_t node() const { return state & NodeHandle::kIndexMask; }
        uint8_t planes() const
        {
            return uint8_t((state >> NodeHandle::kIndexBits) & 0x3fu);
        }
        bool current() const { return (state & (1u << 26)) != 0; }
        bool ideal() const { return (state & (1u << 27)) != 0; }
    };
    static_assert(sizeof(NodeItem) == 8, "node work item must stay 8 bytes");

    // Everything one instance walk needs. Serial selection uses slot 0; a
    // parallel selection gives each worker its own, then concatenates in
    // instance order so results stay bit-identical either way.
    struct Worker
    {
        struct AncestorCandidate
        {
            static constexpr uint32_t Collapsed = 1u << 31;
            static constexpr uint32_t ValueMask = ~Collapsed;

            FrontierEntry fallback;
            // Before finishAncestorCut(), the low bits hold parent + 1.
            // Afterwards they hold the selected covering candidate + 1.
            // Zero means neither; the high bit records a failed ideal leaf.
            uint32_t state = 0;
        };
        static_assert(sizeof(AncestorCandidate) == 16,
                      "ancestor candidate must stay compact");

        struct AncestorIdealEntry
        {
            static constexpr uint32_t Ready = 1u << 31;
            static constexpr uint32_t CandidateMask = ~Ready;

            FrontierEntry entry;
            uint32_t candidateAndReady = 0;

            uint32_t candidate() const
            {
                return candidateAndReady & CandidateMask;
            }
            bool ready() const { return (candidateAndReady & Ready) != 0; }
        };
        static_assert(sizeof(AncestorIdealEntry) == 16,
                      "ancestor ideal entry must stay compact");

        std::vector<WorkItem> work;
        std::vector<NodeItem> nodeStack;

        // Used only by PreferReadyAncestors. Candidate ids travel beside the
        // unchanged hot work records, so the default traversal does not grow
        // either stack item.
        std::vector<uint32_t> workCandidates;
        std::vector<uint32_t> nodeCandidates;
        std::vector<AncestorCandidate> ancestorCandidates;
        std::vector<AncestorIdealEntry> ancestorIdeal;

        // Backing storage for the parallel path, where each worker collects
        // into its own buffers; the serial path points the sinks straight at
        // the caller's output instead.
        detail::FrontierBuffers frontierBuffer;

        FrontierResultSink result;

        void emit(const FrontierEntry& entry, bool current, bool ideal)
        {
            if (current && ideal)
                result.shared.push(entry);
            else if (current)
                result.currentOnly.push(entry);
            else
                result.idealOnly.push(entry);
        }

        uint32_t addAncestorCandidate(uint32_t parent,
                                      const FrontierEntry& fallback)
        {
            const uint32_t id = uint32_t(ancestorCandidates.size());
            FRONTIER_CHECK(id < AncestorCandidate::ValueMask,
                           "ancestor candidate index overflow");
            FRONTIER_ASSERT(parent == kInvalidIndex || parent < id,
                            "ancestor candidate parent must precede child");
            ancestorCandidates.push_back(AncestorCandidate{
                fallback, parent == kInvalidIndex ? 0u : parent + 1u});
            return id;
        }

        void addAncestorIdeal(const FrontierEntry& entry, uint32_t candidate,
                              bool ready)
        {
            FRONTIER_ASSERT(candidate < ancestorCandidates.size(),
                            "ideal entry has no ready ancestor");
            ancestorIdeal.push_back(AncestorIdealEntry{
                entry, candidate | (ready ? AncestorIdealEntry::Ready : 0u)});
            if (!ready)
                ancestorCandidates[candidate].state |=
                    AncestorCandidate::Collapsed;
        }

        void finishAncestorCut()
        {
            // Parent ids are allocated first. One forward pass therefore
            // propagates the outermost collapsed ancestor to every candidate.
            for (uint32_t i = 0; i < ancestorCandidates.size(); ++i)
            {
                AncestorCandidate& candidate = ancestorCandidates[i];
                const uint32_t parentPlusOne =
                    candidate.state & AncestorCandidate::ValueMask;
                const uint32_t inherited =
                    parentPlusOne == 0
                        ? 0u
                        : ancestorCandidates[parentPlusOne - 1u].state &
                              AncestorCandidate::ValueMask;
                if (inherited != 0)
                    candidate.state = inherited;
                else if ((candidate.state & AncestorCandidate::Collapsed) != 0)
                {
                    candidate.state = i + 1u;
                    result.currentOnly.push(candidate.fallback);
                }
                else
                    candidate.state = 0;
            }

            for (const AncestorIdealEntry& ideal : ancestorIdeal)
            {
                const bool swallowed =
                    ancestorCandidates[ideal.candidate()].state != 0;
                if (ideal.ready() && !swallowed)
                    result.shared.push(ideal.entry);
                else
                    result.idealOnly.push(ideal.entry);
            }
        }

        std::vector<uint32_t> touched;      // tree dependencies or optional mount usage
        bool trackTouches = false;
        bool uniqueTouches = false;         // SpatialQuery dependency list
        bool coalesceMountTreeDependencies = false;
#ifdef FRONTIER_STATS
        SelectionStats stats;
#endif

        // Validity-margin tracking for SpatialQuery. Off for every ordinary
        // selection, and the branch is invariant for a whole call.
        bool  trackMargin = false;
        float margin = FLT_MAX;   // min distance to a decision flip so far
        float maxError = 0.0f;    // max effective error among decided nodes
        float bar = 0.0f;         // params.threshold, for the margin arithmetic
        float barInv = 0.0f;      // reciprocal, so output quantization never divides
    };

    // ---- helpers ----
    const SubtreeInstanceRt* resolve(NodeHandle h) const;
    SubtreeInstanceRt* resolve(NodeHandle h)
    {
        return const_cast<SubtreeInstanceRt*>(
            static_cast<const SpatialDatabase*>(this)->resolve(h));
    }
    InstanceId resolveTlasRoot(NodeHandle h) const;
    detail::PayloadWord tryGetPayloadWord(NodeHandle h) const;

    uint32_t allocSubtree();
    void destroySubtree(uint32_t definition);
    const SubtreeDefinitionRt* resolveSubtree(SubtreeHandle h) const;
    void setDefinitionNodeReadiness(uint32_t definition, uint32_t node,
                                    bool ready);
    void initializeMountCoverage(uint32_t slot);
    void ensurePrivateCoverage(uint32_t slot);
    void releasePrivateCoverage(uint32_t slot);
    void propagateSharedCoverage(uint32_t definition, uint32_t node);
    void propagateSharedRootCoverage(uint32_t slot, bool wasCovered);
    void refreshMountDefinitionReadiness(uint32_t slot);
    static constexpr uint32_t kMountTreeDependency = 1u << 31;
    uint32_t traversalDependency(uint32_t slot) const;
    void recordTraversalDependency(Worker& worker, uint32_t slot) const;
    uint32_t dependencyVersion(uint32_t dependency) const;
    bool dependencyMatches(uint32_t dependency, uint32_t version) const;
    void bumpContentVersion(uint32_t slot);

    const detail::SubtreeView& subtreeView(
        const SubtreeInstanceRt& instance) const
    {
        return subtrees_[instance.definition].view;
    }

    uint32_t allocSlot();
    uint32_t registerMount(uint32_t definition, NodeRef owner);
    const std::vector<uint32_t>* mountedChildSlots(
        const SubtreeInstanceRt& instance) const;
    std::vector<uint32_t>& ensureMountedChildSlots(uint32_t slot);
    uint32_t mountedChildSlot(const SubtreeInstanceRt& instance,
                               uint32_t node) const;
    void releaseMountedChildSlots(SubtreeInstanceRt& instance);
    SubtreeInstanceHandle mountTransformed(NodeHandle parent,
                                           uint32_t definition,
                                           const Transform& transform);
    SubtreeInstanceHandle mountTlasRoot(InstanceId dense,
                                        SubtreeHandle subtree,
                                        const Transform& transform);
    void     unmountSlot(uint32_t slot);
    void     unmountTree(uint32_t rootSlot);
    bool     descendantsCovered(uint32_t slot, uint32_t node) const;
    bool     computeCovered(uint32_t slot, uint32_t node) const;
    void     propagateCoverage(uint32_t slot, uint32_t node);

    void lruUnlink(uint32_t slot);
    void lruPushFront(uint32_t slot);
    void lruTouch(uint32_t slot, uint32_t epoch);
    void consumeMountUsage(SpatialQuery& query);
    void consumeMountUsage(std::span<SpatialQuery* const> queries);
    static FrontierResultSink makeSink(detail::FrontierBuffers& buffers,
                                       bool retainExisting = false)
    {
        if (!retainExisting) buffers.clear();
        FrontierResultSink sink{
            Sink<FrontierEntry>(buffers.shared),
            Sink<FrontierEntry>(buffers.currentOnly),
            Sink<FrontierEntry>(buffers.idealOnly)};
        sink.retainsExisting_ = retainExisting;
        return sink;
    }

    // ---- copy-on-write bounds ----
    // Live overlay for (instance, slot), or nullptr. Stale overlays (the
    // mount was removed and its slot reused) report as absent.
    const Overlay* findOverlay(const Instance& inst, uint32_t slot) const;
    // Same, but takes the copy if it is not there yet. Returns an INDEX, not a
    // reference: taking one overlay can reallocate the pool and invalidate a
    // reference to another, and refit holds two at a time as it crosses a mount
    // boundary.
    uint32_t       ensureOverlay(Instance& inst, uint32_t slot);
    void initOverlay(Overlay& ov, uint32_t slot,
                     const SubtreeInstanceRt& instance);
    WideBounds& mutableWideBounds(Overlay& ov, const detail::SubtreeView& subtree,
                                     uint32_t block);
    void           freeOverlays(Instance& inst);
    // Effective bounds for a walk of `slot` by `inst`.
    AABB effectiveNodeBounds(const Instance& inst, uint32_t slot,
                             const SubtreeInstanceRt& instance,
                             uint32_t node) const;
    AABB nodeBoundsFrom(const Overlay* overlay,
                        const detail::SubtreeView& subtree,
                        uint32_t node) const;
    detail::WideBoundsRef wideBoundsFor(
        const Instance& inst, uint32_t slot,
        const SubtreeInstanceRt& instance, uint32_t* sparseOverlay) const;
    WorkItem       makeWorkItem(uint32_t slot, const Instance& inst,
                                uint8_t current, uint8_t ideal,
                                uint8_t mask) const;
    // Debug-only: is `slot` reachable from this instance's root mount?
    bool mountBelongsTo(const Instance& inst, uint32_t slot) const;

    void applyBoundsChange(InstanceId id, uint32_t slot, uint32_t index, const AABB& box);
    void setOverlayNodeBounds(Overlay& overlay,
                              const detail::SubtreeView& subtree,
                              uint32_t index, const AABB& bounds);

    InstanceHandle addTlasRootInstance(const NodeDesc& root,
                                       const InstanceDesc& desc);

    void markTlasStructuralChange();
    void tlasRebuild(bool reorderInstances);
    int32_t tlasBuildRange(std::vector<uint32_t>& items, int lo, int hi, int32_t parent);
    int  tlasSplit(std::vector<uint32_t>& items, int lo, int hi);
    void tlasQuery(const Camera& view, float minPix,
                   std::vector<VisibleItem>& outVisible,
                   std::vector<TlasItem>& stack) const;
    template<bool UseMask, bool UseMinPix>
    void tlasQueryImpl(const Camera& view, float minPix,
                       std::vector<VisibleItem>& outVisible,
                       std::vector<TlasItem>& stack) const;
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
    AABB tlasNodeExtent(uint32_t node, float& maxErr,
                        uint32_t& laneMask) const;
    void tlasNoteEdit();

    bool visibleDescendantsCovered(uint32_t slot, uint32_t node, uint8_t mask,
                                   const Instance& inst,
                                   const Camera& rootLocal,
                                   Worker* dependencyWorker = nullptr) const;
    Camera mountLocalCamera(const Camera& rootLocal, uint32_t slot,
                            uint8_t mask) const;
    bool mountedTreeFullyReady(uint32_t slot) const;
    void propagateFullReadiness(uint32_t slot, bool wasFullyReady);
    void runTlasFlatInstance(uint32_t instIdx, const Camera& view,
                             Worker& w) const;
    void runZeroErrorTlasFlatInstance(uint32_t instIdx, Worker& w) const;
    void runTlasRootInstance(uint32_t instIdx, const Camera& view,
                             const SelectionParams& params, uint8_t mask,
                             Worker& w) const;
    template<bool FullyReady>
    void runSubtree(const WorkItem& item, const Instance& inst,
                    const Camera& local, const SelectionParams& params,
                    Worker& w) const;
    template<bool FullyReady, bool SparseOverlay>
    void runSubtreeImpl(const WorkItem& item, const Instance& inst,
                        const Camera& local,
                        const SelectionParams& params, Worker& w) const;
    template<bool SparseOverlay>
    void runSubtreeAncestorImpl(const WorkItem& item, const Instance& inst,
                                const Camera& local,
                                const SelectionParams& params,
                                uint32_t ancestorCandidate, Worker& w) const;
    void runSubtreeAncestor(const WorkItem& item, const Instance& inst,
                            const Camera& local,
                            const SelectionParams& params,
                            uint32_t ancestorCandidate, Worker& w) const;
    template<bool FullyReady, bool SparseOverlay, bool TrackAncestor = false>
    void wideVisit(const WorkItem& item, const detail::SubtreeView& subtree,
                   float errClamp,
                   uint32_t gen, InstanceId instance, uint32_t node, uint8_t mask,
                   uint8_t currentKids, uint8_t idealKids,
                   const Camera& local, Worker& w,
                   uint32_t ancestorCandidate = kInvalidIndex) const;
    void emitMountedRootLeavesInside(const WorkItem& item,
                                     const detail::SubtreeView& subtree,
                                     float errClamp,
                                     uint32_t generation,
                                     InstanceId instance, float4 qmn,
                                     float4 qmx, float cameraK,
                                     Worker& w) const;
    void emitMountedLeafBatchInside(const SubtreeInstanceRt& owner,
                                    const detail::SubtreeView& ownerSubtree,
                                    NodeItem first, size_t stackBase,
                                    InstanceId instance,
                                    const Camera& rootLocal,
                                    Worker& w) const;
    void selectFrontierCached(const Camera& camera, const SelectionParams& params,
                         SpatialQuery& query, SpatialQuery* usage,
                         FrontierResultSink& outResult) const;
    void selectFrontierUncached(const Camera& camera, const SelectionParams& params,
                           SpatialQuery& query, SpatialQuery* usage,
                           FrontierResultSink& outResult) const;
    void recordMountUsage(SpatialQuery& query, uint32_t slot) const;

    // ---- state ----
    SpatialDatabaseConfig config_;

    std::vector<SubtreeDefinitionRt> subtrees_;
    std::vector<NodeStatePoolRt> nodeStatePools_;
    std::vector<uint32_t> freeSubtrees_;
    size_t liveSubtrees_ = 0;

    std::vector<SubtreeInstanceRt> slots_;
    std::vector<MountTransformRt> mountTransforms_;
    std::vector<MountStamp> mountStamps_;
    std::vector<MountReadiness> mountReadiness_;
    std::vector<MountLinksRt> mountLinks_;
    std::vector<uint32_t> freeMountLinks_;
    std::vector<uint32_t> freeSlots_;
    std::vector<uint32_t> unmountScratch_;
    size_t mountedSubtrees_ = 0;
    uint32_t              generationCounter_ = 0;

    std::vector<Instance> instances_;
    // Cold root identity stream. Payload exists after the first TLAS root.
    std::vector<detail::PayloadWord> tlasRootPayloads_;
    // Packed TLAS-root marker for exact one-node instances; a cold
    // high bit also records the common zero-error case. Keeping this as a
    // lazily allocated compact stream lets mixed forests bypass the 64-byte
    // Instance and wide-block records for flat objects. Databases that
    // have never contained a flat instance allocate no stream at all.
    std::vector<uint32_t> instanceFlatSlots_;
    size_t                flatInstanceCount_ = 0;
    size_t                tlasZeroErrorFlatInstanceCount_ = 0;
    // Cache hits need only this stamp, not the 80-byte Instance record. It is
    // parallel to instances_ and bumped for transform or deformation changes.
    std::vector<uint32_t> instanceFrontierVersions_;
    std::vector<InstanceId> liveInstances_;
    std::vector<uint32_t> freeInstances_;
    // Public InstanceHandle/FrontierEntry ids remain stable while optimize() or the
    // first non-empty build permutes dense storage into TLAS traversal order.
    std::vector<InstanceId> instanceHandleToDense_;
    std::vector<InstanceId> instanceDenseToHandle_;
    std::vector<InstanceId> freeInstanceHandles_;
    uint32_t                instanceLayoutVersion_ = 0;
    bool                    instanceLayoutSpatialized_ = false;

    std::vector<Overlay>  overlays_;
    std::vector<uint32_t> freeOverlays_;
    std::vector<OverlayList> overlayLists_;
    std::vector<uint32_t> freeOverlayLists_;
    size_t                liveOverlays_ = 0;

    std::vector<TlasNode> tlasNodes_;
    std::vector<TlasMeta> tlasMeta_;
    int32_t               tlasRoot_ = -1;
    bool                  tlasDirty_ = true;
    bool                  tlasQualityBuild_ = true;   // quality tier vs Morton
    uint32_t              tlasEdits_ = 0;           // incremental inserts + removes
    std::vector<int32_t>  tlasFreeNodes_;           // nodes emptied by removal
    uint32_t              tlasLeafCount_ = 0;
    uint32_t              tlasQualityCount_ = 0;   // leaves at last quality build
    float                 tlasBaseArea_ = 0.0f;    // lane area at last quality build
    float                 tlasGrownArea_ = 0.0f;   // area added by grow-only refit

    uint32_t lruHead_ = kInvalidIndex, lruTail_ = kInvalidIndex;
    uint32_t frame_ = 0;
    // Submitted bounds in submission order. The generation stamps make
    // entries self-invalidating: if the subtree was unmounted (or either slot was
    // reused) before the flush, the entry is skipped instead of applying to
    // the wrong subtree or the wrong instance.
    struct PendingMove
    {
        AABB       box;
        NodeHandle node;
        uint32_t   instance;
        uint32_t   instGeneration;
    };
    static_assert(sizeof(PendingMove) == 48, "queued bounds edit must stay 48 bytes");
    std::vector<PendingMove> pendingMoves_;

    // Shared TLAS build scratch. Per-selection scratch lives in SpatialQuery.
    struct TlasItem
    {
        uint32_t packed = 0;

        TlasItem() = default;
        TlasItem(int32_t node, uint8_t mask)
            : packed((uint32_t(node) & kInstanceIdMask) |
                     (uint32_t(mask & kAllPlanes) << kInstanceIdBits))
        {}
        uint32_t node() const { return packed & kInstanceIdMask; }
        uint8_t mask() const { return uint8_t(packed >> kInstanceIdBits); }
    };
    static_assert(sizeof(TlasItem) == 4, "TLAS stack item must stay 4 bytes");

    // A normal {uint64 key, uint32 id} pair has 4 bytes of tail padding. Two
    // word halves retain the full 63-bit key with a 12-byte stride in both
    // radix-sort buffers.
    struct MortonItem
    {
        uint32_t keyLo = 0;
        uint32_t keyHi = 0;
        InstanceId instance = 0;

        MortonItem() = default;
        MortonItem(uint64_t key, InstanceId id)
            : keyLo(uint32_t(key)), keyHi(uint32_t(key >> 32)), instance(id)
        {}
        uint64_t key() const { return uint64_t(keyLo) | (uint64_t(keyHi) << 32); }
        bool operator<(const MortonItem& other) const
        {
            const uint64_t a = key(), b = other.key();
            return a < b || (a == b && instance < other.instance);
        }
    };
    static_assert(sizeof(MortonItem) == 12, "Morton sort item must stay 12 bytes");

    std::vector<MortonItem> tlasKeys_;
    std::vector<MortonItem> tlasKeysTmp_;
    std::vector<int32_t>                       tlasLevelTmp_;
    std::vector<uint32_t>                      tlasItemsTmp_;
};

} // namespace frontier
