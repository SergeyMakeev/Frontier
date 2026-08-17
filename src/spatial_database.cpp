#include "frontier/spatial_database.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <memory>
#include <optional>

namespace frontier {

using detail::MutWideBoundsRef;
using detail::SubtreeView;
using detail::WideBlock;
using detail::WideBoundsRef;
using detail::blockLeafLanes;
using detail::blockZeroErrorLanes;
using detail::blockValidLanes;
using detail::metaIsMountable;

#ifdef FRONTIER_STATS
  #define FRONTIER_STAT(w, field, n) ((w).stats.field += (n))
#else
  #define FRONTIER_STAT(w, field, n) ((void)sizeof(w), (void)0)
#endif

namespace {

inline float axisOf(float4 v, int axis)
{
    return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}

inline float surfaceArea(const AABB& b)
{
    if (b.isEmpty()) return 0.0f;
    const float4 e = b.mx - b.mn;
    return 2.0f * (e.x * e.y + e.y * e.z + e.z * e.x);
}

inline uint32_t nextMountGeneration(uint32_t generation)
{
    generation = (generation + 1u) & NodeHandle::kGenerationMask;
    return generation == 0 ? 1u : generation;
}

inline uint32_t frontierCount(uint32_t counts, uint32_t bucket)
{
    return (counts >> (bucket * 10)) & 0x3ffu;
}

inline uint32_t frontierTotal(uint32_t counts)
{
    return frontierCount(counts, 0) + frontierCount(counts, 1) + frontierCount(counts, 2);
}

inline uint32_t frontierDependencyCount(uint32_t counts)
{
    return counts >> 30;
}

inline bool finitePosition(float4 p)
{
    return std::isfinite(p.x) && std::isfinite(p.y) &&
           std::isfinite(p.z);
}

inline bool representableScale(float scale)
{
    return scale > 0.0f && std::isfinite(scale) &&
           std::isfinite(1.0f / scale);
}

inline bool identityYaw(YawRotation yaw)
{
    return yaw.cosine == 1.0f && yaw.sine == 0.0f;
}

inline bool validYaw(YawRotation yaw)
{
    const float norm2 = yaw.cosine * yaw.cosine + yaw.sine * yaw.sine;
    return std::isfinite(norm2) && std::fabs(norm2 - 1.0f) <= 1.0e-3f;
}

inline float boundsRadiusXZ(const AABB& bounds)
{
    const float x = std::max(std::fabs(bounds.mn.x),
                             std::fabs(bounds.mx.x));
    const float z = std::max(std::fabs(bounds.mn.z),
                             std::fabs(bounds.mx.z));
    return std::hypot(x, z);
}

inline bool yawInvariantBounds(float signedRadius)
{
    return std::signbit(signedRadius);
}

inline bool finiteNonEmptyBounds(const AABB& b)
{
    // Positive ordering rejects empty axes and NaNs. Bounding each extent
    // also rejects infinities and finite endpoints whose subtraction
    // overflows, before either can poison grow-only BVH bounds.
    return b.mn.x <= b.mx.x && b.mn.y <= b.mx.y && b.mn.z <= b.mx.z &&
           b.mx.x - b.mn.x < FLT_MAX &&
           b.mx.y - b.mn.y < FLT_MAX &&
           b.mx.z - b.mn.z < FLT_MAX;
}

inline bool validSelectionCamera(const Camera& camera)
{
    if (!finitePosition(camera.pos) || !(camera.k > 0.0f) ||
        !std::isfinite(camera.k))
        return false;
    if (!(camera.envLo.x >= 0.0f) || !(camera.envLo.y >= 0.0f) ||
        !(camera.envLo.z >= 0.0f) || !(camera.envHi.x >= 0.0f) ||
        !(camera.envHi.y >= 0.0f) || !(camera.envHi.z >= 0.0f) ||
        !finitePosition(camera.envLo) || !finitePosition(camera.envHi))
        return false;
    for (const float4 plane : camera.frustum.plane)
        if (!std::isfinite(plane.x) || !std::isfinite(plane.y) ||
            !std::isfinite(plane.z) || !std::isfinite(plane.w))
            return false;
    return true;
}

inline bool frontierCountsOverflow(uint32_t counts)
{
    return frontierDependencyCount(counts) == 3;
}

inline uint32_t frontierOverflowIndex(uint32_t counts)
{
    return counts & 0x3fffffffu;
}

inline uint32_t packFrontierCounts(uint32_t shared, uint32_t currentOnly,
                              uint32_t idealOnly, uint32_t depCount)
{
    return shared | (currentOnly << 10) | (idealOnly << 20) | (depCount << 30);
}

inline uint8_t encodeFrontierErrorRatio(float ratio, bool above)
{
    if (!std::isfinite(ratio)) return above ? 255 : 127;
    if (!(ratio > 0.0f)) return 0;

    const uint32_t bits = std::bit_cast<uint32_t>(ratio);
    const uint32_t biased = (bits >> 23) & 0xffu;
    if (biased == 0) return above ? 128 : 0;
    const int exponent = int(biased) - 127;
    const int mantissa = int((bits >> 20) & 7u);
    int code = 128 + exponent * 8 + mantissa;
    code = std::clamp(code, 0, 255);
    code = above ? std::max(code, 128) : std::min(code, 127);
    return uint8_t(code);
}

} // namespace

uint8_t encodeFrontierError(float error, float threshold)
{
    if (!(error > 0.0f)) return 0;
    if (!(threshold > 0.0f)) return error > threshold ? 255 : 127;

    const bool above = error > threshold;
    return encodeFrontierErrorRatio(error * (1.0f / threshold), above);
}

float decodeFrontierError(uint8_t code, float threshold)
{
    if (!(threshold > 0.0f)) return threshold;
    const int q = code < kFrontierErrorThreshold ? int(code) - 127
                                             : int(code) - 128;
    return threshold * std::exp2(float(q) * (1.0f / 8.0f));
}

namespace {

inline FrontierEntry makeFrontierEntry(NodeHandle node, float error, float threshold,
                             float thresholdInv, InstanceId instance)
{
    if (!(error > 0.0f)) return FrontierEntry{node, uint8_t(0), instance};
    if (!(threshold > 0.0f))
        return FrontierEntry{node, uint8_t(error > threshold ? 255 : 127), instance};
    return FrontierEntry{node,
                    encodeFrontierErrorRatio(error * thresholdInv, error > threshold),
                    instance};
}

} // namespace

// Opaque per-query state. Cached and uncached selection both use it;
// ownership by SpatialQuery is what makes every query a read-only SpatialDatabase operation.
struct QueryScratch
{
    struct ViewMemo
    {
        Camera camera{};
        SelectionParams params{};
        detail::FrontierBuffers output;
        uint32_t mappingVersion = 0;
        uint32_t contentGeneration = 0;
        uint64_t spatialVersion = 0;
        uint64_t lastUsed = 0;
        uint32_t visibleCount = 0;
        bool valid = false;
    };

    std::vector<SpatialDatabase::Worker>      workers{1};
    std::vector<SpatialDatabase::VisibleItem> visible;
    std::vector<SpatialDatabase::VisibleItem> previousVisible;
    std::vector<SpatialDatabase::TlasItem>    tlasStack;
    detail::FrontierBuffers              output;
    detail::AppendBuffer<UserPayload> resolvedPayloadCurrent;
    detail::AppendBuffer<uint8_t> resolvedErrorCurrent;
    detail::AppendBuffer<RenderFrontierRun> renderRuns;
    ViewMemo viewMemo[2];
    uint64_t memoClock = 0;
    uint32_t lastSceneMappingVersion = 0;
    uint32_t lastSceneContentGeneration = 0;
    uint64_t lastSceneSpatialVersion = 0;
    bool haveLastScene = false;
    bool retainedVisible = false;
    bool retainedAllVisible = false;
    bool segmentedRenderRequested = false;
    bool renderRunsValid = false;
    size_t renderEntryCount = 0;

    size_t bytes() const
    {
        size_t n = visible.capacity() * sizeof(visible[0]) +
                   previousVisible.capacity() * sizeof(previousVisible[0]) +
                   tlasStack.capacity() * sizeof(tlasStack[0]) +
                   workers.capacity() * sizeof(SpatialDatabase::Worker) +
                   output.shared.capacity() * sizeof(FrontierEntry) +
                   output.currentOnly.capacity() * sizeof(FrontierEntry) +
                   output.idealOnly.capacity() * sizeof(FrontierEntry) +
                   resolvedPayloadCurrent.capacity() * sizeof(UserPayload) +
                   resolvedErrorCurrent.capacity() * sizeof(uint8_t) +
                   renderRuns.capacity() * sizeof(RenderFrontierRun);
        for (const ViewMemo& memo : viewMemo)
        {
            n += memo.output.shared.capacity() * sizeof(FrontierEntry);
            n += memo.output.currentOnly.capacity() * sizeof(FrontierEntry);
            n += memo.output.idealOnly.capacity() * sizeof(FrontierEntry);
        }
        for (const SpatialDatabase::Worker& w : workers)
        {
            n += w.work.capacity() * sizeof(SpatialDatabase::WorkItem);
            n += w.nodeStack.capacity() * sizeof(SpatialDatabase::NodeItem);
            n += w.workCandidates.capacity() * sizeof(uint32_t);
            n += w.nodeCandidates.capacity() * sizeof(uint32_t);
            n += w.ancestorCandidates.capacity() *
                 sizeof(SpatialDatabase::Worker::AncestorCandidate);
            n += w.ancestorIdeal.capacity() *
                 sizeof(SpatialDatabase::Worker::AncestorIdealEntry);
            n += w.frontierBuffer.shared.capacity() * sizeof(FrontierEntry);
            n += w.frontierBuffer.currentOnly.capacity() * sizeof(FrontierEntry);
            n += w.frontierBuffer.idealOnly.capacity() * sizeof(FrontierEntry);
            n += w.touched.capacity() * sizeof(uint32_t);
        }
        return n;
    }
};

namespace {

inline bool sameFloat4(float4 a, float4 b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

bool sameMemoCamera(const Camera& a, const Camera& b)
{
    if (!sameFloat4(a.pos, b.pos) || a.k != b.k ||
        a.viewMask != b.viewMask || !sameFloat4(a.envLo, b.envLo) ||
        !sameFloat4(a.envHi, b.envHi))
        return false;
    for (uint32_t plane = 0; plane < 6; ++plane)
        if (!sameFloat4(a.frustum.plane[plane], b.frustum.plane[plane]))
            return false;
    return true;
}

inline bool sameMemoParams(const SelectionParams& a,
                           const SelectionParams& b)
{
    return a.threshold == b.threshold && a.minPix == b.minPix &&
           a.currentCutPolicy == b.currentCutPolicy;
}

} // namespace

SpatialQuery::SpatialQuery()
    : scratch_(std::make_unique<QueryScratch>())
{}

SpatialQuery::SpatialQuery(float halfLifeFrames)
    : damper_(halfLifeFrames), scratch_(std::make_unique<QueryScratch>())
{}

SpatialQuery::~SpatialQuery() = default;
SpatialQuery::SpatialQuery(SpatialQuery&&) noexcept = default;
SpatialQuery& SpatialQuery::operator=(SpatialQuery&&) noexcept = default;

void SpatialQuery::resetMountUsage()
{
    mountUse_.clear();
    dirtyMounts_.clear();
}

void SpatialQuery::setMountUsageEnabled(bool enabled)
{
    if (mountUsageEnabled_ == enabled) return;
    mountUsageEnabled_ = enabled;
    if (!enabled) resetMountUsage();
}

SpatialDatabase::SpatialDatabase(const SpatialDatabaseConfig& config) : config_(config)
{
    if (config_.context.workerCount == 0) config_.context.workerCount = 1;
    FRONTIER_CHECK(config_.tlasQuality == TlasQuality::Morton ||
                       config_.tlasQuality == TlasQuality::Median ||
                       config_.tlasQuality == TlasQuality::BinnedSAH,
                   "SpatialDatabase: invalid TLAS quality");
    FRONTIER_CHECK(
        config_.tlasTraversalCost >= 0.0f &&
            std::isfinite(config_.tlasTraversalCost) &&
            config_.tlasIntersectCost >= 0.0f &&
            std::isfinite(config_.tlasIntersectCost) &&
            config_.tlasCountDrift >= 0.0f &&
            std::isfinite(config_.tlasCountDrift) &&
            config_.tlasAreaDrift >= 0.0f &&
            std::isfinite(config_.tlasAreaDrift) &&
            config_.tlasEditFraction >= 0.0f &&
            std::isfinite(config_.tlasEditFraction),
        "SpatialDatabase: TLAS costs and maintenance thresholds must be "
        "finite and non-negative");
    FRONTIER_CHECK(config_.parallelInstanceThreshold == 0 ||
                       config_.context.workerCount <= 1 ||
                       config_.context.parallelFor != nullptr,
                   "SpatialDatabase: parallel selection requires parallelFor");
}

SpatialDatabase::~SpatialDatabase() = default;

// ============================================================================
// handle resolution — two loads and three compares, no hashing anywhere
// ============================================================================

const SpatialDatabase::SubtreeInstanceRt*
SpatialDatabase::resolve(NodeHandle h) const
{
    if (h.isTlasRoot()) return nullptr;
    const uint32_t slot = h.slot();
    const uint32_t index = h.index();
    if (slot >= slots_.size()) return nullptr;
    const MountStamp& stamp = mountStamps_[slot];
    if (!stamp.inUse() || stamp.generation() != h.generation()) return nullptr;
    const SubtreeInstanceRt& rt = slots_[slot];
    if (index == 0 || index >= subtreeView(rt).packedNodeCount()) return nullptr;
    return &rt;
}

InstanceId SpatialDatabase::resolveTlasRoot(NodeHandle h) const
{
    if (!h.isTlasRoot()) return kInvalidInstanceId;
    const InstanceId publicId = h.tlasInstance();
    if (publicId >= instanceHandleToDense_.size()) return kInvalidInstanceId;
    const InstanceId dense = instanceHandleToDense_[publicId];
    if (dense >= instances_.size()) return kInvalidInstanceId;
    const Instance& inst = instances_[dense];
    if (!inst.alive() ||
        (inst.generation & NodeHandle::kTlasGenerationMask) !=
            h.tlasGeneration())
        return kInvalidInstanceId;
    return dense;
}

inline uint32_t packFrontierOverflow(uint32_t index)
{
    FRONTIER_CHECK(index < (1u << 30),
                   "SpatialQuery: exhausted large-frontier count records");
    return (3u << 30) | index;
}

const SpatialDatabase::SubtreeDefinitionRt*
SpatialDatabase::resolveSubtree(SubtreeHandle h) const
{
    if (h.slot >= subtrees_.size()) return nullptr;
    const SubtreeDefinitionRt& subtree = subtrees_[h.slot];
    return subtree.inUse() && subtree.generation == h.generation
               ? &subtree
               : nullptr;
}

uint32_t SpatialDatabase::traversalDependency(uint32_t slot) const
{
    return kMountTreeDependency | slots_[slot].rootSlot;
}

void SpatialDatabase::recordTraversalDependency(Worker& worker,
                                                 uint32_t slot) const
{
    if (!worker.trackTouches) return;
    const uint32_t dependency = worker.coalesceMountTreeDependencies
                                    ? traversalDependency(slot)
                                    : slot;
    if (!worker.uniqueTouches ||
        std::find(worker.touched.begin(), worker.touched.end(), dependency) ==
            worker.touched.end())
        worker.touched.push_back(dependency);
}

uint32_t SpatialDatabase::dependencyVersion(uint32_t dependency) const
{
    return mountStamps_[dependency & ~kMountTreeDependency].contentVersion;
}

bool SpatialDatabase::dependencyMatches(uint32_t dependency,
                                        uint32_t version) const
{
    const uint32_t slot = dependency & ~kMountTreeDependency;
    return slot < mountStamps_.size() && mountStamps_[slot].inUse() &&
           mountStamps_[slot].contentVersion == version;
}

void SpatialDatabase::bumpContentVersion(uint32_t slot)
{
    ++mountStamps_[slot].contentVersion;
    const uint32_t root = slots_[slot].rootSlot;
    if (root != slot) ++mountStamps_[root].contentVersion;
    const NodeRef owner = slots_[root].owner;
    if (owner.isTlasRoot() && owner.index < instanceFrontierVersions_.size())
        invalidateInstanceFrontier(owner.index);
}

// ============================================================================
// subtree definitions — the unit of sharing
// ============================================================================

uint16_t* SpatialDatabase::NodeStatePoolRt::acquire(uint32_t nodeCount)
{
    FRONTIER_CHECK(nodeCount != 0,
                   "SpatialDatabase: empty node-state allocation");
    if (wordsPerBlock == 0) wordsPerBlock = nodeCount;
    FRONTIER_CHECK(wordsPerBlock == nodeCount,
                   "SpatialDatabase: subtree node count changed");

    if (!freeBlocks.empty())
    {
        uint16_t* state = freeBlocks.back();
        freeBlocks.pop_back();
        std::fill(state, state + wordsPerBlock, uint16_t(0));
        return state;
    }

    if (slabs.empty() ||
        slabs.back().usedBlocks == slabs.back().blockCount)
    {
        constexpr size_t kTargetSlabBytes = 1u << 20;
        const uint32_t maxBlocks = uint32_t(std::max<size_t>(
            1, kTargetSlabBytes /
                   (size_t(wordsPerBlock) * sizeof(uint16_t))));
        const uint32_t blocks = std::min(nextSlabBlocks, maxBlocks);
        Slab slab;
        slab.words = std::make_unique<uint16_t[]>(
            size_t(wordsPerBlock) * blocks);
        slab.blockCount = blocks;
        slabs.push_back(std::move(slab));
        nextSlabBlocks = blocks < maxBlocks
                             ? std::min(blocks * 2u, maxBlocks)
                             : maxBlocks;
    }

    Slab& slab = slabs.back();
    uint16_t* state = slab.words.get() +
                      size_t(slab.usedBlocks++) * wordsPerBlock;
    return state;
}

uint16_t* SpatialDatabase::NodeStatePoolRt::shared(uint32_t nodeCount)
{
    if (!sharedState) sharedState = acquire(nodeCount);
    return sharedState;
}

void SpatialDatabase::NodeStatePoolRt::release(uint16_t* state)
{
    if (state && state != sharedState) freeBlocks.push_back(state);
}

size_t SpatialDatabase::NodeStatePoolRt::bytes() const
{
    size_t result = slabs.capacity() * sizeof(Slab) +
                    freeBlocks.capacity() * sizeof(uint16_t*);
    for (const Slab& slab : slabs)
        result += size_t(slab.blockCount) * wordsPerBlock * sizeof(uint16_t);
    return result;
}

uint32_t SpatialDatabase::allocSubtree()
{
    if (!freeSubtrees_.empty())
    {
        const uint32_t definition = freeSubtrees_.back();
        freeSubtrees_.pop_back();
        return definition;
    }
    subtrees_.emplace_back();
    nodeStatePools_.emplace_back();
    return uint32_t(subtrees_.size() - 1);
}

void SpatialDatabase::destroySubtree(uint32_t definition)
{
    FRONTIER_ASSERT(definition < subtrees_.size(),
                    "SpatialDatabase: invalid subtree definition");
    SubtreeDefinitionRt& subtree = subtrees_[definition];
    FRONTIER_CHECK(subtree.firstMount == kInvalidIndex,
                   "SpatialDatabase::releaseSubtree: live instances remain");
    subtree = SubtreeDefinitionRt{};
    nodeStatePools_[definition] = NodeStatePoolRt{};
    if (fullyRefinedLeafPlans_ &&
        definition < fullyRefinedLeafPlans_->size())
        (*fullyRefinedLeafPlans_)[definition] = FullyRefinedLeafPlan{};
    freeSubtrees_.push_back(definition);
    FRONTIER_ASSERT(liveSubtrees_ != 0,
                    "registered-subtree count underflow");
    --liveSubtrees_;
}

SubtreeHandle SpatialDatabase::registerSubtree(SubtreeBytes&& bytes)
{
    detail::validateSubtreeBytes(bytes);
    const SubtreeView view = detail::viewSubtreeBytes(bytes);

    const uint32_t definition = allocSubtree();
    SubtreeDefinitionRt& runtime = subtrees_[definition];
    runtime = SubtreeDefinitionRt{};
    runtime.bytes = std::move(bytes);
    runtime.view = view;
    runtime.generation = ++generationCounter_;
    bool rootLeavesOnly = true;
    bool terminalLeavesZeroError = true;
    const uint32_t firstBlock = runtime.view.wideOffset(0);
    for (uint32_t b = 0; b < runtime.view.wideBlockCount(0); ++b)
    {
        const uint32_t lanes = runtime.view.blockMask_[firstBlock + b];
        if (detail::blockValidLanes(lanes) != detail::blockLeafLanes(lanes))
        {
            rootLeavesOnly = false;
            break;
        }
    }
    for (uint32_t block = 0; block < runtime.view.wideCount(); ++block)
    {
        const uint32_t lanes = runtime.view.blockMask_[block];
        if ((blockLeafLanes(lanes) & ~blockZeroErrorLanes(lanes)) != 0)
        {
            terminalLeavesZeroError = false;
            break;
        }
    }
    float minInnerError = FLT_MAX;
    uint32_t terminalLeafCount = 0;
    for (uint32_t node = 1; node < runtime.view.packedNodeCount(); ++node)
    {
        if (runtime.view.isMountable(node))
        {
            minInnerError = 0.0f;
            break;
        }
        if (runtime.view.childCount(node) != 0)
            minInnerError = std::min(
                minInnerError, runtime.view.geometricError_[node]);
        else
            ++terminalLeafCount;
    }
    if (minInnerError == FLT_MAX || !terminalLeavesZeroError)
        minInnerError = 0.0f;
    runtime.minInnerErrorAndRootFlag =
        std::copysign(minInnerError, rootLeavesOnly ? -1.0f : 1.0f);

    if (fullyRefinedLeafPlans_ &&
        definition < fullyRefinedLeafPlans_->size())
        (*fullyRefinedLeafPlans_)[definition] = FullyRefinedLeafPlan{};
    if (runtime.view.nodeCount() >= 16 && minInnerError > 0.0f)
    {
        if (!fullyRefinedLeafPlans_)
            fullyRefinedLeafPlans_ =
                std::make_unique<std::vector<FullyRefinedLeafPlan>>();
        fullyRefinedLeafPlans_->resize(subtrees_.size());
        FullyRefinedLeafPlan& plan =
            (*fullyRefinedLeafPlans_)[definition];
        struct BuildItem
        {
            uint32_t node;
            bool exit;
        };

        plan.ranges.resize(runtime.view.packedNodeCount());
        plan.terminalNodes.reserve(terminalLeafCount);
        std::vector<BuildItem> stack;
        stack.reserve(runtime.view.nodeCount());
        stack.push_back({0, false});
        while (!stack.empty())
        {
            const BuildItem item = stack.back();
            stack.pop_back();
            FullyRefinedLeafRange& range = plan.ranges[item.node];
            if (item.exit)
            {
                range.count =
                    uint32_t(plan.terminalNodes.size()) - range.begin;
                continue;
            }

            range.begin = uint32_t(plan.terminalNodes.size());
            stack.push_back({item.node, true});
            const uint32_t first = runtime.view.wideOffset(item.node);
            const uint32_t blocks = runtime.view.wideBlockCount(item.node);
            for (uint32_t b = 0; b < blocks; ++b)
            {
                const uint32_t block = first + b;
                const WideBlock& children = runtime.view.wide_[block];
                const uint32_t lanes = runtime.view.blockMask_[block];
                uint32_t leaves = blockLeafLanes(lanes);
                while (leaves)
                {
                    const uint32_t lane =
                        uint32_t(std::countr_zero(leaves));
                    leaves &= leaves - 1;
                    plan.terminalNodes.push_back(children.child[lane]);
                }
                uint32_t inner = blockValidLanes(lanes) &
                                 ~blockLeafLanes(lanes);
                while (inner)
                {
                    const uint32_t lane =
                        uint32_t(std::countr_zero(inner));
                    inner &= inner - 1;
                    stack.push_back({children.child[lane], false});
                }
            }
        }
    }
    ++liveSubtrees_;
    return SubtreeHandle{definition, runtime.generation};
}

void SpatialDatabase::releaseSubtree(SubtreeHandle subtree)
{
    if (!resolveSubtree(subtree)) return;
    destroySubtree(subtree.slot);
}

bool SpatialDatabase::isSubtree(SubtreeHandle subtree) const
{
    return resolveSubtree(subtree) != nullptr;
}

// ============================================================================
// mount lifecycle
// ============================================================================

uint32_t SpatialDatabase::allocSlot()
{
    if (!freeSlots_.empty())
    {
        uint32_t s = freeSlots_.back();
        freeSlots_.pop_back();
        return s;
    }
    FRONTIER_CHECK(slots_.size() < NodeHandle::kInvalidSlot,
                   "SpatialDatabase: exhausted subtree-instance slots");
    slots_.emplace_back();
    mountTransforms_.emplace_back();
    mountStamps_.emplace_back();
    mountReadiness_.emplace_back();
    return uint32_t(slots_.size() - 1);
}

const std::vector<uint32_t>*
SpatialDatabase::mountedChildSlots(const SubtreeInstanceRt& instance) const
{
    return instance.mountLinks == kInvalidIndex
               ? nullptr
               : &mountLinks_[instance.mountLinks].slots;
}

std::vector<uint32_t>& SpatialDatabase::ensureMountedChildSlots(uint32_t slot)
{
    SubtreeInstanceRt& instance = slots_[slot];
    if (instance.mountLinks == kInvalidIndex)
    {
        uint32_t index;
        if (!freeMountLinks_.empty())
        {
            index = freeMountLinks_.back();
            freeMountLinks_.pop_back();
        }
        else
        {
            mountLinks_.emplace_back();
            index = uint32_t(mountLinks_.size() - 1);
        }
        instance.mountLinks = index;
        mountLinks_[index].slots.assign(
            subtreeView(instance).packedNodeCount(), kInvalidIndex);
    }
    return mountLinks_[instance.mountLinks].slots;
}

uint32_t SpatialDatabase::mountedChildSlot(const SubtreeInstanceRt& instance,
                                             uint32_t node) const
{
    const std::vector<uint32_t>* links = mountedChildSlots(instance);
    return links ? (*links)[node] : kInvalidIndex;
}

void SpatialDatabase::releaseMountedChildSlots(SubtreeInstanceRt& instance)
{
    if (instance.mountLinks == kInvalidIndex) return;
    mountLinks_[instance.mountLinks].slots.clear();
    freeMountLinks_.push_back(instance.mountLinks);
    instance.mountLinks = kInvalidIndex;
}

void SpatialDatabase::initializeMountCoverage(uint32_t slot)
{
    SubtreeInstanceRt& instance = slots_[slot];
    SubtreeDefinitionRt& definition =
        subtrees_[instance.definition];
    instance.nodeState = nodeStatePools_[instance.definition].shared(
        definition.view.packedNodeCount());
    definition.sharedNodeState = instance.nodeState;

    MountReadiness& readiness = mountReadiness_[slot];
    readiness.setFullyReady(definition.allNodesReady());
}

void SpatialDatabase::ensurePrivateCoverage(uint32_t slot)
{
    SubtreeInstanceRt& instance = slots_[slot];
    NodeStatePoolRt& pool = nodeStatePools_[instance.definition];
    if (instance.nodeState != pool.sharedState) return;

    const uint32_t count = subtreeView(instance).packedNodeCount();
    uint16_t* state = pool.acquire(count);
    for (uint32_t node = 0; node < count; ++node)
        state[node] = uint16_t(pool.sharedState[node] &
                               ~SubtreeDefinitionRt::kNodeReady);
    instance.nodeState = state;
}

void SpatialDatabase::releasePrivateCoverage(uint32_t slot)
{
    SubtreeInstanceRt& instance = slots_[slot];
    NodeStatePoolRt& pool = nodeStatePools_[instance.definition];
    if (instance.nodeState == pool.sharedState) return;

    uint16_t* state = instance.nodeState;
    instance.nodeState = pool.sharedState;
    pool.release(state);
}

void SpatialDatabase::refreshMountDefinitionReadiness(uint32_t slot)
{
    MountReadiness& readiness = mountReadiness_[slot];
    const bool definitionReady =
        subtrees_[slots_[slot].definition].allNodesReady();
    readiness.setFullyReady(
        definitionReady && readiness.incompleteChildren() == 0);
}

uint32_t SpatialDatabase::registerMount(uint32_t definition, NodeRef owner)
{
    const uint32_t slot = allocSlot();
    SubtreeDefinitionRt& defined = subtrees_[definition];
    SubtreeInstanceRt& instance = slots_[slot];
    MountStamp& stamp = mountStamps_[slot];
    mountReadiness_[slot] = MountReadiness{};
    mountTransforms_[slot] = MountTransformRt{};

    FRONTIER_CHECK(
        defined.view.packedNodeCount() <= detail::kMaxSubtreeNodes,
        "SpatialDatabase: subtree exceeds node-handle index space");
    instance.definition = definition;
    instance.errClamp = FLT_MAX;
    instance.mountLinks = kInvalidIndex;
    const uint32_t generation = nextMountGeneration(stamp.generation());
    stamp.setGeneration(generation);
    instance.setGeneration(generation);
    ++stamp.contentVersion;
    stamp.setInUse(true);
    instance.lastTouched = frame_;
    instance.mountedChildren = 0;
    instance.owner = owner;
    instance.rootSlot = owner.valid() ? slots_[owner.slot].rootSlot : slot;

    MountTransformRt& hot = mountTransforms_[slot];
    hot.generation = generation;
    hot.definitionAndFlags = definition;
    if (defined.rootLeavesOnly())
        hot.definitionAndFlags |= MountTransformRt::kRootLeavesOnly;
    if (defined.view.nodeCount() >= 16 && defined.minInnerError() > 0.0f)
        hot.definitionAndFlags |= MountTransformRt::kFullyRefinedCandidate;

    initializeMountCoverage(slot);
    instance.definitionPrev = kInvalidIndex;
    instance.definitionNext = defined.firstMount;
    if (defined.firstMount != kInvalidIndex)
        slots_[defined.firstMount].definitionPrev = slot;
    defined.firstMount = slot;
    ++mountedSubtrees_;
    lruPushFront(slot);
    return slot;
}

SubtreeInstanceHandle SpatialDatabase::mountTransformed(
    NodeHandle parentNode, uint32_t definition,
    const Transform& transform)
{
    FRONTIER_CHECK(definition < subtrees_.size() &&
                       subtrees_[definition].inUse(),
                   "SpatialDatabase: invalid subtree definition");
    FRONTIER_CHECK(transform.scale > 0.0f && std::isfinite(transform.scale) &&
                       std::isfinite(transform.pos.x) &&
                       std::isfinite(transform.pos.y) &&
                       std::isfinite(transform.pos.z),
                   "SpatialDatabase: invalid mount transform");

    // Stale parent handle: the parent mount was unmounted/collected while
    // this subtree was being streamed. Normal race: reject quietly.
    if (!resolve(parentNode)) return {};

    const NodeRef owner{parentNode.slot(), parentNode.index()};
    {
        const SubtreeInstanceRt& ownerRt = slots_[owner.slot];
        FRONTIER_CHECK(subtreeView(ownerRt).isMountable(owner.index),
                   "SpatialDatabase::mountSubtree: parent is not mountable");
        FRONTIER_CHECK(mountedChildSlot(ownerRt, owner.index) == kInvalidIndex,
                   "SpatialDatabase::mountSubtree: already mounted");
    }

    // (C) across the boundary: the owner must contain the subtree's content.
    // Growing the owner here is not an option — its bytes back every instance
    // definition. Author mount-point bounds that contain what attaches.
    const AABB childBounds = toWorld(subtrees_[definition].view.bounds(),
                                     transform.pos, transform.scale);
    FRONTIER_CHECK(subtreeView(slots_[owner.slot]).nodeBounds(owner.index)
                       .contains(childBounds),
               "SpatialDatabase: the mounted subtree escapes the mount point's "
               "authored bounds — author conservative bounds at build time");

    // (D) across the boundary: the child subtree's effective error ceiling is
    // the owner mount point's own effective error. Carried as a scalar and
    // folded into the wide test, so immutable bytes are never touched. The
    // same definition can hang under many mount points, each with
    // its own ceiling, without rewriting the definition's node data.
    const float childClamp =
        std::min(subtreeView(slots_[owner.slot]).geometricError_[owner.index],
                 slots_[owner.slot].errClamp) /
        transform.scale;

    const MountTransformRt parentTransform = mountTransforms_[owner.slot];
    const float accumulatedScale =
        parentTransform.scale * transform.scale;
    float4 accumulatedPos =
        parentTransform.pos + transform.pos * parentTransform.scale;
    accumulatedPos.w = 1.0f;
    FRONTIER_CHECK(representableScale(accumulatedScale) &&
                       finitePosition(accumulatedPos),
                   "SpatialDatabase::mountSubtree: accumulated transform "
                   "is not representable");

    // NOTE: registerMount can reallocate slots_, so nothing above may be held
    // as a reference across this call.
    const uint32_t slot = registerMount(definition, owner);
    slots_[slot].errClamp = childClamp;
    MountTransformRt& mounted = mountTransforms_[slot];
    mounted.scale = accumulatedScale;
    mounted.errClamp = childClamp;
    mounted.pos = accumulatedPos;

    ensurePrivateCoverage(owner.slot);
    SubtreeInstanceRt& ort = slots_[owner.slot];
    const bool ownerWasFullyReady = mountedTreeFullyReady(owner.slot);
    ensureMountedChildSlots(owner.slot)[owner.index] = slot;
    ort.addMountedChild();
    if (!mountedTreeFullyReady(slot))
        mountReadiness_[owner.slot].addIncompleteChild();
    refreshMountDefinitionReadiness(owner.slot);
    propagateFullReadiness(owner.slot, ownerWasFullyReady);
    // The child can already be covered because readiness is shared and may
    // have been published before this mount existed.
    propagateCoverage(owner.slot, owner.index);
    bumpContentVersion(owner.slot);   // this node now refines further

    return SubtreeInstanceHandle{slot, mountStamps_[slot].generation()};
}

SubtreeInstanceHandle SpatialDatabase::mountSubtree(
    NodeHandle parentNode, SubtreeHandle subtreeHandle,
    const Transform& transform)
{
    const SubtreeDefinitionRt* child = resolveSubtree(subtreeHandle);
    FRONTIER_CHECK(child != nullptr,
                   "SpatialDatabase::mount: invalid or released subtree");

    const InstanceId root = resolveTlasRoot(parentNode);
    if (root != kInvalidInstanceId)
        return mountTlasRoot(root, subtreeHandle, transform);

    const SubtreeInstanceRt* owner = resolve(parentNode);
    if (!owner) return {};
    return mountTransformed(parentNode, subtreeHandle.slot, transform);
}

SubtreeInstanceHandle SpatialDatabase::mountTlasRoot(
    InstanceId dense, SubtreeHandle subtreeHandle,
    const Transform& transform)
{
    FRONTIER_CHECK(representableScale(transform.scale) &&
                       std::isfinite(transform.pos.x) &&
                       std::isfinite(transform.pos.y) &&
                       std::isfinite(transform.pos.z),
                   "SpatialDatabase::mount: invalid mount transform");
    Instance& inst = instances_[dense];
    FRONTIER_CHECK(inst.hasMountableRoot(),
                   "SpatialDatabase::mountSubtree: TLAS root is not mountable");
    FRONTIER_CHECK(inst.rootSlot == kInvalidIndex,
                   "SpatialDatabase::mountSubtree: already mounted");

    const SubtreeDefinitionRt* child = resolveSubtree(subtreeHandle);
    FRONTIER_CHECK(child != nullptr,
                   "SpatialDatabase::mount: invalid or released subtree");
    const float invInstanceScale = 1.0f / inst.scale;
    const AABB rootLocal = instanceLocalBounds(dense);
    const AABB childBounds = toWorld(child->view.bounds(),
                                     transform.pos, transform.scale);
    FRONTIER_CHECK(rootLocal.contains(childBounds),
                   "SpatialDatabase::mount: mounted subtree escapes the TLAS root's "
                   "authored bounds");

    const float rootError = inst.maxErrWorld * invInstanceScale;
    const float childClamp = rootError / transform.scale;
    const uint32_t slot = registerMount(
        subtreeHandle.slot, NodeRef{kInvalidIndex, dense});
    slots_[slot].errClamp = childClamp;
    MountTransformRt& mounted = mountTransforms_[slot];
    mounted.pos = transform.pos;
    mounted.pos.w = 1.0f;
    mounted.scale = transform.scale;
    mounted.errClamp = childClamp;
    inst.rootSlot = slot;
    invalidateInstanceFrontier(dense);
    return SubtreeInstanceHandle{slot, mountStamps_[slot].generation()};
}

bool SpatialDatabase::tryGetNodeTransform(
    NodeHandle node, Transform& outTransform) const
{
    if (resolveTlasRoot(node) != kInvalidInstanceId)
    {
        outTransform = Transform{};
        return true;
    }
    if (!resolve(node)) return false;
    const MountTransformRt& transform = mountTransforms_[node.slot()];
    outTransform.pos = transform.pos;
    outTransform.scale = transform.scale;
    return true;
}

void SpatialDatabase::unmountSubtree(SubtreeInstanceHandle handle)
{
    if (!isMounted(handle)) return;
    unmountTree(handle.slot);
}

bool SpatialDatabase::isMounted(SubtreeInstanceHandle handle) const
{
    return handle.slot < mountStamps_.size() &&
           mountStamps_[handle.slot].inUse() &&
           mountStamps_[handle.slot].generation() == handle.generation;
}

bool SpatialDatabase::hasMountedSubtree(NodeHandle parentNode) const
{
    const InstanceId root = resolveTlasRoot(parentNode);
    if (root != kInvalidInstanceId)
        return instances_[root].rootSlot != kInvalidIndex;
    const SubtreeInstanceRt* rt = resolve(parentNode);
    return rt && mountedChildSlot(*rt, parentNode.index()) != kInvalidIndex;
}

void SpatialDatabase::unmountSlot(uint32_t slot)
{
    SubtreeInstanceRt& rt = slots_[slot];
    if (rt.owner.valid())
    {
        SubtreeInstanceRt& ownerRt = slots_[rt.owner.slot];
        const bool ownerWasFullyReady = mountedTreeFullyReady(rt.owner.slot);
        if (!mountedTreeFullyReady(slot))
            mountReadiness_[rt.owner.slot].removeIncompleteChild();
        ensureMountedChildSlots(rt.owner.slot)[rt.owner.index] = kInvalidIndex;
        ownerRt.removeMountedChild();
        refreshMountDefinitionReadiness(rt.owner.slot);
        propagateFullReadiness(rt.owner.slot, ownerWasFullyReady);
        bumpContentVersion(rt.owner.slot);   // it collapses to a leaf
        propagateCoverage(rt.owner.slot, rt.owner.index);
        if (ownerRt.mountedChildSubtrees() == 0)
            releasePrivateCoverage(rt.owner.slot);
    }
    else if (rt.owner.isTlasRoot())
    {
        const InstanceId root = rt.owner.index;
        if (root < instances_.size() && instances_[root].alive() &&
            instances_[root].rootSlot == slot)
        {
            instances_[root].rootSlot = kInvalidIndex;
            invalidateInstanceFrontier(root);
        }
    }
    lruUnlink(slot);
    const uint32_t definition = rt.definition;
    const uint32_t generation = rt.generation();
    SubtreeDefinitionRt& defined = subtrees_[definition];
    if (rt.definitionPrev == kInvalidIndex)
        defined.firstMount = rt.definitionNext;
    else
        slots_[rt.definitionPrev].definitionNext = rt.definitionNext;
    if (rt.definitionNext != kInvalidIndex)
        slots_[rt.definitionNext].definitionPrev = rt.definitionPrev;
    releaseMountedChildSlots(rt);
    nodeStatePools_[definition].release(rt.nodeState);
    rt = SubtreeInstanceRt{};
    rt.setGeneration(generation);
    mountStamps_[slot].setInUse(false);
    mountReadiness_[slot] = MountReadiness{};
    mountTransforms_[slot] = MountTransformRt{};
    freeSlots_.push_back(slot);
    --mountedSubtrees_;
}

void SpatialDatabase::unmountTree(uint32_t rootSlot)
{
    if (rootSlot == kInvalidIndex || !slots_[rootSlot].inUse()) return;
    // Collect the mounted tree from its root through mount links, then
    // unmount bottom-up. O(this mounted tree), independent of database size.
    unmountScratch_.clear();
    unmountScratch_.push_back(rootSlot);
    for (size_t k = 0; k < unmountScratch_.size(); ++k)
        if (const std::vector<uint32_t>* links =
                mountedChildSlots(slots_[unmountScratch_[k]]))
            for (const uint32_t child : *links)
                if (child != kInvalidIndex) unmountScratch_.push_back(child);
    for (size_t k = unmountScratch_.size(); k-- > 0;)
        unmountSlot(unmountScratch_[k]);
}

// ============================================================================
// shared definition-node readiness and shared/COW placement coverage
// ============================================================================

bool SpatialDatabase::mountedTreeFullyReady(uint32_t slot) const
{
    return mountReadiness_[slot].fullyReady();
}

void SpatialDatabase::propagateFullReadiness(uint32_t slot,
                                              bool wasFullyReady)
{
    bool fullyReady = mountedTreeFullyReady(slot);
    while (fullyReady != wasFullyReady)
    {
        const NodeRef owner = slots_[slot].owner;
        if (!owner.valid()) return;

        slot = owner.slot;
        const bool ownerWasFullyReady = mountedTreeFullyReady(slot);
        if (fullyReady)
            mountReadiness_[slot].removeIncompleteChild();
        else
            mountReadiness_[slot].addIncompleteChild();
        refreshMountDefinitionReadiness(slot);
        wasFullyReady = ownerWasFullyReady;
        fullyReady = mountedTreeFullyReady(slot);
    }
}

bool SpatialDatabase::descendantsCovered(uint32_t slot, uint32_t node) const
{
    const SubtreeInstanceRt& rt = slots_[slot];
    if (node != 0 && subtreeView(rt).isMountable(node))
    {
        const uint32_t child = mountedChildSlot(rt, node);
        return child != kInvalidIndex && slots_[child].inUse() &&
               slots_[child].isCovered(0);
    }
    const uint32_t count = subtreeView(rt).childCount(node);
    return count != 0 && rt.coveredChildCount(node) == count;
}

bool SpatialDatabase::computeCovered(uint32_t slot, uint32_t node) const
{
    const SubtreeInstanceRt& rt = slots_[slot];
    return (node != 0 &&
            subtrees_[rt.definition].isNodeReady(node)) ||
           descendantsCovered(slot, node);
}

void SpatialDatabase::propagateCoverage(uint32_t slot, uint32_t node)
{
    for (;;)
    {
        SubtreeInstanceRt& rt = slots_[slot];
        const bool was = rt.isCovered(node);
        const bool now = computeCovered(slot, node);
        if (was == now) return;
        rt.setCovered(node, now);

        if (node == 0)
        {
            if (rt.owner.isTlasRoot())
            {
                const InstanceId root = rt.owner.index;
                if (root < instanceFrontierVersions_.size())
                    invalidateInstanceFrontier(root);
                return;
            }
            if (!rt.owner.valid()) return;
            const NodeRef owner = rt.owner;
            bumpContentVersion(owner.slot);
            slot = owner.slot;
            node = owner.index;
            continue;
        }

        const uint32_t parent = subtreeView(rt).parent(node);
        if (now)
            rt.addCoveredChild(parent);
        else
            rt.removeCoveredChild(parent);
        node = parent;
    }
}

void SpatialDatabase::propagateSharedCoverage(uint32_t definitionIndex,
                                              uint32_t node)
{
    const SubtreeDefinitionRt& definition = subtrees_[definitionIndex];
    NodeStatePoolRt& pool = nodeStatePools_[definitionIndex];
    uint16_t* state = pool.sharedState;
    if (!state) return;

    const SubtreeView& subtree = definition.view;
    for (;;)
    {
        const bool was = (state[node] & SubtreeInstanceRt::kCovered) != 0;
        const uint32_t children = subtree.childCount(node);
        const bool now =
            (node != 0 && definition.isNodeReady(node)) ||
            (children != 0 &&
             (uint32_t(state[node] >>
                       SubtreeInstanceRt::kCoveredChildShift) &
              detail::kMaxChildren) ==
                 children);
        if (was == now) return;

        if (now)
            state[node] |= SubtreeInstanceRt::kCovered;
        else
            state[node] &= ~SubtreeInstanceRt::kCovered;
        if (node == 0) return;

        const uint32_t parent = subtree.parent(node);
        FRONTIER_ASSERT(
            now ? ((uint32_t(state[parent]) >>
                    SubtreeInstanceRt::kCoveredChildShift) &
                   detail::kMaxChildren) < detail::kMaxChildren
                : ((uint32_t(state[parent]) >>
                    SubtreeInstanceRt::kCoveredChildShift) &
                   detail::kMaxChildren) != 0,
                        "shared covered-child count overflow or underflow");
        if (now)
            state[parent] +=
                uint16_t(1u << SubtreeInstanceRt::kCoveredChildShift);
        else
            state[parent] -=
                uint16_t(1u << SubtreeInstanceRt::kCoveredChildShift);
        node = parent;
    }
}

void SpatialDatabase::propagateSharedRootCoverage(uint32_t slot,
                                                  bool wasCovered)
{
    const SubtreeInstanceRt& instance = slots_[slot];
    if (instance.isCovered(0) == wasCovered) return;

    if (instance.owner.isTlasRoot())
    {
        const InstanceId root = instance.owner.index;
        if (root < instanceFrontierVersions_.size())
            invalidateInstanceFrontier(root);
        return;
    }
    if (!instance.owner.valid()) return;

    const NodeRef owner = instance.owner;
    bumpContentVersion(owner.slot);
    propagateCoverage(owner.slot, owner.index);
}

void SpatialDatabase::setDefinitionNodeReadiness(
    uint32_t definitionIndex, uint32_t node, bool ready)
{
    SubtreeDefinitionRt& definition = subtrees_[definitionIndex];
    if (definition.isNodeReady(node) == ready) return;

    const uint16_t* sharedState = nodeStatePools_[definitionIndex].sharedState;
    const bool sharedRootWasCovered =
        sharedState &&
        (sharedState[0] & SubtreeInstanceRt::kCovered) != 0;

    definition.setNodeReady(node, ready);
    if (ready)
        ++definition.readyNodes;
    else
    {
        FRONTIER_ASSERT(definition.readyNodes != 0,
                        "node readiness count underflow");
        --definition.readyNodes;
    }

    propagateSharedCoverage(definitionIndex, node);

    // A definition fanout commonly touches hundreds of placements below one
    // mounted-tree root. Each placement keeps its own exact stamp, but the
    // coalesced root dependency only needs one invalidation per consecutive
    // root cohort.
    uint32_t lastRoot = kInvalidIndex;
    for (uint32_t slot = definition.firstMount;
         slot != kInvalidIndex; slot = slots_[slot].definitionNext)
    {
        const bool wasFullyReady = mountedTreeFullyReady(slot);
        if (slots_[slot].nodeState == sharedState)
            propagateSharedRootCoverage(slot, sharedRootWasCovered);
        else
            propagateCoverage(slot, node);

        ++mountStamps_[slot].contentVersion;
        const uint32_t root = slots_[slot].rootSlot;
        if (root != lastRoot)
        {
            if (root != slot) ++mountStamps_[root].contentVersion;
            const NodeRef rootOwner = slots_[root].owner;
            if (rootOwner.isTlasRoot() &&
                rootOwner.index < instanceFrontierVersions_.size())
                invalidateInstanceFrontier(rootOwner.index);
            lastRoot = root;
        }
        refreshMountDefinitionReadiness(slot);
        propagateFullReadiness(slot, wasFullyReady);
    }
}

void SpatialDatabase::markNodeReady(NodeHandle node)
{
    if (resolveTlasRoot(node) != kInvalidInstanceId) return;
    const SubtreeInstanceRt* placement = resolve(node);
    if (!placement) return;
    setDefinitionNodeReadiness(placement->definition, node.index(), true);
}

void SpatialDatabase::markNodeUnavailable(NodeHandle node)
{
    const InstanceId root = resolveTlasRoot(node);
    FRONTIER_CHECK(root == kInvalidInstanceId,
                   "SpatialDatabase: TLAS root must stay ready");
    if (root != kInvalidInstanceId) return;

    const SubtreeInstanceRt* placement = resolve(node);
    if (!placement) return;
    setDefinitionNodeReadiness(placement->definition, node.index(), false);
}

bool SpatialDatabase::isNodeReady(NodeHandle node) const
{
    if (resolveTlasRoot(node) != kInvalidInstanceId) return true;
    const SubtreeInstanceRt* placement = resolve(node);
    return placement &&
           subtrees_[placement->definition].isNodeReady(node.index());
}

detail::PayloadWord SpatialDatabase::tryGetPayloadWord(NodeHandle h) const
{
    const InstanceId root = resolveTlasRoot(h);
    if (root != kInvalidInstanceId)
        return tlasRootPayloads_[root];
    const SubtreeInstanceRt* rt = resolve(h);
    if (!rt) return detail::invalidPayloadWord();
    return subtreeView(*rt).payload_[h.index()];
}

std::span<ResolvedFrontierEntry> SpatialDatabase::resolveFrontier(
    FrontierCutView cut, std::span<ResolvedFrontierEntry> storage) const
{
    const size_t required = cut.size();
    if (storage.size() < required) return {};

    ResolvedFrontierEntry* dst = storage.data();
    uint32_t cachedSlot = NodeHandle::kInvalidSlot;
    uint32_t cachedGeneration = 0;
    uint32_t cachedNodeCount = 0;
    const detail::PayloadWord* cachedPayloads = nullptr;

    for (const FrontierEntry& entry : cut)
    {
        const NodeHandle handle = entry.nodeHandle;
        const uint32_t slot = handle.slot();
        detail::PayloadWord payload = detail::invalidPayloadWord();

        if (slot != NodeHandle::kInvalidSlot)
        {
            const uint32_t generation = handle.generation();
            if (slot != cachedSlot || generation != cachedGeneration)
            {
                cachedSlot = slot;
                cachedGeneration = generation;
                cachedPayloads = nullptr;
                cachedNodeCount = 0;
                if (slot < slots_.size())
                {
                    const MountStamp& stamp = mountStamps_[slot];
                    if (stamp.inUse() && stamp.generation() == generation)
                    {
                        const detail::SubtreeView& view =
                            subtreeView(slots_[slot]);
                        cachedPayloads = view.payload_;
                        cachedNodeCount = view.packedNodeCount();
                    }
                }
            }

            const uint32_t index = handle.index();
            if (cachedPayloads && index != 0 && index < cachedNodeCount)
                payload = cachedPayloads[index];
        }
        else
        {
            // TLAS roots are not naturally grouped by mount slot. Preserve the
            // scalar path's generation validation for these uncommon entries.
            const InstanceId root = resolveTlasRoot(handle);
            if (root != kInvalidInstanceId) payload = tlasRootPayloads_[root];
        }

        dst->payload = detail::decodePayload(payload);
        dst->instanceAndError = entry.instanceAndError;
        ++dst;
    }

    return storage.first(required);
}

bool SpatialDatabase::resolveRenderLeaves(
    FrontierCutView cut, std::span<UserPayload> payloads,
    std::span<uint8_t> errors) const
{
    const size_t required = cut.size();
    if (payloads.size() < required || errors.size() < required) return false;

    UserPayload* payloadDst = payloads.data();
    uint8_t* errorDst = errors.data();
    uint32_t cachedSlot = NodeHandle::kInvalidSlot;
    uint32_t cachedGeneration = 0;
    uint32_t cachedNodeCount = 0;
    const detail::PayloadWord* cachedPayloads = nullptr;

    for (const FrontierEntry& entry : cut)
    {
        const NodeHandle handle = entry.nodeHandle;
        const uint32_t slot = handle.slot();
        detail::PayloadWord payload = detail::invalidPayloadWord();

        if (slot != NodeHandle::kInvalidSlot)
        {
            const uint32_t generation = handle.generation();
            if (slot != cachedSlot || generation != cachedGeneration)
            {
                cachedSlot = slot;
                cachedGeneration = generation;
                cachedPayloads = nullptr;
                cachedNodeCount = 0;
                if (slot < slots_.size())
                {
                    const MountStamp& stamp = mountStamps_[slot];
                    if (stamp.inUse() && stamp.generation() == generation)
                    {
                        const detail::SubtreeView& view =
                            subtreeView(slots_[slot]);
                        cachedPayloads = view.payload_;
                        cachedNodeCount = view.packedNodeCount();
                    }
                }
            }

            const uint32_t index = handle.index();
            if (cachedPayloads && index != 0 && index < cachedNodeCount)
                payload = cachedPayloads[index];
        }
        else
        {
            const InstanceId root = resolveTlasRoot(handle);
            if (root != kInvalidInstanceId) payload = tlasRootPayloads_[root];
        }

        *payloadDst++ = detail::decodePayload(payload);
        *errorDst++ = entry.errorCode();
    }
    return true;
}

// ============================================================================
// instances
// ============================================================================

InstanceHandle SpatialDatabase::addTlasRootInstance(
    const NodeDesc& root, const InstanceDesc& desc)
{
    FRONTIER_CHECK(representableScale(desc.scale) &&
                       finitePosition(desc.pos) && validYaw(desc.yaw),
                   "SpatialDatabase::instantiate: invalid transform");
    FRONTIER_CHECK(root.geometricError >= 0.0f &&
                       std::isfinite(root.geometricError),
                   "SpatialDatabase::instantiate: invalid geometric error");
    const detail::PayloadWord rootPayload = detail::encodePayload(root.payload);
    FRONTIER_CHECK(rootPayload != detail::invalidPayloadWord(),
                   "SpatialDatabase::instantiate: reserved invalid payload");
    constexpr uint32_t kTlasRootFlags =
        NodeDesc::FlagMountable | NodeDesc::FlagYawInvariantBounds;
    FRONTIER_CHECK((root.flags & ~kTlasRootFlags) == 0,
                   "SpatialDatabase::instantiate: unknown node flags");
    const AABB rootBounds = root.bounds;
    FRONTIER_CHECK(finiteNonEmptyBounds(rootBounds),
                   "SpatialDatabase::instantiate: empty or non-finite root bounds");
    const AABB worldBounds =
        root.hasYawInvariantBounds() || identityYaw(desc.yaw)
                                 ? toWorld(rootBounds, desc.pos, desc.scale)
                                 : toWorld(rootBounds, desc.pos, desc.scale,
                                           desc.yaw);
    const float worldError = root.geometricError * desc.scale;
    FRONTIER_CHECK(finiteNonEmptyBounds(worldBounds) &&
                       std::isfinite(worldError),
                   "SpatialDatabase::instantiate: transformed root overflows");

    // A new instance is authored in current world space and therefore cannot
    // join an older population's deferred base-space translation.
    materializeTlasGlobalOffset();

    InstanceId id;
    if (!freeInstances_.empty())
    {
        id = freeInstances_.back();
        freeInstances_.pop_back();
    }
    else
    {
        FRONTIER_CHECK(instances_.size() < kInvalidInstanceId,
                       "SpatialDatabase: exhausted the 24-bit InstanceId space");
        instances_.emplace_back();
        if (!instanceOrientations_.empty())
            instanceOrientations_.emplace_back();
        if (!tlasRootPayloads_.empty()) tlasRootPayloads_.emplace_back();
        instanceFrontierVersions_.emplace_back();
        instanceMotionTravel_.emplace_back();
        instanceTlasLoose_.emplace_back();
        instanceDenseToHandle_.push_back(kInvalidInstanceId);
        id = InstanceId(instances_.size() - 1);
    }

    InstanceId handle;
    if (!freeInstanceHandles_.empty())
    {
        handle = freeInstanceHandles_.back();
        freeInstanceHandles_.pop_back();
    }
    else
    {
        FRONTIER_CHECK(instanceHandleToDense_.size() < kInvalidInstanceId,
                       "SpatialDatabase: exhausted the 24-bit instance-handle space");
        handle = InstanceId(instanceHandleToDense_.size());
        instanceHandleToDense_.push_back(kInvalidInstanceId);
    }
    instanceHandleToDense_[handle] = id;
    instanceDenseToHandle_[id] = handle;
    if ((!identityYaw(desc.yaw) || root.hasYawInvariantBounds()) &&
        instanceOrientations_.empty())
        ensureInstanceOrientations();
    if (tlasRootPayloads_.size() < instances_.size())
        tlasRootPayloads_.resize(instances_.size());
    Instance& inst = instances_[id];
    inst = Instance{};
    tlasRootPayloads_[id] = rootPayload;
    inst.pos = desc.pos;
    inst.scale = desc.scale;
    inst.rootSlot = kInvalidIndex;
    inst.setMountableRoot(root.isMountable());
    inst.setZeroErrorRoot(!(root.geometricError > 0.0f));
    inst.setAlive(true);
    do
        inst.generation = ++generationCounter_;
    while ((inst.generation & NodeHandle::kTlasGenerationMask) == 0 ||
           NodeHandle::tlasRoot(handle, inst.generation).hi == kInvalidIndex);
    inst.mask = desc.mask;
    inst.worldBox = worldBounds;
    inst.maxErrWorld = worldError;
    if (!instanceOrientations_.empty())
    {
        InstanceOrientation& orientation = instanceOrientations_[id];
        orientation.localBounds = rootBounds;
        orientation.yaw = desc.yaw;
        const float radius = boundsRadiusXZ(rootBounds);
        orientation.radiusXZ = root.hasYawInvariantBounds()
                                   ? -radius
                                   : radius;
    }
    invalidateInstanceFrontier(id);
    instanceMotionTravel_[id] = 0.0f;
    instanceTlasLoose_[id] = 0;
    inst.liveIndex = uint32_t(liveInstances_.size());
    liveInstances_.push_back(id);
    if (++instanceMappingVersion_ == 0) ++instanceMappingVersion_;
    if (!instanceFlatSlots_.empty())
        if (instanceFlatSlots_.size() < instances_.size())
            instanceFlatSlots_.resize(instances_.size(), kInvalidIndex);
    if (!root.isMountable())
    {
        if (instanceFlatSlots_.empty())
            instanceFlatSlots_.resize(instances_.size(), kInvalidIndex);
        const NodeHandle rootHandle =
            NodeHandle::tlasRoot(handle, inst.generation);
        instanceFlatSlots_[id] = rootHandle.hi;
        ++flatInstanceCount_;
        if (!(root.geometricError > 0.0f))
            ++tlasZeroErrorFlatInstanceCount_;
    }
    else
    {
        if (!instanceFlatSlots_.empty())
            instanceFlatSlots_[id] = kInvalidIndex;
    }
    tlasInsert(id);
    markTlasStructuralChange();
    return InstanceHandle{handle, inst.generation};
}

InstanceHandle SpatialDatabase::instantiate(
    const NodeDesc& root, const InstanceDesc& desc)
{
    return addTlasRootInstance(root, desc);
}

SpatialDatabase::Instance* SpatialDatabase::resolveInstance(
    InstanceHandle ref)
{
    const InstanceId id = denseInstanceId(ref);
    return id == kInvalidInstanceId ? nullptr : &instances_[id];
}

InstanceId SpatialDatabase::denseInstanceId(InstanceHandle ref) const
{
    if (ref.id >= instanceHandleToDense_.size()) return kInvalidInstanceId;
    const InstanceId id = instanceHandleToDense_[ref.id];
    if (id >= instances_.size()) return kInvalidInstanceId;
    const Instance& inst = instances_[id];
    if (!inst.alive() || inst.generation != ref.generation)
        return kInvalidInstanceId;
    return id;
}

InstanceId SpatialDatabase::publicInstanceId(InstanceId dense) const
{
    FRONTIER_ASSERT(dense < instanceDenseToHandle_.size() &&
                    instanceDenseToHandle_[dense] != kInvalidInstanceId,
                "SpatialDatabase: live dense instance has no public handle");
    return instanceDenseToHandle_[dense];
}

void SpatialDatabase::invalidateInstanceFrontier(InstanceId dense,
                                                 uint32_t generation)
{
    if (generation == 0) generation = ++generationCounter_;
    instanceFrontierVersions_[dense] = generation;
    if (++frontierContentGeneration_ == 0) ++frontierContentGeneration_;
}

SpatialDatabase::MotionGroup::MotionGroup(
    std::span<const InstanceHandle> instances)
{
    reset(instances);
}

void SpatialDatabase::MotionGroup::reset(
    std::span<const InstanceHandle> instances)
{
    instances_.clear();
    instances_.append(instances.data(), instances.size());
    physicalOrder_.clear();
    worldBounds_ = AABB::empty();
    positionBounds_ = AABB::empty();
    maxMotionTravel_ = 0.0f;
    spatialVersion_ = 0;
    mappingVersion_ = 0;
    physicalOrderValid_ = false;
}

// Structural change policy: quality rebuilds are reserved for real population
// drift — database assembly, level load, mass despawn. Under steady churn
// (spawn/despawn at roughly constant population) the instance count barely
// moves, so nothing is forced here at all: the edit is applied incrementally
// and tlasEditFraction decides when the accumulated quality loss is worth a
// rebuild. Only a real change in population size forces one immediately, and
// then it takes the quality tier because it is rare and long-lived.
void SpatialDatabase::markTlasStructuralChange()
{
    const uint64_t alive = liveInstances_.size();
    const uint64_t drift = alive > tlasQualityCount_ ? alive - tlasQualityCount_
                                                     : tlasQualityCount_ - alive;
    if (float(drift) > float(tlasQualityCount_) * config_.tlasCountDrift)
    {
        tlasDirty_ = true;
        tlasQualityBuild_ = true;
    }
}

void SpatialDatabase::removeInstance(InstanceHandle ref)
{
    Instance* inst = resolveInstance(ref);
    if (!inst) return;   // stale ref: the instance is already gone
    const InstanceId id = InstanceId(inst - instances_.data());

    if (!instanceFlatSlots_.empty() &&
        instanceFlatSlots_[id] != kInvalidIndex)
    {
        FRONTIER_ASSERT(flatInstanceCount_ != 0,
                        "flat-instance count underflow");
        if (inst->hasZeroErrorRoot())
        {
            FRONTIER_ASSERT(tlasZeroErrorFlatInstanceCount_ != 0,
                            "zero-error TLAS-flat count underflow");
            --tlasZeroErrorFlatInstanceCount_;
        }
        --flatInstanceCount_;
        instanceFlatSlots_[id] = kInvalidIndex;
    }

    freeOverlays(*inst);
    if (inst->rootSlot != kInvalidIndex)
        unmountTree(inst->rootSlot);
    tlasRemove(id);
    const uint32_t liveIndex = instances_[id].liveIndex;
    const InstanceId moved = liveInstances_.back();
    liveInstances_[liveIndex] = moved;
    instances_[moved].liveIndex = liveIndex;
    liveInstances_.pop_back();
    if (++instanceMappingVersion_ == 0) ++instanceMappingVersion_;
    if (liveInstances_.empty()) instanceLayoutSpatialized_ = false;
    instances_[id].liveIndex = kInvalidIndex;
    instances_[id].setAlive(false);
    if (!instanceOrientations_.empty())
        instanceOrientations_[id] = InstanceOrientation{};
    instanceHandleToDense_[ref.id] = kInvalidInstanceId;
    instanceDenseToHandle_[id] = kInvalidInstanceId;
    freeInstanceHandles_.push_back(ref.id);
    freeInstances_.push_back(id);
    markTlasStructuralChange();

    tlasRootPayloads_[id] = 0;
}

void SpatialDatabase::moveInstance(InstanceHandle ref,
                                   const InstanceTransform& transform)
{
    const InstanceId dense = denseInstanceId(ref);
    if (dense == kInvalidInstanceId) return;
    materializeTlasGlobalOffset();
    uint32_t mutationGeneration = 0;
    float batchMaxTravel = 0.0f;
    moveInstanceDense(dense, transform.pos, transform.scale, transform.yaw,
                      mutationGeneration, batchMaxTravel);
    instanceMotionTravelGlobal_ += batchMaxTravel;
    ++instanceSpatialVersion_;
}

void SpatialDatabase::setInstanceRenderAsUnit(InstanceHandle ref, bool enabled)
{
    const InstanceId dense = denseInstanceId(ref);
    if (dense == kInvalidInstanceId) return;
    Instance& inst = instances_[dense];
    if (inst.renderAsUnit() == enabled) return;
    inst.setRenderAsUnit(enabled);
    invalidateInstanceFrontier(dense);
}

void SpatialDatabase::ensureInstanceOrientations()
{
    if (!instanceOrientations_.empty()) return;
    instanceOrientations_.resize(instances_.size());
    for (const InstanceId dense : liveInstances_)
    {
        const Instance& inst = instances_[dense];
        const float invScale = 1.0f / inst.scale;
        const AABB localBounds = AABB::fromMinMax(
            (inst.worldBox.mn - inst.pos) * invScale,
            (inst.worldBox.mx - inst.pos) * invScale);
        InstanceOrientation& orientation = instanceOrientations_[dense];
        orientation.localBounds = localBounds;
        orientation.yaw = YawRotation{};
        orientation.radiusXZ = boundsRadiusXZ(localBounds);
    }
}

AABB SpatialDatabase::instanceLocalBounds(InstanceId dense) const
{
    if (!instanceOrientations_.empty())
        return instanceOrientations_[dense].localBounds.toAABB();
    const Instance& inst = instances_[dense];
    const float invScale = 1.0f / inst.scale;
    return AABB::fromMinMax((inst.worldBox.mn - inst.pos) * invScale,
                            (inst.worldBox.mx - inst.pos) * invScale);
}

void SpatialDatabase::setInstanceLocalBounds(InstanceId dense,
                                             const AABB& bounds)
{
    if (instanceOrientations_.empty()) return;
    InstanceOrientation& orientation = instanceOrientations_[dense];
    const bool invariant = yawInvariantBounds(orientation.radiusXZ);
    orientation.localBounds = bounds;
    const float radius = boundsRadiusXZ(bounds);
    orientation.radiusXZ = invariant ? -radius : radius;
}

void SpatialDatabase::moveInstanceDense(InstanceId dense, float4 pos, float scale,
                                        YawRotation yaw,
                                        uint32_t& mutationGeneration,
                                        float& batchMaxTravel)
{
    Instance& inst = instances_[dense];
    FRONTIER_CHECK(representableScale(scale) &&
                       finitePosition(pos) && validYaw(yaw),
                   "SpatialDatabase::moveInstance: invalid transform");
    const YawRotation oldYaw = instanceOrientations_.empty()
                                   ? YawRotation{}
                                   : instanceOrientations_[dense].yaw;
    const bool yawChanged = yaw.cosine != oldYaw.cosine ||
                            yaw.sine != oldYaw.sine;
    // Animation systems commonly submit a stable cohort every frame even
    // when many members did not move. Preserve their frontier records and
    // avoid touching the exact TLAS leaf or ancestor chain at all.
    if (!yawChanged && scale == inst.scale && pos.x == inst.pos.x &&
        pos.y == inst.pos.y && pos.z == inst.pos.z)
        return;
    if (!identityYaw(yaw) && instanceOrientations_.empty())
        ensureInstanceOrientations();
    AABB worldBox;
    float maxErrWorld = inst.maxErrWorld;
    float nextTravel = instanceMotionTravel_[dense];
    float translationTravel = 0.0f;
    float rotationTravel = 0.0f;
    const bool invariant = !instanceOrientations_.empty() &&
        yawInvariantBounds(instanceOrientations_[dense].radiusXZ);
    if (scale == inst.scale && (!yawChanged || invariant))
    {
        const float4 delta = pos - inst.pos;
        worldBox = AABB::fromMinMax(inst.worldBox.mn + delta,
                                    inst.worldBox.mx + delta);
        // L1 distance conservatively bounds Euclidean translation while
        // avoiding a square root per moved root. It is exact for the common
        // single-axis animation/streaming shifts.
        translationTravel = std::fabs(delta.x) + std::fabs(delta.y) +
                            std::fabs(delta.z);
        if (yawChanged)
        {
            const float yawChordBound =
                std::fabs(yaw.cosine - oldYaw.cosine) +
                std::fabs(yaw.sine - oldYaw.sine);
            rotationTravel = inst.scale *
                             std::fabs(instanceOrientations_[dense].radiusXZ) *
                             yawChordBound;
        }
        nextTravel += translationTravel + rotationTravel;
    }
    else
    {
        const AABB localBounds = instanceLocalBounds(dense);
        const float invOldScale = 1.0f / inst.scale;
        const float localError = inst.maxErrWorld * invOldScale;
        worldBox = invariant || identityYaw(yaw)
                       ? toWorld(localBounds, pos, scale)
                       : toWorld(localBounds, pos, scale, yaw);
        maxErrWorld = localError * scale;
        if (scale == inst.scale)
        {
            const float4 delta = pos - inst.pos;
            translationTravel = std::fabs(delta.x) + std::fabs(delta.y) +
                                std::fabs(delta.z);
            const float yawChordBound =
                std::fabs(yaw.cosine - oldYaw.cosine) +
                std::fabs(yaw.sine - oldYaw.sine);
            rotationTravel = inst.scale *
                             std::fabs(instanceOrientations_[dense].radiusXZ) *
                             yawChordBound;
            nextTravel += translationTravel + rotationTravel;
        }
    }
    FRONTIER_CHECK(finiteNonEmptyBounds(worldBox) &&
                       std::isfinite(maxErrWorld),
                   "SpatialDatabase::moveInstance: transformed root overflows");
    if (scale != inst.scale || !std::isfinite(nextTravel))
    {
        // Scale changes alter the error field itself. Extremely long-running
        // translation odometers also restart safely by invalidating once.
        instanceMotionTravel_[dense] = 0.0f;
        if (mutationGeneration == 0)
            mutationGeneration = ++generationCounter_;
        invalidateInstanceFrontier(dense, mutationGeneration);
    }
    else
    {
        instanceMotionTravel_[dense] = nextTravel;
        batchMaxTravel = std::max(batchMaxTravel,
                                  translationTravel + rotationTravel);
    }
    inst.pos = pos;
    inst.scale = scale;
    inst.worldBox = worldBox;
    inst.maxErrWorld = maxErrWorld;
    if (!instanceOrientations_.empty())
        instanceOrientations_[dense].yaw = yaw;
    tlasItemsTmp_.push_back(dense);
}

void SpatialDatabase::refreshMotionGroup(MotionGroup& group) const
{
    group.physicalOrder_.clear();
    group.physicalOrder_.reserve(group.instances_.size());
    for (uint32_t source = 0; source < group.instances_.size(); ++source)
    {
        const InstanceId dense = denseInstanceId(group.instances_[source]);
        if (dense != kInvalidInstanceId)
            group.physicalOrder_.push_back({dense, source});
    }
    if (group.physicalOrder_.size() > 1)
        std::sort(group.physicalOrder_.begin(), group.physicalOrder_.end());

    // Duplicate refs retain the last caller position, matching scalar calls.
    size_t out = 0;
    for (size_t i = 0; i < group.physicalOrder_.size();)
    {
        size_t last = i;
        while (last + 1 < group.physicalOrder_.size() &&
               group.physicalOrder_[last + 1].dense ==
                   group.physicalOrder_[i].dense)
            ++last;
        group.physicalOrder_[out++] = group.physicalOrder_[last];
        i = last + 1;
    }
    group.physicalOrder_.resize_uninitialized(out);
    group.mappingVersion_ = instanceMappingVersion_;
    group.physicalOrderValid_ = true;

    group.worldBounds_ = AABB::empty();
    group.positionBounds_ = AABB::empty();
    group.maxMotionTravel_ = 0.0f;
    for (const MotionGroup::Slot slot : group.physicalOrder_)
    {
        const Instance& inst = instances_[slot.dense];
        group.worldBounds_.expand(AABB::fromMinMax(
            inst.worldBox.mn + tlasGlobalOffset_,
            inst.worldBox.mx + tlasGlobalOffset_));
        group.positionBounds_.expand(inst.pos + tlasGlobalOffset_);
        group.maxMotionTravel_ = std::max(
            group.maxMotionTravel_, instanceMotionTravel_[slot.dense]);
    }
    group.spatialVersion_ = instanceSpatialVersion_;
}

void SpatialDatabase::moveInstances(MotionGroup& group,
                          std::span<const float4> positions,
                          float scale)
{
    FRONTIER_CHECK(group.instances_.size() == positions.size(),
               "SpatialDatabase::moveInstances: motion-group/position count mismatch");
    FRONTIER_CHECK(representableScale(scale),
                   "SpatialDatabase::moveInstances: invalid scale");
    if (!group.physicalOrderValid_ ||
        group.mappingVersion_ != instanceMappingVersion_)
        refreshMotionGroup(group);

    // A complete population translated by one common delta can keep the TLAS
    // byte-for-byte unchanged. Validate the whole batch before mutating so
    // contract failures and a differential fallback remain transactional.
    if (!tlasDirty_ && !group.physicalOrder_.empty() &&
        group.physicalOrder_.size() == liveInstances_.size())
    {
        const MotionGroup::Slot first = group.physicalOrder_[0];
        const Instance& firstInst = instances_[first.dense];
        const float4 delta =
            positions[first.source] - (firstInst.pos + tlasGlobalOffset_);
        const bool sameScale = scale == firstInst.scale;
        const float travel = std::fabs(delta.x) + std::fabs(delta.y) +
                             std::fabs(delta.z);
        const float4 nextOffset = tlasGlobalOffset_ + delta;
        bool common = sameScale && finitePosition(positions[first.source]) &&
                      std::isfinite(travel) &&
                      std::isfinite(instanceUniformTravel_ + travel) &&
                      finitePosition(nextOffset);
        for (const MotionGroup::Slot slot : group.physicalOrder_)
        {
            const Instance& inst = instances_[slot.dense];
            const float4 candidateDelta =
                positions[slot.source] - (inst.pos + tlasGlobalOffset_);
            const AABB candidateBox = AABB::fromMinMax(
                inst.worldBox.mn + nextOffset, inst.worldBox.mx + nextOffset);
            common = common && scale == inst.scale &&
                     finitePosition(positions[slot.source]) &&
                     candidateDelta.x == delta.x &&
                     candidateDelta.y == delta.y &&
                     candidateDelta.z == delta.z &&
                     finiteNonEmptyBounds(candidateBox);
        }
        if (common)
        {
            if (travel == 0.0f) return;
            tlasGlobalOffset_ = nextOffset;
            tlasGlobalOffset_.w = 0.0f;
            instanceUniformTravel_ += travel;
            instanceMotionTravelGlobal_ += travel;
            ++instanceSpatialVersion_;
            group.worldBounds_ = AABB::fromMinMax(
                group.worldBounds_.mn + delta, group.worldBounds_.mx + delta);
            group.positionBounds_ = AABB::fromMinMax(
                group.positionBounds_.mn + delta,
                group.positionBounds_.mx + delta);
            group.spatialVersion_ = instanceSpatialVersion_;
            return;
        }
    }

    materializeTlasGlobalOffset();

    // All members belong to one writer mutation batch. They may share a
    // frontier version: cache validity only compares each instance's current
    // stamp with its own recorded stamp. Advancing the database generation
    // once also removes a serialized increment from every translated root.
    uint32_t mutationGeneration = 0;
    float batchMaxTravel = 0.0f;
    for (const MotionGroup::Slot slot : group.physicalOrder_)
    {
        const YawRotation yaw = instanceOrientations_.empty()
                                    ? YawRotation{}
                                    : instanceOrientations_[slot.dense].yaw;
        moveInstanceDense(slot.dense, positions[slot.source], scale, yaw,
                          mutationGeneration, batchMaxTravel);
    }
    instanceMotionTravelGlobal_ += batchMaxTravel;
    if (!group.physicalOrder_.empty()) ++instanceSpatialVersion_;
}

void SpatialDatabase::moveInstances(
    MotionGroup& group, std::span<const InstanceTransform> transforms)
{
    FRONTIER_CHECK(
        group.instances_.size() == transforms.size(),
        "SpatialDatabase::moveInstances: motion-group/transform count mismatch");
    if (!group.physicalOrderValid_ ||
        group.mappingVersion_ != instanceMappingVersion_)
        refreshMotionGroup(group);

    materializeTlasGlobalOffset();
    uint32_t mutationGeneration = 0;
    float batchMaxTravel = 0.0f;
    for (const MotionGroup::Slot slot : group.physicalOrder_)
    {
        const InstanceTransform& transform = transforms[slot.source];
        moveInstanceDense(slot.dense, transform.pos, transform.scale,
                          transform.yaw, mutationGeneration, batchMaxTravel);
    }
    instanceMotionTravelGlobal_ += batchMaxTravel;
    if (!group.physicalOrder_.empty()) ++instanceSpatialVersion_;
}

void SpatialDatabase::translateInstances(MotionGroup& group, float4 delta)
{
    FRONTIER_CHECK(finitePosition(delta),
                   "SpatialDatabase::translateInstances: invalid delta");
    if (!group.physicalOrderValid_ ||
        group.mappingVersion_ != instanceMappingVersion_)
        refreshMotionGroup(group);
    if (group.physicalOrder_.empty()) return;
    if (group.spatialVersion_ != instanceSpatialVersion_)
        refreshMotionGroup(group);

    delta.w = 0.0f;
    const float travel = std::fabs(delta.x) + std::fabs(delta.y) +
                         std::fabs(delta.z);
    FRONTIER_CHECK(std::isfinite(travel),
                   "SpatialDatabase::translateInstances: invalid delta");
    if (travel == 0.0f) return;

    // The MotionGroup mapping is unique and dense-sorted. Equal cardinality
    // therefore proves that it contains every live instance exactly once;
    // the API contract itself proves their common delta. Validate overflow
    // against one aggregate root box and commit only two scalars.
    if (group.physicalOrder_.size() == liveInstances_.size())
    {
        const float4 nextOffset = tlasGlobalOffset_ + delta;
        bool valid = finitePosition(nextOffset) &&
                     std::isfinite(instanceUniformTravel_ + travel) &&
                     finiteNonEmptyBounds(AABB::fromMinMax(
                         group.worldBounds_.mn + delta,
                         group.worldBounds_.mx + delta)) &&
                     finiteNonEmptyBounds(AABB::fromMinMax(
                         group.positionBounds_.mn + delta,
                         group.positionBounds_.mx + delta));
        FRONTIER_CHECK(valid,
                       "SpatialDatabase::translateInstances: transformed "
                       "population overflows");
        tlasGlobalOffset_ = nextOffset;
        tlasGlobalOffset_.w = 0.0f;
        instanceUniformTravel_ += travel;
        instanceMotionTravelGlobal_ += travel;
        ++instanceSpatialVersion_;
        group.worldBounds_ = AABB::fromMinMax(
            group.worldBounds_.mn + delta, group.worldBounds_.mx + delta);
        group.positionBounds_ = AABB::fromMinMax(
            group.positionBounds_.mn + delta,
            group.positionBounds_.mx + delta);
        group.spatialVersion_ = instanceSpatialVersion_;
        return;
    }

    materializeTlasGlobalOffset();

    // The aggregate snapshot proves every destination and odometer before the
    // first instance or TLAS lane changes, retaining transactional failure
    // while removing a redundant validation pass over the cohort.
    FRONTIER_CHECK(
        finiteNonEmptyBounds(AABB::fromMinMax(
            group.worldBounds_.mn + delta, group.worldBounds_.mx + delta)) &&
            finiteNonEmptyBounds(AABB::fromMinMax(
                group.positionBounds_.mn + delta,
                group.positionBounds_.mx + delta)) &&
            std::isfinite(group.maxMotionTravel_ + travel),
        "SpatialDatabase::translateInstances: transformed group overflows");

    for (const MotionGroup::Slot slot : group.physicalOrder_)
    {
        Instance& inst = instances_[slot.dense];
        inst.pos = inst.pos + delta;
        inst.pos.w = 1.0f;
        inst.worldBox = AABB::fromMinMax(inst.worldBox.mn + delta,
                                         inst.worldBox.mx + delta);
        instanceMotionTravel_[slot.dense] += travel;
        tlasOnInstanceMoved(slot.dense);
    }
    instanceMotionTravelGlobal_ += travel;
    ++instanceSpatialVersion_;
    group.worldBounds_ = AABB::fromMinMax(
        group.worldBounds_.mn + delta, group.worldBounds_.mx + delta);
    group.positionBounds_ = AABB::fromMinMax(
        group.positionBounds_.mn + delta,
        group.positionBounds_.mx + delta);
    group.maxMotionTravel_ += travel;
    group.spatialVersion_ = instanceSpatialVersion_;
}

void SpatialDatabase::materializeTlasGlobalOffset()
{
    if (tlasGlobalOffset_.x == 0.0f && tlasGlobalOffset_.y == 0.0f &&
        tlasGlobalOffset_.z == 0.0f)
        return;
    for (const InstanceId dense : liveInstances_)
    {
        Instance& inst = instances_[dense];
        inst.pos = inst.pos + tlasGlobalOffset_;
        inst.pos.w = 1.0f;
        inst.worldBox = AABB::fromMinMax(
            inst.worldBox.mn + tlasGlobalOffset_,
            inst.worldBox.mx + tlasGlobalOffset_);
    }
    for (TlasNode& node : tlasNodes_)
    {
        uint32_t lanes = node.validLanes();
        while (lanes)
        {
            const uint32_t lane = uint32_t(std::countr_zero(lanes));
            lanes &= lanes - 1;
            const AABB box = node.bounds.lane(lane);
            node.bounds.setLane(
                lane, AABB::fromMinMax(box.mn + tlasGlobalOffset_,
                                       box.mx + tlasGlobalOffset_));
        }
    }
    tlasGlobalOffset_ = float4::vec(0.0f, 0.0f, 0.0f);
}

// ============================================================================
// copy-on-write bounds overlays
//
// Bounds are the only authored data the runtime overrides. Giving a
// deformed instance a private copy of just those keeps immutable topology,
// payloads, and errors shared with every placement of the definition. Mount
// readiness coverage and mount links remain ordinary placement state.
// ============================================================================

const SpatialDatabase::Overlay* SpatialDatabase::findOverlay(const Instance& inst, uint32_t slot) const
{
    if (!inst.hasOverlayList()) return nullptr;   // common case, one compare
    const std::vector<OverlayRef>& refs = overlayLists_[inst.overlayList()].refs;
    const auto it = std::lower_bound(
        refs.begin(), refs.end(), slot,
        [](const OverlayRef& r, uint32_t s) { return r.slot < s; });
    if (it == refs.end() || it->slot != slot) return nullptr;
    const Overlay& ov = overlays_[it->index];
    // The subtree may have been unmounted (and its slot reused) since the
    // overlay was taken, in which case it describes a mount that is gone.
    if (!ov.inUse() || ov.generation != mountStamps_[slot].generation()) return nullptr;
    return &ov;
}

void SpatialDatabase::initOverlay(Overlay& ov, uint32_t slot,
                                  const SubtreeInstanceRt& rt)
{
    const SubtreeView& pg = subtreeView(rt);
    ov.generation = mountStamps_[slot].generation();
    ov.rootBounds = pg.bounds();
    if (pg.wideCount() >= Overlay::kSparseWideMinBlocks)
    {
        std::vector<WideBounds>().swap(ov.wide);
        ov.widePatch.assign(pg.wideCount(), kInvalidIndex);
        ov.patchedWide.clear();
    }
    else
    {
        ov.wide.resize(pg.wideCount());
        for (uint32_t b = 0; b < pg.wideCount(); ++b)
            ov.wide[b] = pg.wide_[b].bounds;
        std::vector<uint32_t>().swap(ov.widePatch);
        std::vector<WideBounds>().swap(ov.patchedWide);
    }
}

WideBounds& SpatialDatabase::mutableWideBounds(Overlay& ov,
                                                const SubtreeView& pg,
                                     uint32_t block)
{
    if (!ov.sparseWide()) return ov.wide[block];

    const uint32_t patch = ov.widePatch[block];
    if (patch != kInvalidIndex) return ov.patchedWide[patch];

    // Once edits cover a sixteenth of the blocks, dense storage removes the
    // sparse lookup from future selections and refits.
    if ((ov.patchedWide.size() + 1) *
            Overlay::kSparsePromotionDenominator >
        pg.wideCount())
    {
        ov.wide.resize(pg.wideCount());
        for (uint32_t b = 0; b < pg.wideCount(); ++b)
            ov.wide[b] = pg.wide_[b].bounds;
        for (uint32_t b = 0; b < pg.wideCount(); ++b)
            if (ov.widePatch[b] != kInvalidIndex)
                ov.wide[b] = ov.patchedWide[ov.widePatch[b]];
        std::vector<uint32_t>().swap(ov.widePatch);
        std::vector<WideBounds>().swap(ov.patchedWide);
        return ov.wide[block];
    }

    ov.widePatch[block] = uint32_t(ov.patchedWide.size());
    ov.patchedWide.push_back(pg.wide_[block].bounds);
    return ov.patchedWide.back();
}

uint32_t SpatialDatabase::ensureOverlay(Instance& inst, uint32_t slot)
{
    if (!inst.hasOverlayList())
    {
        uint32_t list;
        if (!freeOverlayLists_.empty())
        {
            list = freeOverlayLists_.back();
            freeOverlayLists_.pop_back();
        }
        else
        {
            FRONTIER_CHECK(overlayLists_.size() < Instance::kOverlayListMask,
                       "SpatialDatabase: exhausted overlay-list index space");
            overlayLists_.emplace_back();
            list = uint32_t(overlayLists_.size() - 1);
        }
        inst.setOverlayList(list);
    }
    std::vector<OverlayRef>& refs = overlayLists_[inst.overlayList()].refs;
    const auto it = std::lower_bound(
        refs.begin(), refs.end(), slot,
        [](const OverlayRef& r, uint32_t s) { return r.slot < s; });

    if (it != refs.end() && it->slot == slot)
    {
        const uint32_t idx = it->index;
        Overlay& ov = overlays_[idx];
        if (!ov.inUse() || ov.generation != mountStamps_[slot].generation())
            initOverlay(ov, slot, slots_[slot]);   // stale: retake from new mount
        return idx;
    }

    uint32_t idx;
    if (!freeOverlays_.empty())
    {
        idx = freeOverlays_.back();
        freeOverlays_.pop_back();
    }
    else
    {
        overlays_.emplace_back();
        idx = uint32_t(overlays_.size() - 1);
    }
    Overlay& ov = overlays_[idx];
    ov.slot = slot;
    initOverlay(ov, slot, slots_[slot]);
    ++liveOverlays_;
    refs.insert(it, OverlayRef{slot, idx});
    return idx;
}

void SpatialDatabase::freeOverlays(Instance& inst)
{
    if (!inst.hasOverlayList()) return;
    std::vector<OverlayRef>& refs = overlayLists_[inst.overlayList()].refs;
    for (const OverlayRef& r : refs)
    {
        Overlay& ov = overlays_[r.index];
        if (!ov.inUse()) continue;
        ov = Overlay{};
        freeOverlays_.push_back(r.index);
        --liveOverlays_;
    }
    refs.clear();
    freeOverlayLists_.push_back(inst.overlayList());
    inst.clearOverlayList();
}

AABB SpatialDatabase::effectiveNodeBounds(
    const Instance& inst, uint32_t slot, const SubtreeInstanceRt& rt,
    uint32_t node) const
{
    const SubtreeView& subtree = subtreeView(rt);
    return nodeBoundsFrom(findOverlay(inst, slot), subtree, node);
}

AABB SpatialDatabase::nodeBoundsFrom(
    const Overlay* overlay, const SubtreeView& subtree,
    uint32_t node) const
{
    if (node == 0)
    {
        return overlay ? overlay->rootBounds.toAABB() : subtree.bounds();
    }

    const uint32_t block = subtree.nodeBlock(node);
    const uint32_t lane = subtree.nodeLane(node);
    if (overlay)
    {
        if (!overlay->sparseWide()) return overlay->wide[block].lane(lane);
        const uint32_t patch = overlay->widePatch[block];
        if (patch != kInvalidIndex)
            return overlay->patchedWide[patch].lane(lane);
    }
    return subtree.wide_[block].bounds.lane(lane);
}

void SpatialDatabase::setOverlayNodeBounds(
    Overlay& overlay, const SubtreeView& subtree, uint32_t index,
    const AABB& bounds)
{
    FRONTIER_ASSERT(index != 0, "implicit-parent bounds use rootBounds");
    mutableWideBounds(overlay, subtree, subtree.nodeBlock(index))
        .setLane(subtree.nodeLane(index), bounds);
}

WideBoundsRef SpatialDatabase::wideBoundsFor(
    const Instance& inst, uint32_t slot, const SubtreeInstanceRt& rt,
    uint32_t* sparseOverlay) const
{
    if (const Overlay* ov = findOverlay(inst, slot))
    {
        if (ov->sparseWide())
        {
            *sparseOverlay = uint32_t(ov - overlays_.data());
            return subtreeView(rt).wideBounds();
        }
        return WideBoundsRef::packed(ov->wide.data());
    }
    return subtreeView(rt).wideBounds();
}

SpatialDatabase::WorkItem SpatialDatabase::makeWorkItem(uint32_t slot, const Instance& inst,
                                    uint8_t current, uint8_t ideal,
                                    uint8_t mask) const
{
    uint32_t sparse = kInvalidIndex;
    const WideBoundsRef wide =
        wideBoundsFor(inst, slot, slots_[slot], &sparse);
    return WorkItem{slot, wide, current, ideal, mask, sparse};
}

Camera SpatialDatabase::mountLocalCamera(const Camera& rootLocal,
                                         uint32_t slot, uint8_t mask) const
{
    const MountTransformRt& transform = mountTransforms_[slot];
    if (transform.scale == 1.0f && transform.pos.x == 0.0f &&
        transform.pos.y == 0.0f && transform.pos.z == 0.0f)
        return rootLocal;

    // A traversal mask only loses planes as it descends.  Transforming the
    // already-dismissed planes at every mount is wasted work, and is
    // particularly expensive for scenes made from many small definitions.
    const float invScale = 1.0f / transform.scale;
    Camera local{};
    local.pos = (rootLocal.pos - transform.pos) * invScale;
    local.k = rootLocal.k;
    local.viewMask = rootLocal.viewMask;
    local.envLo = rootLocal.envLo * invScale;
    local.envHi = rootLocal.envHi * invScale;
    for (uint32_t p = 0; p < 6; ++p)
    {
        if ((mask & (uint8_t(1u) << p)) == 0) continue;
        const float4 plane = rootLocal.frustum.plane[p];
        local.frustum.plane[p] = {
            plane.x, plane.y, plane.z,
            (dot3(plane, transform.pos) + plane.w) * invScale};
    }
    return local;
}

bool SpatialDatabase::mountBelongsTo(const Instance& inst, uint32_t slot) const
{
    return inst.rootSlot != kInvalidIndex && slot < slots_.size() &&
           slots_[slot].rootSlot == inst.rootSlot;
}

size_t SpatialDatabase::overlayBytes() const
{
    size_t n = 0;
    for (const Overlay& ov : overlays_)
        if (ov.inUse())
            n += ov.wide.size() * sizeof(WideBounds) +
                 ov.widePatch.size() * sizeof(uint32_t) +
                 ov.patchedWide.size() * sizeof(WideBounds);
    return n;
}

size_t SpatialDatabase::subtreeInstanceStateBytes() const
{
    size_t bytes = slots_.capacity() * sizeof(SubtreeInstanceRt) +
                   mountTransforms_.capacity() * sizeof(MountTransformRt) +
                   mountStamps_.capacity() * sizeof(MountStamp) +
                   mountReadiness_.capacity() * sizeof(MountReadiness) +
                   freeSlots_.capacity() * sizeof(uint32_t) +
                   unmountScratch_.capacity() * sizeof(uint32_t) +
                   mountLinks_.capacity() * sizeof(MountLinksRt) +
                   freeMountLinks_.capacity() * sizeof(uint32_t);
    bytes += nodeStatePools_.capacity() * sizeof(NodeStatePoolRt);
    for (const NodeStatePoolRt& pool : nodeStatePools_)
        bytes += pool.bytes();
    for (const MountLinksRt& links : mountLinks_)
        bytes += links.slots.capacity() * sizeof(uint32_t);
    return bytes;
}

size_t SpatialDatabase::instanceOrientationStateBytes() const
{
    return instanceOrientations_.capacity() * sizeof(InstanceOrientation);
}

// ============================================================================
// motion: lazy, coalesced, deduplicated conservative grow-only refit
// ============================================================================

void SpatialDatabase::setNodeBounds(InstanceHandle ref, NodeHandle h,
                                    const AABB& localBounds)
{
    // Positive ordering check: rejects empty boxes AND NaN (every NaN
    // comparison is false, so !isEmpty() would let NaN through and poison
    // ancestor boxes forever — grow-only refit never un-grows).
    FRONTIER_CHECK(finiteNonEmptyBounds(localBounds),
                   "SpatialDatabase::setNodeBounds: empty or non-finite bounds");
    if (!h.valid()) return;
    const Instance* inst = resolveInstance(ref);
    if (!inst) return;         // stale instance ref
    const InstanceId root = resolveTlasRoot(h);
    if (root != kInvalidInstanceId)
    {
        FRONTIER_CHECK(root == InstanceId(inst - instances_.data()),
                       "SpatialDatabase::setNodeBounds: root belongs to "
                       "another instance");
        pendingMoves_.push_back({localBounds, h, ref.id, ref.generation});
        return;
    }
    if (!resolve(h)) return;   // stale handle: the subtree was unmounted or collected
    FRONTIER_CHECK(mountBelongsTo(*inst, h.slot()),
                   "SpatialDatabase::setNodeBounds: node is not in this "
                   "instance's mounted tree");
    pendingMoves_.push_back({localBounds, h, ref.id, ref.generation});
}

void SpatialDatabase::applyUpdates()
{
    ++frame_;
    flushBounds();
    // Publish the final instance boxes after all local-bound edits in this
    // mutation batch have been folded into them. A large actor-motion cohort
    // can then rebuild exact TLAS lanes once instead of immediately growing
    // stale pre-deformation bounds and touching the same ancestors again.
    flushInstanceMoves();
    if (tlasDirty_)
    {
        const bool firstSpatialization =
            !instanceLayoutSpatialized_ && !liveInstances_.empty();
        tlasRebuild(firstSpatialization);
    }
}

void SpatialDatabase::optimize()
{
    // The rebuild below consumes the exact Instance records directly, so no
    // intermediate refit of the old topology is necessary.
    tlasItemsTmp_.clear();
    flushBounds();
    tlasDirty_ = true;
    tlasQualityBuild_ = true;
    tlasRebuild(true);
}

void SpatialDatabase::flushBounds()
{
    // Applied in submission order, so the last box per node is the final
    // state (last write wins). There is deliberately NO dedup structure:
    // with grow-only refit, a repeated move of the same node rewrites the
    // same hot wide lane and early-outs at the parent
    // (~hot-cache cost) — while every dedup scheme we measured (per-node
    // stamps, per-mount dirty chains, a transient hash set) paid more in
    // cold cache lines than the walks it skipped. Movers sharing ancestors
    // dedup naturally the same way: the walk stops at the first ancestor
    // that already contains the change, so a shared parent is grown once
    // and merely re-checked by the rest. Stale entries (instance removed,
    // mount removed, slot reused) self-invalidate via generation stamps.
    const bool hadPendingMoves = !pendingMoves_.empty();
    if (hadPendingMoves) materializeTlasGlobalOffset();
    for (const PendingMove& m : pendingMoves_)
    {
        if (m.instance >= instanceHandleToDense_.size()) continue;
        const InstanceId dense = instanceHandleToDense_[m.instance];
        if (dense >= instances_.size()) continue;
        const Instance& inst = instances_[dense];
        if (!inst.alive() || inst.generation != m.instGeneration) continue;
        if (resolveTlasRoot(m.node) == dense)
        {
            const YawRotation yaw = instanceOrientations_.empty()
                                        ? YawRotation{}
                                        : instanceOrientations_[dense].yaw;
            const bool invariant = !instanceOrientations_.empty() &&
                yawInvariantBounds(
                    instanceOrientations_[dense].radiusXZ);
            const AABB worldBox = invariant || identityYaw(yaw)
                                      ? toWorld(m.box, inst.pos, inst.scale)
                                      : toWorld(m.box, inst.pos, inst.scale,
                                                yaw);
            FRONTIER_CHECK(
                finiteNonEmptyBounds(worldBox),
                "SpatialDatabase::flushBounds: transformed root bounds "
                "overflow");
            instances_[dense].worldBox = worldBox;
            setInstanceLocalBounds(dense, m.box);
            invalidateInstanceFrontier(dense);
            tlasOnInstanceMoved(dense);
            continue;
        }
        if (!resolve(m.node)) continue;
        applyBoundsChange(dense, m.node.slot(), m.node.index(), m.box);
    }
    pendingMoves_.clear();
    if (hadPendingMoves) ++instanceSpatialVersion_;
}

void SpatialDatabase::applyBoundsChange(InstanceId id, uint32_t slot, uint32_t index,
                              const AABB& box)
{
    uint32_t curSlot = slot;
    uint32_t cur     = index;
    AABB     curBox  = box;
    bool     exact   = true;   // the submitted node is SET; ancestors only grow

    // A deform privatises bounds into this instance's own overlay, so it can
    // never reach another instance -- one counter on the instance is the whole
    // invalidation. Bumped here rather than deeper because a change that stops
    // early (the ancestor box already contained it) still moved this node.
    invalidateInstanceFrontier(id);

    while (true)
    {
        // Taking the copy is what makes this instance stop sharing bounds for
        // this subtree only. Crossing a boundary below promotes
        // the owner too, so exactly the ancestor path is privatised.
        const uint32_t oi = ensureOverlay(instances_[id], curSlot);
        const SubtreeView& pg = subtreeView(slots_[curSlot]);
        Overlay& overlay = overlays_[oi];
        AABB nodeBox;

        if (exact)
        {
            nodeBox = curBox;
        }
        else
        {
            nodeBox = nodeBoundsFrom(&overlay, pg, cur);
            if (nodeBox.contains(curBox)) return;   // ancestors already conservative
            nodeBox.expand(curBox);
        }
        setOverlayNodeBounds(overlay, pg, cur, nodeBox);

        while (cur != 0)
        {
            const uint32_t p = pg.parent(cur);
            if (p == 0)
            {
                AABB rootBounds = overlay.rootBounds.toAABB();
                if (rootBounds.contains(nodeBox)) return;
                rootBounds.expand(nodeBox);
                overlay.rootBounds = rootBounds;
                cur = 0;
                break;
            }

            AABB parentBox = nodeBoundsFrom(&overlay, pg, p);
            if (parentBox.contains(nodeBox)) return;
            parentBox.expand(nodeBox);             // grow immediately, shrink lazily
            setOverlayNodeBounds(overlay, pg, p, parentBox);
            nodeBox = parentBox;
            cur = p;
        }

        // The subtree outgrew its implicit parent: cross the mount boundary.
        const NodeRef owner = slots_[curSlot].owner;
        if (!owner.valid())
        {
            FRONTIER_ASSERT(owner.isTlasRoot() && owner.index == id,
                            "root mount lost its TLAS owner");
            Instance& root = instances_[id];
            AABB rootLocal = instanceLocalBounds(id);
            const MountTransformRt& transform = mountTransforms_[curSlot];
            const AABB mountedRoot = toWorld(
                overlay.rootBounds.toAABB(), transform.pos, transform.scale);
            FRONTIER_CHECK(
                finiteNonEmptyBounds(mountedRoot),
                "SpatialDatabase::flushBounds: bounds overflow at root "
                "mount boundary");
            rootLocal.expand(mountedRoot);
            const YawRotation yaw = instanceOrientations_.empty()
                                        ? YawRotation{}
                                        : instanceOrientations_[id].yaw;
            const bool invariant = !instanceOrientations_.empty() &&
                yawInvariantBounds(instanceOrientations_[id].radiusXZ);
            const AABB worldBox = invariant || identityYaw(yaw)
                                      ? toWorld(rootLocal, root.pos,
                                                root.scale)
                                      : toWorld(rootLocal, root.pos,
                                                root.scale, yaw);
            FRONTIER_CHECK(
                finiteNonEmptyBounds(worldBox),
                "SpatialDatabase::flushBounds: transformed instance bounds "
                "overflow");
            root.worldBox = worldBox;
            setInstanceLocalBounds(id, rootLocal);
            tlasOnInstanceMoved(id);
            return;
        }
        const MountTransformRt mountedTransform = mountTransforms_[curSlot];
        const MountTransformRt parentTransform = mountTransforms_[owner.slot];
        const float relativeScale = mountedTransform.scale / parentTransform.scale;
        float4 relativePos =
            (mountedTransform.pos - parentTransform.pos) / parentTransform.scale;
        relativePos.w = 1.0f;
        curBox = toWorld(overlay.rootBounds.toAABB(), relativePos, relativeScale);
        FRONTIER_CHECK(
            finiteNonEmptyBounds(curBox),
            "SpatialDatabase::flushBounds: bounds overflow at mount "
            "boundary");
        curSlot = owner.slot;
        cur     = owner.index;
        exact   = false;
    }
}

AABB SpatialDatabase::nodeBounds(InstanceHandle ref, NodeHandle h)
{
    flushBounds();
    Instance* inst = resolveInstance(ref);
    const InstanceId root = resolveTlasRoot(h);
    if (inst && root == InstanceId(inst - instances_.data()))
        return instanceLocalBounds(root);
    const SubtreeInstanceRt* rt = resolve(h);
    if (!inst || !rt) return AABB::empty();
    FRONTIER_CHECK(mountBelongsTo(*inst, h.slot()),
                   "SpatialDatabase::nodeBounds: node is not in this "
                   "instance's mounted tree");
    return effectiveNodeBounds(*inst, h.slot(), *rt, h.index());
}

// ============================================================================
// top-level BVH
// ============================================================================

void SpatialDatabase::tlasNoteGrowth(float addedArea)
{
    tlasGrownArea_ += addedArea;
    // Cost drift: a population that stays constant while everything moves
    // never trips the count-drift trigger, but grow-only refit bloats the
    // lanes all the same. Watching the added area catches exactly that case.
    if (tlasBaseArea_ > 0.0f && tlasGrownArea_ > tlasBaseArea_ * config_.tlasAreaDrift)
    {
        tlasDirty_ = true;
        tlasQualityBuild_ = true;
    }
}

// Grow-only propagation up the parent chain, shared by motion refit and by
// incremental insertion. Stops at the first ancestor that already covers the
// box, its error and its layer mask -- which is what keeps a small move O(1)
// rather than O(depth).
//
// The layer mask matters here and does not for a pure move: an ancestor's
// laneMask must be a superset of its subtree's instance masks, or tlasQuery's
// layer filter will cull a visible instance. A move cannot change a mask, so
// that term is always already satisfied on the motion path.
float SpatialDatabase::tlasGrowUp(uint32_t nodeIdx, const AABB& box, float maxErr,
                        uint32_t laneMask)
{
    float added = 0.0f;
    TlasNode* node = &tlasNodes_[nodeIdx];
    while (node->parent >= 0)
    {
        const int32_t childIdx = int32_t(nodeIdx);
        nodeIdx = uint32_t(node->parent);
        node = &tlasNodes_[nodeIdx];
        TlasMeta& meta = tlasMeta_[nodeIdx];
        uint32_t l = 0;
        for (; l < kWide; ++l)
            if ((node->validMask & (1u << l)) && node->child[l] == childIdx) break;
        if (l == kWide) break;   // already unlinked; nothing above to grow
        AABB laneBox = node->bounds.lane(l);
        if (laneBox.contains(box) && meta.maxErr.v[l] >= maxErr &&
            (meta.laneMask[l] & laneMask) == laneMask)
            break;
        const float was = surfaceArea(laneBox);
        laneBox.expand(box);
        node->bounds.setLane(l, laneBox);
        meta.maxErr.v[l] = std::max(meta.maxErr.v[l], maxErr);
        meta.laneMask[l] |= laneMask;
        added += surfaceArea(laneBox) - was;
    }
    return added;
}

AABB SpatialDatabase::tlasNodeExtent(uint32_t node, float& maxErr,
                                     uint32_t& laneMask) const
{
    const TlasNode& n = tlasNodes_[node];
    const TlasMeta& meta = tlasMeta_[node];
    AABB u = AABB::empty();
    maxErr = 0.0f;
    laneMask = 0;
    for (uint32_t l = 0; l < kWide; ++l)
    {
        if (!(n.validMask & (1u << l))) continue;
        u.expand(n.bounds.lane(l));
        maxErr = std::max(maxErr, meta.maxErr.v[l]);
        laneMask |= meta.laneMask[l];
    }
    return u;
}

int32_t SpatialDatabase::tlasAllocNode()
{
    if (!tlasFreeNodes_.empty())
    {
        const int32_t idx = tlasFreeNodes_.back();
        tlasFreeNodes_.pop_back();
        TlasNode& n = tlasNodes_[uint32_t(idx)];
        TlasMeta& meta = tlasMeta_[uint32_t(idx)];
        n.bounds = WideBounds::allEmpty();
        meta.maxErr = float8::splat(0.0f);
        n.validMask = 0;
        n.parent = -1;
        for (uint32_t l = 0; l < kWide; ++l)
        {
            n.child[l] = 0;
            meta.laneMask[l] = 0;
        }
        return idx;
    }
    FRONTIER_CHECK(tlasNodes_.size() < kInvalidInstanceId,
               "SpatialDatabase: exhausted the 24-bit TLAS node space");
    const int32_t idx = int32_t(tlasNodes_.size());
    TlasNode& n = tlasNodes_.emplace_back();
    TlasMeta& meta = tlasMeta_.emplace_back();
    n.bounds = WideBounds::allEmpty();
    meta.maxErr = float8::splat(0.0f);
    n.parent = -1;
    for (uint32_t l = 0; l < kWide; ++l)
    {
        n.child[l] = 0;
        meta.laneMask[l] = 0;
    }
    return idx;
}

// Incremental edits trade a little tree quality for O(depth) instead of a full
// rebuild. This is what bounds how much of that accumulates.
void SpatialDatabase::tlasNoteEdit()
{
    tlasLevelTmp_.clear();
    ++tlasEdits_;
    if (float(tlasEdits_) > float(tlasLeafCount_) * config_.tlasEditFraction)
        tlasDirty_ = true;
}

// Descend to the leaf whose lane box grows least, then either take a free lane
// there or SPLIT it: a new node takes the full leaf's place in its parent and
// holds the leaf plus the new instance. Splitting always succeeds, which is why
// there is no "the tree is full, give up and rebuild" case -- the alternative,
// hunting for a free lane somewhere up the chain, fails immediately on a tree
// that was just built full.
void SpatialDatabase::tlasInsert(InstanceId id)
{
    if (tlasDirty_) return;   // the pending rebuild will enumerate this instance
    materializeTlasGlobalOffset();
    Instance& inst = instances_[id];
    if (tlasRoot_ < 0)
    {
        tlasDirty_ = true;    // no tree yet; let a build make the first one
        return;
    }

    uint32_t cur = uint32_t(tlasRoot_);
    for (;;)
    {
        const TlasNode& n = tlasNodes_[cur];
        int32_t  bestChild = -1;
        float    bestGrowth = FLT_MAX;
        for (uint32_t l = 0; l < kWide; ++l)
        {
            if (!(n.validMask & (1u << l))) continue;
            if (n.child[l] < 0) continue;   // an instance, not a subtree
            AABB box = n.bounds.lane(l);
            const float was = surfaceArea(box);
            box.expand(inst.worldBox);
            const float growth = surfaceArea(box) - was;
            if (growth < bestGrowth)
            {
                bestGrowth = growth;
                bestChild = n.child[l];
            }
        }
        if (bestChild < 0) break;   // this node holds instances: place here
        cur = uint32_t(bestChild);
    }

    const uint32_t full = (1u << kWide) - 1;
    uint32_t host = cur;
    if (tlasNodes_[cur].validLanes() == full)
    {
        // Split. The new node replaces `cur` wherever `cur` was referenced, and
        // adopts it, so nothing above needs to know the difference.
        const int32_t mIdx = tlasAllocNode();
        TlasNode& m = tlasNodes_[uint32_t(mIdx)];
        TlasMeta& mMeta = tlasMeta_[uint32_t(mIdx)];
        TlasNode& l0 = tlasNodes_[cur];

        float    childErr = 0.0f;
        uint32_t childMask = 0;
        const AABB childBox = tlasNodeExtent(cur, childErr, childMask);

        m.parent = l0.parent;
        m.bounds.setLane(0, childBox);
        mMeta.maxErr.v[0] = childErr;
        m.child[0] = int32_t(cur);
        mMeta.laneMask[0] = childMask;
        m.validMask = 1u;
        l0.parent = mIdx;

        if (m.parent < 0)
            tlasRoot_ = mIdx;
        else
        {
            TlasNode& p = tlasNodes_[uint32_t(m.parent)];
            for (uint32_t l = 0; l < kWide; ++l)
                if ((p.validMask & (1u << l)) && p.child[l] == int32_t(cur))
                {
                    p.child[l] = mIdx;
                    break;
                }
        }
        host = uint32_t(mIdx);
    }

    TlasNode& h = tlasNodes_[host];
    TlasMeta& hMeta = tlasMeta_[host];
    const uint32_t lane = uint32_t(std::countr_zero(~h.validMask & full));
    h.bounds.setLane(lane, inst.worldBox);
    hMeta.maxErr.v[lane] = inst.maxErrWorld;
    h.child[lane] = ~int32_t(id);
    hMeta.laneMask[lane] = inst.mask;
    h.setLeafLane(lane);
    inst.setTlasPlacement(host, lane);
    ++tlasLeafCount_;

    tlasNoteGrowth(tlasGrowUp(host, inst.worldBox, inst.maxErrWorld, inst.mask));
    tlasNoteEdit();
}

// Invalidate the lane and unlink any node that empties. Boxes are left loose,
// which is the same grow-only discipline motion uses: a lane that is larger
// than its contents costs a little traversal and nothing else.
void SpatialDatabase::tlasRemove(InstanceId id)
{
    if (tlasDirty_) return;
    Instance& inst = instances_[id];
    if (inst.tlasNode() == kInvalidInstanceId) return;

    uint32_t nodeIdx = inst.tlasNode();
    const uint32_t lane = inst.tlasLane();
    if (nodeIdx >= tlasNodes_.size() ||
        !(tlasNodes_[nodeIdx].validMask & (1u << lane)) ||
        tlasNodes_[nodeIdx].child[lane] != ~int32_t(id))
    {
        tlasDirty_ = true;   // bookkeeping disagrees; rebuild rather than guess
        return;
    }

    tlasNodes_[nodeIdx].clearLane(lane);
    inst.clearTlasPlacement();
    if (tlasLeafCount_) --tlasLeafCount_;

    while (tlasNodes_[nodeIdx].validLanes() == 0)
    {
        const int32_t parent = tlasNodes_[nodeIdx].parent;
        if (parent < 0)
        {
            tlasRoot_ = -1;
            tlasFreeNodes_.push_back(int32_t(nodeIdx));
            break;
        }
        TlasNode& p = tlasNodes_[uint32_t(parent)];
        for (uint32_t l = 0; l < kWide; ++l)
            if ((p.validMask & (1u << l)) && p.child[l] == int32_t(nodeIdx))
            {
                p.clearLane(l);
                break;
            }
        tlasFreeNodes_.push_back(int32_t(nodeIdx));
        nodeIdx = uint32_t(parent);
    }

    tlasNoteEdit();
}

void SpatialDatabase::tlasOnInstanceMoved(InstanceId id)
{
    if (tlasDirty_) return;
    Instance& inst = instances_[id];
    if (inst.tlasNode() == kInvalidInstanceId)
    {
        tlasDirty_ = true;
        return;
    }

    // Leaf lanes retain a grow-only motion envelope. After the first excursion
    // a bounded oscillator stays inside that envelope and no longer writes
    // either its leaf or ancestors. Non-overview queries retest the small set
    // of loose roots against their exact Instance bounds in tlasQueryImpl.
    const uint32_t nodeIdx = inst.tlasNode();
    const uint32_t lane = inst.tlasLane();
    TlasNode& node = tlasNodes_[nodeIdx];
    TlasMeta& meta = tlasMeta_[nodeIdx];
    const AABB oldLeafBounds = node.bounds.lane(lane);
    const bool envelopeAlreadyCovers =
        oldLeafBounds.contains(inst.worldBox) &&
        meta.maxErr.v[lane] >= inst.maxErrWorld;
    if (envelopeAlreadyCovers)
    {
        if (oldLeafBounds.mn.x != inst.worldBox.mn.x ||
            oldLeafBounds.mn.y != inst.worldBox.mn.y ||
            oldLeafBounds.mn.z != inst.worldBox.mn.z ||
            oldLeafBounds.mx.x != inst.worldBox.mx.x ||
            oldLeafBounds.mx.y != inst.worldBox.mx.y ||
            oldLeafBounds.mx.z != inst.worldBox.mx.z ||
            meta.maxErr.v[lane] != inst.maxErrWorld)
            instanceTlasLoose_[id] = 1;
        return;
    }

    AABB envelope = oldLeafBounds;
    envelope.expand(inst.worldBox);
    node.bounds.setLane(lane, envelope);
    meta.maxErr.v[lane] = std::max(meta.maxErr.v[lane], inst.maxErrWorld);
    instanceTlasLoose_[id] = 1;

    const float added =
        tlasGrowUp(nodeIdx, envelope, inst.maxErrWorld, inst.mask);

    tlasNoteGrowth(added);
}

void SpatialDatabase::flushInstanceMoves()
{
    if (tlasItemsTmp_.empty()) return;
    if (tlasDirty_)
    {
        tlasItemsTmp_.clear();
        return;
    }

    // Once a quarter of the population moves, independently probing the leaf
    // envelope and shared ancestors costs more memory traffic than streaming
    // the complete compact TLAS once. The exact refit also removes loose-lane
    // retests from the following query.
    if (tlasItemsTmp_.size() * 4 >= tlasLeafCount_)
    {
        tlasItemsTmp_.clear();
        tlasRefitAllExact();
    }
    else
    {
        for (const InstanceId dense : tlasItemsTmp_)
            if (dense < instances_.size() && instances_[dense].alive())
                tlasOnInstanceMoved(dense);
        tlasItemsTmp_.clear();
    }
}

void SpatialDatabase::tlasRefitAllExact()
{
    if (tlasRoot_ < 0) return;
    materializeTlasGlobalOffset();

    if (tlasLevelTmp_.empty())
    {
        std::vector<uint32_t>& stack = tlasItemsTmp_;
        stack.clear();
        stack.push_back(uint32_t(tlasRoot_));
        while (!stack.empty())
        {
            const uint32_t packed = stack.back();
            stack.pop_back();
            const uint32_t nodeIndex = packed & kInstanceIdMask;
            if ((packed & ~kInstanceIdMask) != 0)
            {
                tlasLevelTmp_.push_back(int32_t(nodeIndex));
                continue;
            }

            stack.push_back(nodeIndex | (1u << kInstanceIdBits));
            const TlasNode& node = tlasNodes_[nodeIndex];
            uint32_t lanes = node.validLanes();
            while (lanes)
            {
                const uint32_t lane = uint32_t(std::countr_zero(lanes));
                lanes &= lanes - 1;
                if (node.child[lane] >= 0)
                    stack.push_back(uint32_t(node.child[lane]));
            }
        }
    }

    float exactArea = 0.0f;
    for (const int32_t signedNodeIndex : tlasLevelTmp_)
    {
        const uint32_t nodeIndex = uint32_t(signedNodeIndex);
        TlasNode& node = tlasNodes_[nodeIndex];
        TlasMeta& meta = tlasMeta_[nodeIndex];
        uint32_t lanes = node.validLanes();
        while (lanes)
        {
            const uint32_t lane = uint32_t(std::countr_zero(lanes));
            lanes &= lanes - 1;
            const int32_t child = node.child[lane];
            AABB bounds;
            float maxError;
            uint32_t layerMask;
            if (child < 0)
            {
                const InstanceId dense = InstanceId(~child);
                const Instance& instance = instances_[dense];
                bounds = instance.worldBox;
                maxError = instance.maxErrWorld;
                layerMask = instance.mask;
                instanceTlasLoose_[dense] = 0;
            }
            else
            {
                bounds = tlasNodeExtent(uint32_t(child), maxError,
                                        layerMask);
            }
            node.bounds.setLane(lane, bounds);
            meta.maxErr.v[lane] = maxError;
            meta.laneMask[lane] = layerMask;
            exactArea += surfaceArea(bounds);
        }
    }

    tlasGrownArea_ = std::max(0.0f, exactArea - tlasBaseArea_);
    if (tlasBaseArea_ > 0.0f &&
        tlasGrownArea_ > tlasBaseArea_ * config_.tlasAreaDrift)
    {
        tlasDirty_ = true;
        tlasQualityBuild_ = true;
    }
}

// 21 bits per axis -> 63-bit Morton key.
static inline uint64_t expandBits21(uint64_t v)
{
    v &= 0x1FFFFFull;
    v = (v | v << 32) & 0x1F00000000FFFFull;
    v = (v | v << 16) & 0x1F0000FF0000FFull;
    v = (v | v << 8)  & 0x100F00F00F00F00Full;
    v = (v | v << 4)  & 0x10C30C30C30C30C3ull;
    v = (v | v << 2)  & 0x1249249249249249ull;
    return v;
}

// Stable LSD radix sort of the 63-bit Morton coordinate. Up to six 11-bit
// passes keep the histogram in L1 and turn the rebuild's O(N log N) comparison
// sort into linear streaming passes. Equal coordinates retain live-instance
// order; their order is immaterial to the tree and stability keeps it fully
// deterministic without paying for a second comparison sort.
template <class Item>
static void radixSortMorton(std::vector<Item>& keys,
                            std::vector<Item>& scratch,
                            uint64_t keyVariation)
{
    if (keys.size() < 1024)
    {
        std::sort(keys.begin(), keys.end());
        return;
    }
    if (keyVariation == 0) return;

    constexpr uint32_t kBits = 11;
    constexpr uint32_t kBuckets = 1u << kBits;
    constexpr uint64_t kMask = kBuckets - 1;
    std::array<uint32_t, kBuckets> offsets{};
    scratch.resize(keys.size());

    auto* src = &keys;
    auto* dst = &scratch;
    for (uint32_t shift = 0; shift < 63; shift += kBits)
    {
        if (((keyVariation >> shift) & kMask) == 0) continue;
        offsets.fill(0);
        for (const auto& item : *src)
            ++offsets[size_t((item.key() >> shift) & kMask)];

        uint32_t next = 0;
        for (uint32_t& count : offsets)
        {
            const uint32_t begin = next;
            next += count;
            count = begin;
        }

        for (const auto& item : *src)
            (*dst)[offsets[size_t((item.key() >> shift) & kMask)]++] = item;
        std::swap(src, dst);
    }
    if (src != &keys) keys.swap(scratch);
}

// Partition items[lo, hi) into [lo, m) and [m, hi). BinnedSAH scans 16 bins on
// all three axes and takes the cheapest plane; Median (and any degenerate SAH
// case, e.g. coincident centroids) falls back to a longest-axis median split,
// which always makes progress.
int SpatialDatabase::tlasSplit(std::vector<uint32_t>& items, int lo, int hi)
{
    const int count = hi - lo;
    if (count <= 1) return hi;

    AABB cb = AABB::empty();
    for (int k = lo; k < hi; ++k)
        cb.expand(instances_[items[k]].worldBox.center());
    const float4 ext = cb.mx - cb.mn;

    if (config_.tlasQuality == TlasQuality::BinnedSAH)
    {
        constexpr int kBins = 16;
        float bestCost = FLT_MAX;
        int   bestAxis = -1, bestBin = -1;

        for (int axis = 0; axis < 3; ++axis)
        {
            const float e = axisOf(ext, axis);
            if (!(e > 0.0f)) continue;
            const float base  = axisOf(cb.mn, axis);
            const float scale = float(kBins) / e;

            AABB binBox[kBins];
            int  binCount[kBins] = {};
            for (int i = 0; i < kBins; ++i) binBox[i] = AABB::empty();
            for (int k = lo; k < hi; ++k)
            {
                const Instance& in = instances_[items[k]];
                int b = int((axisOf(in.worldBox.center(), axis) - base) * scale);
                b = b < 0 ? 0 : (b >= kBins ? kBins - 1 : b);
                binBox[b].expand(in.worldBox);
                ++binCount[b];
            }

            float leftArea[kBins];
            int   leftCount[kBins];
            AABB  acc = AABB::empty();
            int   cnt = 0;
            for (int i = 0; i < kBins; ++i)
            {
                acc.expand(binBox[i]);
                cnt += binCount[i];
                leftArea[i]  = surfaceArea(acc);
                leftCount[i] = cnt;
            }

            acc = AABB::empty();
            cnt = 0;
            for (int i = kBins - 1; i >= 1; --i)
            {
                acc.expand(binBox[i]);
                cnt += binCount[i];
                const int l = leftCount[i - 1], r = cnt;
                if (l == 0 || r == 0) continue;
                const float cost =
                    config_.tlasTraversalCost +
                    config_.tlasIntersectCost *
                        (leftArea[i - 1] * float(l) + surfaceArea(acc) * float(r));
                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestAxis = axis;
                    bestBin  = i;
                }
            }
        }

        if (bestAxis >= 0)
        {
            const float base  = axisOf(cb.mn, bestAxis);
            const float scale = float(kBins) / axisOf(ext, bestAxis);
            const auto  mid = std::partition(
                items.begin() + lo, items.begin() + hi,
                [&](uint32_t idx)
                {
                    int b = int(
                        (axisOf(instances_[idx].worldBox.center(), bestAxis) - base) *
                        scale);
                    b = b < 0 ? 0 : (b >= kBins ? kBins - 1 : b);
                    return b < bestBin;
                });
            const int m = int(mid - items.begin());
            if (m > lo && m < hi) return m;
        }
    }

    const int axis = (ext.x >= ext.y && ext.x >= ext.z) ? 0 : (ext.y >= ext.z ? 1 : 2);
    const int mid = (lo + hi) / 2;
    std::nth_element(items.begin() + lo, items.begin() + mid, items.begin() + hi,
                     [&](uint32_t a, uint32_t b)
                     {
                         return axisOf(instances_[a].worldBox.center(), axis) <
                                axisOf(instances_[b].worldBox.center(), axis);
                     });
    return mid;
}

// Recursive kWide-way build: log2(kWide) levels of binary splits per node.
// Slower than the Morton path but produces noticeably tighter trees. Used for
// structural rebuilds (instances added/removed), which are rare and long-lived.
int32_t SpatialDatabase::tlasBuildRange(std::vector<uint32_t>& items, int lo, int hi, int32_t parent)
{
    const int32_t idx = tlasAllocNode();
    tlasNodes_[idx].parent = parent;

    const int count = hi - lo;
    if (count <= int(kWide))
    {
        for (int k = 0; k < count; ++k)
        {
            const uint32_t instIdx = items[lo + k];
            Instance& inst = instances_[instIdx];
            TlasNode& n = tlasNodes_[idx];
            TlasMeta& meta = tlasMeta_[idx];
            n.bounds.setLane(uint32_t(k), inst.worldBox);
            meta.maxErr.v[k] = inst.maxErrWorld;
            n.child[k] = ~int32_t(instIdx);
            meta.laneMask[k] = inst.mask;
            n.setLeafLane(uint32_t(k));
            inst.setTlasPlacement(uint32_t(idx), uint32_t(k));
        }
        return idx;
    }

    // The fixed kWide-way splitter is excellent for large ranges, but using
    // all kWide groups directly above the leaves creates mostly empty leaf
    // nodes. Pack the final two levels explicitly: a 20-instance BVH8 range
    // needs three leaf children, not eight children holding 2-3 instances
    // each. The range is already spatially local; a longest-axis ordering
    // keeps each packed leaf local as well.
    if (count <= int(kWide * kWide))
    {
        AABB centroidBounds = AABB::empty();
        for (int k = lo; k < hi; ++k)
            centroidBounds.expand(
                instances_[items[k]].worldBox.center());
        const float4 extent = centroidBounds.extent();
        const int axis =
            (extent.x >= extent.y && extent.x >= extent.z)
                ? 0
                : (extent.y >= extent.z ? 1 : 2);
        std::sort(items.begin() + lo, items.begin() + hi,
                  [&](uint32_t a, uint32_t b)
                  {
                      const float av = axisOf(
                          instances_[a].worldBox.center(), axis);
                      const float bv = axisOf(
                          instances_[b].worldBox.center(), axis);
                      return av < bv || (av == bv && a < b);
                  });

        uint32_t lane = 0;
        for (int begin = lo; begin < hi; begin += int(kWide), ++lane)
        {
            const int end = std::min(begin + int(kWide), hi);
            const int32_t child =
                tlasBuildRange(items, begin, end, idx);
            float maxError = 0.0f;
            uint32_t layerMask = 0;
            const AABB bounds = tlasNodeExtent(
                uint32_t(child), maxError, layerMask);
            TlasNode& node = tlasNodes_[idx];
            TlasMeta& meta = tlasMeta_[idx];
            node.bounds.setLane(lane, bounds);
            meta.maxErr.v[lane] = maxError;
            node.child[lane] = child;
            meta.laneMask[lane] = layerMask;
            node.validMask |= 1u << lane;
        }
        return idx;
    }

    int cuts[kWide + 1] = {};
    cuts[0] = lo;
    cuts[kWide] = hi;
    for (uint32_t span = kWide; span > 1; span /= 2)
        for (uint32_t first = 0; first < kWide; first += span)
        {
            const uint32_t middle = first + span / 2;
            cuts[middle] = tlasSplit(
                items, cuts[first], cuts[first + span]);
        }

    for (uint32_t g = 0; g < kWide; ++g)
    {
        if (cuts[g] >= cuts[g + 1]) continue;
        const int32_t child = tlasBuildRange(items, cuts[g], cuts[g + 1], idx);

        // Union the child's lanes into our lane for it.
        AABB u = AABB::empty();
        float me = 0.0f;
        uint32_t lm = 0;
        const TlasNode& cn = tlasNodes_[child];
        const TlasMeta& childMeta = tlasMeta_[child];
        for (uint32_t l = 0; l < kWide; ++l)
        {
            if (!(cn.validMask & (1u << l))) continue;
            u.expand(cn.bounds.lane(l));
            me = std::max(me, childMeta.maxErr.v[l]);
            lm |= childMeta.laneMask[l];
        }
        TlasNode& n = tlasNodes_[idx];
        TlasMeta& meta = tlasMeta_[idx];
        n.bounds.setLane(g, u);
        meta.maxErr.v[g] = me;
        n.child[g] = child;
        meta.laneMask[g] = lm;
        n.validMask |= 1u << g;
    }
    return idx;
}

void SpatialDatabase::reorderInstancesByTlas()
{
    FRONTIER_ASSERT(pendingMoves_.empty(),
                    "SpatialDatabase: cannot reorder instances with queued "
                    "deformation edits");

    const size_t slotCount = instances_.size();
    const size_t liveCount = liveInstances_.size();
    std::vector<InstanceId> order;
    order.reserve(slotCount);

    // Match tlasQuery's reverse-DFS stack and ascending leaf-lane order
    // exactly. A visible query is therefore a monotonic subsequence of these
    // dense ids; culling can skip ranges but cannot turn the stream random.
    if (tlasRoot_ >= 0)
    {
        std::vector<int32_t> stack;
        stack.push_back(tlasRoot_);
        while (!stack.empty())
        {
            const int32_t node = stack.back();
            stack.pop_back();
            const TlasNode& n = tlasNodes_[uint32_t(node)];
            for (uint32_t lane = 0; lane < kWide; ++lane)
            {
                if (!(n.validMask & (1u << lane))) continue;
                const int32_t child = n.child[lane];
                if (child >= 0)
                    stack.push_back(child);
                else
                    order.push_back(InstanceId(~child));
            }
        }
    }
    FRONTIER_ASSERT(order.size() == liveCount,
                "SpatialDatabase: TLAS traversal did not contain every live instance");

    std::vector<InstanceId> oldToNew(slotCount, kInvalidInstanceId);
    for (InstanceId next = 0; next < liveCount; ++next)
        oldToNew[order[next]] = next;

    // Rewrite the TLAS before moving its parallel instance streams.
    for (TlasNode& n : tlasNodes_)
    {
        uint32_t lanes = n.validLanes();
        while (lanes)
        {
            const uint32_t lane = uint32_t(std::countr_zero(lanes));
            lanes &= lanes - 1;
            if (n.child[lane] < 0)
            {
                const InstanceId old = InstanceId(~n.child[lane]);
                n.child[lane] = ~int32_t(oldToNew[old]);
            }
        }
    }

    // Public handles are independent of dense positions, so dead dense slots
    // no longer need to survive a rebuild. Compacting them here makes both
    // memory and subsequent permutations scale with the live population,
    // rather than with the database's historical peak.
    std::vector<Instance> newInstances(liveCount);
    std::vector<InstanceOrientation> newOrientations;
    const bool hadOrientations = !instanceOrientations_.empty();
    if (hadOrientations) newOrientations.resize(liveCount);
    std::vector<detail::PayloadWord> newTlasRootPayloads;
    const bool hadTlasRootPayloads = !tlasRootPayloads_.empty();
    if (hadTlasRootPayloads) newTlasRootPayloads.resize(liveCount);
    std::vector<uint32_t> newFrontierVersions(liveCount);
    std::vector<float> newMotionTravel(liveCount);
    std::vector<uint8_t> newTlasLoose(liveCount);
    std::vector<InstanceId> newDenseToHandle(liveCount, kInvalidInstanceId);
    std::vector<uint32_t> newFlat;
    const bool hadFlatStream = !instanceFlatSlots_.empty();
    if (hadFlatStream) newFlat.resize(liveCount, kInvalidIndex);

    for (InstanceId next = 0; next < liveCount; ++next)
    {
        const InstanceId old = order[next];
        newInstances[next] = std::move(instances_[old]);
        if (hadOrientations)
            newOrientations[next] = instanceOrientations_[old];
        if (hadTlasRootPayloads)
            newTlasRootPayloads[next] = tlasRootPayloads_[old];
        newFrontierVersions[next] = instanceFrontierVersions_[old];
        newMotionTravel[next] = instanceMotionTravel_[old];
        newTlasLoose[next] = instanceTlasLoose_[old];
        newDenseToHandle[next] = instanceDenseToHandle_[old];
        if (hadFlatStream) newFlat[next] = instanceFlatSlots_[old];
        if (newInstances[next].rootSlot != kInvalidIndex)
            slots_[newInstances[next].rootSlot].owner.index = next;
    }
    instances_.swap(newInstances);
    if (hadOrientations)
        instanceOrientations_.swap(newOrientations);
    if (hadTlasRootPayloads)
        tlasRootPayloads_.swap(newTlasRootPayloads);
    instanceFrontierVersions_.swap(newFrontierVersions);
    instanceMotionTravel_.swap(newMotionTravel);
    instanceTlasLoose_.swap(newTlasLoose);
    instanceDenseToHandle_.swap(newDenseToHandle);
    if (hadFlatStream) instanceFlatSlots_.swap(newFlat);

    liveInstances_.resize(liveCount);
    for (InstanceId dense = 0; dense < liveCount; ++dense)
    {
        liveInstances_[dense] = dense;
        instances_[dense].liveIndex = dense;
        const InstanceId handle = instanceDenseToHandle_[dense];
        FRONTIER_ASSERT(handle < instanceHandleToDense_.size(),
                    "SpatialDatabase: dense instance has an invalid public handle");
        instanceHandleToDense_[handle] = dense;
    }
    freeInstances_.clear();

    instanceLayoutSpatialized_ = liveCount != 0;
    if (++instanceLayoutVersion_ == 0) ++instanceLayoutVersion_;
    if (++instanceMappingVersion_ == 0) ++instanceMappingVersion_;
}

// Two-tier rebuild policy:
//  - structural rebuilds (add/remove) take the quality path: rare,
//    long-lived, quality matters (contribution culling leans on tight
//    maxErr/bounds lanes);
//  - motion rebuilds (area threshold) take the Morton path: one sort plus
//    contiguous groups of kWide per level, ~5x faster to build while keeping
//    grow-only ancestor bloat bounded.
void SpatialDatabase::tlasRebuild(bool reorderInstances)
{
    // Rebuild enumerates instance bounds, so first bring a deferred uniform
    // translation into those records and the old TLAS nodes.
    materializeTlasGlobalOffset();
    tlasNodes_.clear();
    tlasMeta_.clear();
    tlasLevelTmp_.clear();
    tlasFreeNodes_.clear();
    tlasRoot_ = -1;
    tlasEdits_ = 0;
    tlasGrownArea_ = 0.0f;
    tlasDirty_ = false;
    for (const InstanceId dense : liveInstances_)
        instanceTlasLoose_[dense] = 0;

    const bool promoted = tlasQualityBuild_;
    const bool quality =
        promoted && config_.tlasQuality != TlasQuality::Morton;
    tlasQualityBuild_ = false;

    if (quality)
    {
        std::vector<uint32_t>& items = tlasItemsTmp_;
        items.assign(liveInstances_.begin(), liveInstances_.end());
        tlasLeafCount_ = uint32_t(items.size());
        if (!items.empty())
            tlasRoot_ = tlasBuildRange(items, 0, int(items.size()), -1);
    }
    else
    {
        tlasKeys_.clear();
        uint64_t firstMorton = 0;
        uint64_t mortonVariation = 0;
        AABB cb = AABB::empty();
        for (const InstanceId i : liveInstances_)
            cb.expand(instances_[i].worldBox.center());
        const float4 lo = cb.mn;
        const float4 ext = cb.extent();
        const float sx = ext.x > 0.0f ? 2097151.0f / ext.x : 0.0f;
        const float sy = ext.y > 0.0f ? 2097151.0f / ext.y : 0.0f;
        const float sz = ext.z > 0.0f ? 2097151.0f / ext.z : 0.0f;
        for (const InstanceId i : liveInstances_)
        {
            const float4 c = instances_[i].worldBox.center();
            const uint64_t kx = expandBits21(uint64_t((c.x - lo.x) * sx));
            const uint64_t ky = expandBits21(uint64_t((c.y - lo.y) * sy));
            const uint64_t kz = expandBits21(uint64_t((c.z - lo.z) * sz));
            const uint64_t morton = (kx << 2) | (ky << 1) | kz;
            if (tlasKeys_.empty()) firstMorton = morton;
            else mortonVariation |= morton ^ firstMorton;
            tlasKeys_.push_back({morton, i});
        }
        tlasLeafCount_ = uint32_t(tlasKeys_.size());
        if (!tlasKeys_.empty())
        {
            radixSortMorton(tlasKeys_, tlasKeysTmp_, mortonVariation);

            // Leaf level: consecutive groups of kWide instances.
            std::vector<int32_t>& cur = tlasLevelTmp_;
            cur.clear();
            for (size_t base = 0; base < tlasKeys_.size(); base += kWide)
            {
                const int32_t idx = tlasAllocNode();
                TlasNode& n = tlasNodes_[idx];
                TlasMeta& meta = tlasMeta_[idx];
                const uint32_t cnt =
                    uint32_t(std::min<size_t>(kWide, tlasKeys_.size() - base));
                for (uint32_t k = 0; k < cnt; ++k)
                {
                    const uint32_t instIdx = tlasKeys_[base + k].instance;
                    Instance& inst = instances_[instIdx];
                    n.bounds.setLane(k, inst.worldBox);
                    meta.maxErr.v[k] = inst.maxErrWorld;
                    n.child[k] = ~int32_t(instIdx);
                    meta.laneMask[k] = inst.mask;
                    n.setLeafLane(k);
                    inst.setTlasPlacement(uint32_t(idx), k);
                }
                cur.push_back(idx);
            }

            // Inner levels: group kWide nodes until one remains.
            std::vector<int32_t> next;
            while (cur.size() > 1)
            {
                next.clear();
                for (size_t base = 0; base < cur.size(); base += kWide)
                {
                    const int32_t idx = tlasAllocNode();
                    TlasNode& n = tlasNodes_[idx];
                    TlasMeta& meta = tlasMeta_[idx];
                    const uint32_t cnt =
                        uint32_t(std::min<size_t>(kWide, cur.size() - base));
                    for (uint32_t k = 0; k < cnt; ++k)
                    {
                        const int32_t childIdx = cur[base + k];
                        TlasNode& cn = tlasNodes_[childIdx];
                        const TlasMeta& childMeta = tlasMeta_[childIdx];
                        cn.parent = idx;
                        AABB u = AABB::empty();
                        float me = 0.0f;
                        uint32_t lm = 0;
                        for (uint32_t l = 0; l < kWide; ++l)
                        {
                            if (!(cn.validMask & (1u << l))) continue;
                            u.expand(cn.bounds.lane(l));
                            me = std::max(me, childMeta.maxErr.v[l]);
                            lm |= childMeta.laneMask[l];
                        }
                        n.bounds.setLane(k, u);
                        meta.maxErr.v[k] = me;
                        n.child[k] = childIdx;
                        meta.laneMask[k] = lm;
                        n.validMask |= 1u << k;
                    }
                    next.push_back(idx);
                }
                cur.swap(next);
            }
            tlasRoot_ = cur[0];
        }
    }

    // Morton is itself the selected quality tier when configured explicitly.
    // Record its population baseline just like median/SAH promoted builds.
    if (promoted) tlasQualityCount_ = tlasLeafCount_;

    if (reorderInstances) reorderInstancesByTlas();

    // Baseline for the area-drift trigger: the total lane area this build
    // started from. Motion is allowed to add a configured fraction of it
    // before the tree is considered bloated enough to rebuild.
    tlasBaseArea_ = 0.0f;
    for (const TlasNode& n : tlasNodes_)
        for (uint32_t l = 0; l < kWide; ++l)
            if (n.validMask & (1u << l)) tlasBaseArea_ += surfaceArea(n.bounds.lane(l));
    tlasItemsTmp_.clear();
    tlasLevelTmp_.clear();
}

template<bool UseMask, bool UseMinPix>
void SpatialDatabase::tlasQueryImpl(const Camera& view, float minPix,
                          std::vector<VisibleItem>& outVisible,
                          std::vector<TlasItem>& stack) const
{
    outVisible.clear();
    FRONTIER_CHECK(!tlasDirty_ && pendingMoves_.empty() &&
                       tlasItemsTmp_.empty(),
                   "SpatialQuery::selectFrontier: call applyUpdates() after "
                   "database changes");
    if (tlasRoot_ < 0) return;

    const float4 qmn = view.queryMin(), qmx = view.queryMax();

    // A convex frustum that contains every root lane contains every instance
    // below those lanes. Avoid visiting every internal node merely to
    // rediscover the common overview/shadow-view case where the complete
    // population is visible with no active planes.
    if constexpr (!UseMask && !UseMinPix)
    {
        if (tlasRootContainsPopulation(view))
        {
            outVisible.reserve(liveInstances_.size());
            for (const InstanceId instance : liveInstances_)
                outVisible.emplace_back(instance, uint8_t(0));
            return;
        }
    }

    stack.clear();
    stack.push_back({tlasRoot_, kAllPlanes});
    while (!stack.empty())
    {
        const TlasItem it = stack.back();
        stack.pop_back();
        const TlasNode& n = tlasNodes_[it.node()];

        const uint8_t inMask = it.mask();
        uint8_t outMasks[kWide];
        uint32_t survivors = inMask
                                 ? testWideAabb(n.bounds, view.frustum, inMask,
                                                outMasks) & n.validMask
                                 : n.validLanes();
        if (!survivors) continue;

        // Query-level dispatch removes this block entirely for the default
        // all-ones view mask.
        if constexpr (UseMask)
        {
            const TlasMeta& meta = tlasMeta_[it.node()];
            for (uint32_t l = 0; l < kWide; ++l)
                if (!(meta.laneMask[l] & view.viewMask)) survivors &= ~(1u << l);
            if (!survivors) continue;
        }

        if constexpr (UseMinPix)
        {
            const TlasMeta& meta = tlasMeta_[it.node()];
            const float8 d2 = distanceToBoxesSq(n.bounds, qmn, qmx);
            const float8 errs = screenErrorFromSq8(meta.maxErr, view.k, d2);
            for (uint32_t l = 0; l < kWide; ++l)
                if (errs.v[l] < minPix) survivors &= ~(1u << l);
        }

        while (survivors)
        {
            const uint32_t l = uint32_t(std::countr_zero(survivors));
            survivors &= survivors - 1;
            const int32_t c = n.child[l];
            if (c >= 0)
                stack.push_back({c, inMask ? outMasks[l] : uint8_t(0)});
            else
            {
                const uint32_t instance = uint32_t(~c);
                uint8_t exactMask = inMask ? outMasks[l] : uint8_t(0);
                if (instanceTlasLoose_[instance])
                {
                    const Instance& inst = instances_[instance];
                    if (exactMask != 0 &&
                        testAabb(inst.worldBox, view.frustum, exactMask) ==
                            CullState::Outside)
                        continue;
                    if constexpr (UseMinPix)
                    {
                        const float distance =
                            distanceToBox(inst.worldBox, qmn, qmx);
                        if (screenError(inst.maxErrWorld, view.k, distance) <
                            minPix)
                            continue;
                    }
                }
                outVisible.emplace_back(instance, exactMask);
            }
        }
    }
}

// ============================================================================
// garbage collection
// ============================================================================

void SpatialDatabase::lruUnlink(uint32_t slot)
{
    SubtreeInstanceRt& rt = slots_[slot];
    const uint32_t prev = rt.lruPrev();
    const uint32_t next = rt.lruNext();
    if (prev != kInvalidIndex) slots_[prev].setLruNext(next);
    else if (lruHead_ == slot) lruHead_ = next;
    if (next != kInvalidIndex) slots_[next].setLruPrev(prev);
    else if (lruTail_ == slot) lruTail_ = prev;
    rt.setLruPrev(kInvalidIndex);
    rt.setLruNext(kInvalidIndex);
}

void SpatialDatabase::lruPushFront(uint32_t slot)
{
    SubtreeInstanceRt& rt = slots_[slot];
    rt.setLruPrev(kInvalidIndex);
    rt.setLruNext(lruHead_);
    if (lruHead_ != kInvalidIndex) slots_[lruHead_].setLruPrev(slot);
    lruHead_ = slot;
    if (lruTail_ == kInvalidIndex) lruTail_ = slot;
}

void SpatialDatabase::lruTouch(uint32_t slot, uint32_t epoch)
{
    SubtreeInstanceRt& rt = slots_[slot];
    if (rt.lastTouched == epoch || int32_t(epoch - rt.lastTouched) <= 0) return;
    rt.lastTouched = epoch;
    if (lruHead_ == slot) return;
    lruUnlink(slot);
    lruPushFront(slot);
}

void SpatialDatabase::consumeMountUsage(SpatialQuery& query)
{
    SpatialQuery* queries[] = {&query};
    consumeMountUsage(queries);
}

bool SpatialDatabase::tlasRootContainsPopulation(const Camera& view) const
{
    FRONTIER_CHECK(!tlasDirty_ && pendingMoves_.empty() &&
                       tlasItemsTmp_.empty(),
                   "SpatialQuery::selectFrontier: call applyUpdates() after "
                   "database changes");
    if (tlasRoot_ < 0) return true;

    const TlasNode& root = tlasNodes_[uint32_t(tlasRoot_)];
    uint8_t rootMasks[kWide];
    const uint32_t valid = root.validLanes();
    const uint32_t inside =
        testWideAabb(root.bounds, view.frustum, kAllPlanes, rootMasks) & valid;
    if (inside != valid) return false;

    uint32_t lanes = valid;
    while (lanes)
    {
        const uint32_t lane = uint32_t(std::countr_zero(lanes));
        lanes &= lanes - 1;
        if (rootMasks[lane] != 0) return false;
    }
    return true;
}

void SpatialDatabase::consumeMountUsage(std::span<SpatialQuery* const> queries)
{
    struct Event
    {
        uint32_t slot;
        uint32_t lastUsed;
    };

    // Validate the complete caller set before consuming any feedback. A
    // mixed-database contract failure must not clear earlier queries in the
    // span and leave retention policy half-applied.
    for (SpatialQuery* query : queries)
        if (query)
            FRONTIER_CHECK(
                query->database_ == nullptr || query->database_ == this,
                "SpatialDatabase::collect: SpatialQuery belongs to another "
                "SpatialDatabase");

    size_t eventCapacity = 0;
    for (SpatialQuery* query : queries)
        if (query) eventCapacity += query->dirtyMounts_.size();
    std::vector<Event> events;
    events.reserve(eventCapacity);
    uint32_t firstEventEpoch = 0;
    bool oneEventEpoch = true;

    for (SpatialQuery* query : queries)
    {
        if (!query) continue;
        for (const uint32_t slot : query->dirtyMounts_)
        {
            if (slot >= query->mountUse_.size()) continue;
            SpatialQuery::MountUseRec& rec = query->mountUse_[slot];
            rec.setPending(false);
            if (slot >= slots_.size()) continue;
            const MountStamp& stamp = mountStamps_[slot];
            if (!stamp.inUse() || stamp.generation() != rec.generation()) continue;
            if (events.empty())
                firstEventEpoch = rec.lastUsed;
            else if (rec.lastUsed != firstEventEpoch)
                oneEventEpoch = false;
            events.push_back({slot, rec.lastUsed});
        }
        query->dirtyMounts_.clear();
    }

    // Feedback may have accumulated for several frames and may come from
    // several cameras. Replay it oldest-to-newest so push-front preserves a
    // true LRU order instead of depending on context-list or discovery order.
    // The normal per-frame collection case gives every event the same epoch;
    // equal-age LRU order is irrelevant, so avoid sorting that cohort at all.
    if (!oneEventEpoch)
        std::sort(events.begin(), events.end(), [this](const Event& a,
                                                       const Event& b)
        {
            return frame_ - a.lastUsed > frame_ - b.lastUsed;
        });
    for (const Event& event : events) lruTouch(event.slot, event.lastUsed);
}

void SpatialDatabase::recordMountUsage(SpatialQuery& query, uint32_t slot) const
{
    FRONTIER_ASSERT(query.database_ == this,
                    "SpatialQuery is not bound to this SpatialDatabase");
    const uint32_t generation = mountStamps_[slot].generation();
    if (query.mountUse_.size() <= slot)
        query.mountUse_.resize(size_t(slot) + 1);
    SpatialQuery::MountUseRec& rec = query.mountUse_[slot];
    if (rec.generation() != generation)
    {
        rec = SpatialQuery::MountUseRec{};
        rec.setGeneration(generation);
    }
    rec.lastUsed = frame_;
    if (!rec.pending())
    {
        rec.setPending(true);
        query.dirtyMounts_.push_back(slot);
    }
}

void SpatialDatabase::tlasQuery(const Camera& view, float minPix,
                      std::vector<VisibleItem>& outVisible,
                      std::vector<TlasItem>& stack) const
{
    const bool useMask = view.viewMask != ~0u;
    const bool useMinPix = minPix > 0.0f;
    if (useMask)
    {
        if (useMinPix)
            tlasQueryImpl<true, true>(view, minPix, outVisible, stack);
        else
            tlasQueryImpl<true, false>(view, minPix, outVisible, stack);
    }
    else if (useMinPix)
        tlasQueryImpl<false, true>(view, minPix, outVisible, stack);
    else
        tlasQueryImpl<false, false>(view, minPix, outVisible, stack);
}

CollectResult SpatialDatabase::collect(size_t maxMountedSubtrees,
                                       uint32_t minAge)
{
    size_t unmounted = 0;
    uint32_t slot = lruTail_;
    while (mountedSubtreeCount() > maxMountedSubtrees &&
           slot != kInvalidIndex)
    {
        const uint32_t prev = slots_[slot].lruPrev();
        const SubtreeInstanceRt& rt = slots_[slot];
        const bool eligible = rt.inUse() &&
                              rt.mountedChildSubtrees() == 0 &&
                              (frame_ - rt.lastTouched) >= minAge;
        if (eligible)
        {
            unmountSlot(slot);
            ++unmounted;
        }
        slot = prev;
    }
    return {unmounted};
}

CollectResult SpatialDatabase::collect(SpatialQuery& query, size_t maxMountedSubtrees,
                                       uint32_t minAge)
{
    consumeMountUsage(query);
    return collect(maxMountedSubtrees, minAge);
}

CollectResult SpatialDatabase::collect(std::span<SpatialQuery* const> queries,
                             size_t maxMountedSubtrees, uint32_t minAge)
{
    consumeMountUsage(queries);
    return collect(maxMountedSubtrees, minAge);
}

// ============================================================================
// frontier selection
// ============================================================================

// One SIMD issue per kWide children: masked tri-state frustum, distance and
// screen error, lanes = children. Surviving PLAIN LEAVES are emitted right
// here (they are in both cuts by definition — no visit, no metadata reads);
// surviving interior/mountable nodes go onto the DFS stack with their err and
// narrowed plane mask carried along.
//
// Normal and dense-overlay bounds come through item.bounds() without a per-block
// branch. The sparse-overlay instantiation consults its compact patch table;
// template dispatch happens once per subtree rather than once per block.
template<bool FullyReady, bool SparseOverlay, bool TrackAncestor>
void SpatialDatabase::wideVisit(
    const WorkItem& item, const SubtreeView& pg, float errClamp, uint32_t gen,
    InstanceId instance, uint32_t node, uint8_t mask, uint8_t currentKids,
    uint8_t idealKids, const Camera& local, Worker& w,
    uint32_t ancestorCandidate) const
{
    const uint32_t cc = pg.childCount(node);
    uint32_t b = pg.wideOffset(node);
    const float8 clamp = float8::splat(errClamp);
    // LOD distance is measured to the camera ENVELOPE, not to a point: this
    // is the whole of hysteresis. The envelope collapses to local.pos when
    // damping is off, and then this is bit-identical to a point query.
    const float4 qmn = local.queryMin(), qmx = local.queryMax();
    for (uint32_t base = 0; base < cc; base += kWide, ++b)
    {
        const WideBlock& blk = pg.wide_[b];
        const WideBounds& wb = [&]() -> const WideBounds&
        {
            if constexpr (!SparseOverlay)
                return item.bounds(b);
            else
            {
                const Overlay& ov = overlays_[item.sparseOverlay];
                const uint32_t patch = ov.widePatch[b];
                return patch == kInvalidIndex ? pg.wide_[b].bounds
                                              : ov.patchedWide[patch];
            }
        }();
        // One load carries the valid, terminal-leaf, and zero-error lane masks.
        // `survivors` never exceeds kWide bits, so ANDing it with the whole
        // word keeps exactly the valid lanes.
        const uint32_t lanes = pg.blockMask_[b];
        FRONTIER_STAT(w, wideBlocksTested, 1);
        uint8_t outMasks[kWide];
        const uint32_t survivors =
            testWideAabb(wb, local.frustum, mask, outMasks) & lanes;
        if (!survivors) continue;
        FRONTIER_STAT(w, lanesSurvived, uint64_t(std::popcount(survivors)));

        const uint32_t leafLanes = blockLeafLanes(lanes);
        if ((survivors & ~leafLanes) == 0 &&
            (errClamp <= 0.0f ||
             (survivors & ~blockZeroErrorLanes(lanes)) == 0))
        {
            uint32_t zeroLeaves = survivors;
            while (zeroLeaves)
            {
                const uint32_t l =
                    uint32_t(std::countr_zero(zeroLeaves));
                zeroLeaves &= zeroLeaves - 1;
                const uint32_t c = blk.child[l];
                const FrontierEntry entry{
                    NodeHandle{item.slot(), c, gen}, uint8_t(0), instance};
                if constexpr (TrackAncestor)
                {
                    const bool ready =
                        subtrees_[slots_[item.slot()].definition]
                            .isNodeReady(c);
                    w.addAncestorIdeal(entry, ancestorCandidate, ready);
                }
                else if constexpr (FullyReady)
                    w.result.shared.push(entry);
                else
                    w.emit(entry, currentKids != 0, idealKids != 0);
            }
            continue;
        }

        // The clamp is invariant (D) across a mount boundary, applied here
        // rather than baked into the definition. One vminps; a no-op when
        // where errClamp is FLT_MAX.
        //
        // Squared distance, then one reciprocal square root: never a sqrt and
        // never a divide. See the note on screenErrorFromSq8 for why that is
        // worth more than the arithmetic it saves.
        const float8 eff = min8(blk.error, clamp);
        const float8 d2 = distanceToBoxesSq(wb, qmn, qmx);
        const float8 errs = screenErrorFromSq8(eff, local.k, d2);

        uint32_t leaves = survivors & leafLanes;
        while (leaves)
        {
            const uint32_t l = uint32_t(std::countr_zero(leaves));
            leaves &= leaves - 1;
            const uint32_t c = blk.child[l];
            const FrontierEntry entry =
                makeFrontierEntry(NodeHandle{item.slot(), c, gen}, errs.v[l], w.bar,
                             w.barInv, instance);
            if constexpr (TrackAncestor)
            {
                const bool ready =
                    subtrees_[slots_[item.slot()].definition].isNodeReady(c);
                w.addAncestorIdeal(entry, ancestorCandidate, ready);
            }
            else if constexpr (FullyReady)
                w.result.shared.push(entry);
            else
                w.emit(entry, currentKids != 0, idealKids != 0);
        }

        uint32_t inner = survivors & ~leafLanes;
        while (inner)
        {
            const uint32_t l = uint32_t(std::countr_zero(inner));
            inner &= inner - 1;
            const uint32_t c = blk.child[l];
            const uint8_t planes = outMasks[l];
            if constexpr (FullyReady)
                w.nodeStack.push_back({c, errs.v[l], planes, 1, 1});
            else
                w.nodeStack.push_back(
                    {c, errs.v[l], planes, currentKids, idealKids});
            if constexpr (TrackAncestor)
                w.nodeCandidates.push_back(ancestorCandidate);

            // This lane is the only kind that gets DECIDED: runSubtree asks
            // whether its error clears the bar, and plain leaves (handled
            // above) are emitted without asking. The answer flips when the
            // distance reaches eff * k / bar, so the gap between where this
            // node is and where that happens is how far the camera may travel
            // before this instance's cut could differ. See SpatialQuery.
            if (w.trackMargin && (FullyReady || idealKids))
            {
                const float mountScale = mountTransforms_[item.slot()].scale;
                w.maxError = std::max(w.maxError, eff.v[l] * mountScale);
                const float flipAt = eff.v[l] * local.k / w.bar;
                const float d = std::sqrt(d2.v[l]);
                const float slack =
                    (d > flipAt ? d - flipAt : flipAt - d) * mountScale;
                if (slack < w.margin) w.margin = slack;
            }
        }
    }
}

void SpatialDatabase::emitMountedRootLeavesInside(
    const WorkItem& item, const SubtreeView& subtree, float errClamp,
    uint32_t generation, InstanceId instance, float4 qmn, float4 qmx,
    float cameraK, Worker& w) const
{
    const uint32_t count = subtree.childCount(0);
    uint32_t block = subtree.wideOffset(0);
    const float8 clamp = float8::splat(errClamp);
    for (uint32_t base = 0; base < count; base += kWide, ++block)
    {
        const WideBlock& children = subtree.wide_[block];
        const WideBounds& bounds = item.bounds(block);
        const uint32_t lanes = blockValidLanes(subtree.blockMask_[block]);
        FRONTIER_STAT(w, wideBlocksTested, 1);
        FRONTIER_STAT(w, lanesSurvived, uint64_t(std::popcount(lanes)));

        const float8 error = min8(children.error, clamp);
        const float8 distanceSq = distanceToBoxesSq(bounds, qmn, qmx);
        const float8 screen =
            screenErrorFromSq8(error, cameraK, distanceSq);
        uint32_t remaining = lanes;
        while (remaining)
        {
            const uint32_t lane = uint32_t(std::countr_zero(remaining));
            remaining &= remaining - 1;
            w.result.shared.push(makeFrontierEntry(
                NodeHandle{item.slot(), children.child[lane], generation},
                screen.v[lane], w.bar, w.barInv, instance));
        }
    }
}

void SpatialDatabase::emitMountedLeafBatchInside(
    const SubtreeInstanceRt& owner, const SubtreeView& ownerSubtree,
    NodeItem current,
    size_t stackBase, InstanceId instance, const Camera& rootLocal,
    Worker& w) const
{
    const std::vector<uint32_t>& ownerLinks =
        mountLinks_[owner.mountLinks].slots;
    uint32_t childSlot = ownerLinks[current.node()];
    const uint32_t childDefinition = mountTransforms_[childSlot].definition();
    const SubtreeView& childView = subtrees_[childDefinition].view;
    const uint32_t childCount = childView.childCount(0);
    const uint32_t firstBlock = childView.wideOffset(0);
    const float4 rootQmn = rootLocal.queryMin();
    const float4 rootQmx = rootLocal.queryMax();
    // Cached traversal needs one exact root stamp. Mount-usage traversal still
    // records each physical placement, but does so inside this tight batch.
    if (w.coalesceMountTreeDependencies)
        recordTraversalDependency(w, childSlot);

    for (;;)
    {
        if (!w.coalesceMountTreeDependencies)
            recordTraversalDependency(w, childSlot);
        const MountTransformRt& mount = mountTransforms_[childSlot];
        const float invScale = 1.0f / mount.scale;
        const float4 qmn = (rootQmn - mount.pos) * invScale;
        const float4 qmx = (rootQmx - mount.pos) * invScale;
        const float8 clamp = float8::splat(mount.errClamp);
        FRONTIER_STAT(w, subtreesVisited, 1);

        uint32_t block = firstBlock;
        for (uint32_t base = 0; base < childCount;
             base += kWide, ++block)
        {
            const WideBlock& children = childView.wide_[block];
            const uint32_t lanes =
                blockValidLanes(childView.blockMask_[block]);
            FRONTIER_STAT(w, wideBlocksTested, 1);
            FRONTIER_STAT(w, lanesSurvived,
                          uint64_t(std::popcount(lanes)));

            const float8 error = min8(children.error, clamp);
            const float8 distanceSq =
                distanceToBoxesSq(children.bounds, qmn, qmx);
            const float8 screen =
                screenErrorFromSq8(error, rootLocal.k, distanceSq);
            uint32_t remaining = lanes;
            while (remaining)
            {
                const uint32_t lane =
                    uint32_t(std::countr_zero(remaining));
                remaining &= remaining - 1;
                w.result.shared.push(makeFrontierEntry(
                    NodeHandle{childSlot, children.child[lane],
                               mount.generation},
                    screen.v[lane], w.bar, w.barInv, instance));
            }
        }

        if (w.nodeStack.size() <= stackBase) return;
        const NodeItem next = w.nodeStack.back();
        if (!(next.err > w.bar) || next.planes() != 0 ||
            !metaIsMountable(ownerSubtree.meta_[next.node()]))
            return;
        const uint32_t nextSlot = ownerLinks[next.node()];
        if (nextSlot == kInvalidIndex) return;
        const MountTransformRt& nextMount = mountTransforms_[nextSlot];
        if (!nextMount.rootLeavesOnly() ||
            nextMount.definition() != childDefinition)
            return;

        w.nodeStack.pop_back();
        FRONTIER_STAT(w, nodesVisited, 1);
        current = next;
        childSlot = nextSlot;
    }
}

bool SpatialDatabase::visibleDescendantsCovered(uint32_t slot, uint32_t node, uint8_t mask,
                                      const Instance& inst,
                                      const Camera& rootLocal,
                                      Worker* dependencyWorker) const
{
    if (dependencyWorker)
        recordTraversalDependency(*dependencyWorker, slot);
    if (descendantsCovered(slot, node)) return true;

    // A fully-inside node has no invisible branch that can excuse a missing
    // payload. The propagated structural summary is therefore definitive.
    if (mask == 0) return false;

    const SubtreeInstanceRt* rt = &slots_[slot];
    if (node != 0 && subtreeView(*rt).isMountable(node))
    {
        const uint32_t child = mountedChildSlot(*rt, node);
        if (child == kInvalidIndex) return false;
        slot = child;
        node = 0;
        rt = &slots_[slot];
        if (dependencyWorker)
            recordTraversalDependency(*dependencyWorker, slot);
    }

    const MountTransformRt& transform = mountTransforms_[slot];
    const bool identityTransform =
        transform.scale == 1.0f && transform.pos.x == 0.0f &&
        transform.pos.y == 0.0f && transform.pos.z == 0.0f;
    std::optional<Camera> transformed;
    const Camera& local = identityTransform
                              ? rootLocal
                              : transformed.emplace(
                                    mountLocalCamera(rootLocal, slot, mask));
    const SubtreeView& subtree = subtreeView(*rt);
    const uint32_t count = subtree.childCount(node);
    if (count == 0) return false;
    const Overlay* overlay = findOverlay(inst, slot);

    uint32_t child = node + 1;
    for (uint32_t k = 0; k < count; ++k)
    {
        uint8_t childMask = mask;
        const AABB childBounds = nodeBoundsFrom(overlay, subtree, child);
        if (testAabb(childBounds, local.frustum, childMask) != CullState::Outside &&
            !rt->isCovered(child))
        {
            if (!visibleDescendantsCovered(slot, child, childMask, inst, rootLocal,
                                           dependencyWorker))
                return false;
        }
        child += subtree.subtreeSize_[child];
    }
    return true;
}

template<bool FullyReady>
void SpatialDatabase::runSubtree(const WorkItem& item, const Instance& inst,
                    const Camera& local,
                    const SelectionParams& params, Worker& w) const
{
    if (item.sparseOverlay == kInvalidIndex)
        runSubtreeImpl<FullyReady, false>(item, inst, local, params, w);
    else
        runSubtreeImpl<FullyReady, true>(item, inst, local, params, w);
}

void SpatialDatabase::runFullyReadyRootLeaves(
    const WorkItem& item, const Instance& inst, const Camera& rootLocal,
    Worker& w) const
{
    const SubtreeInstanceRt& rt = slots_[item.slot()];
    const MountTransformRt& transform = mountTransforms_[item.slot()];
    const bool identityTransform =
        transform.scale == 1.0f && transform.pos.x == 0.0f &&
        transform.pos.y == 0.0f && transform.pos.z == 0.0f;
    std::optional<Camera> transformed;
    const Camera& local = identityTransform
                              ? rootLocal
                              : transformed.emplace(mountLocalCamera(
                                    rootLocal, item.slot(), item.mask()));
    recordTraversalDependency(w, item.slot());
    FRONTIER_STAT(w, subtreesVisited, 1);
    const SubtreeView& pg = subtrees_[rt.definition].view;
    const uint32_t gen = rt.generation();
    const InstanceId instance =
        publicInstanceId(InstanceId(&inst - instances_.data()));
    w.nodeStack.clear();
    wideVisit<true, false>(item, pg, rt.errClamp, gen, instance, 0,
                           item.mask(), 1, 1, local, w);
    FRONTIER_ASSERT(w.nodeStack.empty(),
                    "root-leaf definition produced interior work");
}

bool SpatialDatabase::tryRunFullyRefinedBoundary(
    const WorkItem& item, const Instance& inst, const Camera& rootLocal,
    Worker& w) const
{
    const SubtreeInstanceRt& rt = slots_[item.slot()];
    const MountTransformRt& transform = mountTransforms_[item.slot()];
    const bool identityTransform =
        transform.scale == 1.0f && transform.pos.x == 0.0f &&
        transform.pos.y == 0.0f && transform.pos.z == 0.0f;
    std::optional<Camera> transformed;
    const Camera& local = identityTransform
                              ? rootLocal
                              : transformed.emplace(mountLocalCamera(
                                    rootLocal, item.slot(), item.mask()));
    const SubtreeDefinitionRt& definition = subtrees_[rt.definition];
    const SubtreeView& pg = definition.view;
    FRONTIER_ASSERT(fullyRefinedLeafPlans_ &&
                        rt.definition < fullyRefinedLeafPlans_->size(),
                    "fully-refined definition has no leaf plan");
    const FullyRefinedLeafPlan& plan =
        (*fullyRefinedLeafPlans_)[rt.definition];
    const float innerError =
        std::min(definition.minInnerError(), rt.errClamp);
    if (inst.hasOverlayList() || !(innerError > 0.0f)) return false;

    const AABB bounds = pg.bounds();
    const float4 qmn = local.queryMin();
    const float4 qmx = local.queryMax();
    const float dx = std::max(std::fabs(qmn.x - bounds.mx.x),
                              std::fabs(qmx.x - bounds.mn.x));
    const float dy = std::max(std::fabs(qmn.y - bounds.mx.y),
                              std::fabs(qmx.y - bounds.mn.y));
    const float dz = std::max(std::fabs(qmn.z - bounds.mx.z),
                              std::fabs(qmx.z - bounds.mn.z));
    const float farthestSq = dx * dx + dy * dy + dz * dz;
    const float refineDistance = innerError * local.k * w.barInv;
    if (!(refineDistance * refineDistance > farthestSq)) return false;

    recordTraversalDependency(w, item.slot());
    FRONTIER_STAT(w, subtreesVisited, 1);
    FRONTIER_STAT(w, fullyRefinedSubtrees, 1);
    const uint32_t gen = rt.generation();
    const InstanceId instance =
        publicInstanceId(InstanceId(&inst - instances_.data()));
    w.nodeStack.clear();
    const auto visit = [&](uint32_t node, uint8_t mask)
    {
        if (mask == 0)
        {
            const FullyRefinedLeafRange range = plan.ranges[node];
            const uint32_t* terminal =
                plan.terminalNodes.data() + range.begin;
            w.result.shared.pushGenerated(
                range.count, [&](uint32_t i)
                {
                    return FrontierEntry{
                        NodeHandle{item.slot(), terminal[i], gen}, uint8_t(0),
                        instance};
                });
            return;
        }

        const uint32_t count = pg.childCount(node);
        uint32_t block = pg.wideOffset(node);
        for (uint32_t base = 0; base < count; base += kWide, ++block)
        {
            const WideBlock& children = pg.wide_[block];
            const uint32_t lanes = pg.blockMask_[block];
            FRONTIER_STAT(w, wideBlocksTested, 1);
            uint8_t outMasks[kWide];
            const uint32_t survivors =
                testWideAabb(item.bounds(block), local.frustum, mask,
                             outMasks) & lanes;
            if (!survivors) continue;
            FRONTIER_STAT(w, lanesSurvived,
                          uint64_t(std::popcount(survivors)));

            const uint32_t leafLanes = blockLeafLanes(lanes);
            uint32_t leaves = survivors & leafLanes;
            while (leaves)
            {
                const uint32_t lane = uint32_t(std::countr_zero(leaves));
                leaves &= leaves - 1;
                w.result.shared.push(FrontierEntry{
                    NodeHandle{item.slot(), children.child[lane], gen},
                    uint8_t(0), instance});
            }

            uint32_t inner = survivors & ~leafLanes;
            while (inner)
            {
                const uint32_t lane = uint32_t(std::countr_zero(inner));
                inner &= inner - 1;
                w.nodeStack.push_back(
                    {children.child[lane], 0.0f, outMasks[lane], 1, 1});
            }
        }
    };

    visit(0, item.mask());
    while (!w.nodeStack.empty())
    {
        const NodeItem entry = w.nodeStack.back();
        w.nodeStack.pop_back();
        FRONTIER_STAT(w, nodesVisited, 1);
        visit(entry.node(), entry.planes());
    }
    return true;
}

void SpatialDatabase::runSubtreeAncestor(
    const WorkItem& item, const Instance& inst, const Camera& local,
    const SelectionParams& params, uint32_t ancestorCandidate, Worker& w) const
{
    if (item.sparseOverlay == kInvalidIndex)
        runSubtreeAncestorImpl<false>(
            item, inst, local, params, ancestorCandidate, w);
    else
        runSubtreeAncestorImpl<true>(
            item, inst, local, params, ancestorCandidate, w);
}

template<bool SparseOverlay>
void SpatialDatabase::runSubtreeAncestorImpl(
    const WorkItem& item, const Instance& inst, const Camera& rootLocal,
    const SelectionParams& params, uint32_t ancestorCandidate, Worker& w) const
{
    const SubtreeInstanceRt& rt = slots_[item.slot()];
    const MountTransformRt& transform = mountTransforms_[item.slot()];
    const bool identityTransform =
        transform.scale == 1.0f && transform.pos.x == 0.0f &&
        transform.pos.y == 0.0f && transform.pos.z == 0.0f;
    std::optional<Camera> transformed;
    const Camera& local = identityTransform
                              ? rootLocal
                              : transformed.emplace(mountLocalCamera(
                                    rootLocal, item.slot(), item.mask()));
    recordTraversalDependency(w, item.slot());

    FRONTIER_STAT(w, subtreesVisited, 1);
    const SubtreeDefinitionRt& definition = subtrees_[rt.definition];
    const SubtreeView& pg = definition.view;
    const uint32_t gen = rt.generation();
    const InstanceId instance =
        publicInstanceId(InstanceId(&inst - instances_.data()));
    const float bar = params.threshold;

    w.nodeStack.clear();
    w.nodeCandidates.clear();
    wideVisit<false, SparseOverlay, true>(
        item, pg, rt.errClamp, gen, instance, 0, item.mask(), 0, 1, local, w,
        ancestorCandidate);

    while (!w.nodeStack.empty())
    {
        const NodeItem e = w.nodeStack.back();
        w.nodeStack.pop_back();
        FRONTIER_ASSERT(!w.nodeCandidates.empty(),
                        "ancestor candidate stack underflow");
        uint32_t candidate = w.nodeCandidates.back();
        w.nodeCandidates.pop_back();

        const uint32_t i = e.node();
        FRONTIER_STAT(w, nodesVisited, 1);
        const NodeHandle here{item.slot(), i, gen};
        const bool ready = definition.isNodeReady(i);
        const FrontierEntry entry =
            makeFrontierEntry(here, e.err, bar, w.barInv, instance);

        if (!(e.err > bar))
        {
            w.addAncestorIdeal(entry, candidate, ready);
            continue;
        }

        const bool mountable = metaIsMountable(pg.meta_[i]);
        const uint32_t childSlot =
            mountable ? mountedChildSlot(rt, i) : kInvalidIndex;
        if (mountable && childSlot == kInvalidIndex)
        {
            w.addAncestorIdeal(entry, candidate, ready);
            continue;
        }

        if (ready)
            candidate = w.addAncestorCandidate(candidate, entry);

        if (mountable)
        {
            w.work.push_back(
                makeWorkItem(childSlot, inst, 0, 1, e.planes()));
            w.workCandidates.push_back(candidate);
        }
        else
        {
            wideVisit<false, SparseOverlay, true>(
                item, pg, rt.errClamp, gen, instance, i, e.planes(), 0, 1,
                local, w, candidate);
        }
    }

    FRONTIER_ASSERT(w.nodeCandidates.empty(),
                    "ancestor candidate stack mismatch");
}

template<bool FullyReady, bool SparseOverlay>
void SpatialDatabase::runSubtreeImpl(const WorkItem& item,
                        const Instance& inst,
                        const Camera& rootLocal, const SelectionParams& params,
                        Worker& w) const
{
    const SubtreeInstanceRt& rt = slots_[item.slot()];
    const MountTransformRt& transform = mountTransforms_[item.slot()];
    const bool identityTransform =
        transform.scale == 1.0f && transform.pos.x == 0.0f &&
        transform.pos.y == 0.0f && transform.pos.z == 0.0f;
    std::optional<Camera> transformed;
    const Camera& local = identityTransform
                              ? rootLocal
                              : transformed.emplace(mountLocalCamera(
                                    rootLocal, item.slot(), item.mask()));
    recordTraversalDependency(w, item.slot());

    FRONTIER_STAT(w, subtreesVisited, 1);
    const SubtreeDefinitionRt& definition = subtrees_[rt.definition];
    const SubtreeView& pg = definition.view;
    const uint32_t gen = rt.generation();
    const InstanceId instance =
        publicInstanceId(InstanceId(&inst - instances_.data()));

    // One bar, no history: damping is already folded into the view's camera
    // envelope, which widened the measured error rather than moving the
    // threshold. That is what makes selection a pure read of the SpatialDatabase.
    const float bar = params.threshold;

    w.nodeStack.clear();
    const size_t stackBase = 0;
    wideVisit<FullyReady, SparseOverlay>(
        item, pg, rt.errClamp, gen, instance, 0, item.mask(), item.current(),
        item.ideal(), local, w);

    while (!w.nodeStack.empty())
    {
        const NodeItem e = w.nodeStack.back();
        w.nodeStack.pop_back();
        const uint32_t i = e.node();
        FRONTIER_STAT(w, nodesVisited, 1);

        const NodeHandle here{item.slot(), i, gen};

        if constexpr (FullyReady)
        {
            if (e.err > bar && e.planes() == 0 &&
                !inst.hasOverlayList() &&
                metaIsMountable(pg.meta_[i]) &&
                rt.mountLinks != kInvalidIndex)
            {
                const uint32_t childSlot =
                    mountLinks_[rt.mountLinks].slots[i];
                if (childSlot != kInvalidIndex &&
                    mountTransforms_[childSlot].rootLeavesOnly())
                {
                    emitMountedLeafBatchInside(
                        rt, pg, e, stackBase, instance, rootLocal, w);
                    continue;
                }
            }

            if (!(e.err > bar))
            {
                w.result.shared.push(
                    makeFrontierEntry(here, e.err, bar, w.barInv, instance));
                continue;
            }

            const bool exp = metaIsMountable(pg.meta_[i]);
            const uint32_t childSlot =
                exp ? mountedChildSlot(rt, i) : kInvalidIndex;
            if (exp)
            {
                if (childSlot == kInvalidIndex)
                    w.result.shared.push(
                        makeFrontierEntry(here, e.err, bar, w.barInv, instance));
                else
                {
                    const MountTransformRt& childMount =
                        mountTransforms_[childSlot];
                    const bool directLeaves =
                        childMount.rootLeavesOnly() &&
                        !inst.hasOverlayList();
                    if (directLeaves)
                    {
                        recordTraversalDependency(w, childSlot);
                        const SubtreeView& childView =
                            subtrees_[childMount.definition()].view;
                        const WorkItem childItem{
                            childSlot, childView.wideBounds(), 1, 1,
                            e.planes()};
                        FRONTIER_STAT(w, subtreesVisited, 1);
                        if (e.planes() == 0)
                        {
                            const MountTransformRt& childTransform =
                                mountTransforms_[childSlot];
                            const float invScale = 1.0f / childTransform.scale;
                            const float4 qmn =
                                (rootLocal.queryMin() - childTransform.pos) *
                                invScale;
                            const float4 qmx =
                                (rootLocal.queryMax() - childTransform.pos) *
                                invScale;
                            emitMountedRootLeavesInside(
                                childItem, childView, childMount.errClamp,
                                childMount.generation, instance, qmn, qmx,
                                rootLocal.k, w);
                        }
                        else
                        {
                            const Camera childLocal = mountLocalCamera(
                                rootLocal, childSlot, e.planes());
                            wideVisit<true, false>(
                                childItem, childView, childMount.errClamp,
                                childMount.generation, instance, 0,
                                e.planes(), 1, 1, childLocal, w);
                        }
                    }
                    else
                        w.work.push_back(
                            makeWorkItem(childSlot, inst, 1, 1,
                                         e.planes()));
                }
            }
            else
                wideVisit<true, SparseOverlay>(item, pg, rt.errClamp, gen,
                                               instance, i, e.planes(), 1, 1,
                                               local, w);
        }
        else
        {
            const bool current = e.current();
            const bool ideal = e.ideal();
            uint8_t nextCurrent = 0;
            uint8_t nextIdeal = 0;

            // Current-only traversal happens when the ideal frontier stopped
            // at an unavailable proxy whose descendants nevertheless form a
            // complete ready cover. Stop at the nearest ready descendant;
            // otherwise continue through the precomputed cover.
            if (!ideal)
            {
                if (definition.isNodeReady(i))
                {
                    w.result.currentOnly.push(
                        makeFrontierEntry(here, e.err, bar, w.barInv, instance));
                    continue;
                }
                nextCurrent = 1;
            }
            else if (!(e.err > bar))
            {
                const bool shared = current &&
                                    definition.isNodeReady(i);
                const FrontierEntry entry =
                    makeFrontierEntry(here, e.err, bar, w.barInv, instance);
                w.emit(entry, shared, true);
                if (!current || shared) continue;
                // The ideal proxy is unavailable, but a recursively complete
                // ready descendant cut exists because the current walk reached it.
                nextCurrent = 1;
            }

            const uint32_t m = pg.meta_[i];
            const bool exp = metaIsMountable(m);

            const uint32_t childSlot =
                exp ? mountedChildSlot(rt, i) : kInvalidIndex;

            if (ideal && e.err > bar && exp && childSlot == kInvalidIndex)
            {
                FRONTIER_ASSERT(!current || definition.isNodeReady(i),
                                "unavailable current mount proxy");
                const FrontierEntry entry =
                    makeFrontierEntry(here, e.err, bar, w.barInv, instance);
                w.emit(entry, current, true);
                continue;
            }

            if (ideal && e.err > bar)
            {
                const bool canDescend =
                    !current ||
                    visibleDescendantsCovered(item.slot(), i, e.planes(), inst, rootLocal,
                                               w.trackTouches ? &w : nullptr);
                if (current && !canDescend)
                {
                    FRONTIER_ASSERT(definition.isNodeReady(i),
                                    "uncovered current subtree");
                    w.result.currentOnly.push(
                        makeFrontierEntry(here, e.err, bar, w.barInv, instance));
                }
                nextCurrent = uint8_t(current && canDescend);
                nextIdeal = 1;
            }

            FRONTIER_ASSERT(nextCurrent || nextIdeal,
                            "node has no current or ideal continuation");
            if (exp)
            {
                FRONTIER_ASSERT(childSlot != kInvalidIndex,
                                "uncovered mounted subtree");
                w.work.push_back(makeWorkItem(
                    childSlot, inst, nextCurrent, nextIdeal, e.planes()));
            }
            else
                wideVisit<false, SparseOverlay>(
                    item, pg, rt.errClamp, gen, instance, i, e.planes(),
                    nextCurrent, nextIdeal, local, w);
        }
    }
}

void SpatialDatabase::runTlasRootInstance(
    uint32_t instIdx, const Camera& view, const SelectionParams& params,
    uint8_t mask, Worker& w) const
{
    const Instance& inst = instances_[instIdx];
    FRONTIER_STAT(w, instancesVisited, 1);

    const float worldDistance =
        inst.maxErrWorld > 0.0f
            ? distanceToBox(inst.worldBox, view.queryMin(), view.queryMax())
            : 0.0f;
    const float error = inst.maxErrWorld > 0.0f
                            ? screenError(inst.maxErrWorld, view.k,
                                          worldDistance)
                            : 0.0f;
    if (w.trackMargin && inst.maxErrWorld > 0.0f)
    {
        const float worldFlip = inst.maxErrWorld * view.k / w.bar;
        const float worldSlack = worldDistance > worldFlip
                                     ? worldDistance - worldFlip
                                     : worldFlip - worldDistance;
        w.margin = std::min(w.margin, worldSlack / inst.scale);
        w.maxError = std::max(w.maxError,
                              inst.maxErrWorld / inst.scale);
    }

    const InstanceId outputInstance = publicInstanceId(instIdx);
    const NodeHandle root = NodeHandle::tlasRoot(outputInstance,
                                                 inst.generation);
    const uint32_t childSlot = inst.rootSlot;
    if (!(error > params.threshold) || childSlot == kInvalidIndex)
    {
        w.result.shared.push(makeFrontierEntry(
            root, error, w.bar, w.barInv, outputInstance));
        return;
    }

    const Camera local = toLocal(view, inst.pos, inst.scale, mask);
    const bool fullyReady = mountedTreeFullyReady(childSlot);
    if (fullyReady)
    {
        const WorkItem rootItem =
            makeWorkItem(childSlot, inst, 1, 1, mask);
        if (mountTransforms_[childSlot].rootLeavesOnly() &&
            !inst.hasOverlayList())
        {
            runFullyReadyRootLeaves(rootItem, inst, local, w);
            return;
        }
        // The one-time proof costs more than the ordinary walk for toy
        // hierarchies. Registration sets this flag only for eligible trees
        // with at least sixteen authored nodes.
        if (mask != 0 &&
            mountTransforms_[childSlot].fullyRefinedCandidate() &&
            tryRunFullyRefinedBoundary(rootItem, inst, local, w)) [[unlikely]]
            return;
        // The first placement is already in hand. Enter it directly; the work
        // stack is only needed for mounted descendants discovered by the walk.
        runSubtree<true>(rootItem, inst, local, params, w);
        while (!w.work.empty())
        {
            const WorkItem item = w.work.back();
            w.work.pop_back();
            runSubtree<true>(item, inst, local, params, w);
        }
        return;
    }

    if (params.currentCutPolicy == CurrentCutPolicy::PreferReadyAncestors)
    {
        w.work.clear();
        w.workCandidates.clear();
        w.ancestorCandidates.clear();
        w.ancestorIdeal.clear();

        const uint32_t rootCandidate = w.addAncestorCandidate(
            kInvalidIndex,
            makeFrontierEntry(root, error, w.bar, w.barInv, outputInstance));
        runSubtreeAncestor(makeWorkItem(childSlot, inst, 0, 1, mask), inst,
                           local, params, rootCandidate, w);
        while (!w.work.empty())
        {
            const WorkItem item = w.work.back();
            w.work.pop_back();
            FRONTIER_ASSERT(!w.workCandidates.empty(),
                            "ancestor work stack underflow");
            const uint32_t candidate = w.workCandidates.back();
            w.workCandidates.pop_back();
            runSubtreeAncestor(item, inst, local, params, candidate, w);
        }
        FRONTIER_ASSERT(w.workCandidates.empty(),
                        "ancestor work stack mismatch");
        w.finishAncestorCut();
        return;
    }

    const bool currentCanDescend = visibleDescendantsCovered(
        childSlot, 0, mask, inst, local,
        w.trackTouches ? &w : nullptr);
    if (!currentCanDescend)
        w.result.currentOnly.push(makeFrontierEntry(
            root, error, w.bar, w.barInv, outputInstance));

    runSubtree<false>(makeWorkItem(childSlot, inst,
                                  uint8_t(currentCanDescend), 1, mask),
                      inst, local, params, w);
    while (!w.work.empty())
    {
        const WorkItem item = w.work.back();
        w.work.pop_back();
        const bool branchFullyReady = item.current() && item.ideal() &&
                                      mountedTreeFullyReady(item.slot());
        if (branchFullyReady)
            runSubtree<true>(item, inst, local, params, w);
        else
            runSubtree<false>(item, inst, local, params, w);
    }
}

// Keep the two mounted-root walkers adjacent to each other and to the subtree
// traversal machinery they call. The identity-only body stays first and
// unchanged; databases with an orientation stream use this second variant.
void SpatialDatabase::runOrientedTlasRootInstance(
    uint32_t instIdx, const Camera& view, const SelectionParams& params,
    uint8_t mask, Worker& w) const
{
    const Instance& inst = instances_[instIdx];
    FRONTIER_STAT(w, instancesVisited, 1);

    const float worldDistance =
        inst.maxErrWorld > 0.0f
            ? distanceToBox(inst.worldBox, view.queryMin(), view.queryMax())
            : 0.0f;
    const float error = inst.maxErrWorld > 0.0f
                            ? screenError(inst.maxErrWorld, view.k,
                                          worldDistance)
                            : 0.0f;
    if (w.trackMargin && inst.maxErrWorld > 0.0f)
    {
        const float worldFlip = inst.maxErrWorld * view.k / w.bar;
        const float worldSlack = worldDistance > worldFlip
                                     ? worldDistance - worldFlip
                                     : worldFlip - worldDistance;
        w.margin = std::min(w.margin, worldSlack / inst.scale);
        w.maxError = std::max(w.maxError,
                              inst.maxErrWorld / inst.scale);
    }

    const InstanceId outputInstance = publicInstanceId(instIdx);
    const NodeHandle root = NodeHandle::tlasRoot(outputInstance,
                                                 inst.generation);
    const uint32_t childSlot = inst.rootSlot;
    if (!(error > params.threshold) || childSlot == kInvalidIndex)
    {
        w.result.shared.push(makeFrontierEntry(
            root, error, w.bar, w.barInv, outputInstance));
        return;
    }

    const YawRotation yaw = instanceOrientations_[instIdx].yaw;
    const Camera local = identityYaw(yaw)
                             ? toLocal(view, inst.pos, inst.scale, mask)
                             : toLocal(view, inst.pos, inst.scale, yaw, mask);
    const bool fullyReady = mountedTreeFullyReady(childSlot);
    if (fullyReady)
    {
        const WorkItem rootItem =
            makeWorkItem(childSlot, inst, 1, 1, mask);
        if (mountTransforms_[childSlot].rootLeavesOnly() &&
            !inst.hasOverlayList())
        {
            runFullyReadyRootLeaves(rootItem, inst, local, w);
            return;
        }
        if (mask != 0 &&
            mountTransforms_[childSlot].fullyRefinedCandidate() &&
            tryRunFullyRefinedBoundary(rootItem, inst, local, w)) [[unlikely]]
            return;
        runSubtree<true>(rootItem, inst, local, params, w);
        while (!w.work.empty())
        {
            const WorkItem item = w.work.back();
            w.work.pop_back();
            runSubtree<true>(item, inst, local, params, w);
        }
        return;
    }

    if (params.currentCutPolicy == CurrentCutPolicy::PreferReadyAncestors)
    {
        w.work.clear();
        w.workCandidates.clear();
        w.ancestorCandidates.clear();
        w.ancestorIdeal.clear();

        const uint32_t rootCandidate = w.addAncestorCandidate(
            kInvalidIndex,
            makeFrontierEntry(root, error, w.bar, w.barInv, outputInstance));
        runSubtreeAncestor(makeWorkItem(childSlot, inst, 0, 1, mask), inst,
                           local, params, rootCandidate, w);
        while (!w.work.empty())
        {
            const WorkItem item = w.work.back();
            w.work.pop_back();
            FRONTIER_ASSERT(!w.workCandidates.empty(),
                            "ancestor work stack underflow");
            const uint32_t candidate = w.workCandidates.back();
            w.workCandidates.pop_back();
            runSubtreeAncestor(item, inst, local, params, candidate, w);
        }
        FRONTIER_ASSERT(w.workCandidates.empty(),
                        "ancestor work stack mismatch");
        w.finishAncestorCut();
        return;
    }

    const bool currentCanDescend = visibleDescendantsCovered(
        childSlot, 0, mask, inst, local,
        w.trackTouches ? &w : nullptr);
    if (!currentCanDescend)
        w.result.currentOnly.push(makeFrontierEntry(
            root, error, w.bar, w.barInv, outputInstance));

    runSubtree<false>(makeWorkItem(childSlot, inst,
                                  uint8_t(currentCanDescend), 1, mask),
                      inst, local, params, w);
    while (!w.work.empty())
    {
        const WorkItem item = w.work.back();
        w.work.pop_back();
        const bool branchFullyReady = item.current() && item.ideal() &&
                                      mountedTreeFullyReady(item.slot());
        if (branchFullyReady)
            runSubtree<true>(item, inst, local, params, w);
        else
            runSubtree<false>(item, inst, local, params, w);
    }
}

void SpatialDatabase::runTlasFlatInstance(uint32_t instIdx,
                                          const Camera& view, Worker& w) const
{
    const uint32_t marker = instanceFlatSlots_[instIdx];
    const bool zeroError = instances_[instIdx].hasZeroErrorRoot();
    const Instance* spatial = zeroError ? nullptr : &instances_[instIdx];
    FRONTIER_STAT(w, instancesVisited, 1);
    const float error = !zeroError && spatial->maxErrWorld > 0.0f
                            ? screenError(
                                  spatial->maxErrWorld, view.k,
                                  distanceToBox(spatial->worldBox,
                                                view.queryMin(),
                                                view.queryMax()))
                            : 0.0f;
    const InstanceId outputInstance = publicInstanceId(instIdx);
    NodeHandle handle;
    handle.lo = NodeHandle::kInvalidSlot |
                ((outputInstance & 0xfffu) << NodeHandle::kSlotBits);
    handle.hi = marker;
    w.result.shared.push(makeFrontierEntry(
        handle,
        error, w.bar, w.barInv, outputInstance));
}

void SpatialDatabase::runZeroErrorTlasFlatInstance(
    uint32_t instIdx, Worker& w) const
{
    FRONTIER_STAT(w, instancesVisited, 1);
    const InstanceId outputInstance = publicInstanceId(instIdx);
    NodeHandle handle;
    handle.lo = NodeHandle::kInvalidSlot |
                ((outputInstance & 0xfffu) << NodeHandle::kSlotBits);
    handle.hi = instanceFlatSlots_[instIdx];
    w.result.shared.push(makeFrontierEntry(
        handle, 0.0f, w.bar, w.barInv, outputInstance));
}

void SpatialDatabase::selectFrontierUncached(const Camera& camera, const SelectionParams& params,
                              SpatialQuery& query, SpatialQuery* usage,
                              FrontierResultSink& outResult) const
{
    QueryScratch& scratch = *query.scratch_;
    outResult.shared.clear();
    outResult.currentOnly.clear();
    outResult.idealOnly.clear();
    scratch.retainedVisible = false;
    query.wholeReusable_ = false;
#ifdef FRONTIER_STATS
    query.stats_ = SelectionStats{};
#endif
    query.reused_ = 0;

    if (usage)
    {
        FRONTIER_ASSERT(usage == &query && query.database_ == this,
                        "mount usage belongs to this SpatialQuery");
    }

    const Camera damped = query.damper_.damp(camera);
    const Camera tlasView =
        (tlasGlobalOffset_.x == 0.0f && tlasGlobalOffset_.y == 0.0f &&
         tlasGlobalOffset_.z == 0.0f)
            ? damped
            : toLocal(damped, tlasGlobalOffset_, 1.0f);
    tlasQuery(tlasView, params.minPix, scratch.visible, scratch.tlasStack);

    const uint32_t nVis = uint32_t(scratch.visible.size());
    query.walked_ = nVis;
    const uint32_t workerCount = config_.context.workerCount;
    const bool parallel = config_.parallelInstanceThreshold > 0 && workerCount > 1 &&
                          nVis >= config_.parallelInstanceThreshold;
    if (!parallel)
    {
        Worker& w = scratch.workers[0];
        w.work.clear();
        w.nodeStack.clear();
        w.touched.clear();
        w.trackTouches = usage != nullptr;
        w.uniqueTouches = false;
        w.coalesceMountTreeDependencies = false;
        w.result = outResult;
#ifdef FRONTIER_STATS
        w.stats = SelectionStats{};
#endif
        w.bar = params.threshold;
        w.barInv = params.threshold > 0.0f ? 1.0f / params.threshold : 0.0f;

        if (flatInstanceCount_ == liveInstances_.size())
        {
            if (tlasZeroErrorFlatInstanceCount_ == liveInstances_.size())
            {
                for (uint32_t i = 0; i < nVis; ++i)
                {
                    const uint32_t instIdx = scratch.visible[i].instance();
                    runZeroErrorTlasFlatInstance(instIdx, w);
                }
            }
            else
            {
                for (uint32_t i = 0; i < nVis; ++i)
                {
                    const uint32_t instIdx = scratch.visible[i].instance();
                    runTlasFlatInstance(instIdx, tlasView, w);
                }
            }
        }
        else if (flatInstanceCount_ == 0)
        {
            if (instanceOrientations_.empty())
            {
                for (uint32_t i = 0; i < nVis; ++i)
                    runTlasRootInstance(
                        scratch.visible[i].instance(), tlasView, params,
                        scratch.visible[i].mask(), w);
            }
            else
            {
                for (uint32_t i = 0; i < nVis; ++i)
                    runOrientedTlasRootInstance(
                        scratch.visible[i].instance(), tlasView, params,
                        scratch.visible[i].mask(), w);
            }
        }
        else
        {
            // Flat objects bypass the mounted hierarchy in a mixed forest.
            for (uint32_t i = 0; i < nVis; ++i)
            {
                const uint32_t instIdx = scratch.visible[i].instance();
                if (instanceFlatSlots_[instIdx] != kInvalidIndex)
                {
                    if (instances_[instIdx].hasZeroErrorRoot())
                        runZeroErrorTlasFlatInstance(instIdx, w);
                    else
                        runTlasFlatInstance(instIdx, tlasView, w);
                }
                else
                {
                    if (instanceOrientations_.empty())
                        runTlasRootInstance(instIdx, tlasView, params,
                                            scratch.visible[i].mask(), w);
                    else
                        runOrientedTlasRootInstance(
                            instIdx, tlasView, params,
                            scratch.visible[i].mask(), w);
                }
            }
        }

        outResult = w.result;
        w.result = FrontierResultSink{};
        if (usage)
            for (const uint32_t slot : w.touched) recordMountUsage(*usage, slot);
#ifdef FRONTIER_STATS
        query.stats_ = w.stats;
#endif
        return;
    }

    // ---- parallel selection -------------------------------------------------
    // Each worker takes a contiguous run of visible instances and fills its
    // own buffers, so concatenating in worker order reproduces the serial
    // order exactly — the cut is bit-identical whether or not this path runs.
    if (scratch.workers.size() < workerCount) scratch.workers.resize(workerCount);

    struct Chunk
    {
        const SpatialDatabase* database;
        QueryScratch*          scratch;
        const Camera*    camera;
        const SelectionParams* params;
        uint32_t         nVis;
        uint32_t         workerCount;
        uint32_t         flatMode;   // 0 none, 1 mixed,
                                     // 3 all zero-error, 4 all flat
    } chunk{this, &scratch, &tlasView, &params, nVis, workerCount,
            flatInstanceCount_ == liveInstances_.size()
                ? (tlasZeroErrorFlatInstanceCount_ == liveInstances_.size()
                       ? 3u
                       : 4u)
                : (flatInstanceCount_ == 0 ? 0u : 1u)};

    for (uint32_t k = 0; k < workerCount; ++k)
    {
        Worker& w = scratch.workers[k];
        w.work.clear();
        w.nodeStack.clear();
        w.touched.clear();
        w.trackTouches = usage != nullptr;
        w.uniqueTouches = false;
        w.coalesceMountTreeDependencies = false;
        w.frontierBuffer.clear();
        w.result = makeSink(w.frontierBuffer);
#ifdef FRONTIER_STATS
        w.stats = SelectionStats{};
#endif
        w.bar = params.threshold;
        w.barInv = params.threshold > 0.0f ? 1.0f / params.threshold : 0.0f;
    }

    config_.context.parallelFor(
        workerCount,
        [](uint32_t k, void* payload)
        {
            auto* c = static_cast<Chunk*>(payload);
            const SpatialDatabase& database = *c->database;
            const uint32_t per = (c->nVis + c->workerCount - 1) / c->workerCount;
            const uint32_t lo = std::min(k * per, c->nVis);
            const uint32_t hi = std::min(lo + per, c->nVis);
            Worker& w = c->scratch->workers[k];
            if (c->flatMode == 0)
            {
                if (database.instanceOrientations_.empty())
                {
                    for (uint32_t i = lo; i < hi; ++i)
                        database.runTlasRootInstance(
                            c->scratch->visible[i].instance(), *c->camera,
                            *c->params, c->scratch->visible[i].mask(), w);
                }
                else
                {
                    for (uint32_t i = lo; i < hi; ++i)
                        database.runOrientedTlasRootInstance(
                            c->scratch->visible[i].instance(), *c->camera,
                            *c->params, c->scratch->visible[i].mask(), w);
                }
            }
            else if (c->flatMode == 3)
            {
                for (uint32_t i = lo; i < hi; ++i)
                {
                    const uint32_t instIdx = c->scratch->visible[i].instance();
                    database.runZeroErrorTlasFlatInstance(instIdx, w);
                }
            }
            else if (c->flatMode == 4)
            {
                for (uint32_t i = lo; i < hi; ++i)
                {
                    const uint32_t instIdx = c->scratch->visible[i].instance();
                    database.runTlasFlatInstance(instIdx, *c->camera, w);
                }
            }
            else
            {
                for (uint32_t i = lo; i < hi; ++i)
                {
                    const uint32_t instIdx = c->scratch->visible[i].instance();
                    if (database.instanceFlatSlots_[instIdx] != kInvalidIndex)
                    {
                        if (database.instances_[instIdx].hasZeroErrorRoot())
                            database.runZeroErrorTlasFlatInstance(instIdx, w);
                        else
                            database.runTlasFlatInstance(instIdx, *c->camera, w);
                    }
                    else
                    {
                        if (database.instanceOrientations_.empty())
                            database.runTlasRootInstance(
                                instIdx, *c->camera, *c->params,
                                c->scratch->visible[i].mask(), w);
                        else
                            database.runOrientedTlasRootInstance(
                                instIdx, *c->camera, *c->params,
                                c->scratch->visible[i].mask(), w);
                    }
                }
            }
        },
        &chunk, config_.context.user);

    for (uint32_t k = 0; k < workerCount; ++k)
    {
        Worker& w = scratch.workers[k];
        outResult.shared.pushRange(w.frontierBuffer.shared.data(),
                                uint32_t(w.frontierBuffer.shared.size()));
        outResult.currentOnly.pushRange(w.frontierBuffer.currentOnly.data(),
                                     uint32_t(w.frontierBuffer.currentOnly.size()));
        outResult.idealOnly.pushRange(w.frontierBuffer.idealOnly.data(),
                                   uint32_t(w.frontierBuffer.idealOnly.size()));
        if (usage)
            for (const uint32_t slot : w.touched) recordMountUsage(*usage, slot);
#ifdef FRONTIER_STATS
        query.stats_.instancesVisited += w.stats.instancesVisited;
        query.stats_.subtreesVisited += w.stats.subtreesVisited;
        query.stats_.nodesVisited += w.stats.nodesVisited;
        query.stats_.wideBlocksTested += w.stats.wideBlocksTested;
        query.stats_.lanesSurvived += w.stats.lanesSurvived;
#endif
        w.result = FrontierResultSink{};
    }
}

// ---------------------------------------------------------------------------
// Cached selection
// ---------------------------------------------------------------------------

void SpatialQuery::reset()
{
    rec_.clear();
    recCold_.clear();
    secondDep_.clear();
    overflowCounts_.clear();
    freeOverflowCounts_.clear();
    store_.clear();
    resolvedPayloadStore_.clear();
    resolvedErrorStore_.clear();
    resolvedRecords_.clear();
    used_ = garbage_ = reused_ = walked_ = 0;
    travel_ = kTravel_ = 0.0f;
    wholeMargin_ = wholeTravel_ = wholeKTravel_ = wholeMaxSlope_ = 0.0f;
    wholeInstanceTravel_ = 0.0;
    primed_ = false;
    k_ = bar_ = 0.0f;
    currentCutPolicy_ = CurrentCutPolicy::PreferReadyDescendants;
#ifdef FRONTIER_STATS
    stats_ = SelectionStats{};
#endif
    database_ = nullptr;
    instanceLayoutVersion_ = 0;
    visibleMappingVersion_ = 0;
    wholeContentGeneration_ = wholeEpoch_ = 0;
    wholeReusable_ = false;
    resetMountUsage();
    if (scratch_)
    {
        scratch_->output.clear();
        scratch_->resolvedPayloadCurrent.clear();
        scratch_->resolvedErrorCurrent.clear();
        scratch_->renderRuns.clear();
        scratch_->segmentedRenderRequested = false;
        scratch_->renderRunsValid = false;
        scratch_->renderEntryCount = 0;
        scratch_->visible.clear();
        scratch_->previousVisible.clear();
        scratch_->retainedVisible = false;
        scratch_->retainedAllVisible = false;
        for (QueryScratch::ViewMemo& memo : scratch_->viewMemo)
        {
            memo.output.clear();
            memo.valid = false;
            memo.lastUsed = 0;
        }
        scratch_->memoClock = 0;
        scratch_->haveLastScene = false;
    }
    ++epoch_;
    // The half-life is configuration and survives; the accumulated window is
    // state and does not. This is the half that reset() exists for: records
    // would have expired on their own, an envelope stretched across a teleport
    // would not.
    damper_.reset();
}

void SpatialQuery::setReuseEnabled(bool enabled)
{
    if (reuseEnabled_ == enabled) return;
    reset();
    reuseEnabled_ = enabled;
}

size_t SpatialQuery::bytes() const
{
    return rec_.capacity() * sizeof(Rec) +
           recCold_.capacity() * sizeof(RecCold) +
           secondDep_.capacity() * sizeof(SecondDep) +
           overflowCounts_.capacity() * sizeof(OverflowCounts) +
           freeOverflowCounts_.capacity() * sizeof(uint32_t) +
           store_.capacity() * sizeof(FrontierEntry) +
           resolvedPayloadStore_.capacity() * sizeof(UserPayload) +
           resolvedErrorStore_.capacity() * sizeof(uint8_t) +
           resolvedRecords_.capacity() * sizeof(uint8_t) +
           mountUse_.capacity() * sizeof(MountUseRec) +
           dirtyMounts_.capacity() * sizeof(uint32_t) +
           (scratch_ ? scratch_->bytes() : 0);
}

// Runs are allocated by bumping and abandoned when an instance's cut outgrows
// its block, so the slab accumulates holes. Squeeze them out once the holes
// outweigh the live data. Records keep their contents; only `begin` moves.
void SpatialQuery::compact()
{
    // Record ids and slab-allocation order are unrelated (the latter follows
    // TLAS traversal order).  Compacting in-place while iterating rec_ could
    // therefore move one run over the still-unread source of another run.
    // Compaction is deliberately rare, so use a same-sized scratch slab and
    // keep the existing allocation headroom while making the copy order moot.
    detail::AppendBuffer<FrontierEntry> packed;
    packed.resize_uninitialized(store_.size());
    detail::AppendBuffer<UserPayload> resolvedPayloadsPacked;
    detail::AppendBuffer<uint8_t> resolvedErrorsPacked;
    const bool haveResolved = !resolvedPayloadStore_.empty();
    if (haveResolved)
    {
        resolvedPayloadsPacked.resize_uninitialized(store_.size());
        resolvedErrorsPacked.resize_uninitialized(store_.size());
    }
    uint32_t w = 0;
    for (size_t i = 0; i < rec_.size(); ++i)
    {
        Rec& r = rec_[i];
        RecCold& cold = recCold_[i];
        if (cold.capacity == 0) continue;
        if (r.validUntil <= travel_ + r.kSlope * kTravel_ || r.epoch != epoch_)
        {
            // Not reusable anyway: drop the block rather than move it.
            cold.capacity = 0;
            if (frontierCountsOverflow(r.counts))
                freeOverflowCounts_.push_back(frontierOverflowIndex(r.counts));
            r.counts = 0;
            if (!resolvedRecords_.empty()) resolvedRecords_[i] = 0;
            continue;
        }
        uint32_t shared = frontierCount(r.counts, 0);
        uint32_t current = frontierCount(r.counts, 1);
        uint32_t ideal = frontierCount(r.counts, 2);
        if (frontierCountsOverflow(r.counts))
        {
            const OverflowCounts& counts = overflowCounts_[frontierOverflowIndex(r.counts)];
            shared = counts.shared;
            current = counts.current;
            ideal = counts.ideal;
        }
        const uint32_t count = shared + current + ideal;
        if (count)
            std::memcpy(packed.data() + w, store_.data() + r.begin,
                        size_t(count) * sizeof(FrontierEntry));
        if (haveResolved && !resolvedRecords_.empty() && resolvedRecords_[i] &&
            shared + current != 0)
        {
            std::memcpy(resolvedPayloadsPacked.data() + w,
                        resolvedPayloadStore_.data() + r.begin,
                        size_t(shared + current) * sizeof(UserPayload));
            std::memcpy(resolvedErrorsPacked.data() + w,
                        resolvedErrorStore_.data() + r.begin,
                        size_t(shared + current) * sizeof(uint8_t));
        }
        r.begin = w;
        cold.capacity = count;
        w += count;
    }
    store_.swap(packed);
    if (haveResolved)
    {
        resolvedPayloadStore_.swap(resolvedPayloadsPacked);
        resolvedErrorStore_.swap(resolvedErrorsPacked);
    }
    used_ = w;
    garbage_ = 0;
}

void SpatialDatabase::selectFrontierCached(const Camera& camera, const SelectionParams& params,
                            SpatialQuery& query, SpatialQuery* usage,
                            FrontierResultSink& outResult) const
{
    QueryScratch& scratch = *query.scratch_;
    const bool segmentedRender = scratch.segmentedRenderRequested;
    if (!segmentedRender) scratch.renderRunsValid = false;
#ifdef FRONTIER_STATS
    query.stats_ = SelectionStats{};
#endif

    if (usage)
    {
        FRONTIER_ASSERT(usage == &query && query.database_ == this,
                        "mount usage belongs to this SpatialQuery");
    }

    // The SpatialQuery owns hysteresis, so it takes the raw Camera and damps it
    // here. Everything below -- the cull, the walk, the odometer -- sees `dv`
    // and only `dv`, which is what makes the reuse argument about the envelope
    // rather than about the camera.
    const Camera dv = query.damper_.damp(camera);

    // An all-visible stream is exactly `liveInstances_` with zero plane masks.
    // If the root lanes remain wholly inside, one BVH-width test proves that
    // the retained stream is still exact; do not rewrite and compare 40 KiB.
    const Camera tlasView =
        (tlasGlobalOffset_.x == 0.0f && tlasGlobalOffset_.y == 0.0f &&
         tlasGlobalOffset_.z == 0.0f)
            ? dv
            : toLocal(dv, tlasGlobalOffset_, 1.0f);
    const bool allVisible = params.minPix == 0.0f && dv.viewMask == ~0u &&
                            tlasRootContainsPopulation(tlasView);
    const bool retainVisible = allVisible && scratch.retainedAllVisible &&
                               query.visibleMappingVersion_ ==
                                   instanceMappingVersion_;
    if (!retainVisible)
    {
        scratch.visible.swap(scratch.previousVisible);
        tlasQuery(tlasView, params.minPix, scratch.visible, scratch.tlasStack);
    }

    const uint32_t nVis = uint32_t(scratch.visible.size());

    query.reused_ = query.walked_ = 0;
    if (query.instanceLayoutVersion_ != instanceLayoutVersion_)
    {
        query.rec_.clear();
        query.recCold_.clear();
        query.secondDep_.clear();
        query.overflowCounts_.clear();
        query.freeOverflowCounts_.clear();
        query.store_.clear();
        query.resolvedPayloadStore_.clear();
        query.resolvedErrorStore_.clear();
        query.resolvedRecords_.clear();
        scratch.renderRunsValid = false;
        query.used_ = query.garbage_ = 0;
        query.instanceLayoutVersion_ = instanceLayoutVersion_;
    }
    if (query.rec_.size() < instances_.size())
    {
        query.rec_.resize(instances_.size());
        query.recCold_.resize(instances_.size());
        if (!query.resolvedRecords_.empty())
            query.resolvedRecords_.resize(instances_.size(), 0);
        if (!query.secondDep_.empty())
            query.secondDep_.resize_uninitialized(instances_.size());
    }

    // How far the query envelope moved since the last call, added to this
    // view's odometer. One number for the whole frame; every record's validity
    // is then a single compare against it.
    const float4 qmn = dv.queryMin(), qmx = dv.queryMax();
    if (query.primed_)
    {
        const float4 dmn = max4(qmn - query.lastQmn_, query.lastQmn_ - qmn);
        const float4 dmx = max4(qmx - query.lastQmx_, query.lastQmx_ - qmx);
        query.travel_ += length3(max4(dmn, dmx));
    }
    query.lastQmn_ = qmn;
    query.lastQmx_ = qmx;
    query.primed_ = true;

    // Threshold and current-cut policy changes invalidate every result in
    // O(1). Projection-scale changes instead feed an odometer: each record
    // bounds how far any flip point can move per unit k, so gradual damped zoom
    // consumes its margin instead of voiding the whole cache.
    if (query.bar_ != params.threshold ||
        query.currentCutPolicy_ != params.currentCutPolicy)
    {
        ++query.epoch_;
        query.bar_ = params.threshold;
        query.currentCutPolicy_ = params.currentCutPolicy;
        query.kTravel_ = 0.0f;
    }
    else
        query.kTravel_ += std::fabs(dv.k - query.k_);
    query.k_ = dv.k;

    // The normal view-returning API owns its output inside this query. If the
    // complete visible stream is unchanged and the conservative global
    // validity budget still covers camera/zoom travel, those contiguous
    // buffers are already the exact answer. Avoid 10k record probes and 10k
    // tiny range appends just to reconstruct identical bytes.
    const float wholeTravel = query.travel_ - query.wholeTravel_;
    const float wholeKTravel = query.kTravel_ - query.wholeKTravel_;
    const double wholeInstanceTravel =
        instanceMotionTravelGlobal_ - query.wholeInstanceTravel_;
    const bool sameVisible = retainVisible ||
        (scratch.retainedVisible &&
        scratch.visible.size() == scratch.previousVisible.size() &&
        (scratch.visible.empty() ||
         std::memcmp(scratch.visible.data(), scratch.previousVisible.data(),
                     scratch.visible.size() * sizeof(VisibleItem)) == 0));
    const bool retainedAnswer = outResult.retainsExisting_ ||
                                (segmentedRender && scratch.renderRunsValid);
    if (retainedAnswer && usage == nullptr &&
        query.wholeReusable_ && sameVisible &&
        query.visibleMappingVersion_ == instanceMappingVersion_ &&
        query.wholeContentGeneration_ == frontierContentGeneration_ &&
        query.wholeEpoch_ == query.epoch_ &&
        double(wholeTravel + query.wholeMaxSlope_ * wholeKTravel) +
                wholeInstanceTravel <
            query.wholeMargin_)
    {
        query.reused_ = nVis;
        query.walked_ = 0;
        return;
    }

    if (segmentedRender)
    {
        scratch.renderRuns.clear();
        scratch.renderEntryCount = 0;
        if (query.resolvedRecords_.empty())
            query.resolvedRecords_.resize(instances_.size(), 0);
        if (query.resolvedPayloadStore_.size() < query.store_.size())
        {
            query.resolvedPayloadStore_.resize_uninitialized(
                query.store_.size());
            query.resolvedErrorStore_.resize_uninitialized(
                query.store_.size());
        }
    }

    bool patchOutput = !segmentedRender && outResult.retainsExisting_ &&
                       usage == nullptr &&
                       query.wholeReusable_ && sameVisible &&
                       query.wholeEpoch_ == query.epoch_;
    bool rebuildOutput = false;
    if (!patchOutput)
    {
        outResult.shared.clear();
        outResult.currentOnly.clear();
        outResult.idealOnly.clear();
        if (!segmentedRender) scratch.retainedVisible = false;
    }

    // The one safe moment to squeeze the slab: before any offset recorded this
    // pass could be moved out from under us.
    if (query.garbage_ > query.used_ / 2) query.compact();

    Worker& w = scratch.workers[0];
    w.work.clear();
    w.nodeStack.clear();
#ifdef FRONTIER_STATS
    w.stats = SelectionStats{};
#endif
    w.trackTouches = true;
    w.uniqueTouches = true;
    w.coalesceMountTreeDependencies = usage == nullptr;
    w.bar = params.threshold;
    w.barInv = params.threshold > 0.0f ? 1.0f / params.threshold : 0.0f;

    bool wholeReusable = usage == nullptr;
    float wholeMargin = FLT_MAX;
    float wholeMaxSlope = 0.0f;

    const auto recordUsage = [&](uint32_t slot)
    {
        if (usage) recordMountUsage(*usage, slot);
    };
    for (uint32_t i = 0; i < nVis; ++i)
    {
        const uint32_t instIdx = scratch.visible[i].instance();
        const uint8_t visibleMask = scratch.visible[i].mask();
        // Opted-in render-native actors are already batched at instance
        // granularity. Once the TLAS proves that their root intersects the
        // frustum, retain the whole LOD cut and let the renderer clip the
        // small boundary actor. Ordinary/static instances and the handle API
        // keep exact descendant culling. The Instance record is fetched only
        // for the uncommon nonzero-mask shell.
        const uint8_t mask =
            visibleMask != 0 && segmentedRender &&
                    instances_[instIdx].renderAsUnit()
                ? 0
                : visibleMask;
        SpatialQuery::Rec& r = query.rec_[instIdx];
        const bool overflow = frontierCountsOverflow(r.counts);
        const SpatialQuery::OverflowCounts* fullCounts =
            overflow ? &query.overflowCounts_[frontierOverflowIndex(r.counts)]
                     : nullptr;
        const uint32_t depCount = overflow ? fullCounts->dependencies
                                           : frontierDependencyCount(r.counts);

        // Everything the record was taken under, re-checked, in one cache
        // line. `mask == 0` is the frustum condition: this instance is wholly
        // inside, so no plane was tested anywhere within it and camera
        // rotation cannot matter. `travel_ < validUntil` is the margin.
        bool hit = mask == 0 &&
                   query.travel_ + instanceUniformTravel_ +
                           instanceMotionTravel_[instIdx] +
                           r.kSlope * query.kTravel_ <
                       r.validUntil &&
                   r.epoch == query.epoch_ &&
                   r.frontierVersion == instanceFrontierVersions_[instIdx];
        if (hit && depCount != 0)
            hit = dependencyMatches(r.depSlot, r.depVersion);
        if (hit && depCount == 2)
        {
            const SpatialQuery::SecondDep& dep = query.secondDep_[instIdx];
            hit = dependencyMatches(dep.slot, dep.version);
        }
        // A mounted-tree dependency does not enumerate the physical mounts
        // needed by streaming usage feedback. Re-walk that uncommon
        // usage tracking remains exact.
        if (hit && usage &&
            ((depCount != 0 && (r.depSlot & kMountTreeDependency) != 0) ||
             (depCount == 2 &&
              (query.secondDep_[instIdx].slot & kMountTreeDependency) != 0)))
            hit = false;

        if (hit)
        {
            if (depCount != 0) recordUsage(r.depSlot);
            if (depCount == 2) recordUsage(query.secondDep_[instIdx].slot);
            // The whole saving is the walk that did not happen. Copying the
            // recorded entries out is ~1.5% of the call at 80k instances, and
            // handing back a descriptor instead measured no better while
            // costing the caller an indirection: see SpatialQuery.
            const uint32_t shared = overflow ? fullCounts->shared
                                             : frontierCount(r.counts, 0);
            const uint32_t current = overflow ? fullCounts->current
                                              : frontierCount(r.counts, 1);
            const uint32_t ideal = overflow ? fullCounts->ideal
                                            : frontierCount(r.counts, 2);
            if (segmentedRender)
            {
                const uint32_t renderCount = shared + current;
                if (renderCount != 0)
                {
                    if (!query.resolvedRecords_[instIdx])
                    {
                        const std::span<const FrontierEntry> source(
                            query.store_.data() + r.begin, renderCount);
                        const std::span<UserPayload> payloads(
                            query.resolvedPayloadStore_.data() + r.begin,
                            renderCount);
                        const std::span<uint8_t> errors(
                            query.resolvedErrorStore_.data() + r.begin,
                            renderCount);
                        const bool resolved = resolveRenderLeaves(
                            FrontierCutView{source, {}}, payloads, errors);
                        FRONTIER_ASSERT(resolved,
                                        "render leaf resolution size mismatch");
                        (void)resolved;
                        query.resolvedRecords_[instIdx] = 1;
                    }
                    scratch.renderRuns.push_back(
                        {r.begin, renderCount,
                         publicInstanceId(InstanceId(instIdx))});
                    scratch.renderEntryCount += renderCount;
                }
            }
            else if (!patchOutput && !rebuildOutput)
            {
                SpatialQuery::RecCold& cold = query.recCold_[instIdx];
                cold.output[0] = outResult.shared.count();
                cold.output[1] = outResult.currentOnly.count();
                cold.output[2] = outResult.idealOnly.count();
                const FrontierEntry* entries = query.store_.data() + r.begin;
                outResult.shared.pushRange(entries, shared);
                outResult.currentOnly.pushRange(entries + shared, current);
                outResult.idealOnly.pushRange(entries + shared + current, ideal);
            }
            const float remaining =
                r.validUntil - query.travel_ -
                instanceUniformTravel_ -
                instanceMotionTravel_[instIdx] -
                r.kSlope * query.kTravel_;
            wholeMargin = std::min(wholeMargin, remaining);
            wholeMaxSlope = std::max(wholeMaxSlope, r.kSlope);
            ++query.reused_;
            continue;
        }

        // ---- walk it ----
        // Hits deliberately never fetch the 80-byte Instance record.
        const Instance& inst = instances_[instIdx];
        w.frontierBuffer.clear();
        w.result = makeSink(w.frontierBuffer);
        w.touched.clear();
        w.margin = FLT_MAX;
        w.maxError = 0.0f;
        w.trackMargin = true;
        if (flatInstanceCount_ != 0 &&
            instanceFlatSlots_[instIdx] != kInvalidIndex)
        {
            if (instances_[instIdx].hasZeroErrorRoot())
                runZeroErrorTlasFlatInstance(instIdx, w);
            else
                runTlasFlatInstance(instIdx, tlasView, w);
        }
        else
        {
            if (instanceOrientations_.empty())
                runTlasRootInstance(instIdx, tlasView, params, mask, w);
            else
                runOrientedTlasRootInstance(instIdx, tlasView, params,
                                            mask, w);
        }
        w.trackMargin = false;
        for (const uint32_t slot : w.touched) recordUsage(slot);

        const uint32_t nShared = uint32_t(w.frontierBuffer.shared.size());
        const uint32_t nCurrent = uint32_t(w.frontierBuffer.currentOnly.size());
        const uint32_t nIdeal = uint32_t(w.frontierBuffer.idealOnly.size());
        const uint32_t n = nShared + nCurrent + nIdeal;
        const bool eligible = mask == 0 &&
                              w.touched.size() <= SpatialQuery::kMaxDeps;
        SpatialQuery::RecCold& cold = query.recCold_[instIdx];
        const uint32_t oldShared = overflow ? fullCounts->shared
                                            : frontierCount(r.counts, 0);
        const uint32_t oldCurrent = overflow ? fullCounts->current
                                             : frontierCount(r.counts, 1);
        const uint32_t oldIdeal = overflow ? fullCounts->ideal
                                           : frontierCount(r.counts, 2);
        if (cold.capacity < n)
        {
            query.garbage_ += cold.capacity;
            if (size_t(query.used_) + n > query.store_.size())
            {
                query.store_.resize_uninitialized(
                    std::max<size_t>(size_t(query.used_) + n, query.store_.size() * 2 + 256));
                if (segmentedRender || !query.resolvedPayloadStore_.empty())
                {
                    query.resolvedPayloadStore_.resize_uninitialized(
                        query.store_.size());
                    query.resolvedErrorStore_.resize_uninitialized(
                        query.store_.size());
                }
            }
            r.begin = query.used_;
            cold.capacity = n;
            query.used_ += n;
        }
        if (eligible && w.touched.size() == 2 && query.secondDep_.empty())
            query.secondDep_.resize_uninitialized(instances_.size());
        const uint32_t oldCounts = r.counts;
        if (nShared <= 0x3ffu && nCurrent <= 0x3ffu && nIdeal <= 0x3ffu)
        {
            if (frontierCountsOverflow(oldCounts))
                query.freeOverflowCounts_.push_back(
                    frontierOverflowIndex(oldCounts));
            r.counts = packFrontierCounts(nShared, nCurrent, nIdeal,
                                          eligible
                                              ? uint32_t(w.touched.size())
                                              : 0u);
        }
        else
        {
            uint32_t overflowIndex;
            if (frontierCountsOverflow(oldCounts))
                overflowIndex = frontierOverflowIndex(oldCounts);
            else if (!query.freeOverflowCounts_.empty())
            {
                overflowIndex = query.freeOverflowCounts_.back();
                query.freeOverflowCounts_.resize_uninitialized(
                    query.freeOverflowCounts_.size() - 1);
            }
            else
            {
                overflowIndex = uint32_t(query.overflowCounts_.size());
                query.overflowCounts_.emplace_back();
            }
            query.overflowCounts_[overflowIndex] = {
                nShared, nCurrent, nIdeal,
                eligible ? uint32_t(w.touched.size()) : 0u};
            r.counts = packFrontierOverflow(overflowIndex);
        }
        FrontierEntry* dst = query.store_.data() + r.begin;
        if (nShared)
            std::memcpy(dst, w.frontierBuffer.shared.data(),
                        size_t(nShared) * sizeof(FrontierEntry));
        if (nCurrent)
            std::memcpy(dst + nShared, w.frontierBuffer.currentOnly.data(),
                        size_t(nCurrent) * sizeof(FrontierEntry));
        if (nIdeal)
            std::memcpy(dst + nShared + nCurrent, w.frontierBuffer.idealOnly.data(),
                        size_t(nIdeal) * sizeof(FrontierEntry));

        if (!query.resolvedRecords_.empty())
        {
            query.resolvedRecords_[instIdx] = 0;
            if (segmentedRender && nShared + nCurrent != 0)
            {
                const std::span<const FrontierEntry> source(
                    dst, nShared + nCurrent);
                const std::span<UserPayload> payloads(
                    query.resolvedPayloadStore_.data() + r.begin,
                    nShared + nCurrent);
                const std::span<uint8_t> errors(
                    query.resolvedErrorStore_.data() + r.begin,
                    nShared + nCurrent);
                const bool resolved = resolveRenderLeaves(
                    FrontierCutView{source, {}}, payloads, errors);
                FRONTIER_ASSERT(resolved,
                                "render leaf resolution size mismatch");
                (void)resolved;
                query.resolvedRecords_[instIdx] = 1;
            }
        }

        if (eligible)
        {
            // The margin is measured in the instance's own space, where the
            // walk measures distances; the odometer runs in world space, so
            // scale it across. Anything non-finite (nothing was ever decided,
            // so nothing can flip) becomes an unbounded budget.
            const float m = w.margin * inst.scale;
            r.kSlope = w.maxError * inst.scale / params.threshold;
            const float consumed = query.travel_ +
                                   instanceUniformTravel_ +
                                   instanceMotionTravel_[instIdx] +
                                   r.kSlope * query.kTravel_;
            r.validUntil = m >= FLT_MAX - consumed ? FLT_MAX : consumed + m;
            r.epoch = query.epoch_;
            r.frontierVersion = instanceFrontierVersions_[instIdx];
            if (!w.touched.empty())
            {
                r.depSlot = w.touched[0];
                r.depVersion = dependencyVersion(w.touched[0]);
            }
            if (w.touched.size() == 2)
            {
                SpatialQuery::SecondDep& dep = query.secondDep_[instIdx];
                dep.slot = w.touched[1];
                dep.version = dependencyVersion(w.touched[1]);
            }
        }
        else
        {
            r.validUntil = 0.0f;
        }

        if (eligible)
        {
            const float remaining =
                r.validUntil - query.travel_ -
                instanceUniformTravel_ -
                instanceMotionTravel_[instIdx] -
                r.kSlope * query.kTravel_;
            wholeMargin = std::min(wholeMargin, remaining);
            wholeMaxSlope = std::max(wholeMaxSlope, r.kSlope);
        }
        else
            wholeReusable = false;

        if (patchOutput)
        {
            if (oldShared == nShared && oldCurrent == nCurrent &&
                oldIdeal == nIdeal)
            {
                if (nShared)
                    std::memcpy(scratch.output.shared.data() + cold.output[0],
                                w.frontierBuffer.shared.data(),
                                size_t(nShared) * sizeof(FrontierEntry));
                if (nCurrent)
                    std::memcpy(
                        scratch.output.currentOnly.data() + cold.output[1],
                        w.frontierBuffer.currentOnly.data(),
                        size_t(nCurrent) * sizeof(FrontierEntry));
                if (nIdeal)
                    std::memcpy(scratch.output.idealOnly.data() + cold.output[2],
                                w.frontierBuffer.idealOnly.data(),
                                size_t(nIdeal) * sizeof(FrontierEntry));
            }
            else
            {
                patchOutput = false;
                rebuildOutput = true;
            }
        }
        else if (!segmentedRender && !rebuildOutput)
        {
            // From the walk buffer, not from the slab: same bytes, still hot.
            cold.output[0] = outResult.shared.count();
            cold.output[1] = outResult.currentOnly.count();
            cold.output[2] = outResult.idealOnly.count();
            outResult.shared.pushRange(w.frontierBuffer.shared.data(), nShared);
            outResult.currentOnly.pushRange(w.frontierBuffer.currentOnly.data(), nCurrent);
            outResult.idealOnly.pushRange(w.frontierBuffer.idealOnly.data(), nIdeal);
        }
        if (segmentedRender && nShared + nCurrent != 0)
        {
            scratch.renderRuns.push_back(
                {r.begin, nShared + nCurrent,
                 publicInstanceId(InstanceId(instIdx))});
            scratch.renderEntryCount += nShared + nCurrent;
        }
        ++query.walked_;
    }

    if (rebuildOutput)
    {
        outResult.shared.clear();
        outResult.currentOnly.clear();
        outResult.idealOnly.clear();
        for (const VisibleItem visible : scratch.visible)
        {
            const uint32_t instIdx = visible.instance();
            const SpatialQuery::Rec& r = query.rec_[instIdx];
            const bool overflow = frontierCountsOverflow(r.counts);
            const SpatialQuery::OverflowCounts* fullCounts =
                overflow
                    ? &query.overflowCounts_[frontierOverflowIndex(r.counts)]
                    : nullptr;
            const uint32_t shared = overflow ? fullCounts->shared
                                             : frontierCount(r.counts, 0);
            const uint32_t current = overflow ? fullCounts->current
                                              : frontierCount(r.counts, 1);
            const uint32_t ideal = overflow ? fullCounts->ideal
                                            : frontierCount(r.counts, 2);
            SpatialQuery::RecCold& cold = query.recCold_[instIdx];
            cold.output[0] = outResult.shared.count();
            cold.output[1] = outResult.currentOnly.count();
            cold.output[2] = outResult.idealOnly.count();
            const FrontierEntry* entries = query.store_.data() + r.begin;
            outResult.shared.pushRange(entries, shared);
            outResult.currentOnly.pushRange(entries + shared, current);
            outResult.idealOnly.pushRange(entries + shared + current, ideal);
        }
    }

    w.result = FrontierResultSink{};
    query.wholeReusable_ = wholeReusable;
    query.wholeMargin_ = wholeMargin;
    query.wholeMaxSlope_ = wholeMaxSlope;
    query.wholeTravel_ = query.travel_;
    query.wholeKTravel_ = query.kTravel_;
    query.wholeInstanceTravel_ = instanceMotionTravelGlobal_;
    query.wholeEpoch_ = query.epoch_;
    query.wholeContentGeneration_ = frontierContentGeneration_;
    scratch.retainedVisible = outResult.retainsExisting_ || segmentedRender;
    scratch.retainedAllVisible =
        (outResult.retainsExisting_ || segmentedRender) && allVisible;
    if (segmentedRender) scratch.renderRunsValid = true;
    query.visibleMappingVersion_ = instanceMappingVersion_;
#ifdef FRONTIER_STATS
    query.stats_ = w.stats;
#endif
}

void SpatialQuery::selectFrontier(const SpatialDatabase& database, const Camera& camera,
                             const SelectionParams& params, FrontierResultSink& outResult)
{
    FRONTIER_CHECK(database_ == nullptr || database_ == &database,
               "SpatialQuery::selectFrontier: SpatialQuery belongs to another SpatialDatabase; call reset()");
    FRONTIER_CHECK(params.threshold > 0.0f &&
                       std::isfinite(params.threshold),
                   "SpatialQuery::selectFrontier: threshold must be finite "
                   "and positive");
    FRONTIER_CHECK(params.minPix >= 0.0f && std::isfinite(params.minPix),
                   "SpatialQuery::selectFrontier: minPix must be finite and "
                   "non-negative");
    FRONTIER_CHECK(
        params.currentCutPolicy ==
                CurrentCutPolicy::PreferReadyDescendants ||
            params.currentCutPolicy ==
                CurrentCutPolicy::PreferReadyAncestors,
        "SpatialQuery::selectFrontier: invalid current-cut policy");
    FRONTIER_CHECK(validSelectionCamera(camera),
                   "SpatialQuery::selectFrontier: invalid camera");
    database_ = &database;
    SpatialQuery* usage = mountUsageEnabled_ ? this : nullptr;
    // A TLAS-only one-node object has no BLAS walk to cache. Recording and
    // copying one entry per instance only adds work, so reuse-enabled queries
    // deliberately take the same direct path as uncached queries here.
    const bool hierarchyCanBenefitFromReuse =
        database.flatInstanceCount_ != database.liveInstances_.size();
    if (reuseEnabled_ && hierarchyCanBenefitFromReuse)
        database.selectFrontierCached(camera, params, *this, usage, outResult);
    else
        database.selectFrontierUncached(camera, params, *this, usage, outResult);
}

void SpatialQuery::selectFrontier(const SpatialDatabase& database, const Camera& camera,
                             const SelectionParams& params,
                             FrontierResult& outResult)
{
    FrontierResultSink sink = SpatialDatabase::makeSink(outResult.buffers_);
    selectFrontier(database, camera, params, sink);
    outResult.sync();
}

FrontierResultView SpatialQuery::selectFrontier(const SpatialDatabase& database, const Camera& camera,
                                      const SelectionParams& params)
{
    FRONTIER_CHECK(database_ == nullptr || database_ == &database,
               "SpatialQuery::selectFrontier: SpatialQuery belongs to another SpatialDatabase; call reset()");
    FRONTIER_CHECK(params.threshold > 0.0f &&
                       std::isfinite(params.threshold),
                   "SpatialQuery::selectFrontier: threshold must be finite "
                   "and positive");
    FRONTIER_CHECK(params.minPix >= 0.0f && std::isfinite(params.minPix),
                   "SpatialQuery::selectFrontier: minPix must be finite and "
                   "non-negative");
    FRONTIER_CHECK(
        params.currentCutPolicy ==
                CurrentCutPolicy::PreferReadyDescendants ||
            params.currentCutPolicy ==
                CurrentCutPolicy::PreferReadyAncestors,
        "SpatialQuery::selectFrontier: invalid current-cut policy");
    FRONTIER_CHECK(validSelectionCamera(camera),
                   "SpatialQuery::selectFrontier: invalid camera");

    QueryScratch& scratch = *scratch_;
    const bool memoEligible =
        reuseEnabled_ && !mountUsageEnabled_ && damper_.halfLife() == 0.0f &&
        database.flatInstanceCount_ != database.liveInstances_.size();
    const bool stableScene = scratch.haveLastScene &&
        scratch.lastSceneMappingVersion == database.instanceMappingVersion_ &&
        scratch.lastSceneContentGeneration ==
            database.frontierContentGeneration_ &&
        scratch.lastSceneSpatialVersion == database.instanceSpatialVersion_;
    if (memoEligible)
    {
        for (QueryScratch::ViewMemo& memo : scratch.viewMemo)
        {
            if (!memo.valid ||
                memo.mappingVersion != database.instanceMappingVersion_ ||
                memo.contentGeneration !=
                    database.frontierContentGeneration_ ||
                memo.spatialVersion != database.instanceSpatialVersion_ ||
                !sameMemoCamera(memo.camera, camera) ||
                !sameMemoParams(memo.params, params))
                continue;
            database_ = &database;
            reused_ = memo.visibleCount;
            walked_ = 0;
#ifdef FRONTIER_STATS
            stats_ = SelectionStats{};
#endif
            memo.lastUsed = ++scratch.memoClock;
            scratch.lastSceneMappingVersion = database.instanceMappingVersion_;
            scratch.lastSceneContentGeneration =
                database.frontierContentGeneration_;
            scratch.lastSceneSpatialVersion = database.instanceSpatialVersion_;
            scratch.haveLastScene = true;
            return memo.output.view();
        }
    }

    detail::FrontierBuffers& output = scratch_->output;
    FrontierResultSink sink = SpatialDatabase::makeSink(output, true);
    selectFrontier(database, camera, params, sink);

    if (memoEligible && stableScene)
    {
        QueryScratch::ViewMemo* destination = nullptr;
        for (QueryScratch::ViewMemo& memo : scratch.viewMemo)
            if (memo.lastUsed != 0 &&
                memo.mappingVersion == database.instanceMappingVersion_ &&
                memo.contentGeneration ==
                    database.frontierContentGeneration_ &&
                memo.spatialVersion == database.instanceSpatialVersion_ &&
                sameMemoCamera(memo.camera, camera) &&
                sameMemoParams(memo.params, params))
            {
                destination = &memo;
                break;
            }
        if (!destination)
            destination = scratch.viewMemo[0].lastUsed == 0
                              ? &scratch.viewMemo[0]
                              : (scratch.viewMemo[1].lastUsed == 0
                                     ? &scratch.viewMemo[1]
                                     : (scratch.viewMemo[0].lastUsed <=
                                                scratch.viewMemo[1].lastUsed
                                            ? &scratch.viewMemo[0]
                                            : &scratch.viewMemo[1]));
        const bool recurring =
            destination->lastUsed != 0 &&
            destination->mappingVersion == database.instanceMappingVersion_ &&
            destination->contentGeneration ==
                database.frontierContentGeneration_ &&
            destination->spatialVersion == database.instanceSpatialVersion_ &&
            sameMemoCamera(destination->camera, camera) &&
            sameMemoParams(destination->params, params);
        if (recurring)
        {
            destination->output = output;
            destination->visibleCount = reused_ + walked_;
            destination->valid = true;
        }
        else
        {
            destination->camera = camera;
            destination->params = params;
            destination->mappingVersion = database.instanceMappingVersion_;
            destination->contentGeneration =
                database.frontierContentGeneration_;
            destination->spatialVersion = database.instanceSpatialVersion_;
            destination->valid = false;
        }
        destination->lastUsed = ++scratch.memoClock;
    }
    scratch.lastSceneMappingVersion = database.instanceMappingVersion_;
    scratch.lastSceneContentGeneration = database.frontierContentGeneration_;
    scratch.lastSceneSpatialVersion = database.instanceSpatialVersion_;
    scratch.haveLastScene = memoEligible;
    return output.view();
}

RenderFrontierView SpatialQuery::selectRenderFrontier(
    const SpatialDatabase& database, const Camera& camera,
    const SelectionParams& params)
{
    QueryScratch& scratch = *scratch_;
    const bool segmented =
        reuseEnabled_ &&
        database.flatInstanceCount_ != database.liveInstances_.size();
    if (segmented)
    {
        scratch.segmentedRenderRequested = true;
        struct SegmentedRequestGuard
        {
            QueryScratch& scratch;
            ~SegmentedRequestGuard()
            {
                scratch.segmentedRenderRequested = false;
            }
        } guard{scratch};
        FrontierResultSink discard;
        selectFrontier(database, camera, params, discard);
        return {
            std::span<const UserPayload>(resolvedPayloadStore_.data(),
                                         resolvedPayloadStore_.size()),
            std::span<const uint8_t>(resolvedErrorStore_.data(),
                                     resolvedErrorStore_.size()),
            std::span<const RenderFrontierRun>(scratch.renderRuns.data(),
                                               scratch.renderRuns.size()),
            scratch.renderEntryCount};
    }

    // Reuse-disabled and all-flat databases do not retain per-instance
    // hierarchical records. Materialize one contiguous fallback run so the
    // renderer consumes the same scatter/gather interface in every mode.
    const FrontierResultView frontier = selectFrontier(database, camera, params);
    const size_t count = frontier.currentSize();
    scratch.resolvedPayloadCurrent.resize_uninitialized(count);
    scratch.resolvedErrorCurrent.resize_uninitialized(count);
    const bool resolved = database.resolveRenderLeaves(
        frontier.current(),
        std::span<UserPayload>(scratch.resolvedPayloadCurrent.data(), count),
        std::span<uint8_t>(scratch.resolvedErrorCurrent.data(), count));
    FRONTIER_ASSERT(resolved,
                    "render frontier resolution size mismatch");
    (void)resolved;
    scratch.renderRuns.clear();
    if (count != 0)
    {
        FRONTIER_CHECK(count <= UINT32_MAX,
                       "render frontier exceeds 32-bit run capacity");
        uint32_t index = 0;
        for (const FrontierEntry& entry : frontier.current())
        {
            const InstanceId instance = entry.instance();
            if (scratch.renderRuns.empty() ||
                scratch.renderRuns.back().instance != instance)
                scratch.renderRuns.push_back({index, 1, instance});
            else
                ++scratch.renderRuns.back().count;
            ++index;
        }
    }
    scratch.renderEntryCount = count;
    scratch.renderRunsValid = false;
    return {
        std::span<const UserPayload>(scratch.resolvedPayloadCurrent.data(),
                                     count),
        std::span<const uint8_t>(scratch.resolvedErrorCurrent.data(), count),
        std::span<const RenderFrontierRun>(scratch.renderRuns.data(),
                                           scratch.renderRuns.size()),
        count};
}

} // namespace frontier
