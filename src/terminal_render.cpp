#include "frontier/spatial_database.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace frontier {

struct TerminalRenderQuery::Impl
{
    struct Range
    {
        uint32_t begin = 0;
        uint32_t count = 0;
    };

    struct Plan
    {
        uint32_t generation = 0;
        bool eligible = false;
        std::vector<UserPayload> payloads;
        std::vector<Range> ranges;
    };

    const SpatialDatabase* database = nullptr;
    std::vector<uint32_t> visible;
    std::vector<uint32_t> tlasStack;
    std::vector<uint32_t> nodeStack;
    std::vector<Plan> plans;
    std::vector<TerminalRenderRun> runs;
    size_t leafCount = 0;
};

namespace {

inline bool finitePosition(float4 p)
{
    return std::isfinite(p.x) && std::isfinite(p.y) &&
           std::isfinite(p.z);
}

inline bool validCamera(const Camera& camera)
{
    if (!finitePosition(camera.pos) || !(camera.k > 0.0f) ||
        !std::isfinite(camera.k) || camera.viewMask == 0 ||
        !finitePosition(camera.envLo) || !finitePosition(camera.envHi))
        return false;
    for (uint32_t p = 0; p < 6; ++p)
    {
        const float4 plane = camera.frustum.plane[p];
        if (!finitePosition(plane) || !std::isfinite(plane.w)) return false;
    }
    return true;
}

inline bool identityYaw(YawRotation yaw)
{
    return yaw.cosine == 1.0f && yaw.sine == 0.0f;
}

inline bool validYaw(YawRotation yaw)
{
    if (!std::isfinite(yaw.cosine) || !std::isfinite(yaw.sine)) return false;
    const float lengthSq = yaw.cosine * yaw.cosine + yaw.sine * yaw.sine;
    return std::fabs(lengthSq - 1.0f) <= 1.0e-4f;
}

inline bool finiteNonEmptyBounds(const AABB& bounds)
{
    return !bounds.isEmpty() && finitePosition(bounds.mn) &&
           finitePosition(bounds.mx);
}

inline uint32_t packItem(uint32_t value, uint8_t mask)
{
    return (value & kInstanceIdMask) |
           (uint32_t(mask & kAllPlanes) << kInstanceIdBits);
}

inline uint32_t itemValue(uint32_t packed)
{
    return packed & kInstanceIdMask;
}

inline uint8_t itemMask(uint32_t packed)
{
    return uint8_t(packed >> kInstanceIdBits);
}

inline uint32_t packInstanceError(InstanceId instance, uint8_t error)
{
    return (instance & kInstanceIdMask) |
           (uint32_t(error) << kInstanceIdBits);
}

} // namespace

TerminalRenderQuery::TerminalRenderQuery() : impl_(std::make_unique<Impl>()) {}
TerminalRenderQuery::~TerminalRenderQuery() = default;
TerminalRenderQuery::TerminalRenderQuery(TerminalRenderQuery&&) noexcept =
    default;
TerminalRenderQuery& TerminalRenderQuery::operator=(
    TerminalRenderQuery&&) noexcept = default;

void TerminalRenderQuery::reset()
{
    impl_ = std::make_unique<Impl>();
}

size_t TerminalRenderQuery::bytes() const
{
    size_t bytes = impl_->visible.capacity() * sizeof(uint32_t) +
                   impl_->tlasStack.capacity() * sizeof(uint32_t) +
                   impl_->nodeStack.capacity() * sizeof(uint32_t) +
                   impl_->runs.capacity() * sizeof(TerminalRenderRun) +
                   impl_->plans.capacity() * sizeof(Impl::Plan);
    for (const Impl::Plan& plan : impl_->plans)
        bytes += plan.payloads.capacity() * sizeof(UserPayload) +
                 plan.ranges.capacity() * sizeof(Impl::Range);
    return bytes;
}

TerminalRenderView TerminalRenderQuery::select(
    const SpatialDatabase& database, const Camera& camera,
    float errorThreshold, bool coarsenRenderUnits)
{
    return select(database, camera, {}, errorThreshold, coarsenRenderUnits);
}

TerminalRenderView TerminalRenderQuery::select(
    const SpatialDatabase& database, const Camera& camera,
    std::span<const TerminalInstanceBatch> batches, float errorThreshold,
    bool coarsenRenderUnits)
{
    FRONTIER_CHECK(validCamera(camera),
                   "TerminalRenderQuery::select: invalid camera");
    FRONTIER_CHECK(errorThreshold > 0.0f &&
                       std::isfinite(errorThreshold),
                   "TerminalRenderQuery::select: error threshold must be "
                   "finite and positive");
    FRONTIER_CHECK(impl_->database == nullptr || impl_->database == &database,
                   "TerminalRenderQuery::select: query belongs to another "
                   "SpatialDatabase; call reset()");
    impl_->database = &database;
    impl_->runs.clear();
    impl_->leafCount = 0;

    FRONTIER_CHECK(!database.tlasDirty_ && database.pendingMoves_.empty() &&
                       database.tlasItemsTmp_.empty(),
                   "TerminalRenderQuery::select: call applyUpdates() after "
                   "database changes");
    if (database.tlasRoot_ < 0 && batches.empty())
        return {{}, 0};

    const Camera view =
        database.tlasGlobalOffset_.x == 0.0f &&
                database.tlasGlobalOffset_.y == 0.0f &&
                database.tlasGlobalOffset_.z == 0.0f
            ? camera
            : toLocal(camera, database.tlasGlobalOffset_, 1.0f);

    // Retain the production TLAS layout but keep this query's scratch compact:
    // both a visible hit and a stack item are one 24-bit id plus six mask bits.
    std::vector<uint32_t>& visible = impl_->visible;
    std::vector<uint32_t>& stack = impl_->tlasStack;
    visible.clear();
    if (database.tlasRoot_ < 0)
    {
        // A database may exist only to own immutable definitions consumed by
        // external actor batches.
    }
    else if (camera.viewMask == ~0u &&
             database.tlasRootContainsPopulation(view))
    {
        visible.reserve(database.liveInstances_.size());
        for (const InstanceId dense : database.liveInstances_)
            visible.push_back(packItem(dense, 0));
    }
    else
    {
        stack.clear();
        stack.push_back(packItem(uint32_t(database.tlasRoot_), kAllPlanes));
        while (!stack.empty())
        {
            const uint32_t item = stack.back();
            stack.pop_back();
            const uint32_t nodeIndex = itemValue(item);
            const uint8_t inMask = itemMask(item);
            const SpatialDatabase::TlasNode& node =
                database.tlasNodes_[nodeIndex];
            uint8_t outMasks[kWide];
            uint32_t survivors =
                inMask ? testWideAabb(node.bounds, view.frustum, inMask,
                                      outMasks) &
                             node.validLanes()
                       : node.validLanes();
            if (!survivors) continue;

            if (camera.viewMask != ~0u)
            {
                const SpatialDatabase::TlasMeta& meta =
                    database.tlasMeta_[nodeIndex];
                for (uint32_t lane = 0; lane < kWide; ++lane)
                    if (!(meta.laneMask[lane] & camera.viewMask))
                        survivors &= ~(1u << lane);
                if (!survivors) continue;
            }

            while (survivors)
            {
                const uint32_t lane =
                    uint32_t(std::countr_zero(survivors));
                survivors &= survivors - 1;
                const int32_t child = node.child[lane];
                const uint8_t mask = inMask ? outMasks[lane] : uint8_t(0);
                if (child >= 0)
                {
                    stack.push_back(packItem(uint32_t(child), mask));
                    continue;
                }

                const InstanceId dense = InstanceId(~child);
                uint8_t exactMask = mask;
                if (database.instanceTlasLoose_[dense] && exactMask != 0 &&
                    testAabb(database.instances_[dense].worldBox,
                             view.frustum, exactMask) == CullState::Outside)
                    continue;
                visible.push_back(packItem(dense, exactMask));
            }
        }
    }

    if (impl_->plans.size() < database.subtrees_.size())
        impl_->plans.resize(database.subtrees_.size());

    const auto buildPlan = [&](uint32_t definition) -> Impl::Plan&
    {
        Impl::Plan& plan = impl_->plans[definition];
        const SpatialDatabase::SubtreeDefinitionRt& runtime =
            database.subtrees_[definition];
        if (plan.generation == runtime.generation) return plan;

        plan = Impl::Plan{};
        plan.generation = runtime.generation;
        const detail::SubtreeView& subtree = runtime.view;
        plan.eligible = subtree.valid();
        uint32_t terminalCount = 0;
        for (uint32_t node = 1; node < subtree.packedNodeCount(); ++node)
        {
            if (subtree.isMountable(node)) plan.eligible = false;
            if (subtree.childCount(node) == 0)
            {
                ++terminalCount;
                if (subtree.geometricError_[node] != 0.0f)
                    plan.eligible = false;
            }
        }
        if (!plan.eligible) return plan;

        struct BuildItem
        {
            uint32_t node;
            bool exit;
        };
        plan.payloads.reserve(terminalCount);
        plan.ranges.resize(subtree.packedNodeCount());
        std::vector<BuildItem> buildStack;
        buildStack.reserve(subtree.nodeCount());
        buildStack.push_back({0, false});
        while (!buildStack.empty())
        {
            const BuildItem item = buildStack.back();
            buildStack.pop_back();
            Impl::Range& range = plan.ranges[item.node];
            if (item.exit)
            {
                range.count = uint32_t(plan.payloads.size()) - range.begin;
                continue;
            }

            range.begin = uint32_t(plan.payloads.size());
            buildStack.push_back({item.node, true});
            const uint32_t first = subtree.wideOffset(item.node);
            const uint32_t blocks = subtree.wideBlockCount(item.node);
            for (uint32_t b = 0; b < blocks; ++b)
            {
                const uint32_t block = first + b;
                const detail::WideBlock& children = subtree.wide_[block];
                const uint32_t lanes = subtree.blockMask_[block];
                uint32_t leaves = detail::blockLeafLanes(lanes);
                while (leaves)
                {
                    const uint32_t lane =
                        uint32_t(std::countr_zero(leaves));
                    leaves &= leaves - 1;
                    const uint32_t child = children.child[lane];
                    plan.ranges[child] = {
                        uint32_t(plan.payloads.size()), 1};
                    plan.payloads.push_back(detail::decodePayload(
                        subtree.payload_[child]));
                }
                uint32_t inner = detail::blockValidLanes(lanes) &
                                 ~detail::blockLeafLanes(lanes);
                while (inner)
                {
                    const uint32_t lane =
                        uint32_t(std::countr_zero(inner));
                    inner &= inner - 1;
                    buildStack.push_back({children.child[lane], false});
                }
            }
        }
        return plan;
    };

    const auto appendRun = [&](const UserPayload* payloads, uint32_t count,
                               uint32_t instanceAndError)
    {
        if (count == 0) return;
        if (!impl_->runs.empty())
        {
            TerminalRenderRun& previous = impl_->runs.back();
            if (previous.instanceAndError == instanceAndError &&
                previous.payloads + previous.count == payloads)
            {
                previous.count += count;
                impl_->leafCount += count;
                return;
            }
        }
        impl_->runs.push_back({payloads, count, instanceAndError});
        impl_->leafCount += count;
    };

    const auto appendDefinition = [&](Impl::Plan& plan,
                                      const detail::SubtreeView& subtree,
                                      const Camera& local, uint8_t rootMask,
                                      uint32_t instanceWord)
    {
        std::vector<uint32_t>& nodeStack = impl_->nodeStack;
        nodeStack.clear();
        nodeStack.push_back(packItem(0, rootMask));
        while (!nodeStack.empty())
        {
            const uint32_t item = nodeStack.back();
            nodeStack.pop_back();
            const uint32_t node = itemValue(item);
            const uint8_t nodeMask = itemMask(item);
            if (nodeMask == 0)
            {
                const Impl::Range range = plan.ranges[node];
                appendRun(plan.payloads.data() + range.begin, range.count,
                          instanceWord);
                continue;
            }

            const uint32_t first = subtree.wideOffset(node);
            const uint32_t blocks = subtree.wideBlockCount(node);
            for (uint32_t b = 0; b < blocks; ++b)
            {
                const uint32_t block = first + b;
                const detail::WideBlock& children = subtree.wide_[block];
                const uint32_t lanes = subtree.blockMask_[block];
                uint8_t outMasks[kWide];
                const uint32_t survivors =
                    testWideAabb(children.bounds, local.frustum, nodeMask,
                                 outMasks) &
                    detail::blockValidLanes(lanes);
                uint32_t leaves =
                    survivors & detail::blockLeafLanes(lanes);
                while (leaves)
                {
                    const uint32_t lane =
                        uint32_t(std::countr_zero(leaves));
                    leaves &= leaves - 1;
                    const Impl::Range terminalRange =
                        plan.ranges[children.child[lane]];
                    FRONTIER_ASSERT(terminalRange.count == 1,
                                    "terminal plan leaf is missing");
                    appendRun(plan.payloads.data() + terminalRange.begin, 1,
                              instanceWord);
                }
                uint32_t inner =
                    survivors & ~detail::blockLeafLanes(lanes);
                while (inner)
                {
                    const uint32_t lane =
                        uint32_t(std::countr_zero(inner));
                    inner &= inner - 1;
                    nodeStack.push_back(packItem(children.child[lane],
                                                 outMasks[lane]));
                }
            }
        }
    };

    // A terminal query is intentionally strict: it represents the fully
    // resident zero-error cut, so it never silently substitutes an unavailable
    // proxy or applies a copy-on-write overlay to immutable definition ranges.
    for (const uint32_t packedVisible : visible)
    {
        const InstanceId dense = itemValue(packedVisible);
        const SpatialDatabase::Instance& instance = database.instances_[dense];
        const InstanceId outputInstance = database.publicInstanceId(dense);
        const uint32_t instanceWord = packInstanceError(outputInstance, 0);
        const uint32_t rootSlot = instance.rootSlot;
        if (rootSlot == kInvalidIndex)
        {
            const float error = instance.maxErrWorld > 0.0f
                                    ? screenError(
                                          instance.maxErrWorld, view.k,
                                          distanceToBox(instance.worldBox,
                                                        view.queryMin(),
                                                        view.queryMax()))
                                    : 0.0f;
            appendRun(database.tlasRootPayloads_.data() + dense, 1,
                      packInstanceError(
                          outputInstance,
                          encodeFrontierError(error, errorThreshold)));
            continue;
        }

        FRONTIER_CHECK(!instance.hasOverlayList(),
                       "TerminalRenderQuery::select: terminal ranges do not "
                       "support deformed instances");
        FRONTIER_CHECK(database.mountedTreeFullyReady(rootSlot),
                       "TerminalRenderQuery::select: mounted tree is not "
                       "fully ready");
        const SpatialDatabase::SubtreeInstanceRt& mounted =
            database.slots_[rootSlot];
        Impl::Plan& plan = buildPlan(mounted.definition);
        FRONTIER_CHECK(plan.eligible,
                       "TerminalRenderQuery::select: definition contains "
                       "nested mounts or non-terminal geometric error");

        uint8_t mask = itemMask(packedVisible);
        if (coarsenRenderUnits && instance.renderAsUnit()) mask = 0;
        if (mask == 0)
        {
            const Impl::Range range = plan.ranges[0];
            appendRun(plan.payloads.data() + range.begin, range.count,
                      instanceWord);
            continue;
        }
        Camera instanceLocal;
        if (database.instanceOrientations_.empty())
            instanceLocal = toLocal(view, instance.pos, instance.scale, mask);
        else
        {
            const YawRotation yaw = database.instanceOrientations_[dense].yaw;
            instanceLocal = identityYaw(yaw)
                                ? toLocal(view, instance.pos, instance.scale,
                                          mask)
                                : toLocal(view, instance.pos, instance.scale,
                                          yaw, mask);
        }
        const Camera local =
            database.mountLocalCamera(instanceLocal, rootSlot, mask);
        const detail::SubtreeView& subtree =
            database.subtrees_[mounted.definition].view;
        appendDefinition(plan, subtree, local, mask, instanceWord);
    }

    // Homogeneous actor batches deliberately stay outside the mutable TLAS.
    // Their simulation-owned SoA transforms are the current publication; this
    // query performs one exact root cull and only descends partially visible
    // actors. Static/general instances above retain normal TLAS acceleration.
    for (const TerminalInstanceBatch& batch : batches)
    {
        FRONTIER_CHECK(batch.yaws.empty() ||
                           batch.yaws.size() == batch.positions.size(),
                       "TerminalRenderQuery::select: batch yaw count does not "
                       "match its position count");
        FRONTIER_CHECK(batch.clusterBounds.empty() ||
                           batch.clusterBounds.size() ==
                               batch.clusters.size(),
                       "TerminalRenderQuery::select: cluster bound count does "
                       "not match its cluster count");
        FRONTIER_CHECK(batch.scale > 0.0f && std::isfinite(batch.scale) &&
                           std::isfinite(1.0f / batch.scale),
                       "TerminalRenderQuery::select: invalid batch scale");
        FRONTIER_CHECK(finiteNonEmptyBounds(batch.localBounds),
                       "TerminalRenderQuery::select: invalid batch bounds");
        FRONTIER_CHECK(
            uint64_t(batch.firstInstance) + batch.positions.size() <=
                uint64_t(kInvalidInstanceId),
            "TerminalRenderQuery::select: batch instance range exceeds the "
            "24-bit id space");
        const SpatialDatabase::SubtreeDefinitionRt* definition =
            database.resolveSubtree(batch.definition);
        FRONTIER_CHECK(definition != nullptr,
                       "TerminalRenderQuery::select: stale batch definition");
        Impl::Plan& plan = buildPlan(batch.definition.slot);
        FRONTIER_CHECK(plan.eligible,
                       "TerminalRenderQuery::select: batch definition "
                       "contains nested mounts or non-terminal geometric "
                       "error");
        if ((batch.mask & camera.viewMask) == 0) continue;

        const detail::SubtreeView& subtree = definition->view;
        const Impl::Range rootRange = plan.ranges[0];
        const UserPayload* const rootPayloads =
            plan.payloads.data() + rootRange.begin;

        const auto worldBoundsAt = [&](size_t i) -> AABB
        {
            const float4 position = batch.positions[i];
            const YawRotation yaw = batch.yaws.empty()
                                        ? YawRotation{}
                                        : batch.yaws[i];
            FRONTIER_CHECK(finitePosition(position) && validYaw(yaw),
                           "TerminalRenderQuery::select: invalid batch "
                           "transform");
            const AABB worldBounds =
                batch.yawInvariantBounds || identityYaw(yaw)
                    ? toWorld(batch.localBounds, position, batch.scale)
                    : toWorld(batch.localBounds, position, batch.scale, yaw);
            FRONTIER_CHECK(finiteNonEmptyBounds(worldBounds),
                           "TerminalRenderQuery::select: transformed batch "
                           "bounds overflow");
            return worldBounds;
        };

        const auto appendActor = [&](size_t i, uint8_t inheritedMask)
        {
            const float4 position = batch.positions[i];
            const YawRotation yaw = batch.yaws.empty()
                                        ? YawRotation{}
                                        : batch.yaws[i];
            const AABB worldBounds = worldBoundsAt(i);
            uint8_t mask = inheritedMask;
            if (testAabb(worldBounds, camera.frustum, mask) ==
                CullState::Outside)
                return;
            if (coarsenRenderUnits && batch.renderAsUnit) mask = 0;

            const uint32_t instanceWord = packInstanceError(
                batch.firstInstance + InstanceId(i), 0);
            if (mask == 0)
            {
                appendRun(rootPayloads, rootRange.count, instanceWord);
                return;
            }

            const Camera local = identityYaw(yaw)
                                     ? toLocal(camera, position, batch.scale,
                                               mask)
                                     : toLocal(camera, position, batch.scale,
                                               yaw, mask);
            appendDefinition(plan, subtree, local, mask, instanceWord);
        };

        if (batch.clusters.empty())
        {
            for (size_t i = 0; i < batch.positions.size(); ++i)
                appendActor(i, kAllPlanes);
            continue;
        }

#if FRONTIER_CONTRACT_CHECKS
        size_t expectedFirst = 0;
        for (const TerminalInstanceCluster cluster : batch.clusters)
        {
            FRONTIER_CHECK(cluster.count != 0 &&
                               cluster.first == expectedFirst &&
                               uint64_t(cluster.first) + cluster.count <=
                                   batch.positions.size(),
                           "TerminalRenderQuery::select: clusters must form "
                           "an ordered gap-free partition");
            expectedFirst += cluster.count;
        }
        FRONTIER_CHECK(expectedFirst == batch.positions.size(),
                       "TerminalRenderQuery::select: clusters do not cover "
                       "the placement stream");
        for (size_t i = 0; i < batch.positions.size(); ++i)
            FRONTIER_CHECK(
                finitePosition(batch.positions[i]) &&
                    (batch.yaws.empty() || validYaw(batch.yaws[i])),
                "TerminalRenderQuery::select: invalid batch transform");
#endif

        for (size_t clusterIndex = 0;
             clusterIndex < batch.clusters.size(); ++clusterIndex)
        {
            const TerminalInstanceCluster cluster =
                batch.clusters[clusterIndex];
            const size_t first = cluster.first;
            const size_t end = first + cluster.count;
            AABB clusterBounds = batch.clusterBounds.empty()
                                     ? AABB::empty()
                                     : batch.clusterBounds[clusterIndex];
            if (batch.clusterBounds.empty() && batch.yawInvariantBounds)
            {
                float4 minPosition = batch.positions[first];
                float4 maxPosition = minPosition;
                FRONTIER_CHECK(finitePosition(minPosition),
                               "TerminalRenderQuery::select: invalid batch "
                               "position");
                for (size_t i = first + 1; i < end; ++i)
                {
                    const float4 position = batch.positions[i];
                    FRONTIER_CHECK(finitePosition(position),
                                   "TerminalRenderQuery::select: invalid "
                                   "batch position");
                    minPosition = min4(minPosition, position);
                    maxPosition = max4(maxPosition, position);
                }
                clusterBounds = AABB::fromMinMax(
                    batch.localBounds.mn * batch.scale + minPosition,
                    batch.localBounds.mx * batch.scale + maxPosition);
            }
            else if (batch.clusterBounds.empty())
            {
                for (size_t i = first; i < end; ++i)
                    clusterBounds.expand(worldBoundsAt(i));
            }
            FRONTIER_CHECK(finiteNonEmptyBounds(clusterBounds),
                           "TerminalRenderQuery::select: clustered batch "
                           "bounds overflow");
#if FRONTIER_CONTRACT_CHECKS
            if (!batch.clusterBounds.empty())
                for (size_t i = first; i < end; ++i)
                    FRONTIER_CHECK(
                        clusterBounds.contains(worldBoundsAt(i)),
                        "TerminalRenderQuery::select: published cluster "
                        "bounds do not cover their current actors");
#endif

            uint8_t clusterMask = kAllPlanes;
            if (testAabb(clusterBounds, camera.frustum, clusterMask) ==
                CullState::Outside)
                continue;
            if (clusterMask == 0)
            {
                for (size_t i = first; i < end; ++i)
                {
                    FRONTIER_CHECK(
                        finitePosition(batch.positions[i]) &&
                            (batch.yaws.empty() || validYaw(batch.yaws[i])),
                        "TerminalRenderQuery::select: invalid batch "
                        "transform");
                    appendRun(
                        rootPayloads, rootRange.count,
                        packInstanceError(
                            batch.firstInstance + InstanceId(i), 0));
                }
                continue;
            }

            for (size_t i = first; i < end; ++i)
                appendActor(i, clusterMask);
        }
    }

    return {std::span<const TerminalRenderRun>(impl_->runs.data(),
                                               impl_->runs.size()),
            impl_->leafCount};
}

} // namespace frontier
