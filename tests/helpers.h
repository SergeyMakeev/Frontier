#pragma once

#include <algorithm>
#include <stdexcept>
#include <vector>

#include "frontier/builder.h"
#include "frontier/spatial_database.h"

namespace frontier {

// Tests intentionally resolve payloads by scanning live mounted subtrees.
// Production code retains handles from frontier results instead.
struct SpatialDatabase::TestAccess
{
    static NodeHandle findNode(SpatialDatabase& database, UserPayload payload)
    {
        for (uint32_t slot = 0; slot < database.slots_.size(); ++slot)
        {
            const SubtreeInstanceRt& instance = database.slots_[slot];
            if (!instance.inUse()) continue;
            const detail::SubtreeView& subtree = database.subtreeView(instance);
            for (uint32_t node = 1; node <= subtree.nodeCount(); ++node)
            {
                NodeHandle handle{slot, node,
                                  database.mountStamps_[slot].generation()};
                const UserPayload candidate = database.tryGetPayload(handle);
                if (candidate == payload)
                    return handle;
            }
        }
        return {};
    }

    static NodeHandle requireNode(SpatialDatabase& database,
                                  UserPayload payload)
    {
        NodeHandle handle = findNode(database, payload);
        if (!handle.valid())
            throw std::logic_error("payload is not mounted");
        return handle;
    }

    static NodeHandle requireNode(SpatialDatabase& database,
                                  InstanceHandle owner,
                                  UserPayload payload)
    {
        const Instance* topLevel = database.resolveInstance(owner);
        if (!topLevel) throw std::logic_error("instance is stale");
        const InstanceId dense =
            InstanceId(topLevel - database.instances_.data());
        for (uint32_t slot = 0; slot < database.slots_.size(); ++slot)
        {
            const SubtreeInstanceRt& instance = database.slots_[slot];
            if (!instance.inUse()) continue;
            const SubtreeInstanceRt& root =
                database.slots_[instance.rootSlot];
            if (!root.owner.isTlasRoot() || root.owner.index != dense) continue;
            const detail::SubtreeView& subtree = database.subtreeView(instance);
            for (uint32_t node = 1; node <= subtree.nodeCount(); ++node)
            {
                NodeHandle handle{slot, node,
                                  database.mountStamps_[slot].generation()};
                const UserPayload candidate = database.tryGetPayload(handle);
                if (candidate == payload)
                    return handle;
            }
        }
        throw std::logic_error("payload is not mounted on instance");
    }

    static AABB instanceBounds(SpatialDatabase& database,
                               InstanceHandle handle)
    {
        database.flushBounds();
        const Instance* instance = database.resolveInstance(handle);
        return instance
                   ? AABB::fromMinMax(
                         instance->worldBox.mn + database.tlasGlobalOffset_,
                         instance->worldBox.mx + database.tlasGlobalOffset_)
                   : AABB::empty();
    }

    static AABB unflushedInstanceBounds(SpatialDatabase& database,
                                        InstanceHandle handle)
    {
        const Instance* instance = database.resolveInstance(handle);
        return instance ? instance->worldBox : AABB::empty();
    }

    static YawRotation instanceYaw(const SpatialDatabase& database,
                                   InstanceHandle handle)
    {
        const InstanceId dense = database.denseInstanceId(handle);
        return dense != kInvalidInstanceId &&
                       !database.instanceOrientations_.empty()
                   ? database.instanceOrientations_[dense].yaw
                   : YawRotation{};
    }

    static AABB tlasLeafBounds(SpatialDatabase& database,
                               InstanceHandle handle)
    {
        const InstanceId dense = database.denseInstanceId(handle);
        if (dense == kInvalidInstanceId) return AABB::empty();
        const Instance& instance = database.instances_[dense];
        return database.tlasNodes_[instance.tlasNode()].bounds.lane(
            instance.tlasLane());
    }

    static bool tlasLeafIsLoose(SpatialDatabase& database,
                                InstanceHandle handle)
    {
        const InstanceId dense = database.denseInstanceId(handle);
        return dense != kInvalidInstanceId &&
               database.instanceTlasLoose_[dense] != 0;
    }

    static size_t definitionBytes() { return sizeof(SubtreeDefinitionRt); }
    static const void* definitionData(const SpatialDatabase& database,
                                      SubtreeHandle handle)
    {
        const SubtreeDefinitionRt* definition =
            database.resolveSubtree(handle);
        return definition ? definition->bytes.data() : nullptr;
    }
    static size_t mountedStateBytes() { return sizeof(SubtreeInstanceRt); }
    static size_t mountStampBytes() { return sizeof(MountStamp); }
    static size_t mountReadinessBytes() { return sizeof(MountReadiness); }
    static size_t instanceBytes() { return sizeof(Instance); }
    static size_t tlasNodeBytes()
    {
        return sizeof(TlasNode) + sizeof(TlasMeta);
    }
    static size_t tlasNodeCount(const SpatialDatabase& database)
    {
        return database.tlasNodes_.size();
    }
    static size_t liveInstanceSlots(const SpatialDatabase& database)
    {
        return database.liveInstances_.size();
    }
    static size_t allocatedInstanceSlots(const SpatialDatabase& database)
    {
        return database.instances_.size();
    }
    static uint32_t instanceLayoutVersion(const SpatialDatabase& database)
    {
        return database.instanceLayoutVersion_;
    }
    static uint32_t instanceMappingVersion(const SpatialDatabase& database)
    {
        return database.instanceMappingVersion_;
    }
    static InstanceId denseInstanceId(const SpatialDatabase& database,
                                      InstanceHandle handle)
    {
        return database.denseInstanceId(handle);
    }

    static bool overlayIsSparse(SpatialDatabase& database,
                                InstanceHandle owner, NodeHandle node)
    {
        const Instance* instance = database.resolveInstance(owner);
        if (!instance || node.isTlasRoot()) return false;
        const Overlay* overlay = database.findOverlay(*instance, node.slot());
        return overlay && overlay->sparseWide();
    }

    static void markAllNodesReady(SpatialDatabase& database)
    {
        for (uint32_t slot = 0; slot < database.slots_.size(); ++slot)
        {
            const SubtreeInstanceRt& instance = database.slots_[slot];
            if (!instance.inUse()) continue;
            const detail::SubtreeView& subtree =
                database.subtreeView(instance);
            const uint32_t count = subtree.nodeCount();
            for (uint32_t node = 1; node <= count; ++node)
                database.markNodeReady(NodeHandle{
                    slot, node, database.mountStamps_[slot].generation()});
        }
    }

    static NodeHandle nodeAt(SpatialDatabase& database,
                             SubtreeInstanceHandle instance,
                             uint32_t packedIndex)
    {
        if (!database.isMounted(instance)) return {};
        const SubtreeInstanceRt& mounted = database.slots_[instance.slot];
        if (packedIndex == 0 ||
            packedIndex > database.subtreeView(mounted).nodeCount())
            return {};
        return NodeHandle{instance.slot, packedIndex, instance.generation};
    }
};

} // namespace frontier

namespace frontiertest {

using namespace frontier;
using TestAccess = SpatialDatabase::TestAccess;

inline AABB box(float half = 1.0f,
                float4 center = float4::point(0.0f, 0.0f, 0.0f))
{
    const float4 extent = float4::vec(half, half, half);
    return AABB::fromMinMax(center - extent, center + extent);
}

inline NodeDesc node(UserPayload payload, float error, const AABB& bounds,
                     bool mountable = false)
{
    NodeDesc result;
    result.payload = payload;
    result.geometricError = error;
    result.bounds = bounds;
    result.flags = mountable ? NodeDesc::FlagMountable : 0;
    return result;
}

inline SubtreeBytes makeLodSubtree(UserPayload coarse = 10,
                                   UserPayload left = 11,
                                   UserPayload right = 12)
{
    SubtreeBuilder builder;
    const auto root = builder.createNode(node(coarse, 16.0f, box(4.0f)));
    builder.createNode(root, node(left, 0.0f,
                                  box(1.0f, float4::point(-2, 0, 0))));
    builder.createNode(root, node(right, 0.0f,
                                  box(1.0f, float4::point(2, 0, 0))));
    return builder.build();
}

inline SubtreeBytes makeLeafSubtree(UserPayload payload, float half = 1.0f)
{
    SubtreeBuilder builder;
    builder.createNode(node(payload, 0.0f, box(half)));
    return builder.build();
}

inline InstanceHandle instantiateFor(SpatialDatabase& database,
                                     SubtreeHandle subtree,
                                     const AABB& bounds,
                                     float error = 64.0f,
                                     InstanceDesc desc = {},
                                     Transform mountTransform = {})
{
    const InstanceHandle instance = database.instantiate(
        node(1, error, bounds, true), desc);
    const SubtreeInstanceHandle mounted =
        database.mountSubtree(instance.rootNode(), subtree, mountTransform);
    if (!mounted.valid()) throw std::logic_error("root mount failed");
    return instance;
}

inline NodeHandle handleOf(SpatialDatabase& database, UserPayload payload)
{
    return TestAccess::requireNode(database, payload);
}

inline std::vector<UserPayload> payloads(const SpatialDatabase& database,
                                         const FrontierResultView& result)
{
    std::vector<UserPayload> output;
    for (const FrontierEntry& entry : result)
    {
        const UserPayload payload = database.tryGetPayload(entry.nodeHandle);
        if (payload == kInvalidPayload)
            throw std::logic_error("stale frontier handle");
        output.push_back(payload);
    }
    std::sort(output.begin(), output.end());
    return output;
}

inline std::vector<UserPayload> refinedPayloads(
    const SpatialDatabase& database, SpatialQuery& query,
    FrontierResultView current,
    uint32_t maxDepth = SpatialQuery::UnlimitedDepth,
    uint32_t maxNodes = UINT32_MAX)
{
    const FrontierRefinementView refinement =
        query.computeFrontierRefinement(database, current, maxDepth, maxNodes);
    std::vector<FrontierEntry> terminal(current.begin(), current.end());
    for (uint32_t group = 0; group < refinement.groupCount(); ++group)
    {
        const NodeHandle parent = refinement.parent(group);
        const auto found = std::find_if(
            terminal.begin(), terminal.end(), [&](const FrontierEntry& entry)
            { return entry.nodeHandle == parent; });
        if (found == terminal.end())
            throw std::logic_error("refinement group parent is not terminal");
        const size_t offset = size_t(found - terminal.begin());
        terminal.erase(found);
        const std::span<const FrontierEntry> children =
            refinement.children(group);
        terminal.insert(terminal.begin() + offset, children.begin(),
                        children.end());
    }

    std::vector<UserPayload> output;
    output.reserve(terminal.size());
    for (const FrontierEntry& entry : terminal)
    {
        const UserPayload payload = database.tryGetPayload(entry.nodeHandle);
        if (payload == kInvalidPayload)
            throw std::logic_error("stale refinement handle");
        output.push_back(payload);
    }
    std::sort(output.begin(), output.end());
    return output;
}

inline Camera cameraAt(float z = -20.0f,
                       float4 target = float4::point(0, 0, 0))
{
    return makeLookAtCamera(float4::point(target.x, target.y, z), target);
}

inline FrontierResultView select(SpatialDatabase& database, SpatialQuery& query,
                                 const Camera& camera,
                                 SelectionParams params = {})
{
    database.applyUpdates(0);
    return query.selectFrontier(database, camera, params);
}

} // namespace frontiertest
