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
            const Subtree& subtree = database.subtreeView(instance);
            for (uint32_t node = 1; node <= subtree.nodeCount(); ++node)
            {
                NodeHandle handle{slot, node,
                                  database.mountStamps_[slot].generation()};
                UserPayload candidate = 0;
                if (database.tryGetPayload(handle, candidate) &&
                    candidate == payload)
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
            const Subtree& subtree = database.subtreeView(instance);
            for (uint32_t node = 1; node <= subtree.nodeCount(); ++node)
            {
                NodeHandle handle{slot, node,
                                  database.mountStamps_[slot].generation()};
                UserPayload candidate = 0;
                if (database.tryGetPayload(handle, candidate) &&
                    candidate == payload)
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
        return instance ? instance->worldBox : AABB::empty();
    }

    static size_t definitionBytes() { return sizeof(SubtreeDefinitionRt); }
    static size_t mountedStateBytes() { return sizeof(SubtreeInstanceRt); }
    static size_t mountStampBytes() { return sizeof(MountStamp); }
    static size_t mountResidencyBytes() { return sizeof(MountResidency); }
    static size_t instanceBytes() { return sizeof(Instance); }
    static size_t tlasNodeBytes() { return sizeof(TlasNode); }
    static size_t liveInstanceSlots(const SpatialDatabase& database)
    {
        return database.liveInstances_.size();
    }

    static void markAllPayloadsResident(SpatialDatabase& database)
    {
        for (uint32_t slot = 0; slot < database.slots_.size(); ++slot)
        {
            const SubtreeInstanceRt& instance = database.slots_[slot];
            if (!instance.inUse()) continue;
            const uint32_t generation =
                database.mountStamps_[slot].generation();
            const uint32_t count =
                database.subtreeView(instance).nodeCount();
            for (uint32_t node = 1; node <= count; ++node)
                database.markPayloadResident(
                    NodeHandle{slot, node, generation});
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
                     SubtreeKey child = {}, Transform transform = {})
{
    NodeDesc result;
    result.payload = payload;
    result.geometricError = error;
    result.bounds = bounds;
    result.childSubtree = child;
    result.childTransform = transform;
    return result;
}

inline Subtree makeLodSubtree(SubtreeKey key, UserPayload coarse = 10,
                              UserPayload left = 11,
                              UserPayload right = 12)
{
    SubtreeBuilder builder(key);
    const auto root = builder.createNode(node(coarse, 16.0f, box(4.0f)));
    builder.createNode(root, node(left, 0.0f,
                                  box(1.0f, float4::point(-2, 0, 0))));
    builder.createNode(root, node(right, 0.0f,
                                  box(1.0f, float4::point(2, 0, 0))));
    return builder.build();
}

inline Subtree makeLeafSubtree(SubtreeKey key, UserPayload payload,
                               float half = 1.0f)
{
    SubtreeBuilder builder(key);
    builder.createNode(node(payload, 0.0f, box(half)));
    return builder.build();
}

inline InstanceHandle instantiateFor(SpatialDatabase& database,
                                     SubtreeHandle subtree,
                                     SubtreeKey key,
                                     const AABB& bounds,
                                     float error = 64.0f,
                                     InstanceDesc desc = {},
                                     Transform childTransform = {})
{
    const InstanceHandle instance = database.instantiate(
        node(1, error, bounds, key, childTransform), desc);
    const SubtreeInstanceHandle mounted =
        database.mountSubtree(instance.rootNode(), subtree);
    if (!mounted.valid()) throw std::logic_error("root mount failed");
    return instance;
}

inline NodeHandle handleOf(SpatialDatabase& database, UserPayload payload)
{
    return TestAccess::requireNode(database, payload);
}

inline std::vector<UserPayload> payloads(const SpatialDatabase& database,
                                         const FrontierResultView& result,
                                         bool ideal = true)
{
    std::vector<UserPayload> output;
    const auto append = [&](std::span<const FrontierEntry> entries)
    {
        for (const FrontierEntry& entry : entries)
        {
            UserPayload payload = 0;
            if (!database.tryGetPayload(entry.nodeHandle, payload))
                throw std::logic_error("stale frontier handle");
            output.push_back(payload);
        }
    };
    append(result.shared);
    append(ideal ? result.idealOnly : result.currentOnly);
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
    database.applyUpdates();
    return query.selectFrontier(database, camera, params);
}

} // namespace frontiertest
