#include "frontier/spatial_database.h"

#include <cfloat>
#include <cmath>

namespace frontier {
namespace {

inline bool finitePosition(float4 p)
{
    return std::isfinite(p.x) && std::isfinite(p.y) &&
           std::isfinite(p.z);
}

inline bool validYaw(YawRotation yaw)
{
    const float norm2 = yaw.cosine * yaw.cosine + yaw.sine * yaw.sine;
    return std::isfinite(norm2) &&
           std::fabs(norm2 - 1.0f) <= 1.0e-3f;
}

inline bool finiteNonEmptyBounds(const AABB& bounds)
{
    return bounds.mn.x <= bounds.mx.x &&
           bounds.mn.y <= bounds.mx.y &&
           bounds.mn.z <= bounds.mx.z &&
           bounds.mx.x - bounds.mn.x < FLT_MAX &&
           bounds.mx.y - bounds.mn.y < FLT_MAX &&
           bounds.mx.z - bounds.mn.z < FLT_MAX;
}

} // namespace

void SpatialDatabase::moveRigidInstances(
    RigidMotionGroup& rigid, std::span<const float4> positions,
    std::span<const YawRotation> yaws)
{
    MotionGroup& group = rigid.motion_;
    FRONTIER_CHECK(group.instances_.size() == positions.size() &&
                       positions.size() == yaws.size(),
                   "SpatialDatabase::moveRigidInstances: "
                   "motion-group/stream count mismatch");
    if (!group.physicalOrderValid_ ||
        group.mappingVersion_ != instanceMappingVersion_)
        refreshMotionGroup(group);

    if (rigid.checkedMappingVersion_ != group.mappingVersion_)
    {
        rigid.allYawInvariant_ = !group.physicalOrder_.empty() &&
                                 !instanceOrientations_.empty();
        for (const MotionGroup::Slot slot : group.physicalOrder_)
            rigid.allYawInvariant_ =
                rigid.allYawInvariant_ &&
                std::signbit(instanceOrientations_[slot.dense].radiusXZ);
        rigid.checkedMappingVersion_ = group.mappingVersion_;
    }

    materializeTlasGlobalOffset();
    uint32_t mutationGeneration = 0;
    float batchMaxTravel = 0.0f;
    if (!rigid.allYawInvariant_)
    {
        for (const MotionGroup::Slot slot : group.physicalOrder_)
        {
            const Instance& instance = instances_[slot.dense];
            moveInstanceDense(slot.dense, positions[slot.source],
                              instance.scale, yaws[slot.source],
                              mutationGeneration, batchMaxTravel);
        }
    }
    else
    {
        // A yaw-invariant authored envelope makes rigid publication a pure
        // translation of the exact world AABB. The persistent group already
        // owns a generation-validated, duplicate-resolved, dense-sorted map.
        for (const MotionGroup::Slot slot : group.physicalOrder_)
        {
            Instance& instance = instances_[slot.dense];
            InstanceOrientation& orientation =
                instanceOrientations_[slot.dense];
            float4 pos = positions[slot.source];
            const YawRotation yaw = yaws[slot.source];
            FRONTIER_CHECK(finitePosition(pos) && validYaw(yaw),
                           "SpatialDatabase::moveRigidInstances: "
                           "invalid transform");
            pos.w = 1.0f;
            const YawRotation oldYaw = orientation.yaw;
            if (pos.x == instance.pos.x && pos.y == instance.pos.y &&
                pos.z == instance.pos.z &&
                yaw.cosine == oldYaw.cosine && yaw.sine == oldYaw.sine)
                continue;

            const float4 delta = pos - instance.pos;
            const AABB worldBox = AABB::fromMinMax(
                instance.worldBox.mn + delta,
                instance.worldBox.mx + delta);
            const float translationTravel =
                std::fabs(delta.x) + std::fabs(delta.y) +
                std::fabs(delta.z);
            const float yawChordBound =
                std::fabs(yaw.cosine - oldYaw.cosine) +
                std::fabs(yaw.sine - oldYaw.sine);
            const float rotationTravel =
                instance.scale * -orientation.radiusXZ * yawChordBound;
            const float travel = translationTravel + rotationTravel;
            const float nextTravel =
                instanceMotionTravel_[slot.dense] + travel;
            FRONTIER_CHECK(finiteNonEmptyBounds(worldBox),
                           "SpatialDatabase::moveRigidInstances: "
                           "transformed root overflows");

            if (!std::isfinite(nextTravel))
            {
                instanceMotionTravel_[slot.dense] = 0.0f;
                if (mutationGeneration == 0)
                    mutationGeneration = ++generationCounter_;
                invalidateInstanceFrontier(slot.dense,
                                           mutationGeneration);
            }
            else
            {
                instanceMotionTravel_[slot.dense] = nextTravel;
                batchMaxTravel = std::max(batchMaxTravel, travel);
            }
            instance.pos = pos;
            instance.worldBox = worldBox;
            orientation.yaw = yaw;
            tlasItemsTmp_.push_back(slot.dense);
        }
    }
    instanceMotionTravelGlobal_ += batchMaxTravel;
    if (!group.physicalOrder_.empty()) ++instanceSpatialVersion_;
}

} // namespace frontier
