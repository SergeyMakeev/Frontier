#include <frontier/builder.h>
#include <frontier/spatial_database.h>

#include "debugdraw/debugdraw.h"
#include "args.h"
#include "camera.h"
#include "entry/entry.h"
#include "entry/input.h"
#include "imgui/imgui.h"
#include "hardware_info.h"
#include "window_frame.h"

#include <bgfx/bgfx.h>
#include <bx/bounds.h>
#include <bx/math.h>
#include <bx/timer.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace
{

using namespace frontier;

constexpr uint16_t kMainView = 0;
constexpr int kDistrictBlockCount = 8;
constexpr int kDistrictsPerAxis = 3;
constexpr int kDistrictCount = kDistrictsPerAxis * kDistrictsPerAxis;
constexpr int kBlockCount = kDistrictBlockCount * kDistrictsPerAxis;
constexpr float kBlockSpacing = 24.0f;
constexpr float kDistrictSpan = kDistrictBlockCount * kBlockSpacing;
constexpr float kCityHalfExtent = kBlockCount * kBlockSpacing * 0.5f;
constexpr float kCameraFarPlane = 2000.0f;
constexpr float kRoadHalfWidth = 3.8f;
constexpr float kSidewalkOuterExtent = 8.15f;
constexpr float kSidewalkInnerExtent = 6.35f;
constexpr float kSidewalkPathHalfExtent = 7.25f;
constexpr float kSidewalkCornerRadius = 1.20f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kWorstCaseWaveAmplitude = 7.0f;
constexpr float kWorstCaseWaveFrequency = 1.8f;

enum class Payload : UserPayload
{
    HouseTop = 1,
    HouseCoarse,
    HouseBody,
    HouseRoof,
    CarTop,
    CarCoarse,
    CarBody,
    CarCabin,
    PedestrianTop,
    PedestrianCoarse,
    PedestrianBody,
    PedestrianHead,
    TreeTop,
    TreeCoarse,
    TreeTrunk,
    TreeCrown,
    TowerTop,
    TowerDistrict,
    TowerCoarse,
    TowerMedium,
    TowerFine,
    TowerBase,
    TowerShaft,
    TowerCrown,
};

constexpr size_t kPayloadSlotCount =
    static_cast<size_t>(Payload::TowerCrown) + 1;
constexpr size_t kHouseDetailResourceCount = 3;
constexpr size_t kTowerFacadePayloadCount =
    size_t(Payload::TowerFine) - size_t(Payload::TowerDistrict) + 1;
constexpr size_t kTowerDetailResourceCount =
    size_t(Payload::TowerCrown) - size_t(Payload::TowerDistrict) + 1;
constexpr size_t kTowerPayloadCount =
    size_t(Payload::TowerCrown) - size_t(Payload::TowerTop) + 1;
static_assert(kTowerFacadePayloadCount <= kMaxNodePayloads);
constexpr uint16_t kHeroTowerInstancesPerAsset = 3;
constexpr size_t kTowerBlocksPerDistrict = 6;
constexpr size_t kTowerInstanceCount =
    kTowerBlocksPerDistrict * kDistrictCount;
constexpr size_t kHeroTowerAssetCount =
    (kTowerInstanceCount + kHeroTowerInstancesPerAsset - 1) /
    kHeroTowerInstancesPerAsset;
constexpr size_t kHeroTowerResourceBase =
    kPayloadSlotCount + kHouseDetailResourceCount;
constexpr size_t kStreamingResourceSlotCount =
    kHeroTowerResourceBase +
    kHeroTowerAssetCount * kTowerDetailResourceCount;
constexpr size_t kMaxStreamingResidencyGroupSize = 3;
constexpr size_t kStreamingLogCapacity = 96;
constexpr size_t kStreamingConvergenceHistorySize = 300;
constexpr float kStreamingConvergenceSampleInterval = 1.0f / 30.0f;
constexpr float kHeroPressureTestBudgetMiB = 85.0f;
constexpr float kHeroOrbitCenterX = -12.0f;
constexpr float kHeroOrbitCenterZ = -12.0f;
constexpr float kHeroOrbitTargetHeight = 20.0f;
constexpr float kHeroOrbitCameraHeight = 27.0f;
constexpr float kHeroOrbitRadius = 58.0f;
constexpr float kHeroApproachStartRadius = 150.0f;
constexpr float kHeroApproachSeconds = 30.0f;
constexpr float kHeroOrbitAngularSpeed = 0.035f;
constexpr float kTowerRootHalfExtent = 5.2f;
constexpr float kTowerRootHeight = 46.5f;
constexpr float kHeroOrbitLookaheadSeconds = 6.0f;
constexpr float kHeroOrbitWarmupSeconds = 20.0f;
constexpr float kHeroOrbitTestSeconds = 180.0f;
// TowerTop has 0.9 m geometric error versus a roughly 46 m facade, so a
// 3-pixel top error corresponds to a skyscraper around 150 pixels tall.
constexpr float kHeroOrbitDominantFallbackErrorPixels = 3.0f;
constexpr uint32_t kHeroOrbitMaxRapidReloads = 1;
constexpr float kHeroOrbitRapidReloadWindowSeconds = 5.0f;
constexpr float kStreamingReplacementMinimumGain = 1.05f;
constexpr size_t kPerformanceHistorySize = 300;
constexpr float kPerformanceHistorySampleInterval = 1.0f / 60.0f;

enum class HouseStyle : uint8_t
{
    HouseA,
    HouseB,
};

enum class StreamingResourceState : uint8_t
{
    Unloaded,
    Loading,
    Resident,
};

struct RepresentationInfo
{
    const char* name = "Unknown";
    float byteSizeMiB = 0.0f;
    bool coarsest = false;
};

struct StreamingErrorStats
{
    uint32_t count = 0;
    float minimum = std::numeric_limits<float>::max();
    float maximum = 0.0f;
    double total = 0.0;

    void reset()
    {
        count = 0;
        minimum = std::numeric_limits<float>::max();
        maximum = 0.0f;
        total = 0.0;
    }

    void add(float screenError)
    {
        ++count;
        minimum = std::min(minimum, screenError);
        maximum = std::max(maximum, screenError);
        total += screenError;
    }

    float average() const
    {
        return count != 0 ? float(total / double(count)) : 0.0f;
    }
};

struct VirtualResource
{
    Payload payload = Payload::HouseTop;
    HouseStyle houseStyle = HouseStyle::HouseA;
    StreamingResourceState state = StreamingResourceState::Unloaded;
    NodeHandle representative{};
    uint8_t representativePayloadIndex = 0;
    float byteSizeMiB = 0.0f;
    float lastDemandTime = 0.0f;
    StreamingErrorStats currentErrors;
    StreamingErrorStats currentBenefitErrors;
    StreamingErrorStats idealErrors;
    StreamingErrorStats transitionErrors;
    StreamingErrorStats prefetchErrors;
    StreamingErrorStats residentBenefitErrors;
    float importanceScore = 0.0f;
    float scorePerMiB = 0.0f;
    std::string decision = "not evaluated";
    std::string lastAction = "never loaded";
    std::array<size_t, kMaxStreamingResidencyGroupSize> residencyGroup{};
    uint16_t heroAsset = UINT16_MAX;
    uint8_t residencyGroupCount = 0;
    bool pinned = false;
};

struct PendingStreamingGroup
{
    std::vector<size_t> resources;
    float readyAt = 0.0f;
    float byteSizeMiB = 0.0f;
    float scorePerMiB = 0.0f;
    uint64_t serial = 0;
};

struct FrontierErrorSnapshot
{
    std::vector<float> instanceMaxPixels;
    std::vector<uint8_t> instanceVisible;
    std::vector<uint8_t> selectedGroupInstance;
    float rmsPixels = 0.0f;
    float worstPixels = 0.0f;
    uint32_t visibleInstances = 0;
};

struct StreamingErrorImprovement
{
    std::string resourceNames;
    uint64_t groupSerial = 0;
    uint32_t affectedInstances = 0;
    float localBeforeRmsPixels = 0.0f;
    float localAfterRmsPixels = 0.0f;
    float localBeforeWorstPixels = 0.0f;
    float localAfterWorstPixels = 0.0f;
    float sceneBeforeRmsPixels = 0.0f;
    float sceneAfterRmsPixels = 0.0f;
    float sceneBeforeWorstPixels = 0.0f;
    float sceneAfterWorstPixels = 0.0f;
    bool valid = false;
};

struct StreamingCandidateGroup
{
    std::vector<size_t> resources;
    float importanceScore = 0.0f;
    float scorePerMiB = 0.0f;
    bool qualityFloor = false;
};

struct StreamingLogEntry
{
    float time = 0.0f;
    ImVec4 color{};
    std::string message;
};

enum class PerformanceTimer : uint8_t
{
    Total,
    Ui,
    Simulation,
    ApplyUpdates,
    TlasRebuild,
    Camera,
    Selection,
    CutStats,
    Render,
    Streaming,
    StreamingRefinement,
    StreamingErrorMeasurement,
    StreamingPlanner,
    StreamingPlannerIndex,
    StreamingPlannerDemand,
    StreamingPlannerScore,
    StreamingPlannerResidency,
    StreamingPlannerOther,
    StreamingScenario,
    FrameSubmit,
    RenderThread,
    Gpu,
    WaitRender,
    WaitSubmit,
    Other,
    Count,
};

constexpr size_t kPerformanceTimerCount =
    static_cast<size_t>(PerformanceTimer::Count);

enum class RebuildStrategy : uint8_t
{
    Manual,
    Periodic,
    WhenRecommended,
};

enum class StreamingCutStrategy : uint8_t
{
    QualityPerByte,
    RetainReadyDetail,
};

enum class EntityKind : uint8_t
{
    House,
    Car,
    Pedestrian,
    Tree,
    Tower,
};

struct SceneDefinition
{
    SubtreeHandle handle;
    uint32_t authoredNodeCount = 0;
};

// Count authoring calls before packing can introduce internal spatial nodes.
// Additional payload LODs belong to the same node and do not increase the count.
class SceneDefinitionBuilder
{
public:
    template <typename... Args>
    SubtreeBuilder::NodeId createNode(Args&&... args)
    {
        const auto id = builder_.createNode(std::forward<Args>(args)...);
        ++authoredNodeCount_;
        return id;
    }

    SceneDefinition registerWith(SpatialDatabase& database)
    {
        return {database.registerSubtree(builder_.build()), authoredNodeCount_};
    }

private:
    SubtreeBuilder builder_;
    uint32_t authoredNodeCount_ = 0;
};

struct Entity
{
    float4 localPosition = float4::point(0.0f, 0.0f, 0.0f);
    float4 position = float4::point(0.0f, 0.0f, 0.0f);
    float scale = 1.0f;
    float localYaw = 0.0f;
    float yaw = 0.0f;
    uint32_t color = 0xffffffff;
    uint32_t authoredHlodNodeCount = 0;
    uint64_t lastMotionFrame = 0;
    EntityKind kind = EntityKind::House;
    HouseStyle houseStyle = HouseStyle::HouseA;
    uint16_t heroAsset = UINT16_MAX;
};

constexpr uint32_t entityKindBit(EntityKind kind)
{
    return 1u << uint32_t(kind);
}

constexpr uint32_t kAllEntityKindMask =
    entityKindBit(EntityKind::House) |
    entityKindBit(EntityKind::Car) |
    entityKindBit(EntityKind::Pedestrian) |
    entityKindBit(EntityKind::Tree) |
    entityKindBit(EntityKind::Tower);

struct SceneTreeSelection
{
    uint32_t entityKindMask = 0;
    uint16_t heroAsset = UINT16_MAX;
    InstanceId instance = kInvalidInstanceId;
    Payload firstPayload = Payload::HouseTop;
    Payload lastPayload = Payload::TowerCrown;
    std::string label;

    bool active() const { return entityKindMask != 0; }
};

struct CarPath
{
    float centerX = 0.0f;
    float centerZ = 0.0f;
    float halfX = 24.0f;
    float halfZ = 24.0f;
    float cornerRadius = 6.0f;
    float phase = 0.0f;
    float speed = 0.0f;
    bool reverse = false;
};

struct PedestrianPath
{
    float centerX = 0.0f;
    float centerZ = 0.0f;
    float phase = 0.0f;
    float speed = 0.0f;
    bool reverse = false;
};

struct CameraPose
{
    float4 position = float4::point(0.0f, 0.0f, 0.0f);
    float4 target = float4::point(0.0f, 0.0f, 1.0f);
    std::array<float, 16> view{};
    std::array<float, 16> projection{};
    std::array<float, 16> viewProjection{};
};

struct FrozenCullState
{
    Camera camera{};
    CameraPose pose{};
    bool valid = false;
};

struct PerformanceSample
{
    float totalMs = 0.0f;
    float uiMs = 0.0f;
    float simulationMs = 0.0f;
    float applyUpdatesMs = 0.0f;
    float tlasRebuildMs = 0.0f;
    float cameraMs = 0.0f;
    float selectionMs = 0.0f;
    float cutStatsMs = 0.0f;
    float renderMs = 0.0f;
    float streamingMs = 0.0f;
    float streamingRefinementMs = 0.0f;
    float streamingErrorMeasurementMs = 0.0f;
    float streamingPlannerMs = 0.0f;
    float streamingPlannerIndexMs = 0.0f;
    float streamingPlannerDemandMs = 0.0f;
    float streamingPlannerScoreMs = 0.0f;
    float streamingPlannerResidencyMs = 0.0f;
    float streamingPlannerOtherMs = 0.0f;
    float streamingScenarioMs = 0.0f;
    float frameSubmitMs = 0.0f;
    float renderThreadMs = 0.0f;
    float gpuMs = 0.0f;
    float waitRenderMs = 0.0f;
    float waitSubmitMs = 0.0f;
    uint32_t drawCalls = 0;
    uint32_t triangles = 0;
    int32_t transientVertexBytes = 0;
    int32_t transientIndexBytes = 0;
};

uint32_t abgr(uint8_t red, uint8_t green, uint8_t blue,
              uint8_t alpha = 255)
{
    return (uint32_t(alpha) << 24) | (uint32_t(blue) << 16) |
           (uint32_t(green) << 8) | uint32_t(red);
}

float random01(uint32_t& state)
{
    state = state * 1664525u + 1013904223u;
    return float(state >> 8) * (1.0f / 16777216.0f);
}

float roundedLoopLength(float halfX, float halfZ, float cornerRadius)
{
    const float straightX = 2.0f * (halfX - cornerRadius);
    const float straightZ = 2.0f * (halfZ - cornerRadius);
    return 2.0f * (straightX + straightZ) + 2.0f * kPi * cornerRadius;
}

float roundedLoopLength(const CarPath& path)
{
    return roundedLoopLength(path.halfX, path.halfZ, path.cornerRadius);
}

void sampleRoundedLoop(const CarPath& path, float time,
                       float4& outPosition, float& outYaw)
{
    const float radius = path.cornerRadius;
    const float straightX = 2.0f * (path.halfX - radius);
    const float straightZ = 2.0f * (path.halfZ - radius);
    const float arc = kPi * 0.5f * radius;
    const float perimeter = roundedLoopLength(path);
    float distance = std::fmod(path.phase + path.speed * time, perimeter);
    if (distance < 0.0f)
        distance += perimeter;
    if (path.reverse)
        distance = perimeter - distance;

    float x = -path.halfX + radius;
    float z = -path.halfZ;
    float tangentX = 1.0f;
    float tangentZ = 0.0f;
    const auto sampleArc = [&](float centerX, float centerZ,
                               float startAngle, float localDistance)
    {
        const float angle = startAngle + localDistance / radius;
        x = centerX + std::cos(angle) * radius;
        z = centerZ + std::sin(angle) * radius;
        tangentX = -std::sin(angle);
        tangentZ = std::cos(angle);
    };

    if (distance < straightX)
    {
        x += distance;
    }
    else if ((distance -= straightX) < arc)
    {
        sampleArc(path.halfX - radius, -path.halfZ + radius,
                  -kPi * 0.5f, distance);
    }
    else if ((distance -= arc) < straightZ)
    {
        x = path.halfX;
        z = -path.halfZ + radius + distance;
        tangentX = 0.0f;
        tangentZ = 1.0f;
    }
    else if ((distance -= straightZ) < arc)
    {
        sampleArc(path.halfX - radius, path.halfZ - radius,
                  0.0f, distance);
    }
    else if ((distance -= arc) < straightX)
    {
        x = path.halfX - radius - distance;
        z = path.halfZ;
        tangentX = -1.0f;
        tangentZ = 0.0f;
    }
    else if ((distance -= straightX) < arc)
    {
        sampleArc(-path.halfX + radius, path.halfZ - radius,
                  kPi * 0.5f, distance);
    }
    else if ((distance -= arc) < straightZ)
    {
        x = -path.halfX;
        z = path.halfZ - radius - distance;
        tangentX = 0.0f;
        tangentZ = -1.0f;
    }
    else
    {
        distance -= straightZ;
        sampleArc(-path.halfX + radius, -path.halfZ + radius,
                  kPi, distance);
    }

    if (path.reverse)
    {
        tangentX = -tangentX;
        tangentZ = -tangentZ;
    }
    outPosition = float4::point(path.centerX + x, 0.08f,
                                path.centerZ + z);
    outYaw = std::atan2(tangentZ, tangentX);
}

AABB bounds(float minX, float minY, float minZ,
            float maxX, float maxY, float maxZ)
{
    return AABB::fromMinMax(float4::point(minX, minY, minZ),
                            float4::point(maxX, maxY, maxZ));
}

constexpr float authoredGeometricError(Payload payload)
{
    switch (payload)
    {
    case Payload::HouseTop:         return 0.75f;
    case Payload::HouseCoarse:      return 0.28f;
    case Payload::CarTop:           return 0.55f;
    case Payload::CarCoarse:        return 0.20f;
    case Payload::PedestrianTop:    return 0.32f;
    case Payload::PedestrianCoarse: return 0.12f;
    case Payload::TreeTop:          return 0.65f;
    case Payload::TreeCoarse:       return 0.22f;
    case Payload::TowerTop:         return 0.90f;
    case Payload::TowerDistrict:    return 0.70f;
    case Payload::TowerCoarse:      return 0.52f;
    case Payload::TowerMedium:      return 0.35f;
    case Payload::TowerFine:        return 0.18f;
    default:                        return 0.0f;
    }
}

static_assert(authoredGeometricError(Payload::TowerTop) >=
              authoredGeometricError(Payload::TowerDistrict));
static_assert(authoredGeometricError(Payload::TowerDistrict) >=
              authoredGeometricError(Payload::TowerCoarse));
static_assert(authoredGeometricError(Payload::TowerCoarse) >=
              authoredGeometricError(Payload::TowerMedium));
static_assert(authoredGeometricError(Payload::TowerMedium) >=
              authoredGeometricError(Payload::TowerFine));
static_assert(authoredGeometricError(Payload::TowerFine) >=
              authoredGeometricError(Payload::TowerBase));

NodeDesc node(Payload payload, float error, const AABB& nodeBounds,
              uint32_t flags = 0)
{
    NodeDesc result;
    result.payload = UserPayload(payload);
    result.geometricError = error;
    result.flags = flags;
    result.bounds = nodeBounds;
    return result;
}

RepresentationInfo representationInfo(Payload payload)
{
    switch (payload)
    {
    case Payload::HouseTop:         return {"House fallback", 0.05f, true};
    case Payload::HouseCoarse:      return {"House coarse", 0.18f, false};
    case Payload::HouseBody:        return {"House detailed body", 0.65f, false};
    case Payload::HouseRoof:        return {"House detailed roof", 0.35f, false};
    case Payload::CarTop:           return {"Car fallback", 0.02f, true};
    case Payload::CarCoarse:        return {"Car coarse", 0.05f, false};
    case Payload::CarBody:          return {"Car detailed body", 0.10f, false};
    case Payload::CarCabin:         return {"Car detailed cabin", 0.06f, false};
    case Payload::PedestrianTop:    return {"Pedestrian fallback", 0.005f, true};
    case Payload::PedestrianCoarse: return {"Pedestrian coarse", 0.012f, false};
    case Payload::PedestrianBody:   return {"Pedestrian body", 0.025f, false};
    case Payload::PedestrianHead:   return {"Pedestrian head", 0.010f, false};
    case Payload::TreeTop:          return {"Tree fallback", 0.02f, true};
    case Payload::TreeCoarse:       return {"Tree coarse", 0.08f, false};
    case Payload::TreeTrunk:        return {"Tree detailed trunk", 0.12f, false};
    case Payload::TreeCrown:        return {"Tree detailed crown", 0.35f, false};
    case Payload::TowerTop:         return {"Skyscraper fallback", 0.50f, true};
    case Payload::TowerDistrict:    return {"Skyscraper district", 0.75f, false};
    case Payload::TowerCoarse:      return {"Skyscraper coarse", 1.50f, false};
    case Payload::TowerMedium:      return {"Skyscraper medium", 3.00f, false};
    case Payload::TowerFine:        return {"Skyscraper fine", 5.00f, false};
    case Payload::TowerBase:        return {"Skyscraper detailed base", 2.00f, false};
    case Payload::TowerShaft:       return {"Skyscraper detailed shaft", 6.00f, false};
    case Payload::TowerCrown:       return {"Skyscraper detailed crown", 2.00f, false};
    }
    return {};
}

bool isHouseDetailPayload(Payload payload)
{
    return payload >= Payload::HouseCoarse && payload <= Payload::HouseRoof;
}

bool isTowerDetailPayload(Payload payload)
{
    return payload >= Payload::TowerDistrict &&
           payload <= Payload::TowerCrown;
}

constexpr size_t towerRollingFallbackDetailIndex(Payload payload)
{
    if (payload < Payload::TowerDistrict || payload > Payload::TowerCrown)
        return kTowerDetailResourceCount;
    const size_t detailIndex =
        size_t(payload) - size_t(Payload::TowerDistrict);
    if (detailIndex == 0)
        return kTowerDetailResourceCount;
    const size_t fineIndex =
        size_t(Payload::TowerFine) - size_t(Payload::TowerDistrict);
    return std::min(detailIndex - 1, fineIndex);
}

static_assert(towerRollingFallbackDetailIndex(Payload::TowerDistrict) ==
              kTowerDetailResourceCount);
static_assert(towerRollingFallbackDetailIndex(Payload::TowerCoarse) == 0);
static_assert(towerRollingFallbackDetailIndex(Payload::TowerMedium) == 1);
static_assert(towerRollingFallbackDetailIndex(Payload::TowerFine) == 2);
static_assert(towerRollingFallbackDetailIndex(Payload::TowerBase) == 3);
static_assert(towerRollingFallbackDetailIndex(Payload::TowerShaft) == 3);
static_assert(towerRollingFallbackDetailIndex(Payload::TowerCrown) == 3);

EntityKind payloadEntityKind(Payload payload)
{
    if (payload <= Payload::HouseRoof)
        return EntityKind::House;
    if (payload <= Payload::CarCabin)
        return EntityKind::Car;
    if (payload <= Payload::PedestrianHead)
        return EntityKind::Pedestrian;
    if (payload <= Payload::TreeCrown)
        return EntityKind::Tree;
    return EntityKind::Tower;
}

const char* streamingStateName(StreamingResourceState state)
{
    switch (state)
    {
    case StreamingResourceState::Unloaded: return "unloaded";
    case StreamingResourceState::Loading:  return "loading";
    case StreamingResourceState::Resident: return "resident";
    }
    return "unknown";
}

bx::Aabb box(float minX, float minY, float minZ,
             float maxX, float maxY, float maxZ)
{
    return {{minX, minY, minZ}, {maxX, maxY, maxZ}};
}

#ifdef FRONTIER_DEBUG_TOOLS
bx::Aabb debugBox(const AABB& bounds)
{
    return {{bounds.mn.x, bounds.mn.y, bounds.mn.z},
            {bounds.mx.x, bounds.mx.y, bounds.mx.z}};
}

const char* tlasQualityName(TlasQuality quality)
{
    switch (quality)
    {
    case TlasQuality::SpatialBins: return "Spatial bins";
    case TlasQuality::Median: return "Median";
    case TlasQuality::BinnedSAH: return "Binned SAH";
    case TlasQuality::MeanSplit: return "Mean split";
    }
    return "Unknown";
}
#endif

void sampleSidewalkLoop(const PedestrianPath& path, float time,
                        float4& outPosition, float& outYaw)
{
    CarPath sidewalk;
    sidewalk.centerX = path.centerX;
    sidewalk.centerZ = path.centerZ;
    sidewalk.halfX = kSidewalkPathHalfExtent;
    sidewalk.halfZ = kSidewalkPathHalfExtent;
    sidewalk.cornerRadius = kSidewalkCornerRadius;
    sidewalk.phase = path.phase;
    sidewalk.speed = path.speed;
    sidewalk.reverse = path.reverse;
    sampleRoundedLoop(sidewalk, time, outPosition, outYaw);
    outPosition = float4::point(outPosition.x, 0.08f, outPosition.z);
}

class DynamicCity final : public entry::AppI
{
public:
    DynamicCity(const char* name, const char* description, const char* url)
        : entry::AppI(name, description, url), query_(4.0f),
          streamingLookaheadQuery_(4.0f),
          streamingErrorMeasurementQuery_(0.0f)
    {}

    void init(int32_t argc, const char* const* argv, uint32_t width,
              uint32_t height) override
    {
        const Args rendererArgs(argc, argv);
        // Retain the conservative X11 default. Native Wayland/OpenGL selects
        // 4x below, before creating the EGL surface, unless explicitly overridden.
        bool enableMsaa = !BX_ENABLED(BX_PLATFORM_LINUX || BX_PLATFORM_RPI);
        bool msaaSpecified = false;
        bool nativeWindowFrame = false;
        for (int32_t index = 1; index < argc; ++index)
        {
            if (std::strcmp(argv[index], "--no-msaa") == 0)
            {
                enableMsaa = false;
                msaaSpecified = true;
            }
            else if (std::strcmp(argv[index], "--msaa") == 0)
            {
                enableMsaa = true;
                msaaSpecified = true;
            }
            else if (std::strcmp(argv[index], "--native-window-frame") == 0)
            {
                nativeWindowFrame = true;
            }
            else if (std::strcmp(argv[index], "--unlit") == 0)
            {
                unlitSurfaces_ = true;
            }
            else if (std::strcmp(argv[index], "--streaming-self-test") == 0)
            {
                streamingSelfTest_ = true;
            }
            else if (std::strcmp(argv[index],
                                 "--streaming-dynamic-self-test") == 0 ||
                     std::strcmp(argv[index],
                                 "--streaming-orbit-self-test") == 0)
            {
                streamingSelfTest_ = true;
            }
            else if (std::strncmp(argv[index],
                                  "--streaming-test-camera-time=", 29) == 0)
            {
                streamingTestCameraTime_ =
                    std::strtof(argv[index] + 29, nullptr);
            }
            else if (std::strncmp(argv[index],
                                  "--streaming-test-budget=", 24) == 0)
            {
                streamingTestBudgetMiB_ =
                    std::strtof(argv[index] + 24, nullptr);
            }
            else if (std::strncmp(
                         argv[index],
                         "--streaming-test-viewport-height=", 33) == 0)
            {
                streamingTestViewportHeight_ =
                    std::strtof(argv[index] + 33, nullptr);
            }
        }
        width_ = width;
        height_ = height;
        debug_ = BGFX_DEBUG_NONE;

        bgfx::Init init;
        init.type = rendererArgs.m_type;
        init.vendorId = rendererArgs.m_pciId;
        init.platformData.nwh =
            entry::getNativeWindowHandle(entry::kDefaultWindowHandle);
        init.platformData.ndt = entry::getNativeDisplayHandle();
        init.platformData.type = entry::getNativeWindowHandleType();
#if BX_PLATFORM_LINUX || BX_PLATFORM_RPI
        windowSystem_ = init.platformData.type == bgfx::NativeWindowHandleType::Wayland
                            ? "Wayland" : "X11";
#endif
        city::enableWindowFrame(!nativeWindowFrame &&
            init.platformData.type == bgfx::NativeWindowHandleType::Wayland);
        // The Mali-G52/Panfrost report confirms seams with single-sample GL
        // and clean coverage with 4x MSAA on this native window path. Select
        // it at initialization, so plain --gl uses the verified EGL setup.
        if (!msaaSpecified &&
            init.platformData.type == bgfx::NativeWindowHandleType::Wayland &&
            (init.type == bgfx::RendererType::OpenGL ||
             init.type == bgfx::RendererType::Count))
            enableMsaa = true;
        reset_ = (enableMsaa ? BGFX_RESET_MSAA_X4 : 0) |
                 (streamingSelfTest_ ? 0 : BGFX_RESET_VSYNC);
        init.resolution.width = width_;
        init.resolution.height = height_;
        init.resolution.reset = reset_;
        if (!bgfx::init(init))
        {
            std::fprintf(stderr, "ERROR: Failed to initialize the city renderer.\n");
            std::exit(EXIT_FAILURE);
        }
        const bgfx::Caps* rendererCaps = bgfx::getCaps();
        cpuModel_ = city::cpuModel();
        gpuModel_ = rendererCaps->frontierGpuName[0] != '\0'
                        ? rendererCaps->frontierGpuName : "Model unavailable";
        if (bgfx::getRendererType() == bgfx::RendererType::OpenGL ||
            bgfx::getRendererType() == bgfx::RendererType::OpenGLES)
            gpuDriver_ = rendererCaps->frontierGpuDriver;
        std::printf("Frontier city renderer: %s | requested MSAA: %s | "
                    "GPU: %04x:%04x\n",
                    bgfx::getRendererName(bgfx::getRendererType()),
                    enableMsaa ? "4x" : "off",
                    unsigned(rendererCaps->vendorId),
                    unsigned(rendererCaps->deviceId));
        std::printf("CPU: %s\nGPU: %s\n", cpuModel_.c_str(), gpuModel_.c_str());
        if (!windowSystem_.empty())
            std::printf("Window system: %s\n", windowSystem_.c_str());
        if (!gpuDriver_.empty())
            std::printf("Graphics driver: %s\n", gpuDriver_.c_str());
        std::fflush(stdout);

        bgfx::setDebug(debug_);
        bgfx::setViewClear(kMainView,
                           BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                           0x86b8d8ff, 1.0f, 0);
        bgfx::setViewMode(kMainView, bgfx::ViewMode::Sequential);
        entry::setWindowTitle(entry::kDefaultWindowHandle,
                              "Frontier - Dynamic City");

        ddInit();
        imguiCreate();
        ImGui::GetStyle().TreeLinesFlags =
            ImGuiTreeNodeFlags_DrawLinesToNodes;
        cameraCreate();
        createRoofGeometry();
        createScene();
        initializeVirtualStreaming();
        if (streamingSelfTest_)
            startHeroPressureScenario();
        previousCounter_ = bx::getHPCounter();
    }

    int shutdown() override
    {
        if (isValid(roofGeometry_))
            ddDestroy(roofGeometry_);
        cameraDestroy();
        imguiDestroy();
        ddShutdown();
        bgfx::shutdown();
        return streamingSelfTestExitCode_;
    }

    bool update() override
    {
        const uint32_t previousWidth = width_;
        const uint32_t previousHeight = height_;
        const uint32_t previousReset = reset_;
        if (entry::processEvents(width_, height_, debug_, reset_, &mouse_))
            return false;
        if (width_ != previousWidth || height_ != previousHeight ||
            reset_ != previousReset)
        {
            // entry::processEvents gates its reset on the last event's window
            // handle. A later global key/modifier event can hide a resize.
            // bgfx ignores an identical reset if the entry layer already did it.
            bgfx::reset(width_, height_, reset_);
        }

        const bool leftMouseDown =
            mouse_.m_buttons[entry::MouseButton::Left] != 0;
        const uint8_t modifiers = inputGetModifiersState();
        const bool controlDown =
            (modifiers & (entry::Modifier::LeftCtrl |
                          entry::Modifier::RightCtrl)) != 0;
        const bool towerPickRequested =
            leftMouseDown && !leftMouseWasDown_ && controlDown;
        leftMouseWasDown_ = leftMouseDown;

        const int64_t now = bx::getHPCounter();
        const double frequency = double(bx::getHPFrequency());
        const float deltaTime = streamingSelfTest_
                                    ? 0.1f
                                    : std::clamp(
                                          float(double(now - previousCounter_) /
                                                frequency),
                                          1.0e-4f, 0.1f);
        previousCounter_ = now;
        smoothedFps_ += ((1.0f / deltaTime) - smoothedFps_) * 0.05f;

        PerformanceSample performance;
        int64_t stageStart = now;

        beginDebugUi();
        city::drawWindowFrame();
        drawGlobalMenuBar();
        if (showFrontierDebug_)
            drawDebugUi();
        if (showTlasMaintenance_)
            drawTlasMaintenanceUi();
        if (showVirtualStreaming_)
            drawVirtualStreamingUi();
        if (showSceneStats_)
            drawSceneStatsUi();
        if (showPerformance_)
            drawPerformanceUi();
        if (showSceneHierarchy_)
            drawSceneTreeUi();
        else
        {
            sceneTreeSelection_ = {};
            pendingHierarchyRevealHero_ = UINT16_MAX;
            pendingHierarchyRevealInstance_ = kInvalidInstanceId;
            currentInstanceEntryCounts_.clear();
            sceneTreeStatsValid_ = false;
        }
#ifdef FRONTIER_DEBUG_TOOLS
        if (showTlasHealth_)
            drawTlasHealthUi();
        if (showQueryCache_)
            drawQueryCacheUi();
#endif
        const bool uiHasFocus = ImGui::MouseOverArea();
        imguiEndFrame();
        int64_t stageEnd = bx::getHPCounter();
        performance.uiMs = milliseconds(stageStart, stageEnd);

        ++motionFrame_;
        updatedHlodNodes_ = 0;
        movedInstances_ = 0;
        stageStart = stageEnd;
        if (streamingResetRequested_)
        {
            if (virtualStreamingEnabled_)
                resetVirtualStreaming();
            else
                makeAllVirtualResourcesResident(false);
            streamingResetRequested_ = false;
        }
        if (heroPressureScenarioRequested_)
        {
            startHeroPressureScenario();
            heroPressureScenarioRequested_ = false;
        }
        if (houseReplacementPending_)
        {
            replaceHouses(pendingHouseStyle_);
            houseReplacementPending_ = false;
            if (!virtualStreamingEnabled_)
                makeAllVirtualResourcesResident(false);
        }
        if (virtualStreamingToggleRequested_)
        {
            virtualStreamingToggleRequested_ = false;
            virtualStreamingEnabled_ = !virtualStreamingEnabled_;
            streamingPaused_ = false;
            if (virtualStreamingEnabled_)
            {
                resetVirtualStreaming();
                appendStreamingLog(
                    ImVec4(0.50f, 0.85f, 1.0f, 1.0f),
                    "VIRTUAL STREAMING enabled: reset to coarsest residency");
            }
            else
            {
                heroPressureScenarioActive_ = false;
                makeAllVirtualResourcesResident(true);
            }
        }
        if (restoreSceneAfterStress_)
        {
            updateWholeSceneWave(simulationTime_, 0.0f);
            restoreSceneAfterStress_ = false;
        }
        if (!freezeSimulation_)
        {
            simulationTime_ += deltaTime;
            if (animateWholeScene_)
            {
                updateWholeSceneWave(simulationTime_,
                                     kWorstCaseWaveAmplitude);
            }
            else
            {
                updateActors(simulationTime_);
            }
        }
        stageEnd = bx::getHPCounter();
        performance.simulationMs = milliseconds(stageStart, stageEnd);

        stageStart = stageEnd;
        const uint32_t maintenanceBudget = unlimitedTlasMaintenance_
                                               ? kUnlimitedTlasMaintenance
                                               : uint32_t(std::max(
                                                     tlasMaintenanceBudget_,
                                                     0));
        lastUpdateReport_ = database_.applyUpdates(maintenanceBudget);
        stageEnd = bx::getHPCounter();
        performance.applyUpdatesMs = milliseconds(stageStart, stageEnd);

        const bool rebuildScheduleEnabled =
            rebuildStrategy_ != RebuildStrategy::Manual;
        if (rebuildScheduleEnabled)
            timeSinceRebuild_ += deltaTime;
        else
            timeSinceRebuild_ = 0.0f;
        const bool scheduledCheckDue =
            rebuildScheduleEnabled &&
            timeSinceRebuild_ >= rebuildIntervalSeconds_;
        bool scheduledRebuild = false;
        if (scheduledCheckDue)
        {
            timeSinceRebuild_ = 0.0f;
            if (rebuildStrategy_ == RebuildStrategy::Periodic)
            {
                scheduledRebuild = true;
            }
            else
            {
                ++rebuildRecommendationChecks_;
                scheduledRebuild =
                    lastUpdateReport_.topologyRebuildRecommended;
                if (!scheduledRebuild)
                    ++rebuildRecommendationSkips_;
            }
        }

        const bool manualFull = fullOptimizationRequested_;
        const bool manualTopology = topologyOptimizationRequested_;
        if (manualFull || manualTopology || scheduledRebuild)
        {
            const OptimizationMode mode = manualFull
                ? OptimizationMode::TopologyAndLayout
                : manualTopology
                    ? OptimizationMode::TopologyOnly
                    : scheduledOptimizationMode_;
            stageStart = stageEnd;
            database_.optimize(mode);
            stageEnd = bx::getHPCounter();
            performance.tlasRebuildMs = milliseconds(stageStart, stageEnd);
            lastRebuildMs_ = performance.tlasRebuildMs;
            lastOptimizationMode_ = mode;
            ++rebuildCount_;
            if (mode == OptimizationMode::TopologyAndLayout)
                ++fullOptimizationCount_;
            else
                ++topologyOptimizationCount_;
            lastUpdateReport_.maintenanceNodesPending = 0;
            lastUpdateReport_.areaGrowthRatio = 0.0f;
            lastUpdateReport_.topologyRebuildRecommended = false;
            lastRebuildTrigger_ = manualFull || manualTopology
                                      ? RebuildStrategy::Manual
                                      : rebuildStrategy_;
            fullOptimizationRequested_ = false;
            topologyOptimizationRequested_ = false;
            timeSinceRebuild_ = 0.0f;
#ifdef FRONTIER_DEBUG_TOOLS
            tlasHealthValid_ = false;
#endif
        }

        stageStart = stageEnd;
        cameraTime_ += deltaTime;
        CameraPose automaticPose;
        updateAutomaticCamera(cameraTime_, automaticPose.position,
                              automaticPose.target);
        if (heroPressureScenarioActive_)
        {
            const float orbitTime =
                streamingTime_ - heroPressureScenarioStartTime_ +
                heroPressureOrbitPhaseOffset_;
            updateHeroPressureOrbitCamera(
                orbitTime, automaticPose.position, automaticPose.target);
        }
        if (seedFreeCamera_)
        {
            seedFreeCameraFrom(automaticPose.position, automaticPose.target);
            seedFreeCamera_ = false;
        }
        if (freeCamera_)
            cameraUpdate(deltaTime, mouse_, uiHasFocus);

        CameraPose displayPose = freeCamera_
                                     ? makeFreeCameraPose()
                                     : makeCameraPose(automaticPose.position,
                                                      automaticPose.target);
        if (towerPickRequested && !uiHasFocus && !streamingSelfTest_)
            pickTowerAtMouse(displayPose);
        if (captureCullCamera_)
        {
            captureFrozenCull(displayPose);
            captureCullCamera_ = false;
        }

        bgfx::setViewTransform(kMainView, displayPose.view.data(),
                               displayPose.projection.data());
        bgfx::setViewRect(kMainView, 0, 0,
                          uint16_t(std::min(width_, uint32_t(UINT16_MAX))),
                          uint16_t(std::min(height_, uint32_t(UINT16_MAX))));
        bgfx::touch(kMainView);
        stageEnd = bx::getHPCounter();
        performance.cameraMs = milliseconds(stageStart, stageEnd);

        stageStart = stageEnd;
        const Camera frontierCamera = freezeCullCamera_ && frozenCull_.valid
                                          ? frozenCull_.camera
                                          : makeFrontierCamera(displayPose);
        const SelectionParams selection{
            .threshold = lodThreshold_,
            .minPix = contributionCullPixels_,
            .currentCutPolicy =
                !virtualStreamingEnabled_ ||
                        streamingCutStrategy_ ==
                            StreamingCutStrategy::QualityPerByte
                    ? CurrentCutPolicy::PreferReadyAncestors
                    : CurrentCutPolicy::PreferReadyDescendants,
        };
        const FrontierResultView frontier =
            query_.selectFrontier(database_, frontierCamera, selection);
        FrontierResultView prefetchFrontier{};
        if (heroPressureScenarioActive_)
        {
            float4 prefetchPosition;
            float4 prefetchTarget;
            const float prefetchTime =
                streamingTime_ - heroPressureScenarioStartTime_ +
                heroPressureOrbitPhaseOffset_ +
                kHeroOrbitLookaheadSeconds;
            updateHeroPressureOrbitCamera(
                prefetchTime, prefetchPosition, prefetchTarget);
            prefetchFrontier = streamingLookaheadQuery_.selectFrontier(
                database_,
                makeFrontierCamera(
                    makeCameraPose(prefetchPosition, prefetchTarget)),
                selection);
        }
        observeHeroPressureFrontier(frontier);
        stageEnd = bx::getHPCounter();
        performance.selectionMs = milliseconds(stageStart, stageEnd);

        stageStart = stageEnd;
        updateFrontierStats(frontier);
#ifdef FRONTIER_DEBUG_TOOLS
        if (showQueryCache_)
            recordQueryCacheHistory();
#endif
        stageEnd = bx::getHPCounter();
        performance.cutStatsMs = milliseconds(stageStart, stageEnd);

        stageStart = stageEnd;
        render(frontier);
        stageEnd = bx::getHPCounter();
        performance.renderMs = milliseconds(stageStart, stageEnd);

        stageStart = stageEnd;
        updateVirtualStreaming(frontier, prefetchFrontier, frontierCamera,
                               selection, deltaTime, performance);
        const int64_t streamingPlannerEnd = bx::getHPCounter();
        updateHeroPressureScenario();
        stageEnd = bx::getHPCounter();
        const float streamingUpdateMs =
            milliseconds(stageStart, streamingPlannerEnd);
        performance.streamingPlannerMs = std::max(
            0.0f, streamingUpdateMs - performance.streamingRefinementMs);
        const float accountedPlannerMs =
            performance.streamingPlannerIndexMs +
            performance.streamingPlannerDemandMs +
            performance.streamingPlannerScoreMs +
            performance.streamingPlannerResidencyMs;
        performance.streamingPlannerOtherMs = std::max(
            0.0f, performance.streamingPlannerMs - accountedPlannerMs);
        performance.streamingScenarioMs =
            milliseconds(streamingPlannerEnd, stageEnd);
        performance.streamingMs = milliseconds(stageStart, stageEnd);

        stageStart = stageEnd;
        bgfx::frame();
        stageEnd = bx::getHPCounter();
        performance.frameSubmitMs = milliseconds(stageStart, stageEnd);
        performance.totalMs = milliseconds(now, stageEnd);
        captureBgfxPerformance(performance);
        recordPerformance(performance, deltaTime);
        return !streamingSelfTestFinished_;
    }

private:
    void beginDebugUi()
    {
        const uint8_t buttons =
            (mouse_.m_buttons[entry::MouseButton::Left] ? IMGUI_MBUT_LEFT : 0) |
            (mouse_.m_buttons[entry::MouseButton::Right] ? IMGUI_MBUT_RIGHT : 0) |
            (mouse_.m_buttons[entry::MouseButton::Middle] ? IMGUI_MBUT_MIDDLE : 0);
        imguiBeginFrame(
            mouse_.m_mx, mouse_.m_my, buttons, mouse_.m_mz,
            uint16_t(std::min(width_, uint32_t(UINT16_MAX))),
            uint16_t(std::min(height_, uint32_t(UINT16_MAX))));
    }

    void drawGlobalMenuBar()
    {
        if (!ImGui::BeginMainMenuBar())
            return;

        if (ImGui::BeginMenu("Debug windows"))
        {
            ImGui::MenuItem("Frontier controls", nullptr,
                            &showFrontierDebug_);
            ImGui::MenuItem("TLAS maintenance", nullptr,
                            &showTlasMaintenance_);
            ImGui::MenuItem("Virtual streaming", nullptr,
                            &showVirtualStreaming_);
            ImGui::MenuItem("Scene stats", nullptr, &showSceneStats_);
            ImGui::MenuItem("Performance", nullptr, &showPerformance_);
            ImGui::MenuItem("Scene hierarchy", nullptr,
                            &showSceneHierarchy_);
#ifdef FRONTIER_DEBUG_TOOLS
            ImGui::MenuItem("TLAS health", nullptr, &showTlasHealth_);
            ImGui::MenuItem("Query cache", nullptr, &showQueryCache_);
#endif
            ImGui::Separator();
            if (ImGui::MenuItem("Show all"))
            {
                showFrontierDebug_ = true;
                showTlasMaintenance_ = true;
                showVirtualStreaming_ = true;
                showSceneStats_ = true;
                showPerformance_ = true;
                showSceneHierarchy_ = true;
#ifdef FRONTIER_DEBUG_TOOLS
                showTlasHealth_ = true;
                showQueryCache_ = true;
#endif
            }
            if (ImGui::MenuItem("Hide all"))
            {
                showFrontierDebug_ = false;
                showTlasMaintenance_ = false;
                showVirtualStreaming_ = false;
                showSceneStats_ = false;
                showPerformance_ = false;
                showSceneHierarchy_ = false;
#ifdef FRONTIER_DEBUG_TOOLS
                showTlasHealth_ = false;
                showQueryCache_ = false;
#endif
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Rendering"))
        {
            ImGui::MenuItem("Wireframe scene", nullptr, &wireframeDebug_);
            ImGui::MenuItem("Hierarchy level tint", nullptr,
                            &hierarchyTint_);
#ifdef FRONTIER_DEBUG_TOOLS
            ImGui::Separator();
            ImGui::MenuItem("TLAS AABBs by depth", nullptr,
                            &drawTlasAabbs_);
            ImGui::MenuItem("Loose vs exact bounds", nullptr,
                            &drawLooseBounds_);
#endif
            ImGui::EndMenu();
        }
        ImGui::TextDisabled("Frontier Dynamic City");
        ImGui::EndMainMenuBar();
    }

    static const char* rebuildStrategyName(RebuildStrategy strategy)
    {
        switch (strategy)
        {
        case RebuildStrategy::Manual: return "manual";
        case RebuildStrategy::Periodic: return "periodic";
        case RebuildStrategy::WhenRecommended:
            return "when recommended";
        }
        return "unknown";
    }

    static const char* optimizationModeName(OptimizationMode mode)
    {
        return mode == OptimizationMode::TopologyOnly
                   ? "topology only"
                   : "topology + layout";
    }

    void drawDebugUi()
    {
        ImGui::SetNextWindowPos(ImVec2(12.0f, 36.0f + city::windowFrameHeight()), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(390.0f, 660.0f),
                                 ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Frontier debug", &showFrontierDebug_))
        {
            ImGui::End();
            return;
        }

        ImGui::Text("Backend: %s",
                    bgfx::getRendererName(bgfx::getRendererType()));
        if (!windowSystem_.empty())
        {
            ImGui::Text("Window system: %s", windowSystem_.c_str());
            if (windowSystem_ == "Wayland" && ImGui::IsItemHovered())
                ImGui::SetTooltip(city::windowFrameHeight() > 0.0f
                    ? "Drag the title bar to move; drag an edge or corner to resize."
                    : "Native frame: Alt + left drag to move; Alt + Shift + left drag to resize.");
        }
        ImGui::TextWrapped("CPU: %s", cpuModel_.c_str());
        ImGui::TextWrapped("GPU: %s", gpuModel_.c_str());
        if (ImGui::IsItemHovered() && !gpuDriver_.empty())
            ImGui::SetTooltip("%s", gpuDriver_.c_str());
        ImGui::Separator();
        ImGui::Text("Simulation");
        ImGui::Separator();
        ImGui::Checkbox("Freeze simulation", &freezeSimulation_);
        ImGui::Checkbox("Hierarchy level tint", &hierarchyTint_);
        ImGui::Checkbox("Wireframe scene rendering", &wireframeDebug_);
        ImGui::Checkbox("Unlit surfaces (seam test)", &unlitSurfaces_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Disable lighting without changing geometry or depth.\n"
                              "If the seams disappear, they come from shading.");
        if (hierarchyTint_)
            ImGui::TextWrapped(
                "Tint follows the selected cut. Higher LOD thresholds expose "
                "more green and yellow nodes; lower values refine toward red.");
        ImGui::SliderFloat("LOD threshold (px)", &lodThreshold_, 0.5f, 12.0f,
                           "%.1f");
        ImGui::SliderFloat("Contribution cull (px)",
                           &contributionCullPixels_, 0.0f, 100.0f, "%.2f");

        ImGui::Text("Workloads");
        ImGui::Separator();
        ImGui::Text("Active houses: %s | generation %u",
                    activeHouseStyle_ == HouseStyle::HouseA
                        ? "House A"
                        : "House B",
                    houseGeneration_);
        if (ImGui::Button("Replace all with House A"))
        {
            pendingHouseStyle_ = HouseStyle::HouseA;
            houseReplacementPending_ = true;
        }
        if (ImGui::Button("Replace all with House B"))
        {
            pendingHouseStyle_ = HouseStyle::HouseB;
            houseReplacementPending_ = true;
        }
        if (houseReplacementPending_)
            ImGui::TextDisabled("Replacement queued for this frame");

        const char* stressLabel = animateWholeScene_
                                      ? "Stop stress test"
                                      : "Start stress test";
        if (ImGui::Button(stressLabel))
        {
            animateWholeScene_ = !animateWholeScene_;
            if (!animateWholeScene_)
                restoreSceneAfterStress_ = true;
        }
        if (animateWholeScene_)
            ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.20f, 1.0f),
                               "Every instance has independent wave motion");

        ImGui::Text("Camera");
        ImGui::Separator();
        bool requestedFreeCamera = freeCamera_;
        if (ImGui::Checkbox("Free camera (WASD/QE + RMB)",
                            &requestedFreeCamera))
        {
            if (!freezeCullCamera_ || requestedFreeCamera)
            {
                if (requestedFreeCamera && !freeCamera_)
                    seedFreeCamera_ = true;
                freeCamera_ = requestedFreeCamera;
                query_.reset();
                streamingLookaheadQuery_.reset();
            }
        }

        bool requestedFreezeCull = freezeCullCamera_;
        if (ImGui::Checkbox("Freeze camera / cull state",
                            &requestedFreezeCull))
        {
            freezeCullCamera_ = requestedFreezeCull;
            if (freezeCullCamera_)
            {
                if (!freeCamera_)
                    seedFreeCamera_ = true;
                freeCamera_ = true;
                captureCullCamera_ = true;
                drawCullFrustum_ = true;
            }
            else
            {
                frozenCull_.valid = false;
                query_.reset();
                streamingLookaheadQuery_.reset();
            }
        }
        if (freezeCullCamera_)
        {
            freeCamera_ = true;
            ImGui::Checkbox("Visualize frozen frustum planes",
                            &drawCullFrustum_);
        }
        if (ImGui::Button("Reset free camera to auto view"))
        {
            freeCamera_ = true;
            seedFreeCamera_ = true;
        }

        ImGui::Text("Hierarchy tint legend");
        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.18f, 0.90f, 0.26f, 1.0f),
                           "Top / fallback");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.10f, 1.0f), "Middle");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.95f, 0.15f, 0.08f, 1.0f), "Leaves");
        if (freezeCullCamera_ && frozenCull_.valid)
            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.8f, 1.0f),
                               "Culling from frozen magenta frustum");
        ImGui::End();
    }

    void drawTlasMaintenanceUi()
    {
        ImGui::SetNextWindowPos(ImVec2(414.0f, 36.0f + city::windowFrameHeight()),
                                ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(390.0f, 480.0f),
                                 ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("TLAS maintenance", &showTlasMaintenance_))
        {
            ImGui::End();
            return;
        }

        ImGui::Text("applyUpdates");
        ImGui::Separator();
        ImGui::Checkbox("Unlimited applyUpdates budget",
                        &unlimitedTlasMaintenance_);
        if (!unlimitedTlasMaintenance_)
            ImGui::SliderInt("Repair nodes / applyUpdates",
                             &tlasMaintenanceBudget_, 0, 16384);
        ImGui::Text("Last apply: %u repaired | %u pending",
                    lastUpdateReport_.maintenanceNodesProcessed,
                    lastUpdateReport_.maintenanceNodesPending);
        ImGui::Text("Stored area growth %.1f%%",
                    lastUpdateReport_.areaGrowthRatio * 100.0f);
        if (lastUpdateReport_.requiredBuildPerformed)
            ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.20f, 1.0f),
                               "TLAS health: correctness build performed");
        else if (lastUpdateReport_.topologyRebuildRecommended)
            ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.20f, 1.0f),
                               "TLAS health: topology rebuild recommended");
        else
            ImGui::TextColored(ImVec4(0.35f, 0.85f, 0.45f, 1.0f),
                               "TLAS health: OK; rebuild not needed");

        ImGui::Text("Topology rebuild");
        ImGui::Separator();
        ImGui::Text("Strategy");
        if (ImGui::RadioButton(
                "Manual only",
                rebuildStrategy_ == RebuildStrategy::Manual))
        {
            rebuildStrategy_ = RebuildStrategy::Manual;
            timeSinceRebuild_ = 0.0f;
        }
        if (ImGui::RadioButton(
                "Periodic",
                rebuildStrategy_ == RebuildStrategy::Periodic))
        {
            rebuildStrategy_ = RebuildStrategy::Periodic;
            timeSinceRebuild_ = 0.0f;
        }
        if (ImGui::RadioButton(
                "When recommended",
                rebuildStrategy_ == RebuildStrategy::WhenRecommended))
        {
            rebuildStrategy_ = RebuildStrategy::WhenRecommended;
            timeSinceRebuild_ = 0.0f;
        }

        if (rebuildStrategy_ == RebuildStrategy::Manual)
        {
            ImGui::TextDisabled(
                "Use either button below to rebuild explicitly");
        }
        else
        {
            ImGui::SliderFloat("Strategy interval (seconds)",
                               &rebuildIntervalSeconds_, 0.25f, 60.0f,
                               "%.2f");
            const float remaining = std::max(
                0.0f, rebuildIntervalSeconds_ - timeSinceRebuild_);
            if (rebuildStrategy_ == RebuildStrategy::Periodic)
            {
                ImGui::TextWrapped(
                    "Rebuilds every interval, regardless of advice");
                ImGui::Text("Next rebuild in %.2f s", remaining);
            }
            else
            {
                ImGui::TextWrapped(
                    "Checks UpdateReport every interval and rebuilds "
                    "only when recommended");
                ImGui::Text("Next recommendation check in %.2f s", remaining);
            }
            ImGui::Text("Scheduled mode");
            if (ImGui::RadioButton(
                    "Topology only (spatial bins, preserves layout)",
                    scheduledOptimizationMode_ ==
                        OptimizationMode::TopologyOnly))
                scheduledOptimizationMode_ = OptimizationMode::TopologyOnly;
            if (ImGui::RadioButton(
                    "Topology + layout (configured quality + compaction)",
                    scheduledOptimizationMode_ ==
                        OptimizationMode::TopologyAndLayout))
                scheduledOptimizationMode_ =
                    OptimizationMode::TopologyAndLayout;
        }
        if (ImGui::Button("Optimize topology now"))
            topologyOptimizationRequested_ = true;
        ImGui::SameLine();
        if (ImGui::Button("Optimize topology + layout now"))
            fullOptimizationRequested_ = true;
        if (rebuildCount_ == 0)
        {
            ImGui::Text("No runtime topology rebuilds");
        }
        else
        {
            ImGui::Text("%llu rebuilds | last %.1f us (%s, %s)",
                        static_cast<unsigned long long>(rebuildCount_),
                        lastRebuildMs_ * 1000.0f,
                        optimizationModeName(lastOptimizationMode_),
                        rebuildStrategyName(lastRebuildTrigger_));
            ImGui::Text(
                "topology only %llu | topology + layout %llu",
                static_cast<unsigned long long>(topologyOptimizationCount_),
                static_cast<unsigned long long>(fullOptimizationCount_));
        }
        if (rebuildStrategy_ == RebuildStrategy::WhenRecommended ||
            rebuildRecommendationChecks_ != 0)
        {
            ImGui::Text("Recommendation checks %llu | skipped %llu",
                        static_cast<unsigned long long>(
                            rebuildRecommendationChecks_),
                        static_cast<unsigned long long>(
                            rebuildRecommendationSkips_));
        }

        ImGui::End();
    }

    void drawVirtualStreamingUi()
    {
        ImGui::SetNextWindowPos(ImVec2(520.0f, 36.0f + city::windowFrameHeight()),
                                ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(760.0f, 680.0f),
                                 ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Virtual streaming", &showVirtualStreaming_))
        {
            ImGui::End();
            return;
        }

        ImGui::TextWrapped(
            "computeFrontierRefinement() supplies complete child groups; "
            "this sample simulates their memory, latency, readiness, and "
            "eviction without loading real files. Sizes are per reusable "
            "representation resource, shared by all of its placements.");
        ImGui::Separator();
        if (ImGui::Button(virtualStreamingEnabled_
                              ? "Turn off virtual streaming"
                              : "Turn on virtual streaming"))
            virtualStreamingToggleRequested_ = true;
        ImGui::SameLine();
        ImGui::TextColored(
            virtualStreamingEnabled_
                ? ImVec4(0.35f, 0.85f, 0.45f, 1.0f)
                : ImVec4(0.35f, 0.72f, 1.0f, 1.0f),
            "Virtual streaming: %s",
            virtualStreamingEnabled_ ? "ON" : "OFF - everything resident");
        if (!virtualStreamingEnabled_)
        {
            ImGui::TextWrapped(
                "All active representation resources are immediately ready. "
                "The memory budget, latency, and eviction policy are ignored "
                "until virtual streaming is turned on again.");
        }
        int cutStrategy = int(streamingCutStrategy_);
        const char* cutStrategies[] = {
            "Quality per byte (coarsen)",
            "Retain ready detail (sticky)",
        };
        if (ImGui::Combo("Residency cut strategy", &cutStrategy,
                         cutStrategies, int(std::size(cutStrategies))))
        {
            streamingCutStrategy_ = StreamingCutStrategy(cutStrategy);
            query_.reset();
            streamingLookaheadQuery_.reset();
            appendStreamingLog(
                ImVec4(0.50f, 0.85f, 1.0f, 1.0f),
                streamingCutStrategy_ == StreamingCutStrategy::QualityPerByte
                    ? "POLICY quality-per-byte: threshold target or ready "
                      "ancestor; over-detail may be reclaimed"
                    : "POLICY sticky detail: prefer complete ready "
                      "descendants");
        }
        if (streamingCutStrategy_ == StreamingCutStrategy::QualityPerByte)
        {
            ImGui::TextWrapped(
                "Quality-per-byte uses PreferReadyAncestors. A previously "
                "loaded descendant cannot override the camera's coarser LOD "
                "target, so stale over-detail leaves the current cut and can "
                "fund a more valuable refinement elsewhere.");
        }
        else
        {
            ImGui::TextWrapped(
                "Sticky detail uses PreferReadyDescendants for comparison. "
                "It preserves a complete ready descendant cover, which can "
                "retain over-detail and starve higher-value loads under a "
                "tight budget.");
        }
        ImGui::SliderFloat("Memory budget (MiB)",
                           &virtualMemoryBudgetMiB_, 0.5f, 512.0f, "%.1f");
        const float detailedHeroChildrenMiB =
            representationInfo(Payload::TowerBase).byteSizeMiB +
            representationInfo(Payload::TowerShaft).byteSizeMiB +
            representationInfo(Payload::TowerCrown).byteSizeMiB;
        const float oneHeroTransitionBudgetMiB =
            virtualPinnedMiB() +
            representationInfo(Payload::TowerFine).byteSizeMiB +
            detailedHeroChildrenMiB;
        if (ImGui::Button("Set one-hero transition budget"))
            virtualMemoryBudgetMiB_ = oneHeroTransitionBudgetMiB;
        ImGui::SameLine();
        ImGui::TextDisabled("%.2f MiB", oneHeroTransitionBudgetMiB);
        ImGui::TextWrapped(
            "The one-hero preset covers pinned fallbacks plus one 5 MiB "
            "fine parent and its complete 10 MiB detailed child group. "
            "Other representations still compete for the same budget.");
        ImGui::SliderFloat("Load latency (seconds)",
                           &streamingLatencySeconds_, 0.0f, 5.0f, "%.2f");
        ImGui::SliderFloat("Unload delay (seconds)",
                           &streamingUnloadDelaySeconds_, 0.0f, 15.0f,
                           "%.2f");
        ImGui::SliderInt("Concurrent load groups",
                         &maxConcurrentStreamingLoads_, 1, 16);
        ImGui::Checkbox("Pause virtual streaming", &streamingPaused_);
        if (ImGui::Button("Reset to coarsest residency"))
            streamingResetRequested_ = true;
        ImGui::SameLine();
        if (ImGui::Button("Clear log"))
            streamingLog_.clear();

        ImGui::Text("Regression scenario");
        ImGui::Separator();
        if (!heroPressureScenarioActive_)
        {
            if (ImGui::Button("Start close skyscraper orbit test"))
                heroPressureScenarioRequested_ = true;
        }
        else if (ImGui::Button("Stop hero pressure test"))
        {
            heroPressureScenarioActive_ = false;
            heroPressureScenarioPassed_ = false;
            heroPressureScenarioFinished_ = true;
        }
        ImGui::TextWrapped(
            "Slowly orbits the captured central skyscraper at close range "
            "while the simulation runs normally. It resets residency, applies "
            "an 85 MiB budget, and fails if the focal, screen-dominant hero "
            "or any other >150 px-tall hero drops to its pinned fallback after "
            "a 20 second warm-up. It also requires rolling fallback release "
            "coverage and rejects rapid unload/reload loops.");
        if (heroPressureScenarioActive_)
        {
            const float elapsed =
                streamingTime_ - heroPressureScenarioStartTime_;
            ImGui::TextColored(
                ImVec4(1.0f, 0.78f, 0.20f, 1.0f),
                "RUNNING %.1f / %.0f s | %u resource-state transitions",
                elapsed, kHeroOrbitTestSeconds,
                heroPressureScenarioTransitions_);
            ImGui::Text("Focal LOD: %s | fallback frames after warm-up: %u",
                        towerLodName(heroPressureLastFocalRank_),
                        heroPressureFocalFallbackFrames_);
            ImGui::Text(
                "All dominant heroes: fallback frames %u | max at once %u",
                heroPressureDominantFallbackFrames_,
                heroPressureMaxDominantFallbacks_);
            ImGui::Text(
                "Rapid reloads: %u total | worst resource %u / %u",
                heroPressureRapidReloads_,
                heroPressureMaxResourceRapidReloads_,
                kHeroOrbitMaxRapidReloads);
            ImGui::Text("Worst per-resource state transitions: %u",
                        heroPressureMaxResourceTransitions_);
            ImGui::Text(
                "Make-before-break: invalidations %llu | error regressions %u",
                static_cast<unsigned long long>(
                    streamingCurrentCutInvalidations_ -
                    heroPressureScenarioStartCurrentCutInvalidations_),
                heroPressureGeometricErrorRegressions_);
            ImGui::Text(
                "Rolling fallback: observed %u | old ladders released %u | "
                "violations %u",
                heroPressureRollingPinObservations_,
                heroPressureRollingReleaseObservations_,
                heroPressureRollingPinViolations_);
            ImGui::Text(
                "LOD error measurements %llu | positive %llu | regressions "
                "%llu",
                static_cast<unsigned long long>(
                    streamingErrorMeasurements_ -
                    heroPressureScenarioStartErrorMeasurements_),
                static_cast<unsigned long long>(
                    streamingPositiveErrorMeasurements_ -
                    heroPressureScenarioStartPositiveErrorMeasurements_),
                static_cast<unsigned long long>(
                    streamingErrorMeasurementRegressions_ -
                    heroPressureScenarioStartErrorMeasurementRegressions_));
        }
        else if (heroPressureScenarioFinished_)
        {
            const char* resultText =
                "PASS: dominant heroes retained streamed LODs";
            if (!heroPressureScenarioPassed_)
            {
                if (!heroPressureScenarioWithinBudget_)
                    resultText = "FAIL: committed memory exceeded budget";
                else if (heroPressureScenarioCurrentCutInvalidations_ != 0)
                    resultText = "FAIL: loading invalidated current cut";
                else if (heroPressureGeometricErrorRegressions_ != 0)
                    resultText = "FAIL: visible geometric error regressed";
                else if (
                    heroPressureScenarioAtomicPublicationViolations_ != 0)
                    resultText = "FAIL: streaming group published partially";
                else if (heroPressureRollingPinViolations_ != 0)
                    resultText = "FAIL: fallback pin retained stale ancestors";
                else if (heroPressureRollingPinObservations_ == 0 ||
                         heroPressureRollingReleaseObservations_ == 0)
                    resultText = "FAIL: rolling fallback release not covered";
                else if (heroPressureScenarioErrorMeasurementRegressions_ != 0)
                    resultText = "FAIL: measured LOD error increased";
                else if (heroPressureScenarioErrorMeasurements_ == 0 ||
                         heroPressureScenarioPositiveErrorMeasurements_ == 0)
                    resultText = "FAIL: LOD error measurement not covered";
                else if (heroPressureFocalObservedFrames_ == 0)
                    resultText = "FAIL: focal hero was never observed";
                else if (heroPressureFocalFallbackFrames_ != 0 ||
                         heroPressureDominantFallbackFrames_ != 0)
                    resultText =
                        "FAIL: screen-dominant hero reached fallback";
                else if (heroPressureMaxResourceRapidReloads_ >
                         kHeroOrbitMaxRapidReloads)
                    resultText = "FAIL: repeated rapid resource reload loop";
                else
                    resultText = "FAIL: regression invariant failed";
            }
            ImGui::TextColored(
                heroPressureScenarioPassed_
                    ? ImVec4(0.35f, 0.85f, 0.45f, 1.0f)
                    : ImVec4(1.0f, 0.35f, 0.25f, 1.0f),
                "%s after %.1f s | %u transitions | loads +%llu | "
                "unloads +%llu | demotions +%llu",
                resultText,
                heroPressureScenarioElapsed_,
                heroPressureScenarioTransitions_,
                static_cast<unsigned long long>(
                    heroPressureScenarioLoads_),
                static_cast<unsigned long long>(
                    heroPressureScenarioUnloads_),
                static_cast<unsigned long long>(
                    heroPressureScenarioDemotions_));
            ImGui::Text(
                "Dominant fallback frames: %u | focal fallback frames: %u",
                heroPressureDominantFallbackFrames_,
                heroPressureFocalFallbackFrames_);
            ImGui::Text(
                "Rapid reloads: %u total | worst resource %u / %u",
                heroPressureRapidReloads_,
                heroPressureMaxResourceRapidReloads_,
                kHeroOrbitMaxRapidReloads);
            ImGui::Text("Worst per-resource state transitions: %u",
                        heroPressureMaxResourceTransitions_);
            ImGui::Text(
                "Handoff: invalidations %llu | error regressions %u | "
                "atomic groups %llu",
                static_cast<unsigned long long>(
                    heroPressureScenarioCurrentCutInvalidations_),
                heroPressureGeometricErrorRegressions_,
                static_cast<unsigned long long>(
                    heroPressureScenarioAtomicGroupCompletions_));
            ImGui::Text(
                "Rolling fallback: observed %u | old ladders released %u | "
                "violations %u",
                heroPressureRollingPinObservations_,
                heroPressureRollingReleaseObservations_,
                heroPressureRollingPinViolations_);
            ImGui::Text(
                "LOD error measurements %llu | positive %llu | regressions "
                "%llu",
                static_cast<unsigned long long>(
                    heroPressureScenarioErrorMeasurements_),
                static_cast<unsigned long long>(
                    heroPressureScenarioPositiveErrorMeasurements_),
                static_cast<unsigned long long>(
                    heroPressureScenarioErrorMeasurementRegressions_));
        }

        ImGui::Text("Memory");
        ImGui::Separator();
        const float residentMiB = virtualResidentMiB();
        const float loadingMiB = virtualLoadingMiB();
        const float committedMiB = residentMiB + loadingMiB;
        ImGui::Text("Resident %.2f MiB | loading %.2f MiB",
                    residentMiB, loadingMiB);
        ImGui::Text("Committed %.2f / %.2f MiB",
                    committedMiB, virtualMemoryBudgetMiB_);
        ImGui::ProgressBar(
            std::clamp(committedMiB /
                           std::max(virtualMemoryBudgetMiB_, 0.001f),
                       0.0f, 1.0f),
            ImVec2(-1.0f, 8.0f), "");
        ImGui::Text("Current cut %.2f MiB | ideal %.2f MiB",
                    currentFrontierMemoryMiB_, idealFrontierMemoryMiB_);
        ImGui::Text("Rolling fallback pins %.2f MiB",
                    protectedFallbackMemoryMiB_);
        ImGui::Text("Outside active target: %u resources | %.2f MiB",
                    lastReclaimableResourceCount_,
                    lastReclaimableResidentMiB_);

        ImGui::Text("Last published LOD error improvement");
        ImGui::Separator();
        if (!lastStreamingErrorImprovement_.valid)
        {
            ImGui::TextDisabled("No LOD publication measured yet");
        }
        else
        {
            const StreamingErrorImprovement& improvement =
                lastStreamingErrorImprovement_;
            ImGui::TextWrapped("Group #%llu: %s",
                               static_cast<unsigned long long>(
                                   improvement.groupSerial),
                               improvement.resourceNames.c_str());
            if (improvement.affectedInstances == 0)
            {
                ImGui::TextDisabled(
                    "Current cut unchanged at this camera; this publication "
                    "serves prefetch or future demand.");
            }
            else
            {
                ImGui::Text(
                    "Affected objects %u | local RMS %.3f -> %.3f px | "
                    "gain %.3f px (%.1f%%)",
                    improvement.affectedInstances,
                    improvement.localBeforeRmsPixels,
                    improvement.localAfterRmsPixels,
                    improvement.localBeforeRmsPixels -
                        improvement.localAfterRmsPixels,
                    errorImprovementPercent(
                        improvement.localBeforeRmsPixels,
                        improvement.localAfterRmsPixels));
                ImGui::Text(
                    "Overall RMS %.3f -> %.3f px | gain %.4f px "
                    "(%.3f%%)",
                    improvement.sceneBeforeRmsPixels,
                    improvement.sceneAfterRmsPixels,
                    improvement.sceneBeforeRmsPixels -
                        improvement.sceneAfterRmsPixels,
                    errorImprovementPercent(
                        improvement.sceneBeforeRmsPixels,
                        improvement.sceneAfterRmsPixels));
                ImGui::Text(
                    "Scene worst %.3f -> %.3f px | local worst %.3f -> "
                    "%.3f px",
                    improvement.sceneBeforeWorstPixels,
                    improvement.sceneAfterWorstPixels,
                    improvement.localBeforeWorstPixels,
                    improvement.localAfterWorstPixels);
            }
        }
        ImGui::TextDisabled(
            "Same-camera counterfactual; RMS uses each visible instance's "
            "worst selected-entry projected geometric error.");
        ImGui::Text(
            "Measured %llu publications | cut changed %llu | positive gain "
            "%llu | prefetch/unchanged %llu",
            static_cast<unsigned long long>(streamingErrorMeasurements_),
            static_cast<unsigned long long>(
                streamingVisibleErrorMeasurements_),
            static_cast<unsigned long long>(
                streamingPositiveErrorMeasurements_),
            static_cast<unsigned long long>(
                streamingNoCutChangePublications_));

        ImGui::Text("Convergence");
        ImGui::Separator();
        const float convergence = lastIdealEntryCount_ != 0
                                      ? float(lastConvergedEntryCount_) /
                                            float(lastIdealEntryCount_)
                                      : 1.0f;
        ImGui::Text("Current %u entries | ideal %u | matched %u",
                    lastCurrentSize_, lastIdealEntryCount_,
                    lastConvergedEntryCount_);
        ImGui::ProgressBar(std::clamp(convergence, 0.0f, 1.0f),
                           ImVec2(-1.0f, 14.0f));
        if (streamingConvergenceActive_)
            ImGui::Text("Converging for %.2f s",
                        streamingTime_ - streamingConvergenceStartTime_);
        else if (lastStreamingConvergenceSeconds_ > 0.0f)
            ImGui::Text("Last convergence completed in %.2f s",
                        lastStreamingConvergenceSeconds_);
        else
            ImGui::Text("Convergence time: not completed yet");
        if (streamingConvergenceHistoryCount_ != 0)
        {
            const int offset = streamingConvergenceHistoryCount_ ==
                                       kStreamingConvergenceHistorySize
                                   ? int(streamingConvergenceHistoryCursor_)
                                   : 0;
            ImGui::PushStyleColor(ImGuiCol_PlotLines,
                                  ImVec4(0.35f, 0.72f, 1.0f, 1.0f));
            ImGui::PlotLines(
                "##streaming-convergence",
                streamingConvergenceHistory_.data(),
                int(streamingConvergenceHistoryCount_), offset,
                "Convergence history (%)", 0.0f, 100.0f,
                ImVec2(-1.0f, 42.0f));
            ImGui::PopStyleColor();
        }
        ImGui::Text("Remaining refinement: %u groups | %u entries | %s",
                    lastRefinementGroups_, lastRefinementEntries_,
                    refinementPlanComplete_ ? "complete plan"
                                            : "bounded plan");
        if (heroPressureScenarioActive_)
            ImGui::Text("6 s camera look-ahead: %u entries | %u groups",
                        lastPrefetchEntryCount_, lastPrefetchGroupCount_);
        if (!virtualStreamingEnabled_)
        {
            ImGui::TextColored(
                ImVec4(0.35f, 0.72f, 1.0f, 1.0f),
                "Status: virtual streaming disabled; all active resources "
                "resident");
        }
        else if (!streamingPlanValid_)
        {
            ImGui::TextColored(ImVec4(0.65f, 0.70f, 0.78f, 1.0f),
                               "Status: waiting for first refinement plan");
        }
        else if (lastRefinementGroups_ == 0)
        {
            ImGui::TextColored(ImVec4(0.35f, 0.85f, 0.45f, 1.0f),
                               "Status: current frontier matches ideal");
        }
        else if (streamingPaused_)
        {
            ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.20f, 1.0f),
                               "Status: streaming paused");
        }
        else if (idealFrontierMemoryMiB_ > virtualMemoryBudgetMiB_ + 0.001f)
        {
            ImGui::TextColored(
                ImVec4(1.0f, 0.35f, 0.25f, 1.0f),
                "Status: ideal exceeds budget by %.2f MiB",
                idealFrontierMemoryMiB_ - virtualMemoryBudgetMiB_);
        }
        else if (lastBudgetBlockedGroups_ != 0 &&
                 pendingStreamingGroups_.empty())
        {
            ImGui::TextColored(
                ImVec4(1.0f, 0.55f, 0.20f, 1.0f),
                "Status: transition stalled; needs %.2f MiB committed",
                minimumBlockedCommitMiB_);
        }
        else
        {
            ImGui::TextColored(ImVec4(0.35f, 0.72f, 1.0f, 1.0f),
                               "Status: converging toward ideal");
        }
        ImGui::Text("Active loads %u / %d | budget-blocked groups %u",
                    uint32_t(pendingStreamingGroups_.size()),
                    maxConcurrentStreamingLoads_, lastBudgetBlockedGroups_);
        ImGui::Text("Admission rejects: capacity %u | insufficient gain %u",
                    lastCapacityBlockedGroups_, lastValueBlockedGroups_);
        ImGui::Text("Completed %llu resources | evicted %llu",
                    static_cast<unsigned long long>(streamingLoadsCompleted_),
                    static_cast<unsigned long long>(streamingUnloads_));
        ImGui::Text("Quality demotions to ready ancestors: %llu",
                    static_cast<unsigned long long>(
                        streamingQualityDemotions_));

        if (ImGui::TreeNode("Representation residency"))
        {
            ImGui::TextDisabled(
                "%u hero skyscraper assets | <= %u instances each | "
                "10.0 MiB max-detail leaves per hero",
                uint32_t(kHeroTowerAssetCount),
                uint32_t(kHeroTowerInstancesPerAsset));
            ImGui::TextWrapped(
                "Score estimates visual benefit: current-cut demand is 4x, "
                "the immediate refinement is 3x, predicted camera demand is "
                "1.5x, and ideal-endpoint demand is 1x. Each role contributes "
                "instances x (0.65 avg + 0.35 "
                "max screen error) / LOD threshold. Complete groups load by "
                "score/MiB; the group membership reported by refinement is "
                "retained for atomic pressure eviction. The lowest-value "
                "complete group in the safest demand class goes first.");
            ImGui::TextWrapped(
                "A resident representation keeps the parent-error benefit "
                "recorded when it was admitted. Rescoring it with its own "
                "smaller child error would create load/evict oscillation.");
            ImGui::TextWrapped(
                "Replacement is transactional: the simulator first plans a "
                "complete victim set without changing residency. It commits "
                "only when the full request fits and its total visual value "
                "exceeds unused cache victims by at least 5%%. Current-cut "
                "resources are never load victims: resident current geometry "
                "and its loading replacement are both charged to the hard "
                "budget, so an upgrade without overlap capacity stays "
                "blocked.");
            ImGui::TextWrapped(
                "Loads use make-before-break publication. Every member of a "
                "complete group remains unavailable while loading, then all "
                "members become ready together. The next Frontier query may "
                "select the finer cut only after that atomic publication; its "
                "previous current representation remains ready throughout.");
            ImGui::TextWrapped(
                "Fallback residency is rolling rather than cumulative. A "
                "selected representation pins only its immediate ready "
                "predecessor. The current cut is protected separately during "
                "a replacement load; after the handoff, the pin advances and "
                "older LODs become reclaimable after the unload delay.");
            ImGui::TextWrapped(
                "Minimum visible quality is enforced before scalar scoring: "
                "a screen-dominant hero's district representation is loaded "
                "first and cannot be selected as a victim. This quality-floor "
                "request may displace lower-priority cache, refinement, or "
                "fallback-pinned resources; remaining bytes are still assigned "
                "by score/MiB.");
            ImGui::TextWrapped(
                "Refinement rows use the projected error of the parent that "
                "the representation replaces; terminal child error is often "
                "zero and does not measure the benefit of loading it.");
            ImGui::TextDisabled(
                "Data columns: instance count | min / avg / max screen "
                "error or eliminated parent error (px)");

            std::array<size_t, kStreamingResourceSlotCount> resourceOrder{};
            for (size_t index = 0; index < resourceOrder.size(); ++index)
                resourceOrder[index] = index;
            std::stable_sort(
                resourceOrder.begin(), resourceOrder.end(),
                [this](size_t lhs, size_t rhs)
                {
                    const VirtualResource& left = virtualResources_[lhs];
                    const VirtualResource& right = virtualResources_[rhs];
                    if (left.scorePerMiB != right.scorePerMiB)
                        return left.scorePerMiB > right.scorePerMiB;
                    if (left.importanceScore != right.importanceScore)
                        return left.importanceScore > right.importanceScore;
                    return lhs < rhs;
                });

            constexpr ImGuiTableFlags tableFlags =
                ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable |
                ImGuiTableFlags_Hideable | ImGuiTableFlags_ScrollX |
                ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit;
            if (ImGui::BeginTable("##representation-residency", 12,
                                  tableFlags, ImVec2(-1.0f, 270.0f),
                                  2040.0f))
            {
                ImGui::TableSetupScrollFreeze(2, 1);
                ImGui::TableSetupColumn("Rank", 0, 42.0f);
                ImGui::TableSetupColumn("Representation", 0, 205.0f);
                ImGui::TableSetupColumn("State", 0, 90.0f);
                ImGui::TableSetupColumn("Current node", 0, 190.0f);
                ImGui::TableSetupColumn("Current benefit (4x)", 0, 190.0f);
                ImGui::TableSetupColumn("Next (3x)", 0, 190.0f);
                ImGui::TableSetupColumn("Look-ahead (1.5x)", 0, 190.0f);
                ImGui::TableSetupColumn("Ideal (1x)", 0, 190.0f);
                ImGui::TableSetupColumn("Bytes", 0, 105.0f);
                ImGui::TableSetupColumn("Score", 0, 82.0f);
                ImGui::TableSetupColumn("Score/MiB", 0, 92.0f);
                ImGui::TableSetupColumn("Policy / last action", 0, 350.0f);
                ImGui::TableHeadersRow();

                uint32_t rank = 0;
                for (size_t slot : resourceOrder)
                {
                    const VirtualResource& resource = virtualResources_[slot];
                    if (resource.byteSizeMiB <= 0.0f)
                        continue;
                    ++rank;
                    char name[96];
                    virtualResourceName(resource, name, sizeof(name));
                    const ImVec4 color =
                        resource.pinned
                            ? ImVec4(0.50f, 0.85f, 1.0f, 1.0f)
                        : resource.state == StreamingResourceState::Resident
                            ? ImVec4(0.35f, 0.85f, 0.45f, 1.0f)
                        : resource.state == StreamingResourceState::Loading
                            ? ImVec4(1.0f, 0.78f, 0.20f, 1.0f)
                            : ImVec4(0.58f, 0.58f, 0.62f, 1.0f);
                    const uint64_t bytes = uint64_t(
                        resource.byteSizeMiB * 1024.0f * 1024.0f + 0.5f);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%u", rank);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextColored(color, "%s", name);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextColored(
                        color, "%s%s", streamingStateName(resource.state),
                        resource.pinned ? " (pinned)" : "");
                    const auto drawRole = [](const StreamingErrorStats& errors)
                    {
                        if (errors.count == 0)
                        {
                            ImGui::TextDisabled("0 | -- / -- / --");
                            return;
                        }
                        ImGui::Text("%u | %.2f / %.2f / %.2f", errors.count,
                                    errors.minimum, errors.average(),
                                    errors.maximum);
                    };
                    ImGui::TableSetColumnIndex(3);
                    drawRole(resource.currentErrors);
                    ImGui::TableSetColumnIndex(4);
                    drawRole(resource.currentBenefitErrors);
                    ImGui::TableSetColumnIndex(5);
                    drawRole(resource.transitionErrors);
                    ImGui::TableSetColumnIndex(6);
                    drawRole(resource.prefetchErrors);
                    ImGui::TableSetColumnIndex(7);
                    drawRole(resource.idealErrors);
                    ImGui::TableSetColumnIndex(8);
                    ImGui::Text("%llu B",
                                static_cast<unsigned long long>(bytes));
                    ImGui::TableSetColumnIndex(9);
                    if (resource.pinned)
                        ImGui::Text("INF");
                    else
                        ImGui::Text("%.1f", resource.importanceScore);
                    ImGui::TableSetColumnIndex(10);
                    if (resource.pinned)
                        ImGui::Text("INF");
                    else
                        ImGui::Text("%.1f", resource.scorePerMiB);
                    ImGui::TableSetColumnIndex(11);
                    ImGui::TextWrapped("%s | last: %s",
                                       resource.decision.c_str(),
                                       resource.lastAction.c_str());
                }
                ImGui::EndTable();
            }
            ImGui::TreePop();
        }

        ImGui::Text("Virtual load / unload log");
        ImGui::Separator();
        if (streamingLog_.empty())
        {
            ImGui::TextDisabled("No streaming events yet");
        }
        else
        {
            const size_t first = streamingLog_.size() > 18
                                     ? streamingLog_.size() - 18
                                     : 0;
            for (size_t index = streamingLog_.size(); index-- > first;)
            {
                const StreamingLogEntry& entry = streamingLog_[index];
                ImGui::TextColored(entry.color, "[%6.2f] %s", entry.time,
                                   entry.message.c_str());
            }
        }
        ImGui::End();
    }

    void drawSceneStatsUi()
    {
        ImGui::SetNextWindowPos(ImVec2(414.0f, 36.0f + city::windowFrameHeight()),
                                ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(350.0f, 470.0f),
                                 ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Scene stats", &showSceneStats_))
        {
            ImGui::End();
            return;
        }
        ImGui::TextColored(ImVec4(0.34f, 0.82f, 1.0f, 1.0f),
                           "Frontier Dynamic City / bgfx");
        ImGui::Separator();
        ImGui::Text("%u houses | %u towers | %u trees",
                    houseCount_, towerCount_, treeCount_);
        ImGui::Text("House style %s | generation %u",
                    activeHouseStyle_ == HouseStyle::HouseA ? "A" : "B",
                    houseGeneration_);
        ImGui::Text("%u cars | %u pedestrians",
                    unsigned(carHandles_.size()),
                    unsigned(pedestrianHandles_.size()));
        ImGui::Text("Authored HLOD nodes: %llu",
                    static_cast<unsigned long long>(authoredHlodNodeCount_));
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Full authored hierarchy per scene instance, including the "
                "aggregate parent.\nExcludes generated spatial nodes; multiple "
                "LOD payloads on one node count once.");
        ImGui::Text("Current cut %u", lastCurrentSize_);
        ImGui::Text("Updated HLOD nodes/frame: %llu",
                    static_cast<unsigned long long>(updatedHlodNodes_));
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Authored nodes affected by changed instance transforms in the "
                "last completed frame.\nEach hierarchy counts once, even if moved "
                "twice. Includes culled and nonresident nodes.\nRigid motion "
                "updates the instance transform; it does not rewrite every "
                "child node. Spatial maintenance is counted separately.");
        ImGui::Text("Moved instances/frame: %u", movedInstances_);
        ImGui::Text("Refinement %u groups | %u entries",
                    lastRefinementGroups_, lastRefinementEntries_);
        ImGui::Text("Virtual residency %.2f / %.2f MiB | %u loading",
                    virtualResidentMiB(), virtualMemoryBudgetMiB_,
                    uint32_t(pendingStreamingGroups_.size()));
        ImGui::Text("Frontier convergence %u / %u ideal entries",
                    lastConvergedEntryCount_, lastIdealEntryCount_);
        ImGui::Text("Query cache %u reused | %u walked",
                    lastQueryReused_, lastQueryWalked_);
        ImGui::Text("%.0f fps | simulation %.1f s", smoothedFps_,
                    simulationTime_);
        ImGui::Text("Whole-scene stress: %s",
                    animateWholeScene_ ? "ACTIVE" : "off");
        if (unlimitedTlasMaintenance_)
            ImGui::Text("applyUpdates budget unlimited | %u pending",
                        lastUpdateReport_.maintenanceNodesPending);
        else
            ImGui::Text("applyUpdates budget %d | %u pending",
                        tlasMaintenanceBudget_,
                        lastUpdateReport_.maintenanceNodesPending);
        if (rebuildStrategy_ == RebuildStrategy::Manual)
            ImGui::Text("TLAS rebuild manual only | %llu calls",
                        static_cast<unsigned long long>(rebuildCount_));
        else if (rebuildStrategy_ == RebuildStrategy::Periodic)
            ImGui::Text("TLAS %s periodic %.1f s | %llu calls",
                        optimizationModeName(scheduledOptimizationMode_),
                        rebuildIntervalSeconds_,
                        static_cast<unsigned long long>(rebuildCount_));
        else
            ImGui::Text("TLAS %s advised %.1f s | %llu calls",
                        optimizationModeName(scheduledOptimizationMode_),
                        rebuildIntervalSeconds_,
                        static_cast<unsigned long long>(rebuildCount_));
        ImGui::TextColored(
            freezeSimulation_ ? ImVec4(1.0f, 0.55f, 0.2f, 1.0f)
                              : ImVec4(0.45f, 0.95f, 0.55f, 1.0f),
            "Simulation %s", freezeSimulation_ ? "FROZEN" : "running");
        ImGui::Text("Display camera: %s",
                    freeCamera_ ? "free" : "automatic");
        ImGui::TextColored(
            freezeCullCamera_ ? ImVec4(1.0f, 0.35f, 0.8f, 1.0f)
                              : ImVec4(0.75f, 0.75f, 0.75f, 1.0f),
            "Culling camera: %s",
            freezeCullCamera_ ? "FROZEN" : "display camera");
        ImGui::End();
    }

    static float measuredPerformanceMs(const PerformanceSample& sample)
    {
        return sample.uiMs + sample.simulationMs + sample.cameraMs +
               sample.applyUpdatesMs + sample.tlasRebuildMs +
               sample.selectionMs + sample.cutStatsMs + sample.renderMs +
               sample.streamingMs + sample.frameSubmitMs;
    }

    static float performanceTimerMs(const PerformanceSample& sample,
                                    PerformanceTimer timer)
    {
        switch (timer)
        {
        case PerformanceTimer::Total: return sample.totalMs;
        case PerformanceTimer::Ui: return sample.uiMs;
        case PerformanceTimer::Simulation: return sample.simulationMs;
        case PerformanceTimer::ApplyUpdates: return sample.applyUpdatesMs;
        case PerformanceTimer::TlasRebuild: return sample.tlasRebuildMs;
        case PerformanceTimer::Camera: return sample.cameraMs;
        case PerformanceTimer::Selection: return sample.selectionMs;
        case PerformanceTimer::CutStats: return sample.cutStatsMs;
        case PerformanceTimer::Render: return sample.renderMs;
        case PerformanceTimer::Streaming: return sample.streamingMs;
        case PerformanceTimer::StreamingRefinement:
            return sample.streamingRefinementMs;
        case PerformanceTimer::StreamingErrorMeasurement:
            return sample.streamingErrorMeasurementMs;
        case PerformanceTimer::StreamingPlanner:
            return sample.streamingPlannerMs;
        case PerformanceTimer::StreamingPlannerIndex:
            return sample.streamingPlannerIndexMs;
        case PerformanceTimer::StreamingPlannerDemand:
            return sample.streamingPlannerDemandMs;
        case PerformanceTimer::StreamingPlannerScore:
            return sample.streamingPlannerScoreMs;
        case PerformanceTimer::StreamingPlannerResidency:
            return sample.streamingPlannerResidencyMs;
        case PerformanceTimer::StreamingPlannerOther:
            return sample.streamingPlannerOtherMs;
        case PerformanceTimer::StreamingScenario:
            return sample.streamingScenarioMs;
        case PerformanceTimer::FrameSubmit: return sample.frameSubmitMs;
        case PerformanceTimer::RenderThread: return sample.renderThreadMs;
        case PerformanceTimer::Gpu: return sample.gpuMs;
        case PerformanceTimer::WaitRender: return sample.waitRenderMs;
        case PerformanceTimer::WaitSubmit: return sample.waitSubmitMs;
        case PerformanceTimer::Other:
            return std::max(0.0f,
                            sample.totalMs - measuredPerformanceMs(sample));
        case PerformanceTimer::Count: break;
        }
        return 0.0f;
    }

    void drawPerformanceTimer(const char* label, PerformanceTimer timer,
                              const ImVec4& color)
    {
        const float valueMs = performanceTimerMs(performance_, timer);
        ImGui::Text("%-18s %8.1f us", label, valueMs * 1000.0f);
        const auto& history =
            performanceHistory_[static_cast<size_t>(timer)];
        float historyMaximumUs = 1.0f;
        if (performanceHistoryCount_ != 0)
        {
            float minimumUs = history[0];
            float maximumUs = history[0];
            double totalUs = 0.0;
            for (size_t index = 0; index < performanceHistoryCount_; ++index)
            {
                minimumUs = std::min(minimumUs, history[index]);
                maximumUs = std::max(maximumUs, history[index]);
                totalUs += history[index];
            }
            historyMaximumUs = maximumUs;
            const float averageUs =
                float(totalUs / double(performanceHistoryCount_));
            ImGui::TextDisabled("min %.1f | max %.1f | avg %.1f us",
                                minimumUs, maximumUs, averageUs);
        }
        else
        {
            ImGui::TextDisabled("min -- | max -- | avg -- us");
        }
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, color);
        ImGui::ProgressBar(
            std::clamp(valueMs /
                           std::max(performance_.totalMs, 0.001f),
                       0.0f, 1.0f),
            ImVec2(-1.0f, 5.0f), "");
        ImGui::PopStyleColor();

        if (performanceHistoryCount_ == 0)
            return;
        const float scaleMax = historyMaximumUs * 1.10f;
        const int offset = performanceHistoryCount_ ==
                                   kPerformanceHistorySize
                               ? int(performanceHistoryCursor_)
                               : 0;
        ImGui::PushID(label);
        ImGui::PushStyleColor(ImGuiCol_PlotLines, color);
        ImGui::PlotLines("##timer-history", history.data(),
                         int(performanceHistoryCount_), offset, nullptr,
                         0.0f, scaleMax, ImVec2(-1.0f, 30.0f));
        ImGui::PopStyleColor();
        ImGui::PopID();
    }

    void drawPerformanceUi()
    {
        ImGui::SetNextWindowPos(ImVec2(414.0f, 258.0f + city::windowFrameHeight()),
                                ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(350.0f, 430.0f),
                                 ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Performance", &showPerformance_))
        {
            ImGui::End();
            return;
        }
        ImGui::Text("Smoothed values | rolling 5-10 second charts");
        ImGui::Text("Renderer: %s", bgfx::getRendererName(bgfx::getRendererType()));
        ImGui::Text("Requested MSAA: %s",
                    (reset_ & BGFX_RESET_MSAA_MASK) != 0 ? "4x" : "off");

        ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.20f, 1.0f), "Frontier");
        ImGui::Separator();
        drawPerformanceTimer("Frontier select", PerformanceTimer::Selection,
                             ImVec4(1.0f, 0.78f, 0.20f, 1.0f));
        drawPerformanceTimer("Motion submit", PerformanceTimer::Simulation,
                             ImVec4(0.35f, 0.90f, 0.50f, 1.0f));
        drawPerformanceTimer("applyUpdates",
                             PerformanceTimer::ApplyUpdates,
                             ImVec4(0.28f, 0.82f, 0.58f, 1.0f));
        drawPerformanceTimer("TLAS rebuild", PerformanceTimer::TlasRebuild,
                             ImVec4(0.98f, 0.50f, 0.18f, 1.0f));
        drawPerformanceTimer("Virtual streaming", PerformanceTimer::Streaming,
                             ImVec4(0.90f, 0.60f, 0.25f, 1.0f));
        ImGui::Indent();
        drawPerformanceTimer("Frontier refine",
                             PerformanceTimer::StreamingRefinement,
                             ImVec4(0.98f, 0.70f, 0.28f, 1.0f));
        drawPerformanceTimer("Error measurement",
                             PerformanceTimer::StreamingErrorMeasurement,
                             ImVec4(0.46f, 0.80f, 0.92f, 1.0f));
        drawPerformanceTimer("Streaming planner",
                             PerformanceTimer::StreamingPlanner,
                             ImVec4(0.82f, 0.52f, 0.22f, 1.0f));
        ImGui::Indent();
        drawPerformanceTimer("Index build",
                             PerformanceTimer::StreamingPlannerIndex,
                             ImVec4(0.90f, 0.58f, 0.24f, 1.0f));
        drawPerformanceTimer("Demand + groups",
                             PerformanceTimer::StreamingPlannerDemand,
                             ImVec4(0.86f, 0.50f, 0.22f, 1.0f));
        drawPerformanceTimer("Score + rank",
                             PerformanceTimer::StreamingPlannerScore,
                             ImVec4(0.80f, 0.44f, 0.20f, 1.0f));
        drawPerformanceTimer("Residency policy",
                             PerformanceTimer::StreamingPlannerResidency,
                             ImVec4(0.74f, 0.38f, 0.18f, 1.0f));
        drawPerformanceTimer("Planner residual",
                             PerformanceTimer::StreamingPlannerOther,
                             ImVec4(0.62f, 0.34f, 0.18f, 1.0f));
        ImGui::Unindent();
        drawPerformanceTimer("Scenario checks",
                             PerformanceTimer::StreamingScenario,
                             ImVec4(0.70f, 0.42f, 0.18f, 1.0f));
        ImGui::Unindent();

        ImGui::TextColored(ImVec4(0.90f, 0.45f, 0.85f, 1.0f), "bgfx");
        ImGui::Separator();
        drawPerformanceTimer("Debug draw", PerformanceTimer::Render,
                             ImVec4(0.90f, 0.45f, 0.85f, 1.0f));
        drawPerformanceTimer("bgfx::frame", PerformanceTimer::FrameSubmit,
                             ImVec4(0.95f, 0.35f, 0.35f, 1.0f));
        drawPerformanceTimer("Render thread", PerformanceTimer::RenderThread,
                             ImVec4(0.82f, 0.42f, 0.88f, 1.0f));
        drawPerformanceTimer("GPU", PerformanceTimer::Gpu,
                             ImVec4(0.72f, 0.36f, 0.82f, 1.0f));
        drawPerformanceTimer("Wait render", PerformanceTimer::WaitRender,
                             ImVec4(0.95f, 0.52f, 0.58f, 1.0f));
        drawPerformanceTimer("Wait submit", PerformanceTimer::WaitSubmit,
                             ImVec4(0.95f, 0.62f, 0.48f, 1.0f));
        ImGui::Text("%u draws | %u triangles", performance_.drawCalls,
                    performance_.triangles);
        ImGui::Text("Transient VB %.1f KiB | IB %.1f KiB",
                    float(std::max(performance_.transientVertexBytes, 0)) /
                        1024.0f,
                    float(std::max(performance_.transientIndexBytes, 0)) /
                        1024.0f);

        ImGui::TextColored(ImVec4(0.35f, 0.70f, 1.0f, 1.0f), "Other");
        ImGui::Separator();
        drawPerformanceTimer("ImGui", PerformanceTimer::Ui,
                             ImVec4(0.35f, 0.70f, 1.0f, 1.0f));
        drawPerformanceTimer("Camera + views", PerformanceTimer::Camera,
                             ImVec4(0.55f, 0.80f, 0.95f, 1.0f));
        drawPerformanceTimer("Cut accounting", PerformanceTimer::CutStats,
                             ImVec4(0.80f, 0.68f, 0.28f, 1.0f));
        drawPerformanceTimer("Unaccounted", PerformanceTimer::Other,
                             ImVec4(0.55f, 0.55f, 0.60f, 1.0f));

        ImGui::Text("Frame summary");
        ImGui::Separator();
        ImGui::Text("Sample CPU %.1f us (%.0f fps) | GPU %.1f us",
                    performance_.totalMs * 1000.0f,
                    performance_.totalMs > 0.0f
                        ? 1000.0f / performance_.totalMs
                        : 0.0f,
                    performance_.gpuMs * 1000.0f);
        drawPerformanceTimer("Total CPU frame", PerformanceTimer::Total,
                             ImVec4(0.80f, 0.82f, 0.88f, 1.0f));
        ImGui::End();
    }

#ifdef FRONTIER_DEBUG_TOOLS
    void sampleTlasHealth()
    {
        if (tlasHealthValid_ && cameraTime_ < nextTlasHealthSampleTime_)
            return;
        tlasHealth_ = database_.debugTlasSummary();
        tlasHealthValid_ = true;
        nextTlasHealthSampleTime_ =
            tlasHealth_.buildRequired ? cameraTime_
                                      : cameraTime_ + 0.25f;
        tlasDebugDepth_ = std::clamp(
            tlasDebugDepth_, 0, int(tlasHealth_.maxDepth));
    }

    void drawTlasHealthUi()
    {
        sampleTlasHealth();
        ImGui::SetNextWindowPos(ImVec2(12.0f, 478.0f + city::windowFrameHeight()),
                                ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(390.0f, 410.0f),
                                 ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("TLAS health", &showTlasHealth_))
        {
            ImGui::End();
            return;
        }

        ImGui::Text("Quality active %s | configured %s | width %u",
                    tlasQualityName(tlasHealth_.activeQuality),
                    tlasQualityName(tlasHealth_.configuredQuality), kWide);
        ImGui::Text("Nodes %u active | %u allocated | %u free",
                    tlasHealth_.activeNodes, tlasHealth_.allocatedNodes,
                    tlasHealth_.freeNodes);
        ImGui::Text("Instances %u | loose %u", tlasHealth_.instanceCount,
                    tlasHealth_.looseInstanceCount);
        ImGui::Text("Internal lanes %u | instance lanes %u",
                    tlasHealth_.internalLaneCount,
                    tlasHealth_.instanceLaneCount);
        ImGui::Text("Max depth %u | edits since rebuild %u",
                    tlasHealth_.maxDepth,
                    tlasHealth_.editsSinceRebuild);
        ImGui::Text("Rebuild baseline %u instances",
                    tlasHealth_.rebuildBaselineInstances);
        ImGui::Text("TLAS storage %.1f KiB",
                    float(tlasHealth_.bytes) / 1024.0f);

        ImGui::Text("Lane occupancy %.1f%%",
                    tlasHealth_.averageLaneOccupancy * 100.0f);
        ImGui::ProgressBar(tlasHealth_.averageLaneOccupancy,
                           ImVec2(-1.0f, 6.0f), "");
        const float areaLimit = database_.config().tlasAreaDrift;
        ImGui::Text("Motion area growth %.1f%% / %.1f%% limit",
                    tlasHealth_.areaGrowthRatio * 100.0f,
                    areaLimit * 100.0f);
        ImGui::ProgressBar(
            std::clamp(tlasHealth_.areaGrowthRatio /
                           std::max(areaLimit, 1.0e-6f),
                       0.0f, 1.0f),
            ImVec2(-1.0f, 6.0f), "");
        ImGui::Text("Maintenance queue %u nodes",
                    tlasHealth_.maintenanceNodesPending);
        ImGui::Text("Last maintenance %u processed | %u pending",
                    lastUpdateReport_.maintenanceNodesProcessed,
                    lastUpdateReport_.maintenanceNodesPending);
        if (unlimitedTlasMaintenance_)
            ImGui::Text("applyUpdates budget: unlimited");
        else
            ImGui::Text("applyUpdates budget: %d nodes",
                        tlasMaintenanceBudget_);
        ImGui::Text("Rebuild strategy: %s | mode: %s",
                    rebuildStrategyName(rebuildStrategy_),
                    optimizationModeName(scheduledOptimizationMode_));
        if (tlasHealth_.buildRequired)
            ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.20f, 1.0f),
                               "TLAS health: correctness build required");
        else if (tlasHealth_.topologyRebuildRecommended)
            ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.20f, 1.0f),
                               "TLAS health: topology rebuild recommended");
        else
            ImGui::TextColored(ImVec4(0.35f, 0.85f, 0.45f, 1.0f),
                               "TLAS health: OK; rebuild not needed");

        ImGui::Separator();
        ImGui::Text("Spatial visualizations");
        ImGui::Checkbox("TLAS AABBs by depth", &drawTlasAabbs_);
        if (tlasHealth_.maxDepth != 0)
            ImGui::SliderInt("AABB depth", &tlasDebugDepth_, 0,
                             int(tlasHealth_.maxDepth));
        ImGui::SliderInt("AABB draw limit", &tlasDebugBoxLimit_, 64, 65536);
        if (drawTlasAabbs_)
        {
            ImGui::Text("Drawing %zu / %zu boxes",
                        lastTlasBoxesDrawn_, lastTlasBoxesTotal_);
            ImGui::TextDisabled(
                "Complete cut includes terminal leaves above this depth");
            if (lastTlasBoxesDrawn_ < lastTlasBoxesTotal_)
                ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.20f, 1.0f),
                                   "Increase the draw limit to see all boxes");
        }

        ImGui::Checkbox("Loose envelopes vs exact bounds",
                        &drawLooseBounds_);
        ImGui::SliderInt("Loose draw limit", &looseBoundsDrawLimit_,
                         16, 2048);
        if (drawLooseBounds_)
        {
            ImGui::Text("Drawing %zu / %zu loose instances",
                        lastLooseBoundsDrawn_, lastLooseBoundsTotal_);
            if (lastLooseBoundsTotal_ == 0)
                ImGui::TextDisabled(
                    "Bulk motion uses exact refits, so loose bounds may be empty");
        }
        ImGui::Checkbox("X-ray debug bounds", &debugBoundsXray_);
        ImGui::TextColored(ImVec4(0.20f, 0.85f, 1.0f, 1.0f),
                           "Cyan: TLAS internal");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.80f, 0.18f, 1.0f),
                           "Gold: instance");
        ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.10f, 1.0f),
                           "Orange: loose envelope");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.20f, 1.0f, 0.35f, 1.0f),
                           "Green: exact");
        ImGui::End();
    }

    void drawQueryCacheUi()
    {
        const QueryCacheDebugSummary cache = query_.debugCacheSummary();
        const uint32_t total = cache.reused + cache.walked;
        const float hitRate = total != 0
                                  ? float(cache.reused) / float(total)
                                  : 0.0f;
        ImGui::SetNextWindowPos(ImVec2(776.0f, 700.0f + city::windowFrameHeight()),
                                ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(390.0f, 330.0f),
                                 ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Query cache", &showQueryCache_))
        {
            ImGui::End();
            return;
        }

        ImGui::Text("Last selection %u reused | %u walked",
                    cache.reused, cache.walked);
        ImGui::Text("Hit rate %.1f%%", hitRate * 100.0f);
        ImGui::ProgressBar(hitRate, ImVec2(-1.0f, 7.0f), "");
        ImGui::Text("State %s | epoch %u",
                    cache.reuseEnabled ? "enabled" : "disabled",
                    cache.epoch);
        ImGui::Text("Primed %s | whole-cut reusable %s",
                    cache.primed ? "yes" : "no",
                    cache.wholeReusable ? "yes" : "no");
        ImGui::Text("Record slots %u | live entries %u",
                    cache.recordSlots, cache.liveEntries);
        ImGui::Text("Garbage entries %u | slab entries %u",
                    cache.garbageEntries, cache.slabEntries);
        ImGui::Text("Cache memory %.1f KiB", float(cache.bytes) / 1024.0f);
        ImGui::Text("Travel %.2f | projection travel %.2f",
                    cache.positionTravel, cache.projectionTravel);
        ImGui::Text("Mount usage tracking %s",
                    cache.mountUsageEnabled ? "enabled" : "disabled");

        if (queryCacheHistoryCount_ != 0)
        {
            const int offset = queryCacheHistoryCount_ ==
                                       kPerformanceHistorySize
                                   ? int(queryCacheHistoryCursor_)
                                   : 0;
            ImGui::PlotLines(
                "##query-cache-history", queryCacheHitHistory_.data(),
                int(queryCacheHistoryCount_), offset,
                "Cache hit history (%)", 0.0f, 100.0f,
                ImVec2(-1.0f, 64.0f));
        }
#ifdef FRONTIER_STATS
        const SelectionStats& stats = query_.lastSelectionStats();
        ImGui::Separator();
        ImGui::Text("Traversal instrumentation");
        ImGui::Text("Instances %llu | subtrees %llu | nodes %llu",
                    static_cast<unsigned long long>(stats.instancesVisited),
                    static_cast<unsigned long long>(stats.subtreesVisited),
                    static_cast<unsigned long long>(stats.nodesVisited));
        ImGui::Text("Wide blocks %llu | lanes survived %llu",
                    static_cast<unsigned long long>(stats.wideBlocksTested),
                    static_cast<unsigned long long>(stats.lanesSurvived));
#else
        ImGui::Separator();
        ImGui::TextDisabled("FRONTIER_STATS is disabled");
        ImGui::TextWrapped(
            "Detailed traversal counters are unavailable. Enabling "
            "FRONTIER_STATS instruments hot traversal paths and affects "
            "measured performance.");
#endif
        ImGui::End();
    }

    void recordQueryCacheHistory()
    {
        const uint32_t total = lastQueryReused_ + lastQueryWalked_;
        queryCacheHitHistory_[queryCacheHistoryCursor_] =
            total != 0 ? 100.0f * float(lastQueryReused_) / float(total)
                       : 0.0f;
        queryCacheHistoryCursor_ =
            (queryCacheHistoryCursor_ + 1) % kPerformanceHistorySize;
        queryCacheHistoryCount_ =
            std::min(queryCacheHistoryCount_ + 1,
                     kPerformanceHistorySize);
    }
#endif

    uint32_t payloadCount(
        const std::array<uint32_t, kPayloadSlotCount>& counts,
        Payload payload) const
    {
        return counts[static_cast<size_t>(payload)];
    }

    ImVec4 payloadUiColor(Payload payload) const
    {
        const uint32_t color = hierarchyTint(payload);
        return ImVec4(float(color & 0xff) / 255.0f,
                      float((color >> 8) & 0xff) / 255.0f,
                      float((color >> 16) & 0xff) / 255.0f, 1.0f);
    }

    bool sceneTreeSelectionEquals(uint32_t entityKindMask,
                                  uint16_t heroAsset,
                                  Payload firstPayload,
                                  Payload lastPayload,
                                  const char* label,
                                  InstanceId instance =
                                      kInvalidInstanceId) const
    {
        return sceneTreeSelection_.active() &&
               sceneTreeSelection_.entityKindMask == entityKindMask &&
               sceneTreeSelection_.heroAsset == heroAsset &&
               sceneTreeSelection_.instance == instance &&
               sceneTreeSelection_.firstPayload == firstPayload &&
               sceneTreeSelection_.lastPayload == lastPayload &&
               sceneTreeSelection_.label == label;
    }

    void toggleSceneTreeSelection(uint32_t entityKindMask,
                                  uint16_t heroAsset,
                                  Payload firstPayload,
                                  Payload lastPayload,
                                  const char* label,
                                  InstanceId instance = kInvalidInstanceId)
    {
        if (sceneTreeSelectionEquals(entityKindMask, heroAsset,
                                     firstPayload, lastPayload, label,
                                     instance))
        {
            sceneTreeSelection_ = {};
            return;
        }
        sceneTreeSelection_.entityKindMask = entityKindMask;
        sceneTreeSelection_.heroAsset = heroAsset;
        sceneTreeSelection_.instance = instance;
        sceneTreeSelection_.firstPayload = firstPayload;
        sceneTreeSelection_.lastPayload = lastPayload;
        sceneTreeSelection_.label = label;
    }

    bool beginSceneTreeNode(const char* id, const char* label,
                            uint32_t entityKindMask,
                            uint16_t heroAsset,
                            Payload firstPayload,
                            Payload lastPayload,
                            bool defaultOpen = false,
                            const char* selectionLabel = nullptr,
                            InstanceId instance = kInvalidInstanceId)
    {
        ImGuiTreeNodeFlags flags =
            ImGuiTreeNodeFlags_SpanAvailWidth |
            ImGuiTreeNodeFlags_OpenOnArrow;
        if (defaultOpen)
            flags |= ImGuiTreeNodeFlags_DefaultOpen;
        const char* stableSelectionLabel =
            selectionLabel ? selectionLabel : label;
        if (sceneTreeSelectionEquals(entityKindMask, heroAsset,
                                     firstPayload, lastPayload,
                                     stableSelectionLabel, instance))
            flags |= ImGuiTreeNodeFlags_Selected;
        const bool open = ImGui::TreeNodeEx(id, flags, "%s", label);
        if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
        {
            toggleSceneTreeSelection(entityKindMask, heroAsset,
                                     firstPayload, lastPayload,
                                     stableSelectionLabel, instance);
        }
        return open;
    }

    bool sceneTreeSelectionMatches(Payload payload, InstanceId instance,
                                   const Entity& entity) const
    {
        if (!sceneTreeSelection_.active() ||
            (sceneTreeSelection_.entityKindMask &
             entityKindBit(entity.kind)) == 0)
            return false;
        if (sceneTreeSelection_.heroAsset != UINT16_MAX &&
            entity.heroAsset != sceneTreeSelection_.heroAsset)
            return false;
        if (sceneTreeSelection_.instance != kInvalidInstanceId &&
            sceneTreeSelection_.instance != instance)
            return false;
        return payload >= sceneTreeSelection_.firstPayload &&
               payload <= sceneTreeSelection_.lastPayload;
    }

    size_t sceneTreeResourceSlot(
        Payload payload, uint16_t heroAsset = UINT16_MAX) const
    {
        if (isTowerDetailPayload(payload))
        {
            if (heroAsset >= kHeroTowerAssetCount)
                return kStreamingResourceSlotCount;
            return kHeroTowerResourceBase +
                   size_t(heroAsset) * kTowerDetailResourceCount +
                   size_t(payload) - size_t(Payload::TowerDistrict);
        }
        if (isHouseDetailPayload(payload) &&
            activeHouseStyle_ == HouseStyle::HouseB)
        {
            return kPayloadSlotCount +
                   size_t(payload) - size_t(Payload::HouseCoarse);
        }
        const size_t slot = size_t(payload);
        return slot < virtualResources_.size()
                   ? slot
                   : kStreamingResourceSlotCount;
    }

    uint32_t sceneTreeCurrentCount(
        Payload payload, uint16_t heroAsset = UINT16_MAX) const
    {
        if (heroAsset < kHeroTowerAssetCount &&
            payload >= Payload::TowerTop &&
            payload <= Payload::TowerCrown)
        {
            return currentHeroPayloadCounts_[heroAsset][
                size_t(payload) - size_t(Payload::TowerTop)];
        }
        return payloadCount(currentPayloadCounts_, payload);
    }

    void sceneTreeResidencyLabel(Payload payload, uint16_t heroAsset,
                                 char* output, size_t outputSize) const
    {
        const size_t slot = sceneTreeResourceSlot(payload, heroAsset);
        if (slot >= virtualResources_.size())
        {
            std::snprintf(output, outputSize, "residency n/a");
            return;
        }
        const VirtualResource& resource = virtualResources_[slot];
        const StreamingResourceState state =
            sceneTreeStatsValid_ ? sceneTreeResourceStates_[slot]
                                 : resource.state;
        const bool fallbackPinned =
            sceneTreeStatsValid_ &&
            sceneTreeFallbackPinned_[slot] &&
            state == StreamingResourceState::Resident;
        const char* pinLabel = resource.pinned
                                   ? "/pinned-permanent"
                                   : fallbackPinned
                                         ? "/pinned-fallback"
                                         : "";
        std::snprintf(output, outputSize, "%s%s",
                      streamingStateName(state), pinLabel);
    }

    bool beginPayloadTreeNode(const char* id, const char* label,
                              Payload payload, bool defaultOpen = false,
                              uint16_t heroAsset = UINT16_MAX)
    {
        const uint32_t current =
            sceneTreeCurrentCount(payload, heroAsset);
        const ImVec4 color = current != 0
                                 ? payloadUiColor(payload)
                                 : ImVec4(0.52f, 0.52f, 0.56f, 1.0f);
        char residency[48];
        sceneTreeResidencyLabel(payload, heroAsset, residency,
                                sizeof(residency));
        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGuiTreeNodeFlags flags =
            ImGuiTreeNodeFlags_SpanAvailWidth |
            ImGuiTreeNodeFlags_OpenOnArrow;
        if (defaultOpen)
            flags |= ImGuiTreeNodeFlags_DefaultOpen;
        const uint32_t entityMask = entityKindBit(
            payloadEntityKind(payload));
        char selectionLabel[128];
        if (heroAsset < kHeroTowerAssetCount)
            std::snprintf(selectionLabel, sizeof(selectionLabel),
                          "Hero asset %02u / %s",
                          uint32_t(heroAsset) + 1, label);
        else
            std::snprintf(selectionLabel, sizeof(selectionLabel),
                          "%s", label);
        if (sceneTreeSelectionEquals(entityMask, heroAsset,
                                     payload, payload, selectionLabel))
            flags |= ImGuiTreeNodeFlags_Selected;
        const bool open = ImGui::TreeNodeEx(
            id, flags, "%s  [error %.2f m | %s | current %u]", label,
            authoredGeometricError(payload), residency, current);
        if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
        {
            toggleSceneTreeSelection(entityMask, heroAsset, payload,
                                     payload, selectionLabel);
        }
        ImGui::PopStyleColor();
        return open;
    }

    void drawPayloadTreeLeaf(const char* id, const char* label,
                             Payload payload,
                             uint16_t heroAsset = UINT16_MAX)
    {
        const uint32_t current =
            sceneTreeCurrentCount(payload, heroAsset);
        const ImVec4 color = current != 0
                                 ? payloadUiColor(payload)
                                 : ImVec4(0.52f, 0.52f, 0.56f, 1.0f);
        char residency[48];
        sceneTreeResidencyLabel(payload, heroAsset, residency,
                                sizeof(residency));
        ImGui::PushStyleColor(ImGuiCol_Text, color);
        const uint32_t entityMask = entityKindBit(
            payloadEntityKind(payload));
        ImGuiTreeNodeFlags flags =
            ImGuiTreeNodeFlags_Leaf |
            ImGuiTreeNodeFlags_NoTreePushOnOpen |
            ImGuiTreeNodeFlags_SpanAvailWidth;
        char selectionLabel[128];
        if (heroAsset < kHeroTowerAssetCount)
            std::snprintf(selectionLabel, sizeof(selectionLabel),
                          "Hero asset %02u / %s",
                          uint32_t(heroAsset) + 1, label);
        else
            std::snprintf(selectionLabel, sizeof(selectionLabel),
                          "%s", label);
        if (sceneTreeSelectionEquals(entityMask, heroAsset,
                                     payload, payload, selectionLabel))
            flags |= ImGuiTreeNodeFlags_Selected;
        ImGui::TreeNodeEx(
            id, flags,
            "%s  [error %.2f m | %s | current %u]", label,
            authoredGeometricError(payload), residency, current);
        if (ImGui::IsItemClicked())
        {
            toggleSceneTreeSelection(entityMask, heroAsset, payload,
                                     payload, selectionLabel);
        }
        ImGui::PopStyleColor();
    }

    void drawTowerPlacementTreeLeaf(InstanceHandle handle,
                                    uint32_t placementOrdinal)
    {
        if (handle.id >= entities_.size())
            return;
        const Entity& entity = entities_[handle.id];
        if (entity.kind != EntityKind::Tower ||
            entity.heroAsset >= kHeroTowerAssetCount)
            return;

        const uint32_t current =
            handle.id < currentInstanceEntryCounts_.size()
                ? currentInstanceEntryCounts_[handle.id]
                : 0;
        char residency[48];
        sceneTreeResidencyLabel(Payload::TowerTop, entity.heroAsset,
                                residency, sizeof(residency));
        char displayLabel[192];
        std::snprintf(
            displayLabel, sizeof(displayLabel),
            "Placement %u: instance %u at (%.1f, %.1f, %.1f)  "
            "[error %.2f m | %s | current %u]",
            placementOrdinal, handle.id, entity.position.x,
            entity.position.y, entity.position.z,
            authoredGeometricError(Payload::TowerTop), residency, current);
        char selectionLabel[128];
        std::snprintf(selectionLabel, sizeof(selectionLabel),
                      "Hero asset %02u / placement %u (instance %u)",
                      uint32_t(entity.heroAsset) + 1, placementOrdinal,
                      handle.id);

        ImGuiTreeNodeFlags flags =
            ImGuiTreeNodeFlags_Leaf |
            ImGuiTreeNodeFlags_NoTreePushOnOpen |
            ImGuiTreeNodeFlags_SpanAvailWidth;
        const uint32_t towerMask = entityKindBit(EntityKind::Tower);
        if (sceneTreeSelectionEquals(
                towerMask, entity.heroAsset, Payload::TowerTop,
                Payload::TowerCrown, selectionLabel, handle.id))
            flags |= ImGuiTreeNodeFlags_Selected;

        ImGui::PushID(int(handle.id));
        ImGui::TreeNodeEx("placement", flags, "%s", displayLabel);
        if (ImGui::IsItemClicked())
        {
            toggleSceneTreeSelection(
                towerMask, entity.heroAsset, Payload::TowerTop,
                Payload::TowerCrown, selectionLabel, handle.id);
        }
        if (pendingHierarchyRevealInstance_ == handle.id)
        {
            ImGui::SetScrollHereY(0.5f);
            pendingHierarchyRevealHero_ = UINT16_MAX;
            pendingHierarchyRevealInstance_ = kInvalidInstanceId;
        }
        ImGui::PopID();
    }

    void drawSceneTreeUi()
    {
        const bool revealTower =
            pendingHierarchyRevealHero_ < kHeroTowerAssetCount;
        if (revealTower)
        {
            ImGui::SetNextWindowFocus();
            ImGui::SetNextWindowCollapsed(false, ImGuiCond_Always);
        }
        ImGui::SetNextWindowPos(ImVec2(776.0f, 36.0f + city::windowFrameHeight()),
                                ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(720.0f, 652.0f),
                                 ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Scene hierarchy", &showSceneHierarchy_))
        {
            ImGui::End();
            if (!showSceneHierarchy_)
            {
                sceneTreeSelection_ = {};
                pendingHierarchyRevealHero_ = UINT16_MAX;
                pendingHierarchyRevealInstance_ = kInvalidInstanceId;
                sceneTreeStatsValid_ = false;
            }
            return;
        }
        ImGui::TextWrapped(
            "Live authored Frontier topology. Payload rows show local-space "
            "geometric error, virtual residency, planner pinning, and current "
            "selected-entry count. Permanent coarsest resources are shown as "
            "pinned-permanent; ready fallback ancestors protected by the "
            "streaming planner are shown as pinned-fallback. Click a row to "
            "tint matching current-cut geometry; click it again to clear the "
            "selection. Ctrl+left-click a skyscraper in the viewport to "
            "select and reveal its exact placement.");
        if (sceneTreeSelection_.active())
        {
            ImGui::TextColored(ImVec4(0.18f, 0.90f, 1.0f, 1.0f),
                               "Selected: %s",
                               sceneTreeSelection_.label.c_str());
            ImGui::SameLine();
            if (ImGui::SmallButton("Clear selection"))
                sceneTreeSelection_ = {};
        }
        else
        {
            ImGui::TextDisabled("Selected: none");
        }
        ImGui::Separator();

        const uint32_t staticCount = houseCount_ + towerCount_ + treeCount_;
        const uint32_t dynamicCount = uint32_t(carHandles_.size() +
                                               pedestrianHandles_.size());
        constexpr uint32_t staticKindMask =
            entityKindBit(EntityKind::House) |
            entityKindBit(EntityKind::Tree) |
            entityKindBit(EntityKind::Tower);
        constexpr uint32_t dynamicKindMask =
            entityKindBit(EntityKind::Car) |
            entityKindBit(EntityKind::Pedestrian);
        char treeLabel[160];
        std::snprintf(treeLabel, sizeof(treeLabel),
                      "City scene  (%u instances)",
                      staticCount + dynamicCount);
        if (revealTower)
            ImGui::SetNextItemOpen(true, ImGuiCond_Always);
        if (beginSceneTreeNode(
                "scene-root", treeLabel, kAllEntityKindMask, UINT16_MAX,
                Payload::HouseTop, Payload::TowerCrown, true,
                "City scene"))
        {
            std::snprintf(treeLabel, sizeof(treeLabel),
                          "%s environment  (%u)",
                          animateWholeScene_ ? "Animated" : "Static",
                          staticCount);
            if (revealTower)
                ImGui::SetNextItemOpen(true, ImGuiCond_Always);
            if (beginSceneTreeNode(
                    "static-scene", treeLabel, staticKindMask,
                    UINT16_MAX, Payload::HouseTop,
                    Payload::TowerCrown, true, "Static environment"))
            {
                std::snprintf(
                    treeLabel, sizeof(treeLabel),
                    "Houses %s, generation %u  (%u instances)",
                    activeHouseStyle_ == HouseStyle::HouseA ? "A" : "B",
                    houseGeneration_, houseCount_);
                if (beginSceneTreeNode(
                        "houses", treeLabel,
                        entityKindBit(EntityKind::House), UINT16_MAX,
                        Payload::HouseTop, Payload::HouseRoof, false,
                        "Houses"))
                {
                    if (beginPayloadTreeNode("house-top", "Top / fallback",
                                             Payload::HouseTop, true))
                    {
                        if (beginPayloadTreeNode("house-coarse", "Coarse",
                                                 Payload::HouseCoarse, true))
                        {
                            drawPayloadTreeLeaf("house-body", "Body",
                                                Payload::HouseBody);
                            drawPayloadTreeLeaf("house-roof", "Roof",
                                                Payload::HouseRoof);
                            ImGui::TreePop();
                        }
                        ImGui::TreePop();
                    }
                    ImGui::TreePop();
                }

                std::snprintf(
                    treeLabel, sizeof(treeLabel),
                    "Skyscrapers  (%u instances, %u shared assets)",
                    towerCount_, uint32_t(kHeroTowerAssetCount));
                if (revealTower)
                    ImGui::SetNextItemOpen(true, ImGuiCond_Always);
                if (beginSceneTreeNode(
                        "towers", treeLabel,
                        entityKindBit(EntityKind::Tower), UINT16_MAX,
                        Payload::TowerTop, Payload::TowerCrown, true,
                        "Skyscrapers"))
                {
                    ImGui::TextDisabled(
                        "Each asset has three instance roots and one shared "
                        "streamed definition.");
                    for (uint16_t hero = 0;
                         hero < kHeroTowerAssetCount; ++hero)
                    {
                        uint32_t currentEntries = 0;
                        for (uint32_t count :
                             currentHeroPayloadCounts_[hero])
                            currentEntries += count;
                        uint32_t resident = 0;
                        uint32_t loading = 0;
                        uint32_t unloaded = 0;
                        uint32_t fallbackPinned = 0;
                        const size_t resourceBase =
                            kHeroTowerResourceBase +
                            size_t(hero) * kTowerDetailResourceCount;
                        for (size_t detail = 0;
                             detail < kTowerDetailResourceCount; ++detail)
                        {
                            const size_t slot = resourceBase + detail;
                            const StreamingResourceState state =
                                sceneTreeStatsValid_
                                    ? sceneTreeResourceStates_[slot]
                                    : virtualResources_[slot].state;
                            switch (state)
                            {
                            case StreamingResourceState::Resident:
                                ++resident;
                                break;
                            case StreamingResourceState::Loading:
                                ++loading;
                                break;
                            case StreamingResourceState::Unloaded:
                                ++unloaded;
                                break;
                            }
                            if (sceneTreeStatsValid_ &&
                                state == StreamingResourceState::Resident &&
                                sceneTreeFallbackPinned_[slot])
                                ++fallbackPinned;
                        }

                        ImGui::PushID(int(hero));
                        std::snprintf(
                            treeLabel, sizeof(treeLabel),
                            "Hero asset %02u  (%u placements | "
                            "resident/loading/unloaded "
                            "%u/%u/%u | fallback-pinned %u | current %u)",
                            uint32_t(hero) + 1,
                            uint32_t(kHeroTowerInstancesPerAsset), resident,
                            loading, unloaded, fallbackPinned,
                            currentEntries);
                        char heroSelectionLabel[32];
                        std::snprintf(heroSelectionLabel,
                                      sizeof(heroSelectionLabel),
                                      "Hero asset %02u",
                                      uint32_t(hero) + 1);
                        const bool revealHero =
                            revealTower &&
                            pendingHierarchyRevealHero_ == hero;
                        if (revealHero)
                            ImGui::SetNextItemOpen(true, ImGuiCond_Always);
                        const bool assetOpen = beginSceneTreeNode(
                            "hero-asset", treeLabel,
                            entityKindBit(EntityKind::Tower), hero,
                            Payload::TowerTop, Payload::TowerCrown,
                            hero == 0, heroSelectionLabel);
                        if (assetOpen)
                        {
                            char topologySelectionLabel[80];
                            std::snprintf(
                                topologySelectionLabel,
                                sizeof(topologySelectionLabel),
                                "Hero asset %02u / instance root nodes",
                                uint32_t(hero) + 1);
                            if (revealHero)
                                ImGui::SetNextItemOpen(
                                    true, ImGuiCond_Always);
                            if (beginSceneTreeNode(
                                    "instance-roots",
                                    "Instance root nodes (3 independent "
                                    "nodes)",
                                    entityKindBit(EntityKind::Tower), hero,
                                    Payload::TowerTop, Payload::TowerCrown,
                                    true, topologySelectionLabel))
                            {
                                uint32_t placementOrdinal = 0;
                                for (InstanceHandle handle : towerHandles_)
                                {
                                    if (handle.id >= entities_.size() ||
                                        entities_[handle.id].heroAsset != hero)
                                        continue;
                                    drawTowerPlacementTreeLeaf(
                                        handle, ++placementOrdinal);
                                }
                                drawPayloadTreeLeaf(
                                    "tower-top-payload",
                                    "payload slot 0: Top / fallback",
                                    Payload::TowerTop, hero);
                                std::snprintf(
                                    topologySelectionLabel,
                                    sizeof(topologySelectionLabel),
                                    "Hero asset %02u / facade node",
                                    uint32_t(hero) + 1);
                                if (beginSceneTreeNode(
                                        "facade-node",
                                        "Mounted facade node (1 shared "
                                        "definition, 3 placements, 4 "
                                        "payload slots)",
                                        entityKindBit(EntityKind::Tower),
                                        hero, Payload::TowerDistrict,
                                        Payload::TowerCrown, true,
                                        topologySelectionLabel))
                                {
                                    drawPayloadTreeLeaf(
                                        "tower-district-payload",
                                        "payload slot 0: District",
                                        Payload::TowerDistrict, hero);
                                    drawPayloadTreeLeaf(
                                        "tower-coarse-payload",
                                        "payload slot 1: Coarse",
                                        Payload::TowerCoarse, hero);
                                    drawPayloadTreeLeaf(
                                        "tower-medium-payload",
                                        "payload slot 2: Medium",
                                        Payload::TowerMedium, hero);
                                    drawPayloadTreeLeaf(
                                        "tower-fine-payload",
                                        "payload slot 3: Fine",
                                        Payload::TowerFine, hero);
                                    std::snprintf(
                                        topologySelectionLabel,
                                        sizeof(topologySelectionLabel),
                                        "Hero asset %02u / detail children",
                                        uint32_t(hero) + 1);
                                    if (beginSceneTreeNode(
                                            "detail-children",
                                            "Structural detail child nodes",
                                            entityKindBit(EntityKind::Tower),
                                            hero, Payload::TowerBase,
                                            Payload::TowerCrown, true,
                                            topologySelectionLabel))
                                    {
                                        drawPayloadTreeLeaf(
                                            "tower-base-node",
                                            "Base node, payload slot 0",
                                            Payload::TowerBase, hero);
                                        drawPayloadTreeLeaf(
                                            "tower-shaft-node",
                                            "Shaft node, payload slot 0",
                                            Payload::TowerShaft, hero);
                                        drawPayloadTreeLeaf(
                                            "tower-crown-node",
                                            "Crown node, payload slot 0",
                                            Payload::TowerCrown, hero);
                                        ImGui::TreePop();
                                    }
                                    ImGui::TreePop();
                                }
                                ImGui::TreePop();
                            }
                            ImGui::TreePop();
                        }
                        ImGui::PopID();
                    }
                    ImGui::TreePop();
                }

                std::snprintf(treeLabel, sizeof(treeLabel),
                              "Trees  (%u instances)", treeCount_);
                if (beginSceneTreeNode(
                        "trees", treeLabel,
                        entityKindBit(EntityKind::Tree), UINT16_MAX,
                        Payload::TreeTop, Payload::TreeCrown, false,
                        "Trees"))
                {
                    if (beginPayloadTreeNode("tree-top", "Top / fallback",
                                             Payload::TreeTop, true))
                    {
                        if (beginPayloadTreeNode("tree-coarse", "Coarse",
                                                 Payload::TreeCoarse, true))
                        {
                            drawPayloadTreeLeaf("tree-trunk", "Trunk",
                                                Payload::TreeTrunk);
                            drawPayloadTreeLeaf("tree-crown", "Crown",
                                                Payload::TreeCrown);
                            ImGui::TreePop();
                        }
                        ImGui::TreePop();
                    }
                    ImGui::TreePop();
                }
                ImGui::TreePop();
            }

            std::snprintf(treeLabel, sizeof(treeLabel),
                          "Dynamic actors  (%u)", dynamicCount);
            if (beginSceneTreeNode(
                    "dynamic-scene", treeLabel, dynamicKindMask,
                    UINT16_MAX, Payload::HouseTop,
                    Payload::TowerCrown, true, "Dynamic actors"))
            {
                std::snprintf(treeLabel, sizeof(treeLabel),
                              "Cars  (%u instances)",
                              unsigned(carHandles_.size()));
                if (beginSceneTreeNode(
                        "cars", treeLabel,
                        entityKindBit(EntityKind::Car), UINT16_MAX,
                        Payload::CarTop, Payload::CarCabin, false,
                        "Cars"))
                {
                    if (beginPayloadTreeNode("car-top", "Top / fallback",
                                             Payload::CarTop, true))
                    {
                        if (beginPayloadTreeNode("car-coarse", "Coarse",
                                                 Payload::CarCoarse, true))
                        {
                            drawPayloadTreeLeaf("car-body", "Body",
                                                Payload::CarBody);
                            drawPayloadTreeLeaf("car-cabin", "Cabin",
                                                Payload::CarCabin);
                            ImGui::TreePop();
                        }
                        ImGui::TreePop();
                    }
                    ImGui::TreePop();
                }

                std::snprintf(treeLabel, sizeof(treeLabel),
                              "Pedestrians  (%u instances)",
                              unsigned(pedestrianHandles_.size()));
                if (beginSceneTreeNode(
                        "pedestrians", treeLabel,
                        entityKindBit(EntityKind::Pedestrian), UINT16_MAX,
                        Payload::PedestrianTop,
                        Payload::PedestrianHead, false,
                        "Pedestrians"))
                {
                    if (beginPayloadTreeNode(
                            "pedestrian-top", "Top / fallback",
                            Payload::PedestrianTop, true))
                    {
                        if (beginPayloadTreeNode(
                                "pedestrian-coarse", "Coarse",
                                Payload::PedestrianCoarse, true))
                        {
                            drawPayloadTreeLeaf(
                                "pedestrian-body", "Body",
                                Payload::PedestrianBody);
                            drawPayloadTreeLeaf(
                                "pedestrian-head", "Head",
                                Payload::PedestrianHead);
                            ImGui::TreePop();
                        }
                        ImGui::TreePop();
                    }
                    ImGui::TreePop();
                }
                ImGui::TreePop();
            }
            ImGui::TreePop();
        }
        ImGui::End();
        if (!showSceneHierarchy_)
        {
            sceneTreeSelection_ = {};
            pendingHierarchyRevealHero_ = UINT16_MAX;
            pendingHierarchyRevealInstance_ = kInvalidInstanceId;
            sceneTreeStatsValid_ = false;
        }
    }

    float cameraAspect() const
    {
        return height_ != 0 ? float(width_) / float(height_) : 1.0f;
    }

    CameraPose makeCameraPose(float4 position, float4 target) const
    {
        CameraPose pose;
        pose.position = position;
        pose.target = target;
        bx::mtxLookAt(pose.view.data(),
                      bx::Vec3{position.x, position.y, position.z},
                      bx::Vec3{target.x, target.y, target.z});
        bx::mtxProj(pose.projection.data(), 58.0f, cameraAspect(),
                    0.25f, kCameraFarPlane,
                    bgfx::getCaps()->homogeneousDepth);
        bx::mtxMul(pose.viewProjection.data(), pose.view.data(),
                   pose.projection.data());
        return pose;
    }

    CameraPose makeFreeCameraPose() const
    {
        const bx::Vec3 position = cameraGetPosition();
        const bx::Vec3 target = cameraGetAt();
        CameraPose pose = makeCameraPose(
            float4::point(position.x, position.y, position.z),
            float4::point(target.x, target.y, target.z));
        cameraGetViewMtx(pose.view.data());
        bx::mtxMul(pose.viewProjection.data(), pose.view.data(),
                   pose.projection.data());
        return pose;
    }

    Camera makeFrontierCamera(const CameraPose& pose) const
    {
        const float pixelHeight =
            streamingSelfTest_ && streamingTestViewportHeight_ > 0.0f
                ? streamingTestViewportHeight_
                : float(height_);
        return makePerspectiveCamera(
            pose.position, pose.target - pose.position,
            float4::vec(0.0f, 1.0f, 0.0f), 58.0f * kPi / 180.0f,
            cameraAspect(), pixelHeight, 0.25f, kCameraFarPlane);
    }

    void seedFreeCameraFrom(float4 position, float4 target)
    {
        const float4 direction = target - position;
        const float length = std::sqrt(direction.x * direction.x +
                                       direction.y * direction.y +
                                       direction.z * direction.z);
        const float inverseLength = length > 1.0e-6f ? 1.0f / length : 1.0f;
        const float x = direction.x * inverseLength;
        const float y = direction.y * inverseLength;
        const float z = direction.z * inverseLength;
        cameraSetPosition(bx::Vec3{position.x, position.y, position.z});
        cameraSetHorizontalAngle(std::atan2(x, z));
        cameraSetVerticalAngle(std::asin(std::clamp(y, -1.0f, 1.0f)));
        cameraUpdate(0.0f, mouse_, true);
    }

    void captureFrozenCull(const CameraPose& pose)
    {
        frozenCull_.pose = pose;
        frozenCull_.camera = makeFrontierCamera(pose);
        frozenCull_.valid = true;
        query_.reset();
        streamingLookaheadQuery_.reset();
    }

    void selectTowerInstance(InstanceId instance)
    {
        if (instance >= entities_.size())
            return;
        const Entity& entity = entities_[instance];
        if (entity.kind != EntityKind::Tower ||
            entity.heroAsset >= kHeroTowerAssetCount)
            return;

        uint32_t placementOrdinal = 0;
        bool found = false;
        for (InstanceHandle handle : towerHandles_)
        {
            if (handle.id >= entities_.size() ||
                entities_[handle.id].heroAsset != entity.heroAsset)
                continue;
            ++placementOrdinal;
            if (handle.id == instance)
            {
                found = true;
                break;
            }
        }
        if (!found)
            return;

        char selectionLabel[128];
        std::snprintf(selectionLabel, sizeof(selectionLabel),
                      "Hero asset %02u / placement %u (instance %u)",
                      uint32_t(entity.heroAsset) + 1, placementOrdinal,
                      instance);
        sceneTreeSelection_.entityKindMask =
            entityKindBit(EntityKind::Tower);
        sceneTreeSelection_.heroAsset = entity.heroAsset;
        sceneTreeSelection_.instance = instance;
        sceneTreeSelection_.firstPayload = Payload::TowerTop;
        sceneTreeSelection_.lastPayload = Payload::TowerCrown;
        sceneTreeSelection_.label = selectionLabel;
        pendingHierarchyRevealHero_ = entity.heroAsset;
        pendingHierarchyRevealInstance_ = instance;
        showSceneHierarchy_ = true;
    }

    void pickTowerAtMouse(const CameraPose& pose)
    {
        if (width_ == 0 || height_ == 0 || mouse_.m_mx < 0 ||
            mouse_.m_my < 0 || uint32_t(mouse_.m_mx) >= width_ ||
            uint32_t(mouse_.m_my) >= height_)
            return;

        float inverseViewProjection[16];
        bx::mtxInverse(inverseViewProjection, pose.viewProjection.data());
        const float mouseX =
            float(mouse_.m_mx) / float(width_) * 2.0f - 1.0f;
        const float mouseY =
            -(float(mouse_.m_my) / float(height_) * 2.0f - 1.0f);
        const bx::Ray ray =
            bx::makeRay(mouseX, mouseY, inverseViewProjection);

        float nearestDistance = std::numeric_limits<float>::max();
        InstanceId nearestInstance = kInvalidInstanceId;
        for (InstanceHandle handle : towerHandles_)
        {
            if (handle.id >= entities_.size())
                continue;
            const Entity& entity = entities_[handle.id];
            if (entity.kind != EntityKind::Tower)
                continue;

            const float halfExtent =
                kTowerRootHalfExtent * entity.scale;
            const bx::Aabb worldBounds = {
                {entity.position.x - halfExtent, entity.position.y,
                 entity.position.z - halfExtent},
                {entity.position.x + halfExtent,
                 entity.position.y + kTowerRootHeight * entity.scale,
                 entity.position.z + halfExtent},
            };
            bx::Hit hit;
            if (!bx::intersect(ray, worldBounds, &hit))
                continue;
            const float distance = std::max(hit.plane.dist, 0.0f);
            if (distance < nearestDistance)
            {
                nearestDistance = distance;
                nearestInstance = handle.id;
            }
        }

        if (nearestInstance != kInvalidInstanceId)
            selectTowerInstance(nearestInstance);
    }

    static float milliseconds(int64_t begin, int64_t end)
    {
        return float(double(end - begin) * 1000.0 /
                     double(bx::getHPFrequency()));
    }

    void updateFrontierStats(const FrontierResultView& frontier)
    {
        currentPayloadCounts_.fill(0);
        for (auto& counts : currentHeroPayloadCounts_)
            counts.fill(0);
        if (showSceneHierarchy_)
        {
            for (size_t slot = 0; slot < virtualResources_.size(); ++slot)
                sceneTreeResourceStates_[slot] =
                    virtualResources_[slot].state;
            sceneTreeStatsValid_ = true;
            if (currentInstanceEntryCounts_.size() < entities_.size())
                currentInstanceEntryCounts_.resize(entities_.size(), 0);
            std::fill(currentInstanceEntryCounts_.begin(),
                      currentInstanceEntryCounts_.end(), 0);
            for (const FrontierEntry& entry : frontier)
            {
                const UserPayload rawPayload =
                    database_.tryGetPayload(entry.nodeHandle,
                                            entry.payloadIndex());
                const size_t slot = size_t(rawPayload);
                if (rawPayload != kInvalidPayload &&
                    slot < kPayloadSlotCount)
                {
                    ++currentPayloadCounts_[slot];
                    if (entry.instance() <
                        currentInstanceEntryCounts_.size())
                        ++currentInstanceEntryCounts_[entry.instance()];
                    if (rawPayload >= UserPayload(Payload::TowerTop) &&
                        rawPayload <= UserPayload(Payload::TowerCrown) &&
                        entry.instance() < entities_.size())
                    {
                        const Entity& entity = entities_[entry.instance()];
                        if (entity.kind == EntityKind::Tower &&
                            entity.heroAsset < kHeroTowerAssetCount)
                        {
                            ++currentHeroPayloadCounts_[entity.heroAsset][
                                slot - size_t(Payload::TowerTop)];
                        }
                    }
                }
            }
        }
        lastCurrentSize_ = uint32_t(frontier.size());
        lastQueryReused_ = query_.reused();
        lastQueryWalked_ = query_.walked();
    }

    void captureBgfxPerformance(PerformanceSample& sample) const
    {
        const bgfx::Stats* stats = bgfx::getStats();
        if (stats == nullptr)
            return;

        const auto durationMs = [](int64_t duration, int64_t frequency)
        {
            return frequency > 0
                       ? float(double(duration) * 1000.0 /
                               double(frequency))
                       : 0.0f;
        };
        sample.renderThreadMs =
            durationMs(std::max(int64_t(0),
                                stats->cpuTimeEnd - stats->cpuTimeBegin),
                       stats->cpuTimerFreq);
        sample.gpuMs =
            durationMs(std::max(int64_t(0),
                                stats->gpuTimeEnd - stats->gpuTimeBegin),
                       stats->gpuTimerFreq);
        sample.waitRenderMs =
            durationMs(std::max(int64_t(0), stats->waitRender),
                       stats->cpuTimerFreq);
        sample.waitSubmitMs =
            durationMs(std::max(int64_t(0), stats->waitSubmit),
                       stats->cpuTimerFreq);
        sample.drawCalls = stats->numDraw;
        sample.triangles = stats->numPrims[bgfx::Topology::TriList];
        sample.transientVertexBytes = stats->transientVbUsed;
        sample.transientIndexBytes = stats->transientIbUsed;
    }

    void recordPerformance(const PerformanceSample& sample, float deltaTime)
    {
        const bool firstSample = performanceSampleCount_ == 0;
        const auto smooth = [firstSample](float& destination, float value)
        {
            destination = firstSample
                              ? value
                              : destination + (value - destination) * 0.10f;
        };
        smooth(performance_.totalMs, sample.totalMs);
        smooth(performance_.uiMs, sample.uiMs);
        smooth(performance_.simulationMs, sample.simulationMs);
        smooth(performance_.applyUpdatesMs, sample.applyUpdatesMs);
        smooth(performance_.tlasRebuildMs, sample.tlasRebuildMs);
        smooth(performance_.cameraMs, sample.cameraMs);
        smooth(performance_.selectionMs, sample.selectionMs);
        smooth(performance_.cutStatsMs, sample.cutStatsMs);
        smooth(performance_.renderMs, sample.renderMs);
        smooth(performance_.streamingMs, sample.streamingMs);
        smooth(performance_.streamingRefinementMs,
               sample.streamingRefinementMs);
        smooth(performance_.streamingPlannerMs, sample.streamingPlannerMs);
        smooth(performance_.streamingPlannerIndexMs,
               sample.streamingPlannerIndexMs);
        smooth(performance_.streamingPlannerDemandMs,
               sample.streamingPlannerDemandMs);
        smooth(performance_.streamingPlannerScoreMs,
               sample.streamingPlannerScoreMs);
        smooth(performance_.streamingPlannerResidencyMs,
               sample.streamingPlannerResidencyMs);
        smooth(performance_.streamingPlannerOtherMs,
               sample.streamingPlannerOtherMs);
        smooth(performance_.streamingScenarioMs, sample.streamingScenarioMs);
        smooth(performance_.frameSubmitMs, sample.frameSubmitMs);
        smooth(performance_.renderThreadMs, sample.renderThreadMs);
        smooth(performance_.gpuMs, sample.gpuMs);
        smooth(performance_.waitRenderMs, sample.waitRenderMs);
        smooth(performance_.waitSubmitMs, sample.waitSubmitMs);
        performance_.drawCalls = sample.drawCalls;
        performance_.triangles = sample.triangles;
        performance_.transientVertexBytes = sample.transientVertexBytes;
        performance_.transientIndexBytes = sample.transientIndexBytes;

        performanceHistoryElapsed_ += deltaTime;
        if (performanceHistoryCount_ == 0 ||
            performanceHistoryElapsed_ >=
                kPerformanceHistorySampleInterval)
        {
            performanceHistoryElapsed_ = std::fmod(
                performanceHistoryElapsed_,
                kPerformanceHistorySampleInterval);
            for (size_t timer = 0; timer < kPerformanceTimerCount; ++timer)
            {
                performanceHistory_[timer][performanceHistoryCursor_] =
                    performanceTimerMs(
                        sample, static_cast<PerformanceTimer>(timer)) *
                    1000.0f;
            }
            performanceHistoryCursor_ =
                (performanceHistoryCursor_ + 1) % kPerformanceHistorySize;
            performanceHistoryCount_ =
                std::min(performanceHistoryCount_ + 1,
                         kPerformanceHistorySize);
        }
        ++performanceSampleCount_;
    }

    void createRoofGeometry()
    {
        const DdVertex vertices[] = {
            {-3.2f, 5.4f, -3.2f},
            { 3.2f, 5.4f, -3.2f},
            { 3.2f, 5.4f,  3.2f},
            {-3.2f, 5.4f,  3.2f},
            { 0.0f, 8.5f,  0.0f},
        };
        const uint16_t indices[] = {
            0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4,
            0, 3, 2, 0, 2, 1,
        };
        roofGeometry_ = ddCreateGeometry(
            uint32_t(std::size(vertices)), vertices,
            uint32_t(std::size(indices)), indices);
    }

    static AABB houseBounds(HouseStyle style)
    {
        return style == HouseStyle::HouseA
                   ? bounds(-3.4f, 0.0f, -3.4f, 3.4f, 8.8f, 3.4f)
                   : bounds(-3.7f, 0.0f, -3.7f, 3.7f, 7.6f, 3.7f);
    }

    SceneDefinition createHouseDefinition(HouseStyle style)
    {
        // Geometric errors are world-space deviations. Sub-meter values make
        // the default 0.75 px threshold span several LODs across this city rather
        // than forcing every visible object to its zero-error leaves.
        SceneDefinitionBuilder builder;
        const bool houseA = style == HouseStyle::HouseA;
        const AABB all = houseA
                             ? bounds(-3.3f, 0.0f, -3.3f,
                                      3.3f, 8.6f, 3.3f)
                             : bounds(-3.6f, 0.0f, -3.0f,
                                      3.6f, 7.4f, 3.0f);
        const auto coarse = builder.createNode(
            node(Payload::HouseCoarse,
                 authoredGeometricError(Payload::HouseCoarse), all));
        builder.createNode(
            coarse,
            node(Payload::HouseBody, 0.0f,
                 houseA ? bounds(-3.0f, 0.0f, -3.0f,
                                 3.0f, 5.6f, 3.0f)
                        : bounds(-3.4f, 0.0f, -2.8f,
                                 3.4f, 6.5f, 2.8f)));
        builder.createNode(
            coarse,
            node(Payload::HouseRoof, 0.0f,
                 houseA ? bounds(-3.3f, 5.3f, -3.3f,
                                 3.3f, 8.6f, 3.3f)
                        : bounds(-3.6f, 6.3f, -3.0f,
                                 3.6f, 7.4f, 3.0f)));
        return builder.registerWith(database_);
    }

    SceneDefinition createCarDefinition()
    {
        SceneDefinitionBuilder builder;
        const AABB all = bounds(-2.2f, 0.0f, -2.2f, 2.2f, 2.2f, 2.2f);
        const auto coarse = builder.createNode(
            node(Payload::CarCoarse,
                 authoredGeometricError(Payload::CarCoarse), all));
        builder.createNode(coarse, node(Payload::CarBody, 0.0f,
                                        bounds(-2.1f, 0.1f, -1.0f,
                                               2.1f, 1.0f, 1.0f)));
        builder.createNode(coarse, node(Payload::CarCabin, 0.0f,
                                        bounds(-0.9f, 0.9f, -0.85f,
                                               1.0f, 1.8f, 0.85f)));
        return builder.registerWith(database_);
    }

    SceneDefinition createPedestrianDefinition()
    {
        SceneDefinitionBuilder builder;
        const AABB all = bounds(-1.1f, 0.0f, -1.1f, 1.1f, 2.25f, 1.1f);
        const auto coarse = builder.createNode(
            node(Payload::PedestrianCoarse,
                 authoredGeometricError(Payload::PedestrianCoarse), all));
        builder.createNode(coarse, node(Payload::PedestrianBody, 0.0f,
                                        bounds(-0.35f, 0.05f, -0.35f,
                                               0.35f, 1.75f, 0.35f)));
        builder.createNode(coarse, node(Payload::PedestrianHead, 0.0f,
                                        bounds(-0.3f, 1.65f, -0.3f,
                                               0.58f, 2.25f, 0.3f)));
        return builder.registerWith(database_);
    }

    SceneDefinition createTreeDefinition()
    {
        SceneDefinitionBuilder builder;
        const AABB all = bounds(-1.8f, 0.0f, -1.8f, 1.8f, 6.5f, 1.8f);
        const auto coarse = builder.createNode(
            node(Payload::TreeCoarse,
                 authoredGeometricError(Payload::TreeCoarse), all));
        builder.createNode(coarse, node(Payload::TreeTrunk, 0.0f,
                                        bounds(-0.35f, 0.0f, -0.35f,
                                               0.35f, 3.4f, 0.35f)));
        builder.createNode(coarse, node(Payload::TreeCrown, 0.0f,
                                        bounds(-1.8f, 2.2f, -1.8f,
                                               1.8f, 6.5f, 1.8f)));
        return builder.registerWith(database_);
    }

    SceneDefinition createTowerDefinition()
    {
        SceneDefinitionBuilder builder;
        const AABB all = bounds(-5.0f, 0.0f, -5.0f, 5.0f, 46.0f, 5.0f);
        // TowerTop remains the permanent per-instance fallback. Streamed
        // facade payloads live on the registered definition so readiness and
        // virtual resource cost stay shared by all placements of this asset.
        const std::array<PayloadLodDesc,
                         kTowerFacadePayloadCount - 1> facadeLods{{
            {UserPayload(Payload::TowerCoarse),
             authoredGeometricError(Payload::TowerCoarse)},
            {UserPayload(Payload::TowerMedium),
             authoredGeometricError(Payload::TowerMedium)},
            {UserPayload(Payload::TowerFine),
             authoredGeometricError(Payload::TowerFine)},
        }};
        const auto district = builder.createNode(
            node(Payload::TowerDistrict,
                 authoredGeometricError(Payload::TowerDistrict), all),
            facadeLods);
        builder.createNode(district, node(Payload::TowerBase, 0.0f,
                                          bounds(-5.0f, 0.0f, -5.0f,
                                                 5.0f, 7.0f, 5.0f)));
        builder.createNode(district, node(Payload::TowerShaft, 0.0f,
                                          bounds(-4.2f, 6.8f, -4.2f,
                                                 4.2f, 38.0f, 4.2f)));
        builder.createNode(district, node(Payload::TowerCrown, 0.0f,
                                          bounds(-4.2f, 37.8f, -4.2f,
                                                 4.2f, 46.0f, 4.2f)));
        return builder.registerWith(database_);
    }

    void rememberEntity(InstanceHandle handle, const Entity& entity)
    {
        if (entities_.size() <= handle.id)
            entities_.resize(size_t(handle.id) + 1);
        entities_[handle.id] = entity;
    }

    void rememberStreamingRepresentatives(
        SubtreeInstanceHandle mounted, const Entity& entity)
    {
        if (!mounted.valid())
            return;
        const auto mountedNode = [mounted](uint32_t packedIndex)
        {
            return NodeHandle{mounted.slot, packedIndex,
                              mounted.generation};
        };
        const auto remember = [this, &mountedNode](
                                  size_t slot, uint32_t packedIndex,
                                  Payload expectedPayload,
                                  uint8_t payloadIndex = 0)
        {
            if (slot < virtualResources_.size())
            {
                const NodeHandle handle = mountedNode(packedIndex);
                if (database_.tryGetPayload(handle, payloadIndex) !=
                    UserPayload(expectedPayload))
                {
                    FRONTIER_ASSERT(
                        false,
                        "city streaming representative index mismatch");
                    return;
                }
                virtualResources_[slot].representative = handle;
                virtualResources_[slot].representativePayloadIndex =
                    payloadIndex;
            }
        };

        // These sample definitions are authored in traversal order, so their
        // public mounted handles plus packed indices provide stable readiness
        // handles even before a camera query has visited the representation.
        switch (entity.kind)
        {
        case EntityKind::House:
        {
            const size_t base = entity.houseStyle == HouseStyle::HouseA
                                    ? size_t(Payload::HouseCoarse)
                                    : kPayloadSlotCount;
            for (size_t detail = 0; detail < kHouseDetailResourceCount;
                 ++detail)
                remember(
                    base + detail, uint32_t(detail + 1),
                    static_cast<Payload>(size_t(Payload::HouseCoarse) +
                                         detail));
            break;
        }
        case EntityKind::Tower:
            if (entity.heroAsset < kHeroTowerAssetCount)
            {
                const size_t base =
                    kHeroTowerResourceBase +
                    size_t(entity.heroAsset) * kTowerDetailResourceCount;
                for (size_t detail = 0;
                     detail < kTowerDetailResourceCount; ++detail)
                {
                    const bool facadePayload =
                        detail < kTowerFacadePayloadCount;
                    const uint32_t packedIndex =
                        facadePayload
                            ? 1u
                            : uint32_t(2 + detail -
                                       kTowerFacadePayloadCount);
                    remember(
                        base + detail, packedIndex,
                        static_cast<Payload>(
                            size_t(Payload::TowerDistrict) + detail),
                        facadePayload ? uint8_t(detail) : uint8_t(0));
                }
            }
            break;
        case EntityKind::Tree:
            remember(size_t(Payload::TreeCoarse), 1,
                     Payload::TreeCoarse);
            remember(size_t(Payload::TreeTrunk), 2,
                     Payload::TreeTrunk);
            remember(size_t(Payload::TreeCrown), 3,
                     Payload::TreeCrown);
            break;
        case EntityKind::Car:
            remember(size_t(Payload::CarCoarse), 1,
                     Payload::CarCoarse);
            remember(size_t(Payload::CarBody), 2, Payload::CarBody);
            remember(size_t(Payload::CarCabin), 3,
                     Payload::CarCabin);
            break;
        case EntityKind::Pedestrian:
            remember(size_t(Payload::PedestrianCoarse), 1,
                     Payload::PedestrianCoarse);
            remember(size_t(Payload::PedestrianBody), 2,
                     Payload::PedestrianBody);
            remember(size_t(Payload::PedestrianHead), 3,
                     Payload::PedestrianHead);
            break;
        }
    }

    InstanceHandle instantiateActor(Payload fallback, float error,
                                    const AABB& localBounds,
                                    const Entity& entity,
                                    const SceneDefinition& definition)
    {
        Entity placed = entity;
        placed.localPosition = entity.position;
        placed.localYaw = entity.yaw;
        const NodeDesc root = node(
            fallback, error, localBounds,
            NodeDesc::FlagMountable | NodeDesc::FlagYawInvariantBounds);
        const InstanceDesc placement{
            .pos = placed.position,
            .scale = placed.scale,
            .yaw = yawRotation(placed.yaw),
        };
        const InstanceHandle handle = database_.instantiate(root, placement);
        const SubtreeInstanceHandle mounted =
            database_.mountSubtree(handle.rootNode(), definition.handle);
        placed.authoredHlodNodeCount =
            1 + (mounted.valid() ? definition.authoredNodeCount : 0);
        rememberStreamingRepresentatives(mounted, placed);
        rememberEntity(handle, placed);
        authoredHlodNodeCount_ += placed.authoredHlodNodeCount;
        return handle;
    }

    Entity makeHouse(HouseStyle style, float4 position,
                     uint32_t& random) const
    {
        Entity house;
        house.kind = EntityKind::House;
        house.houseStyle = style;
        house.position = position;
        if (style == HouseStyle::HouseA)
        {
            house.scale = 0.64f + random01(random) * 0.12f;
            house.yaw = random01(random) > 0.5f ? 0.0f : kPi * 0.5f;
            house.color = abgr(
                uint8_t(145 + random01(random) * 80),
                uint8_t(120 + random01(random) * 85),
                uint8_t(95 + random01(random) * 95));
        }
        else
        {
            house.scale = 0.62f + random01(random) * 0.16f;
            house.yaw = float(uint32_t(random01(random) * 4.0f)) *
                        kPi * 0.5f;
            house.color = abgr(
                uint8_t(105 + random01(random) * 95),
                uint8_t(135 + random01(random) * 85),
                uint8_t(145 + random01(random) * 90));
        }
        return house;
    }

    void createHouseAt(HouseStyle style, float4 position, uint32_t& random)
    {
        const Entity house = makeHouse(style, position, random);
        const size_t definition = static_cast<size_t>(style);
        houseHandles_.push_back(instantiateActor(
            Payload::HouseTop,
            authoredGeometricError(Payload::HouseTop), houseBounds(style), house,
            houseDefinitions_[definition]));
    }

    void resetWholeSceneMotionGroup()
    {
        wholeSceneHandles_.clear();
        wholeSceneHandles_.reserve(
            houseHandles_.size() + towerHandles_.size() +
            treeHandles_.size() + carHandles_.size() +
            pedestrianHandles_.size());
        const auto append = [this](const auto& handles)
        {
            wholeSceneHandles_.insert(wholeSceneHandles_.end(),
                                      handles.begin(), handles.end());
        };
        append(houseHandles_);
        append(towerHandles_);
        append(treeHandles_);
        append(carHandles_);
        append(pedestrianHandles_);
        wholeScenePositions_.resize(wholeSceneHandles_.size());
        wholeSceneYaws_.resize(wholeSceneHandles_.size());
        wholeSceneMotion_.reset(wholeSceneHandles_);
    }

    void replaceHouses(HouseStyle style)
    {
        unloadHouseResources(activeHouseStyle_);
        std::vector<float4> lots;
        lots.reserve(houseHandles_.size());
        for (InstanceHandle handle : houseHandles_)
        {
            if (handle.id < entities_.size())
            {
                lots.push_back(entities_[handle.id].localPosition);
                authoredHlodNodeCount_ -=
                    entities_[handle.id].authoredHlodNodeCount;
            }
            database_.removeInstance(handle);
        }

        houseHandles_.clear();
        houseHandles_.reserve(lots.size());
        activeHouseStyle_ = style;
        ++houseGeneration_;
        uint32_t random = 0x5eed1234u ^
                          (houseGeneration_ * 0x9e3779b9u) ^
                          (style == HouseStyle::HouseA
                               ? 0x13579bdfu
                               : 0x2468ace0u);
        for (float4 position : lots)
            createHouseAt(style, position, random);
        houseCount_ = uint32_t(houseHandles_.size());
        resetWholeSceneMotionGroup();
        if (animateWholeScene_)
            updateWholeSceneWave(simulationTime_,
                                 kWorstCaseWaveAmplitude);
        query_.reset();
        streamingLookaheadQuery_.reset();
    }

    void createScene()
    {
        houseDefinitions_[static_cast<size_t>(HouseStyle::HouseA)] =
            createHouseDefinition(HouseStyle::HouseA);
        houseDefinitions_[static_cast<size_t>(HouseStyle::HouseB)] =
            createHouseDefinition(HouseStyle::HouseB);
        const SceneDefinition carDefinition = createCarDefinition();
        const SceneDefinition pedestrianDefinition =
            createPedestrianDefinition();
        const SceneDefinition treeDefinition = createTreeDefinition();
        std::array<SceneDefinition, kHeroTowerAssetCount> towerDefinitions{};
        for (SceneDefinition& definition : towerDefinitions)
            definition = createTowerDefinition();
        uint32_t random = 0x5eed1234u;
        size_t towerIndex = 0;

        const AABB treeBounds =
            bounds(-1.9f, 0.0f, -1.9f, 1.9f, 6.7f, 1.9f);
        const AABB towerBounds =
            bounds(-kTowerRootHalfExtent, 0.0f, -kTowerRootHalfExtent,
                   kTowerRootHalfExtent, kTowerRootHeight,
                   kTowerRootHalfExtent);
        const std::array<float, 2> offsets{-3.35f, 3.35f};
        const auto isTowerBlock = [](int x, int z)
        {
            const int localX = x % kDistrictBlockCount;
            const int localZ = z % kDistrictBlockCount;
            return (localX == 3 && localZ == 3) ||
                   (localX == 4 && localZ == 3) ||
                   (localX == 3 && localZ == 4) ||
                   (localX == 4 && localZ == 4) ||
                   (localX == 2 && localZ == 3) ||
                   (localX == 5 && localZ == 4);
        };
        for (int z = 0; z < kBlockCount; ++z)
        {
            for (int x = 0; x < kBlockCount; ++x)
            {
                const float centerX =
                    (float(x) - (float(kBlockCount) - 1.0f) * 0.5f) *
                    kBlockSpacing;
                const float centerZ =
                    (float(z) - (float(kBlockCount) - 1.0f) * 0.5f) *
                    kBlockSpacing;
                if (isTowerBlock(x, z))
                {
                    const uint16_t heroAsset = uint16_t(
                        towerIndex % kHeroTowerAssetCount);
                    Entity tower;
                    tower.kind = EntityKind::Tower;
                    tower.heroAsset = heroAsset;
                    tower.position = float4::point(centerX, 0.0f, centerZ);
                    tower.scale = 0.82f + random01(random) * 0.28f;
                    tower.yaw = float((x + z) & 1) * kPi * 0.5f;
                    uint32_t heroRandom =
                        0x71f15eadu ^
                        (uint32_t(heroAsset) * 0x9e3779b9u);
                    tower.color = abgr(
                        uint8_t(105 + random01(heroRandom) * 55),
                        uint8_t(125 + random01(heroRandom) * 60),
                        uint8_t(145 + random01(heroRandom) * 65));
                    towerHandles_.push_back(instantiateActor(
                        Payload::TowerTop,
                        authoredGeometricError(Payload::TowerTop), towerBounds,
                        tower, towerDefinitions[heroAsset]));
                    ++towerIndex;
                    ++towerCount_;
                }
                else
                {
                    for (float offsetZ : offsets)
                    {
                        for (float offsetX : offsets)
                        {
                            createHouseAt(
                                HouseStyle::HouseA,
                                float4::point(centerX + offsetX, 0.0f,
                                              centerZ + offsetZ),
                                random);
                            ++houseCount_;
                        }
                    }
                }

                const std::array<float, 2> treeCorners{-1.0f, 1.0f};
                for (float cornerSign : treeCorners)
                {
                    Entity tree;
                    tree.kind = EntityKind::Tree;
                    const float treeCorner =
                        kSidewalkPathHalfExtent - kSidewalkCornerRadius;
                    tree.position = float4::point(
                        centerX + cornerSign * treeCorner, 0.0f,
                        centerZ + cornerSign * treeCorner);
                    tree.scale = 0.65f + random01(random) * 0.20f;
                    tree.yaw = random01(random) * kPi * 2.0f;
                    tree.color = abgr(
                        uint8_t(45 + random01(random) * 35),
                        uint8_t(115 + random01(random) * 80),
                        uint8_t(42 + random01(random) * 40));
                    treeHandles_.push_back(instantiateActor(
                        Payload::TreeTop,
                        authoredGeometricError(Payload::TreeTop), treeBounds,
                        tree, treeDefinition));
                    ++treeCount_;
                }
            }
        }
        FRONTIER_ASSERT(towerIndex == kTowerInstanceCount,
                        "unexpected hero skyscraper instance count");

        const AABB carBounds =
            bounds(-2.3f, 0.0f, -2.3f, 2.3f, 2.3f, 2.3f);
        constexpr int carsPerDistrict = 48;
        constexpr int carCount = carsPerDistrict * kDistrictCount;
        carHandles_.reserve(carCount);
        carPaths_.reserve(carCount);
        carPositions_.resize(carCount);
        carYaws_.resize(carCount);
        for (int index = 0; index < carCount; ++index)
        {
            CarPath path;
            const int district = index / carsPerDistrict;
            const int localCar = index % carsPerDistrict;
            const int districtX = district % kDistrictsPerAxis;
            const int districtZ = district / kDistrictsPerAxis;
            path.centerX =
                (float(districtX) -
                 (float(kDistrictsPerAxis) - 1.0f) * 0.5f) *
                kDistrictSpan;
            path.centerZ =
                (float(districtZ) -
                 (float(kDistrictsPerAxis) - 1.0f) * 0.5f) *
                kDistrictSpan;
            const int route = localCar % 4;
            path.halfX = route == 0 ? 24.0f
                                    : route == 1 ? 48.0f : 72.0f;
            path.halfZ = route == 0 ? 24.0f
                                    : route == 2 ? 48.0f : 72.0f;
            path.reverse = (localCar & 1) != 0;
            const float laneOffset = path.reverse ? -1.7f : 1.7f;
            path.halfX += laneOffset;
            path.halfZ += laneOffset;
            path.cornerRadius = 5.5f;
            path.phase = random01(random) * roundedLoopLength(path);
            path.speed = 8.0f + random01(random) * 8.0f;

            Entity car;
            car.kind = EntityKind::Car;
            sampleRoundedLoop(path, 0.0f, car.position, car.yaw);
            const uint8_t red = uint8_t(45 + random01(random) * 200);
            const uint8_t green = uint8_t(45 + random01(random) * 180);
            const uint8_t blue = uint8_t(45 + random01(random) * 200);
            car.color = abgr(red, green, blue);

            carHandles_.push_back(instantiateActor(
                Payload::CarTop,
                authoredGeometricError(Payload::CarTop), carBounds, car,
                carDefinition));
            carPaths_.push_back(path);
        }
        carMotion_.reset(carHandles_);

        const AABB pedestrianBounds =
            bounds(-1.15f, 0.0f, -1.15f, 1.15f, 2.3f, 1.15f);
        constexpr int pedestriansPerDistrict = 96;
        constexpr int pedestrianCount =
            pedestriansPerDistrict * kDistrictCount;
        constexpr int totalBlockCount = kBlockCount * kBlockCount;
        pedestrianHandles_.reserve(pedestrianCount);
        pedestrianPaths_.reserve(pedestrianCount);
        pedestrianPositions_.resize(pedestrianCount);
        pedestrianYaws_.resize(pedestrianCount);
        for (int index = 0; index < pedestrianCount; ++index)
        {
            const int block = index * totalBlockCount / pedestrianCount;
            const int blockX = block % kBlockCount;
            const int blockZ = block / kBlockCount;
            PedestrianPath path;
            path.centerX =
                (float(blockX) - (float(kBlockCount) - 1.0f) * 0.5f) *
                kBlockSpacing;
            path.centerZ =
                (float(blockZ) - (float(kBlockCount) - 1.0f) * 0.5f) *
                kBlockSpacing;
            path.phase = random01(random) * roundedLoopLength(
                kSidewalkPathHalfExtent, kSidewalkPathHalfExtent,
                kSidewalkCornerRadius);
            path.speed = 0.9f + random01(random) * 1.4f;
            path.reverse = random01(random) > 0.5f;

            Entity pedestrian;
            pedestrian.kind = EntityKind::Pedestrian;
            sampleSidewalkLoop(path, 0.0f, pedestrian.position,
                               pedestrian.yaw);
            pedestrian.scale = 0.88f + random01(random) * 0.22f;
            pedestrian.color = abgr(
                uint8_t(70 + random01(random) * 170),
                uint8_t(70 + random01(random) * 170),
                uint8_t(70 + random01(random) * 170));

            pedestrianHandles_.push_back(instantiateActor(
                Payload::PedestrianTop,
                authoredGeometricError(Payload::PedestrianTop), pedestrianBounds,
                pedestrian, pedestrianDefinition));
            pedestrianPaths_.push_back(path);
        }
        pedestrianMotion_.reset(pedestrianHandles_);

        resetWholeSceneMotionGroup();

        database_.applyUpdates(0);
        database_.optimize(OptimizationMode::TopologyAndLayout);
    }

    void updateMovingActorSources(float time)
    {
        for (size_t index = 0; index < carPaths_.size(); ++index)
        {
            const CarPath& path = carPaths_[index];
            float4 position;
            float yaw;
            sampleRoundedLoop(path, time, position, yaw);
            Entity& entity = entities_[carHandles_[index].id];
            entity.localPosition = position;
            entity.localYaw = yaw;
        }

        for (size_t index = 0; index < pedestrianPaths_.size(); ++index)
        {
            float4 position;
            float yaw;
            sampleSidewalkLoop(pedestrianPaths_[index], time, position, yaw);
            Entity& entity = entities_[pedestrianHandles_[index].id];
            entity.localPosition = position;
            entity.localYaw = yaw;
        }
    }

    void updateEntityPose(Entity& entity, float4 position, float yaw)
    {
        const bool changed = entity.position.x != position.x ||
                             entity.position.y != position.y ||
                             entity.position.z != position.z || entity.yaw != yaw;
        if (changed && entity.lastMotionFrame != motionFrame_)
        {
            entity.lastMotionFrame = motionFrame_;
            updatedHlodNodes_ += entity.authoredHlodNodeCount;
            ++movedInstances_;
        }
        entity.position = position;
        entity.yaw = yaw;
    }

    void updateActors(float time)
    {
        updateMovingActorSources(time);
        for (size_t index = 0; index < carHandles_.size(); ++index)
        {
            Entity& entity = entities_[carHandles_[index].id];
            updateEntityPose(entity, entity.localPosition, entity.localYaw);
            carPositions_[index] = entity.position;
            carYaws_[index] = yawRotation(entity.yaw);
        }
        database_.moveRigidInstances(carMotion_, carPositions_, carYaws_);

        for (size_t index = 0; index < pedestrianHandles_.size(); ++index)
        {
            Entity& entity = entities_[pedestrianHandles_[index].id];
            updateEntityPose(entity, entity.localPosition, entity.localYaw);
            pedestrianPositions_[index] = entity.position;
            pedestrianYaws_[index] = yawRotation(entity.yaw);
        }
        database_.moveRigidInstances(pedestrianMotion_, pedestrianPositions_,
                                     pedestrianYaws_);
    }

    void updateWholeSceneWave(float time, float amplitude)
    {
        updateMovingActorSources(time);
        for (size_t index = 0; index < wholeSceneHandles_.size(); ++index)
        {
            const InstanceHandle handle = wholeSceneHandles_[index];
            Entity& entity = entities_[handle.id];
            const float phase = entity.localPosition.x * 0.045f +
                                entity.localPosition.z * 0.037f +
                                float(handle.id % 251u) * 0.017f;
            float4 position = entity.localPosition;
            position.y += amplitude * std::cos(
                time * kWorstCaseWaveFrequency + phase);
            updateEntityPose(entity, position, entity.localYaw);
            wholeScenePositions_[index] = entity.position;
            wholeSceneYaws_[index] = yawRotation(entity.yaw);
        }
        database_.moveRigidInstances(wholeSceneMotion_,
                                     wholeScenePositions_,
                                     wholeSceneYaws_);
    }

    void updateAutomaticCamera(float time, float4& position,
                               float4& target) const
    {
        const float angle = time * 0.028f;
        const float radius =
            kCityHalfExtent * 1.22f +
            std::sin(time * 0.043f) * kCityHalfExtent * 0.18f;
        position = float4::point(
            std::cos(angle) * radius,
            kCityHalfExtent * 0.44f +
                std::sin(time * 0.061f) * kCityHalfExtent * 0.16f,
            std::sin(angle) * radius);
        const float targetTravel = kDistrictSpan * 0.38f;
        target = float4::point(std::sin(time * 0.035f) * targetTravel,
                               10.0f,
                               std::cos(time * 0.031f) * targetTravel);
    }

    void updateHeroPressureOrbitCamera(float time, float4& position,
                                       float4& target) const
    {
        // The automated regression approaches the focal tower before
        // orbiting it, which makes any temporary LOD regression directly
        // observable. The interactive scenario retains the close orbit.
        float radius = kHeroOrbitRadius;
        float orbitTime = time;
        if (streamingSelfTest_)
        {
            const float approach = std::clamp(
                time / kHeroApproachSeconds, 0.0f, 1.0f);
            radius = kHeroApproachStartRadius +
                     (kHeroOrbitRadius - kHeroApproachStartRadius) *
                         approach;
            orbitTime = std::max(time - kHeroApproachSeconds, 0.0f);
        }
        const float angle =
            -kPi * 0.5f + orbitTime * kHeroOrbitAngularSpeed;
        position = float4::point(
            kHeroOrbitCenterX + std::cos(angle) * radius,
            kHeroOrbitCameraHeight,
            kHeroOrbitCenterZ + std::sin(angle) * radius);
        target = float4::point(kHeroOrbitCenterX, kHeroOrbitTargetHeight,
                               kHeroOrbitCenterZ);
    }

    void drawWorld(DebugDrawEncoder& encoder)
    {
        float worldTransform[16];
        bx::mtxIdentity(worldTransform);
        encoder.setTransform(worldTransform);
        encoder.setWireframe(wireframeDebug_);
        encoder.setColor(abgr(86, 130, 78));
        encoder.draw(box(-kCityHalfExtent - 18.0f, -0.35f,
                         -kCityHalfExtent - 18.0f,
                         kCityHalfExtent + 18.0f, -0.10f,
                         kCityHalfExtent + 18.0f));

        encoder.setColor(abgr(54, 59, 66));
        for (int road = 0; road <= kBlockCount; ++road)
        {
            const float coordinate =
                (float(road) - float(kBlockCount) * 0.5f) * kBlockSpacing;
            encoder.draw(box(-kCityHalfExtent - 12.0f, -0.09f,
                             coordinate - kRoadHalfWidth,
                             kCityHalfExtent + 12.0f, 0.0f,
                             coordinate + kRoadHalfWidth));
            encoder.draw(box(coordinate - kRoadHalfWidth, -0.08f,
                             -kCityHalfExtent - 12.0f,
                             coordinate + kRoadHalfWidth, 0.01f,
                             kCityHalfExtent + 12.0f));
        }

        // Each lot gets a raised continuous sidewalk inside the curb. The
        // pedestrian centerline runs through the middle of these four slabs.
        encoder.setColor(abgr(174, 178, 181));
        for (int blockZ = 0; blockZ < kBlockCount; ++blockZ)
        {
            for (int blockX = 0; blockX < kBlockCount; ++blockX)
            {
                const float centerX =
                    (float(blockX) - (float(kBlockCount) - 1.0f) * 0.5f) *
                    kBlockSpacing;
                const float centerZ =
                    (float(blockZ) - (float(kBlockCount) - 1.0f) * 0.5f) *
                    kBlockSpacing;
                encoder.draw(box(
                    centerX - kSidewalkOuterExtent, 0.01f,
                    centerZ - kSidewalkOuterExtent,
                    centerX + kSidewalkOuterExtent, 0.13f,
                    centerZ - kSidewalkInnerExtent));
                encoder.draw(box(
                    centerX - kSidewalkOuterExtent, 0.01f,
                    centerZ + kSidewalkInnerExtent,
                    centerX + kSidewalkOuterExtent, 0.13f,
                    centerZ + kSidewalkOuterExtent));
                encoder.draw(box(
                    centerX - kSidewalkOuterExtent, 0.01f,
                    centerZ - kSidewalkInnerExtent,
                    centerX - kSidewalkInnerExtent, 0.13f,
                    centerZ + kSidewalkInnerExtent));
                encoder.draw(box(
                    centerX + kSidewalkInnerExtent, 0.01f,
                    centerZ - kSidewalkInnerExtent,
                    centerX + kSidewalkOuterExtent, 0.13f,
                    centerZ + kSidewalkInnerExtent));
            }
        }

        encoder.setColor(abgr(205, 188, 86));
        encoder.setWireframe(true);
        encoder.drawGrid(Axis::Y, bx::Vec3{0.0f, 0.025f, 0.0f}, 14, 12.0f);
        encoder.setWireframe(wireframeDebug_);
    }

    void setEntityTransform(DebugDrawEncoder& encoder, const Entity& entity)
    {
        float transform[16];
        bx::mtxSRT(transform, entity.scale, entity.scale, entity.scale,
                   0.0f, entity.yaw, 0.0f,
                   entity.position.x, entity.position.y, entity.position.z);
        encoder.setTransform(transform);
    }

    uint32_t hierarchyTint(Payload payload) const
    {
        uint32_t depth = 0;
        uint32_t maxDepth = 2;
        switch (payload)
        {
        case Payload::HouseTop:
        case Payload::CarTop:
        case Payload::PedestrianTop:
        case Payload::TreeTop:
            depth = 0;
            break;
        case Payload::HouseCoarse:
        case Payload::CarCoarse:
        case Payload::PedestrianCoarse:
        case Payload::TreeCoarse:
            depth = 1;
            break;
        case Payload::HouseBody:
        case Payload::HouseRoof:
        case Payload::CarBody:
        case Payload::CarCabin:
        case Payload::PedestrianBody:
        case Payload::PedestrianHead:
        case Payload::TreeTrunk:
        case Payload::TreeCrown:
            depth = 2;
            break;
        case Payload::TowerTop:
            depth = 0;
            maxDepth = 5;
            break;
        case Payload::TowerDistrict:
            depth = 1;
            maxDepth = 5;
            break;
        case Payload::TowerCoarse:
            depth = 2;
            maxDepth = 5;
            break;
        case Payload::TowerMedium:
            depth = 3;
            maxDepth = 5;
            break;
        case Payload::TowerFine:
            depth = 4;
            maxDepth = 5;
            break;
        case Payload::TowerBase:
        case Payload::TowerShaft:
        case Payload::TowerCrown:
            depth = 5;
            maxDepth = 5;
            break;
        }

        const float normalized = float(depth) / float(maxDepth);
        if (normalized <= 0.5f)
        {
            const float t = normalized * 2.0f;
            return abgr(uint8_t(45.0f + 210.0f * t),
                        uint8_t(230.0f + 20.0f * t), 34);
        }
        const float t = (normalized - 0.5f) * 2.0f;
        return abgr(255, uint8_t(250.0f * (1.0f - t) + 38.0f * t),
                    uint8_t(34.0f * (1.0f - t) + 22.0f * t));
    }

    void drawPayload(DebugDrawEncoder& encoder, Payload payload,
                     InstanceId instance,
                     const Entity& entity)
    {
        setEntityTransform(encoder, entity);
        encoder.setWireframe(wireframeDebug_);
        const bool selected =
            sceneTreeSelectionMatches(payload, instance, entity);
        const uint32_t selectedTint = abgr(46, 224, 255);
        const auto paint = [&](uint32_t authoredColor)
        {
            encoder.setColor(selected
                                 ? selectedTint
                                 : hierarchyTint_ ? hierarchyTint(payload)
                                                  : authoredColor);
        };
        switch (payload)
        {
        case Payload::HouseTop:
        case Payload::HouseCoarse:
            paint(entity.color);
            if (entity.houseStyle == HouseStyle::HouseA)
                encoder.draw(box(-3.2f, 0.0f, -3.2f,
                                 3.2f, 7.4f, 3.2f));
            else
                encoder.draw(box(-3.5f, 0.0f, -2.9f,
                                 3.5f, 7.2f, 2.9f));
            break;
        case Payload::HouseBody:
            paint(entity.color);
            if (entity.houseStyle == HouseStyle::HouseA)
            {
                encoder.draw(box(-3.0f, 0.0f, -3.0f,
                                 3.0f, 5.5f, 3.0f));
                paint(abgr(74, 50, 35));
                encoder.draw(box(-0.7f, 0.0f, -3.08f,
                                 0.7f, 2.4f, -2.92f));
                paint(abgr(95, 183, 220));
                encoder.draw(box(-2.3f, 2.5f, -3.09f,
                                 -1.1f, 3.8f, -2.91f));
                encoder.draw(box(1.1f, 2.5f, -3.09f,
                                 2.3f, 3.8f, -2.91f));
            }
            else
            {
                encoder.draw(box(-3.4f, 0.0f, -2.8f,
                                 3.4f, 6.5f, 2.8f));
                paint(abgr(50, 58, 66));
                encoder.draw(box(-0.75f, 0.0f, -2.89f,
                                 0.75f, 2.55f, -2.71f));
                paint(abgr(105, 205, 225));
                encoder.draw(box(-2.75f, 2.0f, -2.90f,
                                 -1.0f, 5.35f, -2.70f));
                encoder.draw(box(1.0f, 2.0f, -2.90f,
                                 2.75f, 5.35f, -2.70f));
            }
            break;
        case Payload::HouseRoof:
            if (entity.houseStyle == HouseStyle::HouseA)
            {
                paint(abgr(132, 57, 48));
                encoder.draw(roofGeometry_);
            }
            else
            {
                paint(abgr(72, 76, 82));
                encoder.draw(box(-3.6f, 6.35f, -3.0f,
                                 3.6f, 6.75f, 3.0f));
                paint(abgr(118, 124, 130));
                encoder.draw(box(-1.15f, 6.70f, -1.0f,
                                 1.15f, 7.35f, 1.0f));
            }
            break;
        case Payload::CarTop:
        case Payload::CarCoarse:
            paint(entity.color);
            encoder.draw(box(-2.1f, 0.15f, -1.0f, 2.1f, 1.45f, 1.0f));
            break;
        case Payload::CarBody:
            paint(entity.color);
            encoder.draw(box(-2.1f, 0.28f, -1.0f, 2.1f, 1.05f, 1.0f));
            paint(abgr(30, 32, 36));
            encoder.draw(box(-1.35f, 0.08f, -1.08f, -0.65f, 0.55f, -0.88f));
            encoder.draw(box(0.65f, 0.08f, -1.08f, 1.35f, 0.55f, -0.88f));
            encoder.draw(box(-1.35f, 0.08f, 0.88f, -0.65f, 0.55f, 1.08f));
            encoder.draw(box(0.65f, 0.08f, 0.88f, 1.35f, 0.55f, 1.08f));
            break;
        case Payload::CarCabin:
            paint(entity.color);
            encoder.draw(box(-0.9f, 1.0f, -0.83f, 0.95f, 1.72f, 0.83f));
            paint(abgr(104, 174, 205));
            encoder.draw(box(-0.72f, 1.12f, -0.86f, 0.72f, 1.58f, -0.80f));
            encoder.draw(box(-0.72f, 1.12f, 0.80f, 0.72f, 1.58f, 0.86f));
            break;
        case Payload::PedestrianTop:
        case Payload::PedestrianCoarse:
            paint(entity.color);
            encoder.draw(box(-0.34f, 0.05f, -0.34f, 0.34f, 2.1f, 0.34f));
            paint(abgr(226, 178, 139));
            encoder.drawCone(bx::Vec3{0.28f, 1.82f, 0.0f},
                             bx::Vec3{0.58f, 1.82f, 0.0f}, 0.11f);
            break;
        case Payload::PedestrianBody:
            paint(entity.color);
            encoder.drawCylinder(bx::Vec3{0.0f, 0.1f, 0.0f},
                                 bx::Vec3{0.0f, 1.65f, 0.0f}, 0.27f);
            break;
        case Payload::PedestrianHead:
            paint(abgr(226, 178, 139));
            encoder.draw(bx::Sphere{{0.0f, 1.92f, 0.0f}, 0.27f});
            encoder.drawCone(bx::Vec3{0.22f, 1.92f, 0.0f},
                             bx::Vec3{0.52f, 1.92f, 0.0f}, 0.10f);
            break;
        case Payload::TreeTop:
            paint(entity.color);
            encoder.draw(box(-1.6f, 0.0f, -1.6f, 1.6f, 6.2f, 1.6f));
            break;
        case Payload::TreeCoarse:
            paint(entity.color);
            encoder.drawCone(bx::Vec3{0.0f, 2.0f, 0.0f},
                             bx::Vec3{0.0f, 6.3f, 0.0f}, 1.75f);
            break;
        case Payload::TreeTrunk:
            paint(abgr(104, 70, 42));
            encoder.drawCylinder(bx::Vec3{0.0f, 0.0f, 0.0f},
                                 bx::Vec3{0.0f, 3.5f, 0.0f}, 0.30f);
            break;
        case Payload::TreeCrown:
            paint(entity.color);
            encoder.draw(bx::Sphere{{0.0f, 4.15f, 0.0f}, 1.70f});
            encoder.draw(bx::Sphere{{0.0f, 5.35f, 0.0f}, 1.30f});
            break;
        case Payload::TowerTop:
            paint(entity.color);
            encoder.draw(box(-5.0f, 0.0f, -5.0f, 5.0f, 44.0f, 5.0f));
            break;
        case Payload::TowerDistrict:
            paint(entity.color);
            encoder.draw(box(-4.8f, 0.0f, -4.8f, 4.8f, 42.0f, 4.8f));
            encoder.draw(box(-3.0f, 42.0f, -3.0f, 3.0f, 45.0f, 3.0f));
            break;
        case Payload::TowerCoarse:
            paint(entity.color);
            encoder.draw(box(-5.0f, 0.0f, -5.0f, 5.0f, 7.0f, 5.0f));
            encoder.draw(box(-4.1f, 7.0f, -4.1f, 4.1f, 39.0f, 4.1f));
            encoder.draw(box(-3.0f, 39.0f, -3.0f, 3.0f, 45.0f, 3.0f));
            break;
        case Payload::TowerMedium:
            paint(entity.color);
            encoder.draw(box(-5.0f, 0.0f, -5.0f, 5.0f, 7.0f, 5.0f));
            encoder.draw(box(-4.2f, 7.0f, -4.2f, 4.2f, 24.0f, 4.2f));
            encoder.draw(box(-3.7f, 24.0f, -3.7f, 3.7f, 38.0f, 3.7f));
            encoder.draw(box(-2.8f, 38.0f, -2.8f, 2.8f, 45.0f, 2.8f));
            break;
        case Payload::TowerFine:
            paint(entity.color);
            encoder.draw(box(-5.0f, 0.0f, -5.0f, 5.0f, 7.0f, 5.0f));
            encoder.draw(box(-4.2f, 7.0f, -4.2f, 4.2f, 22.0f, 4.2f));
            encoder.draw(box(-3.8f, 22.0f, -3.8f, 3.8f, 38.0f, 3.8f));
            encoder.draw(box(-2.8f, 38.0f, -2.8f, 2.8f, 45.0f, 2.8f));
            break;
        case Payload::TowerBase:
            paint(entity.color);
            encoder.draw(box(-5.0f, 0.0f, -5.0f, 5.0f, 7.0f, 5.0f));
            paint(abgr(72, 112, 140));
            encoder.draw(box(-3.8f, 1.2f, -5.08f, 3.8f, 5.6f, -4.92f));
            break;
        case Payload::TowerShaft:
            paint(entity.color);
            encoder.draw(box(-4.2f, 6.8f, -4.2f, 4.2f, 38.0f, 4.2f));
            paint(abgr(92, 174, 210));
            for (float floor = 9.0f; floor < 37.0f; floor += 3.0f)
            {
                encoder.draw(box(-3.7f, floor, -4.27f,
                                 3.7f, floor + 1.1f, -4.13f));
                encoder.draw(box(-3.7f, floor, 4.13f,
                                 3.7f, floor + 1.1f, 4.27f));
            }
            break;
        case Payload::TowerCrown:
            paint(entity.color);
            encoder.draw(box(-3.0f, 37.8f, -3.0f, 3.0f, 44.0f, 3.0f));
            encoder.drawCone(bx::Vec3{0.0f, 44.0f, 0.0f},
                             bx::Vec3{0.0f, 46.0f, 0.0f}, 1.5f);
            break;
        }
    }

    void drawFrozenCullFrustum(DebugDrawEncoder& encoder)
    {
        if (!freezeCullCamera_ || !frozenCull_.valid || !drawCullFrustum_)
            return;

        bx::Plane planes[6] = {
            bx::InitNone, bx::InitNone, bx::InitNone,
            bx::InitNone, bx::InitNone, bx::InitNone,
        };
        bx::buildFrustumPlanes(planes,
                               frozenCull_.pose.viewProjection.data(),
                               bgfx::getCaps()->homogeneousDepth);
        const bx::Vec3 points[8] = {
            bx::intersectPlanes(planes[0], planes[2], planes[4]),
            bx::intersectPlanes(planes[0], planes[3], planes[4]),
            bx::intersectPlanes(planes[0], planes[3], planes[5]),
            bx::intersectPlanes(planes[0], planes[2], planes[5]),
            bx::intersectPlanes(planes[1], planes[2], planes[4]),
            bx::intersectPlanes(planes[1], planes[3], planes[4]),
            bx::intersectPlanes(planes[1], planes[3], planes[5]),
            bx::intersectPlanes(planes[1], planes[2], planes[5]),
        };
        DdVertex vertices[8];
        for (size_t index = 0; index < std::size(points); ++index)
            vertices[index] = {points[index].x, points[index].y,
                               points[index].z};

        // Both windings make every translucent plane visible from inside and
        // outside the captured culling volume.
        static const uint16_t indices[] = {
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7,
            0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5,
            0, 1, 5, 0, 5, 4, 3, 7, 6, 3, 6, 2,
            0, 1, 2, 0, 2, 3, 4, 6, 5, 4, 7, 6,
            0, 7, 4, 0, 3, 7, 1, 6, 2, 1, 5, 6,
            0, 5, 1, 0, 4, 5, 3, 6, 7, 3, 2, 6,
        };

        float identity[16];
        bx::mtxIdentity(identity);
        encoder.push();
        encoder.setTransform(identity);
        encoder.setState(true, false, false);
        encoder.setWireframe(false);
        encoder.setColor(abgr(230, 50, 205, 34));
        encoder.drawTriList(uint32_t(std::size(vertices)), vertices,
                            uint32_t(std::size(indices)), indices);
        encoder.setState(false, false, false);
        encoder.setWireframe(true);
        encoder.setColor(abgr(255, 80, 220));
        encoder.drawFrustum(frozenCull_.pose.viewProjection.data());
        encoder.drawOrb(frozenCull_.pose.position.x,
                        frozenCull_.pose.position.y,
                        frozenCull_.pose.position.z, 1.5f);
        encoder.pop();
    }

#ifdef FRONTIER_DEBUG_TOOLS
    void drawSpatialDebugBounds(DebugDrawEncoder& encoder)
    {
        if (!drawTlasAabbs_ && !drawLooseBounds_)
        {
            lastTlasBoxesTotal_ = lastTlasBoxesDrawn_ = 0;
            lastLooseBoundsTotal_ = lastLooseBoundsDrawn_ = 0;
            return;
        }

        float identity[16];
        bx::mtxIdentity(identity);
        encoder.push();
        encoder.setTransform(identity);
        encoder.setState(!debugBoundsXray_, false, false);
        encoder.setWireframe(true);

        if (drawTlasAabbs_)
        {
            tlasDebugBoxes_.resize(size_t(tlasDebugBoxLimit_));
            lastTlasBoxesTotal_ = database_.debugTlasBoxes(
                uint32_t(tlasDebugDepth_), tlasDebugBoxes_);
            lastTlasBoxesDrawn_ =
                std::min(lastTlasBoxesTotal_, tlasDebugBoxes_.size());
            for (size_t index = 0; index < lastTlasBoxesDrawn_; ++index)
            {
                const TlasDebugBox& item = tlasDebugBoxes_[index];
                switch (item.kind)
                {
                case TlasDebugBoxKind::Root:
                    encoder.setColor(abgr(230, 70, 255));
                    break;
                case TlasDebugBoxKind::Internal:
                    encoder.setColor(abgr(50, 205, 255));
                    break;
                case TlasDebugBoxKind::Instance:
                    encoder.setColor(item.loose ? abgr(255, 85, 30)
                                                : abgr(255, 205, 45));
                    break;
                }
                encoder.draw(debugBox(item.bounds));
            }
        }
        else
        {
            lastTlasBoxesTotal_ = lastTlasBoxesDrawn_ = 0;
        }

        if (drawLooseBounds_)
        {
            looseDebugBounds_.resize(size_t(looseBoundsDrawLimit_));
            lastLooseBoundsTotal_ = database_.debugLooseInstanceBounds(
                looseDebugBounds_);
            lastLooseBoundsDrawn_ =
                std::min(lastLooseBoundsTotal_, looseDebugBounds_.size());
            for (size_t index = 0; index < lastLooseBoundsDrawn_; ++index)
            {
                const LooseInstanceDebugBounds& item =
                    looseDebugBounds_[index];
                encoder.setColor(abgr(255, 85, 25));
                encoder.draw(debugBox(item.envelope));
                encoder.setColor(abgr(45, 255, 85));
                encoder.draw(debugBox(item.exact));
            }
        }
        else
        {
            lastLooseBoundsTotal_ = lastLooseBoundsDrawn_ = 0;
        }
        encoder.pop();
    }
#endif

    void render(const FrontierResultView& frontier)
    {
        DebugDrawEncoder encoder;
        encoder.begin(kMainView);
        encoder.setLighting(!unlitSurfaces_);
        drawWorld(encoder);
        for (const FrontierEntry& entry : frontier)
        {
            const UserPayload rawPayload =
                database_.tryGetPayload(entry.nodeHandle,
                                        entry.payloadIndex());
            if (rawPayload == kInvalidPayload ||
                entry.instance() >= entities_.size())
                continue;
            drawPayload(encoder, Payload(rawPayload), entry.instance(),
                        entities_[entry.instance()]);
        }
        drawFrozenCullFrustum(encoder);
#ifdef FRONTIER_DEBUG_TOOLS
        drawSpatialDebugBounds(encoder);
#endif
        encoder.end();
    }

    size_t virtualResourceSlot(Payload payload, InstanceId instance) const
    {
        if (isTowerDetailPayload(payload))
        {
            if (instance < entities_.size() &&
                entities_[instance].kind == EntityKind::Tower &&
                entities_[instance].heroAsset < kHeroTowerAssetCount)
            {
                return kHeroTowerResourceBase +
                       size_t(entities_[instance].heroAsset) *
                           kTowerDetailResourceCount +
                       size_t(payload) - size_t(Payload::TowerDistrict);
            }
            return kStreamingResourceSlotCount;
        }
        if (isHouseDetailPayload(payload) &&
            instance < entities_.size() &&
            entities_[instance].kind == EntityKind::House &&
            entities_[instance].houseStyle == HouseStyle::HouseB)
        {
            return kPayloadSlotCount +
                   size_t(payload) - size_t(Payload::HouseCoarse);
        }
        return size_t(payload);
    }

    size_t towerDistrictResourceSlot(InstanceId instance) const
    {
        if (instance >= entities_.size() ||
            entities_[instance].kind != EntityKind::Tower ||
            entities_[instance].heroAsset >= kHeroTowerAssetCount)
            return kStreamingResourceSlotCount;
        return kHeroTowerResourceBase +
               size_t(entities_[instance].heroAsset) *
                   kTowerDetailResourceCount;
    }

    size_t streamingRollingFallbackSlot(size_t slot) const
    {
        if (slot >= virtualResources_.size())
            return kStreamingResourceSlotCount;
        const VirtualResource& resource = virtualResources_[slot];
        const Payload payload = resource.payload;
        if (isTowerDetailPayload(payload) &&
            resource.heroAsset < kHeroTowerAssetCount)
        {
            const size_t fallbackDetail =
                towerRollingFallbackDetailIndex(payload);
            if (fallbackDetail >= kTowerDetailResourceCount)
                return kStreamingResourceSlotCount;
            return kHeroTowerResourceBase +
                   size_t(resource.heroAsset) * kTowerDetailResourceCount +
                   fallbackDetail;
        }

        switch (payload)
        {
        case Payload::HouseBody:
        case Payload::HouseRoof:
            return resource.houseStyle == HouseStyle::HouseB
                       ? kPayloadSlotCount
                       : size_t(Payload::HouseCoarse);
        case Payload::CarBody:
        case Payload::CarCabin:
            return size_t(Payload::CarCoarse);
        case Payload::PedestrianBody:
        case Payload::PedestrianHead:
            return size_t(Payload::PedestrianCoarse);
        case Payload::TreeTrunk:
        case Payload::TreeCrown:
            return size_t(Payload::TreeCoarse);
        default:
            return kStreamingResourceSlotCount;
        }
    }

    void markStreamingRollingFallback(
        size_t slot,
        std::array<bool, kStreamingResourceSlotCount>& fallbackDemand) const
    {
        const size_t fallbackSlot = streamingRollingFallbackSlot(slot);
        if (fallbackSlot >= virtualResources_.size())
            return;
        // A fallback pin protects only a representation that is ready now.
        // The current cut itself is protected independently while its finer
        // replacement loads. Once that replacement becomes current, this pin
        // rolls forward to the just-replaced representation and older levels
        // become normal eviction candidates.
        if (virtualResources_[fallbackSlot].state ==
            StreamingResourceState::Resident)
            fallbackDemand[fallbackSlot] = true;
    }

    void validateStreamingRollingFallbacks(
        const std::array<bool, kStreamingResourceSlotCount>& currentDemand,
        const std::array<bool, kStreamingResourceSlotCount>& prefetchDemand,
        const std::array<bool, kStreamingResourceSlotCount>&
            fallbackDemand)
    {
        bool validate = heroPressureScenarioActive_;
#ifndef NDEBUG
        validate = true;
#endif
        if (!validate)
            return;

        bool valid = true;
        for (size_t fallbackSlot = 0;
             fallbackSlot < fallbackDemand.size(); ++fallbackSlot)
        {
            if (!fallbackDemand[fallbackSlot])
                continue;
            bool directlyRequired = false;
            for (size_t demandedSlot = 0;
                 demandedSlot < currentDemand.size(); ++demandedSlot)
            {
                if (!currentDemand[demandedSlot] &&
                    !prefetchDemand[demandedSlot])
                    continue;
                directlyRequired |=
                    streamingRollingFallbackSlot(demandedSlot) ==
                    fallbackSlot;
            }
            valid &= directlyRequired &&
                     virtualResources_[fallbackSlot].state ==
                         StreamingResourceState::Resident;
        }

        constexpr size_t fineDetail =
            size_t(Payload::TowerFine) -
            size_t(Payload::TowerDistrict);
        for (uint16_t hero = 0; hero < kHeroTowerAssetCount; ++hero)
        {
            const size_t heroBase =
                kHeroTowerResourceBase +
                size_t(hero) * kTowerDetailResourceCount;
            bool structuralDemand = false;
            for (size_t detail = fineDetail + 1;
                 detail < kTowerDetailResourceCount; ++detail)
            {
                structuralDemand |= currentDemand[heroBase + detail] ||
                                    prefetchDemand[heroBase + detail];
            }
            bool facadeDemand = false;
            for (size_t detail = 0; detail <= fineDetail; ++detail)
            {
                facadeDemand |= currentDemand[heroBase + detail] ||
                                prefetchDemand[heroBase + detail];
            }
            if (!structuralDemand || facadeDemand)
                continue;

            bool olderPinned = false;
            bool olderCommitted = false;
            for (size_t detail = 0; detail < fineDetail; ++detail)
            {
                const size_t slot = heroBase + detail;
                olderPinned |= fallbackDemand[slot];
                olderCommitted |=
                    virtualResources_[slot].state !=
                    StreamingResourceState::Unloaded;
            }
            const bool finePinned = fallbackDemand[heroBase + fineDetail];
            const bool heroValid = finePinned && !olderPinned;
            valid &= heroValid;
            if (heroPressureScenarioActive_)
            {
                ++heroPressureRollingPinObservations_;
                if (heroValid && !olderCommitted)
                    ++heroPressureRollingReleaseObservations_;
            }
        }

        if (!valid && heroPressureScenarioActive_)
            ++heroPressureRollingPinViolations_;
#ifndef NDEBUG
        FRONTIER_ASSERT(
            valid,
            "rolling fallback pins must contain only direct ready predecessors");
#endif
    }

    void virtualResourceName(const VirtualResource& resource, char* output,
                             size_t outputSize) const
    {
        const RepresentationInfo info = representationInfo(resource.payload);
        if (isHouseDetailPayload(resource.payload))
        {
            std::snprintf(output, outputSize, "%s (%s)", info.name,
                          resource.houseStyle == HouseStyle::HouseA
                              ? "House A"
                              : "House B");
        }
        else if (isTowerDetailPayload(resource.payload) &&
                 resource.heroAsset != UINT16_MAX)
        {
            std::snprintf(output, outputSize, "Hero %02u / %s",
                          uint32_t(resource.heroAsset) + 1, info.name);
        }
        else
        {
            std::snprintf(output, outputSize, "%s", info.name);
        }
    }

    size_t streamingResidencyGroup(
        size_t slot,
        std::array<size_t, kMaxStreamingResidencyGroupSize>& group) const
    {
        const VirtualResource& resource = virtualResources_[slot];
        if (resource.residencyGroupCount == 0)
        {
            group[0] = slot;
            return 1;
        }
        group = resource.residencyGroup;
        return resource.residencyGroupCount;
    }

    float streamingRoleScore(const StreamingErrorStats& errors,
                             float roleWeight) const
    {
        if (errors.count == 0)
            return 0.0f;
        const float weightedError =
            errors.average() * 0.65f + errors.maximum * 0.35f;
        const float normalizedError = std::max(
            weightedError / std::max(lodThreshold_, 0.01f), 0.05f);
        return roleWeight * float(errors.count) * normalizedError;
    }

    static StreamingErrorStats rebaseStreamingErrorCount(
        const StreamingErrorStats& source, uint32_t count)
    {
        if (source.count == 0 || count == 0)
            return {};
        StreamingErrorStats result = source;
        result.count = count;
        result.total = double(source.average()) * double(count);
        return result;
    }

    void finalizeVirtualResourceScores()
    {
        for (VirtualResource& resource : virtualResources_)
        {
            if (resource.byteSizeMiB <= 0.0f)
                continue;
            if (resource.pinned)
            {
                resource.importanceScore =
                    std::numeric_limits<float>::infinity();
                resource.scorePerMiB =
                    std::numeric_limits<float>::infinity();
                resource.decision = "keep: pinned coarsest fallback";
                continue;
            }

            resource.importanceScore =
                streamingRoleScore(resource.currentBenefitErrors, 4.0f) +
                streamingRoleScore(resource.transitionErrors, 3.0f) +
                streamingRoleScore(resource.prefetchErrors, 1.5f) +
                streamingRoleScore(resource.idealErrors, 1.0f);
            resource.scorePerMiB =
                resource.importanceScore /
                std::max(resource.byteSizeMiB, 0.001f);
            if (resource.state == StreamingResourceState::Loading)
                resource.decision = "keep: load already in flight";
            else if (resource.currentErrors.count != 0)
                resource.decision = "keep: used by current frontier";
            else if (resource.transitionErrors.count != 0)
                resource.decision = "candidate: immediate refinement";
            else if (resource.prefetchErrors.count != 0)
                resource.decision = "candidate: predicted camera demand";
            else if (resource.idealErrors.count != 0)
                resource.decision = "candidate: ideal endpoint";
            else if (resource.state == StreamingResourceState::Resident)
                resource.decision = "evictable: outside current plan";
            else
                resource.decision = "unloaded: outside current plan";
        }
    }

    void appendStreamingLog(const ImVec4& color, std::string message)
    {
        if (streamingLog_.size() == kStreamingLogCapacity)
            streamingLog_.erase(streamingLog_.begin());
        streamingLog_.push_back(
            StreamingLogEntry{streamingTime_, color, std::move(message)});
    }

    float virtualPinnedMiB() const
    {
        float result = 0.0f;
        for (const VirtualResource& resource : virtualResources_)
            if (resource.pinned)
                result += resource.byteSizeMiB;
        return result;
    }

    float virtualResidentMiB() const
    {
        float result = 0.0f;
        for (const VirtualResource& resource : virtualResources_)
            if (resource.state == StreamingResourceState::Resident)
                result += resource.byteSizeMiB;
        return result;
    }

    float virtualLoadingMiB() const
    {
        float result = 0.0f;
        for (const VirtualResource& resource : virtualResources_)
            if (resource.state == StreamingResourceState::Loading)
                result += resource.byteSizeMiB;
        return result;
    }

    void initializeVirtualStreaming()
    {
        for (size_t slot = 1; slot < kPayloadSlotCount; ++slot)
        {
            const Payload payload = static_cast<Payload>(slot);
            if (isTowerDetailPayload(payload))
                continue;
            const RepresentationInfo info = representationInfo(payload);
            VirtualResource& resource = virtualResources_[slot];
            resource.payload = payload;
            resource.houseStyle = HouseStyle::HouseA;
            resource.byteSizeMiB = info.byteSizeMiB;
            resource.pinned = info.coarsest;
            resource.state = info.coarsest
                                 ? StreamingResourceState::Resident
                                 : StreamingResourceState::Unloaded;
            resource.lastAction = info.coarsest ? "resident at startup"
                                                : "never loaded";
        }
        for (size_t index = 0; index < kHouseDetailResourceCount; ++index)
        {
            const Payload payload = static_cast<Payload>(
                size_t(Payload::HouseCoarse) + index);
            const RepresentationInfo info = representationInfo(payload);
            VirtualResource& resource =
                virtualResources_[kPayloadSlotCount + index];
            resource.payload = payload;
            resource.houseStyle = HouseStyle::HouseB;
            resource.byteSizeMiB = info.byteSizeMiB;
            resource.lastAction = "never loaded";
        }
        for (size_t hero = 0; hero < kHeroTowerAssetCount; ++hero)
        {
            for (size_t detail = 0; detail < kTowerDetailResourceCount;
                 ++detail)
            {
                const Payload payload = static_cast<Payload>(
                    size_t(Payload::TowerDistrict) + detail);
                const RepresentationInfo info = representationInfo(payload);
                VirtualResource& resource = virtualResources_[
                    kHeroTowerResourceBase +
                    hero * kTowerDetailResourceCount + detail];
                resource.payload = payload;
                resource.heroAsset = uint16_t(hero);
                resource.byteSizeMiB = info.byteSizeMiB;
                resource.lastAction = "never loaded";
            }
        }
        char message[160];
        std::snprintf(message, sizeof(message),
                      "Initialized %.3f MiB of pinned coarsest resources",
                      virtualResidentMiB());
        appendStreamingLog(ImVec4(0.50f, 0.85f, 1.0f, 1.0f), message);
        std::snprintf(
            message, sizeof(message),
            "%u independent hero skyscraper assets, at most %u instances each",
            uint32_t(kHeroTowerAssetCount),
            uint32_t(kHeroTowerInstancesPerAsset));
        appendStreamingLog(ImVec4(0.50f, 0.85f, 1.0f, 1.0f), message);
    }

    void unloadVirtualResource(size_t slot, const char* reason)
    {
        VirtualResource& resource = virtualResources_[slot];
        if (resource.pinned ||
            resource.state != StreamingResourceState::Resident)
            return;
        if (resource.representative.valid())
            database_.markPayloadUnavailable(
                resource.representative,
                resource.representativePayloadIndex);
        resource.state = StreamingResourceState::Unloaded;
        resource.residentBenefitErrors.reset();
        resource.decision = std::string("evicted: ") + reason;
        resource.lastAction = std::string("unloaded: ") + reason;
        ++streamingUnloads_;
        if (heroPressureScenarioActive_)
            heroPressureLastUnloadTime_[slot] = streamingTime_;

        char name[96];
        char message[240];
        virtualResourceName(resource, name, sizeof(name));
        std::snprintf(message, sizeof(message),
                      "UNLOAD %s (%.3f MiB, score %.1f/MiB, %s)", name,
                      resource.byteSizeMiB, resource.scorePerMiB, reason);
        appendStreamingLog(ImVec4(1.0f, 0.48f, 0.25f, 1.0f), message);
    }

    void cancelPendingStreamingGroup(size_t groupIndex, const char* reason)
    {
        const PendingStreamingGroup& group =
            pendingStreamingGroups_[groupIndex];
        for (size_t slot : group.resources)
        {
            VirtualResource& resource = virtualResources_[slot];
            if (resource.state == StreamingResourceState::Loading)
            {
                resource.state = StreamingResourceState::Unloaded;
                resource.residentBenefitErrors.reset();
                resource.decision =
                    std::string("canceled: ") + reason;
                resource.lastAction =
                    std::string("load canceled: ") + reason;
            }
        }
        char message[192];
        std::snprintf(message, sizeof(message),
                      "CANCEL load group #%llu (%.3f MiB, score %.1f/MiB, %s)",
                      static_cast<unsigned long long>(group.serial),
                      group.byteSizeMiB, group.scorePerMiB, reason);
        appendStreamingLog(ImVec4(0.75f, 0.62f, 0.35f, 1.0f), message);
        pendingStreamingGroups_.erase(
            pendingStreamingGroups_.begin() + groupIndex);
    }

    void makeAllVirtualResourcesResident(bool announce)
    {
        while (!pendingStreamingGroups_.empty())
            cancelPendingStreamingGroup(
                pendingStreamingGroups_.size() - 1,
                "virtual streaming disabled");

        uint32_t activeResources = 0;
        for (VirtualResource& resource : virtualResources_)
        {
            if (resource.byteSizeMiB <= 0.0f)
                continue;
            const bool activeHouseResource =
                !isHouseDetailPayload(resource.payload) ||
                resource.houseStyle == activeHouseStyle_;
            const bool active = resource.pinned || activeHouseResource;
            if (!active)
            {
                if (resource.representative.valid() &&
                    database_.isPayloadReady(
                        resource.representative,
                        resource.representativePayloadIndex))
                    database_.markPayloadUnavailable(
                        resource.representative,
                        resource.representativePayloadIndex);
                resource.state = StreamingResourceState::Unloaded;
                resource.residentBenefitErrors.reset();
                resource.decision = "inactive house style";
                resource.lastAction =
                    "unloaded: inactive house style";
                continue;
            }

            ++activeResources;
            if (resource.representative.valid() &&
                !database_.isPayloadReady(
                    resource.representative,
                    resource.representativePayloadIndex))
                database_.markPayloadReady(
                    resource.representative,
                    resource.representativePayloadIndex);
            resource.state = StreamingResourceState::Resident;
            resource.residentBenefitErrors.reset();
            resource.decision =
                "resident: virtual streaming disabled";
            resource.lastAction =
                "resident immediately: virtual streaming disabled";
        }

        query_.reset();
        streamingLookaheadQuery_.reset();
        streamingErrorMeasurementQuery_.reset();
        lastStreamingErrorImprovement_ = {};
        currentFrontierMemoryMiB_ = virtualResidentMiB();
        idealFrontierMemoryMiB_ = currentFrontierMemoryMiB_;
        protectedFallbackMemoryMiB_ = 0.0f;
        lastBudgetBlockedGroups_ = 0;
        lastCapacityBlockedGroups_ = 0;
        lastValueBlockedGroups_ = 0;
        minimumBlockedCommitMiB_ = 0.0f;
        if (announce)
        {
            char message[192];
            std::snprintf(
                message, sizeof(message),
                "VIRTUAL STREAMING disabled: %u active resources resident "
                "immediately (%.2f MiB)",
                activeResources, virtualResidentMiB());
            appendStreamingLog(ImVec4(0.35f, 0.72f, 1.0f, 1.0f),
                               message);
        }
    }

    void unloadHouseResources(HouseStyle style)
    {
        for (size_t group = pendingStreamingGroups_.size(); group-- > 0;)
        {
            bool belongsToStyle = false;
            for (size_t slot : pendingStreamingGroups_[group].resources)
            {
                const VirtualResource& resource = virtualResources_[slot];
                belongsToStyle |= isHouseDetailPayload(resource.payload) &&
                                  resource.houseStyle == style;
            }
            if (belongsToStyle)
                cancelPendingStreamingGroup(group, "house generation replaced");
        }
        for (size_t slot = 0; slot < virtualResources_.size(); ++slot)
        {
            const VirtualResource& resource = virtualResources_[slot];
            if (isHouseDetailPayload(resource.payload) &&
                resource.houseStyle == style)
                unloadVirtualResource(slot, "house generation replaced");
        }
    }

    void resetVirtualStreaming()
    {
        while (!pendingStreamingGroups_.empty())
            cancelPendingStreamingGroup(
                pendingStreamingGroups_.size() - 1, "reset to coarsest");
        for (size_t slot = 0; slot < virtualResources_.size(); ++slot)
            unloadVirtualResource(slot, "reset to coarsest");
        lastRefinementGroups_ = 0;
        lastRefinementEntries_ = 0;
        lastPrefetchGroupCount_ = 0;
        lastPrefetchEntryCount_ = 0;
        lastIdealEntryCount_ = 0;
        lastConvergedEntryCount_ = 0;
        currentFrontierMemoryMiB_ = virtualResidentMiB();
        idealFrontierMemoryMiB_ = virtualResidentMiB();
        lastBudgetBlockedGroups_ = 0;
        lastCapacityBlockedGroups_ = 0;
        lastValueBlockedGroups_ = 0;
        minimumBlockedCommitMiB_ = 0.0f;
        lastReclaimableResourceCount_ = 0;
        lastReclaimableResidentMiB_ = 0.0f;
        streamingPlanValid_ = false;
        streamingConvergenceHistory_.fill(0.0f);
        streamingConvergenceHistoryCursor_ = 0;
        streamingConvergenceHistoryCount_ = 0;
        streamingConvergenceHistoryElapsed_ = 0.0f;
        streamingConvergenceActive_ = false;
        lastStreamingConvergenceSeconds_ = 0.0f;
        query_.reset();
        streamingLookaheadQuery_.reset();
        streamingErrorMeasurementQuery_.reset();
        lastStreamingErrorImprovement_ = {};
        appendStreamingLog(ImVec4(0.50f, 0.85f, 1.0f, 1.0f),
                           "RESET complete: only coarsest resources resident");
    }

    uint32_t residentHeroFineCount() const
    {
        uint32_t result = 0;
        for (const VirtualResource& resource : virtualResources_)
            if (resource.payload == Payload::TowerFine &&
                resource.heroAsset != UINT16_MAX &&
                resource.state == StreamingResourceState::Resident)
                ++result;
        return result;
    }

    static int towerLodRank(Payload payload)
    {
        switch (payload)
        {
        case Payload::TowerTop: return 0;
        case Payload::TowerDistrict: return 1;
        case Payload::TowerCoarse: return 2;
        case Payload::TowerMedium: return 3;
        case Payload::TowerFine: return 4;
        case Payload::TowerBase:
        case Payload::TowerShaft:
        case Payload::TowerCrown: return 5;
        default: return -1;
        }
    }

    static const char* towerLodName(int rank)
    {
        constexpr const char* names[] = {
            "fallback", "district", "coarse", "medium", "fine", "detail",
        };
        return rank >= 0 && rank < int(std::size(names))
                   ? names[rank]
                   : "not visible";
    }

    void observeHeroPressureFrontier(const FrontierResultView& frontier)
    {
        if (!heroPressureScenarioActive_ ||
            heroPressureFocalInstance_ == kInvalidInstanceId)
            return;

        int focalRank = -1;
        float focalError = 0.0f;
        float focalGeometricError = 0.0f;
        uint32_t dominantFallbacks = 0;
        float worstDominantFallbackError = 0.0f;
        InstanceId worstDominantFallbackInstance = kInvalidInstanceId;
        for (const FrontierEntry& entry : frontier)
        {
            const UserPayload rawPayload =
                database_.tryGetPayload(entry.nodeHandle,
                                        entry.payloadIndex());
            if (rawPayload == kInvalidPayload ||
                rawPayload >= UserPayload(kPayloadSlotCount))
                continue;
            const Payload payload = Payload(rawPayload);
            const int rank = towerLodRank(payload);
            if (rank < 0)
                continue;
            const float error = entry.approximateError(lodThreshold_);
            if (entry.instance() == heroPressureFocalInstance_)
            {
                focalRank = std::max(focalRank, rank);
                focalError = std::max(focalError, error);
                focalGeometricError = std::max(
                    focalGeometricError,
                    authoredGeometricError(payload));
            }
            if (payload == Payload::TowerTop &&
                error >= kHeroOrbitDominantFallbackErrorPixels)
            {
                ++dominantFallbacks;
                if (error > worstDominantFallbackError)
                {
                    worstDominantFallbackError = error;
                    worstDominantFallbackInstance = entry.instance();
                }
            }
        }

        const float elapsed =
            streamingTime_ - heroPressureScenarioStartTime_;
        const bool approaching =
            streamingSelfTest_ &&
            heroPressureOrbitPhaseOffset_ + elapsed <=
                kHeroApproachSeconds + 0.001f;
        if (approaching && focalRank >= 0)
        {
            if (std::isfinite(heroPressureLastFocalGeometricError_) &&
                focalGeometricError >
                    heroPressureLastFocalGeometricError_ + 0.0001f)
            {
                ++heroPressureGeometricErrorRegressions_;
                std::printf(
                    "FRONTIER_STREAMING_SELF_TEST ERROR_REGRESSION "
                    "t=%.2f previous=%.2fm current=%.2fm lod=%s\n",
                    elapsed, heroPressureLastFocalGeometricError_,
                    focalGeometricError, towerLodName(focalRank));
                std::fflush(stdout);
            }
            heroPressureLastFocalGeometricError_ =
                focalGeometricError;
        }
        if (focalRank >= 0)
            ++heroPressureFocalObservedFrames_;
        if (elapsed >= kHeroOrbitWarmupSeconds && focalRank == 0)
        {
            ++heroPressureFocalFallbackFrames_;
            heroPressureWorstFallbackError_ = std::max(
                heroPressureWorstFallbackError_, focalError);
            if (heroPressureFirstFallbackTime_ < 0.0f)
                heroPressureFirstFallbackTime_ = elapsed;
        }
        if (elapsed >= kHeroOrbitWarmupSeconds && dominantFallbacks != 0)
        {
            ++heroPressureDominantFallbackFrames_;
            heroPressureMaxDominantFallbacks_ = std::max(
                heroPressureMaxDominantFallbacks_, dominantFallbacks);
            heroPressureWorstDominantFallbackError_ = std::max(
                heroPressureWorstDominantFallbackError_,
                worstDominantFallbackError);
            if (heroPressureFirstDominantFallbackTime_ < 0.0f)
                heroPressureFirstDominantFallbackTime_ = elapsed;
        }

        if (streamingSelfTest_ &&
            dominantFallbacks != heroPressureLastDominantFallbacks_ &&
            elapsed >= kHeroOrbitWarmupSeconds)
        {
            uint32_t hero = UINT32_MAX;
            if (worstDominantFallbackInstance < entities_.size())
                hero = entities_[worstDominantFallbackInstance].heroAsset;
            std::printf(
                "FRONTIER_STREAMING_SELF_TEST DOMINANT_FALLBACK t=%.2f "
                "count=%u worstError=%.2fpx instance=%u hero=%u\n",
                elapsed, dominantFallbacks, worstDominantFallbackError,
                worstDominantFallbackInstance,
                hero != UINT32_MAX ? hero + 1 : 0);
            if (hero != UINT32_MAX && hero < kHeroTowerAssetCount &&
                dominantFallbacks != 0)
            {
                const size_t heroBase =
                    kHeroTowerResourceBase +
                    size_t(hero) * kTowerDetailResourceCount;
                for (size_t detail = 0;
                     detail < kTowerDetailResourceCount; ++detail)
                {
                    const VirtualResource& resource =
                        virtualResources_[heroBase + detail];
                    char name[96];
                    virtualResourceName(resource, name, sizeof(name));
                    std::printf(
                        "FRONTIER_STREAMING_SELF_TEST HERO_STATE "
                        "resource=\"%s\" state=%s score=%.2f/MiB "
                        "current=%u transition=%u prefetch=%u ideal=%u "
                        "decision=\"%s\"\n",
                        name, streamingStateName(resource.state),
                        resource.scorePerMiB,
                        resource.currentErrors.count,
                        resource.transitionErrors.count,
                        resource.prefetchErrors.count,
                        resource.idealErrors.count,
                        resource.decision.c_str());
                }
            }
            std::fflush(stdout);
        }
        heroPressureLastDominantFallbacks_ = dominantFallbacks;

        if (focalRank != heroPressureLastFocalRank_)
        {
            if (streamingSelfTest_)
            {
                std::printf(
                    "FRONTIER_STREAMING_SELF_TEST FOCAL_LOD t=%.2f "
                    "lod=%s error=%.2fpx committed=%.2f/%.2fMiB\n",
                    elapsed, towerLodName(focalRank), focalError,
                    virtualResidentMiB() + virtualLoadingMiB(),
                    virtualMemoryBudgetMiB_);
                std::fflush(stdout);
            }
            heroPressureLastFocalRank_ = focalRank;
        }
    }

    void startHeroPressureScenario()
    {
        if (animateWholeScene_)
        {
            animateWholeScene_ = false;
            updateWholeSceneWave(simulationTime_, 0.0f);
        }
        restoreSceneAfterStress_ = false;
        freezeSimulation_ = false;
        virtualStreamingEnabled_ = true;
        streamingPaused_ = false;
        streamingCutStrategy_ = StreamingCutStrategy::QualityPerByte;
        virtualMemoryBudgetMiB_ = streamingSelfTest_
                                      ? streamingTestBudgetMiB_
                                      : kHeroPressureTestBudgetMiB;
        streamingLatencySeconds_ = 0.65f;
        streamingUnloadDelaySeconds_ = 2.0f;
        maxConcurrentStreamingLoads_ = 3;
        streamingLog_.clear();
        resetVirtualStreaming();

        heroPressureOrbitPhaseOffset_ =
            std::max(streamingTestCameraTime_, 0.0f);
        float4 position;
        float4 target;
        updateHeroPressureOrbitCamera(heroPressureOrbitPhaseOffset_, position,
                                      target);
        freeCamera_ = false;
        freezeCullCamera_ = false;
        frozenCull_.valid = false;

        heroPressureFocalInstance_ = kInvalidInstanceId;
        float closestFocalDistanceSquared =
            std::numeric_limits<float>::infinity();
        for (InstanceHandle handle : towerHandles_)
        {
            const Entity& tower = entities_[handle.id];
            const float dx = tower.position.x - kHeroOrbitCenterX;
            const float dz = tower.position.z - kHeroOrbitCenterZ;
            const float distanceSquared = dx * dx + dz * dz;
            if (distanceSquared < closestFocalDistanceSquared)
            {
                closestFocalDistanceSquared = distanceSquared;
                heroPressureFocalInstance_ = handle.id;
            }
        }

        for (size_t slot = 0; slot < virtualResources_.size(); ++slot)
        {
            heroPressureScenarioStates_[slot] =
                virtualResources_[slot].state;
            heroPressureScenarioResourceTransitions_[slot] = 0;
            heroPressureScenarioResourceRapidReloads_[slot] = 0;
            heroPressureLastUnloadTime_[slot] =
                -std::numeric_limits<float>::infinity();
        }
        heroPressureScenarioStartTime_ = streamingTime_;
        heroPressureScenarioTransitions_ = 0;
        heroPressureScenarioStartLoads_ = streamingLoadsCompleted_;
        heroPressureScenarioStartUnloads_ = streamingUnloads_;
        heroPressureScenarioStartDemotions_ = streamingQualityDemotions_;
        heroPressureScenarioStartCurrentCutInvalidations_ =
            streamingCurrentCutInvalidations_;
        heroPressureScenarioStartAtomicPublicationViolations_ =
            streamingAtomicPublicationViolations_;
        heroPressureScenarioStartAtomicGroupCompletions_ =
            streamingAtomicGroupCompletions_;
        heroPressureScenarioStartCapacityBlockedRequests_ =
            streamingCapacityBlockedRequests_;
        heroPressureScenarioStartErrorMeasurements_ =
            streamingErrorMeasurements_;
        heroPressureScenarioStartPositiveErrorMeasurements_ =
            streamingPositiveErrorMeasurements_;
        heroPressureScenarioStartErrorMeasurementRegressions_ =
            streamingErrorMeasurementRegressions_;
        heroPressureScenarioLoads_ = 0;
        heroPressureScenarioUnloads_ = 0;
        heroPressureScenarioDemotions_ = 0;
        heroPressureScenarioCurrentCutInvalidations_ = 0;
        heroPressureScenarioAtomicPublicationViolations_ = 0;
        heroPressureScenarioAtomicGroupCompletions_ = 0;
        heroPressureScenarioCapacityBlockedRequests_ = 0;
        heroPressureScenarioErrorMeasurements_ = 0;
        heroPressureScenarioPositiveErrorMeasurements_ = 0;
        heroPressureScenarioErrorMeasurementRegressions_ = 0;
        heroPressureScenarioElapsed_ = 0.0f;
        heroPressureScenarioFineHeroes_ = 0;
        heroPressureFocalObservedFrames_ = 0;
        heroPressureFocalFallbackFrames_ = 0;
        heroPressureWorstFallbackError_ = 0.0f;
        heroPressureFirstFallbackTime_ = -1.0f;
        heroPressureLastFocalGeometricError_ =
            std::numeric_limits<float>::infinity();
        heroPressureGeometricErrorRegressions_ = 0;
        heroPressureRollingPinObservations_ = 0;
        heroPressureRollingReleaseObservations_ = 0;
        heroPressureRollingPinViolations_ = 0;
        heroPressureLastFocalRank_ = -2;
        heroPressureDominantFallbackFrames_ = 0;
        heroPressureMaxDominantFallbacks_ = 0;
        heroPressureLastDominantFallbacks_ = 0;
        heroPressureMaxResourceTransitions_ = 0;
        heroPressureRapidReloads_ = 0;
        heroPressureMaxResourceRapidReloads_ = 0;
        heroPressureWorstDominantFallbackError_ = 0.0f;
        heroPressureFirstDominantFallbackTime_ = -1.0f;
        heroPressureScenarioActive_ = true;
        heroPressureScenarioFinished_ = false;
        heroPressureScenarioPassed_ = false;
        heroPressureScenarioWithinBudget_ = true;
        showVirtualStreaming_ = !streamingSelfTest_;
        char startMessage[192];
        std::snprintf(
            startMessage, sizeof(startMessage),
            "TEST start: close focal-skyscraper orbit with moving actors, "
            "%.2f MiB quality-per-byte budget",
            virtualMemoryBudgetMiB_);
        appendStreamingLog(ImVec4(0.50f, 0.85f, 1.0f, 1.0f), startMessage);
        if (streamingSelfTest_)
        {
            std::printf(
                "FRONTIER_STREAMING_SELF_TEST START budget=%.2fMiB "
                "actors=moving orbit=(%.1f,%.1f,%.1f)->"
                "(%.1f,%.1f,%.1f) focal_instance=%u\n",
                virtualMemoryBudgetMiB_,
                position.x, position.y, position.z,
                target.x, target.y, target.z,
                heroPressureFocalInstance_);
            std::fflush(stdout);
        }
    }

    void finishHeroPressureScenario(bool passed)
    {
        heroPressureScenarioActive_ = false;
        heroPressureScenarioFinished_ = true;
        heroPressureScenarioPassed_ = passed;
        heroPressureScenarioElapsed_ =
            streamingTime_ - heroPressureScenarioStartTime_;
        heroPressureScenarioLoads_ =
            streamingLoadsCompleted_ - heroPressureScenarioStartLoads_;
        heroPressureScenarioUnloads_ =
            streamingUnloads_ - heroPressureScenarioStartUnloads_;
        heroPressureScenarioDemotions_ =
            streamingQualityDemotions_ -
            heroPressureScenarioStartDemotions_;
        heroPressureScenarioCurrentCutInvalidations_ =
            streamingCurrentCutInvalidations_ -
            heroPressureScenarioStartCurrentCutInvalidations_;
        heroPressureScenarioAtomicPublicationViolations_ =
            streamingAtomicPublicationViolations_ -
            heroPressureScenarioStartAtomicPublicationViolations_;
        heroPressureScenarioAtomicGroupCompletions_ =
            streamingAtomicGroupCompletions_ -
            heroPressureScenarioStartAtomicGroupCompletions_;
        heroPressureScenarioCapacityBlockedRequests_ =
            streamingCapacityBlockedRequests_ -
            heroPressureScenarioStartCapacityBlockedRequests_;
        heroPressureScenarioErrorMeasurements_ =
            streamingErrorMeasurements_ -
            heroPressureScenarioStartErrorMeasurements_;
        heroPressureScenarioPositiveErrorMeasurements_ =
            streamingPositiveErrorMeasurements_ -
            heroPressureScenarioStartPositiveErrorMeasurements_;
        heroPressureScenarioErrorMeasurementRegressions_ =
            streamingErrorMeasurementRegressions_ -
            heroPressureScenarioStartErrorMeasurementRegressions_;
        heroPressureScenarioFineHeroes_ = residentHeroFineCount();

        char message[256];
        std::snprintf(
            message, sizeof(message),
            "TEST %s after %.1f s: %u transitions, %u fine heroes, "
            "%.2f/%.2f MiB, dominant fallback frames %u",
            passed ? "PASS" : "FAIL",
            heroPressureScenarioElapsed_, heroPressureScenarioTransitions_,
            heroPressureScenarioFineHeroes_,
            virtualResidentMiB() + virtualLoadingMiB(),
            virtualMemoryBudgetMiB_,
            heroPressureDominantFallbackFrames_);
        appendStreamingLog(
            passed ? ImVec4(0.35f, 0.85f, 0.45f, 1.0f)
                   : ImVec4(1.0f, 0.35f, 0.25f, 1.0f),
            message);
        std::printf(
            "FRONTIER_STREAMING_SELF_TEST %s elapsed=%.1fs transitions=%u "
            "loads=%llu unloads=%llu demotions=%llu fineHeroes=%u "
            "committed=%.2fMiB budget=%.2fMiB blocked=%u "
            "capacityRejected=%u valueRejected=%u focalFallbackFrames=%u "
            "firstFallback=%.2fs worstFallbackError=%.2fpx "
            "dominantFallbackFrames=%u maxDominantFallbacks=%u "
            "firstDominantFallback=%.2fs worstDominantError=%.2fpx "
            "cutInvalidations=%llu errorRegressions=%u "
            "atomicPublishViolations=%llu atomicGroups=%llu "
            "capacityBlocks=%llu "
            "rollingPinObservations=%u rollingReleases=%u "
            "rollingPinViolations=%u "
            "errorMeasurements=%llu positiveErrorMeasurements=%llu "
            "errorMeasurementRegressions=%llu "
            "rapidReloads=%u maxResourceRapidReloads=%u "
            "maxResourceTransitions=%u\n",
            passed ? "PASS" : "FAIL", heroPressureScenarioElapsed_,
            heroPressureScenarioTransitions_,
            static_cast<unsigned long long>(heroPressureScenarioLoads_),
            static_cast<unsigned long long>(heroPressureScenarioUnloads_),
            static_cast<unsigned long long>(heroPressureScenarioDemotions_),
            heroPressureScenarioFineHeroes_,
            virtualResidentMiB() + virtualLoadingMiB(),
            virtualMemoryBudgetMiB_, lastBudgetBlockedGroups_,
            lastCapacityBlockedGroups_, lastValueBlockedGroups_,
            heroPressureFocalFallbackFrames_,
            heroPressureFirstFallbackTime_,
            heroPressureWorstFallbackError_,
            heroPressureDominantFallbackFrames_,
            heroPressureMaxDominantFallbacks_,
            heroPressureFirstDominantFallbackTime_,
            heroPressureWorstDominantFallbackError_,
            static_cast<unsigned long long>(
                heroPressureScenarioCurrentCutInvalidations_),
            heroPressureGeometricErrorRegressions_,
            static_cast<unsigned long long>(
                heroPressureScenarioAtomicPublicationViolations_),
            static_cast<unsigned long long>(
                heroPressureScenarioAtomicGroupCompletions_),
            static_cast<unsigned long long>(
                heroPressureScenarioCapacityBlockedRequests_),
            heroPressureRollingPinObservations_,
            heroPressureRollingReleaseObservations_,
            heroPressureRollingPinViolations_,
            static_cast<unsigned long long>(
                heroPressureScenarioErrorMeasurements_),
            static_cast<unsigned long long>(
                heroPressureScenarioPositiveErrorMeasurements_),
            static_cast<unsigned long long>(
                heroPressureScenarioErrorMeasurementRegressions_),
            heroPressureRapidReloads_,
            heroPressureMaxResourceRapidReloads_,
            heroPressureMaxResourceTransitions_);
        for (size_t slot = 0; slot < virtualResources_.size(); ++slot)
        {
            const uint32_t transitions =
                heroPressureScenarioResourceTransitions_[slot];
            if (transitions < (passed ? 5u : 3u))
                continue;
            const VirtualResource& resource = virtualResources_[slot];
            char name[96];
            virtualResourceName(resource, name, sizeof(name));
            std::printf(
                "FRONTIER_STREAMING_SELF_TEST %s resource=\"%s\" "
                "transitions=%u rapidReloads=%u state=%s score=%.1f/MiB "
                "action=\"%s\"\n",
                passed ? "SETTLING" : "CHURN", name, transitions,
                heroPressureScenarioResourceRapidReloads_[slot],
                streamingStateName(resource.state), resource.scorePerMiB,
                resource.lastAction.c_str());
        }
        std::fflush(stdout);
        if (streamingSelfTest_)
        {
            streamingSelfTestExitCode_ = passed ? 0 : 1;
            streamingSelfTestFinished_ = true;
        }
    }

    void updateHeroPressureScenario()
    {
        if (!heroPressureScenarioActive_)
            return;
        for (size_t slot = 0; slot < virtualResources_.size(); ++slot)
        {
            const StreamingResourceState state = virtualResources_[slot].state;
            if (heroPressureScenarioStates_[slot] == state)
                continue;
            heroPressureScenarioStates_[slot] = state;
            ++heroPressureScenarioTransitions_;
            ++heroPressureScenarioResourceTransitions_[slot];
            heroPressureMaxResourceTransitions_ = std::max(
                heroPressureMaxResourceTransitions_,
                heroPressureScenarioResourceTransitions_[slot]);
        }

        const float elapsed =
            streamingTime_ - heroPressureScenarioStartTime_;
        const float committed = virtualResidentMiB() + virtualLoadingMiB();
        const bool withinBudget =
            committed <= virtualMemoryBudgetMiB_ + 0.001f;
        heroPressureScenarioWithinBudget_ &= withinBudget;
        if (elapsed >= kHeroOrbitTestSeconds)
        {
            const uint64_t currentCutInvalidations =
                streamingCurrentCutInvalidations_ -
                heroPressureScenarioStartCurrentCutInvalidations_;
            const uint64_t atomicPublicationViolations =
                streamingAtomicPublicationViolations_ -
                heroPressureScenarioStartAtomicPublicationViolations_;
            const uint64_t atomicGroupCompletions =
                streamingAtomicGroupCompletions_ -
                heroPressureScenarioStartAtomicGroupCompletions_;
            const uint64_t capacityBlockedRequests =
                streamingCapacityBlockedRequests_ -
                heroPressureScenarioStartCapacityBlockedRequests_;
            const uint64_t qualityDemotions =
                streamingQualityDemotions_ -
                heroPressureScenarioStartDemotions_;
            const bool handoffPassed =
                currentCutInvalidations == 0 &&
                atomicPublicationViolations == 0 &&
                heroPressureGeometricErrorRegressions_ == 0;
            const bool regressionCoveragePassed =
                !streamingSelfTest_ ||
                (atomicGroupCompletions != 0 &&
                 capacityBlockedRequests != 0 &&
                 qualityDemotions == 0 &&
                 heroPressureRollingPinObservations_ != 0 &&
                 heroPressureRollingReleaseObservations_ != 0 &&
                 streamingErrorMeasurements_ !=
                     heroPressureScenarioStartErrorMeasurements_ &&
                 streamingPositiveErrorMeasurements_ !=
                     heroPressureScenarioStartPositiveErrorMeasurements_);
            const bool passed =
                heroPressureScenarioWithinBudget_ &&
                heroPressureFocalObservedFrames_ != 0 &&
                heroPressureFocalFallbackFrames_ == 0 &&
                heroPressureDominantFallbackFrames_ == 0 &&
                heroPressureMaxResourceRapidReloads_ <=
                    kHeroOrbitMaxRapidReloads &&
                heroPressureRollingPinViolations_ == 0 &&
                streamingErrorMeasurementRegressions_ ==
                    heroPressureScenarioStartErrorMeasurementRegressions_ &&
                handoffPassed && regressionCoveragePassed;
            finishHeroPressureScenario(passed);
        }
    }

    void validateStreamingHandoff(
        const FrontierResultView& frontier, bool preserveCurrentCut)
    {
        bool validate = heroPressureScenarioActive_;
#ifndef NDEBUG
        validate = true;
#endif
        if (!validate)
            return;

        uint32_t currentCutViolations = 0;
        if (preserveCurrentCut)
        {
            for (const FrontierEntry& entry : frontier)
            {
                if (database_.isPayloadReady(
                        entry.nodeHandle, entry.payloadIndex()))
                    continue;
                ++currentCutViolations;
                if (streamingSelfTest_)
                {
                    const UserPayload payload = database_.tryGetPayload(
                        entry.nodeHandle, entry.payloadIndex());
                    std::printf(
                        "FRONTIER_STREAMING_SELF_TEST "
                        "CURRENT_CUT_INVALIDATED t=%.2f instance=%u "
                        "payload=%llu index=%u\n",
                        streamingTime_ - heroPressureScenarioStartTime_,
                        entry.instance(),
                        static_cast<unsigned long long>(payload),
                        uint32_t(entry.payloadIndex()));
                }
            }
            streamingCurrentCutInvalidations_ += currentCutViolations;
        }

        uint32_t pendingPublicationViolations = 0;
        for (const PendingStreamingGroup& group : pendingStreamingGroups_)
        {
            for (size_t slot : group.resources)
            {
                const VirtualResource& resource = virtualResources_[slot];
                if (resource.state != StreamingResourceState::Loading ||
                    database_.isPayloadReady(
                        resource.representative,
                        resource.representativePayloadIndex))
                    ++pendingPublicationViolations;
            }
        }
        streamingAtomicPublicationViolations_ +=
            pendingPublicationViolations;
        if (streamingSelfTest_ &&
            (currentCutViolations != 0 ||
             pendingPublicationViolations != 0))
            std::fflush(stdout);
        FRONTIER_ASSERT(
            currentCutViolations == 0,
            "streaming load invalidated an entry in the current cut");
        FRONTIER_ASSERT(
            pendingPublicationViolations == 0,
            "pending streaming groups must remain wholly unavailable");
    }

    FrontierErrorSnapshot captureStreamingErrorSnapshot(
        const Camera& camera, const SelectionParams& selection,
        std::span<const size_t> groupResources,
        PerformanceSample& performance)
    {
        const int64_t begin = bx::getHPCounter();
        const FrontierResultView measured =
            streamingErrorMeasurementQuery_.selectFrontier(
                database_, camera, selection);

        FrontierErrorSnapshot result;
        result.instanceMaxPixels.resize(entities_.size(), 0.0f);
        result.instanceVisible.resize(entities_.size(), 0);
        result.selectedGroupInstance.resize(entities_.size(), 0);
        for (const FrontierEntry& entry : measured)
        {
            const InstanceId instance = entry.instance();
            if (instance >= entities_.size())
                continue;
            if (result.instanceVisible[instance] == 0)
            {
                result.instanceVisible[instance] = 1;
                ++result.visibleInstances;
            }
            const float error =
                entry.approximateError(selection.threshold);
            result.instanceMaxPixels[instance] = std::max(
                result.instanceMaxPixels[instance], error);

            if (groupResources.empty())
                continue;
            const UserPayload rawPayload = database_.tryGetPayload(
                entry.nodeHandle, entry.payloadIndex());
            if (rawPayload == kInvalidPayload ||
                rawPayload >= UserPayload(kPayloadSlotCount))
                continue;
            const size_t slot = virtualResourceSlot(
                Payload(rawPayload), instance);
            if (std::find(groupResources.begin(), groupResources.end(),
                          slot) != groupResources.end())
                result.selectedGroupInstance[instance] = 1;
        }

        double squaredError = 0.0;
        for (size_t instance = 0;
             instance < result.instanceMaxPixels.size(); ++instance)
        {
            if (result.instanceVisible[instance] == 0)
                continue;
            const float error = result.instanceMaxPixels[instance];
            squaredError += double(error) * double(error);
            result.worstPixels = std::max(result.worstPixels, error);
        }
        if (result.visibleInstances != 0)
        {
            result.rmsPixels = float(std::sqrt(
                squaredError / double(result.visibleInstances)));
        }
        performance.streamingErrorMeasurementMs +=
            milliseconds(begin, bx::getHPCounter());
        return result;
    }

    StreamingErrorImprovement recordStreamingErrorImprovement(
        const PendingStreamingGroup& group,
        const std::string& resourceNames,
        const FrontierErrorSnapshot& before,
        const FrontierErrorSnapshot& after)
    {
        StreamingErrorImprovement result;
        result.resourceNames = resourceNames;
        result.groupSerial = group.serial;
        result.sceneBeforeRmsPixels = before.rmsPixels;
        result.sceneAfterRmsPixels = after.rmsPixels;
        result.sceneBeforeWorstPixels = before.worstPixels;
        result.sceneAfterWorstPixels = after.worstPixels;

        double localBeforeSquared = 0.0;
        double localAfterSquared = 0.0;
        for (size_t instance = 0;
             instance < after.selectedGroupInstance.size(); ++instance)
        {
            if (after.selectedGroupInstance[instance] == 0 ||
                instance >= before.instanceVisible.size() ||
                before.instanceVisible[instance] == 0 ||
                after.instanceVisible[instance] == 0)
                continue;
            const float beforeError = before.instanceMaxPixels[instance];
            const float afterError = after.instanceMaxPixels[instance];
            localBeforeSquared +=
                double(beforeError) * double(beforeError);
            localAfterSquared += double(afterError) * double(afterError);
            result.localBeforeWorstPixels = std::max(
                result.localBeforeWorstPixels, beforeError);
            result.localAfterWorstPixels = std::max(
                result.localAfterWorstPixels, afterError);
            ++result.affectedInstances;
        }
        if (result.affectedInstances != 0)
        {
            result.localBeforeRmsPixels = float(std::sqrt(
                localBeforeSquared / double(result.affectedInstances)));
            result.localAfterRmsPixels = float(std::sqrt(
                localAfterSquared / double(result.affectedInstances)));
        }
        result.valid = true;
        lastStreamingErrorImprovement_ = result;
        ++streamingErrorMeasurements_;
        if (result.affectedInstances == 0)
        {
            ++streamingNoCutChangePublications_;
        }
        else
        {
            ++streamingVisibleErrorMeasurements_;
            if (result.localAfterRmsPixels + 0.0001f <
                result.localBeforeRmsPixels)
                ++streamingPositiveErrorMeasurements_;
            if (result.localAfterRmsPixels >
                result.localBeforeRmsPixels + 0.01f)
                ++streamingErrorMeasurementRegressions_;
#ifndef NDEBUG
            FRONTIER_ASSERT(
                result.localAfterRmsPixels <=
                    result.localBeforeRmsPixels + 0.01f,
                "a newly selected LOD must not increase affected-object error");
#endif
        }
        return result;
    }

    static float errorImprovementPercent(float before, float after)
    {
        return before > 0.0001f
                   ? 100.0f * (before - after) / before
                   : 0.0f;
    }

    void completePendingStreamingGroups(
        const Camera& errorCamera, const SelectionParams& selection,
        PerformanceSample& performance)
    {
        bool haveBeforeSnapshot = false;
        FrontierErrorSnapshot beforeSnapshot;
        for (size_t groupIndex = 0;
             groupIndex < pendingStreamingGroups_.size();)
        {
            const PendingStreamingGroup& group =
                pendingStreamingGroups_[groupIndex];
            if (group.readyAt > streamingTime_)
            {
                ++groupIndex;
                continue;
            }

            bool publishable = !group.resources.empty();
            for (size_t slot : group.resources)
            {
                const VirtualResource& resource = virtualResources_[slot];
                const UserPayload payload =
                    database_.tryGetPayload(
                        resource.representative,
                        resource.representativePayloadIndex);
                publishable &=
                    resource.state == StreamingResourceState::Loading &&
                    payload == UserPayload(resource.payload) &&
                    !database_.isPayloadReady(
                        resource.representative,
                        resource.representativePayloadIndex);
            }

            if (publishable)
            {
                if (!haveBeforeSnapshot)
                {
                    streamingErrorMeasurementQuery_.reset();
                    beforeSnapshot = captureStreamingErrorSnapshot(
                        errorCamera, selection, {}, performance);
                    haveBeforeSnapshot = true;
                }
                for (size_t slot : group.resources)
                {
                    VirtualResource& resource = virtualResources_[slot];
                    database_.markPayloadReady(
                        resource.representative,
                        resource.representativePayloadIndex);
                }
                bool allReady = true;
                for (size_t slot : group.resources)
                {
                    const VirtualResource& resource =
                        virtualResources_[slot];
                    allReady &= database_.isPayloadReady(
                        resource.representative,
                        resource.representativePayloadIndex);
                }
                if (!allReady)
                {
                    ++streamingAtomicPublicationViolations_;
                    for (size_t slot : group.resources)
                    {
                        VirtualResource& resource =
                            virtualResources_[slot];
                        if (database_.isPayloadReady(
                                resource.representative,
                                resource.representativePayloadIndex))
                        {
                            database_.markPayloadUnavailable(
                                resource.representative,
                                resource.representativePayloadIndex);
                        }
                    }
                    publishable = false;
                    FRONTIER_ASSERT(
                        false,
                        "streaming group publication must be atomic");
                }
            }

            uint32_t completed = 0;
            uint32_t stale = 0;
            std::string names;
            for (size_t slot : group.resources)
            {
                char name[96];
                virtualResourceName(virtualResources_[slot], name,
                                    sizeof(name));
                if (!names.empty())
                    names += ", ";
                names += name;
            }
            if (publishable)
            {
                for (size_t slot : group.resources)
                {
                    VirtualResource& resource = virtualResources_[slot];
                    resource.state = StreamingResourceState::Resident;
                    resource.lastDemandTime = streamingTime_;
                    resource.decision = "keep: load completed";
                    resource.lastAction = "load completed";
                    ++completed;
                    ++streamingLoadsCompleted_;
                }
                if (group.resources.size() > 1)
                    ++streamingAtomicGroupCompletions_;
            }
            else
            {
                for (size_t slot : group.resources)
                {
                    VirtualResource& resource = virtualResources_[slot];
                    if (resource.state ==
                        StreamingResourceState::Loading)
                    {
                        resource.state =
                            StreamingResourceState::Unloaded;
                        resource.residentBenefitErrors.reset();
                        resource.decision =
                            "unloaded: atomic group publish aborted";
                        resource.lastAction =
                            "load completion skipped: stale group";
                    }
                    ++stale;
                }
            }

            char message[768];
            if (publishable)
            {
                const FrontierErrorSnapshot afterSnapshot =
                    captureStreamingErrorSnapshot(
                        errorCamera, selection, group.resources,
                        performance);
                const StreamingErrorImprovement improvement =
                    recordStreamingErrorImprovement(
                        group, names, beforeSnapshot, afterSnapshot);
                beforeSnapshot = afterSnapshot;
                if (improvement.affectedInstances != 0)
                {
                    std::snprintf(
                        message, sizeof(message),
                        "LOAD complete group #%llu: %s (%u resources) | "
                        "%.3f MiB, score "
                        "%.1f/MiB | error on %u objects: local RMS %.3f -> "
                        "%.3f px (%.1f%% better), overall RMS %.3f -> %.3f "
                        "px (%.3f%% better), scene worst %.3f -> %.3f px",
                        static_cast<unsigned long long>(group.serial),
                        names.c_str(), completed, group.byteSizeMiB,
                        group.scorePerMiB,
                        improvement.affectedInstances,
                        improvement.localBeforeRmsPixels,
                        improvement.localAfterRmsPixels,
                        errorImprovementPercent(
                            improvement.localBeforeRmsPixels,
                            improvement.localAfterRmsPixels),
                        improvement.sceneBeforeRmsPixels,
                        improvement.sceneAfterRmsPixels,
                        errorImprovementPercent(
                            improvement.sceneBeforeRmsPixels,
                            improvement.sceneAfterRmsPixels),
                        improvement.sceneBeforeWorstPixels,
                        improvement.sceneAfterWorstPixels);
                }
                else
                {
                    std::snprintf(
                        message, sizeof(message),
                        "LOAD complete group #%llu: %s (%u resources) | "
                        "%.3f MiB, score "
                        "%.1f/MiB | current cut unchanged at this camera "
                        "(prefetch/future demand)",
                        static_cast<unsigned long long>(group.serial),
                        names.c_str(), completed, group.byteSizeMiB,
                        group.scorePerMiB);
                }
            }
            else
            {
                std::snprintf(
                    message, sizeof(message),
                    "LOAD complete group #%llu: %s | 0/%u resources, "
                    "%.3f MiB, score %.1f/MiB (atomic publish aborted)",
                    static_cast<unsigned long long>(group.serial),
                    names.c_str(), uint32_t(group.resources.size()),
                    group.byteSizeMiB, group.scorePerMiB);
            }
            appendStreamingLog(
                stale == 0 ? ImVec4(0.35f, 0.85f, 0.45f, 1.0f)
                           : ImVec4(1.0f, 0.55f, 0.20f, 1.0f),
                message);
            pendingStreamingGroups_.erase(
                pendingStreamingGroups_.begin() + groupIndex);
        }
    }

    void recordStreamingConvergence(float deltaTime)
    {
        const bool converged = lastRefinementGroups_ == 0;
        if (!converged && !streamingConvergenceActive_)
        {
            streamingConvergenceActive_ = true;
            streamingConvergenceStartTime_ = streamingTime_;
        }
        else if (converged && streamingConvergenceActive_)
        {
            streamingConvergenceActive_ = false;
            lastStreamingConvergenceSeconds_ =
                streamingTime_ - streamingConvergenceStartTime_;
        }

        streamingConvergenceHistoryElapsed_ += deltaTime;
        if (streamingConvergenceHistoryCount_ == 0 ||
            streamingConvergenceHistoryElapsed_ >=
                kStreamingConvergenceSampleInterval)
        {
            streamingConvergenceHistoryElapsed_ = std::fmod(
                streamingConvergenceHistoryElapsed_,
                kStreamingConvergenceSampleInterval);
            streamingConvergenceHistory_[streamingConvergenceHistoryCursor_] =
                lastIdealEntryCount_ != 0
                    ? 100.0f * float(lastConvergedEntryCount_) /
                          float(lastIdealEntryCount_)
                    : 100.0f;
            streamingConvergenceHistoryCursor_ =
                (streamingConvergenceHistoryCursor_ + 1) %
                kStreamingConvergenceHistorySize;
            streamingConvergenceHistoryCount_ = std::min(
                streamingConvergenceHistoryCount_ + 1,
                kStreamingConvergenceHistorySize);
        }
    }

    void updateVirtualStreaming(const FrontierResultView& frontier,
                                const FrontierResultView& prefetchFrontier,
                                const Camera& errorCamera,
                                const SelectionParams& selection,
                                float deltaTime,
                                PerformanceSample& performance)
    {
        performance.streamingRefinementMs = 0.0f;
        performance.streamingErrorMeasurementMs = 0.0f;
        performance.streamingPlannerIndexMs = 0.0f;
        performance.streamingPlannerDemandMs = 0.0f;
        performance.streamingPlannerScoreMs = 0.0f;
        performance.streamingPlannerResidencyMs = 0.0f;
        const int64_t plannerStageStart = bx::getHPCounter();
        if (!streamingPaused_)
            streamingTime_ += deltaTime;

        if (!virtualStreamingEnabled_)
        {
            sceneTreeFallbackPinned_.fill(false);
            std::array<bool, kStreamingResourceSlotCount> currentDemand{};
            for (VirtualResource& resource : virtualResources_)
            {
                resource.currentErrors.reset();
                resource.currentBenefitErrors.reset();
                resource.idealErrors.reset();
                resource.transitionErrors.reset();
                resource.prefetchErrors.reset();
                resource.importanceScore =
                    resource.pinned
                        ? std::numeric_limits<float>::infinity()
                        : 0.0f;
                resource.scorePerMiB = resource.importanceScore;
                if (resource.state == StreamingResourceState::Resident)
                    resource.decision =
                        "resident: virtual streaming disabled";
            }
            for (const FrontierEntry& entry : frontier)
            {
                const UserPayload rawPayload =
                    database_.tryGetPayload(entry.nodeHandle,
                                            entry.payloadIndex());
                if (rawPayload == kInvalidPayload ||
                    rawPayload >= UserPayload(kPayloadSlotCount))
                    continue;
                const size_t slot = virtualResourceSlot(
                    Payload(rawPayload), entry.instance());
                if (slot >= virtualResources_.size())
                    continue;
                VirtualResource& resource = virtualResources_[slot];
                currentDemand[slot] = true;
                const float error =
                    entry.approximateError(lodThreshold_);
                resource.currentErrors.add(error);
                resource.currentBenefitErrors.add(error);
                resource.idealErrors.add(error);
            }

            currentFrontierMemoryMiB_ = 0.0f;
            lastReclaimableResourceCount_ = 0;
            lastReclaimableResidentMiB_ = 0.0f;
            for (size_t slot = 0; slot < virtualResources_.size(); ++slot)
            {
                const VirtualResource& resource = virtualResources_[slot];
                if (resource.pinned || currentDemand[slot])
                    currentFrontierMemoryMiB_ += resource.byteSizeMiB;
                if (!resource.pinned && !currentDemand[slot] &&
                    resource.state == StreamingResourceState::Resident)
                {
                    ++lastReclaimableResourceCount_;
                    lastReclaimableResidentMiB_ += resource.byteSizeMiB;
                }
            }
            idealFrontierMemoryMiB_ = currentFrontierMemoryMiB_;
            protectedFallbackMemoryMiB_ = 0.0f;
            lastRefinementGroups_ = 0;
            lastRefinementEntries_ = 0;
            lastPrefetchGroupCount_ = 0;
            lastPrefetchEntryCount_ = 0;
            lastIdealEntryCount_ = uint32_t(frontier.size());
            lastConvergedEntryCount_ = lastIdealEntryCount_;
            lastBudgetBlockedGroups_ = 0;
            lastCapacityBlockedGroups_ = 0;
            lastValueBlockedGroups_ = 0;
            minimumBlockedCommitMiB_ = 0.0f;
            streamingPlanValid_ = true;
            refinementPlanComplete_ = true;
            recordStreamingConvergence(deltaTime);
            performance.streamingPlannerDemandMs =
                milliseconds(plannerStageStart, bx::getHPCounter());
            return;
        }

        const bool preserveCurrentCut =
            virtualResidentMiB() + virtualLoadingMiB() <=
            virtualMemoryBudgetMiB_ + 0.001f;

        int64_t refinementStart = bx::getHPCounter();
        const FrontierRefinementView refinement =
            query_.computeFrontierRefinement(
                database_, frontier, SpatialQuery::UnlimitedDepth);
        int64_t refinementEnd = bx::getHPCounter();
        performance.streamingRefinementMs +=
            milliseconds(refinementStart, refinementEnd);
        const int64_t indexStart = refinementEnd;
        lastRefinementGroups_ = uint32_t(refinement.groupCount());
        lastRefinementEntries_ = uint32_t(refinement.entries().size());
        streamingPlanValid_ = true;
        refinementPlanComplete_ = refinement.complete();

        std::array<bool, kStreamingResourceSlotCount> currentDemand{};
        std::array<bool, kStreamingResourceSlotCount> idealDemand{};
        std::array<bool, kStreamingResourceSlotCount> transitionDemand{};
        std::array<bool, kStreamingResourceSlotCount> fallbackDemand{};
        std::array<bool, kStreamingResourceSlotCount> prefetchDemand{};
        // This is a lexicographic quality constraint, not another score
        // weight. A screen-dominant hero must retain (or acquire) at least
        // its first streamed representation before spare bytes are spent on
        // fine detail elsewhere.
        std::array<bool, kStreamingResourceSlotCount> qualityFloorDemand{};
        for (VirtualResource& resource : virtualResources_)
        {
            resource.currentErrors.reset();
            resource.currentBenefitErrors.reset();
            resource.idealErrors.reset();
            resource.transitionErrors.reset();
            resource.prefetchErrors.reset();
            resource.importanceScore = 0.0f;
            resource.scorePerMiB = 0.0f;
            resource.decision = "not demanded by current plan";
        }

        const auto resourceForEntry = [this](const FrontierEntry& entry)
        {
            const UserPayload rawPayload =
                database_.tryGetPayload(entry.nodeHandle,
                                        entry.payloadIndex());
            if (rawPayload == kInvalidPayload ||
                rawPayload >= UserPayload(kPayloadSlotCount))
                return kStreamingResourceSlotCount;
            const size_t slot = virtualResourceSlot(
                Payload(rawPayload), entry.instance());
            if (slot >= virtualResources_.size())
                return kStreamingResourceSlotCount;
            virtualResources_[slot].representative = entry.nodeHandle;
            virtualResources_[slot].representativePayloadIndex =
                entry.payloadIndex();
            return slot;
        };

        const float refinementThreshold = refinement.threshold();

        const int64_t indexEnd = bx::getHPCounter();
        performance.streamingPlannerIndexMs =
            milliseconds(indexStart, indexEnd);
        int64_t demandStart = indexEnd;

        lastIdealEntryCount_ = 0;
        lastConvergedEntryCount_ = 0;
        for (uint32_t currentIndex = 0; currentIndex < frontier.size();
             ++currentIndex)
        {
            const FrontierEntry& entry = frontier.entries[currentIndex];
            const size_t slot = resourceForEntry(entry);
            if (slot == kStreamingResourceSlotCount)
                continue;
            currentDemand[slot] = true;
            const float screenError =
                entry.approximateError(refinementThreshold);
            virtualResources_[slot].currentErrors.add(screenError);
            const UserPayload rawPayload =
                database_.tryGetPayload(entry.nodeHandle,
                                        entry.payloadIndex());
            if (rawPayload == UserPayload(Payload::TowerTop) &&
                screenError >= kHeroOrbitDominantFallbackErrorPixels)
            {
                const size_t districtSlot =
                    towerDistrictResourceSlot(entry.instance());
                if (districtSlot < qualityFloorDemand.size())
                    qualityFloorDemand[districtSlot] = true;
            }
            else if (rawPayload == UserPayload(Payload::TowerDistrict) &&
                     screenError * (0.90f / 0.70f) >=
                         kHeroOrbitDominantFallbackErrorPixels)
            {
                const size_t districtSlot =
                    towerDistrictResourceSlot(entry.instance());
                if (districtSlot < qualityFloorDemand.size())
                    qualityFloorDemand[districtSlot] = true;
            }
            if (refinement.currentExpansion(currentIndex) == kInvalidIndex)
            {
                idealDemand[slot] = true;
                virtualResources_[slot].idealErrors.add(screenError);
                ++lastIdealEntryCount_;
                ++lastConvergedEntryCount_;
            }
        }

        std::vector<StreamingCandidateGroup> candidates;
        for (uint32_t groupIndex = 0;
             groupIndex < refinement.groupCount(); ++groupIndex)
        {
            const bool immediate = refinement.depth(groupIndex) == 1;
            const float parentError =
                refinement.parentEntry(groupIndex).approximateError(
                    refinementThreshold);
            std::vector<size_t> groupResources;
            const std::span<const FrontierEntry> children =
                refinement.children(groupIndex);
            for (uint32_t childIndex = 0; childIndex < children.size();
                 ++childIndex)
            {
                const FrontierEntry& entry = children[childIndex];
                const size_t slot = resourceForEntry(entry);
                if (slot == kStreamingResourceSlotCount)
                    continue;
                if (refinement.childExpansion(groupIndex, childIndex) ==
                    kInvalidIndex)
                {
                    idealDemand[slot] = true;
                    virtualResources_[slot].idealErrors.add(parentError);
                    ++lastIdealEntryCount_;
                }
                if (immediate)
                {
                    groupResources.push_back(slot);
                    transitionDemand[slot] = true;
                    virtualResources_[slot].transitionErrors.add(parentError);
                }
            }
            if (!immediate || groupResources.empty())
                continue;
            std::sort(groupResources.begin(), groupResources.end());
            groupResources.erase(
                std::unique(groupResources.begin(), groupResources.end()),
                groupResources.end());
            if (groupResources.size() > kMaxStreamingResidencyGroupSize)
                continue;
            for (size_t slot : groupResources)
            {
                VirtualResource& resource = virtualResources_[slot];
                resource.residencyGroupCount =
                    uint8_t(groupResources.size());
                std::copy(groupResources.begin(), groupResources.end(),
                          resource.residencyGroup.begin());
            }
            const auto found = std::find_if(
                candidates.begin(), candidates.end(),
                [&groupResources](const StreamingCandidateGroup& candidate)
                { return candidate.resources == groupResources; });
            if (found == candidates.end())
                candidates.push_back({std::move(groupResources)});
        }

        lastPrefetchEntryCount_ = uint32_t(prefetchFrontier.size());
        lastPrefetchGroupCount_ = 0;
        if (!prefetchFrontier.empty())
        {
            refinementStart = bx::getHPCounter();
            performance.streamingPlannerDemandMs +=
                milliseconds(demandStart, refinementStart);
            const FrontierRefinementView prefetchRefinement =
                streamingLookaheadQuery_.computeFrontierRefinement(
                    database_, prefetchFrontier,
                    SpatialQuery::UnlimitedDepth);
            refinementEnd = bx::getHPCounter();
            performance.streamingRefinementMs +=
                milliseconds(refinementStart, refinementEnd);
            demandStart = refinementEnd;
            lastPrefetchGroupCount_ =
                uint32_t(prefetchRefinement.groupCount());
            const float prefetchThreshold = prefetchRefinement.threshold();

            for (const FrontierEntry& entry : prefetchFrontier)
            {
                const size_t slot = resourceForEntry(entry);
                if (slot == kStreamingResourceSlotCount)
                    continue;
                prefetchDemand[slot] = true;
                const float screenError =
                    entry.approximateError(prefetchThreshold);
                virtualResources_[slot].prefetchErrors.add(screenError);
                const UserPayload rawPayload =
                    database_.tryGetPayload(entry.nodeHandle,
                                            entry.payloadIndex());
                if (rawPayload == UserPayload(Payload::TowerTop) &&
                    screenError >= kHeroOrbitDominantFallbackErrorPixels)
                {
                    const size_t districtSlot =
                        towerDistrictResourceSlot(entry.instance());
                    if (districtSlot < qualityFloorDemand.size())
                        qualityFloorDemand[districtSlot] = true;
                }
                else if (
                    rawPayload == UserPayload(Payload::TowerDistrict) &&
                    screenError * (0.90f / 0.70f) >=
                        kHeroOrbitDominantFallbackErrorPixels)
                {
                    const size_t districtSlot =
                        towerDistrictResourceSlot(entry.instance());
                    if (districtSlot < qualityFloorDemand.size())
                        qualityFloorDemand[districtSlot] = true;
                }
            }
            for (uint32_t groupIndex = 0;
                 groupIndex < prefetchRefinement.groupCount(); ++groupIndex)
            {
                if (prefetchRefinement.depth(groupIndex) != 1)
                    continue;
                const float parentError =
                    prefetchRefinement.parentEntry(groupIndex)
                        .approximateError(prefetchThreshold);
                std::vector<size_t> groupResources;
                for (const FrontierEntry& entry :
                     prefetchRefinement.children(groupIndex))
                {
                    const size_t slot = resourceForEntry(entry);
                    if (slot == kStreamingResourceSlotCount)
                        continue;
                    groupResources.push_back(slot);
                    prefetchDemand[slot] = true;
                    virtualResources_[slot].prefetchErrors.add(parentError);
                }
                std::sort(groupResources.begin(), groupResources.end());
                groupResources.erase(
                    std::unique(groupResources.begin(),
                                groupResources.end()),
                    groupResources.end());
                if (groupResources.empty() ||
                    groupResources.size() >
                        kMaxStreamingResidencyGroupSize)
                    continue;
                for (size_t slot : groupResources)
                {
                    VirtualResource& resource = virtualResources_[slot];
                    resource.residencyGroupCount =
                        uint8_t(groupResources.size());
                    std::copy(groupResources.begin(), groupResources.end(),
                              resource.residencyGroup.begin());
                }
                const auto found = std::find_if(
                    candidates.begin(), candidates.end(),
                    [&groupResources](
                        const StreamingCandidateGroup& candidate)
                    { return candidate.resources == groupResources; });
                if (found == candidates.end())
                    candidates.push_back({std::move(groupResources)});
            }
        }

        const int64_t demandEnd = bx::getHPCounter();
        performance.streamingPlannerDemandMs +=
            milliseconds(demandStart, demandEnd);
        const int64_t scoreStart = demandEnd;

        for (size_t slot = 0; slot < currentDemand.size(); ++slot)
            if (currentDemand[slot] || prefetchDemand[slot])
                markStreamingRollingFallback(slot, fallbackDemand);
        validateStreamingRollingFallbacks(
            currentDemand, prefetchDemand, fallbackDemand);
        sceneTreeFallbackPinned_ = fallbackDemand;

        for (VirtualResource& resource : virtualResources_)
        {
            if (resource.currentErrors.count == 0)
                continue;
            const StreamingErrorStats& benefitSource =
                resource.residentBenefitErrors.count != 0
                    ? resource.residentBenefitErrors
                    : resource.currentErrors;
            resource.currentBenefitErrors = rebaseStreamingErrorCount(
                benefitSource, resource.currentErrors.count);
        }

        finalizeVirtualResourceScores();
        for (StreamingCandidateGroup& candidate : candidates)
        {
            float byteSizeMiB = 0.0f;
            for (size_t slot : candidate.resources)
            {
                candidate.importanceScore +=
                    virtualResources_[slot].importanceScore;
                byteSizeMiB += virtualResources_[slot].byteSizeMiB;
                candidate.qualityFloor |= qualityFloorDemand[slot];
            }
            candidate.scorePerMiB =
                candidate.importanceScore / std::max(byteSizeMiB, 0.001f);
        }
        std::stable_sort(
            candidates.begin(), candidates.end(),
            [](const StreamingCandidateGroup& lhs,
               const StreamingCandidateGroup& rhs)
            {
                if (lhs.qualityFloor != rhs.qualityFloor)
                    return lhs.qualityFloor;
                if (lhs.scorePerMiB != rhs.scorePerMiB)
                    return lhs.scorePerMiB > rhs.scorePerMiB;
                return lhs.importanceScore > rhs.importanceScore;
            });

        currentFrontierMemoryMiB_ = 0.0f;
        idealFrontierMemoryMiB_ = 0.0f;
        protectedFallbackMemoryMiB_ = 0.0f;
        lastReclaimableResourceCount_ = 0;
        lastReclaimableResidentMiB_ = 0.0f;
        for (size_t slot = 0; slot < virtualResources_.size(); ++slot)
        {
            VirtualResource& resource = virtualResources_[slot];
            if (resource.pinned || currentDemand[slot])
                currentFrontierMemoryMiB_ += resource.byteSizeMiB;
            if (resource.pinned || idealDemand[slot])
                idealFrontierMemoryMiB_ += resource.byteSizeMiB;
            if (!resource.pinned && fallbackDemand[slot] &&
                resource.state == StreamingResourceState::Resident)
            {
                protectedFallbackMemoryMiB_ += resource.byteSizeMiB;
                if (!currentDemand[slot] && !transitionDemand[slot])
                    resource.decision =
                        "keep: rolling immediate fallback pin";
            }
            if (!resource.pinned && qualityFloorDemand[slot] &&
                resource.state == StreamingResourceState::Resident)
                resource.decision =
                    "keep: minimum quality for screen-dominant hero";
            if (resource.pinned || currentDemand[slot] || idealDemand[slot] ||
                transitionDemand[slot] || prefetchDemand[slot] ||
                fallbackDemand[slot])
                resource.lastDemandTime = streamingTime_;
            else if (resource.state == StreamingResourceState::Resident)
            {
                ++lastReclaimableResourceCount_;
                lastReclaimableResidentMiB_ += resource.byteSizeMiB;
            }
        }

        recordStreamingConvergence(deltaTime);

        const int64_t scoreEnd = bx::getHPCounter();
        performance.streamingPlannerScoreMs =
            milliseconds(scoreStart, scoreEnd);

        if (streamingPaused_)
            return;

        const int64_t residencyStart = scoreEnd;

        struct ResidentGroup
        {
            std::array<size_t, kMaxStreamingResidencyGroupSize> resources{};
            size_t count = 0;
            float byteSizeMiB = 0.0f;
            float importanceScore = 0.0f;
            float scorePerMiB = 0.0f;
            bool current = false;
            bool transition = false;
            bool fallback = false;
            bool qualityFloor = false;
        };
        const auto residentGroupFor =
            [&](size_t seed)
            {
                ResidentGroup result;
                std::array<size_t, kMaxStreamingResidencyGroupSize>
                    topologyGroup{};
                const size_t topologyCount =
                    streamingResidencyGroup(seed, topologyGroup);
                for (size_t index = 0; index < topologyCount; ++index)
                {
                    const size_t slot = topologyGroup[index];
                    const VirtualResource& resource = virtualResources_[slot];
                    if (resource.pinned ||
                        resource.state != StreamingResourceState::Resident)
                        continue;
                    result.resources[result.count++] = slot;
                    result.byteSizeMiB += resource.byteSizeMiB;
                    result.importanceScore += resource.importanceScore;
                    result.current |= currentDemand[slot];
                    result.transition |= transitionDemand[slot];
                    result.fallback |= fallbackDemand[slot];
                    result.qualityFloor |= qualityFloorDemand[slot];
                }
                result.scorePerMiB =
                    result.importanceScore /
                    std::max(result.byteSizeMiB, 0.001f);
                return result;
            };
        const auto unloadResidentGroup =
            [&](const ResidentGroup& group, const char* reason,
                bool qualityDemotion)
            {
                if (qualityDemotion)
                    ++streamingQualityDemotions_;
                for (size_t index = 0; index < group.count; ++index)
                    unloadVirtualResource(group.resources[index], reason);
            };

        for (size_t group = pendingStreamingGroups_.size(); group-- > 0;)
        {
            bool stillDemanded = false;
            for (size_t slot : pendingStreamingGroups_[group].resources)
                stillDemanded |= currentDemand[slot] || idealDemand[slot] ||
                                 transitionDemand[slot] ||
                                 prefetchDemand[slot];
            if (!stillDemanded)
                cancelPendingStreamingGroup(group, "no longer in ideal plan");
        }

        while (virtualResidentMiB() + virtualLoadingMiB() >
                   virtualMemoryBudgetMiB_ &&
               !pendingStreamingGroups_.empty())
        {
            size_t cancellationGroup = 0;
            for (size_t group = 1; group < pendingStreamingGroups_.size();
                 ++group)
            {
                if (pendingStreamingGroups_[group].scorePerMiB <
                    pendingStreamingGroups_[cancellationGroup].scorePerMiB)
                    cancellationGroup = group;
            }
            cancelPendingStreamingGroup(cancellationGroup,
                                        "memory budget reduced");
        }

        while (virtualResidentMiB() > virtualMemoryBudgetMiB_)
        {
            ResidentGroup evictionGroup;
            int bestClass = 3;
            float lowestScorePerMiB =
                std::numeric_limits<float>::infinity();
            float largestMiB = 0.0f;
            for (size_t slot = 0; slot < virtualResources_.size(); ++slot)
            {
                const ResidentGroup group = residentGroupFor(slot);
                if (group.count == 0 || group.fallback ||
                    group.qualityFloor)
                    continue;
                const int evictionClass = !group.current &&
                                                  !group.transition
                                              ? 0
                                          : !group.current
                                              ? 1
                                              : 2;
                if (evictionClass < bestClass ||
                    (evictionClass == bestClass &&
                     (group.scorePerMiB < lowestScorePerMiB ||
                      (group.scorePerMiB == lowestScorePerMiB &&
                       group.byteSizeMiB > largestMiB))))
                {
                    evictionGroup = group;
                    bestClass = evictionClass;
                    lowestScorePerMiB = group.scorePerMiB;
                    largestMiB = group.byteSizeMiB;
                }
            }
            if (evictionGroup.count == 0)
                break;
            if (evictionGroup.current)
                unloadResidentGroup(
                    evictionGroup,
                    "memory budget reduced; coarsen complete group to ready "
                    "ancestor",
                    true);
            else
                unloadResidentGroup(evictionGroup,
                                    "memory budget reduced", false);
        }

        completePendingStreamingGroups(
            errorCamera, selection, performance);

        for (size_t slot = 0; slot < virtualResources_.size(); ++slot)
        {
            const VirtualResource& resource = virtualResources_[slot];
            const bool demanded = resource.pinned || currentDemand[slot] ||
                                  idealDemand[slot] || transitionDemand[slot] ||
                                  prefetchDemand[slot] ||
                                  fallbackDemand[slot];
            if (!demanded &&
                resource.state == StreamingResourceState::Resident &&
                streamingTime_ - resource.lastDemandTime >=
                    streamingUnloadDelaySeconds_)
                unloadVirtualResource(slot, "outside current ideal plan");
        }

        lastBudgetBlockedGroups_ = 0;
        lastCapacityBlockedGroups_ = 0;
        lastValueBlockedGroups_ = 0;
        minimumBlockedCommitMiB_ = 0.0f;
        for (const StreamingCandidateGroup& candidate : candidates)
        {
            std::string candidateNames;
            float candidateMiB = 0.0f;
            for (size_t slot : candidate.resources)
            {
                char name[96];
                virtualResourceName(virtualResources_[slot], name,
                                    sizeof(name));
                if (!candidateNames.empty())
                    candidateNames += ", ";
                candidateNames += name;
                candidateMiB += virtualResources_[slot].byteSizeMiB;
            }
            std::vector<size_t> missingResources;
            float missingMiB = 0.0f;
            for (size_t slot : candidate.resources)
            {
                const VirtualResource& resource = virtualResources_[slot];
                if (resource.state == StreamingResourceState::Unloaded)
                {
                    missingResources.push_back(slot);
                    missingMiB += resource.byteSizeMiB;
                }
            }
            if (missingResources.empty())
                continue;
            if (pendingStreamingGroups_.size() >=
                size_t(maxConcurrentStreamingLoads_))
            {
                for (size_t slot : missingResources)
                    virtualResources_[slot].decision =
                        "queued: concurrent load-group limit";
                continue;
            }

            const float committedMiB =
                virtualResidentMiB() + virtualLoadingMiB();
            float plannedCommittedMiB = committedMiB;
            float plannedVictimImportance = 0.0f;
            std::vector<ResidentGroup> evictionPlan;
            std::array<bool, kStreamingResourceSlotCount> plannedVictims{};
            while (plannedCommittedMiB + missingMiB >
                   virtualMemoryBudgetMiB_)
            {
                ResidentGroup evictionGroup;
                int bestEvictionClass = 2;
                float lowestScorePerMiB =
                    std::numeric_limits<float>::infinity();
                float largestMiB = 0.0f;
                for (size_t slot = 0; slot < virtualResources_.size(); ++slot)
                {
                    const ResidentGroup group = residentGroupFor(slot);
                    bool alreadyPlanned = false;
                    for (size_t index = 0; index < group.count; ++index)
                        alreadyPlanned |=
                            plannedVictims[group.resources[index]];
                    if (group.count == 0 || alreadyPlanned ||
                        group.qualityFloor ||
                        (!candidate.qualityFloor && group.transition) ||
                        (!candidate.qualityFloor && group.fallback) ||
                        group.current ||
                        (!candidate.qualityFloor &&
                         group.scorePerMiB >= candidate.scorePerMiB))
                        continue;
                    const bool betterTie =
                        evictionGroup.count != 0 &&
                        group.scorePerMiB == lowestScorePerMiB &&
                        group.byteSizeMiB > largestMiB;
                    constexpr int evictionClass = 0;
                    if (evictionClass < bestEvictionClass ||
                        (evictionClass == bestEvictionClass &&
                         (group.scorePerMiB < lowestScorePerMiB ||
                          betterTie)))
                    {
                        evictionGroup = group;
                        bestEvictionClass = evictionClass;
                        lowestScorePerMiB = group.scorePerMiB;
                        largestMiB = group.byteSizeMiB;
                    }
                }
                if (evictionGroup.count == 0)
                    break;
                for (size_t index = 0; index < evictionGroup.count; ++index)
                    plannedVictims[evictionGroup.resources[index]] = true;
                plannedCommittedMiB -= evictionGroup.byteSizeMiB;
                plannedVictimImportance += evictionGroup.importanceScore;
                evictionPlan.push_back(evictionGroup);
            }

            const bool capacityFeasible =
                plannedCommittedMiB + missingMiB <=
                virtualMemoryBudgetMiB_ + 0.001f;
            constexpr float replacementMinimumGain =
                kStreamingReplacementMinimumGain;
            const bool valueImproves =
                candidate.qualityFloor || evictionPlan.empty() ||
                candidate.importanceScore >=
                    plannedVictimImportance *
                        replacementMinimumGain;
            if (!capacityFeasible || !valueImproves)
            {
                ++lastBudgetBlockedGroups_;
                if (!capacityFeasible)
                {
                    ++lastCapacityBlockedGroups_;
                    ++streamingCapacityBlockedRequests_;
                }
                else
                    ++lastValueBlockedGroups_;
                const float required = committedMiB + missingMiB;
                if (minimumBlockedCommitMiB_ == 0.0f)
                    minimumBlockedCommitMiB_ = required;
                else
                    minimumBlockedCommitMiB_ =
                        std::min(minimumBlockedCommitMiB_, required);
                char blockedReason[192];
                if (!capacityFeasible)
                {
                    std::snprintf(
                        blockedReason, sizeof(blockedReason),
                        "blocked: complete %.2f MiB request cannot fit; "
                        "%.2f MiB after eligible victims",
                        missingMiB, plannedCommittedMiB + missingMiB);
                }
                else
                {
                    std::snprintf(
                        blockedReason, sizeof(blockedReason),
                        "blocked: request value %.2f does not exceed victim "
                        "value %.2f by %.0f%%",
                        candidate.importanceScore,
                        plannedVictimImportance,
                        (replacementMinimumGain - 1.0f) * 100.0f);
                }
                for (size_t slot : missingResources)
                    virtualResources_[slot].decision = blockedReason;
                continue;
            }

            if (streamingSelfTest_ && !evictionPlan.empty())
            {
                std::string victimNames;
                float victimMiB = 0.0f;
                for (const ResidentGroup& victimGroup : evictionPlan)
                {
                    victimMiB += victimGroup.byteSizeMiB;
                    for (size_t index = 0; index < victimGroup.count; ++index)
                    {
                        const VirtualResource& victim =
                            virtualResources_[victimGroup.resources[index]];
                        char name[96];
                        virtualResourceName(victim, name, sizeof(name));
                        if (!victimNames.empty())
                            victimNames += ", ";
                        victimNames += name;
                    }
                }
                std::printf(
                    "FRONTIER_STREAMING_SELF_TEST EXCHANGE t=%.2f "
                    "request=\"%s\" value=%.2f density=%.2f/MiB "
                    "groupMiB=%.2f missingMiB=%.2f victims=\"%s\" "
                    "value=%.2f MiB=%.2f\n",
                    streamingTime_, candidateNames.c_str(),
                    candidate.importanceScore, candidate.scorePerMiB,
                    candidateMiB, missingMiB, victimNames.c_str(),
                    plannedVictimImportance, victimMiB);
            }
            for (const ResidentGroup& evictionGroup : evictionPlan)
            {
                FRONTIER_ASSERT(
                    !evictionGroup.current,
                    "load admission cannot evict the current cut");
                unloadResidentGroup(
                    evictionGroup,
                    "lower group score/MiB than requested group", false);
            }

            PendingStreamingGroup group;
            group.readyAt = streamingTime_ + streamingLatencySeconds_;
            group.byteSizeMiB = missingMiB;
            group.scorePerMiB = candidate.scorePerMiB;
            group.serial = ++streamingGroupSerial_;
            std::string names;
            for (size_t slot : missingResources)
            {
                VirtualResource& resource = virtualResources_[slot];
                if (heroPressureScenarioActive_ &&
                    std::isfinite(heroPressureLastUnloadTime_[slot]) &&
                    streamingTime_ - heroPressureLastUnloadTime_[slot] <=
                        kHeroOrbitRapidReloadWindowSeconds)
                {
                    ++heroPressureRapidReloads_;
                    ++heroPressureScenarioResourceRapidReloads_[slot];
                    heroPressureMaxResourceRapidReloads_ = std::max(
                        heroPressureMaxResourceRapidReloads_,
                        heroPressureScenarioResourceRapidReloads_[slot]);
                }
                resource.state = StreamingResourceState::Loading;
                resource.lastDemandTime = streamingTime_;
                resource.residentBenefitErrors = resource.transitionErrors;
                char decision[128];
                std::snprintf(decision, sizeof(decision),
                              "loading: complete group score %.1f/MiB",
                              candidate.scorePerMiB);
                resource.decision = decision;
                resource.lastAction = "load requested";
                group.resources.push_back(slot);
                char name[96];
                virtualResourceName(resource, name, sizeof(name));
                if (!names.empty())
                    names += ", ";
                names += name;
            }
            char message[320];
            std::snprintf(
                message, sizeof(message),
                "LOAD request group #%llu: %s (%.3f MiB, score %.1f/MiB, "
                "%.2f s)",
                static_cast<unsigned long long>(group.serial), names.c_str(),
                group.byteSizeMiB, candidate.scorePerMiB,
                streamingLatencySeconds_);
            appendStreamingLog(ImVec4(1.0f, 0.78f, 0.20f, 1.0f), message);
            pendingStreamingGroups_.push_back(std::move(group));
        }

        if (streamingLatencySeconds_ <= 0.0f)
            completePendingStreamingGroups(
                errorCamera, selection, performance);
        validateStreamingHandoff(frontier, preserveCurrentCut);
        performance.streamingPlannerResidencyMs =
            milliseconds(residencyStart, bx::getHPCounter());
    }

    SpatialDatabase database_;
    SpatialQuery query_;
    SpatialQuery streamingLookaheadQuery_;
    SpatialQuery streamingErrorMeasurementQuery_;
    std::vector<Entity> entities_;

    std::array<SceneDefinition, 2> houseDefinitions_{};
    std::vector<InstanceHandle> houseHandles_;
    std::vector<InstanceHandle> towerHandles_;
    std::vector<InstanceHandle> treeHandles_;

    std::vector<InstanceHandle> carHandles_;
    std::vector<CarPath> carPaths_;
    std::vector<float4> carPositions_;
    std::vector<YawRotation> carYaws_;
    SpatialDatabase::RigidMotionGroup carMotion_;

    std::vector<InstanceHandle> pedestrianHandles_;
    std::vector<PedestrianPath> pedestrianPaths_;
    std::vector<float4> pedestrianPositions_;
    std::vector<YawRotation> pedestrianYaws_;
    SpatialDatabase::RigidMotionGroup pedestrianMotion_;

    std::vector<InstanceHandle> wholeSceneHandles_;
    std::vector<float4> wholeScenePositions_;
    std::vector<YawRotation> wholeSceneYaws_;
    SpatialDatabase::RigidMotionGroup wholeSceneMotion_;

    GeometryHandle roofGeometry_ = {UINT16_MAX};
    entry::MouseState mouse_;
    uint32_t width_ = 1280;
    uint32_t height_ = 720;
    uint32_t debug_ = BGFX_DEBUG_NONE;
    uint32_t reset_ = BGFX_RESET_VSYNC;
    std::string cpuModel_;
    std::string gpuModel_;
    std::string gpuDriver_;
    std::string windowSystem_;
    int64_t previousCounter_ = 0;
    float smoothedFps_ = 60.0f;
    float simulationTime_ = 0.0f;
    float cameraTime_ = 0.0f;
    float lodThreshold_ = 0.75f;
    float contributionCullPixels_ = 0.75f;
    bool freezeSimulation_ = false;
    bool animateWholeScene_ = false;
    bool restoreSceneAfterStress_ = false;
    bool houseReplacementPending_ = false;
    bool hierarchyTint_ = false;
    bool wireframeDebug_ = false;
    bool unlitSurfaces_ = false;
    bool freeCamera_ = false;
    bool freezeCullCamera_ = false;
    bool drawCullFrustum_ = true;
    bool seedFreeCamera_ = false;
    bool captureCullCamera_ = false;
    bool showFrontierDebug_ = true;
    bool showTlasMaintenance_ = false;
    bool showVirtualStreaming_ = false;
    bool showSceneStats_ = false;
    bool showPerformance_ = false;
    bool showSceneHierarchy_ = false;
    bool unlimitedTlasMaintenance_ = false;
    bool fullOptimizationRequested_ = false;
    bool topologyOptimizationRequested_ = false;
    int tlasMaintenanceBudget_ = 256;
    RebuildStrategy rebuildStrategy_ = RebuildStrategy::WhenRecommended;
    RebuildStrategy lastRebuildTrigger_ = RebuildStrategy::Manual;
    OptimizationMode scheduledOptimizationMode_ =
        OptimizationMode::TopologyOnly;
    OptimizationMode lastOptimizationMode_ =
        OptimizationMode::TopologyOnly;
    StreamingCutStrategy streamingCutStrategy_ =
        StreamingCutStrategy::QualityPerByte;
    float rebuildIntervalSeconds_ = 2.0f;
    float timeSinceRebuild_ = 0.0f;
    float lastRebuildMs_ = 0.0f;
    uint64_t rebuildCount_ = 0;
    uint64_t topologyOptimizationCount_ = 0;
    uint64_t fullOptimizationCount_ = 0;
    uint64_t rebuildRecommendationChecks_ = 0;
    uint64_t rebuildRecommendationSkips_ = 0;
    UpdateReport lastUpdateReport_{};
#ifdef FRONTIER_DEBUG_TOOLS
    bool showTlasHealth_ = false;
    bool showQueryCache_ = false;
    bool drawTlasAabbs_ = false;
    bool drawLooseBounds_ = false;
    bool debugBoundsXray_ = false;
    bool tlasHealthValid_ = false;
    int tlasDebugDepth_ = 0;
    int tlasDebugBoxLimit_ = 2048;
    int looseBoundsDrawLimit_ = 512;
    float nextTlasHealthSampleTime_ = 0.0f;
    TlasDebugSummary tlasHealth_{};
    std::vector<TlasDebugBox> tlasDebugBoxes_;
    std::vector<LooseInstanceDebugBounds> looseDebugBounds_;
    size_t lastTlasBoxesTotal_ = 0;
    size_t lastTlasBoxesDrawn_ = 0;
    size_t lastLooseBoundsTotal_ = 0;
    size_t lastLooseBoundsDrawn_ = 0;
    std::array<float, kPerformanceHistorySize> queryCacheHitHistory_{};
    size_t queryCacheHistoryCursor_ = 0;
    size_t queryCacheHistoryCount_ = 0;
#endif
    FrozenCullState frozenCull_;
    HouseStyle activeHouseStyle_ = HouseStyle::HouseA;
    HouseStyle pendingHouseStyle_ = HouseStyle::HouseA;
    uint32_t houseGeneration_ = 0;
    uint64_t authoredHlodNodeCount_ = 0;
    uint64_t motionFrame_ = 0;
    uint64_t updatedHlodNodes_ = 0;
    uint32_t movedInstances_ = 0;
    uint32_t houseCount_ = 0;
    uint32_t towerCount_ = 0;
    uint32_t treeCount_ = 0;
    uint32_t lastCurrentSize_ = 0;
    uint32_t lastRefinementGroups_ = 0;
    uint32_t lastRefinementEntries_ = 0;
    uint32_t lastPrefetchGroupCount_ = 0;
    uint32_t lastPrefetchEntryCount_ = 0;
    uint32_t lastIdealEntryCount_ = 0;
    uint32_t lastConvergedEntryCount_ = 0;
    uint32_t lastQueryReused_ = 0;
    uint32_t lastQueryWalked_ = 0;
    std::array<VirtualResource, kStreamingResourceSlotCount>
        virtualResources_{};
    std::array<StreamingResourceState, kStreamingResourceSlotCount>
        heroPressureScenarioStates_{};
    std::array<uint32_t, kStreamingResourceSlotCount>
        heroPressureScenarioResourceTransitions_{};
    std::array<uint32_t, kStreamingResourceSlotCount>
        heroPressureScenarioResourceRapidReloads_{};
    std::array<float, kStreamingResourceSlotCount>
        heroPressureLastUnloadTime_{};
    std::vector<PendingStreamingGroup> pendingStreamingGroups_;
    std::vector<StreamingLogEntry> streamingLog_;
    StreamingErrorImprovement lastStreamingErrorImprovement_;
    float virtualMemoryBudgetMiB_ = kHeroPressureTestBudgetMiB;
    float streamingLatencySeconds_ = 0.65f;
    float streamingUnloadDelaySeconds_ = 2.0f;
    float streamingTime_ = 0.0f;
    float currentFrontierMemoryMiB_ = 0.0f;
    float idealFrontierMemoryMiB_ = 0.0f;
    float protectedFallbackMemoryMiB_ = 0.0f;
    float minimumBlockedCommitMiB_ = 0.0f;
    int maxConcurrentStreamingLoads_ = 3;
    uint32_t lastBudgetBlockedGroups_ = 0;
    uint32_t lastCapacityBlockedGroups_ = 0;
    uint32_t lastValueBlockedGroups_ = 0;
    uint32_t lastReclaimableResourceCount_ = 0;
    float lastReclaimableResidentMiB_ = 0.0f;
    uint64_t streamingGroupSerial_ = 0;
    uint64_t streamingLoadsCompleted_ = 0;
    uint64_t streamingUnloads_ = 0;
    uint64_t streamingQualityDemotions_ = 0;
    uint64_t streamingCurrentCutInvalidations_ = 0;
    uint64_t streamingAtomicPublicationViolations_ = 0;
    uint64_t streamingAtomicGroupCompletions_ = 0;
    uint64_t streamingCapacityBlockedRequests_ = 0;
    uint64_t streamingErrorMeasurements_ = 0;
    uint64_t streamingVisibleErrorMeasurements_ = 0;
    uint64_t streamingPositiveErrorMeasurements_ = 0;
    uint64_t streamingNoCutChangePublications_ = 0;
    uint64_t streamingErrorMeasurementRegressions_ = 0;
    uint64_t heroPressureScenarioStartLoads_ = 0;
    uint64_t heroPressureScenarioStartUnloads_ = 0;
    uint64_t heroPressureScenarioStartDemotions_ = 0;
    uint64_t heroPressureScenarioStartCurrentCutInvalidations_ = 0;
    uint64_t heroPressureScenarioStartAtomicPublicationViolations_ = 0;
    uint64_t heroPressureScenarioStartAtomicGroupCompletions_ = 0;
    uint64_t heroPressureScenarioStartCapacityBlockedRequests_ = 0;
    uint64_t heroPressureScenarioStartErrorMeasurements_ = 0;
    uint64_t heroPressureScenarioStartPositiveErrorMeasurements_ = 0;
    uint64_t heroPressureScenarioStartErrorMeasurementRegressions_ = 0;
    uint64_t heroPressureScenarioLoads_ = 0;
    uint64_t heroPressureScenarioUnloads_ = 0;
    uint64_t heroPressureScenarioDemotions_ = 0;
    uint64_t heroPressureScenarioCurrentCutInvalidations_ = 0;
    uint64_t heroPressureScenarioAtomicPublicationViolations_ = 0;
    uint64_t heroPressureScenarioAtomicGroupCompletions_ = 0;
    uint64_t heroPressureScenarioCapacityBlockedRequests_ = 0;
    uint64_t heroPressureScenarioErrorMeasurements_ = 0;
    uint64_t heroPressureScenarioPositiveErrorMeasurements_ = 0;
    uint64_t heroPressureScenarioErrorMeasurementRegressions_ = 0;
    std::array<float, kStreamingConvergenceHistorySize>
        streamingConvergenceHistory_{};
    size_t streamingConvergenceHistoryCursor_ = 0;
    size_t streamingConvergenceHistoryCount_ = 0;
    float streamingConvergenceHistoryElapsed_ = 0.0f;
    float streamingConvergenceStartTime_ = 0.0f;
    float lastStreamingConvergenceSeconds_ = 0.0f;
    bool streamingPaused_ = false;
    bool streamingResetRequested_ = false;
    bool virtualStreamingEnabled_ = true;
    bool virtualStreamingToggleRequested_ = false;
    bool streamingPlanValid_ = false;
    bool refinementPlanComplete_ = false;
    bool streamingConvergenceActive_ = false;
    float heroPressureScenarioStartTime_ = 0.0f;
    float heroPressureScenarioElapsed_ = 0.0f;
    float heroPressureOrbitPhaseOffset_ = 0.0f;
    float heroPressureWorstFallbackError_ = 0.0f;
    float heroPressureFirstFallbackTime_ = -1.0f;
    float heroPressureLastFocalGeometricError_ =
        std::numeric_limits<float>::infinity();
    float heroPressureWorstDominantFallbackError_ = 0.0f;
    float heroPressureFirstDominantFallbackTime_ = -1.0f;
    float streamingTestCameraTime_ = 0.0f;
    float streamingTestBudgetMiB_ = kHeroPressureTestBudgetMiB;
    float streamingTestViewportHeight_ = 0.0f;
    InstanceId heroPressureFocalInstance_ = kInvalidInstanceId;
    uint32_t heroPressureScenarioTransitions_ = 0;
    uint32_t heroPressureScenarioFineHeroes_ = 0;
    uint32_t heroPressureFocalObservedFrames_ = 0;
    uint32_t heroPressureFocalFallbackFrames_ = 0;
    uint32_t heroPressureDominantFallbackFrames_ = 0;
    uint32_t heroPressureMaxDominantFallbacks_ = 0;
    uint32_t heroPressureLastDominantFallbacks_ = 0;
    uint32_t heroPressureMaxResourceTransitions_ = 0;
    uint32_t heroPressureRapidReloads_ = 0;
    uint32_t heroPressureMaxResourceRapidReloads_ = 0;
    uint32_t heroPressureGeometricErrorRegressions_ = 0;
    uint32_t heroPressureRollingPinObservations_ = 0;
    uint32_t heroPressureRollingReleaseObservations_ = 0;
    uint32_t heroPressureRollingPinViolations_ = 0;
    int heroPressureLastFocalRank_ = -2;
    bool heroPressureScenarioRequested_ = false;
    bool heroPressureScenarioActive_ = false;
    bool heroPressureScenarioFinished_ = false;
    bool heroPressureScenarioPassed_ = false;
    bool heroPressureScenarioWithinBudget_ = true;
    bool streamingSelfTest_ = false;
    bool streamingSelfTestFinished_ = false;
    int streamingSelfTestExitCode_ = 0;
    bool leftMouseWasDown_ = false;
    SceneTreeSelection sceneTreeSelection_;
    uint16_t pendingHierarchyRevealHero_ = UINT16_MAX;
    InstanceId pendingHierarchyRevealInstance_ = kInvalidInstanceId;
    std::array<uint32_t, kPayloadSlotCount> currentPayloadCounts_{};
    std::array<std::array<uint32_t, kTowerPayloadCount>,
               kHeroTowerAssetCount> currentHeroPayloadCounts_{};
    std::array<StreamingResourceState, kStreamingResourceSlotCount>
        sceneTreeResourceStates_{};
    std::array<bool, kStreamingResourceSlotCount>
        sceneTreeFallbackPinned_{};
    std::vector<uint32_t> currentInstanceEntryCounts_;
    bool sceneTreeStatsValid_ = false;
    PerformanceSample performance_;
    std::array<std::array<float, kPerformanceHistorySize>,
               kPerformanceTimerCount> performanceHistory_{};
    size_t performanceHistoryCursor_ = 0;
    size_t performanceHistoryCount_ = 0;
    float performanceHistoryElapsed_ = 0.0f;
    uint64_t performanceSampleCount_ = 0;
};

} // namespace

ENTRY_IMPLEMENT_MAIN(
    DynamicCity,
    "frontier-city",
    "Dynamic buildings, trees, traffic, pedestrians, and debug cameras selected by Frontier.",
    "https://github.com/bkaradzic/bgfx");
