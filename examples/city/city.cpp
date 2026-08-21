#include <frontier/builder.h>
#include <frontier/spatial_database.h>

#include "debugdraw/debugdraw.h"
#include "camera.h"
#include "entry/entry.h"
#include "imgui/imgui.h"

#include <bgfx/bgfx.h>
#include <bx/bounds.h>
#include <bx/math.h>
#include <bx/timer.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iterator>
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
constexpr size_t kPerformanceHistorySize = 300;
constexpr float kPerformanceHistorySampleInterval = 1.0f / 60.0f;

enum class PerformanceTimer : uint8_t
{
    Total,
    Ui,
    Simulation,
    Camera,
    Selection,
    CutStats,
    Render,
    Streaming,
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

enum class HouseStyle : uint8_t
{
    HouseA,
    HouseB,
};

enum class EntityKind : uint8_t
{
    House,
    Car,
    Pedestrian,
    Tree,
    Tower,
};

struct Entity
{
    float4 localPosition = float4::point(0.0f, 0.0f, 0.0f);
    float4 position = float4::point(0.0f, 0.0f, 0.0f);
    float scale = 1.0f;
    float localYaw = 0.0f;
    float yaw = 0.0f;
    uint32_t color = 0xffffffff;
    EntityKind kind = EntityKind::House;
    HouseStyle houseStyle = HouseStyle::HouseA;
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
    float cameraMs = 0.0f;
    float selectionMs = 0.0f;
    float cutStatsMs = 0.0f;
    float renderMs = 0.0f;
    float streamingMs = 0.0f;
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
    case TlasQuality::Morton: return "Morton";
    case TlasQuality::Median: return "Median";
    case TlasQuality::BinnedSAH: return "Binned SAH";
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
        : entry::AppI(name, description, url), query_(4.0f)
    {}

    void init(int32_t, const char* const*, uint32_t width,
              uint32_t height) override
    {
        width_ = width;
        height_ = height;
        reset_ = BGFX_RESET_VSYNC | BGFX_RESET_MSAA_X4;
        debug_ = BGFX_DEBUG_NONE;

        bgfx::Init init;
        init.platformData.nwh =
            entry::getNativeWindowHandle(entry::kDefaultWindowHandle);
        init.platformData.ndt = entry::getNativeDisplayHandle();
        init.platformData.type = entry::getNativeWindowHandleType();
        init.resolution.width = width_;
        init.resolution.height = height_;
        init.resolution.reset = reset_;
        bgfx::init(init);

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
        return 0;
    }

    bool update() override
    {
        if (entry::processEvents(width_, height_, debug_, reset_, &mouse_))
            return false;

        const int64_t now = bx::getHPCounter();
        const double frequency = double(bx::getHPFrequency());
        const float deltaTime = std::clamp(
            float(double(now - previousCounter_) / frequency), 1.0e-4f, 0.1f);
        previousCounter_ = now;
        smoothedFps_ += ((1.0f / deltaTime) - smoothedFps_) * 0.05f;

        PerformanceSample performance;
        int64_t stageStart = now;

        beginDebugUi();
        drawGlobalMenuBar();
        if (showFrontierDebug_)
            drawDebugUi();
        if (showSceneStats_)
            drawSceneStatsUi();
        if (showPerformance_)
            drawPerformanceUi();
        if (showSceneHierarchy_)
            drawSceneTreeUi();
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

        stageStart = stageEnd;
        if (houseReplacementPending_)
        {
            replaceHouses(pendingHouseStyle_);
            houseReplacementPending_ = false;
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
        lastUpdateReport_ = database_.applyUpdates(
            uint32_t(std::max(tlasMaintenanceBudget_, 0)));
        stageEnd = bx::getHPCounter();
        performance.simulationMs = milliseconds(stageStart, stageEnd);

        stageStart = stageEnd;
        cameraTime_ += deltaTime;
        CameraPose automaticPose;
        updateAutomaticCamera(cameraTime_, automaticPose.position,
                              automaticPose.target);
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
            .currentCutPolicy = CurrentCutPolicy::PreferReadyDescendants,
        };
        const FrontierResultView frontier =
            query_.selectFrontier(database_, frontierCamera, selection);
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
        publishVisibleResources(frontier);
        stageEnd = bx::getHPCounter();
        performance.streamingMs = milliseconds(stageStart, stageEnd);

        stageStart = stageEnd;
        bgfx::frame();
        stageEnd = bx::getHPCounter();
        performance.frameSubmitMs = milliseconds(stageStart, stageEnd);
        performance.totalMs = milliseconds(now, stageEnd);
        captureBgfxPerformance(performance);
        recordPerformance(performance, deltaTime);
        return true;
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

    void drawDebugUi()
    {
        ImGui::SetNextWindowPos(ImVec2(12.0f, 36.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(390.0f, 590.0f),
                                 ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Frontier debug", &showFrontierDebug_))
        {
            ImGui::End();
            return;
        }

        ImGui::Text("Simulation");
        ImGui::Separator();
        ImGui::Checkbox("Freeze simulation", &freezeSimulation_);
        ImGui::Checkbox("Hierarchy level tint", &hierarchyTint_);
        ImGui::Checkbox("Wireframe scene rendering", &wireframeDebug_);
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

    void drawSceneStatsUi()
    {
        ImGui::SetNextWindowPos(ImVec2(414.0f, 36.0f),
                                ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(350.0f, 250.0f),
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
        ImGui::Text("Current cut %u | ideal %u", lastCurrentSize_,
                    lastIdealSize_);
        ImGui::Text("Resources ready %u", resourcesPublished_);
        ImGui::Text("Query cache %u reused | %u walked",
                    lastQueryReused_, lastQueryWalked_);
        ImGui::Text("%.0f fps | simulation %.1f s", smoothedFps_,
                    simulationTime_);
        ImGui::Text("Whole-scene stress: %s",
                    animateWholeScene_ ? "ACTIVE" : "off");
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
        case PerformanceTimer::Camera: return sample.cameraMs;
        case PerformanceTimer::Selection: return sample.selectionMs;
        case PerformanceTimer::CutStats: return sample.cutStatsMs;
        case PerformanceTimer::Render: return sample.renderMs;
        case PerformanceTimer::Streaming: return sample.streamingMs;
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
        ImGui::SetNextWindowPos(ImVec2(414.0f, 258.0f),
                                ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(350.0f, 430.0f),
                                 ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Performance", &showPerformance_))
        {
            ImGui::End();
            return;
        }
        ImGui::Text("Smoothed values | rolling 5-10 second charts");

        ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.20f, 1.0f), "Frontier");
        ImGui::Separator();
        drawPerformanceTimer("Frontier select", PerformanceTimer::Selection,
                             ImVec4(1.0f, 0.78f, 0.20f, 1.0f));
        drawPerformanceTimer("Motion + DB apply",
                             PerformanceTimer::Simulation,
                             ImVec4(0.35f, 0.90f, 0.50f, 1.0f));
        drawPerformanceTimer("Resource publish", PerformanceTimer::Streaming,
                             ImVec4(0.90f, 0.60f, 0.25f, 1.0f));

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
    void refreshTlasHealth()
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
        refreshTlasHealth();
        ImGui::SetNextWindowPos(ImVec2(12.0f, 478.0f),
                                ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(390.0f, 410.0f),
                                 ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("TLAS health", &showTlasHealth_))
        {
            ImGui::End();
            return;
        }

        ImGui::Text("Quality %s | width %u",
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
        ImGui::Text("Quality baseline %u instances",
                    tlasHealth_.qualityBaselineInstances);
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
        ImGui::SliderInt("Repair nodes / frame", &tlasMaintenanceBudget_,
                         0, 4096);
        if (tlasHealth_.buildRequired)
            ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.20f, 1.0f),
                               "TLAS correctness build required");
        else if (tlasHealth_.optimizeRecommended)
            ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.20f, 1.0f),
                               "TLAS optimize recommended");

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
        ImGui::SetNextWindowPos(ImVec2(776.0f, 700.0f),
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

    bool beginPayloadTreeNode(const char* id, const char* label,
                              Payload payload, bool defaultOpen = false)
    {
        const uint32_t current = payloadCount(currentPayloadCounts_, payload);
        const uint32_t ideal = payloadCount(idealPayloadCounts_, payload);
        const ImVec4 color = current != 0
                                 ? payloadUiColor(payload)
                                 : ImVec4(0.52f, 0.52f, 0.56f, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGuiTreeNodeFlags flags =
            ImGuiTreeNodeFlags_SpanAvailWidth |
            ImGuiTreeNodeFlags_OpenOnArrow;
        if (defaultOpen)
            flags |= ImGuiTreeNodeFlags_DefaultOpen;
        const bool open = ImGui::TreeNodeEx(
            id, flags, "%s  [current %u | ideal %u]", label, current, ideal);
        ImGui::PopStyleColor();
        return open;
    }

    void drawPayloadTreeLeaf(const char* id, const char* label,
                             Payload payload)
    {
        const uint32_t current = payloadCount(currentPayloadCounts_, payload);
        const uint32_t ideal = payloadCount(idealPayloadCounts_, payload);
        const ImVec4 color = current != 0
                                 ? payloadUiColor(payload)
                                 : ImVec4(0.52f, 0.52f, 0.56f, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGui::TreeNodeEx(
            id,
            ImGuiTreeNodeFlags_Leaf |
                ImGuiTreeNodeFlags_NoTreePushOnOpen |
                ImGuiTreeNodeFlags_SpanAvailWidth,
            "%s  [current %u | ideal %u]", label, current, ideal);
        ImGui::PopStyleColor();
    }

    void drawSceneTreeUi()
    {
        ImGui::SetNextWindowPos(ImVec2(776.0f, 36.0f),
                                ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(492.0f, 652.0f),
                                 ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Scene hierarchy", &showSceneHierarchy_))
        {
            ImGui::End();
            return;
        }
        ImGui::TextWrapped(
            "Live Frontier topology. Bright nodes participate in the current "
            "cut; values show current / ideal selected entries.");
        ImGui::Separator();

        const uint32_t staticCount = houseCount_ + towerCount_ + treeCount_;
        const uint32_t dynamicCount = uint32_t(carHandles_.size() +
                                               pedestrianHandles_.size());
        if (ImGui::TreeNodeEx(
                "scene-root",
                ImGuiTreeNodeFlags_DefaultOpen |
                    ImGuiTreeNodeFlags_SpanAvailWidth,
                "City scene  (%u instances)", staticCount + dynamicCount))
        {
            if (ImGui::TreeNodeEx(
                    "static-scene",
                    ImGuiTreeNodeFlags_DefaultOpen |
                        ImGuiTreeNodeFlags_SpanAvailWidth,
                    "%s environment  (%u)",
                    animateWholeScene_ ? "Animated" : "Static",
                    staticCount))
            {
                if (ImGui::TreeNode(
                        "houses", "Houses %s, generation %u  (%u instances)",
                        activeHouseStyle_ == HouseStyle::HouseA ? "A" : "B",
                        houseGeneration_, houseCount_))
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

                if (ImGui::TreeNode("towers", "Skyscrapers  (%u instances)",
                                    towerCount_))
                {
                    if (beginPayloadTreeNode("tower-top", "Top / fallback",
                                             Payload::TowerTop, true))
                    {
                        if (beginPayloadTreeNode("tower-district", "District",
                                                 Payload::TowerDistrict, true))
                        {
                            if (beginPayloadTreeNode("tower-coarse", "Coarse",
                                                     Payload::TowerCoarse,
                                                     true))
                            {
                                if (beginPayloadTreeNode(
                                        "tower-medium", "Medium",
                                        Payload::TowerMedium, true))
                                {
                                    if (beginPayloadTreeNode(
                                            "tower-fine", "Fine",
                                            Payload::TowerFine, true))
                                    {
                                        drawPayloadTreeLeaf(
                                            "tower-base", "Base",
                                            Payload::TowerBase);
                                        drawPayloadTreeLeaf(
                                            "tower-shaft", "Shaft",
                                            Payload::TowerShaft);
                                        drawPayloadTreeLeaf(
                                            "tower-crown", "Crown",
                                            Payload::TowerCrown);
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
                    ImGui::TreePop();
                }

                if (ImGui::TreeNode("trees", "Trees  (%u instances)",
                                    treeCount_))
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

            if (ImGui::TreeNodeEx(
                    "dynamic-scene",
                    ImGuiTreeNodeFlags_DefaultOpen |
                        ImGuiTreeNodeFlags_SpanAvailWidth,
                    "Dynamic actors  (%u)", dynamicCount))
            {
                if (ImGui::TreeNode("cars", "Cars  (%u instances)",
                                    unsigned(carHandles_.size())))
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

                if (ImGui::TreeNode("pedestrians",
                                    "Pedestrians  (%u instances)",
                                    unsigned(pedestrianHandles_.size())))
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
        return makePerspectiveCamera(
            pose.position, pose.target - pose.position,
            float4::vec(0.0f, 1.0f, 0.0f), 58.0f * kPi / 180.0f,
            cameraAspect(), float(height_), 0.25f, kCameraFarPlane);
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
    }

    static float milliseconds(int64_t begin, int64_t end)
    {
        return float(double(end - begin) * 1000.0 /
                     double(bx::getHPFrequency()));
    }

    void updateFrontierStats(const FrontierResultView& frontier)
    {
        currentPayloadCounts_.fill(0);
        idealPayloadCounts_.fill(0);
        if (showSceneHierarchy_)
        {
            const auto countCut = [&](const auto& cut, auto& counts)
            {
                for (const FrontierEntry& entry : cut)
                {
                    const UserPayload rawPayload =
                        database_.tryGetPayload(entry.nodeHandle);
                    const size_t slot = size_t(rawPayload);
                    if (rawPayload != kInvalidPayload &&
                        slot < kPayloadSlotCount)
                        ++counts[slot];
                }
            };
            countCut(frontier.current(), currentPayloadCounts_);
            countCut(frontier.ideal(), idealPayloadCounts_);
        }
        lastCurrentSize_ = uint32_t(frontier.currentSize());
        lastIdealSize_ = uint32_t(frontier.idealSize());
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
        smooth(performance_.cameraMs, sample.cameraMs);
        smooth(performance_.selectionMs, sample.selectionMs);
        smooth(performance_.cutStatsMs, sample.cutStatsMs);
        smooth(performance_.renderMs, sample.renderMs);
        smooth(performance_.streamingMs, sample.streamingMs);
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

    SubtreeHandle createHouseDefinition(HouseStyle style)
    {
        // Geometric errors are world-space deviations. Sub-meter values make
        // the default 3 px threshold span several LODs across this city rather
        // than forcing every visible object to its zero-error leaves.
        SubtreeBuilder builder;
        const bool houseA = style == HouseStyle::HouseA;
        const AABB all = houseA
                             ? bounds(-3.3f, 0.0f, -3.3f,
                                      3.3f, 8.6f, 3.3f)
                             : bounds(-3.6f, 0.0f, -3.0f,
                                      3.6f, 7.4f, 3.0f);
        const auto coarse = builder.createNode(
            node(Payload::HouseCoarse, 0.28f, all));
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
        return database_.registerSubtree(builder.build());
    }

    SubtreeHandle createCarDefinition()
    {
        SubtreeBuilder builder;
        const AABB all = bounds(-2.2f, 0.0f, -2.2f, 2.2f, 2.2f, 2.2f);
        const auto coarse = builder.createNode(
            node(Payload::CarCoarse, 0.20f, all));
        builder.createNode(coarse, node(Payload::CarBody, 0.0f,
                                        bounds(-2.1f, 0.1f, -1.0f,
                                               2.1f, 1.0f, 1.0f)));
        builder.createNode(coarse, node(Payload::CarCabin, 0.0f,
                                        bounds(-0.9f, 0.9f, -0.85f,
                                               1.0f, 1.8f, 0.85f)));
        return database_.registerSubtree(builder.build());
    }

    SubtreeHandle createPedestrianDefinition()
    {
        SubtreeBuilder builder;
        const AABB all = bounds(-1.1f, 0.0f, -1.1f, 1.1f, 2.25f, 1.1f);
        const auto coarse = builder.createNode(
            node(Payload::PedestrianCoarse, 0.12f, all));
        builder.createNode(coarse, node(Payload::PedestrianBody, 0.0f,
                                        bounds(-0.35f, 0.05f, -0.35f,
                                               0.35f, 1.75f, 0.35f)));
        builder.createNode(coarse, node(Payload::PedestrianHead, 0.0f,
                                        bounds(-0.3f, 1.65f, -0.3f,
                                               0.58f, 2.25f, 0.3f)));
        return database_.registerSubtree(builder.build());
    }

    SubtreeHandle createTreeDefinition()
    {
        SubtreeBuilder builder;
        const AABB all = bounds(-1.8f, 0.0f, -1.8f, 1.8f, 6.5f, 1.8f);
        const auto coarse = builder.createNode(
            node(Payload::TreeCoarse, 0.22f, all));
        builder.createNode(coarse, node(Payload::TreeTrunk, 0.0f,
                                        bounds(-0.35f, 0.0f, -0.35f,
                                               0.35f, 3.4f, 0.35f)));
        builder.createNode(coarse, node(Payload::TreeCrown, 0.0f,
                                        bounds(-1.8f, 2.2f, -1.8f,
                                               1.8f, 6.5f, 1.8f)));
        return database_.registerSubtree(builder.build());
    }

    SubtreeHandle createTowerDefinition()
    {
        SubtreeBuilder builder;
        const AABB all = bounds(-5.0f, 0.0f, -5.0f, 5.0f, 46.0f, 5.0f);
        const auto district = builder.createNode(
            node(Payload::TowerDistrict, 0.70f, all));
        const auto coarse = builder.createNode(
            district, node(Payload::TowerCoarse, 0.52f, all));
        const auto medium = builder.createNode(
            coarse, node(Payload::TowerMedium, 0.35f, all));
        const auto fine = builder.createNode(
            medium, node(Payload::TowerFine, 0.18f, all));
        builder.createNode(fine, node(Payload::TowerBase, 0.0f,
                                      bounds(-5.0f, 0.0f, -5.0f,
                                             5.0f, 7.0f, 5.0f)));
        builder.createNode(fine, node(Payload::TowerShaft, 0.0f,
                                      bounds(-4.2f, 6.8f, -4.2f,
                                             4.2f, 38.0f, 4.2f)));
        builder.createNode(fine, node(Payload::TowerCrown, 0.0f,
                                      bounds(-4.2f, 37.8f, -4.2f,
                                             4.2f, 46.0f, 4.2f)));
        return database_.registerSubtree(builder.build());
    }

    void rememberEntity(InstanceHandle handle, const Entity& entity)
    {
        if (entities_.size() <= handle.id)
            entities_.resize(size_t(handle.id) + 1);
        entities_[handle.id] = entity;
    }

    InstanceHandle instantiateActor(Payload fallback, float error,
                                    const AABB& localBounds,
                                    const Entity& entity,
                                    SubtreeHandle definition)
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
        database_.mountSubtree(handle.rootNode(), definition);
        rememberEntity(handle, placed);
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
            Payload::HouseTop, 0.75f, houseBounds(style), house,
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
        std::vector<float4> lots;
        lots.reserve(houseHandles_.size());
        for (InstanceHandle handle : houseHandles_)
        {
            if (handle.id < entities_.size())
                lots.push_back(entities_[handle.id].localPosition);
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
    }

    void createScene()
    {
        houseDefinitions_[static_cast<size_t>(HouseStyle::HouseA)] =
            createHouseDefinition(HouseStyle::HouseA);
        houseDefinitions_[static_cast<size_t>(HouseStyle::HouseB)] =
            createHouseDefinition(HouseStyle::HouseB);
        const SubtreeHandle carDefinition = createCarDefinition();
        const SubtreeHandle pedestrianDefinition =
            createPedestrianDefinition();
        const SubtreeHandle treeDefinition = createTreeDefinition();
        const SubtreeHandle towerDefinition = createTowerDefinition();
        uint32_t random = 0x5eed1234u;

        const AABB treeBounds =
            bounds(-1.9f, 0.0f, -1.9f, 1.9f, 6.7f, 1.9f);
        const AABB towerBounds =
            bounds(-5.2f, 0.0f, -5.2f, 5.2f, 46.5f, 5.2f);
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
                    Entity tower;
                    tower.kind = EntityKind::Tower;
                    tower.position = float4::point(centerX, 0.0f, centerZ);
                    tower.scale = 0.82f + random01(random) * 0.28f;
                    tower.yaw = float((x + z) & 1) * kPi * 0.5f;
                    tower.color = abgr(
                        uint8_t(105 + random01(random) * 55),
                        uint8_t(125 + random01(random) * 60),
                        uint8_t(145 + random01(random) * 65));
                    towerHandles_.push_back(instantiateActor(
                        Payload::TowerTop, 0.90f, towerBounds,
                        tower, towerDefinition));
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
                        Payload::TreeTop, 0.65f, treeBounds,
                        tree, treeDefinition));
                    ++treeCount_;
                }
            }
        }

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
                Payload::CarTop, 0.55f, carBounds, car, carDefinition));
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
                Payload::PedestrianTop, 0.32f, pedestrianBounds,
                pedestrian, pedestrianDefinition));
            pedestrianPaths_.push_back(path);
        }
        pedestrianMotion_.reset(pedestrianHandles_);

        resetWholeSceneMotionGroup();

        database_.applyUpdates(0);
        database_.optimize();
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

    void updateActors(float time)
    {
        updateMovingActorSources(time);
        for (size_t index = 0; index < carHandles_.size(); ++index)
        {
            Entity& entity = entities_[carHandles_[index].id];
            entity.position = entity.localPosition;
            entity.yaw = entity.localYaw;
            carPositions_[index] = entity.position;
            carYaws_[index] = yawRotation(entity.yaw);
        }
        database_.moveRigidInstances(carMotion_, carPositions_, carYaws_);

        for (size_t index = 0; index < pedestrianHandles_.size(); ++index)
        {
            Entity& entity = entities_[pedestrianHandles_[index].id];
            entity.position = entity.localPosition;
            entity.yaw = entity.localYaw;
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
            entity.position = entity.localPosition;
            entity.position.y += amplitude * std::cos(
                time * kWorstCaseWaveFrequency + phase);
            entity.yaw = entity.localYaw;
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
                     const Entity& entity)
    {
        setEntityTransform(encoder, entity);
        encoder.setWireframe(wireframeDebug_);
        const auto paint = [&](uint32_t authoredColor)
        {
            encoder.setColor(hierarchyTint_ ? hierarchyTint(payload)
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
        drawWorld(encoder);
        for (const FrontierEntry& entry : frontier.current())
        {
            const UserPayload rawPayload =
                database_.tryGetPayload(entry.nodeHandle);
            if (rawPayload == kInvalidPayload ||
                entry.instance() >= entities_.size())
                continue;
            drawPayload(encoder, Payload(rawPayload),
                        entities_[entry.instance()]);
        }
        drawFrozenCullFrustum(encoder);
#ifdef FRONTIER_DEBUG_TOOLS
        drawSpatialDebugBounds(encoder);
#endif
        encoder.end();
    }

    void publishVisibleResources(const FrontierResultView& frontier)
    {
        // This is a tiny stand-in for an async GPU streamer: ideal-cut nodes
        // become ready after first visibility and are renderable next frame.
        uint32_t budget = 12u * uint32_t(kDistrictCount);
        for (const FrontierEntry& entry : frontier.ideal())
        {
            if (budget == 0)
                break;
            if (!database_.isNodeReady(entry.nodeHandle))
            {
                database_.markNodeReady(entry.nodeHandle);
                --budget;
                ++resourcesPublished_;
            }
        }
    }

    SpatialDatabase database_;
    SpatialQuery query_;
    std::vector<Entity> entities_;

    std::array<SubtreeHandle, 2> houseDefinitions_{};
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
    int64_t previousCounter_ = 0;
    float smoothedFps_ = 60.0f;
    float simulationTime_ = 0.0f;
    float cameraTime_ = 0.0f;
    float lodThreshold_ = 3.0f;
    float contributionCullPixels_ = 0.75f;
    bool freezeSimulation_ = false;
    bool animateWholeScene_ = false;
    bool restoreSceneAfterStress_ = false;
    bool houseReplacementPending_ = false;
    bool hierarchyTint_ = false;
    bool wireframeDebug_ = false;
    bool freeCamera_ = false;
    bool freezeCullCamera_ = false;
    bool drawCullFrustum_ = true;
    bool seedFreeCamera_ = false;
    bool captureCullCamera_ = false;
    bool showFrontierDebug_ = true;
    bool showSceneStats_ = false;
    bool showPerformance_ = false;
    bool showSceneHierarchy_ = false;
    int tlasMaintenanceBudget_ = 256;
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
    uint32_t houseCount_ = 0;
    uint32_t towerCount_ = 0;
    uint32_t treeCount_ = 0;
    uint32_t lastCurrentSize_ = 0;
    uint32_t lastIdealSize_ = 0;
    uint32_t lastQueryReused_ = 0;
    uint32_t lastQueryWalked_ = 0;
    uint32_t resourcesPublished_ = 0;
    std::array<uint32_t, kPayloadSlotCount> currentPayloadCounts_{};
    std::array<uint32_t, kPayloadSlotCount> idealPayloadCounts_{};
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
