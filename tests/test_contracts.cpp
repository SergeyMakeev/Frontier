#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>

#include "frontier/detail/subtree_data.h"
#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

namespace {

struct AllocationTracker
{
    size_t allocations = 0;
    size_t frees = 0;
    size_t lastAlignment = 0;
};

void* trackedAlloc(size_t bytes, size_t alignment, void* user)
{
    auto& tracker = *static_cast<AllocationTracker*>(user);
    ++tracker.allocations;
    tracker.lastAlignment = alignment;
    return defaultAlloc(bytes, alignment, nullptr);
}

void trackedFree(void* ptr, void* user)
{
    auto& tracker = *static_cast<AllocationTracker*>(user);
    if (ptr) ++tracker.frees;
    defaultFree(ptr, nullptr);
}

} // namespace

TEST(Contracts, SerializedSubtreeRejectsCorruption)
{
    SubtreeBytes bytes = makeLeafSubtree(7);
    bytes.data()[0] ^= std::byte{0xff};
    SpatialDatabase database;
    EXPECT_THROW(database.registerSubtree(std::move(bytes)), std::logic_error);
}

TEST(Contracts, DefaultAllocatorRejectsInvalidOrOverflowingRequests)
{
    EXPECT_EQ(defaultAlloc(16, 3, nullptr), nullptr);
    EXPECT_EQ(defaultAlloc(std::numeric_limits<size_t>::max(), 64, nullptr),
              nullptr);
}

TEST(Contracts, AppendBufferRejectsUnrepresentableCapacity)
{
    detail::AppendBuffer<uint64_t> buffer;
    EXPECT_THROW(buffer.reserve(std::numeric_limits<size_t>::max()),
                 std::length_error);
}

TEST(Contracts, SubtreeBytesPreserveCustomAllocatorOwnershipAcrossCopiesAndMoves)
{
    AllocationTracker tracker;
    FrontierContext context;
    context.alloc = &trackedAlloc;
    context.free = &trackedFree;
    context.user = &tracker;

    {
        SubtreeBuilder builder;
        builder.createNode(node(7, 0.0f, box()));
        SubtreeBytes source = builder.build(context);
        EXPECT_EQ(tracker.lastAlignment, kSubtreeByteAlignment);

        SubtreeBytes copy = source;
        ASSERT_NE(copy.data(), source.data());
        EXPECT_EQ(std::memcmp(copy.data(), source.data(), source.size()), 0);

        const void* movedAllocation = copy.data();
        SubtreeBytes moved = std::move(copy);
        EXPECT_TRUE(copy.empty());
        EXPECT_EQ(moved.data(), movedAllocation);

        SubtreeBytes assigned;
        assigned = source;
        EXPECT_EQ(std::memcmp(assigned.data(), source.data(), source.size()), 0);

        SpatialDatabase database;
        const SubtreeHandle handle =
            database.registerSubtree(std::move(moved));
        EXPECT_TRUE(moved.empty());
        database.releaseSubtree(handle);
    }

    EXPECT_EQ(tracker.allocations, 3u);
    EXPECT_EQ(tracker.frees, tracker.allocations);
}

TEST(Contracts, SerializedSubtreeValidationMatchesBuildMode)
{
    const auto makeHierarchy = []
    {
        SubtreeBuilder builder;
        const auto root =
            builder.createNode(node(1, 8.0f, box(2.0f)));
        builder.createNode(root, node(2, 0.0f, box(1.0f)));
        return builder.build();
    };
    const auto registerStructurallyCorrupt = [](SubtreeBytes bytes)
    {
        SpatialDatabase database;
#if FRONTIER_VALIDATE_SUBTREES
        EXPECT_THROW(database.registerSubtree(std::move(bytes)),
                     std::logic_error);
#else
        const SubtreeHandle subtree =
            database.registerSubtree(std::move(bytes));
        EXPECT_TRUE(subtree.valid());
        database.releaseSubtree(subtree);
#endif
    };

    {
        SubtreeBytes bytes = makeHierarchy();
        const auto* header = reinterpret_cast<const detail::SubtreeHeader*>(
            bytes.data());
        auto* payload = reinterpret_cast<detail::PayloadWord*>(
            bytes.data() + header->payloadOffset);
        payload[1] = detail::invalidPayloadWord();
        registerStructurallyCorrupt(std::move(bytes));
    }
    {
        SubtreeBytes bytes = makeHierarchy();
        const auto* header = reinterpret_cast<const detail::SubtreeHeader*>(
            bytes.data());
        auto* parent = reinterpret_cast<uint32_t*>(
            bytes.data() + header->parentOffset);
        parent[2] = header->nodeCount;
        registerStructurallyCorrupt(std::move(bytes));
    }
    {
        SubtreeBytes bytes = makeHierarchy();
        const auto* header = reinterpret_cast<const detail::SubtreeHeader*>(
            bytes.data());
        auto* parent = reinterpret_cast<uint32_t*>(
            bytes.data() + header->parentOffset);
        parent[2] = detail::packParent(1, detail::kMaxChildren);
        registerStructurallyCorrupt(std::move(bytes));
    }
    {
        SubtreeBytes bytes = makeHierarchy();
        auto* header = reinterpret_cast<detail::SubtreeHeader*>(bytes.data());
        header->rootBoundsMax[0] = header->rootBoundsMin[0] - 1.0f;
        registerStructurallyCorrupt(std::move(bytes));
    }
    // Reserved header words are part of the constant-time format envelope and
    // remain checked even when the full traversal scan is compiled out.
    {
        SubtreeBytes bytes = makeHierarchy();
        auto* header = reinterpret_cast<detail::SubtreeHeader*>(bytes.data());
        header->reserved[0] = 1;
        SpatialDatabase database;
        EXPECT_THROW(database.registerSubtree(std::move(bytes)),
                     std::logic_error);
    }
    {
        SubtreeBytes bytes = makeHierarchy();
        auto* header = reinterpret_cast<detail::SubtreeHeader*>(bytes.data());
        header->branchingFactor = kWide == 4 ? 8 : 4;
        SpatialDatabase database;
        EXPECT_THROW(database.registerSubtree(std::move(bytes)),
                     std::logic_error);
    }
    {
        SubtreeBytes bytes = makeHierarchy();
        auto* header = reinterpret_cast<detail::SubtreeHeader*>(bytes.data());
        header->payloadBytes = header->payloadBytes == 4 ? 8 : 4;
        SpatialDatabase database;
        EXPECT_THROW(database.registerSubtree(std::move(bytes)),
                     std::logic_error);
    }
    {
        SubtreeBytes bytes = makeHierarchy();
        auto* header = reinterpret_cast<detail::SubtreeHeader*>(bytes.data());
        header->invalidPayloadWord ^= 1;
        SpatialDatabase database;
        EXPECT_THROW(database.registerSubtree(std::move(bytes)),
                     std::logic_error);
    }
    {
        SubtreeBytes bytes = makeHierarchy();
        const auto* header = reinterpret_cast<const detail::SubtreeHeader*>(
            bytes.data());
        auto* subtreeSize = reinterpret_cast<uint32_t*>(
            bytes.data() + header->subtreeSizeOffset);
        subtreeSize[1] = 0;
        registerStructurallyCorrupt(std::move(bytes));
    }
    {
        SubtreeBytes bytes = makeHierarchy();
        const auto* header = reinterpret_cast<const detail::SubtreeHeader*>(
            bytes.data());
        auto* wide = reinterpret_cast<detail::WideBlock*>(
            bytes.data() + header->wideOffset);
        wide[0].bounds.mxy.v[0] = wide[0].bounds.mny.v[0] - 1.0f;
        registerStructurallyCorrupt(std::move(bytes));
    }
    {
        SubtreeBytes bytes = makeHierarchy();
        const auto* header = reinterpret_cast<const detail::SubtreeHeader*>(
            bytes.data());
        auto* error = reinterpret_cast<float*>(
            bytes.data() + header->errorOffset);
        error[1] = std::numeric_limits<float>::quiet_NaN();
        registerStructurallyCorrupt(std::move(bytes));
    }
    {
        SubtreeBytes bytes = makeHierarchy();
        const auto* header = reinterpret_cast<const detail::SubtreeHeader*>(
            bytes.data());
        auto* wide = reinterpret_cast<detail::WideBlock*>(
            bytes.data() + header->wideOffset);
        wide[0].child[0] = kInvalidIndex;
        registerStructurallyCorrupt(std::move(bytes));
    }
    {
        SubtreeBytes bytes = makeHierarchy();
        const auto* header = reinterpret_cast<const detail::SubtreeHeader*>(
            bytes.data());
        auto* mask = reinterpret_cast<uint32_t*>(
            bytes.data() + header->maskOffset);
        mask[0] ^= 1u << 31;
        registerStructurallyCorrupt(std::move(bytes));
    }
}

TEST(Contracts, InvalidConfigurationAndSelectionInputsAreRejected)
{
    SpatialDatabaseConfig invalidConfig;
    invalidConfig.tlasAreaDrift =
        std::numeric_limits<float>::quiet_NaN();
    EXPECT_THROW(SpatialDatabase{invalidConfig}, std::logic_error);

    invalidConfig = {};
    invalidConfig.parallelInstanceThreshold = 1;
    invalidConfig.context.workerCount = 2;
    invalidConfig.context.parallelFor = nullptr;
    EXPECT_THROW(SpatialDatabase{invalidConfig}, std::logic_error);

    SpatialDatabase database;
    database.instantiate(node(1, 0.0f, box()));
    database.applyUpdates();
    SpatialQuery query;

    SelectionParams params;
    params.threshold = 0.0f;
    EXPECT_THROW(query.selectFrontier(database, cameraAt(), params),
                 std::logic_error);
    params = {};
    params.minPix = -1.0f;
    EXPECT_THROW(query.selectFrontier(database, cameraAt(), params),
                 std::logic_error);
    params = {};
    params.currentCutPolicy = static_cast<CurrentCutPolicy>(255);
    EXPECT_THROW(query.selectFrontier(database, cameraAt(), params),
                 std::logic_error);

    Camera invalidCamera = cameraAt();
    invalidCamera.pos.x = std::numeric_limits<float>::infinity();
    EXPECT_THROW(query.selectFrontier(database, invalidCamera, {}),
                 std::logic_error);
}

TEST(Contracts, DefinitionsHaveHandleIdentityNotContentKeys)
{
    SpatialDatabase database;
    SubtreeHandle first = database.registerSubtree(makeLeafSubtree(1));
    SubtreeHandle second = database.registerSubtree(makeLeafSubtree(1));
    EXPECT_NE(first.slot, second.slot);
}

TEST(Contracts, DefinitionCannotBeReleasedWhileMounted)
{
    SpatialDatabase database;
    SubtreeHandle subtree = database.registerSubtree(
        makeLeafSubtree(1));
    InstanceHandle instance = instantiateFor(database, subtree, box(2));
    EXPECT_THROW(database.releaseSubtree(subtree), std::logic_error);
    database.removeInstance(instance);
    database.releaseSubtree(subtree);
    EXPECT_FALSE(database.isSubtree(subtree));
}

TEST(Contracts, OnlyMountableNodesAcceptMounts)
{
    SpatialDatabase database;
    SubtreeHandle parent = database.registerSubtree(
        makeLeafSubtree(10));
    SubtreeHandle child = database.registerSubtree(
        makeLeafSubtree(20));
    instantiateFor(database, parent, box(2));
    EXPECT_THROW(database.mountSubtree(handleOf(database, 10), child),
                 std::logic_error);

    InstanceHandle flatRoot = database.instantiate(node(30, 4.0f, box(2)));
    EXPECT_THROW(database.mountSubtree(flatRoot.rootNode(), child),
                 std::logic_error);
}

TEST(Contracts, FixedOutputReportsOverflow)
{
    SpatialDatabase database;
    for (uint32_t i = 0; i < 8; ++i)
        database.instantiate(node(i + 1, 0.0f, box()));
    database.applyUpdates();

    std::array<FrontierEntry, 3> shared{};
    FrontierResultSink sink{Sink<FrontierEntry>{shared},
                            Sink<FrontierEntry>{{}},
                            Sink<FrontierEntry>{{}}};
    SpatialQuery query;
    query.selectFrontier(database, cameraAt(), {}, sink);
    EXPECT_EQ(sink.shared.count(), shared.size());
    EXPECT_EQ(sink.shared.dropped(), 5u);
}

TEST(Contracts, FixedOutputGeneratedRangeReportsOverflow)
{
    std::array<uint32_t, 3> output{};
    Sink<uint32_t> sink{output};
    sink.push(7);
    sink.pushGenerated(4, [](uint32_t i) { return 100 + i; });

    EXPECT_EQ(output, (std::array<uint32_t, 3>{7, 100, 101}));
    EXPECT_EQ(sink.count(), output.size());
    EXPECT_EQ(sink.dropped(), 2u);
}

TEST(Contracts, EncodedFrontierErrorPreservesTheExactThresholdSide)
{
    constexpr float threshold = 4.0f;
    const float below = std::nextafter(threshold, 0.0f);
    const float above = std::nextafter(
        threshold, std::numeric_limits<float>::infinity());

    EXPECT_EQ(encodeFrontierError(0.0f, threshold), 0u);
    EXPECT_LT(encodeFrontierError(below, threshold),
              kFrontierErrorThreshold);
    EXPECT_EQ(encodeFrontierError(threshold, threshold),
              kFrontierErrorThreshold - 1);
    EXPECT_GE(encodeFrontierError(above, threshold),
              kFrontierErrorThreshold);
    EXPECT_EQ(encodeFrontierError(
                  std::numeric_limits<float>::infinity(), threshold),
              255u);
    EXPECT_FLOAT_EQ(decodeFrontierError(127, threshold), threshold);
    EXPECT_FLOAT_EQ(decodeFrontierError(128, threshold), threshold);

    float previous = decodeFrontierError(0, threshold);
    for (uint32_t code = 1; code < 256; ++code)
    {
        const float decoded = decodeFrontierError(uint8_t(code), threshold);
        EXPECT_LE(previous, decoded) << "code " << code;
        previous = decoded;
    }
}

TEST(Contracts, FrontierCutViewsJoinBucketsWithoutCopying)
{
    static_assert(std::forward_iterator<FrontierCutView::iterator>);

    const std::array<FrontierEntry, 2> shared{
        FrontierEntry{{}, uint8_t(0), 1},
        FrontierEntry{{}, uint8_t(0), 2}};
    const std::array<FrontierEntry, 1> currentOnly{
        FrontierEntry{{}, uint8_t(0), 3}};
    const std::array<FrontierEntry, 2> idealOnly{
        FrontierEntry{{}, uint8_t(0), 4},
        FrontierEntry{{}, uint8_t(0), 5}};
    const FrontierResultView result{shared, currentOnly, idealOnly};

    const FrontierCutView current = result.current();
    ASSERT_EQ(current.size(), 3u);
    auto it = current.begin();
    EXPECT_EQ(&*it++, &shared[0]);
    EXPECT_EQ(&*it++, &shared[1]);
    EXPECT_EQ(&*it++, &currentOnly[0]);
    EXPECT_EQ(it, current.end());

    std::vector<InstanceId> idealInstances;
    for (const FrontierEntry& entry : result.ideal())
        idealInstances.push_back(entry.instance());
    EXPECT_EQ(idealInstances,
              (std::vector<InstanceId>{1, 2, 4, 5}));

    const FrontierResultView suffixOnly{{}, currentOnly, {}};
    ASSERT_FALSE(suffixOnly.current().empty());
    EXPECT_EQ(suffixOnly.current().begin()->instance(), 3u);
    EXPECT_TRUE(FrontierResultView{}.current().empty());
    EXPECT_EQ(FrontierResultView{}.ideal().begin(),
              FrontierResultView{}.ideal().end());
}

TEST(Contracts, OwningFrontierResultsRemainSelfContainedAfterCopyAndMove)
{
    SpatialDatabase database;
    database.instantiate(node(1, 0.0f, box()));
    database.instantiate(node(2, 0.0f, box()));
    database.applyUpdates();

    SpatialQuery query;
    FrontierResult original;
    query.selectFrontier(database, cameraAt(), {}, original);
    ASSERT_EQ(original.shared.size(), 2u);

    FrontierResult copy = original;
    EXPECT_EQ(payloads(database, copy),
              (std::vector<UserPayload>{1, 2}));
    EXPECT_NE(copy.shared.data(), original.shared.data());

    FrontierResult assigned;
    assigned = original;
    EXPECT_EQ(payloads(database, assigned),
              (std::vector<UserPayload>{1, 2}));
    EXPECT_NE(assigned.shared.data(), original.shared.data());

    FrontierResult moved = std::move(copy);
    EXPECT_TRUE(copy.empty());
    EXPECT_EQ(payloads(database, moved),
              (std::vector<UserPayload>{1, 2}));

    FrontierResult moveAssigned;
    moveAssigned = std::move(assigned);
    EXPECT_TRUE(assigned.empty());
    EXPECT_EQ(payloads(database, moveAssigned),
              (std::vector<UserPayload>{1, 2}));
}

TEST(Contracts, MovedSpatialQueryRetainsItsBindingAndAllocations)
{
    SpatialDatabase database;
    const SubtreeHandle subtree =
        database.registerSubtree(makeLeafSubtree(2));
    instantiateFor(database, subtree, box(), 64.0f);
    database.applyUpdates();
    const Camera camera = cameraAt(-100.0f);

    SpatialQuery source;
    (void)source.selectFrontier(database, camera, {});
    SpatialQuery moved(std::move(source));
    const FrontierResultView movedResult =
        moved.selectFrontier(database, camera, {});
    EXPECT_EQ(movedResult.idealSize(), 1u);
    EXPECT_EQ(moved.reused(), 1u);

    SpatialQuery assigned;
    assigned = std::move(moved);
    const FrontierResultView assignedResult =
        assigned.selectFrontier(database, camera, {});
    EXPECT_EQ(assignedResult.idealSize(), 1u);
    EXPECT_EQ(assigned.reused(), 1u);
}

TEST(Contracts, SelectionStatisticsMatchTheBuildMode)
{
    SpatialDatabase database;
    database.instantiate(node(1, 0.0f, box()));
    database.applyUpdates();
    SpatialQuery query;
    query.setReuseEnabled(false);
    (void)query.selectFrontier(database, cameraAt(), {});

    const SelectionStats& stats = query.lastSelectionStats();
#ifdef FRONTIER_STATS
    EXPECT_EQ(stats.instancesVisited, 1u);
#else
    EXPECT_EQ(stats.instancesVisited, 0u);
#endif
}

TEST(Contracts, HotTypesStayCompact)
{
    EXPECT_EQ(sizeof(NodeHandle), 8u);
    EXPECT_EQ(sizeof(InstanceHandle), 8u);
    EXPECT_EQ(sizeof(SubtreeHandle), 8u);
    EXPECT_EQ(sizeof(SubtreeInstanceHandle), 8u);
#if !defined(FRONTIER_STATS) && defined(NDEBUG)
    if constexpr (sizeof(void*) == 8)
        EXPECT_EQ(sizeof(SpatialQuery), 304u);
#endif
    EXPECT_EQ(sizeof(FrontierEntry), 12u);
    EXPECT_EQ(sizeof(float8), size_t(kWide) * sizeof(float));
    EXPECT_EQ(sizeof(WideBounds), size_t(kWide) * 6 * sizeof(float));
    EXPECT_EQ(sizeof(detail::WideBlock), size_t(kWide) * 8 * sizeof(float));
    EXPECT_EQ(TestAccess::tlasNodeBytes(), kWide == 8 ? 320u : 160u);
    EXPECT_LE(TestAccess::definitionBytes(), 160u);
    EXPECT_LE(TestAccess::mountedStateBytes(), 64u);
    EXPECT_EQ(TestAccess::mountStampBytes(), 8u);
    EXPECT_EQ(TestAccess::mountReadinessBytes(), 4u);
    EXPECT_LE(TestAccess::instanceBytes(), 80u);
}

TEST(Contracts, TlasRootsArePermanentlyReady)
{
    SpatialDatabase database;
    const InstanceHandle first =
        database.instantiate(node(91, 0.0f, box()));
    const InstanceHandle second =
        database.instantiate(node(91, 0.0f, box()));
    EXPECT_TRUE(database.isNodeReady(first.rootNode()));
    EXPECT_TRUE(database.isNodeReady(second.rootNode()));
    database.markNodeReady(first.rootNode());
    EXPECT_THROW(database.markNodeUnavailable(first.rootNode()),
                 std::logic_error);

    database.removeInstance(first);
    EXPECT_FALSE(database.isNodeReady(first.rootNode()));
    database.markNodeUnavailable(first.rootNode());
    EXPECT_THROW(database.markNodeUnavailable(second.rootNode()),
                 std::logic_error);
    database.removeInstance(second);
    EXPECT_FALSE(database.isNodeReady(second.rootNode()));
    database.markNodeUnavailable(second.rootNode());
}
