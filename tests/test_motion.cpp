#include <gtest/gtest.h>

#include <array>
#include <limits>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

TEST(Motion, MovesTlasOwnedNodeWithoutSubtreeState)
{
    SpatialDatabase database;
    InstanceHandle instance =
        database.instantiate(node(7, 0.0f, box(1.0f)));
    database.moveInstance(
        instance, Transform{float4::point(100, 2, 3), 2.0f});
    database.applyUpdates(0);

    const AABB world = TestAccess::instanceBounds(database, instance);
    EXPECT_TRUE(world.contains(box(2.0f, float4::point(100, 2, 3))));
    EXPECT_EQ(database.mountedSubtreeCount(), 0u);
}

TEST(Motion, YawPreservesExactLocalBoundsAcrossRotationAndDeformation)
{
    SpatialDatabase database;
    const AABB authored = AABB::fromMinMax(
        float4::point(-1.0f, -0.5f, -3.0f),
        float4::point(2.0f, 0.5f, 4.0f));
    InstanceDesc desc;
    desc.pos = float4::point(10.0f, 1.0f, 20.0f);
    desc.scale = 2.0f;
    desc.yaw = {0.0f, 1.0f};
    const InstanceHandle instance =
        database.instantiate(node(7, 0.0f, authored), desc);

    const AABB rotated = TestAccess::instanceBounds(database, instance);
    EXPECT_FLOAT_EQ(rotated.mn.x, 2.0f);
    EXPECT_FLOAT_EQ(rotated.mx.x, 16.0f);
    EXPECT_FLOAT_EQ(rotated.mn.z, 18.0f);
    EXPECT_FLOAT_EQ(rotated.mx.z, 24.0f);
    const AABB recovered = database.nodeBounds(instance, instance.rootNode());
    EXPECT_FLOAT_EQ(recovered.mn.x, authored.mn.x);
    EXPECT_FLOAT_EQ(recovered.mx.z, authored.mx.z);

    const AABB deformed = AABB::fromMinMax(
        float4::point(-2.0f, -1.0f, -5.0f),
        float4::point(3.0f, 1.0f, 6.0f));
    database.setNodeBounds(instance, instance.rootNode(), deformed);
    database.flushBounds();
    database.moveInstance(
        instance,
        InstanceTransform{float4::point(30.0f, 2.0f, 40.0f), 1.5f,
                          YawRotation{}});

    const AABB unrotated = TestAccess::instanceBounds(database, instance);
    EXPECT_FLOAT_EQ(unrotated.mn.x, 27.0f);
    EXPECT_FLOAT_EQ(unrotated.mx.x, 34.5f);
    EXPECT_FLOAT_EQ(unrotated.mn.z, 32.5f);
    EXPECT_FLOAT_EQ(unrotated.mx.z, 49.0f);
    const AABB finalLocal = database.nodeBounds(instance, instance.rootNode());
    EXPECT_FLOAT_EQ(finalLocal.mn.x, deformed.mn.x);
    EXPECT_FLOAT_EQ(finalLocal.mx.z, deformed.mx.z);
}

TEST(Motion, MotionGroupSubmitsIndependentYawTransforms)
{
    SpatialDatabase database;
    const AABB authored = AABB::fromMinMax(
        float4::point(-1.0f, -1.0f, -3.0f),
        float4::point(1.0f, 1.0f, 3.0f));
    const InstanceHandle first = database.instantiate(node(1, 0.0f, authored));
    const InstanceHandle second = database.instantiate(node(2, 0.0f, authored));
    const std::array<InstanceHandle, 2> handles{first, second};
    SpatialDatabase::MotionGroup group(handles);
    const std::array<InstanceTransform, 2> transforms{
        InstanceTransform{float4::point(10.0f, 0.0f, 0.0f), 1.0f,
                          YawRotation{0.0f, 1.0f}},
        InstanceTransform{float4::point(20.0f, 0.0f, 0.0f), 2.0f,
                          YawRotation{1.0f, 0.0f}}};
    database.moveInstances(group, transforms);

    const AABB firstBounds = TestAccess::instanceBounds(database, first);
    EXPECT_FLOAT_EQ(firstBounds.mn.x, 7.0f);
    EXPECT_FLOAT_EQ(firstBounds.mx.x, 13.0f);
    EXPECT_FLOAT_EQ(firstBounds.mn.z, -1.0f);
    EXPECT_FLOAT_EQ(firstBounds.mx.z, 1.0f);
    const AABB secondBounds = TestAccess::instanceBounds(database, second);
    EXPECT_FLOAT_EQ(secondBounds.mn.x, 18.0f);
    EXPECT_FLOAT_EQ(secondBounds.mx.x, 22.0f);
    EXPECT_FLOAT_EQ(secondBounds.mn.z, -6.0f);
    EXPECT_FLOAT_EQ(secondBounds.mx.z, 6.0f);
}

TEST(Motion, LargeMotionBatchPublishesOneExactTlasRefit)
{
    constexpr uint32_t count = 64;
    SpatialDatabase database;
    std::vector<InstanceHandle> handles;
    std::vector<InstanceTransform> transforms(count);
    handles.reserve(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(i % 8) * 4.0f, 0.0f,
                                 float(i / 8) * 4.0f);
        handles.push_back(database.instantiate(
            node(1000 + i, 0.0f, box()), desc));
        transforms[i].pos = float4::point(
            200.0f - float(i % 8) * 5.0f, float(i & 1u),
            -100.0f + float(i / 8) * 6.0f);
    }
    database.applyUpdates(0);
    database.optimize();

    SpatialDatabase::MotionGroup group(handles);
    database.moveInstances(group, transforms);
    database.applyUpdates(0);

    for (const InstanceHandle handle : handles)
    {
        const AABB exact = TestAccess::instanceBounds(database, handle);
        const AABB leaf = TestAccess::tlasLeafBounds(database, handle);
        EXPECT_FLOAT_EQ(leaf.mn.x, exact.mn.x);
        EXPECT_FLOAT_EQ(leaf.mn.y, exact.mn.y);
        EXPECT_FLOAT_EQ(leaf.mn.z, exact.mn.z);
        EXPECT_FLOAT_EQ(leaf.mx.x, exact.mx.x);
        EXPECT_FLOAT_EQ(leaf.mx.y, exact.mx.y);
        EXPECT_FLOAT_EQ(leaf.mx.z, exact.mx.z);
        EXPECT_FALSE(TestAccess::tlasLeafIsLoose(database, handle));
    }
}

TEST(Motion, IdentityOnlySceneKeepsOrientationStreamUnallocated)
{
    SpatialDatabase database;
    const InstanceHandle instance =
        database.instantiate(node(1, 0.0f, box()));
    EXPECT_EQ(database.instanceOrientationStateBytes(), 0u);
    database.moveInstance(
        instance, Transform{float4::point(5.0f, 0.0f, 0.0f), 1.0f});
    EXPECT_EQ(database.instanceOrientationStateBytes(), 0u);

    database.moveInstance(
        instance,
        InstanceTransform{float4::point(5.0f, 0.0f, 0.0f), 1.0f,
                          YawRotation{0.0f, 1.0f}});
    EXPECT_GT(database.instanceOrientationStateBytes(), 0u);
    EXPECT_EQ(TestAccess::instanceBytes(), 80u);
}

TEST(Motion, AuthoredYawInvariantRootKeepsOneBroadphaseEnvelope)
{
    SpatialDatabase database;
    NodeDesc root = node(1, 0.0f, box(5.0f));
    root.flags |= NodeDesc::FlagYawInvariantBounds;
    InstanceDesc desc;
    desc.pos = float4::point(10.0f, 0.0f, 20.0f);
    desc.yaw = {0.0f, 1.0f};
    const InstanceHandle instance = database.instantiate(root, desc);

    AABB bounds = TestAccess::instanceBounds(database, instance);
    EXPECT_FLOAT_EQ(bounds.mn.x, 5.0f);
    EXPECT_FLOAT_EQ(bounds.mx.x, 15.0f);
    EXPECT_FLOAT_EQ(bounds.mn.z, 15.0f);
    EXPECT_FLOAT_EQ(bounds.mx.z, 25.0f);

    database.moveInstance(
        instance,
        InstanceTransform{float4::point(30.0f, 0.0f, 40.0f), 1.0f,
                          yawRotation(0.75f)});
    bounds = TestAccess::instanceBounds(database, instance);
    EXPECT_FLOAT_EQ(bounds.mn.x, 25.0f);
    EXPECT_FLOAT_EQ(bounds.mx.x, 35.0f);
    EXPECT_FLOAT_EQ(bounds.mn.z, 35.0f);
    EXPECT_FLOAT_EQ(bounds.mx.z, 45.0f);
}

TEST(Motion, RigidMotionGroupStreamsYawInvariantActors)
{
    SpatialDatabase database;
    NodeDesc root = node(1, 8.0f, box(5.0f));
    root.flags |= NodeDesc::FlagYawInvariantBounds;
    InstanceDesc firstDesc;
    firstDesc.pos = float4::point(1.0f, 0.0f, 2.0f);
    InstanceDesc secondDesc;
    secondDesc.pos = float4::point(4.0f, 0.0f, 8.0f);
    const InstanceHandle first = database.instantiate(root, firstDesc);
    const InstanceHandle second = database.instantiate(root, secondDesc);
    const std::array<InstanceHandle, 2> handles{first, second};
    SpatialDatabase::RigidMotionGroup group(handles);
    const std::array<float4, 2> positions{
        float4::point(11.0f, 0.0f, 12.0f),
        float4::point(24.0f, 0.0f, 28.0f)};
    const std::array<YawRotation, 2> yaws{
        YawRotation{0.0f, 1.0f}, YawRotation{-1.0f, 0.0f}};

    database.moveRigidInstances(group, positions, yaws);
    database.applyUpdates(0);

    const AABB firstBounds = TestAccess::instanceBounds(database, first);
    const AABB secondBounds = TestAccess::instanceBounds(database, second);
    EXPECT_FLOAT_EQ(firstBounds.center().x, 11.0f);
    EXPECT_FLOAT_EQ(firstBounds.center().z, 12.0f);
    EXPECT_FLOAT_EQ(secondBounds.center().x, 24.0f);
    EXPECT_FLOAT_EQ(secondBounds.center().z, 28.0f);
    EXPECT_FLOAT_EQ(TestAccess::instanceYaw(database, first).cosine, 0.0f);
    EXPECT_FLOAT_EQ(TestAccess::instanceYaw(database, first).sine, 1.0f);
    EXPECT_FLOAT_EQ(TestAccess::instanceYaw(database, second).cosine, -1.0f);
    EXPECT_FLOAT_EQ(TestAccess::instanceYaw(database, second).sine, 0.0f);
    EXPECT_EQ(TestAccess::instanceBytes(), 80u);
}

TEST(Motion, RigidMotionGroupFallsBackForOrientedBounds)
{
    SpatialDatabase database;
    const AABB authored = AABB::fromMinMax(
        float4::point(-1.0f, -1.0f, -3.0f),
        float4::point(1.0f, 1.0f, 3.0f));
    const InstanceHandle instance =
        database.instantiate(node(1, 0.0f, authored));
    const std::array<InstanceHandle, 1> handles{instance};
    SpatialDatabase::RigidMotionGroup group(handles);
    const std::array<float4, 1> positions{
        float4::point(10.0f, 0.0f, 20.0f)};
    const std::array<YawRotation, 1> yaws{YawRotation{0.0f, 1.0f}};

    database.moveRigidInstances(group, positions, yaws);
    database.applyUpdates(0);

    const AABB bounds = TestAccess::instanceBounds(database, instance);
    EXPECT_FLOAT_EQ(bounds.mn.x, 7.0f);
    EXPECT_FLOAT_EQ(bounds.mx.x, 13.0f);
    EXPECT_FLOAT_EQ(bounds.mn.z, 19.0f);
    EXPECT_FLOAT_EQ(bounds.mx.z, 21.0f);
}

TEST(Motion, NodeBoundsUseCopyOnWritePerTopLevelInstance)
{
    SpatialDatabase database;
    SubtreeHandle subtree = database.registerSubtree(makeLodSubtree());
    InstanceHandle first = instantiateFor(database, subtree, box(5.0f));
    InstanceDesc shifted;
    shifted.pos = float4::point(100, 0, 0);
    InstanceHandle second = instantiateFor(database, subtree, box(5.0f),
                                           64.0f, shifted);

    NodeHandle firstLeaf = TestAccess::requireNode(database, first, 11);
    NodeHandle secondLeaf = TestAccess::requireNode(database, second, 11);
    const AABB authored = database.nodeBounds(second, secondLeaf);
    const AABB moved = box(1.0f, float4::point(20, 0, 0));
    database.setNodeBounds(first, firstLeaf, moved);
    database.flushBounds();

    EXPECT_TRUE(database.nodeBounds(first, firstLeaf).contains(moved));
    EXPECT_TRUE(database.nodeBounds(second, secondLeaf).contains(authored));
    EXPECT_FALSE(database.nodeBounds(second, secondLeaf).contains(moved));
    EXPECT_GT(database.overlayCount(), 0u);
}

TEST(Motion, LargeBoundsOverlayPromotesFromSparseToDense)
{
    SpatialDatabase database;
    SubtreeBuilder builder;
    constexpr uint32_t count = detail::kMaxChildren;
    for (uint32_t i = 0; i < count; ++i)
        builder.createNode(node(1000 + i, 0.0f, box()));
    const SubtreeHandle definition =
        database.registerSubtree(builder.build());
    const InstanceHandle instance = database.instantiate(
        node(1, 16.0f, box(), true));
    const SubtreeInstanceHandle placement =
        database.mountSubtree(instance.rootNode(), definition);
    ASSERT_TRUE(placement.valid());

    const NodeHandle first = TestAccess::nodeAt(database, placement, 1);
    database.setNodeBounds(instance, first, box(0.75f));
    database.flushBounds();
    EXPECT_TRUE(TestAccess::overlayIsSparse(database, instance, first));

    SpatialQuery query;
    const FrontierResultView sparseResult =
        select(database, query, cameraAt(-1000.0f));
    EXPECT_EQ(sparseResult.idealSize(), count);

    // Cross the one-sixteenth promotion threshold. Block zero was patched
    // above; BVH4 has twice as many half-sized blocks as BVH8.
    const uint32_t wideBlocks = (count + kWide - 1) / kWide;
    const uint32_t patchesToPromote =
        wideBlocks / 16 + 1;
    for (uint32_t block = 1; block < patchesToPromote; ++block)
        database.setNodeBounds(
            instance,
            TestAccess::nodeAt(database, placement, 1 + block * kWide),
            box(0.75f));
    database.flushBounds();
    EXPECT_FALSE(TestAccess::overlayIsSparse(database, instance, first));

    const FrontierResultView denseResult =
        select(database, query, cameraAt(-1000.0f));
    EXPECT_EQ(denseResult.idealSize(), count);
}

TEST(Motion, MountedTransformsCompose)
{
    SpatialDatabase database;
    SubtreeHandle detail = database.registerSubtree(
        makeLeafSubtree(30));
    SubtreeBuilder parentBuilder;
    parentBuilder.createNode(node(
        20, 8.0f, box(2.0f, float4::point(5, 0, 0)), true));
    SubtreeHandle parent = database.registerSubtree(parentBuilder.build());

    Transform parentTransform{float4::point(10, 0, 0), 3.0f};
    instantiateFor(database, parent, box(40.0f), 64.0f, {},
                   parentTransform);
    database.mountSubtree(handleOf(database, 20), detail,
        Transform{float4::point(5, 0, 0), 2.0f});

    Transform transform;
    ASSERT_TRUE(database.tryGetNodeTransform(handleOf(database, 30), transform));
    EXPECT_FLOAT_EQ(transform.scale, 6.0f);
    EXPECT_FLOAT_EQ(transform.pos.x, 25.0f);
}

TEST(Motion, RejectsUnrepresentableAccumulatedMountTransform)
{
    SpatialDatabase database;

    SubtreeBuilder detailBuilder;
    detailBuilder.createNode(node(30, 0.0f, box(1.0e-20f)));
    const SubtreeHandle detail =
        database.registerSubtree(detailBuilder.build());

    SubtreeBuilder ownerBuilder;
    ownerBuilder.createNode(node(20, 8.0f, box(1.0f), true));
    const SubtreeHandle owner =
        database.registerSubtree(ownerBuilder.build());

    const InstanceHandle root = database.instantiate(
        node(1, 64.0f, box(1.0e30f), true));
    ASSERT_TRUE(database.mountSubtree(
        root.rootNode(), owner,
        Transform{float4::point(0, 0, 0), 1.0e30f}).valid());

    EXPECT_THROW(database.mountSubtree(
                     handleOf(database, 20), detail,
                     Transform{float4::point(0, 0, 0), 1.0e10f}),
                 std::logic_error);
    EXPECT_EQ(database.mountedSubtreeCount(), 1u);
}

TEST(Motion, MotionGroupIgnoresStaleHandles)
{
    SpatialDatabase database;
    InstanceHandle a = database.instantiate(node(1, 0.0f, box()));
    InstanceHandle b = database.instantiate(node(2, 0.0f, box()));
    std::array<InstanceHandle, 2> handles{a, b};
    SpatialDatabase::MotionGroup group(handles);
    database.removeInstance(a);
    std::array<float4, 2> positions{
        float4::point(100, 0, 0), float4::point(20, 0, 0)};
    database.moveInstances(group, positions);
    database.applyUpdates(0);

    const AABB moved = TestAccess::instanceBounds(database, b);
    EXPECT_NEAR(moved.center().x, 20.0f, 0.001f);
}

TEST(Motion, CachedMotionGroupRefreshesBeforeDenseSlotReuse)
{
    SpatialDatabase database;
    const InstanceHandle old = database.instantiate(node(1, 0.0f, box()));
    const InstanceHandle survivor =
        database.instantiate(node(2, 0.0f, box()));
    const std::array<InstanceHandle, 2> handles{old, survivor};
    SpatialDatabase::MotionGroup group(handles);

    const std::array<float4, 2> firstPositions{
        float4::point(10, 0, 0), float4::point(20, 0, 0)};
    database.moveInstances(group, firstPositions);

    database.removeInstance(old);
    const InstanceHandle replacement = database.instantiate(
        node(3, 0.0f, box()), InstanceDesc{.pos = float4::point(7, 0, 0)});
    const std::array<float4, 2> secondPositions{
        float4::point(100, 0, 0), float4::point(30, 0, 0)};
    database.moveInstances(group, secondPositions);
    database.applyUpdates(0);

    EXPECT_NEAR(TestAccess::instanceBounds(database, survivor).center().x,
                30.0f, 0.001f);
    EXPECT_NEAR(TestAccess::instanceBounds(database, replacement).center().x,
                7.0f, 0.001f);
}

TEST(Motion, MotionGroupDuplicateHandlesUseTheFinalCallerPosition)
{
    SpatialDatabase database;
    const InstanceHandle instance =
        database.instantiate(node(1, 0.0f, box()));
    const std::array<InstanceHandle, 3> handles{
        instance, instance, instance};
    SpatialDatabase::MotionGroup group(handles);
    const std::array<float4, 3> positions{
        float4::point(10, 0, 0),
        float4::point(20, 0, 0),
        float4::point(30, 0, 0)};

    database.moveInstances(group, positions);
    database.applyUpdates(0);
    EXPECT_NEAR(TestAccess::instanceBounds(database, instance).center().x,
                30.0f, 0.001f);
}

TEST(Motion, MotionGroupResetReplacesTheCallerCohort)
{
    SpatialDatabase database;
    const InstanceHandle first =
        database.instantiate(node(1, 0.0f, box()));
    const InstanceHandle second =
        database.instantiate(node(2, 0.0f, box()));

    const std::array<InstanceHandle, 1> firstCohort{first};
    SpatialDatabase::MotionGroup group(firstCohort);
    const std::array<float4, 1> firstPosition{
        float4::point(10, 0, 0)};
    database.moveInstances(group, firstPosition);

    const std::array<InstanceHandle, 1> secondCohort{second};
    group.reset(secondCohort);
    EXPECT_EQ(group.size(), 1u);
    const std::array<float4, 1> secondPosition{
        float4::point(20, 0, 0)};
    database.moveInstances(group, secondPosition);
    database.applyUpdates(0);

    EXPECT_NEAR(TestAccess::instanceBounds(database, first).center().x,
                10.0f, 0.001f);
    EXPECT_NEAR(TestAccess::instanceBounds(database, second).center().x,
                20.0f, 0.001f);
}

TEST(Motion, RigidPopulationTranslationMaterializesBeforeDifferentialEdits)
{
    SpatialDatabase database;
    const InstanceHandle first =
        database.instantiate(node(1, 0.0f, box()));
    const InstanceHandle second = database.instantiate(
        node(2, 0.0f, box()),
        InstanceDesc{.pos = float4::point(10, 0, 0)});
    database.applyUpdates(0);

    const std::array<InstanceHandle, 2> handles{first, second};
    SpatialDatabase::MotionGroup all(handles);
    database.translateInstances(all, float4::vec(100, 0, 0));
    database.applyUpdates(0);

    EXPECT_NEAR(TestAccess::instanceBounds(database, first).center().x,
                100.0f, 0.001f);
    EXPECT_NEAR(TestAccess::instanceBounds(database, second).center().x,
                110.0f, 0.001f);

    database.moveInstance(
        first, Transform{float4::point(200, 0, 0), 1.0f});
    const InstanceHandle added = database.instantiate(
        node(3, 0.0f, box()),
        InstanceDesc{.pos = float4::point(7, 0, 0)});
    database.applyUpdates(0);

    EXPECT_NEAR(TestAccess::instanceBounds(database, first).center().x,
                200.0f, 0.001f);
    EXPECT_NEAR(TestAccess::instanceBounds(database, second).center().x,
                110.0f, 0.001f);
    EXPECT_NEAR(TestAccess::instanceBounds(database, added).center().x,
                7.0f, 0.001f);
}

TEST(Motion, SweptTlasLeafRetestsItsExactCurrentBounds)
{
    SpatialDatabase database;
    const InstanceHandle moving =
        database.instantiate(node(1, 0.0f, box()));
    database.instantiate(
        node(2, 0.0f, box()),
        InstanceDesc{.pos = float4::point(3, 0, 0)});
    database.applyUpdates(0);

    const std::array<InstanceHandle, 1> handles{moving};
    SpatialDatabase::MotionGroup group(handles);
    SpatialQuery query;

    database.translateInstances(group, float4::vec(100, 0, 0));
    EXPECT_EQ(payloads(database, select(database, query, cameraAt(-20.0f))),
              (std::vector<UserPayload>{2}));

    database.translateInstances(group, float4::vec(-100, 0, 0));
    EXPECT_EQ(payloads(database, select(database, query, cameraAt(-20.0f))),
              (std::vector<UserPayload>{1, 2}));
}

TEST(Motion, ContributionCullRetestsExactBoundsAfterScaleChanges)
{
    SpatialDatabase database;
    const InstanceHandle scaled =
        database.instantiate(node(1, 0.0f, box(10.0f)));
    for (uint32_t i = 0; i < 7; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(i + 1) * 4.0f, 0.0f, 0.0f);
        database.instantiate(node(2 + i, 0.0f, box(0.1f)), desc);
    }
    database.applyUpdates(0);

    SpatialQuery query;
    SelectionParams params;
    params.minPix = 10.0f;
    const Camera camera = cameraAt(-1000.0f);
    EXPECT_EQ(payloads(database, query.selectFrontier(database, camera, params)),
              (std::vector<UserPayload>{1}));

    database.moveInstance(
        scaled, Transform{float4::point(0.0f, 0.0f, 0.0f), 0.01f});
    database.applyUpdates(0);
    EXPECT_TRUE(query.selectFrontier(database, camera, params).empty());

#ifdef FRONTIER_DEBUG_TOOLS
    const TlasDebugSummary summary = database.debugTlasSummary();
    EXPECT_EQ(summary.looseInstanceCount, 1u);
    std::array<LooseInstanceDebugBounds, 1> looseBounds{};
    ASSERT_EQ(database.debugLooseInstanceBounds(looseBounds), 1u);
    EXPECT_EQ(looseBounds[0].instance, scaled);
    EXPECT_TRUE(looseBounds[0].envelope.contains(looseBounds[0].exact));
    EXPECT_GT(looseBounds[0].envelope.extent().x,
              looseBounds[0].exact.extent().x);
#endif

    database.moveInstance(
        scaled, Transform{float4::point(0.0f, 0.0f, 0.0f), 1.0f});
    database.applyUpdates(0);
    EXPECT_EQ(payloads(database, query.selectFrontier(database, camera, params)),
              (std::vector<UserPayload>{1}));
}

TEST(Motion, ApplyUpdatesRepairsOnlyTheExplicitNodeBudget)
{
    SpatialDatabaseConfig config;
    config.tlasAreaDrift = 1000.0f;
    config.tlasCountDrift = 1000.0f;
    config.tlasEditFraction = 1000.0f;
    SpatialDatabase database(config);
    std::vector<InstanceHandle> handles;
    for (uint32_t i = 0; i < 32; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(i % 8) * 4.0f, 0.0f,
                                 float(i / 8) * 4.0f);
        handles.push_back(database.instantiate(
            node(1000 + i, 0.0f, box()), desc));
    }
    EXPECT_TRUE(database.applyUpdates(0).requiredBuildPerformed);

    const InstanceHandle moving = handles.front();
    database.moveInstance(
        moving, Transform{float4::point(1000.0f, 0.0f, 0.0f), 1.0f});
    const UpdateReport deferred = database.applyUpdates(0);
    EXPECT_EQ(deferred.maintenanceNodesProcessed, 0u);
    EXPECT_EQ(deferred.maintenanceNodesPending, 1u);
    EXPECT_FALSE(deferred.requiredBuildPerformed);
    EXPECT_TRUE(TestAccess::tlasLeafIsLoose(database, moving));
    EXPECT_TRUE(TestAccess::tlasLeafBounds(database, moving).contains(
        TestAccess::instanceBounds(database, moving)));

    const UpdateReport oneNode = database.applyUpdates(1);
    EXPECT_EQ(oneNode.maintenanceNodesProcessed, 1u);
    EXPECT_GT(oneNode.maintenanceNodesPending, 0u);
    EXPECT_FALSE(TestAccess::tlasLeafIsLoose(database, moving));
    const AABB exact = TestAccess::instanceBounds(database, moving);
    const AABB leaf = TestAccess::tlasLeafBounds(database, moving);
    EXPECT_FLOAT_EQ(leaf.mn.x, exact.mn.x);
    EXPECT_FLOAT_EQ(leaf.mx.x, exact.mx.x);

    const UpdateReport drained =
        database.applyUpdates(kUnlimitedTlasMaintenance);
    EXPECT_GT(drained.maintenanceNodesProcessed, 0u);
    EXPECT_EQ(drained.maintenanceNodesPending, 0u);
}

TEST(Motion, TighteningReducesCurrentAreaWithoutImplicitRebuild)
{
    SpatialDatabaseConfig config;
    config.tlasAreaDrift = 0.0f;
    config.tlasCountDrift = 1000.0f;
    config.tlasEditFraction = 1000.0f;
    SpatialDatabase database(config);
    std::vector<InstanceHandle> handles;
    for (uint32_t i = 0; i < 32; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(i % 8) * 4.0f, 0.0f,
                                 float(i / 8) * 4.0f);
        handles.push_back(database.instantiate(
            node(2000 + i, 0.0f, box()), desc));
    }
    database.applyUpdates(0);

    const InstanceHandle moving = handles.front();
    database.moveInstance(
        moving, Transform{float4::point(1000.0f, 0.0f, 0.0f), 1.0f});
    const UpdateReport grown = database.applyUpdates(0);
    EXPECT_FALSE(grown.requiredBuildPerformed);
    EXPECT_TRUE(grown.topologyRebuildRecommended);
    EXPECT_GT(grown.areaGrowthRatio, 0.0f);

    database.moveInstance(
        moving, Transform{float4::point(0.0f, 0.0f, 0.0f), 1.0f});
    database.applyUpdates(0);
    const UpdateReport repaired =
        database.applyUpdates(kUnlimitedTlasMaintenance);
    EXPECT_FALSE(repaired.requiredBuildPerformed);
    EXPECT_EQ(repaired.maintenanceNodesPending, 0u);
    EXPECT_NEAR(repaired.areaGrowthRatio, 0.0f, 1.0e-6f);
    EXPECT_FALSE(repaired.topologyRebuildRecommended);
}

TEST(Motion, RejectsInvalidTransformsWithoutCorruptingTlas)
{
    SpatialDatabase database;
    NodeDesc invalidFlags = node(1, 0.0f, box());
    invalidFlags.flags = 1u << 31;
    EXPECT_THROW(database.instantiate(invalidFlags), std::logic_error);

    InstanceDesc invalidPosition;
    invalidPosition.pos.x = std::numeric_limits<float>::infinity();
    EXPECT_THROW(database.instantiate(node(2, 0.0f, box()), invalidPosition),
                 std::logic_error);

    InstanceDesc invalidYaw;
    invalidYaw.yaw = {1.0f, 1.0f};
    EXPECT_THROW(database.instantiate(node(2, 0.0f, box()), invalidYaw),
                 std::logic_error);

    InstanceHandle instance =
        database.instantiate(node(3, 0.0f, box()));
    const AABB before = TestAccess::instanceBounds(database, instance);
    Transform invalidMove;
    invalidMove.pos.x = std::numeric_limits<float>::infinity();
    EXPECT_THROW(database.moveInstance(instance, invalidMove),
                 std::logic_error);
    InstanceTransform invalidYawMove;
    invalidYawMove.yaw = {0.0f, 0.0f};
    EXPECT_THROW(database.moveInstance(instance, invalidYawMove),
                 std::logic_error);
    const AABB after = TestAccess::instanceBounds(database, instance);
    EXPECT_FLOAT_EQ(after.mn.x, before.mn.x);
    EXPECT_FLOAT_EQ(after.mx.x, before.mx.x);

    const float tooSmall = std::numeric_limits<float>::denorm_min();
    EXPECT_THROW(database.instantiate(
                     node(4, 0.0f, box()),
                     InstanceDesc{.scale = tooSmall}),
                 std::logic_error);
    EXPECT_THROW(database.moveInstance(
                     instance,
                     Transform{float4::point(0, 0, 0), tooSmall}),
                 std::logic_error);

    const SubtreeHandle detail =
        database.registerSubtree(makeLeafSubtree(5));
    const InstanceHandle mountable = database.instantiate(
        node(6, 8.0f, box(), true));
    EXPECT_THROW(database.mountSubtree(
                     mountable.rootNode(), detail,
                     Transform{float4::point(0, 0, 0), tooSmall}),
                 std::logic_error);
}

TEST(Motion, RejectsBoundsThatOverflowTheInstanceTransform)
{
    SpatialDatabase database;
    const InstanceHandle instance = database.instantiate(
        node(1, 0.0f, box(1.0e-10f)),
        InstanceDesc{.scale = 1.0e30f});
    const AABB before = TestAccess::instanceBounds(database, instance);

    database.setNodeBounds(instance, instance.rootNode(), box(1.0e20f));
    EXPECT_THROW(database.flushBounds(), std::logic_error);

    const AABB after =
        TestAccess::unflushedInstanceBounds(database, instance);
    EXPECT_FLOAT_EQ(after.mn.x, before.mn.x);
    EXPECT_FLOAT_EQ(after.mx.x, before.mx.x);
}

TEST(Motion, BoundsOverrideRequiresTheOwningInstance)
{
    SpatialDatabase database;
    const SubtreeHandle subtree =
        database.registerSubtree(makeLeafSubtree(10));
    const InstanceHandle first = instantiateFor(database, subtree, box(2.0f));
    const InstanceHandle second = instantiateFor(database, subtree, box(2.0f));
    const NodeHandle secondNode =
        TestAccess::requireNode(database, second, 10);

    EXPECT_THROW(database.setNodeBounds(first, secondNode, box(0.5f)),
                 std::logic_error);
    EXPECT_THROW(database.setNodeBounds(first, second.rootNode(), box(0.5f)),
                 std::logic_error);
    EXPECT_THROW(database.nodeBounds(first, secondNode), std::logic_error);
    EXPECT_EQ(database.overlayCount(), 0u);
}
