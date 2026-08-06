#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <set>
#include <tuple>

#include "helpers.h"

// Cross-cutting API contracts: cut invariants that must hold on any world,
// determinism, multi-view damper isolation, instance-slot reuse (ABA),
// geometric edge cases, and memory budgets.

using namespace hlod;
using namespace hlodtest;
using TA = World::TestAccess;

namespace {

struct Outputs
{
    CutResults cut;
};

Outputs run(World& w, const Camera& v, const CutParams& p)
{
    Outputs o;
    selectCutUncached(w, v, p, o.cut);
    return o;
}

std::set<UserId> cutIds(World& world, const CutResults& cut)
{
    std::set<UserId> s;
    for (const auto& e : currentCut(cut)) s.insert(payloadOf(world, e));
    return s;
}

using ResultKey = std::tuple<uint32_t, UserPayload, uint8_t, InstanceId,
                             uint32_t, uint32_t>;

std::vector<ResultKey> resultKeys(World& world, const CutResults& cut)
{
    std::vector<ResultKey> keys;
    keys.reserve(cut.size());
    const auto append = [&](const std::vector<CutEntry>& entries, uint32_t bucket)
    {
        for (const CutEntry& e : entries)
            keys.emplace_back(bucket, payloadOf(world, e), e.errorCode(),
                              e.instance(), e.nodeHandle.lo, e.nodeHandle.hi);
    };
    append(cut.shared, 0);
    append(cut.currentOnly, 1);
    append(cut.idealOnly, 2);
    return keys;
}

// Deterministic random world: several instances of a paged tree, a couple of
// rounds of expansion attaches, partial residency. Given the same seed it
// reproduces the exact same World (same slots, same pages, same residency).
struct RandomWorld
{
    World w;
    std::vector<UserId> allIds;

    explicit RandomWorld(uint32_t seed)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> uni(0.0f, 1.0f);

        TreeGen gen;
        gen.fanout = 2 + uint32_t(rng() % 3);
        gen.depth = 1 + uint32_t(rng() % 2);

        const int numInstances = 1 + int(rng() % 3);
        for (int inst = 0; inst < numInstances; ++inst)
        {
            Page pg = gen.makeRootPage(unitRegion(40.0f), 64.0f, 2);
            const auto ids = pageIds(pg);
            allIds.insert(allIds.end(), ids.begin(), ids.end());
            const float4 pos = float4::point(uni(rng) * 400 - 200, uni(rng) * 100 - 50,
                                             uni(rng) * 400 - 200);
            w.addInstance(std::move(pg), pos, 0.5f + uni(rng) * 2.0f);
        }

        for (int round = 0; round < 2; ++round)
        {
            std::vector<UserId> exps;
            for (const auto& [id, r] : gen.recipes)
                if (contains(w, id) && !isAttached(w, id)) exps.push_back(id);
            std::sort(exps.begin(), exps.end());
            int budget = 16;
            for (UserId id : exps)
            {
                if (budget <= 0) break;
                if (uni(rng) > 0.4f) continue;
                Page child = gen.makeChildPage(id);
                const auto ids = pageIds(child);
                allIds.insert(allIds.end(), ids.begin(), ids.end());
                attachPage(w, id, std::move(child));
                --budget;
            }
        }

        for (UserId id : allIds)
            if (contains(w, id) && uni(rng) < 0.6f) markResident(w, id);
        w.applyUpdates();
    }
};

Camera randomView(std::mt19937& rng)
{
    std::uniform_real_distribution<float> uni(0.0f, 1.0f);
    const float4 pos = float4::point(uni(rng) * 800 - 400, uni(rng) * 300 - 150,
                                     uni(rng) * 800 - 400);
    const float4 tgt = float4::point(uni(rng) * 200 - 100, 0, uni(rng) * 200 - 100);
    return makeLookAtCamera(pos, tgt);
}

} // namespace

// ---------------------------------------------------------------------------
// Structural invariants that must hold for ANY world and ANY camera:
//   - the current cut is an antichain (no entry is an ancestor of another)
//     and every entry is resident;
//   - the ideal cut is an antichain;
//   - every entry carries a live compact node handle in the correct bucket.
// ---------------------------------------------------------------------------
TEST(Contracts, CutInvariantsHoldOnRandomWorlds)
{
    std::mt19937 rng(2024);
    std::uniform_real_distribution<float> uni(0.0f, 1.0f);

    for (int iter = 0; iter < 15; ++iter)
    {
        RandomWorld rw(1000 + uint32_t(iter));
        World& w = rw.w;

        CutParams p;
        p.threshold = 1.0f + uni(rng) * 30.0f;

        for (int frame = 0; frame < 3; ++frame)
        {
            const Camera v = randomView(rng);
            const Outputs o = run(w, v, p);

            const std::set<UserId> cut = cutIds(w, o.cut);
            ASSERT_EQ(cut.size(), currentCutSize(o.cut))
                << "duplicate payloads in current cut";
            for (const auto& e : currentCut(o.cut))
            {
                const UserId id = payloadOf(w, e);
                EXPECT_TRUE(w.isResident(e.nodeHandle))
                    << "current entry " << id << " not resident";
                for (UserId anc : TA::ancestorIds(w, id))
                    EXPECT_EQ(cut.count(anc), 0u)
                        << "cut contains ancestor " << anc << " of " << id;
            }

            std::set<UserId> ideal;
            for (const auto& e : idealCut(o.cut)) ideal.insert(payloadOf(w, e));
            ASSERT_EQ(ideal.size(), idealCutSize(o.cut))
                << "duplicate payloads in ideal";
            for (const auto& e : idealCut(o.cut))
            {
                const UserId id = payloadOf(w, e);
                for (UserId anc : TA::ancestorIds(w, id))
                    EXPECT_EQ(ideal.count(anc), 0u)
                        << "ideal contains ancestor " << anc << " of " << id;

                const NodeHandle found = TA::requireByScan(w, id);
                EXPECT_EQ(found, e.nodeHandle);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Identical inputs must give byte-identical outputs: same world construction
// + same camera path => the same three bucket sequences, in the same order,
// with bit-equal errors. (No unordered containers or pointers may leak into
// the traversal order.)
// ---------------------------------------------------------------------------
TEST(Contracts, DeterministicAcrossIdenticalWorlds)
{
    for (uint32_t seed : {7u, 42u, 314u})
    {
        RandomWorld a(seed), b(seed);
        // Damping on: the camera envelope history must match too.
        CameraDamper da(4.0f), db(4.0f);
        std::mt19937 rngA(seed * 3), rngB(seed * 3);
        const CutParams p{6.0f, 0.5f};

        for (int frame = 0; frame < 6; ++frame)
        {
            a.w.applyUpdates();
            b.w.applyUpdates();
            const Camera va = da.damp(randomView(rngA));
            const Camera vb = db.damp(randomView(rngB));
            const Outputs oa = run(a.w, va, p);
            const Outputs ob = run(b.w, vb, p);

            EXPECT_EQ(resultKeys(a.w, oa.cut), resultKeys(b.w, ob.cut))
                << "seed " << seed << " frame " << frame;
        }
    }
}

// Parallel workers preserve the exact per-bucket order and classifications.
TEST(Contracts, ParallelSelectionMatchesSerialBucketedCut)
{
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 2;
    const Page proto = gen.makeRootPage(unitRegion(20.0f), 64.0f, 0);
    HLodBuilder flatBuilder;
    flatBuilder.createRoot(
        9000, 0.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0), float4::vec(1, 1, 1)));
    const Page flatProto = flatBuilder.build();

    World serial;
    WorldConfig config;
    config.context.workerCount = 4;
    config.parallelInstanceThreshold = 1;
    World parallel(config);

    const AssetHandle serialAsset = serial.registerAsset(proto.clone());
    const AssetHandle parallelAsset = parallel.registerAsset(proto.clone());
    const AssetHandle serialFlat = serial.registerAsset(flatProto.clone());
    const AssetHandle parallelFlat = parallel.registerAsset(flatProto.clone());
    for (uint32_t i = 0; i < 16; ++i)
    {
        const float4 pos = float4::point(float(i & 3u) * 2.0f, 0.0f,
                                         float(i >> 2) * 2.0f);
        serial.addInstance((i & 1u) ? serialFlat : serialAsset, pos);
        parallel.addInstance((i & 1u) ? parallelFlat : parallelAsset, pos);
    }

    serial.applyUpdates();
    parallel.applyUpdates();
    const Camera view = makeLookAtCamera(float4::point(3, 8, -40),
                                         float4::point(3, 0, 3));
    const CutParams params{0.25f, 0.0f};
    const Outputs a = run(serial, view, params);
    const Outputs b = run(parallel, view, params);

    ASSERT_FALSE(a.cut.empty());
    EXPECT_EQ(resultKeys(parallel, b.cut), resultKeys(serial, a.cut));
}

TEST(Contracts, ViewBindsToOneWorldUntilReset)
{
    World first, second;
    first.applyUpdates();
    second.applyUpdates();

    const World& publishedFirst = first;
    const World& publishedSecond = second;
    const Camera camera = makeLookAtCamera(float4::point(0, 0, -10),
                                           float4::point(0, 0, 0));
    const CutParams params{4.0f, 0.0f};
    CutResults cut;
    View view;

    EXPECT_NO_THROW(view.selectCut(publishedFirst, camera, params, cut));
    EXPECT_THROW(view.selectCut(publishedSecond, camera, params, cut),
                 std::logic_error);

    view.reset();
    EXPECT_NO_THROW(view.selectCut(publishedSecond, camera, params, cut));
}

TEST(Contracts, ViewRequiresPublishedWorld)
{
    World world;
    View view;
    const Camera camera = makeLookAtCamera(float4::point(0, 0, -10),
                                           float4::point(0, 0, 0));
    const CutParams params{4.0f, 0.0f};
    CutResults cut;

    EXPECT_THROW(view.selectCut(world, camera, params, cut), std::logic_error);
    world.applyUpdates();
    EXPECT_NO_THROW(view.selectCut(world, camera, params, cut));
}

// ---------------------------------------------------------------------------
// Multiple damped views share the world but not each other's history: each
// view's memory lives entirely in its own CameraDamper, and selectCut is a pure
// read of the World. Interleaving view B must not change view A's outputs vs
// running A alone on an identical world (and vice versa). This is the
// regression test for moving hysteresis off the nodes onto the camera.
// ---------------------------------------------------------------------------
TEST(Contracts, MultiCameraDamperIsolation)
{
    const uint32_t seed = 99;
    RandomWorld both(seed), onlyA(seed), onlyB(seed);
    for (UserId id : both.allIds)
        if (contains(both.w, id))
        {
            markResident(both.w, id);
            markResident(onlyA.w, id);
            markResident(onlyB.w, id);
        }

    // Heavy damping: the envelope history matters.
    CameraDamper dA(8.0f), dB(8.0f), dAlone(8.0f), dBlone(8.0f);
    const CutParams p{6.0f, 0.0f};

    for (int frame = 0; frame < 8; ++frame)
    {
        both.w.applyUpdates();
        onlyA.w.applyUpdates();
        onlyB.w.applyUpdates();

        const float t = float(frame) * 0.35f;
        const Camera vA = makeLookAtCamera(
            float4::point(std::cos(t) * 300, 120, std::sin(t) * 300), float4::point(0, 0, 0));
        const Camera vB = makeLookAtCamera(
            float4::point(-std::sin(t) * 90, 25, std::cos(t) * 90), float4::point(30, 0, -20));

        CutResults cutA, cutB, cutAlone, cutBlone;
        selectCutUncached(both.w, dA.damp(vA), p, cutA);
        selectCutUncached(both.w, dB.damp(vB), p, cutB);   // interleaved with A every frame
        selectCutUncached(onlyA.w, dAlone.damp(vA), p, cutAlone);
        selectCutUncached(onlyB.w, dBlone.damp(vB), p, cutBlone);

        EXPECT_EQ(resultKeys(both.w, cutA), resultKeys(onlyA.w, cutAlone))
            << "frame " << frame;
        EXPECT_EQ(resultKeys(both.w, cutB), resultKeys(onlyB.w, cutBlone))
            << "frame " << frame;
    }
}

// ---------------------------------------------------------------------------
// Instance slots are recycled; a stale InstanceRef (remove + add reusing the
// slot) must never act on the slot's new occupant — the generation stamp
// makes moveInstance/removeInstance safe no-ops, exactly like stale
// NodeHandles.
// ---------------------------------------------------------------------------
TEST(Contracts, StaleInstanceRefIsIgnored)
{
    TreeGen gen;
    World w;

    Page pgA = gen.makeRootPage(unitRegion(5.0f), 16.0f, 0);
    const auto refA = w.addInstance(std::move(pgA), float4::point(0, 0, 0));
    w.removeInstance(refA);

    Page pgB = gen.makeRootPage(unitRegion(5.0f), 16.0f, 0);
    const auto idsB = pageIds(pgB);
    const auto refB = w.addInstance(std::move(pgB), float4::point(0, 0, 0));
    markAllResident(w, idsB);
    ASSERT_EQ(refA.id, refB.id);   // the slot was recycled (LIFO free list)
    ASSERT_NE(refA.generation, refB.generation);

    w.applyUpdates();
    const Camera v = makeLookAtCamera(float4::point(0, 0, -30), float4::point(0, 0, 0));
    CutResults cut;
    selectCutUncached(w, v, {4, 0}, cut);
    ASSERT_FALSE(cut.empty());

    // Stale move: B must not teleport.
    w.moveInstance(refA, float4::point(50000, 0, 0));
    cut.clear();
    selectCutUncached(w, v, {4, 0}, cut);
    EXPECT_FALSE(cut.empty()) << "stale moveInstance displaced the new instance";

    // Stale remove: B must survive.
    w.removeInstance(refA);
    cut.clear();
    selectCutUncached(w, v, {4, 0}, cut);
    EXPECT_FALSE(cut.empty()) << "stale removeInstance killed the new instance";

    // The live ref still works.
    w.removeInstance(refB);
    cut.clear();
    selectCutUncached(w, v, {4, 0}, cut);
    EXPECT_TRUE(cut.empty());
}

// ---------------------------------------------------------------------------
// Geometry edge cases
// ---------------------------------------------------------------------------

// Zero-extent (point) leaves: the wide culling path and the distance kernels
// must handle degenerate boxes; the cut matches the scalar reference.
TEST(Contracts, PointLeavesMatchReference)
{
    HLodBuilder b;
    const auto root = b.createRoot(1, 64.0f, AABB::empty());
    for (int i = 0; i < 12; ++i)
    {
        const float4 c = float4::vec(float(i % 4) * 10 - 15, 0, float(i / 4) * 10 - 10);
        b.createNode(root, 100 + uint64_t(i), 0.0f, AABB::fromCenterExtent(c, float4::vec(0, 0, 0)));
    }
    Page pg = b.build();
    const auto ids = pageIds(pg);

    World w;
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);
    w.applyUpdates();

    const Camera v = makeLookAtCamera(float4::point(0, 20, -60), float4::point(0, 0, 0));
    const CutParams p{4.0f, 0.0f};
    const Outputs o = run(w, v, p);
    const RefResult want = TA::referenceCut(w, v, p);

    std::set<UserId> wantIds;
    for (const auto& e : currentCut(want.cut)) wantIds.insert(payloadOf(w, e));
    EXPECT_EQ(cutIds(w, o.cut), wantIds);
    EXPECT_FALSE(o.cut.empty());
}

// Camera inside the tree: distances hit zero, screen errors saturate to huge
// finite values, nothing NaNs, and the result still matches the reference.
TEST(Contracts, CameraInsideTreeMatchesReference)
{
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 2;
    Page pg = gen.makeRootPage(unitRegion(50.0f), 64.0f, 0);
    const auto ids = pageIds(pg);

    World w;
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    markAllResident(w, ids);
    w.applyUpdates();

    const Camera v = makeLookAtCamera(float4::point(1, 2, 3), float4::point(40, 0, 40));
    const CutParams p{4.0f, 0.0f};
    const Outputs o = run(w, v, p);
    const RefResult want = TA::referenceCut(w, v, p);

    ASSERT_FALSE(o.cut.empty());
    for (const auto& e : currentCut(o.cut))
        EXPECT_TRUE(std::isfinite(e.approximateError(p.threshold)))
            << payloadOf(w, e);

    std::set<UserId> wantIds;
    for (const auto& e : currentCut(want.cut)) wantIds.insert(payloadOf(w, e));
    EXPECT_EQ(cutIds(w, o.cut), wantIds);
}

// A world far from the origin (1e6 units out): absolute-coordinate math must
// not diverge between the wide path and the scalar reference.
TEST(Contracts, FarFromOriginMatchesReference)
{
    TreeGen gen;
    Page pg = gen.makeRootPage(unitRegion(20.0f), 32.0f, 0);
    const auto ids = pageIds(pg);

    const float4 farPos = float4::point(1.0e6f, 0, 1.0e6f);
    World w;
    w.addInstance(std::move(pg), farPos);
    markAllResident(w, ids);
    w.applyUpdates();

    const Camera v = makeLookAtCamera(farPos + float4::vec(0, 30, -80), farPos);
    const CutParams p{4.0f, 0.0f};
    const Outputs o = run(w, v, p);
    const RefResult want = TA::referenceCut(w, v, p);

    std::set<UserId> wantIds;
    for (const auto& e : currentCut(want.cut)) wantIds.insert(payloadOf(w, e));
    EXPECT_EQ(cutIds(w, o.cut), wantIds);
    EXPECT_FALSE(o.cut.empty());
}

// Extreme instance scale: the local-space transform keeps error selection
// scale-invariant and equivalent to the reference.
TEST(Contracts, ScaledInstanceMatchesReference)
{
    TreeGen gen;
    for (float scale : {0.01f, 1.0f, 250.0f})
    {
        Page pg = gen.makeRootPage(unitRegion(10.0f), 32.0f, 0);
        const auto ids = pageIds(pg);

        World w;
        w.addInstance(std::move(pg), float4::point(0, 0, 0), scale);
        markAllResident(w, ids);
        w.applyUpdates();

        const Camera v = makeLookAtCamera(float4::point(0, 10 * scale, -40 * scale),
                                          float4::point(0, 0, 0));
        const CutParams p{4.0f, 0.0f};
        const Outputs o = run(w, v, p);
        const RefResult want = TA::referenceCut(w, v, p);

        std::set<UserId> wantIds;
        for (const auto& e : currentCut(want.cut)) wantIds.insert(payloadOf(w, e));
        EXPECT_EQ(cutIds(w, o.cut), wantIds) << "scale " << scale;
        EXPECT_FALSE(o.cut.empty()) << "scale " << scale;
    }
}

// NaN / empty / infinite boxes are rejected at the API boundary. Grow-only
// refit can never un-grow, so one NaN box would poison ancestors forever.
TEST(Contracts, NonFiniteBoundsRejected)
{
    TreeGen gen;
    Page pg = gen.makeRootPage(unitRegion(5.0f), 16.0f, 0);
    World w;
    const auto inst = w.addInstance(std::move(pg), float4::point(0, 0, 0));
    const NodeHandle leaf = nodeAt(inst.rootPage, 2);

    const float nan = std::nanf("");
    const float inf = std::numeric_limits<float>::infinity();
    EXPECT_THROW(w.setNodeBounds(inst, leaf, AABB::fromCenterExtent(
                     float4::vec(nan, 0, 0), float4::vec(1, 1, 1))), std::logic_error);
    EXPECT_THROW(w.setNodeBounds(inst, leaf, AABB::fromCenterExtent(
                     float4::vec(0, 0, 0), float4::vec(nan, 1, 1))), std::logic_error);
    EXPECT_THROW(w.setNodeBounds(inst, leaf, AABB::fromCenterExtent(
                     float4::vec(0, 0, 0), float4::vec(inf, 1, 1))), std::logic_error);
    EXPECT_THROW(w.setNodeBounds(inst, leaf, AABB::empty()), std::logic_error);

    // Zero-extent is legal (a point object).
    EXPECT_NO_THROW(w.setNodeBounds(inst, leaf, AABB::fromCenterExtent(
                        float4::vec(1, 2, 3), float4::vec(0, 0, 0))));
    w.flushBounds();
}

// ---------------------------------------------------------------------------
// Memory budgets: catch accidental per-node or per-page bloat. Page data is
// the dominant cost at scale; the budgets are deliberately loose enough to
// survive refactors but tight enough to flag a new hot-array or a hash map
// sneaking back in.
// ---------------------------------------------------------------------------
TEST(Contracts, CompactCutRepresentation)
{
    EXPECT_EQ(sizeof(NodeHandle), 8u);
    EXPECT_EQ(sizeof(CutEntry), 12u);

    const NodeHandle handle{NodeHandle::kInvalidSlot - 1,
                            NodeHandle::kIndexMask,
                            NodeHandle::kGenerationMask};
    EXPECT_TRUE(handle.valid());
    EXPECT_EQ(handle.slot(), NodeHandle::kInvalidSlot - 1);
    EXPECT_EQ(handle.index(), NodeHandle::kIndexMask);
    EXPECT_EQ(handle.generation(), NodeHandle::kGenerationMask);

    constexpr float threshold = 4.0f;
    EXPECT_LT(encodeCutError(threshold, threshold), kCutErrorThreshold);
    EXPECT_LT(encodeCutError(std::nextafter(threshold, 0.0f), threshold),
              kCutErrorThreshold);
    EXPECT_GE(encodeCutError(std::nextafter(
                  threshold, std::numeric_limits<float>::infinity()), threshold),
              kCutErrorThreshold);

    uint8_t previous = 0;
    for (float error : {0.0f, 0.01f, 0.25f, 1.0f, 3.99f, 4.0f,
                        4.01f, 8.0f, 64.0f, 1.0e20f})
    {
        const uint8_t code = encodeCutError(error, threshold);
        EXPECT_GE(code, previous);
        previous = code;
    }

    const CutEntry entry{handle, 8.0f, threshold, kInvalidInstanceId - 1};
    EXPECT_EQ(entry.nodeHandle, handle);
    EXPECT_EQ(entry.instance(), kInvalidInstanceId - 1);
    EXPECT_TRUE(entry.overThreshold());
    EXPECT_TRUE(std::isfinite(entry.approximateError(threshold)));

    CutEntry sharedStorage[1];
    CutEntry currentStorage[1];
    CutEntry idealStorage[1];
    CutResultSink sink{Sink<CutEntry>{sharedStorage, 1},
                       Sink<CutEntry>{currentStorage, 1},
                       Sink<CutEntry>{idealStorage, 1}};
    sink.shared.push(entry);
    sink.shared.push(entry);
    sink.currentOnly.push(entry);
    sink.idealOnly.push(entry);
    EXPECT_EQ(sink.shared.count(), 1u);
    EXPECT_EQ(sink.shared.dropped(), 1u);
    EXPECT_EQ(sink.currentOnly.count(), 1u);
    EXPECT_EQ(sink.idealOnly.count(), 1u);
}

TEST(Contracts, MemoryBudgets)
{
    TreeGen gen8;
    gen8.fanout = 8;
    gen8.depth = 2;
    const Page p8 = gen8.makeRootPage(unitRegion(50.0f), 64.0f, 0);
    RecordProperty("bytes_per_node_fanout8", int(p8.byteSize() / p8.nodeCount()));
    EXPECT_LE(p8.byteSize() / p8.nodeCount(), 128u);

    TreeGen gen4;
    gen4.fanout = 4;
    gen4.depth = 3;
    const Page p4 = gen4.makeRootPage(unitRegion(50.0f), 64.0f, 0);
    RecordProperty("bytes_per_node_fanout4", int(p4.byteSize() / p4.nodeCount()));
    EXPECT_LE(p4.byteSize() / p4.nodeCount(), 176u);

#ifdef NDEBUG
    // Fixed runtime layouts. These are hot-array strides or multiply by the
    // number of assets/pages; recording them makes accidental padding growth
    // visible in benchmark output as well as enforcing broad ceilings here.
    RecordProperty("assetrt_bytes", int(TA::assetRtBytes()));
    RecordProperty("pagert_bytes", int(TA::pageRtBytes()));
    RecordProperty("page_stamp_bytes", int(TA::pageStampBytes()));
    RecordProperty("page_residency_bytes", int(TA::pageResidencyBytes()));
    RecordProperty("overlay_bytes", int(TA::overlayBytes()));
    RecordProperty("instance_bytes", int(TA::instanceBytes()));
    RecordProperty("instance_tlas_bytes", int(TA::instanceTlasBytes()));
    RecordProperty("tlas_node_bytes", int(TA::tlasNodeBytes()));
    RecordProperty("work_item_bytes", int(TA::workItemBytes()));
    RecordProperty("node_item_bytes", int(TA::nodeItemBytes()));
    RecordProperty("pending_move_bytes", int(TA::pendingMoveBytes()));
    RecordProperty("tlas_item_bytes", int(TA::tlasItemBytes()));
    RecordProperty("morton_item_bytes", int(TA::mortonItemBytes()));
    EXPECT_EQ(TA::assetRtBytes(), 112u);
    EXPECT_EQ(TA::pageRtBytes(), 104u);
    EXPECT_EQ(TA::pageStampBytes(), 8u);
    EXPECT_EQ(TA::pageResidencyBytes(), 8u);
    EXPECT_EQ(TA::overlayBytes(), 56u);
    EXPECT_EQ(TA::instanceBytes(), 64u);
    EXPECT_EQ(TA::instanceTlasBytes(), 48u);
    EXPECT_EQ(TA::tlasNodeBytes(), 320u);
    EXPECT_EQ(TA::workItemBytes(), 24u);
    EXPECT_EQ(TA::nodeItemBytes(), 8u);
    EXPECT_EQ(TA::pendingMoveBytes(), 48u);
    EXPECT_EQ(TA::tlasItemBytes(), 4u);
    EXPECT_EQ(TA::mortonItemBytes(), 12u);
#endif
}
