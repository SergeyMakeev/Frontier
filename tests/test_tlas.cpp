// Incremental structural edits to the top-level tree.
//
// Spawning or removing an instance used to mark the whole TLAS dirty, so the
// next cut rebuilt it from scratch: one spawn cost 2.1 ms at 20k instances and
// 9.5 ms at 80k, the same as five hundred spawns (ARCHITECTURE.md, experiment
// L). Both operations are now applied in place in O(depth).
//
// That trades a guarantee for speed. A rebuilt tree is correct by construction;
// an edited one is correct only if every edit maintains the invariants the query
// relies on -- lane boxes containing their contents, lane masks covering their
// subtree's layers, parent links, and each live instance appearing exactly once.
// So the tests here lean on two things a cut comparison alone will not catch:
// TAX::tlasValidate audits the structure directly, and the equivalence tests
// compare against the same world after a forced rebuild.

#include <gtest/gtest.h>

#include <random>
#include <set>

#include "helpers.h"

namespace hlodtest {
namespace {

// Cut output carries the World's compact dense InstanceId. Generation remains
// on InstanceRef for mutation safety, while render-side entity identity is a
// caller table indexed by this id.
struct Placed
{
    World::InstanceRef ref;
    InstanceId         id;
};

// A shared asset with a shallow tree, resident throughout: these tests are
// about the top level, so nothing should ever be waiting on streaming.
//
// The config is a constructor argument because most of these tests need to
// isolate ONE of the three rebuild triggers. Leaving all three at their defaults
// means an edit trips whichever fires first, and a test that meant to exercise
// in-place editing silently measures a rebuild instead.
struct Field
{
    World       w;
    AssetHandle asset{};
    TreeGen     gen;

    explicit Field(const WorldConfig& cfg = WorldConfig{}) : w(cfg)
    {
        gen.fanout = 4;
        gen.depth = 1;
        Page proto = gen.makeRootPage(unitRegion(2.0f), 1.0f, 0);
        const uint32_t nodes = proto.nodeCount();
        asset = w.registerAsset(std::move(proto));
        // The root mount only exists once an instance references the asset.
        const World::InstanceRef seed = w.addInstance(asset, float4::point(0, 0, 0));
        markAllResident(w, w.assetRootPage(asset), nodes);
        w.removeInstance(seed);
    }

    Placed add(float x, float z, uint32_t mask = ~0u)
    {
        InstanceDesc d;
        d.pos = float4::point(x, 0, z);
        d.mask = mask;
        const World::InstanceRef ref = w.addInstance(asset, d);
        return {ref, ref.id};
    }
};

// Looks straight down from high enough that the footprint covers halfExtent in
// both directions around (x, z). Getting this wrong is the easy way to write a
// test that passes because the instance it is asserting about was never in the
// frustum: fovY 1.4 gives tan(fovY/2) ~ 0.84, so 1.4x the half-extent clears it.
Camera viewFrom(float x, float z, float halfExtent, uint32_t mask = ~0u)
{
    Camera v = makePerspectiveCamera(float4::point(x, halfExtent * 1.4f, z),
                                     float4::vec(0, -1, 0.001f), float4::vec(0, 1, 0),
                                     1.4f, 1.0f, 1080.0f, 0.1f, 1.0e9f);
    v.viewMask = mask;
    return v;
}

// Isolates one trigger by putting the others out of reach.
WorldConfig onlyEditBudget()
{
    WorldConfig c;
    c.tlasAreaDrift = 1.0e9f;
    c.tlasCountDrift = 1.0e9f;
    return c;
}
WorldConfig neverRebuild()
{
    WorldConfig c = onlyEditBudget();
    c.tlasEditFraction = 1.0e9f;
    c.tlasEscapeFraction = 1.0e9f;
    return c;
}

std::multiset<InstanceId> instanceIdsOf(const CutResults& cut)
{
    std::multiset<uint32_t> out;
    for (const CutEntry& e : currentCut(cut)) out.insert(e.instance());
    return out;
}

const CutParams kParams{8.0f, 0.0f};

}   // namespace

// ---------------------------------------------------------------------------
// The property that matters: an edited tree answers exactly like a rebuilt one
// ---------------------------------------------------------------------------

TEST(Tlas, SpawnIsVisibleWithoutRebuilding)
{
    Field f;
    for (int i = 0; i < 200; ++i)
        f.add(float(i % 20) * 4.0f - 40.0f, float(i / 20) * 4.0f - 20.0f);

    const Camera v = viewFrom(0, 0, 60.0f);
    CutResults before;
    selectCutUncached(f.w, v, kParams, before);
    ASSERT_FALSE(TAX::tlasDirty(f.w));

    const Placed fresh = f.add(2.0f, 2.0f);
    EXPECT_FALSE(TAX::tlasDirty(f.w)) << "a single spawn must not dirty the tree";
    EXPECT_EQ(TAX::tlasValidate(f.w), "");

    CutResults after;
    selectCutUncached(f.w, v, kParams, after);
    EXPECT_GT(after.size(), before.size());
    EXPECT_TRUE(instanceIdsOf(after).count(fresh.id)) << "the spawned instance is missing";
}

TEST(Tlas, RemoveVanishesWithoutRebuilding)
{
    Field f;
    std::vector<Placed> refs;
    for (int i = 0; i < 200; ++i)
        refs.push_back(f.add(float(i % 20) * 4.0f - 40.0f, float(i / 20) * 4.0f - 20.0f));

    const Camera v = viewFrom(0, 0, 60.0f);
    CutResults cut;
    selectCutUncached(f.w, v, kParams, cut);
    const InstanceId victim = refs[97].id;
    ASSERT_TRUE(instanceIdsOf(cut).count(victim));

    f.w.removeInstance(refs[97].ref);
    EXPECT_FALSE(TAX::tlasDirty(f.w)) << "a single removal must not dirty the tree";
    EXPECT_EQ(TAX::tlasValidate(f.w), "");

    selectCutUncached(f.w, v, kParams, cut);
    EXPECT_EQ(instanceIdsOf(cut).count(victim), 0u) << "a removed instance is still emitted";
}

// The strong one. Churn, move and re-spawn for many rounds, and after every
// round require that the incrementally edited tree produces the same cut as the
// same world rebuilt from scratch. Incremental edits are allowed to make the
// tree WORSE, never to make it wrong.
TEST(Tlas, IncrementalEditsMatchARebuiltTree)
{
    Field f;
    std::mt19937 rng(20260804);
    std::uniform_real_distribution<float> uni(-60.0f, 60.0f);

    std::vector<Placed> refs;
    for (int i = 0; i < 400; ++i) refs.push_back(f.add(uni(rng), uni(rng)));

    for (int round = 0; round < 60; ++round)
    {
        for (int k = 0; k < 3; ++k)
        {
            const size_t i = rng() % refs.size();
            f.w.removeInstance(refs[i].ref);
            refs[i] = refs.back();
            refs.pop_back();
        }
        for (int k = 0; k < 3; ++k) refs.push_back(f.add(uni(rng), uni(rng)));
        for (int k = 0; k < 5; ++k)
        {
            const size_t i = rng() % refs.size();
            f.w.moveInstance(refs[i].ref, float4::point(uni(rng), 0, uni(rng)));
        }

        ASSERT_EQ(TAX::tlasValidate(f.w), "") << "round " << round;

        const Camera v = viewFrom(uni(rng) * 0.3f, uni(rng) * 0.3f, 90.0f);
        CutResults incremental;
        selectCutUncached(f.w, v, kParams, incremental);

        TAX::forceTlasRebuild(f.w);
        CutResults rebuilt;
        selectCutUncached(f.w, v, kParams, rebuilt);

        EXPECT_EQ(instanceIdsOf(incremental), instanceIdsOf(rebuilt)) << "round " << round;
    }
}

// ---------------------------------------------------------------------------
// The specific ways an in-place edit can go wrong
// ---------------------------------------------------------------------------

// A tree fresh from a build has every leaf lane occupied, so almost every
// insert has to split a full leaf rather than take a free lane. That path
// re-parents a node, which is the one edit that rewrites a link somebody else
// holds.
TEST(Tlas, SplittingFullLeavesKeepsEveryInstanceReachable)
{
    Field f(neverRebuild());
    for (int i = 0; i < 64; ++i) f.add(float(i % 8) * 8.0f, float(i / 8) * 8.0f);

    CutResults cut;
    const Camera v = viewFrom(28.0f, 28.0f, 40.0f);
    selectCutUncached(f.w, v, kParams, cut);
    ASSERT_FALSE(TAX::tlasDirty(f.w));
    const size_t nodesBefore = TAX::tlasNodeCount(f.w);

    // Every one of these lands inside the existing extent, so each descends to
    // a leaf that is already full.
    std::vector<Placed> added;
    for (int i = 0; i < 64; ++i)
    {
        added.push_back(f.add(float(i % 8) * 8.0f + 1.0f, float(i / 8) * 8.0f + 1.0f));
        ASSERT_FALSE(TAX::tlasDirty(f.w));
        ASSERT_EQ(TAX::tlasValidate(f.w), "") << "after insert " << i;
    }
    EXPECT_GT(TAX::tlasNodeCount(f.w), nodesBefore) << "no split ever happened";

    selectCutUncached(f.w, v, kParams, cut);
    const std::multiset<InstanceId> ids = instanceIdsOf(cut);
    for (const Placed& r : added)
        EXPECT_TRUE(ids.count(r.id)) << "instance " << r.id << " lost by a split";
}

// An ancestor's lane mask must cover every layer in its subtree or the query's
// layer filter culls a visible instance. A rebuild computes those unions; an
// insert has to push the new mask up the chain, and nothing else in the library
// exercises that.
TEST(Tlas, InsertionPropagatesLayerMasksUpTheTree)
{
    Field f;
    for (int i = 0; i < 200; ++i)
        f.add(float(i % 20) * 4.0f - 40.0f, float(i / 20) * 4.0f - 20.0f, 0x1u);

    CutResults cut;
    selectCutUncached(f.w, viewFrom(0, 0, 60.0f), kParams, cut);
    ASSERT_FALSE(TAX::tlasDirty(f.w));

    // A spawn on a layer no existing instance uses: every ancestor lane mask on
    // its path is currently 0x1 and must pick up 0x4.
    const Placed odd = f.add(2.0f, 2.0f, 0x4u);
    ASSERT_FALSE(TAX::tlasDirty(f.w));
    EXPECT_EQ(TAX::tlasValidate(f.w), "");

    selectCutUncached(f.w, viewFrom(0, 0, 60.0f, 0x4u), kParams, cut);
    EXPECT_TRUE(instanceIdsOf(cut).count(odd.id))
        << "the layer filter culled a freshly inserted instance";

    // And a view on the layer it is not on must not pick it up.
    selectCutUncached(f.w, viewFrom(0, 0, 60.0f, 0x1u), kParams, cut);
    EXPECT_EQ(instanceIdsOf(cut).count(odd.id), 0u);
}

// An instance spawned outside the current root extent has to grow every lane on
// its path, or it is culled by an ancestor that does not know it is there. Area
// drift is disabled here so this exercises the growth rather than the rebuild it
// would otherwise provoke -- see the next test for that.
TEST(Tlas, InsertionOutsideTheRootExtentGrowsThePath)
{
    Field f(neverRebuild());
    for (int i = 0; i < 200; ++i)
        f.add(float(i % 20) * 2.0f - 20.0f, float(i / 20) * 2.0f - 10.0f);
    CutResults cut;
    selectCutUncached(f.w, viewFrom(0, 0, 40.0f), kParams, cut);
    ASSERT_FALSE(TAX::tlasDirty(f.w));

    const Placed out = f.add(900.0f, 900.0f);
    ASSERT_FALSE(TAX::tlasDirty(f.w));
    EXPECT_EQ(TAX::tlasValidate(f.w), "");

    selectCutUncached(f.w, viewFrom(900.0f, 900.0f, 20.0f), kParams, cut);
    EXPECT_TRUE(instanceIdsOf(cut).count(out.id)) << "an out-of-extent spawn is unreachable";
}

// The flip side, with the default config: a spawn that expands the tree's total
// lane area by more than tlasAreaDrift is charged to the same budget grow-only
// motion refit uses, and forces a rebuild. That is intended -- a tree whose root
// just grew by a factor really is a bad fit -- but it does mean a spawn is only
// cheap when it lands somewhere the tree already covers.
TEST(Tlas, ADistantSpawnTripsTheAreaBudget)
{
    Field f;
    for (int i = 0; i < 200; ++i)
        f.add(float(i % 20) * 2.0f - 20.0f, float(i / 20) * 2.0f - 10.0f);
    CutResults cut;
    selectCutUncached(f.w, viewFrom(0, 0, 40.0f), kParams, cut);
    ASSERT_FALSE(TAX::tlasDirty(f.w));

    const Placed out = f.add(9000.0f, 9000.0f);
    EXPECT_TRUE(TAX::tlasDirty(f.w));

    // Still correct, just by the slower route.
    selectCutUncached(f.w, viewFrom(9000.0f, 9000.0f, 20.0f), kParams, cut);
    EXPECT_EQ(TAX::tlasValidate(f.w), "");
    EXPECT_TRUE(instanceIdsOf(cut).count(out.id));
}

// Emptying a node unlinks it and puts its slot on a free list; a later split
// pops it. So a recycled node must not still be referenced by anybody. Every
// rebuild trigger is disabled, or the removals below would rebuild the tree and
// the free list would never be read.
TEST(Tlas, EmptyingAndRefillingReusesNodesSafely)
{
    Field f(neverRebuild());
    for (int i = 0; i < 128; ++i) f.add(float(i % 16) * 4.0f, float(i / 16) * 4.0f);
    std::vector<Placed> refs;
    for (int i = 0; i < 128; ++i)
        refs.push_back(f.add(float(i % 16) * 4.0f + 1.0f, float(i / 16) * 4.0f + 1.0f));

    CutResults cut;
    const Camera v = viewFrom(30.0f, 14.0f, 50.0f);
    selectCutUncached(f.w, v, kParams, cut);
    ASSERT_FALSE(TAX::tlasDirty(f.w));
    ASSERT_EQ(TAX::tlasValidate(f.w), "");

    // Drop the second cohort. Because they were inserted incrementally they sit
    // in split nodes, several of which empty completely.
    for (const Placed& r : refs)
    {
        f.w.removeInstance(r.ref);
        ASSERT_EQ(TAX::tlasValidate(f.w), "");
    }
    ASSERT_FALSE(TAX::tlasDirty(f.w));
    const size_t nodesAfterRemoval = TAX::tlasNodeCount(f.w);

    std::vector<Placed> again;
    for (int i = 0; i < 128; ++i)
    {
        again.push_back(f.add(float(i % 16) * 4.0f + 2.0f, float(i / 16) * 4.0f + 2.0f));
        ASSERT_EQ(TAX::tlasValidate(f.w), "") << "after respawn " << i;
    }
    EXPECT_EQ(TAX::tlasNodeCount(f.w), nodesAfterRemoval)
        << "respawning grew the node array instead of reusing emptied nodes";

    selectCutUncached(f.w, v, kParams, cut);
    const std::multiset<InstanceId> ids = instanceIdsOf(cut);
    for (const Placed& r : again)
        EXPECT_TRUE(ids.count(r.id)) << "respawned instance " << r.id << " is missing";

    // Dense ids are intentionally recycled. The old generation-stamped refs
    // must nevertheless be unable to remove the new occupants of those ids.
    for (const Placed& r : refs) f.w.removeInstance(r.ref);
    selectCutUncached(f.w, v, kParams, cut);
    const std::multiset<InstanceId> afterStaleRemoves = instanceIdsOf(cut);
    for (const Placed& r : again)
        EXPECT_TRUE(afterStaleRemoves.count(r.id));
}

TEST(Tlas, RemovingEveryInstanceLeavesAQueryableEmptyWorld)
{
    Field f;
    std::vector<Placed> refs;
    for (int i = 0; i < 30; ++i) refs.push_back(f.add(float(i) * 5.0f, 0.0f));
    for (const Placed& r : refs) f.w.removeInstance(r.ref);
    EXPECT_EQ(TAX::tlasValidate(f.w), "");

    CutResults cut;
    const Camera v = viewFrom(70.0f, 0, 120.0f);
    selectCutUncached(f.w, v, kParams, cut);
    EXPECT_TRUE(cut.empty());

    // And the world still works afterwards.
    const Placed fresh = f.add(70.0f, 0.0f);
    selectCutUncached(f.w, v, kParams, cut);
    EXPECT_TRUE(instanceIdsOf(cut).count(fresh.id));
}

// The edit budget exists so accumulated quality loss is bounded. Sustained
// churn must eventually rebuild rather than degrading forever.
TEST(Tlas, SustainedChurnEventuallyForcesARebuild)
{
    Field f(onlyEditBudget());
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> uni(-60.0f, 60.0f);
    std::vector<Placed> refs;
    for (int i = 0; i < 200; ++i) refs.push_back(f.add(uni(rng), uni(rng)));

    CutResults cut;
    const Camera v = viewFrom(0, 0, 90.0f);
    selectCutUncached(f.w, v, kParams, cut);
    ASSERT_FALSE(TAX::tlasDirty(f.w));

    // tlasEditFraction defaults to 0.05, so ~10 edits against 200 instances.
    bool rebuilt = false;
    for (int round = 0; round < 40 && !rebuilt; ++round)
    {
        const size_t i = rng() % refs.size();
        f.w.removeInstance(refs[i].ref);
        refs[i] = refs.back();
        refs.pop_back();
        refs.push_back(f.add(uni(rng), uni(rng)));
        rebuilt = TAX::tlasDirty(f.w);
    }
    EXPECT_TRUE(rebuilt) << "the edit budget never triggered a rebuild";

    selectCutUncached(f.w, v, kParams, cut);
    EXPECT_FALSE(TAX::tlasDirty(f.w));
    EXPECT_EQ(TAX::tlasValidate(f.w), "");
}

// tlasEscapeFraction is a fraction of the population, not a counter of move
// calls. A small cohort that keeps moving may loosen its own lanes, but must
// not eventually look like the whole world escaped merely by being updated on
// many frames. Distinct escaped instances still consume the budget normally.
TEST(Tlas, EscapeBudgetCountsDistinctInstances)
{
    WorldConfig config = neverRebuild();
    config.tlasEscapeFraction = 0.25f;
    Field f(config);
    std::vector<Placed> refs;
    for (int i = 0; i < 200; ++i)
        refs.push_back(f.add(float(i % 20) * 4.0f, float(i / 20) * 4.0f));

    CutResults cut;
    selectCutUncached(f.w, viewFrom(40.0f, 20.0f, 80.0f), kParams, cut);
    ASSERT_FALSE(TAX::tlasDirty(f.w));

    for (int frame = 0; frame < 100; ++frame)
        f.w.moveInstance(refs[0].ref,
                         float4::point(frame & 1 ? 1000.0f : -1000.0f, 0.0f, 0.0f));
    EXPECT_FALSE(TAX::tlasDirty(f.w));
    EXPECT_EQ(TAX::tlasEscapes(f.w), 1u);

    for (int i = 1; i <= 50; ++i)
        f.w.moveInstance(refs[size_t(i)].ref,
                         float4::point(1000.0f + float(i), 0.0f, 1000.0f));
    EXPECT_TRUE(TAX::tlasDirty(f.w));
}

// A mass despawn is a real population change, not churn, and should still take
// the immediate quality rebuild rather than thousands of incremental removals.
TEST(Tlas, MassDespawnStillForcesAQualityRebuild)
{
    Field f;
    std::vector<Placed> refs;
    for (int i = 0; i < 200; ++i)
        refs.push_back(f.add(float(i % 20) * 4.0f, float(i / 20) * 4.0f));
    CutResults cut;
    const Camera v = viewFrom(40.0f, 20.0f, 60.0f);
    selectCutUncached(f.w, v, kParams, cut);
    ASSERT_FALSE(TAX::tlasDirty(f.w));

    // tlasCountDrift defaults to 0.2: half the population is well past it.
    for (int i = 0; i < 100; ++i) f.w.removeInstance(refs[size_t(i)].ref);
    EXPECT_TRUE(TAX::tlasDirty(f.w));

    selectCutUncached(f.w, v, kParams, cut);
    EXPECT_EQ(TAX::tlasValidate(f.w), "");
    const std::multiset<InstanceId> ids = instanceIdsOf(cut);
    for (int i = 0; i < 100; ++i) EXPECT_EQ(ids.count(refs[size_t(i)].id), 0u);
}

// Moving an instance that was placed by an incremental insert has to work the
// same as moving one the builder placed: the refit walks the same parent chain,
// which a split has since rewritten.
TEST(Tlas, MovingAnIncrementallyInsertedInstanceRefitsCorrectly)
{
    Field f(neverRebuild());
    for (int i = 0; i < 64; ++i) f.add(float(i % 8) * 8.0f, float(i / 8) * 8.0f);
    CutResults cut;
    selectCutUncached(f.w, viewFrom(28.0f, 28.0f, 40.0f), kParams, cut);
    ASSERT_FALSE(TAX::tlasDirty(f.w));

    const Placed r = f.add(9.0f, 9.0f);
    ASSERT_FALSE(TAX::tlasDirty(f.w));
    f.w.moveInstance(r.ref, float4::point(300.0f, 0, 300.0f));
    EXPECT_EQ(TAX::tlasValidate(f.w), "");

    selectCutUncached(f.w, viewFrom(300.0f, 300.0f, 20.0f), kParams, cut);
    EXPECT_TRUE(instanceIdsOf(cut).count(r.id)) << "moved after insertion and lost";
}

// Flat instances bypass the page walk, but a grow-only TLAS leaf still keeps
// its old extent after motion. The direct path must retest the precise world
// box or it would emit the object in both its old and new locations.
TEST(Tlas, FlatInstanceRetestsExactBoundsAfterMotion)
{
    World w(neverRebuild());
    HLodBuilder builder;
    builder.createRoot(
        77, 6.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0), float4::vec(1, 1, 1)));
    const AssetHandle asset = w.registerAsset(builder.build());
    const World::InstanceRef instance =
        w.addInstance(asset, float4::point(0, 0, 0));
    w.applyUpdates();

    const Camera oldView = viewFrom(0.0f, 0.0f, 12.0f);
    CutResults cut;
    selectCutUncached(w, oldView, kParams, cut);
    ASSERT_EQ(currentCut(cut).size(), 1u);
    const RefResult reference = TAX::referenceCut(w, oldView, kParams);
    ASSERT_EQ(currentCut(reference.cut).size(), 1u);
    EXPECT_EQ(currentCut(cut)[0].errorCode(),
              currentCut(reference.cut)[0].errorCode());

    w.moveInstance(instance, float4::point(300.0f, 0, 0));
    w.applyUpdates();
    ASSERT_FALSE(TAX::tlasDirty(w));
    EXPECT_EQ(TAX::tlasValidate(w), "");

    selectCutUncached(w, oldView, kParams, cut);
    EXPECT_TRUE(cut.empty());
}

// The radix rebuild must also handle the worst Morton distribution: almost
// every centroid quantises to the same key. Stable scatter keeps that cohort
// deterministic without needing a comparison sort inside the equal-key run.
TEST(Tlas, MortonRebuildHandlesCoincidentCentroids)
{
    WorldConfig config = onlyEditBudget();
    config.tlasEscapeFraction = 0.0f;
    Field f(config);

    std::vector<Placed> refs;
    refs.reserve(1100);
    for (int i = 0; i < 1100; ++i) refs.push_back(f.add(0.0f, 0.0f));

    const Camera view = viewFrom(0.0f, 0.0f, 400.0f);
    CutResults cut;
    selectCutUncached(f.w, view, kParams, cut);   // initial quality build

    f.w.moveInstance(refs[0].ref, float4::point(300.0f, 0.0f, 300.0f));
    selectCutUncached(f.w, view, kParams, cut);   // forced Morton rebuild

    EXPECT_EQ(TAX::tlasValidate(f.w), "");
    EXPECT_EQ(instanceIdsOf(cut),
              instanceIdsOf(TAX::referenceCut(f.w, view, kParams).cut));
}

}   // namespace hlodtest
