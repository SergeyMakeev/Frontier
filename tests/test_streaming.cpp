#include <gtest/gtest.h>

#include <algorithm>
#include <set>
#include <tuple>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;
using TA = SpatialDatabase::TestAccess;

namespace {

struct Outputs
{
    SpatialDatabase* world = nullptr;
    FrontierResult cut;
};

Outputs frame(SpatialDatabase& w, const Camera& v, const SelectionParams& p)
{
    w.applyUpdates();
    Outputs o;
    o.world = &w;
    selectFrontierUncached(w, v, p, o.cut);
    return o;
}

std::set<UserId> frontierIds(const Outputs& o)
{
    std::set<UserId> ids;
    for (const auto& e : currentFrontier(o.cut)) ids.insert(payloadOf(*o.world, e));
    return ids;
}

} // namespace

TEST(Streaming, BorrowedAssetKeepsExternalOwnership)
{
    PageBuilder builder;
    builder.createRoot(
        42, 0.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0), float4::vec(1, 1, 1)));
    Page backing = builder.build();
    const void* const originalData = backing.data();

    SpatialDatabase world;
    const AssetHandle asset =
        world.registerAsset(static_cast<const PageView&>(backing));
    const SpatialDatabase::InstanceRef instance =
        world.addInstance(asset, float4::point(0, 0, 0));

    FrontierResult cut;
    selectFrontierUncached(
        world,
        makeLookAtCamera(float4::point(0, 0, -8), float4::point(0, 0, 0)),
        SelectionParams{4.0f, 0.0f}, cut);
    ASSERT_EQ(cut.currentSize(), 1u);
    EXPECT_EQ(payloadOf(world, currentFrontier(cut).front()), 42u);

    world.removeInstance(instance);
    world.releaseAsset(asset);
    EXPECT_TRUE(backing.valid());
    EXPECT_EQ(backing.data(), originalData);
}

// ---------------------------------------------------------------------------
// All-or-nothing refinement: a parent refines only when EVERY child is
// covered by resident cuts; ideal-only entries expose what the caller may load.
// ---------------------------------------------------------------------------
TEST(Streaming, AllOrNothingRefinement)
{
    PageBuilder b;
    const auto root = b.createRoot(1, 32.0f);
    b.createNode(root, 10, 1.0f, AABB::fromCenterExtent(float4::vec(-2, 0, 0), float4::vec(1, 1, 1)));
    b.createNode(root, 11, 1.0f, AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
    b.createNode(root, 12, 1.0f, AABB::fromCenterExtent(float4::vec(2, 0, 0), float4::vec(1, 1, 1)));
    Page pg = b.build();

    SpatialDatabase w;
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    // Root is pinned-resident automatically. Children start non-resident.

    const Camera v = makeLookAtCamera(float4::point(0, 0, -20), float4::point(0, 0, 0));
    const SelectionParams p{4.0f, 0.0f};

    auto o = frame(w, v, p);
    EXPECT_EQ(frontierIds(o), std::set<UserId>{1});                    // parent draws
    EXPECT_EQ(o.cut.idealOnly.size(), 3u);

    markResident(w, 10);
    markResident(w, 11);
    o = frame(w, v, p);
    EXPECT_EQ(frontierIds(o), std::set<UserId>{1});                    // still all-or-nothing
    auto missing = std::find_if(o.cut.idealOnly.begin(), o.cut.idealOnly.end(),
                                [&](const FrontierEntry& e) {
        return !w.isResident(e.nodeHandle);
    });
    ASSERT_NE(missing, o.cut.idealOnly.end());
    EXPECT_EQ(payloadOf(w, *missing), 12u);                       // only the missing one

    // The production flow: the caller chooses an ideal entry and completes
    // the load through its node handle.
    w.markResident(missing->nodeHandle);
    o = frame(w, v, p);
    EXPECT_EQ(frontierIds(o), (std::set<UserId>{10, 11, 12}));         // refined

    // Invariant F: everything drawn is resident.
    for (const auto& e : currentFrontier(o.cut))
        EXPECT_TRUE(w.isResident(e.nodeHandle));

    // Eviction: parent falls back next frame, no holes.
    markNonResident(w, 11);
    o = frame(w, v, p);
    EXPECT_EQ(frontierIds(o), std::set<UserId>{1});
    missing = std::find_if(o.cut.idealOnly.begin(), o.cut.idealOnly.end(),
                           [&](const FrontierEntry& e) {
        return !w.isResident(e.nodeHandle);
    });
    ASSERT_NE(missing, o.cut.idealOnly.end());
    EXPECT_EQ(payloadOf(w, *missing), 11u);
}

using ResultKey = std::tuple<uint32_t, UserPayload, uint8_t>;

std::vector<ResultKey> resultKeys(SpatialDatabase& world, const FrontierResultView& cut)
{
    std::vector<ResultKey> keys;
    const auto append = [&](const auto& entries, uint32_t bucket)
    {
        for (const FrontierEntry& e : entries)
            keys.emplace_back(bucket, payloadOf(world, e), e.errorCode());
    };
    append(cut.shared, 0);
    append(cut.currentOnly, 1);
    append(cut.idealOnly, 2);
    std::sort(keys.begin(), keys.end());
    return keys;
}

TEST(Streaming, ResidentDescendantsBypassMissingIntermediateProxies)
{
    PageBuilder builder;
    const auto root = builder.createRoot(
        1, 64.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0), float4::vec(8, 8, 8)));
    const auto intermediate = builder.createNode(
        root, 2, 16.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0), float4::vec(4, 4, 4)));
    builder.createNode(
        intermediate, 3, 0.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0), float4::vec(1, 1, 1)));

    SpatialDatabase world;
    world.addInstance(builder.build(), float4::point(0, 0, 0));
    markResident(world, 3);   // payload 2 deliberately remains non-resident

    const Camera view =
        makeLookAtCamera(float4::point(0, 0, -12), float4::point(0, 0, 0));
    const Outputs output = frame(world, view, {4.0f, 0.0f});

    EXPECT_FALSE(isResident(world, 2));
    EXPECT_EQ(frontierIds(output), std::set<UserId>{3});
    ASSERT_EQ(output.cut.shared.size(), 1u);
    EXPECT_TRUE(output.cut.currentOnly.empty());
    EXPECT_TRUE(output.cut.idealOnly.empty());
}

TEST(Streaming, InvisibleMissingBranchDoesNotBlockResidentCover)
{
    PageBuilder builder;
    const auto root = builder.createRoot(1, 128.0f);
    const auto visible = builder.createNode(
        root, 2, 32.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0), float4::vec(2, 2, 2)));
    builder.createNode(
        visible, 3, 0.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0), float4::vec(1, 1, 1)));
    const auto hidden = builder.createNode(
        root, 4, 32.0f,
        AABB::fromCenterExtent(float4::point(100, 0, 0), float4::vec(2, 2, 2)));
    builder.createNode(
        hidden, 5, 0.0f,
        AABB::fromCenterExtent(float4::point(100, 0, 0), float4::vec(1, 1, 1)));

    SpatialDatabase world;
    world.addInstance(builder.build(), float4::point(0, 0, 0));
    markResident(world, 3);   // the off-screen branch remains entirely missing

    const Camera view =
        makeLookAtCamera(float4::point(0, 0, -20), float4::point(0, 0, 0));
    const Outputs output = frame(world, view, {4.0f, 0.0f});

    EXPECT_EQ(frontierIds(output), std::set<UserId>{3});
    ASSERT_EQ(output.cut.shared.size(), 1u);
    EXPECT_TRUE(output.cut.currentOnly.empty());
    EXPECT_TRUE(output.cut.idealOnly.empty());
}

// ---------------------------------------------------------------------------
// The ideal frontier: nodes below the current frontier appear on its ideal-only side;
// the difference between the cuts is the prefetch target.
// ---------------------------------------------------------------------------
TEST(Streaming, IdealFrontierLeadsCurrentFrontier)
{
    TreeGen gen;
    gen.fanout = 2;
    gen.depth = 3;
    Page pg = gen.makeRootPage(unitRegion(30.0f), 64.0f, 0);
    const auto ids = pageIds(pg);

    SpatialDatabase w;
    w.addInstance(std::move(pg), float4::point(0, 0, 0));
    // Nothing resident except the pinned root.

    const Camera v = makeLookAtCamera(float4::point(0, 0, -50), float4::point(0, 0, 0));
    const auto o = frame(w, v, {4.0f, 0.0f});

    // Current cut: the root proxy only. Ideal cut: the deeper frontier.
    EXPECT_EQ(frontierIds(o), std::set<UserId>{ids.front()});
    EXPECT_GT(idealFrontierSize(o.cut), 1u);
    for (const auto& e : idealFrontier(o.cut))
        EXPECT_NE(payloadOf(w, e), ids.front()); // the root refines in the ideal frontier
}

// ---------------------------------------------------------------------------
// Expansion life cycle: shared while the collapsed proxy suffices; high-error
// while too coarse; descend after attach + residency; fall back after detach.
// ---------------------------------------------------------------------------
TEST(Streaming, ExpansionLifeCycle)
{
    TreeGen gen;
    gen.fanout = 2;
    gen.depth = 1;
    Page root = gen.makeRootPage(unitRegion(20.0f), 64.0f, 1);
    const auto rootIds = pageIds(root);

    SpatialDatabase w;
    w.addInstance(std::move(root), float4::point(0, 0, 0));
    markAllResident(w, rootIds);

    ASSERT_FALSE(gen.recipes.empty());

    const SelectionParams p{4.0f, 0.0f};

    // Far away: collapsed expansion points draw their proxies; steady state.
    {
        const Camera far = makeLookAtCamera(float4::point(0, 0, -100000), float4::point(0, 0, 0));
        const auto o = frame(w, far, p);
        for (const auto& e : idealFrontier(o.cut)) EXPECT_FALSE(e.overThreshold());
    }

    // Close: the expansion points are too coarse. Error plus the external
    // recipe graph is sufficient; there is no separate expansion tag.
    const Camera near = makeLookAtCamera(float4::point(0, 0, -45), float4::point(0, 0, 0));
    std::vector<UserId> needed;
    {
        const auto o = frame(w, near, p);
        for (const auto& e : idealFrontier(o.cut))
            if (e.overThreshold() && gen.recipes.count(payloadOf(w, e)))
                needed.push_back(payloadOf(w, e));
        ASSERT_FALSE(needed.empty());
        // Collapsed nodes still draw (they are leaves for now).
        const auto ids = frontierIds(o);
        for (UserId id : needed)
        {
            EXPECT_TRUE(ids.count(id));
            EXPECT_NE(std::find_if(o.cut.shared.begin(), o.cut.shared.end(),
                                   [&](const FrontierEntry& e) {
                                       return payloadOf(w, e) == id;
                                   }),
                      o.cut.shared.end());
        }
    }

    // Attach the pages: the caller can load the final ideal nodes directly;
    // intermediate proxy payloads are not required.
    for (UserId id : needed)
    {
        Page child = gen.makeChildPage(id);
        EXPECT_TRUE(attachPage(w, id, std::move(child)).valid());
        EXPECT_TRUE(isAttached(w, id));
    }
    {
        const auto o = frame(w, near, p);
        bool loaded = false;
        for (const FrontierEntry& entry : idealFrontier(o.cut))
            if (!w.isResident(entry.nodeHandle))
            {
                w.markResident(entry.nodeHandle);
                loaded = true;
            }
        EXPECT_TRUE(loaded);
    }
    {
        const auto o = frame(w, near, p);
        const auto ids = frontierIds(o);
        for (UserId id : needed) EXPECT_FALSE(ids.count(id));   // refined through
    }

    // Detach: one frame later the proxy is back. REPLACE means no holes.
    detachPage(w, needed[0]);
    EXPECT_FALSE(isAttached(w, needed[0]));
    {
        const auto o = frame(w, near, p);
        EXPECT_TRUE(frontierIds(o).count(needed[0]));
    }
}

TEST(Streaming, FullyResidentSummaryTracksAttachedMountTree)
{
    PageBuilder rootBuilder;
    const auto root = rootBuilder.createRoot(1, 64.0f);
    const auto expansion = rootBuilder.createNode(
        root, 2, 32.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0), float4::vec(2, 2, 2)));
    rootBuilder.markExpansion(expansion);
    Page rootPage = rootBuilder.build();
    const auto rootIds = pageIds(rootPage);

    SpatialDatabase world;
    world.addInstance(std::move(rootPage), float4::point(0, 0, 0));
    markAllResident(world, rootIds);
    EXPECT_TRUE(TA::fullyResidentTree(world, 1));

    PageBuilder childBuilder;
    childBuilder.createRoot(
        10, 0.0f,
        AABB::fromCenterExtent(float4::point(0, 0, 0), float4::vec(1, 1, 1)));
    Page childPage = childBuilder.build();
    const uint32_t childNodes = childPage.nodeCount();
    const PageHandle child = attachPage(world, 2, std::move(childPage));
    ASSERT_TRUE(child.valid());
    EXPECT_FALSE(TA::fullyResidentTree(world, 1));

    markAllResident(world, child, childNodes);
    EXPECT_TRUE(TA::fullyResidentTree(world, 1));

    const Camera view =
        makeLookAtCamera(float4::point(0, 0, -8), float4::point(0, 0, 0));
    const Outputs output = frame(world, view, {4.0f, 0.0f});
    EXPECT_TRUE(output.cut.currentOnly.empty());
    EXPECT_TRUE(output.cut.idealOnly.empty());
    EXPECT_EQ(frontierIds(output), std::set<UserId>{10});

    world.markNonResident(nodeAt(child, 1));
    EXPECT_FALSE(TA::fullyResidentTree(world, 1));
    world.markResident(nodeAt(child, 1));
    EXPECT_TRUE(TA::fullyResidentTree(world, 1));

    detachPage(world, 2);
    EXPECT_TRUE(TA::fullyResidentTree(world, 1));
}

// ---------------------------------------------------------------------------
// Attach-time clamp: invariant (D) across the page boundary is enforced by a
// per-mount scalar (errClamp), NOT by rewriting the child page's error array
// — page bytes are immutable at runtime. A child authored with a too-large
// error must behave exactly as one authored at the expansion node's error.
// ---------------------------------------------------------------------------
TEST(Streaming, AttachClampsChildErrors)
{
    auto makeRoot = [] {
        PageBuilder rb;
        const auto r = rb.createRoot(1, 8.0f);
        const auto e = rb.createNode(r, 2, 0.5f,   // deliberately small error
                                     AABB::fromCenterExtent(float4::vec(0, 0, 0),
                                                            float4::vec(2, 2, 2)));
        rb.markExpansion(e);
        return rb.build();
    };
    auto makeChild = [](float rootErr) {
        PageBuilder cb;
        cb.createRoot(10, rootErr,
                      AABB::fromCenterExtent(float4::vec(0, 0, 0), float4::vec(1, 1, 1)));
        return cb.build();
    };
    auto cutOf = [](SpatialDatabase& w) {
        markResident(w, 2);
        markResident(w, 10);
        // Close enough that the expansion node (err 0.5, box within 2 units)
        // always wants to refine into the attached page.
        const Camera v = makeLookAtCamera(float4::point(0, 0, -6), float4::point(0, 0, 0));
        return frame(w, v, {4.0f, 0.0f});
    };

    SpatialDatabase w;
    w.addInstance(makeRoot(), float4::point(0, 0, 0));
    attachPage(w, 2, makeChild(100.0f));   // way larger than the expansion node's 0.5

    // The page bytes are untouched; the clamp lives on the mount.
    const PageView& attached = TA::pageOf(w, 10);
    EXPECT_EQ(attached.geometricError[1], 100.0f);
    EXPECT_EQ(TA::errClampOf(w, 10), 0.5f);

    // Behaviorally the over-erroneous child is indistinguishable from one
    // authored at the clamp: same cut, same reported errors.
    SpatialDatabase wRef;
    wRef.addInstance(makeRoot(), float4::point(0, 0, 0));
    attachPage(wRef, 2, makeChild(0.5f));

    const Outputs got = cutOf(w), want = cutOf(wRef);
    ASSERT_FALSE(got.cut.empty());
    EXPECT_EQ(resultKeys(w, got.cut), resultKeys(wRef, want.cut));
    EXPECT_TRUE(frontierIds(got).count(10));   // the walk did reach the child
}

// ---------------------------------------------------------------------------
// Garbage collection: LRU + watermark + dwell; pinned and in-use pages
// survive; child pages collapse before their parents; freed ids reported.
// ---------------------------------------------------------------------------
TEST(Streaming, GarbageCollection)
{
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 1;
    Page root = gen.makeRootPage(unitRegion(400.0f), 512.0f, 2);
    const auto rootIds = pageIds(root);

    SpatialDatabase w;
    w.addInstance(std::move(root), float4::point(0, 0, 0));
    markAllResident(w, rootIds);

    // Attach all level-1 pages, and one level-2 page under the LAST of them
    // (so the deep page is far from the region we will look at).
    std::vector<UserId> level1;
    for (const auto& [id, rec] : gen.recipes)
        if (contains(w, id)) level1.push_back(id);
    std::sort(level1.begin(), level1.end());
    std::vector<UserId> level2;
    std::vector<UserId> pageProbe;
    for (UserId id : level1)
    {
        Page child = gen.makeChildPage(id);
        const auto ids = pageIds(child);
        pageProbe.push_back(ids.front());
        attachPage(w, id, std::move(child));
        markAllResident(w, ids);
        std::vector<UserId> exps;
        for (UserId c : ids)
            if (gen.recipes.count(c)) exps.push_back(c);
        if (!exps.empty()) level2 = exps;   // keep the last page's expansions
    }
    ASSERT_FALSE(level2.empty());
    {
        Page grand = gen.makeChildPage(level2[0]);
        const auto ids = pageIds(grand);
        attachPage(w, level2[0], std::move(grand));
        markAllResident(w, ids);
    }
    const size_t allAttached = w.attachedPageCount();   // 1 root + 4 + 1

    // Look INTO the first slab, facing away from all the other slabs, so only
    // level1[0]'s page (plus the root page) is touched by the walk.
    const AABB region0 = gen.recipes.at(level1[0]).region;
    const float4 c0 = region0.center();
    const Camera v = makeLookAtCamera(c0 + float4::vec(60, 0, 0),
                                      c0 - float4::vec(150, 0, 0));
    const AABB region1 = gen.recipes.at(level1[1]).region;
    const float4 c1 = region1.center();
    const Camera shadow = makeLookAtCamera(c1 + float4::vec(60, 0, 0),
                                           c1 - float4::vec(150, 0, 0));
    FrontierResult cut;
    FrontierResult shadowCut;
    SpatialQuery selection;
    SpatialQuery shadowQuery;
    shadowQuery.setReuseEnabled(false);
    PageUsageContext usage;
    for (int f = 0; f < 10; ++f)
    {
        w.applyUpdates();
        selection.selectFrontier(w, v, {0.5f, 0.0f}, usage, cut);
        shadowQuery.selectFrontier(w, shadow, {0.5f, 0.0f}, shadowCut);
    }
    // Selection only accumulates feedback. The SpatialDatabase LRU is updated when the
    // caller explicitly chooses this camera at collect time.
    EXPECT_LT(TA::lastTouched(w, pageProbe[0]), w.frame());
    PageUsageContext* usageList[] = {&usage};
    EXPECT_EQ(w.collect(usageList, 1, 100).detachedPages, 0u);
    EXPECT_EQ(TA::lastTouched(w, pageProbe[0]), w.frame());
    EXPECT_EQ(w.attachedPageCount(), allAttached);

    // With a small dwell, unseen pages collapse; the viewed page, its
    // ancestors, and the pinned root survive. freed ids are reported.
    const CollectResult collected = w.collect(2, 3);
    EXPECT_GT(collected.detachedPages, 0u);
    EXPECT_FALSE(collected.freedPayloads.empty());
    EXPECT_TRUE(isAttached(w, level1[0]));           // still in view
    EXPECT_FALSE(isAttached(w, level1[1]));          // shadow view was untracked
    for (UserId id : rootIds) EXPECT_TRUE(contains(w, id));   // pinned root intact

    // The unseen level-1 pages are gone, and the level-2 page went first
    // (it can only be collected before/with its parent, never after).
    EXPECT_FALSE(isAttached(w, level2[0]));

    // A page the cut still needs is touched every frame and can never
    // be collected, no matter how tight the budget.
    for (int f = 0; f < 10; ++f)
    {
        w.applyUpdates();
        selection.selectFrontier(w, v, {0.5f, 0.0f}, usage, cut);
        w.collect(usage, 0, 3);
    }
    EXPECT_TRUE(isAttached(w, level1[0]));
}

// ---------------------------------------------------------------------------
// collect() contract, exactly:
//   - the minAge dwell is a hard boundary — a page untouched for minAge-1
//     frames survives, at exactly minAge it is eligible;
//   - freedPayloads reports precisely the resident payloads of the collected
//     pages: no duplicates, nothing from surviving pages, nothing that was
//     never resident.
// ---------------------------------------------------------------------------
TEST(Streaming, CollectMinAgeBoundaryAndExactFreedPayloads)
{
    TreeGen gen;
    gen.fanout = 4;
    gen.depth = 1;
    Page root = gen.makeRootPage(unitRegion(100.0f), 256.0f, 1);
    const auto rootIds = pageIds(root);

    SpatialDatabase w;
    w.addInstance(std::move(root), float4::point(0, 0, 0));
    markAllResident(w, rootIds);

    std::vector<UserId> exps;
    for (const auto& [id, rec] : gen.recipes)
        if (contains(w, id)) exps.push_back(id);
    std::sort(exps.begin(), exps.end());
    ASSERT_FALSE(exps.empty());

    // Attach one page per expansion; mark every other node resident and
    // remember exactly which — that is the whole expected freed set.
    std::set<UserId> expectFreed;
    for (UserId id : exps)
    {
        Page child = gen.makeChildPage(id);
        const auto ids = pageIds(child);
        attachPage(w, id, std::move(child));
        for (size_t i = 0; i < ids.size(); i += 2)
        {
            markResident(w, ids[i]);
            expectFreed.insert(ids[i]);
        }
    }

    const uint32_t minAge = 5;
    for (uint32_t f = 0; f + 1 < minAge; ++f) w.applyUpdates();

    // One frame short of the dwell: nothing may be collected.
    const CollectResult beforeBoundary = w.collect(0, minAge);
    EXPECT_EQ(beforeBoundary.detachedPages, 0u);
    EXPECT_TRUE(beforeBoundary.freedPayloads.empty());
    for (UserId id : exps) EXPECT_TRUE(isAttached(w, id));

    // At exactly minAge every streamed page is eligible; the freed report is
    // exactly the resident payloads of the collected pages.
    w.applyUpdates();
    const CollectResult atBoundary = w.collect(0, minAge);
    EXPECT_EQ(atBoundary.detachedPages, exps.size());
    const std::set<UserId> freedSet(atBoundary.freedPayloads.begin(),
                                    atBoundary.freedPayloads.end());
    EXPECT_EQ(freedSet.size(), atBoundary.freedPayloads.size())
        << "duplicate payloads in freed report";
    EXPECT_EQ(freedSet, expectFreed);
    for (UserId id : exps) EXPECT_FALSE(isAttached(w, id));

    // The pinned root page is untouchable regardless of age or budget.
    for (UserId id : rootIds) EXPECT_TRUE(contains(w, id));
}

// ---------------------------------------------------------------------------
// GC churn stress: a fast camera sweeps a dense paged world while pages are
// attached ahead and collected behind it every frame; the sweep wraps so
// collapsed regions get re-expanded. The optimized walk must stay equivalent
// to the brute-force reference throughout (stale scratch, LRU state and slot
// reuse must never leak into results).
// ---------------------------------------------------------------------------
TEST(Streaming, GcChurnStress)
{
    TreeGen gen;
    gen.fanout = 3;
    gen.depth = 1;
    Page root = gen.makeRootPage(unitRegion(500.0f), 4096.0f, 3);
    const auto rootIds = pageIds(root);

    SpatialDatabase w;
    w.addInstance(std::move(root), float4::point(0, 0, 0));
    markAllResident(w, rootIds);

    const SelectionParams p{4.0f, 0.0f};
    FrontierResult cut;
    PageUsageContext usage;

    size_t attaches = 0, collected = 0;
    for (int f = 0; f < 120; ++f)
    {
        // Two full passes over the world at high speed, low altitude.
        const float x = -500.0f + 1000.0f * float(f % 60) / 60.0f;
        const Camera v = makePerspectiveCamera(
            float4::point(x, 40.0f, 0), float4::vec(1.0f, -0.3f, 0.0f),
            float4::vec(0, 1, 0), 1.0f, 16.0f / 9.0f, 1080.0f, 0.1f, 1.0e9f);

        w.applyUpdates();
        selectFrontierUncached(w, v, p, usage, cut);

        if (f % 7 == 0)   // spot-check equivalence on the exact same state
        {
            const RefResult want = TA::referenceFrontier(w, v, p);
            EXPECT_EQ(resultKeys(w, cut), resultKeys(w, want.cut))
                << "frame " << f;
        }

        int budget = 8;
        for (const auto& e : idealFrontier(cut))
        {
            if (budget <= 0 || !e.overThreshold()) continue;
            const UserId id = payloadOf(w, e);
            if (!gen.recipes.count(id) || w.isAttached(e.nodeHandle)) continue;
            // Production flow: content keyed by payload, attach by handle.
            Page child = gen.makeChildPage(id);
            const uint32_t n = child.nodeCount();
            const PageHandle ph = w.attachPage(e.nodeHandle, std::move(child));
            ASSERT_TRUE(ph.valid());
            markAllResident(w, ph, n);
            --budget;
            ++attaches;
        }
        for (const FrontierEntry& entry : idealFrontier(cut))
        {
            const UserId id = payloadOf(w, entry);
            const bool expandable = entry.overThreshold() &&
                                    gen.recipes.count(id) &&
                                    !w.isAttached(entry.nodeHandle);
            if (!expandable && !w.isResident(entry.nodeHandle))
                w.markResident(entry.nodeHandle);
        }
        collected += w.collect(usage, 6, 2).detachedPages;
    }

    // The stress actually stressed: pages were expanded, collapsed, and
    // re-expanded, and GC kept the attached set near its budget.
    EXPECT_GT(attaches, 40u);
    EXPECT_GT(collected, 20u);
    EXPECT_LT(w.attachedPageCount(), 40u);
    for (UserId id : rootIds) EXPECT_TRUE(contains(w, id));   // pinned root intact
}

// ---------------------------------------------------------------------------
// Streaming loop toward the ideal frontier: consuming ideal entries frame by frame
// converges, and the ideal frontier equals the current frontier at the fixed point.
// ---------------------------------------------------------------------------
TEST(Streaming, ConvergesToIdealFrontier)
{
    TreeGen gen;
    gen.fanout = 3;
    gen.depth = 2;
    Page root = gen.makeRootPage(unitRegion(50.0f), 64.0f, 1);

    SpatialDatabase w;
    w.addInstance(std::move(root), float4::point(0, 0, 0));

    const Camera v = makeLookAtCamera(float4::point(0, 10, -80), float4::point(0, 0, 0));
    const SelectionParams p{4.0f, 0.0f};

    Outputs o;
    for (int f = 0; f < 64; ++f)
    {
        o = frame(w, v, p);
        bool progress = false;
        for (const FrontierEntry& entry : idealFrontier(o.cut))
        {
            const UserId id = payloadOf(w, entry);
            if (entry.overThreshold() && gen.recipes.count(id) &&
                !w.isAttached(entry.nodeHandle))
            {
                Page child = gen.makeChildPage(id);
                w.attachPage(entry.nodeHandle, std::move(child));
                progress = true;
            }
            else if (!w.isResident(entry.nodeHandle))
            {
                w.markResident(entry.nodeHandle);
                progress = true;
            }
        }
        if (!progress) break;
    }

    // Fixed point: no missing ideal payloads or unattached pages; ideal == current.
    std::set<UserId> ideal, current = frontierIds(o);
    for (const auto& e : idealFrontier(o.cut))
    {
        EXPECT_TRUE(w.isResident(e.nodeHandle));
        ideal.insert(payloadOf(w, e));
    }
    EXPECT_EQ(ideal, current);
}

// ---------------------------------------------------------------------------
// API misuse is rejected.
// ---------------------------------------------------------------------------
TEST(Streaming, ApiContracts)
{
    TreeGen gen;
    Page root = gen.makeRootPage(unitRegion(10.0f), 16.0f, 1);
    const auto rootIds = pageIds(root);

    SpatialDatabase w;
    w.addInstance(std::move(root), float4::point(0, 0, 0));

    // Contract violations on LIVE handles throw.
    EXPECT_THROW(markNonResident(w, rootIds.front()), std::logic_error);    // pinned root
    EXPECT_THROW(attachPage(w, rootIds.front(), Page{}), std::logic_error); // not expansion

    UserId expId = gen.recipes.begin()->first;
    Page child = gen.makeChildPage(expId);
    attachPage(w, expId, std::move(child));
    EXPECT_THROW(attachPage(w, expId, gen.makeChildPage(expId)), std::logic_error);

    // STALE handles are not errors — the normal race between streaming
    // completion and GC. Mutating calls no-op, queries report absence, and
    // attach is rejected by returning an invalid PageHandle.
    const NodeHandle stale{9999, 1, 424242};
    w.markResident(stale);
    w.markNonResident(stale);
    w.detachPage(stale);
    EXPECT_FALSE(w.isResident(stale));
    EXPECT_FALSE(w.isAttached(stale));
    EXPECT_FALSE(w.attachPage(stale, gen.makeChildPage(expId)).valid());
}
