#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <thread>
#include <vector>

#include "helpers.h"

using namespace frontier;
using namespace frontiertest;

namespace {

class DeterministicRng
{
public:
    explicit DeterministicRng(uint32_t seed) : state_(seed) {}

    uint32_t next()
    {
        uint32_t x = state_;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        state_ = x;
        return x;
    }

    uint32_t bounded(uint32_t limit) { return limit ? next() % limit : 0; }

    float coordinate()
    {
        return float(int32_t(bounded(201)) - 100);
    }

private:
    uint32_t state_;
};

void threadedParallelFor(uint32_t count,
                         void (*fn)(uint32_t, void*),
                         void* payload, void*)
{
    std::vector<std::thread> threads;
    threads.reserve(count);
    for (uint32_t i = 0; i < count; ++i)
        threads.emplace_back([=] { fn(i, payload); });
    for (std::thread& thread : threads) thread.join();
}

bool sameEntries(std::span<const FrontierEntry> a,
                 std::span<const FrontierEntry> b)
{
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i)
        if (a[i].nodeHandle != b[i].nodeHandle ||
            a[i].instanceAndError != b[i].instanceAndError)
            return false;
    return true;
}

} // namespace

TEST(Torture, RandomizedTlasChurnMatchesTheLiveInstanceModel)
{
    struct Record
    {
        InstanceHandle handle;
        UserPayload payload = 0;
        uint32_t mask = 0;
        bool alive = false;
    };

    SpatialDatabase database;
    DeterministicRng rng(0x91e10da5u);
    std::vector<Record> records;
    uint32_t liveCount = 0;
    UserPayload nextPayload = 1000;

    const auto spawn = [&]
    {
        InstanceDesc desc;
        desc.pos = float4::point(rng.coordinate(), rng.coordinate(), 0.0f);
        desc.scale = std::array<float, 3>{0.5f, 1.0f, 2.0f}[
            rng.bounded(3)];
        desc.mask = 1u << rng.bounded(3);
        const UserPayload payload = nextPayload++;
        records.push_back({database.instantiate(
                               node(payload, 0.0f, box(0.25f)), desc),
                           payload, desc.mask, true});
        ++liveCount;
    };

    const auto pickLive = [&]() -> size_t
    {
        for (;;)
        {
            const size_t index = rng.bounded(uint32_t(records.size()));
            if (records[index].alive) return index;
        }
    };

    for (uint32_t i = 0; i < 48; ++i) spawn();

    SpatialQuery cached;
    SpatialQuery uncached;
    uncached.setReuseEnabled(false);
    Camera camera = cameraAt(-1000.0f);

    const auto verify = [&](uint32_t step)
    {
        camera.viewMask = 1u << rng.bounded(3);
        database.applyUpdates();

        std::vector<UserPayload> expected;
        for (const Record& record : records)
            if (record.alive && (record.mask & camera.viewMask) != 0)
                expected.push_back(record.payload);
        std::sort(expected.begin(), expected.end());

        const std::vector<UserPayload> cachedPayloads = payloads(
            database, cached.selectFrontier(database, camera, {}), false);
        const std::vector<UserPayload> uncachedPayloads = payloads(
            database, uncached.selectFrontier(database, camera, {}), false);
        EXPECT_EQ(cachedPayloads, expected) << "step " << step;
        EXPECT_EQ(uncachedPayloads, expected) << "step " << step;
        EXPECT_EQ(TestAccess::liveInstanceSlots(database), liveCount)
            << "step " << step;
    };

    for (uint32_t step = 0; step < 2000; ++step)
    {
        const uint32_t operation = rng.bounded(100);
        if (liveCount < 24 || (liveCount < 192 && operation < 34))
        {
            spawn();
        }
        else if (operation < 67)
        {
            Record& record = records[pickLive()];
            database.moveInstance(
                record.handle,
                Transform{float4::point(rng.coordinate(), rng.coordinate(), 0),
                          std::array<float, 3>{0.5f, 1.0f, 2.0f}[
                              rng.bounded(3)]});
        }
        else if (operation < 88)
        {
            Record& record = records[pickLive()];
            const InstanceHandle stale = record.handle;
            database.removeInstance(stale);
            database.removeInstance(stale);
            database.moveInstance(
                stale, Transform{float4::point(500, 0, 0), 1.0f});
            EXPECT_EQ(database.tryGetPayload(stale.rootNode()),
                      kInvalidPayload);
            record.alive = false;
            --liveCount;
        }
        else
        {
            database.optimize();
        }

        if ((step % 13) == 0) verify(step);
    }
    verify(2000);
}

TEST(Torture, RandomizedReadinessMaintainsACompleteRenderableCover)
{
    struct ModelNode
    {
        UserPayload payload = 0;
        std::vector<uint32_t> children;
        bool ready = false;
    };

    SpatialDatabase database;
    SubtreeBuilder builder;
    std::vector<ModelNode> model;

    std::function<uint32_t(SubtreeBuilder::NodeId, uint32_t)> addNode =
        [&](SubtreeBuilder::NodeId parent, uint32_t depth)
    {
        const uint32_t index = uint32_t(model.size());
        const UserPayload payload = 100 + index;
        model.push_back({payload, {}, false});
        const NodeDesc desc = node(payload, depth ? 64.0f : 0.0f,
                                   box(4.0f));
        const SubtreeBuilder::NodeId id =
            parent == kInvalidIndex ? builder.createNode(desc)
                                    : builder.createNode(parent, desc);
        if (depth)
        {
            const uint32_t left = addNode(id, depth - 1);
            const uint32_t right = addNode(id, depth - 1);
            model[index].children = {left, right};
        }
        return index;
    };

    const uint32_t modelRoot = addNode(kInvalidIndex, 4);
    const SubtreeHandle definition =
        database.registerSubtree(builder.build());
    const InstanceHandle instance = database.instantiate(
        node(1, 128.0f, box(4.0f), true));
    SubtreeInstanceHandle placement =
        database.mountSubtree(instance.rootNode(), definition);
    ASSERT_TRUE(placement.valid());

    std::vector<NodeHandle> handles(model.size());
    const auto refreshHandles = [&]
    {
        for (size_t i = 0; i < model.size(); ++i)
            handles[i] = TestAccess::requireNode(database, model[i].payload);
    };
    refreshHandles();

    std::vector<UserPayload> expectedIdeal;
    for (const ModelNode& entry : model)
        if (entry.children.empty()) expectedIdeal.push_back(entry.payload);
    std::sort(expectedIdeal.begin(), expectedIdeal.end());

    const std::function<bool(uint32_t, std::vector<UserPayload>&)> cover =
        [&](uint32_t index, std::vector<UserPayload>& output)
    {
        const ModelNode& entry = model[index];
        if (entry.children.empty())
        {
            if (!entry.ready) return false;
            output.push_back(entry.payload);
            return true;
        }

        std::vector<UserPayload> descendants;
        bool complete = true;
        for (const uint32_t child : entry.children)
            if (!cover(child, descendants)) complete = false;
        if (complete)
        {
            output.insert(output.end(), descendants.begin(), descendants.end());
            return true;
        }
        if (!entry.ready) return false;
        output.push_back(entry.payload);
        return true;
    };

    SpatialQuery query;
    SelectionParams params{.threshold = 1.0f};
    const Camera camera = cameraAt(-20.0f);
    DeterministicRng rng(0xc001d00du);

    for (uint32_t step = 0; step < 1600; ++step)
    {
        const uint32_t changed = rng.bounded(uint32_t(model.size()));
        model[changed].ready = !model[changed].ready;
        if (model[changed].ready)
            database.markNodeReady(handles[changed]);
        else
            database.markNodeUnavailable(handles[changed]);

        if (step != 0 && (step % 137) == 0)
        {
            const NodeHandle stale = handles[rng.bounded(
                uint32_t(handles.size()))];
            database.unmountSubtree(placement);
            EXPECT_EQ(database.tryGetPayload(stale), kInvalidPayload);
            placement = database.mountSubtree(
                instance.rootNode(), definition);
            ASSERT_TRUE(placement.valid());
            refreshHandles();
        }

        std::vector<UserPayload> expectedCurrent;
        if (!cover(modelRoot, expectedCurrent)) expectedCurrent.push_back(1);
        std::sort(expectedCurrent.begin(), expectedCurrent.end());

        const FrontierResultView result =
            select(database, query, camera, params);
        EXPECT_EQ(payloads(database, result, false), expectedCurrent)
            << "step " << step;
        EXPECT_EQ(payloads(database, result, true), expectedIdeal)
            << "step " << step;
    }
}

TEST(Torture, ParallelAndSerialSelectionAreBitIdentical)
{
    SpatialDatabaseConfig config;
    config.context.workerCount = 4;
    config.context.parallelFor = &threadedParallelFor;
    config.parallelInstanceThreshold = 1;
    SpatialDatabase database(config);

    constexpr uint32_t count = 256;
    for (uint32_t i = 0; i < count; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(int(i % 32) - 16) * 2.0f,
                                 float(int(i / 32) - 4) * 2.0f, 0.0f);
        desc.mask = 1u << (i % 3);
        database.instantiate(node(1000 + i, float(i % 5), box(0.5f)),
                             desc);
    }
    database.applyUpdates();

    Camera camera = cameraAt(-1000.0f);
    camera.viewMask = 0x3;
    SpatialQuery serial;
    SpatialQuery parallel;
    parallel.setReuseEnabled(false);
    const FrontierResultView serialResult =
        serial.selectFrontier(database, camera, {});
    const FrontierResultView parallelResult =
        parallel.selectFrontier(database, camera, {});

    EXPECT_TRUE(sameEntries(serialResult.shared, parallelResult.shared));
    EXPECT_TRUE(sameEntries(serialResult.currentOnly,
                            parallelResult.currentOnly));
    EXPECT_TRUE(sameEntries(serialResult.idealOnly,
                            parallelResult.idealOnly));
}

TEST(Torture, IndependentQueriesReadOnePublishedSnapshotConcurrently)
{
    SpatialDatabase database;
    constexpr uint32_t count = 192;
    for (uint32_t i = 0; i < count; ++i)
    {
        InstanceDesc desc;
        desc.pos = float4::point(float(int(i % 24) - 12) * 2.0f,
                                 float(int(i / 24) - 4) * 2.0f, 0.0f);
        database.instantiate(node(1000 + i, 0.0f, box(0.5f)), desc);
    }
    database.applyUpdates();
    const Camera camera = cameraAt(-1000.0f);
    std::atomic<bool> valid{true};

    std::vector<std::thread> readers;
    for (uint32_t threadIndex = 0; threadIndex < 8; ++threadIndex)
    {
        readers.emplace_back([&, threadIndex]
        {
            try
            {
                SpatialQuery query;
                query.setReuseEnabled((threadIndex & 1u) == 0);
                for (uint32_t iteration = 0; iteration < 100; ++iteration)
                {
                    const FrontierResultView result =
                        query.selectFrontier(database, camera, {});
                    if (result.shared.size() != count ||
                        !result.currentOnly.empty() ||
                        !result.idealOnly.empty())
                    {
                        valid.store(false, std::memory_order_relaxed);
                        return;
                    }
                    for (const FrontierEntry& entry : result.shared)
                    {
                        if (database.tryGetPayload(entry.nodeHandle) ==
                            kInvalidPayload)
                        {
                            valid.store(false, std::memory_order_relaxed);
                            return;
                        }
                    }
                }
            }
            catch (...)
            {
                valid.store(false, std::memory_order_relaxed);
            }
        });
    }
    for (std::thread& reader : readers) reader.join();
    EXPECT_TRUE(valid.load(std::memory_order_relaxed));
}
