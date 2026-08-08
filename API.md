# HLodTree public API guide

This is the integration reference for HLodTree 0.4.0. It covers the public
types in `include/hlod`, their ownership rules, and the frame lifecycle. See
[README.md](README.md) for a short introduction and [hlod_design.md](hlod_design.md)
for the behavioral invariants behind the API.

HLodTree requires C++20. Runtime users normally include `hlod/world.h`;
content-building tools also include `hlod/builder.h`.

## Complete example

The example builds a small hierarchy, registers it once, creates two
instances, selects a render cut, and inspects the ideal frontier for streaming.
The application-specific renderer and content loader are represented by
functions with descriptive names.

```cpp
#include "hlod/builder.h"
#include "hlod/world.h"

#include <array>
#include <utility>

using namespace hlod;

Page buildTreePage()
{
    HLodBuilder builder;
    const auto tree = builder.createRoot(
        100, 16.0f,
        AABB::fromCenterExtent(float4::point(0, 4, 0),
                               float4::vec(4, 4, 4)));

    builder.createNode(
        tree, 101, 2.0f,
        AABB::fromCenterExtent(float4::point(-2, 2, 0),
                               float4::vec(2, 2, 2)));
    builder.createNode(
        tree, 102, 2.0f,
        AABB::fromCenterExtent(float4::point(2, 2, 0),
                               float4::vec(2, 2, 2)));
    return builder.build();
}

int main()
{
    World world;
    const AssetHandle treeAsset = world.registerAsset(buildTreePage());

    const World::InstanceRef first =
        world.addInstance(treeAsset, float4::point(0, 0, 0));
    const World::InstanceRef second =
        world.addInstance(treeAsset, float4::point(20, 0, 0));

    View mainView(4.0f);              // four-frame LOD damping half-life
    PageUsageContext mainUsage;       // this view influences page retention

    const Camera camera = makeLookAtCamera(
        float4::point(10, 8, -30), float4::point(10, 3, 0));

    world.applyUpdates();             // publish the selection snapshot
    const World& published = world;
    const CutView cut = mainView.selectCut(
        published, camera, CutParams{4.0f, 0.0f}, mainUsage);

    const auto draw = [&](const CutEntry& entry)
    {
        UserPayload payload;
        if (published.tryGetPayload(entry.nodeHandle, payload))
            submitToRenderer(payload, entry.instance());
    };

    for (const CutEntry& entry : cut.shared) draw(entry);
    for (const CutEntry& entry : cut.currentOnly) draw(entry);

    // Every selection has finished. World mutation is serial again.
    const auto considerForStreaming = [&](const CutEntry& entry)
    {
        UserPayload payload;
        if (!world.tryGetPayload(entry.nodeHandle, payload)) return;

        if (entry.overThreshold() && contentHasChildPage(payload) &&
            !world.isAttached(entry.nodeHandle))
        {
            Page child = loadChildPage(payload);
            world.attachPage(entry.nodeHandle, std::move(child));
        }
        else if (!world.isResident(entry.nodeHandle) &&
                 payloadIsLoaded(payload))
        {
            world.markResident(entry.nodeHandle);
        }
    };

    for (const CutEntry& entry : cut.shared) considerForStreaming(entry);
    for (const CutEntry& entry : cut.idealOnly) considerForStreaming(entry);

    const CollectResult collected = world.collect(mainUsage, 4096, 120);
    for (UserPayload payload : collected.freedPayloads)
        releaseRenderPayload(payload);

    world.removeInstance(second);
    world.removeInstance(first);
    world.releaseAsset(treeAsset);
}
```

In production, page and payload loads normally complete in later frames. The
host deduplicates requests and applies IO budgets; HLodTree reports the ideal
frontier but does not own an asynchronous loader.

## Frame lifecycle and threading

Use this order for every published snapshot:

1. Apply instance, bounds, topology, and residency mutations to `World`.
2. Call `world.applyUpdates()` once, even if no objects changed.
3. Treat the world as `const` and run selections. Distinct `View` objects may
   select concurrently; each call must also have distinct output storage and,
   when supplied, a distinct `PageUsageContext`.
4. Join every selection before mutating the world or calling `collect()`.

`World` is single-writer and is not internally synchronized. A `View` binds to
the first world it queries and is itself mutable, so the same `View` must not be
used concurrently. `View::reset()` releases that binding.

For parallelism within one uncached view, provide a blocking
`HlodContext::parallelFor`, set `workerCount` above one, set
`WorldConfig::parallelInstanceThreshold`, and call
`view.setReuseEnabled(false)`. The callback may execute tasks in any order but
must not return until all tasks have completed.

## Authoring pages

`HLodBuilder` builds one immutable page at a time:

```cpp
HLodBuilder builder;
const HLodBuilder::NodeId root =
    builder.createRoot(rootPayload, rootError, rootBounds);
const HLodBuilder::NodeId child =
    builder.createNode(root, childPayload, childError, childBounds);
builder.markExpansion(child);  // child remains renderable and has no local children
Page page = builder.build(context);
```

`createRoot()` adds a root to the page forest. `createNode()` adds a child to
an authoring node. Insertion order is arbitrary. `NodeId` is valid only while
authoring that builder; `build()` consumes the builder and packs nodes into
preorder, so it is not a persistent runtime node index.

Every leaf needs non-empty bounds. The builder unions child bounds into their
ancestors, clamps child geometric error so it never exceeds the parent's
effective error, verifies the page contract, and emits the versioned blob.
Every real node needs a renderable `UserPayload`, including expansion points.
Payload values need not be unique.

An expansion point is a leaf whose children live in another page. The attached
child page's root bounds must fit inside the expansion point's authored bounds.
Author those bounds conservatively because attaching shared topology does not
grow the parent.

### Owned and borrowed page data

`Page` is move-only and owns one aligned blob. `PageView` is a non-owning view
of a blob.

- `Page::fromBytes(blob, bytes, context)` validates and copies external data.
- `PageView::fromBytes(blob, bytes)` validates and borrows external data.
- `Page::adopt(blob, bytes, context)` takes ownership of storage allocated by
  that context.
- `Page::clone(context)` makes an explicit owned copy.

The byte range at `page.data()` with length `page.byteSize()` is the serialized
page format. It is little-endian and versioned; readers must validate it rather
than assuming compatibility with another `kPageVersion`.

Blob pointers passed to `PageView::fromBytes()`, `Page::fromBytes()`, or
`Page::adopt()` must be suitably aligned; `kPageAlign` is the portable storage
alignment. Borrowed storage must remain alive and unchanged until the
registered asset is released. An `HlodContext` that allocated an owned `Page`
must outlive the page, including after the page moves into a `World`.

## Assets and instances

Register reusable page data once:

```cpp
AssetHandle owned = world.registerAsset(std::move(page));
AssetHandle mapped = world.registerAsset(PageView::fromBytes(data, size));
```

The owned overload transfers the page. The borrowed overload does not copy.
All instances of a root asset share its immutable page data, residency state,
and attached child-page graph.

Create instances with translation, positive uniform scale, and an optional
layer mask:

```cpp
InstanceDesc desc;
desc.pos = float4::point(10, 0, 5);
desc.scale = 2.0f;
desc.mask = 1u << 3;
World::InstanceRef ref = world.addInstance(asset, desc);
```

An instance is visible only when `desc.mask & camera.viewMask` is nonzero.
Rotation and non-uniform scale are not part of `InstanceDesc`; bake them into
authored data or adapt them outside the library.

`addInstance(Page&&, ...)` is a convenience for one-off content. Repeated
content should use `registerAsset()` so its pages and streaming state are
shared. `removeInstance()` and `moveInstance()` ignore stale references.
`releaseAsset()` requires that no live instances still reference the asset.

`assetRootPage()` returns the shared root mount after at least one instance has
materialized it. `InstanceRef::rootPage` provides the same kind of handle
without another lookup.

### Moving persistent cohorts

For a fixed cohort that moves every frame, retain a `MotionGroup`:

```cpp
std::array<World::InstanceRef, 2> refs{first, second};
World::MotionGroup group(refs);

std::array<float4, 2> positions{
    float4::point(1, 0, 0),
    float4::point(21, 0, 0),
};
world.moveInstances(group, positions);
```

Positions use the original group order. Stale references are ignored and the
last position wins for duplicates. The group caches the world's physical
order and refreshes that cache after a layout change. Use `moveInstance()` for
isolated moves or cohorts whose membership changes continually.

The first TLAS build establishes spatial physical order. Routine updates
preserve it. Call `world.optimize()` at an occasional synchronization point
after disruptive motion or heavy spawn/despawn activity. It flushes pending
bounds, compacts dead slots, rebuilds the quality TLAS, and restores spatial
instance and view-record locality. Public handles and instance ids remain
stable. It is intended for loading screens, menus, teleports, and level
transitions rather than per-frame maintenance.

## Handles and lifetime

The API uses generation-stamped handles:

| Type | Names | Invalidated by |
|---|---|---|
| `AssetHandle` | a registered page asset | `releaseAsset()` |
| `PageHandle` | one mounted page | detaching or collecting that mount |
| `NodeHandle` | one packed node in a mount | invalidation of its page mount |
| `World::InstanceRef` | one live instance slot | `removeInstance()` |

Use `nodeAt(pageHandle, packedIndex)` only when the packed page-local index is
known. Handles returned in `CutEntry` are normally the simplest route for
streaming and residency operations.

A stale handle is an expected asynchronous race. Mutating calls that document
stale tolerance ignore it; queries such as `isResident()` and
`tryGetPayload()` report absence. `attachPage()` returns an invalid
`PageHandle` when its expansion node went stale while the child was loading.

`CutEntry::instance()` is a compact id for the published snapshot and is
suited to indexing caller-side transform or entity tables during that
selection phase. Retain `InstanceRef`, not the bare id, for later mutation.

## Topology and residency

Attach deeper topology under a live expansion node:

```cpp
PageHandle childMount = world.attachPage(expansionNode, childAsset);
// Or transfer a one-off page:
PageHandle anonymousChild =
    world.attachPage(expansionNode, std::move(childPage));
```

Attaching under a shared asset makes that topology available to every instance
of the asset. `detachPage()` collapses the branch. It is a no-op for a stale
handle. A live node that is not an expansion point, an already attached
expansion point, or invalid child-page bounds is a contract violation.

Topology and payload residency are independent. Use `markResident()` and
`markNonResident()` when a node's render payload becomes available or is
evicted. Root payloads are pinned resident. Coverage propagates incrementally,
so the current cut refines only when resident descendants cover the required
visible region.

`tryGetPayload(handle, out)` resolves the immutable `uint64_t` application
payload of a live node. The world does not interpret or index payloads.

## Selection and result ownership

`CutParams::threshold` is the permitted projected geometric error in pixels.
Selection refines above it. `minPix` optionally culls a whole instance when
its maximum projected contribution is smaller than that value; zero disables
this culling.

The simplest query returns `CutView`:

```cpp
CutView cut = view.selectCut(publishedWorld, camera, params);
```

Its three spans are disjoint:

- `shared` belongs to both cuts;
- `currentOnly` contains resident fallbacks needed only for rendering now;
- `idealOnly` belongs only to the fully-resident ideal frontier.

Render `shared + currentOnly`. Inspect `shared + idealOnly` for payload and
topology work. Output order is traversal-defined, not priority order.

The spans refer to storage owned by the `View` and expire on its next
selection, `reset()`, or destruction. Use the explicit owning snapshot when a
cut must survive another query on the same view:

```cpp
CutResults retained;
view.selectCut(publishedWorld, camera, params, retained);
```

For caller-owned fixed storage, use `CutResultSink`:

```cpp
std::array<CutEntry, 4096> sharedStorage;
std::array<CutEntry, 4096> currentStorage;
std::array<CutEntry, 4096> idealStorage;

CutResultSink sink{
    Sink<CutEntry>{sharedStorage},
    Sink<CutEntry>{currentStorage},
    Sink<CutEntry>{idealStorage},
};
view.selectCut(publishedWorld, camera, params, sink);

if (sink.shared.overflowed() || sink.currentOnly.overflowed() ||
    sink.idealOnly.overflowed())
{
    growOutputCapacity();
}
```

`Sink::count()` reports entries written and `dropped()` reports entries that
did not fit. Selection performs no growth allocation for fixed sinks.

Each `CutEntry` contains a `NodeHandle`, a 24-bit instance id, and an encoded
screen error. `overThreshold()` preserves the exact threshold decision.
`approximateError(params.threshold)` decodes the logarithmically quantized
error for prioritization; do not use it as an exact pixel measurement.

## Cameras, damping, and view reuse

Use `makePerspectiveCamera()` or `makeLookAtCamera()` for conventional views.
Engines with a combined view-projection matrix can use
`cameraFromViewProjection()` and specify `ClipRange::ZeroToOne` for D3D,
Metal, or Vulkan, or `ClipRange::MinusOneToOne` for OpenGL. The matrix helper
also needs camera position, viewport height, and the projection matrix's
vertical scale (`1 / tan(fovY / 2)`). `cameraFromPlanes()` accepts six inward
planes for custom volumes.

Set `Camera::viewMask` for layer filtering. `Camera::k` is the projection scale
used by `screenError = geometricError * k / distance`.

A `View` owns its damping state. Construct it with a half-life in frames or use
`setHalfLife()`. Zero disables damping exactly. Call `view.reset()` on a camera
cut or teleport so the damping envelope does not span the discontinuity.

Temporal cut reuse is enabled by default. It returns the same selected nodes
as a fresh walk while its conservative proof remains valid. The encoded error
on a reused entry is the recorded value and may be slightly stale within that
proof margin. `reused()`, `walked()`, `bytes()`, and `lastCutStats()` expose
diagnostics. `CutStats` counters are populated only in builds with
`HLOD_STATS`.

## Per-view page usage and collection

Passing a `PageUsageContext` to selection records which pages that view needed
without mutating the world. A context may accumulate observations over many
published epochs. Supply only retention-relevant contexts to collection:

```cpp
PageUsageContext* retainedViews[] = {&mainUsage, &reflectionUsage};
CollectResult result = world.collect(retainedViews, pageBudget, minAge);
```

`maxAttachedPages` counts streamed pages; pinned instance-root mounts neither
count nor become collection candidates. A page must be old enough, unpinned,
and have no attached child mounts before collection can detach it.

`CollectResult::freedPayloads` is a non-owning span over world-owned storage.
It expires on the next `collect()` call or world destruction. Consume or copy
it before collecting again. `PageUsageContext::reset()` discards accumulated
feedback while retaining the context object; `bytes()` reports its storage.

## Per-instance deformation

Submit local-space node bounds for one instance:

```cpp
world.setNodeBounds(instance, nodeHandle, newLocalBounds);
```

The first edit of an instance/page pair creates a bounds-only copy-on-write
overlay. Topology, payloads, errors, residency, and streaming remain shared.
Edits queue until `applyUpdates()`, `flushBounds()`, or `nodeBounds()` applies
them. Multiple submissions to one node resolve to the last box.

Submission order does not affect correctness. For large batches, group edits
by instance and page so consecutive refits reuse nearby overlay data.
`flushBounds()` is mainly for tools that need updated bounds before publishing;
the normal frame barrier already flushes them. `nodeBounds()` returns the
bounds seen by one instance and also flushes pending edits.

Internal ancestor overlays grow conservatively and do not shrink. This can
loosen culling after sustained large deformation but cannot incorrectly cull
content. Proxy geometry remains the application's responsibility.

## Host integration and diagnostics

`WorldConfig` controls TLAS quality and maintenance thresholds, optional
single-view parallelism, and the copied `HlodContext`. The default context uses
the library page allocator and serial task execution.

```cpp
HlodContext context;
context.alloc = engineAlignedAlloc;
context.free = engineFree;
context.parallelFor = engineBlockingParallelFor;
context.workerCount = engineWorkerCount;
context.user = engineServices;

WorldConfig config;
config.context = context;
config.tlasQuality = TlasQuality::BinnedSAH;
config.parallelInstanceThreshold = 4096;
World world(config);

HLodBuilder builder;
// ...author nodes...
Page page = builder.build(context);
```

`HLodBuilder::build()`, `Page::fromBytes()`, and `Page::clone()` use the
allocation callbacks of the context passed to them; `World` uses
`parallelFor`, `workerCount`, and `user` for an enabled uncached parallel
selection. Allocation callbacks must honor the requested alignment. Callback
code and `context.user` referenced by the world must remain valid for its
lifetime. A context used to allocate a page must remain valid for that page's
lifetime.

By default, contract violations throw `std::logic_error`. Exception-free hosts
can define `HLOD_FATAL(msg)` before the first HLodTree header:

```cpp
#define HLOD_FATAL(msg) EnginePanic(msg)
#include "hlod/world.h"
```

The replacement must not return. Contract violations are programmer or content
errors; stale handles from normal asynchronous streaming races do not use this
path.

### World configuration

`WorldConfig` is copied when the world is constructed:

| Field | Default | Meaning |
|---|---:|---|
| `context` | `defaultContext()` callbacks | allocation and blocking task integration |
| `tlasQuality` | `BinnedSAH` | quality tier for initial and promoted rebuilds |
| `tlasTraversalCost` | `1.0` | relative BVH-node cost used by SAH |
| `tlasIntersectCost` | `1.0` | relative instance-test cost used by SAH |
| `tlasCountDrift` | `0.2` | population-change fraction that promotes a quality rebuild |
| `tlasAreaDrift` | `0.5` | grow-only area increase that promotes a quality rebuild |
| `tlasEscapeFraction` | `0.25` | distinct escaped-leaf fraction that requests repair |
| `tlasEditFraction` | `0.05` | incremental spawn/remove fraction allowed before repair |
| `parallelInstanceThreshold` | `0` | visible-instance threshold for an uncached parallel query; zero disables it |

`TlasQuality::BinnedSAH` favors traversal quality, `Median` recursively splits
the longest axis, and `Morton` minimizes build work. Maintenance thresholds are
fractions of the relevant current or build-time population and area; profile
application content before changing them.

## Public surface reference

The guide above shows the normal lifecycle. The remaining public helpers are
summarized here so ownership or introspection code does not need to depend on
internal structures.

### World queries

| Member | Result |
|---|---|
| `config()` | the copied `WorldConfig` |
| `isAsset(handle)` | whether an asset handle is live |
| `assetCount()` | number of registered live assets |
| `assetRootPage(asset)` | shared root mount, or invalid before materialization/stale asset |
| `isAttached(node)` | whether an expansion point currently has a child page |
| `isResident(node)` | whether a live node payload is resident |
| `tryGetPayload(node, out)` | resolves a live node's opaque payload |
| `attachedPageCount()` | all mounted pages, including pinned roots |
| `streamedPageCount()` | mounted pages eligible for the streaming budget |
| `frame()` | update epoch advanced by `applyUpdates()` |
| `overlayCount()` / `overlayBytes()` | live deformation-overlay count and storage |
| `nodeBounds(instance, node)` | local bounds seen by that instance; flushes pending edits |

All count and state queries obey the same phase rule as other world access:
do not race them with mutation.

### Result helpers

`CutView::currentSize()` is `shared.size() + currentOnly.size()`;
`idealSize()` is `shared.size() + idealOnly.size()`. `size()` counts all three
stored sequences and `empty()` tests that total. These functions describe
storage, not the sum of both logical cuts (shared entries are stored once).

`CutEntry::instance()`, `errorCode()`, `overThreshold()`, and
`approximateError(threshold)` unpack the compact entry. The free
`encodeCutError()` and `decodeCutError()` helpers expose the same
threshold-relative encoding for integrations that store compatible metadata.

`Sink<T>::count()`, `dropped()`, and `overflowed()` report fixed-output status.
`CutResultSink` simply groups one sink for each result sequence.

### Page inspection

`PageView` exposes read-only packed arrays for offline tools and custom
streamers. Index zero is the page sentinel; real nodes begin at index one.

| Member | Contents |
|---|---|
| `nodeCount()` / `wideCount()` | packed node and wide-block counts |
| `data()` / `byteSize()` | complete serialized blob |
| `parent`, `subtreeSize`, `meta` | preorder topology arrays |
| `wide`, `blockMask` | eight-lane child blocks and valid/leaf masks |
| `payload`, `bbox`, `geometricError` | immutable authored node data |
| `childCount(i)` / `isExpansion(i)` | decoded node metadata |
| `wideOffset(i)` / `wideBlockCount(i)` | the node's child-block range |
| `validLanes(b)` / `leafLanes(b)` | decoded lane masks for block `b` |
| `wideBounds()` | strided read-only access to page child bounds |

`pageBlobBytes(nodeCount, wideCount)` computes the packed blob size for tools.
Constants such as `kPageMagic`, `kPageVersion`, `kPageAlign`, `kWide`, and the
format structs are public so a content pipeline can identify and budget blobs;
validation should still go through `PageView::fromBytes()` or
`Page::fromBytes()`.

### Math and camera helpers

`float4::point(x, y, z)` sets `w = 1`; `float4::vec(x, y, z)` sets `w = 0`.
`AABB::fromMinMax()` and `fromCenterExtent()` construct bounds, while
`AABB::empty()`, `isEmpty()`, `contains()`, and `expand()` support authoring and
validation.

`withEnvelope(camera, otherPosition)` creates a two-position damping envelope.
`CameraDamper` is public for code that needs a damped camera outside selection;
do not pre-damp a camera and also enable damping on its `View`. Its
`halfLife()`, `setHalfLife()`, `damp()`, and `reset()` methods mirror the view's
damping behavior.

An engine can use its own interface vector types by defining
`HLOD_USE_CUSTOM_VECTOR_TYPES` and declaring compatible `hlod::float4` and
`hlod::float4x4` types before the first HLodTree header. `float4` must be 16
bytes, at least 16-byte aligned, expose public `float x, y, z, w` members, and
provide the construction, arithmetic, and vector helper operations listed in
`math.h`. Keep this definition identical in every translation unit that uses
HLodTree. The internal eight-lane types and serialized page layout are not
replaceable.

`HLOD_VERSION_MAJOR`, `HLOD_VERSION_MINOR`, `HLOD_VERSION_PATCH`, and
`HLOD_VERSION_STRING` expose the library API version at compile time. The
serialized page version is independent and is exposed as `kPageVersion`.

## API lifetime summary

| Value or view | Valid until |
|---|---|
| `CutView` spans | next selection/reset on that `View`, or view destruction |
| `CutResults` spans | mutation/destruction of that owning `CutResults` |
| fixed `CutResultSink` output | caller-defined storage lifetime |
| `CollectResult::freedPayloads` | next `collect()` call or world destruction |
| borrowed `PageView` asset bytes | asset release, with storage kept alive throughout |
| `PageUsageContext` observations | consumed by collection or `reset()` |
| published selection snapshot | next world mutation after all queries join |
| page/node handle | its mount is detached or collected |
| instance reference | that instance is removed |

For implementation details, see [ARCHITECTURE.md](ARCHITECTURE.md). For page,
selection, streaming, and complexity invariants, see
[hlod_design.md](hlod_design.md).
