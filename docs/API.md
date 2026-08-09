# HLodTree API and integration guide

This is the self-contained integration reference for HLodTree 0.5.0. It
explains the vocabulary, public interface, ownership rules, frame lifecycle,
threading contract, and streaming model needed to use the library correctly.
The [README](../README.md) is a shorter project overview; this guide is the
place to start when writing an integration.

## Concepts and terminology

HLodTree selects a view-dependent set of renderable hierarchy elements. The
following terms describe the data flow from authored content to a frame's
render list.

### Node and hierarchy

A **node** is one renderable LOD choice. It contains:

- an application-defined 64-bit payload;
- local-space bounds;
- geometric error in world units; and
- zero or more child nodes that provide a more detailed representation.

Selecting a node means rendering that node's payload instead of its
descendants. Therefore every real node must be independently renderable,
including a node whose deeper children are streamed separately. A
**hierarchy** is a tree of these nodes. It may describe anything from one
object to a terrain region or city block; it does not have to correspond to a
single mesh or entity.

A node has no transform. Its bounds and renderer data use hierarchy-local
coordinates. The world transform belongs to an instance; per-instance
deformation can override node bounds without adding a node transform.

### Page and expansion point

A **logical page** represents the refinements below exactly one hierarchy
node. Page zero contains the hierarchy root. Calling
`HierarchyBuilder::splitBelow(node)` keeps that renderable node in its parent
page and generates one detail page for its descendants. The boundary node is
an **expansion point** and remains the coarse fallback while its detail page is
unavailable.

`hlod::Hierarchy` owns the indexed generated pages. `hlod::Page` is one
move-only packed page blob, while `hlod::PageView` borrows such a blob from
application-owned storage. A detail blob may physically begin with several
children of its logical root; that continuation layout is an internal packing
detail. Its single logical root is the expansion point that names the page.
`Hierarchy` is also move-only; keep it alive while any of its borrowed page
views are registered with a `World`.

Page ids are local to one `Hierarchy`. At runtime, `hlod::DetailPageRef`
combines a generated page id with the registered root asset that identifies
its hierarchy package, so independent hierarchies may both use page id one
without ambiguity.

### Asset

An **asset** is a page registered with a world as a reusable unit of storage
and sharing. Registering an asset returns an `hlod::AssetHandle`. Every
instance of that asset shares the immutable page bytes, attached child-page
graph, and payload-residency state. In this API, *asset* means a registered
hierarchy page; it does not imply one conventional game object.

`Hierarchy` and `AssetHandle` are different levels: a `Hierarchy` is the
content-pipeline package containing every generated logical page, while an
asset registers one currently available page blob with one `World`. The root
page asset is instanced; a detail page may be transferred when loaded or
registered separately when it is reusable.

### Instance

An **instance** places an asset in a world using a translation, positive
uniform scale, and layer mask. Adding an instance returns an
`hlod::World::InstanceRef`. Many instances can reference the same asset
without copying its hierarchy or streaming state.

### World

A **world** (`hlod::World`) owns the runtime state: registered assets,
instances, mounted pages, residency, and the top-level spatial acceleration
structure. The application mutates a world serially, then calls
`applyUpdates()` to publish a stable snapshot for selection. A world is not a
renderer or asset loader; it stores opaque payloads and reports what should be
rendered or streamed.

### Handle and page mount

A **handle** is an opaque, generation-stamped reference to world-owned runtime
state. The application should store and pass handles, not infer internal
indices:

- `hlod::AssetHandle` identifies a registered asset;
- `hlod::PageHandle` identifies one runtime mount of a page;
- `hlod::NodeHandle` identifies a node inside a page mount; and
- `hlod::World::InstanceRef` identifies a live instance.

A **page mount** is one runtime placement of page data. The same registered
asset can be mounted at more than one expansion point, so a page and a page
mount are not interchangeable. Detaching or collecting a mount makes its page
and node handles stale. Generation stamps prevent those handles from silently
referring to newer objects that reuse the same internal slots.

### Payload, topology, and residency

A **payload** (`hlod::UserPayload`) is the application's 64-bit value stored
on every node—typically an index or id used to find renderer resources.
HLodTree never interprets it, and duplicate payload values are valid.

**Topology** describes which hierarchy nodes are currently known. Attaching a
child page expands topology below an expansion point; detaching it collapses
that branch. **Residency** independently describes whether a known node's
render payload is loaded. A page can be attached while some or all of its
payloads remain non-resident.

### Camera, View, and cut

A **camera** (`hlod::Camera`) is one selection input: frustum, position,
projection scale, viewport height, and layer mask. A **View** (`hlod::View`)
is the persistent per-camera query object. It owns LOD damping, temporal reuse,
scratch memory, statistics, and the default result storage. Use a distinct
View for every concurrently selected camera, shadow cascade, or reflection.

A **cut** is a set of nodes that covers the visible hierarchy without
rendering both a node and its descendants. One query produces two logical
cuts:

- the **current cut**, which uses only resident payloads and is safe to render;
- the **ideal cut**, which shows what full residency and attached topology
  would select.

`hlod::CutView` stores their overlap once in `shared`, current-only fallbacks
in `currentOnly`, and ideal-only choices in `idealOnly`. Each
`hlod::CutEntry` carries a node handle, instance id, and encoded screen error.

### How the pieces fit together

```text
authored node hierarchy
        | build or deserialize
        v
logical node hierarchy -- splitBelow() --> Hierarchy pages
                                               |
                                      register root page
                                               v
World <---- InstanceRef ------------ asset instance
  |
  | applyUpdates(), then View::selectCut(Camera)
  v
CutView ---- current cut ----> renderer
   |
   +-------- ideal cut ------> application streamer
```

All public C++ names are in the `hlod` namespace. HLodTree requires C++20.
Runtime code normally includes `hlod/world.h`; content-building tools also
include `hlod/builder.h`.

### Add HLodTree to a CMake target

The repository exposes the `hlod` CMake target; it is not currently an
installed package:

```cmake
add_subdirectory(path/to/HLod-tree)
target_link_libraries(your_target PRIVATE hlod)
```

The target publishes the library's include directory and C++20 requirement.
No third-party library is required at runtime.

## Integration sequence

A normal integration follows this order:

1. Author one complete logical tree with `HierarchyBuilder`, mark natural
   boundaries with `splitBelow()`, and build its `Hierarchy` package. A content
   pipeline may instead validate serialized page blobs with `Page::fromBytes()`
   or `PageView::fromBytes()`.
2. Construct one `World`, register reusable assets, and add their instances.
3. Construct one persistent `View` per camera-like query.
4. Each frame, submit world changes and call `applyUpdates()` once.
5. Select all views from the published world snapshot.
6. Render `shared + currentOnly`; use `shared + idealOnly` to drive external
   payload and topology streaming.
7. After all selections finish, apply completed loads, collect cold pages,
   and begin the next update phase.

### Find guidance by task

| Task | Section |
|---|---|
| Add the library to a CMake target | [Add HLodTree to a CMake target](#add-hlodtree-to-a-cmake-target) |
| Publish a frame safely or select several cameras | [Frame lifecycle and threading](#frame-lifecycle-and-threading) |
| Build, split, serialize, or own hierarchy data | [Authoring a paged hierarchy](#authoring-a-paged-hierarchy) |
| Share content and move instances | [Assets and instances](#assets-and-instances) |
| Attach hierarchy data or report payload availability | [Topology and residency](#topology-and-residency) |
| Render a cut and choose result storage | [Selection and result ownership](#selection-and-result-ownership) |
| Feed page usage into garbage collection | [Per-view page usage and collection](#per-view-page-usage-and-collection) |
| Update bounds for one deformed instance | [Per-instance deformation](#per-instance-deformation) |
| Connect allocators, jobs, and fatal-error policy | [Host integration and diagnostics](#host-integration-and-diagnostics) |
| Check every non-owning lifetime in one place | [API lifetime summary](#api-lifetime-summary) |

## End-to-end example

This example builds a small hierarchy, registers it once, creates two
instances, selects a render cut, and inspects the ideal frontier for streaming.
The declarations at the top are application-owned renderer and loader entry
points; HLodTree deliberately does not implement those systems.

```cpp
#include "hlod/builder.h"
#include "hlod/world.h"

#include <utility>

// Functions supplied by the application.
void submitToRenderer(hlod::UserPayload payload, hlod::InstanceId instance);
bool payloadIsLoaded(hlod::UserPayload payload);
void requestDetailPageLoad(hlod::NodeHandle node,
                           hlod::DetailPageRef page);
void requestPayloadLoad(hlod::NodeHandle node,
                        hlod::UserPayload payload);
void releaseRenderPayload(hlod::UserPayload payload);

hlod::Hierarchy buildTownHierarchy()
{
    hlod::HierarchyBuilder builder;
    const hlod::HierarchyBuilder::NodeId town =
        builder.createRoot(100, 64.0f, hlod::AABB::empty());

    const hlod::HierarchyBuilder::NodeId building1 =
        builder.createNode(town, 101, 16.0f, hlod::AABB::empty());
    builder.createNode(
        building1, 1001, 2.0f,
        hlod::AABB::fromCenterExtent(hlod::float4::point(-2, 2, 0),
                                     hlod::float4::vec(2, 2, 2)));
    builder.createNode(
        building1, 1002, 2.0f,
        hlod::AABB::fromCenterExtent(hlod::float4::point(2, 2, 0),
                                     hlod::float4::vec(2, 2, 2)));

    const hlod::HierarchyBuilder::NodeId building2 =
        builder.createNode(town, 102, 16.0f, hlod::AABB::empty());
    builder.createNode(
        building2, 2001, 2.0f,
        hlod::AABB::fromCenterExtent(hlod::float4::point(18, 2, 0),
                                     hlod::float4::vec(2, 2, 2)));
    builder.createNode(
        building2, 2002, 2.0f,
        hlod::AABB::fromCenterExtent(hlod::float4::point(22, 2, 0),
                                     hlod::float4::vec(2, 2, 2)));

    builder.splitBelow(building1);
    builder.splitBelow(building2);
    return builder.build();
}

void renderEntry(const hlod::World& world, const hlod::CutEntry& entry)
{
    hlod::UserPayload payload;
    if (world.tryGetPayload(entry.nodeHandle, payload))
        submitToRenderer(payload, entry.instance());
}

void renderCurrentCut(const hlod::World& world, const hlod::CutView& cut)
{
    for (const hlod::CutEntry& entry : cut.shared)
        renderEntry(world, entry);
    for (const hlod::CutEntry& entry : cut.currentOnly)
        renderEntry(world, entry);
}

void updateStreamingForEntry(hlod::World& world,
                             const hlod::CutEntry& entry)
{
    hlod::UserPayload payload;
    if (!world.tryGetPayload(entry.nodeHandle, payload))
        return; // The page was detached while this request was pending.

    const hlod::DetailPageRef detailPage =
        world.detailPage(entry.nodeHandle);
    if (entry.overThreshold() &&
        detailPage.valid() &&
        !world.isAttached(entry.nodeHandle))
    {
        requestDetailPageLoad(entry.nodeHandle, detailPage);
    }

    if (!world.isResident(entry.nodeHandle))
    {
        if (payloadIsLoaded(payload))
            world.markResident(entry.nodeHandle);
        else
            requestPayloadLoad(entry.nodeHandle, payload);
    }
}

void updateStreamingForIdealCut(hlod::World& world,
                                const hlod::CutView& cut)
{
    // Call only after every View selection reading this World has finished.
    for (const hlod::CutEntry& entry : cut.shared)
        updateStreamingForEntry(world, entry);
    for (const hlod::CutEntry& entry : cut.idealOnly)
        updateStreamingForEntry(world, entry);
}

// Call when an asynchronous topology load completes. An invalid return is the
// expected result when the expansion point became stale while loading.
void attachLoadedDetailPage(hlod::World& world,
                            hlod::NodeHandle expansionNode,
                            hlod::Page detailPage)
{
    const hlod::PageHandle detailMount =
        world.attachPage(expansionNode, std::move(detailPage));
    if (!detailMount.valid())
        return;
}

// Call when an asynchronous render-payload load completes. A stale handle is
// ignored by markResident().
void publishLoadedPayload(hlod::World& world, hlod::NodeHandle node)
{
    world.markResident(node);
}

int main()
{
    // The borrowed root-page bytes remain valid while World uses the asset.
    hlod::Hierarchy hierarchy = buildTownHierarchy();
    hlod::World world;
    const hlod::AssetHandle treeAsset =
        world.registerAsset(hierarchy.page(hierarchy.rootPage()));

    const hlod::World::InstanceRef first =
        world.addInstance(treeAsset, hlod::float4::point(0, 0, 0));
    const hlod::World::InstanceRef second =
        world.addInstance(treeAsset, hlod::float4::point(20, 0, 0));

    hlod::View mainView(4.0f);        // four-frame LOD damping half-life
    hlod::PageUsageContext mainUsage; // this view influences page retention

    const hlod::Camera camera = hlod::makeLookAtCamera(
        hlod::float4::point(10, 8, -30),
        hlod::float4::point(10, 3, 0));

    world.applyUpdates();             // publish the selection snapshot
    const hlod::World& published = world;
    const hlod::CutView cut = mainView.selectCut(
        published, camera, hlod::CutParams{4.0f, 0.0f}, mainUsage);

    renderCurrentCut(published, cut);

    // Every selection has finished. World mutation is serial again.
    updateStreamingForIdealCut(world, cut);

    const hlod::CollectResult collected = world.collect(mainUsage, 4096, 120);
    for (hlod::UserPayload payload : collected.freedPayloads)
        releaseRenderPayload(payload);

    world.removeInstance(second);
    world.removeInstance(first);
    world.releaseAsset(treeAsset);
}
```

The application must deduplicate `requestDetailPageLoad()` and
`requestPayloadLoad()` calls and apply its own priorities and IO budgets. Page
and payload completions normally arrive in later frames through functions such
as `attachLoadedDetailPage()` and `publishLoadedPayload()`. HLodTree makes stale
completion handles safe, but it does not own the asynchronous loader.

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

### Correctness checklist

- Register shared hierarchy data once and create multiple instances of its
  `AssetHandle`; do not register a private copy for every placement.
- Call `applyUpdates()` once before each group of selections, including frames
  with no world mutations.
- Give concurrent queries distinct `View`, output, and optional
  `PageUsageContext` objects.
- Wait for every selection to finish before mutating the world or collecting
  pages.
- Render `shared + currentOnly`. Use `shared + idealOnly` only to decide what
  the external streamer should load or attach.
- Do not retain a `CutView` across another selection on the same `View`.
- Retain `InstanceRef`, rather than `CutEntry::instance()`, for future
  instance mutation.
- Deduplicate and budget external load requests. Treat stale completion
  handles as an expected outcome of asynchronous streaming.
- Keep borrowed page bytes and page allocator contexts alive for the full
  lifetimes documented below.

## Authoring a paged hierarchy

`hlod::HierarchyBuilder` accepts one ordinary, single-root logical tree. Mark
natural entity boundaries after authoring the nodes:

```cpp
hlod::HierarchyBuilder builder;
const hlod::HierarchyBuilder::NodeId town =
    builder.createRoot(townPayload, townError, hlod::AABB::empty());
const hlod::HierarchyBuilder::NodeId building =
    builder.createNode(town, buildingPayload, buildingError,
                       hlod::AABB::empty());

builder.createNode(building, wall1Payload, wallError, wall1Bounds);
builder.createNode(building, wall2Payload, wallError, wall2Bounds);

builder.splitBelow(building);
hlod::Hierarchy hierarchy = builder.build(context);
```

The generated root page contains `town` and the renderable `building` proxy.
The generated Building detail page contains the walls and is logically rooted
at `building`. The builder automatically:

- marks `building` as an expansion point in its parent page;
- derives its bounds from every descendant, including descendants moved to a
  detail page;
- clamps geometric errors monotonically across page boundaries;
- assigns deterministic `HierarchyPageId` values;
- writes the detail-page id into the expansion metadata.

`splitBelow()` may be called before or after adding the node's children. It is
invalid on a leaf because there would be no detail page to generate. Boundaries
can nest: splitting below a building and then below one of its floors produces
Town, Building, and Floor pages.

`HierarchyBuilder::NodeId` is stable only within that authoring operation and
is consumed by `build()`. Runtime work uses `NodeHandle`. Payload values remain
opaque and need not be unique.

The root page is `hierarchy.rootPage()`. Use `page(id)` to borrow a generated
blob, `clonePage(id)` to make an owned copy, or `takePage(id)` to transfer its
original allocation. No separate mount manifest is needed: every expansion
stores the id of its detail page. During streaming,
`World::detailPage(nodeHandle)` reads that local id and scopes it with the
registered root asset. The application can map that root asset to its
`Hierarchy` or serialized package without recovering a page through
`UserPayload` or tracking every mounted page.

### Low-level physical page construction

`HLodBuilder` remains available for content pipelines that already own a
physical page partitioner. It builds one packed blob and can expose a forest of
continuation roots. `markExpansion(node, detailPageId)` optionally embeds the
same generated-page lookup. Normal integrations should use
`HierarchyBuilder`; the low-level builder exposes storage representation rather
than the logical single-root model.

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
const hlod::AssetHandle owned = world.registerAsset(std::move(page));
const hlod::AssetHandle mapped =
    world.registerAsset(hlod::PageView::fromBytes(data, size));
```

The owned overload transfers the page. The borrowed overload does not copy.
All instances of a root asset share its immutable page data, residency state,
and attached child-page graph.

Create instances with translation, positive uniform scale, and an optional
layer mask:

```cpp
hlod::InstanceDesc desc;
desc.pos = hlod::float4::point(10, 0, 5);
desc.scale = 2.0f;
desc.mask = 1u << 3;
const hlod::World::InstanceRef ref = world.addInstance(asset, desc);
```

An instance is visible only when `desc.mask & camera.viewMask` is nonzero.
Rotation and non-uniform scale are not part of `InstanceDesc`; bake them into
authored data or adapt them outside the library.

`addInstance(Page&&, ...)` is a convenience for one-off, single-page content.
Paged hierarchies should use `registerAsset()` and retain the root
`AssetHandle`: it scopes `DetailPageRef` values and also lets repeated instances
share pages and streaming state. `removeInstance()` and `moveInstance()` ignore
stale references. `releaseAsset()` requires that no live instances still
reference the asset.

`assetRootPage()` returns the shared root mount after at least one instance has
materialized it. `InstanceRef::rootPage` provides the same kind of handle
without another lookup.

### Moving persistent cohorts

For a fixed cohort that moves every frame, retain a `MotionGroup`:

```cpp
std::array<hlod::World::InstanceRef, 2> refs{first, second};
hlod::World::MotionGroup group(refs);

std::array<hlod::float4, 2> positions{
    hlod::float4::point(1, 0, 0),
    hlod::float4::point(21, 0, 0),
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
const hlod::DetailPageRef detailPage =
    world.detailPage(expansionNode);
if (detailPage.valid())
    requestDetailPageLoad(expansionNode, detailPage);

// Later, after the application loads that generated page blob:
const hlod::PageHandle detailMount =
    world.attachPage(expansionNode, std::move(loadedDetailPage));
```

`detailPage()` is a read-only lookup encoded by `HierarchyBuilder`; it returns
an invalid `DetailPageRef` for a stale handle, a non-expansion node, or a
low-level page whose content pipeline did not embed an id. The reference's
`rootAsset` selects the hierarchy package and its `page` field selects the
generated blob inside that package. The local page id remains stable when the
same generated hierarchy is serialized and loaded again.

Applications that pre-register or memory-map every generated blob can attach
its registered `AssetHandle` instead of transferring an owned `Page`.

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
const hlod::CutView cut =
    view.selectCut(publishedWorld, camera, params);
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
hlod::CutResults retained;
view.selectCut(publishedWorld, camera, params, retained);
```

For caller-owned fixed storage, use `CutResultSink`:

```cpp
std::array<hlod::CutEntry, 4096> sharedStorage;
std::array<hlod::CutEntry, 4096> currentStorage;
std::array<hlod::CutEntry, 4096> idealStorage;

hlod::CutResultSink sink{
    hlod::Sink<hlod::CutEntry>{sharedStorage},
    hlod::Sink<hlod::CutEntry>{currentStorage},
    hlod::Sink<hlod::CutEntry>{idealStorage},
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
hlod::PageUsageContext* retainedViews[] = {&mainUsage, &reflectionUsage};
const hlod::CollectResult result =
    world.collect(retainedViews, pageBudget, minAge);
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
hlod::HlodContext context;
context.alloc = engineAlignedAlloc;
context.free = engineFree;
context.parallelFor = engineBlockingParallelFor;
context.workerCount = engineWorkerCount;
context.user = engineServices;

hlod::WorldConfig config;
config.context = context;
config.tlasQuality = hlod::TlasQuality::BinnedSAH;
config.parallelInstanceThreshold = 4096;
hlod::World world(config);

hlod::HierarchyBuilder builder;
// ...author nodes...
hlod::Hierarchy hierarchy = builder.build(context);
```

`HierarchyBuilder::build()`, `HLodBuilder::build()`, `Page::fromBytes()`, and
`Page::clone()` use the
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

### Generated hierarchy package

| Member | Result |
|---|---|
| `rootPage()` | page zero, or invalid for an empty package |
| `pageCount()` | number of generated logical pages |
| `page(id)` | borrowed view of one generated blob |
| `clonePage(id, context)` | explicit owned copy of one blob |
| `takePage(id)` | transfers one blob out of the package |

`HierarchyPageId` values are deterministic for one authored tree: page zero is
the root and generated detail pages start at one. `takePage()` invalidates
subsequent `page()` or `clonePage()` access for that id.

### World queries

| Member | Result |
|---|---|
| `config()` | the copied `WorldConfig` |
| `isAsset(handle)` | whether an asset handle is live |
| `assetCount()` | number of registered live assets |
| `assetRootPage(asset)` | shared root mount, or invalid before materialization/stale asset |
| `isAttached(node)` | whether an expansion point currently has a child page |
| `detailPage(node)` | generated `DetailPageRef`, scoped by root asset, or invalid |
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
| `detailPage(i)` | generated page id for an expansion, or invalid |
| `wideOffset(i)` / `wideBlockCount(i)` | child-block range; `wideOffset` requires local children |
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
| `Hierarchy::page(id)` view | `takePage(id)`, hierarchy destruction, or replacement |
| borrowed `PageView` asset bytes | asset release, with storage kept alive throughout |
| `PageUsageContext` observations | consumed by collection or `reset()` |
| published selection snapshot | next world mutation after all queries join |
| page/node handle | its mount is detached or collected |
| instance reference | that instance is removed |

For implementation details, see [ARCHITECTURE.md](ARCHITECTURE.md). For page,
selection, streaming, and complexity invariants, see
[hlod_design.md](hlod_design.md).
