# Frontier API and integration guide

This is the self-contained integration reference for Frontier 0.6.0. It
explains the vocabulary, public interface, ownership rules, frame lifecycle,
threading contract, and streaming model needed to use the library correctly.
The [README](../README.md) is a shorter project overview; this guide is the
place to start when writing an integration.

## Concepts and terminology

Frontier selects a view-dependent set of renderable hierarchy elements. The
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

A node has no transform. Its bounds and renderer data use its subtree-local
coordinates. The world transform belongs to a top-level instance, and a
nested subtree mount may add a relative translation and positive uniform
scale. Per-instance deformation can override node bounds without rewriting
shared subtree data.

### Subtree and assembly

A **subtree** is a reusable hierarchy fragment whose root is an implicit,
non-renderable anchor. Its real nodes live in one packed page. When the
subtree is instantiated, the anchor is its TLAS leaf. When it is mounted, the
anchor is the parent expansion node, which remains the renderable coarse
fallback.

Every subtree has a stable application/package `SubtreeKey`. An expansion site
stores a permanent target key and a relative translation/uniform scale.
Repeated target keys are deduplicated in the parent subtree's dependency
table; each site stores only a compact dependency index. Authored content is
therefore a DAG of reusable subtree definitions, while each database placement
is a tree of mounts.

`SubtreeBuilder` is the assembly API. `HierarchyBuilder::splitBelow()` remains
the monolithic-authoring and physical-page-partitioning API. The current
`Subtree` representation owns one packed page plus its dependency sidecar;
compose subtrees for deeper reusable structures.

### Page and expansion point

A **logical page** represents the refinements below exactly one hierarchy
node. Page zero contains the hierarchy root. Calling
`HierarchyBuilder::splitBelow(node)` keeps that renderable node in its parent
page and generates one detail page for its descendants. The boundary node is
an **expansion point** and remains the coarse fallback while its detail page is
unavailable.

`frontier::Hierarchy` owns the indexed generated pages. `frontier::Page` is one
move-only packed page blob, while `frontier::PageView` borrows such a blob from
application-owned storage. A detail blob may physically begin with several
children of its logical root; that continuation layout is an internal packing
detail. Its single logical root is the expansion point that names the page.
`Hierarchy` is also move-only; keep it alive while any of its borrowed page
views are registered with a `SpatialDatabase`.

Page ids are local to one `Hierarchy`. At runtime, `frontier::DetailPageRef`
combines a generated page id with the registered root asset that identifies
its hierarchy package, so independent hierarchies may both use page id one
without ambiguity.

### Asset

An **asset** is a page registered with a spatial database as a reusable unit of storage
and sharing. Registering an asset returns a `frontier::AssetHandle`. Every
instance of that asset shares the immutable page bytes, attached child-page
graph, and payload-residency state. In this API, *asset* means a registered
hierarchy page; it does not imply one conventional game object.

`Hierarchy` and `AssetHandle` are different levels: a `Hierarchy` is the
content-pipeline package containing every generated logical page, while an
asset registers one currently available page blob with one `SpatialDatabase`. The root
page asset is instanced; a detail page may be transferred when loaded or
registered separately when it is reusable.

### Instance

An **instance** places an asset in a spatial database using a translation, positive
uniform scale, and layer mask. Adding an instance returns an
`frontier::SpatialDatabase::InstanceRef`. Many instances can reference the same asset
without copying its hierarchy or streaming state.

### Spatial database

A **spatial database** (`frontier::SpatialDatabase`) owns the runtime state:
registered assets, instances, mounted pages, residency, and the top-level
spatial acceleration structure. The application mutates the database
serially, then calls `applyUpdates()` to publish a stable snapshot for
selection. The database is not a renderer or asset loader; it stores opaque
payloads and reports what should be rendered or streamed.

### Handle and page mount

A **handle** is an opaque, generation-stamped reference to database-owned runtime
state. The application should store and pass handles, not infer internal
indices:

- `frontier::AssetHandle` identifies a registered asset;
- `frontier::SubtreeHandle` identifies a registered logical subtree;
- `frontier::PageHandle` identifies one runtime mount of a page;
- `frontier::MountHandle` is the assembly name for a page mount;
- `frontier::NodeHandle` identifies a node inside a page mount; and
- `frontier::SpatialDatabase::InstanceRef` identifies a live instance.

A **page mount** is one runtime placement of page data. The same registered
asset can be mounted at more than one expansion point, so a page and a page
mount are not interchangeable. Detaching or collecting a mount makes its page
and node handles stale. Generation stamps prevent those handles from silently
referring to newer objects that reuse the same internal slots.

### Payload, topology, and residency

A **payload** (`frontier::UserPayload`) is the application's 64-bit value stored
on every node—typically an index or id used to find renderer resources.
Frontier never interprets it, and duplicate payload values are valid.

**Topology** describes which hierarchy nodes are currently known. Attaching a
child page expands topology below an expansion point; detaching it collapses
that branch. **Residency** independently describes whether a known node's
render payload is loaded. A page can be attached while some or all of its
payloads remain non-resident.

### Camera, spatial query, and frontier result

A **camera** (`frontier::Camera`) is one selection input: frustum, position,
projection scale, viewport height, and layer mask. A **spatial query**
(`frontier::SpatialQuery`) is a reusable, stateful query executor. It owns LOD
damping, temporal reuse, scratch memory, statistics, and default result
storage. Use a distinct `SpatialQuery` for every concurrently selected camera,
shadow cascade, or reflection probe.

A **frontier** is the hierarchical cut through the visible scene: an antichain
of selected nodes that never contains both a node and its descendant. One query
produces two logical frontiers:

- the **current frontier**, which uses only resident payloads and is safe to render;
- the **ideal frontier**, which shows what full residency and attached topology
  would select.

The query normally returns a borrowed **frontier result view**
(`frontier::FrontierResultView`). It stores the overlap once in `shared`, current-only
fallbacks in `currentOnly`, and ideal-only choices in `idealOnly`. Its spans
remain valid until that query runs again or is reset. An owning **frontier result**
(`frontier::FrontierResult`) materializes the same three sequences when they must live
longer. Each `frontier::FrontierEntry` carries a node handle, instance id, and encoded
screen error.

### How the pieces fit together

```text
authored node hierarchy
        | build or deserialize
        v
logical node hierarchy -- splitBelow() --> Hierarchy pages
                                               |
                                      register root page
                                               v
SpatialDatabase <---- InstanceRef ------------ asset instance
  |
  | applyUpdates(), then SpatialQuery::selectFrontier(Camera)
  v
FrontierResultView ---- current frontier ----> renderer
   |
   +-------- ideal frontier ------> application streamer
```

All public C++ names are in the `frontier` namespace. Frontier requires C++20.
Runtime code normally includes `frontier/spatial_database.h`; content-building tools also
include `frontier/builder.h`.

### Add Frontier to a CMake target

The repository exposes the `frontier` CMake target; it is not currently an
installed package:

```cmake
add_subdirectory(path/to/HLod-tree)
target_link_libraries(your_target PRIVATE frontier)
```

The target publishes the library's include directory and C++20 requirement.
No third-party library is required at runtime.

## Integration sequence

A normal integration follows this order:

1. Build reusable components with `SubtreeBuilder` and connect expansion sites
   with permanent `SubtreeKey` values. Monolithic content may instead use
   `HierarchyBuilder::splitBelow()`.
2. Construct one `SpatialDatabase`, register subtrees, and instantiate top-level
   components. Register or transfer legacy pages as needed.
3. Construct one persistent `SpatialQuery` per camera-like query.
4. Each frame, submit database changes and call `applyUpdates()` once.
5. Execute all spatial queries against the published database snapshot.
6. Render `shared + currentOnly`; use `shared + idealOnly` to drive external
   payload and topology streaming.
7. After all selections finish, apply completed loads, collect cold pages,
   and begin the next update phase.

### Find guidance by task

| Task | Section |
|---|---|
| Add the library to a CMake target | [Add Frontier to a CMake target](#add-frontier-to-a-cmake-target) |
| Publish a frame safely or select several cameras | [Frame lifecycle and threading](#frame-lifecycle-and-threading) |
| Build, split, serialize, or own hierarchy data | [Authoring a paged hierarchy](#authoring-a-paged-hierarchy) |
| Assemble reusable hierarchy components | [Assembling reusable subtrees](#assembling-reusable-subtrees) |
| Share content and move instances | [Assets and instances](#assets-and-instances) |
| Attach hierarchy data or report payload availability | [Topology and residency](#topology-and-residency) |
| Render a frontier and choose result storage | [Selection and result ownership](#selection-and-result-ownership) |
| Feed page usage into garbage collection | [Per-query page usage and collection](#per-query-page-usage-and-collection) |
| Update bounds for one deformed instance | [Per-instance deformation](#per-instance-deformation) |
| Connect allocators, jobs, and fatal-error policy | [Host integration and diagnostics](#host-integration-and-diagnostics) |
| Check every non-owning lifetime in one place | [API lifetime summary](#api-lifetime-summary) |

## End-to-end example

This example builds a small hierarchy, registers it once, creates two
instances, selects a render frontier, and inspects the ideal frontier for streaming.
The declarations at the top are application-owned renderer and loader entry
points; Frontier deliberately does not implement those systems.

```cpp
#include "frontier/builder.h"
#include "frontier/spatial_database.h"

#include <utility>

// Functions supplied by the application.
void submitToRenderer(frontier::UserPayload payload, frontier::InstanceId instance);
bool payloadIsLoaded(frontier::UserPayload payload);
void requestDetailPageLoad(frontier::NodeHandle node,
                           frontier::DetailPageRef page);
void requestPayloadLoad(frontier::NodeHandle node,
                        frontier::UserPayload payload);
void releaseRenderPayload(frontier::UserPayload payload);

frontier::Hierarchy buildTownHierarchy()
{
    frontier::HierarchyBuilder builder;
    const frontier::HierarchyBuilder::NodeId town =
        builder.createRoot(100, 64.0f, frontier::AABB::empty());

    const frontier::HierarchyBuilder::NodeId building1 =
        builder.createNode(town, 101, 16.0f, frontier::AABB::empty());
    builder.createNode(
        building1, 1001, 2.0f,
        frontier::AABB::fromCenterExtent(frontier::float4::point(-2, 2, 0),
                                     frontier::float4::vec(2, 2, 2)));
    builder.createNode(
        building1, 1002, 2.0f,
        frontier::AABB::fromCenterExtent(frontier::float4::point(2, 2, 0),
                                     frontier::float4::vec(2, 2, 2)));

    const frontier::HierarchyBuilder::NodeId building2 =
        builder.createNode(town, 102, 16.0f, frontier::AABB::empty());
    builder.createNode(
        building2, 2001, 2.0f,
        frontier::AABB::fromCenterExtent(frontier::float4::point(18, 2, 0),
                                     frontier::float4::vec(2, 2, 2)));
    builder.createNode(
        building2, 2002, 2.0f,
        frontier::AABB::fromCenterExtent(frontier::float4::point(22, 2, 0),
                                     frontier::float4::vec(2, 2, 2)));

    builder.splitBelow(building1);
    builder.splitBelow(building2);
    return builder.build();
}

void renderEntry(const frontier::SpatialDatabase& database, const frontier::FrontierEntry& entry)
{
    frontier::UserPayload payload;
    if (database.tryGetPayload(entry.nodeHandle, payload))
        submitToRenderer(payload, entry.instance());
}

void renderCurrentFrontier(const frontier::SpatialDatabase& database,
                           const frontier::FrontierResultView& result)
{
    for (const frontier::FrontierEntry& entry : result.shared)
        renderEntry(database, entry);
    for (const frontier::FrontierEntry& entry : result.currentOnly)
        renderEntry(database, entry);
}

void updateStreamingForEntry(frontier::SpatialDatabase& database,
                             const frontier::FrontierEntry& entry)
{
    frontier::UserPayload payload;
    if (!database.tryGetPayload(entry.nodeHandle, payload))
        return; // The page was detached while this request was pending.

    const frontier::DetailPageRef detailPage =
        database.detailPage(entry.nodeHandle);
    if (entry.overThreshold() &&
        detailPage.valid() &&
        !database.isAttached(entry.nodeHandle))
    {
        requestDetailPageLoad(entry.nodeHandle, detailPage);
    }

    if (!database.isResident(entry.nodeHandle))
    {
        if (payloadIsLoaded(payload))
            database.markResident(entry.nodeHandle);
        else
            requestPayloadLoad(entry.nodeHandle, payload);
    }
}

void updateStreamingForIdealFrontier(frontier::SpatialDatabase& database,
                                     const frontier::FrontierResultView& result)
{
    // Call only after every SpatialQuery selection reading this SpatialDatabase has finished.
    for (const frontier::FrontierEntry& entry : result.shared)
        updateStreamingForEntry(database, entry);
    for (const frontier::FrontierEntry& entry : result.idealOnly)
        updateStreamingForEntry(database, entry);
}

// Call when an asynchronous topology load completes. An invalid return is the
// expected result when the expansion point became stale while loading.
void attachLoadedDetailPage(frontier::SpatialDatabase& database,
                            frontier::NodeHandle expansionNode,
                            frontier::Page detailPage)
{
    const frontier::PageHandle detailMount =
        database.attachPage(expansionNode, std::move(detailPage));
    if (!detailMount.valid())
        return;
}

// Call when an asynchronous render-payload load completes. A stale handle is
// ignored by markResident().
void publishLoadedPayload(frontier::SpatialDatabase& database, frontier::NodeHandle node)
{
    database.markResident(node);
}

int main()
{
    // The borrowed root-page bytes remain valid while SpatialDatabase uses the asset.
    frontier::Hierarchy hierarchy = buildTownHierarchy();
    frontier::SpatialDatabase database;
    const frontier::AssetHandle treeAsset =
        database.registerAsset(hierarchy.page(hierarchy.rootPage()));

    const frontier::SpatialDatabase::InstanceRef first =
        database.addInstance(treeAsset, frontier::float4::point(0, 0, 0));
    const frontier::SpatialDatabase::InstanceRef second =
        database.addInstance(treeAsset, frontier::float4::point(20, 0, 0));

    frontier::SpatialQuery mainQuery(4.0f); // four-frame LOD damping half-life
    frontier::PageUsageContext mainUsage;   // this query influences page retention

    const frontier::Camera camera = frontier::makeLookAtCamera(
        frontier::float4::point(10, 8, -30),
        frontier::float4::point(10, 3, 0));

    database.applyUpdates();             // publish the selection snapshot
    const frontier::SpatialDatabase& publishedDatabase = database;
    const frontier::FrontierResultView result = mainQuery.selectFrontier(
        publishedDatabase, camera, frontier::SelectionParams{4.0f, 0.0f}, mainUsage);

    renderCurrentFrontier(publishedDatabase, result);

    // Every selection has finished. SpatialDatabase mutation is serial again.
    updateStreamingForIdealFrontier(database, result);

    const frontier::CollectResult collected = database.collect(mainUsage, 4096, 120);
    for (frontier::UserPayload payload : collected.freedPayloads)
        releaseRenderPayload(payload);

    database.removeInstance(second);
    database.removeInstance(first);
    database.releaseAsset(treeAsset);
}
```

The application must deduplicate `requestDetailPageLoad()` and
`requestPayloadLoad()` calls and apply its own priorities and IO budgets. Page
and payload completions normally arrive in later frames through functions such
as `attachLoadedDetailPage()` and `publishLoadedPayload()`. Frontier makes stale
completion handles safe, but it does not own the asynchronous loader.

## Frame lifecycle and threading

Use this order for every published snapshot:

1. Apply instance, bounds, topology, and residency mutations to `SpatialDatabase`.
2. Call `database.applyUpdates()` once, even if no objects changed.
3. Treat the database as `const` and run selections. Distinct `SpatialQuery` objects may
   select concurrently; each call must also have distinct output storage and,
   when supplied, a distinct `PageUsageContext`.
4. Join every selection before mutating the database or calling `collect()`.

`SpatialDatabase` is single-writer and is not internally synchronized. A
`SpatialQuery` binds to the first database it queries and is itself mutable,
so the same query must not be used concurrently. `SpatialQuery::reset()`
releases that binding.

For parallelism within one uncached query, provide a blocking
`FrontierContext::parallelFor`, set `workerCount` above one, set
`SpatialDatabaseConfig::parallelInstanceThreshold`, and call
`query.setReuseEnabled(false)`. The callback may execute tasks in any order but
must not return until all tasks have completed.

### Correctness checklist

- Register shared hierarchy data once and create multiple instances of its
  `AssetHandle`; do not register a private copy for every placement.
- Call `applyUpdates()` once before each group of selections, including frames
  with no database mutations.
- Give concurrent queries distinct `SpatialQuery`, output, and optional
  `PageUsageContext` objects.
- Wait for every selection to finish before mutating the database or collecting
  pages.
- Render `shared + currentOnly`. Use `shared + idealOnly` only to decide what
  the external streamer should load or attach.
- Do not retain a `FrontierResultView` across another selection on the same `SpatialQuery`.
- Retain `InstanceRef`, rather than `FrontierEntry::instance()`, for future
  instance mutation.
- Deduplicate and budget external load requests. Treat stale completion
  handles as an expected outcome of asynchronous streaming.
- Keep borrowed page bytes and page allocator contexts alive for the full
  lifetimes documented below.

## Assembling reusable subtrees

Use `SubtreeBuilder` when several hierarchy sites refine into the same authored
component. The builder exposes an implicit root through `root()`; pass it as
the parent of every real top-level node in the component.

This example authors detailed house contents once and references them from two
coarse house proxies:

```cpp
constexpr frontier::SubtreeKey houseKey{1};
constexpr frontier::SubtreeKey cityKey{2};

frontier::SubtreeBuilder house(houseKey);
house.createNode(
    house.root(), wallPayload, 0.0f,
    frontier::AABB::fromCenterExtent(frontier::float4::point(0, 0, 0),
                                     frontier::float4::vec(1, 2, 1)));
frontier::Subtree houseDetails = house.build();

frontier::SubtreeBuilder city(cityKey);
const auto cityProxy =
    city.createNode(city.root(), cityPayload, 64.0f, frontier::AABB::empty());

const auto leftHouse = city.createNode(
    cityProxy, houseProxyPayload, 16.0f,
    frontier::AABB::fromCenterExtent(frontier::float4::point(-10, 0, 0),
                                     frontier::float4::vec(2, 3, 2)));
city.setExpansion(
    leftHouse, houseKey,
    frontier::SubtreeTransform{frontier::float4::point(-10, 0, 0), 1.0f});

const auto rightHouse = city.createNode(
    cityProxy, houseProxyPayload, 16.0f,
    frontier::AABB::fromCenterExtent(frontier::float4::point(10, 0, 0),
                                     frontier::float4::vec(2, 3, 2)));
city.setExpansion(
    rightHouse, houseKey,
    frontier::SubtreeTransform{frontier::float4::point(10, 0, 0), 1.0f});

frontier::SpatialDatabase database;
const frontier::SubtreeHandle houseAsset =
    database.registerSubtree(std::move(houseDetails));
const frontier::SubtreeHandle cityAsset =
    database.registerSubtree(city.build());
const frontier::SpatialDatabase::InstanceRef cityInstance =
    database.instantiate(cityAsset, frontier::float4::point(0, 0, 0));
```

Before mounting, an over-threshold ideal-frontier entry still selects the
coarse house proxy. Query its permanent content target and attach the already
registered shared definition:

```cpp
if (entry.overThreshold() &&
    database.expansionTarget(entry.nodeHandle) == houseKey &&
    !database.isAttached(entry.nodeHandle))
{
    const frontier::MountHandle mount =
        database.mount(entry.nodeHandle, houseAsset);
    if (mount.valid())
    {
        // Payload residency remains explicit and mount-specific.
        // Mark loaded node handles resident as their renderer data completes.
    }
}
```

Each call to `mount()` creates distinct placement state but retains the same
immutable house page bytes. Mount transforms are authored in the parent
subtree and therefore shared by every top-level instance of that parent. Use
`tryGetNodeTransform()` to recover a selected node's accumulated
mount-to-instance transform; compose it with the transform indexed by
`FrontierEntry::instance()` when submitting renderer work.

The transformed child sentinel bounds must fit inside the expansion proxy's
authored bounds. Effective error is converted across mount scale. Bounds edits
remain child-local and propagate through the relative transform into the
instance's bounds-only COW overlays.

`SubtreeBuilder::build()` currently produces one packed page and an owned
dependency sidecar. Register the resulting object directly; only raw `Page`
blobs currently have the standalone `data()`/`fromBytes()` serialization API.

## Authoring a paged hierarchy

`frontier::HierarchyBuilder` accepts one ordinary, single-root logical tree. Mark
natural entity boundaries after authoring the nodes:

```cpp
frontier::HierarchyBuilder builder;
const frontier::HierarchyBuilder::NodeId town =
    builder.createRoot(townPayload, townError, frontier::AABB::empty());
const frontier::HierarchyBuilder::NodeId building =
    builder.createNode(town, buildingPayload, buildingError,
                       frontier::AABB::empty());

builder.createNode(building, wall1Payload, wallError, wall1Bounds);
builder.createNode(building, wall2Payload, wallError, wall2Bounds);

builder.splitBelow(building);
frontier::Hierarchy hierarchy = builder.build(context);
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
`SpatialDatabase::detailPage(nodeHandle)` reads that local id and scopes it with the
registered root asset. The application can map that root asset to its
`Hierarchy` or serialized package without recovering a page through
`UserPayload` or tracking every mounted page.

### Low-level physical page construction

`PageBuilder` remains available for content pipelines that already own a
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
registered asset is released. A `FrontierContext` that allocated an owned `Page`
must outlive the page, including after the page moves into a `SpatialDatabase`.

## Assets and instances

Register reusable page data once:

```cpp
const frontier::AssetHandle owned = database.registerAsset(std::move(page));
const frontier::AssetHandle mapped =
    database.registerAsset(frontier::PageView::fromBytes(data, size));
```

The owned overload transfers the page. The borrowed overload does not copy.
All instances of a root asset share its immutable page data, residency state,
and attached child-page graph.

Create instances with translation, positive uniform scale, and an optional
layer mask:

```cpp
frontier::InstanceDesc desc;
desc.pos = frontier::float4::point(10, 0, 5);
desc.scale = 2.0f;
desc.mask = 1u << 3;
const frontier::SpatialDatabase::InstanceRef ref = database.addInstance(asset, desc);
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
std::array<frontier::SpatialDatabase::InstanceRef, 2> refs{first, second};
frontier::SpatialDatabase::MotionGroup group(refs);

std::array<frontier::float4, 2> positions{
    frontier::float4::point(1, 0, 0),
    frontier::float4::point(21, 0, 0),
};
database.moveInstances(group, positions);
```

Positions use the original group order. Stale references are ignored and the
last position wins for duplicates. The group caches the database's physical
order and refreshes that cache after a layout change. Use `moveInstance()` for
isolated moves or cohorts whose membership changes continually.

The first TLAS build establishes spatial physical order. Routine updates
preserve it. Call `database.optimize()` at an occasional synchronization point
after disruptive motion or heavy spawn/despawn activity. It flushes pending
bounds, compacts dead slots, rebuilds the quality TLAS, and restores spatial
instance and query-record locality. Public handles and instance ids remain
stable. It is intended for loading screens, menus, teleports, and level
transitions rather than per-frame maintenance.

## Handles and lifetime

The API uses generation-stamped handles:

| Type | Names | Invalidated by |
|---|---|---|
| `AssetHandle` | a registered page asset | `releaseAsset()` |
| `PageHandle` | one mounted page | detaching or collecting that mount |
| `NodeHandle` | one packed node in a mount | invalidation of its page mount |
| `SpatialDatabase::InstanceRef` | one live instance slot | `removeInstance()` |

Use `nodeAt(pageHandle, packedIndex)` only when the packed page-local index is
known. Handles returned in `FrontierEntry` are normally the simplest route for
streaming and residency operations.

A stale handle is an expected asynchronous race. Mutating calls that document
stale tolerance ignore it; queries such as `isResident()` and
`tryGetPayload()` report absence. `attachPage()` returns an invalid
`PageHandle` when its expansion node went stale while the child was loading.

`FrontierEntry::instance()` is a compact id for the published snapshot and is
suited to indexing caller-side transform or entity tables during that
selection phase. Retain `InstanceRef`, not the bare id, for later mutation.

## Topology and residency

Attach deeper topology under a live expansion node:

```cpp
const frontier::DetailPageRef detailPage =
    database.detailPage(expansionNode);
if (detailPage.valid())
    requestDetailPageLoad(expansionNode, detailPage);

// Later, after the application loads that generated page blob:
const frontier::PageHandle detailMount =
    database.attachPage(expansionNode, std::move(loadedDetailPage));
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
so the current frontier refines only when resident descendants cover the required
visible region.

`tryGetPayload(handle, out)` resolves the immutable `uint64_t` application
payload of a live node. The database does not interpret or index payloads.

## Selection and result ownership

`SelectionParams::threshold` is the permitted projected geometric error in pixels.
Selection refines above it. `minPix` optionally culls a whole instance when
its maximum projected contribution is smaller than that value; zero disables
this culling.

The simplest query returns a borrowed `FrontierResultView`:

```cpp
const frontier::FrontierResultView result =
    query.selectFrontier(publishedDatabase, camera, params);
```

Its three spans are disjoint:

- `shared` belongs to both frontiers;
- `currentOnly` contains resident fallbacks needed only for rendering now;
- `idealOnly` belongs only to the fully-resident ideal frontier.

Render `shared + currentOnly`. Inspect `shared + idealOnly` for payload and
topology work. Output order is traversal-defined, not priority order.

The spans refer to storage owned by the `SpatialQuery` and expire on its next
selection, `reset()`, or destruction. Use the explicit owning result when a
result must survive another execution of the same query:

```cpp
frontier::FrontierResult retained;
query.selectFrontier(publishedDatabase, camera, params, retained);
```

For caller-owned fixed storage, use `FrontierResultSink`:

```cpp
std::array<frontier::FrontierEntry, 4096> sharedStorage;
std::array<frontier::FrontierEntry, 4096> currentStorage;
std::array<frontier::FrontierEntry, 4096> idealStorage;

frontier::FrontierResultSink sink{
    frontier::Sink<frontier::FrontierEntry>{sharedStorage},
    frontier::Sink<frontier::FrontierEntry>{currentStorage},
    frontier::Sink<frontier::FrontierEntry>{idealStorage},
};
query.selectFrontier(publishedDatabase, camera, params, sink);

if (sink.shared.overflowed() || sink.currentOnly.overflowed() ||
    sink.idealOnly.overflowed())
{
    growOutputCapacity();
}
```

`Sink::count()` reports entries written and `dropped()` reports entries that
did not fit. Selection performs no growth allocation for fixed sinks.

Each `FrontierEntry` contains a `NodeHandle`, a 24-bit instance id, and an encoded
screen error. `overThreshold()` preserves the exact threshold decision.
`approximateError(params.threshold)` decodes the logarithmically quantized
error for prioritization; do not use it as an exact pixel measurement.

## Cameras, damping, and query reuse

Use `makePerspectiveCamera()` or `makeLookAtCamera()` for conventional views.
Engines with a combined view-projection matrix can use
`cameraFromViewProjection()` and specify `ClipRange::ZeroToOne` for D3D,
Metal, or Vulkan, or `ClipRange::MinusOneToOne` for OpenGL. The matrix helper
also needs camera position, viewport height, and the projection matrix's
vertical scale (`1 / tan(fovY / 2)`). `cameraFromPlanes()` accepts six inward
planes for custom volumes.

Set `Camera::viewMask` for layer filtering. `Camera::k` is the projection scale
used by `screenError = geometricError * k / distance`.

A `SpatialQuery` owns its damping state. Construct it with a half-life in
frames or use `setHalfLife()`. Zero disables damping exactly. Call
`query.reset()` on a camera cut or teleport so the damping envelope does not
span the discontinuity.

Temporal frontier reuse is enabled by default. It returns the same selected nodes
as a fresh walk while its conservative proof remains valid. The encoded error
on a reused entry is the recorded value and may be slightly stale within that
proof margin. `reused()`, `walked()`, `bytes()`, and `lastSelectionStats()` expose
diagnostics. `SelectionStats` counters are populated only in builds with
`FRONTIER_STATS`.

A cached traversal versions an entire mounted tree at its root. Descendant
residency and attachment changes propagate to that stamp, so reuse validation
is one dependency for both a flat page and a city assembled from many mounted
subtrees. Output buckets larger than 1,023 entries remain cacheable through a
sparse full-width count spill; the 32-byte hot per-instance record is unchanged.
When a `PageUsageContext` is supplied, selection re-walks an aggregate cache hit
as needed to enumerate the exact physical pages for retention feedback.

## Per-query page usage and collection

Passing a `PageUsageContext` to selection records which pages that query needed
without mutating the database. A context may accumulate observations over many
published epochs. Supply only retention-relevant contexts to collection:

```cpp
frontier::PageUsageContext* retentionQueries[] = {&mainUsage, &reflectionUsage};
const frontier::CollectResult result =
    database.collect(retentionQueries, pageBudget, minAge);
```

`maxAttachedPages` counts streamed pages; pinned instance-root mounts neither
count nor become collection candidates. A page must be old enough, unpinned,
and have no attached child mounts before collection can detach it.

`CollectResult::freedPayloads` is a non-owning span over database-owned storage.
It expires on the next `collect()` call or database destruction. Consume or copy
it before collecting again. `PageUsageContext::reset()` discards accumulated
feedback while retaining the context object; `bytes()` reports its storage.

## Per-instance deformation

Submit local-space node bounds for one instance:

```cpp
database.setNodeBounds(instance, nodeHandle, newLocalBounds);
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

`SpatialDatabaseConfig` controls TLAS quality and maintenance thresholds,
optional single-query parallelism, and the copied `FrontierContext`. The default context uses
the library page allocator and serial task execution.

```cpp
frontier::FrontierContext context;
context.alloc = engineAlignedAlloc;
context.free = engineFree;
context.parallelFor = engineBlockingParallelFor;
context.workerCount = engineWorkerCount;
context.user = engineServices;

frontier::SpatialDatabaseConfig config;
config.context = context;
config.tlasQuality = frontier::TlasQuality::BinnedSAH;
config.parallelInstanceThreshold = 4096;
frontier::SpatialDatabase database(config);

frontier::HierarchyBuilder builder;
// ...author nodes...
frontier::Hierarchy hierarchy = builder.build(context);
```

`HierarchyBuilder::build()`, `PageBuilder::build()`, `Page::fromBytes()`, and
`Page::clone()` use the
allocation callbacks of the context passed to them; `SpatialDatabase` uses
`parallelFor`, `workerCount`, and `user` for an enabled uncached parallel
selection. Allocation callbacks must honor the requested alignment. Callback
code and `context.user` referenced by the database must remain valid for its
lifetime. A context used to allocate a page must remain valid for that page's
lifetime.

By default, contract violations throw `std::logic_error`. Exception-free hosts
can define `FRONTIER_FATAL(msg)` before the first Frontier header:

```cpp
#define FRONTIER_FATAL(msg) EnginePanic(msg)
#include "frontier/spatial_database.h"
```

The replacement must not return. Contract violations are programmer or content
errors; stale handles from normal asynchronous streaming races do not use this
path.

### Spatial database configuration

`SpatialDatabaseConfig` is copied when the database is constructed:

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

### Reusable subtree package

| Member | Result |
|---|---|
| `key()` | stable authored `SubtreeKey` |
| `page()` | borrowed view of the packed real nodes below the implicit root |
| `dependencies()` | deduplicated permanent child keys |
| `expansions()` | packed-node/dependency/transform records sorted by node |

`Subtree` is move-only. `registerSubtree(std::move(subtree))` transfers its
page and logical metadata into the database.

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

### Spatial database inspection

| Member | Result |
|---|---|
| `config()` | the copied `SpatialDatabaseConfig` |
| `isAsset(handle)` | whether an asset handle is live |
| `isSubtree(handle)` | whether a logical subtree handle is live |
| `assetCount()` | number of registered live assets |
| `assetRootPage(asset)` | shared root mount, or invalid before materialization/stale asset |
| `isAttached(node)` | whether an expansion point currently has a child page |
| `detailPage(node)` | generated `DetailPageRef`, scoped by root asset, or invalid |
| `expansionTarget(node)` | permanent `SubtreeKey`, or invalid for legacy/stale nodes |
| `tryGetNodeTransform(node, out)` | accumulated mount-to-instance translation/scale |
| `isResident(node)` | whether a live node payload is resident |
| `tryGetPayload(node, out)` | resolves a live node's opaque payload |
| `attachedPageCount()` | all mounted pages, including pinned roots |
| `streamedPageCount()` | mounted pages eligible for the streaming budget |
| `frame()` | update epoch advanced by `applyUpdates()` |
| `overlayCount()` / `overlayBytes()` | live deformation-overlay count and storage |
| `mountStateBytes()` | retained mount/transform/residency/link capacity, excluding asset blobs |
| `nodeBounds(instance, node)` | local bounds seen by that instance; flushes pending edits |

All count and state inspection obeys the same phase rule as other database access:
do not race them with mutation.

### Result helpers

`FrontierResultView::currentSize()` is `shared.size() + currentOnly.size()`;
`idealSize()` is `shared.size() + idealOnly.size()`. `size()` counts all three
stored sequences and `empty()` tests that total. These functions describe
storage, not the sum of both logical frontiers (shared entries are stored once).

`FrontierEntry::instance()`, `errorCode()`, `overThreshold()`, and
`approximateError(threshold)` unpack the compact entry. The free
`encodeFrontierError()` and `decodeFrontierError()` helpers expose the same
threshold-relative encoding for integrations that store compatible metadata.

`Sink<T>::count()`, `dropped()`, and `overflowed()` report fixed-output status.
`FrontierResultSink` simply groups one sink for each result sequence.

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
do not pre-damp a camera and also enable damping on its `SpatialQuery`. Its
`halfLife()`, `setHalfLife()`, `damp()`, and `reset()` methods mirror the query's
damping behavior.

An engine can use its own interface vector types by defining
`FRONTIER_USE_CUSTOM_VECTOR_TYPES` and declaring compatible `frontier::float4` and
`frontier::float4x4` types before the first Frontier header. `float4` must be 16
bytes, at least 16-byte aligned, expose public `float x, y, z, w` members, and
provide the construction, arithmetic, and vector helper operations listed in
`math.h`. Keep this definition identical in every translation unit that uses
Frontier. The internal eight-lane types and serialized page layout are not
replaceable.

`FRONTIER_VERSION_MAJOR`, `FRONTIER_VERSION_MINOR`, `FRONTIER_VERSION_PATCH`, and
`FRONTIER_VERSION_STRING` expose the library API version at compile time. The
serialized page version is independent and is exposed as `kPageVersion`.

## API lifetime summary

| Value or borrowed view | Valid until |
|---|---|
| `FrontierResultView` spans | next selection/reset on that `SpatialQuery`, or query destruction |
| `FrontierResult` spans | mutation/destruction of that owning `FrontierResult` |
| fixed `FrontierResultSink` output | caller-defined storage lifetime |
| `CollectResult::freedPayloads` | next `collect()` call or database destruction |
| `Hierarchy::page(id)` view | `takePage(id)`, hierarchy destruction, or replacement |
| borrowed `PageView` asset bytes | asset release, with storage kept alive throughout |
| `PageUsageContext` observations | consumed by collection or `reset()` |
| published selection snapshot | next database mutation after all queries join |
| page/node handle | its mount is detached or collected |
| instance reference | that instance is removed |

For implementation details, see [ARCHITECTURE.md](ARCHITECTURE.md). For page,
selection, streaming, and complexity invariants, see
[frontier_design.md](frontier_design.md).
