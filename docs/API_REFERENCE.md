# Frontier API Reference

This document is the exhaustive reference for Frontier 0.7.0. Start with the
[API guide](API.md) for the integration flow. All names below are in namespace
`frontier` unless stated otherwise. `QueryScratch`,
`SpatialDatabase::TestAccess`, and all declarations in `frontier::detail` are
implementation or test interfaces and are not part of the supported public
API.

Frontier requires C++20.

## Reference conventions

- **Contract violation** means `FRONTIER_FATAL` is called. The default macro
  throws `std::logic_error`. A replacement handler must not return.
- **Stale** means a generation-stamped handle no longer resolves. Operations
  explicitly documented as stale-safe either do nothing, return `false`, or
  return an invalid handle.
- **Writer phase** means no `SpatialQuery::selectFrontier()` is running against
  the database. Unless a function is explicitly a const query, call it only in
  the single-writer phase.
- Spans never own their elements. Their lifetime is stated with the producing
  function.
- Translation components use `float4`; only `x`, `y`, and `z` are spatial.
  Transforms support translation and positive uniform scale, not rotation.

The main headers are:

```cpp
#include <frontier/builder.h>          // NodeDesc, SubtreeBuilder, SubtreeBytes
#include <frontier/spatial_database.h> // runtime and selection API
```

`math.h`, `node.h`, `subtree.h`, `config.h`, and `append_buffer.h` may also be
included directly.

## 1. Compile-time configuration and host context

Declared in `frontier/config.h`.

### Version macros

```cpp
FRONTIER_VERSION_MAJOR   // 0
FRONTIER_VERSION_MINOR   // 7
FRONTIER_VERSION_PATCH   // 0
FRONTIER_VERSION_STRING  // "0.7.0"
```

### Diagnostics macros

```cpp
FRONTIER_FATAL(message)
FRONTIER_CHECK(condition, message)
FRONTIER_ASSERT(condition, message)
```

`FRONTIER_FATAL` handles caller-visible contract failures and must not return.
Define it before including any Frontier header to replace the default
exception. `FRONTIER_CHECK` is always enabled. `FRONTIER_ASSERT` is disabled
under `NDEBUG` unless the host overrides it. Normal streaming races involving
stale handles do not use these macros.

### SIMD and vector configuration

`FRONTIER_FORCE_SCALAR` disables intrinsic implementations. Otherwise the
header selects AVX2, 64-bit NEON, SSE2, or the scalar backend from compiler
target macros. `FRONTIER_PREFETCH(pointer)` exposes the selected best-effort
prefetch operation.

Define `FRONTIER_USE_CUSTOM_VECTOR_TYPES` and declare compatible
`frontier::float4` and `frontier::float4x4` types before the first Frontier
include to use engine vector types. `float4` must be 16 bytes, at least
16-byte aligned, expose `float x/y/z/w`, and support the same construction,
arithmetic, and free-function operations used by `math.h`. `float4x4` must
expose the same 16-float `m` representation used by the camera overload.
`float8` and `WideBounds` are not replaceable.

### Allocation and parallel callbacks

```cpp
using AllocFn = void* (*)(size_t bytes, size_t alignment, void* user);
using FreeFn = void (*)(void* ptr, void* user);
using ParallelForFn = void (*)(
    uint32_t count,
    void (*fn)(uint32_t i, void* payload),
    void* payload,
    void* user);
```

#### `AllocFn`

Allocates `bytes` with at least `alignment` alignment.

- **Parameters:** `bytes` is the requested allocation size; `alignment` is a
  power-of-two alignment; `user` is `FrontierContext::user`.
- **Returns:** a suitably aligned allocation, or `nullptr` on failure.
- **Contract:** allocations requested for `SubtreeBytes` must satisfy
  `kSubtreeByteAlignment`.

#### `FreeFn`

Releases an allocation returned by the paired `AllocFn`.

- **Parameters:** `ptr` is the allocation; `user` is the copied context's user
  pointer.
- **Expected result:** the allocation is no longer accessible.

#### `ParallelForFn`

Invokes `fn(i, payload)` once for every `i` in `[0, count)`.

- **Parameters:** `count` is the task count; `fn` is Frontier's task body;
  `payload` is Frontier-owned call state; `user` is the host context pointer.
- **Contract:** tasks may execute in any order and on any thread, but the
  callback must not return until all tasks have completed.

```cpp
void* defaultAlloc(size_t bytes, size_t alignment, void* user);
void defaultFree(void* ptr, void* user);
void defaultParallelFor(uint32_t count,
                        void (*fn)(uint32_t, void*),
                        void* payload, void* user);
const FrontierContext& defaultContext();
```

The defaults provide aligned host allocation and serial execution.
`defaultContext()` returns a process-wide immutable context using them.

```cpp
struct FrontierContext {
    AllocFn alloc = &defaultAlloc;
    FreeFn free = &defaultFree;
    ParallelForFn parallelFor = &defaultParallelFor;
    uint32_t workerCount = 1;
    void* user = nullptr;
};
```

- `alloc` and `free` own serialized subtree allocations.
- `parallelFor` is used only by eligible uncached selections.
- `workerCount` is the maximum concurrent callback count and sizes per-worker
  storage. A database normalizes zero to one.
- Callback code and anything reachable through `user` must outlive the
  database and all `SubtreeBytes` allocations using the context.

## 2. Math and cameras

Declared in `frontier/math.h`. The scalar types and camera constructors are the
normal application-facing surface. The eight-lane types are public primarily
to support Frontier's traversal representation.

### Scalar/vector helpers

```cpp
float fmadd(float a, float b, float c);
inline constexpr uint32_t kWide = 8;
```

`fmadd()` returns the backend-matching multiply-add result. `kWide` is the
number of lanes in `float8` and `WideBounds`.

```cpp
struct alignas(16) float4 {
    float x, y, z, w;
    static float4 splat(float s);
    static float4 point(float x, float y, float z);
    static float4 vec(float x, float y, float z);
};
```

- `splat()` sets all four components to `s`.
- `point()` sets `w` to 1; `vec()` sets `w` to 0.
- `operator+`, `operator-`, scalar `operator*`, scalar `operator/`, `min4()`,
  `max4()`, `dot3()`, `cross3()`, `length3()`, and `normalize3()` provide the
  corresponding component-wise or three-component operations.
- `normalize3()` requires non-zero length.

```cpp
struct alignas(16) float4x4 {
    float m[16];
    static float4x4 fromMemory(const float* m16);
    float4 coeffs(int component) const;
};
```

- `fromMemory()` copies 16 floats from graphics-API matrix storage.
- `coeffs(component)` returns the coefficients producing clip component 0
  through 3. `component` must be in that range.

### `AABB`

```cpp
struct AABB {
    float4 mn;
    float4 mx;

    static AABB empty();
    static AABB fromMinMax(float4 lo, float4 hi);
    static AABB fromCenterExtent(float4 center, float4 extent);
    bool isEmpty() const;
    void expand(const AABB& other);
    void expand(float4 point);
    bool contains(const AABB& other) const;
    float4 center() const;
    float4 extent() const;
};
```

- `empty()` returns the canonical inverted box.
- `fromMinMax()` and `fromCenterExtent()` construct boxes without validating
  inputs.
- `expand()` grows the receiver to include the argument.
- `contains()` implements true containment: any box contains an empty box, and
  an empty box contains no non-empty box.
- `center()` and `extent()` are meaningful for non-empty boxes.

```cpp
float distanceToBox(const AABB& box, float4 queryMin, float4 queryMax);
float distanceToBox(const AABB& box, float4 point);
AABB toWorld(const AABB& box, float4 position, float scale);
```

The distance overloads return zero for touching or overlapping geometry.
`toWorld()` applies translation and uniform scale and preserves empty boxes;
`scale` is expected to be positive.

### Frustum culling

```cpp
struct Frustum { float4 plane[6]; };
inline constexpr uint8_t kAllPlanes = 0x3f;
enum class CullState : uint8_t { Outside, Partial, Inside };
CullState testAabb(const AABB& box, const Frustum& frustum,
                   uint8_t& ioMask);
```

Planes point inward; a point is inside when `dot3(plane.xyz, point) + plane.w`
is non-negative. `ioMask` supplies the planes still requiring tests and is
narrowed on return. `Inside` leaves it zero, `Partial` leaves undecided bits,
and `Outside` means an active plane rejected the box.

### Camera construction

```cpp
struct Camera {
    float4 pos;
    Frustum frustum;
    float k = 1.0f;
    uint32_t viewMask = ~0u;
    float4 envLo{}, envHi{};

    float4 queryMin() const;
    float4 queryMax() const;
    bool damped() const;
};
```

Screen error is `geometricError * k / distance`. `viewMask` is ANDed with an
instance mask. `envLo` and `envHi` widen the point camera into the LOD query
box `[queryMin(), queryMax()]`; zero values disable that envelope.

```cpp
Camera withEnvelope(const Camera& camera, float4 otherPosition);
```

Returns a copy whose envelope also contains `otherPosition`.

```cpp
Camera makePerspectiveCamera(float4 position, float4 forward, float4 up,
                             float fovY, float aspect,
                             float viewportHeightPx,
                             float nearDistance, float farDistance);
```

- **Parameters:** `fovY` is in radians; distances and basis vectors use world
  space; `viewportHeightPx` establishes pixel error scale.
- **Returns:** a camera with six normalized inward planes and the corresponding
  error scale.
- **Preconditions:** `forward` and the derived right vector are non-zero;
  projection dimensions and clip distances form a valid perspective camera.

```cpp
enum class ClipRange : uint8_t {
    ZeroToOne,
    MinusOneToOne,
};

Camera cameraFromViewProjection(
    const float* matrix16, float4 cameraPosition,
    float viewportHeightPx, float projectionYScale,
    ClipRange range = ClipRange::ZeroToOne);

Camera cameraFromViewProjection(
    const float4x4& viewProjection, float4 cameraPosition,
    float viewportHeightPx, float projectionYScale,
    ClipRange range = ClipRange::ZeroToOne);
```

- **Parameters:** the matrix is the 16-float combined view-projection layout
  shared by row-vector/row-major DirectX and column-vector/column-major GL
  storage; `projectionYScale` is projection matrix element `[1][1]`;
  `range` identifies clip-space depth convention.
- **Returns:** a camera with planes extracted from the matrix. Reverse-Z needs
  no separate flag.

```cpp
Camera cameraFromPlanes(const float4 planes[6], float4 cameraPosition,
                        float errorScaleK);
```

Copies six caller-provided inward planes and the supplied screen-error scale.

```cpp
Camera makeLookAtCamera(float4 position, float4 target,
                        float fovY = 1.0f,
                        float aspect = 16.0f / 9.0f,
                        float viewportHeightPx = 1080.0f,
                        float nearDistance = 0.1f,
                        float farDistance = 1.0e9f);
```

Convenience wrapper around `makePerspectiveCamera()` with a stable derived up
vector. `position` and `target` must differ.

```cpp
Camera toLocal(const Camera& camera, float4 instancePosition,
               float instanceScale);
float screenError(float geometricError, float k, float distance);
```

`toLocal()` returns the camera expressed in the translated, uniformly scaled
instance space and requires positive scale. `screenError()` floors distance at
`1e-30` before division.

### `CameraDamper`

```cpp
class CameraDamper {
public:
    CameraDamper();
    explicit CameraDamper(float halfLifeFrames);
    float halfLife() const;
    void setHalfLife(float frames);
    void reset();
    Camera damp(const Camera& camera);
};
```

- `halfLife()` returns the configured relaxation time in frames.
- `setHalfLife()` clamps non-positive values to zero.
- `reset()` forgets the accumulated envelope but keeps the configured
  half-life.
- `damp()` returns a camera whose position/projection envelope includes recent
  history. Zero or negative half-life returns the input unchanged.
- Do not pre-damp a camera that is also passed to a damped `SpatialQuery`.

### Eight-lane traversal helpers

```cpp
struct alignas(32) float8 {
    float v[kWide];
    static float8 splat(float value);
    float operator[](uint32_t lane) const;
    float& operator[](uint32_t lane);
};
```

`operator+`, `operator-`, `operator*`, `operator/`, `min8()`, `max8()`, and
`sqrt8()` operate lane by lane. Lane indices must be less than `kWide`.

```cpp
struct WideBounds {
    float8 mnx, mny, mnz;
    float8 mxx, mxy, mxz;

    static WideBounds allEmpty();
    void setLane(uint32_t lane, const AABB& bounds);
    AABB lane(uint32_t lane) const;
};
```

`allEmpty()` initializes all eight lanes as empty; `setLane()` and `lane()`
write and read one lane.

```cpp
uint32_t testWideAabb(const WideBounds& bounds, const Frustum& frustum,
                      uint8_t inMask, uint8_t outMasks[kWide]);
float8 distanceToBoxes(const WideBounds& bounds,
                       float4 queryMin, float4 queryMax);
float8 distanceToBoxes(const WideBounds& bounds, float4 point);
float8 distanceToBoxesSq(const WideBounds& bounds,
                         float4 queryMin, float4 queryMax);
float8 screenError8(const float8& errors, float k,
                    const float8& distances);
float8 screenErrorFromSq8(const float8& errors, float k,
                          const float8& squaredDistances);
```

`testWideAabb()` returns one survivor bit per lane and writes narrowed plane
masks for surviving lanes. The distance and error functions return one result
per lane. Their backend-specific numerical path is selected at compile time.

## 3. Node authoring types

Declared in `frontier/node.h`.

```cpp
using UserPayload = uint64_t;
inline constexpr UserPayload kSentinelPayload = ~0ull;
inline constexpr uint32_t kInvalidIndex = 0xffffffffu;
```

`UserPayload` is application-owned opaque data and need not be unique.
`kSentinelPayload` is reserved for the serialized implicit-parent record;
applications should not assign it to a renderable node. `kInvalidIndex` is the
library's public invalid-index value.

### `Transform`

```cpp
struct Transform {
    float4 pos = float4::point(0, 0, 0);
    float scale = 1.0f;
};
```

Represents translation plus positive uniform scale. It occupies 32 bytes.

### `ScalarAABB`

```cpp
struct ScalarAABB {
    struct XYZ { float x, y, z; };
    XYZ mn;
    XYZ mx;

    ScalarAABB();
    ScalarAABB(const AABB& bounds) noexcept;
    ScalarAABB& operator=(const AABB& bounds) noexcept;
    AABB toAABB() const noexcept;
    operator AABB() const noexcept;
    bool isEmpty() const noexcept;
};
```

Exact six-float authoring storage. Conversion to and from `AABB` does not
quantize spatial components. It occupies 24 bytes.

### `NodeDesc`

```cpp
struct NodeDesc {
    enum Flag : uint32_t { FlagMountable = 1u << 0 };

    UserPayload payload = 0;
    float geometricError = 0.0f;
    uint32_t flags = 0;
    ScalarAABB bounds = AABB::empty();

    bool isMountable() const noexcept;
};
```

- `payload` is returned through live node handles.
- `geometricError` must be finite and non-negative.
- `flags` currently accepts `FlagMountable`; keep all reserved bits zero.
- `bounds` is the node's conservative hierarchy-local bound.
- `isMountable()` tests `FlagMountable`.

A mountable builder node must remain a local leaf. A mountable TLAS root or
mounted node may receive one runtime child placement. `NodeDesc` occupies 40
bytes.

## 4. Serialized subtree storage

Declared in `frontier/subtree.h`.

```cpp
inline constexpr size_t kSubtreeByteAlignment = 64;
```

All non-empty serialized subtree arrays are 64-byte aligned.

### `SubtreeBytes`

```cpp
class SubtreeBytes {
public:
    SubtreeBytes();
    explicit SubtreeBytes(
        size_t size,
        const FrontierContext& context = defaultContext());
    ~SubtreeBytes();

    SubtreeBytes(const SubtreeBytes& other);
    SubtreeBytes& operator=(const SubtreeBytes& other);
    SubtreeBytes(SubtreeBytes&& other) noexcept;
    SubtreeBytes& operator=(SubtreeBytes&& other) noexcept;

    std::byte* data();
    const std::byte* data() const;
    size_t size() const;
    bool empty() const;
    std::span<std::byte> bytes();
    std::span<const std::byte> bytes() const;
};
```

- The default constructor creates an empty array.
- The sized constructor allocates exactly `size` bytes using a copy of
  `context`; zero size allocates nothing. Non-zero construction requires
  non-null `alloc` and `free`, successful allocation, and 64-byte alignment.
- Destruction releases the allocation through the copied context.
- Copying duplicates the bytes using the source allocation context.
- Moving transfers the allocation and leaves the source empty.
- `data()` returns the allocation pointer, or `nullptr` when empty.
- `bytes()` returns a mutable or const span over the complete array. Any move,
  assignment, destruction, or successful ownership transfer invalidates it.

The type does not distinguish builder-produced and file-loaded data. A caller
loading from disk constructs the final-sized array and reads directly into
`bytes()`. Persisted arrays are a versioned native traversal format;
registration requires matching format version, byte order, layout, size, and
alignment. They are not a format-independent interchange schema, and
registration is not a hardened validation boundary for untrusted bytes.

## 5. `SubtreeBuilder`

Declared in `frontier/builder.h`.

```cpp
class SubtreeBuilder {
public:
    using NodeId = uint32_t;

    SubtreeBuilder();
    void reserve(uint32_t nodeCount);
    NodeId createNode(const NodeDesc& node);
    NodeId createNode(NodeId parent, const NodeDesc& node);
    SubtreeBytes build(
        const FrontierContext& context = defaultContext());
};
```

#### `reserve(nodeCount)`

- **Parameters:** expected count of renderable builder nodes.
- **Effects:** reserves authoring storage; it does not create nodes.
- **Contract:** the builder must not already have been consumed by `build()`.

#### `createNode(node)`

- **Parameters:** descriptor for a direct child of the eventual runtime mount
  parent.
- **Returns:** a builder-local `NodeId` usable by later `createNode()` calls.
- **Contract:** the builder is unconsumed, error is finite/non-negative, and
  the definition has no more than 511 direct nodes.

#### `createNode(parent, node)`

- **Parameters:** `parent` is a previously returned id; `node` describes its
  new local child.
- **Returns:** the new builder-local id.
- **Contract:** `parent` exists and is not mountable, the builder is
  unconsumed, error is finite/non-negative, and parent fanout stays at or below
  511.

#### `build(context)`

- **Parameters:** allocation context copied into the returned byte array.
- **Returns:** a complete, traversal-ready `SubtreeBytes` value.
- **Effects:** permanently consumes the builder; derives interior and implicit
  parent bounds bottom-up, clamps child errors to parent errors, packs nodes in
  depth-first order, and creates wide traversal blocks.
- **Contract:** at least one renderable node exists and every final renderable
  node bound is non-empty. All earlier authoring contracts still apply.

Builder `NodeId` values do not survive `build()` and are unrelated to runtime
`NodeHandle` values.

## 6. Runtime handles

Declared in `frontier/spatial_database.h`. Handles are small value types with
generation stamps. Preserve the complete value; do not use its fields as
application identity.

### `SubtreeHandle`

```cpp
struct SubtreeHandle {
    uint32_t slot = kInvalidIndex;
    uint32_t generation = 0;
    bool valid() const;
};
```

Names one registered immutable subtree definition. `valid()` tests only the
invalid value; call `SpatialDatabase::isSubtree()` to test liveness in a
specific database. Size: 8 bytes.

### `SubtreeInstanceHandle`

```cpp
struct SubtreeInstanceHandle {
    uint32_t slot = kInvalidIndex;
    uint32_t generation = 0;
    bool valid() const;
};
```

Names one mounted placement. `valid()` is a syntactic check;
`SpatialDatabase::isMounted()` tests liveness. Size: 8 bytes.

### `NodeHandle`

```cpp
struct NodeHandle {
    static constexpr uint32_t kSlotBits = 20;
    static constexpr uint32_t kIndexBits = 20;
    static constexpr uint32_t kGenerationBits = 24;
    static constexpr uint32_t kSlotMask = (1u << kSlotBits) - 1;
    static constexpr uint32_t kIndexMask = (1u << kIndexBits) - 1;
    static constexpr uint32_t kGenerationMask =
        (1u << kGenerationBits) - 1;
    static constexpr uint32_t kInvalidSlot = kSlotMask;
    static constexpr uint32_t kTlasGenerationBits = 20;
    static constexpr uint32_t kTlasGenerationMask =
        (1u << kTlasGenerationBits) - 1;

    NodeHandle();
    NodeHandle(uint32_t slot, uint32_t index, uint32_t generation);

    uint32_t slot() const;
    uint32_t index() const;
    uint32_t generation() const;
    bool isTlasRoot() const;
    bool valid() const;
    uint32_t tlasInstance() const;
    uint32_t tlasGeneration() const;
    static NodeHandle tlasRoot(uint32_t instance,
                               uint32_t instanceGeneration);
    friend bool operator==(NodeHandle, NodeHandle);
};
```

Represents either a renderable node in a mounted placement or a permanent TLAS
root. Normal application code obtains it from `InstanceHandle::rootNode()`, a
`FrontierEntry`, or retained assembly state. The packing helpers are exposed
for value inspection and testing; constructing arbitrary live handles is not a
supported way to discover nodes. Size: 8 bytes.

Mounted handles use 20-bit placement slots, 20-bit local indices, and 24-bit
generations. The reserved slot tag encodes a TLAS root with a 24-bit public
instance id and 20-bit root generation.

### `InstanceHandle`

```cpp
using InstanceId = uint32_t;
inline constexpr uint32_t kInstanceIdBits = 24;
inline constexpr InstanceId kInstanceIdMask = (1u << 24) - 1;
inline constexpr InstanceId kInvalidInstanceId = kInstanceIdMask;

struct InstanceHandle {
    InstanceId id = kInvalidInstanceId;
    uint32_t generation = 0;
    bool valid() const;
    NodeHandle rootNode() const;
};
```

Names one top-level instance. `rootNode()` returns its permanent renderable
root, or an invalid node for an invalid instance handle. `valid()` does not
consult a database. Size: 8 bytes.

## 7. Frontier result types

### Error encoding

```cpp
inline constexpr uint8_t kFrontierErrorThreshold = 128;
uint8_t encodeFrontierError(float error, float threshold);
float decodeFrontierError(uint8_t code, float threshold);
```

The encoding is threshold-relative and logarithmic. Codes 0 through 127 are at
or below the threshold; 128 through 255 are above it. The side of the threshold
is exact, while magnitude is quantized at roughly eight codes per power of
two. `decodeFrontierError()` returns a representative magnitude, not the
original error.

### `FrontierEntry`

```cpp
struct FrontierEntry {
    NodeHandle nodeHandle;
    uint32_t instanceAndError;

    FrontierEntry();
    FrontierEntry(NodeHandle node, float error, float threshold,
                  InstanceId instance);
    FrontierEntry(NodeHandle node, uint8_t encodedError,
                  InstanceId instance);

    InstanceId instance() const;
    uint8_t errorCode() const;
    bool overThreshold() const;
    float approximateError(float threshold) const;
};
```

- `nodeHandle` identifies the renderable or desired node.
- `instance()` returns the public top-level id associated with the entry. It is
  stable while that instance is live and suitable for indexing a caller-side
  entity/transform table during that lifetime; ids may be recycled after
  removal.
- `errorCode()` returns the packed threshold-relative value.
- `overThreshold()` is true for code 128 or greater.
- `approximateError()` decodes a representative pixel error for the supplied
  selection threshold.

Size: 12 bytes.

### Result buckets

```cpp
struct FrontierResultView {
    std::span<const FrontierEntry> shared;
    std::span<const FrontierEntry> currentOnly;
    std::span<const FrontierEntry> idealOnly;

    size_t currentSize() const;
    size_t idealSize() const;
    size_t size() const;
    bool empty() const;
};
```

The current render cut is `shared + currentOnly`. The fully resident desired
cut is `shared + idealOnly`. `size()` counts all three disjoint storage
buckets. A view returned by `SpatialQuery` remains valid until that query's
next selection, `reset()`, move assignment, or destruction.

```cpp
class FrontierResult : public FrontierResultView {
public:
    FrontierResult();
    FrontierResult(const FrontierResult&);
    FrontierResult(FrontierResult&&) noexcept;
    FrontierResult& operator=(const FrontierResult&);
    FrontierResult& operator=(FrontierResult&&) noexcept;
};
```

Owns all three buckets. Copy and move operations retarget the inherited spans
to the destination's storage. Its spans remain valid until the result is
modified, assigned, moved from, or destroyed.

### Fixed output sinks

```cpp
template <class T>
class Sink {
public:
    Sink();
    explicit Sink(std::span<T> storage);
    void push(const T& value);
    void pushRange(const T* values, uint32_t count);
    uint32_t count() const;
    uint32_t dropped() const;
    bool overflowed() const;
};
```

- The default sink has zero capacity and drops every pushed element.
- The span constructor writes into caller-owned storage and requires capacity
  not to exceed `UINT32_MAX`.
- `push()` and `pushRange()` write only elements that fit and count the rest.
- `count()` is the number written; `dropped()` is the number omitted;
  `overflowed()` is equivalent to `dropped() != 0`.
- Caller storage must remain valid and unmodified for the selection call. The
  sink does not own it.

```cpp
struct FrontierResultSink {
    Sink<FrontierEntry> shared;
    Sink<FrontierEntry> currentOnly;
    Sink<FrontierEntry> idealOnly;

    FrontierResultSink();
    FrontierResultSink(Sink<FrontierEntry> shared,
                       Sink<FrontierEntry> currentOnly,
                       Sink<FrontierEntry> idealOnly);
};
```

One independently sized sink per result bucket. Inspect each member's count
and overflow state after selection.

### Selection inputs and diagnostics

```cpp
struct SelectionParams {
    float threshold = 4.0f;
    float minPix = 0.0f;
};
```

- `threshold` is the screen-error refinement threshold in pixels. Use a
  positive finite value for ordinary selection.
- `minPix` enables top-level contribution culling when greater than zero;
  zero disables it.

```cpp
struct InstanceDesc {
    float4 pos{};
    float scale = 1.0f;
    uint32_t mask = ~0u;
};
```

Describes one TLAS placement. `scale` must be finite and positive.
`mask & Camera::viewMask == 0` culls the instance at the top level.

```cpp
struct SelectionStats {
    uint64_t instancesVisited = 0;
    uint64_t subtreesVisited = 0;
    uint64_t nodesVisited = 0;
    uint64_t wideBlocksTested = 0;
    uint64_t lanesSurvived = 0;
};
```

Filled by selection only in builds compiled with `FRONTIER_STATS`. Otherwise
the counters remain zero.

```cpp
struct CollectResult {
    size_t unmountedSubtrees = 0;
    std::span<const UserPayload> freedPayloads;
};
```

`unmountedSubtrees` counts placements removed by one collection pass.
`freedPayloads` lists resident mounted-node payload values made unreachable;
values can repeat. The span is database-owned and remains valid until the next
`collect()` call or database destruction.

## 8. `SpatialQuery`

One mutable query stores damping, exact-cut reuse records, scratch/output,
statistics, and optional mount-usage feedback for one logical view.

```cpp
class SpatialQuery {
public:
    SpatialQuery();
    explicit SpatialQuery(float halfLifeFrames);
    ~SpatialQuery();

    SpatialQuery(SpatialQuery&&) noexcept;
    SpatialQuery& operator=(SpatialQuery&&) noexcept;
    SpatialQuery(const SpatialQuery&) = delete;
    SpatialQuery& operator=(const SpatialQuery&) = delete;
};
```

The default uses zero damping and enabled reuse. The type is movable but not
copyable. A moved-from query may be destroyed or assigned; assign or construct
a valid query before selecting with it again.

### Damping and reuse controls

```cpp
float halfLife() const;
void setHalfLife(float frames);
bool reuseEnabled() const;
void setReuseEnabled(bool enabled);
uint32_t reused() const;
uint32_t walked() const;
const SelectionStats& lastSelectionStats() const;
```

- `halfLife()` and `setHalfLife()` access the query-owned camera damper;
  non-positive values disable damping.
- `reuseEnabled()` reports whether exact frontier-record reuse is active.
- `setReuseEnabled()` resets damping/reuse/usage state when the value changes,
  while retaining allocations and the configured half-life.
- `reused()` and `walked()` report instance counts from the most recent
  selection.
- `lastSelectionStats()` returns a query-owned reference overwritten by the
  next selection or reset.

### Mount-usage controls

```cpp
bool mountUsageEnabled() const;
void setMountUsageEnabled(bool enabled);
void resetMountUsage();
```

Tracking is disabled by default. When enabled, selections record mounted
placements touched by that view. `collect(query, ...)` or
`collect(queries, ...)` consumes pending feedback. Disabling tracking or
calling `resetMountUsage()` discards feedback not yet consumed.

### Selection overloads

```cpp
FrontierResultView selectFrontier(
    const SpatialDatabase& database,
    const Camera& camera,
    const SelectionParams& params);

void selectFrontier(const SpatialDatabase& database,
                    const Camera& camera,
                    const SelectionParams& params,
                    FrontierResultSink& outResult);

void selectFrontier(const SpatialDatabase& database,
                    const Camera& camera,
                    const SelectionParams& params,
                    FrontierResult& outResult);
```

- **Parameters:** `database` must expose a snapshot published by
  `applyUpdates()`; `camera` is raw input and is damped internally; `params`
  controls refinement and contribution culling. The final overload argument
  selects query-owned, caller-fixed, or caller-owned output.
- **Returns/results:** all overloads produce the same ordered three-bucket cut.
  The returned view uses query-owned storage. The sink overload reports
  truncation through its sinks. The owning overload replaces `outResult`'s
  contents.
- **Threading:** the function mutates the query. Do not select concurrently on
  one query. Distinct queries may read the same published database snapshot
  concurrently.
- **Binding:** the first selection binds the query to `database`. Selecting a
  different database before `reset()` is a contract violation.
- **Reuse:** cached entries reproduce the exact node set but may retain their
  earlier quantized magnitude within the proven reuse margin.

### Reset and storage

```cpp
void reset();
size_t bytes() const;
```

`reset()` releases database binding and clears damping history, reuse records,
pending usage, last counters, and current output while retaining allocations,
reuse mode, and configured half-life. Call it for camera cuts or teleports.
`bytes()` returns retained query/cache/scratch capacity in bytes.

## 9. Database configuration

### `TlasQuality`

```cpp
enum class TlasQuality : uint8_t {
    Morton,
    Median,
    BinnedSAH,
};
```

- `Morton` performs one spatial sort and contiguous wide grouping; it is the
  least expensive and loosest build.
- `Median` recursively splits the longest axis at the median.
- `BinnedSAH` uses a binned surface-area heuristic and normally gives the best
  traversal quality.

### `SpatialDatabaseConfig`

```cpp
struct SpatialDatabaseConfig {
    FrontierContext context{};
    TlasQuality tlasQuality = TlasQuality::BinnedSAH;
    float tlasTraversalCost = 1.0f;
    float tlasIntersectCost = 1.0f;
    float tlasCountDrift = 0.2f;
    float tlasAreaDrift = 0.5f;
    float tlasEscapeFraction = 0.25f;
    float tlasEditFraction = 0.05f;
    uint32_t parallelInstanceThreshold = 0;
};
```

- `context` supplies callbacks and is copied into the database.
- `tlasQuality` selects initial and promoted rebuild quality.
- `tlasTraversalCost` and `tlasIntersectCost` are the Binned-SAH cost terms;
  increasing intersection cost favors deeper, tighter trees.
- `tlasCountDrift` is the population-change fraction that promotes a quality
  rebuild.
- `tlasAreaDrift` is the allowed grow-only refit area relative to the last
  quality build.
- `tlasEscapeFraction` is the allowed fraction of distinct leaves that escape
  their build-time lanes.
- `tlasEditFraction` bounds accumulated incremental spawn/removal edits before
  rebuild.
- `parallelInstanceThreshold` is the minimum visible-instance count for
  uncached parallel selection. Zero disables it; `context.workerCount` must
  also exceed one.

Hosts should supply finite, non-negative cost and drift values. Internal
parallel selection is blocking and concatenates worker output in instance
order, so serial and parallel cuts are identical.

## 10. `SpatialDatabase`

```cpp
class SpatialDatabase {
public:
    explicit SpatialDatabase(
        const SpatialDatabaseConfig& config = SpatialDatabaseConfig{});
    ~SpatialDatabase();

    SpatialDatabase(const SpatialDatabase&) = delete;
    SpatialDatabase& operator=(const SpatialDatabase&) = delete;
    SpatialDatabase(SpatialDatabase&&) = delete;
    SpatialDatabase& operator=(SpatialDatabase&&) = delete;

    const SpatialDatabaseConfig& config() const;
};
```

Construction copies the configuration and normalizes `workerCount == 0` to
one. Destruction releases registered definitions and all runtime storage. The
database is neither copyable nor movable. `config()` returns a reference valid
for the database lifetime.

### Registered subtree definitions

```cpp
SubtreeHandle registerSubtree(SubtreeBytes&& bytes);
```

- **Parameters:** an owning, 64-byte-aligned array in the current serialized
  format. Pass a named array with `std::move`; a `build()` temporary binds
  directly.
- **Returns:** a live definition handle unique to this registration.
- **Effects:** validates the serialized header, size, layout, version, and
  implicit-parent sentinel, then moves the allocation into the database. It
  does not unpack or copy node arrays.
- **Contract:** `bytes` is non-empty and valid. Contract failure leaves the
  input's post-failure state unspecified.
- **Notes:** identical arrays registered twice produce independent handles.
  Definition deduplication is application policy.

```cpp
void releaseSubtree(SubtreeHandle subtree);
```

- **Parameters:** definition to release.
- **Effects:** destroys the registered bytes and invalidates the handle.
- **Stale behavior:** an invalid or stale handle is ignored.
- **Contract:** no mounted placement may reference the live definition.

```cpp
bool isSubtree(SubtreeHandle subtree) const;
size_t subtreeCount() const;
```

`isSubtree()` tests slot and generation against this database.
`subtreeCount()` returns the number of live registered definitions.

### Top-level instance lifecycle

```cpp
InstanceHandle instantiate(const NodeDesc& root,
                           const InstanceDesc& desc = {});
```

- **Parameters:** `root` describes the permanent renderable fallback;
  `desc` supplies world translation, uniform scale, and view mask.
- **Returns:** a generation-stamped instance handle whose `rootNode()` is live
  immediately.
- **Effects:** inserts one TLAS leaf. A non-mountable one-node instance needs no
  subtree definition or mounted-placement state.
- **Contract:** root error is finite/non-negative; root bounds are finite and
  non-empty; instance scale is finite and positive; position is expected to be
  finite.
- **Residency:** the root payload is permanent and implicitly resident.

```cpp
void removeInstance(InstanceHandle instance);
```

- **Effects:** removes the TLAS root, recursively unmounts everything beneath
  it, releases its bounds overlays, and invalidates all related instance and
  node handles. Registered definitions remain registered.
- **Stale behavior:** no-op.

```cpp
void moveInstance(InstanceHandle instance, const Transform& transform);
```

- **Parameters:** new world translation and scale for the whole instance.
- **Effects:** moves the TLAS root and every mounted descendant as one object;
  invalidates affected query reuse records but not public handles.
- **Stale behavior:** no-op.
- **Contract:** scale is positive; all transform components are expected to be
  finite.

### `MotionGroup`

```cpp
class SpatialDatabase::MotionGroup {
public:
    MotionGroup();
    explicit MotionGroup(std::span<const InstanceHandle> instances);
    void reset(std::span<const InstanceHandle> instances);
    size_t size() const;
};
```

A motion group owns a copy of a stable caller-order instance cohort and caches
the corresponding physical database order. The cache refreshes automatically
after `optimize()` or another layout change.

- The span constructor is equivalent to default construction plus `reset()`.
- `reset()` replaces the copied handle sequence and invalidates the physical
  order cache.
- `size()` returns the number of caller-order handles, including stale or
  duplicate values.

```cpp
void moveInstances(MotionGroup& group,
                   std::span<const float4> positions,
                   float scale = 1.0f);
```

- **Parameters:** `positions[i]` belongs to the handle copied at group index
  `i`; one uniform scale applies to all live members.
- **Effects:** moves live instances in cached physical order. Stale handles are
  ignored. If the group contains the same live instance more than once, the
  final caller-order position wins.
- **Contract:** position and group sizes match, scale is positive, and values
  are expected to be finite.

### Runtime topology assembly

```cpp
SubtreeInstanceHandle mountSubtree(
    NodeHandle parent,
    SubtreeHandle subtree,
    const Transform& transform = {});
```

- **Parameters:** `parent` is a live mountable TLAS root or mounted leaf;
  `subtree` is a live registered definition; `transform` places that
  definition in the parent's hierarchy space.
- **Returns:** the new mounted placement. If `parent` became stale during
  asynchronous loading, returns an invalid handle without modifying the
  database.
- **Effects:** creates placement-local residency and links while sharing the
  definition bytes. All renderable nodes in the new placement begin
  non-resident. The mount transform is accumulated into top-level
  instance-local space. Child effective errors are capped by the parent's
  effective error without rewriting the definition.
- **Complexity:** acquiring and clearing the placement's
  16-bit-per-packed-node residency block is linear in the child definition's
  node count. The first nested child mounted anywhere in a placement also
  allocates that owner's per-node mount-link array; later links are
  constant-time. Immutable node data is neither copied nor rewritten, and
  residency blocks come from a definition-sized slab pool.
- **Contract:** the definition is live; transform is finite with positive
  scale; live parent is mountable and has no existing mounted child; the
  transformed aggregate definition bounds fit in the parent's containment
  bound. This is the shared authored bound for a mounted node and the current
  instance-local root bound for a TLAS root.

The mount transform is immutable. To reposition a placement, unmount and
mount it again; the replacement receives new residency state and handles.

```cpp
void unmountSubtree(SubtreeInstanceHandle instance);
```

Recursively removes the placement and all mounted descendants. Related mount
and node handles become stale. Registered definitions and application payload
resources are not released. A stale placement handle is ignored.

```cpp
bool isMounted(SubtreeInstanceHandle instance) const;
bool hasMountedSubtree(NodeHandle parent) const;
```

`isMounted()` checks placement slot and generation. `hasMountedSubtree()` is
true when the live root/node has a direct mounted placement, and false for a
stale handle or an empty mount point.

```cpp
bool tryGetNodeTransform(NodeHandle node, Transform& outTransform) const;
```

- **Returns:** `true` for a live root or mounted node; `false` for stale or
  invalid input.
- **Result:** on success, writes the containing placement's accumulated
  local-to-top-level-instance transform. A TLAS root writes identity. On
  failure, `outTransform` is unchanged.

### Payload residency and lookup

```cpp
void markPayloadResident(NodeHandle node);
```

Marks one live mounted node's payload available and incrementally propagates
coverage/residency summaries. A TLAS root or stale node is a no-op. Repeating
the current state is a no-op.

```cpp
void markPayloadNonResident(NodeHandle node);
```

Marks one live mounted node unavailable and propagates coverage changes. A
stale node or an already non-resident node is a no-op. A live TLAS root is a
contract violation because root payloads are permanent.

```cpp
bool isPayloadResident(NodeHandle node) const;
```

Returns `true` for a live TLAS root, the placement-local bit for a live mounted
node, and `false` for stale or invalid input.

```cpp
bool tryGetPayload(NodeHandle node, UserPayload& outPayload) const;
```

Returns `true` and writes the immutable node payload when the handle resolves.
Returns `false` for stale/invalid input and leaves `outPayload` unchanged.

### Per-instance bounds overrides

```cpp
void setNodeBounds(InstanceHandle instance, NodeHandle node,
                   const AABB& localBounds);
```

- **Parameters:** `instance` owns `node`; `localBounds` is the node's new bound
  in its definition-local space.
- **Effects:** queues a last-write-wins exact bound for the node. A root edit
  changes that instance's root bound. For a mounted node, the first edit of an
  `(instance, placement)` pair creates a copy-on-write bounds overlay and
  conservatively grows ancestors across mount boundaries into the TLAS.
  Immutable topology, errors, payloads, and other placements stay shared.
- **Stale behavior:** stale instance or node updates are dropped, including
  updates that become stale while queued.
- **Contract:** bounds are finite and non-empty, and the live node belongs to
  the supplied live instance.

```cpp
void flushBounds();
```

Applies queued bounds edits immediately in submission order. The edited node
is set exactly; ancestor propagation is grow-only. This is normally implicit
in `applyUpdates()` and is exposed for tools or immediate readback.

```cpp
AABB nodeBounds(InstanceHandle instance, NodeHandle node);
```

Flushes pending edits, then returns the effective instance-specific local
bound: an overlay if one exists, otherwise the shared authored bound. Returns
`AABB::empty()` when the instance or node does not resolve. Live inputs are
expected to refer to the same assembled instance tree.

`setNodeBounds()` changes spatial coverage only; it does not store a node pose
or render transform.

### Publication and maintenance

```cpp
void applyUpdates();
```

Flushes bounds, performs any scheduled TLAS rebuild, publishes the state for a
group of read-only selections, and increments `frame()`. Call once before each
selection group, even when the writer made no scene edits, if collection age
should advance. No mutation or collection may overlap selections using the
published snapshot.

```cpp
void optimize();
```

Flushes bounds, forces a quality-tier TLAS rebuild, compacts dead dense
instance slots, and restores physical query-record order. Public
`InstanceHandle` values, root `NodeHandle` values, and `FrontierEntry::instance()`
ids remain stable. This is a heavy synchronization-point operation and does
not advance `frame()` or collection age.

### Mounted-placement collection

```cpp
CollectResult collect(size_t maxMountedSubtrees, uint32_t minAge);

CollectResult collect(SpatialQuery& query,
                      size_t maxMountedSubtrees, uint32_t minAge);

CollectResult collect(std::span<SpatialQuery* const> queries,
                      size_t maxMountedSubtrees, uint32_t minAge);
```

- **Parameters:** `maxMountedSubtrees` is the target placement budget;
  `minAge` is the minimum number of published update epochs since last touch.
  Query overloads first consume opt-in usage accumulated by the supplied
  views; null entries in the span are ignored.
- **Returns:** the number of removed placements and a temporary span of
  resident payload values made unreachable.
- **Effects:** walks the LRU tail and removes eligible cold placements until
  the budget is reached or no candidate remains. Only placements with no
  mounted children and sufficient age are eligible.
- **Binding contract:** every non-null query is unbound or bound to this
  database; feedback from another database is a contract violation.
- **Threading:** collection is a writer operation and must not overlap
  selection.

The no-query overload uses the database's existing LRU timestamps. A query
affects retention only after `setMountUsageEnabled(true)`.

### Introspection

```cpp
size_t mountedSubtreeCount() const;
size_t streamedSubtreeCount() const;
uint32_t frame() const;
size_t overlayCount() const;
size_t overlayBytes() const;
size_t subtreeInstanceStateBytes() const;
```

- `mountedSubtreeCount()` returns the number of live mounted placements.
- `streamedSubtreeCount()` is the same count, retained as a streaming-oriented
  synonym.
- `frame()` returns the latest `applyUpdates()` epoch.
- `overlayCount()` returns live copy-on-write bounds overlay count.
- `overlayBytes()` returns retained overlay storage bytes.
- `subtreeInstanceStateBytes()` returns retained placement records,
  transforms, residency words, stamps, slab capacity, and mount-link capacity;
  immutable registered `SubtreeBytes` are excluded.

## 11. Threading contract

`SpatialDatabase` is single-writer with published concurrent reads:

1. One writer performs registration, assembly, movement, residency, bounds,
   maintenance, or collection.
2. The writer calls `applyUpdates()`.
3. Any number of readers call `selectFrontier()` concurrently using distinct
   `SpatialQuery` and output objects.
4. All readers join before the next database mutation.

One `SpatialQuery` is never concurrently callable because selection mutates
its damping, reuse, usage, statistics, scratch, and output. Const database
lookup helpers are intended for the same stable read interval unless the
application otherwise serializes them with mutation.

The host `parallelFor` callback creates parallel work inside one uncached
selection. It must be blocking. Cached selection remains serial because its
reuse-record updates are query-local and ordered.

## 12. `AppendBuffer<T>` support type

Declared in `frontier/append_buffer.h`. This is retained-capacity append-only
storage used by public sink/result support. `T` must be trivially copyable and
no more aligned than `std::max_align_t`.

```cpp
template <class T>
class AppendBuffer {
public:
    using value_type = T;
    using iterator = T*;
    using const_iterator = const T*;

    AppendBuffer();
    AppendBuffer(const AppendBuffer&);
    AppendBuffer(AppendBuffer&&) noexcept;
    AppendBuffer& operator=(const AppendBuffer&);
    AppendBuffer& operator=(AppendBuffer&&) noexcept;
    ~AppendBuffer();

    void clear() noexcept;
    void reserve(size_t requested);
    void resize_uninitialized(size_t requested);
    void swap(AppendBuffer&) noexcept;
    void push_back(T value);

    template <class... Args>
    T& emplace_back(Args&&... args);

    void append(const T* source, size_t count);

    T* data() noexcept;
    const T* data() const noexcept;
    T* begin() noexcept;
    const T* begin() const noexcept;
    T* end() noexcept;
    const T* end() const noexcept;
    T& operator[](size_t index) noexcept;
    const T& operator[](size_t index) const noexcept;
    T& front() noexcept;
    const T& front() const noexcept;
    T& back() noexcept;
    const T& back() const noexcept;
    size_t size() const noexcept;
    size_t capacity() const noexcept;
    bool empty() const noexcept;
};
```

- Copy operations duplicate logical elements; move operations transfer the
  allocation and leave the source empty.
- `clear()` sets size to zero and retains capacity.
- `reserve()` grows capacity to at least `requested` without changing size.
- `resize_uninitialized()` changes logical size without initializing newly
  exposed elements; every new element must be overwritten before it is read.
- `push_back()` and `emplace_back()` append one value, growing geometrically.
- `append()` copies a contiguous range. `source` must not point into the same
  buffer because growth can relocate it.
- Index, `front()`, and `back()` access require an in-range/non-empty buffer.
- Capacity overflow throws `std::length_error`; allocation failure throws
  `std::bad_alloc`.

`AppendBuffer` uses the C allocation API directly and is not affected by
`FrontierContext`.

## 13. Current representation limits

- One builder node has at most 511 local children; a definition has at most
  511 direct nodes beneath its runtime mount parent.
- Mounted placement slots and definition-local node indices use 20 bits.
- Mounted-node generations use 24 bits.
- TLAS root generations use 20 bits.
- Public instance ids use 24 bits, with the all-ones value reserved.
- `FrontierEntry` instance ids and error codes are packed into one 32-bit word.
- Runtime transforms are translation plus positive uniform scale.

These are representational limits, not suggested operating budgets. Use the
database memory and count introspection methods to set application-specific
budgets.
