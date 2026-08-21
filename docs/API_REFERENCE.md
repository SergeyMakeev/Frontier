# Frontier API Reference

This document is the exhaustive reference for Frontier 0.7.0. Start with the
[API guide](API.md) for the integration flow. All names below are in namespace
`frontier` unless stated otherwise. `QueryScratch`,
`SpatialDatabase::TestAccess`, and all declarations in `frontier::detail` are
implementation or test interfaces and are not part of the supported public
API.

Frontier requires C++20.

## Reference conventions

- A **BVH** is a bounding-volume hierarchy used to reject spatial groups
  without testing every member. Frontier's **TLAS** is its internal 4- or
  8-wide world-space top-level BVH over independently movable instances. Every
  TLAS leaf is also a permanent renderable root.
- Ray-tracing APIs often call a local hierarchy below the TLAS a **BLAS**
  (bottom-level acceleration structure). Frontier's closest equivalent is a
  registered subtree definition; a mount supplies its placement below a
  renderable node. The API does not expose a `BLAS` type because definitions
  can be mounted recursively and a one-node instance needs no lower hierarchy.
- A **frontier** or **cut** is an ancestor-free set of renderable nodes that
  covers the visible scene. The current cut uses resources available now; the
  ideal cut assumes every node in currently mounted topology is ready.
- A **hole-free** current cut has complete logical hierarchy coverage: Frontier
  retains a parent until ready descendants cover every visible branch that the
  parent represented. This does not describe mesh seams or rasterization.
- **Readiness** means the renderer can dispatch a node's opaque payload.
  **Coverage** is the internal summary proving that a node itself, or a complete
  descendant cut, is ready. Neither term means that additional topology is
  mounted. A complete ready descendant cut remains usable when its ancestor is
  unavailable; fallback to a ready ancestor is needed only when descendant
  coverage is incomplete.
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
  Top-level instances support planar yaw plus positive uniform scale. Mounted
  subtree transforms support translation and positive uniform scale.

The main headers are:

```cpp
#include <frontier/builder.h>          // NodeDesc, SubtreeBuilder, SubtreeBytes
#include <frontier/spatial_database.h> // runtime and selection API
```

`math.h`, `node.h`, `subtree.h`, and `config.h` may also be included directly.

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
With compiler exception support enabled, the default throws
`std::logic_error`; with exceptions disabled, it aborts. Define it before
including any Frontier header to install a host panic handler.
`FRONTIER_CHECK` is enabled by default and compiles out when
`FRONTIER_CONTRACT_CHECKS=0`. `FRONTIER_ASSERT` is disabled under `NDEBUG`
unless the host overrides it. Normal streaming races involving
stale handles do not use these macros. Contract failures are programmer errors,
not a recoverable result channel; unless a function explicitly documents an
early preflight guarantee, no transactional rollback is promised if the
default exception is caught.

Internal `AppendBuffer` capacity and allocation failures are not caller
contracts. They throw `std::length_error` or `std::bad_alloc` when compiler
exception support is enabled and abort when it is disabled.

### SIMD and vector configuration

`FRONTIER_FORCE_SCALAR` disables intrinsic implementations. Otherwise the
header selects AVX2, AArch64 NEON, SSE2, or the scalar backend from compiler
target macros.

The `FRONTIER_BVH_WIDTH` preprocessor macro is a build-wide numeric choice of
`4` or `8`. The CMake setting of the same name additionally accepts `AUTO` and
defaults to it. CMake resolves `AUTO` to BVH8 for the eight-lane AVX2 backend
and BVH4 for four-lane SSE2/NEON or forced-scalar builds, then propagates the
resolved numeric value to consumers. Without CMake, the header default follows
the same target-macro policy. An explicit `4` or `8` overrides automatic
selection. The result changes public wide-type sizes, internal TLAS/BLAS layouts,
and the serialized subtree format; the library and every consumer translation
unit must use the same value. BVH4 uses one 128-bit SIMD group. BVH8 uses AVX2
when enabled, or two 128-bit SSE2/NEON groups.

The CMake `FRONTIER_AVX2=ON` option applies AVX2/FMA flags to BVH8 library and
consumer builds; it does not add runtime CPU dispatch. `FRONTIER_SSE2_ONLY=ON`
is the deployment-safe baseline option for older x86/x64 processors. It
overrides `FRONTIER_AVX2`, selects BVH4 for `AUTO`, forces the SSE2 intrinsic
backend, and applies compiler flags that disable SSE3, SSE4, AVX, and FMA code
generation. Explicit BVH8 remains supported as two 128-bit groups. The option
is invalid on non-x86 targets and is mutually exclusive with
`FRONTIER_FORCE_SCALAR`.

Non-CMake integrations may define `FRONTIER_SSE2_ONLY=1` consistently for the
library and all consumers, but must also configure their compiler for an SSE2
ISA baseline. The macro controls Frontier's explicit intrinsics; compiler flags
are what prevent unrelated scalar code and auto-vectorization from emitting a
higher ISA.

`FRONTIER_IPO=ON` enables compiler-supported interprocedural optimization for
Frontier targets. It defaults off for predictable integration builds; the
repository performance runners enable it. Third-party benchmark libraries are
deliberately excluded from this setting.

Define `FRONTIER_USE_CUSTOM_VECTOR_TYPES` and declare compatible
`frontier::float4` and `frontier::float4x4` types before the first Frontier
include to use engine vector types. `float4` must be 16 bytes, at least
16-byte aligned, expose `float x/y/z/w`, and support the same construction,
arithmetic, and free-function operations used by `math.h`. `float4x4` must
expose the same 16-float `m` representation used by the camera overload.
`float8` and `WideBounds` are not replaceable.

Custom vector types are a build-wide ABI choice: the Frontier library and
every translation unit that includes a Frontier header must see the same
definitions and configuration macros.

`FRONTIER_STATS` enables per-query traversal counters. It adds counter updates
to selection and storage to `SpatialQuery`; without it, statistics read as
zero and the traversal carries no instrumentation. This is also a build-wide
ABI choice. CMake propagates it when configured with `-DFRONTIER_STATS=ON`.

`FRONTIER_DEBUG_TOOLS` exposes read-only TLAS and query-cache snapshots. It is
off by default and adds no fields, counters, or branches to selection. Debug
queries inspect already-maintained state only when called and write variable
output through caller-owned spans. CMake propagates the API declaration with
`-DFRONTIER_DEBUG_TOOLS=ON`; define it consistently for the library and its
consumers.

`FRONTIER_CONTRACT_CHECKS` controls caller-precondition checks and defaults to
`1`. Setting it to `0` removes those branches; violating any documented
precondition is then undefined behavior. It does not disable Debug-only
internal assertions, which are controlled by `NDEBUG`.

`FRONTIER_VALIDATE_SUBTREES` controls complete serialized-subtree validation
at registration and defaults to `1`. CMake propagates `0` when configured with
`-DFRONTIER_VALIDATE_SUBTREES=OFF`. Disabled builds retain constant-time
header, layout, and root-range checks, but trust all remaining traversal arrays
and remove their O(nodes + wide blocks) validation code. Use `0` only when all
registered bytes were produced by a compatible trusted Frontier builder. When
`FRONTIER_CONTRACT_CHECKS=0` as well, those constant-time preconditions are
assumed rather than checked.

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
`defaultAlloc()` returns `nullptr` for zero-sized, invalid-alignment, or
size-rounding-overflow requests.

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
normal application-facing surface. The build-width lane types are public
primarily to support Frontier's traversal representation.

### Scalar/vector helpers

```cpp
float fmadd(float a, float b, float c);
inline constexpr uint32_t kWide = FRONTIER_BVH_WIDTH;
```

`fmadd()` returns the backend-matching multiply-add result. `kWide` is the
number of lanes in `float8` and `WideBounds`. Despite its fixed public type
name, `float8` contains four lanes in a BVH4 build and eight lanes in a BVH8
build.

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

- `fromMemory()` copies 16 floats from non-null graphics-API matrix storage.
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
- `isEmpty()` returns true when any spatial axis is inverted.
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
the `YawRotation` overload additionally returns the exact axis-aligned world
enclosure after planar yaw. `scale` is expected to be positive. Matching
`toLocal()` overloads inverse-transform cameras and conservatively enclose a
rotated damping envelope.

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
- **Contract:** the pointer overload requires a non-null 16-float array.

```cpp
Camera cameraFromPlanes(const float4 planes[6], float4 cameraPosition,
                        float errorScaleK);
```

Copies six caller-provided inward planes and the supplied screen-error scale.
The plane-array pointer must be non-null.

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
- `setHalfLife()` clamps non-positive and non-finite values to zero.
- `reset()` forgets the accumulated envelope but keeps the configured
  half-life.
- `damp()` returns a camera whose position/projection envelope includes recent
  history. A disabled half-life returns the input unchanged.
- Do not pre-damp a camera that is also passed to a damped `SpatialQuery`.

### Wide traversal helpers

```cpp
struct alignas(kWide * sizeof(float)) float8 {
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

`allEmpty()` initializes all `kWide` lanes as empty; `setLane()` and `lane()`
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
using UserPayload = FRONTIER_USER_PAYLOAD;
inline constexpr UserPayload kInvalidPayload = FRONTIER_INVALID_PAYLOAD;
inline constexpr uint32_t kInvalidIndex = 0xffffffffu;
```

`UserPayload` is an opaque application-owned render-resource identifier. Equal
values may refer to the same application resource, but they do not couple
Frontier readiness: readiness belongs to a node in a registered definition.
The type must be trivially copyable, default constructible, equality comparable,
have a unique object representation, and occupy exactly four or eight bytes.
By default it is `uint64_t` and `kInvalidPayload` is `UINT64_MAX`. The invalid
value is reserved and cannot be authored. Supply both macros as consistent
build-wide preprocessor definitions for the library and all consumers to select
another type:

```cpp
// Build configuration, before including any Frontier header:
#define FRONTIER_USER_PAYLOAD uint32_t
#define FRONTIER_INVALID_PAYLOAD UINT32_MAX
```

or:

```cpp
#define FRONTIER_USER_PAYLOAD void*
#define FRONTIER_INVALID_PAYLOAD nullptr
```

At the public boundary a header-only `std::bit_cast` codec maps the configured
type to an internal `uint32_t` or `uint64_t`. Serialized payload arrays, builder
storage, and TLAS-root payload streams contain only that unsigned word. The
conversion normally emits no instructions.

Raw pointers are valid for definitions built and registered in the same
process, but pointer-bearing serialized bytes are not meaningful after process
exit or in another address space. `kInvalidIndex` is the library's separate
public invalid-index value.

### `Transform`

```cpp
struct Transform {
    float4 pos = float4::point(0, 0, 0);
    float scale = 1.0f;
};
```

Represents translation plus positive uniform scale. It occupies 32 bytes.

### `YawRotation` and `InstanceTransform`

```cpp
struct YawRotation {
    float cosine = 1.0f;
    float sine = 0.0f;
};

YawRotation yawRotation(float radians);

struct InstanceTransform {
    float4 pos = float4::point(0, 0, 0);
    float scale = 1.0f;
    YawRotation yaw{};
};
```

`YawRotation` is a finite unit cosine/sine pair for rotation around +Y.
This representation lets animation systems submit an existing
forward vector without database-side trigonometry. `InstanceTransform`
occupies 32 bytes and applies only to top-level placements; mounted subtrees
continue to use `Transform`.

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
    enum Flag : uint32_t {
        FlagMountable = 1u << 0,
        FlagYawInvariantBounds = 1u << 1,
    };

    UserPayload payload{};
    float geometricError = 0.0f;
    uint32_t flags = 0;
    ScalarAABB bounds = AABB::empty();

    bool isMountable() const noexcept;
    bool hasYawInvariantBounds() const noexcept;
};
```

- `payload` identifies the render resources selected by the node and is
  returned through live node handles. Equal values are allowed and have no
  implicit effect on readiness. It must not equal `kInvalidPayload`.
- `geometricError` must be finite and non-negative.
- `FlagMountable` marks an expandable assembly boundary.
- On a TLAS root only, `FlagYawInvariantBounds` promises that `bounds`
  contains the root's content at every planar yaw around the local origin.
  Frontier can then keep one translation-only broadphase envelope while
  rotating mounted detail traversal. `SubtreeBuilder` rejects this flag.
- Keep every other reserved flag bit zero.
- `bounds` is the node's conservative hierarchy-local bound.
- `isMountable()` and `hasYawInvariantBounds()` test their corresponding bits.

A mountable builder node must remain a local leaf. A mountable TLAS root or
mounted node may receive one runtime child placement. `NodeDesc` occupies 36
bytes with a four-byte payload and 40 bytes with an eight-byte payload.

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
registration requires matching format version, byte order, layout, size,
alignment, payload-word width and invalid bit pattern, topology, bounds, errors,
and wide traversal data. Validation is linear in node and wide-block count and
does not unpack or copy the arrays. The bytes are not a format-independent
interchange schema and carry no authentication or application-defined
allocation limit; authenticate and size-limit untrusted files before allocating
`SubtreeBytes` for them.

Each real node's authored bound is stored once, in its lane of its parent's
wide traversal block. The parent stream also carries the sibling ordinal needed
to address that lane directly. The definition-wide aggregate bound is stored in
the header because the implicit parent has no parent block. These are serialized
layout details, not additional public objects or lifetime rules.

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
- **Contract:** the builder must not already have been consumed by `build()`;
  the hint must fit the definition-local node-index limit.

#### `createNode(node)`

- **Parameters:** descriptor for a direct child of the eventual runtime mount
  parent.
- **Returns:** a builder-local `NodeId` usable by later `createNode()` calls.
- **Contract:** the builder is unconsumed, error is finite/non-negative,
  reserved flag bits are zero, and the definition has no more than 511 direct
  nodes.

#### `createNode(parent, node)`

- **Parameters:** `parent` is a previously returned id; `node` describes its
  new local child.
- **Returns:** the new builder-local id.
- **Contract:** `parent` exists and is not mountable, the builder is
  unconsumed, error is finite/non-negative, reserved flag bits are zero, and
  parent fanout stays at or below 511.

#### `build(context)`

- **Parameters:** allocation context copied into the returned byte array.
- **Returns:** a complete, traversal-ready `SubtreeBytes` value.
- **Effects:** on success, permanently consumes the builder; derives interior
  and implicit parent bounds bottom-up, clamps child errors to parent errors,
  packs nodes in depth-first order, and creates wide traversal blocks.
- **Contract:** at least one renderable node exists and every final renderable
  node bound is finite and non-empty on all three axes. All earlier authoring
  contracts still apply.

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
    constexpr bool valid() const noexcept;
    friend constexpr bool operator==(SubtreeHandle, SubtreeHandle);
};
```

All handle types in this section are scoped to the `SpatialDatabase` that
created them. Generation stamps protect against stale reuse within that
database, not accidental use in another database whose packed values may
coincide.

Names one registered immutable subtree definition. `valid()` tests only the
invalid value; call `SpatialDatabase::isSubtree()` to test liveness in a
specific database. Size: 8 bytes.

### `SubtreeInstanceHandle`

```cpp
struct SubtreeInstanceHandle {
    uint32_t slot = kInvalidIndex;
    uint32_t generation = 0;
    constexpr bool valid() const noexcept;
    friend constexpr bool operator==(SubtreeInstanceHandle,
                                     SubtreeInstanceHandle);
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

    constexpr NodeHandle();
    constexpr NodeHandle(uint32_t slot, uint32_t index,
                         uint32_t generation);

    constexpr uint32_t slot() const;
    constexpr uint32_t index() const;
    constexpr uint32_t generation() const;
    constexpr bool isTlasRoot() const;
    constexpr bool valid() const;
    constexpr uint32_t tlasInstance() const;
    constexpr uint32_t tlasGeneration() const;
    static constexpr NodeHandle tlasRoot(
        uint32_t instance, uint32_t instanceGeneration);
    friend constexpr bool operator==(NodeHandle, NodeHandle);
};
```

Represents either a renderable node in a mounted placement or a permanent TLAS
root. Normal application code obtains it from `InstanceHandle::rootNode()` or
a `FrontierEntry`. The packing helpers are exposed
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
    constexpr bool valid() const noexcept;
    constexpr NodeHandle rootNode() const noexcept;
    friend constexpr bool operator==(InstanceHandle, InstanceHandle);
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
class FrontierCutView {
public:
    class iterator; // forward iterator yielding const FrontierEntry&

    FrontierCutView();
    FrontierCutView(std::span<const FrontierEntry> first,
                    std::span<const FrontierEntry> second);
    iterator begin() const;
    iterator end() const;
    size_t size() const;
    bool empty() const;
};

struct FrontierResultView {
    std::span<const FrontierEntry> shared;
    std::span<const FrontierEntry> currentOnly;
    std::span<const FrontierEntry> idealOnly;

    FrontierCutView current() const;
    FrontierCutView ideal() const;
    size_t currentSize() const;
    size_t idealSize() const;
    size_t size() const;
    bool empty() const;
};
```

`current()` is a zero-allocation forward range that visits `shared` and then
`currentOnly`. `ideal()` visits `shared` and then `idealOnly`. Neither range
copies entries or makes the buckets contiguous; references point directly into
the source spans. The individual spans remain available for bulk operations
and delta-only processing.

`currentSize()` and `idealSize()` report the corresponding logical range sizes.
`size()` counts all three disjoint storage buckets. A view and any cut range or
iterator obtained from it remain valid until the underlying result storage is
invalidated. For a view returned by `SpatialQuery`, that means its next
selection, `reset()`, move assignment, or destruction.

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

`T` must be trivially copyable.

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
enum class CurrentCutPolicy : uint8_t {
    PreferReadyDescendants,
    PreferReadyAncestors,
};

struct SelectionParams {
    float threshold = 4.0f;
    float minPix = 0.0f;
    CurrentCutPolicy currentCutPolicy =
        CurrentCutPolicy::PreferReadyDescendants;
};
```

- `PreferReadyDescendants` uses a complete ready descendant cut below an
  unavailable ideal node when possible, and otherwise falls back to a ready
  ancestor. It normally produces the most detailed current cut and is the
  default.
- `PreferReadyAncestors` does not search below an unavailable ideal node. It
  falls back to a ready ancestor, normally producing fewer, coarser entries.
- `threshold` is the screen-error refinement threshold in pixels and must be
  positive and finite.
- `minPix` enables top-level contribution culling when greater than zero. An
  instance is rejected only when the conservative projected diameter of its
  world-space bounds is smaller than `minPix`; authored geometric error is not
  used for this decision. Zero disables it. It must be finite and non-negative.
- `currentCutPolicy` selects the replacement rule above. Changing it
  invalidates that query's reusable frontier records in O(1); allocations and
  damping state are retained. Reserved enumerator values are rejected.

```cpp
struct InstanceDesc {
    float4 pos{};
    float scale = 1.0f;
    YawRotation yaw{};
    uint32_t mask = ~0u;
};
```

Describes one TLAS placement. Position components must be finite, `scale` must
be finite, positive, and large enough that `1.0f / scale` remains finite, and
`yaw` must be a finite unit cosine/sine pair. `mask & Camera::viewMask == 0`
culls the instance at the top level.

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
the counters remain zero and each query omits their storage. CMake users enable
and propagate the matching ABI setting with `-DFRONTIER_STATS=ON`; manual
builds must define the macro consistently for the library and every consumer.

```cpp
struct CollectResult {
    size_t unmountedSubtrees = 0;
};
```

`unmountedSubtrees` counts placements removed by one collection pass.
Collection changes topology only and does not alter or report shared
definition-node readiness.

## 8. `SpatialQuery`

One mutable query stores damping, exact-cut reuse records, scratch/output,
optional instrumented statistics, and optional mount-usage feedback for one
logical view.

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
  non-positive and non-finite values disable damping.
- `reuseEnabled()` reports whether exact frontier-record reuse is requested.
  An all-flat TLAS snapshot still takes the faster direct path automatically
  because it has no hierarchy walk to cache.
- On the measured 10,000-root hierarchical workload, an admitted exact view is
  returned by the two-entry whole-cut memo in 10-68 ns. This lookup returns a
  view and does not iterate its 20,000 entries. A deliberately forced miss on
  every record is 44.5-51.5% slower than selecting with reuse disabled. Keep
  reuse for coherent views; disable it when the host knows no record can
  survive. The continuously moving 100,000-leaf city takes 18.254-69.866 us
  per complete payload64 database frame across the four measured machines.
  See the [current performance report](PERFORMANCE.md) for
  conditions.
- `setReuseEnabled()` resets damping/reuse/usage state when the value changes,
  while retaining allocations and the configured half-life.
- `reused()` and `walked()` report instance counts from the most recent
  selection.
- With `FRONTIER_STATS`, `lastSelectionStats()` returns a query-owned reference
  overwritten by the next selection or reset. Without instrumentation it
  returns a process-lifetime immutable zero value and allocates no query state.

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
  `applyUpdates(budget)`; `camera` is raw input and is damped internally; `params`
  controls refinement, contribution culling, and current-cut fallback policy.
  The final overload argument selects query-owned, caller-fixed, or
  caller-owned output.
- **Returns/results:** all overloads produce the same ordered three-bucket cut.
  The returned view uses query-owned storage. The sink overload reports
  truncation through its sinks. The owning overload replaces `outResult`'s
  contents.
- **Threading:** the function mutates the query. Do not select concurrently on
  one query. Distinct queries may read the same published database snapshot
  concurrently.
- **Binding:** the first selection binds the query to `database`. Selecting a
  different database before `reset()` is a contract violation.
- **Contract:** camera position, planes, projection scale, and envelope must
  be finite; projection scale must be positive and envelope extents
  non-negative. `params` must satisfy the contracts documented above.
- **Reuse:** cached entries reproduce the exact node set but may retain their
  earlier quantized magnitude within the proven reuse margin.
  With default masks and no contribution culling, a previously all-visible
  stream is retained when the conservative TLAS root remains wholly inside the
  frustum and the instance mapping epoch is unchanged; otherwise the exact
  TLAS query rebuilds it.
- **Recurring views:** the query-owned `FrontierResultView` overload admits up
  to two exact recurring camera/parameter keys after their second occurrence.
  With zero damping, reuse enabled, and mount-usage tracking disabled, later
  matches return the memoized complete cut directly. Mapping, spatial, and
  content generations guard every snapshot. Unique camera streams retain only
  two small candidate keys and never copy a cut; damping, usage tracking,
  caller sinks, and owning-result overloads use the normal path.

### Reset and storage

```cpp
void reset();
size_t bytes() const;
```

`reset()` releases database binding and clears damping history, reuse records,
pending usage, last counters, and current output while retaining allocations,
reuse mode, and configured half-life. Call it for camera cuts or teleports.
`bytes()` returns retained query/cache/scratch capacity in bytes, including
any recurring-view output snapshots.

### `TerminalRenderQuery`

```cpp
struct TerminalRenderRun {
    const UserPayload* payloads;
    uint32_t count;
    uint32_t instanceAndError;

    InstanceId instance() const;
    uint8_t errorCode() const;
    std::span<const UserPayload> payloadSpan() const;
};

class TerminalRenderView {
public:
    std::span<const TerminalRenderRun> runs() const;
    size_t size() const;
    size_t segmentCount() const;
    bool empty() const;
};

struct TerminalInstanceCluster {
    uint32_t first;
    uint32_t count;
};

struct TerminalInstanceBatch {
    SubtreeHandle definition;
    AABB localBounds;
    std::span<const float4> positions;
    std::span<const YawRotation> yaws;
    std::span<const TerminalInstanceCluster> clusters;
    std::span<const AABB> clusterBounds;
    InstanceId firstInstance;
    float scale;
    uint32_t mask;
    bool yawInvariantBounds;
    bool renderAsUnit;
};

class TerminalRenderQuery {
public:
    TerminalRenderView select(
        const SpatialDatabase& database,
        const Camera& camera,
        float errorThreshold = 4.0f,
        bool coarsenRenderUnits = true);
    TerminalRenderView select(
        const SpatialDatabase& database,
        const Camera& camera,
        std::span<const TerminalInstanceBatch> batches,
        float errorThreshold = 4.0f,
        bool coarsenRenderUnits = true);
    void reset();
    size_t bytes() const;
};
```

This is the max-detail output path for fully resident immutable scenes. It
selects every visible zero-error terminal leaf and represents consecutive
definition leaves as referenced payload ranges. Each run stores one stable
`InstanceId`, one terminal error code, a payload pointer, and a payload count.
The descriptor occupies 16 bytes on a 64-bit target. The renderer must still
iterate every logical payload unless it has a higher-level instancing scheme;
`size()` reports that logical leaf count, not the number of runs.

The contract is intentionally narrower than `SpatialQuery`:

- every selected mounted tree must be fully ready;
- definitions must contain no nested mount points and every terminal node must
  have zero geometric error;
- selected instances must not have copy-on-write bound deformation;
- the caller must publish changes with `applyUpdates(budget)` first.

Violations are contract errors; the query does not silently fall back to a
proxy because that would change max-detail semantics. TLAS-only roots remain
valid one-element runs. `errorThreshold` controls their threshold-relative
error encoding; terminal leaves always use code zero.

With `coarsenRenderUnits=true`, an instance opted into
`setInstanceRenderAsUnit()` is descendant-conservative: once its root
intersects the frustum, the whole terminal range is returned for renderer/GPU
clipping. Pass `false` for exact descendant culling. Payload pointers and the
run span remain valid only until the query's next `select()` or `reset()`, or
until the database is mutated. Queries are movable, not copyable, bind to the
first database they select, and may not be called concurrently; distinct
queries may read one published snapshot concurrently.

`TerminalInstanceBatch` is a non-owning homogeneous placement stream. Its
definition belongs to `database`, `positions` is one current world-space point
per actor, and `yaws` is either empty for identity orientation or has the same
length. `localBounds`, positive uniform `scale`, layer `mask`, yaw-invariant
bound policy, and renderer-coarsening policy are constant for the cohort.
Output ids are `firstInstance + actorIndex`; the exclusive end must not exceed
`kInvalidInstanceId`, and the caller must keep batch ranges disjoint from each
other and from normal instance ids in the same result.

`clusters` is optional. When present, its `{first,count}` records must be an
ordered, gap-free, nonempty partition of the position stream. Spatially
neighboring actors should be contiguous. Incoherent partitions remain correct
but can cost more than leaving `clusters` empty.

`clusterBounds` is independently optional and, when nonempty, must contain one
finite conservative AABB per cluster. Empty makes the query derive each exact
current union from authoritative transforms. Nonempty lets the caller provide
either a lifetime motion envelope or a current bound snapshot. Each AABB must
contain every current member root; contract builds recompute roots and verify
coverage, while Release trusts the same publication contract as other mutable
scene state. Outside bounds reject all members, inside bounds emit root ranges
directly, and partial bounds pass their narrowed plane mask to exact member
tests. Loose bounds reduce acceleration only; under-bounds can cull visible
actors and are a contract violation. Prefer immutable lifetime envelopes when
motion is constrained, exact self-derived unions when it is not, and per-frame
snapshots only when the simulation already produces them cheaply.

Batch spans need remain valid only during `select()`. The actors are not
inserted into the database, have no `InstanceHandle`, mount/readiness state, or
per-actor scale/mask, and are tested in O(batch actor count) per view. The
definition must satisfy the same no-nested-mount/zero-terminal-error contract,
and its render resources are caller-guaranteed resident. A database with no
TLAS roots may own definitions exclusively for batches. Static/general roots
still use the TLAS in the same call; fully inside batch roots append one range,
while partial roots use exact yawed local traversal. With
`coarsenRenderUnits=true`, `renderAsUnit` makes a partially intersecting actor
emit its whole terminal range.

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
    float tlasEditFraction = 0.05f;
    uint32_t parallelInstanceThreshold = 0;
};
```

- `context` supplies callbacks and is copied into the database.
- `tlasQuality` selects initial-build and explicit-`optimize()` quality;
  `refreshTlas()` always builds Morton topology.
- `tlasTraversalCost` and `tlasIntersectCost` are the Binned-SAH cost terms;
  increasing intersection cost favors deeper, tighter trees.
- `tlasCountDrift` is the population-change fraction at which
  `UpdateReport::topologyRebuildRecommended` becomes true.
- `tlasAreaDrift` is the allowed current TLAS lane area growth relative to the
  last TLAS build before a topology rebuild is recommended. Incremental
  maintenance reduces this ratio as it tightens lanes.
- `tlasEditFraction` is the accumulated incremental spawn/removal fraction at
  which a topology rebuild is recommended.
- `parallelInstanceThreshold` is the minimum visible-instance count for
  uncached parallel selection. Zero disables it; `context.workerCount` must
  also exceed one.

Construction rejects unknown quality values and non-finite or negative cost
and drift values. When `parallelInstanceThreshold` is non-zero and
`workerCount > 1`, `context.parallelFor` must be non-null. Internal parallel
selection is blocking and concatenates worker output in instance order, so
serial and parallel cuts are identical.

The three drift thresholds are advisory. They never cause an implicit topology
rebuild during publication; only the application decides when to call
`refreshTlas()` or `optimize()`.

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

Construction validates and copies the configuration and normalizes
`workerCount == 0` to one. Destruction releases registered definitions and all runtime storage. The
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
- **Effects:** validates the serialized format envelope and root range, then
  moves the allocation into the database. With
  `FRONTIER_VALIDATE_SUBTREES=1` (the default), it also validates topology,
  bounds, errors, and all wide traversal arrays. It does not unpack or copy
  serialized node arrays and does not allocate per-node runtime state. Shared
  readiness/coverage state is allocated lazily on first mount.
- **Complexity:** with `FRONTIER_VALIDATE_SUBTREES=1`, O(nodes + wide blocks)
  validation followed by constant-time ownership transfer. With it set to
  `0`, registration and ownership transfer are O(1).
- **Contract:** `bytes` is non-empty and valid. When structural validation is
  disabled, all internal arrays must be trusted and structurally valid;
  violating this precondition can cause undefined behavior during later
  traversal. Contract failure leaves the input's post-failure state
  unspecified.
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
  `desc` supplies world translation, uniform scale, planar yaw, and view mask.
- **Returns:** a generation-stamped instance handle whose `rootNode()` is live
  immediately.
- **Effects:** inserts one TLAS leaf. A non-mountable one-node instance needs no
  subtree definition or mounted-placement state.
- **Contract:** root error is finite/non-negative, reserved root flag bits are
  zero, root bounds are finite and non-empty on all axes, and instance position
  and scale are finite with positive scale and finite reciprocal. The
  transformed root bound and error must remain representable as finite floats.
  If `FlagYawInvariantBounds` is set, the application guarantees that the
  authored root bound contains all content at every possible submitted yaw.
- **Readiness:** the live TLAS root is always ready. Its payload value does not
  affect the readiness of mounted definition nodes with the same value.

```cpp
void removeInstance(InstanceHandle instance);
```

- **Effects:** removes the TLAS root, recursively unmounts everything beneath
  it, releases its bounds overlays, and invalidates all related instance and
  node handles. Registered definitions remain registered.
- **Stale behavior:** no-op.

```cpp
void moveInstance(InstanceHandle instance,
                  const InstanceTransform& transform);
void moveInstance(InstanceHandle instance, const Transform& transform);
```

- **Parameters:** new world translation, scale, and optional planar yaw for the
  whole instance. The `Transform` overload is the identity-yaw convenience
  path.
- **Effects:** moves the TLAS root and every mounted descendant as one object;
  translation and angular displacement consume affected query records' exact
  distance margins, while scale changes invalidate those records. Public
  handles remain valid. Exact instance state changes during submission; TLAS
  maintenance is coalesced and becomes queryable after `applyUpdates(budget)`.
- **Stale behavior:** no-op.
- **Contract:** position and scale are finite, with positive scale and finite
  reciprocal. The transformed root bound and error must remain representable
  as finite floats.

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
the corresponding physical database order. A database mapping epoch validates
the cohort once per call; the cache refreshes automatically after instance
addition, removal, slot reuse, `optimize()`, or another layout change.

The cached order is intended for cohorts updated repeatedly. It keeps dense
instance and motion-odometer writes sequential and improves reuse of nearby TLAS paths,
especially after instances have been spatially reordered. It also avoids a
fresh public-handle-to-dense lookup for every member on every update. The first
update after construction or `reset()`, and any update after a layout change,
resolves and sorts the live cohort; retain the group across frames to amortize
that work.

- The span constructor is equivalent to default construction plus `reset()`.
- `reset()` replaces the copied handle sequence and invalidates the physical
  order cache.
- `size()` returns the number of caller-order handles, including stale or
  duplicate values.

```cpp
void moveInstances(MotionGroup& group,
                   std::span<const float4> positions,
                   float scale = 1.0f);

void moveInstances(MotionGroup& group,
                   std::span<const InstanceTransform> transforms);
```

- **Parameters:** `positions[i]` belongs to the handle copied at group index
  `i`; one uniform scale applies to all live members.
- **Effects:** moves live instances in cached physical order. Stale handles are
  ignored. If the group contains the same live instance more than once, the
  final caller-order position wins. TLAS writes are deferred to the publication
  barrier: small cohorts grow conservative swept paths, while a cohort covering
  at least one quarter of the TLAS leaves triggers one exact bottom-up refit.
- **Reuse accounting:** the largest L1 translation in the effective batch is
  added once to the database-wide conservative motion path. This lets a query
  validate a retained complete cut in O(1); individual per-instance odometers
  remain the exact fallback when that global proof expires.
- **Contract:** position and group sizes match, scale is finite and positive
  with finite reciprocal, and every position belonging to a live instance is
  finite. Each transformed root bound and error must remain representable as
  finite floats.

The `InstanceTransform` overload supplies an independent position, scale, and
yaw for each group member. Angular motion is charged conservatively by the
maximum possible displacement of any point in the authored root bound; this
keeps cached LOD decisions exact without invalidating every rotating actor.

### `RigidMotionGroup`

```cpp
class SpatialDatabase::RigidMotionGroup {
public:
    RigidMotionGroup();
    explicit RigidMotionGroup(
        std::span<const InstanceHandle> instances);
    void reset(std::span<const InstanceHandle> instances);
    size_t size() const;
};

void moveRigidInstances(RigidMotionGroup& group,
                        std::span<const float4> positions,
                        std::span<const YawRotation> yaws);
```

`RigidMotionGroup` is the structure-of-arrays path for a persistent cohort that
changes translation and planar yaw while retaining each instance's existing
scale. `positions[i]` and `yaws[i]` correspond to the handle copied at group
index `i`; both spans must exactly match the group size. Positions and unit
cosine/sine pairs must be finite, and translated root bounds must remain
representable.

When every live member was authored with `FlagYawInvariantBounds`, Frontier
proves that property once per physical instance mapping and uses a streaming
rigid kernel. It translates the already exact world AABB, updates the cold yaw
stream and motion odometer, and queues the dense id without re-entering the
general scale/oriented-bounds path. A group containing any ordinary oriented
bound falls back to the general exact transform implementation. Stale handles
are ignored and duplicates retain the last caller entry, matching
`MotionGroup`.

Keep the group and its SoA arrays alive across frames. Use this path for
traffic, pedestrians, rigid particles, and similar stable actor populations.
Use `moveInstances(MotionGroup&, span<InstanceTransform>)` when scale changes
or when AoS is the application's authoritative layout.

```cpp
void translateInstances(MotionGroup& group, float4 delta);
```

- **Parameters:** one finite world-space translation applied to every live,
  unique group member.
- **Effects:** translates the cohort without changing scale. A group covering
  the complete live population updates a deferred global offset in O(1).
  Subsets update exact instance state in cached physical order and retain
  conservative swept TLAS leaf envelopes across bounded repeated motion.
- **Exactness:** a query that cannot prove a loose leaf wholly visible retests
  that instance's current bound for frustum and contribution culling.
- **Reuse accounting:** the L1 delta is charged once to the whole-cut motion
  odometer and to each affected fallback record (or one shared uniform
  odometer for a complete population).
- **Contract:** the translated cohort bounds, positions, and motion odometers
  must remain finite and representable.

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
- **Effects:** creates a compact placement record while sharing definition
  bytes and the definition's readiness/coverage state. The mount transform is
  accumulated into top-level instance-local space. Child effective errors are
  capped by the parent's effective error without rewriting the definition. A
  placement receiving its first mounted descendant takes a private copy of its
  coverage state because completeness can then differ between placements.
- **Complexity:** the first placement of a definition allocates and zeroes one
  shared 16-bit-per-packed-node state block, linear in that definition's node
  count. Later childless placements attach to it in constant time. Mounting the
  first nested child in an owner copies the owner's state and allocates its
  per-node mount-link array, linear in the owner definition's node count; later
  links are constant-time. Private coverage blocks come from a
  definition-sized slab pool.
- **Contract:** the definition is live; transform is finite with positive
  scale; its accumulated transform remains finite with finite reciprocal; a
  transform mounted directly below a TLAS root also has finite reciprocal;
  live parent is mountable and has no existing mounted child; the transformed
  aggregate definition bounds fit in the parent's containment bound. This is
  the shared authored bound for a mounted node and the current instance-local
  root bound for a TLAS root.

The mount transform is immutable. To reposition a placement, unmount and mount
it again; the replacement receives new handles, shares the definition's current
readiness/coverage state, and takes private coverage only if it later receives
mounted descendants.

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

### Definition-node readiness and payload lookup

```cpp
void markNodeReady(NodeHandle node);
```

- **Parameters:** a live mounted node whose complete GPU resource set is now
  available for dispatch, or a live TLAS root.
- **Effects:** marks the corresponding registered definition node ready and
  incrementally updates coverage in every live placement of that definition.
  Future placements inherit the state. A live TLAS root is already ready, so
  the call is a no-op.
- **Repeat/stale behavior:** no-op when already ready or when the handle is
  stale or invalid.

```cpp
void markNodeUnavailable(NodeHandle node);
```

- **Parameters:** a mounted node that can no longer be dispatched.
- **Effects:** marks the corresponding registered definition node unavailable
  and propagates coverage changes through every live placement of that
  definition. Equal payload values in other definition nodes are unaffected.
- **Repeat/stale behavior:** no-op when already unavailable or when the handle
  is stale or invalid.
- **Contract:** a live TLAS root may not be made unavailable because it is the
  permanent renderable fallback.

```cpp
bool isNodeReady(NodeHandle node) const;
```

Returns `true` for a live TLAS root or a mounted node whose registered
definition-node bit is ready. Returns `false` for unavailable, stale, or invalid
mounted handles. Readiness survives unmounting and is inherited by later
placements of the same registered definition; releasing the definition
discards it. A readiness change cannot be published before the definition has
at least one live placement because the public identifier is a `NodeHandle`.

```cpp
UserPayload tryGetPayload(NodeHandle node) const;
```

Returns the immutable node payload when the handle resolves and
`kInvalidPayload` for stale or invalid input. The value is reserved by the
authoring contract, rejected by checked builders and validated registration,
and is therefore unambiguous. An entry produced by a selection resolves
throughout the same published read interval; failure is relevant when the
application retains a handle across a later writer phase.

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
- **Contract:** bounds are finite and non-empty, remain finite after every
  containing mount and top-level instance transform, and the live node belongs
  to the supplied live instance. Transform overflow is detected when the
  deferred edit is flushed.

```cpp
void flushBounds();
```

Applies queued bounds edits immediately in submission order. The edited node
is set exactly; ancestor propagation is grow-only. This is normally implicit
in `applyUpdates(maintenanceNodeBudget)` and is exposed for tools or immediate
readback.

```cpp
AABB nodeBounds(InstanceHandle instance, NodeHandle node);
```

Flushes pending edits, then returns the effective instance-specific local
bound: an overlay if one exists, otherwise the shared authored bound. Returns
`AABB::empty()` when the instance or node does not resolve. For live inputs,
the node must belong to the supplied instance.

`setNodeBounds()` changes spatial coverage only; it does not store a node pose
or render transform.

### Publication and maintenance

```cpp
inline constexpr uint32_t kUnlimitedTlasMaintenance = UINT32_MAX;

struct UpdateReport {
    uint32_t maintenanceNodesProcessed = 0;
    uint32_t maintenanceNodesPending = 0;
    float areaGrowthRatio = 0.0f;
    bool topologyRebuildRecommended = false;
    bool requiredBuildPerformed = false;
};

UpdateReport applyUpdates(uint32_t maintenanceNodeBudget);
```

Flushes node bounds, coalesces pending instance motion, repairs at most the
specified number of queued TLAS nodes, publishes the state for a group of
read-only selections, and increments `frame()`. Every interruption point is
safe: unprocessed nodes retain conservative grown bounds, which can reduce
culling efficiency but cannot hide a visible instance.

- Pass `0` to pay no optional tightening cost in this update.
- Pass a finite node count to spread repair work predictably across updates.
- Pass `kUnlimitedTlasMaintenance` to drain all currently queued repair work.

`maintenanceNodesProcessed` is the exact work charged to this call and
`maintenanceNodesPending` is the queue size after it. `areaGrowthRatio`
measures current stored lane area above the last TLAS build.
`topologyRebuildRecommended` applies the configured population, edit, and area
drift thresholds; it is advice, not a scheduled action.
`requiredBuildPerformed` is true only for a correctness-required initial or
defensive build. Optional maintenance never rebuilds topology.

Call once before each selection group, even when the writer made no scene
edits, if collection age should advance. No mutation or collection may overlap
selections using the published snapshot.

```cpp
void refreshTlas();
void optimize();
```

Both methods consume pending bounds and instance motion and establish a fresh
population/area drift baseline without advancing `frame()` or collection age.
`refreshTlas()` uses the linear Morton builder, clears incremental maintenance,
and preserves dense slots, physical instance order, layout version, and
mapping version. `optimize()` uses `SpatialDatabaseConfig::tlasQuality`,
compacts dead dense slots, and restores physical query-record order to TLAS
traversal order. Public `InstanceHandle` values, root `NodeHandle` values, and
`FrontierEntry::instance()` ids remain stable in either case.

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
- **Returns:** the number of removed placements.
- **Effects:** walks the LRU tail and removes eligible cold placements until
  the budget is reached or no candidate remains. Only placements with no
  mounted children and sufficient age are eligible. Collection never changes
  definition-node readiness; resource eviction is separate application policy.
- **Binding contract:** every non-null query is unbound or bound to this
  database; feedback from another database is a contract violation.
- **Threading:** collection is a writer operation and must not overlap
  selection.

The no-query overload uses the database's existing LRU timestamps. A query
affects retention only after `setMountUsageEnabled(true)`.

### Introspection

```cpp
size_t mountedSubtreeCount() const;
uint32_t frame() const;
size_t overlayCount() const;
size_t overlayBytes() const;
size_t subtreeInstanceStateBytes() const;
size_t instanceOrientationStateBytes() const;
```

- `mountedSubtreeCount()` returns the number of live mounted placements.
- `frame()` returns the latest `applyUpdates(budget)` epoch.
- `overlayCount()` returns live copy-on-write bounds overlay count.
- `overlayBytes()` returns retained overlay storage bytes.
- `subtreeInstanceStateBytes()` returns retained placement records,
  transforms, shared coverage/readiness words, private coverage copies, stamps,
  slab capacity, definition-to-mount references, and mount-link capacity;
  immutable registered `SubtreeBytes` are excluded.
- `instanceOrientationStateBytes()` returns the optional cold top-level yaw
  and authored-local-bounds stream. It is zero until non-identity yaw is used.

With `FRONTIER_DEBUG_TOOLS` enabled, the following on-demand inspection API is
also available:

```cpp
QueryCacheDebugSummary SpatialQuery::debugCacheSummary() const;

TlasDebugSummary SpatialDatabase::debugTlasSummary() const;
size_t SpatialDatabase::debugTlasBoxes(
    uint32_t depth, std::span<TlasDebugBox> output) const;
size_t SpatialDatabase::debugLooseInstanceBounds(
    std::span<LooseInstanceDebugBounds> output) const;
```

The summaries read existing scalar state. `debugTlasSummary()` reports the
same pending-maintenance, current area growth, required-build, and topology-
rebuild recommendation state as publication. It includes the active and
configured quality tiers plus the population at the last rebuild, and walks the live
TLAS to calculate depth and occupancy. The span methods allocate nothing,
write at most `output.size()` records, and return the complete matching count
so the caller can detect truncation. `debugTlasBoxes()` returns a complete cut
at the requested depth: internal lanes at that level and any terminal instance
lanes encountered earlier. Each record retains its actual depth. TLAS boxes and
exact/envelope bounds are returned in world space, including deferred
population translation. Call these methods only for a published database
snapshot; their work is performed only when explicitly requested.

## 11. Threading contract

`SpatialDatabase` is single-writer with published concurrent reads:

1. One writer performs registration, assembly, movement, readiness changes,
   bounds, maintenance, or collection.
2. The writer calls `applyUpdates(maintenanceNodeBudget)`.
3. Any number of readers call `selectFrontier()` concurrently using distinct
   `SpatialQuery` and output objects.
4. All readers join before the next database mutation.

One `SpatialQuery` is never concurrently callable because selection mutates
its damping, reuse, usage, optional statistics, scratch, and output. Const database
lookup helpers are intended for the same stable read interval unless the
application otherwise serializes them with mutation.

The host `parallelFor` callback creates parallel work inside one uncached
selection. It must be blocking. Cached selection remains serial because its
reuse-record updates are query-local and ordered.

## 12. Current representation limits

- One builder node has at most 511 local children; a definition has at most
  511 direct nodes beneath its runtime mount parent and at most 1,048,575
  renderable nodes in total.
- Mounted placement slots and definition-local node indices use 20 bits.
- Mounted-node generations use 24 bits.
- TLAS root generations use 20 bits.
- Public instance ids use 24 bits, with the all-ones value reserved.
- `FrontierEntry` instance ids and error codes are packed into one 32-bit word.
- Top-level runtime transforms support translation, positive uniform scale,
  and planar yaw. Mounted-subtree transforms support translation and scale.

These are representational limits, not suggested operating budgets. Use the
database memory and count introspection methods to set application-specific
budgets.
