# Current codebase

This document is a compact map of the checked-in Frontier implementation. It
describes the code and contracts that exist now. Public API details are in
[API_REFERENCE.md](API_REFERENCE.md); design history is intentionally kept out
of this document.

## Scope

Frontier is a C++20 scene-selection library. It owns spatial hierarchy state,
streaming readiness, instance placement, visibility selection, and compact
CPU-facing render cuts. It does not own renderer resources, issue graphics API
commands, schedule asynchronous loading, or decide application content keys.

The core library is built as ordinary C++ source with no runtime dependency on
the benchmark framework. SIMD width, payload type, validation, contracts, and
instrumentation are compile-time choices.

## Repository map

| Area | Current responsibility |
|---|---|
| `include/frontier/node.h` | Node descriptors, payload/error representation, flags, transforms |
| `include/frontier/subtree.h` | Owning serialized `SubtreeBytes` allocation |
| `include/frontier/builder.h` | Immutable subtree authoring and serialization |
| `include/frontier/spatial_database.h` | Public database, handles, queries, motion, terminal render APIs |
| `include/frontier/config.h` | Allocator, fatal-handler, parallel-for, and build context |
| `include/frontier/math.h` | Camera, bounds, vectors, matrices, and projection math |
| `src/builder.cpp` | Serialized definition construction |
| `src/subtree.cpp` | Definition validation and serialized-layout handling |
| `src/spatial_database.cpp` | TLAS, instances, mounts, readiness, publication, general selection |
| `src/terminal_render.cpp` | Fully resident terminal-range selection and actor batches |
| `src/rigid_motion.cpp` | Dense rigid-cohort transform publication |
| `src/config.cpp` | Default runtime context and configuration support |
| `tests/` | Deterministic unit, contract, streaming, motion, TLAS, and torture coverage |
| `bench/` | End-to-end scene workloads and machine/kernel characterization |

## Public object model

- `SubtreeBuilder` authors an immutable local hierarchy and produces one
  aligned `SubtreeBytes` allocation.
- `SpatialDatabase::registerSubtree()` consumes that allocation without
  unpacking it and returns a generation-stamped `SubtreeHandle`.
- `SpatialDatabase::instantiate()` creates one permanent renderable TLAS root
  and returns an `InstanceHandle`.
- `mountSubtree()` places a registered definition below a mountable renderable
  node and returns a `SubtreeInstanceHandle`.
- `NodeHandle` identifies one live renderable node discovered through a cut.
- `SpatialQuery` owns all mutable general-selection state.
- `TerminalRenderQuery` owns the strict fully resident max-detail path and
  returns ordered `TerminalRenderRun` ranges.

Payload equality has no semantic meaning inside Frontier. Readiness belongs to
a specific registered definition node; placement and handle identity remain
independent of `UserPayload` values.

## Runtime spatial structure

The database has two spatial levels:

1. A dynamic world-space TLAS contains every top-level renderable instance.
   Its width is the build-wide `FRONTIER_BVH_WIDTH` value: BVH4 or BVH8.
2. Registered definitions provide immutable local hierarchy fragments below
   mount points. A definition may itself contain mountable leaves, so mounted
   trees can be nested.

Every TLAS leaf is a valid renderable fallback. A mounted definition adds
detail below that fallback; it never replaces the root's identity or readiness
contract.

Hot TLAS nodes hold wide bounds, child references, flags, and parent indices.
Maximum error and layer masks are parallel cold metadata. BVH4 uses 128 hot +
32 cold bytes per node; BVH8 uses 256 hot + 64 cold bytes.

## Serialized definitions

`SubtreeBytes` is both the serialization format and the traversal format. One
64-byte-aligned allocation contains:

- a 128-byte versioned header;
- canonical wide bound blocks;
- lane masks;
- payload, parent, subtree-size, metadata, and error arrays;
- the aggregate definition bound.

Per-node scalar bounds are not duplicated. A node's canonical bound is the
lane stored in its parent's wide block. Registration normally validates the
complete structure and then moves the allocation into the database. Trusted
builds may compile out the structural scan with
`FRONTIER_VALIDATE_SUBTREES=OFF`.

Definition-node readiness is shared by every placement of that registered
definition. Placement-specific coverage is allocated only when mounted
children make the shared summary insufficient.

## Placement and instance data

Top-level instances use dense structure-of-arrays streams for hot query and
publication state while stable public handles map to dense indices. Each
instance retains its exact transform, exact world bound, renderable root data,
mask, mounted-root slot, generation, TLAS back-pointer, and dense-list index.

Mounted placement state is split by access frequency:

| Record | Size | Purpose |
|---|---:|---|
| `MountTransformRt` | 32 B | translation/scale, error clamp, generation, definition, root flags |
| `MountStamp` | 8 B | content version, generation, liveness |
| `MountReadiness` | 4 B | fully-ready summary and incomplete-child count |
| `SubtreeInstanceRt` | 56 B | ownership, definition, LRU, mount links, state pointers |
| Node state | 2 B/node | coverage bit and covered-child count |

Mount-link arrays and private coverage blocks are lazy. Childless placements
of the same definition share its readiness/coverage storage.

## Writer and reader lifecycle

The database uses an explicit publish boundary:

1. The single writer performs instance, mount, readiness, or bound mutations.
2. `applyUpdates()` flushes copy-on-write bounds, publishes motion, maintains
   the TLAS, and advances the snapshot epoch.
3. Any number of readers select concurrently, each with a distinct query
   object.
4. The application consumes or submits the returned views.
5. All reads finish before the next writer phase or collection pass.

Returned views reference query-owned storage and remain valid until that query
is reset, destroyed, or used for another selection.

## Selection paths

| Path | Use it for | Output and restrictions |
|---|---|---|
| `SpatialQuery` | General LOD, streaming readiness, nested mounts, nonzero error, deformed bounds | Current/ideal frontier buckets containing `FrontierEntry` handles |
| `TerminalRenderQuery` | Fully resident terminal leaves with zero terminal error | Ordered `TerminalRenderRun` payload ranges; no general streaming/LOD semantics |
| `TerminalInstanceBatch` | Large homogeneous moving cohorts | Caller-owned position/yaw streams plus one definition, constant bounds/scale/mask, consecutive ids |

General selection traverses the TLAS first. Flat roots emit through a direct
path. Hierarchical roots either stop at the renderable root or enter a mounted
definition. Wide bound/error tests visit four or eight children at once, and a
compact DFS stack holds surviving hierarchy work.

Each `SpatialQuery` owns damping, two exact-view memo slots, frontier reuse
records, dependency spills, traversal scratch, output slabs, optional
statistics, and optional mount-use feedback. Fully inside instances can reuse
recorded cuts while their travel budget, threshold epoch, instance version,
and mounted-tree content stamps remain valid.

The terminal path precomputes immutable definition plans containing decoded
payloads and a `{begin,count}` range for every definition node. Fully accepted
branches append one 16-byte run descriptor instead of one handle record per
leaf. Boundary branches continue through exact wide traversal.

## Motion and TLAS publication

Small motion cohorts use conservative grow-only TLAS propagation. When the
pending cohort reaches one quarter of the TLAS population, `applyUpdates()`
streams the retained TLAS postorder once, copies exact dense leaf state, and
recomputes interior bounds sequentially.

`RigidMotionGroup` caches a caller-to-dense mapping for stable top-level
cohorts whose authored bounds are yaw-invariant. Separate contiguous position
and yaw streams update exact world boxes without the generic transform path.
Scale changes and ordinary oriented bounds use the general instance API.

`TerminalInstanceBatch` keeps homogeneous dynamic actors outside the general
instance/TLAS population. Optional contiguous clusters provide a two-level
broadphase. Cluster bounds may be reduced from current member transforms or
supplied as conservative published envelopes. Supplied bounds must contain
every represented actor transform for the entire query snapshot.

## Readiness and current cuts

The ideal cut assumes every known node is renderable. The current cut is the
hole-free renderable cover available now.

- `PreferReadyDescendants` uses a complete visible descendant cover when an
  ideal node is unavailable.
- `PreferReadyAncestors` emits the nearest ready ancestor fallback instead.

Readiness changes update every placement of the affected registered
definition node through its intrusive placement list. They do not scan
unrelated definitions or infer relationships from equal payload values.

## Bounds and overlays

Authored bounds remain immutable. `setNodeBounds()` creates a placement-local
copy-on-write overlay only for touched wide blocks. Small definitions use dense
overlay blocks; large definitions begin sparse and promote when the configured
edit fraction is reached. Queued edits retain caller order, and ancestor
propagation stops when an existing bound already contains the change.

## Build configuration

| Setting | Current role |
|---|---|
| `FRONTIER_USER_PAYLOAD` / `FRONTIER_INVALID_PAYLOAD` | Build-wide payload type and reserved invalid value |
| `FRONTIER_BVH_WIDTH` | `AUTO`, `4`, or `8`; changes SIMD and serialized layout |
| `FRONTIER_AVX2` | Enables the x86 BVH8 AVX2/FMA backend; no runtime dispatch |
| `FRONTIER_FORCE_SCALAR` | Selects the scalar backend |
| `FRONTIER_CONTRACT_CHECKS` | Caller-precondition validation |
| `FRONTIER_VALIDATE_SUBTREES` | Complete serialized-definition validation |
| `FRONTIER_STATS` | Compile-time query instrumentation |
| `FRONTIER_DEBUG_TOOLS` | Opt-in read-only TLAS/query-cache inspection API; no hot-path instrumentation |
| `FRONTIER_IPO` | Interprocedural optimization for Frontier targets |
| `FRONTIER_PGO_MODE` / `FRONTIER_PGO_DIR` | Optional GCC profile generation/use |

Payload type and BVH width are ABI and serialized-format choices. Every linked
translation unit must use matching values. AVX2 builds require a compatible
CPU or application-level binary dispatch.

## Correctness and measured capacity

The current Debug matrix contains 452 tests across payload32/payload64 and
BVH4/BVH8. It covers serialization, contracts, current/ideal cuts, streaming,
mounting, cache validity, motion, TLAS maintenance, parallel determinism,
randomized churn, and concurrent snapshot reads.

The current four-device Release measurement reports a complete moving-city
database frame at:

| Device | Payload64 motion + publication + exact selection |
|---|---:|
| M2 Max | 18.254 us |
| EPYC 9654 | 23.144 us |
| i9-12900K | 38.143 us |
| Cortex-A72 SBC | 69.866 us |

The workload has 100,000 logical leaves, 1,191 TLAS roots, 1,100 moving actor
roots, and a continuously changing 40 mph camera. Complete measurement details
and limitations are in [PERFORMANCE.md](PERFORMANCE.md).

## Current constraints

- `SpatialDatabase` is single-writer; the host provides phase synchronization
  around concurrent readers.
- General query views and terminal views are query-owned and non-owning.
- Mounted placement transforms support translation and uniform scale. Planar
  yaw belongs to top-level instances and terminal actor batches.
- The terminal render path requires fully resident terminal leaves, zero
  terminal error, and the strict supported topology documented by the API.
- Terminal actor batches require homogeneous definitions and spatially ordered
  caller-owned streams with snapshot-stable lifetimes.
- Conservative batch envelopes trade culling tightness for avoiding per-frame
  bound reduction; under-bounds violate correctness.
- BVH width, payload type, and ISA selection are build-wide. The library does
  not perform runtime ISA or serialized-width dispatch.
- Performance profiles disable checks and accept only trusted inputs.
- The benchmark measures CPU database and payload-stream work, not graphics
  driver submission or GPU execution.
