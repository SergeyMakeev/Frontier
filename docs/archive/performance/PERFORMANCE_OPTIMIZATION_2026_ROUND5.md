# Live-city continuous-motion benchmark

> Archived engineering journal. Revisions, APIs, measurements, and conclusions
> in this file are historical and are not current product documentation.

## Goal

The existing moving-camera benchmark intentionally alternates two exact poses
and measures the query's two-entry whole-cut memo. It does not represent a
camera that advances to a new pose every frame. This round adds one realistic,
deterministic city-frame workload without removing that useful recurring-view
microbenchmark.

The timed scope is caller-side actor rigid-transform staging, two batched
spatial motion submissions, publication, and `selectFrontier()`. Immutable
scene and camera construction stay outside the timed region. The result is an
end-to-end spatial-database frame cost, not a renderer or complete game-engine
frame.

## Workload and data layout

The scene contains exactly 100,000 logical leaf instances:

| Population | Placements | Leaves per placement | Logical leaves |
|---|---:|---:|---:|
| Independent static city blocks | 83 | 1,024 | 84,992 |
| Static TLAS singletons | 8 | 1 | 8 |
| Cars | 100 | 50 | 5,000 |
| Pedestrians | 1,000 | 10 | 10,000 |
| **Total** | **1,191 TLAS roots** | | **100,000** |

Each static block owns an independent quadtree-like definition with one root
and five four-way levels. Leaves therefore sit at authored depth five; across
83 blocks the immutable working set is not collapsed into one cache-resident
shared definition. Cars share one 50-leaf definition and pedestrians share one
10-leaf definition, as repeated model assets normally would. Moving an actor
updates its TLAS placement once; every mounted detail leaf inherits that root
translation without 50 or 10 separate database mutations.

The camera and cars travel at 40 mph. Pedestrians travel at 1.5 mph. All paths
advance at exact 1/60-second spatial increments on deterministic circular
tracks. Every actor's local +Z axis follows the track tangent; reverse traffic
faces the opposite tangent. The 4,096-frame camera table has no identical
adjacent pose. Each benchmark repetition is fixed at 8,192 frames, covering
two complete tables and 136.533 seconds of simulated time. Scene spatial
versions change every frame, so recurring-view snapshot admission cannot
answer this workload.

## Experiment 1: shared static block prototype

**Theory.** First validate the population counts, trajectory, motion APIs, and
selection counters using one depth-five static block definition mounted 83
times.

**Result.** Structurally correct but rejected as the final model. The first
payload64 smoke median was approximately 101 us, with about 24,775 visible
entries on its final sampled frame and 93.1% root reuse. Reusing one roughly
block-sized immutable definition for the entire static city gives the
hierarchy unrealistically favorable cache locality. The final workload instead
registers 83 independent immutable blocks. The initial final-frame entry
counter was also replaced by trajectory mean/minimum/maximum counters.

## Experiment 2: independent blocks and fixed complete trajectories

**Status: retained.**

The final benchmark owns 9,881.06 KiB of immutable data with the 64-bit payload
and 9,439.88 KiB with the 32-bit payload. Mounted-placement state is 384.918
KiB and the warmed query owns 1,223.83 KiB. A full trajectory averages
24,072.7 returned leaf entries, with a 16,459 minimum and 28,077 maximum.

Across visible/relevant TLAS roots, an average frame reuses 272.656 and walks
20.431, for 93.029% record reuse. Unlike the exact recurring-view benchmark,
this work is record-level validation and selective traversal under a changing
camera and changing scene.

Local MSVC Release, AVX2/BVH8, IPO; nine sequential repetitions using median
real time:

| Payload | Median per frame | CV | 60 Hz frame budget |
|---|---:|---:|---:|
| 64-bit | 94.5 us | 0.36% | 0.57% |
| 32-bit | 94.1 us | 0.38% | 0.56% |

These numbers validate stability and semantics on the development host; they
are not substituted for the controlled AArch64 result. The next complete SBC
bundle will establish the target baseline under CPU affinity, warmup, frequency,
and thermal telemetry.

Raw local results: `live-city-sequential-payload64.json` and
`live-city-sequential-payload32.json`.

## Experiment 3: tangent-aligned actor yaw

**Problem.** The first benchmark moved roots along circular tracks but left all
car and pedestrian detail frames axis-aligned. That understated a real city's
transform work and made curved traffic visually implausible.

**Theory.** Rotate each repeated actor once at its TLAS placement rather than
submitting rotation or bounds for its 50 or 10 leaves. Keep the existing
80-byte hot `Instance` record unchanged so translation-only databases pay no
working-set penalty. Store rotation-specific state in a parallel stream that
is allocated only after the first non-identity yaw.

**Implementation retained.** `InstanceTransform` and `InstanceDesc` now carry
a unit `(cos(yaw), sin(yaw))` pair while remaining 32 bytes. The pair avoids
`atan2`/`sin`/`cos` in the 60 Hz update path because trajectory and animation
systems normally already own a forward vector. A cold 36-byte record stores:

- the exact authored local root AABB (24 bytes);
- the current yaw cosine/sine (8 bytes); and
- the root's maximum XZ radius (4 bytes).

The local AABB is necessary: an axis-aligned world enclosure of a rotated box
cannot be inverted to recover its authored box. Rotation-aware center/extent
math produces the exact world AABB without transforming eight corners. A
matching inverse camera transform rotates the camera position and active
frustum planes into instance space. A damped camera's rotated envelope becomes
an oriented box, so traversal uses its conservative local AABB enclosure.

Cached cuts are not blindly invalidated on rotation. For a yaw pair change
`(dc, ds)`, the database charges
`scale * radiusXZ * (abs(dc) + abs(ds))` to the instance motion odometer. This
conservatively bounds the world displacement of every point under the angular
step, allowing the existing exact decision margin to reuse a cut only while it
remains safe. Scale changes still invalidate because they change the error
field itself.

Rotation-specific correctness tests cover exact AABB/camera transforms,
deformation followed by rotation, independent batched actor yaw, lazy cold
stream allocation, invalid unit pairs, and 256 frames of
cached-versus-uncached angular selection.

**Canonical exact-bound result.** The untouched parent binaries were preserved
long enough for a same-session Release/AVX2/BVH8/IPO comparison, with contract
and subtree validation disabled in both cases. Axis-aligned medians were 95.3
us (payload64) and 94.8 us (payload32). Exact rotated-AABB medians were 112-113
us for both payload widths, roughly an 18% realism cost. Reuse changed from
93.029% (272.656 reused, 20.431 walked roots per frame) to 92.425% (271.733
reused, 22.273 walked). This exact-bound implementation remains available for
arbitrary authored roots, but it was not the final benchmark policy.

## Experiment 4: authored yaw-swept root envelopes

**Theory.** Production vehicle and crowd broadphases often use a conservative
sphere/cylinder-like root that already contains the model at every heading.
When content supplies that guarantee, changing yaw does not require a new TLAS
AABB: translate the existing envelope, rotate only the camera entering mounted
detail, and continue charging angular travel to cached-cut margins.

**Implementation retained.** A TLAS-root-only
`NodeDesc::FlagYawInvariantBounds` records the authored guarantee. It consumes
one formerly reserved descriptor bit and no hot runtime word; the sign of the
cold radius encodes the policy. `SubtreeBuilder` rejects the flag because mount
transforms do not support yaw. The live-city car and pedestrian roots already
have enough slack to contain every rotated detail leaf, so they set the flag.

The final low-variance nine-repetition canonical medians are 98.7 us for
payload64 (0.41% CV) and 98.6 us for payload32 (0.54% CV). Against exact
rotated bounds, this is about 12% faster. Against the same-session axis-aligned
parent medians of 95.3/94.8 us, realistic tangent yaw now costs only about
3.6%/4.0%. Reuse returns to 93.029% (272.656 reused and 20.431 walked roots per
frame). The rotated detail trajectory returns 24,072.7 entries on average, with
a 16,455 minimum and 28,076 maximum.

Raw retained results are
`perf_reports/optimization-round5/live-city-yaw-envelope-payload64.json` and
`perf_reports/optimization-round5/live-city-yaw-envelope-payload32.json`.

The final correctness matrix adds the invariant-envelope contract case and
contains 404 passing cases across BVH4/BVH8 and 32/64-bit payload layouts.

## Interpretation and tradeoffs

- Top-level instance yaw is planar (rotation around +Y), not a full quaternion.
  It directly covers ground vehicles and pedestrians while avoiding a larger
  hot placement format. Roll, pitch, and rotating mounted-subtree transforms
  are not represented.
- Car and pedestrian paths are deterministic circular traffic, not AI or
  collision simulation. Filling the 1,100 complete rigid transforms is timed;
  gameplay decision-making is not.
- A database that never uses yaw allocates no orientation stream and retains
  the old identity transform/bounds path. Once yaw is used, the stream remains
  allocated for the database lifetime and has one 36-byte record per dense
  instance capacity, including identity-yaw static roots. The live-city case
  reports 41.871 KiB of retained orientation capacity.
- `FlagYawInvariantBounds` is an application-authored containment promise that
  Frontier cannot prove from the conservative root AABB alone. If it is false,
  TLAS culling can be incorrect. Omit the flag to retain exact rotated AABBs.
- All hierarchy nodes are resident and ready. The case does not include
  streaming, allocation, mount churn, or unavailable-resource fallback.
- Approximately 24% of the 100,000 potential leaves are visible on an average
  frame. This is intentional frustum behavior, not an all-visible stress test.
- Frontier result construction is timed. Iterating every returned entry,
  constructing GPU command buffers, rendering, animation, skinning, physics,
  and presentation are outside scope.
- Cars and pedestrians share immutable model definitions; the 83 static blocks
  deliberately do not. This yields repeated-asset locality without pretending
  the complete static city is one duplicated tile.
- The benchmark uses one camera and one serial update/publish/select pipeline.
  It does not model split-screen, multiview, or concurrent simulation writers.
