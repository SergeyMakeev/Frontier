# Live-city continuous-motion benchmark

## Goal

The existing moving-camera benchmark intentionally alternates two exact poses
and measures the query's two-entry whole-cut memo. It does not represent a
camera that advances to a new pose every frame. This round adds one realistic,
deterministic city-frame workload without removing that useful recurring-view
microbenchmark.

The timed scope is caller-side actor position staging, two batched spatial
motion submissions, publication, and `selectFrontier()`. Immutable scene and
camera construction stay outside the timed region. The result is an end-to-end
spatial-database frame cost, not a renderer or complete game-engine frame.

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
tracks. The 4,096-frame camera table has no identical adjacent pose. Each
benchmark repetition is fixed at 8,192 frames, covering two complete tables
and 136.533 seconds of simulated time. Scene spatial versions change every
frame, so recurring-view snapshot admission cannot answer this workload.

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

## Interpretation and tradeoffs

- Actor definitions move by translation and uniform scale because the current
  instance transform does not represent rotation. Cars follow curved tracks
  but their authored detail frames remain axis-aligned.
- Car and pedestrian paths are deterministic circular traffic, not AI or
  collision simulation. Filling the 1,100 submitted positions is timed;
  gameplay decision-making is not.
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
