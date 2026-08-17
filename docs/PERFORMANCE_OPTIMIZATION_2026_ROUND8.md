# Performance optimization round 8: render-native cut resolution

Date: 2026-08-17

## Baseline and acceptance policy

This round starts from repository commit `a8303c8`. That revision already
contains the isolated realistic live-city render-submission benchmark and the
paired Cortex-A72 acceptance harness. Every candidate is compared directly to
that exact revision on CPU 4 of the SBC at a fixed observed 2.208 GHz. A change
is kept only when a fresh-process paired run shows a practically meaningful,
repeatable improvement without material regressions in the selection, motion,
or machine-control cases.

The realistic workload contains 100,000 authored leaf instances, 100 rotating
cars with 50 leaves each, 1,000 rotating pedestrians with 10 leaves each, a
four-to-five-level static hierarchy, continuous 40 mph camera and car motion,
and 1.5 mph pedestrian motion. Its current cut averages about 24,000 entries.

## Experiment 1: grouped render-payload resolution

### Observation

The selection-only frame costs roughly 0.66 ms on the SBC, while resolving and
writing its current render submissions raises the frame to roughly 1.12-1.17
ms. Therefore 41-44% of the measured end-to-end CPU time remains downstream of
selection.

The old submission loop calls `tryGetPayload(NodeHandle)` independently for
every visible leaf. For mounted nodes that entails:

1. decoding the slot, index, and generation from the 64-bit handle;
2. checking whether the handle is a TLAS root;
3. bounds-checking the placement slot;
4. loading and validating its live generation stamp;
5. loading the placement's definition index;
6. loading the definition's immutable payload-array pointer;
7. loading the indexed payload;
8. growing an AoS output vector by one element.

Traversal emits entries in placement order. A car therefore commonly presents
50 consecutive handles with the same slot and generation, and a pedestrian 10.
The scalar API throws that locality away and repeats placement validation and
pointer chasing for every part.

### Theory

Resolve a whole `FrontierCutView` at once. Cache the last decoded mount slot,
generation, immutable payload pointer, and node count; refresh that state only
when the run changes. Write directly into already-sized caller storage, and
copy the existing packed instance/error word unchanged. This should turn one
mount-resolution chain per leaf into roughly one per actor while leaving the
12-byte selection/cache layout untouched.

The new renderer-facing `ResolvedFrontierEntry` replaces `NodeHandle` with
`UserPayload`. It is 8 bytes for a 32-bit payload and 16 bytes for a 64-bit
payload. The old benchmark's natural C++ struct occupied 12 and 16 bytes
respectively, so the 32-bit configuration also writes one third less submission
bandwidth.

### Correctness and tradeoffs

- The API writes into caller-owned preallocated storage and returns the written
  prefix. Undersized storage produces an empty span without a partial result.
- Placement generation is validated once per consecutive run. Every local node
  index remains bounds-checked, and stale entries receive `kInvalidPayload`.
- TLAS-root entries retain their scalar validation path because they have no
  mounted slot and are uncommon in the refined live-city cut.
- The fast path relies only on ordering already guaranteed by traversal output;
  correctness does not rely on entries being grouped.
- This first experiment intentionally leaves `FrontierEntry` and cached query
  storage unchanged, avoiding a possible selection-bandwidth regression. Its
  ceiling is that a separate resolution/write pass still exists; a later
  experiment can cache a render-native parallel stream if this ceiling remains
  important.

### Results

The implementation compiled in Release/LTO for both payload widths and passed
all 408 Debug tests spanning BVH4, BVH8, payload32, and payload64. The new tests
exercise grouped mounted entries, a TLAS root, a two-span cut, packed metadata,
undersized output, and stale handles after both kinds of instance are removed.

Focused SBC report:
`/home/codex-perf/frontier/results/frontier-paired-20260817T102523Z`

| Payload | Scalar baseline | Grouped candidate | Paired change | 95% bootstrap interval | CV baseline / candidate |
|---:|---:|---:|---:|---:|---:|
| 32 | 1114.891 us | 868.775 us | **-22.00%** | [-22.32%, -21.46%] | 0.29% / 0.54% |
| 64 | 1180.451 us | 985.804 us | **-15.99%** | [-21.49%, -9.45%] | 4.00% / 4.85% |

Negative is faster. Payload32 was exceptionally stable: its three independent
ABBA cycle effects were -21.46%, -22.20%, and -22.32%. Payload64 was noisier
(-9.45%, -16.59%, and -21.49%), but every cycle improved and even the upper
confidence bound remained far beyond the -0.25% practical-improvement gate.

All 24 fresh processes held CPU 4 at exactly 2.208 GHz. Temperature stayed
between 45.307 and 46.230 C, CPU/wall divergence was at most 0.022%, and no
cooldown or throttling occurred. One-minute system load varied from 1.003 to
2.918, which likely explains part of the payload64 variance and warrants a
larger follow-up sample before using its point estimate for capacity planning.

The baseline and candidate selection executables are byte-identical in both
payload widths, as are the machine-control executables. Only the isolated
submission binaries differ. This is stronger than a merely flat control result:
the candidate cannot introduce a code-layout or library-code regression into
selection, motion, hierarchy traversal, or the controls in this experiment.

### Decision

Keep and commit. The new bulk API removes 246.1 us/frame in payload32 and 194.6
us/frame at the payload64 medians, improving complete CPU-frame throughput by
1.28x and 1.20x respectively without changing the 12-byte selection entry or
its cache. This is a measured architectural improvement, not a benchmark-only
shortcut: the resolved output contains the same payload, stable instance id,
and error code in the same order, and the caller still receives a concrete
preallocated render-submission stream.

## Next experiment

Grouped lookup still performs one full pass over the 12-byte handle frontier
and writes a second 8/16-byte stream every frame. The next ceiling to attack is
that pass itself. The strongest candidate is a render-native result cached and
patched alongside the query's retained frontier, so unchanged entries require
neither handle decoding nor rewriting. A split payload/instance-error layout is
also worth measuring for payload64 because it uses 12 bytes instead of the 16
bytes required by naturally aligned AoS entries.

## Experiment 2: one globally retained resolved stream

### Theory

Keep a contiguous resolved current cut inside `SpatialQuery`. When cached
selection patches a stable-size per-instance range, resolve only that range in
place; when the whole result remains valid, retain every byte. With about 93%
of visible roots reused, this appeared capable of removing nearly the entire
remaining full-cut resolution pass.

### Correctness result

The prototype passed 412 tests across both BVH and payload widths. Added cases
compared every resolved payload/instance/error tuple against the handle cut
after cache hits, readiness changes, handle-only API calls, uncached selection,
and reset.

### SBC result

Focused report:
`/home/codex-perf/frontier/results/frontier-paired-20260817T103946Z`

Compared with the accepted grouped resolver:

| Case | Payload | Baseline | Candidate | Paired change | 95% interval |
|---|---:|---:|---:|---:|---:|
| live-city selection | 32 | 655.371 us | 649.659 us | -0.57% | [-0.89%, -0.27%] |
| live-city selection | 64 | 662.243 us | 657.423 us | -0.82% | [-1.52%, -0.46%] |
| live-city render | 32 | 871.483 us | 861.513 us | -1.14% | [-1.46%, -0.83%] |
| live-city render | 64 | 983.369 us | 984.647 us | +0.22% | [-5.19%, +5.97%] |

All 48 processes stayed at 2.208 GHz and 45.307-46.230 C; maximum CPU/wall
divergence was 0.022%. The payload64 render samples inherited substantial host
load noise, but even the stable payload32 result is far below the expected
gain. The small selection-only improvement is likely favorable code placement
or secondary control flow and is not enough to justify the extra architecture
by itself.

### Diagnosis and decision

Reject the global layout. The cache's 93% reuse statistic is per visible
instance, but the retained output can be patched only when the complete visible
instance sequence and every preceding bucket count remain unchanged. A car
camera moving through a city crosses visibility boundaries continually. One
entering/leaving instance invalidates the global contiguous layout and forces a
full resolve even though nearly every surviving instance's cached cut is still
valid.

The next prototype moves resolved retention into each instance record. It will
return a compact ordered list of zero-copy per-instance segments. Visibility
churn then adds/removes only segment descriptors; cached leaf payloads never
move, and a re-walk resolves only that instance's new current cut. This trades
one contiguous array for scatter/gather submission, which is a natural fit for
per-instance transforms and render batching.

## Experiment 3: per-instance resolved slabs and zero-copy render runs

### Theory

Make the unit of renderer-facing retention match the existing unit of query
reuse. Each cached instance record already owns a stable block in the handle
slab. Add a parallel resolved slab at the same offset and a one-byte validity
flag per instance. A hit returns the existing resolved prefix without touching
its leaf entries. A miss rewrites and resolves only that instance. Each frame
then emits an ordered list of `{begin, count}` descriptors for the visible
instances instead of rebuilding a globally contiguous leaf array.

In the live-city workload the current cut averages 24,073 leaf entries but is
described by only 291 runs. Visibility churn therefore rewrites about 2.3 KiB
of descriptors, not a 188 KiB payload32 or 376 KiB payload64 submission array.
The roughly 7% of instance records that fail reuse are the only records whose
handle and resolved leaf bytes are rewritten.

### Architecture and data layout

- `SpatialQuery::store_` remains the authoritative 12-byte handle-entry slab.
  A record's three buckets are laid out as `shared`, `currentOnly`, then
  `idealOnly`.
- `SpatialQuery::resolvedStore_` mirrors the same allocation and offsets. Only
  the `shared + currentOnly` prefix is initialized because that is the
  renderable current cut. It uses the existing 8-byte payload32 or 16-byte
  payload64 `ResolvedFrontierEntry`.
- `resolvedRecords_[instance]` is a lazy one-byte validity stream. The normal
  handle API invalidates the byte only when it rewrites that record; switching
  between handle and render queries cannot expose stale payloads.
- `RenderFrontierRun` is exactly eight bytes: a 32-bit slab offset and 32-bit
  count. Runs follow visible-instance order and index the retained resolved
  slab. Offsets remain valid when a later miss reallocates the slab; storing
  raw pointers here would not.
- `RenderFrontierView` returns the immutable slab, the ordered run span, and
  the exact total entry count. Cached hierarchical queries never assemble the
  old global output buckets on this path. Reuse-disabled and all-flat queries
  fall back to one materialized contiguous run, keeping one consumer API.
- Slab compaction copies valid handle and resolved records together, preserving
  matching offsets. Expired records drop both mirrors.

### Correctness

The renderer view is a set-equivalent current cut. Its order is deliberately
instance-major (`shared + currentOnly` per visible instance), rather than the
handle API's global bucket-major order. The test suite resolves an independent
handle query and compares every payload, packed instance id, and error code
after cache hits, readiness changes, an intervening handle-only query,
reuse-disabled queries, camera motion, and reset. It also checks every run
against the slab bounds and verifies that run counts sum to the advertised
entry count.

The prototype compiled in both benchmark payload widths. The local Debug
BVH4/BVH8/payload32/payload64 matrix passed all 412 tests. No existing selection result, handle
layout, or mounted hierarchy was changed.

### SBC results

Focused paired report:
`/home/codex-perf/frontier/results/frontier-paired-20260817T110135Z`

Compared with accepted commit `6d0bac1`:

| Case | Payload | Baseline | Candidate | Paired change | 95% interval | CV baseline / candidate |
|---|---:|---:|---:|---:|---:|---:|
| live-city selection control | 32 | 658.670 us | 654.302 us | -0.34% | [-0.59%, -0.04%] | 0.63% / 0.46% |
| live-city selection control | 64 | 657.719 us | 659.571 us | +0.04% | [-0.81%, +0.53%] | 0.28% / 0.55% |
| live-city render production | 32 | 871.047 us | 576.893 us | **-33.90%** | [-34.62%, -33.14%] | 0.37% / 1.13% |
| live-city render production | 64 | 879.894 us | 634.755 us | **-30.53%** | [-32.18%, -27.81%] | 6.36% / 0.34% |

All six payload32 render samples were large wins; its three cycle effects were
-33.14%, -34.62%, and -33.91%. All payload64 cycles also won despite noisy
baseline processes. The selection control is practically unchanged, showing
that the benefit comes from removing global result assembly and resolution,
not from unrelated traversal code layout.

All 48 processes ran CPU 4 at exactly 2.208 GHz. Temperature stayed between
43.461 and 46.230 C, one-minute load between 1.000 and 1.542, and maximum
CPU/wall divergence was 0.022%.

### Tradeoffs and decision

Keep and commit. Relative to the already grouped-resolver baseline this removes
294.2 us/frame in payload32 and 245.1 us/frame in payload64, producing 1.51x
and 1.44x complete CPU-frame throughput respectively.

The cost is persistent renderer-cache memory: one resolved entry slot beside
each retained handle slot, plus one byte per instance and eight bytes per
visible run. Payload64 retains twice the resolved-slab bytes of payload32 and
still has four bytes of natural AoS padding per entry. Results are no longer a
single contiguous leaf span, so consumers must submit or iterate scatter/gather
runs. This is a favorable fit for instance transforms and batching but may be
less convenient for an API that insists on one flat GPU upload. Such callers
can flatten explicitly, while performance-sensitive renderers avoid paying
that bandwidth every frame.

### Remaining measurement question

This result measures production of the complete renderer-facing structure,
including all motion, TLAS publication, selection, miss resolution, and run
generation. It does not yet time a consumer reading every retained leaf after
selection. The next controlled experiment will add the same payload/metadata
scan to baseline and candidate binaries so the scatter/gather iteration cost
is included without conflating it with this production win.

## Experiment 4: downstream full-leaf scan control

### Measurement design

Compile isolated baseline and candidate submission binaries with the same
GCC 13.3 Release/LTO/BVH4 settings. After producing the render frontier, both
consumers accumulate the payload and packed instance/error word from every
resolved leaf into an observed checksum. The baseline scans one contiguous
span; the candidate scans the same fields through its ordered per-instance
runs. Allocation and graphics-driver calls remain outside scope, but no leaf
can now avoid downstream CPU iteration.

The baseline source is the accepted grouped resolver from `6d0bac1`, rebuilt
in the isolated `scan-base` worktree with only the checksum benchmark change.
The candidate uses the committed per-instance cache from `932d854` with the
same checksum. This avoids comparing the new workload against an old binary
that did not perform it.

### SBC results

Focused paired report:
`/home/codex-perf/frontier/results/frontier-paired-20260817T112005Z`

| Payload | Contiguous baseline | Scatter/gather candidate | Paired change | 95% interval | CV baseline / candidate |
|---:|---:|---:|---:|---:|---:|
| 32 | 922.454 us | 674.842 us | **-26.77%** | [-27.00%, -26.63%] | 0.36% / 0.62% |
| 64 | 975.393 us | 772.535 us | **-20.88%** | [-25.59%, -16.04%] | 6.81% / 0.59% |

Payload32 is decisive: all three independent cycle effects lie within a
0.37-point band (-27.00%, -26.67%, -26.63%). Payload64 baseline samples were
again noisy under varying host load, but every cycle won (-25.59%, -20.72%,
-16.04%) and even the upper confidence bound remains a large improvement.

All 24 processes held CPU 4 at exactly 2.208 GHz. Temperature was
44.384-46.230 C, one-minute load 1.007-2.902, and maximum CPU/wall divergence
0.018%.

The scan costs about 98 us/frame on the payload32 candidate and 138 us/frame
on payload64, versus about 51 and 95 us added to the contiguous baselines.
Thus scatter/gather iteration gives back roughly 47 us (payload32) and 43 us
(payload64), but avoids 294 and 245 us of production work. Complete throughput
including every leaf read remains 1.37x and 1.26x faster.

### Memory observation

The candidate query reports 1,358.5 KiB for payload32 and 1,868.5 KiB for
payload64 versus 1,228.2 KiB for the handle-only baseline query: net query
increases of 130.3 and 640.3 KiB. The increase is smaller than a raw second
slab because the specialized path no longer retains global handle-output
buckets. The benchmark baseline also owns a separate worst-case 100,000-entry
submission vector (781.3 KiB payload32, 1,562.5 KiB payload64), whereas the
candidate view directly references query storage. End-to-end retained memory
in this workload is therefore lower despite the per-instance mirror; callers
that provision a tighter baseline submission vector will see a smaller memory
advantage.

### Decision

Keep the full-leaf scan in the realistic submission benchmark. The earlier
measurement caveat is closed: the performance win survives actual downstream
iteration by a wide margin. The remaining payload64 gap points to the next
experiment: split the naturally padded 16-byte AoS entry into dense payload
and packed-metadata streams, reducing retained bytes and scan bandwidth while
preserving the per-instance run architecture.

## Experiment 5: run-level instance identity and compact leaf streams

### Theory

The proposed payload/metadata SoA still repeats a 24-bit instance id on every
leaf. That value is invariant across an instance cache record and its render
run; renderers also select the instance transform once for the run, not once
per leaf. Move instance identity into the run descriptor, keep payloads in a
dense `UserPayload` stream, and store only the eight-bit error code per leaf.

This changes raw renderer-cache bytes per leaf from 8 to 5 for payload32 and
from a padded 16 to 9 for payload64. `RenderFrontierRun` grows from 8 to 12
bytes by adding the instance id, but only about 291 descriptors represent the
24,073-leaf average cut, so this costs roughly 1.1 KiB per frame while removing
tens or hundreds of KiB from retained leaf storage.

### New API and layout

`RenderFrontierView` now exposes three independent spans:

- `payloadStorage()`: dense immutable application payloads;
- `errorStorage()`: one quantized error byte at the matching offset;
- `runs()`: `{begin, count, instance}` descriptors in visible-instance order.

Indexing a run returns a `RenderFrontierSpan` containing matching payload and
error subspans plus the invariant instance id. The consumer uses the instance
once to select transform/material batching state and streams the two leaf
arrays. The uncached/all-flat fallback groups consecutive entries by instance
and returns the same API.

The query owns parallel payload and error slabs at the same offsets as its
handle cache. A compact bulk resolver validates each mounted-handle run once,
writes payloads and error bytes directly, and omits the redundant instance
word. Compaction moves the handle, payload, and error blocks together.

### Correctness

The renderer-query test reconstructs the legacy `instanceAndError` word from
`run.instance` and each error byte, then compares every complete tuple against
an independently resolved handle query. It exercises hits, readiness
invalidation, an intervening handle-only query, camera changes, reuse-disabled
fallback, and reset; every run is bounds-checked against both storage streams.
All 412 Debug tests passed across BVH4, BVH8, payload32, and payload64.

### SBC result

Direct AoS-versus-compact report:
`/home/codex-perf/frontier/results/frontier-paired-20260817T113718Z`

| Payload | AoS per-instance cache | Compact streams | Paired change | 95% interval | CV baseline / candidate |
|---:|---:|---:|---:|---:|---:|
| 32 | 668.281 us | 652.287 us | **-2.13%** | [-2.63%, -1.87%] | 0.71% / 1.02% |
| 64 | 772.011 us | 722.946 us | **-6.26%** | [-6.64%, -5.58%] | 0.23% / 0.47% |

Every cycle improved: payload32 effects were -2.63%, -1.89%, and -1.87%;
payload64 effects were -5.58%, -6.57%, and -6.64%. All 24 processes held CPU
4 at exactly 2.208 GHz. Temperature stayed at 44.384-46.230 C, one-minute load
at 1.019-2.818, and maximum CPU/wall divergence was 0.020%.

Reported `SpatialQuery` capacity fell from 1,358.5 to 1,169.0 KiB in payload32
(-14.0%) and from 1,868.5 to 1,424.0 KiB in payload64 (-23.8%). These whole-
query reductions include unchanged record, handle, visibility, worker, and run
capacity, so they are smaller than the 37.5%/43.75% raw leaf-layout reductions.

### Tradeoffs and decision

Keep and commit. The API is explicitly instance-major and SoA, which is less
convenient for consumers expecting one AoS leaf object. In exchange, it matches
the actual transform boundary, removes semantically redundant data, reduces
cache/TLB pressure, avoids alignment padding, and gives renderers the option to
stream payloads without touching errors when prioritization is not needed.

The improvement compounds with Experiment 3 rather than replacing it. Applying
the direct paired effects to the full-leaf results yields approximately 28.3%
payload32 and 25.8% payload64 improvement over the accepted grouped-resolver
baseline, while using substantially less retained renderer state than the AoS
prototype.

## Experiment 6: opt-in instance-granular frustum culling

### Observation and theory

The compact renderer still re-walked 20.43 of 293 visible roots per frame.
Records are cacheable only when their top-level root was wholly inside the
frustum; a nonzero plane mask forces exact descendant tests on every call.
Cars and pedestrians are already submitted, transformed, and clipped as actor
runs. For such small articulated actors, culling every child part on the CPU
can cost more than submitting the handful of offscreen boundary parts.

After the TLAS proves an actor root is not outside, erase its descendant plane
mask for the render-native query only. LOD decisions remain exact and continue
to use camera/motion margins; only frustum precision changes from leaf to
actor-root granularity. The normal handle query remains leaf-exact.

### Rejected broad version

The first prototype applied this rule to every hierarchical instance. Locally
it reached 99.96% record reuse and only 0.11 walks per frame, but submissions
rose from about 24,073 to 30,806 leaves (+28.0%) because deep static-world
blocks at the frustum boundary were emitted whole. Despite its dramatic CPU
speed, that is an unreasonable general renderer tradeoff and was rejected
without an SBC acceptance run.

### Revised architecture

Add `SpatialDatabase::setInstanceRenderAsUnit()`. The policy is an explicit
bit in the existing cold instance flag word, so the 80-byte instance record
and 32-byte public descriptor do not grow. Changing the policy invalidates
that instance's cached frontier immediately.

During a render-native cached query, the selector checks the policy only for
the uncommon shell of roots with a nonzero TLAS plane mask. Opted-in roots use
mask zero for their subtree walk/cache record; ordinary roots retain exact
descendant masks. The live-city benchmark opts in the 100 cars and 1,000
pedestrians but leaves all static blocks exact.

This version raises average submissions only from 24,072.7 to 24,139.3
(+0.28%), while reducing walks from 20.43 to 12.97 (-36.5%) and increasing
reuse from 93.03% to 95.58%.

### Correctness

A dedicated test moves a two-part actor to a frustum boundary where the exact
handle query returns one child. Opting into render-as-unit returns both children
as a conservative superset; the handle API still returns exactly one. Disabling
the policy invalidates the retained record and restores the exact render set on
the next call. The complete BVH4/BVH8/payload32/payload64 Debug matrix passed
all 416 tests.

### SBC result

Direct compact-exact versus compact-actor-unit report:
`/home/codex-perf/frontier/results/frontier-paired-20260817T115024Z`

| Payload | Exact descendant cull | Actor-unit candidate | Paired change | 95% interval | CV baseline / candidate |
|---:|---:|---:|---:|---:|---:|
| 32 | 652.736 us | 647.556 us | **-0.89%** | [-1.00%, -0.81%] | 0.46% / 0.58% |
| 64 | 718.757 us | 709.689 us | **-1.31%** | [-1.51%, -0.94%] | 0.43% / 0.39% |

Every cycle improved: payload32 -0.87%, -1.00%, -0.81%; payload64 -1.49%,
-0.94%, -1.51%. All 24 processes held CPU 4 at 2.208 GHz, temperature stayed
44.384-45.307 C, one-minute load 1.024-1.896, and maximum CPU/wall divergence
was 0.021%.

### Tradeoffs and decision

Keep and commit. The caller must opt in only actor-sized roots for which GPU
clipping or meshlet culling is cheaper than CPU child traversal. The render cut
is then a conservative superset of the exact handle cut at the frustum edge;
it never omits visible leaves. LOD selection, readiness, payloads, errors, and
instance identity remain unchanged.

The longer-lived full actor records cross a cache-slab capacity boundary in
this trajectory: reported query capacity grows from 1,169 to 2,257 KiB for
payload32 and 1,424 to 2,768 KiB for payload64. Absolute state remains small,
but the roughly 1.1-1.3 MiB increase is a real cost for a 1% frame gain. A
future bounded-cache/eviction experiment should reclaim invisible actor
records without giving back the boundary reuse.

## Experiment 7: sampled visibility aging and right-sized compaction (rejected)

### Theory

The actor-unit policy retains full cached cuts for roots encountered anywhere
along the driving route. Add one visibility epoch byte per instance, sweep the
record table once every 256 frames, evict cuts absent for three sampled epochs,
and compact into `used - garbage` entries instead of preserving the old slab
capacity. This should shrink the query's long-route working set and might
improve cache/TLB behavior enough to recover part of the memory cost from
Experiment 6.

### Variant A: sampled visibility only

The first implementation marked only the roots visible on each sweep. It cut
payload32 query capacity from 2,257.0 to 776.8 KiB, but short visibility
intervals between two samples were invisible to the aging policy. Average
walks rose from 12.966 to 13.392 roots per frame and reuse fell from 95.576% to
95.431%.

SBC report:
`/home/codex-perf/frontier/results/frontier-paired-20260817T120448Z`

| Payload | Actor-unit baseline | Aging candidate | Paired change | 95% interval | CV baseline / candidate |
|---:|---:|---:|---:|---:|---:|
| 32 | 644.980 us | 676.566 us | **+5.07%** | [+4.78%, +5.53%] | 0.81% / 0.58% |
| 64 | 710.206 us | 746.895 us | **+5.12%** | [+4.72%, +5.45%] | 0.41% / 0.30% |

### Variant B: exact per-frame visibility stamps

The second implementation wrote the current epoch byte for every visible root
on every frame, eliminating sampling blind spots. The capacity result remained
excellent (778.5 KiB for payload32), but revisiting legitimately aged-out
actors still requires new subtree walks. The walked/reuse counters were
effectively unchanged from Variant A, demonstrating that revisit misses rather
than sampling error were the fundamental cost.

SBC report:
`/home/codex-perf/frontier/results/frontier-paired-20260817T121207Z`

| Payload | Actor-unit baseline | Aging candidate | Paired change | 95% interval | CV baseline / candidate |
|---:|---:|---:|---:|---:|---:|
| 32 | 643.219 us | 676.866 us | **+5.70%** | [+5.53%, +5.85%] | 0.44% / 0.87% |
| 64 | 714.039 us | 749.544 us | **+4.88%** | [+4.57%, +5.12%] | 0.48% / 0.45% |

Both reports held CPU 4 at exactly 2.208 GHz with maximum CPU/wall divergence
of 0.023% and temperatures between 44.384 and 45.307 C. The regressions are
therefore decisive, not environmental noise.

### Decision

Reject and fully revert both variants. The experiment offers a valid optional
memory policy (about 1.48 MiB saved in payload32), but it is a CPU-for-memory
trade rather than a performance optimization. This project deliberately keeps
off-route cuts because the cyclic city trajectory revisits them and converts
that retained state directly into fewer walks. No production source from this
experiment is committed; the negative results are retained here to prevent a
future repetition.

## Experiment 8: live-city profile-guided ARM build

### Motivation and decomposition

Direct component measurements on the accepted SBC build put the payload32
live-city driving frame at 656 us and the motion/publication-only companion at
144 us. Roughly 512 us therefore remains in camera query, frontier production,
and output handling. This is large enough that instruction placement, inlining,
and branch layout can matter even after the data-path changes above.

Hardware sampling was not yet available (`perf` was absent and the benchmark
account had no passwordless sudo), so GCC edge/call profiling was used instead.
An instrumented build executed the complete 8,192-frame render-submission
trajectory on pinned CPU 4. It recorded the real contiguous 40 mph camera path,
all 1,100 moving actor roots, recurrent actor revisits, cache hits/misses, and
every downstream leaf read.

### Prototype and a necessary negative control

The first profile corpus contained only the payload32 executable. That build
improved payload32 by 3.95%, but payload64—whose target-specific object paths
had no matching profile—regressed 1.44%. This is a useful negative control:
turning on `-fprofile-use` without training every deployed ABI is not safe.

Prototype report:
`/home/codex-perf/frontier/results/frontier-paired-20260817T122614Z`

| Payload | Normal LTO | Partially trained PGO | Paired change | 95% interval |
|---:|---:|---:|---:|---:|
| 32 | 643.905 us | 620.545 us | **-3.95%** | [-4.55%, -3.37%] |
| 64 | 708.280 us | 717.704 us | **+1.44%** | [+0.76%, +2.20%] |

Training both private payload layouts reversed the payload64 result: a manual
balanced prototype improved payload32 by 3.93% and payload64 by 5.22% in report
`frontier-paired-20260817T123449Z`.

### Productionized build architecture

CMake now exposes two explicit GCC phases:

- `FRONTIER_PGO_MODE=GENERATE` instruments compilation and the LTO link;
- `FRONTIER_PGO_MODE=USE` consumes corrected counts and tolerates genuinely
  unexercised translation units;
- `FRONTIER_PGO_DIR` identifies the corpus shared by both phases.

The flags are intentionally build-wide. GCC consumes profile information at
compile and LTO link time, and the generating executable must link the profile
runtime. A dedicated `frontier_pgo_training` target links the public
`frontier` archive rather than the ABI-explicit benchmark archive. This detail
is required because GCC keys `.gcda` files by object output path; exercising a
byte-identical private library does not train the production archive.

`run_arm_pgo.sh` automates the native ARM pipeline. It creates a unique corpus,
builds instrumented public/payload64/payload32 targets, runs the realistic
trajectory for all three on a caller-selected CPU, reconfigures the same build
for profile use, and emits the optimized production archive and benchmark
binaries. Typical invocation on the SBC is:

```bash
FRONTIER_PGO_CPU=4 FRONTIER_PGO_JOBS=4 ./run_arm_pgo.sh build-arm-pgo
```

### Scripted SBC acceptance result

The final comparison uses binaries produced solely by that script, against the
accepted non-PGO actor-unit build:
`/home/codex-perf/frontier/results/frontier-paired-20260817T124638Z`

| Payload | Normal LTO | Scripted PGO | Paired change | 95% interval | CV baseline / candidate |
|---:|---:|---:|---:|---:|---:|
| 32 | 648.631 us | 619.007 us | **-4.57%** | [-5.14%, -3.77%] | 0.59% / 0.76% |
| 64 | 708.092 us | 675.981 us | **-4.53%** | [-4.92%, -4.32%] | 0.36% / 0.32% |

Every one of the six independent cycle effects improved. All 24 processes ran
on CPU 4 at exactly 2.208 GHz; temperature was 46.230-47.153 C and maximum
CPU/wall divergence was 0.021%. The result is a reproducible build-level gain
on top of all architectural improvements, not a one-off manual flag test.

### Tradeoffs and decision

Keep and commit. PGO adds two full compilations and three 8,192-frame training
runs. Its output is tied to compiler version, source control flow, compile-time
payload layout, and training distribution; an application whose production
camera/actor distribution differs substantially should train with its own
representative driver. The checked-in script deliberately generates profiles
on the target rather than committing fragile path- and CFG-keyed `.gcda`
artifacts.

The default build remains unchanged. Deployments seeking maximum SBC
throughput opt into the scripted mode and receive about 4.5% additional
end-to-end live-city performance in both supported payload layouts.

## Experiment 9: coalesced motion-group TLAS publication (rejected)

### Theory

Cars and pedestrians are submitted through two `MotionGroup` batches. The
existing move path immediately grows each edited TLAS leaf and propagates its
new envelope toward the root. Consecutive actors sometimes share a leaf host,
so a batch-level dirty-host list appeared able to replace repeated ancestor
growth with one propagation per unique host. A second variant deferred the
coalescing boundary across both actor batches until `applyUpdates()`.

### Result

The first controlled report is
`/home/codex-perf/frontier/results/frontier-paired-20260817T125930Z`:

| Payload | Baseline | Candidate | Paired change | 95% interval | Baseline / candidate CV |
|---:|---:|---:|---:|---:|---:|
| 32 | 143.625 us | 143.016 us | -1.95% | [-4.73%, -0.46%] | 3.77% / 0.11% |
| 64 | 143.765 us | 143.888 us | +0.06% | [+0.01%, +0.09%] | 0.14% / 0.15% |

The payload32 paired estimate was created almost entirely by one noisy
baseline cycle: its raw median improvement was only 0.42%, while the three
cycle effects were -4.73%, -0.59%, and -0.46%. Payload64 was a stable slight
loss. Deferring across both groups also worsened a direct motion smoke from
about 144 to 148 us. The moving actors are spatially dispersed enough that
host/ancestor duplication is too small to repay dirty-list maintenance.

### Decision

Reject and fully revert both variants. Motion publication was not the dominant
remaining cost, and the data showed no payload-independent win.

## Experiment 10: direct zero-error actor-root emission (rejected)

### Theory

The car and pedestrian definitions are flat forests of fully-ready,
zero-geometric-error leaves. When their root is already wholly inside the
frustum, no child can make either a culling or LOD decision. Classify this
topology at registration, place a flag in the hot mount record, and emit child
handles directly without entering the generic subtree walker.

### Instrumented result

The branch activated only 0.102 times per live-city frame while the query
re-walked 12.966 roots per frame. Instrumented payload32 selection remained
about 392 us and the measured walk remained about 282 us. The reason is not a
slow implementation: actor-unit caching already retains these flat actor cuts.
Almost every remaining miss belongs to a deep static block intersecting a
frustum plane. The proposed actor shortcut attacked a path that the retained
architecture had already removed.

### Decision

Reject and remove the classification flag and direct-emission loop. Retain the
diagnosis: per-instance reuse percentages hid the fact that essentially all
remaining hierarchy work was concentrated in roughly thirteen large static
boundary roots.

## Experiment 11: provably fully-refined frustum-only traversal

### Phase profile

Temporary `FRONTIER_STATS` timers divided one payload32 frame into independent
phases. The exact values moved slightly with instrumentation, but the stable
shape was:

| Phase | Approximate cost per frame |
|---|---:|
| Actor-position generation | 24-27 us |
| Car motion submission | 45-52 us |
| Pedestrian motion submission | 149-156 us |
| TLAS publication | 1.6-1.7 us |
| Selection | 390-410 us |
| Downstream submission scan | 49-53 us |

Inside selection, about 13 misses consumed 343-362 us. Their hierarchy walk
alone cost 279-293 us and handle-to-payload resolution another 52-56 us. Each
frame visited approximately 2,283 interior nodes, tested 2,296 BVH4 blocks,
and retained 8,759 lanes. This established the actual optimization target:
exact traversal of deep static blocks cut by a moving frustum.

### Key observation

The static-city authoring data uses geometric error 10,000 for every interior
node and zero for every terminal leaf. At the benchmark's camera scale and
1,500 m far plane, every visible interior node must refine. The generic walker
nevertheless performs, for every surviving interior lane:

1. an AABB-to-camera-envelope squared distance;
2. a vector square root and divide on NEON;
3. screen-error comparison and quantization;
4. the LOD stop/descend branch;
5. validity-margin arithmetic for a cut that cannot be cached because the root
   still has a nonzero frustum mask.

All of that work is redundant if the library can prove up front that even the
smallest interior error remains over threshold at the farthest possible point
in the root bounds.

### Conservative proof

Registration computes the minimum geometric error across ordinary interior
nodes. Eligibility is disabled when any node is mountable, any interior error
is zero, or any terminal leaf has nonzero error. For an eligible boundary
placement, selection computes an upper bound `Dmax` on the distance from the
camera damping envelope to any point in the definition root box and tests:

```text
(min(minInteriorError, mountErrorClamp) * cameraK / threshold)^2 > Dmax^2
```

Every descendant box lies inside the validated root bounds, so its actual
minimum camera distance cannot exceed `Dmax`. The left side is the squared
distance at which the least-detailed interior node would cross the threshold.
If the inequality holds, every interior node is strictly over threshold. The
specialized loop may therefore perform only masked frustum tests, push
surviving interior child ids, and emit surviving zero-error leaves. The result
is bit-exact: no overdraw, no omitted leaf, and the same zero error codes.

The optimization is limited to fully-ready, overlay-free, top-level mounted
trees whose TLAS root remains partially intersecting. Roots wholly inside use
the existing retained-cut cache; roots outside were already rejected by the
TLAS. Small definitions below sixteen authored nodes retain the general walk
because the one-time proof costs more than the distance arithmetic it removes.

### Runtime metadata and data layout

No hot structure grows:

- `SubtreeDefinitionRt` remains at its established 160-byte ceiling. The
  magnitude of `minInnerErrorAndRootFlag` stores the conservative minimum
  interior error; its otherwise-unused sign bit preserves the independent
  root-leaves-only classification (negative zero represents a flat root-leaf
  forest).
- `MountTransformRt` remains exactly 32 bytes. Bit 31 of
  `definitionAndFlags` still identifies root-leaf forests; bit 30 now marks a
  sufficiently large fully-refined candidate; the low 30 bits hold the
  definition index.
- The immutable BVH remains the existing BVH4 SoA layout: six bound vectors,
  error vector, and child-id vector per wide block, with packed valid/leaf/
  zero-error masks. The specialized loop reads bounds, masks, and child ids but
  never consumes the error vector.
- Classification is an O(nodes + wide blocks) scan during subtree
  registration, outside frame selection. Query and database frame-state byte
  counts are unchanged.

The extra flag reduces the theoretical definition-index field from 31 to 30
bits (roughly 1.07 billion simultaneous definitions). This is far above the
practical memory limit and buys a single hot test without another pointer or
record load.

### Code-placement experiments

The arithmetic optimization was immediately large, but several layout
variants were measured before acceptance:

1. **Inline specialized template.** Instrumented walk time fell from about
   293 to 217 us. A production paired run
   (`frontier-paired-20260817T134516Z`) improved live-city selection by
   13.45-13.82% and full render by 11.45-12.28%, but enlarged the generic
   fully-ready walker and produced small, ABI-dependent uncached-control
   regressions.
2. **Out-of-line `cold` helper.** This restored generic placement but GCC
   optimized the entire NEON culling loop for size. Report
   `frontier-paired-20260817T141812Z` retained only a 5.11-5.87% selection win
   and 5.15-5.45% full-render win.
3. **Small-tree break-even gate.** The uncached control definition contains
   only three authored nodes; calling the proof there was counterproductive.
   A sixteen-node eligibility threshold removed that executed overhead.
4. **Final isolated section.** Dispatch moved out of `runSubtreeImpl` to the
   fully-ready TLAS-root boundary and uses the spare mount flag. The generic
   fully-ready walker returned to exactly its baseline `0x7f8` bytes in both
   payload ABIs. The specialized helper is no-inline and lives in
   `.text.frontier_refined`, isolating layout without the size-biased `cold`
   optimization. This recovered the full compute win.

### Correctness

A dedicated test builds two matching 21-node boundary hierarchies. The fast
definition has zero-error leaves; the independent reference uses tiny nonzero
leaf errors to disable specialization while preserving terminal selection.
Across camera-boundary placement, their visible payload sets must match. In a
statistics build the test additionally proves that the fast definition enters
the new path and the reference does not.

The complete Debug matrix passed all 420 tests across BVH4, BVH8, payload32,
and payload64. The final generic fully-ready walker has the same symbol size as
the frozen baseline in all production binaries.

### Final SBC acceptance

Final combined report:
`/home/codex-perf/frontier/results/frontier-paired-20260817T143013Z`

| Case | Payload | Baseline | Candidate | Paired change | 95% interval | Baseline / candidate CV |
|---|---:|---:|---:|---:|---:|---:|
| live-city selection | 32 | 651.698 us | 549.488 us | **-15.70%** | [-16.40%, -14.83%] | 0.88% / 1.17% |
| live-city selection | 64 | 653.808 us | 546.012 us | **-16.36%** | [-16.53%, -16.24%] | 0.77% / 0.57% |
| live-city render + scan | 32 | 648.106 us | 547.757 us | **-15.62%** | [-15.82%, -15.51%] | 0.85% / 0.61% |
| live-city render + scan | 64 | 710.811 us | 603.237 us | **-14.86%** | [-15.11%, -14.58%] | 0.47% / 0.65% |
| motion-only | 32 | 143.249 us | 143.194 us | +0.16% | [-0.37%, +0.69%] | 0.25% / 0.59% |
| motion-only | 64 | 143.748 us | 143.331 us | -0.19% | [-0.40%, +0.09%] | 0.13% / 0.30% |
| uncached hierarchy, 50% | 32 | 1635.956 us | 1634.032 us | +0.77% | [-0.41%, +2.36%] | 0.58% / 1.54% |
| uncached hierarchy, 50% | 64 | 1614.468 us | 1577.796 us | -5.78% | [-8.41%, -0.39%] | 7.93% / 1.48% |
| uncached hierarchy, 100% | 32 | 3152.754 us | 3116.519 us | -0.16% | [-0.90%, +0.84%] | 1.28% / 1.59% |
| uncached hierarchy, 100% | 64 | 2962.906 us | 2953.928 us | -0.22% | [-0.76%, +0.51%] | 0.67% / 0.74% |

Every live-city cycle improved by at least 14.58%. Motion and all unrelated
controls are statistically non-regressing or improved; the noisy payload64
50% baseline accounts for its unusually large point estimate. All 120 fresh
processes ran on CPU 4 at exactly 2.208 GHz. Temperature stayed between 45.307
and 46.230 C, one-minute load between 1.002 and 2.041, and maximum CPU/wall
divergence was 0.025%.

### Tradeoffs and decision

Keep and commit. The optimization deliberately helps a specific but common
rendering shape: large, fully-ready static detail trees that are close enough
to require their finest authored level and straddle the moving view boundary.
It provides no benefit to shallow trees, mounted compositions, overlays,
nonzero-error terminal leaves, or trees near an LOD transition; those remain on
the unchanged generic walker. Registration performs one additional topology
scan and one mount-flag bit is consumed. In exchange, the common live-city
frame is about 1.19x faster end to end with exact output and no additional
frame-state memory.

## Experiment 12: retrain PGO after path specialization

### Theory

The accepted frustum-only traversal changes which functions execute in the
live-city training workload. Retraining GCC PGO on that workload could compound
the source-level win through better layout, inlining, and branch probabilities.
The first retraining corpus intentionally matched the previous procedure: the
live-city render-submission frame alone, for the public payload64 library and
both benchmark payload libraries.

### First corpus result

Paired report:
`/home/codex-perf/frontier/results/frontier-paired-20260817T145702Z`

| Case | Payload | Baseline | Candidate | Paired change | 95% interval |
|---|---:|---:|---:|---:|---:|
| live-city selection | 32 | 653.958 us | 519.929 us | **-20.30%** | [-20.67%, -19.95%] |
| live-city selection | 64 | 655.307 us | 517.513 us | **-20.63%** | [-21.07%, -20.14%] |
| live-city render + scan | 32 | 616.646 us | 520.731 us | **-15.48%** | [-16.01%, -14.99%] |
| live-city render + scan | 64 | 673.695 us | 572.834 us | **-14.83%** | [-15.49%, -14.11%] |
| motion-only | 32 | 143.559 us | 126.249 us | **-12.05%** | [-12.13%, -11.98%] |
| motion-only | 64 | 143.852 us | 126.889 us | **-11.91%** | [-12.03%, -11.78%] |
| uncached hierarchy, 50% | 32 | 1671.656 us | 1928.216 us | **+15.61%** | [+14.19%, +17.02%] |
| uncached hierarchy, 50% | 64 | 1609.772 us | 1937.327 us | **+21.61%** | [+19.95%, +23.39%] |
| uncached hierarchy, 100% | 32 | 3138.793 us | 3634.633 us | **+15.31%** | [+14.14%, +16.47%] |
| uncached hierarchy, 100% | 64 | 2975.739 us | 3489.249 us | **+17.36%** | [+16.67%, +17.95%] |

All 240 samples ran at 2.208 GHz between 45.307 and 46.230 C, with load
0.920-1.283 and maximum CPU/wall divergence 0.036%. The regressions are real,
not environmental noise.

### Diagnosis and next corpus

Before specialization, the live-city workload exercised the general fully-ready
walker, so the single-case corpus incidentally trained both the common scenario
and generic mounted-hierarchy traversal. After specialization, those deep static
boundary roots enter the new helper instead. The old generic walker therefore
receives almost no useful counts even though unrelated workloads still depend
on it. GCC's PGO use pass consequently treats important generic blocks as cold;
the 15-22% control loss is the result.

Reject the first corpus. Extend training with the 10,000-instance 50% and 100%
uncached hierarchy cases for all three payload-library variants. This deliberately
teaches GCC both sides of the new architectural split while keeping live-city
render submission as the primary realistic workload. Rebuild and accept only if
the large live-city/motion gains survive without the generic-walker regressions.

### Balanced-corpus screening result

Paired report:
`/home/codex-perf/frontier/results/frontier-paired-20260817T152539Z`

Adding the two generic cases at Google Benchmark's default roughly 0.5-second
minimum materially reduced, but did not eliminate, the overfit:

| Case | Payload | Paired change | 95% interval |
|---|---:|---:|---:|
| live-city selection | 32 | **-20.62%** | [-21.09%, -20.07%] |
| live-city selection | 64 | **-20.73%** | [-21.05%, -20.43%] |
| live-city render + scan | 32 | **-16.75%** | [-17.33%, -16.10%] |
| live-city render + scan | 64 | **-13.51%** | [-13.95%, -13.06%] |
| motion-only | 32 | **-12.83%** | [-12.94%, -12.68%] |
| motion-only | 64 | **-12.63%** | [-12.95%, -12.42%] |
| uncached hierarchy, 50% | 32 | **+5.27%** | [+3.98%, +6.44%] |
| uncached hierarchy, 50% | 64 | **+9.31%** | [+6.72%, +11.30%] |
| uncached hierarchy, 100% | 32 | **+8.64%** | [+7.22%, +10.15%] |
| uncached hierarchy, 100% | 64 | **+8.00%** | [+6.53%, +9.39%] |

The same 240-sample environment was stable at 2.208 GHz and 45.307-46.230 C.
This second result confirms that missing generic profile weight, rather than the
specialized source itself, drives the regression: modest generic coverage cut
the loss roughly in half without sacrificing trained-path gains.

The original live-city trace performs about 18.7 million deep static-node visits
across its fixed 8,192 frames. The short generic cases traverse many instances
but very shallow trees and still contribute fewer relevant inner-loop edge
counts. Reject the default-weight corpus and raise each generic case's minimum
training time from 0.5 to 2 seconds. This approximately quadruples those counts;
the next acceptance run must again preserve the live-city/motion improvements
and remove the remaining control losses.

### Four-times-weighted screen and generated-code diagnosis

The two-second generic corpus collected 848-910 iterations of the 50% case and
508-511 of the 100% case for each payload64 library; payload32 collected 1,007
and 566 respectively. Four-cycle screen:
`/home/codex-perf/frontier/results/frontier-paired-20260817T155401Z`.

| Case | Payload | Paired change | 95% interval |
|---|---:|---:|---:|
| uncached hierarchy, 50% | 32 | **+7.39%** | [+6.64%, +8.50%] |
| uncached hierarchy, 50% | 64 | **+9.85%** | [+7.29%, +11.83%] |
| uncached hierarchy, 100% | 32 | **+10.15%** | [+7.39%, +14.05%] |
| uncached hierarchy, 100% | 64 | **+3.80%** | [+2.50%, +5.12%] |

More counts did not consistently improve the generic path. Symbol inspection
then exposed the compiler decision directly. In the frozen accepted PGO
baseline, `runSubtreeImpl<true, false>` is `0x7f8` bytes. The live-city-only,
default-balanced, and four-times-weighted retrains shrink it to `0x578`,
`0x5c8`, and `0x5c4` bytes respectively. Their complete executable text also
falls from 631,509 bytes to roughly 460-473 KiB. PGO continues to optimize the
fallback for size despite direct training, so corpus weight alone is not a
reliable contract.

Reject the weighted corpus as a complete solution. Mark the general walker
template `hot` for GCC/Clang so it remains speed-optimized regardless of whether
the currently selected realistic training scene reaches it. This is an
architectural statement, not benchmark-specific inlining: non-specializable
deep hierarchies genuinely depend on that fallback. Rebuild with the measured
corpus, verify restored generated-code size, and re-run paired controls before
acceptance.

The compiler-hint screen rejected that theory: GCC ignored the manual `hot`
attribute in the presence of profile feedback and emitted the identical `0x5c4`
symbol and 472,750-byte text segment. Remove the hint. The missing information
is not merely function-level hotness; it is the deep walk's branch and loop-edge
shape. The next corpus must exercise the generic walker with the same city depth,
boundary masks, and camera trajectory as the specialized realistic case.

The exact-byte dual-path corpus used a registration option to leave the static
hierarchy's zero-error bytes unchanged while declining only the specialized
mount flag. Its general trace was much faster than the nonzero-leaf approximation
(about 1,027 versus 1,185 us payload64 in generation mode), proving the intended
zero-error branches were restored. Nevertheless, screen report
`frontier-paired-20260817T162821Z` still regressed the four uncached controls by
9.27-18.57%. The generic symbol remained `0x57c`.

This rules out missing cases, insufficient counts, and mismatched leaf behavior.
The general walker is deliberately polymorphic: sparse/ready state, overlay
shape, hierarchy depth, zero-error masks, and LOD transitions produce mutually
different branch distributions. A single corpus is not a stable optimization
contract for that fallback. The next experiment excludes `runSubtreeImpl` from
profile instrumentation/feedback with GCC/Clang's
`no_profile_instrument_function` attribute. Specialized callers and the rest of
the realistic frame remain PGO-optimized; the fallback should retain ordinary
stable `-O3` code regardless of corpus composition.

The first isolation screen (`frontier-paired-20260817T163348Z`) expanded the
walker from `0x57c` to `0x720`, reduced the 50% losses to 1.92-2.00%, and turned
the 100% controls into 1.00-2.32% improvements. This proves feedback isolation
is the correct lever, but whole-program IPA still transforms the nominally
unprofiled function. Add `noipa` alongside `no_profile_instrument_function` to
make the fallback a self-contained ordinary-`O3` kernel; screen again before a
fresh strict PGO rebuild.

The `noipa` screen (`frontier-paired-20260817T163726Z`) did not improve the
remaining shape: 50% controls were still 1.29-2.71% slower, 100% payload32 was
0.94% slower/inconclusive, and only 100% payload64 remained faster. Reject
`noipa` and retain feedback isolation as the better intermediate. A true
translation-unit boundary is required if the fallback is to receive exactly
ordinary `-O3` code while the surrounding database remains LTO+PGO optimized.

Before extracting a translation unit, test the narrower remaining mechanism:
PGO use globally enables hot/cold block partitioning even for a function whose
own feedback is disabled. Apply a function-local
`no-reorder-blocks-and-partition` optimization attribute together with feedback
isolation. This should preserve the fallback as one speed-oriented body without
also disabling all IPA.

That flag emitted the byte-identical `0x720` walker, so reject and remove it
without another timing run. Inspection then found the missed boundary: the
outer driver delegates every SIMD block to the separately instantiated
`wideVisit` template, which still consumed PGO. Apply feedback isolation to both
the outer DFS driver and its wide inner kernel before considering a
translation-unit split.

The two-function isolation screen
(`frontier-paired-20260817T164600Z`) was worse: the 50% controls regressed
6.99-8.05% and the 100% controls regressed 2.21-3.96%. Explicitly adding
`unroll-loops` produced essentially identical code, so it was removed without a
redundant timing run. A fresh strict build then weighted the exact generic trace
four times more heavily than the specialized trace (32,768 versus 8,192 fixed
frames). The generic walker remained byte-identical at `0x57c`; absolute sample
count was not controlling GCC's decision.

Combining feedback isolation with an explicit `hot` classification did change
code generation, expanding `wideVisit<true, false, false>` from `0xc64` to
`0xcdc`. Timing still rejected it. Report
`frontier-paired-20260817T170541Z` measured 6.95-11.59% regressions in the 50%
controls and 1.67-5.70% in the 100% controls.

### Decision: reject PGO as a shipped optimization

No PGO candidate is accepted or committed. All experimental API switches,
training-only benchmarks, attributes, section/layout hints, and corpus changes
were removed. The evidence is stronger than a single failed corpus: several
semantically valid training mixes moved unrelated mounted-hierarchy throughput
by 5-22%, while attempts to stabilize generated code with compiler-specific
attributes also regressed it. Such a result would be sensitive to an embedding
application's link graph, workload mix, compiler version, and LTO decisions.

The accepted fully-refined traversal from Experiment 11 does not depend on any
of those mechanisms. Further work will use ordinary release builds and pursue
only algorithmic reductions, data layout, memory traffic, and stable explicit
specialization. PGO remains useful as a diagnostic tool, but it is not part of
the performance architecture or acceptance claim.

## Experiment 13: collapse fully-inside refined branches to leaf ranges

### Acceptance contract

This experiment starts from the current repository state after Experiment 12,
not from any PGO build. The frozen SBC baseline is an ordinary CMake `Release`
build at `/home/codex-perf/frontier/worktrees/restored/build-release-alg-base`
with both `FRONTIER_PGO_MODE=OFF` and `FRONTIER_IPO=OFF`. The candidate uses the
same options, compiler, source tree, CPU affinity, and benchmark executables.

The final implementation contains no profile data, LTO dependency,
compiler-specific optimization attribute, custom section, linker script,
function-order file, or separate-object placement optimization. An intermediate
screen did place the specialized helper in a normal second translation unit to
diagnose an unrelated-walker code-generation change. That separation was
removed before final acceptance; the final control and live-city reports below
come from the single ordinary `spatial_database.cpp` implementation.

### Theory

Experiment 11 proved once per eligible placement that every ordinary interior
node must refine. It consequently removed all descendant LOD-distance math, but
the boundary walker still visited every visible interior node, loaded every
wide AABB block, tested it against the frustum, and pushed/popped an explicit
DFS item. In the live city, most visible static blocks cross one or two frustum
planes only near their top. Once a descendant's propagated plane mask becomes
zero, every lower descendant is known visible. Continuing to test that branch
is redundant.

Move this repeated work to definition registration. For every eligible static
definition, precompute the terminal leaves under each interior node in exactly
the order the existing LIFO walker emits them. During selection, continue the
ordinary SIMD boundary walk only while a nonzero plane mask remains. At the
first fully-inside node, append its precomputed leaf range and stop touching
the descendant BVH.

### New definition-side data layout

`FullyRefinedLeafPlan` owns two contiguous arrays:

1. `terminalNodes`: 32-bit authored node indices in exact traversal/output
   order. Its capacity is reserved from the exact terminal count collected by
   the existing registration classification pass.
2. `ranges`: one 8-byte `{begin, count}` pair indexed directly by packed node
   index. Looking up a collapsed branch is therefore one indexed load followed
   by a sequential read from `terminalNodes`.

Registration constructs the arrays with an iterative enter/exit DFS. Leaves of
each wide block are appended in ascending lane order; interior lanes are pushed
in ascending order and therefore consumed in descending LIFO order, matching
the general walker byte-for-byte. The exit record closes the node's range after
all descendants have appended.

The plan store itself is lazy. `SpatialDatabase` contains only one nullable
pointer; the outer vector and all per-definition arrays are allocated only when
a definition has at least 16 authored nodes, no mountable nodes, zero-error
terminal leaves, and positive error on every ordinary interior node. Shallow
trees and ineligible application data allocate nothing. Released definition
slots clear their plan before reuse.

For one 1,365-node live-city static block, the logical arrays contain 1,024
terminal indices and 1,366 ranges: 4,096 + 10,928 = 15,024 bytes. Across the 83
static definitions in the benchmark this is about 1.19 MiB, plus roughly 6 KiB
of outer-vector capacity. This is definition-shared memory: it is not multiplied
by placements, cameras, queries, or frames. The previous immutable bytes remain
unchanged.

### Query-side architecture

The specialized query now has three phases:

1. Reuse the Experiment 11 farthest-corner proof to establish that LOD cannot
   stop at an interior node.
2. Traverse only the frustum boundary. A nonzero propagated plane mask uses the
   same SIMD wide AABB test and exact survivor order as before.
3. When the mask reaches zero, read `{begin, count}` and generate the complete
   `FrontierEntry` span directly from contiguous terminal indices.

`Sink::pushGenerated` supports the third phase. A growable result checks/grows
once and fills its final destination sequentially; a fixed caller span computes
the fitting prefix once and reports the exact dropped count. This removes one
capacity branch and one size update per emitted static leaf without changing
the public fixed-buffer overflow contract.

A second small specialization handles fully-ready definitions whose root
contains only leaves. It invokes the root wide visit directly and skips the
generic subtree DFS driver and empty-stack loop. It preserves mount transforms,
orientation, frustum masks, dependency recording, statistics, error encoding,
and exact output. This recovered 0.8-3.1% in the ordinary hierarchy controls
instead of treating their earlier movement as acceptable noise.

### Experiments and refinements

The first range-collapse screen used per-entry `Sink::push` and an always-live
parallel plan vector. Report `frontier-paired-20260817T172302Z` improved
live-city selection by 11.64% payload32 and 12.76% payload64, and render plus
submission by 13.08% and 9.11%, but moved payload32 hierarchy controls by as
much as +6.12%. Moving the cold member to the end of `SpatialDatabase` alone
did not repair that result (`frontier-paired-20260817T174027Z`). Reject those
layouts.

Generating a complete range after one destination resize was the important
second algorithmic reduction. Focused report
`frontier-paired-20260817T175049Z` improved live-city selection by 19.56% and
18.46%, and render plus submission by 17.22% and 14.04%. A six-cycle full screen
(`frontier-paired-20260817T180213Z`) retained 18.03%/17.95% selection and
17.44%/14.05% render gains, but one noisy payload64 hierarchy point estimate
remained positive.

Direct root-leaf dispatch made the hierarchy work smaller, but a ten-cycle
screen still found one +1.03% payload64 50%-hierarchy result
(`frontier-paired-20260817T183112Z`). Heap-layout inspection identified the
always-live parallel plan vector as the remaining avoidable footprint. Making
the store lazy produced improvements in all four controls in report
`frontier-paired-20260817T184143Z`. Finally, put the specialized helper back in
the ordinary source file and reserve exact terminal capacity, eliminating the
last code-placement dependency.

Final control report: `frontier-paired-20260817T185117Z`.

| Case | Payload | Baseline | Candidate | Paired change | 95% interval |
|---|---:|---:|---:|---:|---:|
| uncached hierarchy, 50% | 32 | 1648.524 us | 1631.677 us | **-0.82%** | [-1.30%, -0.27%] |
| uncached hierarchy, 50% | 64 | 1593.855 us | 1537.719 us | **-4.94%** | [-7.64%, -2.40%] |
| uncached hierarchy, 100% | 32 | 3013.130 us | 2945.160 us | **-2.45%** | [-2.99%, -1.75%] |
| uncached hierarchy, 100% | 64 | 2967.553 us | 2850.256 us | **-3.13%** | [-4.14%, -2.33%] |

All 96 samples ran on CPU 4 at 2.208 GHz and 45.307-46.230 C. Maximum
CPU-time/wall-time divergence was 0.027%. Payload64 50% retained a noisy
baseline CV, but all six paired cycles improved; the other three cases also
improved with tight intervals.

### Final live-city result

Final report: `frontier-paired-20260817T185416Z`.

| Case | Payload | Baseline | Candidate | Paired change | 95% interval |
|---|---:|---:|---:|---:|---:|
| live-city selection | 32 | 580.557 us | 473.262 us | **-18.13%** | [-19.45%, -16.35%] |
| live-city selection | 64 | 582.317 us | 475.773 us | **-18.11%** | [-19.19%, -17.23%] |
| render + submission | 32 | 584.259 us | 476.170 us | **-18.47%** | [-19.11%, -17.69%] |
| render + submission | 64 | 627.203 us | 535.546 us | **-14.50%** | [-14.68%, -14.33%] |
| motion-only | 32 | 169.397 us | 165.069 us | **-2.54%** | [-2.63%, -2.43%] |
| motion-only | 64 | 165.474 us | 165.881 us | +0.12% | [-0.11%, +0.43%] |

All 96 samples ran on CPU 4 at 2.208 GHz and 45.307-46.230 C. Maximum
CPU-time/wall-time divergence was 0.020%. Every live-city cycle improved by at
least 14.33%. Payload64 motion is statistically and practically unchanged; the
optimization does not execute in that motion-only timed region. The selection
speedup is approximately 1.22x on top of the current source baseline and comes
entirely from the algorithm and data layout described above.

### Correctness boundaries and tradeoffs

The fast path remains intentionally narrow. It does not run for small trees,
mountable definitions, nonzero-error terminal leaves, nonpositive-error
interiors, overlays, incomplete readiness, or a camera position that fails the
fully-refined proof. Every such case uses the existing general walker.

The trade is cold registration work and definition-shared memory for lower
per-frame traversal and result-construction work. Eligible definition
registration performs an additional iterative topology walk and allocates two
arrays. Dynamic placement/motion costs do not grow, and no per-query plan is
built. Because ranges store node indices rather than complete
`FrontierEntry` objects, instance slot, generation, and public instance id stay
runtime-correct while the immutable portion remains compact.

Exact output ordering matters to cache records and downstream consumers. The
dedicated boundary test compares the specialized path with an independently
forced general walk over a moving partial-frustum intersection; the complete
four-configuration matrix covers both BVH widths and 32-/64-bit payload ABIs.

The final Debug matrix passed 424/424 tests on the SBC: 106 tests each for
BVH4/payload64, BVH8/payload64, BVH4/payload32, and BVH8/payload32. This includes
the moving boundary equivalence test, randomized serial/parallel and readiness
torture tests, hot-layout contracts, and the new fixed-sink generated-range
overflow contract.

### Decision

Keep and commit. The final candidate meets the integration constraint: it is a
normal portable C++20 implementation with no build-system optimization mode or
link-layout assumption. The observed gain survives both payload ABIs, full
render submission, moving actors, and unrelated hierarchy controls. The cost is
the explicitly bounded, lazy, definition-shared acceleration memory and a cold
registration pass for eligible static definitions.
