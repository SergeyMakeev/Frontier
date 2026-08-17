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

## Experiment 14: batch large actor motion into one exact TLAS refit

### Portability constraint

This experiment follows the integration rule established after the rejected
PGO work. Baseline and candidate are ordinary GCC 13.3 CMake `Release` builds
with `FRONTIER_PGO_MODE=OFF` and `FRONTIER_IPO=OFF`. The implementation has no
profile corpus, compiler optimization attribute, custom text section, linker
ordering rule, function-order file, or target-specific intrinsic. The final
helpers are grouped with the TLAS maintenance code by responsibility; source or
symbol placement is not an acceptance mechanism.

An intermediate version deliberately placed the new publication helpers after
query traversal to see whether unrelated control movement was code-layout
noise. Another tried a special zero-error root kernel. Both ideas are rejected:
the former is not a portable architectural gain and the latter made controls
2-3% slower. The final report below uses ordinary code organization and records
the remaining unrelated control movement instead of tuning around it.

### Measured cost split and theory

The live-city frame has 1,191 top-level instances, of which 1,100 move every
frame: 100 cars plus 1,000 pedestrians. Their detailed 50- and 10-leaf actor
trees are mounted below those roots, so one actor transform changes the exact
world bound of one TLAS leaf rather than independently refitting every authored
part. Selection emits about 24,100 visible frontier entries in roughly 293
render runs per frame.

At the Experiment 13 baseline, payload64 spent approximately 165 us submitting
and publishing actor motion, 304 us selecting the cut after motion was
subtracted, and another 63 us resolving/scanning the render payload. Large-scale
motion was therefore the first remaining phase with a simple opportunity to
remove work.

Previously, every `moveInstanceDense()` immediately called
`tlasOnInstanceMoved()`. That operation loaded the instance's TLAS
back-pointer, examined and usually grew its leaf envelope, then walked parents
until an already-containing ancestor allowed an early exit. This is good for a
few movers: it touches only affected paths and bounded repeated motion often
fits an existing swept envelope. It is poor when 1,100 of 1,191 roots move:
the same shared ancestors are revisited through many scattered leaf-to-root
walks, and loose leaf envelopes add exact-bound retests to the following query.

The replacement is a density-dependent publication algorithm:

1. Motion submission validates and updates the exact dense `Instance` record,
   then appends its dense id to a flat pending stream. It does not mutate TLAS
   nodes immediately.
2. `applyUpdates()` first folds queued local-node deformation into final exact
   instance boxes. It then publishes actor motion, preserving correctness for a
   mixed "move actor and deform part" batch.
3. If fewer than one quarter as many submissions as TLAS leaves are pending,
   the existing grow-only per-instance algorithm runs unchanged.
4. At or above one quarter, publication traverses the complete compact TLAS
   once in bottom-up order. Leaf lanes copy exact bounds, error, and mask from
   dense `Instance` records and clear their loose flags. Interior lanes union
   their already-refitted children. The following selection therefore sees a
   tight tree with no mover-specific exact-bound retests.
5. Exact lane area is accumulated during the same pass. If topology quality is
   now outside the configured area-drift budget, the existing quality rebuild
   is scheduled; a refit never substitutes for a required topology rebuild.

The break-even rule intentionally counts submissions rather than unique ids.
Duplicates remain correct because exact instance state is last-write-wins; at
worst, duplicates choose the full streaming pass earlier. Building a hash set
or stamp array solely to deduplicate would add the random memory traffic this
algorithm is intended to remove.

### Data layout

No runtime record grows and no new allocation is introduced. The algorithm
uses data already present for rebuilds:

- `Instance` remains 80 bytes and is the authoritative exact stream for world
  bound, root error, layer mask, transform, orientation-side index, and TLAS
  back-pointer.
- `TlasNode` remains the query-hot wide SoA record containing bound lanes,
  child references, valid flags, and parent index. `TlasMeta` remains the cold
  parallel stream for maximum error and layer masks.
- `instanceTlasLoose_` remains one byte per dense instance. The exact pass
  clears it as each leaf lane is rewritten.
- `tlasItemsTmp_`, an existing 32-bit rebuild scratch array, holds pending
  dense mover ids between submission and publication. During the exact refit
  it is reused as the iterative DFS stack.
- `tlasLevelTmp_`, another existing 32-bit rebuild scratch array, retains the
  node postorder after its first construction. Subsequent large-motion frames
  stream that order directly. Insert, remove, or rebuild invalidates it.

Build, publication, and read-only selection are already mutually exclusive
under the database writer/read barrier, so these scratch roles cannot overlap.
Capacity was already retained by TLAS construction; keeping the postorder's
logical size live does not increase allocated capacity. The tradeoff is a more
explicit lifetime invariant for the two scratch arrays, covered by structural
edit and exact-refit tests.

### Experiment sequence

The first implementation used two new always-live vectors. It already improved
motion by about 29% (`frontier-paired-20260817T192146Z`) and live-city frames by
18-20% (`frontier-paired-20260817T192411Z`), proving the algorithmic theory, but
the extra object layout and allocations were unnecessary. Reusing rebuild
scratch removed both.

Several control screens then exposed the very integration concern that makes
code-placement tuning unacceptable. Moving the helper within
`spatial_database.cpp` changed non-executing hierarchy controls by around one
percent, and a zero-error root dispatch intended to compensate made them worse.
Neither is kept or credited. The final source keeps only the batching policy,
exact streaming refit, scratch reuse, and correctness support.

### Final SBC result

Final report:
`/home/codex-perf/frontier/results/frontier-paired-20260817T201417Z`.
It compares the exact final source with the frozen ordinary-Release `5397f59`
build over four ABBA cycles per case and payload ABI.

| Case | Payload | Baseline | Candidate | Paired change | 95% interval |
|---|---:|---:|---:|---:|---:|
| live-city selection | 32 | 479.467 us | 393.991 us | **-17.22%** | [-18.25%, -16.28%] |
| live-city selection | 64 | 476.822 us | 401.717 us | **-15.93%** | [-16.44%, -15.42%] |
| render + submission | 32 | 478.983 us | 402.987 us | **-15.60%** | [-16.14%, -15.07%] |
| render + submission | 64 | 539.661 us | 459.392 us | **-14.99%** | [-15.80%, -14.55%] |
| motion-only | 32 | 165.261 us | 117.800 us | **-28.64%** | [-28.91%, -28.29%] |
| motion-only | 64 | 165.754 us | 119.285 us | **-28.21%** | [-28.44%, -27.99%] |
| uncached hierarchy, 50% | 32 | 1596.615 us | 1596.544 us | -1.55% | [-3.87%, +0.83%] |
| uncached hierarchy, 50% | 64 | 1539.487 us | 1541.749 us | +0.50% | [-0.71%, +1.82%] |
| uncached hierarchy, 100% | 32 | 2868.922 us | 2931.242 us | +2.15% | [+1.68%, +2.79%] |
| uncached hierarchy, 100% | 64 | 2908.943 us | 2946.344 us | +1.08% | [+0.58%, +1.63%] |

All 160 samples ran on CPU 4 at exactly 2.208 GHz. Temperature stayed between
45.307 and 46.230 C, one-minute load between 1.000 and 1.297, and maximum
CPU-time/wall-time divergence was 0.028%. Every motion and live-city cycle
improved. Motion coefficients of variation were 0.13-0.52%.

The 100%-hierarchy control does not submit motion and cannot execute the new
publication path. Its 1.08-2.15% change is nevertheless reported as a real
binary-code-generation sensitivity, not relabeled as an algorithmic result.
Chasing it through section placement, source reordering, `hot`/`cold`
attributes, or a profile-trained build would recreate the fragile optimization
being excluded. It remains a cross-compiler/integration measurement concern;
the accepted claim is limited to the executing moving-scene paths above.

### Correctness and tradeoffs

`Motion.LargeMotionBatchPublishesOneExactTlasRefit` creates 64 roots, optimizes
their TLAS, moves the complete cohort, publishes once, and verifies every leaf
lane equals its authoritative `Instance.worldBox` and every loose flag is
clear. The complete Debug matrix passes 428/428 tests across BVH4, BVH8,
payload32, and payload64, including randomized TLAS churn, duplicate/stale
motion groups, global-offset materialization, deformation, and concurrent
read-only queries.

The API timing changes deliberately: actor transform submission no longer
writes the TLAS immediately. Exact instance state is updated during submission,
but callers must reach `applyUpdates()` before selection, which was already the
documented publication contract. `optimize()` consumes pending actor state by
rebuilding directly from exact instances.

For sparse motion, the old grow-only path remains the right algorithm and is
retained. For dense motion, publication becomes O(TLAS nodes) even if many
movers happened to remain inside old envelopes; the one-quarter threshold is a
measured fixed policy rather than a universally optimal constant. The gain is
largest when a large fraction of roots move independently. A complete cohort
sharing one translation should still use `translateInstances()`, whose global
offset is O(1) and strictly cheaper.

### Decision

Keep and commit. The result is an algorithm/data-layout improvement: it changes
the unit of work from scattered per-actor ancestor walks to one sequential
topology pass and reuses existing dense streams and scratch memory. It provides
about a 1.40x motion-phase speedup and a 1.18-1.21x full moving-city speedup on
the SBC without PGO, LTO, custom sections, compiler hints, or link-layout
assumptions.

## Experiment 15: propagate exact-refit summaries into parents

### Theory and implementation

Experiment 14's exact bottom-up pass visits every node lane, and an interior
child is scanned again by `tlasNodeExtent()` when its parent lane is rebuilt.
The candidate accumulated each node's bound, maximum error, and layer mask
while visiting its lanes, then wrote that summary directly into the matching
parent lane. Child-before-parent postorder made the parent lane exact before
the parent was processed. This removed the second child-lane scan without new
state, API changes, build flags, or data-layout growth.

The focused six-cycle motion report
`frontier-paired-20260817T203534Z` confirmed a real executing-path gain:
payload32 improved 2.28% with a 95% interval of [-2.48%, -2.04%], and payload64
improved 1.63% with an interval of [-1.81%, -1.46%].

### Full gate and decision

The full report is
`/home/codex-perf/frontier/results/frontier-paired-20260817T203800Z`.
Motion improved 2.99% payload32 and 2.15% payload64. Full render frames improved
2.28% and 3.38%; handle-returning selection improved 1.00% payload32, while
payload64 was inconclusive at -1.25% with an interval reaching +0.01%.

The unrelated payload32 100%-hierarchy control regressed 4.85%, interval
[+3.98%, +6.05%]. That benchmark submits no actor motion and cannot execute the
changed loop. The result is generated-code/layout coupling, but it is still a
real regression in the produced library binary. Fixing it by relocating the
helper, forcing inline/out-of-line decisions, or adding compiler/linker layout
hints would violate the portability constraint and would not survive an
embedding application's link graph.

**Reject and revert.** The small algorithmic motion gain does not justify a
larger measured selection regression, and no code-placement compensation will
be pursued. Both binaries used ordinary Release with PGO and IPO disabled; all
160 full-gate samples ran at 2.208 GHz between 44.384 and 47.153 C.

## Experiment 16: share immutable actor payload ranges

### Theory

The live-city render query resolves about 24,100 logical leaves even though the
5,000 car-detail leaves come from 100 placements of one immutable 50-leaf
definition and the 10,000 pedestrian-part leaves come from 1,000 placements of
one immutable 10-leaf definition. The renderer still needs one instance id per
actor, but it does not need a private copy of identical payload and zero-error
bytes for every placement.

The proposed representation made a render run either:

- an offset/count into the query-owned resolved payload and error streams; or
- a direct pointer/count into an immutable definition payload stream plus one
  constant error byte.

`RenderFrontierSpan::errorCode(i)` abstracted the two representations. Eligible
actors had to be explicitly `renderAsUnit`, completely ready, overlay-free,
root-leaves-only, free of nested mount points, and composed entirely of
zero-error leaves. Their root still ran the normal screen-error decision. Every
other tree and the exact handle-returning API retained the existing cached
walker. Definition registration encoded the narrow eligibility fact in an
existing cold runtime float; it did not add a per-instance allocation.

This would have removed per-placement cut caching and payload/error resolution
for the repeated car and pedestrian definitions while downstream submission
continued to scan every logical leaf. Therefore any accepted gain would have
represented less library work, not a weakened benchmark consumer.

### Proposed data-layout trade

The immutable definition already owns its packed payload array for as long as a
mounted placement can be selected, so direct ranges required no new payload
storage. The proposed run descriptor grew from 12 to 24 bytes on the 64-bit
ABI to hold a pointer and constant error. At roughly 293 live-city runs this is
about 3.5 KiB of additional transient descriptor bytes, exchanged for avoiding
15,000 repeated actor payload values, 15,000 error bytes, and their cache/write
traffic. Direct pointers would remain valid only for the published database
snapshot and query-view lifetime; this was an intentional API/data-layout
change, not a backwards-compatibility shim.

### Experiment sequence

The first prototype placed the direct-run recognition inline in the generic
cached selector. Focused reports showed a promising render improvement:
`frontier-paired-20260817T205434Z` measured -3.09% payload32 and -6.99%
payload64. The broader report `frontier-paired-20260817T210134Z` measured
-3.14% and -6.51%, but also moved the unrelated payload32 100%-hierarchy
control by +1.53%.

A second version removed eager fully-refined plan allocation for eligible flat
definitions. Report `frontier-paired-20260817T211502Z` improved render by
3.95% payload32 and 6.33% payload64, but the same non-executing control
regressed 4.60%. The allocation reduction was real cold-state cleanup, but it
did not make the produced binary robust.

Next, renderer-only recognition moved to an ordinary responsibility-specific
translation unit. This used no section name, ordering rule, inline/noinline
attribute, profile, IPO, or target-specific compiler mechanism. Six-cycle ABBA
report `/home/codex-perf/frontier/results/frontier-paired-20260817T212716Z`
measured:

| Case | Payload | Baseline | Candidate | Paired change | 95% interval |
|---|---:|---:|---:|---:|---:|
| live-city render | 32 | 407.044 us | 397.139 us | **-2.01%** | [-2.78%, -1.02%] |
| live-city render | 64 | 457.798 us | 439.053 us | **-3.76%** | [-4.50%, -2.99%] |
| uncached hierarchy, 100% | 32 | 2896.937 us | 3033.830 us | **+4.30%** | [+3.81%, +4.70%] |
| uncached hierarchy, 100% | 64 | 2921.075 us | 2907.912 us | -0.23% | [-1.13%, +0.73%] |

All 96 samples ran at 2.208 GHz between 45.307 and 46.230 C; maximum
CPU-time/wall-time divergence was 0.027%. The payload32 regression is much
larger than both the render gain and measurement variation.

Finally, the complete cached selector was specialized into compile-time exact
and segmented-render modes. The exact specialization contained no direct-range
branch and no runtime segmented-render conditions, while only the render
specialization called the new helper. This was a legitimate algorithmic path
split rather than a layout directive, but it generated a second copy of a very
large selector. The decisive control-only report
`/home/codex-perf/frontier/results/frontier-paired-20260817T213843Z` became
worse: payload32 regressed 4.65%, interval [+4.03%, +5.18%], and payload64
regressed 3.07%, interval [+1.92%, +4.29%]. All 48 samples again ran at exactly
2.208 GHz between 45.307 and 46.230 C.

### Decision and architectural lesson

**Reject and revert every implementation and API change.** The direct-range
idea reduces work on its intended path, but the current integration changes the
generated library binary enough to cause a larger, repeatable regression in an
unrelated hot traversal. Growing the public run record also imposes a permanent
cost on every scene, including those with no shareable actor definitions.

No PGO corpus, custom text section, source-order tuning, linker order file,
`hot`/`cold` annotation, forced inline decision, or function placement will be
used to turn this into an accepted win. Such a result would depend on the
standalone benchmark's link graph and could disappear as soon as the static
library is embedded in a real renderer.

Future work should preserve the ordinary-Release acceptance rule and pursue a
layout whose benefit dominates binary perturbations—for example, a renderer
output format selected at query construction whose compact descriptor layout
does not enlarge the common run, or an actor-definition instance stream
consumed directly by a batch renderer. Either design must pass unrelated
payload32 and payload64 hierarchy controls without placement compensation.

## Experiment 17: rigid actor SoA motion publication

### Hardware-guided diagnosis

The accepted `ebfec62` source was rebuilt on the Cortex-A72 with `-O3 -pg` only
for diagnostic sampling; acceptance binaries remained ordinary uninstrumented
CMake `Release` with `FRONTIER_PGO_MODE=OFF` and `FRONTIER_IPO=OFF`. Across the
complete 8,192-frame trajectory, gprof attributed 37.85% of motion-only samples
to 18,022,400 calls of `moveInstanceDense`, 22.43% to `tlasRefitAllExact`, and
16.36% to child extent scans. In the render frame, actor submission still
accounted for 20.88% while payload resolution used 12.58% and fully-refined
static boundary emission used 23.29%.

The live city submits 1,100 actors every frame. Their roots retain scale 1,
carry `FlagYawInvariantBounds`, and change only translation plus yaw, yet the
general 32-byte `InstanceTransform` path loaded scale and repeatedly branched
through unchanged, identity-yaw, scale-change, oriented-bound, overflow, and
frontier-invalidation cases. The profile showed that this flexibility had
become more expensive than the exact TLAS refit itself.

### New API and data layout

`RigidMotionGroup` is a persistent cohort specialized for stable-scale rigid
motion. Callers provide two SoA spans: 16-byte positions and 8-byte
`YawRotation` pairs. The group retains the caller-order handles and the same
dense-sorted `{dense, source}` mapping as `MotionGroup`. Once per mapping it
also proves whether every live member owns an authored yaw-invariant root
envelope.

For an eligible group, each iteration performs only the necessary state change:

1. load the next position and yaw through the dense-sorted source map;
2. translate the existing exact world AABB by the position delta;
3. accumulate translation plus conservative yaw-chord travel;
4. update the 80-byte dense `Instance`, the parallel 36-byte orientation
   record, and the pending dense-id stream.

Scale, maximum error, local bounds, and the broadphase envelope shape remain
unchanged and are not recomputed. Non-invariant groups call the existing exact
general transform implementation, so the API changes representation and fast
path selection without weakening geometry or cache validity.

The group-level eligibility word lives in the cold `RigidMotionGroup`, not the
80-byte per-instance record. The live-city benchmark now produces positions and
yaws directly in SoA form, so both generation and consumption of the new layout
remain inside the timed frame. No conversion pass or work was moved outside the
benchmark.

### Integration isolation

The first prototype placed the kernel in `spatial_database.cpp`. Six-cycle
report `/home/codex-perf/frontier/results/frontier-paired-20260817T215642Z`
measured 31.14-31.63% faster motion and 9.64-9.80% faster full selection. The
render/control report `frontier-paired-20260817T220335Z` measured 7.33-9.46%
faster render frames, but the non-executing payload32 hierarchy controls moved
by +1.78% and +4.52%.

Disassembly showed the generic selector retained exactly the same 0x377c-byte
instruction structure; only relocated literal/call addresses differed. Rather
than tune those addresses, the final architecture moved rigid publication into
the ordinary `src/rigid_motion.cpp` archive member and removed every change from
`spatial_database.cpp`. SHA-256 then proved the complete generic spatial object
byte-identical between frozen baseline and candidate:

- payload32: `ebd0d52fb0ee1ad086873eec87141119132f6784adf645c28efe78f1219367b3`;
- payload64: `37bfc27752c659f569478177f1dfca8386a475e38543fb1b0d8d8d713cce081a`.

This is normal static-library decomposition, not code placement as an
optimization. A consumer that does not call `moveRigidInstances` does not
extract the new archive member and receives byte-for-byte existing generic
code. A consumer that does call it gets the new algorithm without section
names, linker scripts, source-order acceptance, PGO, IPO, or compiler
attributes. Mixed benchmark-executable control timings are therefore recorded
as link-layout noise rather than manipulated into a favorable result.

### Final SBC result

Final executing-path report:
`/home/codex-perf/frontier/results/frontier-paired-20260817T221827Z`. It compares
the isolated-object candidate against frozen ordinary-Release `ebfec62` over
four ABBA cycles per payload ABI.

| Case | Payload | Baseline | Candidate | Paired change | 95% interval |
|---|---:|---:|---:|---:|---:|
| render + submission | 32 | 403.599 us | 361.783 us | **-10.46%** | [-11.83%, -9.46%] |
| render + submission | 64 | 459.419 us | 418.873 us | **-8.61%** | [-9.00%, -8.22%] |
| motion-only | 32 | 117.701 us | 81.461 us | **-30.79%** | [-30.89%, -30.65%] |
| motion-only | 64 | 118.058 us | 82.013 us | **-30.47%** | [-30.62%, -30.32%] |

Every cycle improved. All 64 processes ran on CPU 4 at exactly 2.208 GHz;
temperature stayed between 45.307 and 46.230 C and maximum CPU-time/wall-time
divergence was 0.021%. The earlier six-cycle full-selection report also measured
9.64% payload32 and 9.80% payload64 improvements with every cycle faster.

### Correctness and tradeoffs

The complete Debug matrix passes 436/436 tests across BVH4, BVH8, payload32,
and payload64. New tests verify exact positions and stored yaws for the
yaw-invariant streaming path and exact rotated bounds through the non-invariant
fallback. Existing randomized motion, dense-slot reuse, duplicate/stale group,
deformation, TLAS, cache, and concurrent-query tests remain in the matrix.

The primary tradeoff is API specialization: callers must retain separate
position and yaw arrays plus a `RigidMotionGroup`. It does not support scale
changes; scale remains whatever each instance already owns. Mixed groups are
correct but receive no specialized-kernel benefit. The SoA path is best for
large, stable actor cohorts; sparse or heterogeneous edits should continue to
use scalar `moveInstance` or general `MotionGroup` submission.

### Decision

Keep and commit. This is an algorithm/data-layout improvement with a direct ARM
benefit: it replaces 1,100 entries through a branch-heavy general transform
routine with sequential SoA publication of only the state that actually
changes. It provides approximately 1.44x motion-phase throughput and 1.09-1.12x
complete render-frame throughput on top of the already optimized exact-refit
baseline, while preserving byte-identical generic library code for non-users.
