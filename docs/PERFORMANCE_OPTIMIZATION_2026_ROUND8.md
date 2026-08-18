# Performance optimization round 8: render-native cut resolution

Date: 2026-08-17

## Baseline and acceptance policy

This round starts from repository commit `a8303c8`. That revision already
contains the isolated realistic live-city render-submission benchmark and the
paired Cortex-A72 acceptance harness. Each experiment is compared with its
immediately preceding accepted commit, while `a8303c8` remains the cumulative
anchor. Runs use ordinary CMake `Release` binaries on CPU 4 of the SBC at a
fixed observed 2.208 GHz, with PGO and project IPO disabled. A change is kept
only when a fresh-process paired run shows a practically meaningful,
repeatable improvement without material regressions in the selection, motion,
or machine-control cases.

The acceptance scope is deliberately limited to algorithms and data layouts.
PGO, linker scripts, named hot/cold sections, source or archive ordering,
forced inlining, and favorable address placement are not accepted optimization
mechanisms. Historical layout and PGO experiments remain below as negative
evidence, but their results are neither part of the cumulative speedup nor a
dependency of any kept implementation.

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

## Experiment 18: terminal payload-range frontier

### Post-motion profile and theory

Commit `e292abf` was rebuilt on the Cortex-A72 with `-O3 -pg` for diagnosis
only. The acceptance path remained ordinary uninstrumented CMake `Release`
with PGO and IPO disabled. Selection report directory
`/home/codex-perf/frontier/results/gprof-e292abf-selection-20260817` measured
24,072.7 logical entries per frame and attributed 37.45% of samples to the
fully-refined boundary emitter. The emitter's inlined range generator still
constructed a 12-byte `FrontierEntry` for every inside terminal leaf. Rigid SoA
publication was now 17.98%, exact TLAS refit 13.86%, and child extent scans
8.24%.

The render profile in
`/home/codex-perf/frontier/results/gprof-e292abf-render-20260817` measured
24,139.3 submitted leaves per frame. Fully-refined entry construction used
26.18% of samples and `resolveRenderLeaves` another 15.14%. The renderer then
scanned the same payloads a third time. This made output representation, not
tree search, the dominant opportunity: an inside subtree's terminal payloads
are immutable and consecutive in definition traversal order, while instance id
and zero error are constant for the whole sequence.

The theory was to change the output unit from a leaf record to a referenced
definition range. This cannot make downstream submission disappear: the live
benchmark must still load and checksum every logical payload. It can remove the
intermediate per-leaf handle construction, handle validation, payload lookup,
resolved-payload copy, and repeated instance/error words.

### API and data layout

`TerminalRenderQuery` is an explicit max-detail query for fully resident
content. On a 64-bit target it returns a span of 16-byte
`TerminalRenderRun` records:

```
{ const UserPayload* payloads; uint32_t count; uint32_t instanceAndError; }
```

The packed word is identical in meaning to `FrontierEntry`'s instance/error
word, but occurs once per range. `TerminalRenderView::size()` remains the
logical leaf count; `segmentCount()` exposes the physical run count.

Each query lazily builds one definition plan outside the timed steady state.
The plan owns decoded terminal payloads in the exact ordinary DFS order and one
8-byte `{begin,count}` range per packed node. An inside branch appends its range
without visiting descendants. A partial branch retains the same wide frustum
tests and emits one-element terminal ranges at its boundary. Terminal nodes use
their own one-element range as the node-to-payload index, avoiding a duplicate
32-bit mapping array; that final layout removed roughly 450 KiB from the
live-city prototype.

Exact terminal selection passes `coarsenRenderUnits=false`, preserving
descendant frustum culling and the existing 24,072.7-entry cut. Renderer output
uses the established actor-unit policy: a visible car or pedestrian root keeps
the actor's small terminal range whole for GPU clipping, matching the existing
24,139.3-leaf render cut. Both benchmark warmups compare range and ordinary
result cardinality before timing, and focused tests compare the full sorted
payload set at a moving frustum boundary.

The implementation lives in `src/terminal_render.cpp`, an ordinary static
archive member. It does not alter the general selector implementation. This is
representation and algorithm specialization, with no PGO, IPO, section name,
linker script, source-order requirement, forced inline decision, or target
intrinsic.

### Correctness contract and tradeoffs

The narrow contract is the source of the saving. Every selected mounted tree
must be fully ready, overlay-free, contain no nested mounts, and terminate in
zero-error leaves. The query emits the max-detail terminal cut rather than
performing general geometric-error LOD. TLAS-only roots remain legal one-entry
ranges. Streaming readiness, nested topology, nonzero terminal error, and
deformation continue to use `SpatialQuery`.

Payload pointers and runs are query-owned views valid until the next selection
or reset, or a database mutation. The plan trades cold per-definition payload
and node-range storage for far less per-frame output traffic. In the live city,
the first prototype retained about 1.686 MiB before removal of the duplicate
node mapping, versus about 1.224 MiB for handle selection and 2.253 MiB for the
old renderer cache.

Two focused Debug tests passed on ARM before performance measurement:
`TerminalRenderRangesMatchTheFullyRefinedCurrentCut` checks the complete
payload set and actual range compression at a partial frustum boundary;
`TerminalRenderRejectsNonzeroTerminalError` checks the specialized contract.

### First executing-path observation

An ordinary-Release payload32 smoke run of the first layout measured 223 us for
terminal range selection and 254 us while scanning all 24,139 render payloads.
After separating exact descendant culling from renderer actor-unit policy, the
exact 24,072.7-leaf selection measured 186 us. These were unpaired directional
observations under the scaling warning, not acceptance numbers. A controlled
ABBA comparison against an isolated `e292abf` source snapshot was started next;
the final no-duplicate-map layout must be rebuilt and gated again before a keep
decision.

### Prototype paired result

The first controlled report,
`/home/codex-perf/frontier/results/frontier-paired-20260817T230831Z`, still
contained the redundant 32-bit terminal-node map. Against isolated ordinary
Release commit `e292abf`, exact selection improved by 46.67% payload32 and
46.31% payload64; render plus complete payload scanning improved by 37.11% and
39.54%. The motion-only case was flat. This established that the saving came
from the output algorithm rather than the later memory reduction, but this
prototype is not the accepted implementation.

### Final SBC result

The duplicate-map-free candidate was rebuilt from source and compared with the
same frozen `e292abf` build in report
`/home/codex-perf/frontier/results/frontier-paired-20260817T232701Z`. Four ABBA
cycles and two samples per revision/cycle produced:

| Case | Payload | Baseline | Candidate | Paired change | 95% interval |
|---|---:|---:|---:|---:|---:|
| exact live-city selection | 32 | 357.050 us | 188.766 us | **-46.99%** | [-47.72%, -46.12%] |
| exact live-city selection | 64 | 358.371 us | 183.736 us | **-48.24%** | [-49.41%, -46.68%] |
| render + payload scan | 32 | 364.204 us | 222.023 us | **-38.82%** | [-40.30%, -37.03%] |
| render + payload scan | 64 | 420.997 us | 246.070 us | **-40.54%** | [-41.63%, -39.36%] |
| motion only | 32 | 81.677 us | 81.961 us | +0.64% | [-0.11%, +1.80%] |
| motion only | 64 | 81.748 us | 81.803 us | -0.00% | [-0.40%, +0.35%] |

Every selection and render cycle improved. All 160 processes observed exactly
2.208 GHz; temperature stayed between 44.384 and 47.153 C, maximum CPU/wall
divergence was 0.032%, and system load stayed between 0.778 and 1.158. The
final query retained 1,241.26 KiB for payload32 and 1,573.49 KiB for payload64,
versus roughly 1.686 MiB for the first payload32 prototype. It represented the
average 24,072.7-entry exact cut with 461.6 physical segments and the
24,139.3-entry renderer cut with 460.0 segments.

The payload32 `identity_100` control's paired point estimate was +0.86%, with
a [+0.15%, +1.72%] interval, while the other controls were inconclusive around
zero. This is not attributable to candidate code: SHA-256 proved the complete
machine-control executables byte-identical at
`0f77d52bade50770c185b35e6f275d7a4f28b083e4aacf471f91726150c50845`.
The generic payload32 and payload64 `spatial_database.cpp` objects were also
byte-identical to `e292abf`, at `ebd0d52f...9367b3` and
`37bfc277...e081a` respectively. The control movement is therefore retained
as measurement noise instead of being addressed through link-layout tuning.

### Correctness and decision

The complete Debug matrix passes 444/444 tests on both x64 Windows and the
Cortex-A72, across BVH4, BVH8, payload32, and payload64. In addition to exact
boundary payload-set comparison and contract rejection, the actor-unit test
now compares both coarsened and exact terminal-range output directly with the
established renderer and handle cuts. The run-size assertion is expressed as a
pointer plus two 32-bit words, so the API does not impose an accidental
64-bit-only build restriction.

Keep and commit. This is a direct data-representation improvement: the
renderer still observes every selected payload, instance id, and error code,
but traversal communicates immutable contiguous spans instead of manufacturing
and resolving one transient handle record per leaf.

Direct cumulative report
`/home/codex-perf/frontier/results/frontier-paired-20260817T235041Z` rebuilt both
round-start `a8303c8` and committed `1c72a99` as ordinary non-IPO Release and
ran four ABBA cycles. Exact selection improved 73.07% payload32 and 73.15%
payload64, while render plus complete payload scanning improved 74.67% and
73.89%. The corresponding direct speedups are approximately 3.71-3.72x for
selection and 3.83-3.95x for end-to-end rendering. All 64 processes observed
2.208 GHz, temperature stayed between 43.461 and 46.230 C, and maximum
CPU/wall divergence was 0.031%. This direct result supersedes the larger
cross-report median ratio previously estimated from differently configured
historical binaries.

### Next radical experiment

Motion publication and exact TLAS refit now consume roughly 82 us of a 222-246
us frame. Homogeneous cars and pedestrians already exist in simulation-owned
SoA arrays, yet the current API copies them into general per-instance records
and rebuilds an acceleration structure every frame. The next experiment will
let a terminal query consume immutable-definition actor batches directly from
caller-owned position/yaw spans. Static roots remain in the TLAS; homogeneous
moving batches receive wide root culling and only partial actors enter local
hierarchy traversal. This is an algorithm/data-layout change with an explicit
O(actor-count) per-view tradeoff, and should remove duplicate transform
publication plus dynamic TLAS refit without changing the simulated scene.

## Experiment 19: simulation-owned terminal actor batches

### Theory

The live city has two homogeneous moving cohorts: 100 cars share one immutable
50-leaf definition and 1,000 pedestrians share one immutable 10-leaf
definition. Their current positions and yaws already exist in simulation-owned
SoA arrays. The general instance architecture copied those 1,100 transforms
into dense database records, updated orientation state, accumulated exact TLAS
leaf bounds, and refit affected parents before selection could read them. That
publication remained about 82 us of the 222-246 us terminal-range frame.

For populations of this size, rebuilding/refitting a dynamic acceleration
structure can cost more than directly testing the roots. Keep the 91 static
world roots in the quality TLAS, but describe each homogeneous dynamic cohort
as a non-owning `TerminalInstanceBatch`:

```
{ definition, localBounds, positions[], yaws[], firstInstance,
  scale, mask, yawInvariantBounds, renderAsUnit }
```

The batch does not copy or own transforms. `TerminalRenderQuery` first selects
static/general roots through the unchanged TLAS, then root-culls each batch
placement directly from the caller spans. Fully inside or renderer-coarsened
actors append the definition's terminal range immediately; only partial actors
build a local camera and enter definition traversal. This changes the dynamic
broadphase from per-frame TLAS refit plus logarithmic query to a simple
O(actor-count) scan per view, while eliminating a complete duplicate transform
publication phase.

### Prototype and correctness

The first implementation adds a public batch overload without changing the
existing query. A database with no TLAS roots can own definitions solely for
batch use. Contracts validate definition generation, bounds, scale, transform
counts, yaw values, view mask, terminal eligibility, and the 24-bit output id
range. Batch ids are caller-assigned and must not collide with normal instance
ids used in the same result.

The realistic benchmark constructs the identical static hierarchy and actor
definitions but does not instantiate the 1,100 actors in the general database.
It still generates every position and yaw inside the timed frame. Selection
reads those spans directly; the motion-only companion measures the complete
writer cost, with `ClobberMemory` keeping the generated arrays observable. A
separate legacy scene outside the timer verifies the frame-zero exact/render
cardinality before measurement.

The complete local Debug matrix passes 448/448 tests across BVH4, BVH8,
payload32, and payload64. The new focused test moves a yawed mounted instance
across a partial frustum boundary and compares its exact and actor-unit terminal
payload sets with a batch-only database; it also verifies the caller-assigned
instance id. The full 8,192-frame local benchmark retains exactly 24,072.7
selected and 24,139.3 submitted leaves per frame, including the same minima and
maxima. Query storage falls slightly because dynamic TLAS scratch disappears;
orientation storage falls to zero and retained mounted-instance state drops by
about 193 KiB in the local configuration.

Local Release timings are directional only and show a large reduction in all
three live-city cases. No result is accepted until ordinary non-IPO Release
binaries pass the paired Cortex-A72 gate against committed `1c72a99`.

### Final SBC result

Report `/home/codex-perf/frontier/results/frontier-paired-20260818T001202Z`
compares the actor-batch candidate with a clean bundle/build of committed
`1c72a99`. Both are ordinary non-IPO Release builds with contract checks and
subtree revalidation disabled equally. Four ABBA cycles produced:

| Case | Payload | Baseline | Candidate | Paired change | 95% interval |
|---|---:|---:|---:|---:|---:|
| exact live-city selection | 32 | 186.263 us | 103.523 us | **-44.69%** | [-45.61%, -44.03%] |
| exact live-city selection | 64 | 184.403 us | 103.204 us | **-44.24%** | [-45.39%, -43.15%] |
| render + payload scan | 32 | 232.202 us | 129.836 us | **-44.83%** | [-47.72%, -43.16%] |
| render + payload scan | 64 | 257.124 us | 125.824 us | **-51.83%** | [-53.22%, -50.40%] |
| moving-object writer | 32 | 81.568 us | 8.171 us | **-89.99%** | [-90.06%, -89.94%] |
| moving-object writer | 64 | 81.660 us | 8.196 us | **-89.98%** | [-90.02%, -89.94%] |

Every executing-path cycle improved. Candidate CV was 0.15-0.24% for selection
and payload32 render, 0.86% for payload64 render, and below 0.5% for motion.
All 160 processes observed exactly 2.208 GHz; temperature stayed between
45.307 and 46.230 C and maximum CPU/wall divergence was 0.044%.

The identity controls reported mixed point estimates, including positive
`identity_100` intervals. They cannot be candidate regressions: baseline and
candidate machine-control executables are byte-identical at SHA-256
`0f77d52bade50770c185b35e6f275d7a4f28b083e4aacf471f91726150c50845`.
The general payload32 and payload64 `spatial_database.cpp` objects are likewise
byte-identical at the hashes recorded in Experiment 18. No address/layout
tuning was attempted; these contradictory timings remain environment noise.

The Cortex-A72 smoke counters retained the exact 8,192-frame leaf counts and
extrema. Query storage was 1,239.48 KiB payload32, orientation state was zero,
and mounted-instance state fell from 430.891 to 243.238 KiB. The static 91-root
TLAS remains a normal quality hierarchy; 1,100 actor roots now live only in the
two simulation spans.

### Decision and cumulative position

Keep and commit. This is the intended radical architecture change: simulation
state becomes the single source of truth for homogeneous actor transforms,
while immutable hierarchy/payload data remains shared through the database.
It exchanges general handles, per-actor scale/mask, readiness/deformation, and
sublinear dynamic broadphase queries for zero-copy publication and a cheap flat
root scan. Mixed scenes can use batches and general instances in the same
terminal query, so the tradeoff is chosen per cohort rather than globally.

Final direct frozen-anchor report
`/home/codex-perf/frontier/results/frontier-paired-20260818T001945Z` compared
round-start `a8303c8` directly with committed candidate `1cc5c21`. Both were
ordinary non-IPO CMake Release builds; neither used PGO, linker/source/archive
reordering, hot/cold sections, forced inlining, or favorable address placement.
Four fresh-process ABBA cycles produced:

| Case | Payload | `a8303c8` median | `1cc5c21` median | Paired improvement | Direct speedup | 95% speedup interval |
|---|---:|---:|---:|---:|---:|---:|
| exact selection | 32 | 693.261 us | 103.250 us | 85.12% | 6.72x | 6.69-6.75x |
| exact selection | 64 | 687.425 us | 103.208 us | 85.05% | 6.69x | 6.66-6.74x |
| render plus complete payload scan | 32 | 895.242 us | 129.725 us | 85.53% | 6.91x | 6.88-6.94x |
| render plus complete payload scan | 64 | 925.336 us | 125.837 us | 86.85% | 7.60x | 7.40-7.73x |

The speedups invert the paired candidate/baseline geometric-mean effects; their
intervals likewise invert the deterministic 50,000-resample 95% bootstrap
limits. Candidate CV was 0.08-0.26%. All 64 samples observed 2.208 GHz,
temperature stayed between 44.384 and 46.230 C, and maximum CPU/wall divergence
was 0.038%. This direct result establishes a 6.69-7.60x cumulative improvement
under the accepted algorithm-and-data-layout-only policy, exceeding the 5x goal
without deployment-specific binary-layout assumptions.

## Experiment 20: root-resolved terminal range dispatch

### Theory and profile evidence

The accepted actor-batch query still transforms the camera into every visible
instance and enters `appendDefinition` through a scratch-vector push/pop even
when the instance root is already fully inside the frustum. The same redundant
path occurs for every `renderAsUnit` actor after render coarsening forces its
root mask to zero. In both cases the answer is already the immutable payload
range stored for definition node zero; no local camera or hierarchy walk can
change it.

The SBC does not currently provide `perf` and the remote account has no
passwordless sudo, so a separate `-pg -g` diagnostic build was used only for
hotspot attribution. It is not an accepted benchmark build. Over the complete
8,192-frame payload32 trajectory, gprof sampled the definition-dispatch lambda
at 40.95% of exact-selection time and 40.65% of render-submission time, with
2,385,049 calls (291.14 per frame). The enclosing query body accounted for a
further 43.81% and 31.71%; motion generation was 7.62% and 6.50%.

The experiment will append the root plan range directly whenever culling has
cleared the root plane mask, or render-unit coarsening deliberately clears it.
Only partially intersecting roots will construct a transformed camera and
enter descendant traversal. The output range, instance/error metadata, caller
owned transform layout, culling result, and strict terminal-query contract are
unchanged.

### Prototype and paired screen

Both mounted general instances and external actor batches now test the resolved
root mask before constructing a local camera. A zero mask appends
`plan.ranges[0]` directly with the same instance/error word; the existing
descendant walker remains the only path for a nonzero mask. The full ARM Debug
matrix passes all 448 tests across BVH4/BVH8 and payload32/payload64.

Two-cycle paired screen
`/home/codex-perf/frontier/results/frontier-paired-20260818T003835Z` compared
the preserved `1cc5c21` ordinary-Release binaries with the prototype. Exact
selection improved 13.72% payload32 and 11.34% payload64. Render plus complete
payload scanning improved 9.22% and 8.11%. All intervals excluded zero, all 32
samples observed 2.208 GHz, temperature stayed between 45.307 and 47.153 C,
and maximum CPU/wall divergence was 0.028%. A four-cycle run including motion,
generic selection, and machine controls is required before acceptance.

### Final SBC result and decision

Full report
`/home/codex-perf/frontier/results/frontier-paired-20260818T004038Z` ran four
ABBA cycles for the target cases, motion, two generic-selector populations,
and four independent machine controls:

| Case | Payload | `1cc5c21` median | Candidate median | Paired effect | 95% interval |
|---|---:|---:|---:|---:|---:|
| exact selection | 32 | 103.589 us | 91.311 us | -11.97% | [-14.35%, -9.57%] |
| exact selection | 64 | 103.251 us | 91.491 us | -10.56% | [-11.61%, -8.64%] |
| render plus complete payload scan | 32 | 130.056 us | 117.796 us | -9.46% | [-9.57%, -9.36%] |
| render plus complete payload scan | 64 | 126.195 us | 114.069 us | -9.66% | [-10.07%, -9.02%] |
| motion writer | 32 | 8.135 us | 8.158 us | +0.03% | [-0.31%, +0.38%] |
| motion writer | 64 | 8.198 us | 8.184 us | -0.12% | [-0.27%, +0.02%] |

All 224 samples observed 2.208 GHz, temperature stayed between 44.384 and
48.076 C, and maximum CPU/wall divergence was 0.059%. The four machine
controls had a +0.28% paired geomean and the integer control was flat to 0.01%.

The combined benchmark executable reported 1.67% and 2.53% payload32 losses in
the unrelated 50% and 100% generic-selector cases; payload64 was contradictory.
This is not a changed generic algorithm or data layout. The baseline and
candidate `spatial_database.cpp.o` files are byte-identical for payload32
(`ebd0d52fb0ee1ad086873eec87141119132f6784adf645c28efe78f1219367b3`)
and payload64
(`37bfc27752c659f569478177f1dfca8386a475e38543fb1b0d8d8d8d713cce081a`).
The machine executable is also byte-identical
(`0f77d52bade50770c185b35e6f275d7a4f28b083e4aacf471f91726150c50845`).
Only the combined benchmark text size moved, by 768 bytes, because its live-city
case references the changed terminal archive member. The generic timing delta
is therefore incidental executable address placement--exactly the effect this
round refuses to count as an optimization. A follow-up harness isolation will
make generic controls independent of terminal-query linkage rather than tuning
production layout around this artifact.

Keep and commit. The root-resolved dispatch produces a repeatable 10.56-11.97%
selection gain and 9.46-9.66% end-to-end render gain with unchanged motion,
output, contracts, and generic traversal objects. Composing paired effects with
the direct round-start report places cumulative algorithm/data-layout-only
throughput at approximately 7.48-7.63x for exact selection and 7.63-8.42x for
complete render consumption; a later direct frozen-anchor run will replace
that composed estimate after the next architectural experiment.

## Experiment 21: exact spatially ordered actor clusters

### Theory

The post-Experiment-20 diagnostic profile reduced descendant-dispatch calls
from 291.14 to 19.84 per exact frame and 12.80 per render frame. The enclosing
query body still consumed 37.93% of sampled exact time and 19.83% of render
time. Every frame currently constructs and tests 1,100 actor-root AABBs against
up to six frustum planes even though actors are naturally partitioned into
small spatial neighborhoods.

The next data layout will partition each spatially ordered homogeneous cohort
with immutable `{first,count}` cluster spans. The query
will compute each cluster's exact current union directly from the authoritative
position/yaw spans, then classify the cluster once. An outside cluster skips all
member work. A fully inside cluster appends each actor's definition root range
without actor AABB tests, local cameras, or descendant traversal. Only a partial
cluster tests its members, and those tests inherit the cluster's narrowed plane
mask.

Unlike a caller-published broadphase bound, the cluster union cannot be stale:
it is rebuilt inside the timed query from the same transforms selection consumes.
The cost of the min/max reduction is therefore included in the frame benchmark.
Clusters are exact for both yaw-invariant and yaw-dependent actor bounds. Empty
cluster spans retain the current ungrouped path; clustered spans must form an
ordered, gap-free partition of the position stream. This exchanges spatially
ordered actor storage and a tiny immutable partition table for fewer branchy
scalar plane tests while preserving the exact per-actor result.

### Prototype, rejected layouts, and cluster-width tuning

`TerminalInstanceCluster` stores only two 32-bit words. In contract builds the
records must be nonempty and form an ordered, gap-free partition; all member
transforms remain validated. Release selection reduces each current cluster
union, tests it once, emits root ranges for a fully inside group, and retests
only members of a partial group with the already narrowed plane mask. A new
clustered-versus-ungrouped test compares instance/payload pairs for yawed actors
under three cameras and both exact and render-unit modes. The complete ARM
matrix now passes 452 tests.

The first benchmark layout used 2x2 car tiles and 4x4 pedestrian tiles. Screen
`frontier-paired-20260818T005909Z` showed the expected 20.08-21.49% selection
and 15.74-16.50% render gains, but recovering each actor's original trajectory
index made the isolated motion writer 13.51-14.29% slower. Precomputed motion
seeds made that loss 18.62-18.81% in
`frontier-paired-20260818T010222Z`. Both layouts were rejected.

The accepted benchmark layout retains the original row-major position/yaw
order and byte-for-byte trajectory logic. Cars use adjacent pairs and
pedestrians use short contiguous row segments. This keeps every physical
trajectory and all exact/render leaf-count extrema unchanged while allowing
the query to exploit spatial coherence. Width tuning on the fixed 2-car groups
gave:

| Pedestrian group width | Report | Selection candidate, p32/p64 | Render candidate, p32/p64 | Result |
|---:|---|---:|---:|---|
| 4 | `frontier-paired-20260818T010534Z` | 75.259/75.435 us | 101.457/97.252 us | viable |
| 8 | `frontier-paired-20260818T011008Z` | 74.354/74.482 us | 100.531/96.676 us | best |
| 16 | `frontier-paired-20260818T011153Z` | 76.579/76.699 us | 102.912/99.546 us | reject |

Width 16 creates elongated clusters that remain partial too often. Width 8
reduces cluster-test count enough to beat width 4 without losing useful whole-
cluster rejection. The one-cycle tuning reports choose the candidate only; a
fresh four-cycle acceptance run against frozen `d20f67c` binaries is still
required for the retained effect and motion/control gates.

### Final SBC result and decision

Full report
`/home/codex-perf/frontier/results/frontier-paired-20260818T011613Z` compared
the tuned 2/8 row-cluster candidate with frozen `d20f67c` ordinary-Release
binaries over four ABBA cycles:

| Case | Payload | `d20f67c` median | Candidate median | Paired effect | 95% interval |
|---|---:|---:|---:|---:|---:|
| exact selection | 32 | 91.413 us | 74.114 us | -19.09% | [-19.42%, -18.89%] |
| exact selection | 64 | 91.547 us | 74.451 us | -18.59% | [-18.79%, -18.38%] |
| render plus complete payload scan | 32 | 117.347 us | 100.624 us | -14.21% | [-14.38%, -13.99%] |
| render plus complete payload scan | 64 | 113.551 us | 96.806 us | -14.67% | [-15.16%, -13.94%] |
| motion writer | 32 | 8.158 us | 8.144 us | -0.11% | [-0.50%, +0.27%] |
| motion writer | 64 | 8.166 us | 8.174 us | +0.70% | [-0.36%, +2.50%] |

All 224 samples observed 2.208 GHz, temperature stayed between 44.384 and
47.153 C, maximum CPU/wall divergence was 0.071%, and the four machine controls
had a +0.17% paired geomean. The 8,192-frame exact and render entry averages,
minima, maxima, segment counts, and simulated duration are identical between
revisions.

The combined executable again moved the unrelated identity controls in opposite
directions by payload width: payload32 reported losses while payload64 reported
an improvement or an interval spanning zero. The actual generic traversal
objects remain byte-identical at
`ebd0d52fb0ee1ad086873eec87141119132f6784adf645c28efe78f1219367b3`
for payload32 and
`37bfc27752c659f569478177f1dfca8386a475e38543fb1b0d8d8d713cce081a`
for payload64. The machine executable is byte-identical at
`0f77d52bade50770c185b35e6f275d7a4f28b083e4aacf471f91726150c50845`.
The motion-writer symbol is the same 0x19c bytes and differs only in load
address (`0x9ea0` versus `0x9ee0`), matching its interval around no change.

Keep and commit. Exact clustered bounds are rebuilt inside the timed query, so
the 18.59-19.09% selection and 14.21-14.67% render improvements contain their
own broadphase-maintenance cost and cannot depend on stale external state.

The subsequent direct frozen-anchor report
`/home/codex-perf/frontier/results/frontier-paired-20260818T012314Z` compares
the unmodified round-start `a8303c8` ordinary-Release binaries directly with
the committed `f12fa0f` source over four ABBA cycles:

| Case | Payload | `a8303c8` median | `f12fa0f` median | Paired effect | Direct speedup | 95% interval |
|---|---:|---:|---:|---:|---:|---:|
| exact selection | 32 | 692.069 us | 74.172 us | -89.30% | 9.35x | [-89.35%, -89.24%] |
| exact selection | 64 | 685.753 us | 74.571 us | -89.02% | 9.11x | [-89.22%, -88.73%] |
| render plus complete payload scan | 32 | 895.278 us | 100.557 us | -88.79% | 8.92x | [-88.86%, -88.75%] |
| render plus complete payload scan | 64 | 913.836 us | 96.575 us | -89.72% | 9.73x | [-90.25%, -89.43%] |

All 64 samples ran at 2.208 GHz, temperature stayed between 45.307 and
47.153 C, maximum CPU/wall divergence was 0.035%, and the unchanged machine
control executable had the same SHA-256 in both builds. The remote restored
worktree deliberately remained based at an older bookkeeping commit, so the
report manifest's candidate commit field is not the source identity. The four
candidate inputs were verified byte-for-byte against local `f12fa0f` instead:
`terminal_render.cpp` is
`ee280a663bfc5c8594d2629d308bbbf158488c162df504045ce759b4ae6c01e2`,
the public header is
`73f9dc7a89dedbd9d3274cc74003d462e36fafcc60826bf84bb5f62bc0fe0e89`,
the benchmark is
`2359292fb3122c1810c376f13c41addf0b80dc1b9301759e2f5e15468b92e834`,
and the correctness test translation unit is
`79c250f23e0edb2836a4ebe2655b9b12e8fa064b143cefd54432748634e20f63`.

This direct result replaces the composed estimate. The retained architecture
is therefore independently measured at 9.11-9.35x faster for exact selection
and 8.92-9.73x faster for complete render consumption, using only portable
algorithm and data-layout changes in conventional Release builds.

## Experiment 22: fixed-capacity descendant scratch

### Theory and protocol

After root and cluster classification, the remaining partially visible actor
definitions still enter `appendDefinition`. Its depth-first worklist is a
`std::vector<uint32_t>` that is cleared, pushed, queried, and popped for every
actor that intersects the frustum boundary. Capacity is stable after warmup,
so no allocation occurs, but each library operation still maintains vector
size state and repeats capacity/empty checks around a stack whose maximum
requirement is already known from the immutable definition.

The proposed representation sizes the query-owned descendant scratch once
when a terminal plan is built, then treats it as a fixed-capacity array with a
local stack pointer during selection. This does not change traversal order,
culling, payload ranges, the public API, or output. It only specializes a
general growable container into the actual data structure required by the hot
loop. The risk is a larger persistent scratch allocation equal to the largest
eligible definition's packed node count; the benefit should scale with the
number of boundary descendants.

Before editing, freeze the accepted `f12fa0f` ordinary-Release binaries and
refresh a separate `-pg` diagnostic build. Profiling is used only to attribute
time and call counts; acceptance will compare conventional Release binaries in
paired ABBA runs. Test the stack change alone. Keep and commit it only if the
complete ARM correctness matrix passes and both payload widths show a stable
live-city gain without motion or generic-control regression.

The refreshed diagnostic profile observed 162,534 descendant descents across
8,193 exact queries (19.84/query) and 104,840 across 8,193 render queries
(12.80/query). `appendDefinition` received 45.71% of exact samples and 31.37%
of render samples. The prototype derives the maximum number of simultaneously
pending inner nodes while the immutable payload-range plan is built. It grows
the shared scratch to the maximum required by any cached plan, then uses a raw
array pointer plus local stack index in the hot loop. This retains an overflow
assertion in contract builds and avoids reserving space for every node in a
definition.

### Initial screen

All 452 ARM Debug tests pass. One ABBA screen
`frontier-paired-20260818T013917Z` reported -2.75%/-0.46% exact selection for
payload32/payload64, +0.25%/-0.68% render, and +0.65%/+0.48% motion. Machine
controls had a -0.09% geomean, but the payload32 selection baseline had 6.29%
CV and the generic identity cases again moved in contradictory directions by
payload width. This is too small and inconsistent to accept from one cycle.
Proceed to a four-cycle live-city and motion run against the frozen binary;
reject unless both payload widths and the end-to-end consumer establish a
repeatable improvement.

### Formal result and decision

Report `frontier-paired-20260818T014118Z` compared the isolated prototype with
the frozen `f12fa0f` ordinary-Release binaries over four ABBA cycles:

| Case | Payload | `f12fa0f` median | Prototype median | Paired effect | 95% interval |
|---|---:|---:|---:|---:|---:|
| exact selection | 32 | 74.132 us | 73.677 us | -1.35% | [-2.93%, -0.47%] |
| exact selection | 64 | 74.345 us | 73.876 us | -0.54% | [-0.64%, -0.46%] |
| render plus complete payload scan | 32 | 100.443 us | 100.853 us | +0.28% | [+0.01%, +0.69%] |
| render plus complete payload scan | 64 | 96.647 us | 96.617 us | +0.42% | [-0.72%, +1.57%] |
| motion writer | 32 | 8.148 us | 8.160 us | +0.27% | [-0.00%, +0.67%] |
| motion writer | 64 | 8.212 us | 8.219 us | +0.30% | [+0.09%, +0.47%] |

All 224 samples held 2.208 GHz, temperature was 44.384-47.153 C, maximum
CPU/wall divergence was 0.050%, and the four machine controls had a +0.21%
paired geomean. The generic identity results again contradicted each other and
cannot be caused by the terminal stack implementation.

Reject and revert. The only clean selection result is a small 0.54% payload64
gain, while the complete render consumer fails to improve at either payload
width and motion trends about 0.3% slower. Modern optimization already removes
most of the growable-vector abstraction cost once capacity is stable; exposing
its size as a local index does not remove the actual wide-AABB and payload-run
work that dominates boundary descent. Retain `f12fa0f` production code and use
the profile to pursue a change that eliminates descendant work rather than
micro-specializing its worklist.

## Experiment 23: exact transient hierarchy over actor clusters

### Theory

Experiment 21 classifies 175 exact current cluster bounds independently: 50
two-car groups and 125 eight-pedestrian row segments. A cluster that is outside
or inside saves member work, but the query still performs one root-to-plane
classification for every cluster even when a large contiguous region shares
the same result. The spatial ordering added in that experiment is sufficient
to construct a balanced hierarchy over the cluster sequence without changing
the public placement layout.

The prototype first derives every leaf cluster bound from the authoritative
current position/yaw spans exactly as before. It then unions those bounds into
a query-owned implicit binary tree. Traversal carries narrowed plane masks.
An outside internal node skips all enclosed clusters and actors; an inside node
emits the definition root range for its complete contiguous actor span; only a
partial leaf invokes per-actor bounds and descendant culling. Left-first
traversal preserves actor order. The tree is transient and rebuilt inside the
timed query, so there is no stale caller state and its complete maintenance cost
is included in the live-city result.

The tradeoff is two query-owned scratch arrays: roughly two AABBs per rounded-
up cluster and a shallow traversal stack. This adds a bottom-up min/max pass
each frame. It wins only if hierarchical plane-test and boundary-work savings
exceed that pass, so it will be screened and accepted independently against
the frozen `f12fa0f` binaries.

### Result and decision

All 452 ARM Debug tests passed, but the one-cycle paired screen
`frontier-paired-20260818T015213Z` was decisive:

| Case | Payload | `f12fa0f` median | Hierarchy median | Paired effect |
|---|---:|---:|---:|---:|
| exact selection | 32 | 77.523 us | 86.817 us | +11.99% |
| exact selection | 64 | 74.590 us | 84.244 us | +12.94% |
| render plus complete payload scan | 32 | 100.769 us | 111.089 us | +10.24% |
| render plus complete payload scan | 64 | 96.997 us | 107.310 us | +10.63% |

Motion was +0.20%/+0.66%; all samples held 2.208 GHz at 43.461-46.230 C,
maximum CPU/wall divergence was 0.049%, and the machine-control geomean was
-0.12%. The regression is far outside measurement noise, so a longer run is
unnecessary.

Reject and revert. The accepted width-2/8 leaves are already small and
spatially useful. Building approximately 430 internal/leaf AABBs and walking a
binary tree costs substantially more than directly classifying 175 flat
clusters. The result narrows the design space: retain the flat exact clusters
and optimize either their authoritative bound production or the much more
expensive partial-definition work, without adding a per-frame hierarchy-build
pass.
