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
