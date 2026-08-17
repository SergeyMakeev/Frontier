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
