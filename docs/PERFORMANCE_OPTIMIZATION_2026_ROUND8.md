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
