# Identity-yaw traversal recovery

## Goal

The controlled AArch64 result for `f78ffdc` found a localized regression in
uncached mounted-hierarchy selection relative to controlled commit `8320f75`:

| Case | Payload64 | Payload32 |
|---|---:|---:|
| 10,000 roots, 50% hierarchical | +6.67% | +8.35% |
| 10,000 roots, 100% hierarchical | +10.79% | +7.74% |

Machine and focused-kernel geometric means differed by less than 1%, and the
affected cases had low coefficients of variation, so the change could not be
dismissed as board noise. The runtime change between the controlled baseline
and `f78ffdc` was top-level yaw support. The goal of this round was to recover
the translation-only path without slowing the rotating live-city workload or
growing the 80-byte hot `Instance` record.

The implementation baseline for local experiments was clean commit `631cdbb`.
Its only change after `f78ffdc` is coverage-tool discovery, so its runtime is
the shipped yaw implementation measured by the SBC bundle.

## Measurement protocol

Canonical local comparisons use preserved binaries built from the same MSVC
19.44 Release/AVX2/BVH8/IPO configuration, with contract checks, statistics,
and subtree validation disabled. Each affected case used nine repetitions and
at least 0.5 seconds per repetition; tables report median real time. Raw JSON
is retained under `perf_reports/optimization-round6/`.

For diagnosis, `8320f75` was exported with `git archive` and compiled with the
same compiler, dependency source, and CMake options. This made it possible to
separate a real source-level delta from the larger AArch64 sensitivity.

## Baseline reproduction

The regression reproduced locally, although at roughly 3-5% rather than the
SBC's 7-11%:

| Payload | Roots | Hierarchical | `8320f75` | Current baseline | Delta |
|---|---:|---:|---:|---:|---:|
| 64 | 1,000 | 50% | 25.851 us | 26.793 us | +3.64% |
| 64 | 1,000 | 100% | 46.378 us | 48.846 us | +5.32% |
| 64 | 10,000 | 50% | 251.983 us | 263.667 us | +4.64% |
| 64 | 10,000 | 100% | 467.073 us | 487.634 us | +4.40% |
| 32 | 1,000 | 50% | 25.872 us | 26.870 us | +3.86% |
| 32 | 1,000 | 100% | 46.427 us | 48.280 us | +3.99% |
| 32 | 10,000 | 50% | 253.412 us | 261.284 us | +3.11% |
| 32 | 10,000 | 100% | 468.300 us | 490.334 us | +4.71% |

The local real-time CV was 0.19-0.87%, substantially below every delta.

## Experiment 1: compile-time orientation mode

**Theory.** Template the complete root walker on whether the database owns an
orientation stream, then choose the specialization once outside the
all-hierarchical visible-root loop. The identity specialization contains no
orientation vector access or yaw branch.

**Result: rejected.** Duplicating the complete hot walker changed instruction
layout enough to make all payload64 cases 1.2-2.0% slower than the current
baseline. Payload32 was also slower. Removing the branch in source was not by
itself sufficient if both large specializations remained coupled to the same
optimized region.

## Experiment 2: remove orientation handling

**Theory.** Temporarily force the original translation-only `toLocal()` call
to determine whether the loss lives in placement dispatch or in unrelated
hierarchy code changed by the yaw commit.

**Result: diagnostic only.** The four payload64 medians became 25.735, 46.167,
251.473, and 467.772 us. These are within about 0.3% of the separately compiled
`8320f75` binary. This intentionally incorrect yaw build proved that the
regression came from coupling the rotated-camera path into
`runTlasRootInstance()`, not from TLAS traversal, mounted subtree traversal,
or result construction.

## Experiment 3: hot non-identity-yaw bit

**Theory.** Use one otherwise unused state bit in the existing `Instance`
record to replace the optional-vector emptiness test and cold yaw read for
identity actors. No record or allocation would grow.

**Result: rejected.** The extra branch remained inside the same large walker.
Payload64 was effectively unchanged, and the payload32 improvements were
inconsistent. The vector lookup was not the main cost; the compiler's combined
identity/rotated control flow was.

## Experiment 4: out-of-line rotated camera helper

**Theory.** Keep one walker but move the yaw-aware `toLocal()` implementation
behind a non-inlined helper, reducing register pressure and hot instruction
footprint on the identity side.

**Result: rejected.** The 10,000-root cases improved by about 1-2%, but the
1,000-root/50% case regressed by about 3%. Returning the large `Camera` value
across the helper boundary did not cleanly restore the original code shape.

## Experiment 5: non-inlined specializations

**Theory.** Repeat experiment 1 while explicitly preventing both complete
walkers from being inlined into selection loops.

**Result: rejected.** Results stayed within roughly 1% of the regressed
baseline. Merely controlling inlining did not isolate the two optimized code
regions sufficiently.

## Experiment 6: physically separate root walkers

**Status: retained.**

The final architecture has two private, out-of-line root walkers:

- `runTlasRootInstance()` is translation/scale-only and contains exactly one
  branchless identity `toLocal()` construction after the root LOD decision.
- `runOrientedTlasRootInstance()` owns the optional orientation lookup,
  identity-yaw check, and exact rotated camera transform.

Selection tests `instanceOrientations_.empty()` at population/loop scope for
the common all-hierarchical serial and parallel paths. Mixed forests dispatch
the correct walker for their hierarchical roots. Cached record misses make the
same choice; cache hits still fetch neither walker nor the `Instance` record.
An allocated orientation stream is a database-wide mode: it may contain
identity actors, so the oriented walker retains its per-instance identity-yaw
fast path.

### Retained identity-path result

| Payload | Roots | Hierarchical | Current baseline | Split walker | Improvement | vs `8320f75` |
|---|---:|---:|---:|---:|---:|---:|
| 64 | 1,000 | 50% | 26.793 us | 25.980 us | 3.03% | +0.50% |
| 64 | 1,000 | 100% | 48.846 us | 46.754 us | 4.28% | +0.81% |
| 64 | 10,000 | 50% | 263.667 us | 253.753 us | 3.76% | +0.70% |
| 64 | 10,000 | 100% | 487.634 us | 466.956 us | 4.24% | -0.03% |
| 32 | 1,000 | 50% | 26.870 us | 25.835 us | 3.85% | -0.14% |
| 32 | 1,000 | 100% | 48.280 us | 46.423 us | 3.85% | -0.01% |
| 32 | 10,000 | 50% | 261.284 us | 251.694 us | 3.67% | -0.68% |
| 32 | 10,000 | 100% | 490.334 us | 466.446 us | 4.87% | -0.40% |

All eight final medians are within 0.81% of the pre-yaw binary. The four
10,000-root forced record-cache misses also improved by about 1.6-3.9% for
payload64 and 1.6-2.3% for payload32 in the regression-guard run.

### Rotating live-city guard

The split does not trade identity recovery for yaw performance:

| Payload | Current baseline | Split walker | Change | CVs |
|---|---:|---:|---:|---:|
| 64 | 98.187 us | 98.399 us | +0.22% | 0.64% / 0.57% |
| 32 | 98.801 us | 97.330 us | -1.49% | 0.67% / 0.77% |

Payload64 is statistically flat; payload32 improved in the sampled order. The
trajectory counters are bit-identical: 24,072.7 average entries, 16,455
minimum, 28,076 maximum, 93.0291% reuse, 272.656 reused roots, and 20.4307
walked roots per frame.

## Tradeoffs

- The two walkers intentionally duplicate root decision and mounted-subtree
  dispatch code. This prevents an optimizer from coupling the large yaw camera
  transform back into the identity path, but future behavioral changes must be
  made in both walkers.
- The Release benchmark executables grew by 3,072 bytes for payload64 and
  2,560 bytes for payload32. No runtime record, descriptor, query, or database
  allocation grew.
- Databases that allocate an orientation stream use the oriented walker even
  for individual identity-yaw roots. This preserves the fast no-orientation
  database mode while keeping mixed-heading correctness.
- The exact amount recovered on AArch64 must be confirmed by another
  controlled SBC collection. Local results show source-level recovery to the
  pre-yaw binary but are not substituted for target measurements.

## Verification

- MSVC Release/AVX2/BVH8/IPO builds passed for both payload widths.
- Strict MSVC `/analyze /WX` and `/EHs-c- /WX` library builds passed.
- 404/404 Debug tests passed across BVH4/BVH8 and payload32/payload64,
  including exact yaw camera transforms, invariant root envelopes, angular
  cached-versus-uncached selection, and parallel/serial identity.
- `git diff --check` passed.
