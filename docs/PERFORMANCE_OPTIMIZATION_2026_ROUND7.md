# AArch64 payload32 live-city recovery

## Goal and target evidence

The clean AArch64 `fb8d395` validation recovered the identity-only hierarchy
regression, but the continuously moving, yawed live-city workload showed one
new localized result relative to `f78ffdc`:

| Payload | `f78ffdc` | `fb8d395` | Change |
|---|---:|---:|---:|
| 64-bit | 653.890 us | 653.470 us | -0.06% |
| 32-bit | 643.147 us | 654.405 us | +1.75% |

The payload32 means moved from 645.070 to 653.149 us (+1.25%). The two runs
had 0.56% and 0.46% real-time CV, respectively. Focused architecture-kernel
geomean was unchanged (+0.01%), so the payload32 movement was treated as a
real AArch64 code-layout regression rather than machine drift.

The source delta between those revisions physically split the identity and
rotated root walkers. It did not change actor motion, TLAS maintenance,
frontier caching, or output representation. The leading theory was therefore
instruction placement around the large cached selector and its rotated-root
callee, with a secondary theory around the 36-byte orientation stream.

## Measurement correction

Initial Windows experiments inherited the process scheduler's default
affinity. On the i9-12900K this allowed samples to migrate between P- and
E-cores: real-time medians ranged from about 97 to 106 us and CPU time varied
even more. Those samples are retained under
`perf_reports/optimization-round7/` for diagnosis but are not commit gates.

The corrected protocol launches every benchmark on logical CPU 4 with the
high-performance power plan active and a 0.25-second per-benchmark warmup,
matching the SBC collector's affinity/warmup discipline. Preserved baseline
and experiment executables are alternated, use fixed 8,192-frame trajectories,
and report median real time over seven or nine repetitions.

## Phase-isolation benchmark

`BM_LiveCityMotionFrame` now runs exactly the transform generation, two motion
group submissions, and `applyUpdates()` portion of the full live-city frame.
It uses the same 100 cars, 1,000 pedestrians, continuous headings, and 8,192
frame trajectory, but intentionally omits selection. Together with
`BM_LiveCityDrivingFrame`, it distinguishes actor/TLAS publication regressions
from cached traversal and result-output regressions on the target board.

## Experiment 1: split hot orientation from authored bounds

**Theory.** Movement and rotated selection need only yaw plus the conservative
XZ radius. Moving the 24-byte authored local bounds into a separate cold stream
would reduce the per-frame orientation working set from 36 to 12 bytes per
root while preserving the same total retained bytes.

**Result: rejected.** The isolated payload32 motion geomeans were 24.194 us for
`fb8d395` and 24.454 us for the split. A later full-frame comparison against
the exact baseline was flat: 105.940 versus 105.934 us. The extra stream offset
the bandwidth reduction on this machine, so the layout was restored.

## Experiment 2: hot non-identity-yaw dispatch bit

**Theory.** Record non-identity yaw in a spare bit of the existing 80-byte
`Instance`, dispatch to the already separate walker at call scope, and make the
rotated transform branchless.

**Result: rejected.** The preliminary alternating payload32 geomeans moved
from 98.030 to 98.492 us (+0.47%). The branch moved earlier than the existing
post-root-LOD yaw decision and did not reduce enough work to pay for itself.

## Experiment 3: skip zero damping-envelope rotation

**Theory.** The live-city query uses the default zero damping half-life, yet a
rotated miss still transforms the all-zero camera envelope. Detect zero once
and bypass the center/extent rotation.

**Result: rejected.** Under corrected CPU-4 affinity, the baseline geomean was
102.519 us and the specialization was 104.132 us (+1.57%). Testing six scalar
components plus the new branch cost more than rotating zeros. A SIMD zero test
did not reverse the result, so the original branchless transform was restored.

## Experiment 4: power-of-two orientation stride

**Theory.** Pad the split 12-byte yaw/radius record to 16 bytes. AArch64 can use
shift-only indexing and every eight-byte yaw load remains naturally contained,
at a cost of 4.652 KiB in the live-city scene.

**Result: rejected with the split layout.** Against the 12-byte split, 16-byte
stride improved the pinned payload32 geomean from 105.718 to 104.417 us
(-1.23%). However, the combined split-plus-padding result was flat against the
real repository baseline (105.934 versus 105.940 us). A component win that
does not improve the actual frame was not retained.

## Experiment 5: physically swap the two walkers

**Theory.** Put the rotated function back in the original pre-split source
position and move the identity walker behind cached selection. This directly
tests whether physical function order explains the target result.

**Result: diagnostic only.** Payload32 was flat (104.400 versus 104.352 us),
but payload64 regressed from 105.296 to 107.661 us (+2.25%). The swap was
removed. It nevertheless confirmed that function placement can move the frame
by more than the target regression even when function bodies and dispatch are
unchanged.

## Experiment 6: privileged AArch64 hot text island

GCC 13 AArch64 received `noinline`, `noclone`, and a shared
`.text.hot.frontier_oriented` section attribute on exactly two functions:

- `SpatialDatabase::selectFrontierCached()`;
- `SpatialDatabase::runOrientedTlasRootInstance()`.

The SBC link honored the constraint. In both payload executables the rotated
walker began at `0x9d50` and cached selection at `0xa804`, only 2,740 bytes
apart. The motion-only phase was also decisive: payload32 measured 142.254 us
and payload64 142.288 us, within 0.03%. Actor transform generation and TLAS
publication are not the source of the payload32 loss.

**Result: rejected.** Moving the pair into the linker's privileged hot-text
region made the target worse:

| Guard | `fb8d395` | hot island | Change |
|---|---:|---:|---:|
| payload32 live city | 654.405 us | 658.037 us | +0.55% |
| payload64 live city | 653.470 us | 651.968 us | -0.23% |
| payload32, 10k roots, 50% hierarchy | 1,623.364 us | 1,726.373 us | +6.35% |
| payload32, 10k roots, 100% hierarchy | 3,002.483 us | 3,153.940 us | +5.04% |
| payload64, 10k roots, 50% hierarchy | 1,666.783 us | 1,697.240 us | +1.83% |
| payload64, 10k roots, 100% hierarchy | 2,993.951 us | 3,101.865 us | +3.60% |

The architecture-kernel geomean moved only +0.33%, the selected core remained
at 2.208 GHz, and thermal state matched the control run closely. The hierarchy
losses are therefore too large and too localized to dismiss as host drift.
The `.text.hot.*` prefix grouped the pair successfully but also pulled the
large selector into the early hot region and perturbed unrelated traversal
placement. Proximity alone was the wrong constraint.

## Experiment 7: ordinary AArch64 text island

The next GCC AArch64 variant changed the section to
`.text.frontier_oriented`, preserving `noinline`, `noclone`, and selector–walker
adjacency while removing the privileged `.text.hot.*` classification. The link
again honored the constraint. The rotated walker and cached selector began at
`0x19900` and `0x1a3b4` for payload64 and `0x19990` and `0x1a444` for
payload32. Their sizes were identical across payloads: `0xab4` and `0x2fe4`.

**Result: rejected.** The run was broadly 0.57% slower in the machine-probe
geomean and 0.94% slower in the architecture-kernel geomean, but it still
failed the target after accounting for that drift:

| Guard | `fb8d395` | ordinary island | Change |
|---|---:|---:|---:|
| payload32 live city | 654.405 us | 661.000 us | +1.01% |
| payload64 live city | 653.470 us | 657.388 us | +0.60% |
| payload32, 10k roots, 50% hierarchy | 1,623.364 us | 1,648.503 us | +1.55% |
| payload32, 10k roots, 100% hierarchy | 3,002.483 us | 3,044.704 us | +1.41% |
| payload64, 10k roots, 50% hierarchy | 1,666.783 us | 1,711.647 us | +2.69% |
| payload64, 10k roots, 100% hierarchy | 2,993.951 us | 3,054.538 us | +2.02% |

Motion publication remained payload-neutral at 142.540 us for payload32 and
142.521 us for payload64. Moving the pair within text therefore cannot explain
or recover the payload-specific selection delta. Both ELF section constraints
were removed.

## Retained target experiment: traversal-local walkers

The rotated walker now appears immediately after the identity walker in source,
before flat-root dispatch and both selectors. No compiler or linker attributes
remain. The new locality relationship is between the two root walkers and the
subtree traversal machinery they repeatedly call, rather than between the
rotated walker and a selector that calls it only about 20 times per live-city
frame. Function bodies, dispatch, APIs, and data layouts are unchanged.

An alternating, CPU-4-pinned Windows A/B treated payload32 live city as flat:
median geomean +0.36%, mean geomean -0.15%. Payload64 improved 1.41% by median
geomean and 1.53% by mean. The four 10k identity guards ranged from -2.35% to
+0.26% by median; three improved and the fourth was within noise. This is only
a safety screen because MSVC does not predict GCC AArch64 placement. Direct
paired SBC measurement is the acceptance gate.

## Tradeoffs

- Source adjacency is a soft layout hint, not a linker contract. That avoids
  the global placement damage measured with named ELF sections but means a
  future compiler may choose a different order under LTO.
- Keeping two specialized walkers duplicates their common traversal-control
  body. This preserves the recovered identity fast path at a modest text-size
  cost; the experiment changes no retained data.
- The full benchmark suite gains realistic motion-only and render-submission
  cases. Submission lives in a separate executable so its larger harness does
  not perturb the LTO layout of the selection guards; this costs additional
  build and collection time.

## Required target acceptance

Retain traversal-local walker placement only if paired SBC runs satisfy both:

1. payload32 `BM_LiveCityDrivingFrame` recovers materially toward or below
   643.147 us without a corresponding machine/architecture-probe shift;
2. the four 10,000-root identity hierarchy cases remain within noise of the
   `fb8d395` result (or improve).

Payload64 live city is a guard and must remain statistically flat. The new
`BM_LiveCityMotionFrame` should also be payload-neutral; a change there would
falsify the instruction-locality theory because the retained implementation
does not affect actor motion or TLAS publication.

## Direct paired SBC acceptance

The target gate used GCC 13.3 Release/LTO BVH4 builds pinned to Cortex-A72 CPU
4. Every data point was a fresh process with an untimed warmup. Four ABBA
cycles (`baseline, candidate, candidate, baseline`) produced eight samples per
revision and payload. Frequency stayed at 2.208 GHz for every sample and the
temperature envelope stayed below 46.3 °C. The reported paired effect is the
geometric mean of candidate/baseline after averaging each revision's two
samples within a cycle; negative values are improvements.

The first run, `frontier-paired-20260817T084751Z`, compared the intermediate
ordinary-section build `993ed4e` with the traversal-local source layout
`bc194d9`:

| Case | payload32 | payload64 |
|---|---:|---:|
| live city | -0.40% | -0.16% |
| 10k roots, 50% hierarchy | +0.64% | -0.64% |
| 10k roots, 100% hierarchy | +0.34% | **-1.88%** |
| motion-only control | +1.16% | +0.84% |

The payload32 live-city recovery and payload64 hierarchy win were promising,
but the motion-only result made this comparison insufficient on its own. The
ordinary named-section baseline had already perturbed global text placement;
it could be faster in unrelated code even while losing the selection target.

A deliberately adversarial reverse run,
`frontier-paired-20260817T090252Z`, therefore compared retained `bc194d9`
against a candidate that restored the complete `fb8d395` implementation order.
The table reports the effect of that restoration, so positive numbers are
losses relative to the retained layout:

| Case | payload32 restoration | payload64 restoration |
|---|---:|---:|
| live city | **+0.97%** | -0.10% |
| 10k roots, 50% hierarchy | +1.34% | **+9.76%** |
| 10k roots, 100% hierarchy | +1.23% | **+9.34%** |
| motion-only control | -0.00% | -0.19% |

All four payload64 hierarchy cycles lost by 6.23–11.84% when the old order was
restored. Payload32 live city lost in all four cycles, while actor motion was
flat. This isolates the gain to selection-side instruction layout and clears
the negative control. The traversal-local walker placement is retained.

## Direct SBC experiment pipeline

Remote runs now use a dedicated unprivileged SBC account, NVMe-resident Git
worktrees, and separate build directories for each revision. The paired runner
records the exact commits, dirty patches, executable SHA-256 hashes, runner,
toolchain, kernel, governor, per-process load average, selected-core frequency,
and thermal envelope. Each benchmark sample is an isolated process pinned to
one big core and scheduled in ABBA order.

The paired report format adds four paired machine controls (integer dependency,
unpredictable branch dispatch, distance/error arithmetic, and 2 MiB sequential
read) and a realistic end-to-end city frame. That companion frame iterates the
two-span current cut, resolves every immutable payload, and writes a
preallocated render-submission stream. It intentionally excludes allocator and
graphics-driver work, keeping those platform-specific costs outside the scene
database comparison while removing the former downstream-iteration caveat.

### Instrumentation-layout defect and correction

Expanded report `frontier-paired-20260817T091446Z` initially compiled the new
submission case into the same executable as the established selection guards.
Although both revisions received identical benchmark source, the extra large
function changed whole-program LTO/link placement. Previously stable guards
changed direction: payload32 live city improved 1.02%, but its 50% and 100%
hierarchy guards lost 1.50% and 3.07%, while payload64 live city lost 1.08%.
This report was rejected as an acceptance result because instrumentation had
changed the code layout being measured.

The report also demonstrated why controls must be sampled as isolated
processes. The baseline and candidate machine executables were byte-identical,
yet the cache-hit control's first quartet differed by 7.53% and its overall CV
was about 2%. A same-process seven-repetition screen showed 0.04% CV, proving
that it concealed process-to-process variance rather than characterizing it.
That probe was removed from the paired gate in favor of the distance/error
kernel, whose screen had 0.01% CV.

The corrected build puts `BM_LiveCityRenderSubmissionFrame` only in
`frontier_submission_bench` and its payload32 twin. The primary executables
define `FRONTIER_OMIT_SUBMISSION_BENCH`, so the preprocessor removes the new
helper and benchmark completely before LTO. The established selection,
motion, and hierarchy guards therefore retain their old harness token stream.
The paired runner selects the submission executable only for the end-to-end
case and archives hashes for both executable families.

## Final isolated-harness acceptance

Report `frontier-paired-20260817T094501Z` is the authoritative comparison. It
uses exact library commits `fb8d395` and `bc194d9`, the isolated executable
families above, four ABBA cycles, and 224 fresh processes. All 224 samples ran
at 2.208 GHz; temperature ranged from 44.384 to 47.153 °C; maximum CPU-time
versus wall-time divergence was 0.033%. The four byte-identical machine
controls had a +0.17% paired geomean, inside the ±0.25% practical-equivalence
band.

| Workload | payload32 effect | payload64 effect |
|---|---:|---:|
| live-city selection frame | -0.00% | -0.19% |
| live-city frame + render submission | **-1.71%** | +0.45% (inconclusive) |
| motion/publication only | -0.04% | +0.20% (inconclusive) |
| 10k roots, 50% hierarchy | +4.24% (noisy) | **-8.42%** |
| 10k roots, 100% hierarchy | -0.00% | **-9.52%** |

The four-cycle payload32 50%-hierarchy row had 6.16% candidate CV and one
+11.65% cycle, so it was not accepted at face value. A 12-cycle focused follow-
up, `frontier-paired-20260817T100640Z`, reduced candidate CV to 0.97% and found
+0.28% with a 95% interval of -0.18% to +0.83%. That is inconclusive at the
±0.25% practical threshold, not a reproduced regression. The same follow-up
confirmed an 8.14% payload64 improvement with a wholly negative interval.

The retained candidate's SBC medians in the final full matrix were:

- selection-only live city: 658.378 us payload32, 660.555 us payload64;
- end-to-end live city plus CPU submission: 1,117.066 us payload32,
  1,170.916 us payload64;
- actor motion and TLAS publication: 143.804 us payload32, 143.745 us
  payload64;
- 10k-root payload64 uncached hierarchy: 1,615.909 us at 50% hierarchy and
  2,969.672 us at 100% hierarchy.

**Decision: retain `bc194d9`.** The realistic payload32 frame improves 1.71%,
the payload64 hierarchy path improves 8–10%, neither motion-only control nor
core live-city selection has a reproduced regression, and the long payload32
guard is statistically unresolved around zero. The source-order result remains
layout-sensitive, so the isolated executable boundary and paired target gate
are part of the optimization, not merely test scaffolding.

## Local verification

- Debug BVH4/BVH8 with both 8-byte and 4-byte payloads: 404/404 tests passed.
- Strict MSVC `/analyze /WX`: passed.
- MSVC no-exceptions `/EHs-c- /WX`: passed.
- Release payload32 and payload64 benchmark targets: built successfully.
- Both benchmark inventories contain the driving-frame and motion-only cases.
- The primary inventories omit render submission; both isolated submission
  inventories contain exactly one render-submission case and execute it.
- Linux collector shell syntax: validated with `bash -n`.
- The analyzer validated the 224-sample full report and 96-sample focused
  report without missing, duplicate, or misordered ABBA samples.

The GCC 13 AArch64 target gate passed at `bc194d9`; future compiler or source
layout changes must repeat the paired SBC matrix because source adjacency is a
soft optimizer hint rather than an ABI guarantee.
