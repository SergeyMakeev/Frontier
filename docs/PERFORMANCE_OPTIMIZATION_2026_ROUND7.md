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

## Retained target experiment: normal-text island

The revised GCC AArch64 attribute uses `.text.frontier_oriented`, preserving
the same `noinline` and `noclone` guarantees but removing the privileged
`.text.hot.*` classification. The GNU linker can keep the pair adjacent in the
ordinary text bucket without moving the large cached selector ahead of normal
traversal code. This is a one-field revision of the failed experiment: the
function bodies, calls, API, data layout, and benchmark workload are unchanged.

The attribute remains limited to GCC on AArch64. MSVC, Clang, x86, and other
targets expand it to nothing. The Linux collector records the final linked
addresses in `oriented_text_layout.txt`, so the next bundle will show whether
the pair stayed together and whether it returned to the normal text range.

## Tradeoffs

- The optimization is intentionally compiler/architecture-specific because
  that is where the regression exists. A future AArch64 Clang result needs its
  own evidence before opting into the section attribute.
- A named ordinary ELF text section constrains linker freedom for these two
  functions without assigning the whole pair privileged hot placement. It has
  no data-memory cost.
- Preventing inlining and cloning preserves identity isolation and call
  locality but gives up future GCC LTO freedom for these two functions. The
  target result determines whether that trade is favorable.
- The full benchmark suite gains one realistic motion-only case, increasing
  collection time slightly in exchange for removing ambiguity in future
  dynamic-scene regressions.

## Required target acceptance

Retain the normal-text island only if the next clean SBC run satisfies both:

1. payload32 `BM_LiveCityDrivingFrame` recovers materially toward or below
   643.147 us without a corresponding machine/architecture-probe shift;
2. the four 10,000-root identity hierarchy cases remain within noise of the
   `fb8d395` result (or improve).

Payload64 live city is a guard and must remain statistically flat. The new
`BM_LiveCityMotionFrame` should also be payload-neutral; a change there would
falsify the instruction-locality theory because the retained implementation
does not affect actor motion or TLAS publication.

## Local verification

- Debug BVH4/BVH8 with both 8-byte and 4-byte payloads: 404/404 tests passed.
- Strict MSVC `/analyze /WX`: passed.
- MSVC no-exceptions `/EHs-c- /WX`: passed.
- Release payload32 and payload64 benchmark targets: built successfully.
- Both benchmark inventories contain the driving-frame and motion-only cases.
- Linux collector shell syntax: validated with `bash -n`.

The architecture-specific attributes intentionally do not activate in these
local MSVC builds. GCC 13 AArch64 compilation, linked symbol proximity, and the
performance accept/reject conditions above remain target-board gates.
