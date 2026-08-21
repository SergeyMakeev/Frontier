# Performance optimization round 9: explicit incremental TLAS maintenance

> Archived engineering journal. Revisions, APIs, measurements, and conclusions
> in this file record this experiment and are not the current integration
> reference. Use the API, architecture, and performance documents for the
> current contract.

Date: 2026-08-20

## Baseline and motivating observation

This round starts from repository commit `619ce7d`, the current repository
state when the experiment began.

The motivating application reported a stable frame cost near 0.2 ms followed
by a periodic TLAS-maintenance frame near 2 ms. Motion grew conservative TLAS
lanes cheaply, then population, edit, or accumulated-area drift crossed a
threshold and made the next `applyUpdates()` rebuild the whole TLAS. The
amortized cost could be acceptable while the latency distribution was not: a
quality threshold controlled *when a full repair happened*, not *how much work
one frame accepted*.

The requested policy was deliberately softer than a hard real-time guarantee:

1. maintenance must be an explicit parameter of `applyUpdates()`;
2. one call may stop before all dirty nodes are tight;
3. the caller may pay a small repair charge every frame, pay nothing, or drain
   all work;
4. no duplicate TLAS or other double-buffered maintenance architecture;
5. unprocessed bounds may stay loose because they remain conservative.

## Theories considered

### Time budget

A wall-clock deadline is intuitive but would require a clock query inside the
repair loop, varies with frequency and contention, and makes identical inputs
publish different repair states. It also cannot guarantee a time limit because
one indivisible node repair can cross the deadline.

### Node budget

A node count makes the indivisible unit explicit. One unit reads one wide TLAS
node, replaces each lane with its exact instance bound or current conservative
child extent, updates contribution and layer-mask summaries, and conditionally
queues its parent. The unit has bounded, architecture-dependent cost that an
application can calibrate on its target hardware. This was selected.

### Full duplicate hierarchy

Maintaining a second exact TLAS would let one hierarchy remain queryable while
the other is repaired. It was rejected because the existing grown hierarchy is
already queryable at every intermediate point. A duplicate would add memory,
publication state, and swap rules without improving correctness for this goal.

### FIFO versus a contiguous LIFO stack

A contiguous stack has cheaper storage, but continuous new leaf motion can
starve older work. The implementation uses a deduplicated FIFO queue so every
dirty node eventually advances under any nonzero budget. One byte per allocated
TLAS node prevents duplicate queue entries.

## Implemented architecture

### Publication contract

`applyUpdates(uint32_t maintenanceNodeBudget)` now returns an `UpdateReport`.
The report contains:

- nodes actually processed by this call;
- nodes still pending;
- current TLAS lane-area growth relative to the last TLAS build;
- an advisory `optimizeRecommended` bit;
- whether a correctness-required initial or defensive build occurred.

A budget of zero performs no optional tightening. A finite value repairs at
most that many nodes. `kUnlimitedTlasMaintenance` drains the current queue.

### Incremental repair data

The database owns:

- `std::deque<uint32_t> tlasRepairQueue_`, holding FIFO node indices;
- `std::vector<uint8_t> tlasRepairQueued_`, one deduplication byte per allocated
  node;
- `double tlasCurrentArea_`, the sum of all currently valid stored lane areas;
- `double tlasBaseArea_`, the lane-area sum established by the last TLAS build.

Motion still writes exact bounds to the dense instance array immediately.
Small cohorts grow TLAS leaf and ancestor lanes conservatively. Each changed
leaf is queued once. Repairing a leaf copies exact current instance data into
its lanes and clears their loose flags. Repairing an interior node recomputes
its lanes from child extents. If a repair changes a node with a parent, the
parent is queued. Therefore shrinkage reaches the root incrementally without a
separate dirty-subtree graph.

Area accounting changes at the same sites as TLAS storage: insertions add new
lane and ancestor growth, removals subtract cleared lanes, motion adds envelope
growth, and repair subtracts shrinkage. This makes the reported area ratio
describe the current tree instead of historical accumulated growth.

### Mandatory and advisory work

Correctness-required topology creation remains automatic for the initial build
and defensive recovery from inconsistent bookkeeping. Dense motion at or above
one quarter of the TLAS population still uses the existing exact bottom-up
streaming refit because it is the publication strategy for that update and is
cheaper than scattered grow walks.

Population drift, incremental edit count, and current area growth no longer
schedule an implicit topology rebuild. They only compute
`optimizeRecommended`. `optimize()` is the sole application-controlled quality
rebuild and compaction operation.

## Correctness experiments

New focused tests prove that:

1. the initial publication reports its required build;
2. moving one instance with budget zero leaves exactly one queued leaf and a
   conservative loose envelope;
3. budget one processes exactly the leaf, tightens its instance lanes, and
   leaves its parent queued;
4. unlimited maintenance drains the remaining ancestor chain;
5. area drift recommends optimization without rebuilding;
6. returning an actor to its original position and draining repair restores
   the area ratio and clears the recommendation;
7. count/edit drift is advisory, selection remains correct, and explicit
   `optimize()` clears the recommendation;
8. publishing an empty database before its first population does not consume
   the configured-quality initial build or create a false drift recommendation.

The complete Debug suite passed for both BVH widths: 242/242 tests. This
includes randomized spawn/remove/move churn compared with an independent live
instance model and concurrent readers over a published snapshot. The Release
city sample also compiled with debug TLAS tooling enabled.

## Maintenance-cost benchmark

`BM_TlasIncrementalMaintenance` builds 10,000 instances, moves 100 distributed
actors back and forth, and times motion plus publication with four budgets. It
reports processed, pending, and area growth averaged across calls. Averaging
avoids making the area counter depend on whether benchmark calibration chooses
an odd or even iteration count.

The Windows i9-12900K AVX2/BVH8 Release build measured:

| Payload | Node budget | Wall time/frame | Processed/frame | Pending/frame | Average area growth |
|---:|---:|---:|---:|---:|---:|
| 64-bit | 0 | 1.67 us | 0 | 48.0 | 0.1822% |
| 64-bit | 16 | 3.08 us | 16 | 49.25 | 0.1760% |
| 64-bit | 64 | 7.30 us | 64 | 16.00 | 0.1572% |
| 64-bit | 256 | 8.70 us | 67.5 | 0 | 0.0786% |
| 32-bit | 0 | 1.68 us | 0 | 48.0 | 0.1822% |
| 32-bit | 16 | 3.05 us | 16 | 49.25 | 0.1760% |
| 32-bit | 64 | 7.16 us | 64 | 16.00 | 0.1572% |
| 32-bit | 256 | 8.58 us | 67.5 | 0 | 0.0786% |

Both payload-width wall-time aggregates used five repetitions and had
0.12-0.76% CV. Their process CPU-time samples were scheduler-noisy, so the
table uses the stable high-resolution wall-time column.

The benchmark is not a replacement for application calibration: node cost
depends on BVH width, depth, lane occupancy, and cache state. Its purpose is to
show that the cap is obeyed, backlog is observable, and cost rises smoothly
until the workload drains rather than jumping to an implicit full rebuild.

## Decision and tradeoffs

Keep the experiment.

The application now owns the latency/quality choice directly. A zero budget
can retain broad envelopes indefinitely. A budget below the dirty-node arrival
rate produces a persistent queue and gradually weaker culling. A sufficient
budget converges to exact TLAS bounds. The report exposes both conditions, and
the application can change its budget or schedule `optimize()` at a known safe
point.

The implementation adds one byte per allocated TLAS node, a queue of pending
32-bit node indices, two area accumulators, and a small report computation per
publication. It does not duplicate TLAS nodes or instance data. Node-count
budgets are deterministic but not portable time guarantees; applications
should benchmark representative content on every target and select a budget in
nodes per update.
