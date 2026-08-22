# Frontier refinement computation API

Status: implemented.

This document records the bounded, policy-free API for inspecting refinement
opportunities below the current frontier produced by one `SpatialQuery`,
including removal of the materialized ideal frontier from the selection API.

## 1. Motivation

The previous API computed two cuts during every selection:

```text
current = shared + currentOnly
ideal   = shared + idealOnly
```

The current cut is renderable now. The ideal cut describes what selection
would choose if every known definition node were ready. The ideal cut is useful
as a direction for refinement, but materializing it is an unnecessarily
expensive way to communicate that direction.

In the limiting case, current contains one root while ideal contains millions
of terminal nodes. Computing and storing that complete ideal cut does work
proportional to a result the application may not have enough memory or
bandwidth to load. It also provides too little information about the useful
intermediate cuts between those extremes.

Streaming decisions must normally operate on complete coverage groups. A
parent cannot leave a hole-free current cut until every visible branch it
represents has a ready replacement. Loading one child without the other
required children may therefore provide no visible frontier improvement.

Frontier should instead:

1. select only the current, renderable cut;
2. retain the exact view and selection context that produced it;
3. compute a bounded forest of complete refinement groups on demand;
4. let the application choose resource requests from that forest.

An unbounded refinement computation subsumes the exceptional use cases for a
complete ideal cut. Its terminal leaves form an exhaustive, threshold-directed
target for the currently mounted topology and retained selection context. The
important change is that this cost becomes explicit and opt-in.

## 2. Responsibility boundary

Frontier owns:

- selecting the hole-free frontier that is renderable now;
- retaining the exact damped view and selection parameters used by that
  selection;
- walking mounted topology below the supplied current frontier;
- calculating view-dependent screen errors using the retained context;
- returning complete visible immediate-child groups, never partial groups;
- stopping refinement at the same error threshold used by selection;
- bounding explicit refinement analysis by depth and optional node count.

The application owns:

- external resource identity, byte cost, and sharing;
- memory and in-flight bandwidth budgets;
- quality, gameplay, and per-view priority policy;
- choosing a mixed-depth target cut from returned groups;
- asynchronous loading, cancellation, request coalescing, and hysteresis;
- readiness publication and resource eviction;
- aggregation across cameras.

The existing `UserPayload` remains the only application data stored on a node.
It is sufficient to address arbitrary external resource metadata. The API does
not add resource sizes, keys, priorities, residency records, or loading state
to the hierarchy, and it does not assume equal payload values identify one
shared resource.

## 3. Why this is refinement, not a gap

`computeFrontierGap()` was appropriate while both current and ideal cuts were
explicit inputs. With one supplied cut, there is no longer a gap between two
materialized endpoints.

The operation starts at the current frontier and computes structurally valid
ways to refine it. The resulting name is therefore:

```cpp
computeFrontierRefinement()
```

The result is correspondingly named `FrontierRefinementView`.

`compute` communicates that the method performs hierarchy traversal, child
visibility tests, and projected-error evaluation. `refinement` describes the
data without implying that Frontier chooses a streaming plan. Names such as
`getFrontierRefinement()` suggest retrieval of cached state, while
`planFrontierRefinement()` would place application policy inside the library.

## 4. Goals

The design must:

1. remove materialized ideal-cut output and work needed only to produce it
   from ordinary selection;
2. add no refinement-analysis work to applications that only render current;
3. live on `SpatialQuery`, which owns the exact damped view context;
4. start from the current result of an existing selection;
5. return ordinary `FrontierEntry` values for candidate nodes;
6. identify each coverage group by the existing `NodeHandle` of its parent;
7. expose complete visible immediate-child covers;
8. support several levels so a caller may intentionally skip intermediate
   representations;
9. preserve group boundaries under every output limit;
10. support explicit unlimited traversal for exhaustive workflows;
11. remain a read-only, policy-free building block.

## 5. Non-goals

The API does not:

- select a budgeted streaming frontier automatically;
- mutate readiness, topology, or selection state;
- start, cancel, or track asynchronous resource requests;
- decide whether an intermediate or terminal representation is preferable;
- expose a built-in multi-camera planner;
- enumerate topology that has not been mounted yet;
- guarantee that a bounded traversal has small breadth;
- predict future camera motion.

## 6. Selection API after removing ideal

Ordinary `selectFrontier()` produces one ordered current frontier. The old
three-bucket result exists only to share storage between current and ideal and
is no longer needed.

The implemented logical surface is:

```cpp
struct FrontierResultView
{
    std::span<const FrontierEntry> entries;

    auto begin() const { return entries.begin(); }
    auto end() const { return entries.end(); }
    size_t size() const { return entries.size(); }
    bool empty() const { return entries.empty(); }
};

class SpatialQuery
{
public:
    FrontierResultView selectFrontier(
        const SpatialDatabase& database,
        const Camera& camera,
        const SelectionParams& params);

    void selectFrontier(
        const SpatialDatabase& database,
        const Camera& camera,
        const SelectionParams& params,
        Sink<FrontierEntry>& output);

    void selectFrontier(
        const SpatialDatabase& database,
        const Camera& camera,
        const SelectionParams& params,
        FrontierResult& output);
};
```

`FrontierResult` becomes an owning container for one entry sequence rather
than three buckets. The exact owning-container methods can follow the existing
type's conventions.

The following concepts disappear from the selection result:

- `shared`, `currentOnly`, and `idealOnly`;
- `current()` and `ideal()`;
- `currentSize()` and `idealSize()`;
- the two-span `FrontierCutView` representation;
- three-sink `FrontierResultSink` output.

This is intentionally a breaking API change. The renderer consumes
`result.entries` directly. Streaming integrations replace inspection of
`idealOnly` with an explicit bounded `computeFrontierRefinement()` call.

`CurrentCutPolicy` remains meaningful. Selection may still inspect descendants
when that policy requires them; removing ideal does not promise that all
current-cut algorithms stop at the first unavailable node. For example,
`PreferReadyDescendants` may select a complete ready descendant cover below an
unavailable node. Refinement analysis never coarsens such a result merely to
reconstruct the old ideal cut; current remains authoritative, and only
quality-improving downward alternatives are returned.

## 7. Core refinement model

### 7.1 Implicit target

The old ideal frontier becomes an internal stopping rule, not a public result.
For each visited branch, refinement continues while the effective projected
error exceeds the retained selection threshold and mounted topology offers a
finer representation. It stops when:

- the branch satisfies the threshold;
- no finer mounted topology exists;
- the caller's depth horizon is reached; or
- the caller's node limit prevents another complete group.

A mounted terminal or mountable boundary node may remain over threshold. The
application can use its payload to request topology or content, publish the
result, and recompute refinement against the new snapshot.

### 7.2 Local traversal and visibility

Without an explicit ideal endpoint, refinement must discover the relevant
descendants. It starts from each current entry and walks only its placed
subtree. It does not redo TLAS traversal, top-level instance masking, or
top-level contribution culling.

The method does perform the local work needed to determine a correct visible
child cover:

- child bound/frustum tests using the exact retained damped camera;
- placed-bound and instance-overlay evaluation;
- effective error-clamp propagation across mount boundaries;
- bound distance and projected geometric-error evaluation.

This is still substantially narrower than a fresh spatial selection: the
current frontier already fixes the visible top-level placements and starting
nodes. More importantly, a bounded call does not traverse or materialize the
entire implicit ideal frontier.

The query must retain the complete effective camera used by the immediately
preceding selection, including its damped frustum and envelope. Retaining only
projection scale or a query envelope is insufficient for local child
visibility. The method deliberately accepts no new `Camera` argument because
an arbitrary camera could disagree with the selection that produced current.

### 7.3 Coverage groups

A group is identified by the `NodeHandle` of a renderable parent. Its members
are the complete set of visible immediate runtime children required to cover
the branches represented by that parent for this observation.

The group is view-specific. It need not contain authored children culled for
the retained view. The same parent can therefore have different group
membership in another camera or frame. The parent handle is the natural group
anchor, not a permanent id for one immutable sibling set.

Every member is an ordinary `FrontierEntry`. It carries the runtime node
handle, top-level instance id, and threshold-relative error code already used
by selection results.

### 7.4 Recursive alternatives and multi-level jumps

Suppose refinement returns:

```text
P -> {A, B, C}
A -> {A1, A2}
```

The application may request the one-level target:

```text
{A, B, C}
```

or deliberately skip `A` and request:

```text
{A1, A2, B, C}
```

Only leaves of the application's chosen refinement forest become resource
requests. In the second target, `A` is an evaluated alternative, not a required
resource.

Coverage is recursive. The branch represented by `A` is covered when `A` is
ready or when the complete selected descendant cover `{A1, A2}` is ready.
Consequently `P` can leave current once `B`, `C`, `A1`, and `A2` are ready even
if `A` was never loaded. This matches existing ready-descendant behavior.

### 7.5 Bounded decision horizon

`maxDepth` counts logical parent/child transitions below the current cut. It is
not absolute authored-tree depth and does not count internal BVH blocks.

- depth 1 exposes complete groups that can change current next;
- depth 2 exposes the next alternatives below those groups;
- depth 3 exposes three levels of possible mixed-depth decisions.

Lookahead is useful for more than prefetching. It lets applications compare
representations and intentionally jump over coarse resources to reduce
perceived latency.

The analysis is intended to run repeatedly. Resource completions improve
current; a later bounded query then starts from the improved cut and usually
has a smaller search space. With a stable view, feasible coverage groups, and
sufficient resources, this is a convergent bounded-horizon process.

## 8. Refinement API

Only two new public concepts are introduced:

- `FrontierRefinementView`, the non-owning result view;
- `SpatialQuery::computeFrontierRefinement()`, the explicit computation.

Nodes reuse `FrontierEntry`. Group ids reuse `NodeHandle`. No public refinement
node, refinement group, resource, direction, or planner-policy type is needed.

```cpp
class FrontierRefinementView
{
public:
    // Groups are indexed densely within this view.
    size_t groupCount() const;

    // Natural, generation-stamped group identifier.
    NodeHandle parent(uint32_t groupIndex) const;

    // Complete visible immediate-child cover for parent(groupIndex).
    std::span<const FrontierEntry> children(uint32_t groupIndex) const;

    // One-based logical transition distance below the current frontier.
    uint32_t depth(uint32_t groupIndex) const;

    // Returns kInvalidIndex if this node was not expanded within the limits.
    uint32_t findGroup(NodeHandle node) const;

    // Flat storage backing the child spans. Each entry belongs to one group.
    std::span<const FrontierEntry> entries() const;

    // Threshold used to encode FrontierEntry::errorCode().
    float threshold() const;

    // True when no depth or node limit prevented eligible mounted refinement.
    bool complete() const;

    bool depthLimitReached() const;
    bool nodeLimitReached() const;
    bool empty() const;
};

class SpatialQuery
{
public:
    static constexpr uint32_t UnlimitedDepth = UINT32_MAX;

    FrontierRefinementView computeFrontierRefinement(
        const SpatialDatabase& database,
        FrontierResultView current,
        uint32_t maxDepth,
        uint32_t maxNodes = UINT32_MAX);
};
```

Normal bounded use is explicit:

```cpp
FrontierResultView current =
    query.selectFrontier(database, camera, selectionParams);

FrontierRefinementView refinement =
    query.computeFrontierRefinement(database, current, 3);
```

Exhaustive use is also explicit:

```cpp
FrontierRefinementView refinement = query.computeFrontierRefinement(
    database,
    current,
    SpatialQuery::UnlimitedDepth);
```

Applying every returned group to its parent yields the terminal cut of the
computation. When `complete()` is true, those leaves form the exhaustive
threshold-directed target for the retained context and currently mounted
topology. They coincide with the old ideal cut wherever current has not already
moved below it. If `PreferReadyDescendants` selected a finer ready cover, that
cover remains authoritative rather than being coarsened back to the old ideal.
Thus debug inspection, full prewarming, offline capture, and exhaustive
streaming require no separate ideal selection API.

Unlimited traversal can be more expensive than the old ideal-only output
because it retains every intermediate group as well as the terminal leaves.
That is acceptable for an explicitly requested exhaustive workflow and is why
unlimited depth is not the default.

## 9. Logical result layout

The public view can use compact compressed adjacency internally:

```text
parents: [P, A, B, ...]
offsets: [0, 3, 5, 9, ...]
depths:  [1, 2, 2, ...]
entries: [A, B, C, A1, A2, ...]
```

For example:

```text
parent(0)   = P
children(0) = entries[0..3] = {A, B, C}

parent(1)   = A
children(1) = entries[3..5] = {A1, A2}
```

Groups are returned in nondecreasing `depth()` order. Within a depth,
database traversal order remains deterministic. Group indices are valid only
for one result view and must not be persisted across calls.

`findGroup()` connects an entry to its deeper alternatives. Its private lookup
representation remains an implementation choice to measure before freezing
the ABI.

## 10. Error representation

The initial API reuses the existing packed `FrontierEntry` error code instead
of adding a float to every refinement node. This provides:

- the exact above/below-threshold classification;
- a threshold-relative logarithmic magnitude for prioritization;
- the existing `approximateError(threshold)` decoding API;
- compact output for potentially large traversals.

Values are freshly evaluated using the retained damped context. They are not
copied from cached current entries. If real integrations demonstrate a need
for full float precision, a later API can expose an optional parallel stream
without changing group semantics or enlarging every `FrontierEntry`.

## 11. Limits and atomicity

### 11.1 Depth limit

`maxDepth` must be positive or `UnlimitedDepth`. Expansion stops only between
complete groups. A node at the depth boundary remains a valid intermediate
choice even when finer eligible topology exists.

`depthLimitReached()` is true when at least one over-threshold boundary entry
has finer mounted topology. It is conservative because child visibility beyond
the requested horizon is deliberately not evaluated merely to refine this
diagnostic. It is false for an empty result when current already satisfies the
retained refinement rule.

### 11.2 Node limit

`maxNodes` bounds output breadth as well as work. The query checks it before
emitting a group. If a complete child span would exceed remaining capacity:

- none of that group is emitted;
- every previously emitted group remains complete;
- enumeration stops in deterministic group order;
- `nodeLimitReached()` is true;
- `complete()` is false.

The API never truncates a sibling group. Depth does not bound breadth: current
may contain many parents and one parent may have high fanout. Applications that
need a hard work bound should supply both limits.

## 12. Contract and lifetime

`computeFrontierRefinement()` requires:

- `current` is the complete result of the immediately preceding
  `selectFrontier()` call on the same `SpatialQuery`;
- a fixed caller sink, if used, did not overflow;
- the exact current-result storage and entry bytes remain unchanged;
- `database` is the database to which the query is bound;
- the database remains in the same published read interval as selection;
- no database mutation overlaps selection or refinement computation;
- `maxDepth` is positive or `UnlimitedDepth`.

The method uses the query's retained effective camera, threshold, and database
binding. It accepts no camera or selection parameters, preventing disagreement
with the current cut.

Calling it does not:

- advance camera damping;
- alter exact-cut reuse state;
- change `reused()`, `walked()`, or `lastSelectionStats()`;
- record mounted-subtree usage;
- mutate readiness, topology, or publication state;
- invalidate the supplied current view.

The returned view points into lazily allocated query storage. It remains valid
until the next refinement computation, any selection on that query, `reset()`,
move assignment, or destruction. Its capacity is included in
`SpatialQuery::bytes()`. A query that never calls the method owns no refinement
buffers and pays no refinement-analysis work.

Applications normally copy only chosen `NodeHandle` values and resolved
`UserPayload` values into their long-lived asynchronous request systems. They
must not retain the non-owning view while loading occurs.

## 13. Streaming lifecycle

A normal integration performs:

1. publish database updates;
2. select the current cut for one view;
3. compute a bounded frontier refinement;
4. evaluate its group forest with external resource metadata;
5. choose a complete mixed-depth target cut;
6. resolve payloads only for missing target leaves;
7. load resources asynchronously;
8. call `markNodeReady()` as individual resources complete;
9. select again and treat the new current cut as authoritative;
10. recompute refinement when another streaming decision is needed.

There is no explicit group commit. Existing readiness and coverage logic keeps
the old parent selected while its chosen replacement is incomplete and
naturally advances current when a complete ready cover exists.

The application must not evict the parent merely because it requested a child
group. It may retire that resource according to its own sharing policy only
after later observations show no relevant current view still depends on it.

## 14. Multiple cameras

The API is intentionally single-view. One `SpatialQuery` represents one
coherent camera and retains the context needed to evaluate its refinement.

Split-screen, security-camera, reflection, portal, and shadow integrations run
selection and refinement independently on their existing per-view queries.
The application combines payload demand and protects current resources using
its own camera priorities and resource-sharing model.

This keeps view ids, priority aggregation, and resource identity out of
Frontier. The same parent can have different visible group membership in
different views; each result is authoritative for its own observation.

## 15. Complexity and performance expectations

Ordinary selection no longer partitions, caches, or stores a separate ideal
cut. It emits one current sequence. It may still traverse descendants needed
to determine that sequence under the chosen `CurrentCutPolicy`; the eliminated
work is the materialization and maintenance of a second public result plus any
walk performed solely for that result.

Explicit refinement work is proportional to:

- the current entries used as starting points;
- hierarchy blocks touched within the requested horizon;
- local child visibility and projected-error tests;
- complete groups and entries returned.

The method skips TLAS discovery and top-level culling because current already
identifies visible placements. A bounded call can stop far before the implicit
ideal leaves, which is the primary optimization.

The API does not promise small output. A shallow level may be wide, and
unlimited refinement may visit millions of nodes. `maxDepth` bounds logical
distance; `maxNodes` provides a deterministic breadth/output bound. Repeated
bounded calls are expected to become cheaper as readiness advances current.

## 16. Correctness invariants

An implementation must preserve:

1. every returned group has one live renderable parent;
2. every returned entry is an immediate runtime child of that parent;
3. each group contains every visible child needed for logical coverage;
4. no group is emitted partially;
5. each group is reachable by downward refinement from supplied current;
6. every error and visibility decision uses the originating query's retained
   effective view and per-placement error clamp;
7. group indices and spans remain self-consistent for the view lifetime;
8. refinement computation has no selection, readiness, topology, usage, or
   cache side effects;
9. stale or unrelated current results violate the documented contract rather
   than silently producing an unrelated traversal.

Hole-free means logical visible hierarchy coverage. It does not make claims
about mesh seams, cracks, occlusion, or rasterization.

## 17. Required test matrix

The implementation should cover at least:

- current already satisfies the threshold and produces an empty, complete
  refinement;
- one parent and one complete child group;
- several levels and `findGroup()` links;
- a mixed-depth target that skips an intermediate representation;
- one unavailable sibling keeping a parent in current;
- a ready descendant cover selected below an unavailable node;
- mounted-definition boundaries and recursive mounts;
- an over-threshold mountable terminal used for topology demand;
- different placements of one shared definition;
- dynamically overlaid bounds;
- damping and zoom changes;
- cached current paired with freshly evaluated refinement errors;
- frustum-boundary content and exact visible child membership;
- every finite depth boundary and `UnlimitedDepth`;
- node limits immediately below, exactly at, and above a group boundary;
- group fanout at the authored maximum;
- stale handles, stale results, mismatched queries, mismatched databases, and
  incomplete fixed-sink results;
- proof that calls leave selection stats, reuse accounting, mount usage,
  readiness, and topology unchanged;
- independent per-camera computation over one published snapshot;
- exhaustive refinement terminal leaves matching the old ideal cut where
  current is not already finer, and refinement-dominating it where current is.

## 18. Rejected API elements

### A materialized ideal frontier

Rejected from ordinary selection because bounded refinement can discover only
the decision horizon an application needs. Unlimited refinement covers debug,
prewarm, offline, and exhaustive uses explicitly. Ideal remains the internal
threshold/topology stopping rule rather than a stored public cut.

### `FrontierRefinementDirection`

Rejected because the operation only discovers downward quality improvements
from authoritative current. It does not coarsen current, and no direction
choice is needed.

### `FrontierRefinementNode`

Rejected because `FrontierEntry` already contains the handle, instance, and
screen-error code needed for one evaluated node. Readiness and payload remain
available through existing database methods.

### `FrontierRefinementGroup`

Rejected as a public storage type because a group is naturally its parent
`NodeHandle` plus a complete child span. The result view exposes that relation
directly while retaining compact private adjacency storage.

### Resource callbacks or planner policy

Rejected because applications have different allocation sharing, memory,
bandwidth, priority, cancellation, and eviction models. Frontier provides
structurally valid alternatives, not a universal planner.

### Camera input

Rejected because it could disagree with the damped or cached selection that
produced current. The originating query must retain the exact effective view.

### Multi-camera computation

Rejected because independent per-view queries compose on the user side, where
resource sharing and camera priority are known.

## 19. Open implementation questions

The following do not need to be frozen before prototyping:

- compact private representation for `findGroup()`;
- whether breadth-first order is produced directly or by reordering;
- how to validate current-result provenance cheaply in release builds;
- how best to retain the full effective camera without affecting queries that
  never request refinement;
- when sparse overlay blocks should use scalar lanes versus masked wide math;
- whether measured integrations justify an optional exact-float error stream;
- whether an owning refinement result or caller-memory sink is needed after
  experience with the query-owned view.

These choices do not change the responsibility boundary, group completeness,
or public refinement model.
