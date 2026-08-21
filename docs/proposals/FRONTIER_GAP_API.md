# Frontier gap computation API

Status: proposed; not implemented.

This document defines a bounded, policy-free API for inspecting the hierarchy
between the current and ideal frontiers produced by one `SpatialQuery`.
It records the design intent and draft public contract before implementation.

## 1. Motivation

`selectFrontier()` produces two useful endpoints:

```text
current = shared + currentOnly
ideal   = shared + idealOnly
```

The current cut is renderable now. The ideal cut is the desired cut for the
known topology if every definition node were ready. Those endpoints are enough
for rendering and simple streaming demand, but they do not describe the
intermediate choices available to a resource-constrained streamer.

The gap can be extremely large. In the limiting case, current contains one
root while ideal contains every terminal leaf. Loading the entire ideal cut may
exceed device memory, even though one of the hierarchy's intermediate cuts
would fit and improve quality substantially.

Individual ideal nodes are also the wrong unit for many streaming decisions.
A parent cannot leave a hole-free current cut until every visible branch that
it represents has a ready replacement. In the ordinary case, its relevant
children therefore form a group: one missing member can prevent all the other
members from affecting the current cut.

Frontier should expose enough topology and view-dependent error information to
let an application construct a useful intermediate cut. It should not choose a
memory budget, resource priority, loading order, or eviction policy for the
application.

## 2. Responsibility boundary

Frontier owns:

- relating the supplied current and ideal cuts through mounted runtime
  topology;
- finding intermediate renderable nodes between them;
- calculating view-dependent screen errors with the same query context used
  by selection;
- returning complete visible child groups, never partial sibling groups;
- bounding the analysis by a caller-supplied depth and optional node limit.

The application owns:

- external resource identity, byte cost, and sharing;
- memory and in-flight bandwidth budgets;
- quality, gameplay, and per-view priority policy;
- choosing a mixed-depth target cut from the returned groups;
- asynchronous loading, cancellation, request coalescing, and hysteresis;
- readiness publication and resource eviction;
- aggregation across cameras.

The existing `UserPayload` remains the only application data stored on a node.
It is sufficient to address arbitrary external resource metadata. The gap API
does not add resource sizes, keys, priorities, or residency records to the
hierarchy, and it does not assume equal payload values represent one resource.

## 3. Goals

The proposed API must:

1. add no work to ordinary `selectFrontier()` calls;
2. live on `SpatialQuery`, which owns the exact damped view context;
3. consume both cuts from an existing `FrontierResultView`;
4. walk from current toward ideal through only the relevant topology corridor;
5. return existing `FrontierEntry` values for gap nodes;
6. identify a group by the existing `NodeHandle` of its parent;
7. expose complete immediate-child covers at every returned parent;
8. support several levels of analysis so a caller may skip intermediate
   representations;
9. preserve valid group boundaries under every output limit;
10. remain a read-only, policy-free building block.

## 4. Non-goals

The API does not:

- select a budgeted frontier automatically;
- mutate readiness, topology, or query selection state;
- start, cancel, or track asynchronous resource requests;
- decide whether an intermediate or terminal representation is preferable;
- expose a built-in multi-camera planner;
- enumerate topology that has not been mounted yet;
- replace existing topology-demand handling for over-threshold mountable ideal
  nodes;
- guarantee that a gap is small merely because its traversal is bounded.

## 5. Core model

### 5.1 The topology corridor

The frontier gap is the minimal mounted, visible topology corridor connecting
the two supplied cuts. Shared entries require no gap analysis. Current-only
and ideal-only entries establish the endpoints of every divergent region.

Visibility is not recomputed. A hierarchy branch is relevant when it leads to
an endpoint already selected by the originating `SpatialQuery`. This lets gap
computation skip TLAS traversal, view-mask checks, contribution culling,
frustum tests, readiness fallback search, and current/ideal cut construction.

The query still performs the operations needed for screen error:

- transform the retained damped camera envelope into each relevant placement;
- read effective bounds, including instance-local overlays;
- apply the effective error clamp across mount boundaries;
- calculate bound distance and projected geometric error.

### 5.2 Groups

A group is identified by the `NodeHandle` of a renderable parent. Its members
are the complete set of relevant immediate children that cover the visible
branches represented by that parent for this query.

The group is view-specific. It need not contain authored children that are
irrelevant to the supplied cuts. The same parent may consequently have a
different returned membership in another camera or frame. The parent handle is
a natural group anchor, not a permanent identity for one immutable member set.

Every member is an ordinary `FrontierEntry`. It carries the runtime node
handle, top-level instance id, and threshold-relative screen-error code already
used by frontier results.

### 5.3 Recursive alternatives and multi-level jumps

Suppose bounded analysis returns:

```text
P -> {A, B, C}
A -> {A1, A2}
```

The application may choose the valid intermediate target:

```text
{A, B, C}
```

or skip `A` and choose:

```text
{A1, A2, B, C}
```

Only the leaves of the chosen refinement forest become resource requests. In
the second target, `A` is an evaluated alternative, not a required resource.

Coverage is recursive. The branch represented by `A` is covered when `A` is
ready or when the complete selected descendant cover `{A1, A2}` is ready.
Consequently `P` can leave the current cut once `B`, `C`, `A1`, and `A2` are
ready even if `A` was never loaded. This matches Frontier's existing
ready-descendant behavior.

### 5.4 Search direction

Gap computation always proceeds semantically from current toward ideal. It
does not expose a direction option.

This is usually a physical top-down refinement, but current can contain ready
descendants below an unavailable ideal node under
`PreferReadyDescendants`. In those regions the same current-to-ideal search
moves physically upward. The returned data remains a structural parent-to-
children description; the original cuts establish which endpoint is current
and which is ideal. No `FrontierGapDirection` public type is necessary.

### 5.5 Bounded decision horizon

`maxDepth` counts logical parent/child transitions from the current cut toward
the ideal cut. It is not absolute authored-tree depth and does not count
internal BVH blocks.

- depth 1 exposes the complete groups capable of changing current next;
- depth 2 exposes their next alternatives;
- depth 3 exposes three levels of possible intermediate-cut decisions.

Several levels are useful for more than prefetching. They allow the application
to compare representations and intentionally jump over coarse resources to
reduce perceived latency.

The analysis is intended to run repeatedly. Resource completions improve the
current cut; the next frontier selection therefore narrows the gap and gives a
later bounded query a smaller decision space. With a stable view, feasible
coverage groups, and sufficient resources, this forms a convergent bounded-
horizon process without requiring one complete traversal of a million-node gap.

Partial loading may not shrink the current cut. If one required branch remains
uncovered, its ready siblings cannot replace their parent. Applications should
therefore retain group membership and commonly prioritize completing a
partially loaded group over starting many unrelated incomplete groups.

## 6. Draft public API

Only two new public concepts are proposed:

- `FrontierGapView`, the non-owning result view;
- `SpatialQuery::computeFrontierGap()`, the explicit computation.

Gap nodes reuse `FrontierEntry`. Group ids reuse `NodeHandle`. No public gap
node, gap group, direction, resource, or planner-policy type is introduced.

```cpp
class FrontierGapView
{
public:
    // Groups are indexed densely within this view.
    size_t groupCount() const;

    // Natural, generation-stamped group identifier.
    NodeHandle parent(uint32_t groupIndex) const;

    // Complete relevant immediate-child cover for parent(groupIndex).
    // Every entry contains a screen-error code evaluated for the originating
    // SpatialQuery's most recent damped view context.
    std::span<const FrontierEntry> children(uint32_t groupIndex) const;

    // One-based logical transition distance from current toward ideal.
    uint32_t depth(uint32_t groupIndex) const;

    // Finds the group whose parent is node. This connects an intermediate
    // entry to its deeper alternatives. Returns kInvalidIndex when no group
    // for node was returned within the requested limits.
    uint32_t findGroup(NodeHandle node) const;

    // Flat storage containing every entry returned by children(). Each entry
    // belongs to exactly one immediate-parent group.
    std::span<const FrontierEntry> entries() const;

    // Threshold used to encode FrontierEntry::errorCode().
    float threshold() const;

    // True only when every divergent path reached its ideal endpoint within
    // the requested bounds.
    bool reachesIdeal() const;

    // True when maxNodes prevented at least one whole group from being
    // returned.
    bool nodeLimitReached() const;

    bool empty() const;
};

class SpatialQuery
{
public:
    FrontierGapView computeFrontierGap(
        const SpatialDatabase& database,
        FrontierResultView frontier,
        uint32_t maxDepth,
        uint32_t maxNodes = UINT32_MAX);
};
```

The intended call sequence is:

```cpp
FrontierResultView frontier =
    query.selectFrontier(database, camera, selectionParams);

FrontierGapView gap =
    query.computeFrontierGap(database, frontier, 3);
```

The name `computeFrontierGap()` is deliberate. The method performs meaningful
topology traversal and screen-error computation; `getFrontierGap()` would
incorrectly suggest retrieval of already available state. `selectFrontierGap()`
would suggest another LOD selection operation, while the method remains a
policy-free analysis of two cuts already selected by `SpatialQuery`.

## 7. Logical result layout

The public view can be backed by a compact compressed-adjacency layout:

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

This storage is an implementation detail. It explains why no public
`FrontierGapGroup` is needed: a group index addresses one parent plus one
complete entry span, while the parent handle remains its natural external id.

Groups should be returned in nondecreasing `depth()` order. Within one depth,
database/frontier traversal order should remain deterministic. Group indices
are valid only for one `FrontierGapView` and must not be persisted across calls.

`findGroup()` may use an internal compact lookup built with the result. Its
exact representation and complexity are implementation choices to be measured
before the ABI is frozen.

## 8. Error representation

The initial API uses the existing `FrontierEntry` error code rather than adding
a float to every gap node. This provides:

- the exact above/below-threshold classification;
- a threshold-relative logarithmic magnitude suitable for prioritization;
- the existing `approximateError(threshold)` decoding API;
- compact output for large corridors.

The values returned for gap entries are freshly evaluated using the latest
damped view context retained by the originating `SpatialQuery`. They are not
copied from a cached frontier record. The current and ideal endpoint entries
remain available in the supplied `FrontierResultView`.

If profiling and real integrations show that streaming policy requires full
float precision, a later API can expose an optional parallel float-error stream
without increasing the base `FrontierEntry` or changing group semantics. That
need should be demonstrated before it becomes part of the base result layout.

## 9. Limits and atomicity

### 9.1 Depth limit

`maxDepth` must be positive. The query stops expanding a branch only between
complete groups. A node at the depth boundary remains a valid intermediate
choice even when its path continues toward ideal.

`reachesIdeal()` is false when any divergent path extends beyond the returned
depth. It is true for an empty gap because the two cuts already agree.

### 9.2 Node limit

`maxNodes` is an optional deterministic work and output bound. Its default
leaves output bounded only by `maxDepth`.

The query checks the limit before emitting a group. If the group's complete
child span would exceed the remaining capacity:

- none of that group is emitted;
- every previously emitted group remains complete;
- enumeration stops in deterministic group order;
- `nodeLimitReached()` is true;
- `reachesIdeal()` is false.

The query never returns a truncated child span. Depth alone does not bound
breadth: one parent may have hundreds of relevant children, and the current cut
may contain many divergent parents. The node limit is therefore a useful
safety bound even though depth remains the primary semantic control.

## 10. Method contract and lifetime

`computeFrontierGap()` requires:

- `frontier` is a complete result produced by the immediately preceding
  `selectFrontier()` call on the same `SpatialQuery`;
- fixed caller sinks, if used, did not overflow either logical cut;
- `database` is the same database to which the query is bound;
- the database remains in the same published read interval as selection;
- no database mutation overlaps selection or gap computation;
- `maxDepth` is greater than zero.

The method uses the query's retained damped camera envelope, projection scale,
threshold, and database binding. It accepts no `Camera` or `SelectionParams`,
which prevents the analysis from disagreeing with its input cuts.

Calling `computeFrontierGap()` does not:

- advance camera damping;
- alter the exact-cut reuse cache;
- change `reused()`, `walked()`, or `lastSelectionStats()`;
- record mounted-subtree usage;
- mutate readiness, topology, or database publication state;
- invalidate the supplied `FrontierResultView`.

The returned view points into lazily allocated `SpatialQuery` storage. It
remains valid until the next `computeFrontierGap()`, any selection on that
query, `reset()`, move assignment, or destruction. Gap storage capacity is
included in `SpatialQuery::bytes()`. A query that never calls the method owns no
gap buffers and pays no gap-analysis work.

An application normally copies only the chosen `NodeHandle` values and their
resolved `UserPayload` values into its long-lived asynchronous request system.
It should not retain a `FrontierGapView` while loading occurs.

## 11. Streaming lifecycle

A normal integration performs:

1. publish database updates;
2. select current and ideal cuts for one view;
3. compute a bounded frontier gap;
4. evaluate the returned group forest with application resource metadata;
5. choose a complete mixed-depth target cut;
6. resolve payloads only for the target cut's missing leaves;
7. load resources asynchronously;
8. call `markNodeReady()` as individual resources complete;
9. select again and treat the new current cut as authoritative;
10. recompute the now-current gap when another streaming decision is needed.

There is no explicit group commit. Frontier's existing readiness and coverage
logic keeps the old parent selected while its chosen replacement is incomplete
and naturally advances current when a complete ready cover exists.

The application must not evict the parent merely because it requested a child
group. It may retire that resource according to its own sharing policy only
after later frontier observations show that no relevant current view still
depends on it.

## 12. Multiple cameras

The gap API is intentionally single-view. One `SpatialQuery` represents one
coherent camera and owns the exact view context needed to evaluate its gap.

Split-screen, security-camera, reflection, portal, or shadow integrations call
`selectFrontier()` and `computeFrontierGap()` independently on their existing
per-view queries. The application combines the resulting payload demand and
protected current resources according to its own view priorities and sharing
model.

This composition avoids adding view ids, fixed view masks, priority aggregation,
or resource-identity assumptions to Frontier. The same parent handle may have
different visible group membership in different views; each view's returned
span remains authoritative for that observation.

## 13. Complexity and performance expectations

Ordinary selection receives no additional traversal, output, or per-entry
metadata. Gap computation is an explicit lower-frequency call.

Expected work is proportional to:

- scanning the supplied cut endpoints needed to identify divergent regions;
- the mounted hierarchy blocks touched within the bounded corridor;
- the number of complete groups and entries returned.

The implementation should reuse the database's existing wide bound/error math
and process touched sibling blocks together. It should not enter the TLAS or
redo visibility tests.

The API does not promise that the output is always small. When current is a
root and ideal contains a million visible leaves, even a shallow level can be
wide, and the supplied ideal cut is already large. `maxDepth` bounds how far
the query walks between endpoints; `maxNodes` provides a separate deterministic
breadth/output bound. Repeated bounded computation is expected to shrink the
intermediate search as readiness moves current toward ideal.

## 14. Correctness invariants

An implementation must preserve:

1. every returned group has one live renderable parent;
2. every returned entry is an immediate runtime child of that parent;
3. a group contains every relevant child needed for visible logical coverage;
4. no group is emitted partially;
5. every entry lies on the mounted corridor between the supplied cuts;
6. every error uses the originating query's latest damped view context and
   effective per-placement error clamp;
7. group indices and spans remain self-consistent for the view lifetime;
8. gap computation has no selection, readiness, topology, usage, or cache side
   effects;
9. stale or unrelated frontier results fail the documented contract rather
   than silently producing an unrelated corridor.

Hole-free continues to mean logical visible hierarchy coverage. This API does
not make claims about mesh seams, geometric cracks, occlusion, or rasterization.

## 15. Required test matrix

The implementation should cover at least:

- identical current and ideal cuts produce an empty gap with
  `reachesIdeal() == true`;
- one parent and one complete child group;
- several refinement levels and `findGroup()` links;
- a mixed-depth target that skips an intermediate representation;
- one unavailable sibling keeping a parent in current;
- ready siblings that remain ideal-only behind the same parent;
- current ready descendants below an unavailable ideal ancestor;
- mounted-definition boundaries and recursively mounted definitions;
- different placements of one shared definition;
- dynamically overlaid bounds;
- query damping and zoom changes;
- cache-reused cuts paired with freshly evaluated gap errors;
- frustum-boundary content where only the relevant visible child cover appears;
- every `maxDepth` boundary;
- `maxNodes` immediately below, exactly at, and above a group boundary;
- group fanout at the authored maximum;
- stale handles, stale results, mismatched queries, mismatched databases, and
  incomplete fixed-sink results;
- proof that gap calls leave selection stats, reuse accounting, mount usage,
  readiness, and topology unchanged;
- independent per-camera computation over the same published snapshot.

## 16. Rejected API elements

### `FrontierGapDirection`

Rejected because the gap result describes hierarchy rather than commands. The
operation is always semantically current-to-ideal, and the supplied cuts retain
endpoint meaning. A public direction type would duplicate information and make
the building block look more like a policy engine.

### `FrontierGapNode`

Rejected because `FrontierEntry` already stores the exact result needed for an
evaluated gap node: handle, instance, and screen-error code. Readiness and
payload remain available through existing `SpatialDatabase` methods.

### `FrontierGapGroup`

Rejected as a public storage type because a group is naturally its parent
`NodeHandle` plus a complete child span. `FrontierGapView` can expose that
relationship directly while retaining a compact private adjacency layout.

### Resource-cost callbacks or planner policy

Rejected because applications have different allocation sharing, memory,
bandwidth, priority, cancellation, and eviction models. The library should
provide structurally valid alternatives, not a supposedly universal planner.

### Camera input to gap computation

Rejected because it could disagree with the damped or cached selection that
produced the supplied cuts. The originating `SpatialQuery` already owns the
correct envelope and projection context.

### Multi-camera gap computation

Rejected because independent per-view queries already compose on the user
side, where resource sharing and camera priority are known.

## 17. Open implementation questions

The public behavior above does not require these choices to be fixed before a
prototype:

- the compact internal representation used by `findGroup()`;
- whether grouping is assembled breadth-first directly or reordered after a
  bounded walk;
- how endpoint membership is marked most cheaply for cached and uncached
  frontier layouts;
- when touched sparse overlay blocks should use scalar lanes versus masked wide
  evaluation;
- whether a future optional exact-float error stream is justified by real
  planner policies;
- whether an owning result or caller-memory sink is needed after experience
  with the primary query-owned view.

These are implementation and performance questions. They do not change the
responsibility boundary, group identity, completeness contract, or core API.
