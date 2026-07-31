# HLodTree — external hierarchical LOD cut selection

> All code here is **illustrative pseudocode**, not compilable C++. Allocation, error handling, const-correctness and API polish are deliberately omitted so the structure stays readable.

## 0. The idea

Per-object LOD swaps a distant wall's mesh for a cheaper one, but you still pay a draw call per wall. A distant town costs thousands of draws no matter how simple each mesh is. Hierarchical LOD fixes the count, not just the triangles.

**The tree.** Every node is a *complete, self-sufficient rendering* of everything beneath it. The root draws the whole town as one coarse proxy mesh. A building node draws that entire building as one mesh. A leaf draws one wall at full detail. Detail increases as you descend; cost increases with it.

**The cut.** Each frame we pick a horizontal slice through the tree and draw exactly the nodes on it. Because every node stands in for its whole subtree, a cut covers the world exactly once — no gaps, no overlap. The cut is view-dependent and ragged: it goes deep in branches near the camera and stays shallow in branches far away.

```
                     town                      <- 1 proxy for everything
                  /        \
           buildingA        buildingB   <===== cut: B is far, draw its 1 id
            /  |  \
   ===== w01  w02  w03                  <===== cut: A is close, draw its 3 walls
```

Here the frame selects four ids: `w01`, `w02`, `w03`, `buildingB`. Note `buildingA` is *not* selected — its children replaced it.

**The decision.** Per node, per frame, there is exactly one question: *do I draw myself, or do my children draw instead of me?* Everything in `selectCut` exists to answer that. A node refines when it's too coarse for its size on screen — and only when its children are actually available.

**Refinement is REPLACE only.** A drawn node fully supersedes its subtree; a parent and its descendant are never both drawn. This is a fixed decision, not a configurable mode — it is what lets the cut be a simple antichain and what makes streaming sound.

**The structure is completely external.** `HLodTree` owns no meshes, materials, or render state. Each node carries one caller-supplied **stable numeric id** (`uint64_t`), and `selectCut()` returns lists of those ids. What an id means — a mesh, an instance batch, an imposter — is entirely the caller's business. The one contract this imposes: *every id you register must be renderable on your side*, because any node can end up on the cut, and a cut node that draws nothing is a hole in the world.

## 0.1 Scale requirements — what "fast" means here

The design is driven by four hard requirements:

1. **Unbounded topology.** Planet → continents → cities → buildings → walls. The full tree never fits in memory; its shape must stream (collapse/expand), not just its meshes.
2. **Streaming residency.** Node payloads load and evict at runtime; the cut must stay hole-free under any residency state, and the caller controls residency through an explicit API.
3. **Motion.** Some nodes move. Updates must be **sublinear** — never a full-tree pass.
4. **Sublinear cut selection**, for both very deep trees and huge forests of shallow trees (multi-root is a first-class case). Frustum culling included: no O(N) sweep anywhere per frame.

One honest bound first: nothing beats output size. If a frame's cut has K nodes, selection is Ω(K); if a million trees are genuinely on screen and each must draw, no algorithm is "fast." The achievable target — and the one this design hits — is **output-sensitive** cost:

> per view, per frame: `O(log R + visible roots + ideal-cut region)` — independent of total node count N.

Everything below the cut, everything outside the frustum, and everything not yet expanded costs nothing. The escape hatches for "a million trees visible" are contribution culling and aggregation, both covered in §5.

Note the bound counts *emitted ids*; the per-node constant inside the visible region is squeezed separately by tri-state culling — early reject, early **accept** (a box fully inside the frustum frees its whole subtree from further frustum tests), keep testing only while partially visible. §8 folds this in via plane masks.

## 0.2 Architecture in one paragraph

Two levels, in the spirit of a ray tracer's TLAS/BLAS split. The **bottom level** is a forest of immutable, flat, SoA **pages** — each page a few hundred to a few thousand nodes, built offline by `HLodBuilder`, loaded as one blob. Pages link to child pages through **expansion points**, which is how planet-scale topology streams. The **top level** is a small dynamic BVH over **root instances** (transform + reference to a tree); it owns motion, multi-root, insertion/removal, and coarse culling. `selectCut` walks the top level with the frustum, then runs one pruned pass per surviving instance, emitting the actual cut, the ideal cut, and load requests as it goes.

---

## 1. What the caller sees

```cpp
// --- authoring (offline) ---
HLodBuilder b;
NodeId planet = b.createRoot (planetId, planet_ge, planet_bbox);
NodeId europe = b.createNode (planet, europeId, europe_ge);
b.markExpansion(europe, europePageRef);       // children live in another page
Page page = b.build();                        // one immutable blob

// --- world assembly (runtime) ---
// InstanceRef{id, generation, rootPage}: generation-stamped like NodeHandle.
// Instance slots are recycled; a ref that outlives its instance is a safe
// no-op and can never act on the slot's new occupant.
InstanceRef inst = world.addInstance(rootPage, transform);      // O(log R)
world.moveInstance(inst, newTransform);                         // O(log R); stale => no-op
world.removeInstance(inst);                                     // O(log R); stale => no-op

// --- streaming (runtime) ---
// HANDLE-ONLY API: every call takes a NodeHandle{slot, index, generation}.
// Handles come out of selectCut (requests/expansions carry them) or are
// composed as nodeAt(pageHandle, authoredIndex) — page-local indices are
// fixed at build time, so the content pipeline knows them. The World keeps
// NO id index: no hash map, no lookup, ever. Stale handles (page detached
// or GC'd since) fail the generation check and are safely ignored.
world.markResident   (req.node);              // payload loaded,   O(1)
world.markNonResident(handle);                // payload evicted,  O(1)
PageHandle ph = world.attachPage(exp.node, page);   // expand: topology arrives
world.detachPage     (exp.node);              // collapse: topology leaves
world.collect(lowWatermark, minAge, outFreed); // GC: collapse cold subtrees
                                               //   from the LRU tail (§4.1)

// --- motion (runtime) ---
// LAZY: ~16 ns queue push per call, applied at the next selectCut (or an
// explicit flushBounds()). Move a node as many times a frame as you like —
// the tree is only brought up to date where that matters: at the cut.
world.setNodeBounds(handle, newLocalBounds);
world.flushBounds();                          // optional: tools/tests only

// --- per frame, per view ---
world.selectCut(view, threshold, hysteresis, minPix, scratch,
                outCut,        // [(payload, screenError)] — draw these
                outIdealCut,   // [(payload, handle, screenError, DIRECT |
                               //   NEEDS_EXPANSION)] — cut if everything
                               //   were loaded (optional)
                outRequests);  // [(payload, handle, priority)] — payload
                               //   loads wanted at the cut's frontier (optional)
```

**Payloads are opaque.** Each node carries a caller-supplied 64-bit `UserPayload` — an id, a pointer, an index into the caller's tables; the World never interprets it and never indexes by it, it is simply echoed in outputs. The renderer keys meshes by it, the streamer keys disk content by it — with their own data structures. If caller code ever needs payload→handle resolution, it builds that map itself (and usually doesn't need to: every handle it must act on came out of `selectCut`, and setup-time handles are composed from `addInstance`/`attachPage` results plus authored node indices).

`outIdealCut` and `outRequests` are nullable: a caller that doesn't stream (static, fully resident world) passes nothing and pays for exactly one output — at large cut sizes skipping the second emission is worth ~35% of the whole call (ARCHITECTURE.md, experiment G).

`selectCut` does not render and does not allocate. Returning the cut lets the caller cache it (it is highly temporally coherent), sort it by material/queue, and reuse it per pass. `screenError` is returned per cut node because the caller's own per-object LOD selection wants the same number — one error budget, computed once.

**The ideal cut** is the cut that would have been selected if all topology were expanded and all payloads resident *within known topology*. Each entry carries one of two tags:

- **DIRECT** — this node genuinely belongs on the ideal cut: its error is satisfied, render it as-is. Note a *collapsed* expansion point lands here whenever its proxy is good enough for the current view — that is the desired steady state, not a deficiency: a planet seen from space keeps its continents collapsed indefinitely, and no expansion traffic is generated.
- **NEEDS_EXPANSION** — a collapsed node whose error test demands refinement. Below it there are no ids to name — nobody knows them yet — so the node stands in for its unknown descendants, tagged as provisional. The tag *is* the expansion request: screen error is the priority, and there is no separate expansion entry in `outRequests`.

`idealCut − actualCut` is the streamer's prefetch target; `outRequests` carries the payload loads at the actual cut's frontier — the immediate next step toward the ideal cut. The frontier form matters because all-or-nothing refinement needs the *interior* nodes between the two cuts resident too, and those appear in neither cut; as the streamer satisfies each frame's frontier, the actual cut advances and the next layer of requests surfaces. A streamer therefore consumes both: NEEDS_EXPANSION tags from the ideal cut, load requests from `outRequests`.

**Output order is traversal-defined and must not be relied upon.** In particular, a streamer with a per-frame attach budget must pick NEEDS_EXPANSION entries by *priority* (screen error, descending), never "the first N in the vector": the walk emits deepest-first, so positional selection concentrates the whole budget on whatever region the walk refined first and starves coarse coverage elsewhere — measured as a permanently churning hot set instead of a converging one (see ARCHITECTURE.md, experiment B2).

**Predict expand depth; don't discover it one level per frame.** The walk cannot see below a missing page, so a streamer that only attaches what the current frame requested pays one frame of latency per page level: after a teleport, a chain D pages deep takes D frames no matter how large the budget. But the request's error already says how deep the chain goes — `levels ≈ ceil(log2(err / threshold) / d)` for content whose error shrinks 2^d per page level; a 100 px entry against a 4 px threshold is more than one level with certainty. The recommended streamer keeps a max-heap of candidates keyed by screen error: seed it with the frame's NEEDS_EXPANSION entries, pop the worst, attach its page, estimate the fresh expansions from the page's own data (`geomError · k / distance(bbox, camera)`), push the over-threshold ones, repeat until the budget runs out. Everything needed is already in hand — `attachPage` returns the `PageHandle`, `nodeAt` composes the next level's expansion handles, and the page content is the streamer's own — so multiple levels land in a single frame while staying in global priority order. Two rules matter: keep the heap *global* (naive depth-first chain-expansion lets one hotspot's ×fanout² candidates swallow the whole frame budget and measurably loses to plain discovery), and don't fear over-prediction (a wasted page ages out through GC; a missed one falls back to discovery). Measured after a teleport at equal budget: worst residual error two frames in drops ~120×, converging near-field detail in one attach frame instead of D (ARCHITECTURE.md, experiment N). Never bound this with a far-plane cull instead: a collapsed node renders coarse until its page arrives — distant regions deserve slower convergence, not invisibility.

---

## 2. Page layout

```cpp
// One page: an immutable flat subtree fragment, built by HLodBuilder::build(),
// loaded from disk as a single blob. All arrays index-parallel, page-local.
//
// Layout invariants, guaranteed by build() (per page):
//
//   (A) Preorder: parent[i] < i. A parent's per-frame state is always already
//       computed when we reach the child — the walk is a forward loop,
//       no stack, no recursion.
//
//   (B) Contiguous subtrees: node i and all its descendants in this page
//       occupy [i, i + subtreeSize[i]). Skipping a branch is one add.
//
//   (C) bbox[i] contains every descendant's bbox — *conservatively*.
//       Bounds (bbox + the wide-block lanes mirroring them) are the only
//       mutable data (see §6); they may be loose, never too small.
//
//   (D) geometricError never increases going down. Enforced in-page by build(),
//       across pages by a clamp at attach time (§3).
//
// Index 0 is a sentinel standing in for the node that owns this page (the
// expansion point above it; a virtual parent for a root page). It deletes the
// `parent == INVALID` branch from the inner loop and gives the parent page an
// O(1) place to read "are this page's roots ready" (§4).
struct Page
{
    // ---- hot: read for every VISITED node --------------------------------
    std::vector<uint32_t> parent;       // (A)
    std::vector<uint32_t> subtreeSize;  // (B)  >= 1
    std::vector<uint32_t> meta;         // one packed word: childCount |
                                        //   EXPANSION flag | wide-block offset.
                                        //   Fits because pages are bounded;
                                        //   build() asserts it.

    // ---- hot: read only when a node REFINES ------------------------------
    // Wide child blocks — the BVH4/BVH8 pattern (Embree-style). A refining
    // node tests ALL its children in one SIMD issue: lanes are children,
    // fields are SoA-transposed. Fanout > W chains ceil(count/W) blocks;
    // one interior node's blocks are contiguous.
    std::vector<WideBlock> wide;

    // ---- cold: emission, refit, attach — never in the inner loop ---------
    std::vector<uint64_t> userId;          // stable caller ids — the only
                                           //   thing selectCut ever outputs
    std::vector<AABB>     bbox;            // (C) source of truth for refit;
                                           //   mutable, conservative
    std::vector<float>    geometricError;  // (D) source of truth; the wide
                                           //   lanes are the hot copies
    SmallMap<u32, PageRef> expansion;      // local index -> child page.
                                           //   Expansion points are rare, so
                                           //   a side table, not an array

    uint32_t nodeCount() const { return uint32_t(parent.size()); }
};

// W = SIMD width, a build-time parameter of the page format (4 for
// SSE/NEON, 8 for AVX2). Unused lanes hold empty boxes that fail every test.
struct WideBlock
{
    float    minX[W], minY[W], minZ[W];   // children's bounds, one child
    float    maxX[W], maxY[W], maxZ[W];   //   per lane
    float    error[W];                    // children's geometric error
    uint32_t child[W];                    // children's local indices
};
```

A node's children are either all local or all behind one child page — never mixed (`EXPANSION` implies `childCount == 0`). The per-visited-node working set is deliberately three 4-byte streams (`parent`, `subtreeSize`, `meta`) plus scratch; a node's own bounds and error are not read at its visit at all — its *parent* computed them, W children at a time, from the wide block (§8). The duplication is the price: child bounds live both in the parent's lanes (hot) and in `bbox[]` (cold, for refit and attach-time asserts), and movement must patch both (§6). (Separate `vector`s are for readability — a real page is one blob with spans into it, so loading a page is one read and no fix-up beyond §3's clamp.)

Internal node addressing is `(PageRef, localIndex)` — called `NodeRef` below. It never leaks to the caller; outputs are always `userId`.

---

## 3. Topology streaming — expansion points

A node whose children live in another page is an **expansion point**: `childCount == 0`, `EXPANSION` flag set in `meta`, child page in the side table. While the child page is not loaded, the node *is* a leaf as far as traversal cares — it draws itself, and as long as its proxy satisfies the error test that is a perfectly good steady state. When the error test says it's too coarse, it enters the ideal cut tagged `NEEDS_EXPANSION` (§1); when the streamer attaches the page, traversal descends across the boundary next frame. Collapse is `detachPage` — the node becomes a leaf again, one frame of coarser detail, steady state.

This is residency generalized to topology, and it deliberately reuses the same machinery (§4). Consequences:

- **Expansion points must be renderable.** A collapsed node acts as a leaf and will be drawn; a collapsed continent with no proxy is a hole in the planet. This is the narrow, mandatory form of the old "every node owns geometry" invariant — for plain nodes renderability is a caller contract, for expansion points it is non-negotiable.
- **Cross-page invariants are enforced at attach time**, since pages are built independently:

```cpp
PageHandle attachPage(NodeHandle expansion, Page page)
{
    // A stale expansion handle (its page was detached/collected while this
    // page was being built) is rejected by returning an invalid PageHandle
    // — the normal streaming race, not an error.

    // (D) across the boundary: clamp the child page's errors to the expansion
    // node's error, same forward sweep as build() pass C. Assert in debug so
    // bad content is reported at its source; clamp always so it still works.
    e = geometricError(expansion);
    for (i = 1; i < page.nodeCount(); ++i)
        page.geometricError[i] = min(page.geometricError[i],
                                     page.geometricError[page.parent[i]]);
    page.geometricError[0] = e;   // sentinel carries the owner's error

    refreshWideErrorLanes(page);  // wide blocks hold hot copies of the
                                  // errors — re-mirror after the clamp

    assert(bbox(expansion).contains(page.bbox[1..]));   // (C) across the boundary

    hookUpResidency(expansion, page);                    // §4
}
```

- **A frustum-culled collapsed branch costs nothing at all** — it isn't even in memory. Strictly better than skipping it.
- **Detach only between frames, never mid-traversal.** Same rule as payload eviction.

Page size is a tuning knob: bigger pages amortize boundary crossings, smaller pages give finer memory control. Hundreds to low thousands of nodes is the expected range (§10).

---

## 4. Residency — payloads and topology, one mechanism

Two independent facts per node, one combined O(1) test:

- **resident** — the caller has this node's payload loaded (`markResident` / `markNonResident`).
- **materialized** — this node's children exist in memory: trivially true for local children, "page attached" for expansion points.

The refinement rule is **all-or-nothing**: a node refines only if its children are materialized *and every child is resident*. Otherwise it draws itself and requests what's missing. Refining into a partial child set would leave holes where the missing siblings should be; falling back per-child would let an ancestor and a resident cousin draw on top of each other.

```cpp
// Mutable, parallel to each page's nodes. Separate from Page so pages stay
// immutable blobs.
struct PageResidency
{
    std::vector<uint8_t>  resident;      // payload loaded
    std::vector<uint32_t> readyChildren; // how many of node i's children are resident

    // Incremental counters make the "can I refine?" test O(1) in the walk
    // instead of a loop over children. A page's top-level nodes have
    // parent == 0, so the sentinel's counter is exactly what the *parent
    // page's* expansion point reads — the same mechanism spans the boundary.
    void onLoaded (uint32_t i) { resident[i] = 1; readyChildren[parent[i]]++; }
    void onEvicted(uint32_t i) { resident[i] = 0; readyChildren[parent[i]]--; }
};

bool canRefine(page, res, i)     // O(1), both cases
{
    n = childCountOf(page.meta[i]);
    if (n > 0)
        return res.readyChildren[i] == n;
    if (isExpansion(page.meta[i]) && isAttached(page.expansion[i]))
        return childRes.readyChildren[0] == childCountOf(childPage.meta[0]);
    return false;                // leaf, or collapsed expansion point
}
```

Every instance's root is **pinned** — root page attached, root payload resident, never evicted. That plus all-or-nothing refinement gives the invariant everything else leans on:

> **(F)** if a node is drawn, or is refined through, it is resident and materialized.

Proof is one line: a node is only reached because its parent refined, and its parent only refined because all its children were ready. The pinned root is the base case. Eviction needs no special handling — dropping a payload or a page decrements a counter, so next frame the parent stops refining and draws itself. **Evict between frames, never mid-traversal.**

## 4.1 Garbage collection — keeping expansion bounded

Expansion is one-way traffic without a counterweight: fly around a planet long enough and every page ends up attached. The counterweight is an LRU over attached pages, owned by the structure itself — it is the only party that knows what the walks actually touch.

```cpp
// Global usage clock, one per world (not per view): a page counts as used
// if ANY view's walk reached it this frame — shadow cascades keep pages
// alive, correctly, since their cuts need them too.
//
// Bookkeeping is O(1) and lives in the hook every walk already passes
// through: the FIRST view to touch a page this frame relinks it to the LRU
// head; later views see lastTouched == frame and do nothing.
//
// lastTouched and the list links live in the per-page RUNTIME record (next
// to PageResidency, §4) — the Page blob itself stays immutable.
scratch.forPage(page):
    rt = world.runtime(page);
    if (rt.lastTouched != world.frame)
    {
        rt.lastTouched = world.frame;
        world.lru.moveToHead(rt);             // intrusive list, O(1)
    }
    ...

// Called by the streamer when memory crosses a high watermark; detaches
// pages from the LRU tail until below a low watermark (the gap is the
// hysteresis that stops collapse/expand churn at the budget line).
//
// A tail page is skipped if:
//   - it is pinned (an instance's root page), or
//   - it was touched within minAge frames (dwell: a camera swing might
//     want it right back), or
//   - it still has attached child pages (collapse leaf-pages-first).
//
// Reports the ids whose payloads just became unreachable so the caller
// can free them.
world.collect(lowWatermark, minAge, outFreedIds);
```

Three properties make this safe and cheap:

- **Nothing in use can ever be collected.** A page any view's cut needs is touched every frame, so it can never age toward the tail. GC can only take pages *no* view has walked for `minAge`+ frames — the thrash case (collapse something a view still wants, re-expand next frame) is structurally impossible, not just unlikely.
- **Staleness is monotone down the page tree.** A child page is only reachable *through* its parent page, so touching a child touches every ancestor on the way down — a page is always at least as fresh as its attached children. The LRU tail is therefore naturally deepest-first, and the leaf-pages-first collapse order falls out with no tree analysis; the `attachedChildPages == 0` check is one counter read, belt and suspenders.
- **No scans.** Touch is O(1), and `collect` does work proportional to what it examines at the tail — never a sweep over all pages. This keeps the sublinear budget intact.

Recency alone is the base **priority**; if profiling demands better, rank the tail *region* by (staleness, page bytes, screen error at last touch) — the candidate set is small by construction, so smarter ranking stays cheap.

The same clock can drive **payload eviction inside attached pages** (nodes the cut has receded above — resident but unreached): pages near the LRU tail can be swept against their per-node scratch stamps, O(page size) each and only for candidates. Whether that's worth the complexity over collapsing at page granularity only is a profiling question (§11).

`collect` obeys the same rule as every other mutation: between frames, never mid-traversal.

---

## 5. Top level — instances, motion, multi-root

```cpp
// Dynamic BVH over root instances. Small: R = number of roots, not nodes.
// This is where everything mutable-and-moving lives, so the pages don't have to.
struct TopLevel
{
    // per instance: worldFromLocal transform, inflated world bbox,
    //               root PageRef, max geometric error of the whole tree
    InstanceId add   (PageRef rootPage, Transform t);   // O(log R)
    void       move  (InstanceId, Transform t);         // O(log R) refit
    void       remove(InstanceId);                      // O(log R)

    // Tri-state frustum walk + contribution cull: O(log R + hits).
    // Fully-outside BVH subtrees are rejected; fully-inside ones stop testing
    // planes. Each surviving instance is returned with the set of planes still
    // UNDECIDED at its bounds, so the per-page walk (§8) starts pre-narrowed —
    // an instance deep inside the frustum enters with an empty mask and never
    // tests a plane again.
    // An instance whose *entire tree* projects below minPix pixels is dropped
    // outright — draws nothing. Sound by definition: omitting a subpixel
    // object leaves a subpixel hole. minPix = 0 disables.
    void query(frustum, errorScaleK, minPix, out visibleInstances /*+ masks*/);
};
```

Standard dynamic-BVH machinery applies: inflated bounds so jitter doesn't dirty anything, incremental refit on move, periodic partial rebuilds to keep quality under churn. None of it is novel; it is the one genuinely new *component* this design adds over a single static tree. Two layout requirements so it doesn't become the slow part: flat arrays with indices — no per-node allocations, no pointer chasing — and the same wide-node (BVH4/BVH8) child layout as the pages (§2), so its query tests W children per SIMD issue too.

What the top level buys:

- **Multi-root is the native shape.** A world is a set of instances — one planet, or a million shallow props. The old "sentinel virtual parent of the root" generalizes into this structure.
- **Rigid motion is O(log R) regardless of tree size.** Moving a whole instance is one transform write plus a top-level refit. This is the *preferred* granularity for motion (§6).
- **Contribution culling** answers "a whole forest on the horizon": the per-subtree max error lets one test skip thousands of instances that would each have drawn one subpixel node. If distant clusters should still be *seen* rather than skipped, that's authoring aggregation — an imposter tree whose leaves are expansion points into the individual trees, which the paging mechanism already supports with zero new machinery. Top-level BVH nodes themselves can never join the cut: they have no ids and nothing to draw.

Culling transforms the frustum into each instance's local space once, then tests local-space page bboxes — no per-node transform work.

---

## 6. Moving nodes inside a tree

Supported, with eyes open. Two different things are affected:

**Bounds** — solvable, sublinear. Bounds are the only mutable data in a page: `bbox[]` plus the wide-block lanes that mirror it. Moves queue as flat `{handle, box}` records and the refit runs lazily, at the next `selectCut` (or an explicit `flushBounds()`) — never per move. The flush applies submissions in order with a bottom-up grow-only walk along `parent[]` that stops at the first ancestor already containing the change. That early-out is also the dedup: a thousand movers under one building grow the shared ancestors once and merely re-check them afterwards, and a node moved many times between cuts rewrites the same hot cache lines and stops at its parent (explicit dedup structures — per-node stamps, per-page dirty chains, a transient hash set — were all measured slower than the walks they skip; see ARCHITECTURE.md experiment K). Each refit step writes two places: the node's `bbox[i]` and its lane in the parent's wide block (found by scanning the parent's ≤W lanes for `child == i` — a handful of loads). If a page's top-level bounds grow past its sentinel, propagate to the owning expansion point in the parent page, and ultimately to the instance's top-level entry. Cost: O(dirty-subtree closure depth), amortized by inflated bounds — a node only dirties its parent when it escapes its slack.

Invariant (C) is thereby relaxed from *tight* to **conservative**: ancestor boxes may be loose, never too small. Grow immediately, shrink lazily (periodic refit). Loose boxes cost culling efficiency, never correctness.

**Submission cost.** `setNodeBounds(NodeHandle, …)` is a bounds-check and a queue push — ~16 ns per move, no lookup of any kind. Handles for persistent movers are composed once at attach time (`nodeAt(pageHandle, authoredIndex)`) and cached forever: a handle whose page has since detached (streaming or GC) fails the generation check at flush time and is silently skipped, so callers never track page lifetime — they re-compose handles only after a re-attach.

**Proxies** — not solvable by any refit, and the design says so plainly. A parent's merged representation bakes its children's poses; a child that moves makes every ancestor proxy *visually* stale even though culling stays sound. There is no sublinear fix because it's a content problem, not a data-structure problem. The practical stance: motion at **instance granularity is fully supported**; motion *inside* a tree keeps the structure correct but ancestor proxies won't reflect it. Heavily dynamic objects belong in their own small instances at the top level, not baked deep into a big static tree.

---

## 7. Per-view scratch — no O(N) anything

```cpp
// One instance per *view* — main camera, each shadow cascade, probe. They see
// different cuts over the same forest. Never share one between views.
//
// Storage is per page, allocated lazily the first time a view touches the
// page, freed when the page detaches. Memory is O(materialized nodes) —
// fine; it is per-frame TIME that must be sublinear.
//
// EPOCH STAMPS instead of clears: every entry records the frame it was
// written. A stale stamp reads as "unvisited" — so there is no assign(N, 0)
// anywhere, ever. Nodes the pruned walk stops visiting simply expire.
struct ViewScratch
{
    uint32_t frame;                    // bumped once per selectCut

    // per page, parallel to its nodes.
    // Scattered by the PARENT's wide child-test, W lanes at a time (§8);
    // a node whose `live` stamp is stale was culled — never visited:
    std::vector<uint32_t> live;        // == frame: accepted by frustum
    std::vector<float>    err;         // projected screen error
    std::vector<uint8_t>  planes;      // frustum planes still UNDECIDED here
                                       //   (1 bit per plane); 0 == an ancestor
                                       //   was fully inside — no test below

    // Written at the node's own visit:
    std::vector<uint8_t>  alive;       // this frame: actual refine chain
                                       //   intact from instance root to here
    std::vector<uint32_t> seen;        // frame of last visit — validates sticky
    std::vector<uint8_t>  sticky;      // last visit's IDEAL refine decision —
                                       //   the hysteresis state
};
```

Two deliberate choices:

- **Hysteresis lives on the ideal decision, not the actual one.** Sticking `wants && can` would conflate two unrelated flappings: a node oscillating across the error threshold (what hysteresis is for) and residency arriving/leaving (which shouldn't reset the sticky state). We persist `wants`.
- **Staleness defaults to coarse.** `sticky` is honored only if written last frame; anything older reads as 0 — "start coarse and refine in", the safe direction. A cut receding from a region costs nothing to forget.

---

## 8. Cut selection — one pruned pass

The old two-pass structure (mark everything, then scan everything) had three linear costs: descending below the cut, a full pass-2 sweep, and per-frame clears. All three are gone. The walk **descends only while the node wants to refine** (the ideal-cut decision); the moment a node declines — in the *ideal* sense — its entire subtree is skipped like a frustum-culled one. Since the ideal cut is always at-or-below the actual cut, one walk emits both cuts and all requests inline. There is no second pass to run and nothing below the ideal cut is ever touched.

```cpp
void selectCut(view, threshold, hysteresis, minPix, scratch,
               outCut, outIdealCut, outRequests)
{
    scratch.frame++;
    k = errorScale(view.camera, threshold);   // camera-dependent scalar, once

    topLevel.query(view.frustum, k, minPix, instances);   // O(log R + hits)

    for (inst in instances)
        worklist.push(inst.rootPage, aliveAtEntry: 1,
                      planesAtEntry: inst.undecidedPlanes);  // from the TLAS walk

    while ((page, aliveAtEntry, planesAtEntry) = worklist.pop())
    {
        S = scratch.forPage(page);            // lazily allocated; LRU touch (§4.1)
        S.alive[0] = aliveAtEntry;

        // The sentinel's wide block stamps the page's roots: frustum,
        // distance and error for all of them, one SIMD issue per W.
        wideTest(page, node: 0, planesAtEntry, S);

        for (i = 1; i < page.nodeCount(); )
        {
            // Culled nodes were simply never stamped by their parent's wide
            // test. One 4-byte load decides the skip; (B) makes it one add.
            if (S.live[i] != scratch.frame)
                { i += page.subtreeSize[i]; continue; }

            err = S.err[i];                   // parent already computed it

            // Hysteresis: last VISIT's ideal decision if fresh, else coarse.
            sticky = (S.seen[i] == scratch.frame - 1) ? S.sticky[i] : 0;
            bar    = sticky ? threshold * (1 - hysteresis)
                            : threshold * (1 + hysteresis);

            hasKids = childCountOf(page.meta[i]) > 0 || isExpansion(page.meta[i]);
            wants   = hasKids && err > bar;

            // Parent was visited this frame (that's the only way to get here),
            // so this read needs no guard.
            alive = S.alive[page.parent[i]];

            S.seen[i]   = scratch.frame;
            S.sticky[i] = wants;

            if (!wants)                        // both cuts end here — including
            {                                  // a collapsed node whose proxy
                                               // suffices: DIRECT, no expansion
                if (alive) outCut.push(page.userId[i], err);
                outIdealCut.push(page.userId[i], err, DIRECT);
                i += page.subtreeSize[i];      // nothing below can matter
                continue;
            }

            if (!materialized(page, i))        // collapsed AND too coarse:
            {                                  // children unknown, stand in for
                                               // them; the tag is the request
                if (alive) outCut.push(page.userId[i], err);
                outIdealCut.push(page.userId[i], err, NEEDS_EXPANSION);
                i += page.subtreeSize[i];
                continue;
            }

            ready = canRefine(page, residency, i);       // O(1), §4
            if (!ready && alive)               // actual cut stops; ideal goes on
            {
                outCut.push(page.userId[i], err);
                outRequests.push(loadRequest(childrenOf(page, i)), priority: err);
            }

            S.alive[i] = alive && ready;       // chain survives only through
                                               // fully-ready refinements

            if (isExpansion(page.meta[i]))     // cross the page boundary; the
                worklist.push(page.expansion[i], S.alive[i], S.planes[i]);
            else
                wideTest(page, node: i, S.planes[i], S);  // stamp my children
            ++i;                               // ideal descent continues
        }
    }
}

// One SIMD issue per W children: tri-state frustum against the still-active
// planes, distance, and screen error — lanes are children. Survivors are
// scattered into scratch; they are exactly the nodes the forward loop is
// about to visit, so nothing is computed twice and nothing culled is touched.
void wideTest(page, n, mask, S)
{
    for (blk in wideBlocksOf(page, n))        // ceil(childCount / W) blocks
    {
        vis     = frustumTriState(blk, mask); // per lane: OUTSIDE, or the
                                              //   narrowed plane mask
                                              //   (early accept clears bits)
        laneErr = blk.error * k / distanceToBox(view, blk);  // all W at once

        for (lane in survivors(vis))          // bit-scan over the lane mask
        {
            c = blk.child[lane];
            S.live[c]   = scratch.frame;
            S.err[c]    = laneErr[lane];
            S.planes[c] = vis.newMask[lane];
        }
    }
}
```

**Wide child tests (the BVH4/BVH8 pattern).** The geometry math — plane tests, distance, error projection — is the ALU bulk of the walk, and it is embarrassingly parallel *across siblings*. So it runs at the parent, not at the node: one SIMD issue covers W children from the packed block, and each visited node finds its own results already in scratch — its visit reads three 4-byte tree words (`parent`, `subtreeSize`, `meta`) plus scratch and does only scalar decision logic. The scatter of survivors is cheap because survivors are exactly the nodes the loop visits next. The price is duplicated bounds (parent's lanes + the cold `bbox[]`) and the lane-patching rule in §6; the payoff is that per-node geometry cost drops by roughly the SIMD width, and the fat 24-byte-per-node `bbox` stream leaves the hot loop entirely.

**Tri-state culling** is folded into the same wide test, per lane: fully outside drops the lane (the child is never stamped, and the forward loop skips its whole subtree on one stale-stamp check), *fully inside* clears the remaining planes from that child's mask — no descendant of it pays for a frustum test again — and partial visibility narrows the mask to the undecided planes. The mask rides down through scratch, across page boundaries via the worklist, and enters each instance pre-narrowed by the top-level walk, which runs the same tri-state logic over the BVH. The common cases collapse to almost nothing: a camera inside a city tests a handful of planes near the frustum boundary and none in the interior. None of this changes the output-size bound — every node down to the ideal cut still needs its error decision — it shrinks the per-node constant.

Why this is correct with no explicit cut predicate: a node is only *reached* because every ancestor ideally refined, so "parent refined into me" is implicit in control flow. The actual cut is emitted exactly where the `alive` chain dies — either the node itself declines (`!wants`) or readiness fails below an alive ancestor. A parent and child are never both in `outCut` because `alive` is cleared the moment a node is emitted. Leaves need no special case: `hasKids` is false, so they never want, and always draw when reached. It still leans on (D) — if error could grow downward, a grandchild could re-want under an already-emitted ancestor and corrupt the *ideal* cut's antichain property.

Cost per view: `O(log R + visible instances + ideal-cut region)`. The term that used to be O(N) — everything in-frustum below the cut — is gone, because descent stops at the ideal cut, not at the frustum.

---

## 9. Builder

```cpp
// Authoring-time, offline. Insertion order arbitrary, nothing perf-sensitive.
// Builds ONE page; the content pipeline decides where to split the full tree
// into pages and wires expansion points between them.
class HLodBuilder
{
public:
    NodeId createRoot (uint64_t userId, float error, AABB bbox = {});
    NodeId createNode (NodeId parent, uint64_t userId, float error, AABB bbox = {});
    void   markExpansion(NodeId node, PageRef childPage);  // node must stay leaf

    Page build();   // builder is consumed
};
```

`build()` is emission plus three linear sweeps — linear in *page* size, which is bounded and offline, so it doesn't fight the sublinear budget. All three exploit the same trick: once nodes are in preorder, `parent[i] < i`, so bottom-up work is a reverse loop and top-down work is a forward one.

```cpp
Page HLodBuilder::build()
{
    // ---- Sentinel at index 0: stand-in for this page's owner ----
    emit(bbox: infinite, error: FLT_MAX, parent: 0, subtreeSize: 1,
         childCount: <number of page roots>);
    // attachPage() overwrites the sentinel's error with the real owner's (§3).

    // ---- Pass A: preorder DFS emission — establishes (A) and (B) ----
    // Push children reversed so they pop in declaration order. Sibling order
    // is free — Morton-sort here for spatial locality in the walk.
    ...emit each node: userId, bbox, error, remapped parent,
       meta (childCount | EXPANSION), expansion side-table entry...
    assert(emitted == total);           // else: nodes unreachable from root

    // ---- Pass B: bottom-up fold — subtree sizes and bounds in one reverse sweep
    // Unioning children into the parent ESTABLISHES (C); an author-supplied
    // bbox is treated as a lower bound.
    for (i = total - 1; i >= 1; --i)
    {
        subtreeSize[parent[i]] += subtreeSize[i];
        bbox[parent[i]].expand(bbox[i]);
    }

    // ---- Pass C: enforce monotone error (D), forward sweep ----
    // Assert in debug so bad LOD generation is reported at its source; clamp
    // always so imported assets still work. attachPage() runs the same clamp
    // across page boundaries.
    for (i = 1; i < total; ++i)
    {
        assert(geometricError[i] <= geometricError[parent[i]]);
        geometricError[i] = min(geometricError[i], geometricError[parent[i]]);
    }

    // ---- Pass D: emit wide child blocks ----
    // After B (bounds final) and C (errors final): for every interior node,
    // gather its children — first child at i+1, each next sibling one
    // subtreeSize hop away — into ceil(childCount / W) SoA-transposed blocks
    // (bounds, error, local index per lane; unused lanes get empty boxes).
    // Store the block offset in meta[i].
    for (each interior node i)
        meta[i] |= emitWideBlocks(childrenOf(i)) << OFFSET_SHIFT;

    // ---- Pass E: verify the contract ----
    for (i = 1; i < total; ++i)
    {
        assert(parent[i] < i);                              // (A)
        assert(i + subtreeSize[i] <= total);                // (B)
        assert(childCount(i) == 0 || parent[i + 1] == i);   // (B) first child adjacent
        assert(bbox[parent[i]].contains(bbox[i]));          // (C)
        assert(childCount(i) == 0 || !isExpansion(i));      // local XOR paged children
        assert(wide lanes mirror bbox[]/geometricError[]);  // hot copies consistent
        assert(meta[i] packing fits);                       // page small enough
        assert(userId[i] is unique within reason);          // ids are the output
    }
}
```

Renderability can no longer be asserted here — the structure doesn't own geometry. It is a documented caller contract for every node, and a hard requirement for expansion points (§3). A grouping node with nothing to draw that wins the cut is a hole; give it a proxy or collapse it in the pipeline.

---

## 10. Invariants

| # | Invariant | Established by | Relied on by |
|---|---|---|---|
| A | `parent[i] < i` (preorder, per page) | build() pass A | forward walk reading parent state, no recursion |
| B | subtree == `[i, i + subtreeSize[i])` | build() pass A + B | the one-add branch skip |
| C | parent bbox contains descendants, **conservatively** | pass B; maintained by lazy refit (§6) | culling correctness under motion |
| D | error non-increasing downward | pass C in-page; attach-time clamp cross-page | both cuts being antichains |
| E | every id renderable; expansion points **must** have proxies | caller contract / pipeline | no holes at the cut boundary |
| F | drawn or refined ⇒ resident and materialized | all-or-nothing refinement + pinned roots | streaming without holes or overdraw |
| G | traversal descends only into attached pages | expansion-point test in the walk | topology streaming soundness |
| H | per-view cost `O(log R + visible roots + ideal-cut region)` | pruned descent + epoch stamps + top-level BVH | the whole point |

A–D are static per page and asserted in `build()` / `attachPage()`. E is a contract. F and G are dynamic, maintained by the streamer under the evict-between-frames rule. Break any of A–G and the failure is a rendering artifact — holes, double-draw, popping — not a crash; which is why they're asserted rather than discovered in a playtest.

### Complexity budget

| Operation | Cost |
|---|---|
| `selectCut` (per view) | O(log R + visible roots + ideal-cut region) |
| `markResident` / `markNonResident` | O(1) |
| `attachPage` / `detachPage` | O(page size), streaming-paced |
| LRU touch (per page, per frame) | O(1), first view only |
| `collect` | O(tail candidates examined + pages detached) |
| add / move / remove instance | O(log R) |
| move a node inside a tree | O(depth) amortized, lazy (applied at the next cut) |

No operation is O(total nodes). Per-view memory is O(materialized nodes), reclaimed as pages detach.

---

## 11. Left open

- **Page size.** Bigger amortizes boundary crossings, smaller gives finer memory control. Start in the hundreds-to-low-thousands range; tune on real content.
- **SIMD width W.** A page-format parameter (4 vs 8). Authoring fanout near W wastes no lanes; whether to constrain the build to that, and whether one shipped format or per-platform builds, is a pipeline decision.
- **Top-level BVH specifics.** Refit policy, rebuild cadence, quality under heavy streaming churn. Well-trodden territory (physics broadphases, RT TLAS) but real code with real tuning.
- **GC tuning.** The mechanism is fixed (§4.1: LRU + watermarks + dwell); undecided are the numbers — watermark levels, `minAge`, whether the tail region needs size/error-aware ranking, and whether node-level payload sweeps inside cold pages pay for themselves over page-granularity collapse alone.
- **Request prioritization.** Screen error as priority may need a distance/velocity term to avoid thrash during fast camera motion; the ideal cut gives the streamer the full target set to plan against. Wants profiling.
- **Hysteresis tuning.** The `(1 ± h)` split is the simplest form. If popping persists at branch boundaries, next step is a minimum dwell time per node, not a wider band.
- **Scratch storage.** Dense per-page arrays are described here; if views typically touch a small fraction of materialized pages, a hashed sparse variant may win. Measure first.
