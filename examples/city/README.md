# Frontier dynamic city sample

This sample renders a procedural city through bgfx's portable example entry
layer and debug-draw renderer. The world is a 3-by-3 arrangement of the
original district: 24 by 24 blocks covering nine times the area. Frontier owns
the visibility and LOD decisions for 2,088 houses, 54 skyscrapers, 1,152 trees,
432 moving cars, and 864 moving pedestrians. Cars follow rounded rectangular
roads with tangent-aligned yaw,
so their orientation changes smoothly through corners. Cars and pedestrians
are updated through `RigidMotionGroup`, while `SpatialQuery` follows an
automatic or free-flying camera and produces the render cut each frame.

Every block has a raised sidewalk ring and an explicit building setback.
Pedestrians follow the rounded sidewalk centerline instead of cutting through
building footprints, orient from the path tangent in both travel directions,
and include a small forward-facing mesh marker so their heading remains visible
at every LOD. Street trees sit at alternating rounded-corner centers, with the
pedestrian path curving around their trunks.

The scene deliberately uses no external meshes or textures. Each reusable
Frontier subtree has reusable LOD cuts; skyscrapers use a deliberately deeper
five-level hierarchy. Selected payloads dispatch simple bgfx debug-draw
geometry. Every representation has application-side virtual size metadata: a
skyscraper fallback costs 0.5 MiB, while its three max-detail leaf resources
total 10 MiB. Sizes apply to reusable representation resources, so every
placement of a registered definition shares the same residency. Only the five
coarsest fallback resources start resident.
`computeFrontierRefinement()` analyzes the complete ideal cut each frame, while
the streaming simulator requests only immediate complete-child groups. Groups
become ready atomically after the configured latency, so Frontier advances the
current renderable cut toward the ideal without ever exposing a partial sibling
transition.

The UI is split into independent, movable ImGui windows so diagnostics do not
cover one another. The global **Debug windows** menu in the top bar toggles
each widget independently and provides **Show all** / **Hide all** actions;
each window can also be closed with its title-bar button. Only
**Frontier debug** is open by default. **Frontier debug**
controls simulation freeze,
hierarchy-level tinting (green top nodes, yellow intermediate nodes, red
leaves), optional scene-wide wireframe rendering, LOD and contribution
thresholds, camera modes, and workload generators. **Replace all with House
A/B** removes all 2,088 current house instances and creates a newly randomized
generation of the selected architectural style. The operation is deferred into
the measured motion and `applyUpdates` stages so its structural-update spike is
visible in the performance charts. **Start stress test** moves every
Frontier instance independently up and down with a spatially phase-shifted
cosine wave every simulation frame through one `RigidMotionGroup`. This avoids
coherent rigid motion and deliberately forces the more expensive all-object
motion case; stopping it restores the authored city layout. Simulation freeze
also pauses this stress animation. The separate **TLAS maintenance** window
configures a finite or unlimited node-repair budget for each `applyUpdates`
call and three explicit topology-rebuild strategies: **Manual only**,
unconditional **Periodic**, and **When recommended**. The two scheduled
strategies use a configurable 0.25-to-60-second interval and can call either
`refreshTlas()` (SpatialBins topology, dense layout preserved) or `optimize()`
(configured quality, compaction, and spatial reordering). Recommendation-gated
mode checks the latest `UpdateReport` at that cadence. **Refresh TLAS now** and
**Optimize now** remain available under every strategy. The UI tracks timing
and call counts for both methods so they can be compared. The sample defaults
to **When recommended**, `refreshTlas()`, and a two-second check interval.
The separate **Virtual streaming** window controls the virtual memory budget,
load latency, unload delay, maximum concurrent group loads, and the residency
cut strategy. **Turn off virtual streaming** cancels simulated I/O and makes
every representation resource used by the active scene immediately resident;
the budget, latency, and eviction policy are ignored while it is off, giving a
fully resident ideal-quality baseline. Refinement planning, scoring, admission,
and eviction work are skipped in this mode, apart from lightweight current-cut
UI accounting. Turning streaming on again resets
residency to the pinned coarsest resources and resumes normal convergence.
House A/B replacement also makes the new active style fully resident while
streaming remains off. The simulator defaults to an 85 MiB budget and
**Quality per byte**, which
selects with
`PreferReadyAncestors`: a resident descendant cannot override the camera's
coarser threshold target, so over-detailed resources leave the current cut and
become eviction candidates. **Retain ready detail** switches to
`PreferReadyDescendants` to demonstrate the intentionally sticky alternative
and why it can starve a more valuable load under pressure. The window reports
resident, loading, current-cut, and ideal-cut memory; current-to-ideal frontier
convergence with a rolling history and elapsed completion time; memory outside
the active quality target; budget or transition stalls; and a detailed
representation-residency decision table.
That table reports current-node/current-benefit/immediate-next/predicted-camera/
ideal instance counts, min/average/max projected screen error, exact virtual bytes,
visual-importance score, score-per-MiB, current policy decision, and the last
load or eviction reason. A resident representation retains the parent-error
benefit measured when its load was admitted. Valuing it later by only its own
smaller post-refinement error would make its eviction priority collapse merely
because it loaded, producing a coarse-to-fine streaming feedback loop.
Refinement candidates use the parent error that loading the child group would
eliminate, rather than a terminal child's usually-zero geometric error.
The documented sample policy weights current-cut, immediate-next, six-second
predicted-camera, and ideal demand at 4x, 3x, 1.5x, and 1x, then loads complete
refinement groups by
score-per-MiB and makes pressure eviction use those same complete sibling
groups. This prevents the simulator from retaining an unusable partial
refinement (for example a skyscraper base and crown without its shaft) and
repeatedly reloading its missing member. Replacement is a two-phase transaction:
the simulator plans the complete victim set without changing residency, then
commits only if the request fits and its total visual value exceeds unused
cache victims by at least 5%, or current-cut victims by at least 30%.
Score-per-MiB orders candidates and victims; the total-value test handles
indivisible-resource granularity. A request that cannot complete leaves every
prospective victim untouched. Current-cut replacements also receive fifteen
seconds of minimum residency to reject role-change feedback.
Projected errors are keyed by both logical node and placement instance; this is
essential for hero assets whose shared representation can be tiny in one
placement and fill the screen in another. After resources outside the active
target are exhausted, quality-per-byte mode may replace a lower-score current
group with a higher-score requested group. Ordinary score-based replacements
keep every selected representation's complete ready ancestor chain resident
and charged to the same budget, so pressure coarsens it one available level at
a time instead of jumping directly to the pinned fallback. A second Frontier
query follows the known camera path
six seconds ahead with a lower score weight, providing enough time for several
sequential refinement loads before a skyscraper enters the close view.
Before the scalar score is applied, the sample enforces a minimum-quality
constraint for hero assets: if a skyscraper's pinned fallback would be at least
three pixels of projected error (roughly a 150-pixel-tall facade), its 0.75 MiB
district representation is a quality-floor request. These requests are ordered
ahead of ordinary refinements, may displace lower-priority cached, transition,
or fallback-chain data, and cannot themselves be selected as victims while the
hero remains screen-dominant. The rest of the budget is still optimized by
visual score per MiB. This lexicographic policy prevents a large visible tower
from being sacrificed for fine detail elsewhere just because an indivisible
10 MiB resource group has greater total score.
The UI counts these quality demotions explicitly. A rolling virtual load/unload
log includes the same score context. **Reset to coarsest residency**
makes all non-fallback representations unavailable again. Replacing the house
generation also unloads the previous style's virtual resources before the new
style is streamed. The 54 skyscraper placements are divided among 18
independent hero asset definitions, with exactly three instances per asset.
Every hero has its own district-through-max-detail virtual resources and
therefore competes independently for memory. **Set one-hero transition budget**
selects the transient-safe budget for pinned fallbacks, one 5 MiB fine parent,
and its complete 10 MiB detailed child group; other high-scoring scene
resources still compete inside that same budget.
The **Start close skyscraper orbit test** regression scenario uses the captured
close view, then slowly orbits the central hero skyscraper for 180 simulated
seconds while the normal simulation continues. It resets to coarsest residency,
applies an 85 MiB budget, and records every virtual-resource state transition.
After a 20-second warm-up it fails if the focal tower, or any other tower whose
fallback error indicates an approximately 150-pixel-or-taller facade, reaches
the pinned top representation. It separately detects a true feedback loop by
counting unload-to-reload cycles that recur for the same resource within five
seconds; ordinary load lifecycles across the full orbit are reported but do not
fail the test. The same deterministic check can run without interactive input:

```sh
build-city/examples/city/frontier_city --streaming-orbit-self-test
```

`--streaming-test-camera-time=<seconds>` selects a starting phase on the orbit,
`--streaming-test-budget=<MiB>` overrides the budget, and
`--streaming-test-viewport-height=<pixels>` runs screen-error selection at a
chosen pixel density without allocating a correspondingly large framebuffer.
The older
`--streaming-self-test` and `--streaming-dynamic-self-test` spellings remain
aliases for this regression.

The process exits nonzero on a dominant fallback, excess churn, or budget
violation and prints focal LOD changes, dominant-fallback events, replacement
transactions, and per-resource churn lines. Successful runs also print the
highest-activity resources from the full orbit.
Wireframe can also be toggled directly from the top-bar
**Rendering** menu and composes with hierarchy tinting. **Scene stats** contains
entity, cut, streaming, cache, simulation, and camera status.
**Performance** reports timings in microseconds and puts Frontier selection,
motion/database work, and virtual streaming first. bgfx timing and backend
counters follow, with UI, camera, and diagnostic overhead last. Every timer has
its own rolling raw-sample chart covering roughly 5-10 seconds, including
Frontier selection, motion submission, `applyUpdates`, TLAS rebuild, resource
publication, bgfx submit/render/GPU/wait, UI, camera, accounting, unaccounted,
and total-frame time. Each timer also reports the minimum, maximum, and average
over its visible rolling window. Draw, primitive, and transient-buffer counters
remain alongside the timing charts. **Scene hierarchy** is a live
ImGui tree of each reusable Frontier topology with current selected-entry
counts. **TLAS health** reports topology occupancy, depth, motion-area growth,
the incremental repair queue, active/configured build quality, rebuild policy,
topology-rebuild advice, and storage. It controls complete depth-cut TLAS AABB
rendering and loose-motion envelope comparison. **Query cache** reports reuse
rate, record/slab storage, garbage, cache state, travel, and hit-rate history. The TLAS and
loose-bound visualizations are also independently available from the
**Rendering** menu. All windows except **Frontier debug** are closed by
default; rendering overlays are disabled.

Free camera uses **WASD** to move, **Q/E** to descend/ascend, and right-mouse
drag to look. **Freeze camera / cull state** captures the active culling camera,
switches to the free debug camera, and renders the captured frustum as
translucent magenta planes.

From the repository root:

```sh
cmake -S . -B build-city \
  -DFRONTIER_BUILD_CITY_SAMPLE=ON \
  -DFRONTIER_DEBUG_TOOLS=ON \
  -DFRONTIER_BUILD_TESTS=OFF
cmake --build build-city --config Release --target frontier_city
```

The repository-root launchers perform all three steps in one command:

```sh
bash ./run_city_sample.sh # macOS/Linux
run_city_sample.bat      # Windows
```

Set `FRONTIER_CITY_BUILD_DIR` to use a different build directory. Arguments
after the script name are forwarded to the bgfx application.

Run `build-city/examples/city/frontier_city` on single-config generators. With
Visual Studio, run `build-city/examples/city/Release/frontier_city.exe`.

The first configure downloads the bgfx CMake distribution at the commit pinned
in `CMakeLists.txt`; that distribution brings its matching bgfx, bx, and bimg
submodules. Normal Frontier builds do not download or compile those dependencies.
`FRONTIER_DEBUG_TOOLS` is off by default in normal builds. The launch scripts
enable it for this sample so the read-only TLAS/cache inspection API is present;
no debug scan or bounds enumeration runs while its windows and rendering modes
remain disabled.
