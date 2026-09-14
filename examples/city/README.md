# Frontier dynamic city sample

This sample renders a procedural city through bgfx's portable example entry
layer and debug-draw renderer. The world contains 24 by 24 blocks arranged as
a 3-by-3 grid of districts. Frontier owns
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
Frontier subtree has reusable LOD cuts. A skyscraper's District, Coarse,
Medium, and Fine facade representations share one conservative bound and are
authored as four payload slots on one node; Base, Shaft, and Crown are its
three structural children. Cuts therefore pass through the facade node before
expanding to detailed geometry, without a redundant same-bound node chain.
Selected payloads dispatch simple bgfx debug-draw geometry. Every
representation has application-side virtual size metadata: a
skyscraper fallback costs 0.5 MiB, while its three max-detail leaf resources
total 10 MiB. Sizes apply to reusable representation resources, so every
placement of a registered definition shares the same residency. Only the five
coarsest fallback resources start resident.
The sample calls `computeFrontierRefinement()` with
`SpatialQuery::UnlimitedDepth` and derives
an application-defined exhaustive quality endpoint from the returned group
forest. The UI labels that endpoint **ideal**; it is sample terminology, not a
second cut returned by Frontier. The streaming simulator requests only
immediate complete-child groups. Groups become ready atomically after the
configured latency, so Frontier advances the current renderable cut toward the
quality endpoint without exposing a partial sibling transition.

The UI is split into independent, movable ImGui windows so diagnostics do not
cover one another. The global **Debug windows** menu in the top bar toggles
each widget independently and provides **Show all** / **Hide all** actions;
each window can also be closed with its title-bar button. Only
**Frontier debug** is open by default. **Frontier debug** shows the active
graphics backend (for example OpenGL or Vulkan), CPU model, and selected GPU
model. On Linux ARM systems, the CPU label identifies the core models and their
counts. Hardware names are read once at startup; unavailable information is
labelled explicitly. Hover over the GPU name for the OpenGL driver/version
string, which is also printed in the startup log. This window controls simulation freeze,
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
strategies use a configurable 0.25-to-60-second interval and can call
`optimize(mode)` in either **Topology only** mode (SpatialBins topology, dense
layout preserved) or **Topology + layout** mode (configured quality,
compaction, and spatial reordering). Recommendation-gated mode checks the
latest `UpdateReport` at that cadence. Both modes remain manually available
under every strategy, and the UI tracks their timing and call counts. The
sample defaults to **When recommended**, **Topology only**, and a two-second
check interval.
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
refinement groups by score-per-MiB. Successive payload slots on one node are
singleton groups, while structural refinement and pressure eviction use the
same complete sibling groups. This prevents the simulator from retaining an
unusable partial refinement (for example a skyscraper base and crown without
its shaft) and repeatedly reloading its missing member. Replacement is a
two-phase transaction:
the simulator plans the complete victim set without changing residency, then
commits only if the request fits and its total visual value exceeds unused
cache victims by at least 5%.
Score-per-MiB orders candidates and victims; the total-value test handles
indivisible-resource granularity. A request that cannot complete leaves every
prospective victim untouched. Current-cut resources are never load victims:
the current representation and its loading replacement are both charged to
the hard budget. If that overlap cannot fit after evicting non-current
resources, refinement remains blocked instead of temporarily degrading.
Projected errors are keyed by payload slot and placement instance; this is
essential for hero assets whose shared representation can be tiny in one
placement and fill the screen in another. After resources outside the active
target are exhausted, quality-per-byte mode leaves upgrades queued until safe
capacity exists. Complete groups use make-before-break publication: all
members remain unavailable during loading and become ready atomically, after
which the next Frontier query may select the finer cut. Fallback residency is
rolling rather than cumulative: each selected representation pins only its
immediate ready predecessor. The selected current cut is protected separately
while its replacement loads. Once the finer representation becomes current,
the fallback pin advances to the just-replaced representation and older LODs
become reclaimable after the unload delay. Thus a detailed skyscraper does not
permanently retain its entire District-to-Fine ladder. A second Frontier query
follows the known camera path
six seconds ahead with a lower score weight, providing enough time for several
sequential refinement loads before a skyscraper enters the close view.
Each successful atomic publication also runs a diagnostic same-camera
counterfactual selection immediately before and after the readiness change.
For every visible instance, the sample takes the worst projected geometric
error among that instance's selected entries, then reports both affected-object
and whole-scene RMS error, plus scene-worst error. This keeps a structural
parent-to-many-children transition comparable and distinguishes a meaningful
local improvement from a numerically tiny whole-scene gain. If the new
resource does not enter the current cut, the log identifies it as prefetch or
future demand instead of claiming a visible improvement. These extra queries
run only when a load publishes and appear separately as **Error measurement**
in the performance window.
Before the scalar score is applied, the sample enforces a minimum-quality
constraint for hero assets: if a skyscraper's pinned fallback would be at least
three pixels of projected error (roughly a 150-pixel-tall facade), its 0.75 MiB
district representation is a quality-floor request. These requests are ordered
ahead of ordinary refinements, may displace lower-priority cached, transition,
or fallback-pinned data, and cannot themselves be selected as victims while the
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
selects the transient-safe budget for pinned fallbacks, one 5 MiB fine facade
payload, and its complete 10 MiB detailed child group; other high-scoring scene
resources still compete inside that same budget.
The **Start close skyscraper orbit test** regression scenario slowly orbits the
central hero skyscraper for 180 simulated seconds while the normal simulation
continues. Its automated form first flies from a 150 m radius to the 58 m close
view over 30 seconds, then begins the orbit. It resets to coarsest residency,
applies an 85 MiB budget, and records every virtual-resource state transition.
After a 20-second warm-up it fails if the focal tower, or any other tower whose
fallback error indicates an approximately 150-pixel-or-taller facade, reaches
the pinned top representation. It also verifies make-before-break directly:
every current-cut payload remains ready during load admission, focal authored
geometric error never increases during the approach, committed memory never
exceeds the hard budget, insufficient-capacity upgrades are blocked, and at
least one structural child group publishes atomically. It also requires the
detailed-cut path to exercise rolling fallback pins and observes older
District/Coarse/Medium resources becoming reclaimable and unloaded while Fine
remains pinned. It separately detects a true feedback loop by counting
unload-to-reload cycles that recur for the same resource within five seconds;
ordinary load lifecycles across the full orbit are reported but do not fail the
test. The same deterministic check can run
without interactive input:

```sh
build-city/examples/city/frontier_city --streaming-orbit-self-test
```

`--streaming-test-camera-time=<seconds>` selects a starting time on the
approach/orbit path,
`--streaming-test-budget=<MiB>` overrides the budget, and
`--streaming-test-viewport-height=<pixels>` runs screen-error selection at a
chosen pixel density without allocating a correspondingly large framebuffer.

The process exits nonzero on a dominant fallback, current-cut invalidation,
geometric-error regression, partial group publication, stale transitive
fallback pin, measured LOD-error regression, excess churn, missing
rolling-release/error-measurement/pressure coverage, or any transient budget
violation. It prints focal LOD
changes, replacement transactions, handoff counters, and per-resource churn
lines. When the city sample and unit tests are enabled together, CTest
registers this command as `frontier_city_streaming_make_before_break`.
Wireframe can also be toggled directly from the top-bar
**Rendering** menu and composes with hierarchy tinting. **Scene stats** contains
entity, cut, streaming, cache, simulation, and camera status.
Its **Authored HLOD nodes** total counts the full hierarchy for every current
scene instance, including each aggregate parent, regardless of visibility or
residency. Generated spatial nodes and implicit packing roots are excluded;
multiple LOD payloads on one authored node count once. Shared definitions count
once per placement, and replacing houses removes their old counts before adding
the new hierarchy. The default city contains 18,414 authored HLOD nodes: four
per house, car, pedestrian, and tree, and five per tower.
**Updated HLOD nodes/frame** counts authored nodes affected by changed instance
positions or orientations in the last completed frame, regardless of visibility
or residency. Each instance's complete hierarchy counts once even if moved more
than once that frame. **Moved instances/frame** reports the corresponding
instance count. These are logical world-transform changes: rigid motion updates
an instance transform without rewriting its children, and generated spatial
maintenance is reported separately. Normal motion affects 5,184 authored nodes
in 1,296 cars/pedestrians; whole-scene stress can affect all 18,414 nodes in 4,590
instances. Frozen simulation reports zero unless a pending action changes poses
(for example restoring the scene when stopping stress).
**Performance** reports timings in microseconds and puts Frontier selection,
motion/database work, and virtual streaming first. The virtual-streaming total
is decomposed into all `computeFrontierRefinement()` calls, the remaining
application-side streaming planner, and hero-scenario checks; those three
subtimers are diagnostic children and are not counted again in total CPU time.
The planner is further divided into index construction, demand/group
classification, scoring/ranking, residency policy, and residual timing. These
are also diagnostic children and sum to the planner total rather than adding
new frame time. The sample consumes the refinement view's complete parent
entries and direct expansion links, so index construction contains only fixed
resource-state preparation rather than rebuilding and sorting forest maps.
bgfx timing and backend counters follow, with UI, camera, and diagnostic
overhead last. Every timer has
its own rolling raw-sample chart covering roughly 5-10 seconds, including
Frontier selection, motion submission, `applyUpdates`, TLAS rebuild, resource
publication, publication-time error measurement, bgfx submit/render/GPU/wait,
UI, camera, accounting, unaccounted, and total-frame time. Each timer also
reports the minimum, maximum, and average
over its visible rolling window. Draw, primitive, and transient-buffer counters
remain alongside the timing charts. **Scene hierarchy** is a live ImGui tree
of the authored Frontier topology. Every payload row shows its local-space
geometric error, virtual residency state, planner pinning, and current
selected-entry count. `pinned-permanent` identifies coarsest resources that
can never be evicted; `pinned-fallback` identifies an immediate ready
predecessor temporarily protected by the rolling streaming plan. Each
skyscraper asset row also counts its fallback-pinned resources.
Skyscrapers expand into their 18 independently streamed hero assets: each
asset shows three instance-root Top fallbacks, one shared four-payload facade
node, and its Base/Shaft/Crown structural children. Clicking any hierarchy row
applies a bright cyan tint to matching current-cut geometry. Category and
topology-node rows select their visible descendants, hero rows affect only
that asset's three placements, exact placement rows affect one skyscraper,
and payload rows affect only that payload. **Ctrl+left-click** a skyscraper in
the viewport to select that exact placement, open and focus the hierarchy,
and scroll its row into view. The nearest intersected skyscraper wins.
Click the selected row again, use **Clear selection**, or close the hierarchy
window to remove the tint. **TLAS health** reports
topology occupancy, depth, motion-area growth,
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

On Armbian and other Debian/Ubuntu Linux systems, install the build tools and
graphics development packages once before configuring:

```sh
sudo apt-get update
sudo apt-get install build-essential cmake ninja-build git \
  libx11-dev libgl1-mesa-dev libwayland-dev
```

Use CMake 3.24 or newer and a C++20 compiler. A working desktop alone does not
provide the development headers and linker libraries. In particular,
`Could NOT find X11 (missing: X11_X11_INCLUDE_PATH X11_X11_LIB)` means
`libx11-dev` is missing. The pinned bgfx build also requires OpenGL development
files and, with its default `BGFX_WITH_WAYLAND=ON`, the Wayland EGL library.
The sample checks these dependencies before fetching bgfx and prints the
installation command if any are missing. These packages are only needed for
the city sample, not the core Frontier library.

After installing the packages, rerun `bash ./run_city_sample.sh`; an existing
failed `build-city` configure can be reused. Launch from an X11 desktop session
(or a Wayland desktop with XWayland available), since the sample's native Linux
window layer uses X11.

To configure and build manually from the repository root:

```sh
cmake -S . -B build-city \
  -DCMAKE_BUILD_TYPE=Release \
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

On Linux/SBCs the sample defaults to a single-sample backbuffer to avoid
driver-dependent MSAA resolve/presentation artifacts during window resizing.
Other platforms keep the 4x MSAA default. Use `--msaa` to request 4x MSAA or
`--no-msaa` to disable it explicitly. The startup log and **Performance** window
show the selected renderer and requested MSAA mode; the log also prints the GPU
vendor/device IDs. The viewport and backbuffer are kept in sync even if a global
input event follows a resize event.

The desktop OpenGL backend requires OpenGL 3.1 or newer. CMake selects
`BGFX_OPENGL_VERSION=31`, including when updating an existing build whose cache
contains the old empty default. The pinned debug-draw shaders use integer
transform indices; bgfx's OpenGL 2.1 build supplies floating-point attributes
instead, which can stretch cone/cylinder triangles across the city and look
like cracks in other surfaces. If an older build reports **OpenGL 2.1**, rerun
the launcher to reconfigure and rebuild. An explicitly configured lower
`BGFX_OPENGL_VERSION` must be changed to `31` or higher. The renderer label
reports bgfx's compiled minimum, not the driver's maximum supported version.

If triangle seams persist with OpenGL 3.1, use **Unlit surfaces (seam test)** in
**Frontier debug**, or launch with `--unlit`. This disables surface lighting
while preserving positions, indices, depth testing, and culling. Seams that
disappear implicate the lighting shader; seams that remain need further
geometry/rasterization investigation. Report the CPU, GPU, driver string, and
whether this comparison changes the seams. The OpenGL 3.1 attribute fix does
not by itself establish the cause of every SBC seam artifact.

Renderer-selection arguments are supported, so Linux graphics issues can be
compared with a single-sample backbuffer:

```sh
bash ./run_city_sample.sh --gl --no-msaa # OpenGL
bash ./run_city_sample.sh --vk --no-msaa # Vulkan
```

These request a backend; bgfx may fall back if it cannot initialize it, so check
the renderer reported at startup. Omit `--gl`/`--vk` to let bgfx choose. On other
platforms the standard bgfx arguments such as `--d3d11`, `--d3d12`, and `--mtl`
are also accepted.

The sample applies checked, idempotent adaptations to the pinned bgfx sources
through `cmake/bgfx_diagnostics.cmake`: selected GPU information is appended to
both the C++ and C capability structures, and debug draw gets a lighting switch.
GPU names come from the active GL, Vulkan, DXGI, or Metal device rather than an
unrelated installed adapter. Revisit these adaptations when changing the bgfx
pin; configuration fails if the patch no longer applies.

Run `build-city/examples/city/frontier_city` on single-config generators. With
Visual Studio, run `build-city/examples/city/Release/frontier_city.exe`.

The first configure downloads the bgfx CMake distribution at the commit pinned
in `CMakeLists.txt`; that distribution brings its matching bgfx, bx, and bimg
submodules. Normal Frontier builds do not download or compile those dependencies.
`FRONTIER_DEBUG_TOOLS` is off by default in normal builds. The launch scripts
enable it for this sample so the read-only TLAS/cache inspection API is present;
no debug scan or bounds enumeration runs while its windows and rendering modes
remain disabled.
