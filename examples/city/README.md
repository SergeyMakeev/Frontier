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
geometry. Ideal-cut nodes become ready after first visibility to demonstrate
Frontier's current/ideal readiness model.

The UI is split into independent, movable ImGui windows so diagnostics do not
cover one another. The global **Debug windows** menu in the top bar toggles
each widget independently and provides **Show all** / **Hide all** actions;
each window can also be closed with its title-bar button. Only
**Performance** is open by default. **Frontier debug**
controls simulation freeze,
hierarchy-level tinting (green top nodes, yellow intermediate nodes, red
leaves), optional scene-wide wireframe rendering, LOD and contribution
thresholds, and camera modes. Wireframe can also be toggled directly from the
top-bar **Rendering** menu and composes with hierarchy tinting. **Scene stats**
contains entity, cut, streaming, cache, simulation, and camera status.
**Performance** graphs frame history and breaks sample CPU time into UI,
simulation/database, camera, Frontier selection, debug draw, streaming, and
bgfx submission, alongside bgfx render-thread, GPU, wait, draw, primitive, and
transient-buffer counters. **Scene hierarchy** is a live ImGui tree of each
reusable Frontier topology with current/ideal selected-entry counts.

Free camera uses **WASD** to move, **Q/E** to descend/ascend, and right-mouse
drag to look. **Freeze camera / cull state** captures the active culling camera,
switches to the free debug camera, and renders the captured frustum as
translucent magenta planes.

From the repository root:

```sh
cmake -S . -B build-city \
  -DFRONTIER_BUILD_CITY_SAMPLE=ON \
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
