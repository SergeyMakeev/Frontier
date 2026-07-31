# HLodTree

External hierarchical-LOD cut selection, implemented from `hlod_design.md`.
See `ARCHITECTURE.md` for the implemented architecture, why it is fast, and
the journal of optimization experiments (including the failed ones).

The library owns no meshes or render state: nodes carry an opaque
caller-supplied 64-bit payload (an id, a pointer — never interpreted), and
`World::selectCut()` returns the payloads to draw (the *cut*), the *ideal
cut* (what would be drawn if everything were loaded, with `NEEDS_EXPANSION`
tags doubling as expansion requests), and payload load requests at the
cut's frontier. The runtime API is fully handle-based — the World keeps no
id index and no hash maps; requests and expansion entries carry the
`NodeHandle` to act on, and setup-time handles are composed from attach
results plus authored node indices.

Key pieces (see the design doc for the full rationale):

- `include/hlod/math.h` — minimal SIMD-friendly math (`float4` / `float8`
  only), tri-state masked frustum culling, wide box tests. Intended to be
  swappable for a production SIMD library later.
- `include/hlod/page.h` — immutable flat page: preorder SoA arrays, packed
  `meta` word, BVH8-style wide child blocks.
- `include/hlod/builder.h` — offline `HLodBuilder`, establishes all layout
  invariants and emits wide blocks.
- `include/hlod/world.h` — runtime `World`: instances over a wide dynamic
  top-level BVH, page attach/detach (topology streaming), payload residency,
  epoch-stamped per-view scratch, the single-pass pruned cut, LRU garbage
  collection, and lazy sublinear bounds refit for moving nodes/instances
  (applied at the next `selectCut`, not per move).

## Building

Requires CMake >= 3.24 and a C++20 compiler. Dependencies (GoogleTest,
Google Benchmark) are fetched automatically via CMake `FetchContent`.

```
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release          # unit tests
build\bench\Release\hlod_bench.exe         # performance tests
```

Options: `-DHLOD_BUILD_TESTS=OFF`, `-DHLOD_BUILD_BENCH=OFF`, `-DHLOD_AVX2=OFF`.

## Dependencies

| Package | Version | Use |
|---|---|---|
| GoogleTest | v1.17.0 | unit tests |
| Google Benchmark | v1.9.4 | performance tests |
