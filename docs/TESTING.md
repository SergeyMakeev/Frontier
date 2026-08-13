# Testing Frontier

Frontier's tests are organized around the distinction between caller
contracts, internal invariants, selection correctness, and performance. A
unit-test run is always a Debug build with caller checks, internal assertions,
and serialized-subtree validation enabled. SIMD backends, instrumented builds,
randomized lifecycle models, and sanitizers catch different classes of
failure.

## Correctness suite

Run the normal suite with:

```sh
bash ./run_unit_tests.sh
```

On Windows use `run_unit_tests.bat`. The runner configures Debug, builds the
`frontier_unit_tests` target, and executes both BVH4 and BVH8 versions of every
test. A regular CMake build with `FRONTIER_BUILD_TESTS=ON` does the same: the
configured/default branch width is linked into `frontier_tests`, and the other
width is linked into `frontier_tests_alternate`.

The suite contains:

- scalar, SIMD, frustum, screen-error, camera, and degenerate-math tests;
- serialized-layout, builder-limit, allocator-ownership, stale-handle, and
  public contract tests;
- exact current/ideal frontier tests for shared readiness, mounted coverage,
  both current-cut policies, query reuse, masks, and contribution culling;
- TLAS lifecycle, motion-group, copy-on-write bounds, collection, and cache
  invalidation tests, including sparse-to-dense bounds-overlay promotion;
- deterministic randomized TLAS churn checked against a live-instance model;
- deterministic randomized node-readiness transitions checked against an
  independent complete-cover model, including repeated unmount/remount;
- bit-identical serial/parallel selection and concurrent independent-query
  readers over one published database snapshot.

The randomized tests use fixed seeds. A failure is therefore reproducible and
does not introduce CI flakiness.

## Defensive checks

`FRONTIER_CHECK` enforces caller-visible contracts when
`FRONTIER_CONTRACT_CHECKS=ON`, which is the default and is mandatory for the
normal unit suite. Expected streaming races, such as a generation-stamped
handle becoming stale, remain safe no-ops or return an invalid result instead
of firing a contract failure.

`FRONTIER_ASSERT` protects library-owned counters, stack pairing, fanout, and
coverage bookkeeping in Debug builds. These checks are absent from Release hot
paths. Disabling caller checks does not disable these internal Debug assertions.

The repository CI additionally runs AddressSanitizer plus
UndefinedBehaviorSanitizer with Clang and ThreadSanitizer with GCC. The latter
exercises both the host `parallelFor` path and distinct `SpatialQuery` readers.
An LLVM source-coverage job publishes separate line, region, function, and
branch results for library code while excluding tests and third-party sources.

## Architecture and instrumentation matrix

CI builds GCC, Clang, MSVC, clang-cl, and AppleClang configurations across
x86-64, ARM64, BVH4, BVH8, AVX2, SSE2, NEON, and forced-scalar backends.
Locally, the most important additional configurations are:

```sh
cmake -S . -B build-scalar -DFRONTIER_FORCE_SCALAR=ON
cmake -S . -B build-stats -DFRONTIER_STATS=ON
cmake -S . -B build-bvh4 -DFRONTIER_BVH_WIDTH=4
```

The stats configuration has a contract test that confirms counters are
actually populated. Normal builds test that the API returns immutable zero
counters without retaining per-query instrumentation state.

CI also runs one Debug configuration with
`FRONTIER_VALIDATE_SUBTREES=OFF`. In that mode, tests confirm that registration
still rejects an invalid format envelope while accepting structurally corrupt
internal arrays without scanning them. Do not use this mode with untrusted
serialized input.

## Coverage

Measure library sources and public inline code, excluding GoogleTest and test
sources. Line coverage is useful for finding untouched error paths, but it is
not a substitute for the model-based tests: a traversal branch can execute
without proving that it emitted a complete cut.

For LLVM source-based coverage, configure the test executable with
`-fprofile-instr-generate -fcoverage-mapping`, run it with an
`LLVM_PROFILE_FILE`, merge the profile with `llvm-profdata`, and report the
test executable with `llvm-cov report`. Keep line and branch/region results
separate; tools that expose only line coverage must not be described as branch
coverage.

## Performance coverage

`frontier_bench` and its matched `frontier_bench_payload32` companion measure
end-to-end public workflows with eight- and four-byte payload words: assembly
versus flattening, raw and cached selection, mixed-readiness cut policy,
shared readiness fanout, serialized registration, TLAS scale, instance
lifecycle, mount lifecycle and retention, stable motion groups, and bounds
overrides. Each result records `frontier_payload_bytes` in its benchmark
context so files remain self-describing after collection.
`frontier_machine_bench` characterizes kernels and the machine independently.
See [BENCHMARKING.md](BENCHMARKING.md) for the cases and collection procedure.

Benchmarks are measurements, not correctness tests. Repository performance
runners build Release with `FRONTIER_STATS=OFF`,
`FRONTIER_CONTRACT_CHECKS=OFF`, and `FRONTIER_VALIDATE_SUBTREES=OFF`, then use
the automatic native SIMD width. Record repeated runs on an idle machine and
compare medians and variation before accepting a small change. Never feed that
unchecked profile untrusted or malformed inputs.

## Deliberate integration-level gaps

The repository does not emulate renderer/GPU resource completion, inject an
allocation failure at every retained allocation, or run multi-hour
million-object soak tests in routine CI. Those belong in the embedding
engine's integration and release qualification. Frontier's deterministic
10,000-placement benchmarks and lifecycle torture tests provide fast coverage
of the same mechanisms, but do not replace application-scale soak testing.
