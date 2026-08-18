# Frontier documentation

These documents describe the current codebase, public contract, architecture,
build, tests, benchmarks, and measured behavior.

Current documents state the active contract and implementation directly.
Descriptions of superseded APIs, layouts, and design alternatives belong in
the historical records.

## Current documentation

- [README](../README.md): project overview, supported workflows, build options,
  and headline current performance.
- [Current codebase](CODEBASE.md): source map, runtime architecture, data
  layouts, frame lifecycle, build configuration, and current constraints.
- [API guide](API.md): progressive integration guide from authoring through
  streaming, selection, submission, motion, and collection.
- [API reference](API_REFERENCE.md): complete public signatures, invariants,
  ownership rules, and compile-time configuration.
- [Behavioral design](frontier_design.md): current model and correctness
  contract.
- [Architecture](ARCHITECTURE.md): current data structures, layouts,
  publication model, query paths, and memory checkpoints.
- [Testing](TESTING.md): current correctness matrix and release verification.
- [Benchmarking](BENCHMARKING.md): current workloads, runners, and measurement
  protocol.
- [Performance](PERFORMANCE.md): absolute current
  results on M2 Max, Cortex-A72, i9-12900K, and EPYC 9654.

## Historical records

[HISTORY.md](HISTORY.md) and the [archive](archive/README.md) contain retired
designs, old APIs, optimization experiments, rejected prototypes, and
before/after measurements. They are engineering records and are not part of
the current integration contract.
