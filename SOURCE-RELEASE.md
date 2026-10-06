# Source release review — 6 October 2026

## Corrections in this review

Profile limits are explicit in native reports; CLI mass checks use the selected limit. Unknown profile names fail rather than silently selecting timber; `default` remains an explicit timber alias. The presentation candidate adds convergence-aware rigid-motion CCD, deterministic assembly interfaces and a tested COMPAS FAB trajectory adapter.

## Intended use

**Mesh diagnostics and fabrication preflight.** This is a source distribution for reproducible research and supervised evaluation. A public source release does not certify a machine, material, building or autonomous workflow.

| Contract | Current implementation |
|---|---|
| Input | COMPAS mesh or JSON; metres for fabrication profiles |
| Output | Topology metrics, profile decisions, repair candidates and reports |
| Executed local checks | 35 Python tests, 4 Rust tests, Rust fmt/Clippy, release build |
| Implementation | [Source](src/geometry.rs) |
| Reproducible evidence | [Tests](tests/test_mesh_api.py) |

## Workshop or laboratory workflow

Run diagnostics, inspect boundary/nonmanifold witnesses, then compare the selected profile limits with the actual machine and material. Preserve the original geometry before repair.

Record the input checksum, source revision, dependency versions, host version, units, tolerances, settings and output checksum for every evaluated specimen. The example fixtures establish specific behaviours; representative project data must be evaluated separately.

## Acceptance still required

Self-intersection/solid containment and machine-specific manufacturing acceptance are not established. The COMPAS FAB adapter uses piecewise Cartesian sweeps between sampled FK frames; it is not a substitute for a certified robot safety system or full-link collision checker.

No comparison in this review establishes universal optimality or state-of-the-art superiority. Algorithm choice is justified by the task and tested numerical behaviour. Independent benchmarks should compare the same inputs, correctness criteria and hardware, retaining raw timings and failure cases.

## Publication package

- README, license, source, build metadata and regression tests are included.
- Publish each project as a separate repository, preserving third-party attribution.
- Build from the documented dependencies; proprietary SDKs and compiled outputs are excluded from the source archive.
- Run the provided CI on the actual repository before tagging a release. Local results are not GitHub-hosted CI results.
- Cite the exact version or commit used; no DOI, paper acceptance or external certification is asserted.

## Retained publication evidence

The [executed example](publication/README.md) includes raw results, a reproducible figure, source links and a SHA-256 manifest. The [introduction draft](publication/linkedin.md) describes this evidence within its tested scope.
