# Changelog

## 0.4.0 — presentation candidate, 2026-10-06

- Added high-level, explicitly named APIs for mesh analysis, repair, fabrication
  preflight, assembly contacts and frame-based swept collision.
- Added a headless COMPAS FAB UR5 trajectory adapter and regression: endpoint
  checks are clear while the continuous sweep detects and locates the
  intermediate collision.
- Split translation-only CCD onto Parry's linear shape cast and rotational CCD
  into bounded angular substeps. Results now expose the method, substep count,
  convergence status, conservative-estimate flag and local/world impact data.
- Removed synthetic impact data from no-hit results and handle initial overlap
  explicitly. Contact geometry is labelled reliable only on a converged solve.
- Made assembly-contact output deterministic and canonicalised part-pair order.
- Added Rust formatting, Clippy and unit-test gates to CI, alongside the Python
  matrix and optional COMPAS FAB integration dependency.
- Corrected example filenames and replaced compatibility `zero_copy` calls in
  user-facing examples with the high-level APIs.

## Unreleased — source release review, 2026-09-18

Report actual profile mass limits and reject unknown profile names. Added two regressions covering file and buffer APIs.

Added a practical source-release guide, reproducible issue form and citation metadata; refreshed README navigation and evidence. See [SOURCE-RELEASE.md](SOURCE-RELEASE.md) for remaining acceptance gates.

## Unreleased — second review, 2026-09-16

Concave triangulation; vertex-link diagnostics; nonlinear rotational sweeps; regression coverage for analytical area and time of impact.

Replaced the incorrectly attributed benchmark with direct API measurements and withdrew its old chart/banner.

Added an illustrated README and a capability-to-evidence matrix.

## Unreleased — 2026-09-16

- Replaced reinterpretation of mutable Python buffers with owned snapshots and validated offsets/indices.
- Corrected COMPAS key remapping in Python and JSON paths; added missing COMPAS dependency.
- Propagated unsupported shape-cast errors; corrected empty/non-manifold closure checks and metadata.

- Fixed CLI startup and added three CLI regressions; corrected reported timings and normalised collision-time units.

See VALIDATION.md for remaining release gates.
