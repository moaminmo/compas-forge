# Changelog

## Unreleased — local presentation hardening

- Final release review: reject bool/float/nonfinite subdivision and worker
  budgets consistently; added 12 regression cases.
- Escape HTML report text and script-embedded JSON; reject nonfinite JSON.
  Added three report serialization/security regressions.
- Reorganized the README around actual product contribution, installation,
  reproducible evidence and remaining release gates; corrected stale Rhino 9 claims.
- Added a source-artifact consumer-install check to the distribution workflow.

- Corrected partial-trajectory input validation to check all retained/full joint
  values before FK, including limits and nonfinite values.
- Added 36 optional COMPAS CGAL 0.10.0 differential fixtures, a separate CI job,
  clean-consumer installation check, current ecosystem review and Persian Q&A.

- Added explicit even-odd multi-shell containment and adjacent-face overlap diagnostics.
- Added per-pair motion allowances, local FK-verified refinement and swept-AABB
  clearance filtering with same-build ablation and independent reference corpus.
- Added synchronized moving scene and tool-joint paths, composed derivative bounds
  and continuous geometric attachment-phase validation. No physical grasp guarantee.
- Verified Windows Rhino 8 and Rhino 9 hosts and Grasshopper DataTree interoperability.

- Extended articulated bounds to fixed-state tools, rigid link/tool-attached
  workpieces and static bodies, with official UR5 and UR10e cell regressions.
- Added `compas-forge trajectory`: trusted serialized cell/state/path input,
  JSON evidence with input hash/versions, and non-success exit for unknown.
- Added LAB_WORKFLOW.md for checking user-supplied jobs, not just bundled fixtures.

- Added tighter rotational swept AABBs with arc/chord error inflation and an
  explicit ablation switch. Retained same-build UR5 evidence (1.3056x on 310 queries).
- Added opt-in single-shell solid clearance with initial containment checks,
  conservative unknown outcomes for unsupported inputs and cached eligibility checks.
- Added robot-only articulated deviation bounds for linear joint trajectories,
  spatial subdivision policy, clearance inflation and actual-FK witness validation.
- Added ROBUSTNESS.md with exact-arithmetic derivations and a limitation ledger.

- Added GIL-free Rust surface-clearance verification with Lipschitz interval
  bounds, deterministic serial/parallel batches and explicit unknown outcomes
  on resolution/budget exhaustion. Numerical-error assumptions are exposed.
- Added prepared RobotCell snapshots with explicit cleanup, geometry reuse across
  paths and preservation of unspecified joints in partial trajectories.
- Added analytical clearance regressions, optional independent PyBullet cases,
  snapshot lifetime tests, and paired setup/reuse benchmarks with raw samples.
- Extended the Rhino host script to RhinoCommon conversion, repair, retained CCD
  and clearance verification.

- Finalization audit: reject model/trajectory joint-type mismatches and invalid
  negative/nonfinite timestamps; clean up partially registered link geometry on
  failures. Current isolated suite: 86 passing tests plus one optional PyBullet
  test executed separately on CPython 3.9 (80 seeded reference cases).
- Added FINALIZATION_AUDIT.md with supported integration boundaries, prioritized
  release gates and measured-performance limitations.

- Migrated the robot adapters and retained UR5 demos to COMPAS FAB 2.0.1,
  COMPAS Robots 1.0.1 and the current `RobotCell`/`RobotCellLibrary` API while
  retaining compatibility with the legacy wrapper shape in the adapter.
- Integrated `RobotCell.robot_semantics`/SRDF disabled-collision pairs into
  full-link preflight by default, with an explicit opt-out and result metadata,
  so intended link contacts do not become avoidable false positives.
- Added complete COMPAS FAB 2 `RobotCellState` trajectory preflight covering
  robot/tool/rigid-body pair classes, attached-object and robot-base frames,
  hidden objects, touch links, touch bodies and deterministic pair attribution.
- Added deterministic native batch CCD: one registry read, one GIL release and
  optional Rayon execution for an ordered query set. Retained meshes now cache
  their origin radius and use a conservative swept-sphere AABB rejection before
  invoking Parry's narrowphase.
- Added structured COMPAS FAB trajectory validation for point/joint shape,
  finite values, URDF position and declared-velocity limits, monotonic timing
  and segment average velocity. Collision sweeps now fail before geometry work
  when this contract is violated.
- Mapped every attributed collision back to declared trajectory time, duration
  and normalized time fraction while keeping those fields explicitly empty for
  untimed trajectories.
- Expanded CI to Python 3.9/3.11/3.13/3.14 on Linux, Windows and macOS, and added a
  tag/manual artifact workflow that builds and smoke-tests platform wheels plus
  the source distribution before anything is published.
- Built and smoke-tested ABI-specific Windows wheels with Rhino 8's actual
  CPython 3.9 runtime and a standalone CPython 3.13 runtime matching Rhino 9's
  documented ABI. The Rhino 8 wheel also passed an in-process host test through
  `RunPythonScript`; Rhino 9 host execution remains explicitly unverified.
- Upgraded Parry, PyO3, SIMD-JSON, RTree and earcutr; adapted linear CCD to
  Parry's current world-velocity contract and retained the rotated-frame
  analytical regression test.
- Replaced six-decimal string welding with deterministic spatial hashing and an
  explicit Euclidean tolerance, including merge-distance audit data.
- Added degenerate-face diagnostics, component-aware genus, per-component
  absolute volume and explicit volume reliability.
- Propagated clash-distance failures instead of converting them to collisions;
  exposed the surface-distance classification limit and sorted results.
- Added independent distance-residual verification at CCD impact poses.
- Reworked cached meshes around `Arc<TriMesh>` and `RwLock`, released the GIL
  during native sweeps, accepted packed registration buffers directly and added
  a native Python-dict result path while retaining JSON compatibility.
- Added a correctness-gated CCD benchmark with raw samples and p50/p95/p99
  reporting.
- Added full robot-link COMPAS FAB trajectory preflight with loaded URDF
  collision meshes, adaptive joint subdivision, environment and self-collision
  attribution, deterministic earliest-hit selection and scoped cache cleanup.
- Added rigid-translation and uniform-scale metamorphic CCD tests. Current local
  validation also includes 250 deterministic trajectories checked against an
  analytical slab oracle.
- Added RTree-accelerated non-adjacent triangle self-intersection diagnostics
  and connected them to validity and volume-reliability contracts. Current local
  coverage is 86 Python tests and 9 Rust tests, plus the optional reference test.
- Added an allocation-free borrowed-buffer structural scan with an explicit
  zero-owned-input-copy contract; retained collision meshes still use safe owned
  snapshots for GIL-free concurrent execution.
- Added process-isolated `pyperf` workloads, Hypothesis-generated invariants and
  Trimesh differential checks for shared watertightness, winding and volume
  semantics.
- Added Criterion and Proptest. A generated large-coordinate counterexample
  exposed cancellation in the volume sum; volume is now evaluated in a local
  component frame with compensated summation.
- Retained a process-isolated rigorous pyperf result with machine metadata and
  the benchmark tool's stability warning rather than presenting rounded timing
  claims without raw evidence.
- Audited every public claim from the July COMPAS forum thread. Added explicit,
  nested and reversible acceleration for the non-pluggable COMPAS
  `Mesh.is_closed/is_manifold` methods without import-time monkey-patching.
- Upgraded assembly contact extraction from convex-only clipping to triangulated
  planar patches for simple concave faces, checked all face vertices against the
  contact plane, released the GIL and exposed method/reliability/unit metadata.
- Hardened COMPAS 1.x array and 2.x sparse-map parsing: incomplete schema pairs,
  nonnumeric keys, missing coordinates and missing vertex references now fail
  explicitly instead of being ignored or replaced with zero.
- Added a dedicated single-pass borrowed topology kernel for the two COMPAS Mesh
  predicates. The fair rigorous comparison records `1.66x` end-to-end and
  `2.68x` at the explicitly prepacked kernel boundary on one 16,384-face fixture,
  together with pyperf's stability warnings and raw process-isolated samples.
- Documented the defensible COMPAS ecosystem gap and ruled out unsupported
  novelty, universal-speed and robot-safety claims.

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
