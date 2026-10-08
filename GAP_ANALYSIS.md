# COMPAS Forge: defensible ecosystem gap

This note defines what the project can claim in a technical presentation. It
is deliberately narrower than a marketing claim and should be revised when
the ecosystem changes.

## What already exists

- COMPAS core deliberately exposes mesh-mesh intersection as a plugin point;
  the default implementation raises `PluginNotInstalledError` when no plugin
  supplies it: <https://github.com/compas-dev/compas/blob/main/src/compas/geometry/intersections.py>
- COMPAS CGAL provides compiled C++ geometry algorithms through Nanobind and
  NumPy. Therefore “compiled geometry for COMPAS” and “non-Python kernels” are
  not novel claims: <https://github.com/compas-dev/compas_cgal>
- Current COMPAS FAB supports local PyBullet collision checking as well as ROS
  and MoveIt planning backends. Therefore Forge must not claim to be the first
  local or headless collision solution:
  <https://compas.dev/compas_fab/latest/>
- COMPAS FAB's planner interface intentionally delegates planning-scene and
  collision behaviour to backends:
  <https://compas.dev/compas_fab/latest/developer/generated/compas_fab.backends.interfaces.PlannerInterface.html>
- `compas_ifc` already uses a NumPy/Shapely broadphase/narrowphase for IFC
  element contact and collision. Therefore Forge must not claim the first
  broadphase in the wider COMPAS ecosystem:
  <https://github.com/compas-dev/compas_ifc>

## The product gap Forge targets

Forge combines capabilities that are normally split across mesh repair,
discrete collision engines and planning backends:

1. A local, headless **preflight evidence layer** for COMPAS meshes: topology,
   degeneracy, winding, self-intersection diagnostics, component-aware volume,
   explicit reliability flags and deterministic structured output.
2. **Continuous swept collision** for rigid link motion, rather than only
   endpoint or discrete-state collision checks. Every reported impact carries
   the Parry termination status and an independently recomputed distance
   residual; uncertain results remain conservative and are labelled.
3. A **COMPAS FAB full-link adapter** that bakes URDF collision origins, reuses
   retained native meshes, computes FK at adaptively subdivided joint samples,
   applies RobotCell/SRDF disabled-collision semantics, checks environment and
   remaining non-adjacent self pairs, and reports the earliest implicated pair.
   This is a bounded approximation of coupled-joint motion, not an exact safety
   proof.
4. Two honest data paths: an allocation-free borrowed-buffer structural scan
   for already-contiguous data, and an owned retained snapshot for safe GIL-free
   and concurrent CCD. “Zero-copy everywhere” is explicitly not claimed.
5. A FAB 2 `RobotCellState` path covering the same five robot/tool/body pair
   classes as its PyBullet collision contract, but sweeping continuously between
   sampled states. Ordered native batches use a conservative swept-sphere AABB
   before Parry narrowphase and report rejection/evaluation counts.

The strongest presentation sentence is therefore:

> COMPAS Forge is a reproducible geometry and trajectory preflight layer for
> COMPAS and COMPAS FAB, combining deterministic mesh diagnostics with
> convergence-aware continuous collision evidence.

## Why Rust is justified

Rust is an implementation choice, not the novelty claim. It is justified here
only where measurements and architecture support it:

- retained `Arc<TriMesh>` snapshots avoid rebuilding acceleration structures;
- `RwLock` permits concurrent read-only queries while mutation stays exclusive;
- PyO3 releases the GIL around owned collision kernels;
- borrowed buffer scans can inspect contiguous Python/NumPy memory without an
  additional Rust input copy;
- typed error propagation prevents a failed distance query from silently
  becoming a collision result;
- Parry supplies maintained double-precision broadphase/narrowphase and
  nonlinear rigid-motion shape casts.

Rust does not improve floating-point accuracy by itself, and no universal
speedup over CGAL, libigl, PyBullet or MoveIt has been established.

## Claims allowed after the current verification gates pass

| Claim | Required evidence | Current status |
| --- | --- | --- |
| deterministic diagnostics | repeated-output tests and sorted reports | covered |
| tolerance-aware welding | cross-cell Euclidean test and weld audit | covered |
| microsecond cached linear fixture on the tested machine | release `pyperf` result with hardware and raw JSON | provisional; machine-specific |
| zero additional input copy in structural scan | borrowed-buffer API and allocation contract | covered for that kernel only |
| GIL-free concurrent retained CCD | thread test and owned registry design | covered |
| full-link COMPAS FAB demonstration | loaded UR5 geometry, FK, self/environment pair report | covered headlessly |
| exact coupled-joint safety | exact continuous articulation solver | not claimed |
| exact self-intersection proof | exact predicates and degeneracy policy | not claimed |
| fastest COMPAS collision engine | fair external benchmark corpus | not claimed |

## Evidence still needed before the event

- run rigorous `pyperf` measurements on the presentation machine and retain the
  raw JSON plus environment metadata;
- create a fixed public fixture corpus containing valid, open, non-manifold,
  self-intersecting, near-degenerate and extreme-scale meshes;
- compare shared predicates with Trimesh and, where installable, CGAL/libigl;
  explain semantic disagreements instead of treating one library as truth;
- extend the generated-property suite into long-running fuzzing for buffer
  parsing and topology (the initial Hypothesis and Proptest suites are present);
- record the UR5 demo and one honest failure/uncertainty case;
- keep robot safety, exact arithmetic and universal performance outside the
  claims unless new evidence is produced.
