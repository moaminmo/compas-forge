# Implementation evidence

Final release status, installed-suite counts and source-consumer evidence:
[RELEASE_CHECK.md](RELEASE_CHECK.md). Reports below retain their original build
identities; final host reports contain `host-final-review` in their filenames.

Current ecosystem/consumer review: REVIEW_FA_20261008.md. Reviewed wheels include
retained-state validation for partial paths; COMPAS CGAL 0.10.0 independently
agrees on 36 shared static boundary/solid fixtures. Actual Windows Rhino 8/9
reruns and clean consumer installation passed. Historical rows below retain
their original scope; current host artifacts are *-host-review-20261008.json.

Current evidence and claim boundaries: [2026-10-08 audit](HARDENING_20261008.md).
It supersedes earlier host and motion limitations below. Rust enables retained
native geometry, batched parallel queries and released-GIL compute; no universal
Rust-versus-C++/Python speedup or mathematical novelty is established. The measured
filter ablation isolates an algorithmic optimization within the same Rust build.

Latest extensions and their restricted contracts are documented in
[ROBUSTNESS.md](ROBUSTNESS.md): tight swept bounds, opt-in single-shell solid
containment and articulated error bounds including fixed-state attachments. Older surface-only and
piecewise-rigid limitations below still apply to default/legacy APIs. Earlier
Rhino host checks do not validate these newest additions.

Reviewed 2026-10-07. A passing build is distinguished from executed numerical tests and interactive host validation.

| Capability | Implementation | Verification |
|---|---|---|
| Conditional continuous surface clearance | [Rust](src/clearance.rs), [contract](CLEARANCE.md) | analytical near misses, rotation-only violation, exhausted budget, and 80 PyBullet box comparisons; numerical error bound is not proven |
| Prepared cell reuse | [Adapter](python/compas_forge/cell.py) | snapshot isolation, close lifecycle and partial-joint state regressions; paired end-to-end timings retained |
| Boundary and vertex-manifold diagnostics | [Code](src/geometry.rs) | [Evidence](tests/test_mesh_api.py): Open triangle, closed tetrahedron and disconnected vertex fans |
| Concave polygon triangulation | [Code](src/geometry.rs) | [Evidence](tests/test_mesh_api.py): U-shaped face preserves analytical area |
| Translation and rotation sweeps | [Code](src/lib.rs) | [Evidence](tests/test_mesh_api.py): analytical translation TOI, initial overlap, no-hit semantics and rotation-only intermediate contact |
| Verified impact geometry | [Code](src/lib.rs) | [Evidence](tests/test_mesh_api.py): converged TOI plus independently recomputed distance residual |
| Deterministic tolerance welding | [Code](src/geometry.rs) | [Evidence](tests/test_mesh_api.py): Euclidean tolerance across spatial-hash cell boundaries and recorded merge distance |
| Multi-shell volume and genus | [Code](src/geometry.rs) | [Evidence](tests/test_mesh_api.py): oppositely oriented disconnected shells do not cancel and component-aware genus is reported |
| Self-intersection diagnostics | [Code](src/geometry.rs) | [Evidence](tests/test_mesh_api.py): RTree triangle broad phase and crossing non-adjacent faces |
| Borrowed-buffer structural scan | [Code](src/lib.rs) | [Generated evidence](tests/test_properties.py): bounds and invalid-index oracles without an owned Rust input copy |
| Independent mesh baseline | [Differential evidence](tests/test_trimesh_differential.py) | Trimesh agreement on closed/open watertightness, winding and reliable closed-cube volume |
| Native retained CCD kernel | [Criterion fixture](benches/native_ccd.rs) | Correctness-gated fixture with the Python/JSON/mesh-construction boundary excluded |
| Native cached result path | [Code](src/lib.rs) | [Benchmark](benchmark_ccd.py): correctness-gated same-work native-dict and JSON compatibility timings |
| COMPAS FAB trajectory preflight | [Adapter](python/compas_forge/__init__.py) | [Evidence](tests/test_compas_fab_integration.py): UR5 model FK, clear endpoints and converged intermediate collision |
| Full robot-link collision preflight | [Adapter](python/compas_forge/__init__.py) | [Evidence](tests/test_compas_fab_integration.py): COMPAS FAB 2 RobotCell, seven loaded UR5 collision links, SRDF exclusions, adaptive joint subdivision, environment hit and converged intermediate self-collision |
| Complete RobotCell trajectory preflight | [Adapter](python/compas_forge/__init__.py) | [Evidence](tests/test_compas_fab_integration.py): robot/tool/body pair classes, attachment/base frames, hidden and allowed-touch semantics on the official UR5 gripper/beam cell |
| Native batch CCD and conservative broadphase | [Rust core](src/lib.rs) | [Evidence](tests/test_mesh_api.py): serial/parallel ordered equivalence and explicit far-pair rejection; [cell regression](tests/test_compas_fab_integration.py): broadphase rejection count retained |
| FAB trajectory contract validation | [Adapter](python/compas_forge/__init__.py) | [Evidence](tests/test_compas_fab_integration.py): URDF joint-limit and timestamp-derived velocity violations |
| Physical collision timing | [Adapter](python/compas_forge/__init__.py) | [Evidence](tests/test_compas_fab_integration.py): segment TOI mapped to declared trajectory seconds, duration and normalized time |
| Rhino native-host compatibility | [Guide](RHINO.md) | Actual Windows Rhino 8/9 in-process smoke tests, including Grasshopper DataTree marshaling; full GH component graph untested |
| Deterministic assembly interfaces | [Code](src/lib.rs) | [Evidence](tests/test_mesh_api.py): repeated canonical contact ordering and analytical interface area |
| Concave planar contact patches | [Code](src/lib.rs) | [Evidence](tests/test_mesh_api.py): analytical U-shaped area and rejection of centroid-only false coplanarity |
| Reversible COMPAS Mesh acceleration | [Adapter](python/compas_forge/plugin.py) | [Evidence](tests/test_plugin_acceleration.py): parity with COMPAS fixtures, nesting and restoration |
| Single-pass topology predicates | [Code](src/lib.rs) | [Benchmark](benchmark_topology.py): end-to-end and prepacked workloads on the same closed torus |
| COMPAS 1/2 mesh JSON parsing | [Parser](src/parser.rs) | [Evidence](tests/test_mesh_api.py): sparse remapping and explicit failure for incomplete/ambiguous schemas |
| Python / CLI validation | [Code](python/compas_forge) | [Evidence](tests/test_cli.py): Sparse vertex keys, malformed buffers and CLI input handling |

## Current-review corrections

Tolerance-aware spatial welding; component-aware topology and volume; explicit
volume reliability; deterministic diagnostics; distance-error propagation;
Parry 0.31 velocity-contract adaptation; residual-verified CCD; concurrent cached
queries with short-lived `RwLock` access, retained `Arc<TriMesh>` geometry, GIL
release and a native Python-dict result path.
The full-link COMPAS FAB 2 path adds cached URDF collision geometry, SRDF
disabled-collision semantics, adaptive
joint-space subdivision, deterministic earliest-pair selection and explicit
link/obstacle attribution. Rigid-translation and uniform-scale metamorphic tests
guard CCD coordinate and unit invariance.
Additionally, 250 deterministic random linear cube trajectories match an
independent analytical slab-intersection oracle for hit/miss and TOI.
Hypothesis supplies 165 generated examples for borrowed-buffer bounds/index
contracts and translation-invariant reliable volume.

## Explicit boundaries

Self-intersection checks use floating-point triangle predicates with an RTree broad phase, including shared-edge foldover and shared-vertex overlap diagnostics; they do not constitute exact symbolic classification of every degeneracy. Ear clipping assumes simple planar faces. Contact interfaces triangulate simple planar concave faces and return patches, method, plane deviation and reliability, but remain tolerance-based floating-point results. Surface-only queries cannot distinguish containment; explicit solid clearance adds restricted single-shell or even-odd semantics. Profile values assume metres and are built-in heuristics, not manufacturing certification. Names containing `zero_copy` are compatibility APIs: COMPAS ingestion still performs explicit packing before Rust owns a native snapshot. No universal or cross-library speedup is claimed.

## Historical evidence scope — 2026-10-07

86 Python tests (including 165 generated examples and two Trimesh differential
checks) and 9 Rust tests (including two Proptest properties) on the local
Windows/CPython 3.14 environment, plus Rust formatting, warnings-denied Clippy,
release builds and Windows CPython 3.9/3.13/3.14 wheel checks. Rhino 8 was
tested in-process; Rhino 9 was tested only at its documented CPython 3.13 ABI
boundary, with 59 mesh/clearance regressions on the updated wheel. The new clearance API passed CPython 3.9/3.13/3.14;
one additional optional PyBullet comparison test passed on CPython 3.9.
Continuous clearance is conditional on a user-supplied numerical-error margin,
does not handle solid containment, and does not bound articulated-path error.
The CCD and topology benchmarks retain raw samples and machine
metadata. The topology comparison includes Python packing in the end-to-end
Forge workload and labels the prepacked boundary separately. These checks cover
the stated fixtures; they are not a proof of correctness on all inputs. No
universal performance, state-of-the-art, production-readiness or scientific
novelty claim is made.
