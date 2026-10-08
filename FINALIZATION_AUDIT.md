# Finalization audit — 2026-10-07

Superseded by [the 2026-10-08 hardening audit](HARDENING_20261008.md).
The records below are historical, including their test counts and unsupported
motion lists. Current installed CPython 3.14 wheel: 181 passed, one optional skip.

## Latest usability and fixed-attachment update

The earlier robot-only restriction is removed for rigidly attached fixed-state
tools/workpieces and static scene objects. The derivative bound uses fixed
attachment placement and entity-local radii. Official UR5 and UR10e fixtures,
transformed bases, direct link attachments and a moving gripper/workpiece path
are covered. Current installed-wheel suite: 112 passed, one optional skip.

`compas-forge trajectory` checks a user-supplied serialized cell/state/path and
returns JSON with input, native-binary and adapter hashes. Unknown never exits
success. See LAB_WORKFLOW.md. This command does not drive any physical robot.

## Latest robustness extension (supersedes the older capability limits below)

Final local gates: **103 Python tests passed, one optional module skipped;
11 Rust tests passed; formatting and all-target warnings-denied Clippy passed.**
Python tests used the rebuilt CPython 3.14 release wheel.

- Added endpoint swept AABBs inflated by the proven exact-arithmetic arc/chord
  bound r*theta²/8. Same-build ablation on 310 retained UR5 queries rejected 261
  pairs and reduced the median from 6.47988 s to 4.96329 s (1.3056x). Collision,
  impact time and impact records matched the old broadphase on every query.
  Raw evidence: benchmark-bounds-ablation.json. Separate absolute timings were
  slower than prior sessions; cross-session speedup is NOT claimed. The slower
  run is retained as benchmark-swept-bound-moving.json rather than discarded.
- Added opt-in initial solid containment using winding numbers for connected,
  closed embedded single-shell inputs. Unsupported inputs return unknown.
- Added opt-in robot-only linear-joint interpolation error bounds, subdivision
  from spatial tolerance, inflated clearance and actual-FK witness rechecking.
- See ROBUSTNESS.md for derivations, numerical assumptions and the current
  limitation ledger. Mathematical geometric bounds do not certify floating-point
  arithmetic, embeddedness, controller behaviour or physical machine safety.
- Previous Rhino-host and CPython 3.9/3.13 results below are historical evidence;
  these new additions require renewed host/ABI verification before distribution.

## Earlier presentation-candidate audit

Decision: a research presentation candidate with useful implemented capabilities.
Do not call it a fully validated production robot-safety system or universal
COMPAS accelerator. This audit combines source inspection, executed regression
tests and the earlier benchmark/host evidence; it is not a proof over all inputs.

## Corrections executed in this audit

- Reject trajectory joint types that disagree with the actual robot model.
  Incorrect types otherwise change the angular/prismatic subdivision policy.
- Reject negative and nonfinite time-from-start values.
- Track each retained link mesh as soon as registration succeeds, allowing both
  robot and cell adapters to clean up after a later registration failure.
- Added failure regressions for all three cases.
- Rebuilt and installed the CPython 3.14 release wheel in the isolated audit
  environment: 86 Python tests passed, one optional module skipped; pip check
  found no broken requirements. All 9 Rust tests, formatting and all-target
  warnings-denied Clippy passed again after the native changes.
- Added native GIL-free continuous surface-clearance verification using a
  relative-speed Lipschitz bound and adaptive interval subdivision. Outcomes
  distinguish clear, violation and unknown. See CLEARANCE.md for the explicit
  numerical-error assumption and surface/containment limitations.
- Added PreparedRobotCell: snapshot inputs, retain native geometry across
  trajectories, preserve unspecified joints, serialize lifecycle operations,
  and release resources explicitly. Mutation requires a new snapshot.
- Passed 10 clearance/reference tests under isolated CPython 3.9, including
  80 seeded comparisons with independently built PyBullet 3.2.7. Nine tests
  overlap the main suite: total distinct passing tests across environments is 87.
- Extended the actual Rhino 8 host test: RhinoCommon-to-COMPAS conversion,
  duplicate-vertex welding, retained CCD and clearance all passed.
- Rebuilt the updated CPython 3.13 Windows wheel and passed all 59 mesh/clearance
  tests against it. This is standalone ABI evidence, not a Rhino 9 host test.

## Product position

Forge's proposed value is a common geometry and trajectory preflight report:
mesh diagnostics, retained native geometry, continuous rigid sweeps between
sampled configurations, pair attribution and explicit solver quality.

COMPAS FAB already offers PyBullet and MoveIt collision/planning backends.
COMPAS CGAL already exposes compiled geometry through Nanobind/NumPy.
Using Rust alone is neither a research novelty nor evidence of superior accuracy.

Sources checked:
- https://compas.dev/compas_fab/latest/
- https://github.com/compas-dev/compas_cgal

## Scope that can be defended

| Integration | Evidence | Remaining boundary |
| --- | --- | --- |
| COMPAS Mesh | public adapters and predicate regressions | not every datastructure or plugin |
| COMPAS FAB 2 / Robots | UR5 and gripper/beam fixtures, FK and pair semantics | not every robot, backend or controller |
| Rhino 8 Windows | in-process conversion, analysis, repair, CCD and clearance smoke | Grasshopper data-tree/component workflow remains untested |
| Rhino 9 | updated CPython 3.13 Windows wheel, 59 mesh/clearance tests | no actual Rhino 9 host test |
| CGAL/libigl/IFC/OCC | ecosystem inspection | no claim of direct adapter coverage or measured superiority |
| Linux/macOS | workflow definitions | remote builds and host tests have not run |

The cell adapter holds tool joint configurations and unattached body poses fixed
over the robot trajectory. Coordinated moving tools, external axes and multiple
robots require explicit trajectories and further implementation.
Joint-step subdivision limits sampling increments; it does not bound Cartesian
deviation between the real articulated path and the interpolated link sweep.
The reported impact-distance residual is a numerical consistency check, not a
bound on physical robot positioning error.

## Prioritized work before release

| Priority | Work | Acceptance evidence |
| --- | --- | --- |
| P0 | Grasshopper and Rhino 9 workflow | Rhino 8 extended smoke passed; execute component/data-tree workflow and actual Rhino 9 host test |
| P0 | Broader independent collision corpus | initial 80 PyBullet box cases passed; extend to rotated/nonconvex and near-degenerate meshes, retain disagreements |
| P0 | Installation rehearsal and platform wheels | clean install, full tests, checksum/version records; execute remote CI only after publication approval |
| Done, scoped | Persistent prepared RobotCell | repeated trajectories, input isolation and close regressions; paired setup/reuse timings retained; memory not benchmarked |
| Done, conditional | Rigid surface clearance | clear/violation/unknown, analytical near misses, rotation-only violation and exhausted-budget tests; no rigorous floating-point or articulated-motion bound |
| Partial | Multi-segment timing coverage | nonuniform/untimed helper regression added; exhaustive coverage across every adapter remains open |
| Later | Dynamic tools/external axes/multiple robots | explicit state contract and representative fixtures before claiming support |

Continuous clearance is implemented for linear translation plus shortest-arc
rotation. Clear is conditional on the specified numerical margin bounding
distance-oracle and arithmetic errors. The API reports
`numerical_error_bound_proven=false`. Merely sampling joint configurations more
densely is still insufficient to establish a bound on the true articulated path.

## Performance evidence and useful next optimization

Existing results are fixture-specific: 1.66x for paired topology predicates
including packing, 2.68x at a prepacked boundary, and about 3.40 microseconds for
one cached linear CCD fixture. Refer to retained pyperf JSON and its stability
warnings.

New paired local measurements compare fresh snapshot preparation on every call
against reuse of one prepared snapshot. They are not Rust-versus-Python or
cross-library benchmarks. Serial execution, three warmups, alternating order,
identical output assertions, raw samples and source hashes are retained:

| Fixture | Repeats | Fresh median | Reused median | Ratio |
| --- | --- | --- | --- | --- |
| Stationary UR5/gripper/beam cell | 20 | 310.7973 ms | 9.0161 ms | 34.47x |
| Moving UR5, intermediate self-collision | 12 | 1312.15985 ms | 1010.43485 ms | 1.30x |

The stationary case exits on an existing beam/floor collision and is deliberately
setup-dominated; do not advertise its ratio as general trajectory acceleration.
One-off preparation cost was 302.86 ms / 305.03 ms respectively and must be paid
before reuse. Moving-path FK/query work remains dominant. Raw evidence:
benchmark-prepared-cell.json and benchmark-prepared-moving.json.
Tiny batches can be slower in parallel; no unconditional improvement is claimed.

Freeze new features once the P0 checks pass. Keep a working presentation build,
raw results and one documented limitation case. Production adoption and upstream
inclusion require maintainer review and real-user feedback.
