# Five-workstream hardening audit — 2026-10-08

The later ecosystem review and retained-state correction supersede test counts
and current-build artifact pointers on this page: see REVIEW_FA_20261008.md and
the first section of VALIDATION.md. Reviewed installed suites: 186 on CPython
3.9, 220 on 3.13 with CGAL, and 184 on 3.14, with documented optional skips.
New host outputs are rhino8-host-review-20261008.json and
rhino9-host-review-20261008.json; both passed. Previous artifacts remain historical.

This is a local engineering audit, not a robot-safety certificate, acceptance by
COMPAS maintainers, or a claim of new mathematical algorithms. Nothing was
committed, pushed or published. SOURCE_SHA256.json was not regenerated.

## Implemented scope

| Workstream | Delivered | Boundary still open |
| --- | --- | --- |
| Independent evidence | 120 seeded rotated-box cases at scales 0.001/1/1000; raw and normalized Bullet plus convex bounded least-squares oracle; nonconvex U represented independently as a union of three boxes; near-clearance and fast-rotation cases | Not exhaustive CCD validation against CGAL/IPC, not a physical-cell study |
| Geometry validity | Duplicate triangles, shared-edge foldovers, shared-vertex overlap diagnostics; explicit opt-in even-odd multi-shell/cavity semantics; cached per-shell representatives | Floating-point, not exact predicates; overlapping shell Boolean union unsupported |
| Performance | Per-pair E_a+E_b allowances in one native batch; depth-bounded local articulated refinement with FK reuse; swept-AABB clearance rejection with ablation switch and counters | General speed superiority not established; refinement may increase runtime |
| Motion scope | Direct affine mimic semantics; synchronized scene and per-link articulated tool paths; composed robot/tool bounds; continuous attachment-phase validation | Physical grasp stability, free tool carrying a body and multiple independently articulated robots not covered |
| Distribution/hosts | Windows CPython 3.9/3.13/3.14 wheels; isolated installs; actual Rhino 8 and Rhino 9 host tests; actual Grasshopper DataTree conversion and moving FAB path in host | Remote Linux/macOS CI not run; complete GH Python-component graph/recompute rehearsal remains |

**Do not describe all five workstreams as universally complete.** The scoped
capabilities above are implemented; the right column is an explicit release
boundary rather than an unsupported promise.

## Independent-reference investigation

The first default-margin Bullet run disagreed in 20 of 120 classifications.
Its raw output is retained in reference-corpus-default-margin-20261008.json.
Investigation showed that Bullet's margin could exceed tiny box half-extents;
even a scale-relative positive margin left scale-dependent distance errors in
some millimetre cases. These observations concern this Bullet build/query setup,
not a universal claim that Bullet or COMPAS FAB is broken.

The final corpus retains raw Bullet distances, and also runs Bullet on
unit-normalized geometry. A third independent oracle solves

    min || R_a u - R_b v - (c_b-c_a) ||²,
    -h_a <= u <= h_a, -h_b <= v <= h_b,

using SciPy bounded-variable least squares. For separated convex boxes this is
a globally convex nearest-point problem. Geometry is normalized only for the
reference solves; Forge still receives the original scale. Solver convergence
is checked. Threshold classifications use this third oracle, not a hand-edited
answer or a tolerance enlarged to hide the disagreement.

Reference script: tests/reference_corpus.py. Evidence:
reference-corpus-20261008.json, including versions, native binary hash, poses,
thresholds, raw results and timings. Python tracemalloc peak is explicitly NOT
native/Rust/Bullet peak memory. This corpus does not establish penetration-depth
accuracy or continuous-time completeness. Additional nonconvex checks live in
tests/test_convex_reference.py and tests/test_adversarial_clearance.py.

## Performance evidence and interpretation

benchmark_clearance_filter.py alternates filtered/unfiltered runs on the same
retained UR5/gripper/beam path, same wheel, same serial execution policy. It
excludes mesh preparation. benchmark-clearance-filter.json records seven runs
per mode, full result counters and implementation hashes. The initial measured
medians were 11.59 ms filtered versus 23.64 ms unfiltered (2.04x), with interval
distance evaluations reduced from 148 to 22 and clear classification preserved.
Use the current raw artifact for final measurements; do not compare absolute
timings across older benchmark sessions or call this a Rust-versus-C++ result.

The separate per-pair/global-bound ablation (benchmark-pair-bounds.json) did NOT
show a material speedup on that fixture (ratio ~0.98). Its benefit is less
unnecessary inflation; do not advertise an unmeasured speed gain. The negative
result is retained. Both algorithms can be useful without every fixture being
faster. These small local benchmarks are not robust cross-machine tail-latency
or native-memory benchmarks.

The final 11-repeat-per-mode rerun is retained separately in
benchmark-clearance-filter-final-20261008.json: median **12.2587 ms filtered**
versus **24.6405 ms unfiltered**, ratio **2.01004x** on this local fixture.
reference-corpus-final-20261008.json reruns the 120 cases against the final
CPython 3.9 native build: zero classification mismatches and maximum
scale-normalized witness error 1.7763568394002505e-15. These fixture results
are evidence, not an exhaustive precision or no-missed-collision proof.

Exact release-candidate reruns after the final Python compatibility fix:
benchmark-clearance-filter-release-candidate-20261008.json records **11.657 ms**
filtered versus **23.3617 ms** unfiltered (**2.00409x**, 11 runs per mode).
reference-corpus-release-candidate-20261008.json again records 120 cases,
zero classification mismatches and the same normalized witness error.
Use these release-candidate files for current binary hashes; earlier measurements
remain historical. Rebuilding a Windows native binary need not preserve its hash.

## Motion and work-budget contract

See ROBUSTNESS.md for derivations. For each pair, inflate by E_a+E_b. Bisection
divides the allowance by four. Only two clear children certify their parent.
Actual FK and synchronized scene poses recheck every reported articulated
violation. An unresolved leaf stays unknown. Refinement depth is limited to 8,
default 2; distance budgets apply to each native query, not the entire path.
Topology/winding initialization has separate cost. Distance counters include
initial solid distance checks; FK, pair-query and rejection counters are exposed.

scene_poses cannot override robot-attached geometry. It adds the world/world
pairs which a static snapshot policy could omit. Body touch exclusions still
apply. Tool joint trajectories use `tool_trajectories`, retained per-link geometry
and composed derivative bounds, not an unrelated body pose path. Attachment
changes use `verify_compas_fab_cell_phases`; teleporting geometry, discontinuous
joints or changed visibility at boundaries are rejected. This is geometric
preflight, not physical grasp certification. See ROBUSTNESS.md for the contracts.
The CLI accepts synchronized tool paths and rejects unknown bundle keys.

External axes represented as ordinary supported joints inside the robot model
follow the same fixed-axis derivation. A separately controlled external axis is
not automatically covered. Physical tracking, calibration, controller blending,
elasticity, forces, and dynamic feasibility remain outside this checker.

## Host and packaging evidence

Final installed-wheel regressions: CPython 3.9 **183 passed** (including Bullet),
CPython 3.13 **181 passed, one optional module skipped**, CPython 3.14
**181 passed, one optional module skipped**. The new tool/phase tests run in all
three. Skipped Bullet is not counted as a successful reference comparison there.

The actual installed Rhino 9 WIP was discovered during this audit, correcting the
earlier assumption that no Rhino 9 host was available. It initialized CPython
3.13.13. A Grasshopper assembly-loading difference between Python.NET runtimes
was fixed by adding the assembly directory then resolving by assembly name.
The original failed-host result is retained, not represented as a successful run.

Host scripts: examples/rhino_audit_runner.py and examples/rhino_host_smoke.py.
Earlier outputs: rhino8-host-final-20261008.json and
rhino9-host-final-20261008.json. The phase-enabled rerun writes
rhino8-host-phases-20261008.json and rhino9-host-phases-20261008.json;
only passed=true establishes success. The newer reports include adapter hashes.
After the final null-tool-configuration compatibility correction the exact
release-candidate host reports are rhino8-host-release-candidate-20261008.json
and rhino9-host-release-candidate-20261008.json. Earlier artifacts are retained
as historical evidence, not interchangeable binary hashes.
The scripts exercise RhinoCommon conversion, repair, native CCD/clearance,
per-pair offsets, containment, Grasshopper DataTree marshaling and a moving
COMPAS FAB UR5/gripper/beam path. They do not create or execute physical motion.

Wheels are in target/release-audit-wheels (ignored local build artifacts).
The isolated host package directories are under target/host-audit-cp39 and
target/host-audit-cp313. Never install one interpreter's native wheel into another.
No production/user project dependencies were upgraded globally.

## Remaining release decisions

1. Tool-joint and geometric attachment-phase support is implemented and tested;
   industrial gripper/path fixtures and physical grasp validation remain separate.
2. Full GH component execution/recompute and cross-platform CI need their own
   acceptance results. In-host DataTree tests alone are not that evidence.
3. A real user/lab geometry/path corpus, with consent, is still needed for an
   adoption/performance case study. No physical safety guarantee follows.
4. Publication or any commit/push requires the user's explicit approval.
