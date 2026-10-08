# Validation

## Published GitHub artifact validation — 2026-10-08

Research prerelease `v0.4.0-rc.1` is published. Both remote runs passed all jobs:
15 CI checks and 17 distribution checks, including full installed-wheel suites.
GitHub Windows `cp39` and `cp313` binaries also passed inside Rhino 8/9,
with native SHA-256 equality checked against the downloaded artifacts:
`rhino8-github-release-20261008.json`, `rhino9-github-release-20261008.json`.
See [PUBLICATION_20261008.md](PUBLICATION_20261008.md) for links and remaining scope.
Earlier sections retain their original build identities and audit dates.

## Final release review — 2026-10-08

Final installed Windows wheels: CPython 3.9 **201 passed, 1 skipped**;
CPython 3.13 **235 passed, 1 skipped**; CPython 3.14 **199 passed, 2 skipped**.
The 15 new cases cover integer query budgets and HTML/script-data escaping.
Both actual Rhino hosts passed again: `rhino8-host-final-review-20261008.json`
and `rhino9-host-final-review-20261008.json`. Fresh source-consumer installation,
isolated native build, dependency check and exported-job CLI passed; see
`consumer-sdist-final-20261008.json`. Source installation is no longer an open gate.
See [RELEASE_CHECK.md](RELEASE_CHECK.md) for remaining gates and exact scope.
All sections below record earlier snapshots, not the final test count/build.

## Ecosystem review and retained-state correction — 2026-10-08

The installed reviewed wheels passed 186 tests on CPython 3.9 (CGAL skipped),
220 on CPython 3.13 (Bullet skipped; 36 CGAL fixtures included), and 184 on
CPython 3.14 (CGAL/Bullet modules skipped). Counts overlap across interpreters.
The new regression rejects invalid retained joints in partial trajectories;
full merged configurations are checked before FK and phase-boundary checks.

Current CGAL evidence: reference-cgal-reviewed-20261008.json, 36 static shared
fixtures, no boundary/solid classification disagreement. Consumer installation,
real exported job and CLI passed without dev dependencies; see
consumer-install-check-20261008.json and examples/consumer_install_check.py.
Actual host reruns: rhino8-host-review-20261008.json and
rhino9-host-review-20261008.json, both passed=true with reviewed adapter hashes.
The source archive includes new modules/tests and excludes target/venv; installing
from that source archive was not tested. Remote CI and full GH graph remain gates.

Reviewed filter ablation: 12.0541 ms versus 24.244 ms, ratio 2.0113x on the
specific retained UR5 fixture, 11 alternating runs per mode; see
benchmark-clearance-filter-reviewed-20261008.json. This is not a language comparison.

## Current five-workstream revision — 2026-10-08

Installed CPython 3.14 wheel: **181 passed, one optional PyBullet module skipped**.
Rhino's CPython 3.9: **183 passed**, including Bullet; CPython 3.13:
**181 passed, one optional module skipped**. Rust: **11 passed**, formatting and
all-target/all-feature Clippy with warnings denied passed.
This supersedes the capability restrictions in historical entries below.
Motion coverage now includes direct affine mimic, synchronized moving scene and
tool joints, composed robot/tool derivative bounds and geometric attachment
phases. See [HARDENING_20261008.md](HARDENING_20261008.md) for the independent
reference investigation, same-build performance ablation and Rhino host evidence.

## Historical fixed-attachment and user-job revision — 2026-10-07

The updated installed CPython 3.14 wheel passed **112 tests, one optional module
skipped**. pip check found no broken requirements. Rust kernels are unchanged
from the 11-test/fmt/Clippy result below.

Articulated-bound sampling now covers official UR5 and UR10e cells, both bare and
with gripper/workpiece, under a rotated/translated base frame. Fixed link
attachments and static bodies have explicit bounds. A moving UR5/gripper/beam
fixture checked 124 pair queries (148 distance evaluations), returning clear for
1 mm requested surface clearance with maximum per-entity deviation bound
0.614941 mm. This result assumes the fixture's metre units and stated numerical
and kinematic contracts; it is not a measured physical robot clearance.

The new CLI reads an actual serialized COMPAS FAB cell/state/path bundle, runs
preflight, records input/implementation hashes, and emits non-success for unknown.
The regression verifies real serialized geometry and attributed collision, not
only a mocked command response. A separate mocked test isolates the unknown exit
policy. See LAB_WORKFLOW.md for using user-supplied jobs.

This closes the earlier robot-only restriction for fixed-state attachments.
Moving tool joints, changing grasps and mimic-joint bounds remain unsupported.

## Latest bounds/solid/articulated revision — 2026-10-07

Final rebuilt-wheel regression: **103 passed, 1 optional module skipped** on
Windows CPython 3.14 (82.25 s). The skipped module is PyBullet, whose earlier
CPython 3.9 results below do not count as a rerun of this updated build.

All 11 Rust tests passed, including generated 3D swept-bound enclosure and 49
unfiltered narrowphase comparisons. Formatting and warnings-denied all-target
Clippy passed. The rebuilt CPython 3.14 wheel is used for the current Python suite.

New coverage includes nested/reversed shells, a concave U-prism with an empty
notch, unsupported open/multiple shells and isolated vertices; UR5 sampled
material-point deviation checks, an analytical rotating/prismatic chain, and
actual-FK witness confirmation. These support the derivations in ROBUSTNESS.md,
not a proof over every input.

Same-build native broadphase ablation: 310 queries, 261 rejections; 6 retained
paired samples per mode after warmups; medians 6.47988 s (sphere-only) and
4.96329 s (sphere plus endpoint bound), ratio 1.3056x. Hit flags, TOI and impact
records agreed exactly. Raw samples: benchmark-bounds-ablation.json. This isolates
the new filter; it does not compare Rust with Python or another collision library.
The machine's absolute times differed greatly from earlier runs, so historical
absolute timings below must not be used as a before/after comparison.

Previous CPython 3.9/3.13 and Rhino 8 host evidence below predates these additions.
Rebuild/retest the new features in those environments before distributing them.

## Earlier review — 2026-10-07

The finalization audit rebuilt and installed the CPython 3.14 release wheel and
ran all 86 tests directly in the isolated audit environment, without appending
the development environment to sys.path. Joint-type mismatch, negative timing
and partial native registration cleanup regressions passed. pip check found
no broken requirements. See FINALIZATION_AUDIT.md for outstanding release gates.

**Executed suite:** 86 Python tests (including 165 generated Hypothesis examples and two Trimesh differential checks), 9 Rust tests (including two Proptest properties), `cargo fmt --check`, `cargo clippy --all-targets -- -D warnings`, Criterion and release extension build.

The optional PyBullet module is skipped in the CPython 3.14 environment. In a
separate isolated CPython 3.9 environment, PyBullet 3.2.7 was built from source
and all 10 clearance/reference tests passed, including 80 seeded separated-box
distance/classification comparisons. Nine of those tests overlap the main suite;
there are 87 distinct passing tests across these environments, not 96.
This is a narrow independent oracle check, not a general collision corpus.

The 86-test Python suite was also run from a freshly built CPython 3.14 Windows
wheel in an isolated environment containing COMPAS 2.15.1, COMPAS FAB 2.0.1 and
COMPAS Robots 1.0.1. A source distribution was built successfully. The new
Linux/Windows/macOS wheel workflow is defined but has not yet run on GitHub, so
cross-platform artifact success is still a release gate rather than a claim.

For Rhino compatibility, a `cp39-win_amd64` wheel was built against the actual
CPython 3.9.10 executable deployed by Rhino 8 at
`~/.rhinocode/py39-rh8/python.exe`. In an isolated target containing COMPAS
2.15.1, that runtime imported the native extension and analysed a COMPAS mesh
successfully. The same smoke script then passed inside the Rhino 8 process via
`RunPythonScript`, exercising Rhino, its Python 3 host, COMPAS and the Rust
extension together. The extended host run additionally passed RhinoCommon mesh
conversion, duplicate-vertex repair, retained CCD at TOI 0.25 and clearance
violation detection (Rhino 8.27.25357.11371). The updated `cp313-win_amd64` wheel passed all 59 mesh/clearance tests on
CPython 3.13.13, matching Rhino 9's documented 3.13 ABI. Rhino 9 is not
installed on this machine, and Grasshopper/Rhino 9 host tests remain open.
These 59 tests overlap the main suite and do not increase its distinct count.

Reproduce with the build and test commands in [README.md](README.md). The local environment used Windows x64, CPython 3.14, Rust 1.96 and the installed .NET / native SDKs. Exact dependencies are recorded in package manifests and lockfiles. Native Python extensions were rebuilt in release mode before testing.

Coverage includes concave triangulation, degenerate faces, vertex-link
diagnostics, Euclidean tolerance welding, multi-shell volume, component-aware
genus, analytical linear TOI, rotational sweeps, independent impact-distance
verification, initial overlap, no-hit semantics, deterministic assembly
contacts, concurrent cached queries and a headless COMPAS FAB 2.0.1 / COMPAS
Robots 1.0.1 UR5 trajectory using model forward kinematics. In that retained fixture, both
endpoint checks are clear and the continuous sweep detects a converged
intermediate collision.

Full-link coverage loads seven collision meshes from the current UR5 RobotCell,
applies 11 SRDF disabled-collision pairs, checks the 10 remaining non-adjacent
self-collision pairs, verifies adaptive joint-space subdivision and detects a
converged `upper_arm_link` / `wrist_3_link` collision at approximately 84.825%
of the retained trajectory. Its independent distance residual is about
`2.58e-10`. Metamorphic fixtures also preserve analytical linear TOI under a
global rigid translation and a 1000x uniform scale.

The complete-cell fixture uses the official FAB 2
`RobotCellLibrary.ur5_gripper_one_beam` state. It constructs the permitted
robot-self, robot/tool, robot/body, attached-body/body and tool/body pairs,
applies hidden/touch/attachment/base-frame semantics, and attributes the
retained initial overlap to `body:beam` / `body:floor`. The current fixture
evaluates 31 permitted queries and rejects six with the conservative native
swept-sphere broadphase before Parry narrowphase.
An additional deterministic differential test compares 250 random linear cube
trajectories with an independent analytical slab-intersection oracle; all
hit/miss classifications and impact times agree within `1e-8`.
Generated properties independently check borrowed-buffer bounds and invalid
index counts, plus reliable closed-volume invariance under global translation.
An independent Trimesh baseline agrees on closed/open watertightness, winding
consistency and the reliable closed-cube volume for shared fixtures.
The Rust translation property found a large-coordinate cancellation defect in
the original signed-volume sum. The implementation now translates each shell
to a local anchor and uses compensated summation; the shrunk counterexample is
retained in `proptest-regressions/geometry.txt`.

Forum-commitment regressions cover reversible `Mesh.is_closed/is_manifold`
acceleration with COMPAS semantic parity, array and sparse-map JSON schemas,
explicit rejection of missing coordinates/references, analytical concave contact
area and rejection of a tilted face pair that a centroid-only plane check would
misclassify.

The local CCD experiment used 10,000 timed cached-linear repetitions after 300
warm-ups. In one release run the native-dict path measured 3.7 us median, 4.1 us
p95 and 5.0 us p99; the same query through the JSON compatibility path measured
8.5 us median. Run `benchmark_ccd.py` to retain raw samples and environment
metadata. These are machine-specific boundary timings, not a cross-library
performance result.

The batch benchmark intentionally includes 32-query individual, serial-batch
and parallel-batch workloads. Tiny cube queries do not benefit from batching on
the tested machine because Python-result construction and Rayon scheduling are
larger than the narrowphase itself. Therefore no universal batch speedup is
claimed; the practical benefit is a single registry/GIL boundary plus native
broadphase and parallelism for sufficiently substantial query sets.

A separate rigorous pyperf run on the same Windows/CPython 3.14 host measured
`1.07 ± 0.02 us` for the borrowed structural scan, `3.40 ± 0.13 us` for cached
linear CCD returning a native dictionary, and `8.78 ± 0.15 us` for the JSON
round trip. Pyperf warned that the native-dict sample did not establish sub-1%
variation with 95% confidence. The raw process-isolated result and environment
metadata are retained in `pyperf-presentation-candidate.json`; these figures
are fixture-specific and are not a cross-library speed claim.

The rigorous topology comparison uses the same already-constructed closed
16,384-quad torus. Calling both public COMPAS predicates measured
`40.0 ± 0.7 ms`; Forge including Python topology packing, FFI and both Rust
predicates measured `24.1 ± 1.8 ms` (`1.66x`), while the explicitly prepacked
kernel boundary measured `14.9 ± 1.4 ms` (`2.68x`). Pyperf warned that both
Forge workloads did not establish sub-1% variation with 95% confidence. Raw
samples and metadata are in `pyperf-topology-presentation-candidate.json`.

See [CLAIMS.md](CLAIMS.md) for direct implementation-to-test links.

## Remaining validation boundaries

Exact self-intersection classification for self-touching/degenerate polygons, material behaviour and robot safety are outside the checks. Ear clipping and contact patching assume simple planar faces; contact classification uses floating-point tolerances and is not an exact predicate. Surface distance does not prove solid penetration or containment. Profile values assume metres and are built-in heuristics, not manufacturing certification. Names containing `zero_copy` are compatibility APIs: COMPAS ingestion still performs explicit packing before Rust owns a native snapshot. No universal or cross-library speedup has been established.

Rhino 8 in-process conversion, analysis, repair, CCD and clearance are exercised, but Grasshopper and
Rhino 9 host behaviour have not yet been tested; only the Rhino 9-compatible
CPython ABI has been smoke-tested. Record host version, input model, expected
geometry, observed output and logs when performing additional integration
tests. COMPAS FAB coverage is headless and uses model FK with bounded
joint-space subdivision; it is not a ROS/MoveIt safety certification or an
exact proof over continuous coupled-joint motion. Report benchmark inputs,
hardware, release profile, repetitions and raw timings before making
comparative performance claims.

<details>
<summary>First-review historical record (superseded where the current review differs)</summary>

# Validation record

Date: 2026-09-16. Local platform: Windows x64, CPython 3.14.2, Rust/Cargo 1.96.0, .NET SDK 10.0.300. Native C++: MSVC 19.51. The .NET core-check projects target .NET 8.

## Observed results

Release native extension built for CPython 3.14. Input validation, nonconsecutive COMPAS keys, JSON failure propagation, boundary counts and closed/open fixtures passed. Three CLI checks also passed: help, valid COMPAS input and invalid JSON.

Automated test/check count: **20**. These are small regression suites, not complete correctness evidence. Build success and host runtime validation are separate. CI definitions are supplied; GitHub-hosted runs have not yet occurred.

## Corrections in this revision

- Replaced reinterpretation of mutable Python buffers with owned snapshots and validated offsets/indices.
- Corrected COMPAS key remapping in Python and JSON paths; added missing COMPAS dependency.
- Propagated unsupported shape-cast errors; corrected empty/non-manifold closure checks and metadata.

- Fixed CLI startup, preserved distinct input paths, and replaced invented timing splits with measured totals.

## Remaining release gates

- Rotational collision detection remains an approximation; validate against an independent reference on adversarial trajectories.
- Edge incidence is not full vertex-manifold or self-intersection analysis. Concave face triangulation needs a robust method.
- Validate contact areas, clipping orientation, welding tolerance, preflight units and additional JSON variants on a broader fixture set.
- Automatic COMPAS method interception, macOS/Linux and the full Python matrix remain unverified.

## Reproduce

Run the commands in README.md from a fresh checkout. Record dependency lockfiles, operating system, host version and full test output. For benchmarks also report CPU, release profile, input generator, repetitions and raw timings. No general performance, novelty, manufacturability or security guarantee follows from the present tests.

Status: suitable for transparent source review as a research prototype. A stable production release is not certified by this record.


</details>
