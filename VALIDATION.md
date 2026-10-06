# Validation

## Current review — 2026-10-06

**Executed suite:** 35 Python tests, 4 Rust unit tests, `cargo fmt --check`, `cargo clippy -D warnings`, and release extension build.

Reproduce with the build and test commands in [README.md](README.md). The local environment used Windows x64, CPython 3.14, Rust 1.96 and the installed .NET / native SDKs. Exact dependencies are recorded in package manifests and lockfiles. Native Python extensions were rebuilt in release mode before testing.

Coverage includes concave triangulation, vertex-link diagnostics, analytical
linear TOI, rotational sweeps, initial overlap, no-hit semantics, deterministic
assembly contacts and a headless COMPAS FAB 1.1.4 UR5 trajectory using model
forward kinematics. In that retained fixture, both endpoint checks are clear and
the continuous sweep detects a converged intermediate collision.

See [CLAIMS.md](CLAIMS.md) for direct implementation-to-test links.

## Remaining validation boundaries

Self-intersection, self-touching polygons, material behaviour and robot safety are outside the checks. Ear clipping assumes simple planar faces; contact clipping is intended for convex coplanar faces. A closed triangle surface can contain another without surface intersection, so clash queries are not a general solid-containment predicate. Profile values assume metres and are built-in heuristics, not manufacturing certification. Names containing `zero_copy` are compatibility APIs: input data is copied into owned Rust memory. No comparative speedup has been established.

Interactive Rhino/Revit/Blender behaviour has not yet been exercised in this review. Record host version, input model, expected geometry, observed output and logs when performing integration tests. COMPAS FAB coverage is headless and uses model FK; it is not a ROS/MoveIt safety certification. Report benchmark inputs, hardware, release profile, repetitions and raw measurements before making comparative performance claims.

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
