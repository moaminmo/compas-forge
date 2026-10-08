# COMPAS forum commitments audit

Source post: <https://forum.compas-framework.org/t/compas-forge-a-high-performance-rust-extension-for-geometry-validation-mesh-repair-and-assembly-clearance/1112>

This file maps the July 2026 forum statements to the current implementation.
It is a compatibility and evidence checklist, not a repetition of marketing
language.

| Forum statement | Current support | Evidence / correction |
| --- | --- | --- |
| Geometry and topology validation | supported | boundary/non-manifold vertex and edge diagnostics, degeneracy, winding, components and self-intersections |
| Duplicate vertices and automatic welding | supported with explicit tolerance | deterministic Euclidean spatial hash and per-merge audit; repair does not claim to close holes or remove self-intersections |
| Face-orientation correction | supported for adjacent orientable regions | flip audit is returned; non-orientable/global outward classification is not claimed |
| Assembly collision and clearance | supported as surface-distance evidence | contact/crossing/containment cannot always be distinguished; reliability is reported |
| Interactive HTML/Three.js reports | retained | reporting path exists; host/browser matrix still requires presentation-machine validation |
| Rhino and Grasshopper integration | actual Windows Rhino 8 and Rhino 9 host tests passed | conversion, native queries, FAB path, attachment phases and GH DataTree conversion passed; full GH component graph/recompute remains a release gate |
| Transparent acceleration of `Mesh.is_closed/is_manifold` | supported as explicit reversible opt-in | COMPAS 2.15 methods are not pluggable extension points, so import-time replacement would be misleading and unsafe; use `with compas_forge.mesh_acceleration():`; one rigorous 16,384-face fixture measured `1.66x` end-to-end |
| “Zero-copy” execution | supported only for the borrowed structural scan | normal COMPAS ingestion packs data once; retained CCD intentionally owns a safe native snapshot |
| Rotational CCD | supported with bounded angular subdivision | rigid poses use quaternion interpolation and Parry nonlinear casts; this is not an exact articulated-motion proof and rotational latency is not claimed to be microseconds |
| Exact TOI to six decimals | not a universal guarantee | solver status and independently recomputed distance residual are exposed; accuracy depends on geometry, scale and convergence |
| Parallel masonry contact solver | supported for tolerance-based planar face contact | current polygon contract and adversarial coverage must be stated; “exact” is not claimed with floating-point predicates |
| COMPAS 1.x / 2.x JSON parser | supported for documented mesh schemas | sparse-key remapping and malformed input are tested; “universal” is not claimed for every historical/custom serializer |
| 15–42x COMPAS speedup table | withdrawn as a general claim | the earlier benchmark did not establish a fair universal comparison; current pyperf/Criterion evidence names exact workloads and retains raw data |

## Presentation-safe summary

The forum's durable product direction is preserved: a Rust-backed geometry and
fabrication preflight layer for COMPAS. The defensible improvement is that every
important output now carries a clear numerical/topological contract, regression
evidence and an explicit limitation instead of relying on “exact”, “instant” or
“zero-copy” as universal descriptions.
