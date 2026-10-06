# Implementation evidence

Reviewed 2026-10-06. A passing build is distinguished from executed numerical tests and interactive host validation.

| Capability | Implementation | Verification |
|---|---|---|
| Boundary and vertex-manifold diagnostics | [Code](src/geometry.rs) | [Evidence](tests/test_mesh_api.py): Open triangle, closed tetrahedron and disconnected vertex fans |
| Concave polygon triangulation | [Code](src/geometry.rs) | [Evidence](tests/test_mesh_api.py): U-shaped face preserves analytical area |
| Translation and rotation sweeps | [Code](src/lib.rs) | [Evidence](tests/test_mesh_api.py): analytical translation TOI, initial overlap, no-hit semantics and rotation-only intermediate contact |
| COMPAS FAB trajectory preflight | [Adapter](python/compas_forge/__init__.py) | [Evidence](tests/test_compas_fab_integration.py): UR5 model FK, clear endpoints and converged intermediate collision |
| Deterministic assembly interfaces | [Code](src/lib.rs) | [Evidence](tests/test_mesh_api.py): repeated canonical contact ordering and analytical interface area |
| Python / CLI validation | [Code](python/compas_forge) | [Evidence](tests/test_cli.py): Sparse vertex keys, malformed buffers and CLI input handling |

## Second-review corrections

Concave triangulation; vertex-link diagnostics; nonlinear rotational sweeps; regression coverage for analytical area and time of impact.

## Explicit boundaries

Self-intersection, self-touching polygons, material behaviour and robot safety are outside the checks. Ear clipping assumes simple planar faces; contact clipping is intended for convex coplanar faces. A closed triangle surface can contain another without surface intersection, so clash queries are not a general solid-containment predicate. Profile values assume metres and are built-in heuristics, not manufacturing certification. Names containing `zero_copy` are compatibility APIs: input data is copied into owned Rust memory. No comparative speedup has been established.

## Evidence scope

35 Python tests and 4 Rust unit tests on the local Windows/CPython 3.14 environment, plus Rust formatting and Clippy checks. These checks cover the stated fixtures; they are not a proof of correctness on all inputs. No universal performance, state-of-the-art, production-readiness or scientific novelty claim is made. The repository includes CI configuration; remote CI execution has not been asserted.
