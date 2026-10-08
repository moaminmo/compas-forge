# Final local release review — 8 October 2026

COMPAS Forge 0.4.0 is a **research release candidate**. The owner approved
GitHub publication and the source was pushed. Binary release is gated on CI;
no robot command was performed. Local success is not remote GitHub CI success
or safety certification. Local audit statements below predate publication.

The first remote run exposed two distribution issues: unconditional PyO3
`extension-module` prevented Unix Rust tests from linking libpython, and
manylinux did not automatically select the setup-python matrix interpreter.
The build now follows PyO3's Maturin-managed extension flag (Maturin >=1.9.4)
and selects the matrix interpreter explicitly. CI must pass after these fixes.

## Executed gates

| Gate | Result and scope |
| --- | --- |
| Installed Windows CPython 3.9 wheel | 201 passed, 1 optional CGAL module skipped; Bullet included |
| Installed Windows CPython 3.13 wheel | 235 passed, 1 optional Bullet module skipped; 36 CGAL fixtures included |
| Installed Windows CPython 3.14 wheel | 199 passed, 2 optional reference modules skipped |
| Rust quality | 11 tests, fmt, locked all-target/all-feature warnings-denied Clippy passed; final changes are Python/docs/CI only |
| Release wheels | `cp39`, `cp313`, `cp314` Windows x64 rebuilt with `maturin build --release --locked` |
| Actual Rhino 8 / Rhino 9 WIP | Both passed in-process smoke tests, including conversion, repair, CCD, clearance, solid containment, moving UR5, phases and GH DataTree marshaling |
| Clean source consumer | Fresh CPython 3.13 venv installed the sdist with `[fab]`, compiling its native wheel through isolated PEP 517 build; `pip check` passed |
| Consumer geometry and path | Official UR5/gripper/beam fixture: clear under stated numerical/articulated margins; 124 pair queries, 120 broadphase rejections, 22 distance evaluations |
| Public trajectory CLI | Exported consumer job returned `clear`, exit 0, hashes/versions included, `robot_commands_sent=false` |
| Source archive inspection | New Python modules and regression tests included; no target/venv/native binaries or .git contents found |
| Working-tree whitespace | `git diff --check` passed; only line-ending notices |

Counts overlap across interpreters and must **not** be added. Skipped modules
are optional independent-reference environments, not failed product tests.

Current host artifacts:
[Rhino 8](rhino8-host-final-review-20261008.json),
[Rhino 9](rhino9-host-final-review-20261008.json).
Source-consumer evidence: [consumer-sdist-final-20261008.json](consumer-sdist-final-20261008.json).
Earlier reports/benchmarks retain their own build hashes; they are historical
evidence, not silently relabeled as the final binary.

## Final corrections

- Invalid retained joints cannot hide behind a partial trajectory: check full
  merged values, types, finiteness and limits before FK.
- Reject bool, fractional, nonfinite and nonpositive subdivision/worker budgets.
- Escape report text and script-embedded JSON; reject nonfinite JSON values.
- Correct stale Rhino 9 status and distinguish direct COMPAS CPython support
  from optional RPC. Explain Rust's concrete implementation value, not language
  superiority.
- Add source-consumer installation and real CLI execution to distribution CI.

## Remaining release gates

1. Run the defined Linux/macOS/Windows remote CI and artifact matrix after an
   approved push. Python 3.10/3.12 are declared but not individually locally audited.
2. Test a complete GH Python component graph, repeated recompute, cache cleanup
   and UI workflow on target Rhino versions. DataTree smoke does not close this gate.
3. Evaluate representative lab jobs, CAD tessellation error, units and configured
   exclusions. Interactive HTML currently uses external CDN assets; CLI/JSON are offline.
4. Before physical execution, obtain measured or valid bounds for tracking,
   calibration, tool/workpiece geometry and controller interpolation; use the
   laboratory's independent safety system and supervised acceptance procedure.

Remaining method boundaries: f64 predicates are not exact certificates;
clearance is conditional on the numerical error margin; controller splines,
grasp dynamics, multi-robot motion and intersecting solid-shell Boolean union
are not covered. See [ROBUSTNESS.md](ROBUSTNESS.md) and
[Persian review](REVIEW_FA_20261008.md). No known failing regression remains in
the executed suite, but absence of all possible bugs cannot be established.

## Publication procedure (not executed)

Review the complete diff and newly added files, keep historical manifests
unchanged, select the approved release contents and commit only with approval.
Push and run CI before tagging/publishing binary artifacts. Cite the resulting
commit and wheel hashes in presentation evidence; no maintainer adoption,
industrial certification, novelty priority or universal speedup is claimed.
