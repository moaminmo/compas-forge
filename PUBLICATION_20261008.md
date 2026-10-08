# GitHub publication record — 8 October 2026

Published research prerelease:
[COMPAS Forge v0.4.0-rc.1](https://github.com/moaminmo/compas-forge/releases/tag/v0.4.0-rc.1).
Package metadata is `0.4.0`; the GitHub prerelease flag defines its research
candidate status. No PyPI upload or physical robot execution was performed.

## Source and validation

- Implementation commit: `a8b6eb7d34e3d3bcc676022bbd76d308e1b80d59`.
- Released source/tag: `4c5c7ff0868e7e4cd0750c91c95eef716bfd36d0`, including
  Unix PyO3 test-linking and manylinux interpreter-selection fixes.
- [CI run](https://github.com/moaminmo/compas-forge/actions/runs/37828558383):
  all **15 jobs passed** (Rust quality, two independent-reference jobs,
  Linux/Windows/macOS × Python 3.9/3.11/3.13/3.14).
- [Distribution run](https://github.com/moaminmo/compas-forge/actions/runs/37828663584):
  all **17 jobs passed**, including installed-wheel tests and a clean sdist
  consumer geometry/path/CLI check.

The first remote attempt exposed build/distribution issues and was not
published. This release points to the corrected, successfully validated source.

## Published assets

19 assets: **16 wheels + one sdist + two Rhino-host reports**.
The wheel matrix is Python 3.9/3.11/3.13/3.14 on Windows x64, Linux x86_64,
macOS Intel and macOS ARM64. Each uploaded file's GitHub digest and byte size
were compared with the verified download before publication. The release tag
was checked to point to the exact source SHA above.

GitHub-produced Windows wheels were exercised inside their target Rhino hosts;
native extension hashes matched the downloaded wheels:

- [Rhino 8 report](rhino8-github-release-20261008.json), CPython 3.9.10,
  Rhino 8.27.25357.11371: passed.
- [Rhino 9 report](rhino9-github-release-20261008.json), CPython 3.13.13,
  Rhino 9 WIP 9.0.26237.15343: passed.

Those checks cover RhinoCommon conversion, repair, CCD, surface/solid clearance,
pair offsets, moving UR5 preflight, attachment phases and GH DataTree marshaling.
They are not a complete Grasshopper Python component-graph/recompute test.

## Remaining boundaries

This prerelease is for supervised research evaluation, not industrial safety
acceptance. Full GH component workflow and physical lab acceptance remain open.
Numerical-error certification, controller blending, grasp dynamics, multi-robot
motion and general intersecting-shell solid union are not claimed. Physical
tracking/calibration/tessellation margins need validated laboratory data.
Python 3.10/3.12 are declared but not separately release-matrix audited.

Historical reports/benchmarks retain their original build hashes and are not
relabeled as universal guarantees about these binaries. See [contracts](ROBUSTNESS.md),
[release check](RELEASE_CHECK.md) and [claims](CLAIMS.md). Documentation-only
publication bookkeeping follows the immutable tag without modifying the tested code.
