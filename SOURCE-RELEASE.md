# Source candidate and practical use

Current version: **0.4.0, unpublished local research candidate**.
The authoritative current gates are in [RELEASE_CHECK.md](RELEASE_CHECK.md).
Historical audit reports preserve the state and hashes of their own runs.

## Intended use

Mesh QA and geometric fabrication/robot-path preflight for COMPAS 2 and COMPAS
FAB 2, using loaded collision geometry. Outputs expose diagnostic witnesses,
convergence, pair attribution, exclusions and unresolved states. The software
does not plan or command a robot, certify a machine, or replace its safety system.

## Install and evaluate

Use a matching CPython/platform wheel, or build from source with stable Rust:

```sh
python -m pip install ".[fab]"
python examples/consumer_install_check.py --output install-check.json --job-output job.json
compas-forge trajectory job.json --clearance .001 --articulated-tolerance .001 --serial
```

`job.json` above is a bundled official model fixture, not proof about your own
cell. See [LAB_WORKFLOW.md](LAB_WORKFLOW.md) for exporting your actual geometry,
state, trajectory, scene/tool paths and attachment phases. Rhino 8/9 use their
embedded CPython and matching native wheel; see [RHINO.md](RHINO.md).

Record input checksum, package/build hashes, dependencies, host, units,
tolerances, collision exclusions and output for every job. Preserve original
geometry before repair. Recreate prepared contexts after changing inputs.

## Evidence boundaries

The final installed suites and clean sdist consumer passed locally on Windows.
Independent CGAL/Bullet comparisons cover specified fixtures, not every mesh or
continuous robot trajectory. Mathematical motion envelopes do not establish
an exact floating-point distance-error bound. Physical margins need laboratory
data. A `clear` result is conditional on the documented geometry/motion contracts.

CLI/JSON require no visualization server. Interactive HTML currently needs
internet assets. Full GH component-graph testing, remote cross-platform CI and
physical lab acceptance remain open gates. General solid Boolean union,
controller blending, grasp mechanics and multi-robot trajectories are not claimed.

## Before publication

- Inspect source, documentation, licensing and all new artifacts; preserve
  third-party attribution and historical `SOURCE_SHA256.json`.
- Publish only after owner approval; no commit/push/tag has been performed here.
- Run remote CI before a release tag, and distribute matching audited wheels
  rather than requiring Rhino users to build a Rust extension themselves.
- Identify the exact commit and binary artifacts in reproducibility material.
- Do not claim maintainer acceptance, a DOI/paper, safety certification,
  scientific priority or universal speed superiority.

[Claims](CLAIMS.md) · [Validation](VALIDATION.md) ·
[Mathematical contracts](ROBUSTNESS.md) · [Contributing](CONTRIBUTING.md)
