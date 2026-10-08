# Rhino installation and verification

The 2026-10-08 audit exercised actual Windows Rhino 8 and Rhino 9 hosts, including
tight bounds, solid containment, per-pair offsets and a moving FAB cell path.
See HARDENING_20261008.md and the recorded host JSON results.
After the retained-state correction, both hosts were rerun successfully; exact
final adapter hashes are in rhino8-host-final-review-20261008.json and
rhino9-host-final-review-20261008.json. Full GH graph/recompute remains unverified.

COMPAS Forge contains a compiled Rust extension. Rhino therefore needs a wheel
matching its embedded CPython ABI and platform:

Use Rhino/Grasshopper **Python 3 (CPython)**. The legacy Python 2 IronPython
engine cannot load CPython/PyO3 native extensions and is not supported.

| Host | Python runtime | Required Windows wheel tag | Local evidence |
| --- | --- | --- | --- |
| Rhino 8 | CPython 3.9 | `cp39-cp39-win_amd64` | built with Rhino 8's deployed interpreter and tested inside Rhino 8 |
| Rhino 9 WIP | CPython 3.13 | `cp313-cp313-win_amd64` | built with Rhino 9's deployed interpreter and tested inside Rhino 9 |

## Install

Prefer a wheel produced by the release workflow. In Rhino's Python 3 package
manager, install the matching wheel together with `compas>=2,<3`. Do not install
the `cp39` wheel into Rhino 9 or the `cp313` wheel into Rhino 8.

For a command-line installation into Rhino 8's user runtime on Windows:

```powershell
$rhinoPython = "$env:USERPROFILE\.rhinocode\py39-rh8\python.exe"
& $rhinoPython -m pip install compas==2.15.1 C:\path\to\compas_forge-0.4.0-cp39-cp39-win_amd64.whl
```

Restart the Python 3 engine after installation. McNeel documents this as
**Tools > Reload Python 3 (CPython) Engine** in the Script Editor.

## Host smoke test

Run this as Python 3 in Rhino or a Grasshopper Python 3 component:

```python
from compas.datastructures import Mesh
import compas_forge

mesh = Mesh.from_vertices_and_faces(
    [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    [[0, 1, 2]],
)
report = compas_forge.analyze_mesh(mesh)
assert report["is_valid"] is True
assert report["boundary_edges_count"] == 3
print(compas_forge.__version__, report)
```

The equivalent reusable script is
[`examples/rhino_host_smoke.py`](examples/rhino_host_smoke.py).

The current Windows audit launched Rhino 8 with that script through
`RunPythonScript`. Rhino's embedded CPython 3.9.10 loaded COMPAS 2.15.1 and
COMPAS Forge 0.4.0 inside Rhino 8.27.25357.11371. The extended script returned
`is_valid=true` with three boundary edges, converted an actual RhinoCommon mesh
through `compas_rhino`, welded one duplicate vertex, found the expected retained
CCD impact at 0.25, and classified a below-threshold clearance as `violation`.

This verifies package loading, the COMPAS adapter, the Python/Rust boundary and
native topology, repair, CCD and clearance calls. It does not validate Rhino viewport interaction,
robot safety. The updated smoke script also checks actual Grasshopper DataTree
conversion through compas_ghpython in both hosts. It does not execute a complete
Grasshopper Python-component graph/recompute lifecycle.

## Release gate

Before publishing, execute the smoke test inside Rhino 8 and Rhino 9 on every
supported operating system, then run one representative mesh-repair and one
retained collision query. Record Rhino build, Python version, wheel filename,
COMPAS version and output. The GitHub wheel job is necessary evidence, but it
does not replace an in-host test.
