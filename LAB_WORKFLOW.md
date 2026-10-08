# Using Forge on a real COMPAS FAB job

Updates and release evidence: [HARDENING_20261008.md](HARDENING_20261008.md).
Current ecosystem review: [REVIEW_FA_20261008.md](REVIEW_FA_20261008.md).
After installing the matching local wheel with its FAB extra in a clean CPython
environment, run `python examples/consumer_install_check.py --output install-check.json --job-output job.json`.
This exports a real model fixture for the public CLI; it does not command a robot.
The optional bundle key `scene_poses` maps visible unattached entity keys (for
example `body:obstacle`) to a list of world poses, one per trajectory point.
Frames or `[x,y,z,qx,qy,qz,qw]` are accepted. Scene translations interpolate
linearly; rotations follow the shortest arc. Attached entities cannot be
overridden. Tool joints remain fixed unless `tool_trajectories` supplies a mapping
of tool names to complete JointTrajectory objects with matching point times.
Moving tool joints require `--articulated-tolerance`. Unknown bundle fields
are rejected instead of silently ignoring unmodeled motion.

CLI options `--solid --solid-rule even_odd` enable explicit alternating
material/cavity shell semantics; default solid mode remains single-shell.
`--max-articulated-refinements 2` controls local refinement depth (0..8).

This is offline preflight, not a robot driver. It checks the supplied geometry
and trajectory model and never sends controller commands. Use it before a
supervised dry run, not as a replacement for machine safety systems.

## Export your job

In the environment where you constructed your COMPAS FAB 2 cell and trajectory:

```python
import compas
compas.json_dump({'cell': cell, 'state': state, 'trajectory': trajectory}, 'job.json')
```

Use your actual RobotCell, RobotCellState and JointTrajectory, with collision
meshes loaded. Serialized mesh descriptors preserve their loaded meshes; a URDF
filename alone does not guarantee available geometry. The checker rejects missing
collision geometry instead of silently checking an incomplete robot. Load only
trusted COMPAS data bundles. Geometry, joint displacements and tolerances must use
consistent units (standard FAB/URDF setups normally use metres and radians).

## Run and retain the report

```text
compas-forge trajectory job.json --clearance 0.005 --articulated-tolerance 0.001 --serial
```

The command writes a JSON report to stdout: input SHA-256, package versions,
SHA-256 of the installed native extension and adapter source files,
clearance policy, attributed pair, bounded entities and clear/violation/unknown.
Redirect stdout to a file if desired. Exit codes are:

| Code | Meaning | Action |
| --- | --- | --- |
| 0 | clear under declared contracts/assumptions | retain report; continue independent engineering checks |
| 1 | violation found | inspect the named pair and witness; revise the job |
| 2 | input/runtime error | correct the input or installation; no clearance established |
| 3 | unknown | inspect reason, geometry and computational budget; never treat as clear |

Use `--solid` only when the restricted single-shell contract applies. Robot
collision meshes often consist of multiple shells or triangle soups; unknown is
the correct outcome when those inputs do not satisfy that contract.

`--max-subdivisions` and `--max-evaluations` limit different work: the former
controls joint-path refinement, the latter adaptive distance queries per pair and
subsegment. Increasing either is not a guarantee of resolution or physical safety.

## What attachments mean

Articulated bounds cover robot links, rigidly attached fixed-state tools,
workpieces attached to a link or robot-attached tool, and fixed world bodies/tools.
They include attachment offsets and a fixed transformed robot base. Explicit
tool trajectories use separate collision meshes for each tool link and composed
robot/tool bounds. The fixed ToolModel TCP frame retains COMPAS FAB semantics.
Attachment transforms remain fixed throughout one query.

For geometric attachment/release phases, use the Python API:

```python
report = forge.verify_compas_fab_cell_phases(
    cell,
    [dict(state=before_grasp, trajectory=approach),
     dict(state=after_grasp, trajectory=transport)],
    clearance=0.005, articulated_tolerance=0.001)
```

Each phase can also contain scene_poses/tool_trajectories. All boundary world
poses, joint values and visible geometry sets must be continuous; hidden or
teleported objects are rejected. Times are local to each phase. Grasp stability,
forces and controller synchronization are not checked. This API does not invent
the attachment transform: supply the actual planned states and touch exclusions.

## Repeated paths from Python

```python
import compas_forge as forge
with forge.prepare_compas_fab_cell(cell, state) as prepared:
    for path in candidate_paths:
        report = prepared.verify_clearance(
            path, clearance=0.005, articulated_tolerance=0.001)
        # Only report['status'] == 'clear' passes the declared model checks.
        # Both violation and unknown require attention.
```

Prepare again after changing geometry, attachment, tool configuration or touch
rules. Export/reporting and preflight do not establish calibration, controller
interpolation, dynamics, tracking-error margins or manufacturing tolerances.
See ROBUSTNESS.md for mathematical assumptions and the remaining limitations.
