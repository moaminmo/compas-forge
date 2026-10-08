#! python 3
"""Minimal in-host Rhino/Grasshopper smoke test for a release wheel."""

import json
import os
import sys


extra_path = os.environ.get("COMPAS_FORGE_SMOKE_PATH")
if extra_path:
    sys.path.insert(0, extra_path)

import compas
from compas.datastructures import Mesh

import compas_forge
import Rhino
from compas_rhino.conversions import mesh_to_compas


mesh = Mesh.from_vertices_and_faces(
    [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    [[0, 1, 2]],
)
# Exercise a real RhinoCommon mesh, including the documented conversion path.
rhino_mesh = Rhino.Geometry.Mesh()
for vertex in [[0,0,0],[1,0,0],[0,1,0],[0,0,0]]:
    rhino_mesh.Vertices.Add(*vertex)
rhino_mesh.Faces.AddFace(3,1,2)
rhino_mesh.Normals.ComputeNormals()
converted = mesh_to_compas(rhino_mesh)
fixed, repair = compas_forge.repair_mesh(converted)
assert repair['welded_count'] == 1
assert fixed.number_of_vertices() == 3

cube = Mesh.from_vertices_and_faces(
    [[-.5,-.5,-.5],[.5,-.5,-.5],[.5,.5,-.5],[-.5,.5,-.5],
     [-.5,-.5,.5],[.5,-.5,.5],[.5,.5,.5],[-.5,.5,.5]],
    [[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]],
)
from uuid import uuid4
mesh_id = 'rhino-smoke:' + uuid4().hex
compas_forge.register_mesh_to_cache(mesh_id,cube)
try:
    start,end,fixed_pose = ([-2,0,0,0,0,0,1],[2,0,0,0,0,0,1],[0,0,0,0,0,0,1])
    collision = compas_forge.check_swept_collision_cached_poses(
        mesh_id,start,end,mesh_id,fixed_pose,fixed_pose)
    assert abs(collision['time_of_impact']-0.25) < 1e-8
    query = (mesh_id,[-2,1.1,0,0,0,0,1],[2,1.1,0,0,0,0,1],
             mesh_id,fixed_pose,fixed_pose)
    clearance = compas_forge.verify_clearance_cached_batch_poses([query],0.2)[0]
    assert clearance['status'] == 'violation'
    offset_results = compas_forge.verify_clearance_cached_batch_poses(
        [(mesh_id,[-0,1.3,0,0,0,0,1],[-0,1.3,0,0,0,0,1],mesh_id,fixed_pose,fixed_pose)]*2,
        .2, clearance_offsets=[0,.2])
    assert [r['status'] for r in offset_results] == ['clear','violation']
    outer_id = mesh_id+':outer'
    vertices, faces = cube.to_vertices_and_faces()
    outer = Mesh.from_vertices_and_faces([[4*x,4*y,4*z] for x,y,z in vertices],faces)
    compas_forge.register_mesh_to_cache(outer_id,outer)
    try:
        containment = compas_forge.verify_clearance_cached_batch_poses(
            [(mesh_id,fixed_pose,fixed_pose,outer_id,fixed_pose,fixed_pose)],.1,solid=True)[0]
        assert containment['status'] == 'violation'
        assert containment['initial_solid_relationship'] == 'a_inside_b'
    finally:
        compas_forge.unregister_mesh(outer_id)
finally:
    compas_forge.unregister_mesh(mesh_id)
report = compas_forge.analyze_mesh(mesh)
result = {
    "python": sys.version.split()[0],
    "compas": compas.__version__,
    "compas_forge": compas_forge.__version__,
    "is_valid": report["is_valid"],
    "boundary_edges_count": report["boundary_edges_count"],
    "rhino_version": str(Rhino.RhinoApp.Version),
    "rhino_mesh_conversion": True,
    "welded_count": repair['welded_count'],
    "collision_time_of_impact": collision['time_of_impact'],
    "clearance_status": clearance['status'],
    "pair_offsets": [r['status'] for r in offset_results],
    "solid_containment": containment['initial_solid_relationship'],
}

# Check actual Grasshopper data-tree marshaling, without modifying a document.
import clr
gh_assembly = os.environ.get('COMPAS_FORGE_GH_ASSEMBLY')
if gh_assembly:
    sys.path.append(os.path.dirname(os.path.abspath(gh_assembly)))
clr.AddReference('Grasshopper')
from Grasshopper import DataTree
from Grasshopper.Kernel.Data import GH_Path
from System import Object
from compas_ghpython.sets import ghtree_to_list
tree = DataTree[Object]()
tree.Add(rhino_mesh, GH_Path(0))
tree.Add(rhino_mesh.DuplicateMesh(), GH_Path(1))
branches = ghtree_to_list(tree)
converted_branches = [[mesh_to_compas(item) for item in branch] for branch in branches]
assert len(converted_branches) == 2
assert all(branch[0].number_of_faces() == 1 for branch in converted_branches)
result['grasshopper_tree_branches'] = len(converted_branches)

from compas_fab.robots import RobotCellLibrary, JointTrajectory, JointTrajectoryPoint
cell, state = RobotCellLibrary.ur5_gripper_one_beam()
config = state.robot_configuration
path = JointTrajectory([JointTrajectoryPoint(values,config.joint_types,joint_names=config.joint_names)
    for values in [[0,-1.57,1.57,-1.57,-1.57,0],[.1,-1.57,1.57,-1.57,-1.57,0]]],
    joint_names=config.joint_names)
with compas_forge.prepare_compas_fab_cell(cell,state) as prepared:
    trajectory_result = prepared.verify_clearance(path,.001,articulated_tolerance=.001,parallel=False)
assert trajectory_result['status'] == 'clear'
result['moving_robot_clearance'] = trajectory_result['status']
result['moving_robot_pair_queries'] = trajectory_result['evaluated_pair_queries']
end_state = state.copy()
end_state.robot_configuration.joint_values = list(path.points[-1].joint_values)
hold = JointTrajectory([JointTrajectoryPoint(end_state.robot_configuration.joint_values,
    config.joint_types,joint_names=config.joint_names) for _ in range(2)],joint_names=config.joint_names)
phases = compas_forge.verify_compas_fab_cell_phases(cell,
    [dict(state=state,trajectory=path),dict(state=end_state,trajectory=hold)],
    .001,articulated_tolerance=.001,parallel=False)
assert phases['status'] == 'clear' and phases['validated_transitions'] == 1
result['phase_transition_status'] = phases['status']
import hashlib
from pathlib import Path
result['native_sha256'] = hashlib.sha256(Path(compas_forge._core.__file__).read_bytes()).hexdigest()
result['adapter_sha256'] = {name:hashlib.sha256((Path(compas_forge.__file__).parent/name).read_bytes()).hexdigest()
                            for name in ['cell.py','motion_bounds.py','phases.py']}
result['passed'] = True

if not result["is_valid"] or result["boundary_edges_count"] != 3:
    raise AssertionError(result)

payload = json.dumps(result, sort_keys=True)
print("COMPAS_FORGE_RHINO_SMOKE=" + payload)
output_path = os.environ.get("COMPAS_FORGE_SMOKE_RESULT")
if output_path:
    with open(output_path, "w", encoding="utf-8") as output:
        output.write(payload)
