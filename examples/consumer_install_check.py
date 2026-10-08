"""Check an installed wheel and export a real FAB job without development tools.

python examples/consumer_install_check.py --output install-check.json --job-output job.json
compas-forge trajectory job.json --clearance .001 --articulated-tolerance .001 --serial
"""
import argparse
import hashlib
import json
from importlib.metadata import version
from pathlib import Path
import platform

import compas
from compas.geometry import Box
from compas.datastructures import Mesh
from compas_fab.robots import RobotCellLibrary,JointTrajectory,JointTrajectoryPoint
import compas_forge as forge


def run(job_output=None):
    mesh = Mesh.from_vertices_and_faces(*Box(1).to_vertices_and_faces(triangulated=True))
    topology = forge.mesh_topology_predicates(mesh)
    assert topology['is_closed'] and topology['is_manifold']
    cell,state = RobotCellLibrary.ur5_gripper_one_beam()
    state.robot_configuration.joint_values = [0,-1.57,1.57,-1.57,-1.57,0]
    q = state.robot_configuration
    end = list(q.joint_values)
    end[0] += .1
    path = JointTrajectory([JointTrajectoryPoint(values,q.joint_types,joint_names=q.joint_names)
                            for values in [q.joint_values,end]],joint_names=q.joint_names)
    with forge.prepare_compas_fab_cell(cell,state) as prepared:
        result = prepared.verify_clearance(path,.001,articulated_tolerance=.001,parallel=False)
    assert result['status']=='clear', result
    if job_output:
        compas.json_dump(dict(cell=cell,state=state,trajectory=path),str(job_output))
    return dict(passed=True,python=platform.python_version(),
        versions={name:version(name) for name in ['compas-forge','compas','compas-fab','compas-robots']},
        package_path=str(Path(forge.__file__).resolve()),
        native_sha256=hashlib.sha256(Path(forge._core.__file__).read_bytes()).hexdigest(),
        adapter_sha256={name:hashlib.sha256((Path(forge.__file__).parent/name).read_bytes()).hexdigest()
                        for name in ['cell.py','motion_bounds.py','phases.py']},
        topology=topology,trajectory=result,robot_commands_sent=False)


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output')
    parser.add_argument('--job-output')
    args = parser.parse_args()
    report = run(args.job_output)
    payload = json.dumps(report,indent=2,allow_nan=False)
    if args.output:
        Path(args.output).write_text(payload,encoding='utf8')
    print(payload)
