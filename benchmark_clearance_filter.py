"""Same-wheel clearance broadphase ablation on a retained moving FAB cell."""
import argparse
import hashlib
import json
import statistics
import time
from pathlib import Path

import compas_forge as forge
import compas_forge.cell as adapter
from compas_fab.robots import RobotCellLibrary, JointTrajectory, JointTrajectoryPoint


def run(repeats):
    cell,state = RobotCellLibrary.ur5_gripper_one_beam()
    config = state.robot_configuration
    path = JointTrajectory([JointTrajectoryPoint(v,config.joint_types,joint_names=config.joint_names)
        for v in [[0,-1.57,1.57,-1.57,-1.57,0],[.1,-1.57,1.57,-1.57,-1.57,0]]],joint_names=config.joint_names)
    original = adapter.verify_clearance_cached_batch_poses
    rows = []
    try:
        with forge.prepare_compas_fab_cell(cell,state) as prepared:
            for i in range(repeats):
                for enabled in ([True,False] if i%2 == 0 else [False,True]):
                    def invoke(*args,**kwargs):
                        return original(*args,broadphase=enabled,**kwargs)
                    adapter.verify_clearance_cached_batch_poses = invoke
                    started = time.perf_counter()
                    result = prepared.verify_clearance(path,.001,articulated_tolerance=.001,parallel=False)
                    rows.append(dict(iteration=i,broadphase=enabled,seconds=time.perf_counter()-started,result=result))
    finally:
        adapter.verify_clearance_cached_batch_poses = original
    medians = {str(mode):statistics.median(r['seconds'] for r in rows if r['broadphase']==mode) for mode in [True,False]}
    return dict(schema_version=1,scope='retained UR5/gripper/beam; clearance broadphase only, not a language comparison',
        native_sha256=hashlib.sha256(Path(forge._core.__file__).read_bytes()).hexdigest(),
        adapter_sha256=hashlib.sha256(Path(adapter.__file__).read_bytes()).hexdigest(),
        median_seconds=medians,ratio_unfiltered_over_filtered=medians['False']/medians['True'],rows=rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeats',type=int,default=7)
    parser.add_argument('--output',required=True)
    args = parser.parse_args()
    if args.repeats < 1: parser.error('repeats must be positive')
    report = run(args.repeats)
    with open(args.output,'w',encoding='utf8') as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
