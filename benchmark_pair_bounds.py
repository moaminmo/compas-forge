"""Same-wheel ablation: per-entity pair bounds versus global maximum bounds.

Both modes use identical FK, native solver and local-refinement settings.
This measures the bound policy, not Rust versus Python or competing planners.
"""
import argparse
import hashlib
import json
import statistics
import time
from pathlib import Path

import compas_forge as forge
import compas_forge.motion_bounds as bounds
from compas_fab.robots import RobotCellLibrary, JointTrajectory, JointTrajectoryPoint


def run(repeats):
    cell, state = RobotCellLibrary.ur5_gripper_one_beam()
    config = state.robot_configuration
    path = JointTrajectory([JointTrajectoryPoint(values, config.joint_types, joint_names=config.joint_names)
        for values in [[0,-1.57,1.57,-1.57,-1.57,0],[.1,-1.57,1.57,-1.57,-1.57,0]]],
        joint_names=config.joint_names)
    original = bounds.link_chord_coefficients
    def global_maximum(*args):
        result = original(*args)
        largest = (max(v[0] for v in result.values()), max(v[1] for v in result.values()))
        return {key: largest for key in result}
    rows = []
    try:
        with forge.prepare_compas_fab_cell(cell,state) as prepared:
            for iteration in range(repeats):
                modes = ['pair', 'global'] if iteration % 2 == 0 else ['global', 'pair']
                for mode in modes:
                    bounds.link_chord_coefficients = original if mode == 'pair' else global_maximum
                    started = time.perf_counter()
                    result = prepared.verify_clearance(path,.001,articulated_tolerance=.001,parallel=False)
                    rows.append(dict(iteration=iteration,mode=mode,seconds=time.perf_counter()-started,result=result))
    finally:
        bounds.link_chord_coefficients = original
    medians = {mode:statistics.median(r['seconds'] for r in rows if r['mode']==mode) for mode in ['pair','global']}
    return dict(schema_version=1, scope='one moving UR5/gripper/beam fixture; same-wheel bound-policy ablation',
        native_sha256=hashlib.sha256(Path(forge._core.__file__).read_bytes()).hexdigest(),
        adapter_sha256=hashlib.sha256(Path(forge.__file__).with_name('cell.py').read_bytes()).hexdigest(),
        median_seconds=medians,ratio_global_over_pair=medians['global']/medians['pair'],rows=rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeats',type=int,default=3)
    parser.add_argument('--output',required=True)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('repeats must be positive')
    report = run(args.repeats)
    with open(args.output,'w',encoding='utf8') as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
