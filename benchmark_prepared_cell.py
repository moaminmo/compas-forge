"""End-to-end cell setup versus explicit geometry reuse, with raw timings."""

import argparse
import hashlib
import json
import math
import platform
from pathlib import Path
from statistics import median
from time import perf_counter_ns

import compas
import compas_fab
import compas_forge as forge
from compas_fab.robots import RobotCellLibrary, JointTrajectory, JointTrajectoryPoint


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repetitions', type=int, default=20)
    parser.add_argument('--output', default='benchmark-prepared-cell.json')
    parser.add_argument('--fixture', choices=['stationary_cell','moving_robot'], default='stationary_cell')
    args = parser.parse_args()
    if args.repetitions < 2:
        parser.error('repetitions must be at least two')
    cell,state = (RobotCellLibrary.ur5() if args.fixture == 'moving_robot'
                  else RobotCellLibrary.ur5_gripper_one_beam())
    config = state.robot_configuration
    point = JointTrajectoryPoint(config.joint_values, config.joint_types,
                                 joint_names=config.joint_names)
    end = (JointTrajectoryPoint([0,-math.pi,math.pi,0,0,0],config.joint_types,
                                joint_names=config.joint_names)
           if args.fixture == 'moving_robot' else point)
    trajectory = JointTrajectory([point,end], joint_names=config.joint_names)
    start = perf_counter_ns()
    prepared = forge.prepare_compas_fab_cell(cell,state)
    preparation_ms = (perf_counter_ns()-start)/1e6
    fresh, reused = [],[]
    with prepared:
        expected = prepared.sweep(trajectory,parallel=False)
        mesh_count = prepared.mesh_count
        assert expected['has_collision']
        expected_pair = ({'link:upper_arm_link','link:wrist_3_link'}
                         if args.fixture == 'moving_robot' else {'body:beam','body:floor'})
        assert {expected['entity_a'],expected['entity_b']} == expected_pair
        for _ in range(3):
            assert prepared.sweep(trajectory,parallel=False) == expected
        def measure_new():
            t = perf_counter_ns()
            result = forge.sweep_compas_fab_cell_trajectory(cell,state,trajectory,parallel=False)
            elapsed = (perf_counter_ns()-t)/1e6
            assert result == expected
            fresh.append(elapsed)
        def measure_reused():
            t = perf_counter_ns()
            result = prepared.sweep(trajectory,parallel=False)
            elapsed = (perf_counter_ns()-t)/1e6
            assert result == expected
            reused.append(elapsed)
        for index in range(args.repetitions):
            order = [measure_new,measure_reused] if index%2 == 0 else [measure_reused,measure_new]
            for function in order:
                function()
    root = Path(__file__).parent
    report = {
        'fixture':args.fixture,
        'scope':'geometry reuse versus fresh snapshot setup; not Rust versus another language',
        'platform':platform.platform(), 'python':platform.python_version(),
        'compas':compas.__version__, 'compas_fab':compas_fab.__version__,
        'forge':forge.__version__, 'repetitions':args.repetitions,
        'mesh_count':mesh_count, 'preparation_ms':preparation_ms,
        'fresh_total_ms':fresh, 'reused_query_ms':reused,
        'median_fresh_ms':median(fresh), 'median_reused_ms':median(reused),
        'median_ratio':median(fresh)/median(reused),
        'source_sha256':{str(p):hashlib.sha256((root/p).read_bytes()).hexdigest()
                         for p in ['python/compas_forge/cell.py','src/lib.rs','src/clearance.rs']},
    }
    Path(args.output).write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in
                     ['fresh_total_ms','reused_query_ms','source_sha256']},indent=2))


if __name__ == '__main__':
    main()
