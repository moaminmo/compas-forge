"""Paired same-build broadphase ablation on identical retained UR5 queries."""
import json
import math
import platform
import hashlib
from pathlib import Path
from statistics import median
from time import perf_counter
import compas_forge as forge
import compas_forge.cell as adapter
from compas_fab.robots import RobotCellLibrary,JointTrajectory,JointTrajectoryPoint


def main():
    cell,state = RobotCellLibrary.ur5()
    q = state.robot_configuration
    a = JointTrajectoryPoint(q.joint_values,q.joint_types,joint_names=q.joint_names)
    b = JointTrajectoryPoint([0,-math.pi,math.pi,0,0,0],q.joint_types,joint_names=q.joint_names)
    trajectory = JointTrajectory([a,b],joint_names=q.joint_names)
    original = adapter.check_swept_collision_cached_batch_poses
    queries = []
    def capture(batch,parallel=True):
        queries.extend(batch)
        return original(batch,parallel=parallel)
    samples = {False:[],True:[]}
    with forge.prepare_compas_fab_cell(cell,state) as prepared:
        adapter.check_swept_collision_cached_batch_poses = capture
        try:
            trajectory_result = prepared.sweep(trajectory,parallel=False)
        finally:
            adapter.check_swept_collision_cached_batch_poses = original
        reference = forge.check_swept_collision_cached_batch_native(queries,False,False)
        for repeat in range(8):
            for enabled in ([False,True] if repeat%2 == 0 else [True,False]):
                start = perf_counter()
                results = forge.check_swept_collision_cached_batch_native(queries,False,enabled)
                elapsed = perf_counter()-start
                for expected,result in zip(reference,results):
                    assert result['has_collision'] == expected['has_collision']
                    assert result['time_of_impact'] == expected['time_of_impact']
                    assert result['impact'] == expected['impact']
                if repeat >= 2:
                    samples[enabled].append(elapsed)
    report = dict(platform=platform.platform(),python=platform.python_version(),
        scope='same native build and retained queries; only tight_bounds differs',
        query_count=len(queries),rejections=trajectory_result['broadphase_rejections'],
        baseline_seconds=samples[False],tight_seconds=samples[True],
        median_baseline=median(samples[False]),median_tight=median(samples[True]),
        ratio=median(samples[False])/median(samples[True]),
        source_sha256={p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
                       for p in ['src/bounds.rs','src/lib.rs','benchmark_bounds.py']})
    Path('benchmark-bounds-ablation.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
