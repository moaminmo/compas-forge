"""Seeded independent box-distance corpus. Run directly to save raw evidence.

Bullet primitive OBBs versus Forge triangulated surfaces, separated static
poses only. Timings exclude registration; these are not whole-robot speedups.
Peak tracemalloc counts Python allocations, NOT native/Rust/Bullet memory.
"""
import argparse
import collections
import importlib.metadata
import hashlib
import json
import math
import platform
import random
import time
import tracemalloc
from pathlib import Path
from uuid import uuid4

import compas_forge as forge
from test_clearance import box


def run_corpus():
    import pybullet as bullet
    import numpy as np
    from scipy.optimize import lsq_linear
    from scipy.spatial.transform import Rotation
    rng = random.Random(721031)
    rows = []
    client = bullet.connect(bullet.DIRECT)
    tracemalloc.start()
    try:
        for scale in [1e-3, 1.0, 1e3]:
            key = uuid4().hex
            forge.register_mesh_to_cache(key, box(scale, .6*scale, .3*scale))
            shape = bullet.createCollisionShape(bullet.GEOM_BOX,
                halfExtents=[.5*scale, .3*scale, .15*scale], physicsClientId=client)
            a = bullet.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, physicsClientId=client)
            b = bullet.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, physicsClientId=client)
            unit_shape = bullet.createCollisionShape(bullet.GEOM_BOX,
                halfExtents=[.5,.3,.15],physicsClientId=client)
            unit_a = bullet.createMultiBody(0,unit_shape,physicsClientId=client)
            unit_b = bullet.createMultiBody(0,unit_shape,physicsClientId=client)
            for body in (unit_a,unit_b):
                bullet.changeDynamics(body,-1,collisionMargin=1e-9,physicsClientId=client)
            # Bullet's default margin rounds the box corners and can exceed
            # these millimetre-scale half extents. Match a nearly sharp box,
            # retaining a positive scale-relative margin (not a dynamics claim).
            margin = 1e-9*scale
            for body in (a,b):
                bullet.changeDynamics(body,-1,collisionMargin=margin,physicsClientId=client)
            # Calibrate the oracle on an analytically separated axis-aligned case.
            bullet.resetBasePositionAndOrientation(b,[0,2*scale,0],[0,0,0,1],physicsClientId=client)
            calibration = min(c[8] for c in bullet.getClosestPoints(a,b,5*scale,physicsClientId=client))
            assert abs(calibration-1.4*scale) < 1e-7*scale
            try:
                for index in range(40):
                    qa = bullet.getQuaternionFromEuler([rng.uniform(-math.pi, math.pi) for _ in range(3)])
                    qb = bullet.getQuaternionFromEuler([rng.uniform(-math.pi, math.pi) for _ in range(3)])
                    pa = [0, (1.3+rng.random())*scale, 0]
                    pb = [0, 0, 0]
                    bullet.resetBasePositionAndOrientation(a, pa, qa, physicsClientId=client)
                    bullet.resetBasePositionAndOrientation(b, pb, qb, physicsClientId=client)
                    started = time.perf_counter_ns()
                    contacts = bullet.getClosestPoints(a, b, 5*scale, physicsClientId=client)
                    reference_ns = time.perf_counter_ns()-started
                    raw_reference = min(c[8] for c in contacts)
                    # Also normalize the reference geometry: Bullet's absolute
                    # solver tolerances otherwise change the tiny-box answers.
                    bullet.resetBasePositionAndOrientation(unit_a,[v/scale for v in pa],qa,physicsClientId=client)
                    bullet.resetBasePositionAndOrientation(unit_b,[v/scale for v in pb],qb,physicsClientId=client)
                    reference = scale*min(c[8] for c in bullet.getClosestPoints(unit_a,unit_b,5,physicsClientId=client))
                    # Third independent oracle: globally convex bounded LS for
                    # nearest points in two disjoint solid OBBs. Work in unit
                    # scale; variables are local coordinates inside each box.
                    matrix = np.hstack([Rotation.from_quat(qa).as_matrix(),-Rotation.from_quat(qb).as_matrix()])
                    extents = np.array([.5,.3,.15]*2)
                    solution = lsq_linear(matrix,(np.array(pb)-pa)/scale,
                        bounds=(-extents,extents),method='bvls',tol=1e-13)
                    assert solution.success, 'independent box optimization did not converge'
                    qp_distance = float(np.linalg.norm(solution.fun))*scale
                    assert reference > 0, 'corpus contract requires separated solids'
                    # Both sides of the threshold; zero signed penetration is
                    # deliberately not used as a surface-distance reference.
                    threshold = qp_distance*(.75 if index % 2 else 1.25)
                    query = (key, pa+list(qa), pa+list(qa), key, pb+list(qb), pb+list(qb))
                    started = time.perf_counter_ns()
                    result = forge.verify_clearance_cached_batch_poses(
                        [query], threshold, numerical_margin=scale*1e-8, parallel=False)[0]
                    forge_ns = time.perf_counter_ns()-started
                    expected = 'clear' if threshold < qp_distance else 'violation'
                    rows.append(dict(scale=scale, index=index, pose_a=pa+list(qa), pose_b=pb+list(qb),
                        reference_distance=reference, threshold=threshold, expected=expected,
                        raw_bullet_distance=raw_reference, convex_qp_distance=qp_distance,
                        result=result, reference_query_ns=reference_ns, forge_query_ns=forge_ns))
            finally:
                forge.unregister_mesh(key)
                bullet.removeBody(a, physicsClientId=client)
                bullet.removeBody(b, physicsClientId=client)
                bullet.removeBody(unit_a, physicsClientId=client)
                bullet.removeBody(unit_b, physicsClientId=client)
    finally:
        bullet.disconnect(client)
        _, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
    mismatches = [r for r in rows if r['result']['status'] != r['expected']]
    distance_errors = [abs(r['convex_qp_distance']-r['result']['witness_distance'])/r['scale']
                       for r in rows if r['result']['witness_distance'] is not None]
    return dict(schema_version=1, seed=721031, python=platform.python_version(),
        native_sha256=hashlib.sha256(Path(forge._core.__file__).read_bytes()).hexdigest(),
        versions={p: importlib.metadata.version(p) for p in ['compas-forge', 'compas', 'pybullet']},
        scope='separated static oriented boxes; no penetration, containment or continuous-motion proof',
        reference_collision_margin='positive 1e-9 times scale; both raw and unit-normalized Bullet retained',
        reference_formulation='convex bounded least squares plus unit-normalized Bullet; Forge geometry is NOT normalized',
        memory_scope='tracemalloc Python allocations only; excludes native allocations',
        python_peak_bytes=peak, cases=len(rows), mismatches=len(mismatches),
        statuses=dict(collections.Counter(r['result']['status'] for r in rows)),
        maximum_scale_normalized_witness_error=max(distance_errors, default=0), rows=rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    report = run_corpus()
    with open(args.output, 'w', encoding='utf8') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(json.dumps({k: v for k, v in report.items() if k != 'rows'}))
    raise SystemExit(bool(report['mismatches']))
