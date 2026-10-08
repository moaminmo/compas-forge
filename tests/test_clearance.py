"""Independent analytical distance oracles and bounded-work failure cases."""

import math
import random
import pytest
from compas.datastructures import Mesh
import compas_forge as forge


def box(x=1.0, y=1.0, z=1.0):
    return Mesh.from_vertices_and_faces(
        [[sx*x/2, sy*y/2, sz*z/2] for sx, sy, sz in
         [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
          (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]],
        [[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]],
    )


def pose(x=0.0, y=0.0, angle=0.0):
    return [x,y,0,0,0,math.sin(angle/2),math.cos(angle/2)]


@pytest.fixture
def retained_boxes():
    from uuid import uuid4
    ids = [uuid4().hex, uuid4().hex]
    for mesh_id in ids:
        forge.register_mesh_to_cache(mesh_id, box())
    try:
        yield ids
    finally:
        for mesh_id in ids:
            forge.unregister_mesh(mesh_id)


def test_clearance_random_linear_near_misses_match_analytical_minimum(retained_boxes):
    a,b = retained_boxes
    rng = random.Random(4810)
    queries, expected = [], []
    # Cubes pass along x. Their exact minimum surface gap is y-1.
    for _ in range(60):
        gap = rng.uniform(0.01, 0.4)
        queries.append((a,pose(-3,1+gap),pose(3,1+gap),b,pose(),pose()))
        expected.append("violation" if gap < 0.2 else "clear")
    serial = forge.verify_clearance_cached_batch_poses(queries, 0.2, parallel=False)
    parallel = forge.verify_clearance_cached_batch_poses(queries, 0.2, parallel=True)
    assert serial == parallel
    assert [r['status'] for r in serial] == expected
    for r in serial:
        assert r['numerical_error_bound_proven'] is False


def test_clearance_budget_and_exact_threshold_never_report_clear(retained_boxes):
    a,b = retained_boxes
    query = (a,pose(-3,1.3),pose(3,1.3),b,pose(),pose())
    r = forge.verify_clearance_cached_batch_poses([query], 0.2, max_evaluations=1,broadphase=False)[0]
    assert r['status'] == 'unknown'
    assert r['reason'] == 'evaluation_budget_exhausted'
    assert r['distance_evaluations'] == 1
    query = (a,pose(0,1.2),pose(0,1.2),b,pose(),pose())
    r = forge.verify_clearance_cached_batch_poses([query], 0.2, max_evaluations=8)[0]
    assert r['status'] == 'unknown'


def test_clearance_common_translation_cancels(retained_boxes):
    a,b = retained_boxes
    query = (a,pose(0,3),pose(100,3),b,pose(),pose(100))
    r = forge.verify_clearance_cached_batch_poses([query], 0.5)[0]
    assert r['status'] == 'clear'
    assert r['relative_speed_bound'] == 0
    assert r['distance_evaluations'] == 0
    assert r['broadphase_rejected']


def test_clearance_detects_rotation_only_interior_violation():
    from uuid import uuid4
    a,b = uuid4().hex,uuid4().hex
    forge.register_mesh_to_cache(a, box(4,0.1,0.1))
    forge.register_mesh_to_cache(b, box(0.1,0.1,0.1))
    try:
        fixed = pose(0,1.9)
        start,end = pose(angle=0),pose(angle=math.pi)
        endpoint_queries = [(a,p,p,b,fixed,fixed) for p in [start,end]]
        assert all(r['status']=='clear' for r in
                   forge.verify_clearance_cached_batch_poses(endpoint_queries,0.05))
        r = forge.verify_clearance_cached_batch_poses(
            [(a,start,end,b,fixed,fixed)],0.05)[0]
        assert r['status']=='violation'
        assert 0 < r['witness_time'] < 1
    finally:
        forge.unregister_mesh(a)
        forge.unregister_mesh(b)


@pytest.mark.parametrize('kwargs', [dict(clearance=-1),dict(clearance=math.nan),
    dict(clearance=0.1,numerical_margin=-1),dict(clearance=0.1,time_tolerance=0),
    dict(clearance=0.1,max_evaluations=0)])
def test_clearance_rejects_invalid_policy_even_for_empty_batch(kwargs):
    with pytest.raises(ValueError):
        forge.verify_clearance_cached_batch_poses([],**kwargs)
