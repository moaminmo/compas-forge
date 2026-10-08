"""Regression tests for pair-specific allowances and adjacent foldovers."""
import math
import random
from uuid import uuid4

import pytest
from compas.datastructures import Mesh
import compas_forge as forge
from test_clearance import box, pose


@pytest.mark.parametrize('parallel', [False, True])
def test_native_pair_offsets_preserve_order_and_report_effective_threshold(parallel):
    key = uuid4().hex
    forge.register_mesh_to_cache(key, box())
    try:
        q = (key, pose(0, 1.3), pose(0, 1.3), key, pose(), pose())
        results = forge.verify_clearance_cached_batch_poses(
            [q, q, q], .2, clearance_offsets=[0, .2, .01], parallel=parallel)
        assert [r['status'] for r in results] == ['clear', 'violation', 'clear']
        assert [r['clearance'] for r in results] == pytest.approx([.2, .4, .21])
        assert [r['clearance_offset'] for r in results] == [0, .2, .01]
        for offsets in [[], [-1], [math.nan], [math.inf], [1e308]]:
            with pytest.raises(ValueError, match='clearance_offsets'):
                forge.verify_clearance_cached_batch_poses([q], 1e308, clearance_offsets=offsets)
    finally:
        forge.unregister_mesh(key)


@pytest.mark.parametrize('scale', [1e-3, 1, 1e3])
@pytest.mark.parametrize('folded', [False, True])
def test_shared_edge_foldover_is_not_ignored(scale, folded):
    vertices = [[0, 0, 0], [2, 0, 0], [0, 2, 0], [1, 1 if folded else -1, 0]]
    mesh = Mesh.from_vertices_and_faces(
        [[scale*x, scale*y, scale*z] for x, y, z in vertices], [[0, 1, 2], [1, 0, 3]])
    result = forge.preflight_mesh(mesh)
    assert len(result['self_intersections']) == int(folded)


def test_pair_motion_allowances_are_not_global_maximum(monkeypatch):
    from compas_fab.robots import RobotCellLibrary
    import compas_forge.cell as adapter
    from test_motion_bounds import trajectory
    cell, state = RobotCellLibrary.ur5_gripper_one_beam()
    observed = []

    def oracle(queries, **kwargs):
        if queries:
            observed.append(kwargs['clearance_offsets'])
        return [dict(status='clear', distance_evaluations=1) for _ in queries]

    with forge.prepare_compas_fab_cell(cell, state) as prepared:
        monkeypatch.setattr(adapter, 'verify_clearance_cached_batch_poses', oracle)
        result = prepared.verify_clearance(trajectory(state, .01), .001, articulated_tolerance=.002)
    assert result['status'] == 'clear'
    assert observed
    offsets = observed[0]
    assert max(offsets) <= 2*result['maximum_link_deviation_bound']
    assert min(offsets) < max(offsets)
    assert result['motion_allowance_policy'] == 'sum_of_pair_entity_bounds'


@pytest.mark.parametrize('scale',[1e-3,1,1e3])
@pytest.mark.parametrize('tips,overlap',[
    ([[1,1,-1],[1,1,1]],True),
    ([[-1,0,1],[0,-1,1]],False),
    ([[1,.2,0],[.2,1,0]],True),
    ([[-1,0,0],[0,-1,0]],False),
])
def test_shared_vertex_intersection_beyond_topological_contact(scale,tips,overlap):
    vertices = [[0,0,0],[2,0,0],[0,2,0]]+tips
    mesh = Mesh.from_vertices_and_faces([[scale*x,scale*y,scale*z] for x,y,z in vertices],
                                        [[0,1,2],[0,3,4]])
    assert bool(forge.preflight_mesh(mesh)['self_intersections']) is overlap


@pytest.mark.parametrize('depth,expected', [(0, 'unknown'), (1, 'clear')])
def test_local_refinement_reduces_allowance_quadratically(monkeypatch, depth, expected):
    from compas_robots import Configuration
    import compas_forge.cell as adapter
    prepared = adapter.PreparedRobotCell.__new__(adapter.PreparedRobotCell)
    prepared._pose_map = lambda q: {'a': pose(q.joint_values[0]), 'b': pose(0, 2)}
    start, end = Configuration([0], [1], ['j']), Configuration([1], [1], ['j'])
    offsets = []
    def oracle(queries, clearance_offsets, **kwargs):
        offsets.extend(clearance_offsets)
        return [dict(status='clear', witness_time=None, distance_evaluations=1)]
    monkeypatch.setattr(adapter, 'verify_clearance_cached_batch_poses', oracle)
    query = ('pair','a','b','b','mesh-a',pose(),pose(1),'mesh-b',pose(0,2),pose(0,2))
    initial = dict(status='unknown', witness_time=None, distance_evaluations=1)
    result, counts = prepared._refine_pair(query,initial,start,end,1.0,dict(clearance=.1),depth)
    assert result['status'] == expected
    assert offsets == ([] if depth == 0 else [.25,.25])
    assert counts['pair_queries'] == 2*depth
    if depth:
        assert counts['fk_evaluations'] == 3  # shared midpoint is cached


def test_clearance_broadphase_agrees_with_unfiltered_queries():
    rng = random.Random(72301)
    key = uuid4().hex
    forge.register_mesh_to_cache(key,box(2,.2,.3))
    try:
        queries = [(key,pose(-2,rng.uniform(-4,4),rng.uniform(-1,1)),
                    pose(2,rng.uniform(-4,4),rng.uniform(-1,1)),key,pose(),pose()) for _ in range(100)]
        filtered = forge.verify_clearance_cached_batch_poses(queries,.1)
        unfiltered = forge.verify_clearance_cached_batch_poses(queries,.1,broadphase=False)
        assert [r['status'] for r in filtered] == [r['status'] for r in unfiltered]
        assert any(r['broadphase_rejected'] for r in filtered)
        assert sum(r['distance_evaluations'] for r in filtered) < sum(r['distance_evaluations'] for r in unfiltered)
    finally:
        forge.unregister_mesh(key)
