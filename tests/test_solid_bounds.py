"""Analytical containment cases and tight rotational broadphase regressions."""
import math
from uuid import uuid4
import pytest
from compas.datastructures import Mesh
import compas_forge as forge
from test_clearance import box, pose


def query(mesh_a, mesh_b, pa=None, pb=None, **options):
    a, b = uuid4().hex, uuid4().hex
    try:
        forge.register_mesh_to_cache(a, mesh_a)
        forge.register_mesh_to_cache(b, mesh_b)
        pa, pb = pa or pose(), pb or pose()
        return forge.verify_clearance_cached_batch_poses(
            [(a, pa, pa, b, pb, pb)], .05, **options)[0]
    finally:
        forge.unregister_mesh(a)
        forge.unregister_mesh(b)


@pytest.mark.parametrize('reverse', [False, True])
@pytest.mark.parametrize('swap', [False, True])
def test_nested_boxes_report_solid_violation_even_when_surfaces_clear(reverse, swap):
    small, large = box(), box(4,4,4)
    if reverse:
        large.flip_cycles()
    a,b = (large,small) if swap else (small,large)
    p = pose(10,-20,angle=.7)
    assert query(a,b,p,p)['status'] == 'clear'
    result = query(a,b,p,p,solid=True)
    assert result['status'] == 'violation'
    assert result['initial_solid_relationship'] == ('b_inside_a' if swap else 'a_inside_b')
    assert result['witness_time'] == 0
    assert result['witness_distance'] == 0
    assert result['solid_input_certified'] is False


def test_disjoint_valid_solids_are_clear():
    result = query(box(),box(),pose(4,angle=.8),pose(),solid=True)
    assert result['status'] == 'clear'
    assert result['initial_solid_relationship'] == 'disjoint'


def test_open_surface_solid_query_is_unknown_not_clear():
    mesh = box()
    mesh.delete_face(next(mesh.faces()))
    result = query(mesh,box(),pose(4),solid=True)
    assert result['status'] == 'unknown'
    assert result['reason'] == 'solid_requires_closed_manifold'


def test_multi_shell_solid_query_is_unknown():
    v,f = box().to_vertices_and_faces()
    v2 = [[x+3,y,z] for x,y,z in v]
    multi = Mesh.from_vertices_and_faces(v+v2,f+[[i+8 for i in face] for face in f])
    result = query(multi,box(),pose(10),solid=True)
    assert result['status'] == 'unknown'
    assert result['reason'] == 'solid_requires_single_shell'


def combine_shells(*meshes):
    vertices, faces = [], []
    for mesh in meshes:
        v,f = mesh.to_vertices_and_faces()
        faces.extend([[i+len(vertices) for i in face] for face in f])
        vertices.extend(v)
    return Mesh.from_vertices_and_faces(vertices,faces)


@pytest.mark.parametrize('reverse_inner',[False,True])
def test_even_odd_nested_shells_distinguish_cavity_and_material(reverse_inner):
    inner = box(2,2,2)
    if reverse_inner:
        inner.flip_cycles()
    hollow = combine_shells(box(6,6,6),inner)
    # The central cavity is empty regardless of inner-shell orientation.
    r = query(box(.2,.2,.2),hollow,solid=True,solid_rule='even_odd')
    assert r['status'] == 'clear'
    assert r['solid_rule'] == 'even_odd'
    r = query(box(.2,.2,.2),hollow,pose(2),solid=True,solid_rule='even_odd')
    assert r['status'] == 'violation'
    assert r['initial_solid_relationship'] == 'a_inside_b'


def test_even_odd_checks_every_component_not_just_first_vertex():
    from compas.geometry import Translation
    far = box(.2,.2,.2).transformed(Translation.from_vector([10,0,0]))
    multi = combine_shells(far,box(.2,.2,.2))
    assert query(multi,box(4,4,4),solid=True,solid_rule='even_odd')['status'] == 'violation'


def test_even_odd_disjoint_shells_and_invalid_policy():
    from compas.geometry import Translation
    multi = combine_shells(box(),box().transformed(Translation.from_vector([3,0,0])))
    assert query(multi,box(),pose(10),solid=True,solid_rule='even_odd')['status'] == 'clear'
    for kwargs in [dict(solid_rule='union',solid=True),dict(solid_rule='even_odd')]:
        with pytest.raises(ValueError,match='solid_rule'):
            forge.verify_clearance_cached_batch_poses([],.1,**kwargs)


@pytest.mark.parametrize('in_notch', [False, True])
def test_concave_u_prism_containment_distinguishes_empty_notch(in_notch):
    xy = [(-3,-3),(3,-3),(3,3),(1,3),(1,-1),(-1,-1),(-1,3),(-3,3)]
    vertices = [[x,y,z] for z in [-1,1] for x,y in xy]
    faces = [list(reversed(range(8))), list(range(8,16))]
    faces += [[i,(i+1)%8,(i+1)%8+8,i+8] for i in range(8)]
    u = Mesh.from_vertices_and_faces(vertices,faces)
    result = query(box(.2,.2,.2),u,pose(0 if in_notch else 2,1),solid=True)
    assert result['status'] == ('clear' if in_notch else 'violation')
    assert result['initial_solid_relationship'] == ('disjoint' if in_notch else 'a_inside_b')


def test_isolated_vertex_cannot_be_used_as_containment_representative():
    v,f = box().to_vertices_and_faces()
    mesh = Mesh.from_vertices_and_faces([[0,0,0]]+v,[[i+1 for i in face] for face in f])
    result = query(mesh,box(4,4,4),solid=True)
    assert result['status'] == 'unknown'
    assert result['reason'] == 'solid_isolated_vertices'


def test_endpoint_bound_rejects_long_thin_rotating_separated_bars():
    a,b = uuid4().hex,uuid4().hex
    try:
        for name in [a,b]:
            forge.register_mesh_to_cache(name,box(10,.1,.1))
        r = forge.check_swept_collision_cached_poses(
            a,pose(),pose(angle=.01),b,pose(0,2),pose(0,2,angle=.01))
        assert r['has_collision'] is False
        assert r['method'] == 'swept_endpoint_aabb_rejected'
        # Endpoints alone would miss this rotation-only collision.
        r = forge.check_swept_collision_cached_poses(
            a,pose(),pose(angle=math.pi),b,pose(0,3),pose(0,3))
        assert r['has_collision'] is True
    finally:
        forge.unregister_mesh(a)
        forge.unregister_mesh(b)
