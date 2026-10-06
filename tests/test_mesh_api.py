from array import array
import json
import pytest
from compas.datastructures import Mesh
from compas.geometry import Frame
import compas_forge as forge

def test_unknown_fabrication_profile_is_rejected(tmp_path):
    mesh=Mesh.from_vertices_and_faces([[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]])
    path=tmp_path/'mesh.json'; mesh.to_json(str(path))
    with pytest.raises(ValueError,match='Unknown fabrication profile'):
        forge.run_preflight_profile_zero_copy(mesh,'misspelled-profile')
    with pytest.raises(ValueError,match='Unknown fabrication profile'):
        forge.run_preflight_profile(str(path),'misspelled-profile')

def test_timber_mass_limit_is_explicit_and_applied(tmp_path):
    mesh=Mesh.from_vertices_and_faces(
        [[0,0,0],[3,0,0],[3,.4,0],[0,.4,0],[0,0,.4],[3,0,.4],[3,.4,.4],[0,.4,.4]],
        [[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]])
    path=tmp_path/'mesh.json'; mesh.to_json(str(path))
    for result in [forge.run_preflight_profile_zero_copy(mesh,'kuka-timber'), forge.run_preflight_profile(str(path),'kuka-timber')]:
        assert result['estimated_mass_kg']==pytest.approx(240)
        assert result['max_mass_kg']==150
        assert not result['mass_within_limit'] and not result['is_compliant']

def triangle():
    return array('d', [0,0,0, 1,0,0, 0,1,0]), array('i', [0,1,2]), array('i', [0,3])


def cube_mesh(size=1.0):
    h = size / 2.0
    vertices = [
        [-h, -h, -h], [h, -h, -h], [h, h, -h], [-h, h, -h],
        [-h, -h, h], [h, -h, h], [h, h, h], [-h, h, h],
    ]
    faces = [
        [0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4],
        [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7],
    ]
    return Mesh.from_vertices_and_faces(vertices, faces)

def test_nonconsecutive_vertex_keys():
    mesh = Mesh()
    for key, point in zip([10,30,50], [(0,0,0),(1,0,0),(0,1,0)]):
        mesh.add_vertex(key=key, x=point[0], y=point[1], z=point[2])
    mesh.add_face([10,30,50])
    v, i, o = forge.compas_mesh_to_buffers(mesh)
    assert list(i) == [0,1,2]
    assert json.loads(forge.validate_mesh_buffers(v,i,o))['is_valid']


def test_compas_frame_pose_adapter_uses_xyzw_quaternion_order():
    pose = forge.pose_from_frame(Frame.worldXY())
    assert pose == pytest.approx([0, 0, 0, 0, 0, 0, 1])


def test_frame_based_sweep_matches_array_based_sweep():
    mesh = cube_mesh()
    moving_start = Frame.worldXY()
    moving_start.point.x = -2.0
    moving_end = Frame.worldXY()
    moving_end.point.x = 2.0
    fixed = Frame.worldXY()
    frame_result = forge.sweep_collision(
        mesh, moving_start, moving_end, mesh, fixed, fixed
    )
    array_result = forge.check_swept_collision_zero_copy(
        mesh,
        forge.pose_from_frame(moving_start),
        forge.pose_from_frame(moving_end),
        mesh,
        forge.pose_from_frame(fixed),
        forge.pose_from_frame(fixed),
    )
    assert frame_result['has_collision'] == array_result['has_collision']
    assert frame_result['time_of_impact'] == pytest.approx(array_result['time_of_impact'])


def test_linear_sweep_preserves_world_velocity_for_rotated_frames():
    import math

    mesh = cube_mesh()
    start = Frame.from_euler_angles([0, 0, math.pi / 2], point=[-2, 0, 0])
    end = Frame.from_euler_angles([0, 0, math.pi / 2], point=[2, 0, 0])
    fixed = Frame.from_euler_angles([0, 0, math.pi / 2], point=[0, 0, 0])
    result = forge.sweep_collision(mesh, start, end, mesh, fixed, fixed)

    assert result['has_collision']
    assert result['method'] == 'linear'
    assert result['time_of_impact'] == pytest.approx(0.25, abs=1e-5)

@pytest.mark.parametrize('offsets', [[0,4], [0,-1], [1,3], [0,3,2], [], [0,2,3]])
def test_bad_offsets_raise_value_error(offsets):
    v,i,_ = triangle()
    with pytest.raises(ValueError): forge.validate_mesh_buffers(v,i,array('i',offsets))

@pytest.mark.parametrize('indices', [[-1,1,2], [0,1,99]])
def test_bad_indices(indices):
    v,_,o=triangle()
    with pytest.raises(ValueError): forge.validate_mesh_buffers(v,array('i',indices),o)

@pytest.mark.parametrize('values', [[0,1], [0,0,float('nan')]])
def test_bad_coordinates(values):
    _,i,o=triangle()
    with pytest.raises(ValueError): forge.validate_mesh_buffers(array('d',values),i,o)

def test_missing_assembly_field():
    with pytest.raises(ValueError): forge.compute_assembly_contacts([{}], 0.001)

def test_json_nonconsecutive_keys():
    payload={'dtype':'compas.datastructures/Mesh','data':{'vertex':{'10':{'x':0,'y':0,'z':0},'30':{'x':1,'y':0,'z':0},'50':{'x':0,'y':1,'z':0}},'face':{'0':[10,30,50]}}}
    report=json.loads(forge.validate_compas_json(json.dumps(payload)))
    assert report['vertex_count']==3 and report['is_valid']

def test_invalid_json_mesh_rejected():
    payload={'dtype':'compas.datastructures/Mesh','data':{'vertices':[[0,0]],'faces':[[0,1,2]]}}
    with pytest.raises(ValueError): forge.validate_compas_json(json.dumps(payload))

def test_clash_parser_does_not_silently_drop_invalid_parts():
    with pytest.raises(ValueError): forge.detect_clashes_json([('broken','not json')], 0.01)

def test_open_mesh_is_not_closed():
    from compas_forge.plugin import is_mesh_closed
    mesh=Mesh.from_vertices_and_faces([[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]])
    assert not is_mesh_closed(mesh)
    assert forge.verify_mesh_zero_copy(mesh)['boundary_edges_count']==3

def test_tetrahedron_is_closed():
    from compas_forge.plugin import is_mesh_closed
    mesh=Mesh.from_vertices_and_faces([[0,0,0],[1,0,0],[0,1,0],[0,0,1]],[[0,2,1],[0,1,3],[1,2,3],[2,0,3]])
    assert is_mesh_closed(mesh)


def test_vertex_with_disconnected_fans_is_nonmanifold():
    from compas_forge.plugin import is_mesh_manifold
    mesh=Mesh.from_vertices_and_faces([[0,0,0],[1,0,0],[0,1,0],[-1,0,0],[0,-1,0]],[[0,1,2],[0,3,4]])
    report=forge.verify_mesh_zero_copy(mesh)
    assert report['non_manifold_vertices']==[0]
    assert not is_mesh_manifold(mesh)

def test_concave_face_triangulation_preserves_area():
    vertices=[[0,0,0],[3,0,0],[3,3,0],[2,3,0],[2,1,0],[1,1,0],[1,3,0],[0,3,0]]
    mesh=Mesh.from_vertices_and_faces(vertices,[list(range(8))])
    report=forge.run_preflight_profile_zero_copy(mesh, "default")
    area=0
    for face in report['triangulated_faces']:
        a,b,c=[vertices[i] for i in face]
        area+=abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/2
    assert area==pytest.approx(7)

def test_linear_sweep_has_analytical_time_of_impact():
    mesh=Mesh.from_vertices_and_faces([[0,-1,-1],[0,1,-1],[0,0,1]],[[0,1,2]])
    start=[-2,0,0,0,0,0,1]; end=[2,0,0,0,0,0,1]; fixed=[0,0,0,0,0,0,1]
    result=forge.check_swept_collision_zero_copy(mesh,start,end,mesh,fixed,fixed)
    assert result['has_collision']
    assert result['time_of_impact']==pytest.approx(.5,abs=1e-5)
    assert result['method'] == 'linear'
    assert result['substeps'] == 1
    assert result['impact']['status'] == 'Converged'
    assert result['impact']['converged']
    assert not result['impact']['conservative']
    assert result['impact']['geometry_reliable']
    assert result['impact']['witness_a_world'] == pytest.approx(
        result['impact']['witness_b_world'], abs=1e-5
    )


def test_sweep_without_collision_has_no_synthetic_impact_data():
    mesh = cube_mesh()
    start = [-3, 0, 0, 0, 0, 0, 1]
    end = [-2, 0, 0, 0, 0, 0, 1]
    fixed = [3, 0, 0, 0, 0, 0, 1]
    result = forge.check_swept_collision_zero_copy(mesh, start, end, mesh, fixed, fixed)
    assert not result['has_collision']
    assert result['time_of_impact'] is None
    assert result['impact'] is None


def test_sweep_rejects_non_unit_quaternion():
    mesh = cube_mesh()
    invalid_pose = [0, 0, 0, 0, 0, 0, 2]
    identity = [0, 0, 0, 0, 0, 0, 1]
    with pytest.raises(ValueError, match='unit quaternion'):
        forge.check_swept_collision_zero_copy(
            mesh, invalid_pose, identity, mesh, identity, identity
        )


def test_initial_overlap_marks_impact_geometry_unreliable():
    mesh = cube_mesh()
    identity = [0, 0, 0, 0, 0, 0, 1]
    result = forge.check_swept_collision_zero_copy(
        mesh, identity, identity, mesh, identity, identity
    )
    assert result['has_collision']
    assert result['time_of_impact'] == pytest.approx(0.0)
    assert result['method'] == 'initial_overlap'
    assert result['impact']['status'] == 'PenetratingOrWithinTargetDist'
    assert not result['impact']['geometry_reliable']


def test_rotation_only_sweep_detects_intermediate_contact():
    import math
    bar=Mesh.from_vertices_and_faces([[0,-.05,-.05],[2,-.05,-.05],[2,.05,-.05],[0,.05,-.05],[0,-.05,.05],[2,-.05,.05],[2,.05,.05],[0,.05,.05]],[[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]])
    obstacle=Mesh.from_vertices_and_faces([[1,1,-.2],[1.2,1,-.2],[1.1,1,.2]],[[0,1,2]])
    start=[0,0,0,0,0,0,1]; end=[0,0,0,0,0,math.sin(math.pi/4),math.cos(math.pi/4)]
    assert not forge.check_swept_collision_zero_copy(bar,start,start,obstacle,start,start)['has_collision']
    assert not forge.check_swept_collision_zero_copy(bar,end,end,obstacle,start,start)['has_collision']
    result=forge.check_swept_collision_zero_copy(bar,start,end,obstacle,start,start)
    assert result['has_collision'] and 0<result['time_of_impact']<1
    assert result['method'] == 'nonlinear_substepped'
    assert result['substeps'] > 1
    assert result['impact']['witness_a_world'] == pytest.approx(
        result['impact']['witness_b_world'], abs=1e-5
    )


def test_assembly_contact_order_is_deterministic():
    def translated_cube(name, x):
        mesh = cube_mesh()
        mesh.transform(Frame([x, 0, 0], [1, 0, 0], [0, 1, 0]).to_transformation())
        return name, mesh

    assembly = dict([
        translated_cube("block_2", 2.0),
        translated_cube("block_0", 0.0),
        translated_cube("block_1", 1.0),
    ])
    first = forge.assembly_contacts(assembly, tolerance=1e-6)
    second = forge.assembly_contacts(assembly, tolerance=1e-6)

    assert first == second
    assert [(item["block_a"], item["block_b"]) for item in first] == [
        ("block_0", "block_1"),
        ("block_1", "block_2"),
    ]
