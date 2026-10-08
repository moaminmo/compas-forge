"""Analytical nonconvex, scale and near-threshold regression corpus."""
import math
from uuid import uuid4
import pytest
from compas.datastructures import Mesh
import compas_forge as forge
from test_clearance import box, pose


@pytest.mark.parametrize('scale', [1e-3, 1.0, 1e3])
@pytest.mark.parametrize('gap', [1e-5, .01, .4])
def test_nonconvex_notch_clearance_with_known_surface_distance(scale, gap):
    xy = [(-3,-3), (3,-3), (3,3), (1,3), (1,-1), (-1,-1), (-1,3), (-3,3)]
    vertices = [[x*scale,y*scale,z*scale] for z in [-1,1] for x,y in xy]
    faces = [list(reversed(range(8))), list(range(8,16))]
    faces += [[i,(i+1)%8,(i+1)%8+8,i+8] for i in range(8)]
    a,b = uuid4().hex,uuid4().hex
    try:
        forge.register_mesh_to_cache(a, Mesh.from_vertices_and_faces(vertices, faces))
        forge.register_mesh_to_cache(b, box(.2*scale,.2*scale,.2*scale))
        # The rightmost small-box face is gap from the notch wall x=1.
        p = pose((.9-gap)*scale, scale)
        q = (a,pose(),pose(),b,p,p)
        for multiplier, expected in [(.75,'clear'), (1.25,'violation')]:
            r = forge.verify_clearance_cached_batch_poses([q],gap*scale*multiplier,
                numerical_margin=1e-10*scale, solid=True)[0]
            assert r['status'] == expected
            if expected == 'violation':
                assert r['witness_distance'] == pytest.approx(gap*scale, abs=1e-10*scale)
    finally:
        forge.unregister_mesh(a)
        forge.unregister_mesh(b)


@pytest.mark.parametrize('angle', [.01, .3, math.pi/2, math.pi])
def test_fast_rotation_interior_collision_is_not_certified_clear(angle):
    a,b = uuid4().hex,uuid4().hex
    try:
        forge.register_mesh_to_cache(a,box(4,.02,.02))
        forge.register_mesh_to_cache(b,box(.02,.02,.02))
        obstacle = pose(1.8*math.cos(angle/2),1.8*math.sin(angle/2))
        result = forge.verify_clearance_cached_batch_poses(
            [(a,pose(),pose(angle=angle),b,obstacle,obstacle)],.01,parallel=False)[0]
        assert result['status'] == 'violation'
    finally:
        forge.unregister_mesh(a)
        forge.unregister_mesh(b)
