"""Independent optimization oracle, including a nonconvex union fixture.

This formulation is specific to disjoint solid boxes; it is not a general
triangle-distance or continuous-collision algorithm and is not shipped as a
Forge runtime dependency beyond COMPAS's existing SciPy dependency.
"""
import math
from uuid import uuid4
import numpy as np
import pytest
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
from compas.datastructures import Mesh
import compas_forge as forge
from test_clearance import box, pose


def box_distance(center_a, rotation_a, extents_a, center_b, rotation_b, extents_b):
    matrix = np.hstack([rotation_a,-rotation_b])
    extents = np.array(list(extents_a)+list(extents_b))
    solution = lsq_linear(matrix,np.array(center_b)-center_a,
        bounds=(-extents,extents),method='bvls',tol=1e-13)
    assert solution.success
    return float(np.linalg.norm(solution.fun))


@pytest.mark.parametrize('scale',[1e-3,1,1e3])
def test_nonconvex_u_surface_matches_independent_union_of_three_boxes(scale):
    xy = [(-3,-3),(3,-3),(3,3),(1,3),(1,-1),(-1,-1),(-1,3),(-3,3)]
    vertices = [[x*scale,y*scale,z*scale] for z in [-1,1] for x,y in xy]
    faces = [list(reversed(range(8))),list(range(8,16))]
    faces += [[i,(i+1)%8,(i+1)%8+8,i+8] for i in range(8)]
    a,b = uuid4().hex,uuid4().hex
    try:
        forge.register_mesh_to_cache(a,Mesh.from_vertices_and_faces(vertices,faces))
        forge.register_mesh_to_cache(b,box(.2*scale,.4*scale,.3*scale))
        for angle in np.linspace(0,math.pi,13):
            rotation = Rotation.from_rotvec([angle*.3,angle*.7,angle]).as_matrix()
            quaternion = Rotation.from_matrix(rotation).as_quat().tolist()
            center = [.3,1,0]
            reference = min(box_distance(center,rotation,[.1,.2,.15],c,np.eye(3),e)
                for c,e in [([-2,0,0],[1,3,1]),([2,0,0],[1,3,1]),([0,-2,0],[1,1,1])])*scale
            p = [v*scale for v in center]+quaternion
            query = (a,pose(),pose(),b,p,p)
            result = forge.verify_clearance_cached_batch_poses([query],reference*1.1,
                numerical_margin=scale*1e-10,solid=True)[0]
            assert result['status'] == 'violation'
            assert result['witness_distance'] == pytest.approx(reference,abs=scale*1e-10)
    finally:
        forge.unregister_mesh(a)
        forge.unregister_mesh(b)
