"""Optional shared static fixtures against COMPAS CGAL, including containment.

This compares boundary intersection and solid overlap separately. It is not a
continuous-motion reference or an external performance benchmark.
"""
import argparse
import hashlib
import json
from pathlib import Path
from uuid import uuid4

import pytest
pytest.importorskip('compas_cgal')
import compas_cgal
from compas_cgal.intersections import intersection_mesh_mesh
from compas_cgal.booleans import boolean_intersection_mesh_mesh
from compas.geometry import Box,Rotation,Translation
from compas.datastructures import Mesh
import compas_forge as forge


def compare(scale,angle,case):
    a = Box(2*scale).to_vertices_and_faces(triangulated=True)
    size,x = {'separated':(2,3.5),'crossing':(2,1.1),'contained':(.2,0)}[case]
    b = Mesh.from_vertices_and_faces(*Box(size*scale).to_vertices_and_faces(triangulated=True))
    b.transform(Rotation.from_axis_and_angle([1,2,3],angle))
    b.transform(Translation.from_vector([x*scale,0,.13*scale]))
    b = b.to_vertices_and_faces()
    boundary = bool(len(intersection_mesh_mesh(a,b)))
    _,overlap_faces = boolean_intersection_mesh_mesh(a,b)
    solid = bool(len(overlap_faces))
    ids = [uuid4().hex,uuid4().hex]
    pose = [0,0,0,0,0,0,1]
    try:
        for key,data in zip(ids,[a,b]):
            forge.register_mesh_to_cache(key,Mesh.from_vertices_and_faces(*data))
        query = (ids[0],pose,pose,ids[1],pose,pose)
        surface_result = forge.verify_clearance_cached_batch_poses([query],.001*scale,
                                                                  numerical_margin=1e-9*scale)[0]
        solid_result = forge.verify_clearance_cached_batch_poses([query],.001*scale,
                                                    numerical_margin=1e-9*scale,solid=True)[0]
    finally:
        for key in ids:
            forge.unregister_mesh(key)
    return dict(scale=scale,angle=angle,case=case,cgal_boundary_intersection=boundary,
                cgal_solid_overlap=solid,forge_surface=surface_result['status'],
                forge_solid=solid_result['status'],
                agrees=(surface_result['status']==('violation' if boundary else 'clear') and
                        solid_result['status']==('violation' if solid else 'clear')))


@pytest.mark.parametrize('scale',[.001,1.,1000.])
@pytest.mark.parametrize('angle',[0.,.2,.55,.9])
@pytest.mark.parametrize('case',['separated','crossing','contained'])
def test_cgal_static_boundary_and_solid_semantics(scale,angle,case):
    result = compare(scale,angle,case)
    assert result['agrees'], result
    if case=='contained':
        assert result['cgal_boundary_intersection'] is False
        assert result['cgal_solid_overlap'] is True


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    args = parser.parse_args()
    rows = [compare(scale,angle,case) for scale in [.001,1.,1000.] for angle in [0.,.2,.55,.9]
            for case in ['separated','crossing','contained']]
    report = dict(schema_version=1,scope='static triangulated boxes; boundary vs solid semantics',
                  cgal_version=compas_cgal.__version__,cases=len(rows),
                  mismatches=sum(not row['agrees'] for row in rows),
                  native_sha256=hashlib.sha256(Path(forge._core.__file__).read_bytes()).hexdigest(),rows=rows)
    Path(args.output).write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf8')
    print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
