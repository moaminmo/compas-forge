"""Run the production mesh diagnostics and retain a reproducible result."""
import json
from pathlib import Path
from compas.datastructures import Mesh
import compas_forge as forge

out = Path(__file__).resolve().parents[1] / 'publication'
out.mkdir(exist_ok=True)
vertices = [[0,0,0], [1,0,0], [0,1,0], [0,0,1]]
closed = [[0,2,1],[0,1,3],[1,2,3],[2,0,3]]
results = []
for name, faces in [('open',closed[:-1]),('closed',closed)]:
    report = forge.verify_mesh_zero_copy(Mesh.from_vertices_and_faces(vertices,faces))
    assert report['boundary_edges_count'] == (3 if name == 'open' else 0)
    results.append({'name':name,'vertices':vertices,'faces':faces,'report':report})
(out/'demo.json').write_text(json.dumps({'fixture':'unit tetrahedron with one face omitted / restored','results':results},indent=2),encoding='utf-8')
print('PASS: open tetrahedron has 3 boundary edges; closed tetrahedron has 0.')
