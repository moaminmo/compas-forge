from click.testing import CliRunner
from compas.datastructures import Mesh
from compas_forge.cli import main

def test_cli_help():
    result=CliRunner().invoke(main,['--help'])
    assert result.exit_code==0
    assert 'preflight' in result.output

def test_cli_rejects_invalid_file(tmp_path):
    path=tmp_path/'invalid.json';path.write_text('not json')
    result=CliRunner().invoke(main,['check',str(path)])
    assert result.exit_code==2

def test_cli_checks_compas_file(tmp_path):
    mesh=Mesh.from_vertices_and_faces([[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]])
    path=tmp_path/'mesh.json';mesh.to_json(str(path))
    result=CliRunner().invoke(main,['check',str(path)])
    assert result.exit_code==0, result.output
