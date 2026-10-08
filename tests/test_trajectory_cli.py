import hashlib
import json
import compas
from click.testing import CliRunner
from compas_fab.robots import RobotCellLibrary
from compas_forge.cli import main
from test_motion_bounds import trajectory


def test_real_serialized_cell_is_checked_with_attachment_bounds(tmp_path):
    cell,state = RobotCellLibrary.ur5_gripper_one_beam()
    path = tmp_path/'job.json'
    compas.json_dump(dict(cell=cell,state=state,trajectory=trajectory(state,0)),str(path))
    outcome = CliRunner().invoke(main,['trajectory',str(path),'--clearance','.001',
        '--articulated-tolerance','.001','--serial'])
    assert outcome.exit_code == 1, outcome.output
    report = json.loads(outcome.output)
    assert report['input_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert report['robot_commands_sent'] is False
    assert len(report['implementation_sha256']['native_extension']) == 64
    assert report['result']['status'] == 'violation'
    assert {'tool:gripper','body:beam','body:floor'} <= set(report['result']['bounded_entities'])


def test_trajectory_cli_rejects_incomplete_bundle(tmp_path):
    path = tmp_path/'bad.json'
    path.write_text('{}',encoding='utf8')
    outcome = CliRunner().invoke(main,['trajectory',str(path),'--clearance','.001'])
    assert outcome.exit_code == 2
    assert 'cell must be a serialized RobotCell' in outcome.output


def test_serialized_articulated_tool_path_is_not_ignored(tmp_path):
    from test_tool_paths import tool_scene
    cell,state,tool_path = tool_scene()
    path = tmp_path/'tool-job.json'
    compas.json_dump(dict(cell=cell,state=state,trajectory=trajectory(state,0),
                         tool_trajectories={'gripper':tool_path}),str(path))
    outcome = CliRunner().invoke(main,['trajectory',str(path),'--clearance','.001',
                                      '--articulated-tolerance','.001','--serial'])
    assert outcome.exit_code == 1, outcome.output
    report = json.loads(outcome.output)
    assert report['result']['moving_tool_joint_models'] == ['gripper']
    assert 'tool:gripper/link:finger' in (report['result']['entity_a'],report['result']['entity_b'])


def test_trajectory_cli_does_not_ignore_unmodeled_motion_fields(tmp_path):
    path = tmp_path/'unmodeled.json'
    path.write_text('{"tool_joint_trajectories": {}}',encoding='utf8')
    outcome = CliRunner().invoke(main,['trajectory',str(path),'--clearance','.001'])
    assert outcome.exit_code == 2
    assert 'unsupported bundle fields' in outcome.output


def test_trajectory_cli_unknown_never_exits_success(tmp_path,monkeypatch):
    import compas_forge
    cell,state = RobotCellLibrary.ur5()
    path = tmp_path/'job.json'
    compas.json_dump(dict(cell=cell,state=state,trajectory=trajectory(state,0)),str(path))
    class UnknownPrepared:
        def __enter__(self): return self
        def __exit__(self,*args): pass
        def verify_clearance(self,*args,**kwargs):
            return dict(status='unknown',reason='test_budget_exhaustion')
    monkeypatch.setattr(compas_forge,'prepare_compas_fab_cell',lambda *args: UnknownPrepared())
    outcome = CliRunner().invoke(main,['trajectory',str(path),'--clearance','.001'])
    assert outcome.exit_code == 3
    assert json.loads(outcome.output)['result']['status'] == 'unknown'
