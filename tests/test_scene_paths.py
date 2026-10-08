"""Synchronized moving-world geometry, independently of joint motion."""
import math
import pytest
from compas.geometry import Frame
from compas_fab.robots import RobotCellLibrary, RigidBody
import compas_forge as forge
from test_clearance import box, pose
from test_motion_bounds import trajectory


def moving_scene():
    cell,state = RobotCellLibrary.ur5_gripper_one_beam()
    state.robot_configuration.joint_values = [0,-1.57,1.57,-1.57,-1.57,0]
    state.robot_base_frame = Frame([100,0,0],[1,0,0],[0,1,0])
    for name,size,x in [('beam',.2,0),('floor',1,3)]:
        cell.rigid_body_models[name] = RigidBody([],box(size,size,size),name=name)
        body = state.rigid_body_states[name]
        body.attached_to_link = None
        body.attached_to_tool = None
        body.touch_bodies = []
        body.touch_links = []
        body.frame = Frame([x,0,0],[1,0,0],[0,1,0])
    return cell,state


def test_world_world_pair_is_added_when_obstacle_moves_between_clear_endpoints():
    cell,state = moving_scene()
    with forge.prepare_compas_fab_cell(cell,state) as prepared:
        path = trajectory(state,0)
        assert prepared.verify_clearance(path,.001,articulated_tolerance=.001)['status'] == 'clear'
        result = prepared.verify_clearance(path,.001,articulated_tolerance=.001,
            scene_poses={'body:floor':[pose(-3),pose(3)]})
        assert result['status'] == 'violation'
        assert {result['entity_a'],result['entity_b']} == {'body:floor','body:beam'}
        assert result['pair_counts']['body_body'] == 1
        assert result['moving_scene_entities'] == ['body:floor']
        assert result['maximum_link_deviation_bound'] == 0
        assert 0 < result['segment_result']['witness_time'] < 1


@pytest.mark.parametrize('scene,match',[
    ({'body:missing':[pose(),pose()]},'visible'),
    ({'body:floor':[pose()]},'one pose'),
    ({'body:floor':[pose(),[0,0,0,0,0,0,0]]},'nonzero'),
    ({'body:floor':[pose(),pose(math.nan)]},'finite'),
])
def test_scene_path_validation_fails_closed(scene,match):
    cell,state = moving_scene()
    with forge.prepare_compas_fab_cell(cell,state) as prepared:
        with pytest.raises(ValueError,match=match):
            prepared.verify_clearance(trajectory(state,0),.01,scene_poses=scene)


def test_scene_path_cannot_silently_override_robot_attachment():
    cell,state = RobotCellLibrary.ur5_gripper_one_beam()
    with forge.prepare_compas_fab_cell(cell,state) as prepared:
        with pytest.raises(ValueError,match='attached entity'):
            prepared.verify_clearance(trajectory(state,0),.01,
                scene_poses={'body:beam':[pose(),pose(10)]})
