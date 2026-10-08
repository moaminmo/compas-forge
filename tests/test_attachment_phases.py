import pytest
from compas_fab.robots import RobotCellLibrary
from compas_forge.phases import verify_clearance_phases
from test_scene_paths import moving_scene
from test_motion_bounds import trajectory


def test_identical_continuous_phases_are_clear():
    cell,state = moving_scene()
    phase = dict(state=state,trajectory=trajectory(state,0))
    result = verify_clearance_phases(cell,[phase,phase],.001,articulated_tolerance=.001)
    assert result['status'] == 'clear'
    assert result['validated_transitions'] == 1
    assert result['grasp_stability_verified'] is False
    assert result['robot_commands_sent'] is False


def test_release_preserving_world_pose_is_a_valid_transition():
    cell,state = RobotCellLibrary.ur5_gripper_one_beam()
    placed = cell.compute_attach_objects_frames(state)
    placed.rigid_body_states['beam'].attached_to_tool = None
    placed.rigid_body_states['beam'].attached_to_link = None
    result = verify_clearance_phases(cell,[dict(state=state,trajectory=trajectory(state,0)),
        dict(state=placed,trajectory=trajectory(placed,0))],.001,articulated_tolerance=.001)
    assert result['validated_transitions'] == 1
    assert len(result['phases']) == 2


@pytest.mark.parametrize('change',['body','robot','visibility'])
def test_boundary_teleport_or_disappearance_is_rejected(change):
    cell,state = moving_scene()
    changed = state.copy()
    if change == 'body':
        changed.rigid_body_states['beam'].frame.point.x += .1
    elif change == 'robot':
        changed.robot_configuration.joint_values[0] += .1
    else:
        changed.rigid_body_states['beam'].is_hidden = True
    with pytest.raises(ValueError,match='discontinuous|changed'):
        verify_clearance_phases(cell,[dict(state=state,trajectory=trajectory(state,0)),
            dict(state=changed,trajectory=trajectory(changed,0))],.001,articulated_tolerance=.001)
