"""Articulated tool motion must not be replaced by a frozen collision union."""
import pytest
import math
from compas.geometry import Frame
from compas_robots import RobotModel, ToolModel
from compas_robots.model import Joint
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint
import compas_forge as forge
from test_clearance import box
from test_motion_bounds import trajectory
from test_scene_paths import moving_scene


def tool_scene():
    cell,state = moving_scene()
    model = RobotModel('slider')
    root = model.add_link('root')
    finger = model.add_link('finger',collision_meshes=[box(.2,.2,.2)])
    model.add_joint('slide',Joint.PRISMATIC,root,finger,
                    origin=Frame.worldXY(),axis=[1,0,0],limit=(-5,5))
    tool = ToolModel.from_robot_model(model,Frame.worldXY())
    cell.tool_models['gripper'] = tool
    ts = state.tool_states['gripper']
    ts.attached_to_group = None
    ts.frame = Frame([0,0,0],[1,0,0],[0,1,0])
    ts.configuration = tool.zero_configuration()
    ts.configuration.joint_values = [-2]
    path = JointTrajectory([JointTrajectoryPoint([v],[Joint.PRISMATIC],joint_names=['slide'])
                            for v in [-2,2]],joint_names=['slide'])
    return cell,state,path


def test_moving_finger_detected_and_static_snapshot_preserved():
    cell,state,path = tool_scene()
    with forge.prepare_compas_fab_cell(cell,state) as prepared:
        robot_path = trajectory(state,0)
        before = prepared.verify_clearance(robot_path,.001,articulated_tolerance=.001)
        assert before['status'] == 'clear'
        result = prepared.verify_clearance(robot_path,.001,articulated_tolerance=.001,
                                          tool_trajectories={'gripper':path})
        assert result['status'] == 'violation'
        assert 'tool:gripper/link:finger' in (result['entity_a'],result['entity_b'])
        assert 'tool:gripper/link:finger' in result['bounded_entities']
        assert 'tool:gripper' not in result['bounded_entities']
        retained = prepared.mesh_count
        assert prepared.verify_clearance(robot_path,.001,articulated_tolerance=.001)['status'] == 'clear'
        prepared.verify_clearance(robot_path,.001,articulated_tolerance=.001,
                                  tool_trajectories={'gripper':path})
        assert prepared.mesh_count == retained
    assert prepared.mesh_count == 0


def test_tool_motion_requires_an_explicit_error_bound():
    cell,state,path = tool_scene()
    with forge.prepare_compas_fab_cell(cell,state) as prepared:
        with pytest.raises(ValueError,match='articulated_tolerance'):
            prepared.verify_clearance(trajectory(state,0),.001,tool_trajectories={'gripper':path})


@pytest.mark.parametrize('free_base',[False,True])
def test_coupled_rotation_translation_bound_includes_mixed_derivatives(free_base):
    from compas_robots import Configuration
    from compas_forge.motion_bounds import tool_link_chord_coefficient
    tool = RobotModel('tool')
    root = tool.add_link('root')
    tip = tool.add_link('tip')
    tool.add_joint('p',Joint.PRISMATIC,root,tip,origin=Frame.worldXY(),axis=[1,0,0],limit=(0,3))
    robot = RobotModel('robot')
    base = robot.add_link('base')
    flange = robot.add_link('flange')
    robot.add_joint('r',Joint.CONTINUOUS,base,flange,origin=Frame.worldXY(),axis=[0,0,1])
    qa = Configuration([0],[Joint.CONTINUOUS],['r'])
    qb = Configuration([1],[Joint.CONTINUOUS],['r'])
    ua = Configuration([0],[Joint.PRISMATIC],['p'])
    ub = Configuration([2],[Joint.PRISMATIC],['p'])
    record = dict(link=tip,origin_radius=1.)
    coefficient,angle = tool_link_chord_coefficient(tool,record,ua,ub,robot,
        None if free_base else flange,qa,qb,root_angle=1. if free_base else 0.)
    assert angle == pytest.approx(1.)
    assert coefficient == pytest.approx(1.)  # includes 2 * rotation_rate * slide_rate
    for n in [1,2,8,32]:
        for k in range(101):
            u = k/100
            t = u/n
            actual = [(1+2*t)*math.cos(t),(1+2*t)*math.sin(t)]
            surrogate = [math.cos(t)+u*(2/n)*math.cos(1/n),
                         math.sin(t)+u*(2/n)*math.sin(1/n)]
            assert math.dist(actual,surrogate) <= coefficient/n**2+1e-12


def test_tool_path_limits_rejected():
    cell,state,path = tool_scene()
    path.points[-1].joint_values = [6]
    with forge.prepare_compas_fab_cell(cell,state) as prepared:
        with pytest.raises(ValueError):
            prepared.verify_clearance(trajectory(state,0),.001,articulated_tolerance=.001,
                                      tool_trajectories={'gripper':path})


def test_tool_joints_and_free_base_can_move_together():
    from test_clearance import pose
    cell,state,path = tool_scene()
    with forge.prepare_compas_fab_cell(cell,state) as prepared:
        result = prepared.verify_clearance(trajectory(state,0),.001,articulated_tolerance=.001,
            tool_trajectories={'gripper':path},scene_poses={'tool:gripper':[pose(),pose()]})
        assert result['status'] == 'violation'


def test_attached_tool_bound_covers_actual_compas_coupled_fk():
    from compas_fab.robots import RobotCellLibrary
    from compas.geometry import Quaternion,Rotation,Vector
    from compas_forge.cell import _interpolate_scene_pose
    from compas_forge.motion_bounds import tool_link_chord_coefficient
    cell,state,path = tool_scene()
    _,original = RobotCellLibrary.ur5_gripper_one_beam()
    state.tool_states['gripper'].attached_to_group = original.tool_states['gripper'].attached_to_group
    robot_path = trajectory(state,.2)
    qa = state.robot_configuration
    qb = forge._configuration_from_trajectory_point(robot_path.points[-1],robot_path.joint_names)
    with forge.prepare_compas_fab_cell(cell,state) as prepared:
        ua,ub = prepared._validate_tool_paths({'gripper':path},robot_path,.001)['gripper']
        root = next(r for r in prepared._motion_bound_records() if r.get('key')=='tool:gripper')
        record = prepared._dynamic_tool_records['gripper'][0]
        coefficient,_ = tool_link_chord_coefficient(prepared._robot_cell.tool_models['gripper'],record,
            ua,ub,prepared._robot_cell.robot_model,root['link'],qa,qb,
            root['origin_radius']-root['surrogate_radius'])
        key = 'tool:gripper/link:finger'
        def pose_at(t):
            return prepared._pose_map(forge._interpolate_configuration(qa,qb,t),
                {'gripper':forge._interpolate_configuration(ua,ub,t)})[key]
        for n in [1,4,16]:
            a,b = pose_at(0),pose_at(1/n)
            for k in range(21):
                t = k/20
                actual,surrogate = pose_at(t/n),_interpolate_scene_pose(a,b,t)
                ra = Rotation.from_quaternion(Quaternion(actual[6],*actual[3:6]))
                rs = Rotation.from_quaternion(Quaternion(surrogate[6],*surrogate[3:6]))
                for axis in range(3):
                    v = [0.,0.,0.]
                    v[axis] = record['origin_radius']
                    va,vs = Vector(*v).transformed(ra),Vector(*v).transformed(rs)
                    error = math.dist([actual[i]+va[i] for i in range(3)],
                                      [surrogate[i]+vs[i] for i in range(3)])
                    assert error <= coefficient/n**2+1e-10
