"""The retained state is part of a partial trajectory's input contract."""
import math
import pytest
from compas_fab.robots import RobotCellLibrary,JointTrajectory,JointTrajectoryPoint
import compas_forge as forge


@pytest.mark.parametrize('invalid',[math.nan,math.inf,'outside_limit'])
def test_partial_path_cannot_hide_invalid_retained_joint(invalid):
    cell,state = RobotCellLibrary.ur5()
    config = state.robot_configuration
    index = config.joint_names.index('elbow_joint')
    upper = cell.robot_model.get_joint_by_name('elbow_joint').limit.upper
    config.joint_values[index] = upper+.1 if invalid=='outside_limit' else invalid
    point = JointTrajectoryPoint([0.],[config.joint_types[0]],joint_names=[config.joint_names[0]])
    path = JointTrajectory([point,point],joint_names=[config.joint_names[0]])
    with forge.prepare_compas_fab_cell(cell,state) as prepared:
        with pytest.raises(ValueError,match='retained|configuration'):
            prepared.verify_clearance(path,.001,parallel=False)


@pytest.mark.parametrize('invalid',[True, 1.5, math.nan, math.inf, 0, -1])
def test_subdivision_budget_must_be_a_positive_integer(invalid):
    cell,state = RobotCellLibrary.ur5()
    config = state.robot_configuration
    point = JointTrajectoryPoint(config.joint_values,config.joint_types,joint_names=config.joint_names)
    path = JointTrajectory([point,point],joint_names=config.joint_names)
    with forge.prepare_compas_fab_cell(cell,state) as prepared:
        with pytest.raises(ValueError,match='positive integer'):
            prepared.verify_clearance(path,.001,max_subdivisions=invalid)
    with pytest.raises(ValueError,match='positive integer'):
        forge.sweep_compas_fab_robot_trajectory(cell,path,{},max_subdivisions=invalid)


@pytest.mark.parametrize('invalid',[True, 1.5, math.nan, math.inf, 0, -1])
def test_worker_budget_must_be_a_positive_integer(invalid):
    cell,state = RobotCellLibrary.ur5()
    config = state.robot_configuration
    point = JointTrajectoryPoint(config.joint_values,config.joint_types,joint_names=config.joint_names)
    path = JointTrajectory([point,point],joint_names=config.joint_names)
    with pytest.raises(ValueError,match='positive integer'):
        forge.sweep_compas_fab_robot_trajectory(cell,path,{},workers=invalid)
