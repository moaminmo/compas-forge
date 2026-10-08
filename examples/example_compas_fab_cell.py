"""Headless continuous preflight of a complete COMPAS FAB 2 RobotCell."""

import json

from compas_fab.robots import JointTrajectory, JointTrajectoryPoint, RobotCellLibrary

import compas_forge


def main():
    cell, state = RobotCellLibrary.ur5_gripper_one_beam()
    configuration = state.robot_configuration
    point = JointTrajectoryPoint(
        configuration.joint_values,
        configuration.joint_types,
        joint_names=configuration.joint_names,
    )
    trajectory = JointTrajectory([point, point], joint_names=configuration.joint_names)

    result = compas_forge.sweep_compas_fab_cell_trajectory(
        cell,
        state,
        trajectory,
        parallel=True,
    )
    print("COMPAS FAB 2 full-cell continuous preflight")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
