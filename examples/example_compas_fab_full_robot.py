"""Headless UR5 full-link continuous self-collision demonstration."""

import json
import math
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint, RobotCellLibrary
from compas_robots import Configuration

import compas_forge


def load_ur5_with_collision_geometry():
    robot, _ = RobotCellLibrary.ur5()
    return robot


def main():
    robot = load_ur5_with_collision_geometry()
    names = robot.robot_model.get_configurable_joint_names()
    start_values = [0.0] * len(names)
    end_values = [0.0, -math.pi, math.pi, 0.0, 0.0, 0.0]
    configuration = Configuration.from_revolute_values(start_values, names)
    trajectory = JointTrajectory(
        trajectory_points=[
            JointTrajectoryPoint(
                start_values, configuration.joint_types, joint_names=names
            ),
            JointTrajectoryPoint(
                end_values, configuration.joint_types, joint_names=names
            ),
        ],
        joint_names=names,
    )

    result = compas_forge.sweep_compas_fab_robot_trajectory(
        robot,
        trajectory,
        obstacles={},
        check_self_collision=True,
        max_joint_step=math.radians(10.0),
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
