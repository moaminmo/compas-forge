import math

import compas_fab
import pytest
from compas.datastructures import Mesh
from compas.geometry import Frame
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint, Robot
from compas_robots import Configuration, RobotModel

import compas_forge


def cube_mesh(size):
    h = size / 2.0
    return Mesh.from_vertices_and_faces(
        [
            [-h, -h, -h], [h, -h, -h], [h, h, -h], [-h, h, -h],
            [-h, -h, h], [h, -h, h], [h, h, h], [-h, h, h],
        ],
        [
            [0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4],
            [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7],
        ],
    )


def ur5_robot():
    urdf = compas_fab.get("universal_robot/ur_description/urdf/ur5.urdf")
    return Robot(RobotModel.from_urdf_file(urdf))


def test_compas_fab_joint_trajectory_finds_collision_between_endpoints():
    robot = ur5_robot()
    names = robot.get_configurable_joint_names()
    start_values = [0.0, -math.pi / 2.0, 0.0, -math.pi / 2.0, 0.0, 0.0]
    end_values = [math.pi / 2.0, -math.pi / 2.0, 0.0, -math.pi / 2.0, 0.0, 0.0]
    start = Configuration.from_revolute_values(start_values, names)
    end = Configuration.from_revolute_values(end_values, names)
    start_frame = robot.forward_kinematics(start, options={"solver": "model"})
    end_frame = robot.forward_kinematics(end, options={"solver": "model"})

    obstacle_frame = Frame.worldXY()
    obstacle_frame.point = (start_frame.point + end_frame.point) * 0.5
    tool = cube_mesh(0.04)
    obstacle = cube_mesh(0.08)

    assert not compas_forge.sweep_collision(
        tool, start_frame, start_frame, obstacle, obstacle_frame, obstacle_frame
    )["has_collision"]
    assert not compas_forge.sweep_collision(
        tool, end_frame, end_frame, obstacle, obstacle_frame, obstacle_frame
    )["has_collision"]

    trajectory = JointTrajectory(
        trajectory_points=[
            JointTrajectoryPoint(start_values, start.joint_types, joint_names=names),
            JointTrajectoryPoint(end_values, end.joint_types, joint_names=names),
        ],
        joint_names=names,
    )
    result = compas_forge.sweep_compas_fab_trajectory(
        robot, trajectory, tool, obstacle, obstacle_frame
    )

    assert result["has_collision"]
    assert result["segment_index"] == 0
    assert 0.0 < result["trajectory_fraction"] < 1.0
    assert result["frames_evaluated"] == 2


def test_compas_fab_trajectory_requires_two_points():
    robot = ur5_robot()
    trajectory = JointTrajectory(trajectory_points=[])
    with pytest.raises(ValueError, match="at least two points"):
        compas_forge.sweep_compas_fab_trajectory(
            robot, trajectory, cube_mesh(0.04), cube_mesh(0.08)
        )
