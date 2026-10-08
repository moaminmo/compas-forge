"""Headless COMPAS FAB + COMPAS Forge trajectory preflight.

The two endpoint checks are clear. The continuous sweep detects the collision
between them, demonstrating the failure mode that endpoint-only checking misses.
"""

import math
import time

from compas.datastructures import Mesh
from compas.geometry import Frame
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint, RobotCellLibrary
from compas_robots import Configuration

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


def run_example():
    robot, _ = RobotCellLibrary.ur5()
    names = robot.robot_model.get_configurable_joint_names()
    start_values = [0.0, -math.pi / 2.0, 0.0, -math.pi / 2.0, 0.0, 0.0]
    end_values = [math.pi / 2.0, -math.pi / 2.0, 0.0, -math.pi / 2.0, 0.0, 0.0]
    start = Configuration.from_revolute_values(start_values, names)
    end = Configuration.from_revolute_values(end_values, names)
    start_frame = robot.robot_model.forward_kinematics(start)
    end_frame = robot.robot_model.forward_kinematics(end)

    tool = cube_mesh(0.04)
    obstacle = cube_mesh(0.08)
    obstacle_frame = Frame.worldXY()
    obstacle_frame.point = (start_frame.point + end_frame.point) * 0.5

    start_clear = not compas_forge.sweep_collision(
        tool, start_frame, start_frame, obstacle, obstacle_frame, obstacle_frame
    )["has_collision"]
    end_clear = not compas_forge.sweep_collision(
        tool, end_frame, end_frame, obstacle, obstacle_frame, obstacle_frame
    )["has_collision"]

    trajectory = JointTrajectory(
        trajectory_points=[
            JointTrajectoryPoint(start_values, start.joint_types, joint_names=names),
            JointTrajectoryPoint(end_values, end.joint_types, joint_names=names),
        ],
        joint_names=names,
    )
    started = time.perf_counter_ns()
    result = compas_forge.sweep_compas_fab_trajectory(
        robot, trajectory, tool, obstacle, obstacle_frame
    )
    elapsed_ms = (time.perf_counter_ns() - started) / 1_000_000.0

    print("COMPAS FAB UR5 trajectory preflight")
    print(f"  Endpoint-only check: start_clear={start_clear}, end_clear={end_clear}")
    print(f"  COMPAS Forge continuous sweep: collision={result['has_collision']}")
    print(f"  Collision trajectory fraction: {result['trajectory_fraction']:.6f}")
    print(f"  Solver method: {result['segment_result']['method']}")
    print(f"  Solver status: {result['segment_result']['impact']['status']}")
    print(f"  Evaluation time: {elapsed_ms:.4f} ms")


if __name__ == "__main__":
    run_example()
