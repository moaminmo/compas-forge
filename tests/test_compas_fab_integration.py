import math
import pytest
from compas.datastructures import Mesh
from compas.geometry import Frame
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint, RobotCellLibrary
from compas_fab.robots.time_ import Duration
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


def ur5_robot():
    robot, _ = RobotCellLibrary.ur5()
    return robot


def ur5_robot_with_collision_geometry():
    robot, _ = RobotCellLibrary.ur5()
    return robot


def test_compas_fab_joint_trajectory_finds_collision_between_endpoints():
    robot = ur5_robot()
    names = robot.robot_model.get_configurable_joint_names()
    start_values = [0.0, -math.pi / 2.0, 0.0, -math.pi / 2.0, 0.0, 0.0]
    end_values = [math.pi / 2.0, -math.pi / 2.0, 0.0, -math.pi / 2.0, 0.0, 0.0]
    start = Configuration.from_revolute_values(start_values, names)
    end = Configuration.from_revolute_values(end_values, names)
    start_frame = robot.robot_model.forward_kinematics(start)
    end_frame = robot.robot_model.forward_kinematics(end)

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
            JointTrajectoryPoint(
                start_values,
                start.joint_types,
                time_from_start=Duration(0, 0),
                joint_names=names,
            ),
            JointTrajectoryPoint(
                end_values,
                end.joint_types,
                time_from_start=Duration(2, 0),
                joint_names=names,
            ),
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
    assert result["trajectory_duration"] == pytest.approx(2.0)
    assert result["collision_time_from_start"] == pytest.approx(
        2.0 * result["segment_fraction"]
    )
    assert result["trajectory_time_fraction"] == pytest.approx(
        result["trajectory_fraction"]
    )


def test_compas_fab_trajectory_requires_two_points():
    robot = ur5_robot()
    trajectory = JointTrajectory(trajectory_points=[])
    with pytest.raises(ValueError, match="at least two points"):
        compas_forge.sweep_compas_fab_trajectory(
            robot, trajectory, cube_mesh(0.04), cube_mesh(0.08)
        )


@pytest.mark.parametrize("defect", ["joint_type_mismatch", "invalid_time_from_start"])
def test_trajectory_rejects_wrong_joint_type_and_negative_time(defect):
    from compas_robots.model import Joint

    robot = ur5_robot()
    config = robot.zero_configuration()
    types = list(config.joint_types)
    if defect == "joint_type_mismatch":
        types[0] = Joint.PRISMATIC
    points = [
        JointTrajectoryPoint(
            config.joint_values, types, joint_names=config.joint_names,
            time_from_start=Duration(i - 2 if defect == "invalid_time_from_start" else i, 0),
        )
        for i in range(2)
    ]
    trajectory = JointTrajectory(points, joint_names=config.joint_names)
    report = compas_forge.validate_compas_fab_trajectory(robot, trajectory)
    assert not report["is_valid"]
    assert defect in {error["code"] for error in report["errors"]}
    with pytest.raises(ValueError, match=defect):
        compas_forge.sweep_compas_fab_robot_trajectory(robot, trajectory, {})


def test_partial_link_registration_is_cleaned_after_failure(monkeypatch):
    robot = ur5_robot()
    config = robot.zero_configuration()
    point = JointTrajectoryPoint(
        config.joint_values, config.joint_types, joint_names=config.joint_names
    )
    trajectory = JointTrajectory([point, point], joint_names=config.joint_names)
    original_register = compas_forge.register_mesh_to_cache
    created = []

    def fail_second_registration(mesh_id, mesh):
        if created:
            raise ValueError("simulated invalid second link")
        result = original_register(mesh_id, mesh)
        created.append(mesh_id)
        return result

    monkeypatch.setattr(compas_forge, "register_mesh_to_cache", fail_second_registration)
    with pytest.raises(ValueError, match="invalid second link"):
        compas_forge.sweep_compas_fab_robot_trajectory(robot, trajectory, {})
    pose = [0, 0, 0, 0, 0, 0, 1]
    assert len(created) == 1
    with pytest.raises(ValueError):
        compas_forge.check_swept_collision_cached_poses(
            created[0], pose, pose, created[0], pose, pose
        )


def test_compas_fab_trajectory_validation_reports_joint_limit():
    robot = ur5_robot()
    configuration = robot.zero_configuration()
    names = list(configuration.joint_names)
    values = list(configuration.joint_values)
    values[names.index("elbow_joint")] = math.pi + 0.1
    trajectory = JointTrajectory(
        [
            JointTrajectoryPoint(values, configuration.joint_types, joint_names=names),
            JointTrajectoryPoint(values, configuration.joint_types, joint_names=names),
        ],
        joint_names=names,
    )

    report = compas_forge.validate_compas_fab_trajectory(robot, trajectory)

    assert not report["is_valid"]
    violation = next(error for error in report["errors"] if error["code"] == "joint_limit")
    assert violation["joint"] == "elbow_joint"
    with pytest.raises(ValueError, match="joint_limit"):
        compas_forge.sweep_compas_fab_robot_trajectory(robot, trajectory, {})


def test_compas_fab_trajectory_validation_reports_timing_velocity():
    robot = ur5_robot()
    configuration = robot.zero_configuration()
    names = list(configuration.joint_names)
    start = list(configuration.joint_values)
    end = list(start)
    end[0] = 1.0
    trajectory = JointTrajectory(
        [
            JointTrajectoryPoint(
                start,
                configuration.joint_types,
                time_from_start=Duration(0, 0),
                joint_names=names,
            ),
            JointTrajectoryPoint(
                end,
                configuration.joint_types,
                time_from_start=Duration(0, 10_000_000),
                joint_names=names,
            ),
        ],
        joint_names=names,
    )

    report = compas_forge.validate_compas_fab_trajectory(robot, trajectory)

    assert report["timing_supplied"]
    assert any(error["code"] == "average_velocity_limit" for error in report["errors"])


def test_full_robot_link_sweep_reports_environment_link_and_residual():
    robot = ur5_robot_with_collision_geometry()
    names = robot.robot_model.get_configurable_joint_names()
    values = [0.0] * len(names)
    configuration = Configuration.from_revolute_values(values, names)
    point = JointTrajectoryPoint(values, configuration.joint_types, joint_names=names)
    trajectory = JointTrajectory(
        trajectory_points=[point, point], joint_names=names
    )

    shoulder = robot.robot_model.get_link_by_name("shoulder_link")
    collision = shoulder.collision[0]
    shoulder_mesh = collision.geometry.shape.meshes[0].copy()
    shoulder_mesh.transform(collision.init_transformation)
    face = next(shoulder_mesh.faces())
    obstacle_frame = Frame.worldXY()
    obstacle_frame.point = shoulder_mesh.face_centroid(face)

    result = compas_forge.sweep_compas_fab_robot_trajectory(
        robot,
        trajectory,
        {"fixture": cube_mesh(0.04)},
        {"fixture": obstacle_frame},
        check_self_collision=False,
    )

    assert result["has_collision"]
    assert result["collision_type"] == "environment"
    assert result["link_a"] is not None
    assert result["obstacle"] == "fixture"
    assert result["link_count"] == 7
    assert result["segment_result"]["impact"]["verification_distance"] == pytest.approx(
        0.0, abs=1e-8
    )


def test_full_robot_link_sweep_adaptively_subdivides_joint_motion():
    robot = ur5_robot_with_collision_geometry()
    names = robot.robot_model.get_configurable_joint_names()
    start_values = [0.0] * len(names)
    end_values = [math.radians(12.0)] + [0.0] * (len(names) - 1)
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
        {},
        check_self_collision=False,
        max_joint_step=math.radians(5.0),
    )
    assert not result["has_collision"]
    assert result["evaluated_subsegments"] == 3
    assert result["fk_evaluations"] == 4


def test_full_robot_link_sweep_applies_robot_base_frame():
    robot = ur5_robot_with_collision_geometry()
    names = robot.robot_model.get_configurable_joint_names()
    values = [0.0] * len(names)
    configuration = Configuration.from_revolute_values(values, names)
    point = JointTrajectoryPoint(values, configuration.joint_types, joint_names=names)
    trajectory = JointTrajectory([point, point], joint_names=names)

    base_frame = Frame.worldXY()
    base_frame.point.x = 100.0
    result = compas_forge.sweep_compas_fab_robot_trajectory(
        robot,
        trajectory,
        {"fixture": cube_mesh(0.5)},
        {"fixture": Frame.worldXY()},
        check_self_collision=False,
        robot_base_frame=base_frame,
    )

    assert not result["has_collision"]
    assert result["robot_base_frame_applied"]


def test_full_robot_link_sweep_detects_intermediate_self_collision():
    robot = ur5_robot_with_collision_geometry()
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
        {},
        check_self_collision=True,
        max_joint_step=math.radians(10.0),
        workers=4,
    )
    assert result["has_collision"]
    assert result["collision_type"] == "self"
    assert {result["link_a"], result["link_b"]} == {
        "upper_arm_link",
        "wrist_3_link",
    }
    assert result["self_pair_count"] == 10
    assert result["semantics_disabled_pair_count"] == 11
    assert result["respect_robot_semantics"]
    assert result["workers"] == 4
    assert 0.0 < result["segment_fraction"] < 1.0
    assert result["segment_result"]["impact"]["geometry_reliable"]


def test_full_robot_cell_sweep_covers_tools_bodies_and_touch_semantics():
    cell, state = RobotCellLibrary.ur5_gripper_one_beam()
    configuration = state.robot_configuration
    point = JointTrajectoryPoint(
        configuration.joint_values,
        configuration.joint_types,
        joint_names=configuration.joint_names,
    )
    trajectory = JointTrajectory([point, point], joint_names=configuration.joint_names)

    result = compas_forge.sweep_compas_fab_cell_trajectory(
        cell, state, trajectory, parallel=True
    )

    assert result["has_collision"]
    assert result["collision_type"] == "body_body"
    assert {result["entity_a"], result["entity_b"]} == {
        "body:beam",
        "body:floor",
    }
    assert result["pair_counts"] == {
        "robot_self": 10,
        "robot_tool": 6,
        "robot_body": 13,
        "body_body": 1,
        "tool_body": 1,
    }
    assert result["execution_mode"] == "native_rayon_batch"
    assert result["broadphase_rejections"] >= 1


def test_full_robot_cell_sweep_skips_hidden_objects():
    cell, state = RobotCellLibrary.ur5_gripper_one_beam()
    state.rigid_body_states["floor"].is_hidden = True
    configuration = state.robot_configuration
    point = JointTrajectoryPoint(
        configuration.joint_values,
        configuration.joint_types,
        joint_names=configuration.joint_names,
    )
    trajectory = JointTrajectory([point, point], joint_names=configuration.joint_names)

    result = compas_forge.sweep_compas_fab_cell_trajectory(cell, state, trajectory)

    assert result["pair_counts"]["robot_body"] == 7
    assert "body_body" not in result["pair_counts"]
    assert "tool_body" not in result["pair_counts"]


def test_prepared_cell_reuses_snapshot_and_closes_native_geometry():
    cell, state = RobotCellLibrary.ur5_gripper_one_beam()
    config = state.robot_configuration
    point = JointTrajectoryPoint(config.joint_values, config.joint_types,
                                 joint_names=config.joint_names)
    trajectory = JointTrajectory([point, point], joint_names=config.joint_names)
    with compas_forge.prepare_compas_fab_cell(cell, state) as prepared:
        count = prepared.mesh_count
        first = prepared.sweep(trajectory, parallel=False)
        # Original state edits must not invalidate a prepared native snapshot.
        state.rigid_body_states['floor'].is_hidden = True
        second = prepared.sweep(trajectory, parallel=False)
        assert first == second
        assert prepared.mesh_count == count and count > 7
        clearance = prepared.verify_clearance(trajectory, 0.01, parallel=False)
        assert clearance['status'] == 'violation'
        assert clearance['segment_result']['witness_distance'] < 0.01
    assert prepared.mesh_count == 0
    prepared.close()
    with pytest.raises(RuntimeError, match='closed'):
        prepared.sweep(trajectory)


def test_nonuniform_trajectory_time_mapping_and_untimed_result():
    # Independent arithmetic oracle: halfway through [2, 10] is 6 seconds,
    # not halfway through the three-point trajectory.
    assert compas_forge._collision_timing([0,2,10], True, 1, 0.5) == {
        'collision_time_from_start': 6.0,
        'trajectory_time_fraction': 0.6,
        'trajectory_duration': 10,
    }
    assert compas_forge._collision_timing([0,0,0], False, 1, 0.5) == {
        'collision_time_from_start': None,
        'trajectory_time_fraction': None,
        'trajectory_duration': None,
    }


def test_prepared_partial_trajectory_preserves_unspecified_joint_state():
    cell,state = RobotCellLibrary.ur5_gripper_one_beam()
    state.robot_configuration.joint_values[1] = -0.4
    config = state.robot_configuration
    full = JointTrajectoryPoint(config.joint_values, config.joint_types,
                                joint_names=config.joint_names)
    part = JointTrajectoryPoint([config.joint_values[0]], [config.joint_types[0]],
                                joint_names=[config.joint_names[0]])
    with compas_forge.prepare_compas_fab_cell(cell,state) as prepared:
        complete = prepared.sweep(JointTrajectory([full,full],joint_names=config.joint_names))
        partial = prepared.sweep(JointTrajectory([part,part],joint_names=part.joint_names))
        assert partial == complete
