import json
import math
from array import array
from typing import Optional
from compas.datastructures import Mesh
from compas.geometry import Frame, Quaternion
from compas_forge._core import (
    validate_compas_json,
    detect_clashes_json,
    fix_mesh_json,
    run_preflight_json,
    validate_mesh_buffers,
    scan_mesh_buffers_borrowed,
    topology_predicates_buffers,
    check_swept_collision,
    compute_assembly_contacts,
    fix_mesh_buffers,
    run_preflight_buffers,
    register_mesh,
    register_mesh_buffers,
    unregister_mesh,
    clear_mesh_registry,
    check_swept_collision_cached,
    check_swept_collision_cached_native,
    check_swept_collision_cached_batch_native,
    verify_clearance_cached_batch_native,
)
from compas_forge.reporter import generate_html_report

__version__ = "0.4.0"

# COMPAS plugin discovery metadata for supported CPython hosts.
# The PyO3 extension cannot be imported by Rhino's legacy IronPython runtime.
__all_plugins__ = ["compas_forge.plugin"]

__all__ = [
    "validate_compas_json",
    "detect_clashes_json",
    "fix_mesh_json",
    "run_preflight_json",
    "validate_mesh_buffers",
    "scan_mesh_buffers_borrowed",
    "topology_predicates_buffers",
    "check_swept_collision",
    "compute_assembly_contacts",
    "fix_mesh_buffers",
    "run_preflight_buffers",
    "register_mesh",
    "register_mesh_buffers",
    "unregister_mesh",
    "clear_mesh_registry",
    "check_swept_collision_cached",
    "check_swept_collision_cached_native",
    "check_swept_collision_cached_batch_native",
    "register_mesh_to_cache",
    "clear_mesh_cache",
    "check_swept_collision_cached_poses",
    "check_swept_collision_cached_batch_poses",
    "pose_from_frame",
    "analyze_mesh",
    "scan_mesh_ingress",
    "repair_mesh",
    "preflight_mesh",
    "sweep_collision",
    "sweep_compas_fab_trajectory",
    "sweep_compas_fab_robot_trajectory",
    "sweep_compas_fab_cell_trajectory",
    "prepare_compas_fab_cell",
    "verify_clearance_cached_batch_poses",
    "validate_compas_fab_trajectory",
    "assembly_contacts",
    "compas_mesh_to_buffers",
    "compas_mesh_to_topology_buffers",
    "mesh_topology_predicates",
    "verify_file",
    "verify_mesh_zero_copy",
    "fix_mesh_zero_copy",
    "run_preflight_profile_zero_copy",
    "check_swept_collision_zero_copy",
    "compute_assembly_contacts_zero_copy",
    "check_assembly_clashes",
    "fix_geometry_file",
    "run_preflight_profile",
    "install_mesh_accelerators",
    "uninstall_mesh_accelerators",
    "accelerators_installed",
    "mesh_acceleration",
]


def install_mesh_accelerators():
    """Opt in to reversible process-wide acceleration of two COMPAS Mesh predicates."""
    from compas_forge.plugin import install_mesh_accelerators as install

    return install()


def uninstall_mesh_accelerators():
    """Release one accelerator installation level and restore COMPAS at zero."""
    from compas_forge.plugin import uninstall_mesh_accelerators as uninstall

    return uninstall()


def accelerators_installed():
    """Return whether the opt-in COMPAS Mesh accelerator is active."""
    from compas_forge.plugin import accelerators_installed as installed

    return installed()


def mesh_acceleration():
    """Return the reversible COMPAS Mesh acceleration context manager."""
    from compas_forge.plugin import mesh_acceleration as context

    return context()


def pose_from_frame(frame: Frame) -> list[float]:
    """Convert a COMPAS frame to ``[x, y, z, qx, qy, qz, qw]``.

    The pose layout matches the Rust collision API and common robotics
    conventions used by COMPAS-based workflows.
    """
    if not isinstance(frame, Frame):
        raise TypeError("frame must be a compas.geometry.Frame")
    quaternion = Quaternion.from_frame(frame)
    return [
        frame.point.x,
        frame.point.y,
        frame.point.z,
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    ]


def _coerce_pose(value) -> list[float]:
    if isinstance(value, Frame):
        return pose_from_frame(value)
    try:
        pose = [float(component) for component in value]
    except (TypeError, ValueError) as error:
        raise TypeError("pose must be a COMPAS Frame or an iterable of 7 numbers") from error
    if len(pose) != 7:
        raise ValueError("pose must contain 7 numbers: x, y, z, qx, qy, qz, qw")
    return pose

def register_mesh_to_cache(mesh_id: str, mesh: Mesh) -> str:
    """
    Register an owned Rust snapshot of a COMPAS mesh under a caller-supplied ID.
    Reusing the ID avoids rebuilding the mesh for later swept-collision queries.
    """
    v, idx, off = compas_mesh_to_buffers(mesh)
    return register_mesh_buffers(str(mesh_id), v, idx, off)

def clear_mesh_cache() -> str:
    """
    Clear all registered mesh snapshots from the process-local registry.
    """
    return clear_mesh_registry()

def check_swept_collision_cached_poses(
    mesh1_id: str, pose1_start, pose1_end, 
    mesh2_id: str, pose2_start, pose2_end
) -> dict:
    """
    Evaluates Continuous Collision Detection (CCD) between two pre-registered meshes.
    Poses are passed as a 7-element array: [x, y, z, qx, qy, qz, qw].
    Bypasses mesh reconstruction; runtime depends on geometry and motion.
    """
    return check_swept_collision_cached_native(
        str(mesh1_id), list(pose1_start), list(pose1_end),
        str(mesh2_id), list(pose2_start), list(pose2_end)
    )


def check_swept_collision_cached_batch_poses(
    queries, parallel: bool = True
) -> list[dict]:
    """Run retained-mesh CCD queries in one GIL-free native Rayon batch.

    Each input is ``(mesh_a_id, pose_a_start, pose_a_end, mesh_b_id,
    pose_b_start, pose_b_end)``. Input order is preserved in the results.
    """
    packed = [
        (
            str(mesh_a_id),
            _coerce_pose(pose_a_start),
            _coerce_pose(pose_a_end),
            str(mesh_b_id),
            _coerce_pose(pose_b_start),
            _coerce_pose(pose_b_end),
        )
        for (
            mesh_a_id,
            pose_a_start,
            pose_a_end,
            mesh_b_id,
            pose_b_start,
            pose_b_end,
        ) in queries
    ]
    return check_swept_collision_cached_batch_native(packed, parallel)

def verify_clearance_cached_batch_poses(
    queries, clearance, numerical_margin=1e-8, time_tolerance=1e-5,
    max_evaluations=4096, parallel=True, solid=False,
    clearance_offsets=None,
    solid_rule="single_shell",
    broadphase=True,
):
    """Check surface clearance for cached rigid sweeps; return clear/violation/unknown.

    Query layout is the same as check_swept_collision_cached_batch_poses.
    Clear is conditional on numerical_margin bounding distance and arithmetic
    error. A violation's witness_time is a sample, not first threshold crossing.
    Time is normalized to [0, 1]. Distances use the input mesh coordinate units.
    solid=True also checks initial containment, restricted to valid single-shell
    closed embedded meshes. Unsupported solid inputs return unknown.
    clearance_offsets optionally adds a finite nonnegative allowance per query.
    The reported clearance is the effective threshold including that allowance.
    solid_rule='even_odd' permits disjoint/nested embedded closed shells with
    alternating material/void nesting. This is NOT union of overlapping solids.
    """
    if not isinstance(solid, bool):
        raise TypeError("solid must be a bool")
    if not isinstance(parallel, bool):
        raise TypeError("parallel must be a bool")
    if not isinstance(broadphase, bool):
        raise TypeError("broadphase must be a bool")
    packed = [
        (str(a), _coerce_pose(a0), _coerce_pose(a1),
         str(b), _coerce_pose(b0), _coerce_pose(b1))
        for a, a0, a1, b, b0, b1 in queries
    ]
    return verify_clearance_cached_batch_native(
        packed, clearance, numerical_margin, time_tolerance, max_evaluations, parallel, solid,
        clearance_offsets, solid_rule, broadphase,
    )


def compas_mesh_to_buffers(mesh):
    """
    Extract flat contiguous Python arrays from a COMPAS Mesh object.
    The Rust boundary validates and copies these arrays into owned memory.
    """
    flat_vertices = []
    vertex_keys = list(mesh.vertices())
    key_to_index = {key: index for index, key in enumerate(vertex_keys)}
    for vertex in vertex_keys:
        flat_vertices.extend(mesh.vertex_coordinates(vertex))
    
    face_indices = []
    face_offsets = [0]
    for face in mesh.faces():
        vertices = mesh.face_vertices(face)
        face_indices.extend(key_to_index[key] for key in vertices)
        face_offsets.append(len(face_indices))
        
    vertices_arr = array('d', flat_vertices)
    face_indices_arr = array('i', face_indices)
    face_offsets_arr = array('i', face_offsets)
    
    return vertices_arr, face_indices_arr, face_offsets_arr


def compas_mesh_to_topology_buffers(mesh):
    """Pack only face topology for the dedicated predicate kernel."""
    vertex_keys = list(mesh.vertices())
    key_to_index = {key: index for index, key in enumerate(vertex_keys)}
    face_indices = []
    face_offsets = [0]
    for vertices in mesh.face.values():
        face_indices.extend(key_to_index[key] for key in vertices)
        face_offsets.append(len(face_indices))
    return len(vertex_keys), array("i", face_indices), array("i", face_offsets)


def mesh_topology_predicates(mesh: Mesh) -> dict:
    """Return lightweight closed/manifold predicates without geometry analysis."""
    if not isinstance(mesh, Mesh):
        raise TypeError("mesh must be a compas.datastructures.Mesh")
    vertex_count = mesh.number_of_vertices()
    if vertex_count == 0:
        return {"is_closed": False, "is_manifold": False}
    if mesh.number_of_faces() == 0:
        return {"is_closed": True, "is_manifold": False}
    vertex_count, indices, offsets = compas_mesh_to_topology_buffers(mesh)
    return topology_predicates_buffers(vertex_count, indices, offsets)

def verify_file(filepath: str) -> dict:
    with open(filepath, 'r', encoding='utf-8') as f:
        raw_data = f.read()
    report_raw = validate_compas_json(raw_data)
    return json.loads(report_raw)

def verify_mesh_zero_copy(mesh) -> dict:
    """
    Validates an owned snapshot of the supplied PyBuffer arrays.
    The returned diagnostic report is decoded from JSON.
    """
    v_arr, idx_arr, off_arr = compas_mesh_to_buffers(mesh)
    report_raw = validate_mesh_buffers(v_arr, idx_arr, off_arr)
    return json.loads(report_raw)


def analyze_mesh(mesh: Mesh) -> dict:
    """Return topology diagnostics for a COMPAS mesh.

    The mesh is converted to contiguous Python buffers and copied into an
    owned Rust snapshot before analysis.
    """
    return verify_mesh_zero_copy(mesh)


def scan_mesh_ingress(mesh: Mesh) -> dict:
    """Run the allocation-free Rust ingress scan for a COMPAS mesh.

    COMPAS data still has to be packed into contiguous arrays first. The Rust
    scan then borrows those arrays without another input copy. Call
    :func:`scan_mesh_buffers_borrowed` directly when the producer already owns
    contiguous ``float64``/``int32`` buffers (for example NumPy arrays).
    """
    vertices, indices, offsets = compas_mesh_to_buffers(mesh)
    return scan_mesh_buffers_borrowed(vertices, indices, offsets)

def fix_mesh_zero_copy(mesh, weld_tolerance: float = 1e-6) -> tuple:
    """
    Welds vertices and aligns adjacent face winding in an owned mesh snapshot.
    Returns a reconstructed mesh and a decoded repair report; inspect remaining defects.
    """
    v_arr, idx_arr, off_arr = compas_mesh_to_buffers(mesh)
    report_raw = fix_mesh_buffers(v_arr, idx_arr, off_arr, weld_tolerance)
    report = json.loads(report_raw)
    
    fixed_mesh = Mesh()
    vertices_flat = report["vertices"]
    for i in range(0, len(vertices_flat), 3):
        fixed_mesh.add_vertex(
            x=vertices_flat[i], 
            y=vertices_flat[i+1], 
            z=vertices_flat[i+2]
        )
        
    face_indices = report["face_indices"]
    face_offsets = report["face_offsets"]
    for i in range(len(face_offsets) - 1):
        start = face_offsets[i]
        end = face_offsets[i+1]
        face_verts = face_indices[start:end]
        fixed_mesh.add_face(face_verts)
        
    return fixed_mesh, report


def repair_mesh(mesh: Mesh, weld_tolerance: float = 1e-6) -> tuple[Mesh, dict]:
    """Conservatively weld nearby vertices and align adjacent face winding.

    ``weld_tolerance`` is an explicit Euclidean distance in the mesh coordinate
    units. This operation does not claim to repair holes or self-intersections.
    """
    return fix_mesh_zero_copy(mesh, weld_tolerance)

def run_preflight_profile_zero_copy(mesh, profile_name: str) -> dict:
    """
    Runs fabrication-profile checks on an owned snapshot of mesh buffers.
    Returns a decoded JSON report without writing intermediate mesh files.
    """
    v_arr, idx_arr, off_arr = compas_mesh_to_buffers(mesh)
    report_raw = run_preflight_buffers(v_arr, idx_arr, off_arr, profile_name)
    return json.loads(report_raw)


def preflight_mesh(mesh: Mesh, profile_name: str = "default") -> dict:
    """Evaluate a COMPAS mesh against a named experimental profile."""
    return run_preflight_profile_zero_copy(mesh, profile_name)

def check_swept_collision_zero_copy(
    mesh_a, pose_a_start, pose_a_end, 
    mesh_b, pose_b_start, pose_b_end
) -> dict:
    """
    Evaluates Continuous Collision Detection (CCD) between two moving COMPAS meshes.
    Poses are passed as a 7-element array: [x, y, z, qx, qy, qz, qw].
    Uses Parry nonlinear rigid-motion shape casts over normalized time [0, 1].
    """
    v_arr_a, idx_arr_a, off_arr_a = compas_mesh_to_buffers(mesh_a)
    v_arr_b, idx_arr_b, off_arr_b = compas_mesh_to_buffers(mesh_b)
    
    result_raw = check_swept_collision(
        v_arr_a, idx_arr_a, off_arr_a, list(pose_a_start), list(pose_a_end),
        v_arr_b, idx_arr_b, off_arr_b, list(pose_b_start), list(pose_b_end)
    )
    return json.loads(result_raw)


def sweep_collision(
    mesh_a: Mesh,
    pose_a_start,
    pose_a_end,
    mesh_b: Mesh,
    pose_b_start,
    pose_b_end,
) -> dict:
    """Check continuous rigid-motion collision between two COMPAS meshes.

    Poses may be :class:`compas.geometry.Frame` objects or iterables in
    ``[x, y, z, qx, qy, qz, qw]`` order. Time is normalised to ``[0, 1]``.
    """
    return check_swept_collision_zero_copy(
        mesh_a,
        _coerce_pose(pose_a_start),
        _coerce_pose(pose_a_end),
        mesh_b,
        _coerce_pose(pose_b_start),
        _coerce_pose(pose_b_end),
    )


def sweep_compas_fab_trajectory(
    robot,
    trajectory,
    moving_mesh: Mesh,
    obstacle_mesh: Mesh,
    obstacle_frame: Optional[Frame] = None,
    group=None,
    link=None,
) -> dict:
    """Sweep tool geometry along a COMPAS FAB joint trajectory.

    Each adjacent pair of trajectory points is converted to end-effector
    frames with model-based forward kinematics, then checked as one continuous
    rigid-motion segment. Dense trajectory sampling remains the caller's
    responsibility because joint interpolation is approximated piecewise in
    Cartesian space.
    """
    points = list(getattr(trajectory, "points", []))
    if len(points) < 2:
        raise ValueError("trajectory must contain at least two points")
    validation = _require_valid_compas_fab_trajectory(robot, trajectory)
    trajectory_times = _trajectory_times(trajectory)
    if obstacle_frame is None:
        obstacle_frame = Frame.worldXY()

    frames = [
        _compas_fab_forward_kinematics(robot, point, group=group, link=link)
        for point in points
    ]

    for index, (frame_start, frame_end) in enumerate(zip(frames, frames[1:])):
        segment = sweep_collision(
            moving_mesh,
            frame_start,
            frame_end,
            obstacle_mesh,
            obstacle_frame,
            obstacle_frame,
        )
        if segment["has_collision"]:
            segment_fraction = segment["time_of_impact"]
            fraction_in_trajectory = (index + segment_fraction) / (len(points) - 1)
            return {
                "has_collision": True,
                "segment_index": index,
                "segment_count": len(points) - 1,
                "segment_fraction": segment_fraction,
                "trajectory_fraction": fraction_in_trajectory,
                "segment_result": segment,
                "frames_evaluated": len(frames),
                **_collision_timing(
                    trajectory_times,
                    validation["timing_supplied"],
                    index,
                    segment_fraction,
                ),
            }

    return {
        "has_collision": False,
        "segment_index": None,
        "segment_count": len(points) - 1,
        "segment_fraction": None,
        "trajectory_fraction": None,
        "segment_result": None,
        "frames_evaluated": len(frames),
        **_collision_timing(
            trajectory_times, validation["timing_supplied"], None, None
        ),
    }


def _compas_fab_robot_model(robot):
    """Return a RobotModel from current RobotCell or legacy Robot wrappers."""
    model = getattr(robot, "robot_model", None)
    if model is None:
        model = getattr(robot, "model", None)
    if model is None and hasattr(robot, "compute_transformations"):
        model = robot
    if model is None:
        raise TypeError(
            "robot must be a COMPAS FAB RobotCell, a legacy Robot, or a RobotModel"
        )
    return model


def _compas_fab_forward_kinematics(robot, point, group=None, link=None):
    """Use the COMPAS FAB 2 RobotCell model, with legacy wrapper compatibility."""
    model = _compas_fab_robot_model(robot)
    configuration = _configuration_from_trajectory_point(
        point, model.get_configurable_joint_names()
    )

    # COMPAS FAB 2 delegates model FK to compas_robots.RobotModel.  A group
    # belongs to the former multi-group Robot wrapper API and has no equivalent
    # for a single RobotCell model.
    if getattr(robot, "robot_model", None) is not None or robot is model:
        if group is not None:
            raise ValueError("group is not supported by the COMPAS FAB 2 RobotCell API")
        return model.forward_kinematics(configuration, link_name=link)

    options = {"solver": "model"}
    if link is not None:
        options["link"] = link
    return robot.forward_kinematics(configuration, group=group, options=options)


def _compas_fab_collision_links(robot, cache_prefix: str, registered_ids) -> list[dict]:
    """Bake URDF collision origins and register one retained mesh per link."""
    model = _compas_fab_robot_model(robot)
    records = []
    for link in model.links:
        combined = None
        for collision in link.collision:
            meshes = list(getattr(collision.geometry.shape, "meshes", []) or [])
            if not meshes:
                raise ValueError(
                    f"collision geometry for link '{link.name}' is not loaded; "
                    "load the RobotCell/RobotModel collision geometry first"
                )
            for source in meshes:
                mesh = source.copy()
                if collision.init_transformation is not None:
                    mesh.transform(collision.init_transformation)
                if combined is None:
                    combined = mesh
                else:
                    combined.join(mesh)
        if combined is None:
            continue
        mesh_id = f"{cache_prefix}:link:{link.name}"
        register_mesh_to_cache(mesh_id, combined)
        # Record ownership immediately so the caller cleans up partial failures.
        registered_ids.append(mesh_id)
        radius = max(math.hypot(*combined.vertex_coordinates(key)) for key in combined.vertices())
        records.append({"name": link.name, "mesh_id": mesh_id, "link": link, "origin_radius": radius})
    if not records:
        raise ValueError("robot model has no loaded collision geometry")
    return records


def _configuration_from_trajectory_point(point, fallback_names):
    from compas_robots import Configuration

    names = list(point.joint_names or fallback_names or [])
    values = list(point.joint_values)
    joint_types = list(point.joint_types)
    if len(values) != len(joint_types):
        raise ValueError("trajectory point values and joint types must have equal length")
    if names and len(names) != len(values):
        raise ValueError("trajectory point joint names must match its values")
    return Configuration(values, joint_types, names)


def validate_compas_fab_trajectory(robot, trajectory) -> dict:
    """Validate trajectory structure, URDF limits, timing and average speed."""
    from compas_robots.model import Joint

    model = _compas_fab_robot_model(robot)
    points = list(getattr(trajectory, "points", []))
    errors = []
    names = list(
        getattr(trajectory, "joint_names", None)
        or model.get_configurable_joint_names()
    )
    if len(points) < 2:
        errors.append(
            {"code": "too_few_points", "message": "trajectory needs at least two points"}
        )
    if len(names) != len(set(names)):
        errors.append(
            {"code": "duplicate_joint_name", "message": "joint names must be unique"}
        )

    model_joints = {joint.name: joint for joint in model.get_configurable_joints()}
    unknown = [name for name in names if name not in model_joints]
    for name in unknown:
        errors.append(
            {"code": "unknown_joint", "joint": name, "message": f"unknown joint '{name}'"}
        )

    configurations = []
    times = []
    for point_index, point in enumerate(points):
        try:
            configuration = _configuration_from_trajectory_point(point, names)
        except (TypeError, ValueError) as error:
            errors.append(
                {
                    "code": "invalid_point_shape",
                    "point_index": point_index,
                    "message": str(error),
                }
            )
            continue
        configurations.append(configuration)
        if list(configuration.joint_names) != names:
            errors.append(
                {
                    "code": "joint_order_mismatch",
                    "point_index": point_index,
                    "message": "trajectory points must use the trajectory joint order",
                }
            )
        for joint_index, (name, value) in enumerate(
            zip(configuration.joint_names, configuration.joint_values)
        ):
            joint = model_joints.get(name)
            if joint is not None and configuration.joint_types[joint_index] != joint.type:
                errors.append(
                    {
                        "code": "joint_type_mismatch",
                        "point_index": point_index,
                        "joint": name,
                        "expected": joint.type,
                        "actual": configuration.joint_types[joint_index],
                    }
                )
            if not math.isfinite(value):
                errors.append(
                    {
                        "code": "nonfinite_joint_value",
                        "point_index": point_index,
                        "joint": name,
                        "value": value,
                    }
                )
                continue
            joint = model_joints.get(name)
            if joint is None or joint.type == Joint.CONTINUOUS or joint.limit is None:
                continue
            if value < joint.limit.lower or value > joint.limit.upper:
                errors.append(
                    {
                        "code": "joint_limit",
                        "point_index": point_index,
                        "joint_index": joint_index,
                        "joint": name,
                        "value": value,
                        "lower": joint.limit.lower,
                        "upper": joint.limit.upper,
                    }
                )

        duration = getattr(point, "time_from_start", None)
        times.append(float(duration.seconds) if duration is not None else 0.0)
        if not math.isfinite(times[-1]) or times[-1] < 0.0:
            errors.append(
                {
                    "code": "invalid_time_from_start",
                    "point_index": point_index,
                    "value": times[-1],
                }
            )
        for field in ("velocities", "accelerations", "effort"):
            values = list(getattr(point, field, None) or [])
            if values and len(values) != len(configuration.joint_values):
                errors.append(
                    {
                        "code": "point_vector_length",
                        "point_index": point_index,
                        "field": field,
                        "expected": len(configuration.joint_values),
                        "actual": len(values),
                    }
                )
            for value in values:
                if not math.isfinite(value):
                    errors.append(
                        {
                            "code": "nonfinite_point_vector",
                            "point_index": point_index,
                            "field": field,
                            "value": value,
                        }
                    )
            if field == "velocities" and len(values) == len(configuration.joint_values):
                for name, value in zip(configuration.joint_names, values):
                    joint = model_joints.get(name)
                    velocity_limit = getattr(
                        getattr(joint, "limit", None), "velocity", None
                    )
                    if (
                        math.isfinite(value)
                        and velocity_limit is not None
                        and velocity_limit > 0.0
                        and abs(value) > velocity_limit + 1e-12
                    ):
                        errors.append(
                            {
                                "code": "declared_velocity_limit",
                                "point_index": point_index,
                                "joint": name,
                                "value": value,
                                "limit": velocity_limit,
                            }
                        )

    timing_supplied = any(time != 0.0 for time in times)
    if timing_supplied and len(configurations) == len(points):
        for index, (start, end) in enumerate(zip(times, times[1:])):
            delta_time = end - start
            if not math.isfinite(start) or not math.isfinite(end) or delta_time <= 0.0:
                errors.append(
                    {
                        "code": "non_monotonic_time",
                        "segment_index": index,
                        "start_time": start,
                        "end_time": end,
                    }
                )
                continue
            for name, value_a, value_b in zip(
                configurations[index].joint_names,
                configurations[index].joint_values,
                configurations[index + 1].joint_values,
            ):
                joint = model_joints.get(name)
                velocity_limit = getattr(getattr(joint, "limit", None), "velocity", None)
                if velocity_limit is None or velocity_limit <= 0.0:
                    continue
                average_velocity = abs(value_b - value_a) / delta_time
                if average_velocity > velocity_limit + 1e-12:
                    errors.append(
                        {
                            "code": "average_velocity_limit",
                            "segment_index": index,
                            "joint": name,
                            "value": average_velocity,
                            "limit": velocity_limit,
                        }
                    )

    return {
        "is_valid": not errors,
        "point_count": len(points),
        "joint_count": len(names),
        "timing_supplied": timing_supplied,
        "errors": errors,
    }


def _require_valid_compas_fab_trajectory(robot, trajectory):
    report = validate_compas_fab_trajectory(robot, trajectory)
    if not report["is_valid"]:
        codes = ", ".join(error["code"] for error in report["errors"])
        raise ValueError(f"invalid COMPAS FAB trajectory: {codes}")
    return report


def _trajectory_times(trajectory) -> list[float]:
    """Return COMPAS FAB point times in seconds, using zero when omitted."""
    return [
        float(duration.seconds) if duration is not None else 0.0
        for duration in (
            getattr(point, "time_from_start", None)
            for point in getattr(trajectory, "points", [])
        )
    ]


def _collision_timing(times, timing_supplied, segment_index, segment_fraction):
    """Map a collision inside one trajectory segment to its declared time."""
    duration = times[-1] - times[0] if timing_supplied else None
    if segment_index is None or segment_fraction is None or not timing_supplied:
        return {
            "collision_time_from_start": None,
            "trajectory_time_fraction": None,
            "trajectory_duration": duration,
        }
    collision_time = times[segment_index] + segment_fraction * (
        times[segment_index + 1] - times[segment_index]
    )
    return {
        "collision_time_from_start": collision_time,
        "trajectory_time_fraction": (
            (collision_time - times[0]) / duration if duration and duration > 0.0 else None
        ),
        "trajectory_duration": duration,
    }


def _interpolate_configuration(start, end, fraction):
    from compas_robots import Configuration

    if list(start.joint_names) != list(end.joint_names):
        raise ValueError("trajectory points must use the same joint order")
    if list(start.joint_types) != list(end.joint_types):
        raise ValueError("trajectory points must use the same joint types")
    values = [
        a + (b - a) * fraction
        for a, b in zip(start.joint_values, end.joint_values)
    ]
    return Configuration(values, list(start.joint_types), list(start.joint_names))


def _robot_link_pose_map(robot, configuration, link_records, robot_base_frame=None):
    from compas.geometry import Frame, Transformation

    model = _compas_fab_robot_model(robot)
    transformations = model.compute_transformations(configuration)
    base_transformation = (
        Transformation.from_frame(robot_base_frame)
        if robot_base_frame is not None
        else Transformation()
    )
    poses = {}
    for record in link_records:
        parent_joint = record["link"].parent_joint
        transformation = (
            transformations[parent_joint.name]
            if parent_joint is not None
            else Transformation()
        )
        poses[record["name"]] = pose_from_frame(
            Frame.from_transformation(base_transformation * transformation)
        )
    return poses


def _execute_cached_robot_pair(query):
    result = check_swept_collision_cached_poses(
        query[4], query[5], query[6], query[7], query[8], query[9]
    )
    return query[0], query[1], query[2], query[3], result


def _robot_semantic_disabled_collision_pairs(robot) -> set[frozenset]:
    """Return unordered SRDF collision exclusions exposed by COMPAS FAB."""
    semantics = getattr(robot, "robot_semantics", None)
    if semantics is None:
        semantics = getattr(robot, "semantics", None)
    if semantics is None:
        return set()

    unordered = getattr(semantics, "unordered_disabled_collisions", None)
    if unordered is not None:
        return {frozenset(pair) for pair in unordered}

    ordered = getattr(semantics, "disabled_collisions", None) or []
    return {frozenset(pair) for pair in ordered}


def sweep_compas_fab_robot_trajectory(
    robot,
    trajectory,
    obstacles: dict[str, Mesh],
    obstacle_frames: Optional[dict[str, Frame]] = None,
    check_self_collision: bool = True,
    disabled_self_collision_pairs=None,
    respect_robot_semantics: bool = True,
    robot_base_frame: Optional[Frame] = None,
    max_joint_step: float = math.radians(5.0),
    max_prismatic_step: float = 0.01,
    max_subdivisions: int = 128,
    workers: int = 1,
) -> dict:
    """Check all loaded robot-link collision meshes along a joint trajectory.

    Joint-space segments are adaptively subdivided, then each link's exact rigid
    endpoint poses are swept over every subinterval. Parent-child pairs and,
    by default, SRDF disabled-collision pairs from the RobotCell semantics are
    excluded. The result is a deterministic preflight diagnostic, not a
    robot-safety certificate.
    """
    from uuid import uuid4
    from compas_robots.model import Joint

    points = list(getattr(trajectory, "points", []))
    if len(points) < 2:
        raise ValueError("trajectory must contain at least two points")
    validation = _require_valid_compas_fab_trajectory(robot, trajectory)
    trajectory_times = _trajectory_times(trajectory)
    if not isinstance(obstacles, dict):
        raise TypeError("obstacles must be a name-to-Mesh dictionary")
    if not math.isfinite(max_joint_step) or max_joint_step <= 0.0:
        raise ValueError("max_joint_step must be finite and greater than zero")
    if not math.isfinite(max_prismatic_step) or max_prismatic_step <= 0.0:
        raise ValueError("max_prismatic_step must be finite and greater than zero")
    if isinstance(max_subdivisions, bool) or not isinstance(max_subdivisions, int) or max_subdivisions < 1:
        raise ValueError("max_subdivisions must be a positive integer")
    if isinstance(workers, bool) or not isinstance(workers, int) or workers < 1:
        raise ValueError("workers must be a positive integer")
    if robot_base_frame is not None and not isinstance(robot_base_frame, Frame):
        raise TypeError("robot_base_frame must be a COMPAS Frame")

    obstacle_frames = obstacle_frames or {}
    prefix = f"compas-fab:{uuid4().hex}"
    registered_ids = []
    executor = None
    try:
        if workers > 1:
            from concurrent.futures import ThreadPoolExecutor

            executor = ThreadPoolExecutor(max_workers=workers)
        link_records = _compas_fab_collision_links(robot, prefix, registered_ids)

        obstacle_records = []
        for name in sorted(obstacles):
            mesh = obstacles[name]
            if not isinstance(mesh, Mesh):
                raise TypeError(f"obstacle '{name}' must be a COMPAS Mesh")
            frame = obstacle_frames.get(name, Frame.worldXY())
            if not isinstance(frame, Frame):
                raise TypeError(f"obstacle frame '{name}' must be a COMPAS Frame")
            mesh_id = f"{prefix}:obstacle:{name}"
            register_mesh_to_cache(mesh_id, mesh)
            registered_ids.append(mesh_id)
            obstacle_records.append(
                {"name": str(name), "mesh_id": mesh_id, "pose": pose_from_frame(frame)}
            )

        model = _compas_fab_robot_model(robot)
        adjacency = {
            frozenset((joint.parent.link, joint.child.link))
            for joint in model.joints
            if joint.parent is not None and joint.child is not None
        }
        explicit_disabled = {
            frozenset(pair) for pair in (disabled_self_collision_pairs or [])
        }
        semantic_disabled = (
            _robot_semantic_disabled_collision_pairs(robot)
            if respect_robot_semantics
            else set()
        )
        disabled = explicit_disabled | semantic_disabled
        self_pairs = []
        if check_self_collision:
            for index, first in enumerate(link_records):
                for second in link_records[index + 1:]:
                    pair = frozenset((first["name"], second["name"]))
                    if pair not in adjacency and pair not in disabled:
                        self_pairs.append((first, second))

        configurations = [
            _configuration_from_trajectory_point(point, trajectory.joint_names)
            for point in points
        ]
        evaluated_subsegments = 0
        fk_evaluations = 0
        original_segment_count = len(configurations) - 1

        for segment_index, (start, end) in enumerate(
            zip(configurations, configurations[1:])
        ):
            ratios = []
            for value_a, value_b, joint_type in zip(
                start.joint_values, end.joint_values, start.joint_types
            ):
                delta = abs(value_b - value_a)
                if joint_type in (Joint.REVOLUTE, Joint.CONTINUOUS):
                    ratios.append(delta / max_joint_step)
                elif joint_type == Joint.PRISMATIC:
                    ratios.append(delta / max_prismatic_step)
            subdivisions = max(1, math.ceil(max(ratios, default=0.0)))
            if subdivisions > max_subdivisions:
                raise ValueError(
                    f"segment {segment_index} requires {subdivisions} subdivisions, "
                    f"above max_subdivisions={max_subdivisions}"
                )

            sampled = [
                _interpolate_configuration(start, end, index / subdivisions)
                for index in range(subdivisions + 1)
            ]
            pose_maps = [
                _robot_link_pose_map(
                    robot, configuration, link_records, robot_base_frame
                )
                for configuration in sampled
            ]
            fk_evaluations += len(pose_maps)

            for subsegment_index in range(subdivisions):
                evaluated_subsegments += 1
                start_poses = pose_maps[subsegment_index]
                end_poses = pose_maps[subsegment_index + 1]
                candidates = []
                queries = []

                for link_record in link_records:
                    for obstacle in obstacle_records:
                        queries.append(
                            (
                                "environment",
                                link_record["name"],
                                obstacle["name"],
                                obstacle["name"],
                                link_record["mesh_id"],
                                start_poses[link_record["name"]],
                                end_poses[link_record["name"]],
                                obstacle["mesh_id"],
                                obstacle["pose"],
                                obstacle["pose"],
                            )
                        )

                for first, second in self_pairs:
                    queries.append(
                        (
                            "self",
                            first["name"],
                            second["name"],
                            second["name"],
                            first["mesh_id"],
                            start_poses[first["name"]],
                            end_poses[first["name"]],
                            second["mesh_id"],
                            start_poses[second["name"]],
                            end_poses[second["name"]],
                        )
                    )

                evaluated = (
                    executor.map(_execute_cached_robot_pair, queries)
                    if executor is not None
                    else map(_execute_cached_robot_pair, queries)
                )
                for collision_type, link_a, target, _, result in evaluated:
                    if result["has_collision"]:
                        candidates.append(
                            (
                                result["time_of_impact"],
                                collision_type,
                                link_a,
                                target,
                                result,
                            )
                        )

                if candidates:
                    candidates.sort(key=lambda item: (item[0], item[1], item[2], item[3]))
                    local_toi, collision_type, link_a, target, result = candidates[0]
                    fraction_in_segment = (subsegment_index + local_toi) / subdivisions
                    return {
                        "has_collision": True,
                        "collision_type": collision_type,
                        "link_a": link_a,
                        "link_b": target if collision_type == "self" else None,
                        "obstacle": target if collision_type == "environment" else None,
                        "segment_index": segment_index,
                        "subsegment_index": subsegment_index,
                        "subdivisions_in_segment": subdivisions,
                        "segment_fraction": fraction_in_segment,
                        "trajectory_fraction": (
                            segment_index + fraction_in_segment
                        ) / original_segment_count,
                        "segment_result": result,
                        "link_count": len(link_records),
                        "self_pair_count": len(self_pairs),
                        "disabled_self_pair_count": len(disabled),
                        "semantics_disabled_pair_count": len(semantic_disabled),
                        "respect_robot_semantics": respect_robot_semantics,
                        "robot_base_frame_applied": robot_base_frame is not None,
                        "evaluated_subsegments": evaluated_subsegments,
                        "fk_evaluations": fk_evaluations,
                        "max_joint_step": max_joint_step,
                        "max_prismatic_step": max_prismatic_step,
                        "workers": workers,
                        **_collision_timing(
                            trajectory_times,
                            validation["timing_supplied"],
                            segment_index,
                            fraction_in_segment,
                        ),
                    }

        return {
            "has_collision": False,
            "collision_type": None,
            "link_a": None,
            "link_b": None,
            "obstacle": None,
            "segment_index": None,
            "subsegment_index": None,
            "subdivisions_in_segment": None,
            "segment_fraction": None,
            "trajectory_fraction": None,
            "segment_result": None,
            "link_count": len(link_records),
            "self_pair_count": len(self_pairs),
            "disabled_self_pair_count": len(disabled),
            "semantics_disabled_pair_count": len(semantic_disabled),
            "respect_robot_semantics": respect_robot_semantics,
            "robot_base_frame_applied": robot_base_frame is not None,
            "evaluated_subsegments": evaluated_subsegments,
            "fk_evaluations": fk_evaluations,
            "max_joint_step": max_joint_step,
            "max_prismatic_step": max_prismatic_step,
            "workers": workers,
            **_collision_timing(
                trajectory_times, validation["timing_supplied"], None, None
            ),
        }
    finally:
        if executor is not None:
            executor.shutdown(wait=True)
        for mesh_id in registered_ids:
            unregister_mesh(mesh_id)


def _combined_model_collision_mesh(model, configuration=None):
    """Combine a robot/tool model's loaded collision geometry in model space."""
    from compas.geometry import Transformation

    transformations = model.compute_transformations(configuration or {})
    combined = None
    for link in model.links:
        parent_joint = link.parent_joint
        link_transformation = (
            transformations[parent_joint.name]
            if parent_joint is not None
            else Transformation()
        )
        for collision in link.collision:
            meshes = list(getattr(collision.geometry.shape, "meshes", []) or [])
            if not meshes:
                raise ValueError(
                    f"collision geometry for model link '{link.name}' is not loaded"
                )
            for source in meshes:
                mesh = source.copy()
                if collision.init_transformation is not None:
                    mesh.transform(collision.init_transformation)
                mesh.transform(link_transformation)
                if combined is None:
                    combined = mesh
                else:
                    combined.join(mesh)
    if combined is None:
        raise ValueError("model has no loaded collision geometry")
    return combined


def _combined_rigid_body_collision_mesh(rigid_body):
    meshes = list(getattr(rigid_body, "collision_meshes", []) or [])
    if not meshes:
        raise ValueError("rigid body has no loaded collision geometry")
    combined = meshes[0].copy()
    for mesh in meshes[1:]:
        combined.join(mesh)
    return combined


def sweep_compas_fab_cell_trajectory(
    robot_cell,
    robot_cell_state,
    trajectory,
    max_joint_step: float = math.radians(5.0),
    max_prismatic_step: float = 0.01,
    max_subdivisions: int = 128,
    parallel: bool = True,
) -> dict:
    """Preflight one trajectory; use PreparedRobotCell to reuse native geometry."""
    from compas_forge.cell import PreparedRobotCell

    _require_valid_compas_fab_trajectory(robot_cell, trajectory)
    with PreparedRobotCell(robot_cell, robot_cell_state) as prepared:
        return prepared.sweep(
            trajectory, max_joint_step, max_prismatic_step,
            max_subdivisions, parallel
        )


def prepare_compas_fab_cell(robot_cell, robot_cell_state):
    """Create an explicit-lifetime snapshot for repeated collision/clearance queries."""
    from compas_forge.cell import PreparedRobotCell

    return PreparedRobotCell(robot_cell, robot_cell_state)


def verify_compas_fab_cell_phases(robot_cell, phases, clearance, **options):
    """Check explicit attachment phases with geometric boundary continuity."""
    from compas_forge.phases import verify_clearance_phases
    return verify_clearance_phases(robot_cell,phases,clearance,**options)


def compute_assembly_contacts_zero_copy(meshes_dict: dict, tolerance: float = 0.005) -> list:
    """
    Estimates coplanar contact interfaces, areas, centroids, and normals
    for static meshes using tolerance-based polygon clipping.
    """
    assembly_list = []
    for name, mesh in meshes_dict.items():
        v, idx, off = compas_mesh_to_buffers(mesh)
        assembly_list.append({
            "name": str(name),
            "vertices": v,
            "indices": idx,
            "offsets": off
        })
        
    result_raw = compute_assembly_contacts(assembly_list, tolerance)
    return json.loads(result_raw)


def assembly_contacts(meshes: dict[str, Mesh], tolerance: float = 0.005) -> list:
    """Estimate coplanar contact interfaces in a named COMPAS mesh assembly."""
    return compute_assembly_contacts_zero_copy(meshes, tolerance)

def check_assembly_clashes(files_map: dict, clearance_tolerance: float = 0.0) -> list:
    items = list(files_map.items())
    result_raw = detect_clashes_json(items, clearance_tolerance)
    return json.loads(result_raw)

def fix_geometry_file(filepath: str) -> dict:
    with open(filepath, 'r', encoding='utf-8') as f:
        raw_data = f.read()
    report_raw = fix_mesh_json(raw_data)
    return json.loads(report_raw)

def run_preflight_profile(filepath: str, profile_name: str) -> dict:
    with open(filepath, 'r', encoding='utf-8') as f:
        raw_data = f.read()
    report_raw = run_preflight_json(raw_data, profile_name)
    return json.loads(report_raw)
