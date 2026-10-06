import json
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
    check_swept_collision,
    compute_assembly_contacts,
    fix_mesh_buffers,
    run_preflight_buffers,
    register_mesh,
    clear_mesh_registry,
    check_swept_collision_cached
)
from compas_forge.reporter import generate_html_report

__version__ = "0.4.0"

# COMPAS plugin discovery metadata
# This variable enables discovery inside strict environments lacking setuptools (e.g. IronPython inside Rhino)
__all_plugins__ = ["compas_forge.plugin"]

__all__ = [
    "validate_compas_json",
    "detect_clashes_json",
    "fix_mesh_json",
    "run_preflight_json",
    "validate_mesh_buffers",
    "check_swept_collision",
    "compute_assembly_contacts",
    "fix_mesh_buffers",
    "run_preflight_buffers",
    "register_mesh",
    "clear_mesh_registry",
    "check_swept_collision_cached",
    "register_mesh_to_cache",
    "clear_mesh_cache",
    "check_swept_collision_cached_poses",
    "pose_from_frame",
    "analyze_mesh",
    "repair_mesh",
    "preflight_mesh",
    "sweep_collision",
    "sweep_compas_fab_trajectory",
    "assembly_contacts",
    "compas_mesh_to_buffers",
    "verify_file",
    "verify_mesh_zero_copy",
    "fix_mesh_zero_copy",
    "run_preflight_profile_zero_copy",
    "check_swept_collision_zero_copy",
    "compute_assembly_contacts_zero_copy",
    "check_assembly_clashes",
    "fix_geometry_file",
    "run_preflight_profile"
]


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
    return register_mesh(str(mesh_id), list(v), list(idx), list(off))

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
    result_raw = check_swept_collision_cached(
        str(mesh1_id), list(pose1_start), list(pose1_end),
        str(mesh2_id), list(pose2_start), list(pose2_end)
    )
    return json.loads(result_raw)

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

def fix_mesh_zero_copy(mesh) -> tuple:
    """
    Welds vertices and aligns adjacent face winding in an owned mesh snapshot.
    Returns a reconstructed mesh and a decoded repair report; inspect remaining defects.
    """
    v_arr, idx_arr, off_arr = compas_mesh_to_buffers(mesh)
    report_raw = fix_mesh_buffers(v_arr, idx_arr, off_arr)
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


def repair_mesh(mesh: Mesh) -> tuple[Mesh, dict]:
    """Repair duplicate vertices and adjacent face winding in a mesh snapshot."""
    return fix_mesh_zero_copy(mesh)

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
    if obstacle_frame is None:
        obstacle_frame = Frame.worldXY()

    options = {"solver": "model"}
    if link is not None:
        options["link"] = link
    frames = [
        robot.forward_kinematics(point, group=group, options=options)
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
            return {
                "has_collision": True,
                "segment_index": index,
                "segment_count": len(points) - 1,
                "segment_fraction": segment_fraction,
                "trajectory_fraction": (index + segment_fraction) / (len(points) - 1),
                "segment_result": segment,
                "frames_evaluated": len(frames),
            }

    return {
        "has_collision": False,
        "segment_index": None,
        "segment_count": len(points) - 1,
        "segment_fraction": None,
        "trajectory_fraction": None,
        "segment_result": None,
        "frames_evaluated": len(frames),
    }

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
