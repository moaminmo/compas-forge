use pyo3::buffer::PyBuffer;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use rstar::RTree;
use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock, RwLock};

mod bounds;
mod clearance;
mod geometry;
mod parser;
mod solid;

use geometry::{
    compute_max_planarity_deviation, compute_mesh_volume, compute_min_face_quality,
    count_degenerate_faces, count_face_components, count_unique_edges, find_boundary_edges,
    find_non_manifold_edges, find_self_intersections, has_consistent_winding, triangulate_face,
    unify_winding_directions, weld_vertices, Aabb, SpatialPart,
};
use parser::{CompasDataObject, ValidationResult};
use rayon::prelude::*;
use std::collections::HashSet;

use parry3d_f64::math::{Pose, Rotation, Vector};
use parry3d_f64::query::{
    cast_shapes, cast_shapes_nonlinear, distance, NonlinearRigidMotion, ShapeCastOptions,
    ShapeCastStatus,
};
use parry3d_f64::shape::TriMesh;

struct RegisteredMesh {
    mesh: TriMesh,
    origin_radius: f64,
    local_bounds: ([f64; 3], [f64; 3]),
    solid_issue: OnceLock<Option<&'static str>>,
    solid_representatives: OnceLock<Vec<usize>>,
}

static MESH_REGISTRY: OnceLock<RwLock<HashMap<String, Arc<RegisteredMesh>>>> = OnceLock::new();
const DEFAULT_WELD_TOLERANCE: f64 = 1e-6;
const DEGENERATE_AREA_TOLERANCE: f64 = 1e-12;

fn get_mesh_registry() -> &'static RwLock<HashMap<String, Arc<RegisteredMesh>>> {
    MESH_REGISTRY.get_or_init(|| RwLock::new(HashMap::new()))
}

#[derive(serde::Serialize)]
struct FixedMeshReport {
    weld_tolerance: f64,
    welded_count: usize,
    flipped_count: usize,
    weld_details: Vec<geometry::WeldAudit>,
    flip_details: Vec<geometry::FlipAudit>,
    fixed_json: String,
}

#[derive(serde::Serialize)]
struct FixedBuffersReport {
    vertices: Vec<f64>,
    face_indices: Vec<i32>,
    face_offsets: Vec<i32>,
    weld_tolerance: f64,
    welded_count: usize,
    flipped_count: usize,
    weld_details: Vec<geometry::WeldAudit>,
    flip_details: Vec<geometry::FlipAudit>,
}

#[derive(serde::Serialize)]
struct PreflightResult {
    profile_name: String,
    is_compliant: bool,
    volume_m3: f64,
    volume_reliable: bool,
    estimated_mass_kg: f64,
    max_mass_kg: f64,
    mass_within_limit: bool,
    requires_watertight: bool,
    boundary_edges_count: usize,
    boundary_edges: Vec<(usize, usize)>,
    is_watertight: bool,
    winding_consistent: bool,
    degenerate_faces_count: usize,
    surface_components: usize,
    isolated_vertices_count: usize,
    self_intersections: Vec<(usize, usize)>,
    fits_workspace: bool,
    bounds_x_dim: f64,
    bounds_y_dim: f64,
    bounds_z_dim: f64,
    bounding_box: crate::geometry::Aabb,
    vertices: Vec<Vec<f64>>,
    triangulated_faces: Vec<[u32; 3]>,
    euler_characteristic: i32,
    genus: Option<usize>,
    max_planarity_deviation: f64,
    min_face_quality: f64,
}

#[derive(serde::Serialize)]
struct AssemblyClashResult {
    part_a: String,
    part_b: String,
    has_intersection: bool,
    relationship: String,
    classification_reliable: bool,
    minimum_distance: f64,
    is_clearance_violation: bool,
}

#[derive(serde::Serialize)]
struct SweptCollisionResult {
    has_collision: bool,
    time_of_impact: Option<f64>,
    method: String,
    substeps: usize,
    impact: Option<SweptImpact>,
}

#[derive(serde::Serialize)]
struct SweptImpact {
    status: String,
    converged: bool,
    conservative: bool,
    geometry_reliable: bool,
    verification_distance: Option<f64>,
    witness_a_local: Vec<f64>,
    witness_b_local: Vec<f64>,
    normal_a_local: Vec<f64>,
    normal_b_local: Vec<f64>,
    witness_a_world: Vec<f64>,
    witness_b_world: Vec<f64>,
    normal_a_world: Vec<f64>,
    normal_b_world: Vec<f64>,
}

fn swept_result_to_python(py: Python<'_>, result: &SweptCollisionResult) -> PyResult<Py<PyDict>> {
    let output = PyDict::new(py);
    output.set_item("has_collision", result.has_collision)?;
    output.set_item("time_of_impact", result.time_of_impact)?;
    output.set_item("method", &result.method)?;
    output.set_item("substeps", result.substeps)?;
    if let Some(impact) = &result.impact {
        let impact_output = PyDict::new(py);
        impact_output.set_item("status", &impact.status)?;
        impact_output.set_item("converged", impact.converged)?;
        impact_output.set_item("conservative", impact.conservative)?;
        impact_output.set_item("geometry_reliable", impact.geometry_reliable)?;
        impact_output.set_item("verification_distance", impact.verification_distance)?;
        impact_output.set_item("witness_a_local", &impact.witness_a_local)?;
        impact_output.set_item("witness_b_local", &impact.witness_b_local)?;
        impact_output.set_item("normal_a_local", &impact.normal_a_local)?;
        impact_output.set_item("normal_b_local", &impact.normal_b_local)?;
        impact_output.set_item("witness_a_world", &impact.witness_a_world)?;
        impact_output.set_item("witness_b_world", &impact.witness_b_world)?;
        impact_output.set_item("normal_a_world", &impact.normal_a_world)?;
        impact_output.set_item("normal_b_world", &impact.normal_b_world)?;
        output.set_item("impact", impact_output)?;
    } else {
        output.set_item("impact", py.None())?;
    }
    Ok(output.unbind())
}

#[derive(serde::Serialize)]
struct ContactInterface {
    block_a: String,
    block_b: String,
    area: f64,
    area_m2: f64,
    area_units: String,
    centroid: [f64; 3],
    normal: [f64; 3],
    normal_alignment: f64,
    maximum_plane_deviation: f64,
    classification_reliable: bool,
    method: String,
    vertices_3d: Vec<[f64; 3]>,
    patches_3d: Vec<Vec<[f64; 3]>>,
}

#[derive(Debug, Clone, Copy)]
struct Point2D {
    x: f64,
    y: f64,
}

fn check_duplicates_parallel(vertices: &[Vec<f64>]) -> usize {
    weld_vertices(vertices, &[], DEFAULT_WELD_TOLERANCE).2.len()
}

fn compute_mesh_distance(part_a: &SpatialPart, part_b: &SpatialPart) -> Result<f64, String> {
    let pts_a: Vec<Vector> = part_a
        .vertices
        .iter()
        .filter(|v| v.len() >= 3)
        .map(|v| Vector::new(v[0], v[1], v[2]))
        .collect();

    let pts_b: Vec<Vector> = part_b
        .vertices
        .iter()
        .filter(|v| v.len() >= 3)
        .map(|v| Vector::new(v[0], v[1], v[2]))
        .collect();

    let mut indices_a = Vec::new();
    for face in &part_a.faces {
        indices_a.extend(triangulate_face(&part_a.vertices, face));
    }

    let mut indices_b = Vec::new();
    for face in &part_b.faces {
        indices_b.extend(triangulate_face(&part_b.vertices, face));
    }

    if indices_a.is_empty() || indices_b.is_empty() {
        return Err("triangulation produced no usable triangles".to_string());
    }

    let mesh_a = TriMesh::new(pts_a, indices_a)
        .map_err(|error| format!("failed to build first TriMesh: {error}"))?;
    let mesh_b = TriMesh::new(pts_b, indices_b)
        .map_err(|error| format!("failed to build second TriMesh: {error}"))?;

    let pos_a = Pose::identity();
    let pos_b = Pose::identity();

    distance(&pos_a, &mesh_a, &pos_b, &mesh_b)
        .map(|result| result.distance)
        .map_err(|error| format!("unsupported mesh distance query: {error:?}"))
}

fn parse_pose(arr: &[f64]) -> PyResult<Pose> {
    if arr.len() != 7 {
        return Err(PyValueError::new_err(
            "Pose array must contain exactly 7 elements: [x, y, z, qx, qy, qz, qw]",
        ));
    }
    if arr.iter().any(|v| !v.is_finite())
        || (arr[3..].iter().map(|v| v * v).sum::<f64>() - 1.0).abs() > 1e-6
    {
        return Err(PyValueError::new_err(
            "Pose must be finite with a unit quaternion",
        ));
    }
    let translation = Vector::new(arr[0], arr[1], arr[2]);
    let rotation = Rotation::from_xyzw(arr[3], arr[4], arr[5], arr[6]);
    Ok(Pose {
        rotation,
        translation,
    })
}

fn validate_geometry(vertices: &[Vec<f64>], faces: &[Vec<usize>]) -> PyResult<()> {
    if vertices.is_empty()
        || vertices
            .iter()
            .any(|v| v.len() != 3 || v.iter().any(|x| !x.is_finite()))
        || faces
            .iter()
            .any(|f| f.len() < 3 || f.iter().any(|&i| i >= vertices.len()))
    {
        return Err(PyValueError::new_err(
            "Invalid mesh coordinates or face indices",
        ));
    }
    Ok(())
}

fn validate_buffers(vertices: &[f64], indices: &[i32], offsets: &[i32]) -> PyResult<()> {
    if !vertices.len().is_multiple_of(3) || vertices.iter().any(|v| !v.is_finite()) {
        return Err(PyValueError::new_err(
            "Vertices must contain finite XYZ triples",
        ));
    }
    if offsets.first() != Some(&0)
        || offsets.last().copied().map(|v| v as usize) != Some(indices.len())
        || offsets
            .windows(2)
            .any(|w| w[0] < 0 || w[1] < w[0] || w[1] - w[0] < 3)
        || indices
            .iter()
            .any(|&i| i < 0 || i as usize >= vertices.len() / 3)
    {
        return Err(PyValueError::new_err(
            "Invalid face offsets or vertex indices",
        ));
    }
    Ok(())
}

/// Perform a structural mesh scan directly over borrowed Python buffers.
///
/// This deliberately keeps the GIL: the buffers remain owned by Python for the
/// duration of the scan. Unlike mesh registration and full topology analysis,
/// this hot path does not materialise Rust `Vec`s or retain any borrowed data.
#[pyfunction]
fn scan_mesh_buffers_borrowed(
    py: Python<'_>,
    vertices_obj: &Bound<'_, PyAny>,
    face_indices_obj: &Bound<'_, PyAny>,
    face_offsets_obj: &Bound<'_, PyAny>,
) -> PyResult<Py<PyDict>> {
    let vertices = PyBuffer::<f64>::get(vertices_obj)
        .map_err(|error| PyValueError::new_err(format!("vertices buffer: {error}")))?;
    let indices = PyBuffer::<i32>::get(face_indices_obj)
        .map_err(|error| PyValueError::new_err(format!("indices buffer: {error}")))?;
    let offsets = PyBuffer::<i32>::get(face_offsets_obj)
        .map_err(|error| PyValueError::new_err(format!("offsets buffer: {error}")))?;

    let vertices = vertices
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("vertices buffer must be C-contiguous"))?;
    let indices = indices
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("indices buffer must be C-contiguous"))?;
    let offsets = offsets
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("offsets buffer must be C-contiguous"))?;

    let xyz_aligned = vertices.len().is_multiple_of(3);
    let vertex_count = vertices.len() / 3;
    let mut finite_coordinates = true;
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for (position, value) in vertices.iter().enumerate() {
        let value = value.get();
        if !value.is_finite() {
            finite_coordinates = false;
            continue;
        }
        let axis = position % 3;
        min[axis] = min[axis].min(value);
        max[axis] = max[axis].max(value);
    }

    let mut invalid_index_count = 0usize;
    for value in indices.iter() {
        let value = value.get();
        if value < 0 || value as usize >= vertex_count {
            invalid_index_count += 1;
        }
    }

    let mut offsets_valid = offsets.len() >= 2 && offsets[0].get() == 0;
    let mut undersized_face_count = 0usize;
    let mut previous = 0i32;
    for (position, value) in offsets.iter().enumerate() {
        let value = value.get();
        if value < 0 || (position > 0 && value < previous) {
            offsets_valid = false;
        }
        if position > 0 && value >= previous && value - previous < 3 {
            undersized_face_count += 1;
        }
        previous = value;
    }
    if offsets.last().map(|value| value.get() as usize) != Some(indices.len()) {
        offsets_valid = false;
    }

    let is_structurally_valid = xyz_aligned
        && finite_coordinates
        && offsets_valid
        && invalid_index_count == 0
        && undersized_face_count == 0;
    let output = PyDict::new(py);
    output.set_item("is_structurally_valid", is_structurally_valid)?;
    output.set_item("borrowed_input", true)?;
    output.set_item("owned_input_copy_bytes", 0usize)?;
    output.set_item("vertex_count", vertex_count)?;
    output.set_item("face_count", offsets.len().saturating_sub(1))?;
    output.set_item("index_count", indices.len())?;
    output.set_item(
        "input_bytes",
        vertices.len() * 8 + indices.len() * 4 + offsets.len() * 4,
    )?;
    output.set_item("xyz_aligned", xyz_aligned)?;
    output.set_item("finite_coordinates", finite_coordinates)?;
    output.set_item("offsets_valid", offsets_valid)?;
    output.set_item("invalid_index_count", invalid_index_count)?;
    output.set_item("undersized_face_count", undersized_face_count)?;
    if vertex_count > 0 && finite_coordinates && xyz_aligned {
        output.set_item("bounds_min", min)?;
        output.set_item("bounds_max", max)?;
    } else {
        output.set_item("bounds_min", py.None())?;
        output.set_item("bounds_max", py.None())?;
    }
    Ok(output.unbind())
}

/// Evaluate only the topology predicates needed by COMPAS Mesh methods.
///
/// Coordinates, triangulation, self-intersection and volume are intentionally
/// excluded so this remains a fair accelerator for `is_closed/is_manifold`.
#[pyfunction]
fn topology_predicates_buffers(
    py: Python<'_>,
    vertex_count: usize,
    face_indices_obj: &Bound<'_, PyAny>,
    face_offsets_obj: &Bound<'_, PyAny>,
) -> PyResult<Py<PyDict>> {
    let indices = PyBuffer::<i32>::get(face_indices_obj)
        .map_err(|error| PyValueError::new_err(format!("indices buffer: {error}")))?;
    let offsets = PyBuffer::<i32>::get(face_offsets_obj)
        .map_err(|error| PyValueError::new_err(format!("offsets buffer: {error}")))?;
    let indices = indices
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("indices buffer must be C-contiguous"))?;
    let offsets = offsets
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("offsets buffer must be C-contiguous"))?;
    if offsets.first().map(|value| value.get()) != Some(0)
        || offsets.last().map(|value| value.get() as usize) != Some(indices.len())
        || offsets.windows(2).any(|window| {
            let start = window[0].get();
            let end = window[1].get();
            start < 0 || end < start || end - start < 3
        })
        || indices
            .iter()
            .any(|index| index.get() < 0 || index.get() as usize >= vertex_count)
    {
        return Err(PyValueError::new_err(
            "invalid topology offsets or vertex indices",
        ));
    }

    let mut edge_counts: HashMap<(usize, usize), usize> = HashMap::new();
    let mut vertex_links = vec![Vec::<(usize, usize)>::new(); vertex_count];
    for bounds in offsets.windows(2) {
        let start = bounds[0].get() as usize;
        let end = bounds[1].get() as usize;
        let face = &indices[start..end];
        for position in 0..face.len() {
            let current = face[position].get() as usize;
            let previous = face[(position + face.len() - 1) % face.len()].get() as usize;
            let next = face[(position + 1) % face.len()].get() as usize;
            vertex_links[current].push((previous, next));
            let edge = if current < next {
                (current, next)
            } else {
                (next, current)
            };
            *edge_counts.entry(edge).or_insert(0) += 1;
        }
    }

    let boundary_edges_count = edge_counts.values().filter(|&&count| count == 1).count();
    let mut non_manifold_edges: Vec<_> = edge_counts
        .into_iter()
        .filter_map(|(edge, count)| (count > 2).then_some(edge))
        .collect();
    non_manifold_edges.sort_unstable();
    let isolated_vertices_count = vertex_links.iter().filter(|links| links.is_empty()).count();
    let mut non_manifold_vertices = Vec::new();
    for (vertex, links) in vertex_links.iter().enumerate() {
        if links.is_empty() {
            continue;
        }
        let mut adjacency: HashMap<usize, Vec<usize>> = HashMap::new();
        for &(a, b) in links {
            adjacency.entry(a).or_default().push(b);
            adjacency.entry(b).or_default().push(a);
        }
        let endpoints = adjacency
            .values()
            .filter(|neighbors| neighbors.len() == 1)
            .count();
        let mut seen = HashSet::new();
        let mut stack = vec![*adjacency.keys().next().unwrap()];
        while let Some(neighbor) = stack.pop() {
            if seen.insert(neighbor) {
                stack.extend(&adjacency[&neighbor]);
            }
        }
        if seen.len() != adjacency.len()
            || adjacency.values().any(|neighbors| neighbors.len() > 2)
            || (endpoints != 0 && endpoints != 2)
        {
            non_manifold_vertices.push(vertex);
        }
    }

    let output = PyDict::new(py);
    output.set_item("is_closed", vertex_count > 0 && boundary_edges_count == 0)?;
    output.set_item(
        "is_manifold",
        vertex_count > 0
            && isolated_vertices_count == 0
            && non_manifold_edges.is_empty()
            && non_manifold_vertices.is_empty(),
    )?;
    output.set_item("boundary_edges_count", boundary_edges_count)?;
    output.set_item("non_manifold_edges", non_manifold_edges)?;
    output.set_item("non_manifold_vertices", non_manifold_vertices)?;
    output.set_item("isolated_vertices_count", isolated_vertices_count)?;
    Ok(output.unbind())
}

struct MeshTopologyMetrics {
    boundary_edges: Vec<(usize, usize)>,
    is_watertight: bool,
    winding_consistent: bool,
    degenerate_faces_count: usize,
    surface_components: usize,
    isolated_vertices_count: usize,
    self_intersections: Vec<(usize, usize)>,
    euler_characteristic: i32,
    genus: Option<usize>,
    volume_reliable: bool,
}

fn mesh_topology_metrics(vertices: &[Vec<f64>], faces: &[Vec<usize>]) -> MeshTopologyMetrics {
    let boundary_edges = find_boundary_edges(faces);
    let non_manifold_edges = find_non_manifold_edges(faces);
    let non_manifold_vertices = geometry::find_non_manifold_vertices(faces);
    let degenerate_faces_count = count_degenerate_faces(vertices, faces, DEGENERATE_AREA_TOLERANCE);
    let winding_consistent = has_consistent_winding(faces);
    let self_intersections = find_self_intersections(vertices, faces);
    let surface_components = count_face_components(faces);
    let used_vertices: HashSet<usize> = faces.iter().flatten().copied().collect();
    let isolated_vertices_count = vertices.len().saturating_sub(used_vertices.len());
    let euler_characteristic =
        used_vertices.len() as i32 - count_unique_edges(faces) as i32 + faces.len() as i32;
    let is_watertight = !faces.is_empty()
        && boundary_edges.is_empty()
        && non_manifold_edges.is_empty()
        && non_manifold_vertices.is_empty()
        && degenerate_faces_count == 0;
    let genus_numerator = 2 * surface_components as i32 - euler_characteristic;
    let genus = if is_watertight
        && winding_consistent
        && genus_numerator >= 0
        && genus_numerator % 2 == 0
    {
        Some((genus_numerator / 2) as usize)
    } else {
        None
    };

    MeshTopologyMetrics {
        boundary_edges,
        is_watertight,
        winding_consistent,
        degenerate_faces_count,
        surface_components,
        isolated_vertices_count,
        euler_characteristic,
        genus,
        volume_reliable: is_watertight && winding_consistent && self_intersections.is_empty(),
        self_intersections,
    }
}

fn trimesh_from_buffers(
    vertices_flat: &[f64],
    face_indices: &[i32],
    face_offsets: &[i32],
) -> PyResult<TriMesh> {
    validate_buffers(vertices_flat, face_indices, face_offsets)?;
    let vertex_count = vertices_flat.len() / 3;
    let mut vertices = Vec::with_capacity(vertex_count);
    for chunk in vertices_flat.as_chunks::<3>().0 {
        vertices.push(Vector::new(chunk[0], chunk[1], chunk[2]));
    }

    let face_count = if face_offsets.is_empty() {
        0
    } else {
        face_offsets.len() - 1
    };
    let mut indices = Vec::with_capacity(face_count);
    for i in 0..face_count {
        let start = face_offsets[i] as usize;
        let end = face_offsets[i + 1] as usize;
        let mut face_verts = Vec::with_capacity(end - start);
        for &idx in &face_indices[start..end] {
            face_verts.push(idx as usize);
        }
        let tris = triangulate_face(&vertices, &face_verts);
        for tri in tris {
            indices.push(tri);
        }
    }

    TriMesh::new(vertices, indices)
        .map_err(|e| PyValueError::new_err(format!("Failed to build TriMesh structure: {}", e)))
}

fn rigid_motion(start: &Pose, end: &Pose) -> NonlinearRigidMotion {
    let mut delta = (end.rotation * start.rotation.conjugate()).normalize();
    if delta.w < 0.0 {
        delta = -delta;
    }
    NonlinearRigidMotion::new(
        *start,
        Vector::ZERO,
        end.translation - start.translation,
        delta.to_scaled_axis(),
    )
}

fn swept_origin_sphere_bounds(start: &Pose, end: &Pose, radius: f64) -> ([f64; 3], [f64; 3]) {
    let start = start.translation.to_array();
    let end = end.translation.to_array();
    let mut mins = [0.0; 3];
    let mut maxs = [0.0; 3];
    for axis in 0..3 {
        mins[axis] = start[axis].min(end[axis]) - radius;
        maxs[axis] = start[axis].max(end[axis]) + radius;
    }
    (mins, maxs)
}

fn sweep_registered_meshes(
    a: &RegisteredMesh,
    a0: &Pose,
    a1: &Pose,
    b: &RegisteredMesh,
    b0: &Pose,
    b1: &Pose,
    tight_bounds: bool,
) -> PyResult<SweptCollisionResult> {
    // Every point of a rigid mesh remains inside a sphere centred at its pose
    // translation.  The union of that sphere along the linearly interpolated
    // translation is a conservative swept AABB even while the mesh rotates.
    let (a_min, a_max) = swept_origin_sphere_bounds(a0, a1, a.origin_radius);
    let (b_min, b_max) = swept_origin_sphere_bounds(b0, b1, b.origin_radius);
    let separated = (0..3).any(|axis| a_max[axis] < b_min[axis] || b_max[axis] < a_min[axis]);
    if separated {
        return Ok(SweptCollisionResult {
            has_collision: false,
            time_of_impact: None,
            method: "swept_sphere_aabb_rejected".to_string(),
            substeps: 0,
            impact: None,
        });
    }
    if !tight_bounds {
        return sweep_meshes(&a.mesh, a0, a1, &b.mesh, b0, b1);
    }
    let (a_min, a_max) = bounds::swept_endpoint_bounds(a, a0, a1);
    let (b_min, b_max) = bounds::swept_endpoint_bounds(b, b0, b1);
    if tight_bounds && (0..3).any(|axis| a_max[axis] < b_min[axis] || b_max[axis] < a_min[axis]) {
        return Ok(SweptCollisionResult {
            has_collision: false,
            time_of_impact: None,
            method: "swept_endpoint_aabb_rejected".to_string(),
            substeps: 0,
            impact: None,
        });
    }
    sweep_meshes(&a.mesh, a0, a1, &b.mesh, b0, b1)
}

fn sweep_meshes(
    a: &TriMesh,
    a0: &Pose,
    a1: &Pose,
    b: &TriMesh,
    b0: &Pose,
    b1: &Pose,
) -> PyResult<SweptCollisionResult> {
    let motion_a = rigid_motion(a0, a1);
    let motion_b = rigid_motion(b0, b1);

    let rotation_a = (a1.rotation * a0.rotation.conjugate())
        .to_scaled_axis()
        .length();
    let rotation_b = (b1.rotation * b0.rotation.conjugate())
        .to_scaled_axis()
        .length();
    let max_rotation = rotation_a.max(rotation_b);
    let initial_distance = distance(a0, a, b0, b)
        .map_err(|e| PyValueError::new_err(format!("Unsupported initial distance query: {e:?}")))?;

    let (method, substeps, result) = if initial_distance.distance <= 1e-12 {
        let hit =
            cast_shapes_nonlinear(&motion_a, a, &motion_b, b, 0.0, 1.0, true).map_err(|e| {
                PyValueError::new_err(format!("Unsupported initial-overlap query: {e:?}"))
            })?;
        ("initial_overlap".to_string(), 1, hit)
    } else if max_rotation <= 1e-10 {
        let velocity_a_world = a1.translation - a0.translation;
        let velocity_b_world = b1.translation - b0.translation;
        // Parry's public shape-cast API accepts both velocities in world space and
        // transforms their relative velocity into the first shape's frame internally.
        let options = ShapeCastOptions::with_max_time_of_impact(1.0);
        let hit = cast_shapes(a0, velocity_a_world, a, b0, velocity_b_world, b, options)
            .map_err(|e| PyValueError::new_err(format!("Unsupported linear shape cast: {e:?}")))?;
        ("linear".to_string(), 1, hit)
    } else {
        // Keep each angular interval at or below 1.40625 degrees. Smaller intervals improve
        // convergence for mesh-vs-mesh nonlinear casts while preserving the exact rigid motion.
        let substeps = (max_rotation / (std::f64::consts::PI / 128.0))
            .ceil()
            .clamp(1.0, 256.0) as usize;
        let mut first_hit = None;
        for step in 0..substeps {
            let start_time = step as f64 / substeps as f64;
            let end_time = (step + 1) as f64 / substeps as f64;
            let hit = cast_shapes_nonlinear(&motion_a, a, &motion_b, b, start_time, end_time, true)
                .map_err(|e| {
                    PyValueError::new_err(format!("Unsupported nonlinear shape cast: {e:?}"))
                })?;
            if hit.is_some() {
                first_hit = hit;
                break;
            }
        }
        ("nonlinear_substepped".to_string(), substeps, first_hit)
    };

    Ok(match result {
        Some(hit) => {
            let pose_a_at_impact = motion_a.position_at_time(hit.time_of_impact);
            let pose_b_at_impact = motion_b.position_at_time(hit.time_of_impact);
            let converged = hit.status == ShapeCastStatus::Converged;
            let conservative = matches!(
                hit.status,
                ShapeCastStatus::Failed | ShapeCastStatus::OutOfIterations
            );
            let verification_distance = distance(&pose_a_at_impact, a, &pose_b_at_impact, b)
                .ok()
                .map(|result| result.distance);
            let geometry_reliable = converged
                && verification_distance
                    .is_some_and(|residual| residual.is_finite() && residual <= 1e-8);
            SweptCollisionResult {
                has_collision: true,
                time_of_impact: Some(hit.time_of_impact),
                method,
                substeps,
                impact: Some(SweptImpact {
                    status: format!("{:?}", hit.status),
                    converged,
                    conservative,
                    // Deliberately strict: require both solver convergence and an independent
                    // distance residual at the reported impact poses.
                    geometry_reliable,
                    verification_distance,
                    witness_a_local: hit.witness1.to_array().to_vec(),
                    witness_b_local: hit.witness2.to_array().to_vec(),
                    normal_a_local: hit.normal1.to_array().to_vec(),
                    normal_b_local: hit.normal2.to_array().to_vec(),
                    witness_a_world: (pose_a_at_impact.rotation.mul_vec3(hit.witness1)
                        + pose_a_at_impact.translation)
                        .to_array()
                        .to_vec(),
                    witness_b_world: (pose_b_at_impact.rotation.mul_vec3(hit.witness2)
                        + pose_b_at_impact.translation)
                        .to_array()
                        .to_vec(),
                    normal_a_world: pose_a_at_impact
                        .rotation
                        .mul_vec3(hit.normal1)
                        .to_array()
                        .to_vec(),
                    normal_b_world: pose_b_at_impact
                        .rotation
                        .mul_vec3(hit.normal2)
                        .to_array()
                        .to_vec(),
                }),
            }
        }
        None => SweptCollisionResult {
            has_collision: false,
            time_of_impact: None,
            method,
            substeps,
            impact: None,
        },
    })
}

fn is_inside(p: Point2D, cp1: Point2D, cp2: Point2D) -> bool {
    (cp2.x - cp1.x) * (p.y - cp1.y) - (cp2.y - cp1.y) * (p.x - cp1.x) >= -1e-9
}

fn intersection_point(s: Point2D, p: Point2D, cp1: Point2D, cp2: Point2D) -> Option<Point2D> {
    let dc = Point2D {
        x: cp1.x - cp2.x,
        y: cp1.y - cp2.y,
    };
    let dp = Point2D {
        x: s.x - p.x,
        y: s.y - p.y,
    };
    let n1 = cp1.x * cp2.y - cp1.y * cp2.x;
    let n2 = s.x * p.y - s.y * p.x;
    let num = n1 * dp.x - dc.x * n2;
    let den = dc.x * dp.y - dc.y * dp.x;
    if den.abs() < 1e-12 {
        None
    } else {
        Some(Point2D {
            x: num / den,
            y: (n1 * dp.y - dc.y * n2) / den,
        })
    }
}

fn clip_polygon(subject: &[Point2D], clip: &[Point2D]) -> Vec<Point2D> {
    let mut output = subject.to_vec();
    let len = clip.len();
    if len < 3 {
        return Vec::new();
    }

    for i in 0..len {
        let cp1 = clip[i];
        let cp2 = clip[(i + 1) % len];
        let input = output;
        output = Vec::new();
        if input.is_empty() {
            break;
        }

        let mut s = input[input.len() - 1];
        for &p in &input {
            if is_inside(p, cp1, cp2) {
                if !is_inside(s, cp1, cp2) {
                    if let Some(intersection) = intersection_point(s, p, cp1, cp2) {
                        output.push(intersection);
                    }
                }
                output.push(p);
            } else if is_inside(s, cp1, cp2) {
                if let Some(intersection) = intersection_point(s, p, cp1, cp2) {
                    output.push(intersection);
                }
            }
            s = p;
        }
    }
    output
}

fn polygon_area_and_centroid(poly: &[Point2D]) -> (f64, Point2D) {
    let n = poly.len();
    if n < 3 {
        return (0.0, Point2D { x: 0.0, y: 0.0 });
    }
    let mut area = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;
    for i in 0..n {
        let p1 = poly[i];
        let p2 = poly[(i + 1) % n];
        let factor = p1.x * p2.y - p2.x * p1.y;
        area += factor;
        cx += (p1.x + p2.x) * factor;
        cy += (p1.y + p2.y) * factor;
    }
    area /= 2.0;
    if area.abs() < 1e-9 {
        (0.0, Point2D { x: 0.0, y: 0.0 })
    } else {
        let area_abs = area.abs();
        let factor = 6.0 * area;
        (
            area_abs,
            Point2D {
                x: cx / factor,
                y: cy / factor,
            },
        )
    }
}

fn ensure_counterclockwise(mut polygon: Vec<Point2D>) -> Vec<Point2D> {
    let signed_twice_area: f64 = polygon
        .iter()
        .zip(polygon.iter().cycle().skip(1))
        .take(polygon.len())
        .map(|(a, b)| a.x * b.y - b.x * a.y)
        .sum();
    if signed_twice_area < 0.0 {
        polygon.reverse();
    }
    polygon
}

fn maximum_plane_distance(
    vertices: &[Vector],
    face: &[usize],
    plane_origin: Vector,
    plane_normal: Vector,
) -> f64 {
    face.iter()
        .map(|&index| ((vertices[index] - plane_origin).dot(plane_normal)).abs())
        .fold(0.0, f64::max)
}

fn intersect_projected_faces(
    vertices_a: &[Vector],
    face_a: &[usize],
    vertices_b: &[Vector],
    face_b: &[usize],
    origin: Vector,
    u: Vector,
    v: Vector,
) -> (f64, Point2D, Vec<Vec<Point2D>>) {
    let triangles_a = triangulate_face(vertices_a, face_a);
    let triangles_b = triangulate_face(vertices_b, face_b);
    let project = |point: Vector| {
        let delta = point - origin;
        Point2D {
            x: delta.dot(u),
            y: delta.dot(v),
        }
    };
    let mut total_area = 0.0;
    let mut weighted_x = 0.0;
    let mut weighted_y = 0.0;
    let mut patches = Vec::new();

    for triangle_a in &triangles_a {
        let clip = ensure_counterclockwise(
            triangle_a
                .iter()
                .map(|&index| project(vertices_a[index as usize]))
                .collect(),
        );
        for triangle_b in &triangles_b {
            let subject = ensure_counterclockwise(
                triangle_b
                    .iter()
                    .map(|&index| project(vertices_b[index as usize]))
                    .collect(),
            );
            let patch = clip_polygon(&subject, &clip);
            let (area, centroid) = polygon_area_and_centroid(&patch);
            if area > 0.0 {
                total_area += area;
                weighted_x += centroid.x * area;
                weighted_y += centroid.y * area;
                patches.push(patch);
            }
        }
    }

    let centroid = if total_area > 0.0 {
        Point2D {
            x: weighted_x / total_area,
            y: weighted_y / total_area,
        }
    } else {
        Point2D { x: 0.0, y: 0.0 }
    };
    (total_area, centroid, patches)
}

fn calculate_face_normal_and_centroid(vertices: &[Vector], face: &[usize]) -> (Vector, Vector) {
    let len = face.len();
    let mut centroid = Vector::new(0.0, 0.0, 0.0);
    for &idx in face {
        centroid += vertices[idx];
    }
    centroid /= len as f64;

    let mut normal = Vector::new(0.0, 0.0, 0.0);
    for i in 0..len {
        let vi = vertices[face[i]];
        let vj = vertices[face[(i + 1) % len]];
        normal.x += (vi.y - vj.y) * (vi.z + vj.z);
        normal.y += (vi.z - vj.z) * (vi.x + vj.x);
        normal.z += (vi.x - vj.x) * (vi.y + vj.y);
    }
    if normal.length_squared() > 1e-12 {
        normal = normal.normalize();
    }
    (normal, centroid)
}

#[pyfunction]
fn validate_compas_json(json_str: &str) -> PyResult<String> {
    let mut bytes = json_str.as_bytes().to_vec();

    let obj: CompasDataObject = simd_json::serde::from_slice(&mut bytes).map_err(|err| {
        PyValueError::new_err(format!("Malformed COMPAS JSON Schema (SIMD): {}", err))
    })?;

    let (vertices, faces) = obj
        .data
        .get_vertices_and_faces()
        .map_err(PyValueError::new_err)?;
    validate_geometry(&vertices, &faces)?;

    let duplicate_count = check_duplicates_parallel(&vertices);
    let non_manifold = find_non_manifold_edges(&faces);
    let non_manifold_vertices = geometry::find_non_manifold_vertices(&faces);
    let degenerate_faces_count =
        count_degenerate_faces(&vertices, &faces, DEGENERATE_AREA_TOLERANCE);
    let winding_consistent = has_consistent_winding(&faces);
    let self_intersections = find_self_intersections(&vertices, &faces);
    let bbox = Aabb::from_vertices(&vertices);

    let result = ValidationResult {
        is_valid: duplicate_count == 0
            && non_manifold.is_empty()
            && non_manifold_vertices.is_empty()
            && degenerate_faces_count == 0
            && self_intersections.is_empty(),
        vertex_count: vertices.len(),
        face_count: faces.len(),
        non_manifold_edges: non_manifold,
        non_manifold_vertices,
        duplicate_vertices: duplicate_count,
        duplicate_tolerance: DEFAULT_WELD_TOLERANCE,
        degenerate_faces_count,
        winding_consistent,
        self_intersections,
        boundary_edges_count: find_boundary_edges(&faces).len(),
        bounding_box: bbox,
    };

    serde_json::to_string(&result).map_err(|err| {
        PyValueError::new_err(format!("Failed to serialize diagnostic report: {}", err))
    })
}

#[pyfunction]
fn validate_mesh_buffers(
    py: Python<'_>,
    vertices: &Bound<'_, PyAny>,
    face_indices: &Bound<'_, PyAny>,
    face_offsets: &Bound<'_, PyAny>,
) -> PyResult<String> {
    let vertices =
        PyBuffer::<f64>::get(vertices).map_err(|e| PyValueError::new_err(e.to_string()))?;
    let face_indices =
        PyBuffer::<i32>::get(face_indices).map_err(|e| PyValueError::new_err(e.to_string()))?;
    let face_offsets =
        PyBuffer::<i32>::get(face_offsets).map_err(|e| PyValueError::new_err(e.to_string()))?;
    let v_slice = vertices
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("Vertices buffer must be contiguous and flat"))?;
    let idx_slice = face_indices
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("Face indices buffer must be contiguous and flat"))?;
    let off_slice = face_offsets
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("Face offsets buffer must be contiguous and flat"))?;

    let vertices_flat_owned: Vec<f64> = v_slice.iter().map(|cell| cell.get()).collect();
    let vertices_flat: &[f64] = &vertices_flat_owned;
    let face_indices_owned: Vec<i32> = idx_slice.iter().map(|cell| cell.get()).collect();
    let face_indices: &[i32] = &face_indices_owned;
    let face_offsets_owned: Vec<i32> = off_slice.iter().map(|cell| cell.get()).collect();
    let face_offsets: &[i32] = &face_offsets_owned;

    validate_buffers(vertices_flat, face_indices, face_offsets)?;
    let vertex_count = vertices_flat.len() / 3;
    let mut vertices = Vec::with_capacity(vertex_count);
    for chunk in vertices_flat.as_chunks::<3>().0 {
        vertices.push(vec![chunk[0], chunk[1], chunk[2]]);
    }

    let face_count = if face_offsets.is_empty() {
        0
    } else {
        face_offsets.len() - 1
    };
    let mut faces = Vec::with_capacity(face_count);
    for i in 0..face_count {
        let start = face_offsets[i] as usize;
        let end = face_offsets[i + 1] as usize;
        let mut face = Vec::with_capacity(end - start);
        for &idx in &face_indices[start..end] {
            face.push(idx as usize);
        }
        faces.push(face);
    }

    let duplicate_count = check_duplicates_parallel(&vertices);
    let non_manifold = find_non_manifold_edges(&faces);
    let non_manifold_vertices = geometry::find_non_manifold_vertices(&faces);
    let degenerate_faces_count =
        count_degenerate_faces(&vertices, &faces, DEGENERATE_AREA_TOLERANCE);
    let winding_consistent = has_consistent_winding(&faces);
    let self_intersections = find_self_intersections(&vertices, &faces);
    let bbox = Aabb::from_vertices(&vertices);

    let result = ValidationResult {
        is_valid: duplicate_count == 0
            && non_manifold.is_empty()
            && non_manifold_vertices.is_empty()
            && degenerate_faces_count == 0
            && self_intersections.is_empty(),
        vertex_count,
        face_count,
        non_manifold_edges: non_manifold,
        non_manifold_vertices,
        duplicate_vertices: duplicate_count,
        duplicate_tolerance: DEFAULT_WELD_TOLERANCE,
        degenerate_faces_count,
        winding_consistent,
        self_intersections,
        boundary_edges_count: find_boundary_edges(&faces).len(),
        bounding_box: bbox,
    };

    serde_json::to_string(&result).map_err(|err| {
        PyValueError::new_err(format!(
            "Failed to serialize zero-copy diagnostic report: {}",
            err
        ))
    })
}

#[pyfunction]
#[allow(clippy::too_many_arguments)]
fn check_swept_collision(
    py: Python<'_>,
    v1_obj: &Bound<'_, PyAny>,
    idx1_obj: &Bound<'_, PyAny>,
    off1_obj: &Bound<'_, PyAny>,
    pose1_start_vec: Vec<f64>,
    pose1_end_vec: Vec<f64>,
    v2_obj: &Bound<'_, PyAny>,
    idx2_obj: &Bound<'_, PyAny>,
    off2_obj: &Bound<'_, PyAny>,
    pose2_start_vec: Vec<f64>,
    pose2_end_vec: Vec<f64>,
) -> PyResult<String> {
    let v1_buf = PyBuffer::<f64>::get(v1_obj)
        .map_err(|e| PyValueError::new_err(format!("v1 buffer err: {}", e)))?;
    let idx1_buf = PyBuffer::<i32>::get(idx1_obj)
        .map_err(|e| PyValueError::new_err(format!("idx1 buffer err: {}", e)))?;
    let off1_buf = PyBuffer::<i32>::get(off1_obj)
        .map_err(|e| PyValueError::new_err(format!("off1 buffer err: {}", e)))?;

    let v2_buf = PyBuffer::<f64>::get(v2_obj)
        .map_err(|e| PyValueError::new_err(format!("v2 buffer err: {}", e)))?;
    let idx2_buf = PyBuffer::<i32>::get(idx2_obj)
        .map_err(|e| PyValueError::new_err(format!("idx2 buffer err: {}", e)))?;
    let off2_buf = PyBuffer::<i32>::get(off2_obj)
        .map_err(|e| PyValueError::new_err(format!("off2 buffer err: {}", e)))?;

    let v1_slice = v1_buf
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("v1 not contiguous"))?;
    let idx1_slice = idx1_buf
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("idx1 not contiguous"))?;
    let off1_slice = off1_buf
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("off1 not contiguous"))?;

    let v2_slice = v2_buf
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("v2 not contiguous"))?;
    let idx2_slice = idx2_buf
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("idx2 not contiguous"))?;
    let off2_slice = off2_buf
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("off2 not contiguous"))?;

    let v1_flat_owned: Vec<f64> = v1_slice.iter().map(|cell| cell.get()).collect();
    let v1_flat: &[f64] = &v1_flat_owned;
    let idx1_owned: Vec<i32> = idx1_slice.iter().map(|cell| cell.get()).collect();
    let idx1: &[i32] = &idx1_owned;
    let off1_owned: Vec<i32> = off1_slice.iter().map(|cell| cell.get()).collect();
    let off1: &[i32] = &off1_owned;

    let v2_flat_owned: Vec<f64> = v2_slice.iter().map(|cell| cell.get()).collect();
    let v2_flat: &[f64] = &v2_flat_owned;
    let idx2_owned: Vec<i32> = idx2_slice.iter().map(|cell| cell.get()).collect();
    let idx2: &[i32] = &idx2_owned;
    let off2_owned: Vec<i32> = off2_slice.iter().map(|cell| cell.get()).collect();
    let off2: &[i32] = &off2_owned;

    let p1_start = parse_pose(&pose1_start_vec)?;
    let p1_end = parse_pose(&pose1_end_vec)?;
    let p2_start = parse_pose(&pose2_start_vec)?;
    let p2_end = parse_pose(&pose2_end_vec)?;

    let mesh1 = trimesh_from_buffers(v1_flat, idx1, off1)?;
    let mesh2 = trimesh_from_buffers(v2_flat, idx2, off2)?;

    let res = py.detach(|| sweep_meshes(&mesh1, &p1_start, &p1_end, &mesh2, &p2_start, &p2_end))?;

    serde_json::to_string(&res)
        .map_err(|e| PyValueError::new_err(format!("Serialization error: {}", e)))
}

#[pyfunction]
fn register_mesh(
    mesh_id: String,
    vertices_flat: Vec<f64>,
    face_indices: Vec<i32>,
    face_offsets: Vec<i32>,
) -> PyResult<String> {
    let trimesh = trimesh_from_buffers(&vertices_flat, &face_indices, &face_offsets)?;
    register_trimesh(mesh_id, trimesh)
}

fn register_trimesh(mesh_id: String, trimesh: TriMesh) -> PyResult<String> {
    let origin_radius = trimesh
        .vertices()
        .iter()
        .map(|vertex| vertex.length())
        .fold(0.0, f64::max);
    let mut mins = [f64::INFINITY; 3];
    let mut maxs = [f64::NEG_INFINITY; 3];
    for v in trimesh.vertices() {
        for i in 0..3 {
            mins[i] = mins[i].min(v[i]);
            maxs[i] = maxs[i].max(v[i]);
        }
    }
    let registry = get_mesh_registry();
    let mut guard = registry.write().map_err(|e| {
        PyValueError::new_err(format!("Failed to acquire mesh registry lock: {}", e))
    })?;
    guard.insert(
        mesh_id.clone(),
        Arc::new(RegisteredMesh {
            mesh: trimesh,
            origin_radius,
            local_bounds: (mins, maxs),
            solid_issue: OnceLock::new(),
            solid_representatives: OnceLock::new(),
        }),
    );
    Ok(format!("Mesh '{}' successfully registered.", mesh_id))
}

#[pyfunction]
fn register_mesh_buffers(
    py: Python<'_>,
    mesh_id: String,
    vertices_obj: &Bound<'_, PyAny>,
    face_indices_obj: &Bound<'_, PyAny>,
    face_offsets_obj: &Bound<'_, PyAny>,
) -> PyResult<String> {
    let vertices = PyBuffer::<f64>::get(vertices_obj)
        .map_err(|error| PyValueError::new_err(format!("vertices buffer: {error}")))?;
    let indices = PyBuffer::<i32>::get(face_indices_obj)
        .map_err(|error| PyValueError::new_err(format!("indices buffer: {error}")))?;
    let offsets = PyBuffer::<i32>::get(face_offsets_obj)
        .map_err(|error| PyValueError::new_err(format!("offsets buffer: {error}")))?;
    let vertices = vertices
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("vertices buffer must be contiguous"))?
        .iter()
        .map(|value| value.get())
        .collect::<Vec<_>>();
    let indices = indices
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("indices buffer must be contiguous"))?
        .iter()
        .map(|value| value.get())
        .collect::<Vec<_>>();
    let offsets = offsets
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("offsets buffer must be contiguous"))?
        .iter()
        .map(|value| value.get())
        .collect::<Vec<_>>();
    let trimesh = trimesh_from_buffers(&vertices, &indices, &offsets)?;
    register_trimesh(mesh_id, trimesh)
}

#[pyfunction]
fn clear_mesh_registry() -> PyResult<String> {
    let registry = get_mesh_registry();
    let mut guard = registry.write().map_err(|e| {
        PyValueError::new_err(format!("Failed to acquire mesh registry lock: {}", e))
    })?;
    guard.clear();
    Ok("Mesh registry cleared successfully.".to_string())
}

#[pyfunction]
fn unregister_mesh(mesh_id: &str) -> PyResult<bool> {
    let registry = get_mesh_registry();
    let mut guard = registry.write().map_err(|error| {
        PyValueError::new_err(format!("Failed to acquire mesh registry lock: {error}"))
    })?;
    Ok(guard.remove(mesh_id).is_some())
}

fn sweep_cached_impl(
    py: Python<'_>,
    mesh1_id: &str,
    pose1_start_vec: &[f64],
    pose1_end_vec: &[f64],
    mesh2_id: &str,
    pose2_start_vec: &[f64],
    pose2_end_vec: &[f64],
) -> PyResult<SweptCollisionResult> {
    let p1_start = parse_pose(pose1_start_vec)?;
    let p1_end = parse_pose(pose1_end_vec)?;
    let p2_start = parse_pose(pose2_start_vec)?;
    let p2_end = parse_pose(pose2_end_vec)?;

    let registry = get_mesh_registry();
    let guard = registry.read().map_err(|e| {
        PyValueError::new_err(format!("Failed to acquire mesh registry lock: {}", e))
    })?;

    let mesh1 = Arc::clone(guard.get(mesh1_id).ok_or_else(|| {
        PyValueError::new_err(format!(
            "Mesh ID '{}' not found in registry. Please register it first.",
            mesh1_id
        ))
    })?);

    let mesh2 = Arc::clone(guard.get(mesh2_id).ok_or_else(|| {
        PyValueError::new_err(format!(
            "Mesh ID '{}' not found in registry. Please register it first.",
            mesh2_id
        ))
    })?);
    drop(guard);

    py.detach(|| {
        sweep_registered_meshes(
            mesh1.as_ref(),
            &p1_start,
            &p1_end,
            mesh2.as_ref(),
            &p2_start,
            &p2_end,
            true,
        )
    })
}

#[pyfunction]
fn check_swept_collision_cached(
    py: Python<'_>,
    mesh1_id: String,
    pose1_start_vec: Vec<f64>,
    pose1_end_vec: Vec<f64>,
    mesh2_id: String,
    pose2_start_vec: Vec<f64>,
    pose2_end_vec: Vec<f64>,
) -> PyResult<String> {
    let result = sweep_cached_impl(
        py,
        &mesh1_id,
        &pose1_start_vec,
        &pose1_end_vec,
        &mesh2_id,
        &pose2_start_vec,
        &pose2_end_vec,
    )?;

    serde_json::to_string(&result)
        .map_err(|e| PyValueError::new_err(format!("Serialization error: {}", e)))
}

#[pyfunction]
fn check_swept_collision_cached_native(
    py: Python<'_>,
    mesh1_id: String,
    pose1_start_vec: Vec<f64>,
    pose1_end_vec: Vec<f64>,
    mesh2_id: String,
    pose2_start_vec: Vec<f64>,
    pose2_end_vec: Vec<f64>,
) -> PyResult<Py<PyDict>> {
    let result = sweep_cached_impl(
        py,
        &mesh1_id,
        &pose1_start_vec,
        &pose1_end_vec,
        &mesh2_id,
        &pose2_start_vec,
        &pose2_end_vec,
    )?;

    swept_result_to_python(py, &result)
}

type CachedSweepInput = (String, Vec<f64>, Vec<f64>, String, Vec<f64>, Vec<f64>);

struct PreparedCachedSweep {
    mesh1: Arc<RegisteredMesh>,
    pose1_start: Pose,
    pose1_end: Pose,
    mesh2: Arc<RegisteredMesh>,
    pose2_start: Pose,
    pose2_end: Pose,
}

#[pyfunction(signature = (queries, parallel=true, tight_bounds=true))]
fn check_swept_collision_cached_batch_native(
    py: Python<'_>,
    queries: Vec<CachedSweepInput>,
    parallel: bool,
    tight_bounds: bool,
) -> PyResult<Vec<Py<PyDict>>> {
    let prepared = prepare_cached_sweeps(queries)?;
    let execute = |query: &PreparedCachedSweep| {
        sweep_registered_meshes(
            query.mesh1.as_ref(),
            &query.pose1_start,
            &query.pose1_end,
            query.mesh2.as_ref(),
            &query.pose2_start,
            &query.pose2_end,
            tight_bounds,
        )
    };
    let results = py.detach(|| {
        if parallel {
            prepared
                .par_iter()
                .map(execute)
                .collect::<PyResult<Vec<_>>>()
        } else {
            prepared.iter().map(execute).collect::<PyResult<Vec<_>>>()
        }
    })?;
    results
        .iter()
        .map(|result| swept_result_to_python(py, result))
        .collect()
}

fn prepare_cached_sweeps(queries: Vec<CachedSweepInput>) -> PyResult<Vec<PreparedCachedSweep>> {
    let registry = get_mesh_registry();
    let guard = registry.read().map_err(|error| {
        PyValueError::new_err(format!("Failed to acquire mesh registry lock: {error}"))
    })?;
    let mut prepared = Vec::with_capacity(queries.len());
    for (mesh1_id, pose1_start, pose1_end, mesh2_id, pose2_start, pose2_end) in queries {
        let mesh1 = Arc::clone(guard.get(&mesh1_id).ok_or_else(|| {
            PyValueError::new_err(format!(
                "Mesh ID '{mesh1_id}' not found in registry. Please register it first."
            ))
        })?);
        let mesh2 = Arc::clone(guard.get(&mesh2_id).ok_or_else(|| {
            PyValueError::new_err(format!(
                "Mesh ID '{mesh2_id}' not found in registry. Please register it first."
            ))
        })?);
        prepared.push(PreparedCachedSweep {
            mesh1,
            pose1_start: parse_pose(&pose1_start)?,
            pose1_end: parse_pose(&pose1_end)?,
            mesh2,
            pose2_start: parse_pose(&pose2_start)?,
            pose2_end: parse_pose(&pose2_end)?,
        });
    }
    drop(guard);
    Ok(prepared)
}

#[pyfunction]
fn compute_assembly_contacts(
    py: Python<'_>,
    assembly_list: Vec<Bound<'_, PyDict>>,
    tolerance: f64,
) -> PyResult<String> {
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(PyValueError::new_err(
            "contact tolerance must be finite and nonnegative",
        ));
    }
    struct MeshReconstruction {
        name: String,
        vertices: Vec<Vector>,
        faces: Vec<Vec<usize>>,
        bbox: Aabb,
    }

    let mut meshes = Vec::with_capacity(assembly_list.len());

    for dict_bound in assembly_list {
        let name: String = dict_bound
            .get_item("name")?
            .ok_or_else(|| PyValueError::new_err("Missing assembly field: name"))?
            .extract()?;
        let v_obj = dict_bound
            .get_item("vertices")?
            .ok_or_else(|| PyValueError::new_err("Missing assembly field: vertices"))?;
        let idx_obj = dict_bound
            .get_item("indices")?
            .ok_or_else(|| PyValueError::new_err("Missing assembly field: indices"))?;
        let off_obj = dict_bound
            .get_item("offsets")?
            .ok_or_else(|| PyValueError::new_err("Missing assembly field: offsets"))?;

        let v_buf = PyBuffer::<f64>::get(&v_obj)
            .map_err(|e| PyValueError::new_err(format!("v buffer: {}", e)))?;
        let idx_buf = PyBuffer::<i32>::get(&idx_obj)
            .map_err(|e| PyValueError::new_err(format!("idx buffer: {}", e)))?;
        let off_buf = PyBuffer::<i32>::get(&off_obj)
            .map_err(|e| PyValueError::new_err(format!("off buffer: {}", e)))?;

        let v_slice = v_buf
            .as_slice(py)
            .ok_or_else(|| PyValueError::new_err("v not contiguous"))?;
        let idx_slice = idx_buf
            .as_slice(py)
            .ok_or_else(|| PyValueError::new_err("idx not contiguous"))?;
        let off_slice = off_buf
            .as_slice(py)
            .ok_or_else(|| PyValueError::new_err("off not contiguous"))?;

        let v_flat_owned: Vec<f64> = v_slice.iter().map(|cell| cell.get()).collect();
        let v_flat: &[f64] = &v_flat_owned;
        let idx_owned: Vec<i32> = idx_slice.iter().map(|cell| cell.get()).collect();
        let idx: &[i32] = &idx_owned;
        let off_owned: Vec<i32> = off_slice.iter().map(|cell| cell.get()).collect();
        let off: &[i32] = &off_owned;

        validate_buffers(v_flat, idx, off)?;
        let vertex_count = v_flat.len() / 3;
        let mut vertices = Vec::with_capacity(vertex_count);
        let mut raw_v_vec = Vec::with_capacity(vertex_count);
        for chunk in v_flat.as_chunks::<3>().0 {
            vertices.push(Vector::new(chunk[0], chunk[1], chunk[2]));
            raw_v_vec.push(vec![chunk[0], chunk[1], chunk[2]]);
        }

        let face_count = if off.is_empty() { 0 } else { off.len() - 1 };
        let mut faces = Vec::with_capacity(face_count);
        for i in 0..face_count {
            let start = off[i] as usize;
            let end = off[i + 1] as usize;
            let mut face = Vec::with_capacity(end - start);
            for &idx_val in &idx[start..end] {
                face.push(idx_val as usize);
            }
            faces.push(face);
        }

        let bbox = Aabb::from_vertices(&raw_v_vec);

        meshes.push(MeshReconstruction {
            name,
            vertices,
            faces,
            bbox,
        });
    }

    let mut results = py.detach(|| {
        let contact_interfaces = Mutex::new(Vec::new());
        (0..meshes.len()).into_par_iter().for_each(|i| {
            for j in (i + 1)..meshes.len() {
                let mesh_a = &meshes[i];
                let mesh_b = &meshes[j];

                let dx = (mesh_a.bbox.min_x - mesh_b.bbox.max_x)
                    .max(mesh_b.bbox.min_x - mesh_a.bbox.max_x);
                let dy = (mesh_a.bbox.min_y - mesh_b.bbox.max_y)
                    .max(mesh_b.bbox.min_y - mesh_a.bbox.max_y);
                let dz = (mesh_a.bbox.min_z - mesh_b.bbox.max_z)
                    .max(mesh_b.bbox.min_z - mesh_a.bbox.max_z);
                if dx > tolerance || dy > tolerance || dz > tolerance {
                    continue;
                }

                for f_a in &mesh_a.faces {
                    let (n_a, c_a) = calculate_face_normal_and_centroid(&mesh_a.vertices, f_a);
                    if n_a.length_squared() < 1e-6 {
                        continue;
                    }
                    let plane_deviation_a = maximum_plane_distance(&mesh_a.vertices, f_a, c_a, n_a);

                    for f_b in &mesh_b.faces {
                        let (n_b, _) = calculate_face_normal_and_centroid(&mesh_b.vertices, f_b);
                        if n_b.length_squared() < 1e-6 {
                            continue;
                        }
                        let normal_dot = n_a.dot(n_b);
                        if normal_dot > -0.999 {
                            continue;
                        }
                        let plane_deviation_b =
                            maximum_plane_distance(&mesh_b.vertices, f_b, c_a, n_a);
                        let maximum_plane_deviation = plane_deviation_a.max(plane_deviation_b);
                        if maximum_plane_deviation > tolerance {
                            continue;
                        }

                        let u = if n_a.x.abs() > 0.1 {
                            Vector::new(-n_a.y, n_a.x, 0.0).normalize()
                        } else {
                            Vector::new(0.0, -n_a.z, n_a.y).normalize()
                        };
                        let v = n_a.cross(u).normalize();
                        let (area, centroid_2d, patches) = intersect_projected_faces(
                            &mesh_a.vertices,
                            f_a,
                            &mesh_b.vertices,
                            f_b,
                            c_a,
                            u,
                            v,
                        );
                        let area_epsilon = (tolerance * tolerance).max(f64::EPSILON);
                        if area <= area_epsilon {
                            continue;
                        }

                        let centroid_3d = c_a + u * centroid_2d.x + v * centroid_2d.y;
                        let patches_3d: Vec<Vec<[f64; 3]>> = patches
                            .iter()
                            .map(|patch| {
                                patch
                                    .iter()
                                    .map(|point| {
                                        let point = c_a + u * point.x + v * point.y;
                                        [point.x, point.y, point.z]
                                    })
                                    .collect()
                            })
                            .collect();
                        let vertices_3d = patches_3d.iter().flatten().copied().collect();
                        let classification_reliable =
                            normal_dot <= -1.0 + 1e-8 && maximum_plane_deviation <= tolerance;

                        contact_interfaces.lock().unwrap().push(ContactInterface {
                            block_a: mesh_a.name.clone(),
                            block_b: mesh_b.name.clone(),
                            area,
                            // Compatibility field. Generic meshes do not carry a unit system.
                            area_m2: area,
                            area_units: "mesh_units_squared".to_string(),
                            centroid: [centroid_3d.x, centroid_3d.y, centroid_3d.z],
                            normal: [n_a.x, n_a.y, n_a.z],
                            normal_alignment: normal_dot,
                            maximum_plane_deviation,
                            classification_reliable,
                            method: "triangulated_planar_patch_clipping".to_string(),
                            vertices_3d,
                            patches_3d,
                        });
                    }
                }
            }
        });
        contact_interfaces.into_inner().unwrap()
    });
    for contact in &mut results {
        if contact.block_a > contact.block_b {
            std::mem::swap(&mut contact.block_a, &mut contact.block_b);
            contact.normal = [-contact.normal[0], -contact.normal[1], -contact.normal[2]];
        }
    }
    results.sort_by(|a, b| {
        a.block_a
            .cmp(&b.block_a)
            .then_with(|| a.block_b.cmp(&b.block_b))
    });
    serde_json::to_string(&results).map_err(|err| {
        PyValueError::new_err(format!("Failed to serialize assembly contacts: {}", err))
    })
}

#[pyfunction]
#[pyo3(signature = (json_str, weld_tolerance=DEFAULT_WELD_TOLERANCE))]
fn fix_mesh_json(json_str: &str, weld_tolerance: f64) -> PyResult<String> {
    if !weld_tolerance.is_finite() || weld_tolerance <= 0.0 {
        return Err(PyValueError::new_err(
            "weld_tolerance must be finite and greater than zero",
        ));
    }
    let mut bytes = json_str.as_bytes().to_vec();
    let mut obj: CompasDataObject = simd_json::serde::from_slice(&mut bytes)
        .map_err(|err| PyValueError::new_err(format!("Malformed COMPAS JSON Schema: {}", err)))?;

    let (vertices, faces) = obj
        .data
        .get_vertices_and_faces()
        .map_err(PyValueError::new_err)?;
    validate_geometry(&vertices, &faces)?;

    let (welded_vertices, welded_faces, weld_details) =
        weld_vertices(&vertices, &faces, weld_tolerance);
    let (fixed_faces, flip_details) = unify_winding_directions(welded_faces);

    let welded_count = weld_details.len();
    let flipped_count = flip_details.len();

    obj.data.vertices = Some(welded_vertices);
    obj.data.faces = Some(fixed_faces);
    obj.data.vertex = None;
    obj.data.face = None;

    let fixed_json = serde_json::to_string(&obj)
        .map_err(|err| PyValueError::new_err(format!("Failed to serialize fixed mesh: {}", err)))?;

    let report = FixedMeshReport {
        weld_tolerance,
        welded_count,
        flipped_count,
        weld_details,
        flip_details,
        fixed_json,
    };

    serde_json::to_string(&report)
        .map_err(|err| PyValueError::new_err(format!("Failed to serialize fix report: {}", err)))
}

#[pyfunction]
#[pyo3(signature = (vertices_obj, face_indices_obj, face_offsets_obj, weld_tolerance=DEFAULT_WELD_TOLERANCE))]
fn fix_mesh_buffers(
    py: Python<'_>,
    vertices_obj: &Bound<'_, PyAny>,
    face_indices_obj: &Bound<'_, PyAny>,
    face_offsets_obj: &Bound<'_, PyAny>,
    weld_tolerance: f64,
) -> PyResult<String> {
    if !weld_tolerance.is_finite() || weld_tolerance <= 0.0 {
        return Err(PyValueError::new_err(
            "weld_tolerance must be finite and greater than zero",
        ));
    }
    let v_buf = PyBuffer::<f64>::get(vertices_obj)
        .map_err(|e| PyValueError::new_err(format!("v buffer: {}", e)))?;
    let idx_buf = PyBuffer::<i32>::get(face_indices_obj)
        .map_err(|e| PyValueError::new_err(format!("idx buffer: {}", e)))?;
    let off_buf = PyBuffer::<i32>::get(face_offsets_obj)
        .map_err(|e| PyValueError::new_err(format!("off buffer: {}", e)))?;

    let v_slice = v_buf
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("v not contiguous"))?;
    let idx_slice = idx_buf
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("idx not contiguous"))?;
    let off_slice = off_buf
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("off not contiguous"))?;

    let v_flat_owned: Vec<f64> = v_slice.iter().map(|cell| cell.get()).collect();
    let v_flat: &[f64] = &v_flat_owned;
    let idx_owned: Vec<i32> = idx_slice.iter().map(|cell| cell.get()).collect();
    let idx: &[i32] = &idx_owned;
    let off_owned: Vec<i32> = off_slice.iter().map(|cell| cell.get()).collect();
    let off: &[i32] = &off_owned;

    validate_buffers(v_flat, idx, off)?;
    let vertex_count = v_flat.len() / 3;
    let mut vertices = Vec::with_capacity(vertex_count);
    for chunk in v_flat.as_chunks::<3>().0 {
        vertices.push(vec![chunk[0], chunk[1], chunk[2]]);
    }

    let face_count = if off.is_empty() { 0 } else { off.len() - 1 };
    let mut faces = Vec::with_capacity(face_count);
    for i in 0..face_count {
        let start = off[i] as usize;
        let end = off[i + 1] as usize;
        let mut face = Vec::with_capacity(end - start);
        for &idx_val in &idx[start..end] {
            face.push(idx_val as usize);
        }
        faces.push(face);
    }

    let (welded_vertices, welded_faces, weld_details) =
        weld_vertices(&vertices, &faces, weld_tolerance);
    let (fixed_faces, flip_details) = unify_winding_directions(welded_faces);

    let welded_count = weld_details.len();
    let flipped_count = flip_details.len();

    let mut out_vertices = Vec::with_capacity(welded_vertices.len() * 3);
    for v in &welded_vertices {
        out_vertices.push(v[0]);
        out_vertices.push(v[1]);
        out_vertices.push(v[2]);
    }

    let mut out_idx = Vec::new();
    let mut out_off = Vec::with_capacity(fixed_faces.len() + 1);
    out_off.push(0);
    for face in &fixed_faces {
        for &v_idx in face {
            out_idx.push(v_idx as i32);
        }
        out_off.push(out_idx.len() as i32);
    }

    let report = FixedBuffersReport {
        vertices: out_vertices,
        face_indices: out_idx,
        face_offsets: out_off,
        weld_tolerance,
        welded_count,
        flipped_count,
        weld_details,
        flip_details,
    };

    serde_json::to_string(&report).map_err(|err| {
        PyValueError::new_err(format!("Failed to serialize fixed buffers report: {}", err))
    })
}

#[pyfunction]
fn run_preflight_json(json_str: &str, profile: &str) -> PyResult<String> {
    let mut bytes = json_str.as_bytes().to_vec();
    let obj: CompasDataObject = simd_json::serde::from_slice(&mut bytes).map_err(|err| {
        PyValueError::new_err(format!("Malformed COMPAS JSON Schema (SIMD): {}", err))
    })?;

    let (vertices, faces) = obj
        .data
        .get_vertices_and_faces()
        .map_err(PyValueError::new_err)?;
    validate_geometry(&vertices, &faces)?;

    let bbox = Aabb::from_vertices(&vertices);
    let volume_m3 = compute_mesh_volume(&vertices, &faces);
    let topology = mesh_topology_metrics(&vertices, &faces);
    let boundary_edges_count = topology.boundary_edges.len();

    let bounds_x_dim = bbox.max_x - bbox.min_x;
    let bounds_y_dim = bbox.max_y - bbox.min_y;
    let bounds_z_dim = bbox.max_z - bbox.min_z;

    let max_planarity_deviation = compute_max_planarity_deviation(&vertices, &faces);
    let min_face_quality = compute_min_face_quality(&vertices, &faces);

    let (density, max_mass, max_x, max_y, max_z, require_watertight) = match profile {
        "abb-concrete-3dprint" => (2400.0, 500.0, 1.5, 1.5, 2.0, false),
        "kuka-timber" | "default" => (500.0, 150.0, 3.0, 0.4, 0.4, true),
        _ => {
            return Err(PyValueError::new_err(format!(
                "Unknown fabrication profile: {profile}"
            )))
        }
    };

    let estimated_mass_kg = volume_m3 * density;
    let fits_workspace = bounds_x_dim <= max_x && bounds_y_dim <= max_y && bounds_z_dim <= max_z;

    let planarity_ok = max_planarity_deviation <= 0.005;
    let mesh_quality_ok = min_face_quality >= 0.1;

    let mut is_compliant = fits_workspace
        && topology.volume_reliable
        && (estimated_mass_kg <= max_mass)
        && planarity_ok
        && mesh_quality_ok;
    if require_watertight && !topology.is_watertight {
        is_compliant = false;
    }

    let mut triangulated_faces = Vec::new();
    for face in &faces {
        triangulated_faces.extend(triangulate_face(&vertices, face));
    }

    let result = PreflightResult {
        profile_name: profile.to_string(),
        is_compliant,
        volume_m3,
        volume_reliable: topology.volume_reliable,
        estimated_mass_kg,
        max_mass_kg: max_mass,
        mass_within_limit: estimated_mass_kg <= max_mass,
        requires_watertight: require_watertight,
        boundary_edges_count,
        boundary_edges: topology.boundary_edges,
        is_watertight: topology.is_watertight,
        winding_consistent: topology.winding_consistent,
        degenerate_faces_count: topology.degenerate_faces_count,
        surface_components: topology.surface_components,
        isolated_vertices_count: topology.isolated_vertices_count,
        self_intersections: topology.self_intersections,
        fits_workspace,
        bounds_x_dim,
        bounds_y_dim,
        bounds_z_dim,
        bounding_box: bbox,
        vertices: vertices.clone(),
        triangulated_faces,
        euler_characteristic: topology.euler_characteristic,
        genus: topology.genus,
        max_planarity_deviation,
        min_face_quality,
    };

    serde_json::to_string(&result).map_err(|err| {
        PyValueError::new_err(format!("Failed to serialize preflight report: {}", err))
    })
}

#[pyfunction]
fn run_preflight_buffers(
    py: Python<'_>,
    vertices_obj: &Bound<'_, PyAny>,
    face_indices_obj: &Bound<'_, PyAny>,
    face_offsets_obj: &Bound<'_, PyAny>,
    profile: &str,
) -> PyResult<String> {
    let v_buf = PyBuffer::<f64>::get(vertices_obj)
        .map_err(|e| PyValueError::new_err(format!("v buffer: {}", e)))?;
    let idx_buf = PyBuffer::<i32>::get(face_indices_obj)
        .map_err(|e| PyValueError::new_err(format!("idx buffer: {}", e)))?;
    let off_buf = PyBuffer::<i32>::get(face_offsets_obj)
        .map_err(|e| PyValueError::new_err(format!("off buffer: {}", e)))?;

    let v_slice = v_buf
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("v not contiguous"))?;
    let idx_slice = idx_buf
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("idx not contiguous"))?;
    let off_slice = off_buf
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("off not contiguous"))?;

    let v_flat_owned: Vec<f64> = v_slice.iter().map(|cell| cell.get()).collect();
    let v_flat: &[f64] = &v_flat_owned;
    let idx_owned: Vec<i32> = idx_slice.iter().map(|cell| cell.get()).collect();
    let idx: &[i32] = &idx_owned;
    let off_owned: Vec<i32> = off_slice.iter().map(|cell| cell.get()).collect();
    let off: &[i32] = &off_owned;

    validate_buffers(v_flat, idx, off)?;
    let vertex_count = v_flat.len() / 3;
    let mut vertices = Vec::with_capacity(vertex_count);
    for chunk in v_flat.as_chunks::<3>().0 {
        vertices.push(vec![chunk[0], chunk[1], chunk[2]]);
    }

    let face_count = if off.is_empty() { 0 } else { off.len() - 1 };
    let mut faces = Vec::with_capacity(face_count);
    for i in 0..face_count {
        let start = off[i] as usize;
        let end = off[i + 1] as usize;
        let mut face = Vec::with_capacity(end - start);
        for &idx_val in &idx[start..end] {
            face.push(idx_val as usize);
        }
        faces.push(face);
    }

    let bbox = Aabb::from_vertices(&vertices);
    let volume_m3 = compute_mesh_volume(&vertices, &faces);
    let topology = mesh_topology_metrics(&vertices, &faces);
    let boundary_edges_count = topology.boundary_edges.len();

    let bounds_x_dim = bbox.max_x - bbox.min_x;
    let bounds_y_dim = bbox.max_y - bbox.min_y;
    let bounds_z_dim = bbox.max_z - bbox.min_z;

    let max_planarity_deviation = compute_max_planarity_deviation(&vertices, &faces);
    let min_face_quality = compute_min_face_quality(&vertices, &faces);

    let (density, max_mass, max_x, max_y, max_z, require_watertight) = match profile {
        "abb-concrete-3dprint" => (2400.0, 500.0, 1.5, 1.5, 2.0, false),
        "kuka-timber" | "default" => (500.0, 150.0, 3.0, 0.4, 0.4, true),
        _ => {
            return Err(PyValueError::new_err(format!(
                "Unknown fabrication profile: {profile}"
            )))
        }
    };

    let estimated_mass_kg = volume_m3 * density;
    let fits_workspace = bounds_x_dim <= max_x && bounds_y_dim <= max_y && bounds_z_dim <= max_z;

    let planarity_ok = max_planarity_deviation <= 0.005;
    let mesh_quality_ok = min_face_quality >= 0.1;

    let mut is_compliant = fits_workspace
        && topology.volume_reliable
        && (estimated_mass_kg <= max_mass)
        && planarity_ok
        && mesh_quality_ok;
    if require_watertight && !topology.is_watertight {
        is_compliant = false;
    }

    let mut triangulated_faces = Vec::new();
    for face in &faces {
        triangulated_faces.extend(triangulate_face(&vertices, face));
    }

    let result = PreflightResult {
        profile_name: profile.to_string(),
        is_compliant,
        volume_m3,
        volume_reliable: topology.volume_reliable,
        estimated_mass_kg,
        max_mass_kg: max_mass,
        mass_within_limit: estimated_mass_kg <= max_mass,
        requires_watertight: require_watertight,
        boundary_edges_count,
        boundary_edges: topology.boundary_edges,
        is_watertight: topology.is_watertight,
        winding_consistent: topology.winding_consistent,
        degenerate_faces_count: topology.degenerate_faces_count,
        surface_components: topology.surface_components,
        isolated_vertices_count: topology.isolated_vertices_count,
        self_intersections: topology.self_intersections,
        fits_workspace,
        bounds_x_dim,
        bounds_y_dim,
        bounds_z_dim,
        bounding_box: bbox,
        vertices: vertices.clone(),
        triangulated_faces,
        euler_characteristic: topology.euler_characteristic,
        genus: topology.genus,
        max_planarity_deviation,
        min_face_quality,
    };

    serde_json::to_string(&result).map_err(|err| {
        PyValueError::new_err(format!(
            "Failed to serialize preflight buffers report: {}",
            err
        ))
    })
}

#[pyfunction]
fn detect_clashes_json(items: Vec<(String, String)>, clearance_tolerance: f64) -> PyResult<String> {
    if !clearance_tolerance.is_finite() || clearance_tolerance < 0.0 {
        return Err(PyValueError::new_err(
            "Clearance must be finite and nonnegative",
        ));
    }
    let parts: Vec<SpatialPart> = items
        .iter()
        .enumerate()
        .map(|(idx, (name, json_str))| {
            let obj: CompasDataObject = serde_json::from_str(json_str)
                .map_err(|e| PyValueError::new_err(format!("Invalid mesh {name}: {e}")))?;
            let (vertices, faces) = obj
                .data
                .get_vertices_and_faces()
                .map_err(|error| PyValueError::new_err(format!("Invalid mesh {name}: {error}")))?;
            validate_geometry(&vertices, &faces)?;
            Ok(SpatialPart {
                id: idx,
                name: name.clone(),
                bbox: Aabb::from_vertices(&vertices),
                vertices,
                faces,
            })
        })
        .collect::<PyResult<_>>()?;

    let rtree = RTree::bulk_load(parts.clone());
    let batches: Result<Vec<Vec<AssemblyClashResult>>, String> = parts
        .par_iter()
        .map(|part_a| {
            let min_corner = [
                part_a.bbox.min_x - clearance_tolerance,
                part_a.bbox.min_y - clearance_tolerance,
                part_a.bbox.min_z - clearance_tolerance,
            ];
            let max_corner = [
                part_a.bbox.max_x + clearance_tolerance,
                part_a.bbox.max_y + clearance_tolerance,
                part_a.bbox.max_z + clearance_tolerance,
            ];
            let inflated_envelope = rstar::AABB::from_corners(min_corner, max_corner);
            let mut reports = Vec::new();

            for candidate in rtree.locate_in_envelope_intersecting(inflated_envelope) {
                if part_a.id >= candidate.id {
                    continue;
                }
                let min_dist = compute_mesh_distance(part_a, candidate).map_err(|error| {
                    format!(
                        "mesh distance failed for '{}' and '{}': {}",
                        part_a.name, candidate.name, error
                    )
                })?;
                let has_intersection = min_dist <= 0.0;
                let is_clearance_violation = min_dist < clearance_tolerance;
                let (relationship, classification_reliable) = if min_dist > 1e-12 {
                    ("separated", true)
                } else {
                    // Surface-mesh distance alone cannot distinguish tangential contact,
                    // crossing surfaces, or closed-solid containment.
                    ("surface_contact_or_intersection", false)
                };

                if has_intersection || is_clearance_violation {
                    reports.push(AssemblyClashResult {
                        part_a: part_a.name.clone(),
                        part_b: candidate.name.clone(),
                        has_intersection,
                        relationship: relationship.to_string(),
                        classification_reliable,
                        minimum_distance: min_dist,
                        is_clearance_violation,
                    });
                }
            }
            Ok(reports)
        })
        .collect();
    let mut results: Vec<AssemblyClashResult> = batches
        .map_err(PyValueError::new_err)?
        .into_iter()
        .flatten()
        .collect();
    results.sort_by(|a, b| {
        a.part_a
            .cmp(&b.part_a)
            .then_with(|| a.part_b.cmp(&b.part_b))
    });
    serde_json::to_string(&results).map_err(|err| {
        PyValueError::new_err(format!("Serialization error during clash phase: {}", err))
    })
}

#[cfg(feature = "bench-support")]
pub mod bench_support {
    use super::*;

    static CUBE: OnceLock<TriMesh> = OnceLock::new();

    fn retained_cube() -> &'static TriMesh {
        CUBE.get_or_init(|| {
            let vertices = vec![
                Vector::new(-0.5, -0.5, -0.5),
                Vector::new(0.5, -0.5, -0.5),
                Vector::new(0.5, 0.5, -0.5),
                Vector::new(-0.5, 0.5, -0.5),
                Vector::new(-0.5, -0.5, 0.5),
                Vector::new(0.5, -0.5, 0.5),
                Vector::new(0.5, 0.5, 0.5),
                Vector::new(-0.5, 0.5, 0.5),
            ];
            let triangles = vec![
                [0, 3, 2],
                [0, 2, 1],
                [4, 5, 6],
                [4, 6, 7],
                [0, 1, 5],
                [0, 5, 4],
                [1, 2, 6],
                [1, 6, 5],
                [2, 3, 7],
                [2, 7, 6],
                [3, 0, 4],
                [3, 4, 7],
            ];
            TriMesh::new(vertices, triangles).expect("static cube must be a valid triangle mesh")
        })
    }

    /// Native retained-geometry fixture used only by Criterion.
    ///
    /// This excludes Python conversion, JSON and mesh construction. The result
    /// keeps a correctness value so benchmark runs cannot silently time a no-op.
    pub fn retained_cube_linear_hit() -> (bool, f64) {
        let mesh = retained_cube();
        let start = Pose {
            rotation: Rotation::from_xyzw(0.0, 0.0, 0.0, 1.0),
            translation: Vector::new(-2.0, 0.0, 0.0),
        };
        let end = Pose {
            rotation: Rotation::from_xyzw(0.0, 0.0, 0.0, 1.0),
            translation: Vector::new(2.0, 0.0, 0.0),
        };
        let fixed = Pose::identity();
        let result = sweep_meshes(mesh, &start, &end, mesh, &fixed, &fixed)
            .expect("static benchmark query must be supported");
        (result.has_collision, result.time_of_impact.unwrap_or(-1.0))
    }
}

#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(
        clearance::verify_clearance_cached_batch_native,
        m
    )?)?;
    m.add_function(wrap_pyfunction!(validate_compas_json, m)?)?;
    m.add_function(wrap_pyfunction!(fix_mesh_json, m)?)?;
    m.add_function(wrap_pyfunction!(run_preflight_json, m)?)?;
    m.add_function(wrap_pyfunction!(detect_clashes_json, m)?)?;
    m.add_function(wrap_pyfunction!(validate_mesh_buffers, m)?)?;
    m.add_function(wrap_pyfunction!(scan_mesh_buffers_borrowed, m)?)?;
    m.add_function(wrap_pyfunction!(topology_predicates_buffers, m)?)?;
    m.add_function(wrap_pyfunction!(check_swept_collision, m)?)?;
    m.add_function(wrap_pyfunction!(compute_assembly_contacts, m)?)?;
    m.add_function(wrap_pyfunction!(fix_mesh_buffers, m)?)?;
    m.add_function(wrap_pyfunction!(run_preflight_buffers, m)?)?;

    m.add_function(wrap_pyfunction!(register_mesh, m)?)?;
    m.add_function(wrap_pyfunction!(register_mesh_buffers, m)?)?;
    m.add_function(wrap_pyfunction!(clear_mesh_registry, m)?)?;
    m.add_function(wrap_pyfunction!(unregister_mesh, m)?)?;
    m.add_function(wrap_pyfunction!(check_swept_collision_cached, m)?)?;
    m.add_function(wrap_pyfunction!(check_swept_collision_cached_native, m)?)?;
    m.add_function(wrap_pyfunction!(
        check_swept_collision_cached_batch_native,
        m
    )?)?;

    Ok(())
}
