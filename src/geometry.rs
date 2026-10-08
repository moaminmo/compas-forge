use rayon::prelude::*;
use rstar::{RTree, RTreeObject, AABB as RStarAABB};
use std::collections::{HashMap, HashSet, VecDeque};

use parry3d_f64::math::{Pose, Vector};
use parry3d_f64::query::intersection_test;
use parry3d_f64::shape::Triangle;

/// Represents an axis-aligned bounding box (AABB) in 3D space.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Aabb {
    pub min_x: f64,
    pub min_y: f64,
    pub min_z: f64,
    pub max_x: f64,
    pub max_y: f64,
    pub max_z: f64,
}

impl Aabb {
    pub fn from_vertices(vertices: &[Vec<f64>]) -> Self {
        if vertices.is_empty() {
            return Self {
                min_x: 0.0,
                min_y: 0.0,
                min_z: 0.0,
                max_x: 0.0,
                max_y: 0.0,
                max_z: 0.0,
            };
        }
        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut min_z = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;
        let mut max_z = f64::MIN;

        for v in vertices {
            if v.len() >= 3 {
                if v[0] < min_x {
                    min_x = v[0];
                }
                if v[0] > max_x {
                    max_x = v[0];
                }
                if v[1] < min_y {
                    min_y = v[1];
                }
                if v[1] > max_y {
                    max_y = v[1];
                }
                if v[2] < min_z {
                    min_z = v[2];
                }
                if v[2] > max_z {
                    max_z = v[2];
                }
            }
        }

        Self {
            min_x,
            min_y,
            min_z,
            max_x,
            max_y,
            max_z,
        }
    }
}

/// Dynamic spatial part holding geometric topological elements and spatial structures
#[derive(Debug, Clone)]
pub struct SpatialPart {
    pub id: usize,
    pub name: String,
    pub bbox: Aabb,
    pub vertices: Vec<Vec<f64>>,
    pub faces: Vec<Vec<usize>>,
}

impl RTreeObject for SpatialPart {
    type Envelope = RStarAABB<[f64; 3]>;

    fn envelope(&self) -> Self::Envelope {
        RStarAABB::from_corners(
            [self.bbox.min_x, self.bbox.min_y, self.bbox.min_z],
            [self.bbox.max_x, self.bbox.max_y, self.bbox.max_z],
        )
    }
}

#[derive(Clone)]
struct TriangleRecord {
    id: usize,
    face_index: usize,
    indices: [u32; 3],
    envelope: RStarAABB<[f64; 3]>,
}

impl RTreeObject for TriangleRecord {
    type Envelope = RStarAABB<[f64; 3]>;

    fn envelope(&self) -> Self::Envelope {
        self.envelope
    }
}

// If nondegenerate triangles share only one vertex, intersection beyond that
// vertex must reach an opposite edge of at least one triangle. Test those two
// edges, avoiding the normal topological contact at the shared vertex.
fn shared_vertex_overlap(a: &Triangle, b: &Triangle, shared: Vector) -> bool {
    let edge_hits = |source: &Triangle, target: &Triangle| {
        let points = [source.a, source.b, source.c];
        let edge: Vec<_> = points
            .iter()
            .filter(|p| **p != shared)
            .map(|p| *p - shared)
            .collect();
        if edge.len() != 2 {
            return true;
        }
        let triangle = [target.a - shared, target.b - shared, target.c - shared];
        let Some(normal) = (triangle[1] - triangle[0])
            .cross(triangle[2] - triangle[0])
            .try_normalize()
        else {
            return true;
        };
        let drop = (0..3)
            .max_by(|&i, &j| normal[i].abs().total_cmp(&normal[j].abs()))
            .unwrap();
        let xy = |p: Vector| [p[(drop + 1) % 3], p[(drop + 2) % 3]];
        let cross = |p: [f64; 2], q: [f64; 2], r: [f64; 2]| {
            (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
        };
        let tri = triangle.map(xy);
        let contains = |p| {
            let signs = [
                cross(tri[0], tri[1], p),
                cross(tri[1], tri[2], p),
                cross(tri[2], tri[0], p),
            ];
            signs.iter().all(|v| *v >= 0.0) || signs.iter().all(|v| *v <= 0.0)
        };
        let distances = edge
            .iter()
            .map(|p| normal.dot(*p - triangle[0]))
            .collect::<Vec<_>>();
        let tolerance = 64.0 * f64::EPSILON * edge[0].length().max(edge[1].length());
        if distances.iter().all(|v| v.abs() <= tolerance) {
            let (p, q) = (xy(edge[0]), xy(edge[1]));
            if contains(p) || contains(q) {
                return true;
            }
            for i in 0..3 {
                let (r, s) = (tri[i], tri[(i + 1) % 3]);
                // Bounding boxes plus orientation straddling also handles
                // collinear overlapping edges (without multiplying signs).
                if (0..2)
                    .any(|k| p[k].max(q[k]) < r[k].min(s[k]) || r[k].max(s[k]) < p[k].min(q[k]))
                {
                    continue;
                }
                let opposite = |x: f64, y: f64| (x <= 0.0 && y >= 0.0) || (y <= 0.0 && x >= 0.0);
                if opposite(cross(p, q, r), cross(p, q, s))
                    && opposite(cross(r, s, p), cross(r, s, q))
                {
                    return true;
                }
            }
            false
        } else if (distances[0] > 0.0 && distances[1] > 0.0)
            || (distances[0] < 0.0 && distances[1] < 0.0)
        {
            false
        } else {
            let t = distances[0] / (distances[0] - distances[1]);
            contains(xy(edge[0] + (edge[1] - edge[0]) * t))
        }
    };
    edge_hits(a, b) || edge_hits(b, a)
}

/// Detect non-adjacent intersections and overlap beyond shared edges/vertices.
/// This uses floating-point Parry predicates and is a diagnostic, not an exact
/// symbolic-geometry proof.
pub fn find_self_intersections(vertices: &[Vec<f64>], faces: &[Vec<usize>]) -> Vec<(usize, usize)> {
    let mut records = Vec::new();
    for (face_index, face) in faces.iter().enumerate() {
        for indices in triangulate_face(vertices, face) {
            let points = indices.map(|index| &vertices[index as usize]);
            let min = [
                points
                    .iter()
                    .map(|point| point[0])
                    .fold(f64::INFINITY, f64::min),
                points
                    .iter()
                    .map(|point| point[1])
                    .fold(f64::INFINITY, f64::min),
                points
                    .iter()
                    .map(|point| point[2])
                    .fold(f64::INFINITY, f64::min),
            ];
            let max = [
                points
                    .iter()
                    .map(|point| point[0])
                    .fold(f64::NEG_INFINITY, f64::max),
                points
                    .iter()
                    .map(|point| point[1])
                    .fold(f64::NEG_INFINITY, f64::max),
                points
                    .iter()
                    .map(|point| point[2])
                    .fold(f64::NEG_INFINITY, f64::max),
            ];
            records.push(TriangleRecord {
                id: records.len(),
                face_index,
                indices,
                envelope: RStarAABB::from_corners(min, max),
            });
        }
    }

    let tree = RTree::bulk_load(records.clone());
    let identity = Pose::identity();
    let mut intersections = HashSet::new();
    for first in &records {
        for second in tree.locate_in_envelope_intersecting(first.envelope()) {
            if first.id >= second.id || first.face_index == second.face_index {
                continue;
            }
            let shared: Vec<_> = first
                .indices
                .iter()
                .copied()
                .filter(|index| second.indices.contains(index))
                .collect();
            let triangle = |record: &TriangleRecord| {
                let point = |index: u32| {
                    let value = &vertices[index as usize];
                    Vector::new(value[0], value[1], value[2])
                };
                Triangle::new(
                    point(record.indices[0]),
                    point(record.indices[1]),
                    point(record.indices[2]),
                )
            };
            let intersects = match shared.len() {
                0 => intersection_test(&identity, &triangle(first), &identity, &triangle(second))
                    .is_ok_and(|result| result.intersecting),
                2 => {
                    // Two nondegenerate triangles sharing an edge can overlap
                    // beyond it only when coplanar. Same-side perpendiculars
                    // detect a foldover; opposite sides are normal adjacency.
                    // Near-coplanar same-side pairs are conservatively flagged.
                    let point = |i: u32| {
                        let p = &vertices[i as usize];
                        Vector::new(p[0], p[1], p[2])
                    };
                    let origin = point(shared[0]);
                    let edge = point(shared[1]) - origin;
                    let tip = |r: &TriangleRecord| {
                        point(*r.indices.iter().find(|i| !shared.contains(i)).unwrap()) - origin
                    };
                    let a = edge.cross(tip(first));
                    let b = edge.cross(tip(second));
                    match (a.try_normalize(), b.try_normalize()) {
                        (Some(a), Some(b)) => {
                            a.dot(b) > 0.0 && a.cross(b).length() <= 64.0 * f64::EPSILON
                        }
                        _ => true, // Degenerate shared-edge pair is not a valid shell.
                    }
                }
                3 => true, // Duplicate triangle, including reverse winding.
                1 => {
                    let p = &vertices[shared[0] as usize];
                    shared_vertex_overlap(
                        &triangle(first),
                        &triangle(second),
                        Vector::new(p[0], p[1], p[2]),
                    )
                }
                _ => unreachable!(),
            };
            if intersects {
                intersections.insert(if first.face_index < second.face_index {
                    (first.face_index, second.face_index)
                } else {
                    (second.face_index, first.face_index)
                });
            }
        }
    }
    let mut intersections: Vec<_> = intersections.into_iter().collect();
    intersections.sort_unstable();
    intersections
}

/// Project a simple planar polygon onto its dominant plane and triangulate concavities.
pub fn triangulate_face<V: std::ops::Index<usize, Output = f64>>(
    vertices: &[V],
    face: &[usize],
) -> Vec<[u32; 3]> {
    if face.len() < 3 {
        return Vec::new();
    }
    let mut normal = [0.0; 3];
    for i in 0..face.len() {
        let a = &vertices[face[i]];
        let b = &vertices[face[(i + 1) % face.len()]];
        normal[0] += (a[1] - b[1]) * (a[2] + b[2]);
        normal[1] += (a[2] - b[2]) * (a[0] + b[0]);
        normal[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    let drop = (0..3)
        .max_by(|&a, &b| normal[a].abs().total_cmp(&normal[b].abs()))
        .unwrap();
    let u = (drop + 1) % 3;
    let v = (drop + 2) % 3;
    let coords: Vec<f64> = face
        .iter()
        .flat_map(|&i| [vertices[i][u], vertices[i][v]])
        .collect();
    let indices = earcutr::earcut(&coords, &[], 2).unwrap_or_default();
    indices
        .as_chunks::<3>()
        .0
        .iter()
        .map(|t| {
            let mut triangle = [face[t[0]] as u32, face[t[1]] as u32, face[t[2]] as u32];
            let a = &vertices[triangle[0] as usize];
            let b = &vertices[triangle[1] as usize];
            let c = &vertices[triangle[2] as usize];
            let signed = (b[u] - a[u]) * (c[v] - a[v]) - (b[v] - a[v]) * (c[u] - a[u]);
            if signed * normal[drop] < 0.0 {
                triangle.swap(1, 2);
            }
            triangle
        })
        .collect()
}

/// Vertex manifoldness: the link of each used vertex must be one path or cycle.
pub fn find_non_manifold_vertices(faces: &[Vec<usize>]) -> Vec<usize> {
    let mut links: HashMap<usize, Vec<(usize, usize)>> = HashMap::new();
    for face in faces {
        for i in 0..face.len() {
            links.entry(face[i]).or_default().push((
                face[(i + face.len() - 1) % face.len()],
                face[(i + 1) % face.len()],
            ));
        }
    }
    let mut invalid = Vec::new();
    for (vertex, edges) in links {
        let mut adjacency: HashMap<usize, Vec<usize>> = HashMap::new();
        for (a, b) in edges {
            adjacency.entry(a).or_default().push(b);
            adjacency.entry(b).or_default().push(a);
        }
        let ends = adjacency.values().filter(|x| x.len() == 1).count();
        let mut seen = HashSet::new();
        let mut stack = vec![*adjacency.keys().next().unwrap()];
        while let Some(v) = stack.pop() {
            if seen.insert(v) {
                stack.extend(&adjacency[&v]);
            }
        }
        if seen.len() != adjacency.len()
            || adjacency.values().any(|x| x.len() > 2)
            || (ends != 0 && ends != 2)
        {
            invalid.push(vertex);
        }
    }
    invalid.sort_unstable();
    invalid
}

fn face_components(faces: &[Vec<usize>]) -> Vec<Vec<usize>> {
    let mut edge_to_faces: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for (face_index, face) in faces.iter().enumerate() {
        for i in 0..face.len() {
            let a = face[i];
            let b = face[(i + 1) % face.len()];
            edge_to_faces
                .entry(if a < b { (a, b) } else { (b, a) })
                .or_default()
                .push(face_index);
        }
    }

    let mut adjacency = vec![Vec::new(); faces.len()];
    for incident in edge_to_faces.values() {
        for &a in incident {
            for &b in incident {
                if a != b {
                    adjacency[a].push(b);
                }
            }
        }
    }

    let mut visited = vec![false; faces.len()];
    let mut components = Vec::new();
    for start in 0..faces.len() {
        if visited[start] {
            continue;
        }
        visited[start] = true;
        let mut stack = vec![start];
        let mut component = Vec::new();
        while let Some(face_index) = stack.pop() {
            component.push(face_index);
            for &neighbor in &adjacency[face_index] {
                if !visited[neighbor] {
                    visited[neighbor] = true;
                    stack.push(neighbor);
                }
            }
        }
        components.push(component);
    }
    components
}

/// Count edge-connected surface components.
pub fn count_face_components(faces: &[Vec<usize>]) -> usize {
    face_components(faces).len()
}

/// Check that every shared manifold edge is traversed in opposite directions.
pub fn has_consistent_winding(faces: &[Vec<usize>]) -> bool {
    let mut directions: HashMap<(usize, usize), Vec<bool>> = HashMap::new();
    for face in faces {
        for i in 0..face.len() {
            let a = face[i];
            let b = face[(i + 1) % face.len()];
            let key = if a < b { (a, b) } else { (b, a) };
            directions.entry(key).or_default().push(a < b);
        }
    }
    directions
        .values()
        .all(|uses| uses.len() != 2 || uses[0] != uses[1])
}

/// Count faces that cannot produce a non-degenerate triangulation.
pub fn count_degenerate_faces(
    vertices: &[Vec<f64>],
    faces: &[Vec<usize>],
    area_tolerance: f64,
) -> usize {
    faces
        .iter()
        .filter(|face| {
            if face.iter().copied().collect::<HashSet<_>>().len() < 3 {
                return true;
            }
            let triangles = triangulate_face(vertices, face);
            triangles.is_empty()
                || triangles.iter().any(|tri| {
                    let a = &vertices[tri[0] as usize];
                    let b = &vertices[tri[1] as usize];
                    let c = &vertices[tri[2] as usize];
                    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                    let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
                    let cross = [
                        ab[1] * ac[2] - ab[2] * ac[1],
                        ab[2] * ac[0] - ab[0] * ac[2],
                        ab[0] * ac[1] - ab[1] * ac[0],
                    ];
                    0.5 * (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt()
                        <= area_tolerance
                })
        })
        .count()
}

/// Computes volume from oriented triangles, taking the absolute volume per
/// edge-connected component so oppositely oriented shells cannot cancel out.
pub fn compute_mesh_volume(vertices: &[Vec<f64>], faces: &[Vec<usize>]) -> f64 {
    if vertices.is_empty() || faces.is_empty() {
        return 0.0;
    }
    face_components(faces)
        .par_iter()
        .map(|component| {
            // Shift each shell to a nearby reference before evaluating scalar
            // triple products. The mathematical volume of a closed shell is
            // translation invariant, while this local frame avoids the severe
            // cancellation caused by large world coordinates.
            let anchor_index = faces[component[0]][0];
            let anchor = &vertices[anchor_index];
            let mut component_volume = 0.0;
            let mut compensation = 0.0;
            for &face_index in component {
                let tris = triangulate_face(vertices, &faces[face_index]);
                for tri in tris {
                    let p0 = &vertices[tri[0] as usize];
                    let p1 = &vertices[tri[1] as usize];
                    let p2 = &vertices[tri[2] as usize];
                    let a = [p0[0] - anchor[0], p0[1] - anchor[1], p0[2] - anchor[2]];
                    let b = [p1[0] - anchor[0], p1[1] - anchor[1], p1[2] - anchor[2]];
                    let c = [p2[0] - anchor[0], p2[1] - anchor[1], p2[2] - anchor[2]];
                    let signed_six_volume = a[0] * (b[1] * c[2] - b[2] * c[1])
                        - a[1] * (b[0] * c[2] - b[2] * c[0])
                        + a[2] * (b[0] * c[1] - b[1] * c[0]);
                    let corrected = signed_six_volume - compensation;
                    let next = component_volume + corrected;
                    compensation = (next - component_volume) - corrected;
                    component_volume = next;
                }
            }
            (component_volume / 6.0).abs()
        })
        .sum()
}

/// Counts unique undirected edges in parallel. Used for Euler characteristic calculation.
pub fn count_unique_edges(faces: &[Vec<usize>]) -> usize {
    if faces.is_empty() {
        return 0;
    }
    let unique_edges: HashSet<(usize, usize)> = faces
        .par_iter()
        .flat_map(|face| {
            let mut edges = Vec::new();
            let len = face.len();
            if len < 3 {
                return edges;
            }
            for i in 0..len {
                let u = face[i];
                let v = face[(i + 1) % len];
                let edge = if u < v { (u, v) } else { (v, u) };
                edges.push(edge);
            }
            edges
        })
        .collect();
    unique_edges.len()
}

/// Computes maximum planarity deviation of faces using Newell's Method.
pub fn compute_max_planarity_deviation(vertices: &[Vec<f64>], faces: &[Vec<usize>]) -> f64 {
    if vertices.is_empty() || faces.is_empty() {
        return 0.0;
    }
    faces
        .par_iter()
        .map(|face| {
            let len = face.len();
            if len < 4 {
                return 0.0; // Triangles are always perfectly planar
            }

            // 1. Centroid
            let mut cx = 0.0;
            let mut cy = 0.0;
            let mut cz = 0.0;
            for &idx in face {
                let v = &vertices[idx];
                cx += v[0];
                cy += v[1];
                cz += v[2];
            }
            cx /= len as f64;
            cy /= len as f64;
            cz /= len as f64;

            // 2. Newell's Normal Fitting
            let mut nx = 0.0;
            let mut ny = 0.0;
            let mut nz = 0.0;
            for i in 0..len {
                let idx_i = face[i];
                let idx_j = face[(i + 1) % len];
                let vi = &vertices[idx_i];
                let vj = &vertices[idx_j];
                nx += (vi[1] - vj[1]) * (vi[2] + vj[2]);
                ny += (vi[2] - vj[2]) * (vi[0] + vj[0]);
                nz += (vi[0] - vj[0]) * (vi[1] + vj[1]);
            }
            let norm = (nx * nx + ny * ny + nz * nz).sqrt();
            if norm < 1e-12 {
                return 0.0;
            }
            nx /= norm;
            ny /= norm;
            nz /= norm;

            // 3. Maximum distance of any face vertex to the Newell plane
            let mut max_dev = 0.0;
            for &idx in face {
                let v = &vertices[idx];
                let dx = v[0] - cx;
                let dy = v[1] - cy;
                let dz = v[2] - cz;
                let dist = (dx * nx + dy * ny + dz * nz).abs();
                if dist > max_dev {
                    max_dev = dist;
                }
            }
            max_dev
        })
        .max_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
        .unwrap_or(0.0)
}

/// Evaluates normalized triangle geometric quality metric (q) in parallel.
/// Equitable triangles yield q = 1.0, degenerate flat triangles yield q = 0.0.
pub fn compute_min_face_quality(vertices: &[Vec<f64>], faces: &[Vec<usize>]) -> f64 {
    if vertices.is_empty() || faces.is_empty() {
        return 1.0;
    }
    faces
        .par_iter()
        .map(|face| {
            let tris = triangulate_face(vertices, face);
            if tris.is_empty() {
                return 0.0;
            }
            let mut min_tri_q = 1.0;
            for tri in tris {
                let p0 = &vertices[tri[0] as usize];
                let p1 = &vertices[tri[1] as usize];
                let p2 = &vertices[tri[2] as usize];

                let a =
                    ((p0[0] - p1[0]).powi(2) + (p0[1] - p1[1]).powi(2) + (p0[2] - p1[2]).powi(2))
                        .sqrt();
                let b =
                    ((p1[0] - p2[0]).powi(2) + (p1[1] - p2[1]).powi(2) + (p1[2] - p2[2]).powi(2))
                        .sqrt();
                let c =
                    ((p2[0] - p0[0]).powi(2) + (p2[1] - p0[1]).powi(2) + (p2[2] - p0[2]).powi(2))
                        .sqrt();

                if a < 1e-12 || b < 1e-12 || c < 1e-12 {
                    min_tri_q = 0.0;
                    continue;
                }

                // Heron's formula for triangle area
                let s = (a + b + c) / 2.0;
                let area_sq = s * (s - a) * (s - b) * (s - c);
                let area = if area_sq > 0.0 { area_sq.sqrt() } else { 0.0 };

                let q = (4.0 * 3.0f64.sqrt() * area) / (a * a + b * b + c * c);
                if q < min_tri_q {
                    min_tri_q = q;
                }
            }
            min_tri_q
        })
        .min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
        .unwrap_or(1.0)
}

/// Extract boundary (naked) edges from edge-incidence counts.
pub fn find_boundary_edges(faces: &[Vec<usize>]) -> Vec<(usize, usize)> {
    if faces.is_empty() {
        return Vec::new();
    }

    let mut edge_occurrences = HashMap::new();
    for face in faces {
        let len = face.len();
        if len < 3 {
            continue;
        }
        for i in 0..len {
            let u = face[i];
            let v = face[(i + 1) % len];
            let edge = if u < v { (u, v) } else { (v, u) };
            *edge_occurrences.entry(edge).or_insert(0) += 1;
        }
    }

    let mut edges: Vec<_> = edge_occurrences
        .into_iter()
        .filter(|&(_, count)| count == 1)
        .map(|(edge, _)| edge)
        .collect();
    edges.sort_unstable();
    edges
}

/// High-performance parallel detection of non-manifold topology edges
pub fn find_non_manifold_edges(faces: &[Vec<usize>]) -> Vec<(usize, usize)> {
    if faces.is_empty() {
        return Vec::new();
    }

    let edge_occurrences: HashMap<(usize, usize), usize> = faces
        .par_iter()
        .flat_map(|face| {
            let mut edges = Vec::new();
            let len = face.len();
            if len < 3 {
                return edges;
            }
            for i in 0..len {
                let u = face[i];
                let v = face[(i + 1) % len];
                let edge = if u < v { (u, v) } else { (v, u) };
                edges.push(edge);
            }
            edges
        })
        .fold(HashMap::new, |mut acc, edge| {
            *acc.entry(edge).or_insert(0) += 1;
            acc
        })
        .reduce(HashMap::new, |mut map1, map2| {
            for (edge, count) in map2 {
                *map1.entry(edge).or_insert(0) += count;
            }
            map1
        });

    let mut edges: Vec<_> = edge_occurrences
        .into_par_iter()
        .filter(|&(_, count)| count > 2)
        .map(|(edge, _)| edge)
        .collect();
    edges.sort_unstable();
    edges
}

/// Audit log representing a single welded vertex operation
#[derive(Debug, serde::Serialize, Clone)]
pub struct WeldAudit {
    pub old_index: usize,
    pub merged_into: usize,
    pub coordinates: Vec<f64>,
    pub distance: f64,
}

/// Weld vertices using a deterministic spatial hash and Euclidean tolerance.
pub fn weld_vertices(
    vertices: &[Vec<f64>],
    faces: &[Vec<usize>],
    tolerance: f64,
) -> (Vec<Vec<f64>>, Vec<Vec<usize>>, Vec<WeldAudit>) {
    if vertices.is_empty() {
        return (Vec::new(), Vec::new(), Vec::new());
    }

    let mut unique_vertices = Vec::new();
    let mut cells: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    let mut old_to_new = vec![0; vertices.len()];
    let mut weld_audit_logs = Vec::new();

    for (old_idx, v) in vertices.iter().enumerate() {
        if v.len() < 3 {
            continue;
        }
        let cell = (
            (v[0] / tolerance).floor() as i64,
            (v[1] / tolerance).floor() as i64,
            (v[2] / tolerance).floor() as i64,
        );
        let mut match_index = None;
        let mut match_distance = f64::INFINITY;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(candidates) = cells.get(&(cell.0 + dx, cell.1 + dy, cell.2 + dz)) {
                        for &candidate in candidates {
                            let u: &Vec<f64> = &unique_vertices[candidate];
                            let distance = ((u[0] - v[0]).powi(2)
                                + (u[1] - v[1]).powi(2)
                                + (u[2] - v[2]).powi(2))
                            .sqrt();
                            if distance <= tolerance
                                && (match_index.is_none() || candidate < match_index.unwrap())
                            {
                                match_index = Some(candidate);
                                match_distance = distance;
                            }
                        }
                    }
                }
            }
        }
        if let Some(merged_idx) = match_index {
            old_to_new[old_idx] = merged_idx;
            weld_audit_logs.push(WeldAudit {
                old_index: old_idx,
                merged_into: merged_idx,
                coordinates: v.clone(),
                distance: match_distance,
            });
        } else {
            let new_idx = unique_vertices.len();
            unique_vertices.push(v.clone());
            cells.entry(cell).or_default().push(new_idx);
            old_to_new[old_idx] = new_idx;
        }
    }

    let new_faces: Vec<Vec<usize>> = faces
        .iter()
        .map(|face| face.iter().map(|&old_idx| old_to_new[old_idx]).collect())
        .collect();

    (unique_vertices, new_faces, weld_audit_logs)
}

/// Audit log representing a single face flipped operation
#[derive(Debug, serde::Serialize, Clone)]
pub struct FlipAudit {
    pub face_index: usize,
    pub old_winding: Vec<usize>,
    pub new_winding: Vec<usize>,
}

/// Unify adjacent face winding with a dual-graph traversal and retain an audit trail.
pub fn unify_winding_directions(mut faces: Vec<Vec<usize>>) -> (Vec<Vec<usize>>, Vec<FlipAudit>) {
    if faces.is_empty() {
        return (faces, Vec::new());
    }

    let num_faces = faces.len();
    let mut edge_to_faces = HashMap::new();
    for (f_idx, face) in faces.iter().enumerate() {
        let len = face.len();
        if len < 3 {
            continue;
        }
        for i in 0..len {
            let u = face[i];
            let v = face[(i + 1) % len];
            let edge = if u < v { (u, v) } else { (v, u) };
            edge_to_faces
                .entry(edge)
                .or_insert_with(Vec::new)
                .push(f_idx);
        }
    }

    let mut visited = vec![false; num_faces];
    let mut queue = VecDeque::new();
    let mut flip_audit_logs = Vec::new();

    for start_face in 0..num_faces {
        if visited[start_face] {
            continue;
        }

        visited[start_face] = true;
        queue.push_back(start_face);

        while let Some(curr_idx) = queue.pop_front() {
            let curr_face = faces[curr_idx].clone();
            let len = curr_face.len();
            if len < 3 {
                continue;
            }

            for i in 0..len {
                let u = curr_face[i];
                let v = curr_face[(i + 1) % len];
                let undirected_edge = if u < v { (u, v) } else { (v, u) };

                if let Some(neighbors) = edge_to_faces.get(&undirected_edge) {
                    for &neigh_idx in neighbors {
                        if visited[neigh_idx] {
                            continue;
                        }

                        let neigh_face = &faces[neigh_idx];
                        let n_len = neigh_face.len();
                        if n_len < 3 {
                            continue;
                        }

                        let mut edge_found = false;
                        let mut same_direction = false;

                        for j in 0..n_len {
                            let nu = neigh_face[j];
                            let nv = neigh_face[(j + 1) % n_len];
                            if (nu == u && nv == v) || (nu == v && nv == u) {
                                edge_found = true;
                                if nu == u && nv == v {
                                    same_direction = true;
                                }
                                break;
                            }
                        }

                        if edge_found {
                            if same_direction {
                                let old_winding = faces[neigh_idx].clone();
                                faces[neigh_idx].reverse();
                                let new_winding = faces[neigh_idx].clone();

                                flip_audit_logs.push(FlipAudit {
                                    face_index: neigh_idx,
                                    old_winding,
                                    new_winding,
                                });
                            }
                            visited[neigh_idx] = true;
                            queue.push_back(neigh_idx);
                        }
                    }
                }
            }
        }
    }

    (faces, flip_audit_logs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        #[test]
        fn welding_merges_any_pair_inside_euclidean_tolerance(
            x in -1.0e3f64..1.0e3,
            y in -1.0e3f64..1.0e3,
            z in -1.0e3f64..1.0e3,
            fraction in 0.0f64..=1.0,
        ) {
            let tolerance = 1.0e-6;
            let vertices = vec![
                vec![x, y, z],
                vec![x + tolerance * fraction, y, z],
                vec![x + 1.0, y, z],
            ];
            let (welded, _, audit) = weld_vertices(&vertices, &[], tolerance);
            prop_assert_eq!(welded.len(), 2);
            prop_assert_eq!(audit.len(), 1);
            prop_assert!(audit[0].distance <= tolerance);
        }

        #[test]
        fn tetrahedron_volume_is_translation_invariant(
            tx in -1.0e3f64..1.0e3,
            ty in -1.0e3f64..1.0e3,
            tz in -1.0e3f64..1.0e3,
        ) {
            let vertices = vec![
                vec![tx, ty, tz],
                vec![tx + 1.0, ty, tz],
                vec![tx, ty + 1.0, tz],
                vec![tx, ty, tz + 1.0],
            ];
            let faces = vec![
                vec![0, 2, 1], vec![0, 1, 3], vec![1, 2, 3], vec![2, 0, 3],
            ];
            prop_assert!((compute_mesh_volume(&vertices, &faces) - 1.0 / 6.0).abs() < 1e-10);
        }
    }

    #[test]
    fn triangulates_concave_polygon_without_losing_area() {
        let vertices = vec![
            vec![0.0, 0.0, 0.0],
            vec![3.0, 0.0, 0.0],
            vec![3.0, 3.0, 0.0],
            vec![2.0, 3.0, 0.0],
            vec![2.0, 1.0, 0.0],
            vec![1.0, 1.0, 0.0],
            vec![1.0, 3.0, 0.0],
            vec![0.0, 3.0, 0.0],
        ];
        let triangles = triangulate_face(&vertices, &(0..8).collect::<Vec<_>>());
        let area: f64 = triangles
            .iter()
            .map(|triangle| {
                let a = &vertices[triangle[0] as usize];
                let b = &vertices[triangle[1] as usize];
                let c = &vertices[triangle[2] as usize];
                ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])).abs() / 2.0
            })
            .sum();
        assert!((area - 7.0).abs() < 1e-12);
    }

    #[test]
    fn detects_open_and_closed_boundary_topology() {
        assert_eq!(find_boundary_edges(&[vec![0, 1, 2]]).len(), 3);
        let tetrahedron = vec![vec![0, 2, 1], vec![0, 1, 3], vec![1, 2, 3], vec![2, 0, 3]];
        assert!(find_boundary_edges(&tetrahedron).is_empty());
    }

    #[test]
    fn welding_remaps_faces_and_records_lineage() {
        let vertices = vec![
            vec![0.0, 0.0, 0.0],
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 0.0],
        ];
        let (welded, faces, audit) = weld_vertices(&vertices, &[vec![3, 1, 2]], 1e-6);
        assert_eq!(welded.len(), 3);
        assert_eq!(faces, vec![vec![0, 1, 2]]);
        assert_eq!(audit.len(), 1);
        assert_eq!(audit[0].old_index, 3);
        assert_eq!(audit[0].merged_into, 0);
        assert_eq!(audit[0].distance, 0.0);
    }

    #[test]
    fn welding_uses_euclidean_tolerance_across_grid_boundaries() {
        let vertices = vec![
            vec![0.99e-6, 0.0, 0.0],
            vec![1.01e-6, 0.0, 0.0],
            vec![1.0, 0.0, 0.0],
        ];
        let (welded, _, audit) = weld_vertices(&vertices, &[], 1e-6);
        assert_eq!(welded.len(), 2);
        assert_eq!(audit.len(), 1);
        assert!((audit[0].distance - 0.02e-6).abs() < 1e-15);
    }

    #[test]
    fn disconnected_shell_volumes_do_not_cancel() {
        let mut vertices = vec![
            vec![0.0, 0.0, 0.0],
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ];
        vertices.extend(vertices.clone().into_iter().map(|mut point| {
            point[0] += 2.0;
            point
        }));
        let first = vec![vec![0, 2, 1], vec![0, 1, 3], vec![1, 2, 3], vec![2, 0, 3]];
        let mut faces = first.clone();
        faces.extend(
            first
                .into_iter()
                .map(|face| face.into_iter().rev().map(|index| index + 4).collect()),
        );

        assert_eq!(count_face_components(&faces), 2);
        assert!((compute_mesh_volume(&vertices, &faces) - 1.0 / 3.0).abs() < 1e-12);
    }

    #[test]
    fn detects_non_adjacent_triangle_self_intersection() {
        let vertices = vec![
            vec![-1.0, -1.0, 0.0],
            vec![1.0, -1.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, -0.5, -1.0],
            vec![0.0, -0.5, 1.0],
            vec![0.0, 0.5, 0.0],
        ];
        let faces = vec![vec![0, 1, 2], vec![3, 4, 5]];
        assert_eq!(find_self_intersections(&vertices, &faces), vec![(0, 1)]);
    }

    #[test]
    fn winding_unifier_flips_same_direction_neighbor() {
        let (faces, audit) = unify_winding_directions(vec![vec![0, 1, 2], vec![0, 1, 3]]);
        assert_eq!(audit.len(), 1);
        assert_eq!(faces[1], vec![3, 1, 0]);
    }
}
