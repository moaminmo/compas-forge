use serde::{Deserialize, Serialize};
use std::collections::HashMap;

pub type MeshArrays = (Vec<Vec<f64>>, Vec<Vec<usize>>);

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct CompasDataObject {
    #[serde(rename = "dtype")]
    pub data_type: String,
    pub data: CompasDataPayload,
    #[serde(default)]
    pub guid: String,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct CompasDataPayload {
    // COMPAS 1.x schema representations (Plural)
    #[serde(default)]
    pub vertices: Option<Vec<Vec<f64>>>,
    #[serde(default)]
    pub faces: Option<Vec<Vec<usize>>>,

    // COMPAS 2.x schema representations (Singular map structures)
    #[serde(default)]
    pub vertex: Option<HashMap<String, HashMap<String, f64>>>,
    #[serde(default)]
    pub face: Option<HashMap<String, Vec<usize>>>,

    #[serde(default)]
    pub attributes: Option<HashMap<String, serde_json::Value>>,
}

impl CompasDataPayload {
    /// Dynamically resolves and unifies the mesh geometry representation
    /// across both COMPAS 1.x and COMPAS 2.x database schemas.
    pub fn get_vertices_and_faces(&self) -> Result<MeshArrays, String> {
        if let (Some(vertices), Some(faces)) = (&self.vertices, &self.faces) {
            return Ok((vertices.clone(), faces.clone()));
        }
        if self.vertices.is_some() || self.faces.is_some() {
            return Err("COMPAS array schema must provide both 'vertices' and 'faces'".to_string());
        }

        let vertex_map = self
            .vertex
            .as_ref()
            .ok_or_else(|| "COMPAS map schema is missing 'vertex'".to_string())?;
        let face_map = self
            .face
            .as_ref()
            .ok_or_else(|| "COMPAS map schema is missing 'face'".to_string())?;
        let mut vertex_keys: Vec<usize> = vertex_map
            .keys()
            .map(|key| {
                key.parse::<usize>()
                    .map_err(|_| format!("vertex key '{key}' is not a nonnegative integer"))
            })
            .collect::<Result<_, _>>()?;
        vertex_keys.sort_unstable();

        let mut remap = HashMap::with_capacity(vertex_keys.len());
        let mut vertices = Vec::with_capacity(vertex_keys.len());
        for key in vertex_keys {
            let coordinates = &vertex_map[&key.to_string()];
            let coordinate = |axis: &str| {
                coordinates
                    .get(axis)
                    .copied()
                    .ok_or_else(|| format!("vertex {key} is missing coordinate '{axis}'"))
            };
            let xyz = vec![coordinate("x")?, coordinate("y")?, coordinate("z")?];
            remap.insert(key, vertices.len());
            vertices.push(xyz);
        }

        let mut face_keys: Vec<usize> = face_map
            .keys()
            .map(|key| {
                key.parse::<usize>()
                    .map_err(|_| format!("face key '{key}' is not a nonnegative integer"))
            })
            .collect::<Result<_, _>>()?;
        face_keys.sort_unstable();
        let mut faces = Vec::with_capacity(face_keys.len());
        for key in face_keys {
            let mut face = Vec::with_capacity(face_map[&key.to_string()].len());
            for original_index in &face_map[&key.to_string()] {
                face.push(*remap.get(original_index).ok_or_else(|| {
                    format!("face {key} references missing vertex {original_index}")
                })?);
            }
            faces.push(face);
        }
        Ok((vertices, faces))
    }
}

#[derive(Debug, Serialize)]
pub struct ValidationResult {
    pub is_valid: bool,
    pub vertex_count: usize,
    pub face_count: usize,
    pub non_manifold_edges: Vec<(usize, usize)>,
    pub non_manifold_vertices: Vec<usize>,
    pub duplicate_vertices: usize,
    pub duplicate_tolerance: f64,
    pub degenerate_faces_count: usize,
    pub winding_consistent: bool,
    pub self_intersections: Vec<(usize, usize)>,
    pub boundary_edges_count: usize,
    pub bounding_box: crate::geometry::Aabb,
}
