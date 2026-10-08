//! Restricted solid contract: embedded closed shells, with explicit fill rule.
//! Winding-number classification is floating point, not an exact predicate.
use super::*;

fn eligibility(mesh: &RegisteredMesh) -> Option<&'static str> {
    *mesh.solid_issue.get_or_init(|| {
        let vertices: Vec<Vec<f64>> = mesh
            .mesh
            .vertices()
            .iter()
            .map(|v| v.to_array().to_vec())
            .collect();
        let faces: Vec<Vec<usize>> = mesh
            .mesh
            .indices()
            .iter()
            .map(|f| f.iter().map(|i| *i as usize).collect())
            .collect();
        let used: HashSet<usize> = faces.iter().flatten().copied().collect();
        if used.len() != vertices.len() {
            return Some("solid_isolated_vertices");
        }
        if !find_boundary_edges(&faces).is_empty()
            || !find_non_manifold_edges(&faces).is_empty()
            || !geometry::find_non_manifold_vertices(&faces).is_empty()
        {
            return Some("solid_requires_closed_manifold");
        }
        if !has_consistent_winding(&faces) {
            return Some("solid_requires_consistent_winding");
        }
        if count_degenerate_faces(&vertices, &faces, DEGENERATE_AREA_TOLERANCE) != 0 {
            return Some("solid_degenerate_triangles");
        }
        if !find_self_intersections(&vertices, &faces).is_empty() {
            return Some("solid_self_intersection");
        }
        None
    })
}

fn representatives(mesh: &RegisteredMesh) -> &[usize] {
    mesh.solid_representatives.get_or_init(|| {
        let mut neighbors = vec![Vec::new(); mesh.mesh.vertices().len()];
        for face in mesh.mesh.indices() {
            for k in 0..3 {
                let (a, b) = (face[k] as usize, face[(k + 1) % 3] as usize);
                neighbors[a].push(b);
                neighbors[b].push(a);
            }
        }
        let mut visited = vec![false; neighbors.len()];
        let mut result = Vec::new();
        for seed in 0..neighbors.len() {
            if visited[seed] {
                continue;
            }
            result.push(seed);
            visited[seed] = true;
            let mut stack = vec![seed];
            while let Some(v) = stack.pop() {
                for &next in &neighbors[v] {
                    if !visited[next] {
                        visited[next] = true;
                        stack.push(next);
                    }
                }
            }
        }
        result
    })
}

// Signed solid angles, normalized directions and compensated summation.
fn inside(mesh: &TriMesh, point: Vector, shell_count: usize) -> Option<bool> {
    let mut sum = 0.0;
    let mut correction = 0.0;
    for f in mesh.indices() {
        let mut u = [Vector::ZERO; 3];
        for i in 0..3 {
            let v = mesh.vertices()[f[i] as usize] - point;
            let norm = v.length();
            if norm <= 0.0 || !norm.is_finite() {
                return None;
            }
            u[i] = v / norm;
        }
        let numerator = u[0].dot(u[1].cross(u[2]));
        let denominator = 1.0 + u[0].dot(u[1]) + u[1].dot(u[2]) + u[2].dot(u[0]);
        let angle = 2.0 * numerator.atan2(denominator);
        let y = angle - correction;
        let t = sum + y;
        correction = (t - sum) - y;
        sum = t;
    }
    let winding = (sum / (4.0 * std::f64::consts::PI)).abs();
    // Each embedded oriented shell contributes 0 or +/-1. Parity of their
    // integer sum is orientation-independent: nesting alternates void/material.
    let integer = winding.round();
    if !winding.is_finite() || integer > shell_count as f64 || (winding - integer).abs() > 1e-8 {
        None
    } else {
        Some(integer % 2.0 == 1.0)
    }
}

pub(super) fn initial_relationship(
    q: &PreparedCachedSweep,
    margin: f64,
    even_odd: bool,
) -> PyResult<(&'static str, usize)> {
    if let Some(issue) = eligibility(&q.mesh1).or_else(|| eligibility(&q.mesh2)) {
        return Ok((issue, 0));
    }
    let reps_a = representatives(&q.mesh1);
    let reps_b = representatives(&q.mesh2);
    if !even_odd && (reps_a.len() != 1 || reps_b.len() != 1) {
        return Ok(("solid_requires_single_shell", 0));
    }
    let a = &q.pose1_start;
    let b = &q.pose2_start;
    let d = distance(a, &q.mesh1.mesh, b, &q.mesh2.mesh)
        .map_err(|e| PyValueError::new_err(format!("Unsupported solid distance: {e:?}")))?
        .distance;
    if !d.is_finite() || d < 0.0 {
        return Ok(("solid_invalid_distance", 1));
    }
    if d <= margin {
        return Ok(("surface_contact_or_crossing", 1));
    }
    // With disjoint boundaries, membership is constant over each connected
    // shell. One representative per shell is necessary, not just per mesh.
    let classify = |source: &RegisteredMesh,
                    sp: &Pose,
                    reps: &[usize],
                    target: &RegisteredMesh,
                    tp: &Pose,
                    target_count: usize| {
        let mut contained = false;
        for &index in reps {
            let world = sp.rotation.mul_vec3(source.mesh.vertices()[index]) + sp.translation;
            let local = tp.rotation.conjugate().mul_vec3(world - tp.translation);
            match inside(&target.mesh, local, target_count) {
                Some(value) => contained |= value,
                None => return None,
            }
        }
        Some(contained)
    };
    let a_in_b = classify(&q.mesh1, a, reps_a, &q.mesh2, b, reps_b.len());
    let b_in_a = classify(&q.mesh2, b, reps_b, &q.mesh1, a, reps_a.len());
    Ok((
        match (a_in_b, b_in_a) {
            (Some(true), Some(false)) => "a_inside_b",
            (Some(false), Some(true)) => "b_inside_a",
            (Some(false), Some(false)) => "disjoint",
            (Some(true), Some(true)) if even_odd => "mutual_component_containment",
            _ => "solid_ambiguous_winding",
        },
        1,
    ))
}
