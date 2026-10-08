//! Endpoint AABBs inflated by the rigid arc/chord deviation.
//! In exact arithmetic: ||x(t)-lerp(x(0),x(1),t)|| <= r*theta^2/8.
//! Floating-point padding is defensive, not an interval-arithmetic certificate.
use super::*;

pub(super) fn swept_endpoint_bounds(
    mesh: &RegisteredMesh,
    start: &Pose,
    end: &Pose,
) -> ([f64; 3], [f64; 3]) {
    let mut delta = (end.rotation * start.rotation.conjugate()).normalize();
    if delta.w < 0.0 {
        delta = -delta;
    }
    let theta = delta.to_scaled_axis().length();
    let scale = mesh.origin_radius + start.translation.length() + end.translation.length() + 1.0;
    let padding = mesh.origin_radius * theta * theta / 8.0 + 128.0 * f64::EPSILON * scale;
    // Overflow or invalid intermediate arithmetic must not reject a pair.
    if !padding.is_finite() {
        return ([f64::NEG_INFINITY; 3], [f64::INFINITY; 3]);
    }
    let mut mins = [f64::INFINITY; 3];
    let mut maxs = [f64::NEG_INFINITY; 3];
    for bits in 0..8 {
        let corner = Vector::from_array(std::array::from_fn(|i| {
            if bits & (1 << i) == 0 {
                mesh.local_bounds.0[i]
            } else {
                mesh.local_bounds.1[i]
            }
        }));
        for pose in [start, end] {
            let p = pose.rotation.mul_vec3(corner) + pose.translation;
            if !p.is_finite() {
                return ([f64::NEG_INFINITY; 3], [f64::INFINITY; 3]);
            }
            for i in 0..3 {
                mins[i] = mins[i].min(p[i]);
                maxs[i] = maxs[i].max(p[i]);
            }
        }
    }
    for i in 0..3 {
        mins[i] -= padding;
        maxs[i] += padding;
    }
    (mins, maxs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        #[test]
        fn bounds_enclose_rotating_translating_vertices(
            angle in -std::f64::consts::PI..std::f64::consts::PI,
            tx in -20.0f64..20.0, ty in -20.0f64..20.0,
            length in 0.01f64..30.0,
            ax in -1.0f64..1.0, ay in -1.0f64..1.0,
        ) {
            let vertices = vec![Vector::new(-length,-0.1,0.0),Vector::new(length,-0.1,0.0),Vector::new(length,0.1,0.0)];
            let mesh = RegisteredMesh {
                origin_radius: (length*length+0.01).sqrt(),
                local_bounds: ([-length,-0.1,0.0],[length,0.1,0.0]),
                solid_issue: OnceLock::new(),
                solid_representatives: OnceLock::new(),
                mesh: TriMesh::new(vertices.clone(),vec![[0,1,2]]).unwrap(),
            };
            let start = Pose::from_parts(Vector::new(1.0,-2.0,3.0),Rotation::from_scaled_axis(Vector::new(0.2,-0.4,0.3)));
            let axis = Vector::new(ax,ay,1.0).normalize();
            let end = Pose::from_parts(Vector::new(tx,ty,0.0),Rotation::from_scaled_axis(axis*angle)*start.rotation);
            let (lo,hi) = swept_endpoint_bounds(&mesh,&start,&end);
            let motion = rigid_motion(&start,&end);
            for step in 0..101 {
                let p = motion.position_at_time(step as f64/100.0);
                for v in &vertices {
                    let w = p.rotation.mul_vec3(*v)+p.translation;
                    for i in 0..3 { prop_assert!(lo[i] <= w[i] && w[i] <= hi[i]); }
                }
            }
        }
    }

    #[test]
    fn rejected_pairs_agree_with_unfiltered_narrowphase() {
        let vertices = vec![
            Vector::new(-5.0, -0.1, 0.0),
            Vector::new(5.0, -0.1, 0.0),
            Vector::new(5.0, 0.1, 0.0),
        ];
        let mesh = RegisteredMesh {
            origin_radius: 5.01,
            local_bounds: ([-5.0, -0.1, 0.0], [5.0, 0.1, 0.0]),
            solid_issue: OnceLock::new(),
            solid_representatives: OnceLock::new(),
            mesh: TriMesh::new(vertices, vec![[0, 1, 2]]).unwrap(),
        };
        let a = Pose::identity();
        let mut rejected = 0;
        for step in 1..50 {
            let angle = step as f64 * 0.002;
            let b = Pose::from_parts(Vector::new(0.0, 2.0, 0.0), Rotation::IDENTITY);
            let a1 = Pose::from_parts(
                Vector::ZERO,
                Rotation::from_scaled_axis(Vector::new(0.0, 0.0, angle)),
            );
            let b1 = Pose::from_parts(b.translation, a1.rotation);
            let filtered = sweep_registered_meshes(&mesh, &a, &a1, &mesh, &b, &b1, true).unwrap();
            if filtered.method == "swept_endpoint_aabb_rejected" {
                rejected += 1;
                let reference = sweep_meshes(&mesh.mesh, &a, &a1, &mesh.mesh, &b, &b1).unwrap();
                assert!(!reference.has_collision);
            }
        }
        assert_eq!(rejected, 49);
    }
}
