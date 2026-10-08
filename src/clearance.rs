//! Interval clearance verification for linear translation + shortest-arc rotation.
//! The distance oracle and floating point operations are not interval arithmetic.
//! A clear result is conditional on the caller's numerical-error allowance.

use super::*;

struct ClearanceResult {
    status: &'static str,
    reason: &'static str,
    evaluations: usize,
    witness_time: Option<f64>,
    witness_distance: Option<f64>,
    unresolved: Option<(f64, f64)>,
    speed_bound: f64,
    initial_solid_relationship: Option<&'static str>,
}

fn angular_distance(start: &Pose, end: &Pose) -> f64 {
    let mut delta = (end.rotation * start.rotation.conjugate()).normalize();
    if delta.w < 0.0 {
        delta = -delta;
    }
    delta.to_scaled_axis().length()
}

#[allow(clippy::too_many_arguments)]
fn verify(
    query: &PreparedCachedSweep,
    clearance: f64,
    numerical_margin: f64,
    time_tolerance: f64,
    max_evaluations: usize,
    solid: bool,
    even_odd: bool,
    broadphase: bool,
) -> PyResult<ClearanceResult> {
    let motion_a = rigid_motion(&query.pose1_start, &query.pose1_end);
    let motion_b = rigid_motion(&query.pose2_start, &query.pose2_end);
    // Translation common to both objects cancels. Rotation about each origin
    // displaces any mesh point at at most |omega| * max vertex radius.
    let relative_translation = (query.pose1_end.translation - query.pose1_start.translation)
        - (query.pose2_end.translation - query.pose2_start.translation);
    let speed_bound = relative_translation.length()
        + angular_distance(&query.pose1_start, &query.pose1_end) * query.mesh1.origin_radius
        + angular_distance(&query.pose2_start, &query.pose2_end) * query.mesh2.origin_radius;
    if !speed_bound.is_finite() {
        return Err(PyValueError::new_err(
            "nonfinite motion bound; rescale input geometry",
        ));
    }
    let mut result = ClearanceResult {
        status: "clear",
        reason: "intervals_separated_under_error_assumption",
        evaluations: 0,
        witness_time: None,
        witness_distance: None,
        unresolved: None,
        speed_bound,
        initial_solid_relationship: None,
    };
    if solid {
        let (relation, evaluations) =
            solid::initial_relationship(query, numerical_margin, even_odd)?;
        result.evaluations += evaluations;
        result.initial_solid_relationship = Some(relation);
        match relation {
            "disjoint" => {}
            "a_inside_b" | "b_inside_a" | "mutual_component_containment" => {
                result.status = "violation";
                result.reason = "initial_solid_containment_under_contract";
                result.witness_time = Some(0.0);
                result.witness_distance = Some(0.0);
                return Ok(result);
            }
            "surface_contact_or_crossing" if 2.0 * numerical_margin < clearance => {
                result.status = "violation";
                result.reason = "initial_surface_contact_under_error_assumption";
                result.witness_time = Some(0.0);
                // Contact distance was not necessarily exactly zero.
                return Ok(result);
            }
            _ => {
                result.status = "unknown";
                result.reason = relation;
                result.unresolved = Some((0.0, 1.0));
                return Ok(result);
            }
        }
    }
    if broadphase {
        let (alo, ahi) =
            bounds::swept_endpoint_bounds(&query.mesh1, &query.pose1_start, &query.pose1_end);
        let (blo, bhi) =
            bounds::swept_endpoint_bounds(&query.mesh2, &query.pose2_start, &query.pose2_end);
        let gaps: [f64; 3] =
            std::array::from_fn(|i| (alo[i] - bhi[i]).max(blo[i] - ahi[i]).max(0.0));
        let separation = gaps[0].hypot(gaps[1]).hypot(gaps[2]);
        if separation.is_finite() && separation - numerical_margin > clearance {
            result.reason = "swept_aabb_separation_under_error_assumption";
            return Ok(result);
        }
    }
    let mut stack = vec![(0.0, 1.0)];
    while let Some((lo, hi)) = stack.pop() {
        if result.evaluations >= max_evaluations {
            result.status = "unknown";
            result.reason = "evaluation_budget_exhausted";
            result.unresolved = Some((lo, hi));
            return Ok(result);
        }
        let mid = lo + (hi - lo) * 0.5;
        let pa = motion_a.position_at_time(mid);
        let pb = motion_b.position_at_time(mid);
        let d = distance(&pa, &query.mesh1.mesh, &pb, &query.mesh2.mesh)
            .map_err(|e| PyValueError::new_err(format!("Unsupported clearance distance: {e:?}")))?
            .distance;
        result.evaluations += 1;
        if !d.is_finite() || d < 0.0 {
            result.status = "unknown";
            result.reason = "invalid_distance_oracle";
            result.unresolved = Some((lo, hi));
            return Ok(result);
        }
        if d + numerical_margin < clearance {
            result.status = "violation";
            result.reason = "sample_below_clearance_under_error_assumption";
            result.witness_time = Some(mid);
            result.witness_distance = Some(d);
            // This is a sampled witness, deliberately not called first impact.
            result.unresolved = None;
            return Ok(result);
        }
        let lower_bound = d - numerical_margin - speed_bound * (hi - lo) * 0.5;
        if lower_bound > clearance {
            continue;
        }
        if hi - lo <= time_tolerance || mid <= lo || mid >= hi {
            result.status = "unknown";
            result.reason = "resolution_limit";
            result.unresolved.get_or_insert((lo, hi));
            continue;
        }
        // Depth-first chronological traversal has bounded stack size and stable output.
        stack.push((mid, hi));
        stack.push((lo, mid));
    }
    Ok(result)
}

#[pyfunction(signature = (queries, clearance, numerical_margin=1e-8, time_tolerance=1e-5, max_evaluations=4096, parallel=true, solid=false, clearance_offsets=None, solid_rule="single_shell", broadphase=true))]
#[allow(clippy::too_many_arguments)] // Preserve the public Python keyword policy.
pub(super) fn verify_clearance_cached_batch_native(
    py: Python<'_>,
    queries: Vec<CachedSweepInput>,
    clearance: f64,
    numerical_margin: f64,
    time_tolerance: f64,
    max_evaluations: usize,
    parallel: bool,
    solid: bool,
    clearance_offsets: Option<Vec<f64>>,
    solid_rule: &str,
    broadphase: bool,
) -> PyResult<Vec<Py<PyDict>>> {
    let even_odd = match solid_rule {
        "single_shell" => false,
        "even_odd" if solid => true,
        _ => {
            return Err(PyValueError::new_err(
                "solid_rule must be single_shell, or even_odd with solid=True",
            ))
        }
    };
    if !clearance.is_finite() || clearance <= 0.0 {
        return Err(PyValueError::new_err(
            "clearance must be finite and positive",
        ));
    }
    if !numerical_margin.is_finite() || numerical_margin < 0.0 {
        return Err(PyValueError::new_err(
            "numerical_margin must be finite and nonnegative",
        ));
    }
    if !time_tolerance.is_finite() || time_tolerance <= 0.0 || time_tolerance > 1.0 {
        return Err(PyValueError::new_err("time_tolerance must be in (0, 1]"));
    }
    if max_evaluations == 0 {
        return Err(PyValueError::new_err("max_evaluations must be positive"));
    }
    let offsets = clearance_offsets.unwrap_or_else(|| vec![0.0; queries.len()]);
    if offsets.len() != queries.len()
        || offsets
            .iter()
            .any(|v| !v.is_finite() || *v < 0.0 || !(clearance + v).is_finite())
    {
        return Err(PyValueError::new_err(
            "clearance_offsets must contain one finite nonnegative offset per query, without overflow",
        ));
    }
    let prepared = prepare_cached_sweeps(queries)?;
    let execute = |(q, offset): (&PreparedCachedSweep, &f64)| {
        verify(
            q,
            clearance + offset,
            numerical_margin,
            time_tolerance,
            max_evaluations,
            solid,
            even_odd,
            broadphase,
        )
    };
    let results = py.detach(|| {
        if parallel {
            prepared
                .par_iter()
                .zip(offsets.par_iter())
                .map(execute)
                .collect::<PyResult<Vec<_>>>()
        } else {
            prepared
                .iter()
                .zip(offsets.iter())
                .map(execute)
                .collect::<PyResult<Vec<_>>>()
        }
    })?;
    results
        .into_iter()
        .zip(offsets)
        .map(|(r, offset)| {
            let out = PyDict::new(py);
            out.set_item("status", r.status)?;
            out.set_item("reason", r.reason)?;
            out.set_item("clearance", clearance + offset)?;
            out.set_item("clearance_offset", offset)?;
            out.set_item("numerical_margin", numerical_margin)?;
            out.set_item("time_tolerance", time_tolerance)?;
            out.set_item("distance_evaluations", r.evaluations)?;
            out.set_item(
                "broadphase_rejected",
                r.reason == "swept_aabb_separation_under_error_assumption",
            )?;
            out.set_item("witness_time", r.witness_time)?;
            out.set_item("witness_distance", r.witness_distance)?;
            out.set_item("unresolved_interval", r.unresolved)?;
            out.set_item("relative_speed_bound", r.speed_bound)?;
            out.set_item("initial_solid_relationship", r.initial_solid_relationship)?;
            out.set_item("solid_input_certified", false)?;
            out.set_item("solid_rule", solid_rule)?;
            out.set_item("method", "lipschitz_interval_subdivision")?;
            out.set_item(
                "distance_semantics",
                if even_odd {
                    "even_odd_shell_solid_clearance_under_contract"
                } else if solid {
                    "single_shell_solid_clearance_under_contract"
                } else {
                    "surface_distance_not_solid_containment"
                },
            )?;
            out.set_item("numerical_error_bound_proven", false)?;
            Ok(out.unbind())
        })
        .collect()
}
