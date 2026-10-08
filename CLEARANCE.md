# Continuous surface-clearance verification

New opt-in modes: `solid=True` checks initial containment for a restricted
single-shell solid contract. On prepared cells including fixed-state rigid attachments,
`articulated_tolerance=...` bounds linear-joint-path deviation and inflates
clearance accordingly. See [ROBUSTNESS.md](ROBUSTNESS.md) for derivations,
unsupported inputs and the distinction between geometric and numerical proof.
The surface-only and piecewise-rigid descriptions below apply to the defaults.

Additional options: `solid_rule='even_odd'` (requires `solid=True`) specifies
cavity semantics for disjoint/nested embedded shells. `clearance_offsets=[...]`
adds per-pair allowances. `broadphase=False` disables the swept-AABB clearance
filter for ablation. Prepared-cell `max_articulated_refinements=2` controls local
refinement; `scene_poses` supplies synchronized paths for unattached bodies and
fixed-shape tools. See ROBUSTNESS.md for contracts and exclusions.
Distance-evaluation counters include the initial solid surface-distance query,
but not topology/winding work.

Forge can check a positive clearance threshold over a rigid sweep. This is
distinct from finding the first surface collision. The query returns:

- `clear`: every interval was separated under the specified numerical-error assumption;
- `violation`: a sampled distance plus the error allowance is below the threshold;
- `unknown`: the available resolution or evaluation budget cannot settle the query.

An unknown result must never be treated as clear. `witness_time` is a normalized
sample location, not the first time the threshold is crossed. Equality to the
threshold falls within the unresolved band. Input units are retained.

## Method and assumptions

For normalized time, the relative distance variation is bounded in exact
arithmetic by L = |vA-vB| + |omegaA| rA + |omegaB| rB, where each radius encloses
all mesh vertices about its local pose origin. Linear translation and
shortest-arc rotation are the motion model. The pointwise speed bound also bounds
variation of the minimum distance between the two surfaces.

At interval midpoint m with distance d, every time in [a,b] has distance at least
d - numerical_margin - L(b-a)/2, conditional on the distance and arithmetic
error being bounded by the supplied margin. If this exceeds the requested
clearance, the whole interval can be discarded. Otherwise the interval is
subdivided, or a measured below-threshold witness / unknown result is returned.

This uses Parry floating-point surface distance; the numerical margin is an
explicit assumption, not a formally established interval-arithmetic bound.
The API consequently reports `numerical_error_bound_proven=false`. It does not
detect complete solid containment with separated surfaces. Physical calibration,
robot tracking error and link-path interpolation error require separate margins
and models; they are not accounted for by numerical_margin.

The Rust loop releases the GIL and reuses retained native geometry. Batches can
run through Rayon and preserve input order. max_evaluations is per query, so total
work also depends on the number of pairs and robot subsegments.

## Retained rigid geometry

```python
import compas_forge as forge

forge.register_mesh_to_cache('moving', mesh_a)
forge.register_mesh_to_cache('fixed', mesh_b)
try:
    reports = forge.verify_clearance_cached_batch_poses(
        [('moving', frame_start, frame_end, 'fixed', obstacle_frame, obstacle_frame)],
        clearance=0.005, numerical_margin=1e-8,
        max_evaluations=4096,
    )
finally:
    forge.unregister_mesh('moving')
    forge.unregister_mesh('fixed')
```

Use unique IDs when different callers share a process.

## Prepared COMPAS FAB cell

```python
with forge.prepare_compas_fab_cell(cell, state) as prepared:
    for trajectory in trajectories:
        collision = prepared.sweep(trajectory)
        clearance = prepared.verify_clearance(trajectory, clearance=0.005)
```

Preparation copies cell/state and retains the geometry once. Changes to the
original inputs do not update that snapshot: close it and prepare a new one
after changing geometry, attachments, touch rules, hidden objects or tool state.
close() is idempotent; closed objects reject queries. Partial robot trajectories
override named joints and preserve other joint positions from the captured state.

The cell adapter applies the same five pair classes and exclusions as its
collision path. It reports the first subsegment that has a violation or unresolved
pair; later subsegments need not be checked after that. Its clear result applies
to piecewise rigid link motion between sampled joint configurations. The sampling
increments do not establish a Cartesian bound on the actual articulated motion.
Tool joint states and unattached body frames remain fixed during the trajectory.

## Verification

tests/test_clearance.py checks 60 seeded analytical near-miss sweeps, rotation-only
interior violation, serial/parallel equality, common translation cancellation,
threshold ambiguity, invalid policies and budget exhaustion. The optional
test_pybullet_differential.py compares 80 seeded unit-box clearance cases with
an independent Bullet distance oracle. It deliberately excludes solid-containment
and signed-penetration comparisons, where the contracts differ.

benchmark_prepared_cell.py retains raw paired timings, versions and source hashes
for both a stationary cell and a moving UR5. It measures setup amortization,
not a language-level or cross-library speed comparison.
