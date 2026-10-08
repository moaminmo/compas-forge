# Robustness work: contracts, derivations and remaining limits

This document separates exact-arithmetic mathematical bounds, floating-point
implementations, regression evidence and physical validation. Passing tests is
not a universal correctness proof. No physical robot-safety certification is made.

## 1. Tighter rotating swept bounds (implemented)

For a material point p of radius at most r from the pose origin, rigid motion is
x(t) = c0 + t(c1-c0) + R(t)p for normalized t in [0,1]. Shortest-arc rotation has
constant angular speed theta, so ||x''(t)|| <= r theta². The linear interpolation
remainder, equivalently its Green-function integral, gives

    ||x(t) - ((1-t)x(0)+t x(1))|| <= r theta² t(1-t)/2 <= r theta²/8.

Therefore the union of endpoint AABBs, expanded in every coordinate by r theta²/8,
encloses the complete rigid sweep. Transforming all eight local AABB corners
bounds the endpoint vertices; convex interpolation covers every triangle.
The old origin-sphere bound remains a cheap first stage. The tighter second stage
is useful for long thin links and small angular intervals. It is not guaranteed
to beat the sphere bound in every configuration.

Code: src/bounds.rs. Defensive floating-point padding is added; nonfinite
intermediates disable rejection. This padding is NOT directed-rounding interval
arithmetic. The geometric proof is exact-arithmetic, not a formal proof of all
floating-point executions. Tests include generated translated/rotated material
points, rotation-only interior collision, and comparison to unfiltered narrowphase.
Native batch keyword `tight_bounds=False` permits controlled ablation.

Clearance queries also reject a pair when the Euclidean gap between its swept
endpoint AABBs, minus numerical_margin, exceeds its effective clearance including
the pair's articulated allowance. `broadphase=False` disables this clearance
filter for ablation. Unbounded boxes do not reject a pair. Solid-input validation
still precedes this filter. Neither filter is a floating-point certificate.

## 2. Restricted solid clearance (implemented, opt-in)

Use `verify_clearance_cached_batch_poses(..., solid=True)` or
`prepared.verify_clearance(..., solid=True)`. Surface-only remains the default
for backward compatibility and to avoid silently interpreting open robot meshes
as solids. Ordinary sweep/assembly APIs retain surface semantics.

Solid mode requires one connected closed consistently oriented embedded manifold
shell, without degenerate triangles or isolated vertices. Topology/degeneracy and
the existing self-intersection diagnostics are checked once per retained mesh.
Invalid or unsupported inputs return unknown. Default `solid_rule='single_shell'`
rejects multiple shells. Optional `solid_rule='even_odd'` defines nested closed
shells as alternating material/void, independent of orientation. Crossing shell
surfaces are invalid, not an implicit Boolean union. Global winding reversal is
accepted. Diagnostics now inspect shared-edge foldovers, duplicate triangles
and intersections beyond shared vertices. They remain floating-point predicates,
not an exact certificate. Embeddedness remains an explicit input contract;
`solid_input_certified=false` exposes this boundary.

Each embedded shell contributes winding 0 or +/-1. Parity of their integer sum
is orientation-independent. Accept only a sum within 1e-8 of an integer whose
magnitude does not exceed the shell count, otherwise return unknown. With
disjoint boundaries, membership is constant over each connected shell: test one
representative PER SHELL in both directions, not just the mesh's first vertex.
Representatives and validity diagnostics are cached per retained mesh. This
reasoning assumes embedded disjoint boundaries and exact arithmetic; its
floating-point implementation is not formally certified.

At the initial poses, surface distance separates the contact band. For separated
surfaces, signed triangle solid angles with normalized directions and compensated
summation determine winding number at a representative vertex of each shell.
Values close to |w|=0 and |w|=1 are accepted; other values remain unknown.
This uses the solid-angle/winding formulation described by Jacobson, Kavan and
Sorkine-Hornung (2013), not their complete segmentation/tetrahedralization method:
https://users.cs.utah.edu/~ladislav/jacobson13robust/jacobson13robust.html

For two connected embedded shells with disjoint boundaries, winding is constant
on each connected boundary. Thus mutual representative tests distinguish nested
solids from disjoint solids. Initial containment is a zero solid-distance violation.
Starting disjoint, containment cannot arise continuously without boundary contact.
Consequently initial disjointness plus positive surface clearance throughout the
motion also excludes later containment, under the stated geometric/numerical
assumptions. The initial solid validation is separate from the adaptive distance
evaluation budget and may be expensive on first use.

This is not a signed penetration-depth solver or an exact-predicate classifier.
Winding tolerance and oracle error are not formally certified floating-point bounds.
The result keeps `numerical_error_bound_proven=false`.

## 3. Articulated interpolation error (implemented with fixed-state attachments)

Use `prepared.verify_clearance(trajectory, clearance=..., articulated_tolerance=...)`.
The tolerance is a requested upper bound on each link's surrogate-path deviation,
in model length units. It is separate from clearance and numerical_margin.

The adapter follows COMPAS Robots' `compute_transformations` contract: a product
of fixed-axis rotations about the initialized joint origins and prismatic
translations, with linearly interpolated joint positions. Fixed, revolute,
continuous and in-limit prismatic joints are supported. Direct affine mimic
joints follow COMPAS precedence: an explicit dependent-joint value overrides
the driver; otherwise use multiplier*q+offset. Missing direct drivers are
rejected, not recursively resolved differently from COMPAS. Planar/floating
joints are rejected. Fixed-state tools,
rigid workpieces attached to a link or robot-attached tool, and stationary world
bodies/tools are covered. Explicit moving-tool paths and attachment phases are
also supported under the additional contracts below.
The robot base frame is fixed. User-overridden kinematic transformations are not
covered by the derivation.

For a material point, track bounds B0 >= ||p||, B1 >= ||p'||, B2 >= ||p''||.
Start at the baked collision radius r with (B0,B1,B2)=(r,0,0). Apply the product
right-to-left (leaf towards root). For rotation about fixed origin o with angular
variation d=|q1-q0|, the product rule gives, using OLD right-hand-side values:

    B2_new = B2 + 2 d B1 + d² (B0 + ||o||)
    B1_new = B1 + d (B0 + ||o||)
    B0_new = B0 + 2 ||o||.

For translation q(t)u, add max(|q0|,|q1|)||u|| to B0 and |q1-q0|||u|| to B1;
B2 is unchanged. In-limit linear motion avoids derivative discontinuities from
joint-limit clamping. Let A be the sum of absolute revolute/continuous variations.
The endpoint rigid surrogate's rotation angle is no greater than A. By applying
the chord-remainder inequality to both curves and the triangle inequality:

    ||actual(t)-surrogate(t)|| <= (B2 + r A²)/8 = E.

For n uniform subdivisions, variations scale by 1/n and the derivative terms
by 1/n², so E/n² is a valid bound using the full-segment position envelopes.
The adapter raises subdivisions to satisfy the requested tolerance, subject to
max_subdivisions (exceeding that limit raises an explicit error).
For two links, set-distance changes by at most the sum of their Hausdorff errors.
The implementation uses E_a/n² + E_b/n² for each pair, not twice the largest
error elsewhere in the cell. The native batch accepts one nonnegative allowance
per query without splitting into many Python/native calls.

A clear inflated-surrogate query implies the requested articulated clearance,
conditional on the stated assumptions. A surrogate violation does not prove an
actual violation: the adapter re-evaluates FK and distance at the proposed witness.
If it cannot confirm a violation, it may bisect just this pair's interval,
dividing its allowance by four on each bisection. Both children must be clear to
certify the parent; violation witnesses are rechecked with actual FK. Unresolved
leaves remain unknown. Endpoint FK is memoized within the refinement.
`max_articulated_refinements` is 0..8, default 2. max_evaluations is per native
query; total work includes refinement-tree nodes and static witness checks.
Numerical coefficient
roundoff is not formally bounded; numerical_margin remains an explicit assumption.
This model does not include controller interpolation, calibration, deformation,
tracking error, acceleration limits, force/torque limits or dynamic feasibility.

### Fixed attachment extension

For an entity attached to link l, its transform is C*T_l(q)*F, where C is the
fixed robot base and F is the fixed entity placement. With local entity radius r,
the material point F*p has radius at most R=||translation(F)||+r in the baked robot
reference coordinates. Start the derivative recurrence at (R,0,0), but use the
entity's local radius r for the surrogate rotation term. Thus E=(B2(R)+r*A²)/8.
The fixed placement translation norm is recovered from the entity origin and
the parent's product-transform origin at the captured configuration; rotation
and the fixed base preserve this norm. Stationary entities have E=0.

Tests cover the official UR5/UR10e cells with and without gripper/workpiece,
transformed bases, direct link attachments and a nonstationary collision-free
gripper/workpiece path. A workpiece attached to a tool that is not robot-attached
is explicitly rejected in bounded mode because that is outside this FAB contract.

## 4. Known limitations and disposition

`tool_trajectories={'gripper': trajectory}` retains separate tool-link meshes.
Paths must contain complete tool configurations and exactly the robot path's
point times; `articulated_tolerance` is mandatory. The tool-chain derivative
envelope is composed with the parent robot-chain envelope, retaining the mixed
term `2 * angular_rate * B1`. A fixed attachment adds its translation norm to
B0. For an independently rotating free base with angular variation A,
`B2 += 2*A*B1 + A*A*B0`. The endpoint-surrogate term remains `r*total_A^2`;
the total coefficient is divided by 8 and by the subdivision count squared.
Tests include the analytic rotating-slider path and actual coupled COMPAS FK.
The fixed ToolModel TCP frame, including workpieces attached to it, follows
COMPAS FAB semantics; it does not automatically track an arbitrary finger link.

`verify_compas_fab_cell_phases(cell, phases, clearance, **options)` accepts
explicit fixed-attachment phases with state/trajectory and optional scene/tool
paths. Before checking any phase it rejects discontinuous joint values, changed
visible geometry sets and discontinuous world poses at every phase boundary.
Position tolerance is in model units; orientation tolerance is quaternion chord
distance (q and -q are equivalent). Phase times are local. Touch exclusions are
explicit user policy and can change across phases. A geometrically continuous
release is supported, but grasp force/stability, controller synchronization and
contact mechanics are not inferred or certified.

`scene_poses={'body:name': [pose0, pose1, ...]}` supplies one world pose per robot
trajectory point (COMPAS Frames or seven-number poses). Visible unattached
bodies and fixed-shape unattached tools may move with linear translation and
shortest-arc rotation, synchronized with the joint segment parameter/time.
Their motion is exactly the rigid-surrogate contract: zero articulated error.
Witness rechecks evaluate scene poses at that same time. Moving world/world
body and tool/tool pairs are added; body touch exclusions remain respected.
Attached entities cannot be overridden. Free tools carrying attached bodies
are rejected until an explicit coupled trajectory contract exists.

| Area | Status / why not declared solved | Next acceptance gate |
| --- | --- | --- |
| Tight rigid broadphase | implemented; floating-point proof incomplete | paired benchmarks and independent regression corpus |
| Single-shell containment | opt-in implemented under embedded-shell contract | independent complex-solid reference corpus |
| Multiple shells/cavities | explicit even-odd fill implemented; not arbitrary Boolean union | wider independent shell corpus |
| Adjacent degeneracy/self-touch | shared-edge/vertex overlap diagnostics added, not exact predicates | exact/filtered predicate certification |
| Articulated bound | implemented under fixed-axis/linear-joint contract; UR5 and UR10e covered | more robot models and independent bound checks |
| Fixed-state tools and attached workpieces | implemented using fixed placement and entity radii | wider attachment/geometry corpus |
| Mimic | direct affine driver/explicit override implemented | more real robot models; recursive missing drivers rejected |
| Moving tool joints / attachment phases | per-link paths, composed bounds and continuous boundary validation implemented | industrial gripper corpus; physical grasp stability outside geometric scope |
| Numerical certification | not implemented; error allowance is assumed | interval/directed-rounding or exact filtered kernels |
| Physical error/dynamics | cannot infer from geometry | measured machine uncertainty and validated controller/material models |
| External moving scene | synchronized rigid body/tool paths implemented | multiple articulated robots not covered |
| External robot axis | supported when represented by in-chain revolute/prismatic joints | industrial model fixtures; separate controllers not certified |
| Rhino 8 / 9 | Windows in-host checks executed; see hardening audit for hashes | repeat on final release artifacts |
| Grasshopper | in-host DataTree conversion tested | full component recompute/lifetime rehearsal remains |
| Cross-platform distribution | workflow defined, not remotely run | approved publication followed by successful CI artifacts |
| Offline HTML | current report uses CDN assets | bundled licensed assets plus offline browser test |
| Performance/general superiority | only fixture-specific evidence | same-input accuracy/performance corpus, memory and tail latency |

These are known limits, not a claim that every possible defect has been enumerated.
No commit, push or upstream inclusion follows from this local work.
