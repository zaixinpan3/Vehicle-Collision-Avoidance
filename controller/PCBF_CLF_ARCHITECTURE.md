# Recursively executable hard-CBF--soft-CLF controller

**Implemented:** 2026-09-04

This document is the executable contract for the single algorithm in
`collisionAvoidanceController`. The controller does not optimize a safety
violation value. Collision and road control-barrier constraints are hard;
only the control-Lyapunov condition may be relaxed.

**Collision convention (2026-09-04):** collision freedom is defined at the
discrete prediction steps. A plan whose rectangle-separation constraints hold
at all head and braking-tail nodes is collision-free under this convention.
Both newly optimized plans and stored fallbacks use this same node criterion.

## 1. Hard predictive safety problem

For the augmented ego--target state, let the free head contain `N` stages
and the braking backup contain `N_b` stages. At every covered head and tail
node, the sufficient oriented-rectangle separation margins and road margins
must satisfy

```text
g_collision,i(u) >= 0,
g_road,i(u)      >= 0.
```

Input-magnitude, tire, model-domain, braking-tail, and terminal constraints
are hard as well. The decision is `z = [plan; delta]`, with exactly one
scalar CLF relaxation. Collision and road constraints have no slack.

Here “strict safety” means that the non-relaxed inequalities include the
configured positive clearance. A numerical program cannot impose the open
condition `g > 0`; it imposes the closed condition `g >= 0` after clearance
tightening.

The predictive hard-safe set is

```text
C_H = { z : a hard-feasible head, backup tail, and terminal witness exist }.
```

If no hard-feasible plan exists on the first call, the controller raises
`collisionAvoidanceController:noSolution` and issues no command. No
least-violation recovery plan is returned or stored.

`formulateAvoidanceProblem` builds this problem. Every sample uses the
shifted-plan start (the schedule reference on the first call). Whenever the
target imposes constraints, the obstacle-free cruise reference supplies a
second start. Both solve the same hard-CBF/soft-CLF problem, and the candidate
with the lowest joint objective is selected. Each start gets one solver call;
a numerical failure proceeds to the stored-plan protocol below. There is no
configuration switch that selects a different control algorithm.

## 2. Joint CLF-relaxation and input objective

The exact executed-step CLF condition is

```text
V(e_1) - V(e_0) + W(e_0) <= delta,    delta >= 0.
```

It is represented exactly as a rotated second-order cone. `solveHardCbfClf`
performs one conic solve with the joint objective

```text
min  J_input(plan)
     + relaxationWeight * delta.
```

Thus CLF relaxation and input intervention trade according to their declared
weights, but neither can trade against collision or road safety because the
CBF rows have no slack. The quadratic input cost is represented by an exact
squared-norm epigraph; the CLF relaxation has no quadratic penalty.

## 3. Terminal closure

The head ends in the analytically certified lateral handoff set. The
longitudinal backup tail must reach zero speed and zero final acceleration,
and collision and road rows cover every tail node.

At the resting node, the target predictor supplies the support of its
complete future centre trajectory,

```text
sigma_H(n) = sup_{t >= H Ts} n' p_T(t|k).
```

For each direction on a fixed inertial grid, the terminal row separates the
resting ego centre from `sigma_H(n)` by the two footprint circumradii,
clearance, prediction budget, and lateral-tail envelope. The direction with
the largest incumbent margin is imposed. The grid affects the size of the
certified terminal set, not which target motions are permitted.

`targetPredictionFutureSupport` evaluates this interface exactly for the
current predictor. Stopped, finite-turning, unbounded-straight, and
indefinitely turning continuations are analytic evaluation cases rather than
target-motion assumptions. A replacement predictor must supply the same
exact support operation for its own complete continuation.

Two hard terminal rows keep the resting station on the lane-polyline segment
whose affine Frenet-to-Cartesian map defines the inertial support row.

The only target-motion premise is that the target follows the controller's
complete, exact, shift-consistent prediction. A prediction for which the
implemented resting policy has no separating witness makes the hard problem
infeasible; that is a feasibility result, not a target-motion restriction.

## 4. Shift and incumbent protocol

A stored certificate contains the complete plan, predicted state sequence,
target continuation, shifted LTV schedule, applied first input, and terminal
lateral reference. Before reuse, the next measured ego state, held actuator
when published, and fresh target prediction must match the stored one-step
shift within `controller.shiftConsistencyTolerance`. Target identity,
geometry, route branch, lane, and configuration must also remain unchanged.

A newly solved hard-feasible plan is accepted directly from the certified
optimization. It is not passed through a second gate for exact-prediction
flags, route coordinates, hard CBF rows, terminal invariance, or exact rectangle
distance at the nodes.

If a later improvement solve fails numerically, the shifted incumbent is not
accepted on faith. The controller rebuilds its hard CBF rows at that
trajectory, supplies the exact CLF relaxation required by the fixed plan,
and verifies every bound and hard row. It then repeats exact rectangle checks
at the prediction nodes. A shift satisfying these node checks and the existing
prediction, route, and terminal conditions may provide the fallback action.

## 5. Geometry and discrete collision criterion

Each affine collision row is the sufficient support-function inner
approximation generated at its linearization trajectory and is exact there.
Consequently, a separate exact rectangle-distance calculation is not used to
accept a newly optimized solution.

For prediction node indices `k = 0, ..., N + N_b`, the collision criterion is

```text
dist(ego_rectangle_k, target_rectangle_k) >= required_clearance_k.
```

The required clearance includes the existing geometric and prediction
tightenings and, on tail nodes, the terminal lateral envelope. The controller
does not interpolate between nodes or evaluate midpoint or swept-volume
collisions. A between-node crossing does not reject a plan whose node
constraints hold. This applies to both ordinary optimization and the stored
fallback recheck. The terminal continuation support remains the conservative
admissibility condition described in Section 3.

## 6. Guarantee and boundary

Suppose the hard problem is feasible on the first call and the committed plan
has `planCertified=true`. Assume the declared ego model and actuation are
exact, target prediction is exact and shift-consistent, constraints shift
with the horizon, and the terminal continuation support is exact. Then every
subsequent call has the shifted certified plan as a feasible candidate. The
controller either commits a new certified hard-feasible plan or executes the
independently re-certified shift. Therefore the terminally closed feasible
set is control invariant and collision separation is preserved at the
enforced prediction nodes. No separate between-node claim is made.

The CLF is a performance condition inside this hard-safe set. When zero
relaxation is jointly optimal, the exact first transition achieves the
declared Lyapunov decrease. During avoidance, positive CLF relaxation is
permitted and jointly balanced against input intervention.

Principal diagnostics are `cbfConstraintsHard`, `cbfMinimumMargin`,
`hardCbfSatisfied`, `clfRelaxation`, `clfRelaxationCost`,
`inputDeviationCost`, `jointObjectiveValue`, `clfExactResidual`,
`terminalInvariantCertified`, `terminalContinuationAxis`,
`terminalFutureSupport`, `terminalSupportDirection`,
`terminalSegmentIndex`, `certificateSource`,
`postSolveCertificationPerformed`, `scheduleShifted`,
`targetContinuationShifted`, `fallbackUsed`, and `planCertified`.
`collisionDiscretization` is `"predictionNodesOnly"`. The exact-node
`nodeClearanceMargin` diagnostic is present on the exceptional stored fallback
recheck. There is no swept-clearance diagnostic or subdivision configuration.

## 7. Rectangle support and affine safety rows

For unit direction `n = [cos(alpha); sin(alpha)]`, a rectangle with
half-length `l`, half-width `w` and heading `psi` has support

```text
h(n, psi) = l |cos(alpha - psi)| + w |sin(alpha - psi)|.
```

The configuration obstacle of the ego centre is the Minkowski sum of the
target rectangle and the centrally symmetric ego rectangle. Its support is
the sum of the two footprint supports plus the projected target centre.
`rectangleConfigurationDistance` forms the exact polygon, with at most eight
vertices, and returns its signed distance and a supporting unit normal.
For an outside point the normal faces the closest boundary point; for an
inside point it is the outward normal of the least-penetrated edge.

At each head node, maximize the footprint supports over their declared yaw
uncertainty intervals. Let `b_k` bound the absolute derivative of ego support
on the entire admitted heading interval. Then

```text
h_E(n_k, psi_k) <= h_E(n_k, nominalPsi_k)
                  + b_k |psi_k - nominalPsi_k|.
```

The maximum support and derivative bounds are evaluated analytically at
interval endpoints, trigonometric extrema and kinks. For both signs
`sigma = +/-1`, impose

```text
g_k^sigma(plan) = n_k' p_k(plan)
                 - sigma b_k (psi_k(plan) - nominalPsi_k)
                 - n_k' p_T,k - h_T,k - h_E,k - tightening_k >= 0.
```

These rows lower-bound the true rectangle separation in the chosen
coordinates over the declared heading domain. The uncertainty and geometric
budgets enter `tightening_k`. At the nominal heading the extra Lipschitz term
vanishes. Different starts supply different supporting directions; selecting
between their feasible solutions preserves every hard constraint. This
finite set of starts does not establish a global optimum of the original
nonconvex collision-avoidance problem.

Road boundaries use the corresponding inward normal and a bound on the
boundary's lateral variation over reachable stations. Every covered road
node remains hard. An initially violated constant row makes the candidate
infeasible even when the current input cannot change that row.

## 8. Braking witness and lateral handoff

The free head contains `N` steering/acceleration pairs. Its tail has `N_b`
independent accelerations and the affine longitudinal maps

```text
v_(k+1) = v_k + Ts a_k,
s_(k+1) = s_k + Ts v_k + 0.5 Ts^2 a_k + Ts kappa_k vHat_k d_N.
```

Tail acceleration lies in `[-backupDeceleration, 0]`, speed stays nonnegative,
and the last node has zero speed and zero final acceleration. `N_b` is derived
from the maximum admitted speed, braking bound and sample time. A small tail
quadratic regularization selects a witness while keeping the plan Hessian
positive definite. The terminal steering law is used when a shifted plan
extends its head into the backup; the dynamic-head/kinematic-tail handoff
remains a declared modeling premise.

In traveled arclength `s`, the lateral backup law has the linear dynamics

```text
DeltaD' = ePsi,
ePsi'   = -omega^2 DeltaD - 2 omega ePsi.
```

From `[DeltaD(0); ePsi(0)] = [0; e0]`, its response is
`DeltaD(s) = e0 s exp(-omega s)` and
`ePsi(s) = e0 (1 - omega s) exp(-omega s)`. The shift-consistent lateral
radius is `|e0|/(2 omega)`: the envelope about a later point of the response
is contained in the original envelope. `terminalLateralCertificate` combines
this bound with steering and lateral-acceleration limits. Handoff rows also
bound lateral velocity and yaw-rate error; their course contributions and
state uncertainty reduce the admitted heading error.

Tail footprint support is maximized over the certified heading band, and
the band's lateral drift is included in the separation budget. The resting
node additionally carries the complete-future target support condition from
Section 3. `terminal.supportDirectionCount` controls its fixed direction grid.
These terminal rows are part of every candidate and every stored fallback.

## 9. Module and configuration ownership

`collisionAvoidanceController` owns the stateful shift, candidate comparison,
commands and diagnostics. `formulateAvoidanceProblem` owns the hard affine
rows, cruise objective and exact CLF data. `solveHardCbfClf` translates the
quadratics into cones and invokes `coneprog`. Geometry, prediction and
terminal-support calculations remain shared single-purpose modules.

`collisionAvoidanceControllerConfig` defines and validates the numerical
parameters. Its recursive merge rejects unknown fields. The solver hook
`solver.jointFunction` permits controlled numerical-failure tests of the same
conic problem; it does not select a different optimization formulation.
