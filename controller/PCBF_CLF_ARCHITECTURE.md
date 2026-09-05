# Recursively executable hard-CBF--soft-CLF controller

**Implemented:** 2026-09-04

This document is the executable contract for the default certified mode in
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
are hard as well. Certified mode has no CBF or safety slack variable:
`layout.slackCount` is zero and `layout.slackIndex` is empty.

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

`formulateTwoStageQp(..., obstacleMode="certified")` builds this problem.
The shifted and cruise linearization starts, when enabled, each produce a
hard candidate. Certified mode does not run the elastic collision-slack
refinement used by the optional legacy path.

## 2. Joint CLF-relaxation and input objective

The exact executed-step CLF condition is

```text
V(e_1) - V(e_0) + W(e_0) <= delta,    delta >= 0.
```

It is represented exactly as a rotated second-order cone. Later tangent CLF
rows are not part of certified mode. `solveHardCbfClf` performs one conic
solve with the joint objective

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
shift within `certification.shiftConsistencyTolerance`. Target identity,
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
