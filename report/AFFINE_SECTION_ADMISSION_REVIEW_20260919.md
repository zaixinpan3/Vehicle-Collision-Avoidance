# Affine-section admission: mathematical and implementation review

September 19, 2026. Reviewed controller revision:
`5f0a8267c7b5d0a09bb8d7e01da2681268ed8369`.

**Verdict:** forbidden-interval admission is mathematically sound under fixed
robust geometry and affine control sections. It is a promising way to reduce
fresh-admission computation, but neither its admission rate nor a 50 ms frame
bound has been demonstrated. This review changes no executable algorithm.

## Verified mathematical claim

For one declared family `U = U0 + alpha*v`, propagate both the origin and its
direction through the same fixed affine stage dynamics. Relative positions
then have the form `r_i(alpha) = r_i0 + alpha*w_i`. Suppose every unit support
direction has a fixed threshold valid over the entire family, including yaw,
uncertainty, pose-chart error and outward numerical allowances. Certificate
`l` is `a_il*alpha >= b_il`.

The set failing every certificate for condition `i` is an intersection of
strict scalar half-lines. It is therefore an open interval, possibly empty
or unbounded. Intersect the remaining convex hard constraints with the line
to obtain an interval `I0`. Subtracting `R` forbidden intervals leaves at most
`R+1` closed components, including possible singleton components. This is
exact for the declared scalar certificate family, not for all safe vehicle
trajectories. It does not enumerate combinations of nodewise directions.

The zero-coefficient cases and open endpoints are essential. For SOC reduction,
retain `c+d*alpha >= 0` when squaring `norm(a+b*alpha) <= c+d*alpha`.
For example, `norm(0) <= -1` is false although its squared inequality is true.
Uncertain floating-point roots require conservative rejection or inward bounds,
followed by independent physical verification. Positive numerical separation
must remain distinguishable from a user-imposed physical clearance buffer.

The illustrative section `r(alpha)=(0.9,alpha)` around the square
`[-1,1]^2`, with a 0.1 illustrative margin and `alpha in [0,2]`, has permitted
interval `[1.1,2]` under the four axis certificates. The proposal's example is
correct; its 0.1 margin is not a recommended project parameter.

## Compatibility with the current implementation

| Proposal claim or prerequisite | Finding at the reviewed revision |
| --- | --- |
| Fixed affine dynamics permit scalar propagation | Supported by `ltvBicycleModel.finitePredict`. Propagate two six-state vectors per stage rather than constructing all input sensitivities for a scalar-only solve. |
| Current collision geometry is immediately affine in the scalar | False without a new enclosure. `avoidanceSafetyGeometry.jointValue` uses `yawOffset+yawRow*state`, so ego support varies with the decision. |
| CLF can remain soft | Supported. `formulateAvoidanceProblem` has one nonnegative first-hold norm slack and a squared penalty. Eliminate that slack analytically only after restricting the actual cone and objective. |
| Inherited geometry is overwritten despite a lost witness | Stale. `localShift` preserves old maps, bounds, charts and terminal data; `jointProgram` inherits occupied-set records, angles and bounds. |
| A failed improvement necessarily discards the incumbent | Stale. `localJointSearch` independently verifies the incumbent and returns it if the improvement is not certified. The final complete-frame deadline still rejects a late command. |
| Admission currently has an unbounded restoration loop | Stale. The current limit is three native calls including any base solve. Interval admission could reduce work within this already bounded search. |

Code anchors: [affine prediction](../controller/ltvBicycleModel.m),
[formulation and shift](../controller/formulateAvoidanceProblem.m),
[joint geometry](../controller/avoidanceSafetyGeometry.m),
[admission and incumbent retention](../controller/solveHardCbfClf.m), and
[complete-frame acceptance](../controller/collisionAvoidanceController.m).

## Conditions that require explicit implementation

1. **Fix geometry over the whole amplitude range.** Bound the affine nominal
   yaw variation as well as existing yaw error. Unrestricted yaw-dependent
   geometry does not have the interval property: a point at `(2,0)` lies in a
   rectangle of half-sizes `(2.2,0.2)` rotating by `alpha` whenever
   `abs(sin(alpha)) <= 0.1`, giving several disjoint intervals on `[0,2*pi]`.
   This analytical example isolates the missing fixed-enclosure assumption.
   Family-wide yaw envelopes restore the interval property but add conservatism.
2. **Make the base family useful.** A line through a cruise rollout need not
   intersect the hard terminal set. A minimum-energy deformation for one
   intermediate displacement need not return to terminal membership or satisfy
   terminal input slew. Include those constraints in `I0`, and report an empty
   base interval separately from collision or exit exclusion. Computing a base
   feasible plan through the existing SOCP would invalidate a claim of admission
   without any SOCP; it must be counted if used.
3. **Retain the certificate when expanding the search.** If a scalar witness
   initializes a full trajectory solve, impose the same pose and yaw domains
   that justified its fixed support thresholds. Copying the thresholds while
   freeing decisions outside those domains is unsound. Rebuilding tighter
   approximations also needs explicit witness verification.
4. **Retain the certificate when shifting time.** Setting the next origin to
   the accepted tail gives a feasible `alpha=0` only when its enclosures,
   terminal/departure deadlines and conditioning assumptions are retained.
   Recomputing a larger yaw envelope for a new correction family can exclude
   that origin. Current inherited continuation should remain the starting point.
5. **Account for all preparation.** The current predictor allocates a
   `6 by 2N by (N+1)` dense sensitivity array, and formulation constructs dense
   objective maps. Replacing only the final conic solve retains this quadratic
   storage/work. A scalar implementation should propagate the origin/direction
   directly, accumulate the scalar objective and reduce hard constraints without
   first building the full program. Bound family, node, target and normal counts,
   scalar iterations, terminal preparation and final verification explicitly.

For the actual nonnegative-radius soft CLF, restricting the quadratic objective
and eliminating its minimum feasible slack produces a convex scalar objective.
An exact base-interval minimizer can be clipped to each safe component. Fixed
bisection gives approximate performance optimization; clipping still preserves
the certified interval membership, subject to numerical verification.

## Design decision and validation gate

The proposal fits the accepted tradeoff: return a certified restricted plan or
report that none was found. However, a collection of predefined `v_j` still
restricts maneuver shapes and enumerates families. It does not satisfy a literal
requirement to avoid every trajectory restriction or family enumeration merely
because the bases are derived from dynamics.

A compatible first study is **one signed scalar family**, with a deterministic
dynamics-derived deformation and no catalog of left/right paths. Both signs of
the amplitude remain available. This avoids explicit route enumeration, but
still fixes a control-sequence shape; coupled timing, braking and steering
freedom is limited. Failure must be reported as failure within that family.
The family-construction rule remains to be designed and measured, not assumed
effective from the interval theorem.

Study fresh admission first, preserve current witness-based continuation, and
compare a scalar-only result against an optional single witness-containing
performance solve. Benchmark the same frozen 50 ms frames, including the
112-hold worst case, before selecting either for production. Report base
emptiness, geometric exclusion, terminal/exit exclusion, numerical rejection,
certificate acceptance, sampled clearance and whole-frame time separately.

The [existing 50 ms experiment](CONTROL_PERIOD_50MS_20260919.md) measured a
157.056 ms maximum frame, including 67.420 ms formulation and 70.854 ms solving.
Both portions therefore matter. It also retained straight stationary and
oncoming inter-node collisions. Scalar admission does not repair that safety
scope: the active certificate covers hold nodes of the declared affine plant,
and a faster node-feasible solution is not a whole-hold noncollision proof.

## Sources and verification scope

- Muehlebach and D'Andrea, *A Method for Reducing the Complexity of Model
  Predictive Control in Robotics Applications* (2019),
  [arXiv:1903.07648](https://arxiv.org/abs/1903.07648). The verified abstract
  supports time-shift-invariant trajectory parameterization and warm-start
  compatibility as complexity-reduction principles.
- Makarow, Roessmann and Bertram, *Suboptimal nonlinear model predictive control
  with input move-blocking*, revised 2022,
  [arXiv:2109.12355](https://arxiv.org/abs/2109.12355). The verified abstract
  supports restricted corrections around a stabilizing warm start. Its separate
  fallback construction is not adopted here.

Neither abstract establishes this project's interval collision theorem or
real-time bound. The interval result and counterexamples above are direct
mathematical checks of the supplied proposal; implementation claims were checked
against the named revision. No new MATLAB simulation, timing measurement or
production implementation was performed for this review.
