# Review of the proposed circular-reference redesign

Date: September 17, 2026. Reviewed implementation: `fd46410282d7da45bd696aa5ecee306f595369ef`.
Status: mathematical/code review and proposed implementation scope; no controller
or estimator behavior was changed and no new closed-loop experiment was run.
AI-assisted source inspection, primary-source verification and arithmetic checks
were used. The supplied external analysis is a proposal, not an executed result.

## Applicability to the recorded failure

The [recorded experiment](CIRCULAR_ARC_CONTROLLER_VALIDATION_20260917.md) uses
fixed analytic circles, no road boundaries, and zero ego/target measurement and
process uncertainty. Constant-curvature cruise already completes. The directly
measured problem is the broad actuator-box pose remainder: at radius 100 m and
48 holds, 271.083 m per coordinate, 1280 individually impossible collision rows,
and -136.203 m best selected exit-row margin over the actuator outer box.

| Supplied diagnosis | Verification and relevance |
| --- | --- |
| Curved polylines are rejected by the terminal constructor | Confirmed in `hardEncounterBarrier.localTerminalSet`; the test supplies `referenceCurve`, so this is not its trigger. |
| An analytic curve without a centerline gets an ego-dependent auxiliary line | Confirmed in `readPlanningInputs.localReadLane/localReadCenterline`; fixed centerline plus curve was supplied in the test. Canonical parsing is a useful interface fix, not the measured failure. |
| Full actuator boxes inflate circular chart bounds | Confirmed in `laneGeometry.sweptCellFrames/referenceFrame` and `hardEncounterBarrier.completionRows`; directly matches the measured failure. |
| Support proposals use approximate curved poses | Confirmed in `avoidanceSafetyGeometry.supportNormals`; exact coordinate evaluation would improve proposals but cannot repair impossible hard rows on its own. |
| Repeated boxing can amplify uncertainty | Confirmed in `ltvBicycleModel.finitePredict`; relevant to uncertain experiments. With zero initial measurement radius and zero process residual, this component remains zero and cannot explain the present 271 m charge. Polynomial/roundoff enclosures are separate. |
| A changing reference needs different tracking ingredients | Relevant to a later variable-curvature extension; changing world heading on a circle does not require a changing Frenet equilibrium. |
| Forced range exit can be impossible for a persistent encounter | A valid conditional counterexample, not established for this test: radii are 50/100 m, range is 16 m, and no physical constraint fixes the ego exactly to the circle. |

## Verified local pose construction

For an arc-length parameterization, t'=kappa n and n'=-kappa t. Differentiating
p(s,d)=c(s)+d n(s) gives p_s=(1-kappa d)t and p_d=n. The proposed first-order map
at (s0,d0) is therefore correct:

\[
p_{aff}=c(s_0)+d_0n(s_0)+(1-\kappa_0d_0)t(s_0)(s-s_0)+n(s_0)(d-d_0).
\]

For a circle, psi=theta0+kappa(s-s0)+e_psi is exact with an unwrapped reference
heading. For a smooth variable-curvature path, the supplied derivatives
p_ss=-kappa' d t+kappa(1-kappa d)n, p_sd=-kappa t and p_dd=0 yield the stated bound
by Taylor's integral remainder and the triangle inequality:

\[
E_p=\tfrac12[K_1D+K_0(1+K_0D)]\Delta s^2+K_0\Delta s\Delta d,
\qquad E_\psi=\tfrac12K_1\Delta s^2.
\]

The entire straight segment from anchor to evaluated (s,d) must lie in the
domain used for the derivative bounds. A rectangular local domain supplies
this property. The Euclidean position bound contributes at most E_p along a
unit separation normal; converting it to two component bounds and charging
|n|_1 E_p is also safe but unnecessarily looser.

Exact affine yaw does not make the rectangular footprint support linear. Its
absolute sine/cosine support still needs a valid convex majorant over the
complete admitted yaw range. If that majorant is alpha+beta psi and
p_aff=p0+Jx, psi_aff=psi0+w'x, its state row must contain

\[
a=-J^Tn+\beta w.
\]

For a circle w contains both the station coefficient kappa and the heading-error
coefficient 1. Putting beta only in the e_psi column would lose the station-yaw
coupling. Box uncertainty can be charged as |a|'rho, with geometric remainder,
target support/uncertainty and numerical reserves added separately. Keep unit
tangent/normal fields separate from J; native and interpreted row kernels must
implement the same map and orientation majorant.

## Required certificate conditions

1. Enforce each local domain on every Bernstein coefficient, including its
   uncertainty/remainder radius, for the entire held interval. Enforce the
   required yaw-majorant domain as well. Domains stay hard during restoration.
2. Include a certified local domain at the terminal endpoint used by the finite
   encounter-exit row. Fixing collision cells alone leaves the old large exit
   chart as an independent infeasibility source.
3. Domains restrict each searched convex family; they are not new physical road
   boundaries. Permit domain enlargement, movement or alternative families when
   a local search stalls. One failed local family does not prove global physical
   infeasibility. Choosing domains must not prescribe an avoidance trajectory.
4. Store accepted domains, pose maps, bounds, normals and absolute exit deadline
   with the witness. Conditioning and shifting retain the old covered family;
   accept a new chart family only after verifying a complete feasible witness
   against all of its hard constraints. Do not discard the old domain proof
   simply because a new linearization looks tighter.
5. Keep proposal geometry separate from proof geometry: use the exact Frenet
   coordinate evaluation for nominal direction queries, and certified affine
   maps plus remainder charges for constraints and independent acceptance.

These conditions explain how to preserve the existing continuation argument:
the old suffix remains inside its old hard domains under set inclusion. A
replacement family is an additional verified candidate, not a reinterpretation
of the old one. This is a proof obligation for the proposed implementation,
not a claim that the unimplemented algorithm is already recursively feasible.

## Recommended next scope and validation

First implement the local position/yaw maps, whole-hold domain rows, matching
terminal exit map, exact nominal pose queries and witness transfer. Preserve the
constant-curvature affine generator, curved trim, soft squared CLF, physical
actuation constraints and target release semantics for this controlled test.
Canonical reference parsing can be corrected independently. Road containment,
correlated uncertainty/conditioning, persistent-target continuation, changing
curvature and nonlinear plant inclusion require separate extensions; none is
needed to explain the recorded deterministic-circle failure.

Validate finite-difference Jacobians, domain-wide remainder inclusion, yaw
support majorants, rejection of out-of-domain restoration candidates, terminal
exit domain membership, native/interpreted row parity and witness-preserving
shifts. Then repeat the same four curves and twelve obstacle trials, recording
all search time and failed attempts against the 100 ms frame limit. There is no
proof yet that the proposed local search always succeeds or meets that limit.

## Checks and source limits

A seed-20260917 arithmetic check evaluated 135 circular domains and 5940 points
(corners plus random interior samples). The largest error/bound ratio was
0.980554; the largest bound excess, 7.95e-15 m, occurred at floating-point scale.
The affine yaw residual was at most 4.45e-16 rad. This checks implementation
arithmetic, not the continuum proof or closed-loop performance. Script and
results: `/home/zai/.cache/collisionAvoidance/arc-review-20260917/`.
The supplied repeated-box illustration was reproduced as 18.3889 versus 1.13111;
it remains a simplified rotation example, not a result for the full vehicle.

[Reiter et al., Frenet-Cartesian Model Representations](https://arxiv.org/html/2212.13115v1)
explicitly combines Frenet performance descriptions with Cartesian obstacle
geometry. It studies NMPC/NLP formulations; it does not certify this repository's
SOCP inner approximation or its carried witness. Sections II and III support the
coordinate distinction.

[Koehler, Mueller and Allgoewer, reference generic terminal ingredients](https://arxiv.org/html/1909.12765v2)
provides reference-dependent tracking terminal ingredients. Section II's
recursive-feasibility theorem requires feasible initialization and its stated
reference/terminal assumptions. It is relevant to a later changing-reference
extension, not a substitute for the missing local-domain proof here.
