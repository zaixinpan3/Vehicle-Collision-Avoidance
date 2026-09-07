# Time-varying estimator bounds in predictive control

Version-9 implementation update (September 7, 2026): current estimator
certificates are the initial enclosures of the
[encounter-scoped controller](ENCOUNTER_SCOPED_CBF_CLF.md). Active targets persist
independently of publication. Valid observations condition the carried future
family; new targets need explicit finite motion and nonreturn route contracts
as specified in [TARGET_PREDICTION_CONTRACT.md](TARGET_PREDICTION_CONTRACT.md).

The adapter's current bounds remain current-state claims, with
`futurePredictionIncluded=false`. They do not establish future jerk limits,
physical plant residuals, detection completeness, or discharge. Known targets
can publish subsequent measurements without repeating their retained contract.
Replacing that contract requires independent admission and is not automatic.

Bounded nonzero ego residuals and velocity radii are propagated over the finite
certificate. They no longer require a stationary or dissipative terminal set.
The optional rest construction described later in this research note is not
the version-9 admission rule. Unsupported projection/reference jumps and
uncovered road segments are rejected. The existing nonlinear scenarios still
need validated physical/route contracts for successful robust admission.

## Current-state contract

`onlineNrmmTrackingRuntime` calls `nrmmControllerErrorBounds` to publish
`controllerErrorBound` on ego and target records. The adapter queries the
read-only `output` action with the current sensor frame before publication.
This tightens the enclosure without advancing the observer or using future data.

| Field | Meaning |
| --- | --- |
| `kind` | `ego-state-v1` or `target-state-v1` |
| `time` | The estimate's `stateTime`, in seconds |
| `bounds` | Nonnegative componentwise absolute-error bounds |
| `available` | Whether the conditional enclosure is available |
| `source` | `nrmm-state-time-enclosure` for the runtime |
| `futurePredictionIncluded` | `false`: a current-estimation certificate |
| `scope` | Conditional sampled containment assumptions |
| `floatingPointVerified` | `false`: no directed interval-arithmetic proof |

Ego order: `[x_I; y_I; psi; vx_body; vy_body; yawRate]`, with units
`[m; m; rad; m/s; m/s; rad/s]`. Target order:
`[x_I; y_I; vx_I; vy_I; ax_I; ay_I; psi; yawRate]`, with units
`[m; m; m/s; m/s; m/s^2; m/s^2; rad; rad/s]`.

Numeric compatibility aliases are derived from this same certificate. The
reader gives the certificate precedence over stale numeric aliases. It rejects
unavailable, negative or nonfinite bounds, missing estimator metadata, and
mismatched bound/state timestamps. Supplied ego and target state timestamps
must agree. Raw model inputs without estimator metadata retain the explicit
numeric-bound contract. A source label records provenance; it does not prove
the caller's declared assumptions.

The existing `relativePositionErrorBound = B_rho(t)` remains a Euclidean
radius in the ego body frame. For comparison bounds `b_psi`, `b_v` and
`[B_rho; B_q; B_s]`, absolute position publication uses

\[
b_{p_E}(t)=\|\hat p_E(t)-p_{GPS}(t_m)\|+b_{GPS}
              +\bar v_E(t-t_m),
\]

\[
b_{p_T}(t)=b_{p_E}(t)+B_\rho(t)
 +2\min(\bar R(t),\|\hat\rho(t)\|)\sin(b_\psi(t)/2).
\]

Yaw error is capped at pi. The minimum follows from the two exact
decompositions of `R rho - Rhat rhohat`, using either true or estimated vector
length. Velocity and acceleration use analogous rotation bounds. No statistical
independence is assumed. Each Euclidean radius also bounds each Cartesian
component; this conservative box conversion does not exploit correlations.

At the gyro sample time its noise bound applies. At a later state time, the
declared yaw-acceleration limit times sample age is added. Without a finite
rate premise, the true yaw-rate domain supplies the enclosure.

The synchronized read-only runtime output uses the accepted sensor-frame
timestamp as its canonical state timestamp. The input has already passed the
existing sample-grid tolerance check. This prevents roundoff between two
representations of the same grid instant from falsely making a fresh gyro
old. The `step` output remains one actual sample period ahead and retains
the corresponding hold/domain uncertainty; off-grid frames are still rejected.

Target orientation uses body heading. The velocity-direction radius is
`asin(B_q/|qhat|)` when the velocity ball excludes zero, and pi otherwise.
For the saturated yaw-rate inverse, valid global Lipschitz coefficients are

\[
L_q=\bar a_T/v_{min}^2+2\bar v_T\bar a_T/v_{min}^3,
\qquad L_s=\bar v_T/v_{min}^2.
\]

The implementation combines these error-dependent bounds with independent
yaw-rate and sideslip caps. The reader now consumes `targetHeadingInertial`;
previously it could ignore that published heading and use velocity course,
which has a different center when sideslip is nonzero.

## Future prediction

The target model assumes constant curvature and constant tangential
acceleration during the finite encounter prediction. It does not assert that
the target retains those parameters forever. Current estimation error remains
part of the prediction; no future observer contraction is presumed. See
[TARGET_PREDICTION_CONTRACT.md](TARGET_PREDICTION_CONTRACT.md) for the enclosure.

The estimator no longer publishes `targetPredictionMotionBounds`. Its
operating-domain bounds remain premises of the current estimation enclosure;
they are not independent adversarial future maneuvers. Legacy records carrying
that extra field do not override the controller's finite prediction law.

Every currently published target receives hard geometry rows at all head/tail
nodes, including the last node. There is no infinite-future target support row.
A currently visible target is not removed early merely because its forecast
leaves the sensor range. The adapter stops publication on actual current radar
exit; the controller then removes that target's rows and re-admits the plan.

## Remaining certificate obligations

The ego predictor propagates its initial box through
`r_(j+1) = abs(A_j)*r_j + w_j`, with transition-weighted integration of
continuous forcing. Cartesian-to-Frenet bounds include possible tangent
changes; uncertain admission requires an invertible interior initial chart.
The compatible continuation intersects the new current estimator box with
the previous reachable box at the retained nominal center.
The dissipative terminal set handles nonzero initial velocity radii under
the declared affine ego dynamics. Persistent forcing still requires a different
certificate and remains rejected. This ego/road invariant construction does
not establish permanent separation from a target beyond the finite forecast.

Changed target bounds trigger readmission. On a consistent shift, overlapping
rows retain their validity, but the appended target node gets a fresh row and
a new acceptance check. The old plan is used as fallback only if it passes
that check. `terminalPredictionCertified` reports finite terminal admission;
`terminalInvariantCertified` is false when a target is present. Solver failure
and a rejected appended node can leave no admitted command: indefinite
recursive feasibility against future targets is not claimed.

## Verification

`controllerEstimatorBoundsTest` checks current-bound precedence and updates,
timestamp/availability rejection, heading consistency, finite prediction propagation,
nominal stopping, SOCP tightening and continuation readmission. Its accepted
SOCP examples use an explicitly declared stationary target enclosure and exact
ego dynamics; they are not NRMM closed-loop robustness experiments.

`nrmmPositionErrorBoundTest` checks current-sample tightening and all published
ego/target components against a noisy 20-sample adapter trajectory.
`onlineNrmmTrackingRuntimeTest` parses a physically synchronized turning ego
and in-domain target, and separately rejects an unavailable out-of-domain
target certificate. No uncertainty is zeroed to admit those tests.
`nrmmControllerErrorBoundsTest` checks reconstructed world-state containment
under rotation, angle wrapping, high-gain peaking and near-zero estimated
velocity, conditional on the declared NRMM inverse-model domain.

On September 5, 2026, validation on the concurrent observer-update base
`758a984bbb1805411891e5388d3e212c7666bf2b` passed 159 of 165 tests across ten
affected classes. The six failing estimated-state scenarios still encounter
the exact-ego terminal-certificate guard; full robust integration is incomplete.
Code Analyzer reported no findings in the 13 changed MATLAB files. In the
20-sample noisy adapter trace (seed 83, 0.1 s controller samples), all 14
published components contained their actual errors. Ego and target position
component bounds varied over `[0.040, 0.08889]` m and `[0.32369, 0.54778]` m,
respectively. This is an interface-containment check, not a completed
uncertain-state collision-avoidance experiment.
