# Time-varying estimator bounds in predictive control

The controller reads the estimator's current error enclosure on every sample.
The adapter no longer substitutes `cfg.publishedErrorBound` constants. Old
configurations containing that field receive an explicit migration error.
Sensor noise and true-motion limits remain premises of the enclosure, not
fixed bounds on the estimated state error.

This implements publication, validation, frame conversion and finite-horizon
target prediction. It does **not** complete robust closed-loop admission:
the existing exact-rest certificate still rejects nonzero ego/model uncertainty.
Changing the source of a bound does not remove that terminal limitation.

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

`targetPredictionErrorEnvelope` encloses future true position about the
controller's fixed nominal trajectory. It starts from the current certificate
and takes the minimum of valid speed, acceleration and jerk envelopes. For
example, the acceleration envelope is

\[
b_p(t+\tau\mid t)=b_p(t)+b_v(t)\tau
 +\tfrac12(\bar a_T+\bar a_{nom}(\tau))\tau^2.
\]

The jerk envelope starts from current acceleration error. Nominal maxima
cover the complete prefix `[0,tau]`; the jerk alternative is disabled across
the nominal stop's acceleration jump. Heading error integrates true and
nominal yaw-rate limits and is capped at pi. All margins are fixed data in
one SOCP. Future observer corrections are not assumed: a better future
estimate does not retroactively correct the trajectory propagated today.

## Remaining certificate obligations

The ego predictor already propagates its initial box through
`r_(j+1) = abs(A_j)*r_j + w_j`. It does not need a historical maximum or a
constant first-sample radius. An admitted uncertain-state design must also
enclose the inertial-to-Frenet transformation of that initial box.
Propagation alone does not establish terminal
closure: with initial velocity error `+/-epsilon`, the same open-loop inputs
can stop the nominal state while actual velocities still differ by
`2*epsilon`. The nominal zero terminal input does not turn that set into a
rest equilibrium. The `unsupportedCertificateUncertainty` guard remains.

A finite current target bound plus global speed/acceleration limits also
does not establish finite support of the complete future position set. For
this domain-only future model, terminal support is conservatively infinite
unless zero target speed is guaranteed, which gives a stationary position
ball. A route or more informative complete-future motion certificate could
admit additional cases; the current estimator does not provide one. The
implementation does not freeze `B(t)` forever or truncate growth to manufacture
a terminal certificate.

Changed target bounds trigger the existing compatibility/readmission logic.
The old input sequence must pass acceptance with the new margins. Full
uncertainty-aware operation still needs an output-feedback tube, a valid
terminal policy and an adequate future target-motion contract.

## Verification

`controllerEstimatorBoundsTest` checks current-bound precedence and updates,
timestamp/availability rejection, heading consistency, prediction growth,
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
