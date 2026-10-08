# Model-aided ego estimation

This note describes how the estimator shares the controller's vehicle model,
input and operating domain (October 5, 2026). It complements
[OBSERVER_ISS_THEORY.md](OBSERVER_ISS_THEORY.md), whose continuous-time
cascade and certificates are unchanged. The experiments and audits are in
`report/JOINT_DESIGN_20261005.tex` and
`report/ESTIMATOR_DIRECT_TARGET_20261005.tex`.

## Why the kinematic relation was replaced

The body-velocity observer needs a measurement of the lateral body velocity
`v_y`. It used the kinematic single-track relation `v_y = l_E*r + l_E*d_st`
with `|d_st| <= delta_st` and a sideslip cone `b`. The relation says that the
lateral speed of the body point `l_E` behind the centre of mass is at most
`l_E*delta_st`. With `l_E` equal to the rear-axle distance, that is the rear
tire's lateral slip speed `v_x*tan(alpha_r)`. The avoidance maneuvers exceed it
(0.86-4.25 m/s at 15 m/s). The published bounds then failed to contain the
truth in most frames of the 15-m/s noisy runs
(`report/ESTIMATOR_DOMAIN_20261005.tex`, addendum). Choosing `delta_st` from the
tire saturation slip makes every guaranteed bound five to seven times larger.

## 1. Shared model, input and domain

`config/estimatorConfigurationFromController.m` derives the estimator's
premises from the controller configuration:

| premise | value |
| --- | --- |
| speed interval | controller `speedMinimum` (at least `scheduleSpeedFloor`) to `speedMaximum` |
| yaw-rate maximum | controller `model.yawRateMaximum` |
| acceleration norm | (sum of axle friction forces + road load at maximum speed)/m |
| yaw acceleration | `[lf, lr]*mu*Fz/Iz` |
| sideslip cone `b` | controller `model.sideslipMaximum`, enforced by the controller |
| vehicle model | mass, `lf`, `lr`, gravity, Fiala stiffness and friction, relative parameter uncertainty 5% |
| target domain | scenario contract (`scripts/collisionThreatContract.m`) |

The adapter passes the input that the controller held over each sample
(`frame.heldSteeringAngle`, `frame.heldBrakingRatio`). Sensor bounds are
unchanged.

## 2. Force-balance lateral velocity

`nrmmModelLateralVelocity` uses the lateral force balance at the centre of mass

    m*a_y = Fx_f(b)*sin(d) + Fy_f(alpha_f)*cos(d) + Fy_r(alpha_r),
    alpha_f = atan2(v_y + lf*r, v_x) - d,   alpha_r = atan2(v_y - lr*r, v_x),

with the controller's modified Fiala tires. The accelerometer measures `a_y`.
Each axle force is non-increasing in `v_y`, so the measured `a_y` determines
`v_y`.

**Certified interval.** Assume the sensor bounds (`|a_y - am_y| <= eps_a`,
`|r - u| <= eps_w`, `||v|| in [M - eps_G, M + eps_G]`), the sideslip cone, and
cornering stiffness and friction within +/-5% of their model values. Each
axle force is monotone in `r`, `||v||`, stiffness and friction. Its extremes
over that box are therefore attained at the 16 corners. With `Gmin`, `Gmax` the
per-axle sums (both non-increasing in `v_y`), every admissible `v_y`
satisfies

    Gmin(v_y) <= m*(am_y + eps_a)   and   Gmax(v_y) >= m*(am_y - eps_a).

This is an interval `[lower, upper]`, found by bisection to 1e-5 m/s. An empty
interval means the measurements contradict the model or the cone; the ego bound
is then declared invalid. In 2000 random states the interval contained the
truth in every case, at 0.45 ms per call.

**Point value.** The published centre solves the balance with the nominal
parameters and the measured values. The radius is the larger distance to the
interval ends. Near tire saturation `dF/dalpha -> 0`, so the 5% parameter box
makes the interval wide and asymmetric (up to about 3 m/s). An earlier version
used its midpoint and fed the observer a biased measurement (yaw error p95
0.061 rad, `v_y` error p95 0.92 m/s in a noisy 15-m/s head-on). With the nominal
solution these were 0.0046 rad and 0.031 m/s.

The controller's rear-adhesion cone keeps the rear tire below saturation (see
`controller/PCBF_CLF_ARCHITECTURE.md`, *Stable-handling envelope*), so the
inversion stays informative. Without the shared model or a held input (for
example over the initialization history), the kinematic relation and its
declared mismatch are used unchanged (`nrmmEgoCourseGeometry`).

## 3. Body-velocity observer and its bound

The measurement is `y = [sqrt(max(M^2 - c^2, (cos(b) M)^2)); c]`, with `c` the
force-balance centre. The observer is unchanged:

    vhat' = a_m - u*J*vhat + k*(y - vhat).

**At a sample.** In the cone, `||y - v(t_k)|| <= (eps_G + rho)/cos(b)`, with
`rho` the certified lateral radius. The mean-value bound gives `eps_G/cos(b) +
rho*tan(b)` for the longitudinal component. Combined with `rho` for the
lateral component, this is below `((eps_G + rho)/cos(b))^2`.

**Between samples.** The measurement is held for `tau` after the sample:

    e' = -k e - u J e + (a_m - a) - (u - r) J v + k (y - v(t)),
    ||y - v(t)|| <= (dS + dW)/cos(b),
    dS = min(Vmax + M, eps_G + aEnv*tau),
    dW = rho + (aEnv + (|u| + gyroError)*min(Vmax, M + dS))*tau,

since the speed changes by at most `||a||` per second and
`|v_y'| = |a_y - r v_x| <= ||a|| + |r| ||v||`. The comparison input of
`nrmmPositionErrorBound` is

    accelerationError + gyroError*Vmax + k*(dS + dW)/cos(b) + defect.

## 4. Course correspondence for the yaw observer

The sideslip sine interval is `[lower, upper]` divided by the speed interval and
clipped to the cone. The heading measurement is the GNSS course minus
`asin(c/M)`. Its certified radius is the sideslip interval's distance to that
centre plus the GNSS direction error (`certifiedRotationCorrespondence`). An
uninformative interval contributes the whole circle; the orientation set
then propagates with the gyro.

## 5. Target parameters come from the tracker

The published target acceleration `A` and sideslip `beta` are the tracker's
own reconstruction from its state `[rho; q; s]` (`nrmmTargetTrackerDerivative`).
Commit `06af0a2` had replaced them with a Gauss-Newton window fit of the
constant-parameter trajectory to the radar detections. At the user's
direction (October 5, 2026) the estimator's direct output is published
again, and the window fit is removed. With it, the six-seed noisy campaign
recovered 82 of 84 encounters instead of 84, with no collision
(`report/ESTIMATOR_DIRECT_TARGET_20261005.tex`). Between 0.5 and 3 s, the
per-run RMS error of `A` had a median of 0.27 m/s^2 against 0.019 for the
fit. That of `beta` had a median of 0.0078 rad against 0.0005. Both
failures were forecasts over long horizons (84 and 104 holds). In them, the
error of `A` moved the target by 3.7 and 5.6 m at the end of the horizon.

That error is the radar noise passed through the tracker's gains, and the
certificate fixes their level. Weighting the acceleration error in the shape
program did not lower it, and the best certified gains found lower it by 15%.
A first-order constant-parameter stage behind the tracker would remove most
of it, since the noise is high-frequency. Two prototypes recovered 82 of 84
encounters, with different failures; the stage is not adopted
(`report/TRACKER_NOISE_AND_DESIGN_ABLATIONS_20261005.tex`). The user
accepted 82 of 84 as the current result (October 5, 2026).

## 6. Target acquisition and initialization in the adapter

`localTargetStateFromInertialWindow` fits position, velocity and a constant
acceleration to a window of at least `accelerationFitMinimumDuration` (0.5 s),
in the frame of the GNSS course fitted linearly over the window. The
tangential and normal acceleration components are kept only where they exceed
three standard errors of the fit, estimated from its residuals. A straight
target therefore keeps the constant-velocity zero rather than fitted noise. A
turning one keeps its centripetal acceleration. The result is clipped to the
acceleration domain. Shorter windows keep the zero acceleration.

A target that enters the radar range during a run is acquired after
`minimumRadarSamples` = 20 consecutive detections (0.25 s), whose
least-squares line initializes its velocity. With a single detection and the
oncoming line-of-sight speed prior (15 m/s), a re-acquired 8-m/s target started
7 m/s off. The tracker's transient then moved its published course by 0.3 rad
and its `A` and `beta` across the whole contract within 0.5 s, and the
controller had no solution (noisy 8-m/s curved head-on, second encounter on
the circular road).

## 7. Certified constant-parameter set (optional)

Under the target contract the whole target path is fixed by six constants at
the present sample, `theta = [x; y; chi; V; A; kappa]` in the true ego body
frame (`kappa = sin(beta)/lr`). `nrmmTargetParameterSet` encloses them by set
membership over the history window (2 s): every radar detection is a
constraint `|p(t_j; theta) - z_j| <= r_j`, where `z_j` is the detection carried
into the present body frame and `p` is the controller's `predictTarget`
family. Two certified carriers exist and each detection uses the one with the
smaller radius:

- gyro: relative heading by trapezoidal gyro integration, radius growing with
  the detection's age (`gyroNoise*dt + yawAccelerationMaximum*dt^2/4` per
  step, about 0.024 rad per second of age with the friction bound);
- heading: the detection's own certified ego heading, radius
  `headingRadius*range` (about 0.0075 rad times the range).

The present heading error and the present GNSS error are shared by all past
detections and are nuisance variables of the linear program. The family is
linearized; its remainder is bounded with the Taylor bound of
`targetPositionSupport`. Because the course and curvature nonlinearities
dominate that remainder over the observer's prior (course about +/-0.4 rad,
curvature the full contract), their ranges are split into slices, each
linearized about its own values; slices whose linear program is empty are
discarded and the union of the others is kept. A slice whose solver fails
numerically is kept whole. `A` and `kappa` are carried to the next published
sample and intersected (they are constant), `V` widened by the `A` interval
times the elapsed time, so these intervals are nested from sample to sample.

`nrmmControllerErrorBounds` intersects the result with the published
`predictionErrorSet` (position and course radii, speed, `A` and curvature
intervals). An empty intersection contradicts a premise; the observer set is
then kept and `parameterMembershipSet.reason` records it. With
`projectForecast` the published `A` and `beta` are clipped into the
intersected intervals, with the course unchanged. The options are
`cfg.runtime.targetParameterSet` in `nrmmTrackingConfig`; both switches are
off by default, which leaves the published estimate unchanged.

Measured behaviour (`report/TARGET_PARAMETER_SET_20261008.tex`, 84 noisy
encounters): the published course and speed half-widths halve (median 0.34 to
0.16 rad and 2.75 to 1.05 m/s at 1--2 s after the first published set); `A`
stays at the contract width and curvature until about 2 s. The detection
radius is dominated by the ego's certified motion error carried into the
present frame (0.2--0.4 m at 30--45 m), not by the 0.04-m radar noise. With
the set, 57 of 84 encounters recovered against 55 without it, with 4 new
recoveries and 2 new failures. The linear programs (MATLAB `linprog`) take
0.9 s per published sample (median, up to 8 s), far beyond the 50-ms
controller period; this is a research implementation.

## Tests

`tests/nrmmTargetParameterSetTest.m` checks containment of the truth over a
synthetic encounter with bounded sensor errors, the nesting of `A` and
curvature across samples, the unavailable cases, and that contradictory
detections are reported as inconsistent.

`tests/nrmmModelAidedEstimationTest.m` checks the following:

- the force-balance interval contains the true lateral velocity;
- the nominal point is exact without noise even where the interval is wide;
- contradictory measurements give an empty interval;
- the estimator domain is computed from the controller.

`tests/nrmmTruthEnclosureAuditTest.m` checks that the shared vehicle model
removes the kinematic premise from the truth audit.

## Scope

The ego and target enclosures are conditional on the stated sensor bounds, the
shared model structure with its parameter box, the sideslip cone, and the
target contract. The plant used in the simulations has exactly this structure,
with the nominal parameters. The replay audit (`scripts/replayEstimatorValidity.m`)
covers every frame of the fourteen noisy encounters (seed 20261003, tracker
output). The published ego bounds (5,958 frames) and target bounds (2,291
frames) contained the truth in every frame. The ego bound stayed valid
throughout. The true motion stayed inside the sideslip cone. In three frames
(15-m/s runs) it reached a rear-adhesion ratio of 1.0004-1.0072, since the
controller's adhesion row acts on the estimate. The force-balance certificate
needs only the cone. The target prediction-parameter set is still wide. Its
per-run medians were a course of +/-0.24 to 0.53 rad and a speed of +/-1.9 to
4.0 m/s, with the whole contract range for `A` and curvature. The controller
therefore uses the current enclosure only for each hold (see
`controller/PCBF_CLF_ARCHITECTURE.md`).
