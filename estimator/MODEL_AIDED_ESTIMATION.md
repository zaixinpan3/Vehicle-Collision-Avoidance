# Model-aided ego estimation and contract-consistent target parameters

This note describes how the estimator shares the controller's vehicle model,
input and operating domain (October 5, 2026). It complements
[OBSERVER_ISS_THEORY.md](OBSERVER_ISS_THEORY.md), whose continuous-time
cascade and certificates are unchanged. The experiments and audits are in
`report/JOINT_DESIGN_20261005.tex`.

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

## 5. Contract-consistent target parameters

The target contract (NRMM) keeps the tangential acceleration `A` and the
sideslip `beta` constant, so the path has constant curvature
`kappa = sin(beta)/lr`. The high-gain tracker does not use this constancy. It
reconstructs `A` and `beta` from its acceleration state, which differentiates
radar noise twice and has a transient after initialization. On the noisy
campaign, its `A` error had RMS 0.2-0.4 m/s^2 throughout. Its `beta` error had
RMS 0.03-0.04 rad in the first 0.5 s, the size of the whole contract range
(+/-0.055 rad), and reached that bound within 0.05 s of initialization. The
controller plans with these values. The forecast then moved by about 0.6 m at
1 s ahead from one sample to the next, and the encounter plans became
infeasible.

`nrmmTargetParameterFit` keeps the radar detections of the last
`targetHistoryDuration` (2 s), each with the ego position estimated at its
time and the gyro rate. At a solve the ego heading at each detection is the
current yaw estimate minus the integrated gyro back to that detection, and
the detections are converted to the inertial frame with these headings.
Within the window the relative headings are accurate to the gyro noise. A
constant yaw error `e` rotates the target path and adds `e` times the ego
displacement, so `A` and `kappa` move only by `e` times the ego's
acceleration. The first version converted each detection with the yaw
estimated at its own time. The yaw observer's corrections, times a range of
45 m during an avoidance, then bent a re-acquired straight target's track
into `beta = -0.055` (the contract bound; truth -0.008).
Gauss-Newton fits `theta = [X; Y; course; V; A; kappa]` at the solve time
to the closed-form trajectory `predictiveSafetyGeometry.predictTarget`, the
one the controller predicts with. It starts from a quadratic polynomial fit
of the same window, and clips `V`, `A` and `beta` to the target domain.

**Model selection inside the contract.** `A` and `kappa` stay free only where
they exceed three standard errors of the fit, computed from its residual
variance. Otherwise the contract member with that parameter at zero
(constant speed, straight path) is refitted. With the 0.5-s initialization
window the curvature's standard error is about 0.006 1/m at 8 m/s. The
unrestricted fit once gave a straight head-on target `beta = -0.016`. Its
forecast then drifted 0.6 m into the ego's escape side within 1.1 s, and the
startup plan overlapped that forecast after the slack prefix, where the
fixed-multiplier collision rows are hard and have zero gradient at overlap,
so the first frame had no solution (2 of 12 noisy 15-m/s head-on runs). A
turning target's curvature (0.03 1/m) is significant from the first frame.
The test passes any parameter once the window has made it significant.

Once the window spans `parameterFitMinimumDuration` (0.5 s), the runtime
publishes the fitted `A` and `beta`. Before that, after a mid-run
acquisition, it publishes the contract's simplest member `A = 0`, `beta = 0`,
not the tracker's initial transient. It keeps the tracker's position,
velocity (speed and course) and certified sets. The heading is the tracker
course minus the published `beta`, so the published course is unchanged.

Bookkeeping keeps the certified sets valid about the published values. The
acceleration interval is already recentred at the published `A`. The
published yaw-rate and sideslip radii add their distance to the tracker's
values. The controller computes curvature and sideslip radii as distances from
its published centre to the interval ends.

The fit is an estimate, not an enclosure. An offline prototype without model
selection used synthetic radar noise and the recorded ego estimation errors,
over five encounters and their first 2 s. The fit's `beta` error was at most
0.017 rad and its `A` error at most 0.38 m/s^2 (turning targets, first 0.2 s).
The tracker's were at most 0.063 rad and 1.7 m/s^2.

**Initialization history.** The runtime receives the radar detections of the
initialization window (0.5 s), with the GNSS position standing in for the
ego estimate and the gyro rates for the headings. The fit is therefore
available from the first frame of an encounter that starts inside the radar
range.

## 6. Target initialization in the adapter

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

## Tests

`tests/nrmmModelAidedEstimationTest.m` checks the following:

- the force-balance interval contains the true lateral velocity;
- the nominal point is exact without noise even where the interval is wide;
- contradictory measurements give an empty interval;
- the window fit recovers constant contract parameters and averages bounded noise;
- model selection keeps a straight forecast until curvature is significant;
- gyro-integrated headings keep the fit accurate for a turning ego with a yaw error;
- the estimator domain is computed from the controller.

`tests/nrmmTruthEnclosureAuditTest.m` checks that the shared vehicle model
removes the kinematic premise from the truth audit.

## Scope

The ego and target enclosures are conditional on the stated sensor bounds, the
shared model structure with its parameter box, the sideslip cone, and the
target contract. The plant used in the simulations has exactly this structure,
with the nominal parameters. The replay audit (`scripts/replayEstimatorValidity.m`)
covers every frame of the fourteen noisy encounters (seed 20261003). In it the
published ego and target bounds contained the truth in every frame, and the ego
bound stayed valid throughout. The true motion stayed inside the sideslip cone.
In one frame (15-m/s head-on, 1.25 s) it reached a rear-adhesion ratio of
1.008, since the controller's adhesion row acts on the estimate. The
force-balance certificate needs only the cone. The target prediction-parameter set
is still wide: course about +/-0.4 rad, speed +/-1.7 to 3.7 m/s, and the
whole contract range for `A` and curvature. The controller therefore uses the
current enclosure only for each hold (see `controller/PCBF_CLF_ARCHITECTURE.md`).
