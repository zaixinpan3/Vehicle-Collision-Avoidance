# Quadratic input deviation and CLF relaxation

## Current objective: certificate operating input (September 8, 2026)

For each predicted stage, the input cost is centered at the operating input
used to construct the continuous CLF/LQR certificate:

\[
J=h\sum_{j=0}^{N-1}\left[
 z_j^{\mathsf T}Q_z z_j
 +(u_j-u_\star)^{\mathsf T}R_u(u_j-u_\star)
 +w_{\Delta u}\left\|\frac{u_j-u_{j-1}}{h}\right\|^2
 +\rho\delta_j^2\right]+c_{\mathrm{switch}}.
\]

Here `u = [frontWheelSteeringAngle; brakingRatio]`, `u(-1)` is the
previous applied input, and `z` is the predicted five-coordinate state error
relative to `clf.referenceStart + clf.referenceRate*t`. The diagonal state
weights are the inverse squared error scales. The online input weights remain
`diag(clf.frontWheelSteeringAngleWeight, clf.brakingRatioWeight)`; this change
only replaces the center, without changing any weight. Riccati synthesis
continues to normalize its input weights by the actuator scales.
`encounter.inputRateWeight`, `clf.relaxationWeight`, and the maneuver-switch
constant retain their existing roles. Every finite-horizon stage is included;
there is no appended, unpenalized stopping tail in the current controller.

The certificate now uses the nonlinear cruise trim at the current road
curvature, `vStar = max(referenceSpeed, clf.certificateSpeedFloor)`, and the
model's declared longitudinal acceleration bias. The state and input solve
constant-speed Frenet kinematics and the full modified-Fiala axle-force
balance, including rotation of front-wheel forces and passive road load.
The synthesis matrices are the Jacobians evaluated at that same trim.
See [the curved-cruise derivation](CURVED_CRUISE_CERTIFICATE.md).

`qp.clf.certificate` records `operatingState`, `operatingInput`,
`operatingCurvature`, and `operatingAccelerationBias`. Both objective
transcriptions read this one `operatingInput`. They do not use
`prediction.referencePlan` as the cost center; that stage-dependent plan
remains an initialization and linearization seed. No `uStar - K*z` input
target or desired acceleration is introduced. Curvature and bias enter the
certificate cache key, so a changed working point cannot reuse a stale metric.

The metric and input center are fixed within a formulated horizon. On an
unchanging arc at the reference speed, the CLF state target, Riccati point and
input center agree. Below the synthesis speed floor, the state target still
uses the configured reference speed, while the metric and input center use
the declared floor; this remains a local regularization, not an exact trim
tracking guarantee. State-reference offsets/rates, all cost weights, hard
safety constraints, squared nonnegative slack and delayed execution retain
their existing roles.

For changing curvature, newly formulated frames can change the metric and
reference. `clfReferenceSwitchValue` reports the resulting value jump at the
same current state; `clfMetricChanged` reports a metric change. The per-frame
CLF constraints do not bound that jump. No common Lyapunov function or global
convergence proof for arbitrary curvature variation is claimed. Finite
squared slack also permits a tracking/dissipation tradeoff. Current measured
curve results and their precise plant/timing scope are documented
[separately](../scripts/CURVED_CONTROLLER_RESULTS.md).

## Historical straight-certificate validation (September 8, 2026)

All 157 distinct targeted MATLAB tests in 11 classes pass. Coverage includes
an explicit objective evaluation at a current speed different from the
certificate speed, an independent optimizer comparison, both SOCP
transcriptions, certificate force balance and Riccati identities above and
below the synthesis speed floor, invariance to changes in the prediction seed,
road-load changes, finite sensing, scheduled input elimination and
failure-without-fallback behavior. The new explicit-cost regression first
fails against the unmodified controller (cost difference 3.5378922e-5), then
passes after both centers are corrected.

Factory Code Analyzer checks cover all seven edited MATLAB files. Six have no
findings; the sparse transcription retains 16 sparse-indexing performance
advisories, reproduced on its unchanged baseline. No new finding is introduced.
The tests include declared-model scenarios; no new PassVeh14DOF, joint
closed-loop or realtime qualification is claimed.

## Historical scope

The September 6 results below belong to the older raw-input objective,
`h*sum(u'*Ru*u) + rho*delta^2`, with its former head/continuation structure and
acceleration-input plant. They are retained as experimental history and do not
validate the current finite-horizon signed-beta controller or objective.
The [force-balance study](LONGITUDINAL_FORCE_BALANCE.md) records the later
addition of aerodynamic and rolling resistance.

## Historical validation on September 6, 2026

The measurements below precede the modified Fiala and signed-beta input
migration. They describe the former acceleration-input controller; they are
not physical validation of the current tire and input model.

The final full working-tree suite ran 379 cases: 371 passed and 8 failed,
including 7 incomplete cases. All 66 controller, continuous-CLF, recovery
and sparse-QP cases passed. The eight failing cases were rerun against an
unchanged pre-edit source snapshot and reproduced the same failures:
six estimator integration cases reject unsupported terminal uncertainty;
a pre-existing untracked stationary-pose test requests an unsupported zero
cruise reference; another untracked test expects a curved-motion uncertainty
rejection that the baseline does not issue. These existing issues remain
outside the objective change. The complete suite is therefore not green.

The initial focused run had two numerical assertion failures. Independent
QP costs near 91089 differed by 0.000309 (relative error 3.39e-9); the check
now combines absolute and relative tolerances. The terminal-policy test
now checks the optimized terminal velocity residual separately (below
1e-7) and evaluates exact rest invariance at zero velocity. This preserves
the distinction between numerical feasibility and analytic invariance.
All eight edited MATLAB files pass factory Code Analyzer checks with no
reported issues. The manuscript builds successfully with `latexmk`.

## Thirty-second physical comparison

Two exact-state PassVeh14DOF crossing-target trials use the previous
30-second protocol: straight and radius 400 m, reference speed 15 m/s,
target speed 8 m/s, 50 m perception range, 0.05 s control period,
24 head stages, longitudinal input gain 0.8 and RNG 2026 (twister).
Road samples through 400 m are preserved exactly and appended through
1000 m. The prior 30-second slack-only sources match all pre-edit MATLAB
files byte for byte. Old 10-second target-free trials are reused only as
historical scenario counterfactuals; no new matched target-free trial is
claimed. Each new avoidance trial completed 600 commands, with no
fallback, no observed sampled contact and positive sampled road margin.

Maximum errors over every state sample in 9–10 seconds:

| Scene | Speed (m/s) | Lateral (m) | Heading (rad) | Recovery |
|---|---:|---:|---:|---|
| Straight | 0.657847 | 0.00070452 | 0.00019416 | Speed fails; lateral/heading pass |
| Arc | 0.721030 | 0.11377057 | 0.00658030 | Speed fails; lateral/heading pass |
| Limit | 0.5 | 0.2 | 0.02 | All samples must pass |

Both trials enter all three tolerances at 9.40 s and remain inside them
through the observed 30 s endpoint. This does not satisfy the complete
9–10 s window or establish asymptotic convergence.

Final 25–30 s RMS comparison (old slack-only to new quadratic objective):

| Scene | Speed (m/s) | Lateral (m) | Heading (rad) | Acceleration step RMS (m/s squared) |
|---|---:|---:|---:|---:|
| Straight | 0.132093 → 0.177398 | 0.0171933 → 1.85796e-09 | 0.00299539 → 1.79023e-10 | 8.41788 → 0.000199018 |
| Arc | 0.1364 → 0.109641 | 0.0898924 → 0.0428722 | 0.00661583 → 0.00160034 | 8.70751 → 0.301459 |

The straight initial acceleration changes from -4.561428 to approximately
-9.18e-7 m/s squared. Its late lateral and heading errors approach numerical
zero, but its mean speed error remains -0.1774 m/s; speed RMS is higher than
in the old oscillating trace. On the arc, lateral and heading RMS decline
by about 52% and 76%, respectively, while steering still switches. Arc
steering-step RMS rises from 0.012272 to 0.015457 rad even though its input
range narrows. Thus reduced acceleration variation must not be described
as elimination of every control oscillation or exact nominal recovery.

| Scene | Minimum SAT margin (m) | Minimum road margin (m) | Median / p95 / max call (ms) | Calls above 50 ms |
|---|---:|---:|---:|---:|
| Straight | 1.088251 | 7.641714 | 25.521 / 36.738 / 109.440 | 1 / 600 |
| Arc | 1.510876 | 7.463593 | 28.530 / 38.764 / 73.950 | 1 / 600 |

The 109.440 ms straight call occurs at t=3.10 s; 67.684 ms is in
formulation/witness construction, 21.327 ms in input preparation and
11.655 ms in the QP solve. The 73.950 ms arc call at t=3.15 s spends
47.461 ms in formulation/witness construction and 12.058 ms in the solve.
Both are the first target-visible calls. This localizes the observed
peaks, without identifying a particular construction subroutine as the
cause. The 50 ms all-call deadline criterion therefore fails in both runs.
Timing excludes the twelve discarded preparation calls, perception and
physical simulation; no computation delay is injected into the plant.
Sampled SAT/road checks do not certify intersample safety.

An isolated HEAD-plus-task-patch source copy, excluding unrelated dirty
work, also passes all 66 focused tests. Independent Python computations
match all twelve MATLAB window summaries. Raw trials, CSV traces, scripts,
source inventories, native-binary hashes and comparison plots are retained
in the external evidence bundle; generated simulation files are not added
to the project repository.
