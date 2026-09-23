# Feedback (tube) prediction of the ego uncertainty

Status: implemented 2026-09-23 (Section 7), default on
(`cfg.feedbackPrediction.enabled`). It addresses the open-loop growth of the
ego uncertainty found in
[ADMISSION_INFEASIBILITY_DIAGNOSIS_20260923.md](../report/ADMISSION_INFEASIBILITY_DIAGNOSIS_20260923.md).

## 1. The problem

The controller plans an input sequence `u_0..u_{N-1}` and certifies it for
every true initial state in the estimator box `x_0 - xhat_0 in H_0 = [-eps, eps]`.
The prediction assumes this whole sequence is executed **open loop**, with no
later measurement, so the box is propagated by
`rho_{k+1} = |A_k| rho_k` (`ltvBicycleModel.finitePredict`). Heading and speed
errors integrate into lateral and station errors, and over a 4.8 s encounter
horizon the ten-fold-uncertainty box grows from 0.1 m to 0.52-0.74 m
laterally. Every node constraint, the exit row and the terminal cone must hold
for that whole box. In the three ten-fold cases the box no longer fits the
terminal set, even with the state exactly on the reference.

In execution the controller re-plans every hold from a new estimate whose
error stays within the declared bound. The open-loop box describes an
execution that never happens.

## 2. Formulation

Plan a **feedback policy** instead of an input sequence:

    u_k = v_k + K_k (xhat_k - z_k),        z_{k+1} = A_k z_k + B_k v_k + c_k,

with nominal inputs `v_k` as the decision variables (same count as today),
nominal states `z_k`, and fixed gains `K_k`. `xhat_k` is the estimate that will
be available at hold k.

Estimator contract (explicit): at every future hold,
`x_k - xhat_k in H_k = [-epsbar, epsbar]`, with `epsbar` the per-hold estimator
bound. The plant contract is unchanged (declared held affine plant). Section 2a
shows that the existing estimator already publishes such a bound at every
controller time.

The deviation `e_k = x_k - z_k` obeys, with `eta_k = xhat_k - x_k`:

    e_{k+1} = (A_k + B_k K_k) e_k + B_k K_k eta_k,

so `e_k` lies in a zonotope `E_k` with generator matrix

    G_{k+1} = [ (A_k + B_k K_k) G_k ,  B_k K_k diag(epsbar) ].

`E_k` does not depend on the decision variables, so it is computed once per
frame before the solve. With a stabilizing `K`, `E_k` stays bounded instead of
growing with the horizon: the per-hold measurement error is fed back and
re-injected, but it is not integrated open loop.

## 2a. What the estimator feeds back

`onlineNrmmTrackingRuntime` runs at 80 Hz (`observer.runtime.samplePeriod =
0.0125` s). In the synthetic pipeline every observer sample carries GNSS
position and velocity, IMU acceleration, gyro yaw rate and radar
(`estimatorControllerIntegrationConfig`), and the 0.05 s controller times lie on
that grid. At each controller time `nrmmControllerErrorBounds` publishes
`controllerStateErrorBound`, re-anchored to the current measurements rather
than propagated from the previous frame:

| Channel | Published bound | Source |
| --- | --- | --- |
| position | `\|phat - y_GNSS\| + eps_p + Vmax*age` (age = 0 on the grid) | GNSS fix, 0.04 m noise |
| heading | radius of the propagated and intersected yaw set about `psihat` | gyro plus GNSS course correspondence |
| body velocity | velocity-observer ISS radius `B_v`, ultimately `U_v` | GNSS velocity, IMU, gyro, single-track mismatch premise |
| yaw rate | gyro noise 0.0015 rad/s (+ yaw acceleration * age) | gyro |

The recorded bound in controller coordinates is
`[0.076 m; 0.076 m; 0.048 rad; 0.089 m/s; 0.497 m/s; 0.0015 rad/s]`
(`probeEgoBoundSensitivity`, 2026-09-17). These bounds do not grow from frame
to frame, which is exactly the property the feedback policy needs. The error
itself is the state of first-order observers (for example
`e_v' = -k_v e_v + d`, `k_v = log(20)/T_domain`), so it varies slowly between
holds rather than jumping across its box. Treating it as arbitrary at every
hold is sound but conservative.

Two estimator facts shape the gain. The heading bound (0.048 rad) is 25-100
times the actual heading error (about 0.0005 rad RMSE,
[ESTIMATOR_IN_THE_LOOP_ADMISSION_20260917.md](../report/ESTIMATOR_IN_THE_LOOP_ADMISSION_20260917.md)).
Open-loop propagation integrates it into lateral position (0.048 rad over 3.6 s
at 10 m/s gives 1.7 m). With feedback, lateral position is corrected from the
GNSS fix every hold, so it no longer accumulates. The lateral-velocity bound
(0.497 m/s) comes mostly from the single-track mismatch premise
(`l_E sec b delta_st`), not from sensor noise, so feeding it back injects a
large error for little information.

## 3. Constraint tightening

Every tightening is a constant, because the separation directions are fixed
before the solve. Nothing new enters the SOCP except different constants.

| Constraint | Current tightening | Feedback tightening |
| --- | --- | --- |
| collision record at node k | generators `positionMap*diag(rho_k)`, yaw radius `\|yawRow\| rho_k` | generators `positionMap*G_k`, yaw radius `sum \|yawRow G_k\|` |
| exit row | `\|row\| rho_N` | `sum \|row G_N\|` |
| terminal modal cone | `\|modal\| rho_N` | `sum_j \|modal g_j\|` over the columns of `G_N` |
| actuator amplitude | none (inputs exact) | `\|v_k\| <= umax - sum \|K_k [G_k, diag(epsbar)]\|` |
| slew | none | support of `K_k(e_k+eta_k) - K_{k-1}(e_{k-1}+eta_{k-1})`, written in `G_{k-1}`, `eta_{k-1}`, `eta_k` |

The cost of feedback is the actuator authority it reserves for correcting the
measurement error.

## 4. Recursive feasibility

Fresh admission: `z_0 = xhat_0` (the conditioned center) and `E_0` the
measurement box. The first held input is exact, so `E_1 = A_1 E_0` and its
interval hull, which the next frame intersects with its measurement box, is
the same box as before this change.

Inherited frame at the next hold: the controller does not rebuild the
prediction. It carries the previous frame's rows, records and cones (the
existing shift of the accepted affine family) with the executed plan input
eliminated. Every carried tightening was computed for the whole deviation set
of the previous frame, and that set does not depend on the remaining plan
inputs. With the carried nominal `z_1` the frame issues

    u' = v' + K (xhat' - z_1),

the shifted feedback policy. Its true deviations stay in the carried sets as
long as the estimator error stays within the declared bound, so every carried
constraint still holds for any remaining plan the solver returns. The
previous plan is a feasible point, as before. No zonotope is ever re-boxed and
propagated again, which is the monotonicity concern of
INFORMATION_STATE_PCBF.md, Lemma 1.

The estimator contract is part of the carried certificate. A measurement bound
larger than the carried `estimatorBound` voids it, and the frame is re-admitted
from the measurement (`measurementContractChanged`). The terminal set is
already robustly invariant under its own feedback law with measurement noise
up to the same measurement radius limit (`holdNoise`, `disturbanceSupport`).
The terminal cone requires the whole final deviation set inside it; the
terminal-law slew row uses the correlated deviation of the last planned input.

## 5. Evidence (offline, captured frames)

Computed on the frames captured in the infeasibility diagnosis. `K` is the
existing cruise LQR gain (`terminal.feedback`), and the ten-fold estimator box
is used as `epsbar` at every hold. The margin is the terminal modal margin with
the nominal exactly on the reference, i.e. whether the error set alone fits the
terminal set:

| Case | Open loop | LQR K | 0.5 K | Feedback input use (K): steering / braking ratio |
| --- | --- | --- | --- | --- |
| 39 straight, x10 | -0.029 | **+0.153** | +0.131 | 0.080 rad / 0.37 |
| 42 curved 0.01, x10 | -0.064 | **+0.136** | +0.109 | 0.103 rad / 0.39 |
| 53 curved 0.01, x3 | +0.103 | +0.162 | +0.154 | 0.031 rad / 0.12 |
| curved 0.01, x1 | +0.150 | +0.170 | +0.167 | 0.010 rad / 0.04 |

Terminal lateral radius (x10, straight) falls from 0.515 m to 0.210 m.
Heading radius rises from 0.011 to 0.035 rad because the measurement error is
re-injected every hold. With exact state feedback (no noise) the error set
converges to zero. Zonotope supports matter: the interval-hull measure the
controller uses today would report -0.178 for the same LQR tube.

## 5a. Evidence with the recorded estimator bound

Same frames and gain as above, with the recorded estimator bound used as the
estimation error at every hold, 4.8 s horizon
([estimatorBoundTube.txt](../report/FEEDBACK_TUBE_EVIDENCE_20260923/estimatorBoundTube.txt)).
"Bias-like" treats the error as constant over the horizon, closer to the
observer's slow error dynamics than the arbitrary per-hold worst case.

| Straight stationary | Max lateral radius | Max heading radius | Terminal margin | Feedback use: steering / braking |
| --- | --- | --- | --- | --- |
| open loop (current) | 2.021 m | 0.050 rad | **-0.756** | - |
| LQR K, arbitrary per hold | 0.583 m | 0.097 rad | +0.101 | 0.220 rad / 0.333 |
| LQR K, bias-like | 0.582 m | 0.067 rad | +0.101 | 0.195 rad / 0.333 |
| K without lateral-velocity feedback, arbitrary | 0.445 m | 0.077 rad | +0.120 | 0.170 rad / 0.333 |
| K without lateral-velocity feedback, bias-like | 0.445 m | 0.056 rad | +0.127 | 0.149 rad / 0.333 |

The curved crossing frame gives the same ordering (open loop 2.051 m and
-0.759; without lateral-velocity feedback 0.487 m and +0.101 to +0.113). The
open-loop margin explains why the estimator-in-the-loop admission is
infeasible today. With feedback it becomes feasible, but the cruise LQR gain
reserves about a third of the braking range for the speed error alone. The
gain for this purpose should therefore omit lateral-velocity feedback and use a
softer speed gain. It may later model the observer error dynamics in the tube
instead of an arbitrary per-hold error.

## 6. Not addressed

- **Target uncertainty.** In case 54 the target's declared reachable set grows
  to 18 m radius. The ego policy does not feed back future target
  measurements, so this growth remains.
- **Model mismatch.** The PassVeh14DOF plant violates the zero-residual
  contract; nothing here changes that.
- **Gain design.** Section 5a argues for a gain without lateral-velocity
  feedback and with a softer speed gain; a time-varying LQR along the anchor
  schedule is another option. The gain trades tube size against reserved
  actuator authority.
- **Estimator conservatism.** The heading and lateral-velocity bounds dominate
  the remaining tube; tighter certified estimator bounds would shrink it
  further.

## 7. Implementation

- `config/collisionAvoidanceControllerConfig.m`: `feedbackPrediction` with
  `enabled` (true), `speedGainScale` (0.5) and `lateralVelocityFeedback` (false).
- `ltvBicycleModel.feedbackContract`: `K = [0, -gain]` from the frame's cruise
  certificate with the speed column scaled and the lateral-velocity column
  zeroed; `epsbar = max(initial bound, measurement radius limit)`.
- `ltvBicycleModel.finitePredict`: per-node generators (`cells(k).generators`)
  in one append-only source basis (initial box, one estimator box per node),
  plus an interval part for held process reserves and arithmetic allowances.
  It also returns the per-stage support of the executed-input deviation
  (`feedbackInputSupport`), of its change between holds
  (`feedbackSlewSupport`) and the final input deviation for the terminal slew
  row.
- `avoidanceSafetyGeometry`: collision and exit records take the node
  generators (position zonotope and yaw radius).
- `hardEncounterBarrier`: terminal cone with modal zonotope supports; terminal
  slew support; stored-state check `appliedInput = plan(:,1) + feedbackCorrection`;
  carried `issuedInput` is the plan value; estimator-contract check; the
  target-free horizon proposal uses the same recursion.
- `formulateAvoidanceProblem`: amplitude and slew rows reserve the input
  deviation support; inherited frames compute
  `feedbackCorrection = K (xhat - z_1)`, which also enters the first-hold CLF.
- `collisionAvoidanceController`: issues `plan(:,1) + feedbackCorrection`;
  controller-state format 43 stores the correction; metadata reports the gain,
  bound and correction.
- `tests/feedbackPredictionTest.m`: sampled initial and per-hold estimator
  errors (interior and vertices) stay inside every predicted set, input and
  slew deviations stay inside their reserved supports, inherited frames issue
  the corrected input, and tampered corrections or larger estimator bounds are
  rejected.
