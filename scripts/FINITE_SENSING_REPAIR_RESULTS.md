# Finite-sensing integration repair: experimental results

Experiment date: September 7, 2026. MATLAB R2026a Update 3,
26.1.0.3276743. Outcome: **partial repair; joint avoidance is not complete**.

The implementation removes the original admission and interface failures,
but the final physical straight and circular trials still stop when the
current optimization has no independently verified solution. No backup
controller or previously stored control is executed after failure. The
requested 12 s duration is an observation window, not an early-recovery
requirement or a proof of eventual convergence.

## Implemented changes

- Match target admission, continued observation and release to finite,
  timestamped perception. No target road corridor or infinite-future
  nonreturn promise is required by the finite-sensing path.
- Accept refreshed road measurements while retaining the execution contract;
  shorten lookahead to the actually covered swept geometry. Admission and
  constraint construction share the same frame and recheck each new anchor.
- Use analytic straight/circular Frenet charts, including circular frame
  uncertainty, instead of treating sampled circle vertices as heading jumps.
- Tighten target error enclosures from bounded-noise measurement history,
  transporting common ego-heading uncertainty through gyro history. Preserve
  the NRMM point-observer equations and gains, including the yaw observer.
- Read the active MF62 plant parameters, retain road load, correct the
  tire-speed floor and nonlinear tire/body/Frenet linearization, and check
  each candidate with a nonlinear rollout. Preserve the scenario collision
  rectangle after active plant-parameter refresh.
- Lift the conic program into sparse stage equations and align the external
  solver hook with that full decision vector. Independently verify physical
  controls after solving. Retain joint input/state cost and squared CLF
  slack; introduce no desired-acceleration objective.
- Record actual generated sensor-noise checks and sampled truth-containment
  audits, including the failed controller sample. Infinite unavailable
  bounds cannot pass the containment audit.

The default robust calculation covers the next held interval; later stages
use nonlinear nominal lookahead. Extra future clearance is an optional
allocated reserve, while physical rows remain hard. This is not a
recursive-feasibility theorem. The detailed contract and derivations are in
[FINITE_SENSING_CONTROLLER.md](../controller/FINITE_SENSING_CONTROLLER.md).

## Final trials

All three trials request 12 s, with a 0.05 s controller period, up to 40
prediction stages, a 10 m/s cruise reference and a 10 m/s opposing target.
The target starts 100 m away with a 0.8 m lateral collision offset; perception
has a 30 m radius. The collision rectangles are 5 by 2 m. The circular road
has radius 60 m and 300 m of centerline after the initial station.

The physical trials use PassVeh14DOF and NRMM with seeds 20260907 (straight)
and 20260908 (circle), a 0.0125 s observer period, and RK4 steps at most
0.0025 s. Bounded synthetic noise limits are GNSS position 0.04 m, GNSS
velocity 0.05 m/s, IMU acceleration 0.03 m/s^2, gyro 0.0015 rad/s and radar
position 0.04 m. The target speed domain is 5--20 m/s, scalar acceleration
limit 2 m/s^2, sideslip limit 0.005 rad and additional model jerk zero.
The saved configuration structures are the authoritative complete inputs.

`finiteSensingValidationConfig` supplies derivative residual allowances
`[0.2; 0.06; 0.02; 2.5; 5; 4]` in `[s; d; ePsi; vx; vy; r]` units per
second, steering-rate limit 0.5 rad/s and braking-ratio-rate limit 2/s.
The heading domain remains 0.4 rad. These residual allowances were
identified using earlier traces; they are not globally proved bounds.

| Case | Executed intervals | Stop time | Minimum sampled SAT margin | Outcome |
| --- | ---: | ---: | ---: | --- |
| Physical straight + NRMM | 94 | 4.70 s | 1.966938 m | No verified current solution |
| Physical circle + NRMM | 87 | 4.35 s | 12.170070 m | No verified current solution |
| Nonlinear bicycle + exact observations | 99 | 4.95 s | 0.581052 m | No verified current solution |

The exact-observation diagnostic uses the final straight controller
configuration and the same constant straight target, but its plant is the
modified-Fiala bicycle and its straight road has no curb constraints.
It isolates important estimator/plant differences without being a matched
high-fidelity validation trial. Its SAT margin is evaluated at dense ode45
trace samples. The physical trials use their saved plant trace samples.
None of these sampled positive margins proves clearance after termination.

Both physical runs acquire the target and preserve road clearance before
stopping. Neither passes the target or reaches the recovery window. The
straight and circular minimum road-boundary-function margins are 5.039138
and 7.384036, respectively. These are polynomial-function values, not
Euclidean distances. Extending the requested duration cannot advance a run
after the required stop-on-failure condition.

## Remaining mechanisms

**The straight observer premise is violated.** The current kinematic
correspondence assumes

\[
 |r-v_y/l_r|\leq 0.10\ \mathrm{rad/s}.
\]

This is a rear-tire kinematic premise, not an identity of a dynamic vehicle.
The final straight trace first violates it at 4.20 s and reaches
0.185883 rad/s. At 4.55 s, actual ego yaw error first exceeds the still
published finite yaw bound. The minimum yaw containment slack is
-0.00360669 rad. Small aggregate estimator RMSE does not establish a valid
pointwise error enclosure. The new audit exposes this failure; it does not
repair the premise or feed truth into estimation/control.

**Planning failure also occurs with valid sampled bounds.** In the final
circular trace, every audited ego premise and published known component
bound passes, including the failed sample. Its maximum rear mismatch is
0.086467 rad/s. Nevertheless all maneuver margin programs become infeasible.
The exact-observation diagnostic also fails. Therefore estimator error is
not a sufficient explanation of the remaining local planning failures.
An infeasible convex inner approximation is not proof that physical
collision avoidance is impossible. Current nonlinear relinearization has
no general convergence or future-feasibility guarantee.

**The final sampled plant residuals fit the empirical allowance.** Against
each recorded executed continuous generator, component maxima are
`[0.008232; 0.009093; 0.000863; 1.826167; 3.576491; 2.273151]` for the
straight run and
`[0.000740; 0.001100; 0.000247; 0.836678; 3.105542; 2.550618]` for the circle.
These finite-difference checks do not prove a continuous-time bound over
all states. Endpoint discrepancy divided by the sample period is recorded
separately and is not substituted for a derivative bound.

Earlier counterfactuals distinguish these mechanisms further. An exact
bicycle trial with heading domain 0.6 rad, steering-rate limit 1 rad/s and
20 refinements completes 12 s, ending at 9.997343 m/s, lateral position
-0.000994 m and heading 0.000128 rad. Its collision width is 2.157 m and it
has no curbs. It is a different configuration, not a success of the final
default profile. A companion NRMM bicycle trial with mismatch allowance
0.5 rad/s and yaw-rate domain 1.5 rad/s retains sampled containment but
fails nonlinear planning at 4.15 s. Increasing a single observer bound
does not solve the integration problem.

Exploratory terminal costs, proximal penalties, capped hard target buffers,
fixed lateral normals and pulse-hold seeds did not resolve joint failure
and were removed. A robust repair of dynamic-vehicle observation and a
principled nonlinear feasibility method remain open; no unvalidated new
observer or hidden fallback is presented as their solution.

## Validation and timing

`results = runtests('tests'); assertSuccess(results)` passes all **501**
tests, with zero failed or incomplete cases. Behavior coverage includes
finite sensing, updated road coverage, circle charts, independent lifted
and condensed programs, solver hooks, target-history enclosures, sampled
truth auditing, and discrete Jacobians compared with independent ode45
perturbations. Short physical-prefix tests validate admission only; the
failed full trials above remain separate experimental outcomes.

Code Analyzer inspected 48 changed/new MATLAB files. There are no reported
syntax errors. Nine sparse-index performance findings remain in
`avoidanceStageQp.m`; five pre-existing property-name findings remain in
`modifiedFialaTireTest.m`. `git diff --check` passes. The controller source
budget remains 20 files under the documented counting rule.

| Case | Mean successful controller call | Maximum successful call | Calls over 50 ms | Failed call |
| --- | ---: | ---: | ---: | ---: |
| Physical straight | 84.803 ms | 607.097 ms | 94/94 | 466.249 ms |
| Physical circle | 65.197 ms | 167.706 ms | 86/87 | 410.442 ms |
| Exact bicycle | 63.450 ms | 356.650 ms | 80/99 | Not instrumented |

These are shared-desktop controller-call measurements, not isolated
benchmarks, full perception-to-actuation pipeline measurements or WCET.
They do not establish a 50 ms real-time pass. The physical exact-state
comparison is not a valid basis for attributing the entire timing
difference to estimator execution, which is outside the measured call.

## Reproduction and artifacts

The primary entry is `runEstimatedStateAvoidanceScenarios` with
`Duration=12`, `ReferenceSpeed=10`, `TargetSpeed=10`, `Seed=20260907` and
`CircularRadius=60`. The final single-scenario repetitions use
`runOncomingVehicleAvoidanceScenario` and
`runCircularCenterlineStraightTargetAvoidanceScenario` with the parameters
above and the saved suite estimator configurations. The exact diagnostic
is `runFiniteBicycleDiagnostic(straightFinal.controllerConfiguration,12)`.

Original MAT, CSV, JSON, analysis scripts and plots reside outside the
repository in
`/home/zai/.cache/collisionAvoidance/finite-sensing-repair-20260907/`.
Final records are `final-validation.mat/.csv`,
`final-straight-window-12s.mat`, `final-circle-window-12s.mat`,
`final-exact-12s.mat`, `final-repair-analysis.json`, and
`final-repair-trajectories.pdf/.png`. The earlier counterfactual pair is
`clean-diagnostic-12s.mat`; its analysis is
`repair-evidence-analysis.json`. Archive copies preserve source/configuration
provenance and identify exploratory versions separately from final runs.
