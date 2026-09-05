# Controller design-requirement experiments

`runControllerDesignExperiments` compares nominal cruise with range-triggered
avoidance on a straight path and a circular arc. Both trials use the same
hard-CBF/soft-CLF controller and the actual Vehicle Dynamics Blockset
`PassVeh14DOF` plant. `controllerDesignExperimentConfig` owns the experiment
parameters; it does not add a controller mode or a different control algorithm.

```matlab
addpath('scripts');
study = runControllerDesignExperiments( ...
    OutputDirectory='/tmp/controller-design-experiments', Plot=false);
disp(study.summary);
```

The driver writes each completed or controller-stopped trial to a separate MAT
checkpoint before continuing. It returns controller failures as research
results. An infrastructure exception still raises an error. Once both trials
for a path return, the independent evaluator reports their requirements and
the driver exports a trajectory/clearance/speed/tracking figure. The final
outputs are `summary.csv` and `study.mat`. Generated outputs are not source-code
artifacts and should not be committed to the repository.
`plotControllerDesignExperiment` can regenerate a scene figure directly from
`study.results(k)` and `study.configuration` without rerunning the plant.

## Physical setup

| Quantity | Straight | Left circular arc |
| --- | ---: | ---: |
| Ego cruise speed | 15 m/s (54 km/h) | 15 m/s (54 km/h) |
| Path curvature | 0 | 1/400 m^-1 |
| Initial ego yaw rate | 0 | 0.0375 rad/s |
| Initial ego pose | (0, 0, 0) | (0, 0, 0) |
| Initial body lateral velocity | 0 | 0 |
| Target speed | 8 m/s | 8 m/s |
| Target direction | Across the path | Across the tangent at the encounter |
| Nominal center-coincidence time | 6 s | 6 s |
| Detection radius | 50 m | 50 m |
| Vehicle rectangles, length x width | 4.8 x 1.9 m | 4.8 x 1.9 m |
| Duration | 10 s | 10 s |
| Paved-road offsets, right/left | 6 / 6 m | 6 / 6 m |
| Additional shoulder on each side | 2.6 m | 2.6 m |
| Control sample time | 0.05 s | 0.05 s |
| Dynamic prediction steps | 24 (1.2 s) | 24 (1.2 s) |
| Conic constraint / optimality tolerance | 1e-6 / 1e-6 | 1e-6 / 1e-6 |

The controller also optimizes its braking tail beyond the dynamic prediction
horizon. The tail length and rest-admission conditions remain those of the
single controller implementation. Road geometry is supplied as the same
perception-limited quadratic fits used by the existing Blockset harness.
The target starts outside sensor range, crosses the road from the right, and
continues along its inertial straight line. It is a prescribed moving
rectangle, not a second high-fidelity controlled vehicle.

Let the intended encounter time be `tc=6`, path station `s=15*tc`, and curvature
`k`. The nominal position is `(s,0)` for the straight path and
`(sin(k*s),1-cos(k*s))/k` for the arc. At the encounter, the target velocity is
`8*(-sin(k*s),cos(k*s))`; subtracting `tc` times that velocity from the encounter
position gives its initial position. Thus the intended nominal trajectories
coincide at the encounter by construction. The high-fidelity nominal run must
also actually overlap the target rectangle to establish the counterfactual.
Initial center ranges exceed 100 m; detection should occur after roughly 3 s
of cruise. Actual acquisition is measured from the simulated ego position,
not imposed at an idealized time.

### Plant and sensing contracts

MathWorks lists the preassembled passenger models as 3, 7, and 14 DOF.
`PassVeh14DOF` has six body degrees of freedom and two per wheel, including
wheel rotation and vertical motion. This experiment uses that template,
including its combined-slip Magic Formula tires, suspension, wheel dynamics
and disk brakes. It does not replace the template with a custom bicycle or a
custom function called a 14-DOF model. See the official
[passenger-model description](https://www.mathworks.com/help/vdynblks/ug/passenger-vehicle-dynamics-models.html).

The existing adapter reads mass, yaw inertia, axle geometry, center-of-gravity
height and nominal axle tire parameters from the loaded plant. It maps the
controller's axle longitudinal forces equally to the left/right wheels of
each axle, using loaded rolling radii to obtain drive torque or disk-brake
pressure. Steering and lateral/yaw signs map between the controller's
forward/left/up frame and the template's SAE forward/right/down frame.
Wheel angular speed is initialized from the zero-time loaded rolling radius
to avoid an artificial start-up slip impulse. Fast Restart retains the
compiled plant between control intervals, and operating points carry the
complete dynamic state across those intervals.

These tests isolate control behavior: ego state is plant truth, and target
position/velocity/yaw/extent are exact while its center is within 50 m of the
ego center. There is no sensor noise, delay, occlusion, field-of-view cutoff,
or estimator uncertainty. The controller receives no target before acquisition.
The target trajectory exists throughout the simulation for evaluation.
No random sampling is used. Separate estimator-integrated experiments are
needed to assess sensing and estimation errors.

In the nominal counterfactual, the same controller follows the same path with
an empty target list throughout. The evaluator overlays the identical target
trajectory on that plant run. The first rectangular overlap establishes the
collision threat; the model does not simulate contact forces or post-impact
dynamics. Subsequent nominal states are counterfactual continuation only.

## Acceptance requirements

1. **Valid scenario:** nominal high-fidelity cruise completes and overlaps the
   target; the target initially lies outside 50 m, remains hidden until the
   range test admits it, and appears after at least 2 s of cruise.
2. **Cruise before acquisition:** longitudinal speed error <=0.5 m/s, lateral
   tracking error <=0.2 m, and heading error <=0.02 rad through first detection.
3. **Collision freedom:** the full avoidance run completes, and the independent
   oriented-rectangle separating-axis test finds no contact at every control
   step, including the initial and final state. Inter-sample motion is not a
   collision acceptance criterion. The signed SAT margin is an axis gap, not
   an exact Euclidean clearance distance.
4. **Road containment:** every control-step vehicle rectangle stays within the
   analytic outer road boundaries. Circular-road checks use the rectangle's
   minimum and maximum radius, including possible edge-interior radial minima.
   This check is independent of the controller's fitted-road inequalities.
5. **Control constraints:** all returned commands satisfy the declared steering,
   acceleration and axle-friction limits, and all successful plans report
   satisfied hard rows and terminal admission. Commanded friction utilization
   is model-based and does not establish a plant tire-force bound.
6. **Recovery:** the last complete 1 s of the 10 s trial satisfies the cruise
   tolerances again after acquisition.
7. **Computation:** every attempted controller call completes within 0.05 s.
   This is reported separately from functional acceptance; the simulation is
   offline, and computation does not insert a delay into plant actuation.

An interrupted run with positive observed clearance fails scenario completion
and collision-freedom acceptance. The failed acquisition sample is retained
in `result.attempts`, even when no command was applied at that sample.
These two deterministic examples test specified behaviors; they do not prove
global safety, real-time deployment, robustness to unmodeled plant error, or
performance across all encounters.

## Initial configuration check

The original 48-step setting failed the target-free straight initial solve
with the Blockset-derived vehicle parameters: `coneprog` exit flag -7
(small search direction without meeting its tolerances). At that same state,
12, 24 and 32 steps solved with hard-row residuals near 1e-10. The experiment
therefore fixes 24 steps for both paths and both trials. The arc's initial
24-step solve also returned exit flag -7 at the original 1e-7 tolerances.
Uniform 1e-6 constraint/optimality tolerances admitted it with a hard-row
residual of approximately 1.7e-9. A five-step Blockset arc cruise then completed
with maximum speed/lateral/heading errors of 0.0186 m/s, 0.0025 m and
0.000183 rad. The final paired experiment uses those tolerances for every
trial; geometric contact acceptance is unchanged. The controller's global
defaults and algorithm are unchanged.

The original 1e-7 straight trial is retained as an initial diagnostic. Its
target-free nominal run completed 200 steps with maximum speed error
0.01845 m/s and first rectangular contact at 5.80 s. The target-aware run
acquired the target at 3.10 s and range 49.35 m, then stopped at 3.80 s
with `collisionAvoidanceController:noSolution`. Its smallest observed SAT
gap was still 29.83 m, so it demonstrated a loss of planning feasibility,
not a physical collision. This diagnostic did not meet avoidance completion.

## Executed result: September 5, 2026

The final common configuration was run with MATLAB R2026a Update 3. Both
target-free nominal trials completed all 200 control intervals. The straight
nominal run had maximum speed error 0.01845 m/s. The arc nominal run had
maximum speed/lateral/heading errors of 0.01892 m/s, 0.01310 m and
0.001600 rad. Both satisfy the prescribed initial-cruise requirements.

| Measured event or outcome | Straight | Circular arc |
| --- | ---: | ---: |
| First target observation [s] | 3.10 | 3.15 |
| Center range at acquisition [m] | 49.3485 | 49.5506 |
| First nominal rectangle overlap [s] | 5.80 | 5.80 |
| Nominal minimum SAT margin [m] | -3.2423 | -3.2450 |
| Controller stop time [s] | 3.80 | 3.80 |
| Completed avoidance intervals / requested | 76 / 200 | 76 / 200 |
| Minimum observed avoidance SAT margin [m] | 29.8252 | 31.0443 |
| Minimum observed road margin [m] | 7.6234 | 7.6281 |
| Maximum attempted solve time [s] | 1.2849 | 1.6139 |
| Functional requirements met | No | No |
| 50 ms computation requirement met | No | No |

Both avoidance trials returned `collisionAvoidanceController:noSolution` at
control attempt 77: neither the shifted start nor the cruise start produced
a feasible hard-CBF/soft-CLF plan. The controller issued no command at that
attempt. The target remains a threat under nominal continuation, but the
actual recorded plant trajectory ends at 3.80 s. No physical collision or
completed avoidance maneuver is asserted for this truncated trajectory.
Restoration of cruise after the encounter was not reached.

The maximum hard-row residual among returned plans was 3.44e-7 for the straight
trial and 2.20e-7 for the arc, within the configured numerical tolerance.
The maximum commanded axle-friction utilization was 0.518 and 0.400. These
plan/command checks do not repair the subsequent loss of planning feasibility
on the Blockset plant. The result establishes failure of the implemented
closed loop in these two experiments; it does not prove that no physically
safe maneuver exists. Further investigation should isolate target-constraint
construction, terminal admission and plant/prediction mismatch before changing
the control algorithm. Computation also requires improvement for a 50 ms
deployment period.

The focused final validation contains 82 passing tests (controller,
configuration, rectangle geometry, road fitting, experiment evaluation and
Blockset harness), with no failed or incomplete cases in that selected suite.
This does not represent a passing whole-repository test run. Four trial MAT
checkpoints preserve the simulation output even if a tool call times out;
the final long MATLAB MCP call timed out at the transport layer, and a
subsequent call verified its completed study and saved results.

## Retest of the certificate-preserving controller: September 5, 2026

The four trials were repeated with controller commit
`85f9e340e5a883a00e92cdf209ba248415ea2fc0` using an isolated Git source export.
The same experiment configuration was retained: 15 m/s ego cruise, 8 m/s
crossing target, 50 m perception, straight/400 m circular paths, 0.05 s sample
time, 24 head stages, and 1e-6 SOCP tolerances. The experiment's 24-stage
override remains explicit; this is not a test of the global 48-stage default.
The revised controller uses full steering/acceleration decisions and the same
dynamic model throughout its continuation. There are 98 stages and 99 nodes.
The actual PassVeh14DOF plant and independent control-node evaluator were
unchanged. Uncommitted estimator changes were excluded; sensing remained ideal.

All four trial calls returned and saved checkpoints. Both scenes stopped
before target acquisition, during ordinary cruise:

| Result | Straight | Arc |
| --- | ---: | ---: |
| Requested duration [s] | 10.00 | 10.00 |
| Nominal and target-aware stop time [s] | 1.00 | 0.50 |
| Completed intervals / requested | 20 / 200 | 10 / 200 |
| Failed control attempt | 21 | 11 |
| Target observations received | 0 | 0 |
| Target center range at stop [m] | 85.0119 | 97.2916 |
| Maximum observed speed error [m/s] | 0.01824 | 0.0185 |
| Maximum observed lateral error [m] | 2.08e-8 | 0.0051 |
| Minimum observed target SAT gap [m] | 71.6635 | 87.1463 |
| Minimum observed road margin [m] | 7.6500 | 7.6425 |
| Maximum target-aware controller call [s] | 2.476907 | 5.220744 |
| Complete functional acceptance | No | No |

Within each scene, nominal and target-aware control-state arrays are exactly
equal, as expected before the first observation. Physical tracking errors
remain small through the recorded interval. The acceptance flags are false
because the runs stop before the required cruise/acquisition/encounter/recovery
sequence, not because the measured tracking errors exceed their tolerances.
No contact is observed at the sampled nodes. These short traces establish
neither completed avoidance nor post-avoidance cruise recovery. Every attempted
controller call exceeded 50 ms. These are observed offline wall times, not a
hardware-isolated performance benchmark, and they were not injected as
actuation delays into the plant.

### Straight: successful solver status rejected by terminal acceptance

The failed call reports `collisionAvoidanceController:optimizationFailure`:
the solver returned exit flag 1, but independent acceptance rejected
`terminalRest`. Replaying the recorded states reproduced the result. Its
terminal equality residual is `1.09689129e-5`, slightly above the absolute
acceptance limit `10 * constraintTolerance = 1e-5`. The inequality residual
is `1.0076e-6`, the CLF residual is `1.1421e-6`, and the input bounds pass.
An independent LP including all inequalities, terminal equalities and bounds
is feasible. Thus this failure is a disagreement between the solver's
successful numerical termination and the controller's absolute endpoint
acceptance, rather than established emptiness of the hard feasible set.

A fresh initial-admission solve at the identical measured failure state
passes, with terminal equality residual `8.0734e-8` and inequality residual
`4.4852e-9`. This was a diagnostic snapshot; it was not substituted into the
plant run and does not justify silently loosening the acceptance threshold.

### Arc: a one-micrometre gap between terminal route intervals

The arc call reports `collisionAvoidanceController:noSolution`, with solver
exit flag -2. A separate LP at 1e-9 feasibility/optimality tolerances confirms
infeasibility. Removing only the `routeDomain` rows restores feasibility;
removing road, lateral-domain, heading-domain, speed-domain, friction, or
terminal-rest rows separately does not.

The last two node frames select opposite sides of one polyline boundary:

```text
node 98: s >= 240.599999373430649 m
node 99: s <= 240.599998373430651 m
```

The final acceleration is fixed to zero, the longitudinal bias is zero, and
terminal speed is zero. The final dynamic step therefore requires
`vx_98 = vx_99 = 0` and `s_98 = s_99`. The displayed station inequalities
cannot both hold: their gap is approximately `1e-6 m`.
`avoidanceSafetyGeometry/localFrame` subtracts this guard from a segment's
upper bound. The readmission anchor, rolled out from a slightly different
measured state, has negative terminal speed and crosses that segment boundary
backwards: its last three station values are approximately 240.601094,
240.600531 and 240.599969 m. Selecting a separate segment from each of these
anchor nodes creates the contradictory terminal intervals.

An auxiliary LP that uniformly expands only route intervals needs just
`4.99999994e-7 m` on each side to recover feasibility. This is diagnostic
evidence of the boundary gap; no route bounds were relaxed in an executed
controller. Fresh initial admission at the identical failure state passes
with terminal residual `1.8646e-7` and inequality residual `1.0359e-8`.

### Validation and interpretation

The new code does retain related continuation proposals: 19 straight and
9 arc successful calls use continuation readmission, with no fallback command
in either original trial. At both failure points the old plan is related,
the exact certificate is incompatible with the plant state, and its carried
witness is invalid. The two failures are therefore distinct from the earlier
3.80 s target-collision-plane conflict.

The selected regression suites passed **51/51**, with zero failed or incomplete
cases: `collisionAvoidanceControllerTest` (31), `ltvBicyclePredictionTest` (2),
`controllerDesignExperimentTest` (16), and
`straightCenterlineCruiseScenarioTest` (2). Recorded-state failure replays,
LP feasibility probes, fresh-admission checks, identical pre-detection traces,
zero observation counts, and the terminal route-gap calculation also passed
their diagnostic assertions. This is not a passing whole-repository suite or
a successful high-fidelity avoidance/recovery demonstration.

The next corrections should address terminal equality scaling/acceptance and
consistent route domains when a readmission anchor violates the model's speed
domain. Retest full cruise before drawing conclusions about obstacle avoidance
or recovery. The controller algorithm, solver tolerances, node collision
criterion and physical constraints were not changed during this retest.
