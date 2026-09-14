# Controller design-requirement experiments


## Finite target-prediction correction (2026-09-07)

The user specifies constant target curvature and tangential acceleration only
during the short encounter. Targets cease to be considered after current radar
visibility is lost. The earlier demand for an infinite-future target occupancy
or road/corridor contract was an extra requirement and is withdrawn.

The controller now propagates current target errors through a finite family of
frozen-parameter predictions and imposes finite rectangle constraints through
the last head/tail node. No infinite ray/orbit support is computed. Every newly
appended target node is rebuilt and checked before a shifted plan can serve as
fallback. The existing visibility/publication policy removes hidden targets.
Finite terminal acceptance is distinguished from invariant terminal safety in
both diagnostics and the experiment evaluator. The objective and ego/tire/load
models are unchanged. See [the target contract](../controller/TARGET_PREDICTION_CONTRACT.md).

Both matched physical joint experiments were actually rerun for a requested
30 s on the straight and 400 m arc, with the same 15 m/s ego, 8 m/s crossing
target, 50 m radar, 0.05 s control, 0.0125 s observer, 24 head and 74 continuation
stages, bounded-uniform sensor radii and seeds 20260906/20260907 described below.
Preparation probes remained disabled. Neither run completed:

| Trial | Completed time / commands | Current outcome |
| --- | --- | --- |
| Straight | 3.10 s / 62 | First target's finite propagated error enclosure leaves no accepted plan in the selected domain |
| 400 m arc | 0.00 s / 0 | Existing initial uncertain polyline-chart rejection |

The straight failure has **zero nonfinite hard bounds**, finite position
bounds at every target node, and an accepted ego dissipative terminal model.
Its initial target velocity, acceleration and yaw-rate radii are
27.902881 m/s per component, 4.868243 m/s^2 per component, and 0.124999 rad/s.
Position-component radii grow from 1.013855 m at the current sample to
61.049819 m at 1 s and 390.714908 m at the 4.9 s forecast endpoint. These are
conservative enclosures, not observed target errors or realized movements.
The intervals discard current-state correlations and do not exploit all
estimator operating-domain intersections.

A diagnostic retaining ego, target position and target heading uncertainty,
while zeroing only target velocity/acceleration/yaw-rate uncertainty, admits
the same failed input. An explicit no-target diagnostic also admits it.
Neither diagnostic is an accepted noisy avoidance experiment, and production
bounds are not changed. The remaining straight issue is the initial motion
state enclosure and its conservative propagation, rather than infinite-time
support. The target is first acquired in the failed call; completed-sample
metrics omit that acquisition.

Validation passes 240 distinct cases across 17 test classes (one selected
radar-lifecycle case in the scenario class), with zero factory Code Analyzer
findings in 12 changed MATLAB files. Tests cover current bounds, finite
propagation, 256 corners in each of two initial boxes, endpoint extension,
visibility removal, objective/force behavior and the evaluator. The deliberately
unsafe new terminal-node regression verifies that a solver failure cannot use
an old fallback whose appended target node is now unsafe. No completed joint
avoidance, eventual cruise convergence or new real-time success is claimed.

The following earlier experiments and metrics are preserved as historical
checkpoints. Their infinite-future target requirement is superseded by this
correction; their actual measured stopping conditions are not rewritten.

## Recovery objective and rollback audit (2026-09-06 clarification)

The required recovery behavior is eventual dissipation to nominal constant-speed
cruise. The user specifies no settling deadline. The 9-10 s and 25-30 s windows,
and their error bands, are observations rather than recovery acceptance criteria.
Exceeding a band during one window does not disprove eventual convergence;
remaining inside a band does not establish it. The recorded 30 s trajectories
retain speed bias and arc oscillations, so exact convergence remains unestablished.

The evaluator now reports `avoidancePass` independently of
`finalWindowWithinCruiseBand`, and sets `convergenceStatus` to
`notEstablishedByFiniteRun`. The driver reports `avoidanceRequirementsMet`
instead of `functionalRequirementsMet`. Configuration uses
`criteria.finalObservationWindow` instead of `criteria.recoveryWindow`.
The controller equations, costs, weights, constraints and execution settings are
unchanged by this evaluation correction. Archived raw trials remain unchanged.

The historical rollback audit finds:

| Change | Verified history and disposition |
| --- | --- |
| Discrete LQR preferred input to accelerate recovery | Added in `dc58ef1fd8724577c24b0ef3776abcfaddc9826b`, then explicitly removed in `2ab787947190dacaff26aeef9841acd9447293f7`. No preferred input or sampled feedback gain remains in the current optimization. No further rollback is needed for this change. |
| Continuous Riccati synthesis of the CLF matrix | Already present before `dc58ef1`. It constructs the existing CLF certificate; it is not the removed discrete LQR input target. Retained. |
| Geometry batching, configured solver tolerances and explicit preparation | Separate runtime work in `dc58ef1`; it does not impose a recovery deadline. Retained under the separate real-time objective. |
| Raw input effort plus squared CLF slack | Restored at the user's request in `de10ae43e35c2d5deb851e94d942bdd53bdb7bdd`. Retained. |
| Road load, modified Fiala tires and signed beta | Incorporated in `7d334ac48e117d80261c3ea3b885cafc39d493df` following the force-model and input revisions. Retained. |

No remaining feedback term or constraint added solely to satisfy the mistaken
9-10 s recovery deadline was found in these commits or the current controller.
Historical recovery pass/fail language below and in the archived experiment
reports describes the retired evaluator, not a user-specified settling deadline.

Validation: all 22 evaluator cases and 8 existing cruise-objective/initialization
cases pass. The four changed MATLAB files have zero factory Code Analyzer
findings. Re-evaluating the two saved 30 s physical trials reports avoidance
acceptance and final-band membership for both, with convergence still
`notEstablishedByFiniteRun`. This is analysis of existing data, not a new plant
simulation or a convergence proof.

## Dissipative terminal integration (2026-09-07)

The uncertain-velocity admission gap is now implemented under the declared
scheduled affine model. See
[the terminal derivation](../controller/TERMINAL_CBF_PROOF.md).
The complete physical NRMM/controller experiment remains **incomplete**.
The user requires eventual nominal constant-speed cruise; no 9-10 s recovery
deadline is introduced by this work.

The terminal extension keeps the existing rest input and reserves the complete
remaining pose excursion while uncertain velocity dissipates. Linear sign
expansion gives hard terminal inequalities with no new decision variables.
Raw input effort and squared CLF slack remain the performance objective.
Finite-node frames now include propagated station uncertainty, and a replaced
frame must admit the complete box. Collinear polyline segments share a chart.
The signed-beta/Fiala and air/rolling-load models are retained.

A separate integration defect was found: roundoff between two representations
of one synchronized sample instant could assign a tiny positive age to a fresh
gyro, increasing its error bound from 0.0015 rad/s to about 0.75 rad/s. The
read-only observer output now uses the already accepted sensor timestamp.
Actual held-sample outputs still charge their age, and off-grid frames remain
rejected. The final straight prefix keeps its gyro bound at 0.0015 rad/s.

| Declared-model continuation measurement | Result |
| --- | ---: |
| Initial state-box vertices | 64 |
| Held inputs / duration | 1,200 / 60 s |
| Successful optimizations | 1 |
| Deliberately failed subsequent optimizations | 1,199 |
| Minimum rectangle clearance | 14.552611 m |
| Maximum enclosure excess (roundoff) | 1.177e-13 |
| Final maximum velocity-component magnitude | 1.155090e-05 |

Initial ego speed is 15 m/s. Box radii are
`[0.04, 0.04, 0.014, 0.388, 0.388, 0.0015]` in
`[m, m, rad, m/s, m/s, rad/s]`. The stationary target is at `[60; 0]` m,
the required clearance is 0.25 m, and the sample period is 0.05 s.
`runRobustVelocityCertificateScenario` executes all vertices under the same
inputs and varying current estimation bounds. The final velocity number is a
finite observation; the separate invariant-set derivation establishes the
conditional affine terminal dissipation. This is not a nonlinear-plant run.

The physical experiments retain the previous requested 30 s straight and
400 m arc paths, 15 m/s cruise, 8 m/s crossing target, 50 m radar, 24 head
stages, 0.05 s controller period and 0.0125 s observer period. Uniform bounded
sensor-noise seeds are 20260906 and 20260907; noise radii remain 0.04 m GNSS
position, 0.05 m/s GNSS velocity, 0.03 m/s^2 IMU, 0.0015 rad/s gyro and
0.04 m radar position. Discarded preparation probes are disabled. The actual
plant is PassVeh14DOF; intermediate synthetic observer samples use the existing
endpoint interpolation harness. No truth state replaces a published estimate.

| Physical joint trial | Completed time | Applied commands | Outcome |
| --- | ---: | ---: | --- |
| Straight, final revision | 3.10 s | 62 | First target publication has infinite complete-future support in all 64 directions |
| 400 m arc | 0.00 s | 0 | Initial uncertain box spans noninvertible polyline charts |

The first target appears at the failed straight call at 3.10 s. Its record is
retained in `failureContext`; completed-sample target metrics omit that failed
call and therefore do not report acquisition. A non-pausing diagnostic confirms
that the ego dissipative certificate is accepted while the target creates
16 nonfinite hard bounds. Explicitly removing both sources of target input
(the optional argument must override the bundled ego record) admits the same
current ego state. This ablation is a diagnosis, not an obstacle-avoidance result.

All six ego errors in the 62 completed straight input records lie inside their
published enclosures. Prefix position/velocity/yaw RMSE are
0.008441 m, 0.010050 m/s and
2.877490e-04 rad. These are pre-acquisition metrics, not full avoidance
or post-obstacle recovery measurements.

There are 153 passing focused MATLAB cases across 12 classes,
including sparse/condensed equivalence, terminal invariance, force/damping
rejection, old exact-state behavior, NRMM bounds and fresh-versus-aged gyro
handling. Factory Code Analyzer reports zero findings in 15 in-scope MATLAB
files. Re-evaluation of the 62 saved physical input records succeeds, with a
maximum command difference of 1.198946e-05; exact
command reproduction is not established. No controller-only replay timing
claim is made. In the unprepared physical prefix, median and maximum observed
controller times are 22.049 and
512.505 ms, with
4 calls over 50 ms. This is not a passed real-time experiment.

The first integration revision stopped straight at 0.05 s because the fixed
station chart was too narrow. Widening charts by the reachable station radius
allowed 3.10 s. The timestamp repair removed the spurious gyro-bound jumps;
it did not supply the missing target-motion premise. All intermediate raw
results and source snapshots are preserved outside the repository.

At this earlier checkpoint, an infinite-future target contract was listed as
remaining work. The finite-encounter correction above withdraws that extra
requirement. Curved-path uncertainty, current target initialization/propagation,
physical-model forcing, low-speed observer validity and head/tail intersample
safety remain separate issues.

## Joint estimator-controller admission experiment (2026-09-06)

Both requested 30 s joint trials stop at the first controller call, at time
zero, with `collisionAvoidanceController:unsupportedCertificateUncertainty`.
No control command is applied and no avoidance interval is simulated. This
is an initial-admission failure, not a completed collision-avoidance or cruise
recovery result. Initializing the PassVeh14DOF template is not counted as a
completed closed-loop interval.

The frozen source is `36aa52b564758cd28c52edcef07ebaf4cbf7f677` plus the
pre-existing tracked stationary-pose admission changes. The straight and
400 m radius crossing-target scenes retain 15 m/s ego speed, 8 m/s target
speed, 50 m sensing, 0.05 s control sampling, 24 head stages, current Fiala/beta
equations and solver settings, and the original road grid extended to station
1000 m. The NRMM observer runs at 0.0125 s (four samples per controller period)
with its current integration settings and 0.5 s synthetic initialization
history. This setup requests eventual nominal cruise without a settling
deadline.

`UseStateEstimator=true` publishes actual NRMM estimates and their current
enclosures. Bounded uniform sensor noise uses GNSS position/velocity bounds
0.04 m / 0.05 m/s, body-acceleration bound 0.03 m/s^2, gyro bound 0.0015 rad/s,
and radar-position bound 0.04 m. Seeds are 20260906 and 20260907. The target
speed prior is 8 m/s; its provisional direction remains the adapter's declared
line-of-sight initialization, not target-truth velocity. Rear-axle geometry is
bound to the loaded plant. No uncertainty bound is zeroed or replaced with a
fixed permissive value.

Discarded synthetic controller-preparation probes are disabled for these
admission attempts so the real first-call rejection and complete input are
returned in `failureContext`. This changes initialization instrumentation,
not the controller or its acceptance rules. These runs do not measure warmed
controller or end-to-end real-time performance.

### Measured initial admission and diagnostic replay

| Quantity | Straight | Arc |
| --- | ---: | ---: |
| Requested closed-loop duration (s) | 30 | 30 |
| Completed closed-loop duration (s) | 0 | 0 |
| Applied commands | 0 | 0 |
| Initial longitudinal-speed estimation error (m/s) | 0.0326242 | 0.0260734 |
| Published body-velocity component radius (m/s) | 0.387876 | 0.388352 |
| Predicted terminal longitudinal-velocity radius (m/s) | 0.316000 | 0.317623 |
| Predicted terminal longitudinal-position radius (m) | 1.94925 | 2.01383 |
| Same position radius after one rest-policy step (m) | 1.96496 | 2.02962 |

At the failed sample all six actual ego errors lie inside their published
current bounds; those bounds are available and timestamped at zero. Neither
scene has acquired or published a target yet. The rejection therefore precedes
the target constraint, QP solve and CLF optimization. It cannot be attributed
to a recovery window, a failed evasive maneuver, or an observed estimator
divergence.

A non-pausing conditional breakpoint replays each exact failed input and
captures the unmodified prediction and terminal certificate. The prediction
contains 98 stages (4.9 s at 0.05 s). Its scheduled terminal speed is zero and
the declared disturbance radius is zero, but the terminal velocity radius is
nonzero and the next uncertainty box is not contained in the terminal box.
Both `stationaryVelocities` and `invariant` are false. In particular, the
roughly 0.316 m/s longitudinal radius is not a floating-point residue that
can be removed by a numerical tolerance.

The current rest certificate requires a stationary-pose uncertainty box with
zero velocity radii. Bringing the nominal prediction to rest does not bring
every possible actual velocity to zero under the same continuation. The
missing capability is a terminal policy and invariant set that accommodate
velocity uncertainty. Removing the rejection or omitting estimator bounds
would change the claimed safety contract rather than validate this joint
controller. Future uncertain target-motion admission remains a separate
obligation that these time-zero failures do not reach.

### Validation and retained evidence

All 66 tests across `nrmmControllerErrorBoundsTest`, `nrmmPositionErrorBoundTest`,
`controllerEstimatorBoundsTest` and `onlineNrmmTrackingRuntimeTest` pass. This
validates the tested interfaces and observer behaviors; it does not turn the
failed joint trials into successful ones. The experiment wrapper has zero
factory Code Analyzer findings. Controller, estimator and configuration
MATLAB source hashes remain unchanged.

The external archive retains the exact source and existing working-tree
patch, both failed MAT trials, first-call estimates and bounds, observer
initialization records, complete logs, non-pausing replay probes and test
results under `2026-09-06_Joint_Estimator_Controller_Admission`. The retained
`runJointAdmission(1)` and `runJointAdmission(2)` wrappers reproduce the two
requests against their sibling `source` snapshot. No joint collision success,
cruise convergence, trajectory RMSE, or successful online timing claim is made.

## Current beta-input validation (2026-09-06)

The signed-beta / modified-Fiala controller at `7d334ac48e117d80261c3ea3b885cafc39d493df` was evaluated with the current tracked working-tree stationary-pose admission changes. Controller equations and settings were held fixed. Two 30 s crossing-target trials and two 10 s nominal counterfactuals use the original scene parameters and road grid extended to station 1000 m. Exact-state control, 24 head steps at 0.05 s, 15 m/s reference, 50 m perception, and solver tolerances 1e-6 are retained.

Each error triple is speed (m/s), lateral position (m), heading (rad); descriptive cruise bands are 0.5, 0.2, 0.02. These bands do not define asymptotic convergence.

| Path | Completed avoidance | 9-10 s maximum errors | 25-30 s maximum errors | Final errors |
| --- | ---: | --- | --- | --- |
| straight | 30 s | 0.727707, 1.3491e-05, 7.04829e-07 | 0.0315122, 3.82211e-11, 4.10433e-11 | -0.0314215, -1.50792e-11, -2.46706e-11 |
| arc | 30 s | 0.727975, 0.140574, 0.0012087 | 0.0802448, 0.0176978, 0.00231731 | -0.0789914, -0.0170936, -0.00162779 |

Both paths exceed the descriptive speed band during part of 9-10 s, while lateral and heading errors remain inside their bands. All errors remain inside their bands throughout 25-30 s. Entry into all bands is observed at 9.5 s (straight) and 9.4 s (arc), without a later exit through 30 s; these times are observations, not deadlines. Both avoidance trials complete 600 commands with no fallback or controller failure; minimum sampled rectangle SAT margins are 0.65006 m and 1.0613 m.

Nonzero speed deficits remain, and the arc retains persistent steering, beta and heading oscillations. No-target nominal final speed errors at 10 s are -0.037117 m/s (straight) and -0.0802322 m/s (arc), so the residual is not specific to avoidance.

Three sequential controller-only replays follow the concurrent physical jobs. All runs use one MATLAB computational thread. Preparation is excluded from timed calls; all measured deadline misses are retained.

| Path | Replay median range (ms) | Worst observed call (ms) | Total calls over 50 ms |
| --- | ---: | ---: | ---: |
| straight | 18.337-18.601 | 33.018 | 0 |
| arc | 21.345-21.521 | 38.346 | 0 |

`benchmarkControllerRuntime` now reports dimensionless `brakingRatio` and physical `accelerationMetersPerSecondSquared` separately, with differences in their own units. The raw second input must not be labeled as acceleration. All 175 existing focused cases and one new report-unit regression pass; the new regression also passes on the isolated commit candidate. Two edited/new MATLAB files have zero factory Code Analyzer findings. No current full-suite passing claim is made.

Raw results, exact run source, hashes, extended-run scripts, independent CSV checks, plots and sequential replay details are retained in the external archive under `2026-09-06_Beta_Controller_Recovery`. These are finite deterministic, sampled-pose experiments. Historical comparison includes multiple model/input changes and is not a single-change ablation; exact asymptotic convergence and hardware worst-case timing are not established.

## Historical experiments before the Fiala revision

The following material records experiments and diagnostics from before the modified Fiala
tire revision. Its linear-tire and friction-limit results describe that earlier
controller. The current model uses scheduled Fiala tangents and no separate
axle-friction constraints; see [LTV_BICYCLE_MODEL.md](../controller/LTV_BICYCLE_MODEL.md).

`runControllerDesignExperiments` compares nominal cruise with range-triggered
avoidance on a straight path and a circular arc. Both trials use the same
certificate-preserving predictive SOCP and the actual Vehicle Dynamics Blockset
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

## Historical acceptance criteria (recovery deadline withdrawn)

The following list records the original evaluator. Item 6 was an inferred
experimental criterion, not the user's requirement, and is withdrawn by the
clarification above. Current evaluation separates avoidance acceptance from
tracking observations and leaves asymptotic convergence unestablished.

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
6. **Retired recovery-window check:** the evaluator formerly required the last
   complete 1 s of the 10 s trial to satisfy the cruise bands after acquisition.
   This is retained only to interpret historical outputs, not as acceptance.
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

## Repairs driven by the recorded plant failures

The follow-up investigation replayed the saved failing calls before running
the complete paired experiments again. It identified three separate issues:

1. Terminal velocity equalities could miss the absolute acceptance threshold
   despite a successful conic solver status. They are now eliminated by an
   exact affine parameterization before the same single SOCP, along with
   fixed terminal inputs. Reconstructed terminal velocities satisfy the
   equalities to roundoff. The sole CLF slack is set to its analytic minimum
   for the returned input vector; actuator and safety variables are unchanged.
2. Restricting a prediction node to its anchor's 0.1 m polyline segment
   obstructed station corrections and could give the stationary endpoint
   disjoint domains. A chart now spans the configured station trust radius.
   Explicit position and heading error bounds cover every intersecting
   segment and tighten collision/road support. The last two resting nodes
   share a chart. Independent clearance checks use the physical polyline pose.
3. Repeated readmission on the Blockset plant shifted the initial braking
   schedule toward zero even while the optimized vehicle kept cruising.
   The first repair supplied shifted optimized states for a refreshed
   schedule, starting at measured speed. The later schedule experiment below
   replaces that refresh with the deterministic measured-speed template.
   An exactly applicable certificate still shifts its original model
   unchanged. The proposed continuation must pass all current checks before
   it can authorize fallback.

With these repairs alone, both nominal trials completed all 200 intervals.
Both crossing trials passed initial cruise and target acquisition but stopped
with `noSolution`: straight at 4.80 s after 96 intervals, arc at 4.85 s after
97 intervals. Their smallest observed SAT gaps were respectively 14.738 m and
14.298 m. These are incomplete avoidance runs, not successful avoidance.

A straight-path replay isolated the later conflict to collision and axle
friction constraints. Removing either family restored LP feasibility; removing
road, route, lateral, heading or speed rows separately did not. The previous
plan postponed braking until future acceleration reached approximately
-7.552 m/s^2 at the axle limit. After one plant step, the refreshed prediction
differed from the old shifted prediction by up to 0.0048 m/s in longitudinal
speed and 0.0282 m in lateral position over the continuation. Its carried
plan violated a collision row by 0.0060 m and a
nondimensional friction row by 0.00138. The relevant old and new collision
normals were both rear-facing, so changing that normal was not the cause.
This diagnoses an absence of correction capacity at a boundary plan; it does
not establish physical unavoidability or nonlinear-plant recursive safety.

Adding integrated head-horizon state tracking to the objective was tested in
an isolated candidate. It passed the controller unit tests but the actual
straight crossing trial still stopped at 4.80 s. That objective change was
not retained. The simulation and acceptance requirements were not weakened.

A second candidate reserved 10% of each axle's load-dependent friction
polygon at future stages, retaining the full physical polygon for the current
input. With Euler integration it still failed: straight stopped at 4.75 s
and arc at 4.85 s, with observed SAT gaps of 15.501 m and 14.352 m. Thus this
reserve alone did not repair the high-fidelity closed loop.

The straight reserve-only failure followed a command of -0.0366 rad steering
and -4.1555 m/s^2 acceleration at 4.70 s. Its next measured lateral velocity
and yaw rate were -0.05906 m/s and -0.07614 rad/s. Euler predicted -0.12850
m/s and -0.11249 rad/s. Integrating that same scheduled affine model exactly
under the held input predicted -0.07434 m/s and -0.08385 rad/s: the respective
absolute errors decreased from 0.06944 to 0.01528 m/s and from 0.03635 to
0.00771 rad/s. This is a local model comparison, not an exact nonlinear-plant
model or a disturbance certificate.

The implemented correction uses this block-exponential discretization at
every head and continuation stage, including acceleration-bias propagation
and the CLF Riccati design. Initial station schedules use the corresponding
constant-acceleration integration. Fixed terminal controls and velocity
equalities retain their common-model interpretation. Exact-model certificates
shift the complete scheduled flow unchanged. A recorded-state replay through
the reserve-only failure at 4.75 s then returned an accepted command of
approximately 0.0019 rad and -6.7573 m/s^2. This replay is distinct from a
completed closed-loop plant experiment.

The complete held-input trials with a 10% future-force reserve nevertheless
stopped at 4.80 s (straight) and 4.85 s (arc). Increasing the reserve to 30%
moved those stops to 4.55 s and 4.70 s. Both variants completed nominal cruise,
but neither completed avoidance. A reserve available to the current optimizer
was consumed by its nominal performance choice; it did not correct the
assumed commanded-to-motion relationship. The 30% variant also rejected the
tight 30 m oncoming continuation regression. An LP sweep of that fixed
initial convex domain was infeasible at fractions 0.50 through 0.75 and
feasible at 0.80 through 1.00, in increments of 0.05. This is a domain
diagnostic, not a proof of physical impossibility at smaller fractions.
The future-force-reserve parameter was removed from the implementation.

At the held-input straight failure, the command was -7.23115 m/s^2 while the
next interval's measured body-speed change averaged -6.08489 m/s^2. The
speed discrepancy was 0.05731 m/s and the shifted continuation's collision
row violation was 0.09372 m. Across six recorded braking intervals from the
failed variants, step-average response/command ratios ranged from 0.84148
to 0.94068. These ratios include the complete plant response, not only an
isolated brake actuator.

The current experiment therefore sets `model.longitudinalInputGain = 0.80`.
The controller's exact-model default is 1.0. The declared affine dynamics use
`vxDot = gain*a + rBar*vy + bias`; the same gain enters the entire prediction,
the cruise reference, CLF synthesis/cache and terminal bias cancellation.
Actuator bounds and the full requested longitudinal-force envelope remain
hard at every stage. Load transfer uses the effective model acceleration.
This fixed modeling choice is motivated by the recorded response; it is not
a proven lower bound on the nonlinear plant or a robust tube certificate.

With gain 0.80 and the optimized-state schedule refresh, both nominal runs
again completed 200 intervals. Avoidance still stopped at 4.80 s and 4.85 s,
with minimum observed SAT gaps of 14.735 m and 14.309 m. At the straight
failure, measured speed was only 0.0104 m/s below the previous prediction;
the last requested acceleration was -0.1003 m/s^2. In contrast, refreshing
the entire speed schedule from the optimized trajectory changed it by up to
1.4399 m/s. The carried plan then violated collision and friction rows by
0.02707 m and 0.01757 respectively. Both retaining the shifted schedule and
using a new measured-speed cruise/brake template restored linear feasibility
at that same measured state. Thus reducing the longitudinal-model discrepancy
alone did not resolve the schedule-induced loss of the convex domain.

The older oncoming and circular-target wrappers also contained a separate
0.1 s guard, incompatible with the controller's current 0.05 s default.
Their checks now require equality of observation and control sample periods.
Their existing tests count the actual 0.05 s intervals. Collision, road,
recovery and command acceptance thresholds are unchanged.

The implemented readmission rule uses the same measured-speed cruise/brake
template as initial admission. The shifted optimized inputs remain the one
geometric proposal, but their unexecuted future speeds do not set the new
model schedule. Exact certificate reuse continues to shift the stored model.
Replaying the recorded gain-0.80 straight failure with this rule produces an
accepted plan in one SOCP, with first input -0.014665 rad and -0.396463 m/s^2
and maximum hard-row violation 2.87e-6 at the unchanged 1e-6 solver tolerance
(the existing acceptance allowance is ten times that tolerance). A separate
two-history regression verifies that differing old optimized futures do not
change the readmission model for identical current observations. These checks
establish the implementation behavior; full plant completion remains a
separate experiment.

### Numerical acceptance and the crossing preference

The deterministic schedule repaired the captured linear infeasibility but
did not yet complete the plant trials. With `coneprog` at the original
1e-6 tolerances, avoidance still stopped at 4.80 s and 4.85 s (candidate V7).
At the straight failure, the linear constraints were feasible and the solver
reported success, but its returned vector violated a hard row by 2.0584e-5
and independent node clearance by 2.0291e-5. Both exceed the unchanged 1e-5
acceptance allowance. A tighter replay reduced these residuals; repeating the
whole plant run with both `coneprog` tolerances at 1e-8 nevertheless failed
at 4.80 s (straight) and 5.00 s (arc), with much longer solve times (V9).

`crossingCruiseReference` now gives the controller an explicit preference to
arrive after a target clears the nominal path corridor. The rule accounts
for rectangle supports and the configured clearance, chooses the first node
after the last forecast occupancy, and adds a 0.25 s performance time gap.
It reduces the cruise reference only when the target exits within the
forecast and the stopping station remains ahead. It performs no optimization
and changes no hard constraint. Oncoming targets without a lateral exit keep
the original cruise reference, leaving evasive steering available. This
behavioral choice makes yielding explicit instead of expecting a local
cruise objective to choose a useful crossing maneuver on a saturated boundary.

The yielding reference alone, with the original `coneprog` backend, produced
numerically rejected candidates at initial acquisition: 3.10 s and 3.15 s
(V8). An exact conversion of the same reduced SOCP to SeDuMi's dual form
improved numerical residuals and execution time. On the captured critical
problem, SeDuMi took 0.5592 s and returned a maximum hard-row violation of
2.23e-9. SeDuMi alone, without the yielding reference, still failed the full
straight trial at 4.80 s and the arc at 5.15 s (V10). These comparisons
support retaining both the explicit performance preference and the backend
change. They do not establish that no other formulation could work.

The selected candidate V11 combines the yielding reference with one SeDuMi
call. Its relative internal precision is 1e-9; the experiment's public
constraint and optimality tolerances remain 1e-6, and independent physical
acceptance remains ten times the constraint tolerance. Fixed-input and
terminal-equality elimination is algebraic. No alternate geometry, second
solve or actuator clipping was added. A final sparse-assembly cleanup returned
exactly the same decision on the recorded critical SOCP (infinity-norm
difference zero).

### Completed paired plant experiments

All four V11 trials ran for 10 s and 200 control intervals using the physical
setup and independent evaluator above. Each avoidance result is paired with
its own target-free nominal run from the same frozen implementation. Both
nominal counterfactuals first overlap the target rectangle at 5.80 s.
No contact forces are simulated in those counterfactuals.

| Measured result | Straight | 400 m left arc |
| --- | ---: | ---: |
| First target acquisition [s] | 3.10 | 3.15 |
| Center range at acquisition [m] | 49.3417 | 49.5434 |
| Minimum avoidance SAT gap [m] | 1.450004 | 1.790249 |
| Minimum analytic outer-road margin [m] | 7.649988 | 7.542667 |
| Maximum hard-row violation | 5.72e-9 | 9.98e-9 |
| Maximum terminal velocity residual | 4.26e-14 | 2.49e-14 |
| Final-second maximum speed error [m/s] | 0.015644 | 0.016217 |
| Final-second maximum lateral error [m] | 0.00001123 | 0.013080 |
| Final-second maximum heading error [rad] | 0.0000005635 | 0.001830 |
| Maximum SOCP calls per sample | 1 | 1 |
| Fallback commands | 0 | 0 |
| Median controller time [s] | 0.207499 | 0.231205 |
| Maximum controller time [s] | 0.952330 | 0.982513 |
| All functional requirements | Pass | Pass |
| Every call within 0.05 s | Fail | Fail |

Both vehicles yield longitudinally, allow the target to cross, and recover
cruise before the final assessment window. Recorded figures include the
actual performance-speed reference alongside requested cruise and measured
speed. All command/friction checks pass within the declared numerical
tolerance. The maximum measured commanded friction utilizations are
1.0000000059 and 1.0000000100; these are model-based command checks, not
measured tire-force certificates.

The experiments ran in MATLAB R2026a Update 3 on a Ryzen 7 7800X3D host, with
concurrent offline processes. They are not isolated real-time benchmarks.
The deadline requirement remains unmet, and the plant harness applies no
latency penalty for controller computation. No collision, road, recovery,
range, duration or actuator requirement was relaxed. The gain 0.80 remains
an empirical model choice, not a nonlinear-plant error enclosure.

### Final regression scope

The final implementation passed 119/119 focused tests, including controller
state/readmission, exact held-input flow, route-chart bounds, crossing
reference behavior, configuration validation, geometry, road fitting and
experiment evaluation. Static analysis reported zero findings across all
24 changed MATLAB files. Declared-model solver-outage continuations also
passed in the original and rotated frames with gain 0.80: minimum rectangle
clearance 0.250000291 m against 0.25 m, and terminal velocity residuals
2.77e-14 and 2.82e-14. These retain a tight oncoming steering case in addition
to the plant crossing examples.

A frozen full-suite run, based on estimator commit
`d6741d2ab6b5fb7cd0d755dc61a34490cf1a836a` plus the controller repairs,
reported 284 passes, six failures and two additional skips among 292 tests.
All six failures are in `estimatedStateAvoidanceScenarioTest`, whose nonzero
ego/model uncertainty is rejected by the declared exact-model certificate.
A feedback tube and robust terminal contract remain necessary for that
integration. The two skipped curb tests require an absent historical recorded
dataset. Their assumptions were not bypassed. The final full suite is not
green; the paired truth-sensing plant experiments and the focused controller
checks have the narrower successful scope stated here.

## Sparse native solver and runtime optimization (2026-09-05)

The subsequent runtime change keeps the selected physical controller and
experiment contract. It introduces explicit future-state variables, sparse
dynamic equalities and a native quadratic-conic solve, using the same local
constraint coefficients for the native problem and independent condensed
acceptance. It also batches geometry and friction assembly and caches native
function handles. It does not shorten the horizon, add another solve, loosen
hard constraints or substitute a prescribed braking-only backup.

Both final 10 s avoidance runs and their nominal counterfactuals complete and
pass the unchanged functional evaluator. Minimum sampled SAT gaps are
1.450020 m (straight) and 1.790228 m (arc). Avoidance medians are
37.336/38.693 ms, 95th percentiles 42.203/42.874 ms, and maxima
119.561/75.958 ms. Five of 200 straight calls and one of 200 arc calls miss
50 ms. First target acquisition is the largest avoidance call in each scene.
The fresh process's initial nominal call takes 585.82 ms. No latency is
introduced into the simulated plant by the harness, so the results still
do not demonstrate real-time operation.

Every call uses one SOCP and zero fallbacks. Almost-solved or numerical
termination occurs in 187/200 straight and 194/200 arc avoidance calls; all
returned input plans pass independent acceptance. Maximum hard-row and
terminal residuals are 4.25e-11 and 5.57e-11. Exact optimality is not claimed.
The combined broad regression results are 298 passes and six existing
estimated-state uncertainty-domain failures among 304 tests. Full methods,
negative condensed-solver findings, controlled recorded-input comparisons,
native build instructions and limitations are documented in
[CONTROLLER_RUNTIME.md](../controller/CONTROLLER_RUNTIME.md).
