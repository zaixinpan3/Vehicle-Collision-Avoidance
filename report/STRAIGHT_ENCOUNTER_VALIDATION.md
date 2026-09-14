# Straight encounter validation, September 9, 2026

September 10 follow-up: [the causal diagnosis](../controller/STRAIGHT_SCENE_FEASIBILITY_DIAGNOSIS.md)
isolates contradictory tire-force rows, disturbance/polygon incompatibility,
an independently impossible exit deadline, and scenario interface defects.
The observations below remain the original September 9 results.

The current straight oncoming PassVeh14DOF experiment does **not** run
successfully. Both the original scene and a direct first-detection scene
reject the first optimization, before any command reaches the plant.
This is an admission failure, not an observed loss of feasibility after a
successful admission. No collision-free physical encounter or target exit
was demonstrated.

The tested controller revision was
`339e1b9d3294dc44426b595d4859b2b36b20481a` (certificate version 13).
MATLAB was R2026a Update 3, `26.1.0.3276743`, on `glnxa64`.
No controller, plant, uncertainty, or safety-constraint implementation was
changed for this experiment.

## Acceptance scope

The required guarantee is conditional on the declared assumptions and a
feasible first encounter optimization: retain feasibility and avoid collision
throughout the encounter, and ensure finite-time exit from perception.
Post-exit indefinite driving is outside this requirement. The legacy physical
harness also evaluates cruise recovery, but neither run reached the initial
admission gate, so those additional criteria do not explain these failures.

## Physical experiment results

Both runs used exact current observations, a straight road, ego and oncoming
target speeds of 10 m/s, target lateral offset 0.8 m, target size 5 by 2 m,
and a circular perception radius of 30 m. Right/left road offsets were
6/8 m with a 2.6 m shoulder. The control period was 0.1 s and prediction
length was 16 intervals (1.6 s). The unchanged
`finiteSensingValidationConfig` residual allowance was
`[0.2; 0.06; 0.02; 2.5; 5; 4]` in the derivative units of
`[station; lateral position; heading error; longitudinal velocity;
lateral velocity; yaw rate]`. The scenario maps vehicle and tire parameters
to its initialized PassVeh14DOF plant.

| Measurement | Original scene | First-detection scene |
| --- | ---: | ---: |
| Initial longitudinal target distance (m) | 100 | 29.9 |
| Target visible at first attempt | No | Yes |
| Requested duration (s) | 30 | 10 |
| Attempted control frames | 1 | 1 |
| Applied control intervals | 0 | 0 |
| Failure time (s) | 0 | 0 |
| Complete attempted frame (ms) | 322.423 | 365.528 |
| Enforce 100 ms computation deadline | Yes | No |
| First optimization certified | No | No |
| Target exit demonstrated | No | No |

Both failures were `collisionAvoidanceController:noCertifiedContinuation`:

```text
The joint hard program supplied no checked witness: hard margin:
solver exit flag -2: Clarabel status 2; decision.
```

The original scene failed before target acquisition. The second scene starts
with center distance approximately 29.911 m, inside perception, and disables
the runtime deadline solely to isolate feasibility. It still fails admission.
There are no integrated plant samples for a controlled minimum separation,
passing time, or exit time; the harness reports these as NaN. The original
sequential entry did not start the NRMM/controller experiment because the
controller-only gate failed. Its seed was 20260907, but no stochastic joint
experiment ran. These two failed-frame timing measurements are not a runtime
distribution or a successful real-time qualification.

## Independent admission check

The original scene's saved failure state, road geometry, targets (empty), and
plant-mapped configuration were replayed with the public solver hook. The hook
saved the hard-margin LP and delegated unchanged to `program.defaultSolver()`.
MATLAB `linprog`, with inactive performance-slack columns removed exactly as
in the native solve, also returned exit flag -2 and reported no feasible
solution. This corroborates the native solver result on the actual formulated
problem; it does not establish that every physically possible maneuver is
infeasible. Individual responsible constraint families were not isolated.
A search for exactly opposite physical coefficient rows found no negative
bound-sum contradiction; this limited check does not explain the LP failure.

The physical residual allowance is empirical, and its full-horizon robust
propagation is materially different from the retired short-prefix controller.
These runs do not identify a validated replacement allowance or a successful
new prediction length. Reducing uncertainty or removing terminal exit rows
would change the experiment and was not used to manufacture a pass.

## Declared-model comparison

`runEncounterCertificateScenario(ForceSolverFailure=false)` completed its
existing crossing encounter on the straight reference. Ego speed was 8 m/s;
the target started at `[15; -4]` m with velocity `[0; 32]` m/s; perception
radius was 16 m. It used a 0.1 s period, 16-interval deadline, deterministic
nonzero residual rates `[1e-3; 1e-4; 1e-5; 1e-3; 1e-4; 1e-5]`, independent
matrix-exponential state integration, and partial observations after admission.

| Measurement | Result |
| --- | ---: |
| Applied intervals | 5 |
| Verified exit time (s) | 0.5 |
| Original deadline (s) | 1.6 |
| Minimum sampled rectangle distance (m) | 9.5196140964 |
| Required clearance (m) | 0.25 |
| Maximum sampled CLF residual | -3.6541802451e-5 |
| Maximum endpoint enclosure violation | -7.2465857425e-6 |
| Final certified margin | 0.1744993291 |
| Reported solver fallback count | 0 |

The script's clearance, CLF, and enclosure assertions passed. This verifies
one admitted encounter under the declared affine inclusion. Its fast
transverse target and small residuals differ from the oncoming physical
experiment; it is not evidence that the physical straight scenario works.
Sampled distances alone do not constitute a continuous-time proof.

## Reproduction

From the repository root in MATLAB:

```matlab
addpath('scripts', 'config', 'controller');
report = runStraightRealtimeValidation(Duration=30, Progress=true);

cfg = finiteSensingValidationConfig();
cfg.controller.sampleTime = 0.1;
cfg.controller.horizonSteps = 16;
acquisition = runOncomingVehicleAvoidanceScenario( ...
    Duration=10, CenterlineLengthAfter=1000, ...
    ReferenceSpeed=10, TargetSpeed=10, ...
    TargetInitialLongitudinalDistance=29.9, ...
    TargetLateralOffset=0.8, TargetWidth=2, ...
    UseStateEstimator=false, ControllerConfiguration=cfg, ...
    EnforceRuntimeDeadline=false, Plot=false, Report=true);

diagnostic = runEncounterCertificateScenario(ForceSolverFailure=false);
```

Raw MAT results, the command-window log, machine-readable summary, captured
LP, independent solver result, and capture helper are preserved as experiment
exports in the shared external archive. No full repository test suite was
rerun and no successful physical trajectory or plot was generated.
