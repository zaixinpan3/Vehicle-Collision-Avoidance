# Research entry points

## PCBF / CLF controller

Run the current controller tests, MATLAB code analysis and independent
nonlinear closed-loop replays from the repository root:

```matlab
addpath('scripts');
validateNonlinearPredictiveController('report/new-validation-directory');
```

Then independently audit the exported rectangles and road margins:

```bash
python3 scripts/auditJointPredictiveSafety.py report/new-validation-directory --output report/new-validation-directory/independent-audit.json
```

The deterministic fixtures cover lane recovery, circular lane following,
oncoming avoidance and a turning target. Every issued control is replayed
with tight `ode45` tolerances at 31 samples per hold on the nonlinear bicycle
model itself; no Simulink plant is used. This offline audit reports sampled
safety and timing; it does not authorize online execution.
The controller keeps every ego rectangle corner inside the road
`lateralClearance = [right; left]` at every predicted node; the fixtures use a
four-lane road, `[8.5344; 12.192]` m. Exports state
`roadConstraintsEnforced=true` and record the measured road margin and any
road departure. `auditJointPredictiveSafety.py` still checks a fixed 4-m
half-width that predates this road; its road verdict does not describe the
MATLAB road rows. Historical exports retain their original road-constraint
interpretation. The given path supplies the cruise and recovery reference.

The collision-threat encounters are defined in `collisionThreatScenario`
(seven geometries; `givenPathCollisionBaseline` confirms that unmodified
cruise collides). `runCollisionThreatValidation` runs them with exact states
for a fixed number of frames. `runPotentialFieldCampaign` runs them at 8 and
15 m/s until cruise recovery, with exact states or, with
`EstimatorConfiguration`, with the NRMM estimator and synthetic noisy sensors
in the loop; `extendPotentialFieldRecovery` resumes unfinished runs.
`replayEstimatorValidity` audits the estimator's published bounds against the
recorded truth of such a campaign (it replays seed 20261003).
`synthesizeClfMatrices` writes `config/clfMatrices.json` and must be run before
an experiment at a new operating point.

`runNonlinearPredictiveSafetyValidation` also accepts `Scenarios`, `Frames`,
`ControllerConfiguration`, and `OutputFile` for individual runs. Additional
fixtures `acceleratingTarget`, `acceleratingTurn`, and `brakingTarget` exercise
constant tangential acceleration with constant sideslip. The braking fixture
passes through zero signed velocity at 2 s. Each export records the initial
target state and parameters for independent reconstruction; the audit's
`--files` option selects individual JSON exports. Historical exports without
the revised target state require the audit source from their recorded commit.
The optimizer builds one affine model and solves PCBF slack first, then CLF
slack subject to the attained PCBF cap. It returns both zero-slack and
positive-slack results without nonlinear admission or correction. The previous
trajectory only supplies the next linearization. Returned prediction states
are affine states; independent replay determines measured physical clearance. See
[the controller architecture](../controller/PCBF_CLF_ARCHITECTURE.md).

`prepareCollisionAvoidanceController` warms MATLAB and the optimizer using a
discarded call. `prepareCollisionAvoidancePipeline` additionally probes the
estimator adapter. Neither helper builds native controller libraries:
`buildPredictiveConicSolver` links the required Clarabel adapter, and
`buildControllerKernels` builds the optional MATLAB Coder kernels.

## Estimator and perception research

- `nrmmEstimatorControllerAdapter`, `nrmmTargetMeasurement` and
  `nrmmTargetTruth` provide the estimator scenario interface. The adapter
  drives `onlineNrmmTrackingRuntime` at 80 Hz on bounded-noise sensors
  synthesized from the true path, acquires a target after 20 consecutive
  detections, publishes it only while detected, and passes the controller's
  held input to the estimator.
- `runObserverComparisonScenario`, `runObserverComparisonCampaign`,
  `runSharmaFairnessAudit`, `runSharmaScenarioTunedAudit` and
  `runNrmmOracleTargetComparison` compare the estimator with the reproduced
  Sharma et al. (2026) observer on prescribed kinematic motions (no vehicle
  dynamics, no controller).
- `runOnlineNrmmComplexManeuverScenario`, `runOnlineNrmmTrackingErrorBenchmark`
  and `runNrmmPositionBoundBenchmark` are estimator research drivers.
- `buildNrmmObserverKernel` and `auditNrmmTruthEnclosure` support the estimator's
  independent native kernel and bound audits.
- `fitPerceivedRoadBoundaries`, `evaluatePerceivedRoadBoundarySafety` and
  `quadraticRoadBoundaryRectangleMargin` support offline curb experiments.
  `fitPerceivedRoadBoundaries` fits quadratics to curb polylines offset from
  the centerline, not to LiDAR curb detections. Fitted curb segments are not
  used by the current controller.

Retired controller modes, formal controller admission and native benchmark
scripts have been removed. Their dated reports remain historical results;
use Git history for the source associated with those reports. New reports
and numerical results belong in `report/`.
