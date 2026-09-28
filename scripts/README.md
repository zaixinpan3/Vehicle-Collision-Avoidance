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
with tight `ode45` tolerances at 31 samples per hold. This offline audit
reports sampled safety and timing; it does not authorize online execution.

`runNonlinearPredictiveSafetyValidation` also accepts `Scenarios`, `Frames`,
and `OutputFile` for individual runs. The optimizer
executes both zero-slack and positive-slack results. The previous trajectory
only initializes the new solve. See
[the controller architecture](../controller/PCBF_CLF_ARCHITECTURE.md).

`prepareCollisionAvoidanceController` warms MATLAB and the optimizer using a
discarded call. `prepareCollisionAvoidancePipeline` additionally probes the
estimator adapter. Neither helper builds native controller libraries.

## Estimator and perception research

- `nrmmEstimatorControllerAdapter`, `nrmmTargetMeasurement` and
  `nrmmTargetTruth` provide the estimator scenario interface.
- `runOnlineNrmmComplexManeuverScenario`, `runOnlineNrmmTrackingErrorBenchmark`
  and `runNrmmPositionBoundBenchmark` are estimator research drivers.
- `buildNrmmObserverKernel` and `auditNrmmTruthEnclosure` support the estimator's
  independent native kernel and bound audits.
- `fitPerceivedRoadBoundaries`, `evaluatePerceivedRoadBoundarySafety` and
  `quadraticRoadBoundaryRectangleMargin` support offline curb experiments.
  Fitted curb segments are not global corridors for the current controller.

Retired affine MPC, formal controller admission and native controller benchmark
scripts have been removed. Their dated reports remain historical results;
use Git history for the source associated with those reports. New reports
and numerical results belong in `report/`.
