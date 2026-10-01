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
The controller currently excludes road boundaries. New exports state
`roadConstraintsEnforced=false`; measured road departures remain diagnostics
and do not fail the avoidance audit. Historical exports retain their original
road-constraint interpretation. The given path still supplies the cruise and
recovery reference.

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

Retired controller modes, formal controller admission and native benchmark
scripts have been removed. Their dated reports remain historical results;
use Git history for the source associated with those reports. New reports
and numerical results belong in `report/`.
