# Research entry points

The controller and observer share one continuous-time derivation in
[ALGORITHM.md](../ALGORITHM.md). Fiala forces belong to
`nonlinearBicycleModel` and lane charts to `predictiveSafetyGeometry`.
The observer’s target equations, Jacobian enclosure and position/IMU filter are actions
of `nrmmObserverVectorField`; orientation sets use `nrmmSet.yaw`, and retained
motion-parameter covers use `nrmmTargetHistory("constantMotion",...)`.

## Primary raw-sensor predictive controller

`runJointOutputFeedbackScenario` runs the raw-sensor PCBF controller with
CasADi and IPOPT. It optimizes actuator commands at control knots, propagates
the nonlinear model, and checks clearance at control-period boundaries for
all retained hypotheses. It stores the accepted plan for the next frame.

Load external CasADi with `setupJointNlpSolver` before execution. See the
[setup, scenario campaign, interface and timing definitions](../README.md).

## Estimator and perception research

- `nrmmEstimatorControllerAdapter` provides the estimator scenario
  interface. It drives `onlineNrmmTrackingRuntime` at 80 Hz on bounded-noise sensors
  synthesized from the true path, acquires a target after 20 consecutive
  detections, publishes it only while detected, and passes the controller's
  held input to the estimator.
- `runObserverComparisonScenario` and `runObserverComparisonCampaign`
  compare the estimator with the reproduced
  Sharma et al. (2026) observer on prescribed kinematic motions (no vehicle
  dynamics, no controller).
- `runOnlineNrmmComplexManeuverScenario` and `runOnlineNrmmTrackingErrorBenchmark`
  are estimator research drivers.
- `buildNrmmObserverKernel` and `auditNrmmTruthEnclosure` support the estimator's
  independent native kernel and bound audits.
- `fitPerceivedRoadBoundaries`, `evaluatePerceivedRoadBoundarySafety` and
  `quadraticRoadBoundaryRectangleMargin` support offline curb experiments.
  `fitPerceivedRoadBoundaries` fits quadratics to curb polylines offset from
  the centerline, not to LiDAR curb detections. Fitted curb segments are not
  used by the current controller.
