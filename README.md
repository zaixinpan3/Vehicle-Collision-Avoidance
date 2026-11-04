# Collision Avoidance Research Core

The primary pipeline is a nonlinear dynamic output-feedback controller. It
conditions joint physical hypotheses on raw sensor blocks and optimizes both
an actuator map and a bounded controller-memory update. It does not require a
separately designed state observer.

The implementation uses finite joint scenarios, numerical Fiala-bicycle flows,
and SQP. Every issued plan passes a nonlinear check of those scenarios. This
is a research deployment approximation, **not a robust safety certificate**.
The conditional exact-set theorem and constructive braking terminal component
are in [controller theory](controller/CONTROLLER_THEORY.md).

## Run a raw-sensor experiment

MATLAB R2026a and Optimization Toolbox are used by the current implementation.
Run from the repository root:

```matlab
addpath('scripts');
cfg = struct('controller', struct('horizonSteps', 4));
result = runJointOutputFeedbackScenario(Frames=8, ...
    ControllerConfiguration=cfg, OutputDirectory=string(tempname));
results = runtests('tests/dynamicOutputFeedbackTest.m');
assertSuccess(results)
```

This reproducible experiment uses seed `20261104`, noisy position-only GNSS,
body-frame inertial acceleration, gyro and relative radar. Its initial finite
physical contract is declared once and includes the actual simulation initial
member. It is not a sensor-derived uncertainty enclosure. Subsequent controller
calls receive only raw sensor blocks, the preceding applied command, the
reference path, and retained controller state. An independent `ode45` plant
provides the replay and empirical clearance checks.

`runCollisionThreatValidation` and `runNonlinearControllerExperiment` now use
the joint raw-sensor pipeline. The former defaults to the 25 m/s stopped/braking
lead challenge; successful initialization or recovery is not assumed. The
current finite-difference SQP implementation is substantially slower than the
control hold in measured tests. Budgets are checked between operations and
cannot interrupt a running QP. See the [validation report](report/UNIFIED_OUTPUT_FEEDBACK_20261104.tex)
for actual outcomes and timings.

## Controller interface

```matlab
[command, predictedInput, prediction, state] = ...
    collisionAvoidanceController(block, [], referencePath, cfg, previousState);
```

`block.kind` is `"jointSensorBlock-v1"`. It contains `time`, `heldInput` (the
preceding `[steeringRadians; signedForceRatio]`), and a struct array `samples`.
Each reading contains:

| Field | Meaning and units |
| --- | --- |
| `time` | Absolute sensor time, s |
| `position` | GNSS inertial position, 2-by-1 m |
| `acceleration` | Gravity-compensated body inertial acceleration, 2-by-1 m/s² |
| `yawRate` | Gyro yaw rate, rad/s |
| `relativePosition` | Radar position in the ego frame, 2-by-1 m |
| `radarAvailable` | Logical detection flag; unavailable radar values are masked |

Initialize with explicit `previousState=[]`, one current-boundary reading,
and `block.initialHypotheses`. Each hypothesis contains `ego` as
`[x;y;yaw;vx;vy;r]`, `target` as
`[xT;yT;yawT;VT;AT;betaT;lrT;halfLength;halfWidth;offsetX;offsetY]` (or `[]`),
`parameters` as positive `[CfScale;CrScale;muFrontScale;muRearScale]`, and
`sensorBias` in `[GNSS(2);IMU(2);gyro;radar(2)]` order. The latter two fields
are persistent and correlated with their physical hypothesis.

Optional `initialMemory` lies in the configured cube. Optional
`initialPolicies` supplies one complete `(v,K,A,B,c)` structure per hold as a
numerical seed. The final hold is always the constant braking backup; every
seed is checked before execution.

On continuation, supply every sensor reading since the previous decision,
including detection flags. The hold must contain an even integer number of
sensor periods. Do not resend initial hypotheses or reset memory. The retained
first memory map processes the new block exactly once. Every raw reading
conditions physical hypotheses, even though policy features use normalized
block means and the detection fraction. An empty posterior, changed contract,
missing reading, input mismatch, or unavailable hard-feasible plan produces
an error without committing a command.

Noise bounds, policy dimension, future noise directions, coefficient bounds,
and solver settings live in `cfg.outputFeedback`. A dropout retains the target
and its collision obligation. The current terminal component supports absorbed
targets or targets that maintain constant negative acceleration until absorption.
It does not cover continuing target motion, target restart, residual physical
forces, multiple targets, or hard road constraints. No observer sideslip cone
is imposed on this new pipeline; its physical speed, lateral-velocity and yaw
caps are checked separately.

## Research comparators and layout

The state-estimate overload of `collisionAvoidanceController` and the NRMM
observer remain explicit comparison interfaces. Use
`runCollisionThreatValidation(Pipeline="nrmmComparator", ...)` for the original
separate-observer campaign. Their results do not validate the new pipeline.

- `controller/`: entry point, shared Fiala physics and geometry, joint
  inference/SQP in `solvePredictiveControl`, terminal certificates, and theory.
- `config/`: validated configuration entry points; CLF tables for comparators.
- `estimator/`: independently executable observer/gain research and theory.
- `perception/`: Python YOLO + LiDAR target-position pipeline.
- `scripts/`: sensor/replay drivers, experiments, comparators, and audits.
- `tests/`: behavior-focused MATLAB tests.
- `report/`: measured outcomes, validation data, and implementation findings.

Run the full MATLAB suite from the repository root:

```bash
matlab -batch "results = runtests('tests'); assertSuccess(results)"
```
