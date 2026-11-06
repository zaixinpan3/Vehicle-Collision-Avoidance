# Collision Avoidance Research Core

The primary pipeline is a nonlinear dynamic output-feedback controller. It
conditions joint physical hypotheses on raw sensor blocks and optimizes both
an actuator map and a bounded controller-memory update. It does not require a
separately designed state observer.

The implementation uses finite joint scenarios, numerical Fiala-bicycle flows,
and a selectable sparse multiple-shooting solver. The default `sqpRti` backend
executes one QP without nonlinear acceptance. The `ipopt` backend uses cached
CasADi automatic derivatives, nonlinear iterations, shifted primal/dual warm
starts, and nonlinear policy replay before acceptance. Both are research
deployment approximations, **not robust safety certificates**.
The conditional exact-set theorem and constructive braking terminal component
are in [controller theory](controller/CONTROLLER_THEORY.md).

## Run a raw-sensor experiment

MATLAB R2026a and Optimization Toolbox are used by the current implementation.
Run from the repository root:

```matlab
addpath('scripts');
result = runJointOutputFeedbackScenario(Frames=8, ...
    OutputDirectory=string(tempname));
results = runtests('tests/dynamicOutputFeedbackTest.m');
assertSuccess(results)
```

To try the Thompson-style nonlinear solver route, install the CasADi MATLAB
distribution outside the repository and load it once per MATLAB session:

```matlab
setupJointNlpSolver; % Or pass its installation directory / set CASADI_PATH.
cfg = struct('outputFeedback', struct('method', "ipopt", ...
    'nlpMaximumIterations', 50));
result = runJointOutputFeedbackScenario(Frames=60, Speeds=8, ...
    Scenarios=["headOn","intersectionCrossing"], ...
    ControllerConfiguration=cfg, OutputDirectory=string(tempname));
results = runtests('tests/nonlinearOutputFeedbackTest.m');
assertSuccess(results)
```

`nlpJit=true` compiles model derivatives to C during the first solve for each
configuration, horizon size and initial scenario capacity. Posterior pruning reuses the
compiled graph with zero-weight duplicate slots. Prepare kernels before online operation;
compilation is not part of a real-time control budget. `nlpJit=false` avoids
the compiler requirement. The external CasADi distribution must include IPOPT;
the tested installation is CasADi 3.8.2. No solver binaries are stored here.

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
current deployment uses automatic time coverage, one memory coordinate, two
prediction substeps per hold, and one future-noise direction per retained hypothesis.
The scenario driver performs four discarded virtual startup calls by default to warm
initialization and continuation in MATLAB and the solver; `Warmup=false` exposes cold-start latency. Every recorded
RTI call still performs exactly one QP. Startup warmup time is exported separately.
The control deadline is `min(outputFeedback.maximumSolveSeconds, sampleTime)`;
it is a measured whole-call deadline. IPOPT also receives `maximumSolveSeconds`
as its internal wall-time limit, which excludes inference, preparation,
compilation and replay and is not a hard real-time guarantee. NLP metadata
separates these costs and reports convergence, fallback use and deadline misses.

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
numerical seed. Every stage is optimized, including the final hold. The
appended braking policy is only a warm-start seed. IPOPT has no fixed coefficient
step box; physical input, slew, collision and domain constraints remain hard.
Its candidate must pass fresh nonlinear replay over all retained scenarios.
Otherwise the shifted seed is replayed and used only if it passes; if neither
passes, the call returns an error without committing a new command. This gate
does not certify unsampled uncertainty or continuous-time clearance.

On continuation, supply every sensor reading since the previous decision,
including detection flags. The hold must contain an even integer number of
sensor periods. Do not resend initial hypotheses or reset memory. The retained
first memory map processes the new block exactly once. Every raw reading
conditions physical hypotheses, even though policy features use normalized
block means and the detection fraction. An empty posterior, changed contract,
missing reading, input mismatch, or absence of an accepted plan produces
an error without committing a command.

Noise bounds, policy dimension, future noise directions, coefficient bounds,
preview timing and solver settings live in `cfg.outputFeedback`. The retired
`outputFeedback.horizonSteps` field is rejected. By default, `horizonSeconds=[]`
selects a preview directly from the largest retained ego speed divided by
`previewDecelerationMetersPerSecondSquared`, plus `previewSettlingSeconds`.
Coverage is rounded upward to `horizonResolutionSeconds` and then to full
control holds. These defaults are 4 m/s², 0.5 s and 1 s respectively: at 8 m/s
the preview is 3 s (60 holds at 50 ms), and at 25 m/s it is 7 s (140 holds).
This is an engineering preview scale, not a certified Fiala stopping-time or
feasibility bound, and it imposes no terminal braking constraint.

The preview is recomputed from the posterior before each solve. The previous
stage count is retained when it already covers that preview; otherwise the
shifted seed is extended. No successive-horizon optimization search is used.
Set a positive `horizonSeconds` to request explicit time coverage instead.
An insufficient `controller.maximumHorizonSteps` capacity causes an explicit
error instead of truncating the selected coverage. Horizon growth changes the
NLP graph size and can require a new kernel build; neither the solver budget
nor previously measured short-horizon timings establish real-time operation
with these longer windows. The comparator's `controller.horizonSteps` does
not select the joint-controller window. A dropout retains the target
and its collision obligation. Endpoint constraints impose the same clearance
and physical domain conditions as other prediction nodes. There is no
remaining-braking-travel reserve, fixed final braking input, or terminal
invariance claim in this deployment. Target tangential acceleration and sideslip
stay constant: zero, positive and negative acceleration are all supported,
including curved motion.
Target restart after an absorbing stop, residual physical forces, multiple
targets, and hard road constraints remain outside this implementation. No observer sideslip cone
is imposed on this new pipeline; its physical speed, lateral-velocity and yaw
caps are imposed at prediction nodes (linearized in RTI, nonlinear in IPOPT).

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
