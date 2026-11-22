# Collision Avoidance Research Core

The default controller uses fixed raw-measurement consistency updates and a
compact nonlinear MPC problem. It optimizes one physical trajectory and the
steering and longitudinal-force commands. Observer gains and controller-memory
maps are not optimization variables. Every proposed command sequence is
numerically replayed against all retained physical hypotheses before use.

The nonlinear Fiala bicycle model, constant target tangential acceleration and
sideslip, and absorbing target-stop convention are retained. Moving, turning,
and accelerating targets are admitted; targets need not brake or stop.

The mathematical derivation is in
[CONTROLLER_THEORY.md](controller/CONTROLLER_THEORY.md). Finite hypotheses,
numerical integration and sampled checks are deployment approximations. The
implementation does not claim universal uncertainty coverage, continuous-time
certification, a complete physical-feasibility test, or guaranteed deadlines.

## Run an experiment

The tested environment is MATLAB R2026a with external CasADi 3.8.2 and IPOPT.
The default generated-C compiler is Clang at `-O0`; GCC and `cc` are also
selectable. Solver distributions and compiled kernels stay outside the
repository. NRMM gain synthesis uses YALMIP and SeDuMi. The scenario driver
prepares its shared target design before timing controller calls. Load the
external CasADi installation once per MATLAB session:

```matlab
addpath('scripts');
setupJointNlpSolver; % Optional installation path, or CASADI_PATH.
result = runJointOutputFeedbackScenario( ...
    Scenarios=["headOn","intersectionCrossing"], Speeds=8, Frames=160, ...
    OutputDirectory=string(tempname));
```

For the complete encounter suite:

```matlab
names = ["stoppedLead","hardBrakingLead","brakingLead","headOn", ...
    "acceleratingHeadOn","intersectionCrossing","crossing", ...
    "turningCrossing","curvedHeadOn","curvedCrossing"];
result = runJointOutputFeedbackScenario(Scenarios=names, Speeds=[8,15,25], ...
    Frames=160, RandomSeed=20261107, OutputDirectory=string(tempname));
```

The default `outputFeedback.method="ipopt"` uses 24 control knots where
preview length permits. Preview duration follows the current retained speed,
a configured minimum duration, and inherited coverage. Physical commands
still update every 50 ms, with dynamics and collision checks between knots.
The allocated graph supports preview growth without online recompilation.
With 24 knots, CasADi/IPOPT optimizes 48 actuator variables. Each stage
slack equals its positive clearance deficit after algebraic elimination.

`outputFeedback.nlpCompiler` selects `"clang"`, `"gcc"`, or `"cc"`;
`nlpCompilerOptimization` selects levels 0--3. `nlpJit=false` permits execution
without a C compiler. Record these choices when comparing timings.
Compilation and virtual startup warmup are reported separately. The driver
warms initialization and continuation with four discarded virtual holds; no
warmup command is applied to the recorded plant.

`solverTiming` measures accepted NLP solves, including all solve attempts in
an accepted call. `timing` measures full controller calls, including rejected
calls. Both exclude startup warmup and offline plant simulation. Full-call
measurements include inference, preparation, solve and acceptance. The 100 ms
mean-solve target is distinct from the configured 50 ms control period and
from a worst-case deadline. The solver wall-time limit is not a hard interrupt.
Simulation does not inject computation delay into the plant.
The compact controller uses `initializationSolveSeconds` for startup and
encounter admission and `maximumSolveSeconds` for continuation.

## Information and actuation interface

```matlab
[command, predictedInput, prediction, state] = ...
    collisionAvoidanceController(block, [], referencePath, cfg, previousState);
```

`block.kind` is `"jointSensorBlock-v1"`. It contains `time`, `heldInput`
(the preceding `[steeringRadians; signedForceRatio]`) and all consecutive
sensor readings in `samples`:

| Field | Units and meaning |
| --- | --- |
| `time` | Absolute sensor time, s |
| `position` | Position-only GNSS, 2-by-1 m |
| `acceleration` | Gravity-compensated body acceleration, 2-by-1 m/s² |
| `yawRate` | Gyro yaw rate, rad/s |
| `relativePosition` | Radar position in the ego frame, 2-by-1 m |
| `relativeHeading` | Optional perceived target yaw relative to ego, rad. The simulator supplies it on detected stationary targets. |
| `radarAvailable` | Detection flag; absent values are ignored |

Initialize with `previousState=[]`, one boundary reading, and
`block.initialHypotheses`. Each hypothesis contains:

- `ego`: `[x;y;yaw;vx;vy;r]`.
- `target`: `[xT;yT;yawT;VT;AT;betaT;lrT;halfLength;halfWidth;offsetX;offsetY]`,
  or `[]` for a target-free contract.
- `parameters`: persistent positive `[CfScale;CrScale;muFrontScale;muRearScale]`.
- `sensorBias`: persistent `[GNSS(2);IMU(2);gyro;radar(2)]` offsets.

Hypotheses and their correlations persist through conditioning; their count
is not an optimization-trajectory count. The experiment driver starts with an ego-only prior. Avoidance starts after
NRMM has produced a full target estimate. The independent plant supplies
sensor readings and evaluates the executed motion.

On continuation, supply every reading from the preceding hold. The hold must
contain an even integer number of sensor periods. Do not reset initial
information. Acceleration measurements use the preceding actual input. Radar
dropout preserves target obligations and latent motion parameters. An empty
posterior or inconsistent input history is reported explicitly.

## Planning and acceptance

Endpoint control variables interpolate into a complete sequence of held
commands. Constraints apply to those actual inputs and their increments.
Collision checks use the rectangle distance dual at the control boundaries.
A candidate must pass nonlinear replay for every retained member, including
terminal membership and its continuation. CasADi/IPOPT minimizes the
sum of stage deficits under the inherited cap. A shifted plan must pass the same checks on the updated information.

## Tests and comparisons

```matlab
addpath('scripts'); setupJointNlpSolver;
results = runtests({'tests/compactPredictiveControlTest.m', ...
    'tests/rawSensorContractTest.m','tests/nonlinearOutputFeedbackTest.m', ...
    'tests/collisionAvoidanceControllerConfigTest.m','tests/controllerSourceBudgetTest.m'});
assertSuccess(results)
```
