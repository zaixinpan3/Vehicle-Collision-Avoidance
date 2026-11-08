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
repository. Load the external installation once per MATLAB session:

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

The default `outputFeedback.method="compactNlp"` uses 24 control knots where
preview length permits. Preview duration follows the current retained speed,
a configured minimum duration, and inherited coverage. Physical commands
still update every 50 ms, with dynamics and collision checks between knots.
The allocated graph supports preview growth without online recompilation.
At 24 knots the problem has 192 physical state/input variables and one
restoration slack, independent of the number of retained information members.
State and actuator-rate limits use native two-sided constraint rows. At the
tested 24-knot capacities this reduces the allocated constraint expressions
from 1,184--1,824 to 788--1,188 without changing their feasible set. Candidate
validation supplies the same shooting-grid states used to score that candidate,
so they are not integrated twice. Refined validation skips the unused capacity
beyond the actual preview; its integration grid and clearance checks are unchanged.

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
`maximumSolveSeconds` limits one solver attempt. Additional attempts are
allowed only when no checked plan is available, within
`initializationSolveSeconds` or `recoverySolveSeconds`; their time is included.

The 2026-11-07 baseline run used the command above, MATLAB R2026a, CasADi
3.8.2, Clang `-O0`, and an AMD Ryzen 7 7800X3D. All 30 combinations completed
160 holds (8 simulated seconds) each. The independent `ode45` plant checked
41 poses per hold; the smallest measured footprint clearance was 0.537 m
against a 0.1 m requirement. These are finite simulation results, not a proof
of indefinite safety or hardware deadlines.

| Initial/reference speed | Completed cases | Mean NLP solve | Mean whole call |
| --- | ---: | ---: | ---: |
| 8 m/s | 10/10 | 53.3 ms | 60.1 ms |
| 15 m/s | 10/10 | 56.4 ms | 65.8 ms |
| 25 m/s | 10/10 | 70.6 ms | 81.6 ms |

Startup compilation and four virtual warmup holds were excluded; the recorded
initialization call and all its attempts were included. No recorded call
rebuilt the kernel. The largest full call was 304.4 ms, and 3,827 of 4,800
calls exceeded the configured 50 ms hold. The average 100 ms target was met
in this run; a 50 ms deadline guarantee was not.

Compared with the preceding implementation (`23e0405`) under the same
campaign settings, mean validation time fell from 7.55 to 5.55 ms, mean NLP
time from 62.37 to 60.10 ms, and mean whole-call time from 73.42 to 69.16 ms.
These are separate closed-loop runs: the solver has a wall-time budget and
can take different iterations and commands after reformulation.

An isolated comparison on the same three-member initial problem at 8 m/s,
with an 80 m stopped target and exactly three IPOPT iterations, alternated
12 measured calls per version after warmup. Mean NLP time fell from 40.99 to
37.57 ms. Separately, one-member refined replay at 24, 60 and 120 actual holds
in a 124-hold allocation returned bitwise identical validation outputs;
warm evaluation times fell from 12.71/13.20/13.82 to 4.60/8.60/13.25 ms.

The additional stress run used the same ten encounters at 3 and 30 m/s,
320 holds per case, friction coefficients 0.65, doubled sensor-noise bounds,
five initial members, pose offsets and consecutive radar dropouts:

```matlab
cfg = struct('tire',struct('frictionCoefficient',[.65;.65]), ...
    'outputFeedback',struct('noiseBounds',[.08;.06;.003;.08]));
stress = runJointOutputFeedbackScenario(Scenarios=names, Speeds=[3,30], ...
    Frames=320, RandomSeed=20261108, ControllerConfiguration=cfg, ...
    HypothesisOffsets=[-.01,-.005,0,.005,.01], ...
    EgoPositionOffsetMeters=[0;.3], EgoYawOffsetRadians=.02, ...
    TargetPositionOffsetMeters=[-1;.2], ...
    DropoutHolds=[5:12,45:52,100:110], OutputDirectory=string(tempname));
```

Fifteen of twenty cases completed, including all ten at 30 m/s. Their 4,800
executed holds had minimum measured clearance 0.142 m, mean NLP solve 62.2 ms
and mean full call 76.4 ms. Every completed case had mean NLP solve below
100 ms (largest 72.5 ms), and mean full calls were also below 100 ms (largest
95.6 ms). The largest accepted full call was 1.661 s. Relative to `23e0405`,
mean validation time fell from 13.49 to 10.62 ms and mean full-call time from
83.19 to 76.37 ms; the former maximum case mean was 102.1 ms.

At 3 m/s, `headOn`, `crossing`, `turningCrossing`, `curvedHeadOn` and
`curvedCrossing` were rejected before the first command. They are failures,
not collision-avoidance successes or physical-infeasibility certificates.
Including their rejected calls gives a 79.4 ms aggregate full-call mean and
a 3.122 s maximum. Earlier offline searches with one control knot per hold and a
larger solve budget also found no plan; this is not a completeness proof.
Thus the requested success for every physically feasible encounter remains
unestablished. Across both campaigns, 45 of 50 cases completed and 9,600
executed holds respected the sampled 0.1 m clearance requirement.

A diagnostic replay of the five rejected initial states separated input,
physical-state and geometric violations. The best candidates passed the
input, speed and yaw-rate checks, but failed clearance within the first
0.8--1.24 s of their 3 s preview. Their worst retained-member projection gaps
were as follows; negative gaps mean overlapping rectangles. These are
candidate-specific results, not bounds on every possible control policy.

| Rejected case at 3 m/s | Minimum signed projection gap | Time of minimum |
| --- | ---: | ---: |
| `headOn` | 0.00244 m | 1.240 s |
| `crossing` | -0.26022 m | 1.070 s |
| `turningCrossing` | -0.45950 m | 0.894 s |
| `curvedHeadOn` | -0.07059 m | 1.240 s |
| `curvedCrossing` | -0.42089 m | 0.798 s |

Independent `ode45` replay of the same commands and member parameters agreed
with these gaps within 2e-10 m. Refinement used 64 substeps per hold; the
interval allowance added approximately 15--16 mm at the worst check. The
100 mm clearance requirement already rejected each candidate before that
allowance. The nominal member also violated clearance in all five cases.
Initialization has no previously checked plan to retain, so the
search budget expires without an executable plan.

The stress geometry is consequential. In the straight crossing case, the
ego front has only 0.43148 m before the target's crossing corridor, whereas
straight full braking travels 0.70104 m in the nominal physical model.
Controlled first-frame comparisons retained the other stress settings and
the same sensor-noise realization: moving the target 1 m farther in the
world longitudinal direction admitted all five cases; restoring friction
from 0.65 to 0.85 admitted the two opposing-traffic cases but not the three
crossings. Restoring only ego lateral position and yaw admitted none.
These comparisons establish sensitivity of admission, not closed-loop safety
for the modified cases.

Longer local solves (up to 500 iterations per uninterrupted attempt, with
at least 5 s accumulated search when still rejected) did not resolve the
original cases, including when the 0.20 m intersample planning margin and
0.05 m planning buffer were removed. A search of 3,801 steering/force
schedules per case also found no checked witness. These finite searches do
not distinguish all locally missed solutions from physical infeasibility;
the five cases remain recorded as failures, with no scenario exclusion or
physical-infeasibility claim.

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
| `radarAvailable` | Detection flag; absent values are ignored |

Initialize with `previousState=[]`, one boundary reading, and
`block.initialHypotheses`. Each hypothesis contains:

- `ego`: `[x;y;yaw;vx;vy;r]`.
- `target`: `[xT;yT;yawT;VT;AT;betaT;lrT;halfLength;halfWidth;offsetX;offsetY]`,
  or `[]` for a target-free contract.
- `parameters`: persistent positive `[CfScale;CrScale;muFrontScale;muRearScale]`.
- `sensorBias`: persistent `[GNSS(2);IMU(2);gyro;radar(2)]` offsets.

Hypotheses and their correlations persist through conditioning; their count
is not an optimization-trajectory count. The experiment driver declares its
finite initial contract once, including the actual initial simulated member.
It is not a sensor-derived uncertainty enclosure. Subsequent calls receive
raw measurements, actual applied-input history and retained controller state;
they receive no further plant truth. The independent `ode45` plant is used
for sensor synthesis and evaluation.

On continuation, supply every reading from the preceding hold. The hold must
contain an even integer number of sensor periods. Do not reset initial
information. Acceleration measurements use the preceding actual input. Radar
dropout preserves target obligations and latent motion parameters. An empty
posterior or inconsistent input history is reported explicitly.

## Planning and acceptance

Endpoint control variables interpolate into a complete sequence of held
commands. Constraints apply to those actual inputs and their increments.
A smooth conservative combination of the four rectangle separation axes
provides one collision inequality per check, without geometric decision
variables. Physical state constraints and nonlinear shooting equalities
remain hard. There is no forced braking endpoint, remaining-stop-distance
terminal condition, or target-braking admission gate.

Numerical replay checks the candidate on every retained member. IPOPT timeout
or nonconvergence does not authorize a constraint-violating command. The full
fine input sequence of an accepted plan is retained; its shifted continuation
is independently rechecked and can serve as fallback. Refitting onto coarse
knots supplies a warm start and is not assumed to preserve feasibility. The
appended tail must also pass the check. If no checked plan exists, the runtime
reports failure without labeling the encounter physically impossible.
Tail braking repairs and an elastic feasibility-restoration solve supply
additional candidates. Positive restoration slack never relaxes execution
acceptance: the original clearance checks must still pass.
Geometric rejection can trigger a compiled 64-step-per-hold replay with
endpoint speed-bound checks. This reduces the reserve associated with time
spacing while keeping the requested clearance. The speed-bound argument is
conditional on the continuous physical domain; numerical integration and
finite information remain uncertified.

## Tests and comparisons

```matlab
addpath('scripts'); setupJointNlpSolver;
results = runtests({'tests/compactPredictiveControlTest.m', ...
    'tests/dynamicOutputFeedbackTest.m','tests/nonlinearOutputFeedbackTest.m', ...
    'tests/collisionAvoidanceControllerConfigTest.m','tests/controllerSourceBudgetTest.m'});
assertSuccess(results)
```

The optional `sqpRti` and `ipopt` methods retain the earlier dynamic-memory
formulation as numerical comparators. The separate NRMM observer integration
remains available through `Pipeline="nrmmComparator"`; it is not part of the
compact optimizer. Source files stay in their existing controller,
configuration, estimator and experiment directories. Experiment output goes
to the requested directory; routine runs do not create repository reports.
