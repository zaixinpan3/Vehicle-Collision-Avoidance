# Straight-scene rerun after finite encounter completion

September 14, 2026. Tested controller revision
`083e6f3768852d7c6acee5b1fa6a8f2f4c38351e`, certificate version 21.
The controller and observer algorithms and default configuration were not
changed during this rerun. The experiment adapter now supports
`UseEstimator=false`, supplying exact states with its current 30 m range
gate to isolate controller admission from estimator uncertainty.

## Outcome

The two crossing trials complete 30 s at cruise. The stationary and oncoming
trials obtain no executable encounter certificate. The oncoming failure also
occurs with exact states, so estimator error is not necessary for this
failure. General avoidance and the every-frame 100 ms requirement remain
unmet. A failed admission stops simulation before applying its command.

| Trial | Executed holds | Result | Maximum frame | Frames over 100 ms |
| --- | ---: | --- | ---: | ---: |
| Crossing, exact states | 300 | Safe certificate, 8.000000007 m/s at 30 s | 268.344 ms | 2 / 301 |
| Crossing, bounded noisy states and target disturbances | 300 | Safe certificate, 8.000074357 m/s at 30 s | 81.084 ms | 0 / 301 |
| Stationary obstacle, exact states | 0 | Admission search expires after 46 attempts | 30,236.718 ms | 1 / 1 |
| Oncoming, exact states, current range gate | 36 | First publication at 3.6 s; 18 admission attempts fail | 3,070.693 ms | 5 / 37 |
| Oncoming, actual NRMM, current range gate | 35 | First publication at 3.5 s; 16 admission attempts fail | 3,415.987 ms | 3 / 36 |

The final command of a completed trial is counted in frame timing but is
not executed. Failure-frame time is also counted. Crossing median frames
are 59.197 and 61.539 ms. Exact crossing has a 268.344 ms first frame and a
103.894 ms maximum successor frame; neither is excluded from acceptance.
The bounded-noise crossing ran later in the same MATLAB session, after code
paths were exercised. Its timing does not establish a cold-start or
worst-case bound, and the difference cannot be attributed to noise.

## Experimental scope and reproduction

MATLAB R2026a Update 3; all new trials use seed **20260914**, 0.1 s held
controls, initial 16-stage horizon, road boundaries at lateral positions
-5 and 5 m, and 0.25 m geometric clearance. Admission can extend the horizon.
The exact-state driver retains its 30 s admission-search budget; the joint
driver and its exact-state comparison retain their 3 s search budget. These
are the existing driver settings, not the real-time acceptance threshold.

The plant independently integrates the issued first-stage affine generator
using `expm` from true state. Continuous safety is the declared-model
certificate; an independent 11-points-per-hold rectangle/road audit provides
sampled checks. This is not nonlinear Fiala/physical-vehicle validation.
Measured online time includes input assembly and control, and actual NRMM
sampling/adapter work when enabled; it excludes offline observer synthesis,
plant integration, geometry auditing and result serialization. Computation
overruns are measured but not applied as actuation delay: an overrun trial
does not demonstrate an executable real-time trajectory.

```matlab
addpath('scripts');
out = '/home/zai/.cache/collisionAvoidance/algorithm-rerun-20260914';
exactCrossing = runExactStateRecursiveFeasibilityScenario( ...
    Scenario='crossing',SampleCount=300,Seed=20260914, ...
    OutputDirectory=fullfile(out,'crossing'));
noisyCrossing = runExactStateRecursiveFeasibilityScenario( ...
    Scenario='crossing',SampleCount=300,Seed=20260914, ...
    EgoErrorBound=[.05;.05;.005;.05;.02;.005], ...
    TargetErrorBound=[.1;.1;.1;.1;.05;.05;.01;.01], ...
    TargetJerkAmplitude=[.1;.1],TargetYawAccelerationAmplitude=.05, ...
    OutputDirectory=fullfile(out,'noisy-crossing'));
stationary = runExactStateRecursiveFeasibilityScenario( ...
    Scenario='stationary',SampleCount=300,Seed=20260914, ...
    OutputDirectory=fullfile(out,'stationary'));
exactOncoming = runDeclaredPlantEstimatorControllerScenario( ...
    SampleCount=300,Seed=20260914,UseEstimator=false, ...
    OutputDirectory=fullfile(out,'matched-exact'));
jointOncoming = runDeclaredPlantEstimatorControllerScenario( ...
    SampleCount=300,Seed=20260914, ...
    OutputDirectory=fullfile(out,'joint'));
```

Execution order was regression tests, exact crossing, exact oncoming, joint
oncoming, stationary, and noisy crossing. No forced solver failure was used.

## Safety and cruise observations

The crossing target starts at `[15;-4]` m with velocity `[0;32]` m/s; ego
reference speed is 8 m/s and the confirmation region is 16 m. Both trials
release the encounter at 0.7 s, issue 301 zero-safety-value certified
commands, use fresh optimization throughout, and never execute the road
terminal law. Minimum sampled separation margins are 9.269858 and
9.276598 m; road margins are 3.800000 and 3.790395 m. The large separation
means this is a mild crossing regression, not evidence of a tight evasive
maneuver. The driver continues supplying target observations outside the
region; the controller confirms their exterior membership.

At 30 s the noisy crossing has lateral error 0.004125 m, heading error
0.00007568 rad, and speed error 0.00007436 m/s. Its target follows the
independently integrated sinusoidal jerk/yaw-acceleration law with angular
frequency 1 rad/s and amplitudes `[0.1;0.1]` m/s^3 and 0.05 rad/s^2. Measurement
boxes are listed in the command above, ordered as `[px,py,psi,vx,vy,r]` and
`[px,py,vx,vy,ax,ay,psi,omega]`. These are synthetic bounded measurements,
not outputs of the NRMM observer. No early recovery criterion is imposed;
finite-run small errors do not prove asymptotic convergence under noise.

The stationary target is at `[15;0]` m, ego speed 8 m/s. No control hold is
executed, so this run reports neither a collision nor successful avoidance.

For the matched oncoming pair, ego and target initial speeds are 10 m/s,
target starts at `[100;0.8]` m and moves in the negative road direction.
The actual NRMM uses 80 Hz samples, maximum integration step 0.0125 s, and
the existing synthetic sensor configuration. Exact-state mode has no
observer, noise, or target state uncertainty, and publishes only inside the
same physical 30 m region. Their closed-loop trajectories differ slightly,
so boundary crossing can differ by one sample. Both fail at first target
publication. Sampled separation margins before failure are 22.950178 and
24.935295 m; those cover only target-free prefixes, not an avoidance maneuver.

## Failure diagnosis

1. **Admission remains the controller bottleneck.** Both oncoming runs end
   with `certificateSearchLimit`; the last attempted safety-value LP reports
   Clarabel status 2 and has no decision. This means no finite witness was
   found within the search, not that collision is inevitable. Failure with
   exact states rules out attributing the entire issue to the observer.
2. **Longer initial horizons are not sufficient.** At each saved failure
   observation, fresh admission was probed with 16, 32, and 48 initial
   stages. Exact states at 32 and 48 stages yielded candidates with
   `positiveSafetyViolation`, which the verifier correctly refused to
   execute. NRMM cases still exhausted the search with infeasible attempted
   subproblems. These probes use the supported solver hook to record stage
   counts and call the full default solver; that hook bypasses working-set
   row generation, so their elapsed times are not production benchmarks.
   They restart admission from the saved observation without a carried plan,
   and are diagnostics rather than additional closed-loop runs. Current
   schedule seeds use cruise/braking longitudinal profiles; the attempted
   convex geometry family is not a global search over all evasive paths.
3. **First-detection uncertainty adds a separate restriction.** At 3.5 s
   the NRMM target velocity box radii are `[29.9963;20.2711]` m/s,
   acceleration radii `[2.35849;2.35849]` m/s^2, and yaw radius pi. With the
   published two-axis jerk bound 0.38305 m/s^3, finite propagation over
   3.2 s gives position radii about `[110.33;80.55]` m without future
   measurement shrinkage. This is a broad admission set, not observed
   estimation divergence. All 36 sampled ego enclosures/premises pass; the
   available target-component audit passes at publication. The audit is
   scoped to its available components and does not prove every future bound.
4. **The 100 ms setting does not govern every admission.** In
   `collisionAvoidanceController`, `model.frameTimer` is installed only when
   a carried candidate exists. New-target admission can therefore consume
   the separate seconds-long search budget. The search checks its budget
   between attempts, not during an individual solve. At joint failure the
   observer/adapter takes 41.329 ms and the controller 3374.403 ms; median
   observer/adapter time is 10.193 ms. Control dominates this failed frame.

The old infinite-support gate is absent. Remaining work is to obtain a
zero-violation finite witness for the actual encounter and initial
information sets, and enforce the end-to-end deadline even at admission.
This rerun implements the comparison driver and records evidence; it does
not claim those controller problems have been repaired.

## Checks and retained artifacts

All **42/42** existing cases pass in `finiteEncounterCompletionTest`,
`boundedTargetMotionTest`, `visibleTargetLifecycleTest`, and
`targetObservationConditioningTest`, executed through MATLAB MCP. This is
a focused suite, not the entire repository. Factory Code Analyzer reports
zero findings in the modified driver. Both branches were exercised in the
oncoming trials. Git diff checks pass.

Original MAT traces, scalar summaries, test CSV/MAT, and admission probe
scripts/results remain under the `out` directory above. `summary.json`
contains the measured numbers. An initial diagnostic collection failed on
MATLAB struct-array assignment; changing its temporary collector to a cell
array allowed all six probes to complete. That interrupted probe is not
counted as a completed trial. Diary files were empty or incomplete in the
MCP session and are not treated as evidence of execution; saved result
objects and tool-returned outputs support this report.
