# Controller and estimator stress campaign — September 25, 2026

The original 120-case declared-plant sweep improved from **110/120 to
120/120 passing**. A further 120 operating-condition cases finished at
**116/120 passing** after several diagnose–fix–retest cycles. All **835 workspace
unit tests passed**. These results do **not** establish an all-clear: four
short-range admissions, the nonlinear-vehicle scenarios, estimator-coupled
admission, and execution deadlines remain unresolved.

All numbers below are actual runs. Failed scenarios stop when no command can
be certified; positive clearance before that stop does not mean the encounter
completed safely. Controller certificates cover the declared sampled affine
plant. Finite inter-node audits do not establish continuous-time safety.

## Experiment scope and reproducibility

MATLAB R2026a Update 3 (26.1.0.3276743), local native Clarabel solver, and the
repository's existing simulation models were used. The starting revision was
`fe0c8f5`, with existing uncommitted controller changes. Those included the
zero-default clearance field, input-effort objective removal, fresh admission
after inconsistent measurements, and previous-plan direction seeding. They
were present in the baseline and are included as dependencies of the final
controller source. They are not improvements measured by this campaign.

The new entry point is
[`runControllerEstimatorStressCampaign.m`](../scripts/runControllerEstimatorStressCampaign.m).
Its CSV checkpoints preserve individual failures. The exact-state driver now
accepts speed, friction, actuator slew, audit density, and nonthrowing failure
report options. Controller trials use a 30 s search budget and no enforced
frame deadline so numerical/feasibility failures can be distinguished from
timing failures. Measured frame times are retained separately.

| Campaign | Conditions | Final result |
| --- | --- | --- |
| Existing declared-plant sweep | 120 cases: four encounter classes, road curvature, estimation bounds, target motion, initial errors, road edges, horizons and detection ranges; 240 holds each | 120 complete and pass; 28,800 executed holds |
| Expanded operating conditions | 120 cases; speed 2, 5, 12, 18 m/s; friction 0.2, 0.4, 1.0; sample periods 0.02, 0.1, 0.2 s; steering slew 0.3/0.7 rad/s with braking-ratio slew 2/s; 24 noisy trials | 116 complete and pass; four admission failures; 28,080 executed holds |
| Estimator benchmark | 11 conditions; one noiseless run and eight trials per noisy condition, seeds 20260925–20260932 | 81 finite trajectories; truth operating-domain checks pass |
| Sampling/integration | Nine sensor periods from 0.02 to 1 s, each with requested integration step 0.005 s and one full sensor period | 16 finite runs; both unstable 1 s cases rejected at initialization |
| Detection-range diagnostic | Ten oncoming cases at 24/32 m, including low speed and steering slew limits | 10 complete and pass |
| Physical and coupled validation | Two truth-fed nonlinear vehicle runs, two estimator-fed nonlinear runs, one estimator-fed declared-plant run | All five stop on controller admission failures |
| Regression suite | Full `runtests('tests')`, including 12 new regression cases | 835 passed, zero failed/incomplete |

Expanded cases last 12 s and audit 21 evenly spaced points per hold. The
existing declared sweep and range diagnostic use 11 points per hold. No
independent random claim is made for deterministic parameter sweeps. The
estimator oncoming case lasts 8 s by benchmark design; other benchmark cases
last 12 s. Integration diagnostics last 6 s. The unit count includes six
pre-existing untracked Sharma comparison tests: 829 tests are in the committed
scope. Those comparison sources and tests remain deliberately excluded from
this task's commit, as do the external solver tree and generated binaries.

## Defects and implemented changes

### Solver status was insufficient to authorize a command

A fault-injection hook returned a successful native solution, corrupted its
first actuator decision to 50, and retained either the `Solved` or
`AlmostSolved` flag. The original acceptance path trusted the flag. This is a
fault-injection finding, not evidence that the native solver spontaneously
returned that corrupted decision.

`solveHardCbfClf` now independently checks the full lifted equality,
inequality, and second-order-cone residuals. It also checks hard constraints
again in the original plan coordinates, including physical rows, terminal
cones and target separation. The lifted check allows numerical solver error;
the original hard constraints allow only arithmetic roundoff and preserve the
existing inward reserves. Invalid candidates receive exit flag -8. A fresh
frame rejects them; an inherited frame can retain its previously certified
shifted plan under the unchanged transfer contract.

Both injected success statuses are rejected by the new regressions. A separate
test prevents a relaxed direction-search solution from becoming an executable
command when every subsequent hard solve fails.

### Node grazing produced between-node overlap

The baseline completed eight cases with a sampled body overlap despite
positive node gaps. The worst overlap was **0.0406640 m**. The baseline also
had two explicit optimization failures, for 110 passing cases overall.

The default target separation reserve is now
`0.10 * max(1, sampleTime / 0.05)` m. An explicitly supplied value remains
authoritative. The geometric seed includes the same reserve. The initial
0.10 m fix cleared the original suite, but the expanded 0.20 s oncoming run
still overlapped by about 0.0165 m. Scaling the default with hold duration
removed that observed failure on retest.

Final minimum sampled gaps are **0.0743550 m** in the declared sweep and
**0.0871761 m** in the expanded sweep. This empirical engineering reserve is
not a whole-hold certificate, a speed-dependent stopping-distance rule, or a
proof against arbitrary disturbances. See
[`NODE_SAMPLED_CERTIFICATE.md`](../controller/NODE_SAMPLED_CERTIFICATE.md).

### One passing-side fit concealed feasible alternatives

The initializer's lower geometric fit score did not imply that its fixed
separation directions admitted a feasible trajectory. Trying the other fitted
passing side within the same time budget resolved the two curved, ten-fold
uncertainty failures in the original suite.

After both sides and configured feedback strengths fail, fresh conflicting
admissions now permit at most six phase-I direction updates. A common
nonnegative slack is used only to search separation/exit directions; physical
constraints remain hard. Each updated direction set must pass a separate
complete hard solve and both independent feasibility checks. The relaxed
solution has no execution authority. Normal and inherited successful frames
do not enter this exceptional search.

This recovered the straight oncoming case with 0.7 rad/s steering slew at
16 m detection range. It took three phase-I steps, with geometric slacks
approximately 0.6111, 0.2037, and 0.1945 before a hard solution succeeded.
Extending an offline 0.3 rad/s diagnostic to 24 iterations stalled at positive
slack 0.251818; increasing the online iteration limit was therefore not
adopted. Direction search remains incomplete.

### Low speed exposed horizon truncation and ill-conditioned fitting

The initial encounter horizon was silently capped at four performance
windows even when the proposed physical pass required more time. No later
step extended it. The 2 m/s stationary case consequently failed despite a
feasible longer encounter. The controller now retains the proposed completion
time, up to an explicit `maximumHorizonSteps` allocation limit (default 512),
and reports `encounterHorizonLimit` before allocation when that limit is
exceeded. A nearly speed-matched target tests this bounded failure behavior.

The endpoint-constrained reference fit also squared nearly dependent rows in
its Schur complement. A low-speed fit had terminal error 1.326e-8 against a
1e-8 acceptance threshold, with condition number about 1.68e10. Orthonormalizing
the constraint row space before solving, then removing residual projection
drift, reduced that diagnostic to 2.99e-16 and condition number about 2.29e4.
The low-speed stationary cases now complete.

### Coarse explicit integration destabilized a finite estimator trajectory

At a 0.30 s radar period, requesting a 0.30 s RK4 step produced position RMSE
**36,518,528.6 m**, while a 0.005 s step at the same radar period produced
**0.121944 m**. All samples were still finite, showing why a finiteness check
alone was insufficient.

The runtime now limits the requested RK4 step using the observer correction,
rotation, and certified nonlinear rate scales. The maximum remains a user
upper bound, and nominal small-step settings are unchanged. The fixed coarse
request gives **0.121932 m** position RMSE and closely matches fine integration.
The 0.25 s coarse case likewise improves from 24.8379 m to 0.0529598 m.

Sensor sampling and numerical integration are checked separately. For the
admissible straight constant-velocity error case, form the augmented generator
with `e' = N e - L r`, `r' = -L(1) r`, and reset `r(0) = e(1,0)`.
The exact sample map is
`[I,0] * expm(h*G) * [I;1,0,0]`.
A spectral radius at least one now rejects initialization. At a 1 s radar
period the radius is 4.78011, so fine RK4 integration cannot repair the
sampling instability. Both 1 s cases are explicitly rejected. Passing this
necessary nominal check does not establish nonlinear or dropout stability.

## Estimator measurements after the fixes

These are means of per-trial RMSE values; position is relative position in
metres, velocity is m/s, and acceleration is m/s². The noiseless row has one
trial; each other row has eight.

| Condition | Position | Velocity | Acceleration |
| --- | ---: | ---: | ---: |
| Noiseless | 0.000123 | 0.015312 | 0.006451 |
| Bounded sensor noise | 0.028587 | 0.119757 | 0.182038 |
| Varying target motion | 0.029016 | 0.132375 | 0.314053 |
| One-second radar dropout | 0.055198 | 0.184292 | 0.271025 |
| 25 Hz sensing | 0.043460 | 0.179924 | 0.270896 |
| Aggressive ego motion | 0.026900 | 0.112552 | 0.145949 |
| Oncoming | 0.026360 | 0.096241 | 0.128690 |
| Lane change | 0.029010 | 0.136314 | 0.358940 |
| Twice declared noise | 0.057174 | 0.237826 | 0.363997 |
| Large initial offset | 0.028579 | 0.119857 | 0.182609 |
| Intermittent dropout | 0.032087 | 0.130825 | 0.197831 |

The worst reported post-burn-in position error in the one-second dropout group
was 0.438817 m. Twice-declared noise deliberately exceeds the sensor envelope;
truth motion remaining in its domain does not certify that noise case.
`DigitalCertified` remains false for every trial.

At a 0.5 s radar period, even fine integration has position/velocity/acceleration
RMSE 1.70634 m / 5.77473 m/s / 7.01424 m/s². The repaired integrator reproduces
this poor sampled-data performance; it does not make 2 Hz sensing accurate.

## Remaining failures and diagnostic evidence

### Four short-range controller admissions

All use the oncoming scenario and 16 m confirmation range. The target continues
at its scenario speed (8 m/s on the straight case) when reference ego speed
changes. Failure times are first rejected frames, not collision times.

| Expanded case | Condition | Failure time |
| --- | --- | ---: |
| 25 | Straight road, ego reference 2 m/s | 4.20 s |
| 35 | Straight road, steering slew 0.3 rad/s | 2.60 s |
| 47 | Curvature 0.01/m, steering slew 0.3 rad/s | 2.60 s |
| 48 | Curvature 0.01/m, steering slew 0.7 rad/s | 2.60 s |

Seed width changes and longer performance windows alone did not recover these
admissions. All ten tested range variants at 24/32 m completed, with minimum
sampled gaps from about 0.0614 to 0.4437 m. This establishes successful
configurations with earlier detection. It does not prove that avoidance at
16 m is physically impossible, and it does not justify silently changing a
sensor's stated range.

### Nonlinear vehicle and estimator/controller coupling

Truth-fed PassVeh14DOF straight and circular runs both fail at 1.45 s after
29 executed holds. Last measured gaps remain 1.44174 and 1.65435 m. Repeating
with 0.5/1.0 m separation margins still fails around 1.3–1.4 s. A larger
clearance does not resolve the mismatch between the nonlinear vehicle and the
controller's zero-residual affine plant contract.

At the final successor, the truth-fed straight/circular runs differ from the
carried next prediction by approximately 0.0514/0.0582 m in lateral position,
0.0376/0.0446 rad in heading, 1.3492/1.4841 m/s in lateral velocity, and
1.3136/1.5694 rad/s in yaw rate. The previous accepted frames also refit road
boundaries, requiring fresh admission. These observations invalidate reuse of
an exact-plant guarantee; they do not uniquely apportion failure among vehicle
dynamics, road refits and incomplete direction search.

The estimator-fed straight nonlinear scenario and estimator-fed declared-plant
scenario both fail on first target publication at 3.55 s. In the declared-plant
failure, the admitted target speed interval is [0, 32.5623] m/s and its course
interval has half-width pi. Speed-rate bounds are [-2, 2] m/s² and curvature
bounds approximately [-0.0031, 0.0031]/m. This activates the conservative
path-length-ball prediction: its position remainder is 24.6717 m after 0.5 s,
49.8435 m after 1 s, and 101.687 m after 2 s. Accurate mean estimates alone
therefore do not provide a useful finite-encounter robust certificate at first
detection. This issue persists on the declared ego plant, independently of
the nonlinear vehicle mismatch.

The estimator-fed circular nonlinear run fails earlier, at 1.50 s, before
target publication. Counterfactual replay passes if either road boundaries
are removed or ego uncertainty is set to zero; retaining both fails. Changing
solver optimality tolerance from 1e-7 to 1e-6 or 1e-4 does not recover it.
The joint road/uncertainty restriction is implicated. Those counterfactuals
are diagnostics only and were not adopted as production changes.

The estimator scenarios were explicitly supplied the current zero-residual
controller configuration. Their usual finite-sensing residual configuration
is rejected by the current affine controller contract. No claim is made that
this override certifies the actual nonlinear plant.

### Model-domain and timing limits

The existing controller pass predicate checks completion, sampled separation,
enabled road constraints, actuator slew, and the slack-dependent CLF inequality.
It does not enforce the optional physical state-domain audit. That audit is
negative in 57/120 original cases and 73/120 expanded cases, with minima
-6.19468 and -2.57655 respectively. A passing affine simulation must therefore
not be presented as validated nonlinear-model operation.

The original sweep has 30 cases with a maximum frame above 50 ms; the expanded
sweep has 38. Maxima are 1156.42 and 486.576 ms respectively. Expanded trials
record 1,076 frames exceeding their own hold period (which varies by case).
Runs included cold starts and concurrent MATLAB work, so these are observed
overruns, not isolated performance benchmarks. No real-time claim is made.
Exceptional search and long low-speed horizons trade computation for additional
admission opportunities; deadlines remain enforced when configured.

### MATLAB process teardown

Several batch processes wrote complete result files and then reported
`free(): chunks in smallbin corrupted` during shutdown. The final controller,
estimator and range-diagnostic shell sessions ended with status 137. A separate
`matlab -batch "disp('STRESS_ENVIRONMENT_PROBE_OK')"` also reproduced the
shutdown failure without executing project algorithms. The saved controller
and estimator results were independently loaded and checked in the persistent
MATLAB session. These batch commands are not reported as clean shell exits.
The final full unit suite did exit normally with status 0 and passed
`assertSuccess`; the physical diagnostic process also exited normally while
preserving its caught scenario failures.

## Validation and artifacts

The full final suite passed 835/835, including the 12 new cases in
[`algorithmStressTest.m`](../tests/algorithmStressTest.m). Earlier baseline
regressions demonstrated the injected acceptance, coarse-step integration and
horizon issues before repair. The first full baseline run was interrupted by
an external timeout in one Simulink pipeline test; that incomplete baseline
case is not counted as an algorithm defect. Subsequent full suites passed
830/830 and then 835/835 as coverage expanded. MATLAB Code Analyzer reported
no new warning or error in the changed code; existing informational sparse
assembly/indexing suggestions remain. `git diff --check` passed.

CSV exports and selected diagnostic logs are in
[`ALGORITHM_STRESS_20260925/`](ALGORITHM_STRESS_20260925/). They include both
baseline and final controller/estimator tables, the first expanded sweep,
integration comparisons, all unit-test outcomes, range recovery, and physical
failures. Full MAT traces, scratch diagnostic scripts and the original working
tree patch are retained locally under the ignored directory
`simulation_output/controller_estimator_stress_20260925/`. These are experiment
outputs, not a tracked historical implementation. No generated binary, solver
dependency, literature tree, figure or unrelated comparison artifact is added
to the project commit.

Reproduce the principal campaigns from the repository root:

```matlab
results = runtests('tests');
assertSuccess(results);
addpath('scripts');
runControllerEstimatorStressCampaign( ...
    OutputDirectory='simulation_output/stress_reproduction', ...
    Groups=["declared","operating","estimator","integration"]);

% Targeted unresolved 16 m cases; results retain their original case numbers.
runControllerEstimatorStressCampaign( ...
    OutputDirectory='simulation_output/stress_unresolved', ...
    Groups="operating", CaseIndices=[25,35,47,48]);

% A demonstrated configuration improvement, with the actual larger range.
runExactStateRecursiveFeasibilityScenario(Scenario="oncoming", ...
    SteeringRateMaximum=.3, BrakingRatioRateMaximum=2, ...
    ConfirmationRange=24, SampleCount=240, DeadlineSeconds=Inf, ...
    SearchTimeLimitSeconds=30, RethrowFailure=false);
```

The physical probe used `horizonSteps=32`, `certificateSearchTimeLimit=30`,
`frameDeadlineSeconds=Inf`, and otherwise default controller settings with
`runOncomingVehicleAvoidanceScenario`,
`runCircularCenterlineStraightTargetAvoidanceScenario`, and
`runEstimatedStateAvoidanceScenarios` (plot/report/progress disabled where
available). The declared estimator probe was
`runDeclaredPlantEstimatorControllerScenario(SampleCount=240,
DeadlineSeconds=Inf, OutputDirectory=...)` with its other defaults unchanged.

The next technical requirements are a validated nonlinear prediction/residual
contract, useful first-detection target uncertainty with sufficient sensing
lead time, and a complete-frame timing strategy. No such redesign or guarantee
is claimed by this completed set of fixes and experiments.
