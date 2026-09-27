# Nonlinear controller runtime diagnosis and road-constraint ablation

Prepared 2026-09-26 (America/Chicago). Experimental workspace based on `f58160a5e42f0f12e4d0550f28b4bbe99c3b23a4`; MATLAB 26.1.0.3276743 (R2026a) Update 3.

## Findings

Road geometry is a substantial cost in the experimental `nonlinearShooting` controller. Removing geometric road rows roughly halves the measured straight/circular continuation latency, but leaves seconds of computation. The S-bend continuation improves only 1.21x because nonlinear integration and reference-curvature interpolation are also expensive. These timings diagnose the current experimental workspace, not a regression bisect of Git commits. The latest existing commit (`f58160a`) contains only simulation reports; the nonlinear controller implementation predates this task as uncommitted workspace work. The default controller remains `affineSocp`.

The earlier campaign has 1,080 nonlinear frames. In 1,067 frames, the first seed is already feasible at iteration zero and the optimizer executes four SQP iterations; 1,077 frames invoke the optimizer once. The slowdown is not uncontrolled iteration growth. Four major iterations can still require many line-search trajectory evaluations. Output-function termination after four feasible refinement iterations commonly reports exit flag -1; that is not by itself a rejected control plan.

## Controlled road-on / road-off timing

Each row holds the measured state, target observation, warm plan (when present), model and all solver settings fixed. The road-off model removes only `model.road.boundaries` and `model.road.lateralClearance`. The centerline, path-recovery objective, terminal tolerances, collision constraints, actuation/slew limits and physical state/slip domains remain. One warm-up and three unprofiled repetitions are executed per variant, serially in one single-computational-thread MATLAB process before launching the plant experiments. Values are medians, not hard deadlines.

| Frozen fixture | Road on (s) | Road off (s) | Speedup | Maximum full-plan difference |
|---|---:|---:|---:|---:|
| straight-continuation | 3.9515 | 1.7333 | 2.28x | 1.913e-15 |
| straight-fresh-visible | 1.8756 | 1.1896 | 1.58x | 0.2071 |
| circular-continuation | 2.3605 | 1.1666 | 2.02x | 0.02328 |
| sCurve-continuation | 6.0279 | 4.9750 | 1.21x | 0.04809 |
| sCurve-fresh-visible | 4.6174 | 4.1875 | 1.10x | 0.1029 |

Continuation fixtures use frame 2 at t=0.05 s. The first command is recomputed from the original initial measurement and agrees exactly with the saved campaign command in all three scenes. Fresh-visible fixtures use the measured state at t=1.05 s with an empty previous-controller record; they are admission diagnostics, not replays of the original full encounter history. Plan differences mix steering radians and dimensionless tire utilization and are a diagnostic equality check, not a norm with physical units. Road removal changes several plans; timing differences include the changed SQP search as well as the removed row cost. Every road-on repetition reproduces its frozen baseline within 1e-8; every road-off solve admits a plan and reports zero road rows. The fastest road-off fixture still requires about 1.17 s, versus a 0.05 s control period.

## Where the time goes

All percentages below refer to a separate instrumented MATLAB profiler run, which is slower than clean execution. Inclusive times overlap and must not be added.

- Straight continuation: clean median 3.9644 s, profiled solve 6.4804 s. Constraint evaluation consumes 5.0185 s (77.4% inclusive); the boundary helper alone consumes 3.7034 s (57.1% inclusive). There are 96 full trajectory evaluations and 270,144 boundary-helper calls. With 201 audit nodes, two boundaries and seven evaluations per boundary (one value plus central differences in x, y and yaw), the count is exactly `96 * 201 * 2 * 7`. Polygon clipping runs 540,288 times. Native bicycle prediction is only 0.4172 s in this profile.
- Circular continuation: clean median 2.3227 s; 51 full trajectory evaluations, 143,514 boundary calls, and 2.0337 s inclusive boundary time out of a 3.8977 s profiled solve.
- S-bend continuation: clean median 6.0985 s; the 9.2491 s profiled solve spends 4.3032 s in the rollout wrapper. `ppval` is called 306,152 times and consumes 2.9328 s self time (31.7%). The boundary helper consumes 1.6411 s inclusive. The varying-curvature branch of `ltvBicycleModel.nominalRollout` returns through MATLAB `localProfileRollout` before reaching the native MEX branch used for straight/constant-curvature roads. Central differences for six states and two inputs propagate 17 trajectories through RK4 when state/input Jacobians are requested. This branch already exists in the model; repeated shooting evaluations amplify its cost.
- Each `laneGeometry.referenceCurvature` call evaluates both the curvature polynomial and its derivative, even when the caller requests only curvature. Profile data is cached; this is repeated interpolation/access overhead, not rebuilding the spline on every call.
- Independent RK4 physical-domain audits remain another cost. Straight continuation uses approximately 0.5314 s inclusive in `nonlinearAvoidanceRollout`; S bend uses approximately 1.246 s. These overlap seed/acceptance work and are not additional disjoint percentages.

An external counter-instrumented copy of the solver confirms redundant derivative work without changing accepted plans: straight continuation makes 95 value-only objective calls and 95 value-only constraint calls, plus 12 derivative-output calls each; S-bend continuation makes 40 value-only calls and 12 derivative-output calls each. The current callbacks calculate full gradients/Jacobians in both cases, and trajectory cache misses calculate all state/input sensitivities. Instrumented full-plan differences are exactly zero. The diagnostic copy remains outside the repository and does not replace the production solver.

Public-controller fixed-frame times are close to direct-solver times (straight 3.901 vs 3.964 s, circle 2.317 vs 2.323 s, S bend 6.108 vs 6.098 s; independent timing samples). Road fitting in the prior campaign has medians only 1.87/5.04/4.93 ms; observer runtime is zero for these exact-state runs. Neither sensor fitting nor previous concurrent plant processes explains the seconds-scale latency.

## Full plant ablation

Three exact-state PassVeh14DOF runs request 18 s at 10 m/s, 50 ms control period, 50 m initial target distance, 30 m visibility, 5 by 2 m rectangles and 1.5 m target offset. Circle radius is 100 m. All use the same nonlinear 5 s / four-hold / 12-iteration / 20 s budget profile as the earlier road-on campaign. Separate single-threaded plant processes overlap, so their full-run timings are descriptive; use the serial fixed-input table for speedup claims. No stochastic measurement inputs are used.

| Scene | Complete | Min SAT gap (m) | Post-hoc min road margin (m) | Recovery | Median controller (s) |
|---|---|---:|---:|---|---:|
| straight | True | 0.112878 | 6.825441 | True | 1.267 |
| circular | 1.9 s prefix only | 7.095945 | 6.990871 | False | 1.169 |
| sCurve | True | 0.128961 | 5.970351 | True | 5.597 |

Circular full-run status: the original 18 s attempt stops making progress inside controller frame 39 at t=1.90 s. A checkpointed independent reproduction confirms that plant integration has not started that frame. A direct solver trace remains inside the third fmincon seed after its iteration-zero callback. Initial third-seed objective/constraint derivatives are finite (50 decisions, 3,622 constraints; maximum absolute Jacobian entry 33.4291), but a separate HiGHS LP finds its linearized constraints infeasible within decision bounds; minimum uniform relaxation is 0.501616 in mixed constraint units. This LP is not a nonlinear infeasibility proof and does not identify the exact internal fmincon routine. A diagnostic replay capped before seed three returns no accepted plan; its best retained violation is in collision constraints. The third-seed busy calls outlive the nominal 20 s budget because the budget is enforced only between callbacks. SIGINT does not stop the original process; it and redundant reproductions are explicitly terminated after saving diagnostic inputs/logs. The separately saved 1.90 s prefix is checked against the checkpoint last command; it is not a completed avoidance run.

Road margins refer to outer curbs at signed offsets -8.6/+10.6 m (lane offsets 6/8 m plus 2.6 m shoulders), including each fit-error reserve. The audit also checks parameter-domain coverage. Finite recorded samples are not a continuous-time containment guarantee. Recovery requires complete-body passing and final 2 s maxima within 0.5 m/s speed, 0.2 m lateral and 0.02 rad course error. See the exported JSON for coverage, negative-margin counts, errors and failure status.

## Default affine controller with road constraints off

The same three 18 s avoidance scenarios were also attempted with default `affineSocp` and `RoadBoundaries=false`. These are functional checks, not solver-only timing comparisons.

| Scene | Executed time (s) | Complete | Outcome |
|---|---:|---|---|
| straight_oncoming | 1.80 | False | collisionAvoidanceController:optimizationFailed |
| circular_oncoming | 1.45 | False | collisionAvoidanceController:optimizationFailed |
| varying_curvature_oncoming | 1.75 | False | collisionAvoidanceController:optimizationFailed |

Stopped prefixes are not completed avoidance or recovery. The compact summary retains error messages, physical gap and timing. Road containment is not assessed for these extra affine prefixes.

## Implementation and scope

The existing straight and circular scenario APIs already accept `RoadBoundaries=false`. The S-bend scenario now exposes the same switch, defaulting to true. `runRoadConstraintAblation` explicitly turns it off for the nonlinear development profile. The common scenario adapter removes fitted boundaries and lateral clearance before controller admission; retained centerline/model-domain constraints still apply. No controller, optimizer, plant, horizon, objective weight or collision-constraint code was edited in this operation.

`runRoadConstraintAblation` saves the physical trial and reconstructs curb observations afterward at the executed control poses. Those boundary observations never reach the controller. `auditRoadConstraintAblation.py` checks full vehicle rectangles against those finite quadratic fits over the recorded plant samples. Its separate post-hoc result avoids interpreting a disabled road audit as proof of containment.

The next performance work supported by these measurements is lazy value/derivative evaluation during line search, followed by compiled/vectorized varying-curvature RK4 and efficient curvature-polynomial evaluation. Boundary analytic/batched derivatives matter if road containment is re-enabled. Reducing safety checks or shortening the physical horizon is not necessary to implement those equivalent-computation optimizations. No such optimization is claimed here.

## Reproduction and validation

```matlab
addpath('scripts');
profileNonlinearControllerRuntime(profileDirectory, originalCampaignDirectory);
compareRoadConstraintRuntime(profileDirectory);
runRoadConstraintAblation(outputDirectory, "straight");
runRoadConstraintAblation(outputDirectory, "circular");
runRoadConstraintAblation(outputDirectory, "sCurve");
```

```bash
uv run --with scipy python scripts/auditRoadConstraintAblation.py \
  OUTPUT/straight.mat OUTPUT/circular.mat OUTPUT/sCurve.mat \
  --output OUTPUT/independent-road-free-audit.json
```

`profileNonlinearControllerRuntime` separates three clean samples from profiling, raises only the profiled time budget from 20 to 300 s, and asserts equality of accepted full plans (all five measured differences are zero). Post-measurement harness cleanup removed an unused warm-up output assignment and a nonexistent optional addpath entry; solver calls/settings and numerical work were unchanged. The workspace snapshot includes the cleaned reusable harness. Its diagnostic load uses horizon 5 s, sample time 0.05 s, blockSteps 4, auditSubsteps 2, maxIterations 12 and four feasible refinement iterations: 100 holds, 201 audit nodes and 50 decision variables. These differ from the controller's untouched development defaults. All eight existing nonlinear plan unit tests pass (zero failures/incomplete tests, 1.3128 s); four MATLAB scripts pass Code Analyzer with zero findings and the new Python auditor compiles. Fixed-input assertions, source hashes and independent physical audits support the reported results; no full production unit-test suite is claimed.

Original profile MAT files, full per-line profiles, command logs, counter-instrumented copy and plant results are in:
`/home/zai/.cache/collisionAvoidance/nonlinear-runtime-diagnosis-20260926-211837`.
The source manifest records pre-existing experimental sources; a final snapshot and raw-artifact manifest preserve the executed workspace. Compact measurements accompany this report. Raw MAT files, native dependencies and generated figures are kept outside the project commit. The scoped commit includes the new scripts, the S-bend road switch only, and this report/export bundle; existing controller/configuration/observer/scenario edits remain uncommitted.

The independent audits pass for both completed nonlinear runs, the deliberately shortened circular prefix and all three stopped affine prefixes. Three nonlinear paired-scenario checks pass (the circular prefix deliberately changes only requested duration in addition to road removal). All 165 executed workspace-source/native hashes match; among the 124 initially hashed controller/configuration/script sources, only the intended S-bend scenario switch changed. The external comparison PNG/PDF was generated and visually inspected; its files and all original diagnostic artifacts are listed in `raw-artifact-hashes.csv`.
