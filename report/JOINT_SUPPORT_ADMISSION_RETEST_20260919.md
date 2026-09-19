# Joint-support admission retest

September 19, 2026. Controller-only experiment. Acceptance interpretation corrected
on September 19 after the user clarified that any strictly positive body gap is
acceptable. The controller's configured 0.25 m planning buffer is not a task
acceptance threshold.

## Result

**The previous admission infeasibility is resolved in all thirteen previously
failing fixtures when the joint-support method has an offline search budget.**
All fifteen straight/circular obstacle trials complete 300 holds (30 s), confirm
target release and recover cruise. However, **100 ms execution is still not
qualified**: fourteen of fifteen strict periodic trials fail to produce an
admitted command. The remaining straight crossing case completes but is a weak
avoidance challenge. **All fifteen offline trials pass the sampled noncollision
criterion: actual signed body gap strictly greater than zero.** Straight
stationary and oncoming gaps below the configured planning buffer are diagnostic
buffer shortfalls, not failures of the user's collision-avoidance requirement.

This distinguishes existence of a found hard-certified plan from finding it
before the execution deadline. Neither outcome proves feasibility for arbitrary
encounters or continuous-time safety between nodes.

## Versions and concurrent workspace changes

The first campaign tests committed version
`8529584774ffd17b2cae7d2fb18aac219b2fde13`, where `jointSupport` must be enabled
explicitly and `fixedNormal` remains the default. It completes fifteen offline
trials and fifteen separate periodic trials. Three fixed-normal checks still
reproduce the prior straight-oncoming and circular stationary/crossing
PrimalInfeasible admissions.

Those campaigns finish at approximately 00:41 Chicago time. At 00:42, the working
tree changes again: the old selector is removed and joint support becomes the
sole active-encounter policy. One subsequent supplemental command aborts during
argument parsing because `CertificateMethod` no longer exists; that attempt
executes no simulation and is not counted as a controller failure.

To avoid mixing code versions, the working tree is frozen outside the repository
at **2026-09-19T00:43:24.954693-05:00**, as controller format 37. Its file hashes and
patch relative to the committed version are retained in
`current-source-manifest.json` and `current-source.patch`. The snapshot uses the
existing native binaries without modifying them. A second fifteen-case campaign
runs this new default entry point: each 30 s offline trial is immediately
followed in the same process by its 100 ms-budget trial. **The tables below use
this frozen default-entry campaign.** Further concurrent working-tree edits are
not silently attributed to this snapshot, and are excluded from this task's commit.

## Configuration and checks

- MATLAB R2026a, one computational thread, desktop execution, profiler disabled.
- Reference speed 8 m/s; physical control hold 0.1 s; 300 requested holds per trial.
- Straight and left/right circles of radius 100 m and 50 m: curvature
  0, +/-0.01 and +/-0.02 per metre.
- Existing stationary, oncoming and crossing target fixtures. Curved targets
  move along inertial lines, not along the reference circle.
- Exact held affine plant and exact ego/target sensing; zero process residual,
  zero target jerk; seed 20260912; sensing range 16 m; configured planning buffer
  0.25 m. Experimental collision acceptance requires actual signed body gap > 0;
  neither touching nor overlap is acceptable.
- No physical road boundaries or estimator; existing pose-validity constraints
  remain. No terminal fallback commands are executed.
- Offline frame/search budgets are 30 s. Strict frame/search budgets are 0.1 s.
  The model hold stays 0.1 s in both. Offline execution is an algorithm study,
  not a claim that a late command can be applied safely to a physical vehicle.
- Frame time includes measurement construction and controller execution; plant
  propagation and physical-geometry audits are outside the frame timer.

Initial committed-version regression: **63 passed, zero failed/incomplete**
across joint-support, shifted-nominal, node, curved-pose and constant-curvature
classes. Frozen default-entry regression: **19 passed, zero failed/incomplete**
in `jointSupportCertificateTest`. These are separate version-specific runs,
not 82 distinct test cases. Tests run through MATLAB MCP and do not overlap
the CLI timing processes.

## Full results for the default entry point

The admission-time column is the first active-target frame's total latency.
For oncoming targets this occurs at **2.6 s**, after 26 target-free holds; for
the stationary/crossing fixtures it occurs at time zero. The gap column is the
minimum oriented-rectangle body distance sampled eleven times per hold.

| Curvature / m | Target | Offline holds | First admission / ms | Conic calls at admission | Minimum sampled body gap / m | Strict periodic holds |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| +0.00 | stationary | 300/300 | 999.650 | 3 | 0.225898 | 0/300 |
| +0.00 | oncoming | 300/300 | 179.607 | 9 | 0.186402 | 26/300 |
| +0.00 | crossing | 300/300 | 27.211 | 1 | 9.519691 | 300/300 |
| +0.01 | stationary | 300/300 | 1675.965 | 40 | 0.347674 | 0/300 |
| +0.01 | oncoming | 300/300 | 559.702 | 24 | 0.310048 | 26/300 |
| +0.01 | crossing | 300/300 | 4649.472 | 126 | 0.346356 | 0/300 |
| -0.01 | stationary | 300/300 | 1376.830 | 34 | 0.339601 | 0/300 |
| -0.01 | oncoming | 300/300 | 495.959 | 21 | 0.293824 | 26/300 |
| -0.01 | crossing | 300/300 | 3588.134 | 94 | 0.338406 | 0/300 |
| +0.02 | stationary | 300/300 | 826.023 | 21 | 0.448968 | 0/300 |
| +0.02 | oncoming | 300/300 | 443.575 | 19 | 0.431455 | 26/300 |
| +0.02 | crossing | 300/300 | 4689.786 | 124 | 0.461360 | 0/300 |
| -0.02 | stationary | 300/300 | 906.756 | 22 | 0.421928 | 0/300 |
| -0.02 | oncoming | 300/300 | 394.151 | 17 | 0.392341 | 26/300 |
| -0.02 | crossing | 300/300 | 5935.810 | 138 | 0.418633 | 0/300 |

The prior failures comprise **eight curved stationary/crossing admissions plus
five straight/curved oncoming admissions: thirteen resolved offline failures**.
All fifteen offline cases pass the sampled noncollision check. The straight
stationary gap is **0.225898 m**, and the straight oncoming gap is **0.186402 m**;
both are acceptable positive separations. Thirteen cases also maintain the
configured 0.25 m buffer at all audit samples; that stronger diagnostic is not
required for experimental collision acceptance. These sampled distances do not
prove the global continuous minimum.

The original raw MAT/JSON traces are preserved unchanged. Their historical
`passed` flags used the configured buffer as an acceptance threshold and are
superseded by this explicit reclassification. The revised report JSON separates
`sampledCollisionFree` from `configuredClearanceMaintained`. No trajectories or
runtime measurements are changed by reclassification. The scenario and campaign
drivers now use and expose the same distinction for future experiments; the
controller configuration and its command-certification checks are unchanged.

The +0.02 crossing trial also has a later admission at 22.4 s, after an earlier
release. It is recorded separately from the first admission in the JSON. All
fifteen runs still complete. There are sixteen active admission events in total.

## Why the old first-frame failure changes

The old method fixed directions from a colliding nominal and solved that one
convex family. The new method permits separation and exit angles to move with
the trajectory, uses internal feasibility restoration, and verifies the complete
hard certificate before execution. Positive restoration deficits are never
issued as controls. It retains the full accepted certificate for continuation.

The experiment verifies all **4,500 offline issued commands** using the current
hard checks. All **505 active successor frames** preserve a nonpositive shifted
joint residual and use exactly one conic solve; the largest shifted residual is
**-6.89547103e-05 m**. Slew and relaxed CLF residual checks also pass. This is
numerical evidence under the declared model and contracts, not an unrestricted
recursive-feasibility or physical-vehicle theorem.

The first admitted plans cost 1--138 conic calls. The difficult crossing cases
use 94--138 calls and three initializations. Consequently, the committed
implementation's admission behavior differs materially from the previous
one-fixed-normal-solve policy. Its active successor frames retain one solve.

## Runtime after admission and the 100 ms gate

Actual continuous-trace observations for the frozen snapshot are:

| Frame category | Count | Median / ms | Maximum / ms | Frames over 100 ms |
| --- | ---: | ---: | ---: | ---: |
| Active encounter with a shifted certificate | 505 | 16.191 | 85.474 | 0 |
| Target-free continuation, with target-free predecessor | 3958 | 4.509 | 29.309 | 0 |

These rows describe all frames meeting the stated categories, not repeated
measurements of one selected frame. Admission and target-set transition frames
are accounted for separately in the JSON; they are not hidden in steady-state
statistics. The existing whole-frame timer is retained.

Each strict trial is run immediately after the complete offline run of the
same scenario. Nevertheless fourteen strict trials end without a command at
the first active admission; measured unsuccessful frames are approximately
100--112 ms. The error reports restoration stopping without a hard-safe
certificate under its stall/iteration/time guard. The paired larger-budget
runs establish that these fixtures do have solutions found by this algorithm.

In particular, straight oncoming admission takes **179.607 ms** offline after
2.6 s of previous closed-loop operation. Circular oncoming admissions take
approximately **394--560 ms**. These are not merely application-startup frames.
Some circular crossing admissions require **3.588--5.936 s**. Therefore the
remaining operational issue is admission search latency, even though the tested
active successor frames meet 100 ms. No worst-case timing bound is established.

## Cruise recovery

All fifteen offline trials return to their own straight/steady-turn reference.
Across final two-second windows, maximum absolute lateral, heading and speed
errors are approximately **1.91e-05 m**, **2.41e-08 rad** and
**1.59e-07 m/s**, respectively. The lateral-velocity and yaw-rate errors
are in the JSON. No early-recovery deadline is imposed.

## Reproduction and artifacts

Acceptance-correction validation: independently reload all fifteen original MAT
traces and reconstruct physical gap as `minimumSampledSeparationMargin +
configuration.collision.clearanceMargin`. All fifteen pass the corrected gap,
completion, slew and relaxed-CLF checks. Rerun the straight stationary and
oncoming fixtures for 60 holds each through the corrected campaign driver with
30 s search/frame budgets. Both pass, reproduce gaps 0.225898 m and 0.186402 m,
and explicitly report `configuredClearanceMaintained=false`. Five existing
MATLAB tests pass across `circularArcExactStateScenarioTest` and
`straightRoadBoundaryConfigurationTest`. These supplemental checks use the
current joint-support-only working tree; they do not replace the frozen
campaign's timing measurements. Validation logs, MAT outputs and the exact
working-tree patch are retained under
`/home/zai/.cache/collisionAvoidance/positive-gap-acceptance-20260919/`.

Committed-version offline command:

```matlab
addpath('scripts');
runJointSupportCertificateValidation( ...
    OutputDirectory=outputDirectory, SampleCount=300, ...
    Curvatures=[0,.01,-.01,.02,-.02], Methods="jointSupport");
```

The frozen format-37 entry has no `CertificateMethod` option. The isolated
`runner/runCurrentEntryRetest.m` calls `runExactStateRecursiveFeasibilityScenario`
with `SampleCount=300`, each curvature/scenario, and budget 30 then 0.1 s.
Executable production drivers remain in `scripts/`; reports remain in `report/`.

Raw artifacts: `/home/zai/.cache/collisionAvoidance/joint-support-retest-20260919/`.
They include both campaigns, warmups, the three fixed-normal checks, the
argument-parsing failure log, source snapshot/manifest, final default-entry
traces, and both test logs/MAT results. The committed JSON summarizes all
outcomes and identifies the exact snapshot hashes. Report aggregation asserts
completion, hard verification, shifted residuals, one-call continuation,
actuator slew, relaxed CLF and expected baseline failures. `git diff --check`
passes for this task's reports.

The original retest committed only its report and JSON. This acceptance correction
also updates the two experiment drivers. Concurrent controller/configuration,
driver-policy, test and documentation work, dependencies, binaries and unrelated
untracked files are deliberately excluded. No new controller implementation,
estimator experiment, network modification or hardware test is claimed.
