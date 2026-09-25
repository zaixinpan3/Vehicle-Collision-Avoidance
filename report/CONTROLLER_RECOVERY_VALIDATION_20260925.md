# Controller avoidance and cruise recovery validation — September 25, 2026

This fresh campaign evaluates collision avoidance and subsequent sustained
constant-speed tracking separately. The declared affine model completes all
15 thirty-second runs and returns to 8 m/s path cruise. In the independent
PassVeh14DOF nonlinear vehicle model, both no-target cruise controls finish
18 s successfully, but **none of the five avoidance scenarios completes**.
Straight/circular truth-fed avoidance stops at 2.20 s, estimator-fed avoidance
stops at 1.05 s, and the S-curve reference is rejected before execution.
Two additional longer-horizon nonlinear diagnostics also fail at 2.20 s.
The current controller therefore demonstrates model-level avoidance and
recovery, but does not meet the requested end-to-end nonlinear avoidance
and recovery objective in these tests.

## Criteria and reproducibility

- Base revision: `434e8fa1410dd609782690241775e0567f3ac285`.
- MATLAB R2026a Update 3; installed Vehicle Dynamics Blockset PassVeh14DOF.
- Entry: `scripts/runControllerRecoveryValidation.m`. No controller, estimator,
  solver or physical-model implementation was changed for this campaign.
- Declared model: 30 s, 600 holds at 0.05 s, nominal horizon 1.6 s, 8 m/s,
  friction coefficients [0.85, 0.85], exact sensing, seed 20260925. The NRMM
  target contract and existing per-scenario target trajectories are retained.
  Offline search/frame budgets are 30 s; these are not real-time trials.
- Declared-model geometry audit: every 5 ms, including hold endpoints. A
  strictly positive signed rectangle gap is required. This finite audit is
  not proof of continuous-time collision freedom.
- Recovery: completed requested simulation, with the final **continuous 2 s**
  within longitudinal speed error 0.5 m/s, path lateral error 0.2 m, and
  velocity-course error 0.02 rad. Course includes sideslip; body yaw alone
  is not the direction of travel. Recovery time is the earliest post-pass
  sampled time after which those tolerances remain satisfied through the end.
- Passing is an event over the full trajectory. Testing only the final
  obstacle projection onto the ego heading is misleading after a circular
  path rotates. The initial output was re-summarized from preserved MAT
  results to use the first complete longitudinal passing event.
- Straight bounded runs use physical road edges at lateral +/-5 m. The
  curved runs do not construct road boundaries. They cannot validate staying
  within a finite curved road.
- Raw MAT results are external at
  `/home/zai/.cache/collisionAvoidance/recovery-validation-20260925/`.
  Compact CSV/JSON exports and source/result hashes are in the adjacent
  `CONTROLLER_RECOVERY_VALIDATION_20260925/` directory. Existing unrelated
  working-tree modifications are preserved; their initial status and actual
  source hashes are recorded. The two pre-existing scenario edits make road
  boundaries optional, but their default remains enabled in these trials.

## Declared-model outcomes

All rows complete 30 s, remain moving, have positive audited obstacle gaps
where a target is present, and satisfy final-window recovery. Left/right
curvature is +/-0.01 /m (radius 100 m). Times are measured from scenario start,
not from passing. A dash means no obstacle or no enabled road boundary.

| Scenario | Minimum body gap (m) | Baseline collision | Minimum speed (m/s) | Peak lateral error (m) | Recovery time (s) | Minimum road margin (m) |
| --- | ---: | --- | ---: | ---: | ---: | ---: |
| `crossing_k+0.00` | 9.519588 | No | 8.0000 | 0.0006 | 2.50 | — |
| `crossing_k+0.01` | 0.815335 | Yes | 3.6282 | 3.9271 | 4.40 | — |
| `crossing_k-0.01` | 0.579161 | Yes | 3.4146 | 3.9578 | 4.40 | — |
| `cruise_k+0.00` | — | — | 8.0000 | 0.0000 | 0.00 | — |
| `cruise_k+0.01` | — | — | 8.0000 | 0.0000 | 0.00 | — |
| `cruise_k-0.01` | — | — | 8.0000 | 0.0000 | 0.00 | — |
| `oncoming_k+0.00` | 0.262988 | Yes | 5.7261 | 2.9176 | 6.10 | — |
| `oncoming_k+0.01` | 0.376556 | Yes | 5.9235 | 3.0402 | 6.10 | — |
| `oncoming_k-0.01` | 0.376556 | Yes | 5.9235 | 3.0402 | 6.10 | — |
| `stationary_k+0.00` | 0.098536 | Yes | 4.5564 | 2.3298 | 4.15 | — |
| `stationary_k+0.01` | 0.200979 | Yes | 4.9905 | 2.3281 | 4.20 | — |
| `stationary_k-0.01` | 0.200979 | Yes | 4.9905 | 2.3281 | 4.20 | — |
| `cruise_straight_boundaries` | — | — | 8.0000 | 0.0000 | 0.00 | 4.050000 |
| `oncoming_straight_boundaries` | 0.262992 | Yes | 5.7261 | 2.9176 | 6.10 | 0.270876 |
| `stationary_straight_boundaries` | 0.098536 | Yes | 4.5564 | 2.3298 | 4.15 | 1.441404 |

The default straight crossing target starts at (15, -4) m and crosses at
32 m/s; it clears the path well before the 8 m/s ego reaches the crossing.
Undisturbed cruise already has a 9.5197 m minimum body gap. This row is a
non-conflicting passage check, **not evidence of active collision avoidance**.
The two curved crossing scenarios do collide under undisturbed cruise
(baseline minimum gap -3.3606 m) and require an avoidance maneuver. All other
obstacle rows also have colliding nominal baselines. Thus 10 genuinely
conflicting declared-model trials pass, plus one non-conflicting target trial
and four no-target cruise controls.

Across all declared runs, the final 2 s maximum speed error is below
3.4e-7 m/s, lateral error below 2.52e-5 m, and course error below 2.1e-15 rad.
These are deterministic model results near its equilibrium, not expected
sensor/vehicle accuracy. Genuine encounters recover at 4.15–6.10 s; the
smallest controlled body gap is 0.098536 m. Curved crossing reaches almost
3.96 m lateral excursion, emphasizing why missing curved road boundaries
matter even though the final path error is small.

All 10 genuinely conflicting cases also exceed an **unenforced diagnostic
model-domain bound** during avoidance (negative margin in
`model-domain-audit.csv`). Sampled body-heading errors reach approximately
0.402–0.710 rad versus the configured 0.4 rad diagnostic limit. These
excursions do not invalidate execution of the declared affine equations,
but they weaken any inference about their accuracy for a physical vehicle.
Safety/recovery passing is therefore not a nonlinear-model validation.

## Nonlinear vehicle outcomes

All seven runs request 18 s at 10 m/s, use the existing scenario defaults
including 50 m initial target range, 30 m detection range, 0.05 s updates,
16 nominal prediction stages (0.8 s), and a 5 s search limit. Obstacles move
at 10 m/s; avoidance recovery windows are 2 s. Truth-fed avoidance scenarios
retain default road boundaries (offsets 6/8 m plus 2.6 m shoulders); the two
no-target cruise controls use their default disabled boundaries. Estimator
runs use seed 20260925 and a matching 10 m/s target-speed prior.

Positive separation and road margins in a failed row describe only the
executed prefix. They are **not a completed avoidance pass**. SAT margins
are the vehicle driver's signed separating-axis rectangle margins; the
model driver uses signed rectangle distance. No obstacle margin is applicable
to no-target cruise or a run rejected before its first hold.

| Scenario | Executed / requested (s) | Minimum SAT margin (m) | Minimum road function margin | Sustained final cruise | Outcome |
| --- | --- | ---: | ---: | --- | --- |
| `straight_cruise` | 18.00 / 18 | — | — | Yes | Completed |
| `circular_cruise` | 18.00 / 18 | — | — | Yes | Completed |
| `straight_oncoming` | 2.20 / 18 | 0.971005 | 7.277633 | No | optimizationFailed |
| `circular_oncoming` | 2.20 / 18 | 1.168020 | 7.180001 | No | optimizationFailed |
| `varying_curvature_oncoming` | 0.00 / 18 | — | — | No | unsupportedReferenceJump |
| `estimated_straight_oncoming` | 1.05 / 18 | 23.994603 | 7.598373 | No | optimizationFailed |
| `estimated_circular_oncoming` | 1.05 / 18 | 24.048799 | 7.531961 | No | optimizationFailed |

The straight and circular cruise controls have maximum full-run speed errors
about 0.0018 and 0.0019 m/s; circular maximum lateral error is about 0.0028 m.
These successful no-target controls isolate the failures to more demanding
avoidance/reference/uncertainty conditions rather than basic speed holding.

The S-curve driver supplies a sampled polyline that the current recursive
reference check rejects with `unsupportedReferenceJump`. No physical hold
executes, so this trial does not measure S-curve avoidance. Subsequent
[failure analysis](CONTROLLER_FAILURE_CAUSES_20260925.md) confirms that the
controller supports an explicit varying-curvature reference profile: supplying
that missing reference passes the first frame. It also identifies a separate
missing target motion contract in this scenario. These are scenario interface
defects, not an observed collision or evidence that all S-curves are unsupported.

Both estimator-fed trials stop on the **first target-visible attempt** at
1.05 s. That failed attempt is retained in `failureContext` and `attempts`;
the completed-sample estimator metrics omit it and misleadingly report no
acquisition if read alone. At the failed attempt, published speed estimates
are approximately 10.03 m/s, speed-error radii approximately 30.03 m/s, and
course/yaw radii pi rad. The safety search rejects admission under those
published uncertainties. This does not establish estimator divergence:
completed-prefix estimates are finite and their reported truth-containment
checks pass. The first-publication admission remains unsuccessful.

The shell batch saves all seven MAT results and checkpoint tables before
MATLAB teardown reports `free(): chunks in smallbin corrupted` and exits
with **status 137**. This is not recorded as a clean batch exit. All seven
files were independently loaded and re-summarized in the persistent MATLAB
session; assertions confirmed exactly two completed/recovered no-target runs
and five incomplete avoidance runs. No simulation continuation is inferred
beyond the preserved traces.

Reproduce the two main campaign groups with an external output directory:

```matlab
addpath('scripts');
runControllerRecoveryValidation(outputDirectory, "exact");
runControllerRecoveryValidation(outputDirectory, "vehicle");
% Recompute reports from original MAT files without resimulating:
runControllerRecoveryValidation(outputDirectory, "summarizeExact");
runControllerRecoveryValidation(outputDirectory, "summarizeVehicle");
```

## Regression validation

All **92/92** selected `matlab.unittest` cases passed, with no failures or
incomplete cases. The selected files cover the controller, algorithm stress
regressions, circular trim, recovery rejection contracts, straight/circular
vehicle scenario interfaces, reactive target feedback and NRMM prediction.
`regression-results.csv` preserves the actual individual outcomes. Several
legacy vehicle tests intentionally assert rejection of nonzero model residuals;
passing those tests does not mean the nonlinear avoidance scenarios succeed.

Code Analyzer reports no code issues in the new driver. It emits an environment
notice that an old R2025b settings file cannot be read and defaults are used.
The 15 model runs record 12 frames exceeding the 50 ms hold period, including
cold initialization. MATLAB workloads overlap during part of the campaign;
these timing observations are diagnostic and are not a performance comparison
or real-time qualification.

## Longer-horizon nonlinear diagnostics

Two independent PassVeh14DOF diagnostics use 10 m/s ego and target speeds,
18 s requested duration, a 1.6 s nominal horizon (`horizonSteps=32`), a 30 s
search budget, and no enforced controller deadline. Road boundaries remain
at their scenario defaults. Both stop at **2.20 s**, with
`collisionAvoidanceController:optimizationFailed`; the fixed directions,
previous-plan directions and alternate directions all reject the solve.
Neither passes the target or recovers cruise. Positive recorded separation
before the stop does not establish a successfully completed encounter.

| Diagnostic | Executed / requested (s) | Minimum SAT margin (m) | Last-hold vx prediction error (m/s) | Last-hold vy prediction error (m/s) | Last-hold yaw-rate prediction error (rad/s) |
| --- | --- | ---: | ---: | ---: | ---: |
| Straight oncoming, 32 stages | 2.20 / 18 | 0.972454 | -0.068847 | 0.150082 | 0.085295 |
| Circular oncoming, 32 stages | 2.20 / 18 | 1.168281 | -0.140127 | 0.900486 | 0.755865 |

The errors are measured actual minus predicted successor values at the last
executed hold, directly from saved controller metadata and vehicle states.
They demonstrate model mismatch; they do not by themselves prove a unique
cause of infeasibility. The controller admits only zero declared plant
residuals, whereas this nonlinear plant is an independent experimental
model. Expanding the horizon and search time is insufficient in these two
trials. The scenario's existing default zero-residual controller is explicitly
treated as an uncertified nonlinear validation experiment.

Reproduction for each of `runOncomingVehicleAvoidanceScenario` and
`runCircularCenterlineStraightTargetAvoidanceScenario`:

```matlab
cfg = struct('controller', struct('horizonSteps',32), ...
    'solver', struct('certificateSearchTimeLimit',30, ...
                     'frameDeadlineSeconds',Inf));
r = runOncomingVehicleAvoidanceScenario(Duration=18, ...
    ReferenceSpeed=10, TargetSpeed=10, RecoveryWindow=2, ...
    ControllerConfiguration=cfg, Plot=false, Report=false, Progress=true);
```
