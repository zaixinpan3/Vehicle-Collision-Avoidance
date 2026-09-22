# NRMM/VFFM implementation review and warmed simulation validation

Date: September 22, 2026. Tested controller commit:
`7ec600053b2da40c77f3060a964b9ad3ea211410`.

The declared-model avoidance campaign completes all 12 diagnostic trials with
positive sampled footprint gaps. The strict 50 ms campaign completes only
6 of 12 trials: straight stationary, circular stationary and circular crossing
reject their initial frame in both repetitions. No rejected command is applied.
The smallest refined clearance is only **0.0927376 mm**. Physical quadratic-road
boundaries still prevent terminal certification, and three scenario types
exceed the optional heading diagnostic envelope. These findings prevent a
claim of real-time, physically robust road-constrained avoidance.

The controller and configuration were not changed during this review.
The work makes the independent footprint audit reusable and updates the
analysis script to retain failed attempts instead of aborting their summary.
The machine-readable [results and technical hashes](NRMM_VFFM_SIMULATION_REVIEW_20260922.json)
retain all measured outcomes.

## Implementation review

The reviewed pipeline agrees with the current
[mathematical specification](../controller/NRMM_VFFM_INITIALIZATION.md):

1. `targetPrediction.nrmmFlow` propagates constant acceleration/sideslip
   motion, using a stable sinc displacement and an explicit stopping policy.
   Production reference generation uses the consistent Cartesian
   `nominalFlow` interface. Bounded Cartesian `finiteFlow` remains the hard
   safety predictor; NRMM reference generation does not remove uncertainty.
2. `laneGeometry.normalRoadChart` intersects route normals with valid
   quadratic segments, computes width/midpoint derivatives and checks chart
   regularity. Unbounded sections explicitly use a unit normalization.
3. `solveHardCbfClf.vffmReference` compares ego and target at identical future
   times, retains target acceleration/turning and road-width derivatives, and
   distinguishes course from body heading using the anchor sideslip.
4. Passing ordinates are frozen at the predicted conflict. Gaussian centers
   and chart normalization remain time dependent. This documented design
   choice differs from following the obstacle's lateral position at every
   instant. Both joint side assignments are fitted with terminal equalities;
   physical base-row feasibility precedes support residual in candidate ranking.
5. Fitted footprints initialize analytic separation directions. Those
   directions stay fixed in the full control-sequence SOCP. A new admission
   requires the final optimizer and independent verifier; seeds are never
   issued. Continuation carries previously optimized certificates.

The unit suite includes independent ODE integration, small-curvature limits,
braking stops, quadratic chart derivatives, rotated frames, multiple targets
and finite-difference checks of moving-reference derivatives. This review
found no new discrepancy in those tested constructions. Two joint side
assignments remain an incomplete search and do not establish feasibility for
arbitrary traffic. Gaussian superposition is a reference preference, not a
moving-boundary fluid solution or a collision certificate.

## Experiment protocol

- MATLAB R2026a Update 3; all timed experiments run serially in a separate
  `matlab -singleCompThread` process after the full regression finished.
  No other task-controlled MATLAB computation ran concurrently with timing.
- Ego reference speed 8 m/s; 50 ms prediction/execution holds; 1.6 s nominal
  performance horizon, with encounter completion extending the initial plan.
  Replayed conflict frames have 96 prediction holds.
- Straight and circular roads, curvature 0 and 0.01/m; stationary, oncoming
  and crossing target scenarios. Two repetitions per scenario and budget.
  Every measured trial requests 600 holds (30 simulated seconds).
- Diagnostic frame deadline 30 s versus strict deadline 0.05 s; certificate
  search limit 30 s; exact sensing, zero declared plant residual, seed
  20260912. Physical road boundaries and optional state/slip bounds disabled.
- The crossing target is at route station 15 m with normal offset -7.5 m
  and normal speed 4 m/s in the circular case. The legacy straight-crossing
  target starts at [15,-4] m and crosses at 32 m/s. Its cruise counterfactual
  is collision free; that case is a non-conflict control, not avoidance evidence.
- Two 140-hold warmup runs per scenario exclude **1,680 warmup holds**.
  Prepared-input replay excludes **20 warmup calls**, then measures 22 calls
  per fixture. Encounter initialization in a measured run remains included.
- Frame timing includes input preparation and the controller, but excludes
  simulated plant integration and offline audits. Failed attempts are retained.
  Finite measurements do not provide a worst-case execution-time guarantee.

## Closed-loop outcomes

The minimum body gaps below come from 5 ms reconstruction with 0.1 ms refinement
near the closest approaches in the first diagnostic repetition. Independent
SAT signs agree with signed rectangle-distance signs. All six reconstructions
match saved affine endpoints exactly. All actual avoidance cases have a
colliding cruise counterfactual; the straight-crossing exception is identified.

| Road and target | Diagnostic completion | Strict 50 ms completion | Refined minimum body gap | Largest diagnostic frame |
| --- | ---: | ---: | ---: | ---: |
| Straight, stationary | 2/2 | 0/2 | 0.0000927376 m | 51.498 ms |
| Straight, oncoming | 2/2 | 2/2 | 0.0380147 m | 31.971 ms |
| Straight, crossing (non-conflict control) | 2/2 | 2/2 | 9.51948 m | 20.587 ms |
| Circular, stationary | 2/2 | 0/2 | 0.0994193 m | 55.566 ms |
| Circular, oncoming | 2/2 | 2/2 | 0.134196 m | 33.207 ms |
| Circular, crossing | 2/2 | 0/2 | 0.155002 m | 53.314 ms |

Diagnostic runs execute 7,200 holds with all hard certificates verified.
Six frames exceed 50 ms, all initial admissions. Strict runs execute 3,600
holds and retain six rejected first-frame attempts, giving 3,606 measured
attempts. All six failures explicitly report complete-frame deadline excess;
no hold is executed in those runs. There are no retained-incumbent solver
failures in this campaign.

| Measured population | Calls | Median | Maximum | Over 50 ms |
| --- | ---: | ---: | ---: | ---: |
| Diagnostic, all frames | 7,200 | 7.237 ms | 55.566 ms | 6 |
| Diagnostic, admission | 12 | 41.7725 ms | 55.566 ms | 6 |
| Diagnostic, active continuation | 694 | 12.211 ms | 25.695 ms | 0 |
| Diagnostic, cruise | 6,494 | 7.228 ms | 13.548 ms | 0 |
| Strict, all attempts | 3,606 | 7.171 ms | 55.193 ms | 6 |
| Straight stationary replay | 22 | 50.169 ms | 59.589 ms | 12 |
| Circular crossing replay | 22 | 51.0895 ms | 55.119 ms | 17 |

The longest measured replay is 59.589 ms: input preparation 2.227 ms,
aggregate formulation/initialization 42.589 ms, native solve 7.112 ms and
remaining orchestration/certification 7.661 ms. The longest closed-loop frame
is 55.566 ms: 2.289, 37.311, 8.385 and 7.581 ms respectively. These are
existing coarse runtime categories; aggregate formulation must not be
misreported as isolated VFFM generation time. Initial admission remains the
observed bottleneck; continuation met 50 ms in this finite campaign.

The 30 existing variation fixtures all admit, including 28 with active targets
and two outside the sensing range. They vary mirrored curvature, ego and target
speeds, target station and initial lateral offset. These fixtures informed
prior heuristic development and are not held-out validation. Their timings
are not used to qualify real-time behavior.

## Remaining constraints and physical scope

**Quadratic road containment is not end-to-end supported.** A separate
straight-road test enables boundaries at y = +/-5 m and uses a 30 s deadline.
It executes zero of two requested holds and fails with:
`The recursive cruise certificate requires an unbounded road-free reference domain.`
The new chart utility does not remove the explicit terminal-set restriction
in `hardEncounterBarrier`. General intersection-union containment is also
outside the implementation.

**The straight stationary clearance is too small to support robustness claims.**
At t = 2.3502 s, refined body gap is 9.273756688224745e-5 m. The 0.2 m initializer
allowance shapes the reference and is not a final hard clearance. Finite
positive samples and a node certificate do not prove continuous-time safety
or tolerance to sensing, tracking, integration or vehicle-model error.

**The optional model envelope is exceeded.** Maximum saved-node absolute
heading errors are 0.555012 rad for straight oncoming, 0.511306 rad for circular
oncoming and 0.511301 rad for circular crossing, versus a 0.4 rad diagnostic
limit. State/slip ranges are not enforced as hard constraints in this setup.
The affine plant can satisfy its own certificate while these diagnostics fail.
No nonlinear-vehicle validity follows from these experiments.

The closed-loop target fixtures have constant Cartesian velocity; accelerating
and turning NRMM motion is exercised by analytical/reference unit tests, not
by a new closed-loop turning-target campaign here. The estimator is not in
these exact-state closed loops. No native binary was rebuilt in this review;
prior native parity results retain their original implementation-report scope.

## Checks, artifacts and reproduction

- Full `runtests('tests')`: **792/792 passed**, zero failed/incomplete,
  510.793052 s aggregate duration. The MCP response timed out after 300 s;
  MATLAB continued and saved the results. A later MCP call independently
  loaded `full-tests.mat` and passed `assertSuccess`.
- Factory Code Analyzer: no new findings across nine reviewed files; the
  pre-existing `FNDSB` sparse-index advisory remains at solver line 279.
  The initial analyzer invocation also reported a missing user settings file;
  factory-setting results were collected separately without changing preferences.
- The reusable analyzer was checked against the prior campaign and reproduces
  12 diagnostic completions, seven strict completions and five rejected attempts
  among 4,205 strict timings. It also handles zero-execution state serialization.
- Python compilation, final diff inspection and `git diff --check` passed.
  An unused initial assignment in the promoted audit helper was removed after
  its run; this does not change the numerical reconstruction.

Run the unchanged scenario campaign from the repository root:

```matlab
addpath('scripts');
runFixedDirectionValidation(outputDirectory, Mode="replay");
runFixedDirectionValidation(outputDirectory, Mode="campaign");
runFixedDirectionValidation(outputDirectory, Mode="sensitivity");
auditFixedDirectionCampaign(outputDirectory);
```

The replay mode uses the explicitly configured saved fixture directory; its
default is `/home/zai/.cache/collisionAvoidance/longest-frame-profile-20260921`.
The current raw results, driver, regression MAT/JSON, analyzer output and road
boundary rejection probe are retained at
`/home/zai/.cache/collisionAvoidance/nrmm-vffm-validation-20260922/`.

```bash
python scripts/analyzeFixedDirectionValidation.py \
  /home/zai/.cache/collisionAvoidance/nrmm-vffm-validation-20260922 \
  report/NRMM_VFFM_SIMULATION_REVIEW_20260922.json --tests full-tests.json
```

No initialization parameters were tuned against this rerun. The next priorities
supported by these observations are fresh-admission latency, a meaningful hard
clearance/inter-sample policy, and completion of road-constrained terminal
certification with explicit physical-model validity requirements.
