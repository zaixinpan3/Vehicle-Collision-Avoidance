# Correct primary endpoint defects before secondary refinement — September 29, 2026

The production controller now applies its existing endpoint correction to a primary conic candidate before replacing that candidate with a secondary cost solution. If the corrected candidate satisfies the original nonlinear hard constraints and PCBF slack budget, it returns immediately. Otherwise the existing restoration/secondary search continues. The applied input comes from the corrected candidate, and the outer loop reuses its validated evaluation. No controller mode, configuration field, or constraint was added or relaxed.

The change addresses the mechanism established in [the failure diagnosis](ONCOMING_INITIALIZATION_FAILURE_20260929.md). The previously failed 15 m/s constant-speed oncoming scenario now completes 160 holds and passes the target. Its minimum independently replayed body clearance is 0.015029231541 m, above the unchanged 0.005 m buffer. All fourteen fixtures completed in this campaign; sampled collision and margin outcomes are stated separately below.

## Implementation and regression coverage

- `controller/solvePredictiveControl.m`: after the primary nonlinear rollout, run `localPolishEndpoint` before testing primary admission; use its corrected inputs if admitted. A candidate already feasible exits the existing correction immediately. Correction trials and conic calls continue to share the same controller-call time budget.
- `tests/nonlinearPredictiveSafetyTest.m`: add `correctedPrimaryAvoidanceReturnsBeforeCostRefinement`, exercising the 15 m/s oncoming fixture. Assert the returned control matches its plan, nonlinear hard residual and prefix slack are zero, collision/road margins are positive, endpoint membership passes, and the final admitted primary returns after one conic call without a secondary replacement. The unit test uses a 30-second search budget to test algorithm behavior independently of machine speed; the campaign retains the default 5-second budget.
- `controller/PCBF_CLF_ARCHITECTURE.md`: document correction before primary replacement and unchanged full nonlinear admission.

Finite primary candidates with non-optimal conic exit flags retain their existing nonlinear validation path. Invalid-domain candidates retain the existing fallback. Prefix recovery semantics, hard completion, 5 mm collision buffer, full-body road bounds, input amplitude/slew limits, terminal construction, and first-feasible stopping remain in effect. No witness is returned solely because an affine subproblem reports success.

## Validation configuration and scope

The campaign uses the same fourteen exact-observation fixtures as the prior first-feasible campaign: reference speeds 8 and 15 m/s; 8/16-hold prefixes; 50 ms sample time; maximum horizon 512; initial recovery duration 3 seconds; 5-second soft search budget, 400 inner and 24 outer iterations; default unlimited slew limits; 4.8 by 1.9 m rectangles; friction 0.85; +/-4 m full-body road boundaries. Circular curvature is 0.005 1/m. Recovery and circular fixtures request 40 holds; side braking, turning target, accelerating turn, constant-speed oncoming, and accelerating oncoming request 160. There is one controller and no randomized input.

Independent plant propagation uses ode45 with relative tolerance 1e-11 and absolute tolerance 1e-12, with 31 geometry samples per executed hold. The Python audit independently reconstructs the target and body rectangles. For the circular road, the whole-body radial margin includes edge extrema. No observer, measurement error, noise, or delay is included. Measured computation latency is not applied to the simulated plant. Sampled replay does not establish a continuous-time certificate.

247 MATLAB regression results and 9 Python audit tests pass. The initial nine-suite isolated run passed 204/247 results; 43 observer-related checks errored because the source copy omitted the external YALMIP/SeDuMi directory. The controller suites and new regression passed in that run. Linking the existing repository `solver/` directory into the isolated copy and retrying the two observer suites produced the combined 247 passing results. Executable source did not change between runs. Initial errors and final results are retained; the dependency tree is not committed.

## Closed-loop results

| Metric | Result |
|---|---:|
| Completed fixtures | 14 / 14 |
| Completed fixtures without sampled collision | 14 |
| Full configured-buffer and road audits passed | 13 / 14 |
| Executed holds / geometry samples | 1760 / 54560 |
| Strict overlap samples | 0 |
| Returned maximum hard residual / prefix safety slack | 0 / 0 |
| Subsequent running holds | 1746 |
| **Maximum running controller-call time, excluding initialization** | **37.130 ms** |
| Running calls above 50 ms | 0 |
| Largest initialization call | 3.746770 s |
| Replay samples below the full 5 mm buffer | 6 |

Timing covers the entire controller call, including model construction, solving, nonlinear checks, and output construction. Initial calls are reported separately from running calls. A soft budget is not a hard real-time guarantee.

| Speed (m/s) | Fixture | Holds | Complete | Minimum body distance (m) | Initialization (s) | Maximum running call (s) | Buffer shortfall samples |
|---|---|---:|---|---:|---:|---:|---:|
| 8 | recovery | 40 | Yes | N/A | 0.726096 | 0.035755 | 0 |
| 8 | circular | 40 | Yes | N/A | 0.380937 | 0.021022 | 0 |
| 8 | brakingTarget | 160 | Yes | 4.100000000 | 0.417998 | 0.026107 | 0 |
| 8 | turningTarget | 160 | Yes | 4.262471482 | 0.069881 | 0.035606 | 0 |
| 8 | acceleratingTurn | 160 | Yes | 2.576376711 | 0.065699 | 0.037130 | 0 |
| 8 | oncoming | 160 | Yes | 0.042652650 | 1.071688 | 0.026228 | 0 |
| 8 | acceleratingTarget | 160 | Yes | 0.004808675 | 0.985510 | 0.025934 | 6 |
| 15 | recovery | 40 | Yes | N/A | 0.314323 | 0.021550 | 0 |
| 15 | circular | 40 | Yes | N/A | 0.314234 | 0.022108 | 0 |
| 15 | brakingTarget | 160 | Yes | 4.100000000 | 0.292211 | 0.026814 | 0 |
| 15 | turningTarget | 160 | Yes | 1.004002140 | 0.048963 | 0.027540 | 0 |
| 15 | acceleratingTurn | 160 | Yes | 1.262488813 | 0.048101 | 0.027704 | 0 |
| 15 | oncoming | 160 | Yes | 0.015029232 | 3.746770 | 0.028208 | 0 |
| 15 | acceleratingTarget | 160 | Yes | 0.044619958 | 2.990288 | 0.034237 | 0 |

- 8 m/s acceleratingTarget: 6 replay samples below the configured 5 mm buffer; minimum clearance 0.004808675146 m; strict overlap samples 0.

The previously failed 15 m/s oncoming fixture admits the corrected primary at outer iteration six after 11 total conic calls. Its successful initialization takes 3.746770 seconds. This fixes the search-order failure but does not make initialization a 50 ms operation. No later call in that fixture invokes the optimizer. The final control is taken from the corrected witness, and subsequent holds use nonlinear continuation validation as before.

The shortfall counts above describe configured margin at the sampled independent plant trajectory; they are distinct from strict overlap. The present change targets primary-candidate handling, and its collision constraint sampling schedule is unchanged.

## Reproduction and retained artifacts

```bash
matlab -batch "run('/home/zai/.cache/collisionAvoidance/primary-endpoint-fix-20260929-133506/campaign.m')"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/primary-endpoint-fix-20260929-133506/resume_campaign.m')"
python tests/auditJointPredictiveSafetyTest.py
python /home/zai/.cache/collisionAvoidance/primary-endpoint-fix-20260929-133506/analyze.py
python /home/zai/.cache/collisionAvoidance/primary-endpoint-fix-20260929-133506/collect.py
```

The first command preserves the initial dependency-related test errors; the second retries the observer suites with existing dependencies and executes all fourteen scenarios. The Python analyzer checks geometry, traces, and stopping behavior. Collector assertions require zero returned hard residual/slack, first-feasible termination, no secondary solve after an admitted primary, all 247 passing results, and unchanged captured source hashes. Source/raw manifests and scoped whitespace checks were verified before commit.

Compact results, timing CSVs, initialization traces, initial/final test records, hashes and executed script copies are in [PRIMARY_ENDPOINT_CORRECTION_20260929/](PRIMARY_ENDPOINT_CORRECTION_20260929/). Raw logs, traces and the isolated source copy remain at `/home/zai/.cache/collisionAvoidance/primary-endpoint-fix-20260929-133506`. The baseline project commit is `684ff18fa96836923b98148b4dfa71267161011f`. Unrelated estimator/report edits, agent instruction files, native solver dependencies, raw snapshots and generated binaries are excluded from the implementation commit.
