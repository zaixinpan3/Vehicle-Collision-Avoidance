# Return the first feasible controller plan

Date: September 29, 2026. Base commit: `3f3e4959478ff62ea8a4e2160e1e011dcb523a86`. The user specified that iteration is only for finding a feasible plan, and that a large lane-recovery cost is not a reason to continue after feasibility is established.

## Algorithm change

The controller now returns immediately after its original nonlinear admissibility checks succeed. A shifted or initialized witness needs no optimizer call regardless of its secondary cost or CLF slack. During restoration, the first feasible primary conic trajectory is returned before constructing the secondary problem. Every outer candidate is admitted on feasibility alone; it is not compared against an existing feasible plan for a better cost. A feasible candidate bypasses endpoint polishing, and polishing of an inadmissible endpoint stops once feasibility is reached.

The previous near-zero-secondary-cost gate and the affine/nonlinear CLF-slack agreement gate are removed from outer termination. Safety and lane/CLF objectives continue to guide search only while no witness exists. The former redundant-primary-solve shortcut is removed because a feasible baseline now exits before any subproblem is built. A finite primary result is checked even if its numerical exit flag is not optimal, while budget remains; strict nonlinear evaluation decides admissibility. Numerical solver tolerances and the two-stage search remain available for restoration. No controller mode, new tuning switch or solver method was added.

Admissibility is unchanged: exactly zero nonlinear hard residual and prefix safety slack within the existing shifted budget. Input amplitude/rate bounds, velocity/domain bounds, road and collision rows, hard completion and terminal membership are retained. The configured collision buffer is still 0.005 m. The existing PCBF permits positive-slack recovery prefixes; this change does not relabel those as collision-free. A positive-slack recovery test continues to verify that distinction and the shifted budget. All returned trajectories in this campaign have zero safety slack.

Termination is reported as `feasibleWitness`. The existing `search.converged` / `scvxConverged` field now means that the feasibility stopping target was met; it is not a claim of numerical optimality. Endpoint membership uses the actual terminal radius, with no further polishing merely to reach the former half-radius interior target. Reusing a validated nonlinear primary trajectory also avoids evaluating it twice.

## Previously slowest captured frame

The same reconstructed pre-frame state from the prior profiling task is used: 15 m/s accelerating target, frame 15 at 0.70 s, 62 prediction steps and 5 mm buffer. There is one warmup plus five timed replays.

| Measurement | Before | First feasible return |
|---|---:|---:|
| Warmed median | 5.010805 s | 0.025604 s |
| Optimizer calls | 8-9 in warmed replays | 0 |
| Returned secondary cost | 189578.587371 | 191168.736225 |
| Returned CLF slack | 43.525778207164 | 43.708026325354 |
| Hard residual / safety slack | 0 / 0 | 0 / 0 |

The original recorded frame took 5.019550 s. The new warmed median is 195.7 times smaller than the prior warmed median. The controller executes the revalidated shifted input exactly; it intentionally does not reproduce the later cost-optimized input (maximum component difference 0.001953151093). The high cost and positive CLF slack no longer block return. Timings are measured on this host, not a worst-case execution bound.

## Matched closed-loop campaign

The same fourteen fixtures are rerun at 8 and 15 m/s with prefix lengths 8/16, 50 ms holds, 5 mm buffer, default 5 s search budget, 400 inner/24 outer iterations, 3 s initial recovery horizon, 512-step maximum and default unlimited slew limits. Vehicle rectangles are 4.8 by 1.9 m; friction is 0.85; road boundaries are +/-4 m, with circular curvature 0.005 1/m. Recovery/circular request 40 holds, other cases request 160. Target initial states and every configuration are retained in the summaries and trace hashes. No observer, noise, delay or random draws are used.

Plant successors use independent `ode45` replay (relative/absolute tolerances 1e-11/1e-12), with 31 geometry samples per hold. Python independently checks rectangle distance, signed overlap and full-body road clearance. Measured computation latency is not injected into the simulated plant.

**13/14 cases complete**, producing **1600 holds** and 49600 geometry samples. **1597 holds invoke no optimizer**; the other three are scenario initialization. All **1587 subsequent holds finish within 50 ms** (largest 36.562 ms). All returned plans have zero hard residual and zero safety slack, and every one of 1600 trace checks confirms no outer iteration continues after an admitted candidate. There are 0 strict-overlap samples. 12/14 cases pass the full configured-clearance audit, with 6 samples below the full 5 mm margin.

| Speed | Scenario | Holds | First call, ms | Later median, ms | Later P95, ms | Solver calls, total | Min distance, m | Audit |
|---:|---|---:|---:|---:|---:|---:|---:|---|
| 8 | recovery | 40/40 | 415.523 | 19.077 | 24.596 | 0 | n/a | pass |
| 8 | circular | 40/40 | 361.430 | 13.838 | 19.309 | 0 | n/a | pass |
| 8 | brakingTarget | 160/160 | 365.985 | 3.825 | 21.422 | 0 | 4.100000 | pass |
| 8 | turningTarget | 160/160 | 68.251 | 6.934 | 33.555 | 0 | 4.262471 | pass |
| 8 | acceleratingTurn | 160/160 | 64.024 | 6.625 | 33.916 | 0 | 2.576377 | pass |
| 8 | oncoming | 160/160 | 1013.445 | 3.664 | 23.887 | 4 | 0.042653 | pass |
| 8 | acceleratingTarget | 160/160 | 1007.062 | 3.623 | 23.147 | 4 | 0.004809 | fail |
| 15 | recovery | 40/40 | 294.126 | 15.833 | 21.086 | 0 | n/a | pass |
| 15 | circular | 40/40 | 319.734 | 16.463 | 22.094 | 0 | n/a | pass |
| 15 | brakingTarget | 160/160 | 296.653 | 6.537 | 24.142 | 0 | 4.100000 | pass |
| 15 | turningTarget | 160/160 | 47.189 | 6.526 | 23.722 | 0 | 1.004002 | pass |
| 15 | acceleratingTurn | 160/160 | 47.560 | 6.626 | 24.612 | 0 | 1.262489 | pass |
| 15 | oncoming | 0/160 | n/a | n/a | n/a | 0 | n/a | fail |
| 15 | acceleratingTarget | 160/160 | 3173.710 | 6.640 | 24.856 | 10 | 0.038996 | pass |

The 15 m/s oncoming case still fails to initialize; this is search with no feasible witness available. Cold endpoint construction and initial avoidance search can exceed 50 ms. Fast later calls do not establish a universal real-time guarantee. The remaining intersample 5 mm shortfall is reported even though physical sampled distance remains positive. These dense samples are not a continuous-time safety proof.

- 15 m/s oncoming: 0 returned holds; failed call 5.014182 s at simulated 0.000 s; `collisionAvoidanceController:noFeasibleContinuation: No feasible nonlinear prefix and indefinite continuation were obtained (timeLimit).`.

## Validation and provenance

Final regression results contain **246 passing MATLAB cases** from nine suites, and **9 passing Python audit tests**. The initial full run passed 245/246; the one failure was an outdated requirement that lane following must invoke the optimizer. The test still verified zero slack and decreasing lane CLF. Its optimizer-call assertion was corrected to the requested first-feasible behavior, and the entire predictive-safety class was rerun successfully. Final results combine that targeted retry with the unchanged passing suites; the initial failure log and results are retained. No production executable changed between the full run and targeted retry.

Behavior coverage includes nontrivial-cost feasible initialization, nonzero-cost shifted plan execution with no optimizer calls, no continuation after the first admitted search candidate, positive-slack recovery labeling, cache/fresh equivalence and hard infeasibility rejection. The captured-frame replay additionally checks a large secondary cost and positive CLF slack while returning the exact already available input with zero solves.

Executed commands:

```text
matlab -batch "run('/home/zai/.cache/collisionAvoidance/first-feasible-20260929-110622/campaign.m');"  # initial nine-suite run; stale test expectation fails
matlab -batch "run('/home/zai/.cache/collisionAvoidance/first-feasible-20260929-110622/resume_campaign.m');"  # targeted retry, captured frame, fourteen scenarios
python -m unittest discover -s tests -p auditJointPredictiveSafetyTest.py
python /home/zai/.cache/collisionAvoidance/first-feasible-20260929-110622/analyze.py
git diff --check
```

Compact metrics, per-frame timing, final/initial tests, executed scripts, source manifests and raw-artifact hashes are in [the report bundle](FIRST_FEASIBLE_CONTROLLER_20260929/outcomes.json). Full traces and source snapshots remain outside the repository at:

`/home/zai/.cache/collisionAvoidance/first-feasible-20260929-110622`

Only scoped controller implementation/docs, behavior tests and report artifacts belong to the project commit. Unrelated observer/report changes, dependencies, binaries, source snapshots and agent instructions are excluded. No new Evidence ID is assigned.
