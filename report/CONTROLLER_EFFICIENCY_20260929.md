# Controller efficiency: objective bounds, sparse conic costs and retained evaluations

Date: September 29, 2026. One PCBF/CLF controller; exact observations; default collision buffer 0.005 m. Base commit: `fc777d841d761c0803e078ae98ec854614f123f8`. Final executed source hashes are in [after-manifest.json](CONTROLLER_EFFICIENCY_20260929/after-manifest.json).

## Findings and implemented changes

1. **Stop when improvement is already bounded.** The old slowest frame began with zero safety slack, zero hard residual, and zero secondary cost, yet still called both optimizers and polished an unusable candidate. Both objectives are nonnegative. The new `objectiveLowerBound` exit returns a nonlinear feasible witness when safety is exactly zero and its secondary cost is at most the existing `solver.optimalityTolerance` (1e-7 by default). This bounds any remaining secondary improvement absolutely. It does not accept positive safety slack or a positive hard residual. When the feasible anchor has zero safety but a nontrivial cost, the first optimization is also redundant: zero increments establish its primary optimum, so only the secondary optimization runs. Diagnostics report actual solver calls and do not invent an exit flag for a skipped solve.

2. **Remove a false need for hundreds of prediction steps.** `sin(pi)` roundoff gave a perfectly straight braking target a fictitious transverse acceleration. The terminal condition considers all future time; even a tiny nonzero quadratic coefficient eventually dominates. It therefore delayed endpoint admission to hundreds of steps. Target directions now use `sinpi`/`cospi`, and the straight endpoint calculation uses relative headings in the road frame instead of subtracting world-coordinate projections. No small physical acceleration is rounded to zero. Tests retain a nearby non-cardinal heading and a real acceleration of 1e-18 m/s squared, and verify rotation invariance. Fresh side-braking scenarios now initialize with 68 steps at 8 m/s and 76 at 15 m/s; the prior zero-buffer run used 357 at 8 m/s and failed to establish an endpoint within 512 at 15 m/s.

3. **Keep the quadratic objective local to stages.** One horizon-wide norm cone coupled all stages in the conic factorization. It is replaced by one small squared-norm epigraph per stage plus one for the CLF penalty, constrained to sum below the total objective epigraph. For residual blocks `r_i(z)`, each cone `||[2*r_i(z); t_i-1]|| <= t_i+1` is exactly `t_i >= ||r_i(z)||^2`. Minimizing the sum has the same objective and projected feasible set as the original single cone. The default internal linear solver remains unchanged. This adds auxiliary variables but greatly improves the measured fixed-matrix solves. Numerical exit flags and nonlinear feasibility still govern what was actually achieved.

4. **Reuse evaluations that cannot have changed.** Endpoint polishing only changes the last at most 20 inputs. It now reuses the unmodified prefix and checks input amplitude/rate limits before an expensive rollout. Under exact measured-successor and input-memory equality, shifting a retained witness reuses absolute-time stage evaluations; only an appended endpoint input needs simulation. Any changed measured state still triggers full nonlinear revalidation. Endpoint membership, initial physical bounds, complete input and slew limits are checked on every returned plan. Independent `ode45` successors generally differ from RK4, so exact shifted reuse is intentionally uncommon in this campaign. No measurement tolerance was introduced to bypass validation.

5. **Share one improvement budget.** Formulation, restoration, secondary optimization and endpoint polishing use the time remaining since controller entry, instead of each optimizer receiving a fresh five seconds. Expired budgets prevent new improvement work; an already available feasible witness can be returned. Initial witness construction/revalidation still has to finish. In-flight factorization or integration is not preemptible. MathWorks documents that `coneprog` checks `MaxTime` after iterations, so it can overrun; this remains a soft budget, not a hard real-time guarantee. [coneprog documentation](https://www.mathworks.com/help/optim/ug/coneprog.html)

6. **Batch geometry and preserve equivalent headings.** Four road-corner projections are evaluated together, with preallocated rows and no unrequested road Jacobian. All road and collision rows remain. Target observation yaw differences are compared modulo one turn: crossing +/-pi preserves the same fixed forecast epoch; a genuine trajectory change is still rejected. This fixes the prior accelerating-turn interruption at 5.85 s.

The original strict nonlinear witness admission, lexicographic safety preference, input bounds, road constraints and half-radius terminal polishing target remain. There is no controller-method selector or alternative production controller.

## Same captured frame: controlled before/after replay

The input is the complete pre-frame state captured during the previous slow-frame investigation: 8 m/s side-braking target, frame 12, time 0.55 s, **346 prediction steps and zero additional buffer**. Keeping the old state, horizon and configuration isolates avoided work from the shorter fresh horizon. The before source is the base commit; the after source is the final snapshot. The initial replay is treated as warmup in each group.

| Measurement | Before | After |
|---|---:|---:|
| Timed replays after one warmup | 3 | 7 |
| Median controller time | 9.552558 s | 0.127629 s |
| Conic calls per replay | 2 | 0 |
| Returned hard residual / safety slack / cost | 0 / 0 / 0 | 0 / 0 / 0 |

The measured median speedup is **74.8x**; all applied inputs match the captured command exactly (maximum difference 0). The final controller still fully revalidates this captured changed-state trajectory. Raw diagnostics are in [before-replays.json](CONTROLLER_EFFICIENCY_20260929/before-replays.json) and [after-replays.json](CONTROLLER_EFFICIENCY_20260929/after-replays.json). Host load is not controlled and timings are not a certified worst-case execution bound.

## Fixed secondary conic problems

Three original secondary subproblems were saved before optimization: the 346-step captured braking frame and an oncoming subproblem at each speed. The oncoming matrices are the first reached secondary problems after any necessary restoration, not necessarily outer iteration one. Each formulation was run twice with the same 3 s `MaxTime`. These are matrix microbenchmarks, not controller timings.

| Fixed problem | Original auto, seconds | Stage cones, seconds | Stage-cone flags | Maximum raw linear / cone residual |
|---|---:|---:|---|---:|
| braking346 | 4.708 / 4.587 | 0.950 / 0.755 | 1 / 1 | 6.75e-06 / 0 |
| oncoming8 | 3.090 / 3.101 | 0.345 / 0.356 | -7 / -7 | 9.6e-11 / 6.15e-12 |
| oncoming15 | 3.096 / 3.106 | 0.256 / 0.230 | 1 / 1 | 3.85e-06 / 2.29e-07 |

Every original `auto` solve in this comparison reached its time limit (flag 0). The split 8 m/s problem still stopped with flag -7 (small search direction before the requested tolerances), despite small primal residuals; it is not labeled optimal. Other split cases returned flag 1, but raw residuals are reported independently. Algebraic equivalence does not imply identical finite-iteration outputs or certify nonlinear feasibility. The controller reevaluates the nonlinear trajectory before admission.

Trying `normal`, `schur` and `augmented` directly on the original cones did not provide a reliable success/performance improvement. The 346-step `normal` factorization took about 24.6 s despite a 3 s limit. No backend choice was adopted. The solver chooses its internal factorization for the new sparse structure. [MathWorks conic algorithm](https://www.mathworks.com/help/optim/ug/cone-programming-algorithm.html) Full flags, residuals and objectives: [original](CONTROLLER_EFFICIENCY_20260929/linear-solver-benchmark.json), [stage cones](CONTROLLER_EFFICIENCY_20260929/split-cone-benchmark.json).

## Final closed-loop campaign

Executed with MATLAB R2026a Update 3: 8 and 15 m/s reference speeds; prefix lengths 8 and 16; 50 ms holds; 5 mm additional collision buffer; default 5 s soft improvement budget; 400 inner and 24 outer iteration limits; 3 s initial recovery horizon and 512-step maximum. The straight road is +/-4 m wide; the circular centerline has curvature 0.005 1/m. Ego and target rectangles are 4.8 by 1.9 m; tire friction is 0.85. Default slew limits are unlimited.

Targets start at (24, 0), heading pi and speed 8 m/s. Turning cases use sideslip atan(-0.1*1.6); accelerating cases use tangential acceleration 1 m/s squared. The side-braking target starts at (24, 6), speed 2 m/s, acceleration -1 m/s squared; its signed-velocity model continues through zero. Recovery starts with 0.01 m lateral offset; circular motion starts at its trim. No observer, observation noise, delay or random draws are used.

Each returned hold is propagated independently by `ode45` (relative tolerance 1e-11, absolute tolerance 1e-12), with 31 geometry samples across the 50 ms hold. Python independently checks rectangle distance, signed overlap and whole-body road clearance, including edge extrema on a curved road. Latency is measured but not injected into plant propagation. Recovery/circular request 40 holds; other cases request 160.

**13/14 scenarios completed**, with 1600 returned holds and 49600 replay geometry samples. 13/14 both completed and retained strictly positive sampled separation (or have no target); 11/14 pass the full audit including the configured buffer. There are 0 strict-overlap samples and 10 samples below the full 5 mm margin. 413 returned holds exceed 50 ms. Initialization failures and incomplete trajectories are not counted as successes.

| Speed, m/s | Scenario | Holds | Later median, ms | Later P95, ms | Maximum, ms | >50 ms | Min distance, m | Full audit |
|---:|---|---:|---:|---:|---:|---:|---:|---|
| 8 | recovery | 40/40 | 228.135 | 296.678 | 2912.672 | 40 | n/a | pass |
| 8 | circular | 40/40 | 14.383 | 21.107 | 379.461 | 1 | n/a | pass |
| 8 | brakingTarget | 160/160 | 3.975 | 23.082 | 377.387 | 1 | 4.100000 | pass |
| 8 | turningTarget | 160/160 | 6.968 | 34.328 | 70.503 | 1 | 4.262471 | pass |
| 8 | acceleratingTurn | 160/160 | 6.727 | 35.601 | 65.370 | 1 | 2.576377 | pass |
| 8 | oncoming | 160/160 | 50.527 | 5008.919 | 5015.835 | 82 | 0.004967 | fail |
| 8 | acceleratingTarget | 160/160 | 52.221 | 5008.874 | 5013.338 | 89 | 0.004437 | fail |
| 15 | recovery | 40/40 | 202.617 | 341.301 | 2717.045 | 36 | n/a | pass |
| 15 | circular | 40/40 | 16.582 | 22.060 | 333.776 | 1 | n/a | pass |
| 15 | brakingTarget | 160/160 | 6.922 | 24.684 | 298.231 | 1 | 4.100000 | pass |
| 15 | turningTarget | 160/160 | 6.723 | 25.872 | 49.502 | 0 | 1.004002 | pass |
| 15 | acceleratingTurn | 160/160 | 6.871 | 24.607 | 49.226 | 0 | 1.262489 | pass |
| 15 | oncoming | 0/160 | n/a | n/a | n/a | 0 | n/a | fail |
| 15 | acceleratingTarget | 160/160 | 97.348 | 5008.946 | 5019.550 | 160 | 0.005179 | pass |

Later timing excludes the first returned call. Maximum includes it; failed-call latency is listed separately below. A small full-scenario median can hide several slow seconds around an encounter. Dense sampled clearance is evidence, not an all-continuous-time proof. The 5 mm constraints remain at nodes and midpoints; their full value need not hold between these times.

- 15 m/s oncoming: 0 returned holds, failure at simulated 0.000 s after 5.005 s for the failed call: `collisionAvoidanceController:noFeasibleContinuation: No feasible nonlinear prefix and indefinite continuation were obtained (timeLimit).`.

### Matched 5 mm encounter windows

The prior focused 5 mm campaign requested 60 holds at 8 m/s for each encounter. The following comparison uses exactly the first 60 returned holds of the final campaign; these are closed-loop comparisons, so optimized trajectories can differ.

| Scenario | Before later median, s | After later median, s | Before first-2-s median, s | After first-2-s median, s | Before / after solver calls |
|---|---:|---:|---:|---:|---:|
| oncoming | 5.250 | 2.064 | 5.886 | 4.546 | 708 / 522 |
| acceleratingTarget | 5.333 | 2.558 | 5.790 | 4.624 | 554 / 538 |

The longest returned final call was **5.019550 s**, 15 m/s `acceleratingTarget`, frame 15 at simulated 0.700 s, horizon 62, 9 conic calls, termination `timeLimit`. Its returned hard residual was 0 and safety slack was 0. Its complete outer-step diagnostics are retained in [outcomes.json](CONTROLLER_EFFICIENCY_20260929/outcomes.json); this is not a new phase profiler measurement.

The remaining difficult work is search for a strictly admissible nonlinear completion during the close encounter. The very small certified terminal set, inaccurate affine candidates and invalid tire/model trial points can require repeated restoration, trust-region reduction and endpoint correction. The original merit rule sometimes makes little useful progress. The new lower-bound exit cannot certify that a substantially positive lane/CLF cost is already optimal. Returning an available feasible witness at the budget is legitimate, but it does not make a five-second call real time.

Historical zero-buffer full-campaign measurements are preserved separately in [legacy-zero-buffer-comparison.json](CONTROLLER_EFFICIENCY_20260929/legacy-zero-buffer-comparison.json). They are context, not an identical-configuration speedup experiment. Both accelerating-turn cases previously stopped after 117 holds at the yaw wrap. The new final campaign tests that representation fix over all 160 holds.

## Rejected experiments and limits

- Stopping endpoint polishing at the full terminal radius instead of half radius looked cheaper but lost necessary interior headroom. An intermediate 8 m/s oncoming run stopped after 19/160 holds. The original half-radius target was restored; a focused replay then completed 60/60 holds for both encounters. No safety constraint was loosened.
- Adding normalized secondary cost to the search merit was tested in a separate snapshot. The oncoming trial stopped after 54/60 holds with no feasible continuation and a model-domain search failure. This does not establish that the applied plant state entered that invalid domain. The accelerating-target trial was deliberately interrupted and is not a completed measurement. The merit change was rejected; production keeps the original acceptance rule.
- Backend-only factorization changes were benchmarked but not adopted. Early diagnostic attempts and interrupted campaigns are retained externally; they are not merged into final scenario totals.
- No universal 50 ms guarantee, observation-error robustness claim or continuous-time safety guarantee follows from these offline results. Further reduction of hard-encounter search time requires improving completion feasibility/search conditioning or a separately validated stopping bound; simply relaxing endpoint accuracy already regressed closed-loop behavior.

## Validation and artifact scope

All **245 selected MATLAB tests pass** (0 failed, 0 incomplete); **9 Python audit tests pass**. The nine suites cover terminal continuation, predictive safety, Fiala tires, controller configuration/source budget, longitudinal loads, input geometry and observer integration/model behavior. New behavior tests cover objective-bound stopping, skipping a redundant safety solve, cache/fresh equivalence, exact/near-cardinal motion, tiny genuine acceleration, rotation invariance and target yaw wrap. The core source budget remains unchanged. Full-repository unrelated tests were not claimed.

Executed commands and sources:

```text
matlab -batch "run('/home/zai/.cache/collisionAvoidance/controller-efficiency-20260929-050658/before_replay.m');"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/controller-efficiency-20260929-050658/solver_benchmark.m');"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/controller-efficiency-20260929-050658/split_cone_benchmark.m');"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/controller-efficiency-20260929-050658/headroom_retest.m');"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/controller-efficiency-20260929-050658/merit_trial.m');"  # separate rejected prototype; second case interrupted
matlab -batch "run('/home/zai/.cache/collisionAvoidance/controller-efficiency-20260929-050658/campaign.m');"
python -m unittest discover -s tests -p auditJointPredictiveSafetyTest.py
python /home/zai/.cache/collisionAvoidance/controller-efficiency-20260929-050658/analyze.py
git diff --check
```

Compact results, per-frame timing CSVs, source manifests, fixed-problem measurements, executed script exports and diagnostic patches are stored beside this report. Raw traces, captured matrices, snapshots and logs remain outside the repository at:

`/home/zai/.cache/collisionAvoidance/controller-efficiency-20260929-050658`

The raw-artifact manifest also identifies the reused captured pre-frame MAT file. Rejected prototype details are in [intermediate-findings.json](CONTROLLER_EFFICIENCY_20260929/intermediate-findings.json). Source snapshots and dependencies, binaries, instruction files and unrelated observer/report working changes are excluded from the project commit. No new Evidence ID is assigned to this report bundle.
