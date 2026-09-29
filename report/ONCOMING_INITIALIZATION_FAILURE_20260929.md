# Oncoming initialization failure diagnosis — September 29, 2026

The 15 m/s straight oncoming scenario is feasible under the existing controller constraints. The current search misses a repairable primary candidate: it checks that candidate before endpoint correction, then replaces it with a secondary cost solution and corrects only the replacement. An isolated experiment that corrects and admits the primary candidate first completes the same scenario without changing any constraint. Production controller files were not changed by this investigation.

The inspected production version is `2a606c8d60a44449d4368e05b9014df016d0d518` (`Execute the first feasible controller plan`). The earlier campaign failure took 5.014182 seconds and issued no input. This is initialization failure at time zero, not failure during the subsequent running stage.

## Scenario and initial guess

- Ego: position (0, 0) m, heading 0, speed 15 m/s, zero lateral velocity and yaw rate.
- Target: position (24, 0) m, heading pi, constant opposing speed 8 m/s, zero acceleration and sideslip.
- Both vehicle rectangles are 4.8 by 1.9 m, with zero rectangle offset. The initial body gap is 19.2 m, and the closing speed is 23 m/s. Unchanged straight motion reaches body contact after 0.834783 seconds.
- Straight road: centerline from (-100, 0) to (1000, 0) m, full-body lateral boundaries at +/-4 m. Collision buffer: 0.005 m. Friction coefficients: 0.85.
- Sample time: 0.05 s. Prefix: 16 holds / 0.8 s. Initial completion: 60 holds, making 76 holds / 3.8 s total. The 512-hold maximum is not reached.
- Default slew limits are unlimited; steering and braking amplitude limits, tire dynamics, positive-speed domain, full-body road constraints, endpoint membership, and infinite-suffix separation remain active.
- No observer, observation error, noise, delay, or random draws. Measured computation latency is not injected into the simulated motion.

`localSeed` rolls out lane feedback. Because the initial ego state already follows the lane and reference speed, this produces a straight trajectory ending at (57, 0) m. It satisfies endpoint membership and future separation *after* the encounter but overlaps the target along the finite completion. Its collision margin reaches -1.905 m. The prefix slack is zero because the encounter lies beyond the short prefix; the completion still requires hard collision satisfaction, so the seed is correctly rejected.

The constructed endpoint radius is 0.0009765625 in the controller's transformed eight-state norm. This is **not a distance in meters**. It couples pose, velocity, yaw rate, and final input memory. The tight endpoint makes the difference between an affine prediction and nonlinear rollout consequential even when the avoidance segment is already safe.

## Reproduction and stalled search

Diagnostics ran in a cache copy, with in-memory evaluation/solver logging. Instrumented durations are diagnostic measurements, not replacements for production timing statistics.

| Probe | Controller duration | Conic calls | Outcome |
|---|---:|---:|---|
| Original campaign, production | 5.014182 s | Not retained for the thrown frame | Initialization time limit |
| Instrumented unchanged search, 5 s budget | 5.013601 s | 14 | Initialization time limit |
| Instrumented unchanged search, 30 s budget | 10.903346 s | 48 | All 24 outer iterations exhausted; last status `safetySolveFailed` |

The first primary affine problem reports infeasibility. Elastic completion restoration supplies a candidate, and endpoint correction removes its terminal defect, but its collision margin remains -0.379001 m. The search accepts it only as an improved search anchor, never as an executable witness.

The third outer iteration improves the anchor again. Its hard residual is 0.267333, dominated by endpoint membership; collision margin remains -0.089945 m. Subsequent replacement candidates worsen the nonlinear residual and are rejected. Trust radius shrinks from 0.3125 at that accepted anchor to 0.000000372529 by the final extended-budget iteration. From iteration nine onward both primary and restoration subproblems report infeasibility. This is a stalled local search, not a demonstrated physically unavoidable encounter. Increasing the time budget alone does not fix it.

The 5-second diagnostic records 46 nonlinear evaluations, including primary checks and endpoint correction trials. The conic calls account for 2.179526 seconds of that instrumented call. The remaining time includes endpoint construction, assembly, nonlinear evaluations, corrections, and other orchestration; these residual components were not separately profiled here.

## The missed candidate

The unchanged search's sixth outer iteration, trust radius 0.09765625, contains a finite primary candidate with a successful conic exit flag:

| Quantity | Primary candidate | Secondary replacement | Primary after existing endpoint correction |
|---|---:|---:|---:|
| Nonlinear hard residual | 0.166889597 | 1.448045922 | 0 |
| Collision margin after the 5 mm buffer | +0.010215047 m | -0.014262971 m | +0.010215047 m |
| Primary road margin | +0.391580556 m | — | +0.391580556 m |
| Prefix safety slack | 0 | 0 | 0 |

All of the primary hard residual comes from the endpoint defect. Running the **existing** endpoint correction on this saved primary takes 0.076643 seconds in the offline probe and returns an admissible witness. Its finite-horizon collision margin and road margin remain unchanged because the corrected tail is after the encounter. Admission uses the unchanged exact `hard == 0` and existing PCBF slack-budget rule.

Current control flow in `controller/solvePredictiveControl.m`:

1. Evaluate primary inputs nonlinearly and return if they are already feasible.
2. If the endpoint still fails, build the secondary cost problem.
3. Replace primary inputs with the finite secondary solution.
4. In the outer loop, correct the endpoint of those replacement inputs.

This ordering gives a repairable primary candidate no endpoint-correction opportunity before replacement. In this iteration, the replacement also fails collision constraints and its correction still fails admission. Thus the controller does not continue after an admitted feasible witness; it loses a candidate **before** the existing repair could make it feasible.

Road constraints are not the rejecting constraint for the saved primary or the verified witness. These results do not establish that road constraints can never influence other iterations, but removing them is unnecessary to resolve this observed failure.

## Feasibility and ordering controls

Two offline controls establish that the original scenario and its unchanged constraints admit avoidance:

1. Reconstruct the first 76 executed inputs from the previous successful 15 m/s accelerating-target trace, then evaluate those inputs against the constant-speed oncoming target and the original endpoint. This returns zero hard residual and zero prefix slack. Independent ode45 propagation over all 76 holds, 31 samples per hold, has minimum body distance 0.117834 m and road margin 0.138193 m.
2. Correct the unchanged search's saved primary from iteration six. The same 76-hold propagation has minimum distance 0.015029 m and road margin 0.391368 m. No sampled body contact occurs. The full 5 mm buffer is satisfied at all audited samples.

These borrowed/saved-input controls are offline diagnostics; no preplanned trajectory or alternate controller mode was added to production.

A second isolated source copy adds only two statements before the existing primary feasibility test: call `localPolishEndpoint` and retain its corrected inputs. It then returns the corrected primary if admitted; otherwise the original secondary/restoration search proceeds. Instrumented single-call probes succeed with 30-second and 5-second budgets in 4.622821 and 4.022502 seconds, respectively. Both use 11 conic calls and return `primaryFeasible` at outer iteration six.

An **uninstrumented** closed-loop replay of this isolated ordering change, keeping the normal 5-second search budget, gives:

| Metric | Result |
|---|---:|
| Requested / completed holds | 160 / 160 |
| First initialization call | 4.004120 s |
| Subsequent maximum controller-call time | 0.044711 s |
| Subsequent median / 95th percentile | 0.006590 / 0.025307 s |
| Subsequent calls above 50 ms | 0 / 159 |
| Total conic calls | 11, all at initialization |
| Independent geometry samples | 4,960 |
| Minimum body clearance | 0.015029232 m |
| Minimum full-body road margin | 0.391368445 m |
| Strict collision-free and full-buffer audit | Passed |
| Target passed | Yes |

Python's independent rectangle geometry agrees with MATLAB. Every returned prediction has zero safety slack and satisfies nonlinear witness admission. This control verifies the concrete failure mechanism and a localized remedy for this scenario. It is not a fourteen-scenario regression of a production change, and initialization still takes seconds. The production controller remains at the inspected version.

## Methods and artifacts

Commands actually run:

```bash
matlab -batch "run('/home/zai/.cache/collisionAvoidance/oncoming-failure-20260929/diagnose.m')"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/oncoming-failure-20260929/counterfactual.m')"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/oncoming-failure-20260929/audit.m'); run('/home/zai/.cache/collisionAvoidance/oncoming-failure-20260929/primary_polish.m')"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/oncoming-failure-20260929/closed_loop.m')"
python scripts/auditJointPredictiveSafety.py /home/zai/.cache/collisionAvoidance/oncoming-failure-20260929 --files counterfactual-closed-loop.json --output /home/zai/.cache/collisionAvoidance/oncoming-failure-20260929/independent-audit.json
python /home/zai/.cache/collisionAvoidance/oncoming-failure-20260929/analyze.py
```

Independent ode45 settings are relative tolerance 1e-11 and absolute tolerance 1e-12. No controller unit suite was rerun because production executable files were unchanged; the new checks are failure reproduction, saved-candidate controls, a 160-hold isolated closed-loop simulation, independent Python audit, and collector assertions.

Compact exports and executed script copies are in [ONCOMING_INITIALIZATION_FAILURE_20260929/](ONCOMING_INITIALIZATION_FAILURE_20260929/). `summary.json` retains all solver flags, residuals, iteration decisions, and relevant scalar metrics. CSV files retain each nonlinear evaluation and the isolated replay timing. Instrumentation patches and the isolated ordering patch are explicitly labeled diagnostic artifacts. Raw MAT captures, full JSON traces, logs, and source copies remain at `/home/zai/.cache/collisionAvoidance/oncoming-failure-20260929/`; their hashes and the source manifest identify the inputs used. Setup retries caused by JSON null representations of infinite slew limits and missing instrumentation globals were corrected before the reported experiments; setup failure logs are not counted as controller feasibility outcomes.
