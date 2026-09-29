# Current slowest controller frame: measured time composition

Date: September 29, 2026. Production controller source: `bb5a340d1dd99dabcff2ba8e875562b3d7bbe391`. This investigation changes no production controller, configuration or test behavior.

## Frame and measurement scope

The maximum returned-call latency in the latest 14-case campaign is **5.019550 s**: reference speed **15 m/s**, `acceleratingTarget`, **frame 15**, simulated time **0.70 s**, prediction length **62 steps** (16 prefix plus 46 completion steps). The target starts at (24, 0), heading pi, speed 8 m/s and constant tangential acceleration 1 m/s squared. The collision buffer is 0.005 m, hold duration 0.05 s, and the shared soft improvement budget is 5 s. There is no observation noise or estimator. The original frame has nine secondary conic calls, zero primary/restoration calls, and a feasible returned plan. It ends at the time limit.

The original trace stores aggregate wall time and search outcomes, not per-phase timestamps. The phase table below is a new replay measurement of the reconstructed pre-frame state; it must not be presented as recovered timestamps from the original run.

All preceding 14 frames were reconstructed using their recorded measured ego states. Frames that originally returned their initial feasible witness skip unretained optimization attempts; other frames receive a 60 s budget to reproduce their completed accepted search. Every recorded applied input and CLF slack matches exactly; horizon length, absolute sample index and endpoint index also match. Original full-horizon trajectories were not separately logged, so equality of every historical horizon point cannot be independently checked. The resulting complete pre-frame state is saved in `captured-frame.mat`. The original 5 s configuration is restored for frame-15 latency replays. The production source hashes match the prior experiment snapshot.

Four uninstrumented and four phase-instrumented replays were run, each with its first run labeled warmup. The uninstrumented warmed median is **5.010805 s**; all warmed and all instrumented applied inputs match the original exactly. The uninstrumented warmup stops earlier and differs by 0.001953151 in maximum input component; its returned plan is still feasible. Original and replay search counts differ with timing: original nine calls, warmed uninstrumented eight/eight/nine, instrumented eight. Host load is not controlled; one MATLAB process was observed in the recorded host sample.

## Nonoverlapping wall-time composition

Representative instrumented replay **2** is the middle-duration warmed replay: **5.011209 s**, eight secondary solves. Timers cover exclusive coarse phases, including early returns through cleanup guards. The residual row includes controller input/output work, diagnostic bookkeeping and other unclassified overhead. Road/collision submeasurements below overlap these phases and are not added again.

| Phase | Seconds | Share |
|---|---:|---:|
| Secondary optimization | 3.863362 | 77.09% |
| Dynamics, Jacobians and constraint assembly | 0.629367 | 12.56% |
| Terminal correction | 0.271348 | 5.41% |
| Nonlinear candidate evaluation | 0.192091 | 3.83% |
| Initial witness revalidation | 0.023673 | 0.47% |
| Stage-cost cone construction | 0.013492 | 0.27% |
| Primary safety optimization | 0.000000 | 0.00% |
| Elastic completion restoration | 0.000000 | 0.00% |
| Input parsing, output construction and other overhead | 0.017876 | 0.36% |

Road-boundary row calculations total **0.024859 s (0.50%)**; collision geometry totals **0.118145 s (2.36%)** across 2426 safety-row evaluations. Removing road constraints is not supported as the main speed remedy by this measured composition. These direct geometry counters do not measure how removing constraints would change the optimizer's iterations or factorization.

Full measurements: [phase timing](CURRENT_SLOW_FRAME_20260929/phase-timing.csv), [per-iteration timing](CURRENT_SLOW_FRAME_20260929/iteration-timing.csv), [replays](CURRENT_SLOW_FRAME_20260929/profile-source-replays.json).

## Why the same frame keeps searching

The initial witness is already feasible: zero hard residual and zero safety slack. Its secondary cost is **191168.736225**, dominated (99.9322%) by the squared CLF relaxation penalty. The vehicle is avoiding the approaching target while displaced from the lane reference; zero safety slack does not imply zero lane/CLF cost. The new near-zero-objective exit therefore does not apply.

The trust radius resets to 0.5 at the start of this frame. Frame 14 had accepted a radius of 0.015625, but that information is not retained for the next call. The first six attempted improvements are rejected, mostly because their nonlinear endpoint lies outside the small terminal set. The permitted endpoint norm is 0.0009765625 in the controller's scaled state/input norm; this number is **not a distance in meters**. The following hard violations are likewise not all physical distances.

| Attempt | Trust radius | Hard violation after correction | Retained improvement |
|---:|---:|---:|---|
| 1 | 0.500000000 | 37.8637673 | no |
| 2 | 0.250000000 | 10.3610696 | no |
| 3 | 0.125000000 | 2.65437692 | no |
| 4 | 0.062500000 | 0.528458653 | no |
| 5 | 0.031250000 | 0.0707014171 | no |
| 6 | 0.015625000 | 0.0171603261 | no |
| 7 | 0.007812500 | 0 | yes |
| 8 | 0.009765625 | 0.00201002862 | no |
| 9 | 0.004882812 | not evaluated | no |

The first candidate also violates completion safety, but its terminal error is larger. For subsequent rejected candidates the terminal condition is the dominant recorded hard violation. On attempt 7, endpoint correction lowers the terminal norm from 0.037105015 to 0.000062387, finally admitting an improved witness. The accepted trust radius is 0.0078125, 64 times smaller than the initial radius. This is repeated expensive optimization followed by nonlinear rejection, rather than a long road-geometry calculation.

The retained cost is **189578.587371**, a **0.831804%** reduction. The seventh attempt finishes at approximately **4.470410 s** in the representative replay; roughly **0.540799 s** remains spent before returning. However, the current convergence rule also requires the nonlinear CLF slack to be no larger than the affine solver's slack plus 1e-5. Their values are **43.525778207164** and **43.524712653466**: gap **0.001065553698**, about 106.6 times that tolerance. Thus an accepted safe improvement is not considered converged. Attempt 8 is rejected; attempt 9 reaches the deadline during assembly in this representative replay. In the original run, attempt 9 reached a partial conic solve before the deadline. Both return the seventh attempt's command.

## Optimizer and profiler detail

A separate diagnostic run allows 60 s but caps work at the first seven outer attempts. This completes in **4.586878 s** and reproduces the original command exactly. Each secondary problem has **629 variables, 64 cones**. Across seven solves, the interior-point method performs **372 inner iterations**. All seven report flag -7: small search direction before the requested constraint/optimality conditions. Finite returned points are not labeled optimal; the nonlinear admission check rejects six and retains the seventh. No primary solve is needed because the feasible anchor already achieves zero primary safety cost.

| Attempt | Secondary seconds | Inner iterations | Exit flag |
|---:|---:|---:|---:|
| 1 | 0.539551 | 55 | -7 |
| 2 | 0.553382 | 57 | -7 |
| 3 | 0.744806 | 77 | -7 |
| 4 | 0.535445 | 57 | -7 |
| 5 | 0.402823 | 43 | -7 |
| 6 | 0.422662 | 45 | -7 |
| 7 | 0.352075 | 38 | -7 |

MATLAB profiler replay of the same seven-attempt diagnostic takes **5.244849 s**, with identical applied input. It is separate from the normal-budget phase table. The profiler attributes about 3.57 s to `coneprog`/`interiorPointMethod`; the interior routine does not expose enough children here to assign that whole time specifically to matrix factorization. Dynamics and derivative evaluation are the next visible computational group. Inclusive times below overlap and must not be summed.

| Profile function | Calls | Inclusive seconds |
|---|---:|---:|
| `coneprog` | 7 | 3.568920 |
| `interiorPointMethod` | 7 | 3.568321 |
| `nonlinearBicycleModel>nonlinearBicycleModel.sample` | 2280 | 1.202624 |
| `nonlinearBicycleModel>nonlinearBicycleModel.jacobian` | 15696 | 0.568997 |
| `solvePredictiveControl>localPolishEndpoint` | 7 | 0.409877 |
| `solvePredictiveControl>localSafetyRows` | 2080 | 0.243175 |
| `laneGeometry>laneGeometry.project` | 5223 | 0.042623 |

The evidence points to two next optimization candidates: retaining an effective trust radius across frames, and defining a useful-improvement stopping condition once an admissible witness is already available. Neither proposal is implemented or validated by this investigation. Any such change must preserve strict nonlinear witness admission; merely weakening the terminal condition had already regressed a previous experiment.

## Validation and artifacts

- All 14 reconstruction comparisons match recorded inputs and CLF slacks exactly, with matching horizons and absolute indices.
- Three warmed uninstrumented replays, four instrumented replays and both seven-attempt diagnostics return the original input exactly. Every issued replay plan has zero hard residual and zero safety slack. The differing cold uninstrumented warmup is retained in the records.
- Coarse phase totals reconcile to each measured wall time with a nonnegative overhead remainder. Source hashes match the previously tested controller. No production edits were made, and the 245-test suite was not rerun or claimed as a new check for this reporting-only task.
- The original 14-case campaign is not rerun here; these are fixed-frame replay and profiling measurements, not new closed-loop safety outcomes.

Executed commands:

```text
matlab -batch "run('/home/zai/.cache/collisionAvoidance/current-slow-frame-20260929-105333/capture_frame.m');"
python /home/zai/.cache/collisionAvoidance/current-slow-frame-20260929-105333/instrument.py
matlab -batch "run('/home/zai/.cache/collisionAvoidance/current-slow-frame-20260929-105333/replay_frame.m');"
python /home/zai/.cache/collisionAvoidance/current-slow-frame-20260929-105333/collect_report.py
git diff --check
```

Instrumentation is confined to an external snapshot and exported as [instrumentation.patch](CURRENT_SLOW_FRAME_20260929/instrumentation.patch). The report bundle contains source hashes, reconstruction comparisons, replay measurements, iteration timings, profiler summaries and executed scripts. Full MAT files, the original detailed profiler output and logs remain at:

`/home/zai/.cache/collisionAvoidance/current-slow-frame-20260929-105333`

[Raw artifact hashes](CURRENT_SLOW_FRAME_20260929/raw-artifact-hashes.json) identify the captured state, profiler output and original trace. No raw binary, controller source snapshot, dependency or unrelated working change is committed. No new Evidence ID is assigned.
