# Longest controller frame: measured bottlenecks

Prepared September 28, 2026. Production source: `b0cc119324b5a8def8af47103970ad00bb8385b0`; input experiment report: `CURRENT_CONTROLLER_RERUN_20260928.md` at `206eee312492489efafaad7d668937eea3ea41f5`. Production controller and configuration files are unchanged by this investigation.

## Selected frame and faithful recovery

The maximum over all **1,760 recorded calls** is the default 15 m/s **oncoming scenario, frame 17 (one-based), simulation time 0.80 s**. The original controller call took **5.077416 s**, with an **81-stage horizon (4.05 s)**, 16 sequential-convexification iterations and **32 solver calls: 16 primary LPs and 16 secondary QPs**. Termination was `timeLimit`, with all LP/QP exit flags equal to 1 and no caught exceptions. This is approximately 102 control periods of 50 ms.

The initial ego state `[X,Y,psi,vx,vy,r]` is `[11.738948343529342, 1.5084658683291206, 0.24612904281513504, 14.835939995928964, -0.3485578895154901, -0.24302950870231513]` in SI units. The target is at `(17.6,0)` m, facing pi radians and moving at 8 m/s toward the ego; target acceleration and sideslip are zero. The road is straight with lateral boundaries at +/-4 m. Vehicle rectangles are 4.8 by 1.9 m, and the additional collision buffer is **zero**. Observations are exact; no random draws are used.

The trace did not store the entire warm-start trajectory. `captureLongestControllerFrame` reconstructs frames 1--17 from their recorded measured states and original iteration counts, propagates the target recursively, and saves the complete pre-frame-17 warm state. During reconstruction only, the wall deadline is disabled and each call is capped at its recorded iteration count. Applied inputs match every original prefix frame with **maximum error exactly zero**; solver-call counts and acceptance sequences also match. The recovered frame takes 5.062 s. This establishes recovery of the original numerical work, rather than a cold-start approximation.

MATLAB R2026a on an AMD Ryzen 7 7800X3D, Linux. Host load and clock frequency were not controlled; these are local diagnostics, not a hardware-independent timing guarantee.

## Wall time by phase

Three unprofiled production replays with a fixed 16-iteration budget take **5.013940, 5.063789 and 5.018255 s**. A separate replay using the original five-second deadline takes **5.084266 s**, again returning the same 16-iteration result with `timeLimit`.

Lightweight `tic/toc` instrumentation was added to an **isolated copy** of `solvePredictiveControl.m`. Following one warmup, three fixed-work runs take 4.984995, 5.021457 and 5.013216 s. All reproduce the entire recovered input trajectory exactly and retain the same 32 solver calls. Their mean, **5.006556 s**, breaks down as follows; the rows do not overlap:

| Phase | Mean seconds | Share | Work |
| --- | ---: | ---: | --- |
| Secondary QP | 2.693748 | 53.80% | 16 `quadprog` calls |
| Model, constraint and objective assembly | 1.600796 | 31.97% | 16 complete sequential-step builds |
| Nonlinear candidate evaluation | 0.594776 | 11.88% | Initial anchor plus 16 candidates |
| Primary safety LP | 0.098850 | 1.97% | 16 `linprog` calls |
| Remaining controller and loop work | 0.018386 | 0.37% | Residual wall time |

Assembly includes variational integration, geometry rows, sparse matrices and LP-option construction. Evaluation includes nonlinear integration, geometry checks, terminal residuals and cost. QP-option construction and time outside these intervals remain in the residual. The first five rejected steps account for **1.774385 s** on average, excluding the initial anchor evaluation.

Each LP has **741 variables, 492 equality rows and 1,969 general inequality rows**, plus variable bounds. The QP adds the lexicographic safety-cap row, reaching **1,970 inequalities**. The Hessian has 2,361 nonzero entries; the equality matrix has 3,327. QP iterations range from **25 to 60 per call**, totaling **774**; LP iterations range from 7 to 11. The nonlinear trajectory initializes the outer algorithm, but `quadprog` is called with an empty initial point and no carried primal/dual solver state. A trajectory warm start therefore does not establish an inner-QP warm start.

## Where assembly and evaluation spend time

MATLAB's function profiler was run separately with the same fixed 16 iterations. It takes **6.408300 s**, approximately 27.7% above the unprofiled median. The following times are **inclusive profiler measurements**, not additive components of the original 5.077416 s:

| Function | Calls | Inclusive profiled seconds |
| --- | ---: | ---: |
| `quadprog` | 16 | 2.764233 |
| `nonlinearBicycleModel.sample` | 5,346 | 2.547623 |
| `nonlinearBicycleModel.derivative` | 64,153 | 1.200418 |
| `nonlinearBicycleModel.jacobian` | 31,104 | 1.143150 |
| `modifiedFialaTire.evaluate` | 95,258 | 0.902903 |
| `localSafetyRows` | 5,379 | 0.636652 |
| `modifiedFialaTire.affineModel` | 31,104 | 0.627742 |
| `predictiveSafetyGeometry.dualLinearization` | 5,379 | 0.362485 |
| `laneGeometry.project` | 29,636 | 0.153173 |
| `linprog` | 16 | 0.141747 |

The dynamics, tire and Jacobian entries overlap through their call hierarchy; they must not be summed. Of the sampled-flow total, 1.891095 s occurs during sequential-step assembly and 0.656528 s during nonlinear evaluation. Assembly makes 2,592 half-step flow calls (`16*81*2`); evaluation makes 2,754 (`17*81*2`). Each half hold uses three RK4 substeps; each variational RK4 stage evaluates the nonlinear derivative and analytic Jacobian. This explains the tens of thousands of repeated tire evaluations without invoking numerical finite differences of the full dynamics.

All collision **and** road safety-row work together accounts for approximately **9.94%** of profiled controller time. Road geometry alone is a subset. However, road constraints also contribute **1,304 of the 1,969 LP inequalities**: eight corner/boundary rows at each of 163 node/midpoint locations. Collision rows contribute 652, and other terminal/CLF rows contribute 13. The direct geometry measurement cannot establish how much removing road rows would reduce the QP's linear algebra. That would require a separate ablation, which was not performed here. Removing direct road-geometry work alone cannot recover a 50 ms runtime.

The QP's internal `ipqpsparse` consumes 2.750621 s of the 2.764233 s profiled `quadprog` time. It is shipped as a protected `.p` file, so this profile does not resolve its internal factorization, search-direction and line-search costs. No more specific internal bottleneck is claimed.

## Why the outer loop runs so long

The solver resets the trust radius to **0.5 on every frame**, including frames initialized from a previous trajectory. On this frame the shifted anchor already has nominal safety slack 1.031216e-6 and hard residual zero. Nevertheless, the first proposed changes are much too large for the linear approximation. The first five candidates are rejected; the radius halves repeatedly. The baseline anchor is unchanged during those rejections, so the same nominal dynamics and geometry are rebuilt for iterations 1--6 even though the changing trust radius primarily changes the bounds.

| Iteration | Trust radius | Accepted | Nonlinear safety slack | Hard residual | Returned nonlinear CLF minus QP CLF |
| --- | ---: | --- | ---: | ---: | ---: |
| 1 | 0.5 | No | 35.463625 | 41.855304 | Not relevant while infeasible |
| 2 | 0.25 | No | 1.343782e-3 | 14.505513 | Not relevant while infeasible |
| 3 | 0.125 | No | 3.396254e-4 | 3.261559 | Not relevant while infeasible |
| 4 | 0.0625 | No | 8.571102e-5 | 0.601501 | Not relevant while infeasible |
| 5 | 0.03125 | No | 2.216851e-5 | 0 | Not relevant while infeasible |
| 6 | 0.015625 | Yes | 6.288814e-6 | 0 | 6.320426e-4 |
| 7 | 0.0078125 | Yes | 2.164831e-6 | 0 | 1.440255e-4 |
| 8 | 0.01171875 | Yes | 3.679058e-6 | 0 | 3.678205e-4 |
| 15 | 0.002471924 | Yes | 1.067821e-6 | 0 | 1.795507e-5 |
| 16 | 0.003707886 | Yes | 1.235267e-6 | 0 | **4.193743e-5** |

Starting at iteration 6, every accepted candidate satisfies the configured 1e-5 nominal safety/hard tolerances. The remaining convergence blocker is the additional condition:

```matlab
solution.clfSlack <= step.clfSlack + cfg.solver.feasibilityTolerance
```

At iteration 16 the nonlinear CLF slack is **23.50548392162227**, while the QP's linearized slack is **23.50544198419343**. Their difference **4.193743e-5** still exceeds **1e-5**. This is a CLF linearization-consistency check; it is not a requirement that CLF slack be zero. Likewise, the internal termination label `zeroSlack` does not mean every slack is literally zero. Numerical safety tolerance also does not prove the user's strict-positive physical distance requirement; this profiling task does not change the previous collision audit.

The acceptance and trust-radius rules use different improvement measures. Acceptance allows lower full cost once the candidate's safety/hard merit is within tolerance, while the radius ratio uses **safety + 10*hard only**. At accepted iteration 6, safety rises from 1.031216e-6 to 6.288814e-6 while CLF slack falls from 24.130085 to 24.000972. The safety-only ratio is **-169.77**, so the radius still halves. Later accepted iterations alternate between safety-only ratios around 0.76--0.78 (radius multiplied by 1.5) and negative ratios (radius halved). This produces the observed shrinking oscillation despite continuing CLF improvement.

A diagnostic run removes the wall deadline but retains the original maximum of 24 outer iterations. It converges at **iteration 19**, after **38 solver calls and 5.839235 s**. Final safety slack is 1.032279e-6, hard residual zero, CLF slack 23.453591, and the CLF consistency gap is **5.942032e-6**, finally below tolerance. No solver failure occurs. The five-second limit is checked between complete iterations, so it is neither a preemptive deadline nor the required 50 ms control-period limit.

## Implications and scope

The measured causes are repeated expensive QPs, repeated full variational-model construction, five initial rejected steps, and continued CLF consistency refinement after nominal feasibility. Even one average assembly costs approximately **100 ms**, one average QP **168 ms**, and one nonlinear evaluation **35 ms** on this recovered frame. Reducing only the number of outer iterations would therefore still leave a substantial runtime problem.

The evidence supports investigating reuse of unchanged linearizations across rejected steps; consistent merit and convergence criteria with an explicit CLF-model tolerance; retaining a useful trust radius across frames; and exploiting the QP's stage structure and repeated solves. Horizon length and inner-solver iteration counts also deserve controlled experiments. These are follow-up hypotheses, not implemented changes or validated speedups. Any stopping-rule or constraint change needs renewed geometry and closed-loop validation.

This task adds only diagnostic helpers and reports. It does not remove road constraints, add controller modes, change tolerances, or alter production controller behavior.

## Reproduction and artifacts

Run from the repository root, using the raw input traces retained from the preceding experiment:

```bash
matlab -batch "addpath('scripts'); captureLongestControllerFrame('/home/zai/.cache/collisionAvoidance/current-controller-rerun-20260928-1959','/home/zai/.cache/collisionAvoidance/longest-frame-profile-20260928');"
matlab -batch "addpath('scripts'); profileCapturedControllerFrame('/home/zai/.cache/collisionAvoidance/longest-frame-profile-20260928/captured-frame.mat','/home/zai/.cache/collisionAvoidance/longest-frame-profile-20260928');"
python scripts/instrumentControllerFrameTiming.py controller/solvePredictiveControl.m /home/zai/.cache/collisionAvoidance/longest-frame-profile-20260928/instrumented/solvePredictiveControl.m
matlab -batch "addpath('scripts'); timeCapturedControllerFrame('/home/zai/.cache/collisionAvoidance/longest-frame-profile-20260928/captured-frame.mat','/home/zai/.cache/collisionAvoidance/longest-frame-profile-20260928/instrumented','/home/zai/.cache/collisionAvoidance/longest-frame-profile-20260928/helper-validation');"
```

The initial instrumented measurements used external `timed.m`; the reusable timing helper was subsequently executed as an additional validation. The primary tables above consistently use the first three warmed instrumented measurements. The deadline-free solve is a diagnostic intervention; it is not a rerun of the entire closed loop with a changed production deadline.

`LONGEST_CONTROLLER_FRAME_PROFILE_20260928/` contains the selected frame/configuration, top-20 ranking with total frame count, prefix verification, phase summaries, all timed iteration diagnostics, profiler function totals, analyzer findings and technical hashes. Full recovered warm state (`captured-frame.mat`), profiler workspace (`function-profile.mat`), full ranking and raw logs remain under `/home/zai/.cache/collisionAvoidance/longest-frame-profile-20260928/`. MATLAB JSON exports encode nonfinite values as `null`; in configuration this includes infinite slew limits and the disabled diagnostic deadline.

Validation: all recovered prefix inputs match exactly; fixed-work full input trajectories and solver-call counts match with and without instrumentation/profiling; all measured LP/QP calls succeed; diagnostic MATLAB helper analysis is clean; the instrumented solver retains the 12 existing sparse-indexing notices. Python instrumentation parses and its anchors match the executed source. All 12 tracked production controller/configuration files match the stated production commit byte for byte. No full regression suite was repeated because production code was unchanged.
