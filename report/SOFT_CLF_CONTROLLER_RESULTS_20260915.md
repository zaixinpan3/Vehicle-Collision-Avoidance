# Optimized soft CLF in the single-solve controller

Experiment date: September 15, 2026. This revision replaces the hard CLF in
commit `8da38bc374abfc0d9beaa4b7642660ae8a34130f` with an optimized nonnegative
slack. It retains one optimization per sample, hard physical constraints,
strict solver-status handling, and immediate simulation failure on an
unsuccessful solve. There is no runtime plan checker, backup, or retry.

## Implemented behavior and claim

The three decisions are held steering, held braking ratio, and CLF slack.
The slack enters only the sampled CLF norm cone and its quadratic objective
penalty. Its coefficient in every collision, road, domain, tire-slip, input
and slew row is zero. Target-free operation removes only obstacle rows.
Actuators receive the first two coordinates; the complete decision and slack
are available in the planning output.

The constraint is

\[
\|R\hat e^+\|\le\sqrt a\|R\hat e\|+\epsilon_{\rm num}+\delta,
\qquad\delta\ge0,
\]

with penalty `cfg.clf.relaxationWeight * delta^2`. Slack has units of
\(\sqrt V\). The existing uncertainty argument now gives

\[
V^+\le cV+B+S(\delta),\qquad
B=\frac{c}{c-a}\sigma^2,\quad
S(\delta)=\frac{c}{c-a}\delta(2\sigma+\delta).
\]

Positive slack can permit a tracking-error increase. Penalization alone does
not prove zero slack or unconditional asymptotic convergence. Metadata and
the exact-state experiment report slack, its dissipation allowance, the
slack-dependent residual, and the unrelaxed residual separately. The full
derivation is in [SINGLE_SOLVE_CBF_CLF.md](https://github.com/zaixinpan3/Vehicle-Collision-Avoidance/blob/8529584774ffd17b2cae7d2fb18aac219b2fde13/controller/SINGLE_SOLVE_CBF_CLF.md).
Guarantees remain conditional on successful solves, the declared affine plant,
valid information boxes and execution assumptions. No recursive-feasibility
or nonlinear physical-vehicle guarantee is added.

The formerly unused default penalty 0.01 allowed slow path recovery: a
120-hold clear-road probe with initial errors `[.05;.002;-.05;0;0]` ended with
error norm 0.01061. The same probe with weights 1, 100 and 10000 ended at
0.009614, 0.0002286 and 0.001155 respectively. The default is now 100.
This is a finite tuning experiment, not a general optimal-weight claim.
An initial controller regression run with weight 0.01 failed only the
120-hold recovery threshold; the failure is retained in the experiment cache.

## Avoidance experiment

`runStraightControllerRerun(SampleCount=300,Seed=20260914)` ran all eight
independent trials. A failed trial saves its executed prefix and raises an
error; the campaign then starts the next independent trial. It never applies
a replacement command to the failed trial.

| Trial | Executed holds | Outcome | Minimum sampled clearance margin (m) |
| --- | ---: | --- | ---: |
| Exact stationary | 7 | Infeasible at 0.7 s | 4.34952 |
| Exact oncoming | 29 | Infeasible at 2.9 s | 8.54776 |
| Exact crossing | 300 | Completed; diagnostics pass | 9.26966 |
| Bounded stationary | 5 | Infeasible at 0.5 s | 5.94700 |
| Bounded oncoming | 29 | Infeasible at 2.9 s | 8.77761 |
| Bounded crossing | 0 | Initial infeasibility | No executed interval |
| Range-gated exact oncoming | 42 | Infeasible at 4.2 s | 10.94243 |
| Range-gated actual-NRMM oncoming | 41 | Infeasible at 4.1 s | 12.93829 |

All failures report Clarabel primal-infeasible status. No failed trial is
classified as completed avoidance. The 0.1 s basic experiments use 8 m/s
reference speed, road limits ±5 m, and 0.25 m clearance. Bounded ego radii are
`[.05;.05;.005;.05;.02;.005]`; target radii are
`[.1;.1;.1;.1;.05;.05;.01;.01]`. Target jerk amplitudes are `[.1;.1]` m/s³,
yaw acceleration amplitude is 0.05 rad/s², and frequency is 1 rad/s.
Range-gated experiments use 10 m/s reference speed, 30 m range, initial target
position `[100;.8]` m and velocity `[-10;0]` m/s. The estimated trial uses
actual NRMM outputs.

The soft CLF resolves a focused conflict: at 8 m/s with a stationary target
9.8 m ahead, a positive optimized slack allows braking. The initially zero
cruise-error value becomes positive at the next sample. Independent dense
hold measurements satisfy both swept separation and the sampled barrier.
Overlapping targets and collisions between sampled endpoints remain infeasible.

It does not resolve the campaign's remaining hard-barrier problem. Offline
reconstruction of the first failed exact stationary/oncoming programs gives
the following necessary rows, with a negligible steering coefficient and an
exactly zero slack coefficient:

| Failure | Required row | Required braking ratio | Allowed minimum |
| --- | --- | ---: | ---: |
| Stationary, 0.7 s | `0.0416871 * beta <= -0.0767845` | −1.84193 | −1 |
| Oncoming, 2.9 s | `0.0416871 * beta <= -0.1157784` | −2.77732 | −1 |

Minimizing each row over the entire actuator box still leaves violations
0.03510 m and 0.07409 m. This establishes incompatibility independently of
the CLF and the numerical optimizer. The geometric one-hold barrier does not
anticipate braking sufficiently from these trajectories; it is not a
velocity-dependent viable safe set. A future redesign would need to address
that hard constraint, rather than increase CLF slack or execute an infeasible
command. The bounded-crossing result also retains the known conservatism of
using independent initial upper and final lower barrier bounds.

## Cruise, dissipation and timing

Separate `runExactStateRecursiveFeasibilityScenario` runs use seed 20260912.
These timings were collected before starting the full regression process.

| Run | Holds | Median / maximum frame (ms) | 100 ms overruns | Maximum slack-dependent CLF residual |
| --- | ---: | ---: | ---: | ---: |
| Exact cruise | 900 | 3.7265 / 19.788 | 0 | −6.90e−6 |
| Exact crossing | 900 | 4.2830 / 19.052 | 0 | −2.11e−6 |
| Perturbed exact cruise | 300 | 3.7040 / 5.321 | 0 | −5.71e−6 |
| Small-noise perturbed cruise | 100 | 3.8515 / 5.901 | 0 | −1.31e−4 |

Both perturbed runs start with `[.05;.002;-.05;0;0]` error. The small-noise
ego box is `[.001;.001;.0001;.001;.001;.0001]`. The exact 300-hold run ends with
error norm 0.00035675; the final speed is 8.00035675 m/s. These are practical
recovery measurements, not exact asymptotic convergence. Positive unrelaxed
residuals remain recorded: up to 5.51e−6 in the perturbed exact run and
2.15e−8 in the crossing run.

Profiler instrumentation confirms 20 controller calls, 20 formulations and
20 native optimizer calls over 20 holds. The instrumented run is excluded
from runtime qualification; profiling itself produced a 144.378 ms frame
and overlapped the regression process. A separate fresh MATLAB process
completed 20 cruise holds but took **929.007 ms on its first frame**, giving
one 100 ms overrun. It ran after the full suite finished. Warm timing therefore
does not establish cold-start real-time operation, WCET or delayed-actuation
safety. Simulation computation times are measured, not applied as plant delay.

## Validation and reproducibility

The full suite passes **567/567 tests**, with zero failed or incomplete cases
and 367.60 seconds of test time. The 37 controller regressions pass, including
optimized penalty sensitivity,
the slack-dependent information-box bound, unchanged hard safety constraints,
single solver dispatch with and without targets, and stopping after an injected
failure. Factory `checkcode` reports zero findings in all six edited MATLAB
files. The full suite ran using `matlab -batch` with `runtests('tests')` and
`assertSuccess(results)`; focused development checks used the MATLAB MCP.

Original MAT/JSON/CSV outputs, the penalty sweep, failure-row reconstruction,
profiler data, analyzer results and regression logs are retained at
`/home/zai/.cache/collisionAvoidance/soft-clf-single-solve-20260915/`.
These are local experimental outputs; the committed report is their summary.
Historical hard-CLF results remain unchanged in
[SINGLE_SOLVE_CONTROLLER_RESULTS_20260915.md](SINGLE_SOLVE_CONTROLLER_RESULTS_20260915.md).
