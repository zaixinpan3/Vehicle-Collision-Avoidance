# Quadratic input effort and CLF relaxation

The online objective is

\[
J=h\sum_{j=0}^{N-1}u_j^{\mathsf T}R_u u_j+\rho\delta^2,
\qquad R_u=\operatorname{diag}(w_\phi/s_\phi^2,w_a/s_a^2).
\]

Here `u = [frontWheelSteeringAngle; longitudinalAcceleration]`, `h` is the
control period, `sPhi` is the steering limit and `sA` is the largest absolute
acceleration limit. The existing `clf.frontWheelSteeringAngleWeight` and
`clf.longitudinalAccelerationWeight` supply positive input weights. They
also retain their role in Riccati certificate synthesis. Defaults are both
one; `clf.relaxationWeight = 100` weights squared relaxation.

Only the performance head, stages `0` through `N-1`, has an input cost. The
complete braking/steering continuation remains a hard safety witness without
an input penalty. Penalizing its required stopping effort could favor early
braking even at nominal cruise. No desired acceleration, steering reference,
sampled LQR target or input-rate cost is introduced.

The continuous CLF constraint remains
`LfV + LgV*u0 <= -alpha*V + delta`, with `delta >= 0`. Collision, road,
physical, model-domain, terminal-rest and acceptance constraints are unchanged.
The condensed Hessian has `2*h*Ru` head blocks, a `2*rho` slack coefficient,
and zero continuation blocks. All linear coefficients and the objective
constant are zero. The sparse stage lift preserves exactly this objective;
no native solver rebuild, second optimization or cone change is required.

`inputEffortCost`, `clfRelaxationCost` and `jointObjectiveValue` expose the
cost split. `inputDeviationCost` remains a compatibility alias for
`inputEffortCost`. Given a returned input plan, reconstructing its smallest
feasible nonnegative slack remains valid because the squared slack penalty
is increasing on the nonnegative domain.

A finite squared slack penalty permits trading CLF violation for input
reduction; it is not equivalent to first minimizing slack and then input.
Positive head weights remove ambiguity in the applied controls on a fixed
convex feasible domain, although the unpenalized tail may remain nonunique.
The objective does not impose sampled CLF decrease, establish an intersample
safety certificate, or prove asymptotic recovery for the physical plant.
A raw input penalty also does not explicitly compensate a nonzero constant
model bias or prescribe a curved-road equilibrium input.

Behavioral regression checks cover nominal straight-cruise preservation,
small over/underspeed corrections without predicted overshoot, the squared
cost scale, absence of a continuation cost, and agreement with independent
`quadprog` solutions. The existing hard-row, sparse-lift and terminal-policy
checks continue to apply. Closed-loop measurements are recorded separately
from these model and optimization checks.

## Validation on September 6, 2026

The final full working-tree suite ran 379 cases: 371 passed and 8 failed,
including 7 incomplete cases. All 66 controller, continuous-CLF, recovery
and sparse-QP cases passed. The eight failing cases were rerun against an
unchanged pre-edit source snapshot and reproduced the same failures:
six estimator integration cases reject unsupported terminal uncertainty;
a pre-existing untracked stationary-pose test requests an unsupported zero
cruise reference; another untracked test expects a curved-motion uncertainty
rejection that the baseline does not issue. These existing issues remain
outside the objective change. The complete suite is therefore not green.

The initial focused run had two numerical assertion failures. Independent
QP costs near 91089 differed by 0.000309 (relative error 3.39e-9); the check
now combines absolute and relative tolerances. The terminal-policy test
now checks the optimized terminal velocity residual separately (below
1e-7) and evaluates exact rest invariance at zero velocity. This preserves
the distinction between numerical feasibility and analytic invariance.
All eight edited MATLAB files pass factory Code Analyzer checks with no
reported issues. The manuscript builds successfully with `latexmk`.

## Thirty-second physical comparison

Two exact-state PassVeh14DOF crossing-target trials use the previous
30-second protocol: straight and radius 400 m, reference speed 15 m/s,
target speed 8 m/s, 50 m perception range, 0.05 s control period,
24 head stages, longitudinal input gain 0.8 and RNG 2026 (twister).
Road samples through 400 m are preserved exactly and appended through
1000 m. The prior 30-second slack-only sources match all pre-edit MATLAB
files byte for byte. Old 10-second target-free trials are reused only as
historical scenario counterfactuals; no new matched target-free trial is
claimed. Each new avoidance trial completed 600 commands, with no
fallback, no observed sampled contact and positive sampled road margin.

Maximum errors over every state sample in 9–10 seconds:

| Scene | Speed (m/s) | Lateral (m) | Heading (rad) | Recovery |
|---|---:|---:|---:|---|
| Straight | 0.657847 | 0.00070452 | 0.00019416 | Speed fails; lateral/heading pass |
| Arc | 0.721030 | 0.11377057 | 0.00658030 | Speed fails; lateral/heading pass |
| Limit | 0.5 | 0.2 | 0.02 | All samples must pass |

Both trials enter all three tolerances at 9.40 s and remain inside them
through the observed 30 s endpoint. This does not satisfy the complete
9–10 s window or establish asymptotic convergence.

Final 25–30 s RMS comparison (old slack-only to new quadratic objective):

| Scene | Speed (m/s) | Lateral (m) | Heading (rad) | Acceleration step RMS (m/s squared) |
|---|---:|---:|---:|---:|
| Straight | 0.132093 → 0.177398 | 0.0171933 → 1.85796e-09 | 0.00299539 → 1.79023e-10 | 8.41788 → 0.000199018 |
| Arc | 0.1364 → 0.109641 | 0.0898924 → 0.0428722 | 0.00661583 → 0.00160034 | 8.70751 → 0.301459 |

The straight initial acceleration changes from -4.561428 to approximately
-9.18e-7 m/s squared. Its late lateral and heading errors approach numerical
zero, but its mean speed error remains -0.1774 m/s; speed RMS is higher than
in the old oscillating trace. On the arc, lateral and heading RMS decline
by about 52% and 76%, respectively, while steering still switches. Arc
steering-step RMS rises from 0.012272 to 0.015457 rad even though its input
range narrows. Thus reduced acceleration variation must not be described
as elimination of every control oscillation or exact nominal recovery.

| Scene | Minimum SAT margin (m) | Minimum road margin (m) | Median / p95 / max call (ms) | Calls above 50 ms |
|---|---:|---:|---:|---:|
| Straight | 1.088251 | 7.641714 | 25.521 / 36.738 / 109.440 | 1 / 600 |
| Arc | 1.510876 | 7.463593 | 28.530 / 38.764 / 73.950 | 1 / 600 |

The 109.440 ms straight call occurs at t=3.10 s; 67.684 ms is in
formulation/witness construction, 21.327 ms in input preparation and
11.655 ms in the QP solve. The 73.950 ms arc call at t=3.15 s spends
47.461 ms in formulation/witness construction and 12.058 ms in the solve.
Both are the first target-visible calls. This localizes the observed
peaks, without identifying a particular construction subroutine as the
cause. The 50 ms all-call deadline criterion therefore fails in both runs.
Timing excludes the twelve discarded preparation calls, perception and
physical simulation; no computation delay is injected into the plant.
Sampled SAT/road checks do not certify intersample safety.

An isolated HEAD-plus-task-patch source copy, excluding unrelated dirty
work, also passes all 66 focused tests. Independent Python computations
match all twelve MATLAB window summaries. Raw trials, CSV traces, scripts,
source inventories, native-binary hashes and comparison plots are retained
in the external evidence bundle; generated simulation files are not added
to the project repository.
