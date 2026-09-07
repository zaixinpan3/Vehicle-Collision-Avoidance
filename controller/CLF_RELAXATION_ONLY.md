# CLF relaxation-only objective

This report records experiments and diagnostics from before the modified Fiala
tire revision. Its linear-tire and friction-limit results describe that earlier
controller. The current model uses scheduled Fiala tangents and no separate
axle-friction constraints; see [LTV_BICYCLE_MODEL.md](LTV_BICYCLE_MODEL.md).

This is the historical relaxation-only experiment. The current controller
uses [quadratic input effort and squared relaxation](QUADRATIC_CLF_INPUT_OBJECTIVE.md).
The measurements and interpretation below refer to the preceding objective.

## Implemented change

At the user's request, the sampled LQR input preference introduced by
`dc58ef1` is removed. The online optimization is now

```text
minimize    rho * delta
subject to  delta >= 0
            LfV + LgV*u0 <= -alpha*V + delta
            all existing hard predictive constraints.
```

`rho = cfg.clf.relaxationWeight > 0`. Every head and continuation input cost
is removed, including the former `1e-6` continuation quadratic. There is no
acceleration or steering target, input magnitude penalty, input rate penalty,
or secondary input optimization in the objective. Inputs remain decision
variables constrained by the model, geometry, actuator and tire limits, and
terminal rest. The applied input still comes from independent acceptance.

`formulateAvoidanceProblem` supplies an identically zero Hessian and a linear
vector whose only nonzero coefficient belongs to `delta`. The sparse lift
adds no cost on states. This is a linear program through the existing QP
interface and native Clarabel backend; no native binary or solver setting is
changed. Solver numerical regularization is distinct from an explicit control
objective, and no new regularization is added here.

The discrete gain synthesis, its sample-period cache key, and its public
preferred-input/feedback diagnostics are removed. The original continuous
Riccati construction of `P` and `alpha` remains. In particular,
`clf.frontWheelSteeringAngleWeight` and `clf.longitudinalAccelerationWeight`
still define the certificate's design matrix `R`; they no longer weight an
online input cost. The curvature-dependent cruise state reference and its
equilibrium-input diagnostic are retained for CLF construction and inspection.
They are not optimized as input targets. The legacy `inputDeviationCost`
diagnostic remains available and evaluates to zero.

The geometry and execution improvements from `dc58ef1` remain: batched road
support, configured solver tolerances, explicit discarded initialization,
and one computational thread in the experiment drivers. All physical and
acceptance thresholds retain their definitions.

## What this objective does and does not select

If zero relaxation is feasible, every feasible plan with `delta = 0` is a
mathematical optimizer. The objective does not rank those plans by recovery
speed, input size, smoothness, or future tracking. Different solvers can
therefore return different equally optimal commands. Changing the positive
weight `rho` scales the objective without changing the exact minimizer set;
it can still change numerical behavior at finite solver tolerances.

When all cruise errors are zero, `V = 0` and its gradient is zero. The
instantaneous CLF row permits `delta = 0` for every input admitted by the hard
constraints. In the simplified straight speed channel, a held acceleration
can nevertheless produce `eNext = gamma*Ts*a`, hence a positive next-sample
`V`. This explains why minimizing only the instantaneous derivative relaxation
does not by itself make the desired cruise an invariant sampled equilibrium.
It is an algebraic limitation of this formulation, not a claim that every
optimal command must cause poor tracking.

For a nonzero speed error `e = vx - vRef`, the zero-slack CLF still requires
`2*Pvv*e*gamma*a <= -alpha*Pvv*e^2`. Removing the input penalty removes the
old preference for the smallest admissible acceleration. It does not require
faster decay once this inequality is satisfied. Closed-loop measurements are
therefore needed to evaluate the proposed objective.

## Controlled zero-error probe

At a straight-road state with all cruise errors zero and speed exactly
15 m/s, the native solver selected acceleration `-6.646274 m/s²`, predicting
14.734149 m/s one sample later. An independent `linprog` solve selected
`+4.0 m/s²` for the same initial problem. Both returned zero CLF relaxation
and passed the unchanged independent acceptance checks. These commands are
observations for this solver/configuration, not uniquely required optima.
They demonstrate the objective's indifference between markedly different
commands even at the desired state.

## Reproduction and provenance

```matlab
addpath('scripts');
rng(2026, 'twister');
study = runControllerDesignExperiments( ...
    OutputDirectory='/tmp/clfRelaxationOnlyStudy');
results = runtests({'tests/cruiseRecoveryTest.m', ...
    'tests/continuousTimeClfTest.m', 'tests/sparseAvoidanceQpTest.m'});
assertSuccess(results);
```

The retained experiment directory is
`/tmp/controller-clf-slack-only-20260906`. Its `trial_source` is a copy of the
actual working-tree tracked sources, including disclosed pre-existing
uncommitted research changes. `baseline_source` records the sources before
this task. The isolated `commit_candidate` excludes those unrelated changes.
Source manifests distinguish these contexts; tests on the isolated candidate
are not described as plant runs on a clean commit.

## September 6, 2026 results

Four original 10 s PassVeh14DOF trials used the unchanged 50 ms period,
24-stage head and 74-stage continuation, 15 m/s cruise, 8 m/s crossing target,
50 m acquisition, 400 m arc radius, 4.8 by 1.9 m rectangles, 6 m boundary
offsets, 2.6 m shoulders, gain 0.80, solver tolerances `1e-6`, and RNG
2026 twister. Estimator and sensor noise were disabled. One computational
thread and explicit initialization were retained. All online calls, including
the first and target acquisition, are included.

| Avoidance scene | Recovery speed error (m/s) | Lateral error (m) | Heading error (rad) | Recovery |
| --- | ---: | ---: | ---: | --- |
| straight | 0.236764 | 0.216549 | 0.026827 | Fail |
| arc | 0.231966 | 0.241319 | 0.022657 | Fail |

These are maxima over **every sample from 9 through 10 s**, with unchanged
limits 0.5 m/s, 0.2 m and 0.02 rad.

| Scene | Trial | Initialization (s) | Median / p95 / max call (ms) | Misses / calls |
| --- | --- | ---: | ---: | ---: |
| straight | nominal | 1.518 | 23.072 / 27.999 / 46.824 | 0 / 200 |
| straight | avoidance | 0.548 | 31.446 / 38.901 / 44.598 | 0 / 200 |
| arc | nominal | 0.485 | 23.932 / 25.502 / 29.174 | 0 / 200 |
| arc | avoidance | 0.380 | 32.059 / 35.087 / 44.458 | 0 / 200 |

| Avoidance scene | Minimum SAT (m) | Minimum road margin (m) | Functional | Deadline |
| --- | ---: | ---: | --- | --- |
| straight | 1.462372 | 7.022467 | Fail | Pass |
| arc | 1.836727 | 6.915174 | Fail | Pass |

Requested acceleration over the recovery window and its largest consecutive
step are retained to quantify fluctuations:

| Avoidance scene | Acceleration range (m/s²) | Maximum step (m/s² per 50 ms) |
| --- | ---: | ---: |
| straight | -7.254135 to 3.975905 | 11.219875 |
| arc | -7.259491 to 3.974586 | 11.042115 |

All 800 online calls completed without failure, fallback or deadline miss.
Both avoidance trials passed control-node collision separation, road and
physical-command criteria. Maximum hard-row residuals were `9.25e-7` and
`1.02e-6`, within the unchanged `1e-5` acceptance tolerance. Peak requested
friction utilization was 1.00000004 and 1.00000030, also within that numerical
tolerance. This does not claim extra actuation reserve.

All 20 online inputs in each avoidance recovery window had exactly zero
reconstructed CLF relaxation despite the reported input fluctuations.
The speed criterion passed in both scenes, while both lateral and heading
criteria failed. Thus neither scene passed the combined functional assessment.
All four changed MATLAB files had zero factory Code Analyzer findings.

The objective experiment is implemented as requested. These measured
performance outcomes do not change the objective or relax the acceptance
criteria. The saved comparison figure and state/command traces distinguish
the latest behavior from the earlier sampled-LQR result.

## Validation and limitations

The initial 27 focused tests passed. The isolated commit candidate returned
174 passes, 0 failures and 0 incomplete tests among 174 relevant
controller/configuration/geometry/evaluator and short plant-driver tests.
New coverage verifies equal cost for arbitrary head and continuation inputs
at fixed slack, absence of a sparse input/state quadratic, and agreement of
optimal relaxation with independent `linprog` solves for moderate and
actuator-limited speed errors. Initialization isolation and fault handling
remain tested. The existing `quadprog` compatibility test passes with its
expected warning that the zero-Hessian problem is linear.

The separate before/after probe verifies bitwise-identical initial hard
inequalities, equalities, bounds and affine CLF rows at 8.0, 14.4 and 15.0 m/s.
This comparison holds the initial scenario and geometry construction fixed;
it does not claim identical future trajectories after inputs diverge.
The three native binaries match the preceding experiment hashes.

Timings describe complete online controller calls on this host after separately
measured initialization. They exclude perception and plant execution. The
plant does not inject computation latency; collision separation is evaluated
at control nodes. No hard worst-case time, intersample safety, general sampled
convergence or estimator-robustness theorem is inferred. The full repository
suite was not rerun; earlier unrelated full-suite failures are not claimed fixed.

## Why lateral and heading recovery miss the window

A subsequent analysis of the same saved trials separates three mechanisms.
It introduces no controller change or new plant simulation.

First, the reference-cruise Riccati matrix has a decoupled speed coordinate:
`V = Vspeed + Vlateral`, where `Vspeed = 0.625*(vx-vRef)^2` and `Vlateral`
contains lateral position, heading, lateral velocity and yaw-rate error,
including their cross terms. The CLF constrains the derivative of their sum;
it does not require either part, or either individual pose error, to decrease
at every sample. On the straight road, at fixed 15 m/s reference:

| Time (s) | Total V | Speed part | Lateral-state part |
| ---: | ---: | ---: | ---: |
| 7.0 | 4.55052 | 4.54349 | 0.00702 |
| 7.2 | 2.99931 | 2.87116 | 0.12815 |
| 7.5 | 1.61251 | 0.99715 | 0.61536 |
| 8.0 | 0.82544 | 0.02994 | 0.79550 |

At 7.2 s, the recorded modeled derivatives are approximately
`VspeedDot = -8.48369`, `VlateralDot = +1.19184`, and
`VDot = -7.29185`. The required bound is only `VDot <= -6.04889`.
Thus the lateral-state part is actively increasing while the combined
zero-slack CLF condition passes. This is direct evidence of the permitted
trade between longitudinal and lateral recovery, not merely a hypothesis
based on acceleration fluctuations. These derivative values use the
straight, zero-bias model `vxDot = gamma*a`; no analogous isolated derivative
claim is made for the curved-road data.

Second, minimizing only slack stops distinguishing inputs once zero slack
is feasible. In particular, later predicted CLF values and individual
tracking errors are diagnostics, not costs or recovery constraints. The
optimizer has no criterion requiring the lateral overshoot to be smaller or
to finish before the 9 s acceptance-window boundary. On the recorded straight
run, lateral error grows to 0.499381 m at 8.10 s after the acceleration phase,
then decays; heading error peaks at 0.047574 rad at 7.45 s.

Third, the current continuous-time derivative condition does not impose a
sampled decrease condition. In the 20 straight-road recovery calls at
9.00--9.95 s, every reconstructed slack is zero and every instantaneous CLF
row passes, yet the model's own held-input prediction has `Vnext > Vcurrent`
in six calls. The reference and P are identical at these straight-road
nodes, so the discrepancy already exists within the predictor. The 19
available consecutive recorded-current-state pairs also contain six increases.
Those counts have different denominators and are not claimed to identify the
same six intervals. The modeled and actual observations support a sampling
limitation; they do not assign a causal fraction to plant/model mismatch.

Both runs eventually enter the pose tolerances by 10 s. The actual failed
samples within the complete 9--10 s window are:

| Scene | Lateral violations (> 0.2 m) | Heading violations (> 0.02 rad) | Final lateral / heading error |
| --- | --- | --- | --- |
| Straight | 9.00, 9.05 s | 9.00, 9.05 s | 0.081592 m / -0.006545 rad |
| Arc | 9.00, 9.05, 9.10, 9.15 s | 9.75, 9.80 s | 0.051974 m / -0.013886 rad |

The straight run therefore settles just after the required window starts;
the arc run also has a later heading excursion. A final-sample-only assessment
would pass these two pose channels and conceal those failures. The correct
conclusion is a missed window and insufficient sustained tracking precision,
not a failure to approach cruise at all.

The analysis is retained in `/tmp/controller-lateral-recovery-diagnosis-20260906`.
It verifies that the speed/lateral partition reconstructs the saved total V
within `1e-10` at the fixed 15 m/s reference. Curved-road predicted-V counts
retain their locally changing curvature references and are explicitly not
used as a frozen-reference sampling isolation. No model-only closed-loop
ablation or independent state-decay-controller trial was performed.
