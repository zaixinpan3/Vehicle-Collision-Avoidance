# Cruise recovery and controller execution time

This report records experiments and diagnostics from before the modified Fiala
tire revision. Its linear-tire and friction-limit results describe that earlier
controller. The current model uses scheduled Fiala tangents and no separate
axle-friction constraints; see [LTV_BICYCLE_MODEL.md](LTV_BICYCLE_MODEL.md).

Historical experiment record for commit `dc58ef1`. The subsequent
[CLF relaxation-only experiment](CLF_RELAXATION_ONLY.md) removes the sampled
LQR input preference and all head/continuation input costs at the user's
request. The results below describe the earlier implementation; they do not
validate the current objective. Its geometry and execution improvements remain.

## Scope and reproduced failures

The September 6, 2026 baseline used the actual Vehicle Dynamics Blockset
PassVeh14DOF plant, 15 m/s cruise, an 8 m/s crossing target acquired within
50 m, a 50 ms control period, a 24-stage performance horizon and a 74-stage
continuation. Straight and 400 m radius arc paths each had a nominal
counterfactual and an avoidance trial. The state estimator was disabled.

Both avoidance trials completed without contact at evaluated control nodes,
but the maximum speed errors over **all samples from 9 to 10 seconds** were
0.658066 and 0.743346 m/s, exceeding the unchanged 0.5 m/s limit. Lateral and
heading errors already passed. Final speeds alone were within tolerance;
using only the final sample would have concealed the recovery failure.

The original straight/arc avoidance controller calls had median times
36.250/37.977 ms, maxima 119.501/73.663 ms, and 5/2 deadline misses among
200 calls each. A cold nominal call reached 750.062 ms. Before editing, a
recorded-input replay reproduced the original commands exactly. Warm replay
phase medians were approximately 13–14 ms for formulation and witness checks
and 12 ms for the native solve. These wall-clock observations are not
worst-case execution-time bounds.

## Why recovery was slow

The continuous CLF permits a set of inputs; it does not choose the fastest
cruise recovery within that set. The previous input objective preferred the
cruise equilibrium input at every stage, including zero acceleration on a
straight road without a declared acceleration bias. With speed error
`e = vx - referenceSpeed`, `V = Pvv e^2`, and longitudinal effectiveness
`gamma = 0.8`, the speed-only active CLF approximately gives

```text
2 Pvv e gamma a <= -alpha Pvv e^2,
a >= -alpha e / (2 gamma)    when e < 0.
```

The minimum-effort solution therefore approaches `eDot = -alpha e/2` once
acceleration is no longer saturated. The recorded cruise certificate gave
`alpha = 2.0168 /s`, hence a speed-error rate of about `1.0084 /s`. The
minimum generalized eigenvalue across all five CLF error coordinates sets
this rate; it need not match a desired longitudinal settling time. Plant
resistance absent from the zero-bias predictor further weakens late recovery.
At 8.95 s the original straight trial still had about 0.684 m/s speed error
and requested only 0.862 m/s². This explains the failed recovery window
without attributing it to an infeasible safety QP.

The revised current-input preference is

```text
uPreferred(0) = uEquilibrium(0) - Kd * cruiseError(0).
```

`Kd` comes from discrete Riccati synthesis using the exact held-input
straight-reference bicycle matrices at the configured sample period and
`Ts*Q`, `Ts*R` weights. This is a discrete LQR design, not the exact integral
of the continuous cost. The original continuous LQR gain was not directly
reused as a sampled command: its longitudinal gain is 32 for this setup.
The discrete feedback accounts for the held-input sample period. The QP
still decides the applied command; no clipping or replacement occurs after
independent acceptance. Future performance inputs retain their equilibrium
centres and the continuation retains its existing small input penalty.

Only the objective centre changes. The continuous CLF matrix/rate, its
relaxation rule, hard geometry/physical/domain rows, terminal rest, horizon,
collision clearances and actuator/friction limits retain their definitions.
The added nominal feedback has stable poles for its unconstrained design
model. This is not a proof of global convergence of the constrained,
scheduled, sampled nonlinear plant or of tracking a changing reference.

## Execution changes and their limits

The native solver previously replaced configured feasibility and optimality
tolerances of `1e-6` by `min(1e-9, configuredTolerance)`. It now honors both
configuration values. Independent physical-plan acceptance retains its
original tolerance and checks; solver success alone still does not admit a
plan. The experiments must establish any runtime benefit of this change.

Road support calculations are batched across prediction nodes. Quadratic
boundary extrema still include the full admitted station interval and its
stationary point. For rectangle support
`h(theta) = l*abs(cos(theta)) + w*abs(sin(theta))`, between its quadrant
kinks, `h'' = -h < 0`. Thus `h'` is monotone on each smooth piece, and the
largest absolute slope is bounded by interval endpoints and the two kink
families. Evaluating those candidates directly removes sorting and piece
construction. Covered-node selection, carried-frame admission, support
uncertainty and independent rectangle checks remain in place.

`prepareCollisionAvoidanceController` explicitly runs before periodic
sampling. It exercises empty-target, distant-target, changing crossing-target
and return-to-cruise paths with synthetic targets derived from the initial
pose. It reads no future observation or saved scenario trace. Twelve calculations are attempted; all produced commands and their explicit
certificates are discarded. Expected synthetic admission failures are
recorded without vetoing the real initial problem; unexpected configuration
or input errors still propagate. Its returned record
contains every preparation-call duration and the total initialization cost.
A regression checks that preparation leaves the live persistent certificate
unchanged.

`runCenterlineCruiseScenario` and `benchmarkControllerRuntime` request one
computational thread by default and restore the caller's limit on return.
The native solver already uses one thread. This makes the experiment's
execution configuration explicit and reduces unnecessary small-matrix
threading overhead. `PrepareController=false` and `ComputationalThreads=8`
reproduce the former execution lifecycle on this eight-core host; cold-start
claims require a fresh MATLAB process. Configuration is recorded in results.
Every online call, including the first and target acquisition, remains in
the original 50 ms deadline test. Initialization is a separate cost, not a
successful 50 ms sample.

Desktop and contended-process replays still exhibited outliers. In one
recorded-input run, the first crossing-target calls took 64–97 ms despite
initializing only distant-target paths; their formulation times dominated.
Later passes of that same input sequence were faster. A further replay run
concurrent with the regression suite also missed deadlines. These exploratory
runs are retained; they cannot establish a deployment deadline. A valid
comparison records first-use behavior, initialization, computational threads,
all timed samples and concurrent work, and runs the final benchmark without
another experiment/test process launched by this task.

The simulation plant advances after each command and does not inject
computation latency. Passing observed controller-call deadlines therefore
supports only the measured host/run, not a hard real-time deployment or
intersample collision guarantee. Those claims require a target runtime,
latency-aware plant validation and an appropriate scheduling/timing argument.

## September 6 retest

The final four 10 s trials completed all 800 intervals with zero controller
failures, zero fallbacks and zero online deadline misses. Both scene-level
functional assessments passed. The following values are for avoidance;
speed error is the maximum across all 21 samples in the 9–10 s window.

| Scene | Original / revised recovery speed error (m/s) | Original / revised maximum online call (ms) | Revised minimum SAT margin (m) |
| --- | ---: | ---: | ---: |
| Straight | 0.658066 / 0.012843 | 119.501 / 40.819 | 1.452812 |
| Arc | 0.743346 / 0.012959 | 73.663 / 46.339 | 1.789300 |

Revised avoidance median/p95 times were 35.197/37.287 ms (straight) and
36.722/38.816 ms (arc). Minimum road margins were 7.645986 and 7.542550 m.
Peak commanded friction utilization reached the existing unit limit (about
1.0, versus 0.947 originally); the faster response uses more of the allowed
actuation envelope. No additional traction reserve is claimed.
The nominal counterfactuals contacted the evaluation target at 5.80 s;
first avoidance detections were at 3.10/3.15 s and 49.334/49.535 m.

Initialization took 1.561 s for the first straight nominal trial, followed
by 0.514, 0.566 and 0.404 s for straight avoidance, arc nominal and arc
avoidance. The largest online call across all four trials was 48.752 ms.
These times include every online call after the explicitly recorded
initialization. They exclude perception and plant simulation.

The initial working-tree controller/configuration/geometry/evaluator regressions
passed 171 tests with zero failures or incomplete results. Existing tests
retain continuous-CLF sample-period independence and sparse/condensed QP
agreement against an independent `quadprog` solve. New coverage checks
recovery acceleration, sampled feedback poles, preservation of the live
certificate during initialization, and rectangle-heading bounds at 25 road
orientations.

The final avoidance inputs were also replayed three times per scene (1,200
calls). Every steering and acceleration command matched its recorded value
exactly. There were zero deadline misses; the largest replay calls were
46.145 ms for straight and 47.657 ms for arc. These are recorded-input
runtime checks, separate from the four plant experiments.

The final isolated commit candidate passed 173 tests, including all seven
short plant-driver regressions, with zero failures or incomplete tests.
An initial isolated run exposed two oncoming-driver failures caused by
infeasible synthetic initialization probes. The initializer now records
these failures and continues to the real controller problem, whose safety
checks are unchanged. A focused fault-injection regression verifies this
separation. The four successful plant-run source snapshots precede only
this initialization error guard; the online controller code is unchanged.
With the updated initializer, all 48 probes across the four recorded trial
starts were certified. A further 800-call replay reproduced every command
exactly with zero deadline misses.
The earlier full-suite estimator and stationary-pose failures are outside
this repair and are not claimed resolved.

## Reproduction

```matlab
addpath('scripts');
study = runControllerDesignExperiments(OutputDirectory="/tmp/controllerStudy");
data = load('/tmp/controllerStudy/arc_avoidance.mat');
report = benchmarkControllerRuntime(data.result, Repetitions=3, ...
    OutputDirectory="/tmp/controllerReplay");
results = runtests({'tests/cruiseRecoveryTest.m', ...
    'tests/continuousTimeClfTest.m', 'tests/sparseAvoidanceQpTest.m', ...
    'tests/avoidanceSafetyGeometryTest.m'});
assertSuccess(results);
```

## Primary sources and design decisions

1. Ames, A. D., Xu, X., Grizzle, J. W., and Tabuada, P. (2017).
   *Control Barrier Function Based Quadratic Programs for Safety Critical
   Systems*. IEEE Transactions on Automatic Control, 62(8), 3861–3876.
   [DOI: 10.1109/TAC.2016.2638961](https://doi.org/10.1109/TAC.2016.2638961).
   [Full author manuscript](https://arxiv.org/html/1609.06408), Section IV-A,
   equations (31)–(33), distinguishes an admissible CLF input set and its
   minimum-norm selection. Section IV-B, equations (34)–(37), permits a
   state-dependent quadratic/linear input objective while relaxing the CLF.
   These support the objective-centre decision, not a theorem for this
   project's finite-horizon, sampled controller.
2. MathWorks, [dlqr documentation](https://www.mathworks.com/help/control/ref/lti.dlqr.html).
   The discrete Riccati solution supplies the sampled gain and closed-loop
   poles. This motivates designing the nominal feedback with the actual
   held-input matrices and sample period.
3. Clarabel, [solver settings](https://clarabel.org/stable/api_settings/).
   Feasibility and optimality are separate settings. The implementation
   honors the declared settings while retaining independent acceptance.
4. MathWorks, [measuring program performance](https://www.mathworks.com/help/matlab/matlab_prog/measure-performance-of-your-program.html)
   and [computational thread limits](https://www.mathworks.com/help/matlab/ref/maxnumcompthreads.html).
   Repeated performance measurements and explicit thread settings guide the
   benchmark. Median measurements alone cannot establish an every-call
   deadline; first use and every exceedance are also reported.

Sources checked September 6, 2026. The causal numerical diagnosis above is
from this repository's recorded experiments, not imported from the papers.
