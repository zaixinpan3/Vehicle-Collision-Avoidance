# Straight avoidance with scheduled computation delay

Historical result notice, September 9, 2026: the delayed/partial-horizon
controller and its replay branch have been removed. These dated observations
retain their original experimental scope and do not validate version 13.
See [the current architecture](../controller/PCBF_CLF_ARCHITECTURE.md).

Research date: September 8, 2026.

## Corrected requirement

The complete measured online frame must finish within the actual control
update period. The preceding experiment used a 50 ms control/model period,
accepted computation up to 100 ms, and applied the new command at the
measurement instant in simulated time. Its recorded maxima of 71.802 ms
(controller only) and 94.244 ms (joint) therefore do not establish a runnable
50 ms controller. The historical measurements remain valid as computation
measurements, but the realtime interpretation is withdrawn.

An internal integration step may be shorter than the computation time: it
does not itself require a new actuator command. In the preceding experiment,
however, `controller.sampleTime` was also the actuation/update period. That
is the mismatch corrected here. A deadline test alone cannot compensate for
unmodeled actuation delay. MathWorks likewise treats computation delay as
part of the sampled closed-loop model in its
[computational-delay example](https://www.mathworks.com/help/slcontrol/ug/modeling-computational-delays-and-sampling-effects.html).

## Executed timing contract

The strict straight entry now sets

```matlab
cfg.controller.sampleTime = 0.1;
cfg.controller.horizonSteps = 16;
cfg.controller.inputDelaySteps = 1;
cfg.controller.certifiedSteps = 2;
```

For measurements at `t_k = k T`, `T = 0.1 s`, let `v_k` be the already
scheduled input and `u_k` the input computed during the current frame. The
sampled execution is

\[
 x_{k+1}=F_T(x_k,v_k),\qquad v_{k+1}=u_k,\qquad C_k\leq T.
\]

The optimization fixes its first input to `v_k`, schedules its second
input as `u_k`, and uses Taylor/Bernstein uncertainty tubes and CLF majorants
over the entire first **two** intervals. Later stages retain the existing
nonlinear nominal lookahead and independent nonlinear acceptance. The
lookahead is 1.6 s; the robust prefix is 0.2 s. Neither nominal lookahead
nor two-interval certification constitutes a recursive-feasibility proof.

The simulation integrates the plant with the committed command while the
new result is being computed in modeled time. Early completion waits for
the scheduled next boundary. No solve is treated as instantaneous. The
result records measurement time, observed ready time, scheduled actuation
time and the source frame of each applied input. The initial held input is
the declared physical cruise equilibrium, including air and rolling
resistance. Straight two-point centerlines and declared analytic references
support that initialization.

The solver eliminates the committed input exactly in both the optional
reserve LP and the joint SOCP, including its cross terms in the quadratic
objective. Independent acceptance rejects any changed committed input.
Stored schedules must match their previously checked second-stage input;
road coverage must include both the delay and the new held interval.
Initialization seeds and geometry rebuilds also preserve the fixed prefix.
This does not add a backup policy, a desired-acceleration target or an LQR
execution law. Predictive CBF, CLF with squared slack, the joint input/state
objective and the target NRMM high-gain observer remain the governing
frameworks. The observer algorithm is unchanged.

An enforced deadline greater than the update period is rejected. A failed
solve or deadline miss stops the experiment without applying a new command.
The harness truncates the trace at the failed sample; it does not simulate
post-failure emergency behavior. Synthetic sensor/observer/road/controller
work is timed; physical-plant integration and offline preparation are
excluded. This remains a scheduling-aware simulation, not an operating-system
WCET proof or an actual YOLO/LiDAR/actuator-network timing measurement.

## Diagnostic findings

The nonlinear-bicycle diagnostic uses exact observations, 10 m/s ego and
target speeds, a 100 m initial longitudinal gap, 0.8 m target offset,
30 m sensing and the declared physical residual allowance
`[0.2; 0.06; 0.02; 2.5; 5; 4]`. It does not include fitted hard road rows.

With the new 100 ms scheduled delay and 16 stages, the initial diagnostic
stops at 4.3 s with no accepted fresh solution. The maximum attempted frame
is 191.609 ms, including that failed solve. The minimum sampled SAT margin
before stopping is 8.8787 m; the encounter has not completed.

A failed-sample replay isolates a 6.2709e-7 physical-unit violation of a
stage-2 nominal combined-tire-force row after the native solver reports
success. The independent checker correctly rejects it. Tightening the
solver feasibility tolerance from 1e-7 to 1e-9 removes that numerical
obstruction, but the nonlinear future check still reports a 0.0080613 m
collision-row violation at stage 7 after the allowed refinements. The
production tolerance and physical limits are not weakened.

Longer-lookahead diagnostics also fail: 20 stages stop at 3.9 s, 24 stages
expire at the first frame, and 32 stages stop at 3.7 s. Their maximum
attempted times are 158.661, 212.635 and 239.967 ms, respectively. These are
same-session diagnostic measurements, not isolated deadline certifications.
An elastic LP used only to diagnose the best failed 20-stage formulation
needs a positive common relaxation of approximately 0.004383; the binding
rows include future heading-domain limits and a station-chart limit. This
does not prove physical avoidance impossible. Widening the station chart
from 2 to 10 m still stops at 4.0 s (169.782 ms maximum, deadline enforcement
disabled solely for this functional diagnostic). That variant is not adopted.

The robust affine prefix and nonlinear future rollout have distinct local
centers. A captured 20-stage anchor has a 0.022406 m/s lateral-velocity
discrepancy at the end of the robust prefix; its propagated nominal lateral
position discrepancy reaches approximately 0.027783 m at the horizon.
This is diagnostic evidence of a model-consistency issue to investigate,
not a demonstrated explanation of every infeasible solve. No uncertainty
reset or removal of physical constraints is used to conceal it.

The current evidence therefore does not establish successful delayed
avoidance or realtime usability. Controller-only validation remains the
gate for any joint experiment.

## Straight physical result

A new single-thread MATLAB process runs the strict entry on PassVeh14DOF
with the settings above and a requested 30 s duration. The controller-only
experiment completes 17 intervals and stops on attempted frame 18, at
**1.7 s**, because its complete measured frame takes **238.432 ms**.
The returned result is failed; the joint experiment is not started.

The slow frame spends 99.779 ms in prediction, 70.500 ms in formulation
and witness construction, 58.213 ms in acceptance/commit work, and 5.091 ms
in numerical solving. This breakdown localizes the observed cost; it does
not establish why that particular frame is slow or a reproducible WCET.
The target has not yet entered the 30 m sensing range. The minimum sampled
SAT margin before stopping is 61.014882 m, which says nothing about
completing the encounter or subsequent cruise recovery.

The raw result is
`physical-scheduled-validation/straight-realtime-validation.mat`.
An earlier invocation stopped during setup because the new initialization
incorrectly required an analytic reference even for a two-point straight
centerline. That interface error was fixed before the recorded physical
run; its original error log is retained and is not counted as a simulation.
No repeated unchanged run is substituted for this failed timing result.

## Reproduction and evidence

The strict physical entry is

```matlab
addpath('scripts');
report = runStraightRealtimeValidation( ...
    Duration=30, DeadlineSeconds=0.1, RandomSeed=20260907, ...
    OutputDirectory='/path/to/experiment/output');
```

Original diagnostic results and replay programs are retained outside the
repository in
`~/.cache/collisionAvoidance/straight-scheduled-20260908/`.
New behavior tests cover the fixed input, delayed command identity and
timestamps, schedule corruption, period/deadline consistency and actual
plant use of the previous frame's scheduled command. **411 relevant MATLAB
tests in 42 classes pass**, including all seven new delayed-actuation tests,
with zero failed or incomplete cases. This is the selected controller,
observer and integration regression set, not every repository test.
The test RPC exceeds its 300 s transport timeout, but MATLAB completes the
suite and saves the CSV/MAT results; those files are independently checked
without repeating the suite.

The recorded physical commands exactly match the preceding frames'
scheduled commands, all applied frames complete before their next boundary,
and the expired final command is not applied. Eleven native MEX binaries
match the preceding physical-build hashes; the shared kernel algorithms
are unchanged. Numerical-source inventory and Code Analyzer findings are
preserved with the raw results. These checks validate the implementation
and execution order; they do not turn the failed experiment into a pass.
