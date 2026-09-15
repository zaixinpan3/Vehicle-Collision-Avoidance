# Single-solve CBF–CLF controller experiment, September 15, 2026

## Result and design decision

Format 25 implements the requested single optimization architecture. Every
sample solves the same two-actuator SOCP. Target presence adds only hard
obstacle barrier rows. Road, state-domain, tire-slip, input, slew and cruise
CLF constraints stay hard in every call. There is no terminal policy, backup
trajectory, alternative rollout/normal search, solver retry, safety slack or
post-solve plan checker. Unsuccessful solver status stops the experiment with
`collisionAvoidanceController:optimizationFailed` before another command is
issued. The driver saves its executed prefix before rethrowing when an output
directory is supplied.

This materially simplifies runtime but does not solve the incompatibility
between exact cruising and obstacle avoidance. At zero cruising error, hard
CLF dissipation permits only a tiny numerical neighborhood of the trim. It
cannot permit meaningful braking or lateral deviation. As an obstacle
approaches, the joint hard problem becomes infeasible and the simulation ends.
No recursive-feasibility, all-future target, or physical safe-stop claim remains.

The complete constraints and conditional proof are in
[Single-solve sampled CBF–CLF controller](../controller/SINGLE_SOLVE_CBF_CLF.md).
The obstacle approximation encloses footprints by circumdisks, chooses one
geometric direction per hold, enforces a robust sampled barrier decrease, and
retains Bernstein inequalities for intersample separation. Circumdisks and
independent endpoint uncertainty supports introduce conservatism. The CLF
provides sampled practical dissipation with a reported measurement/numerical
floor. It does not claim pointwise continuous-time dissipation.

## Experiments actually run

The exact driver is `scripts/runExactStateRecursiveFeasibilityScenario.m`;
its historical filename remains, but it no longer claims recursive feasibility.
It integrates the issued affine generator independently with `expm`. The
control period is 0.1 s, reference speed 8 m/s, road bounds are ±5 m and required
clearance is 0.25 m. Physical geometry is measured at 11 points per hold as an
offline diagnostic, not an execution gate or the continuous-time proof.

The eight-trial campaign used seed 20260914 and requested 300 holds per trial.
A failure terminates that trial; the campaign records its saved report before
starting the next independent trial. All seven unsuccessful trials reported
native primal-infeasible status; no previous command was executed afterward.

| Trial | Executed holds | Outcome | Minimum sampled clearance above required margin |
| --- | ---: | --- | ---: |
| Exact stationary target | 7 | Hard problem infeasible at 0.7 s | 4.3500 m |
| Exact oncoming target | 29 | Hard problem infeasible at 2.9 s | 8.5500 m |
| Exact crossing target | 300 | Completed | 9.2697 m |
| Bounded stationary target | 5 | Hard problem infeasible at 0.5 s | 5.9469 m |
| Bounded oncoming target | 28 | Hard problem infeasible at 2.8 s | 10.3253 m |
| Bounded crossing target | 0 | Initial robust problem infeasible | No executed hold |
| Exact range-gated oncoming target | 42 | Hard problem infeasible at 4.2 s | 10.9500 m |
| NRMM range-gated oncoming target | 41 | Hard problem infeasible at 4.1 s | 12.9459 m |

The bounded trials use ego radii
`[.05,.05,.005,.05,.02,.005]` in Cartesian pose/body-velocity order, target radii
`[.1,.1,.1,.1,.05,.05,.01,.01]`, jerk amplitudes `[.1,.1]` m/s³ and yaw
acceleration amplitude .05 rad/s² at 1 rad/s. The range-gated trials use
reference speed 10 m/s, a 30 m observation region, an initially 100 m distant
target with lateral coordinate .8 m and longitudinal velocity −10 m/s. The
NRMM trial uses the actual observer output; its measurements are not replaced
by truth.

Additional runs:

- Clear-road cruise, 900/900 holds: median 2.946 ms, maximum 25.826 ms,
  zero 100 ms overruns; largest sampled CLF residual about −2.27e−9.
- Exact crossing, 900/900 holds: median 3.469 ms, maximum 34.421 ms,
  zero 100 ms overruns; minimum sampled clearance margin 9.26972 m.
- Cruise recovery from `[.05,.002,-.05,0,0]` path/speed error, 300 holds:
  completed, final error norm 2.11e−7, all sampled CLF bounds satisfied.
- Cruise recovery with small nonzero ego error boxes, 100 holds: completed,
  largest true-state CLF residual −8.38e−5 against the reported robust bound.
- Forced solver failure after the first successful command: exactly one hold
  executed; the following solve raised an error and the saved prefix remained
  available. The JSON logger was corrected to keep the injected function
  handle out of its configuration export, so it cannot mask that error.

The fresh MATLAB-process cruise run completed 20 holds but its first frame took
**1.414915 s**. Warm timing is therefore promising for the 100 ms period;
cold startup is not real-time ready. The broader campaign also recorded one
168.568 ms first-frame overrun. Timing includes input assembly and controller
work, excludes plant integration and offline geometry measurements, and does
not apply measured computation delay to simulated actuation. Some development
measurements overlapped a separate regression process. These are observed
latencies, not WCET or physical delayed-actuation guarantees.

The profiler recorded 20 controller calls, 20 `constrained` calls and 20 native
solver-dispatch calls for 20 crossing samples. No backup, terminal, certify or
plan-verification call occurred. Profiled timing is excluded from performance
qualification because instrumentation overhead increased it.

## Validation and evidence locations

Targeted tests cover solver call counts and strict status failures, failure with
existing input memory, equality of permanent constraints/objective with and
without targets, absence of slack variables, actual held-flow CLF dissipation,
uncertain state boxes, curved trim consistency, between-sample collision,
initial overlap, permanent road/domain constraints, estimator-bound use, input
units and rate limits, and saved error reports.

Tests that required deleted terminal, carried-witness, lexicographic-slack or
alternate predictive/backup execution APIs are retired rather than retained as
executable promises of the new controller. Independent geometry, held-flow,
target-conditioning, native-kernel, estimator and perception tests remain.
Historical theoretical counterexamples and model-only audits retain their
separate scope. The current full-suite result is recorded in the final
validation update below.

Development artifacts are under
`/home/zai/.cache/collisionAvoidance/single-solve-cbf-clf-20260915/`:
`campaign/summary.csv`, per-trial MAT/JSON reports, `long/`, `recovery/`,
`bounded/`, `cold/`, `profile.mat`, and regression logs/results. Generated
artifacts and external solver binaries are excluded from the project commit.
The committed code and this report are the durable project record; cache paths
identify the local original experiment outputs.

## Final validation update

The final complete suite passed **564/564** cases. After the final target-array
normalization and force-balance oracle updates, a **95/95** focused run
also passed. The combined latest results cover **566 distinct passing cases**,
with no failures or incomplete cases. Two added cases exercise multiple targets
inside the same solve and rejection of an unsafe second target. This combined
count is not presented as one fresh complete-suite run.

The earlier 557-case integration run retained five expected-interface failures:
two tests did not recognize the new optimization-failure identifier, and three
required the deleted terminal/backup architecture. Those interfaces/tests were
updated or retired. Its log is retained as diagnostic evidence. An initial
full run was interrupted by removal of obsolete test classes while discovery
was active; it is not counted as validation.

Factory `checkcode` reports **zero findings across all 19 edited MATLAB files**.
`git diff --check` passes. The core remains within its 20-source budget. No
external solver or generated kernel source/binary was changed.
