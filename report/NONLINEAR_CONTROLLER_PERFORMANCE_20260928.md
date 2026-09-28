# Nonlinear controller performance campaign

Date: September 28, 2026.

## Scope and execution

This campaign evaluates the nonlinear controller implementation in commit
`15781bcb32395a0980ef61108f1fd2eb0ee15e2a`, whose preceding final validation was
recorded in `c783b007d57e7049b6e0d40dbd2add4e1a1deffb`. Production controller and
configuration code were not changed. The replay driver now exports complete
controller-call timings, candidate/solver counts, tracking errors, and the
independent sampled state trace. Timings use an explicit `tic` handle to avoid
nested timer interference. An option enables the existing convex-improvement
iterations; the prior driver's default disabled these iterations.

MATLAB R2026a Update 3 started successfully. No shared process/service restart
was performed for this campaign. All three MPFR verifiers were rebuilt from
source. The six selected controller/kernel/configuration suites passed 135/135
cases, with zero failed or incomplete cases. This is not the entire repository
suite. The final instrumented driver has one existing Code Analyzer `NASGU`
notice for the deliberately throwing callback's unused return assignment.
An initial instrumentation attempt failed on empty-structure trace
assignment after controller calls; those runs remain in the external
`validation/` directory and are excluded from the performance results. The trace
initialization was corrected before the reported campaign.

The scenarios use a separate tight `ode45` integration of the same nonlinear
combined-slip Fiala ODE, with 31 audit points per 50 ms hold and tolerances
`RelTol=1e-11`, `AbsTol=1e-12`. Reference speed is 8 m/s. The initial tracking
recovery offset is 0.01 m; circular cruise uses curvature 0.005/m. The oncoming
target starts 24 m ahead, moves oppositely at 8 m/s, and has a 4.8 by 1.9 m
rectangle, matching the ego footprint. The corridor is 8 m wide and the required
vehicle clearance is 0.1 m. Default actuator bounds are steering +/-40 degrees
and longitudinal force ratio [-1, 1]; slew-rate limits are infinite in these
fixtures. The minimum planned horizon is eight holds; the
controller extends avoidance plans. Inputs are deterministic; no random seed is
used. The normal improvement search budget is 5 s, the configured frame deadline
is infinite, and certificate computation time is not capped. Deadline misses
are measured against the physical 50 ms hold, not the configured search budget.

The short recovery and circular cases disable improvement. The solver-failure
case injects an exception into the improvement callback. The stored-policy case
sets the improvement deadline to `1e-12` s after its first admission. The online
oncoming case enables two improvement iterations per frame, matching the current
controller's iteration default. An enabled iteration budget does not imply an
LP/QP was actually reached; recorded solver calls distinguish these outcomes.

All scenarios run serially with one MATLAB computational thread. The first call
of a scenario may benefit from caches warmed by preceding cases; these are
observational timings on the shared workstation, not isolated cold-start or
worst-case timing guarantees. Controller-call timing excludes `ode45`, sampled
geometry auditing, progress output and artifact writing.

## Measured results

| Case | Holds | Min vehicle clearance (m) | First call (s) | Later median / p95 (ms) | Calls over 50 ms | LP/QP or hook calls |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Small-error recovery (improvement off) | 8/8 | n/a | 4.476 | 4194.136 / 4529.069 | 8 | 0 |
| Circular cruise (improvement off) | 8/8 | n/a | 0.569 | 543.235 / 546.107 | 8 | 0 |
| Forced improvement failure | 8/8 | n/a | 0.474 | 476.199 / 483.072 | 8 | 8 |
| Oncoming / inherited policy | 210/210 | 0.582879 | 9.414 | 1.842 / 2.392 | 1 | 0 |
| Oncoming / two improvement iterations enabled | 210/210 | 0.171481 | 9.378 | 7413.123 / 10583.713 | 210 | 0 |

All five cases completed: 444 applied holds and 13764 independent audit samples. No sampled collision, road departure, or replay enclosure violation was found. The forced-failure case actually invoked the failing hook eight times; those are not successful optimizer solves.

Stored-policy execution passed the target and dispatched the invariant backup for 21 holds. Final lateral/heading/longitudinal-speed errors were 4.31190202e-05 m, -6.47294778e-06 rad and -6.55475674e-13 m/s. Minimum sampled road margin was 0.069825 m.

The online case passed the target and satisfied all five reported recovery thresholds. It dispatched no terminal-backup holds; replanning continued. Final lateral/heading/longitudinal-speed errors were 4.7206848e-06 m, -1.5516849e-07 rad and -3.11470538e-08 m/s. Minimum sampled road margin was 0.157460 m. Maximum observed controller call was 12.245619 s.

Although two improvement iterations were enabled, **no LP/QP was reached in the online case**. The long calls are therefore not evidence that the convex optimizer itself is slow. Candidate generation and nonlinear certificate work occur before the improvement loop checks its search budget. Certificate time is uncapped in the tested configuration, so this work can exceed the 5 s search budget. The stored suffix is fast when the test deadline forces an immediate return; ordinary online execution still performs costly readmission before returning.

Online executed-plan horizon lengths: [144, 189]. The shift-and-append candidate preserves its length. Timing changes over the run must not be attributed to an assumed shrinking executed-plan horizon.

The offline simulations advance physical time by 50 ms per command regardless of computation time. They demonstrate trajectories under that timing assumption, not a vehicle remaining safe while waiting several seconds for a command. The online controller is not qualified for a 50 ms real-time loop.

## Independent checks and interpretation

The external Python audit independently reconstructs rectangle vertices, applies
separating-axis overlap checks and computes closest vertex/edge distances at all
recorded `ode45` audit points. It also checks the entire rectangular footprint
against the straight or circular corridor. Circular road checks include the
closest points on rectangle edges, not only vertices. Sampled clearance is
compared with the MATLAB result to 1e-8 m; road margin must be nonnegative.
For both oncoming runs, target passage and final transverse errors were also
independently recomputed from the saved world states, and all recorded controls
were finite and within their amplitude limits.
Continuous-time certification remains the responsibility of the MPFR verifier.

Reported recovery thresholds are absolute lateral error below 0.1 m, heading
error below 0.02 rad, longitudinal and lateral speed errors below 0.1 m/s, and
yaw-rate error below 0.02 rad/s. Complete
avoidance/backup handoff additionally requires passing the target and at least
one dispatched invariant-backup hold. Completing an arbitrary short frame count
alone is not full avoidance recovery. A positive CLF slack is retained and is
not described as strict monotonic recovery throughout avoidance.

This is an exact-model nonlinear closed loop, not an independent PassVeh14DOF
physical-plant experiment. Finite fitted road boundaries, varying-curvature
paths, uncertain observations and changing target parameters are outside the
new controller's supported contract and were not silently substituted into
these runs.

## Reproduction and original artifacts

Original JSON traces, logs, native binaries, source hashes, MATLAB batch drivers
and the independent `audit.py` are retained at
`/home/zai/.cache/collisionAvoidance/nonlinear-performance-20260928/`.

The batch commands actually used were:

```bash
matlab -batch "run('/home/zai/.cache/collisionAvoidance/nonlinear-performance-20260928/run.m')"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/nonlinear-performance-20260928/campaign.m')"
python /home/zai/.cache/collisionAvoidance/nonlinear-performance-20260928/audit.py
```

Compact results are adjacent to this report. Raw audit traces and generated
native binaries remain outside the repository. Pre-existing target, observer,
scenario, manuscript and instruction changes are excluded from this task's
commit.
