# Straight controller rerun after the real-time changes

September 13, 2026. Tested algorithm commit `e7c679a07193db8758bb0d1eb226bd591b020023`,
certificate version 20. Three exact-state straight scenes each complete 30 s.
All frame medians are below 100 ms, but 35 of 903 calls exceed 100 ms. Only
the crossing trial meets the every-frame deadline in this run. The stationary
trial stops and actually enters the terminal policy. This is a controller-only
experiment on the declared affine plant, not an estimator-loop or physical
vehicle qualification.

## Algorithm and experiment scope

The current implementation transfers the carried certificate through conditioned
box inclusion, condenses the optimization onto physical unknowns, reduces CLF
sampling to one decrease condition per hold, uses a speed-scaled fresh horizon
with a two-stage minimum, and uses the generated prediction/geometry kernels.
A frame deadline gates additional attempts. The controller and observer are
unchanged by this rerun. A summary-counting defect found while reading the raw
results is corrected below, with a focused regression.

The three sequential trials retain the previous definitions: oncoming target
at (60,0) with velocity (-8,0), crossing at (15,-4) with velocity (0,32), and
stationary at (15,0). Ego cruise speed is 8 m/s, the hold is 0.1 s, the configured
horizon is 16 stages, road edges are y = +/-5 m, footprints are 4.8 by 1.9 m,
and required clearance is 0.25 m. Exactly one target persists. Ego and target
measurement boxes are zero; seed 20260912 is retained for comparability, with
no nonzero noise. No solver failure is injected in these three trials.

The controller deadline is armed at 0.1 s by the current scenario driver.
The true ego follows each accepted first-stage affine generator with independent
`expm` integration; eleven samples per hold audit rectangle geometry. The plant
waits for computation, so overruns are measured without simulating delayed
actuation. All 300 holds execute; the final, 301st command is not executed.

## Safety, cruising and executed policy

| Scene | Final speed (m/s) | Final lateral error (m) | Final heading error (deg) | Extra separation (m) | Extra road margin (m) |
| --- | ---: | ---: | ---: | ---: | ---: |
| oncoming | 8 | 7.025e-16 | -7.986e-15 | 0.4611693 | 0.75447 |
| stationary | 2.845134e-20 | 2.178e-09 | 7.266e-08 | 6.40101e-06 | 3.8 |
| crossing | 8 | 1.579e-07 | -7.035e-08 | 9.269858 | 3.8 |

Both geometric margins have already deducted 0.25 m. Every issued command
passes the controller verification, all 900 successor witness checks pass,
and the reported predictive safety value and descent residual remain zero.
No sampled collision, road-boundary or actuator-slew violation is found. These
finite-run checks are not a new proof audit or nonlinear-plant guarantee.

| Scene | Carried commands | Terminal-law commands | First terminal command (s) | Final cruise outcome |
| --- | ---: | ---: | ---: | --- |
| oncoming | 0 | 0 | None | Cruise recovered/maintained |
| stationary | 287 | 276 | 2.5 | Stopped; recovery unmet |
| crossing | 0 | 0 | None | Cruise recovered/maintained |

The stationary ego issues its first terminal command at 2.5 s and continues
under the terminal policy through the end of the run. Of its 287 carried
commands, 276 are terminal-law commands; 275 terminal-law holds are actually
executed because the final command is unexecuted. The stopping behavior now
includes actual terminal execution, unlike the September 12 rerun of the
older information-state version where fresh optimizations kept the ego stopped.
Oncoming and crossing end near 8 m/s with negligible lateral and heading error.

## Timing

| Scene | First frame (ms) | Median (ms) | Maximum (ms) | Calls over 100 ms | Successor calls over 100 ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| oncoming | 936.674 | 59.683 | 936.674 | 18/301 | 17/300 |
| stationary | 278.104 | 76.493 | 470.491 | 17/301 | 16/300 |
| crossing | 89.710 | 60.710 | 89.710 | 0/301 | 0/300 |

The median-frame target is met in all three trials. The every-frame 100 ms
criterion fails for oncoming and stationary even with admission excluded.
These are observations on this workstation, not WCET bounds or a prediction
of timing on other hardware. Tests run after the sequential scenario batch.

For the oncoming maximum at t = 0, prediction and formulation contribute
about 357 and 332 ms. At t = 5.2 s, a successor frame takes 591 ms, with
529 ms in solving. The stationary maximum at t = 0.8 s takes 470 ms, with
418 ms in solving. Thus the misses are not only startup overhead.
`hardEncounterBarrier.plan` always permits the first attempt and uses measured
attempt duration to decide whether to start another; the admission frame has
no carried witness to take over. This gate is not a preemptive bound on the
first attempt, and late commands can still be returned.

| Scene | Mean prediction (ms) | Mean formulation (ms) | Mean solve (ms) | Mean acceptance (ms) | Median transferred-witness check (microseconds) |
| --- | ---: | ---: | ---: | ---: | ---: |
| oncoming | 12.877 | 32.960 | 31.806 | 1.544 | 23.0 |
| stationary | 7.154 | 45.208 | 29.974 | 0.443 | 31.0 |
| crossing | 11.489 | 30.926 | 15.302 | 1.423 | 23.0 |

The previous independent rerun measured frame medians 852, 951 and 13,874 ms
for oncoming, crossing and stationary. The current rerun is substantially
faster. Runtime-dependent attempt gating and different models/horizons near
rest mean this is a whole-controller comparison, not an isolated attribution
to one optimization. Per-frame timings and the five slowest frames of each
scene are retained in `summary.json`.

## Summary bug and historical correction

`verifyInformationStatePcbf.localRow` assigned `terminalLawFrames = 0` to every
ordinary trial. Only the forced-failure branch overwrote that constant. The
scenario trace and controller flags were correct; the summary suppressed
ordinary terminal execution. The helper now counts `report.terminalActive`
for every trial and leaves the value unknown when admission failed before
that trace existed. The redundant forced-failure-only assignment is removed.

The original round-two stationary exact/noisy JSON traces contain 276/217
terminal command entries, respectively, despite the published summary saying
zero. The two table cells in
[REAL_TIME_ROUND_TWO_RESULTS_20260912.md](REAL_TIME_ROUND_TWO_RESULTS_20260912.md)
are corrected with a dated explanation. Prior archive copies are preserved;
an append-only archive correction identifies the affected report. The original
traces are in `~/.cache/collisionAvoidance/realtime2-pcbf-20260912`, under
`stationary-exact` and `stationary-noisy`. Those historical noisy data are not
presented as a new noisy experiment.

## Validation and reproduction

The nine existing controller/CLF/solver regression classes pass **111/111** cases.
The new campaign-summary regression passes **1/1** case, running a normal
40-hold stationary trial and comparing the summary with its saved terminal
trace. This short regression is separate from the three 30-second timing trials.
MATLAB MCP executes the tests. The first request timed out at 300 s; MATLAB
completed the 111 tests and saved results, which a subsequent MCP call loaded
and verified with `assertSuccess`. Code Analyzer reports no findings in the
changed summary helper or test class; `git diff --check` passes. Controller,
configuration and native kernel hashes are compared against the snapshot taken
before the experiments. No full-suite result from the prior 640-test campaign
is represented as having been rerun here.

```matlab
addpath('controller','config','scripts','tests', ...
    'solver/bicycle','solver/clarabel/matlab');
outputDirectory = fullfile(tempdir,'straight-controller-rerun');
for scenario = ["oncoming","crossing","stationary"]
    report = runExactStateRecursiveFeasibilityScenario(Scenario=scenario, ...
        SampleCount=300,FailAfterAdmission=false,DeadlineSeconds=0.1, ...
        EgoErrorBound=zeros(6,1),TargetErrorBound=zeros(8,1),Seed=20260912, ...
        OutputDirectory=outputDirectory);
end
```

MATLAB R2026a Update 3 and the installed native kernels were used. Original
MAT/JSON traces, summary, deadline outliers, figure PDF/PNG, test results,
source provenance and reproduction scripts are retained at:

`/home/zai/.cache/collisionAvoidance/straight-rerun-20260913`

Archive copies are identified as exports. Estimator-in-the-loop, noisy new
trials, curved roads and nonlinear physical-vehicle validation remain outside
this rerun.
