# Exact-state experiment: remaining controller problems

September 14, 2026. Tested repository revision
`0ad244f6e129e9f57dacf546173321d7a58d80b7`, controller certificate version 21,
using `scripts/runExactStateRecursiveFeasibilityScenario.m`. The controller,
configuration defaults, and experiment driver were unchanged during this audit.

## Results

Eight runs expose a terminal-domain defect, admission-search limitations,
unprepared target re-entry, timing overruns, and an incomplete experiment pass
condition. The most serious new result is that bounded speed error can make
the certified terminal brake drive the declared affine plant into negative
speed. A short experiment still reports success after this happens.

| Run | Executed / requested holds | Driver result | Relevant observation |
| --- | ---: | --- | --- |
| Default stationary | 0 / 120 | Fail | 44 attempts; 30.670899 s admission timeout |
| Default oncoming | 27 / 120 | Fail | Release at 0.1 s; re-entry admission fails at 2.7 s |
| Default crossing | 120 / 120 | Pass | Release at 0.7 s; final speed 8.000000007 m/s |
| Crossing with bounded noise and target disturbances | 120 / 120 | Pass | Final speed 7.999597966 m/s; four frames exceed 100 ms |
| Crossing, forced solver failure, exact states | 300 / 300 | Pass | Carried road tail reaches near-rest; no negative speed |
| Crossing, forced solver failure, full bounded noise | 91 / 300 | Fail | Negative true speed at 6.7 s; controller fails at 9.1 s |
| Crossing, forced solver failure, speed error only | 91 / 200 | Fail | Reproduces the same domain failure without other uncertainty |
| Same speed-only case, shorter duration | 75 / 75 | **False pass for model-domain safety** | Final true speed -0.008559686 m/s; all 76 commands marked certified |

The three default runs retain the script's seed 20260912. All other runs use
seed 20260914. Each hold lasts 0.1 s. Successful runs include one final issued
command that is not executed; failed successor calls are retained in the trace.

No sampled footprint collision or road-boundary violation occurred in the
executed prefixes. The default oncoming prefix's minimum sampled separation
margin is 11.750146 m, and the default crossing margin is 9.269858 m. These
prefix observations do not establish avoidance after a failed admission, or
restore the domain premise once true speed is negative.

## 1. The uncertain terminal brake does not preserve the speed domain

This failure needs only `EgoErrorBound=[0;0;0;.05;0;0]`, valid synthetic
measurements, and `FailAfterAdmission=true`. Target observations are exact and
target jerk/yaw acceleration are zero. Every executed hold uses the issued
certificate's exact affine generator, as required by this experiment.

The road terminal law begins at 1.6 s. It commands braking from the stored
nominal speed, while estimation error evolves under the open-loop generator.
For longitudinal damping `d=0.1962 s^-1` and sample time `h=0.1 s`, the code gives

\[
\bar v_{j+1}=e^{-(d+1)h}\bar v_j,\qquad
e_{j+1}=e^{-dh}e_j,
\]

and therefore

\[
v_j=e^{-djh}\left(e^{-jh}\bar v_0+e_0\right).
\]

Here `hardEncounterBarrier.localTerminalDynamics` implements the nominal
ratio and feedback; `collisionAvoidanceController.localTerminalFrame` uses
the stored center as the feedback argument. Any negative initial speed error
eventually dominates the faster-decaying positive nominal speed.

For the isolated speed-error run at terminal entry:

- Nominal speed, recovered from the issued input and terminal feedback:
  **8.037680170574196 m/s**.
- True speed: **7.988422560113916 m/s**.
- Error: **-0.04925761046028043 m/s**.
- The exact held-input flow crosses zero at **6.695118961 s** on the
  experiment clock; the first negative sampled state is at **6.7 s**.
- The formula reproduces all recorded terminal speed endpoints with maximum
  absolute discrepancy below **3.6e-15 m/s**.
- **24 commands at negative true-speed states**, from 6.7 through 9.0 s, remain
  marked certified. Minimum speed is **-0.010629388 m/s**.
- At 9.1 s, a measurement box entirely below zero triggers
  `collisionAvoidanceController:invalidInput`.

`hardEncounterBarrier.localTerminalSet` explicitly imposes nonnegative
**nominal** speed without charging the speed-error radius. The terminal
velocity-error budgets cover signed excursions, but do not establish the
permanent `speedMinimum=0` obligation. `validateTransition` intersects with
that domain on the assumption that the plant already belongs to it. Once
the true state is negative, this intersection can exclude truth while still
being nonempty. Later measurement rejection exposes the earlier loss of the
premise; it is not evidence that the injected measurement noise exceeded its
declared bound.

**Required correction:** derive a sampled terminal controller and information
set that preserve the actual speed domain under bounded state error. Simply
adding `nominal >= radius` to the current terminal set is insufficient: that
inequality is not invariant under these two different decay rates. Until the
law is corrected and verified, the uncertain road-tail claim needs this
limitation stated explicitly.

## 2. The admission search misses a verified stationary avoidance plan

The default stationary search expires without executing a hold. A separate
instrumented rerun reaches horizons 16 through 39 and also expires; its 47
attempts are not the 44 attempts of the uninstrumented timing run.

Offline fixed-state probes retain the declared model, swept verification,
actuator limits, finite exit, road terminal rows, and zero-safety-value gate.
They compare the existing straight-anchor collision normals with normals
proposed from a consistent left or right passing path. For each fixed
horizon, the dynamics and optimization anchor are identical between these
normal choices. All 12 probes are saved, including failures.

| Scene / horizon | Existing normals: verified violation value | Consistent negative-side normals | Consistent positive-side normals |
| --- | ---: | ---: | ---: |
| Oncoming at 2.7 s / 21 | 10.452695 | 3.260254 | 3.260222 |
| Oncoming at 2.7 s / 32 | 10.452842 | 3.260222 | 3.260215 |
| Stationary at admission / 48 | 1.592466 | **0; certified** | **0; certified** |
| Stationary at admission / 64 | 1.592476 | **0; certified** | **0; certified** |

These are verified candidate values, not a global value function. Positive
values remain non-executable. The positive-side 48-stage solve reaches its
iteration limit but returns a decision that passes independent certification;
the replay below uses the negative-side result, whose solver feasibility flag
is also true. The oncoming probes obtain no zero-violation plan.

An independent 11-points-per-hold replay of the negative-side 48-stage
stationary witness gives:

- Minimum separation beyond the required 0.25 m clearance:
  **0.001107280 m**; minimum road margin: **1.118266688 m**.
- Minimum speed: **7.998448838 m/s**.
- At 4.8 s, ego position `[38.399075080;0.241136974]` m, target reference
  distance **23.400317554 m**, and road-terminal membership verified.
- The complete swept certificate has zero safety value and all hard rows
  satisfied, including full-footprint departure from the 16 m region.

This is a verified finite open-loop witness for the declared model, not an
implemented online search repair or physical-vehicle validation. Its small
clearance reserve also does not cover unmodeled dynamics. It demonstrates
that the default stationary failure is not absence of every finite witness.

### A contradictory normal sequence and premature search termination

The traced failed oncoming attempt at 2.7 s, with 21 stages, assigns adjacent
cells 70 and 71 nearly opposite normals `[0;-1]` and `[0;1]`. Their shared
endpoint is 1.0 s into the candidate. Both closed cell intervals certify that
endpoint, demanding positive-clearance separation on opposite sides of the
same target simultaneously. The tiny longitudinal components are below
1.4e-16 and cannot resolve the conflict inside the bounded road chart.

`avoidanceSafetyGeometry.build` proposes each normal from the anchor's local
rectangle geometry. `hardEncounterBarrier.localPredictionSchedule` generates
longitudinal cruise/braking admission seeds on this straight road.
`avoidanceSafetyGeometry.optimizeNormals` exists, but no production controller
call invokes it. Cellwise normal proposals need a consistent passing-side
search, followed by the existing complete verification.

There is also an early-exit issue in `hardEncounterBarrier.plan`: after ten
infeasible attempts at horizons 16--20, the first 21-stage attempt returns a
finite diagnostic candidate with value **10.452691716**, hard/CLF checks
satisfied, and solver exit flag 2. The verifier correctly refuses execution,
but the planner treats that flag as a reason to return immediately. It does
not try the second 21-stage seed or longer horizons, although its 30 s search
budget has not expired. Treating a positive-value candidate as a recoverable
search outcome would permit those attempts; it would not justify executing
the candidate or prove that the oncoming scene becomes feasible.

## 3. Current exterior membership does not provide re-entry readiness

In the default oncoming scene, the target starts 60 m ahead and approaches
at 8 m/s while ego travels at 8 m/s. The driver publishes that target even
outside its declared 16 m confirmation region. At 0.1 s the controller
confirms exterior membership, releases the encounter, and continues road-only
cruise. At 2.7 s the target footprint is no longer certifiably exterior;
`validateTransition` requires fresh admission and discards the road-only
candidate as a candidate for the new collision obligation. Admission fails.

This follows the implemented encounter-release semantics, but does not solve
the whole approaching-target task. Current exterior membership alone neither
means the target is receding nor ensures that later entry is admissible.
The target-free controller needs an explicit detection/entry-readiness
contract, or a strategy that retains and handles known approaching targets.
The fixed-deadline admitted-encounter theorem does not provide that global
composition result.

## 4. The 100 ms setting is not an end-to-end execution guarantee

| Run | Admission time | Maximum successor time | Frames above 100 ms |
| --- | ---: | ---: | ---: |
| Default stationary | 30,670.899 ms | No successor | 1 / 1 |
| Default oncoming | 138.033 ms | 3,513.876 ms | 3 / 28 |
| Default crossing | 65.806 ms | 89.042 ms | 0 / 121 |
| Bounded-noise crossing | 948.215 ms | 159.298 ms | 4 / 121 |
| Forced exact crossing | 91.464 ms | 69.578 ms | 0 / 301 |
| Forced full-noise crossing | 63.197 ms | 55.793 ms | 0 / 92 |

The default trio share one fresh MATLAB process. The noisy, forced exact,
and forced full-noise runs share a second process, in that order. Separate
speed-only reproductions start their own processes. Startup, warm state, and
normal workstation variation prevent causal timing comparisons between rows.
Instrumented admission and geometry probes are excluded from this table.

`collisionAvoidanceController` installs `model.frameTimer` only when a carried
candidate exists. New admissions use the separate 30 s search budget.
`hardEncounterBarrier.plan` always starts its first attempt, and checks time
between attempts rather than interrupting an overlong solve. Thus a 100 ms
configuration value does not bound all control frames, even with a witness.
The experiment continues after overruns without applying the corresponding
actuation delay. Reported geometric safety is scoped to its ideal execution
clock; late real-world command application is not validated.

## 5. The experiment's pass condition misses domain and truth-inclusion failures

`runExactStateRecursiveFeasibilityScenario` checks certification flags,
candidate verification, value descent, sampled footprint/road separation, and
input slew. It does not check true speed/domain membership or whether the
conditioned information sets continue to contain truth.

The 75-hold speed-only run completes, reports `passed=true`, and marks all 76
commands certified even though nine certified frame states have negative
speed. Its timing qualification is false for a separate reason. A physical
domain audit must contribute to the safety pass condition independently of
the controller's own certification flag. Truth-inclusion checks would also
detect conditioning that removes the true state.

The default crossing target moves laterally at 32 m/s and retains more than
9 m sampled clearance. Its success is useful regression evidence, but does
not exercise a tight evasive maneuver. The stationary witness probe provides
a more demanding geometry example; broader online maneuver coverage remains
necessary.

## Reproduction and retained artifacts

MATLAB R2026a Update 3. Road boundaries are +/-5 m, reference speed 8 m/s,
initial horizon 16, and clearance 0.25 m. The driver integrates the selected
affine stage generator using `expm`; it is not a nonlinear Fiala or perception
pipeline experiment. Dense geometry auditing uses 11 samples per hold and is
independent numerical evidence, not the swept certificate's proof.

Original traces, logs, diagnostic scripts, captured attempts, and all geometry
probe results are under:

`/home/zai/.cache/collisionAvoidance/controller-audit-20260914`

```matlab
addpath('scripts');
out = '/home/zai/.cache/collisionAvoidance/controller-audit-20260914';
for scene = ["stationary","oncoming","crossing"]
    runExactStateRecursiveFeasibilityScenario(Scenario=scene, ...
        OutputDirectory=fullfile(out,'default',scene));
end
runExactStateRecursiveFeasibilityScenario(Scenario='crossing', ...
    SampleCount=120,Seed=20260914, ...
    EgoErrorBound=[.05;.05;.005;.05;.02;.005], ...
    TargetErrorBound=[.1;.1;.1;.1;.05;.05;.01;.01], ...
    TargetJerkAmplitude=[.1;.1],TargetYawAccelerationAmplitude=.05, ...
    OutputDirectory=fullfile(out,'noisy'));
runExactStateRecursiveFeasibilityScenario(Scenario='crossing', ...
    SampleCount=300,Seed=20260914,FailAfterAdmission=true, ...
    OutputDirectory=fullfile(out,'forced-exact'));
runExactStateRecursiveFeasibilityScenario(Scenario='crossing', ...
    SampleCount=300,Seed=20260914,FailAfterAdmission=true, ...
    EgoErrorBound=[.05;.05;.005;.05;.02;.005], ...
    TargetErrorBound=[.1;.1;.1;.1;.05;.05;.01;.01], ...
    TargetJerkAmplitude=[.1;.1],TargetYawAccelerationAmplitude=.05, ...
    OutputDirectory=fullfile(out,'forced-noisy'));
runExactStateRecursiveFeasibilityScenario(Scenario='crossing', ...
    SampleCount=200,Seed=20260914,FailAfterAdmission=true, ...
    EgoErrorBound=[0;0;0;.05;0;0], ...
    OutputDirectory=fullfile(out,'forced-speed-only'));
runExactStateRecursiveFeasibilityScenario(Scenario='crossing', ...
    SampleCount=75,Seed=20260914,FailAfterAdmission=true, ...
    EgoErrorBound=[0;0;0;.05;0;0], ...
    OutputDirectory=fullfile(out,'forced-speed-short'));
```

The cached `auditAttempt.m` is a nonpausing MATLAB conditional-breakpoint
tracer at `hardEncounterBarrier` line 201 of the tested revision. It captures
the model, prediction, program, result and verification after each attempted
solve. Two separate runs capture stationary admission and oncoming re-entry.
`probeAdmissionGeometry.m` reconstructs constant-cruise predictions at fixed
horizons and substitutes normals proposed from a smooth approach to lateral
offset +/-3 m over 0.8 s; it changes no safety threshold.
`auditStoredGeometryProbe.m` independently replays the certified stationary
48-stage negative-side plan. Breakpoints were cleared after tracing.

Validation comprises eight experiment runs, two instrumented admission
reproductions, 12 geometry probes, the independent stationary replay, and the
terminal-speed recurrence comparison. A one-hold crossing run first checked
the tracer itself and is not counted as a campaign trial. No production fix
or new MATLAB test is included in this diagnosis. Regression coverage should
be extended with long uncertain terminal runs, independent domain/inclusion
audits, consistent passing-side admission, and target re-entry before claiming
these problems resolved.
