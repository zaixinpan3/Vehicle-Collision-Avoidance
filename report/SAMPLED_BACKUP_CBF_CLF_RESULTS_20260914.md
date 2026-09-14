# Sampled backup CBF, hard cruise CLF, and runtime validation

September 14, 2026. This implementation follows the
[version-22 rerun](CONTROLLER_V22_RERUN_20260914.md), whose tested controller
source was `e232514d03814c0cc3d1934642861d353d8543a7`. The repository baseline
for this change was `b419e2e8afbc816a52e6725e7a551b0074b47f72`.

## Result and scope

The redesigned policy completes the exact stationary, oncoming and crossing
encounters and restores stable path cruising. In the independent 90 s,
10 m/s, 30 m visibility trial, speed stays at the reference after recovery
and lateral/heading error converges to numerical precision. The median
frame is **4.428 ms**, the maximum **56.072 ms**, and **0/901** frames exceed
100 ms. All 837 executed holds marked as CLF-certified satisfy an
independently evaluated sampled dissipation inequality.

This is a substantial improvement in the declared-model experiment, with
three qualifications:

- A fresh MATLAB process still takes **892.766 ms** for first admission.
  One warm exact-oncoming admission takes **125.921 ms**. A hard 100 ms
  end-to-end guarantee has not been established.
- Broad-noise stationary/oncoming trials still obtain no admission; the
  actual NRMM trial still fails at first target publication, 3.5 s.
- The new policy executes a **cruise-trim affine generator**. The previous
  predictive policy executes its own stage generators. Both drivers integrate
  the generator recorded with each command; this is not a comparison on one
  independently validated nonlinear vehicle plant. Neither policy establishes
  physical-vehicle safety here.

The safety guarantee is a predictive CBF certificate in the augmented
information/witness state, conditional on admission and the declared motion,
measurement and execution contracts. The hard CLF guarantee is sampled
dissipation on certified cruise holds, with a practical bound under persistent
estimation error. It is not pointwise intersample decrease of a quadratic
CLF, nor forced path convergence while the path is obstructed. The derivation
and assumptions are in
[SAMPLED_BACKUP_CBF_CLF.md](../controller/SAMPLED_BACKUP_CBF_CLF.md).

## Why the previous controller did not recover reliably

The earlier 90 s experiment identified repeated performance-stage budget
exhaustion followed by terminal braking, while fresh plans also produced
sustained lateral oscillation. A soft CLF objective with nonzero slack did
not imply actual dissipation. Profiling also showed significant prediction
and constraint-construction cost before optimization. Solver tuning alone
could not remove those costs or turn a relaxed CLF into a guarantee.

The old terminal brake gain restricted terminal longitudinal speed to about
8.7 m/s despite an 18 m/s model domain. That made an immediate certified
handoff unavailable at a normal 10 m/s cruise point. Reducing the brake gain
within the existing comparison-system proof enlarges the admissible speed
range; the recomputed stopping excursion becomes longer. Terminal chart
proposals now use the proposed entry velocity, with invariant pose rows
still enforcing actual chart containment.

## Implemented policy

`controller.executionPolicy="backup"` uses a finite bank of at most two
prescribed feedback rollouts, each capped at 64 holds. Every proposed
continuation must pass the full swept road/collision/domain/slip check,
finite confirmed-exit condition, road terminal membership and input/slew
checks. Up to 16 verified braking-transition holds fit within the same cap
when terminal input slew requires them. Positive safety violation is rejected.

An admitted active encounter executes its stored suffix with measurement
conditioning and the existing sound release guard. After release, the
controller proposes a sampled cruise-feedback input, checks a hard quadratic
dissipation inequality on the current information box, and verifies the
hold and subsequent road-safe continuation. If this candidate fails, the
stored witness or terminal law remains available. A bounded return-path
proposal can replace a witness near its end. No online optimizer is called
by this policy.

`"predictive"` preserves the SOCP search. `"auto"` selects backup for a
positive constant-speed reference and finite frame budget, including the
experiment's default 100 ms setting. Moving-offset references keep the
predictive path under `auto`. A verified format-22 plan can also feed the fast
executor; the original controls and stage generators remain authoritative.

The fixed verifier avoids a dense horizon transcription. Each cell retains
only two held-input columns, substituted before assembling physical margins.
A cached eight-column flow template represents six initial-state coordinates
and two input coordinates. Its arithmetic reserve covers the full state/input
domain and its common starting radius encloses all stage-start boxes. This
last choice adds conservatism in uncertain cases; no uncertainty is capped.
All original Bernstein points and Taylor remainder checks remain.

## Final sequential campaign

The table includes admission, failed decisions and the final unexecuted
command. The eight ordinary trials run sequentially after the explicit cold
stationary trial. No other test job was launched by this task during timing.

| Trial | Executed / requested holds | Maximum frame (ms) | Frames above 100 ms | Outcome |
| --- | ---: | ---: | ---: | --- |
| Cold-process exact stationary | 300 / 300 | 892.766 | 1 / 301 | Safety/truth audits pass; returns to cruise |
| Warm exact stationary | 300 / 300 | 98.562 | 0 / 301 | Safety/truth audits pass; returns to cruise |
| Exact oncoming | 300 / 300 | 125.921 | 1 / 301 | Safety/truth audits pass; returns to cruise |
| Exact crossing | 300 / 300 | 41.220 | 0 / 301 | Safety/truth audits pass; returns to cruise |
| Bounded stationary | 0 / 300 | 127.990 | 1 / 1 | Two proposals rejected; no command issued |
| Bounded oncoming | 0 / 300 | 116.815 | 1 / 1 | Two proposals rejected; no command issued |
| Bounded crossing | 300 / 300 | 45.926 | 0 / 301 | Safety/truth audits pass; practical cruise recovery |
| Exact oncoming, 30 m visibility | 300 / 300 | 43.750 | 0 / 301 | Avoidance and cruise recovery complete |
| Actual NRMM, 30 m visibility | 35 / 300 | 168.937 | 2 / 36 | Target admission fails at 3.5 s |
| Additional exact visibility trial, 90 s | 900 / 900 | 56.072 | 0 / 901 | Stable 10 m/s path cruise |
| Bounded crossing, optimizer forced unavailable | 300 / 300 | 84.316 | 0 / 301 | Completes without invoking an optimizer |

The warm exact-stationary and exact-oncoming successor maxima are 51.509
and 45.793 ms. Their admissions account for their table maxima. The initial
NRMM frame takes 168.937 ms; the failed target-admission frame takes
146.288 ms. Neither is hidden by reporting only successful commands.

An earlier campaign before the final flow-template cache is retained
separately. Its cold stationary maximum was 842.694 ms; warm stationary
admission/maximum was 422.058 ms and its successor maximum was 299.511 ms.
The cache removes repeated stage-flow construction, but these individual
timings are not a controlled worst-case benchmark. The final cold result is
slower, and a warm admission overrun remains. Workstation scheduling and
unrelated load were not controlled.

## Safety and cruise measurements

These geometric margins are additional to the configured 0.25 m clearance.
They come from independent sampled geometry audits. The continuous safety
certificate uses the full swept Bernstein enclosures, not those samples.

| Trial | Minimum extra separation (m) | Minimum extra road margin (m) | Confirmed release (s) |
| --- | ---: | ---: | ---: |
| Exact stationary | 0.153430 | 0.559733 | 4.2 |
| Exact oncoming | 1.039595 | 0.562679 | 5.0 |
| Exact crossing | 9.269716 | 3.800000 | 0.7 |
| Bounded crossing | 9.286414 | 3.790549 | 0.7 |
| Exact 30 m visibility, 30 s and 90 s | 0.683082 | 0.545434 | 6.5 |

All completed trials issue only zero-safety-value certificates. The exact
driver's completed trials also pass independent model-domain and truth-box
containment checks. No terminal braking command occurs in the final ordinary
completed campaign. Legacy predictive-mode tests still exercise terminal
braking when an optimizer fails.

The bounded crossing finishes at 8.000069447 m/s, lateral error 0.004125842 m
and heading error 0.000075509 rad. Its large crossing separation makes it a
mild collision regression, not evidence that the difficult noisy passing
cases are solved.

For the 90 s trial, independent reconstruction from consecutive true sampled
states evaluates

    V(x_(k+1)) - (1-decay_k) V(x_k) - disturbanceBound_k.

Across the 837 CLF-certified executed holds, its largest residual is
`-1.16272e-24`; the decay fraction is at least `0.0736260` per 0.1 s hold.
The numerical disturbance bound is `1.16272e-24` in this exact-state case.
During 80--90 s, longitudinal speed is 10 m/s to reported precision and
absolute lateral error is below `2.4e-52` m. These tiny ideal-model values
mean numerical convergence, not achievable physical positioning accuracy.

The previous independent 90 s run had speed 6.896--9.751 m/s and lateral
error -3.711--3.572 m during the same final interval, with 22 terminal-law
commands and 12 deadline misses. The comparison plot is retained outside
the repository as `cruise-comparison.png` and `.pdf`; it explicitly labels
the different declared generators.

## Remaining failures and limitations

The last bounded stationary/oncoming proposals have safety values 9.85679
and 14.9234, respectively, with zero hard-row violation. Their sufficient
road/collision inequalities fail. The actual NRMM encounter's last proposal
has safety value 2254.13 and hard-row violation 127.343. Its 35 safe
target-free holds do not certify the newly observed encounter. These
outcomes show failure of this finite proposal family, not global
infeasibility, inevitable collision, or observer divergence.

Closed-loop robust backup tubes or a larger independently verified search
family may enlarge admission. Such a change must propagate future feedback
and measurement-error effects explicitly; assuming future measurements will
shrink the current open-loop tube would invalidate the certificate.

The experiment measures computation time but executes an ideal hold clock.
It does not turn overruns into actuation delay. Bounded candidate count is
not a certified worst-case execution-time bound in MATLAB. Deployment needs
initialization before the timed loop, a scheduled executor for buffered
verified controls, and a validated sensing-to-actuation timing contract.
Newly admitted targets must also be detected early enough for a certificate
to become available. No old road-only witness is treated as covering a new
collision obligation.

The common cruise CLF covers a fixed-curvature trim. Small-offset tests cover
straight, left/right radius-400 m and left radius-100 m references. Changing
curvature changes the local metric and is reported; a global Lyapunov theorem
for an arbitrary changing-curvature route is not supplied. Nonzero declared
ego process residuals are rejected by this fixed verifier. Nonlinear vehicle
flow inclusion and physical actuation delay remain outside this guarantee.

## Validation and reproduction

MATLAB R2026a Update 3, GLNXA64, eight computational threads. Seed 20260914,
sample time 0.1 s. The first six campaign trials use reference 8 m/s,
initial horizon 16, 16 m confirmation range, road boundaries +/-5 m and
the unchanged exact-driver scene initializations. Bounded cases use ego
radii `[.05;.05;.005;.05;.02;.005]`, target radii
`[.1;.1;.1;.1;.05;.05;.01;.01]`, jerk amplitudes `[.1;.1]` m/s^3,
yaw-acceleration amplitude .05 rad/s^2 and motion frequency 1 rad/s.
The visibility-gated pair uses reference 10 m/s, target initial position
`[100;.8]` m and velocity `[-10;0]` m/s. Actual NRMM retains its existing
80 Hz sensor/observer and prior configuration. These are different scenes.

From the repository root:

```matlab
addpath('scripts');
out = '/home/zai/.cache/collisionAvoidance/cbf-clf-redesign-20260914/cached-flow-validation';
cold = runExactStateRecursiveFeasibilityScenario( ...
    Scenario="stationary",SampleCount=300,Seed=20260914, ...
    OutputDirectory=fullfile(out,'cold-stationary'));
campaign = runStraightControllerRerun( ...
    SampleCount=300,Seed=20260914,OutputDirectory=fullfile(out,'campaign'));
longRange = runDeclaredPlantEstimatorControllerScenario( ...
    SampleCount=900,UseEstimator=false,Seed=20260914, ...
    OutputDirectory=fullfile(out,'range-exact-90s'));
```

For the forced-unavailable optimizer case, call the exact driver with
`Scenario="crossing", SampleCount=300, FailAfterAdmission=true` and the
bounded radii/motion amplitudes above. Explicit `ExecutionPolicy="predictive"`
selects the legacy optimizer-failure/terminal-law experiment.

The final focused regression run passes **74/74** cases, including all
**20** new backup-policy cases, curved cruise, native held-flow/geometry,
controller repair, bounded target motion and terminal fallback. Independent
CLF tests check all 64 initial-box vertices; new cached-flow tests compare
every vertex through several changing held inputs against matrix-exponential
trajectories. Tests also cover stopped launch, 15 m/s terminal admission,
finite brake/steering slew, expired budgets, corrupted issued inputs,
unsafe proposals, process-residual rejection and format-22 witness execution.

Two earlier whole-suite runs each contained 705 cases and initially reported
3 and 1 failures. These were old optimizer-failure tests whose implicit
`auto` selection no longer called an optimizer. They now explicitly select
`predictive`, preserving their original behavior checks. After focused
reruns and the added cases, the latest result for every distinct test name
is **713/713 passed**, with no failed or incomplete cases. This is an
aggregate of the full suite and targeted reruns, not one fresh 713-case run.
The first MATLAB MCP full-suite request timed out while MATLAB continued;
its eventual saved result is retained. The subsequent batch run completed.

Factory Code Analyzer reports zero findings in all 13 changed/new MATLAB
files. `git diff --check` passes. Controller/configuration core remains
20 source files; no native binaries or solver dependencies are added.

Raw MAT/JSON results, profiling, regression CSV/MAT files, aggregate test
counts, analyzer output, environment details, and the comparison figure
remain under
`/home/zai/.cache/collisionAvoidance/cbf-clf-redesign-20260914/`.
The final campaign is in `cached-flow-validation/`; the earlier pre-cache
campaign remains in its original directories. Generated outputs are
deliberately excluded from the project commit.
