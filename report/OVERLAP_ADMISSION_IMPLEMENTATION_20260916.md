# Overlap-aware admission implementation and straight validation

Date: September 16, 2026. Controller state format: 32. This report separates
successful exact-state controller validation from an unsuccessful joint
observer-controller admission. The declared ego plant is the zero-residual
held affine model. These results do not certify a nonlinear physical vehicle.

## Implemented changes

- Replace ordinary-distance availability with analytic signed support geometry.
  Overlap/contact supplies a usable support direction without claiming safety.
- Search temporally coherent geometric sectors, alternative fresh exit
  directions and finite horizons. No lateral trajectory, avoidance acceleration,
  fixed passing time or recorded avoidance controls are prescribed. The finite
  search is incomplete and is not an exhaustive convex decomposition.
- Use grouped, normalized, search-only collision/exit/terminal deficits. Keep
  dynamics and physical actuator/slew limits hard. Update directions using the
  worst reserved Bernstein/support margin over each complete hold. Restore the
  original performance objective and zero search deficits for the final hard
  solve; independently verify the complete original physical certificate.
- Use one sparse stage transcription for hard and restoration problems. Remove
  the obsolete standalone slack/budget transcription. Preserve the future-state,
  trim-input and squared CLF-slack objective. Reduce exact duplicate rows, then
  generate omitted violated inequalities until the full problem is satisfied.
  Keep original rows and cones as the final acceptance authority.
- Retain the accepted prediction, generators, enclosures, terminal certificate
  and absolute encounter deadline. A geometry replacement must preserve the
  complete inherited witness. Terminal feedback is never an executable fallback.
- Reuse identical affine stage transitions and compiled numeric support kernels.
  Support interpolation handles different adaptive Bernstein orders per hold.
- For uncertain target-free admission, select a certifiable shorter horizon
  within the configured minimum when a longer open-loop tube cannot enter the
  terminal set. A larger subsequent measurement bound triggers a new complete
  admission, explicitly marked as a changed sensing contract. It does not
  inherit a recursive-feasibility claim for the old bound.

The high-gain observer, predictive CBF, squared soft CLF, operating-point input
cost and continuous whole-hold checking remain. There are no artificial road,
state or slip constraints, no execution subdivisions and no fallback commands.
The ordinary-distance query, its obsolete diagnosis driver and inert solver
form/verification selectors have been removed. The controller remains within
its 20-source limit. Native generated files stay outside the tracked source.

## Reproduction

Build updated geometry interfaces once:

```matlab
addpath('scripts');
buildAvoidanceGeometryKernel;
```

Run the persistent validation driver, which saves startup, controller and joint
results separately and uses one MATLAB computational thread:

```matlab
addpath('scripts');
runOverlapAdmissionValidation(OutputDirectory='/absolute/output/directory');
```

The straight controller scenes use seed 20260912, 300 holds of 0.1 s, reference
speed 8 m/s and a complete circular confirmation region of radius 16 m. The
stationary target starts at (15,0) m; the oncoming target starts at (60,0) m
with velocity (-8,0) m/s; the crossing target starts at (15,-4) m with velocity
(0,32) m/s. Both footprints are 4.8 by 1.9 m and clearance is 0.25 m. No road
boundaries are configured. An independent 11-point per-hold truth audit is
reported separately from the continuous certificate and from online timing.

All executed commands must pass the complete physical certificate. The frame
includes measurement assembly, search, all native solves, verification and
output; truth integration and offline audits are excluded. MATLAB/JIT startup
is exercised separately using discarded runs, with no state or command reused.
No empirical timing result is a worst-case execution-time proof.

## Exact-state controller results

The final independent periodic measurement after six discarded two-hold warmups
passed all four 30 s scenes. Artifacts:
`/home/zai/.cache/collisionAvoidance/admission-implementation-20260916/final-validation/controller/`.

| Scene | Executed holds | Minimum sampled excess over 0.25 m clearance | Maximum complete frame |
| --- | ---: | ---: | ---: |
| Stationary | 300/300 | 0.0521195 m | 85.780 ms |
| Oncoming | 300/300 | 0.121145 m | 64.418 ms |
| Crossing | 300/300 | 9.30160 m | 15.460 ms |
| Cruise | 300/300 | No target | 16.241 ms |

Each obstacle encounter obtained confirmed release. Every executed hold was
hard certified, with no terminal command or fallback. At 30 s, all four speed
errors were approximately 1.59e-7 m/s; lateral and heading errors were below
1e-17 in these deterministic trials. This is measured eventual recovery,
without an artificial early-recovery deadline. A soft CLF alone does not prove
asymptotic convergence for arbitrary persistent positive slack.

The first cold two-hold run reached 1280.283 ms; subsequent startup maxima were
160.726, 151.137, 89.848, 110.335 and 86.004 ms. Those are explicitly excluded
from periodic qualification because no plant executes during preparation.
Running an unprepared MATLAB session under a 100 ms gate is not qualified.

Stationary and oncoming admissions used one restoration program and one final
hard program each; checked constraint generation required three and four
native solves respectively. Their original cruise seeds contained 12 and six
overlapping midpoints; primary restoration values were 0.1398864 and 2.0528400
before geometry refinement. Positive search values were never execution
slacks. Crossing and target-free trials did not need restoration.

A stationary admission microbenchmark improved from approximately 115 ms
with complete sparse rows to 80.1 ms with checked constraint generation after
warmup, excluding public input/output work. Its component times were roughly
32.6 ms initial formulation, 10.8 ms restoration, 14.5 ms whole-hold direction
scoring, 10.8 ms geometry rebuilding and 11.4 ms final hard solving. These are
diagnostic microbenchmarks, not substitutes for complete-frame measurements.

## Joint observer-controller finding

The joint run uses actual NRMM output, seed 20260913, the same 8 m/s oncoming
scene and 16 m visibility region. No truth replaces an estimated state or
uncertainty bound. A 5 s diagnostic frame allowance retains the actual 100 ms
period in timing reports; it is not a real-time pass.

Two target-free integration defects were identified and repaired. First, a
mandatory long open-loop uncertainty horizon produced negative terminal SOC
radii before any target was visible; a shorter complete terminal witness was
feasible. Second, normal variation of the published measurement radius was
rejected at the interface. Enlarged bounds now require full fresh admission;
they cannot silently inherit the old terminal sensing certificate.

After these repairs the joint run executed 28 holds and reached first target
publication at 2.8 s. Admission then failed under the full hard certificate.
The observed target radii were approximately:

| Quantity | First-detection component radii |
| --- | --- |
| Position | (0.1455, 1.0203) m |
| Velocity | (10.7148, 20.0499) m/s |
| Acceleration | (2.4383, 2.4563) m/s² |
| Heading / yaw rate | pi rad / 0.0746 rad/s |

An offline audit of this observed state, reset as a fresh admission, found:

| Holds | Minimum terminal SOC radius | Best exit-row margin over an actuator box |
| ---: | ---: | ---: |
| 4 | 0.16774 | -31.494 m |
| 8 | 0.069388 | -28.037 m |
| 12 | -0.029271 | -23.689 m |
| 24 | -0.32525 | -5.5638 m |
| 32 | -0.52257 | 10.488 m |
| 64 | -1.6366 | 101.82 m |

A negative SOC radius makes that terminal cone impossible for every control.
A negative exit margin makes the tested exit row impossible even over the
actuator outer box. These diagnostics concern the tested horizons and selected
exit direction; they are not a proof of physical collision inevitability or
infeasibility for every possible nonconvex policy. The audit resets admission
and does not reproduce the prior conditioned information state exactly.

The final diagnostic bounded search reached a minimum normalized restoration
deficit 62.0756 over 14 attempted families before its 3 s search budget expired.
No restoration iterate was issued. The remaining joint limitation concerns
initial target uncertainty and the open-loop terminal/exit certificate, not
ordinary-distance initialization. Smaller fabricated error bounds, favorable
future measurement resets, relaxed execution collision rows or an executable
fallback would not be valid fixes. A future extension needs a proved sensing-
aware feedback continuation or tighter justified acquisition bounds.

Joint artifacts and diagnostics are under the same cache root in
`joint-functional/`, `joint-repaired/`, `joint-conditioned/`,
`joint-admission-audit.mat` and their named logs. This report does not claim
joint avoidance or joint real-time success.

## Validation scope

Behavior tests cover outside/overlap/contact geometry, successful stationary
and oncoming admission from cruise seeds, no execution from restoration alone,
initial collision rejection, sparse row/objective equivalence, inherited
witness/deadline preservation, adaptive Bernstein order, target-free uncertain
admission and changed measurement contracts. Native support/projection parity,
partial target release, curved continuation and the core source budget remain
part of the repository regression suite.

The current mathematical scope is documented in
[SUPPORT_CONVEXIFICATION.md](../controller/SUPPORT_CONVEXIFICATION.md) and
[TERMINAL_CBF_PROOF.md](../controller/TERMINAL_CBF_PROOF.md). The complete regression result was **621 passed, zero failed, zero incomplete**
in 552.733 s. The full command was
`matlab -batch "results=runtests('tests'); assertSuccess(results)"`; the executed
batch also saved the result objects. Code Analyzer found no code errors; it
reported bounded candidate-array growth and sparse indexed-assembly performance
advisories, plus a local missing-settings warning with fallback to defaults.
An unused rotation helper was removed after this analysis, followed by **20
passing focused admission/native/source-budget regression cases**. Local documentation links and Git whitespace
checks pass.


## Same-sensing comparison and enforced joint deadline

The original exact-state driver supplies targets to the controller even outside
the sensing region; the controller omits an encounter when its entire robust
footprint is exterior. Its oncoming admission starts at 2.6 s. The actual radar
adapter publishes only after the target center is in range, at 2.8 s. These
visibility rules must not be conflated.

The final driver therefore also ran `UseEstimator=false` through the joint
scenario's identical center-based visibility gate. It completed all 300 holds
with no deadline misses and a maximum frame of **68.224 ms**. Every issued
hold was certified; the minimum sampled excess clearance was 0.359272 m, and
the final speed error was about 1.59e-7 m/s. Thus later
publication alone did not prevent exact-state avoidance in this experiment.

With actual observer output, the diagnostic run completed 28 holds, then failed
admission at 2.8 s. Its maximum frame was 3052.551 ms, with three 100 ms misses
under the longer offline allowance. An independent reset with the actual
100 ms gate again completed 28 holds and failed at 2.8 s; the failed frame took
**105.761 ms**, with one deadline miss. Maximum observer times were 67.581 ms
in the first diagnostic run and 23.204 ms in the subsequent periodic run.
The controller receives only the remainder of the full frame budget after
observer preprocessing. Budget checks and native time limits do not provide
preemptive scheduling of MATLAB formulation; the late failed frame issued no
new command. Neither joint completion nor joint realtime is qualified.

The machine-readable [validation summary](OVERLAP_ADMISSION_VALIDATION_20260916.json)
includes every startup maximum, all scene outcomes, seeds/options in the linked
raw artifacts, and both unsuccessful joint attempts. The original complete
logs and trajectories remain under the cache root's `final-validation/`.
The intermediate raw artifacts document the diagnosis and must not be confused
with the final independent measurement.
