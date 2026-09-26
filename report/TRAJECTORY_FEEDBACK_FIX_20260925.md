# Match prediction feedback to trajectory dynamics — September 25, 2026

The identified cruise-feedback/trajectory-dynamics mismatch is repaired.
All three previously failed frame snapshots now admit feasible plans with
their arithmetic reserves and physical constraints intact. All 257 selected
MATLAB tests pass. New nonlinear closed-loop runs still fail later, before
passing the target or recovering cruise; this report does not claim a complete
nonlinear avoidance repair.

## Implementation

Active trajectory-linearized encounters now call
`ltvBicycleModel.trajectoryFeedbackGains` after constructing their held affine
stages. It designs a separate feedback gain for each future hold through a
backward finite-horizon quadratic-cost recursion using the actual discrete
A/B matrices. State/input weights are the existing CLF error scales and LQR
input weights; the final metric comes from the corresponding final cruise
reference, including the scheduled metric on a varying-curvature path.

The existing speed-column scaling and lateral-velocity feedback switch are
applied before updating the cost-to-go. Station feedback remains zero. The
first held input is exact, so its gain is zero. This is a restricted
finite-horizon design, not an unrestricted infinite-horizon stability proof.
No claim is made that it stabilizes every possible trajectory or bounds a
nonlinear plant's mismatch.

`finitePredict` now uses the gain sequence consistently for closed-loop error
propagation, estimator-noise injection, input reserves and slew reserves.
Target-reactive design starts from the corresponding stage's base gain.
Carried policies retain their stored sequences; current-hold feedback metadata
reports the actual gain. The legacy `prediction.feedbackGain` remains the
cruise reference, while `feedbackGainSequence` defines the actual policy.

Cruise-policy and target-free studies retain their existing cruise feedback.
The terminal continuation controller is unchanged. State contract version 46
rejects older controller states. No dynamics-relinearization loop, trust
region, tire-force constraint, nonlinear acceptance step, relaxed physical
constraint, or removal of arithmetic reserve was introduced.

## Recheck of the original failed frames

Loaded the exact model/input anchors captured in
[the preceding diagnosis](TRAJECTORY_FAILURE_DIAGNOSIS_20260925.md).
Discarded any stored `reactionDesign` from the old search so all feedback is
designed anew. Rebuilt the same encounter formulation and ran the existing
fixed-direction search. All three return a feasible result with Clarabel
status 1: one hard solve for straight and S-curve, five for circular.

| Captured frame | Previous maximum lateral bound (m) | Repaired maximum lateral bound (m) | Repaired maximum steering reserve (rad) | Feasible admission |
| --- | ---: | ---: | ---: | --- |
| Straight, 1.75 s | 54.0719 | 5.5937e-7 | 4.9134e-8 | Yes |
| Circular, 1.40 s | 2.9199e26 | 1.8676e-5 | 1.6321e-6 | Yes |
| S-curve, 1.30 s | 8.6954e6 | 1.1499e-6 | 1.0960e-7 | Yes |

The repaired gains are nonzero and vary along the horizon. Feedback remains
enabled. These results resolve the previously contradictory steering reserves
and S-curve pose-domain rejection for those exact saved frames; they are not
a complete closed-loop success claim.

## New nonlinear closed-loop runs

Reran the same three truth-fed PassVeh14DOF drivers using
`runTrajectoryLinearizationValidation`, with Duration=18 s, ego/target speeds
10 m/s, RecoveryWindow=2 s, 50 ms control periods and existing road, tire,
perception and solver settings. These exact-sensing runs use no stochastic
sensor seed. The S-curve remains 0.01 /m peak curvature and 80 m wavelength.

| Scenario | Earlier stop (s) | New stop (s) | Executed holds | New failure | Passed / recovered |
| --- | ---: | ---: | ---: | --- | --- |
| Straight | 1.75 | 2.05 | 41 | `optimizationFailed` | No / No |
| Circular | 1.40 | 1.50 | 30 | `invalidTireOperatingPoint` | No / No |
| S-curve | 1.30 | 1.40 | 28 | `optimizationFailed` | No / No |

All three first see the target at 1.05 s; accepted trajectory-linearized holds
are 20/9/7. Minimum prefix SAT margins are 4.0198/15.3607/17.1810 m, with
road-function margins 7.3918/7.5284/6.9626. Positive clearance of a prematurely
terminated prefix does not establish successful avoidance. Runtime deadlines
are not enforced in these offline plant experiments.

### Check that the new failures are not the old enclosure explosion

Replayed each new run from its saved observation sequence, with the recorded
thread setting and a freshly rebuilt model cache. All executed actuator inputs
match exactly, and the failure identifiers/times reproduce.

At the new straight/S-curve rejected programs, maximum lateral bounds are
9.7010e-7 and 9.2551e-7 m. Maximum steering-feedback supports are 1.2155e-7
and 3.2722e-7 rad, far below the existing 0.69813 rad steering limit. The
old feedback-authority contradiction is absent. The fixed-direction searches
still report infeasibility; this task has not isolated a minimal inconsistent
set of their remaining geometry/terminal constraints and does not claim the
underlying nonlinear maneuver is impossible.

The circular failure occurs before solving, at a future nonlinear anchor
whose tire slip magnitude is 1.6644 rad (about 95.36 degrees), outside the
existing `abs(alpha)<pi/2` Fiala domain. This is a predicted operating point,
not a measurement of the current tire slip. The validity check remains intact.
The saved point and stack are exported in `remaining-details.json`.

## Tests and scope

All **257/257 selected tests** pass, with zero failed or incomplete final
results. Four new `trajectoryFeedbackTest` methods cover a 64-hold alternating
near-limit beta plan, usable error/actuator reserves with nonzero changing
feedback, 100 sampled noisy held-affine trajectories (seed 20260925), correct
input/slew/state enclosure, configured feedback-column restrictions, and the
explicit disabled-feedback setting. Tests use public prediction APIs.

The selection also covers controller/configuration, continuation, Fiala,
vehicle prediction, scheduled reference/terminal, target-reactive prediction,
NRMM target enclosures, geometry, immediate actuation and cruise recovery.
The complete repository test suite was not run. The long target-reaction
release test, which specifically studies an inherited fixed-model family,
now explicitly uses `LinearizationPolicy="cruise"`; the scenario driver exposes
that option while retaining its trajectory default. Its 80 holds complete.
The initial attempt under the trajectory default failed after four holds with
an invalid tire operating point. That large-initial-error trajectory experiment
is not claimed fixed by this test-scope correction.

Code Analyzer with factory settings finds no issues in seven of eight
modified/new MATLAB files. Seven pre-existing performance suggestions remain
in unchanged portions of `hardEncounterBarrier.m`. The diagnostic breakpoints
were cleared. No controller code changed after the successful checks except
comments/documentation.

## Reproducibility and artifacts

```matlab
addpath('scripts');
summary = runTrajectoryLinearizationValidation( ...
    '/home/zai/.cache/collisionAvoidance/trajectory-feedback-fix-20260925');
```

MATLAB R2026a Update 3 was used. Compact test results, old-frame admissions,
new plant summaries, new-failure diagnostics and Code Analyzer output are in
`TRAJECTORY_FEEDBACK_FIX_20260925/`. Raw original/replayed MAT files and retained
diagnostic helper copies remain in the external cache directory above.
The manifest identifies effective scenario sources, including pre-existing
uncommitted optional-road-boundary edits, and the implementation/test artifacts.

Base revision: `023a215ba4f18603c341a0d8b6d001cd4a47a619`. Unrelated observer,
report and scenario changes, solver dependencies, references and generated
binaries are excluded from this task's commit. The completed scope is the
identified feedback mismatch repair and its validation; overall nonlinear
avoidance and cruise recovery remain unresolved.
