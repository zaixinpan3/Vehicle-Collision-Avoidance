# Failure mechanisms in the nonlinear, estimator-fed, and S-curve environments

The three environments fail for different reasons. The truth-fed nonlinear
vehicle exposes a large mismatch between the cruise-linearized prediction and
the executed avoidance response. The estimator-fed runs fail earlier, when
startup motion uncertainty prevents target admission. The S-curve driver is
rejected at its reference interface before any vehicle hold executes; a second
target-interface defect is also reproducible. None of these observations proves
that a physically safe maneuver is impossible.

This analysis follows [the recovery campaign](CONTROLLER_RECOVERY_VALIDATION_20260925.md).
It adds 56 offline failure-frame counterfactuals, four reference-interface
probes, an audit of 88 executed nonlinear holds, and 19 passing existing
reference tests. It changes no production controller, estimator, plant, or
scenario behavior. A solver-accepted counterfactual is not a completed drive.

## Method and evidence

`scripts/analyzeControllerRecoveryFailures.m` loads the original MAT results,
normalizes the saved failure inputs with `readPlanningInputs`, prepares a fresh
admission, and uses the production formulation and finite direction search.
Each replay has a 30 s search allowance and no frame deadline. It is a fresh
admission diagnostic, not an identical replay of the original shifted-plan
search. The saved target, road, actuator history, configuration, and measurement
bounds are retained except for the explicitly named ablation. No resulting
command is applied to a vehicle. Results are deterministic inputs from the
September 25 campaign (estimator seed 20260925, 10 m/s reference and target,
50 ms controller interval, 30 m detection range).

Compact outputs are in `CONTROLLER_FAILURE_CAUSES_20260925/`. Original input
MAT files remain in `/home/zai/.cache/collisionAvoidance/recovery-validation-20260925/`;
56 output MAT files remain in
`/home/zai/.cache/collisionAvoidance/recovery-failure-analysis-20260925/`.
The source and artifact manifest identifies the versions actually analyzed,
including pre-existing working-tree scenario edits. These edits are excluded
from this diagnostic commit.

| Failure-frame change | Truth straight | Truth circular | NRMM straight | NRMM circular |
| --- | --- | --- | --- | --- |
| Fresh, unchanged inputs | Reject | Reject | Reject | Reject |
| Remove target | Accept | Accept | Accept | Accept |
| Remove road rows and terminal clearance | Reject | Reject | Reject | Reject |
| Use preceding prediction as actual successor | Accept | Accept | Reject | Reject |
| Zero ego measurement bounds | Reject | Reject | Reject | Reject |
| Zero target bounds and published parameter bounds | Reject | Reject | Accept | Accept |
| Exact target motion; keep position uncertainty | Reject | Reject | Accept | Accept |
| Zero target yaw bound only | Reject | Reject | Reject | Reject |
| Zero velocity bound; recompute parameter bounds | Reject | Reject | Accept | Reject |
| Above, also zero yaw and yaw-rate bounds | Reject | Reject | Accept | Accept |
| Above velocity change, also zero acceleration bounds | Reject | Reject | Accept | Accept |
| Tighten speed/course from the Cartesian velocity box | Reject | Reject | Reject | Reject |

The CSV also includes road-row-only and all-bounds-zero variants. Motion
ablations clear published parameter bounds so the controller reconstructs them
from the modified state box. They therefore diagnose the motion enclosure as
a whole, rather than only one independent scalar. Zeroing a real uncertainty
is not a safe implementation recommendation.

## 1. Truth-fed PassVeh14DOF: avoidance relies on excessive predicted response

Both original runs stop at 2.20 s, after 44 executed holds, while their executed
prefixes still have positive obstacle separation. Basic no-target cruise
finishes 18 s. At the failed state, removing the obstacle restores feasibility;
removing the road constraints does not. Most decisively, replacing the measured
ego state with the preceding predicted successor restores feasibility in both
fresh replays. That intervention leaves the target and road unchanged.

The prediction discrepancy becomes particularly large during the hold starting
at 2.10 s. Successor quantities below are at 2.15 s.

| Quantity | Straight | Circular |
| --- | ---: | ---: |
| Steering command (rad) | -0.4713 | -0.6241 |
| Front slip computed from current state and command (rad) | 0.4305 | 0.6052 |
| Reported affine front lateral force (kN) | -58.113 | -65.391 |
| Independent same-parameter saturated Fiala force (kN) | -6.202 | -6.505 |
| Affine force / static combined-slip capacity | 9.37 | 10.05 |
| Predicted / actual lateral velocity (m/s) | -1.397 / -0.215 | -1.753 / -0.150 |
| Predicted / actual yaw rate (rad/s) | -1.233 / -0.233 | -1.563 / -0.189 |

The Fiala numbers are an independent diagnostic calculation with the controller
tire parameters, actual state, and issued actuator input. They are **not logged
Blockset axle forces**. Nevertheless, the roughly tenfold force discrepancy and
the directly measured successor errors strongly support prediction outside the
useful tire-linearization range as the principal mechanism. The vehicle does
not develop the lateral response on which the avoidance plan relies. The exact
contributions of tire saturation, load transfer, and other Blockset dynamics
have not been isolated because the saved trace lacks axle-force telemetry.

`ltvBicycleModel.sampledCruise` constructs its affine stage at the cruise trim.
The sampled flow is exact for that declared affine model; that does not make it
exact for the nonlinear vehicle. The formulation limits actuator magnitude and
finite configured slew, while `stateAndSlipBoundsEnforced` is false. Here the
steering magnitude limit is approximately 0.698 rad and the configured steering
rate limit is infinite. Thus a large sudden input can be feasible in the
declared model while demanding an unrealistic lateral response. Successful
avoidance under the same affine equations cannot expose this mismatch.

There is also a recursive-feasibility limitation in this particular harness:
both 44-hold runs have **43 road refits and zero inherited feasible families**.
In `hardEncounterBarrier.prepare`, changed fitted road boundaries trigger a
fresh admission seeded by the shifted plan, before the observed-successor
consistency check. Consequently, zero explicit inconsistent-observation flags
do not mean that the nonlinear vehicle followed the model. The old feasibility
witness is continually discarded. This explains why its theoretical transfer
cannot protect these executions; it does not establish that road refitting
alone causes the final failure. Removing road constraints only at the final
frame still fails, and the nonlinear model mismatch would remain without them.

The horizon is not simply the nominal 0.8 s: encounter completion extends the
last accepted original plans to 56 stages (2.8 s). The earlier 32-nominal-stage
diagnostics actually use 64 stages (3.2 s) near failure and still stop at 2.20 s,
despite a 30 s search allowance. A modest horizon extension and more solve time
are therefore insufficient in these trials.

**Engineering implication:** first validate an avoidance prediction that
respects tire saturation, combined-slip limits, realistic actuator slew, and
measured successor errors. A robust residual tube would require a justified
bound and a compatible certificate; the current zero-residual contract cannot
silently be reinterpreted as such. Preserve road-set meaning across refits, or
explicitly recertify changes and account for loss of the carried witness. Merely
changing solver weights does not address the demonstrated model error.

## 2. PassVeh14DOF plus NRMM: startup uncertainty defeats admission

These runs stop at 1.05 s, on the first controller-visible target frame. Actual
sensor acquisition is earlier, at **1.0125 s**. The 80 Hz observer has only
**three retained position-history samples** by publication, a 37.5 ms tracking
interval. It is inaccurate to say there is only one sample at failure, or to
use the completed-prefix summary to conclude no acquisition occurred.

The point estimates are sensible: 10.030 and 10.035 m/s versus a 10 m/s target.
Their uncertainty contracts are much less informative:

- `nrmmPositionErrorBound.localReset` initializes a velocity-error ball using
  the declared 20 m/s target speed maximum plus the estimate norm, giving
  approximately 30.03 m/s. The 10 m/s initialization prior selects a point;
  it is not an asserted true-speed error bound. Acquisition resets history.
- `nrmmControllerErrorBounds` intersects the Cartesian state enclosure with
  short measurement history, but also publishes NRMM parameter bounds from
  component balls. In `nrmmTargetParameterErrorBounds`, a velocity radius larger
  than the point speed gives an uninformative course radius of pi.
- The controller further intersects these contracts. Its final speed interval
  is **[0, 21.061] m/s** in the straight case and **[0, 21.332] m/s** in the
  circular case, with a full 2-pi course interval. It is not simply [0, 40].
- At a 1 s prediction time, the norm of the target position-box half widths is
  **47.19 m / 47.93 m**. These are conservative enclosure diagnostics, not
  physical disks of reachable positions or proof that every enclosed point is
  reachable. Their size relative to the approximately 24 m current body gap
  illustrates why a robust separation certificate is difficult to construct.

Counterfactuals localize the issue. Removing ego uncertainty, changing to the
predicted ego successor, or removing the road does not restore admission.
Removing target motion uncertainty while retaining target position uncertainty
does restore it in both cases; the 1 s radius norms become 1.38 / 1.42 m.
Removing velocity uncertainty and recomputing parameter bounds suffices for
the straight case. The circular case also needs either the yaw/yaw-rate or the
acceleration uncertainty removed in these probes. This is an interaction of
motion and footprint uncertainty, not evidence of a unique defective yaw gain.

There is avoidable conservatism worth investigating. Direct velocity-box
geometry gives positive speed lower bounds 2.87 / 2.08 m/s and course widths
2.48 / 2.61 rad, instead of a full circle. The implemented tight-box probe
intersects those intervals with the existing contract, preserving the original
velocity box. **Both cases still fail**, with 1 s radius norms 45.58 / 47.58 m.
Improving this conversion alone is not enough; future parameter dependence
and remainder growth also require investigation.

**Engineering implication:** assess admission using acquisition age and
defensible motion information. Earlier sensing, more informative velocity or
direction measurements, or an explicitly justified motion prior can improve
the initial contract. Use set representations that retain velocity-box and
parameter dependencies, then measure enclosure growth. Do not hide a detected
target until estimates mature, or reduce published bounds just to make the
solver pass. The observed failure is not evidence that the NRMM point estimate
diverges; it is failure to find a certificate for the published startup set.

## 3. S-curve: two stale scenario interfaces, not a demonstrated avoidance failure

The original run is rejected at t=0 with `unsupportedReferenceJump` in
`hardEncounterBarrier.localTerminalSet`. The scenario constructs a dense
polyline and `curvatureLaw`, but does not supply `geometry.referenceCurve`.
`runCenterlineCruiseScenario` only forwards that analytic reference when the
field exists. A polyline with changing segment tangents cannot pass this
reference contract.

The controller already supports scheduled varying curvature through an explicit
`referenceCurve.curvatureProfile` and continuation rule. The probe supplies
the existing sinusoidal curvature law, sampled every 0.1 m over 220 m:
`kappa(s)=0.01*sin(2*pi*s/80)` through 80 m, zero thereafter, with constant
terminal curvature continuation. **The first frame passes with the original
road boundaries retained.** Removing boundaries also passes. All 8 scheduled
reference tests and 11 smooth-geometry tests pass.

A second issue is exposed by deliberately supplying the legacy target at this
corrected-reference first frame: `missingPredictionMotion`. The S-curve
driver's `localTargetState` lacks the explicit `nrmm-motion-v1` prediction
contract required by `targetPrediction.admit`, unlike the current straight and
circular drivers. This is a controlled interface probe; the original run never
got far enough to encounter that second exception.

**Engineering implication:** update the scenario's reference and target
contracts, verify consistency between sampled path, curvature profile, and
reference phase, then run the full nonlinear encounter. Passing the corrected
first frame does not demonstrate S-curve tracking, successful avoidance, or
return to cruise. The nonlinear-model issue above may still appear afterward.

## Limits, checks, and next experiments

The direction search is finite. Fresh truth-fed rejections end with restoration
slack about 0.00546 / 0.02115; NRMM rejections about 89.90 / 123.57, after six
restoration solves. These are residuals of the selected subproblems, not global
infeasibility proofs, comparable physical distances, or minimum attainable
safety violations. Successful variants may also retain an earlier nonzero
restoration slack before a later accepted solve.

Some uncertainty-removal probes produce near-singular matrix warnings inside
`ltvBicycleModel.reactionGains` (reported RCOND approximately 1.8e-16). Treat
acceptance as the production solver's result; this experiment does not establish
well-conditioned feedback synthesis for zero-noise degenerate inputs. All 56
replays produce records with no caught exceptions. The separate four reference
probes produce the two expected errors and two first-frame acceptances.
Code Analyzer finds no source issues in the analysis script; its only notice
is an unreadable old R2025b settings file, so defaults are used. The 19 reference
tests pass. The original 92-test campaign is not claimed as a fresh rerun here.

Recommended order of follow-up work:

1. Repair the two S-curve harness contracts and establish a runnable baseline.
2. Instrument physical axle forces and slip, compare one-hold predictions over
   realistic avoidance inputs, and correct the plant/certificate mismatch before
   drawing nonlinear safety conclusions. Test road-refit handling separately.
3. Sweep acquisition distance, history age, and justified target information;
   report both truth containment and admission success, including future-set
   growth. Preserve safety accounting during initial acquisition.
4. Re-run full encounters and apply the original continuous-final-2-s speed,
   lateral-error, and course-error recovery criteria. None of the diagnostic
   acceptances above substitutes for that final test.

Reproduce the offline analysis and relevant regressions from the repository root:

```matlab
addpath('scripts');
rows = analyzeControllerRecoveryFailures( ...
    '/home/zai/.cache/collisionAvoidance/recovery-validation-20260925', ...
    '/home/zai/.cache/collisionAvoidance/recovery-failure-analysis-20260925');
assert(numel(rows)==56 && all(strlength([rows.identifier])==0));
results = runtests({'tests/scheduledReferenceControllerTest.m', ...
                   'tests/smoothReferenceGeometryTest.m'});
assertSuccess(results);
```
