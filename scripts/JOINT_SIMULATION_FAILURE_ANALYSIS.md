# Joint simulation failure analysis

Prepared September 7, 2026. Diagnostic source revision:
`965a50e9a83244dab03ab6441940ebab6b6be139`.

The recorded joint trials fail during admission or certificate construction,
before executing a control interval. They do not demonstrate a collision,
estimator numerical divergence, failed asymptotic cruise recovery, or a solver
proof of infeasibility. Independent saved-input ablations identify several
distinct integration restrictions. No production algorithm, sensor bound,
objective, or stop-on-failure policy was changed for this analysis.

## Evidence and scope

The primary saved straight/circular trials requested 30 s, with ego and target
speeds of 10 m/s, 30 m radial perception, controller/observer periods of
0.05/0.0125 s, bounded uniform noise, and seeds 20260907/20260908. The circle
radius was 400 m and its forward arc length was 600 m. The target initially
lay outside perception at an initial-distance parameter of 100 m. Both trials
stopped at time zero with zero commands. The acquired-target probes used a
23 m initial-distance parameter and seeds 20260909/20260910. They also stopped
at time zero. Their initial estimates and current uncertainty certificates
were finite and available.

Original results are retained in
`/home/zai/.cache/collisionAvoidance/joint-v9-20260907-133705`.
Current diagnostic harnesses and MAT/JSON outputs are retained in
`/home/zai/.cache/collisionAvoidance/failure-analysis-20260907-140145`:
`runFailureAblations.m`, `ablations.mat`, `ablations.json`,
`runFailureFollowup.m`, `followup.mat`, `followup.json`,
`observer-bound-summary.json`, and `observer-cap-diagnostics.json`.
Export copies are preserved in the shared external research archive.

The ablations call the public controller with explicit stored-certificate
arguments and catch failure identifiers. Hypothetical sensor coverage, zero
uncertainty, and supplied exit data are diagnostic interventions only. They
do not establish physical feasibility or justify changing the real bounds.
The 1.5 s follow-up runs the observer against prescribed straight truth;
it does not execute the vehicle plant or controller.

Two cache-harness interface mistakes were corrected or superseded: an initial
extended-boundary probe used the unsupported coverage policy `declared`, and
the follow-up initially attempted to read a raw estimate's nonexistent
`position` field. The corrected probes use the actual road fitter and
`readPlanningInputs`. Neither harness error is a controller failure finding.

## 1. Straight-road coverage and the prediction horizon are inconsistent

The default prediction covers 48 intervals, or 2.4 s. Road fitting uses only
curb points inside a 30 m radial sensor range. The left outer curb is 10.6 m
from the centerline, so its forward longitudinal visibility is already less
than 30 m. The saved fitted interval ends at parameter 27.977068 m.

`avoidanceSafetyGeometry` requires coverage of the complete cell, including
station trust, propagated error, and the vehicle footprint. The first failed
cell starts at 2.266667 s and requires an upper parameter of 28.087978 m:
a 0.110910 m deficit. Reconstructing all 144 cells finds the maximum deficit
in the last cell, starting at 2.383333 s: it requires 29.281115 m, leaving
a **1.304047 m** gap. The first gap is not the maximum horizon gap.

Controlled saved-input interventions give the following results:

| Intervention | Result |
| --- | --- |
| Keep 30 m sensing and 48, 47, or 46 intervals | Road coverage rejection |
| Keep 30 m sensing and use 45 intervals (2.25 s) | First call accepted |
| Use 44, 40, 32, 24, or 16 intervals | First call accepted |
| Keep 48 intervals; refit hypothetical 35 m road sensing | First call accepted |
| Keep 48 intervals and 30 m sensing; set all ego radii to zero | Road coverage rejection remains |

This isolates a finite geometric coverage condition. It is not evidence that
the vehicle has crossed a curb, and it is not caused solely by observer error.
A compatible prediction horizon must account for whole-cell visibility;
changing it also affects encounter admission and requires later validation.

## 2. Repeated perception updates conflict with stored-certificate identity

The controller stores an identity containing the entire configuration except
solver settings, lane, road, and acceleration bias. On the next call it uses
`isequaln` to require exact equality. The scenario driver independently refits
road boundaries at each estimated pose. The fitter changes its local origin,
visible interval, and fitted data even on the same physical straight road.

An exact-model two-call experiment isolates this conflict. With a 40-interval
horizon, the first saved straight input is accepted. Its certified next state
is then used at the next sample, eliminating physical-model mismatch:

| Second-call road and certificate | Result |
| --- | --- |
| Original road, carried certificate | Accepted |
| Newly fitted road, carried certificate | `changedExecutionContract` |
| Newly fitted road, fresh admission | Accepted |
| Original road with a coefficient changed by only 1e-12, carried certificate | `changedExecutionContract` |

The boundary origin changes from x = 0.022932 m to x = 0.524202 m in the
refitted case. Thus removing the initial coverage failure alone is insufficient
for repeated perception-driven operation. This is a saved-input two-call
result, not an executed two-step physical simulation.

Persistent physical route/model identity must be distinguished from updating
measurement representations, with coordinate-consistent certificate checks.
Blindly resetting controller state would discard active encounter obligations
and is not a justified repair. The identity check also applies with no targets.

## 3. Circular-road representation fails two separate chart conditions

The circular reference is represented by approximately 0.1 m straight
segments. At the saved initial input, a position box of plus/minus 0.04 m
produces three candidate noncollinear projection segments.
`stateUncertainty.toFrenet` requires one interior chart, with a special allowance
only for consecutive collinear segments. The timestamp is valid; the chart
condition causes `invalidUncertaintyChart`.

Setting only the position radii to zero bypasses that initial condition, but
then `avoidanceSafetyGeometry` rejects nonzero frame heading variation across
a prediction cell: `unsupportedReferenceJump`, wrapped in
`noCertifiedContinuation`. Setting every ego radius to zero has the same
later result. The 2 m station trust region spans many polyline corners.

This is an implementation support restriction, not an intrinsic inability of
Frenet coordinates to describe the circle. For the exact radius-400 m circle,
the local Frenet Jacobian factor over the declared lateral domain satisfies
`1 - curvature*d >= 1 - 12/400 = 0.97`. The smooth circle is locally regular
throughout that strip. A smooth reference and corresponding whole-cell error
analysis are needed; merely deleting the guards or shrinking the sensor error
is unsupported. MathWorks documents smooth waypoint-based Frenet references
as one representation option, including the distinction between C2 fitting
from positions and C1 fitting when headings are supplied:
[referencePathFrenet documentation](https://www.mathworks.com/help/nav/ref/referencepathfrenet.html).
Using that object alone would not supply this controller's required proofs.

## 4. Encounter exit means leaving the whole allowed route, not leaving perception

The acquired straight target first fails `missingEncounterContract`.
The adapter publishes current state/error bounds but does not supply a future
motion/exit contract. This is more than a missing struct field.

`targetPrediction.admit` requires a `nonreturningHalfspace` whose exit offset
is beyond the support of **all route segments**, the full lateral domain, and
the ego footprint. `exitSchedule` then requires the entire uncertain target
footprint to clear that plane within the finite horizon. This imposes a
post-exit nonreturn premise beyond the local constant-motion assumption.

The straight scenario's route spans x = -200 to 300 m with a 12 m lateral
domain. Its observed oncoming target starts near [22.962168, 0.818821] m
and moves near [-9.999952, 0.031125] m/s. Over the next 2.4 s its nominal
center remains inside that route strip. For any separating normal, the center
therefore cannot exceed the support of the whole allowed route, even before
adding footprint and clearance. Passing the ego does not satisfy this exit.

A numerical scan of 721 normals and 97 times corroborates that geometric
argument: the best exit margin is -15.056838 m even with exact target states;
with the current bounds it is -17.368053 m. The scan itself is not a proof
over continuous normals or times. For the rearward longitudinal normal,
the nearest admissible offset is 202.692583 m; clearing it would take about
22.840586 s under nominal constant velocity, versus the 2.4 s horizon.
A hypothetical contract using that plane reaches `noCertifiedExit`.

Filling `encounterContract` alone cannot repair this oncoming experiment.
The intended finite perception encounter and the implemented globally disjoint
route exit need a consistent specification and certificate. Any guarantee
after the defined encounter must be stated separately; the user's local
motion assumption does not imply perpetual nonreturn or a prescribed target
road corridor. The present controller also retains active obligations when
measurements disappear, rather than treating disappearance as discharge.

## 5. Target point estimates are much tighter than the control error envelope

At the acquired straight input, actual target velocity error is 0.031125 m/s,
but each published Cartesian velocity radius is 30.203224 m/s. Position and
acceleration radii are 0.551708 m and 2.358493 m/s squared. Target heading
radius is pi, providing no useful initial directional restriction.

Initialization uses 28 consecutive radar detections from -0.3375 to 0 s,
estimating direction from the position window and using a configured 10 m/s
speed magnitude. However, the runtime enclosure starts from the declared
physical domain: target speed maximum 20 m/s plus estimated speed 10 m/s.
Rotation uncertainty adds approximately 0.203224 m/s. The observation history
does not supply a tighter certified initial velocity prior through this adapter.

The extra observer-only experiment uses the saved integration configuration,
seed 20260909, ego position [10t, 0] m, target position [23-10t, 0.8] m,
and the same periods/noise, for 1.5 s. Across all 31 controller-time outputs:

| Quantity | Observed range |
| --- | --- |
| Actual target velocity error norm | 0.031125 to 0.548845 m/s |
| Published component velocity radius | 29.705570 to 30.542921 m/s |
| Actual target position error norm | 0.002233 to 0.042255 m |

Every velocity radius matches the physical-domain cap plus the rotation
term to within 9.1e-13 m/s. The sampled comparison bound does not improve
on that cap during this experiment. Waiting 1.5 s therefore does not resolve
this enclosure limitation. Current bound availability remains true, while
the reconstructed estimated-domain validity fraction is 22/31 (0.709677).
Those flags have different meanings; finite available bounds do not establish
that every point estimate lies inside the nominal reconstruction domain.

Even a zero-jerk propagation of the initial Cartesian box gives
`positionRadius(t) = positionRadius(0) + t*velocityRadius(0) + t^2*accelerationRadius(0)/2`.
The component position radius is 15.948132 m after 0.5 s, 31.934178 m after
1 s, and 79.831905 m after 2.4 s. These are enclosure radii, not measured
errors or actual vehicle motion. The initial contract rejection occurs first;
the large tube is an additional quantified obstacle to robust planning, not
an observed downstream solver infeasibility result.

History-conditioned initialization and tighter sampled error propagation
deserve investigation. Any replacement prior must follow from allowed
measurements and declared motion/noise assumptions; it cannot be set from
simulation truth or reduced solely to make the optimizer succeed.

## Repair order and validation limits

1. Make straight-road finite coverage compatible with planning and separate
   persistent route identity from refreshed road measurements.
2. Support smooth curved-reference charts and certify their cell variation.
3. Align target admission/discharge with the specified finite encounter;
   retain explicit scope and re-detection behavior without inventing a global
   target route or nonreturn assumption.
4. Derive useful, valid target velocity/acceleration enclosures from history
   and sampled observer propagation, then test coupled operation.
5. Only after actual closed-loop execution, assess collision clearance,
   eventual nominal cruise recovery, and per-cycle computation deadlines.

Separate latent issues remain to be validated: the default circular reference
has zero yaw-rate offset although the ideal 10 m/s, 400 m centerline motion
requires 0.025 rad/s; the default LTV and physical-plant residual rate bounds
are zero. These are consistency/proof-scope checks, not the observed first
failure causes, and this investigation does not establish physical residual
validity or design a replacement curved-road equilibrium.

No new unit regression run was needed for this research-document-only change.
The experiments above are the new validation. The previously archived
135-test current-policy pass is historical component evidence, not a new run
or evidence of successful joint avoidance. Stop-on-failure remains in force,
and no fallback, arbitrary early-recovery criterion, or algorithm repair was
introduced by these diagnostics.
