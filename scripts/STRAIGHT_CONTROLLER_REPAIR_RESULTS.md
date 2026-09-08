# Straight avoidance: controller-first diagnosis and repair

Research date: September 7, 2026. Scope: the straight oncoming-vehicle scene.
The controller-only gate precedes the estimator/controller experiment. A failed
current solve terminates simulation before the next plant interval; no fallback
input is executed. A 30 s observation window assesses post-encounter cruise,
without an early-return deadline or a claim of exact asymptotic convergence.

## Final physical results

Both 30 s straight PassVeh14DOF gates pass, with no collision or road-boundary
crossing. The uncontrolled baseline overlaps the target (minimum SAT margin
-1.2 m). Every executed input has a fresh independently verified plan, and no
fallback is used. Recovery errors below are maxima over the final **one second
of dense plant trace**, not early-return criteria.

| Gate | Executed intervals | Minimum sampled SAT margin | Tail speed error | Tail lateral error | Tail body-heading error |
| --- | ---: | ---: | ---: | ---: | ---: |
| Controller only | 600 / 600 | 0.434319 m | 0.006939 m/s | 1.72133e-07 m | 1.46368e-11 rad |
| NRMM + controller | 600 / 600 | 1.550882 m | 0.011929 m/s | 0.00499182 m | 0.000744214 rad |

The controller-only vehicle passes the target at 5.028 s; the joint vehicle passes
at 5.020 s. Their maximum absolute body headings are 0.248879 and 0.334759 rad,
within the retained 0.4 rad domain. Final speeds are 9.993061 and 9.990665 m/s
against the 10 m/s reference. Small physical-model offsets and noise remain;
these finite runs do not demonstrate exact zero-error convergence.

All 600 joint controller-sample ego premise and published-bound audits pass.
The independently checked target state has 60 publications between 3.55 and
6.50 s; all eight position/velocity/acceleration/heading/yaw-rate components
remain inside the published bounds. The target is acquired at 3.5125 s and
subsequently leaves perception. Target velocity RMSE is 0.780190 m/s over
publication, including the biased 15 m/s initialization. Its first-half-second
transient includes a 21.202525 m/s^2 longitudinal acceleration estimate error,
which remains enclosed; after that half second its longitudinal velocity and
acceleration RMSEs are 0.116782 m/s and 0.552616 m/s^2. Bounded errors do not imply
that every point estimate is immediately accurate.

The largest rear-kinematic mismatches are 0.129751 rad/s (pure) and
0.171598 rad/s (joint), below the independently declared 0.25 rad/s premise.
Sampled derivative residual maxima, in `[station,lateral,heading,vx,vy,r]`
units per second, are:

- Pure: `[0.002258,0.002262,0.000445,0.986505,2.580333,2.400173]`.
- Joint: `[0.011187,0.011100,0.000917,2.009933,3.198965,2.884058]`.

Both fit the empirical `[0.2,0.06,0.02,2.5,5,4]` allowance. The finite-difference
checks use 30,654 and 30,651 trace samples; 890 and 891 intervals shorter than
1e-6 s are excluded to avoid dividing numerical timestamps into noisy derivatives.
Endpoint discrepancies divided by sample time are reported separately. Neither
finite sampling nor the exclusions establish a global continuous-time bound.

During these physical runs, controller-call mean/max times are
66.953/152.421 ms (pure) and 68.948/507.583 ms (joint), with every call above
50 ms. The 507.583 ms joint peak occurs at 4.10 s and includes three nonlinear
refinements and 12 solver calls; approximately 287 ms is spent solving and
132 ms formulating. The subsequent exact optimizations and measured replays
are reported below.

## Failure mechanisms and causal evidence

The previous straight exact-observation bicycle trial stopped at 4.95 s. The
original physical controller trace also stopped at 4.95 s, with a 0.596654 m
minimum sampled SAT margin and 0.388541 rad maximum absolute heading; it did not
complete avoidance.
Removing heading-domain or actuator-rate rows from saved failed linear programs
restored feasibility. Removing collision rows alone did not. These are diagnostic
constraint ablations, not accepted controls: an infeasible convex inner problem
does not establish that the physical avoidance task is impossible.

1. **Future uncertainty was represented and allocated poorly.** Repeatedly
   replacing a propagated set with an independent axis-aligned box loses the
   signed correlations of lateral velocity and yaw rate. Using `abs(A)` for a
   continuous comparison system additionally changes stabilizing negative
   diagonals into positive growth. The resulting future heading allowance could
   consume almost the whole 0.4 rad domain and implicitly force an aggressive
   return toward zero heading. This was not a requested early-recovery condition.
   Conversely, allocating the already-known current estimation error together
   with optional future reserves could leave a nominal path too close to a hard
   boundary for a rate-limited steering input to recover.
2. **Set intersections and target forecasts discarded useful information.**
   Re-centering an intersection around a displaced observer point enlarged it
   again. Resetting the entire retained target forecast when one coordinate
   slightly left the new box could reverse its nominal yaw rate: a recorded
   example changed from approximately -0.0632 to +0.0625 rad/s, although the old
   yaw-rate value missed the admitted interval by only about 0.0007 rad/s. This
   moved the longer forecast by approximately 0.85 m. The observer equations were
   not the source of this whole-vector reset.
3. **The ego velocity interface discarded directional information.** Its scalar
   body-velocity norm radius was copied into both component bounds. A current
   GNSS velocity ball rotated over the certified yaw set supplies much tighter
   longitudinal bounds near straight travel. At the same recorded 4.35 s sample,
   the old longitudinal radius was 0.516683 m/s; the independently computed new
   radius is 0.100273 m/s, enclosing the actual 0.029075 m/s error. The lateral
   radius remains 0.514355 m/s. This comparison uses matching measurement,
   estimate and truth timestamps and does not represent a modified closed loop.
4. **Numerical coordinates and future slip admission were inconsistent.** Large
   absolute station coordinates contaminated relative solver scaling despite
   small vehicle-state errors. Centering all auxiliary stage states on the
   anchor trajectory fixes that conditioning without relaxing physical-unit
   acceptance. Future tire rows now linearize the same interval-start-speed
   scheduled slip that is used when a stage becomes current. Nonlinear
   acceptance checks both scheduled slip and physical `atan2` slip.

An intermediate repaired physical controller completed 30 s, but its joint
counterpart stopped at 4.40 s, before the encounter. Exact replay reproduced
its issued inputs with zero difference. All sampled sensor contracts, observer
premises, known published component enclosures and empirical model-residual
checks passed. Its remaining heading/rate conflict therefore could not be
explained by a violated observer bound. Separating mandatory propagated current
uncertainty from optional process reserves, and tightening the GNSS-derived
velocity components, addresses this interface/planning conflict.

## Implemented method

The future planning set retains signed generators:

\[
 G_{j+1}=[A_{d,j}G_j,\operatorname{diag}(q_j)],\qquad
 r_{j+1}=\sum_k|G_{j+1}(:,k)|,
\]

where `q` uses the Metzler comparison matrix with the original diagonal and
absolute off-diagonal entries. The certified executed endpoint initializes the
subsequent planning propagation. The initial estimation generators are also
propagated separately; their future model-domain and tire-slip supports remain
mandatory. Only additional future process support enters the optional allocation.
These nonlinear-trajectory Jacobian calculations are planning reserves, not a
nonlinear reachability or recursive-feasibility theorem.

The optional reserve fraction cap is 0.25. If full-cap allocation is infeasible,
99% of its feasible LP maximum, less the arithmetic guard, leaves interior space
for nonlinear refinement. These are calibrated planning settings. They do not
shrink the current estimator enclosure, model residual, vehicle rectangle,
physical road/collision constraints, or actuator-rate limits. The fractional
interior allocation is deliberately not an exact lexicographic optimum.

Ego and target motion boxes retain the midpoint and radius of their intersection.
A retained target nominal hypothesis is projected componentwise into the new
admissible box. On first admission, unresolved acceleration and yaw-rate
coordinates choose the nearest admitted values to zero; nonzero values remain
when zero is excluded. The target is not required to be straight or occupy a
prescribed road corridor. Current target uncertainty is still used in the
executed-interval certificate, and targets can leave the finite sensing range.

For ego velocity, each component of `R(-psi)*g` is a sinusoid in yaw. The GNSS
rotation enclosure evaluates interval endpoints and contained stationary angles,
then adds the velocity-noise radius. For stale samples it also requires and adds
a finite inertial acceleration envelope times sample age; otherwise this extra
tightening is skipped. Bounds remain about the unchanged point-observer state.

The joint input/state objective and squared CLF slack remain. There is no desired
acceleration target. Road load includes aerodynamic and rolling resistance. Every
issued command requires a newly solved, independently checked plan. The empirical
rear-tire kinematic mismatch allowance in the integration profile is 0.25 rad/s,
identified before final validation; it is not asserted to be an exact identity
of the dynamic vehicle or a globally proved bound.

## Reproduction and validation scope

Both physical gates use `runOncomingVehicleAvoidanceScenario` with `Duration=30`,
`CenterlineLengthAfter=1000`, `ReferenceSpeed=10`, `TargetSpeed=10`,
`TargetInitialLongitudinalDistance=100`, `TargetLateralOffset=0.8`, `TargetWidth=2`,
`ControllerConfiguration=finiteSensingValidationConfig()`, `Plot=false` and
`Report=false`. The first has `UseStateEstimator=false`; the joint gate has
`UseStateEstimator=true` and `estimatorControllerIntegrationConfig()` with seed
20260907. No circular-scene experiment is run in this operation.

The PassVeh14DOF ego and target collision rectangles are 5 by 2 m. Sensing radius
is 30 m, controller period 0.05 s, prediction length 40 stages, certified length
one held interval, heading domain 0.4 rad, steering-rate limit 0.5 rad/s and
braking-ratio-rate limit 2/s. The model residual allowance is
`[0.2;0.06;0.02;2.5;5;4]` in `[station;lateral;heading;vx;vy;yawRate]` derivative
units. The active plant parameters and full saved configuration are retained in
the MAT results. The residual allowance is empirically identified, not a global
continuous-time plant-error certificate.

NRMM uses 0.0125 s observer samples with integration steps at most 0.0025 s.
Bounded synthetic noise limits are GNSS position 0.04 m, GNSS velocity 0.05 m/s,
IMU acceleration 0.03 m/s^2, gyro 0.0015 rad/s, and radar position 0.04 m.
The physical joint run retains the default 15 m/s provisional target-speed prior,
while the true target moves at 10 m/s. The three matched-prior bicycle seed trials
use a 10 m/s prior; an additional 15 m/s-prior bicycle trial also completes 12 s.
This distinction is explicit rather than treating the prior as a velocity measurement.
The unchanged point-observer gains are synthesized from the integration profile.
Truth is used for post-run audits, never as joint-controller input.

Three separate nonlinear-bicycle/NRMM straight trials, seeds 20260907--20260909,
complete 12 s. Their minimum sampled rectangle SAT margins are
1.316378, 1.463221 and 1.253437 m. Every sampled ego premise, ego enclosure and
known target component enclosure passes; every issued plan is certified and none
uses fallback. Over the last second, the largest speed, lateral-position and
body-heading errors across these seeds are 0.005252 m/s, 0.006406 m and
0.000998 rad. The 12 s horizon is an observation window, not a required recovery
deadline. These declared-bicycle runs complement the physical gate.

The focused MATLAB regression comprises 288 passing cases, zero failures and
zero incomplete cases across 25 test files. Coverage includes interval rotation
through the angle branch, sample-age behavior, unchanged observer points,
intersection tightness, preservation of compatible target coordinates, centered
lifted/condensed equivalence under station translation, generator propagation,
current-solve failure behavior, and complete straight recovery. After the exact
performance edits, all 34 affected helper/bound cases pass again; they replace
the earlier results in the 288-case distinct summary. Code Analyzer checks all
19 changed MATLAB files with no reported syntax errors and nine existing sparse-
index performance findings. The core source count remains 20; whitespace
checks pass.


## Timing and exact implementation optimizations

Two 600-frame recorded-input replays of the final physical pure-controller
sequence are measured per implementation, with explicit preparation and one
MATLAB computational thread. The first pass is retained separately. The two
exact changes skip unrequested road-load derivatives and avoid temporary cell
arrays when assembling a scalar native road frame. All steering, braking-ratio
and reported gross-acceleration differences from the recorded physical commands
are exactly zero in both original and optimized replays.

| Implementation / replay | Median | P95 | Maximum | Calls above 50 ms |
| --- | ---: | ---: | ---: | ---: |
| Original, first | 62.754 ms | 78.843 ms | 234.627 ms | 600 / 600 |
| Original, second | 62.272 ms | 75.109 ms | 92.159 ms | 600 / 600 |
| Optimized, first | 59.239 ms | 73.714 ms | 145.610 ms | 600 / 600 |
| Optimized, second | 58.911 ms | 71.759 ms | 87.831 ms | 600 / 600 |

The second-pass median improves approximately 5.4%. These are shared-desktop
recorded-controller-input replays, not new closed-loop runs, a full sensor-to-
actuator pipeline benchmark, or WCET. The measurements do not establish a
50 ms real-time pass. Two additional final joint recorded-input replays also
retain exactly zero command differences. Their median/P95/maximum times are
56.167/73.906/492.463 ms and 55.731/72.643/470.280 ms; all 600 calls in each
pass exceed 50 ms. Thus repeated nonlinear refinement remains the dominant peak
latency issue. A prior profiler trace identifies repeated nonlinear
rollouts, geometry construction and conic solves as the principal computation
costs; observer processing is outside the controller-call timing.


## Artifacts and interpretation

Original experiment artifacts are retained in
`/home/zai/.cache/collisionAvoidance/straight-controller-repair-20260907/`.
`straight-repair-trajectories.pdf` compares the original failure with both final
physical traces. `straight-uncertainty-diagnosis.pdf` shows the fixed-sample GNSS
velocity comparison and the same-frozen-prediction wrapping ablation. The latter
plots the full, unallocated future planning radius; it is not a plot of the
executed bound or an assertion that the final trajectory violates its heading
domain. The source snapshots, complete configurations, raw MAT files, numerical
failure programs, test exports and replay timings distinguish intermediate and
final versions. The external Project B archive preserves identified export
copies and the verified project commit record.

The repaired straight experiments support finite sampled avoidance and practical
post-encounter cruise under their stated assumptions. Nonzero physical-model
error, persistent sensor noise and soft CLF slack preclude claiming exact
zero-error asymptotic convergence from these runs. Timing, general nonlinear
recursive feasibility, general-road validation and global physical residual
bounds remain separate questions. No circular-scene result is inferred.
