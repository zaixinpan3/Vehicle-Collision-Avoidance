# Finite-sensing estimator and controller integration

Implementation study: September 7, 2026.

This document describes the finite-sensing policy implemented by the current
controller. It separates the interval that will actually be executed from
the nominal trajectory used to anticipate an encounter. It does not claim
recursive feasibility, infinite-future separation, or exact convergence in
the presence of persistent measurement noise and nonzero CLF slack.

## Execution and prediction

The delay-aware straight validation uses a 100 ms control period and
`controller.inputDelaySteps = 1`. At time `t`, the already committed input
executes through `t + 0.1`; a fresh result is scheduled for the next
100 ms interval. `certifiedSteps = 2` covers both intervals. The first
input is eliminated exactly from the optimization and independently checked;
the returned command is the second stage. A previous scheduled command must
match its verified plan and the next measured execution contract. The
measurement, computation completion and scheduled actuation times are
recorded separately. A failed solve or deadline miss terminates the experiment;
it does not authorize continuing the scheduled sequence after failure.
See [the execution study](../scripts/STRAIGHT_DELAYED_EXECUTION_RESULTS.md)
for the corrected scope and actual results. The default zero-delay setting
remains available for idealized model studies and is not a deployment timing
claim.

At each sample, the controller uses the current observer estimate and its
error enclosure. It intersects this enclosure with the previous executed
prediction and centers the planning box at the midpoint of that intersection.
This preserves both enclosures instead of expanding their intersection
again around a displaced observer point. It does not reset the observer.
An empty intersection remains an error. The previous input and timestamp
are checked as well.

With `controller.certifiedSteps = 1`, collision and road constraints cover
the complete next held interval using Taylor/Bernstein tubes, current state
uncertainty and the declared model residual. The continuous CLF majorant is
also imposed on that interval. Later stages use nonlinear nominal rollouts and swept endpoint chords. The metadata reports `certifiedDuration`, `lookaheadDuration`,
`safetyScope`, and the distinction between robust swept execution and nominal
future chords.
Setting `certifiedSteps = Inf` retains uncertain swept constraints and CLF
majorants throughout the finite horizon; it can legitimately be infeasible
when a first position-only detection leaves target motion unobservable.

An extra lookahead reserve anticipates the next measurement enclosure. A
bounded scalar fraction allocates only the achievable part of that extra
reserve. `anticipationReserveFractionMaximum` sets its cap, currently 0.25.
This is a calibrated planning parameter, not a reduction of the executed
uncertainty enclosure or a proof about future measurement errors. A reduced
allocation uses 99% of its LP optimum, less the arithmetic guard, to leave
space for the subsequent nonlinear refinement. Thus the reduced-buffer
allocation is deliberately interior, not an exact lexicographic optimum.
Physical collision and road rows stay hard at fraction zero.
The allocation phase supplies a constraint target, never an
execution input. The joint performance solve and independent acceptance
are still required. An unattainable optional reserve is therefore not
misreported as a violation of physical clearance.

Future state-domain, tire-slip and clearance margins anticipate consecutive disturbances. A
single-step allowance at every future node omits accumulated heading error;
with a steering-rate limit, a nominal path can then approach the heading
boundary before the vehicle can turn back. The planning allowance follows

\[
 G_{j+1}=[A_{d,j}G_j,\ \operatorname{diag}(q_j)],
 \qquad \rho_{j+1}=\sum_\ell |G_{j+1}(:,\ell)|,\qquad
 q_j=\int_0^h\exp(M_j\tau)\,\bar w\,d\tau,
 \qquad (M_j)_{ii}=(A_j)_{ii},\quad
 (M_j)_{ik}=|(A_j)_{ik}|\quad(i\ne k).
\]

The executed stage initializes the generators with the diagonal of its
Taylor/Bernstein endpoint radius. Signed generators preserve correlations
across discrete maps. Replacing them with an independent box at every
future node discards cancellation in the coupled lateral dynamics and can
inflate a stable system's error allowance. Retaining the diagonal of the continuous generator
preserves damping in the comparison system. Replacing it with `abs(A)`
unnecessarily turns negative diagonal entries into exponential growth.
Future discrete Jacobians belong to the nominal nonlinear rollout; this
recursion supplies a planning reserve, not a nonlinear tube certificate.
The current estimation box is propagated separately, with
`Ginitial(j+1) = Ad(j)*Ginitial(j)`. Its future state-domain and tire-slip
supports remain mandatory: allocating optional process reserves must not
scale down uncertainty already known to exist. Only the remaining future
process support enters the optional fraction. Target
clearance also accounts for the current target enclosure propagated one
sample and its displacement from the retained nominal hypothesis.
The station limits defining a local coordinate
chart are excluded; their swept-frame construction is separate. A reserve
that cannot fit never relaxes the physical domain or the robust executed
interval. Nonlinear acceptance checks those actual physical limits.

The objective includes state error, input deviation from the fixed operating
input used for CLF/LQR certificate synthesis at the current road curvature,
input changes, and squared CLF
slack. Both solver transcriptions read `qp.clf.certificate.operatingInput`;
the stage-dependent prediction seed is not a cost center. See the
[objective definition](QUADRATIC_CLF_INPUT_OBJECTIVE.md). There is no
desired-acceleration objective. Every issued input comes from a new
optimization result that passes independent condensed-form checks. No
stored input or backup controller is executed after a failed solve.

A previous maneuver is tried first under `retainFeasibleManeuver`. Its
shifted inputs only initialize a new optimization; the nominal cruise input
fills any newly exposed prediction stages. `bestFeasibleObjective` instead
evaluates all maneuver candidates. Neither policy promises a global optimum
of the nonconvex obstacle problem.

## Finite encounters

The estimator publishes a `finite-sensing-motion-v1` descriptor with bounded
Cartesian jerk and yaw acceleration. It requires no nonreturning halfspace
beyond the entire road. Each current observation renews the local prediction
window after a motion-set consistency check.

An absent target is released only with a complete, timestamped perception
declaration. If the target's entire reachable position set is still inside
that sensor range, absence contradicts the declaration and is rejected.
Without complete perception, missing observations do not silently release
the target. A later detection is admitted and checked again.

The longer nominal prediction follows constant curvature and tangential
acceleration. Target motion sets are intersected without discarding their
tighter midpoint representation. A propagated previous nominal state is
projected componentwise into the new enclosure; compatible coordinates
remain unchanged. A small excluded yaw-rate coordinate therefore cannot
reset all motion parameters and reverse the complete forecast.
At first admission, unresolved acceleration and yaw-rate coordinates use
the nearest admissible values to zero. A nonzero value is retained when the
enclosure excludes zero. This is a nominal hypothesis, not a measurement
or an assumption that the target must drive straight or occupy a lane.
This uses the
constant-motion assumption to avoid changing a still-compatible forecast
in response to every noisy acceleration estimate. It does not assert that
the retained nominal trajectory is the unknown true trajectory: the
executed-interval safety calculation still uses the full uncertainty set.
The NRMM observer state and its gains are not changed by this selection.

## Estimation bounds from measurement history

The ego velocity enclosure also uses the timestamped GNSS velocity ball and
the certified orientation set. For measured inertial velocity `g`, each
body component of `R(-psi)*g` is a sinusoid in `psi`. Its exact extrema over
the union of yaw intervals occur at interval endpoints or included
stationary angles. Expand these extrema by the GNSS noise radius, then
intersect the resulting component bounds about the unchanged body-velocity
observer point with the existing norm-based enclosure. Near straight travel,
heading uncertainty mainly affects lateral velocity; copying a single norm
radius into both components needlessly inflates longitudinal uncertainty.
For an aged sample, add `accelerationMaximum*age` to its inertial velocity
ball. If no finite acceleration envelope exists, an aged GNSS sample does
not tighten this channel. Empty orientation sets remain unavailable.

`nrmmTargetHistory` adds bounded-noise derivative enclosures to the existing
observer error calculation. With position measurements separated by
`T > 0`, acceleration bound `aMax`, and measurement radii `r0, r1`, a
secant bounds the current velocity with component radius

\[
 (r_0+r_1)/T + a_{\max}T/2.
\]

A measurement age adds `aMax * age`. Three distinct times also give
quadratic-interpolation velocity and acceleration estimates. If `w_i` are
their derivative weights and `jMax` bounds Cartesian jerk, the radius is

\[
 \sum_i |w_i|\left(r_i+j_{\max}|t_i-t|^3/6\right).
\]

These intervals intersect the declared physical domain and the existing
observer enclosure. A single position measurement contributes no invented
velocity information. The published radius always encloses the unchanged
observer point estimate; an inconsistent intersection is unavailable.

When an ego yaw-acceleration bound is supplied, past radar vectors are
transported to a common heading using gyro history. A common absolute
heading error is applied to the weighted **sum** of those vectors. It is
not charged independently to every absolute position before taking a
difference. Trapezoidal gyro integration uses the deterministic per-interval
allowance `gyroNoise * h + yawAccelerationMaximum * h^2 / 4`.

Position chords also enclose the target course direction. If a displacement
ball has center `c` and radius `r < norm(c)`, its direction radius is
`asin(r/norm(c))`. For constant curvature, the chord points along the
midpoint of the swept course angles, so the endpoint allowance is
`yawRateMaximum*T/2`; a nonconstant-curvature bound uses the full
`yawRateMaximum*T`. Measurement age and the declared target sideslip are
added. This updates an error enclosure about the NRMM heading estimate;
it does not replace the heading point estimate with a chord direction.

These bounds remain conditional on the ego model. In particular,
`abs(r-vy/lr) <= singleTrackYawRateMismatchMaximum` is a rear-tire kinematic
premise, not an identity of a dynamic bicycle or PassVeh14DOF model.
`auditNrmmTruthEnclosure` independently compares simulation truth with that
premise, the speed/yaw-rate/sideslip domains, and the published ego and known
target-state component bounds. An unavailable infinite radius never counts
as verified containment. Joint-suite acceptance requires these sampled
checks in addition to finite estimates and correctly formed sensor fields.
The audit is diagnostic only; truth is never supplied to the controller.
It does not prove intersample premises or unmeasured target-motion bounds.

## Geometry and vehicle model

Straight and circular scenarios publish an explicit analytic
`referenceCurve`. Finite circle coordinates, projection, frame variation,
and uncertainty are evaluated from that curve. Dense polyline vertices no
longer introduce artificial heading jumps. Arbitrary unsupported polyline
jumps remain rejected. The allowed lateral strip must stay within the
regular Frenet chart.

Perceived road coefficients are new measurements, so their exact array
identity is not a persistent execution contract. Each new road fit is used
in the current solve. The requested horizon is shortened before solving if
its complete geometry would exceed the fitted source interval. No boundary
is extrapolated to create apparent coverage.
Coverage admission and constraint construction share the same swept-cell
frame, including station trust and chord allowances. Coverage is checked
again after each maneuver seed or nonlinear refinement changes the anchor.

The tire-speed floor is now a low-speed regularization at 1 m/s. Previously,
a 12 m/s floor changed the tire dynamics even in the 10 m/s experiments.
Curved cruise uses compatible lateral velocity, body heading, yaw rate,
steering and longitudinal road-load force from the same frozen model.

The PassVeh14DOF adapter reads the active R2026a MF62 preset for lateral
stiffness and friction as well as rolling resistance. The inactive mask
`PKY1` value was approximately -15.57; the active preset used -30.33.
The active zero-slip lateral derivative was checked numerically using the
MathWorks tire evaluator. Reading the inactive parameter set was a model
input error, not an optimization tolerance issue.
An explicitly requested collision rectangle is preserved after refreshing
active plant parameters. The scenario's 5-by-2 m collision rectangle is an
abstraction distinct from the plant's track and tire geometry.

The default `trajectory` linearization evaluates the Fiala tire tangent at
the actual nominal slip, steering and braking ratio. Its body dynamics
include rotation of the front longitudinal and lateral forces, Coriolis
terms, road-load derivatives and the nonlinear Frenet Jacobian. A nonlinear
RK4 rollout supplies the nominal anchor. Each future affine offset uses
the same nonlinear start state at which its Jacobians were evaluated;
it does not cancel an upstream frozen-step discrepancy by resetting the
following endpoint. Batched central differences of the actual RK4 map
supply future discrete Jacobians; the executed generator retains its
continuous tangent and separate residual allowance.

Every accepted affine candidate is rolled out again through the nonlinear
bicycle. Future road, collision, state-domain and tire constraints are
checked on that rollout. A violating candidate causes a bounded number of
new trajectory linearizations. Future tire rows linearize the same
scheduled slip used to admit the next executed interval:
`(vy + axleDistance*r)/max(vxStart,speedFloor) - steering` for the front
axle, with rear axle distance `-lr` and no rear steering. The denominator
derivative acts on the interval-start state, including at the endpoint;
the numerator acts on that endpoint. Acceptance evaluates both this
scheduled expression and the physical `atan2` slip on the nonlinear
rollout. This prevents a future plan from passing a different slip test
than the one applied when its interval becomes current.
The Fiala law itself satisfies the physical combined-force circle; an
additional future inner polygon is unnecessary. Substituting a different
state into an old force tangent created a repeatable refinement cycle in
the 4.20 s physical-plant sample even when its nonlinear tire forces fit the
polygon. Exhausting the refinement budget still stops control. These
iterations have no general convergence guarantee.

Each future collision/road cell constrains its two endpoint footprints
and adds the acceleration-based chord-deviation allowances `aMax*h^2/8`
and `yawAccelerationMax*h^2/8`. Rectangle support uses a convex maximum of
vertex tangents. For a vertex projection `f(psi)=a*cos(psi)+b*sin(psi)`,
`f''=-f`, so it is concave on each interval where it can contribute
nonnegative support. Its tangent on that interval is an upper bound. The
construction is exact at the anchor orientation within the declared yaw
chart and avoids a triangle-inequality overestimate at diagonal normals.

On executed stages, an inner polygon bounds the **nominal scheduled** combined axle force.
It prevents the linear model from requesting simultaneous braking and
cornering beyond its nominal friction circle. Actual tire saturation,
transients and model discrepancy belong to the separately declared plant
residual. The nominal force polygon is not advertised as a robust bound on
every possible linearized force. Optional steering and braking-ratio rate
limits apply to all optimized input changes.

`measurePlantModelResidualRateBound` now compares logged plant derivatives
with the exact executed generator recorded in controller metadata and reports endpoint discrepancies
separately. Endpoint error divided by sample time is not a continuous-time
disturbance bound. Sampled finite differences and a finite calibration run
do not prove a global physical-model error bound.

## Numerical implementation and research basis

`avoidanceStageQp` introduces auxiliary cell states and sparse dynamic
equalities. Its constraints and objective differences are tested against
the independent condensed program. The auxiliary solver states are not
used to bypass acceptance of the physical control sequence.
These auxiliary states are deviations from the current anchor trajectory.
Centering the dynamics, geometry, CLF cones and objective together removes
large absolute route stations from equality right-hand sides without
changing the optimization problem. This avoids a needless mismatch between
the solver's relative residual scaling and strict physical-unit checks.
The external solver hook and its default solver both return the full lifted
decision vector; independent acceptance extracts and checks its physical
input and CLF-slack prefix.
Identical scheduled matrices and nominal-force rows are reused within a prediction.
The normalized Bernstein transform, compatible cruise references and
validated tire parameters are cached or reused. The performance solve is
tried at full optional reserve first; a successful solve attains the reserve
cap and avoids a redundant allocation LP. Independent acceptance remains
mandatory.

The optional reserve cap is intentionally a performance/safety-buffer
tradeoff within the unchanged hard execution constraints. In the straight
diagnostics, maximizing the complete open-loop planning allowance drove
the nominal trajectory toward heading and tire limits. The smaller cap and
interior allocation removed that behavior in the tested noise seeds.
Neither those experiments nor the linear generator algebra establishes
recursive feasibility for this nonlinear output-feedback controller.

Exploratory terminal state costs, proximal input penalties, capped hard
target buffers, fixed lateral separating normals and pulse-hold steering
seeds did not resolve joint failure and were removed. No extra terminal
cost or desired-acceleration target was retained from those experiments.

The separation between a current feasible solve and a recursive-feasibility
theorem is deliberate. Benders, Ferranti and Koehler explain that changing
environment constraints can invalidate a shifted candidate and that
recursive feasibility requires additional terminal and compatibility
assumptions. Their tutorial supplies this distinction, not a theorem for
this implementation: [A Step-by-step Guide on Nonlinear Model Predictive
Control for Safe Mobile Robot Navigation](https://arxiv.org/html/2507.17856v1),
Sections 3.2 and 5.3.
