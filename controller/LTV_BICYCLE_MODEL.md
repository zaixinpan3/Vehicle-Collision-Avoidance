# Scheduled forward-Euler Frenet LTV dynamic bicycle

The prediction, actuation and tire model supports the single hard-CBF/soft-CLF
algorithm described in `PCBF_CLF_ARCHITECTURE.md`. Its implementation is in
`ltvBicycleStageMatrices`, `ltvBicyclePrediction`, `laneProjection`,
`laneCurvatureAtStation`, `frictionCirclePolygonRows`, `axleFrictionParameters`
and the physical rows and cruise equilibrium of `formulateAvoidanceProblem`.
`collisionAvoidanceControllerConfig` validates and normalizes acceleration
limits. The input and axle friction polygons follow Ge et al. (2022).

## Path coordinates

The lane centerline is a polyline with per-segment curvature and the
cumulative station of every segment start (`lane.segmentStation`,
built by `readPlanningInputs`). `laneProjection(p, lane)` maps a
Cartesian point to `(s, d)`: the station of its projection and its
left-positive lateral offset, with the path heading there;
`laneCurvatureAtStation(s, lane)` is the inverse for the one quantity
the schedule needs. Every position the controller reasons about — the measured
state, the schedule, the target's predicted pose, the road boundaries
— is expressed this way once at the start of a sample, and nothing
the terminal support and fallback clearance checks additionally use the
corresponding Cartesian geometry. The one geometric module, `rectangleConfigurationDistance`,
is called in BOTH frames: in path coordinates by stage 1, with the
heading errors as the yaws, and in Cartesian coordinates for the
physical clearance readout.

## Model

State `x = [s; d; e_ψ; vx; vy; r]` (station, lateral offset, heading
error to the path tangent, body velocities, yaw rate), input
`u = [δ_f; a]` (front road-wheel steering angle, longitudinal
tire-acceleration demand). Dynamic bicycle with linear cornering at the
frozen schedule speed `vBar`, in the Frenet frame of curvature `κ`:

    ṡ   = (vx cos e_ψ − vy sin e_ψ)/(1 − κ d)
    ḋ   = vx sin e_ψ + vy cos e_ψ
    ė_ψ = r − κ ṡ
    v̇x  = a + vy r + b            (bilinear term frozen at vy = 0, r = κ vBar)
    v̇y  = (Fyf + Fyr)/m − vx r
    ṙ   = (lf Fyf − lr Fyr)/Iz
    Fyf = Cf (δ_f − (vy + lf r)/vBar),   Fyr = −Cr (vy − lr r)/vBar

`b` is the longitudinal model bias published by the estimator
(`egoState.longitudinalAccelerationBias`, the offset-free disturbance
term): drag, rolling resistance and wheel-slip lag the declared model
omits, entering the `vx` row of every stage. Zero when nothing is
published.

**Schedule.** `vBar = max(vx_measured, model.scheduleSpeedFloor)`, the
stations `ŝ_k = s_0 + k vBar T_s`, and the centerline curvature `κ_k`
at each of them. The schedule depends only on the measured state and
the route and is regenerated at every sample; no previous solution
enters the linearization.

**Stage map.** The model linearized about the schedule point
`x̂_k = [ŝ_k; 0; 0; vBar; 0; κ_k vBar]` — in path coordinates that point
IS the cruise along the centerline, so every affine term of the
kinematic rows vanishes:

    ṡ ≈ vx + κ vBar d,   ḋ ≈ vBar e_ψ + vy,   ė_ψ ≈ r − κ vx − κ² vBar d,

the velocity rows as before (`c_5 = κ vBar²` from the frozen `vx r`
term). Discretized by ONE FORWARD-EULER step, `A_d = I + T_s A`,
`B_d = T_s B`, `c_d = T_s c` (`ltvBicycleStageMatrices`; a user decision
of 2026-08-23 replacing the exact zero-order hold — do not reintroduce
`expm` without asking). The curvature is treated as locally constant
at each stage's schedule station; its variation along the horizon is
carried node by node. Euler is conditionally stable:
`T_s (Cf + Cr)/(m vBar) < 2` and `T_s (lf² Cf + lr² Cr)/(Iz vBar) < 2`
must hold, an obligation the sample time and `model.scheduleSpeedFloor`
carry together; the configuration validates it at the floor (at the
defaults the yaw rate is 264/vBar 1/s, so the floor must exceed 6.6 m/s
at 50 ms).

The Euler input map has zero position and heading rows, so the current
input cannot change the first predicted pose. Its collision and road rows
remain hard: an unsafe constant row rejects the program. The condensed
prediction `x_k = E_k plan + e_k` includes the head inputs and tail
accelerations. `formulateAvoidanceProblem` rolls out the head at each start
and evaluates the tail through this condensed map.

**Row-tightening radii.** The measured estimate radii
(`controllerStateErrorBound`) at node 0 — the two Cartesian position
radii summed into both `s` and `d`, the rest carried over; at node 1
the one-step reachable set `|A_0| E + T_s W`, `W = model.ltvModelErrorRateBound +
model.plantModelResidualRateBound` (declared per-second rate boxes,
zero by default = the declared model only); the measured radii held
constant beyond. No multi-step reachability is claimed.

## The kinematic braking tail

After node `N` the prediction continues for `N_b` stages as the
terminal set's tail (`PCBF_CLF_ARCHITECTURE.md`, Section 8;
`kinematicBrakingTail`), with the tail accelerations decision
variables:

    v_{k+1} = v_k + T_s a_k,
    s_{k+1} = s_k + T_s v_k + ½ T_s² a_k + T_s κ_k v̂_k d_N,     k = N .. N+N_b−1,

exact for piecewise-constant acceleration up to the coupling term —
the first-order Frenet factor with the speed frozen at the tail
SCHEDULE `v̂`, the time-optimal braking profile from `vBar`, the tail's
analogue of the head's frozen `vBar` (the head's `A(1,2) = κ vBar`).
The lateral states `d, e_ψ, v_y, r` are carried frozen at node `N`: the
tail's lane-keeping law holds the terminal lateral offset within the
band `terminalLateralCertificate` certifies in closed form, and the
rows charge that band, so no lateral dynamics are predicted there.
The dynamic bicycle is not used on the tail because its cornering rows
carry `1/v_x` and the Euler map is stable only above 6.6 m/s: a stop
cannot be predicted with it. The tail's admissibility rows are
`a_k ∈ [−a_b, 0]`, `v_k ≥ 0`, and rest at the last node. Consecutive
tail accelerations are independent. Its length `N_b` is the fastest
acceleration-bounded stop from the speed maximum plus two stages of
rest (74 at the defaults). The junction between
the two models is premise (K) of the safety document: the Euler head
map and the kinematic tail map agree over one stage to `½ T_s² a_b`
longitudinally and to the one-stage lateral motion the terminal rows
admit, which is bounded and measured, not modelled.

## The actuator and tire layer

**Input box.** `|δ_f| ≤ model.frontWheelSteeringAngleMaximum`,
`a ∈ [actuation.longitudinalAccelerationMinimum, Maximum]`
(validated by `collisionAvoidanceControllerConfig`).

**Input changes.** The model has no steering-rate or longitudinal-jerk
constraint. Every stage input is chosen independently inside the input
box and the friction polygons. `egoState.heldActuatorInput` is retained
only as execution feedback for stored-certificate compatibility; it does
not restrict the new plan.

**Friction polygons** (`frictionCirclePolygonRows`, Ge et al. Eqs.
22–33). Per stage and per axle an `N`-edge polygon inscribed in the
axle friction circle, `n_x Fx_i + n_y Fy_i ≤ cos(π/N) μ_i Fz_i(a)`, with
`Fx_i = ρ_i m a`: positive-facing facets use the front-drive shares
`[1; 0]`, negative-facing facets the declared braking shares
`actuation.brakingForceDistribution`, so both actuator regimes form one
convex set without a sign disjunction. `Fz_i(a)` carries the affine
longitudinal load transfer when `vehicle.centerOfGravityHeight` is
declared (`Fzf = m(g lr − a h)/L`, `Fzr = m(g lf + a h)/L`); an
acceleration box that lifts an axle is a configuration error. Four
slip-angle validity rows per stage keep `|α_f|, |α_r| ≤
model.slipAngleMaximum`. The builder reads the `vy`, `r` rows of the
state, which are the same in both coordinate forms, and writes its
rows as `A u + offset ≤ 0`.

## The model-domain rows

- **Speed domain.** `vx_k ∈ [plannedSpeedFloor + r, speedMaximum − r]`
  at nodes 1..N, with `plannedSpeedFloor = min(model.plannedSpeedMinimum,
  max(vx_measured − r, model.speedMinimum))` per sample. The
  frozen-speed linearization is trusted only near the schedule speed;
  `speedMinimum` is bounded away from zero because the cornering rows
  carry `1/vx`.
- **Heading domain.** `|e_ψ,k| ≤ model.headingDomainRadius − r` at
  nodes 2..N: the validity domain of the Frenet linearization, and the
  domain on which the footprint support derivatives are bounded.

## The cruise equilibrium

`cruiseEquilibrium(v_ref, κ, b)` returns the state and input that make
the declared model stationary in the path frame at the reference
speed, the local curvature and the published bias: steady cornering of
the linear-cornering bicycle distributes `m v r` over the axles by the
moment balance (`Fyf = lr/L·m v r`, `Fyr = lf/L·m v r`), the rear slip
fixes the sideslip, the front slip the steering angle (the understeer
form `δ = Lκ + m v² κ (lr/Cf − lf/Cr)/L`), and
`a_eq = −b − vy_eq r_eq`. In path coordinates the CLF error is the
state minus this equilibrium, `[d; e_ψ; vx − v_ref; vy − vy_eq; r − r_eq]`,
with no projection frame to freeze; the CLF error and the
input-deviation cost are both referenced to it, which is what makes
the closed loop stationary at zero error with no integrator.

## What the model does not capture

- Combined-slip saturation beyond the linear tire: represented by the
  polygon boundary, not by a tire-force law.
- Deep slides: the linear cornering and the frozen speed misstate
  them; the slip-angle, speed and heading domain rows bound the claim.
- Curvature variation within a stage and the `(1 − κd)` factor beyond
  first order: charged nowhere; the path-frame geometry of the
  rectangles is charged by the sagitta term of the separation rows.
- Load transfer other than the affine longitudinal term; roll, pitch,
  wheel spin — all present in the harness's 14-DOF plant
  (`arcAvoidanceScenario.m`), none in the model. Their effect is the
  realized per-step margin difference reported by that scenario.
- Lateral model bias: only the longitudinal channel has an offset-free
  term.
