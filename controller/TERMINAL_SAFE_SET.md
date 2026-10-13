# Terminal safe set and recursive feasibility

This note defines the terminal set used by `solvePredictiveControl`, proves
its forward invariance under the terminal law, and states the
recursive-feasibility property of the ideal mathematical optimization together with
its hypotheses. The setting is discrete time with the sample period `h`, the
same as the plan's; Section 4 adds a continuous-time check for the time after
the ego has stopped. Section 9 states what the set has to become when the
estimator and the controller are designed together.

## 1. Purpose

The safety task is a finite encounter: no collision with the target while it
is inside the perception range `R = encounterRangeMeters`. The terminal set
comes from the target's model, not from the target leaving: past the plan's
endpoint both vehicles are continued with a model, the target with its own
forecast and the ego with one braking law, and the set keeps the states whose
continuation stays collision-free while the target is in range.

Neither the ego's stop nor the target's exit has to happen within the plan.
The ego never has to brake: the braking law only certifies that a safe
continuation exists. The plan length only sets how much maneuvering the
optimizer can plan. There is one set and one terminal law; no lane, side,
mode or alternate controller is used, and no encounter duration is preset.

If the set cannot be reached, the problem has no solution and the controller
reports `collisionAvoidanceController:noOptimizationSolution`.

## 2. The terminal law and the joint one-step map

**Target.** The target follows its motion contract (Sharma et al., 2026,
Eq. (2); `predictiveSafetyGeometry.predictTarget`):

    p_T' = V_T [cos(psi_T + beta_T); sin(psi_T + beta_T)],
    psi_T' = V_T sin(beta_T) / l_r,T,   V_T' = A_T,   A_T' = beta_T' = 0,

with braking that ends at rest: the speed is `V_T + A_T t` until it reaches
zero and zero from then on (the state's `A_T` becomes 0 there), and a target
at rest moves off only forward (`A_T > 0`). For a forward target the speed is
`max(0, V_T + A_T t)` (`predictiveSafetyGeometry.targetArc`). Its orbit is a
line for `beta_T = 0` and a circle otherwise, and it never turns back along it.
A stop cannot be run backward, so the controller forecasts forward from the
target's latest estimate and the sample index at which it was taken
(`targetEpoch`, `targetEpochIndex`). The ego's terminal law below ends at rest
in the same way.

**Ego.** The terminal set holds only mild states (Section 3), and inside it
the ego is the simplest vehicle under the simplest law: a kinematic bicycle
with its steering held, braking at a constant mild rate to rest. In a mild
state the declared model is taken to follow this kinematic model; this is
the user's modeling assumption (October 11, 2026), and no bound on the
mismatch is computed. With the steering held, the kinematic bicycle keeps
its sideslip and its course curvature, so the terminal law keeps the ego in
the same constant-(A, beta) family as the target:

    kappa_f(z) = ( A_E = max(-a_b, -V_E/h),  steering held ),

`a_b = terminal.brakingDecelerationMetersPerSecondSquared` (4 m/s^2). The
acceleration is held over each hold, so the speed falls by `a_b h` per hold
and by what is left in the last one: the ego stops exactly at the end of
hold `J = ceil(V_E/(a_b h))`.

The plan's state is the declared model's `x = [p; psi; v_x; v_y; r]` (the
Fiala bicycle above 4 m/s, a kinematic bicycle below 2.5 m/s, which can slow to
rest). Its
reduction to the family is

    V_E = hypot(v_x, v_y),   beta_E = atan2(v_y, v_x),   kappa_E = r / V_E,

and the continuation holds the sideslip `beta_E` and the course curvature
`kappa_E`: the body heading turns with the course at `kappa_E V_E`. Sharma's
kinematic relation `kappa = sin(beta)/l_r` is not used for the ego: it
presumes a rear axle without slip, which the Fiala state does not satisfy: in
the trim of a 4 m/s^2 left turn at 15 m/s the sideslip at the centre of mass
is -0.0093 rad, so that relation would continue the turn as a right turn
(curvature -0.0056 1/m against the actual 0.0178 1/m), about 6 m off the arc
at the end of the 22.5-m stop. Holding the measured yaw rate keeps the arc
the ego is on; in Sharma's parameters it is the effective
`l_r = V_E sin(beta_E) / r`.

**One-step map.** Both continuations are arcs of constant curvature with a
piecewise linear speed, so `phi`, the joint one-step map under `kappa_f`, is
exact and in closed form (`terminalSafeSet.continuation`, `successor`): with
the arc length `s(t)`,

    p_E(t) = p_E + s sinc(kappa_E s / 2) [cos(chi + kappa_E s/2); sin(chi + kappa_E s/2)],
    psi_E(t) = psi_E + kappa_E s,   chi = psi_E + beta_E,

and the target's flow is `predictTarget`. A unit test compares the closed-form
poses with a fine RK4 integration of the same kinematics (agreement to
1e-9 m).

**Mildness.** A state `x` is mild when its sideslip lies in the cone of the
problem, `|v_y| <= tan(beta_max) v_x`, and the held arc's lateral
acceleration is small, `|r V_E| <= a_m`
(`terminal.lateralAccelerationMaximumMetersPerSecondSquared`, 2 m/s^2). The
tires are then far from saturation: braking at 4 m/s^2 with 2 m/s^2 of
lateral acceleration uses 54 percent of the friction (`mu g` = 8.3 m/s^2),
and the configuration requires `hypot(a_b, a_m) < mu g`. Along the
continuation `beta_E` is constant and `r V_E = kappa_E V_E^2` only falls, so a
mild state stays mild. An endpoint in the middle of a swerve is not mild: the
plan has to settle before it is terminal.

## 3. The terminal set

For the plan's endpoint `z` at plan node `N`, let `phi^j(z)`, `j >= 0`, be the
continuation, sampled at its nodes and hold midpoints (the instants at which
the plan's own collision rows are sampled), and `t_e(z)` the first of those
instants at which the target's reference point is farther than `R` from the
ego's (Inf if none). The terminal set is

    X_f = { z : the ego is mild at z,
                every node, midpoint and the parked pose of the continuation
                keeps the ego rectangle on the road (lateralClearance),
                every node and midpoint before t_e(z) clears the target's
                forecast rectangle by the margin,
                and, if t_e(z) is not before the stop, the target's remaining
                track until it leaves the range about the parked reference
                point clears the parked rectangle by the margin }.         (3.1)

The margin is `safetyMarginMeters` (0.2 m, the problem's sampled collision
clearance), widened by the ego enclosure when the estimator publishes one
(Section 9). The road is checked on every pose regardless of the target. The
first exit ends the encounter: in the hybrid model with an absorbing end
state `+`, the terminal set is `X_f U {+}`, and no property of the target
after its exit is used. `j = 0` makes `X_f` a subset of the safe set: the
endpoint itself clears the target and the road.

The terminal sideslip bound is now `terminal.sideslipMaximumRadians` (default
0.05 rad), independently of the estimator operating cone. The rear handling
row is checked under the terminal braking ratio, not merely under the final
optimized hold's ratio. Terminal membership also checks that ratio against
the actuator bounds. These strengthen the settled-state definition; they do
not prove equality of the Fiala model and the closed-form continuation or
admissibility under an arbitrary finite slew limit.

## 4. The parked check in closed form

After the stop the ego is a fixed rectangle `E` and the target moves along its
line or circle. Let `sigma(tau) = V tau + A tau^2 / 2` be its arc length after
the stop until it comes to rest, and constant from then on
(`terminalSafeSet.parkedClearance`).

**The swept stretch.** The arc lengths at which the reference point is within
`R` of the parked reference point form one interval `[a, b]` around 0 (a
quadratic inequality on a line, an arc of angles on a circle, the whole circle
when the circle lies inside the range). From 0 the target moves toward `b`
(or `a`). When `V` and `A` have the same sign it sweeps `[0, b]` and leaves.
When they have opposite signs it brakes to rest at `sigma* = -V^2/(2A)`: if
it leaves before then it sweeps `[0, b]`; otherwise it sweeps `[0, sigma*]`,
stays there and never leaves. On a circle a sweep of at least
`2 pi / |kappa|` is the whole circle. A target at rest without forward
acceleration sweeps the single point 0 and never leaves.

**Rectangle against track.** The distance between `E` and the target
rectangle is continuous along the track, and while the two are apart it is
the least distance from a vertex of one to an edge of the other. Since the
two are apart at `sigma = 0` (the stop is a node of the continuation), the
least distance over the stretch equals the least vertex-to-edge distance over
the stretch; if they met anywhere, a configuration closer than the margin but
apart would come first. On a line the target translates: each target vertex
sweeps a segment against the four parked edges, and each parked vertex sweeps
the opposite segment in the target's frame (segment-to-segment distances). On
a circle the target rotates about the circle's centre `c`: each target vertex
sweeps an arc about `c`, and each parked vertex sweeps the opposite arc in the
target's frame. The arc-to-segment distance has the candidates

- the arc's ends against the segment;
- the segment's ends against the arc: `| |p - c| - rho |` when the arc passes
  the point's foot angle, otherwise covered by the arc's ends (the
  point-to-arc distance);
- the segment's line against the circle: the circle point nearest the line,
  at distance `h - rho` when the line misses the circle and its foot lies on
  the segment and the arc, or an intersection inside both (distance 0).

These 32 pairs give the exact least distance and the target pose attaining
it (the witness, used to linearize). In 2895 random cases (1216 braking, 646
sweeping their whole circle, 1826 never leaving the range) the closed form was
never above the least distance of a dense sampling in time of the forecast
itself (spacing at most 2.4 mm; above it by at most 5.2e-13 m, rounding) and
within 2.8e-7 m of it, and the witness reproduced it to 5.2e-13 m
(`report/TARGET_STOP_AT_REST_20261011`, `checkParkedClearance.m`); the unit
test repeats the comparison on five targets.

## 5. Forward invariance

**Proposition.** `X_f U {+}` is forward invariant under `kappa_f` with the
target on its forecast, at the sample instants, and no continuation from
`X_f` meets the target at a sampled instant before the encounter ends, nor at
any time after the ego has stopped.

*Proof.* Let `z` be in `X_f` with continuation `phi^j(z)`. The continuation of
`phi(z)` is `phi^{j+1}(z)`, `j >= 0`: the speed schedule from `V_E - a_b h` is
the remainder of the schedule from `V_E`, the arc continues on the same circle
with the same sideslip and curvature, and the target's forecast is evaluated
at the same absolute times. Mildness holds at `phi(z)` (Section 2).
Every sampled pose of the new continuation is a pose of the old one, so the
road condition and the clearances before the exit hold; the exit is at the
same absolute instant (or has happened, and the state is `+`); the parked pose
and the target's remaining track are the same. Hence `phi(z)` is in `X_f`, or
the encounter has ended. QED

The successor of the last hold is the parked state (zero speed), whose
continuation is the parked check alone. The unit test follows a member to rest
and checks membership, the remaining stop and exit times and the continuation
poses at every hold.

**Largest within the settled admissible domain.** A state in that domain
but outside `X_f` has a continuation under
`kappa_f` that leaves the road or meets the target before the exit, so no set
that `kappa_f` keeps safe within the domain contains it: `X_f` is the largest such set, and its
shape adds no conservatism. The only design choice is the terminal maneuver.
The construction is the backup-set principle of Chen et al. (2021), used only
to define the terminal set; the plan's free holds still enlarge the feasible
set.

## 6. Recursive feasibility

The problem at sample `k` (plan length `N_k`, prefix `horizonSteps`) is:

- the plan is the sampled model's rollout of the inputs from the current
  estimate;
- road, state-limit and handling rows hold at every node and hold midpoint;
- collision clearance holds at every node and hold midpoint after the prefix
  (hard), and is a minimized slack inside the prefix (soft);
- the endpoint lies in `X_f`.

A plan meeting these rows is *accepted*.

**Hypotheses.**

- H1: the plant is the declared sampled model, and the state is known
  exactly.
- H2: the target follows its forecast (the same constant `A` and `beta`).
- H3: at an endpoint in `X_f`, some hold of the declared model keeps the
  endpoint in `X_f` with its hard rows met and its midpoint clear. A terminal witness must also satisfy actuator, rear-adhesion and slew
  bounds. Equality of the mild model to the kinematic continuation alone
  does not establish those input conditions. The implementation does
  not rely on the assumption: the anchor tries the cruise hold of the CLF's
  certified controller and the holds toward the law's realization on the
  declared model, and keeps a hold only when its endpoint is checked to be in
  `X_f` (Section 7).

**Proposition.** Under H1 and H2, if the plan of sample `k` is accepted and
has at least two holds, the shifted plan at `k+1` is accepted:

- drop the first hold;
- while it is shorter than the prefix, append a hold of the declared model
  that keeps the endpoint in `X_f` (H3).

Without H3 at some sample it is accepted as well, one hold shorter. The sum of
the prefix collision deficits does not increase.

*Proof.* By H1 the new state is the plan's second node, so the shifted inputs
reproduce the old nodes 2..N+1 exactly. Every hard row of the old plan holds
there. The old node `prefix+1` was hard (deficit 0) and is now the last prefix
node, so the prefix deficit sum loses the old first node's deficit and gains
zero. The endpoint is the old endpoint at the same absolute time, and by H2
the forecast is unchanged, so its membership (3.1) is the same evaluation as
before: it is in `X_f`. An appended hold meets the hard rows, its midpoint
clears the target, and its endpoint is in `X_f`, whose condition at `j = 0`
includes the node's own clearance; its deficits are zero. QED

This is an existence proof for the ideal nonlinear problem. It does not
assert that a numerical solver returns a command. The runtime issues the
completed CLF-program input directly; no post-solve step-size rule substitutes
the shifted candidate. Numerical failure in an experiment is separate from
mathematical feasibility. For the bounded-error proof, including the terminal
input and posterior-inclusion premises, see
[ROBUST_RECURSIVE_FEASIBILITY.md](ROBUST_RECURSIVE_FEASIBILITY.md).

The start of an encounter is outside this statement. When the target first
enters the range or re-enters it after leaving, the new encounter is an
initial-feasibility question.

## 7. Implementation

**Membership** (`terminalSafeSet.member`) evaluates (3.1) in order:
mildness, the road along the continuation (every corner of every pose,
one vectorized projection), the target at the continuation's nodes and
midpoints until the first sampled exit, and the parked check. Where the
reference distance less both reaches already exceeds the requirement by
5 m, that lower bound stands for the rectangle distance; elsewhere the
rectangle distance is exact. The continuation from 15 m/s has 61 nodes and
60 midpoints; a call costs 0.3 to 20 ms. `info` records the end
(`exit`, `parked` or `noTarget`), the time to the stop and to the target's
exit, the least clearance beyond the requirement, and the CLF value.

**Terminal rows** (`terminalSafeSet.rows`). In the convex problem the endpoint
correction `dz` must meet the set's conditions linearized at the anchor
endpoint `y`:

- mildness, `|r V_E| <= a_m`, linearized in the endpoint's speed and yaw
  rate, where `y` is within 1 m/s^2 of the bound, keeping up to 0.1 m/s^2 of
  the margin `y` has (without the reserve an endpoint left on the boundary
  takes only tiny steps, because every full step leaves the set, and the plan
  executes its old tail); the sideslip cone is already a row of the problem at
  every node;
- fixed-dual collision rows (the ordinary-distance duals of the rest of the
  problem) at every continuation node and midpoint before the exit, and at the
  parked pose against the witness, the target pose nearest it on its track
  (the minimizer's own rows: the derivative of a minimum is that of its
  minimizer);
- corner rows on the road at the continuation's poses;

each composed with the derivative of the continuation's pose with respect to
`y`, by central differences of the closed form with the schedule held.
Conditions whose clearance exceeds the requirement by more than 5 m are left
out, and so are those whose margin exceeds the most their pose can move over
the endpoint correction's box (|dx| + |dy| + reach |dpsi| through the
continuation's sensitivity; they cannot bind); the exact set decides
anchor membership. Each row keeps up to 5 cm of the margin
it has at `y` (`b - min(0.05, max(b, 0))` for a row bound `b`): without it the
optimizer leaves the endpoint on the boundary of `X_f`, where the next
sample's shift falls out of the set under millimetres of plant mismatch or of
departure of an appended hold from the law. A row is never tighter than `y`
itself, so when `y` is in `X_f` the zero correction meets every row, and the
zero-slack shortcut of the PCBF stage remains consistent with the anchor. Where the closest features of a pose give no
consistent dual (an exact contact, typically the parked witness of a track
that crosses the parked rectangle), the witness is moved 1 mm or 1 cm along
the track, or the row is left out.

**Anchor.** Later samples start from the previous accepted plan, shifted by
one hold. While it is shorter than `horizonSteps`, a hold is appended: the
cruise hold `u = u* + K e` of the CLF's certified controller
(`terminalSafeSet.cruiseInput`, NOMINAL_CLF.md) when it keeps the endpoint in
`X_f` with its rows met and its midpoint clear. Otherwise the plan is first
extended through states outside `X_f` by holds of the startup guidance
(potential guidance with a target, path guidance without one), each meeting
the hard rows and clearing the target at its midpoint and endpoint, until a
node lies in `X_f` again, at most up to `maximumHorizonSteps`: a longer plan
that completes the maneuver. Without it an aggressive maneuver longer than
the plan (a 20-m lane recovery at 15 m/s) could only keep a mild endpoint by
braking in the plan's tail, and the vehicle braked to what was then the
model's 1-m/s speed floor.
If guidance does not reach `X_f`, the appended hold is the hold nearest to the
cruise hold on a segment toward holding the previous steering, which keeps a mild
endpoint mild: at the cruise hold's braking ratio, or, when that does not keep
the endpoint in `X_f`, braking with the ratio whose actuator deceleration is
`a_b` (the law's realization), or toward braking as hard as the rear adhesion
admits (`terminalSafeSet.brakingInput`: each far end is checked in turn, the
first that keeps the endpoint in `X_f` is bisected toward the cruise hold with
five halvings). Holding the speed first matters where only mildness stops the
cruise hold: the plan's tail is not regulated (the CLF stage acts on the first
successor), and a drifting tail whose cruise hold is no longer mild would
otherwise be extended by braking, which drifts it further (observed in the
15-m/s reversed-start recovery test). When no far end keeps the endpoint in
`X_f`, the plan stays one hold shorter (`extensionStopped`). An endpoint no longer in the set (a changed
forecast) is extended by cruise holds until one is. The braking holds are
kept as small as found because the plan's later holds reach the first hold
nearly unchanged: the CLF stage acts on the first successor and the
correction of each hold is bounded by the trust region. At startup the potential-field
rollout runs until its first node in `X_f` from `horizonSteps` on, at most
`maximumHorizonSteps`. The rollout supplies a linearization, never an issued
input.

**Direct execution.** The CLF program's full input sequence is issued without
post-solve scaling or a constraint veto. Its nonlinear prediction is diagnostic
data. The exact terminal evaluator is used to build and extend the anchor; it
is not a post-solve acceptance gate. Accordingly a predicted nonlinear endpoint
need not satisfy the linearized terminal rows exactly. The ideal terminal-set
proof is not a theorem about every local convex solution.

**Records.** The issued plan records `terminalEnd`, `terminalStopSeconds`,
`terminalExitSeconds` and `terminalMarginMeters` (NaN when not evaluated),
`terminalValue` (the predicted endpoint's CLF value), `appendedTerminalSteps`, `appendedBrakingSteps` and
`extensionStopped`.

## 8. Measured behavior

`report/BRAKING_TERMINAL_SET_20261011.tex` compares this set with the CLF-tube
set it replaces (commit `423e3e8`, unchanged at `710a228`) on the same 98
encounters: exact states 9 of 14 against 14 of 14, noisy 60 of 84 against 53
of 84, no collision in either (least clearance 0.089 m exact, 0.422 m noisy).
It recovers 8 of the 12 noisy braking leads, which every earlier variant lost
at the first frame, and plans 16 holds at the first frame instead of 109
(95th-percentile controller time 0.023 s against 0.174 s, exact). Its losses:
four exact encounters (both braking leads, the 8-m/s crossing and turning
crossing) yield to 1.3-3.6 m/s, where the declared model has no acceptable
step, and the exact 15-m/s turning crossing ends with the parked ego met by
the target's track; in three noisy 15-m/s curved crossings no declared-model
hold keeps the endpoint in the set until the plan runs out (H3 fails); five
noisy swerves brake into the rear-adhesion envelope; fourteen noisy failures
are numerical, as are eighteen of the baseline's.

## 9. Joint estimator–controller design

In the closed loop with the estimator, H1 and H2 fail: the state is an
estimate `xhat` with a certified enclosure, the target is an information set
of constant-parameter states, and the plant differs from the model.

**Ego.** The implementation widens every clearance and road margin of (3.1)
by the current enclosure's position radius and the chord `2 L sin(e_psi/2)`
of its yaw radius (`L` the body reach), as a static offset. The continuation
is not propagated from the enclosure: an error `(e_p, e_chi, e_kappa, e_V)` in
position, course, curvature and speed moves a continuation pose at arc length
`s` by up to `e_p + s e_chi + s^2 e_kappa / 2` and the stop by
`V e_V / a_b`, the arc being at most `V^2/(2 a_b)` long (22.5 m from 15 m/s).
A robust set would carry that propagation.

**Plan.** At `k+1` the new estimate departs from the plan's second node by the
innovation `delta`; the shifted inputs reproduce the old plan only up to
`Phi_j delta`, `Phi_j` the transition of the linearized model over `j` holds,
which grows with `j h v` (no restoring force on lateral position). Robust
recursive feasibility needs tightened rows along the horizon and a terminal
set tightened by `sup |Phi_N delta|` (Chisci, Rossiter and Zappa, 2001), or a
plan in feedback form after the prefix (Mayne, Seron and Rakovic, 2005).

**Target.** A terminal set built against the forecast of the information set
`Q_k(t)` (every member cleared until all of it has left the range) is
monotone in `Q_k`, so robust recursive feasibility needs nested information
sets, `Q_(k+1)(t) subset Q_k(t)` for `t >= t_(k+1)`, which a set-membership
update satisfies and the present high-gain observer's ISS enclosures do not.
Holding each unknown constant fixed on each trajectory keeps the target's
reach set to the orbits of the contract; letting `A` and `beta` change at
every instant would enlarge it.

**Common robust continuation.** The bounded-error extension uses joint sets
of vehicle/target states, persistent vehicle parameters and observer/input
memory. One causal terminal rule must certify the entire endpoint set. The
nominal formula max(-a_b,-V_E/h) must not be interpreted as different
commands selected from unknown true speeds in different branches. A common
held steering/braking command together with the declared forward-only stop
rule is one possible witness. With a uniformly certified realized braking
rate a_min>0, every branch stops by V_bar/a_min and travels at most
V_bar^2/(2*a_min) additional arc length. Pose, speed and parameter uncertainty
are propagated through that continuation; static widening of its center is
insufficient. The full parked rectangle family is checked against the full
target family, and residual disturbances capable of moving it require a
parked invariant-set certificate.

This construction retains the accepted mild kinematic terminal model.
Its image/row certificates are carried when the policy is shifted, and
invalid low-speed observer enclosures cannot replace the retained physical
set. The mathematical proof and its exact scalar validation are in
[ROBUST_RECURSIVE_FEASIBILITY.md](ROBUST_RECURSIVE_FEASIBILITY.md) and
[CLOSED_LOOP_PREDICTION_THEORY.md](CLOSED_LOOP_PREDICTION_THEORY.md).
The executable terminal membership remains the center-based construction
described above; this joint certificate is not yet implemented there.

## 10. Limitations

- Recursive feasibility is proven within one encounter, on the declared
  sampled model with exact information (H1, H2) and with the extension that
  keeps the plan from running out (H3). H3 follows from the user's
  assumption that in a mild state the declared model follows the kinematic
  model; no bound on that mismatch is computed, and the implementation checks
  every appended hold on the declared model instead.
- The braking phase is sampled at nodes and hold midpoints, like the plan's
  own collision rows; between them no clearance bound is claimed. The parked
  phase is checked in continuous time.
- Braking ends at rest for both vehicles (target contract since October 11,
  2026, in the forecast and the simulated truth alike). A target that comes to
  rest inside the range stays in it for ever, so its rest pose must clear the
  parked ego; a target at rest moves off only forward.
- The first exit ends the encounter, as the convention of the finite task.
  The plan's own collision rows also apply at a later re-entry; the set does
  not check one.
- The terminal set is certainty-equivalent in the target and widens the ego
  clearance statically (Section 9).
- Mildness is a bound on the held arc's lateral acceleration and the
  sideslip cone; within it the declared model is taken to be the kinematic
  model, without a computed mismatch bound (the user's assumption).

## History

The terminal set of October 6 to 10, 2026 was the CLF tube of the terminal
controller `u = u* + K e`: the boxes in path coordinates that every
trajectory of that controller stays in, missing the target until it left the
range or its relative motion left the tube's collision cone (straight road).
It required the plan to reach a state from which cruising on the given path
clears the target, which needed plans of about 110 holds at the first frame,
and its certified size was bounded by the controller's certificate
(`report/TERMINAL_SET_ALTERNATIVES_20261010.tex`). The certificate of `K`
remains as the CLF's (NOMINAL_CLF.md) and its hold extends the anchor. The
joint-state construction that motivated the present set is in
`report/JOINT_TERMINAL_SET_DESIGN_20261010.tex`.

Until the user's decision of October 11, 2026, the target contract had signed
speed: a target whose speed reached zero turned back along its track and
accelerated in reverse, in the forecast and in the simulated truth, and a
parked ego behind a braking lead was terminal only when the lead's turning
point lay outside the range (`report/BRAKING_TERMINAL_SET_20261011.tex`). The
forecast was then anchored at sample 0 by running the estimate backward; a
stop is not reversible, so it now runs forward from the latest estimate
(`report/TARGET_STOP_AT_REST_20261011.tex`).

The set of October 11 (commit `f94a0e2`) admitted every endpoint whose
braking at 5 m/s^2 with the held arc's lateral acceleration stayed inside the
friction circle (up to 6.7 m/s^2 of lateral acceleration), and its
continuation was a certificate on the Fiala state rather than a model the ego
was taken to follow. At the user's direction the set now holds only mild
states, in which the ego is taken to be a kinematic vehicle under the
simplest law (steering held, 4 m/s^2 to rest), without a mismatch bound
(`report/MILD_TERMINAL_SET_20261011.tex`).

## References

- G. Sharma, H. Alai and R. Rajamani, "Simultaneous ego-vehicle state
  estimation and vehicle trajectory tracking using a multistage high gain
  observer," *Transportation Research Part C* 182, 105411, 2026, Eq. (2).
- J. Huang, H. Wang, K. Margellos and P. Goulart, "Predictive Control Barrier
  Functions: Bridging model predictive control and control barrier
  functions," arXiv:2502.08400, 2025: problem (8), the shifted solution (10).
- Y. Chen, M. Jankovic, M. Santillo and A. D. Ames, "Backup Control Barrier
  Functions: Formulation and Comparative Study," arXiv:2104.11332, 2021.
