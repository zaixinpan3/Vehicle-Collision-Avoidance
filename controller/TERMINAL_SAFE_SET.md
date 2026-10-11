# Terminal safe set and recursive feasibility

This note defines the terminal set used by `solvePredictiveControl`, proves
its forward invariance under the terminal controller, and states the
recursive-feasibility property of the resulting optimization together with
its hypotheses. The theory is in continuous time; Section 7 describes how the
implementation samples it. Section 9 derives what the set has to become when
the estimator and the controller are designed together.

## 1. Purpose

The safety task is a finite encounter: no collision with the target while it
is inside the perception range `R = encounterRangeMeters`. After the
prediction horizon the terminal controller completes the encounter: the
linear feedback `u = u* + K e(x)` on the transverse error, certified together
with the CLF on the nonlinear model (Section 2), so the vehicle dissipates to
the nominal behavior (cruise on the given path at the reference speed). The
terminal set is the set of states
from which that controller does not collide with the target, for the target's
forecast motion under the motion contract (constant tangential acceleration
`A` and constant sideslip `beta`), until the encounter ends (Section 4).

The encounter ends at the first of two events, both computed rather than
preset, and the end is an absorbing state: no property of the target after it
is used.

- The target leaves the perception range (it no longer exists for the
  controller).
- The target's forecast motion relative to the tube's box is outside their
  collision cone: from that time on the two can never come within the
  collision buffer again (Section 4.1).

Nothing about the encounter is preset. The check reads the forecast at the
state's own time first, in closed form, and follows the grid only when
needed, over a span the forecast's own geometry bounds (Section 7); a state
whose tube reaches the end of that span with neither event is not terminal.

No avoidance maneuver, lane, side or mode is designed in advance: the
terminal controller is the only backup. (Lane-hold backups at the other lane
centres were tried on October 8 and removed at the user's direction; see the
history in Section 4.) The set contains no safety distance of its own;
Section 7 lists the two sampling allowances of the implementation.

If the set cannot be reached, the problem has no solution and the controller
reports `collisionAvoidanceController:noOptimizationSolution`.

## 2. The terminal controller

Let `e(x) = [e_y; e_psi; vx - vx*; vy - vy*; r - r*]` be the transverse error
to the path trim and `V(x) = e(x)' P e(x)` the CLF of
[NOMINAL_CLF.md](NOMINAL_CLF.md). The terminal controller is the linear
feedback

    u = u* + K e(x),

whose gain `K` is synthesized together with `P`
(`scripts/synthesizeClfMatrices.m`). The pair is certified on the sampled
nonlinear model itself (RK4 holds of `h`), on the sublevel set
`Omega = {x : V(x) <= 1}`:

    V(x+) <= rho V(x),   rho = exp(-2h/T),   T = clf.convergenceTimeConstantSeconds,   (2.1)
    |K_j e| <= ubar_j   (the certification input box, 0.075 rad and 0.125),
    Omega lies inside the certification box of errors (clf.certification*).

(A force-level variant of the controller, commanding the front lateral force
through the inverse Fiala curve, certifies sets about a third larger at 8 m/s;
it was measured in `report/TERMINAL_SET_ALTERNATIVES_20261010.tex` and not
adopted at the user's direction, because it ties the controller's input to
the tire model's inverse.)

Hence along the terminal controller `V(x_k) <= rho^k V(x_0)`, and with the
hold factor `mu >= 1` below, `sqrt V(x(t)) <= mu exp(-t/T) sqrt V(x_0)` at
every time, which is (2.1) in continuous form up to `mu`.

**How (2.1) is established.** Write `g(e, du)` for the error after one hold
from `x(e)` with the input `u* + du`; the trim is a fixed point, `g(0, 0) = 0`,
so along the ray from `0` to `(e, du)`

    g(e, du) = Gbar(e, du) [e; du],   Gbar(e, du) = integral_0^1 G(t e, t du) dt,     (2.2)

exactly, with `G = [dg/de, dg/d(du)]` the Jacobian of the sampled map
(`terminalSafeSet.jacobians`: the RK4 variational equations composed with
the error chart). The certificate encloses the ray averages `Gbar(e, K e)`
themselves over `Omega` (`terminalSafeSet.rayJacobians`: Gauss-Legendre
quadrature with 8 nodes; the quadrature's residual in (2.2) is carried as a
rank-one term of the residual below). Enclosing the averages and not the
pointwise Jacobians is what makes the set large: for the saturating tire the
averages spread like the secants of the force curve and the pointwise
Jacobians like its tangents, and at the slips the set reaches the tangent
spread is two to three times the secant spread (normalized Fiala curve
`1 - (1 - s)^3`: tangent `(1 - s)^2`, secant `(1 - (1 - s)^3) / (3 s)`). Let
the averages lie in the zonotope

    Z = { G0 + sum_k delta_k D_k + R : |delta_k| <= 1, ||R|| <= eps },

`G0` the Jacobian at the trim, `D_k` the leading principal directions of the
sampled deviations with their coefficient bounds, `R` the residual (all in
box-scaled coordinates). The discrete Lyapunov inequality imposed at every
vertex of `Z`, with the residual absorbed by Petersen's lemma, gives
`V(g(e, K e)) <= rho V(e)` for every `e` in `Omega`. The design is the LMI in
`S = inv(P)`, `Y = K S` over those vertices, with the input rows
`[ubar_j^2, Y_j; Y_j', S] >= 0` and the box rows `S_ii <= box_i^2`,
maximizing the fill `t` in `S >= t diag(box)^2`. Because `Z` depends on
`Omega` and `K`, the design grows the set self-consistently: each round
encloses the averages over `1.5 Omega` and over inputs within `ubar/4` of the
current controller, and solves the LMI with the new set inside `1.5 Omega`
and the new gain within that band of the old one on it, so that every round's
pair is certified by the enclosure it was designed with; the round with the
largest fill is kept. Its vertex inequalities are re-verified by eigenvalues
and the smallest contraction they certify is recorded
(`certifiedContraction <= rho`).

**Hypothesis H3 (the only numerical step).** The ray averages `Gbar(e, K e)`,
`e` in `Omega`, lie in the recorded zonotope. The zonotope is built from
boundary-heavy samples of the inflated set (1200 on the boundary, 600 inside,
recorded seed) and every coefficient bound and the residual are widened by
the margin factor 1.25. The rest is algebra. An independent sample with
another seed checks (2.1), the hold factor and the problem's rows directly
(`terminalSafeSet.certificate`), and a unit test checks fresh ray averages
against the recorded zonotope.

**Hold factor.** The partial-hold maps `e -> e(s)`, `0 < s < h`, are enclosed
over the same samples, and

    mu = max( 1, max_s exp(s/T) max_{Z_s} || F (A_s + B_s K) inv(F) || ),   F = chol(P),

(a convex maximum, attained at a vertex, plus the residual's share) bounds
`sqrt V(x(s)) <= mu exp(-s/T) sqrt V(x_k)` inside every hold. The certificate
is for the declared sampled model, so there is no zero-order-hold gap between
the certificate and the controller.

**Why not the linear certificate.** The previous terminal controller (the
same problem without collision rows, with the CLF row without slack) was
certified on the linearized sampled model only: the fastest contraction `rho*`
with the input bound (time constant 2.1 s at 8 m/s, 1.5 s at 15 m/s against
the required 4 s), and the existence of an admissible input with
`V(x+) <= rho V(x)` on the nonlinear model was an online check (the former
H3). Closing that gap by a Lipschitz bound `L` on the nonlinear remainder,
`sqrt(rho*) + L <= sqrt(rho)`, was tried on October 10 and fails by two
orders of magnitude: the budget is 0.011 at 8 m/s while the remainder ratio
is about `0.9 sqrt(c)` on the level `c` (the modified Fiala tire leaves its
linear range at slips of a few hundredths of a radian, and the former `P`
let the lateral velocity reach 1.4 m/s at `cbar`). The Jacobian enclosure
replaces the single linear model by every Jacobian the controller meets; its
price is a smaller certified set (Section 4), because the tire's variation
over the set must be absorbed with a decay of `1/T`. The study is in
`report/TERMINAL_CONTROLLER_CERTIFICATE_20261010.tex`.

## 3. The CLF tube

For the ellipsoid `{e : e'Pe <= V}` the largest value of `|e_i|` is
`sqrt((inv P)_ii) sqrt(V)`. With the hold factor `mu` of Section 2,
`a_i = mu sqrt((inv P)_ii)` and `r(t) = sqrt(V0) exp(-t/T)`, (2.1) and the
hold factor give at every time, inside the holds included,

    |e_y(t)| <= a_1 r(t),  |e_psi(t)| <= a_2 r(t),  |v_x - v_x*| <= a_3 r(t),
    |v_y - v_y*| <= a_4 r(t),  |yawRate - r*| <= a_5 r(t).                (3.1)

The path station `s` obeys `ds/dt = v_t / (1 - kappa e_y)`, where `v_t` is the
velocity component along the path tangent. The trim has `ds/dt = v_path`.
Writing `theta = yaw - pathHeading = psi* + e_psi` and expanding
`v_t = vx cos theta - vy sin theta` about the trim,

    |v_t - v_t*| <= |dvx| + |dvy| (|sin psi*| + |e_psi|)
                    + |vx*| (|sin psi*| |e_psi| + e_psi^2 / 2) + |vy*| |e_psi|,

so with (3.1)

    |ds/dt - v_path| <= sigma(r) =
        [ a_3 r + a_4 r (|sin psi*| + a_2 r) + |vx*| (|sin psi*| a_2 r + (a_2 r)^2 / 2)
          + |vy*| a_2 r + v_path |kappa| a_1 r ] / (1 - |kappa| a_1 r),          (3.2)

and `|s(t) - s0 - v_path t| <= G(t) = integral_0^t sigma(r(tau)) dtau`.
`sigma` is increasing in `r`, `sigma(lambda r) <= lambda sigma(r)` for
`lambda <= 1`, and `r` decreases, so `G` is concave and the drift remaining
after time `t` is at most `T sigma(r(t))`. On a straight road the heading
enters the station rate only at second order: at 8 m/s and `V = cbar` the
total drift `T sigma` is 4.6 m, against 14.5 m for the first-order bound
`(|vx*| + |vy*|) a_2 r + (a_3 + a_4) r` used until October 9.

The ego rectangle at time `t` therefore lies in a box in path coordinates:

    station in s0 + v_path t +/- (G(t) + stretch(t)),
    lateral in +/- (a_1 r(t) + across(t) + bulge),                         (3.3)

where `across` and `along` are the rectangle's extents normal and tangent to
the path at heading bound `|psi*| + a_2 r(t)`, `stretch = along / (1 - |kappa|
outer)` converts arc length on the outer side to station, and `bulge =
|kappa| L^2 / 8` is the sagitta of the rectangle's long side. This box family
`T(x0)(t)` is the CLF tube of `x0`. It depends on `x0` only through `V(x0)` and
the station `s0`. The ego reference point lies in the smaller box with
`stretch = across = bulge = 0`.

The Euclidean distance between two path boxes is bounded from below by the
larger of their lateral gap and, on a curve of radius `rho_c`, the chord
`2 rho_inner sin(min(pi/2, Delta_s / (2 rho_c)))` at the inner radius of the
two boxes; on a straight path it is `hypot(Delta_s, Delta_d)`.

## 4. The terminal set

Let `B(t)` be the target's forecast rectangle and `q(t)` its reference point,
both in path coordinates, and `T(x)` the tube of `x`. For a state `x` at time
`t_x` define the end of the encounter

    t_e(x) = min( first t >= 0 at which q(t_x + t) is farther than R from
                  every point of the reference-point box of T(x)(t),
                  first t >= 0 from which the relative motion is outside the
                  collision cone (Section 4.1) ),

and require `t_e(x)` to lie within the span the check follows (Section 7),
which the forecast's geometry bounds; it limits the computation only and is
not an encounter duration.

The terminal set is

    S = { x :  V(x) <= cbar,
               T(x)(t) lies inside the road for all t >= 0,
               dist(T(x)(t), B(t_x + t)) > 0 for all t in [0, t_e(x)) }.     (4.1)

In the hybrid model with an absorbing end state `+` (the encounter is over),
the terminal set is `S U {+}`.

`cbar` is the smallest of `terminal.levelMaximum` (1), the certified level
of Section 2 (1: the whole certified set) and `cbar_state / mu^2`, where
`cbar_state` is the largest level whose ellipsoid lies inside the problem's
linear state rows (speed, lateral velocity, yaw rate, sideslip cone, rear
adhesion at the certification braking ratio),

    cbar_state = min_j b_j^2 / (g_j' inv(P) g_j)   for the rows g_j' e <= b_j;

the division by `mu^2` puts the hold midpoints, which lie in the `mu`-inflated
ellipsoid, inside the rows as well. With the certified `P` the state-row
level is far above 1 at every operating point (the certified set keeps the
lateral velocity and yaw rate small), so `cbar = 1`. Because the lateral
extent of the tube is largest at `t = 0`, the road condition is checked
there.

The target's existence ends at its first exit: a target that has left the
range is not followed back in, even if its forecast returns.

**4.1 Proven separation: the collision cone.** On a straight road, after a
grid time `t_s` the ego rectangle's band across the road only shrinks, and its
station interval moves at the trim's station rate `v` and widens by at most
`T sigma(r(t_s))` more, because `sigma(lambda r) <= lambda sigma(r)` and `r`
decays as `exp(-t/T)`. Widened by that drift, the ego box `E` is fixed in a
frame moving at `v` along the road. The target obeys its contract for all
later times, also through a stop (signed speed `V(u) = V_s + A u`,
`u = t - t_s`):

- a straight-line target (`beta = 0`) moves its box `B` by `c s(u)` along and
  `s s(u)` across the road, `s(u) = V_s u + A u^2/2`, `c, s` the cosine and
  sine of its course to the road. In the moving frame the difference of the
  box centres is the parabola
  `q(u) = q(0) + (c V_s - v, s V_s) u + (c A, s A) u^2/2`, and `E` and `B`
  meet at some `u >= 0` iff `q(u)` enters the sum box `|q_s| <= H_s`,
  `|q_d| <= H_d` (the half-widths of `E` and `B` added, with the buffer). The
  relative motions (velocity and acceleration) for which this happens form
  the collision cone of the two boxes, the cone of Chakravarthy and Ghose for
  rectangles and constant acceleration; separation is proven at `t_s` iff the
  relative motion lies outside it. The test is exact for the forecast: the
  least normalized box distance `max(|q_s|/H_s, |q_d|/H_d)` over `u >= 0` is
  attained at `u = 0`, at a vertex or a zero of `q_s` or `q_d`, or where
  `|q_s|/H_s = |q_d|/H_d`; it is evaluated at all of these (at most 11
  candidates), and the boxes meet iff it is at most 1;
- a circling target (`beta ~= 0`) stays in the disk of its circle widened by
  its body reach. A disk beside the ego band, or behind it while the ego moves
  forward, stays apart.

The cone is taken with respect to the terminal controller's converged motion
(the path trim at `v`), not the ego's present velocity, and the transient is absorbed
by the widened box; this is what makes the certificate invariant (Section 5).
A condition on the present relative velocity alone ("the distance is growing
now") would be neither necessary (a target drifting away across the road
while the along-road gap closes is safe although the distance shrinks) nor
sufficient (a faster lead that brakes is safe now and met later); the cone
states that the boxes never meet. The sign conditions of the previous version
(a gap along or across the road that never closes) are the special case in
which one coordinate of `q` alone keeps the boxes apart. On a curved road no
separation is claimed; only the exit ends the encounter.

**History.** The CLF-tube set of October 6 capped the exit at 60 s after each
state and accepted a tube clear for that sliding window (hypothesis H4).
Extending a plan by one hold then checked a new end interval that the old
endpoint had not verified. Requiring the exit alone removes H4 but makes
every state non-terminal for a target that stays in range (a parallel target
at 49.9 m, the braking lead's first frames). A window fixed 60 s after the
encounter's start kept invariance (commit `b836fba`); proven separation
replaces it without any preset time (commit `f5e1ab7`), first as sign
conditions on the gap along or across the road and then as the exact
collision-cone test, which certifies everything the sign conditions did and
also a slow diagonal crosser whose crossing ends after the bound. Lane-hold
backups (the CLF held at the other lane centres of the road, with a guarded
return to the given path) and a startup rollout retried toward the lane the
cone preferred were part of the same study and were removed at the user's
direction, because they are modes (`report/TERMINAL_BACKUP_SET_20261008.tex`).

## 5. Forward invariance

**Proposition.** If `x(0)` is in `S` and a terminal controller acts, then
`x(tau)` is in `S` until the encounter ends, and the ego never meets the target before
the encounter ends. In the hybrid model, `S U {+}` is forward invariant: the
end state `+` is absorbing.

*Proof.* `V(x(tau)) <= exp(-2 tau/T) V(x(0)) <= cbar`. The tube of `x(tau)` has
radius `r_tau(t) = sqrt(V(x(tau))) exp(-t/T) <= r_0(t + tau)`. Since `sigma` is
increasing, `G_tau(t) <= G_0(t + tau) - G_0(tau)`, and by Section 3 the station
of `x(tau)` lies in `s0 + v_path tau +/- G_0(tau)`. Hence every box of the tube
of `x(tau)` lies in the box of the tube of `x(0)` at the same absolute time:

    T(x(tau))(t) is contained in T(x(0))(t + tau).

The same holds for the reference-point boxes, so the exit of `x(tau)` is no
later than the exit of `x(0)`. A relative motion outside the cone for `x(0)`
at a time `t_s` is outside it for `x(tau)` at the same absolute time: the
widened box of `x(tau)` is a subset (smaller tube, smaller remaining drift),
and the target's motion in the same moving frame is the same parabola. So
the end of the encounter seen from `x(tau)` is no later than that seen from
`x(0)`.
Before it the boxes of `x(tau)` are clear of `B` because the larger ones are.
The road condition is inherited the same way. The actual rectangle lies in
`T(x(0))(t)` for all `t` (Section 3), so it does not meet `B` before the
encounter ends. QED

The certificate of Section 2 is for the sampled model: `V(x_(k+1)) <= rho
V(x_k)` at the sample instants, so nesting holds exactly from sample to
sample, and inside a hold the hold factor `mu` bounds the radius, so the tube
(whose half-axes carry `mu`) contains the state between samples as well.

## 6. Recursive feasibility

The problem at sample `k` (with plan length `N_k >= horizonSteps`, the
`prefix`) is:

- the plan is the sampled model's rollout of the inputs from the current
  estimate;
- road, state-limit and handling rows hold at every node and hold midpoint;
- collision clearance holds at every node and hold midpoint after the prefix
  (hard), and is a minimized slack inside the prefix (soft);
- the endpoint lies in `S`.

A plan meeting these rows is *accepted*.

**Hypotheses.**

- H1: the plant is the declared sampled model, and the state is known
  exactly.
- H2: the target follows its forecast (the same constant `A` and `beta`).
- H3: the Jacobians of the sampled error map along the terminal controller
  over `Omega` lie in the recorded enclosure (Section 2). This is the
  certificate's one numerical hypothesis; it is checked offline, and
  `terminalInput` evaluates each appended hold as a guard. (Until October 10,
  H3 was the online existence of such an input, which was not proven.)

The earlier hypothesis H4 (no conflict after a sliding 60-s window) is not
needed: the encounter ends by an exit or by a relative motion outside the
collision cone, both of which only come earlier along the terminal
controller (Section 5).

**Proposition.** Under H1–H3, within one encounter, if the plan of sample `k`
is accepted, the shifted plan at `k+1` is accepted:

- drop the first hold;
- if it is shorter than the prefix or its endpoint is no longer in `S`,
  append holds of the terminal controller.

The sum of the prefix collision deficits does not increase.

*Proof.* By H1 the new state is the plan's second node, so the shifted inputs
reproduce the old nodes 2..N+1 exactly. Every hard row of the old plan holds
there. The old node `prefix+1` was hard (deficit 0) and is now the last
prefix node, so the prefix deficit sum loses the old first node's deficit and
gains zero. The endpoint is the old endpoint at the same absolute time. By H2
the forecast is unchanged, so the endpoint is still in `S`: its exit or
separation is at the same absolute time, within `H` of the same node. An
appended hold satisfies `V(x+) <= rho V(x)` by the certificate of Section 2
(under H3), the input bound by the same certificate, and the state and
handling rows at its nodes and midpoint because `cbar <= cbar_state / mu^2`.
By Section 5 its endpoint is in `S` (or the encounter has ended), and its nodes
and midpoints are inside the
tube of the old endpoint. That tube clears the target by the hard-row
clearance at the check instants, which lie on the tube's grid (Section 7).
QED

The optimizer never has to find this candidate: it is the step `alpha = 0`
of the step-size rule (Section 7). Consequently, once one plan of an
encounter has been accepted, H1–H3 guarantee that no later sample of that
encounter reports no solution.

The start of an encounter is outside this statement. When the target first
enters the range or re-enters it after leaving, the new encounter is an
initial-feasibility question.

## 7. Implementation

**Tube clearance on a grid.** `terminalSafeSet.clear` evaluates (3.3) on the
absolute time grid of step `dt = terminal.timeStepSeconds` (5 ms; it divides
`h/2`, so hold nodes and midpoints lie on it). At each grid point the box gap
must be at least

    max( safetyMarginMeters,  dt/2 * (egoBodySpeed + targetBodySpeed) ).

The second term is the continuous-time no-collision allowance. Between grid
points each body moves at most its body-point speed times `dt/2`:

- the ego speed is bounded by `|v*| + a_3 r + a_4 r + (|r*| + a_5 r) reach`;
- the target speed comes from the forecast, including its yaw rate.

The first term is the problem's own sampled collision clearance (0.2 m at
nodes and midpoints). It is needed so that holds appended by the terminal
controller meet the same hard rows as every other node. The set is otherwise
`dist > 0`.

`G` uses a left Riemann sum, an upper bound because `sigma(r(t))` decreases.
The target table is shared by the checks of one sample and extended lazily.

**Order of the check.** `terminalSafeSet.clear` first evaluates the node's
own grid point alone: the tube and the forecast at that one time, the exit
test and the cone test of Section 4.1 (`localParabolaMeetsBox`). A relative
motion outside the cone, or a target already out of range, is thus certified
in closed form, without a grid. Only otherwise does the check follow the
grid in 2-s chunks (the cone test at every grid point, vectorized), ending at
the first exit or cone certificate after confirming the grid points before it
(and the certificate's own point), over a span that `terminalSafeSet.scanBound`
derives from the forecast:

- straight road, straight-line target: the relative motion is the parabola
  of Section 4.1. If it enters the box of the *settled* tube (level 0) at
  `u_hit`, the tube meets the target then unless the target has left the
  range before; the first time at which the reference point is surely out of
  range (farther than `R` plus the reference box's largest half-diagonal) is
  computed from the same parabola, and if it is not before `u_hit` the node
  is rejected at once (`insideCollisionCone`), otherwise the span ends at
  that time. If the parabola never enters the settled box, it leaves the box
  of the *whole* tube (widened by the total drift `T sigma(r0)`) for good at
  some `u_out`, where the cone certifies at the latest; the span is `u_out`,
  or the settling time `T log(max(T sigma(r0), a_1 r0) / safetyMarginMeters)`
  when the parabola never leaves (a co-moving target inside the widened box,
  which the shrinking box releases as it settles);
- circling target: the span ends when the tube's box has passed the target's
  disk, where the disk test certifies;
- curved road: there is no cone; the span is the settling time plus the time
  the ego needs for the range diameter, `2R/v`. A target still in range then
  has neither passed nor been passed, and the node is rejected
  (`noExitWithinSpan`).

The span is a bound on the computation, not an encounter duration; at
`V = 0` the settled and whole boxes coincide and the check is the single
closed-form evaluation. The issued plan records how its encounter ends
(`terminalEnd`: `exit` or `outsideCollisionCone`).

**Level.** `terminalSafeSet.level` bisects the largest `c` in `[0, cbar]` whose
tube at a given station is clear (clearance is monotone in `c` because tubes
are nested in `c`). An endpoint verified in `S` keeps its own `V` if the
bisection returns slightly less.

**Terminal row.** In the convex problem the endpoint satisfies one
second-order cone,

    || F (e(y) + J dx_N) || <= sqrt(c*),   F = chol(P),

linearized at the anchor endpoint `y`, with `c*` the level at `y`'s station.
When `y` is in `S` the zero correction satisfies it.

**Anchor.** Later samples start from the previous accepted plan, shifted by
one hold, with terminal-controller holds appended if needed
(`terminalSafeSet.terminalInput`): `u = u* + K e(x)` with the certified gain,
one nonlinear hold. The function also evaluates the hold's decrease, input
box and rows, as a guard on the certificate and as the decision for a state
outside the certified set. At startup,
the potential-field rollout runs until its first node in `S` from `prefix` on,
at most `maximumHorizonSteps`. The rollout supplies a linearization, never
an issued input: its endpoint is certified like any other, and the
optimization decides feasibility.

**Step-size rule.** The affine solution is a search direction. The issued
plan is the rollout, on the sampled nonlinear model, of
`anchor + alpha * correction` for the largest `alpha` in
`terminal.acceptanceSteps = [1, 1/2, ..., 1/32, 0]` that:

- meets every hard row;
- ends in `S`;
- does not increase the prefix deficit sum of the anchor.

`alpha = 0` is the candidate of Section 6. If no step is acceptable (only
possible when the anchor itself is not a feasible plan, e.g. at startup), the
problem is linearized again at the full step's rollout. This is repeated up
to `terminal.sqpIterations` (2) times, and then no solution is reported. This
is one sequential convex method, not a second algorithm, and the previous
plan is never issued unless it satisfies the current problem. The issued plan
is always a nonlinear rollout, so the shift in Section 6 reproduces it.

**Trust scale.** The nonlinear rollout of the full affine step departs from
the affine prediction by the second-order remainder. Its size over the
step's input box size squared estimates the coefficient that sets the next
input trust scale (`localTrustRecord`).

## 8. Measured behavior

The present certificate (Section 2: the ray-averaged enclosure and the
self-consistent growth on the linear terminal controller) is measured in
`report/TERMINAL_SET_ALTERNATIVES_20261010.tex` against the first certified
version, commit `8faf395`, on the same 98 encounters interleaved under the
same load: exact 14 of 14 against 12 of 14, noisy 53 of 84 against 39 of 84,
no collision in either (least clearance 0.148 m exact, 0.260 m noisy). The
certified set reaches 1.5 to 1.6 m of lateral error at 8 m/s and 2 m at
15 m/s (0.6 to 1.0 m before), so the plans no longer have to end near the
path: the first-frame horizon falls from 120 to 130 holds to 109 at the
median and the 95th-percentile horizon from 217 to 166 holds (exact), and
the CLF stage's numerical failures on long plans fall from 18 to 1
encounter. Against the last uncertified design (commit `deb4dac`: 14 of 14
and 54 of 84) the certified controller is equal on the exact encounters and
within one on the noisy ones. The twelve noisy braking leads still stop at
their first frames (the startup problem of the first estimate, unchanged
since October 8). The force-level variant of the controller (commit
`e2046e0`, not kept) measured 13 of 14 and 53 of 84 in the same way.

The first certified version (commit `8faf395`, pointwise Jacobians, fixed-gain
final step) was measured in `report/TERMINAL_CONTROLLER_CERTIFICATE_20261010.tex`
against the linear certificate of commit `deb4dac`: exact 12 of 14 against 14
of 14, noisy 39 of 84 against 54 of 84, no collision in either. Fourteen of
the fifteen new noisy failures and both exact ones were `clfNoNumericalResult`,
the CLF stage's conic solve failing numerically on the longer plans that the
0.6-m set required (horizon with a target present 81 holds at the median
against 47; first frame 124 against 90).

The single-backup set with the collision cone and the linear certificate is
measured in `report/TERMINAL_BACKUP_SET_20261008.tex` (third and fourth
addenda). With
the grid check of October 9 (variant H): exact states 14 of 14; noisy 55 of
84, the same runs as the CLF-tube set of October 6 with its sliding window
(96 of 98 runs identical in outcome and length), no collision. With the
closed-form check of October 10 (variant I2, commit `deb4dac`): exact 14 of
14, noisy 54 of 84 (one turning crossing stops on a 4.7-s conic solve of a
222-hold plan), no collision; the membership test costs 0.2 to 0.9 ms per
call where it is closed-form (against 0.6 to 12.9 ms before) and 1.3 to
1.7 ms where it still follows the grid (circling targets, curves), while the
end-to-end controller time, dominated by the conic solver on long plans, is
unchanged at the median and slightly worse in the tail. All 12 noisy braking
leads stop at the first frame in every variant: the first estimate is a lead
at nearly the ego's speed in the ego lane, the terminal controller's tube
meets it, and the startup rollout cannot pass it and return within
`maximumHorizonSteps`; one CLF certified over 4-8 m/s and over 7.5-15 m/s
exists (same report), so a cruise speed chosen in such an interval could
make that state terminal. With lane-hold backups (removed) the result was 59
of 84. The CLF-tube set of
October 6 is measured in
`report/TERMINAL_SAFE_SET_RECURSIVE_FEASIBILITY_20261006.tex`, for the
exact-state campaigns:

- the ode45 plant;
- the declared RK4 plant, on which H1 holds exactly;
- the noisy-estimator campaign.

## 9. Joint estimator–controller design

In the closed loop with the estimator, H1 and H2 fail. The state is an
estimate `xhat` with a certified enclosure `x in xhat + E`, the target is an
information set `Q_k` of constant-parameter states, and the plant differs
from the model. The terminal set has to absorb three effects.

**9.1 Ego: the CLF tube becomes an ISS tube.** The terminal controller acts on
`xhat`. Over one hold, the effect of the estimate error on the true successor
is bounded, in CLF units, by

    gamma_E = sup || F ( e(f(x, kappa(x + eps))) - e(f(x, kappa(x))) ) ||,
              over eps in E.

Then `sqrt V(x_(k+1)) <= sqrt(rho) sqrt V(x_k) + gamma_E` for the true state,
and

    r(t) = exp(-t/T) r_0 + r_inf (1 - exp(-t/T)),
    r_inf = gamma_E / (1 - sqrt(rho))  (about gamma_E T / h),
    r_0  = sqrt V(xhat) + eta_E,  eta_E = sup_eps || F (e(xhat + eps) - e(xhat)) ||.   (9.1)

The tube (3.3) with `r(t)` from (9.1) is nested exactly as in Section 5 iff
`r_1 <= exp(-h/T) r_0 + r_inf (1 - exp(-h/T))`, which is the same ISS
recursion. The robust terminal set is (4.1) with this radius and
`max(r_0, r_inf)^2 <= cbar`. It is nonempty only if `r_inf^2 < cbar`, and if
the steady tube of radius `r_inf` around the path clears the target.

`gamma_E` is the estimate error's effect through the input over one hold, not
the size of the estimate innovation. Using the innovation instead multiplies
it by `T/h = 80` and leaves nothing of `cbar`. The present code inflates the
tube by the current enclosure only (`positionRadius`, `yawRadius`), which
covers `eta_E` in position and yaw but not `r_inf`.

*Size.* With the certificate's input bound, `|K_j eps| <= ubar_j ||F eps||` on
`V <= 1`, so `gamma_E <= sqrt(2) ||F b diag(ubar)|| eta_E`. Here `b` is the
trim's sampled input matrix. That norm is 0.20 at 8 m/s and 0.29–0.30 at
15 m/s, and `1/(1 - sqrt(rho)) = 80.5` at `T = 4 s`, so

    r_inf <= 16.4 eta_E (8 m/s), 23.4 eta_E (15 m/s).

`r_inf^2 < cbar` then needs `eta_E <= 0.039` at 8 m/s on the straight road
(0.058 on the curve, 0.043 and 0.041 at 15 m/s).

Two design levers set this floor:
- `T` enters through `1/(1 - exp(-h/T))`, roughly `T/h`. At the fastest
  certified time constant (2.1 s at 8 m/s) the floor halves.
- A terminal controller with lower gain in the poorly estimated directions
  (lateral velocity, yaw rate) lowers `||F b K||`.

The estimation error the joint design must deliver is measured in Section 8.

**9.2 Plan: open-loop propagation of the innovation.** At `k+1` the new
estimate departs from the plan's second node by the innovation `delta`. The
shifted inputs then reproduce the old plan only up to `Phi_j delta`, `Phi_j`
the transition of the linearized model over `j` holds. The bicycle has no
restoring force on lateral position, and yaw errors integrate into lateral
drift, so `|Phi_j delta|` grows with `j h v`. Robust recursive feasibility
needs one of the following:

- a terminal row tightened by `sup |Phi_N delta|` and hard rows tightened along
  the horizon (Chisci, Rossiter and Zappa, 2001, *Automatica* 37(7));
- or a plan parametrized as feedback after the prefix,
  `u = v + K (x - z)`, so that `delta` contracts (Mayne, Seron and Rakovic,
  2005, *Automatica* 41(2)).

In the present design the terminal phase is already feedback (the CLF
controller); only the optimized part of the plan is open loop.

**9.3 Target: nested information sets.** Let `Q_k(t)` be the forecast of the
information set. A terminal set built against `Q_k(t)` (every member cleared
until all of `Q_k(t)` has left the range) is monotone: a smaller information
set gives a larger terminal set. Robust recursive feasibility therefore needs

    Q_(k+1)(t) is contained in Q_k(t)   for t >= t_(k+1),                    (9.2)

that is, the estimator must publish information sets that only shrink. A
set-membership update `Q_(k+1) = Phi_h(Q_k) intersect M_(k+1)` with the
measurement-consistent set `M_(k+1)` satisfies (9.2) by construction. The
present high-gain NRMM observer publishes ISS enclosures that are not nested
in time, and its intervals for `A` and curvature stay at the contract width
(about +/-1.1 m/s^2 and +/-0.034 1/m). The reach set of that contract widens by
about `0.5 Delta A t^2` along the path and `0.5 Delta kappa V^2 t^2` across it.
At 8 m/s and 3 s that is about 5 m along and 10 m across.
A terminal set robust to it is empty whenever the exit lies more than a few
seconds ahead.

The joint design therefore asks two things of the estimator:

- certified constant-parameter sets that shrink with the observation window
  (a set-membership parameter stage); the terminal set's exit time sets how
  narrow they must be;
- an ego error bound whose effect through the input, `gamma_E`, keeps `r_inf`
  inside `cbar`.

It asks one thing of the controller: the tightening of 9.2, or the feedback
parametrization. The measured sizes of these quantities are in the report of
Section 8.

## 10. Limitations

- Recursive feasibility is proven within one encounter, on the declared
  sampled model with exact information (H1, H2), and on the enclosure
  hypothesis H3 of the terminal controller's certificate, which is numerical
  (sampled with a margin), not an interval proof. Each new encounter (a
  target appearing or re-entering) is an initial-feasibility question.
- The collision cone is applied only on a straight road. On a curve only
  the exit ends an encounter, and a target that stays in range without either
  event within the span of Section 7 makes the state non-terminal, also when
  it would in fact never be met: the span decides completeness, not
  soundness.
- The hold factor is itself certified from the enclosures of the
  partial-hold maps (Section 2); the ode45 plant of the experiments is not
  the declared RK4 model (H1).
- The certified set reaches 1.5--1.6 m of lateral error at 8 m/s and the
  2-m certification box at 15 m/s, bounded by the enclosure's growth with
  the set (Section 2); the box and the input box are configuration settings
  (`clf.certification*`), and the synthesis reports when the robust LMI has
  no solution. A common certificate over a range of cruise speeds was not
  found with one quadratic function
  (`report/TERMINAL_SET_ALTERNATIVES_20261010.tex`).
- The terminal set is certainty-equivalent in the target. Section 9 is a
  derivation; only the ego enclosure's position and yaw parts are
  implemented.
