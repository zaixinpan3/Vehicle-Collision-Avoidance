# Terminal safe set and recursive feasibility

This note defines the terminal set used by `solvePredictiveControl`, proves
its forward invariance under the terminal controller, and states the
recursive-feasibility property of the resulting optimization together with
its hypotheses. The theory is in continuous time; Section 7 describes how the
implementation samples it. Section 9 derives what the set has to become when
the estimator and the controller are designed together.

## 1. Purpose

After the prediction horizon the controller is the same optimization problem
without its PCBF (collision) rows. What remains is the CLF row, now without
slack, so the vehicle dissipates to the nominal behavior (cruise on the given
path at the reference speed). The terminal set is the set of states from which
that controller never collides with the target, for the target's forecast
motion under the motion contract (constant tangential acceleration `A` and
constant sideslip `beta`), until the target leaves the perception range
`R = encounterRangeMeters`. A target outside the perception range no longer
exists for the controller.

No avoidance maneuver, lane, side or mode is designed in advance. The set
contains no safety distance of its own; Section 7 lists the two sampling
allowances of the implementation.

If the set cannot be reached, the problem has no solution and the controller
reports `collisionAvoidanceController:noOptimizationSolution`.

## 2. The terminal controller

Let `e(x) = [e_y; e_psi; vx - vx*; vy - vy*; r - r*]` be the transverse error
to the path trim and `V(x) = e(x)' P e(x)` the CLF of
[NOMINAL_CLF.md](NOMINAL_CLF.md). A terminal controller is any input law whose
closed loop satisfies

    dV/dt <= -(2/T) V,                    T = clf.convergenceTimeConstantSeconds,

or, sampled with hold `h`, `V(x+) <= rho V(x)`, `rho = exp(-2h/T)`. Both give

    V(x(t)) <= exp(-2t/T) V(x(0)).                                        (2.1)

The offline synthesis certifies on `V <= 1` a linear feedback of the sampled
linearized model with bounded input, `|K_j e| <= ubar_j`, and a contraction
`rho* < rho` (time constant 2.1 s at 8 m/s and 1.5 s at 15 m/s against the
required 4 s). On the nonlinear model the existence of an admissible input
with `V(x+) <= rho V(x)` is not proven; it is checked at every hold the
controller appends (Section 6, hypothesis H3).

## 3. The CLF tube

For the ellipsoid `{e : e'Pe <= V}` the largest value of `|e_i|` is
`a_i sqrt(V)`, `a_i = sqrt((inv P)_ii)`. With `r(t) = sqrt(V0) exp(-t/T)`,
(2.1) gives at every time

    |e_y(t)| <= a_1 r(t),  |e_psi(t)| <= a_2 r(t),  |v_x - v_x*| <= a_3 r(t),
    |v_y - v_y*| <= a_4 r(t),  |yawRate - r*| <= a_5 r(t).                (3.1)

The path station `s` obeys `ds/dt = v_t / (1 - kappa e_y)`, where `v_t` is the
velocity component along the path tangent. The trim has `ds/dt = v_path`.
Writing `theta = yaw - pathHeading = psi* + e_psi`,

    |v_t - v_path| <= |dvx| + |dvy| + (|vx*| + |vy*|) |e_psi|,

so with (3.1)

    |ds/dt - v_path| <= sigma(r) =
        [ (|vx*| + |vy*|) a_2 r + (a_3 + a_4) r + v_path |kappa| a_1 r ]
        / (1 - |kappa| a_1 r),                                             (3.2)

and `|s(t) - s0 - v_path t| <= G(t) = integral_0^t sigma(r(tau)) dtau`.
`sigma` is increasing in `r` and `r` decreases, so `G` is concave.

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
both in path coordinates. For a state `x` define the encounter exit

    t_exit(x) = first t >= 0 at which q(t) is farther than R from every
                point of the reference-point box of T(x)(t),

capped at `H_enc = terminal.horizonSeconds` (60 s). The cap is the encounter
window. A target that stays inside the range for longer is taken to have
ended its encounter there (hypothesis H4 below). Without the cap, a target
driving beside the ego at the same speed, inside the range but never in its
way, would make every state non-terminal.

The terminal set is

    S = { x :  V(x) <= cbar,
               T(x)(t) lies inside the road for all t >= 0,
               dist(T(x)(t), B(t)) > 0 for all t in [0, t_exit(x)) }.      (4.1)

`cbar` is the smaller of `terminal.levelMaximum` (1, the region in which the
CLF was synthesized) and the largest level whose ellipsoid lies inside the
problem's linear state rows (speed, lateral velocity, yaw rate, sideslip cone,
rear adhesion at the certification braking ratio),

    cbar_state = min_j b_j^2 / (g_j' inv(P) g_j)   for the rows g_j' e <= b_j.

At 8 m/s it is 0.403 on the straight road and 0.907 on the curve (rear
adhesion binds); at 15 m/s it is 1.146 and 1.541, so `cbar = 1`. Because the
lateral extent of the tube is largest at `t = 0`, the road condition is checked
there.

The target's existence ends at its first exit: a target that has left the
range is not followed back in, even if its forecast returns.

## 5. Forward invariance

**Proposition.** If `x(0)` is in `S` and a terminal controller acts, then
`x(tau)` is in `S` for every `tau >= 0`, and the ego never meets the target
before the target leaves the perception range.

*Proof.* `V(x(tau)) <= exp(-2 tau/T) V(x(0)) <= cbar`. The tube of `x(tau)` has
radius `r_tau(t) = sqrt(V(x(tau))) exp(-t/T) <= r_0(t + tau)`. Since `sigma` is
increasing, `G_tau(t) <= G_0(t + tau) - G_0(tau)`, and by Section 3 the station
of `x(tau)` lies in `s0 + v_path tau +/- G_0(tau)`. Hence every box of the tube
of `x(tau)` lies in the box of the tube of `x(0)` at the same absolute time:

    T(x(tau))(t) is contained in T(x(0))(t + tau).

The same holds for the reference-point boxes, so the exit of `x(tau)` is no
later than the exit of `x(0)`, and before it the boxes of `x(tau)` are clear of
`B` because the larger ones are. The road condition is inherited the same way.
The actual rectangle lies in `T(x(0))(t)` for all `t` (Section 3), so it does
not meet `B(t)` before the exit. QED

The sampled terminal controller satisfies `V(x_(k+1)) <= rho V(x_k)` at the
sample instants. Nesting then holds exactly from sample to sample. Inside a
hold, (2.1) is assumed as in the continuous-time theory; it is not checked
between samples.

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
- H3: at every endpoint the closed loop reaches, the terminal controller has
  an input with `V(x+) <= rho V(x)` that meets the road, state and handling
  rows (checked online).
- H4: an encounter that has not ended `H_enc` after a plan's endpoint brings no
  conflict with that endpoint's tube after that time. Each appended hold moves
  the window end by `h`; without H4 the new end interval would be unverified.

**Proposition.** Under H1–H4, if the plan of sample `k` is accepted, the
shifted plan at `k+1` is accepted:

- drop the first hold;
- if it is shorter than the prefix or its endpoint is no longer in `S`,
  append holds of the terminal controller.

The sum of the prefix collision deficits does not increase.

*Proof.* By H1 the new state is the plan's second node, so the shifted inputs
reproduce the old nodes 2..N+1 exactly. Every hard row of the old plan holds
there. The old node `prefix+1` was hard (deficit 0) and is now the last
prefix node, so the prefix deficit sum loses the old first node's deficit and
gains zero. The endpoint is the old endpoint at the same absolute time. By H2
the forecast is unchanged, so the endpoint is still in `S`. An appended hold
satisfies `V(x+) <= rho V(x)` and its own rows (H3). By Section 5 its endpoint
is in `S`, and its nodes and midpoints are inside the tube of the old endpoint.
That tube clears the target by the hard-row clearance at the check instants,
which lie on the tube's grid (Section 7). QED

The optimizer never has to find this candidate: it is the step `alpha = 0`
of the step-size rule (Section 7). Consequently, once one plan of an
encounter has been accepted, H1–H4 guarantee that no later sample of that
encounter reports no solution.

The start of an encounter is outside this statement. When the target first
enters the range, or re-enters it after leaving, the new information is an
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
The target table is filled in 2-s chunks and reused within a sample.
`terminal.horizonSeconds` (60 s) is the encounter window `H_enc`. A tube clear
for the whole window is in `S` (`clearThroughHorizon`, exit `Inf`).

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
one hold, with the terminal controller appended if needed
(`terminalSafeSet.terminalInput`). The terminal controller tries:

- the path guidance;
- the input that minimizes the successor `V` on the trim's linear sampled
  model;
- a grid around that input.

It keeps the admissible input with the smallest successor `V`. At startup,
the potential-field rollout runs until its first node in `S` from `prefix` on,
at most `maximumHorizonSteps`.

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

See `report/TERMINAL_SAFE_SET_RECURSIVE_FEASIBILITY_20261006.tex` for the
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

- Recursive feasibility is proven on the declared sampled model with exact
  information (H1, H2) and conditionally on two further hypotheses:
  - H3, which is checked online, not proven;
  - H4, the finite encounter window.
- The tube assumes the continuous-time decrease (2.1) inside each hold.
- `cbar` binds the rear-adhesion row at the certification braking ratio.
  Other input-dependent rows (the actual braking of the terminal controller)
  are checked online.
- The terminal set is certainty-equivalent in the target. Section 9 is a
  derivation; only the ego enclosure's position and yaw parts are
  implemented.
