# One analytic nominal CLF

The active controller uses one target-independent function throughout avoidance
and recovery. There is no encounter-dependent CLF, nominal-control substitution,
or executable backup. Every issued command comes from a completed CLF solve.

## Function and its scope

For the given path and desired cruise trim, define

    e(x) = [e_y; e_psi; vx-vx_ref; vy-vy_ref; r-r_ref],
    V(x) = e(x)' P e(x),       P > 0.

Longitudinal path phase is free. `nonlinearBicycleModel.cruise` computes the trim
and reads P, the terminal controller's gain K and their certificate from
`config/clfMatrices.json`; it never computes them. `nominalValue` evaluates
this quadratic directly. There is no LQR design; the gain K is used only by
the terminal controller `u = u* + K e` of the terminal set
([TERMINAL_SAFE_SET.md](TERMINAL_SAFE_SET.md), Section 2), never by the
optimizer. No nominal-policy rollout, finite-difference Hessian, cost-to-go
kernel, or additional terminal value is used to construct the CLF.

## Requested decrease: a convergence time constant

Each sample requires

    V(x_next) <= rho * V(x) + slack,   rho = exp(-2 h / T),

with h = 0.05 s and T = `clf.convergenceTimeConstantSeconds` = 4 s. The error
size sqrt(V) then shrinks by at least 1/e every T seconds, in every direction
of the error as V measures it; one lane width (3.66 m) returns to within
0.3 m in about 10 s. T replaces the former `nominalClf.decreaseFraction`
(eta = 0.5 times e'Qe), whose speed differed between error directions by a
factor of about 25 for the same eta.

## Offline certificate of P and the terminal controller

`scripts/synthesizeClfMatrices.m` computes P and K before experiments, for
each operating point (vehicle parameters, speed, path curvature, sample time
and CLF settings), and writes them with their key and certificate to
`config/clfMatrices.json`. A controller call at an operating point without an
entry is rejected with `collisionAvoidanceController:missingClfMatrix`, an
entry without the certificate with `collisionAvoidanceController:staleClfMatrix`;
nothing is synthesized at run time, and the run time does not need the SDP
solver.

The certificate is on the sampled nonlinear model itself, not on its
linearization. Let `g(e, du)` be the transverse error after one hold from the
state with error `e` under the input `u_ref + du`, and `G = [dg/de, dg/d(du)]`
its Jacobian from the RK4 variational equations and the error chart
(`terminalSafeSet.jacobians`). The trim is a fixed point, so along the ray
from `0` to `(e, du)`

    g(e, du) = Gbar(e, du) [e; du],   Gbar(e, du) = integral_0^1 G(t e, t du) dt,

exactly. The certificate encloses the ray averages `Gbar` along the controller
over the set (`terminalSafeSet.rayJacobians`, Gauss-Legendre quadrature with
8 nodes; the quadrature's residual in the identity is added to the enclosure's
residual as a rank-one term), so a discrete Lyapunov inequality that holds for
every matrix of the enclosure holds for the nonlinear map. Enclosing the ray
averages rather than the pointwise Jacobians matters for a saturating tire:
the averages spread like the secants of the force curve, the pointwise
Jacobians like its tangents, and at the slips the set reaches the tangent
spread is two to three times the secant spread (the first coefficient bound
drops from about 0.13 to 0.06 at 8 m/s). The enclosure is a zonotope in
box-scaled coordinates (the trim's Jacobian, the leading principal directions
of the sampled deviations with their coefficient bounds, and a residual norm,
all widened by a margin factor of 1.25), and the inequality is imposed at its
vertices with the residual absorbed by Petersen's lemma. With `S = inv(P)` and
`Y = K S` the LMI is

    maximize t
    subject to  [rho S, N_v', Z'; N_v, S - lambda_v I, 0; Z, 0, lambda_v I] >= 0   (every vertex v),
                N_v = A_v S + B_v Y,   Z = eps [S; Y],
                [ubar_j^2, Y_j; Y_j', S] >= 0       (j = steering, braking),
                S_ii <= box_i^2,   S >= t diag(box)^2,

with `rho = exp(-2h/T)` the requested contraction, `ubar =
[clf.certificationSteeringRadians; clf.certificationBrakingRatio]` (0.075 rad
and 0.125 on `V <= 1`) and `box` the certification box of
`clf.certificationLateralMeters`, `...HeadingRadians`,
`...SpeedMetersPerSecond`, `...LateralVelocityMetersPerSecond` and
`...YawRateRadiansPerSecond`, inside which `Omega` must lie. Because the
enclosure depends on `Omega` and `K`, the script grows the set
self-consistently: the plain LMI inside half the box gives a first pair; each
round encloses the ray averages over 1.5 times the current set and over
inputs within a quarter of `ubar` of the current controller, and solves the
LMI with the new set inside that inflated set (`S' <= 1.5 S`) and the new
gain within that band of the old one on it (`|(K' - K)_j e| <= ubar_j / 4`
on the new set). Every round's pair is therefore certified by the enclosure
it was designed with, and the set may grow by 1.5 in `V` per round until the
enclosure's growth stops it; the pair with the largest fill is kept. Its
vertex inequalities are re-verified by eigenvalues and the smallest
contraction they certify is recorded (`certifiedContraction`, at most `rho`).
The partial-hold maps are enclosed the same way, on the same samples, to give
the hold factor by which the CLF tube is inflated between samples.
YALMIP/SeDuMi from `solver/`.

The only non-algebraic step is the enclosure (hypothesis H3 of
TERMINAL_SAFE_SET.md): it is built from 1200 boundary and 600 interior
samples of the inflated set with a recorded seed. The entry records the
samples' seed, the coefficient bounds, the residual (with the quadrature's
share), the largest slip angles reached, the inputs' maxima, and an
independent sampled check with another seed (`terminalSafeSet.certificate`:
worst contraction, hold factor, the problem's rows); `nominalClfTest` repeats
that check and tests fresh ray averages against the recorded zonotope.

The former certificate (until October 10, 2026) was the fastest contraction
of the linearized sampled model with the input bound and a shape constraint
(time constants 2.1 s at 8 m/s and 1.5 s at 15 m/s against the required 4 s);
its gain was discarded and the nonlinear decrease was checked online by the
terminal controller. Closing that gap with a Lipschitz bound on the nonlinear
remainder fails by two orders of magnitude (the modified Fiala tire is
nonlinear at slips of a few hundredths of a radian), which is why the
certificate now encloses the Jacobians over the set instead of trusting one
linearization. With the pointwise Jacobians (October 10, first version) the
certified lateral extent was 0.6 m at 8 m/s; the ray averages and the
self-consistent growth (October 11) are what recover the former size
(`report/TERMINAL_SET_ALTERNATIVES_20261010.tex` compares the constructions
tried: secant enclosure, force-level input map, saturation hull,
poly-quadratic function, slower decay, polytope, interval certificate).

**This quadratic is not a global nonlinear CLF.** Large heading errors, tire
saturation, state-domain boundaries, braking memory and the MPC trust region
can make zero slack unavailable even without a target. For example, at a
20-m lateral displacement the sampled tests found positive slack despite an
actual reduction in V. The former cost-to-go argument covered a different,
feedback-reachable region; it cannot be transferred to this simpler function.
The scenario campaign and displaced-state tests therefore measure eventual
cruise recovery separately from local zero-slack decrease.

If the exact nonlinear inequality holds with zero slack on a suitable invariant
domain, it gives exponential decay of the transverse error with time constant T. Zero slack in the
**affine approximation**, together with a fixed model-error tolerance, does not
establish that exact inequality or global asymptotic convergence.

## One convex first-successor constraint

The trajectory is linearized once around a nonlinear rollout. At its first
successor, use the analytic Frenet error Jacobian J:

    e_aff = e(xbar_1) + J(xbar_1) * dx_1,
    V_aff = ||chol(P) * e_aff||^2.

The same affine dynamics relate dx_1 to the first input correction. Its Hessian
is positive semidefinite, so the first-step CLF epigraph is a convex quadratic
constraint represented by one second-order cone, so the optimization is a sparse
conic QP/SOCP rather than a QP with exclusively affine constraints.

The objective is

    (rho + epsilon * lossLower * R(dU)) / max(1,V(x)),
    R(dU) = (1/(2M)) sum_i ||diag(r_delta,r_b)^(-1) dU_i||^2.

The input correction box implies 0 <= R <= 1. `lossLower` is a lower bound on
`e'Qe` over the current affine uncertainty set (exactly `e'Qe` with zero error).
Therefore, at an exact optimum, the regularizer changes minimum physical
CLF slack by at most `epsilon * lossLower`, with
`epsilon = clfTieTolerance = 1e-4`. Its allowance vanishes at nominal behavior.
Inputs are penalized relative to the
linearization input, not zero. This also prevents arbitrary future inputs
from becoming poor trust centers after shifting. Solver tolerances add their
own numerical error.

With an observer enclosure the row is the same, on the estimate:
`budget = exp(-2h/T)*V(xhat)` and `lossLower = e0'Qe0` at the estimate.
Estimation error enters the decrease as an input (input-to-state), not through
a worst case. The previous worst-case form required the successor's largest
value over its enclosure to stay below `exp(-2h/T)` times the smallest current
value consistent with the current enclosure, `max(0, sqrt(V0) - b0)^2`. Near
the path that budget is zero while the successor's enclosure is not, so no
input satisfies it. The CLF stage then minimized the next value greedily; in a
noisy 15-m/s head-on its braking ratio chattered with standard deviation 0.21,
and the run did not recover within 100 s (2026-10-05).
`clfWorstNextValue` equals the modeled successor value and `clfCurrentBudget`
the budget above.

The remaining nonlinear error-chart and dynamics remainders are not enclosed.
Neither zero affine slack nor this implementation proves exact asymptotic
recovery of the nonlinear uncertain plant; with estimation error the closed
loop can at best approach a neighborhood of the path set by that error. See the conditional theorem in
`OBSERVER_ROBUST_PCBF_THEORY.tex` for the additional premises.

This constraint controls the first successor only. Future inputs are chosen
through horizon feasibility and the input correction cost; they do not each
receive a CLF decrease constraint. After shifting, the planned second input
becomes the next first-input anchor. Consequently, a seemingly small change
to a tie objective can alter later anchors and closed-loop return behavior
without materially changing the current minimum CLF slack. The removed input
continuity preference exposed this effect: correction bounds remained active
while the shifted steering plan repeatedly returned toward near-straight
inputs. The frozen-problem replay and fourteen-case rollback results are in
`report/CLF_RETURN_DIAGNOSIS_20261003.tex`. No extra path-deviation penalty or
second CLF is introduced to address that regression.

The PCBF budget is a hard constraint of this convex problem. It comes from the
primary slack minimization of the same frame. A zero primary value never skips
CLF optimization.

## Initialization

The completed CLF optimizer point is issued directly; it is not replayed through
the nonlinear model or damped.
A frame uses the shifted previous plan, or a potential-field rollout at
startup; a shifted problem infeasible inside the trust region is re-solved once
from a fresh potential-field rollout.
A frame still without a solution reports it; no enlarged box follows. See
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md).

`nominalFeedback` and `nominalGuidanceParameters` supply path guidance and a
trim steering correction only for constructing the potential-field seed. They neither
define another CLF nor provide an issued control. The nominalClf configuration
group retains only their guidance parameters; retired cost-to-go horizon and
tail-level settings are rejected.
