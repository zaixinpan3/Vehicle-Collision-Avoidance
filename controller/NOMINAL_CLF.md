# One analytic nominal CLF

The active controller uses one target-independent function throughout avoidance
and recovery. There is no encounter-dependent CLF, nominal-control substitution,
or executable backup. Every issued command comes from a completed CLF solve.

## Function and its scope

For the given path and desired cruise trim, define

    e(x) = [e_y; e_psi; vx-vx_ref; vy-vy_ref; r-r_ref],
    V(x) = e(x)' P e(x),       P > 0.

Longitudinal path phase is free. `nonlinearBicycleModel.cruise` computes the trim
and reads P from `config/clfMatrices.json`; it never computes P. `nominalValue`
evaluates this quadratic directly. There is no LQR design, and the CLF carries
no feedback law. No nominal-policy rollout, finite-difference Hessian,
cost-to-go kernel, or additional terminal value is used to construct the CLF.

## Requested decrease: a convergence time constant

Each sample requires

    V(x_next) <= rho * V(x) + slack,   rho = exp(-2 h / T),

with h = 0.05 s and T = `clf.convergenceTimeConstantSeconds` = 4 s. The error
size sqrt(V) then shrinks by at least 1/e every T seconds, in every direction
of the error as V measures it; one lane width (3.66 m) returns to within
0.3 m in about 10 s. T replaces the former `nominalClf.decreaseFraction`
(eta = 0.5 times e'Qe), whose speed differed between error directions by a
factor of about 25 for the same eta.

## Offline synthesis of P

`scripts/synthesizeClfMatrices.m` computes P before experiments, for each
operating point (vehicle parameters, speed, path curvature, sample time and
CLF settings), and writes it with its key to `config/clfMatrices.json`. A
controller call at an operating point without an entry is rejected with
`collisionAvoidanceController:missingClfMatrix`; nothing is synthesized at run
time, and the run time does not need the SDP solver.

At the trim, the sampled transverse model is e+ = A e + B (u - u_ref). With
S = inv(P) and a certificate gain Y = K S, the script solves by bisection on rho

    minimize rho
    subject to  [rho S, (A S + B Y)'; A S + B Y, S] >= 0,
                [ubar_j^2, Y_j; Y_j', S] >= 0       (j = steering, braking),
                R^2 D <= S <= R^2 shapeRatio D,    D = diag(scales.^2),

with YALMIP/SeDuMi from `solver/`. The first row is (A+BK)'P(A+BK) <= rho P.
The second limits the certificate input |K_j e| to
`clf.certificationSteeringRadians` = 0.075 rad and
`clf.certificationBrakingRatio` = 0.125 on the sublevel set V <= 1. These
equal one maximum trust-region step by default but are CLF settings of their
own, so P does not change with the optimizer's trust settings. The last makes that set contain every error within R =
`clf.certificationRegionScale` = 2 error scales, and keeps P's weights relative
to Q = inv(D) within `clf.shapeRatio` = 10 of each other. Without the input
bound the fastest P needs gains of about 30 (several radians of steering for a
0.5-m offset); without the shape bound the fastest P nearly ignores some error
directions (weights 1e-9 to 0.1 relative to Q). K is discarded.

The certified contraction rho* is the fastest attainable under these
conditions. The requested rho must be slower, otherwise synthesis rejects the
operating point. For the default vehicle the certified error time constants are
2.1 s at 8 m/s and 1.5--1.6 s at 15 m/s, against the requested 4 s.

Passing from this linear certificate to a nonlinear local CLF requires a
smooth error chart, an actual sampling equilibrium, and admissibility of a
neighborhood. The implementation uses RK4 and numerical trim calculation; the
tests verify nonlinear contraction for small signed perturbations, with the
input that minimizes the nonlinear successor value as the witness, not an
interval certificate for an entire set.

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
