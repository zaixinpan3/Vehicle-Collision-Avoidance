# One analytic nominal CLF

The active controller uses one target-independent function throughout avoidance
and recovery. There is no encounter-dependent CLF, nominal-control substitution,
or executable backup. Every issued command comes from a completed CLF solve.

## Function and its scope

For the given path and desired cruise trim, define

    e(x) = [e_y; e_psi; vx-vx_ref; vy-vy_ref; r-r_ref],
    V(x) = e(x)' P e(x),       P > 0.

Longitudinal path phase is free. `nonlinearBicycleModel.cruise` computes the trim
and the discrete LQR matrix P from the transverse local dynamics, the error
scales in `cfg.clf`, and the two input weights. `nominalValue` evaluates this
quadratic directly. No nominal-policy rollout, finite-difference Hessian,
cost-to-go kernel, or additional terminal value is used to construct the CLF.

Let Q be the diagonal inverse squared error scales. The requested decrease is

    V(x_next) - V(x) <= -eta * e(x)' Q e(x) + rho,   rho >= 0,

where `eta = nominalClf.decreaseFraction = 0.5`. This is deliberately not an
arbitrary fixed fraction of V. For the exact sampled local linear system and
its LQR feedback K, the Riccati identity gives

    (A+B K)' P (A+B K) - P = -(Q + K' R K).

Thus eta below one leaves a strict local decrease reserve. Passing from this
identity to a nonlinear local CLF requires a smooth error chart, an actual
sampling equilibrium, and admissibility of a neighborhood. The implementation
uses RK4 and numerical trim calculation; the tests verify nonlinear decrease
for small signed perturbations, not an interval certificate for an entire set.

**This quadratic is not a global nonlinear CLF.** Large heading errors, tire
saturation, state-domain boundaries, braking memory and the MPC trust region
can make zero slack unavailable even without a target. For example, at a
20-m lateral displacement the sampled tests found positive slack despite an
actual reduction in V. The former cost-to-go argument covered a different,
feedback-reachable region; it cannot be transferred to this simpler function.
The scenario campaign and displaced-state tests therefore measure eventual
cruise recovery separately from local zero-slack decrease.

If the exact nonlinear inequality holds with rho=0 on a suitable invariant
domain, summing it gives dissipation of the transverse error. Zero slack in the
**affine approximation**, together with a fixed model-error tolerance, does not
establish that exact inequality or global asymptotic convergence.

## One convex first-successor constraint

The trajectory is linearized once around a nonlinear rollout. At its first
successor, use the analytic Frenet error Jacobian J:

    e_aff = e(xbar_1) + J(xbar_1) * dx_1,
    V_aff = ||chol(P) * e_aff||^2.

The same affine dynamics relate dx_1 to the first input correction. Its Hessian
is positive semidefinite, so the first-step CLF epigraph is a convex quadratic
constraint represented by one second-order cone. The path corridor also uses
cones: the optimization is a sparse conic QP/SOCP, not a QP with exclusively
affine constraints.

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

With current and successor generators `G0,G1`, the same CLF uses

    b0 = sum_j ||chol(P)*J0*G0(:,j)||,
    b1 = sum_j ||chol(P)*J1*G1(:,j)||,
    bQ = sum_j ||sqrt(Q)*J0*G0(:,j)||,
    budget = max(0,sqrt(V0)-b0)^2 - eta*(||sqrt(Q)*e0||+bQ)^2,
    lossLower = max(0,||sqrt(Q)*e0||-bQ)^2.

The successor upper bound is `(1+w)*V_aff+(1+1/w)*b1^2`, using Young's
inequality with positive fixed `w` when `b1>0`; zero uncertainty recovers
`V_aff` exactly. It is constrained by `budget+rho` with the existing single
SOC. This is a sufficient robust inequality for the affine error model,
not another CLF. `clfWorstNextValue` and `clfCurrentBudget` record the separate bounds.

The remaining nonlinear error-chart and dynamics remainders are not enclosed.
Persistent observation uncertainty can require positive slack near the path;
neither zero affine slack nor this implementation proves exact asymptotic
recovery of the nonlinear uncertain plant. See the conditional theorem in
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
There is one trajectory model per frame: a potential-field rollout at startup,
otherwise the shifted previous plan. A frame that cannot be solved reports no
solution; no other seed, retry or enlarged box follows. See
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md).

`nominalFeedback` and `nominalGuidanceParameters` supply path guidance and a
trim steering correction only for constructing the potential-field seed. They neither
define another CLF nor provide an issued control. The nominalClf configuration
group retains their guidance parameters alongside the scalar decreaseFraction;
retired cost-to-go horizon and tail-level settings are rejected.
