# One analytic nominal CLF

The active controller uses one target-independent function throughout avoidance
and recovery. There is no encounter-dependent CLF, nominal-control substitution,
or executable backup. Every issued command comes from a completed CLF solve or
an accepted feasible-segment damping of that solution.

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
constraint represented by one second-order cone. The free-pose terminal core
also retains its cone: the optimization is a sparse conic QP/SOCP, not a QP
with exclusively affine constraints.

The nonlinear agreement calculation evaluates this same V at the true RK4
first successor. It requires one trajectory rollout and an analytic value
calculation, without nominal return simulations or new derivatives.

The objective is

    rho / max(1,V(x)) + epsilon * R(dU),
    R(dU) = (1/(2M)) sum_i ||diag(r_delta,r_b)^(-1) dU_i||^2.

The input correction box implies 0 <= R <= 1. Therefore, at an exact optimum,
the regularizer changes minimum scaled CLF slack by at most
`epsilon = clfTieTolerance = 1e-4`. Inputs are penalized relative to the
linearization input, not zero. This also prevents arbitrary future inputs
from becoming poor trust centers after shifting. Solver tolerances add their
own numerical error.

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

The PCBF budget is a hard constraint of this convex problem. It comes from
shifted stage slacks when available, or a primary slack minimization during
initialization/restoration. A zero primary value never skips CLF optimization.

## Damping and initialization

A full CLF optimizer point may have excessive prediction error. If a primary
point (or the zero-correction inherited point) is feasible in the exact same
assembled convex problem, the controller can interpolate all its variables
with the CLF solution. It tightens rho by evaluating the existing quadratic at
the interpolated point. It tries at most three nonlinear rollouts without
another solve. A full zero-slack solution must remain zero-slack within tolerance
when damped. Positive optimal slack may increase after damping; the metadata
reports the issued slack and `secondaryOptimumApplied=false`.

An inherited safety budget alone does not prove that the zero-correction point
satisfies the terminal cone. Damping is unavailable when that point is infeasible.
There is one trajectory model per initialization. A failed shifted model may
request one fresh potential-field initialization and its model; a fresh model
is never repeatedly relinearized in that hold. Restoring
a failed inherited budget or enlarging an input correction box reuses the
already built model. See [PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md).

`nominalFeedback` and `nominalGuidanceParameters` supply path guidance and a
trim steering correction only for constructing the flow seed. They neither
define another CLF nor provide an issued control. The nominalClf configuration
group retains their guidance parameters alongside the scalar decreaseFraction;
retired cost-to-go horizon and tail-level settings are rejected.
