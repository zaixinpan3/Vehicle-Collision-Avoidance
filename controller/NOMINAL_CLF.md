# One nominal CLF for avoidance and return to cruise

The controller uses one target-independent value at every sample. Target position,
encounter range, and collision constraints do not choose its formula. The two
optimization stages remain PCBF slack first and CLF slack second. An input-increment
regularizer in the second solve resolves otherwise arbitrary future plans, within
an explicit tolerance. There is no third solve or feedback command substitution.

## Construction and the precise decrease argument

Let `e = [e_y, e_psi, v_x-v_x*, v_y-v_y*, r-r*]`. Longitudinal path phase is free.
Let `z` include the ego state and previous input when braking slew is finite,
`F_h(z,u)` denote the held-input RK4 model, and `ell(z) = e' Q e`, with the scales
in `cfg.clf`. The construction feedback `kappa = nonlinearBicycleModel.nominalFeedback`
uses path-course guidance, a friction-limited yaw-rate demand, inverse Fiala force,
and speed feedback. It is independent of the target. Its role is to define a
nominal return cost and the non-avoidance portion of a search initialization.
All issued commands still come from the two-stage optimization.

At the cruise trim, `nominalTail` computes the closed-loop Jacobian `A_kappa`
and solves

    A_kappa' P_f A_kappa - P_f = -2 Q.

The factor 2 leaves a strict reserve for the nonlinear remainder. Local stability
and smoothness imply that a sufficiently small invariant neighborhood exists on
which the nonlinear tail satisfies

    V_f(F_kappa(z)) - V_f(z) <= -ell(z),    V_f(e) = e' P_f e.

The implementation does not claim an interval proof for the entire configured
`tailLevel` ellipsoid. Signed coordinate tests check the nonlinear inequality;
`clfTailReached` is diagnostic and is never an execution gate.

For a **fixed** number K of policy-evaluation holds, define

    V_K(z) = sum_{j=0}^{K-1} ell(F_kappa^j(z)) + V_f(F_kappa^K(z)).

The default K is 2400 at 50 ms. It is the same for all states, with no stopping-time
or encounter-dependent change of function. It adds no optimization variables and
sets no arrival deadline on the optimized avoidance trajectory. The function is
computed on a canonical straight/circular path pose, removing irrelevant absolute
translation and rotation.

On the domain where the nominal rollout is evaluable and its Kth state enters the
invariant neighborhood,

    V_K(F_kappa(z)) - V_K(z)
      = -ell(z) + ell(z_K) + V_f(F_kappa(z_K)) - V_f(z_K)
      <= -ell(z).

Thus an input producing a decrease exists in this domain. This is a constructive
regional CLF argument, not an assumption that the LQR quadratic is valid far from
cruise. Nonlinear convergence of the construction feedback outside the local
neighborhood is tested, not globally proved. Saturation, wrapped-angle charts,
the positive-speed tire domain, and finite input-memory limits restrict the claim.
Existence of a nominal descent input also does not prove compatibility with the
MPC trust box, state limits, or safe completion constraints. Positive optimized
slack can reflect these restrictions as well as an actual collision threat.

## Online constraint and RTI approximation

Every frame uses the same desired inequality

    V_K(F_h(z,u_0)) - V_K(z) <= -eta * ell(z) + rho,   rho >= 0.

Here `eta = cfg.nominalClf.decreaseFraction`. The construction feedback does not
replace the optimized first input when the slack is zero. Nor does a target
leaving range activate another CLF.

Only the two first-input components enter `Phi(u_0) = V_K(F_h(z,u_0))` because the
current measured state is fixed. `localNominalClf` differentiates this complete
composition, including the nonlinear hold, using a centered two-input stencil.
It retains the nonnegative Gauss-Newton residual model and adds the positive
semidefinite part of the residual-curvature contribution to the full 2-by-2
Hessian: H = H_GN + (H_full - H_GN)_+. This captures positive curvature omitted
by Gauss-Newton while preserving a nonnegative quadratic value. Projecting only
the full Hessian can instead produce a negative quadratic minimum. A compiled copy of
`nominalResidual` accelerates the nominal policy evaluations; the MATLAB source
remains the algorithm definition.

**The convex residual model is a local RTI approximation, not a certified upper bound
on the nonlinear value throughout the trust box.** Consequently zero modeled slack
is not, by itself, a theorem of nonlinear or continuous-plant decrease. Independent
ODE45 replay and evaluation of the same V on consecutive measured states quantify
that gap. Within-hold relinearization now measures nonlinear prediction and
first-successor value agreement; it is not a complete nonlinear safety check.
An accurate result with positive CLF slack at the first-input trust boundary
also continues the same two-stage iteration while its predicted or actual
slack reduction exceeds the existing scaled CLF tie resolution. The previous
between-frame trust-scale adaptation remains in use. Thus a numerical correction box
does not become an unintended permanent restriction on nominal recovery.
The best accurate complete pair from the current frame is retained.
An exact inequality
with zero slack implies asymptotic dissipation on an appropriate invariant domain;
fixed numerical errors generally support practical convergence only.

## Why the second solve needs a well-defined future plan

An objective involving only the first-step CLF leaves many future control
sequences equivalent. Their arbitrary second inputs become the next frame's trust
centers. In the unregularized crossing diagnostic at 10 s, the shifted steering
was -0.072452 rad, so the +0.075-rad correction bound allowed only +0.002548 rad.
The optimized next steering was again -0.072487 rad. This repeated even though a
larger positive steering could recover the path. Removing state trust boxes did
not repair that frame; expanding the input correction box reduced its CLF slack
from approximately 45916 to 0.003. This is a reference-propagation defect, not a
physical steering limit.

Use the dimensionless squared input increment

    R(dU) = (1/(2M)) * sum_i ||diag(r_delta,r_b)^(-1) dU_i||^2.

The existing componentwise trust limits give `0 <= R <= 1`. The former conic
epigraph `sigma >= R`, `0 <= sigma <= 1` is redundant and is eliminated exactly.
The compiled conic QP directly minimizes

    rho_scaled + epsilon * R(dU),

with the PCBF optimum retained by its original bound. Inputs are penalized relative
to their linearization values, **not relative to zero and not relative to kappa**.
Strict convexity in the input increment removes arbitrary optimal future plans.
For an exact conic optimum, comparison with an unregularized CLF minimizer gives

    0 <= rho_scaled,regularized - rho_scaled,min <= epsilon.

The default `epsilon = cfg.solver.clfTieTolerance = 1e-4` explicitly bounds this
secondary tradeoff; `clfTieBound` reports its unscaled value. It is not an exact
third lexicographic objective hidden inside the solver. PCBF remains the first
priority, while CLF minimization is resolved within this stated tolerance.
Solver termination errors are additional and are recorded separately. In stage
one, the unbounded CLF slack is eliminated algebraically with its cone.
A fully feasible zero-slack anchor proves a zero primary optimum without a
numerical solve; the secondary always remains. This preserves the PCBF feasible
projection and avoids paying for the secondary objective in the primary solve.

The dynamics/collision linearization still follows the shifted prior inputs,
rolled out from the measured state. An unusable rollout triggers the existing
single fresh flow initialization. The primary solve also distinguishes a finite
relaxed solution from a zero-slack prediction. Its unavoidable current-state lower
bound is `max(0,-min(g(x_current)))`, computed from the already assembled safety
rows. If the shifted primary optimum exceeds that bound, or has no numerical
point, one fresh flow model is formulated. The lower primary objective is retained;
ties retain the shift. The CLF is constructed and solved only after this selection.

If the fresh primary problem returns no point with an infeasibility exit,
its input correction box may be
enlarged once by a factor of two. This is a nested primary problem; state trust,
physical braking bounds, safety rows, and endpoint constraints are unchanged.
The better primary result is retained. An infeasible numerical correction box
does not justify repeatedly shrinking it. Normal frames use one or two numerical solves
for their two objectives, depending on the analytic zero-primary certificate;
fresh initialization and this bounded enlargement can add two primary solves.
These are counts for one local model; within-hold refinement can rebuild
that model and repeat both stages. The accuracy and stopping rules are in
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md).
There is no distance-based recovery anchor, separate recovery controller,
or nominal-control fallback.

The default optimization-node collision buffer is 0.10 m. The replay success
criterion remains positive actual rectangle clearance, not a 0.10-m physical
clearance requirement. Starts and midpoints do not certify every continuous
instant, and a finite positive-slack solver result remains executable under the
declared controller interface. These are material limitations of the safety claim.

## Literature and verification scope

The cost-to-go and terminal-decrease reasoning is consistent with Rawlings,
Mayne and Diehl, *Model Predictive Control: Theory, Computation, and Design*,
2nd edition, 6th printing (2026), Assumption 2.14 and Appendix B.5
([author-hosted text](https://sites.engineering.ucsb.edu/~jbraw/mpc/MPC-book-2nd-edition-6th-printing.pdf)).
The use of a local CLF to close a finite approximation is also discussed by
Jadbabaie, Yu and Hauser, *Unconstrained receding-horizon control of nonlinear
systems*, IEEE TAC 46(5), 776-783 (2001), DOI 10.1109/9.920800
([author repository](https://authors.library.caltech.edu/records/2976d-mh372)).
These sources support general design principles, not a global certificate for
this particular Fiala vehicle or a measured execution deadline.
