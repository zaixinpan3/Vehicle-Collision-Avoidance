# A bounded-error tire prediction design — September 25, 2026

Status: mathematical design proposal and scalar verification, not an adopted
controller change or a successful avoidance experiment. The design responds
to the unrestricted joint-tangent extrapolation identified in the
[remaining-solve audit](SOLVE_FAILURE_AUDIT_20260925.md). It respects the
current preference for one dynamics linearization per frame: a proposed
bounded-error subproblem can remain convex without an inner relinearization
loop. This does not guarantee that a feasible subproblem exists.

## Recommendation and scope

Investigate **angle coordinates for longitudinal utilization together with
certified local approximation domains and explicit nonlinear-remainder
tubes**. These are complementary: coordinates improve conditioning, domains
make error bounds applicable, and tubes account for the remaining error in
constraints. A different variable alone cannot make the nonlinear force
graph globally affine. Adding a soft proximity cost cannot enforce a domain.

The priority is a proof and fixed-frame feasibility study before changing
the controller. No new force constraint, trust-region row, terminal policy,
inner iteration or nonlinear admission mechanism has been implemented here.
The S-curve station-schedule conflict remains a separate design question.

## 1. A coordinate change removes the saturated beta derivative singularity

Let M = mu Fz, sigma = sign(alpha), and hold the saturated slip branch fixed.
The current implemented force is

    F(beta) = -sigma M sqrt(1-beta^2).
    F'(beta) = sigma M beta / sqrt(1-beta^2).
    F''(beta) = sigma M / (1-beta^2)^(3/2).

Both derivatives grow without bound near abs(beta)=1. This explains why even
a modest beta change can defeat a local tangent. The smooth coordinate map

    beta = sin(theta),  -pi/2 <= theta <= pi/2

represents exactly the same utilization range, with

    F(theta) = -sigma M cos(theta),
    dF/dtheta = sigma M sin(theta),
    d2F/dtheta2 = sigma M cos(theta).

Therefore the fixed-branch first-order remainder obeys the global bound

    abs(F(theta)-F(thetaBar)-F'(thetaBar)*(theta-thetaBar))
        <= (M/2)*(theta-thetaBar)^2.

This follows directly from the integral Taylor remainder and
abs(F''(theta)) <= M. It is an exact coordinate derivation, not a replacement
tire law or a novelty claim. For a force-error budget epsilon, the sufficient
angle radius sqrt(2 epsilon / M) follows from the bound instead of an
arbitrary trust-radius choice. The longitudinal force M sin(theta) has the
same M/2 quadratic remainder bound.

Using the straight 2.00 s plan, first hold/front axle, M=6505.273284 N,
betaBar=0.999929413489141 and beta=0.923738750255249:

| Fixed positive saturated-slip branch calculation | Force or bound |
| --- | ---: |
| Exact nonlinear force at candidate beta | -2491.670 N |
| Original beta tangent | -41790.006 N |
| Angle-coordinate tangent | -2556.830 N |
| Angle-tangent absolute error | 65.160 N |
| Proven M/2 quadratic upper bound for that step | 472.615 N |

The original force error is about 39.298 kN. This calculation deliberately
keeps slip sign fixed to isolate the beta singularity. In the actual stored
plan steering also changes sign, and the actual nonlinear force is
**positive** 2491.670 N. The table must not be presented as correcting that
whole stored plan. The fixed-branch bound is inapplicable if the trajectory
leaves the branch and it is used without a corresponding full-model bound.

Twenty-five thousand nine hundred twenty-one deterministic angle pairs on
[-pi/2,pi/2]^2 check the scalar inequality numerically. All pass; the largest
error/bound ratio is 0.9999679. The analytic argument supplies the bound;
finite sampling only checks its numerical evaluation. No random seed applies.

### Important limits of this coordinate change

The full Fiala law includes adhesion/saturation transitions and slip reversal.
The fixed-sign formula is not a global description of that law. Bounds for
the full Jacobian must cover every relevant branch, or use a domain with
explicitly established regularity. In particular, the corner where
theta approaches +/-pi/2 while alpha approaches zero cannot be declared
globally smooth from the one-dimensional calculation. Tire slip-domain,
speed-floor and Frenet-chart regularity conditions also remain.

Computing an infinite beta derivative and multiplying it by cos(theta) is
not a numerically valid endpoint implementation. Transformed derivatives
need direct branch formulas. The physical input remains beta=sin(theta).
Longitudinal dynamics, input limits, rate limits, feedback gains and terminal
contracts all need consistent treatment. Bounds on beta map monotonically
to angle intervals; beta slew constraints between two decision variables
do not become exactly linear. A sufficient angle-slew bound follows from
abs(sin(a)-sin(b)) <= abs(a-b), with possible conservatism.

## 2. Certify the approximation error over an enforced domain

For the sampled nonlinear state transition Phi_h(x,u), let z collect scaled
state and transformed-input coordinates. Use the actual sampled-map tangent

    Phi_h(z) = Phi_h(zBar) + J(zBar)*(z-zBar) + R(z).

If component j of the Jacobian is L_j-Lipschitz throughout a convex scaled
domain containing the segment between zBar and z, then

    abs(R_j(z)) <= (L_j/2)*norm(z-zBar,2)^2.

Hessian norm bounds suffice on smooth branches. Piecewise models require
verified cross-branch Jacobian regularity or separate enclosures; a finite
grid maximum is not a certified Lipschitz constant. Scaling is essential
because metres, radians, speeds and utilization cannot share an unqualified
Euclidean radius.

For a fixed domain radius rho, use w_j = L_j*rho^2/2 and enforce the domain in
the optimization. A box-error tube with a fixed stage feedback K has the
schematic recursion

    rNext >= abs(A+B*K)*r + wNonlinear + wExternal.

Measurement injection, integration error and arithmetic reserves must be
included consistently in wExternal. This is a discrete state-transition
remainder, in state units per hold; a force error in newtons cannot simply
be added to the position tube. Converting tire-force bounds through the
continuous flow is an alternative only with a justified reachability bound.

The domain must contain the *actual tube*, not just the optimizer's nominal
point. For actual z = zNominal + [e; K e], a sufficient condition is

    norm(D*(zNominal-zBar),2)
      + sup_{e in E} norm(D*[e;K*e],2) <= rho.

D applies the declared physical scaling. A conservative box support bound
can make the second term affine in the nonnegative tube radii. With fixed
rho, L_j, K and matrices, these local norm-domain constraints and linear
radius propagation can be represented by SOCP constraints. A quadratic
error-budget epigraph also has a rotated-SOC representation, although its
coupling to the full tube must be derived rather than assumed away.

For linear safety rows Hx <= b, enforce H*xNominal + abs(H)*r <= b. Nonlinear
road/footprint maps need their own valid remainder enclosures. Bounds for the
nonlinear plant mismatch are additional to bounds for this bicycle model.

### The current discretization also needs an explicit account

`ltvBicycleModel.trajectoryStages` currently exponentiates a continuous
Jacobian frozen at the start of each hold. This exactly discretizes that
frozen affine model, but is not generally the Jacobian of the nonlinear
held-input flow. The sampled-map Taylor formula above therefore cannot be
attached to those matrices without further work.

One coherent implementation would integrate the nominal held-input flow and
its variational equations, obtaining A and B for that discrete flow and
c = Phi_h(xBar,uBar)-A*xBar-B*uBar. This remains one preparation/linearization
pass per frame. Alternatively, retain the existing matrices and explicitly
bound their nominal defect, Jacobian discrepancy and integration errors in
addition to the higher-order remainder. No discrete-flow certificate is
claimed to have been computed in this task.

## 3. A useful alternative: force variables and an exact convex capacity set

For each axle, the combined-slip envelope is exactly the second-order cone

    norm([beta; Fy/M],2) <= 1.

This follows from Fx=M*beta and Fx^2+Fy^2 <= M^2. It requires no linearization
of sqrt(1-beta^2). It can rule out the observed 27--42 kN predictions under
the current loads, but envelope membership does not establish the force
actually produced by a given steering angle and state.

With front lateral force as a virtual input, an interior Fiala inverse exists.
For L=M*sqrt(1-beta^2)>0 and abs(Fy)<L, the adhesion branch gives

    alpha = -sign(Fy)*atan((3*L/C)*(1-(1-abs(Fy)/L)^(1/3))),
    delta = atan2(vy+lf*r,vx) - alpha.

The inverse follows by writing abs(Fy)/L = 1-(1-q)^3. Its conditioning
deteriorates at the saturation boundary. Steering amplitude/rate limits
must be enforced through this mapping. Rear force is not an independent
actuator on the current front-steered car, and large-angle force rotation
must remain consistent. Thus this route is a larger architectural change;
the SOC alone does not make the full vehicle optimization exactly convex.

## Sources, interpretation and validation boundary

- Zhang, Thornton and Gerdes, *Tire Modeling to Enable Model Predictive
  Control of Automated Vehicles From Standstill to the Limits of Handling*,
  AVEC 2018: Section 2, Eqs. (4)--(6), and Section 4 distinguish the nonlinear
  Fiala law, friction capacity and affine approximation. Their convex lateral
  formulation treats longitudinal information as supplied, unlike this
  project's simultaneous beta optimization. [Author-hosted paper](https://ddl.stanford.edu/sites/g/files/sbiybj25996/files/media/file/zhang_2018_avec_0.pdf).
- Stephen M. Erlien, *Shared Vehicle Control Using Safe Driving Envelopes for
  Obstacle Avoidance and Stability*, 2015: Chapter 5, Eqs. (5.15)--(5.17),
  uses force optimization, a friction-circle constraint and inverse coupled
  tire mapping to steering. This supports the alternative's architecture,
  not an exact-convexity claim for the current six-state model.
  [Author-hosted thesis](https://ddl.stanford.edu/sites/g/files/sbiybj25996/files/media/file/thesis_erlien_0.pdf).
- Lishkova and Cannon, *A successive convexification approach for robust
  receding horizon control*, arXiv:2302.07744v3, January 27, 2025: Introduction
  and Sections II/V develop nonlinear trajectory tubes that include
  approximation error and discuss conservative error-bound difficulties.
  Their DC-based method is not the exact construction proposed above; its
  recursive-feasibility/stability theorems do not automatically apply here.
  [Primary preprint](https://arxiv.org/pdf/2302.07744v3).

These sources were checked directly. Local Zotero author searches for
Lishkova and Erlien returned no exact matches; unrelated semantic fallback
items were excluded. No library items or notes were modified. This is a
targeted design brief, not a systematic literature review. The angle
transformation, scalar error bound and mapping to the present code are the
analysis in this record, without claiming them as novel literature results.

The numerical output is in `TIRE_PREDICTION_DESIGN_20260925/coordinate-check.json`.
Controller code, controller tests and closed-loop simulations were not changed
or rerun. No avoidance success, nonlinear safety certificate or restored
recursive feasibility has been established. Obtaining valid full-model error
bounds, an admissible nonlinear seed and a compatible terminal guarantee are
remaining implementation tasks. A local one-pass method can reject a frame
when its neighborhood contains no feasible maneuver; bounded error does not
create missing control authority or resolve the S-curve progress conflict.
