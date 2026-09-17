# Curved-road cruise working point and CLF certificate

The working point is a steady turn of the same nonlinear bicycle used by the
trajectory predictor. At a fixed road curvature `kappa`, body longitudinal
speed `vStar`, and declared longitudinal bias `bStar`, solve for lateral
velocity `vyStar`, front steering `deltaStar` and signed force ratio `betaStar`.
The Frenet center offset is zero. Exact kinematics give

\[
 e_{\psi\star}=-\operatorname{atan2}(v_{y\star},v_\star),\qquad
 \dot s_\star=\sqrt{v_\star^2+v_{y\star}^2},\qquad
 r_\star=\kappa\dot s_\star.
\]

In particular, body longitudinal speed and path speed differ when sideslip is
nonzero. Setting `rStar = kappa*vStar` and `ePsiStar = -vyStar/vStar` gives
only the small-angle approximation. The remaining equations are

\[
\begin{aligned}
0&=(F_{xf}^{b}+F_{xr}-F_{\rm road})/m+v_{y\star}r_\star+b_\star,\\
0&=(F_{yf}^{b}+F_{yr})/m-v_\star r_\star,\\
0&=(l_f F_{yf}^{b}-l_r F_{yr})/I_z.
\end{aligned}
\]

The front forces are rotated by `deltaStar` from the wheel frame into the
body frame. Axle lateral forces use the nonlinear modified Fiala law; signed
longitudinal forces use the existing static axle-load scale. Air and rolling
resistance remain explicit. A requested turn with `abs(kappa)*vStar^2` greater than the total tire
acceleration capacity is rejected by a necessary lateral-force check. The
old frozen linear trim is used only as an
initial guess for a three-variable damped Newton solve. Its Jacobian follows
from the existing nonlinear operating-point Jacobians and the exact
kinematic relations above. The solve is bounded to 20 Newton iterations and
12 backtracking evaluations per iteration. Failure raises
`invalidCruiseOperatingPoint`; no approximate trim is silently substituted.
The zero-curvature force balance remains analytic.

At the resulting point, `continuousMatrices(..., operatingPoint)` and the
exact sampled exponential define the declared affine study plant. Constant
curvature uses a discrete Riccati certificate for the five transverse errors.
Both objective transcriptions penalize deviation from the force-balanced trim
input, without adding a feedback input target.

## Smooth spatial references

`referenceCurve.curvatureProfile` is an increasing array of `[station, curvature]`
rows, starting at zero and ending at `referenceCurve.length`. PCHIP interpolation
defines curvature as a function of arc length. The caller must explicitly declare
`continuation='constantCurvature'`. Integrating curvature gives heading; integrating
the unit tangent gives position. Cached polynomial integration includes analytic
truncation and arithmetic allowances. The constant-curvature endpoint continues
beyond the finite profile; this is a declared continuation, not an inferred road.

The certified local map is

    p_aff = c(s0) + d0*n(s0) + (1-kappa(s0)*d0)*t(s0)*(s-s0) + n(s0)*(d-d0),
    psi_aff = theta(s0) + kappa(s0)*(s-s0) + ePsi.

For station/lateral radii rs, rd and lateral extent D, interval bounds K0 and K1
on absolute curvature and its derivative give

    position remainder <= (K1*D + K0*(1+K0*D))*rs^2/2 + K0*rs*rd,
    heading remainder <= K1*rs^2/2.

Position integration allowances are added separately. Bernstein convex-hull
bounds enclose the curvature polynomials, including crossed knots and the
constant tail. All local domains used by collision or exit rows are enforced
on the complete uncertain held tube. Inverse projection retains the carried
station branch. Profile projection with nonzero Cartesian position uncertainty
is currently rejected: a certified inverse enclosure is not yet implemented.
This restriction does not alter the target high-gain observer.

## Immutable scheduled affine model

`ltvBicycleModel.referenceSchedule` prepares a finite sequence of 100 ms (or the
configured sample time) affine generators along the declared profile, followed
by one invariant constant-curvature tail. Reference station advances by
`h*hypot(vxTrim,vyTrim)`. The sequence is indexed by the declared sampling clock
`cfg.clf.referenceEpoch`; `referenceAt` returns an absolute phase. Replanning
does not relinearize an inherited witness or restart the clock at measured station.

For each phase, the constant-trim Jacobian includes the spatial curvature term
`A(3,1)=-sDotRef*kappaPrime`, with the affine offset adjusted at reference station.
The sampled reference defect

    d_j = F_j*xRef_j + G_j*uRef_j + g_j - xRef_(j+1)

is retained and charged in the terminal certificate. A sequence of steady trims
is not falsely treated as an exact solution of the changing-curvature dynamics.
This remains an explicit held affine plant, not a nonlinear flow inclusion claim.

Every finite hold must satisfy the hard Bernstein phase-domain constraint
`abs(s(t)-sRef(t)) <= cfg.encounter.referencePhaseRadius` (default 2 m), with
uncertainty and construction reserves. Both finite and terminal whole-hold rows
also enforce `abs(d) <= 0.8/Kmax`, where Kmax bounds the entire profile and
its declared tail, so `1-kappa(s)*d >= 0.2`. The bound is omitted for zero
curvature. These are model/chart validity constraints, not physical road edges.
The terminal continuation proves the same
condition for all subsequent holds. This prevents a delayed vehicle from using
a straight-tail model while arbitrarily far behind on a bend. It also restricts
admissible speed delays: arbitrary stopping or long yielding is not covered by
this implementation. Phase is not added to the online performance objective.

## Cross-stage soft CLF

Backward discrete Riccati recursion from the constant tail supplies five-dimensional
P_j and K_j, normalized by one common factor. Each phase verifies

    (F_e,j-G_e,j*K_j)' * P_(j+1) * (F_e,j-G_e,j*K_j) < P_j.

The online norm cone evaluates the complete affine successor against the next
reference and P_(j+1), including the known station coupling. Its right side uses
the current five-error norm and P_j. The nonnegative CLF slack remains squared in
the objective. Both condensed and lifted objectives use phase-specific trim
states, inputs and matrices. Station is a certificate coordinate, not an added
tracking penalty. Positive slack does not certify asymptotic convergence while
curvature is changing; final constant-tail recovery is assessed separately.

## Six-dimensional terminal continuation

The terminal feedback uses full error `e_j=x-xRef_j`, including phase. This law
is a prediction witness only; failed optimization still issues no command.
Let W be a common invertible modal transformation, V its inverse, and define

    E_j = {e : abs(W*e) <= a_j},
    Fc_j = F_j-G_j*K6_j,
    C_j >= abs(W*Fc_j*V).

With future measurement radius cap r and exact reference defect d_j, verified
linear synthesis enforces

    C_j*a_j + abs(W*d_j) + abs(W*G_j*K6_j)*r <= a_(j+1) - reserve.

Input support, cross-phase input slew, conjugate-mode radius equality and
whole-hold phase support are imposed jointly. The final phase maps to itself,
with reference station translated at the constant trim path speed. Every
finite stage and the tail are checked; sampled curvature values are not used
as a proof for an undeclared continuum of models.

For held sampled feedback, the within-hold map is `Phi(t)-Gamma(t)*K6_j`.
It is not `exp((A-B*K6_j)*t)`. Bernstein coefficients, truncation and arithmetic
reserves certify the complete phase interval. Small optimized deviations from
the terminal witness also retain these bounds and the next modal set.

The finite witness shifts its original generators, enclosures, hard phase rows,
terminal endpoint and absolute exit deadline. A fresh family replaces it only
when independently verified. Thus the existing shifted-witness induction extends
to this scheduled family under the same execution/measurement premises. See
[the recursive proof](TERMINAL_CBF_PROOF.md).

## Validation and limits

Run `tests/smoothReferenceGeometryTest.m`,
`tests/scheduledReferenceControllerTest.m`, and
`scripts/runSmoothReferenceControllerValidation.m`. The driver separates cold
reference/terminal preparation from complete frame timing, and records actual
spatial curvature, scheduled curvature, phase error, hard certificates, physical
sampled clearance and final trim error. It covers S bends, a smooth transition
and an asymmetric bend, without physical road boundaries.

The supported class is explicit smooth curvature profiles whose trim, terminal
synthesis, local charts and finite encounter search pass their checks. This is
not a guarantee for every plausible road, arbitrary speed delays, nonzero ego
position uncertainty, a nonlinear physical vehicle, or hardware worst-case
execution time. Route preparation and periodic 100 ms qualification are distinct.

Reference-dependent terminal ingredients motivate the cross-stage structure:
[Koehler et al., 2020](https://arxiv.org/html/1909.12765v2). The mixed Frenet tracking
and Cartesian collision representation is also studied by
[Reiter et al.](https://arxiv.org/html/2212.13115v1). Neither source proves this
implementation's sampled affine inclusion; that is checked explicitly above.
