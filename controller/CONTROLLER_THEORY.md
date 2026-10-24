# Controller mathematical theory

This is the current controller theory and source map. It consolidates the six
former controller Markdown documents into one derivation, with a single
statement of the implementation boundary. Repository paths are relative to the
repository root unless linked otherwise. Historical experiment reports describe
the versions that produced them; they are not rewritten by this consolidation.

The controller seeks rectangle nonoverlap while returning to a prescribed path
and cruise speed. It uses one target-independent quadratic CLF and two
lexicographic optimization stages: minimize predictive collision deficits, then
optimize CLF relaxation and deviation from the current trajectory reference.
The terminal set certifies an available safe continuation at the prediction
endpoint; it is not a command to park or a task-completion condition.

There are two mathematical objects whose results must be distinguished:

| Object | Prediction and conclusion |
| --- | --- |
| Executable controller | One local RTI model per reference, per-hold affine uncertainty tightening, a first-successor CLF row and linearized terminal conditions; a completed solve supplies the unchanged first input. |
| Conditional nonlinear output-feedback design | Joint physical/observer information sets, persistent model uncertainty, causal future feedback, sound reachable images and a common terminal continuation; the shifted-policy proof establishes mathematical recursive feasibility under its stated premises. |

The latter theorem is not a certificate for the former approximation.
`robustNonlinearSafetyCertified` remains false. Positive collision slack permits
a clearance deficit. Zero affine slack does not certify nonlinear or
continuous-time safety. Road constraints are disabled by default; the ideal
joint proof below also omits road boundaries. Optional road rows require their
own compatible images and terminal checks.

## Contents

1. [Vehicle model and target prediction](#vehicle-model)
2. [Observer bounds and relative geometry](#observer-geometry)
3. [Collision constraints and the two-stage problem](#optimization)
4. [One nominal CLF and recovery](#nominal-clf)
5. [Joint terminal set and braking continuation](#terminal-set)
6. [Joint feedback-aware prediction](#joint-prediction)
7. [Conditional recursive-feasibility proof](#recursive-feasibility)
8. [RTI references, computation and implementation boundary](#runtime)
9. [Source map and verification](#source-map)
10. [Sources and document migration](#sources)

<a id="vehicle-model"></a>
## 1. Vehicle model and target prediction

### Coordinates, inputs and tire forces

Use the physical ego state and held actuator input

$$
x=(p_x,p_y,\psi,v_x,v_y,r)^\top,\qquad u=(\delta,b)^\top,
\qquad t_k=kh.
$$

Positions are in meters, angles in radians, speeds in m/s, yaw rate in rad/s
and forces in N. The signed longitudinal-force ratio satisfies $-1\le b\le1$.
The default control hold is $h=0.05$ s. With wheelbase $L=l_f+l_r$, the static
axle loads and the force capacity remaining for lateral motion are

$$
F_{zf}=mg\frac{l_r}{L},\quad F_{zr}=mg\frac{l_f}{L},\quad
F_{xi}=b\mu_i F_{zi},\quad D_i=\mu_iF_{zi}\sqrt{1-b^2}.
$$

The tire slip angles use the code's force sign convention:

$$
\alpha_f=\operatorname{atan2}(v_y+l_fr,v_x)-\delta,\qquad
\alpha_r=\operatorname{atan2}(v_y-l_rr,v_x).
$$

For $D_i>0$, let $q_i=C_i|\tan\alpha_i|/(3D_i)$. The modified Fiala force is

$$
F_{yi}=\begin{cases}
-C_i\tan\alpha_i(1-q_i+q_i^2/3),&q_i<1,\\
-D_i\operatorname{sign}(\alpha_i),&q_i\ge1.
\end{cases}
$$

When $D_i=0$, lateral force is zero. The force remains defined at $|b|=1$,
but its joint input tangent is singular there. Only linearization references
are moved ten constraint tolerances inside these endpoints; physical actuator
bounds and already issued commands retain the full interval. Tire saturation
is part of the model. No extra rear-tire nonsaturation cone is imposed by the
optimizer or terminal membership test.

### Dynamic and low-speed equations

Rotate the front axle forces into the body frame:

$$
F_{xf}^B=F_{xf}\cos\delta-F_{yf}\sin\delta,\qquad
F_{yf}^B=F_{xf}\sin\delta+F_{yf}\cos\delta.
$$

Above the low-speed blend, the bicycle equations are

$$
\begin{aligned}
\dot p&=R(\psi)[v_x,v_y]^\top,&\dot\psi&=r,\\
\dot v_x&=(F_{xf}^B+F_{xr}-F_{\rm load}(v_x))/m+v_yr,\\
\dot v_y&=(F_{yf}^B+F_{yr})/m-v_xr,\\
\dot r&=(l_fF_{yf}^B-l_rF_{yr})/I_z.
\end{aligned}
$$

The passive road load is aerodynamic drag plus smooth rolling resistance:

$$
F_{\rm load}(v)=\tfrac12\rho_a C_d A_f v|v|
+mg(c_0+c_1|v|+c_4|v|^4)\tanh(v/v_\epsilon).
$$

At low speed define $r_{\rm kin}=v_x\tan\delta/L$,
$F_{x,\rm kin}=(\max(b,0)+\min(b,0)\tanh(v_x/v_b))
(\mu_fF_{zf}\cos\delta+\mu_rF_{zr})$. The low-speed body map is

$$
\dot v_x=(F_{x,\rm kin}-F_{\rm load}(v_x))/m+v_yr,\quad
\dot v_y=(l_rr_{\rm kin}-v_y)/\tau_\ell,\quad
\dot r=(r_{\rm kin}-r)/\tau_\ell.
$$

Blend the three body derivatives using $w=s^2(3-2s)$ and
$s=\operatorname{clip}((v_x-v_\ell)/(v_h-v_\ell),0,1)$, with default
$[v_\ell,v_h]=[2.5,4]$ m/s and $\tau_\ell=0.1$ s. The position/yaw equations
remain the same. The sampled map $F_h(x,u)$ is RK4 with variational tangents.
Under nonpositive $b$, a sample-end speed below $0.2v_b$ resets
$(v_x,v_y,r)$ to zero; default $v_b=0.25$ m/s. This numerical reset differs
from the exact within-hold stop used by the terminal model in Section 5.
A proof for a continuous physical plant must also enclose integration and
reset discrepancies.

On a curved path with phase $s_p$, lateral displacement $e_y$, heading error
$e_\psi$ and curvature $\kappa_p$, the path chart satisfies

$$
\dot s_p=\frac{v_x\cos e_\psi-v_y\sin e_\psi}{1-\kappa_pe_y},\quad
\dot e_y=v_x\sin e_\psi+v_y\cos e_\psi,\quad
\dot e_\psi=r-\kappa_p\dot s_p.
$$

The chart requires $1-\kappa_pe_y>0$. Longitudinal phase is free in the CLF.

### Target flow and encounter memory

The target has constant tangential acceleration $A_C$ and sideslip $\beta_C$
within one forecast. With $\kappa_C=\sin\beta_C/l_{rC}$, its positive-speed flow is

$$
\dot p_C=V_C[\cos(\psi_C+\beta_C),\sin(\psi_C+\beta_C)]^\top,\quad
\dot\psi_C=\kappa_CV_C,\quad \dot V_C=A_C.
$$

For $A_C<0$, integrate only until $V_C=0$, then retain the stopped pose.
The published acceleration state becomes zero at rest. Do not extrapolate
negative speed or intersect this reset state as if it were an unchanged latent
pre-stop parameter. Each actual observation refreshes the forecast center and
parameters. The conditional theory instead retains every admitted parameter
history consistently through propagation and posterior conditioning.

An encounter ends at its first exit from the perception disk, default radius
50 m. A complete current empty scan or an observed target outside the disk
can clear the runtime encounter; a missing observation alone retains its
forecast. Actual reacquisition creates a new encounter and a new
initial-feasibility question. A predicted center exit cannot certify that all
uncertain branches have exited.

<a id="observer-geometry"></a>
## 2. Observer bounds and relative geometry

### Current-state containment from a comparison system

The controller consumes the estimator's current certified enclosures. Its
observer derivation remains in
[OBSERVER_ISS_THEORY.md](../estimator/OBSERVER_ISS_THEORY.md), including
the shared-model force-balance measurement and its ego error bound.
For the continuous cascade, write

$$
\chi=(\|e_v\|,\sqrt{e_T^\top Q_oe_T})^\top,\quad
D^+\chi\le H\chi+d,\quad
H=\begin{bmatrix}-k_v&0\\c_{Tv}&-\lambda_T\end{bmatrix},
$$

where $k_v,\lambda_T>0$, $c_{Tv}\ge0$ and $d\ge0$. With
$Q_o=(D_oP_oD_o)\otimes I_2$ and
$D_o=\operatorname{diag}(1,\omega_o^{-1},\omega_o^{-2})$, a valid initial bound
$b_0\ge\chi(t_0)$ gives

$$
b(t)=e^{H(t-t_0)}b_0+\int_{t_0}^t e^{H(t-s)}\bar d(s)\,ds,\qquad
\chi(t)\le b(t),\quad \bar d\ge d.
$$

**Derivation.** $H$ is Metzler, hence its transition matrix is entrywise
nonnegative. Apply the comparison principle and variation of constants.
For constant forcing, the integral is $H^{-1}(e^{H\tau}-I)\bar d$.
The off-diagonal entry of $e^{H\tau}$ is
$c_{Tv}(e^{-k_v\tau}-e^{-\lambda_T\tau})/(\lambda_T-k_v)$, or
$c_{Tv}\tau e^{-k_v\tau}$ at equal rates. The steady-state value
$-H^{-1}\bar d$ cannot replace an initial transient that it does not contain.

The resulting ellipsoid has support

$$
\mathcal E_T=\{e:e^\top Q_oe\le b_T^2\},\qquad
\sigma_{\mathcal E_T}(a)=b_T\sqrt{a^\top Q_o^{-1}a}.
$$

This follows by Cauchy--Schwarz in the $Q_o$ metric. Its component radii are
$b_\rho=\sqrt{(P_o^{-1})_{11}}b_T$,
$b_q=\omega_o\sqrt{(P_o^{-1})_{22}}b_T$ and
$b_s=\omega_o^2\sqrt{(P_o^{-1})_{33}}b_T$.
Using the joint support preserves more correlation than independent balls.

The sampled observer requires its own validated finite-time enclosure or a
sampled comparison $b_{k+1}\le M_kb_k+d_k^{\rm obs}$ with $M_k\ge0$.
A continuous theorem alone does not certify RK4, held measurements, timing or
outages. Append independently valid yaw, gyro and absolute-position bounds.
An energy-gain statement alone is not a pointwise yaw enclosure. Current
publication (`futurePredictionIncluded=false`) is not horizon propagation.
A deterministic amplitude contract also cannot be inferred from unbounded
Gaussian simulation noise.

### Rotating-frame reconstruction and common-pose cancellation

Let $J=\left[\begin{smallmatrix}0&-1\\1&0\end{smallmatrix}\right]$ and

$$
\rho=R_E^\top(p_C-p_E),\quad q=R_E^\top v_C,\quad s=R_E^\top a_C.
$$

Differentiation gives the NRMM transport equations

$$
\dot\rho=q-v_E-r_EJ\rho,\quad
\dot q=s-r_EJq,\quad \dot s=\Phi(q,s)-r_EJs,
$$

$$
V_C=\|q\|,\quad A_C=q^\top s/V_C,\quad
\Omega_C=(Jq)^\top s/V_C^2,\quad
\Phi=-\Omega_C^2q+3A_C\Omega_CJq/V_C.
$$

The factor three follows by differentiating
$a_C=A_Ct+\kappa_CV_C^2Jt$ with constant $A_C,\kappa_C$.
$\Omega_C$ is course/yaw rate, not a sideslip derivative. On the
positive-speed, $|\beta_C|<\pi/2$ domain,

$$
\beta_C=\arcsin\frac{l_{rC}(Jq)^\top s}{\|q\|^3},\qquad
\psi_R=\arg q-\beta_C.
$$

A sound relative-yaw enclosure must cover the image of the entire joint
$(q,s)$ set. One conservative bound uses
$\nu=\|\hat q\|-b_q>0$, $S_*=\|\hat s\|+b_s$, and a true domain
$|\beta_C|\le\beta_{\max}<\pi/2$:

$$
b_\beta=\frac{l_{rC}}{\cos\beta_{\max}}
\left(\frac{4S_*b_q}{\nu^3}+\frac{b_s}{\nu^2}\right),\qquad
b_{\psi_R}=\min\left(\pi,\arcsin\frac{b_q}{\|\hat q\|}+b_\beta\right).
$$

Here the center's arcsine argument is clipped to the valid sideslip range.
The derivative bounds of the unclipped argument are $4l_{rC}S_*/\nu^3$ in
$q$ and $l_{rC}/\nu^2$ in $s$; clipping is nonexpansive and arcsine's slope
on that interval is at most $1/\cos\beta_{\max}$. The velocity ball excludes
zero along every connecting segment. At possible zero speed this inverse
supplies no heading certificate; retain a valid heading set or unknown yaw.
Clipping the estimate cannot establish the true domain premise.

For offset body rectangles $B_E,B_C$, rigid-motion invariance gives

$$
\operatorname{sdist}(\mathcal V_E,\mathcal V_C)
=\operatorname{sdist}(B_E,\rho+R(\psi_R)B_C).
$$

Common absolute translation and rotation cancel from instantaneous collision
geometry. Absolute pose uncertainty still affects path/road conditions, and
velocity/rate errors still affect prediction. Relative uncertainties need not
be independent. Ordinary distance is zero even for overlap; its nonnegativity
is therefore insufficient to certify nonoverlap.

<a id="optimization"></a>
## 3. Collision constraints and the two-stage problem

### Separating planes, supports and nonlinear reserves

For a unit normal $n$ pointing from ego toward target, a sufficient separation
condition is

$$
\min_j n^\top(\rho+R(\psi_R)v_{C,j})-h_{B_E}(n)\ge d_0,
\qquad h_{B_E}(n)=\max_{v\in B_E}n^\top v.
$$

Every ego point lies on one side of this plane and every target point at
least $d_0$ beyond it. Cauchy--Schwarz gives Euclidean separation for $d_0>0$;
at $d_0=0$ the nonzero normal excludes interior overlap and allows contact.
The implementation uses zero extra physical clearance. Observer inflation
is separate from a design buffer.

At anchor $\bar p_r=(\bar\rho,\bar\psi_R)$, define

$$
g_j^0=n^\top(\bar\rho+R(\bar\psi_R)v_{C,j})-h_{B_E}(n),\quad
a_j^\top=[n^\top,\ n^\top R(\bar\psi_R)Jv_{C,j}].
$$

For an affine pose correction $\Delta p_r$, pose-error set $E_r$ and bounds
$|\Delta\psi_R|\le t_\psi$, $|e_{\psi_R}|\le b_\psi$, a sound sufficient row is

$$
g_j^0+a_j^\top\Delta p_r-\sigma_{E_r}(-a_j)
-\tfrac12\|v_{C,j}\|(t_\psi+b_\psi)^2
-\epsilon^{\rm hold}-\epsilon^{\rm num}\ge d_0-\xi_i.
$$

The quadratic reserve follows from
$|n^\top R(\psi)J^2v|\le\|v\|$. Nonlinear pose reconstruction and dynamics
require additional sound enclosures. For a zonotope $G[-1,1]^m$,
$\sigma(a)=\|a^\top G\|_1$; for an ellipsoid use Section 2. The decision
domain must actually satisfy the bounds used for every reserve. The current
RTI has unbounded pose corrections and does not implement this complete row;
a local yaw tangent alone has no uniform remainder certificate.

Exact angular support is another enclosure: for each offset vertex, maximize
$n^\top R(\theta)v$ over the allowed interval by checking its endpoints and
interior angular maxima, then maximize over the four vertices. The runtime
uses this directional target-yaw support minus its nominal support. Its ego
yaw reserve remains the uniform chord
$2\,\mathrm{reach}\sin(\min(\pi,b_\psi)/2)$ in RTI; the guide may use exact
ego support at a fixed pose. These orientation bounds do not certify the
remaining nonlinear decision corrections.

Ordinary-distance dual multipliers use the exact closest-feature normal when
separated. At overlap, a nonzero support escape direction replaces the zero
distance dual. The fixed-multiplier corner rows include yaw derivatives.
Encounter-side persistence prevents left/right chattering; these are local
convexification choices, not a proof that a feasible separating chart exists.

### Constraint instants and a continuous-time extension

The runtime constrains nodes and hold midpoints. On a one-RK4-step mesh its
midpoint is the endpoint interpolant, not an exact physical midpoint.
For true whole-hold speed/rate bounds, body reach radii $R_E,R_C$ give

$$
L_d=\bar v_E+\bar v_C+R_E\bar r_E+R_C\bar r_C,\qquad
\epsilon^{\rm hold}=L_dh/4.
$$

**Derivation.** Every body moves in Hausdorff distance by at most translation
plus rotational reach times elapsed time. Every instant is within $h/4$ of
start/midpoint/end. Tightening true nonoverlap certificates at those instants
by the motion reserve therefore suffices throughout the hold. Midpoint
integration/interpolation defects must also be enclosed. This extension is a
sufficient construction, not a reserve already established by current replay.
The ideal theorem below guarantees only the declared constraint instants
unless such whole-hold images are supplied.

### One RTI model and lexicographic objectives

For a nonlinear reference $(\bar x_i,\bar u_i)$ with
$\bar x_{i+1}=F_h(\bar x_i,\bar u_i)$, variational tangents give

$$
\Delta x_0=0,\qquad
\Delta x_{i+1}=A_i\Delta x_i+B_i\Delta u_i.
$$

The current input estimate is fixed; current state error is handled by the
published enclosure, not a free optimizer correction. Fixed collision duals
and terminal tangents are built once on this model. There are at most two
native convex solves per reference, without convergence-driven nonlinear
iterations. A later nonlinear prediction records discrepancy; it neither
repairs nor vetoes the optimized command.

### Implemented constraints and objective units

State and input increments have no artificial search bounds. Actual longitudinal-force
magnitude and optional slew limits remain hard. Steering has no artificial
actuator magnitude or slew limit; the Fiala angle domain is enforced. State
limits, sideslip, optional road-corner rows and terminal
rows retain their existing meanings. No shared elastic variable relaxes them.

Nominal collision rows in the first `controller.horizonSteps` nodes carry
nonnegative PCBF slacks. With observer uncertainty, every tightened collision
node also carries a nonnegative robustness-deficit slack; the nominal collision
row remains hard after the prefix. These explicit collision deficits are
separate from physical state, road, handling and terminal constraints.

The primary stage minimizes `sum(xi)`. If the zero correction with zero slacks
satisfies every primary constraint, it attains the global lower bound and no
native primary solve is needed. Otherwise the native conic solver runs once.
The secondary stage caps the attained primary objective, with the configured
numerical tie, and uses the target-independent soft CLF condition

    V(xhat_next) <= rho * V(xhat_now) + clfSlack,
    rho = exp(-2*h/clf.convergenceTimeConstantSeconds).

`V` is the single precomputed transverse quadratic at prescribed-speed/path
trim. Only the estimate enters it; it does not ask the worst member of an
observer enclosure to contract to zero. The secondary objective is

    J2 = clfSlack + (lambda / (2 N)) sum_k (
         ||Dx (x_aff,k+1 - xbar_k+1)||^2 + ||Du (u_k - ubar_k)||^2).

Here xbar and ubar are this RTI reference, including a fresh planner rollout or
the shifted previous input rollout. They are not the nominal cruise path.
`lambda = rti.referenceDeviationWeight` defaults to 10 and remains active at
zero cruise error; zero is an explicit ablation. Dx uses equal x/y position
units from `clf.lateralPositionErrorScale`, followed by the CLF heading,
speed, lateral-velocity and yaw-rate scales. Du uses the CLF certification
steering/braking units. These are cost units, never bounds. The horizon mean
keeps the overall weight independent of node count. The solver divides the
entire objective by `max(1,V(xhat_now))` for numerical scaling.

The old whole-horizon attraction to the cruise route is removed from this
objective. Cruise recovery remains in the CLF and planner path field. Larger
lambda permits more CLF slack to retain the reference, but cannot relax the
primary PCBF cap or other constraints. This changes performance preference,
not the feasible set or the conditional shift-candidate feasibility argument.
It does not guarantee small nonlinear prediction error or successful RTI.
Both stages must complete; an unfinished secondary stage
does not cause execution of the primary point.

Modified Fiala dynamics include saturation at both axles. There is no extra
rear-tire nonsaturation cone in the optimization or terminal membership test.
The force-balance observer encloses saturated forces too; its interval can
widen when the measurement becomes less informative. Its separate sideslip
premise `|vy| <= tan(model.sideslipMaximum)*vx` remains enforced, with observer
support margins. Removing that premise would require a new observer enclosure.
The precomputed local cruise certificate may use a conservative nonsaturated
neighborhood; that local certificate does not impose a full-horizon cone.

Physical input bounds include force ratios -1 and +1. The optimizer uses
these bounds and issued inputs are unchanged. Only the tangent reference
uses `nonlinearBicycleModel.inputBounds(cfg,true)`, which keeps its force
ratio ten constraint tolerances inside the singular endpoints. Shifting a
saturated previous plan rebuilds this interior seed and its complete rollout;
this does not change a previously issued command. The 100 ms controller budget
reserves 2 ms for completion, including any replan and retry. A successful first
attempt needs one RTI model and at most two native solves. The number of
reference/horizon attempts has no fixed cap; all attempts share the deadline.

The executable collision deficits and finite numerical primary cap must not
be identified with the ideal prefix-only robust value in Section 7. With
uncertainty, the implementation has robustness-deficit slacks at tightened
collision nodes beyond the nominal prefix as well. The conditional proof
uses zero-deficit hard-tail and terminal rows.

The runtime restarts the present ego enclosure $G_0$ at each future hold and
uses $G(t_i+\tau)=\Phi_i(\tau)G_0$ over that hold. Target uncertainty similarly
restarts at its forecast state. This assumes a future posterior width; it
does not construct nonlinear reachable images or prove cross-sample inclusion.
Usable finite bounds tighten the program. Missing or stale certificates do
not block experimental execution; omitted padding does not assert zero error.

<a id="nominal-clf"></a>
## 4. One nominal CLF and recovery

### Function and its scope

For the given path and desired cruise trim, define

    e(x) = [e_y; e_psi; vx-vx_ref; vy-vy_ref; r-r_ref],
    V(x) = e(x)' P e(x),       P > 0.

Longitudinal path phase is free. `nonlinearBicycleModel.cruise` computes the trim
and reads P, the gain K of the cruise controller and their certificate from
`config/clfMatrices.json`; it never computes them. `nominalValue` evaluates
this quadratic directly. There is no LQR design; the gain K is used only by
the cruise controller `u = u* + K e`, whose hold extends the anchor while it
keeps the endpoint in the terminal set (`terminalSafeSet.cruiseInput`;
[the terminal-set initialization discussion](#terminal-set)), never by the
optimizer. The terminal set itself does not use the CLF. No nominal-policy rollout, finite-difference Hessian, cost-to-go
kernel, or additional terminal value is used to construct the CLF.

### Requested decrease: a convergence time constant

Each sample requires

    V(x_next) <= rho * V(x) + slack,   rho = exp(-2 h / T),

with h = 0.05 s and T = `clf.convergenceTimeConstantSeconds` = 4 s. The error
size sqrt(V) then shrinks by at least 1/e every T seconds, in every direction
of the error as V measures it; one lane width (3.66 m) returns to within
0.3 m in about 10 s. T replaces the former `nominalClf.decreaseFraction`
(eta = 0.5 times e'Qe), whose speed differed between error directions by a
factor of about 25 for the same eta.

### Offline certificate of P and the cruise controller

`scripts/synthesizeClfMatrices.m` computes P and K before experiments, for
each operating point (vehicle parameters, speed, path curvature, sample time
and CLF settings), and writes them with their key and certificate to
`config/clfMatrices.json`. A controller call at an operating point without an
entry is rejected with `collisionAvoidanceController:missingClfMatrix`, an
entry without the certificate with `collisionAvoidanceController:staleClfMatrix`;
nothing is synthesized at run time, and the run time does not need the SDP
solver.

The synthesis targets the sampled nonlinear model through a ray-average
enclosure, rather than only the trim linearization. Its coverage premise is
made explicit below. Let `g(e, du)` be the transverse error after one hold from the
state with error `e` under the input `u_ref + du`, and `G = [dg/de, dg/d(du)]`
its Jacobian from the RK4 variational equations and the error chart
(`terminalSafeSet.jacobians`). The trim is a fixed point, so along the ray
from `0` to `(e, du)`

    g(e, du) = Gbar(e, du) [e; du],   Gbar(e, du) = integral_0^1 G(t e, t du) dt,

exactly. The recorded enclosure is constructed from ray averages `Gbar` along
the controller over the set (`terminalSafeSet.rayJacobians`, Gauss-Legendre quadrature with
8 nodes; the quadrature's residual in the identity is added to the enclosure's
residual as a rank-one term), so a discrete Lyapunov inequality that holds for every matrix of the
enclosure transfers to the nonlinear map if the enclosure covers every
required ray average. Enclosing the ray
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
on the new set). Every round's pair therefore satisfies the matrix inequalities for the
enclosure it was designed with, and the set may grow by 1.5 in `V` per round until the
enclosure's growth stops it; the pair with the largest fill is kept. Its
vertex inequalities are re-verified by eigenvalues and the smallest
contraction they certify is recorded (`certifiedContraction`, at most `rho`).
The partial-hold maps are enclosed the same way, on the same samples, to give
the hold factor, the excess of the CLF radius inside a hold.
YALMIP/SeDuMi from `solver/`.

The remaining coverage premise is the enclosure hypothesis (all ray averages
over the set lie in the recorded zonotope): it is built from 1200 boundary and 600 interior
samples of the inflated set with a recorded seed. The entry records the
samples' seed, the coefficient bounds, the residual (with the quadrature's
share), the largest slip angles reached, the inputs' maxima, and an
independent sampled check with another seed (`terminalSafeSet.certificate`:
worst contraction, hold factor, the problem's rows); `nominalClfTest` repeats
that check and tests fresh ray averages against the recorded zonotope.

**This quadratic is not a global nonlinear CLF.** Large heading errors, tire
saturation, state-domain boundaries and braking memory
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

### One convex first-successor constraint

The trajectory is linearized once around a nonlinear rollout. At its first
successor, use the analytic Frenet error Jacobian J:

    e_aff = e(xbar_1) + J(xbar_1) * dx_1,
    V_aff = ||chol(P) * e_aff||^2.

The same affine dynamics relate dx_1 to the first input correction. Its Hessian
is positive semidefinite, so the first-step CLF epigraph is a convex quadratic
constraint represented by one second-order cone, so the optimization is a sparse
conic QP/SOCP rather than a QP with exclusively affine constraints.

The secondary objective is CLF slack plus half
`rti.referenceDeviationWeight` times the horizon mean of squared, unit-scaled
state and input deviations from the current linearization reference. The
default weight is 10, independent of cruise error and reference source; zero
disables the penalty for ablation. Position uses the same CLF lateral-position
unit for both inertial coordinates. Yaw, longitudinal speed, lateral velocity
and yaw rate use the corresponding CLF error units. Inputs use the CLF
certification steering and braking units. The entire objective is divided by
`max(1,V(xhat_now))` in the solver. Reported component costs are unscaled.

The primary PCBF slack cap remains hard. There is no state/input correction
box and no separate whole-horizon cost pulling toward the cruise path. The
penalty is zero at the current reference, which may be a bypass trajectory.
It remains active near nominal motion to regularize otherwise free later
inputs. The CLF still refers to the prescribed cruise path. A larger reference
weight can trade increased CLF slack for smaller trajectory changes; it does
not certify nonlinear CLF decrease or prevent every large correction.

With an observer enclosure the row is the same, on the estimate:
`budget = exp(-2h/T)*V(xhat)` and `lossLower = e0'Qe0` at the estimate.
Estimation error enters the decrease as an input (input-to-state), not through
a worst case. `clfWorstNextValue` equals the modeled successor value and `clfCurrentBudget`
the budget above.

The remaining nonlinear error-chart and dynamics remainders are not enclosed.
Neither zero affine slack nor this implementation proves exact asymptotic
recovery of the nonlinear uncertain plant; with estimation error the closed
loop can at best approach a neighborhood of the path set by that error. See the conditional theorem in
`OBSERVER_ROBUST_PCBF_THEORY.tex` for the additional premises.

This constraint controls the first successor only. Future inputs are chosen
through horizon feasibility and the input correction cost; they do not each
receive a CLF decrease constraint. After shifting, the second planned input becomes the next first-input anchor.
The reference penalty can therefore change later recovery behavior even when
the current minimum CLF slack changes little.

The PCBF budget is a hard constraint of this convex problem. It comes from the
primary slack minimization of the same frame. A zero primary value never skips
CLF optimization.

### What the offline enclosure certifies

The LMI establishes contraction for every matrix in its recorded enclosure.
The enclosure is built from sampled ray averages, a quadrature residual and
an empirical margin factor. Covering sampled points does not alone establish
uniform coverage of every nonlinear ray on the ellipsoid. Thus an unconditional
nonlinear certificate additionally needs a validated enclosure of that complete
domain. The stored certificate and `nominalClfTest` give reproducible numerical
checks; they do not replace that coverage premise.

### Disturbance-dependent recovery: a sufficient theoretical construction

This extension retains the same $V$ but is not the current estimate-only row
or fixed reference penalty. Suppose a common recovery action on a verified
domain satisfies

$$
\sqrt{V(F(x,\kappa(x)))}\le a\sqrt{V(x)},\quad 0\le a<1,
$$

and estimate-feedback error, admitted plant defects and integration effects
perturb the successor in the $P$ norm by at most $D_k$. For $a^2<\lambda_c<1$,

$$
(a\sqrt V+D_k)^2\le\lambda_c V+
\frac{\lambda_c}{\lambda_c-a^2}D_k^2.
$$

The difference is the square
$(\lambda_c-a^2)(\sqrt V-aD_k/(\lambda_c-a^2))^2$.
Choose $Q\succ0$ and $\eta Q\preceq(1-\lambda_c)P$. With
$\ell=e^\top Qe$ and $\sigma_k=\lambda_cD_k^2/(\lambda_c-a^2)$, this yields

$$
V(x^+)-V(x)+\eta\ell(x)\le\delta_k+\sigma_k.
$$

The supply is derived from the disturbance contract; $\delta_k\ge0$ is CLF
relaxation. Zero-relaxation feasibility additionally requires that the common
recovery action satisfy collision, terminal, input and domain constraints.
If that candidate exists and the secondary objective is

$$
\min\;\delta_k+\epsilon\underline\ell_k R(\Delta U),\quad
0\le R\le1,\quad0\le\underline\ell_k\le\inf\ell,\quad0\le\epsilon<\eta,
$$

an optimization gap $\tau_k$ gives
$\delta_k\le\epsilon\ell_k+\tau_k$. Therefore

$$
V_{k+1}-V_k\le-(\eta-\epsilon)\ell_k+\sigma_k+\tau_k.
$$

For $\ell\ge\mu V$, $c=(\eta-\epsilon)\mu\in(0,1]$, iteration gives

$$
V_{k+j}\le(1-c)^jV_k+
\sum_{s=0}^{j-1}(1-c)^{j-1-s}(\sigma_{k+s}+\tau_{k+s}).
$$

Bounded supply/gap gives a practical neighborhood; a vanishing supply/gap
gives convergence on the verified invariant domain. The runtime's fixed,
unbounded quadratic reference-deviation cost is not this bounded vanishing
tie. It cannot inherit this recovery result merely because the CLF row is
soft. The runtime also does not certify the exact nonlinear dissipation row.

<a id="terminal-set"></a>
## 5. Joint terminal set and braking continuation

### State and task boundary

The terminal condition belongs to the **joint encounter**, not to an ego speed
or a parking location. Its state is

    z = (x_E, q_C, a, u_previous),
    x_E = (p_E, psi_E, v_x, v_y, r),
    q_C = (p_C, psi_C, V_C, A_C, beta_C, l_r,C, rectangle_C).

The flag `a` is one while this encounter is active and zero after its first
exit from the ego perception disk, whose configured radius is 50 m. At each
constrained node or midpoint it evolves as

    a_next = a AND (norm(p_C_next - p_E_next) <= R).

During the parked phase the first exit is computed in continuous time. An
ended flag cannot be cleared by predicting a later re-entry. A fresh actual
observation after an actual exit creates a new encounter, with a new
initial-feasibility question. A missing measurement alone is not evidence
of an exit: only a complete current empty scan or a supplied observation
proved outside the disk clears the runtime forecast.

This boundary imposes no arbitrary time limit. If the target never leaves
the disk, it remains part of the encounter. A curved target that stays in
range cannot be dismissed just because it turns or the ego stops.

`jointState`, `jointSuccessor`, and `jointMember` expose this state and its
transition. `member(context,x,node)` and `rows` are the RTI adapters: the
context supplies the target at the same absolute time and the ended flag.
The controller's `encounterId` advances only on actual reacquisition; a
predicted exit does not erase a currently observed target.

Terminal membership certifies an available continuation. It neither commands
parking nor switches off MPC. Moving cruise states may belong to the set.
Normal operation still aims to avoid the target and recover the prescribed
path and speed. This use of a terminal candidate follows Huang et al. (2025),
Section III, equations (8) and (10),
[primary source](https://arxiv.org/abs/2502.08400).

### One terminal controller

In the accepted mild kinematic model, retain the steering and apply a fixed
braking command, with a forward-only absorbing stop:

    b_f = -a_b / longitudinalAccelerationGain,
    delta_f = latched steering,
    dV/dt = -a_b while V > 0; V(t) = 0 after stopping.

`control(context,joint)` returns this command and its modeled acceleration.
The command does not depend on an unknown true speed to choose the last
hold. There is no final weak-braking hold and no reacceleration after rest.
The default `a_b` is 4 m/s^2. With initial speed V and elapsed time t,

    tau = min(t, V/a_b),
    s(t) = V*tau - a_b*tau^2/2,
    t_stop = V/a_b,       s_stop = V^2/(2*a_b).

The stop may occur inside a sampling hold. `continuation` evaluates this
flow at nodes and midpoints; the parked-track check starts at the actual
stop instant, not at the following sample boundary. `successor` is its ego
projection and `jointSuccessor` also advances the target and exit memory.

The mild-model assumption keeps beta_E = atan2(v_y,v_x) and kappa_E = r/V
constant while moving. Consequently

    p_E(t) = p_E + s*sinc(kappa_E*s/2)
                   * direction(psi_E + beta_E + kappa_E*s/2),
    psi_E(t) = psi_E + kappa_E*s.

At rest the pose is fixed. The target uses the same constant tangential
acceleration and sideslip prediction as the main controller, with its own
forward-only stop. There is no backward extrapolation through a stop.

This is the declared terminal model, not an identity with the Fiala plant.
In particular, a held actuator ratio and road loads do not independently
prove exact constant deceleration or curvature in the full plant. A robust
claim needs the compatible terminal uncertainty contract described below.

### Terminal set

Let Phi_f be the joint flow including the absorbing encounter flag. Let C
require the declared physical/input conditions, the reference-point deviation
bound and any enabled road constraints,
and require rectangle nonoverlap only on active branches. The extra design
clearance is zero; uncertainty inflation remains part of the required support.
Then the terminal set is the fixed-policy safe set

    X_f = { z : x_E is mild and Phi_f(z,t) satisfies C
                 at all declared constraint instants }.

Here mildness requires |v_y| <= tan(beta_f)*v_x and |r|*V <= a_m, with
beta_f = 0.05 rad and a_m = 2 m/s^2 by default. The braking ratio must satisfy
actuator limits. No additional rear-tire nonsaturation condition is imposed. Since |kappa_E|*V(t)^2 decreases and beta_E
is constant, the mild conditions persist under the declared flow. Input
memory is part of the joint state; finite slew limits require an admissible
entry command, not just a geometric certificate.

The executable membership test checks:

1. Mildness and terminal braking admissibility.
2. Ego reference-point deviation, and road occupancy when enabled, at nodes,
   midpoints, and its parked pose. Absolute ego position uncertainty tightens
   the deviation bound; no footprint radius or extra physical clearance is
   subtracted from it. `Inf` explicitly disables this bound.
3. Joint rectangle clearance during braking until the first exit.
4. If the ego stops before exit, its parked rectangle against the target's
   remaining in-range track, beginning at the exact stop time.

The fourth check is necessary: a target can collide with the ego shortly
after it stops while both remain in range. It excludes all motion after the
first exit. The check is analytic: target vertices sweep line segments or
circular arcs against ego edges, and vice versa. A target that stops inside
the disk contributes its stationary endpoint. If a circle stays wholly
inside the disk, its swept track still belongs to this encounter.

No integer terminal index is interpreted as an execution deadline. The
prediction endpoint can be a moving mild state with an available braking
continuation; the main controller need not choose that continuation.

### Shift closure and the role of uncertainty

For exact information under the declared terminal contract, take z in X_f.
The continuation of Phi_f(z,h) is the suffix of the original continuation:
the clipped-speed flow has the semigroup property, the target is advanced
by the same elapsed time, and input/encounter memory are retained. Every
active successor row is therefore an old certified row. Once the encounter
ends, no future prediction can reactivate it. Hence Phi_f(z,h) is in X_f.
Appending this rule to a shifted feasible plan supplies the usual
recursive-feasibility witness. A soft performance objective does not remove
that candidate. This is an existence statement for the ideal problem;
local RTI solution accuracy and deadlines are separate.

For the joint observer-controller design, replace a point by a nonempty
joint information set J containing physical ego and target states, shared
model parameters, observer state, input memory, and the encounter flag.
The terminal family is

    B_f = { J : one common causal terminal policy satisfies all hard rows
                on every admissible branch until that branch's first exit }.

The flag is branch-specific: a center outside range does not certify that
all possible targets have exited. A computed outer set may discard the
target only after every remaining active branch has exited. An unknown
parameter remains the same along each branch. Model mismatch already
represented in the current observer enclosure is not added again as an
independent initial error; future physical model defects must still be
propagated jointly with the observer and feedback.

For a common held braking rule with guaranteed realized deceleration
at least a_min > 0, every branch with V <= V_bar stops by V_bar/a_min and
travels at most V_bar^2/(2*a_min). Pose uncertainty remains after stopping.
All reachable parked poses and all still-active target branches must be
checked. A disturbance that can move a parked vehicle needs a compatible
parked invariant set. Holding an invalid low-speed observer bound constant
is not a substitute for this physical continuation certificate.

The terminal family is downward closed. Its exact joint image remains in
B_f because the same policy tail covers every successor branch, including
its input and encounter memory. A certified outer successor must likewise
belong to B_f. This is the terminal-closure step used in
[the recursive feasibility section](#recursive-feasibility) and
[the joint prediction section](#joint-prediction).

The runtime RTI is still an approximation to this information-set problem.
Its terminal geometry uses the target center and the published ego pose
padding; it does not implement a universally certified full nonlinear
terminal reachable set. This redesign does not promote those margins into
a robust certificate. `robustNonlinearSafetyCertified` remains false.

### Constraint-grid closure and implementation detail

A sampled terminal certificate must retain the constraint schedule under a
shift. The uniform nodes/midpoints are closed under a one-hold shift. The
exact stop and remaining parked track must be checked consistently as well.
An arbitrary recomputed grid would need an additional inclusion argument.
Invariance here concerns the declared constraint instants, unless a
continuous-time continuation enclosure is supplied.

The joint transition and terminal continuation use the first-exit convention.
The main RTI horizon no longer declares a future exit from a bounded reference
neighborhood: pose corrections are unbounded. It retains the active target's
collision rows until an observed exit. Terminal rows retain all computed
continuation collision conditions when no certified endpoint correction bound
is supplied, including distant conditions previously omitted by a fixed band.
Both signs of the lateral-acceleration condition are linearized for a moving
endpoint. The terminal continuation's sampled schedule and exit branch remain
local approximations and are not a global nonlinear exit certificate.

`rows` builds the local terminal approximation once. At circular-track
intersection, a nonconvex safe set can admit an inner or an outer local
separation chart. The implementation selects the smaller radial/support
escape using the actual joint geometry, independently of road-side
preferences. These are two local descriptions of the same set under the
same terminal policy; they are not two driving controllers or extra RTI
iterations. Neither chart nor its linearization is a global certificate.

An inherited RTI tail whose endpoint is terminal first tries the certified
cruise-feedback hold. The existing `brakingInput` helper then uses a bounded
set of declared-model holds and five scalar halvings to find a less
restrictive terminal-preserving seed. This numerical initializer is distinct
from the unique `control` law used to define the set. It adds no optimizer
iterations. If no certified hold is found, a moving endpoint uses the
nominal-feedback seed; an already stopped endpoint keeps a nonpositive
longitudinal command. This prevents an uncertified restart from creating a
poor accelerating reference while preserving ordinary cruise recovery. The
optimizer still chooses and issues the final input unchanged.

<a id="joint-prediction"></a>
## 6. Joint feedback-aware prediction

This is the conditional mathematical output-feedback design, not the runtime's
per-hold enclosure restart. Future measurements are unknown bounded variables.
Tube widths may vary, expand temporarily and retain a noise floor; a
constant-width invariant tube is optional. The model, observer, sensor and
terminal contracts must cover the same uncertainty family.

### Predict a control rule that uses future observations

An open-loop plan fixes u_0,...,u_(M-1). A feedback plan fixes causal rules
pi_0,...,pi_(M-1). Each future rule maps the information available at that
sample to an input; it cannot use later measurements. The current observation
is already known, so pi_0 returns a definite optimized current input.

For nominal ego state z and input v, a local rule is u=v+K*(xhat-z).
Define observer error epsilon=x-xhat and physical tracking error eta=x-z.
For sampled plant F and a declared defect w,

    eta_next = F(z+eta,v+K*(eta-epsilon)) - F(z,v) + w.       (1)

The linear version is

    eta_next = (A+B*K)*eta - B*K*epsilon + w.                 (2)

The propagation matrix is A+B*K, not A. Observer error enters as a bounded
feedback disturbance. Input constraints must cover K*(eta-epsilon), including
input memory/slew. Clipping an input outside this model changes the policy.

### Propagate a varying closed-loop tube over the finite horizon

Let q contain physical deviations and actual observer/software state, together
with reference, input and mode memory when needed. The current software state
and observation are known. The initial set retains the observer-certified
physical uncertainty, while the current policy returns one common input.
For each future hold, the joint map G_i includes the nonlinear vehicle,
bounded sensor history, actual observer update and chosen causal policy.
Construct sound outer sets satisfying

    Omega_(i+1) contains {G_i(q,nu): q in Omega_i, nu in W_i}.

The physical tube at node i is the projection of Omega_i, with separate
sound hold/midpoint images as required by the constraint contract. There is
no requirement Omega_(i+1) subset Omega_i or that all projected widths equal
a single radius. Different nominal states, gains, observation quality and
noise contributions can produce different widths. Keep the same policy/set
meaning when shifting the plan and conditioning on the next measurement.

A comparison implementation can use time-varying certified coefficients,

    E_(i+1) <= a_(o,i)*E_i+d_(o,i),
    r_(i+1) <= rho_i*r_i+gamma_i*E_i+d_(F,i).

These describe future observation correction and physical tracking, rather
than a frozen-input plant-only growth factor. Temporary expansion is allowed.
No common contraction metric or constant invariant radius is prerequisite for
finite-horizon propagation. Compact scalar comparisons must still be sound;
joint generators can retain correlations that scalar boxing discards.

For a linear joint surrogate q_next=J_i*q+D_i*nu+c_i, propagate a center m
and generator Z by m_next=J_i*m+c_i and Z_next=[J_i*Z,D_i]. Take directional
supports after the matrix products. Repeatedly propagating coordinate widths
with abs(J_i) loses cancellation between the plant and observer and may itself
create large artificial tails. Initial uncertainty is not restarted at each
hold, and primitive measurement noise is not added again as an independent
observer-error disturbance. The first map uses the known current input;
subsequent maps include the future feedback law and its gyro feedthrough.

Small future estimation error about each branch's updated estimate does not
by itself bound physical deviation from a frozen nominal trajectory. The
feedback policy's input variation must therefore be part of G_i. Replanning
in experiments alone does not specify that prediction policy. Unknown future
measurements remain bounded variables, not known or noise-free values.

The remaining nonlinear work is to construct sound finite-horizon images,
admit their policy-generated inputs and constraints, and connect the endpoint
set to the robust terminal continuation. Once entering the mild terminal
model, braking and a fixed physical parked pose set may be certified without
requiring continued contraction of the noisy yaw estimate. Recursive
feasibility uses the cross-sample inclusion and terminal shift argument below;
it does not require a constant prediction-tube width.

### One uncertainty source and its different consequences

Let theta be a fixed unknown vehicle-parameter vector in Theta. The initial
project family varies front/rear cornering stiffness and friction within the
observer's declared relative box (default 5%); mass, inertia and geometry are
not implicitly uncertain merely because that box is present. Let d_i be any
separately declared residual plant disturbance and nu_i the primitive sensor
noise history. The tire box restricts parameter values; constancy or a drift
rate does not follow from a value bound alone. Actuator errors and external
forces require compatible bounds if admitted; the tire box does not cover
arbitrary additional forces.

Take decision instants after the measurements used for that input are known.
Let q contain the physical ego/target state X, theta, actual observer point
and bound/history state s, the available measurement record y, previous input,
mode/encounter memory, and the policy's reference memory where necessary.
The observer uses its specified nominal model and parameter bounds. The true
plant uses theta. The chronological sampled map is

    u_i = pi_i(s_i,y_i,reference_i),
    q_(i+1) = G_(pi_i)(q_i,d_i,nu_i),   theta_(i+1)=theta_i.       (M1)

G executes the plant, measurement generation, predictor resets, observer
integration and reconstruction in their actual sample/hold order. Future
measurements come from the same physical branch. A raw gyro channel is a
measurement, not privileged true yaw rate. Its noise is sampled once and
shared by every path through the observer and control. G also defines the
held-input trajectory images at constrained instants and over the full hold.

If parameters may vary, replace the identity in (M1) by a declared transition
theta_next in Gamma(theta), including any intra-hold parameter history. A
range-only contract can use Gamma(theta)=Theta, without asserting a drift
rate. Retain one shared parameter history in plant and observer effects.
The outer-image and shift proofs apply with that same transition at both
frames. Equations (M3), (M4) and the constant-parameter term in (M8) specialize
to fixed parameters; they must not be used for a broader varying family.

At the root, current s and y are known: all physically possible branches
receive the same current input. Future s and y become branch dependent. The
root information set is

    I_k = {q consistent with the retained information history,
           X in the published physical enclosure,
           theta in the retained parameter set,
           s=s_k, y=y_k and known input/mode memory}.             (M2)

The product of a published enclosure and Theta is a sound initialization if
only marginal information is available; it can lose correlation with theta.
Subsequent propagation retains the same theta and conditions the joint set.
No new independent parameter draw is added at each hold.

The published error enclosure already contains effects of past admitted
mismatch and noise. It initializes physical uncertainty and can further
condition predicted sets. It is not another independent disturbance to inject
after G has propagated those same sources through the observer. Future plant
mismatch nevertheless acts again through the plant dynamics. Describing that
effect is not adding the past state error for a second time.

For an exact linear illustration, let

    x_next=A*x+B*u+P*deltaTheta+w,   y=C*x+n,
    xhat_next=A*xhat+B*u+L*(y-C*xhat),
    z_next=A*z+B*v,                 u=v+K*(xhat-z).

With eta=x-z and epsilon=x-xhat, the joint recursion is

    [eta_next       ]   [A+B*K  -B*K  P] [eta       ]   [I   0] [w]
    [epsilon_next   ] = [0      A-L*C P] [epsilon   ] + [I  -L] [n]
    [deltaTheta_next]   [0      0     I] [deltaTheta]   [0   0].    (M3)

The block notation uses a final zero row in each disturbance column. The
same w and deltaTheta enter both error equations. In the difference
eta_next-epsilon_next their direct contributions cancel, as the observer's
nominal point update requires. Independent copies would destroy this identity.
This is a timing-specific illustration; (M1) handles actual feedthrough.

For a scheduled affine error block r_next=J_i*r+P_i*deltaTheta+D_i*nu+c_i,
preserve the persistent parameter generator by

    H_(i+1)=J_i*H_i+P_i*G_theta,
    Z_(i+1)=[J_i*Z_i,D_i],   m_(i+1)=J_i*m_i+c_i.              (M4)

H and Z are concatenated only when taking supports, with any initial shared
columns preserved. Equivalently propagate theta as a state with identity
transition. Such an augmented matrix has unit parameter eigenvalues even
when its tracking/observer block is stable; a Schur test on the entire
parameter-augmented matrix is not a requirement. A constant parameter may
produce a bounded tracking offset. Neither (M3) nor (M4) bounds the nonlinear
remainder without the construction below.

**Observer compatibility.** Every published enclosure used in (M2) or a
posterior must be valid for the same Theta, residual-disturbance contract and
sensor bounds used in G. Extra lateral force, for example, changes the
force-balance measurement premise as well as the plant. Its force bound must
enter that measurement's interval calculation; adding it only to plant
prediction while keeping an incompatible observer certificate is unsound.
The current 5% tire family can be studied with no extra residual disturbance.
For an added disturbance, certificate validity must first be extended.

While relying on a finite observer certificate, its continuous-motion
premises must hold over each whole hold. Checking the sideslip/domain premise
only at nodes or midpoints does
not prove it between those instants. Collision clearance remains a separate
node/midpoint contract unless a continuous-time clearance image is certified.
In a terminal mode outside that certificate's domain, retain the propagated
physical set and use an unrestrictive Q instead of a finite invalid enclosure.

### Construct sound nonlinear images, then propagate by induction

Define the exact joint image for a fixed causal rule by

    T_pi(J)={G_pi(q,d,nu): q in J, (d,nu) in the declared history set}.

It retains theta and every shared source and is monotone in J. For each
stage construct an outer set R_(i+1) with

    T_(pi_i)(R_i) subset R_(i+1).                              (M5)

The initial containment (M2) and induction on (M5) prove containment of every
allowed nonlinear trajectory for the entire finite horizon. Initial errors
are not restarted at each hold. Widths can vary, and (M5) requires neither
contraction nor an invariant constant radius.

Here is a constructive way to certify (M5). Partition the relevant joint
state/parameter/noise domain into cells. On a convex cell C for the combined
argument a=(q,d,nu), choose a_C and a matrix L_C. If G is continuously
differentiable on C and a validated convex matrix-box enclosure of
DG(C)-L_C is M_C, then

    G(C) subset G(a_C)+L_C*(C-a_C)+M_C*(C-a_C).                 (M6)

**Proof.** Along the segment from a_C to a,
G(a)-G(a_C)=integral_0^1 DG(a_C+t*(a-a_C))*(a-a_C) dt.
Subtract L_C*(a-a_C); the average remaining Jacobian belongs to M_C by its
convexity. An interval hull of its matrix-vector product is a valid reserve.
For coordinate radii rho and entrywise absolute bound Delta on M_C, the
coordinate reserve is Delta*rho. Keep the affine joint generators before
adding this reserve, so shared theta/noise columns remain shared. QED

If component j has a validated Hessian bound |D^2G_j|<=H_j, a tighter smooth
cell reserve is one half rho'*H_j*rho. These bounds are over the entire cell,
including parameter-state and input-state interactions. A numerical Jacobian
at its center or finite differences at two step sizes do not establish them.
Physical/model mismatch belongs in G before linearization; the reserve covers
the error of approximating that uncertain nonlinear G, rather than a second
copy of the same parameter mismatch.

For switching or discontinuous algorithm branches, split the guards and take
the union of images of every possible branch. The Fiala saturation, low-speed
blend, stop reset, angle conventions and force-balance bisection/fallbacks
cannot all be covered by assuming one globally differentiable map. Interval
extensions of the executed primitive operations give an alternative to (M6).
If an exact inverse replaces the implemented bisection for analysis, its
finite termination error must also be enclosed. Numerical plant integration
is the declared map, or its defect from a continuous plant is included once
in the residual contract; those are distinct model choices.

A monotone outer operator can be defined on a fixed covering partition:
union the validated cell images for all cells intersecting J. A subset of J
then selects a subset of cells. Bounds must remain valid for the cell's whole
domain and the fixed policy. This supplies a constructive sufficient operator,
but can be computationally costly or too wide. An adaptive enclosure need not
be globally monotone; the certificate-carrying proof below avoids requiring
every fresh enclosure to be narrower than its predecessor.

For input/handling/collision rows, bound the input image pi(R_i) and the
appropriate held-state images from the same joint cells. A state tube alone
does not certify input reserves. Nonlinear rectangle rows require sound
geometry bounds as well. All these checks are constraints in the mathematical
optimization; the optimized current input is still executed directly.

Every certified cell must lie in a domain where the plant, observer and
policy updates are defined. An undefined branch cannot be silently omitted
from an enclosure. It must be excluded by a valid domain certificate or
handled by an explicitly declared mode/fallback in G.

### Finite-horizon widths with persistent mismatch and nonlinear reserves

On the certified cells, write the error projection as

    r_(i+1)=J_i*r_i+P_i*deltaTheta+D_i*nu_i+c_i+zeta_i,
    zeta_i in the certified remainder set U_i.                 (M7)

Each U_i encloses the actual joint-map residual on R_i, including parameter
interactions. Bounding residuals independently is a permissible outer
approximation; it must not introduce another copy of parameter or sensor
terms already explicitly present in (M7).

Let Phi(i,j)=J_(i-1)...J_j, with Phi(j,j)=I. For independent root generators
G_0 and G_theta, the parameter response and nominal center are

    H_i=sum_(j=0)^(i-1) Phi(i,j+1)*P_j,
    m_i=Phi(i,0)*m_0+sum_(j=0)^(i-1) Phi(i,j+1)*c_j.

For every direction ell, a finite-horizon support bound is

    |ell*m_i| + ||ell*Phi(i,0)*G_0||_1
       + ||ell*H_i*G_theta||_1
       + sum_j ||ell*Phi(i,j+1)*D_j||_1
       + sum_j support_abs(U_j,ell*Phi(i,j+1)).                 (M8)

Here sensor variables are normalized unit boxes, and support_abs(U,a) is
sup_(zeta in U)|a*zeta|. Shared initial/parameter/noise columns are combined
before taking absolute values; (M8) displays the simpler independent-root
case. Any declared temporal noise correlation can also be retained instead
of replaced by independent boxes.

**Proof.** Unroll (M7), collect all occurrences of the same deltaTheta into
H_i*deltaTheta, and take directional supports of the resulting Minkowski
sum. This proves a varying nonlinear outer tube whenever each remainder
bound has been certified on the tube where it is used. Taking absolute
values before collecting the parameter terms loses cross-step cancellation.
No fixed width, cap or zero future noise enters the derivation. QED

The matrices in Phi describe observer and feedback dynamics, not a
frozen-input plant. If their error-block products satisfy
||Phi(i,j)||<=c*rho^(i-j), rho<1, and the parameter/noise/residual forcing is
uniformly bounded, the geometric sum gives a bounded physical-error tail.
Individual steps may expand; persistent parameters still have identity
dynamics. This is a sufficient mechanism, not an imposed constant-radius
objective. For a finite horizon one can verify (M8) and the associated domain
and constraint inclusions step by step without such a uniform inequality.
This error-block stability condition is not a claim that the uncontrolled
target's absolute reachable set is bounded; safety uses joint/relative images.
The current local Schur matrices alone do not establish either a uniform
nonlinear transition-product bound or validated U_i for the actual 5% family.

### Include the actual observer update in the prediction

The proposed joint model should propagate the physical ego/target state X,
the actual observer runtime state s, nominal policy/reference memory z,
observer bound/set state B, and held-input memory. At a future sample:

    estimates_i = reconstruct(s_i, measurements_i, B_i),
    u_i = pi_i(estimates_i, z_i, B_i),
    X_(i+1) = plantHold(X_i,u_i,defects_i),
    s_(i+1),B_(i+1) = observerHoldAndUpdate(s_i,B_i,u_i,sensorHistory_i),
    z_(i+1) = nominalPolicyUpdate(z_i, estimates_i, u_nominal_i).

The sensor history is generated consistently from the same physical branch,
timestamps, held inputs and bounded noise. Propagate all admitted noise/defect
histories. Feedback uses available estimates, never the unknown true state.
The known current s_i is common to all initial branches; uncertainty starts
from the current physical/parameter enclosure, not arbitrary independent
perturbations of the known software state. Future s becomes branch-dependent
through its measurement update.

The present NRMM runtime's numerical point state is 15-dimensional:
body velocity (2), inertial position (2), GNSS predictor (2), target core (6),
radar predictor (2) and yaw (1). Synchronization, orientation sets, history and
bound updates add information memory. `nrmmObserverVectorField("integrate",...)`
supplies the RK4 point flow; `onlineNrmmTrackingRuntime` resets measurement
predictors, updates sets/bounds, then advances held measurements and reconstructs
outputs. The augmented predictor must follow this order and sample/hold contract.
The eight-state continuous core certificate is useful but is not itself the
complete published ego/target-state update.

This is a linear explanatory derivation, not our NRMM observer equation.
For the nonlinear implementation derive the joint map/Jacobian or certified
comparison dynamics from the actual point flow, reconstruction and bound
updates. Validate sound finite-horizon joint images over the admitted nominal,
error and input domains, including midpoints and mode transitions.
Stable observer ISS and the existing path CLF do not by themselves certify
incremental joint contraction. The uncontrolled target's absolute-position
spread remains; relative/coupled safety coordinates are still needed.

Local-map and nonlinear-replay results are recorded separately in
[the actual NRMM prediction report](../report/JOINT_NRMM_PREDICTION_20261013.md).
Those numerical probes do not provide uniform nonlinear remainder bounds.

Including the updater enables a tighter causal description; it does not
automatically guarantee a smaller tube. A poor outer approximation can still
discard its correlations. The full updater can be used for offline proof and
coefficient construction, with compact certified radius/generator recursions
online. Observer states need not all become optimization decision variables.
The shifted-policy proof below applies to this augmented transition when its
outer images are sound and their causal-policy witnesses and certificates
are retained. A monotone outer operator is one sufficient construction.

### Quantify the nonlinear and mode-dependent extension

At admissible nominal straight cruise above the blend/stop regime, sampled Fiala F
has continuous first derivatives locally. Stable A_c admits a contracting
metric. Continuity gives a sufficiently small neighborhood with contractive
eta derivative in (1) and bounded epsilon derivative. The mean-value argument
then gives (3) for compatible sufficiently small observer errors. This proves
local existence, not a quantified tube for the current NRMM bounds or the
entire 5% parameter box. Compute
and certify its domain, nonlinear/decision reserves, input reserve and hold
fractions. Empirical rollout maxima are not universal bounds.

Moving nominal trajectories need scheduled gains/metrics, for example
A_ci'*P_(i+1)*A_ci<=rho^2*P_i plus nonlinear reserves. The current five-state
path CLF omits longitudinal position; it is not automatically an incremental
certificate between two full trajectories. At low speed the model blends,
steering authority changes, and the stop reset switches maps. A positive-speed
contraction proof cannot simply continue through those modes. A certified
braking transition can end in a parked policy that keeps a nonzero pose set
fixed, without chasing noisy velocity estimates.

### Preserve correlation with the uncontrolled target

Two admitted target speeds V+/-delta produce positions separated by 2*delta*t.
Perfect later measurements distinguish the trajectories but do not make them
coincide. A small observer error about each branch's future estimate is not
a small physical reachable set about a single frozen forecast.

Future policies should depend on ego and target observations. Ego trajectories
then depend on the same target/measurement branch as target trajectories.
A target-relative reference or causal policy/tree retains this relationship.
Check relative rectangle clearance on coupled branches: independent marginal
unions can lose correlation and add unnecessary tightening. A target-relative
reference must still obey rate/input conditions; it cannot handle arbitrary
target maneuvers by assertion. Road-free translation-invariant relative
geometry is useful; the soft path-recovery objective can remain. Shared pose
and gyro uncertainty should retain its correlations.

### Braking in inertial-velocity coordinates

There is a concrete correlation already published by this project's
`nrmmControllerErrorBounds`: its yaw generator also changes inferred body
velocity by [v_y; -v_x] times the yaw radius. In the inertial velocity
R(psi) v, this first-order contribution cancels the corresponding rotation
term. A large body lateral-velocity coordinate radius can therefore mostly
represent uncertain coordinates, rather than equally large uncertain
inertial lateral motion. Dropping this column correlation charges both
terms separately. During deceleration the future velocity dependence must
be recomputed by the joint map; copying a high-speed publication to a
low-speed endpoint does not describe the future observer's uncertainty.
This observation does not authorize discarding genuine physical velocity
error or scaling its bound by a guessed speed ratio.

### Coordinate projection and its reserve

Keep inertial velocity in the joint propagation and reconstruct body velocity
at each nominal pose. Let R_i rotate body coordinates to inertial coordinates,
J=[0,-1;1,0], and vB_i=R_i' vI_i. Shared generator columns give

    G_vB,i = R_i' G_vI,i - J vB_i G_psi,i.

This is a coordinate projection of the propagated joint set, not a rule for
shrinking the present observer bound. The yaw column is multiplied by the
future nominal velocity. Its lateral contribution therefore tends to zero
as that velocity tends to zero, even if heading uncertainty does not shrink.
The inertial-velocity residual, measurement noise, parameter mismatch and
tracking uncertainty remain. Apply the same construction separately to the
physical and estimator projections of the joint set. A predicted small
estimation error alone does not bound physical departure from the nominal.

The rotation projection has a simple nonlinear reserve. If |deltaPsi| <= a
and ||deltaVI|| <= b, the norm of the omitted terms is at most

    2 sin(min(pi,a)/2) b + 0.5 ||vI_i|| a^2.

The first term bounds the product of velocity and rotation errors; the
second bounds the rotation Taylor remainder. It is valid for correlated
errors too. Add a sound enclosing ball or box for this reserve while
retaining the shared linear columns. This reserve certifies only this
coordinate transformation, not the preceding observer/vehicle propagation.

The integration sequence is: initialize from the published correlated set;
execute the definite current input; propagate the plant, persistent mismatch,
sensor samples, actual observer updates and future feedback jointly; project
the resulting physical/estimator sets into the next nominal body frame;
then form constraint supports. Future noise is introduced at its actual
sample, not replaced by a fresh copy of the current error box. A prototype
can use maneuver-dependent numerical joint maps offline, but those maps need
validated nonlinear reserves before they supply certified runtime bounds.

The sample-end rest reset is another branch of this joint map. The plant
replay and observer truth samples must apply it at the same control boundary
as prediction. If the admitted set straddles the speed threshold or the
braking/driving boundary, enclose both reset and non-reset images. A tangent
with zero velocity rows on the stopped branch cannot cover the moving
branch. This is compatible with the exact-set recursive-feasibility theorem;
it is an additional requirement for an approximate implementation.

### Optional constant-width diagnostic

Suppose the observer has a predictable envelope B_next<=a_o*B+d_o, a_o<1,
and (1) has a certified weighted-norm bound

    r_next <= rho*r + gamma*B + d_F,     rho<1.               (3)

Fixed widths can satisfy

    B_bar >= max(B_now,d_o/(1-a_o)),
    r_bar >= max(r_now,(gamma*B_bar+d_F)/(1-rho)).              (4)

If the nonlinear reserve depends on the tube/decision domain, solve
rho*r_bar+gamma*B_bar+d_F(r_bar,B_bar)<=r_bar with input/state admission.
This is an invariant-tube condition. No future measurement realization or
vanishing noise is assumed. A radius above its noise floor may decrease;
a radius below the floor can increase. Strict decrease from every arbitrarily
small initial width is therefore not promised. A fixed invariant width is
one possible sufficient design, rather than the user's requirement. Held-input midpoints need their own
envelope; it may vary with hold fraction but must repeat without secular growth.

Koehler, Mueller and Allgoewer (2021), *Robust output feedback model predictive
control using online estimation bounds*, Proposition 2, equations (24)--(26),
and Theorem 4 give a nonlinear framework with observer/tracking bounds and
recursive feasibility. Applying it here requires our own domain, actuator and
terminal certificates. [Primary source](https://arxiv.org/html/2105.03427v1).

<a id="recursive-feasibility"></a>
## 7. Conditional recursive-feasibility proof

This theorem is an existence statement for the joint nonlinear problem. It
assumes initial feasibility, sound uncertainty contracts, causal policy
closure, universal stage constraints and a common terminal witness. It proves
neither initial feasibility of every encounter nor numerical solver success.
The physical truth may differ from the point estimate throughout the proof.

### Information update and universal stage conditions

Let z include the ego state, target state and motion parameters, and the
previous held input. The input memory makes slew bounds state-dependent hard
constraints rather than an unmodeled boundary condition. A finite encounter
includes an absorbing ended flag at the first exit from the ego perception
disk; re-acquisition is a new initial-feasibility problem. Each reachable
branch carries its own flag. The target is removed from an outer set only
after all remaining active branches have exited. This convention is retained
when the policy and its certificates are shifted.

Include fixed uncertain vehicle parameters in z with theta_next=theta. All
images retain that identity. If parameter variation is allowed, use its
declared transition/history at both frames instead; a value box alone does
not establish temporal constancy. For future output feedback augment z with the
actual observer, measurements and policy memory as in Section 6. The published current physical enclosure and parameter
box can initialize a product outer set; this does not justify drawing a new
independent parameter at each future hold. Enclosures Q must cover the same
physical uncertainty contract as T. Declaring a new residual force without
extending an observer certificate that excludes it is not a sound update.

Let T(J,u) be the exact one-hold reachable set from a joint set J under a
common held input u. It includes any *declared* plant-defect set. T_s(J,u)
is the corresponding image at every constrained fraction s of the hold. The
held-input maps and their defect bounds must be the same at consecutive
samples and must cover the declared nonlinear model, not only its tangent.
These maps are monotone:

    J' subset J  =>  T(J',u) subset T(J,u),
                     T_s(J',u) subset T_s(J,u).

The estimator supplies a joint enclosure Q_k containing the truth. Its ego
part may be xhat_k + E_k with E_k obtained from the ISS certificate. Relative
measurements and common ego-pose error can be retained as joint correlations;
independent outer products are permitted if they still contain the truth.
Missing measurements are represented by the full declared domain.

Use the information recursion

    I_0 = Q_0,
    I_(k+1) = T(I_k,u_k) intersect Q_(k+1).                 (1)

Induction proves that the truth belongs to I_k and I_k is nonempty whenever
the model/defect contract and estimator enclosures are sound. It also proves

    I_(k+1) subset T(I_k,u_k).                             (2)

Equation (2) is the cross-sample inclusion needed by the shift proof. The new
estimate need not equal the old predicted center. There is no assumed decay of
future uncertainty and no assumption that successive raw observer balls are
nested. A smaller ball with a shifted center is not necessarily a subset.

The target projection S_k of I_k obeys
S_(k+1) subset phi_h(S_k). The target flow is a semigroup under its rest rule,
so phi_(t-t_(k+1))(S_(k+1)) subset phi_(t-t_k)(S_k) at the same absolute time.
At rest the published acceleration state is reset to zero; propagate before
intersecting rather than intersecting pre-stop and post-stop acceleration
intervals as if that state were an unchanged latent parameter.

### Monotone stage conditions

H(J,u) denotes physical-state, handling, actuator and slew conditions
for every joint state in J and every declared defect, at the declared nodes
and hold midpoints. C(J,u) is the largest nonnegative collision-margin deficit
at those instants. One slack covers a whole hold, including its endpoint; this
fixes the indexing and prevents dropping an endpoint condition during a shift.
When relying on a finite observer enclosure, its continuous operating-domain
premises additionally require whole-hold domain images. Road rows can be
added under the same monotonicity
argument when roads are brought back into the theoretical scope.
For a smaller initial set, the same input has no harder conditions:

    H(J,u) and J' subset J  =>  H(J',u),
    J' subset J  =>  C(J',u) <= C(J,u).                    (3)

These are universal constraints on exact reachable sets. A convex restriction
may represent them only with sound nonlinear and integration remainder bounds.
A fixed-multiplier separating-plane row before yaw linearization is a valid
lower bound. Its first-order yaw tangent without a remainder allowance is not
a general certificate of the requested exact distance. A nonlinear replay of
one estimated center is not verification of the whole set.

### A common terminal family and finite braking certificate

Define a terminal set B of *joint information sets*, not centers. For each
J in B there is one common admissible braking continuation b=(b_0,b_1,...) whose
reachable states remain settled, satisfy every hard condition, and have zero
collision deficit. The allowed backup family is closed under deleting its
first hold. For example, steering and an admissible brake command held through
and after rest have this property on a forward-only model that stays at rest.
The continuation is a witness for the proof, not an emergency command issued
when a solver fails.

This definition gives two properties, without discarding estimation error:

    J' subset J and J in B  =>  J' in B;                   (4)

    J in B => exists b_0:
       H(J,b_0), C(J,b_0)=0, T(J,b_0) in B.                (5)

Proof of (4): the same continuation controls a subset of the old reachable
states. Proof of (5): the tail (b_1,b_2,...) controls T(J,b_0); every reachable
state and constraint there was already included in the old continuation.
Input memory is propagated in T, so the first appended slew condition is not
silently omitted. Conditions (4)--(5) include the parked phase and all admitted
plant defects, not just a nominal braking arc.

A finite certificate of this definition is possible when all reachable ego
states stop by a common finite bound: verify the hard and zero-deficit rows on
the full set until that bound, then verify invariance of the parked ego set and
its clearance against every remaining target forecast. With an exact
forward-only model and no plant defect, a parked pose stays fixed; its position
and heading uncertainty do not vanish, and its rectangle set is checked
against the target family. With persistent defects capable of moving a parked
vehicle, parked-set invariance requires an additional control/defect condition.
A generic bounded defect is not enough. An infinite safe-backup definition is
not, by itself, a completed finite numerical certificate for the Fiala model.

### Finite braking bound under the mild-terminal contract

Under the accepted terminal modeling assumption, each branch holds its
sideslip and course curvature while one stored steering/braking policy slows
the vehicle to a forward-only absorbing rest state. Let every branch have
0<=V<=V_bar initially and realized braking a(t)>=a_min>0 while V>0.
The rate bound must follow from the declared terminal uncertainty family,
not from the nominal command alone. Then

    V(t)<=max(0,V_bar-a_min*t),
    stopTime<=V_bar/a_min,
    remainingArcLength<=V_bar^2/(2*a_min).                    (T1)

Proof: integrate V_dot<=-a_min until its first zero, then use the rest rule;
integrating the triangular upper speed envelope gives the length bound.
The common rule holds its actuator command and uses the physical stop rule.
It does not choose a different last braking input using each unknown true
speed. For a sampled stop reset, include its specified detection delay in
the time/length bound instead of silently applying the continuous formula.

For each mild branch, constant beta and kappa give decreasing
|lateralAcceleration(t)|=|kappa|*V(t)^2 and preserve the mild sideslip domain.
No extra rear-tire nonsaturation condition is required. Actuator bounds are
checked on b_f and stored steering; only the first backup input changes the
old input memory, so its slew bound is checked explicitly and all subsequent
held-command slew differences are zero. Other declared physical and observer
domain rows are verified on their joint images as usual.

Check the full joint continuation to ceil(V_bar/(a_min*h)) holds for the
continuous absorbing-stop contract. Verify terminal actuator/slew/handling
rows, mildness for every branch and zero collision deficit at every declared
constraint instant. Then check the entire parked ego rectangle family against
every remaining allowed target trajectory, with the established encounter
convention. Fixed physical tire-parameter mismatch does not move a parked
vehicle under this contract. A residual disturbance that can do so requires
a separately invariant parked set and cannot be discarded.

If these checks pass, the same braking tail certifies every successor and
every subset, proving (4)--(5) with nonzero pose and speed uncertainty.
No parked yaw-estimate contraction is needed: the parked policy need not
use an uninformative yaw estimate or intersect an invalid finite enclosure.
The exact scalar uncertain-rate specialization is independently checked by
scripts/checkJointMismatchDerivations.py. This is a finite construction of
the certificate, not a completed numerical membership test for the whole
project vehicle/target parameter domain.

### Primary value and horizon convention

Let $N$ be the fixed slack prefix and $N\le M\le M_{\max}$ the prediction
length. Choose causal policies, their joint reachable sets, nonnegative prefix
slacks and a terminal witness. Impose universal hard conditions on every
hold, collision deficit at most $\xi_i$ for $i<N$, zero deficit thereafter,
and endpoint membership in the terminal family. Define

$$
V_{\rm PCBF}^*(I_k)=\min\sum_{i=0}^{N-1}\xi_{i|k}.
$$

The input at the current known information state is definite. The future
policies, not just nominal held inputs, are the variables that can respond to
future observations. Assume attainment only for conclusions about the optimal
value; a feasible executed plan suffices for the feasibility conclusion.

### Shift policies to prove recursive feasibility

Augment the information state with true ego/target state, observer internal
and bound state, previous input and required mode/history. Its transition
includes plant defects, measurement noise, observer update and policy input.
Let T_pi be the exact image under the same causal policy and uncertainty
contract. A certified plan consists of policies, sets R_0,...,R_M, hard-row
certificates, prefix slacks and a terminal witness, with I_k=R_0 and (M5).
Condition the old first successor on the valid new observer enclosure and
the actual known observer/measurement/input memory:

    I_(k+1)=R_1 intersect Q_(k+1) intersect knownMemory_(k+1),
    true q_(k+1) in I_(k+1) subset R_1.                         (6)

Both sets contain the same true augmented state; intersection is therefore
nonempty. At initialization use (M2). If no valid observer enclosure is
available, Q is the unrestrictive set, not an invalid finite ball. Reference
memory of the shifted policy must remain an allowed candidate. An absorbing
encounter-ended flag is propagated with the same convention on both frames.

Hard rows and collision deficits are universal over joint images, including
policy-generated inputs and declared held-input constraint instants. For each
J in a downward-closed terminal family B, there is a causal rule pi_f and
a certified outer successor R_f(J) in B such that T_(pi_f)(J) subset R_f(J),
all hard rows hold on J under that rule, and its collision deficit is zero.

A concrete admissible policy class can contain scheduled affine output laws
u_i=v_i+K_i*(publishedState_i-z_i) together with the held terminal rule.
Nominal centers/reference memory and stored gains remain free to inherit the
old tail; z_0 is not forced to equal the new estimate in the carried candidate.
The initial physical set still uses the current observer information.
Certificates are evaluated on physical joint sets, irrespective of which
nominal center is used to represent them. This class is closed under dropping
the first rule and appending the allowed terminal rule. A narrower policy
class requires its own closure argument.

**Certificate-carrying theorem.** Suppose a feasible certified plan exists
with 1<=N<=M<=M_max; the observer enclosures obey the same uncertainty
contract; the policy/certificate class permits shifting and terminal
appending; and terminal B has the properties just stated. Execute the
definite optimized current input without changing it after the solve.
Then the next mathematical problem is feasible for every admitted parameter,
disturbance and sensor history.

**Proof.** Shift pi_1,...,pi_(M-1) and append a witness pi_f for R_M. Set
R'_0=I_(k+1), retain R'_i=R_(i+1) for 1<=i<=M-1, and take R'_M to be the
certified terminal successor of R_M. For M>=2 the first new image satisfies

    T_(pi_1)(I_(k+1)) subset T_(pi_1)(R_1) subset R_2.

All subsequent old transitions and row certificates are unchanged. The
last hold starts in R_M; its terminal witness certifies the last rows and
R'_M in B. If M=1, the new root is a subset of R_M, and the same terminal
witness applies by downward closure. Input memory in q includes the applied
input, so the shifted and appended slew rows are included. The new plan has
the same M and respects both horizon limits.

Let old prefix slacks be xi_0,...,xi_(N-1). Use xi'_i=xi_(i+1) for
0<=i<N-1 and xi'_(N-1)=0. That last stage was either an old hard-tail stage
or the appended terminal stage. Its candidate budget is therefore the old
budget minus xi_0. If the executed plan is a primary minimizer, comparison
with the next minimum proves V*(I_(k+1))<=V*(I_k)-xi_0*. Feasibility itself
requires only a feasible executed plan. Zero budget implies the specified
collision clearances; positive slacks do not give a zero-deficit safety
claim. QED

This proof retains valid outer sets, rather than demanding that a freshly
computed adaptive approximation be a subset of the previous approximation.
Any refinement can intersect sound images of the same shifted rule with the
retained old tube; the intersection remains sound. A numerical optimization
failure is outside this existence statement.

For a convex implementation the feasible representation must retain this
carried candidate and its valid separating/geometry/domain certificates.
Replacing them by arbitrary new tangents, gain changes or trust regions can
remove the candidate even though the nonlinear robust problem is feasible.
The current production point-estimate programs do not yet implement this
certificate-carrying formulation.

### The second CLF stage preserves mathematical feasibility

Suppose the primary minimum is attained in the certified formulation above.
The second stage retains the same hard rows and certificates, caps the
primary slack sum at that minimum, and imposes a CLF residual g<=rho_CLF with
rho_CLF>=0 and no finite upper bound. For any primary minimizer whose CLF
residual is finite on its certified set, choose

    rho_CLF=max(0,sup g).

This is a feasible second-stage point: an unbounded soft CLF row cannot remove
that primary candidate. A point/nominal CLF residual is the scalar special
case. A hard CLF row, a bounded CLF slack or a new restrictive trust region
needs an additional feasibility proof. Attainment of a secondary optimum is
separate from this existence statement.

Executing a feasible secondary point preserves recursive feasibility by the
theorem. Optimal primary-value descent additionally requires the exact
primary cap; a relaxed cap with numerical tolerance only yields the
corresponding cost-error inequality. Neither conclusion requires a solver
to successfully return a point in every experiment.

### Issue the optimized current input directly

Optimize u_0 and include u_0=v_0+K_0*(xhat_k-z_0) as a program equality. Future
pi_i are policy parameters, rather than promises to execute nominal v_i
unchanged after observations arrive. The completed current u_0 is issued
unchanged. Keep/free the nominal center consistently so the shifted policy
remains an allowed candidate. This requires no post-solve scaling, clipping
or constraint veto.

Outstanding runtime work: consume current/predicted observer enclosures;
certify scheduled full-state/relative feedback and finite-horizon tubes; add policy
and input-margin constraints; retain target/observer branch correlations;
establish robust predicted-endpoint terminal membership; preserve shifted witnesses.
The terminal continuation is a feasibility witness, not a command to stop
nominal driving when the current state belongs to the terminal family.
The research verifier
demonstrates mechanisms, not completion of this runtime.

Independent exact-arithmetic specializations of (M3)--(M6), persistent
parameter supports, posterior/certificate reuse and a common uncertain-rate
braking law are checked by scripts/checkJointMismatchDerivations.py and
tests/jointMismatchDerivationsTest.py. They check the stated algebra and
specializations, not a quantified full-domain NRMM/Fiala certificate.

### Conservative outer sets and convex problems

An implemented reachable-set operator T# may replace T if it contains T and
is monotone under set inclusion. Update I_(k+1)=T#(I_k,u_k) intersect Q_(k+1)
and propagate every prediction with the same operator. The proof above still
works on those outer sets if H,C and B are certified against them. This is a
constructive alternative to merely postulating a tube-growth inequality.

An arbitrary per-frame linearization is not necessarily such an operator. Its
remainder must cover the complete decision and uncertainty domain. Nor does a
convex inner approximation inherit feasibility simply because the exact
problem is feasible. To obtain a theorem for successive convex problems,
show that the shifted witness is also feasible in the next convex restriction
(e.g. retain its valid separating certificates and reserves) and define the
value function for that formulation. Local convex optimality does not prove
global optimality of (7).

For an executed plan with cost J_k, the argument proves only

    V*(I_(k+1)) <= J_k - xi_(0|k).

If J_k <= V*(I_k)+eta_k has actually been certified, add eta_k to the
optimal-value decrease inequality. Summability needs sum eta_k < infinity;
a fixed numerical tolerance does not imply that. The code's lexicographic cap
is a tolerance relative to its convex primary problem, not a certified eta_k
relative to the exact robust problem.

<a id="runtime"></a>
## 8. RTI references, computation and implementation boundary

### Reference construction and horizon search

A previous accepted input sequence is shifted and rolled out from the current
estimate. Its absolute endpoint is retained while the horizon shrinks by one
hold per frame to `controller.horizonSteps`. Encounter preview may request a
longer horizon. A newly observed target requests a fresh avoidance reference.
Every fresh reference with a target uses one safety-oriented circulation
rollout; a target-free cold reference uses nominal path feedback. The field
constructs the reference; only the optimization supplies execution inputs.

Directional rectangle/error supports and relative motion determine outward
obstacle acceleration and circulation in a persistent passing direction.
The free-motion acceleration is zero in world coordinates. The obstacle
field may change both longitudinal and lateral acceleration; cruise-speed
feedback never overwrites its longitudinal component. Neither route
attraction nor cruise recovery shapes this safety initializer. The initial
passing side and geometric encounter preview use the measured ego motion.
Route coordinates remain available for deviation diagnostics and for the
legacy offset-wall geometry when no boundary scan is supplied. Nominal
path and speed tracking remain the second-stage CLF objective.
No offset waypoint, entry/hold/return schedule, trajectory candidate
bank or seed-admission test is prescribed. Inverse Fiala guidance maps the
acceleration to inputs; the declared nonlinear model produces all returned
states over exactly the requested number of holds. A nonterminal or deficient
seed is allowed to enter the terminal-constrained optimizer.

A failed shifted RTI reference first permits fresh guidance at the same
length. Subsequent primal-infeasible fresh attempts request
$M_{\rm next}=\max(M+1,\lceil1.5M\rceil)$, subject to an explicit horizon cap
and the original call deadline. Every reference rebuilds dynamics, geometry,
collision duals, tightening and terminal rows. There is no fixed attempt cap,
no deadline reset and no guarantee that longer horizons restore local
feasibility. Invalid fresh predictions are failures, not executable commands.
A terminal seed is not a prerequisite for searching a longer optimized
horizon.

Missing tail holds from a terminal seed endpoint first try the stored
cruise-feedback hold, then the bounded cruise-to-braking seed search.
These initialization holds are distinct from the terminal law and are never
a solver-failure execution fallback. If no such hold is found, a moving
endpoint uses nominal feedback and a stopped endpoint retains a nonpositive
longitudinal command.

### Perceived boundaries and the independent deviation audit

An oriented boundary chain has drivable space on its left. Only endpoints
marked as perception range cuts extend as tangent rays; an explicit empty
scan supplies no walls. The current scan is frozen throughout a prediction.
For a local signed inward distance $d$, normal $n$, Hessian $H_d$, footprint
support $s_B$ and wall rate $\lambda_w$, virtual acceleration guidance uses

$$
n^\top a\ge-v^\top H_dv-2\lambda_wn^\top v
-\lambda_w^2(d-s_B).
$$

This follows by applying a second-order comparison condition to $d-s_B$
with locally fixed support. A 2D halfplane projection modifies the preferred
acceleration, with a finite least-violation result if the local inequalities
are incompatible. Changing yaw support is not differentiated by this heuristic.
Without a scan, virtual walls at $e_y=\pm W$, default $W=9$ m, bound the
transverse acceleration around the route-curvature term. `Inf` disables that
fallback. Boundary guidance does not certify arbitrary nonconvex-road
invariance under input-limited Fiala dynamics or later RTI corrections.

`controller.enforceRoadBoundaries=false` disables hard road-edge rows by
default. A boundary scan also clears legacy lateral-clearance rows. With the
legacy opt-in, all ego rectangle corners are constrained at predicted nodes
and midpoints, including the endpoint, and along the terminal continuation.
Common absolute-pose error does not cancel from these conditions.

The scenario's `LateralDeviationLimitMeters=10` independently stops truth
replay on an excessive reference-point deviation. It is not an optimizer row,
a footprint buffer, or the planner's 9 m virtual wall. A finite terminal
reference-point deviation bound, if explicitly supplied to its context, is
an additional declared terminal condition; the default experiment threshold
must not be inferred to supply it.

### Proposed implementation of the conditional design

A joint-prediction integration packet needs known observer software state,
gain/model version, sufficient predictor and bound memory, measurement timing
and availability, primitive noise bounds, persistent parameter contract and
joint root generators. The default 50 ms control hold includes four 12.5 ms
sensor updates. Preserve their actual order, shared gyro feedthrough,
force-balance reconstruction, angle conventions and stop branches.
A current six-state radius alone cannot reconstruct that updater.

Offline scheduled gains and validated nonlinear map/remainder bounds can
support bounded generator or directional-support propagation online. Observer
states need not all become optimizer decision variables. The known-current-
input root map and future feedback maps remain distinct. Physical, estimate,
relative-pose, feedback-input and held-state projections come from the same
joint set. No radius is reduced merely to keep it inside a validation table.

Next-frame posterior conditioning must retain the old certified policy/set
witness if a fresh approximation is wider. A newly chosen guide or gain cannot
silently exclude that witness. The terminal endpoint certificate must cover
one common continuation for every branch; separately feasible pointwise
stopping inputs are insufficient. The remaining realization work is to provide
validated maneuver-wide images, feedback-input margins, robust terminal
membership and carried cross-frame certificates.

### Computation budget and diagnostics

The current research budget is 100 ms (`solver.timeLimitSeconds=0.1`), with
at most 40 native interior-point iterations per stage and a 2 ms completion
reserve. The model sample remains 50 ms; these offline simulations do not
inject computation latency into actuation. Preprocessing
counts against the same timer as model construction and both solves. Reference
rollout, tangent/constraint construction, CLF construction, terminal geometry and
optimized-input prediction check the timer. A native termination callback includes
MEX validation and problem construction, in addition to Clarabel's iteration
timer. Native infeasibility certificates and unfinished iterates are never inputs.

Replay exports distinguish the model-sample deadline (`deadlineMisses`) from
the configured research budget (`controllerBudgetSeconds` and
`controllerBudgetExceededCalls`).

These checks bound the admitted work, not a hard wall-clock guarantee on a desktop
MATLAB process. A running factorization, allocation, initial JIT compilation or OS
preemption cannot be interrupted by the checks. Warm up before timed execution;
`collisionAvoidanceController("warmUp", cfg, road)` uses a separate startup
budget for cold avoidance planning, cruise and bounded low-speed feedback
with curved targets. The optional road also primes its actual curvature,
terminal and avoidance geometry. Every probe command is discarded.
The integrated scenario driver warms after observer initialization, including
one discarded bounded-input startup probe. It does not reuse that probe's
solution, consume the first observation or advance its random stream. These
startup costs are reported separately; first-measurement-to-actuation latency
is not established by this benchmark. Observer publication also avoids repeatedly
reordering MATLAB's search path. Actual
successful and failed controller call times must both be measured from the
same controller timer. Observer/publication preparation belongs to the
separate pipeline timer; failed pipeline latency is not solver latency. A timeout is an allowed
failure, not a successful real-time control result. Raising the budget for a
functional test does not change the bounded RTI/replanning policy.

`search.rtiIterations` and `linearizationCount` count all reference attempts;
`solverCalls` includes their native solves (at most two per model). `attempts`
preserves each result and its `horizonSteps`; `horizonExtensions` counts length increases;
`selectedAttempt` identifies the returned solution, or zero when none is returned.
`nominalRestarted` indicates reference recovery and `restartCount` counts
additional RTI models. Initialization and formulation timings sum all attempts. `convexSolveCompleted`
records completion of both convex objectives on the final reference.
`optimizationConverged` is false because nonlinear convergence is not tested.
`nonlinearValidationPerformed` is false and `nonlinearHardResidual` is unavailable.
The reported PCBF and CLF quantities belong to the affine subproblem;
`predictedState` is a nonlinear prediction of the unchanged optimized inputs.
Independent scenario replay measures clearance, road margins and nominal
recovery throughout the observation. Current-state terminal membership does
not stop simulation or count as task completion. The terminal set certifies
a possible safe continuation at the predicted endpoint; the controller keeps
executing its optimized first input on each frame. Campaign recovery requires
a sustained dwell near the prescribed path and speed after the encounter.
An optional finite observation cap is a diagnostic setting; by default
`Frames=Inf` runs until recovery or failure without a simulation-time limit.
A finite counterfactual baseline qualifies the initial encounter independently
of the closed-loop duration. Resumed observations retain their original
baseline and minimum recovery time.

### Current limits at a glance

| Required mathematical ingredient | Current implementation boundary |
| --- | --- |
| Sound current physical enclosures | Finite published enclosures are used when available; experimental execution may proceed without a supported finite certificate. |
| Nonlinear, feedback-aware horizon images | Future holds restart current-width affine sets; full nonlinear remainders and future observer histories are not certified. |
| Cross-sample posterior inclusion | Joint posterior intersection and retained policy/set witnesses are not implemented. |
| Common robust terminal family | Center-based target geometry and ego padding under the mild terminal model; no full nonlinear parameter-family membership certificate. |
| Exact nonlinear CLF dissipation | First-successor affine error/estimate row with soft slack and a fixed reference penalty. |
| Whole-hold collision/observer-domain validity | Nodes and approximate midpoints do not by themselves establish continuous-time validity. |
| Numerical success | Both objectives must complete; timeout, infeasibility or unfinished solve returns no command. |

<a id="source-map"></a>
## 9. Source map and verification

The controller has eight MATLAB source files in this directory, one MATLAB
configuration entry point and one precomputed JSON CLF table. The module-count
test limits executable source growth; this consolidation adds no runtime module.

| Source | Responsibility |
| --- | --- |
| [collisionAvoidanceController.m](collisionAvoidanceController.m) | Input/target memory, encounter updates, solver orchestration, first-input output, reset and warm-up actions. |
| [readControllerInputs.m](readControllerInputs.m) | Ego/target parsing, timestamped error sets, given path and boundary observations. |
| [solvePredictiveControl.m](solvePredictiveControl.m) | References, affine RTI models, two objectives, adaptive horizon search, tightening, shared timer and discrepancy diagnostics. |
| [terminalSafeSet.m](terminalSafeSet.m) | Joint state/flow, braking continuation, membership/linearized rows, parked-track checks, seed holds and offline certificate helpers. |
| [nonlinearBicycleModel.m](nonlinearBicycleModel.m) | Blended bicycle, road load, RK4/tangents, trim, path error and quadratic CLF. |
| [modifiedFialaTire.m](modifiedFialaTire.m) | Combined-slip forces and analytic derivatives. |
| [predictiveSafetyGeometry.m](predictiveSafetyGeometry.m) | Target flow, uncertainty supports, rectangle duals, circulation/boundary guidance and interval geometry helpers. |
| [laneGeometry.m](laneGeometry.m) | Straight/circular path charts. |
| [collisionAvoidanceControllerConfig.m](../config/collisionAvoidanceControllerConfig.m) | Defaults, merging and validation. |
| [clfMatrices.json](../config/clfMatrices.json) | Offline matrices, gains and recorded enclosure data from `scripts/synthesizeClfMatrices.m`; read-only at controller runtime. |

Together with the configuration entry point, the executable MATLAB count is
nine. Local equation labels such as (1) and (T1) are interpreted within their
derivation section; labels (M1)--(M8) refer to the joint-map construction in
Section 6. The same letter may denote a different local variable in another
derivation; the physical six-state vector is always identified explicitly.

From the repository root, run all MATLAB tests with

```matlab
results = runtests('tests'); assertSuccess(results)
```

Behavior coverage includes `twoStagePredictiveControlTest`,
`ordinaryDistanceDualTest`, `observerPredictiveControlTest`,
`terminalSafeSetTest`, `nominalClfTest`, `unrestrictedPredictiveControlTest`,
`clfNominalRecoveryTest`, `nominalRecoveryValidationTest`,
`lateralDeviationLimitTest`, `predictiveConicSolverTest` and kernel-equivalence
tests. These test algorithm behavior and finite numerical cases; passing them
does not establish a universal nonlinear certificate.

Independent algebra checks are available through

```bash
python scripts/checkJointMismatchDerivations.py
python -m unittest discover -s tests -p jointMismatchDerivationsTest.py
```

They verify exact-arithmetic specializations of shared mismatch/noise,
persistent-parameter support, nonlinear reserves, certificate reuse and a
common uncertain-rate braking rule. Research probes
`scripts/verifyJointNrmmPrediction.m`, `scripts/verifyNrmmRelativeFeedback.m`
and `scripts/verifyFeedbackAwarePrediction.m` measure local maps and sampled
nonlinear behavior separately from uniform validity. Observer initialization
failures remain failed campaign cases; exact-state diagnostics do not replace
joint estimated-state runs. Independent fixed-parameter truth replay audits
collision, road occupancy, reference-point deviation and sustained path/speed
recovery. Terminal membership does not stop the driving experiment.

MATLAB Control System Toolbox and the compiled Clarabel adapter are runtime
dependencies. `scripts/buildPredictiveConicSolver.m` builds the native adapter;
`scripts/buildControllerKernels.m` optionally builds MATLAB Coder kernels.
Optimization Toolbox supports independent comparisons. CLF synthesis uses
YALMIP/SeDuMi offline. Native binaries and `solver/` dependencies are outside
this theory consolidation.

<a id="sources"></a>
## 10. Sources and document migration

The derivations above specialize the repository's declared equations. Related
primary sources establish the general PCBF, output-feedback MPC and
high-order barrier constructions; they do not certify this NRMM/Fiala
implementation:

- Huang, Wang, Margellos and Goulart (2025), *Predictive Control Barrier
  Functions: Bridging model predictive control and control barrier functions*:
  [primary manuscript](https://arxiv.org/html/2502.08400v1), Section III and
  Lemma III.5, for the predictive slack and shifted-plan argument.
- Koehler, Mueller and Allgoewer (2021), *Robust output feedback model
  predictive control using online estimation bounds*:
  [primary manuscript](https://arxiv.org/html/2105.03427v1), Sections III--IV,
  for separate estimation/tracking bounds and robust output-feedback MPC.
- Xiao and Belta (2019), *Control Barrier Functions for Systems with High
  Relative Degree*: [primary manuscript](https://arxiv.org/abs/1903.04706),
  for high-order barrier constructions related to virtual wall guidance.

The former documents map to the following current sections. Their earlier
versions remain in Git history, including details specific to old experiments.

| Former controller Markdown | Current location |
| --- | --- |
| `CONTROLLER_FILES.md` | Section 9, source map; Section 8, runtime scope. |
| `PCBF_CLF_ARCHITECTURE.md` | Sections 1, 3 and 8. |
| `NOMINAL_CLF.md` | Section 4. |
| `TERMINAL_SAFE_SET.md` | Section 5. |
| `CLOSED_LOOP_PREDICTION_THEORY.md` | Sections 6--8. |
| `ROBUST_RECURSIVE_FEASIBILITY.md` | Section 7 and the common terminal certificate in Section 5. |

[OBSERVER_ROBUST_PCBF_THEORY.tex](OBSERVER_ROBUST_PCBF_THEORY.tex) is retained
as the dated October 3--5 derivation source, including superseded terminal
and objective designs. Its useful comparison-system, relative-geometry,
remainder and CLF-supply arguments are incorporated above. Its historical
implementation claims do not override this document. Experiment reports and
the manuscript remain separate artifacts with their own validation records.
