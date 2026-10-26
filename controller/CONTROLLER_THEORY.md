# Controller mathematical theory

This is the current controller theory and source map. It consolidates the six
former controller Markdown documents into one derivation, with a single
statement of the implementation boundary. Repository paths are relative to the
repository root unless linked otherwise. Historical experiment reports describe
the versions that produced them; they are not rewritten by this consolidation.

The controller seeks rectangle nonoverlap during an encounter and resumes
prescribed-path cruise recovery afterward. Two lexicographic stages minimize
predictive collision deficits, then deviation from the current trajectory
reference. Only outside an active encounter does the secondary stage also
include a soft quadratic cruise CLF.
The terminal set certifies an available safe continuation at the prediction
endpoint; it is not a command to park or a task-completion condition.

The executable approximation and the mathematical algorithm have distinct guarantees:

| Object | Prediction and conclusion |
| --- | --- |
| Executable controller | At most one local RTI model per controller call, nonlinear Fiala tire dynamics, per-hold affine uncertainty tightening and terminal tangents. A continuous-time derivative soft CLF row is enabled only outside an active encounter. The completed affine update returns its first input unchanged, without a post-solve nonlinear rollout or feasibility veto. Planning occurs only at encounter onset; later frames shift the previous state/input iterate and append one hold. The first accepted encounter plan fixes its horizon. Failure never requests a longer-horizon retry. |
| Unified observer-to-safety design | Current observer containment, retained physical successor sets, signed whole-hold margins and a common terminal continuation. The information-set shift proves conditional recursive feasibility and zero-deficit safety. Future observer dynamics are needed only for the optional prediction of observation-dependent policies. |

The latter theorem is not a certificate for the former approximation.
`robustNonlinearSafetyCertified` remains false. Positive collision slack permits
a clearance deficit. Zero affine slack does not certify nonlinear or
continuous-time safety. Road constraints are disabled by default; the ideal
information-set proof below also omits road boundaries. Optional road rows require their
own compatible images and terminal checks.

## Contents

1. [Vehicle model and target prediction](#vehicle-model)
2. [Observer bounds and relative geometry](#observer-geometry)
3. [Collision constraints and the two-stage problem](#optimization)
4. [One nominal CLF and recovery](#nominal-clf)
5. [Joint terminal set and braking continuation](#terminal-set)
6. [Optional joint feedback-aware prediction](#joint-prediction)
7. [Unified observer-to-safety formulation](#recursive-feasibility)
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

For a reference $(\bar x_i,\bar u_i)$, variational tangents give

$$
\Delta x_0=0,\qquad
\Delta x_{i+1}=A_i\Delta x_i+B_i\Delta u_i+e_i,
\qquad e_i=F_h(\bar x_i,\bar u_i)-\bar x_{i+1}.
$$

The onset planner supplies a nonlinear rollout, so its defects vanish. Later
frames shift the previous affine state/input iterate, update the first state
from the current estimate, and append a tail hold. Their defects need not
vanish and are retained in the equality right-hand side. The inherited input
prefix is not reintegrated from the new measurement before solving.

The current input estimate is fixed; current state error is handled by the
published enclosure, not a free optimizer correction. Fixed collision duals
and terminal tangents are built once on this model. There are at most two
native objective solves per reference. The completed update supplies the first
optimized input directly. Its predicted states and PCBF slacks remain those
of the affine subproblem; no post-solve nonlinear rollout replaces or rejects
them. The initial planner rollout and stagewise variational integrations belong to
preparation, before solving. Independent experiment replay evaluates the issued
controls on the plant. There is no auxiliary feasibility restoration problem,
damped-step search or adaptive reference penalty.

### Implemented constraints and objective units

State and input increments have no artificial search bounds. Actual longitudinal-force
magnitude and optional slew limits remain hard. Steering has no artificial
actuator magnitude or slew limit; the Fiala angle domain is enforced. State
limits, sideslip, optional road-corner rows and terminal
rows retain their existing meanings. No shared elastic variable relaxes the
problem whose controls are issued.

Nominal collision rows in the first `controller.horizonSteps` nodes carry
nonnegative PCBF slacks. With observer uncertainty, every tightened collision
node also carries a nonnegative robustness-deficit slack; the nominal collision
row remains hard after the prefix. These explicit collision deficits are
separate from physical state, road, handling and terminal constraints.

The primary stage minimizes `sum(xi)`. If the zero correction with zero slacks
satisfies every primary constraint, it attains the global lower bound and no
native primary solve is needed. Otherwise the native conic solver runs once.
The secondary stage caps the attained primary objective, with the configured
numerical tie. During an active target encounter it minimizes only reference
deviation. Outside an encounter it also uses the soft continuous-time CLF row

    Vdot_affine(xhat_now,u0) <= -alpha * V(xhat_now) + clfSlack,
    alpha = 2/clf.convergenceTimeConstantSeconds.

`V` is the single precomputed transverse quadratic at prescribed-speed/path
trim. Only the estimate enters it; it does not ask the worst member of an
observer enclosure to contract to zero. The secondary objective outside an
encounter is

    J2 = clfSlack + (lambda / (2 N)) sum_k (
         ||Dx (x_aff,k+1 - xbar_k+1)||^2 + ||Du (u_k - ubar_k)||^2).

While `model.target` is nonempty, the CLF row and its slack cost are absent:
`J2` consists only of the reference-deviation term. A zero PCBF slack does not
reactivate cruise tracking. A currently out-of-range target or a complete
current sensor scan clearing the retained forecast ends the encounter and
restores the CLF. Missing observations without such clearance retain the
encounter and leave the CLF disabled. The reserved CLF variable slot is fixed
to zero in this case; it has no inequality row or objective coefficient.

Here xbar and ubar are this RTI reference: the encounter-onset planner rollout,
the shifted previous state/input iterate, or an extension of that same reference. They are not the nominal cruise path.
`lambda = rti.referenceDeviationWeight` defaults to 10 and remains active at
zero cruise error; zero is an explicit ablation. Dx uses equal x/y position
units from `clf.lateralPositionErrorScale`, followed by the CLF heading,
speed, lateral-velocity and yaw-rate scales. Du uses the CLF certification
steering/braking units. These are cost units, never bounds. The horizon mean
keeps the overall weight independent of node count. The solver divides the
entire objective by `max(1,V(xhat_now))` for numerical scaling when the CLF is
active; the avoidance-only objective uses unit scaling.

The old whole-horizon attraction to the cruise route is removed from this
objective. Cruise recovery resumes in the CLF after the encounter. Outside an
encounter, larger lambda permits more CLF slack to retain the reference, but cannot relax the
primary PCBF cap or other constraints. This changes performance preference,
not the feasible set or the conditional shift-candidate feasibility argument.
It does not guarantee small nonlinear prediction error or successful RTI.
Both stages must complete; an unfinished secondary stage
does not cause execution of the primary point.

During avoidance, the secondary stage adds only the primary cap and a soft
objective, so the primary point is still feasible. Outside avoidance, the
soft CLF cannot destroy mathematical feasibility of a feasible primary
point. Let $z_1$ be any feasible first-stage point retained under the primary
objective cap, and let $\widehat{\dot V}(z_1)$ be its finite affine CLF
derivative. Keeping its states, inputs and PCBF slacks unchanged, choose

$$
 s_V=\max\{0,\widehat{\dot V}(z_1)+(2/T)V(\hat x)\}.
$$

This satisfies the secondary CLF row because $s_V$ has no upper bound. All
common constraints and the primary cap remain satisfied. The construction
holds for positive as well as zero PCBF slack. A finite quadratic proximity
cost changes which feasible point is preferred, not whether a point exists.
Numerical solver failure is separate from this feasible-set inclusion and
does not prove that the soft CLF made the mathematical second-stage problem
infeasible. The primary cap is enforced only between the two affine stages;
it is not imposed on an independently recomputed nonlinear slack sum.

Modified Fiala dynamics include combined-slip saturation at both axles.
The optimizer uses their local derivatives without additional tire-capacity
cones or friction polygons. A tangent can overestimate the nonlinear force
away from its expansion point; this approximation error does not indicate a
missing capacity limit in Fiala. The unchanged optimized controls are rolled
through the nonlinear model, and the resulting trajectory must pass the
sampled primary-constraint check before execution. Removing the extra cones
does not make an arbitrarily large one-step correction accurate.
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
reserves 2 ms for completion. Each controller call builds at most one RTI
model and performs at most two native objective solves. Initial encounter
horizon sizing precedes this model, and later encounter calls retain that
length; primal infeasibility, numerical failure and nonlinear rejection
are reported without increasing the node count or rebuilding the problem.
No auxiliary restoration solve is run.

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

The performance design uses the continuous-time nonlinear bicycle system:

    Vdot(x,u) <= -alpha V(x) + s,   alpha = 2/T,   s >= 0,

where T = `clf.convergenceTimeConstantSeconds` = 4 s. The slack has units of
CLF value per second and has no upper bound. Exact continuous enforcement
with zero slack implies `V(t) <= exp(-alpha t) V(0)` and hence a time constant
T for sqrt(V). The online implementation evaluates the derivative at the
current estimate and linearizes its input dependence once, as described
below. It does not impose or claim discrete one-step contraction.

### Continuous-time CLF matrix and separate sampled cruise certificate

`scripts/synthesizeClfMatrices.m` computes P and K before experiments, for
each operating point (vehicle parameters, speed, path curvature, sample time
and CLF settings), and writes them with their key and certificate to
`config/clfMatrices.json`. A controller call at an operating point without an
entry is rejected with `collisionAvoidanceController:missingClfMatrix`, an
entry without the certificate with `collisionAvoidanceController:staleClfMatrix`;
nothing is synthesized at run time, and the run time does not need the SDP
solver.

The matrix and gain satisfy the continuous trim Lyapunov inequality

    Ac_cl = Ac + Bc K,
    Ac_cl' P + P Ac_cl <= -alpha P.

The synthesis imposes this as `Ac S + Bc Y + (Ac S + Bc Y)' + alpha S <= 0`,
with `S=P^-1` and `Y=K S`, in physical coordinates (scaled in code).
The design uses a slightly faster rate via `DesignMargin` and rechecks a
strict margin at the requested rate after synthesis.
`continuousLinearDecayRate` records the minimum decay rate found by the
normalized symmetric eigenvalue check. All six retained table entries pass
strictly; their existing P and K values therefore needed no numerical change.
Strictness and smoothness imply continuous nonlinear decrease on a sufficiently
small neighborhood of the trim, with admissible feedback `u=u_ref+K e`.
This argument does not identify that neighborhood with the sampled certificate's
whole `V<=1` set and does not establish a global CLF.

Separately, the held cruise initializer keeps its sampled-model certificate.
Its purpose is to validate a possible appended cruise hold, not to define the
online CLF row. This additional synthesis condition targets the sampled
nonlinear model through a ray-average
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

If the exact continuous nonlinear derivative inequality holds with zero slack
throughout a suitable invariant domain, it gives exponential decay of the transverse error with time constant T. Zero slack in the
**affine approximation**, together with a fixed model-error tolerance, does not
establish that exact inequality or global asymptotic convergence.

### One affine continuous-derivative constraint

This constraint and its slack objective are enabled only outside an active
target encounter. The same quadratic value may be reported during avoidance
as a diagnostic, without influencing the optimization.

At the current estimate xhat, the analytic error Jacobian gives

    g = 2 e(xhat)' P J_e(xhat),
    d0 = g f(xhat,ubar_0),
    d_u = g f_u(xhat,ubar_0).

The exact model derivative is `Vdot(xhat,u)=g f(xhat,u)`. Fiala dynamics are
nonlinear in steering and longitudinal force ratio, so the RTI approximation is

    d0 + d_u * du_0 <= -alpha V(xhat) + s.

This is a single affine inequality. There is no next-state quadratic epigraph
and no CLF second-order cone. Together with the affine primary rows and the
positive-semidefinite quadratic reference penalty, the second stage is a QP.
The first stage is a PCBF slack LP (a QP with zero Hessian). Solver interior
iterations still occur inside each stage; the nonlinear model is built once.

Only the first input appears in this CLF row. The current state is fixed, not
a decision variable. No Taylor approximation of V itself is needed for this
row, and no quadratic remainder of a next-state value is dropped. The exact
Fiala input nonlinearity still limits the accuracy of the local derivative
model for large input corrections.

Outside an encounter the secondary objective is CLF slack plus half
`rti.referenceDeviationWeight` times the horizon mean of squared, unit-scaled
state and input deviations from the current linearization reference. The
default weight is 10 in CLF-rate objective units, independent of cruise error and reference source; zero
disables the penalty for ablation. Position uses the same CLF lateral-position
unit for both inertial coordinates. Yaw, longitudinal speed, lateral velocity
and yaw rate use the corresponding CLF error units. Inputs use the CLF
certification steering and braking units. The entire objective is divided by
`max(1,V(xhat_now))` in the solver. During an encounter only the reference
penalty is optimized, using unit scaling. Reported component costs are unscaled.

The primary PCBF slack cap remains hard. There is no state/input correction
box and no separate whole-horizon cost pulling toward the cruise path. The
penalty is zero at the current reference, which may be a bypass trajectory.
It remains active near nominal motion to regularize otherwise free later
inputs. When active, the CLF refers to the prescribed cruise path. A larger reference
weight can trade increased CLF slack for smaller trajectory changes; it does
not certify nonlinear CLF decrease or prevent every large correction.

With an observer enclosure the row is unchanged and acts on the estimate:
`clfDerivativeLimit = -alpha*V(xhat)`. Estimation error enters the performance
analysis as an input, rather than as a worst-case CLF enclosure budget.
`clfDerivative` is the local affine prediction. The online update does not
evaluate `clfActualDerivative` or `clfActualDerivativeResidual`; both are
reported as NaN. Independent simulations may evaluate the actual derivative.
`clfNextValue` is the value on the affine predicted first successor, labeled
by `clfNextValueScope`; it is not the left side of the optimized CLF condition.
The scenario driver records `actualNextClfValue` on its independent plant
successor. `clfSatisfied`
means zero local CLF slack within solver tolerance, not exact nonlinear decay.
During an encounter `clfEnabled` and `clfSatisfied` are false, `clfFunction`
and `clfLocalModel` are `none`, and the uncomputed slack and derivative fields
are NaN. `secondaryObjective=referenceDeviation` distinguishes this from a
zero-slack cruise solve. `secondaryStageCompleted` records QP completion
independently of the disabled `clfStageCompleted` flag.

The nonlinear input-linearization remainder and sampling effects are not enclosed.
Neither zero affine slack nor this implementation proves exact asymptotic
recovery of the nonlinear uncertain plant; with estimation error the closed
loop can at best approach a neighborhood of the path set by that error. See the conditional theorem in
`OBSERVER_ROBUST_PCBF_THEORY.tex` for the additional premises.

This constraint controls the current continuous derivative through the first input only. Future inputs are chosen
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
beta_f = 0.1 rad and a_m = 2 m/s^2 by default. The braking ratio must satisfy
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

For the unified observer-controller design, replace a point by a nonempty
information set J containing physical ego and target states, shared model
parameters, input memory, and the encounter flag. Add observer state only
when a predicted policy uses future observations. The terminal family is

    B_f = { J : one common causal terminal policy satisfies all hard rows
                on every admissible branch until that branch's first exit }.

The flag is branch-specific: a center outside range does not certify that
all possible targets have exited. A computed outer set may discard the
target only after every remaining active branch has exited. An unknown
parameter remains the same along each branch. Model mismatch already
represented in the current observer enclosure is not added again as an
independent initial error. Future physical model defects remain in the
physical images; the optional future-feedback extension propagates their
shared effects on observer and controller as well.

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
## 6. Optional joint feedback-aware prediction

This extends the physical information-set core of Section 7 to predictions
whose inputs depend on future observations. It is distinct from the runtime's
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
## 7. Unified observer-to-safety formulation

The core algorithm uses one deterministic information set. The observer
certifies the current physical state; the controller propagates that set under
candidate held inputs and conditions the retained successor on the next valid
observation. This closes the estimation-to-safety argument without forecasting
future observer convergence. Predicting future observation-dependent policies
is an optional extension, described at the end of this section and in Section 6.
The two PCBF/CLF priorities and the physical vehicle model are retained.

This formulation specializes the predictive-slack shift argument to sound
observer information. It is not a new claim of general MPC novelty, and it does
not convert the present RTI approximation into a nonlinear safety certificate.

### Current containment is the observer interface

Let q contain the true ego and target states, persistent uncertain parameters,
previous applied input and the mode/encounter memory needed to make the physical
transition Markov. Fixed parameters remain fixed within each branch. Any
additional plant defect has its own declared history; it must also be covered
by the observer certificate when it changes the measurements or observer model.

At time k the observer supplies a set Q_k with

$$
q_k\in Q_k.                                                     \tag{U1}
$$

Its physical projection can use the transient comparison and correlated
relative enclosures of Section 2. No covariance, RMS error or energy-gain
inequality is substituted for pointwise containment. The initial transient,
held measurements, numerical defects, outages and applicable true operating
domain must be covered. A steady-state radius alone does not establish (U1).

The controller need not know the observer's internal state in the core
formulation: future inputs are common held commands. Past estimation error
initializes Q_k. It is not injected again as an independent physical disturbance
at every horizon step. Future physical mismatch still acts through the physical
model. Measurement noise enters new Q sets and the resulting reoptimization;
it does not move the plant under a fixed held command.

### Propagate first, then condition on observations

Let T(J,u) be the exact physical one-hold image and T_tau(J,u) its image at
elapsed time tau in [0,h]. These maps include the declared plant/target modes,
parameter histories and defects. They are monotone under set inclusion.
A certified outer successor R satisfies T(J,u) subset R; ordinary local
affine prediction without a remainder is not such a certificate.

Initialize I_0=Q_0, with compatible known input and mode memory. Store the first
certified successor R_(1|k) when issuing u_k. At the next sample use

$$
I_{k+1}=R_{1|k}\cap Q_{k+1}\cap A_{k+1},\qquad
q_{k+1}\in I_{k+1}\subseteq R_{1|k},                              \tag{U2}
$$

where A fixes newly known input/mode memory if it is not already fixed in Q.
Both sets contain the same truth, so the intersection is nonempty. Raw
observer balls may move and need not be nested. A dropout contributes the
unrestrictive observation set; the physical prediction remains. An empty
intersection is a violated containment/model contract, not permission to
replace the set by the latest point estimate. A genuinely new target or model
contract starts a new initial-feasibility problem.

The posterior may be represented by an outer set only if that representation
still contains the truth and lies inside the retained successor. Arbitrary
reboxing or recentering must not destroy this inclusion.

### One signed margin connects geometry and prediction

Choose a signed geometric margin g(q) whose nonnegative values imply the
required rectangle nonoverlap. A fixed separating-plane certificate from
Section 3 is sufficient. Ordinary distance, which is zero even inside an
intersection, cannot establish nonoverlap at the default zero design clearance.
Define the robust whole-hold margin and deficit by

$$
\begin{aligned}
m(J,u)&=\inf_{0\le\tau\le h}\ \inf_{q\in T_\tau(J,u)} g(q),\\
C(J,u)&=\max\{0,-m(J,u)\}.                                      \tag{U3}
\end{aligned}
$$

Let H(J,u) mean that all physical, handling, actuator, slew and required
observer-domain conditions hold on the same hold images. Therefore

$$
J'\subseteq J\ \Longrightarrow\
\begin{cases}
C(J',u)\le C(J,u),\\
H(J,u)\Longrightarrow H(J',u).
\end{cases}                                                       \tag{U4}
$$

An implementation may certify a lower bound on m using retained
separating normals, supports and validated reserves. For a local row
$g(q)\ge g(\hat q)+a^Te-r$ on its entire error/decision domain,

$$
g(q)\ge g(\hat q)-\sigma_E(-a)-r,\qquad e\in E.                  \tag{U5}
$$

This is the observer-to-safety interface: the estimator controls the set E,
and geometry determines the direction a in which its support matters.
Common ego/target pose and parameter columns are kept before taking support.
Orientation, nonlinear dynamics and numerical error enter r only to the
extent that the image or exact support has not already included them.
No uncertainty source is added twice.

Whole-hold images give continuous-time clearance directly. Alternatively,
valid start/midpoint/end bounds with a true whole-hold Lipschitz constant L_g
can use the reserve L_g*h/4, plus the midpoint/integration defects. Without
such an extension, (U3) must explicitly be restricted to the checked instants,
and the theorem below guarantees only those instants. The executable RTI
currently has that limitation. Prediction also avoids requiring that a
position-only first-order CBF have direct steering/braking authority in the
dynamic bicycle model; no artificial relative-degree-one assumption is added.

### A terminal family with one common continuation

Let B be a downward-closed family of nonempty physical information sets. Each
J in B must admit a common admissible first terminal command u_f and a
certified outer successor R_f in B such that

$$
H(J,u_f),\quad C(J,u_f)=0,\quad T(J,u_f)\subseteq R_f\in B.         \tag{U6}
$$

A common braking continuation and its allowed tails supply this property.
Individually feasible braking inputs for different unknown states do not.
Input memory is part of q, so the first appended slew condition remains.
For the mild terminal contract of Section 5, a lower realized braking bound
$a_{\min}>0$ gives

$$
t_{\rm stop}\le\overline V/a_{\min},\qquad
s_{\rm stop}\le\overline V^2/(2a_{\min}).                         \tag{U7}
$$

Verify the complete uncertain continuation to this time and the parked
rectangle family afterward. A continuous absorbing stop is assumed here;
a delayed stop reset needs its own allowance. Nonzero pose uncertainty remains
at rest. No yaw-estimate contraction is required when the terminal command
uses no yaw measurement. Defects capable of moving a parked vehicle require
a separately invariant parked set. The mild continuation is a declared model
contract, not a proven maneuver-wide Fiala terminal certificate.

### The same two optimization priorities

Fix 1<=N<=M at encounter initialization. Choose held inputs u_0,...,u_(M-1),
nonempty sets R_0,...,R_M, compatible image/row certificates and nonnegative
prefix slacks. The admissible plans satisfy

$$
\begin{aligned}
R_0&=I_k,& T(R_i,u_i)&\subseteq R_{i+1},\\
H(R_i,u_i)&,& C(R_i,u_i)&\le\xi_i,\\
\xi_i&=0\quad(i\ge N),& R_M&\in B.                             \tag{U8}
\end{aligned}
$$

One slack covers each entire hold including its endpoint. Define the
information-state predictive barrier value

$$
V_{\rm P}^*(I_k)=\min\sum_{i=0}^{N-1}\xi_i.                     \tag{U9}
$$

After attaining the primary minimum, the secondary stage retains every hard
row and that primary budget, and minimizes the soft CLF/performance objective.
For a finite CLF residual r_V, a free nonnegative slack s_V=max(0,r_V)
retains any primary candidate. This establishes feasibility, not convergence
of the CLF or existence of a numerical solver result. Section 4 gives the
additional dissipation and recovery-domain premises for convergence.

The feasible representation must allow the old input tail, its retained sets,
row certificates and terminal append. Fresh anchors, arbitrary trust boxes or
new separating charts cannot discard this witness. An exact physical image
is monotone, but an arbitrary adaptive outer approximation need not be;
retaining the old outer certificates avoids requiring that extra property.

### Unified containment and safety theorem

Assume (U1), a sound initial feasible plan, the same physical uncertainty
contract throughout the encounter, sound images/rows in (U8), the conditioning
rule (U2), and terminal closure (U6). Assume the plan/certificate class permits
shifting and terminal appending. Issue the definite first input of a feasible
plan unchanged.

1. The next information set contains the truth and the next mathematical
   optimization problem is feasible.
2. The executed hold satisfies the hard conditions and g(q(t))>=-xi_0
   at every instant included in (U3).
3. If each executed plan attains the primary minimum, then

   $$
   V_{\rm P}^*(I_{k+1})\le V_{\rm P}^*(I_k)-\xi_{0|k}^*.        \tag{U10}
   $$

   Consequently the zero-value information sets are invariant and safe.
   Along an infinite applicable sequence, the first-hold deficits are
   summable and tend to zero. Positive deficits are relaxation; summability
   does not imply finite-time entry into the zero-value set.

**Proof.** Truth containment and I_(k+1) subset R_(1|k) follow from (U2).
Shift u_1,...,u_(M-1), retain the old successor sets and their row certificates,
and append a terminal command/witness from R_M. By monotonicity, the shifted
first image of the new root lies in the old R_2. All later old images and rows
remain unchanged. The terminal append satisfies (U6). If M=1, the new root
is a subset of R_M, so the same terminal command applies by downward closure.
Shift prefix slacks to (xi_1,...,xi_(N-1),0); the final zero was either an old
hard-tail hold or the terminal append. This is a feasible candidate of cost
sum(xi_i)-xi_0. Comparing the next minimum with that candidate proves (U10)
when the old plan attained its minimum. The true first hold belongs to the
certified images, giving assertion 2. Nonnegative values telescope; initial
zero implies all subsequent deficits and values are zero. QED.

With a merely feasible executed plan of cost J_k the valid comparison is
V_P^*(I_(k+1))<=J_k-xi_0. A certified primary suboptimality eta_k adds eta_k
to (U10); summability then needs a summable error budget. The executable
numerical primary cap is not a certified gap to this nonlinear optimum.

### Online algorithm and the optional feedback extension

The core algorithm has five operations:

1. Obtain the current sound observer enclosure and condition the retained
   first successor using (U2); initialize directly only for a new problem.
2. Propagate candidate physical sets and enforce (U3), (U6) and (U8).
3. Minimize the predictive deficit (U9), then optimize performance under the
   same safety budget and common hard conditions.
4. Issue the optimized first command unchanged and store its certified first
   successor, remaining input sequence, sets and terminal witness.
5. Repeat after the next observation. Missing information does not reset the
   prediction width or create an assumed future posterior contraction.

This controller is output feedback: every optimization uses the information
conditioned on the available measurements. Its predicted input sequences are
open loop within each solve. That restriction can be conservative, especially
for long horizons and large model uncertainty; initial robust feasibility is
still a premise, not a universal outcome for every safe-looking encounter.

When the prediction explicitly uses future measurements, replace q by the
joint physical/observer state of Section 6 and replace u_i by a causal rule
pi_i. The exact transition includes the real observer updates, shared noise,
input timing and reconstruction. The same definitions (U2)--(U10) and the same
shift proof apply to that lifted state, provided the policy class is closed
under shifting. No second safety theorem is needed. Only this extension needs
future observer/measurement propagation; current observer containment suffices
for the physical held-input core. Neither representation permits restarting
current-width error sets at future nominal centers without a sound image.

### Executable research specialization

`scripts/verifyUnifiedObserverSafety.m` implements the core with longitudinal
forward-only motion, a constant-speed moving target, absorbing ego rest,
bounded braking and slew, and current bounded-observation boxes. It exhausts
a finite input alphabet, computes the signed minimum gap analytically over
each whole hold, and appends a common braking continuation. A retained box
operator is monotone; no future observer dynamics or assumed measurements
enter the search. Boxes can lose correlations and remain conservative.

The checks include nonnested observations, dropout, positive-deficit recovery,
a one-hold horizon, a parked uncertain state, input-limited infeasibility,
empty-intersection rejection and interior overlap at zero design clearance.
A constructed trajectory has a 4 mm positive gap at start/midpoint/end but a
1 mm overlap between them, which the analytic hold minimum detects.
The tests also distinguish quadratic energy dissipation from pointwise
Euclidean observer containment. These are deterministic analytic-model
checks in ordinary floating point, not a universal NRMM/Fiala certificate.
The production RTI remains the approximation described in Section 8.

<a id="runtime"></a>
## 8. RTI references, computation and implementation boundary

### Reference construction and pre-solve horizon sizing

Encounter preview selects the first reference length. Once a plan is
accepted, its number of input holds $M$ fixes the horizon for that encounter.
Every later call drops the executed input, shifts the remaining $M-1$ inputs
and their state iterate, replaces the first state with the current estimate,
and appends one tail hold. Stagewise dynamics defects enter the SQP equations. Thus the
relative duration $M T_s$ stays fixed and the absolute endpoint advances by
one sample. Updated target forecasts and observer bounds do not trigger
another horizon preview, shorten the plan or add more prediction holds.

The accepted input sequence stores this length directly; state and metadata
also expose `encounterHorizonSteps` (zero when no encounter is active).
A failed solve publishes no new controller state and cannot change the
accepted horizon. A target-free continuation may shrink toward
`controller.horizonSteps`. Once a confirmed exit or complete empty scan
clears the encounter, a subsequent acquisition selects its own new horizon
independently of the previous encounter's length.

The planner is invoked once on the first observation of each encounter,
including a cold start with an observed target. A target-free cold reference
uses nominal path feedback. Later observations update target prediction and
uncertainty, but retain the shifted optimized input sequence. A missing or
invalid input warm start during an active encounter is reported as failure;
it cannot silently trigger planning. After a confirmed target exit or complete
empty scan clears the encounter, a subsequent acquisition starts a new
encounter and permits one new planner invocation.

The field constructs the initial reference; only the optimization supplies
execution inputs. Previously optimized state/input prefixes are shifted
without full-horizon replay or reshaping by the boundary guidance field. Tail construction retains the
existing terminal/nominal seed policies. The tangent initializer's handling
of singular force-ratio endpoints remains as described above.

Directional rectangle/error supports and relative motion determine outward
obstacle acceleration and circulation in a persistent passing direction.
The preferred free-motion acceleration follows the nominal path and speed.
Writing lateral displacement as $e$, course relative to the path as $\chi$,
and lookahead as $D$, the desired course is $\chi_d=-\arctan(e/D)$.
The course-rate request combines path-curvature feedforward, $\dot\chi_d$,
and the existing proportional course-error feedback. Multiplying it by speed
gives the transverse acceleration; the existing speed gain supplies the
longitudinal acceleration. This provides path attraction without waypoints
or a prescribed return time.

The initializer first rolls out this route/boundary policy through the actual
vehicle model. If its uncertainty-inflated rectangle gaps are strictly positive
at all active prediction nodes, it returns that rollout unchanged. Otherwise,
it makes one field rollout with the route acceleration as its preferred input.
The obstacle field may change both longitudinal and lateral acceleration;
cruise-speed feedback never overwrites its longitudinal component afterwards.
Road guidance follows this correction. This is a nominal-path conflict check
inside initialization, not a bypass of any RTI collision or terminal constraint
and not a robust feasibility certificate. Both passes use the same requested
horizon and the existing encounter/perception convention. Only one trajectory
is returned, without scoring alternative maneuvers. A longer preview that first
reveals a conflict may therefore change the initial field activation.

The initial passing side and geometric encounter preview use measured ego
motion. Route coordinates also supply deviation diagnostics and legacy offset
walls when no boundary scan exists. During an encounter the controller's
second stage still penalizes departure from its RTI reference without a CLF;
the cruise CLF resumes outside the encounter. Route attraction here belongs
to the initializer, not to a newly imposed optimization constraint.
No offset waypoint, entry/hold/return schedule, trajectory candidate
bank or seed-admission test is prescribed. The initial circulation speed uses
the current encounter's contour scale, not a turn radius equal to the ego
footprint radius. This avoids demanding a slow, tight orbit from the beginning
of a distant encounter. Inverse Fiala guidance maps the acceleration to
inputs. Its avoidance allocation reserves the requested lateral fraction at
both axles, then uses the remaining combined-slip capacity for braking or
acceleration; the cruise helper's 0.35 force-ratio cap does not apply. A bounded
scalar inversion includes the rotation of front longitudinal force with
steering. This is a heuristic input allocation, not an affine-input barrier
certificate. The declared nonlinear model produces all returned
states over exactly the requested number of holds. A nonterminal or deficient
seed is allowed to enter the terminal-constrained optimizer.

Both objective stages share the one model and the encounter's fixed node
count. The same-length shift matches the horizon convention of Section 7;
it does not establish the joint-set and terminal certificates needed by
that conditional theorem. The existing numerical tail initializer remains
a preparation heuristic. Primal infeasibility,
numerical failure or an unrollable reference returns
failure; none requests a longer horizon, another local model or a new
avoidance path. This does not certify that the selected horizon is sufficient
or that a feasible first frame always exists. The terminal conditions remain
in the optimization and are not relaxed to accept a short horizon.

The offline free-duration experiment follows the same planning gate and
compares fixed and variable duration on one reference with one node count.
Optimizing total duration changes the existing tail intervals; failure does
not trigger a second comparison on an enlarged grid. Explicit offline
refinement of that fixed grid is separate from the online RTI controller.

### Nonlinear rollout checks

After both objectives solve, the unchanged input sequence is rolled through
the nonlinear vehicle. The primary rows are evaluated again at those states:
actual yaw in the same collision normals, the held-input posterior images,
state and actuator limits, and the endpoint's terminal continuation. The
minimum required collision slacks are recomputed. Acceptance requires the
hard residual to be below `solver.feasibilityTolerance` and this nonlinear
slack sum to stay below the affine primary cap, within that tolerance.
The check uses the original constraints and observer uncertainty margins;
there are no measured-discrepancy reserves or adaptive secondary weights.
This numerical acceptance condition does not establish cross-frame posterior
inclusion or global nonlinear optimality.

The check covers the implemented nodes and approximate midpoints only. The
observer's future enclosure assumptions, nonlinear uncertainty remainder and
terminal modeling assumptions remain the implementation boundaries stated
below. A passed numerical check must not be relabeled a robust nonlinear
recursive-safety certificate.

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

The core in Section 7 first needs a retained physical posterior, sound
whole-hold nonlinear images, signed geometry margins and a common uncertain
terminal witness. It does not need to predict future observer software state.
The executable longitudinal reference validates this algorithm on its stated
model; transferring it to Fiala dynamics requires those vehicle-specific
images and certificates.

For the optional prediction of observation-dependent feedback, a
joint-prediction integration packet needs known observer software state,
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
rollout, tangent/constraint construction, CLF construction and terminal geometry
check the timer. A native termination callback includes
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
functional test does not change the encounter-onset planning policy.

`search.rtiIterations` and `linearizationCount` are at most one per controller
call; `solverCalls` is at most two. `attempts` contains the single result and
its `horizonSteps`; `horizonExtensions` and `restartCount` remain zero.
`selectedAttempt` is one for a returned solution and zero otherwise.
`plannerCalls` counts planner invocations: at most one on an onset call and
zero on subsequent calls in the same encounter. `nominalRestarted` stays false.
`convexSolveCompleted` records completion of both convex objectives.
`optimizationConverged` is false because nonlinear convergence is not tested.
`nonlinearValidationPerformed`, `nonlinearPredictionEvaluated` and
`postSolveFeasibilityCheck` are false. `nonlinearHardResidual` and
`linearizationGapMeters` are NaN rather than uncomputed zero residuals.
The reported PCBF value, stage slacks and `affineHardResidual` describe the
affine subproblem. `search.slackCap` only preserves its primary objective
in the second stage; it is not a nonlinear acceptance cap. When no encounter
is active, the CLF objective belongs to the input-affine approximation of the
current continuous derivative. An active encounter disables that objective
and its inequality, while retaining the reference-deviation QP.
`predictedState` is the affine SQP iterate (`predictionModel=affineSqpIterate`).
Compatibility rollout metadata has `available=false` and
`reason=notPerformedByRti`; it does not signal a failed rollout.
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
| Sound physical horizon images; joint images for future feedback | Future holds restart current-width affine sets; full nonlinear remainders and future observer histories are not certified. |
| Cross-sample posterior inclusion | The production RTI does not implement retained posterior intersections or input/policy and set witnesses; the longitudinal research specialization does. |
| Common robust terminal family | Center-based target geometry and ego padding under the mild terminal model; no full nonlinear parameter-family membership certificate. |
| Exact nonlinear CLF dissipation | Current continuous derivative, locally affine in the first input, with soft slack and a fixed reference penalty. |
| Whole-hold collision/observer-domain validity | Nodes and approximate midpoints do not by themselves establish continuous-time validity. |
| Numerical success | Both affine objectives must complete. The optimized first input is returned without a nonlinear feasibility veto; initial planning or inherited-reference optimization may still fail. Nonlinear feasibility or safety does not follow from local solver success alone. |

<a id="source-map"></a>
## 9. Source map and verification

The controller has eight MATLAB source files in this directory, one MATLAB
configuration entry point and one precomputed JSON CLF table. The module-count
test limits executable source growth; this consolidation adds no runtime module.

| Source | Responsibility |
| --- | --- |
| [collisionAvoidanceController.m](collisionAvoidanceController.m) | Input/target memory, encounter updates, solver orchestration, first-input output, reset and warm-up actions. |
| [readControllerInputs.m](readControllerInputs.m) | Ego/target parsing, timestamped error sets, given path and boundary observations. |
| [solvePredictiveControl.m](solvePredictiveControl.m) | References, affine RTI models, two objectives, encounter-onset horizon sizing and fixed-length shifts, tightening, shared timer and discrepancy diagnostics. |
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

The unified physical-information algorithm and its regression checks run with

```matlab
addpath('scripts'); result = verifyUnifiedObserverSafety();
results = runtests('tests/unifiedObserverSafetyTest.m'); assertSuccess(results)
```

Independent algebra checks for the optional joint extension are available through

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
- Tezuka and Nakamura (2020), *Time-varying Obstacle Avoidance by Using
  High-gain Observer and Input-to-State Constraint Safe Control Barrier Function*:
  [publication](https://doi.org/10.1016/j.ifacol.2021.04.192), Sections 4--6,
  for treating estimated obstacle velocity error inside the barrier dynamics.
- Quan, Zhou, Frisk and Chung (2025), *Observer-Based Environment Robust
  Control Barrier Functions for Safety-Critical Control With Dynamic Obstacles*:
  [publication](https://doi.org/10.1109/LCSYS.2025.3581497), Section III, for
  distinct environment-state and velocity-error compensation. Their general
  bounded-error interface motivates (U1); an energy-gain LMI alone is not
  adopted as a pointwise Euclidean radius.
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
