# Controller mathematical theory

This is the current controller theory and source map. It consolidates the six
former controller Markdown documents into one derivation, with a single
statement of the implementation boundary. Repository paths are relative to the
repository root unless linked otherwise. Historical experiment reports describe
the versions that produced them; they are not rewritten by this consolidation.

The controller seeks rectangle nonoverlap during an encounter and resumes
prescribed-path cruise recovery afterward. An active encounter minimizes
predictive collision deficits first, then minimizes nominal tracking and input
variation while preserving that attained optimum. Without a target, one QP
minimizes soft cruise CLF slack plus the same performance cost. Both modes use
state-dependent numerical trust bounds; reference proximity is optional and off
by default. Avoidance uses at most two convex solves on
one RTI model and one shared time budget; an available zero-slack affine
candidate avoids the primary LP. No cross-frame slack budget is imposed.
The terminal set certifies an available safe continuation at the prediction
endpoint; it is not a command to park or a task-completion condition.

The executable approximation and the mathematical algorithm have distinct guarantees:

| Object | Prediction and conclusion |
| --- | --- |
| Executable controller | At most one local RTI model per controller call, nonlinear Fiala tire dynamics, per-hold affine uncertainty tightening and terminal tangents. A continuous-time derivative soft CLF row is enabled only outside an active encounter. The completed affine update returns its first input after numerical input-bound normalization, without a post-solve nonlinear rollout or feasibility veto. Planning occurs only at encounter onset; later frames shift the previous state/input iterate and append one hold. The first accepted encounter plan fixes its horizon. Failure never requests a longer-horizon retry. |
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
3. [Collision constraints and the switched problem](#optimization)
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

The body sideslip $\beta_E=\operatorname{atan2}(v_y,v_x)$ is the angle of
the center-of-gravity velocity, not either tire slip angle $\alpha_f,\alpha_r$.
In low-speed turning, $v_y\simeq l_r r$ can give nonzero body sideslip while
the rear tire slip is near zero. Consequently, a large ratio $|v_y|/v_x$ at
low speed does not by itself establish tire sliding or physical infeasibility.
The shared observer cone is an estimation-domain premise. Enlarging it changes
that premise and the recomputed error bounds, rather than increasing tire
capacity. The low-speed blend and its response lag remain model assumptions,
not an experimental validation of the corresponding real-vehicle motion.

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
## 3. Collision constraints and the switched problem

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
RTI leaves position translations unbounded, bounds local heading corrections
through a numerical dynamics trust estimate, and does not implement this complete row;
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

### One RTI model and target-presence objective switching

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
and terminal tangents are built once on this model. Avoidance uses at most two
native objective solves on that same reference; cruise uses one.
The completed update supplies the first
optimized input, with the numerical bound normalization described below.
Its predicted states and PCBF slacks remain those of the affine subproblem;
no post-solve nonlinear rollout replaces or rejects
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

While `model.target` is nonempty, the first objective is `sum(xi)`. No CLF
row or CLF slack cost participates. A zero-slack affine candidate is checked
before solving: first zero correction, then state corrections obtained from
the same affine dynamics with zero input correction. A feasible candidate
attains the safety lower bound without an LP. This check uses floating-point
roundoff tolerance and does not integrate or certify the nonlinear model.
Otherwise one LP minimizes the safety sum.

A second QP chooses performance among the attained minimum-slack solutions.
Its shared objective is

    J_perf = J_track + J_change + J_ref,
    J_track = w_track/(2 N) sum_k ||De e_aff,k+1||^2,
    J_change = w_change/2 sum_k ||Du (u_k - u_k-1)||^2,
    J_ref = w_ref/(2 N) sum_k (
        ||Dx (x_aff,k+1 - xbar_k+1)||^2 + ||Du (u_k - ubar_k)||^2).

Here e contains lateral position, heading, longitudinal speed, lateral
velocity and yaw-rate errors relative to nominal path trim. Its affine
expansion is evaluated at each reference state; squaring that residual gives
a positive-semidefinite Gauss--Newton objective. The reference xbar/ubar is
the RTI linearization trajectory and is distinct from the nominal path.
The first input difference uses the previously applied command, including
actuator memory supplied with the estimate. J_change is a sum rather than a
horizon mean, so extending N does not cheapen the first command jump.

The refinement retains exactly the same constraints and dynamics and adds
`sum(xi) <= Sstar`, where Sstar is the first stage's attained safety value.
When Sstar is zero, the slacks are fixed to zero. No intentional slack
allowance or safety/performance weighted tradeoff is used. A refinement must
preserve the cap within `solver.constraintTolerance * max(1,Sstar)`; this is
numerical priority, not exact arithmetic. Nominal speed tracking is only a
secondary preference: braking remains allowed when needed for safety. No
CLF inequality is activated during avoidance.

Both stages reuse one RTI model and share the call budget. A zero performance
cost attains its global lower bound without a second native solve. If
refinement fails, exceeds the budget, violates input bounds or exceeds the
safety cap, the completed priority candidate is retained. No third solve,
replanning or nonlinear execution filter is introduced. Positive PCBF slack
remains admissible; state, input, hard-tail and terminal conditions retain
their declared hardness. Soft regularization does not impose a steering-rate
bound or certify nonlinear model accuracy.

Without an active target, collision slacks are fixed to zero and no PCBF LP
is solved. The continuous CLF design is unchanged:

    Vdot <= -alpha V + s,  alpha = 2/clf.convergenceTimeConstantSeconds.

Integrating this condition over a held-input interval h with constant slack s
would imply

    V(x+) <= exp(-alpha h) V(xhat) + tau s,
    tau = (1 - exp(-alpha h))/alpha.

The sole cruise QP imposes the affine tangent of this integrated condition
at the first predicted reference state and minimizes `tau*s + J_perf`.
It has no quadratic CLF constraint or second-order cone. The unbounded
nonnegative slack absorbs any finite affine CLF residual at an otherwise
feasible point. This approximated sampled condition does not establish the
continuous inequality inside the hold, nonlinear descent, or solver success.

The first in-range target observation activates PCBF and the onset planner.
Its first accepted horizon stays fixed throughout the encounter. Zero PCBF
slack does not reactivate CLF. An out-of-range target or a complete sensor
scan clearing the retained forecast restores CLF; missing observations alone
retain PCBF. A new target still requires a feasible initial encounter
problem. No common Lyapunov function or global switched-safety theorem is
asserted. After clearance, cruise references are rebuilt from the current
estimate before the one RTI linearization.

The weights `rti.nominalTrackingWeight`, `rti.inputVariationWeight` and
`rti.referenceDeviationWeight` default to 1, 20 and 0. Zero disables the
corresponding component; all three zero disable avoidance refinement.
Dx uses equal x/y units from `clf.lateralPositionErrorScale`, followed by
heading, speed, lateral-velocity and yaw-rate scales. De uses those five
transverse error units; Du uses the CLF certification steering/braking units.
These are cost units, not bounds. Tracking and reference costs are horizon
means. The cruise objective is scaled by `max(1,V(xhat))` for the solver;
reported component costs are unscaled.

Each call returns the selected optimization's first input, apart from
tolerance-sized physical input-bound normalization. There is
no rejection of the priority solution due solely to refinement failure,
no PCBF-versus-CLF weighted sum, and no inherited
slack budget. The ideal encounter problem still minimizes its safety slack;
its conditional shift argument remains separate from the executable affine
RTI approximation.

Modified Fiala dynamics include combined-slip saturation at both axles.
The optimizer uses their local derivatives without additional tire-capacity
cones or friction polygons. A tangent can overestimate the nonlinear force
away from its expansion point; this approximation error does not indicate a
missing capacity limit in Fiala. Independent experiments roll the issued
controls through the nonlinear plant; there is no online post-solve rollout
or admission check. Removing the extra cones does not make an arbitrarily
large one-step correction accurate.
The force-balance observer encloses saturated forces too; its interval can
widen when the measurement becomes less informative. Its separate sideslip
premise `|vy| <= tan(model.sideslipMaximum)*vx` remains enforced, with observer
support margins. Removing that premise would require a new observer enclosure.
The precomputed local cruise certificate may use a conservative nonsaturated
neighborhood; that local certificate does not impose a full-horizon cone.

### State-dependent numerical trust regions

The default reference-proximity weight is zero. Nominal-path/speed tracking and
actual input-increment regularization remain active. Before either objective
solve, each hold receives a box around its dynamics linearization point in
`q = [psi, vx, vy, r, delta, b]`. Global x/y translations are exactly affine in
the transition and receive no trust bound. The box is frozen for both PCBF
stages, and is also used by the single cruise QP. It adds only variable bounds;
no extra cone, tire-capacity constraint, SQP iteration, replanning call,
post-solve rejection, or horizon extension is introduced.

Let F be the discrete held-input map, J its reference Jacobian, S a diagonal
coordinate scaling, and epsilon the six-component one-hold error budget.
Central differences of the discrete Jacobian estimate the scaled Hessians

    H_l = S (d^2 F_l / dq^2) S / epsilon_l.

The estimate is symmetrized. With

    w_i = max_l sum_j |H_l,ij|,
    s_i = min(1, sqrt(2 / (6 w_i))),
    radius_i = S_ii s_i,

the quadratic surrogate is at most one on the box. This follows from
`2 |d_i d_j| <= d_i^2 + d_j^2` and the six-coordinate allocation of its
quadratic budget. The scales are `[0.3 rad, max(2,0.25|vx|) m/s, 1 m/s,
0.4 rad/s, 0.25 rad, 0.5]`. They cap the extent of the initial local search;
they are numerical design choices, not physical steering or sideslip limits.
The resulting radii vary with speed, slip, yaw rate, steering and combined-slip
loading. Mixed derivatives account for interactions such as steering and
braking rather than treating their independent bounds as independent forces.

Because local curvature can miss a saturation transition, the implementation
also evaluates the exact discrete map at both ends of each coordinate axis and
four mixed-sign box directions. The actual remainder `F(q+d)-F(q)-J*d` is
compared componentwise with epsilon. A failed probe shrinks all radii by
`max(0.1,min(0.8,0.8/sqrt(ratio)))`; at most six probe passes are permitted.
Failure to find a passing neighborhood is reported, not silently accepted.
These are local preparation evaluations, not additional trajectory optimizer
iterations. The same Fiala transition, including its low-speed branch, is used.

The default budget is `[0.04 m, 0.04 m, 0.01 rad, 0.4 m/s, 0.4 m/s,
0.2 rad/s]` per hold. It controls numerical approximation locality; it neither
replaces nor adds another copy of the observer/model-mismatch error tube.
Radii are intersected with existing physical/model-domain bounds. The two
PCBF objectives share this intersection from the outset: the primary optimum
may increase relative to an unrestricted affine problem, and performance
still cannot buy any further PCBF slack. A CLF slack cannot relax a hard
terminal condition or a trust bound.

This is a **numerical locality estimate**, not a uniform nonlinear certificate:
a finite Hessian stencil and finite displacement probes do not bound the
remainder at every interior point. It also does not bound accumulation across
the entire horizon, certify the separate collision/terminal tangents, or
provide the accept/reject convergence theory of a full trust-region SQP
algorithm. No such guarantee is inferred from passing simulations.

Adding these hard numerical bounds restricts the executable local problem.
A shifted nonlinear witness from the ideal recursive-feasibility theorem is
not automatically inside the next box. The shifted multiple-shooting iterate
can have dynamics defects after measurement updates. They remain explicit in
the equalities; consistency does not require replacing the shifted trajectory
by an open-loop nonlinear rollout.

Before solving, forward interval propagation intersects the reachable
correction intervals of these affine equalities with the current variable
bounds, starting with the correction fixed by the initial-state equality.
An empty intersection detects an inconsistency between the dynamics and
boxes. A nonempty intersection is only a necessary condition, not a
feasibility certificate for the complete problem.

When this check finds an empty intersection, compute a particular affine
consistency correction with unchanged inputs:

$$
d_0=\hat x_k-\bar x_0,\qquad
d_{i+1}=A_i d_i+e_i,\qquad \Delta u_i=0.
$$

This forward sensitivity pass uses the existing tangents. It adds no
optimizer call, replanning, nonlinear trajectory rollout or SQP iteration.
Only at the next state whose incoming correction interval misses its box,
reserve the corresponding nonlinear-state radii and recompute that box with
the **same error budget**. Other stages retain their ordinary radii; a large
later open-loop sensitivity alone does not cause an unrelated enlargement.
The interval test is repeated after reallocation. Repairs advance through
the horizon, stop at an unresolved physical bound, and never exceed the
number of stages.
With $r_{\min}=(|d_{i,3:6}|,0,0)$, initialization uses
$r=\max(r_{\rm ordinary},r_{\min})$ and failed probes reduce only excess room:

$$
r\leftarrow r_{\min}+\gamma(r-r_{\min}),\qquad 0<\gamma<1.
$$

The reference Hessian seeds the reallocated box but does not veto it: its
absolute quadratic envelope can overpredict finite displacement errors across
tire saturation or low-speed model transitions. For a required correction,
the reallocated box is checked using the actual discrete-map remainder at all
64 mixed corners and both endpoints of each of the six axes. The componentwise
error budget is unchanged. A needed state correction can therefore be
accommodated by reducing other state/input freedom without also requiring the
pointwise curvature surrogate to remain below the budget. If no candidate box
passes these finite checks, preparation reports failure. This remains a
sampled numerical locality estimate, not an interior or uniform error bound.
Physical bounds are retained throughout. The
shifted reference, general equality defects and optimization objectives stay
unchanged, and an ordinarily compatible box is retained exactly.

This particular correction certifies only the affine dynamics equations;
it need not satisfy physical, collision or terminal constraints. If the
complete hard constraints cannot be satisfied inside the final domain, RTI
reports failure. Applying the ideal recursive proof to this restricted online
algorithm would additionally require an admissible shifted witness inside
every next trust domain. The conditional information-set theorem elsewhere
in this document remains a statement about its stated exact problem.

Physical input bounds include force ratios -1 and +1. After solving, input
values outside their physical bounds by at most `solver.feasibilityTolerance`
are projected to the exact bounds. Larger residuals reject the solver result
with `pcbfInputBoundsViolation` or `clfInputBoundsViolation`. In-bounds values
are not changed. The issued command, stored input sequence and input
corrections use the same normalized values. Affine states and slacks are not
recomputed; their constraint residual is evaluated with the normalized input.
`inputBoundCorrection` reports the largest correction, and
`postSolveInputModification` records whether any value changed. This numerical
interface does not relax the Fiala domain or certify nonlinear feasibility.
Only the tangent reference
uses `nonlinearBicycleModel.inputBounds(cfg,true)`, which keeps its force
ratio ten constraint tolerances inside the singular endpoints. Shifting a
saturated previous plan rebuilds this interior seed and its complete rollout;
this does not change a previously issued command. The 100 ms controller budget
reserves 2 ms for completion. Each controller call builds at most one RTI
model and performs at most two native objective solves during avoidance,
or one during cruise. Initial encounter
horizon sizing precedes this model, and later encounter calls retain that
length; primal infeasibility and numerical failure
are reported without increasing the node count or rebuilding the problem.
No auxiliary restoration solve is run.

The executable affine collision deficits must not
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

### One affine integrated continuous-CLF constraint

The CLF inequality and its slack cost act only outside an active encounter.
The continuous quadratic V=e'Pe and its offline design are retained. At the
first predicted reference state xbar_1, define

    Vbar_1 = e(xbar_1)' P e(xbar_1),
    g_1 = 2 e(xbar_1)' P J_e(xbar_1),
    Vaff_1 = Vbar_1 + g_1 dx_1,
    tau = (1 - exp(-alpha h))/alpha.

The integrated condition above becomes one affine row

    (Vaff_1 - V(xhat))/tau <= -alpha V(xhat) + s.

Its state increment dx_1 is coupled to the first input by the same affine
held-input dynamics used by RTI. This incorporates the duration of the
applied command instead of requesting descent only at its starting instant.
It adds neither another model linearization nor a post-solve nonlinear
rollout. Together with the shared positive-semidefinite performance Hessian,
cruise remains a QP. Native solver iterations are internal to that one QP.

This is a tangent approximation of the endpoint value. It is not an upper
bound: quadratic value terms, curved-path coordinate remainders and the
nonlinear dynamics remainder are not enclosed. Consequently, zero local
slack is not a guarantee of exact nonlinear held-step decay. Even the exact
endpoint inequality alone would not imply the continuous derivative condition
at all intermediate times. The continuous design, sampled implementation
and empirical closed-loop recovery must be distinguished.

Both modes use J_perf from the switched-optimization section. It gives the
controller a persistent route/speed preference and penalizes actual control
increments, alongside the reference regularizer that favors a useful local
linearization. It introduces no road-boundary constraint, hard correction
box or new tire-capacity constraint. A finite weight can still admit a large
correction. PCBF retains lexicographic priority; the new performance terms
cannot intentionally increase its attained optimum. The feasible-set shift
argument of the ideal PCBF problem is therefore unchanged by this tie-breaker.

The observer enclosure does not alter the performance row: it acts on the
estimate. The compatibility field `clfDerivative` now reports
`(Vaff_1-V(xhat))/tau`, the affine integrated-rate surrogate, and
`clfDerivativeLimit=-alpha*V(xhat)`. The explicit label
`clfLocalModel=integratedContinuousClfAffineInNextState` distinguishes it
from an instantaneous model derivative. `clfActualDerivative` and its
residual remain NaN online. `clfNextValue` evaluates V on the affine first
successor; it need not equal the tangent Vaff_1. The independent scenario
driver records `actualNextClfValue` on the plant successor.

`clfSatisfied` denotes zero affine slack within solver tolerance. During
an encounter CLF fields are disabled or NaN, and `optimizationMode=pcbf`
with `optimizationObjective=pcbfSlack`. Its second stage is labeled
`nominalTrackingAndRegularization`, with the attained safety cap recorded.
Cruise uses `integratedClfSlackAndTracking` in one QP and has no PCBF cap.

Neither the affine constraint nor the regularized objective proves exact
nonlinear or uncertain-plant convergence. The observer error, validity of
local approximations and conditional safety assumptions remain separate;
see `OBSERVER_ROBUST_PCBF_THEORY.tex`. Future states receive tracking costs,
but no additional CLF descent inequalities. Target presence alone selects
the optimization mode.

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
If that candidate exists and the recovery objective is

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

In the declared terminal kinematic model, retain the steering and apply a fixed
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

The terminal-model assumption keeps beta_E = atan2(v_y,v_x) and kappa_E = r/V
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

    X_f = { z : x_E is in the shared observer domain and Phi_f(z,t) satisfies C
                 at all declared constraint instants }.

The endpoint uses the same operating cone |v_y| <= tan(beta_max)*v_x as
the other prediction nodes, with beta_max = cfg.model.sideslipMaximum
(0.25 rad by default). There is no separate terminal sideslip parameter or
stricter endpoint cone. This domain belongs to the current observer's
force-balance inversion and error-bound contract; it is not a numerical
sideslip cap specified by Sharma et al. (2026). No additional bound on |r|*V is
imposed, either in membership or in the local optimization rows. The
braking ratio must satisfy actuator limits, and the configured braking rate
must lie inside the longitudinal tire capacity. These checks do not certify
combined-force realizability in the Fiala plant: applicability of the
simplified terminal flow remains the declared modeling assumption above.
No additional rear-tire nonsaturation condition is imposed. Since beta_E
is constant, the shared operating cone persists under the declared flow. Input
memory is part of the joint state; finite slew limits require an admissible
entry command, not just a geometric certificate.

The executable membership test checks:

1. The shared observer operating domain and terminal braking admissibility.
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
prediction endpoint can be a moving state with an available braking
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
neighborhood: position translations remain unbounded; numerical trust bounds
on heading and dynamic states do not certify a perception-exit enclosure. It retains the active target's
collision rows until an observed exit. Terminal rows retain all computed
continuation collision conditions when no certified endpoint correction bound
is supplied, including distant conditions previously omitted by a fixed band.
Both signs of the lateral-acceleration condition are linearized for a moving
endpoint. The terminal continuation's sampled schedule and exit branch remain
local approximations and are not a global nonlinear exit certificate.

`rows` builds the local terminal approximation once. For a straight target
track intersecting the parked footprint, its transverse support normal uses
the sign with the larger minimum vertex gap at the parked reference pose.
An exact tie retains the encounter passing side. A road-forward preference
must not force the parked pose onto the opposite side from its braking
endpoint. This changes the local representation, not the braking policy or
the parked-track membership condition. At circular-track
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
optimizer still chooses the final input, subject only to the numerical
input-bound normalization described above.

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
The target-presence PCBF/CLF modes and the physical vehicle model are retained.

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

During an active encounter the ideal controller minimizes (U9), chooses a
minimum-performance-cost member of its optimal solution set, and applies
its first input. This tie-breaker does not alter the optimal barrier value.
Once the target leaves the
perception-scoped encounter, the collision problem becomes inactive and the
controller selects its cruise CLF objective. Section 4 gives the additional
dissipation and recovery-domain premises; a newly detected encounter requires
its own initial feasible plan. This mode rule adds no cross-frame slack budget.

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
affine optimum is not a certified gap to this nonlinear optimum.

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

On every target-free frame, the controller builds the cruise reference from
the current estimated state using nominal feedback and the nonlinear vehicle
transition. The cruise derivative is linearized about this rollout instead
of the previous avoidance/braking iterate or a large affine cruise correction.
This uses the ordinary target-free horizon schedule, one affine model and one
cruise QP; it is neither another avoidance-planner call nor a post-solve rollout.
The state/input shift is retained throughout each active avoidance encounter.

The field constructs the initial avoidance reference; only the optimization
supplies execution inputs. During avoidance, optimized state/input prefixes are shifted
without full-horizon replay or reshaping by the boundary guidance field. Tail construction retains the
existing terminal/nominal seed policies. The tangent initializer's handling
of singular force-ratio endpoints remains as described above.

The initializer integrates one field through the modified Fiala model. Route
attraction supplies the preferred acceleration, predicted obstacle motion
modifies it, and road guidance follows. Cruise-speed feedback does not overwrite
longitudinal obstacle acceleration afterwards. There is no preliminary route
rollout, discrete cruise/avoidance selection, candidate bank, or planner QP.

For route attraction, write lateral displacement as $e$, course relative to the
path as $\chi$, and lookahead as $D$. The desired course is
$\chi_d=-\arctan(e/D)$. Path-curvature feedforward, $\dot\chi_d$, and proportional
course-error feedback determine a course-rate request; multiplying by speed
provides transverse acceleration. The existing speed gain provides the preferred
longitudinal acceleration. No offset waypoint or return time is prescribed.

**Time-aligned exposure.** At each virtual rollout state $(p,v,\psi)$, query the
exact NRMM target prediction at times $t+\tau_j$. Compare it with the diagnostic
characteristic $p_j=p+\tau_j v$, retaining the current ego body yaw. This is not
a second candidate trajectory: only the generated controls rolled through Fiala
supply the returned state sequence. NRMM retains target acceleration, curvature,
rectangle-reference offsets, and its existing stop-and-hold convention.

Let $\mathcal A_j$ contain the two body axes of each predicted rectangle,
oriented from target center toward ego center. Define the signed separating-axis
quantity

$$
g_j=\max_{a\in\mathcal A_j}\left[
 a^T(p_j-p_T(t+\tau_j))-s_E(-a,\psi)-s_T(a,\psi_T(t+\tau_j))
 -\epsilon_j(a)\right].
$$

Here $s_E,s_T$ are footprint support functions including reference offsets and
current directional yaw uncertainty. The position supports $\epsilon_j$ use the
observer/error information already available at this virtual rollout node,
frozen over this auxiliary preview. They are a guide-shaping approximation,
not a new certified future error tube. With exact poses, $g_j>0$ implies a
separating axis and $g_j<0$ indicates overlap. It is not Euclidean signed distance.
No extra fixed physical clearance is added.

Normalize penetration by the smaller vehicle half-width $b$:

$$
 q_j=\min(1,[-g_j]_+/b),\qquad E_j=q_j(2-q_j),\qquad
 \eta=\max_j E_j.
$$

Exposure disappears continuously at a grazing sample. If target and ego use the
same location at different times, that location alone does not create exposure.
For example, accelerating through a crossing can remove influence, whereas
braking to rest in the ego's future path can create it even with the same current
position and velocity. This is a state-dependent coefficient inside the field,
not an accept/reject decision between complete paths.

The auxiliary preview duration is derived from the perception range $R$, current
reference-point distance $d$, relative speed, and footprint scale $L$:

$$
 H={R+d\over\max(\|v-v_T\|,\lambda L)},\qquad
 L=\sqrt{a_E^2+b_E^2}+\sqrt{a_T^2+b_T^2}.
$$

Its equally spaced nodes have spacing at most the controller sample time. This
is an encounter-scale heuristic, not a stopping-time or reachability guarantee.
The preview ends at its first predicted perception exit after subtracting the
current position-error ball radii, when it starts inside the encounter. It does not create a static forbidden union of all future target
poses. This auxiliary $H$ is independent of the requested seed length and does
not change the fixed RTI optimization horizon. Extending a requested seed thus
extends the same initial field rollout.

**Transported tangential preference.** Let $d_T(p,t+\tau_j)$ be point distance to
the predicted target rectangle, evaluated at the *current* ego reference point,
and $n$ its current outward normal. Normalize the time-aligned exposures by

$$
 w_j={E_j e^{-\lambda\tau_j}\over\sum_i E_i e^{-\lambda\tau_i}}.
$$

If the denominator vanishes, obstacle influence is zero. Otherwise construct

$$
 g_{\mathrm{tan}}=(I-nn^T)\sum_j w_j
       [\nabla_p d_T(p,t+\tau_j)-n],\qquad \ell=Jn.
$$

The future rectangle's closest-point normal supplies each gradient; when the
query is inside it, use zero. The tangent projection removes radial effects
already handled by the clearance field. For a straight translating target,
the local expansion explains the bias:

$$
 (I-nn^T)[\nabla d_T(p,t+\tau)-n]
 \simeq-\tau(I-nn^T)\nabla^2d_T(p,t)v_T.
$$

For a translating point/circular surrogate, writing $r=p-p_T(t)$ gives the
stronger exact identity

$$
 (I-nn^T)\nabla d_T(p,t+\tau)
 =-{\tau(I-nn^T)v_T\over\|r-\tau v_T\|},\qquad n=r/\|r\|.
$$

Positive exposure weights therefore point tangentially opposite target motion,
toward space it is vacating. For rectangles this interpretation is local around
corners; on a flat face the local Hessian can vanish. Finite prediction and
changing nearest features still supply information. Rotating rectangles are
evaluated at their actual predicted poses rather than through the translational
approximation. These relations establish a directional preference, not optimal
path deviation or a collision-avoidance certificate.

Use $\sigma=\operatorname{sign}(\ell^Tg_{\mathrm{tan}})$ for the circulation
request. A negligible signal preserves existing relative tangential motion;
only remaining symmetry uses a deterministic turn hint. The hint no longer
comes from which side of the ego the target currently occupies. This is a soft
passing preference. It neither proves a globally better side nor guarantees
rear passing in every encounter. The weights are evaluated as coefficients of
this constructed vector field; the field is not claimed to be the gradient of
one scalar space-time potential. Switching the sign within virtual seed
construction is distinct from changing the controller's collision constraints
mid-encounter: the planner still runs only at target acquisition.

The existing moving-frame radial/tangential acceleration law supplies a raw
request $a_{\rm raw}$ using this $\sigma$. With route acceleration $a_{\rm free}$,
its outward correction is $c=\eta[n^T(a_{\rm raw}-a_{\rm free})]_+$.
The same nonnegative correction activates the one-sided tangential request,
scaled by $\min(1,c/a_{\rm budget})$. This removes the raw field's inward normal
attraction; it does not prove goal convergence. The combined field can request both braking and
steering, with route attraction always present. The incidence modulation,
finite preview, held-input Fiala allocation, and road correction do not inherit
the ideal unmodulated radial safety proof. Generated-path collision/state
checks remain diagnostics, and `robustCertificate` remains false. PCBF,
terminal, observer, trust-region and RTI acceptance conditions are unchanged.

This construction draws distinct ideas from the supplied literature:

- Finean, Merkt and Havoutis, *Predicted Composite Signed-Distance Fields for
  Real-Time Motion Planning in Dynamic Environments* (2021), Sections 3.1--3.3:
  evaluate predicted obstacle distance at the matching trajectory time. Their
  implementation uses GPMP2 and constant-velocity obstacle prediction; it is
  not a theorem for this vehicle field.
- Karamouzas, Skinner and Guy, *Universal Power Law Governing Pedestrian
  Interactions* (2014), DOI `10.1103/PhysRevLett.113.238701`: anticipatory interaction
  depends on future encounter timing rather than current separation alone.
  Its empirical pedestrian energy law and constants are not transferred to
  Fiala vehicle dynamics.
- Ko and Yang, *Gradient-Based Time-Extended Potential Field Method for
  Real-Time Path Planning in Infrastructure-Based Cooperative Driving Systems*
  (2025), DOI `10.3390/s25175601`, Section 3.3: explicit time-dependent fields.
  The implementation here does not perform gradient descent in a scaled
  $(x,y,t)$ domain.
- Ko and Yang, *Gradient-Based Time-Sequential Potential Field Method for Path
  Planning in Infrastructure-Based Cooperative Driving Automation* (2026),
  DOI `10.3390/s26072163`, Sections 3.1--3.4: query future fields along ego motion.
  Its acceleration candidates, risk scores and jerk-based selection are not
  imported. The tangential transport construction above is a project-specific
  synthesis, not a formula or guarantee attributed to these papers.

For $N$ returned holds and $Q$ auxiliary forecast samples per field evaluation,
the added geometry costs $O(NQ)$, with vectorized rectangle supports. It is not
an $O(N)$ claim when both grids grow under time-step refinement. No trajectory
optimization, candidate search or new user-tuned field gain is added.

Route coordinates also supply deviation diagnostics and legacy offset walls
when no boundary scan exists. Route attraction here belongs to initialization;
it introduces no additional optimization constraint.
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

The selected objective uses one model and the encounter's fixed node count. The same-length shift matches the horizon convention of Section 7;
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

### Offline nonlinear rollout checks

The online RTI update does not replay or reject its selected command after
solving. The research-only `check` probe can independently roll a supplied
input sequence through the nonlinear model and evaluate collision, state,
actuator and terminal residuals. These diagnostics do not define an online
slack cap or an additional optimization stage.

The probe covers implemented nodes and approximate midpoints. Future observer
enclosure assumptions, nonlinear uncertainty remainders and terminal modeling
remain the implementation boundaries stated below. A numerical diagnostic
must not be relabeled a robust nonlinear recursive-safety certificate.

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
at most 40 native interior-point iterations per solve and a 2 ms completion
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
`convexSolveCompleted` records completion of the selected convex objective.
`optimizationConverged` is false because nonlinear convergence is not tested.
`nonlinearValidationPerformed`, `nonlinearPredictionEvaluated` and
`postSolveFeasibilityCheck` are false. `nonlinearHardResidual` and
`linearizationGapMeters` are NaN rather than uncomputed zero residuals.
The reported PCBF value, stage slacks and `affineHardResidual` describe the
affine subproblem. During avoidance, `search.slackCap` is the attained primary
optimum when reference refinement is enabled. `secondaryStageCompleted` and
`secondaryOptimumApplied` distinguish a completed refinement from a retained
priority result. No nonlinear acceptance cap is imposed. When no encounter
is active, the CLF objective belongs to the input-affine approximation of the
current continuous derivative. An active encounter disables that CLF objective
and inequality while retaining the reference-proximity tie-breaker.
`predictedState` is the affine SQP iterate (`predictionModel=affineSqpIterate`).
Compatibility rollout metadata has `available=false` and
`reason=notPerformedByRti`; it does not signal a failed rollout.
Independent scenario replay measures clearance, road margins and nominal
recovery throughout the observation. Current-state terminal membership does
not stop simulation or count as task completion. The terminal set certifies
a possible safe continuation at the predicted endpoint; the controller keeps
executing its optimized first input on each frame. Campaign recovery requires
a sustained dwell near the prescribed path and speed after the encounter.
Actual terminal-controller activation instead fails the driving experiment
before that command is applied. Physical rest also fails the experiment before
another control cycle or recovery can be credited. Rest means the physical
motion states `(v_x,v_y,r)` are exactly zero under the shared sample-end rest
transition; no new speed threshold is introduced. These tests use physical
states and explicit control-mode metadata, not observer estimates, predicted
terminal states, or the sign of an optimized braking input. The current RTI
does not activate a terminal controller. Its terminal constraints and the
simplified theoretical continuation remain unchanged.
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

Behavior coverage includes `switchedPredictiveControlTest`,
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
