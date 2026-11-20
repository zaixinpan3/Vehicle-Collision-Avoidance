# Compact nonlinear output-feedback predictive control

The adopted controller combines fixed measurement-consistency inference with
a condensed nonlinear predictive-CBF optimization of actuator commands.
It enforces a terminal continuation set and retains a complete held-input
moving plan for validation at the next measurement. The first feasible
driving sequence supplies the next command. Collision constraints apply
to control-boundary state nodes. Control knots, integration samples, and sensor
samples have distinct roles; reducing the decision dimension does not remove the physical model.
The derivation below states both the algorithm and the limits of its guarantees.

## 1. Physical model and uncertainty contract

### Coordinates, inputs and tire forces

Use the physical ego state and held actuator input

$$
x=(p_x,p_y,\psi,v_x,v_y,r)^\top,\qquad u=(\delta,b)^\top,
\qquad t_k=kh.
$$

Positions are in meters, angles in radians, speeds in m/s, yaw rate in rad/s
and forces in N. The signed longitudinal-force ratio satisfies $-1\le b\le1$.
Let $h>0$ be the fixed control hold. With wheelbase $L=l_f+l_r$, the static
axle loads and the force capacity remaining for lateral motion are

$$
F_{zf}=mg\frac{l_r}{L},\quad F_{zr}=mg\frac{l_f}{L},\quad
F_{xi}=b\mu_i F_{zi},\quad D_i=\mu_iF_{zi}\sqrt{1-b^2}.
$$

The tire slip angles use the following force sign convention:

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

When $D_i=0$, lateral force is zero. Tire saturation is part of the
physical model. Admitted flows must remain in a domain on which the declared
vehicle and observer dynamics are well posed.

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
$s=\operatorname{clip}((v_x-v_\ell)/(v_h-v_\ell),0,1)$, where
$0<v_\ell<v_h$ and $\tau_\ell>0$ are declared model parameters.
The position/yaw equations remain unchanged. The theoretical transition
$F_h$ is the exact held-input flow of the declared model. Any additional
stopping reset must be specified as part of that model and included in every
joint transition and terminal continuation.

The body sideslip $\beta_E=\operatorname{atan2}(v_y,v_x)$ is the angle of
the center-of-gravity velocity, not either tire slip angle $\alpha_f,\alpha_r$.
In low-speed turning, $v_y\simeq l_r r$ can give nonzero body sideslip while
the rear tire slip is near zero. Consequently, a large ratio $|v_y|/v_x$ at
low speed does not by itself establish tire sliding or physical infeasibility.
The shared observer cone is an estimation-domain premise. Enlarging it changes
that premise and the recomputed error bounds, rather than increasing tire
capacity. The low-speed blend and its response lag remain model assumptions,
whose validity is part of the declared physical contract.

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

For $A_C<0$, the physical target moves until $V_C=0$ and then retains its
stopped pose. The latent acceleration and sideslip parameters remain fixed;
zero physical acceleration after stopping does not redefine those parameters.

The target stopping mode is absorbing. Inference retains its physical track
through missed radar returns and confirmed range exits. A new target requires
an explicit admissible initial information contract. Multiple targets require
joint hypotheses and separate collision constraints for each footprint.

Let the augmented physical Markov state be

$$
\chi=(p_E,R_E,v_E,r_E,\rho,Q,V_T,A_T,\beta_T,\vartheta,\sigma,\eta),
\quad \rho=R_E^\top(p_T-p_E),\quad Q=R_E^\top R_T.
$$

Here $\vartheta$ contains persistent parameters, $\sigma$ the physical modes,
and $\eta$ sensor biases or disturbance memory. Persistent parameters satisfy
$\vartheta^+=\vartheta$; a bias is not a fresh disturbance at each node. With
$J=[0,-1;1,0]$ and $e(\beta)=(\cos\beta,\sin\beta)^\top$,

$$
\dot\rho=V_TQe(\beta_T)-v_E-r_EJ\rho,\qquad
\dot Q=(\kappa_TV_T-r_E)JQ,\qquad
\kappa_T=\sin\beta_T/l_{rT}.
$$

At target absorption set $V_T=0$, retain the pose and latent parameters, and
use $\dot\rho=-v_E-r_EJ\rho$, $\dot Q=-r_EJQ$.

## 2. Raw measurements, timing, and one joint transition

The observations are position-only GNSS, gravity-compensated center-of-mass
body acceleration, gyro yaw rate, and relative radar position when available:

$$
y_p=p_E+\eta_p+n_p,\quad y_r=r_E+\eta_r+n_r,\quad
y_a=f_v(v_E,r_E,u^-;\vartheta)+r_EJv_E+\eta_a+n_a,
\quad y_\rho=\rho+\eta_\rho+n_\rho.
$$

The acceleration at a decision boundary uses the preceding applied input
$u^-$, not the command about to be optimized. A derived velocity is not an
independent GNSS observation. Joint bounded noise needs no statistical
independence. Detection flags and their admissibility contract belong to the
measurement relation. No return may represent a dropout.

Let $h=Lh_s$ and

$$
(\chi^+,Y,\gamma)\in\mathscr R_h(\chi,u)
$$

mean that one admissible realization generates the whole physical hold
$\gamma$, every sensor reading and flag in $Y$, and the endpoint $\chi^+$.
The same physical disturbance must affect motion and any acceleration output
it causes. Intermediate sensor boundaries do not change a held actuator
command or trigger a control-boundary reset. If a declared hybrid reset is
present, specify the ordering of reset and boundary measurement and use it
consistently for prediction, conditioning, and physical replay.

## 3. Fixed information update and causal control

Let $\mathcal I_k$ be the physical states consistent with the initial contract,
all actual applied inputs, and all received raw measurements. Its update is

$$
\mathcal I_{k+1}=\{\chi^+:\exists\chi\in\mathcal I_k,\gamma,
 (\chi^+,Y_{k+1},\gamma)\in\mathscr R_h(\chi,u_k)\}.
$$

The received block is conditioned using the input that actually generated it.
Persistent parameter and bias coordinates remain attached to each physical
history. An unavailable radar reading does not delete a target. The Markov
augmentation makes this set a sufficient information state; it need not be a
minimal statistic. If the contract is valid and the initial state is contained,
the exact update retains the actual successor.

The numerical representation is a finite retained ensemble
$\widehat{\mathcal I}_k=\{\chi_k^s\}_{s=1}^{S_k}$. Each member is propagated
through every sensor period with the applied input. It survives only if every
received residual satisfies its sensor bound. No surviving member is replaced
by an independently fitted point, and parameters are not redrawn. An empty
ensemble is an information-contract failure, not evidence that collision
avoidance is physically impossible. A finite ensemble does not enclose all
states between its members.

For a tracked target let \(d_k^-=\min_s\|p_{T,k}^s-p_{E,k}^s\|\) and let
\(R\) be the radar range. Record entry once a radar return has occurred or
\(d_k^-\le R\). A target explicitly supplied by the initial contract is
included on its first approach. Following entry, the target contributes
collision and terminal separation constraints at decision \(k\) exactly when
\(d_k^-\le R\). If \(d_k^->R\), the target-free planning problem retains
the physical-domain, actuator and moving terminal continuation conditions.
The posterior continues to propagate the target. Re-entry restores its full
node and terminal constraints before a new command can be issued.

The inference equations are fixed. There is no online optimization of an
observer gain or learned memory update. Planning selects one representative
member by

$$
s_0\in\arg\min_s\left\|x_k^s-\frac1{S_k}\sum_a x_k^a\right\|^2.
$$

This member supplies the geometric guide and the reported tracking cost. Every retained member is propagated
under the same proposed input sequence inside the nonlinear constraint oracle
and again at the refined acceptance resolution. This is an approximation to robust output feedback: control at
$t_k$ depends only on $\mathcal I_k$, the reference, and retained plans. Future
open-loop inputs are reoptimized after future measurements arrive.

## 4. Preview time, control knots, and held-input dynamics

For speed bound $v_k^+=\max_s\sqrt{(v_{x,k}^s)^2+(v_{y,k}^s)^2}$ choose

$$
T_k^{\rm req}=\max\{T_{\min},2h,v_k^+/a_p+T_s,T_{\rm space}\},\qquad
N_k=\max\left\{N_{k-1}-1,\left\lceil
 \frac{\Delta T\lceil T_k^{\rm req}/\Delta T\rceil}{h}
\right\rceil\right\}.
$$

For a target and positive cruise reference $v_{\rm ref}$, let

$$
T_{\rm space}=\frac{2(a_E+\max_s a_T^s)}{\max(v_k^+,v_{\rm ref})},
\qquad a=\sqrt{\ell^2+w^2}+\|o\|,
$$

where $\ell,w$ are footprint half dimensions and $o$ is the reference-point
offset. Otherwise set $T_{\rm space}=0$. This spatial lookahead exposes the
scale of passing both footprints at low speed; these radii size the horizon
only and never replace the rectangle-distance constraints.
Here $a_p>0$ is a preview scale, not a certified braking acceleration. The
minimum preview admits avoidance planning even when the ego is slow and a
target is approaching. An explicitly configured duration replaces the
speed-based rule. Neither rule proves that a feasible maneuver exists or
that this is an upper bound on the necessary maneuver time. A uniquely
predicted endpoint for specified inputs does not determine those inputs or
establish reachability of a safe endpoint.

At initialization the inherited-length term is zero. On continuation,
$N_{k-1}-1$ is the length of the unexecuted feasible suffix, which ends at
the old terminal state. The preview may therefore shrink by one hold per
decision as its current time requirement decreases; no terminal action is
appended solely to preserve the previous horizon length. If the current
preview requests a longer horizon, the driving extension must itself satisfy
the executable constraints and end in the terminal set. The accepted plan always spans the
selected preview. The construction below supplies all missing holds.

Use $M\le N$ control intervals, with integer boundaries
$0=c_0<c_1<\cdots<c_M=N$ and $b_i=c_i-c_{i-1}$. The first few intervals
have one hold each; the remaining holds are distributed over the remaining
intervals by rounded equally spaced boundaries. The knot count is the smaller
of the configured count and the accepted horizon, so each interval contains
at least one hold. Independent physical decisions are endpoint commands $v_i\in\mathbb R^2$. Physical states are eliminated by forward
substitution of the dynamics, separately for every retained hypothesis.
With $v_0=u^-$, the commands within interval $i$ are

$$
u_{c_{i-1}+j}=v_{i-1}+\frac{j}{b_i}(v_i-v_{i-1}),
\qquad j=1,\ldots,b_i.
$$

Each $u_j$ is held for the actual control period $h$. This interpolation does
not assume a continuously varying actuator within a hold. Since $b_1=1$,
the next applied command is $v_1$. Input bounds on endpoint commands imply
the same convex bounds on every interpolated command. If a finite steering-rate
bound is explicitly supplied as part of the actuator contract, it is imposed by

$$
|v_{i,\delta}-v_{i-1,\delta}|\le b_i h\dot\delta_{\max};
$$

hence every physical command increment is at most $h\dot\delta_{\max}$.
Without a specified finite bound this inequality is omitted. The analogous
force-ratio increment is imposed when its bound is finite.

Let $\widetilde F_h$ denote the numerical held-input flow, including the
rest reset only at the control boundary. Define the condensed trajectories

$$
x_0^s=x_k^s,\qquad x_{j+1}^s=\widetilde F_h(x_j^s,u_{j+1},\vartheta^s).
$$

The initial integration mesh resolves the sensor period and configured
integration step. Dense poses inside each RK4 step of duration $\Delta$ use
its continuous extension

$$
x(\theta)=x+\Delta\{b_1(\theta)k_1+b_2(\theta)(k_2+k_3)+b_4(\theta)k_4\},
\quad 0\le\theta\le1,
$$

$$
b_1=\theta-\tfrac32\theta^2+\tfrac23\theta^3,\quad
b_2=\theta^2-\tfrac23\theta^3,\quad
b_4=-\tfrac12\theta^2+\tfrac23\theta^3.
$$

The nonlinear validation flow uses a fixed RK4 mesh, with physical-domain
checks inside each hold and collision checks at its boundaries. Local SQP
sensitivities propagate $S_{j+1}=F_x S_j+F_uD_v u_j$ on a cheaper mesh. The
held inputs reconstructed from a trial must pass the original nonlinear
validation. Neither mesh supplies a certified integration-error bound.

The causal measurement-consistency update uses the same fine RK4 resolution,
subdividing each sensor interval without applying the absorbing reset before
the control boundary. Its small compiled flow is cached independently of the
optimization graph. Otherwise, an accurate raw acceleration reading near a
model transition can reject the true physical member solely because inference
used a coarser flow. Sensor noise bounds are unchanged; finer integration is
still not a certified discretization-error enclosure.

The compiled flow graph allocates capacity from configuration size hints and
the preview rule. These hints are not planner velocity constraints. A larger
preview grows the graph; unused slots are identities. Updating the active
preview within the allocation does not require rebuilding the flow graph.
The allocation cannot truncate a requested preview.

## 5. Dual rectangle separation

Write the target footprint as $A_T(y-p_T)\le b_T$, where
$A_T=[I;-I]R_T^\top$ and $b_T=[h_T+o_T;h_T-o_T]$.
Introduce one nonnegative target multiplier shared by all ego vertices $e_a$,
$\lambda\in\mathbb R^4_+$ at each prediction check. Impose

$$
\lambda^\top A_T(p_E+R_E e_a-p_T)-b_T^\top\lambda
\ge d_{\rm safe},\qquad \forall a,
\qquad \|A_T^\top\lambda\|_2^2\le1.
$$

For any target point $y$, $\lambda\ge0$ implies
$\lambda^\top A_T(y-p_T)\le b_T^\top\lambda$.
Consequently each displayed vertex inequality lower-bounds the projection
of every ego-to-target displacement onto $n=A_T^\top\lambda$.
Convexity extends the inequality from vertices to the complete ego footprint;
$\|n\|\le1$ then proves clearance at least $d_{\rm safe}$.
Conversely, strong convex distance duality supplies such a multiplier for
disjoint rectangles with distance at least a positive $d_{\rm safe}$.
The normal is optimized continuously and is not restricted to body-edge axes.
No fixed planning buffer or smooth-max distance approximation is added.
Common translations cancel in $p_E-p_T$.

For the predictive deficit, use the signed extension with a unit normal:

$$
d_\pm(x,q)=\max_{\lambda\ge0,\ \|A_T^\top\lambda\|_2=1}
\min_a\{\lambda^\top A_T(p_E+R_Ee_a-p_T)-b_T^\top\lambda\}.
$$

It equals Euclidean distance for separated convex rectangles and is negative
in penetration. In particular, zero multipliers cannot make an overlapping
configuration's deficit independent of its penetration. The inner dual
optimization is eliminated exactly: closest vertex/edge pairs give its value
outside collision, while Minkowski-polygon facet normals give its signed value
inside collision. The latter use of body-edge normals is exact for penetration;
positive separation is never replaced by a four-axis clearance approximation.
This is algebraic elimination of the geometric witnesses, not removal of the
dual geometry. The resulting value is piecewise smooth at feature changes.


## 6. The compact nonlinear program

Let $e_i$ contain lateral displacement from the fixed reference frame at
node $i$, wrapped heading error, longitudinal-speed error, lateral velocity,
and yaw rate. Reference frames are obtained by projecting the constant-speed
extrapolation from the current nominal pose onto the reference path. The
reported tracking cost is

$$
J=\frac12\sum_{i=1}^M h b_i e_i^\top W e_i
 +\frac{w_u}{2}\sum_{i=1}^M\frac{\|v_i-v_{i-1}\|^2}{b_i}.
$$

The compact default is $W=\operatorname{diag}(0.02,0.2,2,0.1,0.1)$ and
$w_u=0.1$. The lower transverse weight permits a passing excursion while
retaining the fixed cruise path and speed as the zero-error reference. This
performance choice does not supply a general convergence or liveness theorem.

The second term equals the sum of squared increments over all interpolated
held commands. For the node-only collision contract, define

$$
C_j(v)=\max_s\max_{r\in\{j,j+1\}}
\{d_{\rm safe}-d_\pm(x_r^s,q_r^s)\},\qquad j=0,\ldots,N-1.
$$

Each stage covers its two control-boundary nodes; their union includes the
current root and the terminal node. No interior collision sample or motion
reserve enters this deficit. The primary problem is

$$
\min_{v,\xi\ge0}\ \sum_{j=0}^{N-1}\xi_j,\qquad
C_j(v)\le\xi_j,\qquad v\in\mathcal U_N,\qquad
v_{x,j}^s\ge0,\qquad (x_N^s,q_N^s,u_N)\in Z_f\quad\forall s.
$$

Here $\mathcal U_N$ includes physical command bounds and any explicitly
specified input increments. The forward-motion condition and the original
Fiala model domain are checked throughout the sampled trajectory, including
both sides of command changes. On the dynamic or blended branch, the front
wheel velocity must satisfy
$v_x\cos\delta+(v_y+l_f r)\sin\delta>0$, equivalent to
$|\alpha_f|<\pi/2$ on the declared steering domain. This condition is inactive
on the pure low-speed branch. Symbolic guards used to evaluate rejected trial
points do not extend the physical model's domain. Terminal membership is hard
and has no safety slack.
The primary problem has $2M+N$ variables. A driving sequence must also
satisfy $\|(v_x^s,v_y^s)\|_2>0$ after the initial node at the sampled
physical states of each executable hold, including its post-reset endpoint.
An initially stationary ego can depart on the first hold. The strict
condition introduces no calibrated minimum cruise speed.

Let $\tilde v_{x,j}^s$ denote the longitudinal speed just before the
boundary reset. Its stop branch has $b_j\le0$ and
$\tilde v_{x,j}^s<0.2v_{\rm hold}$. During feasibility restoration, the
inequality
$$
\min\{0.2v_{\rm hold}-\tilde v_{x,j}^s,-b_j\}\le 0
$$
excludes the interior of that branch. The original nonlinear flow and the
strict motion check resolve equality cases. These constraints apply to the
executable horizon. The following terminal construction has its own dynamics
and is evaluated as a membership witness.

A feasible zero-deficit driving sequence attains the lower bound of the PCBF
objective. Its first input is issued immediately after validation. Tracking
cost $J$ is available for analysis and does not rank accepted plans.

### A moving terminal set and its continuation

Let the common terminal command be zero steering and the nominal force ratio
for straight travel at the fixed cruise speed,
$$
 u_f=(0,b_f),\qquad
 b_f=\operatorname{clip}_{[b_{\min},b_{\max}]}
        \frac{F_{\rm load}(v_{\rm ref})}{\sum_i\mu_iF_{zi}}.
$$
All hypotheses receive the same command. When the trim is within the actuator
bounds it balances the nominal road load at the reference speed. The resulting
speeds may differ across physical members.
The cruise certificate requires sustainable forward motion under this input.

Write $E(x)$ for the oriented ego rectangle, $e=(\cos\psi,\sin\psi)$ and
$T(q(t))$ for the target rectangle. In the simplified terminal model the ego
continues along the forward ray with fixed orientation and strictly positive
speed. Its speed need not have a specified time profile. Define
$$
 L_E(n,x)=\min_{z\in E(x)} n^\top z,\qquad
 U_T(n,q)=\sup_{t\ge0}\max_{z\in T(q(t))}n^\top z .
$$
A sufficient moving core $C$ requires $v_x>0$ and a witness satisfying
$$
 \|n\|=1,\qquad n^\top e\ge0,\qquad
 L_E(n,x)-U_T(n,q)\ge d_{\rm safe}.
$$
In the absence of a target the separation condition is vacuous. The witness
may differ across hypotheses. One common separating plane over the remaining
sweeps is a sufficient terminal condition. Stage constraints still use the
configuration-specific rectangle distance dual. Some encounters that are
safe solely because the vehicles reach a crossing at different times need
a longer maneuver before they enter this core.

For straight target motion each corner sweeps a segment up to the absorbing
stop, or a ray if the target continues indefinitely. Its upper support is the
larger endpoint projection for a segment. For a ray it is finite for
$n^\top e_T\le0$ and equals the initial projection. At rest use the current
rectangle. For nonzero curvature each corner moves on a circular arc about
the target turn center. Its maximum projection occurs at an endpoint or at
the point whose outward radius is parallel to $n$, if that angle belongs to
the remaining directed arc. A full revolution includes that extremum for
every direction. Maximizing over the four corners gives $U_T$. The angle
search in the admissible semicircle returns a unit-normal witness. Acceptance
uses that witness's support gap, so it needs no globally optimal angle.
A retained normal remains available for its surviving hypothesis on shift.

Under the simplified model the core is positively invariant. After a forward
translation by $\Delta s>0$, $L_E$ increases by $\Delta s\, n^\top e\ge0$.
The remaining target sweep is a subset of its previous sweep, so $U_T$
cannot increase. The same normal certifies the successor.

Let $F_f$ be one full nonlinear prediction hold under $u_f$ and set
$B=\max(1,\lceil T_{\rm preview,min}/h\rceil)$. The accepted terminal set is
the constrained $B$-step predecessor of $C$. Entry requires compatibility of
$u_f$ with the preceding input, forward motion, and admissible nonlinear
states and node clearance along the entire $B$-hold bridge. Its last state
must belong to $C$. SQP differentiates the bridge and the active terminal
support gap. Acceptance recomputes the original nonlinear inequalities.
The bridge allows steering and lateral transients to evolve before the
straight-cruise handoff.

**Physical handoff assumption.** For the full vehicle, the terminal argument
assumes that under $u_f$ a core state remains forward-moving and in the
physical domain, with $n^\top e^+\ge0$ and $L_E(n,x^+)\ge L_E(n,x)$ for
the retained witness. These are conditions on ego motion. Together with
nesting of the target sweep they imply successor separation. The ray model
establishes these conditions for fixed heading and forward translation. A nonlinear vehicle can still
have residual sideslip and yaw rate after $B$ holds. The finite bridge does
not prove the handoff assumption for that vehicle. Each online acceptance
rechecks nonlinear motion and core membership. A violation invalidates
the shift and must be reported if no other plan passes.

The geometric guide constructs one route-following trajectory for the
primary search. At each integration step its tangential acceleration is
proportional to the sum of the transport and separation gradients of the
predicted encounter. In a reflection-symmetric head-on encounter on a
straight reference, the lateral contributions cancel. The resulting
trajectory supplies the initial actuator sequence. For each subsequent
trial, Section 5's separation problem uses that trial's node poses.
When needed, project the initialization onto the linear actuator and terminal
handoff rows. SQP then minimizes the sum of stage deficits under the hard
domain, motion and terminal constraints. Validation of the first feasible
trial ends the search, including a trial found inside a line search.

## 7. Node acceptance and retention of executable plans

For each retained physical member, expand the proposed coarse commands into
all $N$ held commands and propagate its own persistent parameters. Fine RK4
replay uses $K_v=8$ steps per hold by default, with the held command and the
rest reset applied at control boundaries. This is the integration and
physical-domain mesh; it does not define additional collision constraints.
Every new executable proposal is checked for physical input bounds,
configured input increments, and the physical-model domain.

Let $z_j^s=(x_j^s,q_j^s)$ be the joint state at $t_k+jh$. While the target
is active at decision $k$, each proposed plan must satisfy

$$
d_\pm(z_j^s)\ge d_{\rm safe},\qquad
j=0,\ldots,N,\qquad s=1,\ldots,S,
$$

with the same condition at every control boundary of the nonlinear cruise
bridge. These checks use the exact rectangle-distance dual value from
Section 5. The control-boundary state nodes are retained even inside a long
control-knot interval. The first $N$ stage inequalities $C_j\le0$ are
therefore equivalent to clearance at all $N+1$ state nodes. There is no
intersample clearance requirement or collision reserve. A trajectory can
pass these collision inequalities while intersecting between nodes; the
algorithm makes no safety claim at those intermediate times.

Only a sequence passing these checks can supply a command. The condensed trajectory is dynamically consistent with its numerical flow
by construction. Geometry and state checks include the current
root. The full accepted fine input sequence is stored.

For target-free cruise on a realizable constant-curvature reference, let
\(u^\star,x^\star\) denote the physical trim and \(G(x,u^-)\) the
route-guidance command. The nominal guide uses
\(G(x,u^-)+u^\star-G(x^\star,u^\star)\), subject to the actuator and
increment bounds. The straight-road correction is zero. At an admissible
trim this gives \(u^\star\), so a vehicle at trim follows the reference.
An unattainable steady turn uses the bounded route guidance without a trim
correction. This equilibrium identity alone supplies no CLF decay inequality.

### Recursive feasibility on the admitted information domain

Releasing target constraints enlarges the feasible set for the same
horizon. On re-entry the collision obligations increase. The shift proof
below applies through that boundary only if the incoming state and
continuation are feasible for the reactivated encounter. The controller
constructs one guide sampled at the control knots and their discrete
midpoints. Its interpolated knot commands and, if needed, its original
held commands undergo the full nonlinear checks. The retained suffix must
pass those same checks to supply an action. Re-entry feasibility thus
requires a separate admission assumption.

After applying the first input, let $r=N_{k+1}-(N_k-1)\ge0$. Form the
executable candidate
$$
 U_{k+1}^{\rm shift}
   =(u_{1|k},\ldots,u_{N_k-1|k},\underbrace{u_f,\ldots,u_f}_{r}).
$$
The fine held-input suffix remains directly representable even when it no
longer interpolates the new control knots. Generated candidates and this
shift must each pass the same checks over the complete selected horizon.
An optional new guide input is accepted only with a feasible continuation.

Suppose an initial plan is feasible, the posterior contains the true member
and is contained in the predicted successors, the flow and timing agree,
requested horizons fit the numerical allocation, and the core handoff
assumption holds. Let $z\in Z_f$. One step of $F_f$ consumes the first of
the $B$ checked bridge holds. The remaining $B-1$ holds reach $C$. Core
invariance supplies one more admissible moving hold ending in $C$. Thus
$F_f(z)\in Z_f$. Induction gives any $r$ required extension holds under a
single common input. Conditioning can discard physical members, and all
surviving members inherit their own witnesses. When $r=0$ the old endpoint
is already in $Z_f$.

The shifted feasible plan inherits the old stage deficits after the executed
stage and appends $r$ zeros. On a shift-closed policy domain this gives
$$
 V_{N_{k+1}}(\mathcal I_{k+1})
 \le V_{N_k}(\mathcal I_k)-\xi^*_{0|k}.
$$
In particular, an accepted zero-deficit plan supplies a zero-deficit
successor even if the optimizer returns no improvement. This induction gives
continued admissible motion under the stated assumptions. It supplies
neither a positive minimum speed uniform over all initial conditions nor a
proof of return to the nominal lane. Numerical rechecks and finite
hypotheses do not provide a robust physical-model or timing certificate.

## 8. Scope of the resulting statements

The exact information update has the containment property under its stated
contract. The feasible distance dual proves geometric separation for
individual configurations. The command interpolation enforces each actual
input increment. These facts do not convert a finite ensemble, numerical
integration, sampled geometry checks, or finite preview into a universal
continuous-time safety certificate.

Even individual physical feasibility is weaker than output-feedback
feasibility: indistinguishable states may need different actions. To assert
that every information state with some physically safe trajectory is handled
would require a suitable causal viability analysis, uncertainty coverage,
and a complete feasibility method. This local nonlinear program supplies no
such completeness theorem. Experiments can establish success over the tested
contracts and encounters; unclassified failures must remain unclassified.

Mean solve time, full controller-call time, startup compilation, and worst-case
latency are different quantities. A solver time limit is not a hard actuator
deadline, and an average latency does not certify every deadline. The sampled
physical model presumes commands are delivered at their specified boundaries;
latency must be included in the plant/actuation contract for delayed execution.

## 9. Relation to the literature and other numerical backends

Thompson, Dallas, Goh and Balachandran (2024), *Adaptive Nonlinear Model
Predictive Control: Maximizing Tire Force and Obstacle Avoidance in Autonomous
Vehicles*, [paper](https://doi.org/10.1109/TFR.2024.3497925), motivates sparse
nonlinear optimization, few decision nodes over substantial preview, and warm
initialization. Its spatial-coordinate vehicle/actuator transcription is not
identical to this held-input time-coordinate model, which includes rest.

The optional `sqpRti` and `ipopt` dynamic-memory implementations and the
independent NRMM integration are research comparators. The adopted compact
controller does not optimize their memory maps or observer gains. The NRMM
observer's own conditional derivation is in
[OBSERVER_ISS_THEORY.md](../estimator/OBSERVER_ISS_THEORY.md).
