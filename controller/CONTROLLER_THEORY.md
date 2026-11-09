# Compact nonlinear output-feedback predictive control

The adopted controller combines fixed measurement-consistency inference with
a compact nonlinear optimization of physical states and actuator commands.
It retains a complete held-input plan for numerical safety validation and
fallback. Control knots, integration samples, and sensor samples have distinct
roles; reducing the decision dimension does not remove the physical model.
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

The target stopping mode is absorbing. Losing a radar return does not remove
a target, terminate its collision obligation, or justify reinitializing the
physical information. A new target requires an explicit admissible initial
information contract. Multiple targets require joint hypotheses and separate
collision constraints for each footprint.

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

The inference equations are fixed. There is no online optimization of an
observer gain or learned memory update. Planning selects one representative
member by

$$
s_0\in\arg\min_s\left\|x_k^s-\frac1{S_k}\sum_a x_k^a\right\|^2.
$$

Only its physical trajectory is a shooting decision. All retained members
are subsequently propagated under the same proposed input sequence for
acceptance. This is an approximation to robust output feedback: control at
$t_k$ depends only on $\mathcal I_k$, the reference, and retained plans. Future
open-loop inputs are reoptimized after future measurements arrive.

## 4. Preview time, control knots, and held-input dynamics

For speed bound $v_k^+=\max_s\sqrt{(v_{x,k}^s)^2+(v_{y,k}^s)^2}$ choose

$$
T_k^{\rm req}=\max\{T_{\min},2h,v_k^+/a_p+T_s\},\qquad
N_k=\max\left\{N_{k-1},\left\lceil
 \frac{\Delta T\lceil T_k^{\rm req}/\Delta T\rceil}{h}
\right\rceil\right\}.
$$

Here $a_p>0$ is a preview scale, not a certified braking acceleration. The
minimum preview admits avoidance planning even when the ego is slow and a
target is approaching. An explicitly configured duration replaces the
speed-based rule. Neither rule proves that a feasible maneuver exists or
that this is an upper bound on the necessary maneuver time. A uniquely
predicted endpoint for specified inputs does not determine those inputs or
establish reachability of a safe endpoint.

Use $M\le N$ control intervals, with integer boundaries
$0=c_0<c_1<\cdots<c_M=N$ and $b_i=c_i-c_{i-1}$. The first few intervals
have one hold each; the remaining holds are distributed over the remaining
intervals by rounded equally spaced boundaries. The initial knot count is
retained when the preview grows. Independent decisions are endpoint commands
$v_i\in\mathbb R^2$ and physical shooting states $X_i\in\mathbb R^6$.
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

Let $\widetilde F_h$ denote the numerical flow with a held input. It composes
RK4 sensor-period subflows and applies the declared rest reset only at the
control boundary. Inference and planning use the same sensor-period mesh.
The multiple-shooting equality is

$$
X_i=\widetilde F_h(\cdots\widetilde F_h(X_{i-1},u_{c_{i-1}+1},\vartheta^{s_0})
\cdots,u_{c_i},\vartheta^{s_0}),\qquad X_0=x_k^{s_0}.
$$

Additional numerical poses are evaluated at each half hold. Reducing the
number of independent commands and shooting states does not remove these
intermediate motion and collision checks.

The graph initially allocates capacity using configuration size hints and
the preview rule. These hints are not planner velocity constraints; a larger
requested preview grows the graph instead of being rejected. Per-interval
active hold counts are parameters. Unused
integration slots act as identities, and their inequalities are made
unbounded. Changing the active preview within that capacity does not rebuild
the solver. The capacity is not an authorization to truncate a longer
requested preview.

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

## 6. The compact nonlinear program

Let $e_i$ contain lateral displacement from the fixed reference frame at
node $i$, wrapped heading error, longitudinal-speed error, lateral velocity,
and yaw rate. Reference frames are obtained by projecting the constant-speed
extrapolation from the current nominal pose onto the reference path. The
objective is

$$
J=\frac12\sum_{i=1}^M h b_i e_i^\top W e_i
 +\frac{w_u}{2}\sum_{i=1}^M\frac{\|v_i-v_{i-1}\|^2}{b_i}.
$$

The second term equals the sum of squared increments over all interpolated
held commands. The nonlinear program minimizes $J$ over physical states,
commands and the separation multipliers,
subject to the shooting equalities, the declared forward-motion domain, endpoint input
bounds, command-increment inequalities, and the separation inequalities.
There are $8M$ physical state and command variables and four nonnegative dual
variables per collision check. Longitudinal speed upper bounds, lateral velocity
bounds and yaw-rate bounds are not imposed by the collision planner. The target
has its prescribed forecast,
not a separately optimized motion or a requirement to brake or stop.

The endpoint obeys the same physical and collision constraints. There is no
remaining-braking-distance condition, forced final braking input, or assumed
invariant terminal set in this numerical algorithm. Therefore the formulation
does not inherit an invariant-terminal-set recursive-feasibility theorem.

IPOPT performs nonlinear iterations with automatic first derivatives and a
limited-memory Hessian approximation. The nonlinear vehicle equations remain
in the problem; a linearization is only an internal step of the nonlinear
solver. A numerical iteration/time limit can stop the solve before convergence.
Convergence is neither required nor sufficient for acceptance: the proposed
input sequence is checked separately by numerical rollout. Numerical input
bound violations are projected onto the actuator bounds before replay; only
the projected and rechecked commands may be executed.

For feasibility restoration, allocate one additional scalar $s\ge0$ and
require each vertex dual value plus $s$ to be at least $d_{\rm safe}$. Its objective is
$J+10^4s$. During ordinary improvement fix $s=0$. When no checked plan is
available, permit $s\ge0$ temporarily while searching from one geometric
planner rollout. Its dual multipliers are initialized from closest-feature
normals. Measurement-consistent physical hypotheses are distinct from this
single control initialization.

Restoration is a numerical search procedure, not a recovery control law or
an exact lexicographic optimization. Its positive slack never authorizes a
clearance deficit: every executable candidate must independently pass the
original hard checks in Section 7. One continuous solve uses the single initialized trajectory within the
initialization or recovery budget.
These attempts count toward the actual call time; the iteration-level wall
limit is not a guaranteed interrupt deadline.

## 7. Dense acceptance and retention of executable plans

For each retained physical member, expand the proposed coarse commands into
all $N$ held commands, propagate its own persistent parameters, and evaluate
input bounds, configured physical input increments, the forward-motion domain,
and the exact rectangle-distance dual value. Dense acceptance uses $K_v=64$
RK4 steps per hold by default and retains the held command and control-boundary
rest reset. An improvement can use the coarse check when a checked fallback
is already available. There is no fixed extra clearance buffer.

For the given member let $a_E,a_T$ be footprint-radius upper bounds about the
vehicle reference points. Given valid continuous speed/rate bounds, a Lipschitz bound on Euclidean footprint distance is

$$
L_d=\sqrt{v_{x,\max}^2+v_{y,\max}^2}
+\bar V_T+a_E\bar r_E+a_T\bar r_T,\qquad
\bar r_T=|\sin\beta_T/l_{rT}|\bar V_T.
$$

The target maxima are obtained from its prescribed motion over the preview.
For two neighboring check times separated by $\Delta t$, write
$q_0,q_1$ for their exact rectangle distances. Since each is a lower bound
on Euclidean distance, the two endpoint Lipschitz cones give

$$
d(t)\ge\max\{q_0-L_dt,\ q_1-L_d(\Delta t-t)\},\qquad
\underline d=\max\left\{q_0-L_d\Delta t,\ q_1-L_d\Delta t,
\frac{q_0+q_1-L_d\Delta t}{2}\right\}.
$$

Require $\underline d\ge d_{\rm safe}$ on every interval. Use
$\Delta t=h/2$ for the first check and $h/K_v$ after refinement. If both
vehicles remain exactly at their declared rest equilibria under nonpositive
ego force commands, $L_d=0$ is valid; stationary configurations do not need a
moving-vehicle reserve. The implementation estimates the speed/rate extrema
from the replay itself; it does not impose them as state constraints. These
sampled extrema and numerical poses are not validated continuous bounds, so
the interval check is an engineering check, not an end-to-end robust safety
certificate.

Only a sequence passing these checks can supply a command. An NLP iterate
with a shooting defect is assessed using its actual command rollout, not its
inconsistent shooting states. Geometry and state checks include the current
root. The full accepted fine input sequence is stored.

At the next boundary, construct a retained candidate by deleting its executed
first input and extending its tail with the last input as needed. Recheck the
complete sequence from the conditioned information. This sequence is retained
at its original hold resolution. It is not replaced by a fit to new coarse
knots: coarse interpolation is not generally closed under a one-hold shift.
A coarse refit is only a warm start for improvement.

If constant extension fails, try changing the final $0.5$, $1$, or $2$ seconds
of its force-ratio commands to the configured braking input, respecting input
bounds and increments. Each repaired sequence is rechecked on every retained
member. This candidate construction neither constrains the optimized endpoint
to stop nor assumes that braking is safe for every target motion.

Under identical discrete propagation and a valid finite-member update, the
old prefix after conditioning lies inside the old predicted member paths.
This explains why retaining the fine sequence preserves its previous checks
on the inherited interval. A refined verification mesh differs from the
inference mesh, so this exact-discrete argument cannot be transferred to that
numerical pair without an integration-error enclosure; the whole retained plan
is rechecked. The appended interval still requires a fresh check;
without an invariant terminal condition its feasibility is not guaranteed.

A replay-feasible proposal or seed can be accepted even if IPOPT reaches its
iteration or time limit. If improvement fails, a replay-feasible retained fine
plan remains available. If no checked candidate exists, the runtime reports a
planning failure and does not claim physical impossibility or authorize an
unchecked command. In particular, failure of a local solve, a coarse input
parameterization, or a conservative separator is not a proof that all
physically admissible causal controllers fail.

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
