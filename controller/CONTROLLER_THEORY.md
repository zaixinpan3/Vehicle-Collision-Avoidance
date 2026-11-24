# Compact nonlinear output-feedback predictive control

The adopted controller combines ego measurement consistency and NRMM target observation with
a condensed nonlinear predictive-CBF optimization of actuator commands on an
avoidance leg and its connection to the terminal set.
It enforces a terminal continuation set and retains a complete held-input
moving plan for validation at the next measurement. On the zero-deficit
level, predicted tire loading and rear slip guide the choice of driving sequence. Collision constraints apply
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
independent GNSS observation. A detection can also supply the relative
body heading $y_\psi=\operatorname{wrap}(\psi_T-\psi_E)$.
Joint bounded noise needs no statistical
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

The numerical controller retains a finite set of ego hypotheses, each with
its own persistent physical parameters and sensor bias. Each surviving ego
member explains the complete GNSS/IMU block. A newly detected target is
represented by a causal NRMM point observer paired with each member. This
finite collection approximates the exact information update.
A separately declared initial joint contract may already contain a known
track. The traffic experiments start with ego hypotheses only.

Let $t_d$ be the timestamp of the first received radar return. Before $t_d$
there is no target track. At a detection the position in the inertial frame
of ego hypothesis $s$ is
$$
 y_T^s=p_E^s+R_E^s\,\Pi_{\|\rho\|\le R}(y_\rho-\eta_\rho^s).
$$
The projection must respect the radar noise bound. In inertial axes the
observer state is $(\hat p,\hat v,\hat a)$. With radar predictor $z$ reset
to $y_T^s$ at a return, the sampled NRMM equations take the form
$$
 \dot{\hat p}=\hat v+L_1 e,\quad
 \dot{\hat v}=\hat a+L_2 e,\quad
 \dot{\hat a}=\Phi_e(\hat v,\hat a)+L_3 e,\quad
 \dot z=\hat v,\qquad e=z-\hat p .
$$
The innovation is disabled over intervals following a missed sample.
$\Phi_e$, the gain design and the motion reconstruction use the target
equations in [the NRMM derivation](../estimator/OBSERVER_ISS_THEORY.md).
For every positive estimated speed on the unsaturated branch, the outputs are
$V=\|\hat v\|$, $A=\hat v^\top\hat a/V$,
$\omega=(J\hat v)^\top\hat a/V^2$, $\beta=\arcsin(l_r\omega/V)$ and
$\psi_T=\arg(\hat v)-\beta$. The published motion uses $V$ in each denominator.
At exact rest the numerical motion outputs are zero and body heading requires
perception or prior information. The Cartesian jerk extension $\Phi_e$ has its
own positive denominator bound for the observer gain and error analysis.

A single position return identifies neither velocity nor body orientation.
The compact observer requires at least five distinct radar returns and retains
up to twenty to resolve motion and bound its parameters. During acquisition
the ego follows its nominal cruise controller. For a
populated window with resolved displacement, let $T=t_n-t_1$ and
$\tau_j=(t_j-t_n)/T$. The least-squares fit
$p_j\simeq c_0+\tau_j c_1+\tau_j^2 c_2/2$ supplies the current Cartesian
acceleration $c_2/T^2$, projected onto the declared Cartesian acceleration
ball. Refit the affine terms at this acceleration to initialize position and
velocity at the last return. Projection onto a ball containing the true
acceleration preserves the acceleration error bound of the quadratic fit.
A stationary-compatible window can instead initialize $(\bar p,0,0)$
with a measured relative body heading added to ego yaw. In the simulation,
a detected target supplies ideal relative body heading near rest, defined by
$V_T T_{\rm window}\le 2r_{\rm sensor}$ for the retained motion window span and
the nominal radar position error radius. Each ego member converts a received heading to inertial heading
using its own yaw at the measurement time. With position alone,
stationary acquisition remains pending. Avoidance requires a complete target
estimate for every retained member. At the first control boundary with complete
target outputs for all retained members and a detected or in-range target,
the PCBF includes the target immediately. Safety during nominal cruise
before the first feasible encounter plan is an initial-condition assumption.
At a positive return the planning root uses the current NRMM velocity and
acceleration through its inverse motion map. The observed body heading
takes precedence when available. At rest the observer retains the most
recent resolved heading. Its filtered position is projected into the
detection disk and current radar error ball. Footprint dimensions and rear
axle distance are vehicle-class parameters.

The sampled NRMM error comparison in the observer derivation is evaluated
in each member's inertial frame, with zero ego transport and gyro terms.
It propagates the position, velocity, acceleration and radar predictor
error bounds through the same integration steps as the point estimate.
A received position resets the predictor error radius to its sensor bound.
At acquisition let $T=t_n-t_1$, $c=(y_n-y_1)/T$, $r_i$ be the
position error radii for this ego member, and $a_*$ the Cartesian
acceleration bound of the observer design. Integration of the true
velocity over the window gives the initial velocity error radius
$$
r_v=\|\hat v_n-c\|+(r_1+r_n)/T+a_*T/2 .
$$
This also covers a target stopping inside the window. At each return, each
earlier retained position yields such a secant bound for the current velocity.
The smallest of these bounds intersects the propagated velocity error radius. If
$\|\hat v_n\|-r_v>a_*T$, the speed was positive throughout.
On that moving branch the constant-motion law gives a Cartesian jerk
bound $J=\sqrt{V_u^2\bar\omega^4+9\bar A^2\bar\omega^2}$ with
$V_u=\|\hat v_n\|+r_v+a_*T$. Let $w_i$ be the dimensionless acceleration
weights of the existing quadratic fit, and $t_m=(t_1+t_n)/2$.
Expanding position about $t_m$, the fit reproduces the quadratic terms
exactly and each cubic remainder is bounded by $J|t_i-t_m|^3/6$.
Thus $r_a$ is bounded by
$T^{-2}\sum_i |w_i|(r_i+J|t_i-t_m|^3/6)+JT/2$ and independently by
$\|\hat a_n\|+a_*$. When motion throughout the window is unresolved,
only the latter acceleration bound is used.

For a smooth integration substep of length $h$, let
$\Delta\hat p,\Delta\hat v,\Delta\hat a$ denote the accepted observer
increments. Direct integration also gives the componentwise upper bounds
$$
\begin{aligned}
r_p^+ &\le r_p+\|\Delta\hat p-h\hat v\|+h r_v+a_* h^2/2,\\
r_v^+ &\le r_v+\|\Delta\hat v-h\hat a\|+h r_a+J h^2/2,\\
r_a^+ &\le r_a+\|\Delta\hat a\|+Jh .
\end{aligned}
$$
Here $J$ uses $V_u=\|\hat v\|+r_v+a_*h$. Taking the smaller of these
bounds and the ISS comparison bounds preserves both enclosures.
If absorption can occur within a substep, the zero-derivative branch
is included separately before continuing.

Let $P$ be the inertial axis-aligned box for the target reference point.
Over a sensor interval $h_s$, intersect the displacement box
$h_s\hat v+[-b,b]^2$, $b=h_s r_v+a_*h_s^2/2$, with the box obtained
from the upper travel distance under the persistent acceleration interval
and absorbing stop. Add this displacement to $P$ and intersect the result
with each available position observation box and the NRMM position bound.
On the stationary hypothesis the displacement is zero. Its conditional
box intersects successive position observations until their intersection
is empty. For a box with midpoint $p_c$ and half-widths $w$, use $p_c$
as the planning origin. A stationary footprint gains $|R_T^\top|w$ in
body half-widths. For a turning footprint each body half-width gains
$\|w\|_2$. Range-exit checks allow the same reference-point displacement.
This construction conditions on the observer's motion mode and root speed
and heading.

Write $r_v,r_a$ for the propagated Cartesian velocity and acceleration
error radii. The NRMM parameter error map bounds the tangential acceleration
and curvature using the velocity and acceleration balls. The curvature
interval maps to sideslip through $\arcsin(l_r\kappa)$ on the physical
domain. Let $s_*$ be a lower bound on the largest target speed attained on
the current track. Both $(\|\hat v\|-r_v)_+$ and
$(\|y_j-y_i\|-r_i-r_j)_+/(t_j-t_i)$ are valid lower bounds, where
$y_i,y_j$ are the inertial radar positions for the same ego member and
$r_i,r_j$ their error radii. The latter follows from the triangle inequality
and integration of speed. For $s_*>0$, the yaw-rate premise and constant
sideslip imply $|\sin\beta|\le l_r\bar\omega/s_*$. Intersect this bound
with the physical sideslip interval. When $\|\hat v\|>r_v$, also intersect
the Cartesian inverse intervals for $A,\beta$.

For two returns separated by $T>0$, let $c$ be the measured chord length,
$r$ the sum of their position error radii, and $m$ their midpoint time.
With $\bar\omega T/2<\pi/2$, the mean speed lies in
$$
  [L,U]=\left[\frac{(c-r)_+}{T},
                \frac{c+r}{T\cos(\bar\omega T/2)}\right].
$$
The lower bound follows from arc length. For the upper bound, project the
true path onto the course at $m$. Constant sideslip and bounded yaw rate
keep the course within $\bar\omega T/2$ of that direction. If
$L>\bar A T/2$, the end speed is positive by the speed Lipschitz bound;
absorption therefore cannot have occurred. Speed is affine throughout the
window, so this interval also encloses $V(m)$. Save the first such interval
$[L_0,U_0]$ and its midpoint $m_0$ for the track. Each later eligible radar
window with $m>m_0$ supplies the further intersection
$$
  I_A\leftarrow I_A\cap
  \left[\frac{L-U_0}{m-m_0},\frac{U-L_0}{m-m_0}\right].
$$
The result holds for overlapping windows and irregular return times since
both intervals enclose speeds on the same constant-acceleration trajectory.
After using the initial short chord, replace it once with the first eligible
full-window chord to reduce the effect of position noise on the reference
speed. Keep the accumulated intersections and discard the reference upon reacquisition. Retain the intersections
on the same track, including the latent parameters through an absorbing
stop. Start new intervals at reacquisition. An empty intersection excludes
that member. The PCBF uses the resulting intervals $I_A,I_\beta$ at the stage
nodes and terminal continuation. Relative position and initial body heading
are conditioned on the retained ego member.

To enclose the target footprint at elapsed time $t$, write
$s(A,t)=\int_0^t\max(0,V_0+A\tau)\,d\tau$ on the moving branch.
The absorbing stopped branch retains its pose. For the full observer interval
$C=I_A\times I_\beta=[A_l,A_u]\times[\beta_l,\beta_u]$, monotonicity gives
$s\in[s(A_l,t),s(A_u,t)]$. Set $z=s\sin(\beta)/(2l_r)$. The offset of the
reference point and the body heading are

$$
\Delta p=s\operatorname{sinc}(z)e(\psi_0+\beta+z),\qquad
\psi=\psi_0+2z,\qquad \operatorname{sinc}(0)=1.
$$

When the observer operating domain provides speed and yaw-rate bounds, an
admissible arc also satisfies $s\le V_{\max}t$ and
$|z|\le\bar\omega t/2$. If $k_*=\min_{\kappa\in C}|\kappa|>0$ then
$s\le\bar\omega t/k_*$. These inequalities tighten the interval image
over the portion of its motion admitted by the operating domain.

When the curvature interval excludes zero, the chord length
factor also equals $2\sin(z)/\kappa$. Intersecting the two interval bounds
keeps this factor bounded after a full revolution. In this case the signed
projection onto an axis at angle $\alpha$ relative to the initial heading
also equals $[\sin(\beta+\kappa s-\alpha)-\sin(\beta-\alpha)]/\kappa$.
Intersecting its interval extension with the chord projection tightens the
reference-point enclosure on long turns.
Project these interval expressions and the four rotated footprint vertices
onto axes centered on the target heading interval. Coordinate minima and
maxima enclose the reference points in $P_C(t)$ and the complete occupied
footprints in $B_C(t)$.
Each retained ego hypothesis $s$ uses one occupied rectangle for the entire
observer interval at each active control node:
$$
\underline d_j^s=d_\pm(B_E(x_j^s),B_C^s(jh)).
$$
A known point prediction uses its physical rectangle directly. The rectangle
dual retains its continuous separating normal. An outer footprint can
underestimate clearance. When the distance from the ego reference point
to $P_C(jh)$ exceeds the sensor range, the entire target enclosure has
left the encounter. Check that node before releasing subsequent separation
obligations. A released encounter has clearance $+\infty$ for this constraint.
The terminal handoff uses the footprint and velocity enclosures for the
same interval, including admissible absorption. A measured update requires
a fresh replay when shifted containment has not been established.

For an acquired track let $d_k^-=\min_s\|p_{T,k}^s-p_{E,k}^s\|$. The
target contributes node and terminal separation constraints when $d_k^-\le R$.
Missed returns preserve the acquired posterior. After confirmed exit, the
encounter is complete and the target-free plan must still satisfy the
physical-domain and actuator constraints. First detection and re-entry both require a feasible plan for
the newly active constraints before issuing a command.

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

At each decision, size the two legs from the current surviving hypotheses.
Let $h$ be the control period and $C(t)=\lceil\Delta T\lceil t/\Delta T\rceil/h\rceil$
the allocation on the configured time grid. For a target-free hypothesis of
ego speed $v$, the avoidance-leg request is $A_s=\max(2h,v/a_p+T_s)$ and
the connection request is $B_s=0$. Here $a_p$ is a preview deceleration
scale and $T_s$ is the response allowance.

For an active target, let $d=p_T-p_E$, $u=\dot p_E-\dot p_T$ and
$e_T$ be the target's unit course vector including sideslip. The relative
speed along that course is $c=e_T^\top u$. For $c>0$, set
$t_c=[e_T^\top d]_+/c$ and $t_P=L/c$, where $L=2(a_E+a_T)$ and each
footprint radius is its half-diagonal plus the norm of its reference-point
offset. These times describe catching up to the target's longitudinal
position and gaining a combined footprint span at the current velocities.
For $c\le0$ both are infinite, so this passing mechanism does not shorten
the request.

Let $w$ be the sum of the ego half-width, the target half-extents projected
onto the ego lateral axis, and the requested clearance. With the guidance
model's available lateral acceleration $a_y$, define $t_L=2\sqrt{w/a_y}$.
Two opposite accelerations of magnitude $a_y$ cover $w$ in this time with
zero initial and final lateral speed. The connection uses the same sizing
model with the orientation-independent displacement $L$. The two requests
in seconds are
$$
A_s=\max\left(2h,\frac{L}{\max(v,v_{\rm ref})},
       \min\left(\max(v/a_p+T_s,t_L),t_c+t_L\right)\right),
\qquad B_s=\min(t_P,2\sqrt{L/a_y}).
$$
The spatial scale uses circumscribed radii only for duration selection.
The optimizer uses the nonlinear vehicle model, the persistent target motion
contract and the rectangle-distance dual for its actual predictions.
The time requests are engineering estimates. They do not certify reachability
of the terminal set.

For $A^*=C(\max_s A_s)$ and $B_k=C(\max_s B_s)$, set
$$
N_k=\max(N_{\rm suffix},A^*+B_k),\qquad A_k=N_k-B_k.
$$
At startup and upon acquisition after target-free cruise, $N_{\rm suffix}=0$.
The cruise plan has no certificate for the newly introduced target. On a
continuing encounter, $N_{\rm suffix}=N_{k-1}-1$ retains the old terminal
endpoint. Any required extension, including the terminal handoff, is subject
to the ordinary feasibility and deficit-decrease checks.

Use integer boundaries $0=c_0<c_1<\cdots<c_M=N$ with
$b_i=c_i-c_{i-1}$ and a knot at $A_k$. On the avoidance leg the first few
intervals have one hold each. Distribute the remaining holds to rounded
equally spaced boundaries, up to the configured knot count $M_{\max}$. On the
connection use $\min(M_{\max},\lceil B_k/\max_{c_i\le A_k} b_i\rceil)$
additional, equally spaced knots rounded to holds.
Each endpoint command $v_i\in\mathbb R^2$ is an independent decision.
Physical states are eliminated by forward
substitution of the dynamics, separately for every retained hypothesis.
Let $L(v)$ interpolate the endpoint commands linearly between adjacent
knots, starting at $v_0=u^-$. Initialize $\bar u_{1:A_k}$ from one field
rollout. Initialize $\bar u_{A_k+1:N_k}$ by moving toward the nominal cruise trim
within the physical actuator limits. Let $\bar v_i=\bar u_{c_i}$. The encounter solve
interpolates corrections about that rollout,

$$
u_{c_{i-1}+j}=\bar u_{c_{i-1}+j}
 +(1-j/b_i)(v_{i-1}-\bar v_{i-1})+(j/b_i)(v_i-\bar v_i),
\qquad \bar v_0=u^-,\quad j=1,\ldots,b_i .
$$

At $v=\bar v$ this reproduces every held field command. The target-free
knot model uses $u=L(v)$. Both representations are affine in $v$, so
physical bounds on $u_j$ and any configured increment bounds on
$u_j-u_{j-1}$ are linear constraints. For a corrected interpolation,
the off-knot rows must account for $\bar u-L(\bar v)$.
Each command is held for the complete period $h$.

Let $\widetilde F_h$ denote the numerical held-input flow, including the
rest reset only at the control boundary. Define the condensed trajectories

$$
x_0^s=x_k^s,\qquad x_{j+1}^s=\widetilde F_h(x_j^s,u_{j+1},\vartheta^s).
$$

The same recursion carries $x_{A_k}^s$ directly into the connection.
One optimization chooses both sets of commands. The dual clearance,
input, physical-domain and PCBF deficit constraints cover both legs.
Membership in $Z_f$ is imposed at $x_{N_k}^s$. The fixed continuation
used to test that membership starts at $N_k$.

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

The nonlinear validation flow samples the physical domain on a fixed base
mesh and checks collision at hold boundaries. Let $\Delta=h/K_v$ and let
$[v_1,v_2]$ be the kinematic-to-Fiala blend interval. To select a numerical
step, extrapolate the current longitudinal coupling and road load by
$b=\Delta(v_y r-F_{\rm load}(v_x)/m_E)$, and let
$a=\Delta g\max(\mu_f,\mu_r)$ use the member's axle friction coefficients.
When $\Delta>h_s/2$ and the convex hull of
$\{v_x,v_x+b-a,v_x+b+a\}$ intersects $[v_1,v_2]$, compose two RK4
half steps. Otherwise take one RK4 step. This interval selects integration
resolution and carries no reachable-set guarantee. Local flow
sensitivities propagate $S_{j+1}=F_x S_j+F_uD_v u_j$. For an IPOPT search
direction the tangent flow uses $\min(K_v,\max(2,\lceil h/(2h_s)\rceil))$
RK4 substeps per hold, where $h_s$ is the sensor period. At the default
timing this gives two search tangent substeps and four nonlinear base intervals.
Standalone evaluation of a specified input sequence differentiates its
validation mesh. The held inputs reconstructed from a trial must pass the original nonlinear
validation. Neither mesh supplies a certified integration-error bound.

With the default timing the causal measurement-consistency update advances
the ego state by the same numerical map over each sensor interval under the held command.
It evaluates the sensor equations and updates the target at each reading.
The absorbing ego reset applies only at the control boundary.

The compiled flow graph allocates capacity from configuration size hints and
the preview rule. These hints are not planner velocity constraints. A longer
preview uses consecutive blocks of the same graph; unused slots are identities. Updating the active
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
When $d_{\rm safe}=0$, strict separation requires a witness with a strictly
positive minimum over the vertex projections above. The compact controller
checks the sign of the optimized signed distance.
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
 +\frac{w_u}{2}\sum_{j=1}^N\|u_j-u_{j-1}\|^2,\qquad u_0=u^- .
$$

The compact default is $W=\operatorname{diag}(0.02,0.2,2,0.1,0.1)$ and
$w_u=0.1$. The lower transverse weight permits a passing excursion while
retaining the fixed cruise path and speed as the zero-error reference. This
performance choice does not supply a general convergence or liveness theorem.

The guide offset $\bar u-L(\bar v)$ vanishes at both ends of each knot
interval. Its increments sum to zero there, so it adds a constant to the
knot increment cost $\tfrac{w_u}{2}\sum_i\|v_i-v_{i-1}\|^2/b_i$.
For the node-only collision contract, define

$$
C_j(v)=\max_s\max_{r\in\{j,j+1\}}
\{d_{\rm safe}-\underline d_r^s\},\qquad j=0,\ldots,N-1.
$$

Each stage covers its two control-boundary nodes; their union includes the
current root and the terminal node. No interior collision sample or motion
reserve enters this deficit. The primary problem is

$$
\min_{v,\xi\ge0}\ \sum_{j=0}^{N-1}\xi_j,\qquad
C_j(v)\le\xi_j\le d_{\rm safe},\qquad v\in\mathcal U_N,\qquad
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
The primary problem has $2M+N$ variables. The nonnegative longitudinal
speed domain includes rest. When braking enters the model's stop reset,
the post-reset state is propagated through the remaining holds, with the
same node clearance, PCBF deficit and terminal membership conditions.

A feasible zero-deficit driving sequence attains the lower bound of the PCBF
objective. Define a secondary cost over its $N$ held inputs. Write $b_j$ for
the signed longitudinal-force ratio on hold $j$ and $s$ for a retained
physical hypothesis. At the end of that hold, form the dimensionless residual
$$
 a_{j+1}^s=
 \frac{C_r^s}{\mu_r^s F_{zr}}\,
 \operatorname{atan2}\!\left(v_{y,j+1}^s-l_r r_{j+1}^s,
             \sqrt{(v_{x,j+1}^s)^2+v_{\rm floor}^2}\right).
$$
The existing scheduling speed floor regularizes the slip measure near rest.
The factor $\mu_r^s F_{zr}/C_r^s$ sets the angular scale from each member's
rear tire parameters. Define
$$
 J_{\rm tire}(U)=\frac h2\sum_{j=0}^{N-1}
 \left[b_j^2+b_j^4+\frac1S\sum_{s=1}^S(a_{j+1}^s)^2\right].
$$
Here $b_j^2$ measures squared longitudinal force utilization. Since
$D_r^2/(\mu_rF_{zr})^2=1-b_j^2$, the quartic term emphasizes the loss of
lateral force capacity at large traction or braking. All terms are even
under lateral reflection. These terms affect preference within the available
actuator range. The PCBF primary objective and the terminal membership
condition are independent of their values.

For two validated zero-deficit plans, a proposed replacement must strictly
reduce $J_{\rm tire}$ and satisfy $J_{\rm new}\le J_{\rm incumbent}$ up to
numerical tolerance. This comparison preserves a feasible incumbent when
stability and reference tracking favor different plans. It supplies a local
preference and makes no claim of global secondary optimality or yaw stability.

### A terminal set for sustained collision avoidance

Let \(R\) be the radar range for the two reference points, \(h\) the control
hold, and \(d_\pm\) the signed rectangle distance from the separation dual.
A detected encounter ends at the first control node with
\(\|p_T-p_E\|>R\). Equality is in range. Predicted exit can complete a
terminal witness. Actual release still uses the received block and every
surviving member. An isolated missed return inside range preserves the
obligation. Re-entry is a new encounter.

During an active encounter every member receives the same realizable terminal
command $u_f=(0,0)$ (straight steering and neutral drive), which leaves a
stationary ego at rest in the physical model. For an active member, check
actuator compatibility and one physical hold under $u_f$,
including its model domain and the clearance at both nodes. The terminal
set is the one-step preimage, under this common command, of release or the
following **separating-motion set**.

At handoff let $E$ be the oriented ego footprint and let $T_C$ be the
occupied rectangle of the target interval for an active member. In the terminal
model the footprints keep their orientations. The ego translates with
$v_E=v_x e(\psi_E)$, $v_x\ge0$. The target translates with a constant
velocity in its handoff disk $D(c_C,r_C)$. A point target has $r_C=0$.
Obtain the speed and course intervals from the constant-acceleration,
constant-sideslip arc up to handoff, including absorption and the declared
physical bounds. If speed lies in $[l,u]$ and course lies within $\delta$
of $\phi$, set $q=\cos(\min(\pi,\delta))$. The minimum enclosing disk
has center $c_C=m e(\phi)$ by symmetry. For $m\ge0$, the maximum
squared distance is attained at an angular extreme and one of the two
radial endpoints. Minimizing the larger of these two quadratics gives
$m=0$ if $q\le0$ and $m=\min(uq,(l+u)/(2q))$ otherwise. Its radius is
$$
r_C^2=\max_{s\in\{l,u\}}\{(s-m)^2+4sm\sin^2(\min(\pi,\delta)/2)\}.
$$
This encloses all handoff velocities. The frozen continuation applies only
after the two optimized legs and the physical handoff hold.

Write $w_C=v_E-c_C$ and $G_C(n)=\min_{a\in E}n^\top a-
\max_{b\in T_C}n^\top b$. Membership requires, **for each active ego hypothesis**,
$$
\exists\,\|n_C\|=1:\quad
G_C(n_C)\ge d_{\rm safe},\quad G_C(n_C)>0,\quad n_C^\top w_C\ge r_C .
$$
Each ego hypothesis may have its own geometric witness. All members use $u_f$.
On this simplified model the signed projection gap can only increase
or stay constant indefinitely, including when both vehicles remain at rest.
When $w_C=0$ and $r_C=0$, every normal satisfies the velocity inequality,
so membership reduces to the current footprint clearance. When
$\|w_C\|=r_C>0$, the only admissible normal is $w_C/\|w_C\|$.
When $\|w_C\|<r_C$, the velocity disk admits closing motion along every
normal. If $\|w_C\|>r_C$, a finite exit time can also be reported.
In that last case, for an initial reference-point distance bounded by $L_C$, any time
strictly greater than $(R+L_C)/(\|w_C\|-r_C)$ suffices.

The opening condition restricts the rectangle dual normal to an arc of
the unit circle. First evaluate the unconstrained rectangle-distance dual.
If its maximizing normal violates the opening condition and the rectangles
are separated, the two endpoints of the opening arc suffice for testing
positive clearance. Indeed, on the arc of normals with positive gap,
each vertex-pair projection is concave in normal angle, as is their minimum.
Writing $e=w_C/\|w_C\|$, $b=r_C/\|w_C\|$, the endpoints are
$n_\pm=b e\pm\sqrt{1-b^2}Je$. Evaluate the exact footprint supports
at both. The work per ego hypothesis is independent of the time to exit.

For the nonlinear vehicles, **terminal safety and closure under $u_f$ are
handoff assumptions**. They cover continued safe motion while the encounter remains active, admissible
actuation, and preservation of terminal membership under the next physical
hold and compatible information update. The real target continues to obey
its original motion contract. After release, nominal guidance owns path
and speed recovery.

At encounter acquisition, integrate the route-following circulation field
once using the NRMM target forecast and the occupied rectangle at each
preview node. The same full parameter interval enters the PCBF.
During an active encounter its shifted fine commands supply the origin
for new actuator corrections, so zero correction reproduces the available
suffix exactly.

The combined initial commands $\bar u$ define the initial shooting point for a new
encounter. Propagating them from each member gives the footprint poses
for the signed dual in Section 5. Its value and an attaining feature
supply the first stage deficit rows and their control Jacobians. These
rows are defined at penetration as well as at separation. A positive
$C_j(\bar v)$ is a residual at the initialization point. The initial
trajectory may penetrate the target. Subsequent evaluations use the
attaining dual at the new vehicle poses.

Let $H(v)\le0$ collect the nonlinear model-domain and terminal constraints,
and let $L$ be the remaining inherited deficit, infinite on admission.
CasADi transcribes the actuator interpolation and bounds. IPOPT obtains
the condensed trajectory values and their active-branch Jacobians from
the shared nonlinear prediction. A diagonal change of variables scales
steering by its friction-limited turn angle and leaves the feasible set
unchanged. IPOPT selects search directions and step lengths.

For fixed controls the smallest feasible individual slack is
$\xi_j^*(v)=[C_j(v)]_+$. Eliminating these variables gives the same primary
optimization over actuator knots:
$$
\min_v D(v)=\sum_j[C_j(v)]_+,\qquad
\max_j C_j(v)\le d_{\rm safe},\quad D(v)\le L,\quad
H(v)\le0,\quad v\in\mathcal U_N .
$$
On admission the infinite cap is omitted. With $L=0$ the stage inequalities
reduce to $C_j(v)\le0$ for every $j$. The boundary of the feasible set
includes contact when a stage slack equals $d_{\rm safe}$; dispatch still
requires strictly positive measured node clearance. At a feature or active
stage change the reduced functions can be nonsmooth. The local solver uses
the supplied branch sensitivities; convergence to a global minimum is not
assumed.

For a zero-deficit feasibility proposal, let $r=([C(v)]_+,[H(v)]_+)$
collect the stage, physical-domain and terminal violations. Subject to the
actuator bounds, define the nonnegative residual cost
$P(r)=\sum_i (\sqrt{1+r_i^2}-1)$. When only clearance is deficient at the start of the proposal solve, use
$b=d_{\rm safe}$ as the numerical transition scale. Otherwise use $b=0$
to retain the original scaling for mixed physical and terminal violations.
Keep $b$ fixed throughout the solve. Let $\epsilon>0$ be the NLP tolerance and
$k=\sqrt{b^2+\epsilon^2}$. IPOPT uses the increasing transformation
$$
F(P)=
\begin{cases}
 k(\sqrt{2P+\epsilon^2}-\epsilon), & 2P < b^2,\\
 P-b^2/2+k(k-\epsilon), & 2P\ge b^2.
\end{cases}
$$
The slopes agree at the join. Above it, the trajectory gradient is the
original gradient of $P$. Below it, the gradient is multiplied by
$k/\sqrt{2P+\epsilon^2}$, allowing small residuals to produce a numerically
useful search direction. $F(0)=0$, and the zero set and ordering of proposals
are preserved, including at $b=0$. Zero attains the
primary lower bound. A positive residual by itself has
no dispatch authority. With several ego members, the proposal can start
with the member having the largest squared violation on the current
trajectory and add a violating member after full replay. Each member
retains its complete target parameter cover. A committed trial must pass
the original node, terminal, physical-domain and deficit-decrease tests
on every retained member. The original capped primary remains available
when the zero level has not been attained.

The numerical primary lower bound is zero or the unavoidable current-node
deficit. An admissible iterate attaining this bound may end the primary
solve. The online search returns the validated plan on attaining this
bound. When comparing available validated plans at equal deficit, it uses
$J_{\rm tire}$ with $J(v)\le J_{\rm incumbent}+\varepsilon$ for the tracking cost.
A local or interrupted solve can retain a verified incumbent at its actual
stage deficits. Section 7 specifies the nonlinear dispatch and decrease
conditions.

## 7. Node acceptance and retention of executable plans

For each retained physical member, expand the proposed coarse commands into
all $N$ held commands and propagate its own persistent parameters. Fine RK4
replay uses $K_v=4$ base intervals per hold by default, with the held command and the
rest reset applied at control boundaries. This is the integration and
physical-domain mesh; it does not define additional collision constraints.
Every new executable proposal is checked for physical input bounds,
configured input increments, and the physical-model domain.

Let $z_j^s=(x_j^s,q_j^s)$ be the joint state at $t_k+jh$. While the target
is active at decision $k$, each proposed plan must satisfy

$$
\underline d_r^s>0,\qquad \underline d_r^s\ge d_{\rm safe}-\xi_j,\qquad
r\in\{j,j+1\},\quad j=0,\ldots,N-1,\quad s=1,\ldots,S .
$$

Replay assigns $\xi_j=[C_j(v)]_+$ using the lower bound $\underline d_r^s$
at the held-command boundaries for each uncertain member. A point member
uses its exact rectangle-distance dual. On the terminal bridge and in the terminal
set the full $d_{\rm safe}$ remains hard for the duration of the encounter. The default
planning margin is $0.10\,\mathrm{m}$. The simulation counts
contact or overlap at a control node when the true signed distance is
nonpositive. A trajectory can
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
horizon. On initial admission after observation, or on re-entry, the collision obligations increase. The shift proof
below applies through that boundary only if the incoming state and
continuation are feasible for the reactivated encounter. The controller
constructs the avoidance initialization from one field rollout at the
physical hold period and jointly optimizes the connection. A plan returned by that solve must pass the complete
nonlinear checks before supplying a command. Re-entry feasibility thus
requires a separate admission assumption. Refreshing a target point from new
measurements can also move it outside the previously predicted successor
set. The numerical controller rechecks the suffix from the updated root.
The observer point alone supplies no successor-containment guarantee.

After applying the first input, let $r=N_{k+1}-(N_k-1)\ge0$. Starting
from $w_0=u_{N_k-1|k}$, let $w_j$ move toward $u_f$ within one hold's
actuator increment bounds. Active terminal entry requires $u_f$ to be
reachable in that hold, in which case every appended $w_j=u_f$. The
executable candidate is
$$
 U_{k+1}^{\rm shift}
   =(u_{1|k},\ldots,u_{N_k-1|k},w_1,\ldots,w_r).
$$
The shift retains the optimized inputs on both sides of $A_k$, including the
complete unexecuted connection. It appends terminal-law inputs only beyond
the old terminal endpoint. The fine held-input suffix remains directly representable even when it no
longer interpolates the new control knots. Generated candidates and this
shift must each pass the same checks over the complete selected horizon.
An optional new guide input is accepted only with a feasible continuation.
During an active encounter, when both the guide splice and the shifted plan
can be checked in time, the shift replaces the splice if it reduces the tire
cost while preserving tracking performance. If only one plan is validated,
that plan supplies the next command. Increasing $d_{\rm safe}$ requires
feasibility to be established under the new margin before a command is issued.

Suppose an initial plan is feasible, the posterior contains the true member
and is contained in the predicted successors, the flow and timing agree,
the required clearance stays fixed, requested horizons fit the numerical
allocation, and the core handoff assumption holds. Let $z\in Z_f$.
A shift consumes the checked handoff hold. A surviving member reaches
the separating-motion core. The physical handoff assumption supplies the
next safe hold under the same command and another core or release witness.
This gives the required closure conditional on that assumption.
Admissibility of the subsequent nominal handoff and
admission on a new detection are separate assumptions for continuing
operation. Induction gives any $r$ required extension holds under a
single common input. Conditioning can discard physical members, and all
surviving members inherit their own witnesses. When $r=0$ the old endpoint
is already in $Z_f$.

The shifted feasible plan inherits the old stage deficits after the executed
stage and appends $r$ zeros. On a shift-closed policy domain this gives
$$
 V_{N_{k+1}}(\mathcal I_{k+1})
 \le V_{N_k}(\mathcal I_k)-\xi^*_{0|k}.
$$
For a stored feasible iterate, use its replayed sum $J_k=\sum_j\xi_{j|k}$.
On a continuing encounter with unchanged margin, a replacement must satisfy
$J_{k+1}\le J_k-\xi_{0|k}$ up to arithmetic roundoff. A zero remaining
budget fixes every new slack to zero. When observation updates change
the sampled roots, this check still governs execution, but existence of
an acceptable replacement requires a separate containment argument.
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

The NRMM observer's conditional derivation is in
[OBSERVER_ISS_THEORY.md](../estimator/OBSERVER_ISS_THEORY.md).
