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
During acquisition the ego follows its nominal cruise controller. For a
populated window with resolved displacement, let $T=t_n-t_1$ and
$\tau_j=(t_j-t_n)/T$. The least-squares fit
$p_j\simeq c_0+\tau_j c_1+\tau_j^2 c_2/2$ supplies the current Cartesian
state $(c_0,c_1/T,c_2/T^2)$ to the high-gain observer and inverse above.
A stationary-compatible window can instead initialize $(\bar p,0,0)$
with a measured relative body heading added to ego yaw. In the simulation,
a detected target supplies ideal relative body heading near rest, defined by
$V_T T_{\rm acq}\le 2r_{\rm sensor}$ for the acquisition window span and
the nominal radar position error radius. Each ego member converts a received heading to inertial heading
using its own yaw at the measurement time. With position alone,
stationary acquisition remains pending. Avoidance requires a complete target
estimate for every retained member. For a newly observed moving target, the
compact controller also requires $\min_s t_{\mathrm{last},s}-t_{\mathrm{first}}
\ge T_{\mathrm{obs}}$, where the times refer to received radar measurements
and $T_{\mathrm{obs}}=0.40\,\mathrm{s}$ by default. The observer continues
updating during this interval. A complete estimate compatible with rest, or
a target already known under the initial contract, can enter planning directly.
The acquisition window is a timing condition. Safety during nominal cruise
before the first feasible encounter plan is an initial-condition assumption.
At a positive return the planning root retains its previous observer-based constant-motion
forecast while the predicted position explains the measurement and range.
A received body heading updates the forecast orientation even when its position is retained.
When all positions in the acquisition window lie within the radar noise
bound of their mean, a stationary nominal forecast uses the perceived body
heading when available and otherwise retains its last resolved heading. The moving point observer continues to
process returns. Below the Cartesian jerk regularization speed, when no body heading is received,
use a resolved chord between the earliest
and latest retained returns within a quarter turn. For chord vector $c$ and
representative curvature $\kappa=\sin\beta/l_r$, its endpoint body heading is
$\arg c+\arcsin(\kappa\|c\|/2)-\beta$. The representative sideslip is the
midpoint of the retained interval. An unresolved chord supplies no new angle.
This stationary-compatible window does not identify a
unique physical speed. Initializing the target observer, changing the
stationary hypothesis or receiving a return inconsistent with the forecast
refreshes the root. Its filtered center is constrained to both the detection
disk and the current radar error ball. Footprint
dimensions and rear axle distance are declared vehicle-class parameters.

On a retained short arc, let $t_i,y_i,r_i$ denote the time, position and
declared error radius of return $i$. A chord $c_{ij}=y_j-y_i$ has error at
most $e_{ij}=r_i+r_j$. For $T_{ij}=t_j-t_i$ and
$b_{ij}=\bar\omega T_{ij}/2$, the traveled arc length satisfies
$$
 (\|c_{ij}\|-e_{ij})_+\le s_{ij}\le
 (\|c_{ij}\|+e_{ij}) b_{ij}/\sin b_{ij}.
$$
A current curvature bound $K$ also gives the short-arc upper bound
$2\arcsin(K(\|c_{ij}\|+e_{ij})/2)/K$ when its argument is below one.
A later resolved displacement proves that the target was still moving
at the start of that displacement. For all pairs ending by that time,
$s_{ij}/T_{ij}=V_0+A((t_i+t_j)/2-t_0)$. These strips share one unknown
$V_0$. For the angular update, let $s_i$ denote travel from the first
return. The true chord course equals $\theta_0+\kappa(s_i+s_j)/2$.
A resolved chord observes that course within $\arcsin(e_{ij}/\|c_{ij}\|)$.
Bounds on $s_i,s_j$, the quarter-turn observation window and the two
curvature signs give affine strips sharing one unknown initial course.
Only unambiguous angular lifts are used. For either update, at a proposed
slope the largest lower intercept cannot exceed the smallest upper
intercept. Each violation yields a necessary slope bound. Incomplete
elimination retains an outer interval. Measured body headings can also
constrain curvature through $2\sin(\Delta\psi_T/2)/\|c_{\rm true}\|$,
with turn uncertainty $2r_\omega\Delta t$ under the gyro contract.

At acquisition, a joint constraint follows from the same returns. Use the
first-to-last chord as the longitudinal axis and the observation time
midpoint as origin. The midpoint speed is $s_{1n}/T\in[V_l,V_u]$ when
$V_l>\max|A|T/2$ establishes forward motion throughout the window.
The midpoint course differs from the measured chord course by at most
$\delta=\arcsin(e_{1n}/\|c_{1n}\|)+K\max|A|T^2/8$. Under constant
$A,\kappa$, the tangential and normal jerk components are
$-\kappa^2 V^3$ and $3\kappa V A$. Let $J$ bound their resultant using
$V\le V_u+\max|A|T/2$. The mean Cartesian velocity over $[a,b]$,
in midpoint-relative time, differs from $v_m+\dot v_m(a+b)/2$ by at
most $J(a^2+ab+b^2)/6$. Projecting all measured chords onto a direction
therefore bounds the corresponding component of $\dot v_m$.

The position constraints can tighten this bound. All returns share the
six Cartesian coefficients of $p_m+v_m t+\dot v_m t^2/2$. Each measured
position bounds its projection on every chosen unit normal with radius
$r_i+J|t_i|^3/6$. In scaled coordinates these rows have the form
$Hz\le b$. For any nonnegative dual vector $\lambda$ and known
$|z|\le Z$, the valid directional upper bound is
$\lambda^\top b+|c-H^\top\lambda|^\top Z$ for $c^\top z$. The chord
bounds remain available when this optional linear calculation is unavailable.
Let $S=(V_l^2+V_u^2)/2$. In the true midpoint course frame the
acceleration vector is $(A,V_m^2\kappa)$. Rotating through the bounded
angle $\delta$ and allowing $|V_m^2-S|\le(V_u^2-V_l^2)/2$ gives linear
support inequalities for $(A,S\kappa)$. These inequalities persist for the
same associated target across missing returns under its constant-parameter
contract. The moving-parameter evidence survives absorption while the
stopped branch separately propagates the stationary pose. A new target
starts a new track. At low speed, a sideslip inverse clipped to its domain
limit uses the midpoint of its measured sideslip interval as the point
output, minimizing worst absolute error over that interval.

For each retained physical member, the compact predictor uses one NRMM
forecast and the closed intervals $I_A,I_\beta$. The center is clipped to
those intervals. Each acquired support is relaxed, if necessary, to also
include that center. The continuous parameter cells cover the intersection
of $I_A\times I_\beta$ with these supports evaluated at
$\kappa=\sin\beta/l_r$. All surviving combinations are propagated from
the member's estimated root position, heading and speed. Errors in those
root quantities are covered only to the extent represented by the members.

To enclose the target footprint at elapsed time $t$, write
$s(A,t)=\int_0^t\max(0,V_0+A\tau)\,d\tau$ on the moving branch.
The absorbing stopped branch retains its pose. For any closed parameter
cell $C=[A_l,A_u]\times[\beta_l,\beta_u]$, monotonicity gives
$s\in[s(A_l,t),s(A_u,t)]$. Set $z=s\sin(\beta)/(2l_r)$. The offset of the
reference point and the body heading are

$$
\Delta p=s\operatorname{sinc}(z)e(\psi_0+\beta+z),\qquad
\psi=\psi_0+2z,\qquad \operatorname{sinc}(0)=1.
$$

On turning cells whose curvature interval excludes zero, the chord length
factor also equals $2\sin(z)/\kappa$. Intersecting the two interval bounds
keeps this factor bounded after a full revolution. On such cells the signed
projection onto an axis at angle $\alpha$ relative to the initial heading
also equals $[\sin(\beta+\kappa s-\alpha)-\sin(\beta-\alpha)]/\kappa$.
Intersecting its interval extension with the chord projection tightens the
reference-point enclosure on long turns.
Project these interval expressions and the four rotated footprint vertices
onto axes centered on the cell's heading interval. Coordinate minima and
maxima enclose the reference points in $P_C(t)$ and the complete occupied
footprints in $B_C(t)$. Over the preview $T$, let
$L=V_0T+\max(0,A_u)T^2/2$. Integrating the course variation along the
arc bounds the pairwise position spread by
$D=(A_u-A_l)T^2/2+L(\beta_u-\beta_l)+L^2(\kappa_u-\kappa_l)/2$.
The predictor uses four subdivisions per parameter by default. When $D$
exceeds the sensor radius, it uses eight acceleration and sixteen slip
subdivisions and splits any cell that straddles zero curvature. Every
subdivision is closed and the union covers the same motion family. At each control node the clearance
lower bound is

$$
\underline d_j^s=\min_{C\ \mathrm{active}} d_\pm(B_E(x_j^s),B_C^s(jh)).
$$

A known point prediction uses its physical rectangle directly. An empty
active set has clearance $+\infty$. The rectangle dual retains its
continuous separating normal. Using an outer
footprint can underestimate clearance. If the distance from the ego
reference point to $P_C(jh)$ exceeds the sensor range, the entire cell
has left this encounter. Check its exit node and discharge its subsequent
obligations. The terminal bridge and straight continuation carry the same
constant parameter cells forward from the planning root, including cells
whose target can brake to rest. A measured update requires a fresh replay
when shifted containment has not been established.

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

For speed bound $v_k^+=\max_s\sqrt{(v_{x,k}^s)^2+(v_{y,k}^s)^2}$ choose

$$
T_k^{\rm req}=\max\{T_{\min},2h,v_k^+/a_p+T_s,T_{\rm space}\},\qquad
A_k^{\rm req}=\left\lceil\frac{\Delta T\lceil T_k^{\rm req}/\Delta T\rceil}{h}\right\rceil .
$$

During an active encounter let $B_k=\lceil T_{\min}/h\rceil$ be the duration
in holds of the optimized connection to the terminal set. In target-free
cruise take $B_k=0$. The avoidance and total lengths satisfy

$$
N_k=\max\{N_{k-1}-1,A_k^{\rm req}+B_k\},\qquad A_k=N_k-B_k .
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
rollout. Initialize $\bar u_{A_k+1:N_k}$ by moving toward terminal trim
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

The nonlinear validation flow uses a fixed RK4 mesh, with physical-domain
checks inside each hold and collision checks at its boundaries. Local flow
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

### A terminal set for safe exit from sensing range

Let \(R\) be the radar range for the two reference points, \(h\) the control
hold, and \(d_\pm\) the signed rectangle distance from the separation dual.
A detected encounter ends at the first control node with
\(\|p_T-p_E\|>R\). Equality is in range. Predicted exit can complete a
terminal witness. Actual release still uses the received block and every
surviving member. An isolated missed return inside range preserves the
obligation. Re-entry is a new encounter.

While terminal continuation is needed, every member receives the same
realizable command
$$
 u_f=(0,b_f),\qquad b_f=\operatorname{clip}_{[b_{\min},b_{\max}]}
 \frac{F_{\rm load}(v_{\rm ref})}{\sum_i\mu_i F_{zi}} .
$$
A member already outside at the planning endpoint has zero remaining
encounter steps. Otherwise check compatibility with the previous actuator
input and advance the nonlinear dynamics under \(u_f\) for up to
\(B=\max(1,\lceil T_{\rm preview,min}/h\rceil)\) holds. Include each
node through the first exit in the clearance checks and apply the vehicle
domain checks on these holds. If exit occurs, the encounter witness is
complete at that node.

For a member still in range after the bridge, the simplified continuation
under the same command starts with the handoff pose, fixed heading
\(\psi\), and positive handoff longitudinal speed \(v\). Its sampled model
and exit index are
$$
 p_{E,j}=p_{E,0}+jh\,v(\cos\psi,\sin\psi)^\top,\qquad
 q_j=\Phi_T(q_0,jh),\qquad
 J=\min\{j\ge 0:\|p_{T,j}-p_{E,j}\|>R\}.
$$
Here \(\Phi_T\) uses the same constant target parameters and absorbing stop
as the main forecast. The core requires finite \(J\) and
$$
 d_\pm(E_j,T_j)>0,\qquad d_\pm(E_j,T_j)\ge d_{\rm safe},
 \qquad 0\le j\le J .
$$
Each dual compares the two oriented footprints at the **same** node. The
core residual is \(d_{\rm safe}-\min_{0\le j\le J}d_\pm(E_j,T_j)\).
On a fixed exit and closest-feature branch, its derivative follows from
the active node's rectangle dual and the derivative of the straight
translation with respect to handoff position, heading and speed.

For affine relative motion the range exit follows from a line–disk
intersection. Signed clearance is convex along a translational line with
fixed orientations, so its discrete minimum can be located by bisection.
Equal velocities inside the disk give no exit. For turning or accelerating
targets, process node intervals chronologically. An interval can be skipped
when bounds on relative translation and target rotation show that every
node is in range and its clearance cannot improve the current minimum.
For a half-duration \(\Delta\), the reference-point displacement from the
midpoint is bounded by
\(\|v_{\rm rel}\|\Delta+(|A_T|+|\kappa_T|V_{\max}^2)\Delta^2/2\);
the additional corner displacement is bounded by
\(a_T\min(2,|\kappa_T|V_{\max}\Delta)\). Otherwise subdivide or evaluate
the original node duals. This preserves the first exit even when both
endpoints of a larger interval are in range.

Under the simplified continuation, advancing one hold before exit
decrements \(J\) and retains the previously checked suffix. Exit is the
completed state for this encounter. For the full nonlinear vehicle,
preservation of this safe-to-exit continuation at the bridge handoff is a
design assumption. Numerically the controller replays the candidate for
each surviving member. Once the encounter ends, nominal guidance owns
path and speed recovery.

The geometric guide integrates one route-following field. Its
preferred acceleration includes the transport and separation
gradients of the predicted encounter. For an uncertain moving target, use
path coordinates to compute the longitudinal overlap times for its footprint enclosure. At such a time \(t\), let \(y^s(t)\) denote its signed lateral
displacement from the ego footprint center extrapolated with its current
lateral velocity. Let \(r^s(t)\) be the sum of the projected lateral
half-widths and the planning clearance. Under a first-order yaw response
with time constant \(T\), the lateral displacement response to one unit
of corrective acceleration is approximated by
\[
D(t)=\max\{h^2/2,\ t^2/2-Tt+T^2(1-e^{-t/T})\}.
\]
The guide takes the interval hull of
\([(y^s-r^s)/D,(y^s+r^s)/D]\) over the overlap times of the retained
parameter cells, up to each cell's range exit on the extrapolated ego path.
Let $a_r$ be the route acceleration before the nominal target correction.
When its transverse component relative to path curvature feedforward lies
inside the hull and has a unique nearest endpoint, project that component
onto the endpoint and retain the longitudinal component of $a_r$. The
nominal target is already included in the enclosure. Otherwise the guide
uses its nominal circulation field. Both use the same cruise reference.
For uncertain motion, each cell contributes to the field until its own
range exit. When $D$ exceeds the sensor radius $R$, the initial guide
blends its acceleration with a velocity correction directed away from
the forecast target center, with weight $\min(1,\max(0,D/R-1))$.
This affects only the initial controls. Every trial uses its own poses
in the distance dual of Section 5.

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

The numerical primary lower bound is zero or the unavoidable current-node
deficit. An admissible iterate attaining this bound may end the primary
solve. The secondary problem minimizes $J_{\rm tire}$ on the attained
level with $J(v)\le J_{\rm incumbent}+\varepsilon$ for the tracking cost.
A local or interrupted solve can retain a verified incumbent at its actual
stage deficits. Section 7 specifies the nonlinear dispatch and decrease
conditions.

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
\underline d_r^s>0,\qquad \underline d_r^s\ge d_{\rm safe}-\xi_j,\qquad
r\in\{j,j+1\},\quad j=0,\ldots,N-1,\quad s=1,\ldots,S .
$$

Replay assigns $\xi_j=[C_j(v)]_+$ using the lower bound $\underline d_r^s$
at the held-command boundaries for each uncertain member. A point member
uses its exact rectangle-distance dual. On the terminal bridge and in the terminal
set the full $d_{\rm safe}$ remains hard through range exit. The default
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
If a bridge hold reaches exit, the encounter is complete. Otherwise one
step consumes a checked bridge hold. At the far end of a full bridge, the
handoff assumption preserves the core witness for one more hold or
establishes exit. This gives closure in the encounter state augmented with
its completion flag. Admissibility of the subsequent nominal handoff and
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
