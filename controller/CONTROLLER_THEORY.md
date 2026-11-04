# Unified nonlinear output-feedback safety theory

The primary design jointly optimizes a causal control map and a bounded
controller-memory update. A separate point observer is not part of this
controller. The exact information state retains every physical hypothesis
consistent with the complete raw sensor history. The deployed finite-scenario
SQP transcription is explicitly an approximation to this robust problem.

A finite controller architecture does not itself remove the safety theorem.
Unverified uncertainty reduction, numerical flow error, and execution of an
unverified nonlinear step do. The theorem below assumes sound joint set
operations and whole-hold verification; finite-scenario acceptance alone
satisfies neither universal requirement.

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

<a id="joint-prediction"></a>
## 3. Jointly optimized finite controller memory

Choose $m\in[-1,1]^{n_m}$ and shared stage coefficients
$\alpha=(v,K,A,B,c)$ in a compact coefficient set. Define

$$
u=\kappa_\alpha(m)=v+Km,\qquad
m^+=M_\alpha(m,Y)=\tanh(Am+B\varphi(Y)+c). \tag{D1}
$$

The feature map $\varphi$ is fixed and smooth for each detection mode; missing
values are masked and the flags are features. The cube is invariant. Memory
need not approximate physical state and has no separately prescribed observer
gain or observer-convergence requirement.

At the boundary choose $\alpha$, apply $\kappa_\alpha(m)$ throughout the hold,
receive the complete block, and apply that same committed $M_\alpha$. Updating
memory through a different map invalidates the predicted continuation.
Coordinates, feature scales, and model contracts remain fixed when shifting.

Augment $s=(\chi,m,u^-)$. The joint transition $\mathscr G_\alpha$ applies
$\mathscr R_h$, (D1), and $(u^-)^+=\kappa_\alpha(m)$. At a real boundary,

$$
\mathcal I_k\subseteq\mathcal X\times\{m_k\}\times\{u_k^-\}.
$$

The exact information state is the consistency set for the initial contract,
all actual inputs, committed memory maps, and all raw observations. Its update
and unconditioned prediction are

$$
\begin{aligned}
\Psi_\alpha(\mathcal I,Y)
 &=\{s^+:\exists s\in\mathcal I,\gamma,
          (s^+,Y,\gamma)\in\mathscr G_\alpha(s)\},\\
\mathcal T_\alpha(R)
 &=\{s^+:\exists s\in R,Y,\gamma,
          (s^+,Y,\gamma)\in\mathscr G_\alpha(s)\}. \tag{D2}
\end{aligned}
$$

Condition on the entire block, even when $\varphi$ compresses it. Initial
containment and a valid Markov contract imply true-successor containment.
Future memory and physical state remain correlated within $R_i$. All root
hypotheses have the same memory and therefore the same immediate command;
future commands differ only through observations already received.

## 4. Whole-hold hard safety and the predictive problem

For footprint vertices $e_a,c_b$ in their body frames, including reference-point
offsets, define signed clearance and its complete-hold margin by

$$
d(\chi)=\max_{\|n\|=1}\left[\min_a n^\top e_a-
             \max_b n^\top(\rho+Qc_b)\right],
\qquad g(\chi)=d(\chi)-d_{\rm safe},
$$

$$
\underline g(R,\alpha)=\inf_{s\in R,
 (s^+,Y,\gamma)\in\mathscr G_\alpha(s),\;0\le t\le h}g(\gamma(t)).
$$

This is the rectangle vertex specialization of signed-distance separation;
positive clearance separates the bodies and negative clearance permits
penetration. Geometric witness normals may depend on physical configuration.
They are proof witnesses, whereas actuator decisions must satisfy (D1).

Let $\mathsf H(R,\alpha)$ include input bounds, actual command increments
relative to $u^-$, and all physical-domain constraints. Choose a fixed terminal
policy $\alpha_f$ and a joint set $Z_f$ such that

$$
\mathsf H(Z_f,\alpha_f),\quad
\underline g(Z_f,\alpha_f)\ge0,\quad
\mathcal T_{\alpha_f}(Z_f)\subseteq Z_f. \tag{D3}
$$

The hard-safety problem is

$$
\begin{aligned}
\min_{\alpha_{0:N-1},R_{0:N}}\quad&J_{\rm perf}\\
\text{subject to}\quad&R_0=\mathcal I_k,\\
&\mathcal T_{\alpha_i}(R_i)\subseteq R_{i+1},\\
&\mathsf H(R_i,\alpha_i),\quad\underline g(R_i,\alpha_i)\ge0,\\
&R_N\subseteq Z_f,\quad\alpha_i\in\mathcal A. \tag{D4}
\end{aligned}
$$

Performance may penalize route error, effort, input increments, or policy
changes. Global optimality is unnecessary for safety: every executed plan
must be feasible, and local optimization may propose improvements.

<a id="recursive-feasibility"></a>
### Recursive-feasibility theorem

Assume initial containment, an unchanged valid physical/sensor contract, sound
joint prediction and conditioning, whole-hold constraint checks, (D3), and an
initially feasible plan. Assume the admissible policy, set, and certificate
representations retain the shifted tail and terminal append. Each executed
replacement is verified feasible. Then (D4) is recursively feasible, the true
state remains in the information set, and $d(\chi(t))\ge d_{\rm safe}$ for all
covered holds.

**Proof.** The actual block and committed memory map give
$\mathcal I_{k+1}\subseteq R_{1|k}$. Set images under a fixed $\alpha$ are
inclusion-monotone. Use $(\alpha_{1|k},\ldots,\alpha_{N-1|k},\alpha_f)$ and
reuse the old future certificates, restricting them to the surviving branch
where necessary. Every inherited constraint still holds, including the next
command increment: its predecessor is the actual applied root command,
retained in the augmented state. The final inherited image is inside $Z_f$;
(D3) supplies one more safe hold and terminal image. True trajectories belong
to the certified hold sets. Induction proves the claim.

A sound fitted posterior is insufficient if it protrudes beyond the old
successor. Require

$$
\Psi_{\alpha_0}(\widehat{\mathcal I}_k,Y)
 \subseteq\widehat{\mathcal I}_{k+1}\subseteq R_{1|k}. \tag{D5}
$$

Even (D5) does not make an arbitrary fixed-capacity set representation closed
under shifting. Retain the old certificate or prove that the new representation
can encode its required successors without losing feasibility. Policies and
certificate classes, not just root sets, must admit the shift.

No theorem is asserted for every collision-free initial state: indistinguishable
states can need incompatible actions. Safety does not imply tracking convergence.

## 5. Constructive low-speed terminal component

Choose $u_f=(0,-b_0)$, $0<b_0<1$, with $K_f=0$ and $A_f=B_f=c_f=0$.
In the pure low-speed branch the physical equations reduce to

$$
\dot v_x=-B_\vartheta(v_x)+v_yr,\qquad
\dot v_y=-v_y/\tau_\ell,\qquad\dot r=-r/\tau_\ell,
$$

$$
B_\vartheta(v)=\frac{b_0(\mu_fF_{zf}+\mu_rF_{zr})\tanh(v/v_b)+F_{\rm load}(v)}{m_E}.
$$

Assume passive continuous road load, $B_\vartheta(0)=0$, positive friction,
well-posed flows, and no additional residual forces. For $|v|\le v_\ell$,
concavity of $\tanh$ gives

$$
\operatorname{sgn}(v)B_\vartheta(v)\ge\lambda|v|,\qquad
\lambda=\inf_\vartheta\frac{b_0(\mu_fF_{zf}+\mu_rF_{zr})}{m_E}
               \frac{\tanh(v_\ell/v_b)}{v_\ell}>0. \tag{D6}
$$

A persistent-parameter-dependent positive $\lambda(\vartheta)$ is also valid
when retained as part of the same joint state and checked for every member.
Define

$$
s_E=|v_x|+\frac{\tau_\ell}{2}|v_yr|,\qquad
D_E=s_E/\lambda+\tau_\ell|v_y|. \tag{D7}
$$

Since $D^+|v_yr|=-2|v_yr|/\tau_\ell$,
$D^+s_E\le-\lambda|v_x|$ and
$D^+D_E\le-|v_x|-|v_y|\le-\|v_E\|$. Thus $s_E\le v_\ell$ is invariant and
$\int_t^\infty\|\dot p_E\|\,d\tau\le D_E(t)$.

For a forward-only domain additionally require $v_x\ge0$, $v_yr\ge0$.
The product keeps its sign and at $v_x=0$ one has $\dot v_x=v_yr\ge0$.
The unrestricted signed-speed proof alone does not ensure membership in a
forward-only or limited-reverse-speed implementation domain.

For targets guaranteed to maintain $A_T\le-a_0<0$ until absorbing rest, use
$D_T=V_T^2/(2a_0)$. Under the stricter persistent constant-$A_T$ contract one
can sharpen this to $D_T=V_T^2/(-2A_T)$ separately for each moving hypothesis.
Then $\dot D_T=-V_T$. For absorbed targets $D_T=0$ independently of latent
$A_T$. A measured negative acceleration alone is not a future braking promise.

Let $a_E,a_T$ be the maximum vertex radii about the dynamics reference points.
The reserve

$$
h_f=\|\rho\|-a_E-a_T-d_{\rm safe}-D_E-D_T \tag{D8}
$$

is nondecreasing: $d\|\rho\|/dt\ge-\|v_E\|-V_T$, and the travel reserves
account for both terms. The integral argument also avoids differentiability
issues at zero separation; terminal membership with positive footprint radii
already excludes that case.

A concrete $Z_f$ imposes $s_E\le v_\ell$, $h_f\ge0$, the forward conditions,
target braking/absorption, $m\in[-1,1]^{n_m}$, $u^-=u_f$, and any additional
invariant physical limits. Bounds on $|v_y|$, $|r|$, and $s_E$ imply the respective
lateral, yaw, and longitudinal caps. Arbitrary sideslip-ratio limits are not
automatically invariant, since longitudinal and lateral speeds can decay at
different rates. Additional road constraints need their own invariant component.

The pose-preserving reset $(v_x,v_y,r)\mapsto(0,0,0)$ decreases $D_E$ to zero
and increases $h_f$, so it preserves this set if every preceding continuous
flow remains in the declared domain. A numerical integrator still needs a
sound integration error bound before it can certify that continuous flow.

For a smooth sufficient transcription, replace each $|z|$ in (D7) by
$\sqrt{z^2+\epsilon^2}$, or use epigraph variables $p\ge z,p\ge-z$. Both
upper-bound the absolute values and therefore strengthen the entry inequalities.
Their smoothed sublevel sets need not themselves be invariant: they certify
entry to the exact invariant set. Finite-scenario fallback plans must therefore
recheck the smoothed inequalities instead of assuming that exact invariance
preserves their representation.

Continuing/coasting targets, target restart, arbitrary persistent residual
forces, and general road geometry are outside this terminal component. For
example $\dot v_x=-\lambda v_x+\varepsilon$ with $\varepsilon>0$ has infinite
remaining travel. Such contracts need another certificate.

## 6. Finite joint transcription and SQP deployment

Choose correlated initial physical/parameter/bias hypotheses and fixed future
noise cases. Parameters remain persistent; scenarios share every stage
coefficient and the same root memory. For each scenario $s$,

$$
\begin{aligned}
u_i^s&=v_i+K_im_i^s,\\
(\chi_{i+1}^s,Y_{i+1}^s)&=\widetilde{\mathscr R}_h(\chi_i^s,u_i^s;\omega^s),\\
m_{i+1}^s&=\tanh(A_im_i^s+B_i\varphi(Y_{i+1}^s)+c_i).
\end{aligned} \tag{D9}
$$

All coefficients are optimized jointly. The deployment retains scenario shooting
states $(x_i,m_i,u_{i-1})$ and shared stage coefficients in a sparse QP.
Stage-local finite differences perturb only the six ego coordinates and two
inputs; memory and coefficient derivatives are analytic. No coefficient
perturbation repeats the whole-horizon nonlinear rollout. Fixing its last stage to $\alpha_f$ is an additional conservative
restriction that enforces terminal previous-input compatibility; this restricted
policy family remains closed under a tail shift and backup append.

At start, midpoint and endpoint, use a unit witness
$n=(\cos\theta,\sin\theta)$ and offset $b$ with

$$
n^\top e_a-b\ge d_{\rm safe}+\epsilon_{\rm hold},\quad
b-n^\top(\rho+Qc_b)\ge0.
$$

A sufficient intersample reserve is

$$
\epsilon_{\rm hold}=\frac h4(\bar v_E+\bar v_T+a_E\bar r_E+a_T\bar r_T),
$$

provided these are valid continuous speed/rate bounds throughout the hold.
Checking caps only at finite numerical nodes does not establish that premise.

For least-squares residual $r(w)$ and inequalities $c(w)\le0$, an SQP step uses

$$
\min_{\Delta w}\ (J_r^\top r)^\top\Delta w+
\tfrac12\Delta w^\top(J_r^\top J_r+\lambda I)\Delta w,
\quad c+Dc\Delta w\le0.
$$

The actual deployment adds the sparse shooting equalities
$\Delta X_{i+1}-A_i\Delta X_i-B_i\Delta\alpha_i=0$ at a dynamically consistent nominal
rollout, fixes the root increments to zero, and solves exactly one QP. The
shared root policy remains causal. Future shooting memories are tangent
predictions, not actual future measurements. The accepted first policy uses
the known actual memory, and its committed memory map processes the next raw
block. All received raw samples condition the finite hypotheses; future sensor
blocks are coarsened to two substeps by default.

Default deployment dimensions are four stages, one memory coordinate, and one
future-noise direction per retained joint hypothesis. The horizon does not grow
with speed. Separating planes are checked at start, midpoint and endpoint with
a 0.2 m engineering reserve in addition to the requested clearance. This margin
is not a verified intersample bound. The terminal condition uses
$D_E=v_x^2/(2b_0\mu g)+(|v_x|+\tau_\ell|v_y r|/2)/\lambda+\tau_\ell|v_y|$ and the persistent target braking travel
as an extra separation requirement on the terminal rectangle separating plane.
The deployment does not require terminal membership in the circular invariant set.
Here $\lambda=b_0\mu g\tanh(v_\ell/v_b)/v_\ell$; the linear travel term
retains a reserve for the fading low-speed brake. This combined approximation is
not the fading-brake invariant certificate of Section 5 and has no universal
guarantee, especially near low-speed transitions or combined tire saturation.

Numerical state and geometry inequalities allow half the configured constraint
tolerance; actuator magnitude and slew rows have no such allowance. This bounds
the numerical tolerance of the QP, not the nonlinear residual.

The trial coefficients are committed directly after a successful QP. There is
no nonlinear line search, post-step acceptance gate, or certified fallback.
A failed QP returns an explicit error without issuing a new command. This error
is not itself a safe continuation policy. Initialization is one QP from a
numerical seed; MATLAB/solver warmup can run before the timed control loop.
The configured deadline is measured and reported, not enforced by interrupting
the QP. The complete inference, nominal preparation, local linearization, QP,
and command bookkeeping contribute to the exported total call time.

Finite hypotheses can be depleted by valid measurements when the actual state
lies between samples. Finite future cases also omit admissible noise histories.
No scenario weights are probabilities and no deterministic or probabilistic
safety claim follows. Validated nonlinear enclosures, uniform uncertainty
coverage, nested conditioning, and retained certificates are required to
upgrade the deployment to (D4). Restricting the memory dimension alone does
not prevent that upgrade.

## 7. Sources and relation to the comparator

- Zhang, Liniger and Borrelli, *Optimization-Based Collision Avoidance*,
  [primary manuscript](https://arxiv.org/abs/1711.03449), for separating-plane
  and signed-distance optimization. The geometry above is a vertex specialization.
- Huang, Wang, Margellos and Goulart (2025), *Predictive Control Barrier Functions:
  Bridging model predictive control and control barrier functions*,
  [primary manuscript](https://arxiv.org/html/2502.08400v1), Section III, for the
  shifted predictive-deficit construction. Hard safety here does not require
  minimizing a soft deficit. If common stage deficits are admitted, the shift
  yields $V_N(\mathcal I^+)\le V_N(\mathcal I)-\xi_0^\star$ for an attained
  primary optimum. A positive deficit is recovery, not a safety certificate.
- Diehl, Bock and Schloeder (2005), *Real-Time Iterations for Nonlinear Optimal
  Feedback Control*, [primary paper](https://skoge.folk.ntnu.no/prost/proceedings/cdc-ecc05/pdffiles/papers/1186.pdf),
  for local one-step SQP approximations and their regularity/initialization
  assumptions. One QP is not a nonlinear feasibility proof.

The separately designed NRMM observer remains a research comparator whose own
conditional ISS theory is in [OBSERVER_ISS_THEORY.md](../estimator/OBSERVER_ISS_THEORY.md).
Its gain synthesis, force-inversion assumptions, and observer-error supports
are not prerequisites of (D1)--(D9).
