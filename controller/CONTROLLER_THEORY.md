# Controller mathematical theory

This document contains the mathematical model, assumptions, nonlinear
output-feedback predictive barrier problem, and its recursive-feasibility
proof. The theoretical controller executes the first command of an attained
primary slack minimizer. Its terminal family and compatible causal terminal
controller are assumed to exist.

The sensor contract consists of position-only GNSS, IMU acceleration, gyro,
and relative radar position. Future physical motion, observer updates,
measurements and causal feedback share the same uncertainty realizations.
Safety is stated at the declared checked instants: zero clearance excludes
interior overlap but may permit contact. Continuous-time safety requires the
additional whole-hold enclosure stated below.

## Contents

1. [Vehicle model and target prediction](#vehicle-model)
2. [Observer bounds and relative geometry](#observer-geometry)
3. [Collision constraints and the switched problem](#optimization)
4. [One nominal CLF and recovery](#nominal-clf)
5. [Abstract terminal family](#terminal-set)
6. [Joint feedback-aware prediction](#joint-prediction)
7. [Unified observer-to-safety formulation](#recursive-feasibility), including
   [compact sampled certificates](#compact-certificate)
8. [Sources](#sources)

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

Let $R_{\rm sense}>0$ denote the sensing radius. Detection is guaranteed
inside that radius under the stated sensor contract. An observed or robustly
established exit ends the encounter; nominal predicted exit alone does not
remove obligations for branches that remain in range. Reentry starts a new
encounter and requires fresh initial feasibility. The precise information
update is given in (U2).

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
A continuous comparison alone does not establish a sampled comparison with
held measurements or missing readings. Append independently valid yaw, gyro
and absolute-position bounds. An energy-gain statement alone is not a
pointwise yaw enclosure, and current-state containment is not future
propagation. Deterministic amplitude bounds require a bounded-noise contract.

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

### One nonlinear geometry and its continuous witnesses

Let $v_{E,a}$ and $v_{C,b}$, $a,b=1,\ldots,4$, be body vertices measured from
the respective dynamics reference points, including rectangle offsets. Write
$e_a=p_E+R(\psi_E)v_{E,a}$ and $c_b=p_C+R(\psi_C)v_{C,b}$. Define one signed
clearance function, independent of the planner or numerical reference:

$$
 d_s(E,C)=\max_{\|n\|_2=1}\{\min_a n^Te_a-\max_b n^Tc_b\}.       \tag{G1}
$$

For disjoint convex rectangles this is Euclidean separation; for overlapping
interiors it is the negative minimum translational penetration depth. This is
the signed-distance support/dual construction of Zhang, Liniger and Borrelli,
Theorem 2, specialized to rectangle vertices. Ordinary unsigned distance
would be zero even for overlap and cannot represent the desired soft deficit.

Introduce $n(\theta)=(\cos\theta,\sin\theta)$ and a plane offset $b$ measured
relative to $p_C$. The nonlinear collision inequalities are

$$
\begin{aligned}
 n(\theta)^T[p_E-p_C+R(\psi_E)v_{E,a}]-b+\xi&\ge0,&&a=1,\ldots,4,\\
 b-n(\theta)^TR(\psi_C)v_{C,b'}&\ge0,&&b'=1,\ldots,4,\\
 \xi&\ge0. &&
\end{aligned}                                                    \tag{G2}
$$

Eliminating $b$ and $\theta$ gives exactly $d_s(E,C)\ge-\xi$. At zero slack,
the first four rows put every ego vertex on one side and the other four put
every target vertex on the other. With positive slack, the maximum permitted
penetration along the chosen unit direction is explicit. A normal constrained
only by $\|n\|\le1$ would admit $n=0,b=0,\xi=0$ at overlap: the equality
normalization is essential here. The angle is a periodic coordinate on the
unit circle, not a discrete maneuver label. The periodic angle parametrization covers every unit direction.

For uncertain joint state $q\in R_i$, require (G2) for every member with a
common witness $(\theta_i,b_i)$. This common-plane robust condition is
sufficient, and can be more conservative than requiring physical nonoverlap
with a different witness for every unknown member. That quantifier order is
part of the design, not hidden inside a distance calculation. Shrinking $R_i$
retains every previously valid witness. Relative position must be formed
before projecting shared uncertainty, so common ego/target errors cancel
when the joint model actually permits their cancellation.

Changing a separating witness changes an auxiliary decision variable, not
(G1) or the predictive barrier value. The robust common-plane quantifier
and the policy class remain fixed throughout the encounter.

### Constraint instants and a continuous-time extension

Suppose true states at the start, midpoint and end of each hold are enclosed.
For true whole-hold speed/rate bounds, body reach radii $R_E,R_C$ give

$$
L_d=\bar v_E+\bar v_C+R_E\bar r_E+R_C\bar r_C,\qquad
\epsilon^{\rm hold}=L_dh/4.
$$

**Derivation.** Every body moves in Hausdorff distance by at most translation
plus rotational reach times elapsed time. Every instant is within $h/4$ of
start/midpoint/end. Tightening true nonoverlap certificates at those instants
by the motion reserve therefore suffices throughout the hold. The certificates must enclose the true states at those three instants.
The ideal theorem below guarantees only the declared constraint instants
unless such whole-hold images are supplied.

<a id="nominal-clf"></a>
## 4. One nominal CLF and recovery

### Function and its scope

For the given path and desired cruise trim, define

    e(x) = [e_y; e_psi; vx-vx_ref; vy-vy_ref; r-r_ref],
    V(x) = e(x)' P e(x),       P > 0.

Longitudinal path phase is free. A trim $(x_\star,u_\star)$ and a positive
definite matrix $P$ specify a local cruise function. The terminal assumption
does not require this particular CLF.

### Requested decrease: a convergence time constant

The performance design uses the continuous-time nonlinear bicycle system:

    Vdot(x,u) <= -alpha V(x) + s,   alpha = 2/T,   s >= 0,

where $T>0$ is a symbolic convergence time constant. The nonnegative slack
has units of CLF value per second. Exact continuous enforcement with zero
slack implies $V(t)\le e^{-2t/T}V(0)$ on the domain where the inequality
holds. This is not a global CLF claim.

For a smooth trim error system with linearization $(A_c,B_c)$, suppose a gain
$K$ and $P\succ0$ satisfy a strict continuous Lyapunov inequality
\[
 (A_c+B_cK)^TP+P(A_c+B_cK)\prec-\alpha P.
\]
The nonlinear remainder is higher order at the trim. Strictness therefore
implies the same requested decay on a sufficiently small neighborhood in
which $u=u_\star+Ke$ is admissible. The size and invariance of that domain
must be established separately. The CLF describes target-free cruise and
is not required by the encounter PCBF theorem.

### Disturbance-dependent recovery: a sufficient theoretical construction

For the same $V$, suppose a common recovery action on a verified domain satisfies

$$
\sqrt{V(F(x,\kappa(x)))}\le a\sqrt{V(x)},\quad 0\le a<1,
$$

and estimate-feedback error and admitted plant disturbances
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
gives convergence on the verified invariant domain.

<a id="terminal-set"></a>
## 5. Abstract terminal family

The theory assumes a nonempty, downward-closed family of joint information
sets $\mathfrak B$ and an admissible causal terminal controller. From each
member the controller must supply zero collision deficit, satisfy the same
hard constraints as the prediction problem, and propagate into another member
of $\mathfrak B$. The exact conditions and quantifiers are stated in (U4).

This is an existence assumption, not a prescribed braking law or terminal
geometry. Membership represents an available safe continuation; it is not a
normal-driving objective. A statewise safe action chosen with knowledge of an
unknown true state does not suffice: the terminal rule must use the same
observable information and memory as the predicted policies.

<a id="joint-prediction"></a>
## 6. Joint feedback-aware prediction

This supplies the causal observer/plant prediction used in Section 7.
Its held-input specialization recovers the physical information-set case. Future measurements are unknown bounded variables.
Tube widths may vary, expand temporarily and retain a noise floor; a
constant-width invariant tube is optional. The model, observer, sensor and
terminal contracts must cover the same uncertainty family. Equations
(J1)--(J11) give the exact nonlinear construction, including target observation
and target-dependent ego control, before any supplementary representation.

### Exact nonlinear joint construction

The core theory uses exact nonlinear flows, a fixed decision horizon and
symbolic sensor/model bounds. Let sensor times be t_j=j*h_s and control
times t_(kL), with h=L*h_s and L a positive integer. The analysis has no
extra residual disturbance (d=0); uncertain tire stiffness and friction
remain persistent physical parameters. Additional disturbances require the
same extension in the vehicle, sensor, observer and terminal contracts.

Retain the augmented state

$$
 z=(x_E,x_T^{\rm phys},\vartheta_E,\zeta_E,\hat X_T,\hat y,m,u^-,\mathcal H).
 \tag{J1}
$$

Here x_T^phys includes target pose, speed, constant acceleration/curvature
parameters and stopping mode. The ego estimator state zeta_E contains its
position/velocity filters, heading and measurement memory. The target
estimate is Xhat_T=(rhohat,qhat,shat); yhat is its radar predictor; m contains
held sensor values; H contains the absolute observable history, clock and
policy memory. Physical parameters and their current states remain linked
by their physical evolution. They are not independent uncertainties.

A decision state is posterior to the measurements at that boundary. Its
current estimator states, measurement record and observable history are
known; physical quantities remain uncertain. A decision rule u=pi(H) is
constant on all tuples with the same H. At later decisions, different
measurement histories can generate different controls. This is an
optimization over causal rules, not just one future input sequence.

**Well-posedness premise.** Every admitted vehicle/observer flow and reset
must exist uniquely over every sensor interval, including terminal
continuations. A sufficient moving-vehicle domain has v_x bounded away
from zero, tire slip angles bounded away from their tangent singularities,
positive compact tire parameters, bounded velocities/yaw rate and locally
Lipschitz resistance. Matched Fiala saturation pieces are retained. A
low-speed/stopped extension, if used, must have a declared transition and
sensor law. No invalid branch may be dropped to obtain a smaller set.
No observation-error contraction or measured GNSS velocity is presumed.

### Sensor-period flow and causal target correction

The body-frame physical target coordinates are

$$
 \rho=R_E^\top(p_T-p_E),\quad q=R_E^\top\dot p_T,\quad
 s=R_E^\top\ddot p_T.
$$

For forward target motion, set t=R_E^T e(chi_T). Then
q=V*t, s=A*t+kappa*V^2*J*t and
j_phys=-kappa^2*V^3*t+3*A*kappa*V*J*t. Consequently,

$$
 \dot\rho=q-v_E-r_EJ\rho,\quad
 \dot q=s-r_EJq,\quad \dot s=j_{\rm phys}-r_EJs.               \tag{J2}
$$

The target course is chi_T=psi_T+beta_T, with constant
kappa=sin(beta_T)/l_rT. At a deceleration-to-rest event the physical pose
and course freeze, q^+=s^+=0, and the latent constant parameters remain
stored. Target yaw is retained at rest rather than reconstructed from
zero velocity.

Let a_1,a_2,a_3 be the target observer's effective innovation gains; for
the high-gain notation in the estimator theory, a_i=l_i*omega^i.
On a sensor interval the held gyro sample is omega_j=y_(r,j). The exact
observer flow and radar-predictor reset are

$$
\begin{aligned}
 \dot{\hat\rho}&=\hat q-\hat v_E-\omega_jJ\hat\rho
                       +a_1(\hat y-\hat\rho),\\
 \dot{\hat q}&=\hat s-\omega_jJ\hat q+a_2(\hat y-\hat\rho),\\
 \dot{\hat s}&=\Phi_e(\hat q,\hat s)-\omega_jJ\hat s
                       +a_3(\hat y-\hat\rho),\\
 \dot{\hat y}&=\hat q-\hat v_E-\omega_jJ\hat y,\qquad
 \hat y(t_j^+)=y_{\rho,j}.                                    \tag{J3}
\end{aligned}
$$

The estimated target state is continuous at a radar reset. The ego observer
has a deterministic sensor reset P_E, intersample flow f_O and outputs
H_E(zeta_E)=(phat_E,psihat_E,vhat_E), using position-only GNSS, IMU,
gyro and known inputs. These maps retain their internal memory and use
the same held sensor record as (J3).

One control transition is defined chronologically:

1. At the current posterior, select the common input u=pi(H). The IMU
   sample at this boundary was generated with the previous held input.
2. Flow both physical vehicles and both observers for h_s, holding u and
   the current sensor record. Include any declared target stop event.
3. Generate the endpoint measurements from these physical states and one
   shared noise vector. Apply the ego reset, reset yhat on detection, and
   append the complete readings and detection flag to H. Set u^-=u.
4. Repeat steps 2--3 L times without changing u. Stop just after the last
   reset, before selecting the next command.

With F^u the joint sensor-period flow and P^u_(nu,d) the endpoint
measurement/reset map, this defines

$$
 \mathcal G_h(z,\nu;\pi)=
 (P^u_{\nu_L,d_L}\circ F^u)\circ\cdots\circ
 (P^u_{\nu_1,d_1}\circ F^u)(z),\qquad u=\pi(\mathcal H).
 \tag{J4}
$$

The noise block contains fresh measurement primitives only. Physical
parameters are propagated unchanged, and each boundary sample is used once.
Detection is compulsory inside the symbolic sensing radius. With that
contract a missed detection certifies exit at its sample; otherwise exit
requires all states consistent with the history to be outside. The same
observable exit rule is retained under shifting. Ended histories have
vacuous collision obligations; reentry requires new initial feasibility.

### Exact nonlinear estimation and relative-tracking identities

Use estimate-minus-truth errors e_rho,e_q,e_s, e_y=yhat-rho and
e_v=vhat_E-v_E. Let w=omega_j-r_E(t) and lambda=e_y-e_rho.
Subtracting (J2) from (J3) gives

$$
\begin{aligned}
 \dot e_\rho&=e_q-e_v-\omega_jJe_\rho-wJ\rho+a_1\lambda,\\
 \dot e_q&=e_s-\omega_jJe_q-wJq+a_2\lambda,\\
 \dot e_s&=\Phi_e(\hat q,\hat s)-j_{\rm phys}
                       -\omega_jJe_s-wJs+a_3\lambda,\\
 \dot e_y&=e_q-e_v-\omega_jJe_y-wJ\rho.                         \tag{J5}
\end{aligned}
$$

The same gyro and ego-velocity errors enter the first and last rows.
Their cancellation implies, for 0<=tau<h_s following a detection,

$$
 \dot\lambda=-(a_1I+\omega_jJ)\lambda,\qquad
 \lambda(t_j+\tau)=e^{-a_1\tau}R(-\omega_j\tau)
                     [n_{\rho,j}-e_\rho(t_j)].                \tag{J6}
$$

This is the decay of the predictor innovation within a hold, not a claim
that the entire target estimation error contracts. The held-gyro difference
also contains true yaw-rate change:
w=n_(r,j)+r_E(t_j)-r_E(t). At target stopping,
e_s^+=e_s^-+s^- while e_rho and e_q remain continuous.

On a compact declared domain, the existing Lipschitz extension gives

$$
 \|\Phi_e(\hat q,\hat s)-j_{\rm phys}\|
 \le L_q\|e_q\|+L_s\|e_s\|+
                 \|\Phi_e(q,s)-j_{\rm phys}\|.                \tag{J7}
$$

The last term is zero only where the extension agrees with the physical
law. It and the stopping jump are part of the joint propagation.
For the ego output error e_E=H_E(zeta_E)-W(x_E), W=(p_E,psi_E,v_E),
the exact sensor reset is H_E(P_E(zeta_E^-,y,u^-))-W(x_E).
Its flow increment is the difference of the two exact output increments;
where differentiable, dot(e_E)=D H_E*f_O-D W*f_E. Thus e_v in (J5)
is generated by the same uncertain physical/measurement branch.

For a dynamically consistent comparison branch, write Delta for the
difference of corresponding quantities and alpha=psi_T-psi_E. Then

$$
\begin{aligned}
 \Delta\dot x_E&=f_E(x_E,u;\vartheta_E)
                    -f_E(\bar x_E,\bar u;\bar\vartheta_E),\\
 \Delta\dot\rho&=\Delta q-\Delta v_E-r_EJ\Delta\rho
                                      -\Delta r_EJ\bar\rho,\\
 \Delta\dot\alpha&=\Delta(\kappa_TV_T)-\Delta r_E.              \tag{J8}
\end{aligned}
$$

For a causal law u=K_i(c), c consists of available target/ego estimates and
memory. Its input difference is K_i(c)-K_i(cbar), with
Delta(rhohat)=Delta(rho)+Delta(e_rho), and similarly for q,s,v_E.
Equation (J5) therefore connects future target observations to (J8) through
the actual nonlinear force law. The first input difference is zero when
histories are indistinguishable.

### Joint reachable sets, posterior fibres and feedback benefit

For an admissible fixed rule define the exact successor

$$
 T_\pi(S)=\{\mathcal G_h(z,\nu;\pi):z\in S,\ \nu\in{\rm Adm}(z)\},
 \quad \mathcal R_0=I_k,\quad
 \mathcal R_{i+1}=T_{\pi_i}(\mathcal R_i).                      \tag{J9}
$$

Adm includes the observation modes and any noise-continuation memory.
A rule is admissible only if every branch has a well-defined flow and
satisfies its hard constraints. Sets retain history tags. A fibre
R_i^eta contains the tuples with one particular observed history eta;
the union over histories is the unconditional prediction. Each fibre
receives one common command.

For any defined relative-error projection D_i, its exact error set is
D_i(R_i). Component bounds can be defined by

$$
 B_{i+1,a}=\sup_{z\in R_i,\nu\in{\rm Adm}(z)}
       \|D_{i+1,a}(\mathcal G_h(z,\nu;\pi_i))\|.               \tag{J10}
$$

On a declared bounded finite-horizon domain these bounds are finite.
This is a sound theoretical construction, not a general theorem that a
small B exists. On a fibre with reading y_rho, true relative position has
diameter at most 2*nbar_rho, but different fibres can have different centers.
Small conditional estimation error does not shrink physical target
dispersion about one frozen forecast.

A checkable mathematical feedback criterion is obtained by grouping states
with the same observable statistic c into S_c. Let L_c(u) be the supremum of
the next relative-position error under a held u, taking the exact joint image,
and exclude commands violating any hard condition. Define

$$
 U_c(B)=\{u\in\mathcal U:\text{hard conditions on }S_c,\
                         L_c(u)\le B\}.                      \tag{J11}
$$

An admissible selector K(c) in U_c(B) guarantees that one-step relative
bound. If every fibre admits such a selector but the intersection of
all U_c(B) is empty, observation-dependent feedback achieves a bound that
no common command achieves. Nonemptiness and any required regularity of
the selector are substantive conditions, not consequences of bounded
sensor noise. The finite-horizon successor/terminal constraints enforce
their compatibility across later decisions.

A symbolic strict-improvement example stays within the Fiala model.
In straight motion with zero resistance and steering, the exact acceleration
is gamma*b, with |gamma/gamma_0-1|<=epsilon<1. Two target hypotheses start
with the same speed v and headway, zero curvature, and A=+a or -a.
Set h=2*h_s and apply b=0 during the first hold. Initialize both hypotheses
with the same exact relative position and velocity, shat=0 and yhat=rho,
and an exact ego estimate. Use zero sensor noise and choose the moving domain
so that the observer extension agrees with the straight-motion law.
With tau=h_s, the intermediate radar reset gives

$$
 \hat s_x(h)=\sigma\,\frac{a_3a\tau^2}{2a_1}
                         (1-e^{-a_1\tau}),\qquad \sigma\in\{-1,1\}.
$$

The causal next command b=sign(shat_x(h))*4*a/gamma_0 is admissible if
4*a<gamma_0 and the trajectories remain in the moving/sensing domain.
At 2*h, the target deviation from constant-speed motion is sigma*D,
D=2*a*h^2, while the ego correction is sigma*(gamma/gamma_0)*D.
The relative-error bound is epsilon*D. Any common ego endpoint has
worst-case error at least D against the two target endpoints. Small
nonzero noise and parameter perturbations preserve the strict inequality
by continuity while the observer estimate keeps its sign margin.
This proves existence of a strict information benefit, not a uniform
small-tube claim for arbitrary encounters.

The complete posterior construction and shift proof follow in Section 7.
The representations below are supplementary comparisons and enclosure
methods; they are not additional premises of the exact construction
(J1)--(J11).

### Supplementary feedback-rule representations

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

A comparison construction can use time-varying certified coefficients,

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
feedback policy's input variation must therefore be part of G_i. Unknown future
measurements remain bounded variables, not known or noise-free values.

Finite outer representations must enclose the finite-horizon images, admit
the policy-generated inputs and constraints, and meet the terminal-family
condition. Recursive feasibility uses cross-sample inclusion and terminal
closure; it does not require a constant prediction-tube width.

### One uncertainty source and its different consequences

Let theta be a fixed unknown vehicle-parameter vector in a declared set Theta,
including uncertain tire stiffness and friction. Other physical parameters
are uncertain only if included in that set. Let d_i be any
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
If all model uncertainty is represented by the declared parameter family,
the residual disturbance may be zero. For an added disturbance, certificate
validity must first be extended.

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

For switching or nonsmooth model branches, partition their guards and retain
all admissible images. Fiala saturation, the low-speed blend and any declared
stop reset cannot be covered by assuming a single globally differentiable
map. All images here refer to the declared exact physical and observer flows.

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
Pointwise Schur stability alone establishes neither a uniform nonlinear
transition-product bound nor validated U_i over the admitted parameter set.

### Quantify the nonlinear and mode-dependent extension

At admissible nominal straight cruise above the blend/stop regime, sampled Fiala F
has continuous first derivatives locally. Stable A_c admits a contracting
metric. Continuity gives a sufficiently small neighborhood with contractive
eta derivative in (1) and bounded epsilon derivative. The mean-value argument
then gives (3) for compatible sufficiently small observer errors. This proves
local existence. A finite-domain result additionally requires explicit bounds
on that domain, the nonlinear remainder, the input image and all checked hold
fractions.

Moving nominal trajectories need scheduled gains/metrics, for example
A_ci'*P_(i+1)*A_ci<=rho^2*P_i plus nonlinear reserves. The five-state
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

### Inertial-velocity correlation

A shared yaw error changes the reconstructed body velocity to first order
by [v_y; -v_x] times that error. In the inertial velocity
R(psi) v, this first-order contribution cancels the corresponding rotation
term. A large body lateral-velocity coordinate radius can therefore mostly
represent uncertain coordinates, rather than equally large uncertain
inertial lateral motion. Dropping this column correlation charges both
terms separately. During deceleration the future velocity dependence must
be recomputed by the joint map; copying a high-speed bound to a
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

The theoretical object is a nonlinear output-feedback predictive barrier.
It uses the joint future observer/plant evolution from the beginning; it does
not first replace future feedback by open-loop error propagation. Exact-state
MPC is the singleton-information special case. This section specializes the
shift construction in Huang et al., problem (8) and Lemma III.5, while making
its additional observer, moving-target and terminal assumptions explicit.

### Joint information and one copy of uncertainty

Let $q$ be the history-tagged augmented state (J1): true ego and target
states, observer internal states, held sensor records, persistent uncertain
parameters, previous input, absolute observable history and policy memory. A future command has the causal form
$u_i=\pi_i(o_i)$, where $o_i$ consists only of information available to the
controller at that future instant. In particular, it cannot depend on the
unknown true error or future measurement noise. Affine feedback about a
nominal plan is one possible policy class, but the class must allow the shift
and terminal append used below.

Use the exact sensor-period flow/reset composition (J4), including all
intermediate readings and observer updates. Write its complete control-period
transition as

$$
 q^+=\mathcal F(q,\pi(o),\omega),\quad \omega\in\Omega,
 \qquad \vartheta^+=\vartheta.                                 \tag{U1}
$$

The same $\vartheta$ influences the plant and observer residual. A measurement
primitive appears once and is shared between every update using that sensor.
The initial estimation error belongs to the initial information set; it is
not reinjected independently at every future step. Future physical mismatch
still acts through (U1). These distinctions prevent double counting without
pretending that mismatch disappears after estimation.

Let $\mathcal T(R,\pi)$ be the exact image under (U1), and let
$\mathcal T_\tau(R,\pi)$ include a declared checked instant within a hold.
These maps are monotone in $R$ for a fixed policy. The theory uses sound outer
images $\mathcal T(R_i,\pi_i)\subseteq R_{i+1}$, with stored proofs or exact
sets. It neither assumes constant tube width nor requires contraction at every
step.

### Prediction and measurement conditioning must agree

Let $Q_{k+1}$ be the sound current observer enclosure, including the known
observer internal state. Retain the previous first successor and form

$$
 I_{k+1}=R_{1|k}\cap Q_{k+1}\cap A_{k+1},\qquad
 q_{k+1}\in I_{k+1}\subseteq R_{1|k},                            \tag{U2}
$$

where $A_{k+1}$ selects the **entire received sensor block**: intermediate
readings, detection flags, resulting known observer states and input history.
Induction through the physical flows and resets of (J4) keeps the true branch
in $R_{1|k}$; that branch also satisfies these observed facts and any valid
additional enclosure $Q_{k+1}$. This establishes both inclusions in (U2).
At initialization $I_0=Q_0$. A raw new observer ball need not be contained in the
previous one; the intersection with the **predicted successor**, not a
recentered ball, gives the inclusion needed for shifting. Any outer numerical
representation of the posterior must still lie inside that successor.
A missing reading imposes no numerical measurement restriction, but its
detection flag still conveys any information specified by the sensor contract.
In particular, guaranteed in-range detection makes a miss an exit certificate
at that sample. An empty intersection violates the declared model/information contract and cannot be
silently repaired by discarding the old information.

A new target, a changed uncertainty contract, or a genuinely new encounter
requires a new initial-feasibility argument. During an encounter the target's
motion parameters are propagated and conditioned, not independently reset.
An actual, or robustly established, perception exit ends this encounter;
nominal predicted exit alone cannot remove constraints for uncertain members
that remain nearby. Subsequent reentry is a new encounter. None of the
following assertions covers arbitrary unmodeled later encounters.

### One downward-closed constraint and terminal family

Let $H(R_i,\pi_i)$ express the hard input, input-memory and model-domain
conditions for all members at the declared checked instants. No road boundary
is included in the present theoretical task. Define $C(R_i,\pi_i;w_i)$ as the
least nonnegative slack for (G2) at those instants, with witnesses $w_i$ and
the target inactive only under the encounter contract. Endpoint clearance
uses $C_N(R_N;w_N)$. Retaining a witness gives

$$
 R'\subseteq R\ \Longrightarrow\
 C(R',\pi;w)\le C(R,\pi;w),\quad
 H(R,\pi)\Longrightarrow H(R',\pi).                            \tag{U3}
$$

**Abstract terminal-pair assumption.** There exists a nonempty,
downward-closed terminal family $\mathfrak B$ of nonempty joint information
sets, with an admissible causal terminal controller. For every
$R_f\in\mathfrak B$, this controller supplies a **common causal** policy
$\pi_f$, zero-deficit witnesses and a successor $R_f^+\in\mathfrak B$
satisfying

$$
 H(R_f,\pi_f),\quad C(R_f,\pi_f;w_f)=0,\quad
 \mathcal T(R_f,\pi_f)\subseteq R_f^+,\quad C_N(R_f^+;w_f^+)=0.  \tag{U4}
$$

The symbols $\mathcal T$, $H$ and $C$ in (U4) are exactly those used in the
prediction problem. Thus the assumption includes the same uncertainty,
geometry and admissibility requirements, including common separating
witnesses when (G2) is used. The policy cannot select an input from an
unknown true state; its information and memory follow the same causal rules
as the predicted policies. Restricting a certified set retains the policy
and witnesses, and terminal appending retains the required memory.

Existence with these properties is a premise of the core theorem, not a
construction that the theorem promises to deliver. No particular braking
model, stopping distance, terminal shape or terminal-mode switch is required
by the proof.

### The theoretical nonlinear PCBF problem and control law

Fix a finite $N$ at encounter initialization and use the same stage timing on
successive shifts. The baseline checks collision at decision instants. A plan consists of causal policies, nonempty joint sets,
continuous geometric witnesses and nonnegative slacks. Its conditions are

$$
\begin{aligned}
 R_0&=I_k,& \mathcal T(R_i,\pi_i)&\subseteq R_{i+1},\\
 H(R_i,\pi_i)&,& C(R_i,\pi_i;w_i)&\le\xi_i,
             && i=0,\ldots,N-1,\\
 C_N(R_N;w_N)&\le\xi_N,& R_N&\in\mathfrak B,
             &\xi_i&\ge0.                                    \tag{U5}
\end{aligned}
$$

Under (U4) the endpoint slack is redundant and can be chosen zero.
Decision instants, including the endpoint, define the present baseline.
Additional interior checks must use a shift-invariant schedule; whole-hold
versions require all $\tau\in[0,h]$. Neither extension is presumed here.
The paper permits a soft prefix of length $N_\xi\le N$ followed by a
zero-slack hard tail. Its shift also removes the first slack and appends zero,
so the same proof applies; $N_\xi=N$ gives (U5)--(U6).

Define $J_P=\sum_{i=0}^N\xi_i$. The theoretical encounter problem is

$$
 V_P(I_k)=\min_{\Pi,R,w,\xi} J_P,\qquad\text{subject to (U5)}.  \tag{U6}
$$

Choose any attained minimizer $P_k^*=(\Pi_k^*,R_k^*,w_k^*,\xi_k^*)$
and issue its first causal command $\pi_{0|k}^*$. No performance objective,
CLF slack, uniqueness assumption or second optimization is required by this
control law or its safety theorem. The result below holds for every attained
primary minimizer. Newly acquired targets require fresh initial feasibility.

Assume each encountered feasible problem attains its primary minimum.
A compact nonempty feasible sublevel, closed feasibility and
lower-semicontinuous cost suffice; merely allowing arbitrary history-dependent
policies does not establish these properties. The admissible policy class
must permit restriction to surviving absolute histories and terminal
concatenation. Retain all observer, reference and prefix memory needed to
evaluate the old tail. If coordinates are recentered, transport the policies
and witnesses by an invertible relabeling rather than resetting their meaning.

The optimal primary value $V_P(I)$ eliminates all policy, set and plane
variables. It is therefore independent of any particular plane orientation
or initialization path. Changing a plane witness while solving (U6) does not
redefine this predictive barrier function.

### Theorem U: recursive feasibility and zero-deficit invariance

Assume initial feasibility, true-state containment, sound joint images,
conditioning (U2), unchanged encounter/model contracts, existence of the
abstract terminal pair (U4), and a policy/set/certificate representation
closed under shifting.
The controller issues the first command of any attained minimizer of (U6).
There is no assumption on a secondary optimization objective. Then

$$
 V_P(I_{k+1})\le V_P(I_k)-\xi_{0|k}^*\le V_P(I_k).              \tag{U7}
$$

The next mathematical problem is feasible. If $V_P(I_{k_0})=0$, all subsequent
values and checked-instant collision deficits stay zero for this encounter.
Consequently vehicle interiors do not overlap at those instants for any
allowed joint realization. This does not assert a positive physical margin.

**Proof.** Whole-block conditioning (U2) gives
$S_0=I_{k+1}\subseteq R_{1|k}^*$. Restrict the old remaining policies to
the surviving histories; this retains one command on every indistinguishable
history fibre. Exact images are inclusion-monotone for a fixed policy, hence
induction gives
\[
 S_j\subseteq R_{j+1|k}^*,\qquad 0\le j<N.
\]
For outer-set representations, the stored old sets may instead be retained
where they give the required certified inclusions. By (U3), the old hard
constraints, plane witnesses and slacks remain valid. Downward terminal
closure gives $S_{N-1}\in\mathfrak B$. Append its admissible causal terminal
rule and a successor $S_N\in\mathfrak B$ with zero-deficit witnesses.

The shifted sequence $(\xi_1^*,\ldots,\xi_N^*,0)$ is feasible and has sum
$V_P(I_k)-\xi_{0|k}^*$. Comparing the next attained minimum with this candidate
gives (U7). The true current tuple is retained in the conditioned root, so
zero slack separates its footprint interiors at the checked instants.
Nonnegativity and induction preserve zero. For $N=1$, the terminal
restriction applies directly to $S_0\subseteq R_{1|k}^*$. QED.

The sum, not each individual predicted slack, is nonincreasing. First-stage
slack can redistribute while the sum falls. Telescoping (U7) makes the
executed first-stage deficits summable on an infinite applicable sequence;
it does not by itself prove finite-time recovery or Huang's stronger invariant
set convergence conclusions without their additional compactness/regularity
hypotheses. Positive slack is an admitted collision deficit, not safety.
Hard input/domain/terminal inconsistency can still make initialization
infeasible even when collision slack is unrestricted.

### Feasible improvement can replace global optimization

The shift argument also gives a useful stronger algorithmic statement that
does not demand a globally solved nonlinear program on every sample. Store
an actually feasible plan of cost $J_k$ and its shift. Accept any **nonlinearly
certified feasible** replacement with

$$
 J_{k+1}\le J_k-\xi_{0|k}.                                     \tag{U8}
$$

The retained shift always meets this bound, so feasibility and zero-deficit
invariance follow even if an improvement search fails or is stopped early.
This proves a statement about the stored plan cost, not equality to the
optimal value $V_P$. Under exact optimization (U8) is redundant. It is a
sufficient suboptimal control law under the same nonlinear feasibility
and shift assumptions.

<a id="compact-certificate"></a>
### Compact nonlinear certificates: replace history rollout by error comparisons

The exact augmented construction (U1) remains the semantic model. The
following reduction changes its **outer representation**, not the physical
vehicle, the observer, the causal information available to control, or the
meaning of collision slack. It does not introduce a linear tire model, a
friction polygon, a fixed-width tube, or a new terminal-controller design.

There are two different errors to retain:

1. Estimation error: an observer output minus the physical quantity on the
   **same** realization. Future sensing can reduce this error.
2. Physical prediction spread: variation of physical trajectories across
   different admissible realizations. Future sensing does not retrospectively
   reveal which target trajectory occurs, but future ego control can respond
   differently to the resulting estimates.

Replacing target prediction spread by a small future estimation radius is
invalid. Instead the physical relative tube and the estimation-error
comparison must be coupled through the causal policy.

**Relative physical coordinates.** Use
\[
 \rho=R(\psi_E)^T(p_T-p_E),\qquad \alpha=\psi_T-\psi_E,
 \quad v_T^E=R(\psi_E)^T\dot p_T.
\]
Common rigid translation and rotation cancel from this relative pose. For a
nominal comparison branch, (J8) implies exactly
\[
 \Delta\dot\rho=\Delta v_T^E-\Delta v_E
       -\omega_EJ\Delta\rho-\Delta\omega_EJ\bar\rho,
 \quad
 D^+\|\Delta\rho\|\le\|\Delta v_T^E-\Delta v_E\|
                      +|\Delta\omega_E|\|\bar\rho\|.         \tag{C1}
\]
The term \(\Delta\rho^TJ\Delta\rho\) vanishes. Adding
\(|\omega_E|\|\Delta\rho\|\) would create artificial norm growth from
a pure rotation. Likewise, combine the shared terms of
\(\Delta v_T^E-\Delta v_E\) before taking support bounds. Absolute yaw
and position errors still matter to world-referenced policies; their removal
from collision geometry does not remove them from those policies.

Let \(z\) contain sufficient physical coordinates to evaluate the relative
flow, admissibility, and chosen policy outputs. Relative pose alone is not a
closed vehicle state: body velocities, yaw rates and target motion variables
are also required. Keep persistent tire parameters and target encounter
parameters in \(\lambda\), with \(\lambda^+=\lambda\). Their domain may
shrink with new information but is not independently regenerated at future
steps. Constant target acceleration/sideslip is a prediction assumption for
the encounter, not an extra restriction on the standalone NRMM observer.

**A small observer interface.** Equations (S1)--(S8) in
[the observer derivation](../estimator/OBSERVER_ISS_THEORY.md#compact-sampled-comparison)
give explicit sampled bounds for the position/IMU filter, body/target core,
vector yaw innovation, and published ego position. At synchronized detected
samples a candidate interface is
\[
 e=[B_f,B_v,W_T,B_\psi,B_p]^T\ge0.                            \tag{C2}
\]
These are bounds on errors, not simulated observer estimates. Reset predictor
memory is eliminated only at reset instants, using its exact intersample
equations. Retain additional coordinates whenever an output, asynchronous
sensor phase, stop event or policy memory is not covered by (C2). In
particular an NRMM velocity/acceleration error bound is not by itself a target
orientation bound at zero target speed.

The two-state homogeneous sampled core contracts for a sufficiently short
sensor period under the explicit conditions (S4). The position filter
contracts in its Lyapunov metric under bounded acceleration discrepancy.
Neither fact establishes uniform contraction of their yaw-coupled
interconnection. A valid yaw bound may be the whole circle at standstill.

**Close the physical map with future feedback.** A finite-dimensional
causal policy class consists of rules
\[
 u_i=\kappa_i(\hat c_i,m_i;a_i),\qquad \hat c_i=c(z_i)+\epsilon_i,
                                                                  \tag{C3}
\]
where \(m_i\) is sufficient observable memory, \(a_i\) is the stored policy
parameter, and the error output \(\epsilon_i\) is enclosed by (C2).
This identity is an analysis device: the controller evaluates \(\hat c_i\),
not the unknown \(z_i\) or \(\epsilon_i\). At the current decision the
actual observer outputs and **all memory used by the controller** are fixed,
so (C3) has one common first command. Fixing observer memory alone would not
suffice for a policy depending on an unretained earlier reading. Allowing
independent physical and error coordinates must not
override that root equality. Later decisions permit different commands on
distinguishable histories. Control remains held across all intermediate
sensor updates in the decision period, as in (J4).

For the physical exact held-input transition \(F_h\), the controlled map is
\(F_h(z,\kappa_i(c(z)+\epsilon,m;a_i);\lambda)\). On a smooth chart its
derivative with respect to physical state includes the **combined** term
\[
 D_zF_h+D_uF_h\,D_{\hat c}\kappa_i\,D_zc,                    \tag{C4}
\]
and the relevant memory derivatives when memory varies. Bounding
\(D_zF_h\) and the feedback term separately before combining them can erase
stabilizing cancellation. Equation (C4) differentiates the exact held-input
map; it is not continuous feedback recomputed inside a control hold.
Whole-domain derivative bounds, or finite-increment bounds across nonsmooth
pieces, give the nonlinear comparison. A nominal Jacobian without its
remainder does not give that comparison.

**Constructive comparison coefficients.** Choose physical residual blocks
\(w_a\), metrics \(P_a>0\), and radii \(s_a\) with
\(\|w_a\|_{P_a}\le s_a\). Use the sampled observer functionals (C2) for
the remaining blocks. On a convex chart containing all comparison segments,
a valid cross-block coefficient for a differentiable output map \(\Psi_a\)
is
\[
 L_{ab}=\sup_{\mathcal D}
 \|P_a^{1/2}D_b\Psi_a P_b^{-1/2}\|_2.                        \tag{C5}
\]
The map \(\Psi\) includes (C3) before differentiation. Integrating its
derivative along a segment proves the finite-increment bound. Exact sampled
identities such as (S1), (S5), (S8), and common rotations such as (C1) should
be simplified before applying (C5). For observer blocks, signed-error
identities or comparison bounds are used rather than differentiating a norm
at zero. If no common smooth chart exists, partition the domain with valid
transition handling or use a directly verified finite-increment modulus.
Unbounded derivatives at a singular branch do not define finite coefficients.

Together with bounded forcing and persistent-parameter contributions, this
produces a monotone map of the form
\[
 \begin{bmatrix}s^+\\e^+\end{bmatrix}
 \le\mathcal Q_i(s,e,\Lambda;a_i),\qquad
 \mathcal Q_i(q,\Lambda;a_i)=M_iq+d_i(\Lambda),\quad M_i\ge0,  \tag{C6}
\]
when an affine majorant is available; a nonlinear order-preserving majorant
is also admissible. All observer forcings, held-sensor aging, inverse-map
uncertainty, target extension discrepancy and stop resets belong in this
map. Compose sensor-period maps with the input held, then the decision reset;
do not apply a new control policy after every sensor sample. The coefficient
domain must enclose the resulting trajectories and every allowed memory.
Domain containment is part of the certificate, not an outcome assumed after
an otherwise invalid bound has been calculated.

Equation (C5) requires certified whole-domain constants. The five observer
radii alone do not supply the physical-flow and force-inverse bounds in
that equation; those bounds are separate premises of the comparison.

**Retain shared uncertainty where it matters.** A more informative physical
representation is
\[
 z_i=\bar z_i+G_i\eta+w_i,\qquad \eta\in\Lambda,\qquad
 \|w_{a,i}\|_{P_{a,i}}\le s_{a,i}.                            \tag{C7}
\]
Selected columns carry common pose errors, persistent parameters or shared
sensor primitives. A column shared by two outputs is combined before taking
its directional support. Nonlinear products and unretained histories are
enclosed in residual blocks; no affine exactness is assumed. Fresh sensor
noise is introduced once per actual sensor event, and all its uses share
that primitive. A fixed generator budget may absorb old nonpersistent
columns into a sound residual bound. It must not erase the identity of
persistent physical parameters. The state dimension of the comparison can
stay fixed without preserving every statistical or geometric dependence.

A product of separately bounded plant and observer consequences of one
parameter is a conservative enlargement. It is not necessary to add a
second, independently invented mismatch disturbance on top of those same
consequences. Retaining their shared columns can reduce that enlargement.
Noise need not be independent, zero-mean, or averaged down; bounded
deterministic sensor errors are sufficient for these inequalities.

**What bounded width actually requires.** Suppose the full relevant
comparison, after removing constant parameters into the forcing/domain,
has a common weight \(v>0\) and constants \(0\le\gamma<1\), \(d<\infty\)
such that, uniformly over the admitted policies and domains,
\[
 M_i v\le\gamma v,\qquad \|d_i\|_v\le d,
 \quad\|q\|_v=\max_j |q_j|/v_j.
\]
Then induction gives
\[
 \|q_i\|_v\le\gamma^i\|q_0\|_v+
                    d(1-\gamma^i)/(1-\gamma).               \tag{C8}
\]
A radius may increase toward this noise floor without diverging. More
generally, a uniform product bound
\(\|M_{i-1}\cdots M_j\|\le C\gamma^{i-j}\) gives the analogous
geometric-series bound and permits expanding individual steps. If neither
condition is available, the finite-horizon recurrence can still be sound,
but bounded long-run width has not been proved. Position-only sensors at
rest do not justify a positive yaw-contraction rate; saturated steering does
not justify arbitrary relative-motion contraction. Constant uncertain target
parameters cannot contract under \(\lambda^+=\lambda\) in a positive
definite full-state metric. Thus (C8) is a checkable additional condition,
not a universal consequence of having an observer. It is not needed for the
recursive-feasibility argument that follows.

**Finite certificate semantics and collision constraints.** Write
\(\mathcal C=(\bar z,G,\Lambda,q,\mathcal D,\mathcal M)\), where
\(\mathcal M\) constrains observable memory. Define \(\Gamma(\mathcal C)\)
as the set of full joint states consistent with these data and their
anchored error functionals. There must be one physical/observer realization
behind each member, with shared parameters; alternatively a deliberate
outer product enlargement is allowed if all subsequent claims cover that
larger set. At the current root \(\mathcal M\) fixes actual memory. At
future stages it covers every possible memory required by the policy and
the sampled error contract.

Retained primitive values are part of this latent augmented state: all
functionals of a member use the same \(\eta\) in (C7). Selecting a separate
generator value for each inequality would discard the asserted joint
semantics. Primitive retirement is an explicitly sound set enlargement.

For fixed anchors, metrics, generators, domain and memory semantics,
smaller radii and nested parameter/memory domains imply set inclusion.
This is the order property used below. The essential admission test is
\[
 \mathcal T(\Gamma(\mathcal C_i),\pi_i)
      \subseteq\Gamma(\mathcal C_{i+1}),                      \tag{C9}
\]
uniformly over all admissible noise blocks. It is established by the sampled
comparisons, nonlinear residual enclosure, and domain checks. Merely
calculating (C6) at one reference does not establish (C9). Require
\(q_{i+1}\ge\mathcal Q_i(q_i,\Lambda_i;a_i)\), not equality: a previously
valid larger successor can then be retained after the root is tightened.
When monotonicity of a newly computed bound is unavailable, the stored
uniform certificate for the old set remains valid on its subsets.

Collision witnesses can be checked directly in the ego frame. Let \(V_E,V_T\)
be the body-fixed occupied rectangles including reference-point offsets and
\(h_V(n)=\sup_{v\in V}n^Tv\). A unit vector \(n\) from ego toward target
satisfies the sufficient separation condition
\[
 g=n^T\rho-h_{V_E}(n)-h_{V_T}(-R(\alpha)^Tn)\ge0.           \tag{C10}
\]
Use \(\inf_{\Gamma(\mathcal C_i)}g\ge-\xi_i\) for this witness.
This fixes the normal in the ego frame. If the optimization requires one
common **world-frame** normal \(n_W\), as in the corresponding common-plane
formulation of (U5), evaluate (C10) with
\(n=R(\psi_E)^Tn_W\) for each member before taking the infimum. A common
ego-frame normal is not silently interchangeable with a common world-frame
normal under uncertain ego yaw. Both give downward-closed sufficient
certificates, but they are different restricted witness families. The
chosen family must be retained under shifting and in terminal appending.
For example, position residual \(s_\rho\) and circular relative-yaw
radius \(B_\alpha\le\pi\) give, for a fixed ego-frame normal, the valid
geometric reserve
\[
 s_\rho+2r_T\sin(B_\alpha/2),\qquad
 r_T=\max_{v\in V_T}\|v\|,
\]
with position measured in the Euclidean norm and shared-generator support
added or jointly bounded. This is a conservative example, not a required
loss of all position/yaw dependence. The exact support over shared
primitives can be smaller. No positive physical stand-off is introduced.
At zero deficit (C10) excludes interior overlap at the declared nodes and
permits contact. Other hard constraints and the terminal family must use
the same certificate semantics, including any common-witness requirement.

**Nested compression after measurements.** Let \(\mathcal I^+\) be the
posterior obtained by conditioning the old first successor on the entire
received sensor block, as in (U2). It contains the true joint state and is a
subset of \(\Gamma(\mathcal C_{1|k})\). Keeping the old first-successor
anchor, metrics, generators, and domain, choose any bounds
\[
 \phi_j(S;\bar z_1,G_1)\le\beta_j(B)
       \quad\hbox{for every }S\in\mathcal I^+,
 \qquad q_j^+=\min\{q_{j,1|k},\beta_j(B)\}.                 \tag{C11}
\]
Fix the actual new memory and intersect \(\Lambda\) only with sound
parameter restrictions. These operations give
\[
 \mathcal I^+\subseteq\Gamma(\mathcal C_0^+)
              \subseteq\Gamma(\mathcal C_{1|k}).             \tag{C12}
\]
No explicit posterior set is needed if each restriction is proved sound.
Using the old radii and old parameter domain, with actual memory fixed, is
always a valid fallback under the transition contract. Ignoring some
measurements in the compressed update is conservative, but should not be
called exact conditioning. Error bounds \(\beta_j\) must refer to the
**same anchored functionals**: a radius about the new estimate cannot be
substituted for a radius about the old prediction without conversion.
An empty update signals a violated or inconsistent information/model
contract; it cannot be made sound by silently deleting the old certificate.

A smaller recentered ball does not ensure (C12). For example, the cap
\(\{x:\|x\|\le1,\ x_1\ge1/2\}\) has an enclosing ball centered at
\((1/2,0)\) of radius \(\sqrt3/2\); this ball extends outside the old
unit ball. For Euclidean balls, a sufficient replacement condition is
\(\|c_{\rm new}-c_{\rm old}\|+r_{\rm new}\le r_{\rm old}\).
General metrics, generators and nonlinear error functionals require their
own inclusion map. Recentring must also transport the policy, rather than
change what the retained feedback command means.

**Theorem C: compact-certificate recursive feasibility.** Fix the same
decision schedule and horizon \(N\) as in (U5). Replace its reachable sets
by \(\Gamma(\mathcal C_i)\), use (C9), and permit the inherited anchors,
metrics, generators, policies, domains and witnesses as feasible decisions.
Assume:

- The initial certificate contains the true state and has a feasible plan.
- All propagated certificates satisfy the uniform nonlinear contract (C9),
  hard constraints, and the stated uncertainty/detection contract.
- Actual updates satisfy (C12), with a common first command and admissible
  restriction of the old causal policy tail.
- The assumed terminal family has a zero-slack append **representable in
  this certificate and policy class**. Abstract pointwise terminal safety
  alone does not establish this representation property.
- Every optimization used for the optimal-value statement attains its
  primary minimum. For example a nonempty compact feasible sublevel in a
  finite-dimensional closed policy/certificate parametrization and lower
  semicontinuous objective suffice.

Let \(V_C(\mathcal C_0)\) be the minimum nonnegative slack sum over these
certified plans. Executing its first command gives
\[
 0\le V_C(\mathcal C_0^+)\le
       V_C(\mathcal C_0)-\xi_0^*\le V_C(\mathcal C_0).        \tag{C13}
\]
In particular, zero certified slack stays zero at every subsequent active
decision in this encounter.

**Proof.** By (C12), the new root is a subset of the old first successor.
The old second successor remains a valid image bound under the old next
policy, because (C9) was valid for every old first-successor member. Repeat
this restriction through the stored tail. Downward hard/collision conditions
retain the old witnesses and slacks. Apply the assumed representable terminal
append with zero slack. The resulting candidate has slack vector
\((\xi_1^*,\ldots,\xi_N^*,0)\), hence cost
\(V_C(\mathcal C_0)-\xi_0^*\). The next attained minimum cannot exceed it.
True-state containment and nonnegative slacks establish the stated safety
consequence. This proof does not require individual radius contraction or a
constant-width tube. QED.

For the same underlying information,
\(V_P(\mathcal I)\le V_C(\mathcal C)\) if the compact root encloses
\(\mathcal I\) and every compact plan is admissible in (U5), including
its policy and geometric-witness family. No such ordering is asserted
merely from using a different coordinate-dependent witness family.
Equation (C13) concerns the compact certificate value. It does not establish
the corresponding decrease of the exact optimal value along controls that
only minimize the compact problem. With a retained feasible plan and a
certified improvement instead of an attained optimum, (U8) applies to the
stored cost rather than to either optimal value.

**Finite representation and computational meaning.** A finite-memory Markov
policy can use the observer states, prior input, clock and a bounded set of
policy parameters as sufficient observable memory. The physical model and
future-noise contract must be Markov in the augmented state, with persistent
parameters retained. Then the inherited candidate stores \(N\) stage
certificates and policies, not an ever-growing measurement tree. Arbitrary
history-dependent policies or noise laws do not automatically admit this
finite representation. Terminal concatenation must stay in the selected
class. Keeping a literal chain of old histories is not a constant-memory
representation of this construction.

Fresh planning may use a different reference; it cannot delete the inherited
candidate solely because the new local parameterization excludes it. One
retained candidate with its own valid certificate is sufficient for (U8).
A fixed-size outer representation can discard information conservatively;
it cannot represent every nonlinear posterior exactly. No bounded number of
generators is asserted to preserve all useful correlations without inflation.

A bounded total description also requires fixed templates or budgets for
\(\Lambda\), domains, functionals and controller-memory updates. Intersecting
new arbitrary halfspaces forever, or hiding a growing history expression
inside one functional, defeats that claim. Componentwise parameter bounds
can be intersected in a fixed box template; keeping the previous domain is
the sound fallback when no representable tighter restriction is available.
Generator reduction or domain simplification that expands the current root
must satisfy (C12), or separately recertify the retained tail on the expanded
root. The number of radii alone does not settle these issues.

The following bounds provide constructive physical and observer remainders.
The compatible terminal family remains an assumption of Theorem C.

<a id="compact-constructive-enclosure"></a>
### Constructive nonlinear transition certificates

**Finite force increments, including full braking.** Put
\(\sigma(b)=\sqrt{1-b^2}\), \(s=C\tan\alpha\), \(Q=\mu F_z\sigma\), and
\[
 T(s,Q)=-\operatorname{sgn}(s)Q
 \{1-[1-\min(|s|/(3Q),1)]^3\},\qquad T(s,0)=0 .
\]
For \(Q>0\), with \(z=\min(|s|/(3Q),1)\),
\[
 T_s=-(1-z)^2,\quad T_Q=-\operatorname{sgn}(s)z^2(3-2z),
 \quad |T(s,Q)-T(\bar s,\bar Q)|\le|\Delta s|+|\Delta Q|.      \tag{D1}
\]
The increment bound follows by integrating the two partial derivatives,
both bounded by one, then taking the continuous extension at \(Q=0\).
For a braking interval, extrema of \(b^2\) occur at its endpoints and,
when included, zero. Taking square roots of \(1-b^2\) gives its exact
availability range. Use that finite range at saturation, rather than an
infinite braking derivative. Share one \(\sigma\) between axles and retain
the same persistent \(C_f,C_r,\mu_f,\mu_r\). The identity \(|T|\le Q\)
tightens the formula's image; it introduces no extra friction constraint.

If \(Q\ge Q_{\min}>0\), a sharper optional centered remainder is
\[
 |R_T|\le {|\Delta s|^2\over3Q_{\min}}
 +{|\Delta s\,\Delta Q|\over2Q_{\min}}
 +{4|\Delta Q|^2\over9Q_{\min}}.                            \tag{D2}
\]
This uses bounds \(2/(3Q_{\min}),1/(2Q_{\min}),8/(9Q_{\min})\) on
the second partials; the first derivative is Lipschitz across the joins.
A sign-fixed, fully saturated force is exactly linear in \(Q\).

**Explicit state derivatives and exact physical flow.** Input and
parameters are held, so a time-integration remainder needs only \(D_xf\).
On a slab with \(v_x\ge v_*>0\), \(|v_y|\le Y\), \(|r|\le R\),
\(|v_x|\le V\), and \(|\alpha_j|\le\alpha_{j,*}<\pi/2\), define
\[
 D_j=C_{j,\max}\sec^2\alpha_{j,*}
 [(Y+l_jR)/v_*^2,\ 1/v_*,\ l_j/v_*].
\]
These bound tire state derivatives directly from (D1) and the slip formula.
With \(s_\delta,c_\delta\) bounding absolute steering sine and cosine,
the body-state Jacobian has the componentwise majorant
\[
 L_b=\begin{bmatrix}
 s_\delta D_f/m+[L_{\rm load}/m,R,Y]\\
 (c_\delta D_f+D_r)/m+[R,0,V]\\
 (l_fc_\delta D_f+l_rD_r)/I_z
 \end{bmatrix}.                                             \tag{D3}
\]
For the actual load
\(\ell(v)=\rho C_d A v|v|/2+
 mg(c_0+c_1|v|+c_4|v|^4)\tanh(v/v_0)\), use
\[
 L_{\rm load}=|\rho C_d A|V+
 mg[|c_1|+4|c_4|V^3+(|c_0|+|c_1|V+|c_4|V^4)/v_0].
\]
Pose rows are bounded by \(L_{1:2,3}=V+Y\),
\(L_{1:2,4:5}=1,L_{3,6}=1\). The low-speed formula is differentiated
directly. Across its cubic blend add
\(1.5|f_{\rm Fiala}-f_{\rm kin}|e_{v_x}^T/(v_2-v_1)\) to the
componentwise maximum of the two branch bounds. Evaluate the dynamic branch
only on its actual positive-speed domain. These expressions give componentwise bounds on the admitted domain.

For an initial set \(X\), one held causal command \(U\), and persistent
parameters \(\Theta\), certify a slab box \(Z\) satisfying
\[
 X+[0,d]f(Z,U,\Theta)\subseteq Z .
\]
Let \(F\ge|f(Z,U,\Theta)|\) and \(L\ge|D_xf|\) be the bounds above. Then
\[
 F_d(X,U,\Theta)\subseteq X+d f(X,U,\Theta)
                   +[-r_d,r_d],\quad r_d=\tfrac12d^2LF.       \tag{D4}
\]
The Picard inclusion proves containment throughout the slab. The remainder
is \(\int_0^d(d-t)D_xf\,f\,dt\), valid almost everywhere through smooth
blend/saturation joins. Thus (D4) covers the exact ODE, not just an RK step.
At full braking the held state derivative is finite. Expand or subdivide
the certification slab until inclusion succeeds; reject an unresolved
domain. Subdivision creates no new control decision or sensor measurement.
The exactPlant action implements this rule.

Evaluate the signed affine expression \(X+d f(X,U,\Theta)\) before taking
supports. In particular compute \(U=\kappa(\hat c_i)\) once from the
joint decision-boundary set and retain it throughout the hold. Shared
state/feedback columns then cancel before norm bounds, as required by (C3).
The residual bound is not a replacement open-loop propagation of those
columns.

**Small observer kernels with a complete nonlinear remainder.** During
one held sensor block use
\(y=(\hat v,\hat\rho,\hat q,\hat s,\lambda)\), where
\(\lambda=y_{\rm predictor}-\hat\rho\). Its equations are
\[
 \dot y=\mathcal A y+\mathcal B[a_m+k_v\tilde v;\Phi_e(\hat q,\hat s)],
 \quad\mathcal A=A_s\otimes I_2-I_5\otimes\omega_mJ,
\quad A_s=\begin{bmatrix}
 -k_v&0&0&0&0\\-1&0&1&0&a_1\\0&0&0&1&a_2\\
 0&0&0&0&a_3\\0&0&0&0&-a_1
 \end{bmatrix}.                                             \tag{D5}
\]
\(\mathcal B\) injects its vector inputs into blocks one and four.
On detection reset \(\lambda=y_{\rm radar}-\hat\rho\), one shared
expression. Without detection retain it and set all innovation gains to
zero. The only nonlinear forcing in (D5) is the actual saturated NRMM map.

Freeze that forcing at the slab's initial state for the center map only.
Let the already proved global NRMM Lipschitz coefficients be \(L_q,L_s\),
\(\Omega\ge|\omega_m|\), and \(\Lambda\ge\|\lambda(0)\|\).
Displacements from the initial target estimate satisfy the positive system
\[
 \dot d\le
 \begin{bmatrix}\Omega&1\\L_q&L_s+\Omega\end{bmatrix}d+
 [\|\dot{\hat q}(0)\|;\|\dot{\hat s}(0)\|]
 +t[|a_2|;|a_3|](a_1+\Omega)\Lambda,\quad d(0)=0.             \tag{D6}
\]
Initial derivatives are whole-set evaluations of (D5), not sampled maxima.
The last term follows from the exact decaying innovation.
With \(j(t)=[L_q,L_s]d(t)\), the endpoint forcing remainder in
\((\hat\rho,\hat q,\hat s)\), in vector norms, is bounded by
\[
 \left[\int_0^d{(d-t)^2\over2}j(t)\,dt;\
       \int_0^d(d-t)j(t)\,dt;\
       \int_0^d j(t)\,dt\right].                            \tag{D7}
\]
One seven-dimensional matrix exponential evaluates (D6)--(D7), including
constant and time coordinates. Rotations have norm one, so no additional
exponential factor is needed. This is targetObserverResidual.

The exactObserver action also covers uncertainty in held gyro. Append the
constant forcing to the state in (D5), apply the exponential series to
shared affine variables, and bound its tail after order \(p\) by
\[
 (I-d\widehat A/(p+2))^{-1}
 {(d\widehat A)^{p+1}\over(p+1)!}|y_{\rm augmented}(0)|,       \tag{D8}
\]
where \(\widehat A\ge|\mathcal A_{\rm augmented}|\) and
\(\rho(d\widehat A/(p+2))<1\). Later Taylor factors are bounded by that
nonnegative ratio matrix. Subdivide if the test fails.
The published-position map is exactly
\[
 \hat p(d)=p_m+d\hat v_I+e^{-k_pd}(\hat p(0)-p_m).
\]
For \(\dot{\hat\psi}=\omega_m+
k_\psi(Q_2\cos\hat\psi-Q_1\sin\hat\psi)\), with
\(\|Q\|\le Q_*\), its centered Euler remainder is bounded by
\[
 \tfrac12 d^2 k_\psi Q_*(|\omega_m|+k_\psi Q_*).              \tag{D9}
\]
This also covers standstill. A true correspondence has norm at most one;
its affine outer set may require a larger \(Q_*\). No yaw contraction is
assumed. Certification subdivisions inherit the original innovation and
held measurements.

**The forcing coefficients are computed from the same physical branch.**
At actual sensor events evaluate
\(a_B=(\dot v_x-rv_y,\dot v_y+rv_x)^T\) using the same state, command,
and persistent tire variables. Add each position, IMU, gyro, and radar
noise once. Apply the actual position/IMU filter reset and construct the
nominal force-balance pseudo measurement through its finite, clamped
inverse. Interval bisection retains both possibilities for an unresolved
predicate. There is no new independent tire-mismatch input in the observer.
The inverse and the product-normalized yaw correspondence already have
whole-domain arithmetic in nrmmJointEnclosure.

The boundary IMU uses the previous command, whereas (D4) uses the newly
issued command. Thus the acceleration jump
\(a_B(x,u^+,\theta)-a_B(x,u^-,\theta)\) is included explicitly, with
the same \(\theta\). Future target poses use the analytic constant-curvature
flow, persistent encounter parameters, and forward stopping. This remains
a controller prediction convention, not a new standalone-observer premise.

**Transition-containment proposition.** Suppose the root set contains
the true physical states, observer memories and persistent parameters;
the sensor-noise bounds hold; the held causal input is admissible; and the
whole-domain/slab tests above succeed. Compose sensor resets, the
position-filter map, (D4)--(D9), and the analytic target flow. At an
ambiguous perception guard propagate both detection outcomes and join
their observer images. The result encloses every exact continuous
plant/observer realization at the next decision.

Proof is induction over the actual sensor events. Each reset is an outer
image with shared primitives; (D4) encloses the physical flow; (D6)--(D8)
bound all target-observer forcing changes; (D9) and the exact position
formula cover the remaining observer blocks. Inputs are held, so later
measurements do not change earlier physical motion. Persistent parameters
are copied. Every composition therefore preserves inclusion. Fixed current
observer memory makes the first command common.

A joint set crossing the perception guard has both observer continuations
enclosed. For affine images \(A,B\), the join
\((A+B)/2+[-r,r]\), \(r\ge|(A-B)/2|\), contains both images
with their shared primitive assignments. Joining loses some correlation
but cannot discard a possible detection history.
These inclusions supply the transition premise of Theorem C. Together with
(C12) and the assumed compatible terminal append, that theorem gives
recursive feasibility and nonincreasing deficit. Small width, universal
admission, global yaw contraction and finite-time zero deficit do not follow.

### Fixed-dimensional ellipsoidal certificates

A fixed-dimensional representation can use certified joint ellipsoids.

Let \(z=(x_E,x_T,\theta,m)\) contain both physical vehicles, persistent
parameters and finite observer/controller memory. At a future decision,
compose the causal feedback with the sensor-block flow before differentiating.
At the current decision the held command is common, so \(\delta u_0=0\).
Persistent parameters obey \(\theta^+=\theta\); their noise rows are zero.
Assume the nonlinear remainder on the entire admitted domain has been enclosed,
for example by (D4)--(D9). No point Jacobian alone supplies this enclosure.

Define \(\mathcal E(c,Q)=\{c+Q^{1/2}v:\|v\|_2\le1\}\), including singular
\(Q\succeq0\). A shared block expansion is
\[
 \begin{bmatrix}z^+\\y\end{bmatrix}
 =\bar w+F(z-\bar z)+G\eta+r,\quad
 F=\begin{bmatrix}A\\C\end{bmatrix},\quad
 G=\begin{bmatrix}B\\D\end{bmatrix}.
\]
Here \(y\) is the complete received block; the SAME noise coordinates appear
in the physical/observer successor and its measurement rows. Given
\(z\in\mathcal E(c,Q)\), \(\eta\in\mathcal E(0,R)\) and
\(r\in\mathcal E(c_r,R_r)\), set \(Q_0=FQF^T\), \(Q_1=GRG^T\),
\(Q_2=R_r\). For positive weights summing to one on nonzero terms,
\[
 c_J=\bar w+F(c-\bar z)+c_r,\qquad
 Q_J=\sum_j Q_j/\alpha_j.                                  \tag{E1}
\]
This encloses the joint image. Indeed its directional support follows from
\((\sum_j\sqrt{a^TQ_ja})^2\le\sum_j a^TQ_ja/\alpha_j\).
For a selected scaling \(W\succ0\), the weighted-trace minimizing weights
are proportional to \(\sqrt{\operatorname{tr}(WQ_j)}\). A componentwise
remainder \(|r-c_r|\le b\) admits the constructive choice
\(R_r=s\operatorname{diag}(b^2)\), where \(s\) counts nonzero components.
A remainder generator box with \(q\) columns admits \(R_r=qH_rH_r^T\).
These enlargements are conservative; replacing their factors by one is not
an enclosure. A single ellipsoid can leave a model's parameter domain, so
retain that fixed domain as an additional intersection or reject the candidate
if its propagated enclosure cannot be evaluated within a certified domain.

Partition \(Q_J=\left[\begin{smallmatrix}T&S\\S^T&V\end{smallmatrix}\right]\)
according to \((z^+,y)\). Write \(e=y_{\rm obs}-(c_J)_y\) and
\(\tau=e^TV^\dagger e\). Its measurement slice is nonempty exactly when
\(e\in\operatorname{range}(V)\) and \(\tau\le1\). Its state projection is
\[
 \mathcal E\left((c_J)_z+SV^\dagger e,
       (1-\tau)(T-SV^\dagger S^T)\right).                  \tag{E2}
\]
To prove this, factor \(Q_J=HH^T\), partition \(H=(H_z^T,H_y^T)^T\),
and write every latent solution as
\(v=H_y^\dagger e+w\), \(w\in\ker H_y\),
\(\|w\|^2\le1-\tau\). Applying \(H_z\) gives (E2), including the
singular cases. A pseudoinverse without the range test is insufficient.
The actual observer memory is known after the complete block. Its exact
software value can be imposed as another linear slice of this certified
joint set. With nonlinear observer updates, this is justified by the joint
nonlinear enclosure, not by equating an uncorrected tangent prediction to
the exact update.

If the inherited successor is the state projection of this same joint
ellipsoid, (E2), including the known-memory slice, is a subset of it.
Thus the actual posterior and inherited tail satisfy (C12). Keep the old
successor/tail certificates and policy after conditioning; do not require a
fresh ellipsoidal enlargement of the tail to fit inside the old one. Extra
information may be discarded by retaining the inherited certificate when a
new outer approximation would break nesting. A replacement must enclose the
posterior AND be contained in the inherited successor, or the old tail must
be recertified on it. For ellipsoid candidates a simple sufficient containment
test is
\[
 \|Q_h^{\dagger/2}(c_n-c_h)\|_2+
 \|Q_h^{\dagger/2}Q_n^{1/2}\|_2\le1,                       \tag{E3}
\]
with both the displacement and the range of \(Q_n\) inside
\(\operatorname{range}(Q_h)\). Keeping an offset relative to a new reference
is an exact coordinate change; moving the set's physical center is not.
Similarly a feedback reparameterization must preserve the SAME policy,
including its offset. These conditions, together with the assumed terminal
family and attained primary minimum, allow Theorem C's shift argument.
They do not establish small tubes or admission of an arbitrary initial state.

Constant storage does not retain every old measurement intersection exactly:
even a constant two-dimensional parameter with bounded linear measurements
can accumulate arbitrarily many strip facets. Ellipsoidal outer approximation
can discard that history while remaining rigorous under the conditions above.
This limitation concerns representation, not a need to abandon future feedback.

<a id="sources"></a>
## 8. Sources

The following sources provide the general PCBF, set-based estimation and
robust output-feedback control constructions used in these derivations.

- Zhang, Liniger and Borrelli (2018), *Optimization-Based Collision Avoidance*: [primary manuscript](https://arxiv.org/html/1711.03449v3), Section 4 and Theorem 2, for exact signed-distance dual geometry and unit-normal normalization with negative permitted clearance. Equations (G1)--(G2) use its rectangle vertex specialization.
- Cannon, Buerger, Kouvaritakis and Rakovic (2010), *Robust tubes in nonlinear model predictive control*: [primary manuscript](https://users.ox.ac.uk/~engs0169/pdf/cannon_nolcos10.pdf), Sections 2--4, for nonlinear remainder tubes and a retained feasible candidate.
- Scott, Raimondo, Marseglia and Braatz (2016), *Constrained zonotopes: A new tool for set-based estimation and fault detection*: [primary manuscript](https://web.mit.edu/braatzgroup/Scott_Automatica_2016.pdf), Definition 3 and Sections 3--5, for exact linear intersections and the cost of discarding generator constraints.
- Rego, Raffo, Terra and Scott (2025), *Set-based state estimation of nonlinear discrete-time systems using constrained zonotopes and polyhedral relaxations*: [preprint](https://arxiv.org/html/2504.00130v1), Section 3 and Proposition 3, for shared nonlinear graphs, polyhedral outer relaxations and measurement conditioning.
- Kochdumper and Althoff (2021), *Sparse Polynomial Zonotopes: A Novel Set Representation for Reachability Analysis*: [primary manuscript](https://arxiv.org/pdf/1901.01780), for preserving dependent polynomial factors through nonlinear reachability operations.
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
  bounded-error interface motivates the containment premise of Theorem U; an energy-gain LMI alone is not
  adopted as a pointwise Euclidean radius.
