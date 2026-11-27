# Output-feedback collision avoidance with an NRMM observer and a predictive control barrier function

## 1. Vehicle motion and observations

The nominal task is to follow a prescribed path at cruise speed $V_{\mathrm{ref}}>0$. During an encounter the controller chooses steering $\delta$ and signed longitudinal force ratio $b$ to avoid a target with unknown motion. The relative position of the target is the output that drives its motion observer.

**Perception assumption.** Let $p$ and $p_T$ be the two reference positions. Whenever $\|p_T(t)-p(t)\|\le R_d$, the relative position is observed in real time. The observation may have a bounded position error, with zero error giving exact observation. Outside this range the nominal task is path and speed tracking.

Write $e(\theta)=(\cos\theta,\sin\theta)^\top$, $J=\left[\begin{smallmatrix}0&-1\\1&0\end{smallmatrix}\right]$, $R(\theta)=[e(\theta),Je(\theta)]$, $[a]_+=\max(a,0)$, and $\operatorname{sat}_c(a)=\max(-c,\min(c,a))$. Norms are Euclidean. Angle differences use the principal wrapped angle.

### Ego vehicle

The input is $u=(\delta,b)$ with $|b|\le1$. The state is $x=(p,\psi,v_x,v_y,r)$, with inertial reference position $p$, body heading $\psi$, body velocity $v=(v_x,v_y)^\top$ and yaw rate $r$. Thus $\dot p=R(\psi)v$ and $\dot\psi=r$. Let $m,I_z$ be mass and yaw inertia, $g$ gravity, $l_f,l_r$ the axle distances, and $L=l_f+l_r$. For front and rear axles $i=f,r$, define

$$
(F_{zf},F_{zr})=mg(l_r,l_f)/L,\qquad
K_i=\mu_i F_{zi},\quad F_{xi}=bK_i,\quad D_i=K_i\sqrt{1-b^2}.
$$

The friction coefficients $\mu_i$ and cornering stiffnesses $C_i$ stay attached to their physical hypothesis. On the rolling branch the slip angles are $\alpha_f=\operatorname{atan2}(v_y+l_fr,v_x)-\delta$ and $\alpha_r=\operatorname{atan2}(v_y-l_rr,v_x)$. With $z_i=C_i|\tan\alpha_i|/(3D_i)$ when $D_i>0$, the lateral tire law is

$$
F_{yi}=\begin{cases}
0,&D_i=0,\\
-C_i\tan\alpha_i(1-z_i+z_i^2/3),&D_i>0,\ z_i<1,\\
-D_i\operatorname{sign}(\alpha_i),&D_i>0,\ z_i\ge1 .
\end{cases}
$$

Write $(X_f,Y_f)^\top=R(\delta)(F_{xf},F_{yf})^\top$. With air density $\rho_a$, drag coefficient $C_d$, frontal area $S_f$, rolling coefficients $c_0,c_1,c_4$ and smoothing speed $v_\epsilon>0$, define
$F_d(v_x)=\rho_a C_d S_f v_x|v_x|/2+mg(c_0+c_1|v_x|+c_4|v_x|^4)\tanh(v_x/v_\epsilon)$.
Force and moment balance give the rolling derivative of $(v_x,v_y,r)$,

$$
f_d=\left((X_f+F_{xr}-F_d)/m+rv_y,\quad
(Y_f+F_{yr})/m-rv_x,\quad(l_fY_f-l_rF_{yr})/I_z\right)^\top .
$$

At low speed let $r_\ell=v_x\tan\delta/L$, $\tau_\ell>0$, and
$X_\ell=([b]_++\min(b,0)\tanh(v_x/v_b))(K_f\cos\delta+K_r)$ with $v_b>0$.
The low-speed derivative is

$$
f_\ell=\left((X_\ell-F_d)/m+rv_y,\quad(l_rr_\ell-v_y)/\tau_\ell,\quad
(r_\ell-r)/\tau_\ell\right)^\top .
$$

For blending speeds $0<v_\ell<v_h$, set $s=\min(1,\max(0,(v_x-v_\ell)/(v_h-v_\ell)))$ and $w=s^2(3-2s)$. The body derivative is $(1-w)f_\ell+wf_d$, with the pure branches evaluated on their domains. Write $f^\vartheta(x,u)$ for the resulting vehicle vector field with command $u=(\delta,b)$ and persistent physical parameters $\vartheta$. At rest the body velocity and yaw rate are zero, and neutral drive preserves the pose. The forward-motion domain requires $v_x\ge0$ and, where the rolling law contributes, $v_x\cos\delta+(v_y+l_fr)\sin\delta>0$.

### Target vehicle

On a moving arc the target state is $(p_T,\psi_T,V,A,\beta)$, with nonnegative speed $V$, constant tangential acceleration $A$, constant sideslip $\beta$ and rear axle distance $l_T$. With $\kappa=\sin\beta/l_T$,

$$
\dot p_T=V e(\psi_T+\beta),\qquad
\dot\psi_T=\kappa V,\qquad \dot V=A,\qquad\dot A=\dot\beta=0.
$$

Braking to zero speed enters an absorbing stopped mode with constant pose and zero physical velocity and acceleration. The latent parameters of the moving arc persist. Starting with $V_0>0$, let $s(A,t)=V_0 t'+At'^2/2$, where $t'=\min(t,-V_0/A)$ for $A<0$ and $t'=t$ otherwise. Integration with respect to arc length gives

$$
z=\kappa s/2,\qquad
p_T(t)=p_0+s\,\operatorname{sinc}(z)e(\psi_0+\beta+z),\qquad
\psi_T(t)=\psi_0+2z .
$$

Here $\operatorname{sinc}(z)=\sin(z)/z$ with value one at zero. Indeed the course angle increases by $\kappa\,ds$ per unit travel. An already absorbed track uses $s=0$.

### Outputs and information

Let $\rho=R(\psi)^\top(p_T-p)$, with $\psi$ the ego heading. With persistent biases $\eta$ and bounded residual errors $n$, the outputs are

$$
\begin{aligned}
 y_p&=p+\eta_p+n_p,& y_r&=r+\eta_r+n_r,\\
 y_a&=\dot v+rJv+\eta_a+n_a,&
 y_\rho&=\rho+\eta_\rho+n_\rho\quad(\|\rho\|\le R_d).
\end{aligned}
$$

The accelerometer measures inertial acceleration in body axes. Denote the four error bounds by $\epsilon_p,\epsilon_r,\epsilon_a,\epsilon_\rho$. Conditional on an ego pose and bias, the inertial target position output is $y=p+R(\psi)(y_\rho-\eta_\rho)$, hence $\|y-p_T\|\le\epsilon_\rho$ by orthogonality.

Let $\chi$ contain the vehicle states, modes, persistent parameters and biases. Denote by $\mathscr H_t$ all histories that start in the initial set and satisfy the motion and observation equations with the applied input and the outputs received through $t$. The information $\mathcal I(t)$ comprises this consistent family, the known observer state, the position history used by the footprint bound, and the current encounter and actuator information. Write $s\in\mathcal I(t)$ to index a consistent history and its endpoint $\chi^s(t)$. The true history belongs to $\mathscr H_t$ when the initial, motion and measurement assumptions hold. Two histories with the same estimated current pose can provide different observed chords. All members receive the same control. The controller retains the committed plan to determine the inherited horizon and remaining deficit.

## 2. Nonlinear relative motion model observer

### Cartesian chain and inverse motion map

Write $q=\dot p_T$, $a=\ddot p_T$, $d=e(\psi_T+\beta)$ and $\Omega=\kappa V$. Then $q=Vd$, $\dot d=\Omega Jd$ and $a=Ad+V\Omega Jd$. Differentiating, using $\dot\Omega=\kappa A$, gives the closed chain

$$
\dot p_T=q,\qquad \dot q=a,\qquad
\dot a=\Phi(q,a)=-\Omega^2q+3A\Omega Jd,\qquad
A=d^\top a,\quad \Omega=(Jd)^\top a/V .
$$

In ego axes define $(\rho,q_B,a_B)=(R^\top(p_T-p),R^\top q,R^\top a)$ and $\nabla_r z=\dot z+rJz$. Then $\nabla_r\rho=q_B-v$, $\nabla_rq_B=a_B$, $\nabla_ra_B=\Phi(q_B,a_B)$. The inertial observation $y$ allows each retained ego member to use the inertial chain directly.

The Cartesian target chain follows the multistage NRMM observer construction. [Sharma, Alai and Rajamani](https://doi.org/10.1016/j.trc.2025.105411)

Let the declared physical domain bound $V,\|a\|,|A|,|\Omega|,|\beta|$ by $\bar V,\bar a,\bar A,\bar\Omega,\bar\beta$ with $\bar\beta<\pi/2$. These are domain assumptions shared by tracks. Define $d_v=\max(\|q\|,v_*)$, $0<v_*<\bar V$, $Q=\Pi_{\bar V}q$ and $S=\Pi_{\bar a}a$, where $\Pi_c$ projects onto the centered radius-$c$ ball. The globally defined propagation field is

$$
A_e=\operatorname{sat}_{\bar A}(Q^\top S/d_v),\quad
\Omega_e=\operatorname{sat}_{\bar\Omega}((JQ)^\top S/d_v^2),\quad
\Phi_e=-\Omega_e^2Q+3A_e\Omega_e Jq/d_v .
$$

It agrees with $\Phi$ on admissible moving states above $v_*$. For $0<V<v_*$ on the physical moving arc it equals $(V/v_*)^4\Phi$. Hence $\bar j_e=\sqrt{\kappa_{\max}^4 v_*^6+9\bar A^2\kappa_{\max}^2v_*^2}$, $\kappa_{\max}=\sin\bar\beta/l_T$, satisfies $\bar j_e\ge\sup\|\Phi-\Phi_e\|$ on smooth physical arcs, with the stopped branch treated separately. Ball projections and clips are nonexpansive, the denominators are bounded away from zero, and the differentiated factors involving $q$ are bounded in each radial region. Thus finite partial Lipschitz bounds $L_q,L_a$ exist for $\Phi_e$.

For an in-range target, let $e=y-\hat p$ be the instantaneous position innovation. The NRMM observer in inertial coordinates is

$$
\dot{\hat p}=\hat q+L_1 e,\qquad
\dot{\hat q}=\hat a+L_2 e,\qquad
\dot{\hat a}=\Phi_e(\hat q,\hat a)+L_3 e .
$$

In ego axes the same observer design uses the bias-corrected signals $y_r,y_\rho$, the innovation $e_B=y_\rho-\hat\rho$, and the covariant derivatives
$$
 \nabla_{y_r}\hat\rho=\hat q_B-\hat v+L_1 e_B,\qquad
 \nabla_{y_r}\hat q_B=\hat a_B+L_2 e_B,\qquad
 \nabla_{y_r}\hat a_B=\Phi_e(\hat q_B,\hat a_B)+L_3 e_B .
$$
The body-frame error comparison below includes errors in the measured rotation and estimated translation. The inertial form uses positions conditional on the ego history. A discrepancy in the observed position corrects all three Cartesian states. The moving target's velocity and acceleration follow from the chain without differentiating the measured position.

Write $\hat V=\|\hat q\|$. At positive estimated speed the inverse motion map gives

$$
\hat d=\hat q/\hat V,\quad
\hat A=\operatorname{sat}_{\bar A}(\hat d^\top\Pi_{\bar a}\hat a),\quad
\hat\Omega=\operatorname{sat}_{\bar\Omega}((J\hat d)^\top\Pi_{\bar a}\hat a/\hat V),\quad
\hat\beta=\arcsin\!\left(\operatorname{sat}_{\sin\bar\beta}(l_T\hat\Omega/\hat V)\right).
$$

On the identifiable moving branch the body heading is $\arg\hat q-\hat\beta$. At an absorbed stop the physical velocity and acceleration vanish, and position alone no longer identifies the latent sideslip. A stationary position history is compatible with every body heading, so a body-oriented footprint at rest requires known initial heading, previously resolved heading, or a separate heading observation.

### Input-to-state error bound

Write $L_i=\ell_i\omega^i$, $\omega>0$. For $(e_p,e_q,e_a)=(\hat p-p_T,\hat q-q,\hat a-a)$ form $Z=(e_p,e_q/\omega,e_a/\omega^2)$ and the metric $W=\sqrt{Z^\top MZ}$, $M=P\otimes I_2$, $P=P^\top\succ0$. Let

$$
A_\ell=\begin{bmatrix}-\ell_1&1&0\\-\ell_2&0&1\\-\ell_3&0&0\end{bmatrix},\quad
F(B,C)=\omega(A_\ell\otimes I_2)+(e_3\otimes I_2)[\,0,\ B/\omega,\ C\,] .
$$

Here $e_i$ in matrix expressions denotes a Cartesian basis vector of $\mathbb R^3$. A gain certificate requires some $\lambda>0$ such that
$F^\top M+MF\preceq-2\lambda M$ for every $(B,C)$ in a convex enclosure of the joint Jacobian $(D_q\Phi_e,D_a\Phi_e)$, including limiting branches. Integration along the segment between estimate and truth puts their finite field difference in this family. The certificate therefore bounds the actual nonlinear error. A common rotation of the three planar coordinates adds a skew transport term with zero metric derivative.

Let $T_d=R_d/(\bar V_E+\bar V)$, where $\bar V_E$ is the ego speed bound on the observer design domain. The gain design requires a certified decay rate $\lambda\ge 1/T_d$. The symbol $\otimes$ denotes the Kronecker product.

The gain condition uses a joint vector-Jacobian bound for the nonlinear increment. [Sharma, Zemouche and Rajamani](https://doi.org/10.1002/rnc.70732)

The Jacobian family can be obtained directly from the extension. For perturbations $(z,w)$ of $(q,a)$ write $\delta Q=D\Pi_{\bar V}(q)z$, $\delta S=D\Pi_{\bar a}(a)w$. Inside a projection ball its derivative is the identity. Outside, $D\Pi_c(y)=c(I-\hat y\hat y^\top)/\|y\|$, $\hat y=y/\|y\|$. Let $h=d_v$, $h'=0$ below the speed floor and $h'=q^\top z/\|q\|$ above. Let $\chi_A,\chi_\Omega$ be the slopes of the two clips on the smooth branch and set $a_1=Q^\top S/h$, $a_2=(JQ)^\top S/h^2$. Differentiation gives
$$
\begin{aligned}
 A'_e&=\chi_A\big[(\delta Q^\top S+Q^\top\delta S)/h-a_1h'/h\big],\\
 \Omega'_e&=\chi_\Omega\big[((J\delta Q)^\top S+(JQ)^\top\delta S)/h^2-2a_2h'/h\big],\\
 Bz+Cw&=-2\Omega_e\Omega'_e Q-\Omega_e^2\delta Q+
 3(A'_e\Omega_e+A_e\Omega'_e)Jq/h+
 3A_e\Omega_e(Jz/h-Jq\,h'/h^2).
\end{aligned}
$$
All factors in one derivative are evaluated at the same $(q,a)$. The closed convex hull of these paired matrices and their limiting values is a valid family. Along any segment the Lipschitz field is absolutely continuous and its increment is the integral of its directional derivative. At a switching surface a generalized Jacobian gives the tangential derivative. This proves the finite-error inclusion used above. Equivariance under a planar rotation $R$ conjugates $B,C$ by $R$. Also $M$ commutes with $I_3\otimes J$, so $M(I_3\otimes J)+(I_3\otimes J)^\top M=0$. This is why rotating the coordinate frame costs no homogeneous decay rate.

The mathematical criterion for selecting a gain also accounts for position noise entering the velocity estimate. In unscaled coordinates put $A_0=e_1e_2^\top+e_2e_3^\top$, $L=(L_1,L_2,L_3)^\top$, $A_L=A_0-Le_1^\top$. Among dissipative gains at the specified decay rate, minimize an upper bound $\gamma$ subject to
$$
 A_L^\top P_g+P_g A_L+e_2e_2^\top\preceq0,\qquad
 L^\top P_gL\le\gamma,\qquad P_g\succ0 ,
$$
with the same metric $P_g\otimes I_2$ in the unscaled dissipation condition. Indeed integration along $\dot z=A_Lz$ shows that $P_g$ dominates $\int_0^\infty e^{A_L^\top v} e_2e_2^\top e^{A_Lv}\,dv$. Multiplication on both sides by $L$ bounds the squared energy of the linear velocity response to a position impulse. This preference ranks gains. The nonlinear deterministic bound follows from dissipation.

Let $t_0$ begin an in-range observation interval with a bounded initial error set, and write $\nu=y-p_T$. For some admissible mean Jacobian $(B,C)$, subtracting the plant chain from the observer on a smooth motion branch gives

$$
\dot Z=F(B,C)Z+(\omega\ell\otimes I_2)\nu+
 (e_3\otimes I_2)(\Phi_e(q,a)-\dot a)/\omega^2 .
$$

The same position error drives all three rows. Differentiation of $W^2$ and Cauchy–Schwarz in the $M$ norm give
$$
D^+W\le-\lambda W+d_T,\qquad
d_T=\omega\sqrt{\ell^\top P\ell}\,\epsilon_\rho+
       \sqrt{P_{33}}\,\bar j_e/\omega^2 .
$$

Consequently, for $\Delta=t-t_0\ge0$ on that branch,
$$
W(t)\le B(t)=e^{-\lambda\Delta}B(t_0)
                 +(1-e^{-\lambda\Delta})d_T/\lambda,\qquad B(t_0)\ge W(t_0).
$$
By duality of the metric norm, $E_i=\omega^{i-1}\sqrt{(P^{-1})_{ii}}B(t)$ bounds the norm of error component $i=1,2,3$. Write these radii as $E_p,E_q,E_a$. At an absorbing stop the true position and velocity are continuous, while true acceleration jumps from $a^-$ to zero. The triangle inequality permits $B^+=B^-+\sqrt{P_{33}}\|a^-\|/\omega^2$ before continuing the bound on the stopped branch. If stop time is uncertain, the enclosure must cover both the moving and the absorbed histories.

### Ego observer in the body-frame formulation

In the ego body frame the target chain uses body velocity for translation and yaw rate for rotation. Let $m_I$ be ego inertial velocity reconstructed from position and body acceleration, with $\|m_I-R(\psi)v\|\le\epsilon_G$. All ego outputs in this section are bias corrected. The continuous position/IMU reconstruction with bandwidth $\omega_f>0$ has the form

$$
\dot p_f=m_I+2\omega_f(y_p-p_f),\qquad
\dot m_I=R(\hat\psi)y_a+\omega_f^2(y_p-p_f).
$$

For position and velocity errors $(e_f,e_I)$ the linear part in each axis is $H_f=\left[\begin{smallmatrix}-2\omega_f&1\\-\omega_f^2&0\end{smallmatrix}\right]$. Its characteristic polynomial $(s+\omega_f)^2$ is Hurwitz. The error is driven by position noise through $(2\omega_f,\omega_f^2)^\top$ and inertial acceleration mismatch through $(0,1)^\top$. A bounded initial error, bounded position error and bounded acceleration mismatch therefore give a finite $\epsilon_G$ by variation of constants. For an angle error bound $b_\psi\le\pi$, the mismatch is at most $\epsilon_a+2\|y_a\|\sin(b_\psi/2)$.

On the forward cone $v_x\ge\|v\|\cos b_E$, $0\le b_E<\pi/2$, write $M_v=\|m_I\|$. A lateral pseudo-observation $w_m$ gives
$y_v=(\sqrt{\max\{M_v^2-w_m^2,M_v^2\cos^2 b_E\}},w_m)^\top$.
Under $|r-v_y/l_r|\le d_\mathrm{kin}$ the kinematic choice $w_m=l_r y_r$ has radius $\rho_w=l_r(\epsilon_r+d_\mathrm{kin})$. On a valid rolling branch the shared force model also supplies an inverse observation. For each possible lateral velocity $w$, the longitudinal velocity is $v_x=\sqrt{V_E^2-w^2}$. The force balance restricts $w$ to those values for which $|Y_f+F_{yr}-m y_{a,y}|\le m\epsilon_a$ for some admissible speed, yaw rate and tire parameters under the applied input. Here $|V_E-M_v|\le\epsilon_G$ and the forward-cone constraint applies. For the chosen nominal inverse $w_m$, the supremum of $|w-w_m|$ on this set gives a valid $\rho_w$.

On an admitted monotone branch take the nominal lateral force $G_0(w)$ from the front and rear force balance with the observed yaw rate, reconstructed speed and known input. For a positive inverse regularization $\lambda_w(M_v)$, choose $w_m$ from $G_0(w_m)-\lambda_w(M_v)w_m=m y_{a,y}$, with endpoint saturation on $[-M_v\sin b_E,M_v\sin b_E]$. If $[w_-,w_+]$ contains all physically compatible lateral speeds, $\rho_w=\max(|w_m-w_-|,|w_m-w_+|)$ accounts for ambiguous inversion, including tire saturation. At rest an uninformative course leaves the full admitted direction set available.

With the transit-time gain choice $k_v=k_p=k_\psi=\ln(20)/T_d$, the scalar homogeneous error factor after one transit time is $1/20$ for the velocity and position corrections. The body and position observers obey
$\dot{\hat v}=y_a-y_rJ\hat v+k_v(y_v-\hat v)$ and
$\dot{\hat p}_E=m_I+k_p(y_p-\hat p_E)$. For the body-velocity error $e_v=\hat v-v$, subtraction gives $\dot e_v=(-k_v I-y_rJ)e_v+n_a-n_rJv+k_v(y_v-v)$. The longitudinal inverse has derivative bounds $\sec b_E,\tan b_E$ with respect to speed and lateral velocity. Consequently $\|y_v-v\|\le\sec b_E(\epsilon_G+\rho_w)$. Skew symmetry of $J$ gives

$$
D^+\|\hat v-v\|\le-k_v\|\hat v-v\|+
\epsilon_a+\epsilon_r\bar V_E+k_v\sec b_E(\epsilon_G+\rho_w).
$$

For $s_\psi>0$ define $(c,s)=(y_v^\top m_I,(Jy_v)^\top m_I)/(\|y_v\|\|m_I\|+s_\psi^2)$. The parallel yaw law is
$\dot{\hat\psi}=y_r+k_\psi(s\cos\hat\psi-c\sin\hat\psi)$.
If $\mathcal V$ encloses the body velocity, the compatible angles are
$\{\theta:\exists v'\in\mathcal V,\ \|m_I-R(\theta)v'\|\le\epsilon_G\}$.
Intersect these angles with the prior angle set transported by the gyro integral and its error interval. The maximum wrapped distance to $\hat\psi$ on that intersection bounds its output error. At rest the innovation can vanish for every heading. In body axes the target error metric also receives the disturbances from $\hat v-v$ and the common gyro error. The former enters the first row only, contributing at most $\sqrt{P_{11}}\|\hat v-v\|$ to $D^+W$; the latter contributes at most $\epsilon_r\sup\|(\,J\rho,Jq_B/\omega,Ja_B/\omega^2\,)\|_M$ over the domain, where $\|z\|_M=\sqrt{z^\top Mz}$. In particular, for continuous bounded innovation disturbances write $\dot E\le-k_vE+d_E$ and $\dot W\le-\lambda W+\sqrt{P_{11}}E+d_T$, where $E$ bounds the body-velocity error and $d_E,d_T$ collect the respective disturbance terms above. Scalar comparison first gives $\limsup E\le d_E/k_v$, then $\limsup W\le(d_T+\sqrt{P_{11}}d_E/k_v)/\lambda$.

For an explicit transient bound let $\zeta=(E,W)^\top$, $H=\left[\begin{smallmatrix}-k_v&0\\\sqrt{P_{11}}&-\lambda\end{smallmatrix}\right]$ and $b=(d_E,d_T)^\top$, with the gyro forcing included in $d_T$ for body coordinates. Since $H$ has nonnegative off-diagonal entries, $e^{H v}$ is nonnegative for $v\ge0$. Variation of constants yields
$$
 \zeta(t)\le e^{H(t-t_0)}\zeta_0+\int_{t_0}^t e^{H(t-v)}b(v)\,dv .
$$
An initial error set containing the true errors supplies $\zeta_0$. For the ego position error, subtraction gives $\dot e_{p,E}=-k_p e_{p,E}+(m_I-Rv)+k_p n_p$, so its ultimate norm is bounded by $\epsilon_G/k_p+\epsilon_p$. For yaw, with $e_\psi=\operatorname{wrap}(\hat\psi-\psi)$, ideal velocity correspondences give $(c,s)=a_\psi(\cos\psi,\sin\psi)$, $a_\psi=\|v\|^2/(\|v\|^2+s_\psi^2)$. Writing the actual pair as this ideal pair plus $\Delta$, one obtains
$$
 \dot e_\psi=-k_\psi a_\psi\sin e_\psi+n_r+
 k_\psi(-\sin\hat\psi,\cos\hat\psi)\Delta .
$$
Thus in an invariant chart $|e_\psi|\le b<\pi$, the local contraction rate is at least $k_\psi\inf a_\psi\,\operatorname{sinc}(b)$ when $\inf a_\psi>0$. The compatible orientation set supplies a bound even at rest. For a body vector with estimate error at most $E_z$ and heading error $B_\psi$, adding and subtracting the commonly rotated vector gives the inertial error bound $E_z+2\min(\|\hat z\|,\bar z)\sin(B_\psi/2)$ when $\|z\|\le\bar z$. Adding the ego position error then encloses the absolute target position. All bounds are conditional on a valid sensor and motion domain. The filter's acceleration input also depends on yaw, so the triangular comparison for body velocity and target motion alone cannot establish global yaw convergence.

### Persistent parameter and position enclosures

If $\hat V>E_q$, tangency to the velocity ball bounds the course error by $\theta=\arcsin(E_q/\hat V)$. Changing a unit direction by angle $\theta$ changes its vector by at most $2\sin(\theta/2)$. Thus both components of acceleration along and normal to the estimated course have error at most $d_a=E_a+2\|\hat a\|\sin(\theta/2)$. At each such time $t$ on a moving arc define

$$
\mathcal A(t)=\hat d^\top\hat a+[-d_a,d_a],\qquad
\mathcal K(t)=\frac{(J\hat d)^\top\hat a+[-d_a,d_a]}
 {[(\hat V-E_q)^2,(\hat V+E_q)^2]},\qquad
\mathcal B(t)=\arcsin\!\big(\operatorname{sat}_1(l_T\mathcal K(t))\big).
$$

The interval quotient contains all ratios of its numerator and denominator values. Since $(Jd)^\top a=\kappa V^2$, the true pair belongs to $\mathcal A(t)\times\mathcal B(t)$. Constancy of $A$ and $\beta$ gives the stronger bound

$$
(A,\beta)\in I_A\times I_\beta=
\big([-\bar A,\bar A]\times[-\bar\beta,\bar\beta]\big)
\cap\bigcap_{t\in\mathcal T}\big(\mathcal A(t)\times\mathcal B(t)\big),
$$

where $\mathcal T$ comprises the observed times on that moving arc for which $\hat V>E_q$. When $s_*>0$ lower-bounds a speed already attained, $|\sin\beta|\le l_T\bar\Omega/s_*$ also follows from $\Omega=V\sin\beta/l_T$. At an absorbing stop the motion derivatives vanish while the pre-stop parameter restrictions persist.

There is also a constraint on $A$ from displacement alone. For an observed interval $[a,b]$ of duration $T=b-a$, let $C=(y(b)-y(a))/T$ and $\epsilon=\epsilon_a^p+\epsilon_b^p$ be the sum of the two position error radii. Integrating the velocity under $\|\dot q\|\le\bar a$ gives $\|q(b)-C\|\le\epsilon/T+\bar a T/2$, including arcs ending in absorption. Let $\alpha=\bar\Omega T/2<\pi/2$. Projection of the whole velocity history onto the course at the interval midpoint and the chord triangle inequality enclose the mean speed in
$$
 [L,U]=\left[(\|C\|-\epsilon/T)_+,\,(\|C\|+\epsilon/T)/\cos\alpha\right].
$$
If $L>\bar A T/2$, absorption cannot have occurred on that interval because a speed ending at zero is bounded by $\bar A(b-v)$ there. On a moving interval the mean speed equals the midpoint speed by affinity of $V$. Therefore two such intervals on the same moving arc with midpoint times $m_1<m_2$ imply
$$
 A\in[(L_2-U_1)/(m_2-m_1),(U_2-L_1)/(m_2-m_1)] .
$$
Intersecting these bounds with $I_A$ uses the persistence of the *same* acceleration through both observations.

Let $\mathbb B(c,r)$ be the closed ball of radius $r$ about $c$. Conditional on the ego member, the current target reference position belongs to $\mathbb B(y,\epsilon_\rho)\cap\mathbb B(\hat p,E_p)$. Write $P_0$ for a set enclosing all current target positions consistent with the observed history. With exact relative position and known ego pose, $P_0$ may be the singleton $\{p_T\}$.

### Observing the future footprint from positions

Identify an inertial plane vector with a complex number. Then $n^\top z=\operatorname{Re}(\bar n z)$ and multiplication by $e^{i\psi}$ rotates a vector by $\psi$. Work conditionally on one ego and sensor-bias history so its position observations are in a common inertial frame. Choose the current time as zero and any earlier observed time $-T$ on the same target arc. Write $y_0,y_-$ for the observed positions and $\epsilon_0,\epsilon_-$ for their error radii. The physical clock $\theta=(V_0,A,\beta)$ determines the signed arc lengths $s_-\le0$ to the earlier position and $s(t)\ge0$ to a future position. The same clock governs both. Define
$$
 F_\kappa(s)=\int_0^s e^{i\kappa u}\,du
 =s\,\operatorname{sinc}(\kappa s/2)e^{i\kappa s/2},\qquad
 H_\theta=F_\kappa(s_-),\qquad
 G_{\theta,c}(t)=F_\kappa(s(t))+e^{i(\kappa s(t)-\beta)}c .
$$
Here $c$ is a body corner measured from the target dynamics reference point, including its offset. With $d=e^{i(\psi_0+\beta)}$, the earlier reference position and the future corner satisfy
$$
 p_-=p_0+H_\theta d,\qquad z_c(t)=p_0+G_{\theta,c}(t)d .
$$
This follows by integrating the target velocity in arc length and rotating the body offset by its final heading. For $H_\theta\ne0$, subtraction eliminates both the unknown initial position and course:
$$
 z_c(t)=(1-W_c)p_0+W_c p_-,\qquad W_c=G_{\theta,c}(t)/H_\theta .
$$
The complex coefficient includes a rotation. With exact observed positions this identity recovers the corner for that clock; a zero chord leaves the initial course unobservable from this pair. After absorption, the terminal pose persists. An already stationary target uses its position and admitted body-heading set.

For a moving member the NRMM velocity ball gives the speed interval $[\max(0,\|\hat q\|-E_q),\|\hat q\|+E_q]$, intersected with prior physical bounds. The positions further constrain this clock. Put $L=|y_0-y_-|$, $e=\epsilon_0+\epsilon_-$ and $\alpha=\bar\Omega T/2$, where $\bar\Omega$ bounds the yaw rate on the past arc. If $\alpha<\pi$, its length $S$ satisfies
$$
 (L-e)_+\le S\le(L+e)/\operatorname{sinc}(\alpha).
$$
Indeed the true chord is $S\,|\operatorname{sinc}(\kappa S/2)|$ and $|\kappa|S/2\le\alpha$. The lower bound holds for every turn. On a moving arc spanning both observations $V_0=S/T+AT/2$. For a braking target that stops within the interval $V_0=0$ and $S\le |A|T^2/2$. Thus clipping each endpoint of the interval for $S/T+AT/2$ below at zero also covers absorption. These relations restrict the clock without differentiating a measured position.

For any admitted clock family $\Theta$ and any complex $W$, the two position equations give the following support bound in a unit direction $n$:
$$
 \operatorname{Re}(\bar n z_c)\ \le \operatorname{Re}\!\left(\bar n[(1-W)y_0+Wy_-]\right)
 +|1-W|\epsilon_0+|W|\epsilon_-+
 \sup_{\theta\in\Theta}|G_{\theta,c}-WH_\theta|.
$$
Indeed $|d|=1$ and the last residual is the coefficient of $d$. The measurement covectors are $\overline{(1-W)}n$ and $\bar W n$. They depend on the corner, the prediction time and the separation direction. No independence of the two measurement errors is required. For any reference clock with $\bar H\ne0$, taking $W=\bar G_c/\bar H$ cancels its course term. If $|G_{\theta,c}-\bar G_c|\le\Delta G_c$ and $|H_\theta-\bar H|\le\Delta H$ throughout the clock family, then $\Delta G_c+|W|\Delta H$ bounds the residual. The current position and the arc equations independently enclose every corner and remain defined at $\bar H=0$.

For explicit radii let $d_s=\sup_\Theta|s-\bar s|$, $d_-=\sup_\Theta|s_--\bar s_-|$, $d_\kappa=\sup_\Theta|\kappa-\bar\kappa|$ and $d_\varphi=\sup_\Theta|\kappa s-\bar\kappa\bar s|+\sup_\Theta|\beta-\bar\beta|$. From the integral for $F$, $|\partial_sF|=1$ and $|\partial_\kappa F|\le s^2/2$. Since $|e^{iu}-e^{iv}|\le\min(2,|u-v|)$, one may take
$$
 \Delta H=d_-+\tfrac12\bar s_-^2d_\kappa,\qquad
 \Delta G_c=d_s+\tfrac12\bar s^2d_\kappa+|c|\min(2,d_\varphi).
$$

There is a single observed rectangle for all four corners. Let $d_y=(y_- - y_0)/\bar H$ and denote the reference clock by bars. The four centers of the observation bounds are
$$
 \bar z_c=y_0+d_y F_{\bar\kappa}(\bar s)
             + d_y e^{i(\bar\kappa\bar s-\bar\beta)}c .
$$
The multiplier of $c$ is common to every corner. Consequently their convex hull $R_y(t)$ is a translated, rotated and uniformly scaled copy of the physical body rectangle. Its scale is $|d_y|$, including for a rectangle whose reference point is offset. Let
$$
 r_y(t)=\max_{c\in\operatorname{vert}B_T}
 \{\Delta G_c+|W_c|(\Delta H+\epsilon_-)+|1-W_c|\epsilon_0\}.
$$
Every true corner lies within $r_y$ of its corresponding center. Using the same convex coefficients for a body point and its four centers proves $B_T(t)\subseteq R_y(t)\oplus\mathbb B(0,r_y(t))$. At a known absorbing pose, the latest position measurement defines $R_y=y_0+R(\psi_0)B_T$ and $r_y=\epsilon_0$. An unresolved position chord supplies no independent course witness.

Independently, for a member with initial speed $V_0$, body heading $\psi_0$ and mode, let $R_p(t)$ enclose the arc image over the NRMM position set $P_0$ and its persistent $I_A\times I_\beta$. The two footprint enclosures can use different outer bounds for the same member. In particular the position pair can use the entire NRMM speed interval to cover its clock error. Physical coverage across members requires the true initial speed, heading, mode and ego motion to be represented.

To construct $R_p$ explicitly, choose orthonormal enclosure axes $n_1,n_2$ aligned with the center of the predicted body-heading interval. For either axis $n$, a support bound is
$$
 h_p(n)=h_{P_0}(n)+\sup_{(A,\beta)\in I_A\times I_\beta}
 \left[n^\top s\,\operatorname{sinc}(\kappa s/2)e(\psi_0+\beta+\kappa s/2)
 +h_{B_T}(R(\psi_0+\kappa s)^\top n)\right],
 \quad h_S(n)=\sup_{z\in S}n^\top z .
$$
Include the stationary pose on an admitted absorbed branch. The rectangle $-h_p(-n_i)\le n_i^\top z\le h_p(n_i)$, $i=1,2$, contains every possible body point by the arc equations. Any outward upper bound for these four extrema gives the same inclusion property. The ego-to-target distance dual in Section 4 ranges over every unit normal. The two axes here specify an outer footprint for uncertain motion. Root position, heading, mode and sensor errors refer to the same history in both enclosures.

The target contributes a collision obligation while it is in perception range. In prediction, exit is established when every admissible reference position lies farther than $R_d$ from the ego reference point. The clearance obligation includes the boundary of the encounter. After certified exit it is released until the next encounter. An observed re-entry starts a new encounter.

## 3. One initial trajectory from a local acceleration field

### Route field

The fixed route has position $p_\Gamma(s)$, unit tangent $e(\theta)$, left normal $n_\Gamma=Je(\theta)$ and curvature $k_\Gamma$. Use closest-point station on the designated path. Let $e_y=n_\Gamma^\top(p-p_\Gamma)$, $v_c=\|v\|$, $\chi=\psi+\operatorname{atan2}(v_y,v_x)$, $\epsilon=\operatorname{wrap}(\chi-\theta)$, and $V_\mathrm{ref}>0$ be the cruise speed. On the regular chart $1-k_\Gamma e_y>0$, differentiation gives $\dot e_y=v_c\sin\epsilon$ and $\dot s=v_c\cos\epsilon/(1-k_\Gamma e_y)$. With lookahead $D=\max(D_{\min},T_D V_\mathrm{ref})$, define

$$
\epsilon_d=-\arctan(e_y/D),\quad
\dot\epsilon_d=-D v_c\sin\epsilon/(D^2+e_y^2),\qquad
r_d=\frac{k_\Gamma v_c\cos\epsilon}{(1-k_\Gamma e_y)}
       +\dot\epsilon_d-k_\chi\operatorname{wrap}(\epsilon-\epsilon_d),
$$

$$
a_0=k_v^{g}(V_\mathrm{ref}-v_c)e(\chi)+v_c r_d Je(\chi).
$$

The first component restores speed; the second rotates the course toward the route. $D_{\min},T_D,k_\chi,k_v^{g}$ are positive guidance parameters.

### Obstacle field

The field uses one nominal ego state and one predicted target rectangle. Work in the axes $R_T$ of that rectangle at the corresponding prediction time. Let $b=(b_1,b_2)>0$ be its half-extents, $o_T$ its body offset, $c=p_T+R_To_T$ its center, $\Omega=\kappa V$, and $\dot\Omega=\kappa A$. The point motion gives

$$
v_c^T=Vd+\Omega JR_To_T,\qquad a_c^T=Ad+V\Omega Jd+\dot\Omega JR_To_T-\Omega^2R_To_T,\quad d=e(\psi_T+\beta).
$$

For a moving occupancy enclosure let $c_B(t)$ and $b_B(t)$ be its center and half-extents. Its transverse growth contributes to the field through their rates of change on smooth arcs. Let $\gamma=\psi_T+\beta-\theta$ and let $h_1,h_2$ be the nominal target half-extents. With $[z]_{01}=\min(1,\max(0,z))$ and radial gain $k>0$ set
$\zeta=[(h_1|\sin\gamma|-h_2|\cos\gamma|)/h_2]_{01}[1+A/(k^2h_2)]_{01}$.
For a moving enclosure the effective center velocity is $(1-\zeta)v_c^T+\zeta\dot c_B$ and the effective extent rate is $\dot b_g=\zeta\dot b_B$. Use the former as $v_c^T$ below. For a known rigid target the point center velocity applies and $\dot b_g=0$.

Set $q_g=R_T^\top(p-c)$, $w_g=R_T^\top(R(\psi)v-v_c^T)-\Omega Jq_g$, and let $q_*=\Pi_{[-b,b]}q_g$. Outside the rectangle let $d_g=\|q_g-q_*\|$, $n=(q_g-q_*)/d_g$, and $k_g=1/d_g$ in a corner region and zero on a face. Inside or on the rectangle use $d_g=-\min_i(b_i-|q_{g,i}|)$ and $k_g=0$. To continue the field there, assign weights proportional to $\exp(f_i/\min_i b_i)$ to the four faces, with $f=(q_g-b,-q_g-b)$; $n$ is the weighted mean of their outward unit normals. This also defines a finite field at the center.

Let $t=Jn$, $n_W=R_Tn$, $t_W=R_Tt$, $\nu=n^\top w_g-|n|^\top\dot b_g$, $\tau=t^\top w_g$. For the ego body rectangle $B_E$, its support is $\sigma_{B_E}(z)=\max_{y\in B_E}z^\top y$. The field clearance is $g=d_g-\sigma_{B_E}(-R(\psi)^\top n_W)$. Let $\Delta_c=p+R(\psi)o_E-c$, $v_\Delta=R(\psi)v+rJR(\psi)o_E-v_c^T$, $u_\perp=Jv_\Delta/\|v_\Delta\|$ (zero at zero relative speed), and $y_\perp=u_\perp^\top\Delta_c$. With ego half-extents $b_E$, the combined transverse half-width is
$H=|R(\psi)^\top u_\perp|^\top b_E+|R_T^\top u_\perp|^\top b$.
The incidence coefficient is

$$
i=\max\{\mathbf 1_{g<0},\ \mathbf 1_{\Delta_c^\top v_\Delta<0}[1-(y_\perp/H)^2]_+\},
$$

where the closing term is zero at zero relative speed. With tangential gain $k_t>0$, lateral acceleration scale $a_y>0$ and positive field speed $s_g$, define

$$
\begin{aligned}
a_t&=k_t t_W t_W^\top
 \left(s_g u_\perp y_\perp/\sqrt{y_\perp^2+\min(b_{E,2},b_2)^2}-v_c^T\right),\\
a_\mathrm{rel}&=(-k_g\tau^2-2k\nu-k^2g)n+k_g\nu\tau\,t,\\
a_\mathrm{obs}&=R_T(a_\mathrm{rel}+2\Omega Jw_g+\dot\Omega Jq_g-\Omega^2q_g)+a_c^T+a_t,\\
c_n&=i[\,n_W^\top(a_\mathrm{obs}-a_0)\,]_+,\qquad
a_g=a_0+c_n n_W+\min(1,c_n/a_y)a_t .
\end{aligned}
$$

The radial expression comes from $\ddot d_g=n^\top\ddot q_g+k_g\tau^2$ outside the rectangle and the desired radial response $\ddot g+2k\dot g+k^2g=0$. Changes in ego support and envelope shape make this a guidance law. The transport terms follow by differentiating rotating coordinates twice. Choose $a_y$ as a prescribed fraction of $\min_i\mu_i g_\mathrm{grav}$, where $g_\mathrm{grav}$ denotes gravitational acceleration. Let $v_g>0$ be the guide speed scale. At the initial nominal target pose set $s_g=\min(\max(\|w_g\|,v_g),\sqrt{a_y\max(r_E,\|q_g\|)})$, with $r_E=\max_{y\in B_E}\|y\|$. The sign of the transverse contribution follows from the evolving $y_\perp$.

After predicted exit use $a_g=a_0$. For a finite route-guide half-width $W_g$, limit the component $n_\Gamma^\top a_g$ to $a_\mathrm{mid}\pm k_w^2W_g$, leaving its tangential component unchanged. Here $a_\mathrm{mid}=k_\Gamma(v_c\cos\epsilon)^2/(1-k_\Gamma e_y)-2k_w v_c\sin\epsilon-k_w^2e_y$, $k_w>0$. These strip inequalities result from applying a second-order response to $W_g\pm e_y$. Their role is confined to the guiding field.

### From the acceleration field to one avoidance trajectory

Resolve $a_g$ along the instantaneous course. The desired yaw rate is $r_g=(Je(\chi))^\top a_g/\max(v_c,v_g)$, clipped in magnitude to the smaller of $a_y/\max(v_c,v_g)$ and $v_c\sin(\arctan(l_r\tan\delta_{\max}/L))/l_r$. Let $v_s>0$ regularize the guide's tire angles and put $\alpha_{r,g}=\operatorname{atan2}(v_y-l_rr,\max(v_x,v_s))$. For longitudinal ratio $b'$ the rear tire force is $F_r(b')=F_{yr}(\alpha_{r,g},b')$; the requested body-frame front lateral force follows from yaw moment balance,

$$
F^*(b')=(I_z k_r(r_g-r)+l_rF_r(b'))/l_f .
$$

With yaw response $k_r>0$ and front force fraction $0<f_*<1$, reserve
$\eta=\min(f_*,\max(|F^*(0)|/K_f,|F_r(0)|/K_r))$.
The guide requests $b'=(m e(\chi)^\top a_g+F_d)/(K_f+K_r)$, clipped to $\pm\sqrt{1-\eta^2}$ and to the physical longitudinal input bounds. The rolling steering demand is the front-tire inverse for the body-frame lateral force $F^*(b')$. Its defining balance is $b'K_f\sin\delta+F_{yf}(\alpha_f,b')\cos\delta=F^*(b')$ on the admissible slip branch, with demand limited to the available force. At zero lateral capacity the tire slip demand is zero. Blend this rolling demand with the low-speed inverse $\delta_\ell=\arctan(L\tan\beta_g/l_r)$, $\beta_g=\arcsin(\operatorname{sat}_1(l_rr_g/v_c))$ with zero demand at rest, using weight $w$ of Section 1. Let $u_g$ be the resulting admissible actuator command. The integral curve $\dot x_g=f^\vartheta(x_g,u_g)$ starting at the nominal ego state supplies one initial avoidance trajectory. Its signed clearance may be negative.

## 4. Two-leg prediction and signed separation

### Duration, common input and the two connected trajectories

At observation time $t$, let $a_t>0$ be the avoidance duration, $b_t\ge0$ the connection duration and $T_t=a_t+b_t$. For each physical history $s$ in $\mathcal I(t)$ let $x_t^s$ be its ego state. A plan is one common input function $u:[0,T_t]\to\mathcal U$, where $\mathcal U$ contains the admissible steering angles and longitudinal force ratios. Require each resulting trajectory to stay in the physical model domain. Include the initial actuator state and any prescribed actuator dynamics or physical rate limits in admissibility. For every history the two legs satisfy

$$
x^s(\tau)=x_t^s+\int_0^\tau f^{\vartheta_s}(x^s(v),u(v))\,dv,
\qquad 0\le\tau\le T_t,\qquad
x^s(a_t^-)=x^s(a_t^+).
$$

The field in Section 3 supplies the initial trajectory for the first leg. The first leg determines the entry state of the connection. Both input functions are free subject to the same constraints and terminal membership. The planner sees only information available at $t$. Every predicted member follows the common input until the plan is revised using later observations.

To specify the encounter time scales, let $v$ be the ego speed, $a_p>0$ a preview deceleration, $T_r\ge0$ a response allowance, and $a_y>0$ the available lateral acceleration used in the guide. Let $D=2(r_E+r_T)$, where each $r$ bounds the body's radius about its dynamics reference point. Let $w$ be the ego lateral half-width plus the target half-width projected on the ego lateral axis, plus the desired clearance. Opposite constant lateral accelerations of magnitude $a_y$ over two equal durations cover $w$ with zero net lateral velocity, giving $t_L=2\sqrt{w/a_y}$. With target course $d$, let $c=d^\top(\dot p-\dot p_T)$. For $c>0$ the relative motion along $d$ gives $t_c=[d^\top(p_T-p)]_+/c$ and $t_P=D/c$. Take both times as infinite for $c\le0$. Use the admitted target direction on a stationary member. The physical duration requests of a member are
$$
 a_s=\max\!\left\{D/\max(v,V_{\rm ref}),
     \min\big(\max(v/a_p+T_r,t_L),t_c+t_L\big)\right\},\qquad
 b_s=\min(t_P,2\sqrt{D/a_y}).
$$
In target-free motion the respective requests are $v/a_p+T_r$ and zero. The two durations cover the respective suprema of these requests. During a continuing encounter the total horizon also covers the remaining duration of the inherited plan. Any extension can be assigned to the first leg while retaining the requested connection duration. These time scales select a preview. Existence of a trajectory to the terminal set is a separate condition.

### Collision-distance dual

Let $E=p+R(\psi)B_E$ have geometric center $c_E=p+R(\psi)o_E$, unit body axis $u_E=e(\psi)$ and half-extents $a,b$. For a target rectangle $R$, define
$$
 d_\pm(E,R)=\max_{\|n\|=1}
 \left[n^\top c_E-a|n^\top u_E|-b|n^\top Ju_E|-h_R(n)\right],
 \qquad h_R(n)=\max_{z\in R}n^\top z .
$$
For $R=\{z:H z\le f\}$, linear-program duality gives $h_R(n)=\min_{\lambda\ge0,\ H^\top\lambda=n}f^\top\lambda$. At any given time one normal serves every point of both footprints. For congruent rectangles the expression being maximized becomes
$$
 n^\top(c_E-c_T)-a\big(|n^\top u_E|+|n^\top u_T|\big)
                  -b\big(|n^\top Ju_E|+|n^\top Ju_T|\big).
$$
The centered difference body is the sum of four line segments along the two body frames. In general write $D=E-R=\{x-y:x\in E,y\in R\}$. If $0\notin D$, let $z=\Pi_D(0)$. The projection inequality $z^\top(x-z)\ge0$ for all $x\in D$ makes $z/\|z\|$ an attaining normal. No unit normal can give a larger lower projection since $z\in D$. If $0\in D$, the result is $-\min_{\|n\|=1}h_D(n)$, the negative radius of the largest origin-centered disk in $D$, hence minus the distance to the boundary. At contact it is zero. For ego vertices $x_a$ and $R=\{z:Hz\le f\}$ this gives the multiplier form
$$
 d_\pm(E,R)=\max_{\lambda\ge0,\;\|H^\top\lambda\|=1}
                 \min_a\big[\lambda^\top Hx_a-f^\top\lambda\big].
$$
The substitution $H^\top\lambda=n$ and strong support duality give equality for each normal. Unit length retains the penetration sign.

Since $h_{R_y\oplus\mathbb B(0,r_y)}(n)=h_{R_y}(n)+r_y$ for $\|n\|=1$, the two available upper bounds on target support give the clearance certificate at elapsed time $\tau$
$$
 \boxed{\quad g_s(\tau;u)=\max\{d_\pm(E,R_p),\ d_\pm(E,R_y)-r_y\}.\quad}
$$
Indeed subtracting the smaller upper support bound from the least ego projection and maximizing over $n$ commutes with choosing the better of the two bounds. Each term lower-bounds physical signed clearance for the member. The signed distance to the intersection of the enclosures can be larger. With only the prior available $g_s(\tau;u)=d_\pm(E,R_p)$. The normal may depend on time and the physical member. It certifies a geometric statement and never enters the actuator command as hidden information.

## 5. Terminal continuation and the predictive barrier

### Terminal set

Let $d_*>0$ be the planning margin. Set $g_s=+\infty$ during a released encounter. For an active encounter the common terminal command is $u_f=(0,0)$. Define terminal membership on the joint physical and information state, including the actuator state when applicable. Starting there, each still-active member must admit the common neutral command over a physical handoff duration $T_f>0$, respecting actuation and model domain, with $g_s(\tau;u_f)\ge d_*$ throughout its obligation. At the end it must be released or belong to the separation core below. When all members are released the common law may continue admissibly toward cruise trim. A release on one hidden member alone cannot change the command. The handoff is a continuation used to define membership. The two preceding legs remain free to maneuver.

At handoff take the prior target rectangle $T=R_p$ and freeze the footprint shapes and orientations. The ego translates with $v_f=v_xe(\psi)$ and the target translates at a fixed velocity in the sector
$\mathcal V=\{s e(\theta):l\le s\le u,\ |\operatorname{wrap}(\theta-\phi)|\le\delta_v\}$.
At elapsed handoff time $t$ the speed endpoints follow from $[V_0+tI_A]_+$ with the physical speed bound and absorption. The course interval encloses $\psi_0+\beta+\kappa s(A,t)$ for all admissible $(A,\beta)$, with turn restricted to $[-\bar\Omega t,\bar\Omega t]$. Its midpoint and half-width, capped at $\pi$ for the latter, define $\phi,\delta_v$. A known velocity gives a singleton. A stopped track gives zero velocity.

For a unit normal $n=e(\alpha)$, let $G(n)=\min_{y\in E}n^\top y-\max_{z\in T}n^\top z$. The maximal direction cosine in the sector is $p_c=\cos([\lvert\operatorname{wrap}(\alpha-\phi)\rvert-\delta_v]_+)$. Maximizing the remaining linear speed factor gives $\sigma_\mathcal V(n)=\max(lp_c,up_c)$. The core condition is

$$
\exists n,\quad \|n\|=1,\qquad G(n)\ge d_*,\quad G(n)>0,\quad n^\top v_f\ge\sigma_\mathcal V(n).
$$

**Separation in the terminal model.** For every fixed $v_T\in\mathcal V$ the same normal has gap $G(n)+t' n^\top(v_f-v_T)\ge G(n)$ for all $t'\ge0$. This proves clearance for that member throughout the frozen continuation. Each member may have its own normal under the common command. Two stationary separated bodies satisfy the motion inequality with equality. Neither finite-time exit nor positive ego speed is required.

Formally, let $\mathcal K$ denote the released states and the members satisfying the core inequalities. Let $\mathcal R_{u_f}(v,\mathcal I)$ contain all histories reached after elapsed time $v$ under the common handoff law, with their conditional enclosures. The terminal information family is
$$
 Z_f=\left\{\mathcal I\ne\varnothing:
 \begin{array}{l}
 u_f\text{ is jointly admissible on }[0,T_f],\\
 g_s(v;u_f)\ge d_*\text{ for all obligated histories and }0\le v\le T_f,\\
 \mathcal R_{u_f}(T_f,\mathcal I)\subseteq\mathcal K
 \end{array}\right\}.
$$
Here the handoff law uses the released continuation only upon observable release. Feasibility at the end of the connection requires $Z_f$ membership for the possible terminal information states, with a common admissible extension of the plan. Removing histories preserves a terminal witness whenever the retained histories can use its original support bounds and common input.

**Physical handoff assumption.** On the nonlinear physical and observation system, assume the terminal continuation law is realizable from the available information. From any admitted terminal information state it maintains clearance $d_*$, actuator admissibility and terminal membership (or admissible release) for every elapsed duration and surviving history. Safe concatenation includes the actuator transition at the join. The opening-normal argument proves separation under frozen translation. Closure under tire dynamics, changing target heading and subsequent observations is the physical handoff assumption.

### Continuous predictive deficit and causal control

The predictive barrier uses the accumulated relaxation of a clearance constraint, following the PCBF construction of Huang, Wang, Margellos and Goulart. The functional and shift inequality below use elapsed physical time. [Predictive Control Barrier Functions](https://arxiv.org/abs/2502.08400)

Fix the information at $t$ and a common input $u$. On $0\le\tau\le T_t$ set
$$
 c_t(\tau;u)=\sup_{s\in\mathcal I(t)}[d_*-g_s(\tau;u)]_+,\qquad
 D_t(u)=\int_0^{T_t}c_t(\tau;u)\,d\tau ,
$$
where a released member contributes zero. The supremum ranges over every admissible continuation of each history, using the same persistent parameters as in the observations. Assume these bounded deficit functions are measurable. Write $\mathcal U_t$ for common inputs satisfying the dynamics, actuator and physical-domain conditions, strict $g_s>0$ for every obligated member and instant, and membership of the terminal information family for all possible continuations. The terminal continuation must use a common admissible input for indistinguishable histories. The separating normals may differ among them. The primary problem is
$$
 \mathcal V(t,\mathcal I(t))=
 \inf_{\substack{u\in\mathcal U_t,\;\xi:[0,T_t]\to[0,d_*]\\
                 \xi(\tau)\ge c_t(\tau;u)\ \forall\tau}}
                \int_0^{T_t}\xi(\tau)\,d\tau
 =\inf_{u\in\mathcal U_t}D_t(u).
$$
The equality follows by taking the pointwise smallest slack for each feasible input. The infimum of an empty set is $+\infty$. A positive slack spends part of the planning margin while the strict separation constraint still applies. The integral has units of distance times time. A deficit at one instant alone supplies no positive lower bound on this integral.

At admission the controller may choose any feasible primary minimizer when it exists. More generally it can carry the deficit of a feasible committed plan as its certificate. If it follows a prefix of duration $\delta$, the budget for a replacement is
$$
 L(t+\delta)=D_t(u)-\int_0^\delta c_t(v;u)\,dv .
$$
A replacement must be feasible with deficit at most $L(t+\delta)$. Its actual input depends on the information received and the retained plan. During the executed prefix every indistinguishable history receives the same actuator input. Section 6 derives the budget and proves existence of a continuation under the stated hypotheses.

### Nominal cruise and secondary preferences

For a nominal member let $\varepsilon(\tau)$ contain lateral displacement from the route frame, wrapped body-heading error, $v_x-V_{\rm ref}$, $v_y$ and $r$. For the tracking preference the route frame at $\tau$ is the frame at the projection of $p(t)+\tau V_{\rm ref}e(\psi(t))$ onto the given path. For a nonnegative emphasis $w$, $W\succeq0$ and an actuator smoothness weight $w_u\ge0$, the continuous tracking and tire-use preferences are
$$
 J_{\rm tr}=\tfrac12\int_0^{T_t}
 \big[w(\tau)\varepsilon^\top W\varepsilon+w_u\|\dot u\|^2\big]\,d\tau ,
 \qquad
 J_{\rm tire}=\tfrac12\int_0^{T_t}
 \left[b^2+b^4+\langle(\gamma^s)^2\rangle\right]\,d\tau ,
$$
$$
 \gamma^s=(C_r^s/K_r^s)\operatorname{atan2}
 (v_y^s-l_r r^s,\sqrt{(v_x^s)^2+v_s^2}).
$$
The derivative term describes a preference on absolutely continuous commands with square-integrable rate when used. The brackets denote a normalized design average over physical members for secondary performance. The safety constraints use every member. At equal primary deficit the secondary ordering favors lower tire use with no increase in tracking cost. Neither preference is a premise of the safety argument.

For a realizable constant-curvature cruise trim the course is tangent to the route, $\|v^\star\|=V_{\rm ref}$, $r^\star=k_\Gamma V_{\rm ref}$, and $\dot v^\star=\dot r^\star=0$ under $u^\star$. These equations determine the body heading offset $\psi^\star_{\rm rel}=-\operatorname{atan2}(v_y^\star,v_x^\star)$. Use the route command $G$ from Section 3 with the trim correction $u^\star-G(x^\star,u^\star)$ before actuator projection. At an admissible trim this restores exactly $u^\star$. Define the transverse error and nominal control Lyapunov function by
$$
 e_c=(e_y,\operatorname{wrap}(\psi-\theta-\psi^\star_{\rm rel}),
          v_x-v_x^\star,v_y-v_y^\star,r-r^\star)^\top,\qquad
 V_c=e_c^\top P_c e_c,\quad P_c=P_c^\top\succ0 .
$$
The first two derivatives are $v_c\sin(\chi-\theta)$ and $r-k_\Gamma\dot s$, and the last three follow from the body vector field in Section 1. Write their joint field as $f_c(e_c,u)$ and let $A_c=D_e f_c(0,u^\star)$, $B_c=D_u f_c(0,u^\star)$ on a smooth trim branch. Under a realizable local cruise feedback $u=u^\star+K_c e_c$, let $A_K=A_c+B_cK_c$. If $A_K^\top P_c+P_c A_K\preceq-2\alpha_c P_c$, $\alpha_c>0$, differentiation gives
$$
 \dot V_c=2e_c^\top P_c f_c(e_c,u^\star+K_c e_c)
 \le-2\alpha_c V_c+o(\|e_c\|^2).
$$
In a sufficiently small admissible neighborhood the remainder is bounded by $\alpha_c V_c$, yielding local exponential convergence there under this feedback. Equilibrium reproduction by the target-free guide and local CLF decrease under the certified feedback are distinct properties. Global return to cruise after every feasible avoidance maneuver requires additional attraction and reachability hypotheses.

## 6. Conditional recursive feasibility and physical clearance

**Information consistency.** Let $\mathscr H_t$ be the admissible physical histories consistent with all outputs up to $t$. After using $u$ for $\delta>0$, the new histories are exactly the extensions of histories in $\mathscr H_t$ satisfying the plant under that prefix and the outputs on $(t,t+\delta]$. The true history survives by the measurement and motion assumptions. Any continuation from a surviving history on the inherited input suffix was a possible continuation at $t$. For an outer information representation require this same containment on the suffix. Updating a point estimate of the root speed or heading alone does not establish it.

Physical containment and preservation of a *computed clearance bound* are separate conditions. In the old separating direction $n$ the upper bound on target support is
$$
 H_t(n,\tau)=\min\{h_{R_p}(n),\,h_{R_y}(n)+r_y\},
$$
using available witnesses for that member and time. Assume physical flows are well posed on the admitted domain. At the same physical instant on the shifted plan require that an inherited normal can still certify at least its old gap. A sufficient condition is $H_{t+\delta}(n,v)\le H_t(n,v+\delta)$ in an old maximizing direction with the same physical ego trajectory. Retaining a valid old footprint bound for the surviving histories suffices. Two fresh enclosures can each contain the true target without satisfying this directional inequality.

**Proposition.** Suppose a feasible plan is committed at admission, all physical and observation histories are covered, and the above consistency and witness conditions hold. During a continuing encounter require that every new obligation on the overlapping interval previously existed, that $T_{t+\delta}\ge T_t-\delta$, and that the admissible trajectory family contains the retained suffix and the required terminal extension, including any change of leg junction. Assume physical terminal closure as in Section 5. A new encounter requires feasible admission.

For any $0<\delta\le T_t$ during which the chosen input is followed, write $T=T_t$, $T'=T_{t+\delta}$. On $0\le v\le T-\delta$ choose $u'(v)=u(v+\delta)$. Uniqueness of the physical evolution for a fixed history gives the same ego footprint at the same absolute time. The inherited normal and the smaller family of possible histories give
$$
 c_{t+\delta}(v;u')\le c_t(v+\delta;u)
$$
and preserve strict separation even when the margin was relaxed. The old endpoint is still in the terminal family. Append the admissible common terminal continuation for $T'-(T-\delta)$, which may be zero. Terminal closure supplies zero deficit throughout the extension and membership at its endpoint. Integration proves
$$
 D_{t+\delta}(u')\le\int_\delta^{T}c_t(v;u)\,dv
   =D_t(u)-\int_0^\delta c_t(v;u)\,dv .
$$
This provides a feasible successor and proves that the replacement budget is attainable. For a primary minimizer $u^*$ at $t$, taking the infimum at $t+\delta$ gives the same inequality with $\mathcal V(t+\delta,\mathcal I(t+\delta))$ on the left and $\mathcal V(t,\mathcal I(t))$ in place of $D_t(u^*)$ on the right. Existence of a feasible committed plan suffices for the stored-deficit inequality. A zero infimum by itself does not exhibit a zero-deficit plan.

On a closed-loop continuation whose executed prefixes cover the encounter, the inequalities telescope. Starting from an attained zero deficit, every executed prefix has zero integrated deficit. For each covered physical history, $[d_*-d_\pm(E_{\rm true},B_{T,\rm true})]_+$ is bounded by $c_t$ on its active interval. Both physical poses are continuous, including at absorption, so the physical distance is continuous. A strict loss of margin at any time in an encounter of positive duration would persist on a one-sided interval of positive length, contradicting the zero integral. The endpoints follow by continuity. An isolated instantaneous obligation must meet the desired margin directly for that same conclusion. This proves physical clearance $d_*$ on the stated domain. A feasible positive-deficit plan guarantees the separately imposed strict separation. Neither statement guarantees feasibility when a previously unseen target first enters range.
