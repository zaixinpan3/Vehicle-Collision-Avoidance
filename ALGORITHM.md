# Output-feedback collision avoidance with an NRMM observer and a predictive control barrier function

## 1. Vehicle motion and observations

The nominal task is to follow a prescribed path at cruise speed $V_{\mathrm{ref}}>0$. During an encounter the controller chooses steering $\delta$ and signed longitudinal force ratio $b$ to avoid a target with unknown motion. The relative position of the target is the output that drives its motion observer.

**Perception assumption.** Let $p$ and $p_T$ be the two reference positions. Whenever $\|p_T(t)-p(t)\|\le R_d$, the relative position is observed in real time. The observation may have a bounded position error, with zero error giving exact observation. Outside this range the nominal task is path and speed tracking.

Write $e(\theta)=(\cos\theta,\sin\theta)^\top$, $J=\left[\begin{smallmatrix}0&-1\\1&0\end{smallmatrix}\right]$, $R(\theta)=[e(\theta),Je(\theta)]$, $[a]_+=\max(a,0)$, and $\operatorname{sat}_c(a)=\max(-c,\min(c,a))$. Norms are Euclidean. Angle differences use the principal wrapped angle.

### Ego vehicle

The state is $x=(p,\psi,v_x,v_y,r)$, with inertial reference position $p$, body heading $\psi$, body velocity $v=(v_x,v_y)^\top$ and yaw rate $r$. Thus $\dot p=R(\psi)v$ and $\dot\psi=r$. Let $m,I_z$ be mass and yaw inertia, $g$ gravity, $l_f,l_r$ the axle distances, and $L=l_f+l_r$. For front and rear axles $i=f,r$, define

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

For blending speeds $0<v_\ell<v_h$, set $s=\min(1,\max(0,(v_x-v_\ell)/(v_h-v_\ell)))$ and $w=s^2(3-2s)$. The body derivative is $(1-w)f_\ell+wf_d$, with the pure branches evaluated on their domains. Let $F_h^\vartheta(x,u)$ denote the transition over time $h$ under constant command $u=(\delta,b)$ and physical parameters $\vartheta$, including the vehicle's rest transition. At rest the body velocity and yaw rate are zero, and neutral drive preserves the pose. The forward-motion domain requires $v_x\ge0$ and, where the rolling law contributes, $v_x\cos\delta+(v_y+l_fr)\sin\delta>0$.

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

Let $\chi$ contain the vehicle states, modes, persistent parameters and biases. The information set $\mathcal I(t)$ consists of all $\chi(t)$ reached from the initial set by admissible histories consistent with the applied input and the observed outputs up to $t$. The same history must satisfy the motion and observation equations. If the true initial condition, parameters and errors satisfy the assumptions, the true history witnesses $\chi(t)\in\mathcal I(t)$. Each member is the endpoint of at least one consistent physical history. All members receive the same control.

## 2. Nonlinear relative motion model observer

### Cartesian chain and inverse motion map

Write $q=\dot p_T$, $a=\ddot p_T$, $d=e(\psi_T+\beta)$ and $\Omega=\kappa V$. Then $q=Vd$, $\dot d=\Omega Jd$ and $a=Ad+V\Omega Jd$. Differentiating, using $\dot\Omega=\kappa A$, gives the closed chain

$$
\dot p_T=q,\qquad \dot q=a,\qquad
\dot a=\Phi(q,a)=-\Omega^2q+3A\Omega Jd,\qquad
A=d^\top a,\quad \Omega=(Jd)^\top a/V .
$$

In ego axes define $(\rho,q_B,a_B)=(R^\top(p_T-p),R^\top q,R^\top a)$ and $\nabla_r z=\dot z+rJz$. Then $\nabla_r\rho=q_B-v$, $\nabla_rq_B=a_B$, $\nabla_ra_B=\Phi(q_B,a_B)$. The inertial observation $y$ allows each retained ego member to use the inertial chain directly.

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

A discrepancy in the observed position therefore corrects all three Cartesian states. The moving target's velocity and acceleration follow from the chain without differentiating the measured position.

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

With the transit-time gain choice $k_v=k_p=k_\psi=\ln(20)/T_d$, the scalar homogeneous error factor after one transit time is $1/20$ for the velocity and position corrections. The body and position observers obey
$\dot{\hat v}=y_a-y_rJ\hat v+k_v(y_v-\hat v)$ and
$\dot{\hat p}_E=m_I+k_p(y_p-\hat p_E)$. The longitudinal inverse has derivative bounds $\sec b_E,\tan b_E$ with respect to speed and lateral velocity. Consequently $\|y_v-v\|\le\sec b_E(\epsilon_G+\rho_w)$. Skew symmetry of $J$ gives

$$
D^+\|\hat v-v\|\le-k_v\|\hat v-v\|+
\epsilon_a+\epsilon_r\bar V_E+k_v\sec b_E(\epsilon_G+\rho_w).
$$

For $s_\psi>0$ define $(c,s)=(y_v^\top m_I,(Jy_v)^\top m_I)/(\|y_v\|\|m_I\|+s_\psi^2)$. The parallel yaw law is
$\dot{\hat\psi}=y_r+k_\psi(s\cos\hat\psi-c\sin\hat\psi)$.
If $\mathcal V$ encloses the body velocity, the compatible angles are
$\{\theta:\exists v'\in\mathcal V,\ \|m_I-R(\theta)v'\|\le\epsilon_G\}$.
Intersect these angles with the prior angle set transported by the gyro integral and its error interval. The maximum wrapped distance to $\hat\psi$ on that intersection bounds its output error. At rest the innovation can vanish for every heading. In body axes the target error metric also receives the disturbances from $\hat v-v$ and the common gyro error. The former enters the first row only, contributing at most $\sqrt{P_{11}}\|\hat v-v\|$ to $D^+W$; the latter contributes at most $\epsilon_r\sup\|(\,J\rho,Jq_B/\omega,Ja_B/\omega^2\,)\|_M$ over the domain, where $\|z\|_M=\sqrt{z^\top Mz}$. In particular, for continuous bounded innovation disturbances write $\dot E\le-k_vE+d_E$ and $\dot W\le-\lambda W+\sqrt{P_{11}}E+d_T$, where $E$ bounds the body-velocity error and $d_E,d_T$ collect the respective disturbance terms above. Scalar comparison first gives $\limsup E\le d_E/k_v$, then $\limsup W\le(d_T+\sqrt{P_{11}}d_E/k_v)/\lambda$.

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

The target contributes a collision obligation while it is in perception range. In prediction, exit is established when every admissible reference position lies farther than $R_d$ from the ego reference point. The first exit node is checked before releasing subsequent nodes of that encounter. An observed re-entry starts a new encounter.

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

### From acceleration to the initial held inputs

Resolve $a_g$ along the instantaneous course. The desired yaw rate is $r_g=(Je(\chi))^\top a_g/\max(v_c,v_g)$, clipped in magnitude to the smaller of $a_y/\max(v_c,v_g)$ and $v_c\sin(\arctan(l_r\tan\delta_{\max}/L))/l_r$. Let $v_s>0$ regularize the guide's tire angles and put $\alpha_{r,g}=\operatorname{atan2}(v_y-l_rr,\max(v_x,v_s))$. For longitudinal ratio $b'$ the rear tire force is $F_r(b')=F_{yr}(\alpha_{r,g},b')$; the requested body-frame front lateral force follows from yaw moment balance,

$$
F^*(b')=(I_z k_r(r_g-r)+l_rF_r(b'))/l_f .
$$

With yaw response $k_r>0$ and front force fraction $0<f_*<1$, reserve
$\eta=\min(f_*,\max(|F^*(0)|/K_f,|F_r(0)|/K_r))$.
The guide requests $b'=(m e(\chi)^\top a_g+F_d)/(K_f+K_r)$, clipped to $\pm\sqrt{1-\eta^2}$ and to the admissible longitudinal input and increment. The rolling steering demand is the front-tire inverse for the body-frame lateral force $F^*(b')$. Its defining balance is $b'K_f\sin\delta+F_{yf}(\alpha_f,b')\cos\delta=F^*(b')$ on the admissible slip branch, with demand limited to the available force. At zero lateral capacity the tire slip demand is zero. Blend this rolling demand with the low-speed inverse $\delta_\ell=\arctan(L\tan\beta_g/l_r)$, $\beta_g=\arcsin(\operatorname{sat}_1(l_rr_g/v_c))$ with zero demand at rest, using weight $w$ of Section 1. Admissible actuator projection gives the held guide command. Recursively applying $F_h^\vartheta$ to these commands gives **one** initial avoidance trajectory. Its clearance may be negative.

## 4. Two-leg prediction and signed separation

### Duration and admissible command sequences

Let $h>0$ be the control period, $t_k=kh$, and write $\mathcal I_k=\mathcal I(t_k)$. At $t_k$ let $A_h$ and $B$ denote the avoidance and connection lengths in control stages, with $N_k=A_h+B$. The horizon may depend on the encounter. For recursive feasibility on continuation it must accommodate the unexecuted suffix, $N_{k+1}\ge N_k-1$. Initial reachability of the terminal set is a separate premise.

For each admitted initial state and persistent parameter hypothesis $s$, the two legs obey a single dynamics constraint
$$
x_0^s=x_k^s,\qquad x_{j+1}^s=F_h^{\vartheta_s}(x_j^s,u_j),
\qquad j=0,\ldots,N_k-1 .
$$
The initial trajectory for the first leg comes from the field in Section 3. The second leg starts at $x_{A_h}^s$ and ends in $Z_f$. Its starting state is determined by the first leg. The control inputs on **both** legs are decision variables. Each input $u_j$ is common to all hypotheses. Each satisfies the physical actuator bounds and any specified input-increment bounds relative to $u_{j-1}$, with $u_{-1}=u^-$ the preceding applied command. The admissible command family must contain the shifted plan and its terminal continuation for the shift argument in Section 5. Below write $N=N_k$ and $U_i=u_{i-1}$ for hold numbers $1\le i\le N$, with $U_0=u^-$.

### Collision-distance dual

Let $E=p+R(\psi)B_E$ have geometric center $c_E=p+R(\psi)o_E$, unit body axis $u_E=e(\psi)$ and half-extents $a,b$. For a target rectangle $R$, define
$$
 d_\pm(E,R)=\max_{\|n\|=1}
 \left[n^\top c_E-a|n^\top u_E|-b|n^\top Ju_E|-h_R(n)\right],
 \qquad h_R(n)=\max_{z\in R}n^\top z .
$$
For $R=\{z:H z\le f\}$, linear-program duality gives $h_R(n)=\min_{\lambda\ge0,\ H^\top\lambda=n}f^\top\lambda$. One normal serves every ego and target point at a node. For congruent rectangles the expression being maximized becomes
$$
 n^\top(c_E-c_T)-a\big(|n^\top u_E|+|n^\top u_T|\big)
                  -b\big(|n^\top Ju_E|+|n^\top Ju_T|\big).
$$
The centered difference body is the sum of four line segments along the two body frames. More generally the dual maximizes the least projection of $E-R$. Outside the difference body the origin's closest point supplies an attaining unit normal. Inside it the largest lower projection equals minus the distance to the boundary, with zero at contact.

Since $h_{R_y\oplus\mathbb B(0,r_y)}(n)=h_{R_y}(n)+r_y$ for $\|n\|=1$, the two available upper bounds on target support give the node certificate
$$
 \boxed{\quad d_j^s=\max\{d_\pm(E,R_p),\ d_\pm(E,R_y)-r_y\}.\quad}
$$
Indeed subtracting the smaller upper support bound from the least ego projection and maximizing over $n$ commutes with choosing the better of the two bounds. Each term lower-bounds physical signed clearance for the member. The signed distance to the intersection of the enclosures can be larger. With only the prior available $d_j^s=d_\pm(E,R_p)$. Different nodes and members can use different witnesses.

## 5. Terminal continuation and the predictive barrier

### Terminal set

Let $d_*>0$ be the prescribed clearance margin. Write $d_j^s$ for the signed gap certified by the available footprint enclosures for member $s$ at node $j$, with $d_j^s=+\infty$ after release. For an active encounter the common terminal input is $u_f=(0,0)$. Membership in $Z_f$ at the endpoint means that a released member has completed its collision obligation, and each still-active member admits one physical hold under $u_f$ with admissible input transition, model domain, positive separation and full clearance $d_*$ at both obligated nodes. After that hold it must be released or lie in the following core. A target-free continuation uses an admissible input toward cruise trim. The collision terminal condition imposes no cruise-speed or lane-error equality.

At handoff take the prior target rectangle $T=R_p$ and freeze the footprint shapes and orientations. The ego translates with $v_f=v_xe(\psi)$ and the target translates at a fixed velocity in the sector
$\mathcal V=\{s e(\theta):l\le s\le u,\ |\operatorname{wrap}(\theta-\phi)|\le\delta_v\}$.
At elapsed handoff time $t$ the speed endpoints follow from $[V_0+tI_A]_+$ with the physical speed bound and absorption. The course interval encloses $\psi_0+\beta+\kappa s(A,t)$ for all admissible $(A,\beta)$, with turn restricted to $[-\bar\Omega t,\bar\Omega t]$. Its midpoint and half-width, capped at $\pi$ for the latter, define $\phi,\delta_v$. A known velocity gives a singleton. A stopped track gives zero velocity.

For a unit normal $n=e(\alpha)$, let $G(n)=\min_{y\in E}n^\top y-\max_{z\in T}n^\top z$. The maximal direction cosine in the sector is $p_c=\cos([\lvert\operatorname{wrap}(\alpha-\phi)\rvert-\delta_v]_+)$. Maximizing the remaining linear speed factor gives $\sigma_\mathcal V(n)=\max(lp_c,up_c)$. The core condition is

$$
\exists n,\quad \|n\|=1,\qquad G(n)\ge d_*,\quad G(n)>0,\quad n^\top v_f\ge\sigma_\mathcal V(n).
$$

**Separation in the terminal model.** For every fixed $v_T\in\mathcal V$ the same normal has gap $G(n)+t' n^\top(v_f-v_T)\ge G(n)$ for all $t'\ge0$. This proves clearance for that member throughout the frozen continuation. Each member may have its own normal under the common command. Two stationary separated bodies satisfy the motion inequality with equality. Neither finite-time exit nor positive ego speed is required.

**Physical handoff assumption.** For the nonlinear physical and observation model, suppose the common terminal law admits a safe next hold and the resulting conditioned state retains terminal membership or an admissible release continuation. This is the closure assumption used below. The frozen-translation calculation proves the displayed result for its own model; future tire, target-turn and measurement dynamics enter the physical statement through the handoff assumption.

### Primary safety objective and secondary preference

For member $s$, the signed gap $d_i^s$ is the observation and motion certificate of Section 4.

For each stage $0\le j<N$, define
$C_j(U)=\sup_s\max_{i\in\{j,j+1\}}(d_*-d_i^s)$.
The two endpoints ensure that the root, the common leg junction and the terminal node are included. For an information state $\mathcal I$ write $\mathcal U_N(\mathcal I)$ for the common command sequences satisfying the actuator and physical-domain conditions on both legs and the hard terminal condition on every member. The predictive control barrier value on this domain is

$$
V_N(\mathcal I)=\inf_{U\in\mathcal U_N(\mathcal I),\,\xi\ge0}
\left\{\sum_{j=0}^{N-1}\xi_j:\ C_j(U)\le\xi_j\le d_*,\
d_i^s>0\ \text{at every obligated node}\right\}.
$$

An empty feasible set has value $+\infty$. For fixed feasible $U$, the least compatible slack is $\xi_j(U)=[C_j(U)]_+$. This follows separately for every stage since the objective increases with each slack. Write $D(U)=\sum_j\xi_j(U)$. A positive slack consumes part of the planning margin. The condition $d_i^s>0$ excludes contact even at $\xi_j=d_*$. The immutable root lower-bounds $D(U)$ by $[d_*-\inf_s d_0^s]_+$. An admissible plan attaining that bound is primary optimal.

On continuation the committed sequence must also satisfy $D(U)\le L_k$, where $L_k$ is the preceding plan's remaining deficit after its executed stage. Admission starts with $L_k=+\infty$. Only the first hold of a feasible plan is applied; the complete remaining sequence is retained. Thus the actual control is causal even though predictions from different members are compared in defining $C_j$. The domain of the control law consists of nonempty information sets admitting such a plan.

For the nominal member let $\varepsilon_j$ stack lateral displacement from the reference frame, wrapped body-heading error, $v_x-V_\mathrm{ref}$, $v_y$ and $r$. The reference frame at stage $j$ is the route frame at the projection of $p_k+jhV_\mathrm{ref}e(\psi_k)$. With nonnegative tracking weights $w_j$, $W\succeq0$ and $w_u\ge0$, the tracking preference is

$$
J_\mathrm{tr}=\tfrac h2\sum_{j=1}^N w_j\varepsilon_j^\top W\varepsilon_j
+\tfrac{w_u}2\sum_{j=1}^N\|U_j-U_{j-1}\|^2 .
$$

Let $\langle\cdot\rangle$ denote the normalized mean over the physical members used in the secondary preference. Define $\gamma_j^s=(C_r^s/K_r^s)\operatorname{atan2}(v_{y,j}^s-l_r r_j^s,\sqrt{(v_{x,j}^s)^2+v_s^2})$ at the end of hold $j$. With longitudinal ratio $b_j$ on that hold, set

$$
J_\mathrm{tire}=\tfrac h2\sum_{j=1}^N
  \left(b_j^2+b_j^4+\big\langle(\gamma_j^s)^2\big\rangle\right).
$$

This measures longitudinal force use and rear slip relative to the tire force scale. For equal primary deficit, the secondary ordering favors a strict decrease in $J_\mathrm{tire}$ with no increase in $J_\mathrm{tr}$.

For a realizable constant-curvature cruise trim, $v_c=V_\mathrm{ref}$, course is tangent to the route, $r=k_\Gamma V_\mathrm{ref}$ and the body derivatives vanish. Let $u^\star,x^\star$ satisfy these balance equations. The target-free guide adds the constant offset $u^\star-G(x^\star,u^\star)$ to the raw route command $G(x,u^-)$ before actuator projection. At admissible trim this gives $u^\star$; on a straight road the offset is zero. For transverse trim error $e=(e_y,\operatorname{wrap}(\psi-\theta-\psi^\star_\mathrm{rel}),v_x-v_x^\star,v_y-v_y^\star,r-r^\star)$ one may evaluate the nominal CLF $V_\mathrm{clf}=e^\top P_c e$, $P_c\succ0$, with $\dot V_\mathrm{clf}=2e^\top P_c\dot e$, using $\dot\theta=k_\Gamma\dot s$. Here $\psi^\star_\mathrm{rel}$ is the equilibrium body-heading offset from the tangent. For the one-hold transverse linearization $e^+=A_ce+B_c\delta u$ and certified local feedback $\delta u=K_c e$, a metric with $(A_c+B_cK_c)^\top P_c(A_c+B_cK_c)\preceq\rho_cP_c$, $0<\rho_c<1$, gives $V_\mathrm{clf}(e^+)\le\rho_c V_\mathrm{clf}(e)$ on that linear model. The primary problem above imposes no global decrease condition on this CLF. The trim identity establishes equilibrium tracking; asymptotic return to the route would require a separate stability argument.

### Conditional recursive feasibility

**Proposition.** Suppose an encounter begins with a feasible common plan. Assume the dynamics, margin and control period stay fixed and the physical state and footprint satisfy the stated enclosures. Along the unexecuted suffix, every history consistent with the new observations must extend an old admissible history. At each overlapping prediction time the new information must preserve the inherited directional support certificate, and every continuing collision obligation must already have been present in the prediction. Assume terminal closure under the common continuation law and that the new horizon and admissible command family contain the resulting shift and any required extension. A newly encountered target requires feasible admission.

Let $U_k$ be the committed plan and $\xi_{j|k}$ its actual stage deficits. After the hold $[t_k,t_{k+1})$, use the updated information set $\mathcal I(t_{k+1})$. The old prediction node $t_k+(j+1)h$ coincides with the new node $t_{k+1}+jh$. On each surviving path the inherited input produces the same ego footprint at that time. For the two-certificate model, the minimum of the available support upper bounds at the new node must be no larger in the inherited witness direction. It suffices that one new valid enclosure be contained in the enclosure attaining the old certificate. The signed separation bound then persists. The worst endpoint deficit at new stage $j$ is consequently at most $\xi_{j+1|k}$. If $N_{k+1}=N_k-1$, the endpoint already belongs to $Z_f$. Otherwise append $N_{k+1}-(N_k-1)$ holds of the common continuation. Closure inductively supplies their admissibility, terminal membership and zero collision slack. Hence a feasible successor exists, and

$$
D(U^\mathrm{shift}_{k+1})\le D(U_k)-\xi_{0|k},\qquad
L_{k+1}=D(U_k)-\xi_{0|k}.
$$

For an attained primary minimizer this yields $V_{N_{k+1}}(\mathcal I_{k+1})\le V_{N_k}(\mathcal I_k)-\xi^*_{0|k}$. For any feasible committed plan, the budget $L_{k+1}$ gives the same decrease inequality for the committed deficits. Starting from zero, nonnegativity forces every subsequent deficit to remain zero. In particular the replacement may choose a different separating direction. The shift proof uses existence of the inherited witnesses and membership of any replacement in the same feasible set. Under coverage, the actual vehicle then has clearance at least $d_*$ at obligated control nodes. A positive feasible deficit still requires strictly positive node clearance.

The containment premise concerns the joint physical and information evolution. Replacing the root speed, heading or motion mode with a new point estimate can violate it, even when $A,\beta$ are constant on the true trajectory. Thus persistence of the physical parameters alone does not imply consistency of successive prediction sets. Separation between control boundaries and feasibility on admission of a previously unseen target each require an additional argument.
