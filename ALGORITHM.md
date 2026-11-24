# Output-feedback collision avoidance with an NRMM observer and a predictive control barrier function

## 1. Vehicle motion and observations

A control decision is made at $t_k=kh$. The command $u_k=(\delta_k,b_k)$ is held on $[t_k,t_{k+1})$. Steering $\delta$ is positive to the left and the signed longitudinal force ratio $b$ is positive in traction. A sensor block contains the readings during one hold, including its right endpoint. Denote the preceding command by $u^-$. Write $e(\theta)=(\cos\theta,\sin\theta)^\top$, $J=\left[\begin{smallmatrix}0&-1\\1&0\end{smallmatrix}\right]$, $R(\theta)=[e(\theta),Je(\theta)]$, $[a]_+=\max(a,0)$, and $\operatorname{sat}_c(a)=\max(-c,\min(c,a))$. Norms are Euclidean. Angle differences use the principal wrapped angle.

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

For blending speeds $0<v_\ell<v_h$, set $s=\min(1,\max(0,(v_x-v_\ell)/(v_h-v_\ell)))$ and $w=s^2(3-2s)$. The body derivative is $(1-w)f_\ell+wf_d$, with the pure branches evaluated on their domains. At a control boundary, braking or neutral drive with $v_x< v_b/5$ sets $(v_x,v_y,r)$ to zero and preserves the integrated pose. Let $F_h^\vartheta(x,u)$ denote this held motion and boundary reset for parameters $\vartheta$. Intermediate sensor instants do not reset the vehicle. The forward-motion domain requires $v_x\ge0$ and, where the rolling law contributes, $v_x\cos\delta+(v_y+l_fr)\sin\delta>0$. Both sides of an input switch are subject to the model domain wherever the command is evaluated.

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

### Joint observation and conditioning

Let $\rho=R(\psi)^\top(p_T-p)$, and let $\eta$ collect the persistent sensor biases. For the command that generated the reading the sensors obey

$$
y_p=p+\eta_p+n_p,\quad y_r=r+\eta_r+n_r,\quad
y_a=\dot v+rJv+\eta_a+n_a,\quad
y_\rho=\rho+\eta_\rho+n_\rho\quad\text{on a return}.
$$

The acceleration is inertial acceleration resolved in body axes. Its boundary value uses $u^-$. The residuals have declared bounds $\epsilon_p,\epsilon_r,\epsilon_a,\epsilon_\rho$ in their respective norms. A return may also supply relative body heading. Its use as an exact angle requires that measurement contract. Detection is restricted to the disk of radius $R_d$ about the ego reference point. Dropout is possible within the disk.

Let $\chi$ contain the physical states, modes, persistent parameters and biases. Write $(\chi',Y)\in\mathcal R_h(\chi,u)$ if the **same** admissible history produces both the next state and the entire observed block $Y$. Exact information conditioning is

$$
\mathcal I^+=\{\chi':\exists\,\chi\in\mathcal I,\;(\chi',Y)\in\mathcal R_h(\chi,u)\}.
$$

Initial containment implies successor containment because the realized history is a witness. The controller carries a finite collection of ego members. Each propagates its own $x,\vartheta,\eta$ under the applied command and survives only if the entire position, gyro and accelerometer block is consistent. Propagation to the last sensor reading includes the control-boundary rest reset before conditioning on that reading. Each surviving member carries its own target observation history. For a positive radar return set $\tilde\rho=y_\rho-\eta_\rho$ and $\rho_m=\Pi_{\|\,\cdot\,\|\le R_d}\tilde\rho$. Consistency requires $\|\tilde\rho-\rho_m\|\le\epsilon_\rho$. Its inertial target measurement is $y=p+R(\psi)\rho_m$; projection onto the disk containing the true relative position preserves the error bound. Below, $r_i$ is the valid position measurement radius at time $t_i$ conditional on that member.

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

At a detected reading reset the position predictor $z^+=y$. Over the following sensor interval the observer satisfies

$$
\dot{\hat p}=\hat q+L_1 e,\quad
\dot{\hat q}=\hat a+L_2 e,\quad
\dot{\hat a}=\Phi_e(\hat q,\hat a)+L_3e,\quad
\dot z=\hat q .
$$

Here $e=z-\hat p$ following a detection and $e=0$ following a missed reading. At a missed reading the coast predictor starts at $\hat p$. Write $\hat V=\|\hat q\|$. At positive speed the published values are

$$
\hat d=\hat q/\hat V,\quad
\hat A=\operatorname{sat}_{\bar A}(\hat d^\top\Pi_{\bar a}\hat a),\quad
\hat\Omega=\operatorname{sat}_{\bar\Omega}((J\hat d)^\top\Pi_{\bar a}\hat a/\hat V),\quad
\hat\beta=\arcsin\!\left(\operatorname{sat}_{\sin\bar\beta}(l_T\hat\Omega/\hat V)\right).
$$

A resolved moving course yields body heading $\arg\hat q-\hat\beta$. A measured heading takes precedence. With unresolved displacement or active low-speed regularization the track retains its resolved heading. At exact rest the motion outputs vanish and position alone leaves body orientation unobservable.

### Acquisition

Let $(t_i,y_i,r_i)$ be distinct returns in the retained window, from $i=1$ to $n$, $T=t_n-t_1$. Motion initialization requires at least five returns (the window holds up to twenty) and $\|y_n-y_1\|>2\epsilon_\rho$. Put $\tau_i=(t_i-t_n)/T$ and fit $y_i=c_0+\tau_i c_1+\tfrac12\tau_i^2c_2$ in squared Euclidean distance. Set $\hat a=\Pi_{\bar a}(c_2/T^2)$ and refit the constant and linear terms with this acceleration fixed. The intercept and terminal slope give $\hat p,\hat q$ at $t_n$. If all returns lie within $\epsilon_\rho$ of their mean and body heading is observed, a rest-compatible initialization is $(\bar y,0,0)$. A single return supplies position only. Loss of the track beyond its declared coast duration initiates reacquisition.

Let $c=(y_n-y_1)/T$ and $a_*$ bound the Cartesian acceleration throughout the window. Integrating $\|q(t_n)-q(t)\|\le a_*(t_n-t)$ bounds the difference between endpoint and mean velocity by $a_*T/2$. Initial radii for $(\hat p,\hat q,\hat a)$ are therefore

$$
E_p=\|\hat p-y_n\|+r_n,\quad
E_q=\|\hat q-c\|+(r_1+r_n)/T+a_*T/2,\quad E_a=\|\hat a\|+a_* .
$$

At later returns each secant ending at the current return gives the same velocity inequality. Its intersection with the propagated bound retains every compatible velocity. If $\|\hat q\|-E_q>a_*T$, the target was moving throughout. For midpoint $m=(t_n+t_1)/2$, let $w_i$ be the weights that extract $c_2$ from the quadratic fit. The fit reproduces all quadratics, so Taylor expansion around $m$ and nonexpansiveness of acceleration projection also give

$$
\|\hat a-a(t_n)\|\le
 T^{-2}\sum_i |w_i|\big(r_i+\bar j |t_i-m|^3/6\big)+\bar j T/2 .
$$

The two upper bounds on the same error permit taking the smaller radius. A moving-arc jerk bound is $\bar j=\sqrt{V_u^2\omega_u^4+9\bar A^2\omega_u^2}$ for speed and yaw-rate upper bounds $V_u,\omega_u$ valid on the interval, since the two components of $\Phi$ are orthogonal.

### ISS certificate and sampled measurement errors

Write $L_i=\ell_i\omega^i$, $\omega>0$. For estimate-minus-truth errors form $Z=(e_p,e_q/\omega,e_a/\omega^2)$ and the metric $W=\sqrt{Z^\top MZ}$, $M=P\otimes I_2$, $P=P^\top\succ0$. Let

$$
A_\ell=\begin{bmatrix}-\ell_1&1&0\\-\ell_2&0&1\\-\ell_3&0&0\end{bmatrix},\quad
F(B,C)=\omega(A_\ell\otimes I_2)+(e_3\otimes I_2)[\,0,\ B/\omega,\ C\,] .
$$

Here $e_i$ in matrix expressions denotes a Cartesian basis vector of $\mathbb R^3$. A gain certificate requires some $\lambda>0$ such that
$F^\top M+MF\preceq-2\lambda M$ for every $(B,C)$ in a convex enclosure of the joint Jacobian $(D_q\Phi_e,D_a\Phi_e)$, including limiting branches. Integration along the segment between estimate and truth puts their finite field difference in this family. The certificate therefore bounds the actual nonlinear error. A common rotation of the three planar coordinates adds a skew transport term with zero metric derivative.

The gains are fixed by an operating-domain design. Let $T_d=R_d/(\bar V_E+\bar V)$, with $\bar V_E$ the ego speed bound used for observer design. Here and throughout, $\otimes$ denotes the Kronecker product and $|P|$ the entrywise absolute value. The target decay floor is $1/T_d$. For the unit-bandwidth shape write $A_L=A_0-L e_1^\top$, where $A_0$ has ones on its first superdiagonal and zeros elsewhere. The noise-shape preference minimizes $L^\top P L$ subject to the joint Jacobian dissipation inequality, $P\succ0$, and $A_L^\top P+PA_L+e_2e_2^\top\preceq0$. The last inequality bounds the velocity-output Gramian of the linear chain, explaining the preference for small measurement-noise amplification. Normalize $L_i$ by the $i$th power of the slowest decay rate of $A_L$ to obtain $\ell_i$. For the scaled chain the preference is a certified bandwidth with small largest ultimate component radius relative to its domain scale $(R_d,\bar V,\bar a)$, using a metric satisfying the same decay floor. This preference concerns the observer gains; its noise-shape objective is distinct from the deterministic containment inequality.

On a corrected inertial interval let $D_z\ge\|z-p_T\|$. The error injection contains the **same** predictor error in all three rows. Differentiation of $W^2$ and Cauchy–Schwarz in $M$ give

$$
D^+W\le-\lambda W+\omega\sqrt{\ell^\top P\ell}\,D_z+
              \sqrt{P_{33}}\,\bar j_e/\omega^2 ,
\qquad D^+\|z-p_T\|\le c_2 W,\qquad
c_i=\omega^{i-1}\sqrt{(P^{-1})_{ii}} .
$$

The coordinate inequalities $(\|e_p\|,\|e_q\|,\|e_a\|)_i\le c_iW$ follow by duality of the metric norm. Reset $D_z$ to the measurement radius at a return. With continuous position information of error at most $\epsilon$, integration gives $W(t)\le e^{-\lambda t}W(0)+(1-e^{-\lambda t})(\omega\sqrt{\ell^\top P\ell}\,\epsilon+\sqrt{P_{33}}\,\bar j_e/\omega^2)/\lambda$. For sampled information propagate the two coupled upper inequalities as equalities between resets. On a dropout use the component comparison instead,

$$
\dot E_p=E_q,\qquad \dot E_q=E_a,\qquad
\dot E_a=L_qE_q+L_aE_a+\bar j_e .
$$

Both comparison matrices have nonnegative off-diagonals. For either comparison $\dot b=Hb+f$ the upper solution is $e^{H\Delta}b(0)+\int_0^\Delta e^{H(\Delta-t)}f(t)\,dt$ because $e^{Ht}$ is entrywise nonnegative for $t\ge0$. A component box gives the metric radius $\sqrt{b^\top |P| b}$ with $b=(E_p,E_q/\omega,E_a/\omega^2)^\top$. Independent position, velocity and acceleration bounds may be intersected. In particular, on a smooth interval of length $\Delta$, writing $\delta\hat p,\delta\hat q,\delta\hat a$ for observer increments, integration of the physical chain gives the additional end radii

$$
\begin{aligned}
E'_p&=E_p+\|\delta\hat p-\Delta\hat q\|+\Delta E_q+a_*\Delta^2/2,\\
E'_q&=E_q+\|\delta\hat q-\Delta\hat a\|+\Delta E_a+\bar j\Delta^2/2,\\
E'_a&=E_a+\|\delta\hat a\|+\bar j\Delta .
\end{aligned}
$$

Unprimed estimates in these expressions are the initial values. If $\|\hat q\|\le E_q+\bar A\Delta$, absorption inside the interval is possible. Enclose the union with the stopped endpoint, whose velocity and acceleration are zero and whose position has moved at most $\bar A\Delta^2$ from its initial position. Smooth jerk bounds apply only to the moving part. Uniform-in-time sampled bounds require sufficient recurring measurement information; the continuous dissipation inequality alone supplies no contraction during arbitrary dropout.

### Ego observer in the body-frame formulation

The body-frame chain also admits a position/IMU ego observer when ego velocity must be reconstructed. The ego input to the controller's inertial target chain above is the synchronized pose of each retained member. This distinction specifies which uncertainty enters each error equation.

For position measurements spaced by $\Delta$, let $\gamma=\exp(-\omega_f\Delta)$, $\alpha_f^p=1-\gamma^2$, $\beta_f^p=(1-\gamma)^2/\Delta$. Correct the position/velocity pair $(p_f,m_I)$ by adding $(\alpha_f^p,\beta_f^p)(y_p-p_f^-)$ to its prediction, after bias correction. With inertial acceleration estimate $a_I=R(\hat\psi)y_a$, its next prediction is $(p_f+\Delta m_I+\Delta^2a_I/2,m_I+\Delta a_I)$. In each axis the homogeneous posterior error map is $(I-KC)F$, with $K=(\alpha_f^p,\beta_f^p)^\top$, $C=(1,0)$ and $F=\left[\begin{smallmatrix}1&\Delta\\0&1\end{smallmatrix}\right]$. Its trace $2\gamma$ and determinant $\gamma^2$ give a repeated stable pole. A bound $\epsilon_G$ on the velocity error follows by propagating the initial error set through these maps, adding $K$ times the position noise ball on correction and the bounded acceleration integrals on prediction. The secant inequality above gives a second enclosure using an ego acceleration bound. All following ego measurements are understood to be bias corrected.

On the forward cone $v_x\ge\|v\|\cos b_E$, $0\le b_E<\pi/2$, write $M_v=\|m_I\|$. A lateral pseudo-observation $w_m$ gives
$y_v=(\sqrt{\max\{M_v^2-w_m^2,M_v^2\cos^2 b_E\}},w_m)^\top$.
Under $|r-v_y/l_r|\le d_\mathrm{kin}$ the kinematic choice $w_m=l_r y_r$ has radius $\rho_w=l_r(\epsilon_r+d_\mathrm{kin})$. On a valid rolling branch the shared force model also supplies an inverse observation. For each possible $w=v_y$, use $v_x=\sqrt{V_E^2-w^2}$ and form $G(w)=Y_f+F_{yr}$ over admissible $V_E\in[M_v-\epsilon_G,M_v+\epsilon_G]\cap[0,\infty)$, yaw rate, cone, tire parameters and held input. The feasible inverse contains those $w$ for which the force range meets $m[y_{a,y}-\epsilon_a,y_{a,y}+\epsilon_a]$. A containing interval $[w_-,w_+]$ yields $\rho_w=\max(|w_m-w_-|,|w_m-w_+|)$ about the nominal regularized force inverse $G_0(w_m)-\rho_0 w_m=m y_{a,y}$, with prescribed $\rho_0$ equal to the force resolution divided by $M_v\sin b_E$ when this is positive, and endpoint saturation on the nominal cone. The force resolution is $m\epsilon_a+\eta_\vartheta\sum_i\mu_{i,0}F_{zi}$, where $\eta_\vartheta$ bounds fractional tire-parameter error about nominal values $\mu_{i,0}$. A zero-width nominal cone gives $w_m=0$. An uninformative force measurement retains the cone bound.

With the transit-time gain choice $k_v=k_p=k_\psi=\ln(20)/T_d$, the scalar homogeneous error factor after one transit time is $1/20$ for the velocity and position corrections. The body and position observers obey
$\dot{\hat v}=y_a-y_rJ\hat v+k_v(y_v-\hat v)$ and
$\dot{\hat p}_E=m_I+k_p(z_p-\hat p_E)$, with $\dot z_p=m_I$ and $z_p^+=y_p$ at a position reading. The longitudinal inverse has derivative bounds $\sec b_E,\tan b_E$ with respect to speed and lateral velocity. Consequently $\|y_v-v\|\le\sec b_E(\epsilon_G+\rho_w)$. Skew symmetry of $J$ gives

$$
D^+\|\hat v-v\|\le-k_v\|\hat v-v\|+
\epsilon_a+\epsilon_r\bar V_E+k_v\sec b_E(\epsilon_G+\rho_w).
$$

For $s_\psi>0$ define $(c,s)=(y_v^\top m_I,(Jy_v)^\top m_I)/(\|y_v\|\|m_I\|+s_\psi^2)$. The parallel yaw law is
$\dot{\hat\psi}=y_r+k_\psi(s\cos\hat\psi-c\sin\hat\psi)$.
If $\mathcal V$ encloses the body velocity, the compatible angles are
$\{\theta:\exists v'\in\mathcal V,\ \|m_I-R(\theta)v'\|\le\epsilon_G\}$.
Intersect these angles with the prior angle set transported by the gyro integral and its error interval. The maximum wrapped distance to $\hat\psi$ on that intersection bounds its output error. At rest the innovation can vanish for every heading. In body axes the target error metric also receives the disturbances from $\hat v-v$ and the common gyro error. The former enters the first row only, contributing at most $\sqrt{P_{11}}\|\hat v-v\|$ to $D^+W$; the latter contributes at most $\epsilon_r\sup\|(\,J\rho,Jq_B/\omega,Ja_B/\omega^2\,)\|_M$ over the domain, where $\|z\|_M=\sqrt{z^\top Mz}$. In particular, for continuous bounded innovation disturbances write $\dot E\le-k_vE+d_E$ and $\dot W\le-\lambda W+\sqrt{P_{11}}E+d_T$, where $E$ bounds the body-velocity error and $d_E,d_T$ collect the respective disturbance terms above. Scalar comparison first gives $\limsup E\le d_E/k_v$, then $\limsup W\le(d_T+\sqrt{P_{11}}d_E/k_v)/\lambda$. Sampled predictor errors use the reset and dropout comparisons instead.

### Persistent parameter and position enclosures

If $\hat V>E_q$, tangency to the velocity ball bounds the course error by $\theta=\arcsin(E_q/\hat V)$. Changing a unit direction by angle $\theta$ changes its vector by at most $2\sin(\theta/2)$. Thus both components of acceleration along and normal to the estimated course have error at most $d_a=E_a+2\|\hat a\|\sin(\theta/2)$. With interval division by a positive interval denoting the hull of all endpoint quotients,

$$
I_A\leftarrow I_A\cap(\hat d^\top\hat a+[-d_a,d_a]),\quad
K=\frac{(J\hat d)^\top\hat a+[-d_a,d_a]}{[(\hat V-E_q)^2,(\hat V+E_q)^2]},\quad
I_\beta\leftarrow I_\beta\cap\arcsin\!\big(\operatorname{sat}_{1}(l_T K)\big).
$$

Here an update intersects the previous parameter intervals with the displayed sets. Start from $[-\bar A,\bar A]$ and $[-\bar\beta,\bar\beta]$. The curvature quotient follows from $(Jd)^\top a=\kappa V^2$. If the velocity ball includes zero these inverse operations supply no restriction. For any lower bound $s_*>0$ on the largest speed attained on the track, $|\sin\beta|\le l_T\bar\Omega/s_*$. Both $[\hat V-E_q]_+$ and $[\|y_n-y_1\|-r_n-r_1]_+/T$ are such bounds.

Two returns with chord length $c$, time span $T$, combined position radius $r_c$ and $\bar\Omega T/2<\pi/2$ enclose mean speed in
$[L_c,U_c]=[\, [c-r_c]_+/T,(c+r_c)/(T\cos(\bar\Omega T/2))\,]$.
The lower bound uses path length. The upper bound projects velocity onto the midpoint course, extending that course by continuity through a possible stop. If $L_c>\bar A T/2$, endpoint speed is positive, since $V$ is $\bar A$-Lipschitz. The entire window was moving and its mean speed equals its midpoint speed. For an earlier such interval $[L_0,U_0]$ at midpoint $m_0$, a later interval at $m_1$ therefore contributes

$$
I_A\ \leftarrow\ I_A\cap[(L_c-U_0)/(m_1-m_0),(U_c-L_0)/(m_1-m_0)].
$$

The first eligible chord supplies the reference, replaced once by the first full-window chord. Retain intersections for the same track. Empty intersections exclude the member.

Let $P_0$ denote the current world-axis box enclosing the target reference position. Over $\Delta$, translate its prior box by a displacement set contained in both the box $\Delta\hat q+[-b_p,b_p]^2$, $b_p=\Delta E_q+a_*\Delta^2/2$, and $[-s_{\max},s_{\max}]^2$. Here $s_{\max}$ is the upper travel from the previous upper speed and upper tangential acceleration, including a possible stop. Use the intersection of the two displacement boxes, then intersect the resulting position box with available measurement boxes and the $\hat p$ error box. On the rest-compatible branch retain the intersection of successive position boxes under zero displacement. Publish zero motion only while this branch remains compatible. Rest compatibility and a fixed root speed and heading are **conditioning choices** for the forecast. Universal coverage of all possible root speeds, headings and modes would require a separate containment premise.

### Predicted target occupancy and encounter scope

For a member let the forecast root be the midpoint of the applicable position box, with half-width vector $b_0$, selected heading and speed, and parameter rectangle $I_A\times I_\beta$. At time $t$ the entire family uses one geometric enclosure. Set
$S=[s(\inf I_A,t),s(\sup I_A,t)]$, $K=\sin(I_\beta)/l_T$, $Z=KS/2$ using interval products. Monotonicity of speed before absorption proves the travel bounds. Intersect with $S\le\bar Vt$, $|Z|\le\bar\Omega t/2$ and, if $0\notin K$, $S\le\bar\Omega t/\min_{\kappa\in K}|\kappa|$ where those domain bounds apply. A stopped root retains its pose.

Choose axes at $\theta_b=\psi_0+\inf Z+\sup Z$ and $\theta_b+\pi/2$. On either axis of angle $\alpha$, enclose the reference-point displacement by the interval product

$$
S\,\operatorname{sinc}(Z)\cos(\psi_0+I_\beta+Z-\alpha).
$$

Interval trigonometric images include their interior extrema. On $Z\subset[-\pi,\pi]$, the sinc range follows from evenness and monotonicity on $[0,\pi]$; otherwise $[-1,1]$ suffices. On long turning arcs excluding zero curvature the chord factor also belongs to $2\sin(Z)/K$. The axis displacement also belongs to
$[\sin(\psi_0+I_\beta+2Z-\alpha)-\sin(\psi_0+I_\beta-\alpha)]/K$ for $0\notin K$. Intersect applicable bounds. The two axis intervals, translated by the root, enclose the reference points. Add the ranges of the projections of the rotated target body vertices at headings $\psi_0+2Z$ and take coordinate extrema to obtain an oriented occupied rectangle $T_j$. At the root the half-extents gain $|R(\psi_0)^\top| b_0$ by the support of the translation box. For a moving forecast expand each body half-extent by $\|b_0\|$ before the vertex projection; this bounds displacement in every body direction. For a stationary body use the root expansion throughout.

Avoidance begins at a decision boundary with a return history, complete target states for all surviving members, and either a current detection or an in-range member. In-range means distance from ego position to the member's target position box at most $R_d$. Before admission the nominal cruise law applies. In prediction check the first exit node; release subsequent collision obligations once the reference-position enclosure is entirely outside range. Equivalently a sufficient exit test for the arc enclosure before adding root-position error is distance greater than $R_d+\|b_0\|$. New measurements determine actual release and re-entry. A missed return alone gives no range exclusion.

## 3. One initial trajectory from a local acceleration field

### Route field

The fixed route has position $p_\Gamma(s)$, unit tangent $e(\theta)$, left normal $n_\Gamma=Je(\theta)$ and curvature $k_\Gamma$. Use closest-point station on the designated path. Let $e_y=n_\Gamma^\top(p-p_\Gamma)$, $v_c=\|v\|$, $\chi=\psi+\operatorname{atan2}(v_y,v_x)$, $\epsilon=\operatorname{wrap}(\chi-\theta)$, and $V_\mathrm{ref}>0$ be the cruise speed. On the regular chart, differentiation gives $\dot e_y=v_c\sin\epsilon$ and $\dot s=v_c\cos\epsilon/(1-k_\Gamma e_y)$. With lookahead $D=\max(D_{\min},T_D V_\mathrm{ref})$, define

$$
\epsilon_d=-\arctan(e_y/D),\quad
\dot\epsilon_d=-D v_c\sin\epsilon/(D^2+e_y^2),\qquad
r_d=\frac{k_\Gamma v_c\cos\epsilon}{\max(1-k_\Gamma e_y,0.1)}
       +\dot\epsilon_d-k_\chi\operatorname{wrap}(\epsilon-\epsilon_d),
$$

$$
a_0=k_v^{g}(V_\mathrm{ref}-v_c)e(\chi)+v_c r_d Je(\chi).
$$

The first component restores speed; the second rotates the course toward the route. $D_{\min},T_D,k_\chi,k_v^{g}$ are positive guidance parameters.

### Obstacle field

Use the retained member closest in squared Euclidean state distance to the mean of all ego members for this single trajectory. Work in the axes $R_T$ of its forecast rectangle at the current preview node. Let $b=(b_1,b_2)>0$ be its half-extents, $o_T$ its body offset, $c=p_T+R_To_T$ its center, $\Omega=\kappa V$, and $\dot\Omega=\kappa A$. The point motion gives

$$
v_c^T=Vd+\Omega JR_To_T,\qquad a_c^T=Ad+V\Omega Jd+\dot\Omega JR_To_T-\Omega^2R_To_T,\quad d=e(\psi_T+\beta).
$$

For a moving occupancy enclosure its transverse growth contributes a center velocity and extent rate. Let $\Delta_h$ denote the backward difference per hold, $\gamma=\psi_T+\beta-\theta$, and $h_1,h_2$ the nominal target half-extents. With $[z]_{01}=\min(1,\max(0,z))$ and radial gain $k>0$ set
$\zeta=[(h_1|\sin\gamma|-h_2|\cos\gamma|)/h_2]_{01}[1+A/(k^2h_2)]_{01}$.
After the first node of a moving enclosure use $v_c^T\leftarrow(1-\zeta)v_c^T+\zeta\Delta_h c$ and $\dot b_g=\zeta\Delta_h b$. Otherwise $\dot b_g=0$ and the point center velocity applies.

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

All speeds and accelerations in these laws use SI units. The radial expression comes from $\ddot d_g=n^\top\ddot q_g+k_g\tau^2$ outside the rectangle and the desired radial response $\ddot g+2k\dot g+k^2g=0$. Changes in ego support and envelope shape make this a guidance law. The transport terms follow by differentiating rotating coordinates twice. Choose $a_y$ as a prescribed fraction of $\min_i\mu_i g_\mathrm{grav}$, where $g_\mathrm{grav}$ denotes gravitational acceleration. At the initial nominal target pose set $s_g=\min(\max(\|w_g\|,1\,\mathrm{m/s}),\sqrt{a_y\max(r_E,\|q_g\|)})$, with $r_E=\max_{y\in B_E}\|y\|$. The sign of the transverse contribution follows from the evolving $y_\perp$. There is no discrete passing-side variable.

After predicted exit use $a_g=a_0$. For a finite route-guide half-width $W_g$, limit the component $n_\Gamma^\top a_g$ to $a_\mathrm{mid}\pm k_w^2W_g$, leaving its tangential component unchanged. Here $a_\mathrm{mid}=k_\Gamma(v_c\cos\epsilon)^2/\max(1-k_\Gamma e_y,0.1)-2k_w v_c\sin\epsilon-k_w^2e_y$, $k_w>0$. These strip inequalities result from applying a second-order response to $W_g\pm e_y$. Their role is confined to the guiding field.

### From acceleration to the initial held inputs

Resolve $a_g$ along the instantaneous course. The desired yaw rate is $r_g=(Je(\chi))^\top a_g/\max(v_c,1\,\mathrm{m/s})$, clipped in magnitude to the smaller of $a_y/\max(v_c,1\,\mathrm{m/s})$ and $v_c\sin(\arctan(l_r\tan\delta_{\max}/L))/l_r$. Let $v_s>0$ regularize the guide's tire angles and put $\alpha_{r,g}=\operatorname{atan2}(v_y-l_rr,\max(v_x,v_s))$. For longitudinal ratio $b'$ the rear tire force is $F_r(b')=F_{yr}(\alpha_{r,g},b')$; the requested body-frame front lateral force follows from yaw moment balance,

$$
F^*(b')=(I_z k_r(r_g-r)+l_rF_r(b'))/l_f .
$$

With yaw response $k_r>0$ and front force fraction $0<f_*<1$, reserve
$\eta=\min(f_*,\max(|F^*(0)|/K_f,|F_r(0)|/K_r))$.
The guide requests $b'=(m e(\chi)^\top a_g+F_d)/(K_f+K_r)$, clipped to $\pm\sqrt{1-\eta^2}$ and to the admissible longitudinal input and increment. The rolling steering demand inverts the front force on the unsaturated branch, with limited slip. For wheel force $F$, the inverse is

$$
\alpha(F)=-\operatorname{sign}(F)\arctan\!\left[
  (3D_f/C_f)\{1-(1-\min(|F|/D_f,f_*))^{1/3}\}\right].
$$

The uncoupled steering demand is $\operatorname{atan2}(v_y+l_fr,\max(v_x,v_s))-\alpha(F^*(b'))$. The coupled steering correction targets the body-frame force balance $b'K_f\sin\delta+F_{yf}\cos\delta=F^*(b')$ within the same front slip limit. At zero lateral capacity the tire slip demand is zero. Blend this rolling demand with the low-speed inverse $\delta_\ell=\arctan(L\tan\beta_g/l_r)$, $\beta_g=\arcsin(\operatorname{sat}_1(l_rr_g/v_c))$ with zero demand at rest, using weight $w$ of Section 1. Admissible actuator projection gives the held guide command. Recursively applying $F_h^\vartheta$ to these commands gives **one** initial avoidance trajectory. Its clearance may be negative.

## 4. Two-leg prediction and signed separation

### Duration and admissible command sequences

Let $a_p>0$ be the preview deceleration, $T_s\ge0$ a response allowance, $\Delta_T>0$ the time quantization, and $C(t)=\lceil\Delta_T\lceil t/\Delta_T\rceil/h\rceil$. Without an active target a member requests avoidance time $T_a=\max(2h,v_c/a_p+T_s)$ and connection time $T_b=0$. For an active target, define the span $D_p=2(\|b_E\|+\|o_E\|+\|b_T\|+\|o_T\|)$ using the physical body half-extents and offsets. Let $d_T=e(\psi_T+\beta)$ and $c_p=d_T^\top(R(\psi)v-Vd_T)$. For $c_p>0$ use $t_c=[d_T^\top(p_T-p)]_+/c_p$, $t_p=D_p/c_p$; otherwise set both to infinity. With $d_*>0$ the planning clearance, let

$$
W=b_{E,2}+b_{T,1}|\sin(\psi_T-\psi)|+b_{T,2}|\cos(\psi_T-\psi)|+d_*,
\qquad t_L=2\sqrt{W/a_y}.
$$

Two opposite lateral accelerations of duration $t_L/2$ produce displacement $W$ from rest to rest. This gives the timing scale

$$
T_a=\max\left(2h,\frac{D_p}{\max(v_c,V_\mathrm{ref})},
              \min\{\max(v_c/a_p+T_s,t_L),t_c+t_L\}\right),\qquad
T_b=\min(t_p,2\sqrt{D_p/a_y}).
$$

Let $B=C(\max_s T_b^s)$, $N=\max(N_\mathrm{suffix},C(\max_s T_a^s)+B)$ and $A_h=N-B$. Here $s$ indexes retained members, $N_\mathrm{suffix}=N_\mathrm{old}-1$ on continuation, and it is zero on initial admission or newly activated collision obligations. These duration estimates define the two legs. Reachability of the terminal set is a separate feasibility condition.

The dynamics of both legs are $x_{j+1}^s=F_h^{\vartheta_s}(x_j^s,u_j)$, $0\le j<N$, from each member's measured root. The connection starts at the actual predicted endpoint $x_{A_h}^s$. There is one common input at each stage across the members. The connection's initial inputs move toward straight cruise trim $(0,F_d(V_\mathrm{ref})/(K_f+K_r))$ within input limits. Its admissible controls are free to change.

For clarity, let $U_i=u_{i-1}$ for hold numbers $1\le i\le N$ and $U_0=u^-$. Choose knot endpoints $0=c_0<\cdots<c_M=N$ including $A_h$. With $m=\min(A_h,M_{\max})$ avoidance knots and $d=\min(m-1,m_\mathrm{near})$, the first $d$ hold numbers are endpoints, followed by $d+\operatorname{round}(j(A_h-d)/(m-d))$, $j=1,\ldots,m-d$. For $B>0$ put $m_b=\min(M_{\max},\lceil B/\max_{c_i\le A_h}(c_i-c_{i-1})\rceil)$ and add $A_h+\operatorname{round}(jB/m_b)$, $1\le j\le m_b$. $M_{\max}$ and $m_\mathrm{near}$ prescribe the temporal resolution of the admissible commands.

On an encounter let $\bar U$ be the initial held sequence or the executable shifted sequence on continuation. With $\bar v_i=\bar U_{c_i}$, $v_0=\bar v_0=u^-$, and $\theta=j/(c_i-c_{i-1})$, the physical command family is

$$
U_{c_{i-1}+j}=\bar U_{c_{i-1}+j}
 +(1-\theta)(v_{i-1}-\bar v_{i-1})+\theta(v_i-\bar v_i),
\quad 1\le j\le c_i-c_{i-1}.
$$

Zero correction reproduces every initial held command. In target-free cruise use direct linear interpolation of knot commands. Every actual $U_i$ must lie in the physical input box and satisfy $|U_i-U_{i-1}|\le h\,\dot u_{\max}$ componentwise for declared finite increment bounds. The admissible retained plans include the full executable suffix independently of its interpolation on a new grid.

### Collision-distance dual

Let the ego body $B_E=o_E+[-b_E,b_E]$ have vertices $e_a$ and let $E=p+R(\psi)B_E$. At an active node let $T$ be the target rectangle, including its occupancy uncertainty. In axes $R_T$ about a reference point $p_T$, with half-extents $b_T$ and offset $o_T$, it is given by
$H(y-p_T)\le f$, where $H=[I;-I]R_T^\top$ (vertical stacking) and $f=(b_T+o_T,b_T-o_T)$ stacked. For any $n$, its support relative to $p_T$ is

$$
\max_{H z\le f}n^\top z=\min_{\lambda\ge0,\ H^\top\lambda=n} f^\top\lambda .
$$

The rectangle is nonempty and bounded, so linear-program duality attains this value. The minimum of a linear projection on $E$ is attained at a vertex. Eliminating the support for each direction gives the signed separation

$$
d_\pm(E,T)=
\max_{\lambda\ge0,\ \|H^\top\lambda\|=1}
\min_a\big\{\lambda^\top H(p+R(\psi)e_a-p_T)-f^\top\lambda\big\}.
$$

Indeed this is the largest lower projection of $E-T$ over all unit directions. When the origin is outside that difference polygon, its closest point supplies a unit normal attaining its distance. When the origin lies inside, the largest lower projection equals minus the least translation distance to the boundary; at contact it is zero. Thus overlap has negative value. One multiplier is shared across the ego vertices at a node. Different members and nodes may have different witnesses. A change of witness leaves the signed-distance function and the admissible physical constraint unchanged. The enclosing rectangle makes the gap a lower bound for the true footprint gap whenever the true footprint is covered.

## 5. Terminal continuation and the predictive barrier

### Terminal set

Write $d_j^s$ for the signed gap to the occupied rectangle for member $s$ at node $j$, with $d_j^s=+\infty$ after release. For an active encounter the common terminal input is $u_f=(0,0)$. Membership in $Z_f$ at the endpoint means that a released member has completed its collision obligation, and each still-active member admits one physical hold under $u_f$ with admissible input transition, model domain, positive separation and full clearance $d_*$ at both obligated nodes. After that hold it must be released or lie in the following core. A target-free continuation uses an admissible input toward cruise trim. The collision terminal condition imposes no cruise-speed or lane-error equality.

At handoff freeze the footprint shapes and orientations. The ego translates with $v_f=v_xe(\psi)$ and the target translates at a fixed velocity in the sector
$\mathcal V=\{s e(\theta):l\le s\le u,\ |\operatorname{wrap}(\theta-\phi)|\le\delta_v\}$.
At elapsed handoff time $t$ the speed endpoints follow from $[V_0+tI_A]_+$ with the physical speed bound and absorption. The course interval is $\psi_0+I_\beta+KS$, with turn restricted to $[-\bar\Omega t,\bar\Omega t]$. Its midpoint and half-width, capped at $\pi$ for the latter, define $\phi,\delta_v$. A known velocity gives a singleton. A stopped track gives zero velocity.

For a unit normal $n=e(\alpha)$, let $G(n)=\min_{y\in E}n^\top y-\max_{z\in T}n^\top z$. The maximal direction cosine in the sector is $p_c=\cos([\lvert\operatorname{wrap}(\alpha-\phi)\rvert-\delta_v]_+)$. Maximizing the remaining linear speed factor gives $\sigma_\mathcal V(n)=\max(lp_c,up_c)$. The core condition is

$$
\exists n,\quad \|n\|=1,\qquad G(n)\ge d_*,\quad G(n)>0,\quad n^\top v_f\ge\sigma_\mathcal V(n).
$$

**Separation in the terminal model.** For every fixed $v_T\in\mathcal V$ the same normal has gap $G(n)+t' n^\top(v_f-v_T)\ge G(n)$ for all $t'\ge0$. This proves clearance for that member throughout the frozen continuation. Each member may have its own normal under the common command. Two stationary separated bodies satisfy the motion inequality with equality. Neither finite-time exit nor positive ego speed is required.

**Physical handoff assumption.** For the nonlinear physical and observation model, suppose the common terminal law admits a safe next hold and the resulting conditioned state retains terminal membership or an admissible release continuation. This is the closure assumption used below. The frozen-translation calculation proves the displayed result for its own model; future tire, target-turn and measurement dynamics enter the physical statement through the handoff assumption.

### Primary safety objective and secondary preference

For each stage $0\le j<N$, define
$C_j(U)=\max_s\max_{i\in\{j,j+1\}}(d_*-d_i^s)$.
The two endpoints ensure that the root, the common leg junction and the terminal node are included. For an information state $\mathcal I$ write $\mathcal U_N(\mathcal I)$ for the common command sequences satisfying the actuator and physical-domain conditions on both legs and the hard terminal condition on every member. The predictive control barrier value on this domain is

$$
V_N(\mathcal I)=\inf_{U\in\mathcal U_N(\mathcal I),\,\xi\ge0}
\left\{\sum_{j=0}^{N-1}\xi_j:\ C_j(U)\le\xi_j\le d_*,\
d_i^s>0\ \text{at every obligated node}\right\}.
$$

An empty feasible set has value $+\infty$. For fixed feasible $U$, the least compatible slack is $\xi_j(U)=[C_j(U)]_+$. This follows separately for every stage since the objective increases with each slack. Write $D(U)=\sum_j\xi_j(U)$. A positive slack consumes part of the planning margin. Strict separation at dispatch excludes contact even at the endpoint $\xi_j=d_*$. The immutable root lower-bounds $D(U)$ by $[d_*-\min_s d_0^s]_+$. An admissible plan attaining that bound is primary optimal.

On continuation the committed sequence must also satisfy $D(U)\le L_k$, where $L_k$ is the preceding plan's remaining deficit after its executed stage. Admission starts with $L_k=+\infty$. Only the first hold of a feasible plan is applied; the complete remaining sequence is retained. Thus the actual control is causal even though predictions from different members are compared in defining $C_j$. If the posterior is empty or no admissible plan is available the law supplies no command.

For the representative member define the reference frames by projecting $p_k+c_i h V_\mathrm{ref}e(\psi_k)$ onto the fixed route. At each $c_i$ let $\varepsilon_i$ stack lateral displacement from that frame, wrapped body-heading error, $v_x-V_\mathrm{ref}$, $v_y$ and $r$. For $W\succeq0,w_u\ge0$, the tracking preference is

$$
J_\mathrm{tr}=\tfrac h2\sum_i(c_i-c_{i-1})\varepsilon_i^\top W\varepsilon_i
+\tfrac{w_u}2\sum_{j=1}^N\|U_j-U_{j-1}\|^2 .
$$

For $S_m$ members define $\gamma_j^s=(C_r^s/K_r^s)\operatorname{atan2}(v_{y,j}^s-l_r r_j^s,\sqrt{(v_{x,j}^s)^2+v_s^2})$ at each hold endpoint. With longitudinal ratio $b_j$ on hold $j$ (one-based), set

$$
J_\mathrm{tire}=\tfrac h2\sum_{j=1}^N
  \left(b_j^2+b_j^4+\tfrac1{S_m}\sum_s(\gamma_j^s)^2\right).
$$

This measures longitudinal force use and rear slip relative to the tire force scale. Among available plans of equal deficit an incumbent is replaced when $J_\mathrm{tire}$ decreases and $J_\mathrm{tr}$ does not increase, in real arithmetic. Neither preference can buy an increase in the primary deficit.

For a realizable constant-curvature cruise trim, $v_c=V_\mathrm{ref}$, course is tangent to the route, $r=k_\Gamma V_\mathrm{ref}$ and the body derivatives vanish. Let $u^\star,x^\star$ satisfy these balance equations. The target-free guide adds the constant offset $u^\star-G(x^\star,u^\star)$ to the raw route command $G(x,u^-)$ before actuator projection. At admissible trim this gives $u^\star$; on a straight road the offset is zero. For transverse trim error $e=(e_y,\operatorname{wrap}(\psi-\theta-\psi^\star_\mathrm{rel}),v_x-v_x^\star,v_y-v_y^\star,r-r^\star)$ one may evaluate the nominal CLF $V_\mathrm{clf}=e^\top P_c e$, $P_c\succ0$, with $\dot V_\mathrm{clf}=2e^\top P_c\dot e$, using $\dot\theta=k_\Gamma\dot s$. Here $\psi^\star_\mathrm{rel}$ is the equilibrium body-heading offset from the tangent. For a sampled transverse linearization $e^+=A_ce+B_c\delta u$ and certified local feedback $\delta u=K_c e$, a metric with $(A_c+B_cK_c)^\top P_c(A_c+B_cK_c)\preceq\rho_cP_c$, $0<\rho_c<1$, gives $V_\mathrm{clf}(e^+)\le\rho_c V_\mathrm{clf}(e)$ on that linear model. The primary problem above imposes no global decrease condition on this CLF. The trim identity establishes equilibrium tracking; asymptotic return to the route would require a separate stability argument.

### Conditional recursive feasibility

**Proposition.** Suppose that an encounter starts with a feasible common plan; the margin, dynamics and timing stay fixed; each realized successor and footprint is covered; and conditioning on the new block leaves a subset of the previously predicted successors **with compatible future uncertainty and exit obligations**. Suppose the physical handoff and target-free continuation assumptions hold, and the new horizon contains the whole unexecuted suffix with all required common terminal extensions admissible and representable. Require a feasible admission separately whenever a new encounter adds collision obligations.

Let $U_k$ be the committed plan and $\xi_{j|k}$ its actual stage deficits. Applying its first command and conditioning removes the first stage. Every surviving path along the old suffix was covered by the old constraints. Its worst endpoint deficit at new stage $j$ is consequently at most $\xi_{j+1|k}$. If $N_{k+1}=N_k-1$, the endpoint already belongs to $Z_f$. Otherwise append $N_{k+1}-(N_k-1)$ holds of the common continuation. Closure inductively supplies their admissibility, terminal membership and zero collision slack. Hence a feasible successor exists, and

$$
D(U^\mathrm{shift}_{k+1})\le D(U_k)-\xi_{0|k},\qquad
L_{k+1}=D(U_k)-\xi_{0|k}.
$$

For an attained primary minimizer this yields $V_{N_{k+1}}(\mathcal I_{k+1})\le V_{N_k}(\mathcal I_k)-\xi^*_{0|k}$. For any committed incumbent, accepting only successors within $L_{k+1}$ gives the corresponding decrease of committed deficits. Starting from zero, nonnegativity forces every subsequent deficit to remain zero. In particular the replacement may choose a different separating direction. The shift proof uses existence of the inherited witnesses and membership of any replacement in the same feasible set. Under coverage, the actual vehicle then has clearance at least $d_*$ at obligated control nodes. A positive feasible deficit still requires strictly positive node clearance.

The containment premise concerns the joint physical and information evolution. Replacing the root speed, heading or motion mode with a newly observed point can violate it, even when $A,\beta$ are constant on the true trajectory. Checking the shifted trajectory against the new information establishes feasibility for that update when it passes. The induction applies across updates for which its premises hold. The node inequalities imply neither intersample separation nor admission feasibility for an arbitrary previously unseen target.
