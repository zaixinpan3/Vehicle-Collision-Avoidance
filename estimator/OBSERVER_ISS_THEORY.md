# Direct body-velocity and NRMM target observer: model and ISS certificate

The continuous core is the two-state ego body-velocity observer followed by the
six-state NRMM target observer. For one target it has eight continuous states.
A parallel continuous yaw observer supplies the orientation estimate for inertial
outputs, adding one scalar state. A propagated and intersected orientation set
certifies its error about that estimate. Yaw does not enter the body-velocity or
target dynamics or their two-stage ISS comparison.

The target model and globally Lipschitz extension below retain the framework of
Sharma, Alai, and Rajamani, *Simultaneous ego-vehicle state estimation and vehicle
trajectory tracking using a multistage high gain observer*, Transportation
Research Part C 182 (2026), 105411,
[DOI](https://doi.org/10.1016/j.trc.2025.105411). The existing NRMM coordinate
and extension derivations are retained. The ego redesign preserves the bounded
single-track mismatch assumption and combines repeated gyro errors before
bounding their effect.

The claims are fewer assumptions for core stability, one fewer upstream error
state, and smaller disturbance certificates under the comparison conditions in
Section 8. They are not claims of uniformly better estimation trajectories.
All continuous-time error bounds apply throughout time. The separate sampled
realization and its hold-error assumptions are described in
[NRMM_IMPLEMENTATION_NOTES.md](../scripts/NRMM_IMPLEMENTATION_NOTES.md).

## 1. Retained model and covariant chain

Let `J = [0,-1;1,0]`, `R(psi)` denote a planar rotation, and

\[
\nabla_u z=\dot z+uJz,\qquad
\rho=R_E^T(p_C-p_E),\quad q=R_E^Tv_C,\quad s=R_E^Ta_C.
\]

The Sharma target model has constant scalar acceleration `A` and constant
sideslip `betaC`, with positive speed. Equivalently its curvature
`kappa = sin(betaC)/lrC` is constant. This observation is used to derive the
vector field; `A` and `kappa` are not replacement online estimation states.
The exact transformed equations are

\[
\nabla_{\omega_E}\rho=q-v_E,\qquad
\nabla_{\omega_E}q=s,\qquad \nabla_{\omega_E}s=\Phi(q,s),
\]
\[
V=\|q\|,\quad A=\frac{q^Ts}{V},\quad
\Omega=\frac{(Jq)^Ts}{V^2},\qquad
\Phi=-\Omega^2q+3A\Omega J\frac qV.
\]

These are the same NRMM dynamics in physical, covariant companion coordinates.
For example, `q = rhoDot + vE + omegaE*J*rho`. Using this chain avoids
repeated differentiation of measured ego inputs. Ego jerk and angular
acceleration need not vanish. The dynamics and their saturation extension in
`nrmmTargetTrackerDerivative.m` have not been replaced by another motion model.

Truth must remain in the declared positive-speed operating domain:
`a <= |q| <= b`, `|A| <= Amax`, `|Omega| <= Omax`, `|s| <= S`, and
`|rho| <= rhoMax`, where

\[
O_{\max}=b\sin(\beta_{\max})/l_r,\qquad
S=\sqrt{A_{\max}^2+(bO_{\max})^2}.
\]

The physical heading reconstruction is not observable at zero target speed.
Estimated states may peak outside this domain: the globally Lipschitz extension
below keeps the observer defined. Estimated-domain flags report such exits;
they are not substitutes for checking the assumptions on truth.

## 2. Global Lipschitz bounds for the existing extension

Write `r = |q|`, `d = max(r,a)`, `Q = projection_ball(b,q)`,
`Svec = projection_ball(S,s)`, and `e = q/d`. The implemented extension is

\[
\alpha=\operatorname{clip}_{A_{\max}}(Q^TS_{\rm vec}/d),\quad
\nu=\operatorname{clip}_{O_{\max}}((JQ)^TS_{\rm vec}/d^2),
\qquad \Phi_e=-\nu^2Q+3\alpha\nu Je.
\]

It agrees with `Phi` on the true operating domain and is defined on all of
`R^4`. Ball projections and scalar clips are nonexpansive. Their almost-everywhere
Jacobians suffice because the maps are continuous and piecewise differentiable.
Bounds are integrated along the whole line segment in the extended space;
no convexity of the positive-speed annulus is presumed.

For fixed `s`, differentiation in the radial/tangential basis gives:

| Radial region | Bound on `|Dq alpha|` | Bound on `|Dq nu|` | Bound on `|Q| |Dq nu|` |
|---|---:|---:|---:|
| `r <= a` | `S/a` | `S/a^2` | `S/a` |
| `a <= r <= b` | `S/r` | `S/r^2` | `S/r` |
| `r >= b` | `b*S/r^2` | `2*b*S/r^3` | `2*b^2*S/r^3` |

Clipping only decreases these derivative bounds. Consequently, with

\[
c_A=S/a,\quad c_\Omega=S\max(a^{-2},2b^{-2}),\quad
c_{Q\Omega}=S\max(a^{-1},2b^{-1}),
\]
\[
L_q=O_{\max}^2+2O_{\max}c_{Q\Omega}
 +3A_{\max}O_{\max}/a+3O_{\max}c_A+3A_{\max}c_\Omega.
\]

For fixed `q`, retain the acceleration Jacobian's directional structure.
In the basis aligned with `q`, its entrywise absolute values before the
nonexpansive acceleration projection are bounded by

\[
B_s=\begin{bmatrix}0&2O_{\max}\\3O_{\max}&3A_{\max}/a\end{bmatrix},
\qquad L_s=\|B_s\|_2.
\]

At `q=0` the same bound follows by continuity. Thus, globally,

\[
\boxed{\|\Phi_e(q,s)-\Phi_e(\hat q,\hat s)\|
 \le L_q\|q-\hat q\|+L_s\|s-\hat s\|.}
\]

`nrmmTargetLipschitzCertificate.m` evaluates these analytic expressions.
A maximum over finitely many Jacobian evaluations is not used as the proof. The reconstructed
`Omega(q,s)` is differentiated, rather than frozen as a constant in this bound.
The legacy scalar `phi = hypot(Lq,Ls)` is only a diagnostic in numerical SI
coordinates; the physical certificate below uses the separate channels.

## 3. Direct forward-cone velocity measurement

Let
\[
J=\begin{bmatrix}0&-1\\1&0\end{bmatrix},\quad
v=R(\psi_E)^T\dot p_E,\quad a=R(\psi_E)^T\ddot p_E,
\qquad \dot v=a-\omega_EJv.
\]
The measurements and deterministic errors are
\[
m_I=R(\psi_E)v+n_G,\quad u=\omega_E+n_\omega,\quad a_m=a+n_a,
\quad \|n_G\|\le\varepsilon_G,\quad |n_\omega|\le\varepsilon_\omega,
\quad \|n_a\|\le\varepsilon_a.
\]
Acceleration must have the physical meaning above; compensation, calibration,
and residual bias errors belong in its error bound. No derivative of a measured
signal, measurement error, or kinematic mismatch is required by the core.

Retain the bounded-mismatch relation and true forward cone
\[
\omega_E=v_y/l_E+d_{\rm st},\quad |d_{\rm st}|\le\delta_{\rm st},\quad l_E>0,
\]
\[
\mathcal D_E=\{v:\underline V_E\le\|v\|\le\overline V_E,
\ v_x\ge\|v\|\cos b\},\quad 0\le b<\pi/2.
\]
The lower speed may be zero. This is not an exact-no-slip assumption. Write
\(M=\|m_I\|\), \(\nu=M-\|v\|\), so \(|\nu|\le\varepsilon_G\).
With \(c=\cos b\), define
\[
\phi(V,w)=\sqrt{\max\{V^2-w^2,c^2V^2\}},\qquad
\boxed{y_v=[\phi(M,l_Eu),\ l_Eu]^T.}
\]
On the true cone, \(\phi(\|v\|,v_y)=v_x\). The map requires one scalar
maximum and one square root, is defined at standstill, and has no polygon,
projection, optimization, or measurement-enclosing ball. The longitudinal floor
is an extension of the input map, not an assertion that noisy measurements lie
in the true domain. In particular the lateral component is never clipped:
\[
y_y-v_y=l_E(n_\omega+d_{\rm st}).
\]
The observer implemented by `nrmmObserverVectorField.m` is
\[
\boxed{\dot{\hat v}=a_m-uJ\hat v+k_v(y_v-\hat v),\qquad k_v>0.}
\]
`nrmmKinematicVelocityMeasurement.m` implements the algebraic map.

## 4. Exact common-error dynamics and explicit velocity certificate

On the unfloored branch, the derivatives of \(\phi\) are
\((\partial_V\phi,\partial_w\phi)=(\sec\theta,-\tan\theta)\),
where \(\sin\theta=w/V\), \(|\theta|\le b\). On the floor branch they
are \((c,0)\). The map is Lipschitz on \(V\ge0\); integrating along the
segment from \((\|v\|,v_y)\) to \((M,l_Eu)\), including branch crossings
and the origin by continuity, gives the exact finite-error identity
\[
y_v-v=\alpha e_1\nu+l_Eg(\zeta)(n_\omega+d_{\rm st}),\qquad
g(\zeta)=[-\zeta,1]^T,
\]
\[
(\alpha,\zeta)\in\operatorname{co}\bigl(
\{(\sec\theta,\tan\theta):|\theta|\le b\}\cup\{(c,0)\}\bigr).
\]
Thus \(c\le\alpha\le\sec b\), \(|\zeta|\le\tan b\). These are proof
variables, not online states. For \(e_v=\hat v-v\), substitution before taking
norms gives
\[
\boxed{\dot e_v=(-k_vI-uJ)e_v+n_a+k_v\alpha e_1\nu
 +(k_vl_Eg(\zeta)-Jv)n_\omega+k_vl_Eg(\zeta)d_{\rm st}.}
\]
The common gyro column is
\[
G_\omega=[v_y-k_vl_E\zeta,\ k_vl_E-v_x]^T.
\]
Its lateral entry is a signed difference. Treating the measurement error and
propagation gyro error as independent disturbances would lose this cancellation.
No statistical independence is assumed anywhere in the certificate.

For the strongest instantaneous specification let
\[
\mathcal C_m=\{(v,n):v\in\mathcal D_E,\ |\|v\|-M|\le\varepsilon_G,
\ |n|\le\varepsilon_\omega,\ |u-n-v_y/l_E|\le\delta_{\rm st}\}.
\]
Then a sharp forcing bound for these admitted constraints is
\[
\boxed{d_v^\star=\varepsilon_a+
\sup_{(v,n)\in\mathcal C_m}\|k_v(y_v-v)-nJv\|.}
\]
For fixed \(v\), the feasible gyro interval is
\([
\max(-\varepsilon_\omega,u-v_y/l_E-\delta_{\rm st}),
\min(\varepsilon_\omega,u-v_y/l_E+\delta_{\rm st})]\).
The norm is convex in \(n\), so a maximum occurs at an endpoint. This is a
certificate specification; the runtime does not solve this optimization.
An empty set invalidates containment while leaving the algebraic observer defined.
The code checks nonemptiness using the speed interval and the overlap of
\([l_Eu-l_E(\varepsilon_\omega+\delta_{\rm st}),
l_Eu+l_E(\varepsilon_\omega+\delta_{\rm st})]\)
with the cone's lateral interval at the largest feasible speed.

The implemented explicit alternative is
\[
\boxed{C_\omega(k)=\max_{V\in\{\underline V_E,\overline V_E\}}
\sqrt{(V\sin b+kl_E\tan b)^2+(kl_E-V\cos b)^2}.}
\]
Maximizing first over \(\zeta\), then the true angle, gives the expression
inside the maximum. Its square is convex in speed, hence the endpoint maximum.
It follows that
\[
D^+\|e_v\|\le-k_v\|e_v\|+\bar d_v,\quad
\bar d_v=\varepsilon_a+k_v\sec b\,\varepsilon_G
 +C_\omega(k_v)\varepsilon_\omega+k_vl_E\sec b\,\delta_{\rm st},
\]
\[
\boxed{U_v^{\rm new}=\frac{\bar d_v}{k_v}
 =\sec b\,\varepsilon_G+
 \frac{\varepsilon_a+C_\omega(k_v)\varepsilon_\omega}{k_v}
 +l_E\sec b\,\delta_{\rm st}.}
\]
`nrmmVelocityDisturbanceBound.m` evaluates these expressions. The transient
radius solves \(\dot B_v=-k_vB_v+d_v\), with a valid initial error bound,
and either the explicit forcing or a certified upper bound on \(d_v^\star\).

Away from standstill and the floor boundary, the first-order gyro and mismatch
columns are respectively
\[
(k_vl_E/v_x-1)Jv,\qquad (k_vl_E/v_x)Jv.
\]
Choosing \(k_vl_E\) near a representative longitudinal speed suppresses the
first-order gyro term, not mismatch. This is not exact finite-error cancellation
throughout the cone. Optional state-free scheduling
\(k_v(t)=\max(k_{\min},y_x(t)/l_E)\) preserves uniform homogeneous decay
without a gain-derivative bound. The implementation uses a fixed gain.

## 5. Retained target observer and position-normalized certificate

For \(y_\rho=\rho+n_\rho\), \(\|n_\rho\|\le\varepsilon_\rho\), retain
\[
\begin{aligned}
\dot{\hat\rho}&=\hat q-\hat v-uJ\hat\rho+\ell_1\omega_T(y_\rho-\hat\rho),\\
\dot{\hat q}&=\hat s-uJ\hat q+\ell_2\omega_T^2(y_\rho-\hat\rho),\\
\dot{\hat s}&=\Phi_e(\hat q,\hat s)-uJ\hat s+\ell_3\omega_T^3(y_\rho-\hat\rho).
\end{aligned}
\]
The true last row additionally contains \(d_j\), \(\|d_j\|\le\varepsilon_j\).
For declared scalar-acceleration and curvature rates,
\(\varepsilon_j=\sqrt{\bar{\dot A}^{\,2}+\bar V_T^4\bar{\dot\kappa}^{\,2}}\).
The nominal observer still uses constant acceleration and curvature.

With estimate-minus-truth errors define
\[
z_T=[e_\rho^T,e_q^T/\omega_T,e_s^T/\omega_T^2]^T,\quad
W_T=\sqrt{z_T^T(P\otimes I_2)z_T},
\]
\[
A_\ell=\begin{bmatrix}-\ell_1&1&0\\-\ell_2&0&1\\-\ell_3&0&0\end{bmatrix},
\qquad A_\ell^TP+PA_\ell=-I_3,\quad P>0.
\]
All components of \(z_T\) have position units. Relative to the earlier
acceleration-normalized metric, \(W_T\) and every input coefficient are divided
by \(\omega_T^2\); physical component bounds and the structured decay condition
are invariant under this normalization.

The common rotation term contributes zero to the quadratic derivative. Applying
Young inequalities separately to the velocity and acceleration sensitivities
of \(\Delta\Phi_e/\omega_T^2\) yields the sufficient condition
\[
\omega_TI_3-\tau_q(L_q/\omega_T)^2e_2e_2^T-\tau_sL_s^2e_3e_3^T
 -(\tau_q^{-1}+\tau_s^{-1})(Pe_3)(Pe_3)^T\succeq2\lambda_TP.
\]
`synthesizeTargetTrackerCertificate.m` retains its normalized observer LMI,
positive-multiplier search, generalized-eigenvalue rate, and checked residual.
Feasibility of the returned matrix is the certificate; numerical optimization
is not claimed globally optimal or verified with directed rounding.

For true bounds \(\bar\rho,\bar V_T,\bar a_T\), set
\[
h_T=[\bar\rho,\bar V_T/\omega_T,\bar a_T/\omega_T^2]^T,\quad
c_{Tv}=\sqrt{P_{11}},\quad c_{T\rho}=\omega_T\sqrt{\ell^TP\ell},
\]
\[
c_{Tj}=\sqrt{P_{33}}/\omega_T^2,\quad
c_{T\omega}=\sqrt{h_T^T|P|h_T},\quad
\bar d_T=c_{T\omega}\varepsilon_\omega+c_{T\rho}\varepsilon_\rho+c_{Tj}\varepsilon_j.
\]
Then
\[
D^+W_T\le-\lambda_TW_T+c_{Tv}\|e_v\|+\bar d_T,
\]
\[
\|e_\rho\|\le\sqrt{(P^{-1})_{11}}W_T,\quad
\|e_q\|\le\omega_T\sqrt{(P^{-1})_{22}}W_T,\quad
\|e_s\|\le\omega_T^2\sqrt{(P^{-1})_{33}}W_T.
\]

## 6. Two-stage ISS cascade and transient containment

The complete continuous core certificate is
\[
\boxed{D^+\chi\le H\chi+d,\quad
\chi=[\|e_v\|,W_T]^T,\quad
H=\begin{bmatrix}-k_v&0\\c_{Tv}&-\lambda_T\end{bmatrix},\quad
 d=[d_v,\bar d_T]^T.}
\]
The Hurwitz Metzler matrix admits positive backward weights
\(w=-H^{-T}\mathbf1\), so \(w^TH=-\mathbf1^T\). Consequently
\(D^+(w^T\chi)\le-(w^T\chi)/\max(w)+w^Td\).
`nrmmObserverCertificate.m` constructs this two-state comparison.
Given \(B_0\ge\chi(t_0)\), its nonnegative transition matrix gives
\[
\chi(t)\le e^{H(t-t_0)}B_0+\int_{t_0}^t e^{H(t-\tau)}d(\tau)\,d\tau,
\qquad W_{T,\infty}\le(c_{Tv}U_v^{\rm new}+\bar d_T)/\lambda_T.
\]
Initial component radii give a metric bound using
\(D=\operatorname{diag}(1,\omega_T^{-1},\omega_T^{-2})\):
\(W_T(t_0)\le\sqrt{(Db_{T0})^T|P|(Db_{T0})}\).
A steady-state radius does not certify arbitrary initial errors.

The gyro is shared across both stages as well. Its exact stacked column is
\[
n_\omega\begin{bmatrix}k_vl_Eg(\zeta)-Jv\\-(S_T\otimes J)x_T\end{bmatrix},
\quad S_T=\operatorname{diag}(1,\omega_T^{-1},\omega_T^{-2}),\quad
x_T=[\rho^T,q^T,s^T]^T.
\]
A joint support-function or composite Lyapunov calculation must keep this
column: its directional contribution is the absolute value of a sum, before
any triangle inequality. The simple triangular norm certificate above exploits
within-velocity cancellation and does not claim all cross-stage cancellations.

## 7. Explicit gain-selection preference and position output

The target shape is unchanged. Its physical bandwidth minimizes the largest
ultimate physical component bound normalized by its operating-domain scale,
subject to \(\lambda_T\ge1/T_{\rm domain}\), \(\omega_T>1\), where
\(T_{\rm domain}=\bar\rho/(\overline V_E+\bar V_T)\).
The fixed velocity gain retains the declared minimum-bandwidth transit rule
\(k_v=\log(20)/T_{\rm domain}\). This is an engineering preference, not
an ISS optimality claim. Section 4 explains why gain increases need not improve
the common-error certificate. Runtime timing does not enter synthesis.

The independent inertial position output uses
\(\dot{\hat p}_E=m_I+k_p(y_p-\hat p_E)\),
\(k_p=\log(20)/T_{\rm domain}\). Its continuous radius is bounded ultimately
by \(\varepsilon_p+\varepsilon_G/k_p\). This choice keeps all use of the
orientation estimate at output reconstruction. Position and sampled
output predictors are not counted among the eight core observer states.

## 8. Precise comparison with the yaw-mediated design

For \(\overline V_E>0\), \(k>0\), and \(b<\pi/2\),
\[
C_\omega(k)<\overline V_E+kl_E\sec b.
\]
For a positive speed endpoint \(V\), the squared triangle bound minus the
squared combined expression is \(4Vkl_E\cos b>0\). A zero endpoint also
lies strictly below the bound using positive \(\overline V_E\).
At \(b=0\),
\(C_\omega(k)=\max(|kl_E-\underline V_E|,|kl_E-\overline V_E|)\).
This finite-error improvement concerns a certificate, not sensor information.
On the unfloored branch, rotating GNSS by the instantaneous pseudo-heading
\(h_m=\arg(m_I)-\arcsin(l_Eu/M)\) gives exactly \(y_v\).

For the original valid uniform yaw certificate let
\[
a_0=\underline V_E-\varepsilon_G,\quad b_0=\underline V_E-2\varepsilon_G>0,
\quad U=\bar\omega_E+\varepsilon_\omega,
\]
\[
L_\psi=(1-\max\{\sin b,l_EU/a_0\}^2)^{-1/2},\quad
B_F=\arcsin(\varepsilon_G/a_0)+\frac{l_EL_\psi}{b_0}
 (\varepsilon_\omega+\delta_{\rm st}+U\varepsilon_G/a_0).
\]
The old velocity certificate at the same \(k_v\) was
\[
U_v^{\rm old}=\varepsilon_G+\overline V_E(B_F+\varepsilon_\omega/k_\psi)
 +(\varepsilon_a+\overline V_E\varepsilon_\omega)/k_v.
\]
Its raw inversion, proper arc, and invariant yaw-chart premises must hold before
this comparison applies. The mismatch gains satisfy
\[
\Gamma_{\rm st}^{\rm new}=l_E\sec b\le
\Gamma_{\rm st}^{\rm old}=(\overline V_E/b_0)l_EL_\psi.
\]
The redesign retains mismatch dependence while removing its speed-ratio
amplification. For unchanged target gains, multiply the new mismatch coefficient
by \(\sqrt{(P^{-1})_{11}}c_{Tv}/\lambda_T\) for the target-position bound.

A sufficient complete-bound comparison is
\[
\boxed{\text{old certificate valid, same }k_v,\ b\le\pi/3
\quad\Longrightarrow\quad U_v^{\rm new}\le U_v^{\rm old}.}
\]
Use \(C_\omega/k_v\le\overline V_E/k_v+l_E\sec b\),
\(\overline V_E\arcsin(\varepsilon_G/a_0)\ge\varepsilon_G\),
\(\overline V_EL_\psi/b_0\ge\sec b\), and \(\sec b\le2\).
Positive gyro uncertainty makes the comparison strict. For larger cones compare
actual bounds; the core theorem still applies. A smaller worst-case certificate
does not imply better trajectories for every signal: a yaw filter can suppress
particular noise histories.

## 9. Parallel yaw observer and its orientation certificate

Retain the original gyro-plus-heading observer as a parallel output estimator:
\[
\dot{\hat\psi}_E=u+\chi_m k_\psi
 \operatorname{wrap}(h_m-\hat\psi_E),\qquad k_\psi>0.
\]
Here \(h_m\) is the certified kinematic course correspondence heading and
\(\chi_m=1\) only when that correspondence is informative; otherwise
\(\chi_m=0\) and the observer propagates the measured gyro alone. The estimate
is never reset to a measurement center or the center of an orientation set.
`nrmmYawObserverDerivative.m` implements this equation. The original bandwidth
preference is retained: \(k_\psi=\log(20)/T_{\rm domain}\). It does not depend
on runtime timing or impose a uniform low-speed inversion precondition.

On a compatible innovation chart with continuous informative measurements and
\(h_m=\psi_E+\eta_m\), the local yaw error satisfies
\(\dot e_\psi=-k_\psi e_\psi+n_\omega+k_\psi\eta_m\).
Its exponential comparison requires that chart and heading-error premises.
During uninformative intervals only gyro propagation is available; no uniform
yaw convergence claim is made over arbitrary such intervals. These conditions
are separate from the body-frame core ISS theorem. The velocity innovation
remains the direct algebraic map, and the position output still uses inertial
GNSS velocity independently.

The exact measurement consistency set is
\[
\mathcal Y_m=\{\psi\in S^1:\exists(v,n)\in\mathcal C_m,
\ \|m_I-R(\psi)v\|\le\varepsilon_G\}.
\]
The implementation uses the retained course correspondence as a certified outer
arc. If uninformative it contributes \(S^1\). It propagates and intersects
\[
\Psi^- =\operatorname{wrap}\bigl(\Psi(t_0)\oplus\{\int u\}
 \oplus[-\varepsilon_\omega(t-t_0),\varepsilon_\omega(t-t_0)]\bigr),
\qquad \Psi=\Psi^-\cap\mathcal Y_m.
\]
`nrmmYawSet.m` retains disconnected components as a union of closed circular
intervals. For a sampled held gyro, the integral-error radius includes the hold
uncertainty; the sensor error alone cannot certify an arbitrary intersample rate.
Empty intersections stay empty and invalidate orientation outputs. They never
reset the set to an arc center or invalidate an otherwise consistent body core.

At output use the continuous observer estimate and enclose the set about it:
\[
\psi_{\rm out}=\hat\psi_E,\qquad
B_\psi=\sup_{\theta\in\Psi}d_{S^1}(\hat\psi_E,\theta).
\]
`nrmmYawSet("radiusAbout",...)` computes this radius from every retained
interval, including \(\pi\) if an interval contains the observer's antipode.
The set's smallest covering arc remains available as metadata, but its radius
alone is not an error bound about a different \(\hat\psi_E\). The whole circle
has radius \(\pi\) about every estimate; an empty set publishes an invalid,
infinite radius. Changes in \(\hat\psi_E\) do not rotate or reset the body states.
For a body vector with error radius \(E_z\), publish
\[
\hat z^I=R(\psi_{\rm out})\hat z,\qquad
\mathcal Z^I=\bigcup_{\psi\in\Psi}R(\psi)(\hat z+\mathbb B_{E_z}).
\]
Using also the true norm bound \(\|z\|\le\bar z\), a certified error radius is
\(E_z+2\min(\|\hat z\|,\bar z)\sin(B_\psi/2)\).
The true norm bound can further intersect each body ball; this scalar radius
need not enclose points in the unrestricted union that violate that bound.
Absolute target position adds the ego-position radius. The runtime publishes
orientation intervals, body centers/radii, and scalar controller enclosures.
In the sampled implementation the actual RK4 yaw endpoint is used to compute
\(B_\psi\). Set containment therefore bounds that numerical estimate without
requiring a yaw integration-defect inequality. The core still has eight states
for one target; the parallel yaw observer adds one, while the independent
position output and sampled predictors remain outside that core count.

## 10. Scope

Core ISS requires the forward branch, bounded single-track mismatch, continuous
sensor-error bounds, true target operating domain, positive gains and the target
dissipation inequality. It requires neither an ego low-speed inversion condition,
nor admissible raw arcsine, informative yaw arcs, yaw charts or yaw bandwidth.
The positive target-speed condition for the retained NRMM extension remains.
Estimated-domain audits are diagnostics, not proof that truth obeys the premises.
Sampled containment, floating-point qualifications and future prediction remain
separate from the continuous theorem and from closed-loop collision avoidance.

## 11. Measurement-history postprocessing and physical validation

The finite-sensing integration adds `nrmmTargetHistory` after the observer.
Bounded-noise position secants and quadratic interpolation intersect the
current velocity/acceleration enclosures. Gyro transport keeps the common
absolute-heading error correlated across history samples; it does not assume
independent deterministic sensor errors. Constant-curvature position chords
also bound course direction by half the bounded angular sweep. The formulas
and sampled transport allowance are documented in
[FINITE_SENSING_CONTROLLER.md](../controller/FINITE_SENSING_CONTROLLER.md).
All published bounds remain centered about the original NRMM point estimates.
Neither the point-observer vector field nor its gain design changes.

This postprocessing cannot repair a violated premise of the ego observer.
In a dynamic bicycle, rear tire slip permits
\(r-v_y/l_r\ne0\). A small configured mismatch is a true-motion assumption,
not a consequence of the gyroscope or of estimated-domain consistency.
The simulation-only `auditNrmmTruthEnclosure` checks the true mismatch,
speed, yaw rate, sideslip and known state-component errors at logged times.
It never feeds truth to the observer or controller. Passing sampled checks
does not prove continuous-time validity; a violation invalidates application
of the corresponding conditional certificate even if all states are finite.
