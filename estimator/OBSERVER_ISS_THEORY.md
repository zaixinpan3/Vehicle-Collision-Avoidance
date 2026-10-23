# Direct body-velocity and NRMM target observer: model and ISS certificate

The continuous core is the two-state ego body-velocity observer followed by the
six-state NRMM observer of one target. The core has eight continuous states.
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
The vector-Jacobian refinement in Sections 2 and 5 draws on Sharma, Zemouche,
and Rajamani, *Observer Design for Vector Nonlinear Systems With Applications
to Autonomous Vehicles and Robots*, International Journal of Robust and
Nonlinear Control (2026), [DOI](https://doi.org/10.1002/rnc.70732): Lemma 1,
Theorem 1 (13), Corollary 2 (26), and the scaling construction (33)--(35).
We specialize their incremental analysis to the retained covariant chain.
The analytic matrix enclosure and the correlated NRMM gyro bound below are
derived here, not results asserted by that paper. They change certificates and
synthesized gains, while preserving the observer states and vector field.
All continuous-time error bounds apply throughout time. The separate sampled
realization and its hold-error assumptions are described in
[NRMM_IMPLEMENTATION_NOTES.md](../report/NRMM_IMPLEMENTATION_NOTES.md).
With the controller's vehicle model and held input, the lateral component of
the velocity measurement of Section 3 is replaced by a force-balance inversion
with its own certified radius; see
[MODEL_AIDED_ESTIMATION.md](MODEL_AIDED_ESTIMATION.md). Sections 3, 4 and 8
describe the kinematic branch, which is used without that model.
A paired synthetic comparison against a reproduction of the Sharma multistage
observer (`sharmaMultistageObserver.m`: design, sampled runtime and companion-form
map) is recorded in
[OBSERVER_SHARMA_COMPARISON_20260924.md](../report/OBSERVER_SHARMA_COMPARISON_20260924.md);
it measures sampled trajectories, not the certificates above, and predates
the current target gains. The comparison with reference gains tuned per
scenario is in
[TARGET_NOISE_SHAPE_GAINS_20261001.tex](../report/TARGET_NOISE_SHAPE_GAINS_20261001.tex).

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

Truth must remain in the declared operating domain:
`Vmin <= |q| <= b`, `|A| <= Amax`, `|Omega| <= Omax`, `|s| <= S`, and
`|rho| <= rhoMax`, where

\[
O_{\max}=b\sin(\beta_{\max})/l_r,\qquad
S=\sqrt{A_{\max}^2+(bO_{\max})^2}.
\]

For a strictly moving domain, the inverse floor is `a = Vmin > 0`.
A domain including rest instead declares `Vmin = 0` and a separate positive
`regularizationSpeedMinimum = a < b`. This floor regularizes divisions; it
never replaces the Cartesian target velocity by a fictitious positive speed.
The corresponding model discrepancy is bounded in Section 2 and enters the ISS input.
The physical heading reconstruction is not observable at zero target speed.
Without informative heading history, a velocity enclosure containing zero
requires a full-circle heading enclosure and a speed interval including zero. Below the inverse
floor, reconstructed yaw-rate and sideslip errors use the physical domain caps,
because the regularized inverse is not the physical inverse there.
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

It agrees with `Phi` for admissible truth with `V >= a` and is defined on all of
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
L_q^{\rm sep}=O_{\max}^2+2O_{\max}c_{Q\Omega}
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

`nrmmObserverCertificate("targetLipschitz", ...)` evaluates these analytic expressions.
A maximum over finitely many Jacobian evaluations is not used as the proof. The reconstructed
`Omega(q,s)` is differentiated, rather than frozen as a constant in this bound.
The legacy scalar `phi = hypot(Lq,Ls)` is only a diagnostic in numerical SI
coordinates. These scalar bounds support sampled integration and radar-dropout
growth estimates; they no longer constrain continuous gain synthesis.

### 2.1 Coupled vector-Jacobian refinement

The product bound above discards cancellations between derivatives of the
same nonlinear factors. Retain them in the orthonormal frame
\(Q_r=[e_r,Je_r]\), \(e_r=q/r\), for \(r>0\). Write
\(Q_r^TS_{\rm vec}=[x,y]^T\), \(c=\min(r,b)\), \(d=\max(r,a)\), and
\[
u=cx/d,\quad v=cy/d^2,\quad
\alpha=\operatorname{clip}_{A_{\max}}u,\quad
\nu=\operatorname{clip}_{O_{\max}}v.
\]
Let \(\chi,\eta\in\{0,1\}\) be the respective scalar-clip derivatives at
differentiability points. Then \(\chi u=\chi\alpha\) and
\(\eta v=\eta\nu\). With \(B=Q_r^TD_q\Phi_e Q_r\), direct differentiation gives:

| Region | \(B_{11}\) | \(B_{12}\) | \(B_{21}\) | \(B_{22}\) |
|---|---|---|---|---|
| \(r<a\) | \(-(1+2\eta)\nu^2\) | \(\nu(2\eta u-3\alpha)/a\) | \(3\alpha\nu(1+\chi+\eta)/a\) | \(-\nu^2+3\chi\nu v-3\eta\alpha u/a^2\) |
| \(a<r<b\) | \((2\eta-1)\nu^2\) | \(\nu(2\eta u-3\alpha)/r\) | \(-3\eta\alpha\nu/r\) | \(-\nu^2+3\chi\nu v-3\eta\alpha u/r^2\) |
| \(r>b,\ t=b/r\) | \(4t\eta\nu^2\) | \(\nu(2t\eta u-3\alpha)/r\) | \(-3\alpha\nu(\chi+2\eta)/r\) | \(-t\nu^2+3\chi\nu v-3\eta\alpha u/r^2\) |

For brevity set \(A=A_{\max},O=O_{\max}\) in the next formulas. Since clipping
preserves sign, \(\nu v,\alpha u\ge0\); since \(|u|\le S\),
\[
K=\max(3A,2S-3A),\qquad
|2\theta u-3\operatorname{clip}_A u|\le K\quad(0\le\theta\le1).
\]
For fixed \(u\), the maximum is at an endpoint of \(\theta\); the remaining
piecewise affine function has extrema at \(0,\pm A,\pm S\) when those
points lie in the interval. Also define
\[
H(d)=\max\{O^2+3AS/d^2,\ 3OS/d\}.
\]
The negative and positive terms of \(B_{22}\) can be bounded separately,
without summing their magnitudes. The following entrywise envelopes therefore
cover all scalar clips and the three radial regions:
\[
M_1=\begin{bmatrix}3O^2&OK/a\\9AO/a&H(a)\end{bmatrix},\quad
M_2=\begin{bmatrix}O^2&OK/a\\3AO/a&H(a)\end{bmatrix},\quad
M_3=\begin{bmatrix}4O^2&OK/b\\9AO/b&H(b)\end{bmatrix}.
\]
For any vector \(z\), \(|Bz|\le M_i|z|\), hence
\(\|D_q\Phi_e\|_2=\|B\|_2\le\|M_i\|_2\). The implemented bound is
\[
\boxed{L_q=\min\{L_q^{\rm sep},\max_i\|M_i\|_2\}.}
\]
The acceleration projection is independent of \(q\); it is already included
through \(\sqrt{x^2+y^2}\le S\). The earlier \(L_s\) is unchanged.
At \(q=0\) the bound follows by continuity of the extension. At switching
surfaces generalized Jacobians are convex combinations of adjacent limiting
Jacobians, so the same uniform norm bound applies. Equivalently, integrate
almost-everywhere derivatives along segments, using continuity at crossings.
This proves the global finite-increment inequality displayed above, including
segments through zero and saturated estimates outside the physical domain.

The norm reduction remains useful for runtime growth estimates. Continuous gain
synthesis instead retains the complete signed matrices, as follows.

### 2.2 Global analytic enclosure of the full vector Jacobian

Write \(H=[B,C]=[D_q\Phi_e,D_s\Phi_e]\) in the frame aligned with \(q\).
The matrix \(B\) is given in Section 2.1. Before differentiating the
acceleration projection, the other block is
\[
C_0=\begin{cases}
(r/a)^2\begin{bmatrix}0&-2\eta\nu\\3\chi\nu&3\eta\alpha/a\end{bmatrix},&r<a,\\[2mm]
\begin{bmatrix}0&-2\eta\nu\\3\chi\nu&3\eta\alpha/r\end{bmatrix},&a<r<b,\\[2mm]
\begin{bmatrix}0&-2\eta\nu t^2\\3\chi\nu t&3\eta\alpha t^2/b\end{bmatrix},&r>b,\ t=b/r.
\end{cases}
\]
For \(\|s\|<S\), \(C=C_0\). For \(\|s\|>S\), set
\(\sigma=S/\|s\|\in(0,1)\). In this frame,
\[
C=\sigma C_0K,\qquad
K=I-[x,y]^T[x,y]/S^2
 =\frac1{S^2}\begin{bmatrix}y^2&-xy\\-xy&x^2\end{bmatrix},
\quad x^2+y^2=S^2.
\]
Both blocks therefore depend on the same scalar factors, signs and clip
branches. Replacing their entries by independent intervals would lose that
information. `nrmmTargetJacobian("enclosure",domain)` instead constructs a
finite convex enclosure from the following compact variables:

| Region | Compact variables | Raw arguments \((u,v)\) | Feasible acceleration set | Common denominator |
|---|---|---|---|---|
| \(r<a\) | \(X=(r/a)x,\ Y=(r/a)y\) | \((X,Y/a)\) | \(X^2+Y^2\le S^2\) | \(1\) |
| \(a<r<b\) | \((r,X,Y)=(r,x,y/r)\) | \((X,Y)\) | \(X^2+r^2Y^2\le S^2\) | \(r^2\) |
| \(r>b\) | \((t,X,Y)=(b/r,x,y)\) | \((tX,t^2Y/b)\) | \(X^2+Y^2\le S^2\) | \(1\) |

In the middle and outer regions, projected acceleration lies on the indicated
ellipse or circle boundary. In the inner region even projected acceleration
fills the disk: multiplication by \(r/a\) changes its radius. There,
\((r/a)^2K=S^{-2}[Y^2,-XY;-XY,X^2]\). Thus the projected inner block
is polynomial in \((X,Y)\), apart from the independent scalar
\(\sigma\in[0,1]\). For unprojected inner acceleration the factor
\((r/a)^2\in[0,1]\) is enclosed by its two endpoints. This endpoint
relaxation only enlarges the admissible matrix set; it does not assert that
these factors are physically independent.

The extension is equivariant under every \(R\in O(2)\), including reflections:
\(\Phi_e(Rq,Rs)=R\Phi_e(q,s)\). Under reflection \(\nu\) changes sign and
\(JR=\det(R)RJ\), so the two sign changes cancel. Consequently \(Y\ge0\)
suffices, provided the certificate uses \(P\otimes I_2\). Both signs of
\(X\) and all three scalar-acceleration branches are retained. The yaw
branches are unsaturated or positively saturated. They cover all smooth
states up to an orthogonal change of frame.

On each clip branch the numerator of every matrix entry is polynomial in
these compact variables. In the middle region multiply the entire matrix by
the positive denominator \(r^2\), retaining one common denominator for all
eight entries. The implementation subdivides each compact coordinate into
eight intervals, intersects boxes with the scalar-clip conditions, and removes
boxes that cannot meet the acceleration disk or projected boundary. Every
feasible branch point remains in at least one retained box. A retained box
may contain infeasible points, which adds conservatism without compromising
coverage. Its radial polynomial degree is at most five, and its other two
degrees are at most three. No values sampled from state trajectories enter
this construction.

For completeness, let \(N(z)\) be the matrix numerator and \(D(z)>0\) its
common denominator on a retained box. Express both in the same tensor
Bernstein basis \(b_k(z)\ge0\), \(\sum_k b_k(z)=1\):
\[
N(z)=\sum_k N_k b_k(z),\qquad D(z)=\sum_k d_k b_k(z).
\]
The denominators are \(1\) or \(r^2\) with \(r\ge a>0\); their Bernstein
coefficients satisfy \(d_k>0\), also after degree elevation. Hence
\[
\boxed{\frac{N(z)}{D(z)}=
 \sum_k\underbrace{\frac{d_k b_k(z)}{D(z)}}_{\ge0,\ \sum_k=1}
 \frac{N_k}{d_k}.}
\]
The **same weights apply to every entry** of \([B,C]\). The vertices
\(N_k/d_k\), together with the scalar-factor endpoints, therefore enclose
the complete coupled matrix. Bernstein coefficients can lie outside the
image of the nonlinear map; this is a sufficient enclosure, not an exact
convex hull or a claim of minimum conservatism. `evaluate` separately computes
the Cartesian chain-rule Jacobian for diagnostics and testing.

At \(q=0\), \(D\Phi_e=0\), which is explicitly included. Adjacent branches
cover the generalized Jacobians at switching surfaces. For any two extended
states, integrate along the **whole** segment:
\[
\Delta\Phi_e=\int_0^1D\Phi_e(\hat z+\theta(z-\hat z))(z-\hat z)\,d\theta.
\]
A suitable generalized derivative gives the same identity for a segment on
a switching surface. This argument does not assume a common intermediate
point for all partial derivatives or convexity of the physical annulus.
The matrix inequalities in Section 5 hold for all orthogonal conjugates and
all convex averages, so they imply a global finite-error inequality.

This is a matrix-polytopic specialization of the vector incremental technique
in Sharma et al., Lemma 1 and Theorem 1. It preserves vector directions in the
actual SDP. It is not a reproduction of every sector multiplier in their
Equation (13), nor a claim of equivalence to that parameterization. One metric
and one gain cover every region; there is no switching of observer gains.
The enclosure theorem is in real arithmetic. The implementation uses ordinary
double precision and verifies numerical residuals; it does not provide
interval arithmetic or directed-rounding proof certificates.

### 2.3 Regularization discrepancy for domains including rest

For a domain including rest, let `kappaMax = sin(betaMax)/lr`. On a smooth
constant-acceleration, constant-curvature NRMM arc with `0 < V < a`, the physical
acceleration is `s = A e + kappa V^2 J e`. The projections and scalar clips are
inactive at this admissible truth, except for the inverse floor. Consequently,

\[
\Phi=-\kappa^2 V^3 e+3A\kappa VJe,\qquad
\Phi_e=(V/a)^4\Phi,
\]
\[
\|\Phi-\Phi_e\|\le
J_{\mathrm{reg}}=
\sqrt{\kappa_{\max}^4a^6+9A_{\max}^2\kappa_{\max}^2a^2}.
\]

At rest, `q = s = 0` gives `Phi = Phi_e = 0`. The same finite bound therefore
covers stationary targets and smooth low-speed arcs. Gain synthesis and online
observer propagation use `observerModelJerkMaximum = modelJerkMaximum +
regularizationJerkMaximum` exactly once. The physical prediction contract and
measurement-history bounds retain `modelJerkMaximum`: inverse regularization
is not a new physical target maneuver.

An instantaneous acceleration reset when a braking target reaches rest is a
separate hybrid transition. This finite-jerk argument does not certify that
jump or a differentiation window spanning it; it is not needed for a target
that is stationary throughout the observation and prediction interval.

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
The observer implemented by `nrmmObserverVectorField("derivative", ...)` is
\[
\boxed{\dot{\hat v}=a_m-uJ\hat v+k_v(y_v-\hat v),\qquad k_v>0.}
\]
`nrmmObserverVectorField("velocityMeasurement", ...)` implements the algebraic map.

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
`nrmmObserverCertificate("velocity", ...)` evaluates these expressions. The transient
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
For a domain including rest, add `Jreg` from Section 2 to this last-row bound
when comparing truth with the regularized observer. The nominal observer still
uses constant acceleration and curvature.

With estimate-minus-truth errors define
\[
z_T=[e_\rho^T,e_q^T/\omega_T,e_s^T/\omega_T^2]^T,\quad
W_T=\sqrt{z_T^T(P\otimes I_2)z_T},
\]
\[
A_\ell=\begin{bmatrix}-\ell_1&1&0\\-\ell_2&0&1\\-\ell_3&0&0\end{bmatrix},
\qquad P>0.
\]
All components of \(z_T\) have position units. Relative to the earlier
acceleration-normalized metric, \(W_T\) and every input coefficient are divided
by \(\omega_T^2\); physical component bounds and the structured decay condition
are invariant under this normalization.

Rotation equivariance gives
\(\Phi_e(Rq,Rs)=R\Phi_e(q,s)\) for every planar rotation \(R\).
The invariant metric \(P\otimes I_2\) commutes with
\(\mathcal J=I_3\otimes J\), and
\(\mathcal J^T(P\otimes I_2)+(P\otimes I_2)\mathcal J=0\).
Thus arbitrary common measured rotation contributes exactly zero to the
quadratic derivative; its magnitude is not charged as a Lipschitz penalty.

For fixed scalar-block gains, this metric symmetry has a precise justification.
The full Jacobian uncertainty family, including all orthogonal conjugates, is
closed under \(U=I_3\otimes R\), \(R\in O(2)\). If a common quadratic metric
\(M_0\) certifies this family, so does every \(U^TM_0U\). Averaging over the
compact group preserves positivity and all dissipation inequalities and gives
\(\overline M=P\otimes I_2\). Thus imposing this symmetry loses no common
quadratic feasibility for that fixed gain and orthogonally closed family.
This statement does not remove conservatism from the uncertainty enclosure,
the use of a common quadratic function, or the gain-selection objective.

Let \(H_j=[B_j,C_j]\) range over all analytic vertices of Section 2.2 and
set \(M=P\otimes I_2\), \(E=e_3\otimes I_2\), and
\[
D_j(\omega_T)=[0_{2\times2},B_j/\omega_T,C_j],\qquad
F_j=\omega_T(A_\ell\otimes I_2)+ED_j(\omega_T).
\]
The actual, full vector-Jacobian LMIs are
\[
\boxed{F_j^TM+MF_j+2\lambda_TM\preceq0\quad\text{for every }j.}
\]
Equivalently,
\[
[\omega_T(A_\ell^TP+PA_\ell)+2\lambda_TP]\otimes I_2
 +(Pe_3\otimes I_2)D_j+D_j^T(e_3^TP\otimes I_2)\preceq0.
\]
The signed off-diagonal entries and shared nonlinear factors of \(B_j,C_j\)
enter these six-dimensional LMIs directly. There are no scalar Lipschitz
multipliers in this synthesis. For any frame \(R\in O(2)\),
\(I_3\otimes R\) leaves \(M\) and the chain unchanged. Congruence therefore
extends each inequality to the corresponding rotated or reflected Jacobian.
Convexity extends it to the integrated Jacobian of any finite error. This
establishes \(\dot W_T\le-\lambda_TW_T\) for the unforced nonlinear error,
including states outside the physical domain. The common measured rotation
still contributes exactly zero.

`synthesizeNrmmObserverGains.m` selects the gain shape with Section 7's SDP.
For each candidate bandwidth it solves these LMIs for a free \(P\), minimizing
\(\operatorname{tr}(P)\) subject to \(P\succeq I\) and strict numerical margins.
This normalization fixes the homogeneous scale; it is not a globally optimal
physical-error metric. Constraint generation starts with a subset of vertices,
adds violated matrices, and accepts a solution only after checking **every**
vertex of the complete analytic enclosure. Stopping the SDP iteration without
that full check yields an unaccepted candidate, never a certificate.
SeDuMi numerical-termination status 4 is allowed only when the recovered
positive metric, the shape's Gramian inequality, and every Jacobian inequality
pass the same independent strict checks as status 0. Infeasible statuses are
rejected. The reported objective is recomputed from the recovered gains and
metric; solver optimality is not used as a stability premise.
Stored metadata record that status and distinguish numerical residual checks
from directed-rounding verification. The retained worst residual matrix is a diagnostic; the theorem
uses the verified maximum residual eigenvalue over all vertices.

The previous two-channel norm LMI is sufficient for its norm uncertainty set,
but that set ignores Jacobian signs and couplings. The present polynomial
outer enclosure introduces a different relaxation. Neither implementation is
asserted to dominate the other on every domain. The default-domain comparison
is an experimental result, not a general ordering theorem.

For true bounds \(\bar\rho,\bar V_T,\bar a_T\), set
\[
h_T=[\bar\rho,\bar V_T/\omega_T,\bar a_T/\omega_T^2]^T,\quad
c_{Tv}=\sqrt{P_{11}},\quad c_{T\rho}=\omega_T\sqrt{\ell^TP\ell},
\]
\[
c_{Tj}=\sqrt{P_{33}}/\omega_T^2,\quad
\bar d_T=c_{T\omega}\varepsilon_\omega+c_{T\rho}\varepsilon_\rho+c_{Tj}\varepsilon_j.
\]
The separated-domain gyro bound is
\(c_{T\omega}^{\rm sep}=\sqrt{h_T^T|P|h_T}\). It allows impossible simultaneous
extrema of the inner products between \(\rho,q,s\). Instead use the same gyro
error once, in the column
\(-[J\rho,Jq/\omega_T,Js/\omega_T^2]^T\). By rotation invariance and the true
NRMM geometry, take \(q=[V,0]^T\), \(s=[A,V\Omega]^T\). Maximizing over
\(\|\rho\|\le R\) gives the exact squared norm
\[
\begin{aligned}
F(V,A,\Omega)={}&P_{11}R^2+P_{22}V^2/\omega_T^2
 +P_{33}(A^2+V^2\Omega^2)/\omega_T^4+2P_{23}VA/\omega_T^3\\
 &+2R\sqrt{(P_{12}V/\omega_T+P_{13}A/\omega_T^2)^2
                 +(P_{13}V\Omega/\omega_T^2)^2}.
\end{aligned}
\]
For fixed other arguments this is convex in \(V\) and in \(A\), and
nondecreasing in \(|\Omega|\). Therefore its maximum on the declared box is
at \(V\in\{V_{\min},b\}\), \(A\in\{-A_{\max},A_{\max}\}\),
\(|\Omega|=O_{\max}\). Define
\[
\boxed{c_{T\omega}=\min\left\{c_{T\omega}^{\rm sep},
 \sqrt{\max_{V\in\{V_{\min},b\},\ A=\pm A_{\max}}F(V,A,O_{\max})}\right\}.}
\]
The box relaxes the additional sideslip relation \(|\Omega|\le
V\sin(\beta_{\max})/l_r\); this only enlarges the set. At zero speed the
box also permits extra acceleration directions, so it remains an outer set
for stationary truth. The separated bound still applies if an independently
specified acceleration-norm cap is smaller than the box norm cap; the minimum
of the two valid bounds remains valid. The stored maximizing state pertains
to the relaxed box, not necessarily to that intersection.
`nrmmObserverCertificate("targetGyro", ...)` evaluates the four corners using
a Cholesky metric norm. Gain synthesis uses \(R=\bar\rho\); sampled propagation
uses its certified interval range bound. General initial-error and numerical
defect vectors do not satisfy \(q^Ts=VA\), so their existing absolute-metric
enclosures are deliberately retained. Radar noise likewise remains one shared
two-dimensional input through the entire injection vector.

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
`nrmmObserverCertificate("core", ...)` constructs this two-state comparison.
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
any triangle inequality. For dual directions \(a_v\in\mathbb R^2\) and
\(a_T\in\mathbb R^6\), its exact support is
\[
\varepsilon_\omega\left|a_v^T(k_vl_Eg(\zeta)-Jv)
 -a_T^T(S_T\otimes J)x_T\right|.
\]
Separate absolute values give only an upper bound. The implemented triangular
certificate exploits cancellation within the velocity stage and the shared
NRMM target column of Section 5. It still separates the two stage norms and
does not claim cross-stage cancellation. Exploiting the displayed support in
a joint metric would require a new joint dissipation and sampled-predictor
certificate, including the separate force-balance measurement branch.

For fixed metric, gains and domains the refined \(L_q\) and gyro coefficient
never exceed their separated predecessors. The previous dissipation inequality
therefore remains feasible and its ultimate bound cannot increase when only
these coefficients are substituted. After optimizing a new shape and metric,
neither globally optimal bounds nor uniformly improved noisy trajectories
follow; those are separate numerical comparisons.

## 7. Explicit gain-selection preference and position output

The target shape is obtained from one semidefinite program, solved by constraint
generation and full-enclosure verification, in the physical
gains \(L\in\mathbb R^3\). With \(A_L=A-Le_1^T\) and \(Y=PL\) it minimizes
\(\gamma\) subject to the dissipation condition of Section 5 written in
physical coordinates (\(\omega_T=1\)) at the decay floor
\(\lambda_T=1/T_{\rm domain}\), and
\[
A_L^TP+PA_L+e_2e_2^T\preceq0,\qquad
\begin{bmatrix}\gamma&Y^T\\Y&P\end{bmatrix}\succeq0 .
\]
The first inequality makes \(P\) an upper bound of the observability Gramian
of \((A_L,e_2^T)\). For the linear error equation \(\dot e=A_Le-Ln\) driven by
white output noise of unit intensity in one Cartesian component, the stationary
velocity-error variance in that component is therefore at most \(L^TPL\le\gamma\). All conditions are linear in
\((P,Y,\gamma)\), since \(PA_L=PA-Ye_1^T\) and the analytic
Jacobian vertices are fixed. One metric serves the Gramian and the
dissipation inequality, so \(\gamma\) is an upper bound of the smallest
variance attainable among gains with a certificate, not that minimum. The
returned gains already satisfy the dissipation inequality; they are split
into the shape \(\ell=L\oslash(\omega_0,\omega_0^2,\omega_0^3)\) and its
slowest closed-loop rate \(\omega_0\), so the normalized shape has unit
slowest rate. The variance is a performance preference among certified gains.
It ignores \(\Delta\Phi_e\) and the sampled realization, and it is not a
deterministic error bound; the deterministic bounds remain those of Section 5
at the selected bandwidth.

At each candidate bandwidth a free metric is
synthesized as above. A bracketed one-dimensional search selects a feasible
local minimum of the largest ultimate physical component bound normalized by
its operating-domain scale,
subject to \(\lambda_T\ge1/T_{\rm domain}\), \(\omega_T>1\), where
\(T_{\rm domain}=\bar\rho/(\overline V_E+\bar V_T)\). The search reports its
bracket, does not claim global optimality, and rejects infeasible or independently
unverified metric solves. When all disturbances vanish, minimum feasible
bandwidth is the tie-break objective. No simulated tracking error or sensor
sample period enters either optimization.
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
`nrmmObserverVectorField("yawDerivative", ...)` implements this equation. The original bandwidth
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
requiring a yaw integration-defect inequality. The core has eight states; the parallel yaw observer adds one, while the independent
position output and sampled predictors remain outside that core count.

## 10. Scope

Core ISS requires the forward branch, bounded single-track mismatch, continuous
sensor-error bounds, true target operating domain, positive gains and the target
dissipation inequality. It requires neither an ego low-speed inversion condition,
nor admissible raw arcsine, informative yaw arcs, yaw charts or yaw bandwidth.
A positive inverse floor remains necessary for the Lipschitz extension, but a
positive physical target speed is unnecessary when its regularization
discrepancy is included in the last-row disturbance bound as above.
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
and sampled transport allowance are in the help header of
`nrmmTargetHistory.m`.
All published bounds remain centered about the original NRMM point estimates.
Neither the point-observer vector field nor its gain design changes.

This postprocessing cannot repair a violated premise of the ego observer.
In a dynamic bicycle, rear tire slip permits
\(r-v_y/l_r\ne0\). A small configured mismatch is a true-motion assumption,
not a consequence of the gyroscope or of estimated-domain consistency.
The avoidance maneuvers violate it at 15 m/s
(`report/ESTIMATOR_DOMAIN_20261005.tex`); the closed-loop runs therefore use
the force-balance measurement of `MODEL_AIDED_ESTIMATION.md`.
The simulation-only `auditNrmmTruthEnclosure` checks the true mismatch,
speed, yaw rate, sideslip and known state-component errors at logged times.
It never feeds truth to the observer or controller. Passing sampled checks
does not prove continuous-time validity; a violation invalidates application
of the corresponding conditional certificate even if all states are finite.

## Scalar sampled implementation

The runtime has one target state `[rho; q; s]` in `R^6`, one radar-output
predictor in `R^2`, one target comparison enclosure and one sensor-history
record. Together with ego velocity, position, the GNSS-output predictor and
yaw, the packed RK4 state is a fixed 15-vector. Time substeps and historical
samples remain sequences. No target axis, count or radar association step is
present. The controller receives one optional `targetEstimate` with its
state-time enclosures (`nrmmControllerErrorBounds.m`).
