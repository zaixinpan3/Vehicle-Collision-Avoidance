# Observer design and error-bound theory

These separately designed observers remain research comparators. The primary
raw-sensor dynamic output-feedback pipeline jointly optimizes controller memory
and actuation and does not invoke this observer or its gain synthesis.

This document defines the position-only GNSS/IMU filter, body-velocity and
NRMM target observers, continuous vector yaw innovation, and their error
bounds. The sensor contract consists of GNSS position, body-frame inertial
acceleration, gyro yaw rate, and relative radar position.

The continuous body/target ISS result is conditional on valid input-error
bounds. The sampled comparisons include the position/IMU filter, measurement
holding, radar resets and yaw coupling. Uniform boundedness and local yaw
contraction have separate, explicit premises; neither is an unconditional
convergence theorem for the complete interconnection.

All set containments and flows are mathematical statements in real arithmetic.
The joint prediction and recursive-feasibility theory is developed in
[the controller theory](../controller/CONTROLLER_THEORY.md#joint-prediction).

## Position-only GNSS and sensor contract

GNSS supplies position only. Inertial velocity is reconstructed from its
position history and the IMU; it is not an additional sensor measurement.
Every use of this reconstructed velocity must include its estimation error.

### Position/IMU velocity observer

Let `dt` denote the sensor period and let `(p-, v-)` be the predicted filter
state immediately before a new position observation `y`. Define

\[
q=\exp(-\omega_f\,dt),\quad \alpha=1-q^2,\quad
\beta=(1-q)^2/dt,
\]
\[
\hat p^+=\hat p^-+\alpha(y-\hat p^-),\qquad
\hat v^+=\hat v^-+\beta(y-\hat p^-).
\]

The held inertial acceleration estimate is
`aHat = R(psiHat) aMeasured`. Prediction is

\[
\hat p^-_{k+1}=\hat p^+_k+dt\hat v^+_k+\tfrac12dt^2\hat a_k,
\qquad \hat v^-_{k+1}=\hat v^+_k+dt\hat a_k.
\]

The two scalar posterior error poles are both `q`, strictly inside the unit
circle. Choose a fixed bandwidth \(\omega_f>0\).
For a past secant of duration `H` ending at initialization, endpoint velocity
error is bounded by `2 epsilon_p/H + Amax H/2`. This requires bounded position
noise and acceleration over that interval. An initial prior or such a valid
history enclosure supplies the initial error bound.

For the error enclosure, each column of a 2-by-K matrix multiplies an
independent planar unit ball. The rows represent position and velocity.
With `C=[1,0]`, `K=[alpha; beta]`, and `F=[1,dt;0,1]`, measurement updates are

\[
G^+=[(I-KC)G^-,\;K\epsilon_p].
\]

Existing signed columns are retained through `F`; their absolute values are
not taken before propagation. Acceleration mismatch is bounded by

\[
D_a\le\min\{A_{\max}+\|a_m\|,
\epsilon_a+2\|a_m\|\sin(r_\psi/2)+J_I dt\},
\qquad J_I=J_B+r_{\max}A_{\max}.
\]

The second expression applies on a smooth hold with a valid finite body
acceleration-rate envelope; otherwise use the first. At a command-induced
acceleration jump, include the additional allowance given in Section 11.4.
The two disturbance
integrals need not be the same uncertain vector, so prediction appends
independent columns `diag([dt^2/2,dt])*D_a`. Without a finite acceleration bound,
a declared finite speed domain can instead bound these integrals. The velocity
radius is the sum of absolute values in the
velocity row. This radius replaces a velocity-sensor noise constant throughout
course geometry, the current NRMM comparison bound, target measurement history,
and controller publication. Correlation is conservatively enlarged, never
used to claim independence or to reduce the bound without proof.

Past position samples supply additional valid enclosures. For an old
sample at time `t-H`, the velocity at `t` belongs to the ball centered at
`(y(t)-y(t-H))/H`, with radius

\[
 r_{\mathrm{sec}}=(\epsilon_p(t)+\epsilon_p(t-H))/H+A_{\max}H/2.
\]

Here `Amax` bounds inertial acceleration over the entire intervening interval.
This follows by integrating the difference between endpoint velocity and
average velocity. It requires no jerk bound and never averages deterministic
measurement noise down as if it were independent random noise. The triangle
inequality expresses this ball about the point velocity estimate;
the smallest of these bounds and the filter bound encloses their intersection.
When a secant tightens the enclosure, retain the old
position projection and replace the velocity projection by an independent
ball. Their Cartesian product contains the intersection. Scaling old signed
generators would incorrectly shrink retained position/velocity correlations.
This enclosure operation does not reset the point observer.

This is conditional deterministic containment in real arithmetic. It requires
valid initial, sensor, motion-domain, and yaw enclosures. Stable error poles and bounded disturbances imply
a bounded filter error; set representations must also control any outer-
approximation inflation. Neither fact implies that the radius is monotonically
decreasing, or that it is as narrow as a direct velocity sensor's error.

### Continuous yaw innovation

Let `vI` be reconstructed inertial velocity and `vB` the body-velocity
pseudo-observation of Section 3. Choose a fixed normalization scale
\(s_\psi>0\) and gain \(k_\psi>0\), and define

\[
c=\frac{v_B^T v_I}{\|v_B\|\|v_I\|+s_\psi^2},\qquad
s_c=\frac{v_{Bx}v_{Iy}-v_{By}v_{Ix}}{\|v_B\|\|v_I\|+s_\psi^2},
\]
\[
\dot{\hat\psi}=u+k_\psi(s_c\cos\hat\psi-c\sin\hat\psi).
\]

The positive scale makes the law defined at zero velocity. Its innovation
vanishes continuously with motion information; there is no speed-triggered
switch to a course-angle measurement. Section 8 gives a set-based orientation
enclosure, and Section 11 derives the error dynamics of this same innovation.
A full-circle orientation set remains valid when direction is unobservable;
it does not imply heading convergence at rest.

The target model and globally Lipschitz extension below retain the framework of
Sharma, Alai, and Rajamani, *Simultaneous ego-vehicle state estimation and vehicle
trajectory tracking using a multistage high gain observer*, Transportation
Research Part C 182 (2026), 105411,
[DOI](https://doi.org/10.1016/j.trc.2025.105411). The existing NRMM coordinate
and extension derivations are retained. The ego pseudo-measurement uses force balance with a declared vehicle-model
family and bounded sensor errors.

The vector-Jacobian analysis draws on Sharma, Zemouche, and Rajamani,
*Observer Design for Vector Nonlinear Systems With Applications to Autonomous
Vehicles and Robots*, International Journal of Robust and Nonlinear Control
(2026), [DOI](https://doi.org/10.1002/rnc.70732): Lemma 1, Theorem 1 (13),
Corollary 2 (26), and the scaling construction (33)--(35). The analytic matrix
enclosure and correlated NRMM gyro bounds below are project-specific
specializations; they preserve the observer states and vector field.

## Contents

- [NRMM model and nonlinear extension](#target-model): Sections 1--2.
- [Body-velocity pseudo-measurement and error bound](#ego-measurement): Sections 3--4.
- [Target observer, ISS cascade and gain conditions](#target-observer): Sections 5--7.
- [Continuous yaw observer and orientation sets](#yaw-output): Section 8.
- [Assumptions and scope](#scope): Section 9.
- [Reconstruction, initialization and sampled comparisons](#sampled-realization): Sections 10--11.

<a id="target-model"></a>
## 1. Nominal NRMM model and covariant chain

Let `J = [0,-1;1,0]`, `R(psi)` denote a planar rotation, and

\[
\nabla_u z=\dot z+uJz,\qquad
\rho=R_E^T(p_C-p_E),\quad q=R_E^Tv_C,\quad s=R_E^Ta_C.
\]

The nominal NRMM model has constant scalar acceleration `A` and constant
sideslip `betaC`, with positive speed. Equivalently its curvature
`kappa = sin(betaC)/lrC` is constant. This defines the nominal vector field,
not a constant-motion requirement on observed truth. Section 5 includes
bounded physical deviations as a last-row residual. The nominal transformed
equations are

\[
\nabla_{\omega_E}\rho=q-v_E,\qquad
\nabla_{\omega_E}q=s,\qquad \nabla_{\omega_E}s=\Phi(q,s),
\]
\[
V=\|q\|,\quad A=\frac{q^Ts}{V},\quad
\Omega=\frac{(Jq)^Ts}{V^2},\qquad
\Phi=-\Omega^2q+3A\Omega J\frac qV.
\]

To derive the last row, let \(e=q/V\), \(\Omega=\kappa V\), and
\(\nabla_{\omega_E}e=\Omega Je\). Constant \(A\) and \(\kappa\) give
\(\dot V=A\), \(\dot\Omega=\kappa A=A\Omega/V\), and
\[
s=Ae+V\Omega Je,\qquad
\nabla_{\omega_E}s=-V\Omega^2e+3A\Omega Je=\Phi(q,s).
\]
The factor three combines rotation of \(Ae\) and differentiation of
\(V\Omega Je\). This derivation applies on a smooth positive-speed arc;
the extension at rest is treated separately below.

These are the same NRMM dynamics in physical, covariant companion coordinates.
For example, `q = rhoDot + vE + omegaE*J*rho`. Using this chain avoids
repeated differentiation of measured ego inputs. Ego jerk and angular
acceleration need not vanish. The saturation extension below retains this nominal motion model.

Truth must remain in the declared operating domain:
`Vmin <= |q| <= b`, `|A| <= Amax`, `|Omega| <= Omax`, `|s| <= S`, and
`|rho| <= rhoMax`, where

\[
O_{\max}=b\sin(\beta_{\max})/l_r,\qquad
S=\sqrt{A_{\max}^2+(bO_{\max})^2}.
\]

For a strictly moving domain, the inverse floor is `a = Vmin > 0`.
A domain including rest instead declares `Vmin = 0` and a separate positive
`a < b`. This floor regularizes divisions; it
never replaces the Cartesian target velocity by a fictitious positive speed.
The corresponding model discrepancy is bounded in Section 2 and enters the ISS input.
The physical heading reconstruction is not observable at zero target speed.
Without informative heading history, a velocity enclosure containing zero
requires a full-circle heading enclosure and a speed interval including zero. Below the inverse
floor, reconstructed yaw-rate and sideslip errors use the physical domain caps,
because the regularized inverse is not the physical inverse there.
Estimated states may peak outside this domain: the globally Lipschitz extension
below keeps the observer defined. The operating-domain premises constrain truth, not just the estimates.

## 2. Global Lipschitz bounds for the nonlinear extension

Write `r = |q|`, `d = max(r,a)`, `Q = projection_ball(b,q)`,
`Svec = projection_ball(S,s)`, and `e = q/d`. The extension is

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

The reconstructed `Omega(q,s)` is differentiated rather than frozen. The
bounds hold over the full domain; finitely sampled Jacobian maxima do not
establish them.

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
\(\|D_q\Phi_e\|_2=\|B\|_2\le\|M_i\|_2\). A valid bound is
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

The finite-increment bounds control error growth. For gain selection,
retain the complete signed Jacobian matrices as follows.

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
information. A finite convex enclosure can retain that information through the following
compact variables:

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
eight entries. Partition each compact coordinate into finitely many intervals, intersect
the boxes with the scalar-clip conditions, and discard only boxes that cannot
meet the acceleration disk or projected boundary. Every
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
convex hull or a claim of minimum conservatism.

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
The enclosure theorem is in real arithmetic.

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
covers stationary targets and smooth low-speed arcs. Add the regularization discrepancy once to the physical last-row residual
when bounding the observer error. Inverse regularization is not an additional
physical target maneuver.

An instantaneous acceleration reset when a braking target reaches rest is a
separate hybrid transition. This finite-jerk argument does not certify that
jump or a differentiation window spanning it; it is not needed for a target
that is stationary throughout the observation and prediction interval.

<a id="ego-measurement"></a>
## 3. Ego velocity measurements

Let
\[
J=\begin{bmatrix}0&-1\\1&0\end{bmatrix},\quad
v=R(\psi_E)^T\dot p_E,\quad a=R(\psi_E)^T\ddot p_E,
\qquad \dot v=a-\omega_EJv.
\]
The reconstructed velocity, measurements and deterministic errors satisfy
\[
m_I=R(\psi_E)v+n_G,\quad u=\omega_E+n_\omega,\quad a_m=a+n_a,
\quad \|n_G\|\le\varepsilon_G,\quad |n_\omega|\le\varepsilon_\omega,
\quad \|n_a\|\le\varepsilon_a.
\]
Acceleration must have the physical meaning above; compensation, calibration,
and residual bias errors belong in its error bound. No derivative of a measured
signal or measurement error is required by the core.

The reconstructed inertial velocity `m_I` comes from the position/IMU filter;
`n_G` denotes its bounded reconstruction error, not GNSS velocity noise.
Use the declared forward cone
\[
\mathcal D_E=\{v:\underline V_E\le\|v\|\le\overline V_E,
\ v_x\ge\|v\|\cos b\},\qquad 0\le b<\pi/2.
\]
Let \(M=\|m_I\|\), \(c=\cos b\), and
\[
\phi(V,w)=\sqrt{\max\{V^2-w^2,c^2V^2\}}.
\]
On the true cone, \(\phi(\|v\|,v_y)=v_x\). The pseudo-observation and
body-velocity observer are
\[
y_v=[\phi(M,w_m),w_m]^T,\qquad
\dot{\hat v}=a_m-uJ\hat v+k_v(y_v-\hat v),\quad k_v>0.
\]
The force-balance construction below supplies \(w_m\) and a bound on its
error. The cone is a premise of this construction, not a universal consequence
of the NRMM target model.

### 3.1 Shared-model force-balance measurement

Assume the applied held steering \(\delta\), signed longitudinal-force
ratio \(\xi\in[-1,1]\), and a declared vehicle-model family are known.
The force-balance identity below applies on its stated dynamic-model domain.
A different low-speed physical model requires its own valid pseudo-measurement
enclosure; this identity must not be extended to that model without proof.

With fixed mass \(m\), axle distances \(l_f,l_r\), normal loads
\(F_{zf}=mg l_r/(l_f+l_r)\), \(F_{zr}=mg l_f/(l_f+l_r)\), and
\(v_x=\sqrt{V^2-w^2}>0\), lateral force balance is
\[
\begin{aligned}
G(w;V,r,C,\mu,\delta,\xi)
 &=\xi\mu_fF_{zf}\sin\delta
   +F_{yf}(\alpha_f)\cos\delta+F_{yr}(\alpha_r)=m a_y,\\
\alpha_f&=\operatorname{atan2}(w+l_fr,v_x)-\delta,\qquad
\alpha_r=\operatorname{atan2}(w-l_rr,v_x).
\end{aligned}
\]
For axle \(i\), let \(K_i=\mu_iF_{zi}\sqrt{1-\xi^2}\). The modified Fiala law is
\[
F_{yi}(\alpha)=
\begin{cases}
-C_i\tan\alpha\,(1-z+z^2/3),&K_i>0,\ |\tan\alpha|<3K_i/C_i,\\
-K_i\operatorname{sign}(\alpha),&\text{otherwise},
\end{cases}
\quad z=\frac{C_i|\tan\alpha|}{3K_i}.
\]
The stiffness and friction belong to the declared parameter intervals below;
mass and geometry are fixed in this inversion.

Let \(V_- = M-\varepsilon_G\), \(V_+=M+\varepsilon_G\), and assume
\(V_->0\). At a candidate lateral speed \(w\), the forward-cone speed interval is
\[
V\in[\max\{V_-,|w|/\sin b\},V_+],\quad
r\in[u-\varepsilon_\omega,u+\varepsilon_\omega],\quad
(C_i,\mu_i)\in[(1-\eta)(C_{i0},\mu_{i0}),
                 (1+\eta)(C_{i0},\mu_{i0})].
\]
Here \(0\le\eta<1\) is a declared uncertainty bound; at \(b=0\) the
cone forces \(w=0\). Let \(G_{\min}(w),G_{\max}(w)\) be valid lower
and upper force bounds over the entire admitted parameter, speed and yaw-rate
set. Separate per-axle extrema give an outer bound even when those variables
are shared. A reduction to corner extrema requires the corresponding
monotonicity on the whole admitted branch.
Every admissible lateral speed must satisfy
\[
G_{\min}(w)\le m(a_{m,y}+\varepsilon_a),\qquad
G_{\max}(w)\ge m(a_{m,y}-\varepsilon_a),\qquad |w|\le V_+\sin b.
\]
Let \([w_-,w_+]\) contain all lateral speeds satisfying these inequalities.
An empty feasible set invalidates the assumed measurement/model enclosure.
Choose a fixed nominal force law \(G_0\) and a declared positive
regularization coefficient \(\rho(M)>0\). On an admitted monotone branch,
let \(w_m\) solve the regularized nominal equation
\[
G_0(w_m)-\rho(M)w_m=m a_{m,y},
\]
with endpoint saturation on \([-M\sin b,M\sin b]\) when necessary.
The regularization specifies the point estimator; it does not change the
physical force law or the feasible inverse set. Its error is bounded by
\[
\rho_w=\max\{|w_m-w_-|,|w_+-w_m|\},\qquad
 y_v=[\phi(M,w_m),w_m]^T.
\]
A unique physical inverse is unnecessary. The point estimate need not lie
inside the feasible inverse interval for this radius to be valid.

Near saturation the force slope can vanish: a unique inverse is unnecessary,
and the interval widens rather than supplying an unjustified small radius.
If speed information is uninformative, retain the full valid physical-cone
projection and bound its distance from the point estimate. A small inverse
radius cannot be inferred from a saturated force map.

## 4. Body-velocity error dynamics and certificate

On the unfloored branch,
\((\partial_V\phi,\partial_w\phi)=(\sec\theta,-\tan\theta)\),
where \(\sin\theta=w/V\) and \(|\theta|\le b\). On the floor branch
the derivatives are \((\cos b,0)\). Integrating these bounded derivatives
along a segment, including branch crossings and zero by continuity, yields
the finite-error bounds below.

### 4.1 Ego certificate for the force-balance measurement

Use the observer and force-balance pseudo-measurement of Section 3. For \(|w_m-v_y|\le\rho_w\), the global derivative bounds of \(\phi\)
give, even across the floor branch,
\[
|y_x-v_x|\le\sec b\,\varepsilon_G+\tan b\,\rho_w,
\quad |y_y-v_y|\le\rho_w,
\]
\[
\|y_v-v\|^2\le(\sec b\,\varepsilon_G+\tan b\,\rho_w)^2+\rho_w^2
 \le\sec^2 b\,(\varepsilon_G+\rho_w)^2.
\]
The last inequality uses \(\tan b\le\sec b\) and
\(1+\tan^2b=\sec^2b\). Substitution gives
\[
\dot e_v=(-k_vI-uJ)e_v+n_a-n_\omega Jv+k_v(y_v-v),
\]
\[
D^+\|e_v\|\le-k_v\|e_v\|+d_v^{\rm model},\qquad
 d_v^{\rm model}=\varepsilon_a+\varepsilon_\omega\overline V_E
              +k_v\sec b(\varepsilon_G+\rho_w).
\]
No independence is assumed between the force inversion and its sensor inputs.
This bound separates their effects conservatively. Shared sensor errors
remain common sources in any joint set propagation.
If \(\rho_w\le\bar\rho_w\) uniformly, its ultimate radius is
\[
U_v^{\rm model}=\varepsilon_a/k_v+
 \varepsilon_\omega\overline V_E/k_v+\sec b(\varepsilon_G+\bar\rho_w).
\]
With a varying radius use the time-dependent comparison input instead of a
constant ultimate value. Section 11 gives the held-measurement version.

<a id="target-observer"></a>
## 5. Target observer and position-normalized certificate

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
All components of \(z_T\) have position units.

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

The dissipation premise requires every vertex of the analytic enclosure.
A feasible metric for a subset of vertices alone does not establish it.
Section 7 gives a gain-selection objective within the certified family.

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
of the two valid bounds remains valid. The maximizer of the relaxed box need not belong to that intersection.
General initial errors do not satisfy \(q^Ts=VA\), so their metric
bounds must cover the full admitted initial-error set.
Radar noise likewise remains one shared
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
Here \(d_v=d_v^{\rm model}\) is the forcing bound of Section 4.1.
For uniform forcing, use its ultimate radius \(U_v^{\rm model}\).
The Hurwitz Metzler matrix admits positive backward weights
\(w=-H^{-T}\mathbf1\), so \(w^TH=-\mathbf1^T\). Consequently
\(D^+(w^T\chi)\le-(w^T\chi)/\max(w)+w^Td\).
Given \(B_0\ge\chi(t_0)\), its nonnegative transition matrix gives
\[
\chi(t)\le e^{H(t-t_0)}B_0+\int_{t_0}^t e^{H(t-\tau)}d(\tau)\,d\tau,
\qquad W_{T,\infty}\le(c_{Tv}U_v^{\rm model}+\bar d_T)/\lambda_T.
\]
Initial component radii give a metric bound using
\(D=\operatorname{diag}(1,\omega_T^{-1},\omega_T^{-2})\):
\(W_T(t_0)\le\sqrt{(Db_{T0})^T|P|(Db_{T0})}\).
A steady-state radius does not certify arbitrary initial errors.

The gyro and reconstructed velocity are shared sources across the stages.
The triangular norm comparison conservatively separates their effects.
A joint uncertainty propagation must retain their common realization before
any support or norm bound; it must not introduce independent copies of the
same physical noise or parameter error.

## 7. Explicit gain-selection preference and position output

A gain-selection objective is the following semidefinite problem in the physical
gains \(L\in\mathbb R^3\), where
\(A=[0,1,0;0,0,1;0,0,0]\) is the companion chain. With \(A_L=A-Le_1^T\) and \(Y=PL\) it minimizes
\(\gamma\) subject to the dissipation condition of Section 5 written in
physical coordinates (\(\omega_T=1\)) at a declared positive decay
rate \(\lambda_T\), and
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

The bandwidth and positive body-velocity gain are design parameters.
Any selected gains must satisfy the stated dissipation and sampling conditions.
A performance objective does not replace these conditions or establish a
small deterministic error bound by itself.

The inertial position estimate is driven by reconstructed velocity,
\[
\dot{\hat p}_E=m_I+k_p(\hat y_p-\hat p_E),\qquad k_p>0,
\]
where the output predictor resets to each GNSS position and obeys
\(\dot{\hat y}_p=m_I\) between samples. Equation (S8) gives its error
bound, including the shared GNSS noise and velocity-reconstruction error.

<a id="yaw-output"></a>
## 8. Parallel yaw observer and its orientation certificate

Use the continuous dot/cross innovation defined at the start of this document.
Its error equation and conditional contraction bound are (S6)--(S7); no
heading-measurement feedback switch is used.
Orientation-set containment does not require that local contraction result.

For the force-balance branch, enclose \(\sin\beta_E=w/V\) using all
endpoint ratios of \([w_-,w_+]\) and \([V_-,V_+]\), then intersect with
\([-\sin b,\sin b]\). With \(V_->0\), apply \(\arcsin\) to obtain
\([\beta_-,\beta_+]\), set
\[
\hat\beta=\arcsin\!\left(\operatorname{clip}_{\sin b}(w_m/M)\right),\quad
h_m=\arg m_I-\hat\beta,\quad
\varepsilon_h=\arcsin(\varepsilon_G/M)
            +\max_{\beta\in[\beta_-,\beta_+]}|\beta-\hat\beta|.
\]
Here \(h_m\) is only the center of a containing orientation arc, not a
measurement substituted into the continuous yaw feedback law.
The reconstructed-velocity direction term applies when \(M>\varepsilon_G\). Inconsistent or
uninformative inversion supplies no proper heading arc. For a general set formulation, let \(\mathcal V_m\) enclose the true
body velocity consistently with the force and speed observations. Define
\[
\mathcal Y_m=\{\psi\in S^1:\exists v\in\mathcal V_m,
\ \|m_I-R(\psi)v\|\le\varepsilon_G\}.
\]
A containing course-correspondence arc is also admissible. Uninformative
velocity data contribute \(S^1\). The orientation set propagates and intersects
\[
\Psi^- =\operatorname{wrap}\bigl(\Psi(t_0)\oplus\{\int u\}
 \oplus[-\varepsilon_\omega(t-t_0),\varepsilon_\omega(t-t_0)]\bigr),
\qquad \Psi=\Psi^-\cap\mathcal Y_m.
\]
Retain all disconnected components of a circular intersection. For a sampled held gyro, the integral-error radius includes the hold
uncertainty; the sensor error alone cannot certify an arbitrary intersample rate.
Empty intersections stay empty and invalidate orientation outputs. They never
reset the set to an arc center or invalidate an otherwise consistent body core.

At output use the continuous observer estimate and enclose the set about it:
\[
\psi_{\rm out}=\hat\psi_E,\qquad
B_\psi=\sup_{\theta\in\Psi}d_{S^1}(\hat\psi_E,\theta).
\]
The supremum includes every retained component. The whole circle has radius
\(\pi\) about every estimate. An empty set invalidates the enclosure;
it cannot be replaced by a new arbitrary arc. A smallest covering arc centered
elsewhere does not give an error radius about \(\hat\psi_E\).

For a body vector with error radius \(E_z\), publish
\[
\hat z^I=R(\psi_{\rm out})\hat z,\qquad
\mathcal Z^I=\bigcup_{\psi\in\Psi}R(\psi)(\hat z+\mathbb B_{E_z}).
\]
Using also the true norm bound \(\|z\|\le\bar z\), a certified error radius is
\(E_z+2\min(\|\hat z\|,\bar z)\sin(B_\psi/2)\).
The true norm bound can further intersect each body ball; this scalar radius
need not enclose points in the unrestricted union that violate that bound.
Absolute target position adds the ego-position uncertainty. This orientation
set certifies error about the chosen estimate without asserting convergence
of that estimate.

<a id="scope"></a>
## 9. Scope

Core ISS requires a valid forward cone, sensor/reconstructed-input error
bounds, a true target operating domain, positive gains and the target
dissipation inequality. The force-balance pseudo-measurement additionally
requires its physical model family, known applied input and a valid inverse
set on the admitted domain. The positive inverse floor in the NRMM extension
does not require strictly positive physical target speed when its discrepancy
is included in the residual bound.

The position-only sensor contract introduces coupling through yaw and the
position/IMU filter. Section 11 gives a conditional bounded-input cascade for
this interconnection and a separate local yaw result. Neither the continuous
core ISS inequality nor a valid orientation set proves global exponential
convergence of the enlarged observer. Sampled containment and closed-loop
collision avoidance are distinct conclusions.

<a id="sampled-realization"></a>
## 10. Target reconstruction, history and initialization

At positive admissible target speed, the observer state gives the physical
parameters directly:
\[
V_T=\|q\|,\quad A_T=q^Ts/V_T,\quad
\Omega_T=(Jq)^Ts/V_T^2,\quad
\beta_T=\arcsin(l_{rC}\Omega_T/V_T),
\]
\[
\theta_{\rm course}^I=\hat\psi_E+\arg q,\qquad
\theta_C^I=\theta_{\rm course}^I-\beta_T.
\]
Outside the invertible domain, retain physical parameter bounds; a finite
regularized point value does not establish observability. Initial and current
component enclosures can be intersected with valid bounded-noise historical
measurement consequences. For example, if position
measurements \(p_i^m\) at \(t_i<t_j\) have radii \(r_i,r_j\), acceleration is
bounded by \(a_{\max}\), and \(\Delta=t_j-t_i\), then
\[
\left\|v(t_j)-\frac{p_j^m-p_i^m}{\Delta}\right\|
 \le\frac{r_i+r_j}{\Delta}+\frac{a_{\max}\Delta}{2}.
\]
A later output time adds \(a_{\max}(t-t_j)\). Gyro
transport retains the common heading error across samples. Deterministic
sensor noise is not assumed to average down. Intersections only tighten
radii about the original NRMM point estimates; they do not reset observer
states or alter gains. Inconsistent intersections cannot establish containment.

A persistent constant-motion parameter family belongs to the controller's
forecast contract, not to the standalone observer's assumptions on truth.
The observer retains the last-row model residual and only intersects history
constraints justified by its declared motion bounds. See
[the joint prediction theory](../controller/CONTROLLER_THEORY.md#joint-prediction)
for future conditioning and shared target parameters.

A valid transient certificate starts from an explicit prior or a full
physical-domain enclosure. A point fit alone proves neither a small initial
error nor convergence. Historical observations may intersect the initial set
only through valid measurement and motion bounds; delayed acquisition does
not justify treating unknown initial states as exact.

## 11. Sampled observer and error comparisons

The sampled observer processes position, IMU, gyro and radar readings at their
declared event times. Its held measurements and output predictors are part of
the mathematical state. Continuous-time core decay alone does not establish
sampled stability; the reset and aging effects below must also be included.

### 11.1 Held ego measurements

At age \(\tau\) after a sample, let \(a_{\rm env}\) bound true acceleration,
\(\epsilon_\omega(\tau)\) bound held-gyro error, and
\(\epsilon_a(\tau)\) bound held-acceleration error. The declared acceleration
and yaw-acceleration envelopes control their growth; sensor-error bounds at
the sample alone cannot do so. For the force-balance branch set
\[
\begin{aligned}
d_S&=\min\{\overline V_E+M,\varepsilon_G+a_{\rm env}\tau\},\\
d_W&=\rho_w+
 [a_{\rm env}+(|u|+\epsilon_\omega(\tau))
            \min\{\overline V_E,M+d_S\}]\tau.
\end{aligned}
\]
Since \(|\dot V|\le\|a\|\) and
\(|\dot v_y|=|a_y-\omega_Ev_x|\le\|a\|+|\omega_E|\|v\|\),
\[
\|y_v(t_k)-v(t)\|\le\sec b(d_S+d_W),\qquad
 d_v=\epsilon_a(\tau)+\epsilon_\omega(\tau)\overline V_E
        +k_v\sec b(d_S+d_W).
\]
### 11.2 Predictor, correction and dropout comparisons

The radar predictor \(\hat y\) resets to a detection and between updates obeys
\(\dot{\hat y}=\hat q-\hat v-uJ\hat y\). Its error to true \(\rho\) is
bounded by \(B_y\), with reset \(B_y\le\varepsilon_\rho\). When correction
is enabled, the observer injects \(\hat y-\hat\rho\). For
\(c_q=\omega_T\sqrt{(P^{-1})_{22}}\), a valid comparison is
\[
D^+\begin{bmatrix}B_v\\W_T\\B_y\end{bmatrix}\le
\begin{bmatrix}-k_v&0&0\\c_{Tv}&-\lambda_T&c_{T\rho}\\1&c_q&0\end{bmatrix}
\begin{bmatrix}B_v\\W_T\\B_y\end{bmatrix}
+\begin{bmatrix}d_v\\c_{T\omega}\epsilon_\omega+c_{Tj}\varepsilon_j\\
 \bar\rho\epsilon_\omega\end{bmatrix}.
\]
The additional predictor state prevents interpreting the continuous two-stage
Hurwitz comparison as an unconditional sampled stability result. Finite-time propagation of this comparison does not require this matrix
to be Hurwitz.

During radar dropout the innovation is disabled. The comparison then uses
component radii \((B_\rho,B_q,B_s)\), with no negative target decay term:
\[
D^+B_\rho\le B_v+B_q+\bar\rho\epsilon_\omega,\quad
D^+B_q\le B_s+\bar V_T\epsilon_\omega,
\]
\[
D^+B_s\le L_qB_q+L_sB_s+\bar a_T\epsilon_\omega
                  +\varepsilon_j.
\]
The predictor retains its corresponding growth bound. Indefinite dropout
does not inherit the continuous correction-mode convergence rate.

### 11.3 Interface with joint prediction

Current observer enclosures initialize the nonlinear joint construction in
[the controller theory](../controller/CONTROLLER_THEORY.md#recursive-feasibility).
That construction carries the physical vehicles, observers, held readings,
measurement resets and causal controls on the same uncertainty branch.
Persistent model parameters and sensor errors each enter once. Adding an
independent observer-error source after propagating their effects would count
the same uncertainty twice.

Whole-block measurement conditioning and restriction of the old policy tail
supply the cross-frame inclusion used in the controller proof. A current-state
observer error bound alone is not a collision-avoidance theorem, nor does it
imply a constant-width prediction tube.

<a id="compact-sampled-comparison"></a>
### 11.4 Compact sampled comparison without future measurement simulation

This subsection derives a **prediction interface**, not a replacement for the
observer defined above. Future noisy readings need not be generated individually to
bound the observer's error. They can be quantified over in a sampled error
map. The actual observer still processes every received reading. The target's
constant-motion contract belongs to controller prediction; the standalone
observer retains its model-residual allowance for other admissible motion.

All norms below use fixed, declared coordinate scalings. The derivation uses
exact flows and resets. Bounds are uniform over the represented physical
states, observer memories, parameters, and sensor errors, not just one
nominal sensor history.

**Position/IMU filter.** Let \(\Delta>0\) be one sensor period. For the
prior-to-prior error \(z_f=[e_p^-;e_v^-]\), use the filter's matrices
\[
 F=\begin{bmatrix}1&\Delta\\0&1\end{bmatrix},\quad C=[1,0],\quad
 K=\begin{bmatrix}1-q_f^2\\(1-q_f)^2/\Delta\end{bmatrix},\quad
 q_f=e^{-\omega_f\Delta},\quad A_f=F(I-KC).
\]
Correction followed by prediction gives the exact identity
\[
 z_{f,j+1}^-=(A_f\otimes I_2)z_{f,j}^-
 +(FK\otimes I_2)\nu_p+
 \int_0^\Delta
   \left(\begin{bmatrix}\Delta-t\\1\end{bmatrix}\otimes I_2\right)
       \delta a(t)\,dt.                                      \tag{S1}
\]
Here \(\delta a(t)=\hat a_j-a_I(t_j+t)\) is the difference between the
held inertial acceleration estimate and the true acceleration. It includes
yaw error, accelerometer error, and within-hold acceleration change. In
particular it is not an independent second copy of plant mismatch.
The polynomial of \(A_f\) is \((s-q_f)^2\), so it is Schur even though
its Euclidean norm can exceed one. After scaling position and velocity,
let \(P_f>0\) solve \(A_f^TP_fA_f-P_f=-I\). Define
\[
 B_f=\|z_f^-\|_{P_f\otimes I_2},\quad
 \rho_f=\sqrt{1-1/\lambda_{\max}(P_f)}<1.
\]
The induced-norm inequality and the triangle inequality for the integral give
\[
 B_f^+\le \rho_f B_f+
    \|P_f^{1/2}FK\|\varepsilon_p+
 \int_0^\Delta
    \left\|P_f^{1/2}\begin{bmatrix}\Delta-t\\1\end{bmatrix}\right\|
      D_a(t)\,dt.                                             \tag{S2}
\]
With nontrivial scaling, all input columns in (S1)--(S2) are transformed by
the same scaling. No measurement independence assumption is used. The single
GNSS primitive acts on both position and velocity through \(FK\); the two
acceleration integrals are bounded as one operator on a time-varying input,
not identified as the same uncertain vector. Component output bounds follow
from \(\|Lz_f\|\le\|L(P_f^{-1/2}\otimes I_2)\|B_f\).
At a posterior output, first apply \(I-KC\) and the same position-noise
column; do not confuse prior and posterior radii.

The global physical bound \(D_a\le A_{\max}+\|\hat a_j\|\) is available
without yaw contraction. The sharper bound given at the start of this
document uses a valid yaw radius and acceleration aging. Thus bounded
forcing yields a bounded filter enclosure with no future-readings rollout.
This result alone establishes neither yaw observability nor small total
controller uncertainty.

The sensor/control event order matters here. If the boundary IMU sample
precedes a newly applied control, acceleration may jump even with continuous
vehicle state. A finite within-mode jerk bound cannot cover that jump. With
\(J_u\) bounding this acceleration change and \(J_I\) the subsequent
smooth aging bound, a valid refinement is
\[
 D_a(t)\le\min\{A_{\max}+\|a_m\|,
 \varepsilon_a+2\|a_m\|\sin(B_{\psi,j}/2)+J_u+J_I t\}.
\]
The second member requires all of these bounds; otherwise use the first.
There is no \(J_u\) contribution at sensor-only boundaries without an input
change. This is an effect of the existing shared command and model, not a
new independent disturbance. Any use of a jerk-only bound across an input
switch must establish acceleration continuity or add the jump allowance.

**Radar reset and the body/target core.** In correction mode write the
matrix of Section 11.2 as \(M\), its nonnegative forcing as \(b(t)\), and
let \(E=[I_2,0]\), \(e_3=[0,0,1]^T\). Immediately after every guaranteed
detection, the radar predictor error is bounded by \(\varepsilon_\rho\).
Consequently \(e_c=[B_v,W_T]^T\) satisfies
\[
 e_c^+\le A_\Delta e_c+d_\Delta,\quad
 A_\Delta=E e^{M\Delta}E^T,\quad
 d_\Delta=E e^{M\Delta}e_3\varepsilon_\rho+
          \int_0^\Delta E e^{M(\Delta-t)}b(t)\,dt.             \tag{S3}
\]
This is a two-dimensional sampled map; the third radius is needed inside
the hold but is reset, rather than carried as a free state across detections.
It does not reset either actual target-observer error or body-velocity error.
State-dependent forcing must be bounded over the whole intervening hold.

There is a constructive small-period stability result for the homogeneous
map in (S3). Set
\[
 H=EME^T=\begin{bmatrix}-k_v&0\\c_{Tv}&-\lambda_T\end{bmatrix},
 \quad w=-H^{-1}{\bf1}>0,\quad \mu=\|M\|_\infty,\quad W=\|w\|_\infty.
\]
Since \(M\) is Metzler, \(A_\Delta\ge0\). The exponential remainder gives
\[
 A_\Delta w=w-\Delta{\bf1}+r_\Delta,\qquad
 \|r_\Delta\|_\infty\le W(e^{\mu\Delta}-1-\mu\Delta).
\]
For \(0\le x<1\), the exponential series gives
\(e^x-1-x\le x^2/[2(1-x)]\). Thus the explicit sufficient condition
\[
 0<\Delta\le\frac1{\mu(1+\mu W)}                           \tag{S4}
\]
implies \(\mu\Delta<1\) and
\(\|r_\Delta\|_\infty\le W\mu^2\Delta^2/[2(1-\mu\Delta)]
\le\Delta/2\). Consequently
\[
 A_\Delta w\le
 \left(1-\frac{\Delta}{2\max_j w_j}\right)w<w.
\]
These inequalities apply after fixed state scaling. With dimensionless
scaled states and physical time, \(\mu\) has inverse-time units and
\(w,W\) have time units; (S4) is dimensionally consistent. Changing units
without transforming the matrices is invalid. No assertion that the
three-dimensional continuous matrix \(M\) is Hurwitz is needed. Equation
(S4) is sufficient and conservative, not a necessary threshold or a
necessary sampling restriction.
For bounded \(d_\Delta\), a positive weighted maximum norm now yields an
ultimate bound on this core. However, \(b\) includes frontend, force-inverse,
yaw, aging, and model-residual bounds. Their joint interconnection must still
be bounded. Neither (S4) nor the continuous two-state certificate proves that
enlarged claim. Unbounded detection gaps invalidate (S3); use the dropout
map, or an explicitly bounded-gap composition, in that case.

**An exact, sharper elimination of radar-predictor memory.** The common
rotation and common ego-velocity error permit more than separate norm bounds.
In the notation (J5)--(J6) of the controller theory, let
\(e_T=[e_\rho;e_q;e_s]\), \(C_r=[1,0,0]\),
\(a=[a_1,a_2,a_3]^T\), and
\[
 A_0=\begin{bmatrix}0&1&0\\0&0&1\\0&0&0\end{bmatrix},\qquad
 B_\Delta=\int_0^\Delta e^{A_0(\Delta-t)}a e^{-a_1t}\,dt.
\]
On a hold with gyro sample \(\omega_j\), rotation by
\(I_3\otimes R(\omega_jt)\) removes the common skew term. Variation of
constants then gives the exact identity
\[
\begin{split}
 e_T(\Delta)=(I_3\otimes R(-\omega_j\Delta))\bigg\{
 &[(e^{A_0\Delta}-B_\Delta C_r)\otimes I_2]e_T(0)
 +(B_\Delta\otimes I_2)\nu_\rho\\
 &+\int_0^\Delta(e^{A_0(\Delta-t)}\otimes I_2)f_R(t)\,dt
 \bigg\}.
\end{split}
\tag{S5}
\]
Here \(f_R=(I_3\otimes R(\omega_jt))f\), with
\[
 f=\begin{bmatrix}
 -e_v-w_\omega J\rho\\
 -w_\omega Jq\\
 \Phi_e(\hat q,\hat s)-j_{\rm phys}-w_\omega Js
 \end{bmatrix},\qquad w_\omega=\omega_j-\omega_E(t).
\]
There is no additional innovation forcing: it has already produced the
\(-B_\Delta C_r\) and \(B_\Delta\nu_\rho\) terms. The same gyro
primitive enters all displayed rows before any norm is taken. The target
nonlinearity remains in \(f_R\); it cannot be discarded or treated as
independent of \(e_T\). Its whole-domain Lipschitz/residual bounds can be
closed with a comparison inequality. Unlike simulating future observers,
the polynomial-exponential kernels in (S5) require no future measurement
samples. At a target stop, include the stated acceleration-error jump and
compose the two flows. Equation (S5) without that jump is only an identity
on a continuous-mode interval. It supplies an optional sharper enclosure,
not an automatic improvement in every metric.

This elimination depends on the exact shared non-injection terms in (J3):
the predictor and target position observer use the same held gyro and body
velocity, giving \(\dot\lambda=-(a_1I+\omega_jJ)\lambda\).
If a realization adds a discrepancy \(g\) to that equation, the braces in
(S5) additionally contain
\(\int_0^\Delta(B_{\Delta-t}\otimes I_2)R(\omega_jt)g(t)dt\).
Neither innovation saturation nor inconsistent sensor holds can be silently
included in the exact-cancellation claim. Since \(A_0^3=0\), the kernel
\(e^{A_0t}=I+tA_0+t^2A_0^2/2\) is polynomial; its convolution with
\(e^{-a_1t}\) consists only of scalar exponential moments.

**Continuous vector yaw correction.** Denote the normalized dot/cross pair
in the current yaw law explicitly by
\[
 Q(x,y)=\frac{[x^Ty,\det(x,y)]^T}{\|x\|\|y\|+s_\psi^2}.
\]
The normalization is essential: the ideal equal-speed identity alone does
not bound this map on reconstructed velocities of unequal magnitudes.
The displayed product normalization, rather than a different squared-speed
denominator, satisfies \(\|Q\|\le1\). For true velocity pairs,
\[
 Q(v_B,R(\psi)v_B)=a_v[\cos\psi,\sin\psi]^T,\qquad
 a_v=V^2/(V^2+s_\psi^2)\in[0,1].
\]
If the actual held pair obeys
\(\|\hat Q-Q(v_B(t),v_I(t))\|\le\varepsilon_Q(t)\), then
\[
 \dot e_\psi=w_\omega-k_\psi a_v\sin e_\psi+\eta_\psi,
 \qquad |\eta_\psi|\le k_\psi\varepsilon_Q.
\]
On a valid angular chart \(|e_\psi|\le b_\psi<\pi\),
\[
 D^+|e_\psi|\le
 -k_\psi\underline a_v\operatorname{sinc}(b_\psi)|e_\psi|
 +\varepsilon_\omega+k_\psi\varepsilon_Q.                    \tag{S6}
\]
The chart/domain must be established over the interval, for example by
checking that the comparison solution stays below \(b_\psi\).
For \(s_\psi>0\), a simple uniform perturbation bound is
\[
 \varepsilon_Q\le\min\left\{2,
 \frac{2\overline V_I}{s_\psi^2}B_B+
 \frac{2\overline V_B}{s_\psi^2}B_I\right\},                       \tag{S7}
\]
where the velocity domains also enclose the line segments used in the
finite-difference estimate. This follows from \(\|Q\|\le1\) and the
partial Lipschitz constants \(2\overline V_I/s_\psi^2\) and
\(2\overline V_B/s_\psi^2\). Here \(B_B\) bounds the held body-velocity
pseudo-measurement, including force-inversion uncertainty, and \(B_I\)
bounds held reconstructed inertial velocity, including filter uncertainty
and aging. They are not both the body-velocity observer radius \(B_v\).
Equation (S7) is a safe coarse bound, potentially very conservative for
small \(s_\psi\). Shared-vector geometry can give a tighter bound.

At rest \(\underline a_v=0\); no yaw-contraction claim follows. A chart-free conservative circular radius uses
\[
 B_\psi^+\le\min\{\pi,
 B_\psi+\int_0^\Delta(\varepsilon_\omega+k_\psi\varepsilon_Q)dt\}.
\]
The fixed positive scale \(s_\psi\) keeps this normalization defined at
zero motion. Coupling to the position/IMU filter changes the forcing and any
joint stability test; it does not invalidate (S6) when its input bounds hold.
The orientation-set enclosure remains available independently of contraction.

**Published ego position.** This position estimate is distinct from the
position/IMU filter's internal position. Its GNSS output predictor resets
to \(p+\nu_p\), then integrates the same held inertial velocity \(\hat v_I\)
as the position estimate. Their difference obeys
\(\dot{(\hat y_p-\hat p)}=-k_p(\hat y_p-\hat p)\). Therefore
\[
 e_p^+=e^{-k_p\Delta}e_p+(1-e^{-k_p\Delta})\nu_p
       +\int_0^\Delta(\hat v_I-v_I(t))dt,\qquad
 B_p^+\le e^{-k_p\Delta}B_p+(1-e^{-k_p\Delta})\varepsilon_p
       +\int_0^\Delta B_I(t)dt.                              \tag{S8}
\]
The position noise in (S8) is the same reading used in (S1), not a new
physical noise source.

**A bounded observer interface without full yaw contraction.** There is a
useful distinction between a uniform finite bound and a useful small bound.
Suppose the declared physical domain bounds acceleration, velocity, yaw-rate
aging, target model residuals and stop jumps; position/radar/IMU errors are
bounded; detections occur at the assumed periods; and the force-inversion
output is bounded on the resulting measurement domain. Suppose also that
the selected sampled core satisfies \(A_\Delta w\le\gamma_c w\) with
\(0\le\gamma_c<1\). One sufficient test is (S4).

Using the coarse yaw-independent acceleration bound in (S2), let
\(d_f\) uniformly bound its two input terms. Then
\[
 B_{f,j}\le\rho_f^j B_{f,0}
          +\frac{d_f(1-\rho_f^j)}{1-\rho_f}
 \le\bar B_f:=\max\{B_{f,0},d_f/(1-\rho_f)\}.
\]
This supplies a finite held-velocity bound \(\bar B_I\), including aging.
Together with the declared force-inverse and physical-domain bounds it gives
a finite bound \(\bar d_c\) on \(\|d_\Delta\|_w\) in (S3), where
\(\|e_c\|_w=\max_j |e_{c,j}|/w_j\). Hence
\[
 \|e_{c,j}\|_w\le\gamma_c^j\|e_{c,0}\|_w+
             \bar d_c(1-\gamma_c^j)/(1-\gamma_c).
\]
Equation (S8) similarly gives, with \(a_p=e^{-k_p\Delta}<1\),
\[
 B_{p,j}\le a_p^j B_{p,0}+
 (1-a_p^j)\left(\varepsilon_p+
                 \frac{\Delta\bar B_I}{1-a_p}\right).
\]
Finally the circular yaw-error radius is at most \(\pi\). Thus this
observer interface has a horizon-independent finite bound under the stated
domain and sampling conditions, without a uniform yaw-observability premise.
The argument is a bounded-input cascade: the first filter bound does not
require the other observer errors to contract. This establishes boundedness,
not an accurate heading at rest, a small collision reserve, or bounded
physical target-position spread. A full-circle yaw bound and the coarse
force-inverse allowance can be too conservative for control. Tighter
coupled bounds require the corresponding valid geometry or small-gain
condition. The sampling premise must be established for the selected gains.

At synchronized detected-sample boundaries these results give a possible
five-radius observer interface \((B_f,B_v,W_T,B_\psi,B_p)\).
Target orientation reconstruction, target-stop jumps, asynchronous phases,
or other policy memory may require additional bounds. Five is not a claim
of an exact minimal state representation. The comparison has to cover every
memory allowed by its certificate. Intersample predictors can be eliminated
only because their reset and subsequent error equations were included above.
The full controller construction and its cross-frame requirement are given in
[the compact-certificate theorem](../controller/CONTROLLER_THEORY.md#compact-certificate).

### 11.5 Constructive nonlinear forcing bounds

The homogeneous sampled matrices in (S3) and (S5) do not by themselves bound
their forcing. Whole-domain physical and observer remainders are supplied by
[the nonlinear transition construction](../controller/CONTROLLER_THEORY.md#compact-constructive-enclosure).
That construction uses the Fiala force law, finite force increments, valid
physical-flow enclosures and the same sensor event order as the observer.
The physical acceleration generates both the IMU observation and its
prediction discrepancy. At a command change, an earlier IMU sample retains
the old command's acceleration while the new command drives the subsequent
plant flow. Persistent tire parameters remain shared throughout.

The exact radar innovation identity
\[
\lambda=\hat y-\hat\rho,\qquad
\dot\lambda=-(a_1I+uJ)\lambda
\]
eliminates free predictor memory between detections. In the coordinates
\((\hat v,\hat\rho,\hat q,\hat s,\lambda)\), the remaining forcing
includes the explicit nonlinear NRMM map. Its complete remainder, the
nonlinear yaw correction, position/IMU updates and measurement resets must
all be enclosed; homogeneous contraction alone is insufficient.

Composition on the shared sensor clock supplies a nonlinear observer image.
If detection status is uncertain, retain every admissible detection/no-detection
continuation. These bounds establish containment under their premises.
A small uncertainty set or a collision-avoidance conclusion requires the
additional controller conditions, not just this observer image.
