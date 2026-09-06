# Multistage high-gain NRMM observer: model, design, and stability

The estimator retains the framework of Sharma, Alai, and Rajamani,
*Simultaneous ego-vehicle state estimation and vehicle trajectory tracking using
a multistage high gain observer*, Transportation Research Part C 182 (2026),
105411, [DOI](https://doi.org/10.1016/j.trc.2025.105411). The relevant parts are
Section 2.3, Section 3.3, Eq. (65), and Theorem 1, Eqs. (30)--(33).

The online architecture is the ego yaw observer, the ego body-velocity observer,
and the third-order target observer. Inertial position is a downstream output
stage. The target state remains `[rho; q; s]`; its gains remain
`[l1*w; l2*w^2; l3*w^3]`. The improvements are analytic global Lipschitz bounds,
a structured Lyapunov inequality, and a noise-aware choice of bandwidth.

The analysis treats the plant and observer as continuous-time systems.
Measurement-error and model-error bounds apply throughout the interval under
consideration. The results concern solutions of the differential equations
below under their stated operating-domain and yaw-chart assumptions.

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

## 3. Observer equations and the yaw chart condition

The continuous measurement model is
`u3 = omegaE + dOmega`, `am = aE + na`,
`vm = R(psiE)*vE + nv`, and `yR = rho + nR`, with the declared uniform norm
bounds. The corrected GNSS-course correspondence supplies `yF` with a certified
circular error radius `BF`. Its construction uses the declared single-track
mismatch bound; it is not an additional zero-sideslip assumption.

The retained ego observers are

\[
\dot{\hat\psi}=u_3+k_\psi\operatorname{wrap}(y_F-\hat\psi),
\qquad \nabla_{u_3}\hat v=a_m+k_v(R(\hat\psi)^Tv_m-\hat v).
\]

Let `ePsi = psiE - psiHat` be a compatible lift and `dF = yF - psiE`.
The linear comparison for yaw requires `|ePsi| + BF < pi`. A sufficient invariant
condition is

\[
B_\psi=\max\{|e_\psi(0)|,B_F+\bar d_\omega/k_\psi\},
\qquad B_\psi+B_F<\pi.
\]

Inside this chart, `wrap(ePsi+dF)=ePsi+dF`, so the scalar comparison and a
first-exit argument preserve the condition. A proper measurement arc alone is
insufficient: `ePsi=3`, `dF=0.2`, `kPsi=1` yields a positive derivative of
`|ePsi|`, contradicting an unrestricted linear-decay claim. The initial true-error bound and the invariant chart condition are explicit
premises of the continuous-time result.

The target observer is exactly the third-order high-gain chain

\[
\begin{aligned}
\nabla_{u_3}\hat\rho&=\hat q-\hat v+l_1\omega(y_R-\hat\rho),\\
\nabla_{u_3}\hat q&=\hat s+l_2\omega^2(y_R-\hat\rho),\\
\nabla_{u_3}\hat s&=\Phi_e(\hat q,\hat s)+l_3\omega^3(y_R-\hat\rho).
\end{aligned}
\]

There is no extra filter between these estimates and the published acceleration.

## 4. Structured target Lyapunov inequality

Let `Ac` be the three-state integrator-chain matrix, `C=e1^T`, and
`Al=Ac-l*C`. The normalized Sharma-type observer LMI selects `l`. Its existing
normalized pole region and bounded-real noise objective are retained. The
identity `Al^T*P + P*Al = -I` fixes a positive definite, dimensionless metric.

For true-minus-estimated errors use

\[
\epsilon=\operatorname{diag}(\omega^2I_2,\omega I_2,I_2)e_T,
\qquad W_T=\sqrt{\epsilon^T(P\otimes I_2)\epsilon}.
\]

All components of `epsilon` have acceleration units. The common rotation
`-u3*(I3 tensor J)` contributes exactly zero to the quadratic derivative.
Split `DeltaPhi = DeltaPhiq + DeltaPhis` by changing `q` first and then `s`.
The two global bounds become

\[
\|\Delta\Phi_q\|\le(L_q/\omega)\|\epsilon_2\|,\qquad
\|\Delta\Phi_s\|\le L_s\|\epsilon_3\|.
\]

Set `bP=P*e3`. For positive Young multipliers `tq,ts`, define

\[
M=\omega I-t_q(L_q/\omega)^2e_2e_2^T-t_sL_s^2e_3e_3^T
 -(t_q^{-1}+t_s^{-1})b_Pb_P^T.
\]

Twice applying `2 z^T d <= |z|^2/t + t |d|^2` gives

\[
\boxed{M\succeq 2\lambda_TP\quad\Longrightarrow\quad
 \dot W_T\le-\lambda_TW_T\quad\text{without exogenous error inputs}.}
\]

This is a Lyapunov/Lipschitz high-gain proof. It preserves the absence of
position from `Phi` and the `1/omega` scaling of its velocity sensitivity.
`synthesizeTargetTrackerCertificate.m` numerically searches the two positive
multipliers, recovers the generalized-eigenvalue decay rate, subtracts a small
numerical margin, and checks the final dissipation matrix. Feasibility of that
matrix is the certificate; optimizer termination does not prove global
optimality. Ordinary floating-point checks are not a directed-rounding proof.

## 5. Disturbance directions and cascade ISS

If the nominal constant-`A,kappa` model is relaxed, declare rate bounds. The
additional physical jerk is `Adot*e + kappaDot*V^2*J*e`, so

\[
\bar\nu_\Phi=\sqrt{\bar{\dot A}^{\,2}+b^4\bar{\dot\kappa}^{\,2}}.
\]

Both rates default to zero. They enter as uncertainty in the last chain
equation and do not change the nominal observer dynamics.

Define `h = [omega^2*rhoMax, omega*b, S]^T`. Cauchy--Schwarz in the actual metric
gives the directional coefficients

\[
g_{Tv}=\omega^2\sqrt{P_{11}},\quad
g_R=\omega^3\sqrt{l^TPl},\quad
g_\omega=\sqrt{h^T|P|h},\quad g_\Phi=\sqrt{P_{33}}.
\]

Here `|P|` is entrywise absolute value. The target comparison is

\[
D^+W_T\le-\lambda_TW_T+g_{Tv}\|e_v\|
 +g_\omega\bar d_\omega+g_R\bar n_R+g_\Phi\bar\nu_\Phi.
\]

Under the yaw chart condition, the full retained cascade satisfies

\[
D^+\begin{bmatrix}|e_\psi|\\\|e_v\|\\W_T\end{bmatrix}
\le H\begin{bmatrix}|e_\psi|\\\|e_v\|\\W_T\end{bmatrix}+d,
\quad H=\begin{bmatrix}
-k_\psi&0&0\\k_v\bar V_E&-k_v&0\\0&g_{Tv}&-\lambda_T
\end{bmatrix},
\]
\[
d=\begin{bmatrix}
\bar d_\omega+k_\psi B_F\\
\bar n_a+\bar V_E\bar d_\omega+k_v\bar n_v\\
g_\omega\bar d_\omega+g_R\bar n_R+g_\Phi\bar\nu_\Phi
\end{bmatrix}.
\]

`H` is Hurwitz and Metzler. The positive vector `c=-H^{-T}*ones(3,1)` gives
`c^T H=-ones(1,3)`. Therefore `Vc=c^T[|ePsi|,|ev|,WT]^T` is a copositive
cascade Lyapunov function with an explicit ISS comparison. This is implemented
in `nrmmObserverCertificate.m`. The ultimate comparison vector is `-H^{-1}d`.

Component extraction uses the tight ellipsoid factors

\[
\|e_{T,i}\|\le c_iW_T,\qquad
(c_1,c_2,c_3)^T=\operatorname{diag}(\omega^{-2},\omega^{-1},1)
 \sqrt{\operatorname{diag}(P^{-1})}.
\]

The square root is componentwise. This avoids using the worst eigenvalue for
every component and the worst metric direction for every disturbance.
Inertial position remains downstream:
`|ep|_infinity <= np + (|ev|_infinity + VEmax*|ePsi|_infinity)/kp`.
The ego-frame acceleration-difference error is bounded by the target
acceleration error plus accelerometer error, with no velocity term added to
an acceleration quantity.

## 6. Explicit gain-selection preference

The normalized gain shape is unchanged. The physical bandwidth minimizes the
largest ultimate error bound divided by its corresponding declared physical
scale, subject to `lambdaT >= 1/Tdomain` and `omega > 1/s`, where
`Tdomain=rhoMax/(VEmax+VCmax)`. This is an engineering preference for disturbance
rejection with a minimum decay rate. The ISS theorem does not force this
objective or the transit-time convention. Numerical minimization is not claimed
to be globally optimal. Initial error is treated by the continuous-time
comparison solution in Section 7 and is not mixed into the persistent-noise
objective.

For the yaw, body-velocity, and position stages, the decay rate equals the
positive scalar gain. The gain-design preference is to minimize that bandwidth
subject to a 5 percent residual over the physical transit time:

\[
\min_{k>0} k\quad\text{subject to}\quad e^{-kT_{\rm domain}}\le0.05,
\qquad k_\psi=k_v=k_p=\frac{\log20}{T_{\rm domain}}.
\]

Thus the unforced scalar error contracts to 5 percent over one domain transit
time. The 5 percent criterion is a stated continuous settling preference. The yaw-chart premise must still be satisfied. Sensor and model bounds
enter the achieved ultimate radii and the target-bandwidth optimization.
This minimum-bandwidth preference is a declared continuous-time design rule,
not an optimality consequence of ISS. In particular, the scalar ultimate
expression `b+a/k` has no finite interior minimum when `a>0`; no such optimum
is claimed. `synthesizeNrmmObserverGains.m` uses only the physical domains and
continuous error bounds, and accepts a configuration with no `runtime` fields.

## 7. Continuous-time transient and position bounds

Let `chi(t) = [|ePsi(t)|, |ev(t)|, WT(t)]^T`, with `H`, `d`, and `c1`
as defined in Section 5. Choose a nonnegative initial bound `z0 >= chi(0)`.
For the constant disturbance envelope `d`, the continuous comparison solution is

\[
z(t)=e^{Ht}z_0+\int_0^t e^{H(t-\tau)}d\,d\tau.
\]

Since `H` is Metzler, its transition matrix is nonnegative. The differential
comparison inequality in Section 5 therefore gives `chi(t) <= z(t)`
componentwise. Component extraction then yields

\[
\boxed{\|\rho(t)-\hat\rho(t)\|\le B_\rho(t):=c_1z_3(t),
\qquad c_1=\omega^{-2}\sqrt{(P^{-1})_{11}}.}
\]

This bound includes the initial yaw, body-velocity, and target errors through
the entire cascade. A steady-state radius alone does not bound an arbitrary
initial condition. Because `H` is Hurwitz,

\[
\limsup_{t\to\infty}\chi(t)\le-H^{-1}d,
\qquad
\limsup_{t\to\infty}\|\rho(t)-\hat\rho(t)\|
\le c_1(-H^{-1}d)_3.
\]

An initial metric bound can be obtained from known component bounds
`bT0 >= [|eRho(0)|, |eq(0)|, |es(0)|]^T`:

\[
W_T(0)\le\sqrt{(Db_{T0})^T|P|(Db_{T0})},
\qquad D=\operatorname{diag}(\omega^2,\omega,1).
\]

The yaw component of `z0` must satisfy `z0_1 >= |ePsi(0)|` and
`max(z0_1, BF+dOmegaBar/kPsi)+BF < pi`, which is the sufficient invariant
chart condition of Section 3 with an initial upper bound in place of the
unknown error. Initial-set bounds and operating-domain bounds are premises
on truth; the estimate by itself does not establish them.

For inertial reconstruction of the relative vector, let
`R=R(psiE)` and `Rhat=R(psiHat)`. The rotation identity gives

\[
R\rho-\hat R\hat\rho
=\hat R(\rho-\hat\rho)+(R-\hat R)\rho,
\]
\[
\boxed{\|p_C-p_E-\hat R\hat\rho\|
\le B_\rho(t)+2\rho_{\max}\sin\!\left(\frac{\min\{z_1(t),\pi\}}{2}\right).}
\]

An absolute target-position bound also requires an ego-position bound.
For center distance, the reverse triangle inequality directly gives
`|rho(t)| >= max(0, |rhoHat(t)|-Brho(t))`.

## 8. Scope of the continuous-time result

The ISS and position bounds apply on every interval where the true operating
domain, the measurement and model disturbance bounds, the positive gains,
the target dissipation inequality, and the invariant yaw chart are satisfied.
The nominal target model has constant `A` and `kappa`; their declared rate
bounds enter only through the physical jerk disturbance in Section 5.

These are estimation-error results. Rectangle orientation and footprint,
road constraints, ego tracking, and future target prediction require their
own controller analysis before a closed-loop collision-avoidance conclusion
can be drawn.

Implementation details and recorded validation results are documented in
[NRMM_IMPLEMENTATION_NOTES.md](../scripts/NRMM_IMPLEMENTATION_NOTES.md).
