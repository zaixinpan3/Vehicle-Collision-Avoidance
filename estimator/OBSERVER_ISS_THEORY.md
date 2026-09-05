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
A sampled Jacobian maximum is not used as the proof. The reconstructed
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
`|ePsi|`, contradicting an unrestricted linear-decay claim. The runtime's
`yawInnovationChartCompatible` flag is only a pointwise diagnostic; it does not
establish chart invariance or an initial true-error bound.

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
to be globally optimal. The conservative transient formula
`WT(t) <= exp(-lambdaT*t)*WT(0) + WT_infinity` remains available; a full-domain
initial-error bound is not mixed into the persistent-noise objective.

The yaw, velocity, and position gains retain their scalar objective
`max(a/k, Ts*b*k)` on `[1/Tdomain,1/Ts]`. This objective balances process-error
leakage and innovation displacement. It is likewise a stated design preference,
not an optimum implied by the continuous ultimate bound `b+a/k`.

## 7. Sampling, radar prediction, and scope of the theorem

`onlineNrmmTrackingRuntime.m` receives a synchronized frame at `t`, resets its
output predictors, and advances the continuous observers to `t+Ts` with RK4.
The GNSS position predictor follows estimated inertial velocity. The radar
predictor uses its own moving-frame dynamics,

\[
\dot y_P=\hat q-\hat v-u_3Jy_P.
\]

While radar is available its innovation obeys exactly

\[
\dot r=-(K_1I+u_3J)r,\qquad r=y_P-\hat\rho.
\]

Using `-u3*J*rhoHat` in both predictor and observer would omit the rotation of
this residual. The corrected equation is tested against its matrix exponential.
For zero ego rotation, the acceleration injection integrated over one interval
is `(K3/K1)*(1-exp(-K1*Ts))*r(0)`. This explains why excessive bandwidth can
strongly amplify fresh radar noise even with a motion predictor.

The continuous theorem assumes bounded-error measurements throughout time.
A noise bound at one sample instant does not bound the error of a held signal
over an interval without additional intersample assumptions. Predictor resets,
intersample errors, dropouts, and RK4 remainders require a separate digital
stability proof. During radar dropout the correction is removed and the
nominal model coasts; the positive continuous correction decay cannot be claimed
for that interval. Both design and runtime explicitly publish
`sampledImplementationCertified=false`.

Scenario controller margins are separately labeled engineering assumptions.
They are not derived from this continuous observer certificate, and neither the
benchmark nor this document proves closed-loop collision avoidance.

## 8. Executable verification and reproduction

The behavior tests check the NRMM coordinate correspondence, exact equality of
`Phi_e` and `Phi` on the operating domain, global bounds across saturation
regions, the nonlinear Lyapunov derivative, component ellipsoid factors,
wrapped-yaw limitations, radar residual rotation, and track reset isolation.
A noisy seed outside the benchmark seed range checks acceleration performance.

From the repository root:

```matlab
addpath('estimator','config','scripts');
results = runtests({'tests/nrmmStructuredHighGainTest.m', ...
    'tests/observerGainSynthesisTest.m','tests/nrmmCascadeCertificateTest.m', ...
    'tests/nrmmModelFormulationTest.m','tests/nrmmObserverVectorFieldTest.m', ...
    'tests/certifiedKinematicCourseCorrespondenceTest.m', ...
    'tests/onlineNrmmTrackingRuntimeTest.m'});
assertSuccess(results);
runOnlineNrmmComplexManeuverScenario('Plot',false,'NoiseModel','boundedUniform');
benchmark = runOnlineNrmmTrackingErrorBenchmark();
```

A paired historical comparison accepts `BaselineRuntime` and `BaselineDesign`
function handles to independently versioned source exports. Trials share truth,
noise draws, initialization, and sample timing. Continuous ultimate bounds,
empirical RMSE, initial peaks, curvature lag, domain exits, and computation time
are reported separately. Generated results and source exports belong outside
the estimator directory.

## 9. Paired synthetic benchmark

The reference is the previous repository high-gain implementation at commit
`ac20bc4317c373e388b846bcf0f42fc8eb077d61`, not a reproduction of the paper's
full-scale vehicle experiments. Both variants retain the NRMM high-gain cascade.
There are 42 trials: 21 per variant, 12 s duration, 2 s burn-in, bounded-uniform
noise seeds 7--11, one noise-free trial, 50/25 Hz sensing, a changing-motion case,
and a radar dropout from 4 to 5 s. Initial states and sensor draws are identical
within each pair. Noise-free trials retain the same designed gains.

Five-seed mean results:

| Case | Design | Position RMSE (m) | Velocity RMSE (m/s) | Acceleration RMSE (m/s^2) |
|---|---|---:|---:|---:|
| Retained model, 50 Hz | Previous high gain | 0.093528 | 2.7231 | 29.5604 |
| Retained model, 50 Hz | Structured high gain | 0.046948 | 0.48023 | 1.8236 |
| Retained model, 25 Hz | Previous high gain | 0.17934 | 4.8262 | 48.511 |
| Retained model, 25 Hz | Structured high gain | 0.071183 | 0.71372 | 2.6685 |
| Changing motion | Previous high gain | 0.094063 | 2.7576 | 30.1445 |
| Changing motion | Structured high gain | 0.045581 | 0.44295 | 1.6020 |
| One-second dropout | Previous high gain | 2.8549 | 39.6818 | 404.986 |
| One-second dropout | Structured high gain | 0.24014 | 1.4605 | 5.1457 |

Position and acceleration errors use the ego frame; velocity RMSE uses inertial
reconstruction. The default bandwidth changes from 19.011 to 6.4431 /s and the
three gains from approximately `[86.99,2856,32846]` to `[29.48,328.0,1278.7]`.
The sufficient continuous target decay changes from 1.5594 to 0.8000 /s, while
the continuous position ultimate bound decreases from 312.03 to 6.2185 m.
This is a smaller sufficient bound, not a tight error radius for the sampled run.

The mean peak acceleration error during the first 2 s falls from 346.06 to
34.37 m/s^2. Startup peaking remains. Post-burn-in estimates satisfy the full
physical reconstruction domain at about 62.4% of samples in the noisy retained
case; the global extension handles the remaining samples, and the proof does
not require the estimated state to stay inside that physical domain. Noise-free
post-burn-in domain membership is 100%. Mean step time is approximately 2.44 ms
on this host, without a worst-case execution claim.

The changing-motion case declares `|Adot|<=1.5 m/s^3` and
`|kappaDot|<=0.0175 /(m*s)`, with a 55 m range domain. Its best-shift curvature
lag estimate is 0.272 s for the revised observer and 0.644 s for the previous
one; this noisy-signal metric is not an exact phase-delay measurement. The
previous certificate does not include the additional model-jerk forcing.
`DeclaredModelJerkCovered`, `HasRadarDropout`, and `DigitalCertified` in the
trial table make these qualifications explicit. Dropout results are empirical;
the continuous correction theorem is not applied to missing-measurement intervals.
