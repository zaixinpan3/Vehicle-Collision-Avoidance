# Single-target estimation and trajectory planning

The pipeline models one obstacle vehicle. Scalar records replace collections at
every interface. A target may be unobserved, undergoing reacquisition, active in
the controller, or confirmed outside the encounter. None of these modes adds a
second target state. Time samples, road boundaries and two passing-side choices
are separate dimensions and remain sequences.

## Observer state and sampled correction

Let $J=\left[\begin{smallmatrix}0&-1\\1&0\end{smallmatrix}\right]$,
$\rho=R_E^T(p_o-p_E)$, $q=R_E^Tv_o$, and $s=R_E^Ta_o$.
The retained transformed NRMM model is

\[
\dot\rho=q-v_E-r_EJ\rho,\qquad
\dot q=s-r_EJq,\qquad
\dot s=\Phi(q,s)-r_EJs.
\]

With radar-output predictor $\xi_r\in\mathbb R^2$ and detection flag
$d\in\{0,1\}$, the observer is

\[
\dot{\hat z}=f_T(\hat z,\hat v_E,r_m)
+d\begin{bmatrix}k_1I_2\\k_2I_2\\k_3I_2\end{bmatrix}
(\xi_r-\hat\rho),\qquad \hat z=[\hat\rho;\hat q;\hat s].
\]

At an available radar sample, $\xi_r^+=y_r$; otherwise the predictor continues
without radar innovation. There is one availability flag and one optional
identifier. A supplied identifier is checked against the initialized target;
no row matching, permutation, track slot or target-count option exists.

The numerical state has the fixed order

\[
x_{\rm RK4}=[\hat v_E^T,\hat p_E^T,\xi_p^T,\hat z^T,\xi_r^T,\hat\psi_E]^T
\in\mathbb R^{15}.
\]

The target occupies entries 7--12 and its predictor entries 13--14. A reset
accepts a six-vector directly and preserves ego state and time. The separate
sampled enclosure contains one three-vector $(E_\rho,E_q,E_s)$, one Lyapunov
radius, one predictor radius and one history record. Radar correction uses the
existing three-state comparison ODE; dropout uses the five-state comparison
ODE. These dimensions describe error components, not a target collection.
The continuous ISS and conditional sampled-containment qualifications are
unchanged by the scalar representation.

## Initial trajectory

The analytical NRMM pose $p_o(t),\psi_o(t)$ supplies one moving Gaussian center
$s_o(t)$ in the route chart. For each passing sign $\sigma\in\{-1,+1\}$,

\[
g^\sigma(s,t)=b^\sigma(t)
\exp\!\left[-\frac{(s-s_o(t))^2}{2\ell^2}\right],\qquad
p_r^\sigma(t)=F(s_E(t),g^\sigma(s_E(t),t)).
\]

There is one width $\ell$, one obstacle projection and one coefficient per
passing candidate and time node. The Cheng-style stationary condition
$\partial_\eta(\eta^2/2-\eta g^\sigma)=0$ gives this reference. There is no
sum over obstacles or combination of different targets' passing choices.
Without two finite road boundaries the implementation uses the supplied route
with normalization $m=0$, $h=1$ m; this is not a detected road width. The
reference is evaluated directly, as documented in
[NRMM_VFFM_INITIALIZATION.md](NRMM_VFFM_INITIALIZATION.md). It is subsequently
fitted to the ego model and used to initialize the complete constrained solve.

## Controller state and geometry

`readPlanningInputs` returns one optional target. `hardEncounterBarrier.prepare`
stores one optional `model.encounter` and compares its key with the prior
encounter key only to preserve identity across samples. Conditioning and the
finite-flow error enclosure are applied once. The completion certificate has
one exit direction and one exit row when a target is active.

At each prediction node the collision constraint is

\[
n_k^T(p_{o,k}-p_{E,k})
-h_{Q_E}(R_{E,k}^Tn_k)-h_{Q_o}(-R_{o,k}^Tn_k)\ge d_k.
\]

The implementation keeps the existing uncertainty support and numerical
reserves. Each native geometry record contains one fixed scalar `target`, a
Boolean `hasTarget`, and an optional two-vector `normal`. Multiple records
refer to different prediction times. Road-boundary rows remain indexed by
boundary, independently of whether a target is present.

Current observation confirms target release. A missing observation cannot
silently discharge an in-range carried encounter. Release removes its collision
and exit rows while preserving road, input, terminal and remaining time-node
obligations. A changed identity needs verified absence of the previous target
and fresh admission of the new encounter; the two are never stored together.
The saved controller-state format is 42 because its schema changed.

## Interface

| Location | Scalar representation |
| --- | --- |
| Initialization | `targetInitialState` (6-by-1), optional `targetIdentifier` |
| Sensor frame | `radarRelativePosition` (2-vector), `radarDetectionAvailable` (logical), optional `radarTargetIdentifier` |
| Estimator output | `targetState` (6-by-1), `targetEstimate` (scalar structure) |
| Adapter publication | `targetEstimate` (scalar structure or empty) |
| Controller state | `encounter` (scalar structure or empty), `targetReleased` flag during formulation |
| Reference | one scalar width; arrays only over candidate side and time |
| Native geometry | scalar `target`, scalar `hasTarget`, 2-by-1 or 2-by-0 `normal` |

The plural output aliases, configurable target count, target slots, radar row
association, target-array geometry, indexed completion keys and target-set
admission logic have been removed. Rebuild generated observer, geometry and
prepared-controller adapters after changing this source schema. Generated
binaries remain outside the tracked research source.
