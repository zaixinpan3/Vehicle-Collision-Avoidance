# Mathematical problem: joint target-observer and control-feedback prediction for PCBF

Prepared on 2026-11-02. This is a self-contained request for a mathematical derivation, not a claim that the requested construction already exists in the implementation.

## 1. Request to the reader

We need a constructive nonlinear output-feedback prediction model for vehicle collision avoidance. The missing connection is between **future target measurements, future target-observer updates, the ego vehicle's response to those estimates, and the resulting relative collision uncertainty**.

Our current online predictor includes future ego-observer feedback, but predicts the uncertain target motion separately. Consequently, it can demand that one ego reference avoid a large union of target trajectories without representing how the ego will react when future measurements distinguish those trajectories.

Please derive a computationally usable joint prediction and its interface to a predictive control barrier function (PCBF). The three central questions are:

1. What joint state, causal feedback policy, and sampled observer/plant map correctly retain the dependence between target motion, observations, estimates, and ego response?
2. How can we propagate sound, reasonably tight **relative** uncertainty sets over the prediction horizon, including model mismatch and measurement noise, without replacing future feedback by open-loop uncertainty accumulation?
3. How must next-frame measurement conditioning and policy shifting operate so that a Huang-style shifted-plan proof remains valid?

The answer must go beyond naming a generic map or recommending tube MPC. Please supply explicit equations, a tractable policy/set representation, verifiable conditions, and a derivation that can guide implementation. If any requested property is impossible under the stated assumptions, identify the obstruction and the weakest additional assumption needed.

## 2. Scope and design decisions

- There is one ego vehicle and one target vehicle during an encounter.
- The target obeys constant tangential acceleration and constant sideslip during that encounter. Their values are uncertain, not known exactly.
- GNSS measures **position only**. IMU acceleration, gyro yaw rate, and radar relative position are available. Any velocity estimate is computed from these sensors and is not an independent measurement.
- The current synthetic sensor contract gives a measurement every 0.0125 s, with an ego control hold of 0.05 s. Generalize to commensurate periods if useful, but respect the sampled update order.
- The perception radius is 50 m. The current test contract has complete angular coverage and no missed detections inside this radius. Outages would need an explicitly different observation contract.
- We consider one active encounter up to an observed or robustly established perception exit. Reentry is a new encounter. A nominal predicted exit cannot remove collision constraints for uncertain branches that are still nearby.
- Collision safety in this task concerns the declared discrete constraint instants. We are not requesting an intersample collision theorem.
- Road boundaries are used by the heuristic initializer, not imposed in the controller optimization. Do not introduce a road constraint to solve this problem.
- The ego model remains the nonlinear modified-Fiala bicycle. Do not replace it with a point mass or add an independent friction polygon as the main solution. A simplified model is welcome only as an explanatory example.
- The prediction horizon is selected at encounter initialization and then held fixed. There is no repeated horizon extension or replanning within the encounter in the current RTI workflow.
- The theory assumes the existence of a compatible terminal information-set family and terminal controller. Constructing a particular braking terminal set is **outside this request**.
- The theoretical encounter objective minimizes PCBF slack only. The implementation's secondary tracking/control-smoothing optimization is not needed for the theoretical proof.
- The nonlinear mathematical construction and its SQP-RTI approximation must be distinguished. A successful affine solve is not a nonlinear certificate.

We do **not** require a fixed-width invariant tube, a nonincreasing width at every prediction step, or globally contracting absolute target-position uncertainty. Temporary growth and a nonzero noise floor are acceptable. We want the prediction to account for future measurement feedback and avoid unnecessary open-loop accumulation in the quantities relevant to collision avoidance.

## 3. Physical model and primitive uncertainty

### 3.1 Ego vehicle

Let

$$
x_E=(p_{Ex},p_{Ey},\psi_E,v_x,v_y,r_E)^\top,
\qquad u=(\delta,b)^\top,\quad -1\le b\le1.
$$

Here $\delta$ is front steering and $b$ is a signed longitudinal-force ratio; negative values brake. Write the declared sampled plant as

$$
x_{E,k+1}=F_h(x_{E,k},u_k;\theta_E,d_k),
\qquad \theta_{E,k+1}=\theta_{E,k}.
\tag{1}
$$

The uncertain tire parameters $\theta_E$ are persistent within the prediction. The current experimental family perturbs front/rear cornering stiffness and friction within a declared box, commonly 5%. Mass, inertia and geometry are not implicitly uncertain. Any extra residual disturbance $d_k$ must have a separate declared bound consistent with the observer model; it can be set to zero for the initial derivation.

For clarity, the moving-vehicle equations are

$$
\begin{aligned}
\dot p_E&=R(\psi_E)[v_x,v_y]^\top,&\dot\psi_E&=r_E,\\
\dot v_x&=(F_{xf}\cos\delta-F_{yf}\sin\delta+F_{xr}-F_{\rm load})/m+v_yr_E,\\
\dot v_y&=(F_{xf}\sin\delta+F_{yf}\cos\delta+F_{yr})/m-v_xr_E,\\
\dot r_E&=[l_f(F_{xf}\sin\delta+F_{yf}\cos\delta)-l_rF_{yr}]/I_z.
\end{aligned}
$$

With static normal loads, $F_{xi}=b\mu_iF_{zi}$ and $D_i=\mu_iF_{zi}\sqrt{1-b^2}$. The tire slips are

$$
\alpha_f=\operatorname{atan2}(v_y+l_fr_E,v_x)-\delta,
\qquad \alpha_r=\operatorname{atan2}(v_y-l_rr_E,v_x).
$$

For $D_i>0$, define $z_i=C_i|\tan\alpha_i|/(3D_i)$. The force law is

$$
F_{yi}=\begin{cases}
-C_i\tan\alpha_i(1-z_i+z_i^2/3),&z_i<1,\\
-D_i\operatorname{sign}(\alpha_i),&z_i\ge1.
\end{cases}
$$

At $D_i=0$, lateral force is zero; the input derivative need not be finite. The implementation also blends to a kinematic low-speed map over 2.5--4 m/s and has a sample-end stop reset. These branches cannot silently be covered by a globally smooth Jacobian argument. A first theorem may use a stated smooth operating domain, but must identify what is missing outside it. For the general derivation, retain $F_h$ as the declared nonlinear map and include applicable branch or discretization defects explicitly.

### 3.2 Target vehicle: persistent NRMM motion parameters

Let $\chi_T=\psi_T+\beta_T$ be target velocity course and
$\kappa_T=\sin\beta_T/l_{rT}$. During forward motion,

$$
\dot p_T=V_T e(\chi_T),\quad
\dot V_T=A_T,\quad
\dot\chi_T=\kappa_T V_T,\quad
\dot A_T=0,\quad\dot\kappa_T=0,
\tag{2}
$$

where $e(\chi)=(\cos\chi,\sin\chi)^\top$. Target body yaw is
$\psi_T=\chi_T-\arcsin(l_{rT}\kappa_T)$ on the declared sideslip branch.

With elapsed time $\tau$ from a fixed epoch, the traveled distance is
$s(\tau)=V_{T0}\tau+A_T\tau^2/2$, and

$$
p_T(\tau)=p_{T0}+s(\tau)\operatorname{sinc}_u\!\left(\frac{\kappa_Ts(\tau)}2\right)
e\!\left(\chi_{T0}+\frac{\kappa_Ts(\tau)}2\right),
\quad \operatorname{sinc}_u(z)=\sin(z)/z.
\tag{3}
$$

For braking to rest, truncate travel at the stopping time and hold the pose thereafter. Acceleration then resets to zero in the physical stopped continuation; a pre-stop constant acceleration must not be confused with a newly drawn parameter. No reverse-motion extension is assumed.

We retain uncertainty in $(V_{T0},A_T,\kappa_T,\chi_{T0})$ as one joint parameter family in a known compact set $\Theta_{T,k}$, with nonnegative initial speed and the declared admissible sideslip branch. If these quantities are represented as components of the physical target state, do not introduce independent duplicate copies as disturbances.

### 3.3 Measurements

At a sensor instant, with $J=\begin{bmatrix}0&-1\\1&0\end{bmatrix}$,

$$
\begin{aligned}
y_p&=p_E+n_p,\\
y_a&=a_E^B(x_E,u^-;\theta_E)+n_a,\\
y_r&=r_E+n_r,\\
y_\rho&=R(\psi_E)^\top(p_T-p_E)+n_\rho.
\end{aligned}
\tag{4}
$$

The IMU is gravity-compensated at the ego center of mass. At a control boundary, its sample reflects the previously held input $u^-$; the newly selected input applies afterward. The current noise limits are

| Primitive | Deterministic bound |
|---|---:|
| GNSS position $\|n_p\|_2$ | 0.04 m |
| IMU body acceleration $\|n_a\|_2$ | 0.03 m/s² |
| Gyro $|n_r|$ | 0.0015 rad/s |
| Radar relative position $\|n_\rho\|_2$ | 0.04 m |

These are deterministic bounds, not standard deviations. Do not average them down statistically or silently replace vector balls by smaller component bounds. If different samples have no additional temporal correlation contract, allow all bounded sequences. Within a sample, the **same noise realization** is shared by every observer and feedback calculation using that sensor.

## 4. The target observer that the prediction must include

### 4.1 Relative coordinates and continuous equations

Define the physical target state in the ego body frame as

$$
X_T=(\rho,q,s),\quad
\rho=R(\psi_E)^\top(p_T-p_E),\quad
q=R(\psi_E)^\top\dot p_T,\quad
s=R(\psi_E)^\top\ddot p_T.
$$

Here $q$ is target **absolute velocity expressed in the ego frame**, not relative velocity. For $v_E=(v_x,v_y)^\top$,

$$
\begin{aligned}
\dot\rho&=q-v_E-r_EJ\rho,\\
\dot q&=s-r_EJq,\\
\dot s&=\Phi(q,s)-r_EJs.
\end{aligned}
\tag{5}
$$

On the positive-speed physical domain,

$$
V=\|q\|,\quad A=q^\top s/V,\quad
\Omega=(Jq)^\top s/V^2,\quad
\Phi(q,s)=-\Omega^2q+3A\Omega J(q/V).
\tag{6}
$$

The numerical observer uses a specified bounded Lipschitz extension $\Phi_e$. Its difference from $\Phi$, including low-speed regularization, must be included where nonzero. At zero target speed the body heading is not recoverable from Cartesian velocity; retain the relevant physical pose uncertainty.

With known gains $l_1,l_2,l_3$, the correction-mode observer is

$$
\begin{aligned}
\dot{\hat\rho}&=\hat q-\hat v_E-y_rJ\hat\rho+l_1(\hat y-\hat\rho),\\
\dot{\hat q}&=\hat s-y_rJ\hat q+l_2(\hat y-\hat\rho),\\
\dot{\hat s}&=\Phi_e(\hat q,\hat s)-y_rJ\hat s+l_3(\hat y-\hat\rho).
\end{aligned}
\tag{7}
$$

### 4.2 Actual sampled predictor/reset realization

The correction in (7) does not receive continuously available true $\rho$. Its radar predictor resets at each detection and evolves between detections:

$$
\hat y(t_j^+)=y_{\rho,j},\qquad
\dot{\hat y}=\hat q-\hat v_E-y_{r,j}J\hat y.
\tag{8}
$$

Measurements used within a sensor interval are held according to the implementation. Equations (7)--(8), the ego observer and the predictor states are integrated by fixed-substep RK4. With no radar detection, the target innovation is disabled; that is a different observation mode.

The ego observer supplies $\hat v_E,\hat\psi_E,\hat p_E$ and current error sets. Its numerical equations include

$$
\dot{\hat v}_E=y_a-y_rJ\hat v_E+k_v(v_{\rm inf}-\hat v_E),
$$

where $v_{\rm inf}$ is computed using the position/IMU velocity frontend and the declared vehicle-model inversion. The yaw estimate uses gyro propagation and a continuous rotation-vector innovation. Neither $v_{\rm inf}$ nor that innovation is an independent GNSS velocity or heading measurement.

For a standalone formulation, the complete ego update may be denoted by a known map $\Psi_E$, with its internal position/velocity frontend, position predictor, yaw state and measurement-memory states included. Please express the needed derivatives or nonlinear bounds in terms of this map; do not silently replace it by an exact-state observer.

**Important limitation:** a continuous observer ISS inequality does not automatically prove stability or containment of (7)--(8) under sampling. A proof may use a separately established sampled comparison or a direct outer image of the executed sampled map, but must identify which one it uses.

## 5. What exists, and what is missing

The current ego-feedback predictor lifts 21 states:

$$
[x_E\ (6);\ \hat p_E,\hat\psi_E,\hat v_E\ (5);\
\theta_E\ (4);\ u^-\ (2);\ \text{position/velocity frontend}\ (4)].
$$

It propagates future ego sensor updates and ancillary ego feedback using shared parameter/noise columns. The gains are obtained from a finite-horizon Riccati construction along the nominal ego trajectory. Their input authority is accounted for. This is an engineering joint-affine prediction; its complete nonlinear remainder is not certified in ordinary RTI.

**The physical target, six target-observer states, radar predictor/reset, future radar noise, and ego feedback sensitivity to the target estimate are not included in this online joint lift.** The target has a separately propagated constant-motion family.

The recent change retains a union of parameter boxes across frames and intersects it with measurement-history constraints. It preserves combinations such as $V_0+A\Delta/2$ instead of replacing them by independent coordinate hulls. Future target supports use analytic images of these boxes. This is useful information retention, but it does not supply the missing future target-observer/control coupling.

For orientation only, one recorded head-on frame at physical time 2.1 s had a 2.45 s-ahead target-position radius of approximately 31.66 m with independent bounds and 16.17 m with the correlated cover. These are **two calculations on the same information**, not evidence that the requested joint method is implemented or that all uncertainties must be that large physically. The last four-case integrated campaign still completed only two cases successfully. These observations motivate the problem; they are not mathematical premises.

## 6. Distinguish the three uncertainties

The derivation must distinguish:

$$
\underbrace{p_T-\bar p_T}_{\text{physical target deviation from a fixed forecast}},\qquad
\underbrace{p_T-\hat p_T}_{\text{future estimation error}},\qquad
\underbrace{(p_E-p_T)-(\bar p_E-\bar p_T)}_{\text{relative tracking deviation}}.
\tag{9}
$$

Two admissible constant target velocities already generate position separation proportional to time. Constant but uncertain acceleration can generate quadratic separation. Future measurements may reveal which motion is occurring without making those physical trajectories coincide.

The intended reduction is therefore in estimation and relative collision uncertainty through **causal ego response**, not an unsupported shrinkage of the absolute target reachable set. Similarly, a small future estimation error about each branch's own estimate is not a small physical error about one frozen forecast.

Please determine when relevant relative errors can remain small despite growing common absolute motion. Do not require the entire augmented matrix to be Schur: persistent parameter coordinates have identity dynamics, and irrelevant absolute translation may remain neutral.

## 7. Required construction A: a causal joint sampled map

A possible joint state is

$$
z=(x_E,x_T^{\rm phys},\hat x_E,\hat X_T,\eta_E,\hat y,\theta_E,u^-,m),
\tag{10}
$$

where $x_T^{\rm phys}$ carries (2), $\eta_E$ denotes the remaining ego-observer/frontend memory, and $m$ carries encounter/reference/policy memory. This list is a starting point, not a prescribed minimal realization. Avoid duplicate physical parameters when changing coordinates.

At the current decision, the observer software state, available measurements and issued-input history are known; physical states and parameters remain uncertain. Let $I_k$ contain all joint states compatible with that information.

We need causal policies

$$
u_{i|k}=\pi_{i|k}(o_{i|k}),
\quad o_{i|k}=\text{information actually available at that future decision}.
\tag{11}
$$

All worlds consistent with the current information must receive the same first input. Later inputs may differ only after their measurement histories differ. A candidate local parameterization is

$$
u_i=\bar u_i+K_{E,i}(\hat x_{E,i}-\bar{\hat x}_{E,i})
                 +K_{T,i}(\hat X_{T,i}-\bar{\hat X}_{T,i}),
\tag{12}
$$

with appropriate charts and any necessary predictor/reference states. The barred observer states are nominal software states, not assumed equal to physical states. A target-relative or innovation-feedback parameterization is equally welcome if better justified.

Please derive the joint transition

$$
z_{i+1}=\mathcal G_i(z_i,\nu_i;\pi_i),
\tag{13}
$$

including, in the actual chronological order, old-input IMU sampling, predictor resets, observer/frontend updates, new control selection, held-input plant motion and the intervening sensor updates. Specify the convention for prior versus posterior software states so no measurement is assimilated twice.

Here $\nu_i$ collects the primitive noises at every sensor substep and any separately declared residual $d_i$ over that hold. Its admissible set must also account for numerical defects if the theorem concerns a continuous physical plant rather than the declared discrete map. Persistent parameters remain coordinates of $z$, not new components redrawn in $\nu_i$.

The unknown future measurement must be generated from the **same physical branch** in (13). It cannot be an independently chosen estimate inside a ball. Likewise, the same $\theta_E$ influences the true motion and every resulting observer residual; it must not be redrawn at each stage.

Please also explain how the predicted feedback policy relates to executing the first command after each reoptimization. A receding-horizon optimizer is not automatically equivalent to an assumed ancillary feedback gain. The retained future policy must remain a feasible candidate even when a later optimizer chooses another admissible policy.

## 8. Required construction B: explicit error propagation and feedback design

Derive joint error coordinates containing ego tracking error, ego estimation error, target estimation error, relevant relative-position/orientation error, persistent parameter error, and predictor memory. Derive the blocks introduced by $K_T$, radar correction and moving-frame rotation explicitly.

A useful algebra check, not a substitute for the nonlinear derivation, is the linear predictor-observer example

$$
X^+=AX+Bu+E\delta\theta+Dw,\quad y=CX+n,
\quad \hat X^+=A\hat X+Bu+L(y-C\hat X),
$$

$$
\bar X^+=A\bar X+B\bar u,\quad
u=\bar u+K(\hat X-\bar X).
$$

Here $X$ contains both vehicles, $B$ has zero target rows, and $K$ may have both ego and target columns. With $\eta=X-\bar X$, $e=X-\hat X$,

$$
\begin{bmatrix}\eta^+\\e^+\\\delta\theta^+\end{bmatrix}
=
\begin{bmatrix}A+BK&-BK&E\\0&A-LC&E\\0&0&I\end{bmatrix}
\begin{bmatrix}\eta\\e\\\delta\theta\end{bmatrix}
+
\begin{bmatrix}D&0\\D&-L\\0&0\end{bmatrix}
\begin{bmatrix}w\\n\end{bmatrix}.
\tag{14}
$$

This example has its own causal timing; it is not the implemented multi-rate predictor/reset map. It shows why independent copies of the same mismatch or noise destroy cancellations. The vehicle-specific answer must handle the actual timing in Section 7.

Please provide a computable choice or synthesis condition for the policy gains. We need to know when relative tracking can reject target-motion uncertainty while respecting steering/braking authority, not merely that a stabilizing gain would be helpful. Explain what happens at saturation, low speed, poor target observability, or insufficient maneuver authority. A failed condition should be exposed rather than repaired by clipping the uncertainty set.

## 9. Required construction C: sound, useful nonlinear outer images

We need implementable sets $\mathcal R_{i|k}$ satisfying

$$
\mathcal R_{0|k}=I_k,\qquad
\{\mathcal G_i(z,\nu;\pi_i):z\in\mathcal R_{i|k},\nu\in\mathcal N_i\}
\subseteq\mathcal R_{i+1|k}.
\tag{15}
$$

The representation may use constrained zonotopes, joint ellipsoids, parameterized sets, validated local Taylor images or another tractable construction. Explain how it preserves the shared target/ego/observer variables before taking collision supports. Independent bounding balls and repeated absolute-value propagation can discard the correlation we need.

For an affine-plus-remainder construction, a useful required interface is

$$
\zeta_{i+1}=\mathcal A_i\zeta_i+\mathcal D_i\nu_i+c_i+r_i,
\quad r_i\in\mathcal W_i^{\rm rem}.
\tag{16}
$$

Identify the full domain over which $\mathcal W_i^{\rm rem}$ is valid. It must include optimizer corrections, physical uncertainty, observer uncertainty and input feedback excursions. A finite set of Jacobian probes is not a uniform remainder bound. State how branch changes are enclosed, or restrict and explicitly state the theorem's domain.

The published current error bound initializes or conditions $I_k$. Its past mismatch/noise effects must not be added again as a fresh disturbance every stage. Future mismatch still acts through (1), (4), (7) and (13); omitting it would also be wrong.

Please give a checkable bound or criterion showing when the **relevant relative projection** is controlled by closed-loop dynamics and noise, rather than by open-loop target drift. A finite-horizon bound is acceptable. Do not promise a universal small tube or initial feasibility for arbitrary closing speeds and distances.

## 10. Required construction D: collision and input constraints

Vehicles are rectangles, including any body-fixed offset from their dynamics reference point:

$$
\mathcal V_E=p_E+R(\psi_E)\mathcal B_E,\qquad
\mathcal V_T=p_T+R(\psi_T)\mathcal B_T.
$$

For a unit separating normal $n$, define

$$
g_n(z)=\min_{a\in\mathcal B_E}n^\top[p_E+R(\psi_E)a]
       -\max_{b\in\mathcal B_T}n^\top[p_T+R(\psi_T)b].
\tag{17}
$$

A robust stage constraint can use

$$
\inf_{z\in\mathcal R_i}g_{n_i}(z)\ge-\xi_i,
\quad \xi_i\ge0,
\tag{18}
$$

with its chosen separating witnesses retained in a shift. The exact theoretical formulation permits continuous witnesses; no bank of fixed world directions is required. A fixed nominal normal is sufficient for deriving a local support formula, provided its limitation is stated.

Please derive the collision tightening from the **joint relative set**, including orientation uncertainty and shared ego-pose cancellation. Do not subtract the separate worst ego and target position radii when their dependence is available. Input constraints must also contain every command generated by the predicted policy and any required input memory or slew.

At zero slack, nonnegative separation excludes interior overlap but permits contact at equality. A strictly positive-clearance statement needs an additional strictness premise; do not introduce an arbitrary physical buffer such as 0.2 m. No continuous-time collision statement is requested here.

## 11. Required construction E: measurement conditioning and recursive feasibility

After applying the first common input and obtaining the next measurements, the next information set must satisfy

$$
z_{k+1}\in I_{k+1}\subseteq\mathcal R_{1|k}.
\tag{19}
$$

A conceptual update is

$$
I_{k+1}=\mathcal R_{1|k}\cap\mathcal Q_{k+1}\cap\mathcal A_{k+1},
\tag{20}
$$

where $\mathcal Q$ is a valid current observer enclosure and $\mathcal A$ imposes the newly known measurements, software state and memory. A numerical outer approximation of this intersection must not lose containment in the retained predicted successor. Replacing it by an unrelated fresh box is insufficient.

Please give an implementable conditioning representation, explain the treatment of recentering, and show that the old future policies remain in the next admissible policy class. Recomputing gains, normals, references or numerical domains may otherwise exclude the old candidate. An empty consistent-set intersection is a violated premise, not permission to declare zero error.

Assume a nonempty, downward-closed terminal family $\mathfrak B$ with a causal terminal policy whose admissible zero-deficit successor remains in $\mathfrak B$. This assumption uses the same joint dynamics, noise and parameter contract, information structure and collision definition. **Do not spend the response designing a specific braking terminal set.**

For a fixed horizon, the exact nonlinear PCBF problem has causal policies, outer sets, witnesses and common nonnegative stage slacks:

$$
V_P(I_k)=\min\sum_{i=0}^{N}\xi_{i|k},
\tag{21}
$$

subject to (15), hard admissibility, (18), and $\mathcal R_{N|k}\in\mathfrak B$. The terminal family admits zero endpoint deficit. The current theoretical control applies the first command of any attained primary minimizer. A performance objective is not part of this theorem.

The desired shift is the restricted old policy tail plus the terminal policy, with slack sequence
$(\xi_{1|k},\ldots,\xi_{N|k},0)$. Establish, under explicit sufficient assumptions,

$$
V_P(I_{k+1})\le V_P(I_k)-\xi_{0|k}\le V_P(I_k).
\tag{22}
$$

Thus an initially feasible mathematical problem stays feasible, and a zero optimal slack sum stays zero throughout the encounter. This concerns the **sum**, not every predicted slack component individually. Positive slack is allowed but is not a collision-safety claim; nonincrease alone does not prove finite-time arrival at zero.

If avoiding exact global minimization is useful, separately state a retained-feasible-plan variant with cost $J_{k+1}\le J_k-\xi_{0|k}$. Do not equate its stored feasible cost with the global value function. Nor should either exact nonlinear argument be attributed automatically to the current SQP-RTI step.

## 12. Requested answer and practical checks

Please organize the answer around:

1. A diagnosis of which uncertainty can be reduced and which cannot.
2. A concrete joint state, measurement/update timeline and causal feedback policy.
3. Explicit error dynamics and a usable feedback-design condition, including the target-estimate channel.
4. A finite-horizon set recursion with a justified nonlinear remainder and shared uncertainty sources.
5. Relative collision supports, input-authority reserves and next-frame conditioning.
6. A theorem/proof connecting that construction to (22), separating assumptions from established lemmas.
7. Pseudocode for initialization, prediction, measurement conditioning and shifting; indicate which operations can later be approximated in SQP-RTI.
8. The minimum unresolved assumptions or counterexamples, if a complete construction is not possible.

Useful sanity checks are two constant-speed target worlds initially indistinguishable within radar noise, opposite or crossing target motion, acceleration/braking to rest, and operation near the ego's low-speed branch. In the two-world example, the first command must be identical; later target and ego absolute positions may separate while relative separation remains controlled. Also include a case where available control authority is insufficient, so the method does not falsely certify every encounter.

A simulation-only contraction result, a covariance prediction, a future measurement reset without a causal update model, or an assertion that finite partitioning can eventually work is not by itself the requested derivation. Please prioritize a small, explicit construction over adding another full trajectory optimizer or an exhaustive scenario tree.

## 13. Source map and status of this document

This document was prepared with AI assistance from the maintained equations and code. It is a problem specification, not an independently reviewed proof or a literature survey. The mathematical targets above extend the project's Huang-style PCBF shift argument to uncertain observation and moving-target feedback; they are not attributed to the original paper as already proved there.

The following repository files support the implementation description; access to them is optional for understanding the request:

| File | Relevant content |
|---|---|
| `controller/CONTROLLER_THEORY.md`, Sections 6--7 | Causal joint prediction, information conditioning, abstract terminal assumption and conditional PCBF shift theorem |
| `controller/nonlinearBicycleModel.m`, `controller/modifiedFialaTire.m` | Ego transition, tire coupling, low-speed blend and stop reset |
| `estimator/nrmmTargetTrackerDerivative.m` | Relative NRMM coordinates and the regularized nonlinear target map |
| `estimator/nrmmObserverVectorField.m`, `estimator/onlineNrmmTrackingRuntime.m` | Actual target observer, radar predictor, sensor/reset/RK4 ordering |
| `estimator/nrmmPositionVelocity.m`, `estimator/nrmmEgoCourseGeometry.m` | Position-only velocity frontend and model-aided ego observation |
| `estimator/nrmmJointPrediction.m` | Existing 21-state ego-only joint prediction and ancillary gains |
| `estimator/nrmmConstantMotionFamily.m`, `estimator/nrmmTargetHistory.m` | Persistent target-parameter cover and history conditioning |
| `controller/predictiveSafetyGeometry.m` | Target-family supports and rectangle geometry |
| `estimator/OBSERVER_ISS_THEORY.md`, Sections 11--12 | Current enclosures and the sampled-observer scope limitation |

The parameter-cover implementation was introduced in commit `b5d7cdcc876c13a2de2670f6f95c0dc00c99b0b9`. It is not a complete target-feedback tube. Earlier experimental documents may describe different terminal or solver variants; the explicit scope in this request is the one to use.
