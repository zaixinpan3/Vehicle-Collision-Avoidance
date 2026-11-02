# Mathematical problem: nonlinear joint observer-feedback prediction and PCBF recursive feasibility

## 1. The requested mathematical result

Consider collision avoidance between an ego vehicle and a moving target, with bounded sensor noise and uncertain vehicle parameters. We seek a **nonlinear output-feedback predictive controller** whose prediction includes future measurements, future observer updates, and the ego vehicle's causal response to the resulting estimates.

The unresolved connection is:

$$
\text{physical target motion}
\longrightarrow\text{future relative-position measurements}
\longrightarrow\text{future target estimates}
\longrightarrow\text{future ego control}
\longrightarrow\text{relative collision geometry}.
$$

Predicting an uncertain target independently of a single prescribed ego trajectory omits this connection. It can require that trajectory to avoid every possible target path, without allowing the ego to respond differently after measurements distinguish the possibilities.

Please construct this connection mathematically, derive a sound joint uncertainty prediction, and establish its compatibility with a predictive control barrier function (PCBF) shifted-plan proof. The central desired conclusion is:

> If the nonlinear PCBF problem is feasible at one decision instant, a feasible continuation exists at the next instant and its optimal slack sum does not increase. Once that sum is zero, it remains zero throughout the encounter, under explicitly stated assumptions.

This is entirely a question about nonlinear dynamics, nonlinear observation, causal nonlinear control, and set-valued information. Every model parameter, uncertainty bound, and observation period is symbolic. The requested conclusions must follow from explicitly stated mathematical assumptions. Any additional assumption must be identified and justified.

## 2. Scope and premises

- There is one ego vehicle and one target during an encounter.
- The target has constant tangential acceleration and constant sideslip during forward motion in that encounter. These values are uncertain, not exactly known. A stopping continuation is specified below.
- GNSS measures **position only**. Body acceleration, yaw rate, and target relative position are also measured. There is no independent measured velocity or heading.
- Sensor measurements arrive with period $h_s>0$. Control decisions have period $h=Mh_s$, where $M\in\mathbb N_{>0}$, and the input is held between decisions. The clocks share an initial epoch. Any restrictions on $h_s$, $h$, or $M$ needed by the theorem must be derived or stated as assumptions.
- Relative-position detection is available whenever $\|p_T-p_E\|\le R_{\mathrm{sen}}$, where $R_{\mathrm{sen}}>0$ is a symbolic sensing radius. Assume full angular coverage and no missed detections within that set.
- The analysis concerns one encounter until an observed or robustly established perception exit. Reentry starts a new encounter. A nominal forecast of exit cannot remove constraints for admissible branches that remain nearby.
- Collision safety is required at the declared discrete constraint instants. An intersample collision theorem is not requested.
- Road constraints are outside this problem.
- The ego dynamics are the nonlinear modified-Fiala bicycle. Do not replace that model by a point mass or add an independent friction polygon as the main solution.
- The horizon length $N$ is fixed within an encounter. A geometric initializer is not part of the requested proof.
- A compatible terminal family and terminal controller are **assumed to exist**. Their specific braking or vehicle-level construction is outside this request.
- The theoretical objective is PCBF slack minimization. No secondary tracking objective is needed for the theorem.

We do **not** require a fixed-width invariant tube, monotonic shrinkage of every predicted uncertainty width, or contraction of absolute target-position uncertainty. Temporary growth and a noise floor are acceptable. The requirement is reliable prediction that accounts for future observation and control feedback, together with a precise recursive-feasibility argument.

## 3. Physical dynamics and shared uncertainty

Write $R(\psi)$ for planar rotation, $J=\begin{bmatrix}0&-1\\1&0\end{bmatrix}$, and $e(\chi)=(\cos\chi,\sin\chi)^\top$.

### 3.1 Ego vehicle

The ego state and input are

$$
x_E=(p_{Ex},p_{Ey},\psi_E,v_x,v_y,r_E)^\top,
\qquad u=(\delta,b)^\top\in\mathcal U,
\qquad |\delta|\le\delta_{\max},\quad -1\le b\le1.
\tag{1}
$$

Here $r_E$ is yaw rate, $\delta$ is front steering, and $b$ is a signed longitudinal-force ratio; negative $b$ brakes. The moving-vehicle equations are

$$
\begin{aligned}
\dot p_E&=R(\psi_E)[v_x,v_y]^\top,&\dot\psi_E&=r_E,\\
\dot v_x&=(F_{xf}\cos\delta-F_{yf}\sin\delta+F_{xr}-F_{\rm load})/m+v_yr_E,\\
\dot v_y&=(F_{xf}\sin\delta+F_{yf}\cos\delta+F_{yr})/m-v_xr_E,\\
\dot r_E&=[l_f(F_{xf}\sin\delta+F_{yf}\cos\delta)-l_rF_{yr}]/I_z.
\end{aligned}
\tag{2}
$$

Mass $m$, yaw inertia $I_z$, axle distances $l_f,l_r$, gravitational acceleration $g$, and the resistance law $F_{\rm load}(v_x)$ are known. Static normal loads are $F_{zf}=mgl_r/(l_f+l_r)$ and $F_{zr}=mgl_f/(l_f+l_r)$.

For axle $i\in\{f,r\}$, let $F_{xi}=b\mu_iF_{zi}$ and $D_i=\mu_iF_{zi}\sqrt{1-b^2}$. Define

$$
\alpha_f=\operatorname{atan2}(v_y+l_fr_E,v_x)-\delta,
\qquad \alpha_r=\operatorname{atan2}(v_y-l_rr_E,v_x).
$$

For $D_i>0$, put $z_i=C_i|\tan\alpha_i|/(3D_i)$ and use

$$
F_{yi}=\begin{cases}
-C_i\tan\alpha_i(1-z_i+z_i^2/3),&z_i<1,\\
-D_i\operatorname{sign}(\alpha_i),&z_i\ge1.
\end{cases}
\tag{3}
$$

For $D_i=0$, set $F_{yi}=0$. The uncertain parameters $\theta_E=(C_f,C_r,\mu_f,\mu_r)$ belong to a known compact set of positive values and remain constant during the encounter. They must not be replaced by independently redrawn disturbances at successive prediction steps.

Denote the exact held-input transition by

$$
x_{E,k+1}=F_h(x_{E,k},u_k;\theta_E,d_{[k,k+1]}),
\qquad \theta_E^+=\theta_E.
\tag{4}
$$

Here $d$ is an optional additional residual disturbance with a separately stated bound; the first derivation may set $d=0$. The theorem must state a domain on which the vehicle flow is well defined. If a low-speed or stopped extension is needed, state it as an additional model assumption rather than silently extrapolating slip formulas through a singularity. Global differentiability is not a premise.

### 3.2 Target motion

Let $\chi_T=\psi_T+\beta_T$ denote velocity course and $\kappa_T=\sin\beta_T/l_{rT}$. During forward motion,

$$
\dot p_T=V_T e(\chi_T),\qquad
\dot V_T=A_T,\qquad
\dot\chi_T=\kappa_TV_T,\qquad
\dot A_T=0,\qquad\dot\kappa_T=0.
\tag{5}
$$

The sideslip branch is fixed with $|\beta_T|<\pi/2$, so $\psi_T=\chi_T-\arcsin(l_{rT}\kappa_T)$. At a fixed epoch, the uncertain initial pose, speed, acceleration, and curvature belong to one compact **joint** parameter set $\Theta_T$. Preserve their dependence.

Before stopping, with $s(t)=V_{T0}t+A_Tt^2/2$,

$$
p_T(t)=p_{T0}+s(t)\operatorname{sinc}_u\!\left(\frac{\kappa_Ts(t)}2\right)
e\!\left(\chi_{T0}+\frac{\kappa_Ts(t)}2\right),
\qquad\operatorname{sinc}_u(z)=\begin{cases}\sin z/z,&z\ne0,\\1,&z=0.\end{cases}
\tag{6}
$$

If $A_T<0$ and speed reaches zero, stop the travel integral and hold the target pose thereafter. Physical acceleration is then zero. This is a declared mode change, not a new independent uncertain acceleration. At rest, retain body-yaw uncertainty; it cannot be recovered from a zero velocity vector.

### 3.3 Measurements and noise

At sensor time $t_j$,

$$
\begin{aligned}
y_{p,j}&=p_E(t_j)+n_{p,j},\\
y_{a,j}&=a_E^B(x_E(t_j),u^-;\theta_E)+n_{a,j},\\
y_{r,j}&=r_E(t_j)+n_{r,j},\\
y_{\rho,j}&=R(\psi_E(t_j))^\top[p_T(t_j)-p_E(t_j)]+n_{\rho,j}.
\end{aligned}
\tag{7}
$$

The gravity-compensated IMU measures body-frame acceleration at the ego center of mass:
$a_E^B=\dot v_E+r_EJv_E$, where $v_E=(v_x,v_y)^\top$. At a control boundary, $u^-$ is the previously held input; the new command acts after sampling.

Let the measurement noise satisfy the symbolic deterministic bounds

$$
\|n_{p,j}\|_2\le\bar n_p,\qquad
\|n_{a,j}\|_2\le\bar n_a,\qquad
|n_{r,j}|\le\bar n_r,\qquad
\|n_{\rho,j}\|_2\le\bar n_\rho,
$$

where $\bar n_p,\bar n_a,\bar n_r,\bar n_\rho$ are finite nonnegative parameters. More generally, the primitive noise vector may belong to a given compact joint set $\mathcal N$, retaining any known dependence between its components. All sequences allowed by the declared set are admissible unless additional temporal structure is assumed.

Within a sample, the same noise realization is used wherever that measurement enters observation or control. Likewise, the same physical parameter $\theta_E$ affects the vehicle and its resulting sensor readings. The bounds are deterministic set-membership premises, not probabilistic confidence statements.

## 4. Observer dynamics and the ego-estimator interface

### 4.1 Target observer

In the ego frame define

$$
X_T=(\rho,q,s),\qquad
\rho=R(\psi_E)^\top(p_T-p_E),\quad
q=R(\psi_E)^\top\dot p_T,\quad
s=R(\psi_E)^\top\ddot p_T.
$$

Thus $q$ is target absolute velocity expressed in the ego frame, not relative velocity. While the target moves,

$$
\dot\rho=q-v_E-r_EJ\rho,\qquad
\dot q=s-r_EJq,\qquad
\dot s=\Phi(q,s)-r_EJs,
\tag{8}
$$

where

$$
V=\|q\|,\qquad A=q^\top s/V,\qquad
\Omega=(Jq)^\top s/V^2,\qquad
\Phi(q,s)=-\Omega^2q+3A\Omega J(q/V).
\tag{9}
$$

For symbolic positive gains $l_1,l_2,l_3$, consider

$$
\begin{aligned}
\dot{\hat\rho}&=\hat q-\hat v_E-y_{r,j}J\hat\rho+l_1(\hat y-\hat\rho),\\
\dot{\hat q}&=\hat s-y_{r,j}J\hat q+l_2(\hat y-\hat\rho),\\
\dot{\hat s}&=\Phi_e(\hat q,\hat s)-y_{r,j}J\hat s+l_3(\hat y-\hat\rho).
\end{aligned}
\tag{10}
$$

The relative-position predictor resets and flows according to

$$
\hat y(t_j^+)=y_{\rho,j},\qquad
\dot{\hat y}=\hat q-\hat v_E-y_{r,j}J\hat y
\quad(t_j<t<t_{j+1}).
\tag{11}
$$

The gyro sample is held on that sensor interval. The observer state $\hat X_T=(\hat\rho,\hat q,\hat s)$ is continuous at detection; only $\hat y$ resets there. All flows in this problem are exact mathematical flows.

On a compact positive-speed domain, $\Phi_e$ may equal $\Phi$. To cover other estimates, it may be a declared bounded Lipschitz extension. Its discrepancy from the physical law must then be bounded explicitly and propagated. No assertion of uniform observability at zero speed is assumed. The target stopping transition must also be represented if included in the theorem.

The central derivation must include the radar reset, held gyro, ego-velocity estimation error, and common measurement noise in (10)--(11). A continuous-measurement observer theorem alone does not establish the sampled observation result.

### 4.2 Abstract nonlinear ego observer: an explicit mathematical interface

The new task is to couple target observation and ego response, rather than prescribe a particular ego-observer design. Use the following general nonlinear observer class. Let $\zeta_E$ contain all ego-estimator states, including any position/velocity filters, heading estimate, and measurement memory. Its sensor reset and intersample flow are

$$
\begin{aligned}
\zeta_E(t_j^+)&=P_E(\zeta_E(t_j^-),y_{p,j},y_{a,j},y_{r,j},u^-),\\
\dot\zeta_E&=f_O(\zeta_E,y_{p,j},y_{a,j},y_{r,j},u(t)),\\
(\hat p_E,\hat\psi_E,\hat v_E)&=H_E(\zeta_E).
\end{aligned}
\tag{12}
$$

$P_E,f_O,H_E$ are symbolic members of the declared observer class, not a reference to an unstated specific formula. They use only the listed measurements and known inputs. Identity reset is allowed for states corrected solely by the flow. A derived velocity is an internal output, not an additional independent observation.

The desired theorem may be parameterized by these maps. State precisely the regularity, estimation-bound, or incremental-stability assumptions it requires of them. Such assumptions are premises concerning the ego observer, not conclusions to infer merely from the available sensors. If a concrete member of this class is essential to the proposed construction, supply its equations and identify the extra observability assumptions.

At the present decision instant, assume a valid joint set containing the true ego state, target state, persistent parameters, and their dependence on the known observer states is available. This **current** enclosure is an initial condition. It does not by itself provide any future estimation-error bound; that is part of the requested joint propagation.

## 5. The mathematical gap and three different uncertainties

A partially coupled construction can propagate the ego state and ego observer together while maintaining a separate target-parameter family. A set of target parameters can also be intersected with the measurement history. For example, an average speed observation constrains $V_{T0}+A_T\Delta/2$, rather than the two parameters independently.

Neither operation accounts for the complete future chain in Section 1. The required extension includes the physical target, $\hat X_T$, $\hat y$, future radar observations, and the dependence of ego control on the resulting target estimate.

It is essential to distinguish

$$
\underbrace{p_T-\bar p_T}_{\text{physical deviation from one forecast}},\qquad
\underbrace{p_T-\hat p_T}_{\text{estimation error}},\qquad
\underbrace{(p_E-p_T)-(\bar p_E-\bar p_T)}_{\text{relative tracking error}}.
\tag{13}
$$

Two admissible constant velocities already cause physical target-position separation proportional to time. Constant uncertain acceleration can cause quadratic separation. Future observation can reveal which motion occurs without making those physical trajectories coincide.

The intended result concerns how future estimation and ego feedback control **relative** uncertainty. It is not a claim that every absolute reachable set contracts. Distinguish uncertainty across different future measurement histories from estimation error conditioned on one such history.

## 6. Required construction A: the causal nonlinear joint system

A possible joint state is

$$
z=(x_E,x_T^{\rm phys},\zeta_E,\hat X_T,\hat y,\theta_E,u^-,\eta_\pi),
\tag{14}
$$

where $x_T^{\rm phys}$ carries the physical target motion and its persistent parameters, and $\eta_\pi$ carries any encounter, reference, and policy memory. Avoid duplicate independent copies of the same parameter. The present observer and memory states are known; their future values vary with future observations.

Let $I_k$ be the joint information set at a control decision after the available measurements have been assimilated. Policies must be causal:

$$
u_{i|k}=\pi_{i|k}(\mathcal H_{i|k}),
\tag{15}
$$

where $\mathcal H_{i|k}$ consists only of measurements, estimator states, and past controls available at that decision. A nonlinear relative-state policy based on $(\hat\rho,\hat q,\hat s,\hat v_E,\hat\psi_E)$ and policy memory is one possible class; another is welcome if justified.

All physical worlds indistinguishable at the present instant must receive the same first command. Later commands may differ only after their observation histories differ. A policy depending on the future true target state is inadmissible.

Derive the nonlinear transition

$$
z_{i+1}=\mathcal G_h(z_i,\nu_i;\pi_i),
\tag{16}
$$

from (2)--(12), with the declared order of sensor sampling, resets, observer flow, control selection, and held-input vehicle motion. Here $\nu_i$ contains the primitive sensor noises at all substeps and any separately assumed residual disturbance over the control interval. Persistent parameters remain state coordinates.

Each future measurement must come from the same physical branch that generates the associated observer state and control. Do not choose an estimate independently inside an error ball. Give a precise prior/posterior convention so the boundary measurement is neither skipped nor used twice.

Please also explain the relation between the predicted policy and receding-horizon application of the first command. The retained policy tail must remain an admissible candidate after the next measurement, even when the next optimum selects a different policy.

## 7. Required construction B: nonlinear closed-loop error bounds

Derive the coupled nonlinear errors for physical tracking, ego estimation, target estimation, predictor memory, and relevant relative pose. In particular, derive how the radar correction and target-estimate-dependent control enter the relative dynamics.

Propose a nonlinear feedback law or a mathematically checkable condition on its class that bounds the relative error. Possible approaches include an incremental Lyapunov function, an input-to-state comparison, or a nonlinear set-valued propagation argument. The result must show how future target information helps; merely assuming the desired small tube does not answer the question.

If a scalar comparison such as

$$
\mathcal E_{i+1}\le a_i(\mathcal E_i)+b_i(\|\nu_i\|)
\tag{17}
$$

is used, define what $\mathcal E_i$ bounds, derive the functions and their domain, and explain the correlations retained before taking that comparison. A vector comparison or a direct set relation is equally acceptable. No strict contraction at every step is required.

The same mismatch and noise sources may enter both physical and estimated states. They must enter the joint derivation once as shared variables, allowing their actual cancellation or reinforcement. The current enclosure already includes past uncertainty effects; those past effects cannot be re-added as fresh disturbances at each predicted step. Future parameter-dependent motion must still be propagated.

Persistent parameters and common translations need not converge. The quantities requiring control are those that determine relative collision geometry and admissible feedback commands. Identify restrictions caused by insufficient steering/braking authority, poor observability, target stopping, or an invalid vehicle domain. Do not hide such restrictions by truncating uncertainty sets.

## 8. Required construction C: joint reachable sets and conditioning

Construct sets $\mathcal R_{i|k}$ with

$$
\mathcal R_{0|k}=I_k,\qquad
\{\mathcal G_h(z,\nu;\pi_i):z\in\mathcal R_{i|k},\ \nu\in\mathcal N_i\}
\subseteq\mathcal R_{i+1|k}.
\tag{18}
$$

Exact reachable sets are permissible for the theoretical construction. If outer sets are used, their containment must follow from a nonlinear bound or set argument. Preserve joint target/ego/observer dependence before projecting to collision geometry.

A family conditioned on possible observation histories is also permissible. Define explicitly whether each set is conditional or a union over histories, and enforce nonanticipativity. A small conditioned estimation error cannot silently replace the union of physically different future positions.

After applying the first common command and receiving the next observations, require

$$
z_{k+1}\in I_{k+1}\subseteq\mathcal R_{1|k}.
\tag{19}
$$

One conceptual update is

$$
I_{k+1}=\mathcal R_{1|k}\cap\mathcal Q_{k+1}\cap\mathcal A_{k+1},
\tag{20}
$$

where $\mathcal Q_{k+1}$ is a valid observer enclosure and $\mathcal A_{k+1}$ encodes the received observations, known estimator states, and input history. Every constituent must use the same physical/noise contract. An empty intersection is a violated premise, not justification for discarding uncertainty.

Show that conditioning and any coordinate recentering preserve a feasible restriction of the old future policy. If the policy representation changes with its reference, establish closure under that change. Existence of a physically safe continuation alone is insufficient if the next admissible policy class excludes it.

## 9. Required construction D: nonlinear collision and input admissibility

Let the known body-fixed rectangular footprints, including reference-point offsets, be $\mathcal B_E$ and $\mathcal B_T$. Their occupied sets are

$$
\mathcal V_E=p_E+R(\psi_E)\mathcal B_E,\qquad
\mathcal V_T=p_T+R(\psi_T)\mathcal B_T.
$$

For a unit normal $n$, define the exact separation gap

$$
g_n(z)=\min_{a\in\mathcal B_E}n^\top[p_E+R(\psi_E)a]
       -\max_{b\in\mathcal B_T}n^\top[p_T+R(\psi_T)b].
\tag{21}
$$

A sufficient robust stage condition with a common nonnegative slack is

$$
\inf_{z\in\mathcal R_{i|k}}g_{n_{i|k}}(z)\ge-\xi_{i|k},
\qquad \xi_{i|k}\ge0.
\tag{22}
$$

The normal is a continuous geometric witness, not a direction chosen from a finite bank. A common witness for the whole set is sufficient but potentially conservative. An exact nonlinear separation condition or conditional witnesses are acceptable alternatives, provided their quantifiers are explicit and their shifted certificates remain admissible. A history-dependent witness cannot rely on information unavailable in that history.

Use the joint relative-pose set to bound (21). Independent worst-case ego and target position radii can discard useful shared-pose dependence. Require every command generated by the policy to satisfy the input constraints, together with any separately declared state or input-memory conditions.

At zero slack, nonnegative gap excludes interior overlap but permits contact at equality. Strict positive clearance requires an explicit strictness premise; it does not follow from a nonnegative gap alone.

## 10. Required construction E: the PCBF recursive-feasibility theorem

Assume a nonempty, downward-closed terminal family $\mathfrak B$ of information sets. There exists a causal terminal policy such that, for every set in $\mathfrak B$, its admissible successor remains in $\mathfrak B$ with zero collision deficit. This assumption must use the same dynamics, uncertainty, observation structure, and collision definition as the rest of the problem. A particular terminal set or braking law need not be constructed.

For fixed $N$, consider the exact nonlinear problem over admissible causal policies, joint outer sets or exact reachable sets, geometric witnesses, and common stage slacks:

$$
V_P(I_k)=\min\sum_{i=0}^{N}\xi_{i|k},
\tag{23}
$$

subject to (18), input/state admissibility, (22), and $\mathcal R_{N|k}\in\mathfrak B$. The terminal assumption permits $\xi_{N|k}=0$. Apply the first command of an attained primary minimizer; state assumptions needed for attainment.

After the next measurement, the proposed candidate is the restricted old policy tail plus the terminal policy, with slack sequence

$$
(\xi_{1|k},\ldots,\xi_{N|k},0).
$$

Establish sufficient conditions for this candidate to belong to the next problem and hence for

$$
V_P(I_{k+1})\le V_P(I_k)-\xi_{0|k}\le V_P(I_k).
\tag{24}
$$

Separate the proof obligations:

1. The true joint state remains inside the predicted and conditioned information sets.
2. Future observation and control obey causality and the same uncertainty contract.
3. Restricting the retained policy and certificates to the conditioned set preserves admissibility.
4. The terminal extension closes the shifted horizon.
5. The slack sequence gives (24).

The monotonic quantity is the slack **sum**, not each future slack individually. Positive slack is allowed but is not a safety guarantee. Nonincrease alone does not prove finite-time arrival at zero. If a feasible zero-slack solution is attained, explain precisely why the zero-slack property and discrete nonpenetration persist throughout the active encounter.

If exact global minimization is unnecessary for a weaker retained-feasible-plan theorem, state it separately using a feasible cost $J_{k+1}\le J_k-\xi_{0|k}$. Do not identify such a stored cost with the optimal value function.

## 11. Requested derivation and mathematical counterexamples

Please provide:

1. A diagnosis of which uncertainty can be reduced by observation and feedback, and which physical uncertainty cannot.
2. An explicit nonlinear joint state and sampled transition, including target-observer flows and measurement resets.
3. A causal nonlinear feedback construction or checkable feedback-class condition accounting for target estimates and control authority.
4. Nonlinear joint-error or reachable-set propagation with shared noise and persistent parameters.
5. Relative collision conditions, input admissibility, and measurement conditioning.
6. A theorem and proof connecting these constructions to (24), clearly separating premises from proved conclusions.
7. A step-by-step mathematical procedure for initialization, prediction, conditioning, and policy shifting.
8. Any obstruction and the weakest additional assumption needed if the requested result cannot be established.

Use analytical examples or counterexamples to expose any necessary assumptions, particularly indistinguishable target motions, loss of observability at rest, and insufficient ego control authority. For two initially indistinguishable admissible motions, the first command must be identical and later commands may differ only after the observation histories do. Growing absolute separation between worlds need not imply growing relative collision error. Such examples should support the theorem or delineate its scope.

Please do not replace the requested derivation with a covariance prediction, an unexplained future measurement reset, an assumed uniformly narrow tube, or a generic recommendation to use output-feedback MPC. The central task is to establish the missing **nonlinear target-observer and ego-feedback coupling**, then prove that its information updates preserve the PCBF shifted candidate.

This is a mathematical problem specification, not a claim that the requested result is already proved. The PCBF shift argument motivates the desired conclusion; its extension to this uncertain moving-target, sampled-observation setting must be established explicitly.
