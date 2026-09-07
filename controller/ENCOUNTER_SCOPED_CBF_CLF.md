# Encounter-scoped predictive CBF, sampled-data CLF, and maneuver optimization

Decision date: September 7, 2026. Status: governing mathematical specification;
the version-8 MATLAB controller does not yet implement this contract.

The terminal requirement is **certified discharge of an encounter**. While an
encounter is active, the controller must preserve a certified safe continuation
at every time. Ordinary encounters do not require a permanently invariant
joint ego–target terminal set. A finite safe prediction with an uncertified
endpoint does not satisfy this requirement either.

This specification supersedes the terminal, visibility, prediction-node, and
current-state-only CLF requirements in earlier controller design notes.
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md) documents the existing
runtime and its narrower guarantee. The construction below is a project design
derived from the stated requirement; the cited papers supply its background,
not a theorem for this repository's implementation.

## 1. Safety obligation and encounter lifecycle

Let the sample period be \(h>0\), \(t_k=kh\), and issue a held actuator input

\[
u(t)=u_k,\qquad t\in[t_k,t_{k+1}).
\]

For target \(j\), admission and certified discharge occur at
\(\tau_j^{\mathrm{in}}\) and \(\tau_j^{\mathrm{out}}\). Define

\[
\mathcal A(t)=\{j:\tau_j^{\mathrm{in}}\le t<\tau_j^{\mathrm{out}}\}.
\]

Throughout every active encounter, require both

\[
\operatorname{dist}(\mathcal R_e(x(t)),\mathcal R_j(z^j(t)))
\ge d_{\min},\qquad j\in\mathcal A(t),
\tag{1}
\]

and existence of a valid certified continuation. Here the occupied sets are
the complete vehicle rectangles. Continuation means safe execution to a
certified exit, connection to an already certified continuation, or admission
to a separately certified holding/handoff regime. Every connected certificate
must cover the transition into the next one; circular promises of future
certification do not establish continuation.

Each discharge guard \(I\in\mathcal E_j\) must establish that the current
conflict obligation ends under the admitted motion, route, and monitoring
contracts. For a crossing, the entire uncertain target footprint may clear a
defined shared conflict region. That geometric condition must be accompanied
by a contract that prevents an unprotected return to the same conflict, or by
a verified sensing/admission handoff before renewed conflict is possible.
The conflict region must cover the permitted ego maneuvers; changing corridors
cannot silently invalidate a previously used exit guard.

Loss of radar visibility, an absent target record, forecast expiry, positive
instantaneous distance, and ego rest are not discharge guards. Track identity
and encounter identity are separate: an occluded encounter remains active;
reacquisition must not create a gap or discard its obligation. Monitoring
premises include coverage, blackout limits, sensing/processing/actuation delay,
and sufficient time and control authority for renewed joint admission. A range
threshold alone supplies none of those guarantees.

The implementation may evaluate exit guards only at sample times and retain
all current obligations through the complete preceding held interval. This is
conservative and avoids an uncertified within-sample lifecycle transition.
Individual targets may be discharged at different samples. A certificate may
terminate only when all obligations assigned to it have been discharged or
transferred to a valid successor certificate. Road, actuator, and other ongoing
ego obligations must have an applicable controller at the handoff.

## 2. Information and finite-validity motion contracts

Retain the bicycle/tire dynamics as the computational model, with a physical
inclusion

\[
\dot x\in f_e(t,x,u)\oplus\mathcal W_e(t,x,u).
\tag{2}
\]

An affine scheduled approximation is usable only where its residual enclosure
covers the admitted physical dynamics throughout the tube, including the held
intervals. Exact integration of the affine model does not by itself enclose
the nonlinear physical model.

A nonsingular finite-duration target contract can use Cartesian motion and
body yaw separately:

\[
\dot p^j=v^j,\quad \dot v^j=a^j,\quad
\dot a^j\in\mathcal J_j(t,z^j),\qquad
\dot\psi^j=\omega^j,\quad\dot\omega^j\in\mathcal N_j(t,z^j).
\tag{3}
\]

No division by target speed is needed, including when its speed interval
contains zero. For constant componentwise jerk bounds \(\bar J\) and angular
acceleration bound \(\bar N\), a constant-acceleration/constant-yaw-rate nominal
flow has the sufficient radii, for \(0\le t\le T_{\mathrm{valid}}\),

\[
\begin{aligned}
\rho_p(t)&=\rho_p(0)+t\rho_v(0)+\tfrac12t^2\rho_a(0)
                  +\tfrac16t^3\bar J,\\
\rho_v(t)&=\rho_v(0)+t\rho_a(0)+\tfrac12t^2\bar J,\\
\rho_a(t)&=\rho_a(0)+t\bar J,\\
\rho_\psi(t)&=\rho_\psi(0)+t\rho_\omega(0)+\tfrac12t^2\bar N,\\
\rho_\omega(t)&=\rho_\omega(0)+t\bar N.
\end{aligned}
\tag{4}
\]

These follow by successive integration of the admitted derivative bounds.
Yaw support can use all orientations once its radius reaches \(\pi\).
Do not equate body yaw with velocity course without a separate kinematic
premise. The jerk and angular-acceleration bounds need their own motion
contract; estimator operating limits are not automatically future disturbances.
The existing frozen-curvature prediction can be retained as a computational
proposal if a finite nonsingular enclosure covers it and the physical futures.
Setting \(\bar J=0\) does not generally reproduce a turning trajectory with
constant curvature and constant tangential acceleration.

Use a sufficient information state

\[
I_k=(t_k,\mathcal B_k,\mathcal A_k,\mathscr C_k,u_{k-1},q_{k-1}),
\tag{5}
\]

where \(\mathcal B_k\) encloses the joint physical state, \(\mathscr C_k\)
retains the model, observation, route, and validity premises, and \(q\) is the
maneuver state. Fixed uncertain parameters, observation history needed for
future feasibility, and shared disturbances must remain represented in this
information state. Marginal boxes alone need not retain those correlations.

A valid measurement conditions the carried futures:

\[
\mathcal B_{k+1}=\operatorname{Reach}_h(\mathcal B_k,u_k)
                  \cap\mathcal M(y_{k+1}),\qquad
I_{k+1}\in\mathfrak F_h(I_k,u_k,q_k).
\tag{6}
\]

The corresponding future trajectory family must be a subset of the carried
family conditioned on that observation. Numerical equality of shifted arrays
is unnecessary. Pointwise overlap of unrelated forecasts is insufficient.
Any outer approximation used for conditioning must remain within a family for
which the witness is certified, or undergo independent recertification.
An empty intersection indicates an inconsistent contract; it is not successful
conditioning. Forecast replacement or a changed model does not inherit safety
without a verified replacement certificate.

Each contract records absolute validity times. Its required duration ends at
that encounter's certified discharge or valid transfer. No infinite target
trajectory is required. Bounds may grow during a finite certificate; neither
zero persistent residual nor infinite-time bounded pose drift is a universal
admission condition.

## 3. Certify the entire held interval

Let physical margins \(\widetilde g_\ell\) encode rectangle separation,
road containment, and applicable state constraints. Fix positive scales
\(s_\ell\) and use dimensionless \(g_\ell=\widetilde g_\ell/s_\ell\).
Scales must remain consistent on inherited certificates. For \(a=(u,q)\), set

\[
S_h(I,a)=\inf_{\eta\text{ consistent with }I}\;
          \inf_{0\le\tau\le h}\;
          \min_{\ell\in\mathcal L(I)}g_\ell(t+\tau,\zeta^\eta(\tau)).
\tag{7}
\]

The realization \(\eta\) includes initial uncertainty and disturbances;
the input is held over the whole interval. Thus \(S_h\ge0\) is the executed
interval certificate. Sampled-data CBF analysis explicitly distinguishes this
from satisfying a derivative condition only at sample times. See
[Breeden, Garg, and Panagou, Section II and Theorem 1](https://arxiv.org/html/2103.03677v2).

For a fixed unit Cartesian normal \(n\), rectangle half-length \(l\), half-width
\(w\), and body axes \(b_1,b_2\), use

\[
h_R(n,\psi)=l|n^\top b_1(\psi)|+w|n^\top b_2(\psi)|.
\]

Maximize this support over each admitted yaw interval. With position centers
\(\bar p\), componentwise position radii \(\rho^p\), and yaw-maximized
supports \(\bar h\), a physical clearance lower bound is

\[
\underline g_j(\tau,n)=n^\top(\bar p_e-\bar p_j)
 -\bar h_e-\bar h_j-|n|^\top(\rho_e^p+\rho_j^p)-d_{\min}.
\tag{8}
\]

Partition each held interval into cells of width \(\Delta_\ell\), using
one fixed normal \(n_{j\ell}\) in each cell. Bound the absolute rate of the
actual projected clearance by

\[
L_{j\ell}=\bar V_e+\bar V_j+R_e\bar\Omega_e+R_j\bar\Omega_j,
\qquad R_a=\sqrt{l_a^2+w_a^2}.
\tag{9}
\]

The rate bounds cover all actual admitted trajectories, not merely the nominal
centers. Rectangle support is Lipschitz even at an absolute-value corner. For
a required dimensionless margin \(\mu\), the endpoint conditions

\[
\begin{aligned}
\underline g_j(\tau_\ell,n_{j\ell})&\ge
     s_j\mu+\tfrac12L_{j\ell}\Delta_\ell+\epsilon_{\mathrm{num}},\\
\underline g_j(\tau_{\ell+1},n_{j\ell})&\ge
     s_j\mu+\tfrac12L_{j\ell}\Delta_\ell+\epsilon_{\mathrm{num}}
\end{aligned}
\tag{10}
\]

are sufficient. Every point is at most half a cell from one endpoint, so the
actual clearance is at least that endpoint's lower bound minus
\(L_{j\ell}\Delta_\ell/2\). The reserve \(\epsilon_{\mathrm{num}}\) must
cover proven numerical error, in physical margin units. The rate is for the
actual clearance; it need not equal the derivative of its conservative box
envelope. Both endpoints must use the same cell normal. Subdivision or direct
interval minimization may reduce conservatism.

Road-footprint, chart, tire, and other continuous state-domain constraints
need analogous swept enclosures or valid rate bounds. Unknown road segments
cannot silently lose required containment constraints. Certificate normals are
geometry choices; they are distinct from the optimized maneuver \(q\).

## 4. Finite reach–avoid predictive barrier

For the finite-to-exit construction, let a normalized exit margin satisfy

\[
E(I)\ge0\ \Longrightarrow\ I\in\mathcal E_{\mathrm{exit}}.
\tag{11}
\]

It includes current physical safety and verified discharge/handoff conditions
for all assigned obligations. Missing or expired discharge premises make the
stop branch inadmissible; represent it by \(E=-\infty\). With \(n\) remaining
held intervals, a policy \(\Pi\) comprises causal control/maneuver decisions
and a causal integer-valued stopping rule \(0\le\sigma\le n\). Define

\[
M(\Pi;I)=\inf_\eta\min\left\{
 E(I_{\sigma^\eta}^{\eta}),\;
 \inf_{\substack{0\le i<\sigma^\eta\\0\le\tau\le h}}
 \min_{\ell\in\mathcal L(I_i^\eta)}
 g_\ell(t_i+\tau,\zeta_i^\eta(\tau))\right\}.
\tag{12}
\]

Use \(\inf\varnothing=+\infty\); an empty admissible policy/action class has
value \(-\infty\). Actuator, physical-model domain, maneuver transition, and
contract-validity constraints remain hard. The admissible
class \(\mathfrak P_n(I)\) retains all relevant uncertainty/observation
branches. Its value is

\[
H_n(I)=\max_{\Pi\in\mathfrak P_n(I)}M(\Pi;I).
\tag{13}
\]

Assuming maxima are attained, \(H_n(I)\ge0\) is equivalent to existence of an
admissible robust safe continuation to certified exit within \(n\) steps.
Without attainment, a nonnegative supremum alone does not establish a
zero-margin feasible witness. A numerical implementation must verify a
feasible witness and its lower bound.

Time-varying reach–avoid problems permit reaching a destination without
requiring it to be invariant; see
[Fisac, Chen, Tomlin, and Sastry, Section 2 and Lemma 3.5](https://arxiv.org/html/1410.6445v2).
Here the destination is the verified discharge event in information space.
Under a sufficient information state, time-consistent successor families, and
admissible causal policy concatenation, the recursion is

\[
\begin{aligned}
H_0(I)&=E(I),\\
H_n(I)&=\max\left\{E(I),\;
 \max_{a\in\mathcal U_{\mathrm{adm}}(I)}
 \min\left[S_h(I,a),\;
   \inf_{I^+\in\mathfrak F_h(I,a)}H_{n-1}(I^+)\right]\right\}.
\end{aligned}
\tag{14}
\]

Shared latent parameters must not be reset independently between successor
branches. Otherwise this recursion can describe a different uncertainty model.
The \(E\) branch terminates a valid encounter certificate; it is not a command
to remain in that joint physical state forever. Evaluate and take an applicable
terminal branch before requesting another encounter action. For an active
encounter without a valid stop branch, require, with \(0<\gamma\le1\),

\[
\min\left[S_h(I,a),\;
 \inf_{I^+\in\mathfrak F_h(I,a)}H_{n-1}(I^+)\right]
 \ge(1-\gamma)H_n(I).
\tag{15}
\]

This barrier acts on the information state and certificate clock. It preserves
safety and continuation until discharge. Optimization value functions as
predictive barriers are supported by
[Wabersich and Zeilinger, Section III](https://arxiv.org/html/2105.10241v3),
whose terminal-CBF/slack-recovery construction differs from this exit-terminated
value. Its theorem is not transferred unchanged. Flow-based conditions can
also expose held-input effects on high-relative-degree safety margins; see
[Tan, Daş, Ames, and Burdick, Sections III–IV](https://arxiv.org/html/2411.17079v1).

## 5. Explicit predictive tracking CLF

Use a common tracking error and CLF across maneuver choices:

\[
e(t,x)=[d,e_\psi,v_x-v_{\mathrm{des}},v_y-v_y^{\mathrm{eq}},
 r-r^{\mathrm{eq}}]^\top,\qquad V(t,x)=e^\top P(t)e,\quad P(t)\succ0.
\tag{16}
\]

The explicit derivative is

\[
\dot V=2e^\top P\big(\partial_t e+D_xe\,f_e^\eta\big)
          +e^\top\dot P e.
\tag{17}
\]

It includes the derivatives of desired speed, reference heading, lateral
equilibrium velocity, and yaw rate, plus the coordinate-map derivatives. For
\(e=Sx-r(t)\) with constant \(S\), the error derivative is
\(S f_e^\eta-\dot r(t)\). Smoothness is required on each certified chart;
polyline vertices or reference jumps require a smooth representation or
explicit jump inequalities. Recomputing \(P\) or the reference at a sample
does not preserve the preceding Lyapunov value automatically.

For every interval before exit, with a fixed rate \(c>0\), impose

\[
\mathcal L_{V,h}(I_i,u_i)=\sup_{\eta,\,0\le\tau\le h}
  [\partial_tV+\nabla_xV^\top f_e^\eta+cV]
        (t_i+\tau,x_i^\eta(\tau),u_i)
 \le\delta_i,\qquad\delta_i\ge0.
\tag{18}
\]

Here \(\delta_i\) is fixed along that held interval and may be causal across
observation branches. Integration of \(\dot V+cV\le\delta_i\) gives

\[
V(t_i+\tau)\le e^{-c\tau}V(t_i)
       +\frac{1-e^{-c\tau}}{c}\delta_i,
\qquad0\le\tau\le h.
\tag{19}
\]

For continuous reference/metric choices, these estimates concatenate. If a
switch has \(V^+\le V^-+j_i\), its certified jump bound must also enter the
composed tracking estimate. Without such a bound, only each smooth interval's
statement holds. A finite upper residual bound and no hard slack cap preserve
the fallback candidate. Tracking penalties supplement this dissipation
constraint; they do not replace it. Safety does not depend on the slack size.
Yielding is allowed to incur slack against the common cruise reference.

This preserves the distinct safety and relaxed tracking roles of the
[CBF–CLF optimization framework of Ames, Xu, Grizzle, and Tabuada](https://arxiv.org/abs/1609.06408).

## 6. Executable certificate state and optimization

Maintain a verified witness and nonnegative lower bound

\[
c_k=(\Pi_k^{\mathrm{cert}},b_k,n_k,\text{contracts and proof data}),
\qquad M(\Pi_k^{\mathrm{cert}};I_k)\ge b_k\ge0.
\tag{20}
\]

Admission first establishes such a witness. For explicit maneuver choices
(e.g. pass left, pass right, yield/wait, or crossing precedence), let

\[
\ell_i=\|e_i\|_Q^2+\|u_i-u_i^{\mathrm{eq}}\|_R^2
 +\left\|\frac{u_i-u_{i-1}}h\right\|_S^2
 +\lambda_\delta\delta_i^2
 +\lambda_q\mathbf1_{\{q_i\ne q_{i-1}\}}.
\tag{21}
\]

The previous **applied** input determines the first rate term. A nominal
performance objective is permissible; a robust choice is

\[
J(\Pi,\Delta;I_k)=\sup_\eta\left[
 \sum_{i=0}^{\sigma^\eta-1}h\ell_i^\eta+
 V_{\mathrm{exit}}(I_{\sigma^\eta}^\eta)\right].
\]

At an active admitted encounter solve

\[
\begin{aligned}
\min_{\Pi,\Delta,\mu}\quad &J(\Pi,\Delta;I_k)\\
\text{subject to}\quad
 &I_0=I_k,\quad\Pi\text{ causal},\quad\sigma\le n_k,\\
 &I_{i+1}\in\mathfrak F_h(I_i,u_i,q_i)
       \text{ on every admitted branch},\\
 &\text{all actuator, model-domain, maneuver, and validity contracts},\\
 &M(\Pi;I_k)\ge\mu\ge(1-\gamma)b_k,\\
 &\mathcal L_{V,h}(I_i,u_i)\le\delta_i,\quad\delta_i\ge0
       \quad(i<\sigma).
\end{aligned}
\tag{22}
\]

There is no safety slack. A mixed/hybrid search or verified enumeration can
compare admissible maneuvers; selecting a separating normal alone does not
optimize the maneuver. The full problem is not generally a single convex QP:
future quadratic CLF residuals, variable geometry, stopping times, and discrete
choices require a justified transcription. A restricted open-loop witness is
permissible if it certifies every admitted realization; global optimality over
causal policies is not required for safety. Any approximations need sound
lower bounds and must preserve the inherited witness as a feasible candidate.

Independently verify the returned witness, swept margins, terminal guard,
contracts, and CLF bounds before accepting it. Set \(\mu_k\) to an actually
verified lower bound satisfying (22), rather than an unchecked solver variable.
Issue precisely its first accepted actuator vector under the stated execution
contract. A positive solver status, post-solve clipping, or accepting negative
safety margins within a feasibility tolerance does not certify execution.

After execution and a valid observation, unless the exit guard is satisfied,

\[
n_{k+1}=n_k-1,\qquad b_{k+1}=\mu_k,\qquad
\Pi_{k+1}^{\mathrm{cert}}=\operatorname{Tail}(\Pi_k\mid I_{k+1}).
\tag{23}
\]

The endpoint and its absolute validity deadline remain fixed. No terminal
target node is appended. Therefore

\[
b_{k+1}-b_k\ge-\gamma b_k,\qquad
H_{n_{k+1}}(I_{k+1})\ge b_{k+1}\ge0.
\tag{24}
\]

This is an executable barrier inequality on a verified lower bound in the
certificate-augmented state. It is not a claim that the optimizer computed
the exact \(H_n\). If \(b_k=H_{n_k}(I_k)\), (22) also implies (15).

On optimization failure or deadline miss, the incumbent witness is feasible
with \(\mu=b_k\ge(1-\gamma)b_k\). Execute its next held input and retain its
tail; carry or conservatively recompute the CLF slack. The representation must
be closed under conditioning and truncation. Re-linearization, re-anchoring,
reference changes, solver initialization, and the number of optimization
variables must not remove that feasible fallback. Solver latency and actuator
queues must be inside the execution contract; otherwise a stale first input
has no certified execution interval.

## 7. Conditional safety and continuation argument

Assume an initial admitted certificate; nonempty, sound physical information
sets; time-consistent causal futures; valid observation, model, execution, and
monitoring contracts; sound numerical interval bounds; valid discharge/transfer
guards; and closure of the witness class under conditioning and truncation.

For an accepted first action, (22) bounds every physical margin over the
entire next held interval. For every admitted observation, conditioning selects
futures already covered by that witness. Removing its executed prefix leaves
a tail with margin at least \(\mu_k\), to the same certified endpoint. This
proves (23)–(24) and supplies the fallback at the next sample. At any intermediate
time, finish the remainder of the certified held input, then execute the tail.
Continuation therefore exists between samples as well.

Induction gives physical safety and continuation availability while the
encounter remains active, and discharge only through a verified guard. With
an unchanged finite clock, \(n=0\) forces the exit branch, so this particular
construction also supplies finite exit. At no step is permanent joint
ego–target invariance used. The CLF estimate follows independently from (18)
and does not strengthen the safety theorem into a convergence theorem.

## 8. Waiting, renewal, and additional targets

Finite exit is a sufficient construction, not a universal safety requirement.
A replacement may extend the deadline only after its complete continuation
and validity are certified. If a common margin scale and unchanged obligations
are retained, it must also satisfy the carried barrier bound. Repeated verified
renewal preserves safety but need not imply eventual exit or a uniform duration
bound. Automatically resetting \(n\) establishes neither property.

When finite exit cannot be certified, a holding or renewal regime must have
its own proof covering execution and transfer before expiry. A stopped ego
with an active target is still in an encounter. A finite waiting prefix and an
expectation of new forecasts are insufficient. A genuinely nonterminating
encounter needs continuing safety authority on every executed interval; that
does not impose an invariant joint terminal set on every ordinary encounter.

New targets require joint admission with all existing active encounters and
their maneuver commitments. The old witness does not cover an unmodeled new
target. Detection must precede the point at which a certified joint response
becomes impossible under the admitted sensing/response contract. Failed joint
admission must not be hidden by omitting the target. Changing the active set
or normalization requires an explicit joint certificate transition, not
reuse of a scalar bound whose meaning has changed.

## 9. Runtime implementation obligations

The following are required changes, not completed MATLAB features:

| Area | Version-8 runtime | Required behavior |
| --- | --- | --- |
| Terminal | Ego rest/dissipation plus finite target endpoint | Exit or valid continuation/holding transfer with explicit guard |
| Clock | Fixed head/tail; append a rest input and target node | Preserve absolute deadline; condition and truncate; recertify any extension |
| Lifecycle | At most one currently published target | Encounter registry independent of visibility; joint admission and guarded discharge |
| Prediction | Curvature quotient; finite arrays without discharge metadata | Finite-validity nonsingular enclosure and retained future-contract identity |
| Ego residuals | Reject every nonzero persistent model residual | Admit bounded residuals when the complete finite tube and endpoint pass |
| Safety | Node halfspaces and node rectangle diagnostics | Sound swept collision, road, and physical-domain bounds |
| Observation | Target array shift equality; readmission on change | Conditioning/refinement of the certified future family |
| CBF | Checked finite plan without the new margin state | Verified \(b_k,n_k,\Pi_k\) and explicit barrier inequality |
| CLF | One relaxed current-state derivative; future values diagnostic | One bounded residual/slack per predicted held interval; full reference derivatives |
| Maneuver | One chosen convex domain; crossing reference heuristic | Optimize explicit admissible modes, controls, input smoothness, and switching cost |
| Fallback | Revalidate the shifted plan including a fresh last node | Retain the conditioned certified tail without requiring a new extension solve |
| Acceptance | Numerical residual tolerances with node-level scope | Conservative proof bounds; explicit scope, validity, guard, and failure diagnostics |

Own these changes in the existing controller modules and configuration; keep
the estimator responsible for truthful current enclosures. Proposed certificate
data include encounter IDs/status, assigned obligations, absolute contract
validity, joint tubes and latent parameters, held controls, maneuver state,
cell normals/rate/numerical bounds, CLF slacks and reference/metric versions,
exit/transfer proof data, remaining intervals, and the verified barrier margin.
A Boolean `exitCertified` supplied without verifiable premises is insufficient.
The certificate format must be versioned to prevent version-8 node certificates
from authorizing this stronger continuation guarantee.

The first implementation must supply concrete discharge/monitoring contracts,
physical residual/rate bounds, a sound CLF interval bound, and a witness class
whose closure is established. These quantities are not present in the current
public controller inputs. Keep the existing safety scope truthful until those
components and the following acceptance cases are implemented together.

## 10. Required behavioral validation

These are acceptance criteria for the redesign, not tests reported as run:

| Case | Required result |
| --- | --- |
| Safe finite prefix, no exit or handoff | Reject continuation admission |
| Target disappears, forecast ends, or ego stops before exit | Keep encounter obligation; do not declare discharge |
| Crossing footprint clears but return/monitoring premise is absent | Reject exit guard |
| Certified exit inside finite validity | Accept without post-exit target extrapolation |
| Collision between clear sample nodes | Reject through swept-interval verification |
| Endpoint normals differ | Use one common normal per cell or another valid interval proof |
| Low-speed target set contains zero with nonzero yaw rate | Finite nonsingular propagation; no curvature division |
| Bounded nonzero model residual | Retain residual; accept only if the complete certificate passes |
| New observation refines carried futures but changes array values | Retain valid conditioned tail |
| Disjoint observation or incompatible contract replacement | Report contract inconsistency; no inherited authority |
| Solver failures after one admission | Execute inherited inputs to guarded exit, with a shrinking clock |
| \(n=0\) with an active untransferred obligation | Reject; no automatic clock reset |
| Tracking conflicts with safety | Increase CLF slack without weakening safety |
| Changing reference/metric or chart transition | Account for derivative/jump terms in the tracking estimate |
| Competing left/right/yield choices | Compare explicit admissible maneuvers; preserve switch constraints |
| New target conflicts with incumbent maneuver | Require a jointly certified response, retaining every active target |
| Waiting or extended deadline | Accept only a complete verified replacement/holding contract |
| Solver success with an unsafe or numerically uncertified iterate | Reject and use a valid incumbent if available |

Model-level regressions, physical-plant containment experiments, and execution
deadline measurements must report their own scope. Finite simulations or dense
sample checks alone do not prove interval enclosure, discharge, or indefinite
renewability. No new controller run, tracking-decay result, or physical safety
validation is claimed by this specification update.

## Source verification and derivation scope

The five user-supplied references were checked against their arXiv metadata
and the linked full-text sections where stated, on September 7, 2026.
The Ames reference was checked at abstract/metadata level for the CBF–CLF
distinction. Breeden's sampled-data analysis, Fisac's reach–avoid formulation,
Wabersich's predictive value/terminal construction, and Tan's flow-based
sampled-data construction were inspected in full-text HTML at the cited
sections. They are theoretical background; no source establishes this
project's exit guard, physical uncertainty bounds, or numerical implementation.

Equations (4), (10), (12)–(15), and (20)–(24), the conditioning assumptions,
and the encounter lifecycle/acceptance requirements are the project-specific
derivations and design decisions stated here. The CLF comparison estimate
(19) follows directly by integrating (18). This is a focused specification
revision with AI-assisted source checking and mathematical/code inspection,
not an exhaustive literature review or an independent formal verification.
