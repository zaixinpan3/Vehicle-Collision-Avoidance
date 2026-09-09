# Terminal invariance audit and conditional predictive CBF theorem

Research and implementation date: September 8, 2026.

The subsequent hard-margin, retained-deadline implementation and its conditional
encounter proof are in [HARD_PREDICTIVE_CBF.md](HARD_PREDICTIVE_CBF.md). It uses
perception exit rather than the retired invariant terminal set audited here.
References below to the current online controller concern the default
version-10 lookahead policy; the nonlinear counterexamples remain applicable.

## Result and scope

The dissipative terminal construction evaluated in this audit yields a locally Lipschitz,
nonsmooth CBF for its declared, disturbance-free, zero-speed scheduled affine
model and fixed safe pose halfspaces. This result is proved below and evaluated
in a self-contained mathematical experiment. The retired runtime utility and
its unused rest-tail prediction path have been deleted; no copy is retained
as an alternative controller.

Huang, Wang, Margellos and Goulart's safe-MPC argument supplies an appropriate
conditional predictive extension: a safety-slack value function is nonincreasing
because a feasible plan can be shifted and completed inside an invariant terminal
set. A fixed affine auxiliary problem using our terminal set implements that
argument in `scripts/runTerminalCbfProofAudit.m`.

**This does not establish a CBF for the current online vehicle controller.** The
old terminal set contains a boundary with unavoidable outward motion under the
nonlinear bicycle, so it is not a control-invariant set for that model. Its fixed
rest input also cannot preserve a bounded pose budget against persistent forcing.
These are explicit mathematical obstructions, beyond the absence of an online
terminal constraint. Adding the old constraint to the current SOCP would not
repair them. No such integration, zeroing of model uncertainty, additional target
route assumption, or backup execution is introduced by this operation.

## 1. Research method and primary sources

The question is whether the existing terminal set can support a CBF claim for
the current finite-sensing, nonlinear, sampled controller. The method is a
premise-by-premise proof audit, constructive derivation and counterexample search.
This is a focused technical investigation, not a systematic literature review.
Source selection favors complete original papers on predictive barriers,
sampled execution and output-feedback terminal conditions. Search terms included
"predictive control barrier functions terminal set recursive feasibility" and
"finite horizon control barrier function terminal set time varying obstacles".

| Source and material examined | Use and limitation |
|---|---|
| Huang, J., Wang, H., Margellos, K., and Goulart, P. (2025), *Predictive Control Barrier Functions: Bridging model predictive control and control barrier functions*, ECC, pp. 2623–2629. User-provided published PDF in `reference/`; Definition II.1, Eq. (8), Lemmas III.1–III.5, Eqs. (10)–(11), Theorem 4. [Author preprint](https://arxiv.org/abs/2502.08400). | Main value-function/shift argument. Safety slack is distinct from CLF slack. We do not import maximal-invariant-set or recovery claims for our online solver. |
| Wabersich, K. P., and Zeilinger, M. N., *Predictive control barrier functions: Enhanced safety mechanisms for learning-based control*, [full text, version 3](https://arxiv.org/html/2105.10241v3), Sections II-A and III. | Terminal continuation and the distinction between hard feasible safety and soft recovery. Its tightened recovery formulation is not substituted for our objective. |
| Breeden, J., Garg, K., and Panagou, D., *Control Barrier Functions in Sampled-Data Systems*, [full text, version 2](https://arxiv.org/html/2103.03677v2), Section II and Theorem 1. | Sample-node invariance alone does not certify intersample separation. |
| Köhler, J., Müller, M. A., and Allgöwer, F., *Robust output feedback model predictive control using online estimation bounds*, [full text](https://arxiv.org/html/2105.03427), observer bounds and robust MPC terminal assumptions. | Estimator error bounds and robust continuation are separate hypotheses; a high-gain observer alone does not establish the required tube property. |

Titles, authors and the relevant text were checked against the supplied PDF and
primary preprints. No secondary summary is used as a theorem premise. Derivations
and counterexamples below are project analysis. AI-assisted research tools were
used for source inspection, derivation, implementation and validation; numerical
checks are not presented as machine-verified proofs or independent peer review.

## 2. The affine terminal dynamics under examination

Write the terminal state as \(x=(p,v)\), with

\[
p=(s,d,e_\psi),\qquad v=(v_x,v_y,r).
\]

For fixed scheduled curvature, zero scheduled speed and the admissible constant
input \(u_R=(0,-b_\star/g_\beta)\), the existing affine model has

\[
\dot p=Gv,\qquad \dot v=Fv. \tag{1}
\]

There is no persistent additive disturbance in (1). The longitudinal channel is
independent, \(\dot v_x=-a v_x\), \(a>0\), so the domain

\[
\mathcal D_R=\{(p,v):v_x\ge0\}
\]

is invariant. Let \(C\) be the Metzler comparison matrix of \(F\): retain its
diagonal and take absolute values off diagonal. Assume \(C\) is Hurwitz. Define

\[
M=|G|(-C)^{-1}\ge0,\qquad q>0,\qquad Cq<0. \tag{2}
\]

A positive scaling of \(q\propto(-C)^{-1}\mathbf1\) preserves \(Cq<0\).
The numerical vector of ones fixes a comparison direction with compatible
component units; it is not a disturbance assumption. Any application must
choose the scale to enforce its velocity, slip and force constraints. The
fixed input must be admissible. The research LP below is restricted to the
longitudinal subspace, with zero steering, lateral velocity and yaw rate.

Let the fixed terminal pose region be \(Ap\le b\). Its rows must already account
for the full vehicle footprint and any obstacle obligations throughout the
terminal continuation. The scalar input bounds, velocity envelope, and fixed
pose halfspaces concern the declared affine model. They are not a nonlinear
vehicle certificate. Entry slew and a pending input queue need separate checks.

## 3. A terminal barrier, rather than only a membership test

For positive row scales \(\sigma_j\), define

\[
h_f(p,v)=\min\left\{
 \min_j\frac{b_j-A_jp-|A_j|M|v|}{\sigma_j},
 1-\max_i\frac{|v_i|}{q_i}\right\},\qquad
\mathcal X_f=\{x\in\mathcal D_R:h_f(x)\ge0\}. \tag{3}
\]

A supplied strict interior pose establishes nonemptiness. Compactness of the
pose polytope, needed for the predictive theorem below, must also be established
by its geometry. The function
is piecewise affine and locally Lipschitz. We do not claim it is differentiable
at changes of active face or signs. The domain restriction \(v_x\ge0\) is checked
separately; putting \(v_x\) into a margin required to increase would be incorrect
for a stopping trajectory.

**Proposition 1 — terminal invariance and barrier inequality.** Under (1)–(2),
the admissible constant input \(u_R\) makes \(h_f(x(t))\) nondecreasing for all
\(x(0)\in\mathcal D_R\). Every nonempty superlevel set of (3) is invariant under
that input. In particular, \(h_f\ge0\) implies \(Ap\le b\) and \(|v|\le q\).

**Proof.** Componentwise comparison gives \(D^+|v|\le C|v|\). For each pose row,

\[
\begin{aligned}
D^+\big(A_jp+|A_j|M|v|\big)
&\le |A_j||G||v|+|A_j|MC|v|\\
&=0,
\end{aligned}\tag{4}
\]

because \(MC=-|G|\). Thus each pose margin in (3) is nondecreasing. With
\(y=\max_i|v_i|/q_i\), positivity of the off-diagonal entries gives, at every
active component,

\[
D^+y\le \max_i\frac{(Cq)_i}{q_i}y=-\lambda y,
\quad \lambda=\min_i\frac{-(Cq)_i}{q_i}>0. \tag{5}
\]

Therefore \(1-y\) is nondecreasing too. The minimum of these nondecreasing
quantities is nondecreasing. These Dini/comparison inequalities apply to the
absolutely continuous affine trajectories, including zero velocity components.
Domain invariance follows from the independent damped longitudinal channel.
This proves the claims. Equivalently, \(B_f=-h_f\) satisfies

\[
B_f(\Phi_h(x,u_R))-B_f(x)\le0 \tag{6}
\]

for every held duration \(h\ge0\). This is a nonsmooth barrier statement; a
classical gradient/Lie-derivative formula at a nonsmooth face is unnecessary.

**Exact set evaluation in algebra.** Enumerating the eight velocity sign vectors
turns (3) into \(h_f(x)=\min_l(\bar b_l-\bar A_lx)\). For a signed-generator
enclosure \(Z=z+L[-1,1]^m\),

\[
\inf_{x\in Z}h_f(x)=\min_l\left(
\bar b_l-\bar A_lz-\|\bar A_lL\|_1\right). \tag{7}
\]

The exchange of two infima is exact. This is implemented without converting a
propagated generator matrix back into a coordinate box. Independently maximizing
the pose and absolute-velocity terms would generally lose their correlations.
Ordinary floating-point evaluations of (2) and (7) remain numerical evaluations;
the implementation is not an interval-arithmetic proof checker.

## 4. Huang-style predictive extension

Use \(B_N\) for the auxiliary safety value, reserving \(V_{\mathrm{CLF}}\) for the
cruise Lyapunov function. Consider a *fixed* continuous discrete transition \(f\),
compact admissible inputs, normalized continuous state rows \(c(x)\le0\), and
the following problem for \(N\ge1\):

\[
\begin{aligned}
B_N(x)=\min_{u_{0:N-1},\xi_{0:N-1}}&\ \sum_{i=0}^{N-1}\xi_i\\
\text{subject to }&\ x_0=x,\quad x_{i+1}=f(x_i,u_i),\\
&u_i\in\mathcal U,\quad x_i\in\mathcal D_R,\\
&c(x_i)\le\mathbf1\xi_i,\quad \xi_i\ge0,\\
&x_N\in\mathcal X_f.
\end{aligned}\tag{8}
\]

Assume: (i) the nonempty compact terminal set is inside the unrelaxed state set
and is invariant under an admissible terminal law for **the same transition**;
(ii) optima are attained; and (iii) \(B_N\) is continuous on its feasible domain
\(\mathcal D_N\). A fixed affine, polyhedral problem such as the auxiliary LP
below supplies a continuous piecewise-affine value on its feasible domain.
Polytopic state/input sets alone are not a sufficient continuity argument for
an arbitrary nonlinear, relinearized or changing-maneuver optimization; we keep
that regularity obligation explicit rather than importing it from a citation.

**Proposition 2 — predictive CBF.** Under these assumptions, \(\mathcal D_N\) and
\(\mathcal C_N=\{x:B_N(x)=0\}\) are invariant under the first input of a global
minimizer of (8), and

\[
B_N(f(x,u_0^*))-B_N(x)\le-\xi_0^*\le0. \tag{9}
\]

Thus \(B_N\), with safe zero sublevel set \(\mathcal C_N\), is a discrete-time
CBF in the sense of Huang et al., Definition II.1.

**Proof.** Shift an attained minimizing sequence and append the terminal law:

\[
\tilde u=(u_1^*,\ldots,u_{N-1}^*,\kappa_f(x_N^*)),\qquad
\tilde\xi=(\xi_1^*,\ldots,\xi_{N-1}^*,0). \tag{10}
\]

All retained constraints are unchanged. Terminal invariance admits the last
input and the new endpoint. Since \(\mathcal X_f\subseteq\{c\le0\}\), the
appended stage needs zero slack. The new optimum is no larger than this feasible
candidate's cost, proving (9) and domain invariance. Nonnegativity and attainment
give \(B_N=0\) exactly when a zero-slack trajectory to \(\mathcal X_f\) exists.
Equation (9) preserves this zero set, which lies in the physical state set at
sample instants. Continuity supplies the remaining stated CBF regularity.

This is the argument of Huang et al., Eqs. (8)–(11) and Theorem 4, specialized to
our terminal construction. Strict decrease is unnecessary for invariance. We
do not infer exact cruise convergence, a globally maximal safe set, or
continuous-time collision safety from (9). For sampled execution, a transition
must additionally constrain its entire held trajectory, not just its endpoints.

**Relation to the current objective.** The safety slack \(\xi\) is not our CLF
slack \(\delta\). Neither the CLF value nor the joint state/input/slack objective
is identified with \(B_N\). On \(\mathcal C_N\), *any feasible hard-safety MPC*
with the same terminal and dynamics conditions selects a zero-cost minimizer
of (8). It can therefore minimize the existing performance objective, including
input deviation from the CLF operating point and squared CLF slack, without an
extra auxiliary optimization at runtime. Outside \(\mathcal C_N\), the
nonincrease proof requires the auxiliary optimum or an independently proved
descent rule; arbitrary weighted performance tradeoffs do not supply it.
Positive auxiliary safety slack is not permission to execute unsafe controls.

## 5. What must also hold for this estimator/controller

For uncertain observations, the state of the proof is the information set,
including target motion parameters, active obligations, certificate clock and
committed input queue. Every possible successor must admit the conditioned tail.
The policy class must be closed under conditioning and terminal extension;
enclosures must contain the true trajectories. A new measurement can tighten a
retained set, but cannot silently replace its covered futures. A relinearized
inner approximation must retain a feasible representation of the old witness.

For the old affine terminal model with uncertain initial state and no process
disturbance, its constant \(u_R\) is common to all states. Propagating signed
generators preserves the exact affine image, so (7) supplies the needed set
membership check. This is demonstrated by the auxiliary audit. It does not
prove these properties for the online changing linearizations, noisy NRMM
interface, or nonzero process residual.

Moving targets require terminal collision coverage for the remaining obligation,
or a valid finite-encounter exit/transfer. The user's finite sensing convention
does not require extrapolating a target forever. It does require that the
certificate cover the active encounter and any applicable sensing handoff.
Appending a timeless ego-rest step is not such a proof. A shrinking-clock
reach–avoid construction is another possible route, discussed in
`ENCOUNTER_SCOPED_CBF_CLF.md`; Huang's fixed-horizon invariant-terminal theorem
does not establish it automatically.

## 6. Obstructions to applying the old terminal set directly

### Nonlinear boundary counterexample

For a straight road, the scheduled rest model uses \(\dot d=v_y\), and
\(M_{d,v_x}=0\). Its terminal set consequently admits

\[
d=b_d,\quad e_\psi=\theta>0,\quad v_x>0,\quad v_y=r=0
\tag{11}
\]

when the other pose and velocity bounds have slack. For the current nonlinear
bicycle, however,

\[
\dot d=v_x\sin\theta>0. \tag{12}
\]

This instantaneous velocity is independent of steering and drive/brake inputs.
The existing nonnegative velocity excursion term cannot compensate for outward
motion from the boundary. Thus the old set is **not control invariant** for the
nonlinear model, even with perfect state knowledge and no external disturbance.
This is stronger than showing that a particular terminal input performs poorly:
no bounded force/steering action can prevent immediate exit from that admitted
boundary state. It does not prove that a different, smaller nonlinear terminal
set or another CBF is impossible.

The audit uses \(b_d=0.05\) m, \(\theta=0.2\) rad and \(v_x=0.8\) m/s. It also
propagates an interior initial state under zero input for 0.5 s with the actual
nonlinear prediction kernel, separately from the boundary argument. The
resulting lateral displacement exceeds 0.05 m while the affine certificate
remains positive. This is a terminal-set counterexample, not a collision claimed
to have occurred under the online controller.

### Persistent forcing counterexample

An admitted longitudinal residual \(\epsilon>0\) changes the terminal channel to

\[
\dot s=v_x,\qquad \dot v_x=-a v_x+\epsilon.
\]

Even a small steady velocity \(\epsilon/a\) causes unbounded station drift. With
the old budget \(s+v_x/a\le b_s\),

\[
\frac{d}{dt}(s+v_x/a)=\epsilon/a>0. \tag{13}
\]

Therefore its zero-disturbance proof does not extend to arbitrary persistent
forcing. Increasing a finite pose margin postpones violation; it does not make
that set invariant under \(u_R\). The proof therefore excludes any nonzero persistent residual for this fixed
terminal policy; this is a model assumption, not an online configuration change.

### Moving-target counterexample

A stopped ego remains in its own terminal set while a target with constant
velocity reaches it. The audit uses a target initially 10 m ahead, moving at
-2 m/s, and evaluates overlap after 3 s. This respects a short, constant-motion
encounter. It disproves the inference from ego rest to joint collision safety;
it is not an assertion that the actual controller would choose to remain there.

## 7. Implementation decision and validation

The executable audit evaluates (2), (3) and (7), checks exact affine flow and
Huang's shifted LP candidate, and reproduces the three counterexamples. It is
an isolated research experiment, not an admission utility or executable backup
controller. Its local equation helpers are not called by the online pipeline.

The obsolete `terminalDissipation` and `stateUncertainty.terminalRest` APIs,
`ltvBicycleModel.predict`/`brakingSchedule` rest-tail path, unused
`terminal.backupDeceleration` setting, and solver-outage continuation scenario
have been removed. Applicable finite-prediction and uncertainty tests now use
the maintained interfaces. Earlier stationary-pose/dissipation documents are
consolidated here; their implementation history remains in Git (baseline
`64b397e61e811e6e588997091d9869a7db51143b`). There is one maintained online
controller path, with solver failure terminating execution.

The current online controller remains at `recursiveFeasibilityClaimed=false`.
Its default finite-sensing branch has no dissipative terminal constraint; its
future nominal stages, changing local models and empirical residual settings
also fail to establish the assumptions of Proposition 2. Formal integration
requires a terminal set for the actual nonlinear inclusion, terminal target
coverage or valid exit, full execution-tube certification, and a representation
that preserves feasible tails. Actuation queues, slew bounds and numerical
deadlines belong in that argument. No stored controller is executed on failure.

Reproduce the mathematical-model audit with:

```matlab
addpath('scripts');
report = runTerminalCbfProofAudit(OutputDirectory="/absolute/output/path");
assert(report.auditPassed);
assert(~report.fullControllerCbfEstablished);
```

The auxiliary program is a four-stage, 100 ms fixed affine LP using the same
terminal matrix. Steering and lateral/yaw initial states are zero; braking
ratio lies in [-1,0]. It preserves a signed-generator initial enclosure with
station radius 0.01 m and longitudinal-velocity radius 0.001 m/s. It runs five
updates for an initially safe speed and an initially constraint-violating speed.
The latter is a mathematical safety-recovery example, not an executable vehicle
trial. No random seed or measurement noise is used. No runtime qualification is
claimed. See [the audit results](../scripts/TERMINAL_CBF_PROOF_RESULTS.md) for measured results.

## 8. Adversarial review and research decision

| Checkpoint | Challenge | Resolution |
|---|---|---|
| Scope | Would an affine theorem answer the nonlinear controller question? | State the affine result separately and actively search for nonlinear counterexamples. |
| Synthesis | Is strict barrier decrease needed? | Huang's shift gives nonincrease; invariance does not need an added tightening schedule. |
| Synthesis | Can CLF slack or the existing performance cost stand in for safety slack? | No. Define \(B_N\) separately; a hard-feasible performance optimizer is sufficient only on its zero set with matching terminal premises. |
| Synthesis | Does a failed terminal input prove that no terminal control can work? | Use the instantaneous outward boundary velocity in (12), independent of input, to prove noninvariance of this set. |
| Final | Does a passing auxiliary LP qualify the online controller? | No. Preserve the explicit false online-proof flag and all three counterexamples. |
| Final | Do citations prove nonlinear continuity, estimator containment or numeric timing? | No. Keep them as unestablished application conditions. |

The terminal affine CBF and conditional predictive theorem are established in
the stated mathematical scope. A formal CBF proof for the present nonlinear
finite-sensing controller is not established, and the old terminal set cannot
be inserted unchanged to obtain one. This limitation follows from the model
counterexample, independently of the selected literature or solver performance.
