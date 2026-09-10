# Hard predictive barrier with a retained encounter deadline

Retained-deadline construction implemented September 8, 2026; joint target
admission added September 9, 2026. This construction keeps collision, road,
actuator, slew and model-domain constraints hard. The only relaxation
variables are the existing CLF performance slacks. It establishes recursive
feasibility and interval safety **conditional on the declared prediction
inclusion and execution assumptions below**. It does not certify the nonlinear
physical vehicle merely from exact target prediction.

## 1. Runtime contract and use

The controller uses this construction for every call:

```matlab
cfg = collisionAvoidanceControllerConfig();
certificate = [];
[command, inputs, problem, certificate] = ...
    collisionAvoidanceController(ego, targets, road, cfg, certificate);
```

The initial call accepts zero or more identified targets, each with a
`finite-sensing-motion-v1` descriptor, and requires timestamped complete perception:
`ego.perception = struct("time", ego.stateTime, "range", R,
"completeWithinRange", true)`, where finite `R > 0` is a circular,
center-based detection radius in metres. Every target present at admission
must be included jointly. The descriptor's physical motion bounds must remain
valid through the complete admitted horizon, including nonzero Cartesian jerk
when appropriate. Ego uncertainty uses `controllerStateErrorBound`; target
position uncertainty uses `targetPositionInertialErrorBound`.

Each subsequent call supplies the certificate, the next scheduled timestamp,
and the exactly executed `heldActuatorInput`. The road, route, configuration
and sensor radius must remain unchanged during the witness. Missing target
measurements do not renew the forecast or discharge the obligation. A missing
target whose entire retained set is still in complete perception is rejected.
New targets trigger the joint admission check described below. Contradictory
measurements and changed execution contracts are rejected; an error in those
cases is not a protective action.

Every continuation attempts the same hard-margin LP and subordinate CLF SOCP
with the executed prefix fixed. A replacement must preserve the independently
verified margin. If it fails verification, the checked incumbent remains the
execution witness. This is witness selection within one certificate, without
a second control law or a selectable execution policy. The synchronous solver
still requires a completion-time premise; no asynchronous watchdog is proved.
There is no partial-horizon, delayed-input, nonreturn-contract, or discrete
maneuver controller. The sole formulation applies to zero or more targets.

At verified early exit, or at the certified terminal time after execution of
all admitted intervals, the controller returns `command = []`, `inputs = []`,
and `problem.metadata.encounterComplete = true`. The calling system must handle
that completion explicitly. No control is certified beyond that endpoint; a
completed certificate must not be reused as a new encounter admission.

Version-13 certificates retain `qp`, `prediction`, and `decision` in their
**original admission coordinates**. `consumedSteps` locates the executable
suffix, `plan` and `predictedState` expose that suffix, and `deadline` stays
fixed. Likewise `problem.layout` describes the retained full decision while
`problem.inputPlan` exposes the remaining controls. This representation avoids
roundoff from repeatedly eliminating columns. It is not a shortened newly
linearized QP.

## 2. Exact-model predictive CBF

Let the augmented encounter state be

\[
z=(x,u_{\mathrm{previous}},q_T,t),\qquad z^+=F(z,u),
\]

where \(x\) is the six-state ego pose/velocity state and \(q_T\) identifies the
same absolute-time target trajectory at every update. With delay, a committed
queue would also be necessary; this implementation explicitly rejects nonzero
delay in this construction. Assume compact physical inputs \(U\), a continuous
well-posed flow, and continuous normalized margin maps on the full rollout
domain. Operating-domain restrictions belong among the margins, rather than
silently removing trajectories from the compact maximization argument.

For each held interval of duration \(\Delta\), define

\[
G(z,u)=\min_r\inf_{0\le\tau\le\Delta}
          \frac{g_r(x(\tau;z,u),q_T(t+\tau),u,u_{\mathrm{previous}})}{s_r},
\qquad s_r>0. \tag{1}
\]

The collision margin is the oriented-rectangle configuration distance minus
`collision.clearanceMargin`. The remaining functions include the relevant
road, model-domain, input and slew inequalities. Slew is evaluated between
successive held inputs, with the previous input included in \(z\). Constraints
that are independent of \(\tau\) enter (1) as constant interval margins.

Choose once, at admission, \(T=t_0+N_0\Delta\). The terminal function includes
safe endpoint constraints and, for every admitted target \(j\),

\[
e_j(z)=\|p_E-p_{T,j}\|-R-\varepsilon_{\rm exit},\qquad
\varepsilon_{\rm exit}>0. \tag{2}
\]

Let \(b_f\) be the minimum of the normalized terminal margins. Set
\(N_k=N_0-k\) and define

\[
H_N(z)=\max_{\mathbf u\in U^N}
 \min\{\bar\rho,G(z_0,u_0),\ldots,G(z_{N-1},u_{N-1}),b_f(z_N)\},
\quad H_0(z)=\min\{\bar\rho,b_f(z)\}. \tag{3}
\]

Under the stated continuity and compactness conditions the maximum is attained
and \(H_N\) is continuous, generally nonsmooth. Consequently,

\[
H_N(z)\ge0\quad\Longleftrightarrow\quad
\exists\mathbf u,\rho\in[0,\bar\rho]:
G(z_i,u_i)\ge\rho,\ b_f(z_N)\ge\rho. \tag{4}
\]

The signed analytical extension outside this feasible set is useful for
analysis. The online constraints always retain \(\rho\ge0\); they do not
execute negative-margin plans. Neither rectangle distance nor \(H_N\) is
claimed to be continuously differentiable.

**Theorem (finite-encounter safety).** Suppose the initial problem (4) admits
a complete witness, target predictions are consistent in absolute time, and
the actual successor equals the predicted successor. Applying the first input
of any feasible witness is safe for the entire held interval. Its remaining
inputs are feasible at \(z^+\) for horizon \(N-1\), with the same \(\rho\),
previous-input slew conditions, terminal state and absolute terminal time.
Induction proves recursive feasibility and hard safety through \(T\). The
terminal condition verifies perception exit for every admitted target.

**Proof.** The first stage satisfies \(G\ge\rho\ge0\). Delete that stage.
Every remaining inequality was already imposed, and its initial previous input
is exactly the input just applied. No interval is appended and no endpoint is
moved. This supplies a feasible successor candidate. Repeating the argument
proves the claim, including the last interval and \(N=0\) endpoint. Earlier
verified exit may terminate the encounter sooner. Global optimization is not
needed for this safety argument.

For an attained maximizer of (3), the same tail gives

\[
H_{N-1}(F(z,u_0^*))\ge H_N(z). \tag{5}
\]

Thus \(h_k(z)=-H_{N_k}(z)\) satisfies
\(h_{k+1}(F(z,u_0^*))-h_k(z)\le0\), with safe set
\(\mathcal C_k=\{z:H_{N_k}(z)\ge0\}\). This is the finite-time, countdown
analogue of the nonincrease discrete CBF convention. It ends in a completed
mode; an absorbing completed mode is mathematical bookkeeping and makes no
physical assertion about post-exit driving.

A one-second hard prediction alone cannot supply this result. With 12 m gap,
10 m/s closing speed and 2 m/s² maximum deceleration, the gap is
\(d(t)=12-10t+t^2\). It is positive at one second (3 m), while the stopping
distance is 25 m. After 0.5 s the gap and closing speed are 7.25 m and 9 m/s;
the next one-second maximum-braking prediction ends at -0.75 m. This exact
model counterexample is excluded by the complete terminal-admission condition.

## 3. Implemented robust inner certificate

The runtime retains the entire declared sampled affine inclusion from admission:

\[
\dot x=A_i x+B_i u_i+c_i+w_i,\qquad |w_i|\le\bar w_i. \tag{6}
\]

All stages use `finitePredict` held-interval Taylor/Bernstein enclosures.
There are no nominal-only future stages. Previously certified matrices, complete cell tubes, road charts, separating
normals and physical rows stay fixed during an encounter. New target admission
appends rows without replacing them. These rows conservatively imply the physical interval constraints
provided their geometry and enclosure premises hold. A nonlinear vehicle
falls under (6) only if its residual is uniformly bounded by the declared
\(\bar w_i\) on every certified interval and domain. A measured residual or
zero configuration value does not prove that condition.

Target motion remains the original Cartesian jerk/yaw-acceleration inclusion,
evaluated from each target's admission state at the appropriate elapsed time.
New observations are checked against it without renewing it or replacing its
nominal point. A curved exact target need not have zero jerk. For example,
constant speed \(v\) and curvature \(\kappa\) yield Cartesian jerk magnitude
\(v^3\kappa^2\). The regression test includes such a trajectory with nonzero
jerk bounds.

For a terminal target, choose a unit normal \(n_j\) from the terminal anchor.
The endpoint chart bounds the ego center by
\(p_E=o+M x_{N,1:2}+e\), \(|e|\le r_{\rm chart}\).
With target center/radius \((\hat p_{T,j},r_{T,j})\) and ego endpoint radius
\(r_E\), the added affine row enforces

\[
n_j^\top(o+M\hat x_{N,1:2}-\hat p_{T,j})
-|n_j|^\top(|M|r_{E,1:2}+r_{\rm chart}+r_{T,j})
\ge R+\varepsilon_{\rm exit}. \tag{7}
\]

Since \(\|p_E-p_T\|\ge n_j^\top(p_E-p_T)\), (7) verifies circular center
exit. The endpoint also belongs to the final interval's hard collision, road
and operating-domain rows. No nonreturn claim is needed for the encounter's
limited scope. A different sensing geometry requires a different terminal
certificate; (7) must not be reused as if it described another sensor.

The full condensed program has physical decision \(d=[\mathbf u;\delta]\).
Let \(A d\le b^{\rm phys}\) collect all hard rows. The immutable bound
\(b^0=b^{\rm phys}-\epsilon\) includes the mandatory construction reserves
(two existing numerical reserves plus applicable initial reserves). Every
physical hard row has a positive scale of one in its native units; slack
nonnegativity has scale zero. The LP is

\[
\max_{d,\rho}\rho:\quad A d+s\rho\le b^0,\quad
0\le\rho\le\bar\rho,\quad d_{\rm executed}=d_{\rm actual}. \tag{8}
\]

This normalizes metres, radians, velocities, and dimensionless input
ratios by one in the corresponding row's units. Scaling changes the max–min
allocation, never the hard zero-level limits. Collision rows have zero
coefficients in all CLF-slack columns. Unbounded CLF slacks can be reconstructed
for any hard-feasible control, so they impose no extra feasibility requirement
on the margin LP.

The checker computes the supported witness margin from the same original rows:

\[
m(d)=\min\left\{\bar\rho,\min_{r:s_r>0}
\frac{b_r^0-A_r d-\eta_r(d)}{s_r}\right\}, \tag{9}
\]

where \(\eta\) is the explicit floating-point dot-product evaluation allowance.
It also checks hard zero-scale rows, the fixed executed prefix and CLF cones.
A reported negative margin is rejected. The subsequent tracking/CLF SOCP uses
at least the carried margin; its interior target is
\(\max\{m_{\rm carried},0.99m_{\rm LP}\}\). The checked LP candidate remains
available if this subordinate solve fails. The accepted achieved margin,
not the solver's raw \(\rho\), is stored.

## 4. Why the stored witness survives

With the target set unchanged, let \(d_k\) be the checked full
admission-coordinate decision after \(k\) executed intervals. The admissible successor family keeps the same matrices,
base bounds, target envelopes and deadline, and fixes one additional input
pair to the input just issued. Therefore \(d_k\) itself remains a member of
that family. Its remaining intervals still end at \(T\).

This proves the required tail inclusion without asserting that an old plan
satisfies rebuilt geometry. Past stage rows are also retained. They can limit
future margin improvement, but cannot remove the incumbent. Equality of the
original dot products on reuse preserves (9) without recharging a numerical
reserve, shrinking matrices, or treating an acceptance tolerance as clearance.
The stored witness is rechecked before use. Optional replacement is accepted
only when its verified margin is at least \(m_k\), so

\[
m_{k+1}\ge m_k\ge0,\qquad \widehat h_{k+1}-\widehat h_k\le0,
\quad\widehat h_k=-m_k. \tag{10}
\]

Here \(\widehat h\) is a certificate on the **augmented controller state**
containing the stored witness, clock, input history and valid information set.
It is not asserted to be the state-only globally optimized nonlinear value
(3). The LP solves a fixed conservative family; its checked plan provides a
lower bound on the corresponding complete-continuation margin. The tracking
objective and CLF slacks are not predictive barrier values.

For uncertain execution, assume the initial ego/target sets contain the truth
and the same differential inclusions remain valid. Conditioning observations
cannot invalidate the still-possible true trajectory in the retained root
family. The runtime checks nonempty intersections; it does not rebox a shifted
tube and propagate that larger box through new dynamics. The root certificate
already covers every admissible disturbance and future input in its family.
This supplies the robust counterpart of the exact successor assumption in
Section 2. Correct measurement error bounds, geometric enclosures and storage
integrity remain premises, rather than being proved by a nonempty intersection.

Safety is conditional on admission before the first issued held interval,
execution of the certified inputs on schedule, valid full-horizon ego and
target enclosures, and successful joint admission of newly detected targets before issuing the
next input. Other changes to hard constraints remain unsupported.
The acquisition/first-solve interval preceding that admission is not covered
retroactively. Hardware execution, globally valid nonlinear residual bounds,
and worst-case computation time are not validated by this implementation.
Metadata therefore reports the conditional recursive-feasibility scope and
keeps `physicalVehicleGuaranteeEstablished = false` and
`exactPredictionAssumptionsHold = false`.

## 5. Joint admission at first detection

The September 9, 2026 research assumption is that, when a new target is first
observed, the augmented joint state belongs to the controller's certifiable
feasible domain for all retained and new targets. Cases outside that domain
are outside the research guarantee. This is an explicit environmental/admission
assumption, not a claim that every traffic encounter is avoidable, nor an
assumption that every later optimization succeeds.

For this implementation, the domain is defined by the retained affine
inclusion, hard rows, executed input prefix, remaining intervals and original
absolute exit deadline, together with the new targets' complete remaining
collision and terminal-exit rows. It is a conservative inner domain, not the
nonlinear vehicle's maximal viable set. The assumption does not authorize
moving an old deadline, dropping a missing old target, changing old motion
bounds, or classifying a solver failure as physical infeasibility.

Let the old physical rows be `A_old d <= b_old`. At a detection event the
accepted decision must satisfy both those rows and `A_new d <= b_new`, with
all already executed inputs fixed exactly. The new rows enclose every remaining
held interval from detection to the same terminal time. They impose no
retroactive constraint before detection. No CLF slack appears in these safety
rows. The safety-margin floor at this information change is zero; the previous
positive margin need not survive the addition of a new obligation.

`hardEncounterBarrier.advance` first verifies execution and the observations of
known targets, then `localAdmitNewTargets` appends the new constraints. Old
matrices, prediction tubes and geometry remain unchanged. The old input plan
is also independently checked against the **joint** problem: it is an eligible
witness only if every new row passes. If neither it nor a new LP/SOCP solution
is verified, the controller raises `jointAdmissionNotCertified` and issues no
control. This diagnostic alone does not establish that the mathematical joint
problem is infeasible. In particular, the feasibility assumption is distinct
from the practical premise that a checked witness is available before actuation.

After a successful event, all joint rows are retained and the same fixed-prefix
argument in Section 4 applies. Each target has its own integer detection-step
origin in `targetAdmissionSteps`; later observations cannot renew that origin.
The version-14 certificate rejects earlier stored-certificate formats.
Version 14 removes the additional axle-force polygon; the remaining hard
rows and the successor-witness argument are unchanged. Nonlinear Fiala
saturation is intrinsic to its exact force law, while the affine inclusion
still requires valid residual bounds.

Thus induction covers any sequence of successful joint admissions **within the
original complete encounter**. Between information changes, the verified margin
is preserved or increased. At a new-target event, only nonnegativity is required;
there is no claim of a common positive-margin monotonicity across different
constraint sets. Metadata reports `jointAdmissionPerformed`,
`newlyAdmittedTargetKeys`, and the explicit `newTargetAdmissionAssumption`.

This premise does not establish terminal invariance, post-exit driving, a safe
transition after an exhausted deadline, or protection before a target's first
detection. The requested single-path continuing controller still needs the
terminal construction specified in [the design requirements](SINGLE_PATH_RECURSIVE_FEASIBILITY.md).
The weaker lookahead path and its configuration selectors have been removed.

## 6. Terminal continuation required for a single controller

A vehicle following another indefinitely may be safe without reaching (7).
This finite-encounter controller rejects that admission. A fixed-horizon alternative needs a joint
ego–target–actuator terminal set and controller satisfying

\[
z\in\mathcal C_f\Rightarrow G(z,\kappa_f(z))\ge0,\quad
F(z,\kappa_f(z))\in\mathcal C_f. \tag{11}
\]

For a nondecreasing positive max–min value, sufficient additional conditions
are \(G(z,\kappa_f(z))\ge b_f(z)\) and
\(b_f(F(z,\kappa_f(z)))\ge b_f(z)\) when \(b_f(z)\ge0\).
Set invariance alone proves safety, not preservation of all positive margin
levels. Ego rest alone is insufficient against moving targets; the retired
terminal-set counterexamples in `TERMINAL_CBF_PROOF.md` still apply.

## 7. Sources and validation

Huang, Wang, Margellos and Goulart, *Predictive Control Barrier Functions:
Bridging model predictive control and control barrier functions*, ECC 2025,
pp. 2623–2629, DOI
[10.23919/ECC65951.2025.11187291](https://doi.org/10.23919/ECC65951.2025.11187291),
provide the nonincrease CBF definition and shifted feasible-candidate proof
structure (Definition II.1, Eq. (8), Lemma III.1 and Eqs. (10)–(11)); see the
[author preprint](https://arxiv.org/html/2502.08400v1). Their slack-sum recovery
value is not the hard max–min value constructed here.

Choi, Lee, Sreenath, Tomlin and Herbert, *Robust Control Barrier-Value Functions
for Safety-Critical Control*, CDC 2021,
[arXiv:2104.02808](https://arxiv.org/abs/2104.02808), connect reachability values
and barrier certificates. That connection motivates a safety value; the
retained-deadline theorem and immutable-witness implementation above are a
project derivation, not a theorem attributed to that paper.

`tests/hardEncounterBarrierTest.m` exercises admission, impossible exit,
initial overlap, full deadline countdown, early completion, nondecreasing
margins, finite slew, declared uncertainty, nonzero target jerk, solver-free
continuation, failed/unsafe reoptimization and CLF-solve failure. It also
rejects changed sensor ranges, input mismatches, uncertified joint admissions, inconsistent
observations, inflated stored margins, partial certification and unsupported
delay. Dense exact affine-flow clearance checks use 41 times per held interval
as a numerical audit; they do not replace the complete-interval certificate.
The deterministic fixture uses four 100 ms intervals, ego speed 8 m/s,
target center [-8;0] m moving at [-8;0] m/s, 13.5 m sensing radius and 0.1 m
exit buffer. No random seed, recorded data, or vehicle trial is used.

Validation on September 8, 2026: a fresh project-root `runtests('tests')` run
passed all 595 tests with no failures or incomplete tests. The targeted
regression passed 124 tests; the final focused barrier suite passed 22 tests.
Factory Code Analyzer checks reported zero findings in the seven changed
MATLAB files. The core controller contains 20 source files, meeting its limit.

September 9, 2026 joint-admission regression: the focused suite passes 30 tests,
including positive joint admission, simultaneous retention of old targets,
independent checking of the incumbent after a solver failure, rejection of an
unsafe new target, distinct forecast origins, repeated admissions through the
original endpoint, and rejection of deadline renewal. These are declared-model
unit tests, not new vehicle trials or a proof of continuous driving.

The focused controller/configuration, finite-sensing, target prediction and
swept-flow regression batch passed 123/123 tests with no incomplete tests.
After the final fixture preallocation edit, the hard encounter suite passed
30/30 again. Factory Code Analyzer settings reported zero findings for
`hardEncounterBarrier.m` and `hardEncounterBarrierTest.m`. The separate
repository-wide `runtests('tests')` tool request timed out after 300 seconds;
the task-owned MATLAB execution received an interrupt, and no complete-suite
result is claimed.
