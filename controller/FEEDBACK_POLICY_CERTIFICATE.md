# Finite-encounter sampled-feedback certificate

Research derivation, September 10, 2026. The physical-model hypothesis below is
an explicit project assumption: **the authoritative modified Fiala vehicle
model is the true model**. This document proves a conditional controller
construction; it does not label the current affine, open-loop implementation
as an implemented nonlinear feedback certifier. The implementation and
experiment boundary is recorded in `FEEDBACK_SPARSE_EXPERIMENTS.md`.

## 1. Specification and first admission

Let actuation times be t_i=t_0+i h. For each admitted target j, require
continuous-time noncollision until its first certified exit from perception,
and a finite upper bound D_j on that exit time. No safety or invariance claim
is made after discharge. A later reentry is a new admission. The initial
optimization must contain a **complete robust executable policy**, rather
than just a nominal collision-free path or a short prediction prefix.

A deadline is selected jointly with the first feasible complete witness:
D_j=t_0+N_j h for some finite integer N_j. There is no imposed 1.6 s exit
requirement. A search window is an initialization of the search, not the
allowed physical exit time. Once admitted, replacements cannot postpone D_j.
Otherwise the planner could repeatedly predict exit a fixed duration ahead
without ever executing it. An equivalent finite-time progress certificate
could replace this deadline mechanism, but mere safety alone cannot prove exit.

The admission assumption is that a soundly checked complete witness is
available before the first command whose safety depends on that target is
applied. Mere mathematical existence does not imply that a local optimizer
finds it within the available time. For a real-time existence-to-execution
claim, an initialization algorithm must have a verified execution bound on
its stated admission domain. Detection and computation delays, if present,
are part of the committed prefix, not an unmodeled grace period.

The information state I_i contains a set of possible ego/target states,
target motion parameters, measurement histories relevant to future control,
road knowledge, previous applied input, and any actuator or delay memory.
All true quantities belong to these sets. A witness contains nominal states,
causal feedback laws, inlet sets, swept enclosures, local model-domain
certificates, road obligations and the unexpired exit deadlines. Planning
centers are variables and may remain at the old witness centers.

## 2. Exact Fiala model and local inclusion

Use the persistent Cartesian state x=(p_x,p_y,psi,v_x,v_y,r) and input
u=(delta,beta). `ltvBicycleModel.fialaWorldDynamics` calls the authoritative
nonlinear flow with zero curvature and zero external acceleration bias.
It includes body-to-world kinematics, front-force rotation, road load,
longitudinal/lateral coupling, actual atan2 slip and modified Fiala forces.
Under the project hypothesis there is no additional plant discrepancy.
There remains a nonlinear approximation error whenever an affine predictor
is used to certify this nonlinear model.

Write C for an axle stiffness, t=tan(alpha), and
Q=mu F_z sqrt(1-beta^2). In its unsaturated branch the tire force is

    F_y = -C t + C^2 t |t|/(3Q) - C^3 t^3/(27Q^2).

The saturated branch is -Q sign(alpha). A convenient compact certification
domain has v_x >= v_min > the numerical speed floor, |beta| <= b < 1,
and |alpha| <= alpha_max < pi/2, with finite remaining state/input bounds.
These restrictions must be checked on actual atan2 slip and actual feedback
inputs. Scheduled-speed slip inequalities do not establish this obligation.
The domain avoids the singular joint tangent at |beta|=1. Across zero slip
and saturation, use a piecewise bound covering every intersected branch;
a globally twice continuously differentiable force must not be assumed.

For each stage declare a domain D_i first. Choose constant A_i,B_i,c_i and
prove, throughout D_i,

    f_Fiala(x,u) - A_i x - B_i u - c_i in R_i.                  (1)

For smooth pieces, outward-rounded interval Jacobians or Taylor remainders
provide (1). A componentwise second-order remainder can use
(1/2) sum_ab sup_D |H_l,ab| d_a d_b about its expansion point, including
any affine mismatch. Branch unions, integration and rounding must also be
bounded. Sampling maxima, fitted residual constants and ordinary floating
point matrix exponentials alone are not verified domain-wide bounds.

Use the affine nominal flow z_dot=A_i z+B_i v_i+c_i below, with each next
nominal center equal to the preceding nominal endpoint. Then its error
has exactly the residual in (1). If instead z is integrated with the exact
nonlinear model, the residual must subtract its nominal flow as well;
it is a different residual expression and cannot be omitted or counted twice.

**Domain lemma.** Suppose a validated flow construction using (1) encloses
the state/input tube strictly inside D_i, from an admitted inlet. Then the
true trajectory remains in that enclosure over the interval.

Proof. The exact model is locally Lipschitz on the domain. Before a first
exit from D_i, (1) and the validated integral propagation apply. Continuity
puts the hypothetical first-exit state in the same enclosure, whose strict
interior containment excludes a boundary exit. Thus no such exit occurs.
Boundary-touching domains require a direct invariance/validated reachability
argument rather than the strict-interior version of this lemma. This avoids
assuming the desired tube containment in order to establish its own residual.

## 3. Correct sampled-data feedback propagation

For the zero-delay case with measurement error nu_i in V_i, execute

    u(t)=v_i+K_i (x_i+nu_i-z_i),  t in [t_i,t_{i+1}).            (2)

The command is held; the feedback is not recomputed continuously. Put
Phi_i(tau)=exp(A_i tau), Gamma_i(tau)=integral_0^tau exp(A_i s) B_i ds,
and M_i(tau)=Phi_i(tau)+Gamma_i(tau) K_i. Variation of constants gives

    e(t_i+tau)=M_i(tau)e_i+Gamma_i(tau)K_i nu_i
       + integral_0^tau exp(A_i(tau-s)) w_i(s) ds.              (3)

Here w_i(s) is the time-varying nonlinear residual in (1). Formula (3)
is exact before enclosure. In general M_i(tau) differs from
exp((A_i+B_i K_i)tau). For x_dot=u, K=-1 and h=1, these transitions are
respectively zero and exp(-1). `stateUncertainty.sampledFeedbackTransition` implements (3)
in floating point for design and testing, not validated interval arithmetic.

For any support direction a, a sound bound on the last integral is

    sigma_W(a) <= integral_0^tau sigma_R(exp(A_i^T(tau-s))a) ds. (4)

The right-hand side equals the support of the Aumann integral for a fixed
compact convex disturbance set and independent measurable selections; an
outer bound is enough. A constant-disturbance formula is generally invalid.
The tube must cover every tau, with validated numerical remainders.

For E_i=G_i[-1,1]^g (+) {e:e^T P_i e<=r_i^2}, P_i positive definite,

    sigma_E(a)=||G_i^T a||_1+r_i sqrt(a^T P_i^(-1)a).          (5)

Retain signed generators in actual support evaluations. Certified reduction
may replace discarded generators with an outer remainder; repeated full
axis-aligned reboxing sacrifices correlation. Measurement correlation with
ego/target/localization may be retained jointly; Cartesian products are
safe only as explicit conservative outer approximations.

The endpoint obligation is

    E_{i+1} contains M_i(h)E_i (+) Gamma_i(h)K_i V_i (+) W_i.   (6)

Input limits and model domains use the actual joint uncertainty
[e(t_i+tau); K_i(e_i+nu_i)], not independently optimized nominal inputs.
Slew limits require correlation with the preceding command. Actuator dynamics
and nonzero delays require augmented-state versions of (2)--(6); commands
cannot be clipped after verification unless clipping itself is certified.

### Useful bounded tubes, without a changing-metric loophole

A sufficient linear condition is

    M_i(h)^T P_{i+1} M_i(h) <= rho^2 P_i,  0<rho<1,
    0<p_lower I <= P_i <= p_upper I<infinity.                 (7)

If the complete measurement, residual, reduction and arithmetic remainder
has next-metric norm at most w_bar, Minkowski's inequality yields

    s_{i+1} <= rho s_i+w_bar,
    ||e_i||_2 <= [rho^i s_0+(1-rho^i)w_bar/(1-rho)]/sqrt(p_lower). (8)

Both metric bounds must be reported for uniform physical comparisons. A
fixed P is simpler. Without uniform coercivity, M=1 and P_i=rho^(2i)
satisfy the matrix inequality although physical error never contracts.
For one finite horizon, explicit min/max eigenvalues suffice for its bound;
they do not establish a horizon-independent disturbance budget.

A verified nonlinear held-flow Jacobian bound can alternatively establish
contraction directly. A Schur linear sampled map and locally Lipschitz
Jacobian imply a sufficiently small neighborhood with contraction, because
a fixed-P strict Lyapunov inequality persists under a small Jacobian
perturbation. For a moving reference, this must hold uniformly over the
whole reference family. A pointwise LQR gain alone is not that certificate.
Contraction is sufficient for a useful small tube; **recursive feasibility
of a finite witness needs valid propagation, not a universal rho<1**.

Ego feedback cannot contract an independently moving target's uncertainty.
At an update, retain the old valid target future envelope and intersect
with correctly propagated new observations. Do not promise future observer
shrinkage that the incumbent policy has not accounted for.

## 4. Continuous-time geometry and road meaning

For each cell and target, form relative body points at the same time:

    d_ab(t)=p_e(t)+R(psi_e(t))a-p_j(t)-R(psi_j(t))b.

Construct verified Bernstein coefficient sets D_l^ab whose convex hull
contains these relative points throughout the cell. Asynchronous differences
of independent complete sweeps can introduce artificial collisions.
Choose one continuously variable normal n per cell/target and require

    ||n||_2<=1,  inf_(d in D_l^ab) n^T d >= d_safe>0            (9)

for all coefficient sets and rectangle vertex pairs. All relative rectangle
points are convex combinations of vertex differences. Bernstein weights
are nonnegative and sum to one, so (9) implies n^T d_ab(t)>=d_safe for
all times and all body points. Cauchy--Schwarz then proves Euclidean
clearance >=d_safe. No integer maneuver label or unit-norm equality is needed.

For fixed coefficient sets, maximum-margin normal selection is a convex
support problem. With normals fixed, a convex trajectory inner approximation
can propose a new witness. Every accepted candidate still needs a full
nonlinear-domain and swept check. Alternation is not a complete global
feasibility solver; a collision-free initializer can be necessary. Strict
separation and continuity on a compact interval permit sufficiently short
constant-normal cells, but do not guarantee that a local optimizer finds them.

Store safety geometry in a persistent Cartesian frame. Represent possible
physical roads by Rset_i, condition by intersection with new information,
and take the guaranteed drivable region intersection_(R in Rset_i) R.
Consistent conditioning enlarges this guaranteed region. Intersecting
independently fitted inner drivable polygons is a different operation.
Old road patches remain usable only under their declared validity times and
coverage. For every remaining swept footprint, verify valid road containment;
unknown future road is not certified by anticipated future sensing.
A coordinate change must transport centers, generators, gains and uncertainty
consistently, or transform new observations into the admission frame.
Raw structure equality is neither necessary nor a geometric proof.

## 5. Suffix closure and the finite-encounter theorem

Assume (i) the exact Fiala model and execution/sensing contracts hold;
(ii) target/road information is valid and updates condition rather than
invalidate the incumbent possibilities; (iii) admission installs a soundly
verified complete witness with finite deadlines; and (iv) a bounded-time
executor performs membership/contract checks and the certified feedback,
independently of improvement-optimizer completion. These are separately
checkable modeling, information, initialization and execution hypotheses,
not an assumption that every future optimization will find a solution.

Each segment is certified for **every** augmented state in its inlet F_i,
and its reachable endpoint information belongs to F_{i+1}. The inlet includes
admissible target state/parameter/history information and actuator memory.
The witness covers all relevant measurement realizations, so policy suffixes
are conditioned on the actual history rather than replacing it by an
unjustified fresh uncertainty model.

**Suffix lemma.** Executing the first segment of a valid witness makes its
retained suffix a feasible witness at the next actuation time.

Proof. Equations (1)--(6) and the domain lemma contain the true continuous
trajectory in the certified swept tube. Endpoint inclusion puts the actual
augmented information into F_{i+1}. Consistent measurements restrict that
information; they cannot remove the truth or require enlarging the old valid
envelope. Compatible road updates retain all unexpired swept obligations.
Every old subsequent policy and proof therefore applies to the new information.
The suffix has the same terminal deadlines. It is an explicit feasible point
in the next certificate problem, provided its old centers and inlet sets
remain admissible planning choices. No new solver success is used. QED.

Define g_l(W) as verified lower physical margins, with fixed positive unit
scales sigma_l and fixed cap m_max. Let

    m(W)=min{m_max,min_l g_l(W)/sigma_l}.

Removing an executed prefix removes obligations; it does not reduce this
minimum. Retain their certified lower bounds when reusing a proof. Accept
an improvement only if it is complete, preserves deadlines and has margin
at least that of the retained suffix. Recomputing a looser lower bound must
not overwrite a valid inherited bound. With new targets, joint feasibility
and old target obligations must be established separately; a positive old
margin is not guaranteed attainable after adding new constraints.

The augmented-state scalar h(I,W,t)=m(W) is nonnegative and nondecreasing
under such continuation/replacement. It is a maintained predictive-barrier
certificate, not a claim of a smooth state-only CBF or of computing the global
supremum over all policies. Its constituent hard inequalities remain multiple.

**Theorem (requested finite-encounter property).** Under the hypotheses above,
if the first joint robust certificate problem supplies an executable feasible
witness, then at every actuation until discharge the remaining certificate
problem is feasible, the executed continuous-time motion is collision-free,
and every admitted target exits perception by its committed finite deadline.

Proof. The first witness establishes the induction base. At every step the
suffix lemma supplies a feasible continuation; either this witness or a
verified replacement is executed. Equation (9) and road/input obligations
prove safety throughout that sample interval, including boundaries and
command transitions. Finite induction reaches each terminal time D_j.
At that time the terminal enclosure satisfies, for a fixed ||n_exit||<=1,

    inf n_exit^T(p_e(D_j)-p_j(D_j)) >= R_exit+epsilon_exit,      (10)

where R_exit bounds the relevant perception range and epsilon_exit>0.
Thus the target is outside perception by D_j. Earlier exit can be discharged
only by a sound geometric observation/set proof, not by exhausting a counter.
The remaining-step counter decreases because absolute deadlines do not move.
Consequently exit occurs in finite time without any infinite-horizon
invariance assumption. QED.

If perception uses a field of view instead of a radius, replace (10) by
certified set exclusion from that field of view; the proof is unchanged.
The footprint-separation requirement applies throughout the encounter;
center-distance exit is used only when it matches the sensing definition.

A newly entering target requires a new joint witness, including the already
committed input and old deadlines. The old witness alone certifies nothing
about an unadmitted target. The user's admission-domain hypothesis covers
this obligation. An unlimited stream of admitted targets may keep perception
nonempty forever while every individual target still exits by its own finite
deadline. Simultaneous permanent emptiness is not the requested theorem.

## 6. Short-prefix splicing is an inclusion problem

Let an incumbent tail begin at absolute time t_{i+L} with inlet F_old.
Certify a candidate L-step feedback prefix, and prove

    I_candidate(t_{i+L}) subset F_old.                        (11)

The set includes actuator queues/previous input, measurement-dependent policy
memory, target envelopes and correlations used by the tail. Require consistent
road obligations and unchanged old deadlines. Then concatenate the prefix
with the old tail. Prefix safety follows from its proofs, (11) makes the tail
executable for every possible prefix endpoint, and the suffix/exit theorems
apply verbatim. This proves splice sufficiency without rescanning unchanged
proofs. A change of inlet coordinates or feedback metric must be transported.

Matching only nominal position/velocity is insufficient: a larger error set,
different previous steering input or different target envelope can invalidate
the first tail segment. A convenient sufficient condition is identical tail
center and memory, E_candidate subset E_old, and target-information inclusion.
For ellipsoids with equal center, E(P_new,r_new) subset E(P_old,r_old) follows
from r_new^2 lambda_max(P_new^(-1/2)P_old P_new^(-1/2))<=r_old^2.
For shifted centers, a support-function inequality for every direction is
sufficient and necessary for convex compact sets; a bounding norm condition
can be a conservative finite certificate. Joint uncertainty must not be split
inconsistently when checking this condition.

## 7. Performance formulation and computational consequences

The safety theorem never uses soft CLF decay. If nonnegative CLF slacks have
no finite upper bound, every fixed admissible physical input can acquire
large enough slacks to satisfy those rows. Removing them from the optimization
and choosing a quadratic tracking/input objective therefore does not remove
hard physical safety obligations. It does change the performance optimum
and does not preserve a CLF tracking-stability theorem automatically.

After admission, enforcing the inherited margin in a single performance
solve suffices; maximizing that margin again is optional. For a prefix
with L stages and M local rows, stage-local variables give a block-banded
constraint structure rather than dense dependence on all future inputs.
With fixed state dimension, structured QP work can grow approximately
linearly with L. This does not imply a bound for nonlinear geometry search,
validated interval propagation, initial admission or total execution time.
The executor must be scheduled independently and proof reuse must have a
sound information-membership and splice test. The online controller currently
still solves the two-stage performance problem; the one-solve QP and four-step
splice are explicitly offline ablations, not additional controller modes.

## 8. Nonempty local domains and what remains to instantiate

Suppose a true nonlinear nominal maneuver has strictly positive clearance,
road, input, slew, slip-domain and terminal-exit margins through D. Suppose
its actual held feedback has a uniformly valid contraction/radius bound
s_i<=max{s_0,w_bar/(1-rho)} in a coercive metric. Bound each physical margin
loss by L_l s+epsilon_l, including target/road and numerical uncertainties.
Then strict inequalities

    L_l max{s_0,w_bar/(1-rho)}+epsilon_l < g_l(nominal)

and strict domain/input reserves give a feasible tube. By continuity, a
neighborhood of initial conditions and sufficiently small nonzero disturbances
remains feasible. This is a sufficient nonemptiness result, not a claim that
a nominal path or a stabilizable trim exists for every encounter. Finite
noncontractive bounds can be substituted when adequate. No numerical radius
for the diagnosed oncoming maneuver is claimed without validated derivatives,
nonlinear swept enclosures and a feasible nonlinear initializer.

The conditional implication in Section 5 is proved. A first local nonlinear
model-inclusion checker and held-cell domain test are now implemented in
[fialaCertificate.residual](FIALA_INTERVAL_INCLUSION.md). A subsequent
[complete sampled-feedback verifier](FIALA_FEEDBACK_SAMPLE.md) now preserves
shared state/input generators through a full 100 ms held sample, explicitly
bounds affine error, and checks the command and intersample slew limits.
The subsequent [tightening and continuation implementation](FIALA_FEEDBACK_TIGHTENING.md)
now retains correlated ego/previous-command inlets across actuation boundaries,
checks true command changes and certifies prescribed 5 s ego policies in the
reported local domains. Its final measured sample calls fit within 100 ms
on this host, without a WCET claim. Complete encounter collision/road/exit
verification, joint target-information inlets, persistent road semantics and
a scheduled executor remain to be implemented. Larger-radius feasibility
still depends on enclosure quality and the specified feedback law.
The sparse repair and continuous-normal prototype do not
supply these ingredients automatically. An affine trajectory colliding in
exact Fiala numerical replay is a falsification of that particular model
transfer, even when every original affine certificate row passes.

## References and relationship to prior results

- J. Koehler, R. Soloperto, M. A. Mueller and F. Allgoewer,
  [A computationally efficient robust model predictive control framework for
  uncertain nonlinear systems](https://arxiv.org/html/1910.12081v2), Sections
  II--III and Appendix A: incremental stability and robust tube tightening.
  The held-input, finite-exit suffix construction above has its own proof;
  continuous-feedback formulas must not be copied into a sampled executor.
- K. P. Wabersich and M. N. Zeilinger,
  [Predictive control barrier functions: Enhanced safety mechanisms for
  learning-based control](https://arxiv.org/abs/2105.10241): predictive barrier
  certificates. Its theorem is not automatically a finite perception-exit
  theorem for this augmented information state.
- [Biconvex Optimization for Smooth Minimum-Time Trajectories around Convex
  Obstacles](https://arxiv.org/html/2608.02834v1): separation/trajectory
  alternation motivates proposal generation; it does not provide first
  admission completeness or the nonlinear Fiala tube proof.
- G. Frison and M. Diehl,
  [HPIPM: a high-performance quadratic programming framework for model
  predictive control](https://arxiv.org/abs/2003.02547): structured optimal
  control QP computation. Timing below is measured with this project's native
  Clarabel bridge, not HPIPM, and is not a worst-case execution guarantee.
