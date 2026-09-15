# Hard-constrained backup CBF and cruise CLF

September 15, 2026. Certificate format 24 replaces format 23's prescribed
rollout checker with a **two-variable hard-constrained optimizer**. No
independent runtime plan verifier authorizes execution. The full-horizon
predictive policy also uses one hard-constrained solve, with no safety-value
LP, omitted-row generation, slack repair or post-solve checker.

`controller.executionPolicy="backup"` selects the small optimizer;
`"predictive"` selects the full-horizon SOCP. `"auto"` selects backup for a
finite frame budget and continues a format-23/24 stored backup. Moving-offset
references retain predictive mode under auto. Explicit backup requires a
positive constant-speed trim. The simulation's 100 ms budget selects backup.

## A small optimization with a complete continuation

A saturated, slew-limited trim-feedback rollout proposes nominal inputs
`ubar_i`. This proposal does not authorize execution. For the two-dimensional
adjustment `w`, define

    u_0(w) = ubar_0 + w,
    u_i(w) = ubar_i - K * dx_i,       i > 0,
    dx_0 = 0,
    dx_(i+1) = Phi*dx_i + Gamma*(u_i(w)-ubar_i).

Here K acts on the five tracking-error coordinates. Every input, endpoint
and swept polynomial coefficient is affine in w. This is an open-loop
parameterization of the whole sequence, not feedback from hypothetical
future observations. After solving, the entire sequence is evaluated once
and stored. Its uncertainty uses `rho_(i+1)=abs(Phi)*rho_i`; no future
measurement shrinkage is assumed.

For fixed charts, target predictions and separating normals, the program is

    minimize  0.5 * w' * w
    subject to all swept road, collision, chart, model-domain and slip rows,
               all input and slew rows, including first terminal input,
               finite robust exit and road terminal-set rows,
               the sampled cruise CLF cone when cruise is requested.

All physical safety constraints are hard. Every original Bernstein control
point is represented, including through the conservative scalar-bound
reduction described below. A failed proposal may be followed by another
bounded proposal, but no physical violation variable is optimized or used
for execution. This two-variable family is a conservative inner
approximation of the full-horizon search.

Encounter admission tries at most two passing rollouts, each at most 64
holds. Active encounters normally execute their stored feasible suffix.
After confirmed release, the controller first tries a cruise hold and any
necessary braking transition. With a stored witness, this attempt receives
at most 15 ms and 20% of the remaining frame for formulation and solving.
Near exhaustion of the stored plan, one safety-only return-to-path rollout
can use the remaining budget. Initial road-only admission can instead try
a finite rollout with the first-hold CLF cone, then safety-only recovery.
There are at most two solves in an inactive continuation frame and three
in an inactive admission frame. At most 16 transition holds fit within the 64-hold total.
Proposed slew changes use 99% of the configured limit to leave numerical
room; the hard constraint retains the configured limit.

## Compact hard rows without a returned-plan check

Most straight-road continuation rows depend on one control coordinate.
Numerical cruise synthesis also produces small cross-coefficients that are
physically retained. The program imposes hard decision bounds `abs(w)<=r`.
For a row `a_j*w_j+a_l*w_l<=b` with
`abs(a_l)<=1e-10*abs(a_j)`, a sufficient scalar constraint is

    sign(a_j)*w_j <= (b-abs(a_l)*r_l)/abs(a_j).

The cross-effect is charged as an uncertainty support; it is not silently
rounded away. Arithmetic reserves move the resulting bound inward. Taking
the minimum bound for each of the four signed coordinate directions then
represents every contributing inequality. Rows with material coupling and
all SOC constraints are retained. Nonfinite division results are kept in
the original row representation. Native assembly also removes identically
satisfied zero rows and fixed-zero compatibility columns.

This is formulation before solving. It neither checks a returned plan nor
iterates over omitted constraints after a solve. In a captured recovery
problem, native rows fell from 30,197 to 1,356 and solve time from 71.535 to
1.794 ms. This isolated comparison excludes formulation and startup; the
full campaign reports complete-frame timing separately.

## Execution contract and numerical meaning

Only the solver's strict `Solved` status (`exitFlag=1`) and a structurally
valid finite decision authorize a new plan. Approximate, timed-out,
iteration-limited and failed solves cannot replace the stored plan. Solver
hooks must satisfy the same feasibility/status contract as the native
solver. A hook that falsely reports a feasible solution violates that
contract; there is deliberately no independent checker to detect it.

`solveHardCbfClf.certify`, `certifyInputs`, and
`hardEncounterBarrier.verifyFixed`, `verifyCandidate`, `terminalMembership`
remain available for **offline research audits only**. The online paths
never call them. Legacy `rowGeneration`, `witnessVerification` and
`lexicographicTieTolerance` configuration fields are accepted but ignored.

Affine bounds are tightened before solving using arithmetic and solver
feasibility reserves. Native objective normalization preserves minimizers;
its optimality tolerance applies to the normalized objective without a
second scaling. Primal feasibility tolerance is separate and unchanged. The full-horizon
soft-CLF penalty is reduced from 100 to 0.01 after passing experiments
exposed numerical stagnation; this does not change the backup hard cone.
A floating-point solver status is not an interval proof of numerical
feasibility. The mathematical safety claim assumes returned constraint
errors are covered by the construction reserves. No independent numerical
certificate or hardware fault guarantee is claimed.

`candidateAccepted`, `safetyCertified` and `planCertified` describe this
solver-contract-based admission. `acceptance.basis` identifies it explicitly.
Legacy `candidateVerified` and `shiftedSafetyCandidateChecked` metadata mean
that a feasible carried continuation was transferred; they do not indicate
a separate plan audit. Uncomputed physical residuals and margins are NaN. `pcbfValue=0` represents
the hard feasibility level, not a measured post-solve residual.
`postSolveCertificationPerformed=false` is published in both policies.

## Continuous safety and the predictive barrier

The declared ego plant is the recorded held affine generator, with zero
process residual. Target uncertainty follows bounded Cartesian jerk and yaw
acceleration while active. The held-flow template retains six initial-state
columns and two input columns, with the full declared domain in its
arithmetic reserve. Substituting the affine input/state parameterization
leaves two decision columns. Direct tire-slip input rows are transformed
with the same stage input map. Endpoint boxes retain individual propagated
radii; the cached swept template uses a common upper starting radius, which
can be conservative for uncertain encounters.

Taylor remainders, complete yaw intervals, rectangle supports, chart errors
and propagated boxes enter every swept inequality. Nonnegative Bernstein
basis weights then imply separation and road containment throughout every
hold. Finite exit rows require the whole target footprint outside the
confirmation region. Current sound observation, not a timer, releases the
target. The road-only terminal set and terminal input transition remain
hard. No all-future target support is required.

For the augmented information/witness state, let S(W) denote the sum of
nonnegative physical safety violations of a complete continuation whose
terminal and release obligations are satisfied. Feasibility gives S(W)=0
mathematically. After the first input is executed, conditioned boxes are
subsets of the published successor boxes. Monotonicity of reachability
preserves all remaining rows using the same generators and inputs. Thus
S(W_shift)=0; for h_B=-S and any alpha in (0,1],

    h_B(I_next,W_shift) >= (1-alpha)*h_B(I,W) = 0.

This establishes invariance of the admitted zero level and a predictive
barrier interpretation. The controller does not compute a global optimal
PCBF, prove continuity of a value function across all hybrid modes, or
construct a smooth distance CBF on the entire physical state space.

A fresh feasible plan may replace the witness only without extending its
active exit deadline. Confirmed release removes target obligations and
retains road feasibility. At an empty suffix, terminal invariance supplies
the next input directly, without a terminal membership recheck. The full
conditional shift proof and sensor contract are in
[INFORMATION_STATE_PCBF.md](INFORMATION_STATE_PCBF.md). Unlike the general
soft-constraint PCBF value-function theorem, this implementation uses only
the feasible zero level. See Huang et al.,
[Predictive Control Barrier Functions](https://arxiv.org/html/2502.08400v2),
and Wabersich and Zeilinger,
[A predictive safety filter](https://arxiv.org/abs/1812.05506), for the
underlying value-function and stored-backup constructions; their theorems
do not automatically establish this implementation's hybrid or numerical
premises.

## Hard sampled cruise dissipation inside the optimizer

For a fixed curvature and requested speed, let e be the five-dimensional
error from the path trim. Its exact sampled affine map is

    e_next = Phi_e*e + Gamma_e*(u-u_star) + d_trim.

Cruise synthesis computes P positive definite, K, and a checked contraction

    F = Phi_e - Gamma_e*K,       F'*P*F <= q2*P,       q2 < 1.

This model/gain synthesis condition is cached; it is not a runtime candidate
verifier. Select c=1-decayPerHold with q2<c<1, and qmid=(c+q2)/2.
For the observed center ebar and measurement radius r, define

    R = chol(P),
    eLower = max(0, norm(R*ebar)-norm(abs(R)*r)),
    uFeedback = u_star-K*ebar,
    nu = norm(R*Gamma_e*(u_0(w)-uFeedback)).

The hard second-order cone is

    nu <= (sqrt(qmid)-sqrt(q2))*eLower + epsilon_num.

Its right side is known before solving. It allows safety constraints to
modify the nominal feedback within a quantified dissipation budget. Input
or slew saturation is therefore handled inside the constrained problem.
There is no post-solve quadratic CLF acceptance test.

For e=ebar+eta, abs(eta)<=r, the applied input gives

    e_next = F*e + Gamma_e*K*eta
                    + Gamma_e*(u_0-uFeedback) + d_trim.

Since eLower <= norm(R*e), the cone and contraction imply

    norm(R*e_next) <= sqrt(qmid)*norm(R*e) + epsilon,
    epsilon = norm(abs(R*Gamma_e*K)*r)
              + norm(abs(R)*abs(d_trim)) + 2*epsilon_num.

The second numerical allowance covers the assumed cone feasibility error.
Weighted Young's inequality yields the reported guarantee

    V_next <= c*V + b,
    V=e'*P*e,       b=c/(c-qmid)*epsilon^2.

The implementation sets epsilon_num from the declared primal tolerance,
input reach and Gamma/P scaling. The bound assumes the actual cone error
is at most epsilon_num. It is a numerical design allowance, not a separate
verified residual bound. With zero error and exact arithmetic, b=0 and
V decreases exponentially. Finite precision and persistent uncertainty give
practical dissipation to the bound b/(1-c).

`clfDissipationCertified=true` means the executed first hold came from a
feasible program containing this hard cone. Recovery, avoidance, carried
suffixes and terminal stopping do not promise decreasing cruise error and
publish false. Safety may require leaving the cruise-dissipation mode.
The finite return rollout's first hold can carry the cone even when its
successor cannot immediately enter the stopping set. The guarantee is
sampled, not an all-time derivative inequality inside each hold.

The full-horizon predictive policy retains its soft performance CLF cones;
its physical safety rows are hard. The hard sampled cruise guarantee above
belongs to the constrained backup policy. Fixed nonzero curvature admits a
local trim and common P. Arbitrary curvature switching needs an additional
common-metric or switching argument; it is not established here.

## Runtime and validation limits

Two decision variables avoid the large full-horizon factorization, while
cached flow templates reduce formulation work. Every safety row remains represented
in the hard formulation, including the scalar-bound reduction. Timing includes formulation, solver calls and witness transfer
separately. A frame budget limits fresh work but does not bound MATLAB or
operating-system latency. Cold admission, larger uncertain tubes and repeated
infeasible recovery attempts can overrun 100 ms.

The simulation independently measures true footprint/road margins, state
containment, input/slew behavior and sampled CLF differences. These are
experiment audits, not controller execution gates. Results and remaining
failures are recorded in
[the validation report](../report/HARD_CONSTRAINED_CONTROLLER_RESULTS_20260915.md).
No nonlinear physical-vehicle guarantee, universal encounter feasibility,
detection/re-entry guarantee or certified worst-case execution time is claimed.
