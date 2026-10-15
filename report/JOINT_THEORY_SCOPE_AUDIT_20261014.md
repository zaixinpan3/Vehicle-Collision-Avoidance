# Joint observer-controller theory: focused logical audit

Prepared: October 14, 2026, America/Chicago.
Reviewed repository snapshot: `88d50fe5b1d0839bd409bfd7feae6cddfc7217c5`.

## Question and scope

Does the joint estimation-control derivation have a remaining logical gap,
independently of numerical implementation and experiments? This audit examines
the conditional joint recursive-feasibility argument and the strength of its
conclusions. It does not treat an unimplemented set operation, an unrun
experiment, or an unavailable numerical constant as a defect in an exact-set
existence proof. It is a focused review, not a claim that every derivation in
the manuscript has been independently verified.

Sources examined:

- `controller/CLOSED_LOOP_PREDICTION_THEORY.md`, especially the joint map,
  nonlinear-image construction, finite-support recursion, policy-shift proof
  and soft-CLF argument.
- `controller/ROBUST_RECURSIVE_FEASIBILITY.md`, Sections 2--6.
- `controller/TERMINAL_SAFE_SET.md`, terminal-model assumptions and scope.
- `paper/manuscript.tex`, the safety-argument list, prediction model,
  terminal assumption, cruise CLF, joint robust subsection and guarantee scope.
- Follow-up clarification on October 14:
  `controller/OBSERVER_ROBUST_PCBF_THEORY.tex`, the section "One CLF:
  nominal dissipation with an explicit uncertainty supply", especially
  Proposition `prop:clf`, "One-solve CLF recovery with a vanishing tie".
  The initial audit omitted this existing conditional recovery proposition;
  its omission must not be read as absence of a CLF convergence argument
  anywhere in the project.

## Assessment of the main joint theorem

No substantive break was identified in the following implication chain under
its stated premises:

| Step | Logical basis and qualification |
| --- | --- |
| Truth containment | Initial and new enclosures cover the same plant, parameter and sensor contract. The joint map propagates each shared source once. |
| New root inclusion | Intersecting the retained first successor with sound new information and known software/measurement memory preserves truth and gives a subset of that successor. |
| Shifted prediction | Exact images are monotone. Retaining the old outer sets and certificates also permits the shifted candidate without requiring a fresh adaptive approximation to be narrower. |
| Future information | Policies are causal functions of available observer outputs and memory. Current data fix a common current input; later inputs may differ across future measurement branches. |
| Terminal append | A common admissible zero-deficit continuation, closure under taking its tail, and downward closure supply the final hold, including input memory and the parked phase. |
| Horizon and slack indexing | The shifted candidate keeps the same horizon. Its final prefix slack is zero, whether it comes from the old hard tail or the appended terminal hold; the one-hold case is covered. |
| Primary-value descent | For an attained primary minimizer, the shifted candidate costs the old optimum minus the first slack. Feasible suboptimal execution alone gives a comparison with its executed cost. |
| Soft CLF | An unbounded nonnegative CLF slack preserves a primary candidate when its CLF residual is finite. This is a feasibility statement. |

The exact nonlinear image definition is sufficient for this conditional
existence proof. A particular approximate image needs its own soundness
argument, but computing that approximation is not an omitted step of the
exact-set proof. The accepted mild terminal model is an explicit premise;
the review does not replace it with an unproved equality for a different
vehicle model. Initial feasibility, sound enclosures and the terminal
continuation remain hypotheses, rather than conclusions for every encounter.

## Conclusions that do not follow from this theorem

### Finite-time entry into zero collision relaxation

Nonnegative first-hold deficits are summable along the applicable infinite
closed-loop sequence, hence tend to zero. Summability does not imply that
they become exactly zero at a finite sample. For example, set

`xi_k = 2^(-k)` and `V_k = 2^(1-k)` for `k >= 0`.

Then `V_(k+1) = V_k - xi_k`, all deficits are strictly positive, and
`sum xi_k = 2`. This example tests the implication, not the vehicle model.
The stated joint theorem claims summability and zero-set invariance; these
are consistent. Any stronger finite-time safety-recovery claim would need
additional conditions. Positive slack also reduces the clearance guaranteed
at the specified constraint instants.

### Closed-loop return to cruise

The soft-CLF corollary in the joint recursive-feasibility subsection
establishes feasibility of the secondary problem, not convergence by itself.
The project also already contains a conditional CLF recovery proposition in
`controller/OBSERVER_ROBUST_PCBF_THEORY.tex`. These statements serve different
purposes and must be assessed together.

As a logical example, let `x_(k+1) = 2*x_k`, `W(x) = x^2`, and
`0 < rho < 1`. The soft row
`W(x_(k+1)) <= rho*W(x_k) + delta_k` is feasible at every sample with
`delta_k = (4-rho)*x_k^2`, despite divergence from every nonzero initial
state. This illustrates why feasibility alone cannot prove recovery; it
is not a counterexample to the stated joint theorem or a simulated vehicle
trajectory.

The existing recovery proposition supplies the missing implication when a
zero-CLF-slack candidate is admissible in the optimization from some sample
onward. Write the robust row as

`V_(k+1) - V_k <= -eta*ell_k + delta_k + sigma_k`.

Here `sigma_k` is the derived disturbance supply. For the objective
`delta_k + epsilon*ellLower_k*R_k`, with `0 <= R_k <= 1`,
`0 <= ellLower_k <= ell_k`, and `0 <= epsilon < eta`, a zero-slack
candidate costs at most `epsilon*ellLower_k`. An optimizer with objective
gap `tau_k` therefore has

`delta_k <= epsilon*ell_k + tau_k`,

and consequently

`V_(k+1) - V_k <= -(eta-epsilon)*ell_k + sigma_k + tau_k`.

On an invariant recovery domain where `ell_k >= mu*V_k` and
`0 < c = (eta-epsilon)*mu <= 1`, iteration gives

`V_(k+j) <= (1-c)^j*V_k + sum_(s=0)^(j-1) (1-c)^(j-1-s)*(sigma_(k+s)+tau_(k+s))`.

Thus vanishing supplies imply `V_k -> 0`; uniformly bounded supplies give
the corresponding practical neighborhood. Exact minimization of CLF slack
alone is the case `epsilon = tau_k = 0`. Nonzero transient estimation error
need not prevent exact recovery when its supply tends to zero.

For the simpler noise-free multiplicative row
`V_(k+1) <= rho*V_k + delta_k`, `0 < rho < 1`, an admissible zero-slack
candidate and exact slack minimization give `delta_k = 0` and
`V_k <= rho^(k-k0)*V_(k0)`. Since `V=e'*P*e`, `P>0`, this proves
convergence to the nominal path-following cruise manifold with free
longitudinal phase. No separate convergence mechanism is required.

When collision constraints cease to apply, the safety objective no longer
obstructs that candidate. The candidate must still satisfy the other hard
conditions and any retained terminal requirement. If the post-avoidance state
is in a CLF-valid invariant recovery domain where this compatibility holds,
the existing argument proves the requested recovery. Removal of collision
constraints alone does not establish that every possible post-avoidance
state belongs to such a domain. This is the domain/compatibility hypothesis,
not a need for an additional generic Lyapunov theorem.

The manuscript's separate cruise-law argument is local and conditional on
containment of its nonlinear ray-average Jacobians in a recorded zonotope.
The manuscript explicitly says that this enclosure is constructed and
checked using finite samples, and is not a universal nonlinear inclusion
proof. Even a sound local cruise-law certificate would not, on its own,
establish entry into that region or convergence of the full optimized
output-feedback controller. Applying the existing conditional recovery
proposition to a larger post-avoidance set requires the corresponding domain
and candidate-compatibility argument. This does not invalidate either the
joint recursive-feasibility theorem or the existing conditional recovery
proposition. No global recovery claim is inferred from the local certificate.

### Continuous-time clearance and road containment

The joint robust theorem deliberately omits road boundaries and states
collision clearance at the specified constraint instants. It does not imply
clearance between those instants. These are declared scope restrictions,
not hidden failures of the shift argument. Any broader safety statement
must retain these restrictions or prove the additional rows/images.

## Checks and limits

- Manual reconstruction of the image, posterior, terminal, policy and slack
  implications above, including `M = 1`, `M = N`, and `M > N`.
- `python3 scripts/checkJointMismatchDerivations.py`: passed the exact
  rational shared-mismatch, persistent-parameter, nonlinear-reserve,
  carried-certificate and common-braking specializations.
- `python3 tests/jointMismatchDerivationsTest.py`: seven tests passed.
- Follow-up: manually reconstructed the existing CLF recovery proposition
  using the zero-slack comparison candidate, the bounded vanishing tie,
  and the stable scalar convolution; no new experiment was required.
- The two scalar implication examples were checked algebraically; neither
  is a vehicle experiment or a counterexample to the stated theorem.

No manuscript or runtime algorithm was modified. No MATLAB simulation,
controller campaign, literature novelty assessment, full observer-proof
audit or universal nonlinear vehicle certification is claimed. The
appropriate status is: the focused review found the conditional joint
recursive-feasibility proof structurally closed and the follow-up confirmed
an existing conditional CLF recovery proof. It did not establish an
unqualified absence of all theoretical errors, global recovery-domain entry,
or exact recovery under arbitrary persistent disturbances.
