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

The soft-CLF corollary establishes feasibility of the secondary problem,
not convergence of the controller selected after avoidance. In particular,
it does not establish that the optimized CLF slack eventually vanishes or
is bounded by a summable sequence.

As a logical example, let `x_(k+1) = 2*x_k`, `W(x) = x^2`, and
`0 < rho < 1`. The soft row
`W(x_(k+1)) <= rho*W(x_k) + delta_k` is feasible at every sample with
`delta_k = (4-rho)*x_k^2`, despite divergence from every nonzero initial
state. This illustrates why feasibility alone cannot prove recovery; it
is not a counterexample to the stated joint theorem or a simulated vehicle
trajectory.

The manuscript's separate cruise-law argument is local and conditional on
containment of its nonlinear ray-average Jacobians in a recorded zonotope.
The manuscript explicitly says that this enclosure is constructed and
checked using finite samples, and is not a universal nonlinear inclusion
proof. Even a sound local cruise-law certificate would not, on its own,
establish entry into that region or convergence of the full optimized
output-feedback controller. A theoretical claim of inevitable return to
cruise therefore requires an additional argument. This does not invalidate
the joint recursive-feasibility theorem.

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
- The two scalar implication examples were checked algebraically; neither
  is a vehicle experiment or a counterexample to the stated theorem.

No manuscript or runtime algorithm was modified. No MATLAB simulation,
controller campaign, literature novelty assessment, full observer-proof
audit or universal nonlinear vehicle certification is claimed. The
appropriate status is: the focused review found the conditional joint
recursive-feasibility proof structurally closed; it did not establish an
unqualified absence of all theoretical errors or a global recovery theorem.
