# Strict two-vehicle recursive-feasibility requirement

Decision date: September 11, 2026.

This is the current controller research requirement. It supersedes the earlier
finite-perception-encounter scope. The implementation at baseline commit
`5ddff0a1fce63b9e66688e43aa6745305883d720` does not yet satisfy it. Its existing
finite-encounter certificate must not be presented as an indefinite guarantee.

## Scenario and acceptance condition

There are exactly two vehicles: the ego and one persistent target. Both full
states are available at each control sample. The target follows the declared
prediction exactly. The controller must retain that target irrespective of
separation distance. Detection range, visibility, first-detection events,
missing-detection logic and perception exit are outside this problem.

Let `P_N(z_k)` be the controller's hard-constrained planning problem, including
its terminal continuation condition. The required implication is

\[
 P_N(z_{k_0})\text{ feasible}
 \quad\Longrightarrow\quad
 P_N(z_k)\text{ feasible for every }k\ge k_0.                 \tag{1}
\]

Here the state evolves under the controller's issued, correctly applied inputs.
The premise is required once, at any admitted frame. It is not an assumption
that the next solve succeeds, and it is not feasibility of an arbitrary safe
prefix with no continuation condition. Collision, road, state, input and
input-rate constraints remain hard. CLF tracking relaxation does not relax
these conditions. Reference-speed convergence is a separate performance claim.

The model must also establish collision avoidance throughout each held control
interval. A sampled-position constraint alone is insufficient. A target that
moves far away remains part of the deterministic future unless its permanent
irrelevance is proved; distance at one time cannot discharge it.

## State, target prediction and ego execution

Use the augmented state

\[
 z_k=(x_{e,k},x_{t,k},u_{k-1},t_k,q),\qquad z_{k+1}=F(z_k,u_k),              \tag{2}
\]

where `q` retains any parameters or phase needed to identify the single target
trajectory and the fixed physical road. Include additional actuator memory if
required by the execution model. The existing no-delay held-input interface is
sufficient only when those commands are actually applied at the sample time.

One target propagator must define both prediction constraints and target truth.
It must obey the shift identity

\[
 x_{t,k+1}(\tau)=x_{t,k}(\Delta+\tau),\qquad \tau\ge0,                    \tag{3}
\]

for every future time used in the remaining plan and terminal proof. For an
autonomous state model this is its flow semigroup property. A prescribed
trajectory may instead be indexed by absolute time. A fresh forecast may not
change the previously declared future while still claiming exact prediction.
The all-future target law is needed to establish the terminal continuation;
perfect knowledge over one finite window alone is insufficient.

In the current code, `targetPrediction.finiteFlow` uses Cartesian constant
acceleration plus bounded jerk, whereas `targetPrediction.nominalFlow` uses
constant curvature and tangential acceleration, with a stopping convention.
Even with zero declared error and derivative bounds, these are different
trajectories in a turn. The migration must select the intended deterministic
law and use it consistently. Merely zeroing every bound does not do this.
For speed 8 m/s, yaw rate 0.2 rad/s and initial acceleration `[0;1.6]` m/s^2,
the two position predictions differ by 0.053293348807 m after one second.
The Cartesian zero-bound arithmetic enclosure is below 1e-10 m in this
example. A turn needs nonzero Cartesian jerk; this is a model-contract
counterexample, not evidence that a correctly declared jerk enclosure fails.

Exact state measurement does not imply exact ego prediction. The proof below
uses an ego transition that describes the executed ego motion. This can be the
nonlinear bicycle's exact held flow in an ideal model study. An LTV approximation
to that nonlinear plant must instead carry a proved residual enclosure and a
robust successor construction. The new scenario removes observation uncertainty
and target prediction error; it does not by itself authorize deleting the
nonlinear ego model, its approximation residuals, or arithmetic reserves.

## Hard MPC and a constructive successor

Let `C(z,u)` mean that a held interval starting from `z`, under input `u`, meets
all physical constraints for every intermediate time, including command slew
relative to the previous input. Choose a jointly safe terminal set `Z_f` and a
terminal law `kappa_f` satisfying

\[
 z\in Z_f\ \Longrightarrow\
 C(z,\kappa_f(z)),\qquad F(z,\kappa_f(z))\in Z_f.                         \tag{4}
\]

The clock and target state in (2) allow a time-varying physical terminal set to
be written as one set in augmented coordinates. In ordinary coordinates the
condition is `F_k(Z_f(k),kappa_f) subset Z_f(k+1)`.

The planning problem is

\[
\begin{aligned}
 \min_{u_{0:N-1},\delta_{0:N-1}}\;&J(z_k,u,\delta)\\
 \text{subject to }\;&z_0=z_k,\quad z_{i+1}=F(z_i,u_i),\\
 &C(z_i,u_i),\quad i=0,\ldots,N-1,\\
 &z_N\in Z_f,\qquad \delta_i\ge0 .                                  \tag{5}
\end{aligned}
\]

Any CLF rows must admit a finite nonnegative slack for every otherwise feasible
candidate; neither a finite slack cap nor hard performance decrease may remove
the guaranteed successor. No optimality assumption is needed for feasibility.

**Conditional theorem.** Suppose (2)--(4) hold, physical geometry and limits
remain consistent, the ego executes the modeled held input, and an admitted
feasible candidate for (5) is available. Then (1) and interval safety hold under
any controller that selects feasible candidates for (5).

**Proof.** Let `(u_0,...,u_{N-1})` be the accepted candidate and
`(z_0,...,z_N)` its states. After executing `u_0`, the measured next state is
`z_1` by the execution and exact-prediction assumptions. At the next frame use

\[
 \widetilde u=(u_1,\ldots,u_{N-1},\kappa_f(z_N)).                       \tag{6}
\]

The first `N-1` intervals are the same physical future intervals as before;
(3) preserves target motion. Their road, state, collision, actuator and slew
conditions remain satisfied. The retained previous input in (2) makes the
first shifted slew condition the old second slew condition. Equation (4)
certifies the appended interval, its entry slew and the new terminal state.
Choose finite CLF slacks if needed. Thus (6) is a feasible candidate for the
next problem. The same argument applies when `N=1`, when the retained prefix
is empty. Induction proves feasibility at every later frame, and `C` proves
safety between frames. There is no perception-exit stopping time. QED.

If the ego has nonzero disturbances, replace each open-loop candidate by a
certified causal policy/tube. Every realized successor information set must
admit the conditioned tail, and (4) must hold robustly for the same dynamics,
command memory and terminal policy. Exact target prediction alone does not
supply this extra proof.

This is the standard invariant-terminal MPC successor argument, specialized
here to joint vehicle state, absolute target time, held-interval safety and
slew. See Rawlings, Mayne and Diehl, *Model Predictive Control: Theory,
Computation, and Design*, 2nd edition, sixth printing, Section 2.3 (control
invariance and feasible-set recursion) and Section 2.4.5 (time-varying terminal
conditions), available from the [authors' book page](https://sites.engineering.ucsb.edu/~jbraw/mpc/).
The source supports the general MPC construction; it is not a proof for this
repository's current numerical solver or nonlinear plant.

## Terminal safety must include the moving target

An ego-only stopping set is insufficient: an exactly predicted target can
subsequently hit a stationary ego. A candidate terminal set for the exact,
undisturbed nonlinear bicycle is the set of **true stationary equilibria**
whose full footprint is inside the road and remains separated from the target's
entire future swept footprint. An admissible equilibrium input must balance
any known longitudinal bias, and terminal entry must obey slew constraints.
At an exact equilibrium the ego footprint is constant. Shifting the clock
removes a prefix of the target's already checked future; hence all-future
separation persists and the same equilibrium input proves (4).

This establishes a sufficient terminal-set definition, not an implemented
membership algorithm or proof of reachability from every safe state. It may be
empty for some target futures or roads. The old finite-velocity affine rest
budget is a different set and retains the nonlinear counterexample documented
in `TERMINAL_CBF_PROOF.md`. Rounding a small velocity to zero or replacing
terminal invariance by a small endpoint tolerance does not prove the theorem.
A larger invariant maneuvering or cruise set is possible but needs its own
joint safety and model proof.

A simple counterexample explains why the terminal condition is essential.
Let two longitudinal point vehicles have no controllable change in speed,
required gap 1 m, one-second sampling, ego speed 2 m/s and stationary target.
With initial gap 4 m, a one-step prediction is safe throughout, ending at a
2 m gap. The next one-step problem is infeasible: its gap falls below 1 m
before the end. Both states and the target future are exact. Initial finite
horizon feasibility alone therefore does not imply (1).

## Implementation obligations and validation

The current solver preserves a finite admission program and its consumed
prefix. It has no invariant terminal extension. `completionRows` uses
`model.perceptionRange`; `validateAdmission` requires complete perception;
`localCheckTargets` ends the encounter at range exit; the controller returns
no command afterward. Increasing the range, deleting exit rows, lengthening
the horizon or attempting a fresh solve cannot supply (4).

The next implementation must replace those range-dependent admission and
completion semantics with the strict two-state interface, one deterministic
target propagator, a validated joint terminal continuation and executable
shift/append logic. Physical road support remains necessary and must not be
confused with sensor range. A coordinate re-expression of the same road must
preserve its physical constraints. The 20-source-file core limit remains in
force; a second finite-sensing controller path is not the requested design.

An online numerical solver can fail even when the mathematical problem is
feasible. Construct and retain (6) before attempting improvement. Accept a
replacement only after it passes the same hard conditions. Relinearization,
new separating planes, changed trust regions or different tube enclosures
must preserve a representation of the successor; otherwise their newly built
convex subproblem does not inherit the theorem. A numerical deadline guarantee
also requires the certified candidate to remain executable while improvement
runs. The synchronous current solver does not establish that timing property.

Required implementation checks are range-independent admission and commands;
exactly one persistent target at every sample; deterministic prediction shift
consistency (including turns and stopping); repeated feasible successors well
beyond the original horizon; terminal entry and indefinite terminal execution;
input/slew and full-interval road/collision checks; solver-failure continuation;
and rejection of changed target dynamics, inconsistent ego execution or
physically changed constraints. Test runs are evidence of implemented behavior,
not replacements for (4) or a proof over infinitely many frames.

The present change updates the requirement and reports the existing guarantee
scope honestly. `recursiveFeasibilityClaimed` retains its finite-encounter
meaning for current consumers; `recursiveFeasibilityScope` explicitly limits
it to verified perception exit. `indefiniteRecursiveFeasibilityClaimed` and
`terminalContinuationCertified` are both false at admission and continuation.
No range-independent controller, nonlinear terminal membership, complete
strict-scenario simulation or indefinite implementation guarantee is claimed.

Validation of this requirement audit: `hardEncounterBarrierTest` passes all
37 tests, including two new admission/retained-continuation scope checks.
`targetPredictionTest`, `finiteSensingControllerTest` and
`controllerSourceBudgetTest` pass another 24 tests (61 total, no failures or
incomplete tests). Factory Code Analyzer reports zero findings in the four
changed MATLAB files. The full repository suite was not rerun because the
control equations, constraints and solver behavior are unchanged.
The one-step counterexample was evaluated as gaps `[4,2,0]` m against a 1 m
required gap. The turning predictor discrepancy above was evaluated in MATLAB
and is covered by a regression in `targetPredictionTest`. These results
validate the identified limitations, not the requested future implementation.
