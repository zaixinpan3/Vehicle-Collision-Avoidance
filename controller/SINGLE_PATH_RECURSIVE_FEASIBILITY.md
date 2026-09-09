# Single-controller recursive-feasibility requirement

Decision and implementation status: September 9, 2026.

## Required behavior

The controller has one safety formulation and execution
contract, without a selectable weaker lookahead policy. Recursive feasibility
must follow from an explicit successor-witness construction rather than an
assumption that every ordinary online solve remains feasible. Model enclosure,
sampling, held-input execution and sensor assumptions remain explicit.

The accepted research assumption is:

> At the first detection of a new target, the augmented joint state, including
> all retained targets and execution obligations, belongs to the controller's
> certifiable feasible domain. States outside that domain are outside the
> research scope.

This assumption concerns the joint problem. Testing only the new target, or
only the old target set, does not meet it. It does not permit ignoring an
observed target after joint verification fails. No control is issued in that
case, and that failure response is not itself a safety guarantee.

## Implemented admission step

The version-13 hard encounter certificate now appends a new target's swept
collision constraints and endpoint exit constraint to the original hard
program. The old constraints, absolute deadline and executed controls are
preserved. Admission requires a checked joint decision. The incumbent can be
reused only after passing all added constraints. Each target retains its own
detection-step origin for every later prediction and observation check.

The certifiable domain used here includes the remaining original deadline and
the stored geometry. It is deliberately narrower than the set of physical
states admitting some nonlinear maneuver with some different horizon. A
negative solver result is not proof that either mathematical set is empty.

The new information may reduce the accepted positive margin. The requirement
at the event is a nonnegative joint margin; afterward the existing successor
witness preserves that new margin. No repeated feasibility assumption is used
between detection events. See `HARD_PREDICTIVE_CBF.md`, Section 5, for the
implemented argument and diagnostics.

## Remaining terminal obligation

First-detection feasibility does not supply a successor after the last stored
input. This is already visible when no new target is detected: the new-target
assumption imposes no condition on that endpoint. The existing perception-exit
inequality constrains target distance at a finite time; it is not a proof that
road, model-domain, input and slew constraints remain feasible afterward.

A continuing MPC construction requires a certified terminal continuation. If
`C_f` is its terminal set and `kappa_f` its feedback witness, the required
property is that every admissible state in `C_f`, under all declared
disturbances, stays safe throughout the next held interval and returns to
`C_f` at its end. The augmented state must retain previous input, target
obligations and any execution queue. A shifted plan can then append this
witness. An equivalent complete invariant continuation certificate could serve
the same role, but a fresh unproved finite-horizon solve cannot.

The terminal model and constraints must also correspond to the scheduled
prediction and the actual execution inclusion. The counterexamples in
`TERMINAL_CBF_PROOF.md` rule out simply restoring the retired affine rest set
as a nonlinear or persistent-disturbance guarantee. They do not rule out a
new appropriately designed terminal set.

The accepted first-detection assumption has not been broadened into an
assumption of terminal feasibility, safe invisible-target behavior, or
unconditional solver completion. Those are different premises.

## Completion criteria and present status

The old controller branches and selectors are removed. Version 13 uses one
complete-horizon optimization for zero or more targets, with no discrete
maneuver selection, delayed-input execution, nominal-only safety suffix or
nonreturn-route contract. Continuation always attempts the same hard program
and can execute its independently checked incumbent if replacement fails.
The first-detection joint-admission assumption remains unchanged.

The broader continuing-driving requirement is still unfinished: no executable
invariant terminal continuation has been established. At finite completion,
the controller returns no further input. `physicalVehicleGuaranteeEstablished`
remains false. Removing branches does not fill that proof gap.

Completing that broader requirement requires a proved terminal continuation
integrated into the same optimization. Validation must cover operation past
the original horizon, new-target admission near that endpoint, uncertain
successors, and solver failure with a complete successor witness. Passing the
finite-encounter tests alone does not establish those results.

## Validation of the branch removal

The complete MATLAB suite ran 593 tests: 592 passed and one retained an
obsolete expectation of measured wheel output after admission rejection.
That assertion was changed to require absent plant measurements. Two
straight-scene tests and two encounter-audit tests were then rerun successfully;
the latter also validate the corrected current-suffix CLF-slack audit index.
The resulting verified coverage is 593/593 with no unresolved failures or
incomplete tests. This is a full run plus four targeted reruns, not a second
full execution. A preceding focused migration regression passed 96/96 tests.

Both complete-interval geometry kernels compiled successfully. Factory Code
Analyzer checks on 44 changed/new MATLAB files found 16 sparse-index
performance advisories and no other findings. Old empirical short-prefix
physical profiles are tested for their actual full-horizon admission rejection;
those tests do not demonstrate successful physical avoidance or cruise recovery.
