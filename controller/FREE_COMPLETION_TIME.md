# Separate the planning window from encounter completion

Scope update, September 11, 2026: this document describes the existing finite-encounter
construction. The current requirement is the range-independent two-vehicle,
exact-target-prediction problem in [SINGLE_PATH_RECURSIVE_FEASIBILITY.md](SINGLE_PATH_RECURSIVE_FEASIBILITY.md),
including feasibility at every subsequent frame. Statements below that place
post-exit continuation outside the requirement are superseded. The finite
implementation and its proof do not yet establish that stronger guarantee.

Implemented September 10, 2026, certificate version 15.

## Corrected requirement

The requested implication is: under the declared assumptions and successful
first joint encounter admission, subsequent problems remain feasible and the
vehicles remain collision-free until finite target exit. No requirement says
that exit must occur within `controller.horizonSteps * sampleTime`.

Version 14 incorrectly used that configured window as a mandatory exit time,
a target-motion expiry, and the end of target-free execution. Version 15
removes that coupling. This is an implementation correction, not a relaxation
of the user's safety requirement.

## One complete-witness controller

`hardEncounterBarrier.plan` starts from the configured planning window. If a
candidate fails its terminal exit requirement but its nonterminal constraints
have a checked feasible solution, admission constructs one more complete
prediction interval and solves again. Every successful admission includes
all swept collision, road, model, actuator and slew rows through its selected
finite completion time. CLF relaxation remains performance-only.

The terminal-free feasibility probe is internal to numerical search. It never
issues a command, creates an encounter certificate, or substitutes for the
complete witness. There is no alternate control law, partial-safety execution
policy, mode selector or unverified tail. All intervals in the accepted plan
use the same formulation and independent checker.

The accepted witness length M can exceed the configured initial window N.
`planningWindowSteps` identifies N; `horizonSteps`/`remainingSteps` describe the
actual certificate, and `certificateExtensionSteps` exposes the remaining
part beyond N. The whole control plan remains in one decision vector.

As with numerical optimization, admission search has a computation budget.
`solver.certificateSearchTimeLimit` limits wall-clock search work (default
5 seconds); it is not a constraint on trajectory duration or target exit time.
Expiration reports `certificateSearchLimit` and does not declare mathematical
infeasibility. A failed finite-prefix probe, unavailable road coverage or
unverified numerical solve also supplies no complete witness. This is a
conservative constructive search, not an exhaustive free-final-time solver.
The existing strict physical frame deadline remains separately enforced by
the scenario driver.

## Recursive feasibility and finite exit

Let an accepted certificate contain M held controls and robust interval
constraints, with a certified exit at its terminal point. After k executed
intervals, the stored decision itself remains a feasible candidate for the
same problem with those k inputs fixed. Independent verification authorizes
that remaining suffix even if replacement optimization fails. The previous
proof applies with the **selected M**, rather than with a user-imposed exit
window N.

Until exit, the finite integer M-k decreases once per executed interval.
The stored terminal exit condition establishes completion by the end of that
witness under the declared inclusion and execution assumptions. Ordinary
reoptimization cannot repeatedly postpone its completion time. The terminal
time is an internal property of an already feasible witness, not a traffic
requirement inferred from the initial planning window.

New targets entering an already active encounter still require joint admission
with all retained obligations, including the completion commitment already
certified for that encounter. Their forecasts start at their own detection
times. This conservative joint domain remains an explicit assumption; the
change does not claim that all new traffic is admissible. When all old targets
have exited, a newly detected target can start a fresh complete admission.

The target-motion descriptor now states `validityScope=whileEncounterActive`.
It does not invent an expiry from the configured window or assert that motion
bounds hold forever after exit. Geometric exit is verified from conservative
position sets, intersecting the retained ego enclosure with the current
measurement before computing box separation. A coarser measurement cannot
erase an already certified terminal exit. Consuming the last stored input alone never reports completion.

## Target-free execution

A target-free certificate has no encounter exit obligation. When its finite
prediction expires, the same formulation may issue a newly checked target-free
plan. First target detection starts a fresh complete encounter admission from
the current state and the exact previously applied input; it does not inherit
a spent cruise clock. Neither event bypasses the input/observation contract.

The requested encounter theorem begins with target admission. Target-free
renewal is therefore not labeled as recursively guaranteed across an arbitrary
sequence of future solves. Its metadata reports `recursiveFeasibilityClaimed`
as false. This change makes no indefinite post-exit safety claim.

## Validation and limits

The dedicated tests cover a complete certificate longer than the configured
window, continuation beyond that window with a failed optimizer, repeated
target-free windows, late first detection, and motion validity independent of
window length. Existing tests continue to check all hard rows, uncertain
successors, joint target retention, input identity and failed/unsafe solves.

`verifyFreeCompletionTime` propagates the exact declared affine flow with
matrix exponentials, forces every continuation optimization to fail, and
samples rectangle clearance at 41 points in every interval. Both trials start
at ego speed 8 m/s with a target 8 m behind moving at -8 m/s; residual rates
are zero, vehicle/target dimensions are the configured defaults, and there
are no road boundaries on the straight reference. No random seed is needed.

| Initial window | Selected certificate | Sensor radius | Completion distance | Minimum sampled clearance beyond 0.25 m |
| ---: | ---: | ---: | ---: | ---: |
| 0.2 s | 0.4 s | 13.5 m | 14.399867 m | 2.95 m |
| 1.6 s | 1.7 s | 37 m | 37.150000 m | 2.95 m |

These demonstrate that 1.6 s no longer bounds certified encounter duration,
and that a retained complete witness remains executable after the configured
window. They are declared-affine-model validations, not PassVeh14DOF or
nonlinear Fiala vehicle tests. The previously diagnosed physical residual
inflation, fixed-normal conservatism, sensed-road identity mismatch and frame
runtime failures are not resolved by this correction.

Validation closure: the full 600-test run passed 599 tests and exposed one
old target-free-expiry expectation. After correcting that expectation and
adding two terminal-observation regressions, 83 related tests passed. Combining
those replacement results with unaffected full-run results covers 602 distinct
current tests, all passed; this is not a second full-suite run. Factory Code
Analyzer reported zero findings in all ten changed MATLAB files. Both exact
flow replays above passed again after the final implementation edits.
