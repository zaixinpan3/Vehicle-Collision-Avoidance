# Real-time-iteration PCBF/CLF control

Each reference supplies one RTI step: one affine dynamics and geometry model,
with at most two native convex solves on that model. The primary objective
minimizes PCBF deficits; the secondary minimizes CLF slack at the attained
primary cap. If either solve reports primal infeasibility, the controller may
regenerate the reference once and retry within the original call deadline.
There is no convergence-driven nonlinear iteration, elastic restoration, merit
acceptance loop or trust-region backtracking.

The optimized actuator inputs are issued unchanged. A nonlinear rollout predicts
their effect and updates the next frame's trust scale; it does not certify the
nonlinear constraints, accept a different iterate or repair an input. A failed
retry, unfinished solve or exhausted call budget produces no command. No previous-plan
execution or alternate controller substitutes for a solved command.

This is an approximate numerical implementation. Recursive feasibility concerns
the ideal nonlinear problem and its causal information-set policies, independently
of numerical solver success. See [ROBUST_RECURSIVE_FEASIBILITY.md](ROBUST_RECURSIVE_FEASIBILITY.md)
and [OBSERVER_ROBUST_PCBF_THEORY.tex](OBSERVER_ROBUST_PCBF_THEORY.tex). A completed
RTI subproblem is not nonlinear convergence, full nonlinear safety, or a
certificate of those theorem premises.

## Reference and horizon

The state is `x = [px; py; yaw; vx; vy; yawRate]` and the input is
`u = [frontWheelSteeringAngle; signedLongitudinalForceRatio]`. The modified-Fiala
bicycle blends into a kinematic model at low speed. RK4 and its variational
equations use the same mesh. On the default one-step mesh the midpoint is the
endpoint interpolant.

A usable previous input sequence is shifted and rolled out from the current
measured estimate. Its absolute endpoint is retained while the horizon shrinks
one hold per frame, down to `controller.horizonSteps`. This also applies when
the target disappears. When a target is present, the current geometric
encounter preview can request a longer horizon if its forecast has changed.
This length selection does not evaluate terminal membership. Missing tail
holds from a terminal endpoint prefer a checked cruise hold, then a bounded
cruise-to-braking seed search. This
initializer is distinct from the unique terminal braking law. These are
reference construction checks, not post-solve input checks or robust terminal
certificates. When no certificate is found, a moving endpoint uses nominal
feedback and a stopped endpoint keeps a nonpositive longitudinal command.
Thus the terminal certificate does not become a parking objective.

With a previous trajectory, RTI first uses its shifted rollout. Every fresh
reference with a target invokes the second-order circulation planner; no
target uses lane feedback. Local primal infeasibility also invokes this
planner, without a separate risk-screening gate. The encounter horizon starts
at `controller.horizonSteps` and expands using measured motion and NRMM
prediction. The initial search spans the larger of that prefix and the current
stopping time plus settling time, avoiding distant intersections caused by
nearly equal velocities. This chooses a seed length without testing terminal
membership. An infeasible fresh RTI problem may retry on a longer horizon.

The planner generates one trajectory through the declared vehicle dynamics.
There is no candidate bank, weighted score or candidate admission gate. At the
first initialization, the circulation sense follows the existing target/path
passing-side convention. It persists within the encounter; a small forward
tangent advantage does not reverse the route-relative passing side.

The nominal path is an acceleration attractor. In Frenet coordinates, with
lateral displacement e, transverse speed v_n, forward speed v_t and route
curvature kappa, the preferred transverse acceleration is

    a_n = kappa v_t^2 / (1 - kappa e) - 2 lambda v_n - lambda^2 e.

The speed loop restores route progress. Before input mapping, the preferred
acceleration along the current velocity is set by cruise-speed error. This
prevents target-frame transport from making the ego track the target's
translational speed. Wall guidance is applied after that adjustment. The field starts damping
outward motion before the vehicle reaches its maximum deviation. Target-frame
transport includes translation, rotation, angular acceleration and footprint
reference offsets. The target rectangle and the ego's directional body-yaw
support determine clearance; position and yaw uncertainty use directional
supports. The target enclosure restarts each hold, matching the current RTI
implementation rather than introducing a separate open-loop forecast tube.

The obstacle adds only the positive outward-normal correction to this route
acceleration. Circulation activates with that correction and adds acceleration
in the persistent passing direction only when the route field supplies less.
A receding target does not attract the ego toward a target-centred orbit, and
circulation does not slow an already sufficient tangential motion to a fixed
orbit speed. There is no extra physical clearance or contour-offset target.

With `road.boundaryObservation`, the planner uses oriented visible polylines
instead of path-offset walls. The drivable side is on each chain's left.
Only endpoints marked as range cuts continue as tangent rays; visible
junction corners and openings are retained. An explicit empty scan supplies
no walls. All predicted nodes use the current scan, without querying a future
map. Newly observed boundaries update both fresh and shifted reference inputs.
The optimized command is never changed by this guidance.

For the nearest feature on each chain, let d be signed inward distance,
n its gradient, H its local Hessian, and s the ego rectangle's support toward
the boundary at the rollout's actual body yaw. The virtual acceleration obeys

    n' a >= -v' H v - 2 r n' v - r^2 (d - s).

A two-dimensional halfplane projection keeps the preferred acceleration when
it satisfies these inequalities; otherwise it checks the one- and two-active
line intersections. Incompatible local inequalities yield a finite
least-violation acceleration, not a certificate or an admitted feasible path.
This is a local acceleration calculation, not a trajectory candidate bank.
Support changes with yaw are evaluated along the rollout, not differentiated
in this heuristic. Arbitrary nonconvex roads therefore do not acquire a road
invariance or global reachability guarantee.

For callers without a boundary scan, the virtual parallel walls at e = +/- W, with
`planner.guideHalfWidthMeters=9`, modify only the transverse acceleration.
Writing f = kappa v_t^2 / (1 - kappa e) and r = `planner.wallRate`,
the field clips it to

    f - 2 r v_n - r^2 (W + e) <= a_n
       <= f - 2 r v_n + r^2 (W - e).

The wall rate is separate from the path-attraction rate: early recovery toward
the centerline and approach to an outer boundary have different roles.

With exact acceleration realization and initial nonnegative wall distances
and first-order barrier states, these are the second-order wall comparison
inequalities, a special case of the high-order barrier construction in
[Xiao and Belta (2019)](https://arxiv.org/abs/1903.04706). They anticipate
outward velocity, unlike a position-only wall repulsion. This is guidance, not a hard RTI or terminal-set constraint. Actual
Fiala tracking, input limits, observer error and subsequent RTI changes prevent
claiming wall invariance from this virtual field alone. `Inf` disables the
legacy wall fallback. The simulation separately stops at an actual deviation above its
`LateralDeviationLimitMeters` threshold (10 m by default).

The existing yaw loop and inverse Fiala map convert the acceleration into
steering and signed braking ratio. Every state is obtained from
`nonlinearBicycleModel.sample`; no affine-input barrier projection is claimed.
A near-stopped seed completes braking instead of accelerating into the
encounter again. The single rollout returns exactly the requested number of
holds, even when its endpoint is nonterminal or still moving outward. It does
not evaluate a separate terminal continuation or search for a terminal
endpoint. Actual-yaw
nominal and inflated rectangle gaps, state violations and maximum deviation
are diagnostics; a deficient seed can still be passed to RTI and is never a
robust certificate.

The initial trajectory is the field's output, without a global or local
minimum-deviation claim. One forward pass costs O(N) for the requested N
holds. The states and controls supply the RTI linearization center; only the
optimized command is executed.

A solver-reported primal-infeasible subproblem triggers terminal-constrained
horizon search. A failed shifted reference first gets a fresh reference at the
same length. If that also fails, the requested length grows geometrically
(ceil(1.5 N), at least one additional hold). This is a search schedule, not a
fixed time increment, physical terminal-time assumption or attempt limit.
At each larger length, a previous accepted plan remains available for shifting
and extension; a failed fresh guide does not erase it. Without a previous plan,
each larger reference is generated from the current state. Every reference
supplies one RTI model and the PCBF/CLF objectives on that model.

The search succeeds when optimization completes with the endpoint terminal
constraints included. It does not require the heuristic seed to be terminal,
nor does it add a post-solve nonlinear feasibility veto or alter the optimized
inputs. Thus the mathematical stopping condition is terminal-constrained
feasibility; the implementation retains its existing affine RTI scope and
makes no additional nonlinear-membership certificate. The common computation
deadline and an explicitly configured finite `maximumHorizonSteps` can stop
search without a solution. Numerical iteration exhaustion returns failure. An
unavailable shifted reference or prediction requests fresh guidance within
the same deadline. An invalid fresh reference/prediction returns a separate
initialization or prediction failure: extending cannot repair an invalid
reference prefix, and an invalid prediction is never issued as a command.

Each reference rebuilds its dynamics tangents, separation directions, duals,
uncertainty tightening and terminal rows. The shifted plan uses its prior trust
estimate; fresh references use the configured initial trust scale. Model
geometry, physical limits, observer information and the sample timer stay
fixed throughout the search. No attempt resets the deadline.

`controller.horizonSteps` is the minimum requested length. Encounter prediction,
the shifted previous horizon, and terminal-constrained horizon search can request
more holds, subject to `maximumHorizonSteps` and the shared deadline. Neither
the field planner nor an outer reference-extension loop requires the seed's
endpoint to belong to the terminal set. The PCBF optimization retains its
endpoint terminal constraints, linearized around that possibly nonterminal
seed. A shorter reference may require a retry or remain locally infeasible;
removing seed membership checks does not guarantee RTI success. The recursive
feasibility assumptions concern the feasible optimized trajectory and its
terminal continuation, not the heuristic used to initialize the solve.

Road boundaries are deferred by default (`controller.enforceRoadBoundaries=false`).
Input road metadata supplies the nominal path and optional perceived boundary
guidance; edge inequalities are absent from the horizon and terminal continuation.
Offline truth replay independently audits complete footprints in the scenario
road, including polygon unions with junctions. It does not admit or alter controls.
An explicit opt-in retains the optional road-constrained experiment.
The 10 m experiment threshold is absent from these optimizer rows and from
the terminal certificate. Boundary guidance shapes the initialization;
truth replay measures the actual reference-point deviation and stops the
experiment if it exceeds the independently configured threshold.

Every reference is an exact declared-model rollout. The dynamics increments obey

    dx_0 = 0,
    dx_{i+1} = A_i dx_i + B_i du_i.

The dynamics, rectangle multipliers and terminal tangents are rebuilt once per
reference. The nonlinear rollout of the optimized inputs is stored separately from
the affine predicted states. No second model is constructed from that rollout.

## Constraints and objective hierarchy

State and input increments lie in local trust boxes. Actual longitudinal-force
magnitude and optional slew limits remain hard. Steering has no artificial
actuator magnitude or slew limit; the Fiala angle domain is enforced. State
limits, sideslip, reference-point deviation, optional road-corner rows and terminal
rows retain their existing meanings. No shared elastic variable relaxes them.

Nominal collision rows in the first `controller.horizonSteps` nodes carry
nonnegative PCBF slacks. With observer uncertainty, every tightened collision
node also carries a nonnegative robustness-deficit slack; the nominal collision
row remains hard after the prefix. These explicit collision deficits are
separate from physical state, road, handling and terminal constraints.

The primary stage minimizes `sum(xi)`. If the zero correction with zero slacks
satisfies every primary constraint, it attains the global lower bound and no
native primary solve is needed. Otherwise the native conic solver runs once.
The secondary stage caps the attained primary objective, with the configured
numerical tie, and minimizes the target-independent soft CLF condition

    V(xhat_next) <= rho * V(xhat_now) + clfSlack,
    rho = exp(-2*h/clf.convergenceTimeConstantSeconds).

`V` is the single precomputed transverse quadratic at prescribed-speed/path
trim. Only the estimate enters it; it does not ask the worst member of an
observer enclosure to contract to zero. A bounded whole-horizon tracking and
input-proximity tie distinguishes passing/recovery from following a slow lead
and stabilizes otherwise free future controls. It introduces no additional
avoidance controller. Both stages must complete; an unfinished secondary stage
does not cause execution of the primary point.

Modified Fiala dynamics include saturation at both axles. There is no extra
rear-tire nonsaturation cone in the optimization or terminal membership test.
The force-balance observer encloses saturated forces too; its interval can
widen when the measurement becomes less informative. Its separate sideslip
premise `|vy| <= tan(model.sideslipMaximum)*vx` remains enforced, with observer
support margins. Removing that premise would require a new observer enclosure.
The precomputed local cruise certificate may use a conservative nonsaturated
neighborhood; that local certificate does not impose a full-horizon cone.

Physical input bounds include force ratios -1 and +1. The optimizer uses
these bounds and issued inputs are unchanged. Only the tangent reference
uses `nonlinearBicycleModel.inputBounds(cfg,true)`, which keeps its force
ratio ten constraint tolerances inside the singular endpoints. Shifting a
saturated previous plan rebuilds this interior seed and its complete rollout;
this does not change a previously issued command. The 100 ms controller budget
reserves 2 ms for completion, including any replan and retry. A successful first
attempt needs one RTI model and at most two native solves. The number of
reference/horizon attempts has no fixed cap; all attempts share the deadline.

The main `runCollisionThreatValidation` campaign defaults to 25 m/s stopped
and hard-braking lead scenarios with the actual NRMM observer and synthetic
noisy sensors. `runPotentialFieldCampaign` also defaults to this joint
observer/controller setup. Exact-state replays are separate diagnostics and
do not contribute to the primary campaign score. An unsupported target
initialization is retained as `setupError`; the campaign continues with the
next case without substituting exact states. Each encounter must collide both under prescribed
cruise and under immediate maximum straight braking. The latter independent
ode45 counterfactual checks the encounter after stopping too. Slower cases
where this braking policy suffices remain regression fixtures, not evidence
of steering avoidance. For a collinear lead ahead, scalar longitudinal
comparison makes full braking the minimum-progress trajectory. Any other
admissible zero-steering history stays at least as far forward; if full
braking reaches the lead rectangle, that history must contact it by then
or already have crossed its occupied interval. The baseline marks this
nominal-model order argument separately. For general crossing traffic, failure of one braking
policy alone does not prove that every longitudinal timing policy fails.

## Collision geometry and observer information

The fixed collision buffer and the extra planner clearance have been removed.
The reference geometry, affine collision rows and terminal continuation use
zero extra clearance; observer and model-error enclosures still tighten the
rows. Separation uses a nonzero normal, so a zero threshold excludes interior
overlap rather than relying on the vacuous ordinary set-distance bound
`distance >= 0`. Boundary contact remains the mathematical boundary; numerical
tolerances and the existing affine-model approximation are not a guarantee
of strictly positive clearance during nonlinear replay.

The target follows constant tangential acceleration and sideslip within each
prediction; braking ends at rest. Each current target observation refreshes its
forecast center and parameters while retaining the input warm start. Complete
current perception with no target clears the encounter; otherwise a missing
observation retains the last forecast. Extrapolation is not a new certificate.
The joint encounter flag latches the first exit. Within a prediction, an exit
must hold for the published position enclosures and every affine trust-box
correction before later target rows are removed, including terminal rows.
An actual reacquisition starts a new encounter ID and target epoch.

Ordinary rectangle distance supplies feasible separating dual multipliers. Four
fixed-multiplier corner rows include the yaw derivative. At overlap a nonzero
support direction replaces the zero ordinary-distance dual. An encounter-side
choice and smooth longitudinal/lateral support combination supply a consistent
escape derivative. A compatible closest-feature normal replaces it after
separation. Predominantly longitudinal encounters retain the branch across
frames; crossing targets refresh their closest geometry. These are local
convexification choices and do not establish global feasibility.

The observer publishes a relative position ball, course bound, and intervals for
speed, constant acceleration and curvature. Absolute common ego pose cancels
only from pairwise collision geometry; road and state bounds retain ego
uncertainty. Directional supports preserve generator correlation. For a fixed
normal n, target yaw contributes `max_{theta,v} n'*R(theta)*v` minus the nominal
rectangle support, where theta ranges over the entire allowed yaw interval
and v over its four offset vertices. Each maximum is found from interval
endpoints and an interior angular maximum. This exact support never exceeds
the old nominal-support-plus-chord enclosure for the same set. Ego orientation
retains the chord reserve `2*reach*sin(min(pi,b_yaw)/2)` in RTI, uniform over
changes of the nominal yaw; fixed rollout poses use directional ego supports.
Target support coefficients are calculated in a batch once per frame, then
reused for the separating normals. Their parameter-family formulas are unchanged.

The implementation restarts the present ego enclosure `G0` at each future hold
and propagates it through that hold: `G(t_i+tau)=Phi_i(tau)*G0`. The target set
similarly starts at its predicted state at each hold. These are affine images
and an assumed future posterior width. They do not certify nonlinear remainders,
future observation histories or shift-consistent joint reachable sets. No
uncertainty is removed to obtain a command. The first hold also lacks a full
nonlinear remainder certificate. `robustNonlinearSafetyCertified` remains false.

Terminal rows use the mild braking-continuation set in
[TERMINAL_SAFE_SET.md](TERMINAL_SAFE_SET.md): braking to rest at fixed sideslip
and curvature, road and target clearance along the continuation, and parked
clearance against the remaining target track. Motion at the endpoint is allowed;
the stop need not fall inside the prediction horizon. These conditions are
linearized once at the reference endpoint. Ego position/yaw margins are retained;
the target enters as its estimate. This is not full robust terminal membership.

## Computation budget and diagnostics

The current research budget is 100 ms (`solver.timeLimitSeconds=0.1`), with
at most 40 native interior-point iterations per stage and a 2 ms completion
reserve. The model sample remains 50 ms; these offline simulations do not
inject computation latency into actuation. Preprocessing
counts against the same timer as model construction and both solves. Reference
rollout, tangent/constraint construction, CLF construction, terminal geometry and
optimized-input prediction check the timer. A native termination callback includes
MEX validation and problem construction, in addition to Clarabel's iteration
timer. Native infeasibility certificates and unfinished iterates are never inputs.

Replay exports distinguish the model-sample deadline (`deadlineMisses`) from
the configured research budget (`controllerBudgetSeconds` and
`controllerBudgetExceededCalls`).

These checks bound the admitted work, not a hard wall-clock guarantee on a desktop
MATLAB process. A running factorization, allocation, initial JIT compilation or OS
preemption cannot be interrupted by the checks. Warm up before timed execution;
`collisionAvoidanceController("warmUp", cfg, road)` uses a separate startup
budget for cold avoidance planning, cruise and bounded low-speed feedback
with curved targets. The optional road also primes its actual curvature,
terminal and avoidance geometry. Every probe command is discarded.
The integrated scenario driver warms after observer initialization, including
one discarded bounded-input startup probe. It does not reuse that probe's
solution, consume the first observation or advance its random stream. These
startup costs are reported separately; first-measurement-to-actuation latency
is not established by this benchmark. Observer publication also avoids repeatedly
reordering MATLAB's search path. Actual
successful and failed controller call times must both be measured from the
same controller timer. Observer/publication preparation belongs to the
separate pipeline timer; failed pipeline latency is not solver latency. A timeout is an allowed
failure, not a successful real-time control result. Raising the budget for a
functional test does not change the bounded RTI/replanning policy.

`search.rtiIterations` and `linearizationCount` count all reference attempts;
`solverCalls` includes their native solves (at most two per model). `attempts`
preserves each result and its `horizonSteps`; `horizonExtensions` counts length increases;
`selectedAttempt` identifies the returned solution, or zero when none is returned.
`nominalRestarted` indicates reference recovery and `restartCount` counts
additional RTI models. Initialization and formulation timings sum all attempts. `convexSolveCompleted`
records completion of both convex objectives on the final reference.
`optimizationConverged` is false because nonlinear convergence is not tested.
`nonlinearValidationPerformed` is false and `nonlinearHardResidual` is unavailable.
The reported PCBF and CLF quantities belong to the affine subproblem;
`predictedState` is a nonlinear prediction of the unchanged optimized inputs.
Independent scenario replay measures clearance, road margins and nominal
recovery throughout the observation. Current-state terminal membership does
not stop simulation or count as task completion. The terminal set certifies
a possible safe continuation at the predicted endpoint; the controller keeps
executing its optimized first input on each frame. Campaign recovery requires
a sustained dwell near the prescribed path and speed after the encounter.
An optional finite observation cap is a diagnostic setting; by default
`Frames=Inf` runs until recovery or failure without a simulation-time limit.
A finite counterfactual baseline qualifies the initial encounter independently
of the closed-loop duration. Resumed observations retain their original
baseline and minimum recovery time.

Controller-state schema 82 rejects old numerical state schemas. The ideal
recursive-feasibility theorem, its model-mismatch bookkeeping and causal policy
shift are unchanged. Neither RTI success nor its time limit establishes the
ideal theorem's hypotheses for the executable controller.
