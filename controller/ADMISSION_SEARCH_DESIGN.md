# Overlap-aware admission search design

Status: implemented with bounded sector/exit/horizon search and equivalent sparse solves, September 16, 2026. The design requirements below distinguish search soundness from completeness; executed results are in [the implementation report](../report/OVERLAP_ADMISSION_IMPLEMENTATION_20260916.md). It builds on the [initialization diagnosis](../report/DISTANCE_DUAL_INITIALIZATION_DIAGNOSIS_20260916.md) and the [analytic-support ablation](../report/OVERLAP_SUPPORT_PROPOSAL_REVIEW_20260916.md).

## Objective and retained framework

Construct a first hard-certified finite encounter plan from a possibly colliding cruise prediction. Preserve the predictive CBF, soft CLF with squared slack penalty, input cost relative to the CLF operating point, finite observed encounter exit, invariant cruise terminal certificate, and carried prediction witness. The high-gain target observer is unchanged by this admission repair.

The straight-scene control hold and localization period are both 0.1 s. There are no artificial road boundaries, state/slip operating constraints, prescribed lateral paths, additional execution subintervals, or fallback commands. Existing physical actuator bounds and any configured finite slew bounds remain. The terminal law continues to be a prediction certificate. Failed or late admission ends the simulation without issuing a new command.

The runtime requirement applies to the entire estimator-controller frame, including preprocessing, search, solves, verification and output construction. A controller-only 100 ms measurement cannot establish the joint requirement.

## 1. One support-direction interface

Replace the ordinary-distance availability gate with an analytic configuration-polygon support query valid outside, inside and at contact. Separate its outputs into finite unit direction, signed anchor clearance, and degeneracy/tie information. A negative clearance is a search condition, not a formulation error.

For a fixed unit n and configuration obstacle C, `n'*p >= support(C,n)+margin` defines a safe convex half-space independently of whether the old anchor satisfies it. Rebuild the existing robust orientation, uncertainty and whole-hold support bounds for every candidate program. The analytic point query alone is not a safety certificate. The support inequality follows from the support-function separation bound in [Guthrie, Lemma 4](https://arxiv.org/html/2302.09704v1).

Use this interface throughout the new implementation; retire the old ordinary-distance online query and its telemetry when the replacement is validated. Do not retain a runtime selector between an old initializer and a new one. Historical reports continue to describe their original revisions.

## 2. Search compatible constraint families

Search geometric constraint choices, not a prescribed vehicle maneuver. The control sequence remains an optimization variable. No fixed lateral amplitude, steering pulse, cosine path, passing time or desired avoidance acceleration is introduced.

Start with an actuator/rate-admissible numerical input sequence. Cruise input, or an old numerical plan when encounters change, can initialize the search but supplies no safety claim for newly admitted targets.

At symmetric or nearly symmetric configurations, expose alternative support faces explicitly. Resolve ties consistently over neighboring holds instead of selecting each minimum-penetration face independently. Maintain alternatives when they define materially different feasible regions. Previous accepted directions and directions from the current candidate are retained as proposals.

For fixed candidate controls, score a proposed direction by the minimum reserved clearance across all certificate rows of its complete hold, not just its midpoint. Local angle refinement may improve the direction inside a selected geometric branch. A direction is useful even if this best margin is negative. The full trajectory solve, including actuator bounds, determines whether the family is reachable.

Check adjacent families against their common endpoint and joint dynamics. If using a cheap reachable outer set for pruning, reject a family only when incompatibility is established even in that outer set. A failed heuristic score is not proof that a branch is impossible. Avoid imposing an arbitrary normal-angle rate limit as a new physical constraint. Multiple targets require joint compatibility, not independent choices subsequently assumed to coexist.

Switching times are determined through the selected hold constraints and solved controls, with alternative switch placements explored when needed. They are not fixed in advance by a maneuver template. A bounded candidate search is an incomplete search of the nonconvex problem; it must not be called an exhaustive decomposition of the original safe set.

## 3. Feasibility restoration with grouped deficits

Attempt a complete hard convex problem for a promising family. If it is infeasible, optimize a search-only restoration problem instead of stopping at that family. Keep the measured initial state, declared affine dynamics, actuator limits and configured slew limits hard.

Let U denote the controls. Add one nonnegative collision deficit `e(c,j)` for each hold c and target j, shared by all support-majorant/Bernstein rows for that pair:

\[
a_{cj\ell}^{T}U\le b_{cj\ell}+e_{cj},\qquad e_{cj}\ge0.
\]

Here b already contains the intended physical clearance, uncertainty and numerical reserves. For each target-exit row add one deficit, and for each terminal cone add a scalar radius deficit:

\[
a_j^TU\le b_j+e_j^{\rm exit},\qquad
\|F_qU+f_q\|_2\le r_q+e_q^{\rm terminal}.
\]

Relax other future terminal-entry inequalities as needed for search, never physical input limits. The initial implementation has no road-boundary rows. Deficits must cover every future geometric/terminal requirement included in restoration; otherwise a supposedly startable restoration can remain infeasible for an unrelated unrelaxed terminal row.

Grouping does not change the zero-deficit feasible set: all original rows hold exactly when their shared deficit is zero. It limits added collision variables to `numberOfHolds * numberOfTargets`, rather than one variable per support facet and polynomial coefficient.

Minimize a positive weighted sum of normalized deficits. Collision/exit deficits use declared length scales; terminal deficits use positive modal-radius scales. Scales must be finite, positive and logged, and no constraint group may receive zero priority. A lexicographic secondary preference can select a nearby input plan among primary minimizers. It must not trade a positive primary deficit against cruise performance. The original CLF/input objective is restored in the final hard solve, not mixed into the primary feasibility objective.

For finite rows/cones and a nonempty hard input/rate set, sufficiently large deficits make this fixed-family restoration startable. This does not imply that its optimum is zero or that outer iterations reach a safe plan. Report an empty input/rate set separately.

After a control update, recompute complete held-interval margins, propose revised directions and accept progress using an explicit normalized-deficit merit. Retain the best candidate. On stagnation, change the geometric family, switch placement, horizon or fresh exit direction; repeatedly penalizing the same incompatible family is insufficient. These are search hypotheses subject to final verification, not executable policies.

## 4. Restore the original hard acceptance problem

Promising restoration output only triggers a new hard solve. Remove every search deficit, reinstate the original performance objective, include finite exit and terminal conditions, and call the existing independent hard verifier. A small positive deficit is not an acceptance tolerance.

The final hard certificate remains the sole safety admission condition under the declared model, sensing and execution assumptions. The positive-deficit search iterates are never issued or stored as certified continuations. Successful local search is not a physical nonlinear-vehicle guarantee.

The current affine generators can remain fixed during this repair. Geometry search does not require a move to nonlinear Fiala optimization or a new model-inclusion argument.

## 5. Keep admission and continuation asymmetric

Fresh or changed encounters may use the internally infeasible search above. Fresh completion direction and horizon must be explicit candidate choices rather than silently overwritten by the cruise encounter proposal.

For an unchanged admitted encounter, shift the existing certified family as today. A geometry replacement must preserve a complete feasible witness and the admitted absolute exit deadline. Failed optional replacement leaves the inherited formulation available for its hard solve; it does not dispatch a stored command or terminal feedback. Do not rerun unrestricted admission search every frame or postpone a certified deadline indefinitely.

Removing a target still requires the existing current observation contract. The old plan is only a numerical seed when new obstacles or enlarged bounds invalidate its coverage. Recursive feasibility remains conditional on successful admission and compatible subsequent contracts.

## 6. Implementation ownership and computation budget

Use the existing modules and remain within the 20-source controller budget:

| Module | Planned responsibility |
| --- | --- |
| `avoidanceSafetyGeometry` | Unified analytic support query, tie alternatives, whole-hold margin evaluation and compatible direction proposals |
| `formulateAvoidanceProblem` | One authoritative set of physical rows/cones; explicit candidate geometry; grouped search-deficit transcription and final hard transcription |
| `solveHardCbfClf` | Restoration/performance solves and unchanged independent hard certification |
| `hardEncounterBarrier` | Explicit fresh completion candidates; preserved inherited generators, terminal conditions and absolute deadlines |
| `collisionAvoidanceController` | Fresh-search versus inherited-continuation orchestration, best-candidate ownership, shared budget and precise failure reporting |

The active path constructs `program.P/A/b/cones` directly. Editing only the standalone `avoidanceStageQp` utility would not implement this design.

Batch support operations, reuse fixed affine prediction maps and unchanged cone/row blocks, and warm-start successive convex solves where supported. Benchmark single-candidate formation, restoration, hard solving and verification before choosing online iteration/candidate limits. Those limits share one remaining frame budget; each branch does not receive a fresh 100 ms allowance. Reserve time for final hard solving, certification and output construction. A bounded iteration count alone does not establish wall-clock compliance.

Record both startup and steady operation. Any intentional preinitialization must be reported separately and consistently; do not silently omit the first admission or failed frames from runtime statistics. Profiling should start with the first working admission prototype, not be postponed until an unrestricted search is complete.

## Validation order and acceptance evidence

1. Geometry behavior: outside/inside/contact/coincident configurations, deterministic tie alternatives, unit directions and correct support values. Validate complete-hold behavior when midpoint clearance is misleading.
2. First-admission controller fixtures: stationary and oncoming from the original cruise guesses, no recorded avoidance controls. Require a final hard-certified solution; merely constructing a problem or solving restoration is insufficient. Cover incompatible neighboring normals, blocked alternatives, impossible completion and changed targets.
3. Continuation regression: preserve a full witness under conditioning and rejected geometry updates; preserve absolute deadlines and the prediction-only terminal role; never execute positive-deficit candidates or a fallback.
4. Straight closed-loop controller scenes: certify every executed hold, avoid collisions, confirm encounter release and examine eventual convergence to path cruise. Extend runs when needed; do not reinstate an arbitrary early-recovery requirement. Check CLF slack and final errors rather than treating soft CLF alone as proof of asymptotic convergence.
5. Joint estimator-controller scenes: use the same physical scenes and high-gain observer; verify bound consistency and the complete 100 ms frame, including admission and failures. Controller-only timing does not substitute for this test.

Report proposal count, direction changes, restoration iterations, collision/exit/terminal deficits, final hard margins, solver calls, failed attempts and full-frame maximum latency. Distinguish invalid geometry, empty physical input set, positive residual after bounded search, selected-family infeasibility, deadline expiry and certificate rejection. A search failure is not automatically a proof of physical infeasibility.

This design does not imply that every validation case has passed. Sequential convex methods can fail to refine an infeasible initialization, as explicitly discussed after Corollary III.1 in [GuSTO](https://arxiv.org/html/1903.00155v1). Neither universal avoidance feasibility nor a 100 ms worst-case guarantee follows from the design alone.

Research assistance: AI-assisted design synthesis checked against the current module interfaces, prior measured ablations and the cited primary sources. See the implementation report for experiments; the original design decision preceded those runs.
