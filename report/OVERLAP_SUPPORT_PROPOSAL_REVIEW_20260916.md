# Review of overlap-aware support initialization

Date: September 16, 2026. Repository HEAD during review: `a6fd787`; controller source remains unchanged from `f03d5dc`. This is a design review and an offline formulation ablation, not an implementation of an admission-search algorithm.

## Assessment

The supplied proposal correctly distinguishes a support direction from a separator already satisfied by the anchor. For a convex configuration obstacle C and any unit n,

\[
n^T p\ge h_C(n)+m
\quad\Longrightarrow\quad
\operatorname{dist}(p,C)\ge m,\qquad m>0.
\]

The implication follows from `n'*(p-c) >= m` for every c in C and the Cauchy--Schwarz inequality. It does not require the old anchor to satisfy the row. The optimized candidate must satisfy it. A collision-free anchor is therefore not mathematically necessary to construct a valid convex inner approximation.

`avoidanceSafetyGeometry.rectangleDistance` already returns an outward least-penetration face normal inside the rectangle configuration polygon and a closest-boundary direction outside. `distanceDual` discards that normal and marks overlapping anchors unavailable. Replacing that particular initialization contract is justified. For changing vehicle orientation and uncertainty, the current robust support majorants and whole-hold bounds must still be rebuilt; the fixed-pose analytic query alone is not a trajectory certificate.

This use of support directions agrees with Lemma 4 of [A Differentiable Signed Distance Representation for Continuous Collision Avoidance in Optimization-Based Motion Planning](https://arxiv.org/html/2302.09704v1). Analytically querying polygon geometry does not require imposing the nonconvex unit-norm equality in the trajectory SOCP. That equality matters when signed-distance dual variables are themselves optimization variables, as in [Optimization-Based Collision Avoidance, Section 3.2](https://arxiv.org/html/1711.03449v3).

## Minimal offline experiment

The experiment changes only the direction oracle. A temporary copy of the current `formulateAvoidanceProblem` is renamed `formulateOverlapProbe`; its two calls to `distanceDualNormals` are replaced by an analytic midpoint-normal helper. Every other formulation line is retained. The original online source, robust geometry kernel, dynamics, actuator constraints, terminal rows, soft CLF, native trajectory solver, and hard verifier are unchanged. No direction grid, maneuver template, restoration slack, horizon search, or fallback controller is introduced.

Inputs are the first-admission model fixtures from the preceding [initialization diagnosis](DISTANCE_DUAL_INITIALIZATION_DIAGNOSIS_20260916.md): reference 8 m/s, 0.1 s hold, complete sensing range 16 m, no road boundaries, exact affine model, zero uncertainty/target jerk/yaw acceleration, stationary horizon 48 and oncoming horizon 32. The diagnostic budget is 60 s. No random data or seed is used.

| Result | Stationary | Oncoming |
| --- | ---: | ---: |
| Anchor overlap midpoints | 12 | 6 |
| All analytic direction norms | 1 | 1 |
| Complete formulation constructed | Yes | Yes |
| Geometry SOCP calls | 0 | 0 |
| Hard trajectory native status | 2: primal infeasible | 2: primal infeasible |
| Hard-certified trajectory obtained | No | No |

Thus analytic directions remove the construction failure, but independently selecting nearest directions does not solve either admission case.

### Temporal direction inconsistency

Stationary directions are `(-1,0)` for holds 1--15, `(0,-1)` for 16--22, and `(1,0)` for 23--48. This imposes a particular progression of half-spaces with switching times 1.5 and 2.2 s after admission. It does not require the vehicle to remain on the centerline beforehand, but the resulting coupled motion requirements can still exceed actuator capability.

The oncoming schedule additionally changes from approximately `(-1.39e-16,-1)` in hold 11 to `(0,1)` in hold 12, then approximately `(9.25e-17,1)` in hold 13. Adjacent holds 11 and 12 share the endpoint at 1.1 s after admission and demand nearly opposite lateral separation directions. The mathematical scene is nearly symmetric; an independent nearest-face choice does not resolve equivalent directions consistently across time. This is a concrete instance of the proposal's temporal-coherence concern, not merely a hypothetical risk.

### Constraint ablation

The same native conic solver is also called with zero objective on selected **physical linear rows**, without CLF or terminal cones. Dynamics remain condensed into the input-to-state maps. This uses the original physical bounds, without adding new inward numerical reserves.

| Retained rows | Stationary | Oncoming |
| --- | --- | --- |
| Collision only | Solved, status 1; 2522 rows | Approximately primal infeasible, status 5; 2002 rows |
| Collision + actuator/slew rows | Primal infeasible, status 2; 2714 rows | Primal infeasible, status 2; 2130 rows |
| All physical linear rows | Primal infeasible, status 2; 2715 rows | Primal infeasible, status 2; 2131 rows |

The actuator/slew selection preserves the configured bounds; these fixtures have no finite slew limits. These numerical ablations locate the obstruction within the selected collision geometry combined with actuator bounds. Terminal cones and the CLF are not needed for these particular failures. Oncoming's collision-only status 5 is an approximate infeasibility report, not an independently verified exact infeasibility proof. No dual-ray proof is claimed for any ablation.

The preceding diagnosis already rechecked feasible recorded controls against newly constructed current hard programs and obtained successful fresh solves for both scenes. Consequently, failure of the newly selected half-spaces must not be presented as physical collision inevitability or infeasibility of every convexification.

## Design implications and constraints on adoption

1. **Use one coherent geometry interface.** Distinguish direction generation, anchor clearance, and final trajectory certification. Analytic polygon support geometry can serve both separated and overlapping queries. It would be an extension/replacement of the specific ordinary-distance SOCP step, not an exact reproduction of Li's equation (12).
2. **Search compatible constraint families.** Direction choices must be coupled across time and targets, with dynamics and actuator bounds participating in feasibility decisions. A fixed lateral displacement, cosine path, steering pulse, or passing duration should not be introduced as the vehicle's prescribed trajectory. A few hand-selected maneuver hypotheses do not exhaust the nonconvex feasible set.
3. **Evaluate restoration as a separate search mechanism.** Internal geometric/terminal deficits can make an otherwise consistent input-admissible search startable. They must disappear from the accepted hard problem. A positive-deficit candidate is not an executable safety witness. Lexicographic priority can prevent trading feasibility deficit for cruise performance, but cannot itself escape an incompatible fixed family of half-spaces.
4. **Preserve continuation and completion semantics.** Keep the carried certified family and its absolute deadline. A replacement requires a valid complete witness. For fresh admission, terminal direction and horizon may need to participate in the same search; adding a braking seed alone does not create a compatible stopped terminal mode.
5. **Retain the undivided hold.** The suggestion to subdivide certificate intervals is not adopted: the project currently requires one complete held interval. Improve the direction proposal against the complete cell before considering any change to that requirement.
6. **Measure the full frame.** The experiment removes geometric SOCP calls but adds no admission search and measures no worst-case runtime. A future implementation must budget all search, formulation, optimization, verification, and the estimator within the common 100 ms frame. Neither feasibility in every scene nor this deadline follows from the proposed architecture.

The principle of searching from colliding guesses has precedent in [TrajOpt](https://journals.sagepub.com/doi/abs/10.1177/0278364914528132); the publisher abstract describes sequential convex optimization, collision penalties, and continuous collision treatment. That evidence does not establish this vehicle controller's timing or recursive-safety claims. [GuSTO, Corollary III.1 discussion](https://arxiv.org/html/1903.00155v1) explicitly allows failure because an infeasible initialization could not be refined to feasibility. These are methodological references, not implementation validation.

## Reproduction and scope

Original scratch experiments and outputs are retained at `/home/zai/.cache/collisionAvoidance/overlap-support-review-20260916/`: `runProbe.m`, `supportHelper.txt`, the generated `formulateOverlapProbe.m`, both `*-probe.mat` files, `summary.json`, `checkRows.m`, `row-ablation.json`, and execution logs. They are an offline transcription experiment and are not on the online controller path.

Commands actually run:

```matlab
run('/home/zai/.cache/collisionAvoidance/overlap-support-review-20260916/runProbe.m')
run('/home/zai/.cache/collisionAvoidance/overlap-support-review-20260916/checkRows.m')
```

MATLAB R2026a Update 3 returned normally for both runs. An independent Python audit verifies that the copied formulation differs only by the function name and the two intended oracle calls before the helper is appended; it also checks unit normals, the oncoming reversal, and the saved solver statuses. The conic statuses are interpreted from the installed native solver enum. No full regression suite, closed-loop experiment, observer performance test, automatic restoration implementation, or real-time success is claimed. Online initialization remains unresolved.

Research assistance: AI-assisted source review, primary-source comparison, derivation and offline ablations; conclusions checked against saved outputs. The supplied third-party proposal is treated as a design suggestion, not authorization to override existing project requirements.
