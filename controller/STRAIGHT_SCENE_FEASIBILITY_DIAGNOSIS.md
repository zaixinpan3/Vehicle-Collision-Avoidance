# Why the straight scene fails

Historical version-13 diagnosis. On September 10, version 14 removed the
additional force-polygon constraints. The original executable diagnostic is
preserved in Git commit `278663c34a4250ad914f69c4d4ff71ba402c05b0` and archive
evidence EV-0095; it must be run against that source revision. See
[the removal experiment](../scripts/FORCE_CONSTRAINT_REMOVAL_RESULTS.md) for
current behavior.

## Material Passport

- Analysis date: September 10, 2026, America/Chicago.
- Mode: deterministic experiment validation and causal constraint diagnosis.
- Source: September 9 physical-scene MAT exports and captured hard-margin LP,
  originally tested at controller commit
  `339e1b9d3294dc44426b595d4859b2b36b20481a`.
- Verification: exact LP reconstruction, constraint-family removal,
  contradictory-row certificate, independent matrix-exponential calculations,
  diagnostic nominal LPs, and declared-model interface replays completed.
- Scope: explains the observed implementation failures; does not establish
  physical avoidance, change the online controller, or validate relaxed settings.
- Statistical inference: none. No population estimates, significance tests,
  causal claims from correlation, or stochastic success rates are reported.

The straight scene has multiple independent obstacles. The first-frame
failure is caused by robust combined-tire-force constraints. Its severity
combines conservative uncertainty propagation with a declared disturbance
set that is itself incompatible with the chosen force polygon over the full
open-loop horizon. Independently, the 1.6 s exit deadline excludes the
oncoming scene even without plant residuals. Once admission is bypassed for
diagnosis, changing road representations and premature target-free completion
block the scenario interface.

The required safety statement remains conditional: if the assumptions hold
and the first encounter problem is feasible, maintain feasibility and avoid
collision until finite target exit. Post-exit indefinite driving is not an
acceptance criterion. These results concern why the implementation cannot
start and execute the intended encounter; they are not a counterexample
starting from a successfully admitted physical state.

## 1. The actual first-frame contradiction is in tire-force rows

The diagnostic reconstructs the saved original LP using the public parser,
predictor and formulation. Its matrix and bound differences from the
solver-hook capture are both **exactly zero**. CLF performance optimization
is not involved: failure occurs in the preceding hard-margin LP.

| Constraint family | Family alone | All other families |
| --- | --- | --- |
| Combined tire force | Infeasible | Feasible |
| Model domain | Feasible | Infeasible |
| Tire slip | Feasible | Infeasible |
| Right road curb | Feasible | Infeasible |
| Left road curb | Feasible | Infeasible |
| Input, slew and performance-slack bounds | Feasible | Infeasible |

These are diagnostic removals, not candidate controllers. The original scene
contains no perceived target at time zero, so neither collision avoidance
rows nor terminal target-exit rows can cause that first rejection.

Rows **740 and 746** in the captured physical hard matrix are the opposing
front-axle lateral-force constraints at the final Bernstein control point
of the second certification cell, corresponding to 0.0285714286 s. Up to
trigonometric arithmetic, they require

\[
a^\top U\le-0.116980164704545,\qquad
-a^\top U\le-0.116980164704545.
\]

Their bound sum is **-0.233960329409091**. The maximum possible contribution
of the noncancelling coefficients over permitted inputs is only
**1.224646799147353e-16**. Thus the original inequalities are contradictory
within the input bounds; increasing solver iterations or changing CLF weights
cannot supply a feasible input. The discrepancy is not a solver tolerance.
This is a contradiction in sufficient robust constraints, not a claim that
an actual vehicle necessarily saturates at that time.

Source: `modifiedFialaTire.frictionCirclePolygonRows`,
`avoidanceSafetyGeometry.localProjectedRows`, and `solveHardCbfClf`.
The constraint-subset method is consistent with the diagnostic approach in
[MathWorks, Investigate Linear Infeasibilities](https://www.mathworks.com/help/optim/ug/investigate-linear-infeasibilities.html).
We do not claim to have enumerated every irreducible infeasible subset.

## 2. Two different sources of force uncertainty must be separated

At the straight cruise operating point, the six-state generator is constant
over all 16 prediction intervals (checked exactly). The declared independent
disturbance bound is

\[
|w(t)|\le d=[0.2,0.06,0.02,2.5,5,4]^\top.
\]

For zero initial error, the exact directional support of the LTI reachable
disturbance set in force direction \(q\) is

\[
\sigma_q(t)=\int_0^t |q e^{As}|d\,ds.
\]

This retains state correlations. By comparison, forming the tight marginal
state box first gives support
\(|q|\int_0^t|e^{As}|d\,ds\), which can be larger. Both expressions were
evaluated by numerical quadrature with absolute tolerance 1e-10 and relative
tolerance 1e-9; the formula is exact, the numerical values are approximations.

The production implementation adds further conservatism. Each small cell
feeds an independent radius box into the next cell. Its swept polynomial
uses absolute matrix powers even for dissipative diagonal terms. Although
`finitePredict` contains signed-generator calculations, the radius actually
used by safety rows is replaced with `tube.endRadius`; the generator state is
then reset to `diag(radius)`. Correlations are not preserved through the
operative full-horizon safety tube.

| Normalized force uncertainty at 1.6 s | Front axle | Rear axle |
| --- | ---: | ---: |
| Exact directional support | 0.762088 | 0.985869 |
| Tight marginal state-box support | 0.899240 | 1.059818 |
| Implemented endpoint-box support | 1.391581 | 1.639829 |
| Implemented swept Bernstein bound | 2.613730 | 3.079371 |
| Opposing polygon-row allowance | 0.965926 | 0.965926 |

At the identified 0.0285714 s front-axle row, the implemented swept support is
1.082904, whereas the exact directional support is approximately 0.438009.
The inflated bound explains why the current certificate fails so early.
This enclosure is conservative; this analysis does not establish it is
unsound or that its replacement can omit held-interval guarantees.

However, **better enclosure propagation alone does not solve the experiment**.
The rear-axle exact support crosses the polygon allowance at approximately
**0.1426280585 s**. At 1.6 s it exceeds the allowance by 0.0199434564.
An explicit admissible constant disturbance

\[
w=[0,0,0,0,-5,4]^\top
\]

already produces rear directional force error **0.9858511857** at 0.4 s,
computed directly using one augmented matrix exponential. Its negative is
also admissible. No nominal force translation can fit both opposite errors
inside a symmetric row allowance of 0.9659258263.

This incompatibility concerns the declared independent disturbance box,
the affine tire-force map, the inscribed 12-sided polygon, and the retained
open-loop input witness. It is not a proof that the nonlinear physical tire
must exceed its circular force limit. The polygon's pure lateral allowance
is `cos(pi/12)`, not 1. Nor does this prove that an uncertainty-dependent
feedback policy is infeasible: such a policy is absent from the current
open-loop certificate parameterization.

The empirical residual allowance was inherited from identification traces
used with the retired short-prefix design. It was not demonstrated compatible
with this complete-horizon formulation. Short-prefix historical successes
and the small-disturbance crossing fixture do not establish that compatibility.

## 3. A separate exit deadline makes first target admission impossible

For the direct-acquisition case, the target starts at `[29.9; 0.8]` m, moves
at `[-10; 0]` m/s, and must be outside the 30 m perception circle at the
fixed 1.6 s endpoint. The following checks set only the plant residual rate
to zero as a diagnostic; these settings were never used to claim physical
safety or installed as controller defaults.

| Zero-residual diagnostic | Result |
| --- | --- |
| Original target-free hard problem | Feasible |
| Acquired-target core constraints, without collision or exit | Feasible |
| Core plus collision constraints, without exit | Feasible |
| Core plus terminal exit, without collision | Infeasible |
| Complete acquired-target problem | Infeasible |

Four LP extrema under the nominal core constraints bound the ego endpoint by

\[
p_{E,x}\in[13.88916425,17.99999800],\qquad
p_{E,y}\in[-4.67090153,4.67090153]\ \mathrm m.
\]

The target endpoint is `[13.9; 0.8]` m. Every point in that enclosing rectangle
is within **6.83672050 m** of the target. This is already an upper bound using
independent coordinate extrema and ignoring collision constraints. It is far
below the 30 m exit radius (and the additional exit buffer). Changing only
the chosen terminal halfspace direction cannot fix this case.

The 1.6 s prediction length was inherited from an earlier lookahead setting.
Requiring complete encounter exit by that same endpoint creates a different,
much smaller admission domain. Longer prediction may be necessary here, but
simply increasing the horizon is not a validated remedy: the uncertainty
conflict and sensed-road coverage obligations must also be resolved.

## 4. Road observations violate the retained-certificate interface

`runCenterlineCruiseScenario` calls `fitPerceivedRoadBoundaries` every frame.
Even on the same physical straight road, its local origin and sampled fit
interval move with the ego. `localValidateStored` compares the entire
configuration/road/lane identity using `isequaln` and rejects any difference.

A zero-residual, target-free first admission was followed by its exact
predicted successor at 0.1 s. With the original road object, the next solve
succeeded (`checkedContinuationOptimization`). Refitting that same straight
road at the same successor moved the boundary origin from `[0;0]` to
approximately `[0.999951813;0]` m and produced
`collisionAvoidanceController:changedExecutionContract`.

This is an interface mismatch, independently reproduced without plant error.
It does not justify discarding the identity guard: new road data must be
represented or checked in a way that preserves the old certified physical
constraints and their coverage. Merely changing coordinates should not be
confused with a changed physical road.

## 5. Target-free operation ends before the original target arrives

The original scene begins with the target 100 m away. Yet the sole controller
starts a 1.6 s certificate even with no visible targets. A diagnostic using
zero residuals, unchanged road geometry and exact predicted successors
executed all 16 intervals, then returned an empty command and
`encounterComplete=true` at 1.6 s. The oncoming target was still
**68.00484040 m** from the ego, outside the 30 m sensor range.

The scenario stops on that empty command. It therefore cannot even reach
the first-detection event in this otherwise idealized replay. This is a
pre-encounter lifecycle defect in the current scenario/controller integration,
not a demand for post-exit indefinite safety. A single controller can have
well-defined target admission and episode bookkeeping without introducing
alternative maneuver laws; its target-free timer must not terminate the
requested experiment before the encounter starts.

## Implications and repair order

1. Make the declared disturbance set, tire-force constraints and finite
   feedback policy mutually certifiable. Preserve the held-interval proof;
   reducing empirical bounds without validation is not a repair.
2. Replace avoidable box/polynomial overestimation with a verified support
   propagation method. The constant-disturbance counterexample must remain
   a separate design obligation after this numerical improvement.
3. Choose a complete encounter horizon/domain that permits target exit and
   fits the available road and target prediction contracts. Retain recursive
   witness preservation after admission; ordinary receding-horizon feasibility
   cannot simply be assumed.
4. Reconcile changing road-coordinate representations with retained physical
   obligations, and correct target-free lifecycle handling in the same
   controller architecture.
5. Repeat direct first-detection physical validation before the full approach
   scenario and estimator integration. Optimization runtime is a subsequent
   validation obligation; speeding up an infeasible problem is insufficient.

No repair in this list has been implemented or validated by this analysis.
The current conditional theorem does not establish that the intended
experimental states lie in its certifiable domain. That practical admission
question is exactly where these experiments fail.

## Reproduction and validation

`scripts/analyzeStraightEncounterFeasibility.m` reads the original rerun
exports (including `captured-program.mat`) and writes MAT/JSON diagnostics
plus a standalone PDF/PNG uncertainty plot to an external output directory:

```matlab
addpath('scripts');
analysis = analyzeStraightEncounterFeasibility( ...
    '/path/to/2026-09-09-rerun/exports', '/path/to/analysis-output');
```

Its assertions verify exact original-LP reconstruction, the contradictory
rows including floating coefficient cancellation, the constant-disturbance
counterexample, nominal feasibility/exit separation, the road-update error,
and target-free premature completion. No physical plant is advanced by the
diagnostic. No full repository test suite or new high-fidelity experiment
was run for this analysis. Generated plots and raw matrices stay outside the
repository. Results and identified source exports are retained in the shared
external research archive.
