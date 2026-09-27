# Why the retained controller rejects the current vehicle experiments

Prepared September 27, 2026. Base commit: `2a956e00311e595a0382b017bcf06e87db77508e` with the recorded pre-existing workspace changes. No production source, constraints, configuration, or controller architecture was changed during this investigation.

Five failures from the [latest vehicle rerun](SINGLE_CONTROLLER_RERUN_20260927.md) were replayed through the public controller. Every previously issued actuator command matches exactly, and every failure identifier and timestamp is reproduced. The earlier [September 25 constraint audit](SOLVE_FAILURE_AUDIT_20260925.md) is confirmed with fresh snapshots of the current implementation; the circular model-domain failure and estimated-state failures receive additional quantitative analysis here.

The immediate failures have different causes. Removing road boundaries does not resolve the inconsistent straight/S-bend or estimated-state constraint subsets. A separate upstream problem is that accepted affine plans can rely on tire forces inconsistent with the controller's own nonlinear tire law. Consequently, relinearizing on the next measurement can destroy the apparent feasibility of the previous plan.

## 1. Direct causes, checked on the actual failed frames

| Frame | Direct failure | Decisive evidence |
| --- | --- | --- |
| Straight, 2.05 s | Bounded actuators cannot satisfy the selected fixed-direction collision certificates | Actuator/collision subset has a positive independent LP relaxation lower bound; terminal and road rows are unnecessary for the contradiction |
| S bend, 1.40 s | Intermediate scheduled station corridor conflicts with terminal recovery under the frozen dynamics and actuator limits | Actuator/station/terminal subset remains inconsistent without obstacle, road or exit constraints; actuator/terminal alone and actuator/collision alone are feasible |
| Circular, 1.50 s | Future anchor state leaves the Fiala slip domain before optimization | Predicted front slip is -1.664399 rad at stage 54, corresponding to 4.15 s; the permitted magnitude is below pi/2 |
| Estimated straight/circular, 1.05 s | The constructed uncertain-target collision requirements already exceed the selected model's reachable capability | Actuator/collision subsets have independent positive LP lower bounds of at least 43.514/56.634 m in separation-row relaxation; terminal-only subsets are feasible |

These are statements about the actual finite-dimensional models and selected separating directions. They do not establish unavoidable physical collision, infeasibility for every separating direction, or infeasibility of every physically valid trajectory.

## 2. Separating conflicting constraints from solver behavior

For each straight/S-bend frame, the previous-plan and selected fluid-reference directions were rebuilt using the existing conic assembler. The objective was set to zero and selected constraint groups were removed after assembly. Dynamics and the tightened actuator constraints remained. The following conclusions hold for both direction sets:

| Diagnostic modification | Straight | S bend |
| --- | --- | --- |
| Full constraints with zero objective | No feasible result | No feasible result |
| Remove road constraints | Still rejected | Still rejected |
| Remove collision constraints | Feasible | Conflict remains in the actuator/station/terminal subset |
| Remove terminal cones | Still rejected | Feasible witness |
| Remove only station-phase rows | Not applicable | Feasible witness |
| Keep actuator and collision constraints | Inconsistent | Feasible witness |
| Keep actuator and terminal constraints | Feasible witness | Feasible witness |
| Keep actuator, station phase and terminal constraints | Feasible witness | Inconsistent |

The S-bend native solver returns numerical/approximate statuses on some reduced problems. Those statuses alone are not used as proofs. Direct unscaled residuals verify the reported feasible witnesses, while the independent lower bounds below establish the conflicting subsets. In particular, the soft CLF's objective weight cannot fix a constraint contradiction that persists with a zero objective.

The complete investigation includes 72 zero-objective ablations: two direction sets, nine constraint combinations, and four rejected optimization frames (straight, S bend, estimated straight and estimated circular). The circular 1.50 s frame is excluded from this count because its model cannot be constructed.

### Straight: a quantified collision-certificate deficit

Introduce one nonnegative relaxation epsilon in the selected collision separation rows and minimize it, retaining dynamics and actuator bounds. The result is a diagnostic deficit in the certificate, not physical vehicle penetration and not permission to reduce clearance.

| Direction initialization | Minimum SOCP relaxation (m) | Independent necessary LP lower bound (m) | Active LP collision stages |
| --- | ---: | ---: | --- |
| Previous plan | 0.925240346 | 0.924389577 | 9, 10 |
| Selected fluid reference | 2.770192192 | 2.652059428 | 5, 15, 26 |

For the previous-plan directions, stages 9 and 10 correspond to absolute prediction times 2.50 and 2.55 s. The LP is a necessary relaxation of the cones, so its feasible set contains that of the original subset. Its strictly positive lower bound rules out zero-relaxation feasibility within the recorded floating-point residuals. Primal violations and dual stationarity residuals are below 4.1e-14, with matching primal/dual bounds. This goes beyond interpreting a Clarabel rejection code.

### S bend: tracking the path is also being constrained to track a clock

The scheduled model requires `abs(s_k - sReference_k) <= 2 m` throughout the horizon. Its terminal set also uses the scheduled reference. Avoidance therefore has to respect an intermediate longitudinal progress schedule in addition to eventual path and speed recovery.

For the actuator/station/terminal subset, the necessary terminal-box LP requires at least **0.658082710 m** of uniform station-corridor expansion. Its primal violation is 2.73e-12 and dual stationarity residual 8.53e-14; the primal/dual bounds agree to about 5e-13. Independent MATLAB `coneprog`, retaining the modal disks, estimates the subset expansion as **0.854530136 m**, with unscaled residual 3.04e-7.

This is not a proposal to simply enlarge 2 m to 2.855 m. That number solves only a diagnostic subset and does not restore the full physical avoidance problem or justify the existing terminal proof. The conflict also is not necessarily a simple inability to make enough forward progress: in the subset compromise an upper station constraint is active. The substantive issue is compatibility of the frozen dynamics, scheduled intermediate progress and scheduled terminal recovery.

## 3. Upstream defect: tire tangents permit physically inconsistent plans

`modifiedFialaTire.affineModel` takes a joint derivative with respect to slip and signed longitudinal tire-force utilization beta. The near-endpoint beta clamp avoids a singular derivative evaluation; it does not make a tangent globally valid. On a saturated slip branch, for fixed axle force scale C and fixed slip sign:

\[
F_y=-C\sqrt{1-\beta^2}\,\mathrm{sign}(\alpha),\qquad
\frac{\partial F_y}{\partial\beta}
=\frac{C\beta}{\sqrt{1-\beta^2}}\,\mathrm{sign}(\alpha).
\]

As the magnitude of beta approaches one, the remaining lateral capacity vanishes while the derivative grows. The saturated slip derivative is zero; a large candidate steering change can cross to the opposite force-sign branch without that frozen tangent representing it.

Fresh audit of the last accepted plans reproduces:

| Front-axle quantity | Straight plan at 2.00 s, first hold | S-bend plan at 1.35 s, hold 30 |
| --- | ---: | ---: |
| Anchor beta | 0.999929413 | 0.999983961 |
| Candidate beta | 0.923738750 | 0.976272069 |
| Beta derivative (N per unit beta) | +547478 | -1148576 |
| Anchor steering (rad) | -0.408070 | +0.407383 |
| Candidate steering (rad) | +0.698122 | +0.698122 |
| Affine front force (kN) | **-41.790006** | **+27.271762** |
| Nonlinear Fiala force at the same candidate point (kN) | **+2.491670** | **+1.408701** |
| Candidate combined-slip capacity (kN) | 2.491670 | 1.408701 |

The anchor-to-candidate steering difference is within one optimization model; it is not claimed to be the change between two consecutive issued commands. The nonlinear forces above are from the controller's own tire law, not measurements of the independent PassVeh14DOF tire forces. The straight case already affects the first hold and reverses the predicted force sign.

The old accepted input sequence was also replayed in the controller's nonlinear bicycle model as an offline diagnostic:

| Prior plan | Affine final lateral offset (m) | Nonlinear final lateral offset (m) | Affine final speed (m/s) | Nonlinear final speed (m/s) |
| --- | ---: | ---: | ---: | ---: |
| Straight, 2.00 s | 0.000459 | 8.438515 | 10.000088 | 8.472141 |
| S bend, 1.35 s | -0.071705 | 7.178738 | 10.007084 | 3.627107 |

These are open-loop numerical replays of stored plans, not newly executed vehicle experiments or continuous-time certificates. The failed frame's lane, configuration and acceleration bias were used with the previous predicted initial state. The direct force comparison does not depend on that rollout-context assumption.

The evidence supports a causal explanation: earlier affine plans can promise avoidance and recovery using unattainable or oppositely directed tire forces. On the next measurement, trajectory relinearization builds a new problem whose feasible set need not contain the old plan. It does not quantify the contribution of every nonlinear effect to every previous decision.

### Why the previous feasible plan does not guarantee the next frame

`hardEncounterBarrier.prepare` refreshes the trajectory model during an active encounter and excludes that refresh from the unchanged-model witness-transfer branch. `collisionAvoidanceController` explicitly sets `recursiveFeasibilityGuaranteed=false` for these fresh trajectory models. The previous plan supplies an initialization, not an inherited feasibility proof for the newly linearized system.

The declared zero-residual held-affine plant assumptions also do not cover arbitrary nonlinear tire extrapolation or the independent PassVeh14DOF plant. A certificate for the frozen affine problem does not, by itself, bound this nonlinear model mismatch.

## 4. Circular failure is a future modeling failure, not current vehicle slip outside the domain

The exact replay identifies:

- Frame time: 1.50 s; previous accepted plan time: 1.45 s.
- First invalid anchor: prediction stage 54 of 64, at absolute time **4.15 s**.
- Front/rear slips there: **-1.664399 / -1.276402 rad**. The front magnitude exceeds pi/2 by 0.093602 rad (about 5.36 degrees).
- Candidate steering/utilization: 0.698122 rad and 0.999985441.
- Predicted longitudinal/lateral speeds: 3.380415 / -8.032638 m/s; yaw rate 2.072015 rad/s.
- The actual current measured state evaluated with the held input has front/rear slips **+0.749050 / -0.007088 rad**, both within the tire domain.

The saved state at the thrown tire error matches that future anchor exactly. No optimization result is produced for the failed frame. Consequently, raising conic solver iteration limits or removing road rows cannot repair this particular exception: a valid prediction anchor must exist before constraints can be solved.

## 5. Estimated-state admission is blocked by the constructed uncertainty envelopes

The first target observation occurs at 1.05 s. The exact replay preserves the estimator's published uncertainty, rather than replacing it with truth:

| Quantity | Estimated straight | Estimated circular |
| --- | ---: | ---: |
| Target speed interval (m/s) | [2.869824, 19.347320] | [2.076372, 20.476583] |
| Course interval width (rad) | 2.481185 | 2.607358 |
| Position-box half-width vector norm now (m) | 1.380894 | 1.415819 |
| Position-box half-width vector norm after 1 s (m) | **45.583511** | **47.584468** |
| Position-box half-width vector norm after 2 s (m) | 92.893274 | 96.730494 |

These are the enclosures used by the controller, not measured target-position errors, exact reachable-set radii or inevitable displacements. Both the input uncertainty and conservatism of its propagated enclosing box matter.

Removing only collision rows still results in rejection; target-exit constraints remain, and that ablation alone does not isolate which of the remaining requirements is inconsistent. Removing only road rows or only terminal cones also does not yield feasibility. Actuator/terminal-only subsets are feasible, but actuator/collision-only subsets already require large relaxations:

| Frame / directions | SOCP separation-row relaxation (m) | Independent LP lower bound (m) |
| --- | ---: | ---: |
| Estimated straight / previous | 44.137632 | 43.514169 |
| Estimated straight / fluid | 44.937886 | 44.417405 |
| Estimated circular / previous | 57.155016 | 56.634278 |
| Estimated circular / fluid | 58.542538 | 58.089060 |

The worst LP primal residual across these four checks is 6.45e-9 and worst dual stationarity residual 6.61e-10. SOCP residuals are below 2.8e-11. The positive lower bounds confirm inconsistency of the tested conservative subsets; they do not show that the real target will collide. A larger optimizer budget or eliminating road boundaries does not resolve these particular admission conflicts.

## 6. Repair priorities consistent with one real-time controller

1. **Make tire prediction and candidate input changes consistent.** Address the near-saturation joint beta tangent and force-sign changes before treating affine feasibility as useful physical feasibility. Any local approximation must control its validity region and mismatch, or use a force-coordinate formulation with justified actuator realizability. A friction cone appended blindly to the existing saturated tangent is not a complete repair: a tangent line to the normalized friction disk intersects that disk only at its tangent point when the slip derivative is zero.
2. **Reconcile progress constraints with avoidance/recovery intent.** Determine which station restriction is required by the spatial reference model and its terminal argument. If avoidance is intended to allow longitudinal delay, the progress/terminal formulation must reflect that. Merely removing a proof assumption or increasing the 2 m number is not a validated redesign.
3. **Maintain valid prediction anchors.** A shifted plan must remain inside tire and coordinate domains before it can initialize the next convex problem. Anchor repair or rejection must be compatible with actuator limits and the sole controller's acceptance contract; an invalid prior anchor should not be passed directly into differentiation.
4. **Fix uncertain-target admission at its source.** Evaluate initial observability, justified speed/course priors, earlier sensing and tighter sound target-set propagation. Do not erase published uncertainty to make an experiment pass. The large current envelopes explain why truth-fed and estimated trials fail differently.

This investigation does not reintroduce nonlinear shooting, a new controller mode, alternative execution controllers, or a softened production safety constraint. It identifies changes that need their own implementation and validation. The S-bend's second-scale runtime is a separate measured problem; the positive feasibility lower bounds demonstrate why speeding up the present conflicting programs alone would not make them executable.

## 7. Reproduction and evidence

Original scenario inputs: `/home/zai/.cache/collisionAvoidance/single-controller-rerun-20260927-031629`.

Current diagnosis directory: `/home/zai/.cache/collisionAvoidance/failure-analysis-20260927-034213`.

The external wrappers `runDiagnosis.m`, `runMoreDiagnosis.m` and `runEstimatedBounds.m` run through `matlab -batch`. Conditional tracers always return false, preserve source files and never pause a headless MATLAB session. The first wrapper's character/string filename concatenation error was corrected; its log is retained. All three final diagnostic processes exit successfully. No diagnostic candidate is issued to a plant.

The existing `scripts/diagnoseTrajectorySolveFailures.m` supplies the exact-state ablations, independent bounds and model replay. External copies of its diagnostic helpers add the two estimated cases; they are analysis utilities, not controller implementations. Embedded assertions validate exact command reproduction, failure timestamps, directly evaluated cone residuals and LP primal/dual residuals. Five public-input replays, 72 feasibility ablations, six collision-relaxation SOCP/LP pairs, one station-relaxation SOCP/LP pair and two prior-plan rollouts were completed. No new production unit-test run is claimed.

[Compact evidence](CURRENT_OPTIMIZATION_FAILURE_ANALYSIS_20260927/) includes replay/error stacks, the circular failing anchor, uncertainty intervals, constraint ablations, separation/phase lower bounds and tire/rollout comparisons. The raw MAT programs, primal/dual witnesses, executed source archive and diagnostic wrappers stay outside the repository. Source hashes were checked unchanged; original inputs, outputs and scripts have SHA-256 entries in the artifact manifest. Unrelated pending source changes, dependency trees and generated binaries are excluded from this report commit.
