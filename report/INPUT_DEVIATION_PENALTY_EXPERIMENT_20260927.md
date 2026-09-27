# Fixed-linearization input-deviation penalty experiment

Prepared September 27, 2026. Base commit: `039ee609caa4fc1712257a0981d153720a40ec94`, with the previously recorded pending workspace changes.

**Result: substantial early improvement, but no complete avoidance/recovery.** Penalizing departure from each stage's actual linearization input reduces the initial large input jumps and tire-model disagreement. Both tested positive weights extend execution beyond the original failure times in all three exact-state vehicle scenes. All nine closed loops nevertheless fail before 18 s; a large weight still permits a severe late S-bend input jump and tire-force error. The experiment does not establish a real-time controller or a reliable default tuning.

## Objective and implementation

The added cost is

\[
J_{\Delta u}=\lambda\sum_{k=0}^{N-1}
\left[\left(\frac{\delta_k-\bar\delta_k}{\delta_{\max}}\right)^2
+\left(\frac{\beta_k-\bar\beta_k}{\beta_{\max,abs}}\right)^2\right].
\]

The center is read from each stage's `tireModels{k}.operatingInput`, including the existing endpoint clamp used by the actual tire tangent. It is captured in `formulateAvoidanceProblem` and retained when the solver changes its initialization or the sparse state-coordinate origin. The optimized nominal inputs are penalized, not future random feedback corrections. This is a fixed model-input deviation penalty, not an absolute-input or adjacent-hold slew penalty.

`jointCertificate.inputDeviationWeight` supplies lambda. Both condensed and lifted objectives add `2*diag(w)` and `-2*w.*barU`; only an input-independent constant is omitted. Normalization uses the configured actuator magnitudes. A fixed-zero actuator uses unit scale to avoid division by zero, and numeric weights are converted to double for objective assembly. The existing small solver-seed proximal term, CLF/LQR gains, initializer metric, physical input bounds, collision/road constraints, terminal constraints, feedback prediction and controller architecture are unchanged. The existing benchmark exporter now preserves the two new objective fields; no additional controller method was introduced.

The default remains **0**. We retain the implemented term and reproducible positive-weight trials without promoting an unsuccessful tuning to the default. Mathematical detail is in [the objective document](../controller/QUADRATIC_CLF_INPUT_OBJECTIVE.md).

## Closed-loop protocol

MATLAB 26.1.0.3276743 (R2026a) Update 3, independent Vehicle Dynamics Blockset PassVeh14DOF plant, exact controller states, 50 ms control period, 10 m/s ego/target speeds, requested duration 18 s and final 2 s recovery window. Vehicle rectangles are 5 by 2 m. The existing straight, radius-100-m circular and sinusoidal-curvature S-bend scenarios retain their 50 m initial encounter distance, 30 m sensing range, 1.5 m collision lateral offset, road boundaries and shoulder settings. The S bend retains maximum curvature 0.01/m and wavelength 80 m. These exact-state cases use no randomized perception; estimated-state uncertainty cases were not rerun.

Weights 0, 10 and 1000 were run serially with one MATLAB computational thread. Each physical scenario resets its controller state. The solver search budget remains 5 s, complete-frame deadline remains infinite, and no computational delay is injected into the plant. A separately recorded offline S-bend preparation call took 5.654 s with a 30 s diagnostic budget; it issues no plant command and is excluded from closed-loop results. Preparation prevents cold compilation from confounding the objective comparison.

An initial 4 s pilot reproduced the straight/circular baseline failures. Its cold S-bend first frame exhausted the 5 s work budget at t=0; an empty-command summary bug then stopped the pilot harness. Empty results are now handled and the logs/MAT files are retained. The formal campaign uses the original 18 s settings for every case. All three formal zero-weight configurations, complete state histories and issued commands match the previous vehicle rerun exactly.

## Outcomes

Every row stops at its reported failure; no collision-free continuation is inferred afterward. The sole tire-domain failure is the unregularized circular case. All other rows report `collisionAvoidanceController:optimizationFailed`.

| Scene | Weight | Failure time (s) | Max issued steering (deg) | Max issued absolute beta | Minimum sampled SAT gap (m) |
| --- | ---: | ---: | ---: | ---: | ---: |
| straight | 0 | 2.05 | 39.999 | 0.923739 | 4.019774 |
| straight | 10 | 2.40 | 9.012 | 0.526628 | 0.113203 |
| straight | 1000 | 2.55 | 4.264 | 0.116825 | 0.108666 |
| circular | 0 | 1.50 | 39.999 | 0.999988 | 15.360714 |
| circular | 10 | 2.45 | 13.249 | 0.999988 | 0.209581 |
| circular | 1000 | 2.40 | 5.529 | 0.175217 | 0.223646 |
| sCurve | 0 | 1.40 | 39.999 | 0.999988 | 17.181475 |
| sCurve | 10 | 2.35 | 9.460 | 0.437085 | 0.261758 |
| sCurve | 1000 | 2.40 | 39.999 | 0.999988 | 0.221995 |

The straight weight-1000 run meets the scenario's passed-target predicate, but fails at 2.55 s before recovery. All other cases fail before that predicate. None completes the requested horizon or recovers cruise. Positive sampled separation up to stopping is not an intersample safety proof. In particular, smaller minimum gaps in longer runs should not be interpreted as a matched-time safety regression.

## What improved, and what remained wrong

All nine recorded observation sequences were replayed through the public controller. Maximum issued-command difference is exactly zero, and every failure time/identifier matches. The audit compares each accepted affine tire force with the controller's nonlinear Fiala law at the **same candidate state and input**; it does not measure the independent plant's tire force.

To avoid comparing different observation lengths, the following maxima cover target-visible accepted frames only through each baseline's last accepted time: 2.00 s straight, 1.45 s circular and 1.35 s S bend. Normalized deviation is the maximum over both input coordinates and all planned stages. Force mismatch is the maximum over axles and all planned stages in that common window.

| Scene | Weight | Maximum normalized input deviation | Maximum model force mismatch (kN) |
| --- | ---: | ---: | ---: |
| straight | 0 | 1.999971 | 64.072 |
| straight | 10 | 0.104760 | 2.315 |
| straight | 1000 | 0.106592 | 4.282 |
| circular | 0 | 1.999972 | 539.534 |
| circular | 10 | 0.166846 | 2.194 |
| circular | 1000 | 0.164723 | 4.241 |
| sCurve | 0 | 1.999972 | 56.795 |
| sCurve | 10 | 0.186733 | 3.020 |
| sCurve | 1000 | 0.186258 | 6.203 |

This supports the proposed mechanism: the term suppresses the early jumps away from the actual linearization inputs and reduces early tire-tangent extrapolation. It does not make the remaining approximation error small enough to certify the nonlinear plant.

Later failures expose the limitation of a soft penalty. At S-bend time **2.30 s**, weight 1000 still accepts a first steering change from **+0.026804 to -0.698122 rad** and beta from **0.162380 to 0.999988** relative to that first stage's linearization point. The first-hold axle-force mismatch reaches **84.386 kN**; the whole-plan maximum across that run reaches **138.004 kN**. A weight of 1000 can still be outweighed by the predicted state/slack objective. It is not a hard trust region or a force-consistency constraint. The candidate's beta near one also collapses the nonlinear lateral-force capacity, which an unconstrained joint tangent can violate.

Across full observed runs, maximum force-capacity excess remains 3.546 kN for straight weight 1000, 16.174 kN for circular weight 1000, and 138.004 kN for S-bend weight 1000. Weight 10 gives later circular failure than weight 1000 and avoids the latter's extreme late S-bend jump; there is no monotonic closed-loop benefit from increasing the weight. No candidate tire-slip evaluation in the accepted-plan audit exceeds the Fiala slip domain. That does not contradict the baseline circular failure: it occurs during a subsequent nonlinear anchor rollout before an optimization result exists.

A cost-only change leaves the feasible set of any already frozen model unchanged. Earlier plans can change the future states and models, which explains the delayed failures; it cannot directly resolve a contradictory constraint set at a fixed frame. The new weighted failed frames have not received independent infeasibility lower bounds, so their rejection is not claimed to identify a specific contradictory subset. The earlier [failure analysis](CURRENT_OPTIMIZATION_FAILURE_ANALYSIS_20260927.md) remains evidence about the earlier frames only.

## Runtime and validation

Matched target-visible controller-call medians (baseline / weight 10 / weight 1000) are 46.20 / 43.98 / 43.33 ms straight, 52.44 / 47.04 / 45.92 ms circular, and 207.93 / 181.29 / 117.37 ms S bend. These are observational single-run timings, not an isolated benchmark; another MATLAB workload was observed during the late campaign/audit. The S-bend pre-visibility median remains 2.460 / 2.495 / 2.455 **seconds**. A lower whole-run median partly reflects more later, faster encounter frames and must not be described as elimination of the expensive pre-visibility computation. Every S-bend complete frame exceeds 50 ms at every tested weight. No real-time qualification is achieved. `summary.csv` uses MATLAB `prctile`; the separate phase table uses NumPy's default linear percentile convention.

Validation completed:

- **176/176 targeted MATLAB tests passed**, covering fixed-center objective behavior, a same-feasible-problem reduction in input deviation, nonzero centers, invalid/numeric-type weights, condensed/lifted cost equivalence after recentering, trajectory/continuation behavior and exported-frame operation at zero/nonzero weights.
- The first expanded test pass found three missing-field errors in the benchmark exporter. Its field allowlist was corrected and all tests reran successfully. The native generated library itself was not rebuilt; exported-frame tests exercise the MATLAB adapter and existing native conic solver.
- All nine public-input replays reproduce every command exactly, including failure time and identifier. A separate Python rectangle-SAT computation agrees with every minimum gap to below 1e-8 m and independently confirms noncompletion/nonrecovery.
- Factory Code Analyzer checked ten changed MATLAB files. Nine have no findings; `avoidanceStageQp.m` retains two array-growth and one sparse-indexing advisories on unchanged lines. Initial analyzer settings referenced a missing older MATLAB settings file; the final run explicitly uses factory settings.
- Final zero-bound/numeric-type normalization guards and the benchmark export correction were added after the vehicle campaign. The tested vehicle actuator bounds are positive doubles, so these guards leave its objective coefficients unchanged. The final test suite covers the resulting source.

## Reproduction and artifacts

Run from the repository root after adding `scripts`, `controller` and `config` to the path:

```matlab
runInputDeviationPenaltyExperiment(outputDirectory, [0,10,1000], 18);
auditInputDeviationPenaltyExperiment(outputDirectory);
```

The formal wrapper also performs the documented offline S-bend preparation, using the saved pilot first-frame inputs. That wrapper and all supporting inputs are retained. A fresh cold session without preparation can instead encounter the separately documented startup budget failure. Use a new output directory: the experiment refuses to overwrite an existing trial.

Compact numerical evidence is in [INPUT_DEVIATION_PENALTY_EXPERIMENT_20260927](INPUT_DEVIATION_PENALTY_EXPERIMENT_20260927/). Original MAT results, last accepted plans, per-frame traces/audits, Python checks, PNG/PDF plots, logs and a final source snapshot are retained externally at:

`/home/zai/.cache/collisionAvoidance/input-deviation-20260927/`

`final-source-manifest.json` identifies the final workspace snapshot, not a claim that every file was executed. It includes pre-existing pending source changes, deliberately excluded from this task's commit. `native-manifest.json` identifies existing MEX dependencies; dependencies and generated binaries are not committed. `raw-artifact-manifest.json` identifies the frozen technical result bundle. Weekly sources/PDFs are excluded from all evidence hashing.
