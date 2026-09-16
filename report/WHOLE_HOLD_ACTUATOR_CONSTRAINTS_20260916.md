# Whole-hold certificates with actuator-only operating constraints

Date: September 16, 2026. This change implements the user's instruction to remove temporal subdivisions and additional state/tire-slip limits. Predictive collision avoidance, the soft CLF, the finite continuation and the invariant terminal certificate remain. The target high-gain observer is unchanged.

## Implemented behavior

There is exactly **one certified interval per held input**. The 100 ms experiments no longer split each hold into seven intervals. The existing `prediction.cells` container now has one element per prediction stage. Bernstein coefficient positions are polynomial coordinates, not additional execution or sampling intervals.

Removed all online inequalities for station, lateral position, heading error, longitudinal speed, lateral velocity, yaw rate, and front/rear tire slip. Removed speed-domain rejection from the input parser and slip clipping from the admission-time rollout. Retired the unused `ltvBicycleModel.slipRows` constructor. The old range fields remain solely for explicitly labeled diagnostic/model-analysis utilities; changing them does not constrain the online solve. The `minimumCells` configuration field is removed.

The remaining hard conditions are actuator amplitude and configured finite slew limits, collision avoidance, confirmed finite encounter exit, and membership in the invariant terminal certificate. Only the CLF has a nonnegative slack with a squared penalty. The input cost still measures deviation from the certificate's operating point. No fallback input or prescribed maneuver is introduced. Stored controller state is format 29; earlier states must be reset.

The terminal set previously contained hidden state/slip restrictions as well. Its synthesis now uses four held-actuator amplitude rows and finite slew conditions, together with the same stable modal dynamics and sensing-error contract. A held input satisfies its amplitude constraints throughout the period without Bernstein subdivision. The modal terminal restriction is a prediction-side recursive-feasibility certificate, not a new prescribed trajectory or executed terminal policy.

## Whole-period continuous enclosure

For the declared affine hold, write `d = A*x0+B*u+c`. After truncating at order p, the tail satisfies

\[
 |R_p(t)|\le\frac{|A|^p\mathbf1\,\overline d}{(p+1)!}
 \frac{t^{p+1}}{1-\|A\|_\infty h/(p+2)},\quad 0\le t\le h,
\]

provided `p+2 > norm(A,inf)*h` and `d_bar >= norm(d,inf)`. Successive omitted factorial terms have ratios at most `norm(A,inf)*h/(p+2)`. This replaces the previous denominator `1-norm(A,inf)*h`, which required small intervals.

The polynomial order increases until the truncation support at the endpoint is at most 1e-11; order above 64 fails explicitly. Separate arithmetic reserves remain. State magnitudes for this calculation come from `abs(M)*inputLimit+abs(offset)` when the initial state is `M*U+offset`. They follow from the actual initial information set, affine dynamics and actuator box; they are not imposed state bounds. The analogous initial-error and disturbance expansions remain enclosed. Tests compare complete-hold enclosures with independent exact matrix-exponential flows, including disturbance sign switches and `norm(A,inf)*h > 1`.

Collision geometry now obtains heading, lateral and station ranges from these reachable tubes. If the heading enclosure spans a complete rotation, the ego rectangle's circumradius is a global support bound. Otherwise the existing affine support majorants cover the computed interval. All eight separating directions remain available, with one selected direction for each complete hold and target. The finite family therefore changes: for N holds and J targets its raw assignment count is `8^[J*(N+1)]`. Removing state bounds enlarges the admissible operating range, while one normal over a whole hold and larger footprint enclosures can add conservatism. This is not claimed to preserve exactly the former feasible family.

The finite exit chart covers the actuator-reachable final-state envelope. Measured curved-reference charts must remain invertible and future published Frenet measurement bounds must satisfy the stored sensing contract. That condition is distinct from an imposed lateral-position constraint. The successor proof retains its conditional model, sensing, execution and new-target-admission premises; see [the updated proof](../controller/TERMINAL_CBF_PROOF.md).

## Straight controller-only simulations

Three exact held-affine trials execute 300 holds, or 30 s, with reference 8 m/s, complete perception range 16 m, no road boundaries, zero measurement error, zero target jerk/yaw acceleration, steering bounds of 40 degrees and normalized longitudinal input in [-1,1]. Finite slew limits are not enabled in these three baseline scenarios. Seed: 20260912. The diagnostic solver budget is 30 s; real-time qualification remains 100 ms per frame.

| Scenario | Completed holds | Minimum sampled body gap (m; required 0.25) | Median / maximum frame (ms) | Frames over 100 ms | Final speed (m/s) |
| --- | --- | ---: | --- | ---: | ---: |
| Stationary | 300 / 300 | 1.326599 | 13.548 / 403.030 | 2 | 7.999999676 |
| Oncoming | 300 / 300 | 2.224310 | 12.925 / 308.863 | 1 | 7.999999676 |
| Crossing | 300 / 300 | 9.519723 | 12.889 / 85.054 | 0 | 7.999999676 |

All three pass the existing independent hard collision/terminal verification and sampled diagnostic checks, use no terminal/fallback commands, and recover negligible lateral and heading error by 30 s. The body-gap column includes the required 0.25 m; raw JSON stores the excess margin. The former model-domain margin is retained only as a diagnostic and removed from pass/fail acceptance.

The removed restrictions materially change behavior. Stationary avoidance reaches about 21.55 m lateral displacement and 2.01 rad heading error; longitudinal speed briefly falls to -3.07 m/s. Oncoming reaches about 8.23 m lateral displacement and 0.883 rad heading error; its minimum longitudinal speed is 4.84 m/s. These are actual outputs of the unrestricted affine-model optimization, not violations of a retained state bound. No new restriction is added to suppress them. A nonlinear physical-vehicle validity claim is not established by these affine simulations.

Stationary admission needs one integer call (20.812 ms integer time); its additional slow frame occurs at sample index 34, taking 311.136 ms. Oncoming admission at index 26 needs two integer calls totaling 114.403 ms. The large reduction from the previous 735--1681 ms active successor and multi-second admission measurements is material, but **the every-frame 100 ms requirement remains unmet**. Crossing's controller-only timings do not qualify an unmeasured joint estimator-controller pipeline. These diagnostic simulations apply computed commands at ideal hold timestamps and do not inject computation delay into plant motion.

Strict 100 ms reruns stop without a command for stationary admission at t=0 (246.822 ms, zero issued holds) and oncoming admission at t=2.6 s (141.927 ms, 26 preceding target-free holds issued). Their formulation work has already consumed the deadline before a native solve can begin. Crossing completes 300 holds with a maximum 87.154 ms. This is observed controller-only timing on the shared workstation, not a worst-case execution bound.

## Repeated frame comparison

The existing `profileFiniteBranchRuntime` driver was rerun with three admission repetitions and five regular repetitions. Each timing below excludes MATLAB profiling overhead. Captured native programs remove constant rows before counting:

| Frame | Variables | Native matrix rows | Median full call (ms) |
| --- | ---: | ---: | ---: |
| Fresh cruise | 33 | 86 | 15.513 |
| Cruise successor | 33 | 86 | 13.808 |
| Oncoming admission | 65 | 2253 | 312.715 |
| Stationary admission | 97 | 2681 | 330.972 |
| Oncoming active successor | 63 | 2197 | 48.139 |
| Stationary active successor | 95 | 2625 | 78.104 |

For comparison, the previous admission matrices had 33231 and 49631 rows. All 28672/43008 common state/slip rows are gone. Whole-hold adaptive polynomials retain collision rows, while actuator, exit and terminal conditions remain. The profiler's actual first active successor now fits below 100 ms in these repetitions, but that does not negate the slower admission and later stationary frame in the complete scenarios.

## Reproduction and artifacts

Rebuild the changed generated kernels, then run from the repository root:

```matlab
addpath('scripts');
buildBicycleNominalKernel();
buildAvoidanceGeometryKernel();
for scene = ["stationary","oncoming","crossing"]
    runExactStateRecursiveFeasibilityScenario(Scenario=scene,SampleCount=300, ...
        DeadlineSeconds=30,OutputDirectory=fullfile(output,"diagnostic-"+scene));
end
```

Original MAT/JSON outputs and validation results are under `/home/zai/.cache/collisionAvoidance/whole-hold-20260916`. Generated adapters and binaries remain in `solver/bicycle` and are excluded from the source commit. Algorithm sources and their build scripts are committed.

The first affected run passed 35 of 36 cases; an uncertain curved-reference continuation exposed a residual dependency on the former lateral domain in its sensing-radius initialization. After replacing that dependency with the current measurement-box geometry, all 17 recursive-closure cases passed. New behavioral tests verify invariance of the input plan to tiny diagnostic state/slip ranges, one whole interval per held command, unrestricted-heading footprint support, and exact-flow inclusion above the old small-interval threshold. Native/MATLAB parity covers the rebuilt flow and geometry kernels.

The complete 601-case suite initially passed 597, with four failures and no incomplete cases. All four failures asserted removed behavior: rejection beyond a lateral diagnostic range, rejection of a reverse-speed measurement, or rejection of a forward circular chart crossing the circle center. Updated behavior checks preserve rejection of an actually noninvertible measurement chart. All 65 cases in these four affected classes pass after updating expectations.

Final loop cleanup briefly omitted propagation of the carried Taylor endpoint to the next prediction stage. The oncoming regression caught this before release; propagation was restored and a new independent full-horizon matrix-exponential comparison was added. The final seven affected classes pass **53/53** in an independent `matlab -batch` process. That process was used after a MATLAB MCP request stopped responding and timed out; the unreturned request is not credited as a successful check. The merged final coverage comprises the retained full-suite results plus the updated affected classes, not a second full-suite invocation.

Final combined coverage passes **602/602** with no failed or incomplete cases. All 20 changed/new MATLAB files have zero factory `checkcode -id -config=factory` findings. Rebuilt geometry kernels pass their three parity cases. All six saved controller decisions reproduce within 1e-7 after cleanup, and an independent JSON audit checks all 900 accepted diagnostic holds. `git diff --check` passes. Detailed counts are saved in `final-verification.json`; combined and individual MATLAB result objects are retained separately.
