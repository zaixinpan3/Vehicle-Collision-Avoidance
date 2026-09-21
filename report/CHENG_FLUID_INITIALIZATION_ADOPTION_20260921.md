# Adopt Cheng fluid-reference initialization

Date: September 21, 2026. This implementation replaces the former scalar
initialization in the production controller. The fixed-direction full SOCP,
original hard verifier and certificate continuation rules remain unchanged.

## Implemented behavior

`solveHardCbfClf.fluidInitialize` constructs two opposite-side Gaussian references from
the most critical predicted target conflict, fits both references to the affine
vehicle prediction while preserving the final state and input, and computes
analytic rectangle separation normals. The smaller worst reserved support
residual selects one seed without trial trajectory solves. `fixedDirections` holds its normals
constant while optimizing every control input. The generated prepared-frame
adapter calls the same initializer.

The construction follows the modified-function extremum rule in Cheng et al.
(2021), DOI 10.1109/TITS.2020.2990211, Eqs. (34)--(36). Curved Frenet geometry,
predicted moving-target conflict placement and model fitting are project
extensions; the paper itself leaves moving obstacles to future work. See the
[original-paper review and offline study](CHENG_FLUID_INITIALIZATION_REVIEW_20260921.md)
and [current mathematical specification](../controller/JOINT_SUPPORT_CERTIFICATES.md).

The online policy uses one width and compares the two sides using a shared
matrix factorization, with no trial SOCPs. Default
`admission` fields are:

| Field | Default | Meaning |
| --- | ---: | --- |
| `widthScale` | 1.2 | Multiplier on conflict half-length or the minimum base width |
| `minimumWidthMeters` | 3.0 m | Minimum base Gaussian length |
| `clearanceAllowanceMeters` | 0.2 m | Nominal reference-shape allowance only |
| `headingWeight` | 4.0 | Heading weight in the lateral/heading fit |
| `regularizationWeight` | 0.02 | Fit input-effort and input-difference weight |

The amplitude uses ego half-width plus target footprint support at the middle
conflict node and the seed allowance. Relative lateral motion selects the
opposing side first; a nearly stationary lateral tie orders away from target
center, then positive road lateral if centered. Full predicted support
residuals choose between the two fitted sides. This can select passage ahead
of a later crossing target instead of always passing behind. Only the selected
seed enters the SOCP; no width or amplitude grid is searched. There is no
guarantee that this single fixed-normal problem succeeds whenever another
trajectory would be feasible. Multiple
obstacles all retain hard constraints, although only the most critical nominal
conflict shapes the reference.

A seed is allowed to violate collision, actuator or chart constraints. It
cannot be issued or retained as a fallback. A fresh plan requires one successful
full solve followed by the original physical-row, support, terminal and CLF
checks. An inherited frame uses the prior optimized suffix and its directions;
it does not rerun fluid initialization. The only retained fallback is that
independently verified optimized suffix, subject to the complete frame deadline.

## Removed implementation

Deleted the signed control deformation, amplitude-cell partition, discrete
normal dictionary, interval subtraction, scalar objective bisection and
least-violated cell-boundary proposal. Removed their helper functions and
configuration fields (`normalCount`, `amplitudeCells`, `performanceIterations`,
`temporalShoulderFraction`); supplying them now raises an unknown-configuration
error. There is no hidden legacy initializer or selector switch.

Removed the obsolete scalar-only experiment drivers
`analyzeCircularCrossingAdmission.m`, `runTaperedAdmissionValidation.m` and
`analyzeTaperedAdmissionValidation.py`, and replaced scalar-helper tests with
fluid and command-acceptance coverage. Dated reports remain historical evidence;
the deleted scripts are available in the preceding Git history, not in the
current execution path. Updated fixed-direction validation, native replay,
profiling probes and current controller documentation to the new interface.
The controller remains within the existing 20-source budget.

## Development findings

The first generalization of the offline prototype added a pose-chart remainder
to the nominal excursion amplitude. That changed the seed enough to reject
both tighter mirrored curves at curvature magnitude 0.012/m. Nominal path
construction now uses nominal footprint projection at the middle conflict
node. All uncertainty, chart and clearance terms remain enforced in the full
SOCP and original verifier; no physical acceptance margin was reduced.

A first one-side policy passed the six base fixtures but only 18 of 30 broader
variation fixtures. A two-side offline probe found that choosing the smaller
maximum support residual recovers every available side at width 1.2, while
both seeds still fail at curvature magnitude 0.015/m. Increasing the fit
regularization from 0.01 to 0.02 reduces excessive intermediate chart excursions
and recovers these two tighter cases. The production implementation shares the
fit factorization across the two sides and retains one full SOCP. All 30
variation fixtures then pass, including 28 with active target constraints and
two target-out-of-range nominal cases. These cases informed the heuristic;
they are not an independent held-out validation set. All failed probes remain
in the external raw-artifact directory.

The first corrected focused run passed 61 tests covering six mirrored curved
crossings, straight stationary admission, two sample periods, solver failure,
unsafe positive solver status, inherited failure recovery, expired budgets,
exported-frame parity and rejection of retired configuration fields.

## Validation scope

Detailed final regression, closed-loop, warmed timing and independent footprint
results are recorded below and in the matching JSON. Generated binaries and raw
MATLAB artifacts remain outside the repository. Warmup is excluded from timing;
first encounter admission in each measured run remains included.

## Final checks and measurements

The final complete MATLAB suite passes **776/776 tests**, with no failed or
incomplete tests (478.853 s aggregate test duration). The changed count reflects
removal of obsolete scalar-helper tests and addition of fluid-reference cases.
The first implementation passed 772 tests; the final suite includes four more
late-crossing/faster-ego/tighter-curve cases. MATLAB MCP's 300 s waiting limit
expired during each complete suite, but MATLAB continued and saved the final
result objects and JSON; the saved results were independently loaded and checked.

The generated prepared-frame MEX passes original hard verification for admission
and continuation. Its complete admission decision differs from MATLAB by at
most `6.399e-12`; the continuation decision difference is zero. MATLAB Coder
required explicit scalar indexing/reductions and a variable-size point annotation.
These compilation fixes do not change the mathematical algorithm. Factory Code
Analyzer reports one pre-existing `FNDSB` sparse-index performance suggestion
in the unchanged solver reduction; all other checked changed MATLAB files have
no messages. Python compilation, current profiling-probe generation and Git
whitespace checks pass. No optional safety test was weakened to obtain a pass.

Timing uses MATLAB R2026a Update 3, one compute thread, after regression and
native compilation finish. It excludes 20 replay warmup calls and 1,680 campaign
warmup holds. No other task-controlled MATLAB work runs during measurement;
this is still a desktop measurement, not an isolated real-time platform.
Fresh target admission remains measured. No warmed outlier is discarded or
reclassified as startup. The observations do not establish their host-level
cause or a speed improvement over the preceding initializer.

| Warm saved-input replay | Calls | Median [ms] | Maximum [ms] | Above 50 ms |
| --- | ---: | ---: | ---: | ---: |
| Straight stationary admission | 22 | 76.563 | 122.818 | 11 |
| Circular crossing admission | 22 | 47.994 | 123.733 | 6 |

Both rounds are retained. Straight replay round medians are 44.584 and
110.264 ms; circular medians are 50.568 and 47.058 ms. This variation is not
averaged away or attributed to an unmeasured cause. Every replay produces the
same complete decision as its repeated calls, holds normals fixed and uses
one trajectory solve. The maximum replay frames spend 108.696 ms (straight)
and 106.133 ms (circular) in the aggregate formulation/initialization stage.

Six closed-loop cases combine straight/0.01-per-meter circular roads with
stationary, oncoming and crossing targets. Each is repeated twice with a
30 s diagnostic deadline and twice with a strict 50 ms deadline, requesting
600 holds per trial. Reference speed is 8 m/s, sample time 50 ms, performance
horizon 1.6 s, zero model residual, exact sensing and seed 20260912. No physical
road boundaries are enabled. Existing uncertainty and physical constraints
remain in the controller, although these trials use zero measurement errors.

| Warm closed-loop mode | Completed trials | Executed holds | Timed attempts | Median [ms] | Maximum [ms] | Above 50 ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Diagnostic | 12 / 12 | 7,200 | 7,200 | 7.055 | 196.427 | 14 |
| Strict 50 ms | 7 / 12 | 4,200 | 4,205 | 6.997 | 144.187 | 5 |

All diagnostic trials complete, with every executed hold hard-certified and
no sampled collision. Five strict trials reject their first frame and execute
no hold: one straight stationary trial, both circular stationary trials and
both circular crossing trials. The other seven complete 600 holds each.
Rejected attempts are retained in counts and timings. This change therefore
**does not meet the all-frames-within-50-ms criterion**. In the preceding
fixed-direction campaign all 14,400 measured frames completed below 50 ms,
while its separate admission replay also had outliers. The new measurements
must not inherit that earlier campaign's timing conclusion.

An independent SAT implementation reconstructs the exact held affine plant,
checks endpoint agreement (maximum error zero here), samples each hold every
5 ms and refines critical holds every 0.1 ms. One diagnostic repetition per
case is audited separately from timing:

| Scenario | Curvature [1/m] | Refined minimum body gap [m] | Refined SAT gap [m] |
| --- | ---: | ---: | ---: |
| Stationary | 0 | 0.000092736 | 0.000092736 |
| Oncoming | 0 | 0.051102451 | 0.051102451 |
| Crossing | 0 | 9.519482818 | 7.851234387 |
| Stationary | 0.01 | 0.099338471 | 0.099338471 |
| Oncoming | 0.01 | 0.131488684 | 0.131480710 |
| Crossing | 0.01 | 0.149474601 | 0.141572807 |

The straight-crossing nominal counterfactual never threatens collision and
is not evidence of an avoidance intervention. The other five nominal cases
have overlap. All six controlled finite audits have positive separation, but
the straight stationary minimum is only **0.0927 mm**: this is a very small
nominal margin, not evidence of physical robustness. The preceding initializer's
corresponding refined gap was 0.009805352 m, and its circular-crossing gap was
0.323076195 m. The new initializer changes the fixed-normal inner problem and
therefore the resulting clearances, even though acceptance constraints are
unchanged. Neither the node certificate nor finite refinement proves separation
at every continuous time or on a nonlinear vehicle.

## Reproduction and artifacts

From the repository root, the complete regression can be reproduced with:

```bash
matlab -batch "results = runtests('tests'); assertSuccess(results)"
```

The executed regression used the MATLAB MCP interface. Warm validation used a
separate `matlab -singleCompThread` process and these repository calls:

```matlab
addpath('scripts');
runFixedDirectionValidation(directory, Mode="sensitivity");
runFixedDirectionValidation(directory, Mode="replay");
runFixedDirectionValidation(directory, Mode="campaign");
```

Raw MAT results, all successful/failed probes, independent audit scripts,
measurement logs and generated MEX remain in
`/home/zai/.cache/collisionAvoidance/cheng-production-20260921`. The matching
[JSON manifest](CHENG_FLUID_INITIALIZATION_ADOPTION_20260921.json) records every
measured trial and independent technical hashes. Its parent-commit field denotes
the pre-change revision; the final source files are identified by their hashes
and the adoption commit recorded in the external ledger. Solver dependencies,
generated binaries, literature PDFs and unrelated user files are not committed.
