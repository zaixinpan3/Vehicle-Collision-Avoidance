# Recover circular-crossing admission with one hard full-plan solve

The circular crossing that previously rejected its first plan now completes
two deterministic 600-hold / 30 s closed-loop runs under the original hard
constraints. The minimum independently refined body gap is **0.063052740 m**.
The improvement releases the complete control sequence after the original
scalar search fails, rather than prescribing the diagnostic experiment's
successful temporal weights. **Real-time admission remains unresolved:** the
two measured first frames take **61.264 / 64.545 ms**, and separate strict
50 ms runs issue no command.

Source base: `07aa84b263fefa2c265be7fad8fabdbd527f428d`. Numerical results,
checksums and trace locations are in the [JSON companion](CIRCULAR_CROSSING_ADMISSION_REPAIR_20260921.json).
The unchanged baseline is the [September 20 campaign](STRAIGHT_CIRCULAR_VALIDATION_20260920.md),
and the motivating analysis is the [September 21 diagnosis](CIRCULAR_CROSSING_ADMISSION_DIAGNOSIS_20260921.md).

## Implemented behavior

`solveHardCbfClf.admitSection` retains the existing signed affine section,
16 amplitude cells, 32 directions and 32 performance iterations. A certified
scalar candidate still needs zero conic solves. When collision or exit checks
exclude that section but its hard base interval is nonempty:

1. Evaluate the original dictionary residuals at the existing cell boundaries,
   moving base endpoints inward by a floating-point allowance. Select the
   point with the smallest worst record residual among those passing the
   original actuator/chart/terminal/soft-CLF checks.
2. Return this point separately as an **uncertified proposal**. The admitted
   decision stays empty. Neither the proposal nor a failed solve is eligible
   for execution or storage as an incumbent.
3. Construct one `avoidanceStageQp.joint` SOCP around that proposal. Every
   actuator coordinate and certificate angle is free. This releases steering
   and braking timing over the full horizon without changing pose-chart
   domains, approximation remainders, physical rows or terminal constraints.
4. Accept a positive-status solve only after `solveHardCbfClf.certify` checks
   the original hard rows, nonlinear supports, terminal cones and soft CLF.
   The complete controller frame still has to meet its configured deadline.

There is at most one admission-recovery solve and no collision relaxation.
The global support majorants remain valid when their expansion center is
infeasible. Thus a feasible result can satisfy the original certificates even
when the proposed center does not. Unlike continuation, this branch has no
certified incumbent and no guarantee of nonempty convex-subproblem feasibility.
The search is still incomplete; it is not a global solution of the nonconvex
avoidance problem.

The selected circular proposal has positive certificate violation, as expected;
the full solution has approximately **3.949479 mm** minimum reserved
node-certificate gap in the prepared fixture. Only the latter is admitted.
Metadata records section rejection, proposal violation, full-plan verification
and the one restoration-phase call separately. Continuation still retains an
already verified incumbent when improvement fails. The certificate data format
remains version 40.

The generated prepared-frame adapter implements the same recovery and adds
status 4 for full-plan admission. Failed fresh admission returns an empty
decision. The native benchmark reader accepts the new status. Existing replay
binaries require rebuilding; this work does not install generated binaries
in the repository or replace production MATLAB orchestration.

## Experimental configuration

- MATLAB R2026a, `-singleCompThread`, existing native bicycle/geometry kernels
  and Clarabel bridge. Timing runs execute separately from tests and compilation.
- Exact sensing and the declared held affine bicycle plant; seed `20260912`
  with zero perturbation bounds. Input updates, holds and certificate nodes
  share 50 ms; the nominal performance horizon is 1.6 s / 32 holds.
- Ego speed 8 m/s; circular radius 100 m. Circular crossing uses a 4 m/s target
  starting 7.5 m outside the road normal at station 15 m. Both rectangles are
  4.8 by 1.9 m. The initial encounter expands to 96 holds / 4.8 s.
- Pose domain, terminal conditions and zero fixed clearance buffer are
  unchanged. No physical road edges are enabled.
- Three separately recorded 140-hold circular-crossing warmups precede two
  six-case campaigns. Every measured case requests 600 holds. These runs use
  a 30 s diagnostic computation budget; it is not the control period.
- Two subsequent circular-crossing trials enforce the actual 50 ms complete
  frame deadline. Failed attempts remain recorded in their own timing group.
- The existing driver audits footprints every 5 ms. An independent SAT audit
  reconstructs every saved affine endpoint and refines near-contact holds and
  their neighbors to 0.1 ms. Offline audits never choose or reject a command.

## Closed-loop results

All twelve diagnostic-budget trials execute 600 holds. All 7,200 issued
commands retain the original certificate. State and input arrays are exactly
equal between repetitions. The five previously admitted scenarios also retain
exactly the same state and input arrays as the September 20 baseline.

| Road / target | Refined signed body gap, m | Outcome |
| --- | ---: | --- |
| Straight / stationary | -0.000551406 | Existing inter-node overlap remains |
| Straight / oncoming | -0.005442167 | Existing inter-node overlap remains |
| Straight / crossing | 9.519500954 | Clear, but nominal cruise already avoids the target |
| Circular / stationary | 0.082579984 | Sampled avoidance passes; unchanged trajectory |
| Circular / oncoming | 0.089778364 | Sampled avoidance passes; unchanged trajectory |
| **Circular / crossing** | **0.063052740** | **Previously rejected; now completes avoidance and target release** |

For circular crossing, the refined minimum body gap occurs at **1.6826 s**.
The minimum independent SAT margin is **0.056334579 m at 1.6814 s**; SAT margin
and Euclidean body gap are different positive distance measures but agree on
overlap signs. The 5 ms audit reports 0.067579774 m, while the node-only minimum
is 0.112656995 m. The reconstructed affine endpoints have zero discrepancy at
the recorded precision. The unchanged-cruise counterfactual overlaps, with
minimum SAT margin -3.359627 m at 1.875 s, so this is an actual avoidance task.

The executed trajectory reaches maximum node lateral error 3.227888 m and
heading tracking error 0.499983 rad. The optional physical-model diagnostic
envelope is exceeded: its minimum sampled margin is -0.115447. The enforced
pose-chart domains still pass. This does not validate the affine controller
on a nonlinear tire/vehicle plant or establish accuracy beyond that model.

The positive refined gap is a finite-sampling result, not a continuous-time
proof. The online certificate still covers hold nodes. The two unchanged
straight overlaps show why controller-wide collision freedom cannot be claimed.

## Runtime and strict rejection

The complete frame measurement includes input assembly and the controller,
but excludes plant integration and offline audits. Warmups are retained
separately, not mixed with measured frames or silently removed after a miss.

| Diagnostic-budget frame group | Count | Median, ms | P95, ms | Maximum, ms | Over 50 ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| All | 7,200 | 7.069 | 14.320 | 104.576 | 6 |
| First target admission | 12 | 50.225 | 91.947 | 104.576 | 6 |
| Active continuation | 708 | 14.103 | 28.528 | 40.644 | 0 |

Circular crossing needs one full-plan recovery in each repetition; admission
takes 61.264 and 64.545 ms. The strict runs terminate at time zero, with no
issued control and measured rejected-frame times 57.841 and 54.698 ms. One
reaches the native work limit; the other exhausts the search budget before
recovery. This is honest deadline rejection, not real-time qualification.
Cooperative deadline checks do not make computation preemptive at exactly
50 ms.

A separate profiler run identifies formulation, section admission, joint
assembly and the native solve as material costs. Proposal selection is a
smaller component. Profiled and cold-start times are not substituted for the
campaign measurements. Warmup sequences differ from the September 20 campaign,
so cross-campaign latency differences are not a controlled performance A/B test.
Observed maxima are not WCET bounds and measured computation delays are not
applied to the immediate-input simulated plant.

## Validation and reproducibility

- Full repository suite: **750 passed, zero failed or incomplete**.
- Final focused rerun after implementation/code-generation fixes: **40 passed**.
- The ten new full-plan admission tests cover mirrored turns and curvatures
  -0.01, 0.008, 0.01 and 0.012 1/m, original hard constraints, rejected scalar
  admission, failed and unsafe solver responses, expired-budget call exclusion,
  inherited-incumbent preservation, and positive-gap closed-loop completion.
  They were rerun after generated-code compatibility fixes.
- Existing affine admission tests: 18 passed. Standalone adapter tests: six
  passed, including accepted full-plan admission and empty output after failed
  admission. Native benchmark workflow tests: six passed.
- Generated MEX compilation succeeds. Its result passes the original verifier;
  maximum input/CLF decision difference from interpreted replay is
  `5.1031e-12`, maximum angle difference `8.8994e-9`. This validates the kernel's
  result, not its complete-pipeline real-time deployment.
- MATLAB Code Analyzer finds no issues in the new driver/test class. The solver
  retains a pre-existing informational logical-indexing suggestion. Python
  syntax and Git whitespace checks pass. Technical-artifact hashes accompany
  the JSON report; no weekly report hashes are produced.

Production reproduction:

```bash
matlab -singleCompThread -batch "addpath('scripts'); runAdmissionRecoveryValidation(OutputDirectory='/absolute/new/output');"
matlab -singleCompThread -batch "results = runtests('tests'); assertSuccess(results);"
```

Raw artifacts are retained in
`/home/zai/.cache/collisionAvoidance/circular-crossing-repair-20260921/`.
`campaign.log` records the full validation driver, independent `auditCampaign`
and exact-array `compareRepair` checks. `generated-build-final.log` records
successful C generation, followed by a verification-harness path error;
`generated-verification.log` records the corrected verification passing.
The earlier generator attempts exposed variable-size loop and struct-field
declarations, now corrected with an indexed loop and explicit `coder.varsize`.
They were development failures, not accepted simulation results.

The earlier historical diagnostic is reproducible at commit
`07aa84b263fefa2c265be7fad8fabdbd527f428d`; its assertion that production rejects
the fixture is intentionally no longer true after this repair. The current
validation driver replaces that historical failure check for production use.

Committed scope comprises controller search/orchestration documentation,
native replay compatibility, regression tests, a reusable validation driver
and this report. Raw traces, generated classes/binaries/figures, external
solver dependencies and unrelated user materials remain excluded. Further
work is needed on first-frame execution time and inter-node safety; neither
requirement is represented as solved here.
