# Exact-Fiala feedback proof, sparse repair and bounded research trials

September 10, 2026. Baseline:
`53d437b737f4606235f09500d226ccdb870c261d`. Certificate version 16 changes
numerical transcription and stored row-map context, not the online safety
policy. The complete source-derived theorem is in
[FEEDBACK_POLICY_CERTIFICATE.md](FEEDBACK_POLICY_CERTIFICATE.md).

## Decisions and scope

1. Adopt the nonlinear modified Fiala flow as the true model for the proposed
   research theorem. There is no required MF62-to-Fiala discrepancy proof for
   that assumption. Affine linearization and numerical propagation errors
   still need valid bounds. `fialaWorldDynamics` exposes the existing exact
   nonlinear flow in Cartesian coordinates without duplicating its tire law.
2. Prove suffix closure, continuous-time separation, finite exit and
   full-information prefix splicing for a robust sampled-feedback witness.
   The proof includes held feedback, arbitrary time-varying residuals,
   a noncircular domain argument and uniform metric coercivity. It requires
   no post-exit invariant terminal set or externally imposed 1.6 s deadline.
3. Keep one online controller. The sparse path is now the only production
   transcription. Continuous-normal refinement and alternative performance
   formulations are research proposals/experiments, not maneuver modes or
   executable uncertified fallbacks.
4. Do not promote the affine geometry trial to a nonlinear controller: its
   true Fiala replay collides. A contracting local tracking experiment is
   encouraging but does not close the validated nonlinear tube gap.

## Sparse integration repair

The old three-argument `avoidanceStageQp` implemented a sparse lift, but the
actual initial and rebuilt paths used its condensed one-argument alternative.
The hard-margin LP also used the complete condensed matrix. These paths now
share sparse cell states, controls and CLF slacks. A retained context supports
consistent rebuilding after joint admission. `updateAvoidanceStageBounds`
updates only mapped inequalities; it never overwrites dynamics equalities.

Redundancy screening is valid over the complete carried-margin range, and
identical-row screening includes each row's margin coefficient. All original
hard rows remain in `qp` for the independent physical-decision checker. The
old condensed transcription is confined to `tests/condensedAvoidanceTestOracle.m`.
Stored version-15 certificates are rejected because they lack the new contract.

The first sparse performance experiment reported native primal infeasibility
at iteration 1. That was a numerical failure: lifting a checked LP incumbent
and reconstructing its CLF slacks produced equality residual 3.6e-14, minimum
linear margin 4.13e-4 and minimum cone margin 4.02e-5 in the same program.
Positive scaling of P and q, with the corresponding absolute optimality
threshold scaled and reported objective unscaled, restored both native stages
to status 1. The original ~53 ms combined LP/SOCP measurement included this
premature failure and is **not** a successful two-stage timing result.

The corrected saved-frame experiment uses the same original target-free
10 m/s, 0.1 s, 16-step frame and residual-rate allowance
[0.2,0.06,0.02,2.5,5,4]. Keeping that allowance is deliberate for a comparable
numerical workload; it is not part of the new exact-Fiala physical hypothesis.

| Quantity | Earlier condensed measurement | Corrected sparse measurement |
|---|---:|---:|
| Native hard-margin LP | approximately 120--124 ms | 28.962--30.816 ms |
| Native performance SOCP | approximately 204--210 ms | 98.942--106.998 ms |
| Complete saved-frame call after prior warmup | 565.438--616.974 ms | 271.042--368.122 ms |
| LP rows | 21,650 | 9,208 |
| SOCP rows | 30,608 | 17,610 |
| SOCP Lorentz cones | 896 | 841 |

All six corrected native solves return status 1, with 13 LP and 20 SOCP
iterations. Three complete calls are certified. These few samples are not
worst-case timing, and the complete call still exceeds the 100 ms sample
period. Formulation remains roughly 100--137 ms in these replays. Row-map
repair does not solve synchronous scheduling by itself.

The updated `profileNativeAvoidanceSolver` builds a separate instrumented
bridge, applies the identical objective scaling and compares six solves per
phase against production. All twelve status/decision comparisons match
exactly. Last-five median setup/solve-call times are 4.587/24.872 ms for LP
and 17.513/82.982 ms for SOCP; total native times are 29.621 and 100.596 ms.
This confirms that native iteration work and outer formulation remain material
costs after fixing sparse integration.

## Continuous direction refinement on the diagnosed oncoming encounter

`optimizeSeparationNormals` solves a five-variable support SOCP per cell and
target, using synchronous Bernstein positions, rectangle corner differences,
and conservative heading-error disks. Its unit-normal output is only a
proposal. `avoidanceSafetyGeometry` can rebuild all hard swept rows with those
normals; the complete original affine checker decides candidate acceptance.

`evaluateContinuousSeparation` uses the previously saved actual first-detection
input (approximately 29.9 m oncoming separation, 10 m/s each, lateral offset
0.8 m), 32 stages, zero additional affine residual and explicitly extended
road coverage [-200,200] m. Its initializer is a saved feasible affine plan
from the preceding diagnostic, not a globally successful admission procedure.
The original straight-probe formulation is infeasible. Three continuous
normal/trajectory updates pass the complete affine check, with margins
0.040219718, 0.040850803 and 0.040881131, taking 1.656190, 1.218625 and
1.387525 s respectively in the final run.

Replaying the final inputs in the authoritative nonlinear Fiala model, with
held commands and RK4 at 0.001 s, gives minimum sampled excess rectangle
clearance **-0.397880526 m**. The configured clearance is 0.25 m, so the
signed rectangle separation is approximately **-0.147880526 m**: the bodies
overlap in numerical replay. Final center distance is 33.103579737 m, which
does not undo the collision that already occurred. Numerical sampling is
used here to falsify the affine-to-nonlinear transfer, not to certify safety.

Conclusion: continuous normals enlarge the affine feasible region but cannot
repair nonlinear model mismatch. The final replay retains the same negative
finding after objective scaling. An accepted affine plan must not become an
exact-Fiala certificate without validated nonlinear propagation.

## Held feedback under the exact-Fiala assumption

`sampledFeedbackTransition` uses exp(A h)+Gamma(h)K. Tests distinguish it
from continuously updated feedback and verify direct held-input integration.
`evaluateFialaSampledFeedback` designs a six-state sampled LQR about a straight
10 m/s trim, with a fixed positive metric. Both trials start with error
[0.01,-0.01,0.001,0.005,-0.001,0.001] in Cartesian state units and run for
6 s, h=0.1 s, RK4 substep 0.001 s. Measurements are exact, there is no
extra disturbance and no input clipping.

- Fixed metric eigenvalues lie in [0.104706809,106.996449978].
- Linear sampled-map metric contraction is 0.945254096; spectral radius
  is 0.874549656.
- Initial metric error is 0.059368352. Open-loop final error is 0.269650833;
  feedback final error is 0.0000921341.
- Largest observed nonlinear step ratio is 0.933800666. This observed ratio
  is not a bound over a domain.
- Maximum steering is 0.001931664 rad; maximum absolute braking ratio is
  0.015024182. Observed rates are 0.019316636 rad/s and 0.034124389 /s.
  Minimum longitudinal speed is 9.994686312 m/s. Input bounds pass.

This tests the actual sample/hold law and demonstrates local error reduction
including longitudinal position. It is a tracking experiment without a target,
not an avoidance success, nonzero-disturbance tube certification or numerical
proof of a practical oncoming admission domain.

## Soft CLF and four-step prefix ablations

`evaluateCertifiedPrefixCost` starts from the same complete target-free affine
certificate. It compares one inherited-margin SOCP, the same hard constraints
with only quadratic tracking/input performance, and the latter with four free
input stages. For the prefix trial, tail inputs and auxiliary states from the
splice inlet onward are fixed to the incumbent. All original hard rows remain;
tail rows are not yet removed or reused without scanning. Soft CLF slacks are
reconstructed afterward and every candidate passes the original complete
checker. This affine fixed-tail experiment does not instantiate nonlinear
feedback information-set inclusion.

| Offline formulation | Active variables | Lorentz cones | Median native time | Complete acceptance |
|---|---:|---:|---:|---:|
| Inherited-margin SOCP | 726 | 841 | 99.230 ms | 5/5 |
| Quadratic tracking/input QP | 710 | 0 | 24.610 ms | 5/5 |
| Four-step QP prefix, fixed inlet/tail | 176 | 0 | 6.270 ms | 5/5 |

Medians use samples 2--5 of five calls. Native status is 1 in all 15 solves.
Times exclude prefix construction, complete verification and outer controller
work. Modified inputs differ from the incumbent by approximately 0.001950
and 0.001068 in stacked input Euclidean norm. Soft-CLF reconstruction is used
only to compare against the original checker; it does not preserve the
original CLF-slack optimum. Safety depends on the retained hard obligations.

The strongest measured computational opportunity is therefore a single sparse
QP over a short prefix, with a proved full-information tail inlet and reused
tail proofs. Deploying it still needs nonlinear feedback tubes, splice
membership checks and an independently scheduled executor. Removing cones
alone is not a proof of tracking stability or a 6 ms controller.

## Reproduction and validation

Inputs are the actual saved acquisition and runtime exports from EV-0099 and
the preceding zero-residual/road-coverage diagnostic (EV-0101). Raw outputs
for this task were first written to `/tmp/feedback-sparse-work-20260910/` and
are exported into the task's external evidence bundle at archival closure.
No random sampling or seed is used in these deterministic trials.

```matlab
addpath('scripts');
profileForceFreeAdmission('/tmp/force-constraint-removal-20260910', ...
    '/tmp/feedback-sparse-work-20260910/sparse-runtime-final');
evaluateContinuousSeparation('/tmp/force-constraint-removal-20260910', ...
    '/tmp/force-free-diagnosis-20260910', ...
    '/tmp/feedback-sparse-work-20260910/geometry-final');
evaluateCertifiedPrefixCost('/tmp/force-constraint-removal-20260910', ...
    '/tmp/feedback-sparse-work-20260910/prefix');
evaluateFialaSampledFeedback('/tmp/feedback-sparse-work-20260910/feedback');
results=runtests('tests'); assertSuccess(results);
```

The full suite initially passes 609/611 tests. The new large-residual test
mistakenly paired the diagnosed vehicle's residual allowance with a different
crossing model; it is corrected to use the diagnosed straight vehicle's mass,
inertia, axle geometry and tire properties. The other failure is the overly
strong fresh-solve expectation described below. Final focused revalidation
passes 22/22 tests, and name-based replacement yields 611 current passing tests.
This is a full run plus focused revalidation, not a second full run. Factory
Code Analyzer reports sixteen existing sparse-indexing performance advisories
in the lifted transcription and two growth advisories in the offline prefix
driver; no correctness finding is reported.

A continued crossing regression exposed an LP candidate whose inherited-margin
shortfall was 3.42698e-13 despite native status 1. Strict independent checking
rejected that replacement, retained the certified suffix and completed safe
exit at 0.5 s. Requiring zero retained-suffix uses in an otherwise successful
online optimization scenario was an overly strong numerical test expectation;
it is replaced by successful improvement plus verified safe completion. The
hard acceptance margin is not relaxed. This observation reinforces the need
to make execution independent of fresh optimizer success.

The mathematical proof is conditional and its missing nonlinear-verifier
implementation is identified explicitly. Existing PassVeh14DOF empirical
validation configuration is retained as a separate experiment; it is not
silently redefined as true Fiala validation. No infinite-horizon or universal
local-optimizer completeness result is claimed.
