# Terminal CBF proof audit and removal of retired code

Experiment date: September 8, 2026. MATLAB R2026a Update 3.

The mathematical audit passes in its stated affine scope. **A CBF for the
current nonlinear online estimator/controller is not established.** The
[formal derivation](../controller/TERMINAL_CBF_PROOF.md) proves both the affine
terminal result and an obstruction to applying that terminal set unchanged to
the actual nonlinear model.

## Reproduction and experiment scope

```matlab
addpath('scripts');
report = runTerminalCbfProofAudit(OutputDirectory="/absolute/output/path");
assert(report.auditPassed);
assert(~report.fullControllerCbfEstablished);
```

This is a standalone equation audit, not an alternative controller. It calls
current vehicle-model and rectangle-geometry functions but supplies no control
to the online pipeline. Original results are retained outside the repository:
`/home/zai/.cache/collisionAvoidance/terminal-cbf-proof-20260908/final`.
Evidence copies are made only after the project commit and verified push.

The auxiliary Huang-style LP uses a fixed affine zero-speed generator, a
100 ms step and four prediction stages. Each of two examples runs five
updates. Initial lateral velocity, yaw rate and steering are zero; the scalar
braking ratio is in [-1,0]. Station and forward-speed uncertainty radii are
0.01 m and 0.001 m/s. Exact affine transitions carry signed generators.
There is no random seed, measurement noise, observer or runtime qualification.
Safety slack in this LP is separate from the online CLF slack. Its
positive-slack actions are mathematical examples, not authorized commands.

## Measured mathematical checks

| Check | Result |
|---|---|
| Safety value from 2.5 m/s, normalized speed limit 1 m/s | 2.1276854524, 0.6266854524, 0, 0, 0 |
| Safety value from a safe 0.8 m/s initial state | 0 at all five updates |
| Maximum positive shifted-candidate violation | 0 at ordinary solver precision |
| Maximum positive violation of `B(next) - B + headSlack <= 0` | 0 at ordinary solver precision |
| Affine terminal monotonicity, curvature 0, 1/400, 1/100, -1/100 per metre | No negative sampled increment beyond 1e-12 |
| Independent corner evaluation versus signed-generator formula | Maximum absolute difference 5.5512e-17 |

The affine checks use times 0, 0.001, 0.01, 0.1, 1 and 5 s. The proof covers
all times under its explicit model assumptions; the samples are regression
checks of the implementation, not a substitute for that argument.

## Counterexamples to broader claims

For a straight road, the retired set admits a lateral boundary state
`[s,d,ePsi,vx,vy,r] = [0,0.05,0.2,0.8,0,0]`. The actual nonlinear kinematics
have outward speed `vx*sin(ePsi) = 0.1589354646 m/s`, independent of current
bounded steering or force input. Thus the set is not control invariant for
that nonlinear model. Starting at its interior with `d=0` and propagating
zero input for 0.5 s yields `d=0.0772301304 m`, exceeding the 0.05 m budget.
The affine barrier rises from 0.2 to 0.2747534053; the nonlinear endpoint
has barrier -0.5446026072.

A constant longitudinal residual of 0.001 m/s² with damping 0.1962/s,
initial speed 0.01 m/s, and station budget 0.2 m makes the same fixed-policy
pose barrier decrease from 0.7451580020 to -0.7838939857 over 60 s. A finite
larger margin only postpones this persistent drift.

A stopped ego can remain in its own terminal set while a target reaches it:
target initial position 10 m ahead, constant velocity -2 m/s, time interval
3 s, and footprints 4.8 by 1.9 m / 5 by 2 m. The sampled rectangle SAT margin
changes from 5.1 m to -0.9 m. This short encounter requires no infinite target
future assumption. None of these examples is an observed online-controller
collision.

## Maintained code and validation

Removed the unused dissipative/stationary terminal APIs, rest-tail prediction
and braking schedule, unused backup-deceleration setting, and the obsolete
solver-failure continuation experiment. Removed obsolete tests and migrated
still-applicable tests to the current finite-prediction and uncertainty
interfaces. Consolidated the former terminal documents into the formal audit.
There are now 19 core source files. Git history retains the previous version;
no alternate legacy controller implementation is kept in the repository.

The audit has 11 behavior tests. The focused first pass exposed two migrated
prediction tests using retired output semantics; they now verify the current
cumulative domain enclosure against analytic disturbance integration. All
12 finite-prediction tests pass after that correction. Earlier test-scaffolding
outputs are diagnostics, not final-release validation.

The cleanup preserves the online Predictive CBF/CLF objective, high-gain
observer and failure-stop behavior. No complete nonlinear CBF, global
recursive-feasibility, convergence, or pipeline timing claim is added.

Final verification covers **573 distinct tests, all passing**: the full run
passed 572/573, then the circular-cruise test was corrected to use the current
sideslip-aware kinematic identity `r = curvature*hypot(vx,vy)` instead of the
retired `curvature*vx` expectation. That class and the 11 proof-audit tests
passed on retest (12/12). The merged result preserves the original full-run
record and replaces only matching retested results. Nine changed/new MATLAB
files have zero factory Code Analyzer findings. Static checks confirm 19 core
source files, no executable references to deleted APIs, and valid relative
links in changed documents. The tool's first full-suite call timed out waiting
after 300 s; the backend completed and its saved results were retrieved.
