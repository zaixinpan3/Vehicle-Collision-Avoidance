# Joint support certificates: implementation and validation

Date: September 19, 2026. Project B: Collision Avoidance.

Historical experiment revision: `8529584774ffd17b2cae7d2fb18aac219b2fde13`.
The paired results and commands below describe that revision. Joint support
subsequently became the sole algorithm; current usage is documented in
[JOINT_SUPPORT_CERTIFICATES.md](../controller/JOINT_SUPPORT_CERTIFICATES.md).

The selectable `jointSupport` policy completes all nine tested straight/circular
encounters over 120 holds, compared with two completions for the default
`fixedNormal` baseline. All 1,080 final joint-policy commands have independently
verified node certificates. All nine encounters confirm release and recover
cruise. **Seven of nine** also meet the driver's eleven-samples-per-hold clearance
check: the straight stationary and oncoming cases still fall below the requested
0.25 m clearance between certified nodes. Admission remains too slow for a
general 100 ms claim.

## Implementation and research finding

The controller now supports joint input/angle SOCP majorants, exact conic
interval-yaw support, a convex-base feasibility phase, bounded unexecuted
restoration, one hard improvement, and independently verified incumbent
retention. Collision and encounter-exit directions are decisions. Accepted
occupied sets, charts, angles, numerical bounds and terminal obligations shift
together; geometric reconstruction cannot silently exclude the inherited point.
No new core source file or dependency was introduced.

The theoretical proposal is sound under its declared assumptions. Its local
restoration can nevertheless stop at positive violation: the square example's
first majorant forces the zero step. The new tests reproduce that obstruction
and a safe escape from a second free-angle initialization. The implementation
therefore supports up to five deterministic initializations, with separate
monotone violation histories. This bounded search is not globally complete.
The derivation, literature checks and precise assumptions are in
[JOINT_SUPPORT_CERTIFICATES.md](../controller/JOINT_SUPPORT_CERTIFICATES.md).

The independent verifier also checks the physical convex base before adopting
an intermediate restoration center. A lying or inaccurate solver status cannot
move the next touching model outside actuator, terminal or CLF constraints.
A successful improvement flag likewise cannot authorize an unsafe command.
A failed improvement may return an independently verified incumbent, with its
source and actual solver status explicitly reported. Full-frame deadlines still
reject commands.

## Paired offline experiment

Use MATLAB R2026a Update 3, one computational thread, seed 20260912, reference
speed 8 m/s, sample time 0.1 s, 120 holds (12 s), sensing radius 16 m and
clearance 0.25 m. Curvatures are 0 and +/-0.01 1/m. Targets are the existing
stationary, oncoming and crossing fixtures. Ego/target state errors are zero,
road boundaries are disabled, and the held affine plant is declared exact.
Both search and frame budgets are 30 s for this offline algorithm experiment;
the physical sample remains 100 ms. This is not a periodic-execution qualification.

The first campaign executes 18 paired trials. A subsequent audit added the
independent intermediate-base verifier, after which all nine joint-policy
trials were rerun in a fresh MATLAB process. The table uses those final nine
runs and the unchanged baseline measurements. Timing includes first use and
admission, excludes offline truth integration, and is a desktop observation.
The processes were not run concurrently with tests.

Margins below are distance minus the requested 0.25 m clearance, in metres.
A positive node margin is distinct from the sampled inter-node margin. The
maximum frame includes admission. Restoration counts are total conic feasibility
steps in each run; target-free frames use the common original optimizer.

| Scene | Curvature (1/m) | Baseline holds | Joint holds | Node margin (m) | Sampled margin (m) | Restoration solves | Max frame (ms) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| stationary | +0.00 | 120/120 | 120/120 | 0.000223 | -0.024102 | 2 | 985.272 |
| oncoming | +0.00 | 26/120 | 120/120 | 0.000070 | -0.063598 | 8 | 204.919 |
| crossing | +0.00 | 120/120 | 120/120 | 9.269691 | 9.269691 | 0 | 20.376 |
| stationary | +0.01 | 0/120 | 120/120 | 0.100095 | 0.097674 | 39 | 1648.470 |
| oncoming | +0.01 | 26/120 | 120/120 | 0.096496 | 0.060048 | 22 | 535.546 |
| crossing | +0.01 | 0/120 | 120/120 | 0.104378 | 0.096356 | 125 | 4536.695 |
| stationary | -0.01 | 0/120 | 120/120 | 0.093649 | 0.089601 | 33 | 1350.467 |
| oncoming | -0.01 | 26/120 | 120/120 | 0.097138 | 0.043824 | 19 | 458.922 |
| crossing | -0.01 | 0/120 | 120/120 | 0.093669 | 0.088387 | 93 | 3357.760 |

The final straight oncoming body gap at the worst audited sample is 0.186402 m;
the corresponding clearance margin is -0.063598 m. There is no sampled body
overlap in these nine runs, but sampling does not prove a continuous minimum.
The curved cases have positive sampled clearance in both turn directions.

An independent aggregation of final MAT traces checks all **280 active
successor frames**: every shifted nonlinear certificate remains feasible and
every frame uses exactly one conic solve. The largest shifted residual is
-6.896261e-5 m. Active successor timing is median **15.164 ms**, maximum
**84.792 ms**. These exclude admission and do not establish a worst-case bound.
The largest complete frame is **4.536695 s**, on curved crossing admission.
The smallest actual node clearance margin is 7.047832e-5 m. All nine final
lateral errors are below 1.3e-5 m; this finite sample is not a general
convergence theorem.

## Checks

**105 unique MATLAB test cases pass**, zero failed/incomplete in the final
combined results. They cover the new certificate class (19 cases), existing
fixed-normal behavior, node certification, curved pose domains, target
conditioning, scheduled terminal continuation, configuration and the 20-file
source budget. One initial test expected bare target IDs instead of the public
`trackId:` keys; its expectation was corrected to compare retained identities,
and the complete new class was rerun. No production failure is hidden by that
correction.

The geometry tests check 2,000 seeded majorants, touching, nonunit/zero support,
and exact conic versus analytic yaw support for fixed, interval and full-circle
yaw sets. Behavioral tests cover the stalled square, a second-start escape,
positive-slack rejection, infeasible-base rejection, unsafe intermediate
proposals, oncoming admission, uncertain two-target conditioning, scheduled
finite-slew updates, retained-incumbent timeout handling and frame deadlines.
The baseline regression cases continue to use `fixedNormal`.

Factory-settings Code Analyzer reports no errors and eight performance
advisories (array growth and sparse indexing); these include existing code and
one new sparse stage-map assembly site. Diff checks pass. Logs are retained
with the execution artifacts.
The focused suite is not a claim that the entire repository test suite ran.
No estimator, hardware, nonlinear-plant or finite-road qualification is claimed.

## Reproduction and artifacts

Enable the research policy with
`cfg.controller.certificateMethod = "jointSupport"` before the first call;
start with empty stored state. The default remains `fixedNormal` for comparison.
For offline study, configure both the frame deadline and certificate search
budget explicitly; increasing the budget does not establish real-time safety.

```matlab
addpath('scripts');
runJointSupportCertificateValidation( ...
    OutputDirectory='/tmp/joint-support-paired');
runJointSupportCertificateValidation( ...
    OutputDirectory='/tmp/joint-support-final', Methods="jointSupport");
results = runtests({'tests/jointSupportCertificateTest.m', ...
    'tests/shiftedNominalConvexificationTest.m', 'tests/nodeCertificateTest.m', ...
    'tests/curvedPoseDomainTest.m', 'tests/targetObservationConditioningTest.m', ...
    'tests/scheduledTerminalContinuationTest.m', ...
    'tests/collisionAvoidanceControllerConfigTest.m', ...
    'tests/controllerSourceBudgetTest.m'});
assertSuccess(results);
```

The new driver and the extended exact-state driver remain in `scripts/`.
The compact [JSON results](JOINT_SUPPORT_CERTIFICATES_20260919.json) record
settings, both methods, final independent checks and limitations. Raw MAT
traces, logs, intermediate campaign results and final test results are in
`/home/zai/.cache/collisionAvoidance/joint-support-20260919/`.
The baseline source revision is
`4e8a312116ef381e7c10e58a12d0b1c21814061b`.

The task commits only controller/configuration code, the two experiment drivers,
the new behavioral tests and English theory/results documents. Pre-existing
untracked instruction files, reference PDFs, paper artifacts, dependency trees
and native binaries are excluded. The external activity ledger records the
resulting full project commit and verified push. No weekly report hash is used.
