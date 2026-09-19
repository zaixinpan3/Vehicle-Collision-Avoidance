# Joint support as the sole controller algorithm

Date: September 19, 2026. Project B: Collision Avoidance.

The controller now uses joint trajectory/separation-angle optimization for
every active encounter. The fixed-normal execution branch, witness-excluding
replacement helper and certificate-method selector are removed. Target-free
frames solve the shared convex dynamics/CLF/terminal base. The research driver
also has no method selector or paired-baseline loop.

Stored certificates use format 37. Start with an empty controller state after
upgrading; earlier formats and the removed configuration field are rejected.
The public controller signature and joint solver mathematics are unchanged.
The source budget remains 20 files. Native kernels and dependencies are not
changed or rebuilt.

The current derivation is
[JOINT_SUPPORT_CERTIFICATES.md](../controller/JOINT_SUPPORT_CERTIFICATES.md).
The old active algorithm documents are removed; their still-applicable CLF
derivation is retained in [SAMPLED_CLF.md](../controller/SAMPLED_CLF.md).
Dated experimental findings remain historical, with obsolete document links
pinned to their source revision.

## Scenario validation

Run from the repository root:

```bash
matlab -singleCompThread -batch "addpath('scripts'); summary=runJointSupportCertificateValidation(OutputDirectory='/tmp/joint-support-only'); trials=[summary.trials{:}]; assert(numel(trials)==9 && all([trials.completed]) && all([trials.allIssuedCommandsCertified]));"
```

The executed run used MATLAB R2026a Update 3, seed 20260912, 120 holds of
0.1 s, reference speed 8 m/s, sensing range 16 m, clearance 0.25 m, curvature
0 and +/-0.01 per metre, and stationary/oncoming/crossing targets. The plant
and sensing were declared exact, roads had no physical boundaries, and frame
and search budgets were 30 s for offline validation.

All nine trials completed, confirmed target release and issued 1,080
independently node-certified commands. Twelve non-timing summary fields per
trial, including node/inter-node margins, nonlinear residuals, solve counts
and final tracking errors, exactly match the final joint trials from
`8529584774ffd17b2cae7d2fb18aac219b2fde13`. This checks that removing the
selector preserves the tested joint behavior.

The minimum node clearance excess is 0.0000704783 m. Seven of nine cases pass
the eleven-samples-per-hold clearance diagnostic; straight stationary and
oncoming cases retain excesses of -0.0241018 and -0.0635984 m. Node certificates
do not establish intersample safety. The maximum observed frame time is
4.738493 s; this run overlapped repository testing and is not an isolated
runtime benchmark or a real-time qualification.

Raw MAT traces, summary JSON, process logs and the independent comparison are
under `/home/zai/.cache/collisionAvoidance/joint-support-only-20260919/`.

## Regression scope

The continuation tests now check default joint admission, optimized directions,
retained suffix angles and absolute deadlines, hard support residuals, changed
sensing contracts and rejection of older saved states. Curved admission tests
verify hard certificates instead of expecting the retired fixed-normal
algorithm to fail. Node coverage checks certificate records instead of deleted
fixed collision rows. Solver fault injection permits only independently
verified incumbents; restoration slack and full-frame deadline overruns still
cannot authorize commands.

The final focused result covers **173 unique passing cases**, with zero failed
or incomplete cases, across `jointSupportCertificateTest`,
`certificateContinuationTest`, `collisionAvoidanceControllerTest`,
`nodeCertificateTest`, `curvedPoseDomainTest`, `targetObservationConditioningTest`,
`scheduledTerminalContinuationTest`, `collisionAvoidanceControllerConfigTest`,
`controllerSourceBudgetTest`, `recursiveSafetyClosureTest` and
`encounterCertificateScenarioTest`. The initial run exposed the three obsolete
expectations described above; all 19 cases in the two affected classes passed
after updating them. The saved final result replaces those classes with their
rerun results rather than double-counting cases.

The full `runtests('tests')` attempt did **not** complete. The MATLAB MCP call
timed out after 300 s; MATLAB continued running without a result file, with its
last diagnostic coming from `estimatedStateAvoidanceVideoTest`/`getframe` and
reporting unavailable graphics acceleration. The run was interrupted and its
MATLAB process terminated after the interrupt did not stop it. This is not a
full-repository pass, nor proof of a controller failure.

Factory-settings Code Analyzer checks on all 12 changed MATLAB files found
only five sparse-indexing performance advisories in unchanged terminal code.
`git diff --check`, the 20-source budget check and current controller-document
link checks passed.
