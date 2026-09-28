# Direct PCBF execution and retired controller cleanup

Date: September 28, 2026. MATLAB R2026a, Update 3.

## Implemented behavior

Every controller call runs the safety-priority LP and secondary lane CLF QP
inside SCvx, then applies the first returned control. Both zero-slack and
positive-slack results are executable. Previous inputs and predicted states
are only numerical warm starts. There is no stored executable trajectory,
shifted-trajectory fallback, formal nonlinear verifier, MPFR admission layer,
or exact-successor contract. If optimization produces no result, the public
call raises `optimizationFailed`.

The controller retains the single constant-speed, constant-heading-rate
target in the joint state, Li-style rectangle duals, nominal MPC terminal
conditions and the lane CLF. SCvx uses actual/predicted merit reduction to
adjust its trust region. A trial outside the bicycle model domain shrinks
the trust region. Cold and warm starts have the same iteration budget.
LP/QP residuals and nonlinear rollout residuals are diagnostics; they do not
form a separate execution gate. A failed secondary QP can leave the primary
LP result as the current iteration's result.

## Deletion scope

The active core is eight MATLAB source files, including configuration,
with 1,662 total lines. The cleanup removes or replaces 244 tracked paths:
retired affine MPC and SOCP solvers, interval and tube algorithms, native
bridges/builders, their experiment drivers and tests, superseded controller
theory, and old reproduction code under `report/`. The exact list is in
`DIRECT_PCBF_CLEANUP_20260928/cleanup-manifest.json`.

`readControllerInputs` replaces the old planning/admission parser.
`solvePredictiveControl` replaces the executable-plan solver. Active road
load physics now belongs to `nonlinearBicycleModel`; unused lane and tire
helpers and legacy configuration fields were removed. Current API documents,
preparation scripts and estimator-interface coverage use the new names.

Historical numeric results and reports remain. Their old executable source
is available in Git history. Pre-existing edits in six retired tracked files
were preserved before deletion as an external patch; the retired untracked
`nrmmVelocitySectorTest.m` was copied there too:

`/home/zai/.cache/collisionAvoidance/retired-controller-edits-20260928-134721/`

Active estimator/perception algorithms, unrelated observer/report edits,
untracked observer comparison work, reference PDFs, and external/nested
solver dependencies were excluded from the commit.

## Checks and outcomes

- All 379 tests in the remaining repository suite passed; no failed or
  incomplete tests. The full run includes six pre-existing untracked Sharma
  observer tests; those sources remain outside this commit.
- The focused controller/configuration/model suites passed all 136 tests.
  They include first-control equality for zero and positive safety slack,
  fresh optimization with a warm start, and failure when no optimizer result
  exists. No-result behavior is not replaced by replaying old controls.
- MATLAB code analysis covered 12 current files. Its 12 findings are sparse
  indexing performance notices (`SPRIX`), all in SCvx assembly.
- A retired-name scan covered 84 retained source files and found no executable
  references to removed algorithms. Two references remain in comments in
  unrelated, untracked observer-comparison source.
- Python audit syntax and `git diff --check` passed. Controller preparation
  returned a result without a native controller build.

The deterministic replays use 8 m/s ego cruise, 50 ms holds, 4.8 by 1.9 m
rectangles and an 8 m wide corridor. Recovery starts with 0.01 m lateral
error. Circular lane following uses curvature 0.005/m. Both target cases
start at (24, 0) with heading pi and speed 8 m/s; heading rate is zero for
the oncoming target and -0.8 rad/s for the turning target. No random draws
are used. Tight `ode45` replay uses relative/absolute tolerances 1e-11/1e-12
and 31 samples per hold, independently of the controller's RK4 map.

| Fixture | Holds | Minimum polygon distance (m) | Minimum road margin (m) | Final lane error (mm) | Later median solve (s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| recovery | 40 | No target | 3.039233 | 0.708 | 0.021739 |
| circular | 40 | No target | 3.022088 | 0.000 | 0.021518 |
| oncoming | 160 | 0.099026 | 0.337176 | -5.195 | 0.237684 |
| turningTarget | 160 | 2.618563 | 3.046506 | -3.110 | 0.254909 |

All 400 holds completed, with at least two solver calls per hold and no
stored-trajectory execution. Every returned nominal prediction had safety
slack within the declared 1e-5 tolerance and zero measured hard residual.
The largest slack was 7.887858e-6, so this is a numerical tolerance statement,
not an assertion of exact zero slack.

**The strict dense clearance audit passes three of four fixtures.** No
rectangle overlaps or road crossings occurred in the 12,400 dense samples,
and independent Python geometry agrees with MATLAB. However, oncoming
minimum clearance was 0.099026038 m at 1.535 s, falling 0.000973962 m below
the configured 0.10 m margin between prediction samples. The node/midpoint
minimum was 0.099998940 m, within the declared feasibility tolerance. The
Python audit intentionally retains its failed clearance result and exits
with status 1. This is a measured intersample limitation.

Initial oncoming optimization took 5.057279 s and the largest call took
5.102547 s. All target-case calls exceeded the 50 ms hold; a real-time bound
is not established. The time budget is checked between SCvx iterations and
can be overrun by a running LP/QP. These workstation measurements include
normal host activity and are not a controlled comparison with older results.

## Development findings and remaining scope

Early replays exposed a residual veto, first-iterate trust-region update
error, and later LP failures previously hidden by stored-trajectory reuse.
The residual veto was removed, the merit-based trust update corrected, and
the two-iteration warm-start cap removed. A dual-simplex experiment did not
improve the cold solve; the retained primary solver is interior-point.
Compact initial replay outcomes remain in `initial-replay/`. An initial
full-suite results export failed after test execution due to table indexing;
the corrected exporter reran the suite and produced `repository-tests.json`.

The PCBF recursive-feasibility argument remains conditional on the prediction
model, invariant terminal ingredients and feasible optimization. Finite SCvx
iterations and direct execution do not establish those hypotheses by
formal verification. Positive safety slack is a recovery result and does
not imply collision-free motion. The implemented road model is straight or
constant curvature and the tire model requires positive longitudinal speed
above its configured floor. See the current architecture for the equations.

## Reproduction

```matlab
addpath('scripts');
validateNonlinearPredictiveController('report/new-validation-directory');
results = runtests('tests');
assertSuccess(results);
```

```bash
python3 scripts/auditJointPredictiveSafety.py report/new-validation-directory --output report/new-validation-directory/independent-audit.json
```

The strict audit reports failure when the configured dense clearance is
missed, as it does for the oncoming case recorded here. Offline audits are
research validation and are not called during control execution.
