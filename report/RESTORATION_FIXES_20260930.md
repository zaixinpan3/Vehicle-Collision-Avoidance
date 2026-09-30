# Focused restoration fixes and collision-threat regression

Date: September 30, 2026. Baseline: `2c5c8d6ec244ff002db0265c4b106d5b4861a18d`
(the production algorithm from `8d35249675bc53020d2c374397c3a958f60b9b82`).

Two previously rejected fixtures now finish within the same five-second soft
initialization budget: braking lead and turning crossing at 8 m/s. Independent
geometry replay confirms **7/14 avoidance successes**, compared with 5/14 in
[the previous campaign](FREE_TERMINAL_PHASE_20260929.md). All five previous
successes remain successful. Seven initialization failures remain; this change
does not resolve the entire scenario set or establish real-time operation.

## Implementation

`solvePredictiveControl.m` retains the best nonlinear evaluation across raw
primary, endpoint correction, restoration and secondary results. Endpoint
correction follows a separate working candidate while preserving the best
complete result. An admitted witness always has priority; otherwise candidates
are compared by prefix slack plus 100 times the sum of completion slacks,
terminal-norm excess, maximum terminal-geometry deficit and maximum physical
violation. This is a numerical search score, not the predictive barrier value.
It replaces a maximum-violation comparison that could discard a useful decrease
in the aggregate violation minimized by the elastic subproblem.

An improving primary returns for relinearization before secondary lane-cost
work. A feasible primary still returns immediately. If a full conic step is
unhelpful or outside the model domain, at most its half and quarter steps are
checked within the same budget. A row bound can skip an already impossible hard
program. An infeasible elastic program supports bounded trust expansion rather
than blindly shrinking the same infeasible region; an unresolved solver failure
ends that attempt.

Two scalar search variables allow terminal membership and endpoint/future
separation to be repaired along with completion collision rows. They are fixed
at zero in the hard programs. **The original nonlinear endpoint, physical and
completion constraints still decide admission**, with exactly zero hard
residual. No terminal radius, physical limit or collision buffer was weakened.
The existing positive-prefix-slack recovery semantics remain; a positive-slack
return is not labeled collision-free. All seven completed campaign runs have
zero predicted prefix slack on every hold.

The first implementation campaign exposed an intersample collision in the
8 m/s crossing despite zero hard residual at nodes/midpoints. Three dense replay
samples overlap between 1.743333 and 1.746667 s. That intermediate result was
rejected as a regression. Restoring hard terminal restrictions throughout
restoration caused renewed timeouts, and adding all nearby interior rows from
the beginning also lost the desired initialization improvements.

The retained solution is bounded constraint generation: before accepting an
otherwise admissible candidate, nearby encounters are checked at interior
half-hold integration nodes. With the default RK4 mesh these are four extra
points per 50 ms hold. Proximity uses configured ego motion bounds and predicted
target motion over one hold. If an interior point violates clearance, that
stage's interior collision rows enter subsequent convex subproblems. The
candidate is reevaluated before execution. Extra rows are therefore introduced
when needed to repair a detected violation rather than throughout the entire
initial search. This is still a sampled numerical check, not a continuous-time
or model-error safety certificate.

Target prediction preserves the original epoch using absolute integer substeps;
node/midpoint arithmetic remains unchanged. The terminal interval construction
now bounds pose error at every RK4 node and includes the maximum nominal
reference-orbit defect over those nodes, so its collision tube covers the same
additional numerical sample times. Controller state version 55 rejects old
cached witnesses lacking the additional checks. There is still one
controller, with no new driving mode, optimizer method or terminal-policy
library. Road constraints remain absent, and the collision buffer remains
6 mm. The terminal cruise family and conservative future-geometry branches are
unchanged; their numerical pose padding is synchronized with the finer checks.

## Final campaign

The unchanged fourteen fixtures are seven encounter types at 8 and 15 m/s.
Each baseline follows the given path at constant speed and demonstrably
collides. Each controller run requests 160 holds of 50 ms (8 s), uses 8/16
prefix holds and a maximum horizon of 512 holds, exact state/target inputs,
and the default five-second soft solver budget. There are no random draws,
state-observation errors or delays. Tight `ode45` replay uses relative/absolute
tolerances `1e-11`/`1e-12` and 31 samples per hold; its successor is fed back to
the next controller call. Python independently checks target motion and body
geometry. Road departure is a diagnostic only.

| Speed (m/s) | Encounter | Outcome | Minimum replay gap (m) | Initialization (s) | Maximum running call (ms) |
| ---: | --- | --- | ---: | ---: | ---: |
| 8 | Head-on | Avoided; retained baseline success | 0.628114 | 1.399815 | 39.547 |
| 8 | Accelerating head-on | Avoided; retained baseline success | 0.511783 | 0.944001 | 33.370 |
| 8 | Braking lead | **New avoidance success** | 1.230713 | 2.238981 | 71.547 |
| 8 | Crossing | Avoided; retained baseline success | 0.250670 | 2.488174 | 33.650 |
| 8 | Turning crossing | **New avoidance success** | 0.728956 | 4.909288 | 808.422 |
| 8 | Curved head-on | Terminal admission rejected | — | 0.473973 failed call | — |
| 8 | Curved crossing | Terminal admission rejected | — | 0.151421 failed call | — |
| 15 | Head-on | Avoided; retained baseline success | 0.340696 | 0.989842 | 34.651 |
| 15 | Accelerating head-on | Avoided; retained baseline success | 0.269114 | 0.669822 | 33.971 |
| 15 | Braking lead | Initialization time limit | — | 5.020704 failed call | — |
| 15 | Crossing | Initialization time limit | — | 5.012197 failed call | — |
| 15 | Turning crossing | Initialization time limit | — | 5.009071 failed call | — |
| 15 | Curved head-on | Terminal admission rejected | — | 0.417201 failed call | — |
| 15 | Curved crossing | Terminal admission rejected | — | 0.146348 failed call | — |

The new braking-lead initialization uses five conic calls; turning crossing uses
seven. The latter remains close to its five-second budget: timing variability
can still cause a rejection. These measurements are from one final campaign,
not a repeated-run timing distribution or a worst-case execution-time bound.

Of 1,113 running calls, 100 exceed the 50 ms hold: 26 in braking lead and 74 in
turning crossing. The maximum is the turning-crossing call at simulation time
0.45 s, with a 192-hold remaining trajectory. It executes one primary conic
solve plus nonlinear endpoint repair and returns the first feasible candidate;
there is no secondary cost solve. The raw candidate's restoration score is
157.339177 and the retained score is zero. This trace identifies its work but
is not a timed profiler decomposition. Four running conic calls occur in this
fixture; the other six completed fixtures require none after initialization.
Long full-trajectory revalidation also costs time when no new solve is needed.
**Initialization success does not imply the controller meets a 50 ms deadline.**

The three remaining straight-road high-speed failures exhaust the same budget.
The four curved cases still fail during terminal admission, before local
optimization. Their whole-orbit separation restriction is described in the
[preceding diagnosis](FREE_PHASE_FAILURE_ANALYSIS_20260929.md); this focused
restoration change does not alter that certificate. A failed search or
sufficient certificate is not proof that the physical maneuver is impossible.

## Validation and reproducibility

All **258 selected MATLAB tests** pass across the controller, configuration,
vehicle/tire model, terminal continuation, source-budget, input geometry and
threat-scenario suites. Three added behavior tests cover preservation of useful
restoration candidates, original terminal admission after turning-crossing
restoration, and dense `ode45` clearance of a returned crossing trajectory.
The state-version regression now rejects format 54. Functional tests use a
30 s budget to avoid timing-dependent assertions; campaign runs use 5 s.

MATLAB Code Analyzer reports 15 `SPRIX` sparse-indexing performance advisories
in the solver and no findings in the orchestration entry, terminal construction
or test file. There
are no syntax findings. The independent Python audit verifies all fourteen
baseline collisions and all seven successful dense replays. This does not
prove safety between replay samples, under observation/model error, or for
unexecuted rejected cases. The full repository's unrelated estimator suites
were not rerun for these controller changes.

Run from the repository root:

```bash
matlab -batch "run('/home/zai/.cache/collisionAvoidance/restoration-fixes-20260930/runValidation.m')"
python scripts/auditCollisionThreats.py /home/zai/.cache/collisionAvoidance/restoration-fixes-20260930/verified-campaign/campaign.json --output /home/zai/.cache/collisionAvoidance/restoration-fixes-20260930/independent-audit.json
python /home/zai/.cache/collisionAvoidance/restoration-fixes-20260930/analyze.py
```

The [evidence directory](RESTORATION_FIXES_20260930/) contains compact campaign
and audit summaries, per-case comparisons, all test outcomes, analyzer findings,
source/raw hashes and text copies of the reproduction drivers. Full poses,
logs and intermediate failed experiments remain in
`/home/zai/.cache/collisionAvoidance/restoration-fixes-20260930/`.
`campaign/` is the intermediate regression; `verified-campaign/` is the reported
final run. Generated binaries, media, prototype controller copies, nested solver
dependencies and unrelated user estimator/manuscript changes are excluded
from the project commit.
