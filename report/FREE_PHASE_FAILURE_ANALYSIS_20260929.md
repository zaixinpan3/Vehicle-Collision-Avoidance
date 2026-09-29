# Remaining failures after freeing terminal longitudinal phase

Date: September 29, 2026. Controller revision:
`8d35249675bc53020d2c374397c3a958f60b9b82`.
Production controller, configuration and tests are unchanged by this analysis.

The [free-phase campaign](FREE_TERMINAL_PHASE_20260929.md) had five completed
runs and nine initialization rejections. Instrumented replay now distinguishes
slow feasible search, a stalled restoration process, conservative terminal
admission, and a separate intersample safety defect. Removing prescribed
terminal progress did not remove these other mechanisms.

## Results by failed fixture

| Fixture | Original five-second rejection | Extended diagnostic result | Finding |
| --- | --- | --- | --- |
| Braking lead, 8 m/s | Completion collision residual remains | Nominally feasible in 9.680079 s, 20 conic calls | Slow escape from a colliding cruise seed; dense replay gap 9.948062 mm |
| Braking lead, 15 m/s | Completion collision and terminal residual remain | Nominally feasible in 5.904684 s, 12 calls | Same mechanism; dense replay gap 52.758229 mm |
| Crossing, 15 m/s | Best checked candidate clears collision samples but misses the tight terminal norm | Nominally feasible in 7.458109 s, 21 calls | Slow endpoint repair; returned plan subsequently collides between checks |
| Turning crossing on straight road, 8 m/s | Collision/endpoint/future-separation restoration incomplete | 48 calls; no solution after 21.261210 s | Better primary candidate discarded; negative terminal-separation anchor plus shrinking trust blocks restoration |
| Turning crossing on straight road, 15 m/s | Same restoration process stalls | 48 calls; no solution after 11.650025 s | Coupled endpoint/physical/trust constraints block restoration; more time alone does not help |
| Curved head-on, 8 and 15 m/s | Terminal admission rejects before optimization | Zero conic calls | Entire indefinite ego/target circular regions overlap |
| Curved crossing, 8 and 15 m/s | Terminal admission rejects before optimization | Zero conic calls | Sufficient whole-orbit separation rejects timing-dependent possibilities |

Times above are **instrumented initialization measurements**, not running-frame
performance. A larger diagnostic budget does not make these searches real time.
The two turning-crossing extended runs end with `safetySolveFailed` at the outer
iteration limit, rather than exhausting their 30 s budgets. Repeated event
tracing reproduces the same candidate values and 48-call stall, with elapsed
times 22.645 s and 11.913 s. Timing varies with instrumentation and load.

The returned plans were audited open loop over their complete optimized input
sequences. This investigation does not rerun the fourteen-scenario closed-loop
campaign and does not revise its 5/14 completion count.

## 1. A colliding initialization creates infeasible local collision constraints

`localSeed` still rolls out lane feedback through the encounter. It checks
the eventual terminal continuation but does not initialize a finite avoidance
path. Current `dualLinearization` selects a separating face from this rollout;
that face stays fixed within each convex subproblem and changes only when the
nonlinear anchor changes. There is no flow-field initializer on this code path.

At the 1.6 s crossing rendezvous, the selected normal is `[0;1]`. The local
collision rows require at least 3.356 m lateral center displacement for the
perpendicular crossing and 3.400481 m for the turning target. The default
trust radius permits only 2.5 m lateral displacement from the cruise seed.
Single-node LPs allowing yaw perturbation still require collision relaxation
of 0.856000 m and 0.900481 m, respectively. At radius 0.25, the deficits grow
to 2.106000 m and 2.150481 m. Road constraints are absent from these LPs.

The full first conic problem is infeasible for all five straight-road failed
fixtures. Removing collision rows makes each feasible. Removing the terminal
cone alone makes the two braking-lead programs feasible, but leaves all three
crossing programs infeasible. Removing just the future-separation row does
not fix any of the five. Doubling the trust radius does not fix any complete
first program; combining doubled trust with no terminal cone only fixes the
lead cases. Thus the initial crossing failure is not solely a terminal-set
failure, nor solely the one-node trust deficit. Coupled dynamics and separating
directions over the trajectory also matter.

These row-removal controls diagnose local subproblems; they are not admissible
controller plans. Numerical linear residuals for feasible controls are below
`2.81e-8`, with successful conic exit flags. They do not prove global nonlinear
infeasibility when the full program is rejected.

## 2. Braking-lead failures are slow restoration, not impossibility

Both lead seeds have a maximum collision deficit of 1.906 m. In the 8 m/s
case, the first large restoration steps yield nonlinear hard residuals
14.3130, 2.02334 and 1.91670, all worse than the seed, and are rejected.
The trust radius shrinks from 0.5 to 0.0625 before a slight improvement is
accepted. At five seconds the best hard residual is still 1.899078 m,
entirely from completion collision. The terminal norm already passes.
Subsequent small steps eventually grow and produce a feasible overtaking
continuation at 9.680079 s with phase +4.136650 m.

At 15 m/s the best five-second candidate has a 0.334066 m completion collision
deficit and 0.066682 weighted terminal-norm excess. The extended run accepts
a phase +3.289930 m at 5.904684 s. Both returned sequences have positive dense
open-loop body separation. These outcomes rule out global infeasibility of
their current nominal terminal-position/time problem. They identify inefficient
initialization, inaccurate large local steps and slow restoration as the
observed reason for the original budget failures.

## 3. Tight endpoint correction slows the 15 m/s crossing

The best candidate checked within five seconds has zero completion collision
and physical-limit residuals, a 17.310809 mm predicted collision margin beyond
the 6 mm buffer, and ample future-separation margin. Its only positive hard
component is a weighted terminal-norm excess of 0.123262. Terminal coordinate
errors include 11.898 mm lateral displacement, 4.144 mm longitudinal error
relative to its selected phase, and small heading/speed errors. These are
not a failure to reach a prescribed old longitudinal position: the phase has
already changed to -1.531268 m.

The certified terminal norm radius is only `0.0009765625` at 15 m/s and
`0.0001220703125` at 8 m/s. These are **weighted norm radii, not distances in
meters**. Free phase retains the tight conditions on the other coordinates
and input memory. The extended crossing run performs 240 logged nonlinear
evaluations, including repeated suffix corrections, before accepting a plan.
This is feasibility work, not cost improvement after an accepted feasible plan.
Its later intersample collision is described below.

## 4. Turning-crossing restoration can trap itself

The straight-road turning-target certificate encloses the target's entire
future orbit. At the selected completion time, the terminal cruise reference
must be sufficiently ahead of that region. Free phase therefore has a
safety-derived lower bound in these fixtures:

| Speed | Endpoint time | Minimum admitted phase at that time |
| ---: | ---: | ---: |
| 8 m/s | 10.05 s | -0.015165 m |
| 15 m/s | 6.15 s | -0.664788 m |

These are not explicit progress targets added to the optimization. They are
consequences of the existing sufficient policy assumption: after the endpoint,
continue cruising indefinitely without needing another avoidance maneuver.
They sharply limit how far behind the seed the vehicle may finish, even
though phase itself is an unconstrained scalar before safety is imposed.

Endpoint polishing minimizes terminal error but does not preserve future
separation during intermediate search steps. Acceptance of an **iterate**
uses a combined residual improvement; only a final executable witness must
have zero hard residual. Consequently an improved but future-unsafe phase
can become the new linearization anchor. In the stalled runs, these anchors
are phase -0.810106 m with future-separation deficit 0.794941 m at 8 m/s, and
phase -1.016602 m with deficit 0.351814 m at 15 m/s.

Restoration relaxes finite completion collision rows, but keeps the terminal
cone, future-separation row, physical limits and trust bounds hard. It cannot
necessarily move an invalid anchor back into this hard intersection. At
8 m/s, the first repeated infeasibility occurs at radius 0.1220703125: endpoint
position may change by only 0.610352 m, less than the approximately 0.795 m
phase correction demanded by future separation. Shrinking further aggravates
this conflict.

Controlled re-solves of the actual stalled restoration programs confirm:

| Diagnostic change | 8 m/s | 15 m/s |
| --- | --- | --- |
| Original stalled restoration | Infeasible | Infeasible |
| Remove future-separation row only | Feasible | Infeasible |
| Remove terminal cone only | Feasible | Feasible |
| Restore trust radius to 1.0 | Feasible | Feasible |

The 15 m/s stall has a more strongly coupled terminal/trust obstruction; it
cannot be attributed to the future-separation row alone. A diagnostic restart
from its best checked candidate with the usual initial trust radius still
fails after 48 conic calls. This analysis does not establish that its original
nonlinear problem is globally infeasible.

### A better primary candidate is discarded in the 8 m/s case

Event tracing identifies a separate, concrete loss of progress. In iteration
5, the polished primary candidate has hard residual 0.069888, but the secondary
lane-cost solve returns a candidate with residual 23.473857. In iteration 6,
the corresponding values are **0.011389** and **3.677759**. Because the primary
candidate is not completely feasible, the code does not return it to the
outer iteration. The secondary candidates are rejected, and the anchor stays
at residual 0.794941 rather than advancing to either better primary candidate.

The best primary candidate already satisfies collision samples, input/state
limits and future separation; its remaining defect is the tight terminal norm.
A diagnostic restart from that candidate, preserving the same endpoint index,
phase family and all constraints, reaches nominal feasibility in **one conic
call and 0.661178 s**. Tight open-loop replay has a minimum body gap of
259.109765 mm. This isolates a candidate-retention defect; it does not justify
executing the earlier infeasible candidate. Its input/phase trajectory should
have remained available to guide restoration instead of being discarded.

## 5. Four curved cases fail before optimization

The seed checks 445 endpoints at 8 m/s and 437 at 15 m/s, ending at 25.6 s.
Membership passes, but every future-separation test fails. There are zero
conic calls and zero nonlinear candidate evaluations in these four cases.

Ego's terminal reference has radius 200 m. The curved head-on target occupies
an annulus of radii 199.043600--200.983408 m about the same center; the
separation bound is approximately -3.5436 m. The curved-crossing target has
annulus radii 39.028664--41.123662 m and a center about 203.962873 m from ego's
orbit center; the bound is approximately -39.748 m. These numbers are
conservative certificate margins, not actual penetration measurements.

Changing phase rotates ego around the same circle and cannot make the whole
circle disjoint from those regions. The all-future spatial test ignores
relative passage timing. For head-on motion on the same circle, indefinite
opposite-direction cruise also creates repeated future encounters, so an
assumption of permanently returning to that cruise policy is substantively
restrictive. The current certificate rejects this terminal family, not every
possible future avoidance strategy. More optimization time cannot help a
case that is rejected before optimization begins.

## 6. The extended-budget crossing plan collides between constraint points

The 15 m/s crossing plan accepted at 7.458109 s has zero nominal hard residual
and zero prefix slack. Its minimum nominal sampled body gap is 6.238414 mm.
Tight open-loop integration evaluated at the same nodes and midpoints gives
6.238509 mm, also above the configured 6 mm buffer.

At 600 Hz replay, however, twelve samples show strict rectangle overlap from
1.476667 to 1.495000 s, with maximum separating-axis penetration **101.622874 mm**.
The collision is between the 25 ms-spaced constraint checks. It is not caused
by accepting a violation of the 6 mm margin at those checks. A larger search
budget cures its initialization rejection but does not make its returned
trajectory collision-free.

Independent Python target propagation, rectangle distance and separating-axis
overlap agree with MATLAB geometry for all four replays. The other three
replays have positive sampled gaps. The restarted 8 m/s turning-crossing
endpoint has a small positive ODE terminal membership value `1.3531e-8` even
though nominal membership passes; its very small endpoint neighborhood is
sensitive to integration mismatch. Neither these replays nor nominal closure
tests establish a robust or continuous-time recursive-feasibility theorem.

## Recommended priorities within the single controller

1. Retain the best nonlinear primary/restoration candidate as a possible next
   anchor, even when a later secondary candidate is worse. Never execute it
   until all original hard constraints pass.
2. Prevent restoration from becoming trapped outside hard terminal admission:
   account for phase separation during endpoint correction, and restore
   reachability of the hard endpoint/trust intersection rather than blindly
   shrinking trust after infeasibility.
3. Initialize dynamics and separating directions around an obstacle-aware
   trajectory, including yielding where suitable, instead of starting from
   the known colliding cruise trajectory. Improve the conservative terminal
   construction without replacing its certificate with an unsupported tolerance.
4. Replace the whole-orbit terminal restriction with a demonstrably safe
   continuation family capable of the desired future encounters; preserve the
   retained-witness argument. Simply deleting terminal safety is not a remedy.
5. Validate or constrain intersample motion before admitting an executable
   plan. The measured 101.6 mm crossing penetration shows why a 6 mm sample
   buffer alone cannot establish collision freedom for these fixtures.

These are diagnosed changes to pursue, not production changes made in this
report-only task. The separate intersample defect must be addressed before
claiming reliable collision avoidance for all returned plans.

## Methods, artifacts and reproduction

The nine failed fixtures were replayed with the original five-second budget;
the five straight-road cases were also given 30 seconds. Additional controls
include 15 single-node LPs, 30 first-conic ablations, eight stalled-restoration
ablations, two event-traced runs, two diagnostic restarts and four dense open-loop
replays. The fourteen primary diagnostic calls record 685 nonlinear evaluations
and 107 outer iterations. Configuration remains deterministic with no random
draws: 8/15 m/s reference speed, 50 ms hold, 8/16 prefix holds, 512-hold maximum
horizon, no road bounds, 6 mm sampled buffer, and exact state/target inputs.
Replay uses `ode45` with relative/absolute tolerances `1e-11/1e-12`, 31 samples
per hold, propagating the ODE successor through the full input sequence.

Compact results, CSVs, instrumentation patches, source/raw hashes and scripts
are in [`FREE_PHASE_FAILURE_ANALYSIS_20260929/`](FREE_PHASE_FAILURE_ANALYSIS_20260929/).
Full MAT snapshots and replay poses remain at
`/home/zai/.cache/collisionAvoidance/free-phase-failure-analysis-20260929/`.
Production-source hashes were verified unchanged. No production test suite was
rerun for this report-only task; the diagnostic experiments are its validation.
Two diagnostic-driver issues involving heterogeneous struct concatenation and
MATLAB empty-string testing were corrected in the cache; the original failed
and partial logs are preserved, and the final analyzer requires all four replays.

To reproduce, copy the `.txt` driver records into the cache directory without
that suffix and run from the repository root at the stated revision:

```bash
python /home/zai/.cache/collisionAvoidance/free-phase-failure-analysis-20260929/prepare.py
matlab -batch "run('/home/zai/.cache/collisionAvoidance/free-phase-failure-analysis-20260929/diagnose.m')"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/free-phase-failure-analysis-20260929/geometryControls.m'); run('/home/zai/.cache/collisionAvoidance/free-phase-failure-analysis-20260929/affineControls.m')"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/free-phase-failure-analysis-20260929/eventDiagnosis.m'); run('/home/zai/.cache/collisionAvoidance/free-phase-failure-analysis-20260929/restartControls.m')"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/free-phase-failure-analysis-20260929/stalledControls.m'); run('/home/zai/.cache/collisionAvoidance/free-phase-failure-analysis-20260929/replayReturned.m')"
python /home/zai/.cache/collisionAvoidance/free-phase-failure-analysis-20260929/analyze.py
```

The final replay driver was also run separately after correcting its empty-string
test. Source copies and helpers are generated only in the cache; no historical
controller copy, media or generated binary is committed.
