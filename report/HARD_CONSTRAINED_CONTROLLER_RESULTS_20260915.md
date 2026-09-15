# Hard-constrained controller without a runtime checker

Experiment date: September 15, 2026, America/Chicago. Baseline implementation:
`f410e595784f4051d22581aeabd626c3d5f1cc2d`. This report describes the
format-24 source committed with it; earlier reports retain their original
source and experiment scope.

## Implemented behavior

The controller now executes a newly optimized plan from strict successful
solver status. Every swept collision/road, domain/slip, actuator/slew, finite
exit and road-terminal condition enters the optimizer as a hard constraint.
There is no runtime `check.value==0` gate, independent plan verifier,
terminal membership recheck, omitted-row generation or post-solve repair.
A failed solve uses the existing feasible suffix; an unadmitted encounter
cannot borrow a road-only witness for its new target obligations.

The real-time branch optimizes two input-adjustment coordinates over a
bounded rollout family. All remaining planned inputs and states are affine
in those coordinates. Scalar constraint reduction accounts for tiny cross
terms through explicit decision bounds; it never discards them without a
worst-case support allowance. This preserves the full swept constraints without
introducing one input pair per horizon stage. The first clear-road hold also
has a hard second-order cone limiting deviation from a contracting sampled
cruise controller. It yields `V_next <= c*V+b`; uncertainty and numerical
allowances determine b. The full proof and its numerical premises are in
[the design note](../controller/SAMPLED_BACKUP_CBF_CLF.md).

The full-horizon branch uses one hard-constrained SOCP. Its performance CLF
remains soft. The default soft-CLF weight changes from 100 to 0.01 after direct
experiments found numerical stagnation with the larger penalty during
passing. This change does not relax a physical safety condition or the
backup branch's hard cruise cone. Native optimality tolerance applies once
to the normalized objective; the separate primal tolerance remains 1e-8.

Legacy checker/row-generation configuration values are accepted and ignored.
Offline audit utilities remain available to tests and research scripts.
Uncomputed margins are NaN, and metadata identifies solver-based admission
and `postSolveCertificationPerformed=false` explicitly. Conditioning and
sensor/execution contracts remain part of the information-state algorithm.

## Validation method

Original artifacts are retained outside the source tree under
`/home/zai/.cache/collisionAvoidance/hard-constraints-no-checker-20260914`.
The directory name reflects when work started; it does not backdate the
September 15 validation. MATLAB R2026a runs the existing experiment drivers
with 0.1 s holds and seed 20260914. Controller/configuration sources still
total 20 files. External solver trees and generated binaries are excluded
from the project commit.

The sequential timing campaign requests a cold stationary run, exact and
bounded stationary/oncoming/crossing runs, exact and actual-NRMM visibility
tests, a 900-hold exact visibility run, and bounded crossing with every
post-admission solve disabled. No test job runs concurrently with the timing
campaign. The experiment independently evaluates true geometry, model-domain
membership, reachable-box containment and executed sampled CLF differences.
Measured runtime is not applied as simulated actuation delay.

The basic scenarios request 8 m/s and a 16 m confirmation radius. The ego
starts at (0,0), heading zero. Target initial position/velocity pairs are
stationary `(15,0)/(0,0)`, oncoming `(60,0)/(-8,0)`, and crossing
`(15,-4)/(0,32)`, in SI units. Straight road boundaries are at lateral
positions -5 and 5 m; the configured clearance is 0.25 m.

Bounded runs use ego measurement radii
`[.05,.05,.005,.05,.02,.005]` for `[px,py,psi,vx,vy,r]`, target radii
`[.1,.1,.1,.1,.05,.05,.01,.01]` for `[px,py,vx,vy,ax,ay,psi,omega]`,
jerk amplitudes `[.1,.1]` m/s^3 and yaw acceleration amplitude .05 rad/s^2
at frequency 1 rad/s. No artificial uncertainty cap is introduced.
The visibility experiment requests 10 m/s, a 30 m range, and a target
starting at (100,.8) m with velocity (-10,0) m/s. The actual-NRMM run
retains the driver's sensor/observer configuration.

## Diagnosed implementation failures

The first complete regression run had nine failed cases out of 721. Several
were obsolete expectations about independently measured margins and two
lexicographic solves. Real failures included full-horizon numerical
stagnation. Normalized objective tolerances, a smaller soft-CLF penalty and
more time for the first passing family resolved those cases. Transcription
comparison tests request enough objective accuracy to compare their optima;
physical feasibility tolerance is unchanged.

A later 724-case run isolated three recovery failures after enforcing the
native frame deadline. Infeasible or expensive cruise attempts repeatedly
consumed the budget and the controller stayed on its safe braking witness.
Allocating time to recovery alone was insufficient. Profiling found about
30,000 two-variable rows, many distinguished only by cross-coefficients
near 1e-17 relative to their dominant coefficient. Conservative scalar-bound
reduction lowered a captured recovery program from 30,197 to 1,356 native
rows, and its solve from 71.535 to 1.794 ms. This comparison is a diagnostic
measurement, not a complete-frame timing claim. The final regression and
campaign outcomes follow below.

## Final sequential results

| Trial | Executed / requested holds | Median frame (ms) | Maximum frame (ms) | Frames above 100 ms | Outcome |
| --- | ---: | ---: | ---: | ---: | --- |
| exact-stationary | 300 / 300 | 3.385 | 112.076 | 1 / 301 | Completes and returns to cruise |
| exact-oncoming | 300 / 300 | 3.307 | 122.841 | 1 / 301 | Completes and returns to cruise |
| exact-crossing | 300 / 300 | 3.405 | 59.502 | 0 / 301 | Completes and returns to cruise |
| bounded-stationary | 0 / 300 | 143.132 | 143.132 | 1 / 1 | No encounter admitted |
| bounded-oncoming | 0 / 300 | 150.550 | 150.550 | 1 / 1 | No encounter admitted |
| bounded-crossing | 300 / 300 | 3.798 | 55.596 | 0 / 301 | Completes and returns to cruise |
| range-exact-oncoming | 300 / 300 | 2.919 | 66.381 | 0 / 301 | Completes and returns to cruise |
| range-nrmm-oncoming | 35 / 300 | 14.079 | 193.652 | 2 / 36 | First target admission fails at 3.5 s |

Exact stationary/oncoming/crossing each pass the driver's independent
geometry, model-domain and true-state containment audits. Their minimum
separation margins beyond configured clearance are 0.153430, 1.039595 and
9.269716 m, and road margins are 0.559733, 0.562679 and 3.800000 m.
The first two overrun 100 ms at admission; all successor frames in these
three runs stay below 60 ms. There are no terminal stopping commands in
these completed ordinary trials. Bounded crossing also passes, finishing
with 4.13 mm lateral error and 0.0000694 m/s speed error.

A separate fresh-process stationary trial completes all 300 holds but takes
**1,382.885 ms at admission**. Its median frame is 3.445 ms. Cold initialization
is not real-time, even though subsequent admitted operation is much faster.

Bounded stationary and oncoming each reject both attempted hard programs
with native infeasible status and issue no command. This is infeasibility of
the proposed convex families, not proof of an inevitable collision or global
control infeasibility. Actual NRMM completes 35 target-free holds and fails
its first target admission at 3.5 s. Its positive prefix margins do not
establish encounter safety. The limited rollout family, conservative
open-loop uncertainty and finite robust exit remain admission limitations;
feedback-aware robust tubes would require a separate design and proof.

With every post-admission optimizer call forced to fail, bounded crossing
still completes 300 safe holds using the carried sequence and invariant
terminal law. It finishes at 0.000001676 m/s. Maximum/median frame time is
97.274/46.599 ms, with no 100 ms miss. This test establishes continuation
through solver failure, not cruise recovery while the solver is unavailable.

## Ninety-second constant-speed path cruise

The exact 30 m visibility trial completes 900 holds. During 80--90 s its
speed stays at 9.9999999545 m/s and lateral/heading errors are at numerical
precision in the declared affine model. Minimum separation and road margins
are 0.683082 and 0.545434 m beyond the configured clearance. All **837**
executed holds marked as cruise-constrained satisfy the independently
evaluated `V_next-c*V-b<=0` inequality. Maximum residual is
-5.08654e-9, minimum decay fraction per hold is 0.0736260, and maximum
numerical disturbance bound b is 5.08654e-9.

Median/maximum complete-frame timing is **3.464/76.430 ms**, with **0/901**
frames above 100 ms. The extra frame is the final unexecuted decision.
The exported `long-cruise.png` and `long-cruise.pdf` in the cache show speed,
lateral recovery and frame timing, with the active encounter shaded. They
were generated with standard plotting tools and visually inspected.

These are measured results on this workstation; they are not a certified
worst-case execution-time bound or a delayed-actuation experiment.

## Regression and source audits

The final focused run passes **116/116** cases, including all three recovery
cases that failed under the first deadline allocation. The latest result
per name across the 724-case full run and this focused run is **725/725
passed**, zero failed or incomplete. This is an aggregate, not one fresh
725-case full-suite run. Original failed runs remain in the cache, alongside
`aggregate-tests.csv` and `test-counts.json`.

Factory Code Analyzer was run on all 16 modified MATLAB files. Fifteen have
zero findings; `avoidanceStageQp.m` retains one existing sparse-indexing
performance advisory in an unchanged statement. No new analyzer finding
remains. The online-only 12-hold crossing profile records seven constrained
backup calls and twelve feasibility transfers, with zero calls to
`verifyFixed`, `verifyCandidate`, `terminalMembership`, `certify` or
`certifyInputs`. Source inspection confirms both execution policies use
solver status and feasible witness transfer. `git diff --check` passes.

The requested hard-constraint/no-runtime-checker implementation is complete.
Its safety theorem remains conditional on the declared plant, sensing,
execution and numerical solver contracts. Admission latency, the failed
uncertain encounters, arbitrary-curvature cruise and nonlinear vehicle
inclusion remain unresolved. The complete uncertain, physical, hard-real-time
pipeline is therefore not established by these experiments.
