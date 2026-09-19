# Predictive continuation restoration — September 15, 2026

The prediction/continuation architecture is restored. Removing fallback
execution did not justify deleting the prediction horizon, terminal certificate,
or carried witness. The previous one-hold truncation was an architectural
regression. The restored format-26 implementation still stops immediately on
an unsuccessful optimization; a terminal law or old command is never dispatched.

This completes the structural restoration. It does **not** establish that every
straight scenario is feasible or that the controller meets the 100 ms frame
requirement. The final experiments below retain both limitations explicitly.

## Implemented behavior

- The configured horizon is honored: 16 holds by default, extended when finite
  encounter completion needs more stages. The stationary trial admits 48 holds.
- The full input sequence participates in the objective and hard swept
  rectangle, chart/model-domain, actuator, slew, finite-exit and invariant road
  terminal constraints. The circumdisk sampled-decay condition is removed.
- The first-hold sampled CLF retains its nonnegative norm slack and squared
  penalty. Input effort remains centered at the CLF/Riccati operating input.
  The observer and its high-gain mathematical framework are unchanged.
- The fourth output stores the plan, exact node boxes, Bernstein cells, frozen
  affine generators, charts/normals, terminal set and absolute exit deadline.
- During an unchanged active encounter, eliminate the executed input from the
  accepted affine constraint family. Preserve its enclosures and numerical
  reserves. The previous suffix is therefore a feasible candidate in the
  actual next optimization, rather than merely a stored initialization.
- Current ego/target measurements condition the carried reachable sets. Release
  requires current complete perception and confirmed exterior membership or
  consistent absence. Confirmed release restarts a cruise optimization window;
  it never starts terminal-policy execution.
- Translate optimization coordinates about the carried/trim input plan. This
  equivalent numerical change resolved an intermediate strict-solver-status
  failure without accepting an incomplete solve or relaxing a safety row.
- Publish exact affine successor nodes. An intermediate use of polynomial
  node approximations caused spurious measurement-intersection failures and
  was corrected before the final validation.

The implementation uses the existing controller modules. No historical
executable variant or alternate controller is added. The current algorithm,
CLF equations and guarantee limits are documented in
[the controller contract](https://github.com/zaixinpan3/Vehicle-Collision-Avoidance/blob/8529584774ffd17b2cae7d2fb18aac219b2fde13/controller/SINGLE_SOLVE_CBF_CLF.md).

## Feasibility argument and remaining scope

If the accepted family is $[A_0\ A_+]U\le b$, execution of $u_0^*$ gives
$A_+U^+\le b-A_0u_0^*$. Its accepted suffix remains feasible by substitution.
Retaining the original enclosures avoids an unjustified assumption that a
newly relinearized/anchored problem contains that suffix. A finite nonnegative
CLF slack can accommodate any finite hard-feasible input. The absolute exit
deadline is retained, and the invariant road terminal set remains part of the
accepted finite witness.

This is conditional mathematical successor feasibility during an unchanged
admitted active encounter. It assumes valid execution, model bounds, information
set inclusion and the confirmation contract. It does not guarantee numerical
solver completion. New target admission, increased bounds, partial target
release and fresh post-release cruise convexification are separate transitions;
the current implementation does not prove that all those fresh problems contain
the old road witness. The full online `recursiveFeasibilityGuaranteed` flag
therefore remains false; `inheritedFeasibleFamily` identifies the proved
within-encounter construction. A complete global CBF proof is not claimed.

The role of an invariant terminal set and a shifted feasible sequence is
consistent with equation (8), Lemma III.1 and equation (10) of
[Huang, Wang, Margellos and Goulart, *Predictive Control Barrier Functions:
Bridging model predictive control and control barrier functions*](https://arxiv.org/html/2502.08400v2).
Their result is not a proof of every sensing, fresh-admission or numerical
transition in this implementation.

## Final straight experiments

Driver: `scripts/runExactStateRecursiveFeasibilityScenario.m`. MATLAB R2026a
Update 3, Linux, native Clarabel bridge. Seed 20260912; 0.1 s hold; cruise
reference 8 m/s; zero initial ego/target error bounds and zero target jerk/yaw
acceleration. Physical road boundaries are disabled. The lateral model domain
remains $|d|\le4$ m and heading domain $|e_\psi|\le0.4$ rad. The complete
perception region has radius 16 m. Vehicle rectangles are 4.8 by 1.9 m, with
0.25 m required separation.

The stationary target starts at (15,0) m. The oncoming target starts at (60,0)
m with velocity (-8,0) m/s. The crossing target starts at (15,-4) m with velocity
(0,32) m/s. The driver publishes exact target observations; exterior objects are
excluded by the controller's current observation guard. Positive collision
margins below mean separation in excess of the required 0.25 m.

The unlimited-work runs evaluate 300 holds (30 s requested). They isolate
trajectory/continuation behavior from solver timeouts; they are **not real-time
execution demonstrations**.

| Scenario | Executed / requested holds | Result | Minimum sampled separation margin (m) | Final speed (m/s) | Maximum total frame (ms) |
| --- | --- | --- | ---: | ---: | ---: |
| Stationary | 300 / 300 | Avoidance and observed cruise recovery | 0.00792623 | 8.00036293 | 1971.148 |
| Oncoming | 26 / 300 | First active-target admission fails at 2.6 s; strict infeasible status | 13.34914244 over executed prefix | 8.00036292 at failure | 735.910 |
| Crossing | 300 / 300 | Encounter release and observed cruise recovery | 9.26972923 | 8.00036293 | 147.671 |

Stationary release is observed at 4.2 s; crossing release at 0.7 s. There are
41 and 6 inherited optimization frames respectively. Maximum independently
evaluated inherited physical-row residuals are -9.5164e-6 and -0.0456107.
Minimum terminal membership margins over those runs are 0.0314203 and
0.0456074 in the rows' native units. No terminal-policy commands execute.
At 30 s, both completed cases have lateral and heading errors below 1e-16
and speed error about 0.000363 m/s. These are finite-run observations, not a
proof that a positive-slack controller has zero asymptotic tracking error.
Minimum sampled model-domain margins are positive: 8.0150e-5 for stationary
and 0.399884 for crossing. The slack-dependent sampled CLF audit passes.

The oncoming run remains target-free until the target enters the admitted
perception region. It fails when a new avoidance certificate is required.
The inherited-suffix theorem cannot guarantee feasibility of that first
admission. This failure does not establish that every physically possible
oncoming avoidance maneuver is infeasible.

A separate run retains the 0.1 s **native-work budget** and requests 60 holds:

| Scenario | Executed / requested holds | Result | Maximum total frame (ms) | Frames above 100 ms |
| --- | --- | --- | ---: | ---: |
| Stationary | 0 / 60 | Native timeout at initial admission; no command | 399.679 | 1 |
| Oncoming | 26 / 60 | Native timeout at active admission, 2.6 s | 264.450 | 1 |
| Crossing | 60 / 60 | Completes; initial total frame exceeds budget | 149.345 | 1 |

A native solve budget is not a bound on complete pipeline time. The stationary
unlimited-work run also has a maximum successor frame of 1846.092 ms, so its
problem is not limited to cold startup. Its median is 94.4365 ms, with 38 frames
above 100 ms. The controller is not qualified for the requested every-frame
100 ms requirement. Timing excludes plant integration and offline audits and
is not applied as actuation delay; the declared affine plant executes the
issued hold immediately in this diagnostic setup.

Because controller-only admission and real-time requirements remain unmet,
no new estimator-controller campaign is run in this restoration task. Earlier
joint results must not be represented as validation of format 26.

## Validation and reproducibility

- Initial full `runtests('tests')`: 573 tests, 570 passed. Two failed tests still
  assumed unconditioned single-hold target data; the third asserted recovery
  within the short 2.4 s crossing check. The target tests now exercise current
  in-range observations and intersection with the carried prediction. The
  short crossing check tests one solve per frame and absence of terminal
  execution. The separate long cruise-recovery test remains.
- Final targeted run of `collisionAvoidanceControllerTest`,
  `controllerEstimatorBoundsTest` and `encounterCertificateScenarioTest`:
  **57 / 57 passed**. This includes exact/uncertain inherited feasibility,
  100 hypothetical terminal-law steps from accepted exact/uncertain terminal
  boxes, input and entry-slew bounds, malformed/failure statuses, no fallback,
  confirmed release, multiple targets and long cruise recovery.
- Current test inventory: **576 cases**, all covered by passing results from
  the full run plus the final targeted rerun. This is combined coverage, not a
  claim that a second complete 576-case run was performed.
- Factory Code Analyzer: zero final findings in nine changed MATLAB files.
  An unused helper was removed after the first analyzer pass.
- `git diff --check` passes. Solver dependencies, generated binaries,
  unrelated user files and historical source snapshots are excluded from the
  project commit.

Run the final unrestricted cases from the repository root:

```matlab
addpath('scripts');
runExactStateRecursiveFeasibilityScenario(Scenario="stationary", ...
    SampleCount=300, DeadlineSeconds=Inf, OutputDirectory="/tmp/continuation-stationary");
```

Use `Scenario="oncoming"` or `"crossing"` for the other cases. For the budgeted
runs, set `SampleCount=60` and `DeadlineSeconds=0.1`. A failed run saves its
executed prefix and rethrows; it does not execute an old input to continue.

Actual outputs remain under
`/home/zai/.cache/collisionAvoidance/restore-continuation-20260915/`:
`final/unlimited/`, `final/deadline100ms/`, `final/summary.json`,
`final/validation.json`, `all-tests.mat`, and `final-focused-tests.mat`.
The report is stored in `report/`; executable drivers remain in `scripts/`.
