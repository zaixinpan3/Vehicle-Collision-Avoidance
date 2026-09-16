# Recursive feasibility closure — September 15, 2026

The format-27 controller now has a complete conditional recursive-feasibility
argument covering an admitted active encounter, partial/full confirmed release,
prediction exhaustion, and continued target-free operation. The executable
successor construction is described and proved in
[`TERMINAL_CBF_PROOF.md`](../controller/TERMINAL_CBF_PROOF.md). This is a
mathematical guarantee for the declared affine/sensing/admission contract,
not a claim that every new target can be admitted or that execution meets
100 ms. The controller retains Predictive CBF / soft CLF and the existing
target high-gain observer framework.

## What was missing and what changed

The previous implementation inherited a feasible affine suffix only while
all original targets remained active. It started an independent fresh problem
after release and on target-free frames. It also used a zero-speed terminal
model while online prediction used a cruise generator. An invariant terminal
construction for a different generator did not supply a feasible candidate
to those actual successor problems.

The current implementation closes these gaps:

- The terminal set uses the **same admitted generator** as online prediction.
  Its robust modal inequalities retain different closed-loop contraction
  rates, rather than applying the slowest mode's rate to all uncertainty.
  Known continuous and sampled trim residuals are charged explicitly.
- Stable target identities label collision and exit rows. Confirmed release
  removes only those obligations, including partial release. The remaining
  finite witness and fixed exit deadline are inherited by affine substitution.
- A rebuilt no-target performance problem may replace the old family only
  after a concrete suffix-plus-continuation candidate passes all proposed
  affine rows and all six SOCs. Merely using it as an initial guess is not
  sufficient.
- If the finite prediction is exhausted, the same terminal certificate
  provides a feasible candidate to a **new, free one-hold optimization**.
  The terminal control law is never directly executed as a fallback. Domain,
  slip, actuator, current slew, next-entry slew and robust terminal successor
  constraints remain hard. CLF slack remains nonnegative and squared in cost.
- Positive approximate solver results may be executed only after independent
  hard-row, terminal-cone and reserved CLF verification. Exact objective
  optimality is not a premise of recursive feasibility. Infeasible, timed-out
  or iteration-limited solves still terminate execution. There is one native
  solve per frame, no retry and no saved-command execution.
- Verified physical reserves are transferred with the accepted plan. The
  successor does not repeatedly tighten the old constraints until its own
  feasible witness is excluded.

The terminal information state retains true-state modal membership together
with the measurement box. It does not require every later rectangular outer
hull to lie inside the modal set. The finite plan still propagates its full
open-loop uncertainty without assuming favorable future measurement resets.

An initial terminal rollout can extend the proposed target-free admission
horizon when the starting path error needs more recovery time. This affects
certificate search, not the performance objective or an early-recovery test.

## Assumptions and deliberately limited claims

The permanent reference is an analytically continued straight line or
constant-curvature curve. Current experiments have **no physical road
boundaries**, as requested. The lateral model domain remains ±4 m. Straight
reference samples and analytic arc length are not treated as end-of-road
barriers; projection and uncertainty conversion now agree with continued
reference poses. Circular station is unwrapped about the carried node.
Physical road boundaries, polyline heading jumps and varying curvature need
another permanent continuation certificate and are not silently covered.

The plant must follow the declared zero-residual affine hold, with unchanged
physical configuration and sample time. Current ego measurements contain
truth and have radii no greater than the stored sensing limit. That limit is
initialized at admission; curved-coordinate conversion uses a uniform bound
over the declared lateral strip. Enlarging it is a changed contract. Active
target motion and current complete observations must satisfy their declared
bounds and release semantics. Newly appearing targets require new admission;
no finite old witness proves safety against arbitrary new obstacles.

The full proof establishes nonemptiness of the next optimization, not finite
numerical solution time, nonlinear vehicle inclusion, global traffic safety
through arbitrary re-entry, a globally continuous scalar CBF under changing
obligations, or asymptotic tracking with arbitrary persistent CLF slack.

## Validation method

The final campaign is reproduced with:

```matlab
addpath('scripts');
summary = runRecursiveSafetyValidation( ...
    OutputDirectory="/absolute/path/to/results", SampleCount=300);
```

The driver invokes `runExactStateRecursiveFeasibilityScenario` for stationary,
oncoming, crossing, cruise, and uncertain crossing. Each unlimited-work trial
requests 300 holds of 0.1 s, reference speed 8 m/s, 16 m sensing range,
4.8 m × 1.9 m vehicle rectangles, 0.25 m clearance, ±4 m lateral domain,
0.4 rad heading domain and seed 20260912. Physical road boundaries are disabled.

Exact trials publish zero state/motion uncertainty. Uncertain crossing uses
ego component radii `[.01,.01,.001,.01,.01,.001]`, target component radii
`[.1,.1,.05,.05,.01,.01,.01,.01]`, Cartesian jerk amplitudes/bounds
`[.02,.02]` m/s³, zero yaw acceleration and sinusoidal frequency 1 rad/s.
The separate deadline trials request 60 holds with a 0.1 s **native-work**
budget; total frame time is measured separately.

Raw MAT/JSON traces, native-status diagnostics, test results and Code Analyzer
outputs are retained at
`/home/zai/.cache/collisionAvoidance/recursive-closure-20260915/`.
The final campaign is in `final-campaign/`. Generated binaries, third-party
solver trees, unrelated user files and intermediate source snapshots are not
part of the project commit.

## Numerical findings during implementation

The first single-ellipsoid prototype was too conservative to admit the existing
0.1 m ego-error fixture. The final modal certificate restores those tests;
measurement radii were not reduced to obtain a pass. A separate 40-hold local
invariant optimization test alternates errors at the declared 0.1 m / 0.01 rad
bounds, with finite steering/braking rates, and checks true-state modal
membership and actuator transitions after each optimized hold.

Strict rejection of every positive approximate solver status stopped the
stationary trial at 0.9 s, despite a hard-row residual of
`-9.75632412139604e-6` and a valid terminal certificate. The captured native
result had primal residual `2.1062079594023e-8`, dual residual
`3.74231606845572e-13` and status 4. The final implementation uses the verified
feasible optimizer decision, reports its approximate status, and preserves
its certified continuation. It does not claim exact optimality or execute a
backup input. Native optimality tolerance remains `1e-7`.

The one-ellipsoid intermediate implementation and strict-status-only variant
are not retained as executable controller branches.

## Final straight-scene results

| Scenario | Executed holds | Result | Sampled excess separation (m) | Final speed error (m/s) | Median / max frame (ms) | Frames above 100 ms |
| --- | ---: | --- | ---: | ---: | ---: | ---: |
| stationary | 300/300 | Completed | 0.007926241 | 0.000408979 | 94.238 / 1935.857 | 36 |
| oncoming | 26/300 | New-target admission failed at 2.6 s | 13.349043892 | 0.000408925 | 94.654 / 642.773 | 1 |
| crossing | 300/300 | Completed | 9.269714360 | 0.000408979 | 95.839 / 144.435 | 25 |
| cruise | 300/300 | Completed | N/A | 0.000408979 | 93.863 / 102.947 | 6 |
| uncertainCrossing | 300/300 | Completed | 9.269537173 | 0.000413878 | 93.828 / 149.439 | 5 |

Stationary avoidance retains 41 inherited frames and confirms release at
4.2 s. Its smallest sampled model-domain margin is 0.0000801501, terminal
cone margin is 0.04428055, and two approximate solutions pass independent
verification. Exact and uncertain crossing retain six inherited frames and
confirm release at 0.7 s. Every executed hold in this batch passes independent
hard-certificate verification. No terminal law directly issues an actuator
command; the ordinary scenarios obtain verified longer cruise horizons after
release and therefore do not need the local terminal optimization branch.
That branch is exercised separately by the invariant-set optimization tests.

Completed exact scenes end with lateral/heading errors below 1e-16 and speed
about 8.000409 m/s. Uncertain crossing ends with lateral error -0.000339505 m,
heading error -0.0000215854 rad and speed 8.000413878 m/s. These are numerical
simulation outcomes, not a general zero-slack convergence theorem.

The oncoming vehicle is first admitted as an active target at 2.6 s. The
new convex encounter problem is infeasible (native status 2), before any
active-encounter suffix has been accepted. Its 26 preceding no-target holds
are certified. This is an unresolved **new-obligation admission** failure,
not loss of the unchanged old witness. It does not establish that every
possible physical maneuver is infeasible.

### Native 100 ms work-budget trials

| Scenario | Executed holds | Outcome | Maximum total frame (ms) | Frames above 100 ms |
| --- | ---: | --- | ---: | ---: |
| stationary | 0/60 | Native timeout at 0.0 s | 402.981 | 1 |
| oncoming | 26/60 | Native timeout at 2.6 s | 259.183 | 1 |
| crossing | 60/60 | Completed | 147.152 | 6 |

The full every-frame 100 ms requirement is **not met**. In the unlimited-work
stationary trial the maximum successor frame is 1810.098 ms. Even completed
crossing has over-budget frames. These measurements include input assembly
and the controller, but exclude simulated plant integration, offline auditing
and a target estimator. Measured compute delays are not inserted into plant
execution; all mathematical holds remain 0.1 s. Consequently this campaign
is not a real-time physical safety demonstration. No new joint estimator /
controller campaign is claimed while these controller-only limits remain.


## Final implementation checks

A clean full run in MATLAB R2026a Update 3 completed **593/593 tests** with
zero failures and zero incomplete cases:

```matlab
results = runtests('tests');
assertSuccess(results);
```

The new `recursiveSafetyClosureTest` contributes 17 parameterized cases.
They cover straight/curved modal-set support, free terminal optimization,
partial release with reordered target records, continued operation beyond
the original horizon and reference samples, circular station unwrapping,
false numerical success, independently certified approximate solutions,
changed measurement bounds, and 40 optimized holds with alternating bounded
measurement errors. The existing observer tests also pass; observer sources
were not changed.

The final focused run passed 90/90 affected cases. Earlier full runs exposed
uncertainty conservatism, reference/wrapping behavior, and two error-identifier
regressions; those were fixed before the clean 593-case run. Factory Code
Analyzer checks on all 11 changed MATLAB files report zero findings. The core
controller source-budget test still passes with 20 files. `git diff --check`
also passes. Machine-readable check counts are in `validation.json`, and the
clean results are in `clean-all-tests.mat` under the artifact directory above.

The conditional recursion proof and its executable successor construction are
complete for the stated scope. Remaining experimental limitations are the
oncoming target's first admission, every-frame timing, and the unvalidated
transfer from the declared affine plant to nonlinear physical vehicle motion.
