# Finite convex avoidance branches: implementation and validation

Date: September 16, 2026. Scope: exact-state controller admission and recursive continuation in the declared held-affine plant. The high-gain target observer is unchanged. This report does not claim a new successful joint estimator-controller campaign or nonlinear physical-vehicle guarantees.

## Final experiment results

**All three 30 s diagnostic controller trials complete safely and return near cruise. The 100 ms requirement is not met.** The original oncoming failure at t=2.6 s is now admitted by a computed finite convex branch, without a prescribed passing path. This is controller-only validation; no successful new joint estimator-controller trial is claimed.

| Scenario | Executed / requested holds | Minimum sampled body gap (m; required 0.25) | Final speed (m/s; reference 8) | Median / maximum frame (ms) | Frames above 100 ms |
| --- | --- | --- | --- | --- | --- |
| stationary | 300 / 300 | 0.976027933 | 8.000408979 | 102.836 / 15658.374 | 280 / 300 |
| oncoming | 300 / 300 | 0.342057649 | 8.000408979 | 102.722 / 6109.352 | 275 / 300 |
| crossing | 300 / 300 | 9.519697938 | 8.000408979 | 102.583 / 500.129 | 283 / 300 |

Every issued hold passed the independent hard certificate, every recorded inherited witness remained feasible, terminal cone margins were nonnegative, and no terminal/fallback command was executed. Final lateral and heading errors are below 4e-18 m and 6e-19 rad in all three deterministic trials. Final speed error is approximately 0.000409 m/s; these numerical results are near-cruise recovery, not proof of exact zero steady-state error under a soft CLF and finite numerical reserves. The minimum gap column is sampled body distance, including the required 0.25 m; the stored `minimumSampledSeparationMargin` is the excess above that requirement.

Stationary admission selected a feasible full assignment after four integer-master calls (four active disjunction groups, 7.981493 s total integer solve time) and two conic solves. Its complete initial frame took 15.658374 s. There are 337 raw choice groups, or 8^337 assignments.

Oncoming admission at t=2.6 s selected a feasible full assignment after six integer-master calls (six active groups, six integer nodes, 3.932926 s integer solve time) and two conic solves. Its complete frame took 6.109352 s. The existing 32-hold admission horizon sufficed; the controller did not need the longer independent nonlinear witness from the earlier diagnosis. All remaining direction groups were checked against the complete candidate before acceptance. Crossing needed no integer search because the common convex optimum already satisfied a direction in every group. Its minimum gap is large, so this is a comparatively easy regression scene, not a difficult crossing stress test.

The final strict-budget oncoming run stopped at t=0, **before target admission**, with zero issued holds: the complete first target-free frame took 104.902 ms and exceeded the 100 ms deadline. The long-budget runs also exceed 100 ms on most ordinary frames (approximately 102.6--102.8 ms median), in addition to much slower initial admission. Thus the current implementation is not usable as an every-frame 100 ms controller, despite solving the previously rejected finite avoidance task. The extended offline budget must not be confused with real-time qualification.

## Change and mathematical scope

The user authorized conservative polyhedral approximation and finite branch enumeration. Removed `passingNormals`, the left/right selector, and the cosine displacement/time template. All future steering and longitudinal inputs remain optimization variables. Geometry charts cover the actuator-reachable station interval instead of an imposed station corridor about a trial trajectory.

The default direction grid contains eight unit vectors. For every swept cell and target, one branch contains **all** Bernstein collision rows associated with a direction, including footprint supports, uncertainty and arithmetic reserves. The finite exit guard also has eight alternatives. The full approximating set is the union over complete assignments of convex SOCP feasible sets. With C cells and J targets, its raw assignment count is 8^[J(C+1)], not eight complete trajectories. At the original oncoming admission horizon, C=224 and J=1, giving 8^225, approximately 10^203.195 assignments.

This is a conservative inner approximation of feasible controls, not a finite exact partition of all collision-free trajectories. Direction discretization, footprint majorants, a common normal over each swept cell and the declared model/horizon remain possible sources of conservatism. Adjacent branches may overlap.

The search uses a common convex relaxation and then integer branch-and-bound with lazy addition of violated disjunction groups. Every common physical constraint remains in the integer master. Its terminal cone half-spaces are outer relaxations used only for search. A complete assignment must pass the full SOCP and independent physical-row, terminal-cone and CLF checks. Return the first feasible certified branch; do not claim a global performance optimum. Complete assignments reported infeasible may be excluded; timeouts and numerical failures never authorize such exclusion. The shared timer includes input preparation and formulation. An overdue controller invocation returns no command.

The squared CLF slack penalty, operating-point input penalty, predictive continuation, invariant terminal certificate and fail-without-fallback policy remain. Successors inherit the **actual accepted convex family**, its generators and absolute confirmation deadline. They solve one convex problem without re-enumerating the active encounter. Confirmed partial target release removes only discharged obligations. Current local geometry metadata is rebuilt with selected directions and shifted consistently with its condensed rows.

The finite integer search is not an all-frame runtime guarantee. Enumerating a finite but exponential family can take substantially longer than one 100 ms control hold. Feasibility of the selected finite family and timely numerical completion are separate claims.

## Reproduction and parameters

Use `scripts/runExactStateRecursiveFeasibilityScenario.m` from the repository root. Final experiments use stationary, oncoming and crossing scenarios for 300 holds (30 s), sample time 0.1 s, nominal speed 8 m/s, complete perception range 16 m, no road boundaries, retained model domains, exact ego/target states and seed 20260912. Steering is bounded by 40 degrees and normalized longitudinal input by one; actuator-rate limits are infinite in these baseline scenarios. Target jerk and yaw acceleration are zero. The straight target laws and dimensions are those of the script/configuration; no changed target path makes the problem easier.

```matlab
addpath('scripts');
for scenario = ["stationary","oncoming","crossing"]
    runExactStateRecursiveFeasibilityScenario(Scenario=scenario, ...
        SampleCount=300,DeadlineSeconds=120, ...
        OutputDirectory=fullfile(output,"validated-"+scenario));
end
runExactStateRecursiveFeasibilityScenario(Scenario="oncoming", ...
    SampleCount=300,DeadlineSeconds=.1, ...
    OutputDirectory=fullfile(output,'validated-100ms'));
```

The 120 s budget per invocation is a **diagnostic offline search budget**, not the plant hold time or real-time acceptance threshold. `runtime.deadlineSeconds` remains 0.1 s in every report. Input assembly plus controller time excludes plant integration and offline sampled audits. Timing is measured in MATLAB R2026a Update 3 on a shared AMD Ryzen 7 7800X3D workstation, not a controlled worst-case execution platform. Failed runs save their report before raising the original exception. The diagnostic runs apply the newly computed input at the ideal hold timestamp; actual computation delay is not injected into plant motion, so their success is not a delayed-actuation safety certificate.

Raw MAT/JSON results and saved test results remain at `/home/zai/.cache/collisionAvoidance/finite-branches-20260916`. Full scenario JSON includes every state, issued input, frame duration, selected branch, solver counts, margins and failure message. Continuous safety authorization comes from the swept certificate; the 11 samples per hold in the scenario are independent diagnostics, not the safety proof.

## Regression checks and retained fixes

The complete 68-class MATLAB suite passed **597/597**, with zero failed or incomplete cases. After reverting the slower domain-tightened big-M experiment, the final implementation passed **68/68 affected regression cases** in `collisionAvoidanceControllerTest`, `convexBranchSearchTest`, `recursiveSafetyClosureTest`, `controllerSourceBudgetTest` and `straightRoadBoundaryConfigurationTest`. These final reruns supplement the complete suite; they are not represented as a second full-suite invocation. The high-gain observer's 35 online runtime cases passed in the full suite.

New behavioral coverage checks that incompatible first direction choices do not discard another feasible combination, finite-family exhaustion differs from timeout, the direction grid can omit rounded-corner safe points while accepted half-spaces retain Euclidean clearance, and an oncoming admission produces a feasible carried convex successor with no new integer search. The core source count remains 20.

The first complete regression attempt reported 594 passes and three failures (two incomplete): a stale single-call admission assertion, a 1.8141e-13 model-domain numerical certificate excess, and a geometry-only test using the 100 ms wall-clock deadline. Fixes permit finite horizon retries only after numerical infeasibility, pad the actuator-reachable chart by a numerical coverage allowance, and test road/model semantics with a diagnostic budget while retaining separate strict timing experiments. Hard collision/domain/terminal verification was not relaxed. Failed or overdue optimization still produces no control command.

Exploratory performance results are retained, without being described as successes: an all-binary master exceeded a 30 s budget; omitting common domain/slip rows lazily caused a 60 s oncoming timeout and was reverted; a domain-tightened big-M variant passed the full suite but timed out at 30 s in both stationary and oncoming scenario admission. It was also reverted. Tighter mathematical relaxations can change incumbent search behavior and do not guarantee lower wall-clock latency. The released implementation keeps all common physical rows and uses actuator-box big-M bounds.

The full-suite runner saves each class before continuing. The MATLAB tool request timed out at 300 s, but the MATLAB script continued and completed; the final saved results were loaded and checked using `assertSuccess`. Intermediate results and failed variants remain separately named in the cache directory.

See [finite-family derivation and source attribution](../controller/FINITE_CONVEX_BRANCHES.md), [conditional recursive-feasibility proof](../controller/TERMINAL_CBF_PROOF.md), and [the earlier independent nonlinear existence witness](ONCOMING_AVOIDANCE_WITNESS_20260916.md). The latter is a different experiment and does not certify nonlinear execution of these newly optimized affine-model controls.

Factory `checkcode -id -config=factory` reports zero findings in all 11 changed MATLAB source/test files. Final saved affected tests pass `assertSuccess`; the independent JSON audit checks all 900 accepted holds, carried-witness residuals, terminal margins and absence of fallback commands. `git diff --check` passes.
