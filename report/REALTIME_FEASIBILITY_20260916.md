# Real-time feasibility of the finite-branch controller

Date: September 16, 2026. Production baseline: `9afc87cbe1408379c8b3ead15ee9688e8d7ae698`. Requirement: the **complete estimator-controller frame must finish within 100 ms**, including new-target admission. This study profiles the controller and tests one equivalent numerical reduction; it does not change the production controller or observer.

## Assessment

**Further optimization is technically plausible, but the current implementation is substantially short of the requirement.** Ordinary target-free operation is close to 100 ms. Active-encounter convex solves need roughly an order-of-magnitude improvement, and initial branch admission is harder still. The evidence supports a structured reformulation experiment, not a promise of real-time success.

The bottleneck is not just MATLAB overhead or the theoretical number of branches. A single already-selected convex problem takes hundreds of milliseconds to more than one second. Therefore, even an oracle that immediately supplies the winning branch would not make the current controller real-time. The observer cannot explain these controller-only delays.

## Measured frames

The new driver freezes identical inputs and the previous controller state for repeated calls. Admission cases have three unprofiled repetitions; each other case has 15. Successor cases are the actual first successors of accepted encounters, with the plant advanced by the certified held-affine transition. These are frozen-frame replays, not another closed-loop campaign.

| Frame | Median full controller call (ms) | Maximum observed (ms) | Preparation / formulation / solve / verification-output medians (ms) |
| --- | ---: | ---: | --- |
| Fresh target-free cruise | 93.237 | 97.871 | 0.690 / 35.719 / 55.412 / 1.461 |
| Target-free successor | 95.334 | 115.415 | 0.687 / 37.169 / 55.801 / 1.609 |
| Oncoming first admission | 5927.700 | 6188.166 | 1.274 / 790.630 / 5135.010 / 4.897 |
| Stationary first admission | 12280.853 | 12507.824 | 1.082 / 1362.927 / 10921.662 / 7.925 |
| Oncoming active successor | 734.548 | 775.100 | 1.419 / 20.024 / 708.810 / 4.569 |
| Stationary active successor | 1681.413 | 1690.085 | 1.498 / 43.994 / 1627.046 / 8.190 |

Column medians need not sum to the median total. The residual phase includes output construction as well as independent verification. A 120 s diagnostic budget allows measurement of slow successful calls; it does not change the 100 ms requirement or inject computation delay into the plant.

The previous 30 s campaign's roughly 103 ms overall median concealed much slower active encounters because most frames were cruise. The new 93--95 ms cruise medians are measurements under different host load and replay conditions, **not a speedup caused by a code change**. Even this target-free replay has a 115 ms sample. Controller time alone also leaves insufficient margin for an unmeasured observer, adapter and scheduling overhead.

The machine is a shared AMD Ryzen 7 7800X3D workstation, MATLAB R2026a Update 3, eight MATLAB computation threads. The native Clarabel interface requests one thread. No CPU isolation, cold-start worst-case bound or full-pipeline qualification is claimed. Parameters: straight path, 0.1 s holds, reference 8 m/s, 16 m complete perception, no road boundaries, retained 4 m lateral model domain, zero ego/target measurement errors and zero target jerk/yaw-acceleration bounds. Fixtures are deterministic; no random seed is used. Oncoming admission is at 2.6 s with target position 39.2 m and velocity -8 m/s; stationary admission starts with the target at 15 m.

## Why the convex problem is expensive

Separate instrumented calls capture the exact native solver programs. The following row counts are after the current zero-row reduction and include cone rows; they differ slightly from full formulation counts.

| Captured program | Variables | Rows | Matrix nonzeros | Iterations | Native reported time (ms) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Target-free successor | 33 | 14402 | 121930 | 13 | 54.581 |
| Oncoming common relaxation | 65 | 28802 | 473226 | 12 | 200.554 |
| Oncoming selected branch | 65 | 33231 | 591306 | 36 | 788.983 |
| Stationary common relaxation | 97 | 43202 | 1053898 | 13 | 536.494 |
| Stationary selected branch | 97 | 49631 | 1317034 | 30 | 1659.301 |

The online formulation eliminates all predicted states into the input vector. Its objective similarly substitutes the complete state prediction before forming the Hessian. Consequently, later constraints couple many earlier controls. Merely wrapping these matrices in `sparse` does not recover the original local dynamics structure. See [objective and decision construction](../controller/formulateAvoidanceProblem.m).

The continuous certificate is expanded extensively. In these fixtures, each 100 ms hold has seven swept cells and eight Bernstein coefficient points per cell. Sixteen common model-domain/slip rows per point give **896 common rows per hold**, before collision, terminal and actuator constraints. A 32-hold program already has 28672 such rows. The cell-count rule in [finitePredict](../controller/ltvBicycleModel.m) is `max(minimumCells,ceil(2*norm(A,inf)*h))`. These internal certificate cells do not require seven control updates per hold, but their computational cost is real.

Reducing this expansion requires a sound reformulation or a proof that removed rows are redundant. Endpoint-only collision checks, sampled domain checks, arbitrary reduction of Taylor order, or unvalidated remainder changes would alter the continuous-time guarantee.

The current `localCompactPlanarRows` in [solveHardCbfClf](../controller/solveHardCbfClf.m) immediately returns unless the program has exactly two variables. Current multi-hold programs with CLF slack have 33--97 variables, so that specialized reduction does not help them. The existing [stage-local transcription utility](../controller/avoidanceStageQp.m) illustrates useful local dynamics and geometry, but has a different interface and is not on this online solve path. It is not an available configuration switch for the current problem.

## Admission search attribution

In unprofiled admission calls, median integer-solver time is 3.882 s oncoming and 7.819 s stationary. The oncoming search has six integer calls and six reported integer nodes; stationary has four calls and two nodes. Thus the observed cost cannot be explained as literal enumeration of all `8^225` or `8^337` assignments.

A separate oncoming MATLAB profile attributes 3.909 s inclusive to six `slbiClient` calls and 0.988 s to two native conic solves. Ten full geometry builds take 0.807 s inclusive, including 0.380 s in the projection MEX. The integer matrix-assembly helper takes only 0.028 s across its six calls. These inclusive times overlap and must not be summed. Profiling adds overhead; speed claims use the unprofiled table. [MathWorks profiling documentation](https://www.mathworks.com/help/matlab/matlab_prog/profiling-for-improving-performance.html)

The implementation eagerly constructs full geometry for all eight directions, although the lazy integer search activates only six oncoming disjunction groups. Common domain rows are recomputed in these builds. The selected geometry is built again to retain consistent local metadata. There is avoidable work here, but removing all of it would still leave seconds of native integer and conic solve time.

Small node counts do not imply cheap mixed-integer optimization: root relaxations, presolve, cuts and heuristics also consume time. The profile resolves the native client boundary, not its internal distribution among those operations. [MathWorks MILP algorithms](https://www.mathworks.com/help/optim/ug/mixed-integer-linear-programming-algorithms.html), [intlinprog outputs and limits](https://www.mathworks.com/help/optim/ug/intlinprog.html)

Already-admitted successors inherit the selected family and avoid new integer search. This mechanism is already implemented; adding it again is not an optimization proposal. Its measured 735 ms and 1681 ms first successors are the clearest evidence that both numerical layers need attention.

## Equivalent reduction experiment

The research driver groups exactly equal inequality coefficient rows and retains the smallest right-hand side in each group. This preserves their intersection. All cones and objective terms remain unchanged. Every resulting decision is checked against the **complete original** physical, terminal and CLF certificate. No production tolerance or constraint is relaxed.

Three paired solves per fixture give:

| Selected convex program | Rows before / after | Median solve before / after (ms) | Reduction construction (ms) |
| --- | --- | --- | ---: |
| Cruise successor | 14402 / 12654 | 55.789 / 49.324 | 5.388 |
| Oncoming admission branch | 33231 / 29187 | 793.599 / 679.873 | 13.854 |
| Stationary admission branch | 49631 / 43591 | 1677.056 / 1337.060 | 34.207 |
| Oncoming successor | 32219 / 28299 | 705.900 / 605.140 | 24.828 |
| Stationary successor | 48619 / 42703 | 1597.765 / 1368.156 | 30.674 |

About 12% of rows disappear and solver calls improve by approximately 12--20%. This is real headroom, but far short of the required improvement. Preprocessing is separate and partly consumes the gain; these are not full-frame speedups. The admission experiments start with the selected branch and do not time its search. Maximum decision differences range from 6.8e-8 to 1.4e-5 in infinity norm; saved objective values and positive physical margins document numerical, rather than bitwise, agreement. Every original certificate passes.

## Prioritized development decision

1. **Benchmark an equivalent sparse or partially condensed formulation first.** Keep stage/cell states as auxiliary variables and impose local affine dynamics as equalities. Express the same Bernstein rows using their local state/input maps; retain the same state objective, operating-point input penalty, squared CLF slack and terminal cones. This increases variable count while reducing coupling and factorization fill. Compare full input decisions, objectives and original certificates against the captured fixtures. More variables alone do not imply a faster or slower solve; measure both formulations before replacing the online path.
2. **Reduce proven redundant work in certificate construction.** Share common geometry across directions, cache parameter-independent maps and build direction-specific rows as needed. Exact duplicate reduction is demonstrated here. Further domain-based redundancy proofs and tighter validated interval representations are research options, not established speedups. The full independent safety verification remains mandatory; it currently costs only a few milliseconds together with output handling.
3. **Reuse native solver workspaces where structure permits.** The current [MEX](../controller/solveAvoidanceSocpMex.cpp) creates and frees a Clarabel solver on every call. Data updates can avoid repeated setup when dimensions, sparsity and cones stay fixed. Carried horizons shrink, so this needs deliberate templates or compatible workspaces. Clarabel also restricts updates when presolve/chordal processing changes the structure. Cacheable setup must be measured separately; numerical interior-point factorizations do not become reusable merely because symbolic structure is fixed. [Clarabel data-update contract](https://clarabel.org/stable/user_guide_data_updating/)
4. **Then reformulate and reuse the branch master.** Apply the validated local constraint representation to the common relaxation and integer master; investigate persistent model updates and solver state across added disjunctions. Preserve verification of every unactivated group before acceptance. The previous tighter-big-M experiment actually timed out, so stronger relaxations cannot be assumed faster. Do not cap explored branches and report the result as complete enumeration.
5. **Qualify the whole pipeline on a declared operating envelope.** Include observer work, admission, active successors, target release, multiple targets, uncertainty, cold caches and numerical difficult cases. Budget controller work below 100 ms to leave measured room for the other components. Test maximum frame time and delayed actuation, not only median solver time. A timeout that stops without a command respects the failure policy but does not demonstrate usable real-time control.

Sparse factorization with fixed symbolic structure and parameter-to-program generation are established techniques for embedded SOCP. They motivate this experiment, but published timings for other problems do not predict this controller's speed. [Domahidi, Chu and Boyd, ECOS](https://stanford.edu/~boyd/papers/ecos.html), [Chu et al., Code Generation for Embedded Second-Order Cone Programming](https://stanford.edu/~boyd/papers/pdf/ecos_codegen_ecc.pdf)

These first steps preserve the PCBF/soft-CLF/high-gain-observer framework, the finite conservative branch family and predictive continuation. Shortening the certificate horizon, grouping direction choices across cells, reducing direction count or dropping target bounds can shrink the feasible family or weaken guarantees; they are separate mathematical changes, not equivalent code optimizations. Asynchronous admission also needs a certified action and delay model during computation. An old road-only plan cannot simply authorize ignoring a newly detected target.

Practical confidence differs by phase: cruise optimization is a plausible near-term target; active convex control needs a substantial structural improvement that remains unmeasured; initial mixed-integer admission is the main unresolved obstacle. Finite search is not a uniform 100 ms bound. Neither this profile nor a few successful scenarios can guarantee that every feasible encounter in an unrestricted family will be found before the deadline.

## Reproduction and checks

From the repository root in MATLAB:

```matlab
addpath('scripts');
output = fullfile(tempdir,'finite-branch-runtime');
profileFiniteBranchRuntime(output,AdmissionRepetitions=3,RegularRepetitions=15);
benchmarkFiniteBranchRowReduction(output,Repetitions=3);
```

Actual results: `/home/zai/.cache/collisionAvoidance/realtime-feasibility-20260916`. `summary.json`/MAT contains every unprofiled duration and phase; each frame MAT contains its fixture, captured numerical programs and raw profile; profile CSVs list inclusive and self times. `row-reduction.json`/MAT contains paired timings, decisions' differences, physical margins and objectives. Profiler self time is calculated as total time minus child-call totals, following the [profile data definition](https://www.mathworks.com/help/matlab/ref/profile.html).

The driver checks that profiled and captured decisions reproduce the unprofiled result within 1e-7, and that every call is independently certified without fallback. All 30 paired convex solves (15 baseline and 15 reduced) pass the original certificate. Both new scripts have zero factory `checkcode -id -config=factory` findings. No production source changed and no new full regression suite or joint estimator-controller trial was run for this assessment. The previously reported 597-case suite belongs to the baseline implementation, not this profiling campaign.
