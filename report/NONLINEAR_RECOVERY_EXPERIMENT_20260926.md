# Nonlinear controller repair experiments

> Historical record: the experimental nonlinear shooting controller and its dedicated execution/audit scripts were removed on 2026-09-26 at the user's request. Commands and implementation descriptions below refer to the recorded experiment, not the current controller. See [removal decision](NONLINEAR_SHOOTING_REMOVAL_20260926.md).

Date: September 26, 2026. This report accompanies the mathematical derivations in [NONLINEAR_RECOVERY_DESIGN_20260926.md](NONLINEAR_RECOVERY_DESIGN_20260926.md). The new controller is selected explicitly with `solver.method="nonlinearShooting"`; `affineSocp` remains the default. Results below distinguish the declared nonlinear bicycle from the independent PassVeh14DOF plant and distinguish complete recovery from a safe executed prefix.

## Diagnosed failure mechanism

The independent audit of the saved September 26 vehicle runs reconstructs the tire forces implied by the complete affine lateral/yaw acceleration model, using force and moment balance and undoing the front steering rotation. It compares those forces with an independent implementation of the combined-slip Fiala law. These reconstructed forces differ from the raw tire tangent because the full body generator also linearizes force rotation.

* Straight: the first front capacity excess above 1.01 occurs at 1.20 s. At 2.00 s, the implied front force is -49.651 kN while the actual Fiala force is +2.492 kN: both magnitude and sign are wrong.
* Circular: at 1.45 s, beta=-0.99998796 leaves only 31.924 N of front lateral capacity, while the affine acceleration implies -7.815 kN, a capacity ratio of 244.804.
* Warmed S-curve: at 1.30 s, the implied rear force is 2.500 kN against a 408.257 N capacity, a ratio of 6.124.

The [99-hold audit export](NONLINEAR_RECOVERY_EXPERIMENT_20260926/first-hold-tire-audit.csv) preserves the numerical comparisons. They identify an invalid prediction mechanism before the solver failures; they do not establish that every prior horizon or unexecuted continuation was safe.

A separate Python RK4 replay of the saved September 25 circular shifted anchor first leaves the Fiala slip domain at predicted hold 53. Explicitly blending that input sequence with cruise at fractions 0.75 and 0.5 restores the numerical tire domain, with maximum absolute slips 0.759567 and 0.673230 rad. The [homotopy export](NONLINEAR_RECOVERY_EXPERIMENT_20260926/shifted-anchor-homotopy-python.csv) is initializer evidence only. Values beyond the original invalid trajectory's first domain exit have no physical validity.

## Algorithm under test

The optimizer now propagates physical steering/utilization inputs through the nonlinear sampled model, checks actual rectangle separation and road geometry, and imposes hard terminal path/speed tolerances at the vehicle's predicted station. It retains verified feasible candidates during bounded SQP refinement. Every accepted candidate is independently replayed through all internal RK4 stages to reject undefined tire/Frenet states. Infeasible seeds have no execution authority.

The final-state tolerances are [0.25 m lateral, 0.04 rad heading, 0.35 m/s longitudinal speed, 0.25 m/s lateral velocity, 0.08 rad/s yaw rate] relative to the local cruise trim. Station is free. The experiment uses 50 ms control holds, 25 ms optimization geometry checks, and at most 10 ms RK4 integration steps. These checks are numerical and sampled; they do not establish robust nonlinear invariance or continuous-time collision avoidance.

The NRMM change computes the exact radial/angular hull of the Cartesian velocity rectangle before applying the original motion contracts. All published uncertainty remains. At first visibility, the straight/circular startup speed intervals are [2.869824,19.347320] and [2.076372,20.476583] m/s; course widths remain 2.481185/2.607358 rad and one-second position-radius norms remain 45.583511/47.584468 m. This is a tighter enclosure, not successful uncertain encounter admission. The experimental nonlinear controller explicitly rejects nonzero ego or target state uncertainty and does not silently use nominal values instead.

## Experiment accounting

The deterministic Fiala study uses exact state observations, 10 m/s ego/target speeds, 50 m initial Euclidean separation, 30 m target visibility, a 1.5 m target offset at the nominal conflict, and known road edges at +/-5 m. Ego dimensions are the controller defaults (4.8 by 1.9 m); the target is 5 by 2 m. The circular radius is 100 m. The S-curve uses a PCHIP representation of `kappa(s)=0.01*sin(2*pi*s/80)` through 80 m followed by a straight continuation. This is distinct from the finite-perception-road PassVeh14DOF campaign.

Final controller settings are a 5 s prediction horizon, four-hold move blocking, at most 12 SQP iterations per seed, 20 s nominal optimization budget, three candidate seeds, and at most four refinement iterations after a candidate first passes every nonlinear admission check. The time limit is checked at optimizer callbacks and between seeds; it is not a hard real-time deadline on an indivisible function evaluation. Remaining time is shared across untried seeds. Hard constraints are unchanged by early refinement termination.

Recovery requires the entire requested episode to finish, full-body passing, positive sampled SAT gap, sampled road containment, and all samples in the final 2 s within 0.5 m/s speed, 0.2 m lateral, and 0.02 rad velocity-course error. A completed 0.1 s smoke test cannot satisfy that recovery window. Passing and recovery are recomputed independently in Python; an early stopped prefix is never counted as recovery.

| Final Fiala case | Duration | Minimum SAT gap | Body passing time | Final 2 s max speed / lateral / course error | Recovery |
| --- | ---: | ---: | ---: | --- | --- |
| Straight | 10 s | 0.130997 m | 2.74 s | 2.18e-5 m/s / 2.67e-4 m / 3.42e-5 rad | Pass |
| Circular | 10 s | 0.120236 m | 2.75 s | 4.59e-5 m/s / 1.48e-4 m / 1.33e-5 rad | Pass |

The pre-cap straight development run also completes 10 s and recovers, with a minimum SAT gap of 0.117984 m. It used the same 12-iteration maximum without the four-iteration feasible-refinement stop. Its controller median / 95th percentile / maximum runtime was 3.654 / 5.185 / 16.814 s. The final straight run records 2.054 / 2.626 / 4.685 s; circular records 2.115 / 2.771 / 4.713 s approximately. These are offline wall-clock measurements on a shared workstation, not a controlled timing benchmark or a real-time guarantee.

## Verification and reproducibility

All 173 cases in ten relevant MATLAB suites pass on the final controller source: nonlinear seeds/domain replay, nonlinear optimization, public nonlinear execution contracts, velocity-sector enclosures, existing NRMM prediction, configuration, Fiala tires, trajectory linearization/feedback, and the original affine controller. The [test summary](NONLINEAR_RECOVERY_EXPERIMENT_20260926/matlab-tests.json) lists the exact suite files. The final suite count includes the two new refinement-stop tests.

The velocity-sector enclosure tests use Mersenne Twister seed 20260926 and 305 Cartesian velocity samples in each of six rectangles. Existing NRMM tests retain seeds 11 and 5. Closed-loop scenario drivers are deterministic and add no measurement noise.

```matlab
addpath('scripts');
settings = struct('nonlinear',struct('horizonSeconds',5,'blockSteps',4, ...
    'maxIterations',12,'timeLimitSeconds',20,'feasibleRefinementIterations',4));
result = runNonlinearRecoveryStudy(Case="straight",Duration=10, ...
    ControllerConfiguration=settings,OutputFile="/tmp/fiala-straight.mat");
```

Replace `Case` with `"circular"` or `"sCurve"` for the other geometries. The harness explicitly selects `nonlinearShooting`, saves each completed hold atomically, and preserves failure context. Independent replay and refined 5 ms integration use:

```bash
uv run --with scipy python scripts/auditNonlinearRecoveryStudy.py \
    /tmp/fiala-straight.mat --output-dir /tmp/nonlinear-audit --trace-csv --refine
```

The Python auditor reconstructs rectangle SAT separation, recovery, visibility, physical force/slip diagnostics, and nonlinear transitions from the saved raw states and commands. Its curved reference implementation uses independently integrated SciPy PCHIP curvature. A 5 ms replay checks integration sensitivity; it remains a numerical experiment rather than a validated continuous-flow enclosure.

Development limitations are retained: one isolated MATLAB launch failed before tests with MathWorks service error 5001; a first integration test expected an unnecessarily exact zero acceleration despite a road-load input-rate penalty. That assertion was replaced with actual successive-hold speed/lateral tracking checks, after which all 173 tests pass. One isolated test process reported allocator corruption during shutdown after its test results had been saved. The initial high-iteration vehicle probe exceeded two 300 s tool waits and was explicitly interrupted without a completed result; it is not counted as vehicle validation. The interruption used a handled SIGINT as documented by [MathWorks Support](https://ch.mathworks.com/matlabcentral/answers/101540-why-does-control-c-not-break-out-of-an-infinite-loop-on-unix#answer_110888).

Code Analyzer used default settings after a missing personal settings-file warning. No source error was reported; one solver line retains an unnecessary suppression annotation. Generated binaries, external solver sources, unrelated observer/scenario edits, and large original MAT files are outside the project commit.
