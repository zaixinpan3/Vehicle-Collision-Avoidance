# Retained-controller vehicle rerun

Prepared 2026-09-27T03:37:37.001684-05:00.

The existing single-controller vehicle campaign was rerun after removal of the nonlinear shooting implementation. All seven scenario attempts returned and MATLAB exited successfully. The controls and avoidance outcomes below describe fresh simulations; a failed prefix is not counted as completed avoidance or recovery.

The two no-target cruise controls complete, but all three exact-state avoidance trials and both estimated-state avoidance trials stop early. The S-bend controller-call median is approximately 2.23 seconds and every S-bend frame exceeds 50 ms. Removing the shooting branch has therefore not resolved avoidance feasibility or the varying-curvature runtime problem in the retained implementation.

## Results

| Scenario | Executed / requested (s) | Minimum executed SAT gap (m) | Completed | Recovered | Stop identifier |
| --- | ---: | ---: | --- | --- | --- |
| Straight cruise control | 18.00 / 18 | N/A | Yes | Yes | None |
| Circular cruise control | 18.00 / 18 | N/A | Yes | Yes | None |
| Straight oncoming | 2.05 / 18 | 4.019774 | No | No | collisionAvoidanceController:optimizationFailed |
| Circular oncoming | 1.50 / 18 | 15.360714 | No | No | collisionAvoidanceController:invalidTireOperatingPoint |
| S-bend oncoming | 1.40 / 18 | 17.181475 | No | No | collisionAvoidanceController:optimizationFailed |
| Estimated straight oncoming | 1.05 / 18 | 23.994603 | No | No | collisionAvoidanceController:optimizationFailed |
| Estimated circular oncoming | 1.05 / 18 | 24.048799 | No | No | collisionAvoidanceController:optimizationFailed |

SAT gap is the largest separating-axis gap of the actual oriented vehicle rectangles, not Euclidean closest-point distance. Positive prefix margins establish sampled non-overlap only over the executed portion. A zero-hold run has no executed plant trace. Road constraints remain enabled for avoidance trials; the two cruise controls retain their original no-boundary configuration.

## Runtime

| Scenario | Controller median / p95 / max (ms) | Complete frame median / p95 / max (ms) | Frames above 50 ms |
| --- | ---: | ---: | ---: |
| Straight cruise control | 6.77 / 7.25 / 626.85 | 8.97 / 9.53 / 632.77 | 3 / 360 |
| Circular cruise control | 6.74 / 7.06 / 162.67 | 11.55 / 12.14 / 169.56 | 1 / 360 |
| Straight oncoming | 33.72 / 281.37 / 379.42 | 35.57 / 284.90 / 381.40 | 8 / 42 |
| Circular oncoming | 7.96 / 120.02 / 247.03 | 12.97 / 125.71 / 251.89 | 7 / 31 |
| S-bend oncoming | 2228.41 / 2293.01 / 4657.59 | 2233.32 / 2301.45 / 4663.34 | 29 / 29 |
| Estimated straight oncoming | 7.96 / 13.76 / 384.93 | 20.10 / 34.49 / 402.67 | 1 / 22 |
| Estimated circular oncoming | 7.99 / 9.84 / 367.34 | 22.03 / 25.82 / 382.18 | 1 / 22 |

Controller time measures the public controller call, including failed attempts. Complete frame time also includes synthetic sensors, estimation/error bounds and road fitting. Neither includes Simulink plant advancement or offline preparation. The campaign runs serially, one computational thread, with runtime-deadline enforcement disabled and immediate scheduled actuation. These are observed offline timings, not a hard real-time guarantee. The earlier campaign overlapped other MATLAB processes; timing differences are not treated as a controlled speedup.

In the S bend, median controller-call time is 2231.68 ms before target visibility and 169.83 ms while the target is visible. The slow pre-visibility frames show that the measured cost is not confined to an active obstacle encounter. This is a timing observation; identifying the responsible functions requires profiling.

## Comparison with September 26

| Scenario | Previous executed (s) | Current executed (s) | Previous stop | Current stop |
| --- | ---: | ---: | --- | --- |
| Straight cruise control | 18.00 | 18.00 | None | None |
| Circular cruise control | 18.00 | 18.00 | None | None |
| Straight oncoming | 2.05 | 2.05 | collisionAvoidanceController:optimizationFailed | collisionAvoidanceController:optimizationFailed |
| Circular oncoming | 1.50 | 1.50 | collisionAvoidanceController:invalidTireOperatingPoint | collisionAvoidanceController:invalidTireOperatingPoint |
| S-bend oncoming | 0.00 | 1.40 | collisionAvoidanceController:optimizationFailed | collisionAvoidanceController:optimizationFailed |
| Estimated straight oncoming | 1.05 | 1.05 | collisionAvoidanceController:optimizationFailed | collisionAvoidanceController:optimizationFailed |
| Estimated circular oncoming | 1.05 | 1.05 | collisionAvoidanceController:optimizationFailed | collisionAvoidanceController:optimizationFailed |

The historical primary S-bend attempt stopped at time zero on an internal work-budget limit. Its separate warmed repeat stopped at 1.40 s with a 17.181475 m executed SAT gap; the present S-bend result is compared with both records, and its complete recorded control-state trajectory matches that warmed repeat exactly. This difference in startup behavior does not establish successful avoidance.

All seven comparisons verify identical initial state, centerline, duration, reference speed, control period, road offsets and controller configuration after excluding the two deliberately removed selector/nonlinear configuration fields from the historical configuration. Source hashes remain unchanged throughout this rerun. No controller repair or parameter tuning was performed.

## Conditions and verification

- MATLAB R2026a Update 3; Vehicle Dynamics Blockset PassVeh14DOF; 18 s requested per case; 10 m/s ego and target speeds; 0.05 s command holds.
- Avoidance cases use 50 m initial target distance, 30 m visibility, 5 by 2 m ego/target rectangles, 1.5 m target lateral conflict offset, 6/8 m road offsets plus 2.6 m shoulders. The circular radius is 100 m; the S bend uses the existing 0.01 1/m maximum curvature and 80 m wavelength.
- Exact-state cases have no stochastic measurements. Estimated cases retain random seed 20260925, a 10 m/s target-speed prior and the existing estimator integration configuration.
- The vehicle-only section of the existing audit independently reloads all seven MAT files, checks finite control states, completion and final two-second control-grid recovery (0.5 m/s speed, 0.2 m lateral and 0.02 rad course-error limits), and recomputes oriented rectangle SAT from plant traces where present. All consistency checks pass. This verifies accounting, not avoidance success or continuous-time safety. Road-margin values are the existing MATLAB audit output; no new independent road certificate is claimed.
- The execution wrapper, audit subset, comparison script, plots, original MAT files, source snapshot and console logs remain outside the repository. No removed shooting implementation or tool was reintroduced. No new unit-test suite was needed or claimed for this experiment-only rerun.

## Reproduction and artifacts

```matlab
addpath('scripts');
maxNumCompThreads(1);
runControllerRecoveryValidation(outputDirectory, 'vehicle');
```

Base project commit: `35b3a3b63abb5c99f339d5a6c4dead1d85441f4b`. The workspace retains pre-existing target-prediction, observer, scenario and documentation changes. The external source archive and source/native manifests identify the actual executed workspace; the base commit alone is not claimed to contain those pending edits.

Original run directory: `/home/zai/.cache/collisionAvoidance/single-controller-rerun-20260927-031629`.

Compact summaries, comparison checks, independent audit results and SHA-256 manifests are in [the result directory](SINGLE_CONTROLLER_RERUN_20260927/). Standalone PNG/PDF plots are in the original run directory. Source code, configurations and unrelated pending changes are excluded from this experiment report commit.
