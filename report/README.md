# Experiment reports

Latest implementation: [Road boundaries in the terminal set](TERMINAL_LATERAL_CLEARANCE_20260924.md). The terminal set is shrunk by a declared lateral clearance (four rows on the true tracking error with an exact corner allowance for curved references), finite fitted boundaries stay node-row sensor products, and a frame whose fitted boundaries changed is re-admitted from the shifted plan instead of raising changedExecutionContract. The four road-bounded declared-plant cases that every earlier record rejected at frame 0 now complete 240 holds each, collision-free and inside the road; the recursion campaign is identical without a declaration. Follow-up: a perception-limited boundary constrains only the cells inside its range instead of rejecting the frame; with the default 30 m curbs all four PassVeh14DOF scenarios and the estimator-in-the-loop suite now run to the same end points as without curbs (cruise 80/80, avoidance at the close-encounter and first-detection infeasibilities), inside the road throughout.

Latest scenario rerun: [Controller scenario rerun](CONTROLLER_SCENARIO_RERUN_20260924.md). On the current tree (3073d57 plus a concurrent session's uncommitted controller edits) the two PassVeh14DOF cruise scenarios complete 80 of 80 steps for the first time, by re-admitting from the measurement at every hold. No PassVeh14DOF avoidance scenario completes: road boundaries are still rejected at frame 1; without them the fixed-direction program is infeasible one or two holds before the closest approach with truth targets (t = 1.55 and 1.60 s), and at the first radar frame with the estimator's first-detection bounds (speed ±30 m/s, course unbounded) under the NRMM contract. A harness gap in the estimator branch of runCenterlineCruiseScenario (missing held input) was corrected. Declared plant: recursion 5 of 5, sweep 103 of 118 with two three-fold-uncertainty curved cases flipping under the uncommitted edits.

Latest implementation: [Removal of the Cartesian bound from the NRMM target box](NRMM_TIME_TAYLOR_REMOVAL_20260924.md). At the user's direction the NRMM target box is the parameter-Taylor bound alone; the Cartesian constant-acceleration bound on the same paths, intersected with it since d1735f7, is removed. With the estimator's published parameter bounds the box of the 15 m/s fixture is unchanged (5.79 m at 4.8 s); the first-frame supports of slow targets grow by 1–5 % at nominal errors and 27–54 % at ten-fold errors, where the box is now 1.2–1.7 times the Cartesian box of a truth-derived jerk bound. In the NRMM closed-loop campaign the box-only declaration loses three ten-fold cases (26 of 30); with the published bounds every outcome is kept (29 of 30). 808 of 810 tests pass (2 data-dependent skips); the Cartesian campaigns are identical to 72a2ff1.

Previous implementation: [NRMM parameter error bounds from the estimator](NRMM_PARAMETER_INTERFACE_20260924.md). The NRMM estimator now publishes the error bounds of the target's speed, course, speed-rate and curvature from its frame-free component balls, and the controller intersects them with the intervals it derives from the inertial box, carrying and propagating the intervals through inherited frames. On the estimator's reconstruction fixture (a 15 m/s target with a 0.03 rad ego yaw error) the speed half-width falls from 0.81 to 0.13 m/s and the box at 4.8 s from 8.30 to 5.79 m per axis. The reachable box is documented as the intersection of a parameter-Taylor and a time-Taylor bound on the same NRMM paths; measured, the time-Taylor bound still binds for slow targets with large errors. The NRMM closed-loop campaign, now with disc-shaped velocity and acceleration noise, passes 29 of 30 cases in every variant. 810 of 812 tests pass (2 data-dependent skips); the sweep, estimator-bound, maneuvering-target and recursion campaigns are identical to d1735f7.

Earlier implementation: [NRMM-consistent target prediction](NRMM_TARGET_PREDICTION_20260924.md). The target contract `nrmm-motion-v1` declares exact NRMM motion (constant speed-rate and sideslip), and the certificate's target box now encloses every NRMM path through the estimate box (parameter enclosure intersected with the Cartesian enclosure of the same paths). The `J t^3/6` term is gone: for a 15 m/s turning target the box at 4.8 s is 2.85 m instead of 8.89 m per axis (3.02 instead of 26.28 m in the wider estimator domain). The estimator publishes the NRMM contract when its model error is zero, and its acceleration cap is now the magnitude bound; the speed-rate maximum it published before was not a valid cap for turning targets (fix to 7df7bd5). With NRMM truth targets on the declared plant, 26 of 30 cases pass under a truth-derived Cartesian jerk bound, 27 under the NRMM contract and 29 with the reactive policy tried first; the reactive tube carries its reaction to the NRMM remainder as generators. 802 of 804 tests pass (2 data-dependent skips).

Earlier implementation: [Closed-loop target prediction validation](TARGET_REACTION_VALIDATION_20260923.md). Commit 7df7bd5: the target's reachable set now uses the published scalar acceleration maximum (t^3 growth becomes t^2; the value the estimator published was its speed-rate maximum, not a bound on |a|, corrected 2026-09-24), and a target-reactive feedback policy (joint ego-target tube, relative collision records) is tried when the ego-only certificate fails. With target jerk 1-2 m/s^3 and the recorded estimator ego bound, the declared maximum raises the maneuvering-target campaign from 7 to 11 of 12 passing cases; the reactive policy changed no outcome and trims the late record supports by only 1-2 m of 18-32 m. The campaign's truth targets deliberately violate the constant-acceleration/constant-sideslip (NRMM) target model; for a turning NRMM target, most of the certificate's target growth is the Cartesian enclosure of its deterministic turning (7.1 m of 8.9 m at 4.8 s in the estimator domain), not a free maneuver (correction in the report). Sweep, estimator-bound and recursion campaigns are unchanged; 787 of 789 tests pass (2 data-dependent skips).

Earlier implementation: [Feedback prediction validation](FEEDBACK_PREDICTION_VALIDATION_20260923.md). Commit 6c244dd predicts the ego deviation under per-hold feedback instead of open-loop growth. With the recorded estimator bound and per-frame measurement noise, all eight declared-plant scenarios complete 240 holds; four of them (stationary and crossing, straight and curved) were infeasible at the first frame with open-loop prediction. The 118-case sweep resolves the two ten-fold stationary cases, and the ten-fold curved oncoming case becomes infeasible because the feedback reserves steering its full-lock swerve needs. All 780 tests have no failure (778 passed, 2 data-dependent skips).

Latest rerun: [Declared-plant rerun after removing the curved-road chart boxes](CHART_BOX_REMOVAL_RERUN_20260923.md). At 09f29f6 the sweep improves from 95 to 102 passing cases: 7 of the 8 box-blocked curved-road cases now pass with positive clearance. Case 96 (curve, 0.1 rad initial heading error) completes but overlaps the target by 6.2 mm at a hold node, because its plan leaves the former seed neighborhood by about 10 units and the true chart error reaches 8 times the charged allowance. The feedback-tube proposal for the ego uncertainty is in [../controller/FEEDBACK_TUBE_PREDICTION.md](../controller/FEEDBACK_TUBE_PREDICTION.md).

Latest diagnosis: [Why the admission SOCP is infeasible](ADMISSION_INFEASIBILITY_DIAGNOSIS_20260923.md). None of the 11 infeasible sweep cases is a physical impossibility of avoiding the actual target. In 8 curved-road cases the per-node chart boxes around the initializer's seed trajectory act as a trust region; removing only those boxes makes the complete problem (collision, exit, terminal set, actuator and slew limits) feasible. In the 3 ten-fold-uncertainty cases the open-loop propagated uncertainty box no longer fits the terminal invariant set even on the reference, and in one of them the target's declared reachable box (8.24 m per axis at 4.8 s; an earlier "18 m radius" figure was a sum over both axes and the ego part) makes robust avoidance infeasible.

Latest validation rerun: [Controller validation rerun after deleting post-solve verification](CONTROLLER_VALIDATION_RERUN_20260923.md). At commit 0b0855f the 118-case declared-plant sweep reproduces every 2026-09-22 outcome (95 pass, 15 errors, 8 inter-node overlaps; identical failure messages), and the recursion campaign passes all five cases at 600 holds. The four PassVeh14DOF scenarios, the estimator-in-the-loop suite and two of three 50 ms deadline cases still fail for their previously recorded, unrelated causes. The removed checks would not have rejected any issued plan in these runs, and the unchecked previous-plan fallback was never triggered.

Latest failure-mode sweep: [Controller failure-mode sweep on the declared plant](CONTROLLER_FAILURE_MODE_SWEEP_20260922.md). With deadlines disabled, 95 of 118 declared-plant cases pass. Apart from the four road-boundary rejections, eleven curved-road cases find the fixed-direction admission SOCP primal infeasible when the encounter is close and initial error or declared uncertainty is nonzero, and eight straight-road cases pass every node certificate while the exact flow between nodes overlaps the target by up to 2.7 cm. Every oncoming variant leaves the 0.40 rad heading domain under full-lock steering. Reproduce with runDeclaredPlantFailureSweep.

Current study scope: [Single-obstacle NRMM/VFFM initialization](SINGLE_OBSTACLE_VFFM_SCOPE_20260922.md). The controller accepts zero or one obstacle and rejects multiple target records. One Gaussian generates opposite passing-side candidates. All 801 current tests have passing coverage across the full run and the final 159-test regression; the full run initially rejected one obsolete dual-target fixture.

Preceding validation rerun: [Controller validation rerun](CONTROLLER_VALIDATION_RERUN_20260922.md). The declared held affine plant passes all five recursion cases at 600 holds each, collision-free with a satisfied CLF decrease. All four PassVeh14DOF scenario commands in CLAUDE.md fail: the two cruise scenarios stop at the second sample on the zero-radius successor-box test, and the two avoidance scenarios are rejected at the first frame because the terminal certificate admits no road boundary. A stale planningProblem.qp field in scripts/runCenterlineCruiseScenario.m was corrected first; it had aborted every scenario before any result. Only one of three enforced 50 ms deadline cases completes.

Latest controller optimization: [Adopt admission assembly shortcuts and compare warmed runtime](NRMM_VFFM_ADMISSION_OPTIMIZATION_20260922.md). All 799 tests pass. Alternating old/new measurements show circular-crossing median admission time decreasing from 50.469 to 46.377 ms (8.1%), with identical complete decisions and final safety programs. All 12 diagnostic closed loops preserve the prior trajectories exactly; strict 50 ms completion improves to 10/12, with two stationary-case first-frame timeouts remaining.

Preceding admission timing analysis: [NRMM/VFFM component profile and bounded optimization experiments](NRMM_VFFM_ADMISSION_PROFILE_20260922.md). Geometry and certificate assembly dominate the Gaussian reference. External early-row-filtering and dominated-candidate prototypes preserve full decisions and final safety programs on three fixtures; combined circular-crossing median improves by 2.45--3.50 ms against bracketing baselines. That study left production unchanged and motivated the adoption above; two combined prototype scenario types still showed 50 ms overruns.

Latest controller review: [NRMM/VFFM warmed simulation and independent footprint audit](NRMM_VFFM_SIMULATION_REVIEW_20260922.md). All 792 tests and 12 diagnostic trials pass. Only 6 of 12 strict 50 ms trials complete; the other six reject their first frame. Refined circular-crossing gap is 0.155002 m, but straight-stationary clearance is only 0.0927 mm. Physical road boundaries remain unsupported by the terminal certificate, and three cases exceed the optional heading diagnostic envelope.

Original trajectory initialization reconstruction: [NRMM/VFFM reconstruction, derivation and validation](NRMM_VFFM_INITIALIZATION_20260922.md).
That revision introduced analytical target motion at ego arrival times, quadratic-road
normal charts, multi-target superposition and body-yaw-consistent affine fitting.
Its 792-test result describes that revision. The current study is restricted to
a single obstacle; see the [current derivation](../controller/NRMM_VFFM_INITIALIZATION.md).

Previous initialization implementation: [Adopt Cheng fluid-reference initialization](CHENG_FLUID_INITIALIZATION_ADOPTION_20260921.md). The production initializer now compares two Gaussian references using a shared terminal-preserving affine fit and analytic rectangle normals before one full convex trajectory solve. The old direction/amplitude search and its configuration fields are deleted. All 776 tests and 30 variation cases pass; 12 diagnostic closed loops complete. Five strict 50 ms trials reject their first frame, and the straight stationary refined gap is only 0.0927 mm; no hard-real-time or physical-robustness claim is made.

Current controller: [Fixed-direction convex trajectory optimization](../controller/JOINT_SUPPORT_CERTIFICATES.md). Direction search only initializes the trajectory optimization. The selected normals remain fixed, and every new admission requires a full hard-constrained SOCP result. Inherited frames optimize the complete control sequence using retained directions.

Latest initialization study: [Cheng virtual-fluid reference construction and offline convex admission](CHENG_FLUID_INITIALIZATION_REVIEW_20260921.md). The paper's modified-function path rule yields a Gaussian lateral excursion. A motion-aware curved-road prototype supplies fixed normals that admit six mirrored crossing fixtures at selected widths; all 60 parameter-study outcomes, including failures, are retained. That offline study left production unchanged; its findings motivated the production replacement above.

Preceding implementation and validation: [Mandatory full trajectory optimization after direction selection](FIXED_DIRECTION_CONVEX_OPTIMIZATION_20260921.md). This supersedes the scalar direct-output and joint-normal branches of the preceding implementation. Warmup is excluded from the measured runtime results.

Preceding algorithm optimization: [Tapered admission, coarser exploration and warmed paired validation](TAPERED_ADMISSION_OPTIMIZATION_20260921.md). Circular-crossing replay median/max decreases from 57.9145/60.332 to 30.6155/33.421 ms. All 14,400 measured frames complete below 50 ms, including 7,200 under the strict deadline. Refined circular-crossing body gap is 0.075145607 m; known straight inter-node overlaps persist. These finite observations are not a worst-case runtime or continuous-safety guarantee.

Preceding frame profile: [Warmed controller timing and circular-crossing component costs](LONGEST_CONTROLLER_FRAME_PROFILE_20260921.md). Warmup overhead is excluded from the assessment. Clean replay median/max is 34.865/37.236 ms for straight stationary and 60.888/64.033 ms for circular crossing; fresh admission remains online work. Original observations are retained for traceability, and instrumented attribution is reported separately.

Latest admission repair: [Recover circular crossing through full-plan timing freedom](CIRCULAR_CROSSING_ADMISSION_REPAIR_20260921.md). The formerly rejected circular crossing now completes the declared-model closed loop. Diagnostic-budget safety and strict 50 ms frame acceptance are evaluated separately.

Preceding admission diagnosis: [Circular crossing: fixed temporal shape, chart conservatism and verified feasible counterexamples](CIRCULAR_CROSSING_ADMISSION_DIAGNOSIS_20260921.md). The original scalar line has no robust certificate within its chart domain; changing only the temporal deformation yields four plans that pass the unchanged original hard verifier. That diagnostic task left production behavior unchanged; its failure reproducer is pinned to commit `07aa84b263fefa2c265be7fad8fabdbd527f428d`.

Preceding simulation rerun: [Straight/circular avoidance, independent footprint audit and full-frame timing](STRAIGHT_CIRCULAR_VALIDATION_20260920.md). Two measured six-case campaigns confirm two straight inter-node overlaps and circular crossing rejection; continuation maximum is 42.173 ms, while four admission frames exceed 50 ms and the maximum is 108.710 ms. Straight crossing has no nominal collision threat.

Latest MATLAB optimization: [Local support assembly, exact program equivalence and warm runtime retest](MATLAB_ASSEMBLY_OPTIMIZATION_20260920.md). Active continuation median/max is 13.9275/42.613 ms with zero measured 50 ms misses; first admission still reaches 97.773 ms. All 737 tests pass.

Baseline MATLAB profiling: [Warm full frames, first admission, exact replays and assembly hotspots](MATLAB_RUNTIME_PROFILE_20260920.md). This preceding measurement motivated the assembly optimization.

Latest native benchmark: [Generated C executable, same-input timing and complete hybrid-frame limits](STANDALONE_C_BENCHMARK_20260920.md). The numerical kernel is ported; complete standalone orchestration remains unfinished.

Native performance entry: [Automatic recapture, C regeneration, rebuild and executable verification; historical 92.155 ms frame breakdown](NATIVE_BENCHMARK_WORKFLOW_20260920.md). Use `python3 scripts/runNativeControllerBenchmark.py` after source changes; reported timings are from the standalone numerical executable.

Latest implementation: [Affine-section admission: fixed interval computation, 50 ms straight/circular rerun and capability tradeoff](AFFINE_SECTION_ADMISSION_20260919.md).

Latest period change: [50 ms prediction nodes, input holds and control updates; matched-duration runtime and collision rerun](CONTROL_PERIOD_50MS_20260919.md).

Admission design review: [Affine control sections and forbidden-interval computation: verified conditions and integration limits](AFFINE_SECTION_ADMISSION_REVIEW_20260919.md). This preceding review records the design conditions; the implementation and measured outcomes are in the new report above.

Bounded-admission implementation and 100 ms comparison: [One geometric initialization, global homogeneous support majorants, and hard-domain screening](BOUNDED_ADMISSION_20260919.md).

Previous admission profile: [Extract the 4.654 s frame; 120 unsuccessful restoration iterations dominate admission](ADMISSION_FRAME_PROFILE_20260919.md).

Previous runtime retest: [Two warmed rounds, 9,000 frames, and separate active-continuation/admission timings](CONTROLLER_RUNTIME_RETEST_20260919.md).

Previous clearance validation: [Remove the fixed physical clearance buffer; fifteen trials expose two inter-node collisions](NO_CLEARANCE_BUFFER_VALIDATION_20260919.md).

Previous implementation validation: [Remove the legacy algorithm and rerun joint admission/continuation](JOINT_SUPPORT_ONLY_20260919.md).

Research comparison before removal: [Joint admission, retained feasibility and nine straight/circular trials](JOINT_SUPPORT_CERTIFICATES_20260919.md).

Experiment results, validation reports, runtime analyses, and recorded implementation findings live here. Executable experiment drivers remain in `../scripts/`.

Historical fixed-normal validation: [Single-convexification retest: twenty cases, five perturbed cruise recoveries and a refined inter-node clearance audit, September 18, 2026](STRAIGHT_CIRCULAR_SINGLE_SOLVE_RETEST_20260918.md).

Historical runtime profiling: [Single-solve controller: warm formulation/solver costs, first-use latency and profiling isolation, September 18, 2026](SINGLE_SOLVE_RUNTIME_PROFILE_20260918.md).

Latest estimator improvement: [Free-metric gain synthesis, paired noise reduction and response-lag tradeoff, September 16, 2026](NRMM_NOISE_REDUCTION_20260916.md).
Latest estimator evaluation: [Multi-scenario campaign, September 17, 2026](NRMM_ESTIMATOR_SCENARIOS_20260917.md).

Latest implementation and validation: [Overlap-aware admission, straight controller timing and remaining joint-certificate limits, September 16, 2026](OVERLAP_ADMISSION_IMPLEMENTATION_20260916.md).

Latest joint experiment: [Estimator-in-the-loop avoidance blocked by published bounds: diagnosis and admissibility sweeps, September 17, 2026](ESTIMATOR_IN_THE_LOOP_ADMISSION_20260917.md).

Latest period trial: [50 ms control period: cruise fits, admission frames double, September 17, 2026](CONTROL_PERIOD_50MS_20260917.md).

Latest certificate change: [Hold-node safety certificate: semantics, inter-node diagnostic and frame times, September 17, 2026](NODE_CERTIFICATE_ADOPTION_20260917.md).

Latest runtime optimization: [Implementation-only frame-time reduction, strict oncoming qualification and the residual stationary-admission bottleneck, September 17, 2026](CONTROLLER_RUNTIME_OPTIMIZATION_20260917.md).

Latest circular controller validation: [Sixteen circular cases, eight strict 100 ms successes and remaining admission bottlenecks, September 17, 2026](CIRCULAR_RUNTIME_RERUN_20260917.md).

Latest runtime assessment: [Real-time feasibility, active-encounter bottlenecks and equivalent row-reduction benchmark, September 16, 2026](REALTIME_FEASIBILITY_20260916.md).

Latest diagnosis: [Stationary and oncoming distance-dual initialization failures and current feasible witnesses, September 16, 2026](DISTANCE_DUAL_INITIALIZATION_DIAGNOSIS_20260916.md).

Latest admission design review: [Overlap-aware support directions, temporal conflicts and hard-feasibility ablation, September 16, 2026](OVERLAP_SUPPORT_PROPOSAL_REVIEW_20260916.md).

Current formulation: [Joint support certificates](../controller/JOINT_SUPPORT_CERTIFICATES.md). Dated reports below retain their historical findings.

Latest independent rerun: [CLF slack without artificial road boundaries, September 15, 2026](CLF_SLACK_NO_ROAD_RERUN_20260915.md).

Latest failure analysis: [Single-hold infeasibility, shared uncertainty and passing geometry](SINGLE_HOLD_INFEASIBILITY_ANALYSIS_20260915.md).

Each report states its experiment date and scope. Earlier results describe the revision tested at that time.

| Report | File |
| --- | --- |
| Current runtime hotspots: clean frame replays, coarse component probes and repeated admission work | [CONTROLLER_RUNTIME_HOTSPOTS_20260918.md](CONTROLLER_RUNTIME_HOTSPOTS_20260918.md) |
| Straight and circular node-certificate rerun: 17/20 strict successes, inter-node clearance audit and final cruise recovery | [STRAIGHT_CIRCULAR_RERUN_20260918.md](STRAIGHT_CIRCULAR_RERUN_20260918.md) |
| Circular controller rerun: complete diagnostic avoidance/recovery, strict timing and crossing-search regression | [CIRCULAR_RUNTIME_RERUN_20260917.md](CIRCULAR_RUNTIME_RERUN_20260917.md) |
| Estimator-in-the-loop admission failure: published-bound diagnosis, fresh-admission probes and ego/target bound sweeps | [ESTIMATOR_IN_THE_LOOP_ADMISSION_20260917.md](ESTIMATOR_IN_THE_LOOP_ADMISSION_20260917.md) |
| 50 ms control period trial on the S-bend: cruise qualifies, admission frames double | [CONTROL_PERIOD_50MS_20260917.md](CONTROL_PERIOD_50MS_20260917.md) |
| Hold-node safety certificate adoption: semantics, inter-node clearance diagnostic and frame times | [NODE_CERTIFICATE_ADOPTION_20260917.md](NODE_CERTIFICATE_ADOPTION_20260917.md) |
| Implementation-only controller runtime reduction, strict oncoming qualification and residual restoration bottleneck | [CONTROLLER_RUNTIME_OPTIMIZATION_20260917.md](CONTROLLER_RUNTIME_OPTIMIZATION_20260917.md) |
| Overlap-aware admission, equivalent sparse solving, controller timing and joint uncertainty diagnosis | [OVERLAP_ADMISSION_IMPLEMENTATION_20260916.md](OVERLAP_ADMISSION_IMPLEMENTATION_20260916.md) |
| Overlap-aware support proposal: valid constraint construction and remaining temporal/dynamic infeasibility | [OVERLAP_SUPPORT_PROPOSAL_REVIEW_20260916.md](OVERLAP_SUPPORT_PROPOSAL_REVIEW_20260916.md) |
| NRMM derivative-noise reduction with retained continuous decay and paired validation | [NRMM_NOISE_REDUCTION_20260916.md](NRMM_NOISE_REDUCTION_20260916.md) |
| Ordinary-distance degeneracy, first-admission failure and current hard-feasible witnesses | [DISTANCE_DUAL_INITIALIZATION_DIAGNOSIS_20260916.md](DISTANCE_DUAL_INITIALIZATION_DIAGNOSIS_20260916.md) |
| NRMM estimator: tracking accuracy, startup/reacquisition peaks, uncertainty and runtime | [NRMM_ESTIMATOR_EVALUATION_20260916.md](NRMM_ESTIMATOR_EVALUATION_20260916.md) |
| NRMM estimator multi-scenario campaign: ego/target maneuvers, oncoming pass, noise, offsets and radar gaps | [NRMM_ESTIMATOR_SCENARIOS_20260917.md](NRMM_ESTIMATOR_SCENARIOS_20260917.md) |
| Distance-dual-only cleanup, explicit admission failures and current regression results | [DISTANCE_DUAL_ONLY_RESULTS_20260916.md](DISTANCE_DUAL_ONLY_RESULTS_20260916.md) |
| Distance-dual adoption, initialization, recursive inclusion and straight-scene validation | [DISTANCE_DUAL_ADOPTION_20260916.md](DISTANCE_DUAL_ADOPTION_20260916.md) |
| Whole-hold certificates, actuator-only operating limits and straight-scene validation | [WHOLE_HOLD_ACTUATOR_CONSTRAINTS_20260916.md](WHOLE_HOLD_ACTUATOR_CONSTRAINTS_20260916.md) |
| Real-time feasibility, admission versus active convex solves, and equivalent row reduction | [REALTIME_FEASIBILITY_20260916.md](REALTIME_FEASIBILITY_20260916.md) |
| Finite convex branch admission, straight-scene validation and timing limits | [FINITE_CONVEX_BRANCH_RESULTS_20260916.md](FINITE_CONVEX_BRANCH_RESULTS_20260916.md) |
| Nonlinear oncoming avoidance witness, continuous verification and restrictive collision geometry | [ONCOMING_AVOIDANCE_WITNESS_20260916.md](ONCOMING_AVOIDANCE_WITNESS_20260916.md) |
| Oncoming first-admission LP ablations and numerical infeasibility certificate | [ONCOMING_ADMISSION_DIAGNOSIS_20260916.md](ONCOMING_ADMISSION_DIAGNOSIS_20260916.md) |
| Complete conditional recursive-feasibility construction and straight-scene validation | [RECURSIVE_FEASIBILITY_CLOSURE_20260915.md](RECURSIVE_FEASIBILITY_CLOSURE_20260915.md) |
| Predictive continuation restoration, carried feasibility, 30 s straight trials and runtime limits | [PREDICTIVE_CONTINUATION_RESTORATION_20260915.md](PREDICTIVE_CONTINUATION_RESTORATION_20260915.md) |
| Single-hold decay-row obstructions, uncertainty correlation and blocked passing geometry | [SINGLE_HOLD_INFEASIBILITY_ANALYSIS_20260915.md](SINGLE_HOLD_INFEASIBILITY_ANALYSIS_20260915.md) |
| CLF slack, optional road boundaries, and remaining hard-input infeasibility | [CLF_SLACK_NO_ROAD_RERUN_20260915.md](CLF_SLACK_NO_ROAD_RERUN_20260915.md) |
| Optimized CLF slack, preserved hard safety, failure diagnosis and runtime | [SOFT_CLF_CONTROLLER_RESULTS_20260915.md](SOFT_CLF_CONTROLLER_RESULTS_20260915.md) |
| One hard optimization per sample, explicit failure, sampled dissipation and timing | [SINGLE_SOLVE_CONTROLLER_RESULTS_20260915.md](SINGLE_SOLVE_CONTROLLER_RESULTS_20260915.md) |
| Hard-constrained solver execution, sampled cruise CLF and runtime results | [HARD_CONSTRAINED_CONTROLLER_RESULTS_20260915.md](HARD_CONSTRAINED_CONTROLLER_RESULTS_20260915.md) |
| Independent version-23 straight scenes, hard cruise dissipation and runtime audit | [CONTROLLER_V23_RERUN_20260914.md](CONTROLLER_V23_RERUN_20260914.md) |
| Sampled backup CBF, hard cruise CLF, and runtime validation | [SAMPLED_BACKUP_CBF_CLF_RESULTS_20260914.md](SAMPLED_BACKUP_CBF_CLF_RESULTS_20260914.md) |
| Independent version-22 straight scenes and 90 s recovery audit | [CONTROLLER_V22_RERUN_20260914.md](CONTROLLER_V22_RERUN_20260914.md) |
| Terminal-speed, passing-search and work-budget repairs | [CONTROLLER_REPAIR_RESULTS_20260914.md](CONTROLLER_REPAIR_RESULTS_20260914.md) |
| Exact-state experiment: terminal-domain defect and remaining controller problems | [EXACT_STATE_CONTROLLER_AUDIT_20260914.md](EXACT_STATE_CONTROLLER_AUDIT_20260914.md) |
| Straight-scene rerun after finite encounter completion | [ALGORITHM_RERUN_RESULTS_20260914.md](ALGORITHM_RERUN_RESULTS_20260914.md) |
| Bounded target motion: admission fix and terminal limitations | [BOUNDED_TARGET_MOTION_RESULTS_20260913.md](BOUNDED_TARGET_MOTION_RESULTS_20260913.md) |
| Controller design-requirement experiments | [CONTROLLER_DESIGN_EXPERIMENTS.md](CONTROLLER_DESIGN_EXPERIMENTS.md) |
| Why avoidance and cruise recovery stopped | [CONTROLLER_FAILURE_ANALYSIS.md](CONTROLLER_FAILURE_ANALYSIS.md) |
| Curved-controller validation (September 8, 2026) | [CURVED_CONTROLLER_RESULTS.md](CURVED_CONTROLLER_RESULTS.md) |
| Finite encounter completion implementation and validation | [FINITE_COMPLETION_RESULTS_20260913.md](FINITE_COMPLETION_RESULTS_20260913.md) |
| Finite-sensing integration repair: experimental results | [FINITE_SENSING_REPAIR_RESULTS.md](FINITE_SENSING_REPAIR_RESULTS.md) |
| Removing the additional tire-force constraints | [FORCE_CONSTRAINT_REMOVAL_RESULTS.md](FORCE_CONSTRAINT_REMOVAL_RESULTS.md) |
| Information-state PCBF validation | [INFORMATION_STATE_PCBF_RESULTS_20260912.md](INFORMATION_STATE_PCBF_RESULTS_20260912.md) |
| Estimator-controller straight experiment: admission failure | [JOINT_RERUN_RESULTS_20260913.md](JOINT_RERUN_RESULTS_20260913.md) |
| Joint simulation failure analysis | [JOINT_SIMULATION_FAILURE_ANALYSIS.md](JOINT_SIMULATION_FAILURE_ANALYSIS.md) |
| NRMM implementation notes and recorded experiments | [NRMM_IMPLEMENTATION_NOTES.md](NRMM_IMPLEMENTATION_NOTES.md) |
| Estimator and controller pipeline runtime attribution | [PIPELINE_RUNTIME_ANALYSIS.md](PIPELINE_RUNTIME_ANALYSIS.md) |
| Real-time round one: measured outcome | [REAL_TIME_PCBF_RESULTS_20260912.md](REAL_TIME_PCBF_RESULTS_20260912.md) |
| Real-time round two: measured outcome | [REAL_TIME_ROUND_TWO_RESULTS_20260912.md](REAL_TIME_ROUND_TWO_RESULTS_20260912.md) |
| Rolling MPC terminal-certificate correction | [ROLLING_TERMINAL_RESULTS_20260911.md](ROLLING_TERMINAL_RESULTS_20260911.md) |
| Straight avoidance: controller-first diagnosis and repair | [STRAIGHT_CONTROLLER_REPAIR_RESULTS.md](STRAIGHT_CONTROLLER_REPAIR_RESULTS.md) |
| Straight avoidance with scheduled computation delay | [STRAIGHT_DELAYED_EXECUTION_RESULTS.md](STRAIGHT_DELAYED_EXECUTION_RESULTS.md) |
| Straight encounter validation, September 9, 2026 | [STRAIGHT_ENCOUNTER_VALIDATION.md](STRAIGHT_ENCOUNTER_VALIDATION.md) |
| Straight-road rerun of the exact two-vehicle controller | [STRAIGHT_EXACT_STATE_RESULTS_20260911.md](STRAIGHT_EXACT_STATE_RESULTS_20260911.md) |
| Straight avoidance: complete-frame runtime optimization | [STRAIGHT_REALTIME_RESULTS.md](STRAIGHT_REALTIME_RESULTS.md) |
| Independent straight-road rerun of the information-state controller | [STRAIGHT_RERUN_RESULTS_20260912.md](STRAIGHT_RERUN_RESULTS_20260912.md) |
| Straight controller rerun after the real-time changes | [STRAIGHT_RERUN_RESULTS_20260913.md](STRAIGHT_RERUN_RESULTS_20260913.md) |
| Terminal CBF proof audit and removal of retired code | [TERMINAL_CBF_PROOF_RESULTS.md](TERMINAL_CBF_PROOF_RESULTS.md) |
| Visible-target lifecycle and no-target joint cruise | [VISIBLE_TARGET_LIFECYCLE_RESULTS_20260913.md](VISIBLE_TARGET_LIFECYCLE_RESULTS_20260913.md) |
