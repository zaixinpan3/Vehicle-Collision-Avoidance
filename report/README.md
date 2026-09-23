# Experiment reports

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
