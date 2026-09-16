# Experiment reports

Experiment results, validation reports, runtime analyses, and recorded implementation findings live here. Executable experiment drivers remain in `../scripts/`.

Latest implementation and validation: [Distance-dual convexification, straight-scene recovery and remaining timing limits, September 16, 2026](DISTANCE_DUAL_ADOPTION_20260916.md).

Latest runtime assessment: [Real-time feasibility, active-encounter bottlenecks and equivalent row-reduction benchmark, September 16, 2026](REALTIME_FEASIBILITY_20260916.md).

Latest diagnosis: [Verified nonlinear oncoming avoidance exists despite the rejected admission constraints, September 16, 2026](ONCOMING_AVOIDANCE_WITNESS_20260916.md).

Latest independent rerun: [CLF slack without artificial road boundaries, September 15, 2026](CLF_SLACK_NO_ROAD_RERUN_20260915.md).

Latest failure analysis: [Single-hold infeasibility, shared uncertainty and passing geometry](SINGLE_HOLD_INFEASIBILITY_ANALYSIS_20260915.md).

Each report states its experiment date and scope. Earlier results describe the revision tested at that time.

| Report | File |
| --- | --- |
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
