# Declared-plant rerun after removing the curved-road chart boxes, 2026-09-23

Commit `09f29f61b3fb85ffcda0b4339e8a35758fabef40` removes the per-node and terminal
pose-domain rows and the domain screening, by project decision, without
replacing the chart linearization allowance. This rerun measures the effect on
the declared held affine plant. Measured outcomes only; no proof or real-time
claim. Timings are not isolated (other MATLAB sessions were running).

Environment: clean worktree of the commit, MATLAB R2026a batch, one experiment
at a time. Baseline: [CONTROLLER_VALIDATION_RERUN_20260923.md](CONTROLLER_VALIDATION_RERUN_20260923.md)
at `0b0855f`.

## Failure-mode sweep (118 cases, 240 holds, deadline disabled)

| Outcome | Before (with boxes) | After (no boxes) |
| --- | --- | --- |
| Passed | 95 | 102 |
| Controller error | 15 | 7 |
| Inter-node or node overlap | 8 | 9 |

Per-case table: [CONTROLLER_FAILURE_MODE_SWEEP_BOXFREE_20260923.csv](CONTROLLER_FAILURE_MODE_SWEEP_BOXFREE_20260923.csv).
Only the eight cases blocked by the chart boxes in
[ADMISSION_INFEASIBILITY_DIAGNOSIS_20260923.md](ADMISSION_INFEASIBILITY_DIAGNOSIS_20260923.md)
changed. Every other case keeps its outcome class, hold count and failure
identifier.

| Case | Before | After | Minimum node gap | Minimum sampled gap |
| --- | --- | --- | --- | --- |
| 27 crossing, curvature -0.02 | infeasible at t = 0 | pass, 240 holds | 0.190 m | 0.184 m |
| 48 oncoming, 0.01, uncertainty x10 | infeasible at t = 2.6 s | pass | 0.310 m | 0.308 m |
| 53 crossing, 0.01, uncertainty x3 | infeasible at t = 0 | pass | 1.352 m | 1.348 m |
| 94 stationary, 0.01, initial lateral 2 m | infeasible at t = 0 | pass | 0.134 m | 0.058 m |
| **96 stationary, 0.01, initial heading 0.1 rad** | infeasible at t = 0 | **completes 240 holds with a collision** | **-0.0062 m** | **-0.0138 m** |
| 97 stationary, 0.01, initial heading 0.2 rad | infeasible at t = 0 | pass | 0.113 m | 0.113 m |
| 98 stationary, 0.01, initial heading 0.3 rad | infeasible at t = 0 | pass | 0.126 m | 0.124 m |
| 99 stationary, 0.01, initial speed -2 m/s | infeasible at t = 0 | pass | 0.098 m | 0.098 m |

The three ten-fold-uncertainty cases (39, 42, 54) and the four road-boundary
cases still fail as before.

## Case 96: the certificate is violated at a node

Before this change the node gap was positive in all 118 cases: the certificate
held where it applies. In case 96 the gap is negative at a hold node, so the
controller certified a plan that actually overlaps the target at that node.
The accepted plans were rebuilt and compared with the exact curved-road map
([chartError96.txt](CHART_BOX_REMOVAL_RERUN_20260923/chartError96.txt),
[checkChartError.m](CHART_BOX_REMOVAL_RERUN_20260923/checkChartError.m)).
The planned nodes lie up to 9.8-10.2 units outside the former box around the
seed, and the true chart error reaches about 8 times the remainder charged in
the collision records. This is the consequence stated with the removal:
outside `poseTrustRadius` of the seed, the fixed chart allowance no longer
bounds the chart error.

## Recursion campaign (straight roads)

`runRecursiveSafetyValidation`: all five cases pass 600/600 holds with values
identical to the baseline to the printed precision; straight roads never had
pose-domain rows. Deadline runs complete 0/120, 120/120 and 120/120 holds. The
oncoming deadline case was 52/120 before; its failure was a 51 ms frame against
the 50 ms deadline, so the change is within timing variance on this shared
machine and is not attributed to the commit.

## Conclusion

Removing the boxes lets the optimizer leave the seed neighborhood, which
resolves 7 of the 8 box-blocked cases with positive clearance. It also makes
the curved-road certificate unsound once the plan leaves that neighborhood,
and case 96 shows an actual node overlap of 6.2 mm (13.8 mm between nodes).
A deviation-dependent chart allowance, or re-linearizing the chart around the
accepted plan, would keep the certificate valid without a box.
