# Shared trajectory reference for model and collision geometry

Date: September 27, 2026.

## Implemented behavior

Active trajectory-linearized encounters now use the same nonlinear rollout
nodes for dynamics and tire operating points, curved-road pose charts, nominal
rectangle separation directions, terminal exit direction, and the yaw expansion
of collision support constraints. The input-deviation objective remains centered
on that rollout's inputs. This applies to the shifted previous plan and to each
bounded flow-initialization candidate, within the existing controller.

Previously, geometry propagated the same inputs through the frozen affine model.
Those states generally differ from the nonlinear rollout used for the model
Jacobians. Reusing the same input vector did not ensure a shared state reference.
The solver also had an independent phase-I direction-restoration path. Fresh
trajectory frames now keep their reference directions fixed; trying the other
passing side rebuilds the reference and all dependent model/geometry data.

The optimization still evolves the frozen affine model. Changing its numerical
coordinate origin does not change the collision expansion point. For reference
yaw `psiBar`, fixed normal angle `alpha`, and optimized yaw `psi`, the rectangle
support majorant uses the first-order rotation at `alpha - psiBar` plus the global
quadratic remainder `0.5 * norm(egoHalfSize) * (psi - psiBar)^2`. The constant yaw
displacement between the solver's coordinate origin and `psiBar` is included in
both terms. Carried certificates retain their original expansion states.

The previous reference also retains its terminal node. A newly estimated,
shorter encounter horizon cannot truncate the remaining previous plan. The new
model still requires a new hard solve; retaining the reference length is not a
claim of transferred feasibility.

## Regression investigation

The initial geometry-only integration passed 253 of 254 regression cases. The
circular crossing scenario stopped at 2.30 s after 46 holds, with a minimum
sampled body gap of 1.39083 m. The saved failure frame had a 64-hold horizon.
Offline constraint ablations found the base problem and the problem without its
exit record feasible, while replacing only yaw expansion states or obstacle
normals with their old affine counterparts did not restore feasibility.
No diagnostic constraint deletion was applied to the controller.

Preserving the previous reference through its terminal node restored this
scenario's full 140-hold execution and confirmed target release. The isolated
rerun recorded a minimum sampled body gap of 0.172559 m. The regression now also
checks that an active encounter's remaining horizon decreases by at most one
hold per frame. This result uses the declared affine plant; it is not a new
PassVeh14DOF closed-loop result or evidence of real-time qualification.

## Validation and evidence

All 256 tests in 16 classes passed, with zero failed or incomplete cases.
Factory Code Analyzer reported no new diagnostics in the ten inspected MATLAB
files. Seven files were clean; eleven existing `AGROW`, `SPRIX`, and `FNDSB`
advisories across three files match the baseline identifiers, messages and
source statements.

The final regression covers the controller, continuation, trajectory/flow
linearization, collision certificates, input-deviation objective, reactive
prediction, native frame adapter, admission safety, deadlines, and source budget.
New geometry assertions compare direct nonlinear-rollout positions and support
normals for first flow, shifted previous, and alternate flow references. The
fixed-plan conic tests compare independent analytic majorants across fixed,
interval, and full yaw uncertainty; positive/negative clearance; displaced
reference yaw; and changed solver coordinate origins. Rejected flow solves are
checked to avoid independent direction restoration.

Original logs, test results, diagnostic snapshots, and the exact MATLAB driver
are stored outside the repository in
`/home/zai/.cache/collisionAvoidance/flow-geometry-20260927/`. The driver is run with
`matlab -batch "run('/home/zai/.cache/collisionAvoidance/flow-geometry-20260927/verify.m')"`.
It uses MATLAB R2026a and one computational thread. The crossing scenario uses
seed 20260912, curvature 0.01/m, reference speed 8 m/s, 50 ms holds, 140 requested
holds, and 30 s search/deadline budgets. Road boundaries are disabled in that
existing test fixture; no production road constraint was removed.

The earlier physical-plant failures and runtime limitations are not resolved
by these regression results. Hard support acceptance is still evaluated on the
frame's frozen affine prediction, not on a nonlinear-plant certificate.
