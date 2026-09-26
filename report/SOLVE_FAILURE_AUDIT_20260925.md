# Remaining trajectory-solve failures — September 25, 2026

The remaining straight and S-curve failures contain demonstrably inconsistent
constraint subsets. They are not explained solely by a Clarabel status or by
the previously repaired feedback-enclosure explosion. Their immediate
conflicts differ, but the preceding accepted plans also exhibit substantial
affine/nonlinear mismatch, including tire tangents extrapolated far outside
their useful neighborhood. Production controller behavior is unchanged.

## Frames and method

Base revision: `070233f896d7e4c8596d6844ac54261e8f68477a`.
Use the exact saved rejected frames from the
[feedback repair validation](TRAJECTORY_FEEDBACK_FIX_20260925.md): straight at
2.05 s and S-curve at 1.40 s. Their source replays previously reproduced every
issued input exactly. This audit loads those snapshots; it does not rerun the
18 s plant campaign or claim new successful avoidance.

These are truth-fed PassVeh14DOF runs with 50 ms holds, requested ego/target
speed 10 m/s, initial distance 50 m, visibility 30 m, and first observation at
1.05 s. The S-curve uses peak curvature 0.01/m and wavelength 80 m. No stochastic
sensor seed applies. Remaining prediction horizons are 56 and 72 holds.

The diagnostic script rebuilds the previous-plan and selected fluid-reference
fixed-direction SOCPs, sets the objective to zero, and removes selected groups
after assembly. Dynamics and the actual tightened actuator rows remain unless
specified. Collision separation rows may be removed while their now harmless
support epigraphs remain. Station-phase constraints are separated from the
80 m lateral regularity domain although the production labels combine them.
All these changes are confined to diagnostic copies; no resulting command is
issued. A soft CLF cannot establish infeasibility merely through its cost.

Independent necessary LP relaxations replace each Lorentz disk by coordinate
bounds. These relaxations contain the original feasible set. A positive
minimum required relaxation, accompanied by matching primal/dual LP values,
therefore rules out feasibility of the original subset, within the recorded
floating-point residuals. This is stronger evidence than solver status alone.

## Immediate conflicts

| Constraint experiment | Straight, 2.05 s | S-curve, 1.40 s |
| --- | --- | --- |
| Full constraints, zero objective | Rejected | Rejected |
| Remove road rows | Still rejected | Still rejected |
| Remove collision rows | Feasible | No feasible result |
| Remove terminal cones | Still rejected | Feasible |
| Remove station-phase rows only | No such rows | Feasible |
| Actuator + collision only | Inconsistent | Feasible |
| Actuator + terminal only | Feasible | Feasible |
| Actuator + station phase + terminal | Feasible | Inconsistent |

The table is supported by both audited direction seeds. Feasible witnesses
have tiny directly evaluated cone residuals; native statuses 1 and 4 are
distinguished in the CSV. Some reduced problems produce numerical statuses
5 or 10, so they are not treated as proofs on their own. The positive LP dual
bounds below establish the inconsistent subsets independently.

### Straight: conflicting avoidance requirements under bounded inputs

With road, exit and terminal requirements removed, the previous-plan
fixed-direction collision subset still requires a common separation-row
relaxation of **0.925240 m**. Its necessary LP relaxation requires at least
**0.924390 m**, with primal violation 2.00e-14, dual stationarity residual
3.83e-14, and matching dual bound. The LP's active collision multipliers occur
at prediction stages 9 and 10, absolute times **2.50 and 2.55 s**.

The selected opposite-side fluid seed is also inconsistent: the SOCP requires
2.770192 m, and the necessary LP lower bound is 2.652059 m. Active LP collision
stages are 5, 15 and 26. The positive slack measures a deficit in the selected
conservative separation certificates; it is not measured vehicle penetration
or a prescription to reduce safety clearance. Removing terminal requirements
or increasing objective weights cannot repair this already inconsistent
actuator/collision subset.

This does not prove every possible set of normals, nonlinear trajectory, or
earlier action is infeasible. It establishes why these particular convex
problems cannot be solved with their constraints intact. The current ego
lateral position is only -0.0860 m; the last executed prefix's minimum SAT
margin was 4.0198 m. Neither fact alone establishes inevitable physical impact.

### S-curve: the progress schedule conflicts with terminal recovery

The controller imposes a whole-horizon station corridor
`abs(s_k - sReference_k) <= 2 m`, in addition to the final scheduled terminal
set. The reference advances on the cruise schedule. This is stronger than
eventually returning to the supplied path at the requested speed: it also
restricts how far the vehicle may fall behind or move ahead during avoidance.

Actuator plus terminal constraints alone are feasible. Actuator plus collision
alone are also feasible. The inconsistent subset is **actuator + station
corridor + terminal**, even without obstacles, road boundaries or target exit.
Its necessary terminal-box LP requires a uniform station-corridor enlargement
of at least **0.658083 m**. Primal violation is 2.73e-12, dual stationarity
residual 8.53e-14, and primal/dual bounds agree to 5e-13.

Independent MATLAB `coneprog` with the actual modal disks estimates the
subset minimum enlargement at **0.854530 m** (exit flag 1, maximum unscaled
primal violation 3.04e-7). In that best compromise the upper station limit is
active near 4.05 s: predicted station about 43.348 m versus an original upper
limit about 42.501 m, with arithmetic tightening also included. Thus this
conflict is not simply "the car is too slow"; satisfying the frozen dynamics
and final recovery can require violating the intermediate progress corridor.

This is a subset calculation, not a recommendation to change 2 m to 2.855 m,
nor proof that such a change solves the full nonlinear experiment. The
road/obstacle constraints and the meaning of the scheduled terminal guarantee
would still need to be considered.

## Upstream mismatch: a trajectory tangent is still only local

The modified Fiala nonlinear force correctly respects its combined-slip
capacity. On a saturated slip branch it has the form

`Fy = -mu*Fz*sqrt(1-beta^2)*sign(alpha)`.

The implemented joint derivative includes

`dFy/dBeta = mu*Fz*beta/sqrt(1-beta^2)*sign(alpha)`.

As `abs(beta)` approaches 1, the lateral capacity approaches zero while this
derivative becomes very large. Here beta is the **signed longitudinal tire
force utilization**, not a brake-pedal percentage; positive/negative values
follow the model's longitudinal-force convention. On a saturated branch the
slip-angle derivative is zero, so a frozen tangent also fails to represent a
large steering change that crosses to the opposite force branch.

The one-pass trajectory method updates the operating point for each hold, but
then allows the optimizer to change inputs substantially without updating
that frame's tangents. Merely using a nonlinear law to compute Jacobians does
not impose that law's global force limit on the affine extrapolation.

Concrete values from the last **accepted** plans, using their exact saved
tire matrices and optimized states/inputs:

| Quantity, front axle | Straight plan at 2.00 s, hold 1 | S-curve plan at 1.35 s, hold 30 |
| --- | ---: | ---: |
| Anchor beta | 0.9999294 | 0.9999840 |
| Optimized beta | 0.9237388 | 0.9762721 |
| dFy/dBeta (N per unit beta) | +547478 | -1148576 |
| Anchor steering (rad) | -0.408070 | +0.407383 |
| Optimized steering (rad) | +0.698122 | +0.698122 |
| Affine force (kN) | **-41.790** | **+27.272** |
| Nonlinear Fiala at the same candidate point (kN) | **+2.492** | **+1.409** |
| Candidate combined-slip capacity (kN) | 2.492 | 1.409 |

The straight example is particularly direct: its state change is zero at the
first hold, and the front slip slope is zero. The beta change alone adds
-41.713 kN to the -0.077 kN anchor force. Meanwhile steering moves from about
-23.4 to +40 degrees, reversing the nonlinear force's sign. This is not an
error in the saturation formula; it is invalid extrapolation of its local
tangent, already affecting the first issued hold's prediction.

The saved plan rollouts show the consequence within the controller's own
nonlinear bicycle model, before introducing additional 14-DOF plant mismatch:

| Last accepted plan | Affine final lateral position | Nonlinear final lateral position | Affine final speed | Nonlinear final speed |
| --- | ---: | ---: | ---: | ---: |
| Straight, 2.00 s | 0.000459 m | 8.438515 m | 10.0001 m/s | 8.4721 m/s |
| S-curve, 1.35 s | -0.071705 m | 7.178738 m | 10.0071 m/s | 3.6271 m/s |

These are **open-loop diagnostic rollouts of stored input sequences**, not
new measured closed-loop outcomes or simulations of every possible feedback
correction. They use the saved initial predicted state and the failed frame's
lane/configuration/acceleration bias. The direct tire-force comparison above
does not require that rollout-context assumption. S-curve final station is
49.0554 m in the accepted affine prediction but 33.1923 m in the nonlinear
rollout; maximum lateral/yaw differences are 7.2504 m and 1.9364 rad.

At the subsequent rejected frames, the repaired feedback tube's maximum
lateral radii are only 9.70e-7 and 9.26e-7 m. Those radii propagate the modeled
affine disturbances; they do **not** certify the nonlinear Taylor remainder
demonstrated here. The feedback-gain repair remains valid for its tested
held-affine family, but cannot make these two prediction models agree.

This supports the causal interpretation that the earlier affine plans can
promise avoidance/recovery they cannot realize under the nonlinear model.
Fresh relinearization then exposes a different feasible set and can destroy
the preceding frame's apparent feasibility. The diagnostic data establishes
the mismatch and the new constraint conflicts; it does not quantify how much
each nonlinear effect contributed to every preceding control decision.

## Circular case and next decision

The circular run's 1.50 s error remains a different failure: a future anchor
slip reaches 1.6644 rad, exceeding the existing `abs(alpha)<pi/2` domain before
the SOCP is called. It should not be counted as a solver infeasibility.
This audit retains the prior captured evidence; it does not rerun that case.

The next engineering priority is the validity of the joint tire tangent near
longitudinal saturation and across large steering changes. Independently,
the S-curve whole-horizon cruise-phase requirement needs to be reconciled
with the desired avoidance/recovery behavior and its terminal certificate.
Changing solver iterations, CLF weights or feedback gains alone does not
address the certified inconsistent subsets. No additional linearization
iterations, trust regions, force constraints, nonlinear acceptance, terminal
redesign or relaxed production safety rows are introduced by this audit.

## Reproduction and validation

```matlab
addpath('scripts');
summary = diagnoseTrajectorySolveFailures( ...
    '/home/zai/.cache/collisionAvoidance/trajectory-feedback-fix-20260925', ...
    '/home/zai/.cache/collisionAvoidance/solve-failure-audit-20260925');
```

MATLAB R2026a Update 3; existing native Clarabel solver; MATLAB `linprog` and
`coneprog`. The completed diagnostic run executes 36 feasibility ablations,
two minimum collision-relaxation SOCPs and independent LP dual checks, one
station-relaxation LP/SOCP pair, and two nonlinear nominal rollouts. Embedded
assertions check statuses and unscaled primal/dual residuals of the decisive
calculations. No controller test suite was rerun because production code is
unchanged. Factory Code Analyzer reports six array-growth performance suggestions in the diagnostic script and no other findings. Exploratory numerical failures were not interpreted as proofs;
the final findings use the saved passing diagnostics and verified bounds.

Compact CSV/JSON exports and a source/raw-data manifest are in
`SOLVE_FAILURE_AUDIT_20260925/`. Raw diagnostic MAT programs, witnesses and LP
duals remain in the external cache above. No generated binaries, solver
dependencies or unrelated observer/scenario edits are included in this work.
