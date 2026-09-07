# Why avoidance and cruise recovery stopped

This report records experiments and diagnostics from before the modified Fiala
tire revision. Its linear-tire and friction-limit results describe that earlier
controller. The current model uses scheduled Fiala tangents and no separate
axle-friction constraints; see [LTV_BICYCLE_MODEL.md](../controller/LTV_BICYCLE_MODEL.md).

Analysis date: September 5, 2026. Examined implementation:
`69a56a2bce56c20e48bf811d3449c9f4e541ae7e`.
Input experiments and acceptance definitions are in
[CONTROLLER_DESIGN_EXPERIMENTS.md](CONTROLLER_DESIGN_EXPERIMENTS.md).
Subsequent concurrent controller edits are outside this diagnosis. The
numerical analysis and assertions completed before those edits began.

The immediate failure is an infeasible **selected convex approximation** at
3.80 s, not an observed vehicle collision or evidence that every safe maneuver
is impossible. Reoptimizing from the shifted previous plan at the same actual
state succeeds in both scenes, with every original constraint family retained.
The completed diagnosis does not constitute a repaired controller or a new
completed high-fidelity avoidance run.

## Evidence and method

The analysis replayed all 77 recorded controller attempts in each target-aware
trial, using the saved ego states, target estimates, perceived road geometry,
and Blockset-derived configuration. Non-pausing conditional MATLAB breakpoints
captured the previous certificate, assembled models, both candidate problems,
and solver results without changing controller source. Both original failures
were reproduced. MATLAB was R2026a Update 3.

For each of the four failed candidate programs, a separate `linprog` solve
tested the hard affine inequalities and input bounds. The CLF relaxation is
unbounded above, so it cannot repair or cause infeasibility of these hard rows.
An independent numerical Farkas witness also included all finite box bounds:

```text
Abar = [A; -I_lower; I_upper],  bbar = [b; -lb_finite; ub_finite]
lambda >= 0,  sum(lambda) = 1,
Abar' lambda = 0,  bbar' lambda < 0.
```

A feasible decision would imply `0 <= bbar' lambda`, contradicting the last
inequality. The computed negative values were -0.02901/-0.03461 for the straight
scene and -0.03251/-0.03697 for the arc. Equality residuals ranged from
3.56e-15 to 1.18e-13. These are numerical witnesses with residual checks, not
interval-arithmetic certificates. All four primal LPs also returned infeasible.
LP feasibility and optimality tolerances were 1e-9. The production SOCP retained
the experiment's 1e-6 tolerances.

Constraint-family removals below were diagnostic LPs only; they did not produce
controller commands. Alternative seeds were passed through the original
`formulateAvoidanceProblem` and `solveHardCbfClf`, retaining collision, road,
friction, speed, heading, terminal-segment, terminal-handoff, and tail constraints.
Their predicted poses were converted back to Cartesian coordinates and checked
with the independent `rectangleSeparationMargin` at every head and tail node.
No inter-sample collision condition was used.

MathWorks documents the solver contracts in
[linprog](https://www.mathworks.com/help/optim/ug/linprog.html) and
[coneprog](https://www.mathworks.com/help/optim/ug/coneprog.html).
The causal findings below are measurements from this repository.

## 1. The controller repeatedly loses its avoidance nominal

`localCertificateCompatible` requires identity equality and agreement between
the measured state and the previous predicted next state within
`1e-8 * (1 + max(abs(predicted), abs(measured)))` componentwise.
If this test fails, the orchestrator clears the previous certificate **and
also loses the previous plan as a linearization seed**. `localNominalInput`
then returns `prediction.referencePlan` with source `scheduleReference`.
The label `shiftedPlan` on the first candidate does not establish that a
previous plan was actually shifted.

In each replay, all 76 available consecutive lane comparisons matched.
Episode identity matched 75 times; target acquisition explains the remaining
identity change. Every available next-state comparison exceeded the required
tolerance. Successful original target-visible calls all recorded
`nominalSource = scheduleReference`. The state residuals at failure were:

| Measured minus predicted next state | Straight | Arc |
| --- | ---: | ---: |
| Station [m] | 0.00003065 | 0.00000443 |
| Lateral offset [m] | -0.0019063 | -0.0016465 |
| Heading error [rad] | -0.0011579 | -0.0010584 |
| Longitudinal speed [m/s] | -0.0121134 | -0.0114024 |
| Lateral speed [m/s] | -0.0204634 | -0.0089691 |
| Yaw rate [rad/s] | -0.0300972 | -0.0256073 |

The exact-prediction reuse premise is not satisfied by the 14-DOF plant. Rejecting
an old safety certificate is appropriate in that situation; discarding its
trajectory as a starting point for a fresh hard-constrained optimization is
unnecessary. These are different uses of stored information.

The original second start is obstacle-free Riccati cruise feedback. It differs
numerically from the schedule reference, but both are close to cruise and
select essentially the same failing obstacle-side geometry. They do not search
all possible ways to pass or brake before the target.

## 2. The selected tail plane demands an unreachable lateral maneuver

Only the first 24 stages, or 1.2 s, contain dynamic steering decisions. The
subsequent 74-stage braking tail holds the terminal lateral offset in a
certified envelope. Consequently, a lateral requirement at a later tail node
must already be achieved and stabilized by the end of the 1.2 s head.

At the failed sample, the relevant tail node is MATLAB node 48, corresponding
to prediction step 47, 2.35 s ahead, or global time 6.15 s. The original seed
has approached the target's configuration obstacle. Its closest-face normal
changes from nearly longitudinal `[-1;0]` at the preceding node to nearly
lateral `[-0.01487;-0.99989]` in the straight scene, and
`[-0.01621;-0.99987]` in the arc. Freezing this normal imposes a specific
lateral passing requirement on all decisions in that candidate.

The handoff at global time 5.00 s must also satisfy:

| Handoff condition | Straight | Arc |
| --- | ---: | ---: |
| Maximum absolute heading error [rad] | 0.0446821 | 0.0321821 |
| Maximum absolute lateral speed [m/s] | 0.15 | 0.15 |
| Maximum absolute yaw-rate error `r-kappa*vx` [rad/s] | 0.05 | 0.05 |

With all non-collision rows and bounds retained, minimizing the violation of
that **single** collision plane still leaves 0.111642 m of unavoidable
shortfall in the straight case and 0.123324 m in the arc. At these optimum
decisions, the straight plane requires `d <= -2.93262 m` while the attainable
value is `-2.82097 m`; the arc requires `d <= -2.83170 m` while the attainable
value is `-2.70836 m`. The required offset depends slightly on the decision's
station; these pairs are evaluated at the minimizing decisions.

The family-removal experiments agree in all four failed programs:

| Rows removed for diagnosis | Remaining hard LP |
| --- | --- |
| None | Infeasible |
| Tail collision rows, excluding the final rest-support row | Feasible |
| Friction/slip family | Feasible |
| Lateral handoff family | Feasible |
| Road family | Infeasible |
| Speed-domain family | Infeasible |
| Heading-domain family | Infeasible |
| Terminal 0.1 m segment family | Infeasible |
| Final rest collision-support row | Infeasible |
| Terminal segment and final rest-support rows together | Infeasible |

Thus the direct conflict is the selected tail collision plane against the
attainable, stabilized dynamic maneuver under tire limits. Neither road width
nor the narrow terminal segment is the direct cause of this snapshot failure.
Removing physical or safety rows is not a proposed controller correction.

## 3. The failure is avoidable at the planning level

The following counterfactuals use the identical failure state, prediction
matrices, horizon, model parameters, objective, and hard constraint families.
Only the trajectory used to choose the supporting geometry changes; this can
also change the selected terminal segment and support direction.

| Seed for a fresh solve | Straight maximum hard violation | Arc maximum hard violation | Straight minimum predicted SAT gap [m] | Arc minimum predicted SAT gap [m] |
| --- | ---: | ---: | ---: | ---: |
| Original schedule reference | Infeasible | Infeasible | N/A | N/A |
| Shifted previous plan | 8.37e-10 | 1.03e-9 | 0.737322 | 0.644007 |
| Brake at 3 m/s^2 to the 12 m/s head floor, then hold | 1.38e-9 | 1.60e-9 | 1.251853 | 1.563394 |
| Brake at 5 m/s^2 to the head floor, then hold | 1.09e-8 | 7.04e-9 | 1.584917 | 2.162475 |
| Brake at 8 m/s^2 to the head floor, then hold | 5.51e-7 | 7.94e-7 | 2.029423 | 2.233008 |

Each braking seed appends the existing fastest admissible braking-tail profile.
The seed is not the executed command: it is reoptimized by the same SOCP.
The raw shifted previous plans themselves violate hard rows by 0.762606 and
0.394532 in the formulation's mixed row scaling. They must not be executed
without renewed feasibility checks. The successful reoptimized first inputs
for the shifted seeds are `(steering, acceleration) = (0.01738, -0.12495)` and
`(0.02285, -0.09005)` in rad and m/s^2.

This establishes that the original error message describes failure of its two
implemented convex candidates. It does not establish emptiness of every safe
trajectory set. Positive predicted node gaps establish geometry for these
model trajectories, not a completed 14-DOF execution or robustness certificate.

Horizon changes alone are not a general remedy. At the same failure states,
32 head stages admit the first original candidate; 24 and 48 stages reject
both. Changing the horizon also changes the nominal stopping point and the
selected support normals, so feasibility is not monotone in this experiment.

## 4. Avoidance action was repeatedly deferred

The original plans increasingly promise a lateral maneuver near the horizon
end, but the realized vehicle remains near cruise. For example, at 3.55 s the
straight plan ends at `d = -2.132 m`; at 3.75 s it ends near `-2.529 m`.
The actual straight lateral offset at 3.80 s is only `-0.00947 m`. The arc
plan at 3.75 s ends near `-2.436 m`, while the actual path offset at 3.80 s is
`0.00389 m`. All applied longitudinal acceleration commands after acquisition
and before failure are positive, with minima 0.22356/0.22331 m/s^2. These
largely compensate the plant's resistance and preserve approximately 15 m/s;
they do not constitute executed avoidance braking.

This behavior is consistent with the objective structure: the CLF penalizes
only the executed first transition, the input cost favors cruise, and the
low-cost future tail can carry much of the braking promise. Up to 3.65 s the
reported CLF relaxation is approximately zero; substantial steering and CLF
relaxation first appear at 3.70 s. The avoidance nominal is then lost again at
the next sample. The measurements establish repeated deferral; they do not
isolate a unique optimal retuning of the cost weights. Changing the CLF price
at the already-infeasible snapshot cannot repair its hard affine rows.

## 5. The stated shift-invariance argument has a separate counterexample

Model mismatch is not the only obstacle to the stored-backup argument. Replace
the actual failure state with exactly the previous predicted next state, shift
the previous schedule according to `localShiftSchedule`, and build the seed
with the exact formula in `localNominalInput`, including its appended
`localTerminalBackupSteering`. The overlapping dynamic prediction nodes agree
with the previous certificate with maximum componentwise difference 6.25e-13.

Nevertheless, the appended dynamic stage violates the next handoff yaw-rate
bound:

| Exact-model shift check | Straight | Arc |
| --- | ---: | ---: |
| Old handoff yaw-rate error [rad/s] | approximately 0.05 | approximately 0.05 |
| New handoff yaw-rate error [rad/s] | 0.0795667 | 0.0624553 |
| Allowed absolute yaw-rate error [rad/s] | 0.05 | 0.05 |
| Normalized terminal-row violation | 0.591333 | 0.249107 |

The kinematic lateral tail law does not, in this recorded example, preserve
the dynamic bicycle's handoff yaw-rate bound when its first stage becomes an
executable dynamic stage. The exact-model shifted program can be **reoptimized**
successfully, but the shifted incumbent is not itself a feasible witness.
Therefore the current implementation does not establish automatic recursive
feasibility merely from exact state prediction and the stored terminal band.
The terminal-set/dynamic-handoff compatibility obligation remains unresolved.

A further design obligation is the 12 m/s head speed floor: while the measured
speed is at least 12 m/s, the next executable head speed is constrained to be
at least 12 m/s. The tail's ability to reach rest does not by itself establish
that the receding executable head can follow that tail all the way to rest.
This observation is not the direct 3.80 s infeasibility cause; removing the
speed-domain family did not fix that failure.

## 6. Why recovery was not completed

Recovery has no independent failing closed-loop trajectory here. Both original
trials stop at 3.80 s when no command is returned, before nominal contact at
5.80 s and before the final 9--10 s recovery window. The cruise reference stays
active throughout the single controller; there is no missing recovery-mode
switch to enable. The soft first-step CLF also does not by itself guarantee
arrival inside the chosen cruise tolerances by a fixed deadline.

The original nominal-only runs completed cruise, but that does not demonstrate
convergence from a completed avoidance maneuver. Full avoidance, subsequent
path/speed recovery, and the chosen 10 s duration must be validated after the
planning and terminal issues are corrected.

Runtime is another unmet requirement, independent of the stopping cause. All
77 controller attempts in each original avoidance trial exceeded the 50 ms
period; maxima were 1.28487 s and 1.613868 s. The offline harness does not add
these times as actuation delays, so they did not cause its 3.80 s state or
infeasibility. They prevent a present real-time claim.

## Correction priorities and validation boundary

1. Retain the previous trajectory for a fresh optimization independently of
   whether its old safety certificate remains valid. The measured-state
   agreement tolerance must not be widened to disguise plant mismatch.
2. Make the support construction retain viable passing/braking geometry when
   the original cruise-derived approximation fails. The seed counterfactuals
   demonstrate this need without introducing a different controller mode or
   safety relaxation.
3. Make terminal admission and appended control invariant for the same dynamic
   transition used when the tail enters the executable head. Reconcile the
   head speed domain with executable braking to rest before claiming the
   stored-plan safety argument.
4. Assess early avoidance progress and plant-model residuals in the single
   hard-CBF/soft-CLF design. Then rerun both full Blockset scenarios and measure
   complete node safety, recovery, and computation separately.

No production controller code or tuning was changed by this diagnosis.
The archived diagnostic scripts, captured MAT problems, witness CSV files,
alternative-plan node CSVs, `analysis_summary.json`, and the mechanism figure
preserve the actual numerical work. Assertions checked both replay failures,
constraint witnesses, alternative-plan hard residuals and Cartesian node
separation, exact-shift overlap and handoff violations, and horizon probes.
All passed. No new whole-repository unit-test or full plant-run result is claimed.
