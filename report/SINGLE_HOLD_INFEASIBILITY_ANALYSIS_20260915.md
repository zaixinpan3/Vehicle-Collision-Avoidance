# Why the no-road single-hold controller becomes infeasible

September 15, 2026. Examined revision:
`165d47286865ec966e23f75334551248e1e2f583`; controller/observer implementation
unchanged from `4c6574d1ef3df396d356d3030666e83ab7406fa3`.
This investigation adds an offline diagnostic, not a controller modification.

## Main finding

All seven failed cases have an impossible **sampled obstacle-decay row**.
Each such row is infeasible over the entire actuator amplitude box, even
without the CLF, model-domain, tire-slip or other obstacle rows. This is an
algebraic obstruction, independently corroborating Clarabel's infeasibility
status. The physical domain remains enabled in the actual experiments.

The obstruction has three causes with different scopes:

1. A one-hold position barrier delays action and does not preserve feasibility
   of the successor problem. Exact stationary/oncoming remain at cruise until
   the required braking exceeds the actuator limit.
2. Independent initial-upper/final-lower uncertainty bounds discard the shared
   initial error. That extra conservatism is sufficient to explain the immediate
   failure at all four uncertain failure states examined here.
3. Circumdisk vehicle geometry closes the lateral passing route inside the
   retained 4 m lateral model domain. It prevents completion even if the immediate
   decay-row obstruction is repaired.

These findings do not establish that the finite rectangular-vehicle task is
infeasible, or that the observer diverges. They do establish that CLF slack
and removing physical road boundaries cannot fix the present formulation alone.

The [diagnostic figure](/home/zai/.cache/collisionAvoidance/single-hold-diagnosis-20260915/infeasibility-mechanisms.png)
compares the late braking limit, rectangle/disk geometry, and redundant
uncertainty support. A matching standalone PDF is saved beside the PNG.

## Runtime reconstruction and row ablation

The diagnostic replays the previously saved eight-case campaign, including
the same seed and all ego/target noise draws for the bounded scenes. The range
pair uses its saved exact/NRMM measurement records directly. For the zero-hold
bounded crossing, the documented initial truth replaces the unsaved NaN state.

At each failure, reconstructed matrices and right-hand sides agree exactly
with those captured through the actual controller entry point, after its
trivial-row removal: maximum differences are zero in all seven cases.
Solver work deadlines are disabled only for these untimed offline checks.

For each linear row `a_delta*delta + a_beta*beta <= b`, compute

\[
g=\min_{|\delta|\le0.698131701,\,-1\le\beta\le1}
   (a_\delta\delta+a_\beta\beta)-b.
\]

A positive `g` proves row infeasibility. These bounds are wider than the
tire-slip-admissible input set, so the conclusion does not depend on tire slip.
All reported obstruction rows have zero coefficient on CLF slack.

| Failed case | Time (s) | Positive input-box gap (m) | Hard rows, no CLF | Remove only obstacle decay from hard rows | Preserve shared initial-error support |
| --- | ---: | ---: | --- | --- | --- |
| Exact stationary | 0.7 | 0.035100727 | Infeasible | Feasible | Infeasible |
| Exact oncoming | 2.9 | 0.074152414 | Infeasible | Feasible | Infeasible |
| Bounded stationary | 0.5 | 0.047662430 | Infeasible | Feasible | Feasible |
| Bounded oncoming | 2.9 | 0.222007048 | Infeasible | Feasible | Feasible |
| Bounded crossing | 0.0 | 0.024448150 | Infeasible | Feasible | Feasible |
| Range exact oncoming | 4.2 | 0.025841007 | Infeasible | Feasible | Infeasible |
| Range NRMM oncoming | 4.1 | 0.294116242 | Infeasible | Feasible | Feasible |

The linear feasibility checks retain initial obstacle membership, swept
Bernstein separation, input and slew constraints, model/chart domains and
tire slip, except for the explicitly named ablation. Feasible LP candidates
have maximum hard-row residual at most numerical zero. A finite CLF norm
slack is computed for every candidate; the unbounded soft CLF therefore adds
no feasibility obstruction. These candidates are never issued as commands.
Removing all obstacle rows also makes all seven programs feasible. Removing
all uncertainty restores the four uncertain cases at their measured centers,
but the shared-error calculation is the more informative counterfactual
because it retains their published uncertainty and finite motion bounds.

## Why braking starts too late

`collisionAvoidanceController` explicitly sets `model.horizonSteps=1`.
`formulateAvoidanceProblem` predicts only one held input. In the range driver,
the configuration still contains `horizonSteps=16`, but that value is ignored
by this online implementation. No terminal membership or feasible continuation
appears in the program. The current theory document explicitly disclaims
recursive feasibility: see `controller/SINGLE_SOLVE_CBF_CLF.md`.

For a centered straight encounter the selected normal is approximately
`[-1;0]`. Let `D` be longitudinal center separation and

\[
R=2\sqrt{2.4^2+0.95^2}+0.25=5.412363800\ \mathrm m,
\qquad b=D-R.
\]

The required sampled inequality is

\[
b_{k+1}\ge q b_k,\qquad q=e^{-2(0.1)}=0.818730753.
\]

Thus the allowed one-hold loss of separation is only `(1-q)*b_k`.
To leading order, with closing speed `v_c` and ego acceleration `a`,

\[
v_c h+\tfrac12 a h^2\le(1-q)b_k.
\]

The closing motion is order `h`, while the input correction is order `h^2`.
The exact declared flow, including road load, gives approximately
`0.041687053*beta` metres of longitudinal displacement authority per hold.
The equation above explains the mechanism; the actual audit uses the complete
matrix coefficients and enclosure reserves, not this approximation.

| Scene/frame | Barrier (m) | Allowed separation loss (m) | Required beta at zero steering |
| --- | ---: | ---: | ---: |
| Stationary, 0.6 s | 4.787227729 | 0.867777165 | beta <= 1.637195 |
| Stationary, 0.7 s | 3.987133450 | 0.722744678 | beta <= -1.842005 |
| Oncoming, 2.8 s | 9.785012255 | 1.773721803 | beta <= 4.179267 |
| Oncoming, 2.9 s | 8.184939290 | 1.483677781 | beta <= -2.778788 |

In both preceding frames, the decay constraint allows the cruise input
`beta ~= 0.0137`, which compensates road load. Recorded steering is below
`1e-18 rad` and the speed remains approximately 8 m/s. The objective has no
reward for anticipatory avoidance and no constraint requiring successor
feasibility. One frame later, even full braking cannot meet the decay rate.

Turning does not repair that row at a perfectly centered encounter: the
frozen straight affine plant has zero steering-to-longitudinal-position map,
and the selected normal has zero lateral component. The captured steering
coefficients are below `1.4e-21`, included in the input-box calculation.
The single fixed normal and symmetric cruise objective provide no mechanism
for choosing a future left/right passing maneuver. Small off-axis inputs in
the range cases do not prevent their decay-row obstruction either.

### Two counterfactuals separate infeasibility from collision inevitability

At the saved stationary 0.6 s state, the alternate input `[0;-0.95]` satisfies
all current hard rows with maximum residual `-0.049988`. Its successor has
speed 7.197538898 m/s and a feasible next hard program, whereas the actually
chosen cruise successor is infeasible. A suitable finite CLF slack also exists.
This verifies loss of successor feasibility under the selected action; it is
not a proof of indefinite feasibility of the alternate action.

From the actual 0.7 s failure state, a separate finite braking calculation
reaches 0.001 m/s at 1.7 s, after 3.802507860 m of forward travel. All ten holds
retain the original swept separation, model-domain, tire-slip and input rows;
only the decay row is omitted. Maximum retained row residual is `-8.8e-5`,
and the remaining circumdisk margin is 0.184625590 m. The final positive speed
respects the inward numerical reserve on the nonnegative-speed domain; exact
zero speed is not claimed. This calculation is neither an executed fallback
nor a completed passing/cruise experiment.

## Why uncertainty advances or creates failure

The code requires `lower(b_h) >= q*upper(b_0)`. For a shared initial scalar
position error `e` with `|e|<=r`, and unchanged error during a stationary hold,
the actual difference contains `(1-q)*e`. Its worst adverse support is
`(1-q)*r`. Independent endpoint bounds instead charge `(1+q)*r`, an extra
`2*q*r`. They compare different possible initial worlds rather than the same
world propagated through the hold. The bound is safe but unnecessarily strong.

For the straight affine ego map `Phi`, fixed position row `a`, and initial
radius `rho`, the corresponding shared-error support is

\[
|a\Phi-qa|\rho
\quad\text{instead of}\quad
(|a||\Phi|+q|a|)\rho.
\]

The offline counterfactual removes only this redundant initial-error allowance
and the analogous `2*q*|n|^T*r_target_position`. It keeps velocity/acceleration
uncertainty, finite jerk, other tube/chart allowances and swept separation.
The decay-bound improvements are 0.245997486 m (bounded stationary),
0.249114688 m (bounded oncoming), 0.299863781 m (bounded crossing), and
0.413305175 m (NRMM). Each of these four failure-state programs then becomes
feasible without changing estimator gains or shrinking the supplied sets.

For NRMM at 4.1 s, initial barrier bounds are [12.338542050,12.843354131] m.
Its impossible row has gap 0.294116242 m even over the unrestricted actuator
amplitude box. The lost-correlation correction alone exceeds that obstruction.
The observer's uncertainty still consumes real robust margin, and this does
not prove its bounds are optimal. It shows that the present failure does not
require observer divergence and is partly caused by how control uses the bounds.
It also does not establish that this correction alone completes the encounter.

## Why the present geometry cannot certify lateral passing

The barrier represents both 4.8 m by 1.9 m rectangles by enclosing disks,
requiring at least 5.412363800 m of center distance. This is independent of yaw.
For aligned rectangles abreast, the required lateral center separation is
only `0.95+0.95+0.25 = 2.15 m`.

At the instant ego and target exchange longitudinal order, their longitudinal
positions must agree. A target on `y=0` then permits center separation at most
4 m because the ego retains `|d|<=4 m`. The disks cannot satisfy their required
5.412 m separation anywhere in that cross-section. For the range target on
`y=0.8`, the maximum is 4.8 m, also insufficient. This is a geometric/topological
obstruction to continuous passing while that target remains an obligation.
No solver, larger horizon or different CBF rate can remove it while preserving
both this circumdisk approximation and this domain.

Removing physical road boundaries correctly leaves the model domain intact.
Expanding the domain merely to accommodate inflated disks would change model
assumptions and is not the appropriate conclusion. The original oriented
rectangle support can represent lateral separation inside the current domain;
its continuous robust implementation and a feasible maneuver still need design
and validation. Geometric room is not itself a dynamically feasible trajectory.

## Meaning for the CBF framework and next design decision

A function that describes collision-free positions is not automatically a CBF
on that entire state set. A valid controlled-invariance claim also requires
existence of admissible controls on the asserted domain. The present successful
holds satisfy a sufficient sampled barrier condition, but the observed positive
barrier values at infeasible states disprove universal feasibility of that
condition on the admitted geometric set. Continuous-time position separation
also has relative degree two with respect to longitudinal acceleration; exact
sampling introduces input dependence but does not eliminate the limited
order-`h^2` control authority.

This distinction follows the admissible-input/controlled-invariance definition
in Cavorsi et al., [Tractable Compositions of Discrete-Time Control Barrier
Functions](https://arxiv.org/html/2004.01858v1), Section III, and the high-relative-
degree/input-limitation discussion in Xiao and Belta,
[Control Barrier Functions for Systems with High Relative Degree](https://arxiv.org/abs/1903.04706).
Huang et al.'s [Predictive Control Barrier Functions](https://arxiv.org/html/2502.08400v2),
equation (8) and Lemma III.1, obtain the relevant recursive-feasibility argument
using an invariant terminal set and a shifted continuation. The current
one-hold program contains neither, so those conclusions cannot be transferred.

The recommended next design work is to restore input-aware predictive
continuation/viability within the intended Predictive CBF framework, use robust
oriented-rectangle separation to leave passing routes open, and propagate the
shared initial uncertainty through the complete barrier residual. Keep the
soft CLF, hard physical/model constraints and explicit failed-solve behavior.
Do not substitute an arbitrary rate adjustment, obstacle slack, or removal of
model bounds for these missing properties. A longer horizon alone is also
insufficient without a feasible continuation condition and appropriate geometry.
No such online redesign is implemented or claimed validated by this diagnosis.

## Reproduction and validation

Run the committed diagnostic from the repository root:

```matlab
addpath('scripts');
analysis = analyzeSingleHoldInfeasibility( ...
    '/home/zai/.cache/collisionAvoidance/clf-slack-rerun-20260915/final-no-road/campaign/campaign.mat', ...
    '/home/zai/.cache/collisionAvoidance/single-hold-diagnosis-20260915');
```

The campaign is the existing seed-20260914, 0.1 s, no-road experiment reported
in `CLF_SLACK_NO_ROAD_RERUN_20260915.md`. The diagnostic evaluates all saved
failed prefixes and the first 20 holds of the completed exact crossing.
Assertions validate replay equivalence, all seven positive obstruction gaps,
infeasible hard-only LPs, feasible no-decay LPs, the earlier-action comparison,
and every retained hard row of the finite braking calculation. Raw matrices,
models and results are saved as `analysis.mat` and `analysis.json` in the output
directory. The `linprog` checks are an independent solver comparison.

Factory MATLAB Code Analyzer reports zero findings for the new script.
The ordinary analyzer initially reported an unavailable user settings file;
explicit factory settings resolve that administrative warning. Temporary
diagnostic development errors were corrected before the completed run.
The controller/observer algorithms and experiment drivers are unchanged, so
no new full closed-loop campaign, regression-suite count or timing claim is
made. These untimed replay/ablation results are separate from real-time tests.
