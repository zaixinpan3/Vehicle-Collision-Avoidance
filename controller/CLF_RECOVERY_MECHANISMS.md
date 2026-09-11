# Lateral and heading recovery: feasible-set and sampled-flow diagnosis

This report records experiments and diagnostics from before the modified Fiala
tire revision. Its linear-tire and friction-limit results describe that earlier
controller. The current model uses scheduled Fiala tangents and no separate
axle-friction constraints; see [LTV_BICYCLE_MODEL.md](LTV_BICYCLE_MODEL.md).

## Evidence scope

This analysis extends the saved-data diagnosis in `CLF_RELAXATION_ONLY.md`.
It uses the repeated straight and radius-400 m arc avoidance trials admitted
as EV-0062: 10 s duration, 15 m/s cruise, 8 m/s crossing target, 50 m sensing,
50 ms control period, 24 prediction steps plus the rest continuation, seed
2026 twister, estimator disabled, and zero declared acceleration bias.
The four physical trials reproduced EV-0060 exactly. This analysis does not
run a different controller on the physical plant.

The reproducible entry point is:

```matlab
addpath('scripts');
report = analyzeClfRecoveryMechanisms(sourceDirectory, outputDirectory);
```

Each scene is replayed with its saved ego states, observations, road fits,
configuration, and sequential certificate memory. All 400 applied inputs
must match exactly. At 7.2, 8.0, 9.0, 9.7, 9.75, and 9.8 s, the script probes
alternative plans while holding the measured state and original convex
safety domain fixed. Feasible alternatives undergo the production
`solveHardCbfClf.certify` checks, including rectangle clearance and terminal rest.
The counterfactuals are never sent to the plant or propagated as new
closed-loop measurements.

Research status: verified recorded-input replay and fixed-state analysis.
The results establish local mechanisms, not the recovery performance of a
new controller. They are deterministic cases, not independent statistical
samples; no population estimate, confidence interval, or causal percentage
is claimed. The original controller and the user's slack-only objective are
unchanged.

## 1. A decreasing total CLF permits growing lateral error

At the fixed 15 m/s reference, the speed block separates exactly:

\[
V=0.625(v_x-v_{\rm ref})^2+V_{\rm lat},\qquad
V_{\rm lat}=e_{\rm lat}^{\mathsf T}P_{\rm lat}e_{\rm lat},
\]

where the lateral state contains offset, heading, lateral velocity, and yaw
rate errors. At straight-road 7.2 s,

\[
\dot V_{\rm speed}=-8.483690,\quad
\dot V_{\rm lat}=+1.191842,\quad
\dot V=-7.291848\le-6.048887.
\]

The zero-slack total condition is satisfied while the lateral-state energy
grows. This is a directly evaluated derivative tradeoff, not a solver error.
The straight lateral offset later peaks at approximately 0.4994 m at 8.1 s.

However, the next result limits an overly simple interpretation: the solver
cannot necessarily make the lateral derivative negative at those same states.

## 2. Existing lateral states and front-tire limits restrict immediate decay

The linear programs below minimize the lateral derivative. The first keeps
all original hard rows and fixes the total CLF slack to zero. The second
keeps only current-stage axle friction/linear-tire rows and actuator bounds;
it removes the total CLF and all future collision, road, and terminal rows.

| State | Recorded lateral derivative | Minimum with all rows and zero slack | Minimum with current physical constraints only |
|---|---:|---:|---:|
| Straight, 7.2 s | +1.191842 | +1.045226 | +0.941935 |
| Arc, 7.2 s | +0.116468 | +0.074465 | +0.035524 |
| Arc, 9.75 s | +0.044236 | +0.038949 | +0.005465 |

Every minimum in this table remains positive. Existing heading, lateral
velocity and yaw-rate errors cannot be immediately undone within the
declared physical input set. The active current-stage rows at the extrema
are front-axle friction-polygon facets: rows 1/8 or 4/5 of the 20-row stage
block. They are not the linear-tire validity rows 17--20.

For all 12 probed states, dropping only the total CLF while retaining every
hard predictive row achieves the same derivative minimum as the current
physical constraints alone. Thus, for this particular local derivative
optimization, the future obstacle and road rows impose no further reduction
beyond the current physical envelope. This does not establish that the
obstacle was irrelevant to the preceding maneuver.

Adding the separate hard condition
`VlatDot <= -alpha*Vlat` while keeping total slack zero is infeasible at
straight 7.2/8.0 s and arc 7.2/8.0/9.75 s: five of the twelve probes.
The result rules out treating a hard split of the existing continuous CLF as
an automatically feasible fix. A redesigned lateral certificate, appropriately
relaxed conditions, and earlier anticipation require their own validation.

## 3. The optimal set still contains materially different future behavior

All selected original inputs have zero CLF slack, the global minimum of the
nonnegative slack-only objective. Linear programs find the feasible steering
interval, while an additional diagnostic minimizes next-step lateral-state
energy on this same zero-slack set. That diagnostic is a state-space probe,
not a new online input or desired-acceleration cost.

At arc 9.7 s, feasible first steering spans approximately -0.06726 to +0.05177
rad. The applied steering is -0.008904 rad with acceleration +3.9421 m/s^2.
It predicts `Vlat` increasing from 0.033196 to 0.035118. Another fully
certified zero-slack plan uses +0.051767 rad and approximately zero
acceleration, and predicts `Vlat = 0.027632` instead. Its total CLF derivative
also satisfies the original condition. The original objective assigns both
plans the same optimal value.

Therefore nonuniqueness is consequential in this recorded problem. It is
not random noise: repeated runs choose exactly the same inputs. Nor does a
better one-step counterfactual prove better full-horizon plant behavior.

## 4. Instantaneous decay does not guarantee decrease over 50 ms

With the current reference and P frozen at both nodes, exact quadratic
algebra gives

\[
V(e+\Delta e)-V(e)
=2e^{\mathsf T}P\Delta e+\Delta e^{\mathsf T}P\Delta e.
\]

The enforced instantaneous derivative does not bound the second term or the
difference between the exact held flow and its instantaneous tangent.
At straight 9.7 s, the applied acceleration is -7.1142 m/s^2. The recorded
predictor gives a linear increment of -0.016148 and a quadratic increment
of +0.050704, hence a net increase of +0.034556 despite zero slack.
The physical plant's frozen-reference increase is +0.032317.

Across 20 recovery calls, predicted V increases on six straight and nine
arc calls; the corresponding actual-state counts are six and eight. Each
individual difference freezes its current reference and P, including the
final state at 10 s. These are not comparisons between different successive
curvature references. The earlier arc count in `CLF_RELAXATION_ONLY.md`
used native changing-reference diagnostics and 19 pairs; the definitions
and denominators differ.

Taylor, Dorobantu, Yue, Tabuada and Ames (2021), *Sampled-Data Stabilization
with Control Lyapunov Functions via Quadratically Constrained Quadratic
Programs*, Section V, explicitly distinguishes instantaneous CLF constraints
from sampled decrease constraints. Its feedback-linearizability and stable
zero-dynamics assumptions do not themselves certify this vehicle controller.
[Primary source](https://arxiv.org/html/2103.03937v1#S5).

## 5. Tight heading margins expose ordinary prediction error

At arc 9.70 s, the next heading error is predicted as -0.0196533 rad, just
inside the -0.02 rad boundary. The plant reaches -0.0202107 rad: a one-step
prediction discrepancy of -0.0005574 rad changes the acceptance result.
At 9.75 s, even the predictor expects the following state to violate the
limit (-0.0222980 rad); the plant reaches -0.0226566 rad.

The first crossing is sensitive to model error, but the second is already
visible to the model. Across the recovery window, maximum absolute one-step
heading discrepancies are 0.0014871 rad (straight) and 0.0015473 rad (arc).
No percentage of the full trajectory error is attributed to model mismatch.

The requirement concerns every sample from 9 through 10 s, not the final
sample alone. Both final poses are within tolerance. The controller contains
neither those explicit recovery-window bounds nor a 9 s recovery deadline.
For the current P, a sufficient whole-ellipsoid condition for both pose
tolerances would be `V <= 0.0252611`; the separate offset condition is
`V <= 0.0640125`, using `e_i^2 <= V*(P^-1)_ii`. These are sufficient, not
necessary, conditions and are not imposed by the implementation.

## 6. A small curved-reference consistency defect

At 15 m/s and curvature 1/400 m^-1, the reference has
`vyRef = 0.00447679 m/s`, `rRef = 0.0375 rad/s`, and zero heading error.
Even at the reference scheduling speed, the model then gives
`dDot = vBar*ePsi + vyRef = 0.00447679 m/s`. Its lateral, yaw and longitudinal
acceleration residuals are otherwise roundoff-sized. Thus the declared
reference is not exactly stationary in all path-error coordinates.

A heading reference near `-vyRef/vRef = -0.00029845 rad` would cancel this
kinematic residual in the declared linear model. This is much smaller than
the 0.02 rad acceptance tolerance; it should be corrected consistently, but
this analysis does not identify it as the principal recovery failure.

## Design implications and validation limits

The evidence supports studying separate longitudinal and lateral sampled
decrease conditions with their own relaxation variables, retaining hard
safety and avoiding any desired-acceleration objective. A simple unrelaxed
split is contradicted by the feasibility probes. Exact quadratic sampled
conditions generally produce convex quadratic/conic constraints rather
than a pure affine-constrained QP; retaining a standard QP would require a
justified sufficient approximation. Neither candidate has been validated
in an altered-input closed-loop plant trial here.

Removing a strictly convex selection objective also removes its uniqueness
argument. Ames, Xu, Grizzle and Tabuada (2017), *Control Barrier Function
Based Quadratic Programs for Safety Critical Systems*, Section IV-B,
equations (33)--(34) and Theorem 3, assumes positive-definite H for its
stated QP regularity result. The present zero-Hessian LP does not satisfy
that hypothesis. This does not require restoring an input target; it means
that input selection and regularity need a separate justification.
[Primary source](https://arxiv.org/pdf/1609.06408).

Validation: final replay reproduced 400 inputs exactly; 12 measured states
were probed; all 67 feasible full-plan alternatives passed production
certification and numerical residual checks, and five independent hard
lateral-decay probes were infeasible. Twelve additional two-variable
current-physical LPs were solved. Exact increment and derivative partition
identities were checked numerically. Factory MATLAB Code Analyzer reported
zero issues. No new physical plant run or full repository test suite is
claimed. Results are retained in `/tmp/controller-clf-mechanisms-20260906`.
