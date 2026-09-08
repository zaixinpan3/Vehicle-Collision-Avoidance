# Finite-sensing predictive CBF–CLF controller

Version 10 implements the policy in [FINITE_SENSING_CONTROLLER.md](FINITE_SENSING_CONTROLLER.md).
The default certifies the next executed interval and uses a nominal finite
lookahead for anticipation. This is a conditional sampled execution certificate,
not a recursive-feasibility theorem. The stronger nonreturn-contract construction
in [ENCOUNTER_SCOPED_CBF_CLF.md](ENCOUNTER_SCOPED_CBF_CLF.md) remains an optional mode.

```matlab
certificate = [];
[command, inputs, problem, certificate] = ...
    collisionAvoidanceController(ego, targets, road, cfg, certificate);
```

The four-input interface stores the same certificate persistently. The fourth
output can instead be supplied explicitly at the next call. Earlier certificate
versions are rejected. Input order is steering angle [rad], then signed braking
ratio. Every issued command must come from a current solve and independent
acceptance. An unsuccessful call raises `noCertifiedContinuation`; drivers stop
before advancing the plant. No stored control or alternate controller is executed.

## Inputs and encounter lifecycle

The observer publishes point estimates and current componentwise error bounds.
It does not supply future-state truth. The ego enclosure is intersected with
the previous executed tube and represented about the intersection midpoint;
the observer point state remains unchanged. Timestamps, applied input,
model, reference and CLF configuration must remain consistent. An updated road
measurement is admitted into the new problem instead of compared by exact array
identity with the previous fit. Coverage still limits the complete footprint.

Targets carry stable IDs and either a local finite motion descriptor or the
strong exit contract documented in [TARGET_PREDICTION_CONTRACT.md](TARGET_PREDICTION_CONTRACT.md).
A complete current perception declaration can end a finite encounter when a
missing target can have left sensor range. Contradictory absence is rejected.
Without that declaration, missing observations retain the obligation. A later
detection is admitted again. No prescribed target road corridor is required.

## Prediction and constraints

The scheduled Frenet bicycle includes active tire parameters, signed drive and
brake forces, aerodynamic drag and rolling resistance. Its correspondence with
the nonlinear plant depends on declared continuous residual bounds. Finite
experimental residual measurements alone cannot prove that premise globally.

For stages through `certifiedSteps`, Taylor/Bernstein cells enclose the held-input
trajectory with estimator uncertainty, numerical remainder and model residual.
Later stages constrain swept nominal endpoint chords and undergo a nonlinear
rollout check, including actual Fiala forces rather than old force tangents. Setting `certifiedSteps=Inf` applies
the uncertain swept construction throughout the finite horizon. Analytic line
and circle charts avoid artificial reference jumps from sampled vertices.

Each cell uses one separating normal per target. Heading-dependent rectangle
support gives linear hard rows; uncertain heading remains tightened in the
executed interval. The nominal force polygon limits simultaneous braking and
cornering. Optional actuator slew constraints, model domain and road/collision
constraints are hard. A failed inner approximation can trigger bounded steering
anchor backtracking and a new solve of the same constraints. Failure of these
candidates does not prove global infeasibility of the nonconvex problem.

## Objective and acceptance

An optional first phase allocates extra future state, tire-slip and clearance
reserves. The configured fraction cap is 0.25, and a reduced allocation retains
99% of its LP optimum to provide an interior target for nonlinear refinement.
These are planning settings; physical safety constraints and complete current
uncertainty enclosures remain hard even at zero optional fraction.
The propagated initial estimation uncertainty also stays mandatory in future
state-domain and tire-slip rows; only the additional process reserve is allocated.
The joint objective contains state errors, input deviation from the fixed
CLF/LQR certificate operating input at the current road curvature, input changes, and squared nonnegative
CLF slack. Both transcriptions use `qp.clf.certificate.operatingInput`, not
the stage-dependent prediction seed. See the
[objective definition](QUADRATIC_CLF_INPUT_OBJECTIVE.md). It has no
desired-acceleration objective. The positive-definite Riccati metric is
synthesized at the nonlinear curved-cruise trim, including declared bias;
the state reference uses the same trim at the configured reference speed. Convex majorants
of the disturbed CLF derivative constrain each certified interval. Positive
slack and persistent errors imply no claim of exact asymptotic convergence.

`avoidanceStageQp` provides sparse cell-state and condensed SOCP transcriptions.
Their hard rows, cone residuals and objective differences are compared in tests.
The solver's auxiliary states cannot bypass `certifyAvoidancePlan`, which checks
the physical input sequence against the condensed original inequalities and CLF
majorants. A solver status alone never authorizes a command.

The previous maneuver is preferred when newly feasible. Its shifted sequence is
only an optimization seed. Strong exit contracts additionally retain their
finite deadline and algebraically truncated witness. Diagnostics identify
`safetyScope`, `certifiedDuration`, `lookaheadDuration`, `geometryRebuildCount`,
solver calls, acceptance and measured wall time. `fallbackUsed` is always false.
The repository's measured runtimes are not worst-case execution-time guarantees.
