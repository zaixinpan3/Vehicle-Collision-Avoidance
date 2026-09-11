# Hard predictive certificate with an invariant continuation

Version 17, September 11, 2026. The controller now solves the strict exact-state,
two-vehicle study described in
[SINGLE_PATH_RECURSIVE_FEASIBILITY.md](SINGLE_PATH_RECURSIVE_FEASIBILITY.md).
The former finite-perception-exit construction is superseded.

```matlab
cfg = collisionAvoidanceControllerConfig();
certificate = [];
[command, inputs, problem, certificate] = ...
    collisionAvoidanceController(ego, target, road, cfg, certificate);
```

Provide one target's full state and a timestamped ego state. At each later
sample also provide the previously issued `heldActuatorInput` and retained
certificate. No perception packet is needed. Both vehicles execute their
declared predictors exactly; the ego predictor includes the explicitly admitted
terminal schedule. Nonzero initial state errors, ego process residuals and
uncertain future target motion are rejected. The target's exact motion law is
specified in [TARGET_PREDICTION_CONTRACT.md](TARGET_PREDICTION_CONTRACT.md).

The finite program keeps collision, road, actuator, slew, model-domain and
terminal constraints hard. It maximizes a normalized feasible margin, then
optimizes the existing CLF performance objective with unrestricted nonnegative
CLF slacks. The verifier evaluates physical rows independently of solver status
and does not use CLF relaxation to repair hard constraints. A solver's negative
status does not establish global physical infeasibility.

Admission requires the final enclosure to lie in the joint invariant terminal
set. This set combines an affine velocity-comparison excursion bound, complete
road/footprint constraints and a support bound on the target's entire future.
The first terminal command must obey entry slew; the invariant velocity box
also guarantees later input and slew limits. Its actual construction and proof
are given in the main requirement document.

Every prefix continuation preserves the admission schedule, physical rows,
absolute target future and already executed inputs. A checked feasible
replacement can improve performance. Otherwise the incumbent remains valid.
After the finite prefix, the current exact state supplies terminal feedback,
and the invariant suffix produces an analytic feasible continuation forever.
The retained QP is in original admission coordinates; `inputPlan` is the
currently executable prefix or terminal preview.

Metadata distinguishes the model theorem and its practical limits:

| Field | Meaning |
| --- | --- |
| `planCertified` | A complete prefix-plus-invariant-tail continuation is available |
| `recursiveFeasibilityClaimed` / `indefiniteRecursiveFeasibilityClaimed` | Conditional exact scheduled-model feasibility at every later frame |
| `terminalContinuationCertified` | Admission includes the invariant terminal rows |
| `terminalActive` | The finite approach has ended and terminal feedback is executing |
| `certifiedDuration` | Infinite, under the declared assumptions |
| `physicalVehicleGuaranteeEstablished` | False: this is not nonlinear physical-vehicle validation |
| `barrierValue` | Negative retained feasible margin, a witness lower-bound diagnostic |

The retained margin is not advertised as the global optimum of a nonlinear
predictive barrier value function. After terminal entry the invariant-set
argument supplies safety even though a nonnegative speed approaches zero and
its numerical margin need not remain strictly positive. CLF convergence,
robust disturbance rejection and solver worst-case runtime are separate claims.
No perception exit, new-target event, weaker controller mode or unproved fresh
admission is used in the recursive-feasibility argument.
