# Hard predictive safety with a carried recursive witness

The version-19 controller solves a complete rolling problem at every sample
and applies only the first input of the plan it accepts. Before any fresh
optimization, it verifies the previous plan shifted by one stage, with that
plan's own carried prediction data, on the conditioned ego and target
information sets. The theorem that makes this a recursive-feasibility
guarantee, its premises and its proof are in
[INFORMATION_STATE_PCBF.md](INFORMATION_STATE_PCBF.md).

```matlab
certificate = [];
[command, inputs, problem, certificate] = ...
    collisionAvoidanceController(ego, target, road, cfg, certificate);
```

Supply timestamped ego and target estimates with their error boxes
(`controllerStateErrorBound` or estimator certificates; target position,
velocity, acceleration, yaw and yaw-rate bounds), one persistent target, and
the previously issued `heldActuatorInput`. The executed first hold, the
exact target law and the box intersections are validated before anything is
rebuilt. Nonlinear-plant and perception guarantees are outside this study.

The value LP minimizes the accumulated per-stage violation of the collision
and road rows with every other row hard; the CLF SOCP optimizes tracking,
squared CLF relaxation and effort within that value. The independent verifier
recomputes the violation from the physical rows and accepts no violated hard
row. A fresh plan replaces the carried witness only when it is verified and
its value does not exceed the witness's value.

| Field | Meaning |
| --- | --- |
| `planCertified` | The accepted plan passed independent verification (hard rows, CLF cones, terminal rows) |
| `pcbfValue` | Verified accumulated safety violation of the accepted plan; the reported PCBF value |
| `stageViolation` | Its per-stage terms; `stageViolation(1)` is the executed-stage violation |
| `certificateSource` | `checkedOptimization` (fresh plan accepted) or `carriedWitness` (shifted previous plan executed) |
| `candidateVerified`, `candidateValue` | The carried witness was verified at this frame, and its value |
| `pcbfDescentResidual` | `V(k+1) − (V(k) − stageViolation(1)(k))`; nonpositive by the theorem when `V(k)=0` |
| `lexicographicTieResidual` | Verified value minus the value-stage optimum of the accepted fresh plan |
| `freshSolveFailure` | Why the fresh plan was not accepted, if it was not |
| `terminalActive` | The terminal law on the predicted nominal is the issued command (no optimized stage remained) |
| `terminalPolicyRole` | `carriedWitnessTail` |
| `fallbackUsed` | Always false: the carried witness is the program's own feasible solution |
| `initialErrorBound`, `targetErrorBound` | The conditioned ego (Frenet) and target boxes used by this frame |
| `recursiveFeasibilityClaimed`, `indefiniteRecursiveFeasibilityClaimed` | True under the declared premises (`recursiveFeasibilityScope`) |
| `physicalVehicleGuaranteeEstablished` | False |

Errors that end control are premise failures, not controller decisions:
`inconsistentObservation` (a measurement box misses the predicted box),
`carriedWitnessRejected` (the carried witness failed verification, which the
theorem excludes under its premises), `unboundedTargetSupport` (no terminal
halfspace separates the ego from the target's entire future),
`executionContractViolation`, `changedExecutionContract`, and, at admission
only, `noCertifiedContinuation` or `certificateSearchLimit`.
