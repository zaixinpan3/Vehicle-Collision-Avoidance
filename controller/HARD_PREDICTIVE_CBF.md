# Hard predictive safety with a prediction-only terminal witness

The version-18 controller solves a complete rolling problem at every sample.
It applies only the first optimized input. The stopping terminal set remains
hard; its feedback is a mathematical continuation witness and has no runtime
command-dispatch path. A failed or uncertified solve ends control explicitly.

```matlab
certificate = [];
[command, inputs, problem, certificate] = ...
    collisionAvoidanceController(ego, target, road, cfg, certificate);
```

Supply exact timestamped ego and target states, one persistent target, and the
previously issued `heldActuatorInput`. The old first-hold transition and
original absolute-time target law are validated before rebuilding. Perception
exit and nonlinear-plant guarantees are outside this experiment.

The feasibility phase finds a hard-safe interior. The CLF SOCP optimizes
tracking, squared CLF relaxation and effort relative to the certificate
operating input, using only a small numerical interior. Extra feasible margin
is diagnostic and does not override cruise. The independent verifier accepts
no negative physical margin and cannot repair safety with CLF slack.

| Field | Meaning |
| --- | --- |
| `planCertified` | The current finite prediction and hypothetical tail were checked |
| `terminalContinuationCertified` | The predicted endpoint satisfies the stopping certificate |
| `terminalPolicyRole` | `predictionWitnessOnly` |
| `terminalActive`, `fallbackUsed` | Always false |
| `remainingSteps`, `deadline` | Fresh horizon length and moving prediction endpoint |
| `certifiedDuration` | Infinite hypothetical continuation under that plan's scheduled models |
| `commandCertifiedDuration` | One first hold |
| `recursiveFeasibilityClaimed`, `indefiniteRecursiveFeasibilityClaimed` | False: refreshed models and geometry lack a general shift-inclusion proof |
| `physicalVehicleGuaranteeEstablished` | False |
| `barrierValue` | Negative feasible-margin diagnostic, not the globally optimized Huang PCBF |

The terminal invariant-set derivation, precise shift conditions and unresolved
proof applicability are in
[SINGLE_PATH_RECURSIVE_FEASIBILITY.md](SINGLE_PATH_RECURSIVE_FEASIBILITY.md).
Finite successful experiments do not replace those conditions or a 100 ms
execution-time qualification.
