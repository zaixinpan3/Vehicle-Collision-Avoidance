# Finite approach time and infinite certified continuation

Version 17, September 11, 2026. The configured horizon is an initial search
window for reaching an invariant terminal set. It is not a perception-exit
deadline and it does not limit how long the controller can keep operating.

`hardEncounterBarrier.plan` may extend the finite approach until all held
intervals and the invariant terminal condition are certified. A search timeout
reports an unresolved numerical search. It neither proves mathematical
infeasibility nor authorizes execution of a partial prefix.

After admission, the terminal-entry time stays fixed. The executed prefix is
fixed in the retained optimization; the unexecuted suffix plus the invariant
terminal policy remains a feasible candidate. The controller can improve that
candidate under the same dynamics and constraints. If optimization fails, the
candidate remains available. At terminal entry the sampled analytic policy
continues indefinitely and a control preview is returned at every frame.

`remainingSteps` is the number of finite prefix intervals still unexecuted.
It reaches zero without exhausting the safety certificate. `deadline` is the
terminal-entry timestamp; `certifiedDuration=Inf` includes the invariant
suffix. `encounterComplete` never becomes true because of target distance.
The same target remains observable and constrained at every frame.

The new `freeCompletionTimeTest` checks independent exact-model runs beyond
five original horizons, target passage without controller termination,
extended admission, repeated solver failure and optimization with terminal
continuation. The script is `scripts/runExactStateRecursiveFeasibilityScenario.m`.
The proof and exact scheduled-plant qualification are in
[SINGLE_PATH_RECURSIVE_FEASIBILITY.md](SINGLE_PATH_RECURSIVE_FEASIBILITY.md).
