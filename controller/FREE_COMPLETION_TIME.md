# Moving prediction horizon without terminal handoff

Version 18, September 11, 2026. Each call starts a complete prediction at the
current sample. The configured horizon seeds a search that may extend until
all held intervals and the invariant terminal condition are certified.
A search timeout means no plan was certified within the budget; it neither
proves global infeasibility nor authorizes a partial prefix.

`remainingSteps` is the length of the new plan. `deadline` is its predicted
endpoint, not a wall-clock requirement to enter the stopping set. No
already-executed decision variables are retained in the next optimization.
The analytic terminal policy remains only in the mathematical witness.
A failed performance solve raises an error even when a former witness exists.

`freeCompletionTimeTest` checks cruise beyond the first prediction horizon,
a moving endpoint, no terminal takeover and explicit failure termination.
`verifyFreeCompletionTime` saves these two independent scenarios. The complete
straight-road driver saves partial results on failure and logs all frame times.
The same target remains observed and constrained after passage.

A fresh horizon alone does not prove recursive feasibility when models and
convex geometry are rebuilt. The current claim flags are false; terminal
invariance and the missing shift-inclusion premises are explained in
[SINGLE_PATH_RECURSIVE_FEASIBILITY.md](SINGLE_PATH_RECURSIVE_FEASIBILITY.md).
