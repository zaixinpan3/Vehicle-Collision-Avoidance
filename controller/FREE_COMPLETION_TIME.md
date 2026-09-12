# Moving prediction horizon without terminal handoff

Version 19, September 12, 2026. Each call starts a complete fresh prediction
at the current sample and, at continuation frames, first verifies the
previous plan shifted by one stage with its carried data. The configured
horizon seeds the fresh search, which may extend until all held intervals
and the terminal rows are verified. A search timeout means no fresh plan was
verified within the budget; at a continuation frame the carried witness is
then executed, at admission the frame reports `certificateSearchLimit`.

`remainingSteps` is the number of optimized stages in the accepted plan. It
equals the fresh prediction length after an accepted fresh solve and
decreases by one whenever the carried witness is executed instead. When it
reaches zero the carried witness is the analytic terminal law on the
predicted nominal, and `terminalActive` is true. `deadline` is the accepted
prediction's endpoint, not a wall-clock requirement.

`freeCompletionTimeTest` checks cruise beyond the first prediction horizon, a
moving endpoint, no terminal takeover while fresh solves succeed, and the
carried-witness execution down to the terminal law under repeated fresh
failures. `verifyFreeCompletionTime` saves these independent scenarios. The
straight-road driver saves partial results on failure and logs all frame
times. The same target remains observed and constrained after passage.

The recursive-feasibility proof and its premises are in
[INFORMATION_STATE_PCBF.md](INFORMATION_STATE_PCBF.md).
