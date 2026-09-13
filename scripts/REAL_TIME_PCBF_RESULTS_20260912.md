# Real-time round one: measured outcome

September 12, 2026, second campaign of the day. Same seven trials as the
information-state PCBF validation
([`INFORMATION_STATE_PCBF_RESULTS_20260912.md`](INFORMATION_STATE_PCBF_RESULTS_20260912.md)):
three straight-road scenes with exact and with noisy estimates, 300 holds
each, plus the 40-hold forced fresh-failure trial; 8 m/s cruise, 100 ms
holds, seed 20260912. The controller now runs with the frame deadline armed
at 0.1 s, row generation, endpoint CLF sampling, the speed-scaled fresh
horizon (minimum four stages), the triplet-assembled lifted program and the
seed-order rule described in [`../controller/REAL_TIME_PLAN.md`](../controller/REAL_TIME_PLAN.md),
section 6. **Frames are shorter by a factor of three to thirty but still
exceed the 100 ms hold; no real-time claim is made.**

## What changed in the loop

With the deadline armed, a frame runs the witness check, then one fresh
attempt; a second seed runs only if the deadline has not passed. When the
first attempt fails, the verified carried witness is the command, so a
failed attempt now costs one solve instead of the 30 s search budget. A
carried-witness command is a verified plan of the same program and no
longer fails a trial; the count is reported.

## Campaign results

Original results: `~/.cache/collisionAvoidance/realtime-pcbf-20260912`
(the archive holds export copies). Margins have already deducted the
0.25 m clearance.

| Scene | Estimation | Holds | Passed | Carried-witness commands | Max value | Max descent residual | Min separation margin (m) | Min road margin (m) | Final speed (m/s) | Terminal-law frames |
| --- | --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| oncoming | exact | 300 | True | 0 | 0 | 0 | 0.3809 | 0.8174 | 8 | 0 |
| oncoming | noisy | 300 | True | 0 | 0 | 0 | 0.5832 | 0.7081 | 8 | 0 |
| stationary | exact | 300 | True | 144 | 0 | 0 | 9.097e-06 | 3.8 | 2.7116e-12 | 0 |
| stationary | noisy | 300 | True | 63 | 0 | 0 | 0.001568 | 3.792 | 8.2781e-06 | 0 |
| crossing | exact | 300 | True | 0 | 0 | 0 | 9.27 | 3.8 | 8 | 0 |
| crossing | noisy | 300 | True | 0 | 0 | 0 | 9.269 | 3.791 | 8 | 0 |
| crossing | forced fresh failure (noisy) | 40 | True | 40 | 0 | 0 | 9.268 | 3.796 | 0.45361 | 25 |

Frame times, before (campaign of the morning, commit e83df709) and now:

| Scene | Estimation | Median frame before (s) | Median frame now (s) | Max frame before (s) | Max frame now (s) | Median witness check now (s) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| oncoming | exact | 0.819 | 0.251 | 1.27 | 0.828 | 0.0542 |
| oncoming | noisy | 0.786 | 0.219 | 1.56 | 0.408 | 0.0547 |
| stationary | exact | 11.4 | 0.492 | 15.8 | 1.85 | 0.0659 |
| stationary | noisy | 11.2 | 0.555 | 16.5 | 2.3 | 0.0675 |
| crossing | exact | 0.793 | 0.212 | 0.888 | 0.24 | 0.0536 |
| crossing | noisy | 0.796 | 0.214 | 0.857 | 0.236 | 0.0543 |
| crossing | forced fresh failure (noisy) | 0.301 | 0.111 | 2.22 | 0.188 | 0.0232 |

## Reading the table

- **Safety exercised as before.** Every issued command is a verified plan
  at value zero with zero descent residual, in all seven trials. Where the
  fresh attempt failed or the deadline cut the search, the carried witness
  was the command; the stationary trials use it near rest, where the
  model-domain floor on speed makes the first stage of a fresh plan
  infeasible from a decelerating previous input.
- **Rest behaviour.** The exact stationary trial creeps to within
  9 µm of the required clearance and stops (final speed 3e-12 m/s), with
  144 of 300 commands from the carried witness; the noisy one stops
  1.6 mm short with 63. The margin is nonnegative, which is the
  requirement, but there is no reserve beyond the clearance at rest. In
  the noisy trial the true speed of the declared affine plant crosses to
  -2e-5 m/s under road load, an artefact of that plant near rest, which
  the domain conditioning of the speed box absorbs.
- **Two defects fixed by this round.** The terminal row "nominal speed
  minus velocity radius ≥ 0" is not invariant under the braking law and
  cost the witness its terminal membership one hold after a plan rested on
  it; the row now bounds the nominal only, as the design note's (T2)
  states. And the deadline had blocked the anti-lock-in equilibrium seed;
  the accepted plan now records a decelerating tail and the next frame
  solves the equilibrium seed first.
- **Where the cruise frame goes now.** Roughly one third solve (two
  Clarabel calls on a working set), one third tubes and rows, one third
  the witness rebuild. The next steps, in the plan's order, are the
  inclusion-based witness transfer, a cell rule derived from the kernel's
  own Taylor remainder, and native row builders.
- **Not shown.** Nonlinear plant containment, curved roads, target
  velocity or acceleration uncertainty, estimator-in-the-loop operation,
  and any real-time qualification.

## Regression and static analysis

The full repository suite result and Code Analyzer outcome are recorded in
the archive record for this operation.
