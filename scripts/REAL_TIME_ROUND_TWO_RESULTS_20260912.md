# Real-time round two: measured outcome

September 12, 2026, third campaign of the day. Same seven trials as the
information-state PCBF validation (`INFORMATION_STATE_PCBF_RESULTS_20260912.md`):
three straight-road scenes with exact and with noisy estimates, 300 holds
each, plus the 40-hold forced fresh-failure trial; 8 m/s cruise, 100 ms
holds, seed 20260912, frame deadline armed at 0.1 s. The controller now
transfers the carried witness by box inclusion instead of rebuilding it,
solves the tiers as condensed programs on the physical unknowns, samples
the CLF decrease once per hold, floors the fresh horizon at two stages,
gates further attempts by the frame deadline, and runs the generated
kernels (`controller/REAL_TIME_PLAN.md`, section 7). **The guarantee
mechanism is unchanged: every issued command is a verified plan of the
same program, fresh or carried.**

## Campaign results

Original results: `~/.cache/collisionAvoidance/realtime2-pcbf-20260912`
(the archive holds export copies). Margins have already deducted the
0.25 m clearance.

| Scene | Estimation | Holds | Passed | Carried-witness commands | Max value | Max descent residual | Min separation margin (m) | Min road margin (m) | Final speed (m/s) | Terminal-law frames |
| --- | --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| oncoming | exact | 300 | True | 0 | 0 | 0 | 0.4612 | 0.7545 | 8 | 0 |
| oncoming | noisy | 300 | True | 0 | 0 | 0 | 0.6558 | 0.6454 | 8 | 0 |
| stationary | exact | 300 | True | 287 | 0 | 0 | 6.401e-06 | 3.8 | 2.8451e-20 | 0 |
| stationary | noisy | 300 | True | 249 | 0 | 0 | 0.001291 | 3.778 | -7.7783e-06 | 0 |
| crossing | exact | 300 | True | 0 | 0 | 0 | 9.27 | 3.8 | 8 | 0 |
| crossing | noisy | 300 | True | 0 | 0 | 0 | 9.268 | 3.792 | 8 | 0 |
| crossing | forced fresh failure (noisy) | 40 | True | 40 | 0 | 0 | 9.268 | 3.796 | 0.45193 | 25 |

Frame times, before (round one, commit 4719cecf) and now:

| Scene | Estimation | Median frame before (s) | Median frame now (s) | Max frame before (s) | Max frame now (s) | Median witness check now (s) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| oncoming | exact | 0.251 | 0.0617 | 0.828 | 0.943 | 2.3e-05 |
| oncoming | noisy | 0.219 | 0.0607 | 0.408 | 0.385 | 2.2e-05 |
| stationary | exact | 0.492 | 0.0793 | 1.85 | 0.473 | 3.4e-05 |
| stationary | noisy | 0.555 | 0.0755 | 2.3 | 0.313 | 3.4e-05 |
| crossing | exact | 0.212 | 0.0613 | 0.24 | 0.0692 | 2e-05 |
| crossing | noisy | 0.214 | 0.0601 | 0.236 | 0.0696 | 2e-05 |
| crossing | forced fresh failure (noisy) | 0.111 | 0.0484 | 0.188 | 0.0606 | 3.05e-05 |

## Reading the table

- **Safety exercised as before.** Every issued command is a verified plan
  at value zero with zero descent residual in all seven trials. Carried
  witness commands appear near rest, where the model-domain speed floor of
  the declared affine plant makes the first fresh stage infeasible from a
  decelerating input, and in the forced-failure trial by construction.
- **What changed the frame time.** The witness check went from a full
  rebuild of tubes and rows (about 55 ms at cruise, 2 s near rest with the
  interpreted kernels) to bookkeeping and an inclusion check; the two
  conic solves went from the lifted cell-state program to condensed
  working-set programs on 48 unknowns; the CLF cones went from one per
  control point to one per hold; the fresh horizon near rest went to two
  stages; the generated kernels replaced interpreted tubes and rows; and
  a further seed attempt is started only when it fits before the deadline.
- **Not shown.** Nonlinear plant containment, curved roads, target
  velocity or acceleration uncertainty, estimator-in-the-loop operation,
  and worst-case timing on other hardware; the maxima in the table are the
  observed maxima of these runs on this workstation.

## Regression and static analysis

The full repository suite passes 640 of 640 tests (MATLAB R2026a), with two
new cases: the transferred and rebuilt witness modes issue identical
commands down to the terminal law, and the condensed and lifted program
forms return the same value and plan. Code Analyzer over the changed MATLAB
files reports no findings other than the pre-existing sparse-indexing
suggestions.
