# Information-state PCBF validation

September 12, 2026. Certificate version 19. This is a controller-only,
straight-road experiment on the declared scheduled affine bicycle. The ego
executes the accepted plan's first-stage affine generator exactly from the
true state; ego and target measurements carry bounded uniform noise inside the
declared error boxes. The target follows the exact constant-acceleration,
constant-yaw-rate law it was admitted with. Under those premises the design
in [`controller/INFORMATION_STATE_PCBF.md`](../controller/INFORMATION_STATE_PCBF.md)
claims recursive feasibility: whenever one frame admits a certified plan,
every later frame carries a verified feasible witness. **No nonlinear-vehicle,
estimator-in-the-loop, curved-road or real-time claim is made.**

## What changed and why

- The controller now reasons about information-state boxes (an interval
  centre and radius for the ego and for the target) instead of a point
  estimate. Each frame conditions the carried box on the new measurement box
  (predicted successor box intersected with the measurement box, then re-boxed),
  so the carried set can only shrink relative to what the previous certificate
  covered. Lemma 1 of the design note is what makes the previous witness remain
  valid.
- The carried witness (the previous plan shifted by one stage, closed by the
  terminal stopping law) is verified first at every frame, on the conditioned
  boxes, with the same row builders the fresh program uses. A fresh solve is
  accepted only if it verifies and its accumulated violation value does not
  exceed the carried value (plus the configured lexicographic tie tolerance
  when the carried value is positive). If the fresh solve fails or is
  rejected, the verified carried witness is executed. This replaces the
  earlier fail-fast policy; the executed command is always a verified feasible
  solution of the same program, so the descent inequality of the PCBF value
  holds by construction.
- The terminal set is a robust invariant box family for the zero-speed
  scheduled model under sampled braking feedback with open-loop error
  dynamics. Invariance (Proposition 1) and monotonicity under box inclusion
  (Proposition 2) are verified by comparison matrices and directional budgets;
  membership rows are hard in every program.
- The prediction error chain is the interval hull of the exact stage maps
  applied to the initial box radius. A zonotope chain was tried and rejected
  because re-boxing after conditioning is not monotone for zonotopes.
- The lexicographic solve is margin LP with violations fixed at zero (feasible
  means value zero), else value LP (minimum accumulated violation), then the
  CLF SOCP within that budget. The verified accumulated violation
  `check.value` is the reported PCBF value.
- Hard-row reserves are scaled per row (dominant magnitude times the solver's
  relative tolerance, with a floor), so the native solver's relative
  feasibility tolerance can no longer produce sub-tolerance physical
  violations on rows with large coefficients.
- The sampled-data cross-check in the cruise recovery test now uses YALMIP
  with SeDuMi from `solver/` instead of an SQP solve that exhausted memory on
  the reduced program.

## Trial definition

Each trial starts at 8 m/s with a 100 ms hold and a 16-step horizon on a
straight road with boundaries at y = +/-5 m and 0.25 m footprint clearance.
Target initial position/velocity are (60,0)/(-8,0) oncoming, (15,0)/(0,0)
stationary and (15,-4)/(0,32) crossing, in metres and m/s. Exact trials use
zero error boxes. Noisy trials draw uniform measurement noise with seed
20260912 inside the ego box [0.05 m, 0.05 m, 0.005 rad, 0.05 m/s, 0.02 m/s,
0.005 rad/s] and the target box [0.1 m, 0.1 m, 0, 0, 0, 0, 0.01 rad, 0]:
target position and yaw are uncertain, target velocity and acceleration are
exact because a velocity box makes the all-future target support, and with it
the robust terminal set, unbounded. The forced-failure trial replaces the joint
solver by one that always fails after admission, so every frame after the
first executes the carried witness until the terminal law is the command.

Each executed hold flows the true state with `expm` under the accepted plan's
first-stage generator, and eleven samples per hold audit rectangle separation
and road containment on the true state. The descent residual is
`value(k+1) - value(k)` of the verified accumulated violation; the pass rule
requires every continuation frame to verify its carried witness, every issued
command to be certified, and no carried-witness command unless failure is
injected.

## Campaign results

Output directory `~/.cache/collisionAvoidance/information-state-pcbf-20260912` (original results; the archive holds export copies). Margins have already deducted the 0.25 m clearance.

| Scene | Estimation | Holds | Passed | Carried-witness commands | Max value | Max descent residual | Min separation margin (m) | Min road margin (m) | Final speed (m/s) | Final lateral error (m) | Median frame (s) | Max frame (s) | Median witness check (s) | Terminal-law frames |
| --- | --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| oncoming | exact | 300 | True | 0 | 0 | 0 | 0.01481 | 1.464 | 8 | 1.09e-15 | 0.819 | 1.27 | 0.113 | 0 |
| oncoming | noisy | 300 | True | 0 | 0 | 0 | 0.01563 | 1.545 | 8.00001 | -0.00282 | 0.786 | 1.56 | 0.109 | 0 |
| stationary | exact | 300 | True | 0 | 0 | 0 | 0.002405 | 3.8 | 4.06161e-05 | 2.14e-12 | 11.4 | 15.8 | 1.96 | 0 |
| stationary | noisy | 300 | True | 0 | 0 | 0 | 0.004017 | 3.796 | 0.000141239 | -0.0025 | 11.2 | 16.5 | 1.94 | 0 |
| crossing | exact | 300 | True | 0 | 0 | 0 | 9.27 | 3.8 | 8 | -3.05e-08 | 0.793 | 0.888 | 0.111 | 0 |
| crossing | noisy | 300 | True | 0 | 0 | 0 | 9.269 | 3.791 | 8.00001 | -0.00163 | 0.796 | 0.857 | 0.113 | 0 |
| crossing | noisyForcedFreshFailure | 40 | True | 40 | 0 | 0 | 9.268 | 3.796 | 0.453614 | -0.00216 | 0.301 | 2.22 | 0.0427 | 25 |

## Reading the table

- **Safety claim exercised.** In every trial each issued command is a
  verified plan of the same program (fresh or carried), the verified
  accumulated violation is zero at every frame, and the descent residual is
  zero, so the value function never rises. No frame reported
  `carriedWitnessRejected` or `inconsistentObservation`. The forced-failure
  trial executed the carried witness at all 40 continuation frames and
  reached the terminal braking law (25 terminal-law frames), which is the
  behaviour the theorem describes when the fresh solve never verifies.
- **Sampled geometry.** The sampled separation and road margins on the true
  state are audits, not the proof; both remain positive in every trial. The
  oncoming pass is close (about 0.015 m beyond the required 0.25 m clearance
  at the nearest sample) because the equilibrium-seeded plan with the
  smaller objective passes tighter than the shifted-seeded one did.
- **Two defects found and fixed by the campaign.** First, the noisy
  oncoming trial initially settled at 2.7 m/s with every frame certified at
  value zero: the CLF convex majorant is exact only at its seed plan, so a
  braking tail inherited through the shifted seed reproduced itself. The
  fresh search now solves the shifted and equilibrium seeds and keeps the
  verified plan with the smaller (value, objective) pair; frame times roughly
  doubled. Second, the noisy stationary trial ended at hold 41 with an input
  rejection once the ego had stopped and the measured speed centre fell
  below zero by less than the 0.05 m/s measurement radius. The reader now
  admits any speed box meeting the domain, the continuation conditioning
  intersects the box with the domain, and only admission requires an
  in-domain centre. Both fixes are documented in the design note and covered
  by regression tests.
- **Runtime.** No trial meets the 100 ms hold. Cruise-speed frames take
  about 0.8 s (two seeds solved and verified per frame); the stationary
  trials take 9 to 16 s per frame near rest, where the low scheduled speed
  produces many tube cells. Real-time operation is not claimed.
- **Not shown.** Nonlinear plant containment, curved roads, target velocity
  or acceleration uncertainty (which empties the robust terminal set, see
  the design note), and estimator-in-the-loop operation.

## Regression and static analysis

The full repository suite passes 633 of 633 tests (MATLAB R2026a, `runtests("tests")`, results saved beside the campaign outputs);
the two SeDuMi cross-check cases now run through YALMIP with char-valued
option names. Code Analyzer over the 19 changed MATLAB files reports no
findings other than the sparse-indexing suggestions in
`avoidanceStageQp.m` (18, up from 16 at baseline, from the added violation
columns).
