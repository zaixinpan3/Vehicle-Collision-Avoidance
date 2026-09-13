# Real-time plan for the information-state PCBF controller

September 12, 2026. This note records where the time goes in the current
controller (certificate version 19, commit `e83df709`), why, and a staged
redesign that keeps the recursive-feasibility argument of
[`INFORMATION_STATE_PCBF.md`](INFORMATION_STATE_PCBF.md) intact while
bringing a frame under the 100 ms hold. Nothing in this note is
implemented; every number is a measurement of the current code.

## 1. Where the time goes

Measured on the 2026-09-12 campaign (straight road, 8 m/s cruise, 16-stage
horizon, 0.1 s hold, MATLAB R2026a, native Clarabel bridge):

| Phase (median per frame) | Cruise, oncoming exact | Near rest, stationary exact |
| --- | ---: | ---: |
| Prediction (tubes, cells) | 0.052 s | 0.25 s |
| Formulation (rows, lifted program) | 0.18 s | 3.85 s |
| Solve (two seeds: margin LP + CLF SOCP each) | 0.45 s | 5.2 s |
| Carried-witness verification | 0.11 s | 1.96 s |
| Acceptance and commit | 0.006 s | 0.07 s |
| Whole frame | 0.82 s | 11.4 s |

Profiler on twelve cruise frames: native SOCP solver 52 % of wall time
(50 calls, 0.15 s each), formulation 30 % (lifting the cones into the sparse
program 12 %, geometry rows 15 %), prediction 9 %, verification 3 %.

Program structure at a cruise frame (saved frame 99, oncoming):

| Quantity | Cruise (8 m/s) | Slow (2.7 m/s) |
| --- | ---: | ---: |
| Decision variables (inputs + CLF slacks) | 48 | 48 |
| Tube cells (16 stages) | 112 (7 per stage) | 646 (up to 56 per stage) |
| Inequality rows | 23 511 | 134 583 |
| of which model domain / road / tire slip / collision | 10 752 / 7 168 / 3 584 / 1 792 | 62 016 / 41 344 / 20 672 / 10 336 |
| CLF second-order cones (one per tube point) | 896 | 5 168 |
| Lifted conic program | 21 013 rows × 742 columns | 94 513 × 3 946 |
| Inequality rows active at the optimum | 1 | 1 |

Three facts drive everything below. The program has 48 unknowns and tens
of thousands of rows, of which one is active at the optimum. Every row
family is instantiated at every Bernstein control point (eight per cell) of
every cell, and the cell count per stage is `max(2, ceil(2·‖A‖∞·h))`, which
grows without bound as the scheduled speed falls because the lateral tire
terms scale with `1/v`. And the frame runs four interior-point solves
(margin LP and CLF SOCP for each of two seeds), none warm-started, plus a
from-scratch rebuild of the carried witness.

## 2. What the guarantee actually needs

The theorem needs only three things per frame: the conditioned boxes, a
*verification* of the carried witness on them, and, if a fresh plan is to
replace it, a *verification* of that plan with a value no larger than the
carried one. Verification is a sparse matrix-vector product plus the
terminal membership test; it costs 5 ms at cruise today. The solve is not
part of the proof. Therefore:

- **The controller is anytime-safe by construction.** If the fresh solve
  is stopped at a deadline, the verified carried witness is the command,
  exactly as on a solver failure. Real-time *safety* is obtained by adding
  a deadline to the fresh search; real-time *performance* is obtained by
  making the fresh solve fast enough to finish before it. The two are
  separable and should be implemented in that order.
- Anything that makes the fresh solve cheaper is admissible provided the
  candidate it produces is verified in full before acceptance. Conservative
  row aggregation, constraint generation, coarser CLF sampling, shorter
  horizons and warm starts all fall in this class.

## 3. Redesign, in order of expected gain per unit of risk

### 3.1 Deadline and witness (safety first, no proof change)

Add `solver.frameDeadlineSeconds`. The fresh search checks the clock
before each solve and after each verification; when the deadline passes,
the carried witness is committed and the frame reports
`freshSolveFailure = "deadline"`. Frames then never exceed the deadline
by more than one solver call. On its own this changes nothing about speed
but makes every later optimisation safe to deploy incrementally.

### 3.2 Cheap witness verification through Lemma 1 (0.11 s → about 1 ms)

The carried rows are identical functions of the box and monotone under
inclusion (Lemma 1 of the design note). The previous frame verified them on
a box that contains the conditioned box, so the only new work is the
inclusion test, the first-stage rows on the conditioned box, and the
terminal membership. Verification of stages 2..N can be replaced by the
inclusion certificate plus the eps-level arithmetic allowance already
used. Keep the full re-verification as a debug option, not the default.

### 3.3 Skip the margin LP when the witness has value zero (removes 1–2 solves)

The margin LP exists to decide whether a zero-violation plan exists. When
the carried witness verifies with value zero it *is* such a plan, so stage
A is answered and the CLF SOCP can start immediately with violations fixed
at zero and the witness as a feasible point. The value LP is needed only
when the carried value is positive. At cruise this halves the solver calls.

### 3.4 Constraint generation on the fresh solve (0.45 s → tens of ms)

Solve with a working set of a few hundred rows: the stage-endpoint rows,
every row that was active or within a margin at the previous frame, and
the terminal rows. Verify the result against the full row set (5 ms), add
the violated rows, resolve; two rounds suffice in practice for a program
with one active row. The certificate is still the full verification, so
the proof is untouched. With 48 variables and a working set of this size,
a dense active-set QP solver warm-started from the shifted witness is the
natural engine; the conic solver remains only for the CLF cones (see 3.6).

### 3.5 Row aggregation per cell and a sound cell count (rows ÷ 8, cells ÷ 10 near rest)

For an affine row `a·x ≤ b` on a cell whose tube is `x(t) = M(t)d + o(t)`,
`a·M̄ d + a·ō + ‖a‖·r_cell ≤ b` with the cell centre and an enclosure
radius is a sound single row replacing the eight control-point rows; the
full point-wise rows stay in the verification set. Replace the cell rule
`ceil(2·‖A‖∞·h)` by the Taylor-remainder bound the tube kernel already
computes (order 6), and schedule the near-rest stages on the zero-speed
affine model whose `‖A‖` is bounded, which is the model the terminal set
already uses. Expected: about 4 cells per stage at every speed.

### 3.6 Make the CLF tier cheap and seed-independent

The CLF rows are performance, not safety. Impose the sampled-data decrease
at cell endpoints (or once per stage) instead of at every control point,
which removes about 90 % of the cones. Replace the global-curvature
majorant (`curvature = ‖Q‖∞ + 1`, concave part linearised at the seed) by
an exact split `Q = Q⁺ − Q⁻` with the concave part bounded on the box
radius rather than on the distance to the seed; the majorant then no
longer depends on the seed, the equilibrium seed becomes redundant, and
the fresh search returns to one solve. If the cones remain, linearising the
convex part at the warm start turns the whole program into a QP, which
3.4's active-set solver handles in well under a millisecond.

### 3.7 Speed-dependent horizon and terminal-only frames

Near rest a 16-stage horizon covers centimetres while the program is five
times larger. Choose the horizon from the terminal set: the smallest `N`
whose stopping excursion from the current speed fits the chart, with a
floor for lateral recovery. When the conditioned box already satisfies the
terminal membership, the zero-stage plan (terminal law) is a verified plan
in microseconds; command it and re-plan only when the box leaves the set
or a performance objective (cruise recovery) is unmet.

### 3.8 Implementation-level work (formulation 0.18 s → about 20 ms)

Generate the row builders and the lifted-program assembly as native code
like the tube kernel (`bicycleHeldIntervalKernelMex`); assemble sparse
matrices directly instead of dense stage maps; precompute the Taylor
coefficients (the profiler shows 123 566 `factorial` calls in twelve
frames); reuse the previous frame's cell and normal structure for the
witness; keep the solver's workspace allocated across frames.

## 4. Expected budget after the plan (cruise, per frame)

| Phase | Today | Target | Mechanism |
| --- | ---: | ---: | --- |
| Prediction | 52 ms | 15 ms | fewer cells, cached Taylor coefficients |
| Formulation | 180 ms | 20 ms | native row builders, sparse assembly |
| Witness verification | 110 ms | 1 ms | inclusion certificate (3.2) |
| Fresh solve | 450 ms | 10 ms | one warm-started active-set QP on a working set (3.3, 3.4, 3.6) |
| Full verification of the fresh plan | 5 ms | 5 ms | unchanged |
| Whole frame | 820 ms | about 50 ms | |

Near rest the same mechanisms plus 3.5 and 3.7 bring the frame from 11 s
to the same order, because the cell count no longer grows with `1/v` and
the horizon shrinks.

## 5. Order of work and what each step must show

1. 3.1 deadline: every frame ends within the deadline plus one solver call;
   the forced-failure trial's behaviour is reproduced by a deadline.
2. 3.3 and 3.2: identical commands on the exact campaign (same value,
   same plan to solver tolerance), witness check under 5 ms.
3. 3.4 with the active-set QP: identical optimum to the conic solve on the
   saved programs; working-set size and rounds reported per frame.
4. 3.5 and 3.7: sampled margins stay positive; rows and cells per frame
   reported; the noisy campaign passes unchanged.
5. 3.6: single seed, no lock-in on the noisy oncoming trial.
6. 3.8 last, once the algorithmic structure is fixed.

Each step keeps the full verification as the acceptance test, so the
campaign of `scripts/verifyInformationStatePcbf.m` remains the regression
gate; the real-time claim is made only when the median and maximum frame
times of all seven trials are under the hold, on the target hardware,
with the deadline of 3.1 armed.
