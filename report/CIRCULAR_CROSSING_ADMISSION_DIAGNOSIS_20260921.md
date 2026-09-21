## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Mode: run, followed by deterministic numerical diagnosis
- Origin Date: 2026-09-21, America/Chicago
- Verification Status: VERIFIED for the reproduced fixture and stated checks
- Version Label: circular_crossing_diagnosis_v1

# Why circular crossing fails first admission

**Feasible plans exist under the current hard constraints. The production
admission search misses them because it fixes the time shape of the whole input
sequence before searching one amplitude.** Its original line reaches a hard
Frenet chart boundary before obtaining a positive robust collision certificate.
The circular position remainder is decisive near that boundary. Altering only
the time shape produces four independently certified plans under the unchanged
original dynamics, physical rows, chart limits and terminal requirements.

This is an offline diagnosis, not a controller repair. Source revision:
`4b5fd3afdcb60b00d54b08658d50132d1e770771`. All pre-existing source and native
dependency hashes match the September 20 simulation. The executable diagnostic
is [analyzeCircularCrossingAdmission.m](../scripts/analyzeCircularCrossingAdmission.m).
The [JSON report](CIRCULAR_CROSSING_ADMISSION_DIAGNOSIS_20260921.json) contains
the numerical results, provenance and technical-artifact hashes.

## Reproduced failure and actual search

The unchanged fixture has radius 100 m, ego speed 8 m/s and a target crossing
at 4 m/s from a 7.5 m outside offset at road station 15 m. Nominal center arrival
is approximately 1.875 s. Both footprints are 4.8 by 1.9 m. Sensing is exact,
updates/holds/nodes are 50 ms, and the declared plant is the held affine bicycle.
No road edges or fixed physical clearance are added. Diagnostic time budgets
are 30 s. The analytic circle has length 340 m, matching the saved scenario.

Fresh preparation extends the nominal 1.6 s horizon to **96 holds / 4.8 s** for
encounter completion. There are **192 actuator variables**, 96 collision
records and one terminal-exit record. The original nominal physically overlaps
at 16 nodes; its conservative support checks fail at 18 stages, 29 through 46.

The actual branch in `solveHardCbfClf.localJointSearch` is:

1. The nominal fails independent admission.
2. `admitSection` constructs one direction and searches
   `U(alpha) = U0 + alpha*v`, with both signs of alpha permitted.
3. Its base interval is nonempty. Every amplitude is removed by collision
   certificates, returning `collisionExcluded` before objective selection.
4. No certified incumbent exists, so execution stops without issuing a command.

Recorded counts are one family, one horizon, **zero conic solver calls**, zero
restoration calls and zero base solves. The failure occurs before an SOCP is
launched and before scalar performance/CLF-slack minimization. Increasing a conic
iteration limit cannot change this branch. No timeout is involved in these
diagnostic runs. The full joint-support performance optimizer becomes available
only after admission or when an inherited certified incumbent already exists.

## The chosen shape and the binding domain

The direction constructor selects the first, middle and last failed stages:
**29, 38 and 46**, corresponding to **1.45, 1.90 and 2.30 s**. At all three it
requires the same Frenet displacement per unit amplitude:

\[
\begin{bmatrix}\Delta s\\\Delta d\end{bmatrix}
=\alpha\begin{bmatrix}0.4165970245\\0.9090912601\end{bmatrix}.
\]

Thus acceleration/braking timing, steering timing and their relative magnitudes
are fixed by a minimum-energy interpolation. A negative amplitude moves the
vehicle backward relative to nominal progress and toward the outside of the
left bend. The construction also returns the final state and last input to the
nominal values. Its interpolation residual is only `2.78e-16`; the basis is
computed accurately, but represents a deliberately restricted shape.

Intersecting this line with the original hard constraints gives:

| Constraint family | Permitted alpha interval |
| --- | --- |
| Actuator rows, including the held-input constraints | [-8.724437, 8.450702] |
| Local pose-chart rows | **[-4.331618063, 4.331610105]** |
| Terminal pose and modal constraints | Do not further reduce the above intersection |

The chart domain is +/-2 m in station, +/-4 m in lateral position and +/-0.5 rad
in heading around each nominal node. These are hard domains supporting the
circular coordinate approximation, **not road-edge boundaries**. At the negative
endpoint, the lateral displacement reaches -3.999901451 m at **1.60 s**. It
cannot grow further along this fixed line. Its target-avoidance bottleneck is
later, at **1.70 s**. This timing mismatch matters: multiplying the original
shape enlarges both the premature lateral peak and the later clearance demand.

## Physical clearance versus robust certification

The circular Frenet-to-Cartesian position is represented by an affine map plus
an outward position-error disk. For curvature `k=0.01`, station radius `rs=2`
and lateral radius `rd=4`, the existing remainder formula at the nominal center
is approximately

\[
\epsilon_p = \frac{k(1+k r_d)r_s^2}{2}+k r_s r_d
           = 0.0208+0.0800 = \mathbf{0.1008\ m}.
\]

It encloses all poses in the three-dimensional chart box, including combinations
the selected scalar line never visits. It is a geometry approximation reserve,
not measurement noise or an added desired clearance. It cannot simply be
deleted while retaining the certificate's claimed scope.

At the original negative amplitude limit:

- All original base/terminal/soft-CLF checks pass when slack is completed.
- The limiting true rectangle node gap is **0.084753148 m at 1.70 s**.
- Actual Cartesian position error of the affine chart at that node is
  **0.075105777 m**, within its 0.100800002 m disk.
- Even with continuously optimized separation direction and optimistic omission
  of the tiny uncertainty generators, the worst reserved certificate gap is
  **-0.003909625 m**. The best angle at the limiting node is -1.506865 rad.
- The terminal-exit certificate has approximately **7.089588 m** surplus in
  that optimistic calculation; exit is not the rejection category.
- An independent SAT audit of the whole 4.8 s candidate finds minimum sampled
  margin **0.058496299 m at 1.6853 s**. It uses 0.1 ms samples in holds 20--60
  and 5 ms elsewhere. This is a sampled physical witness, not an accepted
  production plan or a continuous-time safety proof.

The 20,001-point amplitude scan finds 201 physically clear node samples but no
clear chart-enclosure samples. A stronger check optimizes separation directions
continuously using exact rectangle configuration distance at 2,001 amplitudes.
For each collision record, its optimistic signed gap is Lipschitz in alpha with

\[
L_i\leq\|P_i x_i'\|_2+\|a_{\mathrm{ego}}\|_2|\psi_i'|.
\]

Using the active record at each grid point to cover the surrounding half grid
cell gives a maximum possible inter-sample certificate gap of
**-0.001637586 m**, including a `1e-8 m` arithmetic allowance. The maximum
per-node Lipschitz constant is 1.157775. This numerical covering calculation
rules out a missed narrow admissible amplitude within the same line and base
domain under the stated model; it is not an interval-arithmetic software proof.
Omitted generator support is nonnegative and at most `2.18e-8 m`, so its omission
makes exclusion harder, while every accepted witness retains it in full.

## Controlled diagnostic comparisons

| Offline change | Result | Interpretation |
| --- | --- | --- |
| Normals 32/128/512 crossed with amplitude cells 16/64/256 | All nine reject | Refining the same line's discretization does not recover this fixture |
| Spatial deformation angle 0:15:165 degrees, same three-time plateau | All twelve reject with the default certificate search | These alternative spatial directions alone are insufficient in the tested family |
| Rebuild lateral chart radius 3.5, 4.0 or 4.01 m | Reject | Domain/remainder tradeoff remains unfavorable |
| Rebuild lateral chart radius 4.05, 4.1, 4.2, 4.5 or 5.0 m | Certify | Slight domain expansion can recover the original shape in this fixture, with its remainder recomputed |
| Keep every original hard constraint; change temporal weights at the same three selected stages | Four certified witnesses | The original hard problem is feasible; the fixed time shape causes lost admission capability |

The last comparison changes the constructor's desired position offsets from
`[1,1,1]` multiples of the transverse vector to the weights below. It uses the
same 32 normals, 16 cells, input metric and final state/input correction.

| Relative displacement weights at 1.45 / 1.90 / 2.30 s | Admission | Minimum original node-certificate gap, m | Minimum dense sampled SAT margin, m |
| --- | --- | ---: | ---: |
| [1, 1, 1] | Rejected | Best optimistic value -0.003909625 | 0.058496299 for the rejected endpoint candidate |
| [1, 1, 0.9] | Certified | 0.022853436 | 0.089972596 |
| [1, 1, 0.8] | Certified | 0.026004494 | 0.094045533 |
| [0.8, 1, 0.8] | Certified | 0.064278383 | 0.154946490 |
| [1, 1.2, 1] | Certified | 0.064714081 | 0.150313533 |

The four witness plans were rechecked in a separate verification phase against
the **original** program. Physical matrices/bounds, terminal physical bounds,
layout and all affine prediction maps remain exactly equal. Only the candidate
and its certifying directions differ. `solveHardCbfClf.certify` accepts each.
All four return to the original terminal state within `1.1e-13` in maximum
component error. Independent affine endpoint reconstruction error is at most
`2.14e-14`.

For [0.8,1,0.8], the maximum node lateral offset is **3.836094 m**, maximum
heading deviation from nominal is **0.364988 rad**, and speed spans
**6.700991--9.543923 m/s**. It starts with steering -0.042011 rad and signed
longitudinal ratio -0.074089. It changes encounter timing and moves outward,
then recovers the nominal terminal motion. The small [1,1,0.9] change already
proves feasibility, but reaches heading deviation 0.457000 rad; the optional
0.4 rad modeling diagnostic is not part of the enforced certificate. The
[0.8,1,0.8] witness avoids that particular concern at the nodes. No nonlinear
plant validity claim follows from either plan.

## Consequence for algorithm design

The supported causal chain is: **one fixed temporal deformation -> a premature
hard chart limit -> insufficient robust gap at a later collision node -> empty
scalar certificate set -> no admitted incumbent -> no full joint improvement**.
The chart remainder amplifies the representation restriction. Terminal exit,
soft CLF infeasibility, conic convergence and runtime expiration are not the
observed rejection mechanisms in this fixture.

The next design study should give the deformation construction useful timing
freedom, or derive tighter certified enclosures conditioned on the candidate
family. Simply reducing separation margins or dropping the chart domain would
invalidate the present reasoning. The successful weight choices are offline
existence counterexamples, not a general policy or justification for a preset
trajectory catalog. Enlarging a chart also enlarges its remainder, so the 4.05 m
result is a sensitivity observation, not a universal tuning rule.

Production admission, whole-hold safety and real-time closure remain unchanged.
This task does not install a new controller, execute the counterexamples as
online commands, rerun a 30 s closed loop with them or establish their WCET.

## Reproduction and checks

```bash
matlab -singleCompThread -batch "addpath('scripts'); analyzeCircularCrossingAdmission('/absolute/new/diagnostic-output');"
```

The harness creates a renamed diagnostic class in the external output directory
to expose the exact production deformation and substitute only a diagnostic
direction. It does not shadow or modify the production class. Original and
diagnostic admission information must agree exactly before any ablation.
Paths are restored on return. The full run saves prepared problems, plans,
per-case JSON, verification results and plot data. No old implementation or
generated class is added to the repository.

The integrated driver completes with all numerical assertions passing. Factory
Code Analyzer reports zero issues. A first diagnostic incorrectly assumed that
exact sensing implied empty numerical generator arrays; inspection found the
retained tiny outward sets and the optimistic exclusion argument was corrected.
Actual acceptance never deletes them. A post-run default-Code-Analyzer assertion
also encountered a missing user settings file; factory-configured analysis and
the final complete batch both pass. No full MATLAB unit suite is newly claimed
for this diagnostic-only change. Production sources and dependencies are unchanged.

Raw artifacts and the inspected `diagnosis-figure.png` / `.pdf` are retained at
`/home/zai/.cache/collisionAvoidance/circular-crossing-diagnosis-20260921/`.
Only the reusable diagnostic, this report, its JSON companion and the report
index are committed; raw MAT files, generated figures/classes, native binaries,
solver dependencies and unrelated user materials remain external or excluded.
