# Stationary and oncoming initialization failures

Date: September 16, 2026. Online source revision: `f03d5dcd96b2d0d4bb717375d5266a28ce6c71a6`.

The current controller cannot construct its first collision geometry from the straight cruise guess in these two scenes. Ordinary distance is zero throughout overlapping configurations, so its optimized dual supplies no separating direction. The controller exits before formulating or solving the hard trajectory problem. **Both scenes nevertheless have feasible input plans under the current model and constraints:** independently rebuilding the current program from recorded numerical inputs passes its hard certificate, and subsequent convex optimization also succeeds.

This investigation adds an offline diagnostic driver. It changes no online algorithm, certificate, observer, constraint, or initialization policy. It does not restore the removed finite-branch implementation.

## Failure location and scene evidence

Fresh admission uses `hardEncounterBarrier.predict` to repeat the cruise equilibrium input across the prediction. Here steering is zero and normalized longitudinal input is `0.013645140013191526`, compensating resistance at 8 m/s. This is an optimization anchor, not a requested avoidance trajectory.

`formulateAvoidanceProblem` then requests every midpoint direction from `avoidanceSafetyGeometry.distanceDualNormals`. One unavailable direction stops admission. This occurs before `avoidanceSafetyGeometry.build`, `completionRows`, the soft-CLF formulation, and the hard trajectory solve. Neither the terminal constraints nor the CLF nor the estimator causes this particular early failure.

The two 4.8 m by 1.9 m vehicles are initially separated. With a 100 ms hold, the measured cruise-anchor geometry is:

| Quantity | Stationary | Oncoming |
| --- | ---: | ---: |
| First target admission, simulation time | 0 s | 2.6 s |
| Initial longitudinal center separation | 15 m | 18.4000007824 m |
| Initial body gap | 10.2 m | 13.6000007824 m |
| Closing speed | 8 m/s | 15.9999996759 m/s |
| Prediction horizon | 4.8 s | 3.2 s |
| Cruise-anchor overlap interval after admission | (1.275, 2.475) s | (0.8500000661, 1.4500000783) s |
| Overlapping anchor midpoints | 12 | 6 |
| First/last overlapping midpoint after admission | 1.35 / 2.45 s | 0.95 / 1.45 s |
| Minimum signed anchor distance | -1.9 m | -1.9 m |

The interval follows directly from `abs(relativePosition - closingSpeed*t) < 4.8`. These are collisions of the *unoptimized prediction*, not collisions already occurring in the simulation. The oncoming scene completes 26 target-free holds before target admission; its failure is not at the start of the whole simulation. Midpoints select directions only; the existing safety rows continue to certify the complete held intervals.

## Why the distance dual loses the direction

For the full rectangle configuration polygon, write the collision region as `A*p <= b`. The ordinary-distance dual used for the anchor is

\[
\max_{\lambda\ge0,\ \|A^\top\lambda\|_2\le1}
       (A p_{\mathrm{anchor}}-b)^\top\lambda.
\]

At a strictly interior point, every component of `A*pAnchor-b` is negative. Every nonzero nonnegative multiplier therefore gives a negative objective, whereas `lambda=0` is feasible and gives zero. The unique optimum is zero, and `A'*lambda=0` cannot define a direction. At an exact boundary point, zero multipliers need not be the unique optimum; the strict-interior argument is sufficient for these experiments.

The production overlap guard exposes this mathematical degeneracy. Removing the guard does not repair it. The offline audit solves the same native SOCP at all 80 midpoints, including points that production skips. At points with signed distance below -0.1 m:

| Raw dual quantity | Stationary | Oncoming |
| --- | ---: | ---: |
| Maximum norm of `A'*lambda` | 8.39027e-9 | 1.25352e-9 |
| Maximum absolute objective | 4.16388e-8 | 3.48879e-8 |
| Largest face residual | -0.2 m | -1.59999891 m |

These norms are below the production direction threshold of 1e-6. All raw solves return an accepted native status. Small nonzero residuals reflect numerical tolerances. One nearly touching oncoming midpoint has signed distance approximately -1.25e-6 m and a numerically larger dual norm; the other five deeply overlapping points still invalidate initialization. Normalizing numerical noise is not a justified geometric solution.

Thus the dependency is circular: steering and acceleration optimization need collision directions, but the only direction generator requires the initial predicted rectangles to be separated. A straight cruise guess in an actual avoidance encounter commonly violates that requirement.

## Current feasible plans exist

Two previously saved numerical control sequences were used as offline witnesses. The driver reads the input sequence and initial scene data, then constructs a **new current-revision program**: current distance-dual normals, swept collision rows, actuator bounds, finite exit rows, terminal cones, and soft CLF. It imports no stored certificate or removed algorithm. The CLF slack is recomputed for the witness. The current hard verifier accepts both input sequences, and the current convex solver produces another accepted solution in each case.

| Reconstructed current-program check | Stationary | Oncoming |
| --- | ---: | ---: |
| Recorded inputs pass hard certificate | Yes | Yes |
| Fresh convex solve feasible | Yes | Yes |
| Fresh solution passes hard certificate | Yes | Yes |
| Minimum physical linear margin of recorded witness | 9.58522e-6 | 9.58555e-6 |
| Minimum physical terminal-cone margin of recorded witness | 0.0229722 | 1.60071e-5 |

The margins are reported in their respective constraint coordinates, not all in meters. These are sufficient existence witnesses for the declared affine model and current scene constraints. They do not establish an automatic initializer, physical nonlinear-vehicle safety, a new closed-loop recovery result, or a 100 ms worst-case runtime.

## Why simple changes are insufficient

**Shortening the horizon:** ten horizons from 0.4 to 6.4 s were inspected. At the largest midpoint-overlap-free horizons, stationary 1.3 s and oncoming 0.9 s, the current selected finite-exit row is already impossible under the actuator box alone. For a row `m*U <= b`, compute the exact minimum over the box:

\[
\min mU=m_+u_{\min}+m_-u_{\max}.
\]

The best possible exit margins `b-min(m*U)` are -16.0619 m and -19.0729 m. These negative values prove failure of this necessary condition even after ignoring collision and other constraints. Longer horizons contain overlapping anchor points. This ablation concerns the current selected exit row; it does not rule out every conceivable exit direction or certificate redesign.

**Small geometric perturbations:** at longitudinal alignment, lateral offsets of 0.001, 0.1, 0.5, 1.0, and 1.8 m all leave the ordinary distance and production direction norm zero. At 2.0 m the distance becomes 0.1 m, still below the required 0.25 m clearance. These point probes are geometric sensitivity checks, not dynamically feasible trajectories or a prescribed lateral maneuver.

**More solver iterations:** the zero interior dual is the correct mathematical optimum, so more iterations cannot supply the missing information. The main control solve is not reached.

**More CLF slack:** it relaxes performance after geometry construction and does not affect this early failure.

## What the cited methods establish

Li et al. use the ordinary-distance dual in Section 3, equation (12). Their equation (13) also includes a collision slack and its quadratic penalty; our collision certificate remains hard, while our slack belongs to the CLF. Accordingly, adopting their dual subproblem is not a reproduction of their entire optimization scheme. The inspected formulation supplies no general initialization guarantee for an arbitrary overlapping anchor. [Li et al., *Real-Time Optimal Trajectory Planning for Autonomous Driving with Collision Avoidance Using Convex Optimization*, 2023](https://link.springer.com/article/10.1007/s42154-023-00222-7).

Freezing zero dual multipliers makes a hard distance row `0 >= dMin` impossible. Adding collision slack changes it to `s >= dMin`, which has no trajectory dependence in that row. Other stages could still move the plan, but slack alone supplies no local escape direction and cannot certify hard safety.

Signed distance includes penetration information. Zhang, Liniger, and Borrelli derive a signed-distance dual with a unit-norm equality, replacing the ordinary-distance inequality. The equality is nonconvex; this is not a drop-in equivalent convex SOCP. Their analysis also distinguishes ordinary distance's zero interior from penetration depth. [Zhang et al., *Optimization-Based Collision Avoidance*, Sections 2.4 and 3.2, arXiv:1711.03449v3](https://arxiv.org/html/1711.03449v3).

The justified next design question is an overlap-capable continuous initialization/restoration procedure within the retained geometry framework, followed by the unchanged hard certificate. It must resolve direction degeneracy without prescribing a maneuver, return an actual feasible witness before execution, and be measured against the frame deadline. A signed-distance formulation is a research candidate, not an implemented or proven real-time remedy. The existing predictive continuation protects an already admitted witness; it cannot create the missing first witness.

## Reproduction and validation

Driver: `scripts/diagnoseDistanceDualInitialization.m`. MATLAB R2026a Update 3. Configuration: 8 m/s reference, 0.1 s hold, 16 m complete sensing region, no road boundaries, exact affine prediction and zero uncertainty/target jerk/yaw acceleration; 60 s diagnostic work budget. No random perturbations or seed are used. The self-contained mode reproduces the oncoming target-free prefix with the current controller.

```matlab
addpath('scripts');
a = diagnoseDistanceDualInitialization('/tmp/dual-initialization');
```

Optional recorded numerical witnesses are original experiment outputs at `/home/zai/.cache/collisionAvoidance/li-dual-20260916/adopted-profile/{stationary,oncoming}Admission.mat`, associated with revision `8a9344d3c796eb98d02eb2b747c90014638e661c`. Only scene data and control values are used, not their old configuration or certificate.

```matlab
b = diagnoseDistanceDualInitialization('/tmp/dual-witness-recheck', ...
    WitnessDirectory='/home/zai/.cache/collisionAvoidance/li-dual-20260916/adopted-profile');
```

Actual outputs: `/home/zai/.cache/collisionAvoidance/dual-initialization-diagnosis-20260916/`, including `self-contained/`, `final/`, midpoint/horizon CSVs, raw dual MAT files, summary JSON, `validation.mat`, and `final-diagnosis.log`. Without optional witness data, `provided=false` means the witness experiment was not performed.

Both modes reproduced the production initialization exception and passed all diagnostic assertions. Factory Code Analyzer reported zero findings for the new script. A separate Python audit checked both 80-row midpoint datasets, all 20 horizon rows per mode, strict-interior dual degeneracy, negative exit bounds at every sampled overlap-free horizon, and both current hard-feasible witness/optimization results. The full unit suite was not rerun for this offline-only addition; the preceding implementation report records its own earlier run. No new end-to-end or observer-controller performance campaign is claimed.

Research assistance: AI-assisted source inspection, primary-paper comparison, mathematical derivation, and diagnostic scripting were checked against saved MATLAB outputs and an independent data audit. Report conclusions are limited to those artifacts and stated assumptions.
