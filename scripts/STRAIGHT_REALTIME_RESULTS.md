# Straight avoidance: complete-frame runtime optimization

**Timing correction, September 8, 2026:** the results below are historical
computation measurements with instantaneous actuation in simulated time.
Their 71.802/94.244 ms maxima exceed the 50 ms control period. They do not
pass a runnable 20 Hz control requirement, even though they are below a
separate 100 ms threshold. The strict entry now uses a 100 ms control period
and explicitly scheduled actuation delay; see
[the corrected execution study](STRAIGHT_DELAYED_EXECUTION_RESULTS.md).
The historical raw results and their original archive record are preserved.

Research date: September 8, 2026. The current requirement is **100 ms for
every complete online frame**, including the observer and road fitting.
The earlier 50 ms runtime requirement is superseded. Physical experiments
are restricted to the straight scene, with the controller-only gate first.

## Timing and physical scope

The frame timer starts before synthetic sensor/input generation and ends
after the controller returns its independently verified command. It includes
the NRMM updates, published error bounds and sensor audits, road fitting,
planning, numerical solves and independent acceptance. Every attempted frame
is retained, including frame one, target acquisition and failed attempts.
Offline preparation, plant integration, plotting and result exports are
separate costs. Actual YOLO/LiDAR inference is absent from this synthetic
experiment and has not been timed here.

`EnforceRuntimeDeadline=true` stops before plant advancement if a fresh solve
fails or a frame exceeds the budget. An expired command is never applied;
no backup controller or stored input is substituted. The reusable
`runStraightRealtimeValidation` entry enables this rule and starts the joint
experiment only after the pure controller passes.

The mathematical sample period remains **0.05 s**, with a 32-stage, 1.6 s
lookahead and one robustly certified held interval. The 100 ms measurement
budget is distinct from that discretization. The harness advances simulated
time after computing the command; it does not inject computation delay.
Consequently an observed 100 ms pass does not establish schedulability at
20 Hz, delay-aware closed-loop safety, a platform WCET bound, or exact
asymptotic convergence. No early recovery deadline is introduced.

## Historical straight physical results without computation delay

The historical sequential validation passes its instantaneous-actuation
functional gates and separate 100 ms computation threshold. Each run completes
600 attempted and applied intervals over 30 s, with zero solve failures,
zero deadline misses, fresh independently checked controls and no fallback.

| Metric | Controller only | NRMM + controller |
| --- | ---: | ---: |
| Median complete frame (ms) | 25.229 | 32.048 |
| 95th percentile complete frame (ms) | 30.318 | 37.724 |
| Maximum complete frame (ms) | **71.802** | **94.244** |
| Time of maximum frame (s) | 0.05 | 3.55 |
| Minimum sampled rectangle SAT separation (m) | 0.429478 | 1.537276 |
| Maximum speed error in 29--30 s (m/s) | 0.006939 | 0.011929 |
| Maximum lateral error in 29--30 s (m) | 0.000002 | 0.005012 |
| Maximum body-heading error in 29--30 s (rad) | 1.48e-11 | 0.000735 |
| Fitted road-boundary checks | Pass | Pass |

The joint maximum occurs at target acquisition, with 12.981 ms observer,
1.977 ms road fitting and 79.236 ms controller time. The remaining 0.050 ms
is surrounding frame work. That frame performs four numerical solves and
one full nonlinear refinement. The observed margin to the budget is only
5.756 ms; these measurements establish a pass for this recorded execution,
not a bound on future operating-system scheduling or unmeasured work.

All 60 target publications from 3.55 through 6.50 s contain sampled truth
in all eight published components: Cartesian position, velocity,
acceleration, heading and yaw rate. The ego premise and published-bound
checks also pass. The maximum sampled kinematic yaw-model mismatch is
0.129276 rad/s for exact observations and 0.176358 rad/s for the joint run,
both within the existing 0.25 rad/s observer premise.

Independent midpoint finite differences of the physical trace give maximum
absolute residuals against the nonlinear modified-Fiala/road-load model:

| State derivative | Declared allowance | Controller only | Joint |
| --- | ---: | ---: | ---: |
| Longitudinal position (m/s) | 0.2 | 0.000604 | 0.001836 |
| Lateral position (m/s) | 0.06 | 0.001829 | 0.003345 |
| Heading (rad/s) | 0.02 | 0.000408 | 0.000963 |
| Longitudinal velocity (m/s^2) | 2.5 | 0.947457 | 2.071525 |
| Lateral velocity (m/s^2) | 5 | 2.544307 | 2.710997 |
| Yaw rate (rad/s^2) | 4 | 2.371155 | 2.335134 |

The audit uses 30,638 and 30,641 intervals respectively. It excludes 899
and 891 intervals shorter than 1 microsecond to avoid ill-conditioned
finite differences at plant restarts. These sampled residual checks do not
prove a global continuous-time enclosure. The small remaining cruise errors
show practical recovery under physical model mismatch and measurement noise;
the experiment does not establish exact convergence to zero error.

The pure and joint runs execute in that order in one newly launched,
unpinned MATLAB process, each after its own independent preparation.
The recorded pipeline preparation phase takes 2.788 and 3.727 s respectively
and is outside periodic execution; those figures exclude other plant setup,
gain synthesis and binary building. All online frames, including startup
and acquisition, are included.
The final result is `physical-shared-validation/straight-realtime-validation.mat`;
the frozen source/build manifest identifies 208 files, including eleven
native MEX binaries in the tested installation. The final repository numerical sources match that
snapshot; only this result document changes afterward. Original raw results,
derived CSV/JSON audits, failed attempts and the standalone figure are
preserved in the external experiment evidence export.

## Diagnosis

The functional baseline is commit
`d0bea504e556315dcfc8abe5519f0a61e3c8e518`, documented in
[the preceding straight repair](STRAIGHT_CONTROLLER_REPAIR_RESULTS.md).
Its joint physical controller peak was 507.583 ms at 4.10 s: three nonlinear
refinements and twelve numerical solves. Replaying that sample confirmed
actual future heading/slip violations, rather than insignificant numerical
roundoff. Profiling was used to locate work; profiled durations are not
deadline measurements.

Three costs compounded: oversized conic problems, repeated nonlinear
prediction/formulation, and previously unused code paths at acquisition.
The first screened physical candidate completed 30 s but still reached
207.422 ms at target acquisition. With the compiled kernels and only eight
independent initialization probes, a fresh joint physical process stopped
at 3.55 s: **183.674 ms total**, including 20.883 ms observer, 1.867 ms road
fit and 160.871 ms controller. It performed four solves and one nonlinear
refinement; formulation/witness checking and acceptance took about 73 and
33 ms. This failure is retained. Earlier same-process diagnostic maxima
below 100 ms did not establish that fresh-process acquisition was prepared.
Extending preparation to 24 probes reduced that fresh-process peak to
127.251 ms (20.950 ms observer and 104.288 ms controller), which still
failed and stopped before applying the command. This motivated the further
endpoint transcription, observer compilation and batched geometry changes.
That next full sequential run passed the pure 600-frame gate (73.421 ms
maximum) but the joint acquisition still took **109.107 ms**: 18.085 ms
observer, 1.993 ms road fitting and 88.979 ms controller. It also stopped
before plant advancement. The final optimization therefore also compiles
the complete constraint projection and reuses constant observer comparison
flows. This third stopped joint attempt is preserved as a failure. The subsequent
projection-kernel run again passed the pure gate (74.913 ms maximum), but
joint acquisition still took **107.125 ms**, including 17.725 ms observer,
2.080 ms road fitting and 87.268 ms controller. This fourth stopped attempt
motivated removal of redundant observer publication and validation of one
RK4 step per held sensor interval. A subsequent one-step prefix still
expired at acquisition (110.272 ms); adding the held-interval kernel and
compact row signatures alone also expired (114.661 ms). Both stopped
attempts are retained. Later changes compile the target-history enclosure
and the independent nonlinear nominal checker. That version still expired
at 110.697 ms; enlarging the heading domain to 0.45 rad also expired at
107.280 ms and was reverted. Batched lane queries and initialization-only
prediction then reduced the unpinned acquisition frame to 104.727 ms
(14.978 ms observer, 2.020 ms road fitting, 87.677 ms controller), still a
failed deadline. None of these stopped runs is counted as a pass.

## Implemented changes

### Algebraic reduction with full independent checks

For each input component and future stage, amplitude and rate limits imply
an interval `[l_j,u_j]`. A condensed geometric row `a U <= b` is redundant if
its maximum over those intervals, plus an arithmetic guard, is below
`b - alphaMax * reserve`. This proves redundancy for **every** optional
reserve allocation. Bounds derived from the rate-limited input chain make
this screen tighter: expand all inputs about any one anchor, bound the
anchor by its interval and each intervening difference by the slew limit.
Every resulting expression bounds the support from above, so their minimum
does too. If the intervals are inconsistent, screening is disabled.

Only the solver transcription drops those rows. The original condensed
rows, all actuator limits, all swept geometric checks and all physical-unit
acceptance checks remain. Explicit row mappings preserve reserve allocation
and independent residual calculations.

Endpoint inequalities now use the already-present endpoint state directly,
instead of substituting its dynamics into each row. Initial-state terms in
scheduled-slip constraints remain explicit. This preserves the same affine
system while reducing sparse fill. For identical left sides, a row is removed
only if another bound dominates at both ends of the optional reserve interval,
or the bounds are exactly identical; affine dependence then establishes
domination throughout the interval. One captured refined acquisition problem
has 3,622 original inequalities and 766 retained solver inequalities, with
2,972 nonzero constraint coefficients versus 4,514 before the endpoint change.

Inside the first held interval, the initial state is fixed and a CLF
quadratic depends on only the two current inputs:

\[
 q_i(u)=u^T H_i u+a_i^T u+c_i,\qquad H_i=R_i^T R_i.
\]

A thin QR factor supplies the two-column root. The same inequality
`q_i(u) <= delta` is represented by the four-dimensional Lorentz cone

\[
 \left\|\begin{bmatrix}2R_i u\\t_i-1\end{bmatrix}\right\|_2
 \le t_i+1,\qquad t_i=\delta-a_i^T u-c_i.
\]

This replaces the former ten-dimensional lift. If a quadratic is uniformly
dominated by another over the admitted two-input box, it also cannot change
the maximum required common slack. The screen proves domination by a
center/radius bound on the quadratic difference, with a rounding guard.
Later certified intervals keep their original cones. All original CLF
quadratics remain in the independent checker. Unconstrained future slack
coordinates are removed only from the native numerical solve and expanded
before the usual post-solve slack recomputation; the public decision layout
is preserved.

### Fewer speculative solves and feasible initial guesses

When optional future reserve allocation is active, the controller now solves
its linear program directly, then solves the joint input/state and squared
CLF-slack SOCP. It no longer first attempts a full-reserve SOCP that can
fail before the LP. The reserve cap remains 0.25; an interior allocation
uses 99% of a sub-cap optimum, less the existing guard. This planning policy
is not an exact lexicographic optimum. Mandatory current uncertainty and
hard collision, road, model-domain and actuator constraints are unchanged.

Initial guesses for nonlinear prediction now satisfy input amplitude and
slew limits. These guesses initialize optimization and are never issued as
commands. Nominal failure diagnostics expose the offending row and stage.
The objective still jointly penalizes control effort, state deviation and
squared CLF relaxation. No desired acceleration or LQR feedback target is
added; aerodynamic and rolling resistance remain in the model.

### Native prediction and removal of duplicate work

Two MATLAB Coder kernels call shared static methods in `ltvBicycleModel`:
the nonlinear RK4 rollout with batched finite-difference sensitivities, and
the stage Jacobians with Metzler disturbance-flow calculations. RK4
substeps, tire forces, road load and uncertainty formulas are retained.
Vehicle parameters are runtime inputs, not scenario constants embedded in a
binary. MATLAB implementations remain available for numerical comparison.
Two shared numeric kernels in `avoidanceSafetyGeometry` build obstacle
and road support rows and project every complete cell constraint into local
and condensed coordinates. Both process the whole horizon in one call.
Model-domain, scheduled-slip, force and corridor rows, initial uncertainty
and optional reserves are preserved. The MATLAB wrapper retains labels and
the independent checker retains the full original constraint system.
Condensed state costs use one weighted stacked matrix product instead of
repeated products containing identically zero slack columns.
A third prediction kernel compiles all Taylor/Bernstein cells in one
certified held interval, including numerical and physical-disturbance
radii and cancellation-aware endpoint bounds. A third geometry kernel
batches the independent nonlinear lookahead check, including the scheduled
slip denominator at the interval start. A captured acquisition comparison
with the prior checker gives identical residuals for every checked row.
Sparse duplicate detection also uses an exact compact representation of
nonzero column indices and values, without quantization or hashing.
Generated adapters and binaries stay under `solver/bicycle`; the controller
still respects its 20-source-file budget.

Target forecasts are batched, unused target radii are not calculated, and
road-coverage frames are reused only for the identical anchor. The complete
set of swept-cell frame queries now shares one lane validation and one native
call, with a separate station radius for each cell. Polyline vertices and both
sides of their chart jumps remain enclosed. For a new maneuver, a temporary
initialization prediction retains exactly the nominal maps used to choose its
input seed while omitting future uncertainty tubes that will be discarded.
The maneuver-specific prediction rebuilds all tubes, and formulation explicitly
rejects any initialization-only prediction. The observer
still integrates every high-gain update and updates its complete bound state.
Intermediate sensor steps omit full output structures that no consumer
uses. A paired closed-loop diagnostic before/after that observer change
gave bitwise identical states and inputs; a unit test also compares the
entire one-output and two-output runtime states. At controller time,
acquisition decisions use the point pose before constructing the one full
current output. The adapter retains that published estimate; it advances
the internal runtime to the next sensor timestamp without building an unused
future output. For an already acquired track, the current publication and
the next observer interval now share their synchronized measurement admission
through the runtime `sample` action. The standalone `output` and `step` APIs
retain their previous semantics. Acquisition and dormant-track resets retain
separate handling when their radar inputs differ. Exact regression comparisons
cover radar correction and dropout, and all three 12 s diagnostic trajectories
and input sequences are bitwise unchanged by this fusion. A recorded
acquisition replay preserves the published
estimate and complete internal runtime to `1e-10`.

For a straight road, cruise equilibrium is available in closed form: zero
lateral state and steering, reference longitudinal speed, and longitudinal
input exactly balancing aerodynamic/rolling load after the actuator gain
and bias. This removes an unnecessary three-variable nonlinear equilibrium
calculation. The curved-road equilibrium calculation is unchanged.

The high-gain RK4 point integration is also compiled from the shared
`nrmmObserverRk4Interval` function, with all gains, domains and held
measurements supplied at runtime. It returns **every** accepted substep and
its first derivative. The original runtime then performs every substep
error-bound propagation and domain audit in the original order. Point
dynamics do not depend on those separate bound states, so computing their
trajectory together does not change the integration equations. A saved
pre-refactoring observer step agrees, including its bound/audit state, within
`1e-10`; native/reference tests also cover multiple targets, radar dropouts,
heading corrections and gyro-only propagation. The binary lives under
`solver/nrmm`. The error comparison system also caches its state/input
transition for the identical matrix and integration step. For constant
forcing, `exp(h*[H,I;0,0])` supplies both the homogeneous flow and the input
integral; changing measurements changes the forcing, not that transition.
This replaces repeated forced matrix exponentials without changing gains,
substeps, comparison equations or domain audits. Radar correction/dropout
and changed forcing are compared with direct augmented exponentials.
The Cartesian measurement-history enclosure is also compiled from its
shared MATLAB algorithm. Secants, interpolation remainders, common gyro
transport, heading bounds and coasting-age terms are retained; this does
not replace or reset the target high-gain observer. Identical complete inputs
also share the kinematic course correspondence across runtime, enclosure and
history calls. A changed measurement or bound takes the validation path. The
fixed velocity-disturbance coefficients are cached independently of the
changing held sensor errors; their actual forcing is recomputed every time.

The strict straight validation profile now takes **one RK4 step per 0.0125 s
held sensor interval**, instead of five 0.0025 s substeps. The high-gain
observer equations and synthesized gains are unchanged; each accepted step
still includes the a posteriori integration defect and every domain audit.
The general integration configuration retains its original step limit.
Replaying the same 12 s truth and noise stream with a 20-times finer
0.000625 s step gives identical gains and maximum component differences of
`3.91e-5 m` target position, `6.12e-4 m/s` velocity, `0.003215 m/s^2`
acceleration and `1.83e-5 rad` heading. Ego differences are below `1.08e-8`
in their respective units. All checked bounds contain truth in both
replays. The target linear correction has RK4 spectral radius 0.874722
at the selected step; this linear diagnostic is not a nonlinear stability
proof. A behavior test also checks containment through high-gain peaking
and the numerical-defect envelope over the interpolating path.

Initialization now exercises an independent synthetic acquisition sequence
with a separate random stream and temporary certificate. Twenty-four probes
include retained-certificate admission and nonlinear refinement. Expected
probe failures are recorded and discard only the temporary certificate;
synthetic kinematic probes do not execute the calculated avoidance commands.
No probe state, noise draw, plan or command enters the actual experiment.
All probe times, successful refinements and solver counts are recorded.
This prepares expensive code paths without reading the real future trace.

### Evaluated planning settings

The validation profile uses 32 stages and solver objective tolerance `1e-4`.
Hard feasibility tolerance remains `1e-7`, and independent physical checks
retain their existing tolerances. A 24-stage joint diagnostic failed at
3.90 s; a 36-stage diagnostic completed but required up to three refinements
and reached 160.666 ms. Subsequent 28- and 30-stage diagnostics completed
but needed up to three and two refinements, with maxima 123.930 and
108.439 ms; they did not improve the selected profile. Increasing the
sample period to 0.1 s with twenty
stages failed the pure diagnostic at 5.30 s. Extra future heading padding
failed the joint case at 3.65 s, and reducing the interior reserve allocation
from 0.99 to 0.90 failed at 4.80 s. These candidates were rejected.

Two enlarged initial-guess footprint variants reduced acquisition
refinement but caused later deadline or solve failures across noise seeds;
neither is retained. Exact full condensing made acquisition solves slower,
and partial condensing offered no dependable improvement after assembly
cost. An early-screen/coordinate-list assembly prototype also gave no
useful speedup and was reverted. Further native-kernel horizon comparisons
did not justify changing the selected 32-stage profile. These trials are
preserved separately from the final implementation. Compiling the entire
SOCP assembly was also rejected: sparse and dense native prototypes matched
the reference matrices but took roughly 16 and 10 ms per captured problem,
versus roughly 6 ms for MATLAB.

A reduced SQP correction could repair the initial nonlinear violation while
fixing every certified held input. It used two future-input directions and
a reduced optional reserve, with full independent checks. However, complete
noisy encounters developed later deadline overruns (including 105.142 and
100.382 ms in the final prototype), so the correction was reverted. A
0.45 rad heading-domain candidate passed three bicycle diagnostics but failed
the physical acquisition deadline (107.280 ms) and is not selected. Removing optional-reserve maximization caused acquisition
solve failures and is rejected. The final numerical framework therefore
retains the original 0.40 rad domain, reserve policy and full nonlinear
refinement procedure. A further active-domain tangent-cut prototype reduced
the first acquisition to three solves and no full rebuild, but changed later
planning enough to cause a 122.631 ms deadline failure at 3.90 s. Restricting
that local correction to small relative domain violations still failed at
113.560 ms. Both prototypes were reverted; no tangent-cut repair is retained.
Relaxing solver feasibility to `2e-7` or `5e-7` did not reduce the captured
four-solve cost; `1e-6` failed independent hard-row acceptance. The original
`1e-7` tolerance is retained. Condensing only the reserve LP also lost the
advantage of its sparse state dynamics: a captured problem took a median
5.646 ms with the retained lifted Clarabel formulation, 27.038 ms with a
condensed Clarabel LP, and 15.772 ms with condensed HiGHS dual simplex
(19 warm repeats, separate from deadline measurements). The backends agreed
on the reserve fraction at the displayed precision; neither alternative is
retained. Pinning the entire MATLAB process to CPU 6 also failed: the pure
controller stopped before its second plant interval at 119.438 ms. That
execution setting is not retained, and its slowdown was not causally isolated.

## Regression and diagnostic checks

The final relevant regression selection passes **401 distinct cases in 41 classes**,
with no failed or incomplete cases: 382 controller/observer/numerical cases
and 19 straight integration cases. This covers original versus lifted costs
and constraints, CLF cone equivalence, native versus MATLAB prediction and
geometry, all observer substep dynamics, bound propagation, target sensing
lifecycle, independent preparation, and stopping before a late or failed
command is applied. Circular physical scenario methods are excluded.
The count takes the latest result per test name from the complete relevant
selection and subsequent affected observer/adapter rechecks; those checks
are not concurrent with the final runtime experiment.

The new projection kernel is compared with the shared MATLAB calculation
for robust executed tubes and nominal future cells, including local endpoint
rows, scheduled-slip denominator derivatives, every uncertainty contribution
and both maneuver corridors. Cached comparison-system flows agree with direct
forced matrix exponentials for changed forcing and radar correction/dropout.
All changed/new MATLAB files are checked with factory Code Analyzer settings;
only 16 sparse block-assignment performance advisories remain in the SOCP
transcription. The core source count is still 20. Earlier fixture failures
and the stale test expectation of a 0.10 rad/s yaw-model premise are retained
in earlier test CSVs; the baseline configuration already declared 0.25 rad/s.

A nonlinear-bicycle diagnostic passes 12 s with exact observations. Three
joint 12 s diagnostics of the final single-step profile retain the mismatched
15 m/s target prior:

| Noise seed | Maximum complete frame (ms) | Minimum sampled SAT margin (m) |
| --- | ---: | ---: |
| 20260907 | 67.224 | 1.515185 |
| 20260908 | 67.158 | 1.511940 |
| 20260909 | 70.579 | 1.541814 |

Every diagnostic has 240 applied fresh commands, no failure and no fallback.
These cheap diagnostics use a nonlinear modified-Fiala bicycle and do not
include the fitted hard road-boundary constraints. They are separate from
the full physical gates, whose complete frame includes road fitting and the
resulting constraints.

## Reproduction

Build once before periodic execution, with MATLAB Coder and a configured
MEX compiler available for the numeric kernels:

```matlab
addpath('scripts');
buildAvoidanceSocpSolver();
buildBicycleNominalKernel();
buildAvoidanceGeometryKernel();
buildNrmmObserverKernel();
report = runStraightRealtimeValidation( ...
    OutputDirectory='/tmp/straight-realtime', RandomSeed=20260907);
```

The joint observer's gain synthesis also requires the repository's existing
YALMIP and SeDuMi dependencies. They are preparation dependencies; no online
gain optimization, dependency download or compilation is performed.

Both physical gates use PassVeh14DOF for 30 s, 10 m/s reference and target
speeds, an initial longitudinal gap of 100 m, lateral target offset 0.8 m,
5 by 2 m rectangles, a 30 m sensing radius and 1000 m of forward centerline.
The NRMM prior is 15 m/s while target truth is 10 m/s. Observer period is
0.0125 s and the largest integration substep is 0.0125 s in this profile. Noise and observer
domains remain as in the preceding repair. The plant residual allowance
`[0.2;0.06;0.02;2.5;5;4]` is empirical, not a globally proved continuous-time
enclosure. Full configurations and failed attempts are saved with results.

Measurements use Linux x86-64, MATLAB R2026a Update 3, an AMD Ryzen 7 7800X3D
and one MATLAB computational thread; the native Clarabel solver is also
single-threaded. The desktop is shared. No circular physical experiment is
part of this operation.

Exploratory same-MCP-evaluation replays immediately after tests also showed
316--436 ms, followed by 83--87 ms in a separate evaluation. These timings
are retained in `post-test-evaluation-timings.json`; their cause is not
established and no favorable repeat is substituted for a failed physical
frame. Post-build replays of the final projection kernel also ranged from
76 to 159 ms before initialization was exercised; those durations remain in
`projected-acquisition.mat`. The offline preparation is part of the declared
startup procedure. Final physical deadline assessment uses the separately
launched process and all its online frames after that procedure.

## Engineering references

MathWorks recommends profiling expensive application code and placing its
loops inside the generated MEX entry to avoid repeated call overhead. This
motivates the batched kernels; the numerical agreement and performance
claims above come from local checks, not the documentation.
[MATLAB algorithm acceleration](https://www.mathworks.com/help/coder/matlab-algorithm-acceleration.html),
[MEX acceleration practices](https://www.mathworks.com/help/coder/ug/best-practices-for-using-mex-functions-to-accelerate-matlab-algorithms.html).

Clarabel exposes separate feasibility and objective-gap tolerances. The
implementation changes only the latter in the validation profile and
continues to check the original physical constraints after solving.
[Clarabel settings](https://clarabel.org/stable/api_settings/).

The algebraic screening arguments are project derivations. They do not
extend the retained first-interval certificate into a recursive-feasibility,
nonlinear reachability or robust asymptotic-stability theorem.
