# Fiala enclosure and runtime bottlenecks: diagnosis and implementation

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Mode: run
- Origin Date: 2026-09-11
- Verification Status: VERIFIED for the reported executable experiments;
  mathematical claims remain conditional on the explicit model contracts.
- Version Label: fiala_feedback_tightening_v1

The previous [one-sample milestone](FIALA_FEEDBACK_SAMPLE.md) failed at inlet
radius 0.01 and took longer than its 100 ms actuation period. This change
fixes the tire derivative enclosure, reduces interval arithmetic overhead,
introduces a directional second-order residual, retains new uncertainty
directions, and propagates a prescribed feedback policy across actual sample
boundaries. It preserves the exact modified Fiala hypothesis and explicitly
bounds affine linearization error. No avoidance modes or tire-force constraints
are added. The online encounter controller is still unchanged.

## 1. Isolated causes and measurements

The same ten one-sample fixtures were rerun before editing the numerical
algorithm. The original failures at radius 0.01 were reproduced for straight
and both turning cases; seven of ten cases completed.

Native timing for the first straight sample was approximately:

| Work | Native time (ms) |
|---|---:|
| Domain enclosure | 28.7 |
| Affine residual construction | 26.7 |
| Matrix maps | 86.4 |
| Nominal RK4 proposals | 40.6 |
| Endpoint generator propagation | 11.8 |
| Entire native call, including other work/output | 209.4 |
| Entire MATLAB call | 231.4 |

The largest measured component was the matrix-map arithmetic. Every old
elementary operation initialized and cleared two or three MPFR variables;
generic multiplication evaluated all four endpoint pairs in both directions.
Even zero derivative components incurred these costs. No attempt is made to
attribute a measured percentage to allocation alone: allocation reuse, exact
zero shortcuts and sign-aware products were evaluated as one arithmetic change.

The old Fiala interval implementation evaluated an adhesion polynomial on the
whole input box whenever any part of that box could be adhesive. Its derivatives
were then hulled with saturation derivatives. This is an outer enclosure, but
includes polynomial extrapolation where the tire is actually saturated, and
loses cancellation even inside the adhesion branch. Replacing only these
bounds made all ten cases pass, including the three original radius-0.01
failures. The derivative-bound change is therefore sufficient to remove that
particular one-sample failure, without reducing the inlet or measurement errors.

After that replacement, the arithmetic change reduced the ten observed call
times to approximately 36--65 ms. Further changes below add tighter residuals
and more retained generators; the final implementation must be judged by its
own timings, not those of this intermediate ablation.

The distinction between correlation retention and boxing matters across
samples too. An explicitly reboxed diagnostic at radius 0.001 failed in its
eighth sample. Preserving the augmented state/previous-command correlations
completed 50 samples. This diagnostic only isolated enclosure growth; its
fixed previous-input point was not a joint actuator-memory certificate.
The final sequence below uses the correlated previous command and checks
the actual command difference.

Repeated enclosure and set reduction are established sources of conservatism
in reachability analysis. Sparse polynomial zonotopes retain additional
nonlinear dependencies to mitigate wrapping; the present implementation uses
ordinary signed generators plus validated local remainders and does not claim
to implement polynomial zonotopes. See Kochdumper and Althoff,
[Sparse Polynomial Zonotopes: A Novel Set Representation for Reachability
Analysis](https://arxiv.org/abs/1901.01780).

## 2. Exact tire differential bounds

Let t=tan(alpha), Q=capacity*sqrt(1-beta^2)>0 and

    q = min(C*abs(t)/(3*Q),1).

The complete adhesive/saturated force can be written

    F(t,Q) = -sign(t)*Q*(3*q-3*q^2+q^3).

Its first partial derivatives are

    F_t = -C*(1-q)^2,
    F_Q = -sign(t)*q^2*(3-2*q).                              (1)

These expressions cover saturation by q=1. At zero slip, F_Q=0 and F_t=-C.
The force is decreasing in t; its magnitude is increasing in Q. Force bounds
can therefore be obtained at the appropriate box endpoints. The two cubics
on [0,1] are monotone and bounded by [0,1]; these exact mathematical properties
justify interval intersection with that range. This does not clip an executed
command, a residual, or a physical trajectory.

The partial second derivatives away from zero slip are

    F_tt = sign(t)*2*C^2*(1-q)/(3*Q),
    F_tQ = -2*C*(1-q)*q/Q,
    F_QQ = sign(t)*6*q^2*(1-q)/Q.                            (2)

They vanish at the saturation boundary. At zero slip, enclose sign(t) in
[-1,1] for (2). The first derivative is locally Lipschitz on the supported
compact domain, and second derivatives exist almost everywhere with these
bounds. The integral Taylor argument does not require a globally continuous
Hessian. The interval chain rule composes (1)--(2) with tan(alpha) and Q(beta),
including their curvature and the complete world dynamics.

`fialaIntervalCore.hpp` now shares the physical flow expression between the
ordinary interval Jacobian and a second-directional-derivative scalar type.
There is no separately maintained polynomial plant or fitted residual model.
The speed, beta and angle regularity conditions from the first milestone
remain explicit domain requirements.

## 3. Directional second-order affine residual

For a cell domain D containing nominal center c, let d range over D-c. The
scalar differential type carries a value interval, a first directional
derivative interval and a second directional derivative interval. Initialize
each augmented coordinate as `(D_j,d_j,0)` and evaluate the shared physical
flow. The resulting second derivative encloses

    d' H_f(y) d,       for every y in D and d in D-c.           (3)

This requires only one interval direction, not 64 explicit Hessian entries.
With representable affine coefficients F,a and J(c) enclosing the exact
Jacobian at c, use

    r(D) subset f(c)-F*c-a + (J(c)-F)*(D-c)
                         + 0.5*secondDirectionalBound.       (4)

Proof: for an actual displacement d, apply the twice-integrated fundamental
theorem of calculus to f(c+s*d), s in [0,1]. Convexity keeps the segment in D.
The remainder is the integral of `(1-s)*d' H_f(c+s*d) d`. Its nonnegative
weight integrates to one half, giving (4). The center defect and rounded
Jacobian difference remain explicit. Directional derivative values are
interval bounds over the entire declared domain, not values sampled along
a numerical trajectory.

The first-exit/Picard proof still certifies the domain before using (4).
Domain proposals now start with 5% inflation instead of 50%, expand only
violated coordinates by 10%, and must pass the same strict containment test.
These factors guide proposals; they do not relax acceptance. In an ablation
with a 128-generator inlet budget, the second-order bound extended radius
0.003 to 50 samples; tighter domain proposals additionally extended radius
0.005 to 50 samples. Radius 0.01 with the original gain still did not complete.

The validated matrix exponential, arbitrary time-varying residual integral,
RK4 center-defect accounting and exact serialized-duration correction remain
as proved in the one-sample document. Recentring a returned remainder may
shift a stored nominal center; that center is an enclosure/reference center,
not a claim of an exact nonlinear nominal solution.

## 4. Correlated continuation and bounded generator count

The augmented set is q=[x;u_previous] in c+G[-1,1]^g+R. Within a held sample,
the propagated dynamic remainder is converted to six new coordinate generators
after each accepted cell. Its center shift and conversion roundoff are charged
outward. Those directions are then propagated with their signs in later cells.
Held-command coefficients and their remainder stay unchanged; new local
generator columns have zero held-command rows.

At an actual sample boundary, reduce old generators if necessary, then convert
all eight remainder coordinates to shared generators before applying the new
feedback law. For retained generator index set S, the reduction is

    R_reduced = R + [-r_discard,r_discard],
    r_discard,j >= sum_(g not in S) abs(G_jg).                 (5)

Directed sums make (5) an outer enclosure. Ranking by scaled column size
affects tightness only; the inclusion proof holds for any retained index set.
The scales `[1,1,0.1,10,1,1,0.1,1]` specify the ranking coordinates, not
physical error assumptions or acceptance tolerances.

The next command is computed once from the current uncertain state:

    u_new = v + K*(x-z+nu).

It inherits every state generator and adds six measurement-error generators.
The difference u_new-u_previous uses the difference of their shared generator
coefficients, not two independently boxed command sets. Remaining box and
roundoff terms are enclosed explicitly. This proves the input and intersample
slew inequalities for every realization covered by the augmented inlet.
No actuator-rate proof for a continuous steering motor or delay queue is implied.

The default `maximumGenerators=64` applies at each sample inlet after feedback.
Thereafter each certification cell adds six generators: the default 20-cell
sample has at most 184. A 4096-column hard limit rejects further work rather
than discarding unaccounted uncertainty. With 128 inlet generators the turning
sequence had a 96.0 ms median and a 100.047 ms observed maximum; reducing the
inlet budget to 64 retained feasibility while lowering that cost, at the price
of wider final enclosures. Reduction is performed only at actual sample
boundaries, so it does not mutate the held-command representation inside a
sample.

`fialaCertificate.sample(...,inlet=previousCertificate)` checks that the
previous sample was completely accepted and that its true-model parameters
match. Its correlated endpoint supersedes the supplied box/prior-input point.
`fialaCertificate.sequence` constructs a finite prescribed policy from a
nominal input sequence and gain. Each new reference center comes from the
previous nominal endpoint. It stores the actual nominal/gain/noise contract
for numerical execution. It performs no target avoidance optimization.

Inductively, if every sample is accepted, the true initial augmented state
belongs to the inlet and every measurement error obeys its box, each executed
held command obeys its contract and the resulting continuous trajectory belongs
to the stored cell enclosures throughout the complete finite policy. Its
dynamic suffix remains usable without reboxing or recalculating earlier
commands. Collision, road and exit obligations must still be verified before
this becomes the encounter certificate in the user's requested theorem.

## 5. Arithmetic and scheduling changes

MPFR scratch variables are now reused per native thread. Addition/multiplication
by exact zero and multiplication by one use exact identities. A sign table
selects product endpoints before directed rounding; rounded products are never
used to guess which exact product is extreme. The independent test adapter
compares 100 signed, zero, subnormal and overflow cases against exhaustive
endpoint enumeration. Precision remains 128 bits with outward binary64
conversion and no host rounding-mode changes. This follows the allocation
guidance in the [GNU MPFR manual, Section 4.8](https://www.mpfr.org/mpfr-current/mpfr.html#Efficiency).

The successful domain flow enclosure is reused for duration correction, and
value-only evaluations avoid unnecessary Jacobians. Returned `timingSeconds`
separates domain checks, residuals, matrix maps, nominal proposals and endpoint
propagation. These clocks do not participate in the inclusion inequalities.

An optional `maximumComputationTime` stops unsuccessful certification with
`accepted=false` and reason `timeBudget`. It is checked between cell/refinement
attempts. One atomic attempt, result serialization and MATLAB wrapper work may
extend beyond that budget. It is a rejection budget, not a hard execution-time
proof. The default is unlimited for offline certification. A future scheduled
executor must retain its already complete incumbent while such work runs.

## 6. Final reproducible results

All experiments use the same exact default Fiala parameters, 10 m/s initial
speed and h=0.1 s. Each listed inlet radius applies numerically to each state
coordinate in its own SI unit; measurement radius is one tenth as large.
The original gain is

    K = [0,-0.3,-1,0,-0.03,-0.03; -0.2,0,0,-0.3,0,0].

All ten original one-sample cases now pass, including both signs of 0.04 rad
steering at radius 0.01. Final observed call times are 49.4--85.2 ms. These
times include the additional second-order and generator work; the earlier
36--65 ms arithmetic-only result is not the final end-to-end measurement.

The finite-policy experiment runs 50 samples (5 s). The turning input is
0.04 rad for the first three samples and zero thereafter. Its reference evolves
with the nonlinear enclosure; it is not a complete lane-change/avoidance plan.

| Inlet radius | Steering | Lateral gain factor | Accepted samples | Final lateral radius | Median / max call (ms) |
|---:|---|---:|---:|---:|---:|
| 0.001 | Straight | 1 | 50/50 | 0.730 mm | 60.3 / 67.3 |
| 0.005 | Straight | 1 | 50/50 | 6.814 mm | 46.1 / 48.9 |
| 0.001 | Pulse | 1 | 50/50 | 0.737 mm | 79.5 / 82.0 |
| 0.01 | Straight | 0.25 | 50/50 | 83.232 mm | 46.3 / 49.2 |
| 0.01 | Straight | 1 | 4/50; next call rejects | Incomplete | 43.5 / 82.8 |

The last case uses an 80 ms native computation budget. Unbudgeted ablations
also failed to complete this radius with the original gain; budgeting is not
the sole obstacle. The quarter-gain row is an explicit alternative gain
experiment, not evidence that the original gain passed and not a controller
mode. Its wider position enclosure illustrates that gentler feedback trades
tracking for less injected input/measurement uncertainty. This gain is not
installed in the online controller or claimed optimal.

Every complete sample in the sequence experiments passed 64 deterministic
cosine-point replays with fresh measurement errors each actual sample, 20 RK4
steps per certification cell, and zero observed swept/endpoint/command-change
violations. The inclusion proof is analytic and interval-based; replays only
try to falsify implementation errors. There is no RNG or inferred work-hour
allocation. Timing variation and limited host observations do not establish WCET.

All **78/78** related tests pass: 19 sampled-policy tests, 5 interval-kernel
tests, 14 residual tests, 25 tire tests, 12 prediction tests and 3 sampled
transition tests. Six changed/new MATLAB files have no factory Code Analyzer
findings. Both production C++ entry points and the test adapter pass C++17
syntax checks with `-Wall -Wextra -Wpedantic`. A template-renaming build error
was corrected before these final checks. The online path is unchanged, so
the full repository suite was not repeated.

```matlab
addpath('controller','config','scripts');
evaluateFialaFeedbackSample('/tmp/fialaSampleStudy');
evaluateFialaFeedbackSequence('/tmp/fialaSequenceStudy');
results=runtests({'tests/fialaFeedbackSampleTest.m', ...
    'tests/fialaIntervalKernelTest.m','tests/fialaResidualCertificateTest.m', ...
    'tests/modifiedFialaTireTest.m','tests/ltvBicyclePredictionTest.m', ...
    'tests/sampledFeedbackTransitionTest.m'});
assertSuccess(results);
```

Actual baseline, ablation, final MAT/JSON and check logs are exported to the
external evidence archive; generated native binaries remain excluded.

The next integration gate is to attach synchronous swept footprint/road
constraints and perception-exit obligations to a complete finite nonlinear
policy, then exercise its retained suffix while improvement optimization fails.
Acceptance of the first **single-sample flow check** is not the first complete
joint encounter optimization required by that theorem. The present 5 s
experiment does not impose a target-exit deadline and does not by itself prove
collision freedom or the practical size of the full encounter admission domain.
