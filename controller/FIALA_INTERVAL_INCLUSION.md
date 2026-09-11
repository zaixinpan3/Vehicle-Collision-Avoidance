# Verified Fiala residuals: implementation decision and first milestone

Implementation update, September 11, 2026: the following first-milestone
record retains its original experimental results. The shared tire interval
kernel and full feedback propagation have since been improved; see
[the tightening and finite-policy implementation](FIALA_FEEDBACK_TIGHTENING.md).

September 10, 2026. Following the feedback/sparse study, the next implementation
priority is **sound nonlinear model inclusion before further maneuver or
runtime optimization**. Modified Fiala is the true model under the project
hypothesis. Its difference from an affine prediction must be explicitly
bounded, not fitted from simulation maxima or set to zero.

This milestone implements a domain-wide residual checker and a sufficient
first-exit check for one held-feedback cell. It does not yet implement an
entire correlated feedback tube, collision certificate, admission procedure
or real-time safety executor. The existing online controller is unchanged.

## 1. Implemented interface and mathematical meaning

`fialaCertificate.residual(lower,upper,A,B,c,cfg)` receives explicit bounds on

    y = [p_x,p_y,psi,v_x,v_y,r,delta,beta].

It returns outward-rounded lower/upper bounds enclosing

    f_Fiala(x,u) - A*x - B*u - c

for every state/input in that entire box. `residualRateBound` is the
componentwise symmetric outer radius of this interval. Units follow state
rates: m/s, m/s, rad/s, m/s^2, m/s^2 and rad/s^2. The first three state
coordinates are Cartesian coordinates in a persistent local world frame.

The native implementation uses MPFR directed rounding at 128-bit precision,
then converts each operation outward to binary64 interval endpoints. Addition,
subtraction, multiplication, division, powers, square root, atan, tan,
sin, cos and tanh all follow this rule. It does not assume that ordinary
MATLAB transcendental operations plus a chosen epsilon are validated bounds.
The library's rounding contract is described in the
[GNU MPFR manual](https://www.mpfr.org/mpfr-current/mpfr.html).
System build metadata reports MPFR 4.2.2. No library is vendored.

The real-valued equations use exact binary parameters supplied by the caller.
The axle force scales are the same cached values returned by
`modifiedFialaTire.parameters`, shared with `fialaWorldDynamics`. The verifier
covers the mathematical model; a floating-point numerical plant integrator
has separate rounding and integration errors. Sampling-based tests allow
1e-9 for comparison with that numerical evaluator, **not** as a widening or
relaxation of the certified residual interval.

The supported compact domain requires v_x strictly above the speed floor,
|beta|<1, heading and steering within [-1.5,1.5] rad, and actual axle slips
within [-1.5,1.5] rad. The latter are computed from
atan((v_y+lever*r)/v_x)-delta, which equals the authoritative atan2 expression
because v_x is strictly positive. These are conservative local regularity
restrictions, not additional tire-force constraints. Domains outside them,
reversed bounds and arithmetic overflow are rejected.

The duplicated numerical/interval evaluators implement the same explicit
world-flow equations: rotated front longitudinal/lateral forces, rear forces,
road load, v_y*r and -v_x*r coupling, and yaw moment. Nonlinear-to-affine
bounds are computed against the supplied A,B,c; rounded tangent coefficients
and any affine-offset discrepancy therefore remain included in the residual.
A successful point comparison alone is not the justification for equivalence;
the algebraic flow and tire expressions must remain synchronized on edits.

## 2. Centered interval Jacobian enclosure

The checker computes a value and all eight partial derivatives as intervals
through the complete nonlinear expression. With a representable center y_c
inside D, L=[A B], and an interval Jacobian J(D), it encloses

    r(D) subset r(y_c) + (J(D)-L)*(D-y_c).                    (1)

Proof: integrate the derivative of f(y)-Ly-c along the segment from y_c to
y. The box is convex, so every point on the segment remains inside D.
Every participating derivative belongs to J(D). Interval multiplication and
addition contain the resulting integral; directed arithmetic retains outward
inclusion. The center residual is evaluated with intervals too, so rounded
A,B,c do not introduce an uncharged defect.

For modified Fiala, set t=tan(alpha), Q=capacity*sqrt(1-beta^2) and
q=C*|t|/(3Q). The adhesion expression is -C*t*(1-q+q^2/3), and the saturated
expressions are -Q or Q. Each branch that can intersect the input box is
included; values and derivative intervals are combined by an outer hull.
Evaluating a branch on extra points can widen its enclosure but cannot exclude
an admissible value. At zero slip, the generalized derivative of abs(t) is
[-1,1]. The composed force is continuously differentiable there, and its
actual derivative is enclosed. At the adhesion/saturation boundary the force
and its first derivatives match. Equivalently, the piecewise locally Lipschitz
mean-value argument holds almost everywhere along each segment. No globally
smooth Hessian or fixed tire branch is assumed.

This centered form avoids subtracting two independently wide flow intervals.
Near a valid tangent, shrinking a box therefore reduces the principal
remainder approximately quadratically. It does not guarantee a tight bound
near every saturation/domain boundary. The returned interval is valid even
when it is too conservative for an avoidance certificate.

## 3. Independent held-feedback domain check

An optional `feedback` structure supplies `inletLower`, `inletUpper`,
`nominal=[z;v]`, `gain=K`, `measurementRadius`, and `duration=h`. The checker
first encloses the command actually held throughout the cell:

    U = v + K*(X_initial-z+[-measurementRadius,measurementRadius]).

This calculation also uses directed interval arithmetic. U must belong to
the declared input domain. For the state domain D_x it evaluates

    S = X_initial + [0,h]*f_Fiala(D_x,U),
    X_end = X_initial + h*f_Fiala(D_x,U).                     (2)

`heldCell.accepted` is true only if S lies strictly inside D_x and the inlet
and held inputs satisfy their domain conditions. The first-exit argument is
then sound: before a hypothetical first exit, all derivatives belong to the
flow enclosure, so the state at that exit would still lie strictly inside
D_x, a contradiction. Thus (2) encloses the complete cell and its endpoint,
and establishes that the residual from (1) can be used throughout that cell.
No contraction assumption or numerical simulation is used in this proof.

This is a sufficient Picard-style enclosure, not an optimal reachable set.
State/input correlation is conservatively discarded here. A rejected large
cell can require time subdivision or a better enclosure; rejection does not
prove that the real motion is unsafe. No automatic enlargement of the model
domain or clipping of the feedback command is performed after checking.
`trajectoryCertified` stays false because collision/road/input/slew and a
complete terminal encounter policy have not been certified by this API.

The 5 ms cell in the experiment is a local validation example, not a change
to the online controller's 100 ms sample period. When building multiple
certification cells inside one held sample, the **original sampled command
set must be retained**. Recomputing K from every cell endpoint would certify
a different, faster-sampled controller and is not allowed.

## 4. Deterministic experiment results

`evaluateFialaResidualInclusion` starts at the default vehicle's 10 m/s trim
with a nonzero domain radius

    [0.001,0.001,0.001,0.03,0.01,0.01,0.005,0.003].

The affine coefficients are the existing operating-point tangent. No extra
plant discrepancy is assumed. The validated residual-rate bounds are:

| Component | Full radius | Half radius | Quarter radius |
|---|---:|---:|---:|
| p_x | 3.00450e-5 | 7.50562e-6 | 1.87570e-6 |
| p_y | 6.00200e-5 | 1.50025e-5 | 3.75031e-6 |
| psi | 0 | 0 | 0 |
| v_x | 0.00458614 | 0.00113651 | 0.000282911 |
| v_y | 0.03314944 | 0.00823701 | 0.00205300 |
| r | 0.04596561 | 0.01142211 | 0.00284692 |

The exact yaw kinematic row has zero remainder. Halving the box reduces the
largest bound by approximately four. Four unoptimized MATLAB-to-native calls
(including a further eighth-radius box) take 10.456--12.923 ms each on this
host. These are observational timings, not execution-time bounds.

A 5 ms held-feedback cell passes (2), with initial state radius 1e-4 in each
coordinate and measurement radius 1e-5. The declared state domain radius is
[0.1,0.01,0.01,0.2,0.02,0.02], and input radius is [0.02,0.03]. Its gain is

    K = [0,-0.3,-1,0,-0.03,-0.03; -0.2,0,0,-0.3,0,0].

Validated held steering lies in approximately [-0.0001496,0.0001496] rad,
and beta lies in [0.0146478842,0.0147578842]. The swept lateral position lies
in [-0.000709992,0.000709992] m and the swept yaw rate in
[-0.004615852,0.004615852] rad/s. Changing only the proposed duration to 0.5 s
fails domain inclusion and is rejected. The declared residual-rate bound for
this larger domain is [0.00142998,0.00400787,0,0.0625787,0.371713,0.503367].

The earlier oncoming affine witness from EV-0104 is also audited against its
original affine matrices, in its straight Cartesian-equivalent chart. Each
stage domain covers its nominal Bernstein coefficients plus explicitly stated
padding and input radius 0.001. The maximum domain residual bounds are

    [0.720545,0.656066,0,0.326141,2.261273,3.070794].

At the checked nominal points alone, maximum absolute enclosed residuals are
approximately [0.702915,0.648725,0,0.285046,1.854763,2.705253]. In particular,
lateral acceleration and yaw-acceleration differences are material even
before adding uncertainty. These are certified local model comparisons,
not a new safe trajectory or a claim that the true trajectory stays in those
stage domains. They explain why an affine-only direction improvement cannot
be transferred directly to exact Fiala execution.

## 5. Validation, reproduction and next acceptance gates

Fourteen new tests cover the whole-domain enclosure versus 656 independent
numerical points for each of five tire cases, quadratic radius scaling, the
exact yaw row, speed/slip/beta regularity rejection, arithmetic overflow,
accepted held-feedback containment and rejection of excessive duration or
out-of-domain feedback inputs. Tests use samples to falsify implementation
errors; the enclosure proof comes from (1), branch coverage and arithmetic.
Together with existing Fiala, prediction and sampled-feedback tests, **54/54
pass**. The four MATLAB files have no factory Code Analyzer findings. A C++17
syntax check with `-Wall -Wextra -Wpedantic` reports no findings. A full
repository regression was not repeated because existing online code is
unchanged and this is an independent verifier component.

```matlab
addpath('scripts','controller','config');
buildFialaIntervalVerifier('/tmp/fialaVerifier');
addpath('/tmp/fialaVerifier');
evaluateFialaResidualInclusion('/tmp/fialaStudy', ...
    '/tmp/feedback-sparse-work-20260910/geometry-final');
results=runtests({'tests/fialaResidualCertificateTest.m', ...
    'tests/modifiedFialaTireTest.m','tests/ltvBicyclePredictionTest.m', ...
    'tests/sampledFeedbackTransitionTest.m'});
assertSuccess(results);
```

Generated binaries remain outside the repository. The build requires MPFR/GMP
headers and libraries and compiles a fresh MEX. If this host emits its known
post-link inspection error, the builder requires actual execution of the
fresh binary and analytic flow checks; merely finding a file is insufficient.

The decided next gates, in order, are:

Implementation update: the complete held-sample part of gate 1 is now
implemented and proved in [FIALA_FEEDBACK_SAMPLE.md](FIALA_FEEDBACK_SAMPLE.md).
It generates local nonlinear nominal centers, relinearizes within certified
domains and retains shared command/state generators. It does not yet generate
a complete nonlinear avoidance candidate or complete gates 2--4.

1. Generate the candidate nominal motion from true Fiala dynamics and
   relinearize along that candidate. Keep the incumbent unchanged until the
   complete replacement passes. Propagate a complete 100 ms held-feedback
   interval through adaptive certification cells, preserving its sampled command
   and correlations.
   Check each cell's declared domain with this nonlinear inclusion component.
2. Retain signed generators and validated outer remainders across samples;
   verify the full state/input/slew tube. Compare full nominal nonlinear
   trajectories with the enclosing tube, including the diagnosed oncoming case.
3. Put continuous separation directions on that verified nonlinear tube,
   then require a complete finite-exit certificate and suffix closure under
   deliberately failed improvement solves. No nominal-only fallback is admitted.
4. Only after this safety test passes, integrate the short-prefix QP, proved
   full-information tail splice and independently scheduled executor.

This order preserves the user's specification: one avoidance controller,
explicit affine error, and conditional safety/feasibility through target exit.
