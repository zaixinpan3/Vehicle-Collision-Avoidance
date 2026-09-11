# Complete sampled-and-held nonlinear Fiala inclusion

September 10, 2026. This implements the next gate after
[local interval residual verification](FIALA_INTERVAL_INCLUSION.md): a complete
100 ms sample with one feedback evaluation, explicitly bounded affine error,
and shared state/input generators retained through internal time subdivision.
The exact modified Fiala world model and its binary parameter values define
the true dynamics for this result. No extra tire resultant-force constraint
or avoidance-mode selection is introduced.

This is an independent verifier component. The online encounter controller
has not yet been replaced. An accepted result proves nonlinear flow inclusion
and the stated command bounds for one sample. It does not establish collision
clearance, road containment, an encounter exit certificate, contraction,
recursive feasibility across samples, or a real-time execution guarantee.

## 1. Interface and execution contract

```matlab
certificate=certifyFialaFeedbackSample(lower,upper,[z;v],K, ...
    measurementRadius,previousInput,cfg);
```

The six states are `[px,py,psi,vx,vy,r]` in a persistent Cartesian frame.
The two commands are front steering angle and signed longitudinal force
ratio `[delta,beta]`. The sampling duration is `cfg.controller.sampleTime`.
The inlet is a box, the measurement error is a box, and `previousInput` is
the known prior held command. The modeled command is the exact affine law

    u = v + K*(x(0)-z+nu),       nu in [-measurementRadius,measurementRadius],
    u(t) = u,                   0 <= t < h.                    (1)

This zero-delay, stateless command contract has no saturation or actuator
tracking dynamics. Input and intersample slew bounds are checked on the
complete uncertain command before propagation. The slew contract is
`abs(u-previousInput) <= h*rateMaximum`; it is not a continuous steering-motor
rate proof. Numerical command-evaluation errors, actuator memory, delay and
tracking errors require explicit augmentation before this component can
certify an executor implementing a different contract.

`accepted` is true only when all accepted cells cover exactly `[0,h]` and
the execution check passes. A partial prefix has `accepted=false`, even if
its `verifiedThrough` time is positive. Budget or domain failure never
produces a substitute input. `trajectoryCertified` deliberately remains false.

The local regularity domain is inherited from the interval Fiala checker:
positive speed above its floor, strictly interior `abs(beta)<1`, and bounded
heading, steering and actual axle slip. These are explicit restrictions of
the certifiable domain. They are not inferred from successful simulation.

## 2. Correlated augmented inlet

Augment the state by the held command, q=[x;u], with q_dot=[f(x,u);0]. Store

    Q = c + G*[-1,1]^12 + R,                                  (2)

where R is an interval box. Six shared generators encode inlet state errors;
six more encode measurement errors. The command rows contain K times the
corresponding generators. Thus, the state deviation and the resulting held
command retain their common latent variables throughout the sample.

All products and center defects are computed using outward MPFR arithmetic.
Generator midpoint rounding is enclosed in R. A componentwise outer box of
(2) is used for domain checks, but does not replace G in propagation. The
command center, all command generators and its remainder are copied exactly
at every internal cell boundary. Model relinearization never re-evaluates (1).

## 3. Cell-domain proof and explicit affine residual

For a proposed cell of duration d, let X be the state projection of the
inlet outer box and U its held-command projection. Choose a convex domain
D containing this inlet and the nominal center. Compute a flow enclosure
and require

    S = X + [0,d]*f(D) subset interior(D_x).                    (3)

Here D_u=U remains constant. The strict condition is applied only to the
dynamic state coordinates. Domain generation starts from a short Picard
probe and expands it conservatively; unsuccessful candidates are subdivided.

On the supported compact domain the Fiala flow is locally Lipschitz. Before
a hypothetical first domain exit, its derivative belongs to f(D). Integration
places the exit state in S, strictly inside D_x, a contradiction. This proves
existence throughout the cell, swept inclusion (3), and validity of every
model bound on D, without assuming the endpoint tube that will be computed.

At c select representable affine coefficients F and a, with zero bottom
rows. The shared interval automatic Jacobian gives J(D). Bound

    w(D) subset f_aug(c)-F*c-a + (J(D)-F)*(D-c).                (4)

The segment from c to each q in D stays inside D. Integrating the derivative
of f_aug(q)-F*q-a along that segment proves (4). Every intersecting tire
branch is included in the interval Jacobian; no sampled residual maximum is
used. The center defect includes rounded affine coefficients and offset.
The two held-command residual rows are exactly zero.

## 4. Validated endpoint propagation

For the constant cell affine system define

    T(d)=exp(F*d),             Gamma(d)=integral_0^d exp(F*s) ds.

The exact augmented trajectory satisfies

    q(d) = T(d)*q(0) + Gamma(d)*a
           + integral_0^d exp(F*(d-s))*w(q(s)) ds.              (5)

The residual may vary arbitrarily with time inside (4). It is not replaced
by a constant disturbance. With r=max(abs(w_lower),abs(w_upper)), a valid
componentwise radius for its integral is

    integral_0^d exp(abs(F)*s) ds * r.                          (6)

The inequality follows from the matrix exponential series and the triangle
inequality. In particular, `abs(exp(F*s)) <= exp(abs(F)*s)` componentwise.

Both matrix maps use outward interval Taylor arithmetic through order p=12.
Let alpha be an outward bound on the induced infinity norm of F and z=alpha*d.
For the exponential remainder use

    tau = exp(z)*z^(p+1)/(p+1)!.

Indeed, `(p+1+j)! >= (p+1)!*j!`, so the omitted scalar series is at most tau.
This norm bound also bounds every matrix entry. The integral remainder has
norm at most d*tau. The positive-matrix polynomial in (6) is evaluated
componentwise; its omitted contribution is charged once as
`d*tau*norm(r,inf)` to every dynamic component. Known zero held-command rows
retain their exact transition and zero disturbance rather than a generic
scalar remainder.

A nonlinear RK4 step proposes the next nominal center c_next. It supplies
no certified integration bound by itself. Instead, the implementation charges
the complete interval difference `T*c+Gamma*a-c_next` to the remainder:

    G_next = midpoint(T*G),
    R_next contains T*R + T*c + Gamma*a - c_next
                    + generator_rounding_error + disturbance_integral. (7)

For each interval generator coefficient, the difference from its stored
midpoint contributes a symmetric outer remainder. Thus (7) contains (5)
even if the proposed RK4 center is inaccurate. Repeated domain checks and
relinearization reduce local defects without assuming them to vanish.

Serialized cell endpoints are binary64 values. Their exact real difference
is enclosed by directed subtraction `[d_lower,d_upper]`. The verifier proves
the flow through d_upper, then adds

    -[0,d_upper-d_lower]*f(D)

to its endpoint remainder, with the gap itself rounded outward. This covers
the exact serialized endpoint time by integrating backward along the already
enclosed short suffix. Adjacent cells use identical stored endpoint/start
values, so no unverified time gaps arise. The endpoint outer box must also
fit the declared domain; otherwise the candidate is refined.

Induction over accepted cells now proves: every realization of the inlet and
measurement errors under (1) belongs to each reported swept enclosure and
each reported endpoint set. The original command correlations are retained
in all cells. If the final endpoint equals h and the execution checks passed,
the complete sampled flow is certified. This induction is within one held
sample; it is not the encounter suffix theorem.

## 5. Reproducible experiments and limitations

`evaluateFialaFeedbackSample` uses the default 10 m/s trim, h=0.1 s, and

    K = [0,-0.3,-1,0,-0.03,-0.03; -0.2,0,0,-0.3,0,0].

The inlet radius below is applied numerically to each state coordinate in
its own SI unit; it is not a dimensionless common physical distance.
Measurement radii are one tenth of the inlet radii. Previous input equals
the nominal command. These are deterministic local validation fixtures,
not calibrated sensor contracts or an avoidance maneuver.

| Inlet radius | Steering (rad) | Complete sample | Accepted cells | Observed time (s) |
|---:|---:|:---:|---:|---:|
| 1e-4 | 0 / +0.04 / -0.04 | All pass | 20 each | 0.234 / 0.262 / 0.280 |
| 1e-3 | 0 / +0.04 / -0.04 | All pass | 20 each | 0.231 / 0.281 / 0.271 |
| 1e-2 | 0 | Rejected at 0.0908984 s | 24 | 0.513 |
| 1e-2 | +0.04 / -0.04 | Rejected at 0.0761328 s | 21 each | 0.536 / 0.543 |
| 1e-4, initial 0.1 s cell proposal | 0 | Pass after 11 refinements | 6 | 0.169 |

The fixed 5 ms subdivision is the default. The adaptive coarse proposal is
faster in this example but much looser: its final straight-case lateral
position radius is approximately 0.002416 m, versus 0.0001788 m with 5 ms
cells. Cell count alone is therefore not a sufficient optimization objective.

At inlet radius 1e-2 the straight-case final retained remainder radii in
lateral velocity and yaw rate reach approximately 4.24 m/s and 6.29 rad/s.
The interval derivative/residual bounds have become very large. This
demonstrates enclosure inflation and limits the current useful certification
domain. It does not demonstrate unsafe true motion or prove physical
infeasibility. The boxed nonlinear remainder and broad tire-branch derivative
hulls are candidates for tighter validated propagation or spatial subdivision.
Their individual contributions have not yet been isolated quantitatively.

Each of the seven complete cases also passed 64 deterministic cosine-point
replays with 20 numerical RK4 steps per certification cell and zero observed
containment violation. Sampling is an implementation falsification check;
the set inclusion follows from (3)--(7). The command bounds remained bitwise
identical across every cell. There is no random seed because no RNG is used.

All **63/63** related tests pass: 9 complete-sample tests, 14 residual tests,
25 tire-model tests, 12 prediction tests and 3 sampled-feedback tests. The
new tests cover complete coverage, nonzero steering, shared held commands,
adaptive refinement, partial-prefix rejection, actuator bounds and slew
rejection. Six MATLAB files have zero factory Code Analyzer findings; both
C++ entry points pass C++17 syntax checks with `-Wall -Wextra -Wpedantic`.
The full repository suite was not repeated because the online path is unchanged.

```matlab
addpath('scripts','controller','config');
evaluateFialaFeedbackSample('/tmp/fialaFeedbackStudy');
results=runtests({'tests/fialaFeedbackSampleTest.m', ...
    'tests/fialaResidualCertificateTest.m','tests/modifiedFialaTireTest.m', ...
    'tests/ltvBicyclePredictionTest.m','tests/sampledFeedbackTransitionTest.m'});
assertSuccess(results);
```

Builds require system MPFR/GMP. `buildFialaIntervalVerifier` now compiles and
executes both fresh MEX verifiers. Their interval arithmetic, Fiala equations
and parameter extraction are shared to prevent divergent verifier versions.
Generated binaries and raw experiment exports remain outside the repository.

## 6. Next implementation gate

The next gate is propagation across actual actuation boundaries with a
correlated inlet and a new measurement-conditioned feedback command, followed
by full-encounter nonlinear swept collision and road checks. It must preserve
the policy and inlet needed for the suffix proof in
[the feedback certificate design](FEEDBACK_POLICY_CERTIFICATE.md). The present
API accepts only an inlet box; repeatedly calling it would discard intersample
correlation and does not implement that gate.

Before attempting the complete oncoming encounter, tighten and diagnose the
remaining enclosure inflation at useful error radii. Retaining initial
generators alone does not guarantee a bounded tube. The measured 0.169--0.281 s
successful verification times exceed the 0.1 s actuation period on this host;
they are not worst-case bounds. Offline proof generation, proof reuse and an
independently scheduled executor still need to be integrated and measured.
No perception-exit deadline is imposed by this one-sample experiment.
