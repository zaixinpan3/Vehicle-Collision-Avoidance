# Joint-state predictive safety with sequential convexification

Date: September 28, 2026. MATLAB R2026a Update 3.

## Implemented design

The public controller now uses a nine-coordinate joint ego/target state. One
target has constant tangential speed and constant heading rate, treated as
independent parameters. Its autonomous analytic prediction is eliminated exactly
from the optimization variables. A separately supplied target acceleration must
agree with this model; acceleration can be inferred when heading rate is explicit.

The former optimization inside a previously certified control box is replaced by
sequential convexification of the nonlinear held vehicle dynamics and polygon
distance certificates. A lane-feedback rollout initializes the search and can
intersect the target. No collision-avoidance trajectory is supplied. Nonzero
signed support directions allow restoration from overlapping nominal rectangles.

Each convex iteration first minimizes the sum of nonnegative safety slacks with
an LP. A QP then minimizes the lane CLF objective while retaining the LP safety
optimum within the configured numerical tie tolerance. State and input trust
bounds are adapted using fresh nonlinear rollouts. The execution gate still
requires directed full-interval collision, road, physical, actuator, and invariant
terminal certificates. Positive search slack never authorizes a command.

The same lane CLF is active throughout. Stored feedback suffixes and the
invariant lane continuation preserve recursive feasibility under exact execution
and unchanged contracts. If the nominal endpoint settles before its validated
enclosure does, the lane-feedback continuation is extended and the entire
candidate is revalidated. This resolved an initial terminal-admission failure
without reducing the terminal or input-memory requirements.

The formulation follows Huang et al., *Predictive Control Barrier Functions:
Bridging model predictive control and control barrier functions*, Section III,
(8) and the shifted-candidate argument in Lemma III.5
([author preprint](https://arxiv.org/abs/2502.08400))
<!--ref:huang2025--><!--anchor:section:III-->.
The fixed-normal dual/trajectory iteration follows Li et al., *Real-Time Optimal
Trajectory Planning for Autonomous Driving with Collision Avoidance Using Convex
Optimization*, Sections 3.1–3.2, (9)–(13)
([published article](https://doi.org/10.1007/s42154-023-00222-7))
<!--ref:li2023--><!--anchor:section:3-->.
Both supplied local PDFs were examined. The two-body geometry, joint-state
terminal conditions, nonlinear admission gate, and secondary CLF are project
extensions. The complete equations and guarantee scope are in
[the controller theory document](../controller/NONLINEAR_PREDICTIVE_CBF.md).

## Validation

All **139 selected MATLAB cases passed**, with zero failures or incomplete cases.
Coverage includes the new joint dynamics and independent target heading rate,
dual restoration at overlap, admission without an avoidance plan, the safety
slack cap, rejected unsafe proposals, solver-failure continuation, invariant
terminal execution, interval kernels, and configuration/source-budget checks.
The complete core remains within 24 source files. A separate no-road-boundary
smoke check also completed both optimization stages and retained its certificate.

Five deterministic nonlinear ODE replays completed **266 holds**. Every issued
hold was independently integrated using `ode45` with relative tolerance `1e-11`
and absolute tolerance `1e-12`. At 31 samples per hold, no replayed state escaped
the reported interval enclosure. The independent Python polygon/road audit
passed all five cases, covering **8,246 samples**.

| Replay | Holds | Minimum sampled vehicle clearance | Minimum sampled road margin | Result |
| --- | ---: | ---: | ---: | --- |
| Lane recovery | 8 | No target | 3.0397 m | CLF decreases; short recovery prefix |
| Constant-curvature cruise | 8 | No target | 3.0221 m | Maintains the reference trim |
| Forced solver failure | 8 | No target | 3.0500 m | Certified continuation remains available |
| Oncoming, stored policy | 210 | 0.19210 m | 0.20696 m | Completes passing and lane recovery |
| Oncoming, normal replanning | 32 | 0.14105 m | 0.44585 m | Completes the requested 1.6 s prefix |

The oncoming fixtures use 8 m/s ego and target speeds, an initial target position
of `[24;0]` m and heading `pi`, 4.8 m by 1.9 m vehicles, a 4 m clearance on each
side of the road centerline, a 0.05 s hold, and 0.10 m required vehicle clearance.
All other parameters come from the committed controller defaults. There is no
random sampling or random seed. The circular reference curvature is 0.005 1/m.

The stored-policy case first constructs a 150-hold certified plan using 23 SCA
iterations and 46 LP/QP calls. A deliberately exhausted improvement deadline on
later calls exercises recursive execution: 209 calls inherit the certificate,
including 60 invariant-continuation holds. At 10.5 s the lateral error is
`-1.82e-10` m. The shorter replanning replay ends during avoidance with a 2.109 m
lateral error and is not evidence of completed recovery under repeated
optimization. Positive CLF slack during avoidance is reported, including a
62.35 maximum in the stored-policy replay.

## Timing and limits

Initial oncoming admission took 24.16 s in the stored-policy replay and 23.96 s
in the replanning replay. Later stored-policy calls had a 2.108 ms median and
3.038 ms 95th percentile. Normal replanning had a 6.777 s median and 10.789 s
95th percentile. This is **not a demonstrated 50 ms online optimization system**.
Initial construction can exceed the improvement time budget.

Recursive feasibility applies after a zero-slack witness is admitted, for exact
successors, the declared nonlinear plant, an unchanged global road contract,
the fixed target parameters, and the actual applied-input memory. Arbitrary
initial admission, disturbances, general road curvature, global nonlinear
optimality, and convergence from positive PCBF values remain unproved. The
terminal target-separation construction can be conservative. The native
continuous-interval certificate and the sampled replay audit have distinct roles.

The final selected-file Code Analyzer pass has no findings. The wider analyzer
export retains sparse-assembly/preallocation performance advisories and two
existing unused-variable advisories; these are not syntax errors. Its unused
target-parser argument warning was resolved before the final selected-file pass.

## Reproduction and artifacts

From the repository root:

```bash
matlab -batch "addpath('controller','config','scripts'); validateNonlinearPredictiveController('report/JOINT_PCBF_SCA_20260928');"
python3 scripts/auditJointPredictiveSafety.py report/JOINT_PCBF_SCA_20260928 --output report/JOINT_PCBF_SCA_20260928/independent-audit.json
git diff --check
```

The validation driver builds native kernels in a temporary directory. It selects
the nonlinear controller, Fiala interval/sample/tire, configuration, and source
budget suites; it does not run the older affine public-interface suites.

The accompanying directory contains full test results, code-analysis exports,
ODE replay traces, the independent geometry audit, a compact summary and hashes
of independent source/reference/result artifacts. Pre-existing target-prediction,
observer, scenario and document changes, reference PDFs, generated binaries and
external solver trees are excluded from this task's commit.
