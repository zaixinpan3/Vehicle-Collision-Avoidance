# CLF slack rerun without artificial road boundaries

September 15, 2026. Controller/observer source:
`4c6574d1ef3df396d356d3030666e83ab7406fa3`. The initial experiments used a
fixed working-tree snapshot; every controller, configuration and estimator
MATLAB source in that snapshot was subsequently verified against this commit.
This task changes the experiment drivers and their boundary configuration,
not the controller or observer algorithms.

## Result

Adding CLF slack and removing the artificial road boundaries does not resolve
the main avoidance failures. In the final eight-case campaign, only the
exact crossing completes 30 s. Exact stationary/oncoming fail at 0.7/2.9 s;
the actual NRMM pipeline fails at 4.1 s. Removing the road boundaries leaves
all eight completion outcomes and failure times unchanged relative to the
soft-CLF campaign with boundaries.

An offline necessary-feasibility check identifies a hard linear constraint
that cannot be satisfied anywhere inside the actuator box at each exact
stationary/oncoming failure state. Its CLF-slack coefficient is zero. Thus
the remaining failures cannot be attributed only to a hard CLF or to the
artificial road boundaries. They also concern feasibility of the hard
single-hold safety constraints under the available actuator authority.

The final warm campaign has no frames above 100 ms. Separate cold startup
still takes 792.760 ms, and earlier warm repeats also overrun. Every-frame
real-time qualification is therefore not established. Failed trials stop
before issuing another command; positive separation in their saved prefixes
does not mean the avoidance task completed.

## Corrected scenario configuration

Both experiment drivers previously constructed physical boundaries at
`y=-5 m` and `y=+5 m`. They were hard constraints on the complete vehicle
footprint plus 0.25 m clearance, not just the reference path. For aligned
heading, the 1.9 m wide vehicle had an approximate center-position limit
`|d| <= 3.8 m` from those boundaries.

The user clarified that this experiment has no road boundaries. The following
drivers now default to `UseRoadBoundaries=false`:

- `runStraightControllerRerun`
- `runExactStateRecursiveFeasibilityScenario`
- `runDeclaredPlantEstimatorControllerScenario`

The reference centerline and all declared model-domain constraints remain.
In particular, the scene still uses `|d| <= 4 m`; removing physical road
boundaries does not permit leaving that model domain. Steering, input slew,
velocity, heading, tire-slip and obstacle constraints also remain.
`UseRoadBoundaries=true` explicitly restores the former +/-5 m experiment
for comparison. Reports record `roadBoundariesEnabled`; an absent road
margin is NaN/JSON null, not a claim of infinite measured clearance.

## Final sequential campaign

Seed 20260914, held-control period 0.1 s, 300 requested holds per trial.
The first six scenes use an 8 m/s reference. The physical-range pair uses
10 m/s, an opposing target initially at `[100;0.8]` m, velocity `[-10;0]`
m/s and current 30 m visibility. NRMM consumes actual observer output.

| No-road trial | Executed holds | Outcome/time | Maximum frame (ms) | Misses / attempted frames |
| --- | ---: | --- | ---: | ---: |
| Exact stationary | 7 | Infeasible at 0.7 s | 35.466 | 0 / 8 |
| Exact oncoming | 29 | Infeasible at 2.9 s | 17.793 | 0 / 30 |
| Exact crossing | 300 | Completes 30 s | 18.299 | 0 / 300 |
| Bounded stationary | 5 | Infeasible at 0.5 s | 5.648 | 0 / 6 |
| Bounded oncoming | 29 | Infeasible at 2.9 s | 5.271 | 0 / 30 |
| Bounded crossing | 0 | Initial problem infeasible | 5.675 | 0 / 1 |
| Exact range-gated oncoming | 42 | Infeasible at 4.2 s | 8.367 | 0 / 43 |
| Actual NRMM range-gated oncoming | 41 | Infeasible at 4.1 s | 27.261 | 0 / 42 |

All seven failures report native Clarabel status 2 and raise
`collisionAvoidanceController:optimizationFailed`. The exact driver attempts
one decision per requested hold and includes a failed decision in timing.
The range driver also computes a final unexecuted command if it reaches its
requested end; these range trials terminate earlier and include the failure
frame. This differs from older exact drivers that always included a final
unexecuted command.

Final NRMM first publishes its target at 3.6 s, executes five target-present
holds, and fails at 4.1 s. It therefore gets beyond first target admission,
but does not finish the encounter. Available sampled ego/target containment
and ego-premise audits pass. This result does not establish observer
divergence or successful joint avoidance.

Minimum sampled separation beyond the required clearance is approximately
4.3495 m in the exact stationary prefix, 8.5473 m in exact oncoming,
9.2697 m in exact crossing, and 12.9381 m in the NRMM prefix. No geometry
measurement exists for the zero-hold bounded crossing. Offline audits sample
eleven points per hold; the online constraints retain swept Bernstein rows.

## Controlled comparisons and hard input obstruction

The preceding independent hard-CLF baseline at revision `8da38bc` executed
`[7,29,300,5,28,0,42,41]` holds in the same case order. Adding slack with
the old road boundaries changes that sequence to
`[7,29,300,5,29,0,42,41]`. Removing only the road boundaries retains the
latter sequence. The extra bounded-oncoming hold is partial progress, not
task completion. These comparisons supersede an explanation attributing all
failure solely to hard CLF dissipation.

The offline diagnostic reconstructs each exact failure state, supplies no
road boundaries, and captures the same hard-safety/soft-CLF program. For a
linear row independent of slack, it minimizes the left side analytically
over `|steering| <= 0.698131701` rad and `-1 <= beta <= 1`. The following
rows remain impossible even if every other constraint, including the CLF,
is omitted:

| Failure state | Hard row, neglecting steering coefficient below 1.4e-21 | Best possible left side | Positive infeasibility gap |
| --- | --- | ---: | ---: |
| Stationary, 0.7 s | `0.041687053 beta <= -0.076787780` | -0.041687053 | 0.035100727 |
| Oncoming, 2.9 s | `0.041687053 beta <= -0.115839467` | -0.041687053 | 0.074152414 |

The diagnostic includes the tiny steering coefficient in its actual
calculation. CLF slack has exactly zero coefficient in both rows.
Unbounded CLF relaxation therefore cannot make these particular programs
feasible. This proves an input obstruction at the reached states; it does
not prove that collision is inevitable or that an earlier, differently
planned avoidance maneuver was impossible. The present one-hold controller
does not guarantee recursive feasibility.

## Long runs and slack-dependent recovery

The original range-gated oncoming scene was requested for 900 holds again;
it stops at 4.2 s, rather than being reported as a completed 90 s trial.
Additional no-road clear-cruise and exact-crossing runs each complete 900
holds. A clear-road recovery run starts with
`[d,ePsi,vx-vRef,vy,r]=[.05,.002,-.05,0,0]` and completes 300 holds.

| Trial | Holds | Median / max frame (ms) | Final tracking error norm | Maximum relaxed CLF residual |
| --- | ---: | ---: | ---: | ---: |
| Clear cruise | 900 | 2.802 / 22.445 | 0.000228636 | -5.68055e-6 |
| Exact crossing | 900 | 3.269 / 6.774 | 0.003002649 | -2.13498e-6 |
| Perturbed clear-road recovery | 300 | 2.721 / 3.832 | 0.000228636 | -4.46172e-6 |

All three runs have zero deadline misses and one recorded solve per executed
hold. The error norm combines the five state coordinates in their declared
units and is only a compact diagnostic. Clear cruise/recovery settle near
8.000228636 m/s with negligible lateral error. During 80--90 s, crossing
speed is 8.00005177--8.00005882 m/s, while lateral position ranges from
-0.00300207 to -0.00264611 m. This is small-error path following, not exact
zero-error convergence: the roughly 3 mm lateral offset is still changing.
The crossing has about 9.27 m additional separation and is a mild collision
case that does not require the difficult passing maneuvers.

Only the CLF is relaxed, with a nonnegative norm-cone slack and squared
penalty weight 100. The recorded bound has the form
`V(next) <= c V(current) + B + Bslack`. Its verification includes the
optimized slack allowance. Maximum unrelaxed residuals are positive in
crossing (`3.69680e-8`) and perturbed recovery (`5.34468e-6`), even though
their relaxed inequalities pass. End-of-run slacks remain approximately
`1.58e-4` and `1.89e-4`. Thus `clfDissipationCertified` must be read with
its `slackDependentSampledBound` scope, not as strict unrelaxed dissipation.

## Timing, validation and reproduction

The final warm campaign is not a worst-case benchmark. Earlier same-source
soft-CLF runs with boundaries have maximum frames 207.586 ms (stationary)
and 146.698 ms (NRMM), with 2/8 and 3/42 misses. An earlier no-road repeat
has a 171.840 ms stationary first frame. A separate fresh MATLAB process,
using the fixed no-road snapshot of the same controller, takes 792.760 ms
at first stationary admission and then fails at 0.7 s, with 1/8 misses.
Its successor maximum is 74.767 ms. Initialization/caching and workstation
scheduling are not controlled sufficiently to infer a certified timing bound.

All timed experiments ran sequentially after tests; no other MATLAB work was
launched by this task concurrently. Timing includes online input assembly,
observer/adapter work where applicable, controller work and failed decisions.
It excludes offline gain initialization, true-plant integration, audits and
saving. Overruns are measured, not applied as actuation delay. The plant is
the controller's declared affine held generator, not a validated nonlinear
physical vehicle.

The final focused run passes **89/89** cases, including three new behaviors:
a vehicle at d=3.9 m can operate without road boundaries while inside its
model domain; the former road boundary rejects that overlapping footprint;
and d=4.1 m remains inadmissible without roads. The pre-change hard-CLF full
suite separately passed 566/566 after the MCP response timed out but MATLAB
completed and saved results. That baseline count is not claimed for the
current soft-CLF tree. The initial soft-CLF selection passed 86/86.
Factory Code Analyzer reports zero findings in all four changed/new MATLAB
files; Git diff checks and saved-result/link validation pass.

From the repository root:

```matlab
addpath('scripts');
out = fullfile(tempdir,'clf-slack-no-road-repeat');
campaign = runStraightControllerRerun(SampleCount=300, Seed=20260914, ...
    UseRoadBoundaries=false, OutputDirectory=out);
% Set UseRoadBoundaries=true only for an explicit +/-5 m comparison.
```

First-six scene targets are stationary at `[15;0]`, oncoming at `[60;0]`
with velocity `[-8;0]`, and crossing at `[15;-4]` with velocity `[0;32]`.
Ego starts at `[0;0;0;8;0;0]`. Bounded cases use ego radii
`[.05;.05;.005;.05;.02;.005]`, target radii
`[.1;.1;.1;.1;.05;.05;.01;.01]`, jerk amplitudes `[.1;.1]` m/s^3,
yaw-acceleration amplitude .05 rad/s^2 and frequency 1 rad/s. NRMM uses
the existing 80 Hz configuration with .0125 s maximum integration step.
MATLAB is R2026a Update 3, GLNXA64, eight computational threads.

Raw outputs are under
`/home/zai/.cache/collisionAvoidance/clf-slack-rerun-20260915/`.
`final-no-road/` contains the final campaign, long runs, range audits and
input-obstruction audit. Earlier `road-bounded-campaign/`, `no-road-campaign/`,
source snapshots/manifests, cold results and test records are preserved.
The independent hard baseline remains under
`/home/zai/.cache/collisionAvoidance/controller-v25-rerun-20260915/`.
No generated artifacts, external solver trees or historical implementation
copies are added to the repository. The existing native solver SHA-256 is
`30f2cca72ef8622733f6bfd32f6567be0dafede7a6dfe94d050ca33c23344e5a`.
