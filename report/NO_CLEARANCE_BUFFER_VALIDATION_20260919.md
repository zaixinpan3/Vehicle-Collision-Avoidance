# Remove the fixed physical clearance buffer

September 19, 2026. Controller-only implementation and validation.

## Outcome

The `collision.clearanceMargin` configuration and its runtime uses are removed.
Collision certificates now require separation of the occupied sets without an
additional physical offset. Existing uncertainty, chart-error and numerical
reserves remain. Experimental noncollision acceptance is signed body gap > 0.

**Removing the parameter is complete, but the resulting controller does not
avoid collision in every tested hold.** All fifteen offline trials numerically
complete 300 holds and return to cruise; thirteen pass the sampled noncollision
check. Straight stationary and oncoming trials have positive certified node
gaps but negative inter-node physical gaps. Their report `passed` flags are false.
The existing offline audit records the entire trace rather than terminating at
an observed collision; completion alone is not safety success.

## Implementation

- Remove the default and validation block; the existing strict configuration
  merger rejects an override containing the removed field.
- Set the collision records' physical offset to zero. Keep the separate
  encounter-exit distance, uncertainty support and floating-point allowances.
- Remove the same physical offset from geometry, optional road-boundary rows,
  perception-region admissibility and encounter-duration proposals. Preserve
  boundary-fitting error and all actuator, CLF and terminal checks.
- Reduce numeric geometry settings from five entries to four and update the
  MATLAB Coder build interface and test fixtures. Rebuild all three geometry,
  projection and support MEX kernels locally. No generated binaries are committed.
- Bump carried controller-state format from 37 to 38: the retained geometry data
  has changed shape. Existing runs must start with an empty controller state.
- Update scenario and diagnostic drivers to report physical gap directly;
  remove the obsolete configured-buffer-retention metric. Historical reports
  and original raw traces keep their original configurations and measurements.

The predictive CBF/soft-CLF formulation, joint-support admission method and
carried-certificate continuation remain in place. No alternative physical
buffer, execution fallback or new control subinterval is added.

## Regression and experiment configuration

MATLAB R2026a Update 3. The focused regression covers configuration validation,
strict joint verification, certified nodes, native/interpreted geometry,
curved and smooth pose maps, controller behavior, certificate continuation,
circular cruise and road-boundary configuration: **170 unique cases pass**.
The initial run exposes two test-fixture problems: a stale 0.25 m residual and
a new positive-gap fixture exactly at an actuator boundary. After correcting
those fixtures, all 34 cases in the two affected classes pass. Contact and
penetration fail independent joint verification; a 0.01 m positive gap passes;
0.02 m position uncertainty still invalidates that nominal gap.

Factory-settings Code Analyzer checks 22 changed MATLAB files: no code
errors, 5 sparse-indexing performance advisories. In-scope diff checks pass.

The fresh campaign uses one computational thread; 300 holds of 0.1 s per case;
reference speed 8 m/s; exact declared affine ego plant and exact measurements;
zero residual and target jerk; 16 m sensing range; seed 20260912; no road
boundaries or estimator. Frame/search budgets are 30 s for this offline study.
These measurements do not establish 100 ms qualification, and no new strict
periodic campaign is claimed. Each hold is audited at eleven physical times.

| Curvature / m | Target | Minimum node body gap / m | Minimum sampled body gap / m | Sampled acceptance | Maximum frame / ms |
| --- | --- | ---: | ---: | --- | ---: |
| +0.00 | stationary | 0.000079880 | -0.225366441 | **FAIL** | 136.211 |
| +0.00 | oncoming | 0.000069076 | -0.059269626 | **FAIL** | 174.192 |
| +0.00 | crossing | 9.519691539 | 9.519691539 | pass | 14.289 |
| +0.01 | stationary | 0.100416490 | 0.095783436 | pass | 1630.565 |
| +0.01 | oncoming | 0.097425466 | 0.059140499 | pass | 422.546 |
| +0.01 | crossing | 0.141754413 | 0.033278233 | pass | 2163.147 |
| -0.01 | stationary | 0.096097552 | 0.094127564 | pass | 1338.775 |
| -0.01 | oncoming | 0.097309713 | 0.040426588 | pass | 417.551 |
| -0.01 | crossing | 0.114716243 | 0.014291569 | pass | 2647.715 |
| +0.02 | stationary | 0.202354615 | 0.201938623 | pass | 834.562 |
| +0.02 | oncoming | 0.197621587 | 0.177103045 | pass | 455.135 |
| +0.02 | crossing | 0.216070787 | 0.204440166 | pass | 4778.262 |
| -0.02 | stationary | 0.179132816 | 0.178996204 | pass | 869.013 |
| -0.02 | oncoming | 0.199870371 | 0.140658529 | pass | 506.755 |
| -0.02 | crossing | 0.177695809 | 0.121878767 | pass | 4350.826 |

All 4,500 issued commands pass the existing hard **node** certificate. This is
consistent with the two failed physical audits because that certificate does
not cover the interior of a held command. The twelve circular trials pass this
sampled study; their smallest observed gap is 0.014292 m. Finite samples alone
do not prove continuous-time noncollision in the passing cases.

## Independent verification of the two collisions

Reconstruct every straight-scenario endpoint from the declared generator and
saved held input: maximum discrepancy is exactly zero in this replay. Reproduce
the eleven-point audit, then inspect the worst hold at 1,001 evenly spaced times.
At each reported minimum, independently project both rectangles onto all four
edge-normal axes. All four projection intervals overlap, confirming a physical
intersection rather than a remaining buffer subtraction.

| Target | Worst audited hold / s | Gap at start / m | Gap at end / m | Dense minimum / m | Time of dense minimum / s |
| --- | --- | ---: | ---: | ---: | ---: |
| stationary | 1.5–1.6 | 0.000085606 | 0.000079880 | -0.233853857 | 1.5254 |
| oncoming | 3.7–3.8 | 0.000069076 | 0.001530043 | -0.059415820 | 3.7631 |

The finding is a gap between node safety and whole-hold safety. A fixed 0.25 m
buffer was not a proof of whole-hold safety, and is not restored here. Satisfying
the user's continuous physical noncollision requirement still needs a valid
inter-node safety mechanism within the chosen control framework.

## Reproduction and artifacts

```matlab
addpath('scripts');
buildAvoidanceGeometryKernel();
runJointSupportCertificateValidation( ...
    OutputDirectory=outputDirectory, SampleCount=300, ...
    Curvatures=[0,.01,-.01,.02,-.02]);
```

Raw trials, tests, build logs, dense audit driver/results, Code Analyzer output
and source patch are under
`/home/zai/.cache/collisionAvoidance/remove-clearance-20260919/`.
The companion JSON records all fifteen outcomes, the exact source-patch hash,
independent collision evidence and checks. Pre-existing untracked files,
dependencies, generated native artifacts and unrelated work are excluded from
the project commit. No estimator, Raspberry Pi or network work is included.
