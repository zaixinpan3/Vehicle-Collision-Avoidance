# Restricted admission: temporal taper and coarser certificate exploration

Prepared September 21, 2026. Baseline parent revision:
`c622028432b8989494dea0b1e010002a269ace2b`.

The warmed circular-crossing admission frame falls from **57.9145 to
30.6155 ms median**, and from **60.332 to 33.421 ms maximum**, in a same-input
four-way ablation. Each variant has 22 measured calls. The default now finds
an independently certified scalar plan without the first full-plan SOCP.
Across 24 warmed closed-loop trials, all 14,400 requested frames return
certified commands, with a maximum of **35.101 ms** and no measured 50 ms
misses. Half the trials enforce the actual 50 ms frame deadline.

These are observed desktop timings, not a worst-case execution-time bound.
The full-plan recovery remains available when the restricted search fails.
The circular-crossing trajectory has a positive refined body gap, but the
known straight-road inter-node overlaps persist. Timing acceptance must not
be confused with continuous collision avoidance in every scenario.

## Algorithmic decision

The previous [component profile](LONGEST_CONTROLLER_FRAME_PROFILE_20260921.md)
identified base construction, unsuccessful scalar admission, initialization of
a full-plan candidate, joint-program assembly, and one conic solve. Reducing
only the number of geometric searches leaves the expensive unsuccessful
admission path intact. The
[earlier temporal-shape diagnosis](CIRCULAR_CROSSING_ADMISSION_DIAGNOSIS_20260921.md)
also supplied feasible counterexamples on other control lines. This motivates
changing the one searched line before optimizing its implementation further.

The controller still searches one scalar amplitude:

\[
U(\alpha)=U_0+\alpha d.
\]

The direction is the existing minimum-energy interpolation through the first,
middle, and last nominally violated stages of the selected target, with the
same terminal correction rows. Previously these three positions requested
equal transverse displacement. Curved predictions now use

\[
w_k=\rho+(1-\rho)\sin\left(\pi\frac{k-k_a}{\max(k_b-k_a,1)}\right),
\qquad \rho=0.8.
\]

The selected positions therefore request approximately `[0.8, 1, 0.8]`
times the amplitude instead of `[1, 1, 1]`. These weights specify three
interpolation targets, not a prescribed sinusoid at every prediction node.
Reduced displacement at the ends of the conflict interval can avoid early
chart-domain excursions while retaining displacement near the middle. The
conflict stages and transverse direction are computed from the current
prediction; neither a scenario name nor fixed crossing time enters the rule.

Exactly straight predictions keep the plateau. An initial uniform-taper
implementation made the straight oncoming regression fixture require a
full-plan solve; restoring its plateau removes that regression. Curved shapes
are selected once, without a bank of temporal candidates or multiple starts.

The geometric admission settings also change:

| Setting | Previous | Current |
| --- | ---: | ---: |
| Support directions | 32 | 16 |
| Amplitude cells | 16 | 8 |
| Cell/direction combinations per record | 512 | 128 |
| Scalar objective iterations | 32 | 32 |
| Crossing admission horizon | 96 holds / 4.8 s | 96 holds / 4.8 s |

For the same control line, fewer normals and wider yaw-enclosure cells can
exclude amplitudes that the finer certificate search could accept. This is
the deliberately reduced exploration requested for runtime improvement.
The tapered line itself is different from, rather than a subset of, the old
plateau line. Neither change relaxes the original independent acceptance
checks. Physical input/slew rows, pose domains, sampled collision conditions,
terminal conditions, CLF treatment, and continuation logic are unchanged.
The full control sequence remains free in the one recovery SOCP if needed.

Implementation: `config/collisionAvoidanceControllerConfig.m`,
`controller/solveHardCbfClf.m`, and the generated-frame adapter's prediction
packing in `scripts/standaloneControllerBenchmark.m`. The latter retains
`scheduleCurvature`, now needed by the shared numerical kernel. To restore
the previous search settings, set `temporalShoulderFraction=1`,
`normalCount=32`, and `amplitudeCells=16` under `cfg.admission`.

## Shape selection and sensitivity

A diagnostic sweep tested 144 combinations: six mirrored curvatures
`+/-0.008`, `+/-0.010`, and `+/-0.012` per meter; shoulder fractions
`1.0, 0.9, 0.8, 0.7`; 16, 24, or 32 normals; and 8 or 16 cells. This was
an offline selection experiment, not an online catalog.

| Shoulder fraction | Scalar certificates with 16 normals / 8 cells |
| --- | ---: |
| 1.0 | 0/6 |
| 0.9 | 4/6 |
| 0.8 | 6/6 |
| 0.7 | 0/6 |

The nonmonotonic result matters: arbitrarily reducing the shoulders is not
better. The selected value is an empirical design setting, not an optimal
shape theorem.

A separate paired sensitivity experiment covers 30 initial conditions for
each of the previous and current configurations. Besides the six tuning
curvatures, it includes held-out curvatures `+/-0.005` and `+/-0.015` per
meter, ego speeds 7 and 9 m/s, target speeds 3, 3.5, 4.5, and 5 m/s, target
stations 13 and 17 m, and transverse offsets 6.5 and 8.5 m, with left/right
mirrors. Baseline values are ego speed 8 m/s, target speed 4 m/s, station
15 m, and offset 7.5 m. Parameters vary individually, not as a Cartesian
product.

Both configurations certify all 30 cases: 28 active-target encounters and
two inactive-target returns. The previous configuration invokes full-plan
admission eight times; the current configuration invokes it zero times.
These finite cases show no observed admission loss, but do not establish
global completeness or guarantee that coarser searches never need recovery.
Sensitivity calls are not warmed timing evidence and are excluded from all
latency claims.

## Same-input warmed ablation

The saved straight-stationary and circular-crossing fixtures come from the
preceding profile. Each variant runs five excluded warmups followed by eleven
measured invocations, in each of two rounds. The second round reverses variant
order. The total is 176 measured calls and 80 excluded replay warmups; no measurement uses the MATLAB line
profiler. The legacy variant reproduces the saved complete decision within
`1e-9`; repeated decisions within every block are identical. All candidates
pass the unchanged original hard verifier.

MATLAB R2026a Update 3 (`26.1.0.3276743`), an AMD Ryzen 7 7800X3D, and one
MATLAB computational thread are used. No other experimental MATLAB workload
runs concurrently with timing. The desktop is not CPU-isolated.

| Circular-crossing variant | Median, ms | Maximum, ms | Native admission solves per call |
| --- | ---: | ---: | ---: |
| Plateau, 32 normals / 16 cells | 57.9145 | 60.332 | 1 |
| Taper, 32 normals / 16 cells | 33.3550 | 35.196 | 0 |
| Plateau, 16 normals / 8 cells | 51.5190 | 54.763 | 1 |
| **Taper, 16 normals / 8 cells** | **30.6155** | **33.421** | **0** |

The combined change reduces median latency by 47.1% and observed maximum
latency by 44.6%. Tapering produces the largest benefit by avoiding failed
section recovery, proposal selection, joint-program assembly, and native
admission solving. Coarser exploration provides an additional reduction.
This does not mean the common formulation or geometry construction has been
removed.

| Straight-stationary variant | Median, ms | Maximum, ms |
| --- | ---: | ---: |
| Plateau, 32 / 16 | 31.4720 | 38.623 |
| Taper setting, 32 / 16; straight gate retains plateau | 31.1820 | 32.142 |
| Plateau, 16 / 8 | 28.3230 | 29.704 |
| **Default, 16 / 8; straight gate retains plateau** | **28.3095** | **29.171** |

At equal grid resolution, the two straight variants execute the same
algorithm and produce the same decision. Their timing differences illustrate
run-to-run variability; they are not a taper speedup on straight roads.

For direct comparison with the previously discussed **39.169 ms formulation
bucket**, the following partitions use each variant's actual maximum frame
in this new, uninstrumented paired replay. They are not retroactive
subdivisions of the historical 64.033 ms frame.

| Disjoint bucket, ms | Previous settings: maximum frame | Current settings: maximum frame |
| --- | ---: | ---: |
| Input preparation | 0.892 | 0.763 |
| Formulation, including scalar admission and any joint assembly | 36.761 | 26.122 |
| Conic solve wrapper | 15.944 | 0.000 |
| Other frame work by subtraction | 6.735 | 6.536 |
| **Total** | **60.332** | **33.421** |

Across all 22 calls, formulation medians are 35.118 and 23.640 ms. These
component medians should not be summed into a fictitious individual frame.
The remaining formulation work is the main candidate for a later optimization;
this change does not claim to have accelerated each geometry kernel.

## Closed-loop results and independent footprint audit

Six cases combine curvature 0 or 0.01/m with stationary, oncoming, or
crossing target motion. Each case first runs two excluded 140-hold warmups.
Two measured repetitions each run a diagnostic 30 s frame budget and a
strict 50 ms frame budget, for 24 trials of 600 holds and 14,400 frames.
The 1,680 warmup holds are excluded. Fresh admission remains measured online
work even at simulation time zero.

All 24 trials complete, all issued plans pass the original hard verifier,
and no trial needs the full-plan initial recovery. Continuation still uses
the full-plan solver. Diagnostic and strict trials each contribute 7,200
frames. There are no observed 50 ms misses in either population, and the
largest frame is 35.101 ms in strict circular crossing. The diagnostic-budget
result alone would not prove strict-budget command acceptance; both were run.

| Measured population | Diagnostic calls | Median / maximum, ms | Strict calls | Median / maximum, ms |
| --- | ---: | ---: | ---: | ---: |
| All frames | 7,200 | 7.029 / 33.739 | 7,200 | 7.033 / 35.101 |
| Fresh active-target admission | 12 | 27.264 / 33.739 | 12 | 27.110 / 35.101 |
| Active continuation | 710 | 13.9035 / 31.834 | 710 | 13.836 / 32.340 |
| Cruise/no active encounter | 6,478 | 7.019 / 9.938 | 6,478 | 7.024 / 11.857 |

The overall median is dominated by cruise frames. The admission-only ablation
above is the relevant evidence for the change to the expensive initial frame.

The experiment uses exact sensing, the declared held affine vehicle model,
50 ms input holds, and seed 20260912. It does not include perception,
estimation, scheduling delays, or actuator latency. Optional state/slip
diagnostic bounds are not enforced; the circular-crossing diagnostic minimum
is -0.105295 while enforced pose-chart checks pass. This limits transfer to a
nonlinear physical vehicle. No new stochastic draws are needed for the
deterministic paired replays.

Independent audit reconstructs the held affine flow for the first diagnostic
repetition of all six cases, reproduces every saved state endpoint exactly
in this run, and reproduces the driver's 5 ms body-gap minimum within
`1e-9`. An independent separating-axis implementation agrees on overlap
signs. Near-contact holds and neighboring holds are then sampled every
0.1 ms, outside all timing regions.

| Road / target | Refined minimum signed body gap, m | Nominal cruise collision in sampled audit |
| --- | ---: | --- |
| Straight / stationary | -0.000618137 | Yes |
| Straight / oncoming | -0.005808359 | Yes |
| Straight / crossing | 9.519500954 | No |
| Circular / stationary | 0.097775407 | Yes |
| Circular / oncoming | 0.100862580 | Yes |
| **Circular / crossing** | **0.075145607** | **Yes** |

The circular crossing now has approximately 7.51 cm refined body clearance,
versus 6.31 cm in the preceding full-plan-admission experiment. This comparison
describes the resulting trajectories; the optimization objective was runtime.
All three curved trials avoid overlap in the sampled audit. Straight crossing
has no nominal threat, so its large gap is not evidence of a difficult
avoidance maneuver. The two straight overlaps are real counterexamples to
continuous collision avoidance despite positive node margins; their depth
changes slightly from the preceding experiment. The 0.1 ms audit is finite
sampling, not a continuous-time safety proof.

## Regression and generated-code validation

The complete MATLAB suite executed 763 cases with 762 passes and one failed
legacy assertion: the default straight 50 ms plan was required to have minimum
node clearance below 0.25 m. The coarser search selects a plan with
0.261010128 m minimum node clearance. That upper bound is not a safety
requirement and is incompatible with deliberately more conservative search.
The default test continues to check every accepted node for strictly positive
physical separation. A separate fine-search test retains the original
sub-0.25-m witness at both 50 and 100 ms, preserving the no-fixed-buffer
regression check without requiring every chosen plan to pass close to a target.

The revised `nodeCertificateTest` passes all 12 cases. Combining this affected
class rerun with the unchanged results from the complete suite gives
**765 passing current cases, zero failures and zero incomplete cases**.
The original full-suite result and its sole failure are retained separately;
this count is not presented as a second complete-suite execution. Production
sources did not change after the complete suite began. New taper tests cover
six mirrored curvatures, six invalid shoulder settings and the prepared-frame
adapter; existing full-plan failure/unsafe-result tests explicitly retain
legacy plateau settings so that the recovery branch remains exercised.

The generated `standaloneControllerFrameMex` is rebuilt from current sources.
Tapered admission returns status 1 and differs from the MATLAB complete
decision by at most `2.6645352591003757e-15`. The next optimized continuation
returns status 2 with zero decision difference. Both pass the original
independent verifier. This validates the prepared numerical kernel; it is
not a complete standalone-controller runtime benchmark.

Factory Code Analyzer checks ten touched MATLAB files. It reports one
pre-existing `find`/indexing suggestion in the solver and five growth
suggestions in the bounded, offline sensitivity-case constructor. No other
messages remain under factory settings. Python syntax, data-population and
residual assertions, report links, and Git whitespace checks are also checked.

## Reproduction and retained evidence

Run timing modes sequentially, with no competing benchmark process:

```matlab
addpath('scripts');
directory = '/home/zai/.cache/collisionAvoidance/restricted-admission-20260921';
runTaperedAdmissionValidation(directory, Mode='replay');
runTaperedAdmissionValidation(directory, Mode='sensitivity');
runTaperedAdmissionValidation(directory, Mode='campaign');
```

The replay fixtures default to the prior profile cache; use `FixtureDirectory`
to supply another saved copy. Campaign results are MAT/JSON artifacts under
`diagnostic/`, `strict/`, and excluded `warmup/`. Diagnostic sweep, independent
audit, generated-MEX verification, and logs are retained in the same external
cache. Generated binaries and solver dependencies are deliberately excluded
from the project commit. The machine-readable companion report contains
technical artifact SHA-256 values and the paired metrics.

```bash
python3 scripts/analyzeTaperedAdmissionValidation.py \
  /home/zai/.cache/collisionAvoidance/restricted-admission-20260921 \
  report/TAPERED_ADMISSION_OPTIMIZATION_20260921.json
```

The new defaults reduce exploration rather than proving universal admission.
Unsuccessful scalar admission can still invoke a costly full-plan SOCP, so
the observed 35.101 ms maximum must not be treated as a guaranteed bound.
Preserving the hard acceptance checks and the original horizon avoids trading
this measured speedup for a weaker sampled certificate.
