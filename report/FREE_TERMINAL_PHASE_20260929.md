# Free longitudinal phase for the terminal continuation

Date: September 29, 2026. Base revision:
`8f552d0df9d539793ff43adc1e0fd93ad6c189fd`.

## Problem and implemented change

The previous terminal implementation required the endpoint to lie inside a
small eight-dimensional neighborhood of a reference pose with preselected
longitudinal progress at that time. This was an implementation restriction,
not a user requirement. It was a neighborhood condition, not an exact
position equality, but it could force unnecessary progress recovery after
avoidance or braking.

The controller now optimizes a scalar reference phase `sigma`, measured in
meters. The terminal set is the union of certified neighborhoods over all
admissible phases. A straight reference can translate along its cruise
direction; a circular reference can rotate around its existing discrete RK4
orbit. There is no zero-phase target, phase penalty or explicit phase bound.
The same controller and first-feasible stopping rule are retained. Road
constraints remain absent; the physical collision buffer remains 6 mm.

The implementation retains the complete augmented terminal error, including
longitudinal error and previous input, relative to the **selected** reference.
It does not simply delete one row from the old contraction norm. Every phase
choice must pass all nonlinear input, slew, sampled collision, endpoint and
future target-separation checks. The selected phase is saved with the feasible
trajectory and preserved when shifting and appending its terminal feedback.

The global nominal RK4 bicycle model is equivariant to planar rigid motion.
For any fixed phase, the moving-frame terminal error dynamics therefore have
the same gain, contraction bound, defect and physical limits as the original
family. The old feasible trajectory with its old phase remains an available
next-step candidate. This preserves the structural recursive-feasibility
argument under its existing nominal successor and fixed-target assumptions.
Target separation is checked again at the actual endpoint time, so moving
the reference cannot silently move it through a target.

The detailed family definition, symmetry argument, shift proof and numerical
scope are in [the architecture](../controller/PCBF_CLF_ARCHITECTURE.md).
The extension adds one scalar to each conic subproblem and jointly adjusts
phase and final controls during endpoint correction. Existing eight-state
contraction enclosures are reused. Controller state format is now 54.

## Validation

All **255 selected MATLAB tests** in eight controller/scenario suites pass,
including 15 additional parameterized cases. Coverage includes closure on
straight and positive/negative curvature at phases -20, 0 and 20 m; analytic
phase derivatives against finite differences; phase-dependent future target
separation at matching absolute times; and a measured 5 m longitudinal delay
that chooses a new phase and then executes 24 retained steps with no new
conic solves. The latter crosses the original endpoint time.

All **12 Python audit tests** pass. Code Analyzer reports no findings in five
of the six checked MATLAB files. The solver retains 13 sparse-indexing
performance advisories in existing assembly operations; no syntax error is
reported. This is controller/scenario coverage, not the entire repository
test suite.

The unchanged deterministic campaign contains seven encounter types at 8 and
15 m/s. All fourteen uncontrolled, constant-speed given-path baselines collide,
independently verified. Exact observations and fixed target epochs are used;
no random draws, sensing errors or computation delay are injected. Each run
requests 160 holds of 50 ms. Prefix lengths are 8/16 holds, recovery length
3 s, maximum horizon 512 holds and soft search budget 5 s. Issued holds are
replayed with `ode45`, relative/absolute tolerances `1e-11/1e-12`, at 31 samples
per hold. Python independently reconstructs target motion and body geometry.
The ODE successor is fed into the next controller call. Its numerical
difference from the RK4 prediction triggers fresh nonlinear evaluation of
the stored continuation. The completed runs pass that evaluation without
new conic solves. This is experimental evidence under exact state feedback,
not an extension of the nominal recursive proof to arbitrary model mismatch.

| Speed (m/s) | Encounter | Outcome | Minimum body gap (m) | Selected phase (m) | Initialization (s) | Maximum running call (ms) |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| 8 | Head-on | Completed | 0.618080 | +1.465916 | 1.565222 | 30.425 |
| 8 | Accelerating head-on | Completed | 0.126496 | +1.967051 | 1.462847 | 24.985 |
| 8 | Braking lead | Search time limit | — | — | 5.010505 | — |
| 8 | Crossing | Completed; previously failed | 0.177078 | -6.452458 | 1.980545 | 24.669 |
| 8 | Turning crossing | Search time limit | — | — | 5.026607 | — |
| 8 | Curved head-on | Terminal admission rejected | — | — | 0.474957 | — |
| 8 | Curved crossing | Terminal admission rejected | — | — | 0.147124 | — |
| 15 | Head-on | Completed | 0.340780 | -1.630046 | 0.955237 | 27.599 |
| 15 | Accelerating head-on | Completed | 0.269103 | -1.035729 | 0.657455 | 26.555 |
| 15 | Braking lead | Search time limit | — | — | 5.016787 | — |
| 15 | Crossing | Search time limit | — | — | 5.006638 | — |
| 15 | Turning crossing | Search time limit | — | — | 5.008286 | — |
| 15 | Curved head-on | Terminal admission rejected | — | — | 0.409789 | — |
| 15 | Curved crossing | Terminal admission rejected | — | — | 0.143875 | — |

There are **5/14 completed runs and 9 initialization failures**, compared
with 4/14 and 10 in the preceding no-road campaign. Failed cases issue no
control and have no avoidance trajectory. Completed runs have zero predicted
hard residual and prefix slack, and positive dense-replay separation.

The newly successful 8 m/s crossing keeps its initial endpoint time at 3.4 s
but chooses a reference 6.452458 m behind the old reference at that time.
Its first command combines steering and braking. This directly demonstrates
removal of the prescribed progress target without extending the completion
horizon. Its closest body gap is 177.078 mm.

Across the 795 non-initial controller calls, the maximum is **30.425 ms**,
with no calls over 50 ms and no new conic solves: these calls execute retained
continuations. Successful initialization takes 0.657455--1.980545 s. These are
public controller-call measurements from one unprofiled campaign, excluding
the offline audit, not solver-only timings. They do not establish a runtime
bound for a fresh restoration search or a statistical speed comparison.

## Remaining restrictions and negative outcomes

Free phase removes a prescribed longitudinal position at the endpoint. It
does **not** remove the finite completion horizon or the requirement to reach
a certified local cruise neighborhood by its end. The initialization still
selects the endpoint index from a lane-feedback rollout. Terminal admission
is sufficient and conservative, not a complete characterization of viable
states.

Five lead/crossing cases still exhaust the search budget. Their observed
termination does not prove that the nonlinear problem is infeasible or
identify one exclusive remaining cause. Four curved cases still fail the
all-future orbit-separation test before optimization. Free phase does not
change the full circular orbit used by that test. Previously diagnosed
colliding initial guesses, local separating directions and conservative
terminal geometry remain relevant limitations; they were not redesigned in
this change.

The recursive argument concerns the declared nominal discrete RK4 map with
consistent state, clock, input memory, target and constraint context. Existing
floating-point enclosure limitations remain. Positive sampled replay gaps
are experimental results, not a general intersample or observation-error
safety guarantee. The earlier diagnostic intersample-collision mechanism is
not repaired by freeing terminal phase.

## Reproduction and artifacts

Compact results, individual test outcomes, analyzer findings, source/raw hash
manifests and reproduction scripts are in
[`FREE_TERMINAL_PHASE_20260929/`](FREE_TERMINAL_PHASE_20260929/).
Raw trajectory exports and logs remain under
`/home/zai/.cache/collisionAvoidance/free-terminal-phase-20260929/`.
No media or generated binary is added to the repository.

The executed commands were:

```bash
matlab -batch "r=runtests({'tests/terminalContinuationTest.m','tests/nonlinearPredictiveSafetyTest.m'}); t=table(r); writetable(t(:,{'Name','Passed','Failed','Incomplete','Duration'}),'/home/zai/.cache/collisionAvoidance/free-terminal-phase-20260929/initial-tests.csv'); assertSuccess(r)"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/free-terminal-phase-20260929/runValidation.m')"
python tests/auditJointPredictiveSafetyTest.py
python scripts/auditCollisionThreats.py /home/zai/.cache/collisionAvoidance/free-terminal-phase-20260929/campaign.json --output /home/zai/.cache/collisionAvoidance/free-terminal-phase-20260929/independent-audit.json
python /home/zai/.cache/collisionAvoidance/free-terminal-phase-20260929/analyze.py
```

The saved `runValidation.m.txt` can be copied back to the cache as
`runValidation.m`; the analysis driver is retained as `analyze.py.txt`.
Source hashes were recorded after the tests/campaign; execution sources were
unchanged between those runs and manifest capture.
