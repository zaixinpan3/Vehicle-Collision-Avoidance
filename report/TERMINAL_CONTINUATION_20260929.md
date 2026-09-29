# Time-indexed terminal continuation

Prepared September 29, 2026. Implementation and deterministic numerical checks.

## Implemented formulation

Replaced the fixed lane terminal condition with a fixed-absolute-endpoint backward-reachable tube represented by a hard, zero-slack completion trajectory. Its state has six ego coordinates and the previous two inputs. The endpoint family has a separately constructed nonlinear RK4 contraction bound and an all-future geometric separation condition. The completion shortens until the endpoint; thereafter the endpoint policy supplies each appended control.

Target predictions now share a fixed epoch with absolute node/midpoint indexing. State format 52 retains the continuation, achieved prefix slacks and endpoint family. Improved solutions must satisfy the defining nonlinear constraints and the shifted achieved-slack bound. Both zero and positive prefix slack remain executable; positive slack is recovery, not a collision-free claim. A failed improvement solve preserves the retained witness under the nominal successor assumption.

The CLF remains soft and secondary. SCvx uses Li-style signed support duals and the same 2-norm endpoint cone in optimization and evaluation. Numerical completion restoration and a short terminal shooting correction help construct feasible candidates. Finite suboptimal secondary conic points are evaluated even without an optimal exit flag; actual nonlinear feasibility and achieved slack decide whether a witness is retained. Conic status alone does not authorize execution.

Removed the old fixed terminal ellipsoid admission rows, inscribed 1-norm polytope, input-radius band, clipped feedback append and observation-driven target reinitialization. Four profiling/capture helpers tied to the retired version-51 state and LP/QP internals were deleted; their reports and Git history remain. Nine focused MATLAB controller/configuration sources remain, including the new endpoint-construction module. No native or MPFR dependency was introduced.

## Endpoint construction

The bound is for the global-coordinate RK4 prediction map, with held inputs and augmented input memory. The moving reference uses the sampled pose increment, including the discrete circle geometry. The trim defect is bounded rather than assumed zero. The sufficient condition is gamma*r + d < r, with admissible input, slew, velocity and internal tire-domain bounds over the neighborhood. Cached Jacobian enclosures use outward-padded double arithmetic, Taylor remainders and an inverse-residual bound. They are construction work, not an online trajectory-verification pipeline.

| Speed (m/s) | Curvature (1/m) | Radius in scaled coordinates | Gamma | Strict contraction room |
| ---: | ---: | ---: | ---: | ---: |
| 8 | 0 | 0.000122070312 | 0.9973806500 | 3.19735e-07 |
| 8 | -0.005 | 0.000122070312 | 0.9986617598 | 1.63352e-07 |
| 8 | 0.005 | 0.000122070312 | 0.9986617598 | 1.63352e-07 |
| 15 | 0 | 0.0009765625 | 0.9990303433 | 9.46919e-07 |
| 15 | -0.005 | 0.00048828125 | 0.9976917063 | 1.12709e-06 |
| 15 | 0.005 | 0.00048828125 | 0.9976917063 | 1.12709e-06 |

The seed is deliberately sufficient and can be small. The MPC terminal region is the backward-reachable tube to this seed, not the seed neighborhood itself. A failure to find this seed or its completion does not prove that every alternative terminal family is infeasible.

## Checks

- Selected regression suites: **234/234 passed**. The 75 terminal/controller checks were repeated after the final candidate-selection change: **75/75 passed**.
- Python collision/target/audit behavior checks: **7 passed**. The audit requires exact reported zero safety slack and zero hard residual, and recognizes retained witnesses without requiring a new solve.
- Factory MATLAB Code Analyzer: ten changed MATLAB sources/scripts/tests examined; only twelve sparse-indexing performance notices in the SCvx assembler remain.
- Endpoint closure checks cover both curvature signs, actuator memory and finite slew. Continuation checks cross the original endpoint with optimization disabled and exercise positive-slack shifts, fixed target epochs, time gaps, changed input memory and infeasible completions.

### Nominal execution beyond the original endpoint

Each run uses 180 holds at 50 ms, 8 m/s, an eight-stage prefix, zero added collision buffer and 4.8 x 1.9 m rectangles. The first call has a 60-second soft search limit; subsequent calls disable improvement, forcing use of the retained witness. Ego successors use the same nominal RK4 map. Independent ODE45 replays sample 31 points per hold (relative tolerance 1e-11, absolute 1e-12); these replay samples do not set the next nominal state. No random draws.

| Scenario | Holds | Original endpoint M | Last absolute sample | Retained holds | Max hard / safety slack | Min replay clearance (m) |
| --- | ---: | ---: | ---: | ---: | --- | ---: |
| recovery | 180/180 | 89 | 179 | 179 | 0 / 0 | No target |
| oncoming | 180/180 | 68 | 179 | 179 | 0 / 0 | 0.011769657 |
| circular | 180/180 | 68 | 179 | 179 | 0 / 0 | No target |

The independent Python replay audits pass in **10/10** cases. All recorded shifted-budget violations: 0. These finite experiments check implementation behavior; the indefinite argument comes from the endpoint successor relation and the nominal shifted-witness hypotheses.

### ODE successor restoration and runtime limitations

Fresh four-hold runs use ODE45 successors, the default five-second soft solve budget, and seven fixtures: recovery, oncoming, circular road, turning target, accelerating target, accelerating turn and braking target. A changed ego successor disables the old shifted-witness claim and requires renewed feasibility. The known target epoch is unchanged. These 0.2-second smoke runs do not establish full-encounter safety.

| Scenario | Holds | Largest call (s) | Outcome |
| --- | ---: | ---: | --- |
| recovery | 4/4 | 6.046 | Completed |
| oncoming | 4/4 | 6.405 | Completed |
| circular | 4/4 | 1.449 | Completed |
| turningTarget | 4/4 | 5.460 | Completed |
| acceleratingTarget | 4/4 | 6.788 | Completed |
| acceleratingTurn | 4/4 | 5.445 | Completed |
| brakingTarget | 4/4 | 9.919 | Completed |

A separate two-hold 15 m/s default-budget smoke run completed recovery and circular-road cases but **failed to initialize the oncoming case within five seconds**; no command was returned for that case. A diagnostic 60-second oncoming solve before the final suboptimal-iterate eligibility change found a feasible zero-slack witness at outer iteration 4 and retained it through the time limit (63.1454 s total, 26 conic calls). Its secondary conic solves repeatedly reported exit flag -7; this motivated evaluating finite suboptimal secondary points rather than automatically discarding them. The final five-second smoke run still timed out during oncoming initialization. This remains a solver-performance limitation.

An earlier braking-target stress solve outlasted the outer iteration deadline and was interrupted. The configured soft time budget is now also passed to coneprog through MaxTime; all seven final short ODE runs complete. This is still a soft limit: active factorization and terminal correction can overrun it. The present conic implementation does **not** establish real-time 50 ms initialization or reoptimization. Host load was uncontrolled and some diagnostic MATLAB runs overlapped; these timings are not an isolated performance benchmark.

## Guarantee scope

Recursive feasibility is conditional on the declared nominal transition, consistent absolute target/road/model predictions, valid input memory and a feasible nonlinear witness. The hard completion and endpoint invariance close the indefinite shift. There is no global nonconvex optimality claim, continuous-intersample collision guarantee, disturbance-robust guarantee or unconditional lane-convergence claim. Ordinary floating-point trajectory/geometry evaluation is not a formal real-arithmetic execution certificate; numerical boundary decisions and plant/integration errors need additional error analysis for stronger guarantees. No positive solver residual tolerance is treated as exact hard feasibility.

## Reproduction and artifacts

Run the suites listed in `scripts/validateNonlinearPredictiveController.m`; the two focused terminal/controller suites and `python -m unittest discover -s tests -p auditJointPredictiveSafetyTest.py` exercise the changed behavior. The full validation driver still includes longer ODE scenarios; that entire driver was not run for this task.

```matlab
addpath('scripts');
runNonlinearPredictiveSafetyValidation(Scenarios=["recovery","oncoming","circular"], ...
    Frames=180, StateTransition="nominalRk4", ImproveAfterInitialization=false, ...
    ControllerConfiguration=struct('referenceSpeed',8,'controller',struct('horizonSteps',8), ...
        'solver',struct('timeLimitSeconds',60)), OutputFile='nominal.json');
```

Compact results, endpoint bounds, analyzer findings and source hashes are in `TERMINAL_CONTINUATION_20260929/`. Full traces and preliminary diagnostic logs are retained in `/home/zai/.cache/collisionAvoidance/terminal-continuation-20260929/`. JSON encodes nonfinite values as null. Preliminary pre-fix audits are retained separately and are not the final result table.
