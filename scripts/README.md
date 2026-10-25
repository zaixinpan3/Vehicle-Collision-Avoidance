# Research entry points

## PCBF / CLF controller

Run the primary joint observer/controller campaign from the repository root:

```matlab
addpath('scripts');
runCollisionThreatValidation(OutputDirectory=string(tempname));
```

The defaults use the actual NRMM observer with synthetic noisy sensors in
25 m/s stopped-lead and hard-braking-lead encounters. Both encounters must
collide under unmodified cruise and immediate maximum straight braking.
Every collision-threat fixture must also have an independent nonlinear
bypass witness inside its declared scenario road (normally 9 m to either side
of the nominal path; a polygon union for junctions).
`bypassFeasibilityBaseline` tries prescribed offset maneuvers offline,
then replays the selected held inputs with `ode45`. It checks positive
rectangle clearance, complete vehicle footprints inside the road, model
state/input limits, encounter exit and return to nominal cruise. Its states
and inputs are exported in `baselineBypass` and never supplied to the online
planner, controller or observer. Failure to find a witness leaves the scenario
unqualified; it does not prove that the physical problem is impossible.
This is nominal truth-based existence evidence, not a robust-feasibility
certificate for the observer's full uncertainty set.
An unsupported observer initialization is recorded as `setupError`, and the
campaign continues with the next case. It never substitutes exact states.
Runs continue until sustained nominal cruise recovery or failure; a finite
`Frames` value is an explicit diagnostic cap, not a recovery deadline.

Every issued control is replayed
with tight `ode45` tolerances at 31 samples per hold on the nonlinear bicycle
model itself; no Simulink plant is used. This offline audit reports sampled
safety and timing; it does not authorize online execution.
Hard controller road constraints are disabled by default; the given path
supplies the cruise and recovery reference. `perceiveRoadBoundaries` clips the
scenario map to the current perception disk and sends only oriented visible
boundary chains to the planner. Only range-cut endpoints extend along their
tangents; the perception circle is not a wall. Visible junction openings remain
open, and unknown junctions cannot influence the current prediction. No full
map is supplied to the online planner. This ideal road sensor has range limits
but no occlusion or measurement noise. Exports retain the offline truth road,
each scan and complete-footprint replay margins. These are empirical checks,
not hard road constraints or a road-safety certificate.
The legacy `auditJointPredictiveSafety.py` checks a fixed 4-m half-width;
its road verdict does not describe this road-free campaign.

The collision-threat encounters are defined in `collisionThreatScenario`
(ten geometries; `givenPathCollisionBaseline` confirms that unmodified
cruise collides). Crossing targets decelerate to rest inside the road under
constant tangential acceleration and sideslip until stopping; they no longer
start outside the road or continue across its opposite edge. Head-on target
distance scales with closing speed to preserve a 1.6 s nominal meeting time.
The audit also checks the target's full footprint during the actual encounter;
leaving the road is an invalid scenario, not an optimization failure. These
scenario-road checks do not enable online road constraints or change the
existing 10 m ego-reference-point simulation stopping threshold.
`intersectionCrossing` uses the union of two perpendicular road rectangles;
the target travels along the crossing road. More general scenario maps use
`road.drivablePolygons`, a cell array of polygon vertex matrices whose union
defines the drivable region. A matrix may use NaN-separated rings to describe
holes such as traffic islands. The full map is available only to the synthetic
road sensor and offline qualification/audits. The nominal route remains an
independent cruise reference, not a definition of the road boundary.
`runPotentialFieldCampaign(pwd,tempname)` runs the seven named geometries at
8 and 15 m/s, also with the actual noisy-sensor NRMM
observer by default. Both campaign entry points accept a nonempty
`EstimatorConfiguration` to change sensor/observer settings;
`extendPotentialFieldRecovery` resumes unfinished runs using their saved settings.
`replayEstimatorValidity` audits the estimator's published bounds against the
recorded truth of such a campaign (it replays seed 20261003).
`synthesizeClfMatrices` writes `config/clfMatrices.json` and must be run before
an experiment at a new operating point.

`runNonlinearPredictiveSafetyValidation` accepts `Scenarios`, `Frames`,
`ControllerConfiguration`, `EstimatorConfiguration`, and `OutputFile` for
individual runs. Its empty-estimator mode and
`validateNonlinearPredictiveController` retain exact-state diagnostic fixtures;
these do not count toward the primary joint campaign results. Additional
fixtures `acceleratingTarget`, `acceleratingTurn`, and `brakingTarget` exercise
constant tangential acceleration with constant sideslip. The braking fixture
passes through zero signed velocity at 2 s. Each export records the initial
target state and parameters for independent reconstruction; the audit's
`--files` option selects individual JSON exports. Historical exports without
the revised target state require the audit source from their recorded commit.
The optimizer builds at most one local RTI model per controller call, then
solves the PCBF and CLF objectives on that same model. Planning occurs only
at encounter onset. Later frames shift the previously accepted inputs and
reroll them from the current estimate. Encounter preview may size the horizon
before optimization, but a failed solve does not increase its node count,
rebuild the model or call the planner. Terminal constraints remain enforced.
Completed optimized inputs are returned unchanged only after their nonlinear
rollout passes the sampled primary-constraint checks. The default 100 ms
computation budget can return an explicit failure. Independent replay checks
actual clearance and recovery. See
[the controller architecture](../controller/CONTROLLER_THEORY.md#runtime).
The campaign entry names containing `PotentialField` are retained for callers;
they run the current controller with second-order potential/circulation initialization and Fiala-model candidate rollouts.
Routine outputs can use `/tmp/` instead of a persistent validation directory.

`exploreFreeTimePredictiveControl` is an offline total-duration experiment.
Pass a controller model with a previous input trajectory, or the `snapshot`
from a joint-simulation failure MAT file. It compares the original two-stage
objectives with and without one scalar duration correction on the same
reference. The first `horizonSteps` holds keep the actual sample interval;
the remaining holds share the duration change uniformly. The final intervals
are at least one sample long. `DurationIncreaseFraction` bounds the additional
tail duration, and there is no time penalty in either objective.

The avoidance planner runs only at encounter onset. Later calls inherit the
previous optimized input sequence. Each invocation compares fixed and free
duration on the same reference and node count; a failed comparison is returned
without adding nodes or replanning. `attempts` contains that one comparison,
`completedPlannerCalls` counts completed planner invocations, and
`horizonExtensions` remains zero. `ExtendHorizonOnFailure`, `FreshReference`
and `ReplanOnFailure` are removed. A missing warm start during an active
encounter cannot activate the planner.

The probe differentiates the nonlinear rollout, target timestamps, per-hold
uncertainty tightening and terminal continuation. `MovingReference=true`
moves the state reference with duration while keeping reference controls
fixed; the alternative holds reference states fixed. Horizon collision
normals are frozen during differentiation. For stable row assembly, every
horizon target row is retained, even beyond perception exit, and redundant
row pruning is disabled. This is a deliberately more restrictive diagnostic
than the online first-exit formulation. Terminal row changes and inconsistent
finite-difference slopes reject that reference's duration model. A finer integration
mesh is shared by the fixed/free comparison; its cost is not an online timing
claim. Returned `fixed`/`free` results separate convex solver success from
nonlinear rollout diagnostics, including exact terminal membership. The
fixed/free pair uses the same reference; it is not a comparison of two
independent closed-loop controllers. Elapsed time includes research
formulations and audits, so it is not an online execution-time measurement.

`RefinementIterations>1` requests an offline SQP-style experiment and is not
one-iteration RTI. The probe neither issues inputs nor changes online defaults.
It retains the executable controller's current per-hold enclosure
approximation, without assuming measurement updates inside a longer hold.
It does not provide a robust joint observer prediction or a grid-shift proof.
In particular, a uniform tail grid generally does not remain uniform after
executing one fixed prefix hold and shifting the remaining policy. A variable
duration alone therefore establishes neither recursive feasibility nor safety.

`prepareCollisionAvoidanceController` warms MATLAB and the optimizer using a
discarded call. It does not build native controller libraries:
`buildPredictiveConicSolver` links the required Clarabel adapter, and
`buildControllerKernels` builds the optional MATLAB Coder kernels.

## Estimator and perception research

- `nrmmEstimatorControllerAdapter` provides the estimator scenario
  interface. It drives `onlineNrmmTrackingRuntime` at 80 Hz on bounded-noise sensors
  synthesized from the true path, acquires a target after 20 consecutive
  detections, publishes it only while detected, and passes the controller's
  held input to the estimator.
- `runObserverComparisonScenario` and `runObserverComparisonCampaign`
  compare the estimator with the reproduced
  Sharma et al. (2026) observer on prescribed kinematic motions (no vehicle
  dynamics, no controller).
- `runOnlineNrmmComplexManeuverScenario` and `runOnlineNrmmTrackingErrorBenchmark`
  are estimator research drivers.
- `buildNrmmObserverKernel` and `auditNrmmTruthEnclosure` support the estimator's
  independent native kernel and bound audits.
- `fitPerceivedRoadBoundaries`, `evaluatePerceivedRoadBoundarySafety` and
  `quadraticRoadBoundaryRectangleMargin` support offline curb experiments.
  `fitPerceivedRoadBoundaries` fits quadratics to curb polylines offset from
  the centerline, not to LiDAR curb detections. Fitted curb segments are not
  used by the current controller.

Retired controller modes, formal controller admission and native benchmark
scripts have been removed. On October 6, 2026 the research, audit and
validation scripts that no weekly record cites were removed as well (last
present at commit `1fca601`). Their dated reports remain historical results;
use Git history for the source associated with those reports. New reports
and numerical results belong in `report/`.
