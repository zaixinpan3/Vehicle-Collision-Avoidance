# Hold-node safety certificate: adoption, semantics and measured effect

September 17, 2026. Outcome: **completed as decided**. By project decision the
online controller now certifies safety at the 0.1 s sampling nodes of the
exact sampled affine plant: if the ego and target rectangles are separated at
every node, the plan is accepted. This is a deliberate weakening of the
previous whole-hold certificate; what is and is not claimed is stated in
`controller/NODE_SAMPLED_CERTIFICATE.md` and summarized here together with the
measured consequences.

## Decision and semantics

Before this change every collision, chart-domain and phase row was imposed on
all Bernstein coefficients of a polynomial enclosure of the exact flow inside
each held command, with a 1e-11 truncation remainder. At 8 m/s that enclosure
needed 25 to 26 coefficients per hold, so a 48-hold admission carried about
15,500 physical rows. The decision replaces "every instant of every hold" by
"every hold node":

- The declared plant is the exact sampled transition
  `x_k = A_k x_{k-1} + B_k u_k + c_k` from `expm(h*[A,B,c;0])`; node boxes are
  the interval hulls `|A_k| rho_{k-1}` plus the held process reserve and a
  floating-point allowance.
- Collision rows use the target's bounded reachable set at the node time;
  chart-domain and phase rows are imposed at the node; the modal terminal set,
  finite exit row, actuator and slew rows, soft CLF cone, lifted transcription,
  constraint generation, restoration search and independent certificate are
  unchanged. Recursive feasibility is inherited node by node.
- Nothing is claimed between two nodes. The vehicle moves about 0.8 m per
  hold at 8 m/s and a target can cross the ego path between two nodes at which
  both rectangles are clear. The test
  `collisionAvoidanceControllerTest/aCrossingBetweenTwoClearNodesIsOutsideTheNodeCertificate`
  records such an admitted crossing (a target moving at 200 m/s through the
  ego position 0.05 s after a node). The controller reports
  `metadata.wholeHoldCertificate = false` and
  `metadata.certificateSampling = "holdNodes"`.
- The scheduled terminal family keeps its stronger whole-hold phase synthesis,
  which remains valid at nodes. `stateUncertainty.heldInterval`,
  `ltvBicycleModel.fixedPredict` and `encounter.taylorOrder` remain for
  offline audits only. Eight theory documents carry a dated notice that their
  whole-hold statements now hold at nodes.

## Implementation

- `ltvBicycleModel.finitePredict` emits one node cell per held command (plan
  map, offset, box, `A_k`, `B_k`, `c_k`, node time); the held-interval kernel
  left the online path.
- `avoidanceSafetyGeometry.localCellRows` gained a one-point branch that
  evaluates the target set at the node time; multi-point cells keep the
  Bernstein path for offline audits. The three geometry kernels were
  regenerated and the native/interpreted parity test now includes one-point
  cells.
- `formulateAvoidanceProblem.localShift` reconstructs the node clock as
  `stage*h`.
- A latent defect in `avoidanceStageQp.build` was fixed: `find` returns row
  vectors for a cell with a single row, which the old 25-coefficient cells
  never produced.
- `runSmoothReferenceControllerValidation` records the node clearance and,
  separately, the 11-point inter-node sampled clearance of each trial.

## Final 300-hold validation

Protocol as in `SMOOTH_REFERENCE_VALIDATION_20260917.md`: MATLAB R2026a Update
3, one computational thread, 8 m/s, 0.1 s holds, exact ego and target states,
no road boundaries, 16 m perception range, 0.25 m clearance margin, diagnostic
5 s budget and independent strict 100 ms trials. "Previous" is the whole-hold
certificate after the implementation-only optimization of the same day.

| Path / scene | Previous max frame (ms) | Node max frame (ms) | Node median (ms) | Previous strict | Node strict |
| --- | ---: | ---: | ---: | --- | --- |
| sBend / cruise | 18.790 | 12.753 | 6.910 | 300/300 | 300/300, max 11.674 |
| transition / cruise | 11.943 | 9.494 | 6.933 | 300/300 | 300/300, max 9.661 |
| asymmetric / cruise | 11.463 | 7.782 | 6.691 | 300/300 | 300/300, max 7.418 |
| sBend / stationary | 176.055 | 67.125 | 7.499 | 0/300 | 300/300, max 64.587 |
| sBend / oncoming | 91.140 | 37.424 | 7.359 | 300/300 | 300/300, max 37.806 |

Every trial completes 300/300 holds with every command certified and every
strict trial qualifies. The stationary admission frame spends 28.1 ms in
formulation and 36.0 ms in four native solves (one restoration); the oncoming
admission spends 19.2 ms and 12.4 ms with two native solves. Only one frame of
the whole campaign exceeds 50 ms.

Clearance and maneuver, S-bend encounters:

| Quantity | Stationary, previous | Stationary, node | Oncoming, previous | Oncoming, node |
| --- | ---: | ---: | ---: | ---: |
| Node clearance above the 0.25 m margin (m) | 0.244 | 0.132 | 0.350 | 0.148 |
| 11-point sampled clearance above the margin (m) | 0.244 | 0.132 | 0.350 | 0.117 |
| Inter-node dip below the node clearance (m) | 0 | 0.0006 | 0 | 0.031 |
| Maximum lateral offset (m) | 2.636 | 2.446 | 2.963 | 2.548 |
| Largest station phase error (m) | 0.924 | 0.672 | 0.212 | 0.883 |

The node certificate is less conservative: it accepts plans that pass the
target with less clearance and smaller lateral excursions. Between nodes the
oncoming pass comes 3.1 cm closer than at the nodes, still 11.7 cm above the
margin in these deterministic scenes; the stationary pass dips 0.6 mm. These
dips are scene observations, not bounds.

## Validation

- Affected test classes after the change (node certificate, controller,
  admission search, estimator bounds, native geometry, prediction, actuator
  constraints, pose domain, scheduled reference, recursive closure): 138
  passed, 0 failed. New `tests/nodeCertificateTest.m` verifies that node states
  equal the exact sampled plant, that every node of an accepted stationary
  encounter plan is physically clear by the margin, that collision rows use
  node-time target sets, that carried nodes shift by one hold, and the
  metadata flags. Two whole-hold assertions were rewritten for node semantics
  and one interior-hold audit was renamed as a scene-specific physical check.
- Full working-tree suite after the change: 666 passed, 0 failed,
  0 incomplete (`runtests('tests')`, MATLAB R2026a Update 3).
- `git diff --check` and Code Analyzer on the changed sources: clean apart
  from pre-existing advisories.
- Historical analysis scripts that assume multi-coefficient cells
  (`analyzeSingleHoldInfeasibility`, `analyzeForceFreeStraightFeasibility`,
  `evaluateFialaResidualInclusion`) were not updated.

## Reproduction

```matlab
addpath('scripts');
runSmoothReferenceControllerValidation( ...
    OutputDirectory='/home/zai/.cache/collisionAvoidance/node-certificate-20260917/final', ...
    SampleCount=300, RunStrictTiming=true);
```

Raw MAT/JSON traces, the 60-hold comparison against the whole-hold baseline,
the focused test logs and the kernel regeneration log are in
`/home/zai/.cache/collisionAvoidance/node-certificate-20260917/`. The compact
summary is
[NODE_CERTIFICATE_ADOPTION_20260917.json](NODE_CERTIFICATE_ADOPTION_20260917.json).
Frame times are shared-workstation observations, not a worst-case bound.
