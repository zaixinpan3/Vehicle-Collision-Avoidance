# Synchronize the manuscript with the current algorithms

September 19, 2026. Manuscript revision; no controller or estimator code change.

The manuscript now describes the direct body-velocity observer, parallel yaw
output, two-stage ISS comparison, free-metric target synthesis and the current
joint trajectory/support controller. Its existing section order, notation,
lemma/proof presentation, introduction literature discussion and perception
section are retained. The latter two text spans are byte-identical to the input.

## Source scope

The initial repository HEAD was
`32617fc75a25dce6b0b8e54e4d6744f209fce9fe`. The manuscript had last been updated
with the September 6 tire-model revision. The current estimator is the committed
free-metric design introduced in `a6fd787`; the controller working tree already
contained a separate, uncommitted format-39 single-restriction change. This
revision follows that inspected working-tree algorithm, without committing or
changing the concurrent controller work.

Controller, configuration, estimator, test, script and report sources were
copied to a local audit snapshot at `2026-09-19T10:49:10-05:00`. The only later
change found in the inspected method sources was relocation of two comments in
`solveHardCbfClf.m`; the executable logic was unchanged. Full source hashes are
in the companion JSON. The report does not claim that the paper commit itself
contains the independently developed controller revision.

## Mathematical changes

- Abstract, introduction and system overview: align contributions and scope
  with the direct velocity core, sampled controller and unsupported physical
  road-boundary branch. Correct front-wheel force rotation and nonlinear slip
  expressions while retaining the modified Fiala law.
- Estimator design: derive the direct forward-cone velocity measurement and
  combined gyro-error column; keep yaw and inertial position outside the core
  cascade; use a position-normalized target metric and a two-by-two Metzler
  comparison. Replace the fixed Lyapunov identity by the free-metric Schur LMI.
- Controller design: use the force-balanced steady turn and immutable exact
  held affine generators; replace terminal rest by the modal cruise set,
  measurement-error support, terminal-entry slew and finite sensing exit.
  Present the joint-angle support residual, touching convex majorant and SOC
  support epigraph, one convex restriction and independent hard acceptance.
- Replace the instantaneous continuous CLF row by the sampled norm cone and
  its slack-dependent dissipation allowance. Update the objective to penalize
  full-horizon tracking, trim-relative inputs, CLF slack and the small proximal
  term. State conditional continuation using the complete shifted certificate.
- Results, discussion and conclusion: replace obsolete numerical claims with
  version-identified estimator and controller records. Explicitly distinguish
  the preceding multi-solve admission results from the current single
  restriction, and node certification from inter-node physical collision.

The added OBCA citation was verified against the authors' arXiv v3 metadata
and full text, Section 4: Xiaojing Zhang, Alexander Liniger and Francesco
Borrelli, *Optimization-Based Collision Avoidance*,
[arXiv:1711.03449v3](https://arxiv.org/abs/1711.03449v3).
It supports the exact convex-body certificate viewpoint; the paper does not
attribute this project's angular majorant or its admission properties to OBCA.

## Numerical evidence and checks

The gain comparison comes from `NRMM_NOISE_REDUCTION_TRIALS_20260916.csv`
and its retained design export. Independent Python recomputation confirms all
eight paired RMSE rows from 62 trial records. Nominal velocity and acceleration
RMSE reductions are 74.81655% and 89.89902%. The current common-gyro coefficient
is 16.5416323, the velocity forcing bound is 0.200662914, and the target
position/velocity/acceleration ultimate radii are 2.2071633 m, 12.7289506 m/s
and 30.2633254 m/s². Substitution in the original free-metric dissipation
inequality gives minimum eigenvalue `1.000136034e-6`; copositive weights and
physical component extraction agree with the exported design. The units of
the velocity Lipschitz coefficient are corrected to s⁻².

The separate multi-scenario estimator record is
`NRMM_ESTIMATOR_SCENARIOS_20260917.md/.csv`. Its 101 trials and 55,701
published samples are retained with the noise-contract violation and digital
certificate qualifications. No new trajectories are generated here.

`NO_CLEARANCE_BUFFER_VALIDATION_20260919.json` supplies the fifteen-case
controller record and independent dense collision audits: all 4,500 commands
pass node verification, but only thirteen cases pass sampled noncollision.
The two failing holds have positive endpoints and negative dense gaps, and all
four independently projected rectangle axes overlap. These are not successful
whole-hold avoidance trials.

`CONTROLLER_RUNTIME_RETEST_20260919.json`, tested at
`0bac5eecb7fcdcd83c3347e5cea1e11a396aded3` (format 38), supplies the 9,000-frame
timing record. Category counts reconcile exactly. All 986 active continuations
use one solve; 27 of 32 admissions exceed 100 ms. The manuscript expressly
does not assign these admission timings or successful admissions to format 39.
The counterexample for a globally feasible square escape with an empty local
restriction is checked against `jointSupportCertificateTest.m`; that class is
not rerun for this manuscript task.

Validation performed:

1. ARS block-manifest patches apply with hash preconditions in two scoped
   passes. They preserve 124/149 and 98/151 untouched blocks, respectively;
   neither pass changes the top-level section structure. Formatting and
   numerical corrections follow as local patches.
2. `audit.py` recomputes the metric certificate and paired values, reconciles
   recorded collision/timing outcomes, checks balanced LaTeX environments,
   verifies 116 unique labels and 28 resolved citation keys, and verifies the
   two unchanged prose spans.
3. `latexmk -pdf -interaction=nonstopmode -halt-on-error manuscript.tex`
   produces a 14-page PDF with no undefined references, unresolved citations
   or overfull boxes. The revised estimator, controller and result pages are
   rendered with `pdftoppm` and visually inspected.
4. In-scope `git diff --check` passes. No new MATLAB test run, simulation,
   solver timing measurement or full bibliography audit is claimed.

The working audit is under
`/home/zai/.cache/collisionAvoidance/manuscript-sync-20260919/`: source snapshot,
manifest, two patch/apply records, numerical audit, build log and page previews.
The compiled manuscript is a local derived output. Existing untracked figure
assets are used as supplied and are not newly admitted to Git by this task.

Only the manuscript, bibliography, paper README and this revision's report/JSON
are committed. Concurrent controller/configuration/tests, root instruction and
README files, existing figure assets, dependencies, native binaries and other
untracked material are excluded. Author/submission declarations remain to be
completed, and the revised draft still requires scientific review before
submission.
