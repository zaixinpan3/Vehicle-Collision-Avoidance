# Repository Guidelines

## Project Structure

This repository is a research-only codebase. Each directory owns one domain:

- `controller/` — the predictive controller as one entry
  (`collisionAvoidanceController.m`) over a small fixed set of modules: input
  parsing (`readControllerInputs.m`), the two-stage PCBF/CLF solve with its
  anchor rollout, constraints and trust estimate (`solvePredictiveControl.m`),
  the Fiala bicycle model, trim and CLF value (`nonlinearBicycleModel.m`,
  `modifiedFialaTire.m`), target prediction, uncertainty tubes, potential
  guidance and rectangle geometry (`predictiveSafetyGeometry.m`), and path
  coordinates (`laneGeometry.m`). `CONTROLLER_FILES.md` is the module map and a
  test limits the number of source files. Controller theory documents live
  beside the code.
- `config/` — all MATLAB configuration entry points and the precomputed CLF
  matrix table. Controller configuration merging and validation live with the
  defaults in `collisionAvoidanceControllerConfig.m`.
- `estimator/` — flat: the runtime (`onlineNrmmTrackingRuntime.m`), vector
  fields, certificates, gain synthesis, error bounds, the reproduced Sharma
  comparator (`sharma*.m`), and the theory documents
  (`OBSERVER_ISS_THEORY.md`, `MODEL_AIDED_ESTIMATION.md`). Only executable
  estimator and gain-design algorithms plus their theory documents belong
  here — no scenario drivers, benchmarks, adapters, reports, or generated
  artifacts.
- `perception/` — the Python YOLO+LiDAR target-position pipeline (the
  offline curb-detection experiment was removed on 2026-10-06).
- `scripts/` — closed-loop scenario drivers and campaigns, the
  estimator-controller adapter, estimator research scenarios and benchmarks,
  audits, kernel builds, synthetic-perception road fitting, and video
  rendering.
- `report/` — retained reports of important research results, subject to the
  Research Report Retention threshold below. Do not place reports in `scripts/`.

MATLAB unit tests live in `tests/` and use `matlab.unittest` class files ending
in `Test.m`. Do not reintroduce CARLA/ROS tooling, recorded datasets,
literature PDFs outside `reference/`, generated figures, local virtual
environments, or archived/historical implementations.

## Protected Algorithm Requirements

The user requires the following algorithmic structure. Performance work must
preserve it; a faster implementation that drops it is a different algorithm,
not an implementation of the adopted controller.

- Preserve the predictive control barrier function (PCBF) formulation, its
  primary safety objective and the conditions used to establish its decrease.
  A weighted feasibility-restoration penalty or ordinary collision-constrained
  NMPC is not an equivalent replacement for that formulation.
- Preserve terminal-set membership, a realizable terminal continuation law,
  and the invariance, information-consistency and horizon-shift conditions
  supporting recursive feasibility. Endpoint clearance, tail repetition,
  finite rollout checks and heuristic braking do not replace these conditions.
- Preserve the collision-distance dual-variable formulation. Do not replace
  it with clearance along a fixed finite list of body normals, bounding
  circles, smooth-max surrogates or other conservative geometry without an
  explicit user decision. Algebraic elimination must preserve equivalence.
  Dualizing the geometry does not by itself make the coupled nonlinear vehicle
  optimal-control problem globally convex.
- Preserve the target contract of constant tangential acceleration and
  sideslip, including its declared absorbing stop. A terminal construction
  valid only for braking targets must not become an admission restriction on
  the whole algorithm. Removing an unjustified terminal restriction requires
  repairing the terminal construction, not deleting the terminal requirement.
- Preserve one planner-generated initialization trajectory and the fixed
  cruise reference to which avoidance is intended to return. Physical-state
  hypotheses are uncertainty representations, not additional planner seeds.

Optimize numerical structure within these requirements: sparse derivatives,
caching, warm starts, condensing, integration work and solver implementation.
Changes to control parameterization, uncertainty coverage, horizon handling,
solver acceptance or discretization must explicitly account for their effect
on the theorem assumptions and shifted-plan representability. Sampling and
passing simulations cannot substitute for a recursive-feasibility proof.

Before changing controller formulation, trace the affected mathematical
constraint, implementation path and behavior validation. Do not rewrite the
mathematical theory to legitimize an implementation that has lost a required
algorithm component. Report implementation gaps and unverified guarantees
truthfully. Changing these protected requirements needs an explicit user
instruction; a request for faster execution is not such authorization.

## Literature Sources

The local Zotero library is a valuable source of research papers. Consider it
alongside other literature sources when looking for papers.

## Build and Test

Run all tests from the repository root:

```bash
matlab -batch "results = runtests('tests'); assertSuccess(results)"
```

Run the online estimator tests only:

```bash
matlab -batch "results = runtests('tests/onlineNrmmTrackingRuntimeTest.m'); assertSuccess(results)"
```

Run the synthetic research scenario:

```bash
matlab -batch "addpath('scripts'); runOnlineNrmmComplexManeuverScenario('Plot', false);"
```

## Coding Style

Use 4-space indentation. MATLAB functions, local helpers, variables, and config
fields use lower camel case (`referenceSpeed`, `localTestConfig`). Test methods
should read as behavior statements. Prefer compact structured data with explicit
units for vehicle, sensor, and timing values.

## Testing Guidelines

Add or update `matlab.unittest` coverage in `tests/` for controller, observer,
runtime, or target-tracking changes. Keep tests behavior-focused and avoid
asserting internal plumbing that is not part of the research algorithm.

## Commit and PR Notes

Use concise imperative commit subjects, such as `Simplify observer gain design`.
Pull requests should describe the algorithm behavior changed and list MATLAB
checks run.

## Documentation Language

Write every document in this repository and every editable archive record in
English, including Markdown, LaTeX, code comments, help headers, weekly
reports, and evidence descriptions. This applies regardless of the language
used in the request.

## Research Report Retention

Retain a standalone research report only when the result is important enough
to warrant long-term reference, or the user explicitly requests a report.
Examples include a material algorithm or model decision supported by analysis,
a significant implementation milestone with meaningful validation, a major
experimental comparison supporting a research claim, or a consequential
failure diagnosis that changes the research direction or implementation.

A concrete calculation, code change, test pass, or measurement is not by itself
sufficient reason to create or retain a report. Do not routinely preserve
ordinary explanations, exploratory derivations, small prototype trials,
one-off microbenchmarks, routine test output, intermediate debugging notes,
solver transcripts, or abandoned attempts as standalone reports or dated
report directories. Use the conversation or temporary files for such material;
retain useful implementation and tests in their normal source locations.
An unsuccessful attempt warrants a report only if its finding meets the same
importance threshold. When in doubt, do not create a standalone report.

Consolidate related important findings into an existing relevant report rather
than creating a new report for each turn or small subtask. Theory documents
contain the complete mathematical derivation of the adopted algorithm; they
are not logs of implementation work, exploratory proposals, or discussions.

Report retention and activity-ledger eligibility are separate decisions.
Qualifying work may need a concise ledger entry and a Git reference without
needing a new report, exported evidence bundle, Evidence ID, or standalone
commit-record document. Archival requirements must not be used to manufacture
low-value reports or to preserve every intermediate output. This retention
rule does not authorize rewriting append-only ledger history or silently
deleting evidence already admitted to the external archive.

## OPT Self-Employment Evidence Archiving

These rules apply only to substantive engineering or academic research progress
in this repository or its related research materials. Do not archive every
conversation, task, command, or file edit.

### Archiving Eligibility

Archive work only when it produces an identifiable implementation, technical
decision, research finding, or experimental result, for example:

- Actual code implementation, fixes, refactoring, or configuration changes that
  advance project behavior, correctness, performance, or maintainability.
- Algorithm, estimator, controller, or model design and improvements, including
  concrete derivations, assumptions, comparisons, and design decisions.
- Academic research such as literature analysis with project-relevant findings,
  theoretical analysis, or substantive manuscript, figure, and research-report
  development.
- Experiments, simulations, tests, and data processing or analysis that produce
  meaningful validation, measurements, findings, or reusable research outputs.

Do not trigger archiving for ordinary questions and explanations, status
updates, navigation, file searches, routine inspections or repeated checks,
general brainstorming or scheduling without a concrete technical result,
environment or tool administration, agent-instruction edits, cosmetic-only
changes, or archive housekeeping. A command run, file changed, or commit made
does not by itself establish substantive project progress. Research reading or
discussion qualifies when it yields an identifiable analysis, derivation,
finding, or design decision; a source-code change is not required.

Partial or failed engineering and research attempts qualify when they produce
meaningful implementation progress, diagnostic evidence, or a research finding.
Record their actual outcome without claiming success. For mixed tasks, archive
only the qualifying work and combine closely related steps into one record.

This eligibility gate governs all ledger, weekly/monthly report, Evidence Index,
and archive-related commit/push/completion requirements below. If no work
qualifies, skip that workflow and its final-response checklist. Explicitly
requested archive maintenance or required integrity corrections may still be
performed, but must not be presented as new engineering or research progress
or recursively generate records merely for maintaining records.

The purpose of the archive is to preserve truthful, contemporaneous, and
verifiable evidence of work. It is not permission to manufacture evidence,
backdate a record, invent a daily distribution of hours, or make a legal
conclusion about immigration compliance.

### Project Identity

- This repository is the **Collision Avoidance** project, filed in the archive
  as Project B.
- Ledger `project` field: `Pan Dynamics Collision Avoidance Research`.
- Ledger `record_id` prefix: `PDCA-YYYYMMDD-NNN`, numbered per activity date.
- The sibling Vehicle Localization repository is Project A and uses
  `Pan Dynamics Vehicle Localization Research` with the `PDVL-` prefix. Both
  projects share one external archive root, one append-only ledger, one
  Evidence Index, and one weekly-report series. Never create a parallel
  project-specific archive or weekly series.

### Fixed Context

- The SEVP Portal identifies `Zaixin Pan` as self-employed.
- `Pan Dynamics Research LLC` is the legal entity through which the documented
  internal research and development project is conducted.
- The documented self-employment start date is `2026-07-12`.
- The user has provided standing confirmation that each active project work
  week is exactly `20.0` actual hours. Project B was already active, and
  Project A joined it in week `2026-W36` (`08/30/2026`--`09/05/2026`), so both
  projects are active together from that week. The shared weekly report must
  therefore state `40.0` total hours from `2026-W36`: `20.0` for Project A and
  `20.0` for Project B as separate rows. Weeks before `2026-W36` had one
  active project and stand at `20.0`; do not restate them. This confirmation
  does not authorize an invented daily or per-operation distribution or a
  signature.
- The degree field is Mechanical Engineering.
- This is ordinary post-completion OPT, not a STEM OPT extension. Do not create
  Form I-983 records or apply STEM OPT employer rules unless the user's status
  actually changes and the user provides supporting documentation.
- Do not describe Pan Dynamics Research LLC work as occurring before
  `2026-07-12`, and do not silently change the SEVP employer identity. If an
  official record changes, record the new fact and its source without rewriting
  earlier records.

### Canonical Archive

Use `../Pan_Dynamics_OPT_Archive_Kit/` as the shared archive root.

- The archive root is deliberately outside this repository and is not under
  version control. Do not recreate the removed in-repository archive, do not
  initialize a repository in the external archive root, and do not require an
  archive commit. The user's `2026-09-03` decision is that archive records are
  written directly in place.
- Maintain
  `../Pan_Dynamics_OPT_Archive_Kit/00_Project_Activity_Ledger.jsonl` as the
  append-only, machine-readable activity ledger. Never rewrite or delete an
  existing line; correct an earlier record only by appending a new entry with
  a `correction_of` reference.
- Use the existing current weekly and monthly report templates. Do not create a
  competing report series or evidence index when a current one already exists.
- Treat
  `../Pan_Dynamics_OPT_Archive_Kit/05_Evidence_Response_Packet_Index.csv` as the
  canonical EV-number index unless a later archive status file expressly names
  a replacement.
- Keep operational code, configurations, and research data in their normal
  repository locations. Put evidentiary manifests, report exports, and archive
  copies under the archive root.
- Preserve original evidence. An export or copy must be identified as an export
  or copy and must not replace the original.
- All editable archive documents use standalone LaTeX (`.tex`) as the
  canonical source format. Do not create or update DOCX files.

### Git Commit and Commit-Record Archiving

The user has given standing instruction that completion of the archive workflow
includes committing the current in-scope project work and archiving the
resulting Git commit record. A substantive task is not complete until this
commit-and-record closure has been performed. The external archive takes no
commit of its own.

1. Inspect the final diff and run checks appropriate to the change. Commit the
   in-scope project artifacts produced by the task with a concise imperative
   subject. Stage deliberately: preserve unrelated user changes, and exclude
   external dependencies, vendored trees, generated binaries, and nested
   repositories such as `solver/` unless the user explicitly places them in
   scope.
2. Push the project commit and verify the remote result when an `origin` remote
   is configured. If no remote is configured, do not invent a push result;
   record the verified local commit and report that no remote was available.
3. Capture the full commit SHA, subject, committed file scope or diffstat, and
   the checks actually run and their outcomes.

If a substantive task produces no non-archive project artifact, do not invent
one merely to create a project commit; write only the truthful external archive
update. Do not create an archive commit or report its absence as a blocker.

Edits limited to `AGENTS.md`, `CLAUDE.md`, or other agent-operation instruction
files are administrative agent configuration, not project research activity.
Commit those instruction changes when requested, but do not add them to the OPT
activity ledger, weekly or monthly work records, or Evidence Index unless the
user explicitly identifies a separate underlying project operation that must be
archived.

### Record Qualifying Work

For each coherent unit of work that meets Archiving Eligibility, append one
JSON object on one line to the activity ledger. Group implementation,
investigation, and validation steps serving the same engineering or research
purpose into one record; do not create a record per command or minor edit.
Describe the concrete progress or finding and its supporting artifacts or
methods. Do not append a duplicate record at task completion for work already
recorded.

Keep the record proportional to the significance of the work. Existing source
files, tests, and the verified commit can supply the evidence; do not create a
separate technical report or evidence bundle merely to complete the ledger.
An empty `evidence_ids` list is appropriate when no new evidence item is
intentionally admitted. Apply Research Report Retention before creating any
additional persistent report or export.

Each ledger object must contain:

```json
{
  "record_id": "PDCA-YYYYMMDD-NNN",
  "recorded_at": "ISO-8601 timestamp with America/Chicago offset",
  "activity_date": "YYYY-MM-DD",
  "project": "Pan Dynamics Collision Avoidance Research",
  "operation_type": "literature|design|code|simulation|test|data|manuscript|archive",
  "summary": "specific factual description",
  "affected_files": ["repository-relative path"],
  "commands_or_methods": ["command or method actually used"],
  "outputs": ["repository-relative path"],
  "evidence_ids": ["EV-####"],
  "versions_or_hashes": ["Git commit, file version, or SHA-256 if available"],
  "outcome": "completed|partial|failed",
  "actual_user_hours": null,
  "notes": ""
}
```

- Record the technical operation as project activity without assigning or
  distinguishing authorship between Zaixin Pan and Codex.
- Never convert Codex runtime, terminal elapsed time, file timestamps, commit
  timestamps, or an estimated task duration into the user's OPT work hours.
- Leave per-operation `actual_user_hours` as `null` unless the user supplies
  operation-specific time. Do not divide or allocate the standing weekly
  `20.0` hours among individual operations.
- Earlier ledger entries are append-only. Correct an error with a new entry
  containing a `correction_of` reference; do not silently rewrite history.

### Technical Detail Required

Use concrete descriptions that another person can verify:

- Literature work: record the paper title, authors or DOI when available,
  sections or equations examined, and the resulting design decision or open
  question.
- Design work: record the estimator, controller, model, interface, assumptions,
  equations, and design decision affected.
- Code work: record the files and functions changed, implemented behavior, and
  full Git commit SHA and file hash when available. Follow the mandatory
  commit-and-record workflow above.
- Simulation or test work: record the script or test, configuration, scenario,
  material parameters and random seed, command actually run, result location,
  key metrics, and whether it passed, failed, or was inconclusive.
- Data work: record the input, processing method, output files, units, and
  metrics such as minimum distance, tracking error, collision rate, or
  computation time when applicable.
- Manuscript work: record the section, equation, figure, table, or claim
  changed and the source or PDF version produced.
- Failed and partial work must be recorded honestly. Do not describe an
  unexecuted simulation or test as completed.

### Weekly and Monthly Records

- Summarize related work as coherent research milestones. Do not expand the
  weekly report with a separate narrative for every small task, calculation,
  or test run, or rebuild it after every ledger entry. Consolidate updates at
  weekly reconciliation or when the user requests an interim report.
- Write every weekly report entirely in English as a standalone LaTeX source
  and compile a readable matching PDF with the same base name in the same
  weekly directory. Both files are required; PDF output is not optional.
- One weekly report covers the person, not one project. From `2026-W36`, place
  both Project A and Project B in the same report with separate `20.0`-hour
  rows and a `40.0`-hour total. Do not open another report series.
- Weekly and monthly records are internal activity summaries, not evidence
  items. Do not assign them Evidence IDs, add them to an evidence index or
  evidence manifest, or calculate, store, refresh, or list hashes for their
  source or PDF files. A weekly report must never cite its own hash as
  evidence.
- Weekly reports may cite Git commit SHAs and hashes of independent source
  files, datasets, tests, papers, videos, and other technical artifacts. These
  hashes identify the supporting artifacts, not the report and not the user's
  hours.
- Enter the standing confirmed total automatically: `20.0` per active project,
  which is `40.0` from `2026-W36` while both projects are active. Flexible
  daily distribution is acceptable; leave it unspecified unless the user
  supplies the actual distribution.
- If a weekly report is written after the covered week, label it explicitly as
  a retrospective late entry, state the true preparation date and factual
  reason, disclose the evidence limitations and pending facts, and never
  backdate it. For Week 07, the reason is that after moving, the desktop
  computer storing the weekly-report archive was temporarily unavailable for
  work; work continued from a laptop without that archive, preventing timely
  recordkeeping until archive access was restored. Week 07 is not a format
  exception: it requires the same English standalone `.tex` source and
  matching readable PDF as every other weekly report.
- Repository history and technical artifacts may support the nature and
  continuity of the work, but they do not independently prove how many hours
  the user worked.
- At weekly close, reconcile the weekly report with the activity ledger and
  actual artifacts. The standing `20.0`-hour confirmation does not need to be
  requested again.
- Build monthly summaries from the completed weekly records. Do not introduce
  new hours or activities during monthly aggregation.
- On `2026-07-27`, the user granted Codex standing authorization to
  electronically sign completed internal OPT archive records using
  `/s/ Zaixin Pan`. This authorization covers weekly work logs, monthly
  summaries, internal project reports, evidence manifests, and internal archive
  certifications generated from the factual activity ledger and actual
  artifacts.
- Use the actual America/Chicago calendar date on which Codex applies the
  signature. Never backdate a signature.
- Before applying the authorized signature, the document must contain no blank
  substantive fields, placeholders, contradictory hours, pending-certification
  language, unsupported activities, or unresolved Evidence IDs.
- Do not apply this standing signature authorization to government forms,
  submissions to SEVP, USCIS, a DSO, or another agency, tax filings, contracts,
  banking documents, immigration filings, intellectual-property assignments,
  notarized documents, sworn statements, statements made under penalty of
  perjury, or third-party documents. Those require separate document-specific
  authorization.
- The user may revoke or narrow this standing authorization at any time.

### Evidence IDs and Integrity

- Assign an Evidence ID only when an actual file or defined evidence bundle is
  intentionally admitted into the archive. Minor edits may remain referenced
  by path in the ledger.
- Evidence admission is not automatic for each qualifying task or commit.
  Create a new bundle only for important results that warrant independent
  preservation or when explicitly requested. Prefer references to existing
  artifacts over duplicate exports of routine work.
- Before using an EV number, consult the canonical index. Never guess or reuse
  an Evidence ID.
- Every EV entry must identify the file name, evidence date, archive location,
  concise description, project, applicable week, whether it is an original or
  export copy, and a Git hash, version, or SHA-256 when applicable.
- Weekly and monthly reports are excluded from EV admission and hashing. If a
  legacy index still contains a weekly-report row, its report-hash field is not
  applicable; do not refresh, cite, or propagate the historical hash. Correct
  current indexes without rewriting append-only ledger history.
- An EV number must resolve to an existing file. A blank template is not
  evidence.
- Do not alter file metadata or Git history to make evidence appear older.
- Do not fabricate customers, revenue, bank activity, subscriptions, expenses,
  purchases, licenses, contracts, proposals, invoices, correspondence,
  simulation runs, measurements, or third-party validation.
- If no financial transaction or applicable license exists, record no such
  evidence. A factual license-applicability check may cite an official source,
  but it must not be presented as an issued license.

### Privacy and Task Completion

- Never archive passwords, API keys, authentication cookies, private keys, or
  unrelated personal data. Use a redacted reference when an evidentiary record
  contains sensitive identifiers.
- Do not copy EAD, passport, I-20, SEVIS, tax, or banking documents into the
  repository unless the user explicitly requests the copy and identifies the
  intended protected archive location.
- Complete the closure in this order: project commit, verified push when a
  remote is configured, then the external archival record of that commit. No
  archive-record commit exists or is required. Do not leave substantive
  in-scope changes uncommitted unless a concrete blocker is recorded and
  reported.
- Report facts only. Do not state that the archive guarantees OPT compliance or
  that DHS, USCIS, SEVP, a DSO, or a court will accept a particular document.

## Canonical Archive Document Format

All editable documents in `../Pan_Dynamics_OPT_Archive_Kit/` use standalone
LaTeX (`.tex`) as the canonical source format. Do not create or update DOCX
files. Keep each canonical `.tex` file at the corresponding external archive
location and edit it directly. Every weekly report must also have a matching,
readable derived PDF in the same weekly directory. Preserve historical ledger
references to removed DOCX artifacts, and record format migrations or renames
through append-only corrections rather than rewriting history.
