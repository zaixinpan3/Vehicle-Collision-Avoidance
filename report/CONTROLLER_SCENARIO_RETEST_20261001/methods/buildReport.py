"""Render the independent retest report from its recorded results."""
from pathlib import Path
import json,sys,re
raw=Path(sys.argv[1]);root=Path(sys.argv[2]);s=json.loads((raw/'summary.json').read_text())
assert s['caseCount']==14
source=json.loads((raw/'source-manifest.json').read_text());w=s['worstFrame'];run=s['runtime']
recovered=[r for r in s['cases'] if r['recovered']]
no_recovery=[r for r in s['cases'] if not r['recovered']]
def esc(t):
    return str(t).replace('_',r'\_')
def num(v,d=3):return '---' if v is None else f'{v:.{d}f}'
rows=[]
for c in s['cases']:
    rows.append(f"{c['speed']} & {esc(c['scenario'])} & {c['duration']:.2f} & {c['minimumGap']:.4f} & {num(c['recoveryConfirmation'],2)} & {1000*c['runtime']['median']:.1f} & {1000*c['runtime']['maximum']:.1f}"+r' \\')
parts=[]
for key,label in [('initializationSeconds','Initialization'),('formulationSeconds','Problem formulation'),('pcbfSlackSeconds','PCBF solve'),('clfSlackSeconds','CLF solve'),('anchorDeviationSeconds','Anchor-deviation solve'),('otherSeconds','Other controller work')]:
    parts.append(f"{label} & {1000*w[key]:.3f} & {100*w[key]/w['controllerSeconds']:.1f}"+r' \\')
text=r'''\documentclass[11pt]{article}
\usepackage[margin=0.85in]{geometry}
\usepackage{amsmath,booktabs,xurl,hyperref}
\usepackage[T1]{fontenc}
\setlength{\emergencystretch}{3em}
\title{Independent Scenario Retest of the Current Collision-Avoidance Controller}
\author{Pan Dynamics Collision Avoidance Research}
\date{October 1, 2026}
\begin{document}
\maketitle
\section{Outcome and scope}
This is a fresh rerun of the committed controller, without algorithm or
configuration edits. Tested commit: \path{COMMIT}.
All fourteen ideal given-path cruise baselines collide from initially separated
states. The current controller produces COLLISIONS collision cases and
INTERRUPTS interrupted cases in HOLDS executed holds. Nominal recovery is
confirmed in RECOVERED of fourteen cases. Recovered cases enter and hold the
tolerances by TMIN--TMAX~s, including a five-second dwell.

\textbf{Design discrepancy.} During this retest the user reiterated that the
controller must always use one CLF. The tested implementation does not meet
that requirement: \path{controller/solvePredictiveControl.m}, lines 311--318,
uses \texttt{laneQuadratic} while collision rows are active and substitutes
\texttt{recoveryCostToGo} when \texttt{departure==0}. It also changes the anchor
and adds a third anchor-deviation solve in the latter branch. One active CLF
constraint per frame is not the same as one unchanged CLF definition. These
results characterize the committed branch-dependent implementation; they do
not validate the requested unified-CLF design. No correction of that design
was included in this test-only task.

\section{Protocol}
Seven deterministic threat fixtures are run sequentially at 8 and 15~m/s,
using 50-ms holds and primary horizons of 8 and 16 steps respectively, with
60 completion steps. The default optimization-node safety buffer is 0.05~m;
road boundaries are disabled. Target motion is known and observations are
exact, without noise or delay. Plant advancement uses independent ODE45 with
relative/absolute tolerances $10^{-11}/10^{-12}$. Rectangle gaps are measured
at 31 points per hold (1.667~ms spacing). This is sampled simulation evidence,
not a proof of continuous-time clearance or delayed real-time execution.

Each case runs until recovery or an 80-s observation cap. Recovery requires
five continuous seconds of sampled errors within 0.1~m lateral error,
$1^\circ$ heading error, 0.1~m/s longitudinal speed error, 0.05~m/s lateral
velocity error and 0.01~rad/s yaw-rate error, beginning no earlier than 8~s.
Completing 80~s alone is not recovery. Positive measured body clearance is the
collision criterion; the 5-cm node buffer is not imposed as a separate
intersample pass requirement.

Runtime is wall time around the complete controller call, including input
handling, initialization, model/constraint construction and every optimization
stage. Offline ODE replay, geometric audit, serialization and two preparatory
warm-ups per speed are excluded. Scenario first calls are included. MATLAB
R2026a ran with \texttt{-singleCompThread} on an AMD Ryzen 7 7800X3D. Another
project computation was present on the machine; these are measured runtimes
under shared load, not isolated-hardware benchmarks. Runtime was not inserted
as a plant actuation delay.

\section{Per-scenario results}
\begin{center}\small
\begin{tabular}{rlrrrrr}
\toprule
Speed & Scenario & End (s) & Min gap (m) & Recovery (s) & Median (ms) & Max (ms)\\
\midrule
ROWS
\bottomrule
\end{tabular}
\end{center}
A dash means no confirmed recovery by the observation limit. Every recovery
time includes the full five-second dwell.

NONRECOVERY

\section{Controller runtime and numerical outcomes}
Across all HOLDS holds, the median full-call runtime is MEDIAN~ms,
the 95th percentile is P95~ms and the maximum is MAXIMUM~ms.
OVER holds (OVERPERCENT\%) exceed the user's 100-ms target. Thus the
current measured implementation does not meet that target.
The percentile uses linear interpolation between sorted observations.

The slowest frame is SCENARIO at SPEED~m/s and simulation time TIME~s.
Its recorded breakdown is:
\begin{center}
\begin{tabular}{lrr}
\toprule
Component & Time (ms) & Share (\%)\\
\midrule
PARTS
\bottomrule
\end{tabular}
\end{center}
The slowest frame has solver exit flags FLAGS, in stage order.
There are RESTARTS flow reinitializations and NONCONVERGED frames with at
least one nonpositive solver exit flag. Finite returned vectors are executed
under the current controller contract; a nonpositive flag is not counted as
an empty-result interruption. Per-stage exit flags and timings are retained
in the data bundle. This retest adds no online acceptance checks.

On the cost-to-go branch, ZERO of FREE frames have CLF slack within the
scaled numerical tolerance $10^{-5}\max(1,V)$; DECREASE of PAIRS consecutive
same-branch ODE-plant pairs meet the prescribed decrease within
$10^{-6}\max(1,V)$. This branch-specific observation cannot establish one
common CLF across branch changes.

\section{Verification and reproducibility}
The independent Python geometry recomputation agrees with the MATLAB minimum
clearance in every case to within $10^{-8}$~m; recovery dwell was independently
recomputed from the recorded errors. The campaign shell returned status 143 after the completion marker and all
fourteen JSON/MAT result pairs plus the warm-up export were written; its
termination cause was not established. Results and completeness were verified
from the saved artifacts, without claiming a zero process exit status.
Frozen MATLAB, scenario and native-kernel
hashes identify the tested implementation. No new controller changes were
made, and no historical unit-test count is claimed as a test run in this task.

Data, complete frame summaries, configurations, timing environment and methods:
\path{report/CONTROLLER_SCENARIO_RETEST_20261001/}.
The manifest records the original large JSON/MAT traces in the cache and the
exported report artifacts. Native dependencies, generated report PDFs, raw
large traces and unrelated working-tree changes are excluded from the project
commit. The reproduction file gives the exact campaign and analysis commands.
\end{document}
'''
findings=[]
for c in no_recovery:
    findings.append(f"At {c['speed']}~m/s, {esc(c['scenario'])} ends with lateral error {c['finalError'][0]:.2f}~m, body clearance {c['finalGap']:.2f}~m and {c['recoveryClfFrames']} cost-to-go-CLF frames. The observed trajectory does not return to nominal cruise within 80~s.")
values=dict(COMMIT=source['testedCommit'],COLLISIONS=s['collisionCases'],INTERRUPTS=s['interruptedCases'],HOLDS=s['totalHolds'],RECOVERED=s['recovered'],TMIN=min(c['recoveryConfirmation'] for c in recovered),TMAX=max(c['recoveryConfirmation'] for c in recovered),ROWS='\n'.join(rows),NONRECOVERY='\n\n'.join(findings),MEDIAN=f"{run['median']*1000:.3f}",P95=f"{run['p95']*1000:.3f}",MAXIMUM=f"{run['maximum']*1000:.3f}",OVER=run['over100ms'],OVERPERCENT=f"{100*run['over100ms']/run['count']:.1f}",SCENARIO=esc(w['scenario']),SPEED=w['speed'],TIME=f"{w['time']:.2f}",PARTS='\n'.join(parts),FLAGS=str(w['solverFlags']),RESTARTS=len(s['flowRestarts']),NONCONVERGED=sum(c['nonpositiveStageFrames'] for c in s['cases']),ZERO=sum(c['recoveryClfZeroSlack'] for c in s['cases']),FREE=sum(c['recoveryClfFrames'] for c in s['cases']),DECREASE=sum(c['recoveryClfDecreasePairs'] for c in s['cases']),PAIRS=sum(c['recoveryClfConsecutivePairs'] for c in s['cases']))
for key in sorted(values,key=len,reverse=True):
    value=values[key]
    if key in ('TMIN','TMAX'):value=f'{value:.2f}'
    text=re.sub(r'\b'+key+r'\b',lambda _:str(value),text)
(root/'report/CONTROLLER_SCENARIO_RETEST_20261001.tex').write_text(text)
