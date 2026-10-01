from pathlib import Path
import json
root=Path('/home/zai/Downloads/ResearchProjects/collisionAvoidance');raw=Path(__file__).parent
s=json.loads((raw/'summary.json').read_text());serial=json.loads((raw/'serial-summary.json').read_text());seeds=json.loads((raw/'seeds.json').read_text());w=serial['worst']
def esc(v):
 return str(v).replace('_',r'\_').replace('&',r'\&').replace('%',r'\%')
def f(v,d=3):return '--' if v is None else f'{v:.{d}f}'
rows=[]
for r in s['cases']:
 status='Recovered' if r['recovered'] else ('Collision' if r['minimumClearance']==0 else ('No result' if r['failure'] else 'Not recovered'))
 rows.append(f"{r['speed']} & {esc(r['scenario'])} & {r['seconds']:.2f} & {f(r['minimumClearance'])} & {f(r['finalError'][0])} & {r['solverRestarts']} & {status}"+r' \\')
seedrows=[f"{r['speed']} & {r['scenario']} & {f(r['minimumSampledClearance'],4)} & {r['terminalNormOverRadius']:.3g}"+r' \\' for r in seeds]
text=r'''\documentclass[10pt]{article}
\usepackage[margin=0.7in]{geometry}
\usepackage{amsmath,booktabs,longtable,hyperref,xurl}
\hypersetup{colorlinks=true,urlcolor=blue}
\setlength{\parindent}{0pt}\setlength{\parskip}{5pt}
\title{Real-time iteration with flow reinitialization: implementation and experiment}
\author{Collision Avoidance Research}\date{October 1, 2026}
\begin{document}\maketitle
\section{Change and execution contract}
The controller now shifts the previous optimized inputs and rolls them out from the current measured state. It rebuilds the Fiala dynamics, tire derivatives, rectangle separating directions and CLF linearization on that single trajectory. Each ordinary frame performs one lexicographic pair: minimize PCBF slack, then minimize CLF slack at the achieved PCBF level. The previous affine state trajectory is no longer a dynamics reference.

A warm attempt returning no finite vector in either stage triggers one fresh moving-target flow rollout and a complete model rebuild, provided the same frame budget still has time. The restarted problem computes its own primary optimum and cap. There is at most one restart; a fresh flow attempt is not repeated identically. Inputs that cannot be rolled out in the model domain instead use fresh initialization before optimization. Neither route executes a separate flow controller or a retained control plan. Positive optimized slack and finite nonconverged solver points retain the earlier direct-return contract. No nonlinear candidate validation or new execution admission test was added.

With $\Delta=0.5$, numerical corrections satisfy
\[
 |u_i-\bar u_i|\le\Delta[0.15,0.25]^T.
\]
The steering correction of $0.075$ rad is a local iteration limit, centered anew each frame; there is still no actuator steering magnitude or slew constraint. It is not a certified universal model-error bound. This RTI-style implementation carries local updates across samples; it does not claim the local stability theorem of classical RTI applies automatically to these slack objectives and collision constraints. See \url{https://cdn.syscop.de/publications/Diehl2005c.pdf} for the original RTI principle. Controller continuation version is 63. The existing endpoint core, encounter-range convention and tire equations are unchanged.

\section{Experiment protocol}
Fourteen deterministic collision-threat cases use speeds 8 and 15 m/s, exact state observations, known target motion, no road constraints, a 6-mm collision buffer and 50-ms control holds. Prefix lengths are 8 and 16; the fixed 3-second completion adds 60 stages. All baselines collide under ideal given-path cruise. No random sampling is used. The closed-loop plant uses independent \texttt{ode45} with relative tolerance $10^{-11}$ and absolute tolerance $10^{-12}$; rectangle clearance is measured at 31 nodes per hold. The online predictor uses its unchanged nominal RK4 model. Measured computation time is logged, not inserted as an actuation delay in this exact-observation simulation.

All cases first ran serially for 20 seconds in one single-thread MATLAB R2026a process, following two preparation calls at each speed. Timing includes the complete controller call, excludes offline replay, and reports preparation separately. Noncolliding cases were then resumed up to 80 seconds in three concurrent single-thread processes for recovery assessment. The two already-colliding cases were not extended. Parallel-extension times are retained in raw data but are not pooled into the serial timing benchmark. Recovery requires five consecutive seconds within $[0.1\,\mathrm m,1^\circ,0.1\,\mathrm{m/s},0.05\,\mathrm{m/s},0.01\,\mathrm{rad/s}]$ for transverse position, heading, speed, lateral velocity and yaw rate, starting no earlier than 8 seconds. This is a finite sampled recovery observation, not an asymptotic proof.

The solver soft budget remains 5 seconds and maximum iterations 400. Bounded outer work is not a measured 100-ms deadline guarantee. Native kernel and source hashes, configurations, raw continuations and run methods are recorded in the companion artifact manifest. No profiler ran during the serial campaign.

\section{Observed closed-loop outcomes}
'''
text+=f"All 14 serial cases completed 400 holds without an empty-result interruption. Two flow restarts occurred, both restoring the full two-stage solve. In the full observation windows, {s['recovered']} cases met the recovery criterion, {s['collisionCases']} cases contained sampled overlap, and {s['noResultCases']} cases reported an interrupted control call. Failure to recover by the cap is not a proof of infinite-time nonconvergence.\n"
text+=r'''\begin{longtable}{rlrrrrl}
\toprule Speed & Scenario & Time (s) & Min gap (m) & Final $e_y$ (m) & Restarts & Outcome \\
\midrule\endhead
'''+ '\n'.join(rows)+r'''
\bottomrule\end{longtable}
In the serial campaign, the 15-m/s curved-head-on case restarted at 0.10 s: the warm PCBF solve returned exit $-2$, and the fresh flow PCBF and CLF solves returned $[1,1]$. The whole call took 175.606 ms. The 15-m/s curved-crossing case restarted at 19.80 s with the same exit sequence, taking 183.572 ms. A third restart at 21.60 s in the parallel curved-crossing extension again returned $[-2,1,1]$. All three restarts produced a second-stage numerical control. These are direct examples of the requested recovery path. They do not establish that every bad initialization or nonlinear collision produces an empty convex problem.

\section{Cold flow trajectory quality}
The initializer itself was replayed at the start, midpoint and end of each prediction hold. Thirteen of fourteen initial trajectories had positive sampled rectangle clearance. The 15-m/s curved-head-on initialization collided: the retained 30-m encounter convention initially treats an approaching target beyond range as departed, suppressing flow guidance. The new flow restart at 0.10 s works once the target is within range. This experiment did not redesign that encounter convention.

The endpoint column below is the intrinsic terminal norm divided by the core radius; values greater than one mean that the search seed is outside the endpoint core. A useful avoidance seed need not satisfy this tiny terminal set, and none of these seed measurements is an execution certificate. Every recorded initialization has zero nominal discrete dynamics defect by construction.
\begin{longtable}{rlrr}
\toprule Speed & Scenario & Seed min gap (m) & Endpoint norm/radius \\
\midrule\endhead
'''+ '\n'.join(seedrows)+r'''
\bottomrule\end{longtable}
\section{Why two cases still collide}
Independent Python rectangle replay confirms the first overlaps at 1.103333 s for 15-m/s head-on and 1.068333 s for accelerating head-on. Replaying the original controller contexts reproduced their issued inputs exactly. Both collision-producing holds returned two positive solver exit flags and PCBF slack near $10^{-6}$, inside the reported $10^{-5}$ tolerance.

This failure is specifically between constrained sample times. For the head-on hold beginning at 1.10 s, the true ODE clearances at start, midpoint and endpoint are 9.718, 34.207 and 166.409 mm, yet overlap occurs between the first two. For the accelerating case's 1.05-s hold, these clearances are 49.048, 29.408 and 168.396 mm, again with overlap before the midpoint. The affine sampled positions also remain separated. Thus the evidence does not support attributing these two collisions primarily to positive relaxation or large one-step linearization error. A 6-mm margin at these particular sample nodes is insufficient to guarantee the whole intersample motion. The current empty-result restart condition cannot detect this mechanism. No intersample safety construction was added as part of the RTI change.

\section{Serial runtime and remaining limitations}
'''
text+=f"Across {serial['totalHolds']} serial holds, median full-call time was {serial['medianSeconds']*1000:.3f} ms, maximum {serial['maxSeconds']*1000:.3f} ms, and {serial['over100ms']} holds exceeded 100 ms. The maximum occurred in 8-m/s brakingLead at t={w['time']:.2f} s.\n"
text+=r'\begin{center}\begin{tabular}{lr}\toprule Component & Time (ms) \\\midrule'+'\n'
for name,value in [('Input rollout',w['initializationSeconds']),('Model and constraint assembly',w['formulationSeconds']),('PCBF conic solve',w['hold']['solverStages'][0]['seconds']),('CLF conic solve',w['hold']['solverStages'][1]['seconds']),('Other controller work',w['otherSeconds'])]:text+=f'{name} & {value*1000:.3f}'+r' \\'+'\n'
text+=r'''\bottomrule\end{tabular}\end{center}
The slow CLF solve returned a finite point with exit $-7$, so the unchanged output contract still used it. Most of this frame's cost is within the second convex solver, not the flow initializer or repeated nonlinear search. The RTI reference update and one-restart bound do not themselves establish a 100-ms execution limit.

Initialization repair and successful local solves also do not prove eventual nominal recovery. The measured non-recovered cases retain large tracking errors despite ongoing numerical solutions. Their positive CLF slack is not a trigger for the requested empty-result flow restart. No input regularization, changed CLF objective, terminal redesign or unrequested execution fallback was introduced.

\section{Validation and reproducibility}
All 200 tests in nine relevant MATLAB classes passed, including measured-state rollout, a failed shifted solve followed by fresh flow, stopping after one failed restart, model-domain reinitialization, preserving both objective stages, finite iteration-limit output, braking bounds and steering correction versus physical actuator limits. The 20 Python audit tests passed. Offline tests do not reintroduce online candidate admission.

Source and companion data are under \path{report/RTI_FLOW_REINITIALIZATION_20261001/}. That directory contains run scripts, compact results, exact source/native hashes, and sanitized console-log exports. Original logs, frozen source and full MAT/JSON traces remain under \path{/home/zai/.cache/collisionAvoidance/rti-controller-20261001/}; the manifest hashes them. Generated PDFs, native binaries and raw large continuations are excluded from the project commit. No unrelated estimator or manuscript changes are included.
\end{document}
'''
(root/'report/RTI_FLOW_REINITIALIZATION_20261001.tex').write_text(text)
