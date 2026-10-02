from pathlib import Path
import json,hashlib,shutil,subprocess
root=Path('/home/zai/Downloads/ResearchProjects/collisionAvoidance');raw=Path(__file__).parent
bundle=root/'report/UNIFIED_CLF_DESIGN_20261001';bundle.mkdir(exist_ok=True);(bundle/'methods').mkdir(exist_ok=True)
p=json.loads((raw/'probe.json').read_text());rows=[]
for r in p['recordedStates']:
    if r['time']<9:continue
    rows.append(f"{r['speed']} & {r['time']:.2f} & {r['steps']} & {r['value']:.4g} & {r['stage']:.4g} & {r['nominalValueChange']/r['stage']:.4f} & {r['issuedValueChange']/r['stage']:.4f}"+r' \\')
text=r'''\documentclass[11pt]{article}
\usepackage[margin=0.85in]{geometry}
\usepackage{amsmath,amssymb,booktabs,xurl,hyperref}
\usepackage[T1]{fontenc}
\setlength{\emergencystretch}{3em}
\title{One CLF Throughout Avoidance and Nominal Recovery\\Design Correction and Offline Diagnostics}
\author{Pan Dynamics Collision Avoidance Research}
\date{October 1, 2026}
\begin{document}\maketitle
\section{Decision and scope}
Use one target-independent nominal-behaviour CLF at every frame. Collision
constraints retain first priority; their interaction with the same CLF is
expressed through its optimized slack. Neither the CLF definition nor the
requirement to recover should depend on an encounter-distance gate.

This is a design analysis of controller commit
\path{9de49299a7c4f7d6b05e6a0dea48453fb7877a5a}, following the independent retest
at \path{811633202b9bf6f4c9125c332e88092b85372d6b}. Production control code is
unchanged. The new diagnostics evaluate recorded states, not a new closed-loop
controller. No claim is made that the proposed correction already passes all
fourteen scenarios or meets 100~ms.

\section{What has to change}
The present implementation has three coupled changes at an encounter-exit
index of zero: it substitutes the accumulated recovery cost for the lane LQR
quadratic, replaces the shifted-plan initialization with a recovery-feedback
rollout, and runs a third optimization to keep the issued input close to that
feedback. Therefore deleting the CLF conditional alone is insufficient to
justify the old recovery proof. That proof was tied to executing a particular
input, whereas the unified optimizer must be free to choose an avoiding input.

The two straight-crossing cases never leave the encounter range and never use
the accumulated-cost function. At 80~s they have lateral errors of
$-621.03$/$-629.01$~m at 8/15~m/s. Restoring the lane quadratic everywhere would
also be inadequate: its fixed 1\% one-hold decay was already shown to fail far
from the path in \path{report/CLF_NO_RISK_DISSIPATION_20261001.tex}.

\section{A single mathematical object}
Let $y$ contain the ego state in the path frame and any required input memory.
The nominal set $\mathcal A$ contains all phases of given-path cruising at the
desired speed. Define the error $e(y)$ by the existing five transverse errors;
there is no fixed longitudinal position or arrival time. Set
$\ell(y)=e(y)^TQe(y)$ with $Q\succ0$. When finite slew limits matter, the
construction must include their input memory; the current default has infinite
braking slew and permits the five-dimensional quotient analysis.

Choose an admissible nominal stabilizing feedback $\kappa$ on a declared
domain. It is used to construct/evaluate the CLF, not as an executable fallback
or a controller selected after an encounter. With $F_\kappa(y)=F_h(y,\kappa(y))$,
the ideal construction is
\[
 V_\infty(y)=\sum_{j=0}^{\infty}\ell(F_\kappa^j(y)),\qquad
 V_\infty(F_\kappa(y))-V_\infty(y)=-\ell(y).
\]
This requires a finite sum and suitable bounds relative to distance from
$\mathcal A$ on the chosen invariant domain. Convergence in a finite grid alone
does not establish these assumptions. Targets do not occur in this definition.

\subsection{A finite construction with an explicit proof obligation}
A practical exact finite construction, before numerical approximation, is
\[
 V_K(y)=\sum_{j=0}^{K-1}\ell(F_\kappa^j(y))+V_f(F_\kappa^K(y)).
\]
Choose one fixed $K$ on the declared operating domain $\mathcal D$ and certify
that $F_\kappa^K(y)$ enters a positively invariant local core $\mathcal X_f$ for
all $y\in\mathcal D$. In that core require the nonlinear held-input inequality
\[
 V_f(F_\kappa(z))-V_f(z)\le-\ell(z),\qquad z\in\mathcal X_f.
\]
Writing $y_K=F_\kappa^K(y)$ gives directly
\[
 V_K(F_\kappa(y))-V_K(y)
 =-\ell(y)+\ell(y_K)+V_f(F_\kappa(y_K))-V_f(y_K)
 \le-\ell(y).
\]
$V_f$ is a component of this one function, evaluated in the same formula at
every frame. It is not an encounter-dependent second CLF. $K$ is a parameter
of CLF construction, not a constraint that the optimized avoidance trajectory
must return to the path in $K$ steps. The certified domain and a sufficient $K$
remain to be established for the actual vehicle. They are not inferred from
the diagnostic samples below.

The current $P_f$ solves a linearized equality with exactly $Q$, leaving no
strict nonlinear remainder reserve. A constructive correction is to solve
$A_\kappa^TP_fA_\kappa-P_f=-\gamma Q$, $\gamma>1$, then bound the nonlinear
remainder and shrink $\mathcal X_f$ until the above inequality, invariance and
model-domain conditions hold. This is the standard terminal-decrease principle;
see Rawlings, Mayne and Diehl, Assumption 2.14 and Appendix B.5~\cite{mpc}.
The construction here is an application to the nominal path quotient, not a
claim that the reference proves this particular Fiala implementation.

\section{Exactly two optimization stages}
With the same $V$ at every frame, require
\[
 V(F_h(y_k,u_0))-V(y_k)\le-\eta\ell(y_k)+\rho,\qquad
 \rho\ge0,\quad0<\eta<1.
\]
The online priorities remain
\[
 \sigma_k^*=\min_{\mathbf u,\boldsymbol\xi,\rho}\sum_i\xi_i,
 \qquad
 \min_{\mathbf u,\boldsymbol\xi,\rho}\rho
 \quad\text{subject to }\sum_i\xi_i\le\sigma_k^*+\varepsilon_{\rm lex}.
\]
Both stages contain the same dynamics, collision, admissibility, completion
and CLF constraints. The CLF slack is unbounded above in the first stage, so
its descent requirement does not silently impose another safety priority.
Remove the third anchor-deviation solve. At zero slack, every input satisfying
an accurate decrease constraint has the needed descent; there is no reason to
force the specific construction feedback $\kappa$ to be executed.

If the true-model inequality holds with $\rho=0$ for every subsequent frame,
then $\eta\sum_k\ell(y_k)\le V(y_{k_0})$, so $e(y_k)\to0$, provided the
trajectory stays in the certified domain. One isolated zero-slack frame is
not a convergence result. With persistent numerical tolerances only practical
convergence is generally established unless their error is made summable or
vanishing relative to the descent term. No fixed fraction of $V$ per frame is
required, so the design does not impose a fixed exponential recovery rate.

A positive optimized slack means incompatibility inside the current constrained
local problem. It is not proof of a physical collision threat: trust regions,
terminal rows and approximation conservatism can also cause it. Conversely,
removing a target-distance gate allows recovery while a target is still nearby
whenever the constraints permit it. CLF feasibility must not be inferred solely
from range.

\section{Make the convex constraint apply to the chosen input}
The present residual Gauss--Newton model is exact at its anchor but is not an
upper bound on the nonlinear next-state value. Zero slack in that model alone
therefore does not justify the true-model decrease inequality. Its current
third-stage feedback anchoring is part of why no-risk frames succeeded.

For the fixed measured state consider the two-input function
$\Phi_k(u)=V(F_h(y_k,u))$, including the input-memory successor. In a region
where its gradient in the scaled coordinates has a certified Lipschitz bound,
a convex upper model is
\[
 \Phi_k(\bar u)+g_k^T\Delta u+\tfrac{L_k}{2}\|D^{-1}\Delta u\|^2,
 \qquad u=\bar u+\Delta u.
\]
Constrain this upper model by $V(y_k)-\eta\ell(y_k)+\rho$. The bound must cover
both the value function and the nonlinear hold dynamics. This is a candidate
constraint construction, not a fitted Hessian asserted to be a certificate.
Any value/gradient approximation error needs a valid additional bound. The
result is one convex quadratic/conic CLF constraint in the existing two-stage
optimization; it needs no post-solve nonlinear acceptance loop.

The present feedback has saturation and heading-wrap boundaries. Fixed $K$
removes the stopping-time derivative issue but does not remove those
nonsmoothness issues. Smooth regions or a valid nonsmooth upper model must be
established on the required domain; an unrestricted global Hessian bound is
not claimed. A conservative bound can also exclude zero slack even when an
exact descent input exists, which must be measured during implementation.

Retain the shifted previous trajectory, with moving-target flow initialization
when it is unavailable or the numerical solve fails. Their purpose is to build
an affine model, not define $V$. Remove the distance-triggered recovery-anchor
branch. Also address the previously measured shifted-plan trust-region trap:
a valid CLF does not guarantee its descent input lies in the numerical trust
region or admits the required safe completion. The single-CLF refactor must
test this distinction rather than attributing every positive slack to safety.
No extra physical steering restriction, road constraint or executable backup
is proposed here.

\section{Offline diagnostics from the two failed crossing cases}
The current accumulated-cost evaluator was evaluated at six recorded times
per speed, including the applied input's held-model successor and the nominal
construction input's successor. All twelve evaluations and their successors
reached its stopping level before the configured 120-s cap. The controls below
are probes, not collision-certified or newly executed policies.
\begin{center}\small
\begin{tabular}{rrrrrrr}\toprule
Speed & Time (s) & Steps & $V$ & $\ell$ & $\Delta V_\kappa/\ell$ & $\Delta V_{\rm issued}/\ell$\\\midrule
TABLE
\bottomrule\end{tabular}\end{center}
At both speeds the issued controls increase this common candidate function
at all four sampled late times, while the construction feedback decreases it.
This supports replacing the unsuitable encounter-phase quadratic; it does not
establish that the nominal descent input satisfies the PCBF horizon constraints.

Forty additional small perturbations, at 8/15~m/s and curvatures 0/0.005~m$^{-1}$,
start inside the existing stopping set at $V_f=5\times10^{-4}$. All satisfy
half-stage decrease, but sixteen do not satisfy full-stage decrease to
$10^{-12}$ absolute tolerance. The largest absolute Bellman-equality residual
is $1.7430\times10^{-6}$; the largest residual/stage-cost ratio is 0.006737.
Thus the current local equality claim is too strong, although these samples
do not disprove its configured half-stage decrease condition. Adaptive
stopping also makes a fixed-length finite-difference residual a different
local extension when the stopping index changes.

\section{Computational implications and implementation scope}
One existing value evaluation at the late 8-m/s crossing state used 1,874
auxiliary rollout steps and about 72~ms in this diagnostic. The current six
perturbed rollouts would multiply that work. Do not simply enable that
implementation everywhere and assume the 100-ms requirement will follow.
Values and input sensitivities should be propagated together, with derivatives
of the construction feedback, rather than obtained by six long perturbed
rollouts. A compact offline representation is usable only after its decrease
and approximation bounds are established. These acceleration routes remain
implementation work, not a measured speed claim. Wang and Boyd~\cite{fast}
show why exploiting a quadratic CLF's structure can be fast; their linear/
quadratic result is not a timing guarantee for the present nonlinear controller.

Concrete source scope: replace the CLF conditional in
\path{solvePredictiveControl.localFormulate}; remove the recovery-only third
stage in \path{localStep}; decouple initialization in \path{localInitialization};
replace the evaluator/derivatives in \path{nonlinearBicycleModel}; expose one
CLF identifier and one decrease rule in metadata. Do not alter the PCBF
priority or claim that its affine geometry becomes a nonlinear safety proof.

Acceptance evidence must include: identical current-state $V$ for identical
ego/path states with different target ranges; zero-slack true-model descent
for optimized inputs, not only $\kappa$; single-CLF two-stage operation while
targets remain nearby; trust-region/terminal compatibility diagnostics; all
fourteen closed loops through avoidance and sustained nominal recovery; and
full controller-call runtime against 100~ms. None of those new closed-loop
acceptance tests has been run for a refactored implementation in this task.

\section{Artifacts and review}
\path{report/UNIFIED_CLF_DESIGN_20261001/} contains recorded probe inputs,
results, exact method, source hashes and reproduction instructions. MATLAB
completed the probe with exit status zero. No controller code was modified.
The analysis used the academic-research-suite verification and assumption-review
workflow inline: evidence, proposed construction and open proof obligations
are separated. This is a concrete correction design, not a claim of completed
controller implementation.

\begin{thebibliography}{9}
\bibitem{mpc} J. B. Rawlings, D. Q. Mayne and M. M. Diehl.
\emph{Model Predictive Control: Theory, Computation, and Design}, second
edition, sixth printing, 2026. Assumption 2.14 and Appendix B.5.
\url{https://sites.engineering.ucsb.edu/~jbraw/mpc/MPC-book-2nd-edition-6th-printing.pdf}
\bibitem{fast} Y. Wang and S. Boyd. Fast Evaluation of Quadratic
Control-Lyapunov Policy. \emph{IEEE Transactions on Control Systems Technology},
19(4), 939--946, 2011. DOI: 10.1109/TCST.2010.2056371.
\url{https://web.stanford.edu/~boyd/papers/pdf/fast_clf.pdf}
\end{thebibliography}
\end{document}
'''.replace('TABLE','\n'.join(rows))
report=root/'report/UNIFIED_CLF_DESIGN_20261001.tex';report.write_text(text)
for name in ['states.json','probe.json']:shutil.copy2(raw/name,bundle/name)
for name in ['probe.m','buildReport.py']:shutil.copy2(raw/name,bundle/'methods'/name)
(bundle/'probe-output.txt').write_text('\n'.join(x.rstrip() for x in (raw/'probe.log').read_text().splitlines())+'\n')
(bundle/'REPRODUCTION.txt').write_text('''Single-CLF design diagnostics, October 1, 2026.
Production controller was not changed. Working HEAD at analysis: 811633202b9bf6f4c9125c332e88092b85372d6b.
Controller source: 9de49299a7c4f7d6b05e6a0dea48453fb7877a5a, frozen in:
/home/zai/.cache/collisionAvoidance/recovery-clf-validation-20261001/source
Raw task directory:
/home/zai/.cache/collisionAvoidance/unified-clf-design-20261001

Inputs were extracted from the fresh crossing JSON for both speeds at hold
indices round(t/0.05)+1 for t=[0,2,10,30,60,79.95] seconds. Each record contains
the measured state, prior applied input (zero on the first frame) and the actual
issued input. All are deterministic recorded values; no random seed is used.

Executed:
matlab -singleCompThread -batch "run('/home/zai/.cache/collisionAvoidance/unified-clf-design-20261001/probe.m')"
The probe evaluates the existing target-independent recovery value at each
state and two successors, using the held-input RK4 model. The nominal input
is NOT collision-screened and no new closed-loop controller is executed.
Additional 40 signed coordinate perturbations have terminal quadratic 0.0005,
for speed 8/15 m/s and curvature 0/0.005 per meter. They test the Bellman
identity separately from the configured half-stage decrease condition.
Probe completed with exit status 0 and marker PROBE-COMPLETE.

Repeat by copying methods/probe.m and states.json into a new raw directory,
adjusting source/output in probe.m to a source checkout and output directory,
and providing the native binaries identified by source-manifest.json.
Build report: python3 RAW/buildReport.py (its root/raw paths identify this run).
Report compiled with pdflatex -interaction=nonstopmode -halt-on-error.

Full new-controller regression and performance tests have NOT been run.
Report contains a proposed design and its explicit proof/implementation gaps.
The previous 14-case retest supplies observed failure states, not evidence
that the proposed unified algorithm already resolves them.
''')
old=json.loads((Path('/home/zai/.cache/collisionAvoidance/recovery-clf-validation-20261001/source-manifest.json')).read_text())
for rel,digest in old['files'].items():assert hashlib.sha256((root/rel).read_bytes()).hexdigest()==digest
(bundle/'source-manifest.json').write_text(json.dumps(old,indent=2)+'\n')
def entry(path):return dict(path=str(path.relative_to(root)),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
files=sorted(p for p in bundle.rglob('*') if p.is_file() and p.name!='manifest.json')+[report]
(bundle/'manifest.json').write_text(json.dumps(dict(kind='Design/probe artifact hashes; no weekly or monthly report hashes',baseCommit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),controllerSourceUnchanged=True,files=[entry(p) for p in files],primarySources=['https://sites.engineering.ucsb.edu/~jbraw/mpc/MPC-book-2nd-edition-6th-printing.pdf','https://web.stanford.edu/~boyd/papers/pdf/fast_clf.pdf']),indent=2)+'\n')
print('Built design report and probe bundle.')
