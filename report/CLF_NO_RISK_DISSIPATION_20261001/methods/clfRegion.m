% Where is V = e'Pe actually a CLF for the 1%-per-hold condition?
% A CLF on a sublevel set {V <= c} needs, at every state in it, an admissible
% input with V(x1) <= 0.99 V(x0). The feasibility map (straight lane, v = v_ref,
% v_y = r = 0, inputs |delta| <= 0.7, |b| <= 0.999) is a slice of the state space,
% so the smallest V0 among its infeasible points is an UPPER bound c* on any
% certified level. Exactly reversed headings (+-180 deg) are skipped: there the
% wrapped heading makes the map report an artificial decrease.
% Compared with: V0 at the start of each case's no-risk phase (5-cm replays), and
% how often the idealized one-step policies found no 1% input while V >= 1.
D='/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001';
source='/home/zai/.cache/collisionAvoidance/margin-5cm-20261001/source';
addpath(fullfile(source,'controller'),fullfile(source,'config'));
M=readtable(fullfile(D,'clf-feasibility-map.csv'));
cstar=struct();
for speed=[8,15]
    r=M(M.speed==speed & abs(abs(M.headingError)-pi)>1e-6,:);feasible=logical(r.onePercentFeasible);bad=r(~feasible,:);
    [c,i]=min(bad.V0);cstar.(sprintf('v%d',speed))=c;
    fprintf('v=%2d m/s: smallest V0 with no 1%% input = %.3g at e_y=%.1f m, e_psi=%+.0f deg (upper bound on a certified level)\n', ...
        speed,c,bad.lateralError(i),rad2deg(bad.headingError(i)));
    ok=r(feasible,:);
    fprintf('          largest V0 with a 1%% input on the slice = %.3g; infeasible points %d of %d\n',max(ok.V0),height(bad),height(r));
end
fprintf('\nNo-risk phase start (first free frame at or after 2 s), 5-cm replays:\n');
files=dir(fullfile(D,'replay','*-full.mat'));
for i=1:numel(files)
    load(fullfile(files(i).folder,files(i).name),'record');f=record.frame;
    k=find([f.encounterExit]==0 & [f.time]>=2,1);if isempty(k),continue;end
    c=cstar.(sprintf('v%d',record.speed));
    fprintf('  %-18s %2d m/s: t=%5.2f s, V0=%10.4g = %8.3g x c*\n',record.scenario,record.speed,f(k).time,f(k).clfInitialValue,f(k).clfInitialValue/c);
end
fprintf('\nIdealized one-step policies: holds with V >= 1 and no 1%% input (positive slack)\n');
G=dir(fullfile(D,'greedy','*.mat'));
for i=1:numel(G)
    load(fullfile(G(i).folder,G(i).name),'res');c=cstar.(sprintf('v%d',res.speed));
    V=res.V(1:numel(res.rho));pos=res.rho>1e-6*max(1,V);
    above=V>c;inside=V>=1 & V<=c;
    fprintf('  %-16s %-18s %2d: V>=1 holds %4d, positive slack %5.1f%%; inside c*: %4d holds, positive %5.1f%%; above c*: %4d holds, positive %5.1f%%\n', ...
        res.policy,res.scenario,res.speed,nnz(V>=1),100*mean(pos(V>=1)),nnz(inside),100*mean(pos(inside)),nnz(above),100*mean(pos(above)));
end
