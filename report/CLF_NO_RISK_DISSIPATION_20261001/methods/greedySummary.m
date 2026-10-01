% Summary table of the idealized one-step CLF policies (greedy/*.mat).
D='/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001';
files=dir(fullfile(D,'greedy','*.mat'));T=table();tol=[.1;pi/180;.1;.05;.01];h=0.05;
for i=1:numel(files)
    load(fullfile(files(i).folder,files(i).name),'res');r=res;E=r.E;X=r.X;
    inside=all(abs(E)<=tol,1);entry=find(inside,1);firstEntry=NaN;insideAfter=NaN;
    if ~isempty(entry),firstEntry=(entry-1)*h;insideAfter=mean(inside(entry:end));end
    if isfinite(r.recoveredAfter),outcome="recovered";
    elseif strlength(r.failure)>0,outcome="failed: "+r.failure;
    elseif ~isempty(entry) && insideAfter>0.95,outcome="converged; 5-s dwell reset by grid-induced yaw-rate excursions";
    else,outcome="not recovered in 80 s";
    end
    e0=E(:,1);
    T=[T;table(string(r.policy),string(r.scenario),r.speed,r.startTime,string(r.note),e0(1),e0(2),e0(3),e0(4),e0(5), ...
        r.steps*h,r.recoveredAfter,firstEntry,insideAfter,r.maxAbsLateral,min(X(4,:)),max(abs(X(5,:))),max(abs(X(6,:))), ...
        max(abs(r.U(1,:))),r.positiveSlackSteps/max(1,numel(r.rho)),outcome, ...
        'VariableNames',["policy","scenario","speed","startTime","startNote","startLateral","startHeading", ...
        "startSpeedError","startLateralVelocityError","startYawRateError","simulatedSeconds","recoveredAfter", ...
        "firstInsideTolerance","shareInsideAfterEntry","maxAbsLateral","minSpeed","maxAbsLateralVelocity", ...
        "maxAbsYawRate","maxAbsSteering","shareOfHoldsWithPositiveSlack","outcome"])]; %#ok<AGROW>
end
T.near=abs(T.startLateral)<=10;
T=sortrows(T,["near","speed","scenario","policy"],["descend","ascend","ascend","ascend"]);
writetable(T,fullfile(D,'greedy-summary.csv'));
disp(T(:,["policy","scenario","speed","startLateral","recoveredAfter","firstInsideTolerance","minSpeed","maxAbsLateralVelocity","maxAbsSteering","outcome"]));
for nearValue=[true,false]
    S=T(T.near==nearValue,:);cases=unique(S.scenario+"-"+S.speed);
    fprintf('%s starts: %d cases; ',string(nearValue),numel(cases));
    for p=unique(S.policy).'
        Q=S(S.policy==p,:);ok=isfinite(Q.recoveredAfter) | startsWith(Q.outcome,"converged");
        fprintf('%s %d/%d (recovery %s s); ',p,nnz(ok),height(Q),mat2str(sort(Q.recoveredAfter(isfinite(Q.recoveredAfter))).'));
    end
    fprintf('\n');
end
