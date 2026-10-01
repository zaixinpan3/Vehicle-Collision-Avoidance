% First-step correction versus the rest of the returned plan in the no-risk
% frames whose issued steering is on its trust-box edge (rho > 0). The residual is
% the issued steering minus the straight trim (the terminal reference input that
% is appended at the tail of every shifted plan), signed by the first correction.
D='/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001';
source='/home/zai/.cache/collisionAvoidance/margin-5cm-20261001/source';
addpath(fullfile(source,'controller'),fullfile(source,'config'));
files=dir(fullfile(D,'replay','*-full.mat'));T=table();
for i=1:numel(files)
    load(fullfile(files(i).folder,files(i).name),'record');f=record.frame;
    straight=terminalContinuation.build(record.configuration,0).reference.input(1);
    free=find([f.encounterExit]==0);edge=[];d0=[];later=[];second=[];residual=[];
    for k=free
        d=f(k).plan(1,:)-f(k).anchorInputs(1,:);V0=f(k).clfInitialValue;
        if f(k).clfSlack>1e-6*max(1,V0) && abs(d(1))>0.0749
            edge(end+1)=k;d0(end+1)=d(1);later(end+1)=sum(d(2:end));second(end+1)=d(2); %#ok<AGROW>
            residual(end+1)=sign(d(1))*(f(k).input(1)-straight); %#ok<AGROW>
        end
    end
    if isempty(edge),continue;end
    T=[T;table(string(record.scenario),record.speed,numel(edge),mean(sign(later)==-sign(d0)),mean(sign(second)==-sign(d0)), ...
        median(-later./d0),median(abs(second)),median(residual),prctile(residual,10),prctile(residual,90), ...
        'VariableNames',["scenario","speed","steeringEdgeFrames","shareLaterSumOpposite","shareSecondOpposite", ...
        "medianCancelledFraction","medianAbsSecondCorrection","medianResidualSteering","p10ResidualSteering","p90ResidualSteering"])]; %#ok<AGROW>
end
T=sortrows(T,["speed","scenario"]);writetable(T,fullfile(D,'cancellation.csv'));disp(T);
