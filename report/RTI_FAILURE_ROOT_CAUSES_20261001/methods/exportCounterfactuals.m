% Summarize every saved counterfactual record into one CSV.
W='/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001';
addpath(fullfile(W,'source','controller'),fullfile(W,'source','config'));
files=dir(fullfile(W,'cf','*.mat'));T=table();tol=[.1;pi/180;.1;.05;.01];
for i=1:numel(files)
    load(fullfile(files(i).folder,files(i).name),'record');f=record.frame;
    stem=erase(files(i).name,'.mat');parts=split(stem,'-');variant=string(parts{1});
    e=[f.error1];time=[f.time]+record.configuration.controller.sampleTime;inside=all(abs(e)<=tol,1)&time>=8;
    recovered=NaN;start=NaN;
    for k=1:numel(time)
        if ~inside(k),start=NaN;elseif isnan(start),start=time(k);end
        if inside(k) && time(k)-start>=5-1e-10,recovered=time(k);break;end
    end
    [gap,kg]=min([f.clearance]);failure="";failureTime=NaN;
    if isfield(record,'failure'),failure=string(record.failure);failureTime=record.failureTime;end
    margin=record.configuration.collision.safetyMarginMeters;
    row=table(variant,string(record.scenario),record.speed,margin,numel(f),failure,failureTime,gap,f(kg).time,recovered, ...
        e(1,end),e(3,end),max([f.primaryOptimum]),max([f.seconds]), ...
        'VariableNames',["variant","scenario","speed","safetyMarginMeters","frames","failure","failureTime","minimumClearance", ...
        "minimumClearanceHold","recoveryConfirmation","finalLateralError","finalSpeedError","maximumPcbfOptimum","maximumCallSeconds"]);
    T=[T;row]; %#ok<AGROW>
end
writetable(T,fullfile(W,'bundle','counterfactuals.csv'));disp(T(:,["variant","scenario","speed","frames","failureTime","minimumClearance","recoveryConfirmation","finalLateralError","maximumCallSeconds"]));
