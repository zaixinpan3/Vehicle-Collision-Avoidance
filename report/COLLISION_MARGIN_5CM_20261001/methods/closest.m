% Closest approach and relaxed frames in the 5-cm campaign (set names).
source='/home/zai/.cache/collisionAvoidance/margin-5cm-20261001/source';
addpath(fullfile(source,'controller'),fullfile(source,'config'));T=table();
for speed=[8,15]
for name=["headOn","acceleratingHeadOn","brakingLead","crossing","turningCrossing","curvedHeadOn","curvedCrossing"]
    r=jsondecode(fileread(sprintf('/home/zai/.cache/collisionAvoidance/margin-5cm-20261001/campaign/speed%d/%s.json',speed,name)));r=r.results;
    tr=r.trace;cfg=r.configuration;q=r.targetInitialState;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    best=Inf;bestTime=NaN;nodeBest=Inf;
    for k=1:numel(tr)
        for j=1:numel(tr(k).auditTimes)
            p=predictiveSafetyGeometry.targetFlow(q,tr(k).time+tr(k).auditTimes(j));
            d=predictiveSafetyGeometry.rectangle(tr(k).auditStates(j,1:3).',shape,p(1:3),p(8:11));
            if d<best,best=d;bestTime=tr(k).time+tr(k).auditTimes(j);end
            if j==1 || j==16,nodeBest=min(nodeBest,d);end
        end
    end
    relaxed=find([tr.primaryOptimum]>1e-5);
    fprintf('%2d %-18s closest %.4f m at %.3f s; minimum at hold start/midpoint samples %.4f m; relaxed frames %s (max %.3g)\n', ...
        speed,name,best,bestTime,nodeBest,mat2str([tr(relaxed).time],4),max([[tr.primaryOptimum],0]));
    T=[T;table(speed,name,best,bestTime,nodeBest,numel(relaxed),max([[tr.primaryOptimum],0]), ...
        'VariableNames',["speed","scenario","closestClearance","closestTime","minimumSampledClearance","relaxedFrames","maximumPcbfOptimum"])]; %#ok<AGROW>
end
end
writetable(T,'/home/zai/.cache/collisionAvoidance/margin-5cm-20261001/closest-approach.csv');
