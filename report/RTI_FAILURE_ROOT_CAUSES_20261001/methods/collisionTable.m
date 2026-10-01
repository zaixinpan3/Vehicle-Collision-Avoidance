% Collision-hold measurements for the two 15-m/s collisions, exported to CSV.
source='/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/source';
addpath(fullfile(source,'controller'),fullfile(source,'config'),fullfile(source,'scripts'));
cases={{"headOn",1.10},{"acceleratingHeadOn",1.05}};T=table();
for c=1:numel(cases)
    name=cases{c}{1};hold=cases{c}{2};
    load(sprintf('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/replay/%s-15.mat',name),'record');
    cfg=record.configuration;h=cfg.controller.sampleTime;q=record.targetEpoch;margin=cfg.collision.safetyMarginMeters;
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    k=find(abs([record.frame.time]-hold)<1e-9,1);f=record.frame(k);
    tt=linspace(0,h,5001);
    [~,s]=ode45(@(~,y)nonlinearBicycleModel.derivative(y,f.input,cfg),tt,f.state,odeset('RelTol',1e-11,'AbsTol',1e-12));
    d=zeros(size(tt));depth=d;
    for j=1:numel(tt)
        p=predictiveSafetyGeometry.targetFlow(q,f.time+tt(j));
        d(j)=predictiveSafetyGeometry.rectangle(s(j,1:3).',shape,p(1:3),p(8:11));depth(j)=localPenetration(s(j,1:3).',shape,p(1:3),p(8:11));
    end
    rk=nonlinearBicycleModel.sample(f.state,f.input,cfg);mid=(f.state+rk)/2;pm=predictiveSafetyGeometry.targetFlow(q,f.time+h/2);
    interpMid=predictiveSafetyGeometry.rectangle(mid(1:3),shape,pm(1:3),pm(8:11));
    check=predictiveSafetyGeometry.intervalClearance(f.state(1:3),s(end,1:3).',shape,q,f.time,h,margin);
    inside=tt(depth>0);[~,jm]=min(abs(tt-h/2));
    priorPcbf=max([record.frame(1:k).primaryOptimum]);
    row=table(name,15,f.time,f.primaryOptimum,priorPcbf,string(mat2str(f.stageFlags)),d(1),d(jm),interpMid,d(end),f.affineMargin, ...
        norm(rk(1:2)-s(end,1:2).'),1e3*inside(1),1e3*inside(end),1e3*(inside(end)-inside(1)),1e3*max(depth),check.certified,check.lipschitz, ...
        'VariableNames',["scenario","speed","holdStart","pcbfOptimum","maxPcbfOptimumUpToHold","exitFlags","clearanceStart","clearanceTrueMidpoint", ...
        "clearanceInterpolatedMidpoint","clearanceEnd","affinePlanMinimumNodeClearance","oneStepRk4PositionError","overlapStartMs","overlapEndMs", ...
        "overlapDurationMs","maximumPenetrationMm","offlineIntervalCertificate","distanceLipschitzBound"]);
    T=[T;row]; %#ok<AGROW>
end
out='/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/bundle';if ~isfolder(out),mkdir(out);end
writetable(T,fullfile(out,'collision-holds.csv'));disp(T);
function d=localPenetration(poseE,shapeE,poseT,shapeT)
    re=[cos(poseE(3)),-sin(poseE(3));sin(poseE(3)),cos(poseE(3))];rt=[cos(poseT(3)),-sin(poseT(3));sin(poseT(3)),cos(poseT(3))];
    signs=[-1,1,1,-1;-1,-1,1,1];e=poseE(1:2)+re*(shapeE(3:4)+shapeE(1:2).*signs);t=poseT(1:2)+rt*(shapeT(3:4)+shapeT(1:2).*signs);
    d=Inf;for axis=[re,rt],pe=axis.'*e;pt=axis.'*t;d=min(d,min(max(pe),max(pt))-max(min(pe),min(pt)));end;d=max(d,0);
end
