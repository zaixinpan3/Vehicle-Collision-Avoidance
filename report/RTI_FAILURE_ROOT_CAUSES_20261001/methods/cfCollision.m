% Locate overlaps in a counterfactual record and classify them (set file).
source='/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/source';
addpath(fullfile(source,'controller'),fullfile(source,'config'),fullfile(source,'scripts'));
load(file,'record');f=record.frame;cfg=record.configuration;h=cfg.controller.sampleTime;q=record.targetEpoch;
shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
bad=find([f.clearance]<=0);
fprintf('%s: %d holds with overlap (first at t=%.2f); PCBF optimum max over those holds %.3g\n',file,numel(bad),f(bad(1)).time,max([f(bad).primaryOptimum]));
for k=bad(1:min(3,end))
    tt=linspace(0,h,2001);
    [~,s]=ode45(@(~,y)nonlinearBicycleModel.derivative(y,f(k).input,cfg),tt,f(k).state,odeset('RelTol',1e-11,'AbsTol',1e-12));
    d=zeros(size(tt));
    for j=1:numel(tt),p=predictiveSafetyGeometry.targetFlow(q,f(k).time+tt(j));d(j)=predictiveSafetyGeometry.rectangle(s(j,1:3).',shape,p(1:3),p(8:11));end
    [~,jm]=min(abs(tt-h/2));inside=tt(d<=0);
    fprintf('  hold %.2f s: pcbf=%.3g flags=%s start %.4f mid %.4f end %.4f | overlap +%.2f..+%.2f ms | affine plan min %.4f@%d | u=[%.3f %.3f]\n', ...
        f(k).time,f(k).primaryOptimum,mat2str(f(k).stageFlags),d(1),d(jm),d(end),1e3*inside(1),1e3*inside(end),f(k).affineMargin,f(k).affineMarginStage,f(k).input);
end
