% Dense continuous-time clearance within holds around the first overlap.
% Usage: set name, speed, holds (vector of frame times) before running.
source='/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/source';
addpath(fullfile(source,'controller'),fullfile(source,'config'),fullfile(source,'scripts'));
load(sprintf('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/replay/%s-%d.mat',name,speed),'record');
cfg=record.configuration;h=cfg.controller.sampleTime;q=record.targetEpoch;
shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
for t0=holds
    k=find(abs([record.frame.time]-t0)<1e-9,1);f=record.frame(k);
    tt=linspace(0,h,2001);
    [~,s]=ode45(@(~,y)nonlinearBicycleModel.derivative(y,f.input,cfg),tt,f.state,odeset('RelTol',1e-11,'AbsTol',1e-12));
    d=zeros(size(tt));
    for j=1:numel(tt)
        p=predictiveSafetyGeometry.targetFlow(q,f.time+tt(j));
        d(j)=predictiveSafetyGeometry.rectangle(s(j,1:3).',shape,p(1:3),p(8:11));
    end
    rk=nonlinearBicycleModel.sample(f.state,f.input,cfg);mid=(f.state+rk)/2;
    pm=predictiveSafetyGeometry.targetFlow(q,f.time+h/2);
    dMidInterp=predictiveSafetyGeometry.rectangle(mid(1:3),shape,pm(1:3),pm(8:11));
    [~,jm]=min(abs(tt-h/2));
    affMid=(f.affineStates(:,1)+f.affineStates(:,2))/2;
    dAffMid=predictiveSafetyGeometry.rectangle(affMid(1:3),shape,pm(1:3),pm(8:11));
    pe=predictiveSafetyGeometry.targetFlow(q,f.time+h);
    dAffEnd=predictiveSafetyGeometry.rectangle(f.affineStates(1:3,2),shape,pe(1:3),pe(8:11));
    [dmin,jmin]=min(d);inside=tt(d<=0);
    fprintf('hold %.2f-%.2f s: u=[%.4f %.4f] yawRate=%.3f->%.3f rad/s\n',f.time,f.time+h,f.input,f.state(6),s(end,6));
    fprintf('  ODE45 clearance: start %.5f, true midpoint %.5f, end %.5f, minimum %.5f at +%.2f ms\n',d(1),d(jm),d(end),dmin,1e3*tt(jmin));
    if ~isempty(inside),fprintf('  strict overlap from +%.2f ms to +%.2f ms\n',1e3*inside(1),1e3*inside(end));end
    fprintf('  controller samples: affine start node %.5f, affine end node %.5f, affine interpolated midpoint %.5f; RK4 interpolated midpoint %.5f (margin %.3f)\n', ...
        d(1),dAffEnd,dAffMid,dMidInterp,cfg.collision.safetyMarginMeters);
    fprintf('  position gap true vs interpolated midpoint %.4f m, heading gap %.5f rad; one-step RK4 end error %.2e m\n', ...
        norm(s(jm,1:2).'-mid(1:2)),abs(s(jm,3)-mid(3)),norm(rk(1:2)-s(end,1:2).'));
end
