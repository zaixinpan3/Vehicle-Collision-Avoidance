% Overlap depth, timing and the offline interval certificate for one hold.
% Set name, speed, hold (frame time) before running.
source='/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/source';
addpath(fullfile(source,'controller'),fullfile(source,'config'),fullfile(source,'scripts'));
load(sprintf('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/replay/%s-%d.mat',name,speed),'record');
cfg=record.configuration;h=cfg.controller.sampleTime;q=record.targetEpoch;margin=cfg.collision.safetyMarginMeters;
shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
k=find(abs([record.frame.time]-hold)<1e-9,1);f=record.frame(k);
tt=linspace(0,h,5001);
[~,s]=ode45(@(~,y)nonlinearBicycleModel.derivative(y,f.input,cfg),tt,f.state,odeset('RelTol',1e-11,'AbsTol',1e-12));
depth=zeros(size(tt));distance=depth;relativeSpeed=depth;
for j=1:numel(tt)
    p=predictiveSafetyGeometry.targetFlow(q,f.time+tt(j));
    distance(j)=predictiveSafetyGeometry.rectangle(s(j,1:3).',shape,p(1:3),p(8:11));
    depth(j)=localPenetration(s(j,1:3).',shape,p(1:3),p(8:11));
    dx=nonlinearBicycleModel.derivative(s(j,:).',f.input,cfg);
    relativeSpeed(j)=norm(dx(1:2)-p(4)*[cos(p(3)+p(6));sin(p(3)+p(6))]);
end
inside=depth>0;
fprintf('%s %d hold %.2f-%.2f s: overlap %.2f ms (from +%.2f to +%.2f ms), maximum penetration %.2f mm at +%.2f ms; relative speed %.2f-%.2f m/s\n', ...
    name,speed,f.time,f.time+h,1e3*(nnz(inside)-1)*(tt(2)-tt(1)),1e3*tt(find(inside,1)),1e3*tt(find(inside,1,'last')), ...
    1e3*max(depth),1e3*tt(find(depth==max(depth),1)),min(relativeSpeed),max(relativeSpeed));
start=f.state(1:3);finish=s(end,1:3).';middle=s(round(end/2),1:3).';
whole=predictiveSafetyGeometry.intervalClearance(start,finish,shape,q,f.time,h,margin);
firstHalf=predictiveSafetyGeometry.intervalClearance(start,middle,shape,q,f.time,h/2,margin);
secondHalf=predictiveSafetyGeometry.intervalClearance(middle,finish,shape,q,f.time+h/2,h/2,margin);
fprintf('  offline intervalClearance on the node-to-node interpolant: certified=%d lowerBound=%.4f lipschitz=%.2f m/s cuts=%d minSampled=%.5f\n', ...
    whole.certified,whole.lowerBound,whole.lipschitz,numel(whole.fractions),whole.minimumSampledClearance);
fprintf('  first half: certified=%d minSampled=%.5f; second half: certified=%d minSampled=%.5f\n', ...
    firstHalf.certified,firstHalf.minimumSampledClearance,secondHalf.certified,secondHalf.minimumSampledClearance);
fprintf('  a sampled check every %.1f ms can miss a clearance drop of up to L*dt/2 = %.3f m (L from intervalClearance)\n', ...
    1e3*h/2,whole.lipschitz*h/4);
function d=localPenetration(poseE,shapeE,poseT,shapeT)
    % Separating-axis overlap: positive penetration depth when overlapping, else 0.
    re=[cos(poseE(3)),-sin(poseE(3));sin(poseE(3)),cos(poseE(3))];rt=[cos(poseT(3)),-sin(poseT(3));sin(poseT(3)),cos(poseT(3))];
    signs=[-1,1,1,-1;-1,-1,1,1];
    e=poseE(1:2)+re*(shapeE(3:4)+shapeE(1:2).*signs);t=poseT(1:2)+rt*(shapeT(3:4)+shapeT(1:2).*signs);
    d=Inf;
    for axis=[re,rt]
        pe=axis.'*e;pt=axis.'*t;d=min(d,min(max(pe),max(pt))-max(min(pe),min(pt)));
    end
    d=max(d,0);
end
