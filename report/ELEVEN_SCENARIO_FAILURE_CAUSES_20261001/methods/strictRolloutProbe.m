root='/home/zai/.cache/collisionAvoidance/eleven-failure-causes-20261001';source='/home/zai/.cache/collisionAvoidance/direct-two-stage-scenarios-20261001/source';addpath(root);cd(source);addpath('controller','config','scripts');data=load(fullfile(root,'onset-problems.mat'));c=data.cases{1};q=c.capture.problem;m=c.capture.model;cfg=m.cfg;a=c.capture.anchor;
[s,z]=runProbe(q,q.safetyObjective);q.a=[q.a;q.safetyObjective.'];q.b=[q.b;sum(max(0,z(q.slackIndices)))+1e-6];o=zeros(size(q.lower));o(q.clfIndex)=1;[s,z]=runProbe(q,o);u=a.inputs+reshape(z(q.inputIndices),size(a.inputs));x=m.initialState;minimum=Inf;at=NaN;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
for k=1:size(u,2)
    [t,xx]=ode45(@(~,x)nonlinearBicycleModel.derivative(x,u(:,k),cfg),[0,.025,.05],x,odeset('RelTol',1e-11,'AbsTol',1e-12));
    for j=1:numel(t)
        time=(m.sampleIndex+k-1)*.05+t(j);target=predictiveSafetyGeometry.targetFlow(m.targetEpoch,time);yaw=xx(j,3);re=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];rt=[cos(target(3)),-sin(target(3));sin(target(3)),cos(target(3))];
        vertices=xx(j,1:2).'+re*(shape(3:4)+shape(1:2).*[-1,1,1,-1;-1,-1,1,1]);tv=target(1:2)+rt*(target(10:11)+target(8:9).*[-1,1,1,-1;-1,-1,1,1]);axes=[re,rt];aa=axes.'*vertices;bb=axes.'*tv;gap=max(max(min(aa,[],2)-max(bb,[],2),min(bb,[],2)-max(aa,[],2)));
        if gap<minimum,minimum=gap;at=time;end
    end
    x=xx(end,:).';
end
entry=struct('scenario',c.scenario,'callTime',c.time,'strictOverlap',minimum< -1e-9,'minimumSeparatingAxisGapMeters',minimum,'absoluteTimeSeconds',at,'scope','Counterfactual nonlinear execution of the complete saved-frame optimized input sequence; sample starts, midpoints and ends');
fprintf('STRICT-ROLLOUT %s time %.3f axisGap=%.9g at=%.3f\n',c.scenario,c.time,minimum,at);file=fopen(fullfile(root,'strict-rollout.json'),'w');fprintf(file,'%s\n',jsonencode(entry));fclose(file);
