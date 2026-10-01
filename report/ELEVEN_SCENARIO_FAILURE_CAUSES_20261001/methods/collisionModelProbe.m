root='/home/zai/.cache/collisionAvoidance/eleven-failure-causes-20261001';source='/home/zai/.cache/collisionAvoidance/direct-two-stage-scenarios-20261001/source';addpath(root);cd(source);addpath('controller','config','scripts');data=load(fullfile(root,'onset-problems.mat'));results=struct([]);rollouts=struct([]);
for j=1:numel(data.cases)
    c=data.cases{j};p=c.capture.problem;m=c.capture.model;cfg=m.cfg;anchor=c.capture.anchor;
    for mode=["base","noTerminalCone","noTailCollision","noStateTrust","noTrust","noClfCone"]
        q=p;
        if mode=="noTerminalCone",q.cones=q.cones(2);end
        if mode=="noClfCone",q.cones=q.cones(1);end
        if mode=="noTailCollision",remove=startsWith(p.rowGroups,"collision") & p.rowStages>cfg.controller.horizonSteps;q.a(remove,:)=[];q.b(remove)=[];end
        if any(mode==["noStateTrust","noTrust"])
            ix=p.stateIndices;q.lower(ix(:))=-Inf;q.upper(ix(:))=Inf;
            lo=[max(cfg.model.speedMinimum,cfg.model.scheduleSpeedFloor+1e-4);-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];hi=[cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
            q.lower(ix(4:6,2:end))=lo-anchor.states(4:6,2:end);q.upper(ix(4:6,2:end))=hi-anchor.states(4:6,2:end);
        end
        if mode=="noTrust"
            iu=p.inputIndices;lo=[-cfg.model.frontWheelSteeringAngleMaximum;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];hi=[cfg.model.frontWheelSteeringAngleMaximum;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];q.lower(iu)=lo-anchor.inputs;q.upper(iu)=hi-anchor.inputs;
        end
        [s,z]=runProbe(q,q.safetyObjective);entry=struct('speed',c.speed,'scenario',c.scenario,'time',c.time,'mode',mode,'stats',s);results=[results,entry];fprintf('COLLISION %s %.2f %s flag=%d safety=%.8g\n',c.scenario,c.time,mode,s.flag,s.safety);
        if mode=="base" && ~isempty(z)
            q.a=[q.a;q.safetyObjective.'];q.b=[q.b;sum(max(0,z(q.slackIndices)))+1e-6];o=zeros(size(q.lower));o(q.clfIndex)=1;[s,z]=runProbe(q,o);
            u=anchor.inputs+reshape(z(q.inputIndices),size(anchor.inputs));aff=anchor.states+reshape(z(q.stateIndices),size(anchor.states));x=m.initialState;states=x;gaps=[];shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];ok=true;failure="";
            for k=1:size(u,2)
                try
                    [t,xx]=ode45(@(~,x)nonlinearBicycleModel.derivative(x,u(:,k),cfg),[0,.025,.05],x,odeset('RelTol',1e-11,'AbsTol',1e-12));
                catch e,ok=false;failure=string(e.identifier);break;end
                for h=1:numel(t),target=predictiveSafetyGeometry.targetFlow(m.targetEpoch,(m.sampleIndex+k-1)*.05+t(h));gaps(end+1)=predictiveSafetyGeometry.rectangle(xx(h,1:3).',shape,target(1:3),target(8:11));end
                x=xx(end,:).';states(:,end+1)=x;
            end
            gapsAff=zeros(1,size(aff,2));for k=1:size(aff,2),target=predictiveSafetyGeometry.targetFlow(m.targetEpoch,(m.sampleIndex+k-1)*.05);gapsAff(k)=predictiveSafetyGeometry.rectangle(aff(1:3,k),shape,target(1:3),target(8:11));end
            entry=struct('speed',c.speed,'scenario',c.scenario,'time',c.time,'safety',s.safety,'affineNodeMinimumGap',min(gapsAff),'nonlinearRolloutMinimumGap',min(gaps),'rolloutComplete',ok,'failure',failure,'completedHolds',size(states,2)-1,'maxPositionError',max(vecnorm(states(1:2,:)-aff(1:2,1:size(states,2)))),'maxYawError',max(abs(states(3,:)-aff(3,1:size(states,2)))),'maxYawRateError',max(abs(states(6,:)-aff(6,1:size(states,2)))));rollouts=[rollouts,entry];
            fprintf('ROLLOUT %s %.2f affineGap=%.8g nonlinearGap=%.8g holds=%d maxPosition=%.7g\n',c.scenario,c.time,entry.affineNodeMinimumGap,entry.nonlinearRolloutMinimumGap,entry.completedHolds,entry.maxPositionError);
        end
    end
end
file=fopen(fullfile(root,'collision-model-probe.json'),'w');fprintf(file,'%s\n',jsonencode(struct('ablations',results,'rollouts',rollouts)));fclose(file);
