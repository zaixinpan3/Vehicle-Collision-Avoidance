source='/home/zai/.cache/collisionAvoidance/rti-controller-20261001/source';
output='/home/zai/.cache/collisionAvoidance/rti-controller-20261001';
cd(source);addpath('controller','config','scripts');records=struct([]);
for name=["headOn","acceleratingHeadOn"]
    saved=load(fullfile(output,'campaign','speed15',name+".mat"),'continuation');s=saved.continuation;r=s.result;
    cfg=r.configuration;[~,~,road]=collisionThreatScenario(name,cfg);prior=[];
    entry=struct('scenario',name,'frames',struct([]));shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    for index=1:24
        trace=r.trace(index);x=trace.state;
        ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6),'stateTime',trace.time,'heldActuatorInput',[0;0]);
        if index>1,ego.heldActuatorInput=r.trace(index-1).input;end
        target=[];
        if index==1
            q=r.targetInitialState;target=struct('targetPositionInertial',q(1:2),'targetYawInertial',q(3), ...
                'targetVelocityInertial',q(4)*[cos(q(3)+q(6));sin(q(3)+q(6))], ...
                'targetTangentialAcceleration',q(5),'targetSideslip',q(6),'targetRearAxleDistance',q(7));
        end
        [~,~,p,prior]=collisionAvoidanceController(ego,target,road,cfg,prior);
        if index<19,continue;end
        u=p.inputTrajectory(:,1);anchor=p.model.linearization;
        [mid,am,bm]=nonlinearBicycleModel.hold(anchor.states(:,1),anchor.inputs(:,1),cfg);
        affineMid=mid+am*(x-anchor.states(:,1))+bm*(u-anchor.inputs(:,1));
        nonlinearMid=nonlinearBicycleModel.sample(x,u,cfg,[],cfg.controller.sampleTime/2);
        nonlinearNext=nonlinearBicycleModel.sample(x,u,cfg);
        gap=zeros(3,3);nodeStates=cat(3,[x,affineMid,p.predictedState(:,2)], ...
            [x,nonlinearMid,nonlinearNext],[x,trace.auditStates(16,:).',trace.auditStates(31,:).']);
        for prediction=1:3
            for k=1:3
                q=predictiveSafetyGeometry.targetFlow(r.targetInitialState,trace.time+(k-1)*cfg.controller.sampleTime/2);
                gap(prediction,k)=predictiveSafetyGeometry.rectangle(nodeStates(1:3,k,prediction),shape,q(1:3),q(8:11));
            end
        end
        row=struct('time',trace.time,'inputDifference',norm(u-trace.input,inf),'gapsByAffineRk4Ode',gap, ...
            'nextPredictionError',nonlinearNext-p.predictedState(:,2),'midPredictionError',nonlinearMid-affineMid, ...
            'anchorFirstInput',anchor.inputs(:,1),'input',u,'slack',p.solution.safety, ...
            'flags',[p.metadata.search.stages.exitFlag]);
        if isempty(entry.frames),entry.frames=row;else,entry.frames(end+1)=row;end
    end
    if isempty(records),records=entry;else,records(end+1)=entry;end
end
f=fopen(fullfile(output,'collision-probes.json'),'w');fprintf(f,'%s\n',jsonencode(records));fclose(f);
