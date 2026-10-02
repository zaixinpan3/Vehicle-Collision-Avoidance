source='/home/zai/.cache/collisionAvoidance/unified-clf-implementation-20261001/assessment/source';
output='/home/zai/.cache/collisionAvoidance/collision-diagnosis-20261002';
cd(source);addpath('controller','config','scripts');
raw=jsondecode(fileread('/home/zai/.cache/collisionAvoidance/unified-clf-implementation-20261001/assessment/campaign/speed15/turningCrossing.json'));r=raw.results;
[~,epoch,road,cfg]=collisionThreatScenario('turningCrossing',struct('referenceSpeed',15,'controller',struct('horizonSteps',16),'collision',struct('safetyMarginMeters',.1)));prior=[];
frames=struct([]);nodes=[];
shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
tire=modifiedFialaTire.parameters(cfg);
for i=1:35
    f=r.trace(i);x=f.state;u=[0;0];if i>1,u=r.trace(i-1).input;end
    ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6),'heldActuatorInput',u,'stateTime',(i-1)*.05);
    q=predictiveSafetyGeometry.targetFlow(epoch,(i-1)*.05);direction=[cos(q(3)+q(6));sin(q(3)+q(6))];velocity=q(4)*direction;rate=q(4)*sin(q(6))/q(7);
    target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',velocity,'targetYawInertial',q(3),'targetYawRate',rate,'targetSideslip',q(6),'targetTangentialAcceleration',q(5),'targetRearAxleDistance',q(7),'targetAccelerationInertial',q(5)*direction+rate*[-velocity(2);velocity(1)]);
    [command,controls,pred,prior]=collisionAvoidanceController(ego,target,road,cfg,prior);
    a=pred.model.linearization;z=pred.predictedState;xn=x;
    frame=struct('time',f.time,'inputDifference',norm(controls(:,1)-f.input,inf),'search',pred.metadata.search,'slacks',pred.solution.stageSlacks,'input',controls(:,1),'state',x);
    if i>=14
        for j=1:min(24,size(controls,2))
            qt=predictiveSafetyGeometry.targetFlow(epoch,f.time+(j-1)*.05);
            projection=laneGeometry.project(a.states(1:2,j),pred.model.lane);
            dual=predictiveSafetyGeometry.dualLinearization(a.states(1:3,j),shape,qt(1:3),qt(8:11),[-sin(projection.heading);cos(projection.heading)]);
            linearMargin=min(dual.value+dual.jacobian*(z(1:3,j)-a.states(1:3,j)))-.1;
            affineGap=predictiveSafetyGeometry.rectangle(z(1:3,j),shape,qt(1:3),qt(8:11));
            nonlinearGap=predictiveSafetyGeometry.rectangle(xn(1:3),shape,qt(1:3),qt(8:11));
            tangent=modifiedFialaTire.affineModel(a.states(:,j),a.inputs(:,j),cfg);
            fy=tangent.force+tangent.state*(z(:,j)-a.states(:,j))+tangent.input*(controls(:,j)-a.inputs(:,j));
            ratio=max(hypot(fy./tire.longitudinalForceScale,controls(2,j)));
            slack=0;if j<=numel(frame.slacks),slack=frame.slacks(j);end
            nodes=[nodes;f.time,j,f.time+(j-1)*.05,linearMargin,affineGap,nonlinearGap,norm(z(1:2,j)-xn(1:2)),ratio,slack,z(:,j).',xn.',a.states(:,j).',controls(:,j).',a.inputs(:,j).'];
            xn=nonlinearBicycleModel.sample(xn,controls(:,j),cfg);
        end
        save(fullfile(output,sprintf('frame-%02d.mat',i)),'pred','prior','command');
    end
    if isempty(frames),frames=frame;else,frames(end+1)=frame;end
    fprintf('FRAME %.2f inputDifference %.3g primary %.9g lower %.9g source %s\n',f.time,frame.inputDifference,frame.search.primaryOptimum,frame.search.primaryLowerBound,frame.search.initialization);
end
fid=fopen(fullfile(output,'frames.json'),'w');fprintf(fid,'%s\n',jsonencode(frames));fclose(fid);
writematrix(nodes,fullfile(output,'nodes.csv'));
assert(max([frames.inputDifference])<1e-6,'Recorded command mismatch');
