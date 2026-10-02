cd('/home/zai/Downloads/ResearchProjects/collisionAvoidance');addpath('controller','config','scripts','/home/zai/.cache/collisionAvoidance/unified-clf-implementation-20261001/expanded');
raw=jsondecode(fileread('/home/zai/.cache/collisionAvoidance/unified-clf-implementation-20261001/expanded/turningCrossing.json'));r=raw.results;
[~,epoch,road,cfg]=collisionThreatScenario('turningCrossing',struct('referenceSpeed',15,'controller',struct('horizonSteps',16),'collision',struct('safetyMarginMeters',.1)));prior=[];
rows=[];
for i=1:21
 f=r.trace(i);x=f.state;u=[0;0];if i>1,u=r.trace(i-1).input;end
 ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6),'heldActuatorInput',u,'stateTime',(i-1)*.05);
 q=predictiveSafetyGeometry.targetFlow(epoch,(i-1)*.05);direction=[cos(q(3)+q(6));sin(q(3)+q(6))];velocity=q(4)*direction;rate=q(4)*sin(q(6))/q(7);
 target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',velocity,'targetYawInertial',q(3),'targetYawRate',rate,'targetSideslip',q(6),'targetTangentialAcceleration',q(5),'targetRearAxleDistance',q(7),'targetAccelerationInertial',q(5)*direction+rate*[-velocity(2);velocity(1)]);
 [~,~,pred,prior]=collisionAvoidanceController(ego,target,road,cfg,prior);
 a=pred.model.linearization;z=pred.predictedState;controls=prior.inputTrajectory;tire=modifiedFialaTire.parameters(cfg);peak=0;where=0;
 for j=1:size(controls,2)
  tangent=modifiedFialaTire.affineModel(a.states(:,j),a.inputs(:,j),cfg);
  fy=tangent.force+tangent.state*(z(:,j)-a.states(:,j))+tangent.input*(controls(:,j)-a.inputs(:,j));
  ratio=max(hypot(fy./tire.longitudinalForceScale,controls(2,j)));
  if ratio>peak,peak=ratio;where=j;end
 end
 rows=[rows;(i-1)*.05,pred.metadata.search.primaryOptimum,peak,where];

end
disp(array2table(rows,'VariableNames',{'time','primary','maximumAffineForceRatio','stage'}));writematrix(rows,'/home/zai/.cache/collisionAvoidance/unified-clf-implementation-20261001/expanded/forceProbe.csv');
