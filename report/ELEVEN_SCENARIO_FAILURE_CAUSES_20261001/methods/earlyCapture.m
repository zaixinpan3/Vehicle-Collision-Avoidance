root='/home/zai/.cache/collisionAvoidance/eleven-failure-causes-20261001';source='/home/zai/.cache/collisionAvoidance/direct-two-stage-scenarios-20261001/source';addpath(root);cd(source);addpath('controller','config','scripts');cases={};checks=struct([]);
for pair={{8,"acceleratingHeadOn",[41,101,201]}, {15,"headOn",[20,21,22]}, {15,"acceleratingHeadOn",[19,20,21]}}
    speed=pair{1}{1};name=pair{1}{2};selected=pair{1}{3};load(fullfile(fileparts(source),'campaign',"speed"+speed,name+".mat"),'continuation');c=continuation;
    [~,q0,road,cfg]=collisionThreatScenario(name,c.result.configuration);prior=[];maximumDifference=0;
    for frame=1:max(selected)
        tr=c.result.trace(frame);x=tr.state;previous=[0;0];if frame>1,previous=c.result.trace(frame-1).input;end
        ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6),'stateTime',tr.time,'heldActuatorInput',previous);
        q=predictiveSafetyGeometry.targetFlow(q0,tr.time);direction=[cos(q(3)+q(6));sin(q(3)+q(6))];velocity=q(4)*direction;w=q(4)*sin(q(6))/q(7);
        target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',velocity,'targetYawInertial',q(3),'targetYawRate',w,'targetSideslip',q(6),'targetTangentialAcceleration',q(5),'targetRearAxleDistance',q(7),'targetAccelerationInertial',q(5)*direction+w*[-velocity(2);velocity(1)],'targetLength',2*q(8),'targetWidth',2*q(9),'targetRectangleOffset',q(10:11));
        if ismember(frame,selected)
            captured=[];try,captureController(ego,target,road,cfg,prior);catch e,assert(strcmp(e.identifier,'diagnostic:captured'));end
            cases{end+1}=struct('speed',speed,'scenario',name,'time',tr.time,'frame',frame,'capture',captured,'prior',prior,'ego',ego,'target',target,'road',road,'trace',tr);
        end
        [command,~,prediction,prior]=collisionAvoidanceController(ego,target,road,cfg,prior);maximumDifference=max(maximumDifference,norm(command.actuatorInput-tr.input,inf));
    end
    entry=struct('speed',speed,'scenario',name,'replayedFrames',frame,'maximumInputDifference',maximumDifference);checks=[checks,entry];
    fprintf('REPLAY %g %s frames=%d maxInputDifference=%.9g\n',speed,name,frame,maximumDifference);
end
save(fullfile(root,'early-problems.mat'),'cases','checks');file=fopen(fullfile(root,'replay-checks.json'),'w');fprintf(file,'%s\n',jsonencode(checks));fclose(file);
