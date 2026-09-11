function result = verifyFreeCompletionTime(outputDirectory)
%verifyFreeCompletionTime Exact affine-flow checks beyond configured windows.
% These declared-inclusion trials are not nonlinear or PassVeh14DOF experiments.
% Every continuation optimization is forced to fail after successful admission.
    arguments
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    result=struct();
    result.short=localTrial(2,13.5);
    result.onePointSix=localTrial(16,37);
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    save(fullfile(outputDirectory,'completion-replays.mat'),'result');
    file=fopen(fullfile(outputDirectory,'completion-replays.json'),'w');assert(file>=0);
    cleanup=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(result,PrettyPrint=true));
    disp(result.short);disp(result.onePointSix);
end

function result=localTrial(windowSteps,range)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('horizonSteps',windowSteps,'sampleTime',0.1), ...
        'model',struct('linearizationPolicy','cruise','lateralDomainRadius',2)));
    ego=struct('position',[0;0],'yaw',0,'speed',8,'stateTime',0, ...
        'perception',struct('time',0,'range',range,'completeWithinRange',true));
    target=struct('trackId',1,'targetPositionInertial',[-8;0], ...
        'targetVelocityInertial',[-8;0],'targetAccelerationInertial',[0;0], ...
        'targetHeadingInertial',pi,'targetYawRate',0, ...
        'predictionMotion',struct('kind','finite-sensing-motion-v1','jerkBound',[0;0],'yawAccelerationBound',0));
    route=[-100,0;2000,0];
    [command,~,problem,stored]=collisionAvoidanceController(ego,target,route,cfg,[]);
    count=stored.remainingSteps;
    result=struct('planningWindowSteps',windowSteps,'certificateSteps',count, ...
        'planningWindowSeconds',windowSteps*cfg.controller.sampleTime, ...
        'certificateSeconds',stored.certifiedDuration,'sensorRangeMeters',range, ...
        'searchAttempts',problem.metadata.certificateSearchAttempts, ...
        'scope',"exact declared affine flow; no nonlinear vehicle claim", ...
        'time',(0:count).'*cfg.controller.sampleTime,'input',zeros(2,count), ...
        'state',zeros(6,count+1),'margin',zeros(count+1,1),'completed',false, ...
        'minimumSampledClearance',inf,'continuationSolverCalls',zeros(count,1));
    result.state(:,1)=problem.model.initialEgoState;result.margin(1)=stored.margin;
    cfg.solver.jointFunction=@localFail;
    for stage=1:count
        input=command.actuatorInput;result.input(:,stage)=input;
        generator=[stored.prediction.continuousA(:,:,stage), ...
            stored.prediction.continuousB(:,:,stage),stored.prediction.continuousC(:,stage);zeros(3,9)];
        for tau=linspace(0,cfg.controller.sampleTime,41)
            state=expm(tau*generator)*[result.state(:,stage);input;1];
            [position,heading]=laneGeometry.fromFrenet(state(1:6),problem.model.lane);
            targetPosition=[-8-8*((stage-1)*cfg.controller.sampleTime+tau);0];
            clearance=avoidanceSafetyGeometry.rectangleDistance(position,heading,targetPosition,pi, ...
                [cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.target.defaultLength/2;cfg.target.defaultWidth/2]) ...
                -cfg.collision.clearanceMargin;
            result.minimumSampledClearance=min(result.minimumSampledClearance,clearance);
        end
        result.state(:,stage+1)=state(1:6);
        ego=struct('position',position,'yaw',heading,'speed',state(4), ...
            'lateralVelocity',state(5),'yawRate',state(6),'stateTime',result.time(stage+1), ...
            'heldActuatorInput',input,'perception',struct('time',result.time(stage+1), ...
                'range',range,'completeWithinRange',true));
        target.targetPositionInertial=targetPosition;
        [command,~,problem,stored]=collisionAvoidanceController(ego,target,route,cfg,stored);
        result.margin(stage+1)=stored.margin;
        result.continuationSolverCalls(stage)=problem.metadata.solverCallCount;
        if stage<count,assert(~isempty(command) && problem.metadata.planCertified);end
    end
    result.completed=stored.encounterComplete;
    result.finalCenterDistance=norm(ego.position-target.targetPositionInertial);
    assert(count>windowSteps && result.completed && isempty(command));
    assert(result.minimumSampledClearance>=0 && all(result.margin>=0));
    assert(result.finalCenterDistance>=range+cfg.encounter.perceptionExitBuffer);
end

function result=localFail(~,~)
    result=struct('decision',[],'exitFlag',-999,'output',struct());
end
