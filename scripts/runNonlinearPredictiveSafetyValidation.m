function report = runNonlinearPredictiveSafetyValidation(options)
%runNonlinearPredictiveSafetyValidation Deterministic nonlinear closed-loop audit.
% A tight ode45 replay checks each issued hold independently of the RK4 proposal.
% The audit is offline evidence; it is not an execution admission layer.
    arguments
        options.Frames (1,1) double {mustBeInteger,mustBePositive} = 8
        options.Scenarios (1,:) string = ["recovery","oncoming","circular","turningTarget"]
        options.OutputFile (1,1) string = ""
        options.ControllerConfiguration (1,1) struct = struct('referenceSpeed',8, ...
            'controller',struct('horizonSteps',8))
    end
    root=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(root,'controller'),fullfile(root,'config'));
    results=cell(1,numel(options.Scenarios));
    for index=1:numel(options.Scenarios)
        name=options.Scenarios(index);
        [ego,target,road,cfg]=localFixture(name,options.ControllerConfiguration);prior=[];
        frames=0;minimumClearance=Inf;minimumRoadMargin=Inf;maximumSeconds=0;failure="";
        firstClf=NaN;lastClf=NaN;maximumSlack=0;maxHorizon=0;warmStartedFrames=0;finalError=[];
        trace=struct([]);failedFrameSeconds=NaN;failureTime=NaN;passedTarget=false;
        for frame=1:options.Frames
            frameTimer=tic;
            try
                [command,~,problem,prior]=collisionAvoidanceController(ego,target,road,cfg,prior);
                frameSeconds=toc(frameTimer);
                maximumSeconds=max(maximumSeconds,frameSeconds);maxHorizon=max(maxHorizon,problem.metadata.horizonSteps);
                x=problem.model.initialState;input=command.actuatorInput;
                h=cfg.controller.sampleTime;
                [times,states]=ode45(@(~,state)nonlinearBicycleModel.derivative(state,input,cfg), ...
                    linspace(0,h,31),x,odeset('RelTol',1e-11,'AbsTol',1e-12));
                shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
                for j=1:numel(times)
                    q=predictiveSafetyGeometry.targetFlow(problem.model.target,times(j));
                    if ~isempty(q)
                        distance=predictiveSafetyGeometry.rectangle(states(j,1:3).',shape,q(1:3),q(7:10));
                        minimumClearance=min(minimumClearance,distance);
                    end
                    rotation=[cos(states(j,3)),-sin(states(j,3));sin(states(j,3)),cos(states(j,3))];
                    vertices=states(j,1:2).'+rotation*(shape(3:4)+shape(1:2).*[-1,1,1,-1;-1,-1,1,1]);
                    for vertex=1:4
                        projection=laneGeometry.project(vertices(:,vertex),problem.model.lane);
                        minimumRoadMargin=min([minimumRoadMargin;projection.lateralPosition+4;4-projection.lateralPosition]);
                    end
                end
                if frame==1,firstClf=problem.metadata.clfInitialValue;end
                lastClf=problem.metadata.clfNextValue;maximumSlack=max(maximumSlack,problem.metadata.clfSlack);
                next=states(end,:).';ego=localEgo(next,ego.stateTime+h,input);
                finalError=nonlinearBicycleModel.error(next,problem.model.lane,problem.model.terminal.reference);
                warmStartedFrames=warmStartedFrames+(problem.metadata.search.initialization=="shiftedWarmStart");
                search=problem.metadata.search;
                entry=struct('time',ego.stateTime-h,'controllerSeconds',frameSeconds, ...
                    'horizonSteps',problem.metadata.horizonSteps,'source',search.source, ...
                    'solverCalls',search.solverCalls,'initialization',search.initialization, ...
                    'scvxConverged',search.converged,'terminationReason',search.terminationReason, ...
                    'sequentialIterations',{search.sequentialIterations},'solverFailures',search.failures, ...
                    'predictiveBarrierValue',problem.metadata.predictiveBarrierValue, ...
                    'hardResidual',problem.solution.hard,'clfSlack',problem.metadata.clfSlack, ...
                    'state',x,'nextState',next,'input',input,'transverseError',finalError, ...
                    'auditTimes',times,'auditStates',states, ...
                    'predictionCollisionMargin',problem.metadata.minimumCollisionMargin);
                if isempty(trace),trace=entry;else,trace(end+1)=entry;end %#ok<AGROW>
                if ~isempty(target)
                    q=predictiveSafetyGeometry.targetFlow(problem.model.target,h);
                    target=localTarget(q);
                    passedTarget=passedTarget || next(1)-q(1)>cfg.vehicle.length/2+q(7);
                end
                frames=frames+1;
                if mod(frame,4)==0,fprintf('%s: %d/%d holds, last %.3f s, source %s\n', ...
                    name,frame,options.Frames,frameSeconds,search.source);end
            catch exception
                failedFrameSeconds=toc(frameTimer);failureTime=(frame-1)*cfg.controller.sampleTime;
                failure=string(exception.identifier)+": "+string(exception.message);break;
            end
        end
        seconds=[];solverCalls=0;
        if ~isempty(trace),seconds=[trace.controllerSeconds];solverCalls=sum([trace.solverCalls]);end
        later=seconds(2:end);
        firstSeconds=NaN;if ~isempty(seconds),firstSeconds=seconds(1);end
        medianLater=NaN;p95Later=NaN;
        if ~isempty(later),medianLater=median(later);p95Later=prctile(later,95);end
        results{index}=struct('scenario',name,'requestedFrames',options.Frames,'executedFrames',frames, ...
            'completed',frames==options.Frames,'failure',failure,'minimumReplayClearanceMeters',minimumClearance, ...
            'minimumReplayRoadMarginMeters',minimumRoadMargin,'maximumFrameSeconds',maximumSeconds, ...
            'maximumHorizonSteps',maxHorizon,'initialClfValue',firstClf,'finalClfValue',lastClf, ...
            'maximumClfSlack',maximumSlack,'sampleTimeSeconds',cfg.controller.sampleTime,'randomSeed',[], ...
            'finalTransverseError',finalError,'warmStartedFrames',warmStartedFrames, ...
            'requiredClearanceMeters',cfg.collision.safetyMarginMeters, ...
            'firstFrameSeconds',firstSeconds,'subsequentMedianSeconds',medianLater, ...
            'subsequentP95Seconds',p95Later,'deadlineMisses',nnz(seconds>cfg.controller.sampleTime), ...
            'failedFrameSeconds',failedFrameSeconds,'failureTime',failureTime,'passedTarget',passedTarget, ...
            'totalSolverCalls',solverCalls,'configuration',cfg,'trace',trace);
        fprintf('%s: %d/%d frames, max %.3f s, failure %s\n',name,frames,options.Frames,maximumSeconds,failure);
    end
    report=struct('model',"nonlinear combined-slip Fiala; one constant-speed/heading-rate target", ...
        'scope',"nominal PCBF/CLF/SCvx with independent offline replay", ...
        'stateObservation',"exact ego and target states; no observer, noise or delay", ...
        'replay',struct('integrator',"ode45",'relativeTolerance',1e-11, ...
            'absoluteTolerance',1e-12,'samplesPerHold',31),'results',[results{:}]);
    if strlength(options.OutputFile)>0
        file=fopen(options.OutputFile,'w');assert(file>=0);cleanup=onCleanup(@()fclose(file));
        fprintf(file,'%s\n',jsonencode(report));
    end
end

function [ego,target,road,cfg]=localFixture(name,controllerConfiguration)
    cfg=collisionAvoidanceControllerConfig(controllerConfiguration);
    road=struct('centerline',[-100,0;1000,0],'lateralClearance',[4;4]);
    x=[0;0;0;cfg.referenceSpeed;0;0];target=[];
    switch name
        case "recovery",x(2)=.01;
        case "oncoming"
            target=localTarget([24;0;pi;8;0;0;2.4;.95;0;0]);
        case "circular"
            road=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',.005,'length',200), ...
                'lateralClearance',[4;4]);
            trim=nonlinearBicycleModel.cruise(cfg,.005);x=trim.state;x(1:2)=0;
        case "turningTarget"
            target=localTarget([24;0;pi;8;0;-.8;2.4;.95;0;0]);
        otherwise,error('runNonlinearPredictiveSafetyValidation:unknownScenario','Unknown scenario %s.',name);
    end
    ego=localEgo(x,0,[0;0]);
end
function ego=localEgo(x,time,input)
    ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6), ...
        'stateTime',time,'heldActuatorInput',input);
end
function target=localTarget(q)
    velocity=q(4)*[cos(q(3)+q(5));sin(q(3)+q(5))];
    target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',velocity, ...
        'targetYawInertial',q(3),'targetYawRate',q(6),'targetSideslip',q(5), ...
        'targetAccelerationInertial',q(6)*[-velocity(2);velocity(1)],'targetLength',2*q(7), ...
        'targetWidth',2*q(8),'targetRectangleOffset',q(9:10));
end
