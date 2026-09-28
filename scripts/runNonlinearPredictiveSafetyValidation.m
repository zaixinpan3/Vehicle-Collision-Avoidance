function report = runNonlinearPredictiveSafetyValidation(options)
%runNonlinearPredictiveSafetyValidation Deterministic nonlinear closed-loop audit.
% A tight ode45 replay checks each issued hold independently of the RK4 proposal.
% Certification still comes from the directed interval verifier, not this audit.
    arguments
        options.Frames (1,1) double {mustBeInteger,mustBePositive} = 8
        options.Scenarios (1,:) string = ["recovery","oncoming","circular","solverFailure"]
        options.OutputFile (1,1) string = ""
        options.MaximumImprovementIterations (1,1) double {mustBeInteger,mustBeNonnegative} = 0
    end
    root=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(root,'controller'),fullfile(root,'config'));
    results=cell(1,numel(options.Scenarios));
    for index=1:numel(options.Scenarios)
        name=options.Scenarios(index);[ego,target,road,cfg]=localFixture(name);prior=[];
        if name~="solverFailure",cfg.nonlinear.maximumImprovementIterations=options.MaximumImprovementIterations;end
        frames=0;minimumClearance=Inf;maximumEnclosureViolation=0;maximumSeconds=0;failure="";
        firstClf=NaN;lastClf=NaN;maximumSlack=0;maxHorizon=0;inheritedFrames=0;backupFrames=0;finalError=[];
        trace=struct([]);failedFrameSeconds=NaN;failureTime=NaN;passedTarget=false;
        for frame=1:options.Frames
            frameTimer=tic;
            try
                [command,~,problem,prior]=collisionAvoidanceController(ego,target,road,cfg,prior);
                frameSeconds=toc(frameTimer);
                maximumSeconds=max(maximumSeconds,frameSeconds);maxHorizon=max(maxHorizon,problem.metadata.horizonSteps);
                x=problem.model.initialState;input=command.actuatorInput;
                sample=problem.certificate.samples{1};h=cfg.controller.sampleTime;
                [times,states]=ode45(@(~,state)nonlinearBicycleModel.derivative(state,input,cfg), ...
                    linspace(0,h,31),x,odeset('RelTol',1e-11,'AbsTol',1e-12));
                shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
                for j=1:numel(times)
                    q=nonlinearSafetyCertificate.targetFlow(problem.model.target,times(j));
                    if ~isempty(q)
                        distance=nonlinearSafetyCertificate.rectangle(states(j,1:3).',shape,q(1:3),q(7:10));
                        minimumClearance=min(minimumClearance,distance);
                    end
                    cellIndex=find([sample.cells.start]<=times(j) & [sample.cells.end]>=times(j),1);
                    box=sample.cells(cellIndex).swept(1:6,:);
                    maximumEnclosureViolation=max(maximumEnclosureViolation,max([box(:,1)-states(j,:).';states(j,:).'-box(:,2)]));
                end
                if frame==1,firstClf=problem.metadata.clfInitialValue;end
                lastClf=problem.metadata.clfNextValue;maximumSlack=max(maximumSlack,problem.metadata.clfSlack);
                next=states(end,:).';ego=localEgo(next,ego.stateTime+h,input);
                finalError=nonlinearBicycleModel.error(next,problem.model.lane,problem.model.backup.reference);
                inheritedFrames=inheritedFrames+problem.metadata.inheritedCertificate;
                backupFrames=backupFrames+problem.metadata.terminalBackupDispatched;
                search=problem.metadata.admissionSearch;
                entry=struct('time',ego.stateTime-h,'controllerSeconds',frameSeconds, ...
                    'horizonSteps',problem.metadata.horizonSteps,'source',search.source, ...
                    'solverCalls',search.solverCalls,'certifiedCandidates',search.certifiedCandidates, ...
                    'rejectedCandidates',search.rejectedCandidates,'inherited',problem.metadata.inheritedCertificate, ...
                    'backup',problem.metadata.terminalBackupDispatched,'clfSlack',problem.metadata.clfSlack, ...
                    'state',x,'nextState',next,'input',input,'transverseError',finalError, ...
                    'auditTimes',times,'auditStates',states, ...
                    'certificateCollisionMargin',problem.metadata.minimumCollisionMargin);
                if isempty(trace),trace=entry;else,trace(end+1)=entry;end %#ok<AGROW>
                if ~isempty(target)
                    q=nonlinearSafetyCertificate.targetFlow(problem.model.target,h);
                    target=localTarget(q);
                    passedTarget=passedTarget || next(1)-q(1)>cfg.vehicle.length/2+q(7);
                end
                frames=frames+1;
                if mod(frame,4)==0,fprintf('%s: %d/%d holds, last %.3f s, source %s\n', ...
                    name,frame,options.Frames,frameSeconds,search.source);end
                if name=="storedPolicy",cfg.solver.frameDeadlineSeconds=1e-12;end
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
            'maximumReplayEnclosureViolation',maximumEnclosureViolation,'maximumFrameSeconds',maximumSeconds, ...
            'maximumHorizonSteps',maxHorizon,'initialClfValue',firstClf,'finalClfValueUpper',lastClf, ...
            'maximumClfSlack',maximumSlack,'sampleTimeSeconds',cfg.controller.sampleTime,'randomSeed',[], ...
            'finalTransverseError',finalError,'inheritedFrames',inheritedFrames,'backupFrames',backupFrames, ...
            'requiredClearanceMeters',cfg.collision.safetyMarginMeters, ...
            'maximumImprovementIterations',cfg.nonlinear.maximumImprovementIterations, ...
            'firstFrameSeconds',firstSeconds,'subsequentMedianSeconds',medianLater, ...
            'subsequentP95Seconds',p95Later,'deadlineMisses',nnz(seconds>cfg.controller.sampleTime), ...
            'failedFrameSeconds',failedFrameSeconds,'failureTime',failureTime,'passedTarget',passedTarget, ...
            'totalSolverCalls',solverCalls,'trace',trace);
        fprintf('%s: %d/%d frames, max %.3f s, failure %s\n',name,frames,options.Frames,maximumSeconds,failure);
    end
    report=struct('model',"nonlinear combined-slip Fiala; one constant-speed/heading-rate target", ...
        'certificate',"directed interval flow and invariant cruise backup",'results',[results{:}]);
    if strlength(options.OutputFile)>0
        file=fopen(options.OutputFile,'w');assert(file>=0);cleanup=onCleanup(@()fclose(file));
        fprintf(file,'%s\n',jsonencode(report,PrettyPrint=true));
    end
end

function [ego,target,road,cfg]=localFixture(name)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('horizonSteps',8),'nonlinear',struct('maximumImprovementIterations',0)));
    road=struct('centerline',[-100,0;1000,0],'lateralClearance',[4;4]);
    x=[0;0;0;8;0;0];target=[];
    switch name
        case "recovery",x(2)=.01;
        case {"oncoming","storedPolicy"}
            target=localTarget([24;0;pi;8;0;0;2.4;.95;0;0]);
        case "circular"
            road=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',.005,'length',200), ...
                'lateralClearance',[4;4]);
            trim=nonlinearBicycleModel.cruise(cfg,.005);x=trim.state;x(1:2)=0;
        case "solverFailure"
            cfg.nonlinear.maximumImprovementIterations=1;cfg.nonlinear.proposalFunction=@localFail;
            cfg.solver.certificateSearchTimeLimit=100;
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
function plan=localFail(~,~)
    plan=[];
    error('validation:forcedSolverFailure','Deliberate improvement failure.');
end
