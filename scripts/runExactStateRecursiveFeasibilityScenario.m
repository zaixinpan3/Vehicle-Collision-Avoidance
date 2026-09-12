function report = runExactStateRecursiveFeasibilityScenario(options)
%runExactStateRecursiveFeasibilityScenario Exact scheduled-plant regression.
% The plant executes each published continuous generator exactly with expm.
% This is not a nonlinear Fiala vehicle or perception-pipeline experiment.
    arguments
        options.Scenario (1,1) string {mustBeMember(options.Scenario,["stationary","oncoming","crossing"])} = "stationary"
        options.SampleCount (1,1) double {mustBeInteger,mustBePositive} = 120
        options.FailAfterAdmission (1,1) logical = false
        options.OutputDirectory (1,1) string = ""
        options.DeadlineSeconds (1,1) double {mustBePositive} = 0.1
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"controller"),fullfile(root,"config"));
    cfg = collisionAvoidanceControllerConfig(struct("referenceSpeed",8, ...
        "controller",struct("sampleTime",0.1,"horizonSteps",16), ...
        "model",struct("lateralDomainRadius",4), ...
        "solver",struct("certificateSearchTimeLimit",30)));
    frameTimer = tic;
    ego = struct("position",[0;0],"yaw",0,"speed",8,"stateTime",0);
    target = struct("trackId",1,"targetPositionInertial",[15;0], ...
        "targetVelocityInertial",[0;0],"targetAccelerationInertial",[0;0], ...
        "targetHeadingInertial",0,"targetYawRate",0);
    if options.Scenario=="oncoming"
        target.targetPositionInertial = [60;0];
        target.targetVelocityInertial = [-8;0];
        target.targetHeadingInertial = pi;
    elseif options.Scenario=="crossing"
        target.targetPositionInertial = [15;-4];
        target.targetVelocityInertial = [0;32];
        target.targetHeadingInertial = pi/2;
    end
    boundary = struct("origin",zeros(2,1),"longitudinalDirection",[1;0], ...
        "lateralDirection",[0;1],"coefficients",[0;0;-5], ...
        "parameterRange",[-100;2000],"safeSideSign",1);
    boundaries = [boundary;boundary];
    boundaries(2).coefficients(3) = 5;
    boundaries(2).safeSideSign = -1;
    road = struct("centerline",[-100,0;2000,0],"boundaries",boundaries);
    try
        [command,~,problem,certificate] = collisionAvoidanceController(ego,target,road,cfg,[]);
    catch exception
        report = struct("scenario",options.Scenario,"configuration",cfg, ...
            "passed",false,"completed",false,"failureIdentifier",string(exception.identifier), ...
            "failureMessage",string(exception.message),"executedHolds",0, ...
            "sampleCount",options.SampleCount,"failureTime",0,"runtimeQualified",false, ...
            "admissionSeconds",toc(frameTimer));
        localSave(report,options);
        return;
    end
    frameSeconds = zeros(1,options.SampleCount+1);
    frameSeconds(1) = toc(frameTimer);
    admissionSteps = certificate.prediction.stageCount;
    admissionMargin = certificate.margin;
    initialTarget = certificate.encounters;
    terminal = certificate.qp.terminal;
    x = problem.model.initialEgoState;
    states = zeros(6,options.SampleCount+1);
    states(:,1) = x;
    inputs = zeros(2,options.SampleCount+1);
    inputs(:,1) = command.actuatorInput;
    certified = true(1,options.SampleCount+1);
    terminalActive = false(size(certified));
    retained = false(size(certified));
    shiftedCandidateFeasible = false(size(certified));
    horizonSteps = zeros(size(certified));
    horizonSteps(1) = certificate.remainingSteps;
    predictionEndTime = zeros(size(certified));
    predictionEndTime(1) = certificate.deadline;
    phaseSeconds = zeros(4,numel(certified));
    phaseSeconds(:,1) = localPhases(problem.metadata.runtime);
    failureIdentifier = "";
    failureMessage = "";
    minimumSeparation = inf;
    minimumRoadMargin = inf;
    maximumSlewViolation = 0;
    originalCfg = cfg;
    if options.FailAfterAdmission, cfg.solver.jointFunction = @localFailedSolve; end
    for sample = 1:options.SampleCount
        generator = [problem.metadata.executedContinuousGenerator;zeros(3,9)];
        % Dense geometry checks are an independent numerical audit of the
        % analytic prefix and invariant-tail certificates, not their proof.
        for fraction = linspace(0,1,11)
            elapsed = fraction*cfg.controller.sampleTime;
            value = expm(elapsed*generator)*[x;command.actuatorInput;1];
            [position,heading] = laneGeometry.fromFrenet(value(1:6),problem.model.lane);
            targetState = targetPrediction.finiteFlow(initialTarget, ...
                (sample-1)*cfg.controller.sampleTime+elapsed);
            separation = avoidanceSafetyGeometry.rectangleDistance(position,heading, ...
                targetState(1:2),targetState(7),[cfg.vehicle.length/2;cfg.vehicle.width/2; ...
                initialTarget.halfLength;initialTarget.halfWidth]);
            minimumSeparation = min(minimumSeparation,separation-cfg.collision.clearanceMargin);
            lateralSupport = cfg.vehicle.length/2*abs(sin(heading))+cfg.vehicle.width/2*abs(cos(heading));
            minimumRoadMargin = min(minimumRoadMargin,5-abs(position(2))-lateralSupport-cfg.collision.clearanceMargin);
        end
        x = value(1:6);
        time = sample*cfg.controller.sampleTime;
        frameTimer = tic;
        ego = struct("position",position,"yaw",heading,"speed",x(4), ...
            "lateralVelocity",x(5),"yawRate",x(6),"stateTime",time, ...
            "heldActuatorInput",command.actuatorInput);
        target.targetPositionInertial = targetState(1:2);
        target.targetVelocityInertial = targetState(3:4);
        target.targetAccelerationInertial = targetState(5:6);
        target.targetHeadingInertial = targetState(7);
        target.targetYawRate = targetState(8);
        previous = command.actuatorInput;
        states(:,sample+1) = x;
        try
            [command,~,problem,certificate] = collisionAvoidanceController(ego,target,road,cfg,certificate);
        catch exception
            frameSeconds(sample+1) = toc(frameTimer);
            certified(sample+1) = false;
            inputs(:,sample+1) = NaN;
            failureIdentifier = string(exception.identifier);
            failureMessage = string(exception.message);
            break;
        end
        horizonSteps(sample+1) = certificate.remainingSteps;
        predictionEndTime(sample+1) = certificate.deadline;
        phaseSeconds(:,sample+1) = localPhases(problem.metadata.runtime);
        frameSeconds(sample+1) = toc(frameTimer);
        states(:,sample+1) = x;
        inputs(:,sample+1) = command.actuatorInput;
        certified(sample+1) = problem.metadata.planCertified;
        terminalActive(sample+1) = problem.metadata.terminalActive;
        retained(sample+1) = problem.metadata.fallbackUsed;
        shiftedCandidateFeasible(sample+1) = problem.metadata.carriedWitnessFeasible;
        limit = cfg.controller.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
        maximumSlewViolation = max([maximumSlewViolation;abs(command.actuatorInput-previous)-limit]);
        if mod(sample,25)==0
            fprintf('%s progress: %d/%d holds, speed %.6g m/s, horizon %d, last frame %.3f s\n', ...
                options.Scenario,sample,options.SampleCount,x(4),certificate.remainingSteps,frameSeconds(sample+1));
            if strlength(options.OutputDirectory)>0
                if ~isfolder(options.OutputDirectory), mkdir(options.OutputDirectory); end
                progress = struct("executedHolds",sample,"state",states(:,1:sample+1), ...
                    "frameSeconds",frameSeconds(1:sample+1),"horizonSteps",horizonSteps(1:sample+1), ...
                    "minimumSeparation",minimumSeparation,"minimumRoadMargin",minimumRoadMargin);
                save(fullfile(options.OutputDirectory,options.Scenario+"-progress.mat"),"progress");
            end
        end
    end
    issued = 1:sample+1;
    frameSeconds = frameSeconds(issued);
    states = states(:,issued); inputs = inputs(:,issued);
    certified = certified(issued); terminalActive = terminalActive(issued); retained = retained(issued);
    report = struct("scenario",options.Scenario,"configuration",originalCfg, ...
        "sampleCount",options.SampleCount,"time",(0:sample)*cfg.controller.sampleTime, ...
        "state",states,"input",inputs,"admissionSteps",admissionSteps,"admissionMargin",admissionMargin, ...
        "planCertified",certified,"terminalActive",terminalActive,"retainedWitnessUsed",retained, ...
        "minimumSampledSeparationMargin",minimumSeparation,"minimumSampledRoadMargin",minimumRoadMargin, ...
        "maximumSlewViolation",maximumSlewViolation,"terminal",terminal, ...
        "finalTargetDistance",norm(ego.position-target.targetPositionInertial), ...
        "solverFailureInjected",options.FailAfterAdmission, ...
        "scope","Each executed hold follows its published affine predictor; terminal feedback is prediction-only; no global recursive-feasibility or nonlinear-vehicle claim");
    report.completed = strlength(failureIdentifier)==0;
    report.failureIdentifier = failureIdentifier;
    report.failureMessage = failureMessage;
    report.executedHolds = sample;
    report.horizonSteps = horizonSteps(issued);
    report.shiftedCandidateFeasible = shiftedCandidateFeasible(issued);
    report.predictionEndTime = predictionEndTime(issued);
    report.phaseSeconds = phaseSeconds(:,issued);
    report.phaseNames = ["prediction","formulation","solve","acceptance"];
    report.finalCruiseError = states(2:6,end)-[0;0;cfg.referenceSpeed;0;0];
    report.passed = report.completed && all(certified) && ~any(terminalActive) && ~any(retained) && minimumSeparation>=0 ...
        && minimumRoadMargin>=0 && maximumSlewViolation<=0;
    report.runtime = struct("frameSeconds",frameSeconds, ...
        "deadlineSeconds",options.DeadlineSeconds, ...
        "admissionSeconds",frameSeconds(1), ...
        "maximumSuccessorSeconds",max(frameSeconds(2:end)), ...
        "deadlineMisses",nnz(frameSeconds>options.DeadlineSeconds), ...
        "deadlineMet",all(frameSeconds<=options.DeadlineSeconds), ...
        "scope","Input assembly and controller calls; excludes plant integration and geometry audit; diagnostic execution continues after deadline misses");
    report.runtimeQualified = report.passed && report.runtime.deadlineMet;
    localSave(report,options);
    fprintf('%s: %d/%d certified commands; %d executed holds; separation %.6g m; road %.6g m; final speed %.6g m/s\n', ...
        options.Scenario,nnz(certified),numel(certified),sample,minimumSeparation,minimumRoadMargin,states(4,end));
    if ~report.completed, fprintf('Control failed: %s\n',failureMessage); end

end

function result = localFailedSolve(~,~)
    result = struct("decision",[],"exitFlag",-999,"output",struct());
end

function values = localPhases(runtime)
    values = [runtime.predictionSeconds;runtime.formulationAndWitnessSeconds; ...
        runtime.solveSeconds;runtime.acceptanceAndCommitSeconds];
end

function localSave(report,options)
    if strlength(options.OutputDirectory)==0, return; end
    if ~isfolder(options.OutputDirectory), mkdir(options.OutputDirectory); end
    save(fullfile(options.OutputDirectory,options.Scenario+"-exact-state.mat"),"report");
    file = fopen(fullfile(options.OutputDirectory,options.Scenario+"-exact-state.json"),'w');
    cleanup = onCleanup(@() fclose(file));
    fprintf(file,'%s\n',jsonencode(report,PrettyPrint=true));
end
