function report = runExactStateRecursiveFeasibilityScenario(options)
%runExactStateRecursiveFeasibilityScenario Declared-plant recursive-feasibility regression.
% The plant executes each accepted plan's first-stage continuous generator
% exactly with expm from the TRUE state. Ego and target measurements are the
% truth plus uniform noise inside the declared error boxes; the controller
% receives the noisy estimates with those boxes. Optional sinusoidal jerk
% and yaw acceleration perturb the nominal target law within supplied bounds.
% Truth is integrated analytically, independently of controller propagation.
% This is not a nonlinear Fiala vehicle
% or perception-pipeline experiment.
    arguments
        options.Scenario (1,1) string {mustBeMember(options.Scenario,["stationary","oncoming","crossing"])} = "stationary"
        options.SampleCount (1,1) double {mustBeInteger,mustBePositive} = 120
        options.FailAfterAdmission (1,1) logical = false
        options.OutputDirectory (1,1) string = ""
        options.DeadlineSeconds (1,1) double {mustBePositive} = 0.1
        options.EgoErrorBound (6,1) double {mustBeNonnegative,mustBeFinite} = zeros(6,1)
        options.TargetErrorBound (8,1) double {mustBeNonnegative,mustBeFinite} = zeros(8,1)
        options.Seed (1,1) double {mustBeInteger,mustBeNonnegative} = 20260912
        options.MinimumHorizonSteps (1,1) double {mustBeInteger,mustBePositive} = 2
        options.TargetJerkAmplitude (2,1) double {mustBeFinite} = zeros(2,1)
        options.TargetYawAccelerationAmplitude (1,1) double {mustBeFinite} = 0
        options.TargetMotionFrequency (1,1) double {mustBeFinite,mustBePositive} = 1
        options.ConfirmationRange (1,1) double {mustBeFinite,mustBePositive} = 16
        options.ExecutionPolicy (1,1) string {mustBeMember(options.ExecutionPolicy,["auto","backup","predictive"])} = "auto"
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"controller"),fullfile(root,"config"));
    cfg = collisionAvoidanceControllerConfig(struct("referenceSpeed",8, ...
        "controller",struct("sampleTime",0.1,"horizonSteps",16,"minimumHorizonSteps",options.MinimumHorizonSteps, ...
            "executionPolicy",options.ExecutionPolicy), ...
        "model",struct("lateralDomainRadius",4), ...
        "solver",struct("certificateSearchTimeLimit",30,"frameDeadlineSeconds",options.DeadlineSeconds)));
    stream = RandStream("mt19937ar",Seed=options.Seed);
    h = cfg.controller.sampleTime;
    egoBound = options.EgoErrorBound;
    targetBound = options.TargetErrorBound;
    truthTarget = struct("center",[15;0;0;0;0;0;0;0],"radius",zeros(8,1), ...
        "jerkAmplitude",options.TargetJerkAmplitude, ...
        "yawAccelerationAmplitude",options.TargetYawAccelerationAmplitude, ...
        "frequency",options.TargetMotionFrequency, ...
        "halfLength",cfg.target.defaultLength/2,"halfWidth",cfg.target.defaultWidth/2);
    if options.Scenario=="oncoming"
        truthTarget.center = [60;0;-8;0;0;0;pi;0];
    elseif options.Scenario=="crossing"
        truthTarget.center = [15;-4;0;32;0;0;pi/2;0];
    end
    boundary = struct("origin",zeros(2,1),"longitudinalDirection",[1;0], ...
        "lateralDirection",[0;1],"coefficients",[0;0;-5], ...
        "parameterRange",[-100;2000],"safeSideSign",1);
    boundaries = [boundary;boundary];
    boundaries(2).coefficients(3) = 5;
    boundaries(2).safeSideSign = -1;
    road = struct("centerline",[-100,0;2000,0],"boundaries",boundaries);
    lane = [];
    % True Cartesian state [px; py; psi; vx; vy; r] before admission; the
    % Frenet truth is derived from it once the lane chart is known.
    x = [0;0;0;8;0;0];
    frameTimer = tic;
    ego = localEgoMeasurement(x,0,[],egoBound,stream,lane);
    ego.perception = struct("time",0,"range",options.ConfirmationRange,"completeWithinRange",true);
    target = localTargetMeasurement(truthTarget,0,targetBound,stream);
    try
        [command,~,problem,certificate] = collisionAvoidanceController(ego,target,road,cfg,[]);
    catch exception
        report = struct("scenario",options.Scenario,"configuration",cfg, ...
            "passed",false,"completed",false,"failureIdentifier",string(exception.identifier), ...
            "failureMessage",string(exception.message),"executedHolds",0, ...
            "sampleCount",options.SampleCount,"failureTime",0,"runtimeQualified",false, ...
            "admissionSeconds",toc(frameTimer),"egoErrorBound",egoBound,"targetErrorBound",targetBound, ...
            "targetMotion",truthTarget,"seed",options.Seed, ...
            "confirmationRange",options.ConfirmationRange,"solverFailureInjected",options.FailAfterAdmission);
        localSave(report,options);
        return;
    end
    lane = problem.model.lane;
    projection = laneGeometry.project(x(1:2),lane);
    x = [projection.station;projection.lateralPosition; ...
        atan2(sin(x(3)-projection.heading),cos(x(3)-projection.heading));x(4:6)];
    count = options.SampleCount+1;
    frameSeconds = zeros(1,count);
    frameSeconds(1) = toc(frameTimer);
    admissionSteps = certificate.horizonSteps;
    admissionMargin = certificate.margin;
    terminal = certificate.terminal;
    states = zeros(6,count);
    states(:,1) = x;
    inputs = zeros(2,count);
    inputs(:,1) = command.actuatorInput;
    certified = true(1,count);
    terminalActive = false(1,count);
    candidateExecuted = false(1,count);
    candidateVerified = false(1,count);
    certificateSource = strings(1,count);
    certificateSource(1) = problem.metadata.certificateSource;
    pcbfValue = zeros(1,count);
    pcbfValue(1) = problem.metadata.pcbfValue;
    stageZeroViolation = zeros(1,count);
    stageZeroViolation(1) = localFirst(problem.metadata.stageViolation);
    descentResidual = nan(1,count);
    optimizedStages = zeros(1,count);
    optimizedStages(1) = certificate.remainingSteps;
    horizonSteps = zeros(1,count);
    horizonSteps(1) = certificate.horizonSteps;
    predictionEndTime = zeros(1,count);
    predictionEndTime(1) = certificate.deadline;
    exitDeadline = nan(1,count);
    exitDeadline(1) = problem.metadata.targetCertifiedUntil;
    releaseConfirmed = false(1,count);
    egoRadius = zeros(6,count);
    egoRadius(:,1) = problem.metadata.initialErrorBound;
    targetRadius = zeros(8,count);
    targetRadius(:,1) = problem.metadata.targetErrorBound;
    egoContainment = nan(1,count);
    targetContainment = nan(1,count);
    [egoContainment(1),targetContainment(1)] = localContainment(x, ...
        localTargetTruth(truthTarget,0),problem,certificate);
    admissionFrame = false(1,count);
    admissionFrame(1) = true;
    witnessSeconds = zeros(1,count);
    phaseSeconds = zeros(4,count);
    phaseSeconds(:,1) = localPhases(problem.metadata.runtime);
    failureIdentifier = "";
    failureMessage = "";
    minimumSeparation = inf;
    minimumRoadMargin = inf;
    minimumDomainMargin = inf;
    minimumSpeed = x(4);
    maximumSlewViolation = 0;
    originalCfg = cfg;
    if options.FailAfterAdmission, cfg.solver.jointFunction = @localFailedSolve; end
    for sample = 1:options.SampleCount
        generator = [problem.metadata.executedContinuousGenerator;zeros(3,9)];
        % Dense geometry checks on the TRUE state are an independent numerical
        % audit of the verified prefix and terminal certificates, not their proof.
        for fraction = linspace(0,1,11)
            elapsed = fraction*h;
            value = expm(elapsed*generator)*[x;command.actuatorInput;1];
            [position,heading] = laneGeometry.fromFrenet(value(1:6),lane);
            targetState = localTargetTruth(truthTarget,(sample-1)*h+elapsed);
            separation = avoidanceSafetyGeometry.rectangleDistance(position,heading, ...
                targetState(1:2),targetState(7),[cfg.vehicle.length/2;cfg.vehicle.width/2; ...
                truthTarget.halfLength;truthTarget.halfWidth]);
            minimumSeparation = min(minimumSeparation,separation-cfg.collision.clearanceMargin);
            lateralSupport = cfg.vehicle.length/2*abs(sin(heading))+cfg.vehicle.width/2*abs(cos(heading));
            minimumRoadMargin = min(minimumRoadMargin,5-abs(position(2))-lateralSupport-cfg.collision.clearanceMargin);
            minimumSpeed = min(minimumSpeed,value(4));
            minimumDomainMargin = min(minimumDomainMargin,localDomainMargin(value(1:6),cfg));
        end
        x = value(1:6);
        time = sample*h;
        frameTimer = tic;
        ego = localEgoMeasurement(x,time,command.actuatorInput,egoBound,stream,lane);
        ego.perception = struct("time",time,"range",options.ConfirmationRange,"completeWithinRange",true);
        target = localTargetMeasurement(truthTarget,time,targetBound,stream);
        priorSuccessorMargin = min(certificate.stateErrorBound(:,2)-abs(x-certificate.predictedState(:,2)));
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
        metadata = problem.metadata;
        frameSeconds(sample+1) = toc(frameTimer);
        inputs(:,sample+1) = command.actuatorInput;
        certified(sample+1) = metadata.planCertified;
        terminalActive(sample+1) = metadata.terminalActive;
        certificateSource(sample+1) = metadata.certificateSource;
        candidateExecuted(sample+1) = metadata.certificateSource=="carriedWitness";
        candidateVerified(sample+1) = metadata.candidateVerified;
        pcbfValue(sample+1) = metadata.pcbfValue;
        stageZeroViolation(sample+1) = localFirst(metadata.stageViolation);
        descentResidual(sample+1) = metadata.pcbfDescentResidual;
        optimizedStages(sample+1) = certificate.remainingSteps;
        horizonSteps(sample+1) = certificate.horizonSteps;
        predictionEndTime(sample+1) = certificate.deadline;
        exitDeadline(sample+1) = metadata.targetCertifiedUntil;
        releaseConfirmed(sample+1) = metadata.confirmedRelease;
        admissionFrame(sample+1) = metadata.jointAdmissionPerformed;
        [egoContainment(sample+1),targetContainment(sample+1)] = localContainment( ...
            x,localTargetTruth(truthTarget,time),problem,certificate);
        egoContainment(sample+1) = min(egoContainment(sample+1),priorSuccessorMargin);
        egoRadius(:,sample+1) = metadata.initialErrorBound;
        if metadata.hasTarget, targetRadius(:,sample+1) = metadata.targetErrorBound; end
        witnessSeconds(sample+1) = metadata.candidateSeconds;
        phaseSeconds(:,sample+1) = localPhases(metadata.runtime);
        limit = h*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
        maximumSlewViolation = max([maximumSlewViolation;abs(command.actuatorInput-previous)-limit]);
        if mod(sample,25)==0
            fprintf('%s progress: %d/%d holds, speed %.6g m/s, value %.3g, source %s, last frame %.3f s\n', ...
                options.Scenario,sample,options.SampleCount,x(4),metadata.pcbfValue, ...
                metadata.certificateSource,frameSeconds(sample+1));
            if strlength(options.OutputDirectory)>0
                if ~isfolder(options.OutputDirectory), mkdir(options.OutputDirectory); end
                progress = struct("executedHolds",sample,"state",states(:,1:sample+1), ...
                    "frameSeconds",frameSeconds(1:sample+1),"pcbfValue",pcbfValue(1:sample+1), ...
                    "minimumSeparation",minimumSeparation,"minimumRoadMargin",minimumRoadMargin);
                save(fullfile(options.OutputDirectory,options.Scenario+"-progress.mat"),"progress");
            end
        end
    end
    issued = 1:sample+1;
    frameSeconds = frameSeconds(issued);
    states = states(:,issued); inputs = inputs(:,issued);
    certified = certified(issued); terminalActive = terminalActive(issued);
    candidateExecuted = candidateExecuted(issued);
    report = struct("scenario",options.Scenario,"configuration",originalCfg, ...
        "sampleCount",options.SampleCount,"time",(0:sample)*h,"seed",options.Seed, ...
        "egoErrorBound",egoBound,"targetErrorBound",targetBound,"targetMotion",truthTarget, ...
        "state",states,"input",inputs,"admissionSteps",admissionSteps,"admissionMargin",admissionMargin, ...
        "planCertified",certified,"terminalActive",terminalActive,"retainedWitnessUsed",candidateExecuted, ...
        "candidateExecuted",candidateExecuted,"candidateVerified",candidateVerified(issued), ...
        "certificateSource",certificateSource(issued),"pcbfValue",pcbfValue(issued), ...
        "stageZeroViolation",stageZeroViolation(issued),"descentResidual",descentResidual(issued), ...
        "optimizedStages",optimizedStages(issued),"egoErrorRadius",egoRadius(:,issued), ...
        "targetErrorRadius",targetRadius(:,issued),"witnessSeconds",witnessSeconds(issued), ...
        "minimumSampledSeparationMargin",minimumSeparation,"minimumSampledRoadMargin",minimumRoadMargin, ...
        "minimumSampledModelDomainMargin",minimumDomainMargin,"minimumSampledSpeed",minimumSpeed, ...
        "egoContainmentMargin",egoContainment(issued),"targetContainmentMargin",targetContainment(issued), ...
        "admissionFrame",admissionFrame(issued), ...
        "maximumSlewViolation",maximumSlewViolation,"terminal",terminal, ...
        "finalTargetDistance",norm(ego.position-target.targetPositionInertial), ...
        "solverFailureInjected",options.FailAfterAdmission, ...
        "scope","Each executed hold follows the accepted plan's first-stage affine generator from the true state; measurements carry bounded noise; conditioned-set inclusion transfers the feasible carried witness; no nonlinear-vehicle claim");
    report.completed = strlength(failureIdentifier)==0;
    report.failureIdentifier = failureIdentifier;
    report.failureMessage = failureMessage;
    report.executedHolds = sample;
    report.horizonSteps = horizonSteps(issued);
    report.predictionEndTime = predictionEndTime(issued);
    report.exitDeadline = exitDeadline(issued);
    report.releaseConfirmed = releaseConfirmed(issued);
    report.confirmationRange = options.ConfirmationRange;
    report.phaseSeconds = phaseSeconds(:,issued);
    report.phaseNames = ["prediction","formulation","solve","acceptance"];
    report.finalCruiseError = states(2:6,end)-[0;0;cfg.referenceSpeed;0;0];
    assessment = assessExactStateSafety(report);
    fields = fieldnames(assessment);
    for index = 1:numel(fields), report.(fields{index}) = assessment.(fields{index}); end
    report.runtime = struct("frameSeconds",frameSeconds, ...
        "deadlineSeconds",options.DeadlineSeconds, ...
        "admissionSeconds",frameSeconds(1), ...
        "maximumSuccessorSeconds",max(frameSeconds(2:end)), ...
        "deadlineMisses",nnz(frameSeconds>options.DeadlineSeconds), ...
        "deadlineMet",all(frameSeconds<=options.DeadlineSeconds), ...
        "scope","Input assembly and controller calls; excludes plant integration and geometry audit; diagnostic execution continues after deadline misses");
    report.runtimeQualified = report.passed && report.runtime.deadlineMet;
    localSave(report,options);
    fprintf('%s: %d/%d certified commands; %d executed holds; %d carried-witness commands; max value %.3g; separation %.6g m; road %.6g m; final speed %.6g m/s\n', ...
        options.Scenario,nnz(certified),numel(certified),sample,nnz(candidateExecuted), ...
        max(pcbfValue(issued)),minimumSeparation,minimumRoadMargin,states(4,end));
    if ~report.completed, fprintf('Control failed: %s\n',failureMessage); end
end

function margin = localDomainMargin(state,cfg)
% Independent physical state-domain audit, separate from plan certification.
    lower = [-cfg.model.lateralDomainRadius;-cfg.model.headingDomainRadius; ...
        cfg.model.speedMinimum;-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];
    upper = [cfg.model.lateralDomainRadius;cfg.model.headingDomainRadius; ...
        cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
    margin = min([state(2:6)-lower;upper-state(2:6)]);
end

function [egoMargin,targetMargin] = localContainment(state,target,problem,certificate)
% Check both the conditioned information state and the executed witness box.
    difference = state-problem.model.initialEgoState;
    difference(3) = atan2(sin(difference(3)),cos(difference(3)));
    egoMargin = min([problem.model.initialFrenetErrorBound-abs(difference); ...
        certificate.stateErrorBound(:,1)-abs(state-certificate.predictedState(:,1))]);
    targetMargin = inf;
    if ~isempty(certificate.encounters)
        difference = target-certificate.encounters.center;
        difference(7) = atan2(sin(difference(7)),cos(difference(7)));
        targetMargin = min(certificate.encounters.radius-abs(difference));
    end
end

function ego = localEgoMeasurement(x,time,heldInput,bound,stream,lane)
% Truth plus uniform noise inside the declared box, published with that box.
    if isempty(lane)
        position = x(1:2);
        heading = x(3);
    else
        [position,heading] = laneGeometry.fromFrenet(x,lane);
    end
    noise = bound.*(2*rand(stream,6,1)-1);
    ego = struct("position",position+noise(1:2),"yaw",heading+noise(3),"speed",x(4)+noise(4), ...
        "lateralVelocity",x(5)+noise(5),"yawRate",x(6)+noise(6),"stateTime",time, ...
        "controllerStateErrorBound",bound);
    if ~isempty(heldInput), ego.heldActuatorInput = heldInput; end
end

function target = localTargetMeasurement(truth,time,bound,stream)
    state = localTargetTruth(truth,time);
    noise = bound.*(2*rand(stream,8,1)-1);
    target = struct("trackId",1,"targetPositionInertial",state(1:2)+noise(1:2), ...
        "targetVelocityInertial",state(3:4)+noise(3:4),"targetAccelerationInertial",state(5:6)+noise(5:6), ...
        "targetHeadingInertial",state(7)+noise(7),"targetYawRate",state(8)+noise(8), ...
        "targetPositionInertialErrorBound",bound(1:2),"targetVelocityInertialErrorBound",bound(3:4), ...
        "targetAccelerationInertialErrorBound",bound(5:6),"targetYawErrorBound",bound(7), ...
        "targetYawRateErrorBound",bound(8), ...
        "predictionMotion",struct("kind","finite-sensing-motion-v1", ...
            "jerkBound",abs(truth.jerkAmplitude), ...
            "yawAccelerationBound",abs(truth.yawAccelerationAmplitude)));
end

function state = localTargetTruth(truth,time)
% Independent integrals of j(t)=J*cos(w*t), yawAcceleration(t)=H*cos(w*t).
    x = truth.center;
    w = truth.frequency;
    sine = sin(w*time);
    cosine = 1-cos(w*time);
    state = [(x(1:2)+x(3:4)*time+x(5:6)*time^2/2 ...
            +truth.jerkAmplitude*(time/w^2-sine/w^3)); ...
        x(3:4)+x(5:6)*time+truth.jerkAmplitude*cosine/w^2; ...
        x(5:6)+truth.jerkAmplitude*sine/w; ...
        x(7)+x(8)*time+truth.yawAccelerationAmplitude*cosine/w^2; ...
        x(8)+truth.yawAccelerationAmplitude*sine/w];
end

function value = localFirst(values)
    value = 0;
    if ~isempty(values), value = values(1); end
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
