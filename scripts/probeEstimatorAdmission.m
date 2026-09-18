function report = probeEstimatorAdmission(options)
%probeEstimatorAdmission Fresh admission attempts along a recorded exact-state run.
% Replays the NRMM adapter on the ego truth of a recorded declared-plant run
% (runDeclaredPlantEstimatorControllerScenario with UseEstimator=false) and, at
% every controller sample after radar acquisition, records the published ego
% and target bounds and attempts one fresh controller admission with a
% diagnostic search budget. No command is applied; the ego follows the
% recorded trajectory. This isolates acquisition-time uncertainty from the
% controller's own runtime.
    arguments
        options.ExactRunFile (1,1) string
        options.OutputDirectory (1,1) string = ""
        options.Seed (1,1) double {mustBeInteger,mustBeNonnegative} = 20260913
        options.SearchBudgetSeconds (1,1) double {mustBePositive} = 5
        options.MaximumProbeSeconds (1,1) double {mustBePositive} = 4
        options.MinimumRadarSamples (1,1) double {mustBeInteger,mustBePositive} = 1
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'estimator'),fullfile(root,'solver','nrmm'));
    loaded = load(options.ExactRunFile,'report');exact = loaded.report;
    cfg = exact.configuration;
    cfg.solver.frameDeadlineSeconds = options.SearchBudgetSeconds;
    cfg.solver.certificateSearchTimeLimit = options.SearchBudgetSeconds;
    estimator = exact.estimatorConfiguration;
    estimator.randomSeed = options.Seed;
    estimator.initialization.minimumRadarSamples = options.MinimumRadarSamples;
    scenario = exact.options;
    targetFunction = @(t,~) localTarget(t,scenario.TargetInitialDistance,scenario.ReferenceSpeed,scenario.TargetLateralPosition);
    road = struct('centerline',[-100,0;2000,0]);
    [context,~] = nrmmEstimatorControllerAdapter('initialize',estimator,localTruth(exact.state(:,1)),targetFunction);
    count = numel(exact.time);
    time = exact.time;published = false(1,count);attempted = false(1,count);certified = false(1,count);
    identifier = strings(1,count);message = strings(1,count);seconds = nan(1,count);horizon = nan(1,count);
    deficit = nan(1,count);nativeSolves = nan(1,count);
    targetVelocityBound = nan(2,count);targetPositionBound = nan(2,count);targetYawBound = nan(1,count);
    targetAccelerationBound = nan(2,count);egoBound = nan(6,count);jerkBound = nan(1,count);
    targetVelocityError = nan(1,count);targetPositionError = nan(1,count);
    acquisitionTime = NaN;
    for k = 1:count
        truth = exact.state(:,k);
        [context,ego,targets] = nrmmEstimatorControllerAdapter('sample',context,time(k),localTruth(truth),targetFunction(time(k),[]));
        if k>1, ego.heldActuatorInput = exact.input(:,k-1); end
        egoBound(:,k) = ego.controllerErrorBound.bounds(:);
        published(k) = ~isempty(targets);
        if ~published(k), continue; end
        if isnan(acquisitionTime), acquisitionTime = time(k); end
        bounds = targets.controllerErrorBound.bounds(:);
        targetPositionBound(:,k) = bounds(1:2);targetVelocityBound(:,k) = bounds(3:4);
        targetAccelerationBound(:,k) = bounds(5:6);targetYawBound(k) = bounds(7);
        jerkBound(k) = targets.predictionMotion.jerkBound(1);
        truthTarget = targetFunction(time(k),[]);
        targetPositionError(k) = norm(targets.targetPositionInertial-truthTarget.targetPositionInertial);
        targetVelocityError(k) = norm(targets.targetVelocityInertial-truthTarget.targetVelocityInertial);
        if time(k)-acquisitionTime > options.MaximumProbeSeconds, continue; end
        attempted(k) = true;timer = tic;
        try
            [~,~,problem] = collisionAvoidanceController(ego,targets,road,cfg,[]);
            certified(k) = problem.metadata.planCertified;horizon(k) = problem.metadata.horizonSteps;
            nativeSolves(k) = problem.metadata.solverCallCount;
        catch exception
            identifier(k) = string(exception.identifier);message(k) = string(exception.message);
            token = regexp(exception.message,'minimum normalized search deficit ([0-9.eE+-]+|Inf)','tokens','once');
            if ~isempty(token), deficit(k) = str2double(token{1}); end
        end
        seconds(k) = toc(timer);
        fprintf('t=%.2f range %.1f m: certified %d (%s) %.3f s; target bounds pos [%.3f %.3f] vel [%.2f %.2f] yaw %.2f; vel error %.3f\n', ...
            time(k),norm(targets.targetPositionInertial-truth(1:2)),certified(k),identifier(k),seconds(k), ...
            targetPositionBound(1,k),targetPositionBound(2,k),targetVelocityBound(1,k),targetVelocityBound(2,k),targetYawBound(k),targetVelocityError(k));
    end
    report = struct('time',time,'published',published,'attempted',attempted,'certified',certified, ...
        'failureIdentifier',identifier,'failureMessage',message,'attemptSeconds',seconds,'horizonSteps',horizon, ...
        'normalizedDeficit',deficit,'nativeSolves',nativeSolves,'acquisitionTime',acquisitionTime, ...
        'targetPositionBound',targetPositionBound,'targetVelocityBound',targetVelocityBound, ...
        'targetAccelerationBound',targetAccelerationBound,'targetYawBound',targetYawBound,'jerkBound',jerkBound, ...
        'targetPositionError',targetPositionError,'targetVelocityError',targetVelocityError,'egoBound',egoBound, ...
        'exactRunFile',options.ExactRunFile,'seed',options.Seed,'searchBudgetSeconds',options.SearchBudgetSeconds, ...
        'minimumRadarSamples',options.MinimumRadarSamples, ...
        'scope',"Open-loop estimator replay on a recorded exact-state trajectory; fresh admission probes only; no command applied");
    firstCertified = find(certified,1);
    if isempty(firstCertified)
        fprintf('No fresh admission certified within %.1f s after acquisition at %.4f s.\n',options.MaximumProbeSeconds,acquisitionTime);
    else
        fprintf('First certified fresh admission at t=%.2f s, %.2f s after acquisition (%.4f s).\n',time(firstCertified),time(firstCertified)-acquisitionTime,acquisitionTime);
    end
    if strlength(options.OutputDirectory)>0
        if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
        save(fullfile(options.OutputDirectory,'estimator-admission-probes.mat'),'report');
    end
end

function state = localTruth(x)
    state=struct('position',x(1:2),'yawAngle',x(3),'longitudinalVelocity',x(4),'lateralVelocity',x(5),'yawRate',x(6));
end

function target = localTarget(time,distance,speed,lateral)
    target=struct('targetPositionInertial',[distance-speed*time;lateral],'targetVelocityInertial',[-speed;0], ...
        'targetAccelerationInertial',[0;0],'targetYawInertial',pi,'targetYawRate',0,'targetLength',4.8,'targetWidth',1.9);
end
