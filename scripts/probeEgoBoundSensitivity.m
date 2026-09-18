function report = probeEgoBoundSensitivity(options)
%probeEgoBoundSensitivity Admissible ego error bounds at a recorded acquisition frame.
% Takes the ego state and the true target of the first published frame of a
% recorded exact-state declared-plant run, gives the target a small exact-like
% bound, and sweeps the ego state-error bound through fresh controller
% admissions with a diagnostic search budget. It also reports how the
% controller's interval-hull propagation grows each ego box over the cruise
% stage flow, which is the mechanism that widens the certified nodes.
    arguments
        options.ExactRunFile (1,1) string
        options.OutputDirectory (1,1) string = ""
        options.EstimatorEgoBound (6,1) double {mustBeNonnegative} = [0.076;0.076;0.048;0.089;0.497;0.0015]
        options.TargetPositionBound (1,1) double {mustBeNonnegative} = 0.2
        options.TargetYawBound (1,1) double {mustBeNonnegative} = 0.05
        options.SearchBudgetSeconds (1,1) double {mustBePositive} = 5
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    loaded = load(options.ExactRunFile,'report');exact = loaded.report;
    cfg = exact.configuration;
    cfg.solver.frameDeadlineSeconds = options.SearchBudgetSeconds;
    cfg.solver.certificateSearchTimeLimit = options.SearchBudgetSeconds;
    frame = find(exact.published,1);
    truth = exact.state(:,frame);time = exact.time(frame);
    target = exact.targetEstimate{frame};
    target.targetPositionInertialErrorBound = repmat(options.TargetPositionBound,2,1);
    target.targetVelocityInertialErrorBound = zeros(2,1);
    target.targetAccelerationInertialErrorBound = zeros(2,1);
    target.targetYawErrorBound = options.TargetYawBound;target.targetYawRateErrorBound = 0;
    target.predictionMotion = struct('kind',"finite-sensing-motion-v1",'jerkBound',zeros(2,1),'yawAccelerationBound',0);
    road = struct('centerline',[-100,0;2000,0]);
    full = options.EstimatorEgoBound;
    names = ["zero";"estimator";"position only";"yaw only";"speed only";"lateral velocity only";"yaw rate only"; ...
        "estimator x0.5";"estimator x0.25";"estimator x0.1";"estimator without lateral velocity";"estimator without yaw and lateral velocity"];
    bounds = zeros(6,numel(names));
    bounds(:,2) = full;bounds(1:2,3) = full(1:2);bounds(3,4) = full(3);bounds(4,5) = full(4);bounds(5,6) = full(5);bounds(6,7) = full(6);
    bounds(:,8) = .5*full;bounds(:,9) = .25*full;bounds(:,10) = .1*full;
    bounds(:,11) = full;bounds(5,11) = 0;bounds(:,12) = full;bounds([3,5],12) = 0;
    count = numel(names);
    certified = false(count,1);identifier = strings(count,1);seconds = zeros(count,1);
    deficit = nan(count,1);horizon = nan(count,1);terminalBox = nan(6,count);
    stateMatrix = ltvBicycleModel.stageMatrices(0,cfg.referenceSpeed,cfg.controller.sampleTime,cfg);
    for index = 1:count
        ego = struct('position',truth(1:2),'yaw',truth(3),'speed',truth(4),'lateralVelocity',truth(5), ...
            'yawRate',truth(6),'stateTime',time,'controllerStateErrorBound',bounds(:,index), ...
            'perception',struct('time',time,'range',exact.estimatorConfiguration.sensor.radar.rangeMaximum,'completeWithinRange',true));
        if frame>1, ego.heldActuatorInput = exact.input(:,frame-1); end
        timer = tic;
        try
            [~,~,problem] = collisionAvoidanceController(ego,target,road,cfg,[]);
            certified(index) = problem.metadata.planCertified;horizon(index) = problem.metadata.horizonSteps;
            terminalBox(:,index) = problem.prediction.initialErrorBound(:,end);
        catch exception
            identifier(index) = string(exception.identifier);
            token = regexp(exception.message,'minimum normalized search deficit ([0-9.eE+-]+|Inf)','tokens','once');
            if ~isempty(token), deficit(index) = str2double(token{1}); end
        end
        seconds(index) = toc(timer);
        % Interval-hull growth of this box over 36 cruise stages, the controller's chain.
        box = bounds(:,index);
        for stage = 1:36, box = abs(stateMatrix)*box; end
        terminalHull = box;
        fprintf('%-42s certified %d %-48s deficit %7.4g horizon %3g  |Phi|^36 box: d %.3f m psi %.3f rad (%.2f s)\n', ...
            names(index),certified(index),identifier(index),deficit(index),horizon(index),terminalHull(2),terminalHull(3),seconds(index));
    end
    report = struct('time',time,'range',norm(target.targetPositionInertial-truth(1:2)),'names',names,'egoErrorBound',bounds, ...
        'certified',certified,'failureIdentifier',identifier,'normalizedDeficit',deficit,'horizonSteps',horizon, ...
        'terminalNodeBox',terminalBox,'seconds',seconds,'exactRunFile',options.ExactRunFile, ...
        'scope',"Synthetic ego bounds with an exact-like target on a recorded acquisition frame; fresh admission probes; no command applied");
    if strlength(options.OutputDirectory)>0
        if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
        save(fullfile(options.OutputDirectory,'ego-bound-sensitivity.mat'),'report');
    end
end
