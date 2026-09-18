function report = probeTargetBoundSensitivity(options)
%probeTargetBoundSensitivity Admissible target error bounds at a recorded acquisition frame.
% Takes the ego state and the true target of the first published frame of a
% recorded exact-state declared-plant run, attaches the recorded estimator ego
% bound, and sweeps synthetic target velocity, acceleration, yaw and motion
% bounds through fresh controller admissions with a diagnostic search budget.
% This measures what the certificate can admit; it is not an estimator result.
    arguments
        options.ExactRunFile (1,1) string
        options.OutputDirectory (1,1) string = ""
        options.EgoErrorBound (6,1) double {mustBeNonnegative} = [0.076;0.076;0.048;0.089;0.497;0.0015]
        options.PositionBound (1,1) double {mustBeNonnegative} = 0.2
        options.VelocityBounds (1,:) double {mustBeNonnegative} = [0,0.1,0.25,0.5,1,2]
        options.AccelerationBounds (1,:) double {mustBeNonnegative} = [0,0.5,2]
        options.YawBounds (1,:) double {mustBeNonnegative} = [0.05,0.3]
        options.JerkBound (1,1) double {mustBeNonnegative} = 0
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
    ego = struct('position',truth(1:2),'yaw',truth(3),'speed',truth(4),'lateralVelocity',truth(5), ...
        'yawRate',truth(6),'stateTime',time,'controllerStateErrorBound',options.EgoErrorBound, ...
        'perception',struct('time',time,'range',exact.estimatorConfiguration.sensor.radar.rangeMaximum,'completeWithinRange',true));
    if frame>1, ego.heldActuatorInput = exact.input(:,frame-1); end
    truthTarget = exact.targetEstimate{frame};
    road = struct('centerline',[-100,0;2000,0]);
    grid = {options.VelocityBounds,options.AccelerationBounds,options.YawBounds};
    [velocity,acceleration,yaw] = ndgrid(grid{:});
    count = numel(velocity);
    certified = false(count,1);identifier = strings(count,1);seconds = zeros(count,1);
    deficit = nan(count,1);horizon = nan(count,1);
    for index = 1:count
        target = truthTarget;
        target.targetPositionInertialErrorBound = repmat(options.PositionBound,2,1);
        target.targetVelocityInertialErrorBound = repmat(velocity(index),2,1);
        target.targetAccelerationInertialErrorBound = repmat(acceleration(index),2,1);
        target.targetYawErrorBound = yaw(index);
        target.targetYawRateErrorBound = 0;
        target.predictionMotion = struct('kind',"finite-sensing-motion-v1",'jerkBound',repmat(options.JerkBound,2,1), ...
            'yawAccelerationBound',0);
        timer = tic;
        try
            [~,~,problem] = collisionAvoidanceController(ego,target,road,cfg,[]);
            certified(index) = problem.metadata.planCertified;horizon(index) = problem.metadata.horizonSteps;
        catch exception
            identifier(index) = string(exception.identifier);
            token = regexp(exception.message,'minimum normalized search deficit ([0-9.eE+-]+|Inf)','tokens','once');
            if ~isempty(token), deficit(index) = str2double(token{1}); end
        end
        seconds(index) = toc(timer);
        fprintf('velocity %.2f acceleration %.2f yaw %.2f: certified %d %s deficit %.4g (%.2f s)\n', ...
            velocity(index),acceleration(index),yaw(index),certified(index),identifier(index),deficit(index),seconds(index));
    end
    report = struct('time',time,'range',norm(truthTarget.targetPositionInertial-truth(1:2)), ...
        'egoErrorBound',options.EgoErrorBound,'positionBound',options.PositionBound,'jerkBound',options.JerkBound, ...
        'velocityBound',velocity(:),'accelerationBound',acceleration(:),'yawBound',yaw(:), ...
        'certified',certified,'failureIdentifier',identifier,'normalizedDeficit',deficit,'horizonSteps',horizon, ...
        'seconds',seconds,'exactRunFile',options.ExactRunFile, ...
        'scope',"Synthetic target bounds on a recorded acquisition frame; fresh admission probes; no command applied");
    if strlength(options.OutputDirectory)>0
        if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
        save(fullfile(options.OutputDirectory,'target-bound-sensitivity.mat'),'report');
    end
end
