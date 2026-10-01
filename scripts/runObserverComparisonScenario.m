function result = runObserverComparisonScenario(options)
% runObserverComparisonScenario Open-loop observer comparison on scenario geometry.
%
% Builds the truth trajectories of the controller scenario drivers (straight
% oncoming, circular centerline with a tangent straight target, circular
% following, avoidance swerve, near-zero relative velocity, aggressive ego
% weaving) without the controller in the loop, synthesizes the synchronized
% sensor frames of the estimator contract, and runs the cascaded measured-input
% NRMM observer and the Sharma et al. (2026) multistage high-gain observer
% variants on identical truth, noise draws and initial estimates.
%
% Scenario geometries (ego starts at the origin with zero heading):
%   straightOncoming    ego 15 m/s straight; target 95 m ahead, heading pi, 15 m/s,
%                       1.5 m lateral offset (runOncomingVehicleAvoidanceScenario).
%   circularCrossing    ego on the R = 100 m left arc at 15 m/s; straight target
%                       tangent to the arc, built to meet the ego at the nominal
%                       collision time with a 1.5 m offset
%                       (runCircularCenterlineStraightTargetAvoidanceScenario).
%   circularFollowing   ego on the R = 100 m arc at 15 m/s; target 30 m ahead
%                       on the same arc at 15.5 m/s (constant curvature, constant
%                       sideslip, an exact Sharma-model target).
%   avoidanceSwerve     ego straight 15 m/s, then a 3.5 m double lane change
%                       with braking to 11 m/s from t = 2 s (nonzero ego jerk and
%                       yaw acceleration); oncoming target at 95 m, 12 m/s.
%   lowRelativeVelocity ego straight 15 m/s; target 30 m ahead, same direction,
%                       15.3 m/s (the RMM singularity case of the paper, Sec. 4.3).
%   aggressiveEgo       the benchmark's weaving ego (12 +/- 3 m/s, two yaw-rate
%                       sinusoids) with the retained Sharma-model target.
%
% Options: Scenario, Estimators ("structured" and the Sharma variant names
% decoded by prepareObserverComparisonEstimators, or a prepared struct array), Seed,
% NoiseModel ("boundedUniform"|"none"), NoiseScale, SamplePeriod,
% IntegrationStepMaximum, Duration (scenario default when NaN),
% InitialOffsetScale, TransientDuration (2 s), DropoutIntervals,
% SingleTrackMismatch (constant omegaE-vEy/lrE in rad/s), Plot, Report.
%
% Metrics discard the transient window and include the paper's Table 3
% quantities (target speed, relative speed and course-angle error, RMS and
% maximum) together with relative-position, inertial-velocity, acceleration,
% ego position, ego velocity, ego yaw and ego sideslip errors. The same key
% errors are also reported over the last half of the run ("steady" fields).

    arguments
        options.Scenario (1, 1) string {mustBeMember(options.Scenario, ["straightOncoming", ...
            "circularCrossing", "circularFollowing", "avoidanceSwerve", ...
            "lowRelativeVelocity", "aggressiveEgo"])} = "straightOncoming"
        options.Estimators = ["structured", "sharmaCertifiedHold", "sharmaCertifiedPredictor", ...
            "sharmaMatchedHold", "sharmaMatchedPredictor", "sharmaCorrectedMatchedHold", ...
            "sharmaCorrectedMatchedPredictor"]
        options.Seed (1, 1) double {mustBeInteger, mustBeNonnegative} = 7
        options.NoiseModel (1, 1) string ...
            {mustBeMember(options.NoiseModel, ["boundedUniform", "none"])} = "boundedUniform"
        options.NoiseScale (1, 1) double {mustBeNonnegative} = 1.0
        options.SamplePeriod (1, 1) double {mustBePositive} = 0.02
        options.IntegrationStepMaximum (1, 1) double {mustBePositive} = 0.005
        options.Duration (1, 1) double = NaN
        options.InitialOffsetScale (1, 1) double {mustBeNonnegative} = 1.0
        options.TransientDuration (1, 1) double {mustBeNonnegative} = 2.0
        options.DropoutIntervals (:, 2) double = zeros(0, 2)
        options.SingleTrackMismatch (1, 1) double {mustBeFinite} = 0.0
        options.Plot (1, 1) logical = false
        options.Report (1, 1) logical = true
    end

    localAddProjectPaths();
    scenario = localScenarioDefinition(options.Scenario);
    if ~isnan(options.Duration)
        scenario.duration = options.Duration;
    end
    cfg = localConfiguration(scenario, options);
    time = (0.0:cfg.runtime.samplePeriod:scenario.duration).';
    truth = localTruth(time, scenario, cfg, options.SingleTrackMismatch);
    measurements = localMeasurements(truth, cfg, options);
    initial = localInitialEstimate(truth, options.InitialOffsetScale);
    if isstruct(options.Estimators)
        estimators = options.Estimators;     % prepared by an earlier call (same scenario)
    else
        estimators = prepareObserverComparisonEstimators(string(options.Estimators), cfg);
    end

    runs = struct([]);
    for index = 1:numel(estimators)
        estimator = estimators(index);
        estimate = localRunObserver(time, measurements, initial, cfg, estimator);
        metrics = localMetrics(truth, estimate, time, options.TransientDuration);
        run = struct("name", estimator.name, "description", estimator.description, ...
            "runtime", estimator.runtime, "design", estimator.design, ...
            "estimate", estimate, "metrics", metrics);
        if isempty(runs)
            runs = run;
        else
            runs(end+1) = run; %#ok<AGROW>
        end
    end

    result = struct("options", options, "scenario", scenario, "config", cfg, ...
        "truth", truth, "measurements", measurements, "initialEstimate", initial, ...
        "runs", runs, "truthDomainValid", localTruthDomainValid(truth, cfg));
    if options.Report
        localReport(result);
    end
    if options.Plot
        result.figure = plotObserverComparison(result);
    end
end

%% Scenario definitions

function scenario = localScenarioDefinition(name)
    radius = 100.0;
    cruiseSpeed = 15.0;
    scenario = struct("name", name, "duration", 12.0, "relativePositionMaximum", 50.0, ...
        "targetSideslipMaximum", 0.015, "targetSpeedMinimum", 10.0, ...
        "egoYawRateMaximum", 0.30, "egoSpeedMaximum", 20.0, "egoSpeedMinimum", 5.0, ...
        "egoSideslipMaximum", 0.05);
    switch name
        case "straightOncoming"
            scenario.ego = localEgoProfile("straight", cruiseSpeed, radius);
            scenario.target = localStraightTarget([95.0; 1.5], pi, 15.0);
            scenario.duration = 6.0;
            scenario.relativePositionMaximum = 100.0;
            scenario.description = "Straight centerline, head-on target 95 m ahead at 15 m/s";
        case "circularCrossing"
            scenario.ego = localEgoProfile("circular", cruiseSpeed, radius);
            scenario.target = localCrossingTarget(radius, cruiseSpeed, 15.0, 90.0, 1.5);
            scenario.duration = 6.0;
            scenario.relativePositionMaximum = 100.0;
            scenario.description = sprintf("R = %.0f m left arc, straight tangent target " ...
                + "meeting the ego at t = %.2f s", radius, scenario.target.collisionTime);
        case "circularFollowing"
            scenario.ego = localEgoProfile("circular", cruiseSpeed, radius);
            leadArc = 30.0;
            scenario.target = localArcTarget(radius, leadArc, 15.5, 1.6);
            scenario.duration = 12.0;
            scenario.relativePositionMaximum = 50.0;
            scenario.targetSideslipMaximum = 0.03;
            scenario.description = "R = 100 m arc, target 30 m ahead on the same arc at 15.5 m/s";
        case "avoidanceSwerve"
            scenario.ego = localEgoProfile("swerve", cruiseSpeed, radius);
            scenario.target = localStraightTarget([95.0; 1.5], pi, 12.0);
            scenario.duration = 6.5;
            scenario.relativePositionMaximum = 100.0;
            scenario.egoYawRateMaximum = 0.40;
            % The swerve reaches 0.053 rad with the MnCAV rear-axle distance.
            scenario.egoSideslipMaximum = 0.06;
            scenario.description = "Straight cruise, 3.5 m double lane change with braking " ...
                + "from t = 2 s; oncoming target at 95 m, 12 m/s";
        case "lowRelativeVelocity"
            scenario.ego = localEgoProfile("straight", cruiseSpeed, radius);
            scenario.target = localStraightTarget([30.0; 0.0], 0.0, 15.3);
            scenario.duration = 12.0;
            scenario.relativePositionMaximum = 50.0;
            scenario.description = "Straight road, lead target 30 m ahead at 15.3 m/s";
        case "aggressiveEgo"
            scenario.ego = localEgoProfile("aggressive", 12.0, radius);
            scenario.target = localSharmaTarget([25.0; 4.0], 0.10 + 0.0064, 12.5, 0.08, 0.0064, 1.6);
            scenario.duration = 12.0;
            scenario.relativePositionMaximum = 80.0;
            scenario.description = "Weaving, accelerating ego with the retained Sharma-model target";
        otherwise
            error("runObserverComparisonScenario:unknownScenario", "Unknown scenario %s.", name);
    end
end

function ego = localEgoProfile(kind, speed, radius)
% localEgoProfile Analytic speed and yaw-rate profiles with first derivatives.
    switch kind
        case "straight"
            ego = struct("speed", @(t) speed + zeros(size(t)), "speedDot", @(t) zeros(size(t)), ...
                "yawRate", @(t) zeros(size(t)), "yawRateDot", @(t) zeros(size(t)));
        case "circular"
            rate = speed/radius;
            ego = struct("speed", @(t) speed + zeros(size(t)), "speedDot", @(t) zeros(size(t)), ...
                "yawRate", @(t) rate + zeros(size(t)), "yawRateDot", @(t) zeros(size(t)));
        case "swerve"
            % Yaw-rate pulse A*sin(2*pi*tau/T)^3 over one period: zero net heading,
            % continuous yaw acceleration at the window edges, lateral shift set
            % by quadrature; braking is a smooth tanh speed drop.
            start = 2.0;
            period = 3.0;
            lateralShift = 3.5;
            brakeCentre = 2.5;
            brakeWidth = 0.6;
            speedDrop = 4.0;
            speedFunction = @(t) speed - 0.5*speedDrop*(1.0 + tanh((t - brakeCentre)/brakeWidth));
            window = @(t) double(t >= start & t <= start + period);
            unitPulse = @(t) sin(2.0*pi*(t - start)/period).^3.*window(t);
            unitHeading = @(t) integral(unitPulse, start, t);
            unitShift = integral(@(t) arrayfun(@(u) speedFunction(u)*sin(unitHeading(u)), t), ...
                start, start + period, ArrayValued=false);
            amplitude = lateralShift/unitShift;
            ego = struct( ...
                "speed", speedFunction, ...
                "speedDot", @(t) -0.5*speedDrop/brakeWidth*sech((t - brakeCentre)/brakeWidth).^2, ...
                "yawRate", @(t) amplitude*unitPulse(t), ...
                "yawRateDot", @(t) amplitude*3.0*(2.0*pi/period) ...
                    *sin(2.0*pi*(t - start)/period).^2.*cos(2.0*pi*(t - start)/period).*window(t));
        case "aggressive"
            ego = struct( ...
                "speed", @(t) speed + 3.0*sin(0.5*t), "speedDot", @(t) 1.5*cos(0.5*t), ...
                "yawRate", @(t) 0.12*sin(0.35*t) + 0.16*sin(1.2*t), ...
                "yawRateDot", @(t) 0.042*cos(0.35*t) + 0.192*cos(1.2*t));
        otherwise
            error("runObserverComparisonScenario:unknownEgo", "Unknown ego profile %s.", kind);
    end
    ego.kind = kind;
end

function target = localSharmaTarget(position, course, speed, acceleration, sideslip, rearAxle)
% localSharmaTarget Constant scalar acceleration, constant sideslip target (paper eq. 2).
    target = struct("initialPosition", position(:), "initialCourse", course, ...
        "initialSpeed", speed, "scalarAcceleration", acceleration, "sideslip", sideslip, ...
        "rearAxleDistance", rearAxle, "curvature", sin(sideslip)/rearAxle, "collisionTime", NaN);
end

function target = localStraightTarget(position, course, speed)
    target = localSharmaTarget(position, course, speed, 0.0, 0.0, 1.6);
end

function target = localArcTarget(radius, leadArc, speed, rearAxle)
    angle = leadArc/radius;
    position = radius*[sin(angle); 1.0 - cos(angle)];
    target = localSharmaTarget(position, angle, speed, 0.0, asin(rearAxle/radius), rearAxle);
end

function target = localCrossingTarget(radius, egoSpeed, targetSpeed, initialDistance, lateralOffset)
% localCrossingTarget Straight target meeting the arc-following ego (controller geometry).
    egoPosition = @(t) radius*[sin(egoSpeed*t/radius); 1.0 - cos(egoSpeed*t/radius)];
    heading = @(t) egoSpeed*t/radius;
    initialPosition = @(t) egoPosition(t) ...
        + lateralOffset*[-sin(heading(t)); cos(heading(t))] ...
        + targetSpeed*t*[cos(heading(t)); sin(heading(t))];
    residual = @(t) norm(initialPosition(t)) - initialDistance;
    upper = 2.0*initialDistance/(egoSpeed + targetSpeed);
    while residual(upper) < 0.0
        upper = 1.5*upper;
    end
    collisionTime = fzero(residual, [0.0, upper]);
    course = atan2(-sin(heading(collisionTime)), -cos(heading(collisionTime)));
    target = localStraightTarget(initialPosition(collisionTime), course, targetSpeed);
    target.collisionTime = collisionTime;
    target.lateralOffset = lateralOffset;
end

%% Configuration, truth and measurements

function cfg = localConfiguration(scenario, options)
    cfg = nrmmTrackingConfig();
    cfg.runtime.samplePeriod = options.SamplePeriod;
    cfg.runtime.integrationStepMaximum = options.IntegrationStepMaximum;
    cfg.target.domain.relativePositionMaximum = scenario.relativePositionMaximum;
    cfg.target.domain.sideslipMaximum = scenario.targetSideslipMaximum;
    cfg.target.domain.speedMinimum = scenario.targetSpeedMinimum;
    cfg.ego.domain.yawRateMaximum = scenario.egoYawRateMaximum;
    cfg.ego.yaw.sideslipDomainMaximum = scenario.egoSideslipMaximum;
    cfg.ego.domain.speedMaximum = scenario.egoSpeedMaximum;
    cfg.ego.domain.speedMinimum = scenario.egoSpeedMinimum;
end

function truth = localTruth(time, scenario, cfg, singleTrackMismatch)
% localTruth Kinematically consistent ego and exact Sharma-model target truth.
%
% The ego follows omegaE = VE*sin(betaE)/lrE + singleTrackMismatch.
% Positions integrate the analytic velocities on a grid twenty times finer than
% the sample grid and are then sampled.
    refinement = 20;
    fineTime = linspace(time(1), time(end), refinement*(numel(time) - 1) + 1).';
    egoRearAxle = cfg.ego.yaw.rearAxleDistance;
    ego = scenario.ego;
    speed = ego.speed(fineTime);
    speedDot = ego.speedDot(fineTime);
    yawRate = ego.yawRate(fineTime);
    yawRateDot = ego.yawRateDot(fineTime);
    yaw = cumtrapz(fineTime, yawRate);
    normalized = egoRearAxle*(yawRate - singleTrackMismatch)./speed;
    sideslip = asin(normalized);
    sideslipDot = egoRearAxle*(yawRateDot.*speed ...
        - (yawRate - singleTrackMismatch).*speedDot)./speed.^2 ...
        ./sqrt(1.0 - normalized.^2);
    course = yaw + sideslip;
    courseRate = yawRate + sideslipDot;
    velocity = speed.*[cos(course), sin(course)];
    position = [cumtrapz(fineTime, velocity(:, 1)), cumtrapz(fineTime, velocity(:, 2))];
    bodyAcceleration = speedDot.*[cos(sideslip), sin(sideslip)] ...
        + (speed.*courseRate).*[-sin(sideslip), cos(sideslip)];
    acceleration = localRotateRows(yaw, bodyAcceleration);

    target = scenario.target;
    targetSpeed = target.initialSpeed + target.scalarAcceleration*fineTime;
    targetCourse = target.initialCourse + target.curvature ...
        .*(target.initialSpeed*fineTime + 0.5*target.scalarAcceleration*fineTime.^2);
    targetVelocity = targetSpeed.*[cos(targetCourse), sin(targetCourse)];
    targetPosition = target.initialPosition.' ...
        + [cumtrapz(fineTime, targetVelocity(:, 1)), cumtrapz(fineTime, targetVelocity(:, 2))];
    targetYawRate = targetSpeed*target.curvature;
    targetAcceleration = target.scalarAcceleration*[cos(targetCourse), sin(targetCourse)] ...
        + (targetSpeed.*targetYawRate).*[-sin(targetCourse), cos(targetCourse)];

    select = 1:refinement:numel(fineTime);
    truth = struct("time", time, ...
        "egoPosition", position(select, :), "egoVelocity", velocity(select, :), ...
        "egoAcceleration", acceleration(select, :), "egoBodyAcceleration", bodyAcceleration(select, :), ...
        "egoSpeed", speed(select), "egoYaw", yaw(select), "egoYawRate", yawRate(select), ...
        "egoYawAcceleration", yawRateDot(select), "egoSpeedDot", speedDot(select), ...
        "egoSideslip", sideslip(select), "egoCourse", course(select), ...
        "singleTrackMismatch", singleTrackMismatch, ...
        "targetPosition", targetPosition(select, :), "targetVelocity", targetVelocity(select, :), ...
        "targetAcceleration", targetAcceleration(select, :), "targetSpeed", targetSpeed(select), ...
        "targetCourse", targetCourse(select), "targetHeading", targetCourse(select) - target.sideslip, ...
        "targetYawRate", targetYawRate(select));
    % Ego body jerk magnitude (finite difference of the exact body acceleration),
    % reported to quantify how far each scenario is from the paper's Assumption 1.
    fineStep = fineTime(2) - fineTime(1);
    fineBodyJerk = [gradient(bodyAcceleration(:, 1), fineStep), gradient(bodyAcceleration(:, 2), fineStep)];
    truth.egoBodyJerk = fineBodyJerk(select, :);

    sampleCount = numel(time);
    transformed = zeros(sampleCount, 6);
    relativeVelocity = zeros(sampleCount, 2);
    for sampleIdx = 1:sampleCount
        rotationTranspose = localRotation(truth.egoYaw(sampleIdx)).';
        rho = rotationTranspose*(truth.targetPosition(sampleIdx, :) - truth.egoPosition(sampleIdx, :)).';
        q = rotationTranspose*truth.targetVelocity(sampleIdx, :).';
        s = rotationTranspose*truth.targetAcceleration(sampleIdx, :).';
        bodyVelocity = rotationTranspose*truth.egoVelocity(sampleIdx, :).';
        transformed(sampleIdx, :) = [rho; q; s].';
        relativeVelocity(sampleIdx, :) = (q - bodyVelocity ...
            - truth.egoYawRate(sampleIdx)*[0.0, -1.0; 1.0, 0.0]*rho).';
    end
    truth.targetTransformedState = transformed;
    truth.relativeVelocity = relativeVelocity;
    truth.relativeRange = vecnorm(transformed(:, 1:2), 2, 2);
end

function measurements = localMeasurements(truth, cfg, options)
% localMeasurements Bounded-uniform synchronized frames (same draw layout as the benchmark).
    sampleCount = numel(truth.time);
    switch options.NoiseModel
        case "none"
            unitNoise = zeros(sampleCount, 9);
        case "boundedUniform"
            previous = rng;
            restore = onCleanup(@() rng(previous));
            rng(options.Seed, "twister");
            unitNoise = 2.0*rand(sampleCount, 9) - 1.0;
        otherwise
            error("runObserverComparisonScenario:invalidNoise", "Unknown noise model.");
    end
    componentScale = options.NoiseScale/sqrt(2.0);
    measurements = struct();
    measurements.gpsPosition = truth.egoPosition ...
        + componentScale*cfg.measurement.gps.positionNoiseMaximum*unitNoise(:, 1:2);
    measurements.gpsVelocity = truth.egoVelocity ...
        + componentScale*cfg.measurement.gps.velocityNoiseMaximum*unitNoise(:, 3:4);
    measurements.bodyAcceleration = truth.egoBodyAcceleration ...
        + componentScale*cfg.measurement.imu.noiseMaximum*unitNoise(:, 5:6);
    measurements.yawRate = truth.egoYawRate ...
        + options.NoiseScale*cfg.measurement.gyroscope.noiseMaximum*unitNoise(:, 7);
    measurements.radarRelativePosition = truth.targetTransformedState(:, 1:2) ...
        + componentScale*cfg.measurement.radar.noiseMaximum*unitNoise(:, 8:9);
    measurements.radarAvailable = true(sampleCount, 1);
    for interval = options.DropoutIntervals.'
        measurements.radarAvailable(truth.time >= interval(1) & truth.time < interval(2)) = false;
    end
end

function initial = localInitialEstimate(truth, scale)
% localInitialEstimate Truth plus the benchmark's documented initialization offsets.
    initial = struct( ...
        "egoPosition", truth.egoPosition(1, :).' + scale*[1.0; -0.8], ...
        "egoYaw", truth.egoYaw(1) + scale*0.03, ...
        "egoBodyVelocity", [truth.egoSpeed(1); 0.0] + scale*[0.5; -0.3], ...
        "targetState", truth.targetTransformedState(1, :).' + scale*[1.0; -0.5; 0.5; 0.5; 0.2; -0.2]);
end

function estimate = localRunObserver(time, measurements, initial, cfg, estimator)
    runtimeOptions = struct("initialTime", time(1), "targetIdentifier", "scenario-target-1", ...
        "egoInitialPosition", initial.egoPosition, "egoInitialYaw", initial.egoYaw, ...
        "egoInitialBodyVelocity", initial.egoBodyVelocity, "targetInitialState", initial.targetState);
    runtime = estimator.runtime("initialize", cfg, runtimeOptions, estimator.design);
    sampleCount = numel(time);
    egoPosition = NaN(sampleCount, 2);
    egoVelocity = NaN(sampleCount, 2);
    egoYaw = NaN(sampleCount, 1);
    egoSideslip = NaN(sampleCount, 1);
    targetState = NaN(sampleCount, 6);
    relativeVelocity = NaN(sampleCount, 2);
    targetVelocityInertial = NaN(sampleCount, 2);
    targetCourseInertial = NaN(sampleCount, 1);
    targetSpeed = NaN(sampleCount, 1);
    targetYawRate = NaN(sampleCount, 1);
    stepSeconds = NaN(sampleCount, 1);
    failureMessage = "";
    integrationStep = cfg.runtime.integrationStepMaximum;

    egoPosition(1, :) = initial.egoPosition.';
    egoVelocity(1, :) = (localRotation(initial.egoYaw)*initial.egoBodyVelocity).';
    egoYaw(1) = initial.egoYaw;
    egoSideslip(1) = atan2(initial.egoBodyVelocity(2), initial.egoBodyVelocity(1));
    targetState(1, :) = initial.targetState.';
    targetSpeed(1) = norm(initial.targetState(3:4));
    targetVelocityInertial(1, :) = (localRotation(initial.egoYaw)*initial.targetState(3:4)).';
    targetCourseInertial(1) = localWrapToPi(initial.egoYaw ...
        + atan2(initial.targetState(4), initial.targetState(3)));

    lastValid = 1;
    for sampleIdx = 1:(sampleCount - 1)
        frame = struct("time", time(sampleIdx), ...
            "xGps", measurements.gpsPosition(sampleIdx, 1), "yGps", measurements.gpsPosition(sampleIdx, 2), ...
            "vxGps", measurements.gpsVelocity(sampleIdx, 1), "vyGps", measurements.gpsVelocity(sampleIdx, 2), ...
            "longitudinalAcceleration", measurements.bodyAcceleration(sampleIdx, 1), ...
            "lateralAcceleration", measurements.bodyAcceleration(sampleIdx, 2), ...
            "yawRateMeasured", measurements.yawRate(sampleIdx), ...
            "radarRelativePosition", measurements.radarRelativePosition(sampleIdx, :), ...
            "radarDetectionAvailable", measurements.radarAvailable(sampleIdx));
        if ~frame.radarDetectionAvailable
            frame.radarRelativePosition(:) = NaN;
        end
        try
            timer = tic;
            [runtime, output] = estimator.runtime("step", runtime, frame);
            stepSeconds(sampleIdx + 1) = toc(timer);
        catch failure
            failureMessage = string(failure.message);
            break
        end
        next = sampleIdx + 1;
        egoPosition(next, :) = output.egoState([1, 4]).';
        egoVelocity(next, :) = output.egoState([2, 5]).';
        egoYaw(next) = output.egoYaw;
        egoSideslip(next) = atan2(output.egoBodyVelocity(2), output.egoBodyVelocity(1));
        targetState(next, :) = output.targetState.';
        relativeVelocity(next, :) = output.targetEstimate.relativeVelocity.';
        targetVelocityInertial(next, :) = output.targetEstimate.targetVelocityInertial.';
        targetCourseInertial(next) = output.targetEstimate.targetCourseAngleInertial;
        targetSpeed(next) = output.targetEstimate.targetSpeed;
        targetYawRate(next) = output.targetEstimate.targetYawRate;
        if isfield(output, "integrationStep")
            integrationStep = output.integrationStep;
        end
        if all(isfinite(output.targetState)) && all(isfinite(output.egoState))
            lastValid = next;
        elseif failureMessage == ""
            failureMessage = sprintf("non-finite estimate at t = %.3f s", time(next));
        end
    end
    estimate = struct("egoPosition", egoPosition, "egoVelocity", egoVelocity, "egoYaw", egoYaw, ...
        "egoSideslip", egoSideslip, "targetState", targetState, "relativeVelocity", relativeVelocity, ...
        "targetVelocityInertial", targetVelocityInertial, "targetCourseInertial", targetCourseInertial, ...
        "targetSpeed", targetSpeed, "targetYawRate", targetYawRate, "stepSeconds", stepSeconds, ...
        "completed", failureMessage == "" && lastValid == sampleCount, ...
        "failureMessage", failureMessage, "lastValidSample", lastValid, ...
        "integrationStep", integrationStep, "runtime", runtime);
end

%% Metrics

function metrics = localMetrics(truth, estimate, time, transientDuration)
    evaluation = time >= transientDuration;
    % Second window: the last half of the run, reported separately because the
    % comparator's yaw stage has a long initial excursion (its acceleration
    % states start at zero and drive the acceleration-based heading).
    steadyStart = max(transientDuration, 0.5*time(end));
    steady = time >= steadyStart;
    positionError = vecnorm(estimate.targetState(:, 1:2) - truth.targetTransformedState(:, 1:2), 2, 2);
    velocityError = vecnorm(estimate.targetState(:, 3:4) - truth.targetTransformedState(:, 3:4), 2, 2);
    accelerationError = vecnorm(estimate.targetState(:, 5:6) - truth.targetTransformedState(:, 5:6), 2, 2);
    inertialVelocityError = vecnorm(estimate.targetVelocityInertial - truth.targetVelocity, 2, 2);
    speedError = estimate.targetSpeed - truth.targetSpeed;
    relativeSpeedError = vecnorm(estimate.relativeVelocity, 2, 2) - vecnorm(truth.relativeVelocity, 2, 2);
    courseErrorDeg = rad2deg(localWrapToPi(estimate.targetCourseInertial - truth.targetCourse));
    egoPositionError = vecnorm(estimate.egoPosition - truth.egoPosition, 2, 2);
    egoVelocityError = vecnorm(estimate.egoVelocity - truth.egoVelocity, 2, 2);
    egoYawErrorDeg = rad2deg(localWrapToPi(estimate.egoYaw - truth.egoYaw));
    egoSideslipErrorDeg = rad2deg(localWrapToPi(estimate.egoSideslip - truth.egoSideslip));

    metrics = struct();
    metrics.completed = estimate.completed;
    metrics.failureMessage = estimate.failureMessage;
    divergenceRadius = 1.0e3;                              % m
    metrics.diverged = ~estimate.completed || any(positionError > divergenceRadius) ...
        || any(~isfinite(positionError));
    metrics.divergenceRadius = divergenceRadius;
    metrics.transientDuration = transientDuration;
    metrics.relativePositionRmse = localRms(positionError, evaluation);
    metrics.fullPositionRmse = localRms(positionError, true(size(time)));
    metrics.fullSpeedRmse = localRms(speedError, true(size(time)));
    metrics.relativePositionMax = localMax(positionError, evaluation);
    metrics.targetVelocityRmse = localRms(velocityError, evaluation);
    metrics.targetVelocityInertialRmse = localRms(inertialVelocityError, evaluation);
    metrics.targetAccelerationRmse = localRms(accelerationError, evaluation);
    metrics.targetSpeedRmse = localRms(speedError, evaluation);
    metrics.targetSpeedMax = localMax(speedError, evaluation);
    metrics.relativeSpeedRmse = localRms(relativeSpeedError, evaluation);
    metrics.relativeSpeedMax = localMax(relativeSpeedError, evaluation);
    metrics.courseRmseDeg = localRms(courseErrorDeg, evaluation);
    metrics.courseMaxDeg = localMax(courseErrorDeg, evaluation);
    metrics.egoPositionRmse = localRms(egoPositionError, evaluation);
    metrics.egoVelocityRmse = localRms(egoVelocityError, evaluation);
    metrics.egoYawRmseDeg = localRms(egoYawErrorDeg, evaluation);
    metrics.egoYawMaxDeg = localMax(egoYawErrorDeg, evaluation);
    metrics.egoSideslipRmseDeg = localRms(egoSideslipErrorDeg, evaluation);
    metrics.steadyStateStart = steadyStart;
    metrics.steadyRelativePositionRmse = localRms(positionError, steady);
    metrics.steadyTargetVelocityRmse = localRms(velocityError, steady);
    metrics.steadyTargetSpeedRmse = localRms(speedError, steady);
    metrics.steadyRelativeSpeedRmse = localRms(relativeSpeedError, steady);
    metrics.steadyCourseRmseDeg = localRms(courseErrorDeg, steady);
    metrics.steadyEgoYawRmseDeg = localRms(egoYawErrorDeg, steady);
    metrics.peakTransientPositionError = localMax(positionError, ~evaluation);
    metrics.peakTransientVelocityError = localMax(velocityError, ~evaluation);
    metrics.peakTransientAccelerationError = localMax(accelerationError, ~evaluation);
    metrics.convergenceTime = localConvergenceTime(time, positionError, velocityError, 0.5, 1.0);
    metrics.meanStepMilliseconds = 1000.0*mean(estimate.stepSeconds(2:end), "omitnan");
    metrics.maximumStepMilliseconds = 1000.0*max(estimate.stepSeconds(2:end), [], "omitnan");
    metrics.integrationStep = estimate.integrationStep;
    metrics.egoBodyJerkMaximum = max(vecnorm(truth.egoBodyJerk, 2, 2));
    metrics.egoYawAccelerationMaximum = max(abs(truth.egoYawAcceleration));
    metrics.relativeRangeMaximum = max(truth.relativeRange);
end

function value = localRms(error, mask)
    selected = error(mask);
    if any(~isfinite(selected))
        value = Inf;
    else
        value = sqrt(mean(selected.^2));
    end
end

function value = localMax(error, mask)
    selected = abs(error(mask));
    if any(~isfinite(selected))
        value = Inf;
    else
        value = max(selected);
    end
end

function convergence = localConvergenceTime(time, positionError, velocityError, positionTolerance, velocityTolerance)
% localConvergenceTime First time after which both errors stay inside tolerance.
    inside = positionError <= positionTolerance & velocityError <= velocityTolerance;
    inside(~isfinite(positionError) | ~isfinite(velocityError)) = false;
    stays = flipud(cumprod(flipud(double(inside))));
    index = find(stays > 0, 1, "first");
    if isempty(index)
        convergence = Inf;
    else
        convergence = time(index);
    end
end

function valid = localTruthDomainValid(truth, cfg)
    state = truth.targetTransformedState;
    speed = vecnorm(state(:, 3:4), 2, 2);
    domain = cfg.target.domain;
    acceleration = sum(state(:, 3:4).*state(:, 5:6), 2)./speed;
    curvature = sum([-state(:, 4), state(:, 3)].*state(:, 5:6), 2)./speed.^3;
    valid = all(speed >= domain.speedMinimum - 1.0e-7 & speed <= domain.speedMaximum + 1.0e-7 ...
        & abs(acceleration) <= domain.scalarAccelerationMaximum + 1.0e-7 ...
        & abs(curvature) <= sin(domain.sideslipMaximum)/domain.rearAxleDistance + 1.0e-7 ...
        & truth.relativeRange <= domain.relativePositionMaximum + 1.0e-7) ...
        && all(truth.egoSpeed >= cfg.ego.domain.speedMinimum - 1.0e-7 ...
        & truth.egoSpeed <= cfg.ego.domain.speedMaximum + 1.0e-7) ...
        && all(abs(truth.egoYawRate) <= cfg.ego.domain.yawRateMaximum + 1.0e-7) ...
        && all(abs(truth.egoSideslip) <= cfg.ego.yaw.sideslipDomainMaximum + 1.0e-7) ...
        && abs(truth.singleTrackMismatch) ...
            <= cfg.ego.yaw.singleTrackYawRateMismatchMaximum + 1.0e-7;
end

function localReport(result)
    fprintf("\nObserver comparison: %s (%s)\n", result.scenario.name, result.scenario.description);
    fprintf("  %d samples at %.3g s, seed %d, noise %s x%.2g, truth domain valid: %d\n", ...
        numel(result.truth.time), result.config.runtime.samplePeriod, result.options.Seed, ...
        result.options.NoiseModel, result.options.NoiseScale, result.truthDomainValid);
    fprintf("  %-32s %10s %10s %10s %10s %10s %10s %8s\n", "estimator", "rho RMSE", "V_C RMSE", ...
        "Vrel RMSE", "course RMS", "yaw RMS", "conv. t", "diverged");
    for run = result.runs
        m = run.metrics;
        fprintf("  %-32s %10.4g %10.4g %10.4g %10.4g %10.4g %10.2f %8d\n", run.name, ...
            m.relativePositionRmse, m.targetSpeedRmse, m.relativeSpeedRmse, m.courseRmseDeg, ...
            m.egoYawRmseDeg, m.convergenceTime, m.diverged);
    end
end

%% Helpers

function rotated = localRotateRows(angle, vectors)
    rotated = [cos(angle).*vectors(:, 1) - sin(angle).*vectors(:, 2), ...
        sin(angle).*vectors(:, 1) + cos(angle).*vectors(:, 2)];
end

function rotation = localRotation(angle)
    rotation = [cos(angle), -sin(angle); sin(angle), cos(angle)];
end

function value = localWrapToPi(value)
    value = mod(value + pi, 2.0*pi) - pi;
end

function localAddProjectPaths()
    repoRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(repoRoot, "config"));
    addpath(fullfile(repoRoot, "estimator"));
end
