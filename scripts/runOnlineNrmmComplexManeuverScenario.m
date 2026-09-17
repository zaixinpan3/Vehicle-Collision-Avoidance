function result = runOnlineNrmmComplexManeuverScenario(varargin)
% runOnlineNrmmComplexManeuverScenario Reproducible open-loop estimator scenario.
% Options: Plot, Report, Duration (12 s), Seed (7), NoiseModel ('none' or
% 'boundedUniform'), Config (nrmmTrackingConfig), DropoutIntervals (N-by-2 s),
% TargetMotion ('retained', 'varying' or 'laneChange'), and EgoManeuver
% ('retained', 'straight' or 'aggressive'). Initial estimates use documented
% offsets; metrics discard the first 2 s. The retained truth and random draws
% are unchanged from the paired high-gain baseline. The varying case declares
% changing geometric A and curvature and measures response lag; laneChange
% applies one sinusoidal curvature pulse (zero net heading change).
% Scenario-campaign options, all defaulting to the retained values:
% TargetInitialPosition (1-by-2 m), TargetInitialHeading (rad),
% TargetInitialSpeed (m/s), NoiseScale (multiplies every bounded draw, so a
% value above 1 exceeds the declared sensor bounds) and InitialOffsetScale
% (multiplies the documented initial estimate offsets).
% Lyapunov ultimate bounds are reported separately for the continuous model;
% they are not asserted as certified sample-by-sample digital error radii.

    options = localOptions(varargin{:});
    localAddProjectPaths();
    % Declared initial transient discarded before every RMSE evaluation.
    transientDuration = 2.0;                          % s

    cfg = options.configuration;
    design = options.designFunction(cfg);
    time = (0.0:cfg.runtime.samplePeriod:options.duration).';
    truth = localTruth(time, ...
        design.yaw.courseModel.rearAxleDistance, ...
        design.target.domain.rearAxleDistance, options);
    measurements = localMeasurements(truth, cfg, options);
    initial = localInitialEstimate(truth, options);
    estimate = localRunObserver(time, measurements, initial, cfg, design, options);
    metrics = localMetrics(truth, estimate, time, transientDuration, cfg, options);
    metrics.curvatureLagSeconds = localCurvatureLag(truth,estimate,options);
    metrics.continuousUltimateBounds = [design.ultimateBounds.relativePosition, ...
        design.ultimateBounds.targetVelocity,design.ultimateBounds.targetAcceleration];
    metrics.continuousTargetDecayRate = design.target.lambda;
    metrics.targetBandwidth = design.target.bandwidth;
    requiredModelJerk = hypot(cfg.target.model.scalarAccelerationRateMaximum, ...
        cfg.target.domain.speedMaximum^2*cfg.target.model.curvatureRateMaximum);
    coveredModelJerk = 0;
    if isfield(design.target,"modelJerkMaximum")
        coveredModelJerk = design.target.modelJerkMaximum;
    end
    metrics.declaredModelJerkCovered = coveredModelJerk+1e-12 >= requiredModelJerk;
    metrics.hasRadarDropout = ~isempty(options.dropoutIntervals);



    result = struct();
    result.options = options;
    result.config = cfg;
    result.design = design;
    result.truth = truth;
    result.measurements = measurements;
    result.initialEstimate = initial;
    result.estimate = estimate;
    result.metrics = metrics;

    if options.report
        localReport(metrics, design, cfg.runtime.samplePeriod);
    end
    if options.plot && ~batchStartupOptionUsed
        localPlot(result);
    end
end

function truth = localTruth(time, ...
        egoRearAxleDistance, targetRearAxleDistance, options)
% localTruth Single-track ego maneuver and an exact Sharma-model target.
%
% Ego: sinusoidal speed and yaw-rate profiles, with betaE recovered from
% VE*sin(betaE)/lrE = psiEdot. Jerk and yaw acceleration are analytically
% nonzero. Target: VCdot = AC (constant), betaC constant, and
% psiCdot = VC*sin(betaC)/lrC, so [rho; q; s] follows the observer model.

    [egoSpeed,egoSpeedDot,egoSpeedDdot,egoYaw,egoYawRate, ...
        egoYawAcceleration,egoYawJerk] = localEgoProfile(time,options.egoManeuver);
    normalizedYawRate = egoRearAxleDistance*egoYawRate./egoSpeed;
    normalizedYawRateDerivative = egoRearAxleDistance ...
        .* (egoYawAcceleration.*egoSpeed-egoYawRate.*egoSpeedDot) ...
        ./ egoSpeed.^2;
    normalizedYawRateSecondDerivative = egoRearAxleDistance .* ( ...
        egoYawJerk./egoSpeed ...
        - 2.0*egoYawAcceleration.*egoSpeedDot./egoSpeed.^2 ...
        - egoYawRate.*egoSpeedDdot./egoSpeed.^2 ...
        + 2.0*egoYawRate.*egoSpeedDot.^2./egoSpeed.^3);
    egoSideslip = asin(normalizedYawRate);
    egoSingleTrackMismatch = egoYawRate ...
        - egoSpeed.*sin(egoSideslip)/egoRearAxleDistance;
    egoSideslipRate = normalizedYawRateDerivative ...
        ./ sqrt(1.0-normalizedYawRate.^2);
    egoSideslipAcceleration = normalizedYawRateSecondDerivative ...
            ./ sqrt(1.0-normalizedYawRate.^2) ...
        + normalizedYawRate.*normalizedYawRateDerivative.^2 ...
            ./ (1.0-normalizedYawRate.^2).^(3.0/2.0);
    egoCourse = egoYaw+egoSideslip;
    egoCourseRate = egoYawRate+egoSideslipRate;
    egoCourseAcceleration = ...
        egoYawAcceleration+egoSideslipAcceleration;
    egoVelocity = egoSpeed.*[cos(egoCourse), sin(egoCourse)];
    egoPosition = [cumtrapz(time, egoVelocity(:, 1)), ...
        cumtrapz(time, egoVelocity(:, 2))];
    egoSideslipDirection = [cos(egoSideslip), sin(egoSideslip)];
    egoSideslipNormal = [-sin(egoSideslip), cos(egoSideslip)];
    egoBodyAcceleration = egoSpeedDot.*egoSideslipDirection ...
        + (egoSpeed.*egoCourseRate).*egoSideslipNormal;
    egoAcceleration = localRotateRows(egoYaw, egoBodyAcceleration);
    egoCourseDirection = [cos(egoCourse), sin(egoCourse)];
    egoCourseNormal = [-sin(egoCourse), cos(egoCourse)];
    egoJerk = (egoSpeedDdot-egoSpeed.*egoCourseRate.^2) ...
            .* egoCourseDirection ...
        + (2.0*egoSpeedDot.*egoCourseRate ...
            + egoSpeed.*egoCourseAcceleration).*egoCourseNormal;

    % Sharma target parameters (all constant by assumption).
    % Retained defaults: 12.5 m/s, heading 0.10 rad, position [25, 4] m.
    targetInitialSpeed = options.targetInitialSpeed;  % m/s VC(0)
    targetScalarAcceleration = 0.08;                  % m/s^2 AC
    targetSideslip = 0.0064;                          % rad betaC
    targetInitialHeading = options.targetInitialHeading; % rad psiC(0)
    targetInitialPosition = options.targetInitialPosition; % m, inertial

    targetSpeed = targetInitialSpeed+targetScalarAcceleration*time;
    targetHeading = targetInitialHeading ...
        + (sin(targetSideslip)/targetRearAxleDistance) ...
        .*(targetInitialSpeed*time+0.5*targetScalarAcceleration*time.^2);
    targetCourse = targetHeading+targetSideslip;
    targetYawRate = targetSpeed*sin(targetSideslip) ...
        / targetRearAxleDistance;
    targetVelocity = targetSpeed.*[cos(targetCourse), sin(targetCourse)];
    targetPosition = targetInitialPosition ...
        + [cumtrapz(time, targetVelocity(:, 1)), ...
        cumtrapz(time, targetVelocity(:, 2))];
    % aC = AC*e(course) + VC*omegaC*J*e(course).
    targetAcceleration = ...
        targetScalarAcceleration*[cos(targetCourse), sin(targetCourse)] ...
        + (targetSpeed.*targetYawRate) ...
        .*[-sin(targetCourse), cos(targetCourse)];

    targetCurvature = repmat(sin(targetSideslip)/targetRearAxleDistance,numel(time),1);
    scalarAccelerationTruth = repmat(targetScalarAcceleration,numel(time),1);
    if options.targetMotion ~= "retained"
        profile = localTargetMotionProfile(options.targetMotion,options.duration, ...
            targetScalarAcceleration,targetCurvature(1));
        initialCourse = targetInitialHeading+targetSideslip;
        [~,trajectory] = ode113(@(t,x) localProfileDerivative(t,x,profile), ...
            time,[targetInitialPosition.';initialCourse;targetInitialSpeed], ...
            odeset("RelTol",1.0e-11,"AbsTol",1.0e-12));
        scalarAccelerationTruth = profile.scalarAcceleration(time);
        targetCurvature = profile.curvature(time);
        targetPosition = trajectory(:,1:2);
        targetCourse = trajectory(:,3);
        targetSpeed = trajectory(:,4);
        targetYawRate = targetSpeed.*targetCurvature;
        targetVelocity = targetSpeed.*[cos(targetCourse),sin(targetCourse)];
        targetAcceleration = scalarAccelerationTruth.*[cos(targetCourse),sin(targetCourse)] ...
            +(targetSpeed.^2.*targetCurvature).*[-sin(targetCourse),cos(targetCourse)];
        % This heading is the instantaneous constant-sideslip reconstruction;
        % no changing-sideslip body-yaw model is asserted in this stress case.
        targetHeading = targetCourse-asin(targetRearAxleDistance*targetCurvature);
    end
    sampleCount = numel(time);
    targetTransformedState = zeros(sampleCount, 6);
    for sampleIdx = 1:sampleCount
        rotationTranspose = localRotation(egoYaw(sampleIdx)).';
        rho = rotationTranspose*(targetPosition(sampleIdx, :) ...
            - egoPosition(sampleIdx, :)).';
        q = rotationTranspose*targetVelocity(sampleIdx, :).';
        s = rotationTranspose*targetAcceleration(sampleIdx, :).';
        targetTransformedState(sampleIdx, :) = [rho; q; s].';
    end

    truth = struct( ...
        "time", time, ...
        "egoPosition", egoPosition, ...
        "egoVelocity", egoVelocity, ...
        "egoAcceleration", egoAcceleration, ...
        "egoJerk", egoJerk, ...
        "egoSpeed", egoSpeed, ...
        "egoYaw", egoYaw, ...
        "egoSideslip", egoSideslip, ...
        "egoCourse", egoCourse, ...
        "egoSingleTrackMismatch", egoSingleTrackMismatch, ...
        "egoYawRate", egoYawRate, ...
        "egoYawAcceleration", egoYawAcceleration, ...
        "egoBodyAcceleration", egoBodyAcceleration, ...
        "targetPosition", targetPosition, ...
        "targetVelocity", targetVelocity, ...
        "targetAcceleration", targetAcceleration, ...
        "targetSpeed", targetSpeed, ...
        "targetHeading", targetHeading, ...
        "targetCourse", targetCourse, ...
        "targetYawRate", targetYawRate, ...
        "targetScalarAcceleration", scalarAccelerationTruth, "targetCurvature", targetCurvature, ...
        "targetTransformedState", targetTransformedState);
end

function profile = localTargetMotionProfile(motion,duration,acceleration,curvature)
% localTargetMotionProfile Time functions of scalar acceleration and curvature.
%
% varying: smooth 0.2 s tanh transition at mid-duration raising A by 0.6 m/s^2
% and lowering curvature by 0.007 1/m. laneChange: one full sine period of
% curvature with amplitude 0.005 1/m over 4 s starting at 4 s, so the heading
% returns to its pre-maneuver value and the target shifts laterally by about
% 2 m; the peak curvature rate is 0.005*2*pi/4 1/(m s). With the retained
% base curvature 0.004 1/m the peak 0.009 1/m stays inside the 0.015 rad
% sideslip domain (0.009375 1/m).
    profile = struct("window",[NaN,NaN]);
    switch motion
        case "varying"
            centre = duration/2;
            transition = @(t) 0.5*(1+tanh((t-centre)/0.2));
            profile.scalarAcceleration = @(t) acceleration+0.6*transition(t);
            profile.curvature = @(t) curvature-0.007*transition(t);
            profile.window = [centre-0.5,centre+1.5];
            profile.curvatureRateMaximum = 0.0175;
        case "laneChange"
            start = 4.0;
            period = 4.0;
            amplitude = 0.005;
            pulse = @(t) amplitude*sin(2*pi*(t-start)/period).*(t >= start & t <= start+period);
            profile.scalarAcceleration = @(t) acceleration+zeros(size(t));
            profile.curvature = @(t) curvature+pulse(t);
            profile.window = [start-0.5,start+period+0.5];
            profile.curvatureRateMaximum = amplitude*2*pi/period;
        otherwise
            error("runOnlineNrmmComplexManeuverScenario:invalidTargetMotion", ...
                "TargetMotion must be 'retained', 'varying' or 'laneChange'.");
    end
end

function derivative = localProfileDerivative(time,state,profile)
    derivative = [state(4)*cos(state(3));state(4)*sin(state(3)); ...
        profile.curvature(time)*state(4);profile.scalarAcceleration(time)];
end

function [speed,speedDot,speedDdot,yaw,yawRate,yawAcceleration,yawJerk] = ...
        localEgoProfile(time,maneuver)
% localEgoProfile Analytic ego speed and yaw profiles with exact derivatives.
%
% retained: the paired-baseline profile. straight: constant 12 m/s, zero yaw
% rate. aggressive: 12 +/- 3 m/s speed swings and the retained slow turn plus
% a 0.16 rad/s, 1.2 rad/s weave (peak yaw rate 0.28 rad/s, peak yaw
% acceleration about 0.23 rad/s^2), inside the default ego domain.
    switch maneuver
        case "retained"
            speed = 12.0+1.2*sin(0.25*time);
            speedDot = 0.30*cos(0.25*time);
            speedDdot = -0.075*sin(0.25*time);
            yawRate = 0.12*sin(0.35*time);
            yawAcceleration = 0.042*cos(0.35*time);
            yawJerk = -0.0147*sin(0.35*time);
            yaw = (0.12/0.35)*(1.0-cos(0.35*time));
        case "straight"
            speed = 12.0+zeros(size(time));
            speedDot = zeros(size(time));
            speedDdot = zeros(size(time));
            yawRate = zeros(size(time));
            yawAcceleration = zeros(size(time));
            yawJerk = zeros(size(time));
            yaw = zeros(size(time));
        case "aggressive"
            speed = 12.0+3.0*sin(0.5*time);
            speedDot = 1.5*cos(0.5*time);
            speedDdot = -0.75*sin(0.5*time);
            yawRate = 0.12*sin(0.35*time)+0.16*sin(1.2*time);
            yawAcceleration = 0.042*cos(0.35*time)+0.192*cos(1.2*time);
            yawJerk = -0.0147*sin(0.35*time)-0.2304*sin(1.2*time);
            yaw = (0.12/0.35)*(1.0-cos(0.35*time))+(0.16/1.2)*(1.0-cos(1.2*time));
        otherwise
            error("runOnlineNrmmComplexManeuverScenario:invalidEgoManeuver", ...
                "EgoManeuver must be 'retained', 'straight' or 'aggressive'.");
    end
end

function measurements = localMeasurements(truth, cfg, options)
% localMeasurements Synchronized sensor frames from truth signals.
%
% Bounded-uniform noise respects the configured per-sensor bounds; the
% two-component channels are scaled by 1/sqrt(2) so the vector norm stays
% within the deterministic bound.

    sampleCount = numel(truth.time);
    switch options.noiseModel
        case "none"
            unitNoise = zeros(sampleCount, 9);
        case "boundeduniform"
            previousRandomState = rng;
            restoreRandomState = onCleanup( ...
                @() rng(previousRandomState));
            rng(options.seed, "twister");
            unitNoise = 2.0*rand(sampleCount, 9)-1.0;
        otherwise
            error("runOnlineNrmmComplexManeuverScenario:invalidNoiseModel", ...
                "NoiseModel must be 'none' or 'boundedUniform'.");
    end
    % NoiseScale = 1 realizes the declared bounds exactly; larger values
    % deliberately violate them to probe degradation.
    componentScale = options.noiseScale/sqrt(2.0);
    gpsPositionBound = componentScale ...
        * cfg.measurement.gps.positionNoiseMaximum;
    gpsVelocityBound = componentScale ...
        * cfg.measurement.gps.velocityNoiseMaximum;
    imuBound = componentScale*cfg.measurement.imu.noiseMaximum;
    gyroscopeBound = options.noiseScale*cfg.measurement.gyroscope.noiseMaximum;
    radarBound = componentScale*cfg.measurement.radar.noiseMaximum;

    measurements = struct();
    measurements.gpsPosition = truth.egoPosition ...
        + gpsPositionBound*unitNoise(:, 1:2);
    measurements.gpsVelocity = truth.egoVelocity ...
        + gpsVelocityBound*unitNoise(:, 3:4);
    measurements.bodyAcceleration = truth.egoBodyAcceleration ...
        + imuBound*unitNoise(:, 5:6);
    measurements.yawRate = truth.egoYawRate+gyroscopeBound*unitNoise(:, 7);
    measurements.radarRelativePosition = ...
        truth.targetTransformedState(:, 1:2)+radarBound*unitNoise(:, 8:9);
end

function initial = localInitialEstimate(truth, options)
% localInitialEstimate Truth plus the documented initialization offsets.
%
% The target q offset is below 1 m/s at unit scale. The same offsets are
% used for the improved high-gain observer and an independently versioned
% comparator; InitialOffsetScale multiplies every offset uniformly.

    scale = options.initialOffsetScale;
    initial = struct( ...
        "egoPosition", truth.egoPosition(1, :).'+scale*[1.0; -0.8], ...
        "egoYaw", truth.egoYaw(1)+scale*0.03, ...
        "egoBodyVelocity", [truth.egoSpeed(1); 0.0]+scale*[0.5; -0.3], ...
        "targetState", truth.targetTransformedState(1, :).' ...
        + scale*[1.0; -0.5; 0.5; 0.5; 0.2; -0.2]);
end

function estimate = localRunObserver(time, measurements, initial, cfg, design, options)
% localRunObserver Step the runtime over the synchronized frame sequence.

    runtimeOptions = struct( ...
        "initialTime", time(1), ...
        "targetCount", 1, ...
        "targetIdentifiers", "scenario-target-1", ...
        "egoInitialPosition", initial.egoPosition, ...
        "egoInitialYaw", initial.egoYaw, ...
        "egoInitialBodyVelocity", initial.egoBodyVelocity, ...
        "targetInitialState", initial.targetState);
    runtime = options.runtimeFunction( ...
        "initialize", cfg, runtimeOptions, design);

    sampleCount = numel(time);
    egoState = NaN(sampleCount, 6);
    egoYaw = NaN(sampleCount, 1);
    egoYawRate = NaN(sampleCount, 1);
    targetState = NaN(sampleCount, 6);
    targetVelocityInertial = NaN(sampleCount, 2);
    targetHeadingInertial = NaN(sampleCount, 1);
    targetSpeed = NaN(sampleCount, 1);
    targetYawRate = NaN(sampleCount, 1);
    stepSeconds = NaN(sampleCount, 1);
    relativePositionErrorBound = NaN(sampleCount,1);
    positionErrorBoundAvailable = false(sampleCount,1);
    if isfield(runtime,"positionErrorBound")
        relativePositionErrorBound(1) = runtime.positionErrorBound.targetComponents(1);
        positionErrorBoundAvailable(1) = runtime.positionErrorBound.valid(1);
    end

    % Sample 1 carries the initial estimate mapped through the same output
    % transformations the runtime applies.
    initialRotation = localRotation(initial.egoYaw);
    initialVelocity = initialRotation*initial.egoBodyVelocity;
    initialAcceleration = initialRotation ...
        *measurements.bodyAcceleration(1, :).';
    egoState(1, :) = [initial.egoPosition(1); initialVelocity(1); ...
        initialAcceleration(1); initial.egoPosition(2); ...
        initialVelocity(2); initialAcceleration(2)].';
    egoYaw(1) = initial.egoYaw;
    egoYawRate(1) = measurements.yawRate(1);
    targetState(1, :) = initial.targetState.';
    initialEgo = struct( ...
        "bodyVelocity", initial.egoBodyVelocity, ...
        "yawRate", measurements.yawRate(1));
    [~, initialTargetEstimate] = nrmmTargetTrackerDerivative( ...
        initial.targetState, initialEgo, design.target.domain);
    targetVelocityInertial(1, :) = ...
        (initialRotation*initialTargetEstimate.targetVelocity).';
    targetHeadingInertial(1) = localWrapToPi(initial.egoYaw ...
        + initialTargetEstimate.targetCourseAngleEgoFrame ...
        - initialTargetEstimate.targetSideslip);
    targetSpeed(1) = initialTargetEstimate.targetSpeed;
    targetYawRate(1) = initialTargetEstimate.targetYawRate;

    output = struct();
    for sampleIdx = 1:(sampleCount-1)
        frame = struct( ...
            "time", time(sampleIdx), ...
            "xGps", measurements.gpsPosition(sampleIdx, 1), ...
            "yGps", measurements.gpsPosition(sampleIdx, 2), ...
            "vxGps", measurements.gpsVelocity(sampleIdx, 1), ...
            "vyGps", measurements.gpsVelocity(sampleIdx, 2), ...
            "longitudinalAcceleration", ...
                measurements.bodyAcceleration(sampleIdx, 1), ...
            "lateralAcceleration", ...
                measurements.bodyAcceleration(sampleIdx, 2), ...
            "yawRateMeasured", measurements.yawRate(sampleIdx), ...
            "radarRelativePosition", ...
                measurements.radarRelativePosition(sampleIdx, :), ...
            "radarDetectionAvailable", true);
        if any(frame.time >= options.dropoutIntervals(:,1) & frame.time < options.dropoutIntervals(:,2))
            frame.radarRelativePosition(:) = NaN;
            frame.radarDetectionAvailable = false;
        end
        timer = tic;
        [runtime, output] = options.runtimeFunction( ...
            "step", runtime, frame);
        stepSeconds(sampleIdx+1) = toc(timer);
        if isfield(output,"relativePositionErrorBound")
            relativePositionErrorBound(sampleIdx+1) = output.relativePositionErrorBound;
            positionErrorBoundAvailable(sampleIdx+1) = output.positionErrorBoundAvailable;
        end
        targetOutput = output.targetEstimates(1);
        egoState(sampleIdx+1, :) = output.egoState.';
        egoYaw(sampleIdx+1) = output.egoYaw;
        egoYawRate(sampleIdx+1) = output.egoYawRate;
        targetState(sampleIdx+1, :) = output.targetStates(1, :);
        targetVelocityInertial(sampleIdx+1, :) = ...
            targetOutput.targetVelocityInertial.';
        targetHeadingInertial(sampleIdx+1) = ...
            targetOutput.targetHeadingInertial;
        targetSpeed(sampleIdx+1) = targetOutput.targetSpeed;
        targetYawRate(sampleIdx+1) = targetOutput.targetYawRate;
    end

    estimate = struct( ...
        "egoState", egoState, ...
        "egoYaw", egoYaw, ...
        "egoYawRate", egoYawRate, ...
        "targetState", targetState, ...
        "targetVelocityInertial", targetVelocityInertial, ...
        "targetHeadingInertial", targetHeadingInertial, ...
        "targetSpeed", targetSpeed, ...
        "targetYawRate", targetYawRate, ...
        "stepSeconds", stepSeconds, ...
        "relativePositionErrorBound",relativePositionErrorBound, ...
        "positionErrorBoundAvailable",positionErrorBoundAvailable, ...
        "finalOutput", output, ...
        "runtime", runtime);
end

function metrics = localMetrics(truth, estimate, time, transientDuration, cfg, options)
% localMetrics RMSE against truth after the declared transient window.

    evaluation = time >= transientDuration;
    finalOutput = estimate.finalOutput;
    metrics = struct();
    metrics.transientDuration = transientDuration;
    metrics.egoPositionRmse = localVectorRmse( ...
        estimate.egoState(:, [1, 4])-truth.egoPosition, evaluation);
    metrics.egoVelocityRmse = localVectorRmse( ...
        estimate.egoState(:, [2, 5])-truth.egoVelocity, evaluation);
    metrics.egoAccelerationRmse = localVectorRmse( ...
        estimate.egoState(:, [3, 6])-truth.egoAcceleration, evaluation);
    metrics.egoYawRmse = sqrt(mean(localWrapToPi( ...
        estimate.egoYaw(evaluation)-truth.egoYaw(evaluation)).^2));
    metrics.relativePositionRmse = localVectorRmse( ...
        estimate.targetState(:, 1:2) ...
        - truth.targetTransformedState(:, 1:2), evaluation);
    metrics.targetVelocityRmse = localVectorRmse( ...
        estimate.targetVelocityInertial-truth.targetVelocity, evaluation);
    metrics.targetAccelerationRmse = localVectorRmse( ...
        estimate.targetState(:, 5:6) ...
        - truth.targetTransformedState(:, 5:6), evaluation);
    metrics.targetHeadingRmse = sqrt(mean(localWrapToPi( ...
        estimate.targetHeadingInertial(evaluation) ...
        - truth.targetHeading(evaluation)).^2));
    metrics.allSamplesFinite = ...
        all(isfinite(estimate.egoState), "all") ...
        && all(isfinite(estimate.egoYaw), "all") ...
        && all(isfinite(estimate.egoYawRate), "all") ...
        && all(isfinite(estimate.targetState), "all") ...
        && all(isfinite(estimate.targetVelocityInertial), "all") ...
        && all(isfinite(estimate.targetHeadingInertial), "all");
    metrics.targetSpeedDomainValidCumulative = ...
        finalOutput.targetCertifiedSpeedDomainValid;
    metrics.targetSpeedDomainValidFinalInterval = ...
        finalOutput.targetCertifiedSpeedDomainValidThisInterval;
    metrics.operatingDomainValidCumulative = ...
        finalOutput.estimatedOperatingDomainValid;
    metrics.operatingDomainValidFinalInterval = ...
        finalOutput.estimatedOperatingDomainValidThisInterval;
    metrics.minimumReconstructedTargetSpeed = ...
        finalOutput.minimumReconstructedTargetSpeed;
    metrics.maximumAuditedRelativePosition = ...
        finalOutput.estimatedOperatingDomainAudit.maximumRelativePosition;
    physicalError = [vecnorm(estimate.targetState(:,1:2)-truth.targetTransformedState(:,1:2),2,2), ...
        vecnorm(estimate.targetState(:,3:4)-truth.targetTransformedState(:,3:4),2,2), ...
        vecnorm(estimate.targetState(:,5:6)-truth.targetTransformedState(:,5:6),2,2)];
    metrics.peakInitialPositionError = max(physicalError(time <= transientDuration,1));
    metrics.peakInitialVelocityError = max(physicalError(time <= transientDuration,2));
    metrics.peakInitialAccelerationError = max(physicalError(time <= transientDuration,3));
    metrics.maximumPositionError = max(physicalError(evaluation,1));
    metrics.maximumAccelerationError = max(physicalError(evaluation,3));
    metrics.meanStepMilliseconds = 1000*mean(estimate.stepSeconds(2:end));
    metrics.maximumStepMilliseconds = 1000*max(estimate.stepSeconds(2:end));
    metrics.digitalErrorBoundCertified = false;
    metrics.positionBoundAvailableFraction = mean(estimate.positionErrorBoundAvailable);
    metrics.positionBoundContainmentFraction = mean(estimate.positionErrorBoundAvailable ...
        & physicalError(:,1) <= estimate.relativePositionErrorBound+1e-10);
    metrics.minimumPositionBoundSlack = min(estimate.relativePositionErrorBound-physicalError(:,1));
    metrics.meanPositionErrorBound = mean(estimate.relativePositionErrorBound(evaluation));
    metrics.maximumPositionErrorBound = max(estimate.relativePositionErrorBound(evaluation));
    metrics.estimatedDomainValidFraction = mean(localPhysicalDomain(estimate.targetState(evaluation,:),cfg));
    metrics.truthOperatingDomainValid = all(localPhysicalDomain(truth.targetTransformedState,cfg));
    if options.targetMotion == "varying"
        metrics.truthOperatingDomainValid = metrics.truthOperatingDomainValid ...
            && cfg.target.model.scalarAccelerationRateMaximum >= 1.5 ...
            && cfg.target.model.curvatureRateMaximum >= 0.0175;
    elseif options.targetMotion == "laneChange"
        profile = localTargetMotionProfile("laneChange",options.duration,0,0);
        metrics.truthOperatingDomainValid = metrics.truthOperatingDomainValid ...
            && cfg.target.model.curvatureRateMaximum+1e-12 >= profile.curvatureRateMaximum;
    end
    metrics.domainViolationQuantities = unique( ...
        finalOutput.estimatedOperatingDomainAudit.violationQuantities, ...
        "stable");
end

function valid = localPhysicalDomain(state,cfg)
    speed = vecnorm(state(:,3:4),2,2);
    acceleration = sum(state(:,3:4).*state(:,5:6),2)./max(speed,realmin);
    curvature = sum([-state(:,4),state(:,3)].*state(:,5:6),2)./max(speed,1.0e-100).^3;
    domain = cfg.target.domain;
    valid = speed >= domain.speedMinimum-1.0e-7 & speed <= domain.speedMaximum+1.0e-7 ...
        & abs(acceleration) <= domain.scalarAccelerationMaximum+1.0e-7 ...
        & abs(curvature) <= sin(domain.sideslipMaximum)/domain.rearAxleDistance+1.0e-7 ...
        & vecnorm(state(:,1:2),2,2) <= domain.relativePositionMaximum+1.0e-7;
end

function lag = localCurvatureLag(truth,estimate,options)
    lag = NaN;
    if options.targetMotion == "retained"
        return
    end
    q = estimate.targetState(:,3:4);
    s = estimate.targetState(:,5:6);
    curvature = sum([-q(:,2),q(:,1)].*s,2)./vecnorm(q,2,2).^3;
    profile = localTargetMotionProfile(options.targetMotion,options.duration,0,0);
    selected = truth.time >= profile.window(1) & truth.time <= profile.window(2);
    candidates = (-1:options.configuration.runtime.samplePeriod:2).';
    error = Inf(size(candidates));
    for index = 1:numel(candidates)
        delayedTruth = interp1(truth.time,truth.targetCurvature, ...
            truth.time(selected)-candidates(index),"linear",NaN);
        error(index) = mean((curvature(selected)-delayedTruth).^2,"omitnan");
    end
    [~,index] = min(error);
    lag = candidates(index);
end

function value = localVectorRmse(error, evaluation)
    magnitude = vecnorm(error, 2, 2);
    value = sqrt(mean(magnitude(evaluation).^2));
end

function localReport(metrics,design,samplePeriod)
    fprintf("\nMultistage high-gain NRMM observer\n");
    fprintf("  sample period: %.3g s; target bandwidth: %.4g /s\n", ...
        samplePeriod,design.target.bandwidth);
    fprintf("  position / velocity / acceleration RMSE: %.6g m / %.6g m/s / %.6g m/s^2\n", ...
        metrics.relativePositionRmse,metrics.targetVelocityRmse,metrics.targetAccelerationRmse);
    fprintf("  continuous target decay: %.4g /s; position ultimate bound: %.4g m\n", ...
        design.target.lambda,design.ultimateBounds.relativePosition);
    fprintf("  These continuous-flow bounds are not digital samplewise certificates.\n");
    fprintf("  mean / maximum step time: %.4g / %.4g ms\n", ...
        metrics.meanStepMilliseconds,metrics.maximumStepMilliseconds);
    fprintf("  truth domain valid: %d; estimated domain fraction: %.4f\n", ...
        metrics.truthOperatingDomainValid,metrics.estimatedDomainValidFraction);
end

function localPlot(result)
    figure(Name="Cascaded measured-input NRMM observer scenario");
    tiledlayout(2, 2);
    nexttile;
    plot(result.truth.egoPosition(:, 1), result.truth.egoPosition(:, 2), ...
        LineWidth=1.5);
    hold on;
    plot(result.truth.targetPosition(:, 1), ...
        result.truth.targetPosition(:, 2), LineWidth=1.5);
    axis equal;
    grid on;
    legend("ego", "target", Location="best");
    title("Truth trajectories");
    nexttile;
    plot(result.truth.time, result.estimate.egoState(:, [1, 4]) ...
        - result.truth.egoPosition);
    grid on;
    title("Ego position error (m)");
    nexttile;
    plot(result.truth.time, result.estimate.targetState(:, 1:2) ...
        - result.truth.targetTransformedState(:, 1:2));
    grid on;
    title("Relative-position (rho) error (m)");
    nexttile;
    plot(result.truth.time, result.estimate.targetVelocityInertial ...
        - result.truth.targetVelocity);
    grid on;
    title("Target inertial-velocity error (m/s)");
end

function rotated = localRotateRows(angle, vectors)
    rotated = [ ...
        cos(angle).*vectors(:, 1)-sin(angle).*vectors(:, 2), ...
        sin(angle).*vectors(:, 1)+cos(angle).*vectors(:, 2)];
end

function rotation = localRotation(angle)
    rotation = [cos(angle), -sin(angle); sin(angle), cos(angle)];
end

function value = localWrapToPi(value)
    value = mod(value+pi, 2.0*pi)-pi;
end

function options = localOptions(varargin)
    localAddProjectPaths();
    parser = inputParser;
    parser.FunctionName = mfilename;
    addParameter(parser, "Plot", true, ...
        @(x) islogical(x) && isscalar(x));
    addParameter(parser, "Report", true, ...
        @(x) islogical(x) && isscalar(x));
    addParameter(parser, "Seed", 7, ...
        @(x) isnumeric(x) && isscalar(x) && isfinite(x));
    addParameter(parser, "Duration", 12.0, ...
        @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0.0);
    addParameter(parser, "NoiseModel", "none", ...
        @(x) isstring(x) || ischar(x));
    addParameter(parser, "Config", nrmmTrackingConfig(), @(x) isstruct(x) && isscalar(x));
    addParameter(parser, "DropoutIntervals", zeros(0,2), ...
        @(x) isnumeric(x) && size(x,2) == 2 && all(isfinite(x), "all") && all(x(:,2) >= x(:,1)));
    addParameter(parser,"TargetMotion","retained", ...
        @(x) any(string(x) == ["retained","varying","laneChange"]));
    addParameter(parser,"EgoManeuver","retained", ...
        @(x) any(string(x) == ["retained","straight","aggressive"]));
    addParameter(parser,"TargetInitialPosition",[25.0,4.0], ...
        @(x) isnumeric(x) && isequal(size(x),[1,2]) && all(isfinite(x)));
    addParameter(parser,"TargetInitialHeading",0.10, ...
        @(x) isnumeric(x) && isscalar(x) && isfinite(x));
    addParameter(parser,"TargetInitialSpeed",12.5, ...
        @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
    addParameter(parser,"NoiseScale",1.0, ...
        @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
    addParameter(parser,"InitialOffsetScale",1.0, ...
        @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
    addParameter(parser,"RuntimeFunction",@onlineNrmmTrackingRuntime,@(x) isa(x,"function_handle"));
    addParameter(parser,"DesignFunction",@synthesizeNrmmObserverGains,@(x) isa(x,"function_handle"));
    parse(parser, varargin{:});
    options = struct( ...
        "runtimeFunction", parser.Results.RuntimeFunction, "designFunction", parser.Results.DesignFunction, ...
        "targetMotion", string(parser.Results.TargetMotion), ...
        "egoManeuver", string(parser.Results.EgoManeuver), ...
        "targetInitialPosition", double(parser.Results.TargetInitialPosition), ...
        "targetInitialHeading", double(parser.Results.TargetInitialHeading), ...
        "targetInitialSpeed", double(parser.Results.TargetInitialSpeed), ...
        "noiseScale", double(parser.Results.NoiseScale), ...
        "initialOffsetScale", double(parser.Results.InitialOffsetScale), ...
        "configuration", parser.Results.Config, "dropoutIntervals", parser.Results.DropoutIntervals, ...
        "plot", parser.Results.Plot, ...
        "report", parser.Results.Report, ...
        "seed", double(parser.Results.Seed), ...
        "duration", double(parser.Results.Duration), ...
        "noiseModel", lower(string(parser.Results.NoiseModel)));
end

function localAddProjectPaths()
    repoRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(repoRoot, "config"));
    addpath(fullfile(repoRoot, "estimator"));
end
