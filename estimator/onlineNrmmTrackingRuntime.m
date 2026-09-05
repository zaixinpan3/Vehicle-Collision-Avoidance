function varargout = onlineNrmmTrackingRuntime(action, varargin)
% onlineNrmmTrackingRuntime Sample-native exact-flow finite-window estimator.
% 'step' assimilates a frame at time t and predicts to t+samplePeriod. Relative
% tracking uses GNSS speed, body IMU, and relative gyro increments; absolute
% yaw is a separate circular set used only for inertial output reconstruction.
% Each target has a nominal CCA trajectory fit and an independent conservative
% physical-state enclosure. Empty sets are reported and require an explicit
% reinitialization; fit residuals never become deterministic error radii.
% Optional frame.egoMotion supplies duration, yawIncrement, translation (in
% the start ego frame), yawErrorMaximum, and translationErrorMaximum. Its
% bounds must cover the ENTIRE interval. Otherwise holds use explicit domain
% and optional intersample derivative bounds in nrmmTrackingConfig.

    switch lower(string(action))
        case "initialize"
            varargout{1} = localInitialize(varargin{:});
        case "step"
            [varargout{1}, varargout{2}] = localStep(varargin{:});
        case "output"
            runtime = varargin{1};
            input = localSynchronizedInput(varargin{2}, runtime);
            localCheckTime(runtime, input);
            varargout{1} = localOutput(runtime, input);
        case "resettarget"
            varargout{1} = localResetTarget(varargin{:});
        otherwise
            error("onlineNrmmTrackingRuntime:invalidAction", "Unknown runtime action.");
    end
end

function runtime = localInitialize(cfg, options, design)
    localAddProjectPaths();
    if nargin < 1 || isempty(cfg)
        cfg = nrmmTrackingConfig();
    end
    if nargin < 2
        options = struct();
    end
    if nargin < 3 || isempty(design)
        design = nrmmWindowEstimatorDesign(cfg);
    end
    if abs(cfg.runtime.samplePeriod-design.samplePeriod) > localTimeTolerance(design.samplePeriod)
        error("onlineNrmmTrackingRuntime:inconsistentSamplePeriod", ...
            "Runtime and design sample periods must agree.");
    end
    if ~isfield(design, "method") || design.method ~= "exact-flow-finite-window" ...
            || ~isequaln(design.configuration, cfg)
        error("onlineNrmmTrackingRuntime:inconsistentDesign", ...
            "Supply nrmmWindowEstimatorDesign for the same configuration.");
    end
    count = localOptionPositiveInteger(options, "targetCount", 1);
    [identifiers, stable] = localTargetIdentifiers(options, count);
    runtime = struct("observerDesign", design, "samplePeriod", design.samplePeriod, ...
        "currentTime", localOptionFiniteScalar(options, "initialTime", 0), ...
        "positionEstimate", localRequiredOptionVector(options, "egoInitialPosition", 2), ...
        "yawEstimate", localRequiredOptionScalar(options, "egoInitialYaw"), ...
        "bodyVelocityEstimate", localRequiredOptionVector(options, "egoInitialBodyVelocity", 2), ...
        "targetState", localInitialTargets(options, count), ...
        "targetCount", count, "targetIdentifiers", identifiers, ...
        "targetIdentifiersStable", stable, "yawSet", [-pi, pi], ...
        "history", struct([]), "inputBoundsConsistent", true, ...
        "lastGnssTime", NaN, "lastImuTime", NaN, "lastGyroscopeTime", NaN, ...
        "lastRadarTime", NaN(count, 1), "lastInputSampleTime", NaN);
    runtime.targetSets = repmat(nrmmTargetSet("initialize", design.target.domain), count, 1);
    runtime.fitDiagnostics = repmat({struct("status", "initialization", ...
        "available", false, "measurementConsistent", false)}, count, 1);
    runtime.lastBodyInformation = struct("nonempty", false, "radius", Inf);
    runtime.lastMotion = struct("yawIncrement", 0, "translation", [0;0], ...
        "yawErrorMaximum", pi, "translationErrorMaximum", Inf, "source", "initialization");
    runtime.lastIntervalDomainAudit = localAudit(runtime);
    runtime.estimatedDomainAudit = runtime.lastIntervalDomainAudit;
    runtime.targetCertifiedSpeedDomainValid = runtime.estimatedDomainAudit.speedValid;
    runtime.lastIntervalTargetCertifiedSpeedDomainValid = runtime.targetCertifiedSpeedDomainValid;
    runtime.minimumReconstructedTargetSpeed = runtime.estimatedDomainAudit.minimumTargetSpeed;
end

function [runtime, output] = localStep(runtime, frame)
    input = localSynchronizedInput(frame, runtime);
    localCheckTime(runtime, input);
    cfg = runtime.observerDesign.configuration;
    body = nrmmBodyVelocitySet(input.gnssVelocity, input.yawRate, cfg);
    runtime.lastBodyInformation = body;
    runtime.inputBoundsConsistent = runtime.inputBoundsConsistent && body.nonempty ...
        && abs(input.yawRate) <= cfg.ego.domain.yawRateMaximum ...
            +cfg.measurement.gyroscope.noiseMaximum;
    if body.nonempty
        velocity = body.representative;
    else
        velocity = runtime.bodyVelocityEstimate;
    end
    motion = localEgoMotion(frame, input, velocity, body.radius, cfg);
    runtime.lastMotion = motion;
    runtime.yawSet = nrmmCircularSet("intersect", runtime.yawSet, body.yawArcs);
    sampleYaw = localYawRepresentative(runtime.yawSet, runtime.yawEstimate);
    runtime.yawSet = nrmmCircularSet("predict", runtime.yawSet, ...
        motion.yawIncrement, motion.yawErrorMaximum);
    runtime.yawEstimate = localYawRepresentative(runtime.yawSet, sampleYaw+motion.yawIncrement);
    runtime.positionEstimate = input.gnssPosition+localRotation(sampleYaw)*motion.translation;
    runtime.bodyVelocityEstimate = motion.bodyVelocityEnd;
    sample = struct("time", input.time, "radar", input.radarRelativePosition, ...
        "available", input.radarDetectionAvailable, "motion", motion);
    if isempty(runtime.history)
        runtime.history = sample;
    else
        runtime.history(end+1) = sample;
    end
    retain = [runtime.history.time] >= input.time-cfg.window.duration-localTimeTolerance(input.time);
    runtime.history = runtime.history(retain);
    [poses, queryPose] = localWindowPoses(runtime.history, cfg.window.numericalAllowance);
    queryTime = input.time+runtime.samplePeriod;
    domain = runtime.observerDesign.target.domain;
    for index = 1:runtime.targetCount
        prior = runtime.targetSets(index);
        if input.radarDetectionAvailable(index)
            y = input.radarRelativePosition(index, :).';
            prior.lower(1:2) = max(prior.lower(1:2), y-cfg.measurement.radar.noiseMaximum);
            prior.upper(1:2) = min(prior.upper(1:2), y+cfg.measurement.radar.noiseMaximum);
        end
        prior = nrmmTargetSet("predict", prior, runtime.samplePeriod, motion, runtime.observerDesign);
        [times, points, radii] = localWindowMeasurements(runtime.history, poses, index, cfg);
        seed = runtime.targetState(:, index);
        if input.radarDetectionAvailable(index)
            seed(1:2) = input.radarRelativePosition(index, :).';
        end
        nominal = localPrediction(seed, motion, runtime.samplePeriod, domain);
        fixedNominal = kron(eye(3), localRotation(queryPose.yawIncrement))*nominal;
        fixedNominal(1:2) = fixedNominal(1:2)+queryPose.translation;
        if input.radarDetectionAvailable(index)
            fit = fitNrmmTrajectoryWindow(times, points, queryTime, fixedNominal, runtime.observerDesign);
        else
            fit = fitNrmmTrajectoryWindow(zeros(0,1), zeros(2,0), queryTime, ...
                fixedNominal, runtime.observerDesign);
            fit.status = "coasting";
        end
        if fit.available
            fixed = fit.state;
            fixed(1:2) = fixed(1:2)-queryPose.translation;
            nominal = kron(eye(3), localRotation(-queryPose.yawIncrement))*fixed;
        end
        fit.measurementConsistent = fit.available && all(fit.residual <= radii+cfg.window.numericalAllowance);
        outer = nrmmTargetSet("window", times, points, radii, queryTime, queryPose, runtime.observerDesign);
        runtime.targetSets(index) = nrmmTargetSet("intersect", prior, outer);
        runtime.targetState(:, index) = nominal;
        runtime.fitDiagnostics{index} = fit;
    end
    runtime.currentTime = queryTime;
    runtime.lastInputSampleTime = input.time;
    runtime.lastGnssTime = input.time;
    runtime.lastImuTime = input.time;
    runtime.lastGyroscopeTime = input.time;
    runtime.lastRadarTime(input.radarDetectionAvailable) = input.time;
    runtime.lastIntervalDomainAudit = localAudit(runtime);
    runtime.estimatedDomainAudit.valid = runtime.estimatedDomainAudit.valid && runtime.lastIntervalDomainAudit.valid;
    runtime.estimatedDomainAudit.speedValid = runtime.estimatedDomainAudit.speedValid && runtime.lastIntervalDomainAudit.speedValid;
    runtime.estimatedDomainAudit.maximumRelativePosition = max(runtime.estimatedDomainAudit.maximumRelativePosition, ...
        runtime.lastIntervalDomainAudit.maximumRelativePosition);
    runtime.estimatedDomainAudit.violationQuantities = unique([runtime.estimatedDomainAudit.violationQuantities; ...
        runtime.lastIntervalDomainAudit.violationQuantities]);
    runtime.targetCertifiedSpeedDomainValid = runtime.estimatedDomainAudit.speedValid;
    runtime.lastIntervalTargetCertifiedSpeedDomainValid = runtime.lastIntervalDomainAudit.speedValid;
    runtime.minimumReconstructedTargetSpeed = min(runtime.minimumReconstructedTargetSpeed, ...
        runtime.lastIntervalDomainAudit.minimumTargetSpeed);
    output = localOutput(runtime, input);
end

function motion = localEgoMotion(frame, input, velocity, velocityRadius, cfg)
    h = cfg.runtime.samplePeriod;
    angle = h*input.yawRate;
    % Exact nominal ego integration for held body acceleration and yaw rate.
    if abs(angle) < 1.0e-3
        c1 = 1-angle^2/6+angle^4/120;
        s1 = angle/2-angle^3/24+angle^5/720;
        c2 = 0.5-angle^2/24+angle^4/720;
        s2 = angle/6-angle^3/120+angle^5/5040;
    else
        c1 = sin(angle)/angle;
        s1 = (1-cos(angle))/angle;
        c2 = (1-cos(angle))/angle^2;
        s2 = (angle-sin(angle))/angle^2;
    end
    translation = h*velocity+h^2*[c2,-s2;s2,c2]*input.bodyAcceleration;
    velocityEnd = localRotation(-angle)*(velocity+h*[c1,-s1;s1,c1]*input.bodyAcceleration);
    yawError = min(h*(cfg.ego.domain.yawRateMaximum+abs(input.yawRate)), ...
        h*cfg.measurement.gyroscope.noiseMaximum ...
        +h^2*cfg.ego.intersample.yawAccelerationMaximum/2);
    translationError = min(h*cfg.ego.domain.speedMaximum+norm(translation), ...
        h*velocityRadius+h^2*(cfg.ego.intersample.accelerationMaximum+norm(input.bodyAcceleration))/2);
    motion = struct("duration", h, "yawIncrement", angle, "translation", translation, ...
        "yawErrorMaximum", yawError+cfg.window.numericalAllowance, ...
        "translationErrorMaximum", translationError+cfg.window.numericalAllowance, ...
        "bodyVelocityEnd", velocityEnd, "source", "held-samples-with-explicit-enclosures");
    if isfield(frame, "egoMotion")
        supplied = frame.egoMotion;
        fields = ["duration", "yawIncrement", "translation", "yawErrorMaximum", "translationErrorMaximum"];
        if ~isstruct(supplied) || ~isscalar(supplied) || ~all(isfield(supplied, fields))
            error("onlineNrmmTrackingRuntime:invalidEgoMotion", "Incomplete integrated ego-motion contract.");
        end
        scalars = [supplied.duration; supplied.yawIncrement; supplied.yawErrorMaximum; supplied.translationErrorMaximum];
        if ~isequal(size(scalars), [4,1]) || any(~isfinite(scalars)) ...
                || any(scalars(3:4) < 0) || ~isequal(size(supplied.translation), [2,1]) ...
                || any(~isfinite(supplied.translation)) || abs(supplied.duration-h) > localTimeTolerance(h)
            error("onlineNrmmTrackingRuntime:invalidEgoMotion", "Invalid integrated ego-motion values or duration.");
        end
        for field = fields
            motion.(field) = supplied.(field);
        end
        motion.source = "supplied-bounded-increments";
    end
end

function [poses, pose] = localWindowPoses(history, padding)
    pose = struct("translation", [0;0], "yawIncrement", 0, ...
        "translationErrorMaximum", 0, "yawErrorMaximum", 0);
    poses = repmat(pose, numel(history), 1);
    for index = 1:numel(history)
        poses(index) = pose;
        motion = history(index).motion;
        pose.translationErrorMaximum = pose.translationErrorMaximum ...
            +motion.translationErrorMaximum+2*norm(motion.translation) ...
            *sin(min(pi, pose.yawErrorMaximum)/2)+padding;
        pose.translation = pose.translation+localRotation(pose.yawIncrement)*motion.translation;
        pose.yawIncrement = pose.yawIncrement+motion.yawIncrement;
        pose.yawErrorMaximum = pose.yawErrorMaximum+motion.yawErrorMaximum+padding;
    end
end

function [time, points, radius] = localWindowMeasurements(history, poses, target, cfg)
    selected = arrayfun(@(sample) sample.available(target), history);
    indices = find(selected);
    time = [history(selected).time].';
    points = zeros(2, numel(indices));
    radius = zeros(numel(indices), 1);
    for slot = 1:numel(indices)
        index = indices(slot);
        pose = poses(index);
        points(:, slot) = pose.translation+localRotation(pose.yawIncrement)*history(index).radar(target, :).';
        radius(slot) = pose.translationErrorMaximum+cfg.measurement.radar.noiseMaximum ...
            +2*cfg.target.domain.relativePositionMaximum*sin(min(pi, pose.yawErrorMaximum)/2) ...
            +cfg.window.numericalAllowance;
    end
end

function physical = localPrediction(prior, motion, h, domain)
    q = prior(3:4);
    speed = max(norm(q), domain.speedMinimum);
    course = atan2(q(2), q(1));
    acceleration = min(max(dot(q, prior(5:6))/speed, -domain.scalarAccelerationMaximum), domain.scalarAccelerationMaximum);
    % A prediction leaving the positive chart uses an explicitly constrained
    % nominal boundary value. The independent hard set is not clipped to it.
    speed = min(speed, domain.speedMaximum);
    acceleration = min(max(acceleration, (domain.speedMinimum-speed)/h), (domain.speedMaximum-speed)/h);
    curvature = min(max(dot([-q(2);q(1)], prior(5:6))/speed^3, ...
        -domain.curvatureMaximum), domain.curvatureMaximum);
    [~, physical] = nrmmExactFlow([prior(1:2);course;speed;acceleration;curvature], ...
        h, motion.yawIncrement, motion.translation);
end

function runtime = localResetTarget(runtime, index, physical)
    validateattributes(index, {'double'}, {'scalar','integer','positive','<=',runtime.targetCount});
    validateattributes(physical, {'double'}, {'size',[6,1],'finite'});
    runtime.targetState(:, index) = physical;
    runtime.targetSets(index) = nrmmTargetSet("initialize", runtime.observerDesign.target.domain);
    runtime.lastRadarTime(index) = NaN;
    for sample = 1:numel(runtime.history)
        runtime.history(sample).available(index) = false;
        runtime.history(sample).radar(index, :) = NaN;
    end
    runtime.fitDiagnostics{index} = struct("status", "explicitTrackReset", ...
        "available", false, "measurementConsistent", false);
end

function certificate = localCertificate(runtime, index)
    box = runtime.targetSets(index);
    radius = nrmmTargetSet("radius", box, runtime.targetState(:, index));
    nonempty = all(box.lower <= box.upper) && runtime.inputBoundsConsistent;
    if nonempty
        status = "outerNonempty";
    else
        status = "inconsistent";
        radius(:) = Inf;
    end
    certificate = struct("status", status, "outerNonempty", nonempty, ...
        "relativePosition", radius(1), "targetVelocity", radius(2), ...
        "targetAcceleration", radius(3), "lower", box.lower, "upper", box.upper, ...
        "assumptions", "declared-domain-sensor-intersample-and-model-rate-bounds", ...
        "method", "analytic-Taylor-outer-enclosure", "machineVerified", false);
end

function audit = localAudit(runtime)
    state = runtime.targetState;
    domain = runtime.observerDesign.target.domain;
    speed = vecnorm(state(3:4, :));
    a = sum(state(3:4, :).*state(5:6, :))./max(speed, realmin);
    kappa = sum([-state(4, :);state(3, :)].*state(5:6, :))./max(speed, realmin).^3;
    position = vecnorm(state(1:2, :));
    speedValid = all(speed >= domain.speedMinimum-1.0e-7 & speed <= domain.speedMaximum+1.0e-7);
    quantities = strings(0,1);
    if ~speedValid
        quantities(end+1) = "targetSpeed";
    end
    if any(abs(a) > domain.scalarAccelerationMaximum+1.0e-7)
        quantities(end+1) = "targetScalarAcceleration";
    end
    if any(abs(kappa) > domain.curvatureMaximum+1.0e-7)
        quantities(end+1) = "targetCurvature";
    end
    if any(position > domain.relativePositionMaximum+1.0e-7)
        quantities(end+1) = "relativePosition";
    end
    audit = struct("valid", isempty(quantities), "speedValid", speedValid, ...
        "minimumTargetSpeed", min(speed), "maximumRelativePosition", max(position), ...
        "violationQuantities", quantities(:), "estimatedStateChecked", true, "trueTrajectoryChecked", false);
end

function output = localOutput(runtime, input)
    rotation = localRotation(runtime.yawEstimate);
    velocity = rotation*runtime.bodyVelocityEstimate;
    acceleration = rotation*input.bodyAcceleration;
    egoState = [runtime.positionEstimate(1); velocity(1); acceleration(1); ...
        runtime.positionEstimate(2); velocity(2); acceleration(2)];
    output = struct("time", runtime.currentTime, "stateTime", runtime.currentTime, ...
        "inputSampleTime", input.time, "lastGnssTime", runtime.lastGnssTime, ...
        "lastImuTime", runtime.lastImuTime, "lastGyroscopeTime", runtime.lastGyroscopeTime, ...
        "lastRadarTime", runtime.lastRadarTime, "egoState", egoState, ...
        "egoPositionInertial", runtime.positionEstimate, "egoVelocityInertial", velocity, ...
        "egoBodyVelocity", runtime.bodyVelocityEstimate, "egoAccelerationBody", input.bodyAcceleration, ...
        "egoAccelerationInertial", acceleration, "egoYaw", runtime.yawEstimate, ...
        "egoYawRate", input.yawRate, "egoYawRateMeasured", input.yawRate, ...
        "egoYawSet", runtime.yawSet, "egoYawRadius", localYawRadius(runtime.yawSet, runtime.yawEstimate), ...
        "bodyVelocityInformation", runtime.lastBodyInformation, "egoMotion", runtime.lastMotion, ...
        "inputBoundsConsistent", runtime.inputBoundsConsistent, ...
        "radarDetectionAvailable", input.radarDetectionAvailable, ...
        "targetCertifiedSpeedDomainValid", runtime.targetCertifiedSpeedDomainValid, ...
        "targetCertifiedSpeedDomainValidThisInterval", runtime.lastIntervalTargetCertifiedSpeedDomainValid, ...
        "minimumReconstructedTargetSpeed", runtime.minimumReconstructedTargetSpeed, ...
        "estimatedOperatingDomainValid", runtime.estimatedDomainAudit.valid, ...
        "estimatedOperatingDomainValidThisInterval", runtime.lastIntervalDomainAudit.valid, ...
        "estimatedOperatingDomainAudit", runtime.estimatedDomainAudit, ...
        "lastIntervalOperatingDomainAudit", runtime.lastIntervalDomainAudit, ...
        "targetStates", runtime.targetState.', "targetEstimates", localTargetEstimates(runtime, input));
end

function localCheckTime(runtime, input)
    if abs(input.time-runtime.currentTime) > localTimeTolerance([input.time;runtime.currentTime])
        error("onlineNrmmTrackingRuntime:offSampleGrid", ...
            "The synchronized frame time must equal the estimator time.");
    end
end

function angle = localYawRepresentative(arcs, fallback)
    if isempty(arcs)
        angle = localWrapToPi(fallback);
        return
    end
    connected = arcs;
    if size(arcs,1) > 1 && arcs(1,1) == -pi && arcs(end,2) == pi
        connected = [arcs(2:end-1,:); arcs(end,1), arcs(1,2)+2*pi];
    end
    centres = mean(connected,2);
    [~, index] = min(abs(localWrapToPi(centres-fallback)));
    angle = localWrapToPi(centres(index));
end

function radius = localYawRadius(arcs, angle)
    if isempty(arcs)
        radius = Inf;
        return
    end
    antipode = localWrapToPi(angle+pi);
    if any(arcs(:,1) <= antipode & antipode <= arcs(:,2))
        radius = pi;
    else
        radius = max(abs(localWrapToPi(arcs(:)-angle)));
    end
end

function estimates = localTargetEstimates(runtime, observerInput)
    design = runtime.observerDesign;
    rotation = localRotation(runtime.yawEstimate);
    planarCross = [0.0, -1.0; 1.0, 0.0];
    ego = struct( ...
        "bodyVelocity", runtime.bodyVelocityEstimate, ...
        "yawRate", observerInput.yawRate);
    measuredBodyAcceleration = observerInput.bodyAcceleration;
    estimates = struct([]);
    for targetIdx = 1:runtime.targetCount
        [~, estimate] = nrmmTargetTrackerDerivative( ...
            runtime.targetState(:, targetIdx), ego, ...
            design.target.domain);
        estimate.targetCurvature = dot([-estimate.targetVelocity(2); ...
            estimate.targetVelocity(1)], estimate.targetAcceleration) ...
            /max(estimate.targetSpeed, realmin)^3;
        estimate.errorBound = localCertificate(runtime, targetIdx);
        estimate.windowFit = runtime.fitDiagnostics{targetIdx};
        estimate.certifiedSpeedMinimum = ...
            design.target.domain.speedMinimum;
        estimate.certifiedSpeedDomainValid = ...
            estimate.targetSpeed >= estimate.certifiedSpeedMinimum;
        estimate.relativePositionDerivative = ...
            estimate.targetVelocity-runtime.bodyVelocityEstimate ...
            - observerInput.yawRate*planarCross ...
                * estimate.relativePosition;
        estimate.velocityDifferenceEgoFrame = ...
            estimate.targetVelocity-runtime.bodyVelocityEstimate;
        estimate.accelerationDifferenceEgoFrame = ...
            estimate.targetAcceleration-measuredBodyAcceleration;
        estimate.targetPositionInertial = runtime.positionEstimate ...
            + rotation*estimate.relativePosition;
        estimate.targetVelocityInertial = ...
            rotation*estimate.targetVelocity;
        estimate.targetAccelerationInertial = ...
            rotation*estimate.targetAcceleration;
        estimate.targetCourseAngleInertial = localWrapToPi( ...
            runtime.yawEstimate+estimate.targetCourseAngleEgoFrame);
        estimate.targetHeadingInertial = localWrapToPi( ...
            estimate.targetCourseAngleInertial ...
            - estimate.targetSideslip);
        estimate.stateTime = runtime.currentTime;
        estimate.estimateTime = runtime.currentTime;
        estimate.lastRadarTime = runtime.lastRadarTime(targetIdx);
        % Retained as a compatibility alias with corrected semantics.
        estimate.measurementTime = estimate.lastRadarTime;
        if runtime.targetIdentifiersStable
            estimate.trackId = runtime.targetIdentifiers(targetIdx);
        else
            estimate.temporarySlotIdentifier = ...
                "target-slot-" + string(targetIdx);
        end
        if isempty(estimates)
            estimates = estimate;
        else
            estimates(targetIdx, 1) = estimate;
        end
    end
end

function observerInput = localSynchronizedInput(frame, runtime)
    [radarRelativePosition, radarDetectionAvailable] = ...
        localRadarPosition(frame, runtime);
    observerInput = struct( ...
        "time", localRequiredFrameScalar(frame, "time"), ...
        "gnssPosition", [ ...
            localRequiredFrameScalar(frame, "xGps"); ...
            localRequiredFrameScalar(frame, "yGps")], ...
        "gnssVelocity", [ ...
            localRequiredFrameScalar(frame, "vxGps"); ...
            localRequiredFrameScalar(frame, "vyGps")], ...
        "bodyAcceleration", [ ...
            localRequiredFrameScalar(frame, "longitudinalAcceleration"); ...
            localRequiredFrameScalar(frame, "lateralAcceleration")], ...
        "yawRate", localRequiredFrameScalar(frame, "yawRateMeasured"), ...
        "radarRelativePosition", radarRelativePosition, ...
        "radarDetectionAvailable", radarDetectionAvailable, ...
        "radarTargetIdentifiers", runtime.targetIdentifiers);
end

function [radar, available] = localRadarPosition(frame, runtime)
    targetCount = runtime.targetCount;
    if ~isstruct(frame) || ~isscalar(frame) ...
            || ~isfield(frame, "radarRelativePosition") ...
            || isempty(frame.radarRelativePosition)
        error("onlineNrmmTrackingRuntime:missingSynchronizedRadar", ...
            "Every observer frame must contain synchronized radar positions.");
    end
    radar = double(frame.radarRelativePosition);
    if targetCount == 1 && isvector(radar) && numel(radar) == 2
        radar = radar(:).';
    end
    available = true(targetCount, 1);
    if isfield(frame, "radarDetectionAvailable") ...
            && ~isempty(frame.radarDetectionAvailable)
        available = frame.radarDetectionAvailable;
        if targetCount == 1 && isscalar(available)
            available = available(:);
        end
        if ~islogical(available)
            if isnumeric(available) && all(isfinite(available), "all") ...
                    && all(available == 0.0 | available == 1.0, "all")
                available = logical(available);
            else
                error("onlineNrmmTrackingRuntime:" ...
                    + "invalidRadarDetectionAvailability", ...
                    "radarDetectionAvailable must be logical.");
            end
        end
    end
    validSize = isequal(size(radar), [targetCount, 2]) ...
        && isequal(size(available), [targetCount, 1]);
    if ~validSize
        error("onlineNrmmTrackingRuntime:invalidSynchronizedRadar", ...
            "Radar data must contain one two-vector and one flag per target.");
    end
    identifiersRequired = targetCount > 1;
    identifiersProvided = isfield(frame, "radarTargetIdentifiers") ...
        && ~isempty(frame.radarTargetIdentifiers);
    if identifiersRequired && ~identifiersProvided
        error("onlineNrmmTrackingRuntime:missingRadarTargetIdentifiers", ...
            "Multi-target frames must contain radarTargetIdentifiers.");
    end
    if identifiersProvided
        inputIdentifiers = string(frame.radarTargetIdentifiers(:));
        identifiersValid = numel(inputIdentifiers) == targetCount ...
            && all(~ismissing(inputIdentifiers)) ...
            && all(strlength(inputIdentifiers) > 0) ...
            && numel(unique(inputIdentifiers)) == targetCount;
        if ~identifiersValid
            error("onlineNrmmTrackingRuntime:" ...
                + "invalidRadarTargetIdentifiers", ...
                "radarTargetIdentifiers must be unique nonempty text.");
        end
        [knownIdentifiers, inputOrder] = ismember( ...
            runtime.targetIdentifiers, inputIdentifiers);
        if ~all(knownIdentifiers) ...
                || ~all(ismember(inputIdentifiers, ...
                    runtime.targetIdentifiers))
            error("onlineNrmmTrackingRuntime:" ...
                + "unknownRadarTargetIdentifier", ...
                "Every radar identifier must match an initialized track.");
        end
        radar = radar(inputOrder, :);
        available = available(inputOrder);
    end
    availableRowsFinite = all(isfinite(radar(available, :)), "all");
    unavailableRowsMissing = all( ...
        isnan(radar(~available, :)), "all");
    if ~availableRowsFinite || ~unavailableRowsMissing
        error("onlineNrmmTrackingRuntime:invalidSynchronizedRadar", ...
            "Detected radar rows must be finite and unavailable rows NaN.");
    end
end

function targetState = localInitialTargets(options, targetCount)
    if ~isfield(options, "targetInitialState") ...
            || isempty(options.targetInitialState)
        error("onlineNrmmTrackingRuntime:missingInitialState", ...
            "options.targetInitialState is required.");
    end
    targetState = double(options.targetInitialState);
    if targetCount == 1 && isvector(targetState) ...
            && numel(targetState) == 6
        targetState = targetState(:);
    end
    if ~isequal(size(targetState), [6, targetCount]) ...
            || any(~isfinite(targetState), "all")
        error("onlineNrmmTrackingRuntime:invalidInitialState", ...
            "targetInitialState must be a finite 6-by-targetCount " ...
            + "matrix of [rho; q; s] columns.");
    end
end

function [identifiers, stable] = ...
        localTargetIdentifiers(options, targetCount)
    stable = false;
    identifiers = "target-slot-" + string((1:targetCount).');
    if ~isfield(options, "targetIdentifiers") ...
            || isempty(options.targetIdentifiers)
        if targetCount > 1
            error("onlineNrmmTrackingRuntime:" ...
                + "missingTargetIdentifiers", ...
                "Multi-target initialization requires stable target identifiers.");
        end
        return;
    end
    identifiers = string(options.targetIdentifiers(:));
    if numel(identifiers) ~= targetCount ...
            || any(ismissing(identifiers)) ...
            || any(strlength(identifiers) == 0) ...
            || numel(unique(identifiers)) ~= targetCount
        error("onlineNrmmTrackingRuntime:invalidTargetIdentifiers", ...
            "options.targetIdentifiers must contain one unique, nonempty " ...
            + "identifier per target.");
    end
    stable = true;
end

function value = localOptionPositiveInteger(options, fieldName, defaultValue)
    value = localOptionFiniteScalar(options, fieldName, defaultValue);
    if value < 1.0 || value ~= floor(value)
        error("onlineNrmmTrackingRuntime:invalidOption", ...
            "options.%s must be a positive integer.", fieldName);
    end
    value = double(value);
end

function value = localOptionFiniteScalar(options, fieldName, defaultValue)
    if isfield(options, fieldName) && ~isempty(options.(fieldName))
        value = double(options.(fieldName));
    else
        value = defaultValue;
    end
    if ~isscalar(value) || ~isfinite(value)
        error("onlineNrmmTrackingRuntime:invalidOption", ...
            "options.%s must be a finite scalar.", fieldName);
    end
end

function value = localRequiredOptionScalar(options, fieldName)
    if ~isfield(options, fieldName) || isempty(options.(fieldName))
        error("onlineNrmmTrackingRuntime:missingInitialState", ...
            "options.%s is required.", fieldName);
    end
    value = double(options.(fieldName));
    if ~isscalar(value) || ~isfinite(value)
        error("onlineNrmmTrackingRuntime:invalidInitialState", ...
            "options.%s must be a finite scalar.", fieldName);
    end
end

function value = localRequiredOptionVector(options, fieldName, count)
    if ~isfield(options, fieldName) || isempty(options.(fieldName))
        error("onlineNrmmTrackingRuntime:missingInitialState", ...
            "options.%s is required.", fieldName);
    end
    value = localOptionVector(options, fieldName, count, []);
end

function value = localOptionVector(options, fieldName, count, defaultValue)
    if isfield(options, fieldName) && ~isempty(options.(fieldName))
        value = double(options.(fieldName));
    else
        value = defaultValue;
    end
    if ~isequal(size(value), [count, 1]) || any(~isfinite(value))
        error("onlineNrmmTrackingRuntime:invalidInitialState", ...
            "options.%s must be a finite %d-by-1 vector.", ...
            fieldName, count);
    end
end

function value = localRequiredFrameScalar(frame, fieldName)
    if ~isstruct(frame) || ~isscalar(frame) ...
            || ~isfield(frame, fieldName) || isempty(frame.(fieldName))
        error("onlineNrmmTrackingRuntime:missingSynchronizedSignal", ...
            "Every observer frame must contain %s.", fieldName);
    end
    value = double(frame.(fieldName));
    if ~isscalar(value) || ~isfinite(value)
        error("onlineNrmmTrackingRuntime:invalidSynchronizedSignal", ...
            "frame.%s must be a finite scalar.", fieldName);
    end
end

function rotation = localRotation(yaw)
    rotation = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
end

function value = localWrapToPi(value)
    value = mod(value+pi, 2.0*pi)-pi;
end

function tolerance = localTimeTolerance(time)
    tolerance = 64.0*eps(max(1.0, max(abs(time))));
end

function localAddProjectPaths()
    estimatorRoot = fileparts(mfilename("fullpath"));
    repoRoot = fileparts(estimatorRoot);
    addpath(fullfile(repoRoot, "config"));
    addpath(estimatorRoot);
end
