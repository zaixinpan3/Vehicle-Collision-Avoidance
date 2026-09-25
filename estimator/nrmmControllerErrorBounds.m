function output = nrmmControllerErrorBounds(output, bound, input, design)
%nrmmControllerErrorBounds Publish state-time enclosures for control.
% The comparison state bounds yaw, body velocity and [rho; q; s]. Absolute
% position also uses the timestamped GNSS ball and the true speed domain.
% These are current estimation bounds, not a prediction-horizon certificate.

    age = max(0.0, output.stateTime-input.time);
    egoPosition = norm(output.egoPositionInertial-input.gnssPosition) ...
        + design.sensors.positionNoiseMaximum ...
        + design.operatingDomain.egoSpeedMaximum*age;
    egoYawRate = design.operatingDomain.egoYawRateMaximum+abs(input.yawRate);
    if isfinite(bound.holdBounds.yawAcceleration)
        egoYawRate = min(egoYawRate, design.sensors.gyroscopeNoiseMaximum ...
            + bound.holdBounds.yawAcceleration*age);
    elseif age == 0.0
        egoYawRate = design.sensors.gyroscopeNoiseMaximum;
    end
    velocityBounds = localVelocityBounds(output, bound, input, design, age);
    egoBounds = [egoPosition; egoPosition; bound.yaw; velocityBounds; egoYawRate];
    egoAvailable = bound.egoValid && bound.orientationSet.valid && all(isfinite(egoBounds));
    if ~egoAvailable
        egoBounds(:) = inf;
    end
    % These separate channels do not depend on an absolute orientation set.
    % An empty yaw intersection makes the full controller vector unavailable,
    % while retaining valid body-velocity, GNSS-position and gyro enclosures.
    output.egoPositionErrorBound = egoPosition;
    output.egoYawErrorBound = bound.yaw;
    output.egoBodyVelocityErrorBound = bound.bodyVelocity;
    output.egoYawRateErrorBound = egoYawRate;
    if ~bound.egoValid
        output.egoPositionErrorBound = Inf;
        output.egoBodyVelocityErrorBound = Inf;
        output.egoYawRateErrorBound = Inf;
    end
    output.controllerStateErrorBound = egoBounds;
    output.controllerErrorBoundSource = "nrmm-state-time-enclosure";
    output.controllerErrorBound = localCertificate( ...
        "ego-state-v1", output.stateTime, egoBounds, egoAvailable, bound.scope);

    if isempty(output.targetEstimate)
        return;
    end
    domain = design.target.domain;
    rotationError = 2*sin(min(bound.yaw, pi)/2);
    target = output.targetEstimate;
    components = bound.targetComponents;
    range = min(bound.trueRangeMaximum, norm(target.relativePosition));
    positionRadius = egoPosition+components(1)+range*rotationError;
    velocityRadius = components(2) ...
        + min(domain.speedMaximum, norm(target.targetVelocity))*rotationError;
    accelerationRadius = components(3) ...
        + min(domain.accelerationNormBound, norm(target.targetAcceleration))*rotationError;
    speed = norm(target.targetVelocity);
    courseRadius = pi;
    if components(2) < speed
        courseRadius = asin(min(1.0, components(2)/speed));
    end
    % Global Lipschitz bounds for the saturated inverse reconstruction.
    speedFloor = domain.speedMinimum;
    yawRateRadius = min(domain.yawRateMaximum+abs(target.targetYawRate), ...
        (domain.accelerationNormBound/speedFloor^2 ...
        + 2*domain.speedMaximum*domain.accelerationNormBound/speedFloor^3)*components(2) ...
        + domain.speedMaximum/speedFloor^2*components(3));
    sideslipRadius = min(domain.sideslipMaximum+abs(target.targetSideslip), ...
        domain.rearAxleDistance/cos(domain.sideslipMaximum) ...
        *(yawRateRadius/speedFloor ...
        + domain.yawRateMaximum/speedFloor^2*components(2)));
    yawRadius = min(pi, bound.yaw+courseRadius+sideslipRadius);
    values = [repmat(positionRadius, 2, 1); repmat(velocityRadius, 2, 1); ...
        repmat(accelerationRadius, 2, 1); yawRadius; yawRateRadius];
    historyAvailable = true;
    if isfield(bound,"targetHistory")
        history = nrmmTargetHistory("enclose",bound.targetHistory,output.stateTime);
        if history.samples > 0
            historyAvailable = history.available;
            center = [target.targetPositionInertial;target.targetVelocityInertial;target.targetAccelerationInertial];
            historyRadius = max(abs([history.lower-center,history.upper-center]),[],2);
            values(1:6) = min(values(1:6),historyRadius);
            velocityBall = norm(values(3:4));
            if velocityBall < norm(target.targetVelocityInertial)
                values(7) = min(values(7),asin(velocityBall/norm(target.targetVelocityInertial)) ...
                    +domain.sideslipMaximum+abs(target.targetSideslip));
            end
            if history.heading.available
                difference = abs(atan2(sin(history.heading.center-target.targetHeadingInertial), ...
                    cos(history.heading.center-target.targetHeadingInertial)));
                historyAvailable = historyAvailable && difference<=values(7)+history.heading.radius;
                values(7) = min(values(7),difference+history.heading.radius);
            end
            output.targetEstimate.measurementHistoryEnclosure = history;
        end
    end
    available = egoAvailable && bound.valid && all(isfinite(values));
    available = available && historyAvailable;
    if ~available
        values(:) = inf;
    end
    output.targetEstimate.controllerErrorBound = localCertificate( ...
        "target-state-v1", output.stateTime, values, available, bound.scope);
    output.targetEstimate.controllerErrorBoundSource = "nrmm-state-time-enclosure";
    output.targetEstimate.targetPositionInertialErrorBound = values(1:2);
    output.targetEstimate.targetVelocityInertialErrorBound = values(3:4);
    output.targetEstimate.targetAccelerationInertialErrorBound = values(5:6);
    output.targetEstimate.targetYawErrorBound = values(7);
    output.targetEstimate.targetYawRateErrorBound = values(8);
    % The controller reads scalarAccelerationMaximum as a bound on the
    % acceleration magnitude |a| = hypot(A, V*omega), not on the speed-rate A.
    output.targetEstimate.predictionMotion = [];
    if design.target.modelJerkMaximum == 0
        % Exact NRMM: constant speed-rate and constant sideslip, so a path of
        % constant curvature sin(beta)/l_r. The controller propagates every
        % such path through the estimate box and the NRMM parameter error
        % bounds published below, which come from the tracker's component
        % balls (frame-free speed, speed-rate and normal acceleration; the
        % course adds the ego yaw error) rather than from the inertial box.
        % A nonzero modelJerkMaximum admits motion outside every NRMM path;
        % no motion contract describes it, so none is published and the
        % controller refuses the target at admission.
        output.targetEstimate.predictionMotion = struct( ...
            "kind","nrmm-motion-v1", ...
            "curvatureMaximum",sin(domain.sideslipMaximum)/domain.rearAxleDistance, ...
            "speedRateMaximum",domain.scalarAccelerationMaximum, ...
            "scalarAccelerationMaximum",domain.accelerationNormBound);
        parameters = nrmmTargetParameterErrorBounds(target.targetVelocity, ...
            target.targetAcceleration, components(2), components(3), bound.yaw);
        if ~available
            parameters = struct("speedErrorBound", Inf, "courseErrorBound", Inf, ...
                "speedRateErrorBound", Inf, "curvatureInterval", [-Inf; Inf]);
        end
        output.targetEstimate.targetSpeedErrorBound = parameters.speedErrorBound;
        output.targetEstimate.targetCourseErrorBound = parameters.courseErrorBound;
        output.targetEstimate.targetSpeedRateErrorBound = parameters.speedRateErrorBound;
        output.targetEstimate.targetCurvatureInterval = parameters.curvatureInterval;
    end
end

function radius = localVelocityBounds(output, bound, input, design, age)
    radius = repmat(bound.bodyVelocity, 2, 1);
    if ~bound.orientationSet.valid || ~isfield(output, "egoBodyVelocity") ...
            || ~isfield(input, "gnssVelocity")
        return;
    end
    noise = design.sensors.velocityNoiseMaximum;
    if age > 0
        if ~isfield(bound.holdBounds, "acceleration") ...
                || ~isfinite(bound.holdBounds.acceleration)
            return;
        end
        noise = noise+bound.holdBounds.acceleration*age;
    end
    % Rotate the timestamped GNSS velocity ball over the certified yaw set.
    % Keep the observer point unchanged. Taking extrema before boxing retains
    % the small longitudinal error near straight travel; a norm radius copied
    % to both body components discards this directional information.
    velocity = input.gnssVelocity(:);
    coefficients = [velocity(1), velocity(2); velocity(2), -velocity(1)];
    intervals = bound.orientationSet.intervals;
    for component = 1:2
        cosine = coefficients(component, 1);
        sine = coefficients(component, 2);
        phase = atan2(sine, cosine);
        stationary = phase+(-2:3)*pi;
        inside = any(stationary >= intervals(:, 1) & stationary <= intervals(:, 2), 1);
        angles = [intervals(:); stationary(inside).'];
        values = cosine*cos(angles)+sine*sin(angles);
        padding = noise+64*eps(max(1, norm(velocity)));
        lower = min(values)-padding;
        upper = max(values)+padding;
        radius(component) = min(radius(component), ...
            max(abs([lower, upper]-output.egoBodyVelocity(component))));
    end
end

function certificate = localCertificate(kind, time, values, available, scope)
    certificate = struct("kind", kind, "time", time, "bounds", values, ...
        "available", available, "source", "nrmm-state-time-enclosure", ...
        "scope", scope, "futurePredictionIncluded", false, ...
        "floatingPointVerified", false);
end
