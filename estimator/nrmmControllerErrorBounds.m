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

    domain = design.target.domain;
    rotationError = 2*sin(min(bound.yaw, pi)/2);
    for index = 1:numel(output.targetEstimates)
        target = output.targetEstimates(index);
        components = bound.targetComponents(:, index);
        range = min(bound.trueRangeMaximum(index), norm(target.relativePosition));
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
            history = nrmmTargetHistory("enclose",bound.targetHistory{index},output.stateTime);
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
                output.targetEstimates(index).measurementHistoryEnclosure = history;
            end
        end
        available = egoAvailable && bound.valid(index) && all(isfinite(values));
        available = available && historyAvailable;
        if ~available
            values(:) = inf;
        end
        output.targetEstimates(index).controllerErrorBound = localCertificate( ...
            "target-state-v1", output.stateTime, values, available, bound.scope);
        output.targetEstimates(index).controllerErrorBoundSource = "nrmm-state-time-enclosure";
        output.targetEstimates(index).targetPositionInertialErrorBound = values(1:2);
        output.targetEstimates(index).targetVelocityInertialErrorBound = values(3:4);
        output.targetEstimates(index).targetAccelerationInertialErrorBound = values(5:6);
        output.targetEstimates(index).targetYawErrorBound = values(7);
        output.targetEstimates(index).targetYawRateErrorBound = values(8);
        jerkMaximum = hypot(domain.speedMaximum*domain.yawRateMaximum^2, ...
            3*domain.scalarAccelerationMaximum*domain.yawRateMaximum)+design.target.modelJerkMaximum;
        output.targetEstimates(index).predictionMotion = struct( ...
            "kind","finite-sensing-motion-v1","jerkBound",repmat(jerkMaximum,2,1), ...
            "scalarAccelerationMaximum",domain.scalarAccelerationMaximum, ...
            "yawAccelerationBound",domain.scalarAccelerationMaximum*domain.yawRateMaximum/domain.speedMinimum ...
                +design.target.modelJerkMaximum/domain.speedMinimum);
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
