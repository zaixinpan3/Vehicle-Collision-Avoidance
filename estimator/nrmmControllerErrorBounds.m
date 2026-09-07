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
    egoBounds = [egoPosition; egoPosition; bound.yaw; ...
        bound.bodyVelocity; bound.bodyVelocity; egoYawRate];
    egoAvailable = bound.egoValid && all(isfinite(egoBounds));
    if ~egoAvailable
        egoBounds(:) = inf;
    end
    output.egoPositionErrorBound = egoBounds(1);
    output.egoYawErrorBound = egoBounds(3);
    output.egoBodyVelocityErrorBound = egoBounds(4);
    output.egoYawRateErrorBound = egoBounds(6);
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
        available = egoAvailable && bound.valid(index) && all(isfinite(values));
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
    end
end

function certificate = localCertificate(kind, time, values, available, scope)
    certificate = struct("kind", kind, "time", time, "bounds", values, ...
        "available", available, "source", "nrmm-state-time-enclosure", ...
        "scope", scope, "futurePredictionIncluded", false, ...
        "floatingPointVerified", false);
end
