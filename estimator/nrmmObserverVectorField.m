function [derivative, model] = nrmmObserverVectorField( ...
        estimate, measurement, design)
% nrmmObserverVectorField Evaluate the continuous observer equations.
%
% The eight-state core for one target consists of body velocity and [rho;q;s].
% Absolute yaw is used only by the output reconstruction, never by this field.
% The independent inertial position output uses the GNSS velocity directly.
% Synchronization, orientation sets, predictors and integration remain outside.

    bodyVelocity = localVectorField(estimate, "bodyVelocity", 2);
    position = localVectorField(estimate, "position", 2);
    targetState = localMatrixField(estimate, "targetState", 6);
    targetCount = size(targetState, 2);

    yawRate = localScalarField(measurement, "yawRate");
    gnssVelocity = localVectorField(measurement, "gnssVelocity", 2);
    bodyAcceleration = localVectorField( ...
        measurement, "bodyAcceleration", 2);
    positionReference = localVectorField( ...
        measurement, "positionReference", 2);
    radarReference = localMatrixField( ...
        measurement, "radarReference", 2);
    if size(radarReference, 2) ~= targetCount
        error("nrmmObserverVectorField:targetCountMismatch", ...
            "radarReference and targetState must have the same column count.");
    end
    radarAvailable = localAvailability(measurement, targetCount);

    planarCross = [0.0, -1.0; 1.0, 0.0];
    velocityMeasurement = nrmmKinematicVelocityMeasurement( ...
        gnssVelocity, yawRate, design);
    velocityInnovation = velocityMeasurement-bodyVelocity;
    bodyVelocityDerivative = bodyAcceleration ...
        - yawRate*planarCross*bodyVelocity ...
        + design.velocity.gain*velocityInnovation;

    inertialVelocity = gnssVelocity;
    positionDerivative = inertialVelocity ...
        + design.position.gain*(positionReference-position);

    ego = struct("bodyVelocity", bodyVelocity, "yawRate", yawRate);
    innovationGains = design.target.innovationGains;
    targetDerivative = zeros(size(targetState));
    targetPlantDerivative = zeros(size(targetState));
    for targetIdx = 1:targetCount
        plantDerivative = nrmmTargetTrackerDerivative( ...
            targetState(:, targetIdx), ego, design.target.domain);
        observerDerivative = plantDerivative;
        if radarAvailable(targetIdx)
            radarInnovation = radarReference(:, targetIdx) ...
                - targetState(1:2, targetIdx);
            observerDerivative = observerDerivative+[ ...
                innovationGains(1)*radarInnovation; ...
                innovationGains(2)*radarInnovation; ...
                innovationGains(3)*radarInnovation];
        end
        targetPlantDerivative(:, targetIdx) = plantDerivative;
        targetDerivative(:, targetIdx) = observerDerivative;
    end

    derivative = struct( ...
        "bodyVelocity", bodyVelocityDerivative, ...
        "position", positionDerivative, ...
        "targetState", targetDerivative);
    model = struct( ...
        "inertialVelocity", inertialVelocity, ...
        "targetPlantDerivative", targetPlantDerivative);
end

function value = localScalarField(input, fieldName)
    if ~isstruct(input) || ~isfield(input, fieldName)
        error("nrmmObserverVectorField:missingField", ...
            "%s is required.", fieldName);
    end
    value = double(input.(fieldName));
    if ~isscalar(value) || ~isfinite(value)
        error("nrmmObserverVectorField:invalidField", ...
            "%s must be a finite scalar.", fieldName);
    end
end

function value = localVectorField(input, fieldName, rowCount)
    value = localMatrixField(input, fieldName, rowCount);
    if size(value, 2) ~= 1
        error("nrmmObserverVectorField:invalidField", ...
            "%s must be a finite %d-vector.", fieldName, rowCount);
    end
end

function value = localMatrixField(input, fieldName, rowCount)
    if ~isstruct(input) || ~isfield(input, fieldName)
        error("nrmmObserverVectorField:missingField", ...
            "%s is required.", fieldName);
    end
    value = double(input.(fieldName));
    if ~ismatrix(value) || size(value, 1) ~= rowCount ...
            || any(~isfinite(value), "all")
        error("nrmmObserverVectorField:invalidField", ...
            "%s must be a finite %d-by-N matrix.", fieldName, rowCount);
    end
end

function available = localAvailability(measurement, targetCount)
    if ~isfield(measurement, "radarAvailable")
        error("nrmmObserverVectorField:missingField", ...
            "radarAvailable is required.");
    end
    available = logical(measurement.radarAvailable(:));
    if numel(available) ~= targetCount
        error("nrmmObserverVectorField:targetCountMismatch", ...
            "radarAvailable must contain one flag per target.");
    end
end
