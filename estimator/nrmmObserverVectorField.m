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
    targetState = localVectorField(estimate, "targetState", 6);

    yawRate = localScalarField(measurement, "yawRate");
    gnssVelocity = localVectorField(measurement, "gnssVelocity", 2);
    bodyAcceleration = localVectorField( ...
        measurement, "bodyAcceleration", 2);
    positionReference = localVectorField( ...
        measurement, "positionReference", 2);
    radarReference = localVectorField( ...
        measurement, "radarReference", 2);
    radarAvailable = localAvailability(measurement);

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
    targetPlantDerivative = nrmmTargetTrackerDerivative(targetState,ego,design.target.domain);
    targetDerivative = targetPlantDerivative;
    if radarAvailable
        radarInnovation = radarReference-targetState(1:2);
        targetDerivative = targetDerivative+[innovationGains(1)*radarInnovation; ...
            innovationGains(2)*radarInnovation;innovationGains(3)*radarInnovation];
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
    if ~isstruct(input) || ~isfield(input,fieldName)
        error("nrmmObserverVectorField:missingField","%s is required.",fieldName);
    end
    value=double(input.(fieldName));
    if ~isvector(value) || numel(value)~=rowCount || any(~isfinite(value))
        error("nrmmObserverVectorField:invalidField","%s must be a finite %d-vector.",fieldName,rowCount);
    end
    value=value(:);
end

function available = localAvailability(measurement)
    if ~isfield(measurement,"radarAvailable")
        error("nrmmObserverVectorField:missingField","radarAvailable is required.");
    end
    validateattributes(measurement.radarAvailable,{'logical'},{'scalar'});
    available=measurement.radarAvailable;
end
