function support = targetPredictionFutureSupport( ...
        direction, startTime, prediction)
% targetPredictionFutureSupport Support of the complete target prediction.
%
% support(j) is
%
%   sup direction(:,j)' * targetPosition(t),  t >= startTime,
%
% for the controller's declared constant-curvature,
% constant-tangential-acceleration target prediction. The computation is
% analytic over the complete continuation: it includes the remaining
% finite arc when the prediction stops, the complete future orbit when a
% curved prediction continues indefinitely, and the complete ray when a
% straight prediction continues indefinitely. These are evaluation cases
% of one prediction law, not admissible target-motion classes.
%
% A future target predictor may replace this module provided it returns
% the same exact support operation for its own complete predicted
% continuation. The terminal controller consumes only this interface.

    localValidateInput(direction, startTime, prediction);
    direction = double(direction);
    startTime = double(startTime);
    position = double(prediction.initialPosition(:));
    courseDirection = double(prediction.initialCourseDirection(:));
    speed = double(prediction.initialSpeed);
    acceleration = double(prediction.tangentialAcceleration);
    curvature = double(prediction.curvature);
    stopTime = double(prediction.stopTime);

    propagationStart = min(startTime, stopTime);
    arcStart = speed*propagationStart ...
        + 0.5*acceleration*propagationStart^2;
    if isfinite(stopTime)
        arcEnd = speed*stopTime+0.5*acceleration*stopTime^2;
        arcEnd = max(arcEnd, arcStart);
    elseif speed > 0.0 || acceleration > 0.0
        arcEnd = inf;
    else
        arcEnd = arcStart;
    end

    if curvature == 0.0
        support = localStraightSupport( ...
            direction, position, courseDirection, arcStart, arcEnd);
        return;
    end
    support = localCurvedSupport(direction, position, courseDirection, ...
        curvature, arcStart, arcEnd);
end

function support = localStraightSupport( ...
        direction, position, courseDirection, arcStart, arcEnd)
    positionProjection = direction.'*position;
    courseProjection = direction.'*courseDirection;
    support = positionProjection+courseProjection*arcStart;
    if isfinite(arcEnd)
        endProjection = positionProjection+courseProjection*arcEnd;
        support = max(support, endProjection);
    else
        support(courseProjection > 0.0) = inf;
    end
    support = support.';
end

function support = localCurvedSupport( ...
        direction, position, courseDirection, curvature, ...
        arcStart, arcEnd)
    courseNormal = [-courseDirection(2); courseDirection(1)];
    circleCentre = position+courseNormal/curvature;
    radius = 1.0/abs(curvature);
    centreProjection = direction.'*circleCentre;
    if ~isfinite(arcEnd) ...
            || abs(curvature*(arcEnd-arcStart)) >= 2.0*pi
        support = (centreProjection+radius).';
        return;
    end

    thetaStart = curvature*arcStart;
    thetaEnd = curvature*arcEnd;
    sineCoefficient = direction.'*courseDirection/curvature;
    cosineCoefficient = -direction.'*courseNormal/curvature;
    startValue = sineCoefficient*sin(thetaStart) ...
        + cosineCoefficient*cos(thetaStart);
    endValue = sineCoefficient*sin(thetaEnd) ...
        + cosineCoefficient*cos(thetaEnd);
    oscillatorySupport = max(startValue, endValue);

    angleTravel = thetaEnd-thetaStart;
    angleSpan = abs(angleTravel);
    if angleSpan > 0.0
        travelSign = sign(angleTravel);
        maximumPhase = atan2(sineCoefficient, cosineCoefficient);
        relativePhase = mod( ...
            travelSign*(maximumPhase-thetaStart), 2.0*pi);
        angleTolerance = 64.0*eps(1.0+max( ...
            [abs(thetaStart), abs(thetaEnd), angleSpan]));
        containsMaximum = relativePhase <= angleSpan+angleTolerance ...
            | relativePhase >= 2.0*pi-angleTolerance;
        oscillatorySupport(containsMaximum) = radius;
    end
    support = (centreProjection+oscillatorySupport).';
end

function localValidateInput(direction, startTime, prediction)
    if ~isnumeric(direction) || ~isreal(direction) ...
            || size(direction, 1) ~= 2 || isempty(direction) ...
            || any(~isfinite(direction), "all")
        error("collisionAvoidanceController:invalidTargetPrediction", ...
            "direction must be a finite real 2-by-M matrix.");
    end
    directionNorm = vecnorm(double(direction), 2, 1);
    if any(abs(directionNorm-1.0) > 1.0e-10)
        error("collisionAvoidanceController:invalidTargetPrediction", ...
            "Every target-continuation support direction must be unit.");
    end
    if ~isnumeric(startTime) || ~isreal(startTime) ...
            || ~isscalar(startTime) || ~isfinite(startTime) ...
            || startTime < 0.0
        error("collisionAvoidanceController:invalidTargetPrediction", ...
            "startTime must be a nonnegative finite scalar.");
    end
    required = ["initialPosition", "initialCourseDirection", ...
        "initialSpeed", "tangentialAcceleration", "curvature", ...
        "stopTime"];
    if ~isstruct(prediction) || ~isscalar(prediction) ...
            || ~all(isfield(prediction, required))
        error("collisionAvoidanceController:invalidTargetPrediction", ...
            "prediction must contain the complete target motion law.");
    end
    localFiniteVector(prediction.initialPosition, "initialPosition");
    localFiniteVector( ...
        prediction.initialCourseDirection, "initialCourseDirection");
    courseNorm = norm(double(prediction.initialCourseDirection(:)));
    if abs(courseNorm-1.0) > 1.0e-10
        error("collisionAvoidanceController:invalidTargetPrediction", ...
            "initialCourseDirection must be unit.");
    end
    localFiniteScalar(prediction.initialSpeed, "initialSpeed");
    if prediction.initialSpeed < 0.0
        error("collisionAvoidanceController:invalidTargetPrediction", ...
            "initialSpeed must be nonnegative.");
    end
    localFiniteScalar( ...
        prediction.tangentialAcceleration, "tangentialAcceleration");
    localFiniteScalar(prediction.curvature, "curvature");
    stopTime = prediction.stopTime;
    if ~isnumeric(stopTime) || ~isreal(stopTime) ...
            || ~isscalar(stopTime) || isnan(stopTime) || stopTime < 0.0
        error("collisionAvoidanceController:invalidTargetPrediction", ...
            "stopTime must be a nonnegative scalar or Inf.");
    end
end

function localFiniteVector(value, name)
    if ~isnumeric(value) || ~isreal(value) || numel(value) ~= 2 ...
            || any(~isfinite(value), "all")
        error("collisionAvoidanceController:invalidTargetPrediction", ...
            "%s must be a finite real 2-vector.", name);
    end
end

function localFiniteScalar(value, name)
    if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
            || ~isfinite(value)
        error("collisionAvoidanceController:invalidTargetPrediction", ...
            "%s must be a finite real scalar.", name);
    end
end
