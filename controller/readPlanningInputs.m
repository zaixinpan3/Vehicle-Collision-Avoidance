function [ego, lane, road, targets] = readPlanningInputs( ...
        egoState, targetEstimate, laneCenterline, cfg)
% readPlanningInputs Parse and validate the controller's three raw inputs.
%
% Converts the ego state (explicit controller-state fields or the
% cascaded estimator's egoState vector form), the lane centerline or
% scalar road-geometry structure (finite local-quadratic boundaries
% and route selection), and the target estimate record (at most one
% target) into the canonical planning structures the controller
% consumes. All validation of the
% public input contract lives here; downstream modules assume these
% structures are well formed. An empty targetEstimate falls back to a
% targetEstimates field bundled on the ego input, and an empty
% laneCenterline falls back to a long straight line through the current
% ego pose.

    if isempty(targetEstimate)
        targetEstimate = localBundledTargetEstimate(egoState);
    end
    ego = localReadEgoState(egoState, cfg);
    [lane, road] = localReadLane(laneCenterline, ego, cfg);
    targets = localReadTargets(targetEstimate, ego, cfg);
end

function targetEstimate = localBundledTargetEstimate(egoInput)
    targetEstimate = struct();
    if isstruct(egoInput) && isscalar(egoInput) ...
            && isfield(egoInput, "targetEstimates")
        targetEstimate = egoInput.targetEstimates;
    end
end

function ego = localReadEgoState(data, cfg)
    if ~isstruct(data) || ~isscalar(data)
        error("collisionAvoidanceController:invalidInput", ...
            "egoState must be a scalar structure.");
    end
    if isfield(data, "egoState")
        ego = localReadNrmmEgoState(data, cfg);
        return;
    end

    if isfield(data, "position") && isnumeric(data.position) ...
            && numel(data.position) == 2
        position = localFiniteVector(data.position, 2, "egoState.position");
    else
        position = [localRequiredStateScalar(data, ...
            ["positionX", "x"], "ego position x"); ...
            localRequiredStateScalar(data, ...
            ["positionY", "y"], "ego position y")];
    end
    yaw = localRequiredStateScalar(data, ...
        ["yawAngle", "yaw", "heading"], "ego yaw angle");
    longitudinalVelocity = localRequiredStateScalar(data, ...
        ["longitudinalVelocity", "speed"], ...
        "ego longitudinal velocity");
    lateralVelocity = localOptionalStateScalar( ...
        data, "lateralVelocity", 0.0);
    yawRate = localOptionalStateScalar(data, "yawRate", 0.0);
    speedRoundoff = 100.0*eps(max([ ...
        1.0, abs(longitudinalVelocity), ...
        abs(cfg.model.speedMinimum), abs(cfg.model.speedMaximum)]));
    if longitudinalVelocity < cfg.model.speedMinimum-speedRoundoff ...
            || longitudinalVelocity ...
                > cfg.model.speedMaximum+speedRoundoff
        error("collisionAvoidanceController:invalidInput", ...
            "The measured longitudinal velocity lies outside the model domain.");
    end
    rotation = localRotationMatrix(yaw);
    ego = struct();
    ego.position = position;
    ego.yaw = yaw;
    ego.inertialVelocity = rotation ...
        * [longitudinalVelocity; lateralVelocity];
    ego.modelState = [position; yaw; longitudinalVelocity; ...
        lateralVelocity; yawRate];
    ego.stateErrorBound = localOptionalNonnegativeInputVector( ...
        data, "controllerStateErrorBound", 6);
    ego.longitudinalAccelerationBias = localOptionalStateScalar( ...
        data, "longitudinalAccelerationBias", 0.0);
    ego.heldActuatorInput = localOptionalHeldActuatorInput(data);
end

function ego = localReadNrmmEgoState(data, cfg)
    % The cascaded estimator publishes egoState as the six-element vector
    % [x; vx; ax; y; vy; ay] in inertial coordinates; no jerk state exists.
    estimate = localFiniteVector( ...
        data.egoState, 6, "egoState.egoState");
    yaw = localRequiredStateScalar(data, ...
        "egoYaw", "estimated ego yaw angle");
    yawRate = localRequiredStateScalar(data, ...
        ["egoYawRate", "egoYawRateMeasured"], ...
        "state-time-aligned estimated ego yaw rate");
    position = estimate([1, 4]);
    inertialVelocity = estimate([2, 5]);
    bodyVelocity = localRotationMatrix(yaw).' * inertialVelocity;
    velocityRoundoff = 100.0 * eps(max( ...
        1.0, norm(inertialVelocity, inf)));
    bodyVelocity(abs(bodyVelocity) <= velocityRoundoff) = 0.0;
    longitudinalVelocity = bodyVelocity(1);
    speedRoundoff = 100.0*eps(max([ ...
        1.0, abs(longitudinalVelocity), ...
        abs(cfg.model.speedMinimum), abs(cfg.model.speedMaximum)]));
    if longitudinalVelocity < cfg.model.speedMinimum-speedRoundoff ...
            || longitudinalVelocity ...
                > cfg.model.speedMaximum+speedRoundoff
        error("collisionAvoidanceController:invalidInput", ...
            "The estimated longitudinal velocity lies outside " ...
            + "the model domain.");
    end

    ego = struct();
    ego.position = position;
    ego.yaw = yaw;
    ego.inertialVelocity = inertialVelocity;
    ego.modelState = [position; yaw; bodyVelocity; yawRate];
    ego.stateErrorBound = localOptionalNonnegativeInputVector( ...
        data, "controllerStateErrorBound", 6);
    ego.longitudinalAccelerationBias = localOptionalStateScalar( ...
        data, "longitudinalAccelerationBias", 0.0);
    ego.heldActuatorInput = localOptionalHeldActuatorInput(data);
end

function value = localRequiredStateScalar(data, aliases, description)
    for alias = aliases
        if isfield(data, alias)
            value = data.(alias);
            if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
                    || ~isfinite(value)
                error("collisionAvoidanceController:invalidInput", ...
                    "%s must be a finite real scalar.", description);
            end
            value = double(value);
            return;
        end
    end
    error("collisionAvoidanceController:invalidInput", ...
        "egoState is missing %s.", description);
end

function held = localOptionalHeldActuatorInput(data)
% The actuator position the vehicle is currently holding, [deltaF; a].
%
% This is ESTIMATOR data - a steering-angle sensor reading and the
% actuator's share of the measured longitudinal acceleration - not a
% memory of what this controller commanded last sample. Read as a
% measurement it introduces no cross-sample controller state, and it
% stays correct when a command is not achieved, when another
% controller shares the loop, and across an episode restart.
%
% Absent, the anchor row is not built and the program is exactly the
% one that treats the first input as free.

    held = zeros(0, 1);
    if isfield(data, "heldActuatorInput") ...
            && ~isempty(data.heldActuatorInput)
        held = localFiniteVector(data.heldActuatorInput, 2, ...
            "egoState.heldActuatorInput");
        held = held(:);
    end
end

function value = localOptionalStateScalar(data, aliases, defaultValue)
    value = defaultValue;
    for alias = aliases
        if isfield(data, alias) && ~isempty(data.(alias))
            rawValue = data.(alias);
            if ~isnumeric(rawValue) || ~isreal(rawValue) ...
                    || ~isscalar(rawValue) || ~isfinite(rawValue)
                error("collisionAvoidanceController:invalidInput", ...
                    "egoState.%s must be a finite real scalar.", alias);
            end
            value = double(rawValue);
            return;
        end
    end
end

function value = localFiniteVector(value, count, description)
    if ~isnumeric(value) || ~isreal(value) || numel(value) ~= count ...
            || any(~isfinite(value), "all")
        error("collisionAvoidanceController:invalidInput", ...
            "%s must contain %d finite real values.", description, count);
    end
    value = double(value(:));
end

function value = localOptionalNonnegativeInputVector( ...
        data, fieldName, count)
    value = zeros(count, 1);
    if ~isfield(data, fieldName) || isempty(data.(fieldName))
        return;
    end
    rawValue = data.(fieldName);
    if ~isnumeric(rawValue) || ~isreal(rawValue) ...
            || numel(rawValue) ~= count ...
            || any(~isfinite(rawValue), "all") ...
            || any(rawValue < 0.0, "all")
        error("collisionAvoidanceController:invalidInput", ...
            "%s must contain %d nonnegative finite values.", ...
            fieldName, count);
    end
    value = double(rawValue(:));
end

function value = localOptionalNonnegativeInputScalar(data, fieldName)
    value = 0.0;
    if ~isfield(data, fieldName) || isempty(data.(fieldName))
        return;
    end
    rawValue = data.(fieldName);
    if ~isnumeric(rawValue) || ~isreal(rawValue) ...
            || ~isscalar(rawValue) || ~isfinite(rawValue) ...
            || rawValue < 0.0
        error("collisionAvoidanceController:invalidInput", ...
            "%s must be a nonnegative finite scalar.", fieldName);
    end
    value = double(rawValue);
end

function [lane, road] = localReadLane(rawGeometry, ego, cfg)
    rawLane = rawGeometry;
    if isstruct(rawGeometry)
        if ~isscalar(rawGeometry)
            error("collisionAvoidanceController:invalidInput", ...
                "Road geometry must be a scalar structure.");
        end
        if isfield(rawGeometry, "centerline")
            rawLane = rawGeometry.centerline;
        elseif isfield(rawGeometry, "points")
            rawLane = rawGeometry.points;
        else
            rawLane = [];
        end
    end
    lane = localReadCenterline(rawLane, ego);
    road = localReadRoadGeometry(rawGeometry, lane, cfg);
end

function lane = localReadCenterline(rawLane, ego)
    persistent cachedRawLane cachedLane
    if isempty(rawLane)
        tangent = [cos(ego.yaw), sin(ego.yaw)];
        rawLane = ego.position.' + [-1000.0; 1000.0] .* tangent;
    elseif ~isempty(cachedLane) && isequaln(rawLane, cachedRawLane)
        lane = cachedLane;
        return;
    end
    if ~isnumeric(rawLane) || ~isreal(rawLane) ...
            || size(rawLane, 2) ~= 2 || size(rawLane, 1) < 2 ...
            || any(~isfinite(rawLane), "all")
        error("collisionAvoidanceController:invalidInput", ...
            "laneCenterline must be a finite real N-by-2 polyline.");
    end
    points = double(rawLane);
    segment = diff(points, 1, 1);
    segmentLength = vecnorm(segment, 2, 2);
    active = segmentLength > 100.0 * eps(max(1.0, max(abs(points), [], "all")));
    if ~any(active)
        error("collisionAvoidanceController:invalidInput", ...
            "laneCenterline must contain at least one nonzero segment.");
    end
    lane = struct();
    lane.segmentStart = points(1:end - 1, :);
    lane.segmentStart = lane.segmentStart(active, :);
    lane.segment = segment(active, :);
    lane.segmentLength = segmentLength(active);
    lane.tangent = lane.segment ./ lane.segmentLength;
    lane.segmentCurvature = localPolylineSegmentCurvature( ...
        lane.tangent, lane.segmentLength);
    % Cumulative station of every segment start: the path coordinate
    % s of the model and the barrier rows.
    lane.segmentStation = [0.0; cumsum(lane.segmentLength(1:end-1))];
    cachedRawLane = rawLane;
    cachedLane = lane;
end

function road = localReadRoadGeometry(rawGeometry, lane, cfg)
    road = struct();
    road.boundaries = repmat(localEmptyRoadBoundary(), 0, 1);
    road.routeBranchId = "";
    if ~isstruct(rawGeometry)
        return;
    end

    boundaries = repmat(localEmptyRoadBoundary(), 0, 1);
    if isfield(rawGeometry, "boundaries") ...
            && ~isempty(rawGeometry.boundaries)
        rawBoundaries = rawGeometry.boundaries;
        if ~isstruct(rawBoundaries)
            error("collisionAvoidanceController:invalidInput", ...
                "roadGeometry.boundaries must be a structure array.");
        end
        rawBoundaries = rawBoundaries(:);
        boundaryCount = numel(rawBoundaries);
        boundaries = repmat( ...
            localEmptyRoadBoundary(), boundaryCount, 1);
        for boundaryIdx = 1:boundaryCount
            boundaries(boundaryIdx) = localReadRoadBoundary( ...
                rawBoundaries(boundaryIdx), boundaryIdx, cfg);
        end
    end

    if isempty(boundaries)
        boundaryRouteIds = strings(1, 0);
    else
        boundaryRouteIds = [boundaries.routeBranchId];
    end
    explicitRoute = isfield(rawGeometry, "routeBranchId") ...
        && ~isempty(rawGeometry.routeBranchId);
    if explicitRoute
        routeBranchId = localRoadTextScalar( ...
            rawGeometry.routeBranchId, ...
            "roadGeometry.routeBranchId", false);
    else
        nonemptyRouteIds = unique(boundaryRouteIds, "stable");
        nonemptyRouteIds(nonemptyRouteIds == "") = [];
        if numel(nonemptyRouteIds) > 1
            error("collisionAvoidanceController:ambiguousRoadRoute", ...
                "Several road routes are present. Supply one " ...
                + "roadGeometry.routeBranchId.");
        elseif isscalar(nonemptyRouteIds)
            routeBranchId = nonemptyRouteIds;
        else
            routeBranchId = "";
        end
    end
    nonemptyRouteIds = boundaryRouteIds(boundaryRouteIds ~= "");
    if routeBranchId ~= "" && ~isempty(nonemptyRouteIds) ...
            && ~any(nonemptyRouteIds == routeBranchId)
        error("collisionAvoidanceController:unknownRoadRoute", ...
            "roadGeometry.routeBranchId does not identify any supplied " ...
            + "road component.");
    end

    activeBoundary = boundaryRouteIds == "" ...
        | boundaryRouteIds == routeBranchId;
    boundaries = boundaries(activeBoundary);
    for boundaryIdx = 1:numel(boundaries)
        localValidateRoadBoundarySafeSide( ...
            boundaries(boundaryIdx), lane);
    end

    road.boundaries = boundaries;
    road.routeBranchId = routeBranchId;
end

function boundary = localEmptyRoadBoundary()
    boundary = struct( ...
        "origin", zeros(2, 1), ...
        "longitudinalDirection", [1.0; 0.0], ...
        "lateralDirection", [0.0; 1.0], ...
        "coefficients", zeros(1, 3), ...
        "parameterRange", zeros(1, 2), ...
        "safeSideSign", 1.0, ...
        "normalDistanceErrorBound", 0.0, ...
        "routeBranchId", "", ...
        "boundaryId", "", ...
        "coveragePolicy", "strict");
end

function boundary = localReadRoadBoundary(data, boundaryIdx, cfg)
    requiredFields = [ ...
        "origin", "longitudinalDirection", "lateralDirection", ...
        "coefficients", "parameterRange", "safeSideSign"];
    missing = requiredFields(~isfield(data, requiredFields));
    if ~isempty(missing)
        error("collisionAvoidanceController:invalidRoadBoundary", ...
            "Road boundary %d is missing %s.", ...
            boundaryIdx, strjoin(missing, ", "));
    end
    if isfield(data, "normalErrorBound")
        error("collisionAvoidanceController:invalidRoadBoundary", ...
            "Road boundary %d uses the ambiguous normalErrorBound field. " ...
            + "Use normalDistanceErrorBound in metres.", boundaryIdx);
    end

    boundary = localEmptyRoadBoundary();
    boundary.origin = localFiniteVector( ...
        data.origin, 2, "road boundary origin");
    boundary.longitudinalDirection = localFiniteVector( ...
        data.longitudinalDirection, 2, ...
        "road boundary longitudinalDirection");
    boundary.lateralDirection = localFiniteVector( ...
        data.lateralDirection, 2, ...
        "road boundary lateralDirection");
    frame = [boundary.longitudinalDirection, ...
        boundary.lateralDirection];
    frameError = norm(frame.' * frame - eye(2), inf);
    frameTolerance = cfg.road.orthonormalTolerance;
    if frameError > frameTolerance ...
            || det(frame) <= 0.0 ...
            || abs(det(frame) - 1.0) > 2.0 * frameTolerance
        error("collisionAvoidanceController:invalidRoadBoundary", ...
            "Road boundary %d directions must form a right-handed " ...
            + "orthonormal frame.", boundaryIdx);
    end
    boundary.longitudinalDirection = ...
        boundary.longitudinalDirection ...
        / norm(boundary.longitudinalDirection);
    boundary.lateralDirection = [ ...
        -boundary.longitudinalDirection(2); ...
        boundary.longitudinalDirection(1)];

    coefficients = localFiniteVector( ...
        data.coefficients, 3, "road boundary coefficients");
    parameterRange = localFiniteVector( ...
        data.parameterRange, 2, "road boundary parameterRange");
    if parameterRange(1) >= parameterRange(2)
        error("collisionAvoidanceController:invalidRoadBoundary", ...
            "Road boundary %d parameterRange must increase strictly.", ...
            boundaryIdx);
    end
    safeSideSign = data.safeSideSign;
    if ~isnumeric(safeSideSign) || ~isreal(safeSideSign) ...
            || ~isscalar(safeSideSign) ...
            || ~any(double(safeSideSign) == [-1.0, 1.0])
        error("collisionAvoidanceController:invalidRoadBoundary", ...
            "Road boundary %d safeSideSign must be +1 or -1.", ...
            boundaryIdx);
    end

    boundary.coefficients = coefficients.';
    boundary.parameterRange = parameterRange.';
    boundary.safeSideSign = double(safeSideSign);
    boundary.normalDistanceErrorBound = ...
        localOptionalNonnegativeInputScalar( ...
            data, "normalDistanceErrorBound");
    if isfield(data, "routeBranchId") ...
            && ~isempty(data.routeBranchId)
        boundary.routeBranchId = localRoadTextScalar( ...
            data.routeBranchId, ...
            "road boundary routeBranchId", true);
    end
    if isfield(data, "boundaryId") && ~isempty(data.boundaryId)
        boundary.boundaryId = localRoadTextScalar( ...
            data.boundaryId, "road boundary boundaryId", false);
    else
        boundary.boundaryId = "boundary-" + string(boundaryIdx);
    end
    if isfield(data, "coveragePolicy") ...
            && ~isempty(data.coveragePolicy)
        boundary.coveragePolicy = localRoadTextScalar( ...
            data.coveragePolicy, ...
            "road boundary coveragePolicy", false);
    end
    if ~any(boundary.coveragePolicy == [ ...
            "strict", "perceptionLimited", ...
            "knownNominalPathOffset"])
        error("collisionAvoidanceController:invalidRoadBoundary", ...
            "Road boundary %d coveragePolicy must be strict, " ...
            + "perceptionLimited, or knownNominalPathOffset.", ...
            boundaryIdx);
    end
end

function value = localRoadTextScalar(rawValue, fieldName, allowEmpty)
    if ~(ischar(rawValue) || (isstring(rawValue) && isscalar(rawValue)))
        error("collisionAvoidanceController:invalidRoadBoundary", ...
            "%s must be a text scalar.", fieldName);
    end
    value = string(rawValue);
    if ismissing(value) || (~allowEmpty && strlength(value) == 0)
        error("collisionAvoidanceController:invalidRoadBoundary", ...
            "%s must not be empty or missing.", fieldName);
    end
end

function localValidateRoadBoundarySafeSide(boundary, lane)
    sReference = mean(boundary.parameterRange);
    coefficients = boundary.coefficients;
    curvePoint = boundary.origin ...
        + boundary.longitudinalDirection * sReference ...
        + boundary.lateralDirection ...
            * polyval(coefficients, sReference);
    projection = laneProjection(curvePoint, lane);
    centerlineOffset = projection.point - boundary.origin;
    centerlineS = boundary.longitudinalDirection.' * centerlineOffset;
    centerlineL = boundary.lateralDirection.' * centerlineOffset;
    parameterTolerance = 100.0 * eps(max( ...
        [1.0, abs(centerlineS), abs(boundary.parameterRange)]));
    if centerlineS < boundary.parameterRange(1) - parameterTolerance ...
            || centerlineS ...
                > boundary.parameterRange(2) + parameterTolerance
        error("collisionAvoidanceController:invalidRoadBoundary", ...
            "Road boundary %s has no selected-centerline reference " ...
            + "inside its finite parameter range.", ...
            boundary.boundaryId);
    end
    safeSideValue = boundary.safeSideSign * ( ...
        centerlineL - polyval(coefficients, centerlineS));
    validationTolerance = 100.0 * eps(max( ...
        [1.0, abs(centerlineL), ...
        abs(polyval(coefficients, centerlineS))]));
    if safeSideValue <= validationTolerance
        error("collisionAvoidanceController:invalidRoadBoundary", ...
            "Road boundary %s safeSideSign does not place the selected " ...
            + "route centerline strictly on the safe side.", ...
            boundary.boundaryId);
    end
end

function curvature = localPolylineSegmentCurvature( ...
        tangent, segmentLength)
    segmentCount = size(tangent, 1);
    curvature = zeros(segmentCount, 1);
    if segmentCount == 1
        return;
    end

    heading = unwrap(atan2(tangent(:, 2), tangent(:, 1)));
    centerSpacing = 0.5 * ( ...
        segmentLength(1:(end - 1)) + segmentLength(2:end));
    interfaceCurvature = diff(heading) ./ centerSpacing;
    curvature(1) = interfaceCurvature(1);
    curvature(end) = interfaceCurvature(end);
    if segmentCount > 2
        curvature(2:(end - 1)) = 0.5 * ( ...
            interfaceCurvature(1:(end - 1)) ...
            + interfaceCurvature(2:end));
    end
end

function targets = localReadTargets(rawTargets, ego, cfg)
    if ~isstruct(rawTargets)
        error("collisionAvoidanceController:invalidInput", ...
            "targetEstimate must be a structure.");
    end
    targets = repmat(localEmptyTarget(), 0, 1);
    record = localTargetRecord(rawTargets);
    if isempty(record)
        return;
    end
    [target, active] = localReadTargetRecord(record, ego, cfg);
    if active
        target.key = localTargetRecordKey(record, 1);
        targets(end + 1, 1) = target;
    end
end

function record = localTargetRecord(rawTargets)
% AT MOST ONE TARGET. This controller does not solve the multi-target
% problem, so neither a structure array nor a vectorized record is
% admitted: more than one target is a declared input error, never a
% silently truncated list.

    record = [];
    if isempty(rawTargets)
        return;
    end
    if ~isscalar(rawTargets)
        error("collisionAvoidanceController:invalidInput", ...
            "targetEstimate must describe at most one target; this " ...
            + "controller does not solve the multi-target problem.");
    end
    if isempty(fieldnames(rawTargets))
        return;
    end
    localRejectVectorizedTarget(rawTargets);
    record = rawTargets;
end

function localRejectVectorizedTarget(rawRecord)
% Reject a record whose fields carry more than one target.

    for fieldName = localTargetVectorFields()
        if ~isfield(rawRecord, fieldName) ...
                || isempty(rawRecord.(fieldName))
            continue;
        end
        value = rawRecord.(fieldName);
        if isnumeric(value) && ismatrix(value) ...
                && size(value, 1) == 2 && size(value, 2) > 1
            error("collisionAvoidanceController:invalidInput", ...
                "%s must describe one target: a 2-element vector.", ...
                fieldName);
        end
    end
    for fieldName = localTargetScalarFields()
        if ~isfield(rawRecord, fieldName) ...
                || isempty(rawRecord.(fieldName))
            continue;
        end
        value = rawRecord.(fieldName);
        if (isnumeric(value) || islogical(value) || isstring(value)) ...
                && isvector(value) && numel(value) > 1
            error("collisionAvoidanceController:invalidInput", ...
                "%s must describe one target: a scalar.", fieldName);
        end
    end
end

function scalarFields = localTargetScalarFields()
    scalarFields = [ ...
        "relativePositionX", "relativePositionY", ...
        "targetVelocityX", "targetVelocityY", ...
        "relativeVelocityX", "relativeVelocityY", ...
        "targetAccelerationX", "targetAccelerationY", ...
        "targetLength", "targetWidth", ...
        "targetYawInertial", "targetYawRelative", ...
        "targetYawAngle", "targetYaw", "yawAngle", "yaw", ...
        "heading", "relativeYaw", "relativeHeading", ...
        "targetYawRate", "yawRate", "courseRate", ...
        "targetYawErrorBound", "targetYawRateErrorBound", ...
        "targetPredictionYawAccelerationErrorBound", ...
        "trackId", "targetId", "objectId", "id", ...
        "relativePositionFrame", "targetVelocityFrame", ...
        "relativeVelocityFrame", "targetAccelerationFrame", ...
        "targetYawFrame"];
end

function vectorFields = localTargetVectorFields()
    vectorFields = [ ...
        "targetPositionInertial", "targetVelocityInertial", ...
        "targetAccelerationInertial", "relativePosition", ...
        "relativeVelocity", "relativeAcceleration", ...
        "targetVelocity", "targetAcceleration", ...
        "targetPositionInertialErrorBound", ...
        "targetVelocityInertialErrorBound", ...
        "targetPredictionAccelerationInertialErrorBound"];
end

function target = localEmptyTarget()
    target = struct("position", zeros(2, 1), ...
        "velocity", zeros(2, 1), "acceleration", zeros(2, 1), ...
        "length", 0.0, "width", 0.0, ...
        "key", "", ...
        "yaw", 0.0, "yawRate", 0.0, ...
        "positionErrorBound", zeros(2, 1), ...
        "velocityErrorBound", zeros(2, 1), ...
        "predictionAccelerationErrorBound", zeros(2, 1), ...
        "yawErrorBound", 0.0, "yawRateErrorBound", 0.0, ...
        "predictionYawAccelerationErrorBound", 0.0);
end

function key = localTargetRecordKey(data, recordIdx)
    identityFields = ["trackId", "targetId", "objectId", "id"];
    for fieldName = identityFields
        if ~isfield(data, fieldName) || isempty(data.(fieldName))
            continue;
        end
        rawValue = data.(fieldName);
        if (isstring(rawValue) || ischar(rawValue)) ...
                && isscalar(string(rawValue))
            value = string(rawValue);
        elseif isnumeric(rawValue) && isreal(rawValue) ...
                && isscalar(rawValue) && isfinite(rawValue)
            value = string(double(rawValue));
        else
            error("collisionAvoidanceController:invalidInput", ...
                "%s must be a finite scalar identifier.", fieldName);
        end
        if strlength(value) == 0
            error("collisionAvoidanceController:invalidInput", ...
                "%s must not be empty.", fieldName);
        end
        key = fieldName + ":" + value;
        return;
    end
    key = "anonymousTarget:" + string(recordIdx);
end

function [target, active] = localReadTargetRecord(data, ego, cfg)
    target = localEmptyTarget();
    [position, positionAvailable] = localTargetPosition(data, ego);
    if ~positionAvailable
        active = false;
        return;
    end
    velocity = localTargetVelocity(data, ego);
    acceleration = localTargetAcceleration(data, ego);
    lengthValue = localTargetDimension( ...
        data, "targetLength", cfg.target.defaultLength);
    widthValue = localTargetDimension( ...
        data, "targetWidth", cfg.target.defaultWidth);
    yaw = localTargetYaw(data, ego, velocity);
    yawRate = localTargetYawRate(data, velocity, acceleration);

    target.position = position;
    target.velocity = velocity;
    target.acceleration = acceleration;
    target.length = lengthValue;
    target.width = widthValue;
    target.yaw = yaw;
    target.yawRate = yawRate;
    target.positionErrorBound = ...
        localOptionalNonnegativeInputVector( ...
            data, "targetPositionInertialErrorBound", 2);
    target.velocityErrorBound = ...
        localOptionalNonnegativeInputVector( ...
            data, "targetVelocityInertialErrorBound", 2);
    target.predictionAccelerationErrorBound = ...
        localOptionalNonnegativeInputVector( ...
            data, ...
            "targetPredictionAccelerationInertialErrorBound", 2);
    target.yawErrorBound = localOptionalNonnegativeInputScalar( ...
        data, "targetYawErrorBound");
    target.yawRateErrorBound = ...
        localOptionalNonnegativeInputScalar( ...
            data, "targetYawRateErrorBound");
    target.predictionYawAccelerationErrorBound = ...
        localOptionalNonnegativeInputScalar( ...
            data, "targetPredictionYawAccelerationErrorBound");
    active = true;
end

function yaw = localTargetYaw(data, ego, velocity)
    if isfield(data, "targetYawInertial") ...
            && ~isempty(data.targetYawInertial)
        yaw = localFiniteTargetScalar( ...
            data.targetYawInertial, "targetYawInertial");
        yaw = localWrapToPi(yaw);
        return;
    end
    if isfield(data, "targetYawRelative") ...
            && ~isempty(data.targetYawRelative)
        relativeYaw = localFiniteTargetScalar( ...
            data.targetYawRelative, "targetYawRelative");
        yaw = localWrapToPi(ego.yaw + relativeYaw);
        return;
    end

    ambiguousAliases = [ ...
        "targetYawAngle", "targetYaw", "yawAngle", "yaw", "heading"];
    for fieldName = ambiguousAliases
        if isfield(data, fieldName) && ~isempty(data.(fieldName))
            yaw = localFiniteTargetScalar( ...
                data.(fieldName), fieldName);
            frameSpecified = isfield(data, "targetYawFrame") ...
                && ~isempty(data.targetYawFrame);
            if frameSpecified
                frame = localFrame(data, "targetYawFrame", "");
                if ismember(frame, ["ego", "body"])
                    yaw = ego.yaw + yaw;
                end
            end
            yaw = localWrapToPi(yaw);
            return;
        end
    end

    for fieldName = ["relativeYaw", "relativeHeading"]
        if isfield(data, fieldName) && ~isempty(data.(fieldName))
            relativeYaw = localFiniteTargetScalar( ...
                data.(fieldName), fieldName);
            yaw = localWrapToPi(ego.yaw + relativeYaw);
            return;
        end
    end

    speed = norm(velocity);
    if speed > 100.0 * eps(max(1.0, speed))
        yaw = atan2(velocity(2), velocity(1));
        return;
    end
    error("collisionAvoidanceController:invalidInput", ...
        "Every stationary active target requires targetYawInertial " ...
        + "or targetYawRelative for the oriented rectangle geometry.");
end

function yawRate = localTargetYawRate( ...
        data, velocity, acceleration)
    for fieldName = ["targetYawRate", "yawRate", "courseRate"]
        if isfield(data, fieldName) && ~isempty(data.(fieldName))
            yawRate = localFiniteTargetScalar( ...
                data.(fieldName), fieldName);
            return;
        end
    end
    speedSquared = velocity.' * velocity;
    if speedSquared > 100.0 * eps(max(1.0, speedSquared))
        yawRate = localPlanarCross(velocity, acceleration) ...
            / speedSquared;
        return;
    end
    yawRate = 0.0;
end

function value = localFiniteTargetScalar(value, fieldName)
    if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
            || ~isfinite(value)
        error("collisionAvoidanceController:invalidInput", ...
            "%s must be a finite real scalar.", fieldName);
    end
    value = double(value);
end

function [position, available] = localTargetPosition(data, ego)
    available = false;
    position = zeros(2, 1);
    if isfield(data, "targetPositionInertial") ...
            && ~isempty(data.targetPositionInertial)
        rawPosition = data.targetPositionInertial;
        if isnumeric(rawPosition) && numel(rawPosition) == 2 ...
                && all(isnan(rawPosition), "all")
            return;
        end
        position = localFiniteVector( ...
            rawPosition, 2, "targetPositionInertial");
        available = true;
        return;
    end

    if isfield(data, "relativePosition") && ~isempty(data.relativePosition)
        rawPosition = data.relativePosition;
        if isnumeric(rawPosition) && numel(rawPosition) == 2 ...
                && all(isnan(rawPosition), "all")
            return;
        end
        relativePosition = localFiniteVector( ...
            rawPosition, 2, "relativePosition");
    elseif isfield(data, "relativePositionX") ...
            && isfield(data, "relativePositionY")
        rawPosition = [data.relativePositionX; data.relativePositionY];
        if isnumeric(rawPosition) && numel(rawPosition) == 2 ...
                && all(isnan(rawPosition), "all")
            return;
        end
        relativePosition = localFiniteVector( ...
            rawPosition, 2, "relative target position");
    else
        return;
    end
    frameSpecified = isfield(data, "relativePositionFrame") ...
        && ~isempty(data.relativePositionFrame);
    if frameSpecified
        frame = localFrame(data, "relativePositionFrame", "");
    else
        frame = "ego";
    end
    relativePosition = localVectorInInertialFrame( ...
        relativePosition, frame, ego.yaw);
    position = ego.position + relativePosition;
    available = true;
end

function velocity = localTargetVelocity(data, ego)
    if isfield(data, "targetVelocityInertial") ...
            && ~isempty(data.targetVelocityInertial)
        velocity = localFiniteVector( ...
            data.targetVelocityInertial, 2, "targetVelocityInertial");
        return;
    end
    if isfield(data, "targetVelocity") && ~isempty(data.targetVelocity)
        velocity = localFiniteVector( ...
            data.targetVelocity, 2, "targetVelocity");
        frame = localRequiredFrame(data, "targetVelocityFrame");
        velocity = localVectorInInertialFrame(velocity, frame, ego.yaw);
        return;
    end
    if isfield(data, "targetVelocityX") ...
            && isfield(data, "targetVelocityY")
        velocity = localFiniteVector([data.targetVelocityX; ...
            data.targetVelocityY], 2, "target velocity");
        frame = localRequiredFrame(data, "targetVelocityFrame");
        velocity = localVectorInInertialFrame(velocity, frame, ego.yaw);
        return;
    end

    if isfield(data, "relativeVelocity") && ~isempty(data.relativeVelocity)
        relativeVelocity = localFiniteVector( ...
            data.relativeVelocity, 2, "relativeVelocity");
    elseif isfield(data, "relativeVelocityX") ...
            && isfield(data, "relativeVelocityY")
        relativeVelocity = localFiniteVector([data.relativeVelocityX; ...
            data.relativeVelocityY], 2, "relative velocity");
    else
        error("collisionAvoidanceController:invalidInput", ...
            "Every active target requires an inertial target velocity " ...
            + "or a relative velocity with an explicit frame contract.");
    end
    frame = localRequiredFrame(data, "relativeVelocityFrame");
    relativeVelocity = localVectorInInertialFrame( ...
        relativeVelocity, frame, ego.yaw);
    velocity = ego.inertialVelocity + relativeVelocity;
end

function acceleration = localTargetAcceleration(data, ego)
    if isfield(data, "targetAccelerationInertial") ...
            && ~isempty(data.targetAccelerationInertial)
        acceleration = localFiniteVector( ...
            data.targetAccelerationInertial, 2, ...
            "targetAccelerationInertial");
        return;
    end
    if isfield(data, "targetAcceleration") ...
            && ~isempty(data.targetAcceleration)
        acceleration = localFiniteVector( ...
            data.targetAcceleration, 2, "targetAcceleration");
        frame = localRequiredFrame(data, "targetAccelerationFrame");
        acceleration = localVectorInInertialFrame( ...
            acceleration, frame, ego.yaw);
        return;
    end
    if isfield(data, "targetAccelerationX") ...
            && isfield(data, "targetAccelerationY")
        acceleration = localFiniteVector([ ...
            data.targetAccelerationX; data.targetAccelerationY], ...
            2, "target acceleration");
        frame = localRequiredFrame(data, "targetAccelerationFrame");
        acceleration = localVectorInInertialFrame( ...
            acceleration, frame, ego.yaw);
        return;
    end
    error("collisionAvoidanceController:invalidInput", ...
        "Every active target requires targetAccelerationInertial " ...
        + "from the motion estimator, or an equivalent target " ...
        + "acceleration with an explicit frame contract.");
end

function frame = localFrame(data, fieldName, defaultFrame)
    frame = defaultFrame;
    if isfield(data, fieldName) && ~isempty(data.(fieldName))
        frame = lower(string(data.(fieldName)));
    end
    if ~isscalar(frame) || ~ismember(frame, ["ego", "body", "inertial", "world"])
        error("collisionAvoidanceController:invalidInput", ...
            "%s must identify the ego/body or inertial/world frame.", ...
            fieldName);
    end
end

function frame = localRequiredFrame(data, fieldName)
    if ~isfield(data, fieldName) || isempty(data.(fieldName))
        error("collisionAvoidanceController:invalidInput", ...
            "%s is required for coordinate-ambiguous target vectors.", ...
            fieldName);
    end
    frame = localFrame(data, fieldName, "");
end

function vector = localVectorInInertialFrame(vector, frame, yaw)
    if ismember(frame, ["ego", "body"])
        vector = localRotationMatrix(yaw) * vector;
    end
end

function value = localTargetDimension(data, fieldName, defaultValue)
    value = defaultValue;
    if isfield(data, fieldName) && ~isempty(data.(fieldName))
        rawValue = data.(fieldName);
        if ~isnumeric(rawValue) || ~isreal(rawValue) ...
                || ~isscalar(rawValue) || ~isfinite(rawValue) ...
                || rawValue <= 0.0
            error("collisionAvoidanceController:invalidInput", ...
                "%s must be a positive finite scalar.", fieldName);
        end
        value = double(rawValue);
    end
end

function rotation = localRotationMatrix(angle)
    rotation = [cos(angle), -sin(angle); sin(angle), cos(angle)];
end

function angle = localWrapToPi(angle)
    angle = atan2(sin(angle), cos(angle));
end

function value = localPlanarCross(firstVector, secondVector)
    value = firstVector(1) * secondVector(2) ...
        - firstVector(2) * secondVector(1);
end
