function [ego,lane,road,target] = readControllerInputs(egoState,targetEstimate,laneCenterline,cfg)
%readControllerInputs Normalize joint state, reference path and optional road metadata.
    if isempty(targetEstimate) && isstruct(egoState) && isfield(egoState,'targetEstimate')
        targetEstimate=egoState.targetEstimate;
    end
    ego=localReadEgoState(egoState);
    ego.stateTime=localOptionalStateScalar(egoState,"stateTime",NaN);
    [lane,road]=localReadLane(laneCenterline,ego);
    target=[];
    if isempty(targetEstimate),return;end
    validateattributes(targetEstimate,{'struct'},{'scalar'},mfilename,'targetEstimate');
    if isempty(fieldnames(targetEstimate)),return;end
    [position,available]=localTargetPosition(targetEstimate,ego);
    if ~available,return;end
    velocity=localTargetVelocity(targetEstimate,ego);
    acceleration=localTargetAcceleration(targetEstimate,ego);
    tangentialAcceleration=[];
    for field=["targetTangentialAcceleration","targetScalarAcceleration"]
        if isfield(targetEstimate,field) && ~isempty(targetEstimate.(field))
            tangentialAcceleration=localFiniteTargetScalar(targetEstimate.(field),field);break;
        end
    end
    target=struct('position',position,'velocity',velocity,'acceleration',acceleration, ...
        'tangentialAcceleration',tangentialAcceleration, ...
        'yaw',localTargetYaw(targetEstimate,ego,velocity), ...
        'rearAxleDistance',localTargetDimension(targetEstimate,"targetRearAxleDistance",cfg.target.rearAxleDistance), ...
        'length',localTargetDimension(targetEstimate,"targetLength",cfg.target.defaultLength), ...
        'width',localTargetDimension(targetEstimate,"targetWidth",cfg.target.defaultWidth));
end

function ego = localReadEgoState(data)
    if ~isstruct(data) || ~isscalar(data)
        error("collisionAvoidanceController:invalidInput", ...
            "egoState must be a scalar structure.");
    end
    if isfield(data, "egoState")
        ego = localReadNrmmEgoState(data);
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
    rotation = localRotationMatrix(yaw);
    ego = struct();
    ego.position = position;
    ego.yaw = yaw;
    ego.inertialVelocity = rotation ...
        * [longitudinalVelocity; lateralVelocity];
    ego.modelState = [position; yaw; longitudinalVelocity; ...
        lateralVelocity; yawRate];
    ego.heldActuatorInput = localOptionalHeldActuatorInput(data);
end

function ego = localReadNrmmEgoState(data)
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
    ego = struct();
    ego.position = position;
    ego.yaw = yaw;
    ego.inertialVelocity = inertialVelocity;
    ego.modelState = [position; yaw; bodyVelocity; yawRate];
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
% Previous applied actuator input [steering radians; signed braking ratio].
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

function [lane, road] = localReadLane(rawGeometry, ego)
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
    if isempty(rawLane) && isstruct(rawGeometry) && isfield(rawGeometry,"referenceCurve") ...
            && ~isempty(rawGeometry.referenceCurve)
        curve=laneGeometry.validateReferenceCurve(rawGeometry.referenceCurve);
        rawLane=laneGeometry.referencePose(linspace(0,curve.length,max(2,ceil(curve.length/2)+1)),0,curve).';
    end
    lane = localReadCenterline(rawLane, ego);
    if isstruct(rawGeometry) && isfield(rawGeometry, "referenceCurve") ...
            && ~isempty(rawGeometry.referenceCurve)
        lane.referenceCurve = laneGeometry.validateReferenceCurve(rawGeometry.referenceCurve);
    end
    road=struct('lateralClearance',zeros(2,0));
    if isstruct(rawGeometry)
        if isfield(rawGeometry,'boundaries') && ~isempty(rawGeometry.boundaries)
            error('collisionAvoidanceController:unsupportedRoad','Use a global lateralClearance corridor.');
        end
        if isfield(rawGeometry,'lateralClearance') && ~isempty(rawGeometry.lateralClearance)
            road.lateralClearance=localFiniteVector(rawGeometry.lateralClearance,2,'lateralClearance');
            if any(road.lateralClearance<=0)
                error('collisionAvoidanceController:invalidRoadClearance','Lateral clearances must be positive.');
            end
        end
    end
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

function yaw = localTargetYaw(data, ego, velocity)
    for field = ["targetYawInertial", "targetHeadingInertial"]
        if isfield(data, field) && ~isempty(data.(field))
            yaw = localWrapToPi(localFiniteTargetScalar(data.(field), field));
            return;
        end
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
    acceleration=zeros(2,1);
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
