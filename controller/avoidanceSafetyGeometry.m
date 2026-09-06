function geometry = avoidanceSafetyGeometry(model, prediction, state, carried)
%avoidanceSafetyGeometry Supporting halfspaces in certified Cartesian frames.
% Each node has an affine chart with certified position/heading error over
% a station interval spanning polyline segments. A supporting certificate replaces a
% carried one only when it admits the carried state. This is geometric
% evaluation, with no trajectory optimization or alternative solver start.

    arguments
        model (1,1) struct
        prediction (1,1) struct
        state (6,:) double
        carried = []
    end
    count = prediction.nodeCount;
    empty = localEmptyNode(prediction.planCount);
    collision = struct("name", "collision", "id", model.targetKey, ...
        "nodes", repmat(empty, count, 1));
    road = repmat(struct("name", "road", "id", "", ...
        "nodes", repmat(empty, count, 1)), numel(model.road.boundaries), 1);
    candidates = laneGeometry.frameBounds(model.lane, state(1, :), ...
        model.cfg.controller.stationTrustRadius, model.cfg.model.lateralDomainRadius);
    if prediction.scheduleSpeedProfile(end-1) == 0.0
        % The final rest step has a single pose and must use one chart.
        candidates(count-1) = candidates(count);
    end
    roadCandidates = cell(numel(road), 1);
    for boundaryIdx = 1:numel(road)
        boundary = model.road.boundaries(boundaryIdx);
        road(boundaryIdx).id = boundary.boundaryId;
        roadCandidates{boundaryIdx} = localRoadNodes( ...
            empty, model, prediction, state, candidates, boundary);
    end
    frames = candidates;
    replaced = 0;
    for nodeIdx = 1:count
        nominal = state(:, nodeIdx);
        frame = candidates(nodeIdx);
        targetNode = localTargetNode(empty, model, prediction, nominal, frame, nodeIdx);
        roadNodes = repmat(empty, numel(road), 1);
        admitted = nominal(1) >= frame.stationLower ...
            && nominal(1) <= frame.stationUpper;
        if targetNode.covered
            admitted = admitted && targetNode.nominalMargin >= 0.0;
        end
        for boundaryIdx = 1:numel(road)
            roadNodes(boundaryIdx) = roadCandidates{boundaryIdx}(nodeIdx);
            admitted = admitted && (~roadNodes(boundaryIdx).covered ...
                || roadNodes(boundaryIdx).nominalMargin >= 0.0);
            if ~isempty(carried) && carried.road(boundaryIdx).nodes(nodeIdx).covered
                admitted = admitted && roadNodes(boundaryIdx).covered;
            end
        end
        if ~isempty(carried) && ~admitted
            frames(nodeIdx) = carried.frames(nodeIdx);
            collision.nodes(nodeIdx) = carried.collision.nodes(nodeIdx);
            for boundaryIdx = 1:numel(road)
                road(boundaryIdx).nodes(nodeIdx) = ...
                    carried.road(boundaryIdx).nodes(nodeIdx);
            end
        else
            frames(nodeIdx) = frame;
            collision.nodes(nodeIdx) = targetNode;
            for boundaryIdx = 1:numel(road)
                road(boundaryIdx).nodes(nodeIdx) = roadNodes(boundaryIdx);
            end
            replaced = replaced+double(~isempty(carried));
        end
    end
    geometry = struct("frames", frames, "collision", collision, ...
        "road", road, "reanchoredCount", replaced);
end

function node = localTargetNode(node, model, prediction, state, frame, nodeIdx)
    if ~model.hasTarget
        return;
    end
    horizon = model.targetHorizon;
    position = frame.origin+[frame.tangent, frame.lateral]*state(1:2);
    heading = frame.heading+state(3);
    targetPosition = horizon.targetPosition(:, nodeIdx);
    targetHeading = horizon.targetYaw(nodeIdx);
    dimensions = [model.egoHalfLength; model.egoHalfWidth; ...
        model.targetHalfLength; model.targetHalfWidth];
    [distance, normal] = rectangleConfigurationDistance( ...
        position, heading, targetPosition, targetHeading, dimensions);
    terminal = nodeIdx == prediction.nodeCount;
    if terminal
        directions = horizon.terminalSupportDirection;
        support = horizon.terminalFuturePositionSupport;
        margins = directions.'*position-support.' ...
            - hypot(model.targetHalfLength, model.targetHalfWidth);
        for directionIdx = 1:numel(support)
            margins(directionIdx) = margins(directionIdx)-localRectangleSupport( ...
                model.egoHalfLength, model.egoHalfWidth, ...
                directions(:, directionIdx), heading, 0.0);
        end
        [~, selected] = max(margins);
        normal = directions(:, selected);
        targetSupport = support(selected)+hypot(model.targetHalfLength, model.targetHalfWidth);
        node.terminalInvariant = isfinite(targetSupport);
        node.terminalFutureSupport = support(selected);
        node.terminalContinuationAxis = "predictedTrajectorySupport";
        node.terminalSupportDirection = normal;
    else
        targetSupport = normal.'*targetPosition+localRectangleSupport( ...
            model.targetHalfLength, model.targetHalfWidth, normal, ...
            targetHeading, horizon.targetYawErrorBound(nodeIdx)) ...
            + abs(normal).'*horizon.targetPositionErrorBound(:, nodeIdx);
    end
    node = localSupportNode(node, model, prediction, state, frame, ...
        nodeIdx, normal, targetSupport, model.cfg.collision.clearanceMargin);
    node.dualDistance = distance;
    node.terminalSegmentIndex = frame.segmentIndex;
    node.terminalSegmentEnforced = false;
    node.terminalStationLower = frame.stationLower;
    node.terminalStationUpper = frame.stationUpper;
end

function nodes = localRoadNodes(empty, model, prediction, state, frames, boundary)
% Bound every quadratic graph over the complete admitted rectangle range.
% Batch node arithmetic; retain the same endpoint/stationary-point extrema.
    cfg = model.cfg;
    count = numel(frames);
    nodes = repmat(empty, count, 1);
    longitudinal = boundary.longitudinalDirection;
    tangent = reshape([frames.tangent], 2, []);
    lateral = reshape([frames.lateral], 2, []);
    origins = reshape([frames.origin], 2, []);
    errors = reshape([frames.positionErrorBound], 2, []);
    first = longitudinal.'*tangent;
    second = longitudinal.'*lateral;
    origin = longitudinal.'*(origins-boundary.origin);
    stationRange = first.*[[frames.stationLower]; [frames.stationUpper]];
    extent = abs(second)*cfg.model.lateralDomainRadius ...
        + hypot(model.egoHalfLength, model.egoHalfWidth) ...
        + abs(longitudinal).'*errors;
    range = [min(stationRange, [], 1)-extent; max(stationRange, [], 1)+extent]+origin;
    tolerance = cfg.road.parameterRangeTolerance;
    outside = range(2, :) < boundary.parameterRange(1)-tolerance ...
        | range(1, :) > boundary.parameterRange(2)+tolerance;
    covered = range(1, :) >= boundary.parameterRange(1)-tolerance ...
        & range(2, :) <= boundary.parameterRange(2)+tolerance;
    if boundary.coveragePolicy ~= "perceptionLimited" && any(~outside & ~covered)
        nodeIdx = find(~outside & ~covered, 1);
        error("collisionAvoidanceController:roadBoundaryCoverageGap", ...
            "Road boundary %s does not cover node %d's admitted rectangle range.", ...
            boundary.boundaryId, nodeIdx-1);
    end
    selected = find(~outside & covered);
    if isempty(selected), return; end
    range = range(:, selected);
    polynomial = boundary.safeSideSign*boundary.coefficients;
    samples = range;
    if polynomial(1) < 0.0
        stationary = -polynomial(2)/(2.0*polynomial(1));
        samples = [samples; min(max(stationary, range(1, :)), range(2, :))];
    end
    graphSupport = max((polynomial(1)*samples+polynomial(2)).*samples+polynomial(3), [], 1);
    normal = boundary.safeSideSign*boundary.lateralDirection;
    maxSlope = max(abs(2.0*boundary.coefficients(1)*range+boundary.coefficients(2)), [], 1);
    tightening = (cfg.collision.clearanceMargin ...
        + boundary.normalDistanceErrorBound)*hypot(1.0, maxSlope);
    nodes(selected) = localSupportNodes(empty, model, prediction, state(:, selected), ...
        frames(selected), selected, repmat(normal, 1, numel(selected)), ...
        normal.'*boundary.origin+graphSupport, tightening);
end

function nodes = localSupportNodes(empty, model, prediction, state, frames, ...
        indices, inertialNormal, obstacleSupport, clearance)
    frameHeading = [frames.heading];
    heading = state(3, :)+frameHeading;
    radius = prediction.egoStateErrorBound(3, indices)+[frames.headingErrorBound];
    tangent = reshape([frames.tangent], 2, []);
    lateral = reshape([frames.lateral], 2, []);
    normal = [sum(tangent.*inertialNormal, 1); sum(lateral.*inertialNormal, 1)];
    egoSupport = localRectangleSupport(model.egoHalfLength, ...
        model.egoHalfWidth, inertialNormal, heading, radius);
    headingDomain = model.cfg.model.headingDomainRadius+radius;
    slope = localSupportSlopeBound(model.egoHalfLength, model.egoHalfWidth, ...
        inertialNormal, frameHeading+min(-headingDomain, state(3, :)-radius), ...
        frameHeading+max(headingDomain, state(3, :)+radius));
    origins = reshape([frames.origin], 2, []);
    errors = reshape([frames.positionErrorBound], 2, []);
    supportValue = obstacleSupport+egoSupport-sum(inertialNormal.*origins, 1);
    tightening = clearance ...
        + sum(abs(normal).*prediction.egoStateErrorBound(1:2, indices), 1) ...
        + sum(abs(inertialNormal).*errors, 1);
    margin = sum(normal.*state(1:2, :), 1)-supportValue-tightening;
    nodes = repmat(empty, numel(indices), 1);
    for index = 1:numel(indices)
        nodes(index).covered = true;
        nodes(index).normal = normal(:, index);
        nodes(index).egoSupport = egoSupport(index);
        nodes(index).targetSupport = obstacleSupport(index);
        nodes(index).supportValue = supportValue(index);
        nodes(index).headingCoefficient = slope(index);
        nodes(index).nominalHeading = state(3, index);
        nodes(index).tightening = tightening(index);
        nodes(index).nominalMargin = margin(index);
        nodes(index).outside = margin(index) >= 0.0;
        if abs(normal(2, index)) <= 0.087
            nodes(index).regionCode = 1+double(normal(1, index) > 0.0);
        elseif abs(normal(1, index)) <= 0.087
            nodes(index).regionCode = 3+double(normal(2, index) < 0.0);
        else
            nodes(index).regionCode = 5+2*double(normal(1, index) > 0.0)+double(normal(2, index) < 0.0);
        end
    end
end

function node = localSupportNode(node, model, prediction, state, frame, ...
        nodeIdx, inertialNormal, obstacleSupport, clearance)
    heading = state(3)+frame.heading;
    radius = prediction.egoStateErrorBound(3, nodeIdx)+frame.headingErrorBound;
    normal = [frame.tangent, frame.lateral].'*inertialNormal;
    egoSupport = localRectangleSupport(model.egoHalfLength, ...
        model.egoHalfWidth, inertialNormal, heading, radius);
    headingDomain = model.cfg.model.headingDomainRadius+radius;
    slope = localSupportSlopeBound(model.egoHalfLength, model.egoHalfWidth, ...
        inertialNormal, frame.heading+min(-headingDomain, state(3)-radius), ...
        frame.heading+max(headingDomain, state(3)+radius));
    node.covered = true;
    node.normal = normal;
    node.egoSupport = egoSupport;
    node.targetSupport = obstacleSupport;
    node.supportValue = obstacleSupport+egoSupport-inertialNormal.'*frame.origin;
    node.headingCoefficient = slope;
    node.nominalHeading = state(3);
    node.tightening = clearance ...
        + abs(normal).'*prediction.egoStateErrorBound(1:2, nodeIdx) ...
        + abs(inertialNormal).'*frame.positionErrorBound;
    node.nominalMargin = normal.'*state(1:2)-node.supportValue-node.tightening;
    node.outside = node.nominalMargin >= 0.0;
    if abs(normal(2)) <= 0.087
        node.regionCode = 1+double(normal(1) > 0.0);
    elseif abs(normal(1)) <= 0.087
        node.regionCode = 3+double(normal(2) < 0.0);
    else
        node.regionCode = 5+2*double(normal(1) > 0.0)+double(normal(2) < 0.0);
    end
end

function node = localEmptyNode(count)
    node = struct("covered", false, "imposed", false, "normal", zeros(2, 1), ...
        "regionCode", 0, "supportValue", 0.0, "targetSupport", 0.0, ...
        "egoSupport", 0.0, "headingCoefficient", 0.0, "nominalHeading", 0.0, ...
        "tightening", 0.0, "dualDistance", inf, "nominalMargin", inf, ...
        "terminalInvariant", false, "terminalContinuationAxis", "", ...
        "terminalFutureSupport", inf, "terminalSupportDirection", zeros(2, 1), ...
        "terminalSegmentEnforced", false, "terminalSegmentIndex", 0, ...
        "terminalStationLower", -inf, "terminalStationUpper", inf, ...
        "outside", true, "marginMatrix", zeros(2, count), ...
        "marginOffset", zeros(2, 1));
end

function value = localRectangleSupport(halfLength, halfWidth, normal, ...
        yaw, yawRadius)
% The support of an oriented rectangle about its centre, in the unit
% direction n, maximized over the declared yaw uncertainty:
%
%   h(psi) = l |cos(alpha - psi)| + w |sin(alpha - psi)|,
%   value  = max_{|delta| <= r} h(yaw + delta),   alpha = angle(n).
%
% h is pi-periodic and, between its kinks at multiples of pi/2, equals
% R cos(theta -+ phi) with R = hypot(l, w) and phi = atan2(w, l): its
% maxima are at theta = +-phi + k pi, where it reaches R. The maximum
% over an interval is therefore the larger endpoint value, or R when a
% maximizer falls inside. Exact, and the direction-wise treatment of a
% yaw radius that an axis-aligned box can only approximate.
    alpha = atan2(normal(2, :), normal(1, :));
    low = alpha-yaw-yawRadius;
    high = alpha-yaw+yawRadius;
    value = max(localSupportAtAngle(halfLength, halfWidth, low), ...
        localSupportAtAngle(halfLength, halfWidth, high));
    phase = atan2(halfWidth, halfLength);
    reachesMaximum = phase+pi*ceil((low-phase)/pi) <= high ...
        | -phase+pi*ceil((low+phase)/pi) <= high;
    value(reachesMaximum) = hypot(halfLength, halfWidth);
end

function value = localSupportAtAngle(halfLength, halfWidth, angle)
    value = halfLength*abs(cos(angle))+halfWidth*abs(sin(angle));
end

function slope = localSupportSlopeBound(halfLength, halfWidth, normal, ...
        yawLow, yawHigh)
% THE CONSERVATIVE AFFINE BOUND ON THE EGO'S YAW. Stage 2 freezes n but
% not the ego's heading, which the decision moves; the support
% h(psi) = l |cos(alpha - psi)| + w |sin(alpha - psi)| is not affine in
% psi, so the row charges
%
%   h(psi) <= h(psibar) + slope * |psi - psibar|,
%
% two affine rows (sigma = +-1) whose minimum is the bound. This
% returns the exact Lipschitz constant over the admitted interval. Between
% consecutive kinks, h'' = -h < 0, so h' is monotone and |h'| reaches its
% maximum at an endpoint. Only the interval endpoints and the two families
% of kinks are needed; no piece enumeration or sorting is required.
    alpha = atan2(normal(2, :), normal(1, :));
    low = alpha-yawHigh;
    high = alpha-yawLow;
    endpoints = [low; high];
    slope = max(abs(halfLength*abs(sin(endpoints)) ...
        - halfWidth*abs(cos(endpoints))), [], 1);
    lengthKink = pi/2+pi*ceil((low-pi/2)/pi) <= high;
    widthKink = pi*ceil(low/pi) <= high;
    slope(lengthKink) = max(slope(lengthKink), halfLength);
    slope(widthKink) = max(slope(widthKink), halfWidth);
    slope(high <= low) = hypot(halfLength, halfWidth);
end
