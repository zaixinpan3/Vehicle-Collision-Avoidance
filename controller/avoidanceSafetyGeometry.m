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
            boundary = model.road.boundaries(boundaryIdx);
            road(boundaryIdx).id = boundary.boundaryId;
            roadNodes(boundaryIdx) = localRoadNode( ...
                empty, model, prediction, nominal, frame, nodeIdx, boundary);
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

function node = localRoadNode(node, model, prediction, state, frame, nodeIdx, boundary)
% Bound the complete quadratic graph over the rectangle's admitted range.
% Its maximum is attained at an endpoint or the quadratic stationary point.
% This replaces sampled/interpolated road offsets with a sound halfspace.
    cfg = model.cfg;
    longitudinal = boundary.longitudinalDirection;
    coefficients = longitudinal.'*[frame.tangent, frame.lateral];
    origin = longitudinal.'*(frame.origin-boundary.origin);
    stationRange = coefficients(1)*[frame.stationLower, frame.stationUpper];
    extent = abs(coefficients(2))*cfg.model.lateralDomainRadius ...
        + hypot(model.egoHalfLength, model.egoHalfWidth) ...
        + abs(longitudinal).'*frame.positionErrorBound;
    range = [min(stationRange)-extent, max(stationRange)+extent]+origin;
    tolerance = cfg.road.parameterRangeTolerance;
    outside = range(2) < boundary.parameterRange(1)-tolerance ...
        || range(1) > boundary.parameterRange(2)+tolerance;
    covered = range(1) >= boundary.parameterRange(1)-tolerance ...
        && range(2) <= boundary.parameterRange(2)+tolerance;
    if outside || (~covered && boundary.coveragePolicy == "perceptionLimited")
        return;
    end
    if ~covered
        error("collisionAvoidanceController:roadBoundaryCoverageGap", ...
            "Road boundary %s does not cover node %d's admitted rectangle range.", ...
            boundary.boundaryId, nodeIdx-1);
    end
    polynomial = boundary.safeSideSign*boundary.coefficients;
    samples = range;
    if polynomial(1) < 0.0
        stationary = -polynomial(2)/(2.0*polynomial(1));
        samples = [samples, min(max(stationary, range(1)), range(2))];
    end
    graphSupport = max((polynomial(1)*samples+polynomial(2)).*samples+polynomial(3));
    normal = boundary.safeSideSign*boundary.lateralDirection;
    maxSlope = max(abs(2.0*boundary.coefficients(1)*range+boundary.coefficients(2)));
    tightening = (cfg.collision.clearanceMargin ...
        + boundary.normalDistanceErrorBound)*hypot(1.0, maxSlope);
    node = localSupportNode(node, model, prediction, state, frame, ...
        nodeIdx, normal, normal.'*boundary.origin+graphSupport, tightening);
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
    alpha = atan2(normal(2), normal(1));
    low = alpha-yaw-yawRadius;
    high = alpha-yaw+yawRadius;
    value = max(localSupportAtAngle(halfLength, halfWidth, low), ...
        localSupportAtAngle(halfLength, halfWidth, high));
    if yawRadius <= 0.0
        return;
    end
    phase = atan2(halfWidth, halfLength);
    for phaseSign = [-1.0, 1.0]
        maximizer = phaseSign*phase ...
            + pi*ceil((low-phaseSign*phase)/pi);
        if maximizer <= high
            value = hypot(halfLength, halfWidth);
            return;
        end
    end
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
% returns the exact Lipschitz constant of h over the interval the
% program admits: max |dh/dpsi| there. With theta = alpha - psi,
% dh/dtheta = -l sgn(cos) sin + w sgn(sin) cos is sinusoidal of
% amplitude R = hypot(l, w) between the kinks at multiples of pi/2, so
% its magnitude is maximized at an interval endpoint, at a kink (where
% it is l or w) or at the sinusoid's own peak (where it is R) when that
% falls inside. Exact, hence never worse than the retired box bound
% (which charged w along the station axis and l along the lateral one,
% and their sum for a diagonal normal); for a diagonal normal at the
% declared heading domain it is measurably smaller.
    alpha = atan2(normal(2), normal(1));
    low = alpha-yawHigh;
    high = alpha-yawLow;
    radius = hypot(halfLength, halfWidth);
    if ~(high > low)
        slope = radius;
        return;
    end
    kinks = ceil(low/(pi/2))*(pi/2):(pi/2):high;
    edges = unique([low, kinks, high]);
    slope = 0.0;
    for pieceIdx = 1:numel(edges)-1
        pieceLow = edges(pieceIdx);
        pieceHigh = edges(pieceIdx+1);
        middle = 0.5*(pieceLow+pieceHigh);
        cosineSign = localNonzeroSign(cos(middle));
        sineSign = localNonzeroSign(sin(middle));
        % dh/dtheta = cosineCoefficient cos(theta) + sineCoefficient sin(theta)
        cosineCoefficient = halfWidth*sineSign;
        sineCoefficient = -halfLength*cosineSign;
        value = max( ...
            abs(cosineCoefficient*cos(pieceLow) ...
                + sineCoefficient*sin(pieceLow)), ...
            abs(cosineCoefficient*cos(pieceHigh) ...
                + sineCoefficient*sin(pieceHigh)));
        peak = atan2(sineCoefficient, cosineCoefficient);
        extremum = peak+pi*ceil((pieceLow-peak)/pi);
        if extremum >= pieceLow && extremum <= pieceHigh
            value = radius;
        end
        slope = max(slope, value);
    end
end

function value = localNonzeroSign(value)
    if value < 0.0
        value = -1.0;
    else
        value = 1.0;
    end
end
