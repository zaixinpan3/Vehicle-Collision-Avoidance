classdef stateUncertainty
    %stateUncertainty Estimator certificates and state-box propagation, intersection and rest.

    methods (Static)
        function certificate = readCertificate(data, kind)
        %stateUncertainty.readCertificate Validate a current estimator enclosure.
        % A published certificate takes precedence over legacy numeric aliases.
        % Its timestamp must match the estimate; unavailable radii are never zeroed.

            certificate = [];
            if ~isfield(data, "controllerErrorBound")
                if isfield(data, "relativePositionErrorBound") ...
                        || isfield(data, "egoYawErrorBound")
                    error("collisionAvoidanceController:missingEstimatorBound", ...
                        "Estimator output must include its controllerErrorBound certificate.");
                end
                return;
            end
            raw = data.controllerErrorBound;
            required = ["kind", "time", "bounds", "available", ...
                "source", "futurePredictionIncluded"];
            if ~isstruct(raw) || ~isscalar(raw) || ~all(isfield(raw, required)) ...
                    || ~isscalar(string(raw.kind)) || string(raw.kind) ~= kind ...
                    || ~isscalar(string(raw.source)) || strlength(string(raw.source)) == 0 ...
                    || ~islogical(raw.available) || ~isscalar(raw.available) ...
                    || ~isequal(raw.futurePredictionIncluded, false)
                error("collisionAvoidanceController:invalidEstimatorBound", ...
                    "The current estimation-error certificate has an invalid schema or scope.");
            end
            if ~raw.available
                error("collisionAvoidanceController:unavailableEstimatorBound", ...
                    "The estimator marked its state-time enclosure unavailable.");
            end
            count = 6+2*double(kind == "target-state-v1");
            if ~isnumeric(raw.bounds) || ~isreal(raw.bounds) ...
                    || ~isvector(raw.bounds) || numel(raw.bounds) ~= count ...
                    || any(~isfinite(raw.bounds)) || any(raw.bounds < 0.0)
                error("collisionAvoidanceController:invalidEstimatorBound", ...
                    "Estimator bounds must be finite nonnegative component bounds.");
            end
            if ~isfield(data, "stateTime") || ~localFiniteTime(data.stateTime) ...
                    || ~localFiniteTime(raw.time)
                error("collisionAvoidanceController:invalidEstimatorBound", ...
                    "Both the estimate and its bound need finite state timestamps.");
            end
            tolerance = 128*eps(max([1.0, abs(data.stateTime), abs(raw.time)]));
            if abs(raw.time-data.stateTime) > tolerance
                error("collisionAvoidanceController:staleEstimatorBound", ...
                    "The estimation bound and state timestamps do not match.");
            end
            certificate = raw;
            certificate.bounds = double(raw.bounds(:));
        end

        function [radius, chartValid] = toFrenet(cartesianState, cartesianRadius, lane)
        %stateUncertainty.toFrenet Enclose projection of a Cartesian state box.
        % Cover every polyline segment that can be closest to a point in the box.
        % Station uses clipped segment projections; heading includes tangent changes.
        % Output order is [s; d; ePsi; vx; vy; yawRate].

            validateattributes(cartesianRadius, {'double'}, ...
                {'real', 'finite', 'nonnegative', 'numel', 6});
            cartesianRadius = cartesianRadius(:);
            radius = cartesianRadius;
            position = cartesianState(1:2);
            centre = laneGeometry.project(position, lane);
            offset = position(:).'-lane.segmentStart;
            along = sum(offset.*lane.tangent, 2);
            clipped = min(max(along, 0), lane.segmentLength);
            points = lane.segmentStart+clipped.*lane.tangent;
            distances = vecnorm(position(:).'-points, 2, 2);
            distanceRadius = norm(cartesianRadius(1:2));
            tolerance = 128*eps(max(1, max(distances)));
            possible = distances <= min(distances)+2*distanceRadius+tolerance;
            tangents = lane.tangent(possible, :);
            normals = [-tangents(:, 2), tangents(:, 1)];
            alongRadius = abs(tangents)*cartesianRadius(1:2);
            starts = lane.segmentStation(possible);
            lengths = lane.segmentLength(possible);
            % The projected coordinates need not invert to the physical position
            % at a clipped endpoint or a polyline corner. Robust admission requires
            % one interior chart for the entire initial box. The radius above/below
            % still encloses projection when this stronger condition is false.
            chartValid = nnz(possible) == 1 ...
                && all(along(possible)-alongRadius > 0) ...
                && all(along(possible)+alongRadius < lengths);
            if any(cartesianRadius(1:2))
                stationLower = starts+min(max(along(possible)-alongRadius, 0), lengths);
                stationUpper = starts+min(max(along(possible)+alongRadius, 0), lengths);
                radius(1) = max(abs([stationLower; stationUpper]-centre.station));
                lateral = sum(offset(possible, :).*normals, 2);
                lateralRadius = abs(normals)*cartesianRadius(1:2);
                radius(2) = max(abs(lateral-centre.lateralPosition)+lateralRadius);
                headings = atan2(tangents(:, 2), tangents(:, 1));
                headingChange = abs(atan2(sin(headings-centre.heading), ...
                    cos(headings-centre.heading)));
                radius(3) = cartesianRadius(3)+max(headingChange);
            end
            centreError = atan2(sin(cartesianState(3)-centre.heading), ...
                cos(cartesianState(3)-centre.heading));
            if radius(3) > 0 && abs(centreError)+radius(3) >= pi
                radius(3) = 2*pi;
            end
        end

        function radius = heldDisturbance(continuousA, rateRadius, sampleTime)
        %stateUncertainty.heldDisturbance Enclose a continuous disturbance over a held step.
        % The Metzler comparison preserves each diagonal and takes the absolute
        % off-diagonal entries. Its exponential bounds the absolute state transition,
        % so the integral covers coupling between all continuous disturbance channels.

            validateattributes(continuousA, {'double'}, {'real', 'finite', 'square'});
            validateattributes(sampleTime, {'double'}, {'real', 'finite', 'scalar', 'positive'});
            validateattributes(rateRadius, {'double'}, ...
                {'real', 'finite', 'nonnegative', 'numel', size(continuousA, 1)});
            count = size(continuousA, 1);
            radius = zeros(count, 1);
            if ~any(rateRadius)
                return;
            end
            comparison = abs(continuousA);
            comparison(1:count+1:end) = diag(continuousA);
            transition = expm(sampleTime*[comparison, rateRadius(:); zeros(1, count+1)]);
            radius = max(0, transition(1:count, end));
        end

        function [radius, consistent] = intersect( ...
                predictedCenter, predictedRadius, estimate, estimateRadius)
        %stateUncertainty.intersect Retain a nominal center while assimilating a box.
        % Both boxes are assumed to contain the same true state in the same chart.
        % The returned symmetric outer box contains their intersection and remains
        % inside the predicted box. Empty intersection is a contract inconsistency.

            validateattributes(predictedRadius, {'double'}, ...
                {'real', 'finite', 'column', 'nonnegative'});
            validateattributes(estimateRadius, {'double'}, ...
                {'real', 'finite', 'size', size(predictedRadius), 'nonnegative'});
            validateattributes(predictedCenter, {'double'}, ...
                {'real', 'finite', 'size', size(predictedRadius)});
            validateattributes(estimate, {'double'}, ...
                {'real', 'finite', 'size', size(predictedRadius)});
            displacement = estimate-predictedCenter;
            lower = max(-predictedRadius, displacement-estimateRadius);
            upper = min(predictedRadius, displacement+estimateRadius);
            consistent = all(lower <= upper);
            radius = min(predictedRadius, max(abs(lower), abs(upper)));
            if ~consistent
                radius(:) = NaN;
            end
        end

        function certificate = terminalRest(prediction)
        %stateUncertainty.terminalRest Verify a box of stationary poses.
        % The nominal endpoint has exact zero velocities. Under the unchanged rest
        % policy, a box with zero velocity radii is invariant if its propagated box
        % is contained in itself. No tolerance turns a nonzero velocity into rest.

            radius = prediction.egoStateErrorBound(:, end);
            nextRadius = abs(prediction.stageMatrixA(:, :, end))*radius ...
                + prediction.stageDisturbanceErrorBound(:, end);
            certificate = struct("kind", "stationary-pose-box-v1", ...
                "radius", radius, "nextRadius", nextRadius, ...
                "stationaryVelocities", all(radius(4:6) == 0), ...
                "disturbanceFree", all(prediction.stageDisturbanceErrorBound(:, end) == 0), ...
                "invariant", all(nextRadius <= radius), ...
                "zeroSpeedSchedule", prediction.scheduleSpeedProfile(end-1) == 0);
            certificate.accepted = certificate.stationaryVelocities ...
                && certificate.disturbanceFree && certificate.invariant && certificate.zeroSpeedSchedule;
        end
    end
end

function valid = localFiniteTime(value)
    valid = isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end
