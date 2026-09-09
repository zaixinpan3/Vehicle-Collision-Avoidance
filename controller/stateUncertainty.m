classdef stateUncertainty
    %stateUncertainty Estimator certificates and state-box propagation, intersection and rest.

    methods (Static)
        function tubes = heldInterval(a,b,c,map,offset,radius,rate,duration,order,stateLimit,inputLimit,numericalRadius,cellCount)
        %heldInterval Share one complete held-interval enclosure algorithm.
            step = duration/cellCount;
            first = stateUncertainty.flowTube(a,b,c,map,offset,radius,rate,step,order, ...
                stateLimit,inputLimit,numericalRadius);
            tubes = repmat(first,cellCount,1);
            for index = 2:cellCount
                previous = tubes(index-1);
                tubes(index) = stateUncertainty.flowTube(a,b,c,previous.endMap, ...
                    previous.endOffset,previous.endRadius,rate,step,order, ...
                    stateLimit,inputLimit,previous.endNumericalRadius);
            end
        end

        function tube = flowTube(a, b, c, map, offset, radius, rate, duration, order, stateLimit, inputLimit, numericalRadius)
        %flowTube Taylor/Bernstein enclosure of a complete held-input cell.
        % The polynomial is affine in the state and held input. Its remainder
        % uses a scalar exponential-series majorant. The initial nominal
        % state and input must satisfy the supplied absolute domain bounds.
            sizeState = size(a, 1);
            if nargin<12, numericalRadius = zeros(sizeState,1); end
            degree = order+1;
            gain = norm(a, inf)*duration;
            if gain >= 1
                error("collisionAvoidanceController:invalidCertificationCell", ...
                    "Certification cells require norm(A,inf)*duration < 1.");
            end
            columnCount = sizeState+size(b, 2)+1;
            polynomial = zeros(sizeState, columnCount, degree+1);
            polynomial(:, :, 1) = [eye(sizeState), zeros(sizeState, size(b, 2)+1)];
            power = [a, b, c];
            for powerIndex = 1:order
                polynomial(:, :, powerIndex+1) = power;
                power = a*power/(powerIndex+1);
            end
            radiusPolynomial = zeros(sizeState, degree+1);
            radiusPolynomial(:, 1) = radius;
            numericalPolynomial = zeros(sizeState,degree+1);
            numericalPolynomial(:,1) = numericalRadius;
            powerA = eye(sizeState);
            for powerIndex = 1:order
                radiusPolynomial(:, powerIndex+1) = ...
                    abs(powerA*a)*radius/factorial(powerIndex) ...
                    + abs(powerA)*rate/factorial(powerIndex);
                numericalPolynomial(:,powerIndex+1) = abs(powerA*a)*numericalRadius/factorial(powerIndex);
                powerA = powerA*a;
            end
            driftBound = abs(a)*stateLimit+abs(b)*inputLimit+abs(c);
            errorDrift = abs(a)*radius+rate;
            tailWeight = abs(a)^order*ones(sizeState, 1) ...
                /factorial(degree)/(1-gain);
            radiusPolynomial(:, end) = tailWeight*(max(driftBound)+max(errorDrift));
            numericalPolynomial(:,end) = tailWeight*(max(driftBound)+max(abs(a)*numericalRadius));
            % Arithmetic allowance, charged to the enclosures rather than to
            % a post-solve feasibility tolerance. At the copied initial point
            % no polynomial arithmetic has taken place.
            operations = size(map, 2)+sizeState*order+degree^2;
            gamma = operations*eps/(1-operations*eps);
            coefficientMagnitude = abs(map)*inputLimit+abs(offset);
            arithmetic = 16*gamma*(1+abs(a)*coefficientMagnitude ...
                +abs(b)*inputLimit+abs(c))/(1-gain);
            radiusPolynomial(:, 2) = radiusPolynomial(:, 2)+arithmetic;
            numericalPolynomial(:,2) = numericalPolynomial(:,2)+arithmetic;
            transform = stateUncertainty.bernsteinTransform(degree, duration);
            controls = reshape(reshape(polynomial, [], degree+1)*transform.', ...
                sizeState, columnCount, degree+1);
            tubeMap = pagemtimes(controls(:, 1:sizeState, :), map);
            for index = 1:degree+1
                tubeMap(:, :, index) = tubeMap(:, :, index)+controls(:, sizeState+(1:size(b, 2)), index);
            end
            % b contains the condensed held-input columns, so controls already
            % use the same decision coordinates as map.
            tubeOffset = reshape(pagemtimes(controls(:, 1:sizeState, :), offset), sizeState, []) ...
                + reshape(controls(:, end, :), sizeState, []);
            tubeRadius = radiusPolynomial*transform.';
            tubeNumericalRadius = numericalPolynomial*transform.';
            tubeLocalStateMap = controls(:,1:sizeState,:);
            tubeLocalInputMap = controls(:,sizeState+(1:size(b,2)),:);
            tubeLocalOffset = reshape(controls(:,end,:),sizeState,[]);
            transition = controls(:, 1:sizeState, end);
            tubeEndMap = tubeMap(:, :, end);
            tubeEndOffset = tubeOffset(:, end);
            % Retain cancellation in the endpoint transition; the swept tube
            % uses absolute power bounds, while the next cell uses |Phi(h)|.
            process = radiusPolynomial;
            process(:, 1) = 0;
            numericalProcess = numericalPolynomial;
            numericalProcess(:,1) = 0;
            powerA = eye(sizeState);
            for powerIndex = 1:order
                process(:, powerIndex+1) = process(:, powerIndex+1) ...
                    - abs(powerA*a)*radius/factorial(powerIndex);
                numericalProcess(:,powerIndex+1) = numericalProcess(:,powerIndex+1) ...
                    -abs(powerA*a)*numericalRadius/factorial(powerIndex);
                powerA = powerA*a;
            end
            tubeEndRadius = abs(transition)*radius+max(0, process*transform(end, :).');
            tubeEndNumericalRadius = abs(transition)*numericalRadius+max(0,numericalProcess*transform(end,:).');
            tube = struct("map",tubeMap,"offset",tubeOffset,"radius",tubeRadius, ...
                "numericalRadius",tubeNumericalRadius,"localStateMap",tubeLocalStateMap, ...
                "localInputMap",tubeLocalInputMap,"localOffset",tubeLocalOffset, ...
                "endMap",tubeEndMap,"endOffset",tubeEndOffset,"endRadius",tubeEndRadius, ...
                "endNumericalRadius",tubeEndNumericalRadius);
        end

        function transform = bernsteinTransform(degree, duration)
        %bernsteinTransform Power coefficients to Bernstein control points.
            persistent priorDegree priorNormalized
            if coder.target('MATLAB')
                if isequal(degree,priorDegree)
                    transform = priorNormalized.*duration.^(0:degree);
                    return;
                end
            end
            normalized = zeros(degree+1);
            normalized(:,1) = 1;
            for row = 1:degree
                for power = 1:row
                    normalized(row+1,power+1) = normalized(row+1,power) ...
                        *(row-power+1)/(degree-power+1);
                end
            end
            if coder.target('MATLAB'),priorDegree = degree;priorNormalized = normalized;end
            transform = normalized.*duration.^(0:degree);
        end

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
            if isfield(lane, "referenceCurve")
                [radius,chartValid] = laneGeometry.referenceUncertainty( ...
                    cartesianState,cartesianRadius,lane.referenceCurve);
                return;
            end
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
            % Consecutive collinear segments form one invertible chart even
            % when a finely sampled straight centerline has internal vertices.
            active = find(possible);
            straight = all(diff(active) == 1) ...
                && all(abs(tangents-tangents(1, :)) <= 32*eps, "all");
            if straight
                firstAlong = along(active(1));
                span = sum(lengths);
                chartValid = firstAlong-alongRadius(1) > 0 ...
                    && firstAlong+alongRadius(1) < span;
            end
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

    end
end

function valid = localFiniteTime(value)
    valid = isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end
