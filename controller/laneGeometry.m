classdef laneGeometry
    %laneGeometry Polyline projection, Frenet poses, curvature and chart bounds.

    methods (Static)
        function [frame,nominal] = sweptCellFrame(model,tube,anchor)
        % One frame construction for horizon admission and geometry rows.
            cfg = model.cfg;
            nominal = reshape(pagemtimes(tube.map,anchor),6,[])+tube.offset;
            radius = tube.numericalRadius;
            if tube.stage<=cfg.controller.certifiedSteps,radius = tube.radius;end
            station = (min(nominal(1,:))+max(nominal(1,:)))/2;
            extent = cfg.controller.stationTrustRadius ...
                +(max(nominal(1,:))-min(nominal(1,:)))/2+max(radius(1,:));
            frame = laneGeometry.frameBounds(model.lane,station,extent,cfg.model.lateralDomainRadius);
            frame.referenceHeadingErrorBound = frame.headingErrorBound;
            if tube.stage>cfg.controller.certifiedSteps
                tire = modifiedFialaTire.parameters(cfg);
                acceleration = sum(tire.longitudinalForceScale)/cfg.vehicle.m ...
                    +longitudinalRoadLoad(cfg.model.speedMaximum,cfg)/cfg.vehicle.m ...
                    +abs(model.longitudinalAccelerationBias);
                yawAcceleration = dot([cfg.vehicle.lf;cfg.vehicle.lr],tire.longitudinalForceScale)/cfg.vehicle.Iz;
                frame.positionErrorBound = frame.positionErrorBound+acceleration*tube.duration^2/8;
                frame.headingErrorBound = frame.headingErrorBound+yawAcceleration*tube.duration^2/8;
            end
        end
        function curve = validateReferenceCurve(curve)
            required = ["origin", "heading", "curvature", "length"];
            if ~isstruct(curve) || ~isscalar(curve) || ~all(isfield(curve, required))
                error("collisionAvoidanceController:invalidReferenceCurve", ...
                    "A reference curve needs origin, heading, curvature and length.");
            end
            validateattributes(curve.origin, {'double'}, {'real','finite','numel',2});
            validateattributes(curve.heading, {'double'}, {'real','finite','scalar'});
            validateattributes(curve.curvature, {'double'}, {'real','finite','scalar'});
            validateattributes(curve.length, {'double'}, {'real','finite','scalar','positive'});
            if abs(curve.curvature)*curve.length >= 2*pi
                error("collisionAvoidanceController:invalidReferenceCurve", ...
                    "The finite arc must have less than one complete revolution.");
            end
            curve.origin = curve.origin(:);
        end

        function [position, heading] = referencePose(station, lateral, curve)
            heading = curve.heading+curve.curvature*station;
            if curve.curvature == 0
                position = curve.origin+[cos(curve.heading);sin(curve.heading)]*station;
            else
                position = curve.origin+[sin(heading)-sin(curve.heading); ...
                    cos(curve.heading)-cos(heading)]/curve.curvature;
            end
            position = position+[-sin(heading);cos(heading)].*lateral;
        end

        function projection = projectReferenceCurve(position, curve)
            if curve.curvature == 0
                station = [cos(curve.heading),sin(curve.heading)]*(position-curve.origin);
            else
                center = curve.origin+[-sin(curve.heading);cos(curve.heading)]/curve.curvature;
                radial = position-center;
                heading = atan2(curve.curvature*radial(1,:),-curve.curvature*radial(2,:));
                middle = curve.heading+curve.curvature*curve.length/2;
                station = curve.length/2+atan2(sin(heading-middle),cos(heading-middle))/curve.curvature;
            end
            station = min(max(station,0),curve.length);
            [point,heading] = laneGeometry.referencePose(station,0,curve);
            lateral = sum([-sin(heading);cos(heading)].*(position-point),1);
            projection = struct("point",point,"heading",heading, ...
                "station",station,"lateralPosition",lateral);
        end

        function frame = referenceFrame(curve, station, radius, lateralRadius)
            if abs(curve.curvature)*lateralRadius >= 1
                error("collisionAvoidanceController:invalidReferenceCurve", ...
                    "The lateral strip must remain inside a regular Frenet chart.");
            end
            lower = max(0,station-radius);
            upper = min(curve.length,station+radius);
            [point,heading] = laneGeometry.referencePose(station,0,curve);
            tangent = [cos(heading);sin(heading)];
            lateral = [-sin(heading);cos(heading)];
            span = max(abs([lower,upper]-station));
            turn = abs(curve.curvature)*span;
            % Taylor's integral remainder for the centerline, plus rotation
            % of the lateral coordinate; each component is bounded by norm.
            deviation = abs(curve.curvature)*span^2/2+lateralRadius*min(2,turn);
            frame = struct("origin",point-tangent*station,"tangent",tangent, ...
                "lateral",lateral,"heading",heading,"segmentIndex",1, ...
                "stationLower",lower,"stationUpper",upper, ...
                "positionErrorBound",repmat(deviation,2,1),"headingErrorBound",turn);
        end

        function [radius, valid] = referenceUncertainty(state, inputRadius, curve)
            projection = laneGeometry.projectReferenceCurve(state(1:2),curve);
            distanceRadius = norm(inputRadius(1:2));
            radius = inputRadius;
            if curve.curvature == 0
                radius(1) = abs([cos(curve.heading),sin(curve.heading)])*inputRadius(1:2);
                radius(2) = abs([-sin(curve.heading),cos(curve.heading)])*inputRadius(1:2);
            else
                radialDistance = abs(1/curve.curvature-projection.lateralPosition);
                if distanceRadius >= radialDistance
                    valid = false;
                    radius(1:3) = inf;
                    return;
                end
                angle = asin(distanceRadius/radialDistance);
                radius(1) = angle/abs(curve.curvature);
                radius(2) = distanceRadius;
                radius(3) = inputRadius(3)+angle;
            end
            valid = projection.station-radius(1)>0 ...
                && projection.station+radius(1)<curve.length;
        end

        function projection = project(position, lane)
        % laneGeometry.project Project one point or columns of points onto a lane polyline.
        %
        % lane is the parsed centerline structure (segment starts, segments,
        % lengths, unit tangents, per-segment curvature and the cumulative
        % station of every segment start). Returns the closest polyline point
        % with the path heading there, the STATION s of the projection along
        % the centerline, and the signed left-positive lateral offset d - the
        % path coordinates (s, d) of the point.

            if isfield(lane, "referenceCurve")
                projection = laneGeometry.projectReferenceCurve(reshape(position,2,[]),lane.referenceCurve);
                return;
            end
            persistent nativeProjector
            if isempty(nativeProjector) && exist("projectLanePolylineMex", "file") == 3
                nativeProjector = @projectLanePolylineMex;
            end
            if isvector(position)
                position = position(:);
            end
            if ~isempty(nativeProjector)
                values = nativeProjector(position, lane.segmentStart, ...
                    lane.segment, lane.segmentLength, lane.segmentStation, lane.tangent);
            else
                values = zeros(5, size(position, 2));
                for pointIdx = 1:size(position, 2)
                    values(:, pointIdx) = localProjection(position(:, pointIdx), lane);
                end
            end
            projection = struct("point", values(1:2, :), "heading", values(3, :), ...
                "station", values(4, :), "lateralPosition", values(5, :));
        end

        function [position, heading] = fromFrenet(state, lane)
        %laneGeometry.fromFrenet Evaluate the physical polyline pose at each path state.
        % At an interior vertex the outgoing segment defines the coordinate chart.
        % Domain admission belongs to the caller; the end segments extrapolate.

            if isfield(lane, "referenceCurve")
                [position,heading] = laneGeometry.referencePose(state(1,:),state(2,:),lane.referenceCurve);
                heading = heading+state(3,:);
                return;
            end
            segments = discretize(state(1, :), [-inf; lane.segmentStation(2:end); inf]);
            tangent = lane.tangent(segments, :).';
            lateral = [-tangent(2, :); tangent(1, :)];
            position = lane.segmentStart(segments, :).'+tangent.* ...
                (state(1, :)-reshape(lane.segmentStation(segments), 1, [])) ...
                + lateral.*state(2, :);
            heading = atan2(tangent(2, :), tangent(1, :))+state(3, :);
        end

        function curvature = curvature(station, lane)
        % laneGeometry.curvature Centerline curvature at a station.
        %
        % The inverse of laneGeometry.project's station for the one quantity the
        % schedule needs: the curvature of the polyline segment containing
        % station s. Stations beyond either end take the end segment's.

            if isfield(lane, "referenceCurve")
                curvature = lane.referenceCurve.curvature+zeros(size(station));
                return;
            end
            segmentIdx = find(lane.segmentStation <= double(station), 1, "last");
            if isempty(segmentIdx)
                segmentIdx = 1;
            end
            curvature = lane.segmentCurvature(segmentIdx);
        end

        function frame = frameBounds(lane, station, radius, lateralRadius)
        %laneGeometry.frameBounds Bound a polyline's deviation from one affine chart.
        % The station interval can span several segments. On each segment the
        % position difference is affine in station and lateral offset, so extrema
        % over the admitted rectangle occur at its four corners. Heading variation
        % is constant per segment. These bounds include both sides of every vertex.

            if isfield(lane, "referenceCurve")
                frame = arrayfun(@(s) laneGeometry.referenceFrame( ...
                    lane.referenceCurve,s,radius,lateralRadius),station(:));
                return;
            end
            persistent nativeFrames
            if isempty(nativeFrames) && exist("laneFrameBoundsMex", "file") == 3
                nativeFrames = @laneFrameBoundsMex;
            end
            if ~isempty(nativeFrames)
                values = nativeFrames(station, radius, lateralRadius, lane.segmentStart, ...
                    lane.segmentLength, lane.segmentStation, lane.tangent);
                frame = struct("origin", num2cell(values(1:2, :), 1), ...
                    "tangent", num2cell(values(3:4, :), 1), ...
                    "lateral", num2cell([-values(4, :); values(3, :)], 1), ...
                    "heading", num2cell(values(5, :)), "segmentIndex", num2cell(values(6, :)), ...
                    "stationLower", num2cell(values(7, :)), "stationUpper", num2cell(values(8, :)), ...
                    "positionErrorBound", num2cell(values(9:10, :), 1), ...
                    "headingErrorBound", num2cell(values(11, :)));
                frame = frame(:);
            else
                frame = repmat(localFrame(lane, station(1), radius, lateralRadius), numel(station), 1);
                for nodeIdx = 2:numel(station)
                    frame(nodeIdx) = localFrame(lane, station(nodeIdx), radius, lateralRadius);
                end
            end
        end
    end
end

function values = localProjection(position, lane)
    offset = position(:).'-lane.segmentStart;
    fraction = sum(offset.*lane.segment, 2)./lane.segmentLength.^2;
    fraction = min(max(fraction, 0.0), 1.0);
    projectedPoint = lane.segmentStart+fraction.*lane.segment;
    delta = position(:).'-projectedPoint;
    distanceSquared = sum(delta.^2, 2);
    [~, segmentIdx] = min(distanceSquared);
    tangent = lane.tangent(segmentIdx, :);
    selectedDelta = delta(segmentIdx, :);
    values = [projectedPoint(segmentIdx, :).'; atan2(tangent(2), tangent(1)); ...
        lane.segmentStation(segmentIdx)+fraction(segmentIdx)*lane.segmentLength(segmentIdx); ...
        tangent(1)*selectedDelta(2)-tangent(2)*selectedDelta(1)];
end

function frame = localFrame(lane, station, radius, lateralRadius)
    segment = find(lane.segmentStation <= station, 1, "last");
    if isempty(segment)
        segment = 1;
    end
    tangent = lane.tangent(segment, :).';
    lateral = [-tangent(2); tangent(1)];
    origin = lane.segmentStart(segment, :).'-tangent*lane.segmentStation(segment);
    lower = max(lane.segmentStation(1), station-radius);
    upper = min(lane.segmentStation(end)+lane.segmentLength(end), station+radius);
    heading = atan2(tangent(2), tangent(1));
    active = find(lane.segmentStation <= upper ...
        & lane.segmentStation+lane.segmentLength >= lower);
    localTangent = lane.tangent(active, :).';
    localLateral = [-localTangent(2, :); localTangent(1, :)];
    starts = lane.segmentStation(active).';
    low = max(lower, starts);
    high = min(upper, starts+lane.segmentLength(active).');
    originError = lane.segmentStart(active, :).'-origin;
    lowError = originError+localTangent.*(low-starts)-tangent*low;
    highError = originError+localTangent.*(high-starts)-tangent*high;
    lateralError = (localLateral-lateral)*lateralRadius;
    positionError = max(abs([lowError+lateralError, lowError-lateralError, ...
        highError+lateralError, highError-lateralError, zeros(2, 1)]), [], 2);
    difference = atan2(localTangent(2, :), localTangent(1, :))-heading;
    headingError = max([0.0, abs(atan2(sin(difference), cos(difference)))]);
    frame = struct("origin", origin, "tangent", tangent, "lateral", lateral, ...
        "heading", heading, "segmentIndex", segment, ...
        "stationLower", lower, "stationUpper", upper, ...
        "positionErrorBound", positionError, "headingErrorBound", headingError);
end
