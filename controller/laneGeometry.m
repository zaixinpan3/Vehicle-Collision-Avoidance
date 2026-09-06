classdef laneGeometry
    %laneGeometry Polyline projection, Frenet poses, curvature and chart bounds.

    methods (Static)
        function projection = project(position, lane)
        % laneGeometry.project Project one point or columns of points onto a lane polyline.
        %
        % lane is the parsed centerline structure (segment starts, segments,
        % lengths, unit tangents, per-segment curvature and the cumulative
        % station of every segment start). Returns the closest polyline point
        % with the path heading there, the STATION s of the projection along
        % the centerline, and the signed left-positive lateral offset d - the
        % path coordinates (s, d) of the point.

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
