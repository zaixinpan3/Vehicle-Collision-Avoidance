function frame = laneFrameCertificate(lane, station, radius, lateralRadius)
%laneFrameCertificate Bound a polyline's deviation from one affine chart.
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
