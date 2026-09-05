function frame = laneFrameCertificate(lane, station, radius, lateralRadius)
%laneFrameCertificate Bound a polyline's deviation from one affine chart.
% The station interval can span several segments. On each segment the
% position difference is affine in station and lateral offset, so extrema
% over the admitted rectangle occur at its four corners. Heading variation
% is constant per segment. These bounds include both sides of every vertex.

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
    positionError = zeros(2, 1);
    headingError = 0.0;
    for idx = active(:).'
        localTangent = lane.tangent(idx, :).';
        localLateral = [-localTangent(2); localTangent(1)];
        endpoints = [max(lower, lane.segmentStation(idx)), ...
            min(upper, lane.segmentStation(idx)+lane.segmentLength(idx))];
        centreError = lane.segmentStart(idx, :).'-origin ...
            + localTangent*(endpoints-lane.segmentStation(idx)) ...
            - tangent*endpoints;
        lateralError = (localLateral-lateral)*lateralRadius;
        positionError = max(positionError, ...
            max(abs([centreError+lateralError, centreError-lateralError]), [], 2));
        difference = atan2(localTangent(2), localTangent(1))-heading;
        headingError = max(headingError, abs(atan2(sin(difference), cos(difference))));
    end
    frame = struct("origin", origin, "tangent", tangent, "lateral", lateral, ...
        "heading", heading, "segmentIndex", segment, ...
        "stationLower", lower, "stationUpper", upper, ...
        "positionErrorBound", positionError, "headingErrorBound", headingError);
end
