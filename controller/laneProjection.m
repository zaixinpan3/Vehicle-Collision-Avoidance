function projection = laneProjection(position, lane)
% laneProjection Project a point onto the parsed lane centerline polyline.
%
% lane is the parsed centerline structure (segment starts, segments,
% lengths, unit tangents, per-segment curvature and the cumulative
% station of every segment start). Returns the closest polyline point
% with the path heading there, the STATION s of the projection along
% the centerline, and the signed left-positive lateral offset d - the
% path coordinates (s, d) of the point.

    offset = position(:).'-lane.segmentStart;
    fraction = sum(offset.*lane.segment, 2)./lane.segmentLength.^2;
    fraction = min(max(fraction, 0.0), 1.0);
    projectedPoint = lane.segmentStart+fraction.*lane.segment;
    delta = position(:).'-projectedPoint;
    distanceSquared = sum(delta.^2, 2);
    [~, segmentIdx] = min(distanceSquared);
    tangent = lane.tangent(segmentIdx, :);
    selectedDelta = delta(segmentIdx, :);
    projection = struct();
    projection.point = projectedPoint(segmentIdx, :).';
    projection.heading = atan2(tangent(2), tangent(1));
    projection.station = lane.segmentStation(segmentIdx) ...
        + fraction(segmentIdx)*lane.segmentLength(segmentIdx);
    projection.lateralPosition = tangent(1)*selectedDelta(2) ...
        - tangent(2)*selectedDelta(1);
end
