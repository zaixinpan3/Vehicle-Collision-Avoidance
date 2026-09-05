function projection = laneProjection(position, lane)
% laneProjection Project one point or columns of points onto a lane polyline.
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
