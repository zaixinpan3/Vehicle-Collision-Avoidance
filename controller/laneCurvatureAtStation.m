function curvature = laneCurvatureAtStation(station, lane)
% laneCurvatureAtStation Centerline curvature at a station.
%
% The inverse of laneProjection's station for the one quantity the
% schedule needs: the curvature of the polyline segment containing
% station s. Stations beyond either end take the end segment's.

    segmentIdx = find(lane.segmentStation <= double(station), 1, "last");
    if isempty(segmentIdx)
        segmentIdx = 1;
    end
    curvature = lane.segmentCurvature(segmentIdx);
end
