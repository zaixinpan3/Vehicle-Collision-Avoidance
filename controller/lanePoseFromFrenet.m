function [position, heading] = lanePoseFromFrenet(state, lane)
%lanePoseFromFrenet Evaluate the physical polyline pose at each path state.
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
