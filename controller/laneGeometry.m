classdef laneGeometry
    %laneGeometry Nominal straight and circular lane coordinates.
    methods (Static)
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
            if isfield(curve,'curvatureProfile') && ~isempty(curve.curvatureProfile)
                error('collisionAvoidanceController:unsupportedReference','Use a straight or constant-curvature reference.');
            end
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

        function projection = projectReferenceCurve(position, curve, stationHint)
            if nargin<3,stationHint=[];end
            if curve.curvature == 0
                station = [cos(curve.heading),sin(curve.heading)]*(position-curve.origin);
            else
                center = curve.origin+[-sin(curve.heading);cos(curve.heading)]/curve.curvature;
                radial = position-center;
                heading = atan2(curve.curvature*radial(1,:),-curve.curvature*radial(2,:));
                middle = curve.heading+curve.curvature*curve.length/2;
                station = curve.length/2+atan2(sin(heading-middle),cos(heading-middle))/curve.curvature;
                if ~isempty(stationHint)
                    period=2*pi/abs(curve.curvature);
                    station=station+period*round((stationHint-station)/period);
                end
            end
            % The analytic reference continues beyond the display arc. On a
            % circle the optional station hint unwraps the projected coordinate.
            [point,heading] = laneGeometry.referencePose(station,0,curve);
            lateral = sum([-sin(heading);cos(heading)].*(position-point),1);
            projection = struct("point",point,"heading",heading, ...
                "station",station,"lateralPosition",lateral);
        end

        function projection = project(position, lane, stationHint)
            if nargin<3,stationHint=[];end
        % laneGeometry.project Project one point or columns of points onto a lane polyline.
        %
        % lane is the parsed centerline structure (segment starts, segments,
        % lengths, unit tangents, per-segment curvature and the cumulative
        % station of every segment start). Returns the closest polyline point
        % with the path heading there, the STATION s of the projection along
        % the centerline, and the signed left-positive lateral offset d - the
        % path coordinates (s, d) of the point.

            if isfield(lane, "referenceCurve")
                projection = laneGeometry.projectReferenceCurve(reshape(position,2,[]),lane.referenceCurve,stationHint);
                return;
            end
            if all(abs(lane.tangent-lane.tangent(1,:))<1e-12,'all')
                tangent=lane.tangent(1,:).';lateral=[-tangent(2);tangent(1)];
                station=tangent.'*(reshape(position,2,[])-lane.segmentStart(1,:).');
                point=lane.segmentStart(1,:).'+tangent*station;
                projection=struct('point',point,'heading',atan2(tangent(2),tangent(1))+zeros(size(station)), ...
                    'station',station,'lateralPosition',lateral.'*(reshape(position,2,[])-point));
                return;
            end
            position=reshape(position,2,[]);values=zeros(5,size(position,2));
            for pointIdx=1:size(position,2)
                values(:,pointIdx)=localProjection(position(:,pointIdx),lane);
            end
            projection = struct("point", values(1:2, :), "heading", values(3, :), ...
                "station", values(4, :), "lateralPosition", values(5, :));
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
