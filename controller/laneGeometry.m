classdef laneGeometry
    %laneGeometry Smooth-reference projection, Frenet poses and certified charts.

    methods (Static)
        function chart = normalRoadChart(station,lane,road)
        % Intersect route normals with finite quadratic boundaries. The chosen
        % component contains d=0. This is a selected corridor, not a road union.
        % Unbounded sections use a unit lateral scale and keep all hard rows.
            station=reshape(station,1,[]);count=numel(station);
            [position,heading]=laneGeometry.fromFrenet([station;zeros(5,count)],lane);
            curvature=zeros(1,count);derivative=curvature;
            if isfield(lane,'referenceCurve')
                [curvature,derivative]=laneGeometry.referenceCurvature(station,lane.referenceCurve);
            end
            tangent=[cos(heading);sin(heading)];normal=[-sin(heading);cos(heading)];
            midpoint=zeros(3,count);halfWidth=[ones(1,count);zeros(2,count)];
            bounded=false(1,count);valid=true(1,count);
            for node=1:count
                lower=[-Inf;0;0];upper=[Inf;0;0];
                for index=1:numel(road.boundaries)
                    boundary=road.boundaries(index);a=boundary.coefficients(1);
                    b=boundary.coefficients(2);c=boundary.coefficients(3);
                    longitudinal=boundary.longitudinalDirection; lateral=boundary.lateralDirection;
                    offset=position(:,node)-boundary.origin;
                    x=longitudinal.'*offset;y=lateral.'*offset;
                    nx=longitudinal.'*normal(:,node);ny=lateral.'*normal(:,node);
                    polynomial=[-a*nx^2,ny-(2*a*x+b)*nx,y-a*x^2-b*x-c];
                    tolerance=128*eps*(1+norm(polynomial));
                    if x>=boundary.parameterRange(1) && x<=boundary.parameterRange(2) ...
                            && boundary.safeSideSign*polynomial(3)<-tolerance
                        valid(node)=false;
                    end
                    roots=localSectionRoots(polynomial);
                    for rootIndex=1:numel(roots)
                        d=roots(rootIndex);parameter=x+nx*d;
                        if parameter<boundary.parameterRange(1) || parameter>boundary.parameterRange(2),continue;end
                        gradient=lateral-(2*a*parameter+b)*longitudinal;
                        denominator=gradient.'*normal(:,node);
                        % A tangency does not delimit an open component.
                        if abs(denominator)<=tolerance,continue;end
                        k=curvature(node);kp=derivative(node);t=tangent(:,node);n=normal(:,node);
                        first=-(gradient.'*t)*(1-k*d)/denominator;
                        velocity=(1-k*d)*t+first*n;
                        second=-(-2*a*(longitudinal.'*velocity)^2 ...
                            +gradient.'*((-kp*d-2*k*first)*t+k*(1-k*d)*n))/denominator;
                        value=[d;first;second];
                        if boundary.safeSideSign*denominator>0 && d<=0 && d>lower(1),lower=value;end
                        if boundary.safeSideSign*denominator<0 && d>=0 && d<upper(1),upper=value;end
                    end
                end
                bounded(node)=isfinite(lower(1)) && isfinite(upper(1));
                if bounded(node)
                    midpoint(:,node)=(upper+lower)/2;halfWidth(:,node)=(upper-lower)/2;
                    valid(node)=valid(node) && halfWidth(1,node)>0 ...
                        && all(1-curvature(node)*[lower(1),upper(1)]>0);
                end
            end
            chart=struct('position',position,'heading',heading,'tangent',tangent,'normal',normal, ...
                'curvature',curvature,'curvatureDerivative',derivative, ...
                'midpoint',midpoint,'halfWidth',halfWidth,'bounded',bounded,'valid',valid);
        end

        function [frame,nominal] = sweptCellFrame(model,tube,anchor)
        % One frame construction for horizon admission and geometry rows.
            [frame,values] = laneGeometry.sweptCellFrames(model,tube,anchor);
            nominal = values{1};
        end

        function [frames,nominal] = sweptCellFrames(model,tubes,anchor)
        % Batch charts with individual station radii over one validated lane.
            cfg = model.cfg;
            nominal = cell(numel(tubes),1);
            curved = isfield(model.lane,'referenceCurve') && (model.lane.referenceCurve.curvature~=0 ...
                || laneGeometry.isVaryingReference(model.lane)) ...
                && ~isempty(model.encounters);
            if curved
                centers=zeros(3,numel(tubes));radii=centers;
                for index=1:numel(tubes)
                    tube=tubes(index);
                    values=reshape(pagemtimes(tube.map,anchor),6,[])+tube.offset;
                    nominal{index}=values;
                    lower=min(values(1:3,:)-tube.radius(1:3,:),[],2);
                    upper=max(values(1:3,:)+tube.radius(1:3,:),[],2);
                    centers(:,index)=(lower+upper)/2;
                    radii(:,index)=(upper-lower)/2+cfg.controller.poseTrustRadius;
                end
                frames=laneGeometry.localPoseFrames(model.lane.referenceCurve,centers,radii);
                return;
            end
            station = zeros(numel(tubes),1);extent = station;lateralExtent = station;
            for index = 1:numel(tubes)
                tube = tubes(index);
                values = reshape(pagemtimes(tube.map,anchor),6,[])+tube.offset;
                nominal{index} = values;
                radius = tube.radius;
                bound = repmat([cfg.model.frontWheelSteeringAngleMaximum; ...
                    max(abs([cfg.actuation.brakingRatioMinimum,cfg.actuation.brakingRatioMaximum]))], ...
                    size(tube.map,2)/2,1);
                support = reshape(pagemtimes(abs(tube.map),bound),6,[]);
                lower = min(tube.offset(1,:)-support(1,:)-radius(1,:));
                upper = max(tube.offset(1,:)+support(1,:)+radius(1,:));
                allowance = max(128*eps,cfg.encounter.numericalMargin)*(1+max(abs([lower,upper])));
                station(index) = (lower+upper)/2;
                extent(index) = (upper-lower)/2+allowance;
                lateralExtent(index) = max(abs(tube.offset(2,:))+support(2,:)+radius(2,:));
            end
            frames = laneGeometry.frameBounds(model.lane,station,extent,max(lateralExtent));
            for index = 1:numel(tubes)
                frames(index).referenceHeadingErrorBound = frames(index).headingErrorBound;
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
            if isfield(curve,'curvatureProfile') && ~isempty(curve.curvatureProfile)
                profile=curve.curvatureProfile;
                validateattributes(profile,{'double'},{'2d','ncols',2,'finite','real'});
                if size(profile,1)<2 || profile(1,1)~=0 || profile(end,1)~=curve.length ...
                        || any(diff(profile(:,1))<=0)
                    error("collisionAvoidanceController:invalidReferenceCurve", ...
                        "The curvature profile must have increasing stations from zero through length.");
                end
                if ~isfield(curve,'continuation') || ~isscalar(string(curve.continuation)) ...
                        || string(curve.continuation)~="constantCurvature"
                    error("collisionAvoidanceController:invalidReferenceCurve", ...
                        "A curvature profile requires explicit constantCurvature continuation.");
                end
                curve.curvature=profile(1,2);
                curve.origin=curve.origin(:);
                localProfileData(curve);
                return;
            end
            if abs(curve.curvature)*curve.length >= 2*pi
                error("collisionAvoidanceController:invalidReferenceCurve", ...
                    "The finite arc must have less than one complete revolution.");
            end
            curve.origin = curve.origin(:);
        end

        function frame = localPoseFrame(curve,center,radius)
        % A first-order pose map certified on a hard box in [s,d,e_psi].
            validateattributes(center,{'double'},{'size',[3,1],'finite','real'});
            validateattributes(radius,{'double'},{'size',[3,1],'finite','real','nonnegative'});
            frame=laneGeometry.localPoseFrames(curve,center,radius);
        end

        function frames = localPoseFrames(curve,centers,radii)
        % The localPoseFrame of every column of centers and radii. The
        % reference pose, curvature and chart error of all boxes are evaluated
        % in one batch; each frame equals its single-column evaluation.
            validateattributes(centers,{'double'},{'nrows',3,'finite','real'});
            validateattributes(radii,{'double'},{'size',size(centers),'finite','real','nonnegative'});
            count=size(centers,2);
            [positions,headings]=laneGeometry.referencePose(centers(1,:),centers(2,:),curve);
            [positionErrors,headingErrors]=laneGeometry.referenceErrorBound(centers(1,:),curve);
            curvatures=laneGeometry.referenceCurvature(centers(1,:),curve);
            profile=isfield(curve,'curvatureProfile');
            cells=cell(count,1);
            for index=1:count
                center=centers(:,index);radius=radii(:,index);
                position=positions(:,index);heading=headings(index);
                positionError=positionErrors(index);headingError=headingErrors(index);
                tangent=[cos(heading);sin(heading)];lateral=[-tangent(2);tangent(1)];
                k=curvatures(index);
                [k0,k1]=laneGeometry.referenceCurvatureBounds(curve,center(1)-radius(1),center(1)+radius(1));
                jacobian=[(1-k*center(2))*tangent,lateral,zeros(2,4)];
                offset=position-jacobian(:,1:3)*center;
                yawRow=[k,0,1,0,0,0];yawOffset=heading-k*center(1);
                extent=abs(center(2))+radius(2);
                remainder=(k1*extent+k0*(1+k0*extent))*radius(1)^2/2 ...
                    +k0*radius(1)*radius(2)+positionError+extent*headingError;
                yawRemainder=k1*radius(1)^2/2+headingError;
                if profile && k0*extent>=1
                    error("collisionAvoidanceController:singularReferenceDomain", ...
                        "The complete local pose domain must satisfy one minus absolute curvature times lateral extent greater than zero.");
                end
                remainder=remainder+128*eps*(1+norm(position)+norm(offset)+norm(jacobian,'fro')*norm(abs(center)+radius));
                cells{index}=struct('origin',position-tangent*center(1)-lateral*center(2), ...
                    'tangent',tangent,'lateral',lateral,'heading',heading,'segmentIndex',1, ...
                    'stationLower',center(1)-radius(1),'stationUpper',center(1)+radius(1), ...
                    'positionErrorBound',repmat(remainder,2,1),'headingErrorBound',yawRemainder, ...
                    'referenceHeadingErrorBound',yawRemainder,'positionMap',jacobian,'positionOffset',offset, ...
                    'yawRow',yawRow,'yawOffset',yawOffset,'positionRemainder',remainder, ...
                    'domainCenter',center,'domainRadius',radius);
            end
            frames=vertcat(cells{:});
        end

        function [pose,domain] = poseData(frame)
        % Shared numeric format; geometric tangent remains a unit direction.
            if isfield(frame,'positionMap')
                pose=[frame.positionOffset;frame.positionMap(:);frame.yawOffset;frame.yawRow.';frame.positionRemainder];
                domain=[1;frame.domainCenter;frame.domainRadius];
            else
                jacobian=[frame.tangent,frame.lateral,zeros(2,4)];
                pose=[frame.origin;jacobian(:);frame.heading;0;0;1;0;0;0;0];
                domain=zeros(7,1);
            end
        end

        function [position, heading, positionError] = referencePose(station, lateral, curve)
            if isfield(curve,'curvatureProfile') && ~isempty(curve.curvatureProfile)
                [position,heading]=localProfilePose(station,lateral,curve);
                if nargout>2,positionError=laneGeometry.referenceErrorBound(station,curve);end
                return;
            end
            positionError=zeros(size(station));
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
            if isfield(curve,'curvatureProfile') && ~isempty(curve.curvatureProfile)
                projection=localProfileProjection(position,curve,stationHint);
                return;
            end
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
            % circle prepare() unwraps this coordinate about the carried node.
            [point,heading] = laneGeometry.referencePose(station,0,curve);
            lateral = sum([-sin(heading);cos(heading)].*(position-point),1);
            projection = struct("point",point,"heading",heading, ...
                "station",station,"lateralPosition",lateral);
        end

        function frame = referenceFrame(curve, station, radius, lateralRadius)
            % The forward Frenet map is defined for every finite lateral offset.
            % Invertibility of a measurement chart is checked at observation time.
            lower = station-radius;
            upper = station+radius;
            [point,heading] = laneGeometry.referencePose(station,0,curve);
            tangent = [cos(heading);sin(heading)];
            lateral = [-sin(heading);cos(heading)];
            span = max(abs([lower,upper]-station));
            [k0,~]=laneGeometry.referenceCurvatureBounds(curve,lower,upper);
            [positionError,headingError]=laneGeometry.referenceErrorBound(station,curve);
            turn = k0*span+headingError;
            % Taylor's integral remainder for the centerline, plus rotation
            % of the lateral coordinate; each component is bounded by norm.
            deviation = k0*span^2/2+lateralRadius*min(2,turn)+positionError;
            frame = struct("origin",point-tangent*station,"tangent",tangent, ...
                "lateral",lateral,"heading",heading,"segmentIndex",1, ...
                "stationLower",lower,"stationUpper",upper, ...
                "positionErrorBound",repmat(deviation,2,1),"headingErrorBound",turn);
        end

        function [radius, valid] = referenceUncertainty(state, inputRadius, curve)
            distanceRadius = norm(inputRadius(1:2));
            radius = inputRadius;
            if isfield(curve,'curvatureProfile') && ~isempty(curve.curvatureProfile)
                valid=distanceRadius==0;
                if ~valid,radius(1:3)=inf;end
                return;
            end
            projection = laneGeometry.projectReferenceCurve(state(1:2),curve);
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
            valid = all(isfinite(radius));
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
                curvature = laneGeometry.referenceCurvature(station,lane.referenceCurve);
                return;
            end
            segmentIdx = find(lane.segmentStation <= double(station), 1, "last");
            if isempty(segmentIdx)
                segmentIdx = 1;
            end
            curvature = lane.segmentCurvature(segmentIdx);
        end

        function varying = isVaryingReference(lane)
            varying=isfield(lane,'referenceCurve') ...
                && isfield(lane.referenceCurve,'curvatureProfile') ...
                && ~isempty(lane.referenceCurve.curvatureProfile) ...
                && any(lane.referenceCurve.curvatureProfile(:,2)~=lane.referenceCurve.curvatureProfile(1,2));
        end

        function [curvature,derivative] = referenceCurvature(station,curve)
            curvature=curve.curvature+zeros(size(station));derivative=zeros(size(station));
            if ~isfield(curve,'curvatureProfile') || isempty(curve.curvatureProfile),return;end
            data=localProfileData(curve);
            clipped=min(max(station,0),curve.length);
            curvature=ppval(data.curvature,clipped);
            curvature(station<=0)=curve.curvatureProfile(1,2);
            curvature(station>=curve.length)=curve.curvatureProfile(end,2);
            derivative=ppval(data.curvatureDerivative,clipped);
            derivative(station<0 | station>curve.length)=0;
        end

        function [maximum,derivativeMaximum] = referenceCurvatureBounds(curve,lower,upper)
            maximum=abs(curve.curvature);derivativeMaximum=0;
            if ~isfield(curve,'curvatureProfile') || isempty(curve.curvatureProfile),return;end
            data=localProfileData(curve);
            maximum=localPolynomialBound(data.curvature,lower,upper);
            derivativeMaximum=localPolynomialBound(data.curvatureDerivative,max(0,lower),min(curve.length,upper));
            maximum=max(maximum,max(abs(laneGeometry.referenceCurvature([lower,upper],curve))));
        end

        function [positionError,headingError] = referenceErrorBound(station,curve)
            positionError=zeros(size(station));headingError=positionError;
            if ~isfield(curve,'curvatureProfile') || isempty(curve.curvatureProfile),return;end
            data=localProfileData(curve);
            index=discretize(min(max(station,0),curve.length),[-inf,data.position.breaks(2:end-1),inf]);
            distance=max(-station,0)+max(station-curve.length,0);
            headingError=data.headingError+128*eps*(1+abs(station).*abs(laneGeometry.referenceCurvature(station,curve)));
            positionError=reshape(data.positionError(index),size(station))+distance.*headingError ...
                +128*eps*(1+abs(station)+norm(curve.origin));
        end

        function frame = frameBounds(lane, station, radius, lateralRadius)
        %laneGeometry.frameBounds Bound a polyline's deviation from one affine chart.
        % The station interval can span several segments. On each segment the
        % position difference is affine in station and lateral offset, so extrema
        % over the admitted rectangle occur at its four corners. Heading variation
        % is constant per segment. These bounds include both sides of every vertex.

            if isscalar(radius),radius = repmat(radius,numel(station),1);end
            if ~isfield(lane,'referenceCurve') && all(abs(lane.tangent-lane.tangent(1,:))<1e-12,'all')
                tangent=lane.tangent(1,:).';lateral=[-tangent(2);tangent(1)];
                frame=arrayfun(@(s,r) struct('origin',lane.segmentStart(1,:).','tangent',tangent, ...
                    'lateral',lateral,'heading',atan2(tangent(2),tangent(1)), ...
                    'segmentIndex',1,'stationLower',s-r,'stationUpper',s+r, ...
                    'positionErrorBound',zeros(2,1),'headingErrorBound',0),station(:),radius(:));
                return;
            end
            if isfield(lane, "referenceCurve")
                frame = arrayfun(@(s,r) laneGeometry.referenceFrame( ...
                    lane.referenceCurve,s,r,lateralRadius),station(:),radius(:));
                return;
            end
            persistent nativeFrames
            if isempty(nativeFrames) && exist("laneFrameBoundsMex", "file") == 3
                nativeFrames = @laneFrameBoundsMex;
            end
            if ~isempty(nativeFrames)
                values = nativeFrames(station, radius, lateralRadius, lane.segmentStart, ...
                    lane.segmentLength, lane.segmentStation, lane.tangent);
                if size(values,2) == 1
                    frame = struct("origin",values(1:2),"tangent",values(3:4), ...
                        "lateral",[-values(4);values(3)],"heading",values(5), ...
                        "segmentIndex",values(6),"stationLower",values(7), ...
                        "stationUpper",values(8),"positionErrorBound",values(9:10), ...
                        "headingErrorBound",values(11));
                    return;
                end
                frame = struct("origin", num2cell(values(1:2, :), 1), ...
                    "tangent", num2cell(values(3:4, :), 1), ...
                    "lateral", num2cell([-values(4, :); values(3, :)], 1), ...
                    "heading", num2cell(values(5, :)), "segmentIndex", num2cell(values(6, :)), ...
                    "stationLower", num2cell(values(7, :)), "stationUpper", num2cell(values(8, :)), ...
                    "positionErrorBound", num2cell(values(9:10, :), 1), ...
                    "headingErrorBound", num2cell(values(11, :)));
                frame = frame(:);
            else
                frame = repmat(localFrame(lane, station(1), radius(1), lateralRadius), numel(station), 1);
                for nodeIdx = 2:numel(station)
                    frame(nodeIdx) = localFrame(lane, station(nodeIdx), radius(nodeIdx), lateralRadius);
                end
            end
        end
    end
end

function values=localSectionRoots(polynomial)
    a=polynomial(1);b=polynomial(2);c=polynomial(3);values=zeros(1,0);
    if a==0
        if b~=0,values=-c/b;end
        return;
    end
    discriminant=b*b-4*a*c;
    if discriminant<0,return;end
    direction=1;if b<0,direction=-1;end
    q=-.5*(b+direction*sqrt(discriminant));
    if q==0,values=-b/(2*a);else,values=[q/a,c/q];end
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

function data=localProfileData(curve)
% The declared PCHIP curvature defines an arc-length parameterized path.
% Position integrates exp(i*heading) by a polynomial exponential series.
% Every cell carries the analytic exponential-series remainder and a
% conservative floating-point allowance; quadrature tolerance is not proof.
    persistent identity cached
    key={curve.origin(:),curve.heading,curve.length,curve.curvatureProfile};
    if isequaln(key,identity),data=cached;return;end
    profile=curve.curvatureProfile;
    curvature=pchip(profile(:,1),profile(:,2));
    derivative=mkpp(curvature.breaks,curvature.coefs(:,1:3).*[3,2,1]);
    headingCoefficients=zeros(curvature.pieces,5);heading=curve.heading;headingScale=abs(heading);
    for index=1:curvature.pieces
        row=[curvature.coefs(index,:)./[4,3,2,1],heading];
        headingCoefficients(index,:)=row;
        width=diff(curvature.breaks(index:index+1));
        heading=polyval(row,width);
        headingScale=headingScale+sum(abs(row(1:4)).*width.^(4:-1:1));
    end
    headingPolynomial=mkpp(curvature.breaks,headingCoefficients);
    headingError=4096*eps*(1+headingScale);
    breaks=0;coefficients=zeros(0,50);errors=zeros(1,0);
    point=complex(curve.origin(1),curve.origin(2));accumulatedError=0;
    for index=1:curvature.pieces
        start=curvature.breaks(index);last=curvature.breaks(index+1);
        polynomial=curvature.coefs(index,:);
        while start<last
            width=min(2,last-start);offset=start-curvature.breaks(index);
            jet=[polyval(polynomial,offset),polyval(polyder(polynomial),offset), ...
                polyval(polyder(polyder(polynomial)),offset),6*polynomial(1)];
            angle=[0,jet.*width.^(1:4)./[1,2,6,24]];
            while sum(abs(angle))>.25
                width=width/2;
                if start+width==start
                    error("collisionAvoidanceController:invalidReferenceCurve", ...
                        "The curvature profile cannot be resolved at floating-point station precision.");
                end
                angle=[0,jet.*width.^(1:4)./[1,2,6,24]];
            end
            total=zeros(1,49);term=1;total(1)=1;
            for order=1:12
                term=conv(term,1i*angle)/order;
                total(1:numel(term))=total(1:numel(term))+term;
            end
            phase=ppval(headingPolynomial,start);
            integral=width*exp(1i*phase)*total./(1:49);
            normalized=[point,integral];
            physical=normalized./width.^(0:49);
            if any(~isfinite(physical))
                error("collisionAvoidanceController:invalidReferenceCurve", ...
                    "The curvature profile has numerically unresolved integration cells.");
            end
            coefficients(end+1,:)=real(fliplr(physical)); %#ok<AGROW>
            coefficients(end+1,:)=imag(fliplr(physical)); %#ok<AGROW>
            remainder=width*exp(sum(abs(angle)))*sum(abs(angle))^13/factorial(13);
            roundoff=4096*eps*(1+abs(point)+sum(abs(integral)));
            accumulatedError=accumulatedError+remainder+roundoff+width*headingError;
            errors(end+1)=accumulatedError; %#ok<AGROW>
            point=sum(normalized);start=start+width;breaks(end+1)=start; %#ok<AGROW>
        end
    end
    position=mkpp(breaks,coefficients,2);
    data=struct('curvature',curvature,'curvatureDerivative',derivative, ...
        'heading',headingPolynomial,'headingError',headingError, ...
        'position',position,'positionError',errors);
    identity=key;cached=data;
end

function [position,heading]=localProfilePose(station,lateral,curve)
    data=localProfileData(curve);shape=size(station);station=station(:).';
    clipped=min(max(station,0),curve.length);
    heading=ppval(data.heading,clipped);position=ppval(data.position,clipped);
    outside=station<0 | station>curve.length;
    if any(outside)
        distance=station(outside)-clipped(outside);
        curvature=laneGeometry.referenceCurvature(station(outside),curve);initialHeading=heading(outside);
        turn=curvature.*distance;halfTurn=turn/2;
        factor=ones(size(turn));nonzero=halfTurn~=0;
        factor(nonzero)=sin(halfTurn(nonzero))./halfTurn(nonzero);
        direction=initialHeading+halfTurn;
        position(:,outside)=position(:,outside)+[cos(direction);sin(direction)].*(distance.*factor);
        heading(outside)=initialHeading+turn;
    end
    position=position+[-sin(heading);cos(heading)].*reshape(lateral,1,[]);
    heading=reshape(heading,shape);
end

function bound=localPolynomialBound(polynomial,lower,upper)
    if lower>upper,bound=0;return;end
    lower=max(lower,polynomial.breaks(1));upper=min(upper,polynomial.breaks(end));
    if lower>upper,bound=0;return;end
    bound=0;
    for index=find(polynomial.breaks(1:end-1)<=upper & polynomial.breaks(2:end)>=lower)
        lo=max(lower,polynomial.breaks(index))-polynomial.breaks(index);
        hi=min(upper,polynomial.breaks(index+1))-polynomial.breaks(index);
        row=polynomial.coefs(index,:);width=hi-lo;
        if numel(row)==4
            power=[polyval(row,lo),polyval(row(1:3).*[3,2,1],lo)*width, ...
                (3*row(1)*lo+row(2))*width^2,row(1)*width^3];
            bernstein=[power(1),power(1)+power(2)/3, ...
                power(1)+2*power(2)/3+power(3)/3,sum(power)];
        else
            power=[polyval(row,lo),(2*row(1)*lo+row(2))*width,row(1)*width^2];
            bernstein=[power(1),power(1)+power(2)/2,sum(power)];
        end
        bound=max(bound,max(abs(bernstein))+128*eps*(1+sum(abs(power))));
    end
    bound=bound+128*eps*(1+bound);
end

function projection=localProfileProjection(position,curve,stationHint)
% A selected local projection branch, never a global self-intersection claim.
    if nargin<3,stationHint=[];end
    count=size(position,2);station=zeros(1,count);
    data=localProfileData(curve);
    for index=1:count
        point=position(:,index);
        if ~isempty(stationHint)
            seed=stationHint(min(index,numel(stationHint)));
        else
            samples=data.position.breaks;
            sampled=ppval(data.position,samples);
            distance=sum((sampled-point).^2,1);
            [~,nearest]=min(distance);seed=samples(nearest);
            if nearest==1
                seed=min(0,[cos(curve.heading),sin(curve.heading)]*(point-curve.origin));
            elseif nearest==numel(samples)
                lastHeading=ppval(data.heading,curve.length);
                seed=curve.length+max(0,[cos(lastHeading),sin(lastHeading)]*(point-sampled(:,end)));
            end
        end
        for iteration=1:30
            [center,heading]=laneGeometry.referencePose(seed,0,curve);
            tangent=[cos(heading);sin(heading)];normal=[-tangent(2);tangent(1)];
            offset=center-point;k=laneGeometry.referenceCurvature(seed,curve);
            denominator=1+k*(normal.'*offset);
            if denominator<=.1
                error("collisionAvoidanceController:ambiguousReferenceProjection", ...
                    "The selected smooth-reference projection is outside its regular Frenet branch.");
            end
            step=(tangent.'*offset)/denominator;
            seed=seed-min(max(step,-5),5);
            if abs(step)<1e-11,break;end
        end
        [center,heading]=laneGeometry.referencePose(seed,0,curve);
        [positionError,~]=laneGeometry.referenceErrorBound(seed,curve);
        if abs([cos(heading),sin(heading)]*(center-point))>1e-8+positionError
            error("collisionAvoidanceController:ambiguousReferenceProjection", ...
                "A regular stationary projection could not be resolved on the selected reference branch.");
        end
        station(index)=seed;
    end
    [center,heading]=laneGeometry.referencePose(station,0,curve);
    lateral=sum([-sin(heading);cos(heading)].*(position-center),1);
    projection=struct('point',center,'heading',heading,'station',station,'lateralPosition',lateral);
end
