function value = nrmmTargetHistory(action, varargin)
%nrmmTargetHistory Measurement-consistent Cartesian derivative enclosures.
% Bounded-noise secants and quadratic interpolation intersect the physical
% domain. No averaging-down of deterministic noise or observer-state reset.
    switch string(action)
        case "initialize"
            domain = varargin{1};
            modelJerk = varargin{2};
            validateattributes(modelJerk,{'double'},{'scalar','real','finite','nonnegative'});
            sideslipMaximum = 0;
            if isfield(domain,"sideslipMaximum"),sideslipMaximum = domain.sideslipMaximum;end
            value = struct("time",zeros(1,0),"position",zeros(2,0), ...
                "radius",zeros(2,0),"duration",varargin{3}, ...
                "speedMaximum",domain.speedMaximum, ...
                "accelerationMaximum",domain.accelerationNormBound, ...
                "yawRateMaximum",domain.yawRateMaximum,"sideslipMaximum",sideslipMaximum, ...
                "constantCurvature",modelJerk==0, ...
                "jerkMaximum",hypot(domain.speedMaximum*domain.yawRateMaximum^2, ...
                    3*domain.scalarAccelerationMaximum*domain.yawRateMaximum)+modelJerk, ...
                "yawAccelerationMaximum",inf,"records",struct("time",{}, ...
                    "relativePosition",{},"egoPosition",{},"yawRate",{},"heading",{},"headingRadius",{}));
            if numel(varargin)>3
                validateattributes(varargin{4},{'double'},{'scalar','real','nonnegative','nonnan'});
                value.yawAccelerationMaximum = varargin{4};
            end
        case "measure"
            value = varargin{1};
            time = varargin{2};position = varargin{3};radius = varargin{4};
            if ~isfinite(time) || any(~isfinite([position(:);radius(:)])) || any(radius<0)
                error("nrmmTargetHistory:invalidMeasurement","History measurements need finite bounds.");
            end
            if ~isempty(value.time) && time < value.time(end)
                error("nrmmTargetHistory:timeOrder","History timestamps must not decrease.");
            end
            keep = value.time<time & value.time>=time-value.duration;
            value.time = [value.time(keep),time];
            value.position = [value.position(:,keep),position(:)];
            value.radius = [value.radius(:,keep),radius(:)];
        case "enclose"
            history = varargin{1};
            if ~isempty(history.time) && isfield(history,"positionNoise") ...
                    && numel(history.records)==numel(history.time) ...
                    && ~isfield(history,"holdYawAcceleration") ...
                    && exist("nrmmTargetHistoryKernelMex","file")==3
                value = nrmmTargetHistoryKernelMex(history,varargin{2});
            else
                value = localEnclose(varargin{:});
            end
        case "encloseKernel"
            % Shared numeric path for native/reference enclosure checks.
            value = localEnclose(varargin{:});
        case "contractMotion"
            value = localContractMotion(varargin{:});
        case "sensor"
            value = varargin{1};input = varargin{2};design = varargin{3};
            if isfield(input,"velocityErrorMaximum")
                design.sensors.velocityNoiseMaximum=input.velocityErrorMaximum;
            end
            % The runtime supplies the course geometry it integrates with; a
            % caller without it and without the shared vehicle model gets
            % the kinematic correspondence, as before.
            if isfield(input,"courseGeometry")
                course = input.courseGeometry;
            elseif isfield(design,"egoModel") && ~isempty(design.egoModel)
                course = nrmmEgoCourseGeometry("course",input,design);
            else
                course = nrmmEgoCourseGeometry("kinematicCourse",input.gnssVelocity,input.yawRate, ...
                    design.yaw.courseModel.rearAxleDistance,design.sensors.velocityNoiseMaximum, ...
                    design.sensors.gyroscopeNoiseMaximum,design.yaw.courseModel.singleTrackYawRateMismatchMaximum, ...
                    design.yaw.courseModel.sideslipDomainMaximum);
            end
            heading = course.correspondence.heading;
            angle = min(pi,course.correspondence.radius);
            position = input.radarRelativePosition(:);
            relative = [cos(heading),-sin(heading);sin(heading),cos(heading)]*position;
            orientation = min(2*norm(position), ...
                abs([-relative(2);relative(1)])*angle+norm(position)*angle^2/2);
            radius = design.sensors.positionNoiseMaximum+design.sensors.radarNoiseMaximum+orientation;
            value = nrmmTargetHistory("measure",value,input.time,input.gnssPosition+relative,radius);
            record = struct("time",input.time,"relativePosition",position, ...
                "egoPosition",input.gnssPosition,"yawRate",input.yawRate, ...
                "heading",heading,"headingRadius",angle);
            keep = [value.records.time]<input.time & [value.records.time]>=input.time-value.duration;
            if isfield(input,"yawAccelerationMaximum") || isfield(value,"holdYawAcceleration")
                if ~isfield(value,"holdYawAcceleration")
                    value.holdYawAcceleration=repmat(value.yawAccelerationMaximum,1,numel(value.records));
                end
                maximum=value.yawAccelerationMaximum;
                if isfield(input,"yawAccelerationMaximum")
                    validateattributes(input.yawAccelerationMaximum,{'double'},{'scalar','nonnegative','nonnan'});
                    maximum=min(maximum,input.yawAccelerationMaximum);
                end
                value.holdYawAcceleration=[value.holdYawAcceleration(keep),maximum];
            end
            value.records = [value.records(keep),record];
            value.gyroscopeNoise = design.sensors.gyroscopeNoiseMaximum;
            value.positionNoise = design.sensors.positionNoiseMaximum+design.sensors.radarNoiseMaximum;
        otherwise
            error("nrmmTargetHistory:invalidAction","Unknown history action.");
    end
end

function result=localContractMotion(history,time,intervals)
    % Intersect the current constant-(A,kappa) family with PAST bounded
    % position differences. A slice is removed only when its complete NRMM
    % displacement image is disjoint from a measurement enclosure. Keeping
    % the hull of all unresolved slices is conservative, including at the
    % finite refinement budget. Columns are [speed, A, curvature, course].
    result=struct('intervals',intervals,'available',true,'checks',0,'usedSamples',0);
    count=numel(history.time);
    if ~history.constantCurvature || count<3 || abs(history.time(end)-time)>1e-9 ...
            || intervals(1,1)<=0,return;end
    duration=time-history.time;
    % A past stop invalidates the backward constant-acceleration formula.
    % Use only times for which EVERY member remains forward-moving.
    usable=find(duration>0 & intervals(1,1)-max(0,intervals(2,2))*duration>0);
    if isempty(usable),return;end
    selected=unique(usable(round(linspace(1,numel(usable),min(4,numel(usable))))));
    centers=zeros(2,numel(selected));radii=centers;
    correlated=isfinite(history.yawAccelerationMaximum) && numel(history.records)==count;
    if correlated,transport=localTransport(history);end
    for j=1:numel(selected)
        index=selected(j);
        centers(:,j)=history.position(:,index)-history.position(:,end);
        radii(:,j)=history.radius(:,index)+history.radius(:,end);
        if correlated
            [centers(:,j),radii(:,j)]=localCombination(history,transport,[index,count],[1,-1]);
        end
    end
    result.usedSamples=numel(selected);
    % The angle between two consecutive chords equals kappa times half
    % their TOTAL arc length, also under tangential acceleration. This uses
    % both chords together and preserves information that independent
    % position-to-parameter slices can lose.
    first=selected(1);middle=round((first+count)/2);
    if middle>first && middle<count
        pairs=[first,middle;middle,count];chord=zeros(4,1);uncertainty=chord;
        for j=1:2
            indices=pairs(j,:);rows=2*j+[-1,0];
            chord(rows)=history.position(:,indices(2))-history.position(:,indices(1));
            uncertainty(rows)=history.radius(:,indices(1))+history.radius(:,indices(2));
            if correlated
                [chord(rows),uncertainty(rows)]=localCombination(history,transport,indices,[-1,1]);
            end
        end
        primitive=promote(nrmmSet.range([chord-uncertainty;intervals(1,1:2).'], ...
            [chord+uncertainty;intervals(2,1:2).']));
        arc=duration(first)*primitive(5)-duration(first)^2/2*primitive(6);
        dot=primitive(1)*primitive(3)+primitive(2)*primitive(4);
        [arcLow,arcHigh]=bounds(arc);dotLow=bounds(dot);
        if arcLow>0 && max(abs(intervals(:,3)))*arcHigh<pi && dotLow>0
            angle=atan((primitive(1)*primitive(4)-primitive(2)*primitive(3))/dot);
            [kl,kh]=bounds(2*angle/arc);
            intervals(:,3)=[max(intervals(1,3),kl);min(intervals(2,3),kh)];
            if intervals(1,3)>intervals(2,3),result.available=false;return;end
        end
    end
    for sweep=1:3
        previous=intervals;
        for coordinate=[4,3,1,2]
            edges=linspace(intervals(1,coordinate),intervals(2,coordinate),17);
            if edges(end)-edges(1)<1e-12,continue;end
            keep=false(1,16);
            for cell=1:16
                part=intervals;part(:,coordinate)=edges(cell:cell+1).';
                parameters=promote(nrmmSet.range(part(1,:).',part(2,:).'));consistent=true;
                for j=1:numel(selected)
                    dt=-duration(selected(j));
                    arc=parameters(1)*dt+parameters(2)*(dt^2/2);
                    turn=parameters(3)*arc/2;angle=parameters(4)+turn;
                    image=arc*sinc(turn)*[cos(angle);sin(angle)];[lo,hi]=bounds(image);
                    guard=512*eps(max(1,max(abs([lo;hi;centers(:,j)]))));
                    result.checks=result.checks+1;
                    if any(hi<centers(:,j)-radii(:,j)-guard | lo>centers(:,j)+radii(:,j)+guard)
                        consistent=false;break;
                    end
                end
                keep(cell)=consistent;
            end
            retained=find(keep);
            if isempty(retained),result.available=false;return;end
            intervals(:,coordinate)=edges([retained(1),retained(end)+1]).';
        end
        if isequal(intervals,previous),break;end
    end
    result.intervals=intervals;
end

function enclosure = localEnclose(history, time)
    v = history.speedMaximum;a = history.accelerationMaximum;j = history.jerkMaximum;
    lower = [-inf(2,1);-v*ones(2,1);-a*ones(2,1)];
    upper = -lower;
    count = numel(history.time);
    heading = struct("center",0,"radius",pi,"available",false);
    if count == 0
        enclosure = struct("lower",lower,"upper",upper,"available",false,"samples",0,"heading",heading);
        return;
    end
    age = time-history.time(end);
    if age < -128*eps(max(1,abs(time)))
        error("nrmmTargetHistory:futureMeasurement","History cannot use future measurements.");
    end
    age = max(0,age);
    lower(1:2) = history.position(:,end)-history.radius(:,end)-v*age;
    upper(1:2) = history.position(:,end)+history.radius(:,end)+v*age;
    if count > 1
        duration = history.time(end)-history.time(1:end-1);
        centers = (history.position(:,end)-history.position(:,1:end-1))./duration;
        measuredRadius = (history.radius(:,end)+history.radius(:,1:end-1))./duration;
        radii = measuredRadius+a*(duration/2+age);
        for index = 1:count-1
            heading = localHeading(history,centers(:,index),measuredRadius(:,index),duration(index),age,heading);
        end
        lower(3:4) = max(lower(3:4),max(centers-radii,[],2));
        upper(3:4) = min(upper(3:4),min(centers+radii,[],2));
    end
    correlated = isfinite(history.yawAccelerationMaximum) && numel(history.records)==count;
    transport = struct("relative",zeros(2,count),"angleRadius",zeros(1,count),"headingRadius",0);
    if correlated
        transport = localTransport(history);
        for index = 1:count-1
            duration = history.time(end)-history.time(index);
            [center,radius] = localCombination(history,transport,[index,count],[-1,1]/duration);
            heading = localHeading(history,center,radius,duration,age,heading);
            radius = radius+a*(duration/2+age);
            lower(3:4) = max(lower(3:4),center-radius);
            upper(3:4) = min(upper(3:4),center+radius);
        end
    end
    for lag = 1:floor((count-1)/2)
        selected = count-[2*lag,lag,0];
        t = history.time(selected)-time;
        denominator = [(t(1)-t(2))*(t(1)-t(3)), ...
            (t(2)-t(1))*(t(2)-t(3)),(t(3)-t(1))*(t(3)-t(2))];
        velocityWeights = [-(t(2)+t(3)),-(t(1)+t(3)),-(t(1)+t(2))]./denominator;
        accelerationWeights = 2./denominator;
        weights = [velocityWeights;accelerationWeights];
        remainder = j*abs(t).^3/6;
        centers = history.position(:,selected)*weights.';
        radii = (history.radius(:,selected)+remainder)*abs(weights).';
        lower(3:6) = max(lower(3:6),reshape(centers-radii,4,1));
        upper(3:6) = min(upper(3:6),reshape(centers+radii,4,1));
        if correlated
            for order = 1:2
                [center,radius] = localCombination(history,transport,selected,weights(order,:));
                radius = radius+abs(weights(order,:))*remainder.';
                rows = 2*order+(1:2);
                lower(rows) = max(lower(rows),center-radius);
                upper(rows) = min(upper(rows),center+radius);
            end
        end
    end
    guard = 512*eps(max(1,max(abs([lower;upper]))));
    lower = lower-guard;upper = upper+guard;
    enclosure = struct("lower",lower,"upper",upper, ...
        "available",all(lower<=upper),"samples",count,"heading",heading);
end

function heading = localHeading(history,center,radius,duration,age,heading)
% A constant-curvature chord points along its midpoint course, regardless
% of tangential acceleration. Tangential acceleration must not be charged
% as an independent transverse acceleration when bounding this direction.
    magnitude = norm(center);noise = norm(radius);
    turn = history.yawRateMaximum*duration;
    if noise>=magnitude || turn>=pi/2,return;end
    factor = 1;
    if history.constantCurvature,factor = 0.5;end
    angle = asin(min(1,noise/magnitude))+factor*turn ...
        +history.yawRateMaximum*age+history.sideslipMaximum;
    if angle<heading.radius
        heading = struct("center",atan2(center(2),center(1)), ...
            "radius",angle+128*eps,"available",true);
    end
end

function transport = localTransport(history)
    records = history.records;
    dt = diff(history.time);
    gyro = [records.yawRate];
    increments = (gyro(1:end-1)+gyro(2:end)).*dt/2;
    angle = -[fliplr(cumsum(fliplr(increments))),0];
    % For an M-Lipschitz rate, trapezoidal integration error over h is
    % at most M*h^2/4. Both endpoint gyro noise bounds contribute n*h.
    acceleration=history.yawAccelerationMaximum*ones(size(dt));
    if isfield(history,'holdYawAcceleration') && numel(history.holdYawAcceleration)==numel(records)
        acceleration=min(acceleration,history.holdYawAcceleration(1:end-1));
    end
    incrementRadius = history.gyroscopeNoise*dt+acceleration.*dt.^2/4;
    radius = [fliplr(cumsum(fliplr(incrementRadius))),0];
    heading = records(end).heading+angle;
    relative = [records.relativePosition];
    rotated = [cos(heading).*relative(1,:)-sin(heading).*relative(2,:); ...
        sin(heading).*relative(1,:)+cos(heading).*relative(2,:)];
    transport = struct("relative",rotated,"angleRadius",radius, ...
        "headingRadius",records(end).headingRadius);
end

function [center,radius] = localCombination(history,transport,selected,weights)
    relative = transport.relative(:,selected);
    common = relative*weights.';
    center = [history.records(selected).egoPosition]*weights.'+common;
    angle = transport.headingRadius;
    radius = history.positionNoise*sum(abs(weights)) ...
        +min(2*norm(common),abs([-common(2);common(1)])*angle+norm(common)*angle^2/2);
    turns = transport.angleRadius(selected);
    norms = vecnorm(relative,2,1);
    individual = abs([-relative(2,:);relative(1,:)]).*turns ...
        +norms.*(turns.^2/2+2*sin(angle/2).*turns);
    radius = radius+individual*abs(weights).';
end
