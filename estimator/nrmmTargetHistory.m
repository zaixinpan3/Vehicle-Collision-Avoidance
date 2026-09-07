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
            value = localEnclose(varargin{:});
        case "sensor"
            value = varargin{1};input = varargin{2};design = varargin{3};index = varargin{4};
            course = certifiedKinematicCourseCorrespondence(input.gnssVelocity,input.yawRate, ...
                design.yaw.courseModel.rearAxleDistance,design.sensors.velocityNoiseMaximum, ...
                design.sensors.gyroscopeNoiseMaximum,design.yaw.courseModel.singleTrackYawRateMismatchMaximum, ...
                design.yaw.courseModel.sideslipDomainMaximum);
            heading = course.correspondence.heading;
            angle = min(pi,course.correspondence.radius);
            position = input.radarRelativePosition(index,:).';
            relative = [cos(heading),-sin(heading);sin(heading),cos(heading)]*position;
            orientation = min(2*norm(position), ...
                abs([-relative(2);relative(1)])*angle+norm(position)*angle^2/2);
            radius = design.sensors.positionNoiseMaximum+design.sensors.radarNoiseMaximum+orientation;
            value = nrmmTargetHistory("measure",value,input.time,input.gnssPosition+relative,radius);
            record = struct("time",input.time,"relativePosition",position, ...
                "egoPosition",input.gnssPosition,"yawRate",input.yawRate, ...
                "heading",heading,"headingRadius",angle);
            keep = [value.records.time]<input.time & [value.records.time]>=input.time-value.duration;
            value.records = [value.records(keep),record];
            value.gyroscopeNoise = design.sensors.gyroscopeNoiseMaximum;
            value.positionNoise = design.sensors.positionNoiseMaximum+design.sensors.radarNoiseMaximum;
        otherwise
            error("nrmmTargetHistory:invalidAction","Unknown history action.");
    end
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
    incrementRadius = history.gyroscopeNoise*dt+history.yawAccelerationMaximum*dt.^2/4;
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
