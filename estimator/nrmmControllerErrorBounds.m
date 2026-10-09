function value = nrmmControllerErrorBounds(action, varargin)
%nrmmControllerErrorBounds Publish state-time enclosures for control.
%
%   output = nrmmControllerErrorBounds("publish", output, bound, input, design)
%       The comparison state bounds yaw, body velocity and [rho; q; s]. Absolute
%       position also uses the timestamped GNSS ball and the true speed domain.
%       These are current estimation bounds, not a prediction-horizon certificate.
%   bounds = nrmmControllerErrorBounds("targetParameters", velocity, acceleration, ...
%       velocityRadius, accelerationRadius, frameRotationRadius)
%       The true target velocity and acceleration lie within velocityRadius and
%       accelerationRadius (norms) of the estimates velocity and acceleration, given
%       in one frame whose orientation errs by at most frameRotationRadius (rad).
%       Speed, speed-rate and normal acceleration do not depend on the frame. The
%       NRMM parameters of the true state, relative to the estimate's own
%       (speed |v|, course atan2(v), speed-rate v'a/|v|), satisfy
%       speedErrorBound      |V - |v|| <= velocityRadius;
%       courseErrorBound     frameRotationRadius + asin(velocityRadius/|v|), or
%       pi when the velocity ball contains rest;
%       speedRateErrorBound  |A - v'a/|v|| <= accelerationRadius + 2|a| sin(theta/2),
%       theta the course bound of the frame itself;
%       curvatureInterval    [lo; hi] of the normal acceleration over the speed
%       squared, a_N/V^2, over both balls, or [-Inf; Inf] when
%       the velocity ball contains rest.
%       For vector error boxes, supply the Euclidean norms of their half-widths as
%       velocityRadius and accelerationRadius; a component radius alone is insufficient.

    switch string(action)
        case "publish"
            value = localPublish(varargin{1}, varargin{2}, varargin{3}, varargin{4});
        case "targetParameters"
            value = localTargetParameterBounds(varargin{:});
        otherwise
            error("nrmmControllerErrorBounds:invalidAction", ...
                "action must be 'publish' or 'targetParameters'.");
    end
end

%% State-time enclosures published to the controller

function output = localPublish(output, bound, input, design)

    age = max(0.0, output.stateTime-input.time);
    egoPosition = norm(output.egoPositionInertial-input.gnssPosition) ...
        + design.sensors.positionNoiseMaximum ...
        + design.operatingDomain.egoSpeedMaximum*age;
    egoYawRate = design.operatingDomain.egoYawRateMaximum+abs(input.yawRate);
    if isfinite(bound.holdBounds.yawAcceleration)
        egoYawRate = min(egoYawRate, design.sensors.gyroscopeNoiseMaximum ...
            + bound.holdBounds.yawAcceleration*age);
    elseif age == 0.0
        egoYawRate = design.sensors.gyroscopeNoiseMaximum;
    end
    velocityBounds = localVelocityBounds(output, bound, input, design, age);
    egoBounds = [egoPosition; egoPosition; bound.yaw; velocityBounds; egoYawRate];
    egoAvailable = bound.egoValid && bound.orientationSet.valid && all(isfinite(egoBounds));
    if ~egoAvailable
        egoBounds(:) = inf;
    end
    % These separate channels do not depend on an absolute orientation set.
    % An empty yaw intersection makes the full controller vector unavailable,
    % while retaining valid body-velocity, GNSS-position and gyro enclosures.
    output.egoPositionErrorBound = egoPosition;
    output.egoYawErrorBound = bound.yaw;
    output.egoBodyVelocityErrorBound = bound.bodyVelocity;
    output.egoYawRateErrorBound = egoYawRate;
    if ~bound.egoValid
        output.egoPositionErrorBound = Inf;
        output.egoBodyVelocityErrorBound = Inf;
        output.egoYawRateErrorBound = Inf;
    end
    output.controllerStateErrorBound = egoBounds;
    output.controllerErrorBoundSource = "nrmm-state-time-enclosure";
    output.controllerErrorBound = localCertificate( ...
        "ego-state-v1", output.stateTime, egoBounds, egoAvailable, bound.scope);
    if egoAvailable && isfield(output,'egoYaw') && isfield(output,'egoBodyVelocity') ...
            && isfield(input,'gnssVelocity')
        noise=design.sensors.velocityNoiseMaximum;
        if age>0,noise=noise+bound.holdBounds.acceleration*age;end
        if isfinite(noise)
            angle=output.egoYaw;rotation=[cos(angle),sin(angle);-sin(angle),cos(angle)];
            velocity=rotation*input.gnssVelocity(:);
            remainder=norm(velocity)*bound.yaw^2/2;
            offset=abs(velocity-output.egoBodyVelocity(:))+remainder;
            generator=zeros(6,8);
            generator(1:3,1:3)=diag(egoBounds(1:3));
            % The same yaw error rotates the inferred body velocity in the
            % opposite direction. Boxing these coordinates independently
            % loses the near cancellation in inertial velocity and course.
            generator(4:5,3)=[velocity(2);-velocity(1)]*bound.yaw;
            generator(4:5,4:5)=noise*eye(2);
            generator(4:5,6:7)=diag(offset);generator(6,8)=egoYawRate;
            output.controllerErrorBound.generator=generator;
            output.controllerErrorBound.generatorConvention="commonPoseColumnsFirst";
        end
    end

    if isempty(output.targetEstimate)
        return;
    end
    domain = design.target.domain;
    rotationError = 2*sin(min(bound.yaw, pi)/2);
    target = output.targetEstimate;
    components = bound.targetComponents;
    range = min(bound.trueRangeMaximum, norm(target.relativePosition));
    positionRadius = egoPosition+components(1)+range*rotationError;
    velocityRadius = components(2) ...
        + min(domain.speedMaximum, norm(target.targetVelocity))*rotationError;
    accelerationRadius = components(3) ...
        + min(domain.accelerationNormBound, norm(target.targetAcceleration))*rotationError;
    speed = norm(target.targetVelocity);
    courseRadius = pi;
    if components(2) < speed
        courseRadius = asin(min(1.0, components(2)/speed));
    end
    % Global Lipschitz bounds for the saturated inverse reconstruction.
    speedFloor = domain.speedMinimum;
    yawRateRadius = min(domain.yawRateMaximum+abs(target.targetYawRate), ...
        (domain.accelerationNormBound/speedFloor^2 ...
        + 2*domain.speedMaximum*domain.accelerationNormBound/speedFloor^3)*components(2) ...
        + domain.speedMaximum/speedFloor^2*components(3));
    sideslipRadius = min(domain.sideslipMaximum+abs(target.targetSideslip), ...
        domain.rearAxleDistance/cos(domain.sideslipMaximum) ...
        *(yawRateRadius/speedFloor ...
        + domain.yawRateMaximum/speedFloor^2*components(2)));
    yawRadius = min(pi, bound.yaw+courseRadius+sideslipRadius);
    values = [repmat(positionRadius, 2, 1); repmat(velocityRadius, 2, 1); ...
        repmat(accelerationRadius, 2, 1); yawRadius; yawRateRadius];
    historyAvailable = true;
    if isfield(bound,"targetHistory")
        history = nrmmTargetHistory("enclose",bound.targetHistory,output.stateTime);
        if history.samples > 0
            historyAvailable = history.available;
            center = [target.targetPositionInertial;target.targetVelocityInertial;target.targetAccelerationInertial];
            historyRadius = max(abs([history.lower-center,history.upper-center]),[],2);
            values(1:6) = min(values(1:6),historyRadius);
            velocityBall = norm(values(3:4));
            if velocityBall < norm(target.targetVelocityInertial)
                values(7) = min(values(7),asin(velocityBall/norm(target.targetVelocityInertial)) ...
                    +domain.sideslipMaximum+abs(target.targetSideslip));
            end
            if history.heading.available
                difference = abs(atan2(sin(history.heading.center-target.targetHeadingInertial), ...
                    cos(history.heading.center-target.targetHeadingInertial)));
                historyAvailable = historyAvailable && difference<=values(7)+history.heading.radius;
                values(7) = min(values(7),difference+history.heading.radius);
            end
            output.targetEstimate.measurementHistoryEnclosure = history;
        end
    end
    available = egoAvailable && bound.valid && all(isfinite(values));
    available = available && historyAvailable;
    if ~available
        values(:) = inf;
    end
    output.targetEstimate.controllerErrorBound = localCertificate( ...
        "target-state-v1", output.stateTime, values, available, bound.scope);
    output.targetEstimate.controllerErrorBoundSource = "nrmm-state-time-enclosure";
    output.targetEstimate.targetPositionInertialErrorBound = values(1:2);
    output.targetEstimate.targetVelocityInertialErrorBound = values(3:4);
    output.targetEstimate.targetAccelerationInertialErrorBound = values(5:6);
    output.targetEstimate.targetYawErrorBound = values(7);
    output.targetEstimate.targetYawRateErrorBound = values(8);
    % This legacy contract field bounds |a| = hypot(A, V*omega), not A.
    % The prediction set below supplies the constant-parameter intervals.
    output.targetEstimate.predictionMotion = [];
    if design.target.modelJerkMaximum == 0
        % Every member has constant A and beta. Parameter uncertainty does
        % not introduce time-varying target maneuvers or future process noise.
        output.targetEstimate.predictionMotion = struct( ...
            "kind","nrmm-motion-v1", ...
            "curvatureMaximum",sin(domain.sideslipMaximum)/domain.rearAxleDistance, ...
            "speedRateMaximum",domain.scalarAccelerationMaximum, ...
            "scalarAccelerationMaximum",domain.accelerationNormBound);
        parameters = localTargetParameterBounds(target.targetVelocity, ...
            target.targetAcceleration, components(2), components(3), bound.yaw);
        % The inverse reconstruction clips A. Recenter its enclosure at the
        % value actually published to the controller, not the raw projection.
        rawAcceleration = 0;
        if speed > 0
            rawAcceleration = dot(target.targetVelocity,target.targetAcceleration)/speed;
        end
        parameters.speedRateErrorBound = parameters.speedRateErrorBound ...
            + abs(rawAcceleration-target.targetScalarAcceleration);
        relativeCourseRadius = pi;
        if components(2)<speed,relativeCourseRadius=asin(components(2)/speed);end
        if isfield(output.targetEstimate,'measurementHistoryEnclosure') && available
            % The history and observer enclose the same current state. Use
            % their intersection for prediction as well as publication.
            historyParameters = localTargetParameterBounds( ...
                target.targetVelocityInertial,target.targetAccelerationInertial, ...
                norm(values(3:4)),norm(values(5:6)),0);
            historyParameters.speedRateErrorBound = historyParameters.speedRateErrorBound ...
                +abs(rawAcceleration-target.targetScalarAcceleration);
            parameters.speedErrorBound = min(parameters.speedErrorBound,historyParameters.speedErrorBound);
            parameters.speedRateErrorBound = min(parameters.speedRateErrorBound,historyParameters.speedRateErrorBound);
            parameters.courseErrorBound = min(parameters.courseErrorBound,historyParameters.courseErrorBound);
            parameters.curvatureInterval = [max(parameters.curvatureInterval(1),historyParameters.curvatureInterval(1)); ...
                min(parameters.curvatureInterval(2),historyParameters.curvatureInterval(2))];
            historyCourse = min(historyParameters.courseErrorBound,values(7)+sideslipRadius);
            % Convert an inertial course enclosure to the uncertain ego
            % frame. Omitting ego yaw here would understate relative error.
            relativeCourseRadius = min(relativeCourseRadius,historyCourse+bound.yaw);
        end
        output.targetEstimate.predictionErrorSet = localPredictionSet( ...
            output,target,components,parameters,domain,available,relativeCourseRadius);
        if ~available
            parameters = struct("speedErrorBound", Inf, "courseErrorBound", Inf, ...
                "speedRateErrorBound", Inf, "curvatureInterval", [-Inf; Inf]);
        end
        output.targetEstimate.targetSpeedErrorBound = parameters.speedErrorBound;
        output.targetEstimate.targetCourseErrorBound = parameters.courseErrorBound;
        output.targetEstimate.targetSpeedRateErrorBound = parameters.speedRateErrorBound;
        output.targetEstimate.targetCurvatureInterval = parameters.curvatureInterval;
    end
end

function set = localPredictionSet(output,target,components,parameters,domain,available,courseRadius)
    curvatureLimit = sin(domain.sideslipMaximum)/domain.rearAxleDistance;
    speed = norm(target.targetVelocity);
    speedInterval = [max(domain.speedMinimum,speed-parameters.speedErrorBound); ...
        min(domain.speedMaximum,speed+parameters.speedErrorBound)];
    accelerationInterval = [max(-domain.scalarAccelerationMaximum, ...
        target.targetScalarAcceleration-parameters.speedRateErrorBound); ...
        min(domain.scalarAccelerationMaximum, ...
        target.targetScalarAcceleration+parameters.speedRateErrorBound)];
    curvatureInterval = [max(-curvatureLimit,parameters.curvatureInterval(1)); ...
        min(curvatureLimit,parameters.curvatureInterval(2))];
    yaw = NaN;
    if isfield(output,'egoYaw'),yaw = output.egoYaw;end
    intervals = [speedInterval,accelerationInterval,curvatureInterval];
    available = available && isfinite(yaw) && all(isfinite(intervals),'all') ...
        && all(intervals(1,:) <= intervals(2,:));
    set = struct('kind',"nrmm-constant-parameter-set-v1",'time',output.stateTime, ...
        'available',available,'referenceEgoPose',[output.egoPositionInertial;yaw], ...
        'relativePosition',target.relativePosition,'positionRadius',components(1), ...
        'courseCenter',atan2(target.targetVelocity(2),target.targetVelocity(1)), ...
        'courseRadius',courseRadius,'speedInterval',speedInterval, ...
        'accelerationInterval',accelerationInterval,'curvatureInterval',curvatureInterval, ...
        'rearAxleDistance',domain.rearAxleDistance, ...
        'scope',"currentObserverEnclosureWithConstantParameters", ...
        'futureMeasurementsAssumed',false);
end

function radius = localVelocityBounds(output, bound, input, design, age)
    radius = repmat(bound.bodyVelocity, 2, 1);
    if ~bound.orientationSet.valid || ~isfield(output, "egoBodyVelocity") ...
            || ~isfield(input, "gnssVelocity")
        return;
    end
    noise = design.sensors.velocityNoiseMaximum;
    if age > 0
        if ~isfield(bound.holdBounds, "acceleration") ...
                || ~isfinite(bound.holdBounds.acceleration)
            return;
        end
        noise = noise+bound.holdBounds.acceleration*age;
    end
    % Rotate the timestamped GNSS velocity ball over the certified yaw set.
    % Keep the observer point unchanged. Taking extrema before boxing retains
    % the small longitudinal error near straight travel; a norm radius copied
    % to both body components discards this directional information.
    velocity = input.gnssVelocity(:);
    coefficients = [velocity(1), velocity(2); velocity(2), -velocity(1)];
    intervals = bound.orientationSet.intervals;
    for component = 1:2
        cosine = coefficients(component, 1);
        sine = coefficients(component, 2);
        phase = atan2(sine, cosine);
        stationary = phase+(-2:3)*pi;
        inside = any(stationary >= intervals(:, 1) & stationary <= intervals(:, 2), 1);
        angles = [intervals(:); stationary(inside).'];
        values = cosine*cos(angles)+sine*sin(angles);
        padding = noise+64*eps(max(1, norm(velocity)));
        lower = min(values)-padding;
        upper = max(values)+padding;
        radius(component) = min(radius(component), ...
            max(abs([lower, upper]-output.egoBodyVelocity(component))));
    end
end

function certificate = localCertificate(kind, time, values, available, scope)
    certificate = struct("kind", kind, "time", time, "bounds", values, ...
        "available", available, "source", "nrmm-state-time-enclosure", ...
        "scope", scope, "futurePredictionIncluded", false, ...
        "floatingPointVerified", false);
end

%% NRMM parameter error bounds of a target estimate

function bounds = localTargetParameterBounds(velocity, acceleration, ...
        velocityRadius, accelerationRadius, frameRotationRadius)
    arguments
        velocity (2,1) double {mustBeReal,mustBeFinite}
        acceleration (2,1) double {mustBeReal,mustBeFinite}
        velocityRadius (1,1) double {mustBeReal,mustBeNonnegative}
        accelerationRadius (1,1) double {mustBeReal,mustBeNonnegative}
        frameRotationRadius (1,1) double {mustBeReal,mustBeNonnegative}
    end
    speed = norm(velocity);
    theta = pi;
    if velocityRadius < speed
        theta = asin(velocityRadius/speed);
    end
    componentRadius = accelerationRadius+2*norm(acceleration)*sin(theta/2);
    curvatureInterval = [-Inf; Inf];
    if velocityRadius < speed
        normalAcceleration = (velocity(1)*acceleration(2)-velocity(2)*acceleration(1))/speed;
        numerator = normalAcceleration+[-componentRadius; componentRadius];
        denominator = [(speed-velocityRadius)^2, (speed+velocityRadius)^2];
        quotients = numerator./denominator;
        curvatureInterval = [min(quotients, [], "all"); max(quotients, [], "all")];
    end
    bounds = struct("speedErrorBound", velocityRadius, ...
        "courseErrorBound", min(pi, frameRotationRadius+theta), ...
        "speedRateErrorBound", componentRadius, ...
        "curvatureInterval", curvatureInterval);
end
