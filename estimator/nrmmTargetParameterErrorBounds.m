function bounds = nrmmTargetParameterErrorBounds(velocity, acceleration, ...
        velocityRadius, accelerationRadius, frameRotationRadius)
%nrmmTargetParameterErrorBounds NRMM parameter error bounds of a target estimate.
% The true target velocity and acceleration lie within velocityRadius and
% accelerationRadius (norms) of the estimates velocity and acceleration, given
% in one frame whose orientation errs by at most frameRotationRadius (rad).
% Speed, speed-rate and normal acceleration do not depend on the frame. The
% NRMM parameters of the true state, relative to the estimate's own
% (speed |v|, course atan2(v), speed-rate v'a/|v|), satisfy
%   speedErrorBound      |V - |v|| <= velocityRadius;
%   courseErrorBound     frameRotationRadius + asin(velocityRadius/|v|), or
%                        pi when the velocity ball contains rest;
%   speedRateErrorBound  |A - v'a/|v|| <= accelerationRadius + 2|a| sin(theta/2),
%                        theta the course bound of the frame itself;
%   curvatureInterval    [lo; hi] of the normal acceleration over the speed
%                        squared, a_N/V^2, over both balls, or [-Inf; Inf] when
%                        the velocity ball contains rest.
% The same bounds hold for a box of the same radii, which lies inside the ball.
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
