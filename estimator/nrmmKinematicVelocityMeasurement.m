function [velocity, feasible] = nrmmKinematicVelocityMeasurement(gnssVelocity, yawRate, design)
% nrmmKinematicVelocityMeasurement Extend the forward kinematic inverse.
% The lateral measurement is deliberately not clipped: its error is exactly
% l*(nOmega+dSt). The longitudinal floor keeps the map defined at standstill
% and outside the raw square-root branch. No orientation estimate is used.
% The optional feasibility result concerns C_m, before any orientation prior.

    validateattributes(gnssVelocity, {'double'}, {'real','finite','size',[2,1]});
    validateattributes(yawRate, {'double'}, {'real','finite','scalar'});
    model = design.yaw.courseModel;
    speed = norm(gnssVelocity);
    lateral = model.rearAxleDistance*yawRate;
    cosine = cos(model.sideslipDomainMaximum);
    velocity = [sqrt(max(speed^2-lateral^2, (cosine*speed)^2)); lateral];
    if nargout < 2
        return
    end
    sensors = design.sensors;
    domain = design.operatingDomain;
    speedLower = max(domain.egoSpeedMinimum, speed-sensors.velocityNoiseMaximum);
    speedUpper = min(domain.egoSpeedMaximum, speed+sensors.velocityNoiseMaximum);
    lateralError = model.rearAxleDistance*(sensors.gyroscopeNoiseMaximum ...
        + model.singleTrackYawRateMismatchMaximum);
    lateralMaximum = speedUpper*sin(model.sideslipDomainMaximum);
    lateralLower = max(-lateralMaximum, lateral-lateralError);
    lateralUpper = min(lateralMaximum, lateral+lateralError);
    tolerance = 256*eps(max([1,speed,abs(lateral),domain.egoSpeedMaximum]));
    feasible = struct("consistent", speedLower <= speedUpper+tolerance ...
        && lateralLower <= lateralUpper+tolerance, ...
        "speedInterval", [speedLower;speedUpper], ...
        "lateralInterval", [lateralLower;lateralUpper], ...
        "floorActive", abs(lateral) > speed*sin(model.sideslipDomainMaximum));
end
