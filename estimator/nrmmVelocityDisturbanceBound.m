function certificate = nrmmVelocityDisturbanceBound(gain, speedInterval, model, sensors)
% nrmmVelocityDisturbanceBound Bound the combined gyro forcing before norms.
% The same gyro error enters l*u in the innovation and -u*J*v in propagation.
% C_omega bounds [v_y-k*l*zeta; k*l-v_x], with |zeta| <= tan(b).
% Sensor fields may also contain certified effective held-input error bounds.

    persistent cachedGeometry cachedCoefficients
    geometry = {gain,speedInterval,model};
    if ~isequaln(geometry,cachedGeometry)
        cachedCoefficients = localCoefficients(gain,speedInterval,model);
        cachedGeometry = geometry;
    end
    sensorFields = ["velocityNoiseMaximum","gyroscopeNoiseMaximum","accelerometerNoiseMaximum"];
    for field = sensorFields
        validateattributes(sensors.(field), {'double'}, {'real','finite','scalar','nonnegative'});
    end
    certificate = cachedCoefficients;
    forcing = sensors.accelerometerNoiseMaximum ...
        + certificate.speedCoefficient*sensors.velocityNoiseMaximum ...
        + certificate.gyroCoefficient*sensors.gyroscopeNoiseMaximum ...
        + certificate.mismatchCoefficient*model.singleTrackYawRateMismatchMaximum;
    certificate.disturbanceBound = forcing;
    certificate.ultimateBound = forcing/gain;
end

function certificate = localCoefficients(gain,speedInterval,model)
% Geometry and observer gain are fixed across changing held sensor bounds.
    validateattributes(gain, {'double'}, {'real','finite','scalar','positive'});
    validateattributes(speedInterval, {'double'}, {'real','finite','numel',2,'nonnegative'});
    if speedInterval(1) > speedInterval(2)
        error("nrmmVelocityDisturbanceBound:invalidSpeedInterval", ...
            "The speed interval must be ordered.");
    end
    angle = model.sideslipDomainMaximum;
    distance = model.rearAxleDistance;
    validateattributes(angle, {'double'}, {'real','finite','scalar','>=',0,'<',pi/2});
    validateattributes(distance, {'double'}, {'real','finite','scalar','positive'});
    mismatch = model.singleTrackYawRateMismatchMaximum;
    validateattributes(mismatch, {'double'}, {'real','finite','scalar','nonnegative'});
    cosine = cos(angle);
    speed = speedInterval(:);
    gyro = max(hypot(speed*sin(angle)+gain*distance*tan(angle), ...
        gain*distance-speed*cosine));
    speedCoefficient = gain/cosine;
    mismatchCoefficient = gain*distance/cosine;
    certificate = struct("gyroCoefficient",gyro, ...
        "separatedGyroCoefficient",max(speed)+gain*distance/cosine, ...
        "speedCoefficient",speedCoefficient,"mismatchCoefficient",mismatchCoefficient, ...
        "disturbanceBound",0,"ultimateBound",0);
end
