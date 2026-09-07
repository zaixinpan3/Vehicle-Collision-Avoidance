function derivative = nrmmYawObserverDerivative(yaw, measuredRate, correspondence, design)
% nrmmYawObserverDerivative Estimate orientation with gyro and heading correction.
% The wrapped innovation uses the certified course heading when informative.
% Otherwise the observer propagates the measured gyro alone. This parallel
% observer supplies inertial outputs and never drives the body-frame core.

    validateattributes(yaw, {'double'}, {'real','finite','scalar'});
    validateattributes(measuredRate, {'double'}, {'real','finite','scalar'});
    gain = design.yaw.correctionBandwidth;
    validateattributes(gain, {'double'}, {'real','finite','scalar','positive'});
    derivative = measuredRate;
    if correspondence.informative
        heading = correspondence.heading;
        validateattributes(heading, {'double'}, {'real','finite','scalar'});
        innovation = mod(heading-yaw+pi,2*pi)-pi;
        derivative = derivative+gain*innovation;
    end
end
