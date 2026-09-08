function course = certifiedKinematicCourseCorrespondence( ...
        gnssVelocity, yawRateMeasured, rearAxleDistance, ...
        velocityErrorMaximum, yawRateErrorMaximum, ...
        modelYawRateMismatchMaximum, sideslipDomainMaximum)
% certifiedKinematicCourseCorrespondence Certify yaw from course and gyro.
%
% The ego single-track relation is admitted with bounded mismatch
%
%   dSt = omegaE - VE*sin(betaE)/lrE,
%   |dSt| <= modelYawRateMismatchMaximum.
%
% With u3 = omegaE+dOmega, ||vm-vE|| <= velocityErrorMaximum, and
% Vm = ||vm||, the online sideslip estimate is
%
%   betaHat = asin(lrE*u3/Vm).
%
% Whenever Vm exceeds the velocity-error radius, the sine error obeys
%
%   |sin(betaE)-lrE*u3/Vm| <= lrE/(Vm-velocityErrorMaximum) *
%       (yawRateErrorMaximum+modelYawRateMismatchMaximum
%        +abs(u3)*velocityErrorMaximum/Vm).
%
% Intersecting that interval with the declared principal-branch domain
% |betaE| <= sideslipDomainMaximum < pi/2 and mapping its endpoints
% through asin gives an exact pointwise bound on |betaE-betaHat|. The
% returned rotation correspondence uses R(betaHat)*e1 as its measured
% body vector, so its heading is angle(vm)-betaHat and its model-angle
% radius is the certified sideslip-estimation error. The channel becomes
% uninformative at low speed, for an inadmissible asin argument, for
% inconsistent declared bounds, or when the resulting yaw arc is not
% proper.

    % The synchronized runtime, measurement enclosure and history update
    % request the same held geometry. Cache only identical complete inputs;
    % a changed value or shape still takes the full validation path.
    persistent cachedInputs cachedCourse
    inputs = {gnssVelocity,yawRateMeasured,rearAxleDistance,velocityErrorMaximum, ...
        yawRateErrorMaximum,modelYawRateMismatchMaximum,sideslipDomainMaximum};
    if isequaln(inputs,cachedInputs)
        course = cachedCourse;
        return;
    end
    gnssVelocity = localPlanarVector(gnssVelocity, "gnssVelocity");
    yawRateMeasured = localFiniteScalar( ...
        yawRateMeasured, "yawRateMeasured");
    rearAxleDistance = localPositiveScalar( ...
        rearAxleDistance, "rearAxleDistance");
    velocityErrorMaximum = localNonnegativeScalar( ...
        velocityErrorMaximum, "velocityErrorMaximum");
    yawRateErrorMaximum = localNonnegativeScalar( ...
        yawRateErrorMaximum, "yawRateErrorMaximum");
    modelYawRateMismatchMaximum = localNonnegativeScalar( ...
        modelYawRateMismatchMaximum, ...
        "modelYawRateMismatchMaximum");
    sideslipDomainMaximum = localNonnegativeScalar( ...
        sideslipDomainMaximum, "sideslipDomainMaximum");
    if sideslipDomainMaximum >= 0.5*pi
        error("certifiedKinematicCourseCorrespondence:" ...
            + "invalidSideslipDomain", ...
            "sideslipDomainMaximum must be smaller than pi/2 radians.");
    end

    measuredSpeed = norm(gnssVelocity);
    trueSpeedLowerBound = measuredSpeed-velocityErrorMaximum;
    speedCertificateValid = trueSpeedLowerBound > 0.0;
    normalizedYawRate = NaN;
    estimatedSideslip = NaN;
    speedErrorContribution = Inf;
    yawRateAndModelContribution = Inf;
    sineErrorMaximum = Inf;
    trueSideslipSineInterval = [NaN; NaN];
    trueSideslipInterval = [NaN; NaN];
    sideslipErrorMaximum = Inf;
    inversionValid = false;
    boundsConsistent = false;

    if measuredSpeed > 0.0
        normalizedYawRate = rearAxleDistance*yawRateMeasured/measuredSpeed;
        inversionValid = abs(normalizedYawRate) <= 1.0;
    end

    if speedCertificateValid && inversionValid
        estimatedSideslip = asin(normalizedYawRate);
        speedErrorContribution = rearAxleDistance ...
            * abs(yawRateMeasured)*velocityErrorMaximum ...
            / (measuredSpeed*trueSpeedLowerBound);
        yawRateAndModelContribution = rearAxleDistance ...
            * (yawRateErrorMaximum+modelYawRateMismatchMaximum) ...
            / trueSpeedLowerBound;
        sineErrorMaximum = speedErrorContribution ...
            + yawRateAndModelContribution;
        sineDomainMaximum = sin(sideslipDomainMaximum);
        sineLower = max( ...
            -sineDomainMaximum, normalizedYawRate-sineErrorMaximum);
        sineUpper = min( ...
            sineDomainMaximum, normalizedYawRate+sineErrorMaximum);
        consistencyTolerance = 128.0*eps(max(1.0, ...
            max(abs([sineLower, sineUpper, normalizedYawRate]))));
        boundsConsistent = sineLower <= sineUpper+consistencyTolerance;
        if boundsConsistent
            if sineLower > sineUpper
                sineMidpoint = 0.5*(sineLower+sineUpper);
                sineLower = sineMidpoint;
                sineUpper = sineMidpoint;
            end
            trueSideslipSineInterval = [sineLower; sineUpper];
            trueSideslipInterval = asin(trueSideslipSineInterval);
            sideslipErrorMaximum = max(abs( ...
                trueSideslipInterval-estimatedSideslip));
        end
    end

    if isfinite(estimatedSideslip)
        measuredBodyDirection = [ ...
            cos(estimatedSideslip); sin(estimatedSideslip)];
    else
        measuredBodyDirection = [1.0; 0.0];
    end
    if boundsConsistent
        correspondence = certifiedRotationCorrespondence( ...
            gnssVelocity, measuredBodyDirection, ...
            velocityErrorMaximum, 0.0, sideslipErrorMaximum);
    else
        correspondence = certifiedRotationCorrespondence( ...
            gnssVelocity, measuredBodyDirection,velocityErrorMaximum,0.0,0.0);
        correspondence.radius = Inf;
        correspondence.informative = false;
        correspondence.rawRadius = Inf;
        correspondence.modelAngleMaximum = Inf;
    end

    course = struct( ...
        "correspondence", correspondence, ...
        "measuredSpeed", measuredSpeed, ...
        "trueSpeedLowerBound", trueSpeedLowerBound, ...
        "speedCertificateValid", speedCertificateValid, ...
        "normalizedYawRate", normalizedYawRate, ...
        "estimatedSideslip", estimatedSideslip, ...
        "speedErrorContribution", speedErrorContribution, ...
        "yawRateAndModelContribution", ...
            yawRateAndModelContribution, ...
        "sineErrorMaximum", sineErrorMaximum, ...
        "trueSideslipSineInterval", ...
            trueSideslipSineInterval, ...
        "trueSideslipInterval", trueSideslipInterval, ...
        "sideslipErrorMaximum", sideslipErrorMaximum, ...
        "inversionValid", inversionValid, ...
        "boundsConsistent", boundsConsistent, ...
        "rearAxleDistance", rearAxleDistance, ...
        "velocityErrorMaximum", velocityErrorMaximum, ...
        "yawRateErrorMaximum", yawRateErrorMaximum, ...
        "modelYawRateMismatchMaximum", ...
            modelYawRateMismatchMaximum, ...
        "sideslipDomainMaximum", sideslipDomainMaximum);
    cachedInputs = inputs;
    cachedCourse = course;
end

function vector = localPlanarVector(vector, name)
    vector = double(vector);
    if ~isequal(size(vector), [2, 1]) || any(~isfinite(vector))
        error("certifiedKinematicCourseCorrespondence:invalidVector", ...
            "%s must be a finite two-vector.", name);
    end
end

function value = localFiniteScalar(value, name)
    value = double(value);
    if ~isscalar(value) || ~isfinite(value)
        error("certifiedKinematicCourseCorrespondence:invalidScalar", ...
            "%s must be a finite scalar.", name);
    end
end

function value = localPositiveScalar(value, name)
    value = localFiniteScalar(value, name);
    if value <= 0.0
        error("certifiedKinematicCourseCorrespondence:invalidPositive", ...
            "%s must be strictly positive.", name);
    end
end

function value = localNonnegativeScalar(value, name)
    value = localFiniteScalar(value, name);
    if value < 0.0
        error("certifiedKinematicCourseCorrespondence:" ...
            + "invalidNonnegative", ...
            "%s must be nonnegative.", name);
    end
end
