function [stateDerivative, estimate] = nrmmTargetTrackerDerivative( ...
        state, ego, domain)
% nrmmTargetTrackerDerivative Evaluate the measured-input NRMM target map.
%
% The target state is x_T = [rho; q; s] with
%
%   rho = R(psiE)'*(pC - pE)   radar relative position, ego frame (m),
%   q   = R(psiE)'*vC          target absolute velocity, ego frame (m/s),
%   s   = R(psiE)'*aC          target absolute acceleration, ego frame (m/s^2),
%
% ordered as state = [rhoX; rhoY; qX; qY; sX; sY]. This is a one-to-one
% coordinate transformation of the Sharma NRMM target model (constant
% scalar-acceleration, constant-sideslip kinematic single track); it is not
% a different target-motion assumption. The exact dynamics are
%
%   rhoDot = q - vE - omegaE*J*rho,
%   qDot   = s - omegaE*J*q,
%   sDot   = Phi(q, s) - omegaE*J*s,
%
% with Phi(q,s) = -Omega^2*q + 3*A*Omega*J*e(q), A = q'*s/|q|,
% Omega = (J*q)'*s/|q|^2, and e(q) = q/|q|. Only the measured or estimated
% ego body velocity and ego yaw rate enter; no ego jerk or ego angular
% acceleration is required, and no derivative of any measured input is
% assumed zero.
%
% Phi is evaluated through a globally Lipschitz saturation extension Phi_e
% that equals Phi on the certified target operating set
% {speedMinimum <= |q| <= speedMaximum, |s| <= accelerationNormBound,
% |A| <= scalarAccelerationMaximum, |Omega| <= yawRateMaximum} and is
% bounded and Lipschitz on all of R^4, so observer peaking cannot evaluate
% the model outside its physical domain.
%
% ego is a scalar struct with fields bodyVelocity (2-by-1, m/s) and
% yawRate (scalar, rad/s). domain is the certified target operating domain
% (see localTargetDomain for the required fields); pass
% design.target.domain from synthesizeNrmmObserverGains.
%
% estimate returns the inverse-transform reconstructions of the Sharma
% target states (speed, scalar acceleration, yaw rate, course, sideslip,
% relative heading) together with saturation and domain-validity flags.

    arguments
        state (6, 1) double {mustBeFinite}
        ego (1, 1) struct
        domain (1, 1) struct
    end

    [speedMinimum, speedMaximum, scalarAccelerationMaximum, ...
        yawRateMaximum, accelerationNormBound, rearAxleDistance, ...
        sideslipMaximum] = localTargetDomain(domain);
    bodyVelocity = double(ego.bodyVelocity(:));
    egoYawRate = double(ego.yawRate);

    planarCross = [0.0, -1.0; 1.0, 0.0];
    relativePosition = state(1:2);
    targetVelocity = state(3:4);
    targetAcceleration = state(5:6);

    % Radial and scalar saturation give bounded, globally Lipschitz factors
    % that equal the original Phi on the certified operating set.
    targetSpeed = norm(targetVelocity);
    speedFloor = max(targetSpeed, speedMinimum);
    unitDirection = targetVelocity/speedFloor;
    speedScale = min(1.0, speedMaximum/max(targetSpeed, realmin));
    qSaturated = speedScale*targetVelocity;
    accelerationRaw = norm(targetAcceleration);
    accelerationScale = min(1.0, ...
        accelerationNormBound/max(accelerationRaw, realmin));
    sSaturated = accelerationScale*targetAcceleration;

    scalarAccelerationRaw = (qSaturated.'*sSaturated)/speedFloor;
    targetScalarAcceleration = min(max(scalarAccelerationRaw, ...
        -scalarAccelerationMaximum), scalarAccelerationMaximum);
    yawRateRaw = ((planarCross*qSaturated).'*sSaturated)/speedFloor^2;
    targetYawRate = min(max(yawRateRaw, ...
        -yawRateMaximum), yawRateMaximum);

    phiValue = -targetYawRate^2*qSaturated ...
        + 3.0*targetScalarAcceleration*targetYawRate ...
            * planarCross*unitDirection;

    stateDerivative = [ ...
        targetVelocity-bodyVelocity ...
            - egoYawRate*planarCross*relativePosition; ...
        targetAcceleration-egoYawRate*planarCross*targetVelocity; ...
        phiValue-egoYawRate*planarCross*targetAcceleration];

    if nargout < 2
        return
    end

    % Reconstruct diagnostics only when the caller requests the estimate.
    saturatedQuantities = strings(0, 1);
    if targetSpeed < speedMinimum
        saturatedQuantities(end+1, 1) = "targetSpeedBelowMinimum";
    end
    if speedScale < 1.0
        saturatedQuantities(end+1, 1) = "targetSpeedAboveMaximum";
    end
    if accelerationScale < 1.0
        saturatedQuantities(end+1, 1) = "targetAccelerationNorm";
    end
    if abs(scalarAccelerationRaw) > scalarAccelerationMaximum
        saturatedQuantities(end+1, 1) = "targetScalarAcceleration";
    end
    if abs(yawRateRaw) > yawRateMaximum
        saturatedQuantities(end+1, 1) = "targetYawRate";
    end
    saturationActive = ~isempty(saturatedQuantities);
    courseAngleEgoFrame = atan2(targetVelocity(2), targetVelocity(1));
    sideslipSine = min(max( ...
        rearAxleDistance*targetYawRate/speedFloor, ...
        -sin(sideslipMaximum)), sin(sideslipMaximum));
    targetSideslip = asin(sideslipSine);
    speedDomainValid = targetSpeed >= speedMinimum;

    estimate = struct( ...
        "relativePosition", relativePosition, ...
        "relativeVelocity", stateDerivative(1:2), ...
        "targetVelocity", targetVelocity, ...
        "targetAcceleration", targetAcceleration, ...
        "targetSpeed", targetSpeed, ...
        "targetScalarAcceleration", targetScalarAcceleration, ...
        "targetYawRate", targetYawRate, ...
        "targetCourseAngleEgoFrame", courseAngleEgoFrame, ...
        "targetSideslip", targetSideslip, ...
        "targetRelativeHeading", courseAngleEgoFrame-targetSideslip, ...
        "phiValue", phiValue, ...
        "speedDomainValid", speedDomainValid, ...
        "operatingDomainValid", speedDomainValid && ~saturationActive, ...
        "saturationActive", saturationActive, ...
        "saturatedQuantities", saturatedQuantities);
end

function [speedMinimum, speedMaximum, scalarAccelerationMaximum, ...
        yawRateMaximum, accelerationNormBound, rearAxleDistance, ...
        sideslipMaximum] = localTargetDomain(domain)
    requiredFields = {'speedMinimum', 'speedMaximum', ...
        'scalarAccelerationMaximum', 'yawRateMaximum', ...
        'accelerationNormBound', 'rearAxleDistance', 'sideslipMaximum'};
    for index = coder.unroll(1:numel(requiredFields))
        fieldName = requiredFields{index};
        if ~isfield(domain, fieldName) ...
                || ~isscalar(domain.(fieldName)) ...
                || ~isfinite(domain.(fieldName))
            error("nrmmTargetTrackerDerivative:invalidDomain", ...
                "domain.%s must be a finite scalar.", fieldName);
        end
    end
    speedMinimum = double(domain.speedMinimum);
    speedMaximum = double(domain.speedMaximum);
    scalarAccelerationMaximum = ...
        double(domain.scalarAccelerationMaximum);
    yawRateMaximum = double(domain.yawRateMaximum);
    accelerationNormBound = double(domain.accelerationNormBound);
    rearAxleDistance = double(domain.rearAxleDistance);
    sideslipMaximum = double(domain.sideslipMaximum);
    if speedMinimum <= 0.0 || speedMaximum <= speedMinimum ...
            || rearAxleDistance <= 0.0
        error("nrmmTargetTrackerDerivative:invalidDomain", ...
            "The target speed interval and rear-axle distance must be positive.");
    end
end
