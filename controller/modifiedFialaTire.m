classdef modifiedFialaTire
    %modifiedFialaTire Fahmy et al. (2018), Eqs. (7), (11) and (13), by axle.
    % Static loads define Fx_i = beta*mu_i*Fz_i. Both slip-angle and beta
    % derivatives enter the local affine model, without friction-limit rows.

    methods (Static)
        function parameters = parameters(cfg)
            mass = localPositiveScalar(cfg.vehicle.m, "vehicle.m");
            gravity = localPositiveScalar(cfg.vehicle.gravity, "vehicle.gravity");
            lf = localPositiveScalar(cfg.vehicle.lf, "vehicle.lf");
            lr = localPositiveScalar(cfg.vehicle.lr, "vehicle.lr");
            cornering = localAxleVector(cfg.tire.corneringStiffness, "tire.corneringStiffness");
            friction = localAxleVector(cfg.tire.frictionCoefficient, "tire.frictionCoefficient");
            normalLoad = mass*gravity/(lf+lr)*[lr; lf];
            forceScale = friction.*normalLoad;
            parameters = struct("corneringStiffness", cornering, ...
                "frictionCoefficient", friction, "staticNormalLoad", normalLoad, ...
                "longitudinalForceScale", forceScale, ...
                "brakingRatioAccelerationGain", sum(forceScale)/mass);
        end

        function gain = accelerationGain(cfg)
            parameters = modifiedFialaTire.parameters(cfg);
            gain = parameters.brakingRatioAccelerationGain;
        end

        function force = longitudinalForce(brakingRatio, cfg)
            arguments
                brakingRatio (1,1) double {mustBeReal, mustBeFinite}
                cfg (1,1) struct
            end
            parameters = modifiedFialaTire.parameters(cfg);
            force = brakingRatio*parameters.longitudinalForceScale;
        end

        function [force, slipSlope, ratioSlope, intercept] = evaluate(slipAngle, brakingRatio, cfg)
        %evaluate Force [N], dFy/dAlpha [N/rad], dFy/dBeta [N], intercept [N].
        % Use the symmetric |alpha| branch and Eq. (13), eta=sqrt(1-beta^2).
        % Force is defined at beta=+/-1, but no finite joint tangent exists
        % there. Derivative requests require an interior operating point.
            arguments
                slipAngle (2,1) double {mustBeReal, mustBeFinite}
                brakingRatio (1,1) double {mustBeReal, mustBeFinite}
                cfg (1,1) struct
            end
            if any(abs(slipAngle) >= pi/2) || abs(brakingRatio) > 1.0
                error("collisionAvoidanceController:invalidTireOperatingPoint", ...
                    "Fiala force requires abs(alpha)<pi/2 and abs(beta)<=1.");
            end
            if nargout > 1 && abs(brakingRatio) == 1.0
                error("collisionAvoidanceController:singularTireLinearization", ...
                    "A Fiala tangent requires abs(beta)<1; the endpoint force is still defined.");
            end
            parameters = modifiedFialaTire.parameters(cfg);
            cornering = parameters.corneringStiffness;
            capacity = parameters.longitudinalForceScale;
            eta = sqrt(1.0-brakingRatio^2);
            lateralScale = capacity*eta;
            tangent = tan(slipAngle);
            force = -lateralScale.*sign(slipAngle);
            slipSlope = zeros(2, 1);
            ratioSlope = zeros(2, 1);
            adhesion = lateralScale > 0.0 & abs(tangent) < 3*lateralScale./cornering;
            q = cornering(adhesion).*abs(tangent(adhesion))./(3*lateralScale(adhesion));
            force(adhesion) = -cornering(adhesion).*tangent(adhesion) ...
                .* (1.0-q+q.^2/3.0);
            if nargout > 1
                slipSlope(adhesion) = -cornering(adhesion).*(1.0-q).^2 ...
                    .* (1.0+tangent(adhesion).^2);
                ratioSlope = capacity*(brakingRatio/eta).*sign(slipAngle);
                ratioSlope(adhesion) = cornering(adhesion).*tangent(adhesion) ...
                    .*q.*(1.0-2.0*q/3.0)*(brakingRatio/eta^2);
            end
            intercept = force-slipSlope.*slipAngle-ratioSlope*brakingRatio;
        end

        function [slipSlope, ratioSlope, intercept, nominalSlip, nominalForce] = ...
                linearize(kappa, speed, brakingRatio, cfg)
        %linearize Tangent about the scheduled route-following operating point.
        % Transport speed is not floored. Steering tends to zero with speed,
        % retaining the exact rest equilibrium on curved roads.
            tireSpeed = max(speed, cfg.model.scheduleSpeedFloor);
            yawRate = kappa*speed;
            steering = atan((cfg.vehicle.lf+cfg.vehicle.lr)*kappa)*speed/tireSpeed;
            nominalSlip = [(cfg.vehicle.lf*yawRate)/tireSpeed-steering; ...
                -cfg.vehicle.lr*yawRate/tireSpeed];
            [nominalForce, slipSlope, ratioSlope, intercept] = ...
                modifiedFialaTire.evaluate(nominalSlip, brakingRatio, cfg);
        end
    end
end

function value = localPositiveScalar(value, name)
    if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
            || ~isfinite(value) || value <= 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "%s must be a positive finite scalar.", name);
    end
    value = double(value);
end

function value = localAxleVector(value, name)
    if ~isnumeric(value) || ~isreal(value) || ~isvector(value) ...
            || ~ismember(numel(value), [1, 2]) ...
            || any(~isfinite(value), "all") || any(value <= 0.0, "all")
        error("collisionAvoidanceController:invalidConfiguration", ...
            "%s must be positive and scalar or contain [front; rear] values.", name);
    end
    value = double(value(:));
    if isscalar(value)
        value = repmat(value, 2, 1);
    end
end
