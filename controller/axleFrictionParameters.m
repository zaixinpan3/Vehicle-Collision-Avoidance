function parameters = axleFrictionParameters(cfg)
% axleFrictionParameters Front/rear data for the Ge-2022 friction maps.
%
% The bicycle model treats each axle as one tire.  Static normal loads are
% set by lf/lr.  If vehicle.centerOfGravityHeight (or vehicle.cgHeight) is
% available, the normal loads also contain the affine longitudinal load
% transfer
%
%   Fzf = m (g lr - a h) / L,   Fzr = m (g lf + a h) / L.
%
% This dependence remains affine in the MPC input a, so the two inscribed
% polygon families remain linear inequalities.  The braking allocation is
% supplied explicitly as actuation.brakingForceDistribution =
% [rho_fb; rho_rb].

    if ~isstruct(cfg) || ~isscalar(cfg) ...
            || ~all(isfield(cfg, ["vehicle", "tire", "model", ...
                "actuation"]))
        error("collisionAvoidanceController:invalidConfiguration", ...
            "cfg must contain vehicle, tire, model, and actuation.");
    end
    mass = localPositiveScalar(cfg.vehicle.m, "vehicle.m");
    gravity = localPositiveScalar(cfg.vehicle.gravity, "vehicle.gravity");
    lf = localPositiveScalar(cfg.vehicle.lf, "vehicle.lf");
    lr = localPositiveScalar(cfg.vehicle.lr, "vehicle.lr");
    wheelbase = lf + lr;
    corneringStiffness = localAxleVector( ...
        cfg.tire.corneringStiffness, "tire.corneringStiffness", false);
    frictionCoefficient = localAxleVector( ...
        cfg.tire.frictionCoefficient, "tire.frictionCoefficient", true);
    validitySlipLimit = localAxleVector( ...
        cfg.model.slipAngleMaximum, "model.slipAngleMaximum", false);
    if ~isfield(cfg.actuation, "brakingForceDistribution")
        error("collisionAvoidanceController:invalidConfiguration", ...
            "cfg.actuation.brakingForceDistribution must explicitly " ...
            + "provide [rho_fb; rho_rb].");
    end
    brakeDistribution = localForceDistribution( ...
        cfg.actuation.brakingForceDistribution, ...
        "actuation.brakingForceDistribution");

    cgHeight = 0.0;
    if isfield(cfg.vehicle, "centerOfGravityHeight")
        cgHeight = localNonnegativeScalar( ...
            cfg.vehicle.centerOfGravityHeight, ...
            "vehicle.centerOfGravityHeight");
    elseif isfield(cfg.vehicle, "cgHeight")
        cgHeight = localNonnegativeScalar( ...
            cfg.vehicle.cgHeight, "vehicle.cgHeight");
    end

    staticNormalLoad = mass * gravity / wheelbase * [lr; lf];
    normalLoadAccelerationSlope = mass * cgHeight / wheelbase * [-1; 1];
    [minimumAcceleration, maximumAcceleration] = ...
        longitudinalAccelerationBounds(cfg);
    minimumNormalLoad = staticNormalLoad + [ ...
        normalLoadAccelerationSlope(1) * maximumAcceleration; ...
        normalLoadAccelerationSlope(2) * minimumAcceleration];
    if any(minimumNormalLoad <= 0.0)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "The configured acceleration envelope permits axle lift " ...
            + "under the affine load-transfer model.");
    end

    staticFrictionForce = frictionCoefficient .* staticNormalLoad;
    edgeCount = localIntegerScalar( ...
        cfg.model.frictionPolygonEdgeCount, ...
        "model.frictionPolygonEdgeCount");
    if edgeCount < 4 || mod(edgeCount, 2) ~= 0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "model.frictionPolygonEdgeCount must be an even integer >= 4.");
    end
    % Ge et al. Eq. (25) for N = 8, generalized to an even N.  Facet
    % normals with positive longitudinal component use the drive shares;
    % negative-facing facets use the braking shares (their Eqs. 28-29).
    edgeAngle = (0:edgeCount - 1).' * (2.0 * pi / edgeCount) ...
        - pi / 2.0 + pi / edgeCount;
    edgeNormal = [cos(edgeAngle), sin(edgeAngle)];

    parameters = struct( ...
        "mass", mass, ...
        "corneringStiffness", corneringStiffness, ...
        "frictionCoefficient", frictionCoefficient, ...
        "validitySlipAngleMaximum", validitySlipLimit, ...
        "staticNormalLoad", staticNormalLoad, ...
        "normalLoadAccelerationSlope", normalLoadAccelerationSlope, ...
        "staticFrictionForceMaximum", staticFrictionForce, ...
        "driveDistribution", [1.0; 0.0], ...
        "brakeDistribution", brakeDistribution, ...
        "edgeCount", edgeCount, ...
        "edgeNormal", edgeNormal, ...
        "inscribedFraction", cos(pi / edgeCount));
end

function distribution = localForceDistribution(value, fieldName)
    if ~isnumeric(value) || ~isreal(value) || numel(value) ~= 2 ...
            || any(~isfinite(value), "all") || any(value < 0.0, "all")
        error("collisionAvoidanceController:invalidConfiguration", ...
            "%s must contain two nonnegative finite values.", fieldName);
    end
    distribution = double(value(:));
    sumTolerance = 100.0 * eps(max(1.0, max(distribution)));
    if abs(sum(distribution) - 1.0) > sumTolerance
        error("collisionAvoidanceController:invalidConfiguration", ...
            "%s must sum to one in [front; rear] order.", fieldName);
    end
end

function vector = localAxleVector(value, fieldName, allowFourWheels)
    if ~isnumeric(value) || ~isreal(value) || isempty(value) ...
            || any(~isfinite(value), "all") || any(value <= 0.0, "all")
        error("collisionAvoidanceController:invalidConfiguration", ...
            "%s must contain positive finite values.", fieldName);
    end
    value = double(value(:));
    if isscalar(value)
        vector = repmat(value, 2, 1);
    elseif numel(value) == 2
        vector = value;
    elseif allowFourWheels && numel(value) == 4
        vector = [min(value(1:2)); min(value(3:4))];
    else
        error("collisionAvoidanceController:invalidConfiguration", ...
            "%s must be scalar or contain front/rear values%s.", ...
            fieldName, localFourWheelSuffix(allowFourWheels));
    end
end

function suffix = localFourWheelSuffix(allowFourWheels)
    if allowFourWheels
        suffix = " (four per-wheel values are also accepted)";
    else
        suffix = "";
    end
end

function value = localFiniteScalar(value, fieldName)
    if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
            || ~isfinite(value)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "%s must be a finite real scalar.", fieldName);
    end
    value = double(value);
end

function value = localPositiveScalar(value, fieldName)
    value = localFiniteScalar(value, fieldName);
    if value <= 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "%s must be positive.", fieldName);
    end
end

function value = localNonnegativeScalar(value, fieldName)
    value = localFiniteScalar(value, fieldName);
    if value < 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "%s must be nonnegative.", fieldName);
    end
end

function value = localIntegerScalar(value, fieldName)
    value = localPositiveScalar(value, fieldName);
    if value ~= floor(value)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "%s must be an integer.", fieldName);
    end
end
