classdef axleFriction
    %axleFriction Front/rear tire parameters and hard friction-polygon constraints.

    methods (Static)
        function parameters = parameters(cfg)
        % axleFriction.parameters Front/rear data for the Ge-2022 friction maps.
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
        % cfg uses the acceleration bounds validated and normalized by
        % collisionAvoidanceControllerConfig.

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
            minimumAcceleration = cfg.actuation.longitudinalAccelerationMinimum;
            maximumAcceleration = cfg.actuation.longitudinalAccelerationMaximum;
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

        function [matrix, offset, stageRows, stageOffset] = polygonRows(prediction, model)
        % axleFriction.polygonRows Ge-2022 front/rear friction-circle maps.
        %
        % At every MPC stage this function adds TWO independent inscribed-polygon
        % families, one for each bicycle-model axle.  With the paper-aligned input
        %
        %   u = [deltaF; a],
        %
        % the tire forces are
        %
        %   alphaF = (vy + lf*r)/vBar - deltaF,  Fyf = -Cf*alphaF,
        %   alphaR = (vy - lr*r)/vBar,           Fyr = -Cr*alphaR,
        %   Fxi    = rho_i,j*m*a.
        %
        % Following Ge et al. (2022), Eqs. (22)-(33), positive-facing polygon
        % facets use the front-drive distribution rho_d = [1; 0], while
        % negative-facing facets use the configured braking distribution
        % rho_b = [beta; 1-beta].  This facet-wise mapping represents the two
        % one-sided actuator regimes as ONE convex set; it needs neither a sign
        % disjunction nor extra drive/brake inputs.
        %
        % Each axle satisfies the N-edge polygon inscribed in its friction circle,
        %
        %   n_x Fxi + n_y Fyi <= cos(pi/N) * mu_i * Fzi(a).
        %
        % Static Fzi follows the axle geometry.  When the configuration supplies
        % centerOfGravityHeight/cgHeight, Fzi(a) includes affine longitudinal load
        % transfer at the model acceleration gamma*a, where gamma is the declared
        % longitudinal input gain. Full requested Fxi remains in the polygon, so
        % a gain below one does not relax the force-request envelope. Every row is linear.  Four additional rows
        % per stage enforce the configured linear-tire validity limits on alphaF
        % and alphaR.  All rows are nondimensional and HARD.

            cfg = model.cfg;
            horizonSteps = prediction.stageCount;
            inputDimension = model.inputDimension;
            if inputDimension ~= 2
                error("collisionAvoidanceController:invalidFormulation", ...
                    "The Ge-2022 friction map requires input [deltaF; a].");
            end
            % Sized from the prediction, so the rows land in whatever decision
            % block the transcription writes rows over; they still touch only
            % their own stage's two inputs plus that stage's state.
            controlCount = size(prediction.egoStateMatrix, 2);
            parameters = axleFriction.parameters(cfg);
            edgeCount = parameters.edgeCount;
            rowsPerStage = 2 * edgeCount + 4;
            % Local coefficients use [s,d,ePsi,vx,vy,r,deltaF,a]. The same
            % coefficients form the condensed acceptance rows and the sparse SOCP,
            % avoiding a separate physical-constraint implementation in the solver.
            stageRows = zeros(rowsPerStage, 8, horizonSteps);
            stageOffset = zeros(rowsPerStage, horizonSteps);
            speed = max(prediction.scheduleSpeedProfile(1:horizonSteps), ...
                cfg.model.scheduleSpeedFloor);
            inverseSpeed = reshape(1.0./speed, 1, 1, []);
            normal = parameters.edgeNormal;
            for axleIdx = 1:2
                capacity = parameters.staticFrictionForceMaximum(axleIdx);
                cornering = parameters.corneringStiffness(axleIdx);
                lever = cfg.vehicle.lf;
                steering = 1.0;
                if axleIdx == 2
                    lever = -cfg.vehicle.lr;
                    steering = 0.0;
                end
                distribution = parameters.brakeDistribution(axleIdx)*ones(edgeCount, 1);
                distribution(normal(:, 1) >= 0.0) = parameters.driveDistribution(axleIdx);
                range = (axleIdx-1)*edgeCount+(1:edgeCount);
                lateral = -normal(:, 2)*cornering/capacity;
                stageRows(range, 5, :) = lateral.*inverseSpeed;
                stageRows(range, 6, :) = lateral*lever.*inverseSpeed;
                stageRows(range, 7, :) = -lateral*steering.*ones(1, 1, horizonSteps);
                stageRows(range, 8, :) = ((normal(:, 1).*distribution*parameters.mass ...
                    - parameters.inscribedFraction*parameters.frictionCoefficient(axleIdx) ...
                        * parameters.normalLoadAccelerationSlope(axleIdx) ...
                        * cfg.model.longitudinalInputGain)/capacity).*ones(1, 1, horizonSteps);
                stageOffset(range, :) = -parameters.inscribedFraction;
                range = 2*edgeCount+2*(axleIdx-1)+(1:2);
                signs = [1.0; -1.0]/parameters.validitySlipAngleMaximum(axleIdx);
                stageRows(range, 5, :) = signs.*inverseSpeed;
                stageRows(range, 6, :) = signs*lever.*inverseSpeed;
                stageRows(range, 7, :) = -signs*steering.*ones(1, 1, horizonSteps);
                stageOffset(range, :) = -1.0;
            end
            mapped = pagemtimes(stageRows(:, 1:6, :), ...
                prediction.egoStateMatrix(:, :, 1:horizonSteps));
            for stageIdx = 1:horizonSteps
                inputRange = inputDimension*(stageIdx-1)+(1:inputDimension);
                mapped(:, inputRange, stageIdx) = mapped(:, inputRange, stageIdx) ...
                    + stageRows(:, 7:8, stageIdx);
            end
            matrix = reshape(permute(mapped, [1, 3, 2]), [], controlCount);
            mappedOffset = pagemtimes(stageRows(:, 1:6, :), ...
                reshape(prediction.egoStateOffset(:, 1:horizonSteps), 6, 1, []));
            offset = reshape(reshape(mappedOffset, rowsPerStage, [])+stageOffset, [], 1);
        end
    end
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
