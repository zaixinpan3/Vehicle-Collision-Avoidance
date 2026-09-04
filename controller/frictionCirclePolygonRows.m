function [matrix, offset] = frictionCirclePolygonRows(prediction, model)
% frictionCirclePolygonRows Ge-2022 front/rear friction-circle maps.
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
% transfer, which preserves linearity of every row.  Four additional rows
% per stage enforce the configured linear-tire validity limits on alphaF
% and alphaR.  All rows are nondimensional and HARD.

    cfg = model.cfg;
    horizonSteps = model.horizonSteps;
    inputDimension = model.inputDimension;
    if inputDimension ~= 2
        error("collisionAvoidanceController:invalidFormulation", ...
            "The Ge-2022 friction map requires input [deltaF; a].");
    end
    % Sized from the prediction, so the rows land in whatever decision
    % block the transcription writes rows over; they still touch only
    % their own stage's two inputs plus that stage's state.
    controlCount = size(prediction.egoStateMatrix, 2);
    parameters = axleFrictionParameters(cfg);
    edgeCount = parameters.edgeCount;
    rowsPerStage = 2 * edgeCount + 4;
    matrix = zeros(rowsPerStage * horizonSteps, controlCount);
    offset = zeros(rowsPerStage * horizonSteps, 1);

    mass = parameters.mass;
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;
    corneringFront = parameters.corneringStiffness(1);
    corneringRear = parameters.corneringStiffness(2);
    vBar = prediction.scheduleSpeed;
    if ~isnumeric(vBar) || ~isreal(vBar) || ~isscalar(vBar) ...
            || ~isfinite(vBar) || vBar <= 0.0
        error("collisionAvoidanceController:invalidFormulation", ...
            "prediction.scheduleSpeed must be a positive scalar.");
    end

    for stageIdx = 1:horizonSteps
        steeringRow = zeros(1, controlCount);
        steeringRow(inputDimension * (stageIdx - 1) + 1) = 1.0;
        accelerationRow = zeros(1, controlCount);
        accelerationRow(inputDimension * (stageIdx - 1) + 2) = 1.0;
        stateMatrix = prediction.egoStateMatrix(:, :, stageIdx);
        stateOffset = prediction.egoStateOffset(:, stageIdx);

        % Paper Eq. (8): tire sideslip (not lateral-force sign convention).
        alphaFrontRow = (stateMatrix(5, :) ...
            + lf * stateMatrix(6, :)) / vBar - steeringRow;
        alphaFrontOffset = ...
            (stateOffset(5) + lf * stateOffset(6)) / vBar;
        alphaRearRow = (stateMatrix(5, :) ...
            - lr * stateMatrix(6, :)) / vBar;
        alphaRearOffset = ...
            (stateOffset(5) - lr * stateOffset(6)) / vBar;
        lateralForceRow = [ ...
            -corneringFront * alphaFrontRow; ...
            -corneringRear * alphaRearRow];
        lateralForceOffset = [ ...
            -corneringFront * alphaFrontOffset; ...
            -corneringRear * alphaRearOffset];

        rowIdx = rowsPerStage * (stageIdx - 1);
        for axleIdx = 1:2
            staticCapacity = ...
                parameters.staticFrictionForceMaximum(axleIdx);
            normalLoadSlope = ...
                parameters.normalLoadAccelerationSlope(axleIdx);
            mu = parameters.frictionCoefficient(axleIdx);
            for edgeIdx = 1:edgeCount
                normal = parameters.edgeNormal(edgeIdx, :);
                if normal(1) >= 0.0
                    distribution = ...
                        parameters.driveDistribution(axleIdx);
                else
                    distribution = ...
                        parameters.brakeDistribution(axleIdx);
                end
                longitudinalForceRow = ...
                    distribution * mass * accelerationRow;
                % Move the load-dependent polygon radius to the left:
                % n'F - tau*mu*(Fz0 + dFz/da*a) <= 0.
                rowIdx = rowIdx + 1;
                matrix(rowIdx, :) = ( ...
                    normal(1) * longitudinalForceRow ...
                    + normal(2) * lateralForceRow(axleIdx, :) ...
                    - parameters.inscribedFraction * mu ...
                        * normalLoadSlope * accelerationRow) ...
                    / staticCapacity;
                offset(rowIdx) = ( ...
                    normal(2) * lateralForceOffset(axleIdx) ...
                    - parameters.inscribedFraction * staticCapacity) ...
                    / staticCapacity;
            end
        end

        slipLimit = parameters.validitySlipAngleMaximum;
        rowIdx = rowIdx + 1;
        matrix(rowIdx, :) = alphaFrontRow / slipLimit(1);
        offset(rowIdx) = ...
            (alphaFrontOffset - slipLimit(1)) / slipLimit(1);
        rowIdx = rowIdx + 1;
        matrix(rowIdx, :) = -alphaFrontRow / slipLimit(1);
        offset(rowIdx) = ...
            (-alphaFrontOffset - slipLimit(1)) / slipLimit(1);
        rowIdx = rowIdx + 1;
        matrix(rowIdx, :) = alphaRearRow / slipLimit(2);
        offset(rowIdx) = ...
            (alphaRearOffset - slipLimit(2)) / slipLimit(2);
        rowIdx = rowIdx + 1;
        matrix(rowIdx, :) = -alphaRearRow / slipLimit(2);
        offset(rowIdx) = ...
            (-alphaRearOffset - slipLimit(2)) / slipLimit(2);
    end
end
