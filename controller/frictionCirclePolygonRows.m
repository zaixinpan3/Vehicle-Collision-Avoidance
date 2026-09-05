function [matrix, offset, stageRows, stageOffset] = frictionCirclePolygonRows(prediction, model)
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
    parameters = axleFrictionParameters(cfg);
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
