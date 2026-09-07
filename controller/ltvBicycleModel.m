classdef ltvBicycleModel
    %ltvBicycleModel Held-input bicycle dynamics, horizon prediction and braking schedules.

    methods (Static)
        function prediction = predict(model, storedSchedule)
        %ltvBicycleModel.predict One scheduled bicycle model over the complete plan.
        % Both steering and braking ratio are decisions at every stage. The final
        % zero-speed schedule admits a rest equilibrium. A carried schedule is
        % shifted verbatim, including across the performance/continuation boundary.
        % Error boxes include Cartesian-to-Frenet projection and the integrated
        % effect of bounded continuous disturbances at every prediction node.

            arguments
                model (1,1) struct
                storedSchedule = []
            end
            cfg = model.cfg;
            headSteps = model.horizonSteps;
            stageCount = headSteps+model.tailSteps;
            nodeCount = stageCount+1;
            planCount = model.inputDimension*stageCount;
            sampleTime = model.sampleTime;
            inputGain = modifiedFialaTire.accelerationGain(cfg);
            scheduleShifted = ~isempty(storedSchedule);
            if scheduleShifted
                schedule = storedSchedule;
            else
                schedule = localInitialSchedule(model, nodeCount);
            end
            if numel(schedule.speedProfile) ~= nodeCount ...
                    || numel(schedule.station) ~= nodeCount ...
                    || numel(schedule.curvature) ~= nodeCount ...
                    || any(~isfinite([schedule.speedProfile, ...
                        schedule.station, schedule.curvature])) ...
                    || any(schedule.speedProfile < 0.0) ...
                    || schedule.speedProfile(end) ~= 0.0
                error("collisionAvoidanceController:invalidCertificate", ...
                    "The carried schedule must be finite and end at zero speed.");
            end

            reference = zeros(2, stageCount);
            reference(1, :) = atan((cfg.vehicle.lf+cfg.vehicle.lr)*schedule.curvature(1:end-1)) ...
                .* schedule.speedProfile(1:end-1) ...
                ./ max(schedule.speedProfile(1:end-1), cfg.model.scheduleSpeedFloor);
            reference(2, :) = (diff(schedule.speedProfile)/sampleTime ...
                + longitudinalRoadLoad(schedule.speedProfile(1:end-1), cfg)/cfg.vehicle.m ...
                - model.longitudinalAccelerationBias)/inputGain;
            reference(:, end) = [0.0; -model.longitudinalAccelerationBias/inputGain];
            % Only the linearization anchor stays strictly inside (-1,1).
            % The optimization still admits both physical input endpoints.
            nominalRatio = min(max(reference(2, :), -1.0+sqrt(eps)), 1.0-sqrt(eps));

            stateMatrix = zeros(6, 6, stageCount);
            inputMatrix = zeros(6, 2, stageCount);
            affine = zeros(6, stageCount);
            stateMap = zeros(6, planCount, nodeCount);
            stateOffset = zeros(6, nodeCount);
            stateOffset(:, 1) = model.initialEgoState;
            errorBound = zeros(6, nodeCount);
            measured = model.measuredEgoStateErrorBound(:);
            if isfield(model, "initialCartesianState")
                cartesianState = model.initialCartesianState;
            else
                [position, heading] = laneGeometry.fromFrenet(model.initialEgoState, model.lane);
                cartesianState = [position; heading; model.initialEgoState(4:6)];
            end
            if isfield(model, "initialFrenetErrorBound")
                errorBound(:, 1) = model.initialFrenetErrorBound;
            else
                errorBound(:, 1) = stateUncertainty.toFrenet(cartesianState, measured, model.lane);
            end
            rateRadius = cfg.model.ltvModelErrorRateBound(:) ...
                + cfg.model.plantModelResidualRateBound(:);
            disturbanceBound = zeros(6, stageCount);
            for stageIdx = 1:stageCount
                if stageIdx > 1 && schedule.curvature(stageIdx) == schedule.curvature(stageIdx-1) ...
                        && schedule.speedProfile(stageIdx) == schedule.speedProfile(stageIdx-1) ...
                        && nominalRatio(stageIdx) == nominalRatio(stageIdx-1)
                    stateMatrix(:, :, stageIdx) = stateMatrix(:, :, stageIdx-1);
                    inputMatrix(:, :, stageIdx) = inputMatrix(:, :, stageIdx-1);
                    affine(:, stageIdx) = affine(:, stageIdx-1);
                    disturbanceBound(:, stageIdx) = disturbanceBound(:, stageIdx-1);
                else
                    [stateMatrix(:, :, stageIdx), inputMatrix(:, :, stageIdx), ...
                        affine(:, stageIdx), continuousA] = ltvBicycleModel.stageMatrices( ...
                        schedule.curvature(stageIdx), schedule.speedProfile(stageIdx), ...
                        sampleTime, cfg, nominalRatio(stageIdx), model.longitudinalAccelerationBias);
                    disturbanceBound(:, stageIdx) = stateUncertainty.heldDisturbance( ...
                        continuousA, rateRadius, sampleTime);
                end
                inputRange = 2*stageIdx-1:2*stageIdx;
                stateMap(:, :, stageIdx+1) = ...
                    stateMatrix(:, :, stageIdx)*stateMap(:, :, stageIdx);
                stateMap(:, inputRange, stageIdx+1) = ...
                    stateMap(:, inputRange, stageIdx+1)+inputMatrix(:, :, stageIdx);
                stateOffset(:, stageIdx+1) = ...
                    stateMatrix(:, :, stageIdx)*stateOffset(:, stageIdx)+affine(:, stageIdx);
                errorBound(:, stageIdx+1) = ...
                    abs(stateMatrix(:, :, stageIdx))*errorBound(:, stageIdx) ...
                        + disturbanceBound(:, stageIdx);
            end

            prediction = struct( ...
                "scheduleSpeed", max(schedule.speedProfile(1), cfg.model.scheduleSpeedFloor), ...
                "scheduleShifted", scheduleShifted, ...
                "scheduleSpeedProfile", schedule.speedProfile, ...
                "scheduleBrakingRatio", nominalRatio, ...
                "scheduleStation", schedule.station, ...
                "scheduleCurvature", schedule.curvature, ...
                "referenceInput", reference(:, 1:headSteps), ...
                "referencePlan", reference(:), ...
                "stageMatrixA", stateMatrix, "stageMatrixB", inputMatrix, ...
                "stageAffine", affine, "stageCount", stageCount, ...
                "inputCount", 2*headSteps, "tailSteps", model.tailSteps, ...
                "planCount", planCount, "tailIndex", 2*headSteps+1:planCount, ...
                "headNodeCount", headSteps+1, "nodeCount", nodeCount, ...
                "tailNodeIndex", headSteps+2:nodeCount, ...
                "egoStateMatrix", stateMap, "egoStateOffset", stateOffset, ...
                "egoStateErrorBound", errorBound, ...
                "stageDisturbanceErrorBound", disturbanceBound, "scheduleForStore", schedule);
        end

        function [stateMatrix, inputMatrix, affineVector, continuousA] = ...
                stageMatrices(kappa, vBar, sampleTime, cfg, betaBar, accelerationBias)
        % ltvBicycleModel.stageMatrices Exact held-input flow of a scheduled affine bicycle.
        %
        % State [s; d; ePsi; vx; vy; r], input [deltaF; beta]. Modified Fiala
        % slip and beta derivatives are evaluated at the scheduled operating
        % point. Static loads and the tire-speed denominator are frozen. The
        % acceleration bias enters independently of the beta input column.
            if nargin < 5, betaBar = []; end
            if nargin < 6, accelerationBias = 0.0; end
            [continuousA, continuousB, continuousC] = ...
                ltvBicycleModel.continuousMatrices(kappa, vBar, cfg, betaBar, accelerationBias);
            heldTransition = expm(sampleTime*[continuousA, continuousB, continuousC; ...
                zeros(3, 9)]);
            stateMatrix = heldTransition(1:6, 1:6);
            inputMatrix = heldTransition(1:6, 7:8);
            affineVector = heldTransition(1:6, 9);
        end

        function [continuousA, continuousB, continuousC] = continuousMatrices(kappa, vBar, cfg, betaBar, accelerationBias)
        %continuousMatrices Continuous generator of the scheduled Frenet bicycle.
        % xDot = continuousA*x + continuousB*u + continuousC, including the
        % independent declared longitudinal acceleration bias in continuousC.

            arguments
                kappa (1,1) double {mustBeFinite}
                vBar (1,1) double {mustBeFinite, mustBeNonnegative}
                cfg (1,1) struct
                betaBar = []
                accelerationBias (1,1) double {mustBeReal, mustBeFinite} = 0.0
            end

            mass = cfg.vehicle.m;
            yawInertia = cfg.vehicle.Iz;
            lf = cfg.vehicle.lf;
            lr = cfg.vehicle.lr;
            inputGain = modifiedFialaTire.accelerationGain(cfg);
            if isempty(betaBar)
                betaBar = (longitudinalRoadLoad(vBar, cfg)/mass-accelerationBias)/inputGain;
                betaBar = min(max(betaBar, -1.0+sqrt(eps)), 1.0-sqrt(eps));
            end
            [tireSlope, ratioSlope, tireIntercept] = ...
                modifiedFialaTire.linearize(kappa, vBar, betaBar, cfg);
            corneringFront = -tireSlope(1);
            corneringRear = -tireSlope(2);
            % At zero schedule speed the rest state is invariant. Only the tire
            % denominator is regularized; kinematic transport uses vBar itself.
            tireSpeed = max(vBar, cfg.model.scheduleSpeedFloor);
            rBar = kappa*vBar;

            continuousA = zeros(6, 6);
            continuousB = zeros(6, 2);
            continuousC = zeros(6, 1);
            % Path kinematics linearized at (d = 0, ePsi = 0, vy = 0, vx = vBar):
            % every affine term of these rows vanishes at the schedule point.
            continuousA(1, 4) = 1.0;
            continuousA(1, 2) = kappa*vBar;
            continuousA(2, 3) = vBar;
            continuousA(2, 5) = 1.0;
            continuousA(3, 6) = 1.0;
            continuousA(3, 4) = -kappa;
            continuousA(3, 2) = -kappa^2*vBar;
            % Linearize passive road load at the scheduled speed, retaining
            % both its slope and affine intercept in the held-input flow.
            [roadForce, roadSlope] = longitudinalRoadLoad(vBar, cfg);
            continuousA(4, 4) = -roadSlope/mass;
            continuousA(4, 5) = rBar;
            continuousB(4, 2) = inputGain;
            continuousC(4) = (roadSlope*vBar-roadForce)/mass+accelerationBias;
            % Lateral channel at the frozen speed.
            yawStiffness = (lf*corneringFront-lr*corneringRear)/tireSpeed;
            lateralStiffness = (corneringFront+corneringRear)/tireSpeed;
            continuousA(5, 4) = -rBar;
            continuousA(5, 5) = -lateralStiffness/mass;
            continuousA(5, 6) = -(yawStiffness/mass+vBar);
            continuousB(5, 1) = corneringFront/mass;
            continuousB(5, 2) = sum(ratioSlope)/mass;
            continuousC(5) = rBar*vBar+sum(tireIntercept)/mass;
            continuousA(6, 5) = -yawStiffness/yawInertia;
            continuousA(6, 6) = -(lf^2*corneringFront ...
                + lr^2*corneringRear)/(tireSpeed*yawInertia);
            continuousB(6, 1) = lf*corneringFront/yawInertia;
            continuousB(6, 2) = (lf*ratioSlope(1)-lr*ratioSlope(2))/yawInertia;
            continuousC(6) = (lf*tireIntercept(1)-lr*tireIntercept(2))/yawInertia;
        end

        function out = brakingSchedule(action, cfg, varargin)
        %ltvBicycleModel.brakingSchedule Initial speed schedule and continuation length.
        % This helper only constructs an admission anchor. Every optimized stage
        % uses ltvBicycleModel.stageMatrices with steering and braking-ratio decisions;
        % there is no kinematic handoff or separate backup vehicle model.
        %
        % steps = ltvBicycleModel.brakingSchedule("steps", cfg)
        % profile = ltvBicycleModel.brakingSchedule("profile", cfg, steps, speed)

            switch string(action)
                case "steps"
                    out = localSteps(cfg);
                case "profile"
                    out = localProfile(cfg, varargin{:});
                otherwise
                    error("collisionAvoidanceController:invalidAction", ...
                        "ltvBicycleModel.brakingSchedule actions are steps and profile.");
            end
        end
    end
end

function schedule = localInitialSchedule(model, nodeCount)
    cfg = model.cfg;
    speed = min(max(model.initialEgoState(4), 0.0), cfg.model.speedMaximum);
    speedProfile = repmat(speed, 1, nodeCount);
    braking = ltvBicycleModel.brakingSchedule("profile", cfg, model.tailSteps, speed);
    speedProfile(model.horizonSteps+2:end) = ...
        max(0.0, speed+model.sampleTime*cumsum(braking));
    speedProfile(end-1:end) = 0.0;
    station = model.initialEgoState(1) ...
        + 0.5*model.sampleTime*[0.0, ...
            cumsum(speedProfile(1:end-1)+speedProfile(2:end))];
    segment = discretize(station, [-inf; model.lane.segmentStation(2:end); inf]);
    curvature = reshape(model.lane.segmentCurvature(segment), 1, []);
    schedule = struct("speed", speed, "station", station, ...
        "speedProfile", speedProfile, "curvature", curvature);
end

function steps = localSteps(cfg)
    sampleTime = cfg.controller.sampleTime;
    deceleration = cfg.terminal.backupDeceleration;
    stoppingStages = ceil(cfg.model.speedMaximum/(deceleration*sampleTime));
    steps = stoppingStages+2;
end

function profile = localProfile(cfg, steps, speed)
    if steps < 2 || steps ~= round(steps)
        error("collisionAvoidanceController:invalidFormulation", ...
            "The braking-tail length must be an integer of at least two.");
    end
    sampleTime = cfg.controller.sampleTime;
    deceleration = cfg.terminal.backupDeceleration;
    tolerance = 1.0e-9;
    remainingSpeed = max(speed, 0.0);
    profile = zeros(1, steps);
    for stageIdx = 1:steps-1
        if remainingSpeed <= tolerance
            break;
        end
        acceleration = -min(deceleration, remainingSpeed/sampleTime);
        profile(stageIdx) = acceleration;
        remainingSpeed = max(remainingSpeed+sampleTime*acceleration, 0.0);
    end
    if remainingSpeed > tolerance
        error("collisionAvoidanceController:invalidFormulation", ...
            "The braking tail cannot reach rest within %d stages " ...
            + "from %.3f m/s.", steps, speed);
    end
end
