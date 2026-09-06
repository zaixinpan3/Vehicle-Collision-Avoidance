classdef ltvBicycleModel
    %ltvBicycleModel Held-input bicycle dynamics, horizon prediction and braking schedules.

    methods (Static)
        function prediction = predict(model, storedSchedule)
        %ltvBicycleModel.predict One scheduled bicycle model over the complete plan.
        % Both steering and acceleration are decisions at every stage. The final
        % zero-speed schedule admits a rest equilibrium. A carried schedule is
        % shifted verbatim, including across the performance/continuation boundary.
        % Error boxes obey r(k+1) = abs(A(k))*r(k) + Ts*w at every node.

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
            inputGain = cfg.model.longitudinalInputGain;
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

            stateMatrix = zeros(6, 6, stageCount);
            inputMatrix = zeros(6, 2, stageCount);
            affine = zeros(6, stageCount);
            stateMap = zeros(6, planCount, nodeCount);
            stateOffset = zeros(6, nodeCount);
            stateOffset(:, 1) = model.initialEgoState;
            errorBound = zeros(6, nodeCount);
            measured = model.measuredEgoStateErrorBound(:);
            errorBound(:, 1) = [sum(measured(1:2)); sum(measured(1:2)); measured(3:6)];
            disturbance = sampleTime*(cfg.model.ltvModelErrorRateBound(:) ...
                + cfg.model.plantModelResidualRateBound(:));
            for stageIdx = 1:stageCount
                if stageIdx > 1 && schedule.curvature(stageIdx) == schedule.curvature(stageIdx-1) ...
                        && schedule.speedProfile(stageIdx) == schedule.speedProfile(stageIdx-1)
                    stateMatrix(:, :, stageIdx) = stateMatrix(:, :, stageIdx-1);
                    inputMatrix(:, :, stageIdx) = inputMatrix(:, :, stageIdx-1);
                    affine(:, stageIdx) = affine(:, stageIdx-1);
                else
                    [stateMatrix(:, :, stageIdx), inputMatrix(:, :, stageIdx), ...
                        affine(:, stageIdx)] = ltvBicycleModel.stageMatrices( ...
                        schedule.curvature(stageIdx), schedule.speedProfile(stageIdx), ...
                        sampleTime, cfg);
                    affine(:, stageIdx) = affine(:, stageIdx) ...
                        + inputMatrix(:, 2, stageIdx)*(model.longitudinalAccelerationBias/inputGain);
                end
                inputRange = 2*stageIdx-1:2*stageIdx;
                stateMap(:, :, stageIdx+1) = ...
                    stateMatrix(:, :, stageIdx)*stateMap(:, :, stageIdx);
                stateMap(:, inputRange, stageIdx+1) = ...
                    stateMap(:, inputRange, stageIdx+1)+inputMatrix(:, :, stageIdx);
                stateOffset(:, stageIdx+1) = ...
                    stateMatrix(:, :, stageIdx)*stateOffset(:, stageIdx)+affine(:, stageIdx);
                errorBound(:, stageIdx+1) = ...
                    abs(stateMatrix(:, :, stageIdx))*errorBound(:, stageIdx)+disturbance;
            end

            reference = zeros(2, stageCount);
            reference(1, :) = atan(cfg.vehicle.wheelbase*schedule.curvature(1:end-1));
            reference(2, :) = (diff(schedule.speedProfile)/sampleTime ...
                - model.longitudinalAccelerationBias)/inputGain;
            reference(:, end) = [0.0; -model.longitudinalAccelerationBias/inputGain];
            prediction = struct( ...
                "scheduleSpeed", max(schedule.speedProfile(1), cfg.model.scheduleSpeedFloor), ...
                "scheduleShifted", scheduleShifted, ...
                "scheduleSpeedProfile", schedule.speedProfile, ...
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
                "egoStateErrorBound", errorBound, "scheduleForStore", schedule);
        end

        function [stateMatrix, inputMatrix, affineVector] = ...
                stageMatrices(kappa, vBar, sampleTime, cfg)
        % ltvBicycleModel.stageMatrices Exact held-input flow of a scheduled affine bicycle.
        %
        % Closed-form linearization of the dynamic bicycle with linear
        % cornering regularized at a positive tire-speed floor, in path coordinates
        % along the lane centerline -
        % state [s; d; ePsi; vx; vy; r] with s the station, d the left-positive
        % lateral offset and ePsi the heading error to the path tangent - about
        % the schedule point (d = 0, ePsi = 0, vy = 0, vx = vBar, r = kappa*vBar)
        % at the local curvature kappa. One block matrix exponential integrates
        % the affine model with constant input over the sample. This is exact for
        % that scheduled linearization, not for the nonlinear bicycle or plant.
        %
        % Continuous model:
        %   sdot    = (vx cos ePsi - vy sin ePsi)/(1 - kappa d)
        %   ddot    = vx sin ePsi + vy cos ePsi
        %   ePsidot = r - kappa sdot
        %   vxdot   = gamma*a + vy r, gamma = cfg.model.longitudinalInputGain
        %   vydot   = (Fyf + Fyr)/m - vx r
        %   rdot    = (lf Fyf - lr Fyr)/Iz
        %   Fyf = Cf (deltaF - (vy + lf r)/vTire), Fyr = -Cr (vy - lr r)/vTire
        %   vTire = max(vBar, cfg.model.scheduleSpeedFloor)
        %
        % Input [deltaF; a] uses commanded acceleration, with a fixed declared
        % longitudinal effectiveness gain. The default gain is one. The curvature is treated as
        % locally constant at the schedule station of the stage (its variation
        % along the horizon is carried node by node by the schedule). Prediction
        % and continuation use this held-input flow; the CLF uses its continuous
        % generator through continuousMatrices.
        % Position and heading can therefore depend on the new first input. The
        % discretization does not impose a forward-Euler stiffness restriction.

            [continuousA, continuousB, continuousC] = ...
                ltvBicycleModel.continuousMatrices(kappa, vBar, cfg);
            heldTransition = expm(sampleTime*[continuousA, continuousB, continuousC; ...
                zeros(3, 9)]);
            stateMatrix = heldTransition(1:6, 1:6);
            inputMatrix = heldTransition(1:6, 7:8);
            affineVector = heldTransition(1:6, 9);
        end

        function [continuousA, continuousB, continuousC] = continuousMatrices(kappa, vBar, cfg)
        %continuousMatrices Continuous generator of the scheduled Frenet bicycle.
        % xDot = continuousA*x + continuousB*u + continuousC. The caller
        % adds the declared longitudinal acceleration bias to xDot(4).

            arguments
                kappa (1,1) double {mustBeFinite}
                vBar (1,1) double {mustBeFinite, mustBeNonnegative}
                cfg (1,1) struct
            end

            mass = cfg.vehicle.m;
            yawInertia = cfg.vehicle.Iz;
            lf = cfg.vehicle.lf;
            lr = cfg.vehicle.lr;
            corneringStiffness = double(cfg.tire.corneringStiffness(:));
            if isscalar(corneringStiffness)
                corneringStiffness = repmat(corneringStiffness, 2, 1);
            end
            if numel(corneringStiffness) ~= 2 ...
                    || any(~isfinite(corneringStiffness)) ...
                    || any(corneringStiffness <= 0.0)
                error("collisionAvoidanceController:invalidConfiguration", ...
                    "tire.corneringStiffness must be positive and scalar or " ...
                    + "contain front/rear values.");
            end
            corneringFront = corneringStiffness(1);
            corneringRear = corneringStiffness(2);
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
            % vxdot = gamma*a + vy*r, frozen at (vy = 0, r = rBar).
            continuousA(4, 5) = rBar;
            continuousB(4, 2) = cfg.model.longitudinalInputGain;
            % Lateral channel at the frozen speed.
            yawStiffness = (lf*corneringFront-lr*corneringRear)/tireSpeed;
            lateralStiffness = (corneringFront+corneringRear)/tireSpeed;
            continuousA(5, 4) = -rBar;
            continuousA(5, 5) = -lateralStiffness/mass;
            continuousA(5, 6) = -(yawStiffness/mass+vBar);
            continuousB(5, 1) = corneringFront/mass;
            continuousC(5) = rBar*vBar;
            continuousA(6, 5) = -yawStiffness/yawInertia;
            continuousA(6, 6) = -(lf^2*corneringFront ...
                + lr^2*corneringRear)/(tireSpeed*yawInertia);
            continuousB(6, 1) = lf*corneringFront/yawInertia;
        end

        function out = brakingSchedule(action, cfg, varargin)
        %ltvBicycleModel.brakingSchedule Initial speed schedule and continuation length.
        % This helper only constructs an admission anchor. Every optimized stage
        % uses ltvBicycleModel.stageMatrices with steering and acceleration decisions;
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
