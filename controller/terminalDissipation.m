classdef terminalDissipation
    %terminalDissipation Infinite continuation under the declared affine rest input.
    % Nonzero velocity enclosures approach rest while consuming a finite pose
    % budget. The certificate concerns the scheduled model, not an unbounded
    % physical-model residual or an unspecified moving obstacle's future.

    methods (Static)
        function certificate = build(prediction, model)
            cfg = model.cfg;
            input = [0; -model.longitudinalAccelerationBias ...
                /modifiedFialaTire.accelerationGain(cfg)];
            [a, b, c] = ltvBicycleModel.continuousMatrices( ...
                prediction.scheduleCurvature(end), 0, cfg, input(2), ...
                model.longitudinalAccelerationBias);
            velocity = a(4:6, 4:6);
            comparison = abs(velocity);
            comparison(1:4:end) = diag(velocity);
            transport = abs(a(1:3, 4:6));
            stable = max(real(eig(comparison))) < 0;
            excursion = inf(3);
            velocityLimit = zeros(3, 1);
            if stable
                excursion = transport/(-comparison);
                direction = (-comparison)\ones(3, 1);
                slip = [0, 1, cfg.vehicle.lf; 0, 1, -cfg.vehicle.lr] ...
                    /cfg.model.scheduleSpeedFloor;
                scale = min([cfg.model.speedMaximum/direction(1); ...
                    cfg.model.slipAngleMaximum(:)./(abs(slip)*direction)]);
                velocityLimit = scale*direction;
            end
            stationaryPose = all(a(:, 1:3) == 0, "all");
            unforced = all(b*input+c == 0);
            noReverse = velocity(1, 1) < 0 && all(velocity(1, 2:3) == 0);
            disturbanceFree = ~any(cfg.model.ltvModelErrorRateBound) ...
                && ~any(cfg.model.plantModelResidualRateBound);
            fixedSchedule = prediction.scheduleSpeedProfile(end-1) == 0 ...
                && prediction.scheduleCurvature(end-1) == prediction.scheduleCurvature(end);
            certificate = struct("kind", "dissipative-rest-funnel-v1", ...
                "accepted", stable && stationaryPose && unforced && noReverse ...
                    && disturbanceFree && fixedSchedule && cfg.model.speedMinimum == 0, ...
                "continuousA", a, "velocityComparison", comparison, ...
                "poseExcursionMatrix", excursion, "velocityLimit", velocityLimit, ...
                "input", input, "disturbanceFree", disturbanceFree, ...
                "scope", "declaredAffineModelInfiniteTerminalContinuation");
        end

        function [rows, limits] = poseRows(poseRows, poseLimits, radius, certificate)
            % For every pose halfspace a*p<=b, impose
            % a*p + |a|*M*|v| + |a|*r_p + |a|*M*r_v <= b.
            % Enumerating signs is an exact polyhedral representation of |v|.
            signs = 2*double(dec2bin(0:7, 3)-'0')-1;
            count = size(poseRows, 1);
            rows = zeros(8*count, 8);
            limits = zeros(8*count, 1);
            for index = 1:count
                range = 8*(index-1)+(1:8);
                weight = abs(poseRows(index, :))*certificate.poseExcursionMatrix;
                rows(range, 1:3) = repmat(poseRows(index, :), 8, 1);
                rows(range, 4:6) = signs.*weight;
                limits(range) = poseLimits(index)-abs(poseRows(index, :))*radius(1:3) ...
                    -weight*radius(4:6);
            end
        end
    end
end
