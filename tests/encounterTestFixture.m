classdef encounterTestFixture
    %encounterTestFixture Declared-model fixtures, not physical validation data.
    methods (Static)
        function [ego, target, route, cfg] = crossing()
            cfg = collisionAvoidanceControllerConfig(struct("controller", ...
                struct("horizonSteps", 16, "sampleTime", 0.1, "certifiedSteps", Inf), ...
                "model", struct("lateralDomainRadius", 2,"linearizationPolicy","cruise"), "referenceSpeed", 8));
            ego = struct("position", [0; 0], "yaw", 0, "speed", 8, "stateTime", 0);
            cfg.encounter.maneuverSelectionPolicy = "bestFeasibleObjective";
            cfg.encounter.safetyMarginPolicy = "performance";
            route = [-100, 0; 2000, 0];
            contract = struct("kind", "cartesian-jerk-exit-v1", "id", "crossing-1", ...
                "validFrom", 0, "validUntil", 1.6, "jerkBound", [0; 0], ...
                "yawAccelerationBound", 0, "exitNormal", [0; 1], "exitOffset", 4.6, ...
                "postExitRoute", "nonreturningHalfspace");
            target = struct("trackId", 1, "targetPositionInertial", [15; -4], ...
                "targetVelocityInertial", [0; 8], "targetAccelerationInertial", [0; 0], ...
                "targetHeadingInertial", pi/2, "targetYawRate", 0, "encounterContract", contract);
        end

        function ego = nextEgo(stored, lane)
            state = stored.predictedState(:, 2);
            [position, heading] = laneGeometry.fromFrenet(state, lane);
            ego = struct("position", position, "yaw", heading, "speed", state(4), ...
                "lateralVelocity", state(5), "yawRate", state(6), ...
                "stateTime", stored.stateTime+stored.identity.configuration.controller.sampleTime, ...
                "heldActuatorInput", stored.appliedInput);
        end

        function result = fail(~, ~)
            result = struct("decision", [], "exitFlag", -999, "output", struct());
        end

        function result = unsafe(~, program)
            result = program.defaultSolver();
            result.decision(1) = 2;
            result.exitFlag = 1;
        end
    end
end
