classdef collisionRectangleEnvelopeTest < matlab.unittest.TestCase
% collisionRectangleEnvelopeTest Behavior tests for rectangle geometry.

    methods (TestClassSetup)
        function addRepositoryToPath(testCase)
            repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                repositoryRoot));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repositoryRoot, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repositoryRoot, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repositoryRoot, "scripts")));
        end
    end

    methods (TestMethodSetup)
        function resetControllerNominalTrajectory(~)
            collisionAvoidanceController("resetNominalTrajectory");
        end
    end

    methods (Test)
        function separatedRectanglesHavePositiveSignedDistance(testCase)
            cfg = localConfig();
            expectedDistance = 0.25;
            target = localTarget(20.0, 0.0, 0.0);

            [~, ~, problem] = collisionAvoidanceController( ...
                localEgo(), target, localLane(), cfg);
            signedDistance = problem.functions.rectangleSignedDistance( ...
                [0.0; 0.0], 0.0, ...
                [cfg.vehicle.length + expectedDistance; 0.0], 0.0);

            testCase.verifyEqual(signedDistance, ...
                expectedDistance, AbsTol=1.0e-12);
            testCase.verifyEqual(max(signedDistance, 0.0), ...
                expectedDistance, AbsTol=1.0e-12);
            testCase.verifyGreaterThan(signedDistance, 0.0);
        end

        function tangentRectanglesAreNotStrictlySeparated(testCase)
            cfg = localConfig();
            target = localTarget(20.0, 0.0, 0.0);

            [~, ~, problem] = collisionAvoidanceController( ...
                localEgo(), target, localLane(), cfg);
            signedDistance = problem.functions.rectangleSignedDistance( ...
                [0.0; 0.0], 0.0, ...
                [cfg.vehicle.length; 0.0], 0.0);

            testCase.verifyEqual(signedDistance, 0.0, AbsTol=1.0e-12);
            testCase.verifyEqual(max(signedDistance, 0.0), 0.0, ...
                AbsTol=1.0e-12);
            testCase.verifyFalse(signedDistance > 0.0);
        end

        function overlappingRectanglesHaveNegativeBarrier(testCase)
            cfg = localConfig();
            overlap = 0.20;
            target = localTarget( ...
                2.0 * cfg.vehicle.length, 0.0, 0.0);

            [~, ~, problem] = collisionAvoidanceController( ...
                localEgo(), target, localLane(), cfg);
            signedDistance = problem.functions.rectangleSignedDistance( ...
                [0.0; 0.0], 0.0, ...
                [cfg.vehicle.length - overlap; 0.0], 0.0);

            testCase.verifyEqual(signedDistance, ...
                -overlap, AbsTol=1.0e-12);
            testCase.verifyEqual(max(signedDistance, 0.0), 0.0, ...
                AbsTol=1.0e-12);
            testCase.verifyLessThan(signedDistance, 0.0);
        end

        function targetOrientationChangesRectangleDistance(testCase)
            cfg = localConfig();
            target = localTarget(20.0, 0.0, 0.0);
            [~, ~, alignedProblem] = collisionAvoidanceController( ...
                localEgo(), target, localLane(), cfg);

            target.targetYawRelative = pi / 2.0;
            [~, ~, perpendicularProblem] = ...
                collisionAvoidanceController( ...
                localEgo(), target, localLane(), cfg);
            alignedDistance = ...
                alignedProblem.functions.rectangleSignedDistance( ...
                [0.0; 0.0], 0.0, ...
                [cfg.vehicle.length; 0.0], 0.0);
            perpendicularDistance = ...
                perpendicularProblem.functions.rectangleSignedDistance( ...
                [0.0; 0.0], 0.0, ...
                [cfg.vehicle.length; 0.0], pi / 2.0);

            expectedDifference = 0.5 ...
                * (cfg.vehicle.length - cfg.vehicle.width);
            testCase.verifyEqual( ...
                perpendicularDistance - alignedDistance, ...
                expectedDifference, AbsTol=1.0e-12);
            testCase.verifyGreaterThan( ...
                perpendicularDistance, alignedDistance);
        end

        function crossingRectanglesUsePenetrationDepth(testCase)
            cfg = localConfig();
            target = localTarget( ...
                2.0 * cfg.vehicle.length, 0.0, pi / 2.0);

            [~, ~, problem] = collisionAvoidanceController( ...
                localEgo(), target, localLane(), cfg);
            signedDistance = problem.functions.rectangleSignedDistance( ...
                [0.0; 0.0], 0.0, [0.0; 0.0], pi / 2.0);

            testCase.verifyEqual(signedDistance, ...
                -0.5 * (cfg.vehicle.length + cfg.vehicle.width), ...
                AbsTol=1.0e-12);
            testCase.verifyEqual(max(signedDistance, 0.0), 0.0, ...
                AbsTol=0.0);
        end

        function sharmaYawPropagationRotatesRectangleOrientation(testCase)
            cfg = localConfig();
            target = localTarget(20.0, 0.0, 0.0);
            target.targetVelocityX = 10.0;
            target.targetVelocityFrame = "inertial";
            target.targetAccelerationX = 2.0;
            target.targetAccelerationFrame = "inertial";
            target.targetAccelerationY = 4.0;
            target.targetYawRate = 0.4;

            [~, predictedInput, problem] = ...
                collisionAvoidanceController( ...
                localEgo(), target, localLane(), cfg);
            horizon = problem.functions.rollout(predictedInput);

            time = (0:cfg.collision.pcbf.horizonSteps) ...
                * cfg.controller.sampleTime;
            curvature = target.targetYawRate / target.targetVelocityX;
            arcLength = target.targetVelocityX * time ...
                + 0.5 * target.targetAccelerationX * time.^2;
            expectedYaw = curvature * arcLength;
            testCase.verifyEqual(horizon.targetYaw, ...
                expectedYaw, AbsTol=1.0e-12);
            signedDistance = problem.functions.rectangleSignedDistance( ...
                [0.0; 0.0], 0.0, [8.0; 0.0], pi / 2.0);
            initialDistance = problem.functions.rectangleSignedDistance( ...
                [0.0; 0.0], 0.0, [8.0; 0.0], 0.0);
            testCase.verifyGreaterThan(signedDistance, ...
                initialDistance);
        end

        function multipleTargetsShareEveryPredictionTime(testCase)
            cfg = localConfig();
            targets = [ ...
                localTarget(20.0, 4.0, 0.1), ...
                localTarget(30.0, -5.0, -0.2)];
            targets(1).targetYawRate = 0.4;
            targets(2).targetVelocityX = 5.0;
            targets(2).targetAccelerationX = -1.0;
            targets(2).targetYawRate = -0.2;

            [~, predictedInput, problem] = ...
                collisionAvoidanceController( ...
                    localEgo(), targets, localLane(), cfg);
            horizon = problem.functions.rollout(predictedInput);

            nodeTime = (0:cfg.collision.pcbf.horizonSteps) ...
                * cfg.controller.sampleTime;
            expectedNodeYaw = zeros(2, numel(nodeTime));
            for targetIdx = 1:2
                speed = targets(targetIdx).targetVelocityX;
                acceleration = targets(targetIdx).targetAccelerationX;
                curvature = targets(targetIdx).targetYawRate / speed;
                nodeArcLength = speed * nodeTime ...
                    + 0.5 * acceleration * nodeTime.^2;
                expectedNodeYaw(targetIdx, :) = ...
                    targets(targetIdx).targetYawRelative ...
                        + curvature * nodeArcLength;
            end

            testCase.verifySize(horizon.targetYaw, [2, 4]);
            testCase.verifyEqual( ...
                horizon.targetYaw, expectedNodeYaw, AbsTol=1.0e-12);
        end
    end
end

function cfg = localConfig()
% These tests hold the ego nearly static to check rectangle geometry, so
% the planned speed floor is switched off; a positive floor is
% incompatible with a zero reference speed.

    cfg = collisionAvoidanceControllerConfig();
    cfg.collision.pcbf.horizonSteps = 3;
    cfg.referenceSpeed = 0.0;
    cfg.model.plannedSpeedMinimum = 0.0;
end

function ego = localEgo()
% These tests exercise rectangle geometry through the MPC problem, so the
% ego must not satisfy terminal-set membership; otherwise the controller
% correctly hands off to the terminal controller and returns no QP. The
% center lies in the left terminal band so the short-horizon terminal rows
% remain reachable, while lateral velocity is just outside the terminal
% feedback domain (terminalSafety.lateralVelocityMaximum = 0.75 m/s) so
% the shoulder is still geometrically available to the terminal
% constraint while membership fails.

    ego = struct("positionX", 0.0, "positionY", 1.3, ...
        "yawAngle", 0.0, "longitudinalVelocity", 0.1, ...
        "lateralVelocity", 1.0, "yawRate", 0.0);
end

function target = localTarget(x, y, yawRelative)
    target = struct("relativePositionX", x, ...
        "relativePositionY", y, ...
        "relativePositionFrame", "ego", ...
        "targetVelocityX", 10.0, "targetVelocityY", 0.0, ...
        "targetVelocityFrame", "inertial", ...
        "targetAccelerationX", 0.0, "targetAccelerationY", 0.0, ...
        "targetAccelerationFrame", "inertial", ...
        "targetYawRelative", yawRelative, "targetYawRate", 0.0);
end

function road = localLane()
    centerline = [-100.0, 0.0; 100.0, 0.0];
    perception = fitPerceivedRoadBoundaries( ...
        centerline, [0.0; 0.0; 0.0], ...
        PerceptionRange=30.0, ...
        RightOffset=8.0, LeftOffset=0.001, ...
        ShoulderWidth=2.60, ...
        RouteBranchId="through");
    road = perception.roadGeometry;
    road.boundaries = struct([]);
end
