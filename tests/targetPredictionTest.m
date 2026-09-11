classdef targetPredictionTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addControllerPath(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
        end
    end

    methods (Test)
        function zeroJerkCartesianFlowDoesNotRepresentAnExactTurningAnchor(testCase)
            encounter = struct("center", [0;0;8;0;0;1.6;0;0.2], ...
                "radius", zeros(8,1), "contract", struct( ...
                "jerkBound", zeros(2,1), "yawAccelerationBound", 0));

            [cartesian, radius] = targetPrediction.finiteFlow(encounter, 1);
            curved = targetPrediction.nominalFlow(encounter, 1);

            testCase.verifyEqual(cartesian(1:2), [8;0.8], AbsTol=1e-12);
            testCase.verifyEqual(curved(1:2), ...
                [40*sin(0.2);40*(1-cos(0.2))], AbsTol=1e-12);
            testCase.verifyGreaterThan(norm(cartesian(1:2)-curved(1:2)), 0.05);
            testCase.verifyLessThan(norm(radius(1:2)), 1e-10);
        end

        function batchedNominalFlowPreservesCurvatureAndTheStop(testCase)
            encounter = struct("center",[0;0;2;0;-2;.2;0;.1], ...
                "contract",struct("scalarAccelerationMaximum",2,"predictionSampleTime",.05));
            time = [0,.25,.5,1,2];
            [batch,jerk,yawAcceleration] = targetPrediction.nominalFlow(encounter,time);
            [states,jerks,yawAccelerations] = arrayfun(@(t) targetPrediction.nominalFlow(encounter,t), ...
                time,UniformOutput=false);
            testCase.verifyEqual(batch,horzcat(states{:}),AbsTol=1e-14);
            testCase.verifyEqual(jerk,horzcat(jerks{:}),AbsTol=1e-14);
            testCase.verifyEqual(yawAcceleration,horzcat(yawAccelerations{:}),AbsTol=1e-14);
            testCase.verifyEqual(batch(:,end),batch(:,end-1),AbsTol=1e-14);
        end
        function exactInitialMotionStaysExactAcrossTheStop(testCase)
            model = localModel([2; 0], [-2; 0], 0, zeros(8, 1));
            [position, yaw] = targetPrediction.errorEnvelope([0, 0.5, 1, 2, 100], model);
            testCase.verifyEqual(position, zeros(2, 5), AbsTol=1e-14);
            testCase.verifyEqual(yaw, zeros(1, 5), AbsTol=1e-14);
        end

        function initialVelocityErrorPersistsWithoutInventingFutureManeuvers(testCase)
            model = localModel([5; 0], [0; 0], 0, [0.1; 0.1; 0.2; 0.2; zeros(4, 1)]);
            [position, yaw] = targetPrediction.errorEnvelope([0, 1, 2], model);
            testCase.verifyEqual(position(:, 1), [0.1; 0.1], AbsTol=0.0);
            testCase.verifyGreaterThan(position(:, 3), position(:, 2));
            testCase.verifyEqual(yaw, zeros(1, 3), AbsTol=0.0);
        end

        function uncertaintyStopsGrowingAfterEveryPredictionHasStopped(testCase)
            model = localModel([2; 0], [-2; 0], 0, [0.1; 0.1; 0.2; 0; zeros(4, 1)]);
            [position, ~] = targetPrediction.errorEnvelope([10, 100], model);
            testCase.verifyEqual(position(:, 2), position(:, 1), AbsTol=1e-14);
            testCase.verifyGreaterThan(position(:, 1), model.targetPositionErrorBound);
        end

        function unknownCurvatureNearZeroStillHasAFiniteHorizonEnvelope(testCase)
            model = localModel([5; 0], [0; 0], 0, ...
                [0.1; 0.1; 0.1; 0.1; 0.1; 0.1; 0.01; 0.01]);
            [position, yaw] = targetPrediction.errorEnvelope([0, 1, 4], model);
            testCase.verifyTrue(all(isfinite(position), "all"));
            testCase.verifyTrue(all(isfinite(yaw), "all"));
        end

        function legacyOperatingLimitsDoNotBecomeFutureDisturbances(testCase)
            model = localModel([5; 0], [0; 0], 0, zeros(8, 1));
            model.targetPredictionMotionBounds = struct("speedMaximum", 100, ...
                "accelerationNormMaximum", 100, "jerkNormMaximum", 100, "yawRateMaximum", 100);
            [position, yaw] = targetPrediction.errorEnvelope([0, 1, 10], model);
            testCase.verifyEqual(position, zeros(2, 3), AbsTol=0.0);
            testCase.verifyEqual(yaw, zeros(1, 3), AbsTol=0.0);
        end

        function fullInitialBoxPropagatesInsideBothEnclosures(testCase)
            model = localModel([5; 0.3], [-0.8; 0.5], 0.08, ...
                [0.1; 0.2; 0.15; 0.1; 0.08; 0.06; 0.03; 0.01]);
            [positionResidual, yawResidual] = localVertexResiduals(model);
            testCase.verifyLessThanOrEqual(positionResidual, 1e-10);
            testCase.verifyLessThanOrEqual(yawResidual, 1e-10);
        end

        function aVelocityBoxContainingRestRemainsEnclosed(testCase)
            model = localModel([0.1; 0], [0.2; 0], 0.01, ...
                [0.1; 0.1; 0.2; 0.2; 0.1; 0.1; 0.02; 0.02]);
            [positionResidual, yawResidual] = localVertexResiduals(model);
            testCase.verifyLessThanOrEqual(positionResidual, 1e-10);
            testCase.verifyLessThanOrEqual(yawResidual, 1e-10);
        end
    end
end

function model = localModel(velocity, acceleration, yawRate, bounds)
    speed = norm(velocity);
    course = [1; 0];
    curvature = 0;
    if speed > 0
        course = velocity/speed;
        curvature = yawRate/speed;
    elseif norm(acceleration) > 0
        course = acceleration/norm(acceleration);
    end
    tangentAcceleration = course.'*acceleration;
    stopTime = inf;
    if tangentAcceleration < 0
        stopTime = speed/-tangentAcceleration;
    end
    prediction = struct("initialPosition", [0; 0], "initialCourseDirection", course, ...
        "initialSpeed", speed, "tangentialAcceleration", tangentAcceleration, ...
        "curvature", curvature, "stopTime", stopTime);
    model = struct("hasTarget", true, "targetPrediction", prediction, ...
        "targetVelocity", velocity, "targetAcceleration", acceleration, "targetYawRate", yawRate, ...
        "targetPositionErrorBound", bounds(1:2), "targetVelocityErrorBound", bounds(3:4), ...
        "targetAccelerationErrorBound", bounds(5:6), ...
        "targetYawErrorBound", bounds(7), "targetYawRateErrorBound", bounds(8));
    model.targetPredictionSet = targetPrediction.initialSet(model);
end

function [positionResidual, yawResidual] = localVertexResiduals(model)
    time = [0, 0.1, 1, 4, 10, 100];
    [positionBound, yawBound] = targetPrediction.errorEnvelope(time, model);
    [nominalPosition, nominalYaw] = localTrajectory(model.targetPrediction, time);
    bounds = [model.targetPositionErrorBound; model.targetVelocityErrorBound; ...
        model.targetAccelerationErrorBound; model.targetYawErrorBound; model.targetYawRateErrorBound];
    signs = 2*(dec2bin(0:255, 8)-'0').'-1;
    positionResidual = -inf;
    yawResidual = -inf;
    for index = 1:size(signs, 2)
        offset = bounds.*signs(:, index);
        member = localModel(model.targetVelocity+offset(3:4), ...
            model.targetAcceleration+offset(5:6), model.targetYawRate+offset(8), zeros(8, 1));
        [position, yaw] = localTrajectory(member.targetPrediction, time);
        position = position+offset(1:2);
        yawError = atan2(sin(yaw+offset(7)-nominalYaw), cos(yaw+offset(7)-nominalYaw));
        positionResidual = max(positionResidual, max(abs(position-nominalPosition)-positionBound, [], "all"));
        yawResidual = max(yawResidual, max(abs(yawError)-yawBound));
    end
end

function [position, yaw] = localTrajectory(prediction, time)
    elapsed = min(time, prediction.stopTime);
    distance = prediction.initialSpeed*elapsed+0.5*prediction.tangentialAcceleration*elapsed.^2;
    yaw = prediction.curvature*distance;
    course = prediction.initialCourseDirection;
    if prediction.curvature == 0
        position = course*distance;
    else
        position = (course*sin(yaw)+[-course(2); course(1)]*(1-cos(yaw)))/prediction.curvature;
    end
end
