classdef nrmmControllerErrorBoundsTest < matlab.unittest.TestCase
    properties
        Design
    end
    properties (TestParameter)
        geometry = struct( ...
            'rotated', [1.2; 0.03; 0], ...
            'angleBranch', [3.13; -0.04; 0], ...
            'velocityPeaking', [-0.7; 0.2; 1], ...
            'nearZeroVelocity', [0.6; -0.2; 2]);
    end
    methods (TestClassSetup)
        function prepareDesign(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "estimator")));
            testCase.Design = synthesizeNrmmObserverGains(nrmmTrackingConfig());
        end
    end
    methods (Test)
        function reconstructedWorldStateIsContainedAcrossRotationsAndPeaking(testCase, geometry)
            [published, actualError] = localReconstruction(testCase.Design, geometry);
            testCase.verifyTrue(published.controllerErrorBound.available);
            testCase.verifyTrue(published.targetEstimates.controllerErrorBound.available);
            testCase.verifyGreaterThanOrEqual( ...
                [published.controllerErrorBound.bounds; ...
                published.targetEstimates.controllerErrorBound.bounds]-actualError, -1e-10);
        end
    end
end

function [published, actualError] = localReconstruction(design, geometry)
    yaw = geometry(1);
    estimatedYaw = yaw+geometry(2);
    rotation = localRotation(yaw);
    estimatedRotation = localRotation(estimatedYaw);
    position = [4; -2];
    estimatedPosition = position+[0.03; -0.02];
    egoVelocity = [12; 0.1];
    estimatedEgoVelocity = egoVelocity+[0.04; -0.08];
    targetPosition = [25; 5];
    speed = 0.5*(design.target.domain.speedMinimum+design.target.domain.speedMaximum);
    course = -3.13;
    domain = design.target.domain;
    yawRate = min(0.25*domain.yawRateMaximum, ...
        0.5*sin(domain.sideslipMaximum)*speed/domain.rearAxleDistance);
    scalarAcceleration = 0.2*domain.scalarAccelerationMaximum;
    direction = [cos(course); sin(course)];
    targetVelocity = speed*direction;
    targetAcceleration = scalarAcceleration*direction+yawRate*speed*[-direction(2); direction(1)];
    targetYaw = course-asin(domain.rearAxleDistance*yawRate/speed);
    actual = [rotation.'*(targetPosition-position); ...
        rotation.'*targetVelocity; rotation.'*targetAcceleration];
    estimated = actual+[0.1; -0.15; 0.08; -0.1; 0.04; 0.06];
    switch geometry(3)
        case 1
            estimated(3:4) = [100; -120];
            estimated(5:6) = [-30; 50];
        case 2
            estimated(3:4) = [1e-6; -1e-6];
    end
    [~, target] = nrmmTargetTrackerDerivative(estimated, ...
        struct("bodyVelocity", estimatedEgoVelocity, "yawRate", 0), domain);
    target.targetPositionInertial = estimatedPosition+estimatedRotation*estimated(1:2);
    target.targetVelocityInertial = estimatedRotation*estimated(3:4);
    target.targetAccelerationInertial = estimatedRotation*estimated(5:6);
    target.targetHeadingInertial = estimatedYaw+target.targetCourseAngleEgoFrame-target.targetSideslip;
    output = struct("stateTime", 0, "egoPositionInertial", estimatedPosition, ...
        "targetEstimates", target);
    components = vecnorm(reshape(actual-estimated, 2, 3)).';
    bound = struct("yaw", abs(geometry(2)), "bodyVelocity", norm(egoVelocity-estimatedEgoVelocity), ...
        "targetComponents", components, "trueRangeMaximum", norm(actual(1:2)), ...
        "egoValid", true, "valid", true, "scope", "declared test enclosure", ...
        "holdBounds", struct("yawAcceleration", 0), ...
        "orientationSet",nrmmYawSet("initialize",estimatedYaw,abs(geometry(2))));
    input = struct("time", 0, "yawRate", 0, "gnssPosition", ...
        position+design.sensors.positionNoiseMaximum*[1; 1]/sqrt(2));
    published = nrmmControllerErrorBounds(output, bound, input, design);
    actualError = [abs(position-estimatedPosition); abs(geometry(2)); ...
        abs(egoVelocity-estimatedEgoVelocity); 0; ...
        abs(targetPosition-target.targetPositionInertial); ...
        abs(targetVelocity-target.targetVelocityInertial); ...
        abs(targetAcceleration-target.targetAccelerationInertial); ...
        abs(atan2(sin(targetYaw-target.targetHeadingInertial), ...
            cos(targetYaw-target.targetHeadingInertial))); abs(yawRate-target.targetYawRate)];
end

function rotation = localRotation(yaw)
    rotation = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
end
