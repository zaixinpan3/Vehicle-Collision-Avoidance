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
            testCase.verifyTrue(published.targetEstimate.controllerErrorBound.available);
            testCase.verifyGreaterThanOrEqual( ...
                [published.controllerErrorBound.bounds; ...
                published.targetEstimate.controllerErrorBound.bounds]-actualError, -1e-10);
        end
        function freshVelocityBoundsRetainDirectionalInformation(testCase)
            [output, bound, input] = localVelocityFixture();
            published = nrmmControllerErrorBounds(output, bound, input, testCase.Design);
            testCase.verifyLessThan(published.controllerStateErrorBound(4), 0.08);
            testCase.verifyGreaterThan(published.controllerStateErrorBound(5), 0.49);
            testCase.verifyEqual(published.egoBodyVelocity, output.egoBodyVelocity);
            testCase.verifyEqual(published.egoBodyVelocityErrorBound, bound.bodyVelocity);
            for center = [0, 3.13, -1.2]
                bound.orientationSet = nrmmYawSet("initialize", center, 0.05);
                input.gnssVelocity = localRotation(center)*[10;0];
                published = nrmmControllerErrorBounds(output, bound, input, testCase.Design);
                for heading = center+linspace(-0.05, 0.05, 21)
                    for direction = linspace(-pi, pi, 21)
                        noise = testCase.Design.sensors.velocityNoiseMaximum ...
                            *[cos(direction);sin(direction)];
                        truth = localRotation(heading).'*(input.gnssVelocity-noise);
                        testCase.verifyLessThanOrEqual(abs(truth-output.egoBodyVelocity), ...
                            published.controllerStateErrorBound(4:5)+1e-12);
                    end
                end
            end
        end
        function anExactNrmmTargetPublishesTheNrmmContract(testCase)
            published = localReconstruction(testCase.Design, [1.2; 0.03; 0]);
            motion = published.targetEstimate.predictionMotion;
            domain = testCase.Design.target.domain;
            testCase.assertEqual(testCase.Design.target.modelJerkMaximum, 0);
            testCase.verifyEqual(string(motion.kind), "nrmm-motion-v1");
            testCase.verifyEqual(sort(string(fieldnames(motion))), ...
                sort(["kind"; "curvatureMaximum"; "speedRateMaximum"; "scalarAccelerationMaximum"]));
            testCase.verifyEqual(motion.curvatureMaximum, ...
                sin(domain.sideslipMaximum)/domain.rearAxleDistance);
            testCase.verifyEqual(motion.speedRateMaximum, domain.scalarAccelerationMaximum);
            % The controller caps the acceleration magnitude, hypot(A, V*omega).
            testCase.verifyEqual(motion.scalarAccelerationMaximum, domain.accelerationNormBound);
            testCase.verifyGreaterThan(motion.scalarAccelerationMaximum, domain.scalarAccelerationMaximum);
        end
        function thePublishedParameterBoundsContainTheTruthAndBeatTheBox(testCase, geometry)
            [published, ~, truth] = localReconstruction(testCase.Design, geometry);
            estimate = published.targetEstimate;
            velocity = estimate.targetVelocityInertial;acceleration = estimate.targetAccelerationInertial;
            speed = norm(velocity);
            courseError = atan2(sin(truth.course-atan2(velocity(2), velocity(1))), ...
                cos(truth.course-atan2(velocity(2), velocity(1))));
            testCase.verifyLessThanOrEqual(abs(truth.speed-speed), estimate.targetSpeedErrorBound+1e-12);
            testCase.verifyLessThanOrEqual(abs(courseError), estimate.targetCourseErrorBound+1e-12);
            testCase.verifyLessThanOrEqual(abs(truth.scalarAcceleration-dot(velocity, acceleration)/speed), ...
                estimate.targetSpeedRateErrorBound+1e-12);
            curvature = truth.yawRate/truth.speed;
            testCase.verifyGreaterThanOrEqual(curvature, estimate.targetCurvatureInterval(1)-1e-12);
            testCase.verifyLessThanOrEqual(curvature, estimate.targetCurvatureInterval(2)+1e-12);
            if geometry(3) == 0
                % The frame-free speed bound beats the inertial box corner, which
                % carries the ego rotation error and both axes.
                values = estimate.controllerErrorBound.bounds;
                testCase.verifyLessThan(estimate.targetSpeedErrorBound, norm(values(3:4)));
            end
        end
        function aModelErrorPublishesNoMotionContract(testCase)
            % A nonzero model jerk admits motion outside every NRMM path, which
            % no contract describes; the controller then refuses the target.
            design = testCase.Design;
            design.target.modelJerkMaximum = 0.5;
            published = localReconstruction(design, [1.2; 0.03; 0]);
            estimate = published.targetEstimate;
            testCase.verifyTrue(isfield(estimate, "predictionMotion"));
            testCase.verifyEmpty(estimate.predictionMotion);
            testCase.verifyFalse(any(isfield(estimate, ["targetSpeedErrorBound", "targetCourseErrorBound", ...
                "targetSpeedRateErrorBound", "targetCurvatureInterval"])));
        end
        function staleVelocityRequiresAnAccelerationEnvelope(testCase)
            [output, bound, input] = localVelocityFixture();
            output.stateTime = 0.1;
            bound.holdBounds.acceleration = Inf;
            published = nrmmControllerErrorBounds(output, bound, input, testCase.Design);
            testCase.verifyEqual(published.controllerStateErrorBound(4:5), [0.8;0.8]);
            bound.holdBounds.acceleration = 2;
            published = nrmmControllerErrorBounds(output, bound, input, testCase.Design);
            testCase.verifyGreaterThan(published.controllerStateErrorBound(4), 0.2);
            testCase.verifyLessThan(published.controllerStateErrorBound(4), 0.28);
        end
    end
end

function [output, bound, input] = localVelocityFixture()
    output = struct("stateTime",0,"egoPositionInertial",[0;0], ...
        "egoBodyVelocity",[10;0],"targetEstimate",struct.empty);
    bound = struct("yaw",0.05,"bodyVelocity",0.8,"egoValid",true, ...
        "holdBounds",struct("yawAcceleration",0,"acceleration",Inf), ...
        "orientationSet",nrmmYawSet("initialize",0,0.05),"scope","test enclosure");
    input = struct("time",0,"yawRate",0,"gnssPosition",[0;0],"gnssVelocity",[10;0]);
end

function [published, actualError, truth] = localReconstruction(design, geometry)
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
        "targetEstimate", target);
    components = vecnorm(reshape(actual-estimated, 2, 3)).';
    bound = struct("yaw", abs(geometry(2)), "bodyVelocity", norm(egoVelocity-estimatedEgoVelocity), ...
        "targetComponents", components, "trueRangeMaximum", norm(actual(1:2)), ...
        "egoValid", true, "valid", true, "scope", "declared test enclosure", ...
        "holdBounds", struct("yawAcceleration", 0), ...
        "orientationSet",nrmmYawSet("initialize",estimatedYaw,abs(geometry(2))));
    input = struct("time", 0, "yawRate", 0, "gnssPosition", ...
        position+design.sensors.positionNoiseMaximum*[1; 1]/sqrt(2));
    published = nrmmControllerErrorBounds(output, bound, input, design);
    truth = struct("speed", speed, "course", course, "scalarAcceleration", scalarAcceleration, ...
        "yawRate", yawRate, "yaw", targetYaw);
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
