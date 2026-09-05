classdef controllerDesignExperimentTest < matlab.unittest.TestCase
% controllerDesignExperimentTest Scene geometry and honest outcome reporting.
% Synthetic traces below exercise evaluation only, not vehicle dynamics.

    properties (TestParameter)
        sceneIndex = struct("straight", 1, "arc", 2)
    end

    methods (TestClassSetup)
        function addRepositoryPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, "scripts")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, "config")));
        end
    end

    methods (Test)
        function targetStartsHiddenAndCrossesTheCruisePath(testCase, sceneIndex)
            experiment = controllerDesignExperimentConfig();
            scene = experiment.scenes(sceneIndex);
            collisionPosition = scene.targetInitialPosition ...
                + experiment.nominalCollisionTime*scene.targetVelocity;
            testCase.verifyGreaterThan(norm(scene.targetInitialPosition), 50.0);
            testCase.verifyEqual(collisionPosition, scene.collisionPosition, AbsTol=1.0e-12);
            testCase.verifyEqual(norm(scene.targetVelocity), 8.0, AbsTol=1.0e-12);
            testCase.verifyEqual(experiment.perceptionRange, 50.0, AbsTol=0.0);
        end

        function nominalContactAndCompletedAvoidanceAreDistinguished(testCase, sceneIndex)
            [nominal, avoidance, scene, experiment] = localEvaluationFixture(sceneIndex);
            result = evaluateControllerDesignExperiment(nominal, avoidance, scene, experiment);
            testCase.verifyTrue(result.summary.scenarioValid);
            testCase.verifyTrue(result.summary.functionalPass);
            testCase.verifyLessThan(result.summary.nominalMinimumSatMargin, 0.0);
            testCase.verifyGreaterThan(result.summary.avoidanceMinimumSatMargin, 0.0);
            testCase.verifyGreaterThanOrEqual(result.summary.nominalContactTime, 5.7);
            testCase.verifyLessThanOrEqual(result.summary.nominalContactTime, 6.0);
        end

        function failureAtFirstDetectionDoesNotPassCollisionFreedom(testCase, sceneIndex)
            [nominal, avoidance, scene, experiment] = localEvaluationFixture(sceneIndex);
            firstVisible = find(avoidance.attempts.targetVisible, 1);
            avoidance = localTruncateAtFailedAttempt(avoidance, firstVisible);
            result = evaluateControllerDesignExperiment(nominal, avoidance, scene, experiment);
            testCase.verifyTrue(result.noContactObserved);
            testCase.verifyTrue(result.visibilityPass);
            testCase.verifyFalse(result.summary.completed);
            testCase.verifyFalse(result.summary.collisionPass);
            testCase.verifyFalse(result.summary.functionalPass);
            testCase.verifyEqual(result.summary.firstDetectionTime, ...
                avoidance.attempts.time(end), AbsTol=0.0);
        end

        function denseInterSampleContactDoesNotChangeNodeAcceptance(testCase, sceneIndex)
            [nominal, avoidance, scene, experiment] = localEvaluationFixture(sceneIndex);
            avoidance.plantTrace = struct("time", 6.025, ...
                "positionX", scene.collisionPosition(1), ...
                "positionY", scene.collisionPosition(2));
            result = evaluateControllerDesignExperiment(nominal, avoidance, scene, experiment);
            testCase.verifyTrue(result.summary.collisionPass);
        end

        function deadlineFailureIsReportedAlongsideFunctionalSuccess(testCase, sceneIndex)
            [nominal, avoidance, scene, experiment] = localEvaluationFixture(sceneIndex);
            avoidance.attempts.solveTime(1) = 0.2;
            result = evaluateControllerDesignExperiment(nominal, avoidance, scene, experiment);
            testCase.verifyTrue(result.summary.functionalPass);
            testCase.verifyFalse(result.summary.deadlinePass);
        end

        function contactAtAControlStepFailsAvoidance(testCase, sceneIndex)
            [nominal, avoidance, scene, experiment] = localEvaluationFixture(sceneIndex);
            contactIndex = find(abs(avoidance.controlTime-6.0) < 1.0e-9, 1);
            avoidance.controlState(contactIndex,1:2) = scene.collisionPosition.';
            result = evaluateControllerDesignExperiment(nominal, avoidance, scene, experiment);
            testCase.verifyFalse(result.noContactObserved);
            testCase.verifyFalse(result.summary.collisionPass);
        end

        function earlyTargetInformationInvalidatesTheScenario(testCase, sceneIndex)
            [nominal, avoidance, scene, experiment] = localEvaluationFixture(sceneIndex);
            avoidance.attempts.targetVisible(1) = true;
            result = evaluateControllerDesignExperiment(nominal, avoidance, scene, experiment);
            testCase.verifyFalse(result.visibilityPass);
            testCase.verifyFalse(result.summary.scenarioValid);
        end

        function aCornerOutsideTheRoadFailsTheRoadRequirement(testCase, sceneIndex)
            [nominal, avoidance, scene, experiment] = localEvaluationFixture(sceneIndex);
            avoidance.controlState(end,2) = avoidance.controlState(end,2)+9.0;
            result = evaluateControllerDesignExperiment(nominal, avoidance, scene, experiment);
            testCase.verifyLessThan(result.summary.minimumRoadMargin, 0.0);
            testCase.verifyFalse(result.summary.roadPass);
        end
    end
end

function [nominal, avoidance, scene, experiment] = localEvaluationFixture(sceneIndex)
    experiment = controllerDesignExperimentConfig();
    scene = experiment.scenes(sceneIndex);
    time = (0:0.05:experiment.duration).';
    nominal = localTrace(time, 15.0*time, 15.0*ones(size(time)), scene, experiment);
    initialTarget = scene.targetInitialPosition.'+time*scene.targetVelocity.';
    visible = vecnorm(initialTarget-nominal.controlState(:,1:2), 2, 2) <= 50.0;
    detectionTime = time(find(visible, 1));
    % Idealized stop-and-resume trace for the evaluator, not a plant run.
    station = 15.0*min(time, detectionTime)+15.0*max(time-7.0, 0.0);
    speed = 15.0*double(time <= detectionTime | time >= 7.0);
    avoidance = localTrace(time, station, speed, scene, experiment);
end

function result = localTrace(time, station, speed, scene, experiment)
    count = numel(time)-1;
    yaw = scene.curvature*station;
    if scene.curvature == 0.0
        position = [station, zeros(size(station))];
    else
        position = [sin(yaw), 1.0-cos(yaw)]/scene.curvature;
    end
    result = struct();
    result.controlTime = time;
    result.controlState = [position, yaw, speed, zeros(size(time)), ...
        scene.curvature*speed];
    result.controlTracking = struct("lateralError", zeros(size(time)), ...
        "headingError", zeros(size(time)));
    result.metrics = struct("controllerCompletedScenario", true, ...
        "cruiseMaintained", true);
    result.failure = struct("occurred", false);
    target = scene.targetInitialPosition.'+time*scene.targetVelocity.';
    range = vecnorm(target-position, 2, 2);
    metadata = struct("hardCbfSatisfied", true, ...
        "terminalInvariantCertified", true, "hardRowViolation", 0.0);
    result.attempts = struct("time", time(1:count), ...
        "targetVisible", range(1:count) <= 50.0, ...
        "solveTime", 0.01*ones(count,1), ...
        "metadata", {repmat({metadata}, count, 1)});
    command = struct("actuatorInput", [0.0; 0.0], ...
        "axleFrictionUtilization", [0.0; 0.0]);
    result.command = repmat({command}, count, 1);
    result.controllerConfiguration = collisionAvoidanceControllerConfig( ...
        experiment.controllerConfiguration);
end

function result = localTruncateAtFailedAttempt(result, count)
    result.controlTime = result.controlTime(1:count);
    result.controlState = result.controlState(1:count,:);
    result.controlTracking.lateralError = result.controlTracking.lateralError(1:count);
    result.controlTracking.headingError = result.controlTracking.headingError(1:count);
    result.attempts.time = result.attempts.time(1:count);
    result.attempts.targetVisible = result.attempts.targetVisible(1:count);
    result.attempts.solveTime = result.attempts.solveTime(1:count);
    result.attempts.metadata = result.attempts.metadata(1:count);
    result.attempts.metadata{end} = [];
    result.command = result.command(1:count-1);
    result.metrics.controllerCompletedScenario = false;
    result.failure.occurred = true;
end
