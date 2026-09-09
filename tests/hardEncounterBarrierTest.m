classdef hardEncounterBarrierTest < matlab.unittest.TestCase
    %hardEncounterBarrierTest Declared-inclusion tests, not vehicle trials.
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, 'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, 'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, 'tests')));
        end
    end

    methods (Test)
        function admissionCertifiesExitWithHardSafetyAndSoftPerformance(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            testCase.verifyTrue(problem.metadata.recursiveFeasibilityClaimed);
            testCase.verifyFalse(problem.metadata.physicalVehicleGuaranteeEstablished);
            testCase.verifyGreaterThan(stored.margin, 0);
            testCase.verifyGreaterThan(problem.metadata.solverCallCount, 0);
            testCase.verifyGreaterThan(stored.acceptance.exitMargin, 0);
            testCase.verifyEqual(stored.certifiedDuration, 0.4, AbsTol=1e-12);
            safety = problem.qp.barrier.scale > 0;
            testCase.verifyEqual(nnz(problem.qp.inequalityMatrix(safety, problem.layout.relaxationIndex)), 0);
            testCase.verifyEqual(problem.metadata.barrierInterpretation, "storedWitnessLowerBound");
        end

        function aSafeShortSegmentWithoutExitIsRejected(testCase)
            [ego, target, route, cfg] = localFixture();
            target.targetVelocityInertial = [8; 0];
            target.targetHeadingInertial = 0;
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, []), ...
                'collisionAvoidanceController:noCertifiedContinuation');
        end

        function initialOverlapCannotBeRelaxed(testCase)
            [ego, target, route, cfg] = localFixture();
            target.targetPositionInertial = ego.position;
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, []), ...
                'collisionAvoidanceController:noCertifiedContinuation');
        end

        function countdownPreservesMarginAndCompletesAtTheOriginalDeadline(testCase)
            [history, commands] = localRunEncounter();
            testCase.verifyEqual([history.deadline], repmat(0.4, 1, 5), AbsTol=1e-12);
            testCase.verifyEqual([history.remainingSteps], 4:-1:0);
            testCase.verifyGreaterThanOrEqual(diff([history.margin]), zeros(1, 4));
            testCase.verifyTrue(history(end).encounterComplete);
            testCase.verifyEmpty(commands{end});
            testCase.verifyEqual(history(end).stateTime, 0.4, AbsTol=1e-12);
        end

        function retainedExecutionDoesNotNeedAnOptimizer(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            [command, ~, next, updated] = collisionAvoidanceController(ego, target, route, cfg, stored);
            testCase.verifyEqual(command.actuatorInput, stored.plan(:, 2), AbsTol=0);
            testCase.verifyEqual(next.metadata.solverCallCount, 0);
            testCase.verifyEqual(next.metadata.certificateSource, "retainedCertifiedWitness");
            testCase.verifyGreaterThanOrEqual(updated.margin, stored.margin);
        end

        function failedReoptimizationKeepsTheCertifiedTail(testCase)
            [ego, target, route, cfg] = localFixture();
            cfg.encounter.reoptimizeContinuation = true;
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            [command, ~, next, updated] = collisionAvoidanceController(ego, target, route, cfg, stored);
            testCase.verifyEqual(command.actuatorInput, stored.plan(:, 2), AbsTol=0);
            testCase.verifyTrue(next.metadata.fallbackUsed);
            testCase.verifyEqual(updated.deadline, stored.deadline, AbsTol=0);
            testCase.verifyGreaterThanOrEqual(updated.margin, stored.margin);
        end

        function reoptimizationPreservesTheExecutedPrefixAndMargin(testCase)
            [ego, target, route, cfg] = localFixture();
            cfg.encounter.reoptimizeContinuation = true;
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            [~, ~, next, updated] = collisionAvoidanceController(ego, target, route, cfg, stored);
            testCase.verifyEqual(updated.decision(1:2), stored.appliedInput, AbsTol=0);
            testCase.verifyGreaterThanOrEqual(updated.margin, stored.margin);
            testCase.verifyTrue(next.metadata.carriedWitnessFeasible);
        end

        function aSolverCannotChangeAnExecutedInput(testCase)
            [ego, target, route, cfg] = localFixture();
            cfg.encounter.reoptimizeContinuation = true;
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            cfg.solver.jointFunction = @encounterTestFixture.unsafe;
            [command, ~, next] = collisionAvoidanceController(ego, target, route, cfg, stored);
            testCase.verifyEqual(command.actuatorInput, stored.plan(:, 2), AbsTol=0);
            testCase.verifyTrue(next.metadata.fallbackUsed);
        end

        function failedPerformanceSolveRetainsTheCheckedMarginPlan(testCase)
            [ego, target, route, cfg] = localFixture();
            cfg.solver.jointFunction = @localFailPerformance;
            [command, ~, problem] = collisionAvoidanceController(ego, target, route, cfg, []);
            testCase.verifyNotEmpty(command);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyGreaterThan(problem.metadata.carriedMargin, 0);
        end

        function inconsistentEgoObservationInvalidatesTheExecutionPremise(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            ego.position(2) = 1;
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, stored), ...
                'collisionAvoidanceController:inconsistentObservation');
        end

        function targetMeasurementsCannotRenewAnIncompatibleTrajectory(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            target.targetVelocityInertial(1) = 0;
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, stored), ...
                'collisionAvoidanceController:inconsistentObservation');
        end

        function newTargetsRequireJointAdmission(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            target.trackId = 2;
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, stored), ...
                'collisionAvoidanceController:newTargetRequiresAdmission');
        end

        function changedSensorRangeCannotInheritTheTerminalCertificate(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            ego.perception.range = 20;
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, stored), ...
                'collisionAvoidanceController:changedExecutionContract');
        end

        function aChangedHeldInputCannotUseTheStoredPlan(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            ego.heldActuatorInput(2) = ego.heldActuatorInput(2)+0.01;
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, stored), ...
                'collisionAvoidanceController:executionContractViolation');
        end

        function anInflatedStoredMarginIsRejected(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            stored.margin = stored.margin+0.01;
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, stored), ...
                'collisionAvoidanceController:invalidStoredCertificate');
        end

        function exactCurvedTargetMotionMayHaveNonzeroCartesianJerk(testCase)
            [ego, target, route, cfg] = localFixture();
            target.targetYawRate = 0.1;
            target.targetAccelerationInertial = [0; -0.8];
            target.predictionMotion.jerkBound = [0.2; 0.2];
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            ego = encounterTestFixture.nextEgo(stored, problem.model.lane);
            ego.perception = struct('time', ego.stateTime, 'range', 13.5, 'completeWithinRange', true);
            [flow, jerk] = targetPrediction.nominalFlow(stored.encounters, cfg.controller.sampleTime);
            target = localTargetState(target, flow);
            [~, ~, next] = collisionAvoidanceController(ego, target, route, cfg, stored);
            testCase.verifyGreaterThan(jerk, 0);
            testCase.verifyTrue(next.metadata.planCertified);
        end

        function allHeldIntervalsHavePhysicalClearanceInADenseAudit(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            minimum = localDenseClearance(stored, problem.model.lane, cfg);
            testCase.verifyGreaterThanOrEqual(minimum, 0);
        end

        function partialLookaheadCannotClaimACompleteBarrier(testCase)
            [~, ~, ~, cfg] = localFixture();
            cfg.controller.certifiedSteps = 1;
            testCase.verifyError(@() collisionAvoidanceControllerConfig(cfg), ...
                'collisionAvoidanceController:invalidConfiguration');
        end

        function finiteActuatorSlewLimitsSurviveTheShift(testCase)
            [ego, target, route, cfg] = localFixture();
            cfg.model.frontWheelSteeringRateMaximum = 1;
            cfg.model.brakingRatioRateMaximum = 2;
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            [command, ~, ~, updated] = collisionAvoidanceController(ego, target, route, cfg, stored);
            testCase.verifyLessThanOrEqual(abs(command.actuatorInput-stored.appliedInput), [0.1; 0.2]);
            testCase.verifyGreaterThanOrEqual(updated.margin, stored.margin);
        end

        function declaredEgoAndTargetUncertaintyRemainCovered(testCase)
            [ego, target, route, cfg] = localFixture();
            ego.controllerStateErrorBound = [0.001; 0.001; 0.0001; 0.001; 0.0001; 0.0001];
            cfg.model.plantModelResidualRateBound(4) = 0.001;
            target.targetPositionInertialErrorBound = [0.001; 0.001];
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            [~, ~, next, updated] = collisionAvoidanceController(ego, target, route, cfg, stored);
            testCase.verifyTrue(next.metadata.planCertified);
            testCase.verifyGreaterThan(stored.prediction.egoStateErrorBound(4, 1), 0);
            testCase.verifyGreaterThan(stored.encounters.radius(1), 0);
            testCase.verifyGreaterThan(updated.stateErrorBound(4, end), 0);
            testCase.verifyGreaterThanOrEqual(updated.margin, stored.margin);
        end

        function aVerifiedEarlierExitEndsTheEncounterWithoutAnotherInput(testCase)
            [ego, target, route, cfg] = localFixture();
            ego.perception.range = 10;
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            ego.perception.range = 10;
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, stored);
            [ego, target] = localNext(stored, problem.model.lane, target);
            ego.perception.range = 10;
            [command, inputs, next, updated] = collisionAvoidanceController(ego, target, route, cfg, stored);
            testCase.verifyTrue(updated.encounterComplete);
            testCase.verifyLessThan(updated.stateTime, updated.deadline);
            testCase.verifyEmpty(command);
            testCase.verifyEmpty(inputs);
            testCase.verifyEmpty(next.metadata.activeTargetKeys);
        end

        function unsupportedDelayCannotAcquireTheZeroDelayGuarantee(testCase)
            [~, ~, ~, cfg] = localFixture();
            cfg.controller.inputDelaySteps = 1;
            testCase.verifyError(@() collisionAvoidanceControllerConfig(cfg), ...
                'collisionAvoidanceController:invalidConfiguration');
        end
    end
end

function [ego, target, route, cfg] = localFixture()
    cfg = collisionAvoidanceControllerConfig(struct('referenceSpeed', 8, ...
        'controller', struct('horizonSteps', 4, 'sampleTime', 0.1, 'certifiedSteps', Inf), ...
        'model', struct('linearizationPolicy', 'cruise', 'lateralDomainRadius', 2), ...
        'encounter', struct('completionPolicy', 'retainedPerceptionExit')));
    ego = struct('position', [0; 0], 'yaw', 0, 'speed', 8, 'stateTime', 0, ...
        'perception', struct('time', 0, 'range', 13.5, 'completeWithinRange', true));
    target = struct('trackId', 1, 'targetPositionInertial', [-8; 0], ...
        'targetVelocityInertial', [-8; 0], 'targetAccelerationInertial', [0; 0], ...
        'targetHeadingInertial', pi, 'targetYawRate', 0, ...
        'predictionMotion', struct('kind', 'finite-sensing-motion-v1', ...
        'jerkBound', [0; 0], 'yawAccelerationBound', 0));
    route = [-100, 0; 2000, 0];
end

function [ego, target] = localNext(stored, lane, target)
    ego = encounterTestFixture.nextEgo(stored, lane);
    ego.perception = struct('time', ego.stateTime, 'range', 13.5, 'completeWithinRange', true);
    flow = targetPrediction.finiteFlow(stored.encounters, ego.stateTime-stored.admissionTime);
    target = localTargetState(target, flow);
end

function target = localTargetState(target, state)
    target.targetPositionInertial = state(1:2);
    target.targetVelocityInertial = state(3:4);
    target.targetAccelerationInertial = state(5:6);
    target.targetHeadingInertial = state(7);
    target.targetYawRate = state(8);
end

function [history, commands] = localRunEncounter()
    [ego, target, route, cfg] = localFixture();
    [command, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
    history = repmat(stored, 1, 5);
    commands = cell(1, 5);
    commands{1} = command;
    for step = 1:4
        [ego, target] = localNext(stored, problem.model.lane, target);
        [command, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, stored);
        history(step+1) = stored;
        commands{step+1} = command;
    end
end

function solve = localFailPerformance(~, program)
    if numel(program.cones) > 2
        solve = encounterTestFixture.fail([], []);
    else
        solve = program.defaultSolver();
    end
end

function minimum = localDenseClearance(stored, lane, cfg)
    minimum = inf;
    x = stored.predictedState(:, 1);
    h = cfg.controller.sampleTime;
    for stage = 1:stored.remainingSteps
        generator = [stored.prediction.continuousA(:, :, stage), ...
            stored.prediction.continuousB(:, :, stage), stored.prediction.continuousC(:, stage); zeros(3, 9)];
        for tau = linspace(0, h, 41)
            value = expm(tau*generator)*[x; stored.plan(:, stage); 1];
            [position, heading] = laneGeometry.fromFrenet(value(1:6), lane);
            target = targetPrediction.finiteFlow(stored.encounters, (stage-1)*h+tau);
            distance = rectangleConfigurationDistance(position, heading, target(1:2), target(7), ...
                [cfg.vehicle.length/2; cfg.vehicle.width/2; stored.encounters.halfLength; stored.encounters.halfWidth]);
            minimum = min(minimum, distance-cfg.collision.clearanceMargin);
        end
        x = value(1:6);
    end
end
