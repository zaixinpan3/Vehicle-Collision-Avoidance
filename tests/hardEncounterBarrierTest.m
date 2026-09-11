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
        function targetFreeAdmissionRenewsWithoutClaimingEncounterCompletion(testCase)
            [ego,~,route,cfg] = localFixture();
            [~,~,problem,stored] = collisionAvoidanceController(ego,[],route,cfg,[]);
            deadline = stored.deadline;
            for index = 1:cfg.controller.horizonSteps-1
                ego = encounterTestFixture.nextEgo(stored,problem.model.lane);
                ego.perception.completeWithinRange = true;
                [~,~,problem,stored] = collisionAvoidanceController(ego,[],route,cfg,stored);
                testCase.verifyEqual(stored.deadline,deadline,AbsTol=0);
                testCase.verifyEqual(stored.remainingSteps,cfg.controller.horizonSteps-index);
            end
            ego = encounterTestFixture.nextEgo(stored,problem.model.lane);
            ego.perception.completeWithinRange = true;
            [command,~,problem,stored] = collisionAvoidanceController(ego,[],route,cfg,stored);
            testCase.verifyFalse(stored.encounterComplete);
            testCase.verifyNotEmpty(command);
            testCase.verifyGreaterThan(stored.deadline,deadline);
            testCase.verifyFalse(problem.metadata.recursiveFeasibilityClaimed);
        end

        function firstDetectionCanAugmentATargetFreeCertificate(testCase)
            [ego,target,route,cfg] = localFixture();
            [~,~,problem,stored] = collisionAvoidanceController(ego,[],route,cfg,[]);
            ego = encounterTestFixture.nextEgo(stored,problem.model.lane);
            ego.perception.completeWithinRange = true;
            target.targetPositionInertial = [-8.8;0];
            [command,~,next,updated] = collisionAvoidanceController(ego,target,route,cfg,stored);
            testCase.verifyNotEmpty(command);
            testCase.verifyTrue(next.metadata.jointAdmissionPerformed);
            testCase.verifyEqual(updated.admissionTime,ego.stateTime,AbsTol=0);
            testCase.verifyGreaterThan(updated.deadline,stored.deadline);
            testCase.verifyEqual(numel(updated.encounters),1);
        end

        function removedModeSelectorsCannotRestoreAnOldController(testCase)
            for field = ["completionPolicy","reoptimizeContinuation","safetyMarginPolicy", ...
                    "barrierFraction","maneuverSelectionPolicy","corridorOverlap","maneuverSwitchWeight"]
                override = struct("encounter",struct(field,"lookahead"));
                testCase.verifyError(@() collisionAvoidanceControllerConfig(override), ...
                    "collisionAvoidanceController:invalidConfiguration");
            end
        end

        function removedForcePolygonSettingCannotBeReenabled(testCase)
            testCase.verifyError(@() collisionAvoidanceControllerConfig( ...
                struct("model",struct("frictionPolygonSides",12))), ...
                "collisionAvoidanceController:invalidConfiguration");
        end

        function earlierCertificateVersionsCannotResumeTheController(testCase)
            [ego,target,route,cfg] = localFixture();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            [ego,target] = localNext(stored,problem.model.lane,target);
            stored.version = 15;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,route,cfg,stored), ...
                "collisionAvoidanceController:invalidStoredCertificate");
        end

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

        function finiteAdmissionDoesNotClaimAnIndefiniteSuccessor(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);

            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyTrue(problem.metadata.recursiveFeasibilityClaimed);
            testCase.verifyEqual(problem.metadata.recursiveFeasibilityScope, ...
                "untilVerifiedPerceptionExitForDeclaredInclusion");
            testCase.verifyFalse(problem.metadata.indefiniteRecursiveFeasibilityClaimed);
            testCase.verifyFalse(problem.metadata.terminalContinuationCertified);
            testCase.verifyFalse(stored.metadata.indefiniteRecursiveFeasibilityClaimed);
        end

        function retainedContinuationKeepsTheFiniteGuaranteeScope(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            [command, ~, next] = collisionAvoidanceController(ego, target, route, cfg, stored);

            testCase.verifyNotEmpty(command);
            testCase.verifyTrue(next.metadata.carriedWitnessFeasible);
            testCase.verifyFalse(next.metadata.indefiniteRecursiveFeasibilityClaimed);
            testCase.verifyFalse(next.metadata.terminalContinuationCertified);
        end

        function aSafePrefixIsNotExecutedWhenCompleteWitnessSearchIsUnresolved(testCase)
            [ego, target, route, cfg] = localFixture();
            target.targetVelocityInertial = [8; 0];
            target.targetHeadingInertial = 0;
            cfg.solver.certificateSearchTimeLimit = 0.01;
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, []), ...
                'collisionAvoidanceController:certificateSearchLimit');
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

        function optimizerFailureStillExecutesTheCertifiedWitness(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            [command, ~, next, updated] = collisionAvoidanceController(ego, target, route, cfg, stored);
            testCase.verifyEqual(command.actuatorInput, stored.plan(:, 2), AbsTol=0);
            testCase.verifyGreaterThan(next.metadata.solverCallCount, 0);
            testCase.verifyEqual(next.metadata.certificateSource, "retainedCertifiedWitness");
            testCase.verifyGreaterThanOrEqual(updated.margin, stored.margin);
        end

        function failedReoptimizationKeepsTheCertifiedTail(testCase)
            [ego, target, route, cfg] = localFixture();
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
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            [~, ~, next, updated] = collisionAvoidanceController(ego, target, route, cfg, stored);
            testCase.verifyEqual(updated.decision(1:2), stored.appliedInput, AbsTol=0);
            testCase.verifyGreaterThanOrEqual(updated.margin, stored.margin);
            testCase.verifyTrue(next.metadata.carriedWitnessFeasible);
        end

        function aSolverCannotChangeAnExecutedInput(testCase)
            [ego, target, route, cfg] = localFixture();
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

        function feasibleNewTargetsReceiveAJointCertificate(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            nextTarget = target;
            nextTarget.trackId = 2;
            nextTarget.targetPositionInertial(2) = 3;
            [command, ~, next, updated] = collisionAvoidanceController(ego, [target; nextTarget], route, cfg, stored);
            testCase.verifyNotEmpty(command);
            testCase.verifyTrue(next.metadata.jointAdmissionPerformed);
            testCase.verifyEqual(next.metadata.newlyAdmittedTargetKeys, "trackId:2");
            testCase.verifyEqual(next.metadata.activeTargetKeys, ["trackId:1", "trackId:2"]);
            testCase.verifyEqual(updated.deadline, stored.deadline, AbsTol=0);
            testCase.verifyEqual(updated.decision(1:2), stored.appliedInput, AbsTol=0);
            testCase.verifyGreaterThanOrEqual(updated.margin, 0);
            testCase.verifyGreaterThan(updated.acceptance.exitMargin, 0);
            testCase.verifyLessThanOrEqual(stored.qp.inequalityMatrix*updated.decision, stored.qp.physicalBound);
        end

        function anUncertifiedNewTargetCannotUseTheOldTargetOnlyPlan(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            nextTarget = target;
            nextTarget.trackId = 2;
            nextTarget.targetPositionInertial = ego.position;
            testCase.verifyError(@() collisionAvoidanceController(ego, [target; nextTarget], route, cfg, stored), ...
                'collisionAvoidanceController:jointAdmissionNotCertified');
        end

        function jointAdmissionCannotOmitARetainedInRangeTarget(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            target.trackId = 2;
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, stored), ...
                'collisionAvoidanceController:inconsistentPerception');
        end

        function solverFailureCanRetainAnIndependentlyCheckedJointWitness(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            nextTarget = target;
            nextTarget.trackId = 2;
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            [command, ~, next, updated] = collisionAvoidanceController(ego, [target; nextTarget], route, cfg, stored);
            testCase.verifyEqual(command.actuatorInput, stored.plan(:, 2), AbsTol=0);
            testCase.verifyTrue(next.metadata.jointAdmissionPerformed);
            testCase.verifyTrue(updated.acceptance.accepted);
            testCase.verifyEqual(numel(updated.encounters), 2);
        end

        function solverFailureCannotRetainAWitnessThatViolatesANewTarget(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            nextTarget = target;
            nextTarget.trackId = 2;
            nextTarget.targetPositionInertial = ego.position;
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            testCase.verifyError(@() collisionAvoidanceController(ego, [target; nextTarget], route, cfg, stored), ...
                'collisionAvoidanceController:jointAdmissionNotCertified');
        end

        function aNewTargetsForecastStartsAtItsDetectionTime(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            nextTarget = target;
            nextTarget.trackId = 2;
            [~, ~, next, updated] = collisionAvoidanceController(ego, [target; nextTarget], route, cfg, stored);
            nextEgo = encounterTestFixture.nextEgo(updated, next.model.lane);
            nextEgo.perception = struct('time', nextEgo.stateTime, 'range', 13.5, 'completeWithinRange', true);
            oldFlow = targetPrediction.finiteFlow(updated.encounters(1), nextEgo.stateTime-updated.encounters(1).time);
            newFlow = targetPrediction.finiteFlow(updated.encounters(2), nextEgo.stateTime-updated.encounters(2).time);
            observations = [localTargetState(target, oldFlow); localTargetState(nextTarget, newFlow)];
            [command, ~, continuation, continued] = collisionAvoidanceController(nextEgo, observations, route, cfg, updated);
            testCase.verifyNotEmpty(command);
            testCase.verifyFalse(continuation.metadata.jointAdmissionPerformed);
            testCase.verifyGreaterThanOrEqual(continued.margin, updated.margin);
            testCase.verifyEqual(continued.deadline, stored.deadline, AbsTol=0);
        end

        function aNewTargetCannotRenewTheDeadlineToHideAnUncertifiedExit(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            nextTarget = target;
            nextTarget.trackId = 2;
            nextTarget.targetPositionInertial = ego.position+[8; 0];
            nextTarget.targetVelocityInertial = [8; 0];
            nextTarget.targetHeadingInertial = 0;
            testCase.verifyError(@() collisionAvoidanceController(ego, [target; nextTarget], route, cfg, stored), ...
                'collisionAvoidanceController:jointAdmissionNotCertified');
        end

        function aNewTargetsUncertaintyIsIncludedInTheJointCertificate(testCase)
            [ego, target, route, cfg] = localFixture();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [ego, target] = localNext(stored, problem.model.lane, target);
            target.targetPositionInertialErrorBound = [0.001; 0.001];
            nextTarget = target;
            nextTarget.trackId = 2;
            nextTarget.predictionMotion.jerkBound = [0.1; 0.1];
            [~, ~, next, updated] = collisionAvoidanceController(ego, [target; nextTarget], route, cfg, stored);
            [~, radius] = targetPrediction.finiteFlow(updated.encounters(2), updated.certifiedDuration);
            testCase.verifyTrue(next.metadata.jointAdmissionPerformed);
            testCase.verifyGreaterThan(radius(1:2), [0.001; 0.001]);
            testCase.verifyGreaterThan(updated.acceptance.exitMargin, 0);
            testCase.verifyGreaterThanOrEqual(updated.margin, 0);
        end

        function repeatedJointAdmissionsCompleteAtTheOriginalDeadline(testCase)
            [history, commands] = localRunJointEncounter();
            testCase.verifyEqual([history.deadline], repmat(0.4, 1, 5), AbsTol=1e-12);
            testCase.verifyEqual([history.remainingSteps], 4:-1:0);
            testCase.verifyEqual(numel(history(end).encounters), 4);
            testCase.verifyTrue(history(end).encounterComplete);
            testCase.verifyEmpty(commands{end});
            testCase.verifyGreaterThanOrEqual([history.margin], zeros(1, 5));
            testCase.verifyGreaterThanOrEqual(history(end).margin, history(end-1).margin);
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
        'controller', struct('horizonSteps', 4, 'sampleTime', 0.1), ...
        'model', struct('linearizationPolicy', 'cruise', 'lateralDomainRadius', 2)));
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

function [history, commands] = localRunJointEncounter()
    [ego, template, route, cfg] = localFixture();
    [command, ~, problem, stored] = collisionAvoidanceController(ego, template, route, cfg, []);
    history = repmat(stored, 1, 5);
    commands = cell(1, 5);
    commands{1} = command;
    for step = 1:4
        ego = encounterTestFixture.nextEgo(stored, problem.model.lane);
        ego.perception = struct('time', ego.stateTime, 'range', 13.5, 'completeWithinRange', true);
        targets = repmat(template, numel(stored.encounters)+double(step < 4), 1);
        for index = 1:numel(stored.encounters)
            encounter = stored.encounters(index);
            targets(index) = localTargetState(template, ...
                targetPrediction.finiteFlow(encounter, ego.stateTime-encounter.time));
            targets(index).trackId = index;
        end
        if step < 4
            targets(end) = targets(1);
            targets(end).trackId = numel(targets);
        end
        [command, ~, problem, stored] = collisionAvoidanceController(ego, targets, route, cfg, stored);
        history(step+1) = stored;
        commands{step+1} = command;
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
            distance = avoidanceSafetyGeometry.rectangleDistance(position, heading, target(1:2), target(7), ...
                [cfg.vehicle.length/2; cfg.vehicle.width/2; stored.encounters.halfLength; stored.encounters.halfWidth]);
            minimum = min(minimum, distance-cfg.collision.clearanceMargin);
        end
        x = value(1:6);
    end
end
