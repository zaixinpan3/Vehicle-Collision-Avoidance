classdef hardEncounterBarrierTest < matlab.unittest.TestCase
    % Exact scheduled-plant guarantees; no perception or nonlinear-plant claim.
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (Test)
        function admissionIncludesAnIndefiniteTerminalCertificate(testCase)
            [ego,target,road,cfg] = localFixture();
            [command,~,problem,stored] = collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyNotEmpty(command);
            testCase.verifyFalse(problem.metadata.indefiniteRecursiveFeasibilityClaimed);
            testCase.verifyEqual(problem.metadata.terminalPolicyRole,"predictionWitnessOnly");
            testCase.verifyTrue(problem.metadata.terminalContinuationCertified);
            testCase.verifyTrue(problem.metadata.exactPredictionAssumptionsHold);
            testCase.verifyFalse(problem.metadata.physicalVehicleGuaranteeEstablished);
            testCase.verifyEqual(stored.certifiedDuration,inf);
            testCase.verifyGreaterThan(stored.margin,0);
            testCase.verifyGreaterThan(stored.acceptance.exitMargin,0);
            testCase.verifyEqual(stored.encounters.contract.validityScope,"allFutureTime");
        end

        function aFailedPerformanceSolveCannotDispatchTheFeasibilityLp(testCase)
            [ego,target,road,cfg] = encounterTestFixture.crossing();
            cfg.solver.jointFunction = @localPerformanceFailure;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:noCertifiedContinuation');
        end

        function aCruisingPredictionCanCarryAStoppingCertificate(testCase)
            [ego,target,road,cfg] = encounterTestFixture.crossing();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyGreaterThan(stored.predictedState(4,end),7.9);
            testCase.verifyGreaterThan(stored.acceptance.exitMargin,0);
            testCase.verifyFalse(problem.metadata.terminalActive);
            testCase.verifyEqual(problem.metadata.commandCertifiedDuration,cfg.controller.sampleTime);
        end

        function perceptionMetadataCannotChangeThePlan(testCase)
            [ego,target,road,cfg] = localFixture();
            [first,inputs] = collisionAvoidanceController(ego,target,road,cfg,[]);
            ego.perception = struct('range',NaN,'time',-100,'completeWithinRange',false);
            [second,other] = collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyEqual(second.actuatorInput,first.actuatorInput,AbsTol=1e-12);
            testCase.verifyEqual(other,inputs,AbsTol=1e-12);
        end

        function oneTargetIsRequiredEvenWhenItIsFarAway(testCase)
            [ego,~,road,cfg] = localFixture();
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:invalidExactScene');
        end

        function theStrictSceneRejectsAdditionalTargets(testCase)
            [ego,target,road,cfg] = localFixture();
            targets = [target;target];
            targets(2).trackId = 2;
            testCase.verifyError(@() collisionAvoidanceController(ego,targets,road,cfg,[]), ...
                'collisionAvoidanceController:invalidExactScene');
        end

        function egoModelErrorCannotBeSilentlyDiscarded(testCase)
            [ego,target,road,cfg] = localFixture();
            cfg.model.plantModelResidualRateBound(4) = 0.01;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:nonexactStudyInput');
        end

        function targetUncertaintyCannotBeSilentlyDiscarded(testCase)
            [ego,target,road,cfg] = localFixture();
            target.targetPositionInertialErrorBound = [0.1;0];
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:nonexactStudyInput');
        end

        function uncertainFutureMotionCannotBeCalledExact(testCase)
            [ego,target,road,cfg] = localFixture();
            target.predictionMotion = struct('kind','finite-sensing-motion-v1', ...
                'jerkBound',[0.1;0],'yawAccelerationBound',0);
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:nonexactStudyInput');
        end

        function aFailedSolveDoesNotExecuteTheTerminalWitness(testCase)
            [ego,target,road,cfg,stored] = localContinuation();
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,stored), ...
                'collisionAvoidanceController:noCertifiedContinuation');
        end

        function anUnsafeSolverDecisionTerminatesControl(testCase)
            [ego,target,road,cfg,stored] = localContinuation();
            cfg.solver.jointFunction = @localUnsafeSolve;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,stored), ...
                'collisionAvoidanceController:noCertifiedContinuation');
        end

        function targetIdentityCannotChangeBetweenFrames(testCase)
            [ego,target,road,cfg,stored] = localContinuation();
            target.trackId = 2;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,stored), ...
                'collisionAvoidanceController:changedEncounterContract');
        end

        function aTargetCannotDisappearAtAnyDistance(testCase)
            [ego,~,road,cfg,stored] = localContinuation();
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,stored), ...
                'collisionAvoidanceController:invalidExactScene');
        end

        function changedTargetMotionInvalidatesTheExactPremise(testCase)
            [ego,target,road,cfg,stored] = localContinuation();
            target.targetVelocityInertial(1) = target.targetVelocityInertial(1)+0.1;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,stored), ...
                'collisionAvoidanceController:inconsistentObservation');
        end

        function changedEgoMotionInvalidatesTheExactPremise(testCase)
            [ego,target,road,cfg,stored] = localContinuation();
            ego.position(1) = ego.position(1)+0.1;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,stored), ...
                'collisionAvoidanceController:inconsistentObservation');
        end

        function aDifferentAppliedCommandInvalidatesTheExecutionPremise(testCase)
            [ego,target,road,cfg,stored] = localContinuation();
            ego.heldActuatorInput(2) = ego.heldActuatorInput(2)+0.01;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,stored), ...
                'collisionAvoidanceController:executionContractViolation');
        end

        function aLateSampleDoesNotReceiveAnUnprovedCommand(testCase)
            [ego,target,road,cfg,stored] = localContinuation();
            ego.stateTime = ego.stateTime+0.01;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,stored), ...
                'collisionAvoidanceController:executionContractViolation');
        end

        function changingPhysicalConstraintsRequiresNewAdmission(testCase)
            [ego,target,road,cfg,stored] = localContinuation();
            cfg.collision.clearanceMargin = cfg.collision.clearanceMargin+0.1;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,stored), ...
                'collisionAvoidanceController:changedExecutionContract');
        end

        function finiteEncounterCertificatesCannotClaimTheNewGuarantee(testCase)
            [ego,target,road,cfg,stored] = localContinuation();
            stored.version = 16;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,stored), ...
                'collisionAvoidanceController:invalidStoredCertificate');
        end

        function aChangedStoredPlanIsRejected(testCase)
            [ego,target,road,cfg,stored] = localContinuation();
            stored.plan(1,1) = stored.plan(1,1)+0.1;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,stored), ...
                'collisionAvoidanceController:invalidStoredCertificate');
        end

        function terminalPoseBudgetsHaveANonincreasingComparisonBound(testCase)
            [~,~,~,stored] = localAdmit();
            terminal = stored.qp.terminal;
            testCase.verifyLessThan(terminal.comparison*terminal.velocityLimit,zeros(3,1));
            testCase.verifyLessThanOrEqual(terminal.poseExcursion*terminal.comparison ...
                +abs(terminal.continuousA(1:3,4:6)),1e-12*ones(3));
            testCase.verifyGreaterThan(terminal.longitudinalRatio,0);
            testCase.verifyLessThan(terminal.longitudinalRatio,1);
        end

        function terminalClosedFormMatchesIndependentHeldInputIntegration(testCase)
            [~,~,~,stored] = localAdmit();
            [error,minimumMargin] = localTerminalAudit(stored);
            testCase.verifyLessThan(error,1e-10);
            testCase.verifyGreaterThanOrEqual(minimumMargin,-1e-10);
        end

        function terminalWitnessRespectsEntryAndFutureActuatorRates(testCase)
            [ego,target,road,cfg] = localFixture();
            cfg.model.frontWheelSteeringRateMaximum = 0.5;
            cfg.model.brakingRatioRateMaximum = 2;
            [command,inputs,~,stored] = collisionAvoidanceController(ego,target,road,cfg,[]);
            limit = cfg.controller.sampleTime*[0.5;2];
            testCase.verifyLessThanOrEqual(abs(command.actuatorInput),limit+1e-12);
            terminal = stored.qp.terminal;
            states = hardEncounterBarrier.terminalFlow(terminal,stored.predictedState(:,end),0:25);
            tail = terminal.input+terminal.feedback*states;
            testCase.verifyLessThanOrEqual(abs(diff([inputs(:,end),tail],1,2)), ...
                repmat(limit,1,size(tail,2))+1e-12);
            testCase.verifyFalse(stored.metadata.terminalActive);
        end

        function terminalContractionDoesNotRequirePassiveRoadLoad(testCase)
            [ego,target,road,cfg] = localFixture();
            cfg.roadLoad.dragCoefficient = 0;
            cfg.roadLoad.rollingCoefficient = 0;
            [~,~,~,stored] = collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyEqual(stored.qp.terminal.continuousA(4,4),0,AbsTol=0);
            [error,margin] = localTerminalAudit(stored);
            testCase.verifyLessThan(error,1e-10);
            testCase.verifyGreaterThanOrEqual(margin,-1e-10);
        end

        function aRotatingTargetRetainsTheOriginalAbsoluteTimeMotion(testCase)
            [ego,target,road,cfg] = localFixture();
            target.targetYawRate = 0.2;
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,road,cfg,[]);
            for step = 1:3
                ego = encounterTestFixture.nextEgo(stored,problem.model.lane);
                target.targetHeadingInertial = 0.2*ego.stateTime;
                [~,~,problem,stored] = collisionAvoidanceController(ego,target,road,cfg,stored);
                testCase.verifyTrue(problem.metadata.planCertified);
                testCase.verifyEqual(stored.originalEncounter.center(7),0,AbsTol=0);
            end
            testCase.verifyFalse(problem.metadata.terminalActive);
        end

        function aNonzeroIndependentBiasNeedsADifferentTerminalCertificate(testCase)
            [ego,target,road,cfg] = localFixture();
            ego.longitudinalAccelerationBias = 0.1;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:invalidTerminalModel');
        end

        function anAcceleratingTargetIsCheckedBeyondTheTerminalTime(testCase)
            [ego,target,road,cfg] = localFixture();
            target.targetPositionInertial = [-20;0];
            target.targetVelocityInertial = [2;0];
            target.targetAccelerationInertial = [-1;0];
            [~,~,~,stored] = collisionAvoidanceController(ego,target,road,cfg,[]);
            normal = stored.qp.terminal.targetNormals;
            projectedMaximum = normal.'*[-18;0];
            testCase.verifyGreaterThanOrEqual(stored.qp.terminal.futureTargetSupports,projectedMaximum);
            testCase.verifyLessThan(stored.qp.terminal.futureTargetSupports-projectedMaximum,1e-8);
        end
    end
end

function [ego,target,road,cfg] = localFixture()
    [ego,target,road,cfg] = encounterTestFixture.crossing();
    ego = rmfield(ego,'perception');
    target = rmfield(target,'predictionMotion');
    target.targetPositionInertial = [1000;1000];
    target.targetVelocityInertial = [0;0];
    target.targetHeadingInertial = 0;
    cfg.solver.certificateSearchTimeLimit = 20;
end

function [command,inputs,problem,stored] = localAdmit()
    [ego,target,road,cfg] = localFixture();
    [command,inputs,problem,stored] = collisionAvoidanceController(ego,target,road,cfg,[]);
end

function [ego,target,road,cfg,stored] = localContinuation()
    [ego,target,road,cfg] = localFixture();
    [~,~,problem,stored] = collisionAvoidanceController(ego,target,road,cfg,[]);
    ego = encounterTestFixture.nextEgo(stored,problem.model.lane);
end

function result = localUnsafeSolve(~,program)
    result = struct('decision',100*ones(size(program.q)),'exitFlag',1,'output',struct());
end

function [error,minimumMargin] = localTerminalAudit(stored)
    terminal = stored.qp.terminal;
    initial = stored.predictedState(:,end);
    initial(4:6) = [0.1;0.01;-0.005];
    state = initial;
    error = 0;
    minimumMargin = inf;
    for step = 1:100
        input = terminal.input+terminal.feedback*state;
        generator = [terminal.continuousA,terminal.continuousB,terminal.continuousC;zeros(3,9)];
        for time = linspace(0,terminal.sampleTime,11)
            value = expm(time*generator)*[state;input;1];
            minimumMargin = min([minimumMargin;terminal.stateBound-terminal.stateRows*value(1:6)]);
        end
        state = value(1:6);
        expected = hardEncounterBarrier.terminalFlow(terminal,initial,step);
        error = max(error,norm(expected-state,inf));
    end
end

function result = localPerformanceFailure(~,program)
    if nnz(program.P)==0
        result = program.defaultSolver();
    else
        result = encounterTestFixture.fail([],[]);
    end
end
