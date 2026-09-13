classdef finiteEncounterCompletionTest < matlab.unittest.TestCase
    % Behavior of finite exit, observation release and road-only continuation.
    properties (TestParameter)
        verification = struct('transfer',"shiftedRows",'rebuilt',"rebuilt");
        violation = struct('physical',0.1,'belowSolverTolerance',1e-10);
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (Test)
        function twoAxisJerkAdmitsAFiniteWitnessWithAMovingEgoAtExit(testCase)
            [~,~,p,c] = localAdmit();
            testCase.verifyTrue(p.metadata.safetyCertified);
            testCase.verifyEqual(p.metadata.pcbfValue,0,AbsTol=0);
            testCase.verifyEqual(p.metadata.targetCertifiedUntil,1.6,AbsTol=1e-12);
            testCase.verifyEqual(p.metadata.certifiedDuration,1.6,AbsTol=1e-12);
            testCase.verifyGreaterThan(c.predictedState(4,end),7.9);
            testCase.verifyTrue(c.terminal.targetIndependent);
            testCase.verifyEmpty(c.terminal.targetNormals);
            testCase.verifyFalse(p.metadata.indefiniteRecursiveFeasibilityClaimed);
            testCase.verifyEqual(c.encounters.contract.validityScope,"whileEncounterActive");
        end

        function freshWitnessesCannotPostponeAnActiveExit(testCase)
            deadlines = localFreshDeadlines();
            testCase.verifyLessThanOrEqual(diff(deadlines),1e-12);
            testCase.verifyGreaterThan(deadlines,0);
        end

        function aConfirmedExteriorObservationKeepsTheSuffixWhenSolvingFails(testCase,verification)
            [ego,target,road,cfg,c,p] = localBefore(7,verification,true);
            [command,~,next,stored] = collisionAvoidanceController(ego,target,road,cfg,c);
            testCase.verifyTrue(next.metadata.confirmedRelease);
            testCase.verifyTrue(next.metadata.candidateVerified);
            testCase.verifyEqual(next.metadata.certificateSource,"carriedWitness");
            testCase.verifyEqual(command.actuatorInput,p.inputPlan(:,2),AbsTol=1e-12);
            testCase.verifyEmpty(stored.encounters);
            testCase.verifyTrue(next.metadata.roadTailCertified);
            testCase.verifyEqual(next.metadata.pcbfValue,0,AbsTol=0);
        end

        function completeCurrentAbsenceCanReleaseWithoutAFreshSolve(testCase)
            [ego,~,road,cfg,c] = localBefore(7,"shiftedRows",true);
            [~,~,p,next] = collisionAvoidanceController(ego,[],road,cfg,c);
            testCase.verifyTrue(p.metadata.confirmedRelease);
            testCase.verifyTrue(p.metadata.candidateVerified);
            testCase.verifyEmpty(next.encounters);
        end

        function absenceContradictingTheCarriedSetCannotRelease(testCase)
            [ego,~,road,cfg,c] = localBefore(1,"shiftedRows",true);
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,c), ...
                'collisionAvoidanceController:inconsistentObservation');
        end

        function aStaleScanCannotDischargeAnAbsentTarget(testCase)
            [ego,~,road,cfg,c] = localBefore(7,"shiftedRows",true);
            ego.perception.time = 0;
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,c), ...
                'collisionAvoidanceController:unconfirmedTargetDeparture');
        end

        function changingTheRegionCannotDischargeTheEncounter(testCase)
            [ego,~,road,cfg,c] = localBefore(7,"shiftedRows",true);
            ego.perception.range = 10;
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,c), ...
                'collisionAvoidanceController:changedConfirmationRegion');
        end

        function aTimerNeverReleasesWithoutCurrentConfirmation(testCase,verification)
            [ego,target,road,cfg,c] = localBefore(16,verification,false);
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,c), ...
                'collisionAvoidanceController:unconfirmedEncounterExit');
        end

        function theLastConfirmationSampleCanEnterTheRoadTerminalLaw(testCase,verification)
            [ego,target,road,cfg,c] = localBefore(16,verification,false);
            ego.perception.completeWithinRange = true;
            [command,~,p,next] = collisionAvoidanceController(ego,target,road,cfg,c);
            testCase.verifyTrue(p.metadata.confirmedRelease);
            testCase.verifyTrue(p.metadata.terminalActive);
            testCase.verifyTrue(p.metadata.safetyCertified);
            testCase.verifyEmpty(next.encounters);
            testCase.verifyEqual(command.actuatorInput, ...
                c.terminal.input+c.terminal.feedback*c.predictedState(:,2),AbsTol=1e-12);
        end

        function anUnconfirmedFiniteAdmissionIsRejected(testCase)
            [ego,target,road,cfg] = encounterTestFixture.crossing();
            ego = rmfield(ego,'perception');
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:unconfirmedTargetDeparture');
        end

        function positiveViolationIsDiagnosticEvenBelowSolverTolerance(testCase,violation)
            [~,~,p] = localAdmit();
            qp = p.qp;
            row = find(qp.safetyRows,1);
            qp.physicalBound(row) = qp.inequalityMatrix(row,:)*p.decision-violation;
            check = solveHardCbfClf.certify(qp,p.prediction,p.model,p.decision);
            testCase.verifyTrue(check.candidateAccepted);
            testCase.verifyFalse(check.safetyCertified);
            testCase.verifyFalse(check.accepted);
            testCase.verifyGreaterThan(check.value,0);
            testCase.verifyEqual(check.failedConditions,"positiveSafetyViolation");
        end

        function aPositiveValueCandidateCannotAuthorizeACommand(testCase)
            [ego,target,road,cfg] = encounterTestFixture.crossing();
            target.targetPositionInertial = [2;-1];
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:noCertifiedContinuation');
        end

        function zeroJerkDoesNotExtendTheModelScopeToInfinity(testCase)
            [ego,target,road,cfg] = encounterTestFixture.crossing();
            [~,lane,~,parsed] = readPlanningInputs(ego,target,road,cfg);
            admitted = targetPrediction.admitOnline(parsed,0,lane,cfg);
            testCase.verifyEqual(admitted.contract.validityScope,"whileEncounterActive");
        end

        function aNominalExitCannotSpendAnUnobservedFutureUncertaintyReset(testCase)
            [~,~,p] = localAdmit();
            model = p.model;
            model.encounters.radius(1:2) = [100;100];
            qp = formulateAvoidanceProblem(model,p.prediction,p.inputPlan(:));
            check = solveHardCbfClf.certify(qp,p.prediction,model,p.decision);
            testCase.verifyFalse(check.safetyCertified);
            testCase.verifyLessThan(check.exitMargin,0);
        end

        function finalConfirmationUsesTheCertifiedDirectionForAnisotropicBoxes(testCase)
            [~,~,p,c] = localAdmit();
            target = c.encounters;
            target.center(1:2) = [100;20];
            target.center(7) = 0;
            target.radius(1:2) = [100;0];
            completion = c.completion;
            completion.direction = [0;1];
            testCase.verifyFalse(hardEncounterBarrier.observedExterior(p.model,target));
            testCase.verifyTrue(hardEncounterBarrier.observedExterior(p.model,target,completion));
        end

        function anAlteredUnexecutedInputCannotUseStoredAcceptance(testCase)
            [ego,target,road,cfg,c] = localBefore(1,"shiftedRows",true);
            c.plan(2,2) = c.plan(2,2)+0.01;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,c), ...
                'collisionAvoidanceController:invalidStoredCertificate');
        end
    end
end

function [command,inputs,p,c,ego,target,road,cfg] = localAdmit(verification)
    if nargin<1, verification = "shiftedRows"; end
    [ego,target,road,cfg] = encounterTestFixture.crossing();
    target.predictionMotion.jerkBound = [0.1;0.1];
    target.predictionMotion.yawAccelerationBound = 0.05;
    cfg.solver.witnessVerification = verification;
    [command,inputs,p,c] = collisionAvoidanceController(ego,target,road,cfg,[]);
end

function [ego,target,road,cfg,c,p] = localBefore(step,verification,confirmEarlier)
    [~,~,p,c,~,target,road,cfg] = localAdmit(verification);
    cfg.solver.jointFunction = @encounterTestFixture.fail;
    for index = 1:step
        ego = encounterTestFixture.nextEgo(c,p.model.lane);
        ego.perception.completeWithinRange = confirmEarlier;
        target.targetPositionInertial = [15;-4]+[0;32]*ego.stateTime;
        if index<step
            [~,~,p,c] = collisionAvoidanceController(ego,target,road,cfg,c);
        end
    end
end

function deadlines = localFreshDeadlines()
    [~,~,p,c,~,target,road,cfg] = localAdmit();
    deadlines = zeros(1,6);
    deadlines(1) = c.completion.deadline;
    for step = 1:5
        ego = encounterTestFixture.nextEgo(c,p.model.lane);
        target.targetPositionInertial = [15;-4]+[0;32]*ego.stateTime;
        [~,~,p,c] = collisionAvoidanceController(ego,target,road,cfg,c);
        deadlines(step+1) = c.completion.deadline;
    end
end
