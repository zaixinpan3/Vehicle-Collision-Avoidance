classdef hardEncounterBarrierTest < matlab.unittest.TestCase
    % Declared-plant guarantees with a carried witness; no perception or nonlinear-plant claim.
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (Test)
        function admissionCarriesAVerifiedWitnessAndClaimsRecursiveFeasibility(testCase)
            [ego,target,road,cfg] = localFixture();
            [command,~,problem,stored] = collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyNotEmpty(command);
            testCase.verifyTrue(problem.metadata.recursiveFeasibilityClaimed);
            testCase.verifyTrue(problem.metadata.indefiniteRecursiveFeasibilityClaimed);
            testCase.verifyEqual(problem.metadata.terminalPolicyRole,"carriedWitnessTail");
            testCase.verifyTrue(problem.metadata.terminalContinuationCertified);
            testCase.verifyFalse(problem.metadata.physicalVehicleGuaranteeEstablished);
            testCase.verifyEqual(problem.metadata.certificateSource,"checkedOptimization");
            testCase.verifyEqual(problem.metadata.pcbfValue,0);
            testCase.verifyEqual(stored.version,19);
            testCase.verifyEqual(stored.certifiedDuration,inf);
            testCase.verifyEqual(numel(stored.stages),stored.remainingSteps);
            testCase.verifyEqual(numel(stored.cellFrames),numel(stored.prediction.cells));
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

        function aTargetPositionBoxIsAdmittedAndTightensTheTerminalSupport(testCase)
            [ego,target,road,cfg] = localFixture();
            [~,~,~,exact] = collisionAvoidanceController(ego,target,road,cfg,[]);
            target.targetPositionInertialErrorBound = [0.1;0];
            [~,~,problem,boxed] = collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyEqual(boxed.encounters.radius(1),0.1,AbsTol=0);
            testCase.verifyEqual(boxed.qp.terminal.targetNormals,exact.qp.terminal.targetNormals,AbsTol=1e-9);
            testCase.verifyGreaterThanOrEqual(boxed.qp.terminal.futureTargetSupports, ...
                exact.qp.terminal.futureTargetSupports+0.1*abs(exact.qp.terminal.targetNormals(1))-1e-9);
        end

        function uncertainFutureMotionCannotBeCalledExact(testCase)
            [ego,target,road,cfg] = localFixture();
            target.predictionMotion = struct('kind','finite-sensing-motion-v1', ...
                'jerkBound',[0.1;0],'yawAccelerationBound',0);
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:nonexactStudyInput');
        end

        function aFailedFreshSolveExecutesTheCarriedWitness(testCase)
            [ego,target,road,cfg,stored] = localContinuation();
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            [command,~,problem,next] = collisionAvoidanceController(ego,target,road,cfg,stored);
            testCase.verifyEqual(problem.metadata.certificateSource,"carriedWitness");
            testCase.verifyEqual(command.actuatorInput,stored.plan(:,2),AbsTol=0);
            testCase.verifyEqual(next.remainingSteps,stored.remainingSteps-1);
            testCase.verifyFalse(problem.metadata.terminalActive);
            testCase.verifyFalse(problem.metadata.fallbackUsed);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyLessThanOrEqual(problem.metadata.pcbfDescentResidual,0);
            testCase.verifyTrue(startsWith(problem.metadata.freshSolveFailure, ...
                "collisionAvoidanceController:noCertifiedContinuation"));
        end

        function anUnsafeSolverDecisionIsRejectedInFavourOfTheCarriedWitness(testCase)
            [ego,target,road,cfg,stored] = localContinuation();
            cfg.solver.jointFunction = @localUnsafeSolve;
            [command,~,problem] = collisionAvoidanceController(ego,target,road,cfg,stored);
            testCase.verifyEqual(problem.metadata.certificateSource,"carriedWitness");
            testCase.verifyEqual(command.actuatorInput,stored.plan(:,2),AbsTol=0);
            testCase.verifyEqual(problem.metadata.hardRowViolation,0);
        end

        function theTerminalLawIsCommandedOnlyAfterEveryOptimizedStageIsConsumed(testCase)
            [ego,target,road,cfg] = localFixture();
            cfg.controller.horizonSteps = 6;
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,road,cfg,[]);
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            initialSpeed = stored.predictedState(4,1);
            steps = stored.remainingSteps;
            for step = 1:steps+3
                ego = encounterTestFixture.nextEgo(stored,problem.model.lane);
                [~,~,problem,stored] = collisionAvoidanceController(ego,target,road,cfg,stored);
                testCase.verifyEqual(problem.metadata.certificateSource,"carriedWitness");
                testCase.verifyTrue(problem.metadata.planCertified);
                testCase.verifyEqual(stored.remainingSteps,max(0,steps-step));
                testCase.verifyEqual(problem.metadata.terminalActive,stored.remainingSteps==0);
                testCase.verifyEqual(problem.metadata.pcbfValue,0);
            end
            testCase.verifyEqual(problem.metadata.verificationMethod,"terminalInvariance");
            testCase.verifyLessThan(stored.predictedState(4,1),initialSpeed);
        end

        function theCarriedWitnessIsVerifiedAtEveryNoisyFrame(testCase)
            [ego,target,road,cfg] = encounterTestFixture.crossing();
            ego = rmfield(ego,'perception');
            target = rmfield(target,'predictionMotion');
            bound = [0.05;0.05;0.005;0.05;0.02;0.005];
            ego.controllerStateErrorBound = bound;
            target.targetPositionInertialErrorBound = [0.1;0.1];
            target.targetYawErrorBound = 0.01;
            stream = RandStream('mt19937ar','Seed',11);
            [command,~,problem,stored] = collisionAvoidanceController(ego,target,road,cfg,[]);
            truth = problem.model.initialEgoState;
            truthTarget = stored.originalEncounter;
            truthTarget.radius(:) = 0;
            h = cfg.controller.sampleTime;
            for step = 1:8
                generator = [problem.metadata.executedContinuousGenerator;zeros(3,9)];
                value = expm(h*generator)*[truth;command.actuatorInput;1];
                truth = value(1:6);
                [position,heading] = laneGeometry.fromFrenet(truth,problem.model.lane);
                noise = bound.*(2*rand(stream,6,1)-1);
                ego = struct('position',position+noise(1:2),'yaw',heading+noise(3),'speed',truth(4)+noise(4), ...
                    'lateralVelocity',truth(5)+noise(5),'yawRate',truth(6)+noise(6),'stateTime',step*h, ...
                    'heldActuatorInput',command.actuatorInput,'controllerStateErrorBound',bound);
                state = targetPrediction.finiteFlow(truthTarget,ego.stateTime-truthTarget.time);
                target.targetPositionInertial = state(1:2)+[0.1;0.1].*(2*rand(stream,2,1)-1);
                target.targetVelocityInertial = state(3:4);
                target.targetAccelerationInertial = state(5:6);
                target.targetHeadingInertial = state(7)+0.01*(2*rand(stream)-1);
                target.targetYawRate = state(8);
                [command,~,problem,stored] = collisionAvoidanceController(ego,target,road,cfg,stored);
                metadata = problem.metadata;
                testCase.verifyTrue(metadata.candidateVerified);
                testCase.verifyEqual(metadata.candidateValue,0);
                testCase.verifyLessThanOrEqual(metadata.pcbfDescentResidual,0);
                testCase.verifyEqual(metadata.pcbfValue,0);
                testCase.verifyLessThanOrEqual(metadata.initialErrorBound,bound+1e-12);
                testCase.verifyTrue(all(abs(truth-problem.model.initialEgoState) ...
                    <=problem.model.initialFrenetErrorBound+1e-9));
                testCase.verifyLessThanOrEqual(stored.encounters.radius(1:2),[0.1;0.1]+1e-12);
            end
        end

        function terminalMembershipIsMonotoneUnderBoxInclusion(testCase)
            [ego,target,road,cfg] = localFixture();
            ego.controllerStateErrorBound = [0.05;0.05;0.005;0.05;0.02;0.005];
            [~,~,~,stored] = collisionAvoidanceController(ego,target,road,cfg,[]);
            terminal = stored.terminal;
            [center,radius] = localExactTerminalNode(stored);
            testCase.verifyTrue(hardEncounterBarrier.terminalMembership(terminal,center,radius));
            testCase.verifyLessThanOrEqual(terminal.poseBudget,terminal.poseErrorBudget+1e-12);
            shrunk = 0.5*radius;
            stream = RandStream('mt19937ar','Seed',3);
            for trial = 1:20
                shift = (radius-shrunk).*(2*rand(stream,6,1)-1);
                testCase.verifyTrue(hardEncounterBarrier.terminalMembership(terminal,center+shift,shrunk));
            end
        end

        function aRobustTerminalBoxStaysSafeForNominalPlusErrorVertices(testCase)
            [ego,target,road,cfg] = localFixture();
            ego.controllerStateErrorBound = [0.05;0.05;0.005;0.05;0.02;0.005];
            [~,~,~,stored] = collisionAvoidanceController(ego,target,road,cfg,[]);
            terminal = stored.terminal;
            [center,radius] = localExactTerminalNode(stored);
            testCase.verifyTrue(hardEncounterBarrier.terminalMembership(terminal,center,radius));
            testCase.verifyTrue(terminal.errorBudgetFinite);
            worst = localRobustTerminalAudit(terminal,center,radius);
            testCase.verifyGreaterThanOrEqual(worst,-1e-9);
        end

        function aPositiveValueFunctionReportsPredictedViolation(testCase)
            [ego,target,road,cfg] = encounterTestFixture.crossing();
            target = rmfield(target,'predictionMotion');
            target.targetPositionInertial = [2;-1];
            [~,~,problem] = collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyGreaterThan(problem.metadata.pcbfValue,0);
            testCase.verifyGreaterThan(problem.metadata.stageViolation(1),0);
            testCase.verifyEqual(problem.metadata.hardRowViolation,0);
            testCase.verifyEqual(problem.metadata.barrierInterpretation,"verifiedAccumulatedSafetyViolation");
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

        function earlierCertificateVersionsCannotClaimTheCarriedWitness(testCase)
            [ego,target,road,cfg,stored] = localContinuation();
            stored.version = 18;
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
            testCase.verifyLessThan(terminal.errorComparison*terminal.velocityLimit,zeros(3,1));
            testCase.verifyLessThanOrEqual(terminal.poseExcursion*terminal.comparison ...
                +abs(terminal.continuousA(1:3,4:6)),1e-12*ones(3));
            testCase.verifyLessThanOrEqual(terminal.errorExcursion*terminal.errorComparison ...
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
            [node,~] = localExactTerminalNode(stored);
            states = hardEncounterBarrier.terminalFlow(terminal,node,0:25);
            tail = terminal.input+terminal.feedback*states;
            testCase.verifyLessThanOrEqual(abs(diff([inputs(:,end),tail],1,2)), ...
                repmat(limit,1,size(tail,2))+1e-12);
            testCase.verifyFalse(stored.metadata.terminalActive);
        end

        function terminalContractionDoesNotRequirePassiveRoadLoadForAnExactState(testCase)
            [ego,target,road,cfg] = localFixture();
            cfg.roadLoad.dragCoefficient = 0;
            cfg.roadLoad.rollingCoefficient = 0;
            [~,~,~,stored] = collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyEqual(stored.qp.terminal.continuousA(4,4),0,AbsTol=0);
            testCase.verifyFalse(stored.qp.terminal.errorBudgetFinite);
            [error,margin] = localTerminalAudit(stored);
            testCase.verifyLessThan(error,1e-10);
            testCase.verifyGreaterThanOrEqual(margin,-1e-10);
        end

        function admissionNeedsAnInDomainSpeedCentreEvenWhenTheBoxMeetsTheDomain(testCase)
            % Only continuation frames carry a successor box to condition
            % against; admission therefore rejects a negative measured centre.
            [ego,target,road,cfg] = localFixture();
            ego.speed = -0.02;
            ego.controllerStateErrorBound = [0;0;0;0.05;0;0];
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:invalidInput');
        end

        function anUncertainVelocityNeedsRoadLoadDampingForItsErrorBudget(testCase)
            [ego,target,road,cfg] = localFixture();
            cfg.roadLoad.dragCoefficient = 0;
            cfg.roadLoad.rollingCoefficient = 0;
            ego.controllerStateErrorBound = [0;0;0;0.01;0;0];
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:invalidTerminalModel');
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

        function aTargetVelocityBoxMakesTheAllFutureSupportUnbounded(testCase)
            [ego,target,road,cfg] = localFixture();
            target.targetVelocityInertialErrorBound = [0.01;0.01];
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:unboundedTargetSupport');
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

function [center,radius] = localExactTerminalNode(stored)
% The terminal node through the exact stage flows, as completionRows uses it.
    prediction = stored.prediction;
    plan = stored.plan(:);
    center = stored.witnessModel.initialEgoState;
    for stage = 1:prediction.stageCount
        center = prediction.stageMatrixA(:,:,stage)*center ...
            +prediction.stageMatrixB(:,:,stage)*plan(2*stage-1:2*stage)+prediction.stageAffine(:,stage);
    end
    radius = prediction.initialErrorBound(:,end);
end

function [error,minimumMargin] = localTerminalAudit(stored)
    terminal = stored.qp.terminal;
    initial = localExactTerminalNode(stored);
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

function worst = localRobustTerminalAudit(terminal,center,radius)
% True state = nominal + error; the law acts on the nominal only. Check the
% physical pose rows and the velocity box on the true state at eleven points
% of every hold for sixty holds from every vertex of the terminal box.
    generator = [terminal.continuousA,terminal.continuousB,terminal.continuousC;zeros(3,9)];
    fractions = linspace(0,terminal.sampleTime,11);
    flows = arrayfun(@(t) expm(t*generator),fractions,UniformOutput=false);
    signs = 2*double(dec2bin(0:63,6)-'0')-1;
    worst = inf;
    for vertex = 1:size(signs,1)
        nominal = center;
        state = center+signs(vertex,:).'.*radius;
        for step = 1:60
            input = terminal.input+terminal.feedback*nominal;
            for index = 1:numel(flows)
                value = flows{index}*[state;input;1];
                pose = terminal.poseBound-terminal.poseRows*value(1:3);
                velocity = terminal.velocityLimit-abs(value(4:6));
                worst = min([worst;pose;velocity]);
            end
            state = value(1:6);
            nominal = flows{end}(1:6,:)*[nominal;input;1];
        end
    end
end

function result = localPerformanceFailure(~,program)
    if nnz(program.P)==0
        result = program.defaultSolver();
    else
        result = encounterTestFixture.fail([],[]);
    end
end
