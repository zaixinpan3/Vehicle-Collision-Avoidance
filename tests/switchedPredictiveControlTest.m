classdef switchedPredictiveControlTest < matlab.unittest.TestCase
    % Target-presence objective switching and consistent issued commands.
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for directory=["controller","config","scripts"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,directory)));
            end
        end
    end
    methods (Test)
        function cruiseUsesOneClfSolveAndTheUnmodifiedCommand(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            [command,inputs,problem,state]=collisionAvoidanceController(ego,[],road,cfg,[]);
            search=problem.metadata.search;
            testCase.verifyEqual(search.algorithm,"realTimeIteration");
            testCase.verifyEqual(search.rtiIterations,1);
            testCase.verifyEqual(search.linearizationCount,1);
            testCase.verifyEqual([search.stages.objective],"integratedClfSlackAndTracking");
            testCase.verifyLessThanOrEqual(search.solverCalls,1);
            testCase.verifyFalse(search.nominalRestarted);
            testCase.verifyEqual(search.initializationFailure,"");
            testCase.verifyTrue(search.convexSolveCompleted);
            testCase.verifyFalse(problem.metadata.optimizationConverged);
            testCase.verifyFalse(problem.metadata.nonlinearValidationPerformed);
            testCase.verifyFalse(problem.metadata.nonlinearPredictionEvaluated);
            testCase.verifyEqual(problem.metadata.predictionModel,"affineSqpIterate");
            testCase.verifyEqual(problem.metadata.safetyScope,"affineSubproblem");
            testCase.verifyTrue(isnan(problem.metadata.nonlinearHardResidual));
            testCase.verifyLessThanOrEqual(problem.metadata.affineHardResidual,cfg.solver.feasibilityTolerance);
            testCase.verifyEqual(problem.solution.safety,0,AbsTol=cfg.solver.feasibilityTolerance);
            testCase.verifyTrue(isnan(search.primaryOptimum));
            testCase.verifyTrue(isnan(search.slackCap));
            testCase.verifyFalse(search.pcbfStageAttempted);
            testCase.verifyFalse(search.secondaryStageAttempted);
            testCase.verifyEqual(problem.metadata.optimizationMode,"clf");
            testCase.verifyEqual(inputs,problem.model.linearization.inputs+problem.solution.inputCorrection,AbsTol=0);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            testCase.verifyEqual(state.appliedInput,inputs(:,1),AbsTol=0);
            testCase.verifyFalse(problem.metadata.postSolveInputModification);
            testCase.verifyFalse(problem.metadata.postSolveFeasibilityCheck);
            localVerifyRtiPrediction(testCase,problem,cfg);
        end
        function shiftedReferenceUsesTheNewMeasurementInOneIteration(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            [~,~,~,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            ego=localSuccessor(ego,prior,cfg);ego.position(2)=ego.position(2)+.01;
            [command,inputs,problem]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyEqual(problem.metadata.search.initialization,"shiftedPrimalIterate");
            testCase.verifyEqual(problem.metadata.search.plannerCalls,0);
            testCase.verifyEqual(size(inputs,2),size(prior.inputTrajectory,2));
            testCase.verifyEqual(problem.metadata.rtiIterations,1);
            testCase.verifyEqual(problem.model.linearization.states(:,1),problem.model.initialState,AbsTol=0);
            testCase.verifyNotEqual(problem.model.initialState,prior.stateTrajectory(:,2));
            testCase.verifyEqual(problem.model.linearization.states(:,2:end-1),prior.stateTrajectory(:,3:end),AbsTol=0);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            localVerifyRtiPrediction(testCase,problem,cfg);
        end
        function aChangedTargetForecastCannotLengthenTheEncounter(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,problem,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            [~,target]=collisionThreatScenario("headOn",cfg);
            model=problem.model;model.target=target;model.targetEpoch=target;model.targetOnset=false;
            [anchor,search]=solvePredictiveControl("probe",struct('operation',"reference", ...
                'model',model,'previousState',prior));
            testCase.verifyEqual(search.initialization,"shiftedPrimalIterate");
            testCase.verifyEqual(search.plannerCalls,0);
            testCase.verifyEqual(size(anchor.inputs,2),size(prior.inputTrajectory,2));
            testCase.verifyEqual(anchor.inputs(:,1:size(prior.inputTrajectory,2)-1), ...
                prior.inputTrajectory(:,2:end),AbsTol=0);
        end
        function aLongEncounterHorizonDoesNotShrinkAfterExecutingAHold(testCase)
            [ego,road,cfg]=localFixture();
            [~,q]=collisionThreatScenario("brakingLead",cfg);
            [~,~,first,prior]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
            testCase.assertGreaterThan(first.metadata.horizonSteps,cfg.controller.horizonSteps);
            ego=localSuccessor(ego,prior,cfg);
            [~,inputs,second,next]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyEqual(size(inputs,2),first.metadata.horizonSteps);
            testCase.verifyEqual(second.metadata.horizonPolicy,"fixedWithinEncounter");
            testCase.verifyEqual(next.encounterHorizonSteps,prior.encounterHorizonSteps);
            testCase.verifyEqual(second.metadata.endpointIndex,first.metadata.endpointIndex+1);
            testCase.verifyEqual(second.metadata.search.plannerCalls,0);
            testCase.verifyEqual(second.model.linearization.inputs(:,1:end-1),prior.inputTrajectory(:,2:end),AbsTol=0);
            localVerifyRtiPrediction(testCase,second,cfg);
        end
        function aNewEncounterCanChooseADifferentHorizon(testCase)
            [ego,road,cfg]=localFixture();ego.stateTime=0;
            [~,q]=collisionThreatScenario("brakingLead",cfg);
            [~,~,first,prior]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
            ego=localSuccessor(ego,prior,cfg);ego.stateTime=cfg.controller.sampleTime;
            ego.perception=struct('time',ego.stateTime,'range',50,'completeWithinRange',true);
            [~,~,cleared,idle]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyEqual(cleared.metadata.encounterHorizonSteps,0);
            testCase.verifyEqual(idle.encounterHorizonSteps,0);
            ego=localSuccessor(ego,idle,cfg);ego.stateTime=2*cfg.controller.sampleTime;
            ego.perception.time=ego.stateTime;
            [~,~,new,next]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,idle);
            testCase.verifyTrue(new.model.targetOnset);
            testCase.verifyEqual(new.metadata.search.plannerCalls,1);
            testCase.verifyEqual(next.encounterId,prior.encounterId+1);
            testCase.verifyEqual(next.encounterHorizonSteps,new.metadata.horizonSteps);
            testCase.verifyLessThan(new.metadata.horizonSteps,first.metadata.horizonSteps);
        end
        function aNewEncounterReplacesThePreviouslyUnobstructedCruiseSeed(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,problem,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [~,target]=collisionThreatScenario("headOn",cfg);
            model=problem.model;model.target=target;model.targetEpoch=target;model.targetOnset=true;
            request=struct('operation',"reference",'model',model,'previousState',prior);
            [anchor,search]=solvePredictiveControl("probe",request);
            testCase.verifyEqual(search.initialization,"plannedAvoidanceRollout");
            testCase.verifyTrue(anchor.planner.used);
            testCase.verifyEqual(anchor.states(:,1),model.initialState,AbsTol=0);
            testCase.verifyGreaterThan(max(abs(anchor.inputs(1,:))),.01);
        end
        function clearingAnEncounterRebuildsCruiseBeforeOneLinearization(testCase)
            [ego,road,cfg]=localFixture();ego.stateTime=0;
            [~,~,~,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            ego=localSuccessor(ego,prior,cfg);ego.stateTime=cfg.controller.sampleTime;
            ego.perception=struct('time',ego.stateTime,'range',50,'completeWithinRange',true);
            % A stale avoidance input sequence must not be the linearization
            % point once the complete scan releases its encounter.
            prior.inputTrajectory(:,:)=repmat([.25;-.7],1,size(prior.inputTrajectory,2));
            [command,~,problem,state]=collisionAvoidanceController(ego,[],road,cfg,prior);
            reference=problem.model.linearization;
            testCase.verifyEqual(problem.metadata.search.initialization,"cruiseFeedbackRollout");
            testCase.verifyEqual(problem.metadata.optimizationMode,"clf");
            testCase.verifyEqual(problem.metadata.search.plannerCalls,0);
            testCase.verifyEqual(problem.metadata.search.linearizationCount,1);
            testCase.verifyEqual(problem.metadata.search.solverCalls,1);
            testCase.verifyFalse(problem.metadata.postSolveFeasibilityCheck);
            testCase.verifyGreaterThan(reference.inputs(2,1),-.7);
            for k=1:size(reference.inputs,2)
                expected=nonlinearBicycleModel.sample(reference.states(:,k),reference.inputs(:,k),cfg);
                testCase.verifyEqual(reference.states(:,k+1),expected,AbsTol=1e-12);
            end
            testCase.verifyEqual(command.actuatorInput,problem.solution.inputs(:,1),AbsTol=0);
            ego=localSuccessor(ego,state,cfg);ego.stateTime=2*cfg.controller.sampleTime;
            ego.perception.time=ego.stateTime;
            [~,~,next]=collisionAvoidanceController(ego,[],road,cfg,state);
            testCase.verifyEqual(next.metadata.search.initialization,"cruiseFeedbackRollout");
            testCase.verifyEqual(next.metadata.search.linearizationCount,1);
            testCase.verifyEqual(next.metadata.search.plannerCalls,0);
        end
        function coldCrossingSelectsACirculationReferenceBeforeTheRtiSolve(testCase)
            [ego,~,cfg]=localFixture();cfg.referenceSpeed=15;ego.speed=15;
            [~,q,road]=collisionThreatScenario("crossing",cfg);
            [command,inputs,problem]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
            testCase.verifyEqual(problem.metadata.search.initialization,"plannedAvoidanceRollout");
            testCase.verifyTrue(problem.model.targetOnset);
            testCase.verifyEqual(problem.metadata.encounterId,1);
            testCase.verifyEqual(problem.metadata.search.plannerCalls,1);
            % The revised in-road crossing admits the first local solve.
            % Initialization still precedes RTI, without a terminal search.
            testCase.verifyEqual(problem.metadata.rtiIterations,1);
            testCase.verifyFalse(problem.metadata.search.nominalRestarted);
            testCase.verifyGreaterThan(problem.metadata.search.attempts(1).stages(end).exitFlag,0);
            testCase.verifyGreaterThan(max(abs(problem.model.linearization.inputs(1,:))),.01);
            testCase.verifyTrue(problem.metadata.search.planner.used);
            testCase.verifyEqual(problem.metadata.search.planner.candidateCount,1);
            testCase.verifyEqual(problem.metadata.search.planner.algorithm,"safetyCirculation");
            testCase.verifyFalse(problem.metadata.search.planner.robustCertificate);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            localVerifyRtiPrediction(testCase,problem,cfg);
        end
        function aNewLeadThreatBeyondTwoSecondsPlansBeforeItBecomesUrgent(testCase)
            [ego,road,cfg]=localFixture();
            [~,q]=collisionThreatScenario("brakingLead",cfg);
            [~,~,problem,state]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
            search=problem.metadata.search;
            testCase.verifyTrue(search.planner.used);
            testCase.verifyTrue(state.avoidanceInitialized);
            % A successful single field need not contain colliding nodes or
            % produce a prescribed lateral offset to count as planning.
            testCase.verifyEqual(search.planner.candidateCount,1);
            anchor=problem.model.linearization;
            testCase.verifyTrue(anchor.guide.fieldActive);
            % A mild early correction may be mostly longitudinal; do not
            % require an unnecessary minimum steering amplitude.
            testCase.verifyLessThan(min(anchor.inputs(2,:)),0);
            testCase.verifyGreaterThan(size(anchor.inputs,2)*cfg.controller.sampleTime,2);
            % Preview may incidentally produce a terminal endpoint; the
            % planner never appends a terminal extension to obtain it.
            testCase.verifyEqual(anchor.appendedTerminalSteps,0);
            testCase.verifyEqual(anchor.appendedBrakingSteps,0);
            testCase.verifyEqual(search.rtiIterations,1);
            ego=localSuccessor(ego,state,cfg);q=predictiveSafetyGeometry.predictTarget(q,cfg.controller.sampleTime);
            [~,~,next]=collisionAvoidanceController(ego,localTarget(q),road,cfg,state);
            testCase.verifyEqual(next.metadata.search.attempts(1).initialization,"shiftedPrimalIterate");
        end
        function seedsStayDifferentiableWhilePhysicalInputsIncludeSaturation(testCase)
            [ego,road,cfg]=localFixture();cfg.referenceSpeed=15;ego.speed=1.8;
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [physicalLower,physicalUpper]=nonlinearBicycleModel.inputBounds(cfg);
            testCase.verifyEqual([physicalLower(2);physicalUpper(2)],[-1;1]);
            [lower,upper]=nonlinearBicycleModel.inputBounds(cfg,true);
            [input,~,~]=terminalSafeSet.cruiseInput([0;0;0;1.8;0;0],problem.model);
            testCase.verifyLessThanOrEqual(input(2),upper(2));
            testCase.verifyGreaterThanOrEqual(input(2),lower(2));
            testCase.verifyLessThanOrEqual(max(problem.model.linearization.inputs(2,:)),upper(2));
            testCase.verifyGreaterThanOrEqual(min(problem.model.linearization.inputs(2,:)),lower(2));
        end
        function shiftingASaturatedInputRebuildsAnInteriorTangentSeed(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,problem,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            prior.inputTrajectory(2,2:end)=1;
            [~,search,model]=solvePredictiveControl(problem.model,prior,tic);
            testCase.verifyEqual(search.linearizationCount,1);
            testCase.verifyLessThan(max(model.linearization.inputs(2,:)),1);
            testCase.verifyTrue(all(isfinite(model.linearization.states),'all'));
        end
        function eachReferenceUsesAtMostOneRtiModel(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,nominal]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [~,q]=collisionThreatScenario("headOn",cfg);
            model=nominal.model;model.target=q;model.targetEpoch=q;model.targetOnset=true;
            [solution,search]=solvePredictiveControl(model,[],tic);
            testCase.verifyEqual(search.rtiIterations,1);
            testCase.verifyEqual(search.restartCount,0);
            testCase.verifyEqual(search.horizonExtensions,0);
            testCase.verifyNumElements(search.attempts,1);
            testCase.verifyLessThanOrEqual(search.solverCalls,2);
            testCase.verifyFalse(search.nominalRestarted);
            testCase.verifyEqual(search.returned,~isempty(solution));
        end
        function plannedInputsRespectBrakingSlewOnTheSingleAttempt(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,nominal]=collisionAvoidanceController(ego,[],road,cfg,[]);
            % Test the planner's slew handling without requiring an unrelated
            % short-horizon cruise solve to admit a slow actuator from rest.
            cfg.model.brakingRatioRateMaximum=1;
            [~,q]=collisionThreatScenario("crossing",cfg);
            model=nominal.model;model.cfg=cfg;model.previousInput=[0;0];
            model.target=q;model.targetEpoch=q;model.targetOnset=true;
            [~,search,model]=solvePredictiveControl(model,[],tic);
            testCase.verifyEqual(search.rtiIterations,1);
            testCase.verifyEqual(search.planner.candidateCount,1);
            inputs=model.linearization.inputs;
            testCase.verifyLessThanOrEqual(max(abs(diff([0,inputs(2,:)]))), ...
                cfg.controller.sampleTime+1e-12);
            states=model.linearization.states;
            for index=1:size(inputs,2)
                expected=nonlinearBicycleModel.sample(states(:,index),inputs(:,index),cfg);
                testCase.verifyEqual(states(:,index+1),expected,AbsTol=1e-12);
            end
        end
        function curvedHeadOnPlanningKeepsTheRtiPassingSide(testCase)
            for speed=[8,15]
                [~,~,cfg]=localFixture();cfg.referenceSpeed=speed;
                [x,q,road]=collisionThreatScenario("curvedHeadOn",cfg);
                ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
                [~,~,problem]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
                projection=laneGeometry.project(x(1:2),problem.model.lane);
                lateral=[-sin(projection.heading);cos(projection.heading)];
                chosen=sign(problem.model.linearization.guide.direction.'*lateral);
                testCase.verifyNotEqual(chosen,0);
                testCase.verifyEqual(problem.model.collisionSide,chosen);
            end
        end
        function preparingTheReferenceRoadDoesNotIssueACommand(testCase)
            [~,~,cfg]=localFixture();[~,~,road]=collisionThreatScenario("curvedHeadOn",cfg);
            [command,inputs,prediction,state]=collisionAvoidanceController("warmUp",cfg,road);
            testCase.verifyEmpty(command);testCase.verifyEmpty(inputs);
            testCase.verifyEmpty(prediction);testCase.verifyEmpty(state);
        end
        function shiftingThePlanRetainsItsTerminalPassingSide(testCase)
            [~,~,cfg]=localFixture();cfg.referenceSpeed=15;
            % This fixture tests side continuity, independently of corridor admission.
            [x,q,road]=collisionThreatScenario("curvedCrossing",cfg);
            ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
            [~,~,~,prior]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
            ego=localSuccessor(ego,prior,cfg);q=predictiveSafetyGeometry.predictTarget(q,cfg.controller.sampleTime);
            [~,~,problem]=collisionAvoidanceController(ego,localTarget(q),road,cfg,prior);
            testCase.verifyEqual(problem.metadata.search.initialization,"shiftedPrimalIterate");
            testCase.verifyEqual(problem.model.collisionSide,prior.collisionSide);
            testCase.verifyEqual(problem.model.linearization.terminalContext.collisionSide,prior.collisionSide);
        end
        function anInfeasibleWarmReferenceIsNotReplacedByAPlanner(testCase)
            [ego,road,cfg]=localFixture();road.lateralClearance=[3;3];
            cfg.controller.enforceRoadBoundaries=true;
            cfg.controller.horizonSteps=32;cfg.controller.maximumHorizonSteps=32;
            [~,~,first,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            prior.inputTrajectory=repmat([.35;0],1,33);
            model=first.model;model.targetOnset=false;
            prior.stateTrajectory=localTrajectory(model.initialState,prior.inputTrajectory,cfg);
            [solution,search,model]=solvePredictiveControl(model,prior,tic);
            testCase.verifyEmpty(solution);
            testCase.verifyEqual(search.plannerCalls,0);
            testCase.verifyFalse(search.nominalRestarted);
            testCase.verifyNumElements(search.attempts,1);
            testCase.verifyEqual(search.stages(1).exitFlag,-2);
            testCase.verifyEqual(search.stopReason,"pcbfNoNumericalResult");
            testCase.verifyFalse(search.planner.used);
            testCase.verifyEqual(model.linearization.inputs,prior.inputTrajectory(:,2:end),AbsTol=0);
        end
        function anInvalidContinuationCannotSilentlyRequestPlanning(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,first,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            model=first.model;model.targetOnset=false;
            prior.inputTrajectory(:)=NaN;
            [solution,search]=solvePredictiveControl(model,prior,tic);
            testCase.verifyEmpty(solution);
            testCase.verifyEqual(search.plannerCalls,0);
            testCase.verifyEqual(search.initializationFailure,"collisionAvoidanceController:missingWarmStart");
            testCase.verifyEqual(search.linearizationCount,0);
        end
        function extendingTheInitialReferenceDoesNotInvokeThePlannerAgain(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,first]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            original=first.model.linearization;
            request=struct('operation',"reference",'model',first.model,'previousState',[], ...
                'reference',original,'minimumHorizonSteps',size(original.inputs,2)+4);
            [extended,search]=solvePredictiveControl("probe",request);
            testCase.verifyEqual(search.plannerCalls,0);
            testCase.verifyEqual(search.initialization,"extendedInputRollout");
            testCase.verifyEqual(extended.inputs(:,1:size(original.inputs,2)),original.inputs,AbsTol=0);
            testCase.verifyEqual(extended.states(:,1:size(original.states,2)),original.states,AbsTol=1e-12);
            testCase.verifyEqual(size(extended.inputs,2),size(original.inputs,2)+4);
        end
        function anInfeasibleOnsetDoesNotExtendItsHorizon(testCase)
            [ego,road,cfg]=localFixture();cfg.controller.maximumHorizonSteps=32;
            [~,~,first]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            model=first.model;model.road.lateralClearance=[.5;.5];
            [solution,search]=solvePredictiveControl(model,[],tic);
            testCase.verifyEmpty(solution);
            testCase.verifyEqual(search.horizonExtensions,0);
            testCase.verifyEqual(search.plannerCalls,1);
            testCase.verifyEqual(search.linearizationCount,1);
            testCase.verifyNumElements(search.attempts,1);
            testCase.verifyEqual(search.attempts.initialization,"plannedAvoidanceRollout");
            testCase.verifyLessThan(search.horizonSteps,cfg.controller.maximumHorizonSteps);
            testCase.verifyEqual(search.stopReason,"pcbfNoNumericalResult");
        end
        function newMeasurementsWithinAnEncounterDoNotRestartPlanning(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,first,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            ego=localSuccessor(ego,prior,cfg);
            target=localNearTarget();target.targetPositionInertial(2)=6.1;
            [~,~,second,next]=collisionAvoidanceController(ego,target,road,cfg,prior);
            testCase.verifyEqual(first.metadata.search.plannerCalls,1);
            testCase.verifyEqual(second.metadata.search.plannerCalls,0);
            testCase.verifyFalse(second.model.targetOnset);
            testCase.verifyEqual(next.encounterId,prior.encounterId);
            testCase.verifyEqual(second.model.target(2),6.1,AbsTol=1e-12);
            testCase.verifyEqual(second.model.linearization.inputs(:,1:size(prior.inputTrajectory,2)-1), ...
                prior.inputTrajectory(:,2:end),AbsTol=1e-12);
        end
        function targetOnsetUsesTheSameSingleIteration(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,prior,cfg);
            [~,~,problem]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,prior);
            testCase.verifyTrue(problem.model.targetOnset);
            testCase.verifyEqual(problem.metadata.search.plannerCalls,1);
            testCase.verifyEqual(problem.metadata.search.initialization,"plannedAvoidanceRollout");
            testCase.verifyEqual(problem.metadata.rtiIterations,1);
            testCase.verifyFalse(problem.metadata.preplannedAvoidanceTrajectoryRequired);
        end
        function anUncertifiedStoppedTailDoesNotRestartTheSeed(testCase)
            [ego,road,cfg]=localFixture();
            cfg.controller.maximumHorizonSteps=16;
            [~,~,nominal]=collisionAvoidanceController(ego,[],road,cfg,[]);
            model=nominal.model;model.initialState=zeros(6,1);model.previousInput=zeros(2,1);
            % An overlapping stationary target makes membership false. The
            % diagnostic solves may fail; extending the tail must not restart motion.
            q=[3;0;0;0;0;0;1.6;2.4;.95;0;0];
            model.target=q;model.targetEpoch=q;model.targetOnset=false;
            prior=struct('inputTrajectory',zeros(2,cfg.controller.horizonSteps), ...
                'stateTrajectory',zeros(6,cfg.controller.horizonSteps+1));
            [~,search,model]=solvePredictiveControl(model,prior,tic);
            testCase.verifyEqual(search.rtiIterations,1);
            testCase.verifyEqual(search.plannerCalls,0);
            testCase.verifyEqual(search.stopReason,"pcbfNoNumericalResult");
            testCase.verifyLessThanOrEqual(model.linearization.inputs(2,end),0);
            testCase.verifyEqual(model.linearization.states(4:6,end),zeros(3,1),AbsTol=1e-12);
        end
        function aReferenceExitCannotEndAnUnrestrictedPredictedEncounter(testCase)
            [ego,road,cfg]=localFixture();cfg.controller.horizonSteps=40;
            beta=asin(1.6/20);
            q=[40;-20;-beta;60;0;beta;1.6;2.4;.95;0;0];
            [~,~,problem]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
            testCase.verifyTrue(isnan(problem.model.encounterExitRecord));
            testCase.verifyFalse(problem.model.terminalContext.encounterEnded);
            forecast=predictiveSafetyGeometry.predictTarget(q,(0:80)*cfg.controller.sampleTime/2);
            distance=vecnorm(problem.model.sampledStates(1:2,:)-forecast(1:2,:),2,1);
            testCase.verifyTrue(any(distance>cfg.collision.encounterRangeMeters));
            testCase.verifyEqual(problem.solution.encounterExit,41);
            testCase.verifyEqual(problem.metadata.rtiIterations,1);
        end
        function onlyAnActiveTargetDisablesTheCruiseClf(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            target=localNearTarget();far=target;far.targetPositionInertial(1)=-60;
            [~,~,absent]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [~,~,near]=collisionAvoidanceController(ego,target,road,cfg,[]);
            [~,~,distant]=collisionAvoidanceController(ego,far,road,cfg,[]);
            testCase.verifyEqual([absent.metadata.clfInitialValue,near.metadata.clfInitialValue,distant.metadata.clfInitialValue], ...
                repmat(absent.metadata.clfInitialValue,1,3),AbsTol=1e-12);
            testCase.verifyEqual([absent.metadata.clfFunction,near.metadata.clfFunction,distant.metadata.clfFunction], ...
                ["quadraticTransverseError","none","quadraticTransverseError"]);
            testCase.verifyEqual([absent.metadata.clfEnabled,near.metadata.clfEnabled,distant.metadata.clfEnabled], ...
                [true,false,true]);
            testCase.verifyEqual(near.metadata.optimizationObjective,"pcbfSlack");
            testCase.verifyFalse(near.metadata.search.clfStageAttempted);
            testCase.verifyTrue(near.metadata.search.secondaryStageAttempted);
            testCase.verifyTrue(near.metadata.search.pcbfStageCompleted);
            testCase.verifyEqual(near.metadata.optimizationMode,"pcbf");
            testCase.verifyEqual(near.metadata.secondaryObjective,"nominalTrackingAndRegularization");
            testCase.verifyEqual(near.metadata.referenceDeviationWeight,10);
            testCase.verifyNumElements(near.metadata.search.stages,2);
            testCase.verifyLessThanOrEqual(near.metadata.solverCallCount,2);
        end
        function cruiseClfResumesAfterTheEncounterClears(testCase)
            [ego,road,cfg]=localFixture();ego.stateTime=0;
            [~,~,first,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            ego=localSuccessor(ego,prior,cfg);ego.stateTime=cfg.controller.sampleTime;
            [~,~,coasting,prior]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyTrue(coasting.metadata.hasTarget);
            testCase.verifyFalse(coasting.metadata.clfEnabled);
            testCase.verifyFalse(first.metadata.clfEnabled);
            ego=localSuccessor(ego,prior,cfg);ego.stateTime=2*cfg.controller.sampleTime;
            ego.perception=struct('time',ego.stateTime,'range',50,'completeWithinRange',true);
            [~,~,cleared]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyFalse(cleared.metadata.hasTarget);
            testCase.verifyTrue(cleared.metadata.clfEnabled);
            testCase.verifyTrue(cleared.metadata.search.clfStageCompleted);
            testCase.verifyFalse(cleared.metadata.search.pcbfStageAttempted);
            testCase.verifyEqual(cleared.metadata.optimizationMode,"clf");
            testCase.verifyNumElements(cleared.metadata.search.stages,1);
            testCase.verifyEqual(cleared.metadata.solverCallCount,1);
            testCase.verifyEqual(cleared.metadata.optimizationObjective,"integratedClfSlackAndTracking");
        end
        function consecutiveMeasuredSuccessorsRemainSingleIterations(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            [~,~,~,state]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            for frame=1:3
                ego=localSuccessor(ego,state,cfg);
                [~,~,problem,state]=collisionAvoidanceController(ego,[],road,cfg,state);
                testCase.verifyEqual(problem.metadata.search.initialization,"shiftedPrimalIterate");
                testCase.verifyEqual(problem.metadata.search.linearizationCount,1);
                localVerifyRtiPrediction(testCase,problem,cfg);
            end
        end
        function impossibleRoadHasNoRelaxedCommand(testCase)
            [ego,road,cfg]=localFixture();road.lateralClearance=[.5;.5];
            cfg.controller.enforceRoadBoundaries=true;cfg.controller.maximumHorizonSteps=16;
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:noOptimizationSolution');
        end
        function anImpossibleProblemStopsEvenWithoutAHorizonLimit(testCase)
            [ego,road,cfg]=localFixture();cfg.controller.maximumHorizonSteps=Inf;
            [~,~,nominal]=collisionAvoidanceController(ego,[],road,cfg,[]);
            model=nominal.model;model.road.lateralClearance=[.5;.5];timer=tic;
            [solution,search,model]=solvePredictiveControl(model,[],timer);
            testCase.verifyEmpty(solution);
            testCase.verifyEqual(search.horizonExtensions,0);
            testCase.verifyEqual(search.selectedAttempt,0);
            testCase.verifyEqual(search.linearizationCount,1);
            testCase.verifyNumElements(search.attempts,1);
            testCase.verifyEqual(search.stopReason,"clfNoNumericalResult");
            testCase.verifyEqual([search.stages.exitFlag],-2);
            testCase.verifyEqual(size(model.linearization.inputs,2),cfg.controller.horizonSteps);
            testCase.verifyEqual(model.solveTimer,timer);
            testCase.verifyEqual(model.solveTimeLimitSeconds, ...
                cfg.solver.timeLimitSeconds-cfg.solver.completionReserveSeconds);
            testCase.verifyEqual(search.solverCalls,sum([search.attempts.solverCalls]));
        end
        function roadEdgesAreDeferredWithoutChangingTheCruisePath(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            [~,inputs,withoutEdges]=collisionAvoidanceController(ego,[],road,cfg,[]);
            road.lateralClearance=[.5;.5];
            [~,withEdges,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyEmpty(problem.model.road.lateralClearance);
            testCase.verifyEqual(problem.model.terminalContext.clearance,[Inf;Inf]);
            testCase.verifyEqual(problem.model.lane,withoutEdges.model.lane);
            testCase.verifyEqual(problem.metadata.clfInitialValue,withoutEdges.metadata.clfInitialValue);
            testCase.verifyEqual(withEdges,inputs,AbsTol=1e-10);
        end
        function aDisplacedStateNoLongerNeedsAnExtensionToEscapeASearchBox(testCase)
            [ego,road,cfg]=localFixture();
            ego.position=[21.52;7.33];ego.yaw=.017;ego.speed=6.21;
            ego.lateralVelocity=-.153;ego.yawRate=-.14;
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            anchor=problem.model.linearization;count=size(anchor.inputs,2);
            testCase.verifyEqual(count,cfg.controller.horizonSteps);
            testCase.verifyEqual(problem.metadata.search.horizonExtensions,0);
            testCase.verifyEqual(problem.metadata.search.linearizationCount,1);
            testCase.verifyGreaterThan(anchor.states(4,end),5);
            localVerifyRtiPrediction(testCase,problem,cfg);
        end
        function planningPreservesMeasuredVelocityAndFinishesANearStop(testCase)
            [ego,road,cfg]=localFixture();cfg.referenceSpeed=15;cfg.controller.horizonSteps=16;
            ego.position=[84.567;-.623471];ego.yaw=-.0654507;ego.speed=.0799722;
            ego.lateralVelocity=.00427434;ego.yawRate=.00259123;
            target=[84.4527;-14.699;1.04376;16.45;1;.05;1.6;2.4;.95;0;0];
            [~,~,problem]=collisionAvoidanceController(ego,localTarget(target),road,cfg,[]);
            testCase.verifyTrue(problem.metadata.search.planner.used);
            guide=problem.model.linearization.guide;
            testCase.verifyEqual(norm(guide.velocity(:,1)),hypot(ego.speed,ego.lateralVelocity),AbsTol=1e-12);
            testCase.verifyEqual(guide.states(4:6,end),zeros(3,1),AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(max(guide.inputs(2,:)),0);
            testCase.verifyEqual(problem.metadata.search.initialization,"plannedAvoidanceRollout");
        end
        function exhaustedBudgetDoesNotExecuteThePreviousTrajectory(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,prior,cfg);cfg.solver.timeLimitSeconds=1e-12;
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,prior), ...
                'collisionAvoidanceController:noOptimizationSolution');
        end
        function expiredBudgetStartsNoLinearizationOrNumericalSolve(testCase)
            cfg=collisionAvoidanceControllerConfig(struct('solver',struct('timeLimitSeconds',1e-12)));
            [solution,search]=solvePredictiveControl(struct('cfg',cfg),[],tic);
            testCase.verifyEmpty(solution);
            testCase.verifyEqual(search.solverCalls,0);
            testCase.verifyEqual(search.linearizationCount,0);
            testCase.verifyEqual(search.terminationReason,"timeLimit");
            testCase.verifyFalse(search.nominalRestarted);
        end
        function anUnfinishedConicIterationDoesNotSupplyACommand(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;cfg.solver.maxIterations=1;
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:noOptimizationSolution');
        end
        function numericalIterationExhaustionDoesNotTriggerReplanning(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            [~,~,nominal]=collisionAvoidanceController(ego,[],road,cfg,[]);
            model=nominal.model;model.cfg.solver.maxIterations=1;
            [solution,search]=solvePredictiveControl(model,[],tic);
            testCase.verifyEmpty(solution);
            testCase.verifyFalse(search.nominalRestarted);
            testCase.verifyEqual(numel(search.attempts),1);
            testCase.verifyEqual(search.stages(end).exitFlag,0);
        end
        function initialOverlapIsExposedAsPositivePcbfSlack(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,nominal]=collisionAvoidanceController(ego,[],road,cfg,[]);
            model=nominal.model;
            model.target=[0;0;0;40;0;0;1.6;2.4;.95;0;0];
            model.targetEpoch=model.target;model.targetOnset=true;
            [anchor,~,model]=solvePredictiveControl("probe",struct('operation',"reference", ...
                'model',model,'previousState',[]));
            [problem,~,model]=solvePredictiveControl("probe",struct('operation',"formulate", ...
                'model',model,'anchor',anchor,'inherited',false));
            [solution,search]=solvePredictiveControl("probe",struct('operation',"solve", ...
                'model',model,'anchor',anchor,'problem',problem));
            % This is already a colliding initial state. Check the PCBF
            % formulation, without requiring a valid nonlinear command.
            testCase.assertNotEmpty(solution);
            testCase.verifyGreaterThan(solution.safety,0);
            testCase.verifyGreaterThan(search.primaryOptimum,0);
        end
        function returnedInputsRespectBrakingMagnitudeAndSlew(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.05;
            cfg.model.brakingRatioRateMaximum=2;ego.heldActuatorInput=[0;0];
            [~,inputs,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            tolerance=cfg.solver.reducedTolerance;
            testCase.verifyLessThanOrEqual(max(abs(diff([ego.heldActuatorInput(2),inputs(2,:)]))), ...
                cfg.controller.sampleTime*2+tolerance);
            testCase.verifyLessThanOrEqual(inputs(2,:),cfg.actuation.brakingRatioMaximum+tolerance);
            testCase.verifyGreaterThanOrEqual(inputs(2,:),cfg.actuation.brakingRatioMinimum-tolerance);
            localVerifyRtiPrediction(testCase,problem,cfg);
        end
    end
end

function [ego,road,cfg]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'controller',struct('horizonSteps',8), ...
        'solver',struct('timeLimitSeconds',5)));
    ego=struct('position',[0;0],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
    road=struct('centerline',[-100,0;1000,0]);
end
function target=localNearTarget()
    target=struct('targetPositionInertial',[-15;6],'targetVelocityInertial',[-8;0],'targetYawInertial',pi);
end
function ego=localSuccessor(ego,prior,cfg)
    x=[ego.position;ego.yaw;ego.speed;ego.lateralVelocity;ego.yawRate];
    x=nonlinearBicycleModel.sample(x,prior.appliedInput,cfg);
    ego.position=x(1:2);ego.yaw=x(3);ego.speed=x(4);
    ego.lateralVelocity=x(5);ego.yawRate=x(6);ego.heldActuatorInput=prior.appliedInput;
end
function target=localTarget(q)
    target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',q(4)*predictiveSafetyGeometry.direction(q(3)+q(6)), ...
        'targetYawInertial',q(3),'targetSideslip',q(6),'targetTangentialAcceleration',q(5));
end
function localVerifyRtiPrediction(testCase,problem,cfg)
    anchor=problem.model.linearization;
    states=problem.predictedState;inputs=problem.inputTrajectory;
    residual=zeros(6,size(inputs,2));
    for index=1:size(inputs,2)
        [next,a,b]=nonlinearBicycleModel.sample(anchor.states(:,index),anchor.inputs(:,index),cfg);
        expected=next+a*(states(:,index)-anchor.states(:,index)) ...
            +b*(inputs(:,index)-anchor.inputs(:,index));
        residual(:,index)=states(:,index+1)-expected;
    end
    testCase.verifyLessThanOrEqual(max(abs(residual),[],'all'),cfg.solver.feasibilityTolerance);
    testCase.verifyEqual(states,problem.solution.affineStates,AbsTol=0);
end

function states=localTrajectory(x,inputs,cfg)
    states=zeros(6,size(inputs,2)+1);states(:,1)=x;
    for index=1:size(inputs,2)
        states(:,index+1)=nonlinearBicycleModel.sample(states(:,index),inputs(:,index),cfg);
    end
end
