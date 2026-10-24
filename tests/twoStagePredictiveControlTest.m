classdef twoStagePredictiveControlTest < matlab.unittest.TestCase
    % Two objectives per reference, adaptive horizons and unchanged commands.
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for directory=["controller","config","scripts"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,directory)));
            end
        end
    end
    methods (Test)
        function oneRtiModelServesBothObjectivesAndTheUnmodifiedCommand(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            [command,inputs,problem,state]=collisionAvoidanceController(ego,[],road,cfg,[]);
            search=problem.metadata.search;
            testCase.verifyEqual(search.algorithm,"realTimeIteration");
            testCase.verifyEqual(search.rtiIterations,1);
            testCase.verifyEqual(search.linearizationCount,1);
            testCase.verifyEqual([search.stages.objective],["pcbfSlack","clfSlackAndReferenceDeviation"]);
            testCase.verifyLessThanOrEqual(search.solverCalls,2);
            testCase.verifyFalse(search.nominalRestarted);
            testCase.verifyTrue(search.convexSolveCompleted);
            testCase.verifyFalse(problem.metadata.optimizationConverged);
            testCase.verifyTrue(problem.metadata.nonlinearValidationPerformed);
            testCase.verifyEqual(problem.metadata.safetyScope,"nonlinearSampledConstraints");
            testCase.verifyLessThanOrEqual(problem.metadata.nonlinearHardResidual,cfg.solver.feasibilityTolerance);
            testCase.verifyLessThanOrEqual(problem.solution.safety, ...
                search.slackCap+cfg.solver.feasibilityTolerance);
            testCase.verifyEqual(inputs,problem.model.linearization.inputs+problem.solution.inputCorrection,AbsTol=0);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            testCase.verifyEqual(state.appliedInput,inputs(:,1),AbsTol=0);
            testCase.verifyFalse(problem.metadata.postSolveInputModification);
            testCase.verifyTrue(problem.metadata.postSolveFeasibilityCheck);
            localVerifyRollouts(testCase,problem,cfg);
        end
        function shiftedReferenceUsesTheNewMeasurementInOneIteration(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            [~,~,~,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            ego=localSuccessor(ego,prior);ego.position(2)=ego.position(2)+.01;
            [command,inputs,problem]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyEqual(problem.metadata.search.initialization,"shiftedInputRollout");
            testCase.verifyEqual(size(inputs,2),max(cfg.controller.horizonSteps,size(prior.inputTrajectory,2)-1));
            testCase.verifyEqual(problem.metadata.rtiIterations,1);
            testCase.verifyEqual(problem.model.linearization.states(:,1),problem.model.initialState,AbsTol=0);
            testCase.verifyNotEqual(problem.model.initialState,prior.stateTrajectory(:,2));
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            localVerifyRollouts(testCase,problem,cfg);
        end
        function aChangedTargetForecastCanLengthenTheShiftedSeed(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,problem,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [~,target]=collisionThreatScenario("headOn",cfg);
            model=problem.model;model.target=target;model.targetEpoch=target;model.targetOnset=false;
            % Stop at the first numerical attempt so the assertion concerns
            % forecast-driven horizon selection, not an infeasibility retry.
            model.cfg.solver.maxIterations=1;
            [~,search,model]=solvePredictiveControl(model,prior,tic);
            testCase.verifyEqual(search.linearizationCount,1);
            testCase.verifyEqual(search.initialization,"shiftedInputRollout");
            testCase.verifyGreaterThan(size(model.linearization.inputs,2),size(prior.inputTrajectory,2));
            testCase.verifyEqual(model.linearization.inputs(:,1:size(prior.inputTrajectory,2)-1), ...
                prior.inputTrajectory(:,2:end),AbsTol=0);
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
        function coldCrossingSelectsACirculationReferenceBeforeTheRtiSolve(testCase)
            [ego,~,cfg]=localFixture();cfg.referenceSpeed=15;ego.speed=15;
            [~,q,road]=collisionThreatScenario("crossing",cfg);
            [command,inputs,problem]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
            testCase.verifyEqual(problem.metadata.search.initialization,"plannedAvoidanceRollout");
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
            localVerifyRollouts(testCase,problem,cfg);
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
            testCase.verifyGreaterThan(max(abs(problem.model.linearization.inputs(1,:))),.01);
            anchor=problem.model.linearization;
            testCase.verifyGreaterThan(size(anchor.inputs,2)*cfg.controller.sampleTime,2);
            % Preview may incidentally produce a terminal endpoint; the
            % planner never appends a terminal extension to obtain it.
            testCase.verifyEqual(anchor.appendedTerminalSteps,0);
            testCase.verifyEqual(anchor.appendedBrakingSteps,0);
            testCase.verifyEqual(search.rtiIterations,1);
            ego=localSuccessor(ego,state);q=predictiveSafetyGeometry.predictTarget(q,cfg.controller.sampleTime);
            [~,~,next]=collisionAvoidanceController(ego,localTarget(q),road,cfg,state);
            testCase.verifyEqual(next.metadata.search.attempts(1).initialization,"shiftedInputRollout");
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
        function infeasibleReferencesShareOneHorizonSearchBudget(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,nominal]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [~,q]=collisionThreatScenario("headOn",cfg);
            model=nominal.model;model.target=q;model.targetEpoch=q;model.targetOnset=true;
            [solution,search]=solvePredictiveControl(model,[],tic);
            testCase.verifyEqual(search.rtiIterations,1+search.restartCount);
            testCase.verifyLessThanOrEqual(search.solverCalls,2*search.rtiIterations);
            if search.nominalRestarted
                testCase.verifyEqual(search.attempts(1).stages(end).exitFlag,-2);
            end
            testCase.verifyEqual(search.returned,~isempty(solution));
        end
        function plannedInputsRespectBrakingSlewIncludingAReplan(testCase)
            [ego,road,cfg]=localFixture();cfg.model.brakingRatioRateMaximum=1;
            % Isolate slew handling in the original unbounded encounter.
            ego.heldActuatorInput=[0;0];
            [~,~,nominal]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [~,q]=collisionThreatScenario("crossing",cfg);
            model=nominal.model;model.target=q;model.targetEpoch=q;model.targetOnset=true;
            [~,search,model]=solvePredictiveControl(model,[],tic);
            testCase.verifyEqual(search.rtiIterations,1+search.restartCount);
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
            ego=localSuccessor(ego,prior);q=predictiveSafetyGeometry.predictTarget(q,cfg.controller.sampleTime);
            [~,~,problem]=collisionAvoidanceController(ego,localTarget(q),road,cfg,prior);
            testCase.verifyEqual(problem.metadata.search.initialization,"shiftedInputRollout");
            testCase.verifyEqual(problem.model.collisionSide,prior.collisionSide);
            testCase.verifyEqual(problem.model.linearization.terminalContext.collisionSide,prior.collisionSide);
        end
        function anInfeasibleWarmReferenceReplansBeforeReturningAnOptimizedCommand(testCase)
            [ego,road,cfg]=localFixture();road.lateralClearance=[3;3];
            cfg.controller.enforceRoadBoundaries=true;cfg.controller.maximumHorizonSteps=32;
            [~,~,~,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            ego=localSuccessor(ego,prior);
            prior.inputTrajectory=repmat([.25;0],1,32);
            [command,inputs,problem]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,prior);
            search=problem.metadata.search;
            testCase.verifyTrue(search.nominalRestarted);
            testCase.verifyEqual(search.restartCount,1);
            testCase.verifyEqual(search.selectedAttempt,2);
            testCase.verifyEqual(search.linearizationCount,2);
            testCase.verifyEqual(search.attempts(1).stages(end).exitFlag,-2);
            testCase.verifyEqual(search.attempts(2).initialization,"plannedAvoidanceRollout");
            testCase.verifyTrue(search.planner.used);
            testCase.verifyTrue(isnan(search.planner.conflictStartSeconds));
            testCase.verifyEqual(search.solverCalls,sum([search.attempts.solverCalls])+search.restorationSolverCalls);
            testCase.verifyLessThanOrEqual(search.solverCalls,4);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            testCase.verifyEqual(problem.model.linearization.states(:,1),problem.model.initialState,AbsTol=0);
            localVerifyRollouts(testCase,problem,cfg);
        end
        function replanningRebuildsCollisionGeometryFromTheFreshReference(testCase)
            [ego,road,cfg]=localFixture();road.lateralClearance=[3;3];
            cfg.controller.enforceRoadBoundaries=true;
            cfg.controller.horizonSteps=32;cfg.controller.maximumHorizonSteps=32;
            [~,~,fresh,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            prior.inputTrajectory=repmat([.35;0],1,size(prior.inputTrajectory,2));
            prior.avoidanceInitialized=true;
            prior.collisionSide=-fresh.model.collisionSide;
            prior.collisionEscapeNormals=repmat([0;-1],1,size(prior.collisionEscapeNormals,2));
            [solution,search,model]=solvePredictiveControl(fresh.model,prior,tic);
            testCase.assertNotEmpty(solution);
            testCase.verifyEqual(search.selectedAttempt,2);
            testCase.verifyEqual(search.attempts(1).stages(end).exitFlag,-2);
            testCase.verifyEqual(search.initialization,"plannedAvoidanceRollout");
            testCase.verifyEqual(model.linearization.states,fresh.model.linearization.states,AbsTol=0);
            testCase.verifyEqual(model.collisionEscapeNormals,fresh.model.collisionEscapeNormals,AbsTol=0);
            testCase.verifyEqual(model.collisionSide,fresh.model.collisionSide);
            testCase.verifyEqual(model.terminalContext.collisionSide,fresh.model.collisionSide);
            testCase.verifyEqual(model.uncertainty,fresh.model.uncertainty);
            testCase.verifyEqual(solution.inputs,fresh.inputTrajectory,AbsTol=1e-7);
        end
        function targetOnsetUsesTheSameSingleIteration(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,prior);
            [~,~,problem]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,prior);
            testCase.verifyTrue(problem.model.targetOnset);
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
            % diagnostic solves may fail; replanning must not move off either.
            q=[3;0;0;0;0;0;1.6;2.4;.95;0;0];
            model.target=q;model.targetEpoch=q;model.targetOnset=false;
            prior=struct('inputTrajectory',zeros(2,cfg.controller.horizonSteps));
            [~,search,model]=solvePredictiveControl(model,prior,tic);
            testCase.verifyGreaterThanOrEqual(search.rtiIterations,2);
            testCase.verifyEqual(search.stopReason,"horizonLimit");
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
        function targetRangeCannotSelectADifferentClf(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            target=localNearTarget();far=target;far.targetPositionInertial(1)=-60;
            [~,~,absent]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [~,~,near]=collisionAvoidanceController(ego,target,road,cfg,[]);
            [~,~,distant]=collisionAvoidanceController(ego,far,road,cfg,[]);
            testCase.verifyEqual([absent.metadata.clfInitialValue,near.metadata.clfInitialValue,distant.metadata.clfInitialValue], ...
                repmat(absent.metadata.clfInitialValue,1,3),AbsTol=1e-12);
            testCase.verifyEqual([absent.metadata.clfFunction,near.metadata.clfFunction,distant.metadata.clfFunction], ...
                repmat("quadraticTransverseError",1,3));
        end
        function consecutiveMeasuredSuccessorsRemainSingleIterations(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            [~,~,~,state]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            for frame=1:3
                ego=localSuccessor(ego,state);
                [~,~,problem,state]=collisionAvoidanceController(ego,[],road,cfg,state);
                testCase.verifyEqual(problem.metadata.search.initialization,"shiftedInputRollout");
                testCase.verifyEqual(problem.metadata.search.linearizationCount,1);
                localVerifyRollouts(testCase,problem,cfg);
            end
        end
        function impossibleRoadHasNoRelaxedCommand(testCase)
            [ego,road,cfg]=localFixture();road.lateralClearance=[.5;.5];
            cfg.controller.enforceRoadBoundaries=true;cfg.controller.maximumHorizonSteps=16;
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:noOptimizationSolution');
        end
        function anExplicitHorizonLimitStopsAnImpossibleRoadSearch(testCase)
            [ego,road,cfg]=localFixture();cfg.controller.maximumHorizonSteps=16;
            [~,~,nominal]=collisionAvoidanceController(ego,[],road,cfg,[]);
            model=nominal.model;model.road.lateralClearance=[.5;.5];timer=tic;
            [solution,search,model]=solvePredictiveControl(model,[],timer);
            testCase.verifyEmpty(solution);
            testCase.verifyGreaterThanOrEqual(search.horizonExtensions,1);
            testCase.verifyEqual(search.selectedAttempt,0);
            testCase.verifyGreaterThanOrEqual(search.linearizationCount,2);
            testCase.verifyEqual(search.stopReason,"horizonLimit");
            testCase.verifyEqual([search.stages.exitFlag],-2);
            testCase.verifyEqual(size(model.linearization.inputs,2),2*cfg.controller.horizonSteps);
            testCase.verifyEqual(model.solveTimer,timer);
            testCase.verifyEqual(model.solveTimeLimitSeconds, ...
                cfg.solver.timeLimitSeconds-cfg.solver.completionReserveSeconds);
            testCase.verifyEqual(search.solverCalls,sum([search.attempts.solverCalls])+search.restorationSolverCalls);
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
            localVerifyRollouts(testCase,problem,cfg);
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
            ego=localSuccessor(ego,prior);cfg.solver.timeLimitSeconds=1e-12;
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
            localVerifyRollouts(testCase,problem,cfg);
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
function ego=localSuccessor(ego,prior)
    x=prior.stateTrajectory(:,2);ego.position=x(1:2);ego.yaw=x(3);ego.speed=x(4);
    ego.lateralVelocity=x(5);ego.yawRate=x(6);ego.heldActuatorInput=prior.appliedInput;
end
function target=localTarget(q)
    target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',q(4)*predictiveSafetyGeometry.direction(q(3)+q(6)), ...
        'targetYawInertial',q(3),'targetSideslip',q(6),'targetTangentialAcceleration',q(5));
end
function localVerifyRollouts(testCase,problem,cfg)
    for anchor={problem.model.linearization,struct('states',problem.predictedState,'inputs',problem.inputTrajectory)}
        path=anchor{1};residual=zeros(6,size(path.inputs,2));
        for index=1:size(path.inputs,2)
            next=nonlinearBicycleModel.sample(path.states(:,index),path.inputs(:,index),cfg);
            residual(:,index)=path.states(:,index+1)-next;
        end
        testCase.verifyLessThanOrEqual(max(abs(residual),[],'all'),1e-12);
    end
end
