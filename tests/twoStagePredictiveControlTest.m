classdef twoStagePredictiveControlTest < matlab.unittest.TestCase
    % One affine RTI model, two objectives and unchanged optimized inputs.
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
            testCase.verifyEqual([search.stages.objective],["pcbfSlack","clfSlack"]);
            testCase.verifyLessThanOrEqual(search.solverCalls,2);
            testCase.verifyFalse(search.nominalRestarted);
            testCase.verifyTrue(search.convexSolveCompleted);
            testCase.verifyFalse(problem.metadata.optimizationConverged);
            testCase.verifyFalse(problem.metadata.nonlinearValidationPerformed);
            testCase.verifyEqual(problem.metadata.safetyScope,"affineSampledConstraints");
            testCase.verifyTrue(isnan(problem.metadata.nonlinearHardResidual));
            testCase.verifyLessThanOrEqual(problem.solution.safety, ...
                search.slackCap+cfg.solver.feasibilityTolerance);
            testCase.verifyEqual(inputs,problem.model.linearization.inputs+problem.solution.inputCorrection,AbsTol=0);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            testCase.verifyEqual(state.appliedInput,inputs(:,1),AbsTol=0);
            testCase.verifyFalse(problem.metadata.postSolveInputModification);
            testCase.verifyFalse(problem.metadata.postSolveFeasibilityCheck);
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
        function coldCrossingPlansOneReferenceBeforeTheRtiSolve(testCase)
            [ego,~,cfg]=localFixture();[~,q,road]=collisionThreatScenario("crossing",cfg);
            [command,inputs,problem]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
            testCase.verifyEqual(problem.metadata.search.initialization,"plannedAvoidanceRollout");
            testCase.verifyEqual(problem.metadata.rtiIterations,1);
            testCase.verifyFalse(problem.metadata.search.nominalRestarted);
            testCase.verifyLessThan(problem.model.linearization.inputs(2,1),0);
            testCase.verifyTrue(problem.metadata.search.planner.used);
            testCase.verifyEqual(problem.metadata.search.planner.candidateCount,1);
            testCase.verifyEqual(problem.metadata.search.planner.algorithm,"conflictIntervalAnalyticDeformation");
            testCase.verifyFalse(problem.metadata.search.planner.robustCertificate);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            localVerifyRollouts(testCase,problem,cfg);
        end
        function anUnsafeColdReferenceStillGetsOnlyOneRtiModel(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,nominal]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [~,q]=collisionThreatScenario("headOn",cfg);
            model=nominal.model;model.target=q;model.targetEpoch=q;model.targetOnset=true;
            [solution,search]=solvePredictiveControl(model,[],tic);
            testCase.verifyEqual(search.linearizationCount,1);
            testCase.verifyEqual(search.rtiIterations,1);
            testCase.verifyLessThanOrEqual(search.solverCalls,2);
            testCase.verifyFalse(search.nominalRestarted);
            testCase.verifyEqual(search.returned,~isempty(solution));
        end
        function plannedInputsRespectBrakingSlewBeforeTheSingleRti(testCase)
            [ego,road,cfg]=localFixture();cfg.model.brakingRatioRateMaximum=1;
            ego.heldActuatorInput=[0;0];
            [~,~,nominal]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [~,q]=collisionThreatScenario("crossing",cfg);
            model=nominal.model;model.target=q;model.targetEpoch=q;model.targetOnset=true;
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
                testCase.verifyGreaterThan(problem.model.linearization.guide.direction.'*lateral,0);
                testCase.verifyEqual(problem.model.collisionSide,1);
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
            [x,q,road]=collisionThreatScenario("curvedCrossing",cfg);
            ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
            [~,~,~,prior]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
            ego=localSuccessor(ego,prior);q=predictiveSafetyGeometry.predictTarget(q,cfg.controller.sampleTime);
            [~,~,problem]=collisionAvoidanceController(ego,localTarget(q),road,cfg,prior);
            testCase.verifyEqual(problem.metadata.search.initialization,"shiftedInputRollout");
            testCase.verifyEqual(problem.model.collisionSide,prior.collisionSide);
            testCase.verifyEqual(problem.model.linearization.terminalContext.collisionSide,prior.collisionSide);
        end
        function anInfeasibleWarmReferenceDoesNotRestartOrExecuteAnOldPlan(testCase)
            [ego,road,cfg]=localFixture();road.lateralClearance=[3;3];
            [~,~,~,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            ego=localSuccessor(ego,prior);
            prior.inputTrajectory=repmat([.25;0],1,size(prior.inputTrajectory,2));
            testCase.verifyError(@()collisionAvoidanceController(ego,localNearTarget(),road,cfg,prior), ...
                'collisionAvoidanceController:noOptimizationSolution');
        end
        function targetOnsetUsesTheSameSingleIteration(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,prior);
            [~,~,problem]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,prior);
            testCase.verifyTrue(problem.model.targetOnset);
            testCase.verifyEqual(problem.metadata.search.initialization,"shiftedInputRollout");
            testCase.verifyEqual(problem.metadata.rtiIterations,1);
            testCase.verifyFalse(problem.metadata.preplannedAvoidanceTrajectoryRequired);
        end
        function anUncertifiedStoppedTailDoesNotRestartTheSeed(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,nominal]=collisionAvoidanceController(ego,[],road,cfg,[]);
            model=nominal.model;model.initialState=zeros(6,1);model.previousInput=zeros(2,1);
            % An overlapping stationary target makes membership false. The
            % diagnostic solve may fail, but its single seed must not move off.
            q=[3;0;0;0;0;0;1.6;2.4;.95;0;0];
            model.target=q;model.targetEpoch=q;model.targetOnset=false;
            prior=struct('inputTrajectory',zeros(2,cfg.controller.horizonSteps));
            [~,search,model]=solvePredictiveControl(model,prior,tic);
            testCase.verifyEqual(search.rtiIterations,1);
            testCase.verifyLessThanOrEqual(model.linearization.inputs(2,end),0);
            testCase.verifyEqual(model.linearization.states(4:6,end),zeros(3,1),AbsTol=1e-12);
        end
        function aCertifiedPrefixExitAlsoEndsTheTerminalEncounter(testCase)
            [ego,road,cfg]=localFixture();cfg.controller.horizonSteps=40;
            beta=asin(1.6/20);
            q=[40;-20;-beta;60;0;beta;1.6;2.4;.95;0;0];
            [~,~,problem]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
            point=problem.model.encounterExitRecord;
            testCase.verifyTrue(isfinite(point));
            testCase.verifyTrue(problem.model.terminalContext.encounterEnded);
            forecast=predictiveSafetyGeometry.predictTarget(q,(0:80)*cfg.controller.sampleTime/2);
            distance=vecnorm(problem.model.sampledStates(1:2,:)-forecast(1:2,:),2,1);
            testCase.verifyTrue(any(distance(point+1:end)<cfg.collision.encounterRangeMeters));
            testCase.verifyEqual(problem.model.collisionEscapeNormals(:,point:end),zeros(2,81-point+1));
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
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:noOptimizationSolution');
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
        function initialOverlapIsExposedAsPositivePcbfSlack(testCase)
            [ego,road,cfg]=localFixture();
            target=struct('targetPositionInertial',[0;0],'targetVelocityInertial',[40;0],'targetYawInertial',0);
            [command,inputs,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyGreaterThan(problem.solution.safety,0);
            testCase.verifyFalse(problem.metadata.zeroSlack);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
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
