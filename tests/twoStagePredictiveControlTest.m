classdef twoStagePredictiveControlTest < matlab.unittest.TestCase
    % Successive convexification returns an unchanged optimized input sequence.
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for directory=["controller","config","scripts"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,directory)));
            end
        end
    end
    methods (Test)
        function intersectingNominalReferenceIsRestoredByRepeatedConvexification(testCase)
            [ego,~,cfg]=localFixture();[~,q,road]=collisionThreatScenario("headOn",cfg);
            [command,inputs,problem]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
            search=problem.metadata.search;
            testCase.verifyEqual(search.initialization,"nominalFeedbackRollout");
            testCase.verifyGreaterThan(search.convexIterations,1);
            testCase.verifyGreaterThan(search.iterations(1).restorationSlack,.1);
            testCase.verifyGreaterThan(search.linearizationCount,1);
            testCase.verifyLessThanOrEqual(problem.solution.restorationSlack,cfg.scp.constraintTolerance);
            testCase.verifyLessThanOrEqual(problem.solution.hard,cfg.scp.constraintTolerance);
            testCase.verifyTrue(problem.metadata.nonlinearValidationPerformed);
            testCase.verifyEqual(inputs,problem.model.linearization.inputs+problem.solution.inputCorrection,AbsTol=0);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            testCase.verifyFalse(problem.metadata.postSolveInputModification);
            testCase.verifyFalse(problem.metadata.postSolveFeasibilityCheck);
            context=terminalSafeSet.context(problem.model);
            testCase.verifyTrue(terminalSafeSet.member(context,problem.predictedState(:,end),size(inputs,2)));
            localVerifyNonlinearRollout(testCase,problem,cfg);
            localVerifyAnchorRollout(testCase,problem.model.linearization,cfg);
        end
        function laterLeadConflictReceivesALongerPredictionThanAnImmediateHeadOn(testCase)
            [ego,~,cfg]=localFixture();[~,head,road]=collisionThreatScenario("headOn",cfg);
            [~,short]=collisionAvoidanceController(ego,localTarget(head),road,cfg,[]);
            [~,lead,road]=collisionThreatScenario("brakingLead",cfg);
            [~,long,problem]=collisionAvoidanceController(ego,localTarget(lead),road,cfg,[]);
            testCase.verifyGreaterThan(size(long,2),size(short,2));
            testCase.verifyGreaterThan(size(long,2)*cfg.controller.sampleTime,5);
            testCase.verifyLessThanOrEqual(problem.solution.hard,cfg.scp.constraintTolerance);
        end
        function curvedAndTurningReferencesCanRestoreWithoutOpposingEscapeBranches(testCase)
            [~,~,cfg]=localFixture();
            for name=["curvedHeadOn","turningCrossing"]
                [x,q,road]=collisionThreatScenario(name,cfg);
                ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4), ...
                    'lateralVelocity',x(5),'yawRate',x(6));
                [~,~,problem]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
                testCase.verifyLessThanOrEqual(problem.solution.hard,cfg.scp.constraintTolerance);
                testCase.verifyGreaterThan(problem.metadata.search.acceptedIterations,0);
                localVerifyNonlinearRollout(testCase,problem,cfg);
            end
        end
        function insufficientRestorationIterationsDoNotIssueElasticControls(testCase)
            [ego,~,cfg]=localFixture();[~,q,road]=collisionThreatScenario("headOn",cfg);
            cfg.scp.maximumIterations=1;
            testCase.verifyError(@()collisionAvoidanceController(ego,localTarget(q),road,cfg,[]), ...
                'collisionAvoidanceController:noOptimizationSolution');
        end
        function targetFreeControlRetainsLexicographicPriorityAndTheSameClf(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            [command,inputs,problem,state]=collisionAvoidanceController(ego,[],road,cfg,[]);
            search=problem.metadata.search;
            testCase.verifyTrue(search.clfStageCompleted);
            testCase.verifyEqual(search.solverCalls,sum([search.stages.numericalSolve]));
            testCase.verifyEqual([search.stages(1:2:end).objective], ...
                repmat("pcbfSlack",1,search.convexIterations));
            testCase.verifyEqual([search.stages(2:2:end).objective], ...
                repmat("clfSlack",1,search.convexIterations));
            testCase.verifyLessThanOrEqual(problem.solution.safety ...
                +cfg.scp.restorationWeight*problem.solution.restorationSlack, ...
                search.slackCap+cfg.solver.feasibilityTolerance);
            testCase.verifyEqual(problem.metadata.clfFunction,"quadraticTransverseError");
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            testCase.verifyEqual(state.appliedInput,inputs(:,1),AbsTol=0);
            testCase.verifyEqual(problem.metadata.recursiveFeasibilityScope,"idealMathematicalProblemOnly");
            localVerifyNonlinearRollout(testCase,problem,cfg);
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
        function shiftedReferenceUsesTheCurrentMeasurementAndReoptimizes(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            [~,~,~,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            ego=localSuccessor(ego,prior);ego.position(2)=ego.position(2)+.01;
            [command,inputs,problem]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyEqual(problem.metadata.search.initialization,"shiftedInputRollout");
            testCase.verifyEqual(problem.model.linearization.states(:,1),problem.model.initialState,AbsTol=0);
            testCase.verifyNotEqual(problem.model.initialState,prior.stateTrajectory(:,2));
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            localVerifyAnchorRollout(testCase,problem.model.linearization,cfg);
            localVerifyNonlinearRollout(testCase,problem,cfg);
        end
        function infeasibleWarmReferenceCanRestartTheSameOptimizer(testCase)
            [ego,road,cfg]=localFixture();road.lateralClearance=[3;3];
            target=localNearTarget();
            [~,~,~,prior]=collisionAvoidanceController(ego,target,road,cfg,[]);
            ego=localSuccessor(ego,prior);
            % A corrupted local reference must not restrict the next problem.
            prior.inputTrajectory=repmat([.25;0],1,size(prior.inputTrajectory,2));
            [command,inputs,problem]=collisionAvoidanceController(ego,target,road,cfg,prior);
            search=problem.metadata.search;
            testCase.verifyTrue(search.nominalRestarted);
            testCase.verifyEqual(search.restartCount,1);
            testCase.verifyEqual(search.initialization,"nominalFeedbackRollout");
            testCase.verifyGreaterThan(search.selectedAttempt,1);
            testCase.verifyEqual(search.solverCalls,sum([search.stages.numericalSolve]));
            testCase.verifyLessThanOrEqual(problem.solution.hard,cfg.scp.constraintTolerance);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            testCase.verifyFalse(problem.metadata.postSolveInputModification);
            localVerifyNonlinearRollout(testCase,problem,cfg);
        end
        function targetOnsetUsesTheSameOptimizerWithoutAnAvoidancePlanner(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,prior);
            target=struct('targetPositionInertial',[30;0],'targetVelocityInertial',[-8;0],'targetYawInertial',pi);
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,prior);
            testCase.verifyTrue(problem.model.targetOnset);
            testCase.verifyEqual(problem.metadata.search.initialization,"shiftedInputRollout");
            testCase.verifyFalse(problem.metadata.preplannedAvoidanceTrajectoryRequired);
            testCase.verifyGreaterThan(problem.metadata.search.acceptedIterations,0);
        end
        function everyPredictedRectangleKeepsTheRoadAndTheRearHandlingEnvelope(testCase)
            [ego,~,cfg]=localFixture();[~,q,road]=collisionThreatScenario("headOn",cfg);
            [~,inputs,problem]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
            x=problem.predictedState;
            shape=cfg.vehicle.rectangleOffset+[cfg.vehicle.length;cfg.vehicle.width]/2.*[1,1,-1,-1;1,-1,1,-1];
            tire=modifiedFialaTire.parameters(cfg);k=3*tire.longitudinalForceScale(2)/tire.corneringStiffness(2);
            worstRoad=Inf;worstHandling=0;worstFrontAngle=0;worstSideslip=-Inf;
            for index=1:size(inputs,2)
                rotation=[cos(x(3,index+1)),-sin(x(3,index+1));sin(x(3,index+1)),cos(x(3,index+1))];
                projection=laneGeometry.project(x(1:2,index+1)+rotation*shape,problem.model.lane);
                lateral=projection.lateralPosition;
                worstRoad=min([worstRoad;road.lateralClearance(2)-lateral(:);road.lateralClearance(1)+lateral(:)]);
                ends=index+1;if index>1,ends=[index,index+1];end
                for node=ends
                    y=x(:,node);
                    worstHandling=max(worstHandling,norm([(y(5)-cfg.vehicle.lr*y(6))/(k*y(4));inputs(2,index)]));
                    frontAngle=atan2(y(5)+cfg.vehicle.lf*y(6),y(4))-inputs(1,index);
                    worstFrontAngle=max(worstFrontAngle,abs(frontAngle));
                end
                worstSideslip=max(worstSideslip,abs(x(5,index+1))-tan(cfg.model.sideslipMaximum)*x(4,index+1));
            end
            testCase.verifyGreaterThanOrEqual(worstRoad,-cfg.scp.constraintTolerance);
            testCase.verifyLessThanOrEqual(worstHandling,1+cfg.scp.constraintTolerance);
            testCase.verifyLessThan(worstFrontAngle,pi/2);
            testCase.verifyLessThanOrEqual(worstSideslip,cfg.scp.constraintTolerance);
        end
        function consecutiveModelSuccessorsRemainSolvable(testCase)
            [ego,~,cfg]=localFixture();[~,q,road]=collisionThreatScenario("headOn",cfg);
            [~,~,~,state]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
            for frame=1:3
                ego=localSuccessor(ego,state);
                future=predictiveSafetyGeometry.predictTarget(q,frame*cfg.controller.sampleTime);
                [~,~,problem,state]=collisionAvoidanceController(ego,localTarget(future),road,cfg,state);
                testCase.verifyEqual(problem.metadata.search.initialization,"shiftedInputRollout");
                testCase.verifyLessThanOrEqual(problem.solution.hard,cfg.scp.constraintTolerance);
                localVerifyNonlinearRollout(testCase,problem,cfg);
            end
        end
        function impossibleRoadHasNoElasticFallbackCommand(testCase)
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
        function inaccurateConicSolveDoesNotSupplyACommand(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;cfg.solver.maxIterations=1;
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:noOptimizationSolution');
        end
        function initialOverlapRemainsExplicitPositivePcbfSlack(testCase)
            [ego,road,cfg]=localFixture();
            target=struct('targetPositionInertial',[0;0],'targetVelocityInertial',[40;0],'targetYawInertial',0);
            [command,inputs,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyGreaterThan(problem.solution.safety,0);
            testCase.verifyFalse(problem.metadata.zeroSlack);
            testCase.verifyLessThanOrEqual(problem.solution.restorationSlack,cfg.scp.constraintTolerance);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
        end
        function brakingSlewAndMagnitudeBoundsConstrainTheReturnedPlan(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.05;
            cfg.model.brakingRatioRateMaximum=2;ego.heldActuatorInput=[0;0];
            [~,inputs,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            tolerance=cfg.scp.constraintTolerance;
            testCase.verifyLessThanOrEqual(max(abs(diff([ego.heldActuatorInput(2),inputs(2,:)]))), ...
                cfg.controller.sampleTime*2+tolerance);
            testCase.verifyLessThanOrEqual(inputs(2,:),cfg.actuation.brakingRatioMaximum+tolerance);
            testCase.verifyGreaterThanOrEqual(inputs(2,:),cfg.actuation.brakingRatioMinimum-tolerance);
            localVerifyNonlinearRollout(testCase,problem,cfg);
        end
    end
end

function target=localTarget(q)
    target=struct('targetPositionInertial',q(1:2), ...
        'targetVelocityInertial',q(4)*predictiveSafetyGeometry.direction(q(3)+q(6)), ...
        'targetYawInertial',q(3),'targetSideslip',q(6),'targetTangentialAcceleration',q(5));
end

function localVerifyAnchorRollout(testCase,anchor,cfg)
    residual=zeros(6,size(anchor.inputs,2));
    for index=1:size(anchor.inputs,2)
        [~,next]=terminalSafeSet.holdStates(anchor.states(:,index),anchor.inputs(:,index),cfg);
        residual(:,index)=next-anchor.states(:,index+1);
    end
    testCase.verifyLessThanOrEqual(max(abs(residual),[],'all'),1e-12);
end

function localVerifyNonlinearRollout(testCase,problem,cfg)
    % The issued plan is the nonlinear rollout of its inputs from the estimate.
    states=problem.predictedState;inputs=problem.inputTrajectory;
    residual=zeros(6,size(inputs,2));
    testCase.verifyEqual(states(:,1),problem.model.initialState,AbsTol=0);
    for index=1:size(inputs,2)
        [~,next]=terminalSafeSet.holdStates(states(:,index),inputs(:,index),cfg);
        residual(:,index)=states(:,index+1)-next;
    end
    testCase.verifyLessThanOrEqual(max(abs(residual),[],'all'),1e-12);
    testCase.verifyGreaterThanOrEqual(size(inputs,2),cfg.controller.horizonSteps);
    testCase.verifyLessThanOrEqual(size(inputs,2),cfg.controller.maximumHorizonSteps);
end
function [ego,road,cfg]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'controller',struct('horizonSteps',8)));
    ego=struct('position',[0;0],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
    road=struct('centerline',[-100,0;1000,0]);
end
function target=localNearTarget()
    % A receding target within the encounter range that does not obstruct the ego.
    target=struct('targetPositionInertial',[-15;6],'targetVelocityInertial',[-8;0],'targetYawInertial',pi);
end
function ego=localSuccessor(ego,prior)
    x=prior.stateTrajectory(:,2);ego.position=x(1:2);ego.yaw=x(3);ego.speed=x(4);
    ego.lateralVelocity=x(5);ego.yawRate=x(6);ego.heldActuatorInput=prior.appliedInput;
end
