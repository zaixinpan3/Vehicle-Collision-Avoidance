classdef twoStagePredictiveControlTest < matlab.unittest.TestCase
    % Two lexicographic objectives on one shared affine prediction model.
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function targetFreeInitializationRunsBothLexicographicStages(testCase)
            [ego,road,cfg]=localFixture();
            [command,inputs,problem,state]=collisionAvoidanceController(ego,[],road,cfg,[]);
            search=problem.metadata.search;
            testCase.verifyEqual(search.initialization,"laneFeedbackRollout");
            testCase.verifyFalse(search.stages(1).numericalSolve);
            testCase.verifyEqual(search.stages(1).value,0);
            testCase.verifyTrue(search.stages(2).numericalSolve);
            testCase.verifyEqual(search.solverCalls,sum([search.stages.numericalSolve]));
            testCase.verifyEqual([search.stages.objective],["pcbfSlack","clfSlack"]);
            testCase.verifyEqual(problem.metadata.clfFunction,"nominalCostToGo");
            testCase.verifyGreaterThan([search.stages.exitFlag],0);
            testCase.verifyTrue(search.clfStageCompleted);
            testCase.verifyLessThanOrEqual(problem.solution.safety,search.slackCap+cfg.solver.feasibilityTolerance);
            testCase.verifyLessThanOrEqual(search.primaryOptimum,cfg.solver.feasibilityTolerance);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            testCase.verifyEqual(state.appliedInput,inputs(:,1),AbsTol=0);
            testCase.verifyFalse(isfield(state,'witness'));
            testCase.verifyFalse(problem.metadata.nonlinearValidationPerformed);
            testCase.verifyEqual(problem.metadata.safetyScope,"affineSampledConstraints");
            testCase.verifyEqual(problem.metadata.recursiveFeasibilityScope,"notCertifiedForNonlinearPlant");
            localVerifyAffinePrediction(testCase,problem,cfg);
        end
        function encounterUsesTheSameNominalClf(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            [command,inputs,problem]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            search=problem.metadata.search;
            testCase.verifyGreaterThan(problem.metadata.encounterExitStep,0);
            testCase.verifyEqual(search.solverCalls,sum([search.stages.numericalSolve]));
            testCase.verifyEqual([search.stages.objective],["pcbfSlack","clfSlack"]);
            testCase.verifyEqual(problem.metadata.clfFunction,"nominalCostToGo");
            testCase.verifyEqual(problem.metadata.tertiaryObjective,"none");
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            localVerifyAffinePrediction(testCase,problem,cfg);
        end
        function targetRangeCannotSelectADifferentClf(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            target=localNearTarget();far=target;far.targetPositionInertial(1)=-60;
            [~,~,absent]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [~,~,near]=collisionAvoidanceController(ego,target,road,cfg,[]);
            [~,~,distant]=collisionAvoidanceController(ego,far,road,cfg,[]);
            testCase.verifyEqual([absent.metadata.clfInitialValue,near.metadata.clfInitialValue,distant.metadata.clfInitialValue], ...
                repmat(absent.metadata.clfInitialValue,1,3),AbsTol=1e-12);
            testCase.verifyEqual([absent.metadata.clfRequiredDecrease,near.metadata.clfRequiredDecrease,distant.metadata.clfRequiredDecrease], ...
                repmat(absent.metadata.clfRequiredDecrease,1,3),AbsTol=1e-12);
            testCase.verifyEqual([absent.metadata.clfFunction,near.metadata.clfFunction,distant.metadata.clfFunction], ...
                repmat("nominalCostToGo",1,3));
            testCase.verifyTrue(all([absent.metadata.search.clfStageCompleted,near.metadata.search.clfStageCompleted,distant.metadata.search.clfStageCompleted]));
        end
        function shiftedInputsAreRolledOutFromTheNewMeasurement(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            [~,~,~,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            ego=localSuccessor(ego,prior);ego.position(2)=ego.position(2)+.01;
            [command,inputs,problem]=collisionAvoidanceController(ego,[],road,cfg,prior);
            anchor=problem.model.linearization;
            testCase.verifyEqual(problem.metadata.search.initialization,"shiftedInputRollout");
            testCase.verifyEqual(anchor.states(:,1),problem.model.initialState,AbsTol=0);
            testCase.verifyNotEqual(anchor.states(:,1),prior.stateTrajectory(:,2));
            testCase.verifyEqual(anchor.inputs(:,1:end-1),prior.inputTrajectory(:,2:end),AbsTol=0);
            testCase.verifyEqual(problem.predictedState(:,1),problem.model.initialState,AbsTol=1e-7);
            testCase.verifyEqual(problem.metadata.solverCallCount,sum([problem.metadata.search.stages.numericalSolve]));
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            localVerifyAffinePrediction(testCase,problem,cfg);
            localVerifyAnchorRollout(testCase,anchor,cfg);
        end
        function failedShiftReinitializesFlowAndRebuildsBothStages(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            ego=localSuccessor(ego,prior);prior.inputTrajectory(1,end-9:end)=.15;
            [command,inputs,problem]=collisionAvoidanceController(ego,[],road,cfg,prior);
            search=problem.metadata.search;
            testCase.verifyTrue(search.flowRestarted);
            testCase.verifyEqual(search.solverCalls,sum([search.stages.numericalSolve]));
            testCase.verifyEqual(search.linearizationCount,2);
            testCase.verifyEqual([search.attempts.initialization],["shiftedInputRollout","movingTargetFlow"]);
            testCase.verifyEqual(search.initializationFailure,"pcbfNoNumericalResult");
            testCase.verifyEqual([search.stages.objective],["pcbfSlack","pcbfSlack","clfSlack"]);
            testCase.verifyGreaterThan([search.attempts(2).stages.exitFlag],0);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            localVerifyAnchorRollout(testCase,problem.model.linearization,cfg);
            localVerifyAffinePrediction(testCase,problem,cfg);
        end
        function failedFreshInitializationHasOnlyOneBoundedInputExpansion(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,problem,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            model=problem.model;model.initialState(4)=20;
            [solution,search]=solvePredictiveControl(model,prior);
            testCase.verifyEmpty(solution);
            testCase.verifyTrue(search.flowRestarted);
            testCase.verifyEqual(search.linearizationCount,3);
            testCase.verifyEqual(search.solverCalls,sum([search.stages.numericalSolve]));
            testCase.verifyEqual([search.attempts.inputTrustScale],[1,1,2]);
            testCase.verifyFalse(search.clfStageAttempted);
            testCase.verifyEqual(search.terminationReason,"pcbfNoNumericalResult");
        end
        function overlappingHardRowsHaveNoInventedRestorationDirection(testCase)
            [ego,road,cfg]=localFixture();
            cfg.controller.horizonSteps=1;
            cfg.nonlinear.recoveryHorizonSeconds=cfg.controller.sampleTime;
            target=struct('targetPositionInertial',[0;0], ...
                'targetVelocityInertial',[0;0],'targetYawInertial',0);
            % The start and hard completion anchor both overlap. Ordinary
            % distance multipliers cannot provide a positive-buffer hard row.
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:noOptimizationSolution');
        end
        function unevaluableShiftUsesFreshInitializationBeforeOptimization(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            ego=localSuccessor(ego,prior);prior.inputTrajectory(1,end)=pi;
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyEqual(problem.metadata.search.initialization,"movingTargetFlow");
            testCase.verifyEqual(problem.metadata.search.initializationFailure, ...
                "collisionAvoidanceController:invalidTireOperatingPoint");
            testCase.verifyFalse(problem.metadata.search.flowRestarted);
            testCase.verifyEqual(problem.metadata.search.solverCalls,sum([problem.metadata.search.stages.numericalSolve]));
        end
        function exhaustedBudgetDoesNotExecuteThePreviousTrajectory(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,prior);cfg.solver.timeLimitSeconds=1e-12;
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,prior), ...
                'collisionAvoidanceController:noOptimizationSolution');
        end
        function inaccurateIterationLimitResultDoesNotSupplyACommand(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            cfg.solver.maxIterations=1;
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:noOptimizationSolution');
        end
        function initialOverlapIsReportedAsOptimizedPositiveSlack(testCase)
            [ego,road,cfg]=localFixture();
            target=struct('targetPositionInertial',[0;0], ...
                'targetVelocityInertial',[40;0],'targetYawInertial',0);
            [command,inputs,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyEqual(problem.metadata.search.primaryLowerBound, ...
                cfg.collision.safetyMarginMeters,AbsTol=1e-6);
            testCase.verifyGreaterThan(problem.metadata.search.primaryOptimum,0);
            testCase.verifyTrue(problem.metadata.search.stages(1).numericalSolve);
            testCase.verifyGreaterThan(problem.solution.safety,0);
            testCase.verifyFalse(problem.metadata.zeroSlack);
            testCase.verifyTrue(problem.metadata.search.clfStageCompleted);
            testCase.verifyLessThanOrEqual(problem.solution.safety, ...
                problem.metadata.search.slackCap+cfg.solver.feasibilityTolerance);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            testCase.verifyEqual(problem.metadata.search.attempts(1).initialization,"movingTargetFlow");
            testCase.verifyTrue(problem.metadata.search.modelAgreementSatisfied);
            localVerifyAffinePrediction(testCase,problem,cfg);
        end
        function brakingSlewAndMagnitudeBoundsConstrainTheReturnedPlan(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.05;
            cfg.model.brakingRatioRateMaximum=2;
            ego.heldActuatorInput=[0;0];
            [~,inputs,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            limit=cfg.controller.sampleTime*2;tol=cfg.solver.feasibilityTolerance;
            testCase.verifyLessThanOrEqual(max(abs(diff([ego.heldActuatorInput(2),inputs(2,:)]))),limit+tol);
            testCase.verifyLessThanOrEqual(inputs(2,:),cfg.actuation.brakingRatioMaximum+tol);
            testCase.verifyGreaterThanOrEqual(inputs(2,:),cfg.actuation.brakingRatioMinimum-tol);
            localVerifyAffinePrediction(testCase,problem,cfg);
        end
        function numericalSteeringCorrectionDoesNotImposeActuatorSlew(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;ego.yaw=.5;
            ego.heldActuatorInput=[.5;0];
            [command,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            steering=command.frontWheelSteeringAngle;
            testCase.verifyLessThan(steering,-.1);
            testCase.verifyLessThanOrEqual(abs(steering-problem.model.linearization.inputs(1,1)), ...
                cfg.nonlinear.trustRadius*.15+cfg.solver.feasibilityTolerance);
            testCase.verifyGreaterThan(abs(steering-ego.heldActuatorInput(1)),.5);
            testCase.verifyEqual(problem.metadata.solverCallCount,sum([problem.metadata.search.stages.numericalSolve]));
            localVerifyAffinePrediction(testCase,problem,cfg);
        end
        function nonlinearDisagreementRebuildsBothStagesInTheSameFrame(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=1;ego.yaw=.5;
            cfg.nonlinear.trustRadius=5;
            target=struct('targetPositionInertial',[-25;-10],'targetVelocityInertial',[8;0],'targetYawInertial',0);
            [command,~,problem,state]=collisionAvoidanceController(ego,target,road,cfg,[]);
            search=problem.metadata.search;history=search.modelAgreementHistory;
            testCase.verifyGreaterThan(search.refinementCount,1);
            testCase.verifyGreaterThan(history(1).ratio,1);
            testCase.verifyLessThanOrEqual(problem.metadata.predictionAgreement.ratio,1);
            testCase.verifyEqual([search.stages.objective],repmat(["pcbfSlack","clfSlack"],1,search.refinementCount));
            testCase.verifyEqual(problem.metadata.clfFunction,"nominalCostToGo");
            testCase.verifyTrue(search.clfStageCompleted);
            testCase.verifyEqual(command.actuatorInput,state.appliedInput,AbsTol=0);
            testCase.verifyGreaterThan(state.linearizationTrustScale,0);
            testCase.verifyLessThan(state.linearizationTrustScale,1);
            localVerifyAnchorRollout(testCase,problem.model.linearization,cfg);
            localVerifyAffinePrediction(testCase,problem,cfg);
            localVerifyPredictionAgreement(testCase,problem,cfg);
        end
        function brakingEncounterKeepsRefinementProgressAcrossPrimaryFailures(testCase)
            [ego,~,cfg]=localFixture();[~,q,road]=collisionThreatScenario("brakingLead",cfg);
            target=struct('targetPositionInertial',q(1:2), ...
                'targetVelocityInertial',q(4)*[cos(q(3)+q(6));sin(q(3)+q(6))], ...
                'targetYawInertial',q(3),'targetSideslip',q(6), ...
                'targetTangentialAcceleration',q(5),'targetRearAxleDistance',q(7));
            [command,inputs,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            search=problem.metadata.search;
            testCase.verifyGreaterThan(search.refinementCount,1);
            testCase.verifyFalse(search.flowRestarted);
            testCase.verifyTrue(search.modelAgreementSatisfied);
            testCase.verifyTrue(search.clfStageCompleted);
            testCase.verifyGreaterThan(search.attempts(search.selectedAttempt).stages(1).exitFlag,0);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            localVerifyPredictionAgreement(testCase,problem,cfg);
        end
        function exhaustedRelinearizationBudgetDoesNotIssueAnInaccuratePlan(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=1;ego.yaw=.5;
            cfg.nonlinear.trustRadius=5;cfg.nonlinear.maximumLinearizations=1;
            target=struct('targetPositionInertial',[-25;-10],'targetVelocityInertial',[8;0],'targetYawInertial',0);
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:noOptimizationSolution');
        end
        function accuracyDoesNotEndClfDescentAtAnArtificialInputBoundary(testCase)
            [x,q,road,cfg]=collisionThreatScenario("brakingLead",struct('referenceSpeed',15, ...
                'controller',struct('horizonSteps',16)));
            ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
            target=struct('targetPositionInertial',q(1:2), ...
                'targetVelocityInertial',q(4)*[cos(q(3)+q(6));sin(q(3)+q(6))], ...
                'targetYawInertial',q(3),'targetSideslip',q(6), ...
                'targetTangentialAcceleration',q(5),'targetRearAxleDistance',q(7));
            limited=cfg;limited.nonlinear.maximumLinearizations=2;
            [~,~,early]=collisionAvoidanceController(ego,target,road,limited,[]);
            [~,~,refined]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(early.metadata.search.modelAgreementSatisfied);
            testCase.verifyGreaterThan(early.metadata.clfSlack,.1);
            testCase.verifyEqual(refined.metadata.clfFunction,early.metadata.clfFunction);
            tail=nonlinearBicycleModel.nominalTail(cfg,refined.model.nominalReference.curvature);
            phi=@(u)nonlinearBicycleModel.nominalValue(nonlinearBicycleModel.sample(x,u,cfg), ...
                u,refined.model.lane,refined.model.nominalReference,tail,cfg);
            testCase.verifyLessThan(phi(refined.inputTrajectory(:,1)),.01*phi(early.inputTrajectory(:,1)));
            testCase.verifyLessThan(refined.metadata.clfSlack,1e-5);
            localVerifyPredictionAgreement(testCase,refined,cfg);
        end
    end
end

function localVerifyPredictionAgreement(testCase,problem,cfg)
    x=problem.model.initialState;states=zeros(size(problem.predictedState));states(:,1)=x;
    for index=1:size(problem.inputTrajectory,2)
        x=nonlinearBicycleModel.sample(x,problem.inputTrajectory(:,index),cfg);states(:,index+1)=x;
    end
    reach=norm([cfg.vehicle.length;cfg.vehicle.width]/2)+norm(cfg.vehicle.rectangleOffset);
    count=problem.metadata.predictionAgreement.poseConstraintNodeCount;
    error=max(vecnorm(states(1:2,1:count)-problem.predictedState(1:2,1:count)) ...
        +reach*abs(states(3,1:count)-problem.predictedState(3,1:count)));
    testCase.verifyEqual(error,problem.metadata.predictionAgreement.poseErrorMeters,AbsTol=1e-12);
    testCase.verifyLessThanOrEqual(error,cfg.nonlinear.predictionToleranceMeters);
end

function localVerifyAnchorRollout(testCase,anchor,cfg)
    residual=zeros(6,size(anchor.inputs,2));
    for index=1:size(anchor.inputs,2)
        next=nonlinearBicycleModel.sample(anchor.states(:,index),anchor.inputs(:,index),cfg);
        residual(:,index)=next-anchor.states(:,index+1);
    end
    testCase.verifyLessThanOrEqual(max(abs(residual),[],'all'),1e-12);
end

function localVerifyAffinePrediction(testCase,problem,cfg)
    anchor=problem.model.linearization;states=problem.predictedState;inputs=problem.inputTrajectory;
    residual=zeros(6,size(inputs,2));
    for index=1:size(inputs,2)
        [next,a,b]=nonlinearBicycleModel.sample(anchor.states(:,index),anchor.inputs(:,index),cfg);
        affine=next+a*(states(:,index)-anchor.states(:,index))+b*(inputs(:,index)-anchor.inputs(:,index));
        residual(:,index)=states(:,index+1)-affine;
    end
    testCase.verifyLessThanOrEqual(max(abs(residual),[],'all'),cfg.solver.feasibilityTolerance);
    testCase.verifyFalse(problem.metadata.affineValidationPerformed);
    testCase.verifyEqual(size(inputs,2),min(cfg.controller.maximumHorizonSteps, ...
        cfg.controller.horizonSteps+ceil(cfg.nonlinear.recoveryHorizonSeconds/cfg.controller.sampleTime)));
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
