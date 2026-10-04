classdef twoStagePredictiveControlTest < matlab.unittest.TestCase
    % Initialization/restoration and inherited-budget CLF on one affine model.
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function solvedPlanIsIssuedWithoutNonlinearChecking(testCase)
            [ego,~,cfg]=localFixture();
            [~,q,road]=collisionThreatScenario("headOn",cfg);
            target=struct('targetPositionInertial',q(1:2), ...
                'targetVelocityInertial',q(4)*[cos(q(3));sin(q(3))],'targetYawInertial',q(3));
            [command,inputs,problem,state]=collisionAvoidanceController(ego,target,road,cfg,[]);
            search=problem.metadata.search;
            testCase.verifyFalse(problem.metadata.nonlinearPredictionEvaluated);
            testCase.verifyFalse(isfield(problem.metadata,'predictionAgreement'));
            testCase.verifyFalse(isfield(state,'linearizationTrustScale'));
            testCase.verifyEqual(search.terminationReason,"twoStagesReturned");
            testCase.verifyTrue(problem.metadata.secondaryOptimumApplied);
            testCase.verifyLessThanOrEqual(problem.solution.safety,search.slackCap+cfg.solver.constraintTolerance);
            testCase.verifyEqual(problem.solution.clfSlack,max(0,problem.solution.clfNextValue ...
                -problem.solution.clfInitialValue+problem.solution.clfRequiredDecrease),AbsTol=cfg.solver.feasibilityTolerance);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            testCase.verifyEqual(state.stageSlacks,problem.solution.stageSlacks,AbsTol=0);
            endpoint=[problem.predictedState(:,end);inputs(:,end)];
            membership=terminalContinuation.membership(endpoint,problem.metadata.endpointIndex,problem.model.terminal);
            testCase.verifyLessThanOrEqual(membership,cfg.solver.constraintTolerance);
            localVerifyAffinePrediction(testCase,problem,cfg);
        end
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
            testCase.verifyEqual(problem.metadata.clfFunction,"quadraticTransverseError");
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
            testCase.verifyEqual(problem.metadata.clfFunction,"quadraticTransverseError");
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
                repmat("quadraticTransverseError",1,3));
            testCase.verifyTrue(all([absent.metadata.search.clfStageCompleted,near.metadata.search.clfStageCompleted,distant.metadata.search.clfStageCompleted]));
        end
        function shiftedInputsAreRolledOutFromTheNewMeasurement(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            [~,~,~,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            ego=localSuccessor(ego,prior);ego.position(2)=ego.position(2)+.01;
            [command,inputs,problem]=collisionAvoidanceController(ego,[],road,cfg,prior);
            anchor=problem.model.linearization;
            testCase.verifyEqual(problem.metadata.search.initialization,"shiftedInputRollout");
            testCase.verifyFalse(problem.metadata.search.potentialFieldRestarted);
            testCase.verifyEqual(problem.metadata.search.linearizationCount,1);
            testCase.verifyEqual(anchor.states(:,1),problem.model.initialState,AbsTol=0);
            testCase.verifyNotEqual(anchor.states(:,1),prior.stateTrajectory(:,2));
            testCase.verifyEqual(anchor.inputs(:,1:end-1),prior.inputTrajectory(:,2:end),AbsTol=0);
            testCase.verifyEqual(problem.predictedState(:,1),problem.model.initialState,AbsTol=1e-7);
            testCase.verifyEqual(problem.metadata.solverCallCount,sum([problem.metadata.search.stages.numericalSolve]));
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            localVerifyAffinePrediction(testCase,problem,cfg);
            localVerifyAnchorRollout(testCase,anchor,cfg);
        end
        function failedShiftReinitializesPotentialFieldAndRebuildsBothStages(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,localNearTarget(),road,cfg,[]);
            ego=localSuccessor(ego,prior);prior.inputTrajectory(1,end-9:end)=.15;
            [command,inputs,problem]=collisionAvoidanceController(ego,[],road,cfg,prior);
            search=problem.metadata.search;
            testCase.verifyTrue(search.potentialFieldRestarted);
            testCase.verifyEqual([search.attempts.modelBuilt],[true,false,true]);
            testCase.verifyEqual(search.solverCalls,sum([search.stages.numericalSolve]));
            testCase.verifyEqual(search.linearizationCount,2);
            testCase.verifyEqual([search.attempts.initialization],["shiftedInputRollout","shiftedInputRollout","movingTargetPotentialField"]);
            testCase.verifyEqual(search.initializationFailure,"pcbfNoNumericalResult");
            testCase.verifyEqual([search.stages.objective],["inheritedSafetyBudget","clfSlack","pcbfSlack","pcbfSlack","clfSlack"]);
            testCase.verifyGreaterThan([search.attempts(search.selectedAttempt).stages.exitFlag],0);
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
            testCase.verifyTrue(search.potentialFieldRestarted);
            testCase.verifyEqual(search.linearizationCount,2);
            testCase.verifyEqual(search.solverCalls,sum([search.stages.numericalSolve]));
            testCase.verifyEqual(search.attempts(end).inputTrustScale,2);
            testCase.verifyEqual(search.attempts(end).modelBuilt,false);
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
            testCase.verifyEqual(problem.metadata.search.initialization,"movingTargetPotentialField");
            testCase.verifyEqual(problem.metadata.search.initializationFailure, ...
                "collisionAvoidanceController:invalidTireOperatingPoint");
            testCase.verifyFalse(problem.metadata.search.potentialFieldRestarted);
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
            % A nonzero feasible dual retains penetration depth at overlap.
            testCase.verifyEqual(problem.metadata.search.primaryLowerBound, ...
                cfg.vehicle.width+cfg.collision.safetyMarginMeters,AbsTol=1e-6);
            testCase.verifyGreaterThan(problem.metadata.search.primaryOptimum,0);
            testCase.verifyTrue(problem.metadata.search.stages(1).numericalSolve);
            testCase.verifyGreaterThan(problem.solution.safety,0);
            testCase.verifyFalse(problem.metadata.zeroSlack);
            testCase.verifyTrue(problem.metadata.search.clfStageCompleted);
            testCase.verifyLessThanOrEqual(problem.solution.safety, ...
                problem.metadata.search.slackCap+cfg.solver.feasibilityTolerance);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            testCase.verifyEqual(problem.metadata.search.attempts(1).initialization,"movingTargetPotentialField");
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
        function brakingEncounterUsesOneFreshModelAndCompletesTheClf(testCase)
            [ego,~,cfg]=localFixture();[~,q,road]=collisionThreatScenario("brakingLead",cfg);
            target=struct('targetPositionInertial',q(1:2), ...
                'targetVelocityInertial',q(4)*[cos(q(3)+q(6));sin(q(3)+q(6))], ...
                'targetYawInertial',q(3),'targetSideslip',q(6), ...
                'targetTangentialAcceleration',q(5),'targetRearAxleDistance',q(7));
            [command,inputs,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            search=problem.metadata.search;
            testCase.verifyEqual(search.linearizationCount,1);
            testCase.verifyFalse(search.potentialFieldRestarted);
            testCase.verifyTrue(search.clfStageCompleted);
            testCase.verifyGreaterThan(search.attempts(search.selectedAttempt).stages(1).exitFlag,0);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
        end
        function shiftedSlackBudgetSkipsPrimaryAndStillOptimizesClf(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            prior.stageSlacks=[.4,.02,.03,zeros(1,5)];prior.safetyBudget=sum(prior.stageSlacks);
            ego=localSuccessor(ego,prior);
            [command,inputs,problem,next]=collisionAvoidanceController(ego,[],road,cfg,prior);
            search=problem.metadata.search;
            testCase.verifyEqual(search.budgetSource,"shiftedTrajectory");
            testCase.verifyEqual(search.slackCap,.05,AbsTol=1e-14);
            testCase.verifyEqual(search.firstSlackCap,.02,AbsTol=1e-14);
            testCase.verifyEqual(search.solverCalls,1);
            testCase.verifyTrue(isnan(search.primaryOptimum));
            testCase.verifyTrue(search.clfStageCompleted);
            testCase.verifyFalse(problem.metadata.primaryOptimumComputed);
            testCase.verifyTrue(problem.metadata.optimizationConverged);
            testCase.verifyLessThanOrEqual(next.safetyBudget,.05+cfg.solver.feasibilityTolerance);
            testCase.verifyLessThanOrEqual(next.stageSlacks(1),.02+cfg.solver.feasibilityTolerance);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            testCase.verifyEqual(next.stageSlacks,problem.solution.stageSlacks,AbsTol=0);
        end
        function brokenBudgetRunsPrimaryRestorationBeforeIssuingANewPlan(testCase)
            [ego,road,cfg]=localFixture();target=localNearTarget();
            [~,~,~,prior]=collisionAvoidanceController(ego,target,road,cfg,[]);
            prior.stageSlacks=zeros(1,cfg.controller.horizonSteps);prior.safetyBudget=0;
            ego=localSuccessor(ego,prior);
            q=predictiveSafetyGeometry.predictTarget(prior.targetEpoch,cfg.controller.sampleTime);
            ego.position=q(1:2);
            [command,inputs,problem]=collisionAvoidanceController(ego,[],road,cfg,prior);
            search=problem.metadata.search;
            testCase.verifyEqual(search.attempts(1).budgetSource,"shiftedTrajectory");
            testCase.verifyEqual(search.attempts(1).terminationReason,"clfNoNumericalResult");
            testCase.verifyEqual(search.budgetSource,"primaryOptimum");
            testCase.verifyGreaterThan(search.primaryOptimum,0);
            testCase.verifyTrue(search.clfStageCompleted);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
        end
    end
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
