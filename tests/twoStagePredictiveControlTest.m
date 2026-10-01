classdef twoStagePredictiveControlTest < matlab.unittest.TestCase
    % Two lexicographic objectives on one shared affine prediction model.
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function nominalInitializationStillOptimizesBothObjectives(testCase)
            [ego,road,cfg]=localFixture();
            [command,inputs,problem,state]=collisionAvoidanceController(ego,[],road,cfg,[]);
            search=problem.metadata.search;
            testCase.verifyEqual(search.solverCalls,2);
            testCase.verifyEqual([search.stages.objective],["pcbfSlack","clfSlack"]);
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
        function shiftedAffineTrajectoryIsOnlyTheNextLinearization(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,prior);ego.position(2)=ego.position(2)+.01;
            [command,inputs,problem]=collisionAvoidanceController(ego,[],road,cfg,prior);
            anchor=problem.model.linearization;
            testCase.verifyEqual(problem.metadata.search.initialization,"shiftedLinearization");
            testCase.verifyEqual(anchor.states(:,1:end-1),prior.stateTrajectory(:,2:end),AbsTol=0);
            testCase.verifyEqual(anchor.inputs(:,1:end-1),prior.inputTrajectory(:,2:end),AbsTol=0);
            testCase.verifyEqual(problem.predictedState(:,1),problem.model.initialState,AbsTol=1e-7);
            testCase.verifyEqual(problem.metadata.solverCallCount,2);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            localVerifyAffinePrediction(testCase,problem,cfg);
        end
        function exhaustedBudgetDoesNotExecuteThePreviousTrajectory(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,prior);cfg.solver.timeLimitSeconds=1e-12;
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,prior), ...
                'collisionAvoidanceController:noOptimizationSolution');
        end
        function initialOverlapIsReportedAsOptimizedPositiveSlack(testCase)
            [ego,road,cfg]=localFixture();
            target=struct('targetPositionInertial',[0;0], ...
                'targetVelocityInertial',[40;0],'targetYawInertial',0);
            [command,inputs,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyGreaterThan(problem.metadata.search.primaryOptimum,0);
            testCase.verifyGreaterThan(problem.solution.safety,0);
            testCase.verifyFalse(problem.metadata.zeroSlack);
            testCase.verifyTrue(problem.metadata.search.clfStageCompleted);
            testCase.verifyLessThanOrEqual(problem.solution.safety, ...
                problem.metadata.search.slackCap+cfg.solver.feasibilityTolerance);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            testCase.verifyEqual(problem.metadata.search.initialization,"movingTargetFlow");
            localVerifyAffinePrediction(testCase,problem,cfg);
        end
        function finiteSlewAndMagnitudeBoundsConstrainTheReturnedPlan(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.05;
            cfg.model.frontWheelSteeringRateMaximum=.8;cfg.model.brakingRatioRateMaximum=2;
            ego.heldActuatorInput=[0;0];
            [~,inputs,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            limits=cfg.controller.sampleTime*[.8;2];tol=cfg.solver.feasibilityTolerance;
            testCase.verifyLessThanOrEqual(max(abs(diff([ego.heldActuatorInput,inputs],1,2)),[],2),limits+tol);
            testCase.verifyLessThanOrEqual(abs(inputs(1,:)),cfg.model.frontWheelSteeringAngleMaximum+tol);
            testCase.verifyLessThanOrEqual(inputs(2,:),cfg.actuation.brakingRatioMaximum+tol);
            testCase.verifyGreaterThanOrEqual(inputs(2,:),cfg.actuation.brakingRatioMinimum-tol);
            testCase.verifyLessThanOrEqual(problem.solution.hard,tol);
        end
    end
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
    testCase.verifyLessThanOrEqual(problem.solution.hard,cfg.solver.feasibilityTolerance);
    testCase.verifyEqual(size(inputs,2),min(cfg.controller.maximumHorizonSteps, ...
        cfg.controller.horizonSteps+ceil(cfg.nonlinear.recoveryHorizonSeconds/cfg.controller.sampleTime)));
end
function [ego,road,cfg]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'controller',struct('horizonSteps',8)));
    ego=struct('position',[0;0],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
    road=struct('centerline',[-100,0;1000,0]);
end
function ego=localSuccessor(ego,prior)
    x=prior.stateTrajectory(:,2);ego.position=x(1:2);ego.yaw=x(3);ego.speed=x(4);
    ego.lateralVelocity=x(5);ego.yawRate=x(6);ego.heldActuatorInput=prior.appliedInput;
end
