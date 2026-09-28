classdef nonlinearPredictiveSafetyTest < matlab.unittest.TestCase
    % Nominal PCBF/CLF/SCvx behavior; no native verifier dependency.
    properties (TestParameter)
        targetTurn={0,1e-12,.08,-.08};
        orientation={0,.5,2.9};
        curvature={0,.005,-.005};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function targetFlowHasTheSemigroupProperty(testCase,targetTurn)
            q=[3;1;.2;7;.1;targetTurn;2.4;.95;0;0];
            first=predictiveSafetyGeometry.targetFlow(q,.37);
            actual=predictiveSafetyGeometry.targetFlow(first,.29);
            expected=predictiveSafetyGeometry.targetFlow(q,.66);
            testCase.verifyEqual(actual,expected,AbsTol=2e-14);
        end
        function targetSpeedAndHeadingRateAreIndependent(testCase,targetTurn)
            [ego,road,cfg]=localFixture();
            target=localTarget([30;5;.2;6;0;targetTurn;2.4;.95;0;0]);
            [~,~,~,obs]=readPlanningInputs(ego,target,road,cfg);
            q=predictiveSafetyGeometry.target(obs,target,cfg);
            testCase.verifyEqual(q(4),6,AbsTol=1e-14);
            testCase.verifyEqual(q(6),targetTurn);
        end
        function rectangleDualMatchesDistance(testCase,orientation)
            e=[0;0;orientation];t=[10;3;-.2];shape=[2.4;.95;0;0];
            [distance,dual]=predictiveSafetyGeometry.rectangle(e,shape,t,shape);
            testCase.verifyGreaterThan(distance,0);
            testCase.verifyEqual(dual.value,distance,AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual([dual.mu;dual.lambda],zeros(8,1));
            testCase.verifyEqual(norm(dual.normal),1,AbsTol=1e-12);
        end
        function overlapHasANonzeroRestorationDirection(testCase)
            shape=[2.4;.95;0;0];
            rows=predictiveSafetyGeometry.dualLinearization([0;0;0],shape,[0;0;0],shape,[0;1]);
            testCase.verifyLessThan(rows.signedDistance,0);
            testCase.verifyEqual(rows.normal,[0;1]);
            testCase.verifyEqual(rows.jacobian(:,1:2),repmat([0,1],4,1));
        end
        function analyticDynamicsTangentMatchesFiniteDifferences(testCase,curvature)
            [~,~,cfg]=localFixture();x=[2;.2;.1;8;.3;.1];u=[.03;-.12];
            [a,b]=nonlinearBicycleModel.jacobian(x,u,cfg,curvature);
            finite=zeros(6,8);point=[x;u];h=1e-6;
            for j=1:8
                lo=point;hi=point;lo(j)=lo(j)-h;hi(j)=hi(j)+h;
                finite(:,j)=(nonlinearBicycleModel.derivative(hi(1:6),hi(7:8),cfg,curvature) ...
                    -nonlinearBicycleModel.derivative(lo(1:6),lo(7:8),cfg,curvature))/(2*h);
            end
            testCase.verifyEqual([a,b],finite,AbsTol=2e-6);
        end
        function jointTargetCoordinatesAreUnactuated(testCase)
            [~,~,cfg]=localFixture();z=[0;0;0;8;0;0;20;3;.4];parameters=[6;0;.08;2.4;.95;0;0];
            [next,a,b]=nonlinearBicycleModel.jointSample(z,[.01;.02],parameters,cfg);
            target=predictiveSafetyGeometry.targetFlow([z(7:9);parameters],cfg.controller.sampleTime);
            testCase.verifyEqual(next(7:9),target(1:3),AbsTol=1e-14);
            testCase.verifyEqual(a(1:6,7:9),zeros(6,3));
            testCase.verifyEqual(b(7:9,:),zeros(3,2));
            testCase.verifyEqual(nonlinearBicycleModel.jointSample(z,[.01;.02],parameters,cfg),next);
        end
        function fullAndHalfHoldsUseTheSamePredictionMap(testCase)
            [~,~,cfg]=localFixture();x=[0;.2;.1;8;.3;.1];u=[.03;-.12];
            [full,a,b]=nonlinearBicycleModel.sample(x,u,cfg);
            [middle,am,bm]=nonlinearBicycleModel.sample(x,u,cfg,[],cfg.controller.sampleTime/2);
            [last,an,bn]=nonlinearBicycleModel.sample(middle,u,cfg,[],cfg.controller.sampleTime/2);
            testCase.verifyEqual(last,full,AbsTol=1e-14);
            testCase.verifyEqual(an*am,a,AbsTol=1e-13);
            testCase.verifyEqual(an*bm+bn,b,AbsTol=1e-13);
        end
        function laneFollowingRunsWithNoNativeVerifier(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            [command,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyTrue(problem.metadata.planFeasible);
            testCase.verifyTrue(problem.metadata.zeroSlack);
            testCase.verifyLessThan(problem.metadata.clfNextValue,problem.metadata.clfInitialValue);
            testCase.verifySize(command.actuatorInput,[2,1]);
            testCase.verifyFalse(isfield(problem,'certificate'));
            testCase.verifyEqual(problem.metadata.safetyScope,"nominalSampledPrediction");
        end
        function targetMeasurementsInitializeEachNewJointState(testCase)
            [ego,road,cfg]=localFixture();target=localTarget([30;5;0;6;0;0;2.4;.95;0;0]);
            [~,~,~,prior]=collisionAvoidanceController(ego,target,road,cfg,[]);
            ego=localSuccessor(ego,prior);ego.position(2)=ego.position(2)+1e-4;
            target.targetPositionInertial=[30.4;5.01];
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,prior);
            testCase.verifyEqual(problem.model.jointState(7:8),[30.4;5.01]);
            testCase.verifyEqual(problem.metadata.search.initialization,"shiftedPlan");
        end
        function targetDropoutUsesTheConstantParameterPrediction(testCase)
            [ego,road,cfg]=localFixture();target=localTarget([30;5;0;6;0;0;2.4;.95;0;0]);
            [~,~,~,prior]=collisionAvoidanceController(ego,target,road,cfg,[]);
            ego=localSuccessor(ego,prior);
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,prior);
            expected=predictiveSafetyGeometry.targetFlow(prior.target,cfg.controller.sampleTime);
            testCase.verifyEqual(problem.model.target,expected,AbsTol=1e-12);
        end
        function budgetExhaustionKeepsAFeasibleShiftedPlan(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            cfg.solver.frameDeadlineSeconds=1e-12;ego=localSuccessor(ego,prior);
            [~,plan,problem]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyTrue(problem.metadata.shiftedPlanAvailable);
            testCase.verifyEqual(problem.metadata.solverCallCount,0);
            testCase.verifySize(plan,size(prior.plan));
            testCase.verifyEqual(problem.metadata.solutionSource,"shiftedPlan");
        end
        function positiveSlackReportsSafetyRecoveryWithoutClaimingSafety(testCase)
            [ego,road,cfg]=localFixture();cfg.nonlinear.maximumIterations=2;
            target=localTarget([0;0;pi;8;0;0;2.4;.95;0;0]);
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyGreaterThan(problem.metadata.predictiveBarrierValue,0);
            testCase.verifyFalse(problem.metadata.zeroSlack);
            testCase.verifyTrue(problem.metadata.planFeasible);
        end
        function oncomingAvoidanceStartsFromALaneRollout(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=60;
            target=localTarget([24;0;pi;8;0;0;2.4;.95;0;0]);
            [~,plan,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(problem.metadata.zeroSlack);
            testCase.verifyGreaterThan(max(abs(plan(1,:))),.01);
            testCase.verifyFalse(problem.metadata.preplannedAvoidanceTrajectoryRequired);
            testCase.verifyFalse(problem.metadata.drivingModeSwitching);
            testCase.verifyEqual(problem.metadata.search.initialization,"laneFeedbackRollout");
            steps=problem.metadata.search.sequentialIterations;
            for j=1:numel(steps)
                step=steps{j};
                if step.status=="solved"
                    testCase.verifyLessThanOrEqual(step.secondarySafety,step.safetyCap+1e-7);
                end
            end
        end
        function terminalLaneFeedbackWorksOnBothCurvatureSigns(testCase,curvature)
            [ego,~,cfg]=localFixture();trim=nonlinearBicycleModel.cruise(cfg,curvature);
            ego.yaw=trim.state(3);ego.speed=trim.state(4);ego.lateralVelocity=trim.state(5);ego.yawRate=trim.state(6);
            road=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200), ...
                'lateralClearance',[4;4]);
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyTrue(problem.metadata.zeroSlack);
            testCase.verifyLessThanOrEqual(problem.solution.hard,cfg.solver.feasibilityTolerance);
        end
        function returningTargetOrbitIsExcludedByTheTerminalSet(testCase)
            [~,~,cfg]=localFixture();frame=[0;0;0;0;4;4];
            terminal=predictiveSafetyGeometry.terminalSet(cfg,frame);
            % This complete circular sweep still reaches the lane ahead.
            q=[10;0;0;8;0;.8;2.4;.95;0;0];
            margin=predictiveSafetyGeometry.terminalMargin([0;0;0;8;0;0],q,frame,terminal,[2.4;.95;0;0],.1);
            testCase.verifyLessThan(margin,0);
        end
        function finiteSlewLimitsUseAppliedInputMemory(testCase)
            [ego,road,cfg]=localFixture();cfg.model.frontWheelSteeringRateMaximum=.5;
            cfg.model.brakingRatioRateMaximum=.5;ego.heldActuatorInput=[0;0];
            [~,plan]=collisionAvoidanceController(ego,[],road,cfg,[]);
            rate=.5*cfg.controller.sampleTime;
            testCase.verifyLessThanOrEqual(abs(diff([ego.heldActuatorInput,plan],1,2)),rate+1e-10);
            ego=rmfield(ego,'heldActuatorInput');
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:missingInputMemory');
        end
    end
end

function [ego,road,cfg]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'controller',struct('horizonSteps',8), ...
        'nonlinear',struct('maximumImprovementIterations',0)));
    ego=struct('position',[0;0],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
    road=struct('centerline',[-100,0;1000,0],'lateralClearance',[4;4]);
end
function target=localTarget(q)
    velocity=q(4)*[cos(q(3)+q(5));sin(q(3)+q(5))];
    target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',velocity, ...
        'targetYawInertial',q(3),'targetYawRate',q(6),'targetLength',2*q(7),'targetWidth',2*q(8));
end
function ego=localSuccessor(ego,prior)
    x=prior.predictedState(:,2);ego.position=x(1:2);ego.yaw=x(3);ego.speed=x(4);
    ego.lateralVelocity=x(5);ego.yawRate=x(6);ego.heldActuatorInput=prior.appliedInput;
end
