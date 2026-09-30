classdef nonlinearPredictiveSafetyTest < matlab.unittest.TestCase
    % Nominal PCBF/CLF/SCvx behavior; no native verifier dependency.
    properties (TestParameter)
        targetSideslip={0,1e-12,.08,-.08};
        targetAcceleration={0,1.2,-1.2};
        orientation={0,.5,2.9};
        curvature={0,.005,-.005};
        referenceSpeed={8,15};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function targetFlowHasTheSemigroupProperty(testCase,targetSideslip,targetAcceleration)
            q=[3;1;.2;7;targetAcceleration;targetSideslip;1.6;2.4;.95;0;0];
            first=predictiveSafetyGeometry.targetFlow(q,.37);
            actual=predictiveSafetyGeometry.targetFlow(first,.29);
            expected=predictiveSafetyGeometry.targetFlow(q,.66);
            testCase.verifyEqual(actual,expected,AbsTol=2e-14);
        end
        function targetFlowMatchesIndependentIntegration(testCase,targetSideslip,targetAcceleration)
            q=[3;1;.2;7;targetAcceleration;targetSideslip;1.6;2.4;.95;.2;-.1];
            [~,states]=ode45(@(~,x)[x(4)*cos(x(3)+targetSideslip); ...
                x(4)*sin(x(3)+targetSideslip);x(4)*sin(targetSideslip)/1.6;targetAcceleration], ...
                [0,1.3],q(1:4),odeset('RelTol',1e-12,'AbsTol',1e-13));
            actual=predictiveSafetyGeometry.targetFlow(q,1.3);
            testCase.verifyEqual(actual(1:4),states(end,:).',AbsTol=2e-10);
            testCase.verifyEqual(actual(5:end),q(5:end));
        end
        function accelerationChangesSpeedAndHeadingRate(testCase)
            q=[0;0;.2;6;1.5;.1;1.6;2.4;.95;0;0];
            next=predictiveSafetyGeometry.targetFlow(q,2);
            testCase.verifyEqual(next(4),9);
            testCase.verifyEqual(next(3),q(3)+15*sin(.1)/1.6,AbsTol=1e-14);
            testCase.verifyEqual(next(4)*sin(next(6))/next(7),1.5*q(4)*sin(q(6))/q(7),AbsTol=1e-14);
        end
        function brakingRetainsConstantAccelerationThroughZeroVelocity(testCase)
            q=[0;0;0;2;-2;0;1.6;2.4;.95;0;0];
            stopped=predictiveSafetyGeometry.targetFlow(q,1);
            reversed=predictiveSafetyGeometry.targetFlow(stopped,2);
            testCase.verifyEqual(stopped(1:4),[1;0;0;0]);
            testCase.verifyEqual(reversed(1:5),[-3;0;0;-4;-2]);
            testCase.verifyEqual(reversed,predictiveSafetyGeometry.targetFlow(q,3));
        end
        function aTurningTargetRetracesItsPathAfterStopping(testCase,targetSideslip)
            q=[3;1;.2;2;-2;targetSideslip;1.6;2.4;.95;0;0];
            stopped=predictiveSafetyGeometry.targetFlow(q,1);
            returned=predictiveSafetyGeometry.targetFlow(stopped,1);
            testCase.verifyEqual(returned(1:3),q(1:3),AbsTol=2e-14);
            testCase.verifyEqual(returned(4:7),[-2;-2;targetSideslip;1.6]);
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
            [~,~,cfg]=localFixture();z=[0;0;0;8;0;0;20;3;.4;6];parameters=[1.2;.08;1.6;2.4;.95;0;0];
            [next,a,b]=nonlinearBicycleModel.jointSample(z,[.01;.02],parameters,cfg);
            target=predictiveSafetyGeometry.targetFlow([z(7:10);parameters],cfg.controller.sampleTime);
            testCase.verifyEqual(next(7:10),target(1:4),AbsTol=1e-14);
            testCase.verifyEqual(a(1:6,7:10),zeros(6,4));
            testCase.verifyEqual(b(7:10,:),zeros(4,2));
            testCase.verifyEqual(nonlinearBicycleModel.jointSample(z,[.01;.02],parameters,cfg),next);
        end
        function jointTargetTangentIncludesSpeedDependentTurning(testCase,targetSideslip)
            [~,~,cfg]=localFixture();z=[0;0;0;8;0;0;20;3;.4;6];parameters=[1.2;targetSideslip;1.6;2.4;.95;0;0];
            [~,a]=nonlinearBicycleModel.jointSample(z,[0;0],parameters,cfg);
            finite=localTargetTangent([z(7:10);parameters],cfg.controller.sampleTime);
            testCase.verifyEqual(a(7:10,7:10),finite,AbsTol=2e-9);
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
            testCase.verifyFalse(problem.metadata.optimizationReturned);
            testCase.verifyTrue(problem.metadata.zeroSlack);
            testCase.verifyLessThan(problem.metadata.clfNextValue,problem.metadata.clfInitialValue);
            testCase.verifySize(command.actuatorInput,[2,1]);
            testCase.verifyFalse(isfield(problem,'certificate'));
            testCase.verifyEqual(problem.metadata.safetyScope,"nominalSampledPrediction");
        end
        function zeroAdditionalBufferAllowsSeparatedTargetControl(testCase)
            [ego,road,cfg]=localFixture();cfg.collision.safetyMarginMeters=0;
            target=localTarget([30;5;0;6;0;0;1.6;2.4;.95;0;0]);
            [command,inputs,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyFalse(problem.metadata.optimizationReturned);
            testCase.verifyEqual(problem.metadata.search.terminationReason,"feasibleWitness");
            testCase.verifyTrue(problem.metadata.zeroSlack);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1));
            testCase.verifyGreaterThan(problem.metadata.minimumCollisionMargin,0);
        end
        function changedTargetForecastRequiresExplicitReinitialization(testCase)
            [ego,road,cfg]=localFixture();target=localTarget([30;5;0;6;1;0;1.6;2.4;.95;0;0]);
            [~,~,~,prior]=collisionAvoidanceController(ego,target,road,cfg,[]);
            ego=localSuccessor(ego,prior);target.targetPositionInertial=[30.4;5.01];
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,prior), ...
                'collisionAvoidanceController:changedTargetTrajectory');
        end
        function headingWrapPreservesTheFixedTargetForecast(testCase)
            [ego,road,cfg]=localFixture();q=[100;100;pi-.001;8;0;.08;1.6;2.4;.95;0;0];
            [~,~,~,prior]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
            ego=localSuccessor(ego,prior);next=predictiveSafetyGeometry.targetFlow(prior.targetEpoch,cfg.controller.sampleTime);
            [~,~,problem,state]=collisionAvoidanceController(ego,localTarget(next),road,cfg,prior);
            testCase.verifyGreaterThan(next(3),pi);
            testCase.verifyEqual(state.targetEpoch,prior.targetEpoch);
            testCase.verifyEqual(problem.model.target,next);
            testCase.verifyEqual(problem.solution.hard,0);
            next(3)=next(3)+.01;
            testCase.verifyError(@()collisionAvoidanceController(ego,localTarget(next),road,cfg,prior), ...
                'collisionAvoidanceController:changedTargetTrajectory');
        end
        function targetDropoutUsesOneFixedAbsoluteEpoch(testCase)
            [ego,road,cfg]=localFixture();q=[30;5;0;6;1;0;1.6;2.4;.95;0;0];ego.stateTime=10;
            [~,~,~,prior]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
            ego=localSuccessor(ego,prior);ego.stateTime=10+cfg.controller.sampleTime;
            cfg.solver.timeLimitSeconds=1e-12;
            [~,~,problem,state]=collisionAvoidanceController(ego,[],road,cfg,prior);
            expected=predictiveSafetyGeometry.targetFlow(q,cfg.controller.sampleTime);
            testCase.verifyEqual(problem.model.target,expected,AbsTol=1e-12);
            testCase.verifyEqual(state.targetEpoch,q,AbsTol=1e-12);
            testCase.verifyEqual(problem.predictedJointState(7:10,1:end-1),prior.predictedJointState(7:10,2:end-1));
            testCase.verifyEqual(state.sampleIndex,1);
        end
        function nonconsecutiveSampleTimesCannotReuseTheWitness(testCase)
            [ego,road,cfg]=localFixture();ego.stateTime=10;
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,prior);ego.stateTime=10.17;
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,prior), ...
                'collisionAvoidanceController:invalidSampleTime');
        end
        function retiredStateCannotRestoreTheOldTargetMotion(testCase)
            [ego,road,cfg]=localFixture();prior=struct('version',54,'target',ones(10,1));
            [~,~,problem,state]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyEmpty(problem.model.target);
            testCase.verifyEqual(problem.metadata.search.initialization,"laneFeedbackRollout");
            testCase.verifyEqual(state.version,55);
        end
        function everyCallAppliesTheReturnedFeasibleFirstControl(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,prior);
            [command,inputs,problem,state]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyEqual(problem.metadata.solverCallCount,0);
            testCase.verifyTrue(problem.metadata.scvxConverged);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1));
            testCase.verifyEqual(state.appliedInput,inputs(:,1));
            testCase.verifyTrue(problem.metadata.search.shiftAvailable);
            testCase.verifyTrue(problem.metadata.zeroSlack);
        end
        function feasibleInitializationReturnsWithoutOptimization(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=1e-9;
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyEqual(problem.solution.hard,0);
            testCase.verifyEqual(problem.solution.safety,0);
            testCase.verifyLessThanOrEqual(problem.solution.cost,cfg.solver.optimalityTolerance);
            testCase.verifyEqual(problem.metadata.solverCallCount,0);
            testCase.verifyEqual(problem.metadata.search.terminationReason,"feasibleWitness");
        end
        function nontrivialFeasibleCostDoesNotTriggerAnImprovementSolve(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            cfg.nonlinear.maximumIterations=1;cfg.solver.timeLimitSeconds=60;
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyGreaterThan(problem.solution.cost,cfg.solver.optimalityTolerance);
            testCase.verifyEqual(problem.metadata.solverCallCount,0);
            testCase.verifyEmpty(problem.metadata.search.sequentialIterations);
            testCase.verifyEqual(problem.metadata.search.terminationReason,"feasibleWitness");
            testCase.verifyLessThan(problem.metadata.clfNextValue,problem.metadata.clfInitialValue);
            testCase.verifyEqual(problem.solution.hard,0);
        end
        function shiftedFeasiblePlanIsExecutedDespiteItsNonzeroCost(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.1;
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,prior);
            [command,~,problem]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyGreaterThan(problem.solution.cost,cfg.solver.optimalityTolerance);
            testCase.verifyEqual(problem.metadata.solverCallCount,0);
            testCase.verifyEqual(command.actuatorInput,prior.inputTrajectory(:,2));
            testCase.verifyEqual(problem.metadata.controlSource,"retainedContinuation");
            testCase.verifyEqual(problem.metadata.search.terminationReason,"feasibleWitness");
        end
        function cachedContinuationMatchesAFreshNonlinearEvaluation(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.01;cfg.solver.timeLimitSeconds=60;
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,prior);cfg.solver.timeLimitSeconds=1e-12;
            [~,cachedPlan,cached]=collisionAvoidanceController(ego,[],road,cfg,prior);
            prior.witness=rmfield(prior.witness,{'stageSafety','stageHard','stageCost','stageCollision'});
            [~,freshPlan,fresh]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyEqual(cachedPlan,freshPlan);
            testCase.verifyEqual(cached.solution.states,fresh.solution.states,AbsTol=1e-12);
            testCase.verifyEqual(cached.solution.cost,fresh.solution.cost,AbsTol=1e-12);
            testCase.verifyEqual(cached.solution.safety,fresh.solution.safety);
            testCase.verifyEqual(cached.solution.hard,fresh.solution.hard);
            testCase.verifyEqual(cached.solution.clfSlack,fresh.solution.clfSlack,AbsTol=1e-12);
        end
        function anOffsetBrakingTargetDoesNotCreateAFictitiousLongTail(testCase,referenceSpeed)
            [ego,road,cfg]=localFixture();
            cfg.referenceSpeed=referenceSpeed;ego.speed=referenceSpeed;
            cfg.controller.horizonSteps=referenceSpeed+mod(referenceSpeed,2);
            target=localTarget([24;6;pi;2;-1;0;1.6;2.4;.95;0;0]);
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            expected=cfg.controller.horizonSteps+ceil(cfg.nonlinear.recoveryHorizonSeconds/cfg.controller.sampleTime);
            testCase.verifyEqual(problem.metadata.horizonSteps,expected);
            testCase.verifyEqual(problem.metadata.solverCallCount,0);
            testCase.verifyEqual(problem.solution.hard,0);
        end
        function opposingHeadingPreservesExactlyStraightTargetMotion(testCase)
            q=[24;6;pi;2;-1;0;1.6;2.4;.95;0;0];
            next=predictiveSafetyGeometry.targetFlow(q,1e8);
            testCase.verifyEqual(next(2),q(2));
            q(3)=pi-1e-12;
            next=predictiveSafetyGeometry.targetFlow(q,1e8);
            testCase.verifyLessThan(next(2),q(2)-1);
        end
        function tinyPhysicalTransverseAccelerationStillAffectsInfiniteSeparation(testCase)
            [~,~,cfg]=localFixture();seed=localSeedAtOrigin(cfg,0);
            q=[24;6;pi-1e-12;2;-1e-18;0;1.6;2.4;.95;0;0];
            testCase.verifyLessThan(terminalContinuation.separation(seed,q,[0;0;0;0;4;4],cfg),0);
            q(3)=pi;
            testCase.verifyGreaterThan(terminalContinuation.separation(seed,q,[0;0;0;0;4;4],cfg),0);
        end
        function straightEndpointSeparationIsIndependentOfWorldHeading(testCase,orientation)
            [~,~,cfg]=localFixture();seed=localSeedAtOrigin(cfg,0);
            q=[24;6;pi;2;-1;0;1.6;2.4;.95;0;0];
            expected=terminalContinuation.separation(seed,q,[0;0;0;0;4;4],cfg);
            rotation=[cos(orientation),-sin(orientation);sin(orientation),cos(orientation)];
            q(1:2)=rotation*q(1:2);q(3)=q(3)+orientation;
            seed.epochState(1:2)=rotation*seed.epochState(1:2);seed.epochState(3)=seed.epochState(3)+orientation;
            actual=terminalContinuation.separation(seed,q,[0;0;orientation;0;4;4],cfg);
            testCase.verifyGreaterThan(actual,0);
            testCase.verifyEqual(actual,expected,AbsTol=1e-12);
        end
        function solverDeadlineRetainsTheFeasibleContinuation(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,prior);cfg.solver.timeLimitSeconds=1e-12;
            [command,~,problem]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyEqual(command.actuatorInput,prior.inputTrajectory(:,2));
            testCase.verifyEqual(problem.metadata.controlSource,"retainedContinuation");
            testCase.verifyEqual(problem.solution.hard,0);
            testCase.verifyLessThanOrEqual(problem.solution.safety,sum(prior.witness.stageSlacks(2:end)));
        end
        function positiveSlackReportsSafetyRecoveryWithoutClaimingSafety(testCase)
            [ego,road,cfg]=localFixture();cfg.nonlinear.maximumIterations=2;
            target=localTarget([0;0;pi;8;0;0;1.6;2.4;.95;0;0]);
            [command,inputs,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyEqual(command.actuatorInput,inputs(:,1));
            testCase.verifyGreaterThan(problem.metadata.predictiveBarrierValue,0);
            testCase.verifyFalse(problem.metadata.zeroSlack);
            testCase.verifyFalse(problem.metadata.optimizationReturned);
            testCase.verifyEqual(problem.metadata.search.terminationReason,"feasibleWitness");
        end
        function oncomingAvoidanceStartsFromALaneRollout(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=60;
            road.lateralClearance=[.05;.05];
            target=localTarget([24;0;pi;8;0;0;1.6;2.4;.95;0;0]);
            [~,plan,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(problem.metadata.zeroSlack);
            testCase.verifyGreaterThan(max(abs(plan(1,:))),.01);
            testCase.verifyFalse(problem.metadata.preplannedAvoidanceTrajectoryRequired);
            testCase.verifyFalse(problem.metadata.drivingModeSwitching);
            testCase.verifyFalse(problem.metadata.roadConstraintsEnforced);
            testCase.verifyEqual(problem.metadata.search.initialization,"laneFeedbackRollout");
            steps=problem.metadata.search.sequentialIterations;
            testCase.verifyEqual(problem.metadata.search.terminationReason,"feasibleWitness");
            testCase.verifyTrue(steps{end}.retainedAsWitness);
            testCase.verifyEqual(sum(cellfun(@(step)step.retainedAsWitness,steps)),1);
            for j=1:numel(steps)
                step=steps{j};
                if step.status=="solved"
                    testCase.verifyLessThanOrEqual(step.secondarySafety,step.safetyCap+cfg.solver.feasibilityTolerance);
                end
            end
        end
        function correctedPrimaryAvoidanceReturnsBeforeCostRefinement(testCase)
            [ego,road,cfg]=localFixture();ego.speed=15;
            cfg.referenceSpeed=15;cfg.controller.horizonSteps=16;
            cfg.solver.timeLimitSeconds=30;
            target=localTarget([24;0;pi;8;0;0;1.6;2.4;.95;0;0]);
            [command,plan,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyEqual(command.actuatorInput,plan(:,1));
            testCase.verifyEqual(problem.solution.hard,0);
            testCase.verifyEqual(problem.solution.safety,0);
            testCase.verifyGreaterThan(problem.metadata.minimumCollisionMargin,0);
            testCase.verifyFalse(problem.metadata.roadConstraintsEnforced);
            endpoint=[problem.solution.states(:,end);plan(:,end)];
            value=terminalContinuation.membership(endpoint, ...
                problem.model.sampleIndex+size(plan,2),problem.model.terminal);
            testCase.verifyLessThanOrEqual(value,0);
            steps=problem.metadata.search.sequentialIterations;
            testCase.verifyEqual(problem.metadata.search.terminationReason,"feasibleWitness");
            testCase.verifyEqual(steps{end}.status,"primaryFeasible");
            testCase.verifyEqual(steps{end}.calls,1);
            testCase.verifyFalse(steps{end}.secondaryReturned);
            testCase.verifyTrue(steps{end}.retainedAsWitness);
        end
        function brakingLeadRestorationPreservesBetterNonlinearCandidates(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=30;
            cfg.nonlinear.maximumIterations=6;
            target=localTarget([6.25;0;0;8;-.5;0;1.6;2.4;.95;0;0]);
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            steps=problem.metadata.search.sequentialIterations;
            selected=cellfun(@(step)step.nominalRestorationMerit,steps);
            raw=cellfun(@(step)min([step.primaryRawMerit,step.secondaryRawMerit,step.restorationRawMerit]),steps);
            accepted=cellfun(@(step)step.acceptedIterate,steps);
            testCase.verifyEqual(problem.solution.hard,0,AbsTol=0);
            testCase.verifyEqual(problem.solution.safety,0,AbsTol=0);
            testCase.verifyLessThanOrEqual(selected,raw);
            testCase.verifyTrue(any(selected<raw));
            testCase.verifyLessThanOrEqual(diff(selected(accepted)),zeros(1,sum(accepted)-1));
            testCase.verifyLessThanOrEqual(numel(steps),6);
        end
        function turningCrossingRestorationStillRequiresTheOriginalTerminalSet(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=30;
            impact=[8*1.6;0;-pi/2-.05;10;1;.05;1.6;2.4;.95;0;0];
            target=localTarget(predictiveSafetyGeometry.targetFlow(impact,-1.6));
            [command,plan,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            endpoint=[problem.solution.states(:,end);plan(:,end)];
            value=terminalContinuation.membership(endpoint,size(plan,2),problem.model.terminal);
            steps=problem.metadata.search.sequentialIterations;
            testCase.verifyEqual(command.actuatorInput,plan(:,1));
            testCase.verifyEqual(problem.solution.hard,0,AbsTol=0);
            testCase.verifyEqual(problem.solution.safety,0,AbsTol=0);
            testCase.verifyLessThanOrEqual(value,0);
            testCase.verifyGreaterThanOrEqual(problem.solution.terminalSeparationMargin,0);
            testCase.verifyTrue(any(cellfun(@(step)step.completionRestoration,steps)));
            testCase.verifyTrue(steps{end}.retainedAsWitness);
            testCase.verifyEqual(sum(cellfun(@(step)step.retainedAsWitness,steps)),1);
            testCase.verifyFalse(problem.metadata.drivingModeSwitching);
        end
        function restoredCrossingRemainsSeparatedBetweenConstraintSamples(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=30;
            q=[12.8;12.8;-pi/2;8;0;0;1.6;2.4;.95;0;0];
            [~,plan,problem]=collisionAvoidanceController(ego,localTarget(q),road,cfg,[]);
            x=problem.model.initialState;minimum=Inf;h=cfg.controller.sampleTime;
            shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
            for index=1:size(plan,2)
                [times,states]=ode45(@(~,state)nonlinearBicycleModel.derivative(state,plan(:,index),cfg), ...
                    linspace(0,h,31),x,odeset('RelTol',1e-11,'AbsTol',1e-12));
                for sample=1:numel(times)
                    target=predictiveSafetyGeometry.targetFlow(q,(index-1)*h+times(sample));
                    minimum=min(minimum,predictiveSafetyGeometry.rectangle(states(sample,1:3).',shape,target(1:3),target(8:11)));
                end
                x=states(end,:).';
            end
            testCase.verifyEqual(problem.solution.hard,0,AbsTol=0);
            testCase.verifyEqual(problem.solution.safety,0,AbsTol=0);
            testCase.verifyGreaterThan(minimum,cfg.collision.safetyMarginMeters);
        end
        function terminalLaneFeedbackNeedsNoRoadClearance(testCase,curvature)
            [ego,~,cfg]=localFixture();trim=nonlinearBicycleModel.cruise(cfg,curvature);
            ego.yaw=trim.state(3);ego.speed=trim.state(4);ego.lateralVelocity=trim.state(5);ego.yawRate=trim.state(6);
            road=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200), ...
                'lateralClearance',[.05;.05]);
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyTrue(problem.metadata.zeroSlack);
            testCase.verifyLessThanOrEqual(problem.solution.hard,cfg.solver.feasibilityTolerance);
        end
        function returningTargetOrbitMustBeSeparatedAtTheEndpointSeed(testCase)
            [~,~,cfg]=localFixture();frame=[0;0;0;0;4;4];seed=localSeedAtOrigin(cfg,0);
            q=[10;0;0;8;1;asin(.16);1.6;2.4;.95;0;0];
            margin=terminalContinuation.separation(seed,q,frame,cfg);
            testCase.verifyLessThan(margin,0);
        end
        function acceleratingTargetBehindCannotUseAConstantSpeedSeed(testCase)
            [~,~,cfg]=localFixture();seed=localSeedAtOrigin(cfg,0);
            q=[-20;0;0;4;1;0;1.6;2.4;.95;0;0];
            testCase.verifyLessThan(terminalContinuation.separation(seed,q,[0;0;0;0;4;4],cfg),0);
        end
        function acceleratingTargetAheadAllowsIndefiniteFollowing(testCase)
            [~,~,cfg]=localFixture();seed=localSeedAtOrigin(cfg,0);
            q=[20;0;0;10;1;0;1.6;2.4;.95;0;0];
            testCase.verifyGreaterThan(terminalContinuation.separation(seed,q,[0;0;0;0;4;4],cfg),0);
        end
        function deceleratingTargetHasABoundedForwardExcursion(testCase)
            [~,~,cfg]=localFixture();seed=localSeedAtOrigin(cfg,0);
            q=[-20;0;0;10;-2;0;1.6;2.4;.95;0;0];
            testCase.verifyGreaterThan(terminalContinuation.separation(seed,q,[0;0;0;0;4;4],cfg),0);
        end
        function aBrakingTargetCanReturnFromOutsideACircularLane(testCase)
            [~,~,cfg]=localFixture();seed=localSeedAtOrigin(cfg,.005);
            q=[250;200;0;5;-1;0;1.6;2.4;.95;0;0];
            testCase.verifyLessThan(terminalContinuation.separation(seed,q,[0;0;0;.005;4;4],cfg),0);
        end
        function aStationaryTargetWithSideslipRemainsAFixedRectangle(testCase)
            [~,~,cfg]=localFixture();seed=localSeedAtOrigin(cfg,0);
            q=[0;10;0;0;0;-.2;1.6;2.4;.95;0;0];
            testCase.verifyGreaterThan(terminalContinuation.separation(seed,q,[0;0;0;0;4;4],cfg),0);
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
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'controller',struct('horizonSteps',8)));
    ego=struct('position',[0;0],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
    road=struct('centerline',[-100,0;1000,0],'lateralClearance',[4;4]);
end
function target=localTarget(q)
    velocity=q(4)*[cos(q(3)+q(6));sin(q(3)+q(6))];
    target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',velocity, ...
        'targetYawInertial',q(3),'targetTangentialAcceleration',q(5),'targetSideslip',q(6), ...
        'targetRearAxleDistance',q(7),'targetLength',2*q(8),'targetWidth',2*q(9));
end
function finite=localTargetTangent(q,time)
    finite=zeros(4);h=1e-6;
    for j=1:4
        lo=q;hi=q;lo(j)=lo(j)-h;hi(j)=hi(j)+h;
        a=predictiveSafetyGeometry.targetFlow(hi,time);b=predictiveSafetyGeometry.targetFlow(lo,time);
        finite(:,j)=(a(1:4)-b(1:4))/(2*h);
    end
end
function ego=localSuccessor(ego,prior)
    x=prior.stateTrajectory(:,2);ego.position=x(1:2);ego.yaw=x(3);ego.speed=x(4);
    ego.lateralVelocity=x(5);ego.yawRate=x(6);ego.heldActuatorInput=prior.appliedInput;
end

function seed=localSeedAtOrigin(cfg,curvature)
    seed=terminalContinuation.build(cfg,curvature);
    lane=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200));
    seed=terminalContinuation.anchor(seed,seed.reference.state,0,lane);
end
