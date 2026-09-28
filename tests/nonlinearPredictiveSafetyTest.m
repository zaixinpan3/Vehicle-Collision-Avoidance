classdef nonlinearPredictiveSafetyTest < matlab.unittest.TestCase
    properties (TestParameter)
        targetTurn={0,1e-12,.08,-.08};
        orientation={0,.5,2.9};
        curvature={0,.005,-.005};
        perturbation={[-1;-1],[-1;1],[1;-1],[1;1]};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            output=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            buildFialaIntervalVerifier(string(output.Folder));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(output.Folder));
        end
    end
    methods (Test)
        function targetFlowHasConsistentShiftedPredictions(testCase,targetTurn)
            q=localTarget(targetTurn);
            first=nonlinearSafetyCertificate.targetFlow(q,.37);
            shifted=nonlinearSafetyCertificate.targetFlow(first,.29);
            direct=nonlinearSafetyCertificate.targetFlow(q,.66);
            testCase.verifyEqual(shifted,direct,'AbsTol',1e-12);
        end
        function targetBodyHeadingDiffersFromItsVelocityCourse(testCase)
            q=localTarget(.08);next=nonlinearSafetyCertificate.targetFlow(q,1e-5);
            velocity=(next(1:2)-q(1:2))/1e-5;
            testCase.verifyEqual(atan2(velocity(2),velocity(1)),q(3)+q(5),'AbsTol',1e-6);
            testCase.verifyGreaterThan(abs(q(5)),.01);
        end
        function velocityCourseCannotSilentlyReplaceBodyHeading(testCase)
            [ego,road,cfg]=localFixture();target=rmfield(localObservation(),'targetYawInertial');
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:missingBodyHeading');
        end
        function directedTargetFlowContainsTheAnalyticTrajectory(testCase,targetTurn)
            q=localTarget(targetTurn);bounds=nonlinearSafetyMex('target',q,[.3;.4]);
            states=localTargetSamples(q,linspace(.3,.4,101));
            testCase.verifyGreaterThanOrEqual(states(1:3,:),bounds(:,1));
            testCase.verifyLessThanOrEqual(states(1:3,:),bounds(:,2));
        end
        function exactDualWitnessAgreesWithRectangleDistance(testCase,orientation)
            e=[0;0;orientation];t=[9;3;-.4];shape=[2.4;.95;.4;-.1];
            [distance,certificate]=nonlinearSafetyCertificate.rectangle(e,shape,t,shape);
            h=[eye(2);-eye(2)];re=localRotation(e(3));rt=localRotation(t(3));
            testCase.verifyGreaterThan(distance,0);
            testCase.verifyEqual(certificate.value,distance,'AbsTol',1e-12);
            testCase.verifyEqual(re.'*certificate.normal+h.'*certificate.mu,zeros(2,1),'AbsTol',1e-12);
            testCase.verifyEqual(-rt.'*certificate.normal+h.'*certificate.lambda,zeros(2,1),'AbsTol',1e-12);
            testCase.verifyGreaterThanOrEqual([certificate.mu;certificate.lambda],0);
        end
        function overlappingRectanglesCannotProducePositiveClearance(testCase)
            [distance,witness]=nonlinearSafetyCertificate.rectangle([0;0;.3],[2;1;0;0],[0;0;-.4],[2;1;0;0]);
            testCase.verifyEqual(distance,0);testCase.verifyEqual(witness.value,0);
        end
        function variationalFlowMatchesNonlinearEndpointPerturbations(testCase)
            [~,~,cfg]=localFixture();x=[0;0;.1;8;.1;.02];u=[.02;.03];
            [next,a,b]=nonlinearBicycleModel.sample(x,u,cfg);
            dx=[.02;-.01;.002;.005;-.003;.001]*1e-3;du=[.001;-.002]*1e-3;
            actual=nonlinearBicycleModel.sample(x+dx,u+du,cfg);
            testCase.verifyEqual(actual,next+a*dx+b*du,'AbsTol',2e-9);
        end
        function velocityIsNotSilentlyFloored(testCase)
            [~,~,cfg]=localFixture();
            testCase.verifyError(@()nonlinearBicycleModel.sample([0;0;0;0;0;0],[0;0],cfg), ...
                'collisionAvoidanceController:nonlinearDomain');
        end
        function invariantCruiseRegionIsComputedForBothTurnDirections(testCase,curvature)
            [~,~,cfg]=localFixture();b=nonlinearSafetyCertificate.backup(cfg,[0;0;0;curvature;4;4]);
            testCase.verifyGreaterThan(b.invarianceMargin,0);
            testCase.verifyLessThan(b.contractionUpper,1);
            testCase.verifyGreaterThan(b.radius,0);
            testCase.verifyEqual(hypot(b.reference.state(4),b.reference.state(5)),cfg.referenceSpeed,'AbsTol',1e-10);
        end
        function controllerExecutesOnlyAValidatedNonlinearPlan(testCase)
            [ego,road,cfg]=localFixture();[u,plan,p,state]=collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyTrue(p.certificate.accepted);
            testCase.verifyTrue(p.metadata.wholeHoldCertificate);
            testCase.verifyEqual(u.actuatorInput,plan(:,1));
            testCase.verifyEqual(state.version,48);
            testCase.verifyEqual(p.metadata.predictiveBarrierValue,0);
        end
        function defaultCruiseSpeedHasAnAdmissibleInvariantContinuation(testCase)
            [ego,road,~]=localFixture();cfg=collisionAvoidanceControllerConfig();
            ego.speed=cfg.referenceSpeed;cfg.nonlinear.maximumImprovementIterations=0;
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyTrue(problem.certificate.accepted);
            testCase.verifyGreaterThan(problem.model.backup.invarianceMargin,0);
        end
        function nativeWorldFlowSupportsHeadingsOutsideTheOldLocalChart(testCase)
            [~,~,cfg]=localFixture();x=[0;0;2.9;8;0;0];u=[0;.02];
            sample=fialaCertificate.sample(x,x,[x;u],zeros(2,6),zeros(6,1),u,cfg);
            testCase.assertTrue(sample.accepted);
            [~,states]=ode45(@(~,state)nonlinearBicycleModel.derivative(state,u,cfg), ...
                [0,cfg.controller.sampleTime],x,odeset('RelTol',1e-12,'AbsTol',1e-13));
            testCase.verifyGreaterThanOrEqual(states(end,:).',sample.endpoint(1:6,1));
            testCase.verifyLessThanOrEqual(states(end,:).',sample.endpoint(1:6,2));
        end
        function crossingBetweenClearNodesIsRejected(testCase)
            [~,~,cfg]=localFixture();cfg.controller.sampleTime=.1;
            backup=nonlinearSafetyCertificate.backup(cfg,[0;0;0;0;10;10]);
            x=[0;0;0;8;0;0];q=[.4;-10;pi/2;200;0;0;2.4;.95;0;0];
            certificate=nonlinearSafetyCertificate.plan(x,repmat(backup.reference.input,1,4),[0;0], ...
                q,[],[0;0;0;0;10;10],backup,cfg);
            testCase.verifyFalse(certificate.accepted);
            testCase.verifyTrue(any(certificate.reason==["sweptGeometry","rectangleOverlap"]));
        end
        function targetThatWillReturnCannotBeReleasedAtTheHorizon(testCase)
            [~,~,cfg]=localFixture();frame=[0;0;0;0;4;4];b=nonlinearSafetyCertificate.backup(cfg,frame);
            q=[0;10;pi;4;.2;4*sin(.2)/1.65;2.4;.95;0;0];x=[0;0;0;8;0;0];
            [accepted,~]=nonlinearSafetyCertificate.terminal([x,x],b.reference.input,q,0,b,frame,[2.4;.95;0;0],cfg);
            testCase.verifyFalse(accepted);
        end
        function inconsistentConstantSpeedAccelerationIsRejected(testCase)
            [ego,road,cfg]=localFixture();target=localObservation();target.targetYawRate=.2;
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:inconsistentTargetMotion');
        end
        function nonzeroUncertaintyIsNotReportedAsAnExactModelGuarantee(testCase)
            [ego,road,cfg]=localFixture();ego.controllerStateErrorBound=.01*ones(6,1);
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:nonexactStudyInput');
        end
        function noInputIsIssuedWithoutAnInitialSafeWitness(testCase)
            [ego,road,cfg]=localFixture();target=localObservation();target.targetPositionInertial=[0;0];
            cfg.controller.maximumHorizonSteps=16;
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end
        function solverFailureLeavesACertifiedContinuationAvailable(testCase)
            [ego,road,cfg]=localFixture();[~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,prior);cfg.nonlinear.maximumImprovementIterations=1;
            cfg.solver.certificateSearchTimeLimit=100;cfg.nonlinear.proposalFunction=@localFailure;
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyTrue(problem.certificate.accepted);
            testCase.verifyTrue(problem.metadata.carriedWitnessAvailable);
            testCase.verifyNotEmpty(problem.metadata.admissionSearch.proposalFailures);
        end
        function anUnsafeSolverProposalCannotReplaceTheIncumbent(testCase)
            [ego,road,cfg]=localFixture();cfg.nonlinear.maximumImprovementIterations=1;
            cfg.solver.certificateSearchTimeLimit=100;cfg.nonlinear.proposalFunction=@localUnsafe;
            [u,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyTrue(problem.certificate.accepted);
            testCase.verifyLessThan(abs(u.frontWheelSteeringAngle),.01);
            testCase.verifyGreaterThan(problem.metadata.admissionSearch.rejectedCandidates,0);
        end
        function changingTheRoadContractRequiresFreshAdmission(testCase)
            [ego,road,cfg]=localFixture();[~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,prior);road.lateralClearance=[5;5];
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyFalse(problem.metadata.carriedWitnessAvailable);
            testCase.verifyTrue(problem.certificate.accepted);
        end
        function targetDropoutKeepsItsImmutableFutureMotion(testCase)
            [ego,road,cfg]=localFixture();target=localObservation();
            [~,~,~,prior]=collisionAvoidanceController(ego,target,road,cfg,[]);
            ego=localSuccessor(ego,prior);
            [~,~,problem,state]=collisionAvoidanceController(ego,[],road,cfg,prior);
            testCase.verifyTrue(problem.metadata.hasTarget);
            testCase.verifyEqual(state.targetAnchor,prior.targetAnchor);
            testCase.verifyEqual(state.targetStep,1);
            testCase.verifyTrue(problem.certificate.accepted);
        end
        function aChangedTargetTrajectoryCannotReuseTheConstantModel(testCase)
            [ego,road,cfg]=localFixture();target=localObservation();
            [~,~,~,prior]=collisionAvoidanceController(ego,target,road,cfg,[]);
            ego=localSuccessor(ego,prior);target.targetPositionInertial(1)=101;
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,prior), ...
                'collisionAvoidanceController:inconsistentTargetObservation');
        end
        function convexStepsRemainSubjectToNonlinearValidation(testCase)
            [ego,road,cfg]=localFixture();cfg.nonlinear.maximumImprovementIterations=1;
            cfg.solver.certificateSearchTimeLimit=100;
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyTrue(problem.certificate.accepted);
            testCase.verifyGreaterThanOrEqual(problem.metadata.solverCallCount,2);
            testCase.verifyFalse(problem.metadata.convexSubproblemsAreCertifiedInnerApproximations);
            testCase.verifyTrue(problem.metadata.nonlinearAcceptanceRequired);
            step=problem.metadata.admissionSearch.sequentialIterations{1};
            testCase.verifyLessThanOrEqual(step.secondarySafety,step.safetyCap+1e-8);
        end
        function aTurningTargetHasIndependentSpeedAndHeadingRate(testCase)
            [ego,road,cfg]=localFixture();target=localObservation();
            target.targetYawRate=.05;
            target=rmfield(target,'targetAccelerationInertial');
            [~,~,problem,state]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyEqual(problem.model.target(4:6),[4;0;.05],'AbsTol',1e-12);
            testCase.verifyEqual(problem.model.jointState,[problem.model.initialState;100;20;0],'AbsTol',1e-12);
            testCase.verifyEqual(state.predictedJointState(9,2),.05*cfg.controller.sampleTime,'AbsTol',1e-12);
            testCase.verifyEqual(state.predictedJointState(7:8,2), ...
                [100+80*sin(.0025);20+80*(1-cos(.0025))],'AbsTol',1e-11);
            testCase.verifyTrue(problem.certificate.accepted);
        end
        function targetBlockOfTheJointDynamicsIsUnactuated(testCase)
            [~,~,cfg]=localFixture();z=[0;0;0;8;0;0;10;20;.3];
            parameters=[4;0;.1;2.4;.95;0;0];
            [next,a,b]=nonlinearBicycleModel.jointSample(z,[.01;.03],parameters,cfg);
            other=nonlinearBicycleModel.jointSample(z,[-.01;.01],parameters,cfg);
            testCase.verifyEqual(next(7:9),other(7:9),'AbsTol',1e-12);
            testCase.verifyEqual(b(7:9,:),zeros(3,2),'AbsTol',1e-12);
            testCase.verifyEqual(a(1:6,7:9),zeros(6,3),'AbsTol',1e-12);
            testCase.verifyEqual(next(9)-z(9),.1*cfg.controller.sampleTime,'AbsTol',1e-12);
        end
        function overlappingPolygonsProvideADualRestorationDirection(testCase)
            shape=[2.4;.95;.2;-.1];
            rows=nonlinearSafetyCertificate.dualLinearization([0;0;0],shape,[0;0;0],shape,[0;1]);
            h=[eye(2);-eye(2)];
            testCase.verifyEqual(norm(rows.normal),1,'AbsTol',1e-12);
            testCase.verifyEqual(rows.normal+h.'*rows.mu,zeros(2,1),'AbsTol',1e-12);
            testCase.verifyEqual(-rows.normal+h.'*rows.lambda,zeros(2,1),'AbsTol',1e-12);
            testCase.verifyGreaterThanOrEqual([rows.mu;rows.lambda],0);
            testCase.verifyLessThan(rows.signedDistance,0);
            testCase.verifyEqual(rows.normal,[0;1],'AbsTol',1e-12);
        end
        function intersectingLaneRolloutIsRestoredWithoutAvoidancePlan(testCase)
            [ego,road,cfg]=localFixture();target=localObservation();
            target.targetPositionInertial=[24;0];target.targetVelocityInertial=[-8;0];
            target.targetYawInertial=pi;cfg.controller.horizonSteps=8;
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyEmpty(cfg.nonlinear.initialPlan);
            testCase.verifyEqual(problem.metadata.certificateSource,"sequentialConvexification");
            testCase.verifyGreaterThan(max(abs(problem.predictedState(2,:))),1.9);
            testCase.verifyGreaterThanOrEqual(problem.certificate.minimumCollisionMargin,0);
            testCase.verifyLessThanOrEqual(problem.certificate.terminal.normUpper,problem.model.backup.radius);
            testCase.verifyEqual(problem.metadata.predictiveBarrierValue,0);
            steps=[problem.metadata.admissionSearch.sequentialIterations{:}];
            solved=steps([steps.status]=="solved");
            testCase.verifyLessThanOrEqual([solved.secondarySafety],[solved.safetyCap]+1e-7);
            testCase.verifyFalse(problem.metadata.drivingModeSwitching);
        end
        function feedbackContinuationBoundsFutureHeldInputs(testCase)
            [ego,road,cfg]=localFixture();ego.position(2)=.001;
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            sample=problem.certificate.samples{2};
            testCase.verifyTrue(sample.accepted);
            testCase.verifyGreaterThan(norm(problem.certificate.feedbackGains(:,:,2)),0);
            testCase.verifyLessThanOrEqual(sample.initialBox(7,2),cfg.model.frontWheelSteeringAngleMaximum);
            testCase.verifyGreaterThanOrEqual(sample.initialBox(7,1),-cfg.model.frontWheelSteeringAngleMaximum);
        end
        function aCertifiedControlBoxContainsPerturbedNonlinearFlows(testCase,perturbation)
            [ego,road,cfg]=localFixture();[~,inputs,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            policy=struct('referenceStates',problem.certificate.referenceStates, ...
                'feedbackGains',problem.certificate.feedbackGains,'inputRadius',1e-6*ones(size(inputs)));
            m=problem.model;
            region=nonlinearSafetyCertificate.plan(m.initialState,inputs,m.previousInput,[], ...
                m.lane,m.frame,m.backup,cfg,0,policy);
            testCase.assertTrue(region.accepted,region.reason);
            changed=inputs+policy.inputRadius.*perturbation;
            testCase.verifyTrue(nonlinearSafetyMex('inputBoxContains',changed,inputs,1.000001*policy.inputRadius));
            [~,states]=ode45(@(~,x)nonlinearBicycleModel.derivative(x,changed(:,1),cfg), ...
                [0,cfg.controller.sampleTime],m.initialState,odeset('RelTol',1e-12,'AbsTol',1e-13));
            endpoint=states(end,:).';bounds=region.samples{1}.endpoint(1:6,:);
            testCase.verifyGreaterThanOrEqual(endpoint,bounds(:,1));
            testCase.verifyLessThanOrEqual(endpoint,bounds(:,2));
            derivative=nonlinearSafetyMex('clfGradient',region.samples{1},fialaCertificate.parameters(cfg), ...
                m.frame,m.backup.reference.state,m.backup.reference.factor,[1;1]);
            anchorNext=nonlinearBicycleModel.sample(m.initialState,inputs(:,1),cfg);
            anchorError=nonlinearBicycleModel.error(anchorNext,m.lane,m.backup.reference);
            changedError=nonlinearBicycleModel.error(endpoint,m.lane,m.backup.reference);
            increment=changed(:,1)-inputs(:,1);
            upper=derivative(:,1).'*increment+derivative(:,2).'*abs(increment);
            actual=norm(m.backup.reference.factor*changedError)^2-norm(m.backup.reference.factor*anchorError)^2;
            testCase.verifyLessThanOrEqual(actual,upper+1e-12);
        end
        function certificateExhaustionDispatchesTheStoredPolicyAndBackup(testCase)
            [ego,road,cfg]=localFixture();[~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            cfg.nonlinear.maximumCertificateCells=1;
            outcomes=localReplayWithExhaustedVerifier(ego,road,cfg,prior,8);
            testCase.verifyTrue(all(outcomes.certified));
            testCase.verifyTrue(all(outcomes.inherited));
            testCase.verifyTrue(any(outcomes.backup));
        end
        function targetPredictionRequiresAnExplicitClock(testCase)
            [ego,road,cfg]=localFixture();ego=rmfield(ego,'stateTime');
            testCase.verifyError(@()collisionAvoidanceController(ego,localObservation(),road,cfg,[]), ...
                'collisionAvoidanceController:missingTargetClock');
        end
        function finiteSlewLimitsRequireAppliedInputMemory(testCase)
            [ego,road,cfg]=localFixture();cfg.model.frontWheelSteeringRateMaximum=1;
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:missingInputMemory');
        end
        function zeroDistanceIsNotASafetyClearance(testCase)
            [ego,road,cfg]=localFixture();cfg.collision.safetyMarginMeters=0;
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:positiveClearanceRequired');
        end
    end
end

function [ego,road,cfg]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'controller',struct('horizonSteps',4), ...
        'nonlinear',struct('maximumImprovementIterations',0)));
    ego=struct('position',[0;0],'yaw',0,'speed',8,'stateTime',0);
    road=struct('centerline',[-100,0;1000,0],'lateralClearance',[4;4]);
end
function q=localTarget(omega)
    speed=4;lr=1.65;beta=asin(omega*lr/speed);q=[5;3;.4;speed;beta;omega;2.4;.95;.2;-.1];
end
function states=localTargetSamples(q,times)
    states=zeros(10,numel(times));
    for j=1:numel(times),states(:,j)=nonlinearSafetyCertificate.targetFlow(q,times(j));end
end
function r=localRotation(yaw)
    r=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
end
function target=localObservation()
    target=struct('targetPositionInertial',[100;20],'targetVelocityInertial',[4;0], ...
        'targetYawInertial',0,'targetYawRate',0,'targetAccelerationInertial',[0;0]);
end
function ego=localSuccessor(ego,state)
    x=state.predictedState(:,2);ego.position=x(1:2);ego.yaw=x(3);ego.speed=x(4);
    ego.lateralVelocity=x(5);ego.yawRate=x(6);ego.stateTime=ego.stateTime+.05;ego.heldActuatorInput=state.appliedInput;
end
function plan=localFailure(~,~)
    plan=[];
    error('test:forcedFailure','Forced solver failure.');
end
function plan=localUnsafe(certificate,~)
    plan=certificate.inputs;plan(1,1)=1;
end
function outcomes=localReplayWithExhaustedVerifier(ego,road,cfg,prior,count)
    outcomes=struct('certified',false(1,count),'inherited',false(1,count),'backup',false(1,count));
    for index=1:count
        ego=localSuccessor(ego,prior);
        [~,~,problem,prior]=collisionAvoidanceController(ego,[],road,cfg,prior);
        outcomes.certified(index)=problem.certificate.accepted;
        outcomes.inherited(index)=problem.metadata.inheritedCertificate;
        outcomes.backup(index)=problem.metadata.terminalBackupDispatched;
    end
end
