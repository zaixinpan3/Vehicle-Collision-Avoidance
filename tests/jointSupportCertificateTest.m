classdef jointSupportCertificateTest < matlab.unittest.TestCase
    %jointSupportCertificateTest Joint certificates and inherited performance solves.
    properties (TestParameter)
        yawRadius=struct('fixed',0,'interval',.12,'full',pi);
        nonpositiveGap=struct('contact',0,'overlap',-.01);
        reservedGap=struct('certified',.002,'violated',-.002);
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (Test)
        function aSmallPositiveBodyGapPassesTheHardVerifier(testCase)
            program=localGapProgram(.01);
            accepted=solveHardCbfClf.certify(program,[.1;0;1]);
            residual=avoidanceSafetyGeometry.jointResidual(accepted,[.1;0;1],0);
            testCase.verifyEqual(residual,-.01,AbsTol=1e-12);
        end

        function contactAndOverlapCannotPassTheHardVerifier(testCase,nonpositiveGap)
            program=localGapProgram(nonpositiveGap);
            testCase.verifyError(@()solveHardCbfClf.certify(program,[.1;0;1]), ...
                'collisionAvoidanceController:optimizationFailed');
        end

        function uncertainOverlapStillRejectsANominallyPositiveGap(testCase)
            program=localGapProgram(.01);
            program.jointCertificate.records.positionBall=.02;
            testCase.verifyError(@()solveHardCbfClf.certify(program,[.1;0;1]), ...
                'collisionAvoidanceController:optimizationFailed');
        end

        function aRemovedCertificateSelectorIsRejected(testCase)
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct( ...
                'controller',struct('certificateMethod',"jointSupport"))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end

        function supportIsHomogeneousIncludingZero(testCase,yawRadius)
            value=avoidanceSafetyGeometry.supportValue([2.4;.95],yawRadius,[.3;-.7]);
            scaled=avoidanceSafetyGeometry.supportValue([2.4;.95],yawRadius,3*[.3;-.7]);
            zero=avoidanceSafetyGeometry.supportValue([2.4;.95],yawRadius,zeros(2,1));
            testCase.verifyEqual(scaled,3*value,AbsTol=1e-12);
            testCase.verifyEqual(zero,0,AbsTol=0);
        end

        function intervalYawConicSupportMatchesAnalyticMaximum(testCase,yawRadius)
            [actual,expected,solved]=localSupportComparison(yawRadius);
            testCase.verifyTrue(solved);
            testCase.verifyEqual(actual,expected,AbsTol=2e-6);
        end

        function fixedPlanConesAgreeWithIndependentSupport(testCase,yawRadius,reservedGap)
            [program,cfg]=localSquare();point=[.2;0;1];
            records=repmat(localRecord(),2,1);angles=[.4;-.7];
            state=program.prediction.egoStateOffset(:,2)+program.prediction.egoStateMatrix(:,:,2)*point(1:2);
            for index=1:2
                item=records(index);item.egoHalfSize=[2.4;.95];item.targetHalfSize=[2.2;.9];
                item.egoYawRadius=yawRadius;item.targetYawRadius=yawRadius/2;
                item.targetYaw=-.3*index;item.positionBall=.02;
                item.generators=[.2,0,.1;0,.3,.05];
                item.positionMap=[1,.2,.1,.03,.02,.01;-.1,1,.2,.01,.03,.02];
                item.yawRow=[.02,-.01,1,.03,.04,-.02];
                gap=.2;if index==2,gap=reservedGap;end
                residual=avoidanceSafetyGeometry.jointValue(item,state,angles(index));
                item.positionOffset=[cos(angles(index));sin(angles(index))]*(residual+1e-4+gap);
                records(index)=item;
            end
            program.jointCertificate=struct('records',records,'angles',angles,'upperBound',-1e-4*ones(2,1));
            conic=avoidanceStageQp.fixedDirections(program,point,angles,cfg);
            % Fix the complete physical plan; directions are already constant.
            % Only the support epigraph variables remain free to certify it.
            indices=1:numel(point);number=numel(indices);
            lock=sparse(1:number,indices,ones(1,number),number,numel(conic.q));
            conic.A=[lock;conic.A];conic.b=[point;conic.b];
            conic.cones(1)=conic.cones(1)+number;
            conic.P=sparse(numel(conic.q),numel(conic.q));conic.q(:)=0;
            result=solveHardCbfClf.constrained(conic,cfg);
            testCase.verifyEqual(result.feasible,reservedGap>0);
        end

        function majorantsTouchAndBoundRobustSeparation(testCase)
            [minimumGap,touchingError]=localCheckMajorants();
            testCase.verifyGreaterThanOrEqual(minimumGap,-1e-11);
            testCase.verifyLessThan(touchingError,1e-11);
        end

        function oncomingAdmissionFixesDirectionsAndRetainsItsOptimizedSuffix(testCase)
            [first,next,old,stored]=localOncoming(false);
            keep=[old.program.jointCertificate.records.stage]>1;
            oldAngles=old.program.jointCertificate.angles(keep);
            suffix=old.plan(:,2:end);
            values=avoidanceSafetyGeometry.jointResidual(next.program, ...
                suffix(:),oldAngles);
            testCase.verifyTrue(first.metadata.planCertified);
            testCase.verifyEqual(first.metadata.restorationSolverCallCount,0);
            testCase.verifyEqual(first.metadata.solverCallCount,1);
            testCase.verifyTrue(next.metadata.shiftedWitnessContained);
            testCase.verifyEqual(next.metadata.admissionSearch.initialCertificateAngles,oldAngles,AbsTol=0);
            testCase.verifyEqual(next.metadata.conicSolverCallCount,1);
            testCase.verifyLessThanOrEqual(max(values),0);
            testCase.verifyEqual(stored.completion.deadline,old.completion.deadline,AbsTol=0);
            testCase.verifyLessThanOrEqual(first.metadata.solverCallCount,3);
        end

        function obsoleteRestartAndStepSettingsAreRejected(testCase)
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct( ...
                'jointCertificate',struct('maximumStarts',2))), ...
                'collisionAvoidanceController:invalidConfiguration');
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct( ...
                'jointCertificate',struct('positionStep',4))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end

        function compactRoundoffEnclosuresContainTheOriginalTargetUncertainty(testCase)
            [radius,minimumGap]=localCompactUncertainty();
            testCase.verifyEqual(radius,0,AbsTol=0);
            testCase.verifyGreaterThanOrEqual(minimumGap,-1e-14);
        end

        function aDomainWideSeparationProofRetainsTheOriginalSafetyRecord(testCase)
            [program,cfg]=localSquare();
            program.jointCertificate.records.positionOffset=[10;0];
            program.feasibleWitness=[.1;0;1];program.anchorPlan=[.1;0];
            program.geometry.frames.domainCenter=[.9;0;0];
            program.geometry.frames.domainRadius=[0;2;0];
            conic=avoidanceStageQp.fixedDirections(program,program.feasibleWitness,0,cfg);
            [accepted,result]=solveHardCbfClf.fixedDirections(program,struct(),cfg);
            testCase.verifyEqual(conic.domainCertifiedRecords,1);
            testCase.verifyTrue(result.feasible);
            testCase.verifyNumElements(accepted.jointCertificate.records,1);
            accepted.jointCertificate.angles=pi;
            testCase.verifyError(@()solveHardCbfClf.certify(accepted,result.decision), ...
                'collisionAvoidanceController:optimizationFailed');
        end

        function nominalClearanceDoesNotProveSeparationThroughoutTheDomain(testCase)
            [program,cfg]=localSquare();
            program.geometry.frames.domainCenter=[.9;1;0];
            program.geometry.frames.domainRadius=[0;1;0];
            point=[1.5;0;1];program.feasibleWitness=point;
            program.jointCertificate.angles=pi/2;
            conic=avoidanceStageQp.fixedDirections(program,point,pi/2,cfg);
            [accepted,result]=solveHardCbfClf.fixedDirections(program,struct(),cfg);
            testCase.verifyEmpty(conic.domainCertifiedRecords);
            testCase.verifyTrue(result.feasible);
            testCase.verifyGreaterThan(result.decision(1),1.1);
            testCase.verifyLessThanOrEqual(max(avoidanceSafetyGeometry.jointResidual( ...
                accepted,result.decision,accepted.jointCertificate.angles)),0);
        end

        function interruptedImprovementReturnsOnlyTheVerifiedIncumbent(testCase)
            [~,next,old,stored]=localOncoming(true);
            testCase.verifyTrue(next.metadata.certifiedIncumbentUsed);
            testCase.verifyEqual(next.metadata.certificateSource,"retainedCertifiedIncumbent");
            testCase.verifyEqual(next.metadata.solverExitFlag,0);
            testCase.verifyEqual(stored.plan,old.plan(:,2:end),AbsTol=0);
            testCase.verifyLessThanOrEqual(max(next.metadata.jointCertificateResidual),0);
        end

        function boundedPoseAndYawUseHardSupportCertificates(testCase)
            [ego,target,road,cfg]=localControllerFixture();
            ego.controllerStateErrorBound=1e-4*ones(6,1);
            target.targetYawErrorBound=.12;target.targetPositionInertialErrorBound=[.02;.03];
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyLessThanOrEqual(max(problem.metadata.jointCertificateResidual),0);
            testCase.verifyEqual(problem.program.jointCertificate.records(1).targetYawRadius,.12,AbsTol=1e-10);
        end

        function conditioningTwoUncertainTargetsPreservesTheirCertificates(testCase)
            [first,next,oldAngles]=localUncertainTargets();
            testCase.verifyTrue(first.metadata.planCertified);
            testCase.verifyEqual(unique(string({next.program.jointCertificate.records.key})), ...
                sort(first.program.completion.keys));
            testCase.verifyEqual(next.metadata.admissionSearch.initialCertificateAngles,oldAngles,AbsTol=0);
            testCase.verifyTrue(next.metadata.shiftedWitnessContained);
            testCase.verifyLessThanOrEqual(max(next.metadata.jointCertificateResidual),0);
        end

        function scheduledDynamicsAndFiniteSlewKeepTheShiftedCertificate(testCase)
            [first,next,stored]=localScheduledPair();
            testCase.verifyTrue(next.metadata.shiftedWitnessContained);
            testCase.verifyEqual(next.metadata.referencePhaseIndex,first.metadata.referencePhaseIndex+1);
            testCase.verifyEqual(next.metadata.recursiveFeasibilityScope,"shiftedCompleteCertificateUnderUnchangedContracts");
            testCase.verifyLessThanOrEqual(abs(next.inputPlan(:,1)-stored.appliedInput),[.05;.1]+1e-10);
            testCase.verifyLessThanOrEqual(max(next.metadata.jointCertificateResidual),0);
        end

        function aFrameDeadlineStillRejectsACertifiedIncumbent(testCase)
            [ego,target,road,cfg]=localControllerFixture();cfg.solver.frameDeadlineSeconds=1e-9;
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end
    end
end

function program=localGapProgram(gap)
    program=localSquare();
    program.prediction.egoStateOffset(1,:)=1+gap;
    program.jointCertificate.records.clearance=0;
end

function [actual,expected,solved]=localSupportComparison(yawRadius)
    cfg=collisionAvoidanceControllerConfig();halfSize=[2.4;.95];vector=[.3;-.7];
    [d,b,r]=avoidanceSafetyGeometry.yawHull(halfSize,yawRadius);m=numel(b);
    program=struct('P',sparse(m+1,m+1),'q',[1;zeros(m,1)], ...
        'A',sparse([zeros(m,1),-eye(m);-1,b.';zeros(2,1),r*d.']), ...
        'b',[zeros(m+1,1);r*vector],'cones',[0;m;3]);
    result=solveHardCbfClf.constrained(program,cfg);
    actual=result.decision(1);expected=avoidanceSafetyGeometry.supportValue(halfSize,yawRadius,vector);
    solved=result.feasible;
end

function [minimumGap,touchingError]=localCheckMajorants()
    stream=RandStream('mt19937ar',Seed=20260919);record=localRecord();
    record.egoHalfSize=[2.4;.95];record.egoYawRadius=.13;record.targetHalfSize=[2.2;.9];
    record.targetYawRadius=.21;record.targetYaw=-.4;record.generators=[.2,0,.1;0,.3,.05];
    record.positionBall=.02;record.yawRow=[0,0,1,0,0,0];
    gaps=zeros(2000,1);errors=zeros(2000,1);
    for index=1:numel(gaps)
        state=randn(stream,6,1);angle=6*randn(stream);delta=10*randn(stream,6,1);
        actual=avoidanceSafetyGeometry.jointValue(record,state+delta,angle);
        majorant=avoidanceSafetyGeometry.fixedDirectionMajorant(record,state,angle,state+delta);
        gaps(index)=majorant-actual;
        errors(index)=avoidanceSafetyGeometry.fixedDirectionMajorant(record,state,angle,state) ...
            -avoidanceSafetyGeometry.jointValue(record,state,angle);
    end
    minimumGap=min(gaps);touchingError=max(abs(errors));
end

function record=localRecord()
    record=struct('key',"toy",'stage',1,'isExit',false,'positionMap',[eye(2),zeros(2,4)], ...
        'positionOffset',zeros(2,1),'yawRow',zeros(1,6),'yawOffset',0, ...
        'egoHalfSize',zeros(2,1),'egoYawRadius',0,'targetHalfSize',[1;1], ...
        'targetYaw',0,'targetYawRadius',0,'generators',zeros(2,0),'positionBall',0,'clearance',.1);
end

function [radius,minimumGap]=localCompactUncertainty()
    [ego,target,road,cfg]=localControllerFixture();
    target.targetYawErrorBound=1e-10;
    target.targetPositionInertialErrorBound=[1e-10;2e-10];
    [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
    record=problem.program.jointCertificate.records(1);radius=record.targetYawRadius;
    angles=linspace(-pi,pi,101);gaps=zeros(size(angles));
    for index=1:numel(angles)
        normal=[cos(angles(index));sin(angles(index))];
        argument=[cos(angles(index)-record.targetYaw);sin(angles(index)-record.targetYaw)];
        original=avoidanceSafetyGeometry.supportValue(record.targetHalfSize,1e-10,argument) ...
            +abs(normal).'*target.targetPositionInertialErrorBound;
        compact=avoidanceSafetyGeometry.supportValue(record.targetHalfSize,record.targetYawRadius,argument) ...
            +record.positionBall+sum(abs(record.generators.'*normal));
        gaps(index)=compact-original;
    end
    minimumGap=min(gaps);
end

function [program,cfg]=localSquare()
    cfg=collisionAvoidanceControllerConfig();
    offset=repmat([.9;0;0;1;0;0],1,2);map=zeros(6,2,2);map(2,1,2)=1;
    b=zeros(6,2);b(2,1)=1;
    prediction=struct('stageCount',1,'egoStateOffset',offset,'egoStateMatrix',map, ...
        'stageMatrixA',eye(6),'stageMatrixB',b,'stageAffine',zeros(6,1),'cells',struct('stage',1));
    physical=[1,0,0;-1,0,0;0,1,0;0,-1,0];bound=[2;0;1;1];
    geometry=struct('label',strings(0,1),'local',struct('stage',{}),'physicalBound',zeros(0,1), ...
        'normals',{{[1;0]}},'cellData',struct('normals',[1;0]),'frames',struct('heading',0));
    program=struct('P',speye(3),'q',zeros(3,1),'A',sparse([physical;0,0,-1;zeros(6,3)]), ...
        'b',[bound;0;100;zeros(5,1)],'cones',[0;5;6],'decisionRadius',[2;1], ...
        'anchorPlan',zeros(2,1),'feasibleWitness',[0;0;1],'physicalMatrix',physical, ...
        'physicalBound',bound,'safetyBound',bound,'physicalLabels',repmat("actuator",4,1), ...
        'layout',struct('planIndex',1:2,'planCount',2,'decisionCount',3,'relaxationIndex',3), ...
        'terminalCone',struct('matrix',zeros(0,2),'bound',zeros(0,1)), ...
        'terminalConePhysicalBound',zeros(0,1),'terminal',struct('modalMatrix',zeros(0,1),'stateIndex',4), ...
        'terminalOptimization',false,'clfNumericalReserve',1e-6,'prediction',prediction,'geometry',geometry, ...
        'referenceMatrices',repmat(eye(5),1,1,2),'referenceStates',offset,'referenceInputs',zeros(2,1), ...
        'inputWeight',ones(2,1),'slackWeight',1,'inheritedPredictionFamily',false, ...
        'supportGeometry',struct('overlappingNodes',1),'completion',struct('keys',"toy"), ...
        'jointCertificate',struct('records',localRecord(),'angles',0,'upperBound',-1e-4));
end

function [ego,target,road,cfg]=localControllerFixture()
    [ego,target,road,cfg]=encounterTestFixture.crossing();
    cfg.solver.frameDeadlineSeconds=60;cfg.solver.certificateSearchTimeLimit=60;
end

function [first,next,old,stored]=localOncoming(interrupt)
    [ego,target,road,cfg]=localControllerFixture();target.targetPositionInertial=[18.4;0];
    target.targetVelocityInertial=[-8;0];target.targetHeadingInertial=pi;
    [~,~,first,old]=collisionAvoidanceController(ego,target,road,cfg,[]);
    ego=encounterTestFixture.nextEgo(old,first.model.lane);
    target.targetPositionInertial=target.targetPositionInertial+.1*target.targetVelocityInertial;
    if interrupt,cfg.solver.jointFunction=@localTimeout;end
    [~,~,next,stored]=collisionAvoidanceController(ego,target,road,cfg,old);
end

function result=localTimeout(~,program)
    result=struct('decision',100*ones(numel(program.q),1),'exitFlag',0, ...
        'output',struct('message',"Simulated optimizer timeout with an unsafe iterate."));
end

function [first,next,oldAngles]=localUncertainTargets()
    [ego,target,road,cfg]=localControllerFixture();
    ego.controllerStateErrorBound=1e-4*ones(6,1);
    target.targetYawErrorBound=.12;target.targetPositionInertialErrorBound=[.02;.03];
    second=target;second.trackId=2;second.targetPositionInertial=[13;-4];target=[target,second];
    [~,~,first,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
    ego=encounterTestFixture.nextEgo(stored,first.model.lane);ego.controllerStateErrorBound=5e-5*ones(6,1);
    for index=1:numel(target)
        target(index).targetPositionInertial=target(index).targetPositionInertial+.1*target(index).targetVelocityInertial;
        target(index).targetPositionInertialErrorBound=[.01;.02];target(index).targetYawErrorBound=.1;
    end
    [~,~,next]=collisionAvoidanceController(ego,target,road,cfg,stored);
    oldAngles=stored.program.jointCertificate.angles([stored.program.jointCertificate.records.stage]>1);
end

function [first,next,stored]=localScheduledPair()
    [ego,target,road,cfg]=localControllerFixture();
    station=(0:2:40).';fraction=station/40;
    curvature=.01*(10*fraction.^3-15*fraction.^4+6*fraction.^5);
    curve=struct('origin',[0;0],'heading',0,'curvature',0,'length',40, ...
        'curvatureProfile',[station,curvature],'continuation',"constantCurvature");
    road=struct('centerline',road,'referenceCurve',curve);
    cfg.model.frontWheelSteeringRateMaximum=.5;cfg.model.brakingRatioRateMaximum=1;
    [~,~,first,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
    ego=encounterTestFixture.nextEgo(stored,first.model.lane);
    target.targetPositionInertial=target.targetPositionInertial+.1*target.targetVelocityInertial;
    [~,~,next]=collisionAvoidanceController(ego,target,road,cfg,stored);
end
