classdef admissionSupportSearchTest < matlab.unittest.TestCase
    %admissionSupportSearchTest Overlap-aware search with independent hard acceptance.
    properties (TestParameter)
        geometry=struct('aligned',[12;0;0;0], ...
            'rotated',[8;5;.6;-.3],'corner',[5.2;2.1;0;0]);
        scenario = {"stationary","oncoming"};
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function supportDirectionAgreesWithRectangleGeometry(testCase,geometry)
            dimensions=[2.4;.95;2.4;.95];
            [normal,info]=avoidanceSafetyGeometry.supportDirection(geometry(1:2),geometry(3), ...
                [0;0],geometry(4),dimensions);
            [distance,geometricNormal]=avoidanceSafetyGeometry.rectangleDistance( ...
                geometry(1:2),geometry(3),[0;0],geometry(4),dimensions);
            testCase.verifyTrue(info.available);
            testCase.verifyEqual(info.signedDistance,distance,AbsTol=1e-12);
            testCase.verifyEqual(normal,geometricNormal,AbsTol=1e-12);
            testCase.verifyEqual(norm(normal),1,AbsTol=1e-12);
        end

        function overlapSuppliesDirectionsButDoesNotClaimSeparation(testCase)
            [normal,info]=avoidanceSafetyGeometry.supportDirection([0;0],0,[0;0],0,[2.4;.95;2.4;.95]);
            testCase.verifyTrue(info.available);
            testCase.verifyFalse(info.anchorSeparated);
            testCase.verifyEqual(norm(normal),1,AbsTol=1e-12);
            testCase.verifyLessThan(info.signedDistance,0);
            testCase.verifyGreaterThanOrEqual(size(info.alternatives,2),2);
        end

        function touchingFootprintsStillSupplyAUnitDirection(testCase)
            [normal,info]=avoidanceSafetyGeometry.supportDirection([4.8;0],0,[0;0],0,[2.4;.95;2.4;.95]);
            testCase.verifyTrue(info.available);
            testCase.verifyEqual(info.signedDistance,0,AbsTol=1e-12);
            testCase.verifyEqual(norm(normal),1,AbsTol=1e-12);
        end

        function aClearAnchorUsesOneHardTrajectorySolve(testCase)
            [ego,target,cfg,road]=localFixture();
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(problem.metadata.supportGeometry.used);
            testCase.verifyEqual(problem.metadata.trajectorySolverCallCount,1);
            testCase.verifyEqual(problem.metadata.solverCallCount,1+problem.metadata.restorationSolverCallCount);
            testCase.verifyEqual(problem.metadata.restorationSolverCallCount,0);
            testCase.verifyTrue(problem.metadata.postSolveCertificationPerformed);
            testCase.verifyEqual(problem.program.physicalMatrix(:,end),zeros(numel(problem.program.physicalBound),1),AbsTol=0);
        end

        function updatingDirectionsKeepsTheCertifiedSuffixAndExitDeadline(testCase)
            [ego,target,cfg,road]=localFixture();
            [~,~,first,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
            x=stored.predictedState(:,2);
            [ego.position,ego.yaw]=laneGeometry.fromFrenet(x,first.model.lane);
            ego.speed=x(4);ego.lateralVelocity=x(5);ego.yawRate=x(6);
            ego.stateTime=.1;ego.perception.time=.1;ego.heldActuatorInput=stored.appliedInput;
            target.targetPositionInertial=target.targetPositionInertial+.1*target.targetVelocityInertial;
            [~,~,next]=collisionAvoidanceController(ego,target,road,cfg,stored);
            witness=stored.plan(:,2:end);witness=witness(:);
            testCase.verifyTrue(next.metadata.supportGeometry.used);
            testCase.verifyTrue(next.metadata.supportGeometry.witnessPreserved);
            testCase.verifyLessThanOrEqual(max(next.program.physicalMatrix*[witness;0]-next.program.physicalBound),0);
            testCase.verifyEqual(next.program.completion.deadline,first.program.completion.deadline,AbsTol=0);
            testCase.verifyTrue(next.metadata.postSolveCertificationPerformed);
        end

        function shiftedWholeHoldTimesStayNonnegativeAcrossAnEncounter(testCase)
            [minimumTime,updates]=localLongEncounter();
            testCase.verifyGreaterThanOrEqual(minimumTime,0);
            testCase.verifyGreaterThan(updates,0);
        end

        function overlappingCruiseAdmissionFindsAHardCertifiedPlan(testCase,scenario)
            [ego,target,cfg,road]=localFixture();target=localRejectedTarget(target,scenario);
            [command,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyGreaterThan(problem.metadata.restorationSolverCallCount,0);
            testCase.verifyGreaterThan(problem.metadata.admissionSearch.normalizedDeficits(1),0);
            testCase.verifyLessThanOrEqual(max(problem.program.physicalMatrix*problem.decision-problem.program.physicalBound),0);
            testCase.verifyEqual(command.actuatorInput,problem.inputPlan(:,1),AbsTol=0);
            testCase.verifyFalse(problem.metadata.fallbackUsed);
        end

        function sparseRealizationPreservesAllRowsAndTheObjective(testCase)
            [ego,target,cfg,road]=localFixture();target=localRejectedTarget(target,"stationary");
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            [rowError,costError]=localLiftErrors(problem.program);
            testCase.verifyLessThan(rowError,1e-8);
            testCase.verifyLessThan(costError,1e-7);
        end

        function restorationWithoutAHardSolutionCannotIssueACommand(testCase)
            [ego,target,cfg,road]=localFixture();target=localRejectedTarget(target,"stationary");
            cfg.solver.jointFunction=@localRejectHard;
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end

        function anInitiallyCollidingEncounterCannotBeCertified(testCase)
            [ego,target,cfg,road]=localFixture();target=localRejectedTarget(target,"stationary");
            target.targetPositionInertial=[0;0];
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end

        function targetFreeUncertaintySelectsANonemptyCertifiedHorizon(testCase)
            [ego,~,cfg,road]=localFixture();
            ego.controllerStateErrorBound=[.04;.04;.0583;.0637;.4683;.0015];
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyLessThan(problem.metadata.horizonSteps,cfg.controller.horizonSteps);
            testCase.verifyGreaterThanOrEqual(problem.metadata.horizonSteps,cfg.controller.minimumHorizonSteps);
            testCase.verifyEqual(problem.metadata.trajectorySolverCallCount,1);
        end

        function enlargedMeasurementBoundsRequireANewCertificate(testCase)
            [ego,~,cfg,road]=localFixture();
            ego.controllerStateErrorBound=.0001*ones(6,1);
            [~,~,first,stored]=collisionAvoidanceController(ego,[],road,cfg,[]);
            x=stored.predictedState(:,2);
            [ego.position,ego.yaw]=laneGeometry.fromFrenet(x,first.model.lane);
            ego.speed=x(4);ego.lateralVelocity=x(5);ego.yawRate=x(6);
            ego.stateTime=.1;ego.perception.time=.1;ego.heldActuatorInput=stored.appliedInput;
            ego.controllerStateErrorBound=.0002*ones(6,1);
            [~,~,next]=collisionAvoidanceController(ego,[],road,cfg,stored);
            testCase.verifyTrue(next.metadata.measurementContractChanged);
            testCase.verifyFalse(next.metadata.inheritedFeasibleFamily);
            testCase.verifyTrue(next.metadata.planCertified);
            testCase.verifyGreaterThanOrEqual(next.metadata.measurementRadiusLimit,ego.controllerStateErrorBound);
        end

        function aPreviousFormatCertificateIsNotImported(testCase)
            [ego,target,cfg,road]=localFixture();
            [~,~,~,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
            stored.version=30;
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,stored), ...
                'collisionAvoidanceController:invalidControllerState');
        end
    end
end

function [minimumTime,updates]=localLongEncounter()
    [ego,target,cfg,road]=localFixture();
    target.targetPositionInertial=[0;8];target.targetVelocityInertial=[12;0];
    target.targetHeadingInertial=0;stored=[];minimumTime=Inf;updates=0;
    for index=1:22
        [~,~,problem,stored]=collisionAvoidanceController(ego,target,road,cfg,stored);
        minimumTime=min(minimumTime,min([problem.prediction.cells.start]));
        updates=updates+double(problem.metadata.supportGeometry.used);
        x=stored.predictedState(:,2);
        [ego.position,ego.yaw]=laneGeometry.fromFrenet(x,problem.model.lane);
        ego.speed=x(4);ego.lateralVelocity=x(5);ego.yawRate=x(6);
        ego.stateTime=index*.1;ego.perception.time=ego.stateTime;ego.heldActuatorInput=stored.appliedInput;
        target.targetPositionInertial=[0;8]+ego.stateTime*target.targetVelocityInertial;
    end
end

function [ego,target,cfg,road]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('sampleTime',.1),'solver',struct('frameDeadlineSeconds',30)));
    ego=struct('position',[0;0],'yaw',0,'speed',8,'stateTime',0, ...
        'perception',struct('time',0,'range',16,'completeWithinRange',true));
    target=struct('trackId',1,'targetPositionInertial',[12;4], ...
        'targetVelocityInertial',[20;0],'targetAccelerationInertial',[0;0], ...
        'targetHeadingInertial',0,'targetYawRate',0, ...
        'predictionMotion',struct('kind',"finite-sensing-motion-v1", ...
        'jerkBound',[0;0],'yawAccelerationBound',0));
    road=[-100,0;2000,0];
end

function target=localRejectedTarget(target,scenario)
    if scenario=="stationary"
        target.targetPositionInertial=[15;0];target.targetVelocityInertial=[0;0];
    else
        target.targetPositionInertial=[18.4;0];target.targetVelocityInertial=[-8;0];
        target.targetHeadingInertial=pi;
    end
end

function result=localRejectHard(~,program)
    if isfield(program,'restoration') && program.restoration
        result=program.defaultSolver();
    else
        result=struct('decision',zeros(numel(program.q),1),'exitFlag',-2,'output',struct());
    end
end

function [rowError,costError]=localLiftErrors(program)
    lifted=avoidanceStageQp.build(program);n=program.layout.planCount;
    trial=program.anchorPlan+.01*sin((1:n).');decisions={[program.anchorPlan;1],[trial;2]};
    values=zeros(2,2);rowError=0;
    for index=1:2
        decision=decisions{index};prediction=program.prediction;
        states=prediction.egoStateOffset+reshape(pagemtimes(prediction.egoStateMatrix,decision(1:n)),6,[]);
        extra=states(:,2:end)-lifted.stateCenter(:,2:end);
        augmented=[decision;extra(:)];residual=lifted.b-lifted.A*augmented;
        rowError=max(rowError,max(abs([residual(1:lifted.cones(1)); ...
            residual(lifted.cones(1)+1:end)-(program.b(lifted.retainedRows)-program.A(lifted.retainedRows,:)*decision)])));
        values(index,:)=[.5*decision.'*program.P*decision+program.q.'*decision, ...
            .5*augmented.'*lifted.P*augmented+lifted.q.'*augmented];
    end
    costError=abs(diff(values(:,1))-diff(values(:,2)));
end
