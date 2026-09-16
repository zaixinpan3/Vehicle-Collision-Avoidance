classdef distanceDualConvexificationTest < matlab.unittest.TestCase
    %distanceDualConvexificationTest Li-style direction proposals with hard safety.
    properties (TestParameter)
        geometry=struct('aligned',[12;0;0;0], ...
            'rotated',[8;5;.6;-.3],'corner',[5.2;2.1;0;0]);
        rejectedScenario = {"stationary","oncoming"};
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
        function distanceDualAgreesWithIndependentRectangleGeometry(testCase,geometry)
            cfg=collisionAvoidanceControllerConfig();dimensions=[2.4;.95;2.4;.95];
            [normal,dual]=avoidanceSafetyGeometry.distanceDual(geometry(1:2),geometry(3), ...
                [0;0],geometry(4),dimensions,cfg);
            [distance,geometricNormal]=avoidanceSafetyGeometry.rectangleDistance( ...
                geometry(1:2),geometry(3),[0;0],geometry(4),dimensions);
            testCase.verifyTrue(dual.available);
            testCase.verifyEqual(dual.distance,distance,AbsTol=2e-6);
            testCase.verifyEqual(normal,geometricNormal,AbsTol=2e-4);
            testCase.verifyGreaterThanOrEqual(min(dual.multipliers),-2e-7);
            testCase.verifyLessThanOrEqual(norm(dual.matrix.'*dual.multipliers),1+2e-7);
        end

        function overlappingFootprintsDoNotSupplyAnArtificialDirection(testCase)
            cfg=collisionAvoidanceControllerConfig();
            [normal,dual]=avoidanceSafetyGeometry.distanceDual([0;0],0,[0;0],0,[2.4;.95;2.4;.95],cfg);
            testCase.verifyFalse(dual.available);
            testCase.verifyEqual(normal,zeros(2,1),AbsTol=0);
            testCase.verifyEqual(dual.distance,0,AbsTol=0);
            testCase.verifyLessThan(dual.signedDistance,0);
        end

        function aClearAnchorUsesOneHardTrajectorySolve(testCase)
            [ego,target,cfg,road]=localFixture();
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(problem.metadata.dualConvexification.used);
            testCase.verifyEqual(problem.metadata.trajectorySolverCallCount,1);
            testCase.verifyEqual(problem.metadata.solverCallCount,1+problem.metadata.distanceSolverCallCount);
            testCase.verifyGreaterThan(problem.metadata.dualConvexification.distanceSolverCalls,0);
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
            testCase.verifyTrue(next.metadata.dualConvexification.used);
            testCase.verifyTrue(next.metadata.dualConvexification.witnessPreserved);
            testCase.verifyLessThanOrEqual(max(next.program.physicalMatrix*[witness;0]-next.program.physicalBound),0);
            testCase.verifyEqual(next.program.completion.deadline,first.program.completion.deadline,AbsTol=0);
            testCase.verifyTrue(next.metadata.postSolveCertificationPerformed);
        end

        function shiftedWholeHoldTimesStayNonnegativeAcrossAnEncounter(testCase)
            [minimumTime,updates]=localLongEncounter();
            testCase.verifyGreaterThanOrEqual(minimumTime,0);
            testCase.verifyGreaterThan(updates,0);
        end

        function overlappingAdmissionStopsBeforeAnyTrajectorySolve(testCase,rejectedScenario)
            [ego,target,cfg,road]=localFixture();
            target=localRejectedTarget(target,rejectedScenario);
            localCountedSolve('reset',[]);cfg.solver.jointFunction=@localCountedSolve;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
            testCase.verifyEqual(localCountedSolve('count',[]),0);
        end

        function unavailableInitializationIsRecordedAsAnExperimentFailure(testCase,rejectedScenario)
            folder=string(tempname);mkdir(folder);testCase.addTeardown(@()rmdir(folder,'s'));
            testCase.verifyError(@()runExactStateRecursiveFeasibilityScenario(Scenario=rejectedScenario, ...
                SampleCount=60,DeadlineSeconds=30,OutputDirectory=folder), ...
                'collisionAvoidanceController:optimizationFailed');
            saved=load(fullfile(folder,rejectedScenario+'-exact-state.mat'));
            testCase.verifyFalse(saved.report.completed);
            testCase.verifyFalse(saved.report.passed);
            testCase.verifyTrue(contains(saved.report.failureMessage,'Distance-dual initialization is unavailable'));
            testCase.verifyFalse(any(saved.report.terminalCommands));
        end

        function anInfeasibleDualDirectedTrajectoryStopsWithoutAnotherMethod(testCase)
            [ego,target,cfg,road]=localFixture();
            target.targetPositionInertial=[4.6;-4];target.targetVelocityInertial=[0;32];
            target.targetHeadingInertial=pi/2;
            localCountedSolve('reset',[]);cfg.solver.jointFunction=@localCountedSolve;
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
            testCase.verifyGreaterThan(localCountedSolve('count',[]),0);
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
        updates=updates+double(problem.metadata.dualConvexification.used);
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

function result=localCountedSolve(mode,program)
    persistent count
    if string(mode)=="reset",count=0;result=[];return;end
    if string(mode)=="count",result=count;return;end
    count=count+1;result=program.defaultSolver();
end
