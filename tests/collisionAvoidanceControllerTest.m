classdef collisionAvoidanceControllerTest < matlab.unittest.TestCase
    properties (TestParameter)
        targetPresent = {false,true};
        failedStatus = {-999,-2,0,2};
        badDecision = {[],[NaN;0],[Inf;0],[1i;0],zeros(3,1)};
        legacyPolicy = {"auto","backup","predictive","singleSolve"};
        invalidDecay = {0,1,1.01,NaN,Inf};
        errorRadius = {zeros(6,1),[.001;.001;.0001;.001;.001;.0001]};
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
        function decayMustLeaveAFiniteUncertaintyBudget(testCase,invalidDecay)
            testCase.verifyError(@() collisionAvoidanceControllerConfig(struct('clf', ...
                struct('decreaseRateFraction',invalidDecay))),'collisionAvoidanceController:invalidConfiguration');
        end
        function everySampleMakesExactlyOneSolverCall(testCase,targetPresent)
            [ego,target,road,cfg]=localFixture(targetPresent);
            localHook('reset',[]);cfg.solver.jointFunction=@localHook;
            [command,plan,problem,state]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyEqual(localHook('count',[]),1);
            testCase.verifySize(plan,[2,1]);
            testCase.verifyEqual(command.actuatorInput,plan,AbsTol=0);
            testCase.verifyEqual(state.appliedInput,plan,AbsTol=0);
            testCase.verifyEqual(problem.metadata.solverCallCount,1);
            testCase.verifyFalse(problem.metadata.postSolveCertificationPerformed);
            testCase.verifyFalse(problem.metadata.recursiveFeasibilityGuaranteed);
            testCase.verifyEqual(fieldnames(state),{'version';'appliedInput';'stateTime'});
        end
        function targetsOnlyAddObstacleConstraints(testCase)
            [ego,target,road,cfg]=localFixture(true);
            [~,~,empty]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [~,~,active]=collisionAvoidanceController(ego,target,road,cfg,[]);
            first=empty.program;second=active.program;
            [matrix,bound]=localPermanentProgram(second);
            testCase.verifyEqual(matrix,first.A,AbsTol=0);
            testCase.verifyEqual(bound,first.b,AbsTol=0);
            testCase.verifyEqual(second.P,first.P,AbsTol=0);
            testCase.verifyEqual(second.q,first.q,AbsTol=0);
            testCase.verifyEqual(first.obstacleCbfRowCount,0);
            testCase.verifyGreaterThan(second.obstacleCbfRowCount,0);
            testCase.verifyEqual(second.cones(3:end),first.cones(3:end));
        end
        function failedStatusesReturnNoCommandAndAreNeverRetried(testCase,failedStatus)
            [ego,target,road,cfg]=localFixture(true);
            localFailureHook('reset',failedStatus);cfg.solver.jointFunction=@localFailureHook;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
            testCase.verifyEqual(localFailureHook('count',[]),1);
        end
        function malformedSolvedResultsRaiseAnError(testCase,badDecision)
            [ego,target,road,cfg]=localFixture(false);
            cfg.solver.jointFunction=@(~,~) struct('decision',badDecision,'exitFlag',1,'output',struct());
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end
        function aPreviousCommandDoesNotProvideAFallback(testCase)
            [ego,target,road,cfg]=localFixture(false);
            [~,~,problem,state]=collisionAvoidanceController(ego,target,road,cfg,[]);
            ego=localSuccessor(ego,problem,state);
            localFailureHook('reset',-2);cfg.solver.jointFunction=@localFailureHook;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,state), ...
                'collisionAvoidanceController:optimizationFailed');
            testCase.verifyEqual(localFailureHook('count',[]),1);
        end
        function legacyOptionsCannotSelectAnotherController(testCase,legacyPolicy)
            [ego,target,road,cfg]=localFixture(true);
            [command,~,original]=collisionAvoidanceController(ego,target,road,cfg,[]);
            cfg.controller.executionPolicy=legacyPolicy;cfg.controller.horizonSteps=64;
            [changed,plan,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifySize(plan,[2,1]);
            testCase.verifyEqual(problem.program.A,original.program.A,AbsTol=0);
            testCase.verifyEqual(changed.actuatorInput,command.actuatorInput,AbsTol=1e-10);
        end
        function cruiseDissipationHoldsForTheDeclaredInformationBox(testCase,errorRadius)
            [ego,target,road,cfg]=localFixture(false);
            ego.position(2)=.05;ego.yaw=.002;ego.speed=7.95;
            ego.controllerStateErrorBound=errorRadius;
            [command,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            residual=localRobustClfResidual(problem,command);
            testCase.verifyLessThanOrEqual(residual,1e-10);
            testCase.verifyTrue(problem.metadata.clfDissipationCertified);
            testCase.verifySize(problem.program.q,[2,1]);
        end
        function conflictingCruiseAndBarrierConstraintsRaiseAnError(testCase)
            [ego,target,road,cfg]=localFixture(true);
            target.targetPositionInertial=[9;0];target.targetVelocityInertial=[0;0];
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end
        function anOverlappingTargetCannotBeAdmitted(testCase)
            [ego,target,road,cfg]=localFixture(true);
            target.targetPositionInertial=[1;0];
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end
        function obstacleRowsEncloseTheWholeExecutedHold(testCase)
            [ego,target,road,cfg]=localFixture(true);
            target.predictionMotion.jerkBound=[.1;.1];
            target.targetPositionInertialErrorBound=[.1;.1];
            [command,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            [minimum,residual]=localBarrierResidual(problem,command);
            testCase.verifyGreaterThanOrEqual(minimum,-1e-9);
            testCase.verifyLessThanOrEqual(residual,1e-9);
        end
        function multipleTargetsShareTheSameOptimization(testCase)
            [ego,target,road,cfg]=localFixture(true);
            targets=[target,target];targets(2).trackId=2;
            targets(2).targetPositionInertial=[-30;-4];
            localHook('reset',[]);cfg.solver.jointFunction=@localHook;
            [~,plan,problem]=collisionAvoidanceController(ego,targets,road,cfg,[]);
            testCase.verifyEqual(localHook('count',[]),1);
            testCase.verifySize(plan,[2,1]);
            testCase.verifySize(problem.metadata.targetErrorBound,[8,2]);
            testCase.verifySize(problem.program.barrier.normal,[2,2]);
        end
        function anUnsafeSecondTargetCannotBeIgnored(testCase)
            [ego,target,road,cfg]=localFixture(true);
            targets=[target,target];targets(2).trackId=2;
            targets(2).targetPositionInertial=[1;0];
            testCase.verifyError(@() collisionAvoidanceController(ego,targets,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end
        function aCollisionBetweenSafeSampledEndpointsStopsTheSolve(testCase)
            [ego,target,road,cfg]=localFixture(true);
            target.targetPositionInertial=[.4;-10];target.targetVelocityInertial=[0;200];
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end
        function roadConstraintsRemainHardWithoutATarget(testCase)
            [ego,target,road,cfg]=localFixture(false);
            ego.position(2)=cfg.model.lateralDomainRadius+.1;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end
        function targetRemovalUsesTheSameSolveWithoutObstacleRows(testCase)
            [ego,target,road,cfg]=localFixture(true);
            [~,~,problem,state]=collisionAvoidanceController(ego,target,road,cfg,[]);
            ego=localSuccessor(ego,problem,state);
            [~,~,next]=collisionAvoidanceController(ego,[],road,cfg,state);
            testCase.verifyFalse(next.metadata.hasTarget);
            testCase.verifyEqual(next.metadata.obstacleCbfRowCount,0);
            testCase.verifyEqual(next.metadata.solverCallCount,1);
        end
        function inconsistentExecutionStopsBeforeSolving(testCase)
            [ego,target,road,cfg]=localFixture(false);
            [~,~,problem,state]=collisionAvoidanceController(ego,target,road,cfg,[]);
            ego=localSuccessor(ego,problem,state);ego.heldActuatorInput=[.1;.2];
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,state), ...
                'collisionAvoidanceController:executionContractViolation');
        end
        function nonzeroPlantResidualIsOutsideTheClaimedScope(testCase)
            [ego,target,road,cfg]=localFixture(false);
            cfg.model.plantModelResidualRateBound=1e-3*ones(6,1);
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:nonexactStudyInput');
        end
        function anInjectedFailureStopsTheExperimentAfterOneHold(testCase)
            folder=string(tempname);mkdir(folder);testCase.addTeardown(@() rmdir(folder,'s'));
            testCase.verifyError(@() runExactStateRecursiveFeasibilityScenario(Scenario="cruise", ...
                SampleCount=5,FailAfterAdmission=true,OutputDirectory=folder), ...
                'collisionAvoidanceController:optimizationFailed');
            saved=load(fullfile(folder,'cruise-exact-state.mat'));
            testCase.verifyEqual(saved.report.executedHolds,1);
            testCase.verifyEqual(saved.report.failureTime,.1,AbsTol=0);
            testCase.verifyFalse(saved.report.completed);
        end
        function pathAndSpeedRecoverUnderTheHardClf(testCase)
            report=runExactStateRecursiveFeasibilityScenario(Scenario="cruise",SampleCount=120, ...
                InitialTrackingError=[.05;.002;-.05;0;0]);
            testCase.verifyTrue(report.passed);
            testCase.verifyLessThan(norm(report.state(2:6,end)-[0;0;8;0;0]),1e-3);
            testCase.verifyLessThanOrEqual(max(report.clfDissipationResidual),0);
            testCase.verifyEqual(report.solverCallCount,ones(1,120));
        end
    end
end

function [ego,target,road,cfg]=localFixture(present)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('sampleTime',.1),'model',struct('lateralDomainRadius',4)));
    ego=struct('position',[0;0],'yaw',0,'speed',8,'stateTime',0);
    road=[-100,0;2000,0];
    target=[];
    if present
        target=struct('trackId',1,'targetPositionInertial',[30;4], ...
            'targetVelocityInertial',[0;0],'targetAccelerationInertial',[0;0], ...
            'targetHeadingInertial',0,'targetYawRate',0, ...
            'predictionMotion',struct('kind',"finite-sensing-motion-v1", ...
            'jerkBound',[0;0],'yawAccelerationBound',0));
    end
end

function result=localHook(mode,program)
    persistent count
    if string(mode)=="reset",count=0;result=[];return;end
    if string(mode)=="count",result=count;return;end
    count=count+1;result=program.defaultSolver();
end

function result=localFailureHook(mode,value)
    persistent count status
    if string(mode)=="reset",count=0;status=value;result=[];return;end
    if string(mode)=="count",result=count;return;end
    count=count+1;result=struct('decision',[0;0],'exitFlag',status,'output',struct());
end

function ego=localSuccessor(ego,problem,state)
    x=problem.predictedState(:,2);
    [ego.position,ego.yaw]=laneGeometry.fromFrenet(x,problem.model.lane);
    ego.speed=x(4);ego.lateralVelocity=x(5);ego.yawRate=x(6);
    ego.stateTime=ego.stateTime+problem.model.sampleTime;ego.heldActuatorInput=state.appliedInput;
end

function [matrix,bound]=localPermanentProgram(program)
    count=program.cones(2);obstacles=program.obstacleCbfRowCount;
    selected=count-4-obstacles+(1:obstacles);
    matrix=program.A;bound=program.b;matrix(selected,:)=[];bound(selected)=[];
end

function residual=localRobustClfResidual(problem,command)
    meta=problem.metadata;model=problem.model;
    flow=expm(model.sampleTime*[meta.executedContinuousGenerator;zeros(3,9)]);
    % All box corners and interior deterministic points are experimental
    % validation; the norm/Young bound in the derivation supplies the proof.
    signs=2*dec2bin(0:63,6).'-'0'*2-1;
    points=[signs,zeros(6,1),.5*signs];
    initial=model.initialEgoState+model.initialFrenetErrorBound.*points;
    next=flow(1:6,:)*[initial;repmat(command.actuatorInput,1,size(initial,2));ones(1,size(initial,2))];
    e=initial(2:6,:)-meta.clfReferenceState(2:6);f=next(2:6,:)-meta.clfReferenceState(2:6);
    residual=max(sum(f.*(meta.clfMatrix*f),1)-(1-meta.clfDecayPerHold)*sum(e.*(meta.clfMatrix*e),1) ...
        -meta.clfDisturbanceBound);
end

function [minimum,residual]=localBarrierResidual(problem,command)
    model=problem.model;target=model.encounters;normal=problem.program.barrier.normal;
    support=hypot(model.cfg.vehicle.length/2,model.cfg.vehicle.width/2) ...
        +hypot(target.halfLength,target.halfWidth)+model.cfg.collision.clearanceMargin;
    initial=problem.program.barrier.initialUpper;minimum=inf;last=NaN;
    for time=linspace(0,model.sampleTime,101)
        flow=expm(time*[problem.metadata.executedContinuousGenerator;zeros(3,9)]);
        x=flow(1:6,:)*[model.initialEgoState;command.actuatorInput;1];
        position=laneGeometry.fromFrenet(x,model.lane);
        [center,radius]=targetPrediction.finiteFlow(target,time);
        last=normal.'*(position-center(1:2))-abs(normal).'*radius(1:2)-support;
        minimum=min(minimum,last);
    end
    residual=problem.program.barrier.contraction*initial-last;
end
