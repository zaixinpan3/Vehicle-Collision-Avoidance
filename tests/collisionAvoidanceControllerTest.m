classdef collisionAvoidanceControllerTest < matlab.unittest.TestCase
    properties (TestParameter)
        targetPresent = {false,true};
        failedStatus = {-999,-2,0,2};
        badDecision = {[],[NaN;0;0],[Inf;0;0],[1i;0;0],zeros(4,1)};
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
            testCase.verifySize(plan,[2,problem.metadata.horizonSteps]);
            testCase.verifyEqual(command.actuatorInput,plan(:,1),AbsTol=0);
            testCase.verifyEqual(state.appliedInput,plan(:,1),AbsTol=0);
            testCase.verifyEqual(problem.metadata.solverCallCount,1);
            testCase.verifyTrue(problem.metadata.postSolveCertificationPerformed);
            testCase.verifyTrue(problem.metadata.recursiveFeasibilityGuaranteed);
            testCase.verifyGreaterThan(size(state.plan,2),1);
            testCase.verifyTrue(problem.metadata.predictionContinuationRetained);
            testCase.verifyTrue(state.terminal.targetIndependent);
            testCase.verifyFalse(problem.metadata.terminalActive);
        end
        function targetFreeControlRetainsThePermanentTerminalCertificate(testCase)
            [ego,target,road,cfg]=localFixture(true);
            [~,~,empty]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [~,~,active]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyEqual(empty.program.obstacleCbfRowCount,0);
            testCase.verifyGreaterThan(active.program.obstacleCbfRowCount,0);
            testCase.verifyTrue(empty.program.terminal.targetIndependent);
            testCase.verifyFalse(empty.program.completion.active);
            testCase.verifyTrue(active.program.completion.active);
            testCase.verifyEqual(empty.metadata.clfOperatingInput,active.metadata.clfOperatingInput);
        end
        function failedStatusesNeverIssueAnUncertifiedCommand(testCase,failedStatus)
            [ego,target,road,cfg]=localFixture(true);
            localFailureHook('reset',failedStatus);cfg.solver.jointFunction=@localFailureHook;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
            if failedStatus==-2
                testCase.verifyGreaterThan(localFailureHook('count',[]),1);
            else
                testCase.verifyEqual(localFailureHook('count',[]),1);
            end
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
        function configurationControlsThePredictionLengthWithoutSelectingAFallback(testCase,legacyPolicy)
            [ego,target,road,cfg]=localFixture(false);
            cfg.controller.executionPolicy=legacyPolicy;cfg.controller.horizonSteps=8;
            [~,plan,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifySize(plan,[2,8]);
            testCase.verifyEqual(problem.metadata.solverCallCount,1);
            testCase.verifyFalse(problem.metadata.fallbackUsed);
            testCase.verifyEqual(problem.metadata.terminalPolicyRole,"predictionCertificateOnly");
        end
        function cruiseDissipationHoldsForTheDeclaredInformationBox(testCase,errorRadius)
            [ego,target,road,cfg]=localFixture(false);
            ego.position(2)=.05;ego.yaw=.002;ego.speed=7.95;
            ego.controllerStateErrorBound=errorRadius;
            [command,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            residual=localRobustClfResidual(problem,command);
            testCase.verifyLessThanOrEqual(residual,1e-10);
            testCase.verifyTrue(problem.metadata.clfDissipationCertified);
            testCase.verifySize(problem.program.q,[2*problem.metadata.horizonSteps+1,1]);
        end
        function avoidanceCanRelaxCruiseWhileKeepingTheBarrierHard(testCase)
            [ego,target,road,cfg]=localFixture(true);
            target.targetPositionInertial=[15;0];target.targetVelocityInertial=[0;0];
            [command,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            [minimum,residual]=localBarrierResidual(problem,command);
            testCase.verifyGreaterThan(problem.metadata.clfSlack,1e-4);
            testCase.verifyGreaterThanOrEqual(minimum,0);
            testCase.verifyLessThanOrEqual(residual,0);
            testCase.verifyLessThanOrEqual(localRobustClfResidual(problem,command),1e-10);
            testCase.verifyGreaterThan(problem.metadata.clfNextValue,problem.metadata.clfInitialValue);
        end
        function theConfiguredPenaltyControlsOptimizedRelaxation(testCase)
            [ego,target,road,cfg]=localFixture(false);
            ego.position(2)=.05;ego.yaw=.002;ego.speed=7.95;
            cfg.clf.relaxationWeight=.01;
            [~,~,low]=collisionAvoidanceController(ego,target,road,cfg,[]);
            cfg.clf.relaxationWeight=100;
            [~,~,high]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyGreaterThanOrEqual(high.metadata.clfSlack,0);
            testCase.verifyLessThan(high.metadata.clfSlack,low.metadata.clfSlack);
            testCase.verifyEqual(high.metadata.clfSlackPenalty, ...
                cfg.clf.relaxationWeight*high.decision(high.program.layout.relaxationIndex)^2,AbsTol=1e-12);
            testCase.verifyEqual(high.decision(1:2),high.inputPlan(:,1),AbsTol=0);
            testCase.verifyEqual(high.program.A,low.program.A,AbsTol=0);
            testCase.verifyEqual(high.program.b,low.program.b,AbsTol=0);
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
            targets(2).targetPositionInertial=[-12;-4];targets(2).targetVelocityInertial=[-16;0];
            localHook('reset',[]);cfg.solver.jointFunction=@localHook;
            [~,plan,problem]=collisionAvoidanceController(ego,targets,road,cfg,[]);
            testCase.verifyEqual(localHook('count',[]),1);
            testCase.verifySize(plan,[2,problem.metadata.horizonSteps]);
            testCase.verifySize(problem.metadata.targetErrorBound,[8,2]);
            testCase.verifySize(problem.program.completion.direction,[2,2]);
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
        function aMissingTargetInsideTheRangeIsNotAConfirmedRelease(testCase)
            [ego,target,road,cfg]=localFixture(true);
            [~,~,problem,state]=collisionAvoidanceController(ego,target,road,cfg,[]);
            ego=localSuccessor(ego,problem,state);
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,state), ...
                'collisionAvoidanceController:inconsistentObservation');
        end
        function theShiftedPlanIsFeasibleInTheNextOptimization(testCase,errorRadius)
            [ego,target,road,cfg]=localFixture(true);
            ego.controllerStateErrorBound=errorRadius;
            [~,~,problem,state]=collisionAvoidanceController(ego,target,road,cfg,[]);
            ego=localSuccessor(ego,problem,state);
            target.targetPositionInertial=target.targetPositionInertial+cfg.controller.sampleTime*target.targetVelocityInertial;
            [~,~,next]=collisionAvoidanceController(ego,target,road,cfg,state);
            witness=state.plan(:,2:end);
            residual=next.program.physicalMatrix*[witness(:);0]-next.program.physicalBound;
            testCase.verifyLessThanOrEqual(max(residual),0);
            testCase.verifyTrue(next.metadata.inheritedFeasibleFamily);
            testCase.verifyEqual(next.program.completion.deadline,problem.program.completion.deadline);
            testCase.verifyEqual(next.metadata.horizonSteps,problem.metadata.horizonSteps-1);
            testCase.verifyEqual(next.carriedWitness.stages,state.stages(2:end));
        end
        function confirmedDepartureStartsAnotherOptimizedCruisePlan(testCase)
            [ego,target,road,cfg]=localFixture(true);
            target.targetPositionInertial=[18;4];target.targetVelocityInertial=[20;0];
            [~,~,problem,state]=collisionAvoidanceController(ego,target,road,cfg,[]);
            ego=localSuccessor(ego,problem,state);
            target.targetPositionInertial=target.targetPositionInertial+cfg.controller.sampleTime*target.targetVelocityInertial;
            [~,plan,next]=collisionAvoidanceController(ego,target,road,cfg,state);
            testCase.verifyTrue(next.metadata.confirmedRelease);
            testCase.verifyFalse(next.metadata.hasTarget);
            testCase.verifyFalse(next.metadata.terminalActive);
            testCase.verifySize(plan,[2,cfg.controller.horizonSteps]);
            testCase.verifyEqual(next.metadata.solverCallCount,1);
        end
        function acceptedTerminalBoxesHaveAnInvariantContinuation(testCase,errorRadius)
            [ego,target,road,cfg]=localFixture(false);
            ego.controllerStateErrorBound=errorRadius;
            [~,~,~,state]=collisionAvoidanceController(ego,target,road,cfg,[]);
            [margin,inputMargin,slewMargin]=localTerminalAudit(state,cfg);
            testCase.verifyGreaterThanOrEqual(margin,-1e-10);
            testCase.verifyGreaterThanOrEqual(inputMargin,0);
            testCase.verifyGreaterThanOrEqual(slewMargin,0);
        end
        function changingTheSavedPlanInvalidatesItsCertificate(testCase)
            [ego,target,road,cfg]=localFixture(false);
            [~,~,problem,state]=collisionAvoidanceController(ego,target,road,cfg,[]);
            ego=localSuccessor(ego,problem,state);state.plan(2,end)=state.plan(2,end)+.1;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,state), ...
                'collisionAvoidanceController:invalidStoredCertificate');
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
                SampleCount=5,FailAfterAdmission=true,DeadlineSeconds=Inf,OutputDirectory=folder), ...
                'collisionAvoidanceController:optimizationFailed');
            saved=load(fullfile(folder,'cruise-exact-state.mat'));
            testCase.verifyEqual(saved.report.executedHolds,1);
            testCase.verifyEqual(saved.report.failureTime,.1,AbsTol=0);
            testCase.verifyFalse(saved.report.completed);
        end
        function pathAndSpeedRecoverWithTheOptimizedClfSlack(testCase)
            report=runExactStateRecursiveFeasibilityScenario(Scenario="cruise",SampleCount=120, ...
                InitialTrackingError=[.05;.002;-.05;0;0],DeadlineSeconds=Inf);
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
    ego=struct('position',[0;0],'yaw',0,'speed',8,'stateTime',0, ...
        'perception',struct('time',0,'range',16,'completeWithinRange',true));
    road=[-100,0;2000,0];
    target=[];
    if present
        target=struct('trackId',1,'targetPositionInertial',[12;4], ...
            'targetVelocityInertial',[16;0],'targetAccelerationInertial',[0;0], ...
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
    ego.perception.time=ego.stateTime;
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
        -meta.clfDisturbanceBound-meta.clfSlackDissipationBound);
end

function [minimum,residual]=localBarrierResidual(problem,command)
% Independent rectangle audit of the first hold, plus all physical plan rows.
    model=problem.model;target=model.encounters(1);
    generator=[problem.metadata.executedContinuousGenerator;zeros(3,9)];
    minimum=Inf;
    for time=linspace(0,model.sampleTime,21)
        state=expm(time*generator)*[model.initialEgoState;command.actuatorInput;1];
        [position,heading]=laneGeometry.fromFrenet(state(1:6),model.lane);
        center=targetPrediction.finiteFlow(target,time);
        distance=avoidanceSafetyGeometry.rectangleDistance(position,heading,center(1:2),center(7), ...
            [model.cfg.vehicle.length/2;model.cfg.vehicle.width/2;target.halfLength;target.halfWidth]);
        minimum=min(minimum,distance-model.cfg.collision.clearanceMargin);
    end
    residual=max(problem.program.physicalMatrix*problem.decision-problem.program.physicalBound);
end

function [margin,inputMargin,slewMargin]=localTerminalAudit(state,cfg)
% Audit the hypothetical certificate without executing a controller fallback.
    center=state.predictedState(:,end);radius=state.stateErrorBound(:,end);
    prior=state.plan(:,end);margin=Inf;inputMargin=Inf;slewMargin=Inf;
    lower=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
    upper=[cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
    rate=cfg.controller.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    for index=1:100
        [~,margins]=hardEncounterBarrier.terminalMembership(state.terminal,center,radius);
        margin=min(margin,min(margins));
        step=hardEncounterBarrier.terminalStep(state.terminal,center,radius,cfg.controller.sampleTime);
        inputMargin=min([inputMargin;step.input-lower;upper-step.input]);
        slewMargin=min([slewMargin;rate-abs(step.input-prior)]);
        center=step.successor;radius=min(step.successorRadius,state.terminal.measurementRadiusLimit);prior=step.input;
    end
end
