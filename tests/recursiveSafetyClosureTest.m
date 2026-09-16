classdef recursiveSafetyClosureTest < matlab.unittest.TestCase
    %recursiveSafetyClosureTest Closed successor chain and numeric witnesses.
    properties (TestParameter)
        radius = {zeros(6,1),[.001;.001;.0001;.001;.001;.0001]};
        curvature = {0,.02};
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
        function terminalPolicyUsesTheActualPlantAndCertifiesTheWholeBall(testCase,radius,curvature)
            [ego,road,cfg]=localFixture(radius,curvature);
            [~,~,problem,state]=collisionAvoidanceController(ego,[],road,cfg,[]);
            t=state.terminal;
            support=t.holdSupport*t.radius+t.holdNoise;
            testCase.verifyEqual([t.continuousA,t.continuousB,t.continuousC], ...
                problem.metadata.executedContinuousGenerator,AbsTol=0);
            testCase.verifyLessThanOrEqual(max(support-t.holdBound),0);
            testCase.verifyLessThanOrEqual(t.comparison*t.radius+t.disturbanceSupport,t.radius-2*t.reserve);
            testCase.verifyTrue(problem.metadata.recursiveFeasibilityGuaranteed);
        end
        function exhaustedPredictionStillOffersAFreeOptimizedSafeHold(testCase,radius,curvature)
            [ego,road,cfg]=localFixture(radius,curvature);
            [~,~,problem,state]=collisionAvoidanceController(ego,[],road,cfg,[]);
            model=problem.model;t=state.terminal;
            model.initialEgoState=t.reference+[100;real(t.modalBasis*(.99*t.radius))];
            model.initialFrenetErrorBound=radius;
            model.cruiseCertificate=t.cruise;model.carriedWitness=[];
            model.horizonSteps=1;model.terminalOptimization=true;
            model.previousInput=t.input;
            model.initializationPlan=t.input+t.feedback*(model.initialEgoState-t.reference);
            [program,~,~]=formulateAvoidanceProblem(model);
            margins=localConeMargins(program,program.feasibleWitness);
            result=solveHardCbfClf.constrained(program,cfg);
            certified=solveHardCbfClf.certify(program,result.decision);
            testCase.verifyGreaterThanOrEqual(min(margins),0);
            testCase.verifyTrue(result.feasible);
            testCase.verifyEqual(program.layout.planCount,2);
            testCase.verifyGreaterThanOrEqual(min(localPhysicalMargins(certified,result.decision)),0);
        end
        function confirmedPartialReleasePreservesTheOtherEncounter(testCase)
            [ego,road,cfg]=localFixture(zeros(6,1),0);
            target=localTarget(1,[18;4],[20;0]);
            targets=[target,localTarget(2,[12;4],[16;0])];
            [~,~,p,s]=collisionAvoidanceController(ego,targets,road,cfg,[]);
            ego=localNext(ego,p,s);
            targets(1).targetPositionInertial=[20;4];
            targets(2).targetPositionInertial=[13.6;4];
            [~,~,next]=collisionAvoidanceController(ego,targets([2,1]),road,cfg,s);
            testCase.verifyTrue(next.metadata.confirmedRelease);
            testCase.verifyTrue(next.metadata.inheritedFeasibleFamily);
            testCase.verifyNumElements(next.model.encounters,1);
            testCase.verifyEqual(next.program.completion.keys,"trackId:2");
            testCase.verifyFalse(any(ismember(next.program.physicalLabels,["collision:trackId:1","exit:trackId:1"])));
            testCase.verifyGreaterThanOrEqual(min(localConeMargins(next.program,next.program.feasibleWitness)),-1e-11);
        end
        function targetFreeSuccessorsOutliveTheOriginalHorizonAndReferenceSamples(testCase,radius,curvature)
            result=localSequence(radius,40,curvature);
            testCase.verifyTrue(all(result.certified));
            testCase.verifyGreaterThanOrEqual(min(result.margins),0);
            testCase.verifyTrue(all(result.replacement));
            testCase.verifyGreaterThan(result.station,20);
            testCase.verifyFalse(any(result.fallback));
        end
        function aFalseSolverSuccessCannotBecomeAnUnsafeWitness(testCase)
            [ego,road,cfg]=localFixture(zeros(6,1),0);
            cfg.solver.jointFunction=@localUnsafe;
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end
        function anApproximateSolutionStillNeedsACompleteCertificate(testCase)
            [ego,road,cfg]=localFixture(zeros(6,1),0);
            cfg.solver.jointFunction=@localApproximate;
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyTrue(problem.metadata.approximateSolveCertified);
            testCase.verifyTrue(problem.metadata.postSolveCertificationPerformed);
            testCase.verifyGreaterThanOrEqual(min(localPhysicalMargins(problem.program,problem.decision)),0);
        end
        function anEnlargedEgoSensingBoundIsAChangedContract(testCase)
            [ego,road,cfg]=localFixture(zeros(6,1),0);
            [~,~,p,s]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localNext(ego,p,s);ego.controllerStateErrorBound=.001*ones(6,1);
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,s), ...
                'collisionAvoidanceController:changedMeasurementContract');
        end
        function invariantOptimizationSurvivesAlternatingMeasurementErrors(testCase)
            result=localTerminalSequence();
            testCase.verifyGreaterThanOrEqual(min(result.modalMargin),0);
            testCase.verifyGreaterThanOrEqual(min(result.slewMargin),0);
            testCase.verifyGreaterThanOrEqual(min(result.physicalMargin),0);
            testCase.verifyTrue(all(result.optimized));
        end
    end
end

function [ego,road,cfg]=localFixture(radius,curvature)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'controller', ...
        struct('sampleTime',.1,'horizonSteps',8,'minimumHorizonSteps',1), ...
        'model',struct('lateralDomainRadius',4,'frontWheelSteeringRateMaximum',5, ...
        'brakingRatioRateMaximum',10)));
    ego=struct('position',[0;0],'yaw',0,'speed',8,'stateTime',0, ...
        'controllerStateErrorBound',radius,'perception',struct('time',0,'range',16,'completeWithinRange',true));
    road=[-1,0;1,0];
    if curvature~=0
        curve=struct('origin',[0;0],'heading',0,'curvature',curvature,'length',40);
        road=struct('centerline',[0,0;40,0],'referenceCurve',curve);
        [trim,~]=ltvBicycleModel.cruiseEquilibrium(curvature,cfg,0);
        ego.yaw=trim(3);ego.lateralVelocity=trim(5);ego.yawRate=trim(6);
    end
end

function target=localTarget(id,position,velocity)
    target=struct('trackId',id,'targetPositionInertial',position, ...
        'targetVelocityInertial',velocity,'targetAccelerationInertial',[0;0], ...
        'targetHeadingInertial',0,'targetYawRate',0, ...
        'predictionMotion',struct('kind',"finite-sensing-motion-v1", ...
        'jerkBound',[0;0],'yawAccelerationBound',0));
end

function ego=localNext(ego,p,s)
    x=s.predictedState(:,2);
    [ego.position,ego.yaw]=laneGeometry.fromFrenet(x,p.model.lane);
    ego.speed=x(4);ego.lateralVelocity=x(5);ego.yawRate=x(6);
    ego.stateTime=ego.stateTime+p.model.sampleTime;
    ego.heldActuatorInput=s.appliedInput;ego.perception.time=ego.stateTime;
end

function margins=localConeMargins(program,decision)
    value=program.b-program.A*decision;n=program.cones(2);
    margins=value(1:n);
    for dimension=program.cones(3:end).'
        cone=value(n+(1:dimension));
        margins(end+1)=cone(1)-norm(cone(2:end)); %#ok<AGROW>
        n=n+dimension;
    end
end

function margins=localPhysicalMargins(program,decision)
    cone=program.terminalConePhysicalBound-program.terminalCone.matrix*decision(program.layout.planIndex);
    margins=program.physicalBound-program.physicalMatrix*decision;
    for first=1:3:numel(cone)
        margins(end+1)=cone(first)-norm(cone(first+(1:2))); %#ok<AGROW>
    end
end

function result=localSequence(radius,count,curvature)
    [ego,road,cfg]=localFixture(radius,curvature);stored=[];
    if curvature~=0
        station=road.referenceCurve.length/2+pi/abs(curvature)-1;
        [ego.position,heading]=laneGeometry.referencePose(station,0,road.referenceCurve);
        ego.yaw=ego.yaw+heading;
    end
    result=struct('certified',false(1,count),'margins',nan(1,count), ...
        'replacement',false(1,count-1),'fallback',false(1,count));
    for index=1:count
        [~,~,p,stored]=collisionAvoidanceController(ego,[],road,cfg,stored);
        result.certified(index)=p.metadata.recursiveFeasibilityGuaranteed;
        result.margins(index)=min(localPhysicalMargins(p.program,p.decision));
        result.fallback(index)=p.metadata.fallbackUsed;
        if index>1,result.replacement(index-1)=p.program.replacementContainsWitness;end
        ego=localNext(ego,p,stored);
    end
    result.station=stored.predictedState(1,2);
end

function result=localUnsafe(~,program)
    result=program.defaultSolver();result.decision(1)=2;
end

function result=localApproximate(~,program)
    result=program.defaultSolver();result.exitFlag=2;
end

function result=localTerminalSequence()
    radius=[.1;.1;.01;.1;.1;.001];
    [ego,road,cfg]=localFixture(radius,0);
    [~,~,p,s]=collisionAvoidanceController(ego,[],road,cfg,[]);
    t=s.terminal;model=p.model;model.cruiseCertificate=t.cruise;
    model.permanentTerminal=t;model.carriedWitness=[];model.horizonSteps=1;
    model.terminalOptimization=true;
    truth=t.reference;truth(1)=100;
    previous=t.input;count=40;
    result=struct('modalMargin',nan(1,count),'slewMargin',nan(1,count), ...
        'physicalMargin',nan(1,count),'optimized',false(1,count));
    for index=1:count
        model.initialEgoState=truth+(-1)^index*radius;
        model.initialFrenetErrorBound=radius;model.previousInput=previous;
        model.initializationPlan=t.input+t.feedback*(model.initialEgoState-t.reference);
        [program,~,~]=formulateAvoidanceProblem(model);
        solve=solveHardCbfClf.constrained(program,cfg);
        program=solveHardCbfClf.certify(program,solve.decision);
        input=solve.decision(1:2);
        truth=t.cruise.transition(1:6,:)*[truth;input;1];
        result.modalMargin(index)=min(t.radius-abs(t.modalMatrix*(truth(2:6)-t.reference(2:6))));
        rate=cfg.controller.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
        result.slewMargin(index)=min(rate-abs(input-previous));
        result.physicalMargin(index)=min(localPhysicalMargins(program,solve.decision));
        result.optimized(index)=solve.feasible;
        previous=input;
    end
end
