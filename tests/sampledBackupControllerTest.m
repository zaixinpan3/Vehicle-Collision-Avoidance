classdef sampledBackupControllerTest < matlab.unittest.TestCase
    % Safety and dissipation of the bounded-work backup policy.
    properties (TestParameter)
        uncertainty = struct('exact',zeros(6,1),'bounded',[.01;.01;.001;.01;.002;.001]);
        scene = {"stationary","oncoming","crossing"};
        curvature = struct('left',1/400,'right',-1/400,'turn',1/100);
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (Test)
        function cruiseDissipatesForEveryVertexOfTheInformationBox(testCase,uncertainty)
            [problem,worst,roadMargin] = localCruiseAudit(uncertainty);
            testCase.verifyTrue(problem.metadata.clfDissipationCertified);
            testCase.verifyLessThanOrEqual(worst,2e-12);
            testCase.verifyGreaterThan(roadMargin,0);
            testCase.verifyEqual(problem.metadata.solverCallCount,0);
        end

        function cachedSweptTubesEncloseEveryInitialBoxVertex(testCase,uncertainty)
            worst = localSweptAudit(uncertainty);
            testCase.verifyLessThanOrEqual(worst,5e-12);
        end

        function aCurvedPathTrimHasCheckedSampledDissipation(testCase,curvature)
            [problem,residual]=localCurvedAudit(curvature);
            testCase.verifyTrue(problem.metadata.safetyCertified);
            testCase.verifyTrue(problem.metadata.clfDissipationCertified);
            testCase.verifyLessThanOrEqual(residual,1e-12);
            testCase.verifyEqual(problem.metadata.clfOperatingCurvature,curvature,AbsTol=1e-14);
        end

        function anOnlineSolverIsUnnecessaryForAvoidanceAndCruise(testCase,scene)
            report = runExactStateRecursiveFeasibilityScenario(Scenario=scene,SampleCount=180, ...
                ExecutionPolicy="backup",FailAfterAdmission=true,Seed=20260914);
            testCase.verifyTrue(report.passed,report.failureMessage);
            testCase.verifyEqual(report.executedHolds,180);
            testCase.verifyLessThan(norm(report.finalCruiseError),1e-3);
            testCase.verifyTrue(any(report.certificateSource=="sampledClfCruise"));
        end

        function theTenMetrePerSecondRangeSceneReturnsToItsPath(testCase)
            report = runDeclaredPlantEstimatorControllerScenario(SampleCount=300,UseEstimator=false,Seed=20260914);
            testCase.verifyTrue(report.completed,report.failure.message);
            testCase.verifyLessThan(max(abs(report.state(2,end-50:end))),1e-4);
            testCase.verifyLessThan(max(abs(report.state(4,end-50:end)-10)),1e-6);
            testCase.verifyGreaterThan(report.minimumSeparationMargin,0);
            testCase.verifyGreaterThan(report.minimumRoadMargin,0);
        end

        function aStoppedExactEgoCanEnterCertifiedCruise(testCase)
            [ego,road,cfg] = localScene();
            ego.speed=0;
            [command,~,problem] = collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyTrue(problem.metadata.safetyCertified);
            testCase.verifyGreaterThan(command.actuatorInput(2),0);
        end

        function ordinaryCruiseSpeedFitsTheAdjustedStoppingSet(testCase)
            [ego,road,cfg] = localScene();
            ego.speed=15;cfg.referenceSpeed=15;
            [~,~,problem,certificate] = collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyTrue(problem.metadata.clfDissipationCertified);
            testCase.verifyGreaterThan(certificate.terminal.velocityLimit(1),15);
            testCase.verifyGreaterThan(certificate.terminal.longitudinalRatio,0);
            testCase.verifyLessThan(certificate.terminal.longitudinalRatio,1);
        end

        function anExpiredCruiseBudgetKeepsTheRoadBackup(testCase)
            [ego,road,cfg] = localScene();
            [~,~,problem,certificate] = collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=encounterTestFixture.nextEgo(certificate,problem.model.lane);
            cfg.solver.frameDeadlineSeconds=1e-12;
            [~,~,next] = collisionAvoidanceController(ego,[],road,cfg,certificate);
            testCase.verifyTrue(next.metadata.safetyCertified);
            testCase.verifyTrue(next.metadata.terminalActive);
            testCase.verifyFalse(next.metadata.clfDissipationCertified);
            testCase.verifyEqual(next.metadata.certificateSearchAttempts,0);
        end

        function changingTheHeldInputInvalidatesTheWitness(testCase)
            [ego,road,cfg] = localScene();
            [~,~,problem,certificate] = collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=encounterTestFixture.nextEgo(certificate,problem.model.lane);
            ego.heldActuatorInput(1)=ego.heldActuatorInput(1)+.01;
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,certificate), ...
                'collisionAvoidanceController:executionContractViolation');
        end

        function aFiniteBrakeSlewHasACertifiedTransitionTail(testCase)
            [ego,road,cfg] = localScene();
            cfg.model.frontWheelSteeringRateMaximum=.5;
            cfg.model.brakingRatioRateMaximum=2;
            [~,inputs,problem,certificate] = collisionAvoidanceController(ego,[],road,cfg,[]);
            difference=diff([zeros(2,1),inputs],1,2);
            testCase.verifyTrue(problem.metadata.clfDissipationCertified);
            testCase.verifyTrue(problem.metadata.safetyCertified);
            testCase.verifyGreaterThan(certificate.remainingSteps,1);
            testCase.verifyLessThanOrEqual(max(abs(difference(1,:))),.05+1e-12);
            testCase.verifyLessThanOrEqual(max(abs(difference(2,:))),.2+1e-12);
        end

        function fixedVerificationDoesNotDiscardProcessResiduals(testCase)
            [ego,road,cfg] = localScene();
            [~,~,problem] = collisionAvoidanceController(ego,[],road,cfg,[]);
            cruise=ltvBicycleModel.sampledCruise(problem.model);
            model=problem.model;model.cfg.model.plantModelResidualRateBound(4)=.01;
            testCase.verifyError(@() hardEncounterBarrier.verifyFixed(model,cruise.input,cruise.stage), ...
                'collisionAvoidanceController:nonexactStudyInput');
        end

        function theFixedVerifierRejectsAnUnsafeInputSequence(testCase)
            [ego,road,cfg] = localScene();
            [~,~,problem] = collisionAvoidanceController(ego,[],road,cfg,[]);
            cruise=ltvBicycleModel.sampledCruise(problem.model);
            check=hardEncounterBarrier.verifyFixed(problem.model,repmat([.6;0],1,8),cruise.stage);
            testCase.verifyFalse(check.accepted);
            testCase.verifyGreaterThan(check.hardRowViolation,0);
        end

        function aMovingOffsetReferenceIsNotReportedAsConstantCruise(testCase)
            [ego,road,cfg] = localScene();
            cfg.clf.referenceRate(1)=.1;
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:unsupportedCruiseReference');
        end

        function aPreverifiedOptimizedPlanCanFeedTheFastExecutor(testCase)
            [ego,target,road,cfg] = encounterTestFixture.crossing();
            cfg.controller.executionPolicy="auto";
            [~,~,problem,certificate] = collisionAvoidanceController(ego,target,road,cfg,[]);
            ego=encounterTestFixture.nextEgo(certificate,problem.model.lane);
            target.targetPositionInertial=target.targetPositionInertial+.1*target.targetVelocityInertial;
            cfg.solver.frameDeadlineSeconds=.1;
            cfg.solver.jointFunction=@localForbiddenSolver;
            [command,~,next] = collisionAvoidanceController(ego,target,road,cfg,certificate);
            testCase.verifyEqual(command.actuatorInput,certificate.plan(:,2),AbsTol=0);
            testCase.verifyEqual(next.metadata.executedContinuousGenerator, ...
                [certificate.stages(2).continuousA,certificate.stages(2).continuousB,certificate.stages(2).continuousC]);
            testCase.verifyTrue(next.metadata.safetyCertified);
            testCase.verifyEqual(next.metadata.solverCallCount,0);
        end
    end
end

function [ego,road,cfg] = localScene()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',10, ...
        'controller',struct('sampleTime',.1,'horizonSteps',16,'executionPolicy','backup'), ...
        'model',struct('lateralDomainRadius',4),'solver',struct('frameDeadlineSeconds',.1, ...
        'jointFunction',@localForbiddenSolver)));
    ego=struct('position',[0;0],'yaw',0,'speed',10,'stateTime',0, ...
        'perception',struct('time',0,'range',16,'completeWithinRange',true));
    boundary=struct('origin',zeros(2,1),'longitudinalDirection',[1;0],'lateralDirection',[0;1], ...
        'coefficients',[0;0;-5],'parameterRange',[-200;2000],'safeSideSign',1);
    boundaries=[boundary;boundary];boundaries(2).coefficients(3)=5;boundaries(2).safeSideSign=-1;
    road=struct('centerline',[-200,0;2000,0],'boundaries',boundaries);
end

function [problem,worst,minimumRoad] = localCruiseAudit(radius)
    [ego,road,cfg]=localScene();
    ego.position(2)=.15;ego.yaw=.01;ego.speed=9.95;
    ego.controllerStateErrorBound=radius;
    [command,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
    center=problem.model.initialEgoState;
    radius=problem.model.initialFrenetErrorBound;
    signs=2*double(dec2bin(0:63,6).'-'0')-1;
    initial=center+radius.*signs;
    generator=[problem.metadata.executedContinuousGenerator;zeros(3,9)];
    flowed=expm(.1*generator)*[initial;repmat(command.actuatorInput,1,64);ones(1,64)];
    reference=problem.metadata.clfReferenceState;p=problem.metadata.clfMatrix;
    before=initial(2:6,:)-reference;after=flowed(2:6,:)-reference;
    values=sum(after.*(p*after),1)-(1-problem.metadata.clfDissipation.decayPerHold)*sum(before.*(p*before),1);
    worst=max(values)-problem.metadata.clfDissipation.disturbanceBound;
    minimumRoad=Inf;
    for time=linspace(0,.1,21)
        state=expm(time*generator)*[initial;repmat(command.actuatorInput,1,64);ones(1,64)];
        support=cfg.vehicle.length/2*abs(sin(state(3,:)))+cfg.vehicle.width/2*abs(cos(state(3,:)));
        minimumRoad=min(minimumRoad,min(5-abs(state(2,:))-support-cfg.collision.clearanceMargin));
    end
end

function result = localForbiddenSolver(varargin) %#ok<STOUT>
    error('sampledBackupControllerTest:unexpectedSolver','The backup policy called an online optimizer.');
end

function worst = localSweptAudit(radius)
    [ego,road,cfg]=localScene();
    [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
    model=problem.model;model.initialFrenetErrorBound=radius;
    cruise=ltvBicycleModel.sampledCruise(model);
    inputs=cruise.input+[-.015,.018,-.003;-.03,.04,0];
    generator=[cruise.stage.continuousA,cruise.stage.continuousB,cruise.stage.continuousC;zeros(3,9)];
    signs=2*double(dec2bin(0:63,6).'-'0')-1;
    worst=-Inf;
    % The second initial state reuses the cached domain template.
    for lateral=[0,.2]
        model.initialEgoState(2)=lateral;
        prediction=ltvBicycleModel.fixedPredict(model,inputs,cruise.stage);
        initial=model.initialEgoState+radius.*signs;
        for stage=1:size(inputs,2)
            cells=prediction.cells([prediction.cells.stage]==stage);
            for index=1:numel(cells)
                tube=cells(index);degree=size(tube.offset,2)-1;
                points=tube.offset+reshape(pagemtimes(tube.map,inputs(:,stage)),6,[]);
                for fraction=linspace(0,1,11)
                    powers=0:degree;
                    basis=arrayfun(@(power) nchoosek(degree,power),powers) ...
                        .*fraction.^powers.*(1-fraction).^(degree-powers);
                    time=tube.start-(stage-1)*model.sampleTime+fraction*tube.duration;
                    actual=expm(time*generator)*[initial;repmat(inputs(:,stage),1,64);ones(1,64)];
                    residual=abs(actual(1:6,:)-points*basis.')-tube.radius*basis.';
                    worst=max(worst,max(residual,[],'all'));
                end
            end
            actual=expm(model.sampleTime*generator)*[initial;repmat(inputs(:,stage),1,64);ones(1,64)];
            initial=actual(1:6,:);
        end
    end
end

function [problem,residual] = localCurvedAudit(curvature)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',2, ...
        'controller',struct('sampleTime',.1,'horizonSteps',8,'stationTrustRadius',5,'executionPolicy','backup'), ...
        'model',struct('lateralDomainRadius',3)));
    curve=struct('origin',[0;0],'heading',0,'curvature',curvature,'length',300);
    [state,input]=ltvBicycleModel.cruiseEquilibrium(curvature,cfg);
    [position,heading]=laneGeometry.referencePose(20,.02,curve);
    ego=struct('position',position,'yaw',heading+state(3),'speed',state(4), ...
        'lateralVelocity',state(5),'yawRate',state(6),'heldActuatorInput',input,'stateTime',0, ...
        'perception',struct('time',0,'range',30,'completeWithinRange',true));
    road=struct('centerline',laneGeometry.referencePose(0:2:300,0,curve).','referenceCurve',curve);
    [command,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
    metadata=problem.metadata;
    initial=problem.model.initialEgoState;
    next=expm(.1*[metadata.executedContinuousGenerator;zeros(3,9)])*[initial;command.actuatorInput;1];
    before=initial(2:6)-metadata.clfReferenceState;
    after=next(2:6)-metadata.clfReferenceState;
    residual=after.'*metadata.clfMatrix*after ...
        -(1-metadata.clfDissipation.decayPerHold)*before.'*metadata.clfMatrix*before ...
        -metadata.clfDissipation.disturbanceBound;
end
