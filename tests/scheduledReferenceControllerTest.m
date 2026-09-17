classdef scheduledReferenceControllerTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'solver','bicycle')));
        end
    end
    methods (Test)
        function predictedGeneratorsFollowTheChangingReference(testCase)
            [~,~,~,model]=localFixture();
            early=ltvBicycleModel.referenceAt(model,1);bend=ltvBicycleModel.referenceAt(model,25);
            testCase.verifyNotEqual(early.stage.curvature,bend.stage.curvature);
            testCase.verifyNotEqual(early.stage.continuousA,bend.stage.continuousA);
            testCase.verifyGreaterThan(abs(bend.stage.continuousA(3,1)),0);
            testCase.verifyGreaterThan(norm(bend.referenceDefect),0);
        end

        function theScheduledClfContractsIntoTheNextMatrix(testCase)
            [~,~,~,model]=localFixture();
            [maximumViolation,matrixVariation]=localCrossStepContraction(model);
            testCase.verifyLessThanOrEqual(maximumViolation,1e-9);
            testCase.verifyGreaterThan(matrixVariation,0);
        end

        function theConstantContinuationUsesTheDeclaredEndCurvature(testCase)
            [~,~,~,model]=localFixture();bank=ltvBicycleModel.referenceSchedule(model);
            first=ltvBicycleModel.referenceAt(model,bank.tailIndex);
            later=ltvBicycleModel.referenceAt(model,bank.tailIndex+20);
            testCase.verifyEqual(first.stage.curvature,0,AbsTol=0);
            testCase.verifyEqual(first.stage.continuousA,later.stage.continuousA,AbsTol=0);
            testCase.verifyEqual(later.state(1)-first.state(1),20*bank.stepStation,AbsTol=1e-10);
        end

        function theCarriedGeneratorMatchesTheNextReferencePhase(testCase)
            [ego,road,cfg,model]=localFixture();
            [command,~,first,stored]=collisionAvoidanceController(ego,[],road,cfg,[]);
            state=localSuccessor(first.model.initialEgoState,command,first.metadata,cfg.controller.sampleTime);
            ego=localEgo(state,cfg.controller.sampleTime,command.actuatorInput,model.lane);
            [~,~,second]=collisionAvoidanceController(ego,[],road,cfg,stored);
            expected=ltvBicycleModel.referenceAt(model,2);
            testCase.verifyTrue(second.metadata.planCertified);
            testCase.verifyTrue(second.metadata.carriedWitnessAvailable);
            testCase.verifyEqual(second.metadata.executedContinuousGenerator, ...
                [expected.stage.continuousA,expected.stage.continuousB,expected.stage.continuousC],AbsTol=1e-12);
        end

        function aPhaseDelayCannotBeHiddenByReschedulingThePlant(testCase)
            [~,road,cfg,model]=localFixture();time=2;
            reference=ltvBicycleModel.referenceAt(model,round(time/cfg.controller.sampleTime)+1);
            state=reference.state;state(1)=state(1)-cfg.encounter.referencePhaseRadius-1;
            ego=localEgo(state,time,reference.input,model.lane);
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:referencePhaseOutsideDomain');
        end

        function nominalMotionTracksAGentlyChangingReference(testCase)
            [ego,road,cfg,model]=localFixture();
            [errors,certified,curvatures]=localShortRun(ego,road,cfg,model,8);
            testCase.verifyTrue(all(certified));
            testCase.verifyLessThan(max(abs(errors(1,:))),.15);
            testCase.verifyLessThan(max(abs(errors(2,:))),.03);
            testCase.verifyLessThan(max(abs(errors(3,:))),.15);
            testCase.verifyGreaterThan(max(curvatures)-min(curvatures),1e-4);
        end

        function sparsePhaseRowsAndScheduledCostsPreserveTheFullProgram(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [rowError,costError]=localRealizationErrors(problem.program);
            testCase.verifyLessThan(rowError,cfg.solver.constraintTolerance);
            testCase.verifyLessThan(costError,1e-7);
        end

        function nonlinearDiagnosticsRetainSpatialCurvatureSensitivity(testCase)
            [~,~,~,model]=localFixture();
            reference=ltvBicycleModel.referenceAt(model,20);model.initialEgoState=reference.state;
            [~,a]=ltvBicycleModel.nominalRollout(model,reference.input);
            testCase.verifyGreaterThan(abs(a(3,1)),1e-5);
            testCase.verifyTrue(all(isfinite(a),'all'));
        end
    end
end

function [rowError,costError]=localRealizationErrors(program)
    lifted=avoidanceStageQp.build(program);n=program.prediction.planCount;
    cost=zeros(2,2);rowError=0;
    for k=1:2
        decision=zeros(numel(program.q),1);
        decision(1:numel(program.anchorPlan))=program.anchorPlan;
        decision=decision+.01*k*sin((1:numel(program.q)).');
        prediction=program.prediction;
        states=prediction.egoStateOffset+reshape(pagemtimes(prediction.egoStateMatrix,decision(1:n)),6,[]);
        delta=states(:,2:end)-lifted.stateCenter(:,2:end);augmented=[decision;delta(:)];
        residual=lifted.b-lifted.A*augmented;
        expected=program.b(lifted.retainedRows)-program.A(lifted.retainedRows,:)*decision;
        difference=[residual(1:lifted.cones(1));residual(lifted.cones(1)+1:end)-expected];
        scale=1+abs(lifted.b)+abs(lifted.A)*abs(augmented);
        scale(lifted.cones(1)+1:end)=scale(lifted.cones(1)+1:end) ...
            +abs(program.b(lifted.retainedRows))+abs(program.A(lifted.retainedRows,:))*abs(decision);
        rowError=max(rowError,max(abs(difference)./scale));
        cost(k,:)=[.5*decision.'*program.P*decision+program.q.'*decision, ...
            .5*augmented.'*lifted.P*augmented+lifted.q.'*augmented];
    end
    costError=abs(diff(cost(:,1))-diff(cost(:,2)));
end

function [ego,road,cfg,model]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('sampleTime',.1,'minimumHorizonSteps',1), ...
        'solver',struct('frameDeadlineSeconds',30,'certificateSearchTimeLimit',30)));
    curve=struct('origin',[0;0],'heading',0,'curvature',0,'length',120, ...
        'curvatureProfile',[0,0;30,.015;60,0;90,-.015;120,0], ...
        'continuation','constantCurvature');
    road=struct('referenceCurve',curve);
    [state,input]=ltvBicycleModel.cruiseEquilibrium(0,cfg);
    ego=localEgo(state,0,input,road);
    [~,lane,parsedRoad]=readPlanningInputs(ego,[],road,cfg);
    model=struct('cfg',cfg,'lane',lane,'road',parsedRoad,'sampleTime',cfg.controller.sampleTime, ...
        'stateTime',0,'initialEgoState',state,'initialFrenetErrorBound',zeros(6,1), ...
        'longitudinalAccelerationBias',0,'previousInput',input,'referenceSpeed',cfg.referenceSpeed);
    ltvBicycleModel.referenceSchedule(model);
end

function [maximumViolation,variation]=localCrossStepContraction(model)
    maximumViolation=-inf;variation=0;
    for index=[1,20,60,120]
        certificate=ltvBicycleModel.referenceAt(model,index);
        closed=certificate.transition(2:6,2:6)-certificate.transition(2:6,7:8)*certificate.gain;
        residual=closed.'*certificate.nextMatrix*closed-certificate.contraction*certificate.matrix;
        maximumViolation=max(maximumViolation,max(eig((residual+residual.')/2)));
        variation=max(variation,norm(certificate.nextMatrix-certificate.matrix,'fro'));
    end
end

function [errors,certified,curvatures]=localShortRun(ego,road,cfg,model,count)
    errors=zeros(5,count);certified=false(1,count);curvatures=zeros(1,count);stored=[];
    state=model.initialEgoState;
    for sample=1:count
        [command,~,problem,stored]=collisionAvoidanceController(ego,[],road,cfg,stored);
        certified(sample)=problem.metadata.planCertified;curvatures(sample)=problem.metadata.clfOperatingCurvature;
        state=localSuccessor(state,command,problem.metadata,cfg.controller.sampleTime);
        kappa=laneGeometry.curvature(state(1),model.lane);trim=ltvBicycleModel.cruiseEquilibrium(kappa,cfg);
        errors(:,sample)=state(2:6)-trim(2:6);
        ego=localEgo(state,sample*cfg.controller.sampleTime,command.actuatorInput,model.lane);
    end
end

function ego=localEgo(state,time,input,lane)
    [position,heading]=laneGeometry.fromFrenet(state,lane);
    ego=struct('position',position,'yaw',heading,'speed',state(4),'lateralVelocity',state(5), ...
        'yawRate',state(6),'stateTime',time,'heldActuatorInput',input,'controllerStateErrorBound',zeros(6,1), ...
        'perception',struct('time',time,'range',16,'completeWithinRange',true));
end

function state=localSuccessor(state,command,metadata,duration)
    generator=[metadata.executedContinuousGenerator;zeros(3,9)];
    augmented=expm(duration*generator)*[state;command.actuatorInput;1];state=augmented(1:6);
end
