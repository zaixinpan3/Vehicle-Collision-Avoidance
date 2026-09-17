classdef scheduledTerminalContinuationTest < matlab.unittest.TestCase
    properties (TestParameter)
        profile = struct('sBend',"sBend",'transition',"transition");
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function eachPhaseAndThePermanentTailContainTheirSuccessors(testCase,profile)
            model=localFixture(profile);
            bank=ltvBicycleModel.referenceSchedule(model);
            minimumMargin=Inf;maximumDrift=0;
            for index=1:bank.tailIndex+1
                model.referencePhaseIndex=index;
                terminal=hardEncounterBarrier.terminalCertificate(model);
                model.referencePhaseIndex=index+1;
                successor=hardEncounterBarrier.terminalCertificate(model);
                flow=terminal.cruise.transition;
                closed=flow(1:6,1:6)+flow(1:6,7:8)*terminal.feedback;
                drift=flow(1:6,:)*[terminal.reference;terminal.input;1]-successor.reference;
                charge=abs(successor.modalMatrix*closed*terminal.modalBasis)*terminal.radius ...
                    +abs(successor.modalMatrix*drift);
                minimumMargin=min(minimumMargin,min(successor.radius-charge));
                maximumDrift=max(maximumDrift,norm(drift,inf));
            end
            testCase.verifyGreaterThan(minimumMargin,0);
            testCase.verifyGreaterThan(maximumDrift,1e-5);
            testCase.verifyEqual(successor.index,bank.tailIndex+2);
        end

        function boundaryStatesRespectHeldPhaseAndFiniteActuatorRates(testCase,profile)
            model=localFixture(profile);
            bank=ltvBicycleModel.referenceSchedule(model);
            indices=unique([1,round(bank.tailIndex/2),bank.tailIndex,bank.tailIndex+20]);
            lower=[-model.cfg.model.frontWheelSteeringAngleMaximum;model.cfg.actuation.brakingRatioMinimum];
            upper=[model.cfg.model.frontWheelSteeringAngleMaximum;model.cfg.actuation.brakingRatioMaximum];
            rate=model.sampleTime*[model.cfg.model.frontWheelSteeringRateMaximum; ...
                model.cfg.model.brakingRatioRateMaximum];
            phaseMargin=Inf;inputMargin=Inf;slewMargin=Inf;successorMargin=Inf;regularity=Inf;
            for index=indices
                model.referencePhaseIndex=index;
                terminal=hardEncounterBarrier.terminalCertificate(model);
                model.referencePhaseIndex=index+1;
                successor=hardEncounterBarrier.terminalCertificate(model);
                errors=localBoundaryErrors(terminal);
                states=terminal.reference+errors;
                inputs=terminal.input+terminal.feedback*errors;
                inputMargin=min(inputMargin,min([upper-inputs;inputs-lower],[],'all'));
                generator=[terminal.continuousA,terminal.continuousB,terminal.continuousC;zeros(3,9)];
                for fraction=[0,.25,.5,.75,1]
                    flow=expm(fraction*model.sampleTime*generator);
                    held=flow(1:6,:)*[states;inputs;ones(1,size(states,2))];
                    phase=terminal.reference(1)+fraction*(successor.reference(1)-terminal.reference(1));
                    phaseMargin=min(phaseMargin,terminal.phaseRadius-max(abs(held(1,:)-phase)));
                    curvature=laneGeometry.referenceCurvature(held(1,:),model.lane.referenceCurve);
                    regularity=min(regularity,min(1-curvature.*held(2,:)));
                end
                nextInputs=successor.input+successor.feedback*(held-successor.reference);
                slewMargin=min(slewMargin,min(rate-abs(nextInputs-inputs),[],'all'));
                for stateIndex=1:size(held,2)
                    [~,margins]=hardEncounterBarrier.terminalMembership(successor,held(:,stateIndex),zeros(6,1));
                    successorMargin=min(successorMargin,min(margins));
                end
            end
            testCase.verifyGreaterThanOrEqual(phaseMargin,0);
            testCase.verifyGreaterThanOrEqual(inputMargin,0);
            testCase.verifyGreaterThanOrEqual(slewMargin,0);
            testCase.verifyGreaterThanOrEqual(successorMargin,0);
            testCase.verifyGreaterThanOrEqual(regularity,.2);
        end

        function anExactEndpointStartsAConstantPermanentContinuation(testCase)
            model=localFixture("transition");
            model.cfg.referenceSpeed=10;
            curve=model.lane.referenceCurve;
            curve.length=2;curve.curvatureProfile=[0,0;1,0;2,.01];
            model.lane.referenceCurve=curve;
            [~,incomingDerivative]=laneGeometry.referenceCurvature(2-1e-8,curve);
            testCase.verifyGreaterThan(abs(incomingDerivative),.005);
            bank=ltvBicycleModel.referenceSchedule(model);
            tail=ltvBicycleModel.referenceAt(model,bank.tailIndex);
            later=ltvBicycleModel.referenceAt(model,bank.tailIndex+137);
            testCase.verifyEqual(tail.state(1),curve.length,AbsTol=0);
            testCase.verifyEqual(tail.stage.continuousA(3,1),0,AbsTol=0);
            testCase.verifyEqual(tail.matrix,tail.nextMatrix,AbsTol=0);
            testCase.verifyEqual(later.stage,tail.stage);
            tailDefect=tail.transition(1:6,:)*[tail.state;tail.input;1]-tail.nextState;
            laterDefect=later.transition(1:6,:)*[later.state;later.input;1]-later.nextState;
            testCase.verifyEqual(laterDefect,tailDefect,AbsTol=1e-10);
            testCase.verifyEqual(later.referenceDefect,tail.referenceDefect,AbsTol=0);
        end
    end
end

function model=localFixture(profile)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('sampleTime',.1), ...
        'model',struct('frontWheelSteeringRateMaximum',.5,'brakingRatioRateMaximum',1)));
    station=(0:2:40).';fraction=station/40;
    if profile=="sBend"
        curvature=.004*sin(2*pi*fraction);
    else
        curvature=.01*(10*fraction.^3-15*fraction.^4+6*fraction.^5);
    end
    curve=struct('origin',[0;0],'heading',0,'curvature',0,'length',40, ...
        'curvatureProfile',[station,curvature],'continuation',"constantCurvature");
    model=struct('cfg',cfg,'sampleTime',.1,'lane',struct('referenceCurve',curve), ...
        'initialEgoState',zeros(6,1),'stateTime',0,'longitudinalAccelerationBias',0, ...
        'measurementRadiusLimit',zeros(6,1),'road',struct('boundaries',[]));
end

function errors=localBoundaryErrors(terminal)
    directions=[real(terminal.modalBasis),imag(terminal.modalBasis)];
    directions=directions(:,vecnorm(directions)>1e-12);
    scale=max(abs(terminal.modalMatrix*directions)./terminal.radius,[],1);
    errors=directions./scale;
    errors=[errors,-errors];
end
