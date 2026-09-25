classdef targetReactionTest < matlab.unittest.TestCase
    %targetReactionTest Target-reactive feedback policy on an NRMM target.
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (TestMethodSetup)
        function resetController(~)
            collisionAvoidanceController("resetNominalTrajectory");
        end
    end
    methods (Test)
        function sampledReactiveTrajectoriesStayInsideTheJointTube(testCase)
            [ego,target,road,cfg]=encounterTestFixture.nrmmLead(30);
            [~,plan,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.assertEqual(problem.metadata.targetReactionStrength,30);
            testCase.verifyTrue(any(problem.prediction.targetGainSequence(:)) ...
                && any(problem.prediction.targetAccelerationGainSequence(:)));
            excess=nrmmTruthFixture.sampledExcess(problem.program,problem.model,plan,20260924);
            testCase.verifyLessThanOrEqual(excess.joint,1e-9);
            testCase.verifyLessThanOrEqual(excess.record,1e-9);
            testCase.verifyLessThanOrEqual(excess.input,1e-9);
            testCase.verifyLessThanOrEqual(excess.slew,1e-9);
        end

        function reactionShrinksTheRelativeSupportOfLateRecords(testCase)
            [ego,target,road,cfg]=encounterTestFixture.nrmmLead(30);
            [~,~,reactive]=collisionAvoidanceController(ego,target,road,cfg,[]);
            collisionAvoidanceController("resetNominalTrajectory");
            cfg.feedbackPrediction.targetReaction.inputWeightScales=Inf;
            [~,~,egoOnly]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyFalse(isfield(egoOnly.prediction,'targetGainSequence'));
            reactiveSupport=localRecordSupport(reactive.program);
            egoOnlySupport=localRecordSupport(egoOnly.program);
            testCase.verifyLessThan(reactiveSupport(end),egoOnlySupport(end));
        end

        function anInheritedFrameAlsoCorrectsByTheTargetDeviation(testCase)
            [ego,target,road,cfg]=encounterTestFixture.nrmmLead(30);
            [~,~,first,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.assertEqual(first.metadata.targetReactionStrength,30);
            next=localDisplacedSuccessor(ego,stored,first.model.lane);
            nextTarget=localNextTarget(target,first.prediction.targetNominal(:,2),[.03;-.02;.01;0;.005;0]);
            [command,plan,problem,nextStored]=collisionAvoidanceController(next,nextTarget,road,cfg,stored);
            testCase.verifyTrue(problem.metadata.inheritedFeasibleFamily);
            prediction=problem.prediction;
            deviation=problem.model.encounter.center(1:6)-prediction.targetNominal(:,1);
            correction=prediction.feedbackGainSequence(:,:,1) ...
                *(problem.model.initialEgoState-prediction.nominalInitialState) ...
                +prediction.targetGainSequence(:,:,1)*deviation(1:4) ...
                +prediction.targetAccelerationGainSequence(:,:,1)*deviation(5:6);
            testCase.verifyGreaterThan(norm(prediction.targetGainSequence(:,:,1)*deviation(1:4) ...
                +prediction.targetAccelerationGainSequence(:,:,1)*deviation(5:6)),0);
            testCase.verifyEqual(problem.metadata.feedbackCorrection,correction,AbsTol=1e-14);
            testCase.verifyEqual(command.actuatorInput,plan(:,1)+problem.metadata.feedbackCorrection,AbsTol=0);
            testCase.verifyEqual(nextStored.appliedInput,command.actuatorInput,AbsTol=0);
        end

        function aLargerTargetBoundVoidsTheReactiveFamily(testCase)
            [ego,target,road,cfg]=encounterTestFixture.nrmmLead([30,Inf]);
            [~,~,first,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
            next=localDisplacedSuccessor(ego,stored,first.model.lane);
            nextTarget=localNextTarget(target,first.prediction.targetNominal(:,2),zeros(6,1));
            nextTarget.targetVelocityInertialErrorBound=1.5*nextTarget.targetVelocityInertialErrorBound;
            [~,~,problem]=collisionAvoidanceController(next,nextTarget,road,cfg,stored);
            testCase.verifyFalse(problem.metadata.inheritedFeasibleFamily);
        end

        function aLargerAccelerationMaximumVoidsTheCarriedFamily(testCase)
            [ego,target,road,cfg]=encounterTestFixture.nrmmLead(Inf);
            target.predictionMotion.scalarAccelerationMaximum=2;
            [~,~,first,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
            next=localDisplacedSuccessor(ego,stored,first.model.lane);
            nominal=targetPrediction.finiteFlow(first.model.encounter,cfg.controller.sampleTime);
            nextTarget=localNextTarget(target,nominal(1:6),zeros(6,1));
            [~,~,same]=collisionAvoidanceController(next,nextTarget,road,cfg,stored);
            testCase.verifyTrue(same.metadata.inheritedFeasibleFamily);
            nextTarget.predictionMotion.scalarAccelerationMaximum=3;
            [~,~,problem]=collisionAvoidanceController(next,nextTarget,road,cfg,stored);
            testCase.verifyFalse(problem.metadata.inheritedFeasibleFamily);
        end

        function aReleaseAfterTheReactionContinuesWithoutACorrection(testCase)
            % Curved stationary encounter with 2 m initial lateral error: the
            % reactive admission's gains end before the target is released at
            % hold 64; the released, inherited frames carry no target term.
            report=runExactStateRecursiveFeasibilityScenario(Scenario="stationary",RoadCurvature=0.01, ...
                InitialTrackingError=[2;0;0;0;0],SampleCount=80,DeadlineSeconds=Inf,SearchTimeLimitSeconds=30, ...
                FeedbackPrediction=struct('targetReaction',struct('inputWeightScales',[30,100,Inf])));
            testCase.verifyTrue(report.completed);
            testCase.verifyTrue(any(report.confirmedRelease));
            testCase.verifyTrue(any(cellfun(@(search) isfield(search,'reactionAttempts') ...
                && ~isempty(search.reactionAttempts) && isfinite(search.reactionStrength),report.admissionSearch)));
        end
    end
end

function target=localNextTarget(target,nominal,offset)
% A measurement of the next hold: the carried nominal flow plus an offset.
    state=nominal+offset;
    target.targetPositionInertial=state(1:2);target.targetVelocityInertial=state(3:4);
    target.targetAccelerationInertial=state(5:6);
end

function ego=localDisplacedSuccessor(previous,stored,lane)
% A measurement displaced inside the stored successor box.
    state=stored.predictedState(:,2);radius=stored.stateErrorBound(:,2);
    state(2)=state(2)+.5*radius(2);state(3)=state(3)-.5*radius(3);
    [position,heading]=laneGeometry.fromFrenet(state,lane);
    ego=struct('position',position,'yaw',heading,'speed',state(4), ...
        'lateralVelocity',state(5),'yawRate',state(6), ...
        'stateTime',stored.stateTime+stored.identity.configuration.controller.sampleTime, ...
        'heldActuatorInput',stored.appliedInput, ...
        'controllerStateErrorBound',previous.controllerStateErrorBound, ...
        'perception',struct('time',stored.stateTime+stored.identity.configuration.controller.sampleTime, ...
        'range',stored.confirmation.range,'completeWithinRange',true));
end

function support=localRecordSupport(program)
    certificate=program.jointCertificate;support=zeros(numel(certificate.records),1);
    for index=1:numel(certificate.records)
        normal=[cos(certificate.angles(index));sin(certificate.angles(index))];
        support(index)=sum(abs(certificate.records(index).generators.'*normal));
    end
end
