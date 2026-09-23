classdef feedbackPredictionTest < matlab.unittest.TestCase
    %feedbackPredictionTest Feedback-policy prediction of the ego deviation set.
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (TestMethodSetup)
        function resetController(~)
            collisionAvoidanceController("resetNominalTrajectory");
        end
    end
    methods (Test)
        function sampledFeedbackTrajectoriesStayInsideThePredictedSets(testCase)
            [ego,road,cfg]=localCruise();
            [~,plan,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [stateExcess,inputExcess,slewExcess]=localSampledExcess(problem,plan,20260923);
            testCase.verifyTrue(any(problem.prediction.feedbackGain(:)));
            testCase.verifyLessThanOrEqual(stateExcess,1e-9);
            testCase.verifyLessThanOrEqual(inputExcess,1e-9);
            testCase.verifyLessThanOrEqual(slewExcess,1e-9);
        end

        function feedbackKeepsTheDeviationSetBounded(testCase)
            [ego,road,cfg]=localCruise();
            [~,~,withFeedback]=collisionAvoidanceController(ego,[],road,cfg,[]);
            cfg.feedbackPrediction.enabled=false;
            [~,~,openLoop]=collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyEqual(openLoop.prediction.feedbackGain,zeros(2,6));
            testCase.verifyEqual(openLoop.prediction.feedbackInputSupport, ...
                zeros(size(openLoop.prediction.feedbackInputSupport)));
            steps=min(withFeedback.prediction.stageCount,openLoop.prediction.stageCount);
            testCase.verifyLessThan(withFeedback.prediction.initialErrorBound(2,steps+1), ...
                openLoop.prediction.initialErrorBound(2,steps+1));
            % The first held input is exact, so node 1 is the same image of the
            % initial box (up to the horizon-dependent linearization anchor).
            testCase.verifyEqual(withFeedback.prediction.initialErrorBound(:,2), ...
                openLoop.prediction.initialErrorBound(:,2),RelTol=1e-6);
        end

        function aFreshFrameIssuesItsPlannedFirstInput(testCase)
            [ego,target,road,cfg]=localEncounter();
            [command,plan,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyEqual(problem.metadata.feedbackCorrection,zeros(2,1));
            testCase.verifyEqual(command.actuatorInput,plan(:,1),AbsTol=0);
        end

        function anInheritedFrameIssuesThePlanPlusItsFeedbackCorrection(testCase)
            [ego,target,road,cfg]=localEncounter();
            [~,~,first,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
            next=localDisplacedSuccessor(ego,stored,first.model.lane);
            target.targetPositionInertial=target.targetPositionInertial ...
                +cfg.controller.sampleTime*target.targetVelocityInertial;
            [command,plan,problem,nextStored]=collisionAvoidanceController(next,target,road,cfg,stored);
            correction=problem.prediction.feedbackGain ...
                *(problem.model.initialEgoState-problem.prediction.nominalInitialState);
            testCase.verifyTrue(problem.metadata.inheritedFeasibleFamily);
            testCase.verifyGreaterThan(norm(correction),0);
            testCase.verifyEqual(problem.metadata.feedbackCorrection,correction,AbsTol=1e-15);
            testCase.verifyEqual(command.actuatorInput,plan(:,1)+problem.metadata.feedbackCorrection,AbsTol=0);
            testCase.verifyEqual(nextStored.appliedInput,command.actuatorInput,AbsTol=0);
            third=encounterTestFixture.nextEgo(nextStored,problem.model.lane);
            third.controllerStateErrorBound=ego.controllerStateErrorBound;
            target.targetPositionInertial=target.targetPositionInertial ...
                +cfg.controller.sampleTime*target.targetVelocityInertial;
            [~,~,following]=collisionAvoidanceController(third,target,road,cfg,nextStored);
            testCase.verifyTrue(following.metadata.inheritedFeasibleFamily);
        end

        function aTamperedFeedbackCorrectionIsRejected(testCase)
            [ego,target,road,cfg]=localEncounter();
            [~,~,first,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
            stored.feedbackCorrection=stored.feedbackCorrection+[1e-3;0];
            next=encounterTestFixture.nextEgo(stored,first.model.lane);
            next.controllerStateErrorBound=ego.controllerStateErrorBound;
            testCase.verifyError(@()collisionAvoidanceController(next,target,road,cfg,stored), ...
                'collisionAvoidanceController:invalidStoredCertificate');
        end

        function aLargerEstimatorBoundVoidsTheCarriedPrediction(testCase)
            [ego,target,road,cfg]=localEncounter();
            [~,~,first,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
            next=encounterTestFixture.nextEgo(stored,first.model.lane);
            next.controllerStateErrorBound=1.5*ego.controllerStateErrorBound;
            target.targetPositionInertial=target.targetPositionInertial ...
                +cfg.controller.sampleTime*target.targetVelocityInertial;
            [~,~,problem]=collisionAvoidanceController(next,target,road,cfg,stored);
            testCase.verifyTrue(problem.metadata.measurementContractChanged);
            testCase.verifyFalse(problem.metadata.inheritedFeasibleFamily);
        end

        function invalidFeedbackSettingsAreRejected(testCase)
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('feedbackPrediction', ...
                struct('speedGainScale',-1))),'MATLAB:expectedNonnegative');
        end
    end
end

function [ego,road,cfg]=localCruise()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'controller', ...
        struct('sampleTime',.05,'horizonSteps',32,'minimumHorizonSteps',1)));
    ego=struct('position',[0;.05],'yaw',.01,'speed',8,'stateTime',0, ...
        'controllerStateErrorBound',[.076;.076;.048;.089;.497;.0015], ...
        'perception',struct('time',0,'range',16,'completeWithinRange',true));
    road=[-100,0;2000,0];
end

function [ego,target,road,cfg]=localEncounter()
    [ego,target,road,cfg]=encounterTestFixture.crossing();
    cfg.solver.frameDeadlineSeconds=60;cfg.solver.certificateSearchTimeLimit=60;
    ego.controllerStateErrorBound=[.02;.02;.004;.02;.05;.0015];
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

function [stateExcess,inputExcess,slewExcess]=localSampledExcess(problem,plan,seed)
% Simulate the declared plant under u_k = v_k + K (xhat_k - z_k) with random
% initial and per-hold estimator errors (including box vertices).
    prediction=problem.prediction;gain=prediction.feedbackGain;
    epsilon=prediction.estimatorBound;radius=problem.model.initialFrenetErrorBound;
    count=prediction.stageCount;nominal=zeros(6,count+1);
    for node=0:count
        nominal(:,node+1)=prediction.egoStateOffset(:,node+1)+prediction.egoStateMatrix(:,:,node+1)*plan(:);
    end
    stream=RandStream('mt19937ar',Seed=seed);
    stateExcess=-Inf;inputExcess=-Inf;slewExcess=-Inf;
    for trial=1:200
        vertex=mod(trial,2)==0;
        x=nominal(:,1)+radius.*localUnit(stream,vertex);
        previous=zeros(2,1);
        for stage=1:count
            deviation=zeros(2,1);
            if stage>1
                estimate=x+epsilon.*localUnit(stream,vertex);
                deviation=gain*(estimate-nominal(:,stage));
            end
            inputExcess=max(inputExcess,max(abs(deviation)-prediction.feedbackInputSupport(:,stage)));
            slewExcess=max(slewExcess,max(abs(deviation-previous)-prediction.feedbackSlewSupport(:,stage)));
            previous=deviation;
            x=prediction.stageMatrixA(:,:,stage)*x ...
                +prediction.stageMatrixB(:,:,stage)*(plan(:,stage)+deviation)+prediction.stageAffine(:,stage);
            offset=x-nominal(:,stage+1);generators=prediction.cells(stage).generators;
            direction=randn(stream,6,1);
            stateExcess=max([stateExcess;abs(offset)-sum(abs(generators),2); ...
                direction.'*offset-sum(abs(direction.'*generators))]);
        end
    end
end

function value=localUnit(stream,vertex)
    if vertex
        value=sign(randn(stream,6,1));
    else
        value=2*rand(stream,6,1)-1;
    end
end
