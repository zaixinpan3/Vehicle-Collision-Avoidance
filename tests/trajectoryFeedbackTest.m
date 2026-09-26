classdef trajectoryFeedbackTest < matlab.unittest.TestCase
    %trajectoryFeedbackTest Error feedback for changing near-limit tire models.
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function nearLimitInputsRetainUsablePredictionAndActuatorBounds(testCase)
            model = localModel();
            prediction = hardEncounterBarrier.predict(model,model.cruiseCertificate);

            testCase.verifyLessThan(max(prediction.initialErrorBound(2,:)),.01);
            testCase.verifyLessThan(max(prediction.feedbackInputSupport(1,:)),.01);
            testCase.verifyGreaterThan(norm(prediction.feedbackGainSequence(:,:,2),'fro'),.01);
            testCase.verifyEqual(prediction.feedbackGainSequence(:,:,1),zeros(2,6));
            testCase.verifyGreaterThan(norm(prediction.feedbackGainSequence(:,:,2) ...
                -prediction.feedbackGainSequence(:,:,3),'fro'),.01);
        end

        function noisyHeldTrajectoriesStayInsideTheScheduledFeedbackTube(testCase)
            model = localModel();
            model.initialFrenetErrorBound = [1;1;.1;1;1;.1]*1e-5;
            prediction = hardEncounterBarrier.predict(model,model.cruiseCertificate);
            excess = localSampledExcess(model,prediction);

            testCase.verifyLessThanOrEqual(excess,1e-9);
            testCase.verifyGreaterThan(max(prediction.initialErrorBound(2,:)),1e-5);
        end

        function configuredFeedbackColumnsRemainDisabled(testCase)
            model = localModel();
            model.cfg.feedbackPrediction.speedGainScale = 0;
            prediction = hardEncounterBarrier.predict(model,model.cruiseCertificate);

            testCase.verifyEqual(prediction.feedbackGainSequence(:,[1,4,5],:), ...
                zeros(2,3,model.horizonSteps));
        end

        function disabledFeedbackReservesNoCorrectiveInput(testCase)
            model = localModel();model.cfg.feedbackPrediction.enabled = false;
            prediction = hardEncounterBarrier.predict(model,model.cruiseCertificate);

            testCase.verifyEqual(prediction.feedbackGainSequence,zeros(2,6,model.horizonSteps));
            testCase.verifyEqual(prediction.feedbackInputSupport,zeros(2,model.horizonSteps));
        end
    end
end

function model = localModel()
    cfg = collisionAvoidanceControllerConfig(struct('referenceSpeed',8));
    ego = struct('position',[0;0],'yaw',0,'speed',8);
    [~,lane,road] = readPlanningInputs(ego,[],[-100,0;1000,0],cfg);
    count = 64;
    inputs = [.1*ones(1,count);repmat([-.99998,.99998],1,count/2)];
    model = struct('cfg',cfg,'lane',lane,'road',road,'initialEgoState',[100;0;0;8;0;0], ...
        'initialFrenetErrorBound',zeros(6,1),'sampleTime',.05,'horizonSteps',count, ...
        'longitudinalAccelerationBias',0,'previousInput',zeros(2,1), ...
        'encounter',struct('key',1),'cruiseCertificate',[],'initializationPlan',inputs);
    model.cruiseCertificate = ltvBicycleModel.sampledCruise(model);
end

function excess = localSampledExcess(model,prediction)
% Propagate the declared held affine plant with sampled estimator errors.
    inputs = model.initializationPlan;
    nominal = prediction.egoStateOffset ...
        +reshape(pagemtimes(prediction.egoStateMatrix,inputs(:)),6,[]);
    stream = RandStream('mt19937ar',Seed=20260925);
    excess = -Inf;
    for trial = 1:100
        x = nominal(:,1)+model.initialFrenetErrorBound.*sign(randn(stream,6,1));
        previous = zeros(2,1);
        for stage = 1:prediction.stageCount
            estimate = x+prediction.estimatorBound.*sign(randn(stream,6,1));
            correction = prediction.feedbackGainSequence(:,:,stage)*(estimate-nominal(:,stage));
            excess = max([excess;abs(correction)-prediction.feedbackInputSupport(:,stage); ...
                abs(correction-previous)-prediction.feedbackSlewSupport(:,stage)]);
            x = prediction.stageMatrixA(:,:,stage)*x ...
                +prediction.stageMatrixB(:,:,stage)*(inputs(:,stage)+correction) ...
                +prediction.stageAffine(:,stage);
            deviation = x-nominal(:,stage+1);
            direction = randn(stream,6,1);
            excess = max([excess;abs(deviation)-prediction.initialErrorBound(:,stage+1); ...
                direction.'*deviation-sum(abs(direction.'*prediction.cells(stage).generators))]);
            previous = correction;
        end
    end
end
