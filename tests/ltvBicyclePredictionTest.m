classdef ltvBicyclePredictionTest < matlab.unittest.TestCase
    %ltvBicyclePredictionTest Continuation dynamics and error containment.

    methods (TestClassSetup)
        function addModelPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
        end
    end

    methods (Test)
        function errorBoundsContinuePropagatingBeyondTheFirstStep(testCase)
            model = localModel();
            model.measuredEgoStateErrorBound = [0.01; 0.02; 0.003; 0.04; 0.02; 0.005];
            model.cfg.model.ltvModelErrorRateBound = 0.001*ones(6, 1);

            prediction = ltvBicyclePrediction(model);

            expectedThird = abs(prediction.stageMatrixA(:, :, 2)) ...
                * prediction.egoStateErrorBound(:, 2) ...
                + model.sampleTime*model.cfg.model.ltvModelErrorRateBound;
            testCase.verifyEqual(prediction.egoStateErrorBound(:, 3), ...
                expectedThird, AbsTol=1.0e-13);
            testCase.verifyGreaterThan(prediction.egoStateErrorBound(1, end), ...
                prediction.egoStateErrorBound(1, 3));
            testCase.verifyGreaterThan(prediction.egoStateErrorBound(2, end), ...
                prediction.egoStateErrorBound(2, 3));
        end

        function brakingContinuationUsesTheExecutableEulerStationUpdate(testCase)
            model = localModel();
            prediction = ltvBicyclePrediction(model);
            state = squeeze(pagemtimes(prediction.egoStateMatrix, prediction.referencePlan)) ...
                + prediction.egoStateOffset;
            node = prediction.headNodeCount;

            increment = state(1, node+1)-state(1, node);

            testCase.verifyEqual(increment, model.sampleTime*state(4, node), AbsTol=1.0e-13);
            testCase.verifyLessThan(state(4, node+1), state(4, node));
            testCase.verifyEqual(prediction.stageMatrixB(1, :, node), [0.0, 0.0], AbsTol=0.0);
        end
    end
end

function model = localModel()
    cfg = collisionAvoidanceControllerConfig(struct("controller", struct("horizonSteps", 2)));
    ego = struct("positionX", 0.0, "positionY", 0.0, "yawAngle", 0.0, ...
        "longitudinalVelocity", 4.0, "lateralVelocity", 0.0, "yawRate", 0.0);
    [~, lane] = readPlanningInputs(ego, [], [0.0, 0.0; 2000.0, 0.0], cfg);
    model = struct("cfg", cfg, "horizonSteps", 2, "tailSteps", brakingSchedule("steps", cfg), ...
        "inputDimension", 2, "sampleTime", cfg.controller.sampleTime, ...
        "initialEgoState", [0.0; 0.0; 0.0; 4.0; 0.0; 0.0], ...
        "measuredEgoStateErrorBound", zeros(6, 1), ...
        "longitudinalAccelerationBias", 0.0, "lane", lane);
end
