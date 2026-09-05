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
        function commandedAccelerationUsesTheDeclaredInputGain(testCase)
            cfg = collisionAvoidanceControllerConfig(struct( ...
                "model", struct("longitudinalInputGain", 0.8)));
            [stateMatrix, inputMatrix, affine] = ltvBicycleStageMatrices( ...
                0.0, 10.0, 0.05, cfg);
            next = stateMatrix*[0.0; 0.0; 0.0; 10.0; 0.0; 0.0] ...
                + inputMatrix*[0.0; -4.0]+affine;
            testCase.verifyEqual(next(4), 9.84, AbsTol=1.0e-13);
            testCase.verifyEqual(next(1), 0.496, AbsTol=1.0e-13);
        end

        function stiffLateralDynamicsRemainStableUnderHeldInputIntegration(testCase)
            cfg = collisionAvoidanceControllerConfig(struct( ...
                "model", struct("scheduleSpeedFloor", 0.5)));
            stateMatrix = ltvBicycleStageMatrices(0.0, 0.5, 0.05, cfg);
            testCase.verifyTrue(all(isfinite(stateMatrix), "all"));
            testCase.verifyLessThan(max(abs(eig(stateMatrix(5:6, 5:6)))), 1.0);
        end

        function heldInputPredictionIsInvariantToIntegrationSubdivision(testCase)
            model = localModel();
            [fullA, fullB, fullC] = ltvBicycleStageMatrices( ...
                0.0025, 15.0, model.sampleTime, model.cfg);
            [halfA, halfB, halfC] = ltvBicycleStageMatrices( ...
                0.0025, 15.0, 0.5*model.sampleTime, model.cfg);
            testCase.verifyEqual(fullA, halfA*halfA, AbsTol=1.0e-13);
            testCase.verifyEqual(fullB, halfA*halfB+halfB, AbsTol=1.0e-13);
            testCase.verifyEqual(fullC, halfA*halfC+halfC, AbsTol=1.0e-13);
        end

        function aHeldAccelerationBiasAlsoChangesPosition(testCase)
            model = localModel();
            model.cfg.model.longitudinalInputGain = 0.8;
            model.longitudinalAccelerationBias = 0.7;
            prediction = ltvBicyclePrediction(model);
            state = prediction.egoStateOffset(:, 2);
            testCase.verifyEqual(state(4), 4.0+0.7*model.sampleTime, AbsTol=1.0e-13);
            testCase.verifyEqual(state(1), 4.0*model.sampleTime ...
                + 0.5*0.7*model.sampleTime^2, AbsTol=1.0e-13);
        end

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

        function brakingContinuationUsesTheSameHeldInputFlowAsTheHead(testCase)
            model = localModel();
            prediction = ltvBicyclePrediction(model);
            state = squeeze(pagemtimes(prediction.egoStateMatrix, prediction.referencePlan)) ...
                + prediction.egoStateOffset;
            node = prediction.headNodeCount;

            increment = state(1, node+1)-state(1, node);

            acceleration = prediction.referencePlan(2*node);
            expected = model.sampleTime*state(4, node) ...
                + 0.5*model.sampleTime^2*acceleration;
            testCase.verifyEqual(increment, expected, AbsTol=1.0e-13);
            testCase.verifyLessThan(state(4, node+1), state(4, node));
            testCase.verifyEqual(prediction.stageMatrixB(1, :, node), [0.0, 0.5*model.sampleTime^2], AbsTol=1.0e-15);
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
