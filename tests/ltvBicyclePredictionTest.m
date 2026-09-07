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
        function brakingRatioUsesThePaperLongitudinalForceScale(testCase)
            cfg = collisionAvoidanceControllerConfig(struct( ...
                "roadLoad", struct("dragCoefficient", 0, "rollingCoefficient", 0)));
            [stateMatrix, inputMatrix, affine] = ltvBicycleModel.stageMatrices( ...
                0.0, 10.0, 0.05, cfg);
            next = stateMatrix*[0.0; 0.0; 0.0; 10.0; 0.0; 0.0] ...
                + inputMatrix*[0.0; -0.4]+affine;
            testCase.verifyEqual(next(4), 10-0.4*modifiedFialaTire.accelerationGain(cfg)*0.05, AbsTol=1.0e-13);
            testCase.verifyEqual(next(1), 0.5-0.2*modifiedFialaTire.accelerationGain(cfg)*0.05^2, AbsTol=1.0e-13);
        end

        function stiffLateralDynamicsRemainStableUnderHeldInputIntegration(testCase)
            cfg = collisionAvoidanceControllerConfig(struct( ...
                "model", struct("scheduleSpeedFloor", 0.5)));
            stateMatrix = ltvBicycleModel.stageMatrices(0.0, 0.5, 0.05, cfg);
            testCase.verifyTrue(all(isfinite(stateMatrix), "all"));
            testCase.verifyLessThan(max(abs(eig(stateMatrix(5:6, 5:6)))), 1.0);
        end

        function heldInputPredictionIsInvariantToIntegrationSubdivision(testCase)
            model = localModel();
            [fullA, fullB, fullC] = ltvBicycleModel.stageMatrices( ...
                0.0025, 15.0, model.sampleTime, model.cfg);
            [halfA, halfB, halfC] = ltvBicycleModel.stageMatrices( ...
                0.0025, 15.0, 0.5*model.sampleTime, model.cfg);
            testCase.verifyEqual(fullA, halfA*halfA, AbsTol=1.0e-13);
            testCase.verifyEqual(fullB, halfA*halfB+halfB, AbsTol=1.0e-13);
            testCase.verifyEqual(fullC, halfA*halfC+halfC, AbsTol=1.0e-13);
        end

        function aHeldAccelerationBiasAlsoChangesPosition(testCase)
            model = localModel();
            model.longitudinalAccelerationBias = 0.7;
            prediction = ltvBicycleModel.predict(model);
            state = prediction.egoStateOffset(:, 2);
            testCase.verifyEqual(state(4), 4.0+0.7*model.sampleTime, AbsTol=1.0e-13);
            testCase.verifyEqual(state(1), 4.0*model.sampleTime ...
                + 0.5*0.7*model.sampleTime^2, AbsTol=1.0e-13);
        end

        function errorBoundsContinuePropagatingBeyondTheFirstStep(testCase)
            model = localModel();
            model.measuredEgoStateErrorBound = [0.01; 0.02; 0.003; 0.04; 0.02; 0.005];
            model.cfg.model.ltvModelErrorRateBound = 0.001*ones(6, 1);

            prediction = ltvBicycleModel.predict(model);

            expectedThird = abs(prediction.stageMatrixA(:, :, 2)) ...
                * prediction.egoStateErrorBound(:, 2) ...
                + prediction.stageDisturbanceErrorBound(:, 2);
            testCase.verifyEqual(prediction.egoStateErrorBound(:, 3), ...
                expectedThird, AbsTol=1.0e-13);
            testCase.verifyGreaterThan(prediction.egoStateErrorBound(1, end), ...
                prediction.egoStateErrorBound(1, 3));
            testCase.verifyGreaterThan(prediction.egoStateErrorBound(2, end), ...
                prediction.egoStateErrorBound(2, 3));
        end

        function accelerationDisturbanceAlsoEnlargesPositionAtEveryStep(testCase)
            model = localModel();
            model.cfg.model.ltvModelErrorRateBound(4) = 0.1;
            prediction = ltvBicycleModel.predict(model);
            time = (0:2)*model.sampleTime;

            testCase.verifyEqual(prediction.egoStateErrorBound(1, 1:3), ...
                0.5*0.1*time.^2, AbsTol=1e-13);
            testCase.verifyEqual(prediction.egoStateErrorBound(4, 1:3), ...
                0.1*time, AbsTol=1e-13);
        end

        function shiftingTheSchedulePreservesTheExistingFialaFlows(testCase)
            model = localModel();
            model.lane.segmentCurvature(:) = 0.02;
            model.longitudinalAccelerationBias = 0.2;
            previous = ltvBicycleModel.predict(model);
            schedule = previous.scheduleForStore;
            schedule.speedProfile = [schedule.speedProfile(2:end), 0.0];
            schedule.station = [schedule.station(2:end), schedule.station(end)];
            schedule.curvature = [schedule.curvature(2:end), schedule.curvature(end)];
            shifted = ltvBicycleModel.predict(model, schedule);
            testCase.verifyEqual(shifted.stageMatrixA(:, :, 1:end-1), ...
                previous.stageMatrixA(:, :, 2:end), AbsTol=1e-13);
            testCase.verifyEqual(shifted.stageMatrixB(:, :, 1:end-1), ...
                previous.stageMatrixB(:, :, 2:end), AbsTol=1e-13);
            testCase.verifyEqual(shifted.stageAffine(:, 1:end-1), ...
                previous.stageAffine(:, 2:end), AbsTol=1e-13);
        end

        function brakingContinuationUsesTheSameHeldInputFlowAsTheHead(testCase)
            model = localModel();
            prediction = ltvBicycleModel.predict(model);
            state = squeeze(pagemtimes(prediction.egoStateMatrix, prediction.referencePlan)) ...
                + prediction.egoStateOffset;
            node = prediction.headNodeCount;

            increment = state(1, node+1)-state(1, node);

            acceleration = modifiedFialaTire.accelerationGain(model.cfg)*prediction.referencePlan(2*node);
            expected = model.sampleTime*state(4, node) ...
                + 0.5*model.sampleTime^2*acceleration;
            testCase.verifyEqual(increment, expected, AbsTol=1.0e-13);
            testCase.verifyLessThan(state(4, node+1), state(4, node));
            testCase.verifyEqual(prediction.stageMatrixB(1, :, node), [0.0, 0.5*model.sampleTime^2*modifiedFialaTire.accelerationGain(model.cfg)], AbsTol=1.0e-15);
        end
    end
end

function model = localModel()
    % Isolate held-flow and disturbance integration in the zero-road-load limit.
    cfg = collisionAvoidanceControllerConfig(struct("controller", struct("horizonSteps", 2), ...
        "roadLoad", struct("dragCoefficient", 0, "rollingCoefficient", 0)));
    ego = struct("positionX", 0.0, "positionY", 0.0, "yawAngle", 0.0, ...
        "longitudinalVelocity", 4.0, "lateralVelocity", 0.0, "yawRate", 0.0);
    [~, lane] = readPlanningInputs(ego, [], [0.0, 0.0; 2000.0, 0.0], cfg);
    model = struct("cfg", cfg, "horizonSteps", 2, "tailSteps", ltvBicycleModel.brakingSchedule("steps", cfg), ...
        "inputDimension", 2, "sampleTime", cfg.controller.sampleTime, ...
        "initialEgoState", [0.0; 0.0; 0.0; 4.0; 0.0; 0.0], ...
        "measuredEgoStateErrorBound", zeros(6, 1), ...
        "longitudinalAccelerationBias", 0.0, "lane", lane);
end
