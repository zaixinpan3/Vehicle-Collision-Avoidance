classdef robustStationaryPoseCertificateTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
        end
    end
    methods (Test)
        function aPoseBoxIsAdmittedWithoutChangingTheOptimization(testCase)
            [ego, cfg, lane] = localInputs();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, [], lane, cfg, []);

            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyFalse(problem.metadata.exactPredictionAssumptionsHold);
            testCase.verifyEqual(stored.safetyScope, "uncertainFiniteHorizon");
            testCase.verifyGreaterThan(stored.stateErrorBound(1:3, end), zeros(3, 1));
            testCase.verifyGreaterThanOrEqual(stored.stateErrorBound(4:6, end), zeros(3, 1));
            testCase.verifyEqual(problem.metadata.solverCallCount, 1);
        end

        function aNewEstimateCentersTheIntersectedCarriedBox(testCase)
            [ego, cfg, lane] = localInputs();
            [command, ~, ~, stored] = collisionAvoidanceController(ego, [], lane, cfg, []);
            fresh = localNextEgo(ego, stored, command);
            fresh.position(1) = fresh.position(1)+0.02;
            fresh.controllerStateErrorBound(1) = 0.03;
            [~, ~, problem, next] = collisionAvoidanceController(fresh, [], lane, cfg, stored);

            testCase.verifyTrue(problem.metadata.certificateCompatible);
            testCase.verifyTrue(problem.metadata.setMembershipUpdate);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyEqual(next.predictedState(1:2,1),fresh.position,AbsTol=1e-12);
            testCase.verifyEqual(next.stateErrorBound(1,1),0.03,AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual(next.predictedState(:,1)-next.stateErrorBound(:,1), ...
                stored.predictedState(:,2)-stored.stateErrorBound(:,2)-1e-10);
            testCase.verifyLessThanOrEqual(next.predictedState(:,1)+next.stateErrorBound(:,1), ...
                stored.predictedState(:,2)+stored.stateErrorBound(:,2)+1e-10);
        end

        function aLargerObservationBoxDoesNotInvalidateAValidPrediction(testCase)
            [ego, cfg, lane] = localInputs();
            [command, ~, ~, stored] = collisionAvoidanceController(ego, [], lane, cfg, []);
            fresh = localNextEgo(ego, stored, command);
            fresh.controllerStateErrorBound(1:3) = 2*stored.stateErrorBound(1:3, 2);
            [~, ~, problem, next] = collisionAvoidanceController(fresh, [], lane, cfg, stored);

            testCase.verifyTrue(problem.metadata.certificateCompatible);
            testCase.verifyEqual(next.stateErrorBound(1:3, 1), stored.stateErrorBound(1:3, 2), AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(next.stateErrorBound(4:6, 1), stored.stateErrorBound(4:6, 2));
        end

        function disjointStateContractsDoNotAuthorizeAFallback(testCase)
            [ego, cfg, lane] = localInputs();
            [command, ~, ~, stored] = collisionAvoidanceController(ego, [], lane, cfg, []);
            fresh = localNextEgo(ego, stored, command);
            fresh.position(1) = fresh.position(1)+1;

            testCase.verifyError(@() collisionAvoidanceController(fresh, [], lane, cfg, stored), ...
                "collisionAvoidanceController:inconsistentObservation");
        end

        function velocityUncertaintyCannotBeRoundedIntoAStationarySet(testCase)
            [ego, cfg, lane] = localInputs();
            ego.controllerStateErrorBound(4) = 1e-14;
            [~, ~, problem] = collisionAvoidanceController(ego, [], lane, cfg, []);
            testCase.verifyGreaterThan(problem.prediction.egoStateErrorBound(4, end), 0);
            testCase.verifyTrue(problem.metadata.planCertified);
        end

        function aStationaryCurvedRoadPoseBoxHasAFiniteEnclosure(testCase)
            [ego, cfg] = localInputs();
            cfg.referenceSpeed = 0;
            [ego, lane] = localCurvedInputs(ego, 0);
            [~, ~, problem, stored] = collisionAvoidanceController(ego, [], lane, cfg, []);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyEqual(stored.safetyScope, "uncertainFiniteHorizon");
            testCase.verifyGreaterThan(stored.stateErrorBound(1:3, end), zeros(3, 1));
        end

        function movingCurvedRoadPoseBoxesKeepExactVelocityChannels(testCase)
            [ego, cfg] = localInputs();
            [ego, lane] = localCurvedInputs(ego, 5);
            [~, ~, problem, stored] = collisionAvoidanceController(ego, [], lane, cfg, []);
            testCase.verifyGreaterThan(abs(problem.prediction.scheduleCurvature(1)), 0.0);
            testCase.verifyGreaterThanOrEqual(stored.stateErrorBound(4:6, :), ...
                zeros(3, problem.prediction.nodeCount));
            testCase.verifyEqual(stored.safetyScope, "uncertainFiniteHorizon");
        end

        function aNegativeForcingComponentIsRejectedBeforePropagation(testCase)
            [ego, cfg, lane] = localInputs();
            cfg.model.ltvModelErrorRateBound(2) = -0.01;
            testCase.verifyError(@() collisionAvoidanceController(ego, [], lane, cfg, []), ...
                "collisionAvoidanceController:invalidConfiguration");
        end

        function persistentPositionDriftIsRetainedOverTheFiniteCertificate(testCase)
            [ego, cfg, lane] = localInputs();
            cfg.model.plantModelResidualRateBound(1) = 0.001;
            [~, ~, problem, stored] = collisionAvoidanceController(ego, [], lane, cfg, []);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyGreaterThan(stored.stateErrorBound(1, end), stored.stateErrorBound(1, 1));
        end

        function aClippedProjectionCannotCertifyCartesianUncertainty(testCase)
            [ego, cfg, lane] = localInputs();
            ego.position(1) = 0;
            testCase.verifyError(@() collisionAvoidanceController(ego, [], lane, cfg, []), ...
                "collisionAvoidanceController:invalidUncertaintyChart");
        end

        function everyVertexOfABoxSatisfiesItsRobustSlipDomains(testCase)
            [ego, cfg, lane] = localInputs();
            [~, ~, problem] = collisionAvoidanceController(ego, [], lane, cfg, []);
            [worst, robust] = localSlipVertexResiduals(problem.prediction, cfg);
            testCase.verifyEqual(worst, robust, AbsTol=1e-12);
        end
    end
end

function [ego, lane] = localCurvedInputs(ego, speed)
    angle = (0:0.025:1).';
    lane = 400*[sin(angle), 1-cos(angle)];
    ego.position = mean(lane(3:4, :), 1).';
    delta = lane(4, :)-lane(3, :);
    ego.yawAngle = atan2(delta(2), delta(1));
    ego.longitudinalVelocity = speed;
end

function [ego, cfg, lane] = localInputs()
    cfg = collisionAvoidanceControllerConfig(struct("controller", struct("horizonSteps", 4,"certifiedSteps",Inf)));
    lane = [0, 0; 1000, 0];
    ego = struct("position", [10; 0], "yawAngle", 0, ...
        "longitudinalVelocity", 5, "lateralVelocity", 0, "yawRate", 0, ...
        "stateTime", 0, "controllerStateErrorBound", [0.1; 0.1; 0.001; 0; 0; 0]);
end

function fresh = localNextEgo(previous, stored, command)
    state = stored.predictedState(:, 2);
    fresh = previous;
    fresh.position = state(1:2);
    fresh.yawAngle = state(3);
    fresh.longitudinalVelocity = state(4);
    fresh.lateralVelocity = state(5);
    fresh.yawRate = state(6);
    fresh.stateTime = previous.stateTime+0.05;
    fresh.heldActuatorInput = command.actuatorInput;
end

function [worst, robust] = localSlipVertexResiduals(prediction, cfg)
    model = struct("cfg", cfg, "inputDimension", 2);
    radius = [0.02; 0.03; 0.001; 0.02; 0.03; 0.002];
    prediction.egoStateErrorBound = repmat(radius, 1, prediction.nodeCount);
    [~, ~, rows, charged] = tireSlipRows(prediction, model);
    prediction.egoStateErrorBound(:) = 0;
    [~, ~, ~, uncharged] = tireSlipRows(prediction, model);
    vertices = (2*double(dec2bin(0:63, 6).'-'0')-1).*radius;
    worst = max(rows(:, 1:6, 1)*vertices+uncharged(:, 1), [], 2);
    robust = charged(:, 1);
end
