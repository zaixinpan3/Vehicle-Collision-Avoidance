classdef robustStationaryPoseCertificateTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
        end
    end
    methods (Test)
        function aPoseBoxCannotBeAdmittedAsAnExactState(testCase)
            [ego,cfg,lane] = localInputs();
            testCase.verifyError(@() collisionAvoidanceController(ego,encounterTestFixture.stationaryTarget(),lane,cfg,[]), ...
                "collisionAvoidanceController:nonexactStudyInput");
        end
        function tinyVelocityUncertaintyIsStillNonzero(testCase)
            [ego,cfg,lane] = localInputs();
            ego.controllerStateErrorBound = [0;0;0;1e-14;0;0];
            testCase.verifyError(@() collisionAvoidanceController(ego,encounterTestFixture.stationaryTarget(),lane,cfg,[]), ...
                "collisionAvoidanceController:nonexactStudyInput");
        end
        function persistentDriftCannotEnterTheExactStudy(testCase)
            [ego,cfg,lane] = localInputs();
            ego.controllerStateErrorBound(:) = 0;
            cfg.model.plantModelResidualRateBound(1) = 0.001;
            testCase.verifyError(@() collisionAvoidanceController(ego,encounterTestFixture.stationaryTarget(),lane,cfg,[]), ...
                "collisionAvoidanceController:nonexactStudyInput");
        end
        function aNegativeForcingComponentIsRejectedBeforePropagation(testCase)
            [ego,cfg,lane] = localInputs();
            cfg.model.ltvModelErrorRateBound(2) = -0.01;
            testCase.verifyError(@() collisionAvoidanceController(ego,[],lane,cfg,[]), ...
                "collisionAvoidanceController:invalidConfiguration");
        end
        function aClippedProjectionCannotCertifyCartesianUncertainty(testCase)
            [ego,cfg,lane] = localInputs();
            ego.position(1) = 0;
            testCase.verifyError(@() collisionAvoidanceController(ego,[],lane,cfg,[]), ...
                "collisionAvoidanceController:invalidUncertaintyChart");
        end
        function offlineCurvedPoseBoxesRetainVelocityChannels(testCase)
            [ego,cfg] = localInputs();
            [ego,route] = localCurvedInputs(ego,5);
            [parsed,lane] = readPlanningInputs(ego,[],route,cfg);
            [radius,valid] = stateUncertainty.toFrenet(parsed.modelState,parsed.stateErrorBound,lane);
            testCase.verifyTrue(valid);
            testCase.verifyGreaterThan(radius(1:3),zeros(3,1));
            testCase.verifyEqual(radius(4:6),zeros(3,1),AbsTol=0);
        end
        function everyVertexOfAnOfflineBoxSatisfiesItsRobustSlipDomains(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            [~,~,problem] = collisionAvoidanceController(ego,target,route,cfg,[]);
            [worst,robust] = localSlipVertexResiduals(problem.prediction,cfg);
            testCase.verifyEqual(worst,robust,AbsTol=1e-12);
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
    cfg = collisionAvoidanceControllerConfig(struct("controller", struct("horizonSteps", 4)));
    lane = [0, 0; 1000, 0];
    ego = struct("position", [10; 0], "yawAngle", 0, ...
        "longitudinalVelocity", 5, "lateralVelocity", 0, "yawRate", 0, ...
        "stateTime", 0, "controllerStateErrorBound", [0.1; 0.1; 0.001; 0; 0; 0]);
            ego.stateTime = 0;
            ego.perception = struct("time",0,"range",30,"completeWithinRange",true);
end

function [worst, robust] = localSlipVertexResiduals(prediction, cfg)
    model = struct("cfg", cfg, "inputDimension", 2);
    radius = [0.02; 0.03; 0.001; 0.02; 0.03; 0.002];
    prediction.egoStateErrorBound = repmat(radius, 1, prediction.nodeCount);
    [~, ~, rows, charged] = ltvBicycleModel.slipRows(prediction, model);
    prediction.egoStateErrorBound(:) = 0;
    [~, ~, ~, uncharged] = ltvBicycleModel.slipRows(prediction, model);
    vertices = (2*double(dec2bin(0:63, 6).'-'0')-1).*radius;
    worst = max(rows(:, 1:6, 1)*vertices+uncharged(:, 1), [], 2);
    robust = charged(:, 1);
end
