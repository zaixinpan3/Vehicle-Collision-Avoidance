classdef longitudinalRoadLoadTest < matlab.unittest.TestCase
    %longitudinalRoadLoadTest Force balance, dissipativity and shared dynamics.

    properties (TestParameter)
        speed = {0.0, 0.1, 5.0, 15.0};
        invalidCoefficient = {-1, NaN, Inf, 1i, [1, 2]};
    end

    methods (TestClassSetup)
        function addControllerPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
        end
    end

    methods (Test)
        function roadLoadOpposesMotionAndPreservesRest(testCase, speed)
            cfg = collisionAvoidanceControllerConfig();
            positive = longitudinalRoadLoad(speed, cfg);
            negative = longitudinalRoadLoad(-speed, cfg);

            testCase.verifyGreaterThanOrEqual(positive*speed, 0.0);
            testCase.verifyEqual(negative, -positive, AbsTol=1.0e-12);
            testCase.verifyEqual(longitudinalRoadLoad(0, cfg), 0.0, AbsTol=0.0);
        end

        function aerodynamicForceHasThePhysicalQuadraticScaling(testCase)
            cfg = collisionAvoidanceControllerConfig();
            [~, ~, first] = longitudinalRoadLoad(10, cfg);
            [~, ~, second] = longitudinalRoadLoad(20, cfg);

            testCase.verifyEqual(first.aerodynamicForce, ...
                0.5*cfg.roadLoad.airDensity*cfg.roadLoad.dragCoefficient ...
                    *cfg.roadLoad.frontalArea*100, AbsTol=1.0e-12);
            testCase.verifyEqual(second.aerodynamicForce, 4*first.aerodynamicForce, AbsTol=1.0e-12);
            testCase.verifyGreaterThan(first.rollingResistanceForce, 0.0);
        end

        function theRoadLoadSlopeMatchesFiniteDifferences(testCase, speed)
            cfg = collisionAvoidanceControllerConfig(struct("roadLoad", struct( ...
                "rollingSpeedCoefficient", 1.0e-4, "rollingQuarticCoefficient", 1.0e-8)));
            step = 1.0e-6;
            [~, slope] = longitudinalRoadLoad(speed, cfg);
            finiteDifference = (longitudinalRoadLoad(speed+step, cfg) ...
                - longitudinalRoadLoad(speed-step, cfg))/(2*step);

            testCase.verifyEqual(slope, finiteDifference, AbsTol=1.0e-5);
        end

        function coastingDeceleratesAndBalancedForceHoldsCruise(testCase)
            cfg = collisionAvoidanceControllerConfig();
            initial = [0; 0; 0; 15; 0; 0];
            [stateMatrix, inputMatrix, affine] = ltvBicycleModel.stageMatrices(0, 15, 0.05, cfg);
            balance = longitudinalRoadLoad(15, cfg)/(cfg.vehicle.m*modifiedFialaTire.accelerationGain(cfg));
            coasting = stateMatrix*initial+affine;
            cruise = coasting+inputMatrix*[0; balance];

            testCase.verifyLessThan(coasting(4), initial(4));
            testCase.verifyEqual(cruise, [0.75; 0; 0; 15; 0; 0], AbsTol=1.0e-12);
        end

        function commandAndClfUseTheSameLongitudinalForceBalance(testCase)
            [command, problem, cfg] = localProblem(14.8);
            initial = problem.prediction.egoStateOffset(:, 1);
            [stateMatrix, inputMatrix, affine] = ltvBicycleModel.continuousMatrices(0, 14.8, cfg);
            derivative = stateMatrix*initial+inputMatrix*command.actuatorInput+affine;
            forceDerivative = (command.totalLongitudinalActuatorForce ...
                -command.aerodynamicResistanceForce-command.rollingResistanceForce)/cfg.vehicle.m;
            clf = problem.qp.clf;
            valueDerivative = 2*clf.errorOffset(:, 1).'*clf.lyapunovMatrix*derivative(2:6);

            testCase.verifyEqual(command.bodyLongitudinalVelocityDerivative, forceDerivative, AbsTol=1.0e-12);
            testCase.verifyEqual(derivative(4), forceDerivative, AbsTol=1.0e-12);
            testCase.verifyEqual(command.totalLongitudinalActuatorForce, ...
                cfg.vehicle.m*modifiedFialaTire.accelerationGain(cfg)*command.actuatorInput(2), AbsTol=1.0e-10);
            testCase.verifyEqual(forceDerivative, (command.totalLongitudinalTireForce ...
                -command.aerodynamicResistanceForce)/cfg.vehicle.m, AbsTol=1.0e-12);
            testCase.verifyEqual(valueDerivative, clf.lieDerivativeDrift ...
                +clf.lieDerivativeInput*command.actuatorInput, AbsTol=1.0e-10);
            testCase.verifyEqual(sum(command.axleNormalLoad), cfg.vehicle.m*cfg.vehicle.gravity, AbsTol=1.0e-10);
        end

        function theCruiseEquilibriumAccountsForPassiveForces(testCase)
            [~, problem, cfg] = localProblem(15);
            requiredInput = longitudinalRoadLoad(15, cfg)/(cfg.vehicle.m*modifiedFialaTire.accelerationGain(cfg));

            testCase.verifyEqual(problem.qp.clf.certificate.operatingInput, [0;requiredInput], AbsTol=1.0e-12);
            testCase.verifyGreaterThan(requiredInput, 0.0);
        end

        function tireNormalLoadsUseTheStaticWeightDistribution(testCase)
            [command, ~, cfg] = localProblem(14.8);
            expected = cfg.vehicle.m*cfg.vehicle.gravity/(cfg.vehicle.lf+cfg.vehicle.lr) ...
                * [cfg.vehicle.lr; cfg.vehicle.lf];
            testCase.verifyEqual(command.axleNormalLoad, expected, AbsTol=1.0e-10);
        end

        function changingRoadLoadRebuildsTheRiccatiCertificate(testCase)
            [~, first, cfg] = localProblem(14.8);
            cfg.roadLoad.dragCoefficient = 1.2;
            ego = struct("position", [0; 0], "yawAngle", 0, "speed", 14.8);
    ego.stateTime = 0;
    ego.perception = struct("time",0,"range",30,"completeWithinRange",true);
            [~, ~, second] = collisionAvoidanceController(ego, [], [0, 0; 2000, 0], cfg, []);

            testCase.verifyGreaterThan(norm(first.qp.clf.lyapunovMatrix-second.qp.clf.lyapunovMatrix), 1.0e-10);
            testCase.verifyGreaterThan(second.qp.clf.certificate.operatingInput(2), ...
                first.qp.clf.certificate.operatingInput(2));
        end

        function passiveCoefficientsMustBeFiniteNonnegativeScalars(testCase, invalidCoefficient)
            override = struct("roadLoad", struct("rollingCoefficient", invalidCoefficient));

            testCase.verifyError(@() collisionAvoidanceControllerConfig(override), ...
                "collisionAvoidanceController:invalidConfiguration");
        end

        function theRollingTransitionSpeedMustBePositive(testCase)
            testCase.verifyError(@() collisionAvoidanceControllerConfig( ...
                struct("roadLoad", struct("rollingTransitionSpeed", 0))), ...
                "collisionAvoidanceController:invalidConfiguration");
        end
    end
end

function [command, problem, cfg] = localProblem(speed)
    cfg = collisionAvoidanceControllerConfig(struct("controller", struct("horizonSteps", 4)));
    ego = struct("position", [0; 0], "yawAngle", 0, "speed", speed);
    ego.stateTime = 0;
    ego.perception = struct("time",0,"range",30,"completeWithinRange",true);
    [command, ~, problem] = collisionAvoidanceController(ego, [], [0, 0; 2000, 0], cfg, []);
end
