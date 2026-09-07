classdef modifiedFialaTireTest < matlab.unittest.TestCase
    %modifiedFialaTireTest Nonlinear force law and its scheduled affine tangent.

    properties (TestParameter)
        slip = {0.02, -0.04, 0.30, -0.30};
        brakingRatio = {0.0, -0.6, 0.6};
    end

    methods (TestClassSetup)
        function addModelPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
        end
    end

    methods (Test)
        function nonlinearCombinedForcesStayInsideThePhysicalCircle(testCase)
            cfg = collisionAvoidanceControllerConfig();
            tire = modifiedFialaTire.parameters(cfg);
            for ratio = linspace(-1,1,21)
                for angle = linspace(-0.6,0.6,41)
                    force = modifiedFialaTire.evaluate([angle;-angle],ratio,cfg);
                    utilization = ratio^2+(force./tire.longitudinalForceScale).^2;
                    testCase.verifyLessThanOrEqual(utilization,ones(2,1)+1e-14);
                end
            end
        end
        function zeroSlipRecoversTheCorneringStiffness(testCase)
            cfg = collisionAvoidanceControllerConfig();
            [force, slope, ~, intercept] = modifiedFialaTire.evaluate([0; 0], 0.0, cfg);
            testCase.verifyEqual(force, zeros(2, 1), AbsTol=0.0);
            testCase.verifyEqual(slope, -cfg.tire.corneringStiffness, AbsTol=1e-10);
            testCase.verifyEqual(intercept, zeros(2, 1), AbsTol=0.0);
        end

        function forceIsOddAndOpposesSlip(testCase, slip)
            cfg = collisionAvoidanceControllerConfig();
            force = modifiedFialaTire.evaluate([slip; slip], 0.0, cfg);
            reversed = modifiedFialaTire.evaluate(-[slip; slip], 0.0, cfg);
            testCase.verifyEqual(reversed, -force, AbsTol=1e-10);
            testCase.verifyLessThan(force*slip, zeros(2, 1));
        end

        function localSlopeMatchesNonlinearFiniteDifferences(testCase, slip, brakingRatio)
            cfg = collisionAvoidanceControllerConfig();
            alpha = [slip; slip];
            step = 1e-6;
            [force, slope, betaSlope, intercept] = modifiedFialaTire.evaluate(alpha, brakingRatio, cfg);
            upper = modifiedFialaTire.evaluate(alpha+step, brakingRatio, cfg);
            lower = modifiedFialaTire.evaluate(alpha-step, brakingRatio, cfg);
            betaUpper = modifiedFialaTire.evaluate(alpha, brakingRatio+step, cfg);
            betaLower = modifiedFialaTire.evaluate(alpha, brakingRatio-step, cfg);
            testCase.verifyEqual(slope, (upper-lower)/(2*step), AbsTol=1e-3);
            testCase.verifyEqual(betaSlope, (betaUpper-betaLower)/(2*step), AbsTol=1e-3);
            testCase.verifyEqual(slope.*alpha+betaSlope*brakingRatio+intercept, force, AbsTol=1e-10);
        end

        function brakingReducesLateralForceAndLocalSteeringGain(testCase)
            cfg = collisionAvoidanceControllerConfig();
            alpha = [0.05; -0.05];
            [coasting, coastingSlope] = modifiedFialaTire.evaluate(alpha, 0.0, cfg);
            [braking, brakingSlope] = modifiedFialaTire.evaluate(alpha, ...
                -0.6, cfg);
            testCase.verifyLessThan(abs(braking), abs(coasting));
            testCase.verifyLessThan(abs(brakingSlope), abs(coastingSlope));
        end

        function saturationJoinsContinuouslyForBothSlipSigns(testCase)
            cfg = collisionAvoidanceControllerConfig();
            parameters = modifiedFialaTire.parameters(cfg);
            capacity = parameters.frictionCoefficient.*parameters.staticNormalLoad;
            boundary = atan(3*capacity./parameters.corneringStiffness).*[1; -1];
            [atBoundary, slope] = modifiedFialaTire.evaluate(boundary, 0.0, cfg);
            inside = modifiedFialaTire.evaluate(boundary*(1-1e-5), 0.0, cfg);
            outside = modifiedFialaTire.evaluate(boundary*(1+1e-5), 0.0, cfg);
            testCase.verifyEqual(atBoundary, -capacity.*[1; -1], AbsTol=1e-9);
            testCase.verifyEqual(inside, outside, AbsTol=1e-8);
            testCase.verifyEqual(slope, zeros(2, 1), AbsTol=1e-10);
        end

        function fullBrakingAndThrottleHaveZeroLateralForce(testCase)
            cfg = collisionAvoidanceControllerConfig();
            braking = modifiedFialaTire.evaluate([0; -0.1], -1.0, cfg);
            throttle = modifiedFialaTire.evaluate([0.1; -0.1], 1.0, cfg);
            testCase.verifyEqual([braking, throttle], zeros(2, 2), AbsTol=0.0);
            testCase.verifyError(@() modifiedFialaTire.linearize(0.02, 15, 1, cfg), ...
                "collisionAvoidanceController:singularTireLinearization");
        end

        function theCommonRatioSetsBothAxleForces(testCase)
            cfg = collisionAvoidanceControllerConfig();
            parameters = modifiedFialaTire.parameters(cfg);
            force = modifiedFialaTire.longitudinalForce(-0.6, cfg);
            testCase.verifyEqual(force./parameters.longitudinalForceScale, ...
                [-0.6; -0.6], AbsTol=1e-12);
            testCase.verifyEqual(sum(force)/cfg.vehicle.m, ...
                -0.6*cfg.tire.frictionCoefficient(1)*cfg.vehicle.gravity, AbsTol=1e-12);
        end

        function curvedBicycleMatchesTheNonlinearForceAtItsOperatingPoint(testCase)
            cfg = collisionAvoidanceControllerConfig();
            kappa = 0.02;
            speed = 15.0;
            brakingRatio = -0.4;
            steering = atan(cfg.vehicle.wheelbase*kappa);
            state = [0; 0; 0; speed; 0; kappa*speed];
            [stateMatrix, inputMatrix, affine] = ...
                ltvBicycleModel.continuousMatrices(kappa, speed, cfg, brakingRatio);
            slipAngle = [(cfg.vehicle.lf*state(6))/speed-steering; ...
                -cfg.vehicle.lr*state(6)/speed];
            force = modifiedFialaTire.evaluate(slipAngle, ...
                brakingRatio, cfg);
            expected = [speed; 0; 0; modifiedFialaTire.accelerationGain(cfg)*brakingRatio ...
                - longitudinalRoadLoad(speed, cfg)/cfg.vehicle.m; ...
                sum(force)/cfg.vehicle.m-speed*state(6); ...
                [cfg.vehicle.lf, -cfg.vehicle.lr]*force/cfg.vehicle.Iz];
            actual = stateMatrix*state+inputMatrix*[steering; brakingRatio]+affine;
            testCase.verifyEqual(actual, expected, AbsTol=1e-11);
            testCase.verifyGreaterThan(abs(inputMatrix(5, 2)), 0.01);
            testCase.verifyLessThan(inputMatrix(5, 1), cfg.tire.corneringStiffness(1)/cfg.vehicle.m);
            testCase.verifyGreaterThan(norm(affine(5:6)-[speed^2*kappa; 0]), 0.1);
        end

        function curvedRoadRestRemainsInvariantWithFialaTires(testCase)
            cfg = collisionAvoidanceControllerConfig();
            state = [10; 0.1; 0.02; 0; 0; 0];
            [stateMatrix, inputMatrix, affine] = ltvBicycleModel.stageMatrices(0.03, 0, 0.05, cfg);
            testCase.verifyEqual(stateMatrix*state+inputMatrix*[0; 0]+affine, state, AbsTol=1e-12);
        end

        function scalarTireParametersMatchAnExplicitAxlePair(testCase)
            cfg = collisionAvoidanceControllerConfig();
            paired = modifiedFialaTire.evaluate([0.03; -0.04], -0.3, cfg);
            cfg.tire.corneringStiffness = cfg.tire.corneringStiffness(1);
            cfg.tire.frictionCoefficient = cfg.tire.frictionCoefficient(1);
            scalar = modifiedFialaTire.evaluate([0.03; -0.04], -0.3, cfg);
            testCase.verifyEqual(scalar, paired, AbsTol=1e-10);
        end
    end
end
