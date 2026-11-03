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
            tire = nonlinearBicycleModel.tireParameters(cfg);
            for ratio = linspace(-1,1,21)
                for angle = linspace(-0.6,0.6,41)
                    force = nonlinearBicycleModel.tireForces([angle;-angle],ratio,cfg);
                    utilization = ratio^2+(force./tire.longitudinalForceScale).^2;
                    testCase.verifyLessThanOrEqual(utilization,ones(2,1)+1e-14);
                end
            end
        end
        function zeroSlipRecoversTheCorneringStiffness(testCase)
            cfg = collisionAvoidanceControllerConfig();
            [force, slope, ~, intercept] = nonlinearBicycleModel.tireForces([0; 0], 0.0, cfg);
            testCase.verifyEqual(force, zeros(2, 1), AbsTol=0.0);
            testCase.verifyEqual(slope, -cfg.tire.corneringStiffness, AbsTol=1e-10);
            testCase.verifyEqual(intercept, zeros(2, 1), AbsTol=0.0);
        end

        function forceIsOddAndOpposesSlip(testCase, slip)
            cfg = collisionAvoidanceControllerConfig();
            force = nonlinearBicycleModel.tireForces([slip; slip], 0.0, cfg);
            reversed = nonlinearBicycleModel.tireForces(-[slip; slip], 0.0, cfg);
            testCase.verifyEqual(reversed, -force, AbsTol=1e-10);
            testCase.verifyLessThan(force*slip, zeros(2, 1));
        end

        function localSlopeMatchesNonlinearFiniteDifferences(testCase, slip, brakingRatio)
            cfg = collisionAvoidanceControllerConfig();
            alpha = [slip; slip];
            step = 1e-6;
            [force, slope, betaSlope, intercept] = nonlinearBicycleModel.tireForces(alpha, brakingRatio, cfg);
            upper = nonlinearBicycleModel.tireForces(alpha+step, brakingRatio, cfg);
            lower = nonlinearBicycleModel.tireForces(alpha-step, brakingRatio, cfg);
            betaUpper = nonlinearBicycleModel.tireForces(alpha, brakingRatio+step, cfg);
            betaLower = nonlinearBicycleModel.tireForces(alpha, brakingRatio-step, cfg);
            testCase.verifyEqual(slope, (upper-lower)/(2*step), AbsTol=1e-3);
            testCase.verifyEqual(betaSlope, (betaUpper-betaLower)/(2*step), AbsTol=1e-3);
            testCase.verifyEqual(slope.*alpha+betaSlope*brakingRatio+intercept, force, AbsTol=1e-10);
        end

        function brakingReducesLateralForceAndLocalSteeringGain(testCase)
            cfg = collisionAvoidanceControllerConfig();
            alpha = [0.05; -0.05];
            [coasting, coastingSlope] = nonlinearBicycleModel.tireForces(alpha, 0.0, cfg);
            [braking, brakingSlope] = nonlinearBicycleModel.tireForces(alpha, ...
                -0.6, cfg);
            testCase.verifyLessThan(abs(braking), abs(coasting));
            testCase.verifyLessThan(abs(brakingSlope), abs(coastingSlope));
        end

        function saturationJoinsContinuouslyForBothSlipSigns(testCase)
            cfg = collisionAvoidanceControllerConfig();
            parameters = nonlinearBicycleModel.tireParameters(cfg);
            capacity = parameters.frictionCoefficient.*parameters.staticNormalLoad;
            boundary = atan(3*capacity./parameters.corneringStiffness).*[1; -1];
            [atBoundary, slope] = nonlinearBicycleModel.tireForces(boundary, 0.0, cfg);
            inside = nonlinearBicycleModel.tireForces(boundary*(1-1e-5), 0.0, cfg);
            outside = nonlinearBicycleModel.tireForces(boundary*(1+1e-5), 0.0, cfg);
            testCase.verifyEqual(atBoundary, -capacity.*[1; -1], AbsTol=1e-9);
            testCase.verifyEqual(inside, outside, AbsTol=1e-8);
            testCase.verifyEqual(slope, zeros(2, 1), AbsTol=1e-10);
        end

        function fullBrakingAndThrottleHaveZeroLateralForce(testCase)
            cfg = collisionAvoidanceControllerConfig();
            braking = nonlinearBicycleModel.tireForces([0; -0.1], -1.0, cfg);
            throttle = nonlinearBicycleModel.tireForces([0.1; -0.1], 1.0, cfg);
            testCase.verifyEqual([braking, throttle], zeros(2, 2), AbsTol=0.0);
            testCase.verifyError(@() localEndpointTangent(cfg), ...
                "collisionAvoidanceController:singularTireLinearization");
        end

        function theCommonRatioSetsBothAxleForces(testCase)
            cfg = collisionAvoidanceControllerConfig();
            parameters = nonlinearBicycleModel.tireParameters(cfg);
            force = nonlinearBicycleModel.tireLongitudinalForce(-0.6, cfg);
            testCase.verifyEqual(force./parameters.longitudinalForceScale, ...
                [-0.6; -0.6], AbsTol=1e-12);
            testCase.verifyEqual(sum(force)/cfg.vehicle.m, ...
                -0.6*cfg.tire.frictionCoefficient(1)*cfg.vehicle.gravity, AbsTol=1e-12);
        end

        function scalarTireParametersMatchAnExplicitAxlePair(testCase)
            cfg = collisionAvoidanceControllerConfig();
            paired = nonlinearBicycleModel.tireForces([0.03; -0.04], -0.3, cfg);
            cfg.tire.corneringStiffness = cfg.tire.corneringStiffness(1);
            cfg.tire.frictionCoefficient = cfg.tire.frictionCoefficient(1);
            scalar = nonlinearBicycleModel.tireForces([0.03; -0.04], -0.3, cfg);
            testCase.verifyEqual(scalar, paired, AbsTol=1e-10);
        end
    end
end

function localEndpointTangent(cfg)
    [~,~]=nonlinearBicycleModel.tireForces([0.02;0.02],1,cfg);
end
