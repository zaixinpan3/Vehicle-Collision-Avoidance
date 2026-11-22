classdef nonlinearOutputFeedbackTest < matlab.unittest.TestCase
    properties (TestParameter)
        speed = {.04, 1.5, 3.3, 8, 25}
        braking = {-1, -.6, 0, .5, 1}
    end
    methods (TestClassSetup)
        function loadSolver(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            setupJointNlpSolver;
            testCase.addTeardown(@()clear('solvePredictiveControl'));
        end
    end
    methods (Test)
        function symbolicFlowMatchesTheDeclaredPlantAcrossModes(testCase,speed,braking)
            cfg=collisionAvoidanceControllerConfig();
            scales=[.95;1.02;.99;1.005];physical=cfg;
            physical.tire.corneringStiffness=physical.tire.corneringStiffness.*scales(1:2);
            physical.tire.frictionCoefficient=physical.tire.frictionCoefficient.*scales(3:4);
            x=[2;-1;.2;speed;.03;.02];u=[.02;braking];
            sx=casadi.SX.sym('x',6);su=casadi.SX.sym('u',2);
            flow=casadi.Function('flow',{sx,su},{nonlinearBicycleModel.symbolicSensorFlow(sx,su,scales,cfg,.025,true)});
            testCase.verifyEqual(full(flow(x,u)),nonlinearBicycleModel.sensorFlow(x,u,physical,.025,true),AbsTol=1e-10);
        end
        function automaticDerivativesMatchTheExistingAnalyticModel(testCase)
            cfg=collisionAvoidanceControllerConfig();x=[2;-1;.2;8;.1;.02];u=[.02;-.4];
            sx=casadi.SX.sym('x',6);su=casadi.SX.sym('u',2);
            f=nonlinearBicycleModel.symbolicDerivative(sx,su,ones(4,1),cfg);
            tangent=casadi.Function('tangent',{sx,su},{jacobian(f,[sx;su])});
            [a,b]=nonlinearBicycleModel.jacobian(x,u,cfg);
            testCase.verifyEqual(full(tangent(x,u)),[a,b],AbsTol=1e-9);
        end
    end
end
