classdef nrmmModelAidedEstimationTest < matlab.unittest.TestCase
    %nrmmModelAidedEstimationTest Shared-model ego measurement and the controller-derived domain.
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for folder=["config","controller","estimator","scripts"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,folder)));
            end
        end
    end
    methods (Test)
        function forceBalanceIntervalContainsTheTrueLateralVelocity(testCase)
            [model,sensors,cfg]=localModel(0.05);
            stream=RandStream("mt19937ar",Seed=11);
            for sample=1:200
                [x,input,acceleration]=localState(stream,cfg);
                measured=[x(4);x(5)]+sensors.velocityNoiseMaximum*(2*rand(stream,2,1)-1)/sqrt(2);
                rate=x(6)+sensors.gyroscopeNoiseMaximum*(2*rand(stream)-1);
                accel=acceleration+sensors.accelerometerNoiseMaximum*(2*rand(stream,2,1)-1)/sqrt(2);
                lateral=nrmmEgoCourseGeometry("lateralVelocity",measured,rate,accel,input,model,sensors,0.3);
                testCase.verifyTrue(lateral.consistent);
                testCase.verifyGreaterThanOrEqual(x(5),lateral.lower-1e-6);
                testCase.verifyLessThanOrEqual(x(5),lateral.upper+1e-6);
                testCase.verifyGreaterThanOrEqual(lateral.center,lateral.lower);
                testCase.verifyLessThanOrEqual(lateral.center,lateral.upper);
                testCase.verifyEqual(lateral.radius,max(lateral.center-lateral.lower,lateral.upper-lateral.center),AbsTol=1e-12);
            end
        end
        function nominalSolutionIsExactWithoutNoiseEvenWhenTheIntervalIsWide(testCase)
            % Near rear saturation the parameter box widens the interval
            % asymmetrically; the published point is the nominal solution.
            [model,~,cfg]=localModel(0.05);
            exact=struct('velocityNoiseMaximum',0,'gyroscopeNoiseMaximum',0,'accelerometerNoiseMaximum',0);
            x=[0;0;0;14.5;-0.3084;0.602];input=[0.2172;-0.345];
            d=nonlinearBicycleModel.derivative(x,input,cfg);acceleration=[d(4)-x(6)*x(5);d(5)+x(6)*x(4)];
            lateral=nrmmEgoCourseGeometry("lateralVelocity",x(4:5),x(6),acceleration,input,model,exact,0.3);
            testCase.verifyEqual(lateral.center,x(5),AbsTol=2e-5);
            testCase.verifyLessThanOrEqual(lateral.lower,x(5)+1e-6);
            testCase.verifyGreaterThanOrEqual(lateral.upper,x(5)-1e-6);
            testCase.verifyGreaterThan(lateral.radius,0.05);
            model.parameterUncertainty=0;
            narrow=nrmmEgoCourseGeometry("lateralVelocity",x(4:5),x(6),acceleration,input,model,exact,0.3);
            testCase.verifyLessThan(narrow.radius,1e-4);
        end
        function contradictoryMeasurementsGiveAnEmptyInterval(testCase)
            [model,sensors]=localModel(0.05);
            % Straight driving with zero steering cannot produce 8 m/s^2.
            lateral=nrmmEgoCourseGeometry("lateralVelocity",[15;0],0,[0;8],[0;0],model,sensors,0.05);
            testCase.verifyFalse(lateral.consistent);
            testCase.verifyTrue(isinf(lateral.radius));
        end
        function estimatorDomainIsComputedFromTheController(testCase)
            controller=collisionAvoidanceControllerConfig();
            estimator=estimatorConfigurationFromController(estimatorControllerIntegrationConfig(),controller);
            domain=estimator.observer.ego.domain;tire=modifiedFialaTire.parameters(controller);
            testCase.verifyEqual(domain.speedMaximum,controller.model.speedMaximum);
            testCase.verifyEqual(domain.yawRateMaximum,controller.model.yawRateMaximum);
            testCase.verifyEqual(domain.yawAccelerationMaximum, ...
                [controller.vehicle.lf,controller.vehicle.lr]*tire.longitudinalForceScale/controller.vehicle.Iz,RelTol=1e-12);
            testCase.verifyGreaterThan(domain.accelerationNormMaximum,min(tire.frictionCoefficient)*controller.vehicle.gravity);
            testCase.verifyEqual(estimator.observer.ego.yaw.sideslipDomainMaximum,controller.model.sideslipMaximum);
            testCase.verifyEqual(estimator.observer.ego.yaw.rearAxleDistance,controller.vehicle.lr);
            testCase.verifyEqual(estimator.observer.ego.model.mass,controller.vehicle.m);
            testCase.verifyEqual(estimator.observer.ego.model.parameterUncertainty,0.05);
        end
    end
end

function [model,sensors,cfg]=localModel(uncertainty)
    cfg=collisionAvoidanceControllerConfig();
    model=struct('mass',cfg.vehicle.m,'lf',cfg.vehicle.lf,'lr',cfg.vehicle.lr,'gravity',cfg.vehicle.gravity, ...
        'corneringStiffness',cfg.tire.corneringStiffness,'frictionCoefficient',cfg.tire.frictionCoefficient, ...
        'parameterUncertainty',uncertainty);
    sensors=struct('velocityNoiseMaximum',0.05,'gyroscopeNoiseMaximum',0.0015,'accelerometerNoiseMaximum',0.03);
end

function [x,input,acceleration]=localState(stream,cfg)
    while true
        speed=5+13*rand(stream);beta=0.2*(2*rand(stream)-1);
        x=[0;0;0;speed*cos(beta);speed*sin(beta);1.2*(2*rand(stream)-1)];
        input=[0.15*(2*rand(stream)-1);0.6*(2*rand(stream)-1)];
        try
            d=nonlinearBicycleModel.derivative(x,input,cfg);
        catch
            continue
        end
        acceleration=[d(4)-x(6)*x(5);d(5)+x(6)*x(4)];
        return
    end
end
