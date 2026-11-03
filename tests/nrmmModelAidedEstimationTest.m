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
        function predictionPointMapEqualsTheCertifiedObserverPointMap(testCase)
            cfg=collisionAvoidanceControllerConfig();
            setup=estimatorConfigurationFromController(estimatorControllerIntegrationConfig(),cfg);
            design=synthesizeNrmmObserverGains(setup.observer);
            for speed=[0,.5,8,25]
                for brake=[-1,-.5,0,1]
                    for held={[],[.2;brake]}
                        input=struct('gnssVelocity',[speed;.05*speed],'yawRate',.12, ...
                            'bodyAcceleration',[-2;.4],'heldInput',held{1},'velocityErrorMaximum',1.2);
                        full=nrmmEgoCourseGeometry('course',input,design);
                        point=nrmmEgoCourseGeometry('pointCourse',input,design);
                        testCase.verifyEqual(point.lateralMeasurement.center,full.lateralMeasurement.center,AbsTol=0);
                        testCase.verifyEqual(point.correspondence.rotationVector,full.correspondence.rotationVector,AbsTol=0);
                        testCase.verifyFalse(isfield(point.correspondence,'radius'));
                    end
                end
            end
        end
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
                % Both the center and its radius describe the full inverse.
                testCase.verifyLessThanOrEqual(abs(lateral.center-x(5)),lateral.radius+1e-6);
                testCase.verifyEqual(lateral.radius,max(lateral.center-lateral.lower,lateral.upper-lateral.center),AbsTol=1e-12);
            end
        end
        function exactObservableInversionIsRecoveredWhenAllUncertaintyVanishes(testCase)
            % A parameter-uncertain inverse retains its whole interval. The
            % proximal term vanishes only with zero IMU and parameter errors.
            [model,~,cfg]=localModel(0.05);
            exact=struct('velocityNoiseMaximum',0,'gyroscopeNoiseMaximum',0,'accelerometerNoiseMaximum',0);
            x=[0;0;0;14.5;-0.3084;0.602];input=[0.2172;-0.345];
            d=nonlinearBicycleModel.derivative(x,input,cfg);acceleration=[d(4)-x(6)*x(5);d(5)+x(6)*x(4)];
            lateral=nrmmEgoCourseGeometry("lateralVelocity",x(4:5),x(6),acceleration,input,model,exact,0.3);
            testCase.verifyLessThanOrEqual(abs(lateral.center-x(5)),lateral.radius+1e-6);
            testCase.verifyLessThanOrEqual(lateral.lower,x(5)+1e-6);
            testCase.verifyGreaterThanOrEqual(lateral.upper,x(5)-1e-6);
            testCase.verifyGreaterThan(lateral.radius,0.05);
            model.parameterUncertainty=0;
            narrow=nrmmEgoCourseGeometry("lateralVelocity",x(4:5),x(6),acceleration,input,model,exact,0.3);
            testCase.verifyLessThan(narrow.radius,1e-4);
            testCase.verifyEqual(narrow.center,x(5),AbsTol=2e-5);
        end
        function tinyImuChangesDoNotSelectOppositeSlipEdgesAtFullBraking(testCase)
            [model,sensors,cfg]=localModel(0.05);
            x=[0;0;0;8;0;0];input=[.2;-1];
            d=nonlinearBicycleModel.derivative(x,input,cfg);a=d(4:5);
            centers=zeros(1,3);
            for k=1:3
                measured=a+[0;(k-2)*1e-6];
                inverse=nrmmEgoCourseGeometry("lateralVelocity",x(4:5),0,measured,input,model,sensors,.3);
                testCase.verifyTrue(inverse.consistent);
                testCase.verifyLessThanOrEqual(inverse.lower,0);
                testCase.verifyGreaterThanOrEqual(inverse.upper,0);
                testCase.verifyGreaterThan(inverse.radius,2);
                centers(k)=inverse.center;
            end
            testCase.verifyLessThan(max(centers)-min(centers),5e-4);
        end
        function saturatedRearForceStillProducesAnEnclosingInterval(testCase)
            [model,sensors,cfg]=localModel(0.05);
            x=[0;0;0;15;2;0];input=[0;.8];
            tire=nonlinearBicycleModel.tireParameters(cfg);
            saturationTangent=3*tire.longitudinalForceScale(2)*sqrt(1-input(2)^2)/tire.corneringStiffness(2);
            testCase.verifyGreaterThan(x(5)/x(4),saturationTangent);
            d=nonlinearBicycleModel.derivative(x,input,cfg);
            acceleration=[d(4)-x(6)*x(5);d(5)+x(6)*x(4)];
            lateral=nrmmEgoCourseGeometry("lateralVelocity",x(4:5),x(6),acceleration,input,model,sensors,cfg.model.sideslipMaximum);
            testCase.verifyTrue(lateral.consistent);
            testCase.verifyLessThanOrEqual(lateral.lower,x(5));
            testCase.verifyGreaterThanOrEqual(lateral.upper,x(5));
            testCase.verifyTrue(terminalSafeSet.handlingRows(x,input,cfg));
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
            domain=estimator.observer.ego.domain;tire=nonlinearBicycleModel.tireParameters(controller);
            lateralMaximum=min(controller.model.lateralVelocityMaximum, ...
                tan(controller.model.sideslipMaximum)*controller.model.speedMaximum);
            testCase.verifyEqual(domain.speedMaximum,hypot(controller.model.speedMaximum,lateralMaximum));
            testCase.verifyEqual(domain.yawRateMaximum,controller.model.yawRateMaximum);
            testCase.verifyEqual(domain.yawAccelerationMaximum, ...
                (1+estimator.observer.ego.model.parameterUncertainty) ...
                *[controller.vehicle.lf,controller.vehicle.lr]*tire.longitudinalForceScale/controller.vehicle.Iz,RelTol=1e-12);
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
