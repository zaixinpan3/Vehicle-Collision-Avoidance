classdef nrmmModelAidedEstimationTest < matlab.unittest.TestCase
    %nrmmModelAidedEstimationTest Shared-model ego measurement and contract-consistent target fit.
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
                lateral=nrmmModelLateralVelocity(measured,rate,accel,input,model,sensors,0.3);
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
            lateral=nrmmModelLateralVelocity(x(4:5),x(6),acceleration,input,model,exact,0.3);
            testCase.verifyEqual(lateral.center,x(5),AbsTol=2e-5);
            testCase.verifyLessThanOrEqual(lateral.lower,x(5)+1e-6);
            testCase.verifyGreaterThanOrEqual(lateral.upper,x(5)-1e-6);
            testCase.verifyGreaterThan(lateral.radius,0.05);
            model.parameterUncertainty=0;
            narrow=nrmmModelLateralVelocity(x(4:5),x(6),acceleration,input,model,exact,0.3);
            testCase.verifyLessThan(narrow.radius,1e-4);
        end
        function contradictoryMeasurementsGiveAnEmptyInterval(testCase)
            [model,sensors]=localModel(0.05);
            % Straight driving with zero steering cannot produce 8 m/s^2.
            lateral=nrmmModelLateralVelocity([15;0],0,[0;8],[0;0],model,sensors,0.05);
            testCase.verifyFalse(lateral.consistent);
            testCase.verifyTrue(isinf(lateral.radius));
        end
        function windowFitRecoversConstantParametersOfTheNrmmContract(testCase)
            q=[3;-2;.4;9;.8;.045;1.6;2.4;.95;0;0];
            domain=localTargetDomain();
            fit=nrmmTargetParameterFit("initialize",2,.5,domain);
            times=-2.5:1/80:0;
            path=predictiveSafetyGeometry.predictTarget(q,times);
            for index=1:numel(times)
                fit=nrmmTargetParameterFit("append",fit,times(index),[0;0],path(1:2,index),0);
            end
            testCase.verifyLessThanOrEqual(fit.time(end)-fit.time(1),2+1e-9);
            solution=nrmmTargetParameterFit("solve",fit,0,0,0);
            testCase.verifyTrue(solution.available);
            testCase.verifyEqual(solution.acceleration,q(5),AbsTol=1e-5);
            testCase.verifyEqual(solution.sideslip,q(6),AbsTol=1e-6);
            testCase.verifyEqual(solution.speed,q(4),AbsTol=1e-5);
            testCase.verifyEqual(solution.position,q(1:2),AbsTol=1e-5);
            testCase.verifyEqual(solution.course,q(3)+q(6),AbsTol=1e-6);
        end
        function windowFitAveragesBoundedNoiseAndNeedsItsMinimumSpan(testCase)
            q=[0;0;pi;8;0;0;1.6;2.4;.95;0;0];
            domain=localTargetDomain();
            fit=nrmmTargetParameterFit("initialize",2,.5,domain);
            stream=RandStream("mt19937ar",Seed=5);
            times=-1.5:1/80:0;path=predictiveSafetyGeometry.predictTarget(q,times);
            for index=1:numel(times)
                if index==20
                    testCase.verifyFalse(nrmmTargetParameterFit("solve",fit,times(index-1),0,0).available);
                end
                noisy=path(1:2,index)+0.04*(2*rand(stream,2,1)-1)/sqrt(2);
                fit=nrmmTargetParameterFit("append",fit,times(index),[0;0],noisy,0);
            end
            solution=nrmmTargetParameterFit("solve",fit,0,0,0);
            testCase.verifyTrue(solution.available);
            testCase.verifyLessThan(abs(solution.acceleration),0.1);
            testCase.verifyLessThan(abs(solution.sideslip),0.005);
            testCase.verifyLessThan(solution.residualRms,0.04);
            testCase.verifyError(@()nrmmTargetParameterFit("append",fit,0,[0;0],[1;0],0), ...
                "nrmmTargetParameterFit:timeOrder");
        end
        function modelSelectionKeepsAStraightForecastUntilCurvatureIsSignificant(testCase)
            domain=localTargetDomain();stream=RandStream("mt19937ar",Seed=9);
            straight=[0;0;pi;8;0;0;1.6;2.4;.95;0;0];turning=straight;turning(5)=1;turning(6)=.05;
            for q={straight,turning}
                fit=nrmmTargetParameterFit("initialize",2,.5,domain);
                times=-.5:1/80:0;path=predictiveSafetyGeometry.predictTarget(q{1},times);
                for index=1:numel(times)
                    fit=nrmmTargetParameterFit("append",fit,times(index),[0;0], ...
                        path(1:2,index)+0.04*(2*rand(stream,2,1)-1)/sqrt(2),0);
                end
                solution=nrmmTargetParameterFit("solve",fit,0,0,0);
                if q{1}(6)==0
                    testCase.verifyEqual(solution.selected(6),false);
                    testCase.verifyEqual(solution.sideslip,0);
                else
                    testCase.verifyEqual(solution.selected(6),true);
                    testCase.verifyEqual(solution.sideslip,q{1}(6),AbsTol=0.02);
                end
            end
        end
        function gyroHeadingsKeepTheFitAccurateForATurningEgoWithAYawError(testCase)
            % Body-frame detections from a turning, accelerating ego. A
            % constant yaw-estimate error rotates the target path and adds that
            % angle times the ego displacement: A and kappa move only by the
            % error times the ego acceleration, not by the error times range.
            q=[30;5;-2.6;9;.6;.04;1.6;2.4;.95;0;0];domain=localTargetDomain();
            times=-1.5:1/80:0;path=predictiveSafetyGeometry.predictTarget(q,times);
            rate=.5;yaw0=.3;egoPath=[8*times;0.5*times.^2];
            fit=nrmmTargetParameterFit("initialize",2,.5,domain);
            for index=1:numel(times)
                heading=yaw0+rate*times(index);
                rotation=[cos(heading),-sin(heading);sin(heading),cos(heading)];
                radar=rotation.'*(path(1:2,index)-egoPath(:,index));
                fit=nrmmTargetParameterFit("append",fit,times(index),egoPath(:,index),radar,rate);
            end
            exact=nrmmTargetParameterFit("solve",fit,0,yaw0,rate);
            rotated=nrmmTargetParameterFit("solve",fit,0,yaw0+.01,rate);
            testCase.verifyEqual(exact.acceleration,q(5),AbsTol=1e-5);
            testCase.verifyEqual(exact.sideslip,q(6),AbsTol=1e-6);
            testCase.verifyEqual(rotated.acceleration,q(5),AbsTol=.01);
            testCase.verifyEqual(rotated.sideslip,q(6),AbsTol=1e-3);
            testCase.verifyEqual(rotated.course-exact.course,.01,AbsTol=.01);
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

function domain=localTargetDomain()
    domain=struct('rearAxleDistance',1.6,'sideslipMaximum',.055,'speedMinimum',1, ...
        'speedMaximum',20,'scalarAccelerationMaximum',1.1);
end
