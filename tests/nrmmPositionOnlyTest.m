classdef nrmmPositionOnlyTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for directory=["config","estimator","controller","scripts"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,directory)));
            end
        end
    end
    methods (Test)
        function runtimeAcceptsPositionOnlyAndIgnoresFictitiousVelocityChannels(testCase)
            [runtime,frame]=localFixture();
            first=onlineNrmmTrackingRuntime('sample',runtime,frame);
            frame.vxGps=-1e5;frame.vyGps=1e5;
            second=onlineNrmmTrackingRuntime('sample',runtime,frame);
            testCase.verifyEqual(first,second);
        end
        function firstPositionDoesNotTurnAnArbitraryPositionGuessIntoVelocity(testCase)
            [runtime,frame]=localFixture();
            runtime.positionVelocity.point(1:2)=[500;-200];
            output=onlineNrmmTrackingRuntime('output',runtime,frame);
            testCase.verifyEqual(output.inertialVelocityEstimate,[8;0]);
            testCase.verifyEqual(output.inertialVelocityEstimateErrorBound,.2);
        end
        function positionMotionCorrectsAnInitiallyWrongVelocity(testCase)
            [runtime,frame]=localFixture();
            runtime.positionVelocity.point(3:4)=[6;0];
            runtime.positionVelocity.generator=diag([.04,2]);
            for k=0:159
                frame.time=runtime.currentTime;frame.xGps=8*frame.time;
                [runtime,output]=onlineNrmmTrackingRuntime('sample',runtime,frame);
            end
            testCase.verifyEqual(output.inertialVelocityEstimate,[8;0],AbsTol=.001);
            testCase.verifyEqual(output.velocitySource,"position-history-and-imu");
            testCase.verifyGreaterThan(output.inertialVelocityEstimateErrorBound,0);
        end
        function continuousYawInnovationDoesNotUseTheInformativeSwitch(testCase)
            [runtime,~]=localFixture();d=runtime.observerDesign;
            c=struct('rotationVector',[1;0],'informative',true);
            on=nrmmObserverVectorField('yawDerivative',.4,.1,c,d);
            c.informative=false;
            off=nrmmObserverVectorField('yawDerivative',.4,.1,c,d);
            testCase.verifyEqual(on,off);
            testCase.verifyLessThan(off,.1);
            c.rotationVector=zeros(2,1);
            testCase.verifyEqual(nrmmObserverVectorField('yawDerivative',.4,.1,c,d),.1);
        end
        function accelerationAndPositionNoiseRemainInsideTheVelocityEnclosure(testCase)
            [runtime,~]=localFixture();design=runtime.observerDesign;dt=.0125;
            hold=struct('acceleration',2,'bodyAccelerationRate',2,'yawAcceleration',0);
            filter=nrmmPositionVelocity('initialize',[.04;0],[7;0],.04,1,dt,8);
            largest=0;
            for k=0:399
                t=k*dt;truthP=[8*t+1-cos(t);0];truthV=[8+sin(t);0];
                position=truthP+.04*[cos(1.7*k);sin(1.7*k)];
                filter=nrmmPositionVelocity('measure',filter,position,.04);
                error=norm(filter.point(3:4)-truthV);
                testCase.verifyLessThanOrEqual(error,filter.velocityRadius+1e-12);
                largest=max(largest,error);
                acceleration=[cos(t);0]+.03*[cos(.8*k);sin(.8*k)];
                filter=nrmmPositionVelocity('predict',filter,acceleration,0,0,design,hold);
            end
            testCase.verifyGreaterThan(largest,.1);
            testCase.verifyLessThan(filter.velocityRadius,1);
        end
        function yawUncertaintyEnlargesTheImuPredictionBound(testCase)
            [runtime,~]=localFixture();d=runtime.observerDesign;
            f=nrmmPositionVelocity('initialize',[0;0],[8;0],.04,.2,.0125,8);
            h=struct('acceleration',10,'bodyAccelerationRate',0,'yawAcceleration',0);
            narrow=nrmmPositionVelocity('predict',f,[1;0],0,0,d,h);
            wide=nrmmPositionVelocity('predict',f,[1;0],0,.5,d,h);
            testCase.verifyGreaterThan(sum(abs(wide.generator(2,:))),sum(abs(narrow.generator(2,:))));
        end
    end
end

function [runtime,frame]=localFixture()
    cfg=nrmmTrackingConfig();cfg.measurement.gps.positionNoiseMaximum=.04;
    cfg.measurement.imu.noiseMaximum=.03;cfg.ego.domain.speedMinimum=0;
    cfg.ego.domain.accelerationNormMaximum=2;
    cfg.ego.domain.bodyAccelerationRateMaximum=2;
    cfg.ego.domain.yawAccelerationMaximum=.5;
    design=synthesizeNrmmObserverGains(cfg);
    options=struct('egoInitialPosition',[0;0],'egoInitialYaw',0, ...
        'egoInitialBodyVelocity',[8;0],'targetInitialState',[20;0;15;0;0;0], ...
        'inertialVelocityInitialization',struct('value',[8;0],'radius',.2));
    runtime=onlineNrmmTrackingRuntime('initialize',cfg,options,design);
    frame=struct('time',0,'xGps',0,'yGps',0,'longitudinalAcceleration',0, ...
        'lateralAcceleration',0,'yawRateMeasured',0,'radarRelativePosition',[NaN;NaN], ...
        'radarDetectionAvailable',false);
end
