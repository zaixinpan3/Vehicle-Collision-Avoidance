classdef nrmmNativeKernelTest < matlab.unittest.TestCase
    properties (TestParameter)
        targetCount = struct('one',1,'two',2)
        informative = struct('headingCorrection',true,'gyroOnly',false)
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"estimator")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"solver","nrmm")));
        end
    end
    methods (Test)
        function nativeHistoryPreservesCorrelatedSensorAndCoastingBounds(testCase)
            domain = struct("speedMaximum",20,"accelerationNormBound",3, ...
                "yawRateMaximum",0.1,"scalarAccelerationMaximum",2);
            history = nrmmTargetHistory("initialize",domain,0,2,0.6);
            design.sensors = struct("velocityNoiseMaximum",0.05,"positionNoiseMaximum",0.04, ...
                "radarNoiseMaximum",0.04,"gyroscopeNoiseMaximum",0.0015);
            design.yaw.courseModel = struct("rearAxleDistance",1.5, ...
                "singleTrackYawRateMismatchMaximum",0.1,"sideslipDomainMaximum",0.12);
            for time = 0:0.0125:1
                input = struct("time",time,"gnssVelocity",[10;0],"yawRate",0, ...
                    "gnssPosition",[10*time;0], ...
                    "radarRelativePosition",[20-18*time,1]+0.04*[sin(13*time),cos(13*time)]);
                history = nrmmTargetHistory("sensor",history,input,design,1);
            end
            for age = [0,0.3]
                for acceleration = [0.6,Inf]
                    history.yawAccelerationMaximum = acceleration;
                    expected = nrmmTargetHistory("encloseKernel",history,1+age);
                    actual = nrmmTargetHistoryKernelMex(history,1+age);
                    testCase.verifyEqual(actual,expected,AbsTol=1e-10,RelTol=1e-12);
                    testCase.verifyTrue(actual.available);
                    truth = [12-8*age;1;-8;0;0;0];
                    testCase.verifyLessThanOrEqual(actual.lower,truth);
                    testCase.verifyGreaterThanOrEqual(actual.upper,truth);
                end
            end
        end

        function nativeIntegrationPreservesEveryAcceptedSubstep(testCase,targetCount,informative)
            domain = struct("speedMinimum",5,"speedMaximum",20,"scalarAccelerationMaximum",2, ...
                "yawRateMaximum",0.1,"accelerationNormBound",3,"sideslipMaximum",0.1, ...
                "rearAxleDistance",1.6,"relativePositionMaximum",50);
            design = struct("velocity",struct("gain",2.7),"position",struct("gain",2.7), ...
                "target",struct("innovationGains",[8;20;30],"domain",domain), ...
                "yaw",struct("correctionBandwidth",4,"courseModel", ...
                struct("rearAxleDistance",1.5,"sideslipDomainMaximum",0.12)));
            targets = repmat([28;0.8;-12;0.2;0.2;0.1],targetCount,1);
            predictors = repmat([27.95;0.82],targetCount,1);
            state = [10;0.1;0.01;-0.02;0;0;targets;predictors;-pi+0.01];
            measurement = struct("yawRate",0.04,"gnssVelocity",[-10.02;-0.12], ...
                "bodyAcceleration",[0.2;0.1],"radarDetectionAvailable",[true;false(targetCount-1,1)], ...
                "correspondence",struct("informative",informative,"heading",pi-0.01));
            [expected,expectedDerivatives] = nrmmObserverRk4Interval(state,measurement,design,0.0025,5);
            [actual,actualDerivatives] = nrmmObserverRk4IntervalMex(state,measurement,design,0.0025,5);
            testCase.verifyEqual(actual,expected,AbsTol=1e-10);
            testCase.verifyEqual(actualDerivatives,expectedDerivatives,AbsTol=1e-10);
            testCase.verifySize(actual,[7+8*targetCount,6]);
            testCase.verifyLessThanOrEqual(max(abs(actual(end,:))),pi);
        end
    end
end
