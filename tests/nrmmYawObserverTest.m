classdef nrmmYawObserverTest < matlab.unittest.TestCase
    % The continuous yaw observer is independent of the body-frame core.
    properties
        Config
        Design
    end
    methods (TestClassSetup)
        function prepareDesign(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"estimator")));
            testCase.Config = nrmmTrackingConfig();
            testCase.Design = synthesizeNrmmObserverGains(testCase.Config);
        end
    end
    methods (Test)
        function informativeHeadingCorrectsTheGyroPropagation(testCase)
            measurement = struct("heading",0.3,"informative",true);
            derivative = nrmmYawObserverDerivative(0.2,0.12,measurement,testCase.Design);
            testCase.verifyEqual(derivative, ...
                0.12+0.1*testCase.Design.yaw.correctionBandwidth,AbsTol=1e-14);
        end

        function headingCorrectionTakesTheShortDirectionAcrossPi(testCase)
            measurement = struct("heading",-pi+0.02,"informative",true);
            derivative = nrmmYawObserverDerivative(pi-0.03,0,measurement,testCase.Design);
            testCase.verifyEqual(derivative, ...
                0.05*testCase.Design.yaw.correctionBandwidth,AbsTol=1e-14);
        end

        function uninformativeHeadingLeavesOnlyGyroPropagation(testCase)
            measurement = struct("heading",NaN,"informative",false);
            derivative = nrmmYawObserverDerivative(0.8,0.17,measurement,testCase.Design);
            testCase.verifyEqual(derivative,0.17,AbsTol=0);
        end

        function runtimeIntegratesHeadingConvergenceWithoutSnappingToTheSet(testCase)
            runtime = localRuntime(testCase.Config,testCase.Design,0.4,[15;0]);
            [changed,output] = onlineNrmmTrackingRuntime("step",runtime,localFrame(0,[15;0],0));
            expected = 0.4*exp(-testCase.Design.yaw.correctionBandwidth*runtime.samplePeriod);
            testCase.verifyEqual(output.egoYaw,expected,AbsTol=1e-10);
            testCase.verifyEqual(changed.yawEstimate,output.egoYaw,AbsTol=0);
            testCase.verifyGreaterThan(output.egoYaw,0.3);
            testCase.verifyEqual(output.orientationSet.heading,0,AbsTol=1e-12);
            testCase.verifyGreaterThan(output.egoYawErrorBound,output.orientationSet.radius+0.3);
            testCase.verifyLessThanOrEqual(abs(output.egoYaw),output.egoYawErrorBound);
            testCase.verifyLessThanOrEqual(norm(output.egoVelocityInertial-[15;0]), ...
                output.egoInertialVelocityErrorBound);
            testCase.verifyTrue(output.yawObserverCorrectionAvailable);
        end

        function outputInspectionPreservesTheIntegratedYawEstimate(testCase)
            runtime = localRuntime(testCase.Config,testCase.Design,0.4,[15;0]);
            output = onlineNrmmTrackingRuntime("output",runtime,localFrame(0,[15;0],0));
            testCase.verifyEqual(output.egoYaw,0.4,AbsTol=1e-14);
            testCase.verifyEqual(runtime.yawEstimate,0.4,AbsTol=1e-14);
            testCase.verifyLessThanOrEqual(abs(output.egoYaw),output.egoYawErrorBound);
            testCase.verifyEqual(output.orientationSet.heading,0,AbsTol=1e-12);
        end

        function changingYawStateAndGainLeavesTheBodyCoreUnchanged(testCase)
            first = localRuntime(testCase.Config,testCase.Design,0.4,[15;0]);
            design = testCase.Design;
            design.yaw.correctionBandwidth = 3*design.yaw.correctionBandwidth;
            second = localRuntime(testCase.Config,design,-0.7,[15;0]);
            [~,a] = onlineNrmmTrackingRuntime("step",first,localFrame(0,[15;0],0));
            [~,b] = onlineNrmmTrackingRuntime("step",second,localFrame(0,[15;0],0));
            testCase.verifyEqual(a.egoBodyVelocity,b.egoBodyVelocity,AbsTol=0);
            testCase.verifyEqual(a.targetStates,b.targetStates,AbsTol=0);
            testCase.verifyEqual(a.relativePositionErrorBound,b.relativePositionErrorBound,AbsTol=0);
            testCase.verifyEqual(a.egoPositionInertial,b.egoPositionInertial,AbsTol=0);
            testCase.verifyGreaterThan(abs(a.egoYaw-b.egoYaw),0.5);
        end

        function standstillKeepsYawPropagationWithAnUninformativeCourse(testCase)
            cfg = testCase.Config;
            cfg.ego.domain.speedMinimum = 0;
            cfg.ego.yaw.singleTrackYawRateMismatchMaximum = 0.2;
            design = synthesizeNrmmObserverGains(cfg);
            runtime = localRuntime(cfg,design,1.2,[0;0]);
            [~,output] = onlineNrmmTrackingRuntime("step",runtime,localFrame(0,[0;0],0.1));
            testCase.verifyEqual(output.egoYaw,1.2+0.1*cfg.runtime.samplePeriod,AbsTol=1e-13);
            testCase.verifyFalse(output.yawObserverCorrectionAvailable);
            testCase.verifyEqual(output.egoYawErrorBound,pi,AbsTol=0);
            testCase.verifyTrue(output.orientationCertificateAvailable);
            testCase.verifyTrue(output.positionErrorBoundAvailable);
        end

        function inconsistentYawSetNeverResetsTheContinuousObserver(testCase)
            runtime = localRuntime(testCase.Config,testCase.Design,0.4,[15;0]);
            runtime.positionErrorBound.orientationSet = nrmmYawSet("initialize",pi,0.01);
            [~,output] = onlineNrmmTrackingRuntime("step",runtime,localFrame(0,[15;0],0));
            expected = 0.4*exp(-testCase.Design.yaw.correctionBandwidth*runtime.samplePeriod);
            testCase.verifyEqual(output.egoYaw,expected,AbsTol=1e-10);
            testCase.verifyFalse(output.orientationCertificateAvailable);
            testCase.verifyTrue(isinf(output.egoYawErrorBound));
            testCase.verifyTrue(output.positionErrorBoundAvailable);
        end

        function integratedYawCrossesTheCoordinateCutContinuouslyOnTheCircle(testCase)
            cfg = testCase.Config;
            cfg.ego.domain.speedMinimum = 0;
            cfg.ego.yaw.singleTrackYawRateMismatchMaximum = 0.2;
            design = synthesizeNrmmObserverGains(cfg);
            runtime = localRuntime(cfg,design,pi-0.001,[0;0]);
            [~,output] = onlineNrmmTrackingRuntime("step",runtime,localFrame(0,[0;0],0.1));
            testCase.verifyEqual(output.egoYaw,-pi+0.001,AbsTol=1e-12);
            testCase.verifyEqual(output.egoYawErrorBound,pi,AbsTol=0);
            testCase.verifyTrue(output.orientationCertificateAvailable);
        end
    end
end

function runtime = localRuntime(cfg,design,yaw,velocity)
    options = struct("egoInitialPosition",[0;0],"egoInitialYaw",yaw, ...
        "egoInitialBodyVelocity",velocity,"targetInitialState",[30;2;12;0;0;0]);
    runtime = onlineNrmmTrackingRuntime("initialize",cfg,options,design);
end

function frame = localFrame(time,velocity,rate)
    frame = struct("time",time,"gnssTime",time,"imuTime",time,"gyroscopeTime",time, ...
        "xGps",0,"yGps",0,"vxGps",velocity(1),"vyGps",velocity(2), ...
        "longitudinalAcceleration",0,"lateralAcceleration",0,"yawRateMeasured",rate, ...
        "radarTime",time,"radarRelativePosition",[30,2],"radarDetectionAvailable",true);
end
