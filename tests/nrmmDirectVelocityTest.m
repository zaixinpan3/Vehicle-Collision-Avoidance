classdef nrmmDirectVelocityTest < matlab.unittest.TestCase
    % Behavioral checks of the direct inverse and common-error certificate.
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
        function exactKinematicsRecoverTheForwardVelocity(testCase)
            design = testCase.Design;
            truth = 12*[cos(0.03);sin(0.03)];
            measured = nrmmKinematicVelocityMeasurement([0;12], ...
                truth(2)/design.yaw.courseModel.rearAxleDistance,design);
            testCase.verifyEqual(measured,truth,AbsTol=1e-13);
        end

        function longitudinalFloorDoesNotClipTheLateralMeasurement(testCase)
            design = testCase.Design;
            rate = 10;
            measured = nrmmKinematicVelocityMeasurement([1;0],rate,design);
            testCase.verifyEqual(measured, ...
                [cos(design.yaw.courseModel.sideslipDomainMaximum); ...
                design.yaw.courseModel.rearAxleDistance*rate],AbsTol=1e-13);
        end

        function zeroSpeedRequiresNoRawInversionOrYawInformation(testCase)
            cfg = testCase.Config;
            cfg.ego.domain.speedMinimum = 0;
            design = synthesizeNrmmObserverGains(cfg);
            [measurement,feasible] = nrmmKinematicVelocityMeasurement([0;0],0,design);
            testCase.verifyEqual(measurement,[0;0],AbsTol=0);
            testCase.verifyTrue(feasible.consistent);
            testCase.verifyTrue(isfinite(design.ultimateBounds.bodyVelocity));
            testCase.verifyGreaterThan(design.target.domain.speedMinimum,0);
        end

        function impossibleKinematicsAreReportedWithoutAnUndefinedInput(testCase)
            [measurement,feasible] = nrmmKinematicVelocityMeasurement([10;0],20,testCase.Design);
            testCase.verifyTrue(all(isfinite(measurement)));
            testCase.verifyFalse(feasible.consistent);
        end

        function runtimeTracksAtStandstillWithAnUninformativeYawSet(testCase)
            cfg = testCase.Config;
            cfg.ego.domain.speedMinimum = 0;
            options = struct("egoInitialPosition",[0;0], ...
                "egoInitialBodyVelocity",[0;0],"targetInitialState",[20;2;12;0;0;0]);
            runtime = onlineNrmmTrackingRuntime("initialize",cfg,options);
            frame = struct("time",0,"xGps",0,"yGps",0,"vxGps",0,"vyGps",0, ...
                "longitudinalAcceleration",0,"lateralAcceleration",0,"yawRateMeasured",0, ...
                "radarRelativePosition",[20,2]);
            [changed,output] = onlineNrmmTrackingRuntime("step",runtime,frame);
            testCase.verifyEqual(changed.bodyVelocityEstimate,[0;0],AbsTol=0);
            testCase.verifyTrue(all(isfinite(changed.targetState)));
            testCase.verifyTrue(output.positionErrorBoundAvailable);
            testCase.verifyTrue(output.orientationCertificateAvailable);
            testCase.verifyEqual(output.egoYawErrorBound,pi,AbsTol=0);
            testCase.verifyFalse(output.yawCourseChannelValid);
        end

        function gyroCancelsExactlyInTheStraightMatchedCase(testCase)
            design = testCase.Design;
            speed = 12;
            design.yaw.courseModel.sideslipDomainMaximum = 0;
            design.velocity.gain = speed/design.yaw.courseModel.rearAxleDistance;
            estimate = struct("bodyVelocity",[speed;0],"position",[0;0],"targetState",zeros(6,0));
            input = struct("yawRate",0.1,"gnssVelocity",[speed;0], ...
                "bodyAcceleration",[0;0],"positionReference",[0;0], ...
                "radarReference",zeros(2,0),"radarAvailable",false(0,1));
            derivative = nrmmObserverVectorField(estimate,input,design);
            testCase.verifyEqual(derivative.bodyVelocity,[0;0],AbsTol=1e-13);
        end

        function finiteNoiseAndFloorCrossingsRespectTheCombinedBound(testCase)
            [maximumExcess,floorCount] = localFiniteErrorSweep(testCase.Design);
            testCase.verifyLessThanOrEqual(maximumExcess,1e-10);
            testCase.verifyGreaterThan(floorCount,0);
        end

        function combinedGyroBoundIsStrictlySmallerForEveryTestedCone(testCase)
            improvement = localGyroComparison(testCase.Design);
            testCase.verifyGreaterThan(min(improvement),0);
        end

        function straightConeUsesSignedSpeedDifferences(testCase)
            model = testCase.Design.yaw.courseModel;
            model.sideslipDomainMaximum = 0;
            gain = 7/model.rearAxleDistance;
            bound = nrmmVelocityDisturbanceBound(gain,[0,20],model,testCase.Design.sensors);
            testCase.verifyEqual(bound.gyroCoefficient,13,AbsTol=1e-12);
        end

        function oldValidCertificateDominatesAtTheSameGain(testCase)
            [oldBound,newBound] = localOriginalComparison(testCase.Config,testCase.Design);
            testCase.verifyLessThan(newBound,oldBound);
        end

        function targetStillRequiresAPositiveTrueSpeedFloor(testCase)
            cfg = testCase.Config;
            cfg.target.domain.speedMinimum = 0;
            testCase.verifyError(@() synthesizeNrmmObserverGains(cfg), ...
                "synthesizeNrmmObserverGains:invalidPositiveScalar");
        end

        function anEgoDomainConsistingOnlyOfStandstillStillHasACertificate(testCase)
            cfg = testCase.Config;
            cfg.ego.domain.speedMinimum = 0;
            cfg.ego.domain.speedMaximum = 0;
            design = synthesizeNrmmObserverGains(cfg);
            testCase.verifyGreaterThan(design.velocity.gain,0);
            testCase.verifyTrue(isfinite(design.velocity.disturbanceBound));
            testCase.verifySize(design.coreIss.matrix,[2,2]);
        end
    end
end

function [maximumExcess,floorCount] = localFiniteErrorSweep(design)
    stream = RandStream("mt19937ar","Seed",109);
    design.operatingDomain.egoSpeedMinimum = 0;
    design.yaw.courseModel.sideslipDomainMaximum = 1.2;
    design.yaw.courseModel.singleTrackYawRateMismatchMaximum = 0.4;
    design.sensors.velocityNoiseMaximum = 1.5;
    design.sensors.gyroscopeNoiseMaximum = 0.3;
    design.sensors.accelerometerNoiseMaximum = 0.2;
    certificate = nrmmVelocityDisturbanceBound(design.velocity.gain,[0,20], ...
        design.yaw.courseModel,design.sensors);
    maximumExcess = -Inf;
    floorCount = 0;
    for index = 1:600
        speed = 20*rand(stream);
        angle = 1.2*(2*rand(stream)-1);
        truth = speed*[cos(angle);sin(angle)];
        noiseAngle = 2*pi*rand(stream);
        gnssNoise = 1.5*rand(stream)*[cos(noiseAngle);sin(noiseAngle)];
        gyro = 0.3*(2*rand(stream)-1);
        mismatch = 0.4*(2*rand(stream)-1);
        rate = truth(2)/design.yaw.courseModel.rearAxleDistance+mismatch+gyro;
        [measurement,feasible] = nrmmKinematicVelocityMeasurement(truth+gnssNoise,rate,design);
        accelerationNoise = 0.2*[cos(noiseAngle);sin(noiseAngle)];
        forcing = accelerationNoise+design.velocity.gain*(measurement-truth) ...
            -gyro*[-truth(2);truth(1)];
        maximumExcess = max(maximumExcess,norm(forcing)-certificate.disturbanceBound);
        floorCount = floorCount+feasible.floorActive;
    end
end

function improvement = localGyroComparison(design)
    improvement = zeros(21,11);
    for coneIndex = 1:21
        model = design.yaw.courseModel;
        model.sideslipDomainMaximum = (coneIndex-1)/20*1.55;
        for gainIndex = 1:11
            gain = 10^((gainIndex-1)/2-2);
            bound = nrmmVelocityDisturbanceBound(gain,[0,20],model,design.sensors);
            improvement(coneIndex,gainIndex) = bound.separatedGyroCoefficient-bound.gyroCoefficient;
        end
    end
    improvement = improvement(:);
end

function [oldBound,newBound] = localOriginalComparison(cfg,design)
    a = cfg.ego.domain.speedMinimum-cfg.measurement.gps.velocityNoiseMaximum;
    b = a-cfg.measurement.gps.velocityNoiseMaximum;
    rate = cfg.ego.domain.yawRateMaximum+cfg.measurement.gyroscope.noiseMaximum;
    distance = cfg.ego.yaw.rearAxleDistance;
    lipschitz = 1/sqrt(1-max(sin(cfg.ego.yaw.sideslipDomainMaximum),distance*rate/a)^2);
    course = asin(cfg.measurement.gps.velocityNoiseMaximum/a)+distance*lipschitz/b ...
        *(cfg.measurement.gyroscope.noiseMaximum+cfg.ego.yaw.singleTrackYawRateMismatchMaximum ...
        + rate/a*cfg.measurement.gps.velocityNoiseMaximum);
    gain = design.velocity.gain;
    oldBound = cfg.measurement.gps.velocityNoiseMaximum ...
        + cfg.ego.domain.speedMaximum*(course+cfg.measurement.gyroscope.noiseMaximum/gain) ...
        +(cfg.measurement.imu.noiseMaximum+cfg.ego.domain.speedMaximum ...
        *cfg.measurement.gyroscope.noiseMaximum)/gain;
    newBound = design.ultimateBounds.bodyVelocity;
end
