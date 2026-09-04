classdef nrmmCascadeCertificateTest < matlab.unittest.TestCase
    % nrmmCascadeCertificateTest Verify the disturbance and ISS bookkeeping.
    %
    % These tests check that deterministic sensor-error bounds propagate
    % through the positive triangular cascade exactly as stated: channel
    % disturbance bounds, the explicit copositive identity, downstream
    % forward substitution, and monotone noise dependence.

    properties
        Cfg
        Design
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            repoRoot = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repoRoot, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repoRoot, "estimator")));
        end
    end

    methods (TestMethodSetup)
        function synthesizeDefaultDesign(testCase)
            testCase.Cfg = nrmmTrackingConfig();
            testCase.Design = synthesizeNrmmObserverGains(testCase.Cfg);
        end
    end

    methods (Test)
        function sensorInputErrorAndYawRateDomainCompose(testCase)
            cfg = testCase.Cfg;
            design = testCase.Design;

            % Without a gyroscope bias the input error is the noise
            % bound alone.
            gyroscopeNoise = cfg.measurement.gyroscope.noiseMaximum;
            testCase.verifyEqual( ...
                design.sensors.gyroscopeNoiseMaximum, ...
                gyroscopeNoise, RelTol=1.0e-12);
            testCase.verifyEqual( ...
                design.operatingDomain.measuredYawRateMaximum, ...
                cfg.ego.domain.yawRateMaximum+gyroscopeNoise, ...
                RelTol=1.0e-12);
        end

        function yawMeasurementConstantsMatchDefinitions(testCase)
            design = testCase.Design;

            course = design.yaw.uniformCourseCertificate;
            testCase.verifyEqual( ...
                design.yaw.uniformCourseRadiusMaximum, ...
                course.directionRadius+course.sideslipErrorMaximum, ...
                RelTol=1.0e-12);
            testCase.verifyLessThan( ...
                design.yaw.uniformCourseRadiusMaximum, pi);
        end

        function yawDisturbanceBoundAssembles(testCase)
            design = testCase.Design;

            expectedBound = design.sensors.gyroscopeNoiseMaximum ...
                + design.yaw.correctionBandwidth ...
                    * design.yaw.uniformCourseRadiusMaximum;
            testCase.verifyEqual(design.yaw.disturbanceBound, ...
                expectedBound, RelTol=1.0e-12);
        end

        function velocityDisturbanceBoundAssembles(testCase)
            % No bias state and no Lyapunov weighting: the comparison
            % input is the disturbance bound itself.
            cfg = testCase.Cfg;
            design = testCase.Design;

            expectedBound = cfg.ego.domain.speedMaximum ...
                    * design.sensors.gyroscopeNoiseMaximum ...
                + cfg.measurement.imu.noiseMaximum ...
                + design.velocity.gain ...
                    * cfg.measurement.gps.velocityNoiseMaximum;
            testCase.verifyEqual(design.velocity.disturbanceBound, ...
                expectedBound, RelTol=1.0e-12);
            testCase.verifyEqual(design.coreIss.input(2), ...
                expectedBound, RelTol=1.0e-12);
        end

        function targetDisturbanceBoundAssembles(testCase)
            cfg = testCase.Cfg;
            design = testCase.Design;
            bandwidth = design.target.bandwidth;
            domain = design.target.domain;

            expectedInputScale = sqrt( ...
                bandwidth^4*domain.relativePositionMaximum^2 ...
                + bandwidth^2*domain.speedMaximum^2 ...
                + domain.accelerationNormBound^2);
            testCase.verifyEqual(design.target.inputScale, ...
                expectedInputScale, RelTol=1.0e-12);

            expectedBound = design.target.gainConstant ...
                * (expectedInputScale ...
                    * design.sensors.gyroscopeNoiseMaximum ...
                + norm(design.target.injectionVector)*bandwidth^3 ...
                    * cfg.measurement.radar.noiseMaximum);
            testCase.verifyEqual(design.target.disturbanceBound, ...
                expectedBound, RelTol=1.0e-12);
        end

        function coreComparisonInputAssembles(testCase)
            design = testCase.Design;

            expectedInput = [ ...
                design.yaw.disturbanceBound; ...
                design.velocity.disturbanceBound; ...
                design.target.disturbanceBound];
            testCase.verifyEqual(design.coreIss.input, ...
                expectedInput, RelTol=1.0e-12);
        end

        function copositiveDerivativeIdentityHoldsPointwise(testCase)
            certificate = testCase.Design.coreIss;
            comparisonState = [0.2; 0.4; 1.0];
            comparisonDerivative = certificate.matrix*comparisonState ...
                + certificate.input;
            actualDerivative = certificate.weights.' ...
                * comparisonDerivative;
            exactDerivative = -sum(comparisonState) ...
                + certificate.inputProjection;
            scalarIssUpperBound = -certificate.decayRate ...
                    * (certificate.weights.'*comparisonState) ...
                + certificate.inputProjection;

            testCase.verifyEqual(actualDerivative, exactDerivative, ...
                RelTol=1.0e-12);
            testCase.verifyLessThanOrEqual(actualDerivative, ...
                scalarIssUpperBound);
        end

        function coreExcludesOutputFilter(testCase)
            certificate = testCase.Design.coreIss;

            testCase.verifyEqual(certificate.stateOrder, ...
                ["yaw"; "bodyVelocity"; "target"]);
            testCase.verifySize(certificate.matrix, [3, 3]);
            testCase.verifyFalse(any(certificate.stateOrder ...
                == "position"));
        end

        function ultimateBoundsIncreaseWithNoiseBounds(testCase)
            baseline = testCase.Design;

            noisierRadar = testCase.Cfg;
            noisierRadar.measurement.radar.noiseMaximum = ...
                2.0*noisierRadar.measurement.radar.noiseMaximum;
            radarDesign = synthesizeNrmmObserverGains(noisierRadar);
            testCase.verifyGreaterThan( ...
                radarDesign.ultimateBounds.targetLyapunov, ...
                baseline.ultimateBounds.targetLyapunov);
            testCase.verifyGreaterThan( ...
                radarDesign.ultimateBounds.relativePosition, ...
                baseline.ultimateBounds.relativePosition);

            noisierGyroscope = testCase.Cfg;
            noisierGyroscope.measurement.gyroscope.noiseMaximum = ...
                2.0*noisierGyroscope.measurement.gyroscope.noiseMaximum;
            gyroscopeDesign = synthesizeNrmmObserverGains(noisierGyroscope);
            testCase.verifyGreaterThan( ...
                gyroscopeDesign.ultimateBounds.yaw, ...
                baseline.ultimateBounds.yaw);
        end

        function containmentScopeDeclaresEstimatedStatesOnly(testCase)
            scope = testCase.Design.operatingDomain.containmentScope;
            testCase.verifySubstring(scope, "estimated");
            testCase.verifySubstring(scope, "offline audit");
        end
    end
end
