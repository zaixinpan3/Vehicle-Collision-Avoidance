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

        function orientationIsAnOutputSetWithNoCorrectionGain(testCase)
            design = testCase.Design;
            testCase.verifyFalse(isfield(design.yaw,"correctionBandwidth"));
            testCase.verifyEqual(design.yaw.maximumRadius,pi,AbsTol=0);
            testCase.verifyEqual(design.yaw.representation,"propagated-intersected-circle-set");
        end

        function coreHasNoYawInputOrCoupling(testCase)
            design = testCase.Design;
            testCase.verifyFalse(isfield(design.coupling,"velocityYawCoupling"));
            testCase.verifyEqual(design.coreIss.matrix, ...
                [-design.velocity.gain,0;design.coupling.targetVelocityCoupling,-design.target.lambda], ...
                AbsTol=1e-14);
        end

        function velocityDisturbanceBoundAssembles(testCase)
            % No bias state and no Lyapunov weighting: the comparison
            % input is the disturbance bound itself.
            cfg = testCase.Cfg;
            design = testCase.Design;

            c = cos(cfg.ego.yaw.sideslipDomainMaximum);
            expectedBound = cfg.measurement.imu.noiseMaximum ...
                + design.velocity.gain/c*cfg.measurement.gps.velocityNoiseMaximum ...
                + design.velocity.disturbanceCertificate.gyroCoefficient ...
                    *cfg.measurement.gyroscope.noiseMaximum ...
                + design.velocity.gain*cfg.ego.yaw.rearAxleDistance/c ...
                    *cfg.ego.yaw.singleTrackYawRateMismatchMaximum;
            testCase.verifyEqual(design.velocity.disturbanceBound, ...
                expectedBound, RelTol=1.0e-12);
            testCase.verifyEqual(design.coreIss.input(1), ...
                expectedBound, RelTol=1.0e-12);
        end

        function targetDisturbanceBoundAssembles(testCase)
            cfg = testCase.Cfg;
            design = testCase.Design;
            bandwidth = design.target.bandwidth;
            domain = design.target.domain;

            metric = design.target.lyapunovMatrix;
            scaledDomain = [domain.relativePositionMaximum; ...
                domain.speedMaximum/bandwidth;domain.accelerationNormBound/bandwidth^2];
            expectedInputScale = sqrt(scaledDomain.'*abs(metric)*scaledDomain);
            testCase.verifyEqual(design.target.inputScale,expectedInputScale,RelTol=1e-12);
            injection = design.target.injectionVector;
            expectedBound = expectedInputScale*design.sensors.gyroscopeNoiseMaximum ...
                +bandwidth*sqrt(injection.'*metric*injection)*cfg.measurement.radar.noiseMaximum ...
                +sqrt(metric(3,3))/bandwidth^2*design.target.modelJerkMaximum;
            testCase.verifyEqual(design.target.disturbanceBound,expectedBound,RelTol=1e-12);
        end

        function coreComparisonInputAssembles(testCase)
            design = testCase.Design;

            expectedInput = [ ...
                design.velocity.disturbanceBound; ...
                design.target.disturbanceBound];
            testCase.verifyEqual(design.coreIss.input, ...
                expectedInput, RelTol=1.0e-12);
        end

        function copositiveDerivativeIdentityHoldsPointwise(testCase)
            certificate = testCase.Design.coreIss;
            comparisonState = [0.4; 1.0];
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
                ["bodyVelocity"; "target"]);
            testCase.verifySize(certificate.matrix, [2, 2]);
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
                gyroscopeDesign.ultimateBounds.bodyVelocity, ...
                baseline.ultimateBounds.bodyVelocity);
        end

        function containmentScopeDeclaresEstimatedStatesOnly(testCase)
            scope = testCase.Design.operatingDomain.containmentScope;
            testCase.verifySubstring(scope, "estimated");
            testCase.verifySubstring(scope, "offline audit");
        end
    end
end
