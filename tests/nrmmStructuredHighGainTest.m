classdef nrmmStructuredHighGainTest < matlab.unittest.TestCase
    % Check the paper-preserving chain, global bounds, and proof qualifications.
    properties
        Config
        Design
    end
    methods (TestClassSetup)
        function prepareObserver(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            for folder = ["config","estimator","scripts"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,folder)));
            end
            testCase.Config = nrmmTrackingConfig();
            testCase.Design = synthesizeNrmmObserverGains(testCase.Config);
        end
    end
    methods (Test)
        function globalBoundsCoverAllRadialSaturationRegions(testCase)
            domain = testCase.Design.target.domain;
            bound = testCase.Design.target.lipschitzCertificate;
            stream = RandStream("mt19937ar","Seed",43);
            radii = [0,0.5*domain.speedMinimum,domain.speedMinimum*(1+[-1e-7,0,1e-7]), ...
                domain.speedMaximum*(1+[-1e-7,0,1e-7]),10*domain.speedMaximum];
            for radius = radii
                for index = 1:30
                    angle = 2*pi*rand(stream);
                    q = radius*[cos(angle);sin(angle)];
                    s = 2*domain.accelerationNormBound*randn(stream,2,1);
                    dq = randn(stream,2,1)*10^(4*rand(stream)-3);
                    ds = randn(stream,2,1)*10^(4*rand(stream)-3);
                    phi = localPhi(q,s,domain);
                    testCase.verifyLessThanOrEqual(norm(localPhi(q+dq,s,domain)-phi), ...
                        bound.phiVelocity*norm(dq)+1e-11);
                    testCase.verifyLessThanOrEqual(norm(localPhi(q,s+ds,domain)-phi), ...
                        bound.phiAcceleration*norm(ds)+1e-11);
                end
            end
        end

        function nonlinearErrorFieldSatisfiesStructuredLyapunovDecay(testCase)
            target = testCase.Design.target;
            metric = kron(target.lyapunovMatrix,eye(2));
            bandwidth = target.bandwidth;
            scale = kron(diag([1,1/bandwidth,1/bandwidth^2]),eye(2));
            linear = bandwidth*kron(target.closedLoopMatrix,eye(2));
            rotation = kron(eye(3),[0,-1;1,0]);
            stream = RandStream("mt19937ar","Seed",53);
            for index = 1:200
                q = 15*randn(stream,2,1);
                s = 3*randn(stream,2,1);
                physicalError = 10*randn(stream,6,1);
                epsilon = scale*physicalError;
                deltaPhi = localPhi(q,s,target.domain) ...
                    -localPhi(q-physicalError(3:4),s-physicalError(5:6),target.domain);
                derivative = linear*epsilon+[zeros(4,1);deltaPhi/bandwidth^2] ...
                    -5*randn(stream)*rotation*epsilon;
                value = epsilon.'*metric*epsilon;
                actualDerivative = 2*epsilon.'*metric*derivative;
                testCase.verifyLessThanOrEqual(actualDerivative, ...
                    -2*target.lambda*value+1e-8*max(1,value));
            end
        end

        function eachComponentBoundIsAttainedOnItsEllipsoid(testCase)
            target = testCase.Design.target;
            inverseMetric = target.lyapunovMatrix\eye(3);
            scale = [1;1/target.bandwidth;1/target.bandwidth^2];
            for index = 1:3
                epsilon = inverseMetric(:,index)/sqrt(inverseMetric(index,index));
                testCase.verifyEqual(epsilon.'*target.lyapunovMatrix*epsilon,1,AbsTol=1e-12);
                testCase.verifyEqual(abs(epsilon(index))/scale(index), ...
                    target.componentConversion(index),RelTol=1e-12);
            end
        end

        function modelVariationEntersTheLastChainEquationAsBoundedJerk(testCase)
            cfg = testCase.Config;
            cfg.target.model.scalarAccelerationRateMaximum = 0.3;
            cfg.target.model.curvatureRateMaximum = 0.002;
            design = synthesizeNrmmObserverGains(cfg);
            speed = cfg.target.domain.speedMaximum;
            direction = [cos(0.7);sin(0.7)];
            modelError = 0.3*direction+0.002*speed^2*[-direction(2);direction(1)];
            testCase.verifyEqual(norm(modelError),design.target.modelJerkMaximum,RelTol=1e-12);
            testCase.verifyGreaterThan(design.target.disturbanceBound, ...
                design.target.disturbanceCoefficients.modelJerk*norm(modelError));
        end

        function radarPredictorResidualRotatesWithTheEgoFrame(testCase)
            cfg = testCase.Config;
            runtime = localRuntime(cfg,testCase.Design,1);
            frame = localFrame;
            frame.yawRateMeasured = 0.12;
            initialResidual = [0.1;-0.08];
            frame.radarRelativePosition = (runtime.targetState(1:2)+initialResidual).';
            [runtime,~] = onlineNrmmTrackingRuntime("step",runtime,frame);
            actual = runtime.targetOutputPredictor-runtime.targetState(1:2,:);
            generator = -testCase.Design.target.innovationGains(1)*eye(2) ...
                -frame.yawRateMeasured*[0,-1;1,0];
            expected = expm(cfg.runtime.samplePeriod*generator)*initialResidual;
            testCase.verifyEqual(actual,expected,AbsTol=1e-6);
        end

        function resetOneTrackPreservesEgoAndOtherTracks(testCase)
            runtime = localRuntime(testCase.Config,testCase.Design,2);
            runtime.lastRadarTime = [0;0];
            replacement = [40;3;12;2;0.1;0.4];
            changed = onlineNrmmTrackingRuntime("resetTarget",runtime,1,replacement.');
            testCase.verifyEqual(changed.targetState(:,1),replacement,AbsTol=0);
            testCase.verifyEqual(changed.targetOutputPredictor(:,1),replacement(1:2),AbsTol=0);
            testCase.verifyEqual(changed.targetState(:,2),runtime.targetState(:,2),AbsTol=0);
            testCase.verifyEqual(changed.bodyVelocityEstimate,runtime.bodyVelocityEstimate,AbsTol=0);
            testCase.verifyEqual(changed.yawEstimate,runtime.yawEstimate,AbsTol=0);
            testCase.verifyEqual(changed.currentTime,runtime.currentTime,AbsTol=0);
            testCase.verifyTrue(isnan(changed.lastRadarTime(1)));
            testCase.verifyEqual(changed.lastRadarTime(2),0,AbsTol=0);
        end

        function outputUsesOrientationSetsWithoutAChartCondition(testCase)
            runtime = localRuntime(testCase.Config,testCase.Design,1);
            runtime.yawEstimate = pi-1e-4;
            output = onlineNrmmTrackingRuntime("output",runtime,localFrame);
            testCase.verifyTrue(output.orientationCertificateAvailable);
            testCase.verifyFalse(isfield(output,"yawInnovationChartCompatible"));
            testCase.verifyFalse(output.certificateScope.sampledImplementationCertified);
            testCase.verifyEqual(output.targetStates,runtime.targetState.',AbsTol=0);
        end

        function heldOutNoisyScenarioRetainsUsefulAccelerationEstimation(testCase)
            design = testCase.Design;
            result = runOnlineNrmmComplexManeuverScenario("Plot",false,"Report",false, ...
                "Seed",17,"NoiseModel","boundedUniform","DesignFunction",@(~) design);
            testCase.verifyTrue(result.metrics.allSamplesFinite);
            testCase.verifyTrue(result.metrics.truthOperatingDomainValid);
            testCase.verifyLessThan(result.metrics.relativePositionRmse,0.08);
            testCase.verifyLessThan(result.metrics.targetVelocityRmse,1);
            testCase.verifyLessThan(result.metrics.targetAccelerationRmse,3);
            testCase.verifyFalse(result.metrics.digitalErrorBoundCertified);
        end
    end
end

function phi = localPhi(q,s,domain)
    derivative = nrmmTargetTrackerDerivative([zeros(2,1);q;s], ...
        struct("bodyVelocity",zeros(2,1),"yawRate",0),domain);
    phi = derivative(5:6);
end

function runtime = localRuntime(cfg,design,count)
    physical = repmat([30;2;15;0;0;0],1,count);
    options = struct("targetCount",count,"egoInitialPosition",zeros(2,1), ...
        "egoInitialYaw",0,"egoInitialBodyVelocity",[15;0],"targetInitialState",physical);
    if count > 1
        options.targetIdentifiers = "track-"+string((1:count).');
    end
    runtime = onlineNrmmTrackingRuntime("initialize",cfg,options,design);
end

function frame = localFrame()
    frame = struct("time",0,"xGps",0,"yGps",0,"vxGps",15,"vyGps",0, ...
        "longitudinalAcceleration",0,"lateralAcceleration",0,"yawRateMeasured",0, ...
        "radarRelativePosition",[30,2],"radarDetectionAvailable",true);
end
