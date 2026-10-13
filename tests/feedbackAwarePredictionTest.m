classdef feedbackAwarePredictionTest < matlab.unittest.TestCase
    properties (SetAccess=private)
        results
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.results=verifyFeedbackAwarePrediction();
        end
    end
    methods (Test)
        function fullTrackingFeedbackRemovesTheUncontrolledLongitudinalMode(testCase)
            result=testCase.results;
            testCase.verifyEqual(result.oldFullStateSpectralRadius,1,AbsTol=1e-9);
            testCase.verifyLessThan(result.newFullStateSpectralRadius,.99);
        end
        function boundedMeasurementFeedbackAdmitsAFixedLinearTube(testCase)
            result=testCase.results;tube=result.linearTube;
            testCase.verifyLessThan(tube.contraction,1);
            testCase.verifyLessThanOrEqual(result.linearStressPeak,tube.coordinateRadius*(1+1e-9));
            testCase.verifyLessThan(tube.coordinateRadius(2),result.openLoopRadius(2,end)/5);
            testCase.verifyGreaterThan(tube.coordinateRadius(2),result.measurementErrorRadius(2));
        end
        function nonlinearFeedbackProbesAvoidTheOpenPredictionGrowth(testCase)
            result=testCase.results;
            testCase.verifyTrue(result.nonlinearFeedback.inputAvailable);
            testCase.verifyLessThan(result.nonlinearFeedback.peakStateError(2),result.openLoopRadius(2,end)/5);
            testCase.verifyFalse(result.nonlinearRobustCertificate);
            testCase.verifyFalse(result.nrmmObserverExecuted);
        end
        function targetRelativeFeedbackPreservesCorrelationBetweenMovingTrajectories(testCase)
            result=testCase.results.targetRelative;
            testCase.verifyEqual(result.firstInputs(:,1),result.firstInputs(:,2),AbsTol=0);
            testCase.verifyEqual(result.futureSpeedErrorBoundMetersPerSecond, ...
                result.currentSpeedErrorBoundMetersPerSecond,AbsTol=0);
            testCase.verifyGreaterThan(result.targetPositionDiameterMeters,5);
            testCase.verifyGreaterThan(result.egoPositionDiameterMeters,5);
            testCase.verifyLessThan(result.relativePositionDiameterMeters,.01);
            testCase.verifyLessThan(max(result.peakRelativeTrackingErrorMeters),.03);
            testCase.verifyGreaterThan(result.minimumSampledRectangleDistanceMeters,1);
            testCase.verifyLessThan(result.independentMarginalLongitudinalGapMeters,0);
        end
        function fullTrackingFeedbackPreventsVelocityBiasFromAccumulatingAsPositionError(testCase)
            result=testCase.results.constantVelocityBias;
            testCase.verifyGreaterThan(abs(result.oldGainPositionErrorMeters(end)),1);
            testCase.verifyLessThan(abs(result.fullGainPositionErrorMeters(end)),.1);
            testCase.verifyEqual(result.fullGainPositionErrorMeters(end), ...
                result.fullGainSteadyPositionErrorMeters,AbsTol=1e-8);
        end
        function jointRelativeTubeBoundsUncontrolledTargetMotionWithoutGrowingWithTime(testCase)
            result=testCase.results.relativeLinearTube;
            testCase.verifyGreaterThan(result.longitudinalRectangleGapLowerBoundMeters, ...
                result.requiredSafetyMarginMeters);
            testCase.verifyLessThan(result.fixedRelativeCoordinateRadius(1),.5);
            testCase.verifyLessThan(result.maximumAbsoluteInputFromLinearTube(2),1);
            testCase.verifyTrue(result.coversAllAdmittedLinearMeasurementSequences);
            testCase.verifyFalse(result.nonlinearCertificate);
        end
    end
end
