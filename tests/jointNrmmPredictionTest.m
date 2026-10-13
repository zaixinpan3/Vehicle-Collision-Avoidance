classdef jointNrmmPredictionTest < matlab.unittest.TestCase
    properties (SetAccess=private)
        joint
        relative
    end
    methods (TestClassSetup)
        function runActualObserverProbes(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.joint = verifyJointNrmmPrediction("",Speeds=15,HorizonSteps=80,WorldCount=2);
            testCase.relative = verifyNrmmRelativeFeedback("",80);
        end
    end
    methods (Test)
        function realObserverFeedbackPropagatesAVaryingFiniteHorizonTube(testCase)
            result = testCase.joint;row = result.cases;
            testCase.verifyTrue(result.nrmmObserverExecuted);
            testCase.verifyEmpty(row.jointLinearTube);
            testCase.verifyGreaterThan(row.finiteHorizonTube.coordinateRadius(1,end), ...
                row.finiteHorizonTube.coordinateRadius(1,1));
            testCase.verifyLessThan(row.finiteHorizonTube.coordinateRadius(1:2,end), ...
                row.openLoopInitialRadius(1:2,end));
            testCase.verifyLessThan(row.maximumAbsoluteInputFromFiniteHorizonTube(2),1);
        end
        function terminalSupportsAreAttainedByAdmittedLinearNoiseHistories(testCase)
            tube = testCase.joint.cases.finiteHorizonTube;
            testCase.verifyEqual(tube.attainedTerminalRadius,tube.coordinateRadius(:,end),AbsTol=1e-9);
            testCase.verifyGreaterThan(min(tube.sensorContributionAtTail),0);
        end
        function measuredFeedbackKeepsTheNonlinearReplayAndObserverBoundsSmall(testCase)
            row = testCase.joint.cases;replay = row.replay;
            testCase.verifyLessThan(max(replay.closedLoopPeak(1:2)),.1);
            testCase.verifyLessThanOrEqual(replay.closedLoopPeak,row.finiteHorizonTube.maximumCoordinateRadius);
            testCase.verifyEqual(replay.unavailableObserverBoundSamples,0);
            testCase.verifyEqual(replay.uncontainedObserverBoundSamples,0);
            testCase.verifyLessThan(replay.maximumAbsoluteInput(2),1);
        end
        function initialInformationBranchesReceiveTheSameCurrentInput(testCase)
            first = testCase.joint.cases.replay.firstInputs;
            testCase.verifyEqual(first(:,1),first(:,2),AbsTol=0);
            first = testCase.relative.firstInputs;
            testCase.verifyEqual(first(:,1),first(:,2),AbsTol=0);
        end
        function targetRelativeObserverFeedbackRetainsUsefulTrajectoryCorrelation(testCase)
            result = testCase.relative;
            testCase.verifyGreaterThan(result.targetPositionDiameterMeters,.7);
            testCase.verifyLessThan(result.terminalRelativePositionDiameterMeters,.01);
            testCase.verifyLessThan(max(result.peakRelativeTrackingErrorMeters),.04);
            testCase.verifyGreaterThan(result.minimumSampledRectangleClearanceMeters,3);
            testCase.verifyEqual(result.unavailableTargetBoundSamples,0);
            testCase.verifyEqual(result.uncontainedAvailableTargetBoundSamples,0);
        end
        function probesDistinguishNumericalEvidenceFromANonlinearCertificate(testCase)
            result = testCase.joint;
            testCase.verifyFalse(result.productionControllerChanged);
            testCase.verifyFalse(result.nonlinearRobustCertificate);
            testCase.verifyFalse(testCase.relative.nonlinearRobustCertificate);
            testCase.verifyLessThan(result.cases.finiteDifferenceRelativeChange,.005);
        end
    end
end
