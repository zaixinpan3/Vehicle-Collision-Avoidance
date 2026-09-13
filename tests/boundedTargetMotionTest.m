classdef boundedTargetMotionTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (Test)
        function nonzeroJerkAndYawAccelerationReceiveASafetyCertificate(testCase)
            [ego,target,road,cfg] = localFixture();
            [~,~,p,c] = collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(p.metadata.planCertified);
            testCase.verifyFalse(p.metadata.exactPredictionAssumptionsHold);
            testCase.verifyEqual(c.encounters.contract.jerkBound,[0.1;0],AbsTol=0);
            testCase.verifyEqual(c.encounters.contract.yawAccelerationBound,0.05,AbsTol=0);
            testCase.verifyEqual(p.metadata.pcbfValue,0,AbsTol=0);
        end
        function actualChangingAccelerationAndYawRateConditionTheWitness(testCase)
            [ego,target,road,cfg,c] = localContinuation();
            [~,~,p,next] = collisionAvoidanceController(ego,target,road,cfg,c);
            testCase.verifyTrue(p.metadata.candidateVerified);
            testCase.verifyGreaterThan(next.encounters.center(5),0);
            testCase.verifyGreaterThan(next.encounters.center(8),0);
            testCase.verifyLessThanOrEqual(p.metadata.pcbfDescentResidual,0);
        end
        function aMeasurementOutsideTheMotionBoundCannotValidateTheWitness(testCase)
            [ego,target,road,cfg,c] = localContinuation();
            target.targetAccelerationInertial(1) = 1;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,c), ...
                'collisionAvoidanceController:inconsistentObservation');
        end
        function largerFutureBoundsReceiveAFreshCertificate(testCase)
            [ego,target,road,cfg,c] = localContinuation();
            target.predictionMotion.jerkBound(1) = 0.2;
            [~,~,p,next] = collisionAvoidanceController(ego,target,road,cfg,c);
            testCase.verifyTrue(p.metadata.planCertified);
            testCase.verifyTrue(p.metadata.motionBoundsIncreased);
            testCase.verifyFalse(p.metadata.candidateVerified);
            testCase.verifyTrue(isnan(p.metadata.pcbfDescentResidual));
            testCase.verifyEqual(next.encounters.contract.jerkBound,[0.2;0],AbsTol=0);
        end
        function failedReadmissionCannotExecuteTheOldSmallerBoundWitness(testCase)
            [ego,target,road,cfg,c] = localContinuation();
            target.predictionMotion.jerkBound(1) = 0.2;
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,c), ...
                'collisionAvoidanceController:noCertifiedContinuation');
        end
        function newFutureBoundsCannotExcuseAPastObservationViolation(testCase)
            [ego,target,road,cfg,c] = localContinuation();
            target.predictionMotion.jerkBound(1) = 20;
            target.targetAccelerationInertial(1) = 1;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,c), ...
                'collisionAvoidanceController:inconsistentObservation');
        end
        function smallerBoundsRemainCoveredByTheCarriedContract(testCase)
            [ego,target,road,cfg,c] = localContinuation();
            target.predictionMotion.jerkBound(1) = 0.09;
            [~,~,p,next] = collisionAvoidanceController(ego,target,road,cfg,c);
            testCase.verifyTrue(p.metadata.candidateVerified);
            testCase.verifyEqual(next.encounters.contract.jerkBound,c.encounters.contract.jerkBound,AbsTol=0);
        end
        function tinyTwoAxisJerkIsATerminalLimitationRatherThanAnExactInputRule(testCase)
            [ego,target,road,cfg] = localFixture();
            target.predictionMotion.jerkBound = [1e-12;1e-12];
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:unboundedTargetSupport');
        end
        function boundedFlowContainsIndependentChangingMotion(testCase)
            [ego,target,road,cfg] = localFixture();
            [~,lane,~,parsed] = readPlanningInputs(ego,target,road,cfg);
            encounter = targetPrediction.admitOnline(parsed,0,lane,cfg);
            time = linspace(0,2,51);
            [center,radius] = targetPrediction.finiteFlow(encounter,time);
            truth = [15+0.08*time.^3/6;-4+32*time;0.08*time.^2/2;32+0*time; ...
                0.08*time;0*time;pi/2+0.04*time.^2/2;0.04*time];
            testCase.verifyLessThanOrEqual(max(abs(truth-center)-radius,[],'all'),1e-12);
        end
        function forcedFreshFailurePreservesSafetyThroughTheTerminalTail(testCase)
            report = runExactStateRecursiveFeasibilityScenario(Scenario="crossing",SampleCount=22, ...
                FailAfterAdmission=true,TargetJerkAmplitude=[0.1;0],TargetYawAccelerationAmplitude=0.05);
            testCase.verifyTrue(report.completed);
            testCase.verifyTrue(report.allCandidatesVerified);
            testCase.verifyGreaterThan(nnz(report.terminalActive),0);
            testCase.verifyGreaterThan(report.minimumSampledSeparationMargin,0);
            testCase.verifyGreaterThan(report.minimumSampledRoadMargin,0);
            testCase.verifyLessThanOrEqual(report.maximumDescentResidual,0);
        end
    end
end

function [ego,target,road,cfg] = localFixture()
    [ego,target,road,cfg] = encounterTestFixture.crossing();
    target.predictionMotion.jerkBound = [0.1;0];
    target.predictionMotion.yawAccelerationBound = 0.05;
end

function [ego,target,road,cfg,c] = localContinuation()
    [ego,target,road,cfg] = localFixture();
    [~,~,p,c] = collisionAvoidanceController(ego,target,road,cfg,[]);
    ego = encounterTestFixture.nextEgo(c,p.model.lane);
    h = cfg.controller.sampleTime;
    jerk = [0.08;0];
    target.targetPositionInertial = target.targetPositionInertial+target.targetVelocityInertial*h+jerk*h^3/6;
    target.targetVelocityInertial = target.targetVelocityInertial+jerk*h^2/2;
    target.targetAccelerationInertial = jerk*h;
    target.targetHeadingInertial = target.targetHeadingInertial+0.04*h^2/2;
    target.targetYawRate = 0.04*h;
end
