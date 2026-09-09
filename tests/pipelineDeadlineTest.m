classdef pipelineDeadlineTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"scripts")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"controller")));
        end
    end
    methods (Test)
        function anExpiredFrameNeverAdvancesThePlant(testCase)
            cfg = collisionAvoidanceControllerConfig(struct("controller",struct("horizonSteps",8)));
            trial = runFiniteBicycleDiagnostic(cfg,.05,PrepareController=false, ...
                DeadlineSeconds=1e-12,EnforceRuntimeDeadline=true);
            testCase.verifyTrue(trial.failure.occurred);
            testCase.verifyEqual(trial.failure.identifier,"collisionAvoidanceController:runtimeDeadlineExceeded");
            testCase.verifyEqual(trial.time,0);
            testCase.verifyEmpty(trial.input);
            testCase.verifyFalse(trial.runtime.deadlineMet);
        end

        function aFailedPureGatePreventsTheJointPhysicalExperiment(testCase)
            testCase.assumeFalse(isempty(which("PassVeh14DOF.sltx")));
            report = runStraightRealtimeValidation(Duration=0.1, ...
                DeadlineSeconds=1e-12,Progress=false);
            testCase.verifyFalse(report.passed);
            testCase.verifyEmpty(report.joint);
            testCase.verifyEqual(report.controllerOnly.failure.identifier, ...
                "collisionAvoidanceController:noCertifiedContinuation");
            testCase.verifyEqual(report.controllerOnly.controlTime,0);
            testCase.verifyEmpty(report.controllerOnly.command);
        end

        function preparationPreservesLiveSensorNoiseAndObserverState(testCase)
            estimator = estimatorControllerIntegrationConfig();
            cfg = collisionAvoidanceControllerConfig(struct("controller",struct("horizonSteps",4)));
            ego = struct("position",[0;0],"yawAngle",0, ...
                "longitudinalVelocity",15,"lateralVelocity",0,"yawRate",0);
            ego.stateTime = 0;
            ego.perception = struct("time",0,"range",30,"completeWithinRange",true);
            target = @(time,~) struct("targetPositionInertial",[25-10*time;0.8], ...
                "targetVelocityInertial",[-10;0],"targetAccelerationInertial",[0;0]);
            expectedContext = nrmmEstimatorControllerAdapter("initialize",estimator,ego,target);
            actualContext = nrmmEstimatorControllerAdapter("initialize",estimator,ego,target);
            prepareCollisionAvoidancePipeline(ego,[-200,0;2000,0],cfg,estimator);
            [expectedContext,expected,~,expectedFrame] = nrmmEstimatorControllerAdapter( ...
                "sample",expectedContext,0,ego,target(0,[]));
            [actualContext,actual,~,actualFrame] = nrmmEstimatorControllerAdapter( ...
                "sample",actualContext,0,ego,target(0,[]));
            testCase.verifyEqual(actualFrame,expectedFrame);
            testCase.verifyEqual(actual,expected);
            testCase.verifyEqual(actualContext.runtime,expectedContext.runtime);
        end
    end
end
