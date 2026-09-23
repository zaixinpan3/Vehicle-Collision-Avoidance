classdef admissionSafetyTest < matlab.unittest.TestCase
    %admissionSafetyTest Hard command acceptance after fluid initialization.
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (Test)
        function freshAvoidanceRequiresAConvexSolveWithoutCollisionSlack(testCase)
            [ego,target,road,cfg]=localFixture();
            [command,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyEqual(problem.metadata.solverCallCount,1);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyFalse(problem.metadata.admissionSearch.issuedAdmissionWitness);
            testCase.verifyLessThanOrEqual(max(problem.program.physicalMatrix*problem.decision ...
                -problem.program.physicalBound),0);
            testCase.verifyEqual(command.holdSeconds,.05,AbsTol=0);
            testCase.verifyFalse(problem.metadata.fallbackUsed);
        end

        function insufficientActuationCannotIssueAnUnsafePlan(testCase)
            [ego,target,road,cfg]=localFixture();
            cfg.model.frontWheelSteeringAngleMaximum=.001;
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end

        function anExpiredSearchBudgetCannotIssueAFluidSeed(testCase)
            [ego,target,road,cfg]=localFixture();
            cfg.solver.certificateSearchTimeLimit=1e-9;
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end

        function retiredAdmissionIterationSettingsAreRejected(testCase)
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct( ...
                'jointCertificate',struct('maximumAdmissionSolves',3))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end
    end
end

function [ego,target,road,cfg]=localFixture()
    [ego,target,road,cfg]=encounterTestFixture.crossing();
    target.targetPositionInertial=[15;0];target.targetVelocityInertial=[0;0];target.targetHeadingInertial=0;
    cfg.controller.sampleTime=.05;cfg.controller.horizonSteps=32;
    cfg.solver.frameDeadlineSeconds=30;cfg.solver.certificateSearchTimeLimit=30;
end
