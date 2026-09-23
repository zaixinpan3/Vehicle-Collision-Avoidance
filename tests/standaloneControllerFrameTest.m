classdef standaloneControllerFrameTest < matlab.unittest.TestCase
    %standaloneControllerFrameTest Native adapter safety and scalar-target preparation.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'solver','clarabel','matlab')));
        end
    end
    methods (Test)
        function exportedFrameRetainsACompleteHardCertificate(testCase)
            [program,cfg]=localFixture();
            packed=standaloneControllerBenchmark.pack(program);
            [~,~,status]=standaloneControllerFrame(packed,standaloneControllerBenchmark.configuration(cfg));
            testCase.verifyGreaterThan(status,0);
        end

        function exportedCircularAdmissionReleasesTheCompleteInputSequence(testCase)
            [program,cfg]=localCircularFixture();
            packed=standaloneControllerBenchmark.pack(program);
            [decision,angles,status]=standaloneControllerFrame(packed,standaloneControllerBenchmark.configuration(cfg));
            testCase.verifyEqual(status,4);
            testCase.verifyLessThanOrEqual(max(avoidanceSafetyGeometry.jointResidual( ...
                program,decision,angles)),0);
        end

        function failedExportedAdmissionReturnsNoUncertifiedDecision(testCase)
            [program,cfg]=localCircularFixture();
            cfg.solver.maxIterations=1;
            packed=standaloneControllerBenchmark.pack(program);
            [decision,~,status]=standaloneControllerFrame(packed,standaloneControllerBenchmark.configuration(cfg));
            testCase.verifyEqual(status,0);
            testCase.verifyEmpty(decision);
        end

    end
end

function [program,cfg]=localCircularFixture()
    [ego,target,road,cfg]=encounterTestFixture.circularCrossing(.01);
    [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
    program=formulateAvoidanceProblem(problem.model);
end

function [program,cfg]=localFixture()
    [ego,target,road,cfg]=encounterTestFixture.crossing();
    target.targetPositionInertial=[15;0];target.targetVelocityInertial=[0;0];target.targetHeadingInertial=0;
    cfg.controller.sampleTime=.05;cfg.controller.horizonSteps=32;
    cfg.solver.frameDeadlineSeconds=30;cfg.solver.certificateSearchTimeLimit=30;
    [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
    program=problem.program;program.feasibleWitness=problem.decision;
    program.anchorPlan=problem.decision(program.layout.planIndex);
end
