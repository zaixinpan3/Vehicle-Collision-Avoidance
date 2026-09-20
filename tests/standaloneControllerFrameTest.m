classdef standaloneControllerFrameTest < matlab.unittest.TestCase
    %standaloneControllerFrameTest Native adapter safety and target identity.
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
            [decision,angles,status]=standaloneControllerFrame(packed,standaloneControllerBenchmark.configuration(cfg));
            program.jointCertificate.angles=angles;
            certified=solveHardCbfClf.certify(program,decision);
            testCase.verifyGreaterThan(status,0);
            testCase.verifyLessThanOrEqual(max(certified.safetyBound-certified.physicalBound),0);
        end

        function numericalVerifierRejectsANonfiniteClf(testCase)
            [program,~]=localFixture();
            first=program.cones(2)+1;program.b(first)=NaN;
            status=solveHardCbfClf.inspect(program,program.feasibleWitness);
            testCase.verifyEqual(status,2);
        end

        function numericalVerifierRejectsAnActuatorViolation(testCase)
            [program,~]=localFixture();decision=program.feasibleWitness;decision(1)=1e6;
            status=solveHardCbfClf.inspect(program,decision);
            testCase.verifyEqual(status,1);
        end

        function packingPreservesDifferentTargetIdentities(testCase)
            [program,~]=localFixture();
            program.jointCertificate.records(1).key='target-a';
            program.jointCertificate.records(2).key='target-b';
            program.jointCertificate.records(3).key='target-a';
            packed=standaloneControllerBenchmark.pack(program);
            testCase.verifyEqual(packed.jointCertificate.records(1).key,packed.jointCertificate.records(3).key);
            testCase.verifyNotEqual(packed.jointCertificate.records(1).key,packed.jointCertificate.records(2).key);
        end
    end
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
