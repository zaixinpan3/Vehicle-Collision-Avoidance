classdef controllerRepairTest < matlab.unittest.TestCase
    % Regressions for the September 14 declared-plant simulation failures.
    properties (TestParameter)
        errorBound = struct('speedOnly',[0;0;0;.05;0;0], ...
            'fullEgo',[.05;.05;.005;.05;.02;.005]);
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'solver','clarabel','matlab')));
        end
    end
    methods (Test)

        function nativeTimeBudgetPreservesTheOrdinarySolution(testCase)
            [legacy,legacyInfo] = localNativeBudget([]);
            [budgeted,budgetedInfo] = localNativeBudget(1);
            testCase.verifyEqual(legacyInfo.status,1);
            testCase.verifyEqual(budgetedInfo.status,1);
            testCase.verifyEqual(budgeted,legacy,AbsTol=1e-8);
            testCase.verifyEqual(budgeted,2,AbsTol=1e-7);
        end

        function anExpiredNativeBudgetReturnsATimeLimitStatus(testCase)
            [~,information] = localNativeBudget(0);
            testCase.verifyEqual(information.status,8);
        end

        function aCertifiedFlagCannotOverrideANegativeTrueSpeed(testCase)
            report = localAuditRecord();
            report.minimumSampledSpeed = -0.01;
            report.minimumSampledModelDomainMargin = -0.01;
            assessment = assessExactStateSafety(report);
            testCase.verifyFalse(assessment.modelDomainHeld);
            testCase.verifyFalse(assessment.passed);
        end

        function aCertifiedFlagCannotOverrideLossOfTruthContainment(testCase)
            report = localAuditRecord();
            report.egoContainmentMargin(2) = -0.001;
            assessment = assessExactStateSafety(report);
            testCase.verifyFalse(assessment.truthContained);
            testCase.verifyFalse(assessment.passed);
        end

        function aNewAdmissionStartsItsOwnDescentObligation(testCase)
            report = localAuditRecord();
            report.admissionFrame(2) = true;
            report.candidateVerified(2) = false;
            report.descentResidual(2) = NaN;
            assessment = assessExactStateSafety(report);
            testCase.verifyTrue(assessment.passed);
        end

    end
end

function report = localAuditRecord()
    report = struct('completed',true,'planCertified',[true,true], ...
        'candidateVerified',[false,true],'admissionFrame',[true,false], ...
        'descentResidual',[NaN,0],'configuration',collisionAvoidanceControllerConfig(), ...
        'minimumSampledModelDomainMargin',0.1,'minimumSampledSpeed',1, ...
        'egoContainmentMargin',[0,0],'targetContainmentMargin',[0,inf], ...
        'minimumSampledSeparationMargin',1,'minimumSampledRoadMargin',1,'maximumSlewViolation',0);
end

function [decision,information] = localNativeBudget(seconds)
    options = [1e-8,1e-8,100,seconds];
    [decision,information] = solveAvoidanceSocpMex(sparse(1),-2,sparse(-1),0,[0;1],options);
end
