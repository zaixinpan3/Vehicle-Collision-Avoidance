classdef affinePlanAdmissionTest < matlab.unittest.TestCase
    %affinePlanAdmissionTest Scalar sets, robust geometry and command acceptance.
    properties (TestParameter)
        yawRadius={0,.12,.7,pi};
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (Test)
        function linearConstraintsIntersectTheAmplitudeDomain(testCase)
            interval=solveHardCbfClf.linearInterval([1;-2;0],[3;4;0],[-5,5]);
            testCase.verifyEqual(interval,[-2,3],AbsTol=1e-14);
        end

        function anImpossibleConstantRowRejectsTheSection(testCase)
            interval=solveHardCbfClf.linearInterval(0,-1,[-5,5]);
            testCase.verifyEmpty(interval);
        end

        function negativeConeRadiusCannotBeAcceptedAfterSquaring(testCase)
            interval=solveHardCbfClf.ballInterval([0;0],[1;0],-1,[-5,5]);
            testCase.verifyEmpty(interval);
        end

        function aModalConeRestrictsBothEndsOfTheSection(testCase)
            interval=solveHardCbfClf.ballInterval([1;3],[2;0],5,[-10,10]);
            testCase.verifyEqual(interval,[-2.5,1.5],AbsTol=1e-13);
        end

        function aTangentConeRetainsItsSinglePoint(testCase)
            interval=solveHardCbfClf.ballInterval([1;3],[2;0],3,[-10,10]);
            testCase.verifyEqual(interval,[-.5,-.5],AbsTol=1e-13);
        end

        function aConstantConeRetainsItsWholeFeasibleDomain(testCase)
            interval=solveHardCbfClf.ballInterval([1;0],[0;0],2,[-3,4]);
            testCase.verifyEqual(interval,[-3,4],AbsTol=0);
        end

        function overlapOfForbiddenIntervalsIsSubtractedOnce(testCase)
            intervals=solveHardCbfClf.subtractIntervals([-3,4],[-1,2;0,3]);
            testCase.verifyEqual(intervals,[-3,-1;3,4],AbsTol=0);
        end

        function touchingOpenIntervalsPreserveTheSharedEndpoint(testCase)
            intervals=solveHardCbfClf.subtractIntervals([-3,4],[-1,0;0,2]);
            testCase.verifyEqual(intervals,[-3,-1;0,0;2,4],AbsTol=0);
        end

        function completeGeometricExclusionReturnsNoCandidate(testCase)
            intervals=solveHardCbfClf.subtractIntervals([-3,4],[-Inf,Inf]);
            testCase.verifyEmpty(intervals);
        end

        function anEmptyForbiddenSetPreservesTheDeclaredSection(testCase)
            intervals=solveHardCbfClf.subtractIntervals([-3,4],zeros(0,2));
            testCase.verifyEqual(intervals,[-3,4],AbsTol=0);
        end

        function dictionarySupportContainsEveryPermittedYaw(testCase,yawRadius)
            [minimumExcess,maximumExcess]=localSupportCheck(yawRadius);
            testCase.verifyGreaterThanOrEqual(minimumExcess,-1e-13);
            testCase.verifyLessThan(maximumExcess,1e-10);
        end

        function freshAvoidanceRequiresAConvexSolveWithoutCollisionSlack(testCase)
            [ego,target,road,cfg]=localFixture();
            [command,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyEqual(problem.metadata.solverCallCount,1);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyFalse(problem.metadata.admissionSearch.issuedAdmissionWitness);
            testCase.verifyLessThanOrEqual(max(problem.metadata.jointCertificateResidual),0);
            testCase.verifyLessThanOrEqual(max(problem.program.physicalMatrix*problem.decision ...
                -problem.program.physicalBound),0);
            testCase.verifyEqual(command.holdSeconds,.05,AbsTol=0);
            testCase.verifyFalse(problem.metadata.fallbackUsed);
        end

        function insufficientActuationCannotIssueARestrictedPlan(testCase)
            [ego,target,road,cfg]=localFixture();
            cfg.model.frontWheelSteeringAngleMaximum=.001;
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end

        function anExpiredSearchBudgetCannotIssueAScalarCandidate(testCase)
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

function [minimumExcess,maximumExcess]=localSupportCheck(radius)
    angles=linspace(-7,7,301);sizes=[2.4,.95];
    calculated=solveHardCbfClf.rectangleSupports(sizes,radius,angles);
    reference=zeros(size(angles));
    for index=1:numel(angles)
        reference(index)=targetPrediction.rectangleSupport(sizes(1),sizes(2), ...
            [cos(angles(index));sin(angles(index))],0,radius);
    end
    minimumExcess=min(calculated-reference);maximumExcess=max(calculated-reference);
end

function [ego,target,road,cfg]=localFixture()
    [ego,target,road,cfg]=encounterTestFixture.crossing();
    target.targetPositionInertial=[15;0];target.targetVelocityInertial=[0;0];target.targetHeadingInertial=0;
    cfg.controller.sampleTime=.05;cfg.controller.horizonSteps=32;
    cfg.solver.frameDeadlineSeconds=30;cfg.solver.certificateSearchTimeLimit=30;
end
