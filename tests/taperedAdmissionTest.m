classdef taperedAdmissionTest < matlab.unittest.TestCase
    %taperedAdmissionTest Restricted time shapes retain original hard checks.
    properties (TestParameter)
        curvature=struct('left',.01,'right',-.01,'gentlerLeft',.008, ...
            'gentlerRight',-.008,'tighterLeft',.012,'tighterRight',-.012);
        invalidFraction={0,-.1,1.1,NaN,Inf,[.8,1]};
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (Test)
        function curvedCrossingIsCertifiedWithoutANativeSolve(testCase,curvature)
            [ego,target,road,cfg]=encounterTestFixture.circularCrossing(curvature);
            cfg.solver.jointFunction=@encounterTestFixture.fail;
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyEqual(problem.metadata.solverCallCount,0);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyFalse(problem.metadata.admissionSearch.usedFullPlanAdmission);
            testCase.verifyLessThanOrEqual(max(problem.metadata.jointCertificateResidual),0);
            testCase.verifyLessThanOrEqual(max(problem.program.physicalMatrix*problem.decision ...
                -problem.program.physicalBound),0);
            testCase.verifyEqual(problem.metadata.horizonSteps,96);
        end

        function invalidTemporalFractionsAreRejected(testCase,invalidFraction)
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct( ...
                'admission',struct('temporalShoulderFraction',invalidFraction))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end

        function exportedTaperedAdmissionRetainsTheOriginalCertificate(testCase)
            [ego,target,road,cfg]=encounterTestFixture.circularCrossing(.01);
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            program=formulateAvoidanceProblem(problem.model);
            [decision,angles,status]=standaloneControllerFrame(standaloneControllerBenchmark.pack(program), ...
                standaloneControllerBenchmark.configuration(cfg));
            program.jointCertificate.angles=angles;
            checked=solveHardCbfClf.certify(program,decision);
            testCase.verifyEqual(status,1);
            testCase.verifyEqual(decision,problem.decision,AbsTol=1e-10);
            testCase.verifyLessThanOrEqual(max(checked.safetyBound-checked.physicalBound),0);
        end
    end
end
