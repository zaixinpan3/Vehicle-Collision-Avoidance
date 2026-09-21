classdef taperedAdmissionTest < matlab.unittest.TestCase
    %taperedAdmissionTest Temporal seeds initialize fixed-direction trajectory optimization.
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
        function curvedCrossingOptimizesTheFullPlanWithFixedDirections(testCase,curvature)
            [ego,target,road,cfg]=encounterTestFixture.circularCrossing(curvature);
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyEqual(problem.metadata.solverCallCount,1);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyTrue(problem.metadata.admissionSearch.usedFullPlanAdmission);
            testCase.verifyLessThanOrEqual(max(problem.metadata.jointCertificateResidual),0);
            testCase.verifyLessThanOrEqual(max(problem.program.physicalMatrix*problem.decision ...
                -problem.program.physicalBound),0);
            testCase.verifyEqual(problem.metadata.horizonSteps,96);
            testCase.verifyFalse(problem.metadata.admissionSearch.issuedAdmissionWitness);
            testCase.verifyEqual(problem.program.jointCertificate.angles, ...
                problem.metadata.admissionSearch.fixedCertificateAngles,AbsTol=0);
        end

        function invalidTemporalFractionsAreRejected(testCase,invalidFraction)
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct( ...
                'admission',struct('temporalShoulderFraction',invalidFraction))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end

        function aCertifiedDirectionSeedCannotBypassAFailedOptimizer(testCase,curvature)
            [ego,target,road,cfg]=encounterTestFixture.circularCrossing(curvature);
            cfg.solver.jointFunction=@encounterTestFixture.fail;
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end

        function trajectoryOptimizationCanLeaveTheInitializerControlLine(testCase)
            [distance,anglesEqual]=localControlLineDeparture();
            testCase.verifyGreaterThan(distance,1e-4);
            testCase.verifyTrue(anglesEqual);
        end

        function exportedTaperedAdmissionRetainsTheOriginalCertificate(testCase)
            [ego,target,road,cfg]=encounterTestFixture.circularCrossing(.01);
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            program=formulateAvoidanceProblem(problem.model);
            [decision,angles,status]=standaloneControllerFrame(standaloneControllerBenchmark.pack(program), ...
                standaloneControllerBenchmark.configuration(cfg));
            program.jointCertificate.angles=angles;
            checked=solveHardCbfClf.certify(program,decision);
            testCase.verifyEqual(status,4);
            testCase.verifyEqual(decision,problem.decision,AbsTol=1e-10);
            testCase.verifyLessThanOrEqual(max(checked.safetyBound-checked.physicalBound),0);
        end
    end
end

function [distance,anglesEqual]=localControlLineDeparture()
    [ego,target,road,cfg]=encounterTestFixture.circularCrossing(.01);
    [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
    program=formulateAvoidanceProblem(problem.model);
    [seed,angles]=solveHardCbfClf.admitSection(program,cfg);
    direction=seed(program.layout.planIndex)-program.anchorPlan;
    displacement=problem.decision(program.layout.planIndex)-program.anchorPlan;
    distance=norm(displacement-direction*((direction.'*displacement)/(direction.'*direction)));
    anglesEqual=isequal(angles,problem.program.jointCertificate.angles);
end
