classdef fluidInitializationTest < matlab.unittest.TestCase
    %fluidInitializationTest Fluid references initialize fixed-direction trajectory optimization.
    properties (TestParameter)
        curvature=struct('left',.01,'right',-.01,'gentlerLeft',.008, ...
            'gentlerRight',-.008,'tighterLeft',.012,'tighterRight',-.012);
        variation=struct('laterCrossing',[.01,8,3,1], ...
            'fasterEgo',[.01,9,4,1],'tighterCurve',[.015,8,4,-1], ...
            'mirroredLaterCrossing',[-.01,8,3,-1]);
        invalidWidth={0,-.1,NaN,Inf,[.8,1]};
        retiredSetting={'normalCount','amplitudeCells','performanceIterations','temporalShoulderFraction'};
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

        function invalidFluidWidthsAreRejected(testCase,invalidWidth)
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct( ...
                'admission',struct('widthScale',invalidWidth))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end

        function retiredGridSettingsAreRejected(testCase,retiredSetting)
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct( ...
                'admission',struct(retiredSetting,1))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end

        function fluidReferenceOpposesTheCrossingMotion(testCase,curvature)
            [ego,target,road,cfg]=encounterTestFixture.circularCrossing(curvature);
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            information=problem.metadata.admissionSearch.initialization;
            testCase.verifyEqual(sign(information.amplitudeMeters),-sign(curvature));
            testCase.verifyLessThan(information.terminalFitError,1e-8);
            testCase.verifyEqual(problem.metadata.admissionSearch.directionSeedSource,"chengFluidReference");
            testCase.verifyGreaterThan(information.maximumSupportResidual,0);
            testCase.verifyLessThanOrEqual(max(problem.metadata.jointCertificateResidual),0);
        end

        function predictedGeometrySelectsAFeasibleSideWithOneSolve(testCase,variation)
            [ego,target,road,cfg]=localVariation(variation);
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            information=problem.metadata.admissionSearch.initialization;
            testCase.verifyEqual(sign(information.amplitudeMeters),variation(4));
            testCase.verifyEqual(information.referenceCount,2);
            testCase.verifyEqual(problem.metadata.solverCallCount,1);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyLessThanOrEqual(max(problem.metadata.jointCertificateResidual),0);
        end

        function aFluidSeedCannotBypassAFailedOptimizer(testCase,curvature)
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

        function exportedFluidAdmissionRetainsTheOriginalCertificate(testCase)
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
    [seed,angles]=solveHardCbfClf.fluidInitialize(program,cfg);
    direction=seed(program.layout.planIndex)-program.anchorPlan;
    displacement=problem.decision(program.layout.planIndex)-program.anchorPlan;
    distance=norm(displacement-direction*((direction.'*displacement)/(direction.'*direction)));
    anglesEqual=isequal(angles,problem.program.jointCertificate.angles);
end

function [ego,target,road,cfg]=localVariation(value)
    [ego,target,road,cfg]=encounterTestFixture.circularCrossing(value(1));
    cfg.referenceSpeed=value(2);state=ltvBicycleModel.cruiseEquilibrium(value(1),cfg);
    ego.yaw=state(3);ego.speed=state(4);ego.lateralVelocity=state(5);ego.yawRate=state(6);
    target.targetVelocityInertial=target.targetVelocityInertial*(value(3)/4);
end
