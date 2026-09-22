classdef admissionAssemblyOptimizationTest < matlab.unittest.TestCase
    %admissionAssemblyOptimizationTest Preserve hard domains and side selection.
    properties (TestParameter)
        targetCount=struct('none',0,'one',1,'two',2);
        curvature=struct('left',.01,'right',-.01);
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (Test)
        function jointGeometryPreservesRoadAndPoseAdmissibility(testCase,targetCount)
            [complete,joint,plans]=localGeometry(targetCount);
            retained=~startsWith(complete.label,"collision:");
            testCase.verifyTrue(any(startsWith(joint.label,"road:")));
            testCase.verifyEqual(joint.label,complete.label(retained));
            % Changing BLAS row dimensions can change final rounding on road rows.
            testCase.verifyEqual(joint.matrix,complete.matrix(retained,:),AbsTol=1e-13);
            testCase.verifyEqual(joint.physicalBound,complete.physicalBound(retained),AbsTol=0);
            testCase.verifyEqual(joint.matrix*plans-joint.physicalBound, ...
                complete.matrix(retained,:)*plans-complete.physicalBound(retained),AbsTol=1e-12);
            testCase.verifyEqual(joint.cellData,complete.cellData,AbsTol=0);
            testCase.verifyEqual(joint.normals,complete.normals,AbsTol=0);
        end

        function screenedCrossingRetainsItsSelectedFullPlan(testCase,curvature)
            [program,cfg]=localProgram(curvature);
            [selected,angles,information]=solveHardCbfClf.fluidInitialize(program,cfg);
            firstOnly=program;firstOnly.fluidReference.valid(2)=false;
            [expected,expectedAngles]=solveHardCbfClf.fluidInitialize(firstOnly,cfg);
            testCase.verifyLessThanOrEqual(information.referencePhysicalExcess(1),0);
            testCase.verifyGreaterThan(information.referencePhysicalExcess(2),0);
            testCase.verifyEqual(selected,expected,AbsTol=0);
            testCase.verifyEqual(angles,expectedAngles,AbsTol=0);
        end

        function screeningStillSelectsAFeasibleSecondCandidate(testCase,curvature)
            [program,cfg]=localProgram(curvature);
            [expected,expectedAngles]=solveHardCbfClf.fluidInitialize(program,cfg);
            reversed=localReverseReferences(program);
            [actual,angles,information]=solveHardCbfClf.fluidInitialize(reversed,cfg);
            testCase.verifyGreaterThan(information.referencePhysicalExcess(1),0);
            testCase.verifyLessThanOrEqual(information.referencePhysicalExcess(2),0);
            testCase.verifyEqual(actual,expected,AbsTol=2e-12);
            testCase.verifyEqual(angles,expectedAngles,AbsTol=2e-12);
        end
    end
end

function [program,cfg]=localProgram(curvature)
    [ego,target,road,cfg]=encounterTestFixture.circularCrossing(curvature);
    [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
    program=formulateAvoidanceProblem(problem.model);
end

function program=localReverseReferences(program)
    reference=program.fluidReference;
    reference.bump=reference.bump([2,1],:);
    reference.heading=reference.heading([2,1],:);
    reference.valid=reference.valid([2,1]);
    reference.amplitudes=reference.amplitudes([2,1]);
    program.fluidReference=reference;
end

function [complete,joint,plans]=localGeometry(targetCount)
    [ego,target,road,cfg]=encounterTestFixture.circularCrossing(.01);
    [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
    model=problem.model;model.anchorPlan=problem.program.anchorPlan;
    model.encounters=repmat(model.encounters(1),targetCount,1);
    for index=1:targetCount
        model.encounters(index).key="target-"+index;
        model.encounters(index).center(1)=model.encounters(index).center(1)+index;
        model.encounters(index).radius([1,2,7])=[.01;.02;.03];
    end
    boundary=struct('coefficients',[.001,0,-20],'safeSideSign',1, ...
        'longitudinalDirection',[1;0],'lateralDirection',[0;1],'origin',[0;0], ...
        'parameterRange',[-200;200],'normalDistanceErrorBound',.01,'boundaryId',"lower");
    model.road.boundaries=[boundary;boundary];
    model.road.boundaries(2).coefficients=[.001,0,20];
    model.road.boundaries(2).safeSideSign=-1;model.road.boundaries(2).boundaryId="upper";
    prediction=problem.prediction;
    prediction.separationNormals=avoidanceSafetyGeometry.supportNormals(model,prediction,model.anchorPlan);
    complete=avoidanceSafetyGeometry.build(model,prediction);
    joint=avoidanceSafetyGeometry.build(model,prediction,false);
    limit=problem.program.decisionRadius;
    plans=[model.anchorPlan,zeros(size(limit)),limit,-limit];
end
