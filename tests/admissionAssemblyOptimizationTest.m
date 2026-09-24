classdef admissionAssemblyOptimizationTest < matlab.unittest.TestCase
    %admissionAssemblyOptimizationTest Preserve road rows in the joint geometry assembly.
    properties (TestParameter)
        hasTarget=struct('absent',false,'present',true);
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
        function jointGeometryPreservesRoadAndPoseAdmissibility(testCase,hasTarget)
            [complete,joint,plans]=localGeometry(hasTarget);
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
    end
end

function [complete,joint,plans]=localGeometry(hasTarget)
    [ego,target,road,cfg]=encounterTestFixture.circularCrossing(.01);
    [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
    model=problem.model;model.anchorPlan=problem.program.anchorPlan;
    if hasTarget
        model.encounter.radius([1,2,7])=[.01;.02;.03];
    else
        model.encounter=[];
    end
    boundary=struct('coefficients',[.001,0,-20],'safeSideSign',1, ...
        'longitudinalDirection',[1;0],'lateralDirection',[0;1],'origin',[0;0], ...
        'parameterRange',[-200;200],'normalDistanceErrorBound',.01,'boundaryId',"lower",'coveragePolicy',"strict");
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
