classdef inputDeviationObjectiveTest < matlab.unittest.TestCase
    %inputDeviationObjectiveTest Fixed operating-input cost and sparse parity.
    properties (TestParameter)
        invalidWeight = {-1, NaN, Inf, [1, 2], 1+1i}
        penaltyWeight = {100, single(100), int32(100)}
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function invalidPenaltyIsRejected(testCase,invalidWeight)
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct( ...
                'jointCertificate',struct('inputDeviationWeight',invalidWeight))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end
        function penaltyIsMinimizedAtNonzeroModelInputs(testCase,penaltyWeight)
            [base,weighted]=localPrograms(penaltyWeight);
            center=[weighted.inputDeviationCenter;0];
            change=[.02*sin((1:weighted.layout.planCount).');0];
            penalty=@(z).5*z.'*(weighted.P-base.P)*z+(weighted.q-base.q).'*z;
            testCase.verifyGreaterThan(norm(center),.01);
            testCase.verifyEqual((weighted.P-base.P)*center+weighted.q-base.q, ...
                zeros(size(center)),AbsTol=1e-8);
            testCase.verifyEqual(penalty(center+change)-penalty(center), ...
                sum(weighted.inputDeviationWeight.*change(1:end-1).^2),AbsTol=1e-8);
            testCase.verifyEqual(weighted.A,base.A,AbsTol=0);
            testCase.verifyEqual(weighted.b,base.b,AbsTol=0);
        end
        function strongerPenaltyReducesDeviationOnTheSameFeasibleProblem(testCase)
            [base,weighted,cfg]=localPrograms();
            unregularized=solveHardCbfClf.constrained(base,cfg);
            regularized=solveHardCbfClf.constrained(weighted,cfg);
            indices=weighted.layout.planIndex;
            first=unregularized.decision(indices)-weighted.inputDeviationCenter;
            second=regularized.decision(indices)-weighted.inputDeviationCenter;
            testCase.verifyTrue(unregularized.feasible);
            testCase.verifyTrue(regularized.feasible);
            testCase.verifyLessThan(sum(weighted.inputDeviationWeight.*second.^2), ...
                sum(weighted.inputDeviationWeight.*first.^2));
        end
        function recenteringPreservesTheSamePhysicalObjective(testCase)
            [~,program]=localPrograms();
            reference=[program.inputDeviationCenter;1];
            trial=reference+[.02*cos((1:program.layout.planCount).');.2];
            program.anchorPlan=program.anchorPlan+.03*sin((1:program.layout.planCount).');
            lifted=avoidanceStageQp.build(program);
            referenceLifted=localLift(program,lifted,reference);
            trialLifted=localLift(program,lifted,trial);
            cost=@(p,z).5*z.'*p.P*z+p.q.'*z;
            testCase.verifyEqual(cost(program,trial)-cost(program,reference), ...
                cost(lifted,trialLifted)-cost(lifted,referenceLifted),AbsTol=1e-7);
            testCase.verifyEqual(lifted.inputDeviationCenter,program.inputDeviationCenter,AbsTol=0);
        end
    end
end

function [base,weighted,cfg]=localPrograms(penaltyWeight)
    if nargin<1,penaltyWeight=100;end
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'solver',struct('frameDeadlineSeconds',Inf)));
    ego=struct('position',[0;.1],'yaw',.01,'speed',8,'stateTime',0);
    [~,~,problem]=collisionAvoidanceController(ego,[],[-100,0;2000,0],cfg,[]);
    base=problem.program;model=problem.model;
    model.cfg.jointCertificate.inputDeviationWeight=penaltyWeight;
    weighted=formulateAvoidanceProblem(model);
end

function value=localLift(program,lifted,decision)
    n=program.layout.planCount;
    states=program.prediction.egoStateOffset+reshape( ...
        pagemtimes(program.prediction.egoStateMatrix,decision(1:n)),6,[]);
    extra=states(:,2:end)-lifted.stateCenter(:,2:end);
    value=[decision;extra(:)];
end
