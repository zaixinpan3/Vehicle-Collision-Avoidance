classdef unrestrictedPredictiveControlTest < matlab.unittest.TestCase
    % Translations remain free; unrestricted inputs are an explicit comparison.
    properties (Access=private)
        model
        anchor
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
                'controller',struct('horizonSteps',32),'solver',struct('timeLimitSeconds',10)));
            ego=struct('position',[0;.1],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
            [~,~,prediction]=collisionAvoidanceController(ego,[],[],cfg,[]);
            testCase.model=prediction.model;testCase.anchor=prediction.model.linearization;
        end
    end
    methods (Test)
        function dynamicsCanCorrectPositionsFarOutsideTheFormerNeighborhood(testCase)
            anchor=testCase.anchor;anchor.states(1,2:end)=anchor.states(1,2:end)+100;
            [solution,search]=localSolve(anchor,testCase.model,[]);
            testCase.assertNotEmpty(solution);
            testCase.verifyTrue(search.clfStageCompleted);
            testCase.verifyGreaterThan(max(abs(solution.states(1,:)-anchor.states(1,:))),90);
            testCase.verifyEqual(solution.states(:,1),testCase.model.initialState,AbsTol=1e-7);
            testCase.verifyLessThanOrEqual(solution.states(4,:),testCase.model.cfg.model.speedMaximum+1e-5);
        end
        function disablingNumericalTrustPreservesTheUnrestrictedComparison(testCase)
            anchor=testCase.anchor;model=testCase.model;model.cfg.rti.trustRegion.enabled=false;
            % This first command needs more than 0.075 rad steering and
            % 0.125 braking-ratio correction, the old default hard box.
            required=anchor.inputs(:,1)+[.12;-.3];
            [solution,search]=localSolve(anchor,model,required);
            testCase.assertNotEmpty(solution);
            testCase.verifyTrue(search.clfStageCompleted);
            testCase.verifyEqual(solution.inputs(:,1),required,AbsTol=1e-5);
            testCase.verifyLessThanOrEqual(abs(solution.inputs(2,:)),1+1e-5);
        end
        function removingSearchBoundsDoesNotRemoveActuatorBounds(testCase)
            [solution,search]=localSolve(testCase.anchor,testCase.model,[0;1.1]);
            testCase.verifyEmpty(solution);
            testCase.verifyEqual(search.stages(1).exitFlag,-2);
        end
        function aDistantReferenceCannotHideAnActiveCollisionRow(testCase)
            model=testCase.model;cfg=model.cfg;
            model.target=[30;0;0;0;0;0;cfg.vehicle.lr;2.4;.95;0;0];
            model.targetEpoch=model.target;model.targetEpochIndex=model.sampleIndex;
            model.terminalContext=terminalSafeSet.context(model);
            anchor=testCase.anchor;anchor.states(1,2:end)=anchor.states(1,2:end)+100;
            request=struct('operation',"formulate",'model',model,'anchor',anchor,'inherited',false);
            [problem,~,formed]=solvePredictiveControl("probe",request);
            % The reference leaves the perception disk; optimized positions
            % need not. Keep target geometry until actual observed exit.
            testCase.verifyEqual(problem.encounterExit,size(anchor.states,2));
            [solution,search]=localSolve(anchor,model,[]);
            testCase.verifyEmpty(solution);
            testCase.verifyEqual(search.stages(1).exitFlag,-2);
            testCase.verifyTrue(isnan(formed.encounterExitRecord));
        end
        function theLocalUpdateReturnsWithoutANonlinearRemainderGate(testCase)
            [solution,search,model]=solvePredictiveControl(testCase.model,[],tic);
            testCase.assertNotEmpty(solution);
            testCase.verifyTrue(isnan(solution.linearizationGap));
            testCase.verifyFalse(search.rollout.available);
            testCase.verifyEqual(search.rollout.reason,"notPerformedByRti");
            testCase.verifyEqual(solution.states,solution.affineStates,AbsTol=0);
            testCase.verifyFalse(isfield(model,'inputTrustScale'));
            testCase.verifyFalse(isfield(model.cfg.nonlinear,'trustRadius'));
        end
    end
end

function [solution,search]=localSolve(anchor,model,first)
    [problem,~,model]=solvePredictiveControl("probe",struct('operation',"formulate", ...
        'model',model,'anchor',anchor,'inherited',false));
    if ~isempty(first)
        idx=problem.inputIndices(:,1);
        rows=sparse(2,numel(problem.lower));rows(:,idx)=eye(2);
        problem.equal=[problem.equal;rows];problem.rhs=[problem.rhs;first-anchor.inputs(:,1)];
    end
    [solution,search]=solvePredictiveControl("probe",struct('operation',"solve", ...
        'model',model,'anchor',anchor,'problem',problem));
end
