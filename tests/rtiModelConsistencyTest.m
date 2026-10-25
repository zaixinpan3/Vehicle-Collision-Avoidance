classdef rtiModelConsistencyTest < matlab.unittest.TestCase
    % The affine solve must not stand in for nonlinear primary feasibility.
    properties (Access=private)
        model
        anchor
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for directory=["controller","config"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,directory)));
            end
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
                'controller',struct('horizonSteps',8),'solver',struct('timeLimitSeconds',10)));
            ego=struct('position',[0;0],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
            [~,~,prediction]=collisionAvoidanceController(ego,[],[],cfg,[]);
            testCase.model=prediction.model;testCase.anchor=prediction.model.linearization;
        end
    end
    methods (Test)
        function aFiniteRolloutWithACollidingHardTailFailsValidation(testCase)
            model=testCase.model;cfg=model.cfg;anchor=testCase.anchor;
            % Extend straight cruise through a separated, stationary target.
            anchor.inputs=repmat(anchor.inputs(:,1),1,32);
            anchor.states=localRollout(model.initialState,anchor.inputs,cfg);
            model.target=[10;0;0;0;0;0;cfg.vehicle.lr;2.4;.95;0;0];
            model.targetEpoch=model.target;model.targetEpochIndex=model.sampleIndex;
            model.terminalContext=terminalSafeSet.context(model);
            [p,~,model]=localFormulate(anchor,model);
            check=solvePredictiveControl("probe",struct('operation',"check", ...
                'model',model,'anchor',anchor,'problem',p));
            testCase.verifyTrue(all(isfinite(anchor.states),'all'));
            testCase.verifyGreaterThan(check.hard,.1);
            testCase.verifyEqual(check.safety,0,AbsTol=1e-12);
        end
        function terminalStateViolationIsCheckedAfterNonlinearRollout(testCase)
            model=testCase.model;anchor=testCase.anchor;
            anchor.inputs=repmat([.25;0],1,8);
            anchor.states=localRollout(model.initialState,anchor.inputs,model.cfg);
            [p,~,model]=localFormulate(anchor,model);
            check=solvePredictiveControl("probe",struct('operation',"check", ...
                'model',model,'anchor',anchor,'problem',p));
            testCase.verifyGreaterThan(check.terminalViolation,.1);
            testCase.verifyGreaterThan(check.hard,model.cfg.solver.feasibilityTolerance);
        end
        function softClfAcceptsPositivePcbfSlackWithoutChangingThePrimaryTrajectory(testCase)
            model=testCase.model;anchor=testCase.anchor;
            % Fixed straight motion off the route cannot achieve Vdot <= -alpha V.
            % An initially overlapping, faster target requires positive PCBF
            % slack, then leaves. This tests formulation, not safe execution.
            model.initialState(2)=.5;anchor.states(2,:)=.5;
            model.target=[0;.5;0;40;0;0;model.cfg.vehicle.lr;2.4;.95;0;0];
            model.targetEpoch=model.target;model.targetEpochIndex=model.sampleIndex;
            model.terminalContext=terminalSafeSet.context(model);
            [p,~,model]=localFormulate(anchor,model);
            fixed=[p.stateIndices(:);p.inputIndices(:)];
            p.lower(fixed)=0;p.upper(fixed)=0;
            testCase.verifyEqual(p.upper(p.clfIndex),Inf);
            [solution,search]=solvePredictiveControl("probe",struct('operation',"solve", ...
                'model',model,'anchor',anchor,'problem',p));
            testCase.assertNotEmpty(solution);
            testCase.verifyTrue(search.clfStageCompleted);
            testCase.verifyGreaterThan(search.primaryOptimum,0);
            testCase.verifyGreaterThan(solution.clfSlack,0);
            testCase.verifyLessThanOrEqual(solution.safety,search.slackCap+1e-5);
            testCase.verifyEqual(solution.states,anchor.states,AbsTol=1e-6);
            testCase.verifyEqual(solution.inputs,anchor.inputs,AbsTol=1e-6);
        end
        function theSecondaryProblemUsesAnAffineContinuousClfRow(testCase)
            model=testCase.model;anchor=testCase.anchor;
            model.initialState(2)=.1;anchor.states(2,:)=.1;
            [p,~,model]=localFormulate(anchor,model);
            testCase.verifyEmpty(p.cones);
            testCase.verifyEqual(p.upper(p.clfIndex),Inf);
            point=zeros(size(p.lower));point(p.inputIndices(:,1))=[.001;-.002];
            [rate,gradient,value]=nonlinearBicycleModel.nominalDerivative( ...
                model.initialState,anchor.inputs(:,1),model.lane,model.nominalReference,model.cfg);
            expected=rate+gradient*point(p.inputIndices(:,1));
            testCase.verifyEqual((p.clfDerivativeMap*point+p.clfDerivativeOffset)*p.clfScale,expected,AbsTol=1e-12);
            testCase.verifyEqual(p.clfDerivativeLimit,-model.nominalReference.decayRate*value,AbsTol=1e-12);
            testCase.verifyEqual(full(p.clfDerivativeMap(p.stateIndices)),zeros(size(p.stateIndices)),AbsTol=0);
            testCase.verifyEqual(full(p.clfDerivativeMap(p.inputIndices(:,2:end))),zeros(size(p.inputIndices(:,2:end))),AbsTol=0);
            point(p.clfIndex)=max(0,expected-p.clfDerivativeLimit)/p.clfScale;
            testCase.verifyLessThanOrEqual(p.a(end,:)*point-p.b(end),1e-12);
        end
        function avoidanceAllocationUsesLongitudinalAuthorityWithoutCruiseTracking(testCase)
            cfg=testCase.model.cfg;x=[0;0;0;8;0;0];
            straight=nonlinearBicycleModel.accelerationGuidance(x,[-6;0],[0;0],cfg,1,true);
            turning=nonlinearBicycleModel.accelerationGuidance(x,[-6;7],[0;0],cfg,1,true);
            testCase.verifyLessThan(straight(2),-cfg.nominalClf.brakingRatioLimit);
            testCase.verifyGreaterThan(turning(2),straight(2));
            testCase.verifyGreaterThan(turning(1),straight(1));
            changed=cfg;changed.referenceSpeed=5;changed.nominalClf.speedGain=100;
            same=nonlinearBicycleModel.accelerationGuidance(x,[-6;7],[0;0],changed,1,true);
            testCase.verifyEqual(same,turning,AbsTol=0);
            testCase.verifyLessThanOrEqual(abs(turning(2)),1);
        end
    end
end

function [problem,search,model]=localFormulate(anchor,model)
    [problem,search,model]=solvePredictiveControl("probe",struct('operation',"formulate", ...
        'model',model,'anchor',anchor,'inherited',false));
end

function states=localRollout(x,inputs,cfg)
    states=zeros(6,size(inputs,2)+1);states(:,1)=x;
    for k=1:size(inputs,2)
        states(:,k+1)=nonlinearBicycleModel.sample(states(:,k),inputs(:,k),cfg);
    end
end
