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
        function aTireTangentCannotCombineFullBrakingAndLateralForce(testCase)
            [p,~,~]=localFormulate(testCase.anchor,testCase.model);
            point=zeros(numel(p.lower),1);index=p.inputIndices(:,1);
            point(index)=[.1;-1]-testCase.anchor.inputs(:,1);
            violation=-Inf;reference=-Inf;
            for k=1:p.primaryConeCount
                cone=p.cones(k);
                violation=max(violation,max(vecnorm(reshape(cone.A*point-cone.b,cone.size,[])) ...
                    -(cone.D*point-cone.gamma).'));
                reference=max(reference,max(vecnorm(reshape(cone.b,cone.size,[]))+cone.gamma.'));
            end
            testCase.verifyLessThanOrEqual(reference,1e-12);
            testCase.verifyGreaterThan(violation,.1);
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
        function posteriorPropagationChangesEnterTheNumericalModelReserve(testCase)
            model=testCase.model;anchor=testCase.anchor;
            model.uncertainty.egoGenerator=diag([0,0,0,.1,.1,.02]);
            [p,~,model]=localFormulate(anchor,model);
            solution=struct('inputs',anchor.inputs,'states',anchor.states);
            solution.inputs(1,:)=solution.inputs(1,:)+.02;
            solution.states=localRollout(model.initialState,solution.inputs,model.cfg);
            % Remove the nominal error to isolate the changing image of the
            % same posterior, the source of the head-on regression.
            solution.affineStates=solution.states;
            margin=solvePredictiveControl("probe",struct('operation',"remainder", ...
                'model',model,'anchor',anchor,'problem',p,'solution',solution));
            testCase.verifyEqual(margin(:,1),zeros(6,1),AbsTol=0);
            testCase.verifyGreaterThan(max(margin(4:6,3:2:end),[],'all'),1e-4);
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
