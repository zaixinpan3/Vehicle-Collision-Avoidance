classdef rtiModelConsistencyTest < matlab.unittest.TestCase
    % RTI affine predictions and independent offline nonlinear diagnostics.
    properties (Access=private)
        model
        anchor
    end
    properties (TestParameter)
        inputBound = struct('fullBraking',-1,'fullThrottle',1,'configuredBraking',-.6,'configuredThrottle',.4);
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
        function numericalInputExcessReturnsTheExactPhysicalBound(testCase,inputBound)
            excess=sign(inputBound)*testCase.model.cfg.solver.feasibilityTolerance/2;
            [request,raw]=localFixedInputResult(testCase.model,testCase.anchor,inputBound,excess);
            [solution,search]=solvePredictiveControl("probe",request);
            testCase.assertNotEmpty(solution);
            testCase.verifyEqual(solution.inputs(2,:),repmat(inputBound,1,size(raw,2)),AbsTol=0);
            testCase.verifyEqual(solution.inputs(1,:),raw(1,:),AbsTol=1e-12);
            testCase.verifyEqual(solution.states,testCase.anchor.states,AbsTol=1e-10);
            testCase.verifyEqual(solution.inputBoundCorrection,abs(excess),AbsTol=1e-10);
            testCase.verifyGreaterThan(solution.hard,abs(excess)/2);
            testCase.verifyLessThanOrEqual(solution.hard,request.model.cfg.solver.feasibilityTolerance);
            testCase.verifyTrue(all(isfinite(nonlinearBicycleModel.derivative( ...
                request.model.initialState,solution.inputs(:,1),request.model.cfg))));
            testCase.verifyEqual(search.solverCalls,1);
        end
        function materialInputExcessCannotBecomeAnActuatorCommand(testCase,inputBound)
            excess=sign(inputBound)*10*testCase.model.cfg.solver.feasibilityTolerance;
            request=localFixedInputResult(testCase.model,testCase.anchor,inputBound,excess);
            [solution,search]=solvePredictiveControl("probe",request);
            testCase.verifyEmpty(solution);
            testCase.verifyEqual(search.terminationReason,"clfInputBoundsViolation");
            testCase.verifyGreaterThan(search.stages.exitFlag,0);
            testCase.verifyEqual(search.solverCalls,1);
        end
        function inBoundsSolverInputsAreNotMovedToTheBoundary(testCase,inputBound)
            excess=-sign(inputBound)*testCase.model.cfg.solver.feasibilityTolerance/2;
            [request,raw]=localFixedInputResult(testCase.model,testCase.anchor,inputBound,excess);
            solution=solvePredictiveControl("probe",request);
            testCase.assertNotEmpty(solution);
            testCase.verifyEqual(solution.inputs,raw,AbsTol=1e-12);
            testCase.verifyEqual(solution.inputBoundCorrection,0,AbsTol=0);
        end
        function theOnlineUpdateReturnsTheAffineStateDecisionWithoutReplay(testCase)
            model=testCase.model;model.initialState(2)=.5;
            % Supply straight cruise inputs so a correction is needed. A
            % freshly generated path-feedback reference can already satisfy
            % the cruise problem without any affine correction.
            model.probeReference=testCase.anchor;
            previous=struct('inputTrajectory',testCase.anchor.inputs,'stateTrajectory',testCase.anchor.states);
            [solution,search,formed]=solvePredictiveControl(model,previous,tic);
            testCase.assertNotEmpty(solution);
            reference=formed.linearization;
            [next,a,b]=nonlinearBicycleModel.sample(reference.states(:,1),reference.inputs(:,1),model.cfg);
            expected=next+a*(solution.states(:,1)-reference.states(:,1)) ...
                +b*(solution.inputs(:,1)-reference.inputs(:,1));
            actual=nonlinearBicycleModel.sample(model.initialState,solution.inputs(:,1),model.cfg);
            testCase.verifyEqual(solution.states(:,2),expected,AbsTol=1e-5);
            testCase.verifyGreaterThan(norm(solution.states(:,2)-actual),1e-8);
            testCase.verifyFalse(search.rollout.available);
            testCase.verifyEqual(search.terminationReason,"clfReturned");
            testCase.verifyEqual(solution.safety,solution.affineSafety,AbsTol=0);
        end
        function anOfflineProbeFindsTheCollidingHardTail(testCase)
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
        function anOfflineProbeFindsTheTerminalStateViolation(testCase)
            model=testCase.model;anchor=testCase.anchor;
            % Deliberately require a nearly zero terminal sideslip. A turn
            % alone is admissible now that the extra |r| V cap is removed.
            model.cfg.terminal.sideslipMaximumRadians=.001;
            model.terminalContext=terminalSafeSet.context(model);
            anchor.inputs=repmat([.25;0],1,8);
            anchor.states=localRollout(model.initialState,anchor.inputs,model.cfg);
            [p,~,model]=localFormulate(anchor,model);
            check=solvePredictiveControl("probe",struct('operation',"check", ...
                'model',model,'anchor',anchor,'problem',p));
            testCase.verifyGreaterThan(check.terminalViolation,model.cfg.solver.feasibilityTolerance);
            testCase.verifyGreaterThan(check.hard,model.cfg.solver.feasibilityTolerance);
        end
        function avoidanceAcceptsPositivePcbfSlackWithoutAClfObjective(testCase)
            model=testCase.model;anchor=testCase.anchor;
            % Cruise performance is inactive during avoidance. An initially
            % overlapping, faster target requires positive PCBF
            % slack, then leaves. This tests formulation, not safe execution.
            model.initialState(2)=.5;anchor.states(2,:)=.5;
            model.target=[0;.5;0;40;0;0;model.cfg.vehicle.lr;2.4;.95;0;0];
            model.targetEpoch=model.target;model.targetEpochIndex=model.sampleIndex;
            model.terminalContext=terminalSafeSet.context(model);
            [p,~,model]=localFormulate(anchor,model);
            fixed=[p.stateIndices(:);p.inputIndices(:)];
            p.lower(fixed)=0;p.upper(fixed)=0;
            testCase.verifyEqual(p.upper(p.clfIndex),0);
            testCase.verifyEqual(p.performanceObjective(p.clfIndex),0);
            testCase.verifyFalse(isfield(p,'clfRowIndex'));
            [solution,search]=solvePredictiveControl("probe",struct('operation',"solve", ...
                'model',model,'anchor',anchor,'problem',p));
            testCase.assertNotEmpty(solution);
            testCase.verifyTrue(search.pcbfStageCompleted);
            testCase.verifyTrue(search.secondaryStageAttempted);
            testCase.verifyFalse(search.clfStageAttempted);
            testCase.verifyGreaterThan(search.primaryOptimum,0);
            testCase.verifyTrue(isnan(solution.clfSlack));
            testCase.verifyEqual(solution.objectiveValue,solution.safety,AbsTol=0);
            testCase.verifyLessThanOrEqual(solution.safety,search.primaryOptimum+model.cfg.solver.constraintTolerance);
            testCase.verifyEqual(search.slackCap,search.primaryOptimum,AbsTol=0);
            testCase.verifyEqual(solution.states,anchor.states,AbsTol=1e-6);
            testCase.verifyEqual(solution.inputs,anchor.inputs,AbsTol=1e-6);
        end
        function cruiseIntegratesTheContinuousClfAcrossOneHoldAsAnAffineRow(testCase)
            model=testCase.model;anchor=testCase.anchor;
            model.initialState(2)=.1;anchor.states(2,:)=.1;
            [p,~,model]=localFormulate(anchor,model);
            testCase.verifyEmpty(p.cones);
            testCase.verifyEqual(p.upper(p.clfIndex),Inf);
            value=nonlinearBicycleModel.nominalValue(model.initialState,model.lane,model.nominalReference);
            alpha=model.nominalReference.decayRate;h=model.cfg.controller.sampleTime;
            duration=-expm1(-alpha*h)/alpha;
            point=zeros(size(p.lower));increment=[.001;-.002;.001;0;0;0];
            point(p.stateIndices(:,2))=increment;
            [error,jacobian]=nonlinearBicycleModel.errorLinearization(anchor.states(:,2),model.lane,model.nominalReference);
            nextValue=error.'*model.nominalReference.matrix*error;
            nextTangent=nextValue+2*error.'*model.nominalReference.matrix*jacobian*increment;
            expected=(nextTangent-value)/duration;
            testCase.verifyEqual((p.clfDerivativeMap*point+p.clfDerivativeOffset)*p.clfScale,expected,AbsTol=1e-12);
            testCase.verifyEqual(p.clfDerivativeLimit,-alpha*value,AbsTol=1e-12);
            testCase.verifyEqual(p.clfSlackDuration,duration,AbsTol=1e-15);
            testCase.verifyEqual(full(p.clfDerivativeMap(p.inputIndices)),zeros(size(p.inputIndices)),AbsTol=0);
            slack=max(0,(nextTangent-exp(-alpha*h)*value)/duration);
            point(p.clfIndex)=slack/p.clfScale;
            testCase.verifyLessThanOrEqual(p.a(p.clfRowIndex,:)*point-p.b(p.clfRowIndex),1e-12);
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

function [request,inputs]=localFixedInputResult(model,anchor,bound,excess)
    if bound<0,model.cfg.actuation.brakingRatioMinimum=bound;
    else,model.cfg.actuation.brakingRatioMaximum=bound;end
    [problem,~,model]=localFormulate(anchor,model);
    inputs=anchor.inputs;inputs(2,:)=bound+excess;
    % Isolate the solver/output interface with a uniquely determined result.
    % Physical input bounds remain available to the production decoder.
    point=zeros(size(problem.lower));point(problem.inputIndices)=inputs-anchor.inputs;
    count=numel(point);problem.equal=speye(count);problem.rhs=point;
    problem.a=sparse(0,count);problem.b=zeros(0,1);problem.cones=[];
    problem.lower(:)=-Inf;problem.upper(:)=Inf;
    request=struct('operation',"solve",'model',model,'anchor',anchor,'problem',problem);
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
