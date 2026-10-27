classdef referenceDeviationObjectiveTest < matlab.unittest.TestCase
    % PCBF has priority over reference proximity; CLF is cruise-only.
    properties (Access=private)
        model
        anchor
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
                'controller',struct('horizonSteps',32),'solver',struct('timeLimitSeconds',10)));
            ego=struct('position',[0;0],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
            [~,~,prediction]=collisionAvoidanceController(ego,[],[],cfg,[]);
            model=prediction.model;
            [~,target]=collisionThreatScenario("headOn",cfg);
            model.target=target;model.targetEpoch=target;model.targetOnset=true;
            [testCase.anchor,~,testCase.model]=solvePredictiveControl("probe", ...
                struct('operation',"reference",'model',model,'previousState',[]));
        end
    end
    methods (Test)
        function aLargeProximityWeightCannotBuyAdditionalSafetySlack(testCase)
            request=localPriorityProblem(testCase.model,testCase.anchor,1,1e8);
            [solution,search]=solvePredictiveControl("probe",request);
            testCase.assertNotEmpty(solution);
            testCase.verifyTrue(search.secondaryStageCompleted);
            testCase.verifyEqual(search.primaryOptimum,.5,AbsTol=1e-5);
            testCase.verifyEqual(solution.safety,.5,AbsTol=1e-5);
            testCase.verifyEqual(solution.inputCorrection(1,1),.5,AbsTol=1e-5);
            testCase.verifyEqual(search.slackCap,search.primaryOptimum,AbsTol=0);
        end
        function trackingAndSmoothnessCannotBuyAdditionalSafetySlack(testCase)
            request=localPriorityProblem(testCase.model,testCase.anchor,1,10,[1e6,1e6]);
            [solution,search]=solvePredictiveControl("probe",request);
            testCase.assertNotEmpty(solution);
            testCase.verifyEqual(solution.safety,.5,AbsTol=1e-5);
            testCase.verifyEqual(search.slackCap,search.primaryOptimum,AbsTol=0);
            testCase.verifyTrue(search.secondaryStageCompleted);
        end
        function equalZeroSlackSolutionsPreferTheSmallerReferenceCorrection(testCase)
            request=localPriorityProblem(testCase.model,testCase.anchor,.1,10);
            [solution,search]=solvePredictiveControl("probe",request);
            testCase.assertNotEmpty(solution);
            testCase.verifyTrue(search.secondaryStageCompleted);
            testCase.verifyEqual(solution.safety,0,AbsTol=1e-5);
            testCase.verifyEqual(solution.inputCorrection(1,1),.1,AbsTol=1e-5);
            testCase.verifyLessThanOrEqual(search.solverCalls,2);
        end
        function anAffineZeroSlackWitnessAvoidsThePrimaryNumericalSolve(testCase)
            request=localWitnessProblem(testCase.model,testCase.anchor);
            [solution,search]=solvePredictiveControl("probe",request);
            testCase.assertNotEmpty(solution);
            testCase.verifyFalse(search.stages(1).numericalSolve);
            testCase.verifyTrue(search.stages(2).numericalSolve);
            testCase.verifyEqual(search.solverCalls,1);
            testCase.verifyTrue(search.secondaryStageCompleted);
            testCase.verifyEqual(solution.safety,0,AbsTol=1e-5);
        end
        function unfinishedRefinementRetainsTheCompletedPrioritySolution(testCase)
            request=localWitnessProblem(testCase.model,testCase.anchor);
            request.model.cfg.solver.maxIterations=1;
            [solution,search]=solvePredictiveControl("probe",request);
            testCase.assertNotEmpty(solution);
            testCase.verifyTrue(search.pcbfStageCompleted);
            testCase.verifyFalse(search.secondaryStageCompleted);
            testCase.verifyEqual(search.terminationReason,"pcbfReturnedWithoutRefinement");
            testCase.verifyEqual(solution.safety,0,AbsTol=0);
            testCase.verifyEqual(solution.inputs,testCase.anchor.inputs,AbsTol=0);
            testCase.verifyLessThanOrEqual(solution.hard,request.model.cfg.solver.constraintTolerance);
            testCase.verifyEqual(search.solverCalls,1);
        end
        function cruiseDecayCannotAlterAnAvoidanceUpdate(testCase)
            original=localSolve(testCase.model,testCase.anchor,10);
            changed=testCase.model;changed.nominalReference.decayRate=100*changed.nominalReference.decayRate;
            alternative=localSolve(changed,testCase.anchor,10);
            testCase.assertNotEmpty(original);testCase.assertNotEmpty(alternative);
            testCase.verifyFalse(original.clfEnabled);
            testCase.verifyTrue(isnan(original.clfSlack));
            testCase.verifyEqual(alternative.inputs,original.inputs,AbsTol=1e-10);
            testCase.verifyEqual(alternative.states,original.states,AbsTol=1e-10);
        end
        function avoidanceUsesProximityOnlyAfterMinimizingSafetySlack(testCase)
            [original,search]=localSolve(testCase.model,testCase.anchor,10);
            alternative=localSolve(testCase.model,testCase.anchor,1000);
            testCase.assertNotEmpty(original);testCase.assertNotEmpty(alternative);
            testCase.verifyEqual(original.inputs,alternative.inputs,AbsTol=1e-6);
            testCase.verifyEqual(original.states,alternative.states,AbsTol=1e-6);
            testCase.verifyEqual(original.referenceDeviationWeight,10);
            testCase.verifyGreaterThan(original.referenceDeviationCost,0);
            testCase.verifyEqual([search.stages.objective],["pcbfSlack","nominalTrackingAndRegularization"]);
            testCase.verifyTrue(search.secondaryStageCompleted);
            testCase.verifyEqual(search.slackCap,search.primaryOptimum,AbsTol=0);
            testCase.verifyLessThanOrEqual(original.safety,search.slackCap+testCase.model.cfg.solver.constraintTolerance);
        end
        function reportedCostMeasuresStatesAndInputsRelativeToTheActualAnchor(testCase)
            model=testCase.model;model.target=[];model.targetEpoch=[];
            model.initialState(2)=model.initialState(2)+.25;
            model.terminalContext=terminalSafeSet.context(model);
            solution=localSolve(model,testCase.anchor,1);
            testCase.assertNotEmpty(solution);
            cfg=testCase.model.cfg;
            stateUnits=[.5;.5;.1;.25;.5;.2];
            inputUnits=[cfg.clf.certificationSteeringRadians;cfg.clf.certificationBrakingRatio];
            stateCost=mean(sum(((solution.states(:,2:end)-testCase.anchor.states(:,2:end))./stateUnits).^2,1));
            inputCost=mean(sum(((solution.inputs-testCase.anchor.inputs)./inputUnits).^2,1));
            testCase.verifyGreaterThan(stateCost+inputCost,1e-6);
            testCase.verifyEqual(solution.referenceStateDeviation,stateCost,RelTol=1e-10);
            testCase.verifyEqual(solution.referenceInputDeviation,inputCost,RelTol=1e-10);
            testCase.verifyEqual(solution.referenceDeviationCost,.5*(stateCost+inputCost),RelTol=1e-10);
        end
        function zeroWeightDoesNotReenableClfDuringAvoidance(testCase)
            solution=localSolve(testCase.model,testCase.anchor,0);
            testCase.assertNotEmpty(solution);
            testCase.verifyEqual(solution.referenceDeviationCost,0,AbsTol=0);
            testCase.verifyTrue(isnan(solution.secondaryObjectiveValue));
            testCase.verifyEqual(solution.objectiveValue,solution.safety,AbsTol=0);
            testCase.verifyFalse(solution.clfEnabled);
        end
        function pathAttractionReducesDriftWithoutBuyingSafetySlack(testCase)
            [model,seed]=localDisplacedReference(testCase.model);
            model.cfg.rti.nominalTrackingWeight=0;
            baseline=localConfiguredSolve(model,seed);
            model.cfg.rti.nominalTrackingWeight=1;
            [attracted,search]=localConfiguredSolve(model,seed);
            testCase.assertNotEmpty(baseline);testCase.assertNotEmpty(attracted);
            testCase.verifyFalse(attracted.clfEnabled);
            testCase.verifyEqual(attracted.safety,baseline.safety,AbsTol=1e-6);
            testCase.verifyLessThan(norm(attracted.states(2,2:end)),norm(baseline.states(2,2:end))-.01);
            testCase.verifyTrue(search.secondaryStageCompleted);
            testCase.verifyLessThanOrEqual(search.solverCalls,2);
        end
        function softInputVariationReducesTheFirstCommandJump(testCase)
            [model,seed]=localDisplacedReference(testCase.model);
            model.cfg.rti.inputVariationWeight=0;
            baseline=localConfiguredSolve(model,seed);
            model.cfg.rti.inputVariationWeight=10;
            smooth=localConfiguredSolve(model,seed);
            testCase.assertNotEmpty(baseline);testCase.assertNotEmpty(smooth);
            units=[model.cfg.clf.certificationSteeringRadians;model.cfg.clf.certificationBrakingRatio];
            testCase.verifyLessThan(norm((smooth.inputs(:,1)-model.previousInput)./units), ...
                norm((baseline.inputs(:,1)-model.previousInput)./units));
            testCase.verifyEqual(smooth.safety,baseline.safety,AbsTol=1e-6);
            increments=diff([model.previousInput,smooth.inputs],1,2)./units;
            testCase.verifyEqual(smooth.inputVariationCost,5*sum(increments.^2,'all'),RelTol=1e-10);
        end
        function aZeroSlackCruiseReferenceNeedsNoDeparture(testCase)
            cfg=testCase.model.cfg;
            ego=struct('position',[0;0],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
            reference=nonlinearBicycleModel.cruise(cfg,0);ego.heldActuatorInput=reference.input;
            [~,~,prediction]=collisionAvoidanceController(ego,[],[],cfg,[]);
            testCase.verifyLessThan(prediction.metadata.referenceDeviationCost,1e-4);
            testCase.verifyLessThan(prediction.metadata.clfSlack,1e-4);
            testCase.verifyEqual(prediction.metadata.referenceDeviationWeight,10);
        end
    end
end

function request=localPriorityProblem(model,anchor,demand,weight,performanceWeights)
    if nargin<5,performanceWeights=[0,0];end
    model.cfg.rti.referenceDeviationWeight=weight;
    model.cfg.rti.nominalTrackingWeight=performanceWeights(1);model.cfg.rti.inputVariationWeight=performanceWeights(2);
    [p,~,model]=solvePredictiveControl("probe",struct('operation',"formulate", ...
        'model',model,'anchor',anchor,'inherited',true));
    % A small constrained choice: correction + slack >= demand. The
    % correction is at most 0.5, so safety and proximity can conflict.
    count=numel(p.lower);p.equal=sparse(0,count);p.rhs=zeros(0,1);p.cones=[];
    p.lower(:)=0;p.upper(:)=0;
    u=p.inputIndices(1,1);s=p.slackIndices(1);p.upper(u)=.5;p.upper(s)=Inf;
    p.a=sparse([1,1],[u,s],[-1,-1],1,count);p.b=-demand;
    request=struct('operation',"solve",'model',model,'anchor',anchor,'problem',p);
end

function request=localWitnessProblem(model,anchor)
    request=localPriorityProblem(model,anchor,0,10);p=request.problem;
    count=numel(p.lower);ix=p.stateIndices(:);
    p.equal=sparse(1:numel(ix),ix,1,numel(ix),count);p.rhs=zeros(numel(ix),1);p.rhs(8)=.5;
    p.a=sparse(0,count);p.b=zeros(0,1);p.lower(ix)=-Inf;p.upper(ix)=Inf;
    request.problem=p;
end

function [solution,search]=localSolve(model,anchor,weight)
    model.cfg.rti.referenceDeviationWeight=weight;
    model.cfg.rti.nominalTrackingWeight=0;model.cfg.rti.inputVariationWeight=0;
    [problem,~,model]=solvePredictiveControl("probe",struct('operation',"formulate", ...
        'model',model,'anchor',anchor,'inherited',false));
    [solution,search]=solvePredictiveControl("probe",struct('operation',"solve", ...
        'model',model,'anchor',anchor,'problem',problem));
end

function [model,anchor]=localDisplacedReference(model)
    model.initialState=[0;3;0;8;0;0];model.previousInput=model.nominalReference.input;
    model.target=[20;20;0;0;0;0;model.cfg.vehicle.lr;2.4;.95;0;0];
    model.targetEpoch=model.target;model.terminalContext=terminalSafeSet.context(model);
    anchor=struct('states',zeros(6,33),'inputs',repmat(model.previousInput,1,32));
    anchor.states(:,1)=model.initialState;
    for k=1:32
        anchor.states(:,k+1)=nonlinearBicycleModel.sample(anchor.states(:,k),anchor.inputs(:,k),model.cfg);
    end
end

function [solution,search]=localConfiguredSolve(model,anchor)
    [problem,~,model]=solvePredictiveControl("probe",struct('operation',"formulate", ...
        'model',model,'anchor',anchor,'inherited',false));
    [solution,search]=solvePredictiveControl("probe",struct('operation',"solve", ...
        'model',model,'anchor',anchor,'problem',problem));
end
