classdef referenceDeviationObjectiveTest < matlab.unittest.TestCase
    % Reference proximity is used only by the cruise CLF optimization.
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
        function avoidanceIgnoresTheCruiseReferenceWeight(testCase)
            [original,search]=localSolve(testCase.model,testCase.anchor,10);
            alternative=localSolve(testCase.model,testCase.anchor,1000);
            testCase.assertNotEmpty(original);testCase.assertNotEmpty(alternative);
            testCase.verifyEqual(original.inputs,alternative.inputs,AbsTol=1e-10);
            testCase.verifyEqual(original.states,alternative.states,AbsTol=1e-10);
            testCase.verifyEqual(original.referenceDeviationWeight,0);
            testCase.verifyEqual(original.referenceDeviationCost,0);
            testCase.verifyEqual(search.stages.objective,"pcbfSlack");
            testCase.verifyFalse(search.secondaryStageAttempted);
            testCase.verifyTrue(isnan(search.slackCap));
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
        function aZeroSlackCruiseReferenceNeedsNoDeparture(testCase)
            cfg=testCase.model.cfg;
            ego=struct('position',[0;0],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
            [~,~,prediction]=collisionAvoidanceController(ego,[],[],cfg,[]);
            testCase.verifyLessThan(prediction.metadata.referenceDeviationCost,1e-4);
            testCase.verifyLessThan(prediction.metadata.clfSlack,1e-4);
            testCase.verifyEqual(prediction.metadata.referenceDeviationWeight,10);
        end
    end
end

function [solution,search]=localSolve(model,anchor,weight)
    model.cfg.rti.referenceDeviationWeight=weight;
    [problem,~,model]=solvePredictiveControl("probe",struct('operation',"formulate", ...
        'model',model,'anchor',anchor,'inherited',false));
    [solution,search]=solvePredictiveControl("probe",struct('operation',"solve", ...
        'model',model,'anchor',anchor,'problem',problem));
end
