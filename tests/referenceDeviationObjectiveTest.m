classdef referenceDeviationObjectiveTest < matlab.unittest.TestCase
    % A soft reference cost preserves freedom and the primary safety priority.
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
        function aLargerWeightReducesDepartureEvenWhenStartingOnTheRoute(testCase)
            weak=localSolve(testCase.model,testCase.anchor,.01);
            strong=localSolve(testCase.model,testCase.anchor,10);
            testCase.assertNotEmpty(weak);testCase.assertNotEmpty(strong);
            testCase.verifyEqual(strong.clfInitialValue,0,AbsTol=1e-12);
            testCase.verifyLessThan(strong.referenceStateDeviation+strong.referenceInputDeviation, ...
                .9*(weak.referenceStateDeviation+weak.referenceInputDeviation));
            testCase.verifyGreaterThan(strong.clfSlack,weak.clfSlack);
        end
        function referencePreferenceCannotSpendBeyondThePrimarySlackCap(testCase)
            [solution,search]=localSolve(testCase.model,testCase.anchor,10);
            testCase.assertNotEmpty(solution);
            testCase.verifyLessThanOrEqual(solution.safety,search.slackCap+1e-5);
            testCase.verifyEqual(search.stages(2).value,solution.clfSlack+solution.referenceDeviationCost,AbsTol=1e-12);
            testCase.verifyEqual(search.stages(2).objective,"clfSlackAndReferenceDeviation");
        end
        function reportedCostMeasuresStatesAndInputsRelativeToTheActualAnchor(testCase)
            solution=localSolve(testCase.model,testCase.anchor,1);
            testCase.assertNotEmpty(solution);
            cfg=testCase.model.cfg;
            stateUnits=[.5;.5;.1;.25;.5;.2];
            inputUnits=[cfg.clf.certificationSteeringRadians;cfg.clf.certificationBrakingRatio];
            stateCost=mean(sum(((solution.states(:,2:end)-testCase.anchor.states(:,2:end))./stateUnits).^2,1));
            inputCost=mean(sum(((solution.inputs-testCase.anchor.inputs)./inputUnits).^2,1));
            testCase.verifyEqual(solution.referenceStateDeviation,stateCost,RelTol=1e-10);
            testCase.verifyEqual(solution.referenceInputDeviation,inputCost,RelTol=1e-10);
            testCase.verifyEqual(solution.referenceDeviationCost,.5*(stateCost+inputCost),RelTol=1e-10);
        end
        function zeroWeightProvidesAnExplicitClfOnlyAblation(testCase)
            solution=localSolve(testCase.model,testCase.anchor,0);
            testCase.assertNotEmpty(solution);
            testCase.verifyEqual(solution.referenceDeviationCost,0,AbsTol=0);
            testCase.verifyEqual(solution.secondaryObjectiveValue,solution.clfSlack,AbsTol=0);
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
