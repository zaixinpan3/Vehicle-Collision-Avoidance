classdef flowTrajectoryLinearizationTest < matlab.unittest.TestCase
    %flowTrajectoryLinearizationTest Flow references own the model tangents.
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (Test)
        function firstFlowReferenceRebuildsDynamicsTiresAndObjective(testCase)
            [ego,target,road,cfg]=encounterTestFixture.circularCrossing(.01);
            cfg.jointCertificate.inputDeviationWeight=10;
            [command,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            prediction=problem.prediction;
            states=ltvBicycleModel.nominalRollout(problem.model,prediction.linearizationInputs);
            tireInputs=cell2mat(cellfun(@(t)t.operatingInput,prediction.tireModels.',UniformOutput=false));
            tireStates=cell2mat(cellfun(@(t)t.operatingState,prediction.tireModels.',UniformOutput=false));
            coneRows=problem.program.cones(2)+(1:6);
            value=problem.program.b(coneRows)-problem.program.A(coneRows,:)*problem.decision;
            tire=prediction.tireModels{1};

            testCase.verifyEqual(problem.metadata.nominalSource,"flowTrajectory");
            testCase.verifyTrue(problem.metadata.admissionSearch.initialization.modelRebuilt);
            testCase.verifyEqual(prediction.linearizationStates,states,AbsTol=0);
            testCase.verifyEqual(tireStates,states(:,1:end-1),AbsTol=1e-12);
            testCase.verifyEqual(problem.program.inputDeviationCenter,tireInputs(:),AbsTol=0);
            testCase.verifyGreaterThan(norm(prediction.continuousA(:,:,1) ...
                -problem.program.cruiseCertificate.stage.continuousA,'fro'),1e-3);
            testCase.verifyEqual(norm(value(2:end))^2,problem.metadata.clfNextValue,AbsTol=1e-8);
            testCase.verifyEqual(command.axleLateralForce,tire.state*problem.model.initialEgoState ...
                +tire.input*command.actuatorInput+tire.constant,AbsTol=1e-10);
            testCase.verifyFalse(problem.metadata.recursiveFeasibilityGuaranteed);
            testCase.verifyFalse(problem.metadata.admissionSearch.issuedAdmissionWitness);
        end

        function preparedFlowReplayRetainsItsOperatingTrajectory(testCase)
            [ego,target,road,cfg]=encounterTestFixture.circularCrossing(.01);
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            [program,result]=solveHardCbfClf.fixedDirections(problem.program,problem.model,cfg);
            testCase.verifyTrue(result.feasible);
            testCase.verifyEqual(result.decision,problem.decision,AbsTol=1e-10);
            testCase.verifyEqual(program.prediction.linearizationInputs, ...
                problem.prediction.linearizationInputs,AbsTol=0);
            testCase.verifyEqual(program.prediction.linearizationStates, ...
                problem.prediction.linearizationStates,AbsTol=0);
        end

        function failedRefreshedModelsCannotIssueTheOldPlan(testCase)
            [next,target,road,cfg,stored]=localNextFrame();
            cfg.solver.jointFunction=@encounterTestFixture.fail;
            testCase.verifyError(@()collisionAvoidanceController(next,target,road,cfg,stored), ...
                'collisionAvoidanceController:optimizationFailed');
        end

        function invalidPreviousInputsTriggerFlowBeforeOptimization(testCase)
            [~,result,search,model]=localInvalidAnchor([2;0]);
            testCase.verifyEqual(model.nominalSource,"flowTrajectory");
            testCase.verifyEqual(search.previousAnchorFailure, ...
                "collisionAvoidanceController:invalidTrajectoryAnchor");
            testCase.verifyEqual(search.previousPlanSeedStatus,"notAttempted");
            testCase.verifyTrue(result.feasible);
        end

        function invalidPreviousRolloutTriggersFlowBeforeOptimization(testCase)
            [~,result,search,model]=localInvalidAnchor([.69;.99]);
            testCase.verifyEqual(model.nominalSource,"flowTrajectory");
            testCase.verifyEqual(search.previousAnchorFailure, ...
                "collisionAvoidanceController:invalidTireOperatingPoint");
            testCase.verifyEqual(search.previousPlanSeedStatus,"notAttempted");
            testCase.verifyTrue(result.feasible);
        end

        function rejectedPreviousSolveRebuildsFromFlow(testCase)
            [next,target,road,cfg,stored]=localNextFrame();
            cfg.solver.jointFunction=localRejectFirst();
            [~,~,problem]=collisionAvoidanceController(next,target,road,cfg,stored);
            testCase.verifyTrue(startsWith(problem.metadata.admissionSearch.previousPlanSeedStatus,"rejected"));
            testCase.verifyEqual(problem.metadata.nominalSource,"flowTrajectory");
            testCase.verifyGreaterThanOrEqual(problem.metadata.admissionSearch.linearizationRefreshes,1);
            testCase.verifyTrue(problem.metadata.planCertified);
        end

        function alternateSideRebuildsItsOwnReference(testCase)
            [ego,target,road,cfg]=encounterTestFixture.circularCrossing(.01);
            cfg.solver.jointFunction=localRejectFirst();
            cfg.feedbackPrediction.targetReaction.inputWeightScales=Inf;
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(problem.metadata.admissionSearch.alternateInitialization.modelRebuilt);
            testCase.verifyEqual(problem.metadata.admissionSearch.linearizationRefreshes,2);
            testCase.verifyEqual(problem.metadata.admissionSearch.alternateSeedStatus,"accepted");
            testCase.verifyEqual(problem.prediction.linearizationInputs,problem.model.initializationPlan,AbsTol=0);
            testCase.verifyEqual(problem.prediction.linearizationStates, ...
                ltvBicycleModel.nominalRollout(problem.model,problem.model.initializationPlan),AbsTol=0);
        end

        function projectedFlowRespectsAmplitudeAndSlewEvenWhenSolveFails(testCase)
            [ego,target,road,cfg]=encounterTestFixture.circularCrossing(.01);
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            model=problem.model;model=rmfield(model,'initializationPlan');
            model.nominalSource="cruiseInitialization";
            model.cfg.model.frontWheelSteeringRateMaximum=1;
            model.cfg.model.brakingRatioRateMaximum=.5;
            cfg=model.cfg;cfg.solver.jointFunction=@encounterTestFixture.fail;
            program=solveHardCbfClf.prepare(model);
            [program,result,search,model]=solveHardCbfClf.fixedDirections(program,model,cfg);
            input=program.prediction.linearizationInputs;
            change=diff([model.previousInput,input],1,2);
            testCase.verifyGreaterThan(search.linearizationRefreshes,0);
            testCase.verifyGreaterThan(search.initialization.inputProjectionNorm,0);
            testCase.verifyLessThanOrEqual(max(abs(input(1,:))),cfg.model.frontWheelSteeringAngleMaximum);
            testCase.verifyGreaterThanOrEqual(min(input(2,:)),cfg.actuation.brakingRatioMinimum);
            testCase.verifyLessThanOrEqual(max(input(2,:)),cfg.actuation.brakingRatioMaximum);
            testCase.verifyLessThanOrEqual(max(abs(change),[],2),model.sampleTime*[1;.5]+1e-12);
            testCase.verifyFalse(result.feasible);
            testCase.verifyEmpty(result.decision);
        end
    end
end

function [next,target,road,cfg,stored]=localNextFrame()
    [ego,target,road,cfg]=encounterTestFixture.circularCrossing(.01);
    [~,~,problem,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
    next=encounterTestFixture.nextEgo(stored,problem.model.lane);
    target.targetPositionInertial=target.targetPositionInertial ...
        +cfg.controller.sampleTime*target.targetVelocityInertial;
end

function hook=localRejectFirst()
    calls=0;
    hook=@solve;
    function result=solve(~,program)
        calls=calls+1;
        if calls==1
            result=encounterTestFixture.fail([],[]);
        else
            result=program.defaultSolver();
        end
    end
end

function [program,result,search,model]=localInvalidAnchor(input)
    [ego,target,road,cfg]=encounterTestFixture.circularCrossing(.01);
    [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
    model=problem.model;model.nominalSource="shiftedPreviousSolution";
    model.initializationPlan=repmat(input,1,model.horizonSteps);
    program=solveHardCbfClf.prepare(model);
    [program,result,search,model]=solveHardCbfClf.fixedDirections(program,model,cfg);
end
