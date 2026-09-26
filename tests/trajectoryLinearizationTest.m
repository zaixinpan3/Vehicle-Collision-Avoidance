classdef trajectoryLinearizationTest < matlab.unittest.TestCase
    %trajectoryLinearizationTest One-pass avoidance trajectory Jacobians.
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function avoidanceUsesEachPredictedStateAndInput(testCase)
            model = localModel();
            cruise = ltvBicycleModel.sampledCruise(model);
            [prediction,anchor] = hardEncounterBarrier.predict(model,cruise);
            expected = ltvBicycleModel.nominalRollout(model,model.initializationPlan);
            tireStates = cell2mat(cellfun(@(t)t.operatingState, ...
                prediction.tireModels.',UniformOutput=false));
            tireInputs = cell2mat(cellfun(@(t)t.operatingInput, ...
                prediction.tireModels.',UniformOutput=false));

            testCase.verifyEqual(prediction.linearizationPolicy,"trajectory");
            testCase.verifyEqual(anchor,model.initializationPlan(:),AbsTol=0);
            testCase.verifyEqual(tireStates,expected(:,1:end-1),AbsTol=1e-12);
            testCase.verifyEqual(tireInputs,model.initializationPlan,AbsTol=1e-12);
            testCase.verifyGreaterThan(norm(prediction.continuousA(:,:,1) ...
                -prediction.continuousA(:,:,end),'fro'),.1);
        end

        function aSaturatedAnchorHasTheSaturatedTireDerivative(testCase)
            model = localModel();
            model.initializationPlan(1,1) = -.45;
            [prediction,~] = hardEncounterBarrier.predict(model,ltvBicycleModel.sampledCruise(model));
            tire = prediction.tireModels{1};
            x = model.initialEgoState;u = model.initializationPlan(:,1);
            slip = atan2([x(5)+model.cfg.vehicle.lf*x(6); ...
                x(5)-model.cfg.vehicle.lr*x(6)],x(4))-[u(1);0];
            expected = modifiedFialaTire.evaluate(slip,u(2),model.cfg);

            testCase.verifyEqual(tire.force,expected,AbsTol=1e-10);
            testCase.verifyEqual(tire.state(1,:),zeros(1,6),AbsTol=1e-12);
            testCase.verifyEqual(tire.input(1,1),0,AbsTol=1e-12);
        end

        function varyingReferenceJacobianIncludesSpatialCurvature(testCase)
            model = localModel();
            model.lane.referenceCurve = laneGeometry.validateReferenceCurve(struct( ...
                'origin',[0;0],'heading',0,'curvature',0,'length',100, ...
                'curvatureProfile',[0,0;100,.01],'continuation',"constantCurvature"));
            model.initialEgoState = [20;1;.1;8;.2;.1];
            [stages,~] = ltvBicycleModel.trajectoryStages(model,model.initializationPlan(:,1));
            x = model.initialEgoState;u = model.initializationPlan(:,1);h = 1e-4;
            plus = x;minus = x;plus(1)=plus(1)+h;minus(1)=minus(1)-h;
            derivative = (localFlow(plus,u,model)-localFlow(minus,u,model))/(2*h);

            testCase.verifyEqual(stages(1).continuousA(:,1),derivative,AbsTol=1e-9);
            testCase.verifyEqual(stages(1).continuousA*x+stages(1).continuousB*u ...
                +stages(1).continuousC,localFlow(x,u,model),AbsTol=1e-10);
        end

        function nextFrameRefreshesFromTheShiftedPlanAndMeasuredState(testCase)
            [ego,target,road,cfg] = encounterTestFixture.crossing();
            cfg.model.linearizationPolicy = "trajectory";
            cfg.solver.frameDeadlineSeconds = Inf;
            cfg.solver.certificateSearchTimeLimit = 30;
            [~,~,first,stored] = collisionAvoidanceController(ego,target,road,cfg,[]);
            nextEgo = encounterTestFixture.nextEgo(stored,first.model.lane);
            target.targetPositionInertial = target.targetPositionInertial ...
                +cfg.controller.sampleTime*target.targetVelocityInertial;
            [command,~,second] = collisionAvoidanceController(nextEgo,target,road,cfg,stored);
            count = min(size(second.prediction.linearizationInputs,2),size(stored.plan,2)-1);
            next = second.prediction.stageMatrixA(:,:,1)*second.model.initialEgoState ...
                +second.prediction.stageMatrixB(:,:,1)*command.actuatorInput ...
                +second.prediction.stageAffine(:,1);
            error = next(2:6)-second.metadata.clfNextReferenceState(2:6);
            rows = second.program.cones(2)+(1:6);
            clfCone = second.program.b(rows)-second.program.A(rows,:)*second.decision;

            testCase.verifyEqual(second.prediction.linearizationInputs(:,1:count), ...
                stored.plan(:,2:count+1),AbsTol=0);
            testCase.verifyEqual(second.prediction.linearizationStates(:,1), ...
                second.model.initialEgoState,AbsTol=0);
            testCase.verifyFalse(second.metadata.inheritedFeasibleFamily);
            testCase.verifyFalse(second.metadata.recursiveFeasibilityGuaranteed);
            testCase.verifyEqual(second.metadata.clfNextValue, ...
                error.'*second.metadata.clfNextMatrix*error,AbsTol=1e-10);
            testCase.verifyEqual(norm(clfCone(2:end))^2, ...
                second.metadata.clfNextValue,AbsTol=1e-10);
            testCase.verifyEqual(second.metadata.executedContinuousGenerator, ...
                [second.prediction.continuousA(:,:,1),second.prediction.continuousB(:,:,1), ...
                 second.prediction.continuousC(:,1)],AbsTol=0);
            tire = second.prediction.tireModels{1};
            testCase.verifyEqual(command.axleLateralForce, ...
                tire.state*second.model.initialEgoState+tire.input*command.actuatorInput ...
                +tire.constant,AbsTol=1e-10);
        end

        function targetFreePredictionRetainsTheCruiseGenerator(testCase)
            model = localModel();model.encounter = [];
            cruise = ltvBicycleModel.sampledCruise(model);
            prediction = hardEncounterBarrier.predict(model,cruise);
            testCase.verifyEqual(prediction.linearizationPolicy,"cruise");
            testCase.verifyEqual(prediction.continuousA(:,:,1),cruise.stage.continuousA,AbsTol=0);
        end
    end
end

function model = localModel()
    cfg = collisionAvoidanceControllerConfig(struct('referenceSpeed',8));
    ego = struct('position',[0;0],'yaw',.03,'speed',8,'lateralVelocity',.2,'yawRate',.1);
    [~,lane,road] = readPlanningInputs(ego,[],[-100,0;1000,0],cfg);
    model = struct('cfg',cfg,'lane',lane,'road',road,'initialEgoState',[100;.2;.03;8;.2;.1], ...
        'initialFrenetErrorBound',zeros(6,1),'sampleTime',.05,'horizonSteps',4, ...
        'longitudinalAccelerationBias',0,'previousInput',zeros(2,1), ...
        'encounter',struct('key',1),'cruiseCertificate',[], ...
        'initializationPlan',[-.1,-.05,.02,.03;-.2,-.1,0,.02]);
end

function flow = localFlow(state,input,model)
    curvature = laneGeometry.curvature(state(1),model.lane);
    [a,b,c] = ltvBicycleModel.continuousMatrices(curvature,state(4),model.cfg,[],0, ...
        struct('state',state,'input',input));
    flow = a*state+b*input+c;
end
