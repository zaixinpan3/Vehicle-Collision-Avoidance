classdef ltvBicyclePredictionTest < matlab.unittest.TestCase
    %ltvBicyclePredictionTest Finite prediction dynamics and error containment.

    methods (TestClassSetup)
        function addModelPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
        end
    end

    methods (Test)
        function initializationPreservesSeedMapsButCannotIssueAControl(testCase)
            model = localModel();
            model.cfg.model.linearizationPolicy = "trajectory";
            model.cfg.controller.certifiedSteps = 1;
            model.initialEgoState = [10;0.2;0.03;10;0.2;0.1];
            model.initialFrenetErrorBound = [.01;.01;.001;.02;.01;.005];
            model.previousInput = [0.03;0.05];
            model.linearizationInputs = repmat([0.03;0.05],1,model.horizonSteps);
            full = ltvBicycleModel.finitePredict(model,[]);
            initial = ltvBicycleModel.finitePredict(model,[],true);
            testCase.verifyEqual(initial.referencePlan,full.referencePlan,AbsTol=1e-14);
            testCase.verifyEqual(initial.egoStateMatrix,full.egoStateMatrix,AbsTol=1e-12);
            testCase.verifyEqual(initial.egoStateOffset,full.egoStateOffset,AbsTol=1e-12);
            testCase.verifyError(@() formulateAvoidanceProblem(model,initial,initial.referencePlan), ...
                "collisionAvoidanceController:uncertifiedInitialization");
        end

        function futureDisturbancesStayContainedWithoutRepeatedBoxInflation(testCase)
            model = localModel();
            model.horizonSteps = 30;
            model.cfg.controller.horizonSteps = 30;
            model.cfg.controller.certifiedSteps = 1;
            model.initialEgoState(4) = 10;
            model.initialFrenetErrorBound = zeros(6,1);
            model.previousInput = zeros(2,1);
            model.cfg.model.plantModelResidualRateBound = [0;0;0;0;0;0.2];
            prediction = ltvBicycleModel.finitePredict(model,[]);
            stream = RandStream('mt19937ar','Seed',713);
            samples = prediction.domainErrorBound(:,2).*(2*rand(stream,6,200)-1);
            reboxed = prediction.domainErrorBound(:,2);
            for stage = 2:model.horizonSteps
                a = prediction.stageMatrixA(:,:,stage);
                disturbance = stateUncertainty.heldDisturbance( ...
                    prediction.continuousA(:,:,stage),model.cfg.model.plantModelResidualRateBound,model.sampleTime);
                samples = a*samples+disturbance.*(2*rand(stream,6,200)-1);
                testCase.verifyLessThanOrEqual(max(abs(samples),[],2), ...
                    prediction.domainErrorBound(:,stage+1)+1e-11);
                reboxed = abs(a)*reboxed+disturbance;
            end
            testCase.verifyLessThan(prediction.domainErrorBound(3,end),0.99*reboxed(3));
        end
        function executionReservesContainDisturbanceWithoutReversingDamping(testCase)
            model = localModel();
            model.initialFrenetErrorBound = zeros(6,1);
            model.previousInput = zeros(2,1);
            model.cfg.model.plantModelResidualRateBound = [0;0;0;0;0;0.2];
            prediction = ltvBicycleModel.finitePredict(model,[]);
            rate = model.cfg.model.plantModelResidualRateBound;
            a = prediction.continuousA(:,:,2);
            for sign = [-1,1]
                [~,errorState] = ode45(@(time,error) a*error ...
                    +sign*(1-2*(time>model.sampleTime/2))*rate, ...
                    [0,model.sampleTime],zeros(6,1),odeset(RelTol=1e-10,AbsTol=1e-12));
                testCase.verifyLessThanOrEqual(abs(errorState(end,:).'), ...
                    prediction.executionReserve(:,2)+1e-10);
            end
            testCase.verifyGreaterThan(prediction.executionReserve(3,2),0);
            testCase.verifyLessThan(prediction.executionReserve(6,2),2*rate(6)*model.sampleTime);
        end
        function discreteNominalSensitivitiesMatchIndependentOdePerturbations(testCase)
            model = localModel();
            model.initialEgoState = [10;0.2;0.03;10;0.2;0.1];
            input = [0.04;0.05];
            [~,a,b] = ltvBicycleModel.nominalRollout(model,input);
            point = [model.initialEgoState;input];
            reference = zeros(6,8);
            step = 1e-5;
            for column = 1:8
                offset = zeros(8,1);offset(column) = step;
                endpoints = zeros(6,2);
                for side = 1:2
                    changed = point+(3-2*side)*offset;
                    [~,trajectory] = ode45(@(~,x) localNonlinearFlow(x,changed(7:8),0,model.cfg), ...
                        [0,model.sampleTime],changed(1:6),odeset(RelTol=1e-10,AbsTol=1e-12));
                    endpoints(:,side) = trajectory(end,:).';
                end
                reference(:,column) = diff(fliplr(endpoints),1,2)/(2*step);
            end
            testCase.verifyEqual([a,b],reference,AbsTol=2e-3,RelTol=1e-3);
        end
        function nonlinearPredictionAgreesWithIndependentHeldInputIntegration(testCase)
            model = localModel();
            model.initialEgoState = [20;-1;-0.2;9;-0.5;-0.35];
            model.lane.segmentCurvature(:) = 0.01;
            inputs = [-0.13,0.14,0.05;-0.35,-0.2,0.1];
            predicted = ltvBicycleModel.nominalRollout(model,inputs);
            actual = model.initialEgoState;
            for stage = 1:size(inputs,2)
                [~,states] = ode45(@(~,state) localNonlinearFlow( ...
                    state,inputs(:,stage),0.01,model.cfg),[0,model.sampleTime],actual, ...
                    odeset(RelTol=1e-11,AbsTol=1e-13));
                actual = states(end,:).';
                testCase.verifyEqual(predicted(:,stage+1),actual,AbsTol=1e-4);
            end
        end
        function maneuverLinearizationMatchesAxleForceBalanceAndKinematicDerivatives(testCase)
            cfg = collisionAvoidanceControllerConfig();
            state = [20;-1;-0.2;9;-0.5;-0.35];
            input = [-0.13;-0.35];curvature = 0.01;
            point = struct("state",state,"input",input);
            [a,b,c] = ltvBicycleModel.continuousMatrices(curvature,9,cfg,[],0,point);
            expected = localNonlinearFlow(state,input,curvature,cfg);
            testCase.verifyEqual(a*state+b*input+c,expected,AbsTol=1e-11);
            combined = [state;input];jacobian = zeros(6,8);step = 1e-6;
            for index = 1:8
                plus = combined;minus = combined;
                plus(index) = plus(index)+step;minus(index) = minus(index)-step;
                jacobian(:,index) = (localNonlinearFlow(plus(1:6),plus(7:8),curvature,cfg) ...
                    -localNonlinearFlow(minus(1:6),minus(7:8),curvature,cfg))/(2*step);
            end
            testCase.verifyEqual([a,b],jacobian,AbsTol=2e-7);
        end
        function brakingRatioUsesThePaperLongitudinalForceScale(testCase)
            cfg = collisionAvoidanceControllerConfig(struct( ...
                "roadLoad", struct("dragCoefficient", 0, "rollingCoefficient", 0)));
            [stateMatrix, inputMatrix, affine] = ltvBicycleModel.stageMatrices( ...
                0.0, 10.0, 0.05, cfg);
            next = stateMatrix*[0.0; 0.0; 0.0; 10.0; 0.0; 0.0] ...
                + inputMatrix*[0.0; -0.4]+affine;
            testCase.verifyEqual(next(4), 10-0.4*modifiedFialaTire.accelerationGain(cfg)*0.05, AbsTol=1.0e-13);
            testCase.verifyEqual(next(1), 0.5-0.2*modifiedFialaTire.accelerationGain(cfg)*0.05^2, AbsTol=1.0e-13);
        end

        function stiffLateralDynamicsRemainStableUnderHeldInputIntegration(testCase)
            cfg = collisionAvoidanceControllerConfig(struct( ...
                "model", struct("scheduleSpeedFloor", 0.5)));
            stateMatrix = ltvBicycleModel.stageMatrices(0.0, 0.5, 0.05, cfg);
            testCase.verifyTrue(all(isfinite(stateMatrix), "all"));
            testCase.verifyLessThan(max(abs(eig(stateMatrix(5:6, 5:6)))), 1.0);
        end

        function heldInputPredictionIsInvariantToIntegrationSubdivision(testCase)
            model = localModel();
            [fullA, fullB, fullC] = ltvBicycleModel.stageMatrices( ...
                0.0025, 15.0, model.sampleTime, model.cfg);
            [halfA, halfB, halfC] = ltvBicycleModel.stageMatrices( ...
                0.0025, 15.0, 0.5*model.sampleTime, model.cfg);
            testCase.verifyEqual(fullA, halfA*halfA, AbsTol=1.0e-13);
            testCase.verifyEqual(fullB, halfA*halfB+halfB, AbsTol=1.0e-13);
            testCase.verifyEqual(fullC, halfA*halfC+halfC, AbsTol=1.0e-13);
        end

        function aHeldAccelerationBiasAlsoChangesPosition(testCase)
            model = localModel();
            model.longitudinalAccelerationBias = 0.7;
            prediction = ltvBicycleModel.finitePredict(model,[]);
            state = prediction.egoStateOffset(:, 2);
            testCase.verifyEqual(state(4), 4.0+0.7*model.sampleTime, AbsTol=1.0e-13);
            testCase.verifyEqual(state(1), 4.0*model.sampleTime ...
                + 0.5*0.7*model.sampleTime^2, AbsTol=1.0e-13);
        end

        function errorBoundsContinuePropagatingBeyondTheFirstStep(testCase)
            model = localModel();
            model.initialFrenetErrorBound = [0.01; 0.02; 0.003; 0.04; 0.02; 0.005];
            model.cfg.model.ltvModelErrorRateBound = 0.001*ones(6, 1);

            prediction = ltvBicycleModel.finitePredict(model,[]);

            time = (0:model.horizonSteps)*model.sampleTime;
            expectedPositionRadius = .01+.04*time+.5*.001*time.^2;
            expectedVelocityRadius = .04+.001*time;
            testCase.verifyGreaterThanOrEqual(prediction.domainErrorBound(1,:), ...
                expectedPositionRadius-1e-12);
            testCase.verifyGreaterThanOrEqual(prediction.domainErrorBound(4,:), ...
                expectedVelocityRadius-1e-12);
            testCase.verifyGreaterThan(prediction.domainErrorBound(1,end), ...
                prediction.domainErrorBound(1,3));
        end

        function accelerationDisturbanceAlsoEnlargesPositionAtEveryStep(testCase)
            model = localModel();
            model.cfg.model.ltvModelErrorRateBound(4) = 0.1;
            prediction = ltvBicycleModel.finitePredict(model,[]);
            time = (0:2)*model.sampleTime;

            testCase.verifyEqual(prediction.domainErrorBound(1, 1:3), ...
                0.5*0.1*time.^2, AbsTol=1e-10);
            testCase.verifyEqual(prediction.domainErrorBound(4, 1:3), ...
                0.1*time, AbsTol=1e-10);
        end


    end
end

function model = localModel()
    % Isolate held-flow and disturbance integration in the zero-road-load limit.
    cfg = collisionAvoidanceControllerConfig(struct("controller", struct("horizonSteps", 4), ...
        "roadLoad", struct("dragCoefficient", 0, "rollingCoefficient", 0)));
    ego = struct("positionX", 0.0, "positionY", 0.0, "yawAngle", 0.0, ...
        "longitudinalVelocity", 4.0, "lateralVelocity", 0.0, "yawRate", 0.0);
    [~, lane] = readPlanningInputs(ego, [], [0.0, 0.0; 2000.0, 0.0], cfg);
    model = struct("cfg", cfg, "horizonSteps", 4, ...
        "inputDimension", 2, "sampleTime", cfg.controller.sampleTime, ...
        "initialEgoState", [0.0; 0.0; 0.0; 4.0; 0.0; 0.0], ...
        "initialFrenetErrorBound", zeros(6, 1), "previousInput", zeros(2,1), ...
        "longitudinalAccelerationBias", 0.0, "lane", lane);
end

function derivative = localNonlinearFlow(state,input,curvature,cfg)
    vx = state(4);vy = state(5);r = state(6);delta = input(1);
    parameters = modifiedFialaTire.parameters(cfg);
    slip = [atan2(vy+cfg.vehicle.lf*r,vx)-delta;atan2(vy-cfg.vehicle.lr*r,vx)];
    fy = modifiedFialaTire.evaluate(slip,input(2),cfg);
    fx = parameters.longitudinalForceScale*input(2);
    frontX = fx(1)*cos(delta)-fy(1)*sin(delta);
    frontY = fx(1)*sin(delta)+fy(1)*cos(delta);
    stationRate = (vx*cos(state(3))-vy*sin(state(3)))/(1-curvature*state(2));
    derivative = [stationRate;vx*sin(state(3))+vy*cos(state(3));r-curvature*stationRate; ...
        (frontX+fx(2)-longitudinalRoadLoad(vx,cfg))/cfg.vehicle.m+vy*r; ...
        (frontY+fy(2))/cfg.vehicle.m-vx*r; ...
        (cfg.vehicle.lf*frontY-cfg.vehicle.lr*fy(2))/cfg.vehicle.Iz];
end
