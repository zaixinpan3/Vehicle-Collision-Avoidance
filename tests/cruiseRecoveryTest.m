classdef cruiseRecoveryTest < matlab.unittest.TestCase
    %cruiseRecoveryTest Quadratic input effort, CLF relaxation and initialization.

    properties (TestParameter)
        speed = struct("moderateError", 14.4, "actuatorLimitedError", 8.0);
        nearCruiseSpeed = struct("underspeed", 14.95, "overspeed", 15.05);
    end

    methods (TestClassSetup)
        function addControllerPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "scripts")));
        end
    end

    methods (Test)
        function aFailedDiscardedProbeDoesNotVetoTheRealController(testCase)
            cfg = localConfiguration();
            cfg.solver.jointFunction = localFailFirstSolve();
            ego = struct("position", [0; 0], "yawAngle", 0, "speed", 15);
            ego.stateTime = 0;
            ego.perception = struct("time",0,"range",30,"completeWithinRange",true);
            road = [0, 0; 2000, 0];
            preparation = prepareCollisionAvoidanceController(ego, road, cfg);
            [command, ~, problem] = collisionAvoidanceController(ego, [], road, cfg, []);
            testCase.verifyFalse(preparation.allProbesCertified);
            testCase.verifyEqual(preparation.attemptedCalls, 3);
            testCase.verifyEqual(preparation.discardedCommandCount, 2);
            testCase.verifyEqual(preparation.failureIdentifier(1, 1), ...
                "collisionAvoidanceController:noCertifiedContinuation");
            testCase.verifyTrue(all(preparation.probeCertified(2:end, :), "all"));
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyNotEmpty(command);
        end

        function quadraticCostUsesTheCertificateOperatingInputAndRelaxation(testCase)
            cfg = localConfiguration();
            cfg.clf.frontWheelSteeringAngleWeight = 2;
            cfg.clf.brakingRatioWeight = 3;
            ego = struct("position", [0; 0], "yawAngle", 0, "speed", 14.4);
            ego.stateTime = 0;
            ego.perception = struct("time",0,"range",30,"completeWithinRange",true);
            [~, ~, problem] = collisionAvoidanceController(ego, [], [0, 0; 2000, 0], cfg, []);
            decision = zeros(problem.layout.decisionCount, 1);
            decision(1:2) = [0.02; 0.3];
            decision(end) = 0.7;
            expectedCost = localExplicitCost(problem, cfg, decision);
            testCase.verifyEqual(localObjective(problem.qp, decision), expectedCost, AbsTol=1e-8);
            decision(3) = 0.04;
            testCase.verifyEqual(localObjective(problem.qp, decision), ...
                localExplicitCost(problem, cfg, decision), AbsTol=1e-8);
            testCase.verifyFalse(isfield(problem.qp, "preferredInput"));
            testCase.verifyFalse(isfield(problem.qp.clf.certificate, "sampledFeedbackGain"));
        end

        function nominalCruiseWithoutRoadLoadDoesNotCommandDeparture(testCase)
            cfg = localConfiguration();
            cfg.roadLoad.dragCoefficient = 0.0;
            cfg.roadLoad.rollingCoefficient = 0.0;
            ego = struct("position", [0; 0], "yawAngle", 0, "speed", cfg.referenceSpeed);
            ego.stateTime = 0;
            ego.perception = struct("time",0,"range",30,"completeWithinRange",true);
            [command, ~, problem] = collisionAvoidanceController(ego, [], [0, 0; 2000, 0], cfg, []);

            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyLessThan(norm(command.actuatorInput, inf), 1.0e-3);
            testCase.verifyLessThan(problem.metadata.clfValueProfile(2), 1.0e-8);
            testCase.verifyLessThan(max(problem.metadata.clfRelaxation), 1e-4);
        end

        function nearCruiseCorrectionDoesNotOvershoot(testCase, nearCruiseSpeed)
            cfg = localConfiguration();
            ego = struct("position", [0; 0], "yawAngle", 0, "speed", nearCruiseSpeed);
            ego.stateTime = 0;
            ego.perception = struct("time",0,"range",30,"completeWithinRange",true);
            [command, ~, problem] = collisionAvoidanceController(ego, [], [0, 0; 2000, 0], cfg, []);
            initialError = nearCruiseSpeed-cfg.referenceSpeed;
            nextError = initialError+cfg.controller.sampleTime ...
                *command.bodyLongitudinalVelocityDerivative;

            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyLessThan(initialError*command.bodyLongitudinalVelocityDerivative, 0.0);
            testCase.verifyLessThan(abs(nextError), abs(initialError));
            testCase.verifyLessThan(problem.metadata.clfValueProfile(2), problem.metadata.clfInitialValue);
        end

        function quadraticObjectiveMatchesAnIndependentSolver(testCase, speed)
            cfg = localConfiguration();
            ego = struct("position", [0; 0], "yawAngle", 0, "speed", speed);
            ego.stateTime = 0;
            ego.perception = struct("time",0,"range",30,"completeWithinRange",true);
            road = [0, 0; 2000, 0];
            [~, ~, native] = collisionAvoidanceController(ego, [], road, cfg, []);
            cfg.solver.jointFunction = @localIndependentConicSolve;
            [~, ~, independent] = collisionAvoidanceController(ego, [], road, cfg, []);

            testCase.verifyTrue(native.metadata.planCertified);
            testCase.verifyTrue(independent.metadata.planCertified);
            testCase.verifyEqual(native.metadata.jointObjectiveValue, ...
                independent.metadata.jointObjectiveValue, AbsTol=1.0e-5, RelTol=1.0e-6);
            testCase.verifyEqual(native.metadata.clfRelaxationCost, ...
                cfg.controller.sampleTime*cfg.clf.relaxationWeight*sum(native.metadata.clfRelaxation.^2), AbsTol=1.0e-10);
            testCase.verifyEqual(native.metadata.jointObjectiveValue, ...
                localObjective(native.qp, native.decision), AbsTol=1.0e-10);
            testCase.verifyLessThanOrEqual(native.metadata.hardRowViolation, 1.0e-5);
        end

        function preparationDoesNotConsumeOrReplaceTheLiveCertificate(testCase)
            cfg = localConfiguration();
            ego = struct("position", [0; 0], "yawAngle", 0, "speed", 15);
            ego.stateTime = 0;
            ego.perception = struct("time",0,"range",30,"completeWithinRange",true);
            road = [0, 0; 2000, 0];
            collisionAvoidanceController("resetNominalTrajectory");
            testCase.addTeardown(@() collisionAvoidanceController("resetNominalTrajectory"));
            [~, ~, ~, certificate] = collisionAvoidanceController(ego, [], road, cfg);
            next = certificate.predictedState(:, 2);
            ego.position = next(1:2);
            ego.yawAngle = next(3);
            ego.speed = next(4);
            ego.lateralVelocity = next(5);
            ego.yawRate = next(6);
            ego.stateTime = cfg.controller.sampleTime;
            ego.perception.time = ego.stateTime;
            ego.heldActuatorInput = certificate.appliedInput;
            preparation = prepareCollisionAvoidanceController(ego, road, cfg);
            [actual, ~, diagnostic] = collisionAvoidanceController(ego, [], road, cfg);
            expected = collisionAvoidanceController(ego, [], road, cfg, certificate);
            testCase.verifyTrue(preparation.performed);
            testCase.verifyEqual(preparation.discardedCommandCount, 3);
            testCase.verifyTrue(diagnostic.metadata.certificateCompatible);
            testCase.verifyEqual(actual.actuatorInput, expected.actuatorInput, AbsTol=1.0e-9);
        end
    end
end

function hook = localFailFirstSolve()
    count = 0;
    hook = @solve;
    function result = solve(~, program)
        count = count+1;
        if count == 1
            result = struct("decision", [], "exitFlag", -999, "output", struct());
        else
            result = program.defaultSolver();
        end
    end
end

function cfg = localConfiguration()
    cfg = collisionAvoidanceControllerConfig(struct("controller", struct("horizonSteps", 4)));
end

function value = localObjective(qp, decision)
    value = 0.5*decision.'*qp.Hessian*decision+qp.linear.'*decision+qp.constant;
end

function result = localIndependentConicSolve(~, program)
    initial = program.defaultSolver();
    hessian = program.P+triu(program.P,1).';
    options = optimoptions("fmincon", "Display", "off", "Algorithm", "sqp", ...
        "ConstraintTolerance", 1e-9, "OptimalityTolerance", 1e-8, "MaxIterations", 100, ...
        "StepTolerance",1e-12,"SpecifyObjectiveGradient",true,"SpecifyConstraintGradient",true);
    equalities = 1:program.cones(1);
    rows = program.cones(1)+(1:program.cones(2));
    [decision, ~, exitFlag, output] = fmincon(@(z) localQuadraticCost(z,hessian,program.q), ...
        initial.decision, full(program.A(rows,:)), program.b(rows), ...
        full(program.A(equalities,:)),program.b(equalities),[],[], ...
        @(z) localCones(program,z), options);
    result = struct("decision",decision,"exitFlag",exitFlag,"output",output);
end

function [value,gradient] = localQuadraticCost(decision,hessian,linear)
    value = 0.5*decision.'*hessian*decision+linear.'*decision;
    gradient = hessian*decision+linear;
end

function [inequality,equality,gradient,equalityGradient] = localCones(program,decision)
    slack = program.b-program.A*decision;
    dimensions = program.cones(3:end);
    inequality = zeros(numel(dimensions),1);
    equality = [];
    gradient = zeros(numel(decision),numel(dimensions));
    offset = sum(program.cones(1:2));
    for index = 1:numel(dimensions)
        rows = offset+(1:dimensions(index));
        vector = slack(rows(2:end));
        inequality(index) = norm(vector)-slack(rows(1));
        gradient(:,index) = program.A(rows(1),:).' ...
            -program.A(rows(2:end),:).'*(vector/max(norm(vector),realmin));
        offset = offset+dimensions(index);
    end
    equalityGradient = [];
end

function value = localExplicitCost(problem,cfg,decision)
    input = reshape(decision(problem.layout.planIndex),2,[]);
    certificateSpeed = max(cfg.referenceSpeed,cfg.clf.certificateSpeedFloor);
    operatingInput = [0;ltvBicycleModel.roadLoad(certificateSpeed,cfg) ...
        /(cfg.vehicle.m*modifiedFialaTire.accelerationGain(cfg))];
    states = reshape(pagemtimes(problem.prediction.egoStateMatrix, input(:)),6,[]) ...
        +problem.prediction.egoStateOffset;
    error = states(2:6,1:end-1)-problem.qp.clf.referenceStart ...
        -problem.qp.clf.referenceRate*((0:size(input,2)-1)*cfg.controller.sampleTime);
    scales = [cfg.clf.lateralPositionErrorScale;cfg.clf.headingErrorScale;cfg.clf.speedErrorScale; ...
        cfg.clf.lateralVelocityErrorScale;cfg.clf.yawRateErrorScale];
    changes = diff([problem.model.previousInput,input],1,2)/cfg.controller.sampleTime;
    inputWeights = [cfg.clf.frontWheelSteeringAngleWeight;cfg.clf.brakingRatioWeight];
    value = cfg.controller.sampleTime*(sum((error./scales).^2,"all") ...
        +sum(inputWeights.*(input-operatingInput).^2,"all") ...
        +cfg.encounter.inputRateWeight*sum(changes.^2,"all") ...
        +cfg.clf.relaxationWeight*sum(decision(problem.layout.relaxationIndex).^2));
end
