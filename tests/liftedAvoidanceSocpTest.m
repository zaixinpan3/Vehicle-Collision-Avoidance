classdef liftedAvoidanceSocpTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            for folder = ["controller","config","tests"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,folder)));
            end
        end
    end
    methods (Test)
        function screenedConesPreserveTheRequiredClfSlack(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            cfg.model.frontWheelSteeringRateMaximum = 0.5;
            cfg.model.brakingRatioRateMaximum = 2;
            [~,~,problem] = collisionAvoidanceController(ego,target,route,cfg,[]);
            problem.model.anchorPlan = problem.prediction.referencePlan;
            problem.qp.stageProgram = avoidanceStageQp(problem.qp,problem.prediction,problem.model);
            discrepancy = localClfScreeningError(problem,cfg);
            testCase.verifyLessThanOrEqual(discrepancy,1e-8);
            testCase.verifyLessThanOrEqual(numel(problem.qp.stageProgram.clfConstraintIndices),numel(problem.qp.clf.constraints));
        end
        function routeStationOriginDoesNotChangeTheAvoidanceInput(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            [near,~,nearProblem] = collisionAvoidanceController(ego,target,route,cfg,[]);
            route(1,1) = route(1,1)-10000;
            [far,~,farProblem] = collisionAvoidanceController(ego,target,route,cfg,[]);
            testCase.verifyTrue(nearProblem.metadata.planCertified);
            testCase.verifyTrue(farProblem.metadata.planCertified);
            testCase.verifyEqual(far.actuatorInput,near.actuatorInput,AbsTol=1e-5);
        end
        function sparseAndCondensedProgramsDescribeTheSameDecisions(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            cfg.referenceSpeed = 10;
            cfg.model.frontWheelSteeringRateMaximum = 1;
            [~,~,problem] = collisionAvoidanceController(ego,target,route,cfg,[]);
            problem.model.anchorPlan = problem.prediction.referencePlan;
            problem.qp.stageProgram = avoidanceStageQp(problem.qp,problem.prediction,problem.model);
            sparseProgram = problem.qp.stageProgram;
            condensed = avoidanceStageQp(problem.qp);
            first = problem.decision;
            second = first;
            second(1:problem.layout.planCount) = second(1:problem.layout.planCount) ...
                +0.001*sin((1:problem.layout.planCount).');
            second(2:2:problem.layout.planCount) = second(2:2:problem.layout.planCount)+0.02;
            firstLift = localLift(first,problem);
            secondLift = localLift(second,problem);
            equalities = sparseProgram.cones(1);
            for decision = [firstLift,secondLift]
                residual = sparseProgram.A*decision-sparseProgram.b;
                expected = condensed.A*decision(1:sparseProgram.physicalDecisionCount)-condensed.b;
                testCase.verifyEqual(residual(1:equalities),zeros(equalities,1),AbsTol=1e-9);
                hard = sparseProgram.cones(2);
                testCase.verifyEqual(residual(equalities+(1:hard)),expected(sparseProgram.inequalityIndices),AbsTol=1e-8);
                testCase.verifyLessThanOrEqual(localClfEncodingError(sparseProgram,decision,problem.qp),1e-6);
            end
            newDifference = localObjective(sparseProgram,secondLift)-localObjective(sparseProgram,firstLift);
            oldDifference = localObjective(condensed,second)-localObjective(condensed,first);
            testCase.verifyEqual(newDifference,oldDifference,AbsTol=1e-7);
        end
        function completeProgramKeepsSlipAndGeometryWithoutForcePolygons(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            [~,~,problem] = collisionAvoidanceController(ego,target,route,cfg,[]);
            testCase.verifyFalse(any(problem.qp.geometry.label=="combinedTireForce"));
            testCase.verifyTrue(any(problem.qp.geometry.label=="tireSlip"));
            testCase.verifyTrue(any(startsWith(problem.qp.geometry.label,"collision:")));
            testCase.verifyTrue(problem.metadata.planCertified);
        end

    end
end

function error = localClfScreeningError(problem,cfg)
    lower = max(problem.qp.lowerBound(1:2),problem.model.previousInput ...
        -cfg.controller.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]);
    upper = min(problem.qp.upperBound(1:2),problem.model.previousInput ...
        +cfg.controller.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]);
    [first,second] = ndgrid(linspace(lower(1),upper(1),7),linspace(lower(2),upper(2),7));
    inputs = [first(:),second(:)].';error = 0;
    for input = inputs
        decision = problem.decision;decision(1:2) = input;
        residual = zeros(numel(problem.qp.clf.constraints),1);
        for index = 1:numel(residual)
            constraint = problem.qp.clf.constraints(index);
            value = constraint.map*decision+constraint.offset;
            residual(index) = norm(constraint.root*value)^2+constraint.linear.'*value+constraint.constant;
        end
        error = max(error,abs(max(residual)-max(residual(problem.qp.stageProgram.clfConstraintIndices))));
    end
end

function error = localClfEncodingError(program,decision,qp)
    slack = program.b-program.A*decision;
    cursor = sum(program.cones(1:2));
    error = 0;
    for index = 1:numel(program.clfConstraintIndices)
        dimension = program.cones(index+2);
        cone = slack(cursor+(1:dimension));
        constraint = qp.clf.constraints(program.clfConstraintIndices(index));
        physical = decision(1:qp.layout.decisionCount);
        value = constraint.map*physical+constraint.offset;
        expected = physical(qp.layout.relaxationIndex(constraint.stage)) ...
            -norm(constraint.root*value)^2-constraint.linear.'*value-constraint.constant;
        actual = (cone(1)^2-sum(cone(2:end).^2))/4;
        error = max(error,abs(actual-expected));
        cursor = cursor+dimension;
    end
end

function decision = localLift(physical,problem)
    program = problem.qp.stageProgram;
    decision = zeros(size(program.A,2),1);
    decision(1:numel(physical)) = physical;
    plan = physical(problem.layout.planIndex);
    decision(program.stateIndex(:,1)) = problem.model.initialEgoState-program.stateCenter(:,1);
    for index = 1:numel(problem.prediction.cells)
        tube = problem.prediction.cells(index);
        decision(program.stateIndex(:,index+1)) = tube.endMap*plan+tube.endOffset-program.stateCenter(:,index+1);
    end
end

function value = localObjective(program,decision)
    matrix = program.P+triu(program.P,1).';
    value = 0.5*decision.'*matrix*decision+program.q.'*decision;
end
