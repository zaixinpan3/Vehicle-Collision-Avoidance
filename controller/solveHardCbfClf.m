function result = solveHardCbfClf(problem, cfg)
% solveHardCbfClf Solve hard CBF constraints with a joint CLF/input cost.
%
% The certified decision assembled by formulateAvoidanceProblem is
%
%   z = [plan; delta],
%
% where every collision, road, physical, backup-tail, and terminal row is
% hard. delta is the only active relaxation and belongs exclusively to
% the exact first-step discrete CLF inequality. This routine performs one
% conic solve of
%
%   min  J_input(plan) + w delta
%
% subject to those hard rows and the exact quadratic CLF condition. The
% input quadratic is represented by its exact squared-norm epigraph. The
% CLF relaxation has only the configured linear penalty.

    layout = problem.layout;
    result = localEmptyResult();
    if problem.certifiedInfeasible
        result.exitFlag = -2;
        result.message = "hard CBF, terminal, or physical constraints " ...
            + "are infeasible";
        return;
    end
    program = localJointProgram(problem);
    jointSolve = localRunJointProgram(program, cfg);
    result.solverCalls = 1;
    if ~jointSolve.feasible
        result.exitFlag = jointSolve.exitFlag;
        result.message = "joint CLF/input solve failed: " ...
            + jointSolve.message;
        return;
    end

    decision = jointSolve.decision(1:layout.decisionCount);
    jointValue = localJointValue(problem, decision);
    result.decision = decision;
    result.exitFlag = jointSolve.exitFlag;
    result.feasible = true;
    result.iterations = localIterationCount(jointSolve.output);
    result.algorithm = "coneprog hard-CBF joint CLF/input";
    result.message = "candidate returned for independent acceptance: " ...
        + jointSolve.message;
    result.objectiveValue = jointValue;
    result.clfValue = max( ...
        decision(layout.relaxationIndex(1)), 0.0);
end

function program = localJointProgram(problem)
    layout = problem.layout;
    decisionCount = layout.decisionCount;
    planIndex = layout.planIndex;
    planHessian = 0.5*(problem.Hessian(planIndex, planIndex) ...
        + problem.Hessian(planIndex, planIndex).');
    planLinear = problem.linear(planIndex);
    [factor, factorFlag] = chol(planHessian);
    if factorFlag ~= 0
        error("collisionAvoidanceController:invalidProblem", ...
            "The plan/input Hessian must be positive definite.");
    end
    centre = -planHessian\planLinear;

    % t >= 0.5*||R*(plan-centre)||^2 is exactly
    % ||[sqrt(2)*R*(plan-centre); t-1]|| <= t+1.
    augmentedCount = decisionCount+1;
    quadraticMatrix = zeros(size(factor, 1)+1, augmentedCount);
    quadraticMatrix(1:end-1, planIndex) = sqrt(2.0)*factor;
    quadraticMatrix(end, end) = 1.0;
    quadraticCentre = [sqrt(2.0)*factor*centre; 1.0];
    quadraticBound = zeros(augmentedCount, 1);
    quadraticBound(end) = 1.0;
    quadraticCone = secondordercone( ...
        quadraticMatrix, quadraticCentre, quadraticBound, -1.0);

    exactClf = localExactClfCone(problem);
    clfCone = secondordercone( ...
        [exactClf.A, zeros(size(exactClf.A, 1), 1)], ...
        exactClf.b, [exactClf.d; 0.0], exactClf.gamma);

    program = struct();
    program.f = [problem.linear; 0.0];
    program.f(planIndex) = 0.0;
    program.f(end) = 1.0;
    program.cones = {clfCone, quadraticCone};
    program.A = [problem.inequalityMatrix, ...
        zeros(size(problem.inequalityMatrix, 1), 1)];
    program.b = problem.inequalityBound;
    program.Aeq = [problem.equalityMatrix, ...
        zeros(size(problem.equalityMatrix, 1), 1)];
    program.beq = problem.equalityBound;
    program.lb = [problem.lowerBound; 0.0];
    program.ub = [problem.upperBound; inf];

end

function cone = localExactClfCone(problem)
% V(e_1) <= V(e_0) - W(e_0) + delta as a rotated SOC:
%
%   ||[2 R e_1; t - 1]||_2 <= t + 1,
%   t = V(e_0) - W(e_0) + delta,  P = R'R.
    layout = problem.layout;
    clf = problem.clf;
    decisionCount = layout.decisionCount;
    lyapunovFactor = chol(clf.lyapunovMatrix);
    nextMatrix = zeros(size(clf.errorMatrix, 1), decisionCount);
    nextMatrix(:, layout.planIndex) = clf.errorMatrix(:, :, 2);
    nextOffset = clf.errorOffset(:, 2);
    initialError = clf.errorOffset(:, 1);
    availableDecrease = initialError.'*clf.lyapunovMatrix*initialError ...
        - initialError.'*clf.decreaseMatrix*initialError;
    relaxationSelector = zeros(decisionCount, 1);
    relaxationSelector(layout.relaxationIndex(1)) = 1.0;

    matrix = [2.0*lyapunovFactor*nextMatrix; relaxationSelector.'];
    centre = [-2.0*lyapunovFactor*nextOffset; 1.0-availableDecrease];
    bound = relaxationSelector;
    gamma = -(availableDecrease+1.0);
    cone = secondordercone(matrix, centre, bound, gamma);
end

function solve = localRunJointProgram(program, cfg)
    hook = cfg.solver.jointFunction;
    if isempty(hook)
        solve = localDefaultSolve(program, cfg);
        return;
    end
    program.defaultSolver = @() localDefaultSolve(program, cfg);
    try
        solve = hook("joint", program);
    catch exception
        solve = localEmptySolve();
        solve.exitFlag = -999;
        solve.message = string(exception.message);
    end
    solve = localNormalizeSolve(solve, numel(program.f));
end

function solve = localDefaultSolve(program, cfg)
    options = optimoptions("coneprog", "Display", "none", ...
        "ConstraintTolerance", cfg.solver.constraintTolerance, ...
        "OptimalityTolerance", cfg.solver.optimalityTolerance, ...
        "MaxIterations", cfg.solver.maxIterations);
    [decision, ~, exitFlag, output] = coneprog( ...
        program.f, program.cones, program.A, program.b, ...
        program.Aeq, program.beq, program.lb, program.ub, options);
    solve = struct("decision", decision, ...
        "exitFlag", exitFlag, "output", output);
    solve = localNormalizeSolve(solve, numel(program.f));
end

function solve = localNormalizeSolve(solve, decisionCount)
    if ~isstruct(solve) || ~isscalar(solve)
        solve = localEmptySolve();
        solve.message = "solver hook returned no scalar result structure";
        return;
    end
    if ~isfield(solve, "decision"), solve.decision = zeros(0, 1); end
    if ~isfield(solve, "exitFlag"), solve.exitFlag = -999; end
    if ~isfield(solve, "output"), solve.output = struct(); end
    if ~isnumeric(solve.exitFlag) || ~isreal(solve.exitFlag) ...
            || ~isscalar(solve.exitFlag) || ~isfinite(solve.exitFlag)
        solve.exitFlag = -999;
    end
    solve.exitFlag = double(solve.exitFlag);
    % A feasible iterate can preserve safety after an iteration limit or
    % numerical termination. The caller verifies the complete decision.
    solve.feasible = any(solve.exitFlag == [1, 2, 0, -7]) ...
        && isnumeric(solve.decision) && isreal(solve.decision) ...
        && numel(solve.decision) == decisionCount ...
        && all(isfinite(solve.decision), "all");
    solve.decision = solve.decision(:);
    solve.message = "solver exit flag "+string(solve.exitFlag);
    if isstruct(solve.output) && isfield(solve.output, "message")
        solve.message = solve.message+": "+string(solve.output.message);
    end
end

function value = localJointValue(problem, decision)
    value = 0.5*decision.'*problem.Hessian*decision ...
        + problem.linear.'*decision+problem.constant;
end

function iterations = localIterationCount(output)
    iterations = 0;
    if isstruct(output) && isfield(output, "iterations") ...
            && isnumeric(output.iterations) && isscalar(output.iterations)
        iterations = double(output.iterations);
    end
end

function solve = localEmptySolve()
    solve = struct("decision", zeros(0, 1), ...
        "exitFlag", -999, "output", struct(), "feasible", false, ...
        "message", "");
end

function result = localEmptyResult()
    result = struct( ...
        "decision", zeros(0, 1), ...
        "exitFlag", -999, ...
        "feasible", false, ...
        "iterations", 0, ...
        "solverCalls", 0, ...
        "algorithm", "coneprog hard-CBF joint CLF/input", ...
        "message", "", ...
        "objectiveValue", inf, ...
        "clfValue", inf);
end
