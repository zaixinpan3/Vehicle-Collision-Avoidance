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
% native backend retains the quadratic input cost directly and uses explicit
% stage states with sparse dynamics. The public fault-injection hook retains
% its equivalent condensed epigraph interface. The slack penalty is linear.

    layout = problem.layout;
    result = localEmptyResult();
    if problem.certifiedInfeasible
        result.exitFlag = -2;
        result.message = "hard CBF, terminal, or physical constraints " ...
            + "are infeasible";
        return;
    end
    jointSolve = localRunJointProgram(problem, cfg);
    result.solverCalls = 1;
    if ~jointSolve.feasible
        result.exitFlag = jointSolve.exitFlag;
        result.message = "joint CLF/input solve failed: " ...
            + jointSolve.message;
        return;
    end

    decision = jointSolve.decision(1:layout.decisionCount);
    % The sole slack has an analytic minimizer once the input plan is fixed.
    % Reconstruct it exactly instead of rejecting a safe input because the
    % cone solver's epigraph variable differs by its stopping tolerance.
    % No actuator or hard-safety variable is changed by this operation.
    clf = problem.clf;
    plan = decision(layout.planIndex);
    initialError = clf.errorOffset(:, 1);
    nextError = clf.errorMatrix(:, :, 2)*plan+clf.errorOffset(:, 2);
    decision(layout.relaxationIndex) = max(0.0, ...
        nextError.'*clf.lyapunovMatrix*nextError ...
        - initialError.'*(clf.lyapunovMatrix-clf.decreaseMatrix)*initialError);
    jointValue = localJointValue(problem, decision);
    result.decision = decision;
    result.exitFlag = jointSolve.exitFlag;
    result.feasible = true;
    result.iterations = localIterationCount(jointSolve.output);
    result.algorithm = "joint predictive SOCP";
    if isfield(jointSolve.output, "algorithm")
        result.algorithm = string(jointSolve.output.algorithm);
    end
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

function solve = localRunJointProgram(problem, cfg)
    hook = cfg.solver.jointFunction;
    try
        if isempty(hook)
            solve = localDefaultSolve(problem, cfg);
        else
            % Retain the public solver-hook format for fault injection and
            % independent backend comparisons. The online path does not
            % construct this condensed objective epigraph.
            program = localJointProgram(problem);
            program.defaultSolver = @() localDefaultSolve(problem, cfg);
            solve = hook("joint", program);
        end
    catch exception
        solve = localEmptySolve();
        solve.exitFlag = -999;
        solve.output = struct("message", string(exception.message));
    end
    solve = localNormalizeSolve(solve, problem.layout.decisionCount+1);
end

function solve = localDefaultSolve(problem, cfg)
    persistent nativeSolver
    if isempty(nativeSolver)
        if exist("solveAvoidanceSocpMex", "file") ~= 3
            solverPath = fullfile(fileparts(fileparts(mfilename("fullpath"))), ...
                "solver", "clarabel", "matlab");
            if isfolder(solverPath)
                addpath(solverPath);
            end
            if exist("solveAvoidanceSocpMex", "file") ~= 3
                error("collisionAvoidanceController:missingSocpSolver", ...
                    "Build the native solver once using scripts/buildAvoidanceSocpSolver.m.");
            end
        end
        nativeSolver = @solveAvoidanceSocpMex;
    end
    program = problem.stageProgram;
    [stageDecision, output] = nativeSolver( ...
        program.P, program.q, program.A, program.b, program.cones, ...
        [min(1.0e-9, cfg.solver.constraintTolerance), ...
            min(1.0e-9, cfg.solver.optimalityTolerance), cfg.solver.maxIterations]);
    flag = -7;
    switch output.status
        case 1
            flag = 1;
        case 4
            flag = 2;
        case {2, 5}
            flag = -2;
        case {3, 6}
            flag = -3;
        case {7, 8}
            flag = 0;
    end
    physical = stageDecision(1:program.physicalDecisionCount);
    plan = physical(problem.layout.planIndex);
    inputValue = max(0.0, 0.5*plan.'*problem.Hessian( ...
        problem.layout.planIndex, problem.layout.planIndex)*plan ...
        + problem.linear(problem.layout.planIndex).'*plan+problem.constant);
    output.algorithm = "Clarabel sparse predictive SOCP";
    output.message = "Clarabel status "+string(output.status);
    solve = struct("decision", [physical; inputValue], ...
        "exitFlag", flag, "output", output);
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
        "algorithm", "joint predictive SOCP", ...
        "message", "", ...
        "objectiveValue", inf, ...
        "clfValue", inf);
end
