function result = solveHardCbfClf(problem, cfg)
% solveHardCbfClf Minimize input effort and squared CLF relaxation.
%
% The certified decision assembled by formulateAvoidanceProblem is
%
%   z = [plan; delta],
%
% where every collision, road, physical, backup-tail, and terminal row is
% hard. delta relaxes only LfV + LgV*u_0 <= -alpha*V + delta.
% One quadratic program minimizes normalized head input effort plus w*delta^2
% under affine hard constraints, without a desired input or a tail input cost.
% The native backend uses explicit stage states with sparse dynamics;
% the optional solver hook receives the equivalent condensed QP.

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
        result.message = "CLF relaxation solve failed: " ...
            + jointSolve.message;
        return;
    end

    decision = jointSolve.decision(1:layout.decisionCount);
    % Reconstruct the analytic minimum slack for this fixed input plan.
    % No actuator or hard-safety variable is changed by this operation.
    clf = problem.clf;
    currentInput = decision(1:layout.inputDimension);
    decision(layout.relaxationIndex) = max(0.0, ...
        clf.lieDerivativeDrift+clf.lieDerivativeInput*currentInput ...
        + clf.decayRate*clf.initialValue);
    jointValue = localJointValue(problem, decision);
    result.decision = decision;
    result.exitFlag = jointSolve.exitFlag;
    result.feasible = true;
    result.iterations = localIterationCount(jointSolve.output);
    result.algorithm = "joint CBF-CLF-QP";
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
% Standard condensed QP: min 0.5*z'*H*z + f'*z, subject to affine rows.
    program = struct("H", problem.Hessian, "f", problem.linear, ...
        "constant", problem.constant, ...
        "A", [problem.inequalityMatrix; problem.clf.inequalityMatrix], ...
        "b", [problem.inequalityBound; problem.clf.inequalityBound], ...
        "Aeq", problem.equalityMatrix, "beq", problem.equalityBound, ...
        "lb", problem.lowerBound, "ub", problem.upperBound);
end

function solve = localRunJointProgram(problem, cfg)
    hook = cfg.solver.jointFunction;
    try
        if isempty(hook)
            solve = localDefaultSolve(problem, cfg);
        else
            % The hook can solve the condensed QP or inject a failure.
            program = localJointProgram(problem);
            program.defaultSolver = @() localDefaultSolve(problem, cfg);
            solve = hook("joint", program);
        end
    catch exception
        solve = localEmptySolve();
        solve.exitFlag = -999;
        solve.output = struct("message", string(exception.message));
    end
    solve = localNormalizeSolve(solve, problem.layout.decisionCount);
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
        [cfg.solver.constraintTolerance, ...
            cfg.solver.optimalityTolerance, cfg.solver.maxIterations]);
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
    output.algorithm = "Clarabel sparse CBF-CLF-QP";
    output.message = "Clarabel status "+string(output.status);
    solve = struct("decision", physical, ...
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
        "algorithm", "joint CBF-CLF-QP", ...
        "message", "", ...
        "objectiveValue", inf, ...
        "clfValue", inf);
end
