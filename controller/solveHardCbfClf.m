function result = solveHardCbfClf(problem, cfg)
    result = localEmptyResult();
    if problem.certifiedInfeasible
        result.message = "A constant hard constraint is infeasible.";
        result.exitFlag = -2;
        return;
    end
    solve = localRunJointProgram(problem, cfg);
    result.solverCalls = 1;
    result.exitFlag = solve.exitFlag;
    result.message = solve.message;
    if ~solve.feasible, return; end
    decision = solve.decision;
    % Recompute only unbounded CLF slacks; preserve all controls verbatim.
    slacks = zeros(problem.layout.relaxationCount, 1);
    for index = 1:numel(problem.clf.constraints)
        constraint = problem.clf.constraints(index);
        value = constraint.map*decision+constraint.offset;
        residual = norm(constraint.root*value)^2+constraint.linear.'*value+constraint.constant;
        residual = residual+16*(numel(decision)+64)*eps*( ...
            norm(abs(constraint.root)*abs(value))^2+abs(constraint.linear).'*abs(value)+abs(constraint.constant));
        slacks(constraint.stage) = max(slacks(constraint.stage), residual);
    end
    decision(problem.layout.relaxationIndex) = slacks ...
        + cfg.encounter.numericalMargin*(1+abs(slacks));
    result.decision = decision;
    result.feasible = all(isfinite(decision));
    result.iterations = localIterationCount(solve.output);
    result.algorithm = "Clarabel predictive CBF-CLF SOCP";
    result.objectiveValue = localJointValue(problem, decision);
    result.clfValue = slacks;
end


function solve = localRunJointProgram(problem, cfg)
    hook = cfg.solver.jointFunction;
    try
        if isempty(hook)
            solve = localDefaultSolve(problem, cfg);
        else
            % The hook can solve the conic program or inject a failure.
            program = problem.stageProgram;
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
    output.algorithm = "Clarabel predictive CBF-CLF SOCP";
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
        "algorithm", "predictive CBF-CLF SOCP", ...
        "message", "", ...
        "objectiveValue", inf, ...
        "clfValue", inf);
end
