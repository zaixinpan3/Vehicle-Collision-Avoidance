function [result,problem] = solveHardCbfClf(problem, cfg)
    result = localEmptyResult();
    if problem.certifiedInfeasible
        result.message = "A constant hard constraint is infeasible.";
        result.exitFlag = -2;
        return;
    end
    solveCalls = 0;
    solve = [];
    if string(cfg.encounter.safetyMarginPolicy)=="maximize" ...
            && any(startsWith(problem.geometry.label,"collision:"))
        % The reserve problem is linear because CLF slacks are unbounded.
        % Allocate it directly, avoiding a speculative infeasible SOCP.
        [problem,marginSolve] = localReserveMargin(problem,cfg);
        solveCalls = 1;
        if ~marginSolve.feasible
            result.solverCalls = solveCalls;result.exitFlag = marginSolve.exitFlag;
            result.message = "margin optimization: "+marginSolve.message;
            return;
        end
    end
    if isempty(solve)
        solve = localRunJointProgram(problem, cfg);
        solveCalls = solveCalls+1;
    end
    result.solverCalls = solveCalls;
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

function [problem,solve] = localReserveMargin(problem,cfg)
% Maximize the achievable fraction of the additional lookahead reserve.
% Physical safety rows remain hard even when the desired reserve cannot fit.
% This phase supplies a margin target only; it never supplies an input for
% execution. The second phase and the independent checker retain authority.
    original = problem.stageProgram;
    physical = problem.layout.decisionCount;
    equalityCount = original.cones(1);
    inequalityIndices = localInequalityIndices(problem);
    hardCount = numel(inequalityIndices);
    selected = 1:equalityCount+hardCount;
    marginColumn = [zeros(equalityCount,1);problem.anticipationReserve(inequalityIndices)];
    matrix = [original.A(selected,1:physical),sparse(marginColumn),original.A(selected,physical+1:end)];
    total = size(matrix,2);
    extra = sparse(2,total);extra(:,physical+1) = [1;-1];
    linear = zeros(total,1);linear(physical+1) = -1;
    program = struct("P",sparse(total,total),"q",linear,"A",[matrix;extra], ...
        "b",[original.b(selected);problem.reserveFractionMaximum;0], ...
        "cones",[equalityCount;hardCount+2],"physicalDecisionCount",physical+1, ...
        "inactiveSlackIndex",problem.layout.relaxationIndex);
    auxiliary = struct("layout",struct("decisionCount",physical+1),"stageProgram",program);
    solve = localRunJointProgram(auxiliary,cfg);
    if ~solve.feasible,return;end
    margins = problem.inequalityBound-problem.inequalityMatrix*solve.decision(1:physical);
    selectedReserve = problem.anticipationReserve>0;
    fraction = min([solve.decision(physical+1);margins(selectedReserve)./problem.anticipationReserve(selectedReserve);problem.reserveFractionMaximum]);
    % Leave an interior allocation for the subsequent nonlinear refinement.
    % The physical constraints do not change. Maximizing an optional buffer
    % to its exact feasibility frontier made the performance solve fragile.
    if fraction>=problem.reserveFractionMaximum-10*cfg.encounter.numericalMargin
        fraction = problem.reserveFractionMaximum;
    else
        fraction = 0.99*fraction;
    end
    fraction = max(0,fraction-10*cfg.encounter.numericalMargin);
    problem = localApplyReserve(problem,fraction);
end

function problem = localApplyReserve(problem,fraction)
    reserve = fraction*problem.anticipationReserve;
    problem.reserveFraction = fraction;
    problem.inequalityBound = problem.inequalityBound-reserve;
    inequalityIndices = localInequalityIndices(problem);
    rows = problem.stageProgram.cones(1)+(1:numel(inequalityIndices));
    problem.stageProgram.b(rows) = problem.stageProgram.b(rows)-reserve(inequalityIndices);
end

function indices = localInequalityIndices(problem)
    indices = (1:numel(problem.inequalityBound)).';
    if isfield(problem.stageProgram,"inequalityIndices")
        indices = problem.stageProgram.inequalityIndices;
    end
end


function solve = localRunJointProgram(problem, cfg)
    hook = cfg.solver.jointFunction;
    try
        if isempty(hook)
            solve = localDefaultSolve(problem, cfg);
        else
            % The hook solves the full conic program, including auxiliary
            % states. Only physical decisions survive independent checking.
            program = problem.stageProgram;
            program.defaultSolver = @() localDefaultSolve(problem, cfg);
            solve = hook("joint", program);
        end
    catch exception
        solve = localEmptySolve();
        solve.exitFlag = -999;
        solve.output = struct("message", string(exception.message));
    end
    if isstruct(solve) && isscalar(solve) && isfield(solve,"decision") ...
            && numel(solve.decision)==numel(problem.stageProgram.q) ...
            && all(isfinite(solve.decision),"all")
        solve.decision = solve.decision(1:problem.layout.decisionCount);
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
    retained = true(numel(program.q),1);
    if isfield(program,"inactiveSlackIndex"),retained(program.inactiveSlackIndex) = false;end
    [nativeDecision, output] = nativeSolver( ...
        program.P(retained,retained), program.q(retained), program.A(:,retained), program.b, program.cones, ...
        [cfg.solver.constraintTolerance, ...
            cfg.solver.optimalityTolerance, cfg.solver.maxIterations]);
    stageDecision = zeros(numel(program.q),1);stageDecision(retained) = nativeDecision;
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
    output.algorithm = "Clarabel predictive CBF-CLF SOCP";
    output.message = "Clarabel status "+string(output.status);
    solve = struct("decision", stageDecision, ...
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
