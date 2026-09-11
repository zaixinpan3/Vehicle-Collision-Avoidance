function [result,problem] = solveHardCbfClf(problem, cfg)
    result = localEmptyResult();
    if problem.certifiedInfeasible
        result.message = "A constant hard constraint is infeasible.";
        result.exitFlag = -2;
        return;
    end
    [result, problem] = localHardMarginSolve(problem, cfg);
end

function [result, problem] = localHardMarginSolve(problem, cfg)
% Maximize a nonnegative tightening of every physical hard row, then track.
% The checked LP control is retained if the subordinate SOCP fails.
    physical = problem.layout.decisionCount;
    scale = problem.barrier.scale;
    base = problem.stageProgram;
    rows = [base.rowMap.equality;base.rowMap.inequality];
    marginScale = [zeros(numel(base.rowMap.equality),1);scale(base.inequalityIndices)];
    variables = numel(base.q)+1;
    matrix = [base.A(rows,:),sparse(marginScale)];
    extra = sparse(2,variables);extra(:,end) = [1;-1];
    program = struct("P",sparse(variables,variables), ...
        "q",[zeros(variables-1,1);-1],"A",[matrix;extra], ...
        "b",[base.b(rows)+problem.requiredMargin*marginScale; ...
            cfg.encounter.maximumCarriedMargin;-problem.requiredMargin], ...
        "cones",[numel(base.rowMap.equality);numel(base.rowMap.inequality)+2], ...
        "physicalDecisionCount",physical,"inactiveSlackIndex",problem.layout.relaxationIndex);
    if isfield(base,"fixedDecisionIndex")
        program.fixedDecisionIndex = base.fixedDecisionIndex;
        program.fixedDecisionValue = base.fixedDecisionValue;
    end
    auxiliary = struct("layout",struct("decisionCount",physical),"stageProgram",program);
    marginSolve = localRunJointProgram(auxiliary, cfg);
    result = localEmptyResult();
    result.solverCalls = 1;
    result.exitFlag = marginSolve.exitFlag;
    result.message = "hard margin: "+marginSolve.message;
    if ~marginSolve.feasible, return; end
    candidate = localRepairClf(problem, marginSolve.decision(1:physical), cfg);
    model = struct("cfg", cfg);
    check = certifyAvoidancePlan(problem, [], model, candidate);
    if ~check.accepted, return; end
    result = localCertifiedResult(problem, candidate, marginSolve, 1);
    % Give the performance solve an interior target without spending the
    % inherited certificate margin. This remains a lower bound, not H_N.
    problem.requiredMargin = max(problem.requiredMargin, 0.99*check.margin);
    problem.inequalityBound = problem.barrier.baseBound-problem.requiredMargin*scale;
    fixedProgram = problem.stageProgram;
    problem.stageProgram = updateAvoidanceStageBounds(problem);
    if isfield(fixedProgram, "fixedDecisionIndex")
        problem.stageProgram.fixedDecisionIndex = fixedProgram.fixedDecisionIndex;
        problem.stageProgram.fixedDecisionValue = fixedProgram.fixedDecisionValue;
    end
    solve = localRunJointProgram(problem, cfg);
    result.solverCalls = 2;
    if ~solve.feasible, return; end
    decision = localRepairClf(problem, solve.decision, cfg);
    check = certifyAvoidancePlan(problem, [], model, decision);
    if check.accepted
        result = localCertifiedResult(problem, decision, solve, 2);
    end
end

function [decision, slacks] = localRepairClf(problem, decision, cfg)
% Only performance slacks may be repaired; inputs are preserved verbatim.
    slacks = zeros(problem.layout.relaxationCount, 1);
    for index = 1:numel(problem.clf.constraints)
        constraint = problem.clf.constraints(index);
        value = constraint.map*decision+constraint.offset;
        residual = norm(constraint.root*value)^2+constraint.linear.'*value+constraint.constant;
        residual = residual+16*(numel(decision)+64)*eps*( ...
            norm(abs(constraint.root)*abs(value))^2+abs(constraint.linear).'*abs(value)+abs(constraint.constant));
        slacks(constraint.stage) = max(slacks(constraint.stage), residual);
    end
    decision(problem.layout.relaxationIndex) = slacks+cfg.encounter.numericalMargin*(1+abs(slacks));
end

function result = localCertifiedResult(problem, decision, solve, calls)
    result = localEmptyResult();
    result.feasible = true;
    result.decision = decision;
    result.exitFlag = solve.exitFlag;
    result.message = solve.message;
    result.solverCalls = calls;
    result.iterations = localIterationCount(solve.output);
    result.objectiveValue = localJointValue(problem, decision);
    result.clfValue = decision(problem.layout.relaxationIndex);
    result.algorithm = "hard predictive margin LP and CLF SOCP";
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
    fixed = zeros(0,1);value = zeros(0,1);
    if isfield(program,"fixedDecisionIndex")
        fixed = program.fixedDecisionIndex;value = program.fixedDecisionValue;
        retained(fixed) = false;
    end
    linear = program.q(retained) ...
        +(program.P(retained,fixed)+program.P(fixed,retained).')*value;
    bound = program.b-program.A(:,fixed)*value;
    % Positive objective scaling preserves minimizers. The lifted CLF
    % epigraph can otherwise trigger a false native infeasibility report.
    hessian = program.P(retained,retained);
    objectiveScale = 1/max([1;abs(linear);abs(nonzeros(hessian))]);
    [nativeDecision, output] = nativeSolver( ...
        objectiveScale*hessian, objectiveScale*linear, program.A(:,retained), bound, program.cones, ...
        [cfg.solver.constraintTolerance, ...
            cfg.solver.optimalityTolerance*objectiveScale, cfg.solver.maxIterations]);
    output.objectiveValue = output.objectiveValue/objectiveScale;
    output.objectiveScale = objectiveScale;
    stageDecision = zeros(numel(program.q),1);stageDecision(retained) = nativeDecision;
    stageDecision(fixed) = value;
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
