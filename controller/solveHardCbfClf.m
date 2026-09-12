classdef solveHardCbfClf
    %solveHardCbfClf Lexicographic safety-value LP, CLF SOCP and independent verification.
    % Stage A minimizes the accumulated per-stage safety violation with every
    % non-safety row hard; its optimum is the plan's value. Stage B optimizes
    % the CLF performance cost among plans within that value. Verification
    % recomputes the violation of any decision from the physical rows.

    methods (Static)
        function [result,problem] = solve(problem, cfg)
            result = localEmptyResult();
            if problem.certifiedInfeasible
                result.message = "A constant hard constraint is infeasible.";
                result.exitFlag = -2;
                return;
            end
            [result, problem] = localLexicographicSolve(problem, cfg);
        end

        function names = rowNames(qp)
        % Human-readable name of every physical row, for diagnostics.
            planCount = qp.layout.planCount;
            count = qp.layout.horizonSteps;
            geometryRows = numel(qp.geometry.label);
            terminalRows = numel(qp.barrier.completionRows);
            slewRows = numel(qp.physicalBound)-geometryRows-2*planCount-count-terminalRows;
            names = [string(qp.geometry.label); repmat("inputUpperBound",planCount,1); ...
                repmat("inputLowerBound",planCount,1); repmat("clfSlackNonnegative",count,1); ...
                repmat("inputRate",slewRows,1); repmat("terminalSet",terminalRows,1)];
        end

        function [check,decision] = certifyInputs(qp,prediction,model,inputs)
        % Evaluate a mathematical input candidate in the given problem.
            decision = zeros(qp.layout.decisionCount,1);
            decision(qp.layout.planIndex) = inputs(:);
            decision = localRepairClf(qp,decision,model.cfg);
            check = solveHardCbfClf.certify(qp,prediction,model,decision);
        end

        function check = certify(qp, ~, model, decision)
        % Acceptance keeps every non-safety row hard and reports the verified
        % per-stage safety violation; its sum is the value function.
            stages = qp.layout.horizonSteps;
            check = struct("accepted", false, "failedConditions", "decision", ...
                "hardRowViolation", inf, "clfViolation", inf, "margin", -inf, ...
                "sweptClearanceMargin", -inf, "exitMargin", qp.exitMargin, ...
                "value", inf, "stageViolation", inf(stages,1), "violatedHardRows", zeros(0,1));
            if ~isnumeric(decision) || ~isreal(decision) || ~isvector(decision) ...
                    || numel(decision) ~= qp.layout.decisionCount || any(~isfinite(decision))
                return;
            end
            decision = decision(:);
            operations = numel(decision)+2;
            gamma = operations*eps/(1-operations*eps);
            evaluationAllowance = gamma*(abs(qp.physicalBound)+abs(qp.inequalityMatrix)*abs(decision));
            margins = qp.physicalBound-qp.inequalityMatrix*decision-evaluationAllowance;
            violation = max(0, -margins);
            relaxed = qp.safetyRows;
            check.hardRowViolation = max([0; violation(~relaxed)]);
            check.violatedHardRows = find(~relaxed & violation > 0);
            stageViolation = zeros(stages, 1);
            if any(relaxed)
                stageViolation = accumarray(qp.rowStage(relaxed), violation(relaxed), [stages, 1], @max, 0);
            end
            check.stageViolation = stageViolation;
            check.value = sum(stageViolation);
            allowance = model.cfg.encounter.numericalMargin;
            check.sweptClearanceMargin = min([inf; margins(relaxed)])-allowance;
            % Solver buffers are not additional physical obligations. Use
            % independently enclosed physical margins for acceptance and
            % keep every non-safety row hard, including the terminal rows.
            selected = qp.barrier.scale > 0;
            check.margin = min([model.cfg.encounter.maximumCarriedMargin; ...
                margins(selected)./qp.barrier.scale(selected)]);
            check.exitMargin = min([inf; margins(qp.barrier.completionRows)]);
            check.clfViolation = -inf;
            for index = 1:numel(qp.clf.constraints)
                constraint = qp.clf.constraints(index);
                value = constraint.map*decision+constraint.offset;
                residual = norm(constraint.root*value)^2+constraint.linear.'*value+constraint.constant;
                residual = residual+16*(numel(decision)+64)*eps*( ...
                    norm(abs(constraint.root)*abs(value))^2+abs(constraint.linear).'*abs(value)+abs(constraint.constant));
                check.clfViolation = max(check.clfViolation, ...
                    residual-decision(qp.layout.relaxationIndex(constraint.stage)));
            end
            conditions = [all(isfinite(margins)), check.hardRowViolation == 0, ...
                isfinite(check.clfViolation) && check.clfViolation <= 0, ...
                isfinite(check.value), ~qp.certifiedInfeasible];
            names = ["finitePrediction", "hardRows", "sampledDataClf", "finiteValue", "finiteProblem"];
            check.failedConditions = names(~conditions);
            check.accepted = all(conditions);
        end
    end
end

function [result, problem] = localLexicographicSolve(problem, cfg)
% Stage A decides the value: the margin LP with every violation fixed at
% zero is feasible exactly when a zero-violation plan exists, and its point
% supplies a numerical interior; otherwise the value LP minimizes the
% accumulated violation. Stage B optimizes performance within that value.
    physical = problem.layout.decisionCount;
    base = problem.stageProgram;
    scale = problem.barrier.scale;
    model = struct("cfg", cfg);
    result = localEmptyResult();
    rows = [base.rowMap.equality;base.rowMap.inequality;base.rowMap.violation];
    marginScale = [zeros(numel(base.rowMap.equality),1);scale(base.inequalityIndices); ...
        zeros(numel(base.rowMap.violation),1)];
    variables = numel(base.q)+1;
    matrix = [base.A(rows,:),sparse(marginScale)];
    extra = sparse(2,variables);extra(:,end) = [1;-1];
    marginProgram = struct("P",sparse(variables,variables), ...
        "q",[zeros(variables-1,1);-1],"A",[matrix;extra], ...
        "b",[base.b(rows)+problem.requiredMargin*marginScale; ...
            cfg.encounter.maximumCarriedMargin;-problem.requiredMargin], ...
        "cones",[numel(base.rowMap.equality);numel(rows)-numel(base.rowMap.equality)+2], ...
        "physicalDecisionCount",physical, ...
        "inactiveSlackIndex",[problem.layout.relaxationIndex(:).',base.violationIndex(:).']);
    auxiliary = struct("layout",struct("decisionCount",physical),"stageProgram",marginProgram);
    marginSolve = localRunJointProgram(auxiliary, cfg);
    result.solverCalls = 1;
    result.exitFlag = marginSolve.exitFlag;
    result.message = "zero-value margin: "+marginSolve.message;
    if marginSolve.feasible
        valueOptimum = 0;
        candidate = localRepairClf(problem, marginSolve.decision, cfg);
        check = solveHardCbfClf.certify(problem, [], model, candidate);
        if ~check.accepted || check.value > 0
            result.message = result.message+"; "+strjoin(check.failedConditions,",");
            return;
        end
        % Reserve a small numerical interior, not the maximized surplus.
        % Hard safety is already established at the zero level.
        selected = scale>0;
        available = min((problem.barrier.baseBound(selected) ...
            -problem.inequalityMatrix(selected,:)*candidate)./scale(selected));
        interiorMargin = max(problem.requiredMargin, ...
            min(10*cfg.encounter.numericalMargin,0.5*max(0,available)));
        inactive = base.violationIndex;
        budget = 0;
    else
        if ~ismember(marginSolve.exitFlag,[-2,-3])
            return;
        end
        linear = zeros(numel(base.q),1);
        linear(base.violationIndex) = 1;
        valueProgram = struct("P",sparse(numel(base.q),numel(base.q)),"q",linear, ...
            "A",base.A(rows,:),"b",base.b(rows), ...
            "cones",[numel(base.rowMap.equality);numel(rows)-numel(base.rowMap.equality)], ...
            "physicalDecisionCount",physical,"inactiveSlackIndex",problem.layout.relaxationIndex);
        auxiliary = struct("layout",struct("decisionCount",physical),"stageProgram",valueProgram);
        valueSolve = localRunJointProgram(auxiliary, cfg);
        result.solverCalls = 2;
        result.exitFlag = valueSolve.exitFlag;
        result.message = "safety value: "+valueSolve.message;
        if ~valueSolve.feasible, return; end
        valueOptimum = sum(max(0, valueSolve.fullDecision(base.violationIndex)));
        candidate = localRepairClf(problem, valueSolve.decision, cfg);
        check = solveHardCbfClf.certify(problem, [], model, candidate);
        if ~check.accepted
            result.message = result.message+"; "+strjoin(check.failedConditions,",");
            return;
        end
        % A positive value leaves no interior; the tie tolerance bounds the
        % performance stage's violation above the value optimum.
        interiorMargin = problem.requiredMargin;
        inactive = zeros(1, 0);
        budget = valueOptimum+cfg.solver.lexicographicTieTolerance;
    end
    result.valueStageOptimum = valueOptimum;
    problem.inequalityBound = problem.barrier.baseBound-interiorMargin*scale;
    problem.stageProgram = avoidanceStageQp.updateBounds(problem);
    problem.stageProgram.inactiveSlackIndex = inactive;
    problem.stageProgram.b(base.rowMap.budget) = budget;
    solve = localRunJointProgram(problem, cfg);
    calls = result.solverCalls+1;
    result.solverCalls = calls;
    result.exitFlag = solve.exitFlag;
    result.message = "performance: "+solve.message;
    if ~solve.feasible, return; end
    decision = localRepairClf(problem, solve.decision, cfg);
    check = solveHardCbfClf.certify(problem, [], model, decision);
    result.decision = decision;
    if check.accepted
        result = localCertifiedResult(problem, decision, solve, calls, valueOptimum, check);
    else
        result.message = result.message+"; "+strjoin(check.failedConditions,",") ...
            +"; hard violation "+check.hardRowViolation+"; value "+check.value;
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

function result = localCertifiedResult(problem, decision, solve, calls, valueOptimum, check)
    result = localEmptyResult();
    result.feasible = true;
    result.decision = decision;
    result.exitFlag = solve.exitFlag;
    result.message = solve.message;
    result.solverCalls = calls;
    result.iterations = localIterationCount(solve.output);
    result.objectiveValue = localJointValue(problem, decision);
    result.clfValue = decision(problem.layout.relaxationIndex);
    result.valueStageOptimum = valueOptimum;
    result.value = check.value;
    result.lexicographicTieResidual = check.value-valueOptimum;
    result.algorithm = "safety-value LP and CLF SOCP";
end

function solve = localRunJointProgram(problem, cfg)
    hook = cfg.solver.jointFunction;
    try
        if isempty(hook)
            solve = localDefaultSolve(problem, cfg);
        else
            % The hook solves the same reduced conic program as the native
            % path: inactive columns are removed together with the rows they
            % leave identically zero, so no degenerate dependent constraints
            % remain. Only physical decisions survive independent checking.
            program = problem.stageProgram;
            [reduced,retained] = localReducedProgram(program);
            reduced.defaultSolver = @() localReduceSolve(localDefaultSolve(problem, cfg), retained);
            solve = hook("joint", reduced);
            if isstruct(solve) && isscalar(solve) && isfield(solve,"decision") ...
                    && numel(solve.decision)==nnz(retained)
                expanded = zeros(numel(program.q),1);
                expanded(retained) = solve.decision;
                solve.decision = expanded;
            end
        end
    catch exception
        solve = localEmptySolve();
        solve.exitFlag = -999;
        solve.output = struct("message", string(exception.message));
    end
    if isstruct(solve) && isscalar(solve) && isfield(solve,"decision") ...
            && numel(solve.decision)==numel(problem.stageProgram.q) ...
            && all(isfinite(solve.decision),"all")
        solve.fullDecision = solve.decision(:);
        solve.decision = solve.decision(1:problem.layout.decisionCount);
    end
    solve = localNormalizeSolve(solve, problem.layout.decisionCount, numel(problem.stageProgram.q));
end

function [reduced,retained] = localReducedProgram(program)
% Remove inactive columns and the equality/nonnegative rows they leave
% identically zero with a satisfiable right-hand side.
    retained = true(numel(program.q),1);
    if isfield(program,"inactiveSlackIndex"),retained(program.inactiveSlackIndex) = false;end
    matrix = program.A(:,retained);
    equalities = program.cones(1);
    linearRows = equalities+program.cones(2);
    rowNorm = full(sum(abs(matrix(1:linearRows,:)),2));
    trivialEquality = (1:equalities).' <= equalities & rowNorm(1:equalities)==0 & program.b(1:equalities)==0;
    trivialInequality = rowNorm(equalities+1:linearRows)==0 & program.b(equalities+1:linearRows)>=0;
    keep = true(size(matrix,1),1);
    keep(1:equalities) = ~trivialEquality;
    keep(equalities+1:linearRows) = ~trivialInequality;
    reduced = program;
    reduced.P = program.P(retained,retained);
    reduced.q = program.q(retained);
    reduced.A = matrix(keep,:);
    reduced.b = program.b(keep);
    reduced.cones = [equalities-nnz(trivialEquality);program.cones(2)-nnz(trivialInequality);program.cones(3:end)];
    reduced.inactiveSlackIndex = zeros(1,0);
end

function solve = localReduceSolve(solve, retained)
    if isstruct(solve) && isfield(solve,"decision") && numel(solve.decision)==numel(retained)
        solve.decision = solve.decision(retained);
    end
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
    linear = program.q(retained);
    bound = program.b;
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
    output.algorithm = "Clarabel safety-value LP and CLF SOCP";
    output.message = "Clarabel status "+string(output.status);
    solve = struct("decision", stageDecision, ...
        "exitFlag", flag, "output", output);
end

function solve = localNormalizeSolve(solve, decisionCount, variableCount)
    if ~isstruct(solve) || ~isscalar(solve)
        solve = localEmptySolve();
        solve.message = "solver hook returned no scalar result structure";
        return;
    end
    if ~isfield(solve, "decision"), solve.decision = zeros(0, 1); end
    if ~isfield(solve, "fullDecision"), solve.fullDecision = zeros(variableCount, 1); end
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
    solve = struct("decision", zeros(0, 1), "fullDecision", zeros(0, 1), ...
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
        "algorithm", "safety-value LP and CLF SOCP", ...
        "message", "", ...
        "objectiveValue", inf, ...
        "clfValue", inf, ...
        "valueStageOptimum", inf, ...
        "value", inf, ...
        "lexicographicTieResidual", 0);
end
