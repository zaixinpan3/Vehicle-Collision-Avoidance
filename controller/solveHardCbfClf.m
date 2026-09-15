classdef solveHardCbfClf
    %solveHardCbfClf Hard-constrained optimization with solver-status execution.
    % All physical safety rows enter the solve without violation variables.
    % certify/certifyInputs are offline research audit utilities; the online
    % solve never calls them or repairs a returned decision.

    methods (Static)
        function [result,problem] = solve(problem, cfg)
            [result,problem] = localHardSolve(problem,cfg);
        end

        function solve = constrained(program,cfg)
        % Execute only a fully solved hard-constrained program. No external
        % residual calculation or feasible-iterate acceptance follows it.
            program = localCompactPlanarRows(program);
            problem = struct('layout',struct('decisionCount',numel(program.q)),'stageProgram',program);
            solve = localRunJointProgram(problem,cfg);
        end

        function record = solverAcceptance(solve,count)
        % Status metadata, not a separately evaluated safety certificate.
            record = struct('accepted',solve.feasible,'candidateAccepted',solve.feasible, ...
                'safetyCertified',solve.feasible,'value',0,'stageViolation',zeros(count,1), ...
                'hardRowViolation',NaN,'clfViolation',NaN,'margin',NaN, ...
                'sweptClearanceMargin',NaN,'exitMargin',NaN,'failedConditions',strings(1,0), ...
                'basis',"hardConstraintsAndSolverStatus",'solverExitFlag',solve.exitFlag);
            if ~solve.feasible,record.failedConditions="solverStatus";end
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
        % Offline audit of physical rows and CLF residuals; not an execution gate.
            stages = qp.layout.horizonSteps;
            check = struct("accepted", false, "candidateAccepted",false,"safetyCertified",false, ...
                "failedConditions", "decision", ...
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
            check.candidateAccepted = all(conditions);
            check.safetyCertified = check.candidateAccepted && check.value==0;
            check.accepted = check.safetyCertified;
            if check.candidateAccepted && ~check.safetyCertified
                check.failedConditions(end+1) = "positiveSafetyViolation";
            end
        end
    end
end

function program = localCompactPlanarRows(program)
% Intersect all one-coordinate inequalities before the numerical solve.
% This is an equivalent hard-bound reduction (with inward roundoff), not
% a check of a returned plan. Coupled rows and every cone remain unchanged.
    if numel(program.q)~=2 || program.cones(1)~=0,return;end
    count = program.cones(2);
    linear = program.A(1:count,:);bound = program.b(1:count);
    if isfield(program,'decisionRadius')
        radius = program.decisionRadius;
        coefficients = full(linear);
        [small,coordinate] = min(abs(coefficients),[],2);
        large = max(abs(coefficients),[],2);
        selected = small>0 & small<=1e-10*large;
        rows = find(selected);
        % Retain small cross-effects as an uncertainty support over explicit
        % hard decision bounds. No coefficient is silently rounded to zero.
        support = small(selected).*radius(coordinate(selected));
        bound(selected) = bound(selected)-support ...
            -8*eps(max(1,abs(bound(selected))+support));
        coefficients(sub2ind(size(coefficients),rows,coordinate(selected))) = 0;
        linear = [sparse(coefficients);sparse([1,0;-1,0;0,1;0,-1])];
        bound = [bound;radius(1);radius(1);radius(2);radius(2)];
    end
    first = full(linear(:,1));second = full(linear(:,2));
    axis = (first~=0 & second==0) | (first==0 & second~=0);
    if ~any(axis),return;end
    indices = find(axis);
    coefficient = first(axis)+second(axis);
    limit = bound(axis)./abs(coefficient);
    limit = limit-8*eps(max(1,abs(limit)));
    finite = isfinite(limit);
    axis(indices(~finite)) = false;
    coefficient = coefficient(finite);limit = limit(finite);
    coordinate = 1+double(second(axis)~=0);
    group = 2*coordinate-1+double(coefficient<0);
    compact = accumarray(group,limit,[4,1],@min,Inf);
    present = isfinite(compact);
    directions = [1,0;-1,0;0,1;0,-1];
    retained = ~axis;
    program.A = [linear(retained,:);sparse(directions(present,:));program.A(count+1:end,:)];
    program.b = [bound(retained);compact(present);program.b(count+1:end)];
    program.cones(2) = nnz(retained)+nnz(present);
end

function [result,problem] = localHardSolve(problem,cfg)
% Safety violation variables are absent from the condensed optimization.
    n = problem.layout.decisionCount;
    anchor = zeros(n,1);anchor(problem.layout.planIndex) = problem.anchorPlan;
    if string(cfg.solver.programForm)=="lifted"
        program = problem.stageProgram;
        program.inactiveSlackIndex = program.violationIndex;
    else
        matrix = problem.inequalityMatrix;
        bound = problem.inequalityBound-matrix*anchor;
        cones = [0;numel(bound)];
        constraints = problem.clf.constraints;
        rows = zeros(10*numel(constraints),n);limits = zeros(10*numel(constraints),1);
        for index = 1:numel(constraints)
            constraint = constraints(index);
            value = constraint.map*anchor+constraint.offset;
            tau = -constraint.linear.'*constraint.map;
            tau(problem.layout.relaxationIndex(constraint.stage)) = ...
                tau(problem.layout.relaxationIndex(constraint.stage))+1;
            offset = -constraint.linear.'*value-constraint.constant;
            selected = 10*(index-1)+(1:10);
            rows(selected,:) = -[tau;2*constraint.root*constraint.map;tau];
            limits(selected) = [offset+1;2*constraint.root*value;offset-1];
        end
        program = struct('P',triu(sparse(problem.Hessian)), ...
            'q',problem.linear+problem.Hessian*anchor,'A',sparse([matrix;rows]), ...
            'b',[bound;limits],'cones',[cones;10*ones(numel(constraints),1)], ...
            'physicalDecisionCount',n,'inactiveSlackIndex',zeros(1,0));
    end
    timer = tic;
    solve = solveHardCbfClf.constrained(program,cfg);
    result = localEmptyResult();
    result.feasible = solve.feasible;result.exitFlag = solve.exitFlag;
    result.message = solve.message;result.solverCalls = 1;result.output = solve.output;
    result.acceptance = solveHardCbfClf.solverAcceptance(solve,problem.layout.horizonSteps);
    result.programDiagnostics = localProgramDiagnostics("hardConstraints",solve,toc(timer));
    result.algorithm = "hard-constrained CLF SOCP";
    if solve.feasible
        result.decision = solve.decision(1:n);
        if string(cfg.solver.programForm)=="condensed",result.decision = result.decision+anchor;end
        result.objectiveValue = localJointValue(problem,result.decision);
        result.clfValue = result.decision(problem.layout.relaxationIndex);
        result.valueStageOptimum = 0;result.value = 0;result.lexicographicTieResidual = 0;
        result.iterations = localIterationCount(solve.output);
    end
end

function record = localProgramDiagnostics(name, solve, seconds)
    record = struct("program",name,"exitFlag",solve.exitFlag,"rounds",NaN,"workingRows",NaN, ...
        "nativeCalls",NaN,"seconds",seconds);
    if isstruct(solve.output)
        if isfield(solve.output,"rowGenerationRounds"), record.rounds = solve.output.rowGenerationRounds; end
        if isfield(solve.output,"workingRowCount"), record.workingRows = solve.output.workingRowCount; end
        if isfield(solve.output,"nativeCalls"), record.nativeCalls = solve.output.nativeCalls; end
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

function solve = localRunJointProgram(problem, cfg)
    hook = cfg.solver.jointFunction;
    try
        if isempty(hook)
            solve = localDefaultSolve(problem,cfg);
        else
            % The hook solves the same reduced conic program as the native
            % path: inactive columns are removed together with the rows they
            % leave identically zero, so no degenerate dependent constraints
            % remain. The hook must satisfy the same feasibility/status contract.
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
    original = problem.stageProgram;
    [program,retained] = localReducedProgram(original);
    linear = program.q;
    bound = program.b;
    % Positive objective scaling preserves minimizers. The lifted CLF
    % epigraph can otherwise trigger a false native infeasibility report.
    hessian = sparse(program.P);
    objectiveScale = 1/max([1;abs(linear);abs(nonzeros(hessian))]);
    % Apply the requested optimization tolerance to the normalized objective.
    % Rescaling it a second time can demand sub-machine objective accuracy;
    % the independent primal feasibility tolerance is unchanged.
    options = [cfg.solver.constraintTolerance, ...
        cfg.solver.optimalityTolerance,cfg.solver.maxIterations];
    if isfield(cfg.solver,"workTimer") && isfinite(cfg.solver.workTimeLimit)
        remaining = cfg.solver.workTimeLimit-toc(cfg.solver.workTimer);
        if remaining<=0
            solve = localEmptySolve();
            solve.exitFlag = 0;
            solve.output = struct("message","The program work deadline expired before the native solve.");
            return;
        end
        options(4) = remaining;
    end
    [nativeDecision, output] = nativeSolver( ...
        objectiveScale*hessian, objectiveScale*linear, sparse(program.A), bound, program.cones, ...
        options);
    output.objectiveValue = output.objectiveValue/objectiveScale;
    output.objectiveScale = objectiveScale;
    stageDecision = zeros(numel(original.q),1);stageDecision(retained) = nativeDecision;
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
    output.algorithm = "Clarabel hard-constrained CLF SOCP";
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
    % Execution relies on a fully solved optimizer status, never on an
    % independently checked iterate from a failed or incomplete solve.
    solve.feasible = solve.exitFlag==1 ...
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
        "algorithm", "hard-constrained CLF SOCP", ...
        "message", "", ...
        "objectiveValue", inf, ...
        "clfValue", inf, ...
        "valueStageOptimum", inf, ...
        "value", inf, ...
        "lexicographicTieResidual", 0);
end
