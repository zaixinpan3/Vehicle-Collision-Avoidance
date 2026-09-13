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
            if string(cfg.solver.programForm)=="condensed"
                [result, problem] = localCondensedLexicographicSolve(problem, cfg);
            else
                [result, problem] = localLexicographicSolve(problem, cfg);
            end
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
        "inactiveSlackIndex",[problem.layout.relaxationIndex(:).',base.violationIndex(:).'], ...
        "generatedRowCount",numel(base.rowMap.inequality),"anchorPoint",[base.anchorPoint;0]);
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
            "physicalDecisionCount",physical,"inactiveSlackIndex",problem.layout.relaxationIndex, ...
            "generatedRowCount",numel(base.rowMap.inequality),"anchorPoint",base.anchorPoint);
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
    % The value-stage point seeds the performance stage's working set.
    if isfield(marginSolve,"fullDecision") && numel(marginSolve.fullDecision)>=numel(base.q)
        problem.stageProgram.referencePoint = marginSolve.fullDecision(1:numel(base.q));
    end
    if exist("valueSolve","var") && numel(valueSolve.fullDecision)==numel(base.q)
        problem.stageProgram.referencePoint = valueSolve.fullDecision;
    end
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

function [result, problem] = localCondensedLexicographicSolve(problem, cfg)
% The same three tiers on the physical unknowns only: every row is the
% condensed row of the verification, centred at the seed plan so that the
% right-hand sides are margins rather than absolute stations. Stage A is the
% margin LP (feasible exactly when a zero-violation plan exists), else the
% value LP; stage B is the CLF SOCP with the same rotated-cone encoding of
% the convex majorant as the lifted form, written on the physical
% unknowns. Every program runs through row generation; the matrices stay
% dense because every condensed row couples all inputs.
    n = problem.layout.decisionCount;
    count = problem.layout.horizonSteps;
    relaxationIndex = problem.layout.relaxationIndex(:).';
    model = struct("cfg", cfg);
    result = localEmptyResult();
    anchor = zeros(n,1);
    anchor(problem.layout.planIndex) = problem.anchorPlan(:);
    matrix = problem.inequalityMatrix;
    scale = problem.barrier.scale;
    baseMargin = problem.barrier.baseBound-matrix*anchor;
    safety = problem.safetyRows;
    rowStage = problem.rowStage;
    % The geometry rows, the ones subject to generation, are the leading
    % rows of the condensed matrix; no reordering copy is needed.
    generated = size(problem.geometry.matrix,1);
    rowCount = numel(baseMargin);
    orderedMatrix = matrix;
    orderedScale = scale;
    orderedSafety = safety;
    orderedStage = rowStage;
    order = (1:rowCount).';
    % Violation columns: one per stage, entering the safety rows with -1.
    violationColumns = zeros(rowCount,count);
    violationColumns(sub2ind(size(violationColumns),find(orderedSafety),orderedStage(orderedSafety))) = -1;
    % ---- Stage A: margin LP over [delta; m], slacks and violations inactive.
    % Stage A decides zero-violation feasibility and supplies an interior
    % point. When the seed itself satisfies every tightened row, that is
    % settled without a solve. Otherwise the closest zero-violation point
    % to the seed is found by a proximal QP; unlike a margin maximisation it
    % is not degenerate in the inputs, so a working-set solve settles in a
    % round or two instead of wandering through the omitted rows.
    timer = tic;
    seedFeasible = all(baseMargin>=0);
    marginSolve = localEmptySolve();
    marginSolve.feasible = seedFeasible;
    marginSolve.exitFlag = 1;
    marginSolve.decision = zeros(n,1);
    marginSolve.output = struct("message","seed satisfies every row","rowGenerationRounds",0, ...
        "workingRowCount",0,"nativeCalls",0);
    if ~seedFeasible
        proximalProgram = struct("P",speye(n),"q",zeros(n,1), ...
            "A",orderedMatrix,"b",baseMargin(order)-problem.requiredMargin*orderedScale, ...
            "cones",[0;numel(order)],"physicalDecisionCount",n, ...
            "inactiveSlackIndex",relaxationIndex, ...
            "generatedRowCount",generated,"anchorPoint",zeros(n,1));
        auxiliary = struct("layout",struct("decisionCount",n),"stageProgram",proximalProgram);
        marginSolve = localRunJointProgram(auxiliary, cfg);
    end
    result.solverCalls = double(~seedFeasible);
    result.exitFlag = marginSolve.exitFlag;
    result.message = "zero-value feasibility: "+marginSolve.message;
    result.programDiagnostics = localProgramDiagnostics("feasibility",marginSolve,toc(timer));
    if marginSolve.feasible
        valueOptimum = 0;
        candidate = localRepairClf(problem, anchor+marginSolve.decision(1:n), cfg);
        check = solveHardCbfClf.certify(problem, [], model, candidate);
        if ~check.accepted || check.value > 0
            result.message = result.message+"; "+strjoin(check.failedConditions,",");
            return;
        end
        selected = scale>0;
        available = min((problem.barrier.baseBound(selected) ...
            -matrix(selected,:)*candidate)./scale(selected));
        interiorMargin = max(problem.requiredMargin, ...
            min(10*cfg.encounter.numericalMargin,0.5*max(0,available)));
        violationsActive = false;
        budget = 0;
        referencePoint = [marginSolve.decision(1:n);zeros(count,1)];
    else
        if ~ismember(marginSolve.exitFlag,[-2,-3])
            return;
        end
        timer = tic;
        valueProgram = struct("P",sparse(n+count,n+count),"q",[zeros(n,1);ones(count,1)], ...
            "A",[orderedMatrix,violationColumns;zeros(count,n),-eye(count)], ...
            "b",[baseMargin(order);zeros(count,1)], ...
            "cones",[0;numel(order)+count],"physicalDecisionCount",n, ...
            "inactiveSlackIndex",relaxationIndex, ...
            "generatedRowCount",generated,"anchorPoint",zeros(n+count,1));
        auxiliary = struct("layout",struct("decisionCount",n),"stageProgram",valueProgram);
        valueSolve = localRunJointProgram(auxiliary, cfg);
        result.solverCalls = result.solverCalls+1;
        result.exitFlag = valueSolve.exitFlag;
        result.message = "safety value: "+valueSolve.message;
        result.programDiagnostics(end+1) = localProgramDiagnostics("value",valueSolve,toc(timer));
        if ~valueSolve.feasible, return; end
        valueOptimum = sum(max(0, valueSolve.fullDecision(n+(1:count))));
        candidate = localRepairClf(problem, anchor+valueSolve.decision(1:n), cfg);
        check = solveHardCbfClf.certify(problem, [], model, candidate);
        if ~check.accepted
            result.message = result.message+"; "+strjoin(check.failedConditions,",");
            return;
        end
        interiorMargin = problem.requiredMargin;
        violationsActive = true;
        budget = valueOptimum+cfg.solver.lexicographicTieTolerance;
        referencePoint = valueSolve.fullDecision(1:n+count);
    end
    result.valueStageOptimum = valueOptimum;
    problem.inequalityBound = problem.barrier.baseBound-interiorMargin*scale;
    % ---- Stage B: CLF SOCP over [delta; xi]. Each majorant s >= |R v|^2 + l'v + c
    % with v = M(anchor+delta)+o is the rotated cone
    % (tau+1, 2 R v, tau-1) with tau = s - l'v - c, as in the lifted form.
    constraints = problem.clf.constraints;
    coneCount = numel(constraints);
    coneRows = zeros(10*coneCount,n+count);
    coneBound = zeros(10*coneCount,1);
    for index = 1:coneCount
        constraint = constraints(index);
        value0 = constraint.map*anchor+constraint.offset;
        tMap = -constraint.linear.'*constraint.map;
        tMap(relaxationIndex(constraint.stage)) = tMap(relaxationIndex(constraint.stage))+1;
        tOffset = -constraint.linear.'*value0-constraint.constant;
        rows = 10*(index-1)+(1:10);
        coneRows(rows,1:n) = -[tMap;2*constraint.root*constraint.map;tMap];
        coneBound(rows) = [tOffset+1;2*constraint.root*value0;tOffset-1];
    end
    hessian = sparse(n+count,n+count);
    hessian(1:n,1:n) = sparse(problem.Hessian);
    linear = [problem.linear+problem.Hessian*anchor;zeros(count,1)];
    inequality = [orderedMatrix,violationColumns; ...
        zeros(count,n),-eye(count); ...
        zeros(1,n),ones(1,count)];
    bound = [baseMargin(order)-interiorMargin*orderedScale;zeros(count,1);budget];
    inactive = zeros(1,0);
    if ~violationsActive, inactive = n+(1:count); end
    performanceProgram = struct("P",triu(hessian),"q",linear, ...
        "A",[inequality;coneRows],"b",[bound;coneBound], ...
        "cones",[0;size(inequality,1);10*ones(coneCount,1)],"physicalDecisionCount",n, ...
        "inactiveSlackIndex",inactive,"generatedRowCount",generated, ...
        "anchorPoint",zeros(n+count,1),"referencePoint",referencePoint);
    auxiliary = struct("layout",struct("decisionCount",n),"stageProgram",performanceProgram);
    timer = tic;
    solve = localRunJointProgram(auxiliary, cfg);
    calls = result.solverCalls+1;
    result.solverCalls = calls;
    result.exitFlag = solve.exitFlag;
    result.message = "performance: "+solve.message;
    diagnostics = [result.programDiagnostics,localProgramDiagnostics("performance",solve,toc(timer))];
    result.programDiagnostics = diagnostics;
    if ~solve.feasible, return; end
    decision = localRepairClf(problem, anchor+solve.decision(1:n), cfg);
    check = solveHardCbfClf.certify(problem, [], model, decision);
    result.decision = decision;
    if check.accepted
        result = localCertifiedResult(problem, decision, solve, calls, valueOptimum, check);
        result.algorithm = "condensed safety-value LP and CLF SOCP";
        result.programDiagnostics = diagnostics;
    else
        result.message = result.message+"; "+strjoin(check.failedConditions,",") ...
            +"; hard violation "+check.hardRowViolation+"; value "+check.value;
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

function result = localCertifiedResult(problem, decision, solve, calls, valueOptimum, check)
    result = localEmptyResult();
    result.feasible = true;
    result.decision = decision;
    result.exitFlag = solve.exitFlag;
    result.message = solve.message;
    result.solverCalls = calls;
    result.iterations = localIterationCount(solve.output);
    result.output = solve.output;
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
            if cfg.solver.rowGeneration && isfield(problem.stageProgram,"generatedRowCount") ...
                    && problem.stageProgram.generatedRowCount>0
                solve = localGeneratedSolve(problem, cfg);
            else
                solve = localDefaultSolve(problem, cfg);
            end
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

function solve = localGeneratedSolve(problem, cfg)
% Row generation: solve on a working set of the hard rows, check every
% omitted row at the solution, add the violated ones and repeat. A solution
% of the relaxed program that violates no omitted row solves the full
% program; an infeasible relaxed program proves the full one infeasible.
% The accepted plan is verified on all physical rows afterwards regardless.
    program = problem.stageProgram;
    equalities = program.cones(1);
    nonnegative = program.cones(2);
    generated = program.generatedRowCount;
    eligible = equalities+(1:generated).';
    essential = equalities+(generated+1:nonnegative).';
    conic = (equalities+nonnegative+1:size(program.A,1)).';
    variables = numel(program.q);
    reference = zeros(variables,1);
    if isfield(program,"referencePoint") && numel(program.referencePoint)==variables
        reference = program.referencePoint(:);
    elseif isfield(program,"anchorPoint")
        reference(1:min(variables,numel(program.anchorPoint))) = program.anchorPoint(1:min(variables,numel(program.anchorPoint)));
    end
    % Products use the whole matrix once and slice the vector: slicing
    % thousands of dense rows out of the matrix would copy it per round.
    eligibleBound = program.b(eligible);
    fullProduct = program.A*reference;
    slack = eligibleBound-fullProduct(eligible);
    seedCount = min(generated,max(100,4*program.physicalDecisionCount));
    [~,order] = sort(slack,"ascend");
    working = false(generated,1);
    working(order(1:seedCount)) = true;
    working(slack<=0) = true;
    rounds = 0;
    maximumRounds = 6;
    nativeCalls = 0;
    while rounds<maximumRounds
        rounds = rounds+1;
        nativeCalls = nativeCalls+1;
        rows = [(1:equalities).';eligible(working);essential;conic];
        sub = problem;
        sub.stageProgram = program;
        sub.stageProgram.A = program.A(rows,:);
        sub.stageProgram.b = program.b(rows);
        sub.stageProgram.cones = [equalities;nnz(working)+numel(essential);program.cones(3:end)];
        solve = localDefaultSolve(sub, cfg);
        solve.output.rowGenerationRounds = rounds;
        solve.output.workingRowCount = nnz(working);
        solve.output.nativeCalls = nativeCalls;
        if any(solve.exitFlag==[-2,-3])
            % Infeasible or unbounded on a row subset is conclusive for the
            % complete program.
            return;
        end
        if ~any(solve.exitFlag==[1,2]) || numel(solve.decision)~=variables || any(~isfinite(solve.decision))
            % Only a cleanly solved subset can certify the omitted rows;
            % anything else is decided by the complete program.
            break;
        end
        point = solve.decision;
        fullProduct = program.A*point;
        magnitude = abs(program.A)*abs(point);
        residual = fullProduct(eligible)-eligibleBound;
        tolerance = 10*cfg.solver.constraintTolerance*(1+abs(eligibleBound)+magnitude(eligible));
        violated = ~working & residual>tolerance;
        if ~any(violated)
            return;
        end
        working(violated) = true;
    end
    % The working set did not settle: fall back to the complete program.
    solve = localDefaultSolve(problem, cfg);
    solve.output.rowGenerationRounds = rounds+1;
    solve.output.workingRowCount = generated;
    solve.output.nativeCalls = nativeCalls+1;
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
    hessian = sparse(program.P(retained,retained));
    objectiveScale = 1/max([1;abs(linear);abs(nonzeros(hessian))]);
    [nativeDecision, output] = nativeSolver( ...
        objectiveScale*hessian, objectiveScale*linear, sparse(program.A(:,retained)), bound, program.cones, ...
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
