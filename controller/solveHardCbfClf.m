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

function solve = localRunJointProgram(program, cfg)
    hook = cfg.solver.jointFunction;
    program.defaultSolver = @() localDefaultSolve(program, cfg);
    try
        if isempty(hook)
            solve = localDefaultSolve(program, cfg);
        else
            solve = hook("joint", program);
        end
    catch exception
        solve = localEmptySolve();
        solve.exitFlag = -999;
        solve.output = struct("message", string(exception.message));
    end
    solve = localNormalizeSolve(solve, numel(program.f));
end

function solve = localDefaultSolve(program, cfg)
    if isempty(which("sedumi"))
        solverRoot = fullfile(fileparts(fileparts(mfilename("fullpath"))), ...
            "solver", "sedumi");
        if ~isfolder(solverRoot)
            error("collisionAvoidanceController:missingSocpSolver", ...
                "The predictive SOCP requires SeDuMi in %s.", solverRoot);
        end
        addpath(genpath(solverRoot));
    end
    % Eliminate fixed inputs and terminal rest before the single conic call.
    % The physical decision is the dual variable y in SeDuMi's convention:
    % max -f'*y subject to c-A'*y in a product of nonnegative/SOC cones.
    % This conversion preserves the objective and every feasible input.
    [reduced, map, offset] = localEliminateEqualities(program);
    identity = speye(numel(reduced.f));
    upper = find(isfinite(reduced.ub));
    lower = find(isfinite(reduced.lb));
    blocks = cell(numel(reduced.cones)+1, 1);
    constants = cell(size(blocks));
    blocks{1} = [sparse(reduced.A); identity(upper, :); -identity(lower, :)];
    constants{1} = [reduced.b; reduced.ub(upper); -reduced.lb(lower)];
    cones = struct("l", size(blocks{1}, 1), ...
        "q", zeros(1, numel(reduced.cones)));
    for coneIdx = 1:numel(reduced.cones)
        cone = reduced.cones{coneIdx};
        blocks{coneIdx+1} = [-sparse(cone.d.'); -sparse(cone.A)];
        constants{coneIdx+1} = [-cone.gamma; -cone.b];
        cones.q(coneIdx) = size(cone.A, 1)+1;
    end
    % SeDuMi uses relative residuals. Request high internal precision and
    % retain the independent absolute acceptance checks in physical units.
    options = struct("fid", 0, "eps", min([1.0e-9, ...
        cfg.solver.constraintTolerance, cfg.solver.optimalityTolerance]), ...
        "maxiter", cfg.solver.maxIterations);
    [~, reducedDecision, information] = sedumi( ...
        vertcat(blocks{:}).', -reduced.f, vertcat(constants{:}), cones, options);
    exitFlag = 1;
    if information.dinf
        exitFlag = -2;
    elseif information.pinf
        exitFlag = -3;
    elseif information.numerr
        exitFlag = -7;
    end
    decision = zeros(0, 1);
    if numel(reducedDecision) == numel(reduced.f)
        decision = map*reducedDecision+offset;
    end
    output = struct("iterations", information.iter, ...
        "algorithm", "SeDuMi joint predictive SOCP", ...
        "message", "SeDuMi numerr="+string(information.numerr) ...
            +", pinf="+string(information.pinf)+", dinf="+string(information.dinf));
    solve = struct("decision", decision, ...
        "exitFlag", exitFlag, "output", output);
    solve = localNormalizeSolve(solve, numel(program.f));
end

function [reduced, map, offset] = localEliminateEqualities(program)
    count = numel(program.f);
    fixed = find(program.lb == program.ub);
    free = setdiff((1:count).', fixed);
    offset = zeros(count, 1);
    offset(fixed) = program.lb(fixed);
    equality = program.Aeq(:, free);
    bound = program.beq-program.Aeq*offset;
    rowCount = size(equality, 1);
    % Prefer late controls: eliminating the first acceleration would couple
    % the first-step CLF to the sum of every future acceleration. Keep that
    % short performance cone sparse, and use rank-revealing QR on a small
    % suffix of the controls that influence the endpoint.
    active = find(any(equality ~= 0.0, 1));
    selected = active(max(1, end-2*rowCount+1):end);
    if rank(equality(:, selected)) < rowCount
        selected = active;
    end
    [~, triangular, permutation] = qr(equality(:, selected), "vector");
    if rank(triangular) ~= rowCount
        error("collisionAvoidanceController:invalidProblem", ...
            "The terminal velocity equalities must have full row rank.");
    end
    pivot = free(selected(permutation(1:rowCount)));
    independent = setdiff(free, pivot);
    map = sparse(independent, 1:numel(independent), 1.0, ...
        count, numel(independent));
    pivotMatrix = program.Aeq(:, pivot);
    map(pivot, :) = -pivotMatrix\program.Aeq(:, independent);
    % A distributed particular solution avoids encoding a full stop as an
    % artificial hundreds-of-m/s^2 pivot input in the cone offsets.
    offset(free) = equality.'*((equality*equality.')\bound);
    offset(pivot) = offset(pivot) ...
        + pivotMatrix\(program.beq-program.Aeq*offset);
    reduced = struct("f", map.'*program.f, ...
        "A", program.A*map, "b", program.b-program.A*offset, ...
        "lb", program.lb(independent)-offset(independent), ...
        "ub", program.ub(independent)-offset(independent));
    for variable = pivot(:).'
        if isfinite(program.ub(variable))
            reduced.A(end+1, :) = map(variable, :);
            reduced.b(end+1) = program.ub(variable)-offset(variable);
        end
        if isfinite(program.lb(variable))
            reduced.A(end+1, :) = -map(variable, :);
            reduced.b(end+1) = offset(variable)-program.lb(variable);
        end
    end
    reduced.cones = cell(size(program.cones));
    for coneIdx = 1:numel(program.cones)
        cone = program.cones{coneIdx};
        reduced.cones{coneIdx} = secondordercone( ...
            cone.A*map, cone.b-cone.A*offset, ...
            map.'*cone.d, cone.gamma-cone.d.'*offset);
    end
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
