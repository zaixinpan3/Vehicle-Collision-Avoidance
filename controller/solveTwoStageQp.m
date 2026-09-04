function result = solveTwoStageQp(qp, cfg, policy)
% solveTwoStageQp Solve one candidate of the stage-2 program with the QP kernel.
%
% The program is handed to the kernel in deviation coordinates about
% its initial point (the incumbent plan), scaled by the input
% amplitudes, with every row normalized to unit 2-norm - the decision
% spans radians and m/s^2, and handing the raw program over was
% measured to fail on ordinary frames. The kernel
% is quadprog's active set (the condensed program is small and dense,
% the active set's case); an exhausted iteration budget, a spurious
% unbounded-ray verdict, or a failed step computation (exit flags 0,
% -3, -8 - kernel misdiagnoses on a program that is bounded below by
% construction) is answered first by ONE interior-point solve, a
% different method, and then by the step-tolerance ladder resumed from
% the reached iterate. A declared infeasibility (-2) is the answer:
% there is no second kernel and no arbiter - "no solution" is one
% kernel's verdict, reported as such.
%
% policy (optional): maxIterations - the budget of this call (default
% cfg.solver.maxIterations); retry - whether a stalled kernel is
% followed by the other method and the step ladder (default true);
% algorithm - which kernel goes first, "active-set" (default) or
% "interior-point-convex". A candidate that is not the incumbent is
% solved with a smaller budget and no retry; the controller retries
% only when no candidate resolved. THE LADDER'S PROBES ASK FOR
% INTERIOR POINT: their elastic rows put the optimum far from the
% initial point, and the active set walks there one constraint at a
% time - measured 709 pivots and 0.43 s against 16 interior-point
% iterations and 0.06 s for the same probe.
%
% result: decision (empty unless feasible), exitFlag, feasible,
% infeasible, unresolved (stalled without a verdict), iterations,
% solverCalls, retried, algorithm, message, objectiveValue.

    if nargin < 3 || isempty(policy)
        policy = struct();
    end
    if ~isfield(policy, "maxIterations")
        policy.maxIterations = cfg.solver.maxIterations;
    end
    if ~isfield(policy, "retry")
        policy.retry = true;
    end
    if ~isfield(policy, "algorithm")
        policy.algorithm = "active-set";
    end
    policy.algorithm = string(policy.algorithm);
    if policy.algorithm == "active-set"
        retryAlgorithm = "interior-point-convex";
    else
        retryAlgorithm = "active-set";
    end
    initialDecision = qp.initialDecision(:);
    variableScale = qp.variableScale(:);
    scaled = localScaledProgram(qp, initialDecision, variableScale);
    origin = zeros(size(initialDecision));

    algorithm = policy.algorithm;
    options = localQuadprogOptions(cfg, algorithm, policy.maxIterations);
    [scaledDecision, exitFlag, output] = localCallKernel( ...
        scaled, origin, options, cfg);
    solverCalls = 1;
    retried = false;
    retryExitFlags = [0, -3, -8];
    if any(exitFlag == retryExitFlags) && policy.retry ...
            && isempty(cfg.solver.function)
        retried = true;
        [candidate, candidateFlag, candidateOutput] = localCallKernel( ...
            scaled, origin, ...
            localQuadprogOptions(cfg, retryAlgorithm, ...
                cfg.solver.maxIterations), cfg);
        solverCalls = solverCalls+1;
        if candidateFlag > 0
            scaledDecision = candidate;
            exitFlag = candidateFlag;
            output = candidateOutput;
            algorithm = retryAlgorithm;
        end
        if any(exitFlag == retryExitFlags)
            for retryStepTolerance = cfg.solver.stepToleranceRetry(:).'
                retryOptions = options;
                retryOptions.StepTolerance = retryStepTolerance;
                resumePoint = origin;
                if isnumeric(scaledDecision) && isreal(scaledDecision) ...
                        && numel(scaledDecision) == numel(origin) ...
                        && all(isfinite(scaledDecision))
                    resumePoint = scaledDecision(:);
                end
                [scaledDecision, exitFlag, output] = localCallKernel( ...
                    scaled, resumePoint, retryOptions, cfg);
                solverCalls = solverCalls+1;
                if ~any(exitFlag == retryExitFlags)
                    break;
                end
            end
        end
    end

    feasible = exitFlag > 0 && isnumeric(scaledDecision) ...
        && isreal(scaledDecision) ...
        && numel(scaledDecision) == numel(initialDecision) ...
        && all(isfinite(scaledDecision));
    decision = zeros(0, 1);
    objectiveValue = inf;
    if feasible
        decision = initialDecision+variableScale.*scaledDecision(:);
        objectiveValue = 0.5*decision.'*qp.Hessian*decision ...
            + qp.linear.'*decision+qp.constant;
    end
    result = struct( ...
        "decision", decision, ...
        "exitFlag", exitFlag, ...
        "feasible", feasible, ...
        "infeasible", exitFlag == -2, ...
        "unresolved", any(exitFlag == retryExitFlags), ...
        "iterations", localOutputIterations(output), ...
        "solverCalls", solverCalls, ...
        "retried", retried, ...
        "algorithm", algorithm, ...
        "message", localSolverMessage(exitFlag, output), ...
        "objectiveValue", objectiveValue);
end

function scaled = localScaledProgram(qp, initialDecision, variableScale)
% Deviation coordinates z = z0 + S y, rows normalized, sparse storage.
    hessian = sparse(qp.Hessian);
    inequalityMatrix = sparse(qp.inequalityMatrix);
    scaledHessian = variableScale.*hessian.*variableScale.';
    scaled = struct();
    scaled.Hessian = 0.5*(scaledHessian+scaledHessian.');
    scaled.linear = variableScale.*(hessian*initialDecision+qp.linear);
    matrix = inequalityMatrix.*variableScale.';
    bound = qp.inequalityBound-inequalityMatrix*initialDecision;
    rowScale = full(sqrt(sum(matrix.^2, 2)));
    rowScale(rowScale == 0.0) = 1.0;
    scaled.inequalityMatrix = spdiags(1.0./rowScale, 0, ...
        numel(rowScale), numel(rowScale))*matrix;
    scaled.inequalityBound = bound./rowScale;
    scaled.lowerBound = (qp.lowerBound-initialDecision)./variableScale;
    scaled.upperBound = (qp.upperBound-initialDecision)./variableScale;
end

function [decision, exitFlag, output] = localCallKernel( ...
        scaled, initialPoint, options, cfg)
    solverFunction = cfg.solver.function;
    if isempty(solverFunction)
        solverFunction = @quadprog;
    end
    try
        [decision, ~, exitFlag, output] = solverFunction( ...
            scaled.Hessian, scaled.linear, ...
            scaled.inequalityMatrix, scaled.inequalityBound, ...
            zeros(0, numel(initialPoint)), zeros(0, 1), ...
            scaled.lowerBound, scaled.upperBound, ...
            initialPoint, options);
    catch exception
        decision = zeros(0, 1);
        exitFlag = -999;
        output = struct("message", string(exception.message));
    end
    if ~isnumeric(exitFlag) || ~isreal(exitFlag) ...
            || ~isscalar(exitFlag) || ~isfinite(exitFlag)
        exitFlag = -999;
    else
        exitFlag = double(exitFlag);
    end
end

function options = localQuadprogOptions(cfg, algorithm, maxIterations)
    persistent cachedKeys cachedOptions
    key = {maxIterations, cfg.solver.constraintTolerance, ...
        cfg.solver.optimalityTolerance, cfg.solver.stepTolerance, ...
        char(algorithm)};
    if isempty(cachedKeys)
        cachedKeys = {};
        cachedOptions = {};
    end
    for cacheIdx = 1:numel(cachedKeys)
        if isequal(cachedKeys{cacheIdx}, key)
            options = cachedOptions{cacheIdx};
            return;
        end
    end
    options = optimoptions("quadprog", ...
        "Algorithm", char(algorithm), ...
        "Display", "none", ...
        "MaxIterations", maxIterations, ...
        "ConstraintTolerance", cfg.solver.constraintTolerance, ...
        "OptimalityTolerance", cfg.solver.optimalityTolerance, ...
        "StepTolerance", cfg.solver.stepTolerance);
    cachedKeys{end+1} = key;
    cachedOptions{end+1} = options;
end

function iterations = localOutputIterations(output)
    iterations = 0;
    if isstruct(output) && isfield(output, "iterations") ...
            && isnumeric(output.iterations) ...
            && isscalar(output.iterations)
        iterations = double(output.iterations);
    end
end

function message = localSolverMessage(exitFlag, output)
    message = "convex solver exit flag "+string(exitFlag);
    if isstruct(output) && isfield(output, "message")
        message = message+": "+string(output.message);
    end
end
