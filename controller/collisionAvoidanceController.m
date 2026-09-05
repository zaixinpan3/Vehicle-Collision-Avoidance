function [command, predictedInput, planningProblem] = ...
        collisionAvoidanceController( ...
        egoState, targetEstimate, laneCenterline, cfg)
% collisionAvoidanceController Recursive hard-CBF, soft-CLF MPC.
%
% Every covered collision and road row is hard at every prediction node,
% including the measured state and braking tail. One joint conic solve per
% start minimizes input deviation and exact first-step CLF relaxation.
% The starts are the shifted plan and, when target constraints are present,
% the obstacle-free cruise reference. The least-cost feasible plan is used.
% A solver failure may use a compatible stored plan only after rechecking
% its shifted hard rows, node clearance, model assumptions and terminal set.
% See PCBF_CLF_ARCHITECTURE.md for the algorithm and certificate assumptions.
%
% Inputs are ego state, target estimate, route and configuration overrides.
% Outputs are the steering/acceleration command, input plan and diagnostics.
% collisionAvoidanceController("resetNominalTrajectory") clears stored state.

    persistent previousCertificate
    if nargin >= 1 && (ischar(egoState) ...
            || (isstring(egoState) && isscalar(egoState)))
        if nargin ~= 1 ...
                || string(egoState) ~= "resetNominalTrajectory"
            error("collisionAvoidanceController:invalidAction", ...
                "The only controller action is resetNominalTrajectory.");
        end
        previousCertificate = [];
        command = [];
        predictedInput = [];
        planningProblem = [];
        return;
    end

    if nargin < 4
        cfg = [];
    end
    if nargin < 3
        laneCenterline = [];
    end
    if nargin < 2
        targetEstimate = [];
    end

    cfg = localControllerConfiguration(cfg);
    [ego, lane, road, targets] = readPlanningInputs( ...
        egoState, targetEstimate, laneCenterline, cfg);
    model = localPredictionModel(ego, targets, lane, road, cfg);
    model.episodeIdentity = localEpisodeIdentity(model);
    certificateCompatible = localCertificateCompatible( ...
        previousCertificate, model.episodeIdentity, model);
    storedSchedule = [];
    previousPlan = [];
    if certificateCompatible
        model.targetHorizon = localShiftTargetHorizon( ...
            previousCertificate.targetHorizon, model.targetHorizon);
        model.targetPath = localTargetPath(model);
        storedSchedule = localShiftSchedule( ...
            previousCertificate.schedule, model);
        if isempty(storedSchedule)
            certificateCompatible = false;
            previousCertificate = [];
        else
            previousPlan = previousCertificate.plan;
        end
    elseif ~isempty(previousCertificate)
        previousCertificate = [];
    end
    prediction = ltvBicyclePrediction(model, storedSchedule);
    [nominalInput, nominalSource] = localNominalInput( ...
        previousPlan, prediction, model, previousCertificate);
    buildPlanningProblem = nargout >= 3;
    fallbackUsed = false;
    try
        [inputPlan, problem, plan] = localSolve( ...
            model, prediction, nominalInput);
    catch exception
        if ~certificateCompatible
            previousCertificate = [];
            rethrow(exception);
        end
        [inputPlan, problem, plan] = localStoredFallback( ...
            nominalInput, prediction, exception, model);
        fallbackUsed = true;
    end
    if ~fallbackUsed
        problem.metadata.planCertified = true;
        problem.metadata.certificateSource = ...
            "hardConstrainedOptimization";
        problem.metadata.postSolveCertificationPerformed = false;
    end
    previousCertificate = localStoredCertificate( ...
        plan, prediction, model);
    command = localCommand(inputPlan, model);
    predictedInput = inputPlan;
    planningProblem = [];
    if buildPlanningProblem
        problem.nominalSource = nominalSource;
        problem.metadata.nominalSource = nominalSource;
        problem.metadata.fallbackUsed = fallbackUsed;
        problem.metadata.scheduleShifted = prediction.scheduleShifted;
        problem.metadata.targetContinuationShifted = certificateCompatible;
        planningProblem = problem;
    end
end

function identity = localEpisodeIdentity(model)
    identity = struct( ...
        "targetKey", model.targetKey, ...
        "targetGeometry", [model.targetHalfLength; ...
            model.targetHalfWidth], ...
        "routeBranchId", string(model.road.routeBranchId), ...
        "lane", model.lane, ...
        "configuration", model.cfg);
end

function compatible = localCertificateCompatible(certificate, identity, model)
    compatible = ~isempty(certificate) ...
        && isequaln(certificate.episodeIdentity, identity);
    if ~compatible
        return;
    end
    tolerance = model.cfg.controller.shiftConsistencyTolerance;
    compatible = size(certificate.predictedState, 2) >= 2 ...
        && localNumericallyEqual(certificate.predictedState(:, 2), ...
            model.initialEgoState, tolerance) ...
        && localTargetShiftMatches(certificate.targetHorizon, ...
            model.targetHorizon, tolerance);
    held = model.heldActuatorInput(:);
    if compatible && numel(held) == model.inputDimension
        compatible = localNumericallyEqual( ...
            certificate.appliedInput, held, tolerance);
    end
end

function matches = localTargetShiftMatches(previous, fresh, tolerance)
    names = ["targetPosition", "targetYaw", ...
        "targetPositionErrorBound", "targetYawErrorBound"];
    matches = isstruct(previous) && isstruct(fresh);
    if ~matches
        return;
    end
    for name = names
        if ~isfield(previous, name) || ~isfield(fresh, name) ...
                || size(previous.(name), 2) ~= size(fresh.(name), 2)
            matches = false;
            return;
        end
        priorShift = previous.(name)(:, 2:end);
        freshOverlap = fresh.(name)(:, 1:end-1);
        if name == "targetYaw"
            error = atan2(sin(priorShift-freshOverlap), ...
                cos(priorShift-freshOverlap));
            matches = localNumericallyEqual(error, ...
                zeros(size(error)), tolerance);
        else
            matches = localNumericallyEqual( ...
                priorShift, freshOverlap, tolerance);
        end
        if ~matches
            return;
        end
    end
    matches = localTargetContinuationShiftMatches( ...
        previous, fresh, tolerance);
end

function matches = localTargetContinuationShiftMatches( ...
        previous, fresh, tolerance)
% The new complete predicted continuation must be contained in the old
% one in every fixed terminal support direction. Under an exact
% shift-consistent predictor, advancing the start of a future trajectory
% can only reduce its support. This check covers the part beyond the
% finite node overlap without imposing a trajectory class.
    directionName = "terminalSupportDirection";
    supportName = "terminalFuturePositionSupport";
    required = [directionName, supportName];
    matches = all(isfield(previous, required)) ...
        && all(isfield(fresh, required));
    if ~matches
        return;
    end
    previousDirection = previous.(directionName);
    freshDirection = fresh.(directionName);
    previousSupport = previous.(supportName);
    freshSupport = fresh.(supportName);
    matches = localNumericallyEqual( ...
            previousDirection, freshDirection, tolerance) ...
        && isequal(size(previousSupport), size(freshSupport)) ...
        && isreal(previousSupport) && isreal(freshSupport) ...
        && ~any(isnan(previousSupport), "all") ...
        && ~any(isnan(freshSupport), "all") ...
        && ~any(isinf(previousSupport) & previousSupport < 0.0, "all") ...
        && ~any(isinf(freshSupport) & freshSupport < 0.0, "all");
    if ~matches
        return;
    end
    finitePrevious = isfinite(previousSupport);
    matches = all(isfinite(freshSupport(finitePrevious)), "all");
    if ~matches
        return;
    end
    scale = 1.0+max(abs(previousSupport(finitePrevious)), ...
        abs(freshSupport(finitePrevious)));
    matches = all(freshSupport(finitePrevious) ...
        <= previousSupport(finitePrevious)+tolerance*scale, "all");
end

function equal = localNumericallyEqual(first, second, tolerance)
    if ~isnumeric(first) || ~isnumeric(second) ...
            || ~isequal(size(first), size(second)) ...
            || any(~isfinite(first), "all") ...
            || any(~isfinite(second), "all")
        equal = false;
        return;
    end
    scale = 1.0+max(abs(first), abs(second));
    equal = all(abs(first-second) <= tolerance*scale, "all");
end

function horizon = localShiftTargetHorizon(previous, fresh)
    horizon = fresh;
    names = ["targetPosition", "targetYaw", ...
        "targetPositionErrorBound", "targetYawErrorBound"];
    for name = names
        if ~isfield(previous, name) || ~isfield(fresh, name) ...
                || size(previous.(name), 2) ~= size(fresh.(name), 2)
            return;
        end
    end
    horizon.targetPosition = [previous.targetPosition(:, 2:end), ...
        fresh.targetPosition(:, end)];
    horizon.targetYaw = [previous.targetYaw(2:end), fresh.targetYaw(end)];
    horizon.targetPositionErrorBound = [ ...
        previous.targetPositionErrorBound(:, 2:end), ...
        fresh.targetPositionErrorBound(:, end)];
    horizon.targetYawErrorBound = [previous.targetYawErrorBound(2:end), ...
        fresh.targetYawErrorBound(end)];
end

function schedule = localShiftSchedule(previous, model)
    schedule = [];
    nodeCount = model.horizonSteps+model.tailSteps+1;
    required = ["speed", "station", "speedProfile", "curvature", ...
        "tailAcceleration"];
    if ~isstruct(previous) || ~isscalar(previous) ...
            || ~all(isfield(previous, required)) ...
            || numel(previous.station) ~= nodeCount ...
            || numel(previous.speedProfile) ~= nodeCount ...
            || numel(previous.curvature) ~= nodeCount
        return;
    end
    station = [previous.station(2:end), previous.station(end)];
    speedProfile = [previous.speedProfile(2:end), 0.0];
    curvature = [previous.curvature(2:end), ...
        laneCurvatureAtStation(station(end), model.lane)];
    tailAcceleration = previous.tailAcceleration;
    if ~isempty(tailAcceleration)
        tailAcceleration = [tailAcceleration(2:end), 0.0];
    end
    schedule = struct( ...
        "speed", previous.speed, ...
        "station", station, ...
        "speedProfile", speedProfile, ...
        "curvature", curvature, ...
        "tailAcceleration", tailAcceleration);
end

function stored = localStoredCertificate(plan, prediction, model)
    predictedState = localPredictedState(prediction, plan);
    stored = struct( ...
        "plan", plan, ...
        "schedule", prediction.scheduleForStore, ...
        "targetHorizon", model.targetHorizon, ...
        "episodeIdentity", model.episodeIdentity, ...
        "predictedState", predictedState, ...
        "appliedInput", plan(1:model.inputDimension), ...
        "terminalLateralReference", ...
            predictedState(2, prediction.headNodeCount));
end

function [inputPlan, problem, plan] = localStoredFallback( ...
        shiftedPlan, prediction, exception, model)
    plan = shiftedPlan;
    qp = formulateAvoidanceProblem( ...
        model, prediction, plan, "verifiedStoredPlan");
    layout = qp.layout;
    decision = zeros(layout.decisionCount, 1);
    decision(layout.planIndex) = plan;
    exactResidual = localClfExactResiduals(qp.clf, plan);
    decision(layout.relaxationIndex(1)) = max(exactResidual(1), 0.0);
    tolerance = model.cfg.solver.constraintTolerance;
    boundViolation = max([0.0; qp.lowerBound-decision; ...
        decision-qp.upperBound]);
    rowViolation = max([0.0; ...
        qp.inequalityMatrix*decision-qp.inequalityBound]);
    if boundViolation > 10.0*tolerance || rowViolation > 10.0*tolerance
        error("collisionAvoidanceController:invalidStoredCertificate", ...
            "The shifted stored plan failed re-certification (bound " ...
            + "violation %.3g, row violation %.3g) after %s.", ...
            boundViolation, rowViolation, exception.identifier);
    end
    inputPlan = reshape(plan(1:prediction.inputCount), ...
        model.inputDimension, []);
    result = struct("exitFlag", 1, ...
        "iterations", 0, ...
        "algorithm", "verified stored incumbent", ...
        "message", "No optimization used; all shifted rows were checked.");
    metadata = localPlanDiagnostics( ...
        qp, result, decision, model, prediction);
    metadata.fallbackUsed = true;
    metadata.fallbackReasonIdentifier = string(exception.identifier);
    metadata.fallbackReason = string(exception.message);
    metadata.certificateSource = "shiftedStoredPlan";
    problem = struct( ...
        "problemClass", "verifiedStoredCertifiedBackupPlan", ...
        "qp", qp, ...
        "layout", layout, ...
        "prediction", prediction, ...
        "nominalInput", shiftedPlan, ...
        "decision", decision, ...
        "inputPlan", inputPlan, ...
        "plan", plan, ...
        "tailPlan", plan(prediction.tailIndex), ...
        "metadata", metadata);
    [verified, planCertificate] = localFallbackCertificate( ...
        problem, plan, model, prediction);
    if ~verified
        error("collisionAvoidanceController:invalidStoredCertificate", ...
            "The shifted stored plan failed its exact commit certificate " ...
            + "(%s) after %s.", ...
            strjoin(planCertificate.failedConditions, ", "), ...
            exception.identifier);
    end
    problem.metadata.planCertified = true;
    problem.metadata.exactPredictionAssumptionsHold = ...
        planCertificate.exactPredictionAssumptionsHold;
    problem.metadata.nodeClearanceMargin = ...
        planCertificate.nodeClearanceMargin;
    problem.metadata.routeCoordinateValid = ...
        planCertificate.routeCoordinateValid;
    problem.metadata.postSolveCertificationPerformed = true;
end

function [certified, certificate] = localFallbackCertificate( ...
        problem, plan, model, prediction)
    tolerance = model.cfg.solver.constraintTolerance;
    certificate = struct();
    certificate.exactPredictionAssumptionsHold = ...
        localExactPredictionAssumptionsHold(model);
    certificate.nodeClearanceMargin = inf;
    state = localPredictedState(prediction, plan);
    routeEnd = model.lane.segmentStation(end) ...
        + model.lane.segmentLength(end);
    certificate.routeCoordinateValid = all(state(1, :) >= -tolerance) ...
        && all(state(1, :) <= routeEnd+tolerance);
    if model.hasTarget
        egoPose = localFrenetTrajectoryToCartesian( ...
            state(1:3, :), model.lane);
        targetPose = [model.targetHorizon.targetPosition( ...
                :, 1:prediction.nodeCount); ...
            model.targetHorizon.targetYaw(1:prediction.nodeCount)];
        halfDimensions = [model.egoHalfLength; model.egoHalfWidth; ...
            model.targetHalfLength; model.targetHalfWidth];
        egoRadius = hypot(model.egoHalfLength, model.egoHalfWidth);
        terminal = problem.qp.terminal;
        tailEnvelope = terminal.lateralDrift ...
            + 2.0*egoRadius*terminal.headingRadius;
        nodeClearance = repmat( ...
            model.cfg.collision.clearanceMargin, 1, prediction.nodeCount);
        nodeClearance(prediction.tailNodeIndex) = ...
            nodeClearance(prediction.tailNodeIndex)+tailEnvelope;
        nodeMargin = inf(1, prediction.nodeCount);
        for nodeIdx = 1:prediction.nodeCount
            nodeMargin(nodeIdx) = rectangleConfigurationDistance( ...
                egoPose(1:2, nodeIdx), egoPose(3, nodeIdx), ...
                targetPose(1:2, nodeIdx), targetPose(3, nodeIdx), ...
                halfDimensions)-nodeClearance(nodeIdx);
        end
        certificate.nodeClearanceMargin = min(nodeMargin);
    end
    conditionName = [ ...
        "exactPrediction", "routeCoordinate", "hardCbf", ...
        "terminalInvariant", "nodeClearance"];
    conditionHolds = [ ...
        certificate.exactPredictionAssumptionsHold, ...
        certificate.routeCoordinateValid, ...
        problem.metadata.hardCbfSatisfied, ...
        problem.metadata.terminalInvariantCertified, ...
        certificate.nodeClearanceMargin >= -10.0*tolerance];
    certificate.failedConditions = conditionName(~conditionHolds);
    certified = all(conditionHolds);
end

function holds = localExactPredictionAssumptionsHold(model)
    holds = all(model.measuredEgoStateErrorBound == 0.0) ...
        && all(model.cfg.model.ltvModelErrorRateBound == 0.0) ...
        && all(model.cfg.model.plantModelResidualRateBound == 0.0);
    if ~model.hasTarget
        return;
    end
    holds = holds ...
        && all(model.targetPositionErrorBound == 0.0) ...
        && all(model.targetVelocityErrorBound == 0.0) ...
        && all(model.targetPredictionAccelerationErrorBound == 0.0) ...
        && model.targetYawErrorBound == 0.0 ...
        && model.targetYawRateErrorBound == 0.0 ...
        && model.targetPredictionYawAccelerationErrorBound == 0.0;
end

function pose = localFrenetTrajectoryToCartesian(frenetPose, lane)
    nodeCount = size(frenetPose, 2);
    pose = zeros(3, nodeCount);
    for nodeIdx = 1:nodeCount
        station = frenetPose(1, nodeIdx);
        segmentIdx = find(lane.segmentStation <= station, 1, "last");
        if isempty(segmentIdx)
            segmentIdx = 1;
        end
        segmentIdx = min(segmentIdx, numel(lane.segmentLength));
        fraction = (station-lane.segmentStation(segmentIdx)) ...
            / lane.segmentLength(segmentIdx);
        fraction = min(max(fraction, 0.0), 1.0);
        tangent = lane.tangent(segmentIdx, :).';
        normal = [-tangent(2); tangent(1)];
        centre = lane.segmentStart(segmentIdx, :).' ...
            + fraction*lane.segment(segmentIdx, :).' ...
            + normal*frenetPose(2, nodeIdx);
        pose(1:2, nodeIdx) = centre;
        pose(3, nodeIdx) = atan2(tangent(2), tangent(1)) ...
            + frenetPose(3, nodeIdx);
    end
end

% ====================================================================
% Nominal and the solve
% ====================================================================

function [nominalInput, source] = localNominalInput( ...
        previousPlan, prediction, model, certificate)
% The linearization nominal, a PLAN vector (head inputs then tail
% accelerations): advance the head, append terminal steering with the
% tail's first acceleration, and advance the tail with zero appended -
% or the schedule reference with its braking tail. The shift is the
% backup candidate: under the declared model it is a feasible
% point of this sample's program.
    if isnumeric(previousPlan) && isreal(previousPlan) ...
            && numel(previousPlan) == prediction.planCount ...
            && all(isfinite(previousPlan), "all")
        inputPlan = reshape(previousPlan(1:prediction.inputCount), ...
            model.inputDimension, model.horizonSteps);
        tail = previousPlan(prediction.tailIndex);
        shiftedInput = [inputPlan(:, 2:end), ...
            [inputPlan(1, end); tail(1)]];
        state = model.initialEgoState;
        for stageIdx = 1:model.horizonSteps-1
            state = prediction.stageMatrixA(:, :, stageIdx)*state ...
                + prediction.stageMatrixB(:, :, stageIdx) ...
                    * shiftedInput(:, stageIdx) ...
                + prediction.stageAffine(:, stageIdx);
        end
        shiftedInput(1, end) = localTerminalBackupSteering( ...
            state, certificate.terminalLateralReference, ...
            prediction.scheduleCurvature(model.horizonSteps), model.cfg);
        shiftedTail = [tail(2:end); 0.0];
        nominalInput = [shiftedInput(:); shiftedTail(:)];
        source = "shiftedPreviousPlan";
    else
        nominalInput = prediction.referencePlan;
        source = "scheduleReference";
    end
end

function steering = localTerminalBackupSteering( ...
        state, lateralReference, curvature, cfg)
% The feedback law certified by terminalLateralCertificate, evaluated
% when the first stage of the proof-side tail enters the executable head.
    omega = cfg.terminal.lateralBandwidth;
    lateralError = state(2)-lateralReference;
    steering = cfg.vehicle.wheelbase*(curvature ...
        - omega^2*lateralError-2.0*omega*state(3));
    limit = cfg.model.frontWheelSteeringAngleMaximum;
    steering = min(max(steering, -limit), limit);
end

function state = localPredictedState(prediction, plan)
    state = zeros(6, prediction.nodeCount);
    for nodeIdx = 1:prediction.nodeCount
        state(:, nodeIdx) = prediction.egoStateMatrix(:, :, nodeIdx)*plan ...
            + prediction.egoStateOffset(:, nodeIdx);
    end
end

function plan = localCommittedPlan(decision, qp)
% The committed plan: the returned decision's plan columns clipped into
% their bounds (the kernel satisfies bounds to its tolerance).
    planIndex = qp.layout.planIndex;
    plan = min(max(decision(planIndex), qp.lowerBound(planIndex)), ...
        qp.upperBound(planIndex));
end

function inputPlan = localHeadInputPlan(plan, layout)
    inputPlan = reshape(plan(layout.inputIndex), ...
        layout.inputDimension, layout.horizonSteps);
end

function [inputPlan, problem, plan] = localSolve( ...
        model, prediction, nominalInput)
% Solve the same hard-CBF/soft-CLF problem at each prescribed start.
    cfg = model.cfg;
    [firstQp, common] = formulateAvoidanceProblem( ...
        model, prediction, nominalInput, "shiftedPlan");
    startInput = {nominalInput};
    labels = "shiftedPlan";
    if any([firstQp.collision.nodes.imposed])
        startInput{end+1} = common.probeInput;
        labels(end+1) = "cruiseProbe";
    end

    candidateCount = numel(startInput);
    qps = cell(1, candidateCount);
    results = cell(1, candidateCount);
    startViolations = zeros(1, candidateCount);
    solverCalls = 0;
    for candidateIdx = 1:candidateCount
        if candidateIdx == 1
            qps{candidateIdx} = firstQp;
        else
            qps{candidateIdx} = formulateAvoidanceProblem( ...
                model, prediction, startInput{candidateIdx}, ...
                labels(candidateIdx), common);
        end
        startViolations(candidateIdx) = ...
            localStartViolation(qps{candidateIdx});
        results{candidateIdx} = solveHardCbfClf(qps{candidateIdx}, cfg);
        solverCalls = solverCalls+results{candidateIdx}.solverCalls;
    end

    exitFlags = zeros(1, candidateCount);
    objectives = inf(1, candidateCount);
    clfValues = inf(1, candidateCount);
    certifiedInfeasible = false(1, candidateCount);
    messages = strings(1, candidateCount);
    bestIdx = 0;
    for candidateIdx = 1:candidateCount
        candidate = results{candidateIdx};
        exitFlags(candidateIdx) = candidate.exitFlag;
        certifiedInfeasible(candidateIdx) = ...
            qps{candidateIdx}.certifiedInfeasible;
        messages(candidateIdx) = candidate.message;
        if candidate.feasible
            clfValues(candidateIdx) = candidate.clfValue;
            objectives(candidateIdx) = candidate.objectiveValue;
            if bestIdx == 0 || localObjectiveBetter( ...
                    objectives(candidateIdx), objectives(bestIdx), cfg)
                bestIdx = candidateIdx;
            end
        end
    end
    if bestIdx == 0
        if all(exitFlags == -2)
            error("collisionAvoidanceController:noSolution", ...
                "The hard-CBF/soft-CLF problem is infeasible from every " ...
                + "start (%s). No command is issued.", strjoin(labels, ", "));
        end
        error("collisionAvoidanceController:optimizationFailure", ...
            "The conic solver returned no plan (%s).", ...
            strjoin(labels+": "+messages, "; "));
    end

    qp = qps{bestIdx};
    result = results{bestIdx};
    decision = result.decision;
    layout = qp.layout;
    plan = localCommittedPlan(decision, qp);
    inputPlan = localHeadInputPlan(plan, layout);
    problem = struct( ...
        "problemClass", qp.problemClass, "qp", qp, "layout", layout, ...
        "prediction", prediction, "nominalInput", nominalInput, ...
        "decision", decision, "inputPlan", inputPlan, "plan", plan, ...
        "tailPlan", plan(layout.tailIndex), ...
        "metadata", localPlanDiagnostics(qp, result, decision, model, prediction));
    problem.metadata.candidateCount = candidateCount;
    problem.metadata.candidateLabels = labels;
    problem.metadata.candidateExitFlags = exitFlags;
    problem.metadata.candidateObjectives = objectives;
    problem.metadata.candidateClfValues = clfValues;
    problem.metadata.candidateCertifiedInfeasible = certifiedInfeasible;
    problem.metadata.candidateStartViolation = startViolations;
    problem.metadata.candidatesSolved = sum(~certifiedInfeasible);
    problem.metadata.selectedCandidate = labels(bestIdx);
    problem.metadata.solverCallCount = solverCalls;
end

function better = localObjectiveBetter(candidate, incumbent, cfg)
    tolerance = cfg.solver.optimalityTolerance;
    better = candidate ...
        < incumbent-tolerance*(1.0+abs(incumbent));
end

function violation = localStartViolation(qp)
% How far the linearization itself is from separation: the largest
% shortfall of its own stage-1 margins over the nodes that carry rows.
% Zero means the trajectory is separated from the obstacle by the
% declared budgets, and is therefore a feasible point of the hard
% program built at it.
    violation = 0.0;
    nodes = qp.collision.nodes;
    for nodeIdx = 1:numel(nodes)
        if nodes(nodeIdx).imposed
            violation = max(violation, -nodes(nodeIdx).nominalMargin);
        end
    end
end

function [residual, valueProfile, planError] = ...
        localClfExactResiduals(clf, planColumn)
% The exact (unlinearized) CLF rows of a plan,
% V(e_{k+1}) - V(e_k) + f W(e_k), k = 0..N-1, with the Lyapunov value
% and the error at every node.
    nodeCount = size(clf.errorOffset, 2);
    planError = zeros(size(clf.errorOffset));
    valueProfile = zeros(1, nodeCount);
    for nodeIdx = 1:nodeCount
        planError(:, nodeIdx) = clf.errorMatrix(:, :, nodeIdx) ...
            * planColumn+clf.errorOffset(:, nodeIdx);
        valueProfile(nodeIdx) = planError(:, nodeIdx).' ...
            * clf.lyapunovMatrix*planError(:, nodeIdx);
    end
    residual = zeros(1, nodeCount-1);
    for rowIdx = 1:nodeCount-1
        currentError = planError(:, rowIdx);
        residual(rowIdx) = valueProfile(rowIdx+1)-valueProfile(rowIdx) ...
            + currentError.'*clf.decreaseMatrix*currentError;
    end
end

function nodeIdx = localFirstImposedNode(family)
    nodeIdx = 0;
    imposed = find([family.nodes.imposed], 1);
    if ~isempty(imposed)
        nodeIdx = imposed-1;
    end
end

function metadata = localPlanDiagnostics( ...
        qp, result, decision, model, prediction)
% What is reported about the committed plan: the stage-1 duals along
% its linearization trajectory, the plan's linearized separation
% margins, the exact CLF relaxation,
% the objective split, and the kernel's verdict.
    layout = qp.layout;
    metadata = struct();
    metadata.problemClass = qp.problemClass;
    metadata.collisionDiscretization = "predictionNodesOnly";
    metadata.horizonSteps = layout.horizonSteps;
    metadata.tailSteps = layout.tailSteps;
    planColumn = decision(layout.planIndex);
    headNodeCount = prediction.headNodeCount;

    % Every covered target node carries hard separation rows.
    collision = localFamilyReadout(qp.collision, planColumn);
    nodes = qp.collision.nodes;
    metadata.collisionMarginProfile = collision.margin;
    metadata.collisionImposedProfile = collision.imposed;
    metadata.collisionMargin = collision.margin(1);
    metadata.collisionPlanMargin = inf;
    metadata.collisionTailMargin = inf;
    metadata.collisionClosestNode = 0;
    metadata.collisionClosestRegion = 0;
    metadata.collisionActive = false;
    metadata.collisionImposedFrom = localFirstImposedNode(qp.collision);
    metadata.collisionImposedCount = sum(collision.imposed);
    if any(collision.imposed)
        imposedMargin = collision.margin;
        imposedMargin(~collision.imposed) = inf;
        [metadata.collisionPlanMargin, closestIdx] = min(imposedMargin);
        metadata.collisionClosestNode = closestIdx-1;
        metadata.collisionClosestRegion = nodes(closestIdx).regionCode;
        metadata.collisionActive = metadata.collisionPlanMargin <= 1.0e-6;
        metadata.collisionTailMargin = ...
            min(imposedMargin(headNodeCount+1:end));
    end

    % The terminal set: the committed tail, the terminal handoff state
    % against its rows, and the hard target-conditioned separation from
    % the controller's complete predicted continuation.
    metadata = localTerminalDiagnostics(metadata, qp, planColumn, ...
        model, prediction);
    metadata.dualDistanceProfile = [nodes.dualDistance];
    metadata.dualRegionProfile = [nodes.regionCode];
    metadata.dualNormalProfile = [nodes.normal];
    metadata.nominalMarginProfile = [nodes.nominalMargin];
    if isempty(nodes)
        metadata.dualDistanceProfile = zeros(1, 0);
        metadata.dualRegionProfile = zeros(1, 0);
        metadata.dualNormalProfile = zeros(2, 0);
        metadata.nominalMarginProfile = zeros(1, 0);
    end

    metadata.measuredClearance = inf;
    if model.hasTarget
        horizon = model.targetHorizon;
        state = model.initialCartesianState;
        metadata.measuredClearance = rectangleConfigurationDistance( ...
            state(1:2), state(3), horizon.targetPosition(:, 1), ...
            horizon.targetYaw(1), [model.egoHalfLength; ...
                model.egoHalfWidth; model.targetHalfLength; ...
                model.targetHalfWidth]);
    end

    % Road boundaries.
    roadMargin = inf;
    roadProfiles = cell(1, numel(qp.road));
    for boundaryIdx = 1:numel(qp.road)
        readout = localFamilyReadout(qp.road(boundaryIdx), planColumn);
        roadProfiles{boundaryIdx} = readout.margin;
        roadMargin = min(roadMargin, min(readout.margin));
    end
    metadata.roadMargin = roadMargin;
    metadata.roadMarginProfiles = roadProfiles;

    % Row residuals of the committed decision.
    residual = qp.inequalityMatrix*decision-qp.inequalityBound;
    metadata.hardRowViolation = max([0.0; residual]);
    metadata.rowCounts = struct( ...
        "collision", sum(qp.rowFamily == "collision"), ...
        "road", sum(qp.rowFamily == "road"), ...
        "speedDomain", sum(qp.rowFamily == "speedDomain"), ...
        "headingDomain", sum(qp.rowFamily == "headingDomain"), ...
        "friction", sum(qp.rowFamily == "friction"), ...
        "tail", sum(qp.rowFamily == "tail"), ...
        "terminal", sum(qp.rowFamily == "terminal"), ...
        "terminalSegment", sum(qp.rowFamily == "terminalSegment"));

    % CLF relaxations and the exact decrease residual of the plan.
    relaxation = decision(layout.relaxationIndex);
    metadata.clfRelaxation = relaxation;
    metadata.clfInitialValue = qp.clf.initialValue;
    [exactResidual, valueProfile, planError] = ...
        localClfExactResiduals(qp.clf, planColumn);
    metadata.clfValueProfile = valueProfile;
    metadata.clfExactResidual = exactResidual(1)-relaxation;
    metadata.clfPlanError = planError;
    metadata.cbfConstraintsHard = true;
    metadata.cbfMinimumMargin = min( ...
        metadata.collisionPlanMargin, metadata.roadMargin);
    metadata.hardCbfSatisfied = metadata.cbfConstraintsHard ...
        && metadata.hardRowViolation ...
            <= 10.0*model.cfg.solver.constraintTolerance ...
        && metadata.cbfMinimumMargin ...
            >= -10.0*model.cfg.solver.constraintTolerance;

    % Objective split.
    planHessian = qp.Hessian(layout.planIndex, layout.planIndex);
    metadata.inputDeviationCost = 0.5*planColumn.'*planHessian ...
        * planColumn+qp.linear(layout.planIndex).'*planColumn ...
        + qp.constant;
    metadata.clfRelaxationCost = ...
        qp.linear(layout.relaxationIndex)*relaxation;
    metadata.jointObjectiveValue = metadata.inputDeviationCost ...
        + metadata.clfRelaxationCost;
    metadata.objectiveValue = metadata.jointObjectiveValue;

    % Kernel.
    metadata.solverExitFlag = result.exitFlag;
    metadata.solverIterations = result.iterations;
    metadata.solverAlgorithm = result.algorithm;
    metadata.solverMessage = result.message;
    metadata.scheduleSpeed = prediction.scheduleSpeed;
    metadata.plannedSpeedFloor = model.plannedSpeedFloor;
    metadata.hasTarget = model.hasTarget;
end

function readout = localFamilyReadout(family, planColumn)
% The plan's linearized separation margin min_sigma g^sigma_k(u) at
% every node of one family (Inf where the family carries no data),
% and which nodes are covered and imposed.
    nodes = family.nodes;
    nodeCount = numel(nodes);
    readout = struct("margin", inf(1, nodeCount), ...
        "covered", false(1, nodeCount), ...
        "imposed", false(1, nodeCount));
    for nodeIdx = 1:nodeCount
        node = nodes(nodeIdx);
        if ~node.covered
            continue;
        end
        readout.margin(nodeIdx) = min(node.marginMatrix*planColumn ...
            + node.marginOffset);
        readout.covered(nodeIdx) = true;
        readout.imposed(nodeIdx) = node.imposed;
    end
end

function metadata = localTerminalDiagnostics(metadata, qp, planColumn, ...
        model, prediction)
% The terminal set's readout: the committed tail's speed and station
% profiles, the terminal handoff state against its rows, and the hard
% augmented target-continuation row at the resting node.
    layout = qp.layout;
    terminal = qp.terminal;
    terminalNode = prediction.headNodeCount;
    restNode = prediction.nodeCount;
    planState = zeros(6, prediction.nodeCount);
    for nodeIdx = 1:prediction.nodeCount
        planState(:, nodeIdx) = prediction.egoStateMatrix(:, :, nodeIdx) ...
            * planColumn+prediction.egoStateOffset(:, nodeIdx);
    end
    terminalState = planState(:, terminalNode);
    curvature = prediction.scheduleCurvature(terminalNode);
    metadata.tailAccelerationPlan = planColumn(layout.tailIndex).';
    metadata.tailSpeedProfile = planState(4, prediction.tailNodeIndex);
    metadata.tailStationProfile = planState(1, prediction.tailNodeIndex);
    metadata.restStation = planState(1, restNode);
    metadata.terminalSpeed = terminalState(4);
    metadata.terminalHeadingError = terminalState(3);
    metadata.terminalHeadingBound = terminal.headingRow;
    metadata.terminalLateralVelocity = terminalState(5);
    metadata.terminalLateralVelocityBound = terminal.lateralVelocityRow;
    metadata.terminalYawRateError = terminalState(6) ...
        - curvature*terminalState(4);
    metadata.terminalYawRateErrorBound = terminal.yawRateRow;
    metadata.terminalHeadingRadius = terminal.headingRadius;
    metadata.terminalLateralDrift = terminal.lateralDrift;
    metadata.terminalInvariantCertified = true;
    metadata.terminalInvariantMargin = inf;
    metadata.terminalContinuationAxis = "none";
    metadata.terminalFutureSupport = inf;
    metadata.terminalSupportDirection = zeros(2, 1);
    metadata.terminalSegmentIndex = 0;
    if ~model.hasTarget
        return;
    end
    node = qp.collision.nodes(restNode);
    metadata.terminalInvariantMargin = min( ...
        node.marginMatrix*planColumn+node.marginOffset);
    metadata.terminalInvariantCertified = node.terminalInvariant ...
        && metadata.terminalInvariantMargin ...
            >= -model.cfg.solver.constraintTolerance;
    metadata.terminalContinuationAxis = node.terminalContinuationAxis;
    metadata.terminalFutureSupport = node.terminalFutureSupport;
    metadata.terminalSupportDirection = node.terminalSupportDirection;
    metadata.terminalSegmentIndex = node.terminalSegmentIndex;
end

% ====================================================================
% Configuration and the prediction model
% ====================================================================

function cfg = localControllerConfiguration(userCfg)
    persistent cachedUserConfiguration cachedConfiguration cacheValid
    if isempty(cacheValid)
        cacheValid = false;
    end
    if nargin < 1 || isempty(userCfg)
        userCfg = [];
    end
    if cacheValid && isequaln(userCfg, cachedUserConfiguration)
        cfg = cachedConfiguration;
        return;
    end
    localAddConfigurationPath();
    cfg = collisionAvoidanceControllerConfig(userCfg);
    cachedUserConfiguration = userCfg;
    cachedConfiguration = cfg;
    cacheValid = true;
end

function localAddConfigurationPath()
    if exist("collisionAvoidanceControllerConfig", "file") == 2
        return;
    end
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    configurationRoot = fullfile(repositoryRoot, "config");
    if isfolder(configurationRoot)
        addpath(configurationRoot);
    end
end

function model = localPredictionModel(ego, targets, lane, road, cfg)
    model = localStaticPredictionModel(cfg);
    % The measured state in PATH COORDINATES [s; d; ePsi; vx; vy; r]:
    % the projection of the measured position onto the lane and the
    % heading error to the path tangent; the Cartesian state is kept
    % for the physical clearance readout.
    projection = laneProjection(ego.position, lane);
    headingError = atan2(sin(ego.yaw-projection.heading), ...
        cos(ego.yaw-projection.heading));
    model.initialCartesianState = ego.modelState;
    model.initialEgoState = [projection.station; ...
        projection.lateralPosition; headingError; ego.modelState(4:6)];
    model.measuredEgoStateErrorBound = ego.stateErrorBound;
    % Declared longitudinal model bias published by the estimator: the
    % offset-free disturbance term of Ge et al. (2022), entering the
    % prediction's vx row at every stage. An input, not controller
    % state.
    model.longitudinalAccelerationBias = ...
        ego.longitudinalAccelerationBias;
    % The measured actuator position, when published, is retained as
    % execution feedback for stored-certificate compatibility checks.
    model.heldActuatorInput = zeros(0, 1);
    if isfield(ego, "heldActuatorInput")
        model.heldActuatorInput = ego.heldActuatorInput(:);
    end
    % The planned speed floor: the declared planned minimum, lowered
    % to the worst-case measured speed so a slow start is solvable.
    model.plannedSpeedFloor = min(cfg.model.plannedSpeedMinimum, ...
        max(ego.modelState(4)-ego.stateErrorBound(4), ...
            cfg.model.speedMinimum));
    model.lane = lane;
    model.road = road;
    % The reader admits at most one target. Empty-target geometry uses
    % neutral values; target constraints are then absent.
    model.hasTarget = ~isempty(targets);
    model.targetKey = "";
    model.targetPosition = zeros(2, 1);
    model.targetSpeed = 0.0;
    model.targetCourseDirection = zeros(2, 1);
    model.targetCurvature = 0.0;
    model.targetTangentialAcceleration = 0.0;
    model.targetStopTime = inf;
    model.targetYaw = 0.0;
    model.targetHalfLength = 0.0;
    model.targetHalfWidth = 0.0;
    model.targetPositionErrorBound = zeros(2, 1);
    model.targetVelocityErrorBound = zeros(2, 1);
    model.targetPredictionAccelerationErrorBound = zeros(2, 1);
    model.targetYawErrorBound = 0.0;
    model.targetYawRateErrorBound = 0.0;
    model.targetPredictionYawAccelerationErrorBound = 0.0;
    model.targetPrediction = struct();
    if model.hasTarget
        target = targets(1);
        model.targetKey = target.key;
        model.targetPosition = target.position;
        model.targetYaw = target.yaw;
        targetSpeed = norm(target.velocity);
        accelerationNorm = norm(target.acceleration);
        motionTolerance = 100.0*eps(max( ...
            [1.0, targetSpeed, accelerationNorm]));
        if targetSpeed > motionTolerance
            courseDirection = target.velocity/targetSpeed;
            curvature = target.yawRate/targetSpeed;
        elseif accelerationNorm > motionTolerance
            courseDirection = target.acceleration/accelerationNorm;
            curvature = 0.0;
        else
            courseDirection = [cos(target.yaw); sin(target.yaw)];
            curvature = 0.0;
        end
        tangentialAcceleration = dot(target.acceleration, courseDirection);
        courseNormal = [-courseDirection(2); courseDirection(1)];
        nominalAcceleration = tangentialAcceleration*courseDirection ...
            + courseNormal*(curvature*targetSpeed^2);
        accelerationResidual = target.acceleration-nominalAcceleration;
        yawRateResidual = target.yawRate-curvature*targetSpeed;
        reconstructionTolerance = 100.0*eps(max([1.0, ...
            norm(target.acceleration), norm(nominalAcceleration), ...
            abs(target.yawRate), abs(curvature*targetSpeed)]));
        accelerationResidual( ...
            abs(accelerationResidual) <= reconstructionTolerance) = 0.0;
        if abs(yawRateResidual) <= reconstructionTolerance
            yawRateResidual = 0.0;
        end
        model.targetSpeed = targetSpeed;
        model.targetCourseDirection = courseDirection;
        model.targetCurvature = curvature;
        model.targetTangentialAcceleration = tangentialAcceleration;
        if tangentialAcceleration < 0.0
            model.targetStopTime = targetSpeed/-tangentialAcceleration;
        end
        model.targetHalfLength = 0.5*target.length;
        model.targetHalfWidth = 0.5*target.width;
        model.targetPositionErrorBound = target.positionErrorBound;
        model.targetVelocityErrorBound = target.velocityErrorBound;
        model.targetPredictionAccelerationErrorBound = ...
            target.predictionAccelerationErrorBound ...
            + abs(accelerationResidual);
        model.targetYawErrorBound = target.yawErrorBound;
        model.targetYawRateErrorBound = target.yawRateErrorBound ...
            + abs(yawRateResidual);
        model.targetPredictionYawAccelerationErrorBound = ...
            target.predictionYawAccelerationErrorBound;
        model.targetPrediction = struct( ...
            "initialPosition", model.targetPosition, ...
            "initialCourseDirection", model.targetCourseDirection, ...
            "initialSpeed", model.targetSpeed, ...
            "tangentialAcceleration", ...
                model.targetTangentialAcceleration, ...
            "curvature", model.targetCurvature, ...
            "stopTime", model.targetStopTime);
    end
    model.targetHorizon = localTargetHorizon(model);
    model.targetPath = localTargetPath(model);
end

function path = localTargetPath(model)
% The target's predicted box in path coordinates at every node of the
% head and the tail, and one node beyond for the next shift check: the
% projected station and lateral offset of its centre, its heading
% error to the path, and the half-extents of the axis-aligned bound of
% its rotated rectangle in the path frame.
    nodeCount = model.horizonSteps+model.tailSteps+2;
    path = struct("station", zeros(1, nodeCount), ...
        "lateral", zeros(1, nodeCount), ...
        "headingError", zeros(1, nodeCount), ...
        "halfLength", zeros(1, nodeCount), ...
        "halfWidth", zeros(1, nodeCount));
    if ~model.hasTarget
        return;
    end
    horizon = model.targetHorizon;
    for nodeIdx = 1:nodeCount
        projection = laneProjection( ...
            horizon.targetPosition(:, nodeIdx), model.lane);
        headingError = atan2( ...
            sin(horizon.targetYaw(nodeIdx)-projection.heading), ...
            cos(horizon.targetYaw(nodeIdx)-projection.heading));
        path.station(nodeIdx) = projection.station;
        path.lateral(nodeIdx) = projection.lateralPosition;
        path.headingError(nodeIdx) = headingError;
        path.halfLength(nodeIdx) = ...
            model.targetHalfLength*abs(cos(headingError)) ...
            + model.targetHalfWidth*abs(sin(headingError));
        path.halfWidth(nodeIdx) = ...
            model.targetHalfLength*abs(sin(headingError)) ...
            + model.targetHalfWidth*abs(cos(headingError));
    end
end

function model = localStaticPredictionModel(cfg)
    persistent cachedConfiguration cachedModel
    if ~isempty(cachedModel) && isequaln(cfg, cachedConfiguration)
        model = cachedModel;
        return;
    end
    model = struct();
    model.egoHalfLength = 0.5*cfg.vehicle.length;
    model.egoHalfWidth = 0.5*cfg.vehicle.width;
    model.inputDimension = 2;
    model.sampleTime = cfg.controller.sampleTime;
    model.horizonSteps = cfg.controller.horizonSteps;
    % The braking tail's length: derived from the actuator and the
    % speed domain, never declared (kinematicBrakingTail).
    model.tailSteps = kinematicBrakingTail("steps", cfg);
    model.referenceSpeed = cfg.referenceSpeed;
    model.cfg = cfg;
    cachedConfiguration = cfg;
    cachedModel = model;
end

function horizon = localTargetHorizon(model)
% The target's predicted pose and published uncertainty at every finite
% node, plus the exact support of the controller's complete continuation
% from the rest time. The terminal support interface carries every future
% point and therefore needs no stationary/straight/monotone target gate.
    nodeTime = (0:model.horizonSteps+model.tailSteps+1)*model.sampleTime;
    [position, yaw] = localTargetMotionAtTimes(nodeTime, model);
    [positionErrorBound, yawErrorBound] = ...
        localTargetPredictionErrorAtTimes(nodeTime, model);
    horizon = struct();
    horizon.targetPosition = position;
    horizon.targetYaw = yaw;
    horizon.targetPositionErrorBound = positionErrorBound;
    horizon.targetYawErrorBound = yawErrorBound;
    horizon.terminalSupportDirection = zeros(2, 0);
    horizon.terminalFuturePositionSupport = zeros(1, 0);
    horizon.terminalSupportStartTime = ...
        (model.horizonSteps+model.tailSteps)*model.sampleTime;
    if ~model.hasTarget
        return;
    end
    directionCount = ...
        model.cfg.terminal.supportDirectionCount;
    angle = (0:directionCount-1)*(2.0*pi/directionCount);
    horizon.terminalSupportDirection = [cos(angle); sin(angle)];
    horizon.terminalFuturePositionSupport = ...
        targetPredictionFutureSupport( ...
            horizon.terminalSupportDirection, ...
            horizon.terminalSupportStartTime, model.targetPrediction);
end

function [positionErrorBound, yawErrorBound] = ...
        localTargetPredictionErrorAtTimes(time, model)
    time = max(0.0, double(time(:).'));
    timeCount = numel(time);
    positionErrorBound = zeros(2, timeCount);
    yawErrorBound = zeros(1, timeCount);
    if ~model.hasTarget
        return;
    end
    positionErrorBound = model.targetPositionErrorBound ...
        + model.targetVelocityErrorBound*time ...
        + 0.5*model.targetPredictionAccelerationErrorBound*time.^2;
    yawErrorBound = model.targetYawErrorBound ...
        + model.targetYawRateErrorBound*time ...
        + 0.5*model.targetPredictionYawAccelerationErrorBound*time.^2;
end

function [position, yaw] = localTargetMotionAtTimes(time, model)
    time = max(0.0, double(time(:).'));
    timeCount = numel(time);
    position = zeros(2, timeCount);
    yaw = zeros(1, timeCount);
    if ~model.hasTarget
        return;
    end
    propagationTime = min(time, model.targetStopTime);
    arcLength = model.targetSpeed*propagationTime ...
        + 0.5*model.targetTangentialAcceleration*propagationTime.^2;
    [displacement, turnAngle] = localConstantCurvatureMotionAtTimes( ...
        arcLength, model.targetCurvature, model.targetCourseDirection);
    position = model.targetPosition+displacement;
    yaw = model.targetYaw+turnAngle;
end

function [displacement, turnAngle] = ...
        localConstantCurvatureMotionAtTimes( ...
        arcLength, curvature, initialDirection)
    turnAngle = curvature*arcLength;
    normalDirection = [-initialDirection(2); initialDirection(1)];
    parallelDisplacement = zeros(size(arcLength));
    normalDisplacement = zeros(size(arcLength));
    smallTurn = abs(turnAngle) < 1.0e-4;
    turnSquared = turnAngle(smallTurn).^2;
    parallelDisplacement(smallTurn) = arcLength(smallTurn).*( ...
        1.0-turnSquared/6.0+turnSquared.^2/120.0);
    normalDisplacement(smallTurn) = arcLength(smallTurn).*( ...
        turnAngle(smallTurn)/2.0 ...
            - turnAngle(smallTurn).^3/24.0 ...
            + turnAngle(smallTurn).^5/720.0);
    regularTurn = ~smallTurn;
    parallelDisplacement(regularTurn) = ...
        arcLength(regularTurn).*sin(turnAngle(regularTurn)) ...
            ./ turnAngle(regularTurn);
    normalDisplacement(regularTurn) = ...
        arcLength(regularTurn).*(1.0-cos(turnAngle(regularTurn))) ...
            ./ turnAngle(regularTurn);
    displacement = initialDirection*parallelDisplacement ...
        + normalDirection*normalDisplacement;
end

% ====================================================================
% The command
% ====================================================================

function command = localCommand(inputPlan, model)
    firstInput = inputPlan(:, 1);
    cfg = model.cfg;
    state = model.initialEgoState;
    forceScheduleSpeed = max(state(4), cfg.model.scheduleSpeedFloor);
    friction = axleFrictionParameters(cfg);
    steeringAngle = firstInput(1);
    longitudinalAcceleration = firstInput(2);
    % Paper Eq. (8) sideslip convention; lateral force is -C_alpha*alpha.
    frontSlipAngle = (state(5)+cfg.vehicle.lf*state(6)) ...
        / forceScheduleSpeed-steeringAngle;
    rearSlipAngle = (state(5)-cfg.vehicle.lr*state(6)) ...
        / forceScheduleSpeed;
    axleLateralForce = -friction.corneringStiffness ...
        .* [frontSlipAngle; rearSlipAngle];
    if longitudinalAcceleration >= 0.0
        longitudinalDistribution = friction.driveDistribution;
    else
        longitudinalDistribution = friction.brakeDistribution;
    end
    axleLongitudinalForce = friction.mass ...
        * longitudinalAcceleration.*longitudinalDistribution;
    axleNormalLoad = friction.staticNormalLoad ...
        + friction.normalLoadAccelerationSlope*longitudinalAcceleration;
    axleFrictionCapacity = friction.frictionCoefficient.*axleNormalLoad;
    axleFrictionUtilization = hypot( ...
        axleLongitudinalForce./axleFrictionCapacity, ...
        axleLateralForce./axleFrictionCapacity);
    command = struct();
    command.longitudinalAcceleration = longitudinalAcceleration;
    command.bodyLongitudinalVelocityDerivative = ...
        longitudinalAcceleration+state(5)*state(6);
    command.lateralAcceleration = ...
        sum(axleLateralForce)/cfg.vehicle.m-state(4)*state(6);
    command.yawAcceleration = ...
        (cfg.vehicle.lf*axleLateralForce(1) ...
            - cfg.vehicle.lr*axleLateralForce(2))/cfg.vehicle.Iz;
    command.actuatorInput = [steeringAngle; longitudinalAcceleration];
    command.actuatorInputOrder = [ ...
        "frontWheelSteeringAngle", "longitudinalAcceleration"];
    command.totalLongitudinalTireForce = sum(axleLongitudinalForce);
    command.axleLongitudinalForce = axleLongitudinalForce;
    command.axleLateralForce = axleLateralForce;
    command.axleNormalLoad = axleNormalLoad;
    command.axleFrictionCapacity = axleFrictionCapacity;
    command.axleFrictionUtilization = axleFrictionUtilization;
    command.frontFrictionUtilization = axleFrictionUtilization(1);
    command.rearFrictionUtilization = axleFrictionUtilization(2);
    command.tireSideslipAngle = [frontSlipAngle; rearSlipAngle];
    command.frontWheelSteeringAngle = steeringAngle;
end
