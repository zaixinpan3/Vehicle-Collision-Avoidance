function [command, predictedInput, planningProblem, certificate] = ...
        collisionAvoidanceController( ...
        egoState, targetEstimate, laneCenterline, cfg, controllerState)
%collisionAvoidanceController Certificate-preserving predictive CBF-CLF-QP.
% One QP minimizes input effort and squared continuous-time CLF slack.
% A complete steering/acceleration continuation and its geometric certificate
% are controller state. Solver results are checked before use; failure uses
% only a checked carried continuation. Safety covers declared prediction nodes.
%
% Pass the fifth argument and retain the fourth output for explicit state:
%   [command, plan, diagnostics, cNext] = ...
%       collisionAvoidanceController(ego, target, road, cfg, c);
% An empty c starts admission. With four inputs, a persistent compatibility
% interface retains c; resetNominalTrajectory clears that interface.

    persistent previousCertificate
    if nargin == 1 && (ischar(egoState) || isstring(egoState))
        if ~isscalar(string(egoState)) ...
                || string(egoState) ~= "resetNominalTrajectory"
            error("collisionAvoidanceController:invalidAction", ...
                "The only controller action is resetNominalTrajectory.");
        end
        previousCertificate = [];
        command = [];
        predictedInput = [];
        planningProblem = [];
        certificate = [];
        return;
    end
    explicitState = nargin >= 5;
    if ~explicitState
        controllerState = previousCertificate;
        previousCertificate = [];
    end
    if nargin < 4, cfg = []; end
    if nargin < 3, laneCenterline = []; end
    if nargin < 2, targetEstimate = []; end
    runtimeClock = tic;
    cfg = localControllerConfiguration(cfg);
    [ego, lane, road, targets] = readPlanningInputs( ...
        egoState, targetEstimate, laneCenterline, cfg);
    model = localPredictionModel(ego, targets, lane, road, cfg);
    performanceReference = crossingCruiseReference(model);
    model.referenceSpeed = performanceReference.speed;
    model.episodeIdentity = localEpisodeIdentity(model);
    [compatible, model] = localCertificateCompatible( ...
        controllerState, model.episodeIdentity, model);
    related = localCertificateRelated(controllerState, model);
    runtimePreparation = toc(runtimeClock);
    schedule = [];
    geometry = [];
    if related
        schedule = localShiftSchedule(controllerState.schedule);
        if ~compatible
            % Readmission uses the deterministic measured-speed template.
            % An unexecuted optimized future must not abruptly reschedule
            % the entire affine model when a new measurement arrives.
            schedule = [];
        end
    end
    if compatible
        geometry = localShiftGeometry(controllerState.geometry);
    end
    prediction = ltvBicycleModel.predict(model, schedule);
    terminalUncertainty = stateUncertainty.terminalRest(prediction);
    carriedDissipation = compatible && isfield(controllerState.terminalUncertainty, "kind") ...
        && controllerState.terminalUncertainty.kind == "dissipative-rest-funnel-v1";
    if any(prediction.egoStateErrorBound(4:6, :), "all") || carriedDissipation
        terminalUncertainty = terminalDissipation.build(prediction, model);
        prediction.terminalDissipation = terminalUncertainty;
    end
    if ~terminalUncertainty.accepted ...
            || any(cfg.model.ltvModelErrorRateBound ~= 0) ...
            || any(cfg.model.plantModelResidualRateBound ~= 0)
        error("collisionAvoidanceController:unsupportedCertificateUncertainty", ...
            "The declared terminal dynamics do not admit the required " ...
            + "stationary-pose or dissipative-rest certificate. Persistent " ...
            + "forcing and noncontractive velocity dynamics are unsupported.");
    end
    if any(prediction.egoStateErrorBound(:, 1)) ...
            && (~model.initialProjectionChartValid || ~isfinite(model.stateTime))
        error("collisionAvoidanceController:invalidUncertaintyChart", ...
            "Uncertain-state admission requires a finite stateTime and " ...
            + "one invertible interior projection chart for the initial box.");
    end
    prediction.scheduleShifted = compatible;
    runtimePrediction = toc(runtimeClock);
    if related
        inputs = reshape(controllerState.plan, 2, []);
        inputs = [inputs(:, 2:end), [0.0; -model.longitudinalAccelerationBias/modifiedFialaTire.accelerationGain(cfg)]];
        anchorPlan = inputs(:);
        source = "shiftedCertifiedContinuation";
        if ~compatible
            source = "continuationReadmission";
        end
    else
        anchorPlan = prediction.referencePlan;
        source = "initialAdmission";
    end
    qp = formulateAvoidanceProblem(model, prediction, anchorPlan, geometry);
    witnessValid = false;
    witnessDecision = [];
    if related
        witnessDecision = localWitnessDecision(qp, anchorPlan);
        witnessCheck = certifyAvoidancePlan(qp, prediction, model, witnessDecision);
        witnessValid = witnessCheck.accepted;
        if compatible && ~witnessValid
            error("collisionAvoidanceController:invalidStoredCertificate", ...
                "The carried continuation failed certificate preservation: %s.", ...
                strjoin(witnessCheck.failedConditions, ", "));
        end
    end

    runtimeFormulation = toc(runtimeClock);
    result = solveHardCbfClf(qp, cfg);
    runtimeSolve = toc(runtimeClock);
    check = certifyAvoidancePlan(qp, prediction, model, result.decision);
    fallback = ~result.feasible || ~check.accepted;
    if fallback
        if ~witnessValid
            if result.exitFlag == -2 || qp.certifiedInfeasible
                error("collisionAvoidanceController:noSolution", ...
                    "The current state has no feasible certificate in the selected convex domain.");
            end
            error("collisionAvoidanceController:optimizationFailure", ...
                "No accepted plan or applicable continuation: %s (%s).", ...
                result.message, strjoin(check.failedConditions, ", "));
        end
        decision = witnessDecision;
        check = witnessCheck;
        certificateSource = "shiftedStoredPlan";
        if ~compatible
            certificateSource = "revalidatedContinuation";
        end
    else
        decision = result.decision;
        certificateSource = "checkedOptimization";
    end
    % The vector checked above is the vector stored and sent. No clipping.
    plan = decision(qp.layout.planIndex);
    predictedInput = reshape(plan(qp.layout.inputIndex), 2, []);
    command = localCommand(predictedInput, model, prediction);
    predictedState = squeeze(pagemtimes(prediction.egoStateMatrix, plan)) ...
        + prediction.egoStateOffset;
    certificate = struct("version", 7, "plan", plan, ...
        "schedule", prediction.scheduleForStore, "geometry", qp.geometry, ...
        "targetHorizon", model.targetHorizon, "episodeIdentity", model.episodeIdentity, ...
        "predictedState", predictedState, "appliedInput", command.actuatorInput, ...
        "stateErrorBound", prediction.egoStateErrorBound, ...
        "setMembershipEnabled", model.setMembershipUpdate ...
            || any(prediction.egoStateErrorBound, "all"), ...
        "terminalUncertainty", terminalUncertainty, "stateTime", model.stateTime, ...
        "safetyScope", "declaredModelPredictionNodes", "acceptance", check);
    if ~explicitState
        previousCertificate = certificate;
    end
    runtimeAcceptance = toc(runtimeClock);
    metadata = localPlanDiagnostics(qp, result, decision, model, prediction);
    metadata.planCertified = check.accepted;
    metadata.certificateSource = certificateSource;
    metadata.postSolveCertificationPerformed = true;
    metadata.exactPredictionAssumptionsHold = ...
        ~any(prediction.egoStateErrorBound, "all");
    metadata.uncertaintyCertificate = terminalUncertainty;
    metadata.setMembershipUpdate = model.setMembershipUpdate;
    metadata.estimatedFrenetState = model.estimatedFrenetState;
    metadata.currentFrenetEstimationBound = model.currentFrenetEstimationBound;
    metadata.nodeClearanceMargin = check.nodeClearanceMargin;
    metadata.routeCoordinateValid = check.routeCoordinateValid;
    metadata.certificateCompatible = compatible;
    metadata.continuationReadmission = related && ~compatible;
    metadata.carriedWitnessFeasible = witnessValid;
    metadata.fallbackUsed = fallback;
    metadata.scheduleShifted = prediction.scheduleShifted;
    metadata.scheduleRefreshed = related && ~compatible;
    metadata.brakingRatioAccelerationGain = modifiedFialaTire.accelerationGain(cfg);
    metadata.performanceReferenceSpeed = performanceReference.speed;
    metadata.crossingYieldActive = performanceReference.yielding;
    metadata.crossingClearTime = performanceReference.clearTime;
    metadata.targetContinuationShifted = compatible;
    metadata.geometryReanchoredCount = qp.geometry.reanchoredCount;
    metadata.solverCallCount = result.solverCalls;
    metadata.nominalSource = source;
    metadata.acceptance = check;
    metadata.runtime = struct("inputPreparationSeconds", runtimePreparation, ...
        "predictionSeconds", runtimePrediction-runtimePreparation, ...
        "formulationAndWitnessSeconds", runtimeFormulation-runtimePrediction, ...
        "solveSeconds", runtimeSolve-runtimeFormulation, ...
        "acceptanceAndCommitSeconds", runtimeAcceptance-runtimeSolve, ...
        "diagnosticsSeconds", toc(runtimeClock)-runtimeAcceptance);
    planningProblem = struct("problemClass", qp.problemClass, "qp", qp, ...
        "layout", qp.layout, "prediction", prediction, "nominalInput", anchorPlan, ...
        "nominalSource", source, "decision", decision, "inputPlan", predictedInput, ...
        "plan", plan, "tailPlan", reshape(plan(qp.layout.tailIndex), 2, []), ...
        "metadata", metadata);
end

function related = localCertificateRelated(certificate, model)
% Retain maneuver memory when an observation changes. The old plan is only
% a proposal in this case: all geometry and acceptance checks are rebuilt
% from the new environment. It supplies no unverified fallback authority.
    related = isstruct(certificate) && isscalar(certificate) ...
        && all(isfield(certificate, ["version", "episodeIdentity", ...
            "plan", "schedule", "predictedState"])) && isequal(certificate.version, 7);
    if ~related
        return;
    end
    identity = certificate.episodeIdentity;
    related = isstruct(identity) && isscalar(identity) ...
        && all(isfield(identity, ["targetKey", "targetGeometry", ...
            "lane", "configuration"])) ...
        && isequaln(identity.targetKey, model.targetKey) ...
        && isequaln(identity.targetGeometry, ...
            [model.targetHalfLength; model.targetHalfWidth]) ...
        && isequaln(identity.lane, model.lane) ...
        && isequaln(identity.configuration, model.cfg) ...
        && isnumeric(certificate.plan) && isreal(certificate.plan) ...
        && numel(certificate.plan) == 2*(model.horizonSteps+model.tailSteps) ...
        && all(isfinite(certificate.plan), "all");
end

function schedule = localShiftSchedule(previous)
    schedule = previous;
    schedule.station = [previous.station(2:end), previous.station(end)];
    schedule.speedProfile = [previous.speedProfile(2:end), 0.0];
    schedule.curvature = [previous.curvature(2:end), previous.curvature(end)];
end

function geometry = localShiftGeometry(previous)
    geometry = previous;
    geometry.frames = previous.frames([2:end, end]);
    geometry.collision.nodes = previous.collision.nodes([2:end, end]);
    for boundaryIdx = 1:numel(previous.road)
        geometry.road(boundaryIdx).nodes = previous.road(boundaryIdx).nodes([2:end, end]);
    end
end

function decision = localWitnessDecision(qp, plan)
    clf = qp.clf;
    slack = clf.lieDerivativeDrift ...
        + clf.lieDerivativeInput*plan(1:qp.layout.inputDimension) ...
        + clf.decayRate*clf.initialValue;
    decision = [plan; max(slack, 0.0)];
end

function identity = localEpisodeIdentity(model)
    identity = struct( ...
        "targetKey", model.targetKey, ...
        "targetGeometry", [model.targetHalfLength; ...
            model.targetHalfWidth], ...
        "routeBranchId", string(model.road.routeBranchId), ...
        "lane", model.lane, "road", model.road, ...
        "accelerationBias", model.longitudinalAccelerationBias, ...
        "configuration", model.cfg);
end

function [compatible, model] = localCertificateCompatible(certificate, identity, model)
    compatible = isstruct(certificate) && isscalar(certificate) ...
        && all(isfield(certificate, ["version", "episodeIdentity", "plan", ...
            "predictedState", "targetHorizon", "schedule", "geometry", "appliedInput", ...
            "stateErrorBound", "stateTime", "terminalUncertainty"])) ...
        && isequal(certificate.version, 7) ...
        && isequaln(certificate.episodeIdentity, identity);
    if ~compatible
        return;
    end
    tolerance = model.cfg.controller.shiftConsistencyTolerance;
    compatible = size(certificate.predictedState, 2) >= 2 ...
        && localTargetShiftMatches(certificate.targetHorizon, ...
            model.targetHorizon, tolerance);
    held = model.heldActuatorInput(:);
    if compatible && numel(held) == model.inputDimension
        compatible = localNumericallyEqual( ...
            certificate.appliedInput, held, tolerance);
    end
    if ~compatible
        return;
    end
    uncertain = any(certificate.stateErrorBound, "all") ...
        || any(model.initialFrenetErrorBound) ...
        || (isfield(certificate, "setMembershipEnabled") && certificate.setMembershipEnabled);
    if ~uncertain
        compatible = localNumericallyEqual(certificate.predictedState(:, 2), ...
            model.initialEgoState, tolerance) ...
            && ~any(model.initialFrenetErrorBound);
        return;
    end
    timeTolerance = 128*eps(max([1, abs(model.stateTime), abs(certificate.stateTime)]));
    compatible = isfinite(model.stateTime) && isfinite(certificate.stateTime) ...
        && abs(model.stateTime-certificate.stateTime-model.sampleTime) <= timeTolerance;
    if ~compatible
        return;
    end
    [radius, consistent] = stateUncertainty.intersect( ...
        certificate.predictedState(:, 2), certificate.stateErrorBound(:, 2), ...
        model.initialEgoState, model.initialFrenetErrorBound);
    if ~consistent
        error("collisionAvoidanceController:inconsistentStateEnclosures", ...
            "The current estimation box and carried reachable box are " ...
            + "disjoint; the state/model/execution contracts are inconsistent.");
    end
    model.initialEgoState = certificate.predictedState(:, 2);
    model.initialFrenetErrorBound = radius;
    model.setMembershipUpdate = true;
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

function [valueProfile, planError] = localClfValueProfile(clf, planColumn)
% Predicted quadratic values are diagnostics; only Vdot at x_0 is constrained.
    nodeCount = size(clf.errorOffset, 2);
    planError = zeros(size(clf.errorOffset));
    valueProfile = zeros(1, nodeCount);
    for nodeIdx = 1:nodeCount
        planError(:, nodeIdx) = clf.errorMatrix(:, :, nodeIdx) ...
            * planColumn+clf.errorOffset(:, nodeIdx);
        valueProfile(nodeIdx) = planError(:, nodeIdx).' ...
            * clf.lyapunovMatrix*planError(:, nodeIdx);
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
% margins, the continuous-time CLF relaxation,
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

    % The complete bicycle continuation and its resting endpoint.
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
        "tireSlip", sum(qp.rowFamily == "tireSlip"), ...
        "lateralDomain", sum(qp.rowFamily == "lateralDomain"), ...
        "routeDomain", sum(qp.rowFamily == "routeDomain"), ...
        "terminalRest", size(qp.equalityMatrix, 1));

    % Continuous-time CLF relaxation and derivative residual at the current state.
    relaxation = decision(layout.relaxationIndex);
    metadata.clfRelaxation = relaxation;
    metadata.clfInitialValue = qp.clf.initialValue;
    [valueProfile, planError] = localClfValueProfile(qp.clf, planColumn);
    metadata.clfValueProfile = valueProfile;
    metadata.clfDerivative = qp.clf.lieDerivativeDrift ...
        + qp.clf.lieDerivativeInput*planColumn(1:layout.inputDimension);
    metadata.clfDecayRate = qp.clf.decayRate;
    metadata.clfDerivativeResidual = metadata.clfDerivative ...
        + qp.clf.decayRate*qp.clf.initialValue-relaxation;
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
    metadata.inputEffortCost = 0.5*planColumn.'*planHessian ...
        * planColumn+qp.linear(layout.planIndex).'*planColumn ...
        + qp.constant;
    metadata.inputDeviationCost = metadata.inputEffortCost; % Compatibility alias.
    metadata.clfRelaxationCost = ...
        0.5*qp.Hessian(layout.relaxationIndex, layout.relaxationIndex)*relaxation^2 ...
        + qp.linear(layout.relaxationIndex)*relaxation;
    metadata.jointObjectiveValue = metadata.inputEffortCost ...
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

function metadata = localTerminalDiagnostics(metadata, qp, plan, model, prediction)
    state = squeeze(pagemtimes(prediction.egoStateMatrix, plan)) ...
        + prediction.egoStateOffset;
    tail = reshape(plan(qp.layout.tailIndex), 2, []);
    metadata.tailBrakingRatioPlan = tail(2, :);
    metadata.tailAccelerationPlan = modifiedFialaTire.accelerationGain(model.cfg)*tail(2, :);
    metadata.tailSteeringPlan = tail(1, :);
    metadata.tailSpeedProfile = state(4, prediction.tailNodeIndex);
    metadata.tailStationProfile = state(1, prediction.tailNodeIndex);
    metadata.restStation = state(1, end);
    metadata.terminalSpeed = state(4, end);
    metadata.terminalHeadingError = state(3, end);
    metadata.terminalHeadingBound = model.cfg.model.headingDomainRadius;
    metadata.terminalLateralVelocity = state(5, end);
    metadata.terminalYawRateError = state(6, end);
    metadata.terminalRestResidual = norm(state(4:6, end), inf);
    metadata.terminalInvariantCertified = metadata.terminalRestResidual ...
        <= 10.0*model.cfg.solver.constraintTolerance;
    metadata.terminalInvariantMargin = inf;
    metadata.terminalContinuationAxis = "none";
    metadata.terminalFutureSupport = inf;
    metadata.terminalSupportDirection = zeros(2, 1);
    metadata.terminalSegmentIndex = qp.geometry.frames(end).segmentIndex;
    if model.hasTarget
        node = qp.collision.nodes(end);
        metadata.terminalInvariantMargin = min(node.marginMatrix*plan+node.marginOffset);
        metadata.terminalInvariantCertified = metadata.terminalInvariantCertified ...
            && node.terminalInvariant && metadata.terminalInvariantMargin ...
                >= -10.0*model.cfg.solver.constraintTolerance;
        metadata.terminalContinuationAxis = node.terminalContinuationAxis;
        metadata.terminalFutureSupport = node.terminalFutureSupport;
        metadata.terminalSupportDirection = node.terminalSupportDirection;
    end
    if isfield(prediction, "terminalDissipation")
        rows = qp.rowFamily == "terminalDissipation";
        metadata.terminalInvariantMargin = min(qp.inequalityBound(rows) ...
            -qp.inequalityMatrix(rows, qp.layout.planIndex)*plan);
        metadata.terminalInvariantCertified = prediction.terminalDissipation.accepted ...
            && metadata.terminalInvariantMargin >= -10*model.cfg.solver.constraintTolerance;
        metadata.terminalVelocityLimit = prediction.terminalDissipation.velocityLimit;
        metadata.remainingPoseExcursionBound = prediction.terminalDissipation.poseExcursionMatrix ...
            *(abs(state(4:6, end))+prediction.egoStateErrorBound(4:6, end));
    end
end

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
    projection = laneGeometry.project(ego.position, lane);
    headingError = atan2(sin(ego.yaw-projection.heading), ...
        cos(ego.yaw-projection.heading));
    model.initialCartesianState = ego.modelState;
    model.initialEgoState = [projection.station; ...
        projection.lateralPosition; headingError; ego.modelState(4:6)];
    model.measuredEgoStateErrorBound = ego.stateErrorBound;
    [model.initialFrenetErrorBound, model.initialProjectionChartValid] = ...
        stateUncertainty.toFrenet(ego.modelState, ego.stateErrorBound, lane);
    model.currentFrenetEstimationBound = model.initialFrenetErrorBound;
    model.estimatedFrenetState = model.initialEgoState;
    model.setMembershipUpdate = false;
    model.stateTime = ego.stateTime;
    model.egoErrorCertificate = ego.errorCertificate;
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
    model.plannedSpeedFloor = 0.0;
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
    model.targetAccelerationErrorBound = zeros(2, 1);
    model.targetPredictionMotionBounds = [];
    model.targetErrorCertificate = [];
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
        model.targetAccelerationErrorBound = target.accelerationErrorBound;
        model.targetPredictionMotionBounds = target.predictionMotionBounds;
        model.targetErrorCertificate = target.errorCertificate;
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
    % speed domain, never declared (ltvBicycleModel.brakingSchedule).
    model.tailSteps = ltvBicycleModel.brakingSchedule("steps", cfg);
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
        targetPrediction.errorEnvelope(nodeTime, model);
    horizon = struct();
    horizon.targetPosition = position;
    horizon.targetYaw = yaw;
    horizon.targetPositionErrorBound = positionErrorBound;
    horizon.targetYawErrorBound = yawErrorBound;
    horizon.estimationErrorCertificate = model.targetErrorCertificate;
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
    direction = [cos(angle); sin(angle)];
    direction(abs(direction) < 100.0*eps) = 0.0;
    direction = direction./vecnorm(direction);
    horizon.terminalSupportDirection = direction;
    if ~isempty(model.targetPredictionMotionBounds)
        % Speed/acceleration domains alone do not restrict the target to a
        % bounded future halfspace. A present B(t) is not such a restriction.
        horizon.terminalFuturePositionSupport = inf(1, directionCount);
        if model.targetPredictionMotionBounds.speedMaximum == 0.0
            horizon.terminalFuturePositionSupport = model.targetPosition.'*direction ...
                + model.targetPositionErrorBound.'*abs(direction);
        end
        return;
    end
    horizon.terminalFuturePositionSupport = ...
        targetPrediction.futureSupport( ...
            horizon.terminalSupportDirection, ...
            horizon.terminalSupportStartTime, model.targetPrediction);
    % Independent velocity/acceleration error boxes grow without bound.
    % Admit a terminal direction only when its complete future support is
    % finite; a rest-node error radius alone cannot certify future safety.
    horizon.terminalFuturePositionSupport = ...
        horizon.terminalFuturePositionSupport ...
        + model.targetPositionErrorBound.'*abs(direction);
    growth = (model.targetVelocityErrorBound ...
        + model.targetPredictionAccelerationErrorBound).'*abs(direction);
    horizon.terminalFuturePositionSupport(growth > 0.0) = inf;
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

function command = localCommand(inputPlan, model, prediction)
    firstInput = inputPlan(:, 1);
    cfg = model.cfg;
    state = model.initialEgoState;
    forceScheduleSpeed = max(prediction.scheduleSpeedProfile(1), ...
        cfg.model.scheduleSpeedFloor);
    tire = modifiedFialaTire.parameters(cfg);
    steeringAngle = firstInput(1);
    brakingRatio = firstInput(2);
    longitudinalAcceleration = modifiedFialaTire.accelerationGain(cfg)*brakingRatio;
    % Paper Eqs. (8)-(9); use the same scheduled Fiala tangent as prediction.
    frontSlipAngle = (state(5)+cfg.vehicle.lf*state(6)) ...
        / forceScheduleSpeed-steeringAngle;
    rearSlipAngle = (state(5)-cfg.vehicle.lr*state(6)) ...
        / forceScheduleSpeed;
    [tireSlope, ratioSlope, tireIntercept] = modifiedFialaTire.linearize( ...
        prediction.scheduleCurvature(1), prediction.scheduleSpeedProfile(1), ...
        prediction.scheduleBrakingRatio(1), cfg);
    axleLateralForce = tireSlope.*[frontSlipAngle; rearSlipAngle] ...
        +ratioSlope*brakingRatio+tireIntercept;
    axleLongitudinalForce = modifiedFialaTire.longitudinalForce(brakingRatio, cfg);
    [roadForce, ~, roadComponents] = longitudinalRoadLoad(state(4), cfg);
    netLongitudinalAcceleration = longitudinalAcceleration ...
        - roadForce/cfg.vehicle.m+model.longitudinalAccelerationBias;
    axleRollingResistance = roadComponents.rollingResistanceForce ...
        * tire.staticNormalLoad/(cfg.vehicle.m*cfg.vehicle.gravity);
    axleContactForce = axleLongitudinalForce-axleRollingResistance;
    command = struct();
    command.brakingRatio = brakingRatio;
    command.longitudinalAcceleration = longitudinalAcceleration;
    command.bodyLongitudinalVelocityDerivative = ...
        netLongitudinalAcceleration+state(5)*state(6);
    command.lateralAcceleration = ...
        sum(axleLateralForce)/cfg.vehicle.m-state(4)*state(6);
    command.yawAcceleration = ...
        (cfg.vehicle.lf*axleLateralForce(1) ...
            - cfg.vehicle.lr*axleLateralForce(2))/cfg.vehicle.Iz;
    command.actuatorInput = [steeringAngle; brakingRatio];
    command.actuatorInputOrder = [ ...
        "frontWheelSteeringAngle", "brakingRatio"];
    command.totalLongitudinalActuatorForce = sum(axleLongitudinalForce);
    command.totalLongitudinalTireForce = sum(axleContactForce);
    command.axleLongitudinalTireForce = axleContactForce;
    command.aerodynamicResistanceForce = roadComponents.aerodynamicForce;
    command.rollingResistanceForce = roadComponents.rollingResistanceForce;
    command.totalRoadLoadForce = roadForce;
    % Compatibility field consumed by the torque adapter: actuator force
    % before rolling loss, not contact force at the tire patch.
    command.axleLongitudinalForce = axleLongitudinalForce;
    command.axleLateralForce = axleLateralForce;
    command.axleNormalLoad = tire.staticNormalLoad;
    command.tireSideslipAngle = [frontSlipAngle; rearSlipAngle];
    command.frontWheelSteeringAngle = steeringAngle;
end
