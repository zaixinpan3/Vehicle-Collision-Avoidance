function [command, predictedInput, planningProblem] = ...
        collisionAvoidanceController( ...
        egoState, targetEstimate, laneCenterline, cfg)
% collisionAvoidanceController Recursive hard-CBF, soft-CLF controller.
%
% The default certified mode imposes every collision and road CBF row as a
% hard constraint, together with actuator, tire, model-domain, backup-tail,
% and augmented terminal rows. There is no CBF or safety slack. If the
% first-frame hard problem is infeasible, the controller raises noSolution
% and issues no command. Inside the hard-safe feasible set, one conic solve
% jointly minimizes the exact first-step discrete CLF relaxation and nominal
% input intervention using their configured weights.
%
% A feasible result of the certified hard-constrained optimization is
% accepted and stored directly; it is not passed through a duplicate check
% of prediction assumptions, route coordinates, hard CBF rows, terminal
% invariance, exact node clearance, or swept inter-sample clearance. On the
% next call, measured ego state, held actuation when available, and the fresh
% target prediction must equal the stored one-step shift. The LTV schedule is
% shifted rather than regenerated. If a new solve fails, the shifted stored
% plan is the exceptional fallback path and is rechecked before execution.
% An old plan is never silently applied to a changed episode.
%
% The braking tail ends at rest in a hard target-conditioned terminal
% half-space. Its threshold is the support of the controller's COMPLETE
% predicted target continuation in a fixed inertial direction, enlarged by
% footprint circumradii and the certified lateral-tail envelope. The
% support interface admits every trajectory produced by the predictor;
% stationary, straight, monotone, or curvature predicates are never
% terminal admissibility conditions. Exact prediction and shift
% consistency are visible in the returned certification metadata.
%
% The shared two-stage geometry and the optional noncertified legacy path
% are detailed below. Statements about fact-row relaxation, elastic
% refinement, predictive tangent CLF rows, or a monitored one-step rest
% contract apply only when cfg.certification.enabled is false. Certified
% mode may evaluate multiple starts, but every resulting problem is hard.
%
% The legacy path uses one convex quadratic program per sample: the
% two-stage approximate convex optimization of Li, Zhang, Guo, Lenzo
% and Guo (2023) - TWO_STAGE_QP.md - on the scheduled Frenet LTV
% dynamic-bicycle model. Its ingredients, and nothing else:
%
%   SAFETY is the collision-free condition dist(ego_k, target_k) >= d_min
%   at every node of the prediction horizon, made convex in two
%   stages and imposed HARD. STAGE 1 solves, at every node, the DUAL
%   of the distance problem between a LINEARIZATION TRAJECTORY's ego
%   rectangle and the target rectangle in path coordinates (paper
%   Eq. 12): the exact projection of the ego centre onto the TRUE
%   configuration obstacle, the Minkowski sum of the two ORIENTED
%   rectangles (rectangleConfigurationDistance) - a convex polygon of
%   at most eight vertices, with no bounding box taken of either
%   rectangle. Its optimizer is the separating direction n_k. STAGE 2
%   freezes n_k and imposes the affine row (paper Eq. 13 without its
%   slack), written through the two rectangles' SUPPORT FUNCTIONS,
%   with the decision's yaw charged by the exact Lipschitz bound of
%   the ego's support over the heading interval the program admits
%   (TWO_STAGE_SAFETY.md, Lemmas 1'-4). Weak duality makes the row
%   SUFFICIENT for the true rectangle separation at every plan, and
%   strong duality makes it EXACT at the linearization trajectory: the
%   row is the supporting half-plane of the true obstacle facing that
%   trajectory - an edge of one of the rectangles where it faces an
%   edge, the vertex direction where it faces a corner. There is no
%   slack, no priced violation, no face assignment, no barrier chain: a
%   plan that cannot honour every imposed row is "no solution".
%
%   THE TERMINAL SET closes the horizon (TWO_STAGE_SAFETY.md, "The
%   terminal set"). Every program ends in a KINEMATIC BRAKING TAIL of
%   N_b further stages whose accelerations are decision variables, on
%   which the same separation and road rows are imposed, and which
%   must end at REST. A plan is admitted only if the vehicle can come
%   to rest from its terminal state without violating a row on the
%   way - the backup-set construction of the control-barrier-function
%   literature, with the optimizer supplying the witness. Rest is
%   invariant under zero input, so under the declared model and target
%   prediction the previous plan shifted by one stage, with a zero
%   tail input appended, is a feasible point of the next sample's
%   program (Proposition 8): the feasible set is control invariant,
%   and the hard rows are a control barrier function. The tail is
%   never executed. Its lateral band and the terminal handoff rows on
%   the heading error, the lateral velocity and the yaw-rate error are
%   certified in closed form (terminalLateralCertificate), and the
%   tail's length is derived from the actuator (kinematicBrakingTail).
%
%   A hard row is a constraint only where the input has authority over
%   it. Under the forward-Euler stage map the pose one step ahead is a
%   fact of the measured state and the input's authority over a node's
%   margin grows like the fourth power of the node index; one sample of
%   plant mismatch moves a margin by up to the declared
%   cfg.collision.disturbanceBound, and at a node the input cannot move
%   by that much a hard row is either true or infeasible. The rows are
%   imposed wherever some plan in the input reach box
%   satisfies them; a row every plan in the box misses, at a node of
%   the near window, by at most the mismatch it can have accumulated
%   since it was last a hard row, is a fact, and where a candidate is
%   nevertheless infeasible the near-window rows no admissible plan
%   satisfies jointly (the friction polygon, which the reach box does
%   not see) are named by one least-relaxation LP and, within the same
%   allowance, become facts as well (localFactRelaxation). Facts are
%   reported (collisionFactViolation), never imposed; a row missed by
%   more, or beyond the window, makes the candidate infeasible
%   (TWO_STAGE_SAFETY.md, Proposition 2).
%
%   THE LINEARIZATION TRAJECTORY DECIDES THE HOMOTOPY CLASS, and that
%   is the design's one real difficulty. The frozen dual is one of the
%   four supporting half-planes of the target box, chosen by the
%   trajectory alone: a trajectory behind the target yields "stay
%   behind the rear face", a row whose coefficient on the lateral
%   coordinate is zero. Such a program can neither see nor reach a
%   passing plan, and its solution is another behind trajectory - a
%   FIXED POINT that survives any number of re-linearizations at the
%   solution, and that persists after braking has stopped being an
%   answer. No convex relaxation escapes it: the supporting
%   half-planes of the obstacle union to its complement, whose convex
%   hull is the whole plane, so every single convex program is blind to
%   every class but its own.
%
%   THE ANSWER IS NOT A CASE ANALYSIS BUT A SECOND START. This file
%   contains no test on the driving situation - not whether the target
%   is ahead or beside, not whether the plan is following or passing,
%   not which way to go. The controller solves the same trajectory
%   optimization from two STARTS: the shifted previous solution (the
%   receding horizon's warm start) and the cruise probe (the
%   obstacle-free reference, a cold start, which runs THROUGH the
%   obstacle so that stage 1 answers it with the outward normal of the
%   least-penetrated edge - lateral, for an obstacle longer than it is
%   wide entered from behind - and those are rows that credit
%   steering). Each start is refined by the same sequential convex loop
%   (localRefineStart): while its own stage-1 margins show a shortfall,
%   re-solve an ELASTIC program at it - one priced slack per imposed
%   target node, everything else hard - and take the solution as the
%   next linearization, stage 1 recomputed there so the normals rotate
%   with the iterate. A start already separated exits before solving
%   anything, which the shifted plan normally does, so the receding
%   horizon pays nothing for the machinery. Each refined start is then
%   solved as the same hard program and the feasible one with the least
%   EXACT objective is committed (localSolve). Braking, following,
%   passing left and passing right are outcomes of that comparison;
%   none is a case in the code, and no side is ever named - which side
%   a start ends up on is the sign of its own offset from the target's
%   centre line. No rung is ever committed.
%
%   TRACKING is the soft control Lyapunov condition
%   V(e_{k+1}) - V(e_k) <= -f W(e_k) + delta_k on every transition,
%   with V the discrete Riccati certificate of the path-frame cruise
%   error about the cruise EQUILIBRIUM of the declared model, and
%   delta_k >= 0 priced in the objective. Cruising at the reference
%   speed along the nominal path is what the CLF asks for; it is never
%   a tracking cost. The rows are the tangent of V at the nominal
%   error, and a tangent of a convex quadratic credits an unbounded
%   decrease along its downhill direction, so the plan's error is held
%   within a TRUST REGION of the nominal's (cfg.clf.trustRegionScale
%   error scales per channel and sample): the receding-horizon loop is
%   a damped SQP iteration of the exact rows, not an unbounded one.
%
%   THE OBJECTIVE is minimum intervention: the deviation of the whole
%   input plan from the cruise equilibrium input, plus the price of the
%   CLF relaxations. There is no tracking cost, no collision cost, no
%   terminal cost; the tail accelerations are priced only for positive
%   definiteness.
%
% The closed-loop behaviour follows from those alone. In free cruise
% the separation rows are inactive, the relaxations are zero, and the
% plan holds the equilibrium input. When the target's box enters the
% horizon the rows at the far nodes become active, the incumbent
% brakes while its duals face the rear of the obstacle; the second
% start, refined from the obstacle-free reference, offers whatever
% class it converges to; and the objective commits whichever costs
% less inside the rows, the model domain, the friction circles and the
% input box. After the target passes the rows go inactive again
% and the relaxed CLF rows pull the vehicle back to cruise.
%
% In certified mode the returned planCertified flag means that the hard
% constrained solver returned a feasible plan, or that an exceptional stored
% fallback passed its separate recheck. Under the exact shift assumptions,
% the shifted stored candidate supplies the recursive-feasibility induction.
% In legacy mode the weaker per-sample/fact-row claims in
% TWO_STAGE_SAFETY.md apply; they must not be read as this certificate.
%
% The certified memory is the complete plan together with its predicted
% state, target continuation, shifted LTV schedule, terminal lateral
% reference, and applied first input. The first backup-tail acceleration
% enters the head together with the analytic terminal steering law. The
% resetNominalTrajectory action deliberately invalidates all of it. In
% legacy mode only the shifted linearization plan is retained.
%
% Inputs: egoState (explicit controller-state fields or the
% estimator's egoState vector form, optionally heldActuatorInput and
% longitudinalAccelerationBias), targetEstimate (at most one target),
% laneCenterline (an N-by-2 centerline or a road-geometry structure
% with centerline, finite local-quadratic boundaries and a route), and
% the configuration override merged over
% config/collisionAvoidanceControllerConfig. The command is
% [frontWheelSteeringAngle; longitudinalAcceleration] with the
% axle-force readout; predictedInput is the whole plan; the third
% output carries the program, the decision, and the diagnostics
% (metadata: collisionMargin, collisionPlanMargin, collisionFactViolation,
% collisionImposedFrom, candidateLabels, selectedCandidate,
% candidateObjectives, candidateRefinementRungs, dualRegionProfile,
% dualNormalProfile, clfRelaxation, ...).
%
% Modules: readPlanningInputs (input contract), ltvBicyclePrediction
% (schedule and condensation of the Frenet model over
% ltvBicycleStageMatrices and ltvBicycleRollout), formulateTwoStageQp
% (stage 1 duals, stage 2 rows, the cruise probe, CLF data, objective,
% with cruiseEquilibrium local to it), solveHardCbfClf (the certified
% joint conic kernel), solveTwoStageQp (the legacy QP kernel),
% frictionCirclePolygonRows with axleFrictionParameters and
% longitudinalAccelerationBounds (the actuator/tire layer),
% laneProjection and laneCurvatureAtStation (the path frame),
% rectangleConfigurationDistance (the configuration obstacle: stage 1
% in path coordinates, and the physical clearance readout in Cartesian
% coordinates).

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
            model, prediction, nominalInput, true);
    catch exception
        if ~certificateCompatible || ~previousCertificate.certified
            previousCertificate = [];
            rethrow(exception);
        end
        [inputPlan, problem, plan] = localStoredFallback( ...
            nominalInput, prediction, previousCertificate, exception, model);
        fallbackUsed = true;
    end
    if ~fallbackUsed
        problem.metadata.planCertified = cfg.certification.enabled;
        problem.metadata.certificateSource = ...
            "hardConstrainedOptimization";
        problem.metadata.postSolveCertificationPerformed = false;
        if cfg.certification.enabled
            previousCertificate = localStoredCertificate( ...
                plan, prediction, model, problem.metadata);
        else
            previousCertificate = [];
        end
    end
    if fallbackUsed
        previousCertificate = localStoredCertificate( ...
            plan, prediction, model, problem.metadata);
    end
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
    compatible = isstruct(certificate) && isscalar(certificate) ...
        && isfield(certificate, "certified") && certificate.certified ...
        && isfield(certificate, "episodeIdentity") ...
        && isequaln(certificate.episodeIdentity, identity) ...
        && isfield(certificate, "plan") ...
        && isfield(certificate, "schedule") ...
        && isfield(certificate, "targetHorizon") ...
        && isfield(certificate, "predictedState") ...
        && isfield(certificate, "appliedInput") ...
        && isfield(certificate, "terminalLateralReference");
    if ~compatible
        return;
    end
    tolerance = model.cfg.certification.shiftConsistencyTolerance;
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

function stored = localStoredCertificate( ...
        plan, prediction, model, metadata)
    predictedState = localPredictedState(prediction, plan);
    stored = struct( ...
        "certified", true, ...
        "plan", plan, ...
        "schedule", prediction.scheduleForStore, ...
        "targetHorizon", model.targetHorizon, ...
        "episodeIdentity", model.episodeIdentity, ...
        "predictedState", predictedState, ...
        "appliedInput", plan(1:model.inputDimension), ...
        "terminalLateralReference", ...
            predictedState(2, prediction.headNodeCount), ...
        "metadata", metadata);
end

function [inputPlan, problem, plan] = localStoredFallback( ...
        shiftedPlan, prediction, storedCertificate, exception, model)
    plan = shiftedPlan;
    options = struct("label", "verifiedStoredPlan", ...
        "obstacleMode", "certified", "trustRegionScale", inf);
    qp = formulateTwoStageQp(model, prediction, plan, options);
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
    jointObjective = 0.5*decision.'*qp.Hessian*decision ...
        + qp.linear.'*decision+qp.constant;
    result = struct("objectiveValue", jointObjective, "exitFlag", 1, ...
        "iterations", 0, "retried", false, ...
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
    if ~verified || ~storedCertificate.certified
        error("collisionAvoidanceController:invalidStoredCertificate", ...
            "The shifted stored plan failed its exact commit certificate " ...
            + "(%s) after %s.", ...
            strjoin(planCertificate.failedConditions, ", "), ...
            exception.identifier);
    end
    problem.metadata.planCertified = true;
    problem.metadata.exactPredictionAssumptionsHold = ...
        planCertificate.exactPredictionAssumptionsHold;
    problem.metadata.sweptCollisionCertificate = planCertificate.swept;
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
    certificate.swept = struct("certified", true, ...
        "intervalCertified", true(1, 0), ...
        "intervalMargin", inf(1, 0), "minimumMargin", inf, ...
        "maximumDepth", 0, "failureReason", strings(1, 0));
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
        options = struct( ...
            "maxDepth", model.cfg.certification.interSampleMaxDepth, ...
            "distanceTolerance", ...
                model.cfg.certification.interSampleDistanceTolerance);
        intervalClearance = repmat( ...
            model.cfg.collision.clearanceMargin, ...
            1, prediction.nodeCount-1);
        intervalClearance(prediction.headNodeCount:end) = ...
            intervalClearance(prediction.headNodeCount:end)+tailEnvelope;
        certificate.swept = certifySweptRectangleIntervals( ...
            egoPose, targetPose, halfDimensions, intervalClearance, options);
    end
    conditionName = [ ...
        "exactPrediction", "routeCoordinate", "hardCbf", ...
        "terminalInvariant", "nodeClearance", "sweptClearance"];
    conditionHolds = [ ...
        certificate.exactPredictionAssumptionsHold, ...
        certificate.routeCoordinateValid, ...
        problem.metadata.hardCbfSatisfied, ...
        problem.metadata.terminalInvariantCertified, ...
        certificate.nodeClearanceMargin >= -10.0*tolerance, ...
        certificate.swept.certified];
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
% accelerations): the previous plan shifted by one stage - the head
% advanced, its new last input the repeated steering with the tail's
% first acceleration, the tail advanced with a zero input appended -
% or the schedule reference with its braking tail. The shift is the
% candidate of Proposition 8: under the declared model it is a feasible
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
        if isstruct(certificate) ...
                && isfield(certificate, "terminalLateralReference")
            shiftedInput(1, end) = localTerminalBackupSteering( ...
                state, certificate.terminalLateralReference, ...
                prediction.scheduleCurvature(model.horizonSteps), ...
                model.cfg);
        end
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
        model, prediction, nominalInput, buildPlanningProblem)
% The sample's programs, and the only place a behaviour is chosen.
%
% THE CONTROLLER CONTAINS NO TEST ON THE DRIVING SITUATION. It does not
% ask whether the target is ahead or beside, whether the plan is
% following or passing, whether a manoeuvre is called for, or which way
% to go. It solves the same trajectory optimization from a fixed set of
% STARTS, refines each by the same sequential convex loop, solves each
% as the same hard program, and commits the feasible one with the least
% EXACT objective. Braking, following, passing left and passing right
% are outcomes of that comparison, never cases in this file.
%
% THE STARTS are the two the problem itself supplies:
%
%   the SHIFTED PLAN   - the previous solution advanced one stage, the
%                        receding horizon's own warm start;
%   the CRUISE PROBE   - the trajectory the vehicle would follow if the
%                        target were not there (formulateTwoStageQp>
%                        localCruiseProbeInput), the obstacle-free
%                        reference, which is a cold start.
%
% Both are functions of the route, the measured state and the declared
% model; neither names a manoeuvre. The second is formed whenever the
% obstacle CONSTRAINS the program - that is, whenever the target family
% imposes a row - because that is exactly when the linearization can
% decide the answer; it is a statement about the program, not about the
% scene.
%
% WHY MORE THAN ONE START. The frozen dual is one supporting half-plane
% of the configuration obstacle, chosen by the trajectory stage 1 is
% evaluated on, so that trajectory fixes the homotopy class: a braking
% linearization yields rows with no lateral coefficient whose feasible
% set contains no passing plan, and re-solving at the solution
% reproduces it (TWO_STAGE_SAFETY.md, Proposition 5). One start can
% therefore only ever return its own class. Two starts return two, and
% the objective - not the author - decides between them.
%
% CERTIFIED MODE solves the hard program directly at each start. It has no
% collision slack and no elastic refinement; if neither hard candidate is
% feasible on the first frame, controller failure is declared. The optional
% legacy mode may refine each start through localRefineStart before its hard
% solve.
%
% Kernel budget: the first candidate gets the full budget and the retry
% ladder, the others cfg.solver.candidateMaxIterations and no retry;
% the stalled ones are retried only when nothing resolved.
    cfg = model.cfg;
    if cfg.disjunctive.nodeBudget > 0
        [inputPlan, problem, plan] = localSolveDisjunctive(model, ...
            prediction, nominalInput, buildPlanningProblem);
        return;
    end
    firstOptions = struct();
    if cfg.certification.enabled
        firstOptions = struct("label", "shiftedPlan", ...
            "obstacleMode", "certified", "trustRegionScale", inf);
    end
    [firstQp, common] = formulateTwoStageQp( ...
        model, prediction, nominalInput, firstOptions);
    startInput = {nominalInput};
    labels = "shiftedPlan";
    % The obstacle-free reference is a second start exactly when the
    % obstacle constrains this program.
    if ~isempty(cfg.sequentialConvex.penaltySchedule) ...
            && ~isempty(firstQp.collision.nodes) ...
            && any([firstQp.collision.nodes.imposed])
        startInput{end+1} = common.probeInput;
        labels(end+1) = "cruiseProbe";
    end

    candidateCount = numel(startInput);
    qps = cell(1, candidateCount);
    results = cell(1, candidateCount);
    trustDroppedFlags = false(1, candidateCount);
    factRelaxedCounts = zeros(1, candidateCount);
    rungCounts = zeros(1, candidateCount);
    startViolations = zeros(1, candidateCount);
    refinementCalls = 0;
    for candidateIdx = 1:candidateCount
        policy = struct();
        if candidateIdx > 1
            policy = struct( ...
                "maxIterations", cfg.solver.candidateMaxIterations, ...
                "retry", false);
        end
        if cfg.certification.enabled
            refined = startInput{candidateIdx};
            refinementSolves = 0;
            probeOptions = struct("label", labels(candidateIdx), ...
                "obstacleMode", "certified", "trustRegionScale", inf);
            probeQp = formulateTwoStageQp(model, prediction, refined, ...
                probeOptions, common);
            startViolations(candidateIdx) = localStartViolation(probeQp);
        else
            [refined, rungCounts(candidateIdx), ...
                startViolations(candidateIdx), refinementSolves] = ...
                localRefineStart(model, prediction, common, cfg, ...
                    startInput{candidateIdx});
        end
        refinementCalls = refinementCalls+refinementSolves;
        [qps{candidateIdx}, results{candidateIdx}, ...
            trustDroppedFlags(candidateIdx), ...
            factRelaxedCounts(candidateIdx)] = localHardCandidate( ...
                model, prediction, refined, common, cfg, ...
                labels(candidateIdx), policy);
    end
    % Nothing resolved: give the stalled candidates the retry ladder.
    if ~any(cellfun(@(candidate) candidate.feasible, results))
        for candidateIdx = 1:candidateCount
            if results{candidateIdx}.unresolved
                results{candidateIdx} = localSolveCandidate( ...
                    qps{candidateIdx}, cfg, struct());
            end
        end
    end

    exitFlags = zeros(1, candidateCount);
    linearObjectives = inf(1, candidateCount);
    exactObjectives = inf(1, candidateCount);
    clfObjectives = inf(1, candidateCount);
    certified = false(1, candidateCount);
    messages = strings(1, candidateCount);
    solverCalls = refinementCalls;
    bestIdx = 0;
    for candidateIdx = 1:candidateCount
        candidateQp = qps{candidateIdx};
        candidateResult = results{candidateIdx};
        exitFlags(candidateIdx) = candidateResult.exitFlag;
        certified(candidateIdx) = candidateQp.certifiedInfeasible;
        messages(candidateIdx) = candidateResult.message;
        solverCalls = solverCalls+candidateResult.solverCalls;
        if candidateResult.feasible
            linearObjectives(candidateIdx) = candidateResult.objectiveValue;
            if cfg.certification.enabled
                clfObjectives(candidateIdx) = candidateResult.clfValue;
                exactObjectives(candidateIdx) = ...
                    candidateResult.objectiveValue;
            else
                clfObjectives(candidateIdx) = 0.0;
                exactObjectives(candidateIdx) = localExactObjective( ...
                    candidateQp, candidateResult.decision, cfg);
            end
            if bestIdx == 0 || localObjectiveBetter( ...
                    exactObjectives(candidateIdx), ...
                    exactObjectives(bestIdx), cfg)
                bestIdx = candidateIdx;
            end
        end
    end
    if bestIdx == 0
        if all(exitFlags == -2)
            error("collisionAvoidanceController:noSolution", ...
                "No solution: the two-stage convex program is " ...
                + "infeasible from every start (%s). No admissible " ...
                + "input plan satisfies the hard separation rows " ...
                + "imposed from node %d, the model domain, the " ...
                + "friction rows and input bounds over the horizon; " ...
                + "nothing is issued.", strjoin(labels, ", "), ...
                localFirstImposedNode(qps{1}.collision));
        end
        error("collisionAvoidanceController:optimizationFailure", ...
            "The convex solver returned no plan (%s).", ...
            strjoin(labels+": "+messages, "; "));
    end

    qp = qps{bestIdx};
    result = results{bestIdx};
    decision = result.decision;
    layout = qp.layout;
    plan = localCommittedPlan(decision, qp);
    inputPlan = localHeadInputPlan(plan, layout);

    problem = struct();
    if ~buildPlanningProblem
        return;
    end
    problem.problemClass = qp.problemClass;
    problem.qp = qp;
    problem.layout = layout;
    problem.prediction = prediction;
    problem.nominalInput = nominalInput;
    problem.decision = decision;
    problem.inputPlan = inputPlan;
    problem.plan = plan;
    problem.tailPlan = plan(layout.tailIndex);
    problem.metadata = localPlanDiagnostics( ...
        qp, result, decision, model, prediction);
    problem.metadata.candidateCount = candidateCount;
    problem.metadata.candidateLabels = labels;
    problem.metadata.candidateExitFlags = exitFlags;
    problem.metadata.candidateObjectives = exactObjectives;
    problem.metadata.candidateClfValues = clfObjectives;
    problem.metadata.candidateLinearObjectives = linearObjectives;
    problem.metadata.candidateCertifiedInfeasible = certified;
    problem.metadata.candidateFactRelaxed = factRelaxedCounts;
    problem.metadata.candidateRefinementRungs = rungCounts;
    problem.metadata.candidateStartViolation = startViolations;
    problem.metadata.candidatesSolved = sum(~certified);
    problem.metadata.selectedCandidate = labels(bestIdx);
    problem.metadata.objectiveExact = exactObjectives(bestIdx);
    if cfg.certification.enabled
        problem.metadata.jointObjectiveValue = exactObjectives(bestIdx);
    end
    problem.metadata.trustRegionDropped = trustDroppedFlags(bestIdx);
    problem.metadata.factRelaxedNodes = factRelaxedCounts(bestIdx);
    problem.metadata.refinementRungs = rungCounts(bestIdx);
    problem.metadata.solverCallCount = solverCalls;
end

function better = localObjectiveBetter(candidate, incumbent, cfg)
    tolerance = cfg.solver.optimalityTolerance;
    better = candidate ...
        < incumbent-tolerance*(1.0+abs(incumbent));
end

function [inputPlan, problem, plan] = localSolveDisjunctive( ...
        model, prediction, nominalInput, buildPlanningProblem)
% THE DISJUNCTIVE PATH: one linearization, and the homotopy class a
% decision of the optimization rather than of a start.
%
% The exact collision-free condition is the OR over the configuration
% obstacle's facets, and here that OR is in the model
% (formulateTwoStageQp>localDisjunctionData). The program is solved by
% branch and bound over the facet assignment (localDisjunctiveSearch),
% so its answer is the best plan over EVERY class the model admits -
% not the best plan among the classes some set of starts happened to
% reach. Run to exhaustion it is the global optimum of the convexified
% program at this linearization, and it says so (`disjunctiveProven`);
% cut short by the node budget it is anytime, and reports the certified
% gap to the best any class could achieve.
%
% There is no start set here, no refinement ladder and no probe: with
% the disjunction explicit they have nothing left to do. The
% linearization is the shifted previous solution, which fixes the CLF
% tangents, the obstacle's facet directions (through the ego's yaw) and
% the trust region - and the receding horizon is the sequential convex
% iteration over it.
    cfg = model.cfg;
    [qp, result, info, trustDropped, factRelaxed] = ...
        localDisjunctiveCandidate(model, prediction, nominalInput, ...
            [], cfg);
    if ~result.feasible
        % The shifted plan is the feasible point Proposition 8 promises
        % under its premises; when the sample fails, how far it itself
        % is from the rows says which premise failed, and by how much.
        shortfall = localIncumbentShortfall(qp);
        if result.infeasible
            error("collisionAvoidanceController:noSolution", ...
                "No solution: the disjunctive program is infeasible. " ...
                + "No admissible input plan lies outside the " ...
                + "configuration obstacle at every node the input can " ...
                + "reach (%d imposed), whatever facet it is held to, " ...
                + "while satisfying the model domain, friction, tail " ...
                + "and terminal rows and the input bounds; the search " ...
                + "closed its whole tree (%d subproblems). %s Nothing " ...
                + "is issued.", info.imposedNodes, info.explored, ...
                shortfall);
        end
        error("collisionAvoidanceController:optimizationFailure", ...
            "The convex solver returned no plan (%s; %d subproblems). %s", ...
            result.message, info.explored, shortfall);
    end
    decision = result.decision;
    layout = qp.layout;
    plan = localCommittedPlan(decision, qp);
    inputPlan = localHeadInputPlan(plan, layout);

    problem = struct();
    if ~buildPlanningProblem
        return;
    end
    problem.problemClass = qp.problemClass;
    problem.qp = qp;
    problem.layout = layout;
    problem.prediction = prediction;
    problem.nominalInput = nominalInput;
    problem.decision = decision;
    problem.inputPlan = inputPlan;
    problem.plan = plan;
    problem.tailPlan = plan(layout.tailIndex);
    problem.metadata = localPlanDiagnostics( ...
        qp, result, decision, model, prediction);
    problem.metadata.candidateCount = 1;
    problem.metadata.candidateLabels = "disjunctive";
    problem.metadata.candidateExitFlags = result.exitFlag;
    problem.metadata.candidateObjectives = ...
        localExactObjective(qp, decision, cfg);
    problem.metadata.candidateLinearObjectives = result.objectiveValue;
    problem.metadata.candidateCertifiedInfeasible = qp.certifiedInfeasible;
    problem.metadata.candidateFactRelaxed = factRelaxed;
    problem.metadata.candidateRefinementRungs = 0;
    problem.metadata.candidateStartViolation = 0.0;
    problem.metadata.candidatesSolved = 1;
    problem.metadata.selectedCandidate = "disjunctive";
    problem.metadata.objectiveExact = ...
        problem.metadata.candidateObjectives;
    problem.metadata.trustRegionDropped = trustDropped;
    problem.metadata.factRelaxedNodes = factRelaxed;
    problem.metadata.refinementRungs = 0;
    problem.metadata.disjunctiveExplored = info.explored;
    problem.metadata.disjunctiveProven = info.proven;
    problem.metadata.disjunctiveGap = info.gap;
    problem.metadata.disjunctiveBound = info.bestBound;
    problem.metadata.disjunctiveAssigned = info.assignedNodes;
    problem.metadata.disjunctiveImposed = info.imposedNodes;
    problem.metadata.disjunctiveQueuePeak = info.queuePeak;
    problem.metadata.disjunctiveFellBack = info.fellBack;
    problem.metadata.disjunctiveUnresolved = info.unresolved;
    problem.metadata.solverCallCount = info.solverCalls;
end

function [qp, result, info, trustDropped, factRelaxed] = ...
        localDisjunctiveCandidate(model, prediction, ...
        linearizationInput, common, cfg)
% One disjunctive program, searched, with the same two repairs the
% single-normal candidate gets: the trust region is a device for the
% CLF linearization and no hard row may yield to it, and a program the
% search proves infeasible gets the fact relaxation.
    options = struct("label", "disjunctive", "obstacleMode", "hard", ...
        "disjunctive", true, ...
        "trustRegionScale", cfg.clf.trustRegionScale);
    [baseQp, common] = formulateTwoStageQp( ...
        model, prediction, linearizationInput, options, common);
    [qp, result, info] = localDisjunctiveSearch(baseQp, cfg);
    trustDropped = false;
    if ~result.feasible && ~baseQp.certifiedInfeasible && baseQp.trustRegion
        options.trustRegionScale = inf;
        baseQp = formulateTwoStageQp( ...
            model, prediction, linearizationInput, options, common);
        [qp, result, info] = localDisjunctiveSearch(baseQp, cfg);
        trustDropped = true;
    end
    factRelaxed = 0;
    if result.infeasible && ~qp.certifiedInfeasible
        [relaxedQp, droppedNodes] = localFactRelaxation(qp, cfg);
        if droppedNodes > 0
            qp = relaxedQp;
            result = localSolveCandidate(qp, cfg, struct());
            factRelaxed = droppedNodes;
        end
    end
    if ~result.feasible && info.fellBack && ~info.proven
        % A budget-exhausted search that found no incumbent, whose
        % fallback - the linearization's own assignment - is infeasible
        % even after the fact relaxation, has proved nothing about the
        % other assignments: an optimization failure, never "no
        % solution". (Measured: with the fallback's infeasibility
        % converted to a stall before the fact relaxation, the reference
        % scene lost one sample whose fallback the relaxation would have
        % repaired.)
        if result.infeasible
            verdict = "is infeasible after the fact relaxation";
        else
            verdict = "was left unresolved by the kernel";
        end
        result.infeasible = false;
        result.unresolved = true;
        result.exitFlag = 0;
        result.message = "disjunctive search spent its budget without " ...
            + "an incumbent and the linearization's own assignment " ...
            + verdict;
    end
end

function [qp, result, trustDropped, factRelaxed] = ...
        localHardCandidate(model, prediction, linearizationInput, ...
        common, cfg, label, policy)
% One candidate: the HARD program at `linearizationInput`, solved, with
% the two repairs that are not relaxations of it. The trust region is a
% device for the CLF linearization and a hard separation row must never
% yield to it, so an infeasible trust-bounded program is re-solved
% unbounded; and a program the kernel declares infeasible gets the fact
% relaxation (localFactRelaxation), which removes only near-window rows
% no admissible plan satisfies by more than one sample's mismatch.
    if cfg.certification.enabled
        options = struct("label", string(label), ...
            "obstacleMode", "certified", ...
            "trustRegionScale", inf);
        qp = formulateTwoStageQp( ...
            model, prediction, linearizationInput, options, common);
        result = solveHardCbfClf(qp, cfg);
        trustDropped = false;
        factRelaxed = 0;
        return;
    end
    options = struct("label", string(label), "obstacleMode", "hard", ...
        "trustRegionScale", cfg.clf.trustRegionScale);
    qp = formulateTwoStageQp( ...
        model, prediction, linearizationInput, options, common);
    result = localSolveCandidate(qp, cfg, policy);
    trustDropped = false;
    if ~result.feasible && ~qp.certifiedInfeasible && qp.trustRegion
        options.trustRegionScale = inf;
        qp = formulateTwoStageQp( ...
            model, prediction, linearizationInput, options, common);
        result = localSolveCandidate(qp, cfg, policy);
        trustDropped = true;
    end
    factRelaxed = 0;
    if result.infeasible && ~qp.certifiedInfeasible
        [relaxedQp, droppedNodes] = localFactRelaxation(qp, cfg);
        if droppedNodes > 0
            qp = relaxedQp;
            result = localSolveCandidate(qp, cfg, policy);
            factRelaxed = droppedNodes;
        end
    end
end

function [refinedInput, rungs, violation, solverCalls] = ...
        localRefineStart(model, prediction, common, cfg, startInput)
% LEGACY SEQUENTIAL CONVEX REFINEMENT of one start. Certified mode never
% calls this function because its CBF constraints have no slack.
%
% A start is only a place to evaluate stage 1; the hard program that
% follows needs a linearization that is itself separated from the
% obstacle, because then - its rows evaluating at it to the true signed
% distances minus the budgets (Lemma 4) - that trajectory is a feasible
% point of the hard program and the solve cannot fail for want of one.
% So: while the start's own stage-1 margins show a shortfall, re-solve
% an ELASTIC program at it (one priced slack per imposed target node,
% everything else hard) and take the solution as the next
% linearization, with the price rising along
% cfg.sequentialConvex.penaltySchedule. Stage 1 is recomputed at every
% rung, so the normals rotate with the iterate; that is what carries a
% start into another homotopy class, and it is why a start that runs
% through the obstacle - the cruise probe - comes out the other side of
% it rather than being pushed back the way it came.
%
% The loop is uniform and its length is decided by the data: a start
% already separated exits before solving anything, which is what the
% shifted previous solution normally does, so the receding horizon pays
% nothing for the machinery. A rung carries only the rows that decide
% whether a class is REACHABLE - road, speed and heading domain, and
% input box - and neither the friction polygons nor the
% CLF rows and their trust region (formulateTwoStageQp, obstacleMode),
% and is solved by the interior-point kernel: 0.06 s against 0.43 s for
% the full row set, for the same result. No rung is ever committed.
    refinedInput = startInput;
    rungs = 0;
    violation = 0.0;
    solverCalls = 0;
    schedule = cfg.sequentialConvex.penaltySchedule(:).';
    if isempty(schedule)
        return;
    end
    policy = struct("maxIterations", cfg.sequentialConvex.maxIterations, ...
        "retry", false, "algorithm", "interior-point-convex");
    for stepIdx = 1:numel(schedule)+1
        options = struct("label", "refinement", ...
            "obstacleMode", "elastic", ...
            "violationPenalty", schedule(min(stepIdx, numel(schedule))));
        qp = formulateTwoStageQp( ...
            model, prediction, refinedInput, options, common);
        violation = localStartViolation(qp);
        if violation <= cfg.sequentialConvex.violationTolerance ...
                || stepIdx > numel(schedule)
            return;
        end
        result = solveTwoStageQp(qp, cfg, policy);
        solverCalls = solverCalls+result.solverCalls;
        if ~result.feasible
            return;
        end
        refinedInput = result.decision(qp.layout.planIndex);
        rungs = stepIdx;
    end
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

function [qp, result, info] = localDisjunctiveSearch(baseQp, cfg)
% THE DISJUNCTION, SOLVED GLOBALLY: branch and bound over which facet
% of the configuration obstacle each node is outside of.
%
% The exact collision-free condition at a node is the OR over the
% obstacle's facets (formulateTwoStageQp>localDisjunctionData). A
% single convex program can hold only one disjunct, which is why
% freezing one normal makes the linearization decide the homotopy
% class. Here the choice is a decision of the optimization instead: the
% search minimizes the same objective over ALL assignments of facets to
% nodes, so its answer is the best plan in EVERY class the model
% admits, not the best plan in the class a start happened to reach.
%
% The tree needs neither binaries nor a big-M. A node of the tree is a
% partial assignment; its relaxation imposes the assigned nodes' rows
% and NOTHING at the unassigned ones, which is a genuine relaxation of
% the disjunctive program, so its optimum is a valid lower bound for
% the whole subtree. The root - nothing assigned - is the program
% without the target, whose optimum bounds the sample from below. If a
% relaxed solution already satisfies the disjunction at every imposed
% node then it is feasible for the disjunctive program and, being
% optimal for a relaxation, optimal for its subtree: it becomes the
% incumbent and the subtree closes. Otherwise the search branches on
% the node whose best facet is most violated, one child per facet that
% some admissible plan could satisfy at all (facetMaximum >= 0), each
% child inheriting the parent's bound. Best-first, pruned against the
% incumbent.
%
% Run to exhaustion the answer is the GLOBAL optimum of the convexified
% disjunctive program at this linearization, and `proven` says so. Cut
% A subproblem the kernel leaves unresolved - a stalled active set,
% not a declared infeasibility - decides nothing, so its subtree is
% abandoned rather than pruned and the tree no longer counts as
% closed: `proven` is false whenever any subproblem stalled, and the
% count is reported. Every child is handed its parent's solution as
% the point the kernel works about, which is what keeps them from
% stalling in the first place (measured: six of eight children hit the
% iteration budget without it).
%
% Cut short by cfg.disjunctive.nodeBudget the search is anytime: the incumbent is a
% feasible plan of some class and `gap` is the certified distance to
% the best any class could achieve. The one thing it never does is
% claim infeasibility it has not proved - a budget-exhausted search
% with no incumbent falls back to the LINEARIZATION'S OWN ASSIGNMENT
% (the facet each node's projection faces, which is exactly the
% single-normal program this design used before the disjunction was
% made explicit), and if that is infeasible too the sample is an
% optimization failure rather than "no solution".
%
% Node selection is dive-then-best-first, the standard remedy for a
% relaxation this weak: an unassigned node contributes no row at all,
% so the root bound is the obstacle-free optimum and bounds improve
% only as nodes are assigned. Until there is an incumbent the search
% DIVES - always expanding the child whose facet the current solution
% clears by the most - because a feasible plan is worth more than a
% bound; afterwards it is best-first, which is what closes the tree.
    disjunction = baseQp.disjunction;
    nodeCount = numel(disjunction);
    info = struct("explored", 0, "solverCalls", 0, "bestBound", -inf, ...
        "gap", inf, "proven", false, "assignedNodes", 0, ...
        "imposedNodes", 0, "queuePeak", 0, "fellBack", false, ...
        "unresolved", 0);
    empty = zeros(1, nodeCount);
    if nodeCount == 0 || ~any([disjunction.imposed]) ...
            || baseQp.certifiedInfeasible
        qp = localApplyAssignment(baseQp, empty);
        result = localSolveCandidate(qp, cfg, struct());
        info.explored = 1;
        info.solverCalls = result.solverCalls;
        info.proven = true;
        info.gap = 0.0;
        info.bestBound = result.objectiveValue;
        return;
    end
    imposed = find([disjunction.imposed]);
    info.imposedNodes = numel(imposed);
    budget = cfg.disjunctive.nodeBudget;
    incumbentBudget = max(budget, cfg.disjunctive.incumbentBudget);
    relativeGap = cfg.disjunctive.relativeGapTolerance;
    satisfactionTolerance = 10.0*cfg.solver.constraintTolerance;

    queue = struct("assignment", empty, "bound", -inf, ...
        "priority", 0.0, "dive", false, ...
        "warmStart", baseQp.initialDecision);
    bestObjective = inf;
    bestAssignment = empty;
    bestResult = [];
    % The node budget caps the subproblems spent on OPTIMALITY once an
    % incumbent exists; while none does, the search may go on to the
    % incumbent budget - a feasible plan is what the sample needs, and
    % the alternative is a lost sample.
    while ~isempty(queue) && (info.explored < budget ...
            || (isempty(bestResult) && info.explored < incumbentBudget))
        if isempty(bestResult)
            % Dive: the most promising child first - a dive child
            % before any single-node child - until something feasible
            % exists to prune against.
            [~, pick] = max([queue.priority]+1.0e12*[queue.dive]);
        else
            [~, pick] = min([queue.bound]);
        end
        entry = queue(pick);
        queue(pick) = [];
        if entry.bound >= bestObjective-localGapAllowance( ...
                bestObjective, relativeGap)
            continue;
        end
        nodeQp = localApplyAssignment(baseQp, entry.assignment);
        % A child differs from its parent by one node's two rows, so
        % the parent's solution is where its own optimum is looked for:
        % the kernel works in deviation coordinates about this point.
        % A dive child differs by many rows and its optimum is far from
        % the parent's - the probe rungs' case - so it goes to the
        % interior-point kernel; and while no incumbent exists a
        % stalled child gets the retry ladder, since a feasible plan is
        % worth the time and an abandoned subtree may be the only one.
        nodeQp.initialDecision = entry.warmStart;
        policy = struct("retry", isempty(bestResult));
        if entry.dive
            policy.algorithm = "interior-point-convex";
        end
        nodeResult = localSolveCandidate(nodeQp, cfg, policy);
        info.explored = info.explored+1;
        info.solverCalls = info.solverCalls+nodeResult.solverCalls;
        if nodeResult.infeasible && ~nodeQp.certifiedInfeasible ...
                && string(nodeResult.algorithm) ~= "interior-point-convex"
            % A declared infeasibility closes a subtree for good, and on
            % these degenerate children the active set has been measured
            % to declare it wrongly: a tree closed with no incumbent
            % while the linearization's own assignment - an extension
            % of one of its root-to-leaf paths, hence of a relaxation
            % it had called infeasible - then solved feasibly. One
            % interior-point confirmation is asked for before the
            % subtree is closed; a confirmation that stalls leaves the
            % subproblem undecided, which the tree counts as not closed.
            confirmation = localSolveCandidate(nodeQp, cfg, struct( ...
                "retry", false, "algorithm", "interior-point-convex"));
            info.solverCalls = info.solverCalls+confirmation.solverCalls;
            if confirmation.feasible
                nodeResult = confirmation;
            elseif ~confirmation.infeasible
                nodeResult.infeasible = false;
                nodeResult.unresolved = true;
            end
        end
        if ~nodeResult.feasible
            % A kernel that stalls has decided nothing. Pruning here
            % would be pruning a subtree that may hold the optimum, so
            % the subtree is abandoned and the tree is no longer closed:
            % whatever the search returns, it is not proved optimal.
            info.unresolved = info.unresolved ...
                + double(~nodeResult.infeasible);
            continue;
        end
        bound = nodeResult.objectiveValue;
        if bound >= bestObjective-localGapAllowance( ...
                bestObjective, relativeGap)
            continue;
        end
        [worstNode, facetMargin] = localDisjunctionResidual( ...
            disjunction, imposed, nodeResult.decision, ...
            baseQp.layout, satisfactionTolerance);
        if worstNode == 0
            bestObjective = bound;
            bestResult = nodeResult;
            bestAssignment = localCompleteAssignment( ...
                disjunction, imposed, facetMargin);
            continue;
        end
        candidateFacets = find(disjunction(worstNode).facetMaximum >= 0.0);
        margins = facetMargin{worstNode};
        for facetIdx = candidateFacets
            child = entry.assignment;
            child(worstNode) = facetIdx;
            queue(end+1) = struct("assignment", child, ...
                "bound", bound, ...
                "priority", entry.priority+margins(facetIdx), ...
                "dive", false, ...
                "warmStart", nodeResult.decision); %#ok<AGROW>
        end
        % THE DIVE CHILD: every node the relaxed solution violates,
        % assigned at once to the facet it clears by the most - one
        % subproblem standing where a chain of single-node children
        % would end. Its subtree lies inside one of the single-node
        % children's, so its optimum is a valid bound for that subtree
        % and the enumeration stays complete; it is expanded before any
        % single-node child while no incumbent exists. With the
        % terminal set's tail the disjunction holds over a hundred
        % nodes, and a dive that assigns one node per subproblem cannot
        % reach an incumbent within the budget (measured: samples lost
        % to a budget-exhausted search whose fallback was infeasible).
        [dive, divePriority, diveCount] = localDiveAssignment( ...
            disjunction, imposed, entry.assignment, facetMargin, ...
            entry.priority, satisfactionTolerance);
        if diveCount > 1
            queue(end+1) = struct("assignment", dive, ...
                "bound", bound, ...
                "priority", divePriority, ...
                "dive", true, ...
                "warmStart", nodeResult.decision); %#ok<AGROW>
        end
        info.queuePeak = max(info.queuePeak, numel(queue));
    end
    if isempty(queue) && info.unresolved == 0
        info.proven = true;
        info.bestBound = bestObjective;
        info.gap = 0.0;
    elseif isempty(queue)
        info.bestBound = -inf;
        info.gap = inf;
    else
        info.bestBound = min([queue.bound]);
        info.gap = (bestObjective-info.bestBound) ...
            / max(abs(bestObjective), 1.0e-9);
    end
    info.assignedNodes = sum(bestAssignment > 0);
    qp = localApplyAssignment(baseQp, bestAssignment);
    if isempty(bestResult)
        % Nothing feasible was found. Fall back to the linearization's
        % own assignment - the single-normal program - so the sample is
        % never worse than it would have been without the search; and
        % only an exhausted tree may call the sample infeasible.
        info.fellBack = true;
        qp = localApplyAssignment(baseQp, localProjectionAssignment( ...
            baseQp, imposed));
        result = localSolveCandidate(qp, cfg, struct());
        info.solverCalls = info.solverCalls+result.solverCalls;
        info.assignedNodes = numel(imposed);
        % The fallback's verdict is returned as the kernel gave it: an
        % infeasibility goes to the fact relaxation first
        % (localDisjunctiveCandidate), and only what survives that is
        % read against the tree - closed, "no solution"; not closed, an
        % optimization failure, since an unclosed tree proves nothing.
        return;
    end
    result = bestResult;
end

function [assignment, priority, count] = localDiveAssignment( ...
        disjunction, imposed, assignment, facetMargin, priority, ...
        tolerance)
% The dive child's assignment: every unassigned imposed node whose best
% admissible facet the relaxed solution still violates, assigned to
% that facet. Returns how many nodes it assigned.
    count = 0;
    for nodeIdx = imposed
        if assignment(nodeIdx) > 0
            continue;
        end
        margins = facetMargin{nodeIdx};
        if isempty(margins)
            continue;
        end
        margins(disjunction(nodeIdx).facetMaximum < 0.0) = -inf;
        [best, bestFacet] = max(margins);
        if ~isfinite(best) || best >= -tolerance
            continue;
        end
        assignment(nodeIdx) = bestFacet;
        priority = priority+best;
        count = count+1;
    end
end

function text = localIncumbentShortfall(qp)
% How far the linearization plan - the shifted previous plan, or the
% schedule reference - is from being a feasible point of the program
% it was linearized at: the largest violation of its own stage-1
% margins over the imposed nodes (Lemma 4 makes these the true signed
% distances less the budgets) and of the hard model, tail and terminal
% rows, by family and node. Reported in the failure message so that a
% lost sample names the premise that failed.
    nodes = qp.collision.nodes;
    worstNode = 0;
    worstMargin = 0.0;
    for nodeIdx = 1:numel(nodes)
        if nodes(nodeIdx).imposed && nodes(nodeIdx).nominalMargin < worstMargin
            worstMargin = nodes(nodeIdx).nominalMargin;
            worstNode = nodeIdx-1;
        end
    end
    residual = qp.inequalityMatrix*qp.initialDecision-qp.inequalityBound;
    hardRow = qp.rowFamily ~= "clf" & qp.rowFamily ~= "collision";
    worstRow = 0.0;
    worstFamily = "";
    worstRowNode = 0;
    if any(hardRow)
        [worstRow, rowIdx] = max(residual.*hardRow);
        worstFamily = qp.rowFamily(rowIdx);
        worstRowNode = qp.rowNode(rowIdx);
    end
    text = sprintf("The linearization plan's own shortfall: " ...
        + "separation %.4f m at node %d, hard rows %.3g " ...
        + "(%s, node %d).", -worstMargin, worstNode, ...
        max(worstRow, 0.0), worstFamily, worstRowNode);
end

function assignment = localProjectionAssignment(qp, imposed)
% The assignment the linearization itself points to: at each node the
% facet whose normal is nearest the direction stage 1's projection
% returned. It reproduces the single-normal program - the one the
% design solved before the disjunction was explicit - and is the
% search's fallback, so the disjunctive path can only improve on it.
    disjunction = qp.disjunction;
    assignment = zeros(1, numel(disjunction));
    for nodeIdx = imposed
        entry = disjunction(nodeIdx);
        [~, assignment(nodeIdx)] = max(entry.facetNormal.' ...
            * qp.collision.nodes(nodeIdx).normal);
    end
end

function allowance = localGapAllowance(bestObjective, relativeGap)
    if ~isfinite(bestObjective)
        allowance = 0.0;
        return;
    end
    allowance = max(relativeGap*abs(bestObjective), 1.0e-9);
end

function [worstNode, facetMargin] = localDisjunctionResidual( ...
        disjunction, imposed, decision, layout, tolerance)
% Which imposed node the plan is inside the obstacle at, and by how
% much each facet misses there. Zero means the plan satisfies the
% disjunction everywhere, i.e. it is collision-free for the model.
    worstNode = 0;
    worstValue = -tolerance;
    facetMargin = cell(1, numel(disjunction));
    planColumn = decision(layout.planIndex);
    for nodeIdx = imposed
        entry = disjunction(nodeIdx);
        margins = zeros(1, entry.facetCount);
        for facetIdx = 1:entry.facetCount
            rows = 2*(facetIdx-1)+(1:2);
            margins(facetIdx) = min(entry.facetMatrix(rows, :) ...
                * planColumn+entry.facetOffset(rows));
        end
        facetMargin{nodeIdx} = margins;
        best = max(margins);
        if best < worstValue
            worstValue = best;
            worstNode = nodeIdx;
        end
    end
end

function assignment = localCompleteAssignment( ...
        disjunction, imposed, facetMargin)
% The facet each imposed node is outside of, for a plan that satisfies
% the disjunction: the one it clears by the most. The program built on
% that assignment has the plan as a feasible point and the same
% objective, so it is the program the plan was optimal for.
    assignment = zeros(1, numel(disjunction));
    for nodeIdx = imposed
        margins = facetMargin{nodeIdx};
        if isempty(margins)
            continue;
        end
        [~, assignment(nodeIdx)] = max(margins);
    end
end

function qp = localApplyAssignment(qp, assignment)
% Install one facet per assigned node as the program's target rows, and
% write the choice into the readout so every diagnostic downstream sees
% the plan against the facet it was actually held to. An unassigned
% node contributes nothing - that is what makes a partial assignment a
% relaxation.
    disjunction = qp.disjunction;
    assigned = find(assignment > 0);
    rowCount = 2*numel(assigned);
    matrix = zeros(rowCount, qp.layout.decisionCount);
    bound = zeros(rowCount, 1);
    rowNode = zeros(rowCount, 1);
    rowWindow = false(rowCount, 1);
    rowAllowance = zeros(rowCount, 1);
    rowIdx = 0;
    for nodeIdx = assigned
        entry = disjunction(nodeIdx);
        facetIdx = assignment(nodeIdx);
        rows = 2*(facetIdx-1)+(1:2);
        for signIdx = 1:2
            rowIdx = rowIdx+1;
            matrix(rowIdx, qp.layout.planIndex) = ...
                -entry.facetMatrix(rows(signIdx), :);
            bound(rowIdx) = entry.facetOffset(rows(signIdx));
            rowNode(rowIdx) = nodeIdx;
        end
        qp.collision.nodes(nodeIdx).normal = entry.facetNormal(:, facetIdx);
        qp.collision.nodes(nodeIdx).regionCode = entry.facetRegion(facetIdx);
        qp.collision.nodes(nodeIdx).marginMatrix = entry.facetMatrix(rows, :);
        qp.collision.nodes(nodeIdx).marginOffset = entry.facetOffset(rows);
        qp.collision.nodes(nodeIdx).imposed = true;
        rowWindow(rowIdx-1:rowIdx) = entry.window;
        rowAllowance(rowIdx-1:rowIdx) = entry.allowance;
    end
    unassigned = setdiff(find([disjunction.imposed]), assigned);
    for nodeIdx = unassigned
        qp.collision.nodes(nodeIdx).imposed = false;
    end
    qp.inequalityMatrix = [matrix; qp.inequalityMatrix];
    qp.inequalityBound = [bound; qp.inequalityBound];
    qp.rowFamily = [repmat("collision", rowCount, 1); qp.rowFamily];
    qp.rowNode = [rowNode; qp.rowNode];
    qp.rowWindow = [rowWindow; qp.rowWindow];
    qp.rowAllowance = [rowAllowance; qp.rowAllowance];
end

function result = localSolveCandidate(qp, cfg, policy)
% One candidate through the kernel, or the certificate's verdict.
    if qp.certifiedInfeasible
        result = struct( ...
            "decision", zeros(0, 1), ...
            "exitFlag", -2, ...
            "feasible", false, ...
            "infeasible", true, ...
            "unresolved", false, ...
            "iterations", 0, ...
            "solverCalls", 0, ...
            "retried", false, ...
            "algorithm", "reachBoxCertificate", ...
            "message", "certified infeasible: an imposed separation " ...
                + "row exceeds what any admissible plan reaches", ...
            "objectiveValue", inf);
        return;
    end
    if qp.obstacleMode == "certified"
        result = solveHardCbfClf(qp, cfg);
        return;
    end
    result = solveTwoStageQp(qp, cfg, policy);
end

function [qp, droppedNodes] = localFactRelaxation(qp, cfg)
% Which near-window rows can no admissible plan satisfy jointly: the
% least total relaxation of the window rows that makes the program
% feasible, every other row held (one LP, only on an infeasible
% candidate). The rows that need relaxation - each by at most its
% node's allowance - are facts of the state and the previous plan:
% the friction polygon, which the reach box does not see, took their
% capacity away. They are dropped from the program and reported
% (collisionFactViolation covers them). If the program is infeasible
% even with the window relaxed, or a row needs more than one sample's
% mismatch, the conflict is beyond the input's authority and the
% candidate stays infeasible.
    persistent options
    if isempty(options)
        options = optimoptions("linprog", "Display", "none");
    end
    droppedNodes = 0;
    window = qp.rowWindow;
    if ~any(window)
        return;
    end
    rowCount = numel(qp.inequalityBound);
    decisionCount = qp.layout.decisionCount;
    windowCount = sum(window);
    augmented = [sparse(qp.inequalityMatrix), sparse(rowCount, windowCount)];
    augmented(window, decisionCount+(1:windowCount)) = -speye(windowCount);
    cost = [zeros(decisionCount, 1); ones(windowCount, 1)];
    lowerBound = [qp.lowerBound; zeros(windowCount, 1)];
    upperBound = [qp.upperBound; inf(windowCount, 1)];
    [solution, ~, exitFlag] = linprog(cost, augmented, ...
        qp.inequalityBound, [], [], lowerBound, upperBound, options);
    if exitFlag ~= 1
        return;
    end
    relaxation = solution(decisionCount+1:end);
    tolerance = max(cfg.solver.constraintTolerance, 1.0e-9);
    % A fact is mismatch-sized: a row that needs more than its node's
    % allowance - the mismatch it can have accumulated since it was
    % last a hard row - is a conflict, not a fact, and the candidate
    % stays infeasible.
    if any(relaxation > qp.rowAllowance(window)+tolerance)
        return;
    end
    droppedRows = false(rowCount, 1);
    droppedRows(window) = relaxation > tolerance;
    if ~any(droppedRows)
        return;
    end
    droppedNodes = numel(unique(qp.rowNode(droppedRows)));
    % The dropped rows' nodes become facts of their families.
    families = [{"collision", 1}; ...
        arrayfun(@(idx) {"road", idx}, 1:numel(qp.road), ...
            "UniformOutput", false).'];
    for familyIdx = 1:size(families, 1)
        name = families{familyIdx, 1};
        idx = families{familyIdx, 2};
        nodeList = unique(qp.rowNode(droppedRows & qp.rowFamily == name));
        for nodeIdx = nodeList(:).'
            if name == "collision"
                qp.collision.nodes(nodeIdx).imposed = false;
                qp.collision.nodes(nodeIdx).factSource = "relaxation";
            else
                qp.road(idx).nodes(nodeIdx).imposed = false;
                qp.road(idx).nodes(nodeIdx).factSource = "relaxation";
            end
        end
    end
    keep = ~droppedRows;
    qp.inequalityMatrix = qp.inequalityMatrix(keep, :);
    qp.inequalityBound = qp.inequalityBound(keep);
    qp.rowFamily = qp.rowFamily(keep);
    qp.rowNode = qp.rowNode(keep);
    qp.rowWindow = qp.rowWindow(keep);
    qp.rowAllowance = qp.rowAllowance(keep);
end

function value = localExactObjective(qp, decision, cfg)
% The objective with the CLF relaxations replaced by the solution's
% EXACT decrease residuals: what the linearized program approximates,
% and the only value comparable across candidates linearized at the
% same incumbent but solved in different homotopy classes.
    layout = qp.layout;
    inputColumn = decision(layout.inputIndex);
    inputHessian = qp.Hessian(layout.inputIndex, layout.inputIndex);
    inputCost = 0.5*inputColumn.'*inputHessian*inputColumn ...
        + qp.linear(layout.inputIndex).'*inputColumn+qp.constant;
    residual = max(localClfExactResiduals(qp.clf, ...
        decision(layout.planIndex)), 0.0);
    value = inputCost+cfg.clf.relaxationWeight*sum(residual);
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
% margins, the repair capacities and the facts, the CLF relaxations,
% the objective split, and the kernel's verdict.
    layout = qp.layout;
    metadata = struct();
    metadata.problemClass = qp.problemClass;
    metadata.horizonSteps = layout.horizonSteps;
    metadata.tailSteps = layout.tailSteps;
    metadata.trustRegion = qp.trustRegion;
    planColumn = decision(layout.planIndex);
    headNodeCount = prediction.headNodeCount;

    % The target family: node 1 is the measured state, the nodes below
    % the authority threshold are facts, the rest carry the rows - over
    % the head and the tail alike.
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
    % the controller's complete predicted continuation. The legacy
    % restContract fields below alias this imposed invariant condition.
    metadata = localTerminalDiagnostics(metadata, qp, planColumn, ...
        model, prediction);
    % The largest violation of a covered node the rows do NOT cover:
    % the measured state and the nodes below the authority threshold.
    factMargin = collision.margin;
    factMargin(~collision.covered | collision.imposed) = inf;
    metadata.collisionFactViolation = max([0.0, -factMargin]);
    metadata.collisionCapacityProfile = [nodes.capacity];
    metadata.collisionAuthorityProfile = [nodes.authority];
    metadata.collisionBoxMaximumProfile = [nodes.boxMaximum];
    metadata.collisionFactSourceProfile = [nodes.factSource];
    metadata.dualDistanceProfile = [nodes.dualDistance];
    metadata.dualRegionProfile = [nodes.regionCode];
    metadata.dualNormalProfile = [nodes.normal];
    metadata.nominalMarginProfile = [nodes.nominalMargin];
    if isempty(nodes)
        metadata.collisionCapacityProfile = zeros(1, 0);
        metadata.collisionAuthorityProfile = zeros(1, 0);
        metadata.collisionBoxMaximumProfile = zeros(1, 0);
        metadata.collisionFactSourceProfile = strings(1, 0);
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
    hardRow = qp.rowFamily ~= "clf";
    metadata.hardRowViolation = 0.0;
    if any(hardRow)
        metadata.hardRowViolation = max(0.0, max(residual(hardRow)));
    end
    metadata.rowCounts = struct( ...
        "collision", sum(qp.rowFamily == "collision"), ...
        "road", sum(qp.rowFamily == "road"), ...
        "speedDomain", sum(qp.rowFamily == "speedDomain"), ...
        "headingDomain", sum(qp.rowFamily == "headingDomain"), ...
        "friction", sum(qp.rowFamily == "friction"), ...
        "tail", sum(qp.rowFamily == "tail"), ...
        "terminal", sum(qp.rowFamily == "terminal"), ...
        "clfTrust", sum(qp.rowFamily == "clfTrust"), ...
        "clf", sum(qp.rowFamily == "clf"));
    % Nodes at which the CLF trust region binds: where the plan wanted
    % to move further from the nominal than the linearization allows.
    trustRow = qp.rowFamily == "clfTrust";
    metadata.clfTrustActiveNodes = 0;
    if any(trustRow)
        metadata.clfTrustActiveNodes = numel(unique( ...
            qp.rowNode(trustRow & residual >= -1.0e-6)));
    end

    % CLF relaxations and the exact decrease residual of the plan.
    relaxation = decision(layout.relaxationIndex);
    if model.cfg.certification.enabled
        relaxation = relaxation(1);
    end
    metadata.clfRelaxationProfile = relaxation(:).';
    metadata.clfRelaxation = sum(relaxation);
    metadata.clfRelaxationMax = max(relaxation);
    metadata.clfInitialValue = qp.clf.initialValue;
    [exactResidual, valueProfile, planError] = ...
        localClfExactResiduals(qp.clf, planColumn);
    metadata.clfValueProfile = valueProfile;
    if model.cfg.certification.enabled
        metadata.clfExactResidual = exactResidual(1)-relaxation(1);
    else
        metadata.clfExactResidual = max(exactResidual-relaxation(:).');
    end
    metadata.clfPlanError = planError;
    metadata.safetySlackProfile = zeros(1, 0);
    if ~model.cfg.certification.enabled && layout.slackCount > 0
        metadata.safetySlackProfile = ...
            decision(layout.slackIndex).';
    end
    metadata.cbfConstraintsHard = model.cfg.certification.enabled ...
        && layout.slackCount == 0 && isempty(layout.slackIndex);
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
    activeRelaxationIndex = layout.relaxationIndex(1:numel(relaxation));
    relaxationHessian = qp.Hessian( ...
        activeRelaxationIndex, activeRelaxationIndex);
    metadata.clfRelaxationCost = ...
        0.5*relaxation(:).'*relaxationHessian*relaxation(:) ...
        + qp.linear(activeRelaxationIndex).'*relaxation(:);
    metadata.jointObjectiveValue = metadata.inputDeviationCost ...
        + metadata.clfRelaxationCost;
    metadata.objectiveValue = metadata.jointObjectiveValue;

    % Kernel.
    metadata.solverExitFlag = result.exitFlag;
    metadata.solverIterations = result.iterations;
    metadata.solverRetried = result.retried;
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
    % Backward-compatible aliases now report the hard invariant row, not a
    % one-step monitor.
    metadata.restContractHolds = true;
    metadata.restContractMargin = inf;
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
    metadata.restContractMargin = metadata.terminalInvariantMargin;
    metadata.restContractHolds = metadata.terminalInvariantCertified;
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
    % AT MOST ONE TARGET: the reader admits a single record, so the
    % passing-side candidates exhaust the disjunction. The fields hold
    % neutral values - never read - when no target is present.
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
        model.cfg.certification.terminalSupportDirectionCount;
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
