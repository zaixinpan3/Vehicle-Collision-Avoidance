function [qp, common] = formulateAvoidanceProblem( ...
        model, prediction, linearizationInput, label, common)
% formulateAvoidanceProblem Build hard node constraints and exact CLF data.
% Freeze rectangle support normals at this start, then assemble the affine
% collision, road, model, braking-tail and terminal constraints. The sole
% relaxation belongs to the exact first-step CLF cone in solveHardCbfClf.
% Linearization-independent data are shared by the sample's two starts.
    if nargin < 5
        common = localCommonParts(model, prediction);
    end
    layout = common.layout;
    linearizationPlan = linearizationInput(:);
    nominalState = localNominalState(prediction, model, linearizationPlan);
    families = [localTargetFamily(model, prediction, nominalState, ...
            common.tightening, common.terminal); ...
        localRoadFamilies(model, prediction, nominalState, common.terminal)];
    [families, matrix, bound, rowFamily, rowNode, certifiedInfeasible] = ...
        localSeparationRows(families, prediction, common);
    qp = struct( ...
        "problemClass", "recursiveFeasibleHardCbfClfSocpPathCoordinates", ...
        "label", string(label), ...
        "certifiedInfeasible", certifiedInfeasible, "layout", layout, ...
        "Hessian", common.hessian, "linear", common.linear, ...
        "constant", common.constant, ...
        "inequalityMatrix", [matrix; common.modelMatrix], ...
        "inequalityBound", [bound; common.modelBound], ...
        "rowFamily", [rowFamily; common.modelFamily], ...
        "rowNode", [rowNode; common.modelNode], ...
        "lowerBound", common.lowerBound, "upperBound", common.upperBound, ...
        "collision", families(1), "road", families(2:end), ...
        "clf", localClfData(prediction, model, layout, common), ...
        "terminal", common.terminal, "linearizationPlan", linearizationPlan, ...
        "nominalState", nominalState);
end

function common = localCommonParts(model, prediction)
% Physical rows, input bounds, cruise objective and CLF certificate.
% Build once per sample and share across candidate starts.
    horizonSteps = model.horizonSteps;
    inputCount = prediction.inputCount;
    tailSteps = prediction.tailSteps;
    planCount = prediction.planCount;
    % Head inputs, tail accelerations and one first-step CLF relaxation.
    layout = struct( ...
        "inputDimension", model.inputDimension, ...
        "horizonSteps", horizonSteps, ...
        "tailSteps", tailSteps, ...
        "inputCount", inputCount, ...
        "planCount", planCount, ...
        "relaxationCount", 1, ...
        "decisionCount", planCount+1, ...
        "inputIndex", 1:inputCount, ...
        "tailIndex", inputCount+(1:tailSteps), ...
        "planIndex", 1:planCount, ...
        "relaxationIndex", planCount+1);

    common = struct();
    common.layout = layout;
    speedInterval = localPlannedSpeedInterval(prediction, model);
    common.terminal = localTerminalBudget(model, prediction, speedInterval);
    common.tightening = localTargetTightening(model, prediction, ...
        common.terminal);
    common.certificate = localClfCertificate(model);
    common.equilibrium = localCruiseEquilibriumProfile(model, prediction);
    [common.hessian, common.linear, common.constant] = localObjective( ...
        model, common.equilibrium.input, layout);

    [speedMatrix, speedBound] = localSpeedDomainRows( ...
        prediction, model, layout, speedInterval);
    [headingMatrix, headingBound] = localHeadingDomainRows( ...
        prediction, model, layout);
    % frictionCirclePolygonRows writes its rows as matrix*u + offset <= 0
    % over the plan columns; it touches the head inputs only.
    [frictionMatrix, frictionOffset] = ...
        frictionCirclePolygonRows(prediction, model);
    frictionMatrix = [frictionMatrix, zeros(size(frictionMatrix, 1), ...
        layout.decisionCount-size(frictionMatrix, 2))];
    frictionBound = -frictionOffset;
    [tailMatrix, tailBound] = localTailRows(prediction, model, layout);
    [terminalMatrix, terminalBound] = localTerminalRows( ...
        prediction, model, layout, common.terminal);
    common.modelMatrix = [speedMatrix; headingMatrix; frictionMatrix; ...
        tailMatrix; terminalMatrix];
    common.modelBound = [speedBound; headingBound; frictionBound; ...
        tailBound; terminalBound];
    common.modelFamily = [ ...
        repmat("speedDomain", numel(speedBound), 1); ...
        repmat("headingDomain", numel(headingBound), 1); ...
        repmat("friction", numel(frictionBound), 1); ...
        repmat("tail", numel(tailBound), 1); ...
        repmat("terminal", numel(terminalBound), 1)];
    common.modelNode = zeros(numel(common.modelBound), 1);
    [inputLowerBound, inputUpperBound] = localInputBox(model);
    [tailLowerBound, tailUpperBound] = localTailBox(model, layout);
    common.lowerBound = [inputLowerBound; tailLowerBound; 0.0];
    common.upperBound = [inputUpperBound; tailUpperBound; inf];
    common.probeInput = localCruiseProbeInput(model, prediction, common);
end

function nominalState = localNominalState(prediction, model, plan)
% The linearization trajectory at every node: the head by the stage
% rollout in O(N), the tail by its condensed map from the terminal state.
    inputPlan = reshape(plan(1:prediction.inputCount), ...
        model.inputDimension, model.horizonSteps);
    nominalState = zeros(6, prediction.nodeCount);
    nominalState(:, 1) = model.initialEgoState(:);
    for stageIdx = 1:model.horizonSteps
        nominalState(:, stageIdx+1) = ...
            prediction.stageMatrixA(:, :, stageIdx)*nominalState(:, stageIdx) ...
            + prediction.stageMatrixB(:, :, stageIdx)*inputPlan(:, stageIdx) ...
            + prediction.stageAffine(:, stageIdx);
    end
    for nodeIdx = prediction.tailNodeIndex
        nominalState(:, nodeIdx) = prediction.egoStateMatrix(:, :, nodeIdx) ...
            * plan+prediction.egoStateOffset(:, nodeIdx);
    end
end

function terminal = localTerminalBudget(model, prediction, speedInterval)
% THE TERMINAL SET's data for this sample (PCBF_CLF_ARCHITECTURE.md, "The
% terminal set"): the closed-form lateral certificate at the route's
% curvature maximum, the constant yaw radius and lateral drift every
% tail row charges, and the terminal rows' net bounds - the certified
% heading bound less the course contributions of the terminal lateral
% velocity (|vy|/vx at the terminal speed floor) and of the terminal
% yaw-rate error over the yaw mode's time constant, each net of the
% estimate radius at the terminal node.
    cfg = model.cfg;
    curvatureMaximum = max(abs(model.lane.segmentCurvature));
    certificate = terminalLateralCertificate(cfg, curvatureMaximum);
    terminalNode = prediction.headNodeCount;
    radii = prediction.egoStateErrorBound(:, terminalNode);
    speedFloor = max(speedInterval.plannedLower(terminalNode) ...
        - radii(4), cfg.model.speedMinimum);
    lateralVelocityBound = cfg.terminal.lateralVelocityMaximum;
    yawRateErrorBound = cfg.terminal.yawRateErrorMaximum;
    courseRadius = lateralVelocityBound/speedFloor ...
        + yawRateErrorBound*certificate.yawTimeConstant;
    headingRow = certificate.headingBound-courseRadius-radii(3);
    lateralVelocityRow = lateralVelocityBound-radii(5);
    yawRateRow = yawRateErrorBound-radii(6)-curvatureMaximum*radii(4);
    if headingRow <= 0.0 || lateralVelocityRow <= 0.0 || yawRateRow <= 0.0
        error("collisionAvoidanceController:invalidFormulation", ...
            "The terminal rows have no budget left: heading %.4f rad, " ...
            + "lateral velocity %.4f m/s, yaw-rate error %.4f rad/s " ...
            + "after the course contributions and the estimate radii " ...
            + "at the terminal node.", headingRow, lateralVelocityRow, ...
            yawRateRow);
    end
    terminal = struct( ...
        "certificate", certificate, ...
        "headingRadius", certificate.headingBound, ...
        "lateralDrift", certificate.gammaLateral*certificate.headingBound, ...
        "courseRadius", courseRadius, ...
        "headingRow", headingRow, ...
        "lateralVelocityRow", lateralVelocityRow, ...
        "yawRateRow", yawRateRow, ...
        "terminalNode", terminalNode);
end

function [matrix, bound] = localTailRows(prediction, model, layout)
% HARD rows of the kinematic braking tail (kinematicBrakingTail): the
% nonnegative speed at every tail node and REST - zero speed - at the
% last one. The box [-a_b, 0] and the zero last acceleration are bounds
% (localTailBox). Speeds are nondimensionalized by the speed maximum.
    cfg = model.cfg;
    tailSteps = layout.tailSteps;
    speedMaximum = cfg.model.speedMaximum;
    rowCount = tailSteps+1;
    matrix = zeros(rowCount, layout.decisionCount);
    bound = zeros(rowCount, 1);
    rowIdx = 0;
    for stageIdx = 1:tailSteps
        nodeIdx = prediction.tailNodeIndex(stageIdx);
        speedRow = prediction.egoStateMatrix(4, :, nodeIdx);
        speedOffset = prediction.egoStateOffset(4, nodeIdx);
        rowIdx = rowIdx+1;
        matrix(rowIdx, layout.planIndex) = -speedRow/speedMaximum;
        bound(rowIdx) = speedOffset/speedMaximum;
    end
    restNode = prediction.tailNodeIndex(end);
    rowIdx = rowIdx+1;
    matrix(rowIdx, layout.planIndex) = ...
        prediction.egoStateMatrix(4, :, restNode)/speedMaximum;
    bound(rowIdx) = -prediction.egoStateOffset(4, restNode)/speedMaximum;
end

function [lowerBound, upperBound] = localTailBox(model, layout)
% The tail accelerations lie in [-a_b, 0]; the last one is zero, so
% that rest is rest under the appended zero input of the shifted
% candidate.
    lowerBound = repmat(-model.cfg.terminal.backupDeceleration, ...
        layout.tailSteps, 1);
    upperBound = zeros(layout.tailSteps, 1);
    lowerBound(end) = 0.0;
end

function [matrix, bound] = localTerminalRows( ...
        prediction, ~, layout, terminal)
% HARD handoff rows at the terminal node, the conditions under which
% the tail's kinematic lateral certificate applies to the dynamic
% bicycle's terminal state: |ePsi_N| within the certified heading
% budget, |vy_N| within the declared lateral-velocity bound and the
% yaw-rate error |r_N - kappa_N vx_N| within its bound, each net of
% the estimate radii (localTerminalBudget). Nondimensional by the
% bounds.
    node = terminal.terminalNode;
    stateMatrix = prediction.egoStateMatrix(:, :, node);
    stateOffset = prediction.egoStateOffset(:, node);
    curvature = prediction.scheduleCurvature(node);
    rows = { ...
        stateMatrix(3, :), stateOffset(3), terminal.headingRow; ...
        stateMatrix(5, :), stateOffset(5), terminal.lateralVelocityRow; ...
        stateMatrix(6, :)-curvature*stateMatrix(4, :), ...
            stateOffset(6)-curvature*stateOffset(4), terminal.yawRateRow};
    matrix = zeros(2*size(rows, 1), layout.decisionCount);
    bound = zeros(2*size(rows, 1), 1);
    for rowIdx = 1:size(rows, 1)
        valueRow = rows{rowIdx, 1};
        valueOffset = rows{rowIdx, 2};
        limit = rows{rowIdx, 3};
        matrix(2*rowIdx-1, layout.planIndex) = valueRow/limit;
        bound(2*rowIdx-1) = (limit-valueOffset)/limit;
        matrix(2*rowIdx, layout.planIndex) = -valueRow/limit;
        bound(2*rowIdx) = (limit+valueOffset)/limit;
    end
end

function equilibrium = localCruiseEquilibriumProfile(model, prediction)
% The cruise equilibrium of the declared model at every node's
% schedule curvature and the published longitudinal bias: the state
% the CLF error is measured against and the input the objective is
% measured against. A function of the route and the estimator only.
    nodeCount = model.horizonSteps+1;
    equilibrium = struct( ...
        "input", zeros(model.inputDimension, model.horizonSteps), ...
        "lateralVelocity", zeros(1, nodeCount), ...
        "yawRate", zeros(1, nodeCount));
    for nodeIdx = 1:nodeCount
        point = cruiseEquilibrium(model.referenceSpeed, ...
            prediction.scheduleCurvature(nodeIdx), ...
            model.longitudinalAccelerationBias, model.cfg);
        equilibrium.lateralVelocity(nodeIdx) = point.lateralVelocity;
        equilibrium.yawRate(nodeIdx) = point.yawRate;
        if nodeIdx <= model.horizonSteps
            equilibrium.input(:, nodeIdx) = [point.steeringAngle; ...
                point.longitudinalAcceleration];
        end
    end
end

function inputPlan = localCruiseProbeInput(model, prediction, common)
% Roll out the Riccati cruise feedback without a target to supply a second
% separation linearization. Clamp its inputs to the physical bounds; the
% resulting hard-constrained optimization determines the committed plan.
    cfg = model.cfg;
    horizonSteps = model.horizonSteps;
    gain = common.certificate.feedbackGain;
    accelerationMinimum = cfg.actuation.longitudinalAccelerationMinimum;
    accelerationMaximum = cfg.actuation.longitudinalAccelerationMaximum;
    lower = [-cfg.model.frontWheelSteeringAngleMaximum; accelerationMinimum];
    upper = [cfg.model.frontWheelSteeringAngleMaximum; accelerationMaximum];
    inputPlan = zeros(model.inputDimension, horizonSteps);
    state = model.initialEgoState;
    for stageIdx = 1:horizonSteps
        error = state(2:6)-[0.0; 0.0; model.referenceSpeed; ...
            common.equilibrium.lateralVelocity(stageIdx); ...
            common.equilibrium.yawRate(stageIdx)];
        input = common.equilibrium.input(:, stageIdx)-gain*error;
        input = min(max(input, lower), upper);
        inputPlan(:, stageIdx) = input;
        state = prediction.stageMatrixA(:, :, stageIdx)*state ...
            + prediction.stageMatrixB(:, :, stageIdx)*input ...
            + prediction.stageAffine(:, stageIdx);
    end
    % Initialize the tail with the fastest admissible braking profile.
    tail = kinematicBrakingTail("profile", cfg, prediction.tailSteps, ...
        min(max(state(4), 0.0), cfg.model.speedMaximum));
    inputPlan = [inputPlan(:); tail(:)];
end

function node = localEmptyObstacleNode(inputCount)
    node = struct( ...
        "covered", false, ...
        "imposed", false, ...
        "normal", zeros(2, 1), ...
        "regionCode", 0, ...
        "supportValue", 0.0, ...
        "targetSupport", 0.0, ...
        "egoSupport", 0.0, ...
        "headingCoefficient", 0.0, ...
        "nominalHeading", 0.0, ...
        "tightening", 0.0, ...
        "dualDistance", inf, ...
        "nominalMargin", inf, ...
        "terminalInvariant", false, ...
        "terminalContinuationAxis", "", ...
        "terminalFutureSupport", inf, ...
        "terminalSupportDirection", zeros(2, 1), ...
        "terminalSegmentEnforced", false, ...
        "terminalSegmentIndex", 0, ...
        "terminalStationLower", -inf, ...
        "terminalStationUpper", inf, ...
        "outside", true, ...
        "marginMatrix", zeros(2, inputCount), ...
        "marginOffset", zeros(2, 1));
end

function family = localEmptyFamily(name, id, nodeCount, inputCount)
    family = struct("name", string(name), "id", string(id), ...
        "nodes", repmat(localEmptyObstacleNode(inputCount), ...
            nodeCount, 1));
end

function family = localTargetFamily(model, prediction, nominalState, ...
        tightening, terminal)
% STAGE 1 on the TRUE configuration obstacle. At every node the exact
% Minkowski sum of the target rectangle at its predicted pose and the
% ego rectangle at the linearization's pose - both in path coordinates,
% both as oriented rectangles, neither replaced by a bounding box -
% and the exact projection of the linearization's centre onto it
% (rectangleConfigurationDistance): the signed distance, the
% supporting normal n_k, and the polygon's support in that direction.
% Outside, n_k points from the closest boundary point to the centre;
% inside - the probe that has run into the obstacle - it is the outward
% normal of the least-penetrated edge, the same minimum-penetration
% answer for a polygon of any shape. Node 1 is the measured state and
% carries the same hard constraint as every other covered node.
%
% The support is split back into the two rectangles' own supports,
% because stage 2 needs them separately: the target's is a constant of
% the node, the ego's moves with the decision's heading. Each is taken
% as the MAXIMUM over the declared yaw uncertainty of that rectangle
% (localRectangleSupport).
%
% TAIL NODES carry the same row with a CONSTANT ego support: the tail's
% lane-keeping law keeps the ego's heading within the certified band
% about zero (terminalLateralCertificate), so the ego's support is
% maximized over that band once and the row has no heading term, and
% the band's lateral drift is in the node's tightening. The facet set
% is still taken at the linearization's terminal heading.
    nodeCount = prediction.nodeCount;
    family = localEmptyFamily("collision", model.targetKey, nodeCount, ...
        prediction.planCount);
    if ~model.hasTarget
        return;
    end
    target = model.targetPath;
    horizon = model.targetHorizon;
    halfDimensions = [model.egoHalfLength; model.egoHalfWidth; ...
        model.targetHalfLength; model.targetHalfWidth];
    for nodeIdx = 1:nodeCount
        nominal = nominalState(:, nodeIdx);
        centre = [target.station(nodeIdx); target.lateral(nodeIdx)];
        targetHeading = target.headingError(nodeIdx);
        % The configuration obstacle about the TARGET CENTRE: path
        % stations run to hundreds of metres, and the projection is
        % done in metres of relative position.
        [distance, normal, ~, outside] = ...
            rectangleConfigurationDistance( ...
                nominal(1:2)-centre, nominal(3), zeros(2, 1), ...
                targetHeading, halfDimensions);
        [egoYaw, egoYawRadius, slopeRange] = localEgoYawData( ...
            model, prediction, terminal, nodeIdx, nominal(3));
        targetYawRadius = horizon.targetYawErrorBound(nodeIdx);
        node = localEmptyObstacleNode(prediction.planCount);
        node.covered = true;
        node.normal = normal;
        node.regionCode = localRegionCode(normal);
        node.targetSupport = localRectangleSupport( ...
            model.targetHalfLength, model.targetHalfWidth, normal, ...
            targetHeading, targetYawRadius);
        node.egoSupport = localRectangleSupport( ...
            model.egoHalfLength, model.egoHalfWidth, normal, ...
            egoYaw, egoYawRadius);
        % n' z <= supportValue for every z of the obstacle, the ego's
        % part evaluated at the linearization's heading (or over the
        % tail's band).
        node.supportValue = normal.'*centre+node.targetSupport ...
            + node.egoSupport;
        node.headingCoefficient = localSupportSlopeBoundOnRange( ...
            model, normal, slopeRange);
        node.nominalHeading = nominal(3);
        node.tightening = tightening.base(nodeIdx);
        node.dualDistance = distance;
        node.nominalMargin = normal.'*nominal(1:2)-node.supportValue ...
            - node.tightening;
        node.outside = outside;
        family.nodes(nodeIdx) = node;
    end
    family.nodes(nodeCount) = localTerminalInvariantTargetNode( ...
        model, prediction, nominalState(:, nodeCount), ...
        family.nodes(nodeCount), terminal);
end

function node = localTerminalInvariantTargetNode( ...
        model, prediction, nominalState, node, terminal)
% Hard target-conditioned terminal condition. For each fixed inertial
% direction, the target predictor supplies the support of its COMPLETE
% future centre trajectory from the rest time. The best terminal
% half-space on the fixed direction grid separates the resting ego from
% that entire prediction. Rectangle circumradii cover every future target
% and ego yaw. No stationary, straight, monotone, curvature, or target
% motion-class predicate is used.
%
% The Frenet-to-Cartesian map is exactly affine on one lane polyline
% segment. Two additional hard rows keep the terminal station on the
% segment selected at the incumbent, so the inertial separating row is
% valid for every admitted decision and exact at the shifted incumbent.
    restNode = prediction.nodeCount;
    tolerance = model.cfg.controller.shiftConsistencyTolerance;
    [segmentIdx, affineOrigin, tangent, lateralAxis, stationLower, ...
        stationUpper] = localTerminalLaneFrame( ...
            model.lane, nominalState(1), tolerance);
    egoCentre = affineOrigin+tangent*nominalState(1) ...
        + lateralAxis*nominalState(2);
    direction = model.targetHorizon.terminalSupportDirection;
    futureSupport = ...
        model.targetHorizon.terminalFuturePositionSupport;
    targetRadius = hypot(model.targetHalfLength, model.targetHalfWidth);
    egoRadius = hypot(model.egoHalfLength, model.egoHalfWidth);
    coefficient = [direction.'*tangent, direction.'*lateralAxis];
    baseTightening = model.cfg.collision.clearanceMargin ...
        + sum(model.targetHorizon.targetPositionErrorBound(:, restNode)) ...
        + terminal.lateralDrift;
    egoError = prediction.egoStateErrorBound(1:2, restNode);
    directionTightening = baseTightening ...
        + abs(coefficient)*egoError;
    margin = direction.'*egoCentre-futureSupport.' ...
        - targetRadius-egoRadius-directionTightening;
    margin(~isfinite(futureSupport.')) = -inf;
    [~, directionIdx] = max(margin);
    invariantModel = any(isfinite(futureSupport));
    if invariantModel
        inertialNormal = direction(:, directionIdx);
        selectedSupport = futureSupport(directionIdx);
        axisName = "predictedTrajectorySupport";
    else
        inertialNormal = direction(:, 1);
        selectedSupport = 0.0;
        axisName = "noFinitePredictedTrajectorySupport";
    end
    normal = [dot(inertialNormal, tangent); ...
        dot(inertialNormal, lateralAxis)];
    node.normal = normal;
    node.regionCode = localRegionCode(normal);
    node.targetSupport = targetRadius;
    node.egoSupport = egoRadius;
    node.supportValue = selectedSupport+targetRadius+egoRadius ...
        - inertialNormal.'*affineOrigin;
    node.headingCoefficient = 0.0;
    node.nominalHeading = nominalState(3);
    node.tightening = baseTightening+abs(normal).'*egoError;
    node.dualDistance = normal.'*nominalState(1:2)-node.supportValue;
    node.nominalMargin = node.dualDistance-node.tightening;
    node.outside = node.nominalMargin >= 0.0;
    node.terminalInvariant = invariantModel;
    node.terminalContinuationAxis = axisName;
    node.terminalFutureSupport = selectedSupport;
    node.terminalSupportDirection = inertialNormal;
    node.terminalSegmentEnforced = true;
    node.terminalSegmentIndex = segmentIdx;
    node.terminalStationLower = stationLower;
    node.terminalStationUpper = stationUpper;
end

function [segmentIdx, affineOrigin, tangent, lateralAxis, ...
        stationLower, stationUpper] = localTerminalLaneFrame( ...
        lane, station, tolerance)
    segmentIdx = find(lane.segmentStation <= station, 1, "last");
    if isempty(segmentIdx)
        segmentIdx = 1;
    end
    segmentIdx = min(segmentIdx, numel(lane.segmentLength));
    stationLower = lane.segmentStation(segmentIdx);
    stationUpper = stationLower+lane.segmentLength(segmentIdx);
    if segmentIdx < numel(lane.segmentLength)
        guard = max(10.0*tolerance*(1.0+abs(stationUpper)), ...
            100.0*eps(1.0+abs(stationUpper)));
        guard = min(guard, 0.25*lane.segmentLength(segmentIdx));
        stationUpper = stationUpper-guard;
    end
    tangent = lane.tangent(segmentIdx, :).';
    lateralAxis = [-tangent(2); tangent(1)];
    affineOrigin = lane.segmentStart(segmentIdx, :).' ...
        - tangent*stationLower;
end

function [egoYaw, egoYawRadius, slopeRange] = localEgoYawData( ...
        model, prediction, terminal, nodeIdx, nominalHeading)
% What the ego's yaw is at a node for the support rows. Head nodes: the
% linearization's heading with its estimate radius, and the heading
% interval the program admits for the Lipschitz charge. Tail nodes: the
% certified band about zero, and no Lipschitz charge (an empty range),
% since the tail's heading is not a decision.
    if nodeIdx <= prediction.headNodeCount
        egoYaw = nominalHeading;
        egoYawRadius = prediction.egoStateErrorBound(3, nodeIdx);
        slopeRange = localHeadingRange(model, prediction, nodeIdx, ...
            nominalHeading);
    else
        egoYaw = 0.0;
        egoYawRadius = terminal.headingRadius;
        slopeRange = zeros(1, 0);
    end
end

function slope = localSupportSlopeBoundOnRange(model, normal, range)
    if isempty(range)
        slope = 0.0;
        return;
    end
    slope = localSupportSlopeBound(model.egoHalfLength, ...
        model.egoHalfWidth, normal, range(1), range(2));
end

function value = localRectangleSupport(halfLength, halfWidth, normal, ...
        yaw, yawRadius)
% The support of an oriented rectangle about its centre, in the unit
% direction n, maximized over the declared yaw uncertainty:
%
%   h(psi) = l |cos(alpha - psi)| + w |sin(alpha - psi)|,
%   value  = max_{|delta| <= r} h(yaw + delta),   alpha = angle(n).
%
% h is pi-periodic and, between its kinks at multiples of pi/2, equals
% R cos(theta -+ phi) with R = hypot(l, w) and phi = atan2(w, l): its
% maxima are at theta = +-phi + k pi, where it reaches R. The maximum
% over an interval is therefore the larger endpoint value, or R when a
% maximizer falls inside. Exact, and the direction-wise treatment of a
% yaw radius that an axis-aligned box can only approximate.
    alpha = atan2(normal(2), normal(1));
    low = alpha-yaw-yawRadius;
    high = alpha-yaw+yawRadius;
    value = max(localSupportAtAngle(halfLength, halfWidth, low), ...
        localSupportAtAngle(halfLength, halfWidth, high));
    if yawRadius <= 0.0
        return;
    end
    phase = atan2(halfWidth, halfLength);
    for phaseSign = [-1.0, 1.0]
        maximizer = phaseSign*phase ...
            + pi*ceil((low-phaseSign*phase)/pi);
        if maximizer <= high
            value = hypot(halfLength, halfWidth);
            return;
        end
    end
end

function value = localSupportAtAngle(halfLength, halfWidth, angle)
    value = halfLength*abs(cos(angle))+halfWidth*abs(sin(angle));
end

function slope = localSupportSlopeBound(halfLength, halfWidth, normal, ...
        yawLow, yawHigh)
% THE CONSERVATIVE AFFINE BOUND ON THE EGO'S YAW. Stage 2 freezes n but
% not the ego's heading, which the decision moves; the support
% h(psi) = l |cos(alpha - psi)| + w |sin(alpha - psi)| is not affine in
% psi, so the row charges
%
%   h(psi) <= h(psibar) + slope * |psi - psibar|,
%
% two affine rows (sigma = +-1) whose minimum is the bound. This
% returns the exact Lipschitz constant of h over the interval the
% program admits: max |dh/dpsi| there. With theta = alpha - psi,
% dh/dtheta = -l sgn(cos) sin + w sgn(sin) cos is sinusoidal of
% amplitude R = hypot(l, w) between the kinks at multiples of pi/2, so
% its magnitude is maximized at an interval endpoint, at a kink (where
% it is l or w) or at the sinusoid's own peak (where it is R) when that
% falls inside. Exact, hence never worse than the retired box bound
% (which charged w along the station axis and l along the lateral one,
% and their sum for a diagonal normal); for a diagonal normal at the
% declared heading domain it is measurably smaller.
    alpha = atan2(normal(2), normal(1));
    low = alpha-yawHigh;
    high = alpha-yawLow;
    radius = hypot(halfLength, halfWidth);
    if ~(high > low)
        slope = radius;
        return;
    end
    kinks = ceil(low/(pi/2))*(pi/2):(pi/2):high;
    edges = unique([low, kinks, high]);
    slope = 0.0;
    for pieceIdx = 1:numel(edges)-1
        pieceLow = edges(pieceIdx);
        pieceHigh = edges(pieceIdx+1);
        middle = 0.5*(pieceLow+pieceHigh);
        cosineSign = localNonzeroSign(cos(middle));
        sineSign = localNonzeroSign(sin(middle));
        % dh/dtheta = cosineCoefficient cos(theta) + sineCoefficient sin(theta)
        cosineCoefficient = halfWidth*sineSign;
        sineCoefficient = -halfLength*cosineSign;
        value = max( ...
            abs(cosineCoefficient*cos(pieceLow) ...
                + sineCoefficient*sin(pieceLow)), ...
            abs(cosineCoefficient*cos(pieceHigh) ...
                + sineCoefficient*sin(pieceHigh)));
        peak = atan2(sineCoefficient, cosineCoefficient);
        for shift = -2:2
            candidate = peak+shift*pi;
            if candidate > pieceLow && candidate < pieceHigh
                value = radius;
            end
        end
        slope = max(slope, value);
    end
end

function value = localNonzeroSign(value)
    if value < 0.0
        value = -1.0;
    else
        value = 1.0;
    end
end

function range = localHeadingRange(model, prediction, nodeIdx, ...
        nominalHeading)
% The interval the ego's heading error can take at this node: the
% declared heading domain widened by the node's estimate radius, and
% by the linearization's own heading (a probe may sit outside the
% domain). The slope bound must hold over every value the program
% admits AND at the point it is anchored.
    bound = model.cfg.model.headingDomainRadius ...
        + prediction.egoStateErrorBound(3, nodeIdx);
    range = [min(-bound, nominalHeading), max(bound, nominalHeading)];
end

function code = localRegionCode(normal)
% Which way the separating direction faces, for the diagnostics and for
% the escape trigger: 1 behind, 2 ahead, 3 left, 4 right, 5 rear-left,
% 6 rear-right, 7 front-left, 8 front-right. The tolerance is angular
% (about 5 degrees) rather than exact: on the true configuration
% polygon the edge normals carry the ego's and the target's own
% headings, so a normal is almost never exactly axis-aligned, and what
% the trigger asks is whether a row has any lateral coefficient worth
% the name.
    tolerance = 0.087;
    if abs(normal(2)) <= tolerance
        code = 1+double(normal(1) > 0.0);
    elseif abs(normal(1)) <= tolerance
        code = 3+double(normal(2) < 0.0);
    else
        code = 5+2*double(normal(1) > 0.0)+double(normal(2) < 0.0);
    end
end

function tightening = localTargetTightening(model, prediction, terminal)
% Per-node budget of the target rows, charged in the direction of
% separation: the clearance margin (the executable d_min), the ego and
% target position radii, and the curvature sagitta - the largest
% deviation of a rectangle edge from the chord between its corners when
% the path frame is curved, 1/2 kappa_max (l_e^2 + l_T^2). The YAW
% radii are not here: they are taken directionally, inside the support
% of each rectangle (localRectangleSupport), which is exact where an
% inflated bounding box was conservative. Tail nodes add the lateral
% drift of the certified band, gamma_d times the heading bound.
    nodeCount = prediction.nodeCount;
    tightening = struct("base", zeros(1, nodeCount));
    if ~model.hasTarget
        return;
    end
    horizon = model.targetHorizon;
    sagitta = 0.5*max(abs(model.lane.segmentCurvature)) ...
        * (model.egoHalfLength^2+model.targetHalfLength^2);
    for nodeIdx = 1:nodeCount
        egoRadii = prediction.egoStateErrorBound(:, nodeIdx);
        positionRadius = egoRadii(1) ...
            + sum(horizon.targetPositionErrorBound(:, nodeIdx));
        tightening.base(nodeIdx) = model.cfg.collision.clearanceMargin ...
            + positionRadius+sagitta;
        if nodeIdx > prediction.headNodeCount
            tightening.base(nodeIdx) = tightening.base(nodeIdx) ...
                + terminal.lateralDrift;
        end
    end
end

function families = localRoadFamilies(model, prediction, nominalState, ...
        terminal)
% STAGE 1 for the road boundaries: each is a half-plane obstacle in
% path coordinates whose dual is trivial - the separating direction is
% the boundary's inward normal, n = [0; -1] for a left boundary (the ego
% stays right of it), n = [0; 1] for a right one, with support
% h_O(n) = -+d_b. The boundary curve is sampled over its finite
% parameter range, projected onto the lane, and its lateral offset
% interpolated at every schedule station. The tightening carries the
% clearance margin, the position and yaw radii, the declared fit budget
% and the boundary's lateral variation over the stations the plan can
% reach about the schedule (the speed domain over the elapsed time). A
% node whose reachable stations are not covered by the finite section
% is uncovered: perceptionLimited boundaries carry no row there, strict
% ones are a declared input error.
    boundaries = model.road.boundaries;
    boundaryCount = numel(boundaries);
    nodeCount = prediction.nodeCount;
    headNodeCount = prediction.headNodeCount;
    families = repmat(localEmptyFamily("road", "", nodeCount, ...
        prediction.planCount), boundaryCount, 1);
    if boundaryCount == 0
        return;
    end
    cfg = model.cfg;
    speedDeviation = cfg.model.speedMaximum-model.plannedSpeedFloor;
    terminalReach = speedDeviation*(headNodeCount-1)*model.sampleTime;
    for boundaryIdx = 1:boundaryCount
        boundary = boundaries(boundaryIdx);
        families(boundaryIdx).id = string(boundary.boundaryId);
        [boundaryStation, boundaryLateral] = localBoundaryPath( ...
            boundary, model.lane);
        if mean(boundaryLateral) > 0.0
            normal = [0.0; -1.0];
        else
            normal = [0.0; 1.0];
        end
        for nodeIdx = 1:nodeCount
            station = prediction.scheduleStation(nodeIdx);
            radii = prediction.egoStateErrorBound(:, nodeIdx);
            if nodeIdx <= headNodeCount
                reach = speedDeviation*(nodeIdx-1)*model.sampleTime ...
                    + radii(1)+model.egoHalfLength;
                range = station+[-reach, reach];
            else
                % A tail node's station lies between the terminal
                % node's reach and that reach advanced by the fastest
                % admissible tail, whatever the schedule's own braking
                % profile did.
                tailTime = (nodeIdx-headNodeCount)*model.sampleTime;
                terminalStation = prediction.scheduleStation(headNodeCount);
                range = [terminalStation-terminalReach, ...
                    terminalStation+terminalReach ...
                        + cfg.model.speedMaximum*tailTime] ...
                    + [-1.0, 1.0]*(radii(1)+model.egoHalfLength);
            end
            tolerance = cfg.road.parameterRangeTolerance;
            if range(2) < boundaryStation(1)-tolerance ...
                    || range(1) > boundaryStation(end)+tolerance
                continue;
            end
            covered = range(1) >= boundaryStation(1)-tolerance ...
                && range(2) <= boundaryStation(end)+tolerance;
            if ~covered
                if boundary.coveragePolicy == "perceptionLimited"
                    continue;
                end
                error("collisionAvoidanceController:" ...
                        + "roadBoundaryCoverageGap", ...
                    "The stations the plan can reach at node %d are " ...
                    + "only partly covered by finite road boundary " ...
                    + "%s. Supply a segment covering them; the " ...
                    + "controller does not extend a fitted curve.", ...
                    nodeIdx, boundary.boundaryId);
            end
            offsetHere = interp1(boundaryStation, boundaryLateral, ...
                min(max(station, boundaryStation(1)), boundaryStation(end)));
            inRange = boundaryStation >= range(1) ...
                & boundaryStation <= range(2);
            variation = max(abs(boundaryLateral(inRange)-offsetHere));
            if isempty(variation)
                variation = 0.0;
            end
            nominal = nominalState(:, nodeIdx);
            [egoYaw, egoYawRadius, slopeRange] = localEgoYawData( ...
                model, prediction, terminal, nodeIdx, nominal(3));
            node = localEmptyObstacleNode(prediction.planCount);
            node.covered = true;
            node.normal = normal;
            node.regionCode = localRegionCode(normal);
            % The same geometry as the target family: the ego's own
            % support in the boundary's inward normal, at the
            % linearization's heading and over its yaw radius, with the
            % exact slope bound for the decision's heading (over the
            % certified band, with no slope, at a tail node).
            node.egoSupport = localRectangleSupport( ...
                model.egoHalfLength, model.egoHalfWidth, normal, ...
                egoYaw, egoYawRadius);
            node.supportValue = normal(2)*offsetHere+node.egoSupport;
            node.headingCoefficient = localSupportSlopeBoundOnRange( ...
                model, normal, slopeRange);
            node.nominalHeading = nominal(3);
            node.tightening = cfg.collision.clearanceMargin+radii(2) ...
                + boundary.normalDistanceErrorBound+variation;
            if nodeIdx > headNodeCount
                node.tightening = node.tightening+terminal.lateralDrift;
            end
            node.dualDistance = normal.'*nominal(1:2)-node.supportValue;
            node.nominalMargin = node.dualDistance-node.tightening;
            node.outside = node.dualDistance > 0.0;
            families(boundaryIdx).nodes(nodeIdx) = node;
        end
    end
end

function [station, lateral] = localBoundaryPath(boundary, lane)
% Sample the finite local-quadratic boundary and project it onto the
% lane: its stations and lateral offsets, sorted by station.
    sampleCount = max(2, ceil(diff(boundary.parameterRange)/0.5)+1);
    parameter = linspace(boundary.parameterRange(1), ...
        boundary.parameterRange(2), sampleCount);
    station = zeros(1, sampleCount);
    lateral = zeros(1, sampleCount);
    for sampleIdx = 1:sampleCount
        point = boundary.origin ...
            + boundary.longitudinalDirection*parameter(sampleIdx) ...
            + boundary.lateralDirection ...
                * polyval(boundary.coefficients, parameter(sampleIdx));
        projection = laneProjection(point, lane);
        station(sampleIdx) = projection.station;
        lateral(sampleIdx) = projection.lateralPosition;
    end
    [station, order] = sort(station);
    lateral = lateral(order);
    [station, keep] = unique(station, "stable");
    lateral = lateral(keep);
end

% ====================================================================
% Stage 2: the affine separation rows
% ====================================================================

function [families, matrix, bound, rowFamily, rowNode, ...
        certifiedInfeasible] = localSeparationRows(families, prediction, common)
% Impose both signs of the heading-support bound at every covered node.
% Constant stage-0 rows remain hard and reject an initially unsafe state.
    layout = common.layout;
    totalRows = 0;
    for familyIdx = 1:numel(families)
        totalRows = totalRows+2*sum([families(familyIdx).nodes.covered]);
        totalRows = totalRows ...
            + 2*sum([families(familyIdx).nodes.terminalSegmentEnforced]);
    end
    matrix = zeros(totalRows, layout.decisionCount);
    bound = zeros(totalRows, 1);
    rowFamily = strings(totalRows, 1);
    rowNode = zeros(totalRows, 1);
    rowIdx = 0;
    certifiedInfeasible = false;
    for familyIdx = 1:numel(families)
        nodes = families(familyIdx).nodes;
        for nodeIdx = 1:numel(nodes)
            node = nodes(nodeIdx);
            if ~node.covered
                continue;
            end
            stateMatrix = prediction.egoStateMatrix(:, :, nodeIdx);
            stateOffset = prediction.egoStateOffset(:, nodeIdx);
            positionRow = node.normal.'*stateMatrix(1:2, :);
            positionOffset = node.normal.'*stateOffset(1:2);
            for signIdx = 1:2
                secantSign = 3.0-2.0*signIdx;
                node.marginMatrix(signIdx, :) = positionRow ...
                    - secantSign*node.headingCoefficient*stateMatrix(3, :);
                node.marginOffset(signIdx) = positionOffset ...
                    - secantSign*node.headingCoefficient ...
                        * (stateOffset(3)-node.nominalHeading) ...
                    - node.supportValue-node.tightening;
            end
            if node.terminalSegmentEnforced && ~node.terminalInvariant
                certifiedInfeasible = true;
            end
            node.imposed = true;
            if node.imposed
                for signIdx = 1:2
                    rowIdx = rowIdx+1;
                    matrix(rowIdx, layout.planIndex) = ...
                        -node.marginMatrix(signIdx, :);
                    bound(rowIdx) = node.marginOffset(signIdx);
                    rowFamily(rowIdx) = families(familyIdx).name;
                    rowNode(rowIdx) = nodeIdx;
                end
            end
            if node.terminalSegmentEnforced
                % Keep the terminal Frenet station on the polyline
                % segment whose Cartesian map defines the invariant
                % support row. These are hard terminal-domain rows and
                % never receive a safety slack.
                stationRow = stateMatrix(1, :);
                stationOffset = stateOffset(1);
                stationScale = max(1.0, ...
                    node.terminalStationUpper-node.terminalStationLower);
                rowIdx = rowIdx+1;
                matrix(rowIdx, layout.planIndex) = ...
                    stationRow/stationScale;
                bound(rowIdx) = ...
                    (node.terminalStationUpper-stationOffset)/stationScale;
                rowFamily(rowIdx) = "terminalSegment";
                rowNode(rowIdx) = nodeIdx;
                rowIdx = rowIdx+1;
                matrix(rowIdx, layout.planIndex) = ...
                    -stationRow/stationScale;
                bound(rowIdx) = ...
                    (stationOffset-node.terminalStationLower)/stationScale;
                rowFamily(rowIdx) = "terminalSegment";
                rowNode(rowIdx) = nodeIdx;
            end
            families(familyIdx).nodes(nodeIdx) = node;
        end
    end
    matrix = matrix(1:rowIdx, :);
    bound = bound(1:rowIdx);
    rowFamily = rowFamily(1:rowIdx);
    rowNode = rowNode(1:rowIdx);
end

function clf = localClfData(prediction, model, layout, common)
% Exact cruise-error quadratics. Only the first transition is constrained;
% the remaining head nodes supply the predicted CLF value diagnostics.
    certificate = common.certificate;
    equilibrium = common.equilibrium;
    nodeCount = model.horizonSteps+1;
    errorDimension = numel(certificate.errorStateOrder);
    errorMatrix = zeros(errorDimension, layout.planCount, nodeCount);
    errorOffset = zeros(errorDimension, nodeCount);
    for nodeIdx = 1:nodeCount
        errorMatrix(:, :, nodeIdx) = ...
            prediction.egoStateMatrix(2:6, :, nodeIdx);
        errorOffset(:, nodeIdx) = prediction.egoStateOffset(2:6, nodeIdx) ...
            - [0.0; 0.0; model.referenceSpeed; ...
                equilibrium.lateralVelocity(nodeIdx); equilibrium.yawRate(nodeIdx)];
    end
    clf = struct( ...
        "certificate", certificate, ...
        "lyapunovMatrix", certificate.lyapunovMatrix, ...
        "decreaseMatrix", model.cfg.clf.decreaseRateFraction ...
            * certificate.decreaseMatrix, ...
        "errorMatrix", errorMatrix, "errorOffset", errorOffset, ...
        "equilibriumInput", equilibrium.input, ...
        "initialValue", errorOffset(:, 1).' ...
            * certificate.lyapunovMatrix*errorOffset(:, 1));
end

function certificate = localClfCertificate(model)
% Riccati CLF certificate of the path-frame cruise error.
%
% The error state [d; ePsi; vx - vRef; vy; r] is the [d; ePsi; vx; vy;
% r] block of the Frenet stage map at the straight reference cruise
% (kappa = 0) - the station row does not feed back into it. The
% discrete Riccati solution P and gain K certify, for the
% unconstrained linearized error dynamics,
%
%   V(e+) - V(e) = -e' (Q + K' R K) e,
%
% the direction-wise decrease the rows demand (scaled by the
% configured fraction). Input saturation and nonlinearity make the
% certificate local; the relaxation absorbs the states where the
% demanded contraction is not achievable.
    persistent memoKey memoCertificate
    cfg = model.cfg;
    minimumAcceleration = cfg.actuation.longitudinalAccelerationMinimum;
    maximumAcceleration = cfg.actuation.longitudinalAccelerationMaximum;
    accelerationScale = max( ...
        abs(minimumAcceleration), abs(maximumAcceleration));
    key = struct( ...
        "referenceSpeed", max(model.referenceSpeed, ...
            cfg.clf.certificateSpeedFloor), ...
        "sampleTime", model.sampleTime, ...
        "errorScale", [ ...
            cfg.clf.lateralPositionErrorScale; ...
            cfg.clf.headingErrorScale; ...
            cfg.clf.speedErrorScale; ...
            cfg.clf.lateralVelocityErrorScale; ...
            cfg.clf.yawRateErrorScale], ...
        "inputWeight", [ ...
            cfg.clf.frontWheelSteeringAngleWeight; ...
            cfg.clf.longitudinalAccelerationWeight], ...
        "inputScale", [ ...
            cfg.model.frontWheelSteeringAngleMaximum; ...
            accelerationScale], ...
        "vehicle", cfg.vehicle, ...
        "corneringStiffness", cfg.tire.corneringStiffness);
    if ~isempty(memoKey) && isequaln(key, memoKey)
        certificate = memoCertificate;
        return;
    end
    [stageMatrixA, stageMatrixB] = ltvBicycleStageMatrices( ...
        0.0, key.referenceSpeed, key.sampleTime, cfg);
    errorIndex = 2:6;
    errorStateMatrix = stageMatrixA(errorIndex, errorIndex);
    errorInputMatrix = stageMatrixB(errorIndex, :);
    stateWeight = diag(1.0./key.errorScale.^2);
    inputWeight = diag(key.inputWeight./key.inputScale.^2);
    try
        [feedbackGain, lyapunovMatrix] = dlqr( ...
            errorStateMatrix, errorInputMatrix, stateWeight, inputWeight);
    catch riccatiException
        error("collisionAvoidanceController:invalidFormulation", ...
            "The CLF Riccati synthesis at the reference cruise " ...
            + "failed: %s", riccatiException.message);
    end
    decreaseMatrix = stateWeight+feedbackGain.'*inputWeight*feedbackGain;
    decreaseMatrix = 0.5*(decreaseMatrix+decreaseMatrix.');
    decreaseEigenvalue = min(real(eig(decreaseMatrix, lyapunovMatrix)));
    certificate = struct( ...
        "lyapunovMatrix", lyapunovMatrix, ...
        "feedbackGain", feedbackGain, ...
        "decreaseMatrix", decreaseMatrix, ...
        "certifiedDecreaseRate", ...
            min(max(decreaseEigenvalue, 1.0e-6), 1.0), ...
        "errorStateOrder", ["lateralError"; "headingError"; ...
            "speedError"; "lateralVelocity"; "yawRateError"]);
    memoKey = key;
    memoCertificate = certificate;
end

function equilibrium = cruiseEquilibrium( ...
        referenceSpeed, curvature, accelerationBias, cfg)
% cruiseEquilibrium Steady cruise target of the declared model.
%
% Given the demanded cruise speed, the local path curvature, and the
% estimator's longitudinal model bias, returns the state and input
% that make the DECLARED model stationary in the path frame. Steady
% cornering of the linear-cornering bicycle at speed v and yaw rate
% r = curvature*v distributes the centripetal demand over the axles by
% the moment balance, Fyf = lr/L m v r, Fyr = lf/L m v r; the rear slip
% fixes the sideslip and the front slip the steering angle (the
% classic understeer form), and the longitudinal equilibrium cancels
% the bias and the centripetal cross term of vxdot = a + vy r + b.
% Aiming the CLF at this fixed point instead of the kinematic guess is
% what makes the closed loop stationary at zero error with no
% integrator.
    if referenceSpeed <= 0.0
        error("collisionAvoidanceController:invalidInput", ...
            "cruiseEquilibrium requires a positive reference speed.");
    end
    mass = cfg.vehicle.m;
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;
    wheelbase = lf+lr;
    corneringStiffness = double(cfg.tire.corneringStiffness(:));
    if isscalar(corneringStiffness)
        corneringStiffness = repmat(corneringStiffness, 2, 1);
    end
    yawRate = curvature*referenceSpeed;
    lateralForceTotal = mass*referenceSpeed*yawRate;
    frontLateralForce = lr/wheelbase*lateralForceTotal;
    rearLateralForce = lf/wheelbase*lateralForceTotal;
    rearSlipAngle = -rearLateralForce/corneringStiffness(2);
    lateralVelocity = referenceSpeed*rearSlipAngle+lr*yawRate;
    frontSlipAngle = frontLateralForce/corneringStiffness(1);
    steeringAngle = frontSlipAngle ...
        + (lateralVelocity+lf*yawRate)/referenceSpeed;
    longitudinalAcceleration = -accelerationBias-lateralVelocity*yawRate;
    equilibrium = struct( ...
        "referenceSpeed", referenceSpeed, ...
        "curvature", curvature, ...
        "lateralVelocity", lateralVelocity, ...
        "yawRate", yawRate, ...
        "steeringAngle", steeringAngle, ...
        "longitudinalAcceleration", longitudinalAcceleration);
end

% ====================================================================
% Objective, bounds, and the hard model rows
% ====================================================================

function [hessian, linear, constant] = localObjective( ...
        model, equilibriumInput, layout)
% Minimum intervention: the deviation of the whole input plan from the
% cruise equilibrium input, per stage and channel at the Riccati input
% weights over the amplitude scales, plus the price of the CLF
% relaxation: 0.5 z'Hz + f'z + constant.
    cfg = model.cfg;
    [inputWeight, inputScale] = localInputWeightAndScale(cfg);
    hessian = zeros(layout.decisionCount);
    linear = zeros(layout.decisionCount, 1);
    constant = 0.0;
    for stageIdx = 1:layout.horizonSteps
        for inputIdx = 1:layout.inputDimension
            columnIdx = (stageIdx-1)*layout.inputDimension+inputIdx;
            weight = model.sampleTime*inputWeight(inputIdx) ...
                / inputScale(inputIdx)^2;
            reference = equilibriumInput(inputIdx, stageIdx);
            hessian(columnIdx, columnIdx) = 2.0*weight;
            linear(columnIdx) = -2.0*weight*reference;
            constant = constant+weight*reference^2;
        end
    end
    % The tail accelerations are proof-side - the witness that rest is
    % reachable - and carry no price: a curvature six decades below
    % the head's keeps the Hessian positive definite and, among the
    % admissible tails, selects the gentlest.
    tailCurvature = 1.0e-6*model.sampleTime*inputWeight(2)/inputScale(2)^2;
    for tailIdx = layout.tailIndex
        hessian(tailIdx, tailIdx) = 2.0*tailCurvature;
    end
    % The CLF relaxation has only its configured linear price. It carries
    % no quadratic curvature.
    for relaxationIdx = layout.relaxationIndex
        linear(relaxationIdx) = cfg.clf.relaxationWeight;
    end
end

function [inputWeight, inputScale] = localInputWeightAndScale(cfg)
    minimumAcceleration = cfg.actuation.longitudinalAccelerationMinimum;
    maximumAcceleration = cfg.actuation.longitudinalAccelerationMaximum;
    accelerationScale = max( ...
        abs(minimumAcceleration), abs(maximumAcceleration));
    inputScale = [ ...
        cfg.model.frontWheelSteeringAngleMaximum; accelerationScale];
    inputWeight = [ ...
        cfg.clf.frontWheelSteeringAngleWeight; ...
        cfg.clf.longitudinalAccelerationWeight];
end

function [lowerBound, upperBound] = localInputBox(model)
% Physical input box of [deltaF; a] over the horizon.
    cfg = model.cfg;
    accelerationMinimum = cfg.actuation.longitudinalAccelerationMinimum;
    accelerationMaximum = cfg.actuation.longitudinalAccelerationMaximum;
    steeringLimit = cfg.model.frontWheelSteeringAngleMaximum;
    lowerBound = repmat([-steeringLimit; accelerationMinimum], ...
        model.horizonSteps, 1);
    upperBound = repmat([steeringLimit; accelerationMaximum], ...
        model.horizonSteps, 1);
end

function interval = localPlannedSpeedInterval(prediction, model)
% The interval the hard speed rows put the PLANNED speed vx_k in at
% every node, and the radius that separates it from the realized one:
%
%   plannedLower_k <= vx_k(z) <= plannedUpper_k,
%   plannedUpper_k = speedMaximum - radius_k,
%   plannedLower_k = min(floor, releaseSpeed_k - radius_k) + radius_k,
%
% so the realized speed stays inside [floor_k, speedMaximum]. The floor
% is the planned minimum, lowered at every node to the speed the
% LEAST-BRAKING ADMISSIBLE release reaches there - maximum acceleration
% with the schedule steering, through the model. Below that speed the
% row would be a fact no input can change, and a hard row there is
% either true or infeasible, never a constraint.
    cfg = model.cfg;
    horizonSteps = model.horizonSteps;
    speedFloor = model.plannedSpeedFloor;
    speedMaximum = cfg.model.speedMaximum;
    accelerationMaximum = cfg.actuation.longitudinalAccelerationMaximum;
    release = repmat(accelerationMaximum, 1, horizonSteps);
    releaseSteering = prediction.referenceInput(1, :);
    releaseInput = [releaseSteering; release];
    releasePlan = [releaseInput(:); zeros(prediction.tailSteps, 1)];
    nodeCount = horizonSteps+1;
    interval = struct( ...
        "plannedLower", zeros(1, nodeCount), ...
        "plannedUpper", zeros(1, nodeCount), ...
        "radius", zeros(1, nodeCount), ...
        "width", max(speedMaximum-speedFloor, 1.0));
    for nodeIdx = 1:nodeCount
        releaseSpeed = ...
            prediction.egoStateMatrix(4, :, nodeIdx)*releasePlan ...
            + prediction.egoStateOffset(4, nodeIdx);
        radius = prediction.egoStateErrorBound(4, nodeIdx);
        interval.radius(nodeIdx) = radius;
        interval.plannedLower(nodeIdx) = ...
            min(speedFloor, releaseSpeed-radius)+radius;
        interval.plannedUpper(nodeIdx) = speedMaximum-radius;
    end
end

function [matrix, bound] = localSpeedDomainRows(prediction, model, ...
        layout, interval)
% Hard longitudinal-speed rows from the first step on, the interval of
% localPlannedSpeedInterval nondimensionalized by its width.
    horizonSteps = model.horizonSteps;
    intervalWidth = interval.width;
    matrix = zeros(2*horizonSteps, layout.decisionCount);
    bound = zeros(2*horizonSteps, 1);
    for nodeIdx = 2:horizonSteps+1
        speedRow = prediction.egoStateMatrix(4, :, nodeIdx);
        speedOffset = prediction.egoStateOffset(4, nodeIdx);
        rowIdx = 2*(nodeIdx-2)+1;
        matrix(rowIdx, layout.planIndex) = speedRow/intervalWidth;
        bound(rowIdx) = ...
            (interval.plannedUpper(nodeIdx)-speedOffset)/intervalWidth;
        matrix(rowIdx+1, layout.planIndex) = -speedRow/intervalWidth;
        bound(rowIdx+1) = ...
            (speedOffset-interval.plannedLower(nodeIdx))/intervalWidth;
    end
end

function [matrix, bound] = localHeadingDomainRows( ...
        prediction, model, layout)
% Hard validity rows of the heading linearization from the third node
% on: |ePsi| <= headingDomainRadius net of the measured yaw radius,
% nondimensional. The first two nodes are facts of the measured state.
    cfg = model.cfg;
    domainRadius = cfg.model.headingDomainRadius;
    horizonSteps = model.horizonSteps;
    matrix = zeros(2*(horizonSteps-1), layout.decisionCount);
    bound = zeros(2*(horizonSteps-1), 1);
    for nodeIdx = 3:horizonSteps+1
        headingRow = prediction.egoStateMatrix(3, :, nodeIdx);
        headingOffset = prediction.egoStateOffset(3, nodeIdx);
        radius = min(prediction.egoStateErrorBound(3, nodeIdx), ...
            0.5*domainRadius);
        rowIdx = 2*(nodeIdx-3)+1;
        matrix(rowIdx, layout.planIndex) = headingRow/domainRadius;
        bound(rowIdx) = (domainRadius-radius-headingOffset)/domainRadius;
        matrix(rowIdx+1, layout.planIndex) = -headingRow/domainRadius;
        bound(rowIdx+1) = (domainRadius-radius+headingOffset)/domainRadius;
    end
end
