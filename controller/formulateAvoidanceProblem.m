function qp = formulateAvoidanceProblem(model, prediction, anchorPlan, geometry)
%formulateAvoidanceProblem One convex domain with a carried safety witness.
% The only slack is the continuous-time CLF relaxation. Every stage uses
% the same bicycle, actuator, geometry and model-domain constraints.

    arguments
        model (1,1) struct
        prediction (1,1) struct
        anchorPlan (:,1) double
        geometry = []
    end
    cfg = model.cfg;
    count = prediction.planCount;
    layout = struct("inputDimension", 2, "horizonSteps", model.horizonSteps, ...
        "tailSteps", prediction.tailSteps, "inputCount", prediction.inputCount, ...
        "planCount", count, "relaxationCount", 1, "decisionCount", count+1, ...
        "inputIndex", 1:prediction.inputCount, "tailIndex", prediction.tailIndex, ...
        "planIndex", 1:count, "relaxationIndex", count+1);
    nominalState = squeeze(pagemtimes(prediction.egoStateMatrix, anchorPlan)) ...
        + prediction.egoStateOffset;
    geometry = avoidanceSafetyGeometry(model, prediction, nominalState, geometry);
    families = [geometry.collision; geometry.road];
    [frictionMatrix, frictionOffset, frictionRows, frictionConstant] = ...
        axleFriction.polygonRows(prediction, model);
    covered = arrayfun(@(family) nnz([family.nodes.covered]), families);
    rowCount = 2*sum(covered)+8*prediction.nodeCount+numel(frictionOffset);
    matrix = zeros(rowCount, count+1);
    bound = zeros(rowCount, 1);
    rowFamily = strings(rowCount, 1);
    rowNode = zeros(rowCount, 1);
    localRows = zeros(rowCount, 8);
    localBound = zeros(rowCount, 1);
    stateNode = zeros(rowCount, 1);
    inputStage = zeros(rowCount, 1);
    cursor = 0;
    for familyIdx = 1:numel(families)
        nodes = families(familyIdx).nodes;
        selected = find([nodes.covered]);
        if isempty(selected), continue; end
        normals = reshape([nodes(selected).normal], 2, []);
        slope = [nodes(selected).headingCoefficient];
        support = [nodes(selected).supportValue]+[nodes(selected).tightening];
        heading = [nodes(selected).nominalHeading];
        signs = [1.0; -1.0];
        stateMap = prediction.egoStateMatrix(:, :, selected);
        positionMap = pagemtimes(reshape(normals, 1, 2, []), stateMap(1:2, :, :));
        headingMap = reshape(slope, 1, 1, []).*stateMap(3, :, :);
        margins = [positionMap-headingMap; positionMap+headingMap];
        offset = prediction.egoStateOffset(:, selected);
        marginOffset = sum(normals.*offset(1:2, :), 1)-support ...
            - signs.*slope.*(offset(3, :)-heading);
        values = reshape(num2cell(margins, [1, 2]), 1, []);
        [nodes(selected).marginMatrix] = values{:};
        values = num2cell(marginOffset, 1);
        [nodes(selected).marginOffset] = values{:};
        [nodes(selected).imposed] = deal(true);
        families(familyIdx).nodes = nodes;
        range = cursor+(1:2*numel(selected));
        matrix(range, 1:count) = -reshape(permute(margins, [1, 3, 2]), [], count);
        bound(range) = marginOffset(:);
        rowFamily(range) = families(familyIdx).name;
        rowNode(range) = repelem(selected(:), 2);
        localRows(range, 1:2) = repelem(-normals.', 2, 1);
        localRows(range, 3) = reshape(signs.*slope, [], 1);
        localBound(range) = reshape(-support+signs.*slope.*heading, [], 1);
        stateNode(range) = repelem(selected(:)-1, 2);
        cursor = cursor+numel(range);
    end
    geometry.collision = families(1);
    geometry.road = families(2:end);
    scale = [1.0; cfg.model.lateralDomainRadius; ...
        cfg.model.headingDomainRadius; cfg.model.speedMaximum];
    nodeCount = prediction.nodeCount;
    lower = [[geometry.frames.stationLower]; ...
        repmat([-cfg.model.lateralDomainRadius; -cfg.model.headingDomainRadius; 0.0], 1, nodeCount)] ...
        + prediction.egoStateErrorBound(1:4, :);
    upper = [[geometry.frames.stationUpper]; ...
        repmat([cfg.model.lateralDomainRadius; cfg.model.headingDomainRadius; cfg.model.speedMaximum], 1, nodeCount)] ...
        - prediction.egoStateErrorBound(1:4, :);
    maps = prediction.egoStateMatrix(1:4, :, :)./scale;
    maps = [maps; -maps];
    offset = prediction.egoStateOffset(1:4, :);
    limits = [(upper-offset)./scale; (offset-lower)./scale];
    range = cursor+(1:8*nodeCount);
    matrix(range, 1:count) = reshape(permute(maps, [1, 3, 2]), [], count);
    bound(range) = limits(:);
    rowFamily(range) = repmat(["routeDomain"; "lateralDomain"; ...
        "headingDomain"; "speedDomain"], 2*nodeCount, 1);
    rowNode(range) = repelem((1:nodeCount).', 8);
    localRows(range, 1:4) = repmat([diag(1.0./scale); -diag(1.0./scale)], nodeCount, 1);
    limits = [upper./scale; -lower./scale];
    localBound(range) = limits(:);
    stateNode(range) = repelem((0:nodeCount-1).', 8);
    cursor = cursor+numel(range);
    range = cursor+1:rowCount;
    matrix(range, 1:count) = frictionMatrix;
    bound(range) = -frictionOffset;
    rowFamily(range) = "friction";
    localRows(range, :) = reshape(permute(frictionRows, [1, 3, 2]), [], 8);
    localBound(range) = -frictionConstant(:);
    stateNode(range) = repelem((0:prediction.stageCount-1).', size(frictionRows, 1));
    inputStage(range) = stateNode(range)+1;

    % Exact rest in the three velocity states. The terminal policy cancels
    % the declared constant longitudinal bias; its steering is zero.
    terminalInput = [0.0; -model.longitudinalAccelerationBias ...
        / model.cfg.model.longitudinalInputGain];
    equalityMatrix = [prediction.egoStateMatrix(4:6, :, end), zeros(3, 1)];
    equalityBound = -prediction.egoStateOffset(4:6, end);
    lowerInput = [-cfg.model.frontWheelSteeringAngleMaximum; ...
        cfg.actuation.longitudinalAccelerationMinimum];
    upperInput = [cfg.model.frontWheelSteeringAngleMaximum; ...
        cfg.actuation.longitudinalAccelerationMaximum];
    lowerBound = [repmat(lowerInput, prediction.stageCount, 1); 0.0];
    upperBound = [repmat(upperInput, prediction.stageCount, 1); inf];
    lowerBound(count-1:count) = terminalInput;
    upperBound(count-1:count) = terminalInput;
    common = struct("certificate", localClfCertificate(model), ...
        "equilibrium", localCruiseEquilibriumProfile(model, prediction));
    [hessian, linear, constant] = localObjective(model, layout);
    qp = struct("problemClass", "certificatePreservingCbfClfQp", ...
        "layout", layout, "label", "maintainedCertificate", ...
        "Hessian", hessian, "linear", linear, "constant", constant, ...
        "inequalityMatrix", matrix, "inequalityBound", bound, ...
        "equalityMatrix", equalityMatrix, "equalityBound", equalityBound, ...
        "rowFamily", rowFamily, "rowNode", rowNode, ...
        "lowerBound", lowerBound, "upperBound", upperBound, ...
        "collision", families(1), "road", families(2:end), ...
        "clf", localClfData(prediction, model, layout, common), ...
        "geometry", geometry, "linearizationPlan", anchorPlan, ...
        "nominalState", nominalState, ...
        "terminalInput", terminalInput, "certifiedInfeasible", ...
            any(~isfinite(bound)) || any(lowerBound > upperBound) ...
            || any(terminalInput < lowerInput) || any(terminalInput > upperInput));
    qp.stageProgram = avoidanceStageQp(qp, prediction, ...
        localRows, localBound, stateNode, inputStage);
end

function equilibrium = localCruiseEquilibriumProfile(model, prediction)
    nodeCount = model.horizonSteps+1;
    equilibrium = struct("input", zeros(2, model.horizonSteps), ...
        "lateralVelocity", zeros(1, nodeCount), "yawRate", zeros(1, nodeCount));
    for nodeIdx = 1:nodeCount
        point = cruiseEquilibrium(model.referenceSpeed, ...
            prediction.scheduleCurvature(nodeIdx), ...
            model.longitudinalAccelerationBias, model.cfg);
        equilibrium.lateralVelocity(nodeIdx) = point.lateralVelocity;
        equilibrium.yawRate(nodeIdx) = point.yawRate;
        if nodeIdx <= model.horizonSteps
            equilibrium.input(:, nodeIdx) = ...
                [point.steeringAngle; point.longitudinalAcceleration];
        end
    end
end

function [hessian, linear, constant] = localObjective(model, layout)
% Penalize head input effort and squared CLF relaxation without an input target.
% The rest continuation certifies safety and carries no performance cost.
    cfg = model.cfg;
    scale = [cfg.model.frontWheelSteeringAngleMaximum; ...
        max(abs([cfg.actuation.longitudinalAccelerationMinimum, ...
            cfg.actuation.longitudinalAccelerationMaximum]))];
    weight = [cfg.clf.frontWheelSteeringAngleWeight; ...
        cfg.clf.longitudinalAccelerationWeight]./scale.^2;
    diagonal = zeros(layout.decisionCount, 1);
    diagonal(layout.inputIndex) = model.sampleTime ...
        * repmat(weight, layout.horizonSteps, 1);
    diagonal(layout.relaxationIndex) = cfg.clf.relaxationWeight;
    hessian = diag(2.0*diagonal);
    linear = zeros(layout.decisionCount, 1);
    constant = 0.0;
end

function clf = localClfData(prediction, model, layout, common)
% LfV + LgV*u_0 <= -alpha*V + delta at the current scheduled state.
% P and the local cruise reference are held fixed for this derivative.
% Future quadratic values are diagnostics, not optimization constraints.
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
    lyapunovMatrix = certificate.lyapunovMatrix;
    currentError = errorOffset(:, 1);
    initialValue = currentError.'*lyapunovMatrix*currentError;
    [continuousA, continuousB, continuousC] = ltvBicycleModel.continuousMatrices( ...
        prediction.scheduleCurvature(1), prediction.scheduleSpeedProfile(1), model.cfg);
    continuousC(4) = continuousC(4)+model.longitudinalAccelerationBias;
    drift = continuousA*prediction.egoStateOffset(:, 1)+continuousC;
    lyapunovGradient = 2.0*currentError.'*lyapunovMatrix;
    lieDerivativeDrift = lyapunovGradient*drift(2:6);
    lieDerivativeInput = lyapunovGradient*continuousB(2:6, :);
    decayRate = model.cfg.clf.decreaseRateFraction*certificate.certifiedDecreaseRate;
    inequalityMatrix = zeros(1, layout.decisionCount);
    inequalityMatrix(1:layout.inputDimension) = lieDerivativeInput;
    inequalityMatrix(layout.relaxationIndex) = -1.0;
    clf = struct( ...
        "certificate", certificate, ...
        "lyapunovMatrix", lyapunovMatrix, "decayRate", decayRate, ...
        "lieDerivativeDrift", lieDerivativeDrift, ...
        "lieDerivativeInput", lieDerivativeInput, ...
        "inequalityMatrix", inequalityMatrix, ...
        "inequalityBound", -lieDerivativeDrift-decayRate*initialValue, ...
        "errorMatrix", errorMatrix, "errorOffset", errorOffset, ...
        "equilibriumInput", equilibrium.input, ...
        "initialValue", initialValue);
end

function certificate = localClfCertificate(model)
% Continuous Riccati CLF certificate of the path-frame cruise error.
%
% The error state [d; ePsi; vx - vRef; vy; r] is the [d; ePsi; vx; vy;
% r] block of the continuous Frenet generator at the straight reference cruise
% (kappa = 0) - the station row does not feed back into it. The
% continuous Riccati solution P and gain K certify, for the
% unconstrained linearized error dynamics,
%
%   Vdot(e) = -e' (Q + K' R K) e <= -lambdaMin(W,P)*V(e).
%
% The configured fraction scales this certified rate in inverse seconds.
% Curvature, a different scheduled speed, and actuator constraints can
% require positive relaxation; this reference certificate is local.
    persistent memoKey memoCertificate
    cfg = model.cfg;
    minimumAcceleration = cfg.actuation.longitudinalAccelerationMinimum;
    maximumAcceleration = cfg.actuation.longitudinalAccelerationMaximum;
    accelerationScale = max( ...
        abs(minimumAcceleration), abs(maximumAcceleration));
    key = struct( ...
        "referenceSpeed", max(model.referenceSpeed, ...
            cfg.clf.certificateSpeedFloor), ...
        "longitudinalInputGain", cfg.model.longitudinalInputGain, ...
        "scheduleSpeedFloor", cfg.model.scheduleSpeedFloor, ...
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
    [continuousA, continuousB] = ltvBicycleModel.continuousMatrices( ...
        0.0, key.referenceSpeed, cfg);
    errorIndex = 2:6;
    errorStateMatrix = continuousA(errorIndex, errorIndex);
    errorInputMatrix = continuousB(errorIndex, :);
    stateWeight = diag(1.0./key.errorScale.^2);
    inputWeight = diag(key.inputWeight./key.inputScale.^2);
    try
        [feedbackGain, lyapunovMatrix] = lqr( ...
            errorStateMatrix, errorInputMatrix, stateWeight, inputWeight);
    catch riccatiException
        error("collisionAvoidanceController:invalidFormulation", ...
            "The CLF Riccati synthesis at the reference cruise " ...
            + "failed: %s", riccatiException.message);
    end
    decreaseMatrix = stateWeight+feedbackGain.'*inputWeight*feedbackGain;
    decreaseMatrix = 0.5*(decreaseMatrix+decreaseMatrix.');
    decreaseEigenvalue = min(real(eig(decreaseMatrix, lyapunovMatrix)));
    if ~isfinite(decreaseEigenvalue) || decreaseEigenvalue <= 0.0
        error("collisionAvoidanceController:invalidFormulation", ...
            "The continuous CLF certificate must have a positive finite decay rate.");
    end
    certificate = struct( ...
        "lyapunovMatrix", lyapunovMatrix, ...
        "feedbackGain", feedbackGain, ...
        "decreaseMatrix", decreaseMatrix, ...
        "certifiedDecreaseRate", decreaseEigenvalue, ...
        "timeDomain", "continuousTime", ...
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
% the bias and the centripetal cross term of vxdot = gamma*a + vy r + b.
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
    longitudinalAcceleration = (-accelerationBias-lateralVelocity*yawRate) ...
        / cfg.model.longitudinalInputGain;
    equilibrium = struct( ...
        "referenceSpeed", referenceSpeed, ...
        "curvature", curvature, ...
        "lateralVelocity", lateralVelocity, ...
        "yawRate", yawRate, ...
        "steeringAngle", steeringAngle, ...
        "longitudinalAcceleration", longitudinalAcceleration);
end
