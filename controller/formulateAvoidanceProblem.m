function qp = formulateAvoidanceProblem(model, prediction, anchorPlan)
% Convex maneuver-specific SOCP with one robust dissipation slack per sample.
    cfg = model.cfg;
    count = prediction.stageCount;
    planCount = prediction.planCount;
    decisionCount = planCount+count;
    model.anchorPlan = anchorPlan;
    geometry = avoidanceSafetyGeometry(model, prediction);
    lowerInput = repmat([-cfg.model.frontWheelSteeringAngleMaximum; cfg.actuation.brakingRatioMinimum], count, 1);
    upperInput = repmat([cfg.model.frontWheelSteeringAngleMaximum; cfg.actuation.brakingRatioMaximum], count, 1);
    hardMatrix = [geometry.matrix, zeros(size(geometry.matrix, 1), count); ...
        eye(planCount), zeros(planCount, count); -eye(planCount), zeros(planCount, count); ...
        zeros(count, planCount), -eye(count)];
    physicalBound = [geometry.physicalBound; upperInput; -lowerInput; zeros(count, 1)];
    safetyRows = [geometry.safety; false(2*planCount+count, 1)];
    rateLimit = model.sampleTime*repmat([cfg.model.frontWheelSteeringRateMaximum; ...
        cfg.model.brakingRatioRateMaximum],count,1);
    rateMap = eye(planCount)-diag(ones(planCount-2,1),-2);
    ratePrior = [model.previousInput;zeros(planCount-2,1)];
    selected = isfinite(rateLimit);
    hardMatrix = [hardMatrix;rateMap(selected,:),zeros(nnz(selected),count); ...
        -rateMap(selected,:),zeros(nnz(selected),count)];
    physicalBound = [physicalBound;rateLimit(selected)+ratePrior(selected);rateLimit(selected)-ratePrior(selected)];
    safetyRows = [safetyRows;false(2*nnz(selected),1)];
    % Leave room for strict independent acceptance at an active constraint.
    % Acceptance charges one reserve; solving with two does not spend that
    % same allowance on both solver termination and certificate arithmetic.
    reserve = 2*cfg.encounter.numericalMargin*double(any(hardMatrix ~= 0, 2));
    reserve(size(geometry.matrix,1)+2*planCount+(1:count)) = 0;
    domainReserve = [geometry.domainReserve;zeros(numel(physicalBound)-numel(geometry.domainReserve),1)];
    % Future-domain and clearance reserves share the feasible allocation.
    % They prepare future execution without redefining physical limits.
    initialReserve = [geometry.initialReserve;zeros(numel(physicalBound)-numel(geometry.initialReserve),1)];
    bound = physicalBound-model.requiredMargin*double(safetyRows)-reserve-initialReserve;
    anticipationReserve = domainReserve ...
        +[geometry.anticipationReserve;zeros(numel(bound)-numel(geometry.anticipationReserve),1)];
    hessian = zeros(decisionCount);
    linear = zeros(decisionCount, 1);
    constant = cfg.encounter.maneuverSwitchWeight*double(model.maneuver ~= model.previousManeuver);
    equilibriumState = ltvBicycleModel.cruiseEquilibrium( ...
        laneGeometry.curvature(model.initialEgoState(1),model.lane),cfg,model.longitudinalAccelerationBias);
    referenceStart = equilibriumState(2:6)+cfg.clf.referenceOffset ...
        +cfg.clf.referenceRate*(model.stateTime-cfg.clf.referenceEpoch);
    clockScale = double(any(cfg.clf.referenceRate));
    scales = [cfg.clf.lateralPositionErrorScale; cfg.clf.headingErrorScale; cfg.clf.speedErrorScale; ...
        cfg.clf.lateralVelocityErrorScale; cfg.clf.yawRateErrorScale];
    weight = diag(1./scales.^2);
    for stage = 1:count
        map = [prediction.egoStateMatrix(2:6, :, stage), zeros(5, count)];
        offset = prediction.egoStateOffset(2:6, stage)-referenceStart ...
            -cfg.clf.referenceRate*(stage-1)*model.sampleTime;
        hessian = hessian+2*model.sampleTime*(map.'*weight*map);
        linear = linear+2*model.sampleTime*map.'*weight*offset;
        constant = constant+model.sampleTime*offset.'*weight*offset;
    end
    inputWeight = repmat([cfg.clf.frontWheelSteeringAngleWeight; cfg.clf.brakingRatioWeight], count, 1);
    hessian(1:planCount, 1:planCount) = hessian(1:planCount, 1:planCount)+2*model.sampleTime*diag(inputWeight);
    equilibrium = prediction.referencePlan;
    linear(1:planCount) = linear(1:planCount)-2*model.sampleTime*inputWeight.*equilibrium;
    constant = constant+model.sampleTime*sum(inputWeight.*equilibrium.^2);
    difference = eye(planCount)-diag(ones(planCount-2, 1), -2);
    prior = [model.previousInput; zeros(planCount-2, 1)];
    smoothWeight = cfg.encounter.inputRateWeight/model.sampleTime;
    hessian(1:planCount, 1:planCount) = hessian(1:planCount, 1:planCount)+2*smoothWeight*(difference.'*difference);
    linear(1:planCount) = linear(1:planCount)-2*smoothWeight*difference.'*prior;
    constant = constant+smoothWeight*(prior.'*prior);
    hessian(planCount+1:end, planCount+1:end) = 2*model.sampleTime*cfg.clf.relaxationWeight*eye(count);
    certificate = localClfCertificate(model);
    scale = norm(certificate.lyapunovMatrix, inf);
    certificate.lyapunovMatrix = certificate.lyapunovMatrix/scale;
    certificate.decreaseMatrix = certificate.decreaseMatrix/scale;
    p = certificate.lyapunovMatrix;
    rate = cfg.clf.decreaseRateFraction*certificate.certifiedDecreaseRate;
    residual = cfg.model.ltvModelErrorRateBound(:)+cfg.model.plantModelResidualRateBound(:);
    if isfield(prediction,"modelErrorRateBound")
        residual = max(prediction.modelErrorRateBound(:,1:min(count,cfg.controller.certifiedSteps)),[],2);
    end
    disturbance = residual(2:6);
    youngRate = 0.1;
    disturbanceCost = disturbance.'*abs(p)*disturbance/youngRate;
    certifiedCells = prediction.cells([prediction.cells.stage]<=cfg.controller.certifiedSteps);
    cellConstraints = cell(numel(certifiedCells), 1);
    for cellIndex = 1:numel(certifiedCells)
        tube = certifiedCells(cellIndex);
        stage = tube.stage;
        a = prediction.continuousA(2:6, 2:6, stage);
        b = prediction.continuousB(2:6, :, stage);
        c = prediction.continuousC(2:6, stage);
        quadratic = zeros(8);
        quadratic(1:5, 1:5) = a.'*p+p*a+(rate+youngRate)*p;
        quadratic(1:5, 6:7) = p*b;
        quadratic(6:7, 1:5) = b.'*p;
        quadratic(1:5, 8) = p*a*cfg.clf.referenceRate;
        quadratic(8, 1:5) = cfg.clf.referenceRate.'*a.'*p;
        baseLinear = [2*p*(a*referenceStart+c-cfg.clf.referenceRate); zeros(3, 1)];
        curvature = norm(quadratic, inf)+1;
        positive = quadratic+curvature*eye(8);
        factor = chol((positive+positive.')/2);
        points = size(tube.offset, 2);
        seedState = mean(reshape(pagemtimes(tube.map, anchorPlan), 6, [])+tube.offset, 2);
        seedTime = mean(tube.time);
        anchor = [seedState(2:6)-referenceStart-cfg.clf.referenceRate*seedTime; ...
            anchorPlan(2*stage-1:2*stage); clockScale*seedTime];
        % A single convex majorant must cover the entire cell. Independently
        % reanchored bounds at individual control points would not justify
        % the convex-hull argument for an indefinite CLF residual.
        affine = baseLinear-2*curvature*anchor;
        constraints = cell(points, 1);
        for point = 1:points
            map = zeros(8, decisionCount);
            map(1:5, 1:planCount) = tube.map(2:6, :, point);
            map(6:7, 2*stage-1:2*stage) = eye(2);
            offset = [tube.offset(2:6, point)-referenceStart-cfg.clf.referenceRate*tube.time(point); ...
                zeros(2, 1); clockScale*tube.time(point)];
            % A constant reference has no clock term in its residual. Set
            % that unused coordinate to zero to avoid artificial CLF slack.
            error = [tube.radius(2:6, point); zeros(3, 1)];
            errorQuadratic = error.'*abs(positive)*error;
            expansion = 0;
            ratio = 0;
            if errorQuadratic > 0
                ratio = max(1e-6, min(0.2, sqrt(errorQuadratic/max(1e-12, anchor.'*positive*anchor))));
                expansion = (1+1/ratio)*errorQuadratic;
            end
            root = sqrt(1+ratio)*factor;
            additive = curvature*(anchor.'*anchor)+disturbanceCost+expansion+abs(affine).'*error;
            constraints{point} = struct("map", map, "offset", offset, "root", root, ...
                "linear", affine, "constant", additive, "stage", stage,"cellIndex",cellIndex,"pointIndex",point);
        end
        cellConstraints{cellIndex} = vertcat(constraints{:});
    end
    constraints = vertcat(cellConstraints{:});
    initialError = model.initialEgoState(2:6)-referenceStart;
    clf = struct("certificate", certificate, "lyapunovMatrix", p, "decayRate", rate, ...
        "referenceStart", referenceStart, "referenceRate", cfg.clf.referenceRate, ...
        "initialValue", initialError.'*p*initialError, "constraints", constraints);
    drift = prediction.continuousA(:, :, 1)*model.initialEgoState+prediction.continuousC(:, 1);
    clf.lieDerivativeDrift = 2*initialError.'*p*(drift(2:6)-cfg.clf.referenceRate);
    clf.lieDerivativeInput = 2*initialError.'*p*prediction.continuousB(2:6, :, 1);
    clf.errorOffset = prediction.egoStateOffset(2:6, :)-referenceStart ...
        -cfg.clf.referenceRate*((0:count)*model.sampleTime);
    clf.equilibriumInput = reshape(prediction.referencePlan, 2, []);
    layout = struct("decisionCount", decisionCount, "planCount", planCount, ...
        "horizonSteps", count, "inputDimension", 2, "inputIndex", 1:planCount, ...
        "planIndex", 1:planCount, "relaxationIndex", planCount+1:decisionCount, ...
        "tailSteps", 0, "tailIndex", [], "relaxationCount", count);
    qp = struct("encounterMode", true, "problemClass", "encounterPredictiveCbfClfSocp", ...
        "layout", layout, "geometry", geometry, "clf", clf, ...
        "Hessian", hessian, "linear", linear, "constant", constant, ...
        "inequalityMatrix", hardMatrix, "inequalityBound", bound, ...
        "physicalBound", physicalBound, "safetyRows", safetyRows, ...
        "domainReserve",domainReserve,"initialReserve",initialReserve, ...
        "requiredMargin", model.requiredMargin, "exitMargin", model.exitMargin, ...
        "anticipationReserve",anticipationReserve, ...
        "reserveFraction",0, ...
        "reserveFractionMaximum",cfg.encounter.anticipationReserveFractionMaximum, ...
        "equalityMatrix", zeros(0, decisionCount), "equalityBound", zeros(0, 1), ...
        "lowerBound", [lowerInput; zeros(count, 1)], "upperBound", [upperInput; inf(count, 1)], ...
        "certifiedInfeasible", any(bound(~any(hardMatrix, 2)) < 0));
    qp.stageProgram = avoidanceStageQp(qp,prediction,model);
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
    minimumBrakingRatio = cfg.actuation.brakingRatioMinimum;
    maximumBrakingRatio = cfg.actuation.brakingRatioMaximum;
    brakingRatioScale = max( ...
        abs(minimumBrakingRatio), abs(maximumBrakingRatio));
    key = struct( ...
        "referenceSpeed", max(model.referenceSpeed, ...
            cfg.clf.certificateSpeedFloor), ...
        "brakingRatioAccelerationGain", modifiedFialaTire.accelerationGain(cfg), ...
        "scheduleSpeedFloor", cfg.model.scheduleSpeedFloor, ...
        "errorScale", [ ...
            cfg.clf.lateralPositionErrorScale; ...
            cfg.clf.headingErrorScale; ...
            cfg.clf.speedErrorScale; ...
            cfg.clf.lateralVelocityErrorScale; ...
            cfg.clf.yawRateErrorScale], ...
        "inputWeight", [ ...
            cfg.clf.frontWheelSteeringAngleWeight; ...
            cfg.clf.brakingRatioWeight], ...
        "inputScale", [ ...
            cfg.model.frontWheelSteeringAngleMaximum; ...
            brakingRatioScale], ...
        "vehicle", cfg.vehicle, ...
        "roadLoad", cfg.roadLoad, ...
        "tire", cfg.tire, "actuation", cfg.actuation);
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
