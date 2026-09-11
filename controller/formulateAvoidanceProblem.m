function qp = formulateAvoidanceProblem(model, prediction, anchorPlan)
% One convex hard-safety SOCP with one robust dissipation slack per sample.
    cfg = model.cfg;
    count = prediction.stageCount;
    planCount = prediction.planCount;
    decisionCount = planCount+count;
    model.anchorPlan = anchorPlan;
    geometry = avoidanceSafetyGeometry.build(model, prediction);
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
    [exitMatrix, exitBound] = hardEncounterBarrier.completionRows(model, prediction, geometry);
    completionRows = numel(physicalBound)+(1:numel(exitBound)).';
    hardMatrix = [hardMatrix; exitMatrix, zeros(numel(exitBound), count)];
    physicalBound = [physicalBound; exitBound];
    safetyRows = [safetyRows; true(numel(exitBound), 1)];
    % Leave room for strict independent acceptance at an active constraint.
    % Acceptance charges one reserve; solving with two does not spend that
    % same allowance on both solver termination and certificate arithmetic.
    reserve = 2*cfg.encounter.numericalMargin*double(any(hardMatrix ~= 0, 2));
    reserve(size(geometry.matrix,1)+2*planCount+(1:count)) = 0;
    bound = physicalBound-model.requiredMargin*double(safetyRows)-reserve;
    hessian = zeros(decisionCount);
    linear = zeros(decisionCount, 1);
    constant = 0;
    certificate = localClfCertificate(model);
    if certificate.operatingState(4)==cfg.referenceSpeed
        equilibriumState = certificate.operatingState;
    else
        equilibriumState = ltvBicycleModel.cruiseEquilibrium( ...
            certificate.operatingCurvature,cfg,model.longitudinalAccelerationBias);
    end
    referenceStart = equilibriumState(2:6)+cfg.clf.referenceOffset ...
        +cfg.clf.referenceRate*(model.stateTime-cfg.clf.referenceEpoch);
    clockScale = double(any(cfg.clf.referenceRate));
    scales = [cfg.clf.lateralPositionErrorScale; cfg.clf.headingErrorScale; cfg.clf.speedErrorScale; ...
        cfg.clf.lateralVelocityErrorScale; cfg.clf.yawRateErrorScale];
    % Stack state costs once. The slack columns are identically zero in
    % every state map and need not enter the repeated dense products.
    stateMap = reshape(permute(prediction.egoStateMatrix(2:6,:,1:count),[1,3,2]),[],planCount);
    stateOffset = prediction.egoStateOffset(2:6,1:count)-referenceStart ...
        -cfg.clf.referenceRate*((0:count-1)*model.sampleTime);
    weightedMap = stateMap./repmat(scales,count,1);
    weightedOffset = stateOffset(:)./repmat(scales,count,1);
    hessian(1:planCount,1:planCount) = 2*model.sampleTime*(weightedMap.'*weightedMap);
    linear(1:planCount) = 2*model.sampleTime*weightedMap.'*weightedOffset;
    constant = constant+model.sampleTime*(weightedOffset.'*weightedOffset);
    inputWeight = repmat([cfg.clf.frontWheelSteeringAngleWeight; cfg.clf.brakingRatioWeight], count, 1);
    hessian(1:planCount, 1:planCount) = hessian(1:planCount, 1:planCount)+2*model.sampleTime*diag(inputWeight);
    % Center input effort at the same operating input used for Riccati
    % synthesis. Prediction seeds and scheduled equilibria are not targets.
    operatingInput = repmat(certificate.operatingInput,count,1);
    linear(1:planCount) = linear(1:planCount)-2*model.sampleTime*inputWeight.*operatingInput;
    constant = constant+model.sampleTime*sum(inputWeight.*operatingInput.^2);
    difference = eye(planCount)-diag(ones(planCount-2, 1), -2);
    prior = [model.previousInput; zeros(planCount-2, 1)];
    smoothWeight = cfg.encounter.inputRateWeight/model.sampleTime;
    hessian(1:planCount, 1:planCount) = hessian(1:planCount, 1:planCount)+2*smoothWeight*(difference.'*difference);
    linear(1:planCount) = linear(1:planCount)-2*smoothWeight*difference.'*prior;
    constant = constant+smoothWeight*(prior.'*prior);
    hessian(planCount+1:end, planCount+1:end) = 2*model.sampleTime*cfg.clf.relaxationWeight*eye(count);
    scale = norm(certificate.lyapunovMatrix, inf);
    certificate.lyapunovMatrix = certificate.lyapunovMatrix/scale;
    certificate.decreaseMatrix = certificate.decreaseMatrix/scale;
    p = certificate.lyapunovMatrix;
    rate = cfg.clf.decreaseRateFraction*certificate.certifiedDecreaseRate;
    residual = cfg.model.ltvModelErrorRateBound(:)+cfg.model.plantModelResidualRateBound(:);
    if isfield(prediction,"modelErrorRateBound")
        residual = max(prediction.modelErrorRateBound,[],2);
    end
    disturbance = residual(2:6);
    youngRate = 0.1;
    disturbanceCost = disturbance.'*abs(p)*disturbance/youngRate;
    certifiedCells = prediction.cells;
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
            stateErrorBound = [tube.radius(2:6, point); zeros(3, 1)];
            errorQuadratic = stateErrorBound.'*abs(positive)*stateErrorBound;
            expansion = 0;
            ratio = 0;
            if errorQuadratic > 0
                ratio = max(1e-6, min(0.2, sqrt(errorQuadratic/max(1e-12, anchor.'*positive*anchor))));
                expansion = (1+1/ratio)*errorQuadratic;
            end
            root = sqrt(1+ratio)*factor;
            additive = curvature*(anchor.'*anchor)+disturbanceCost+expansion+abs(affine).'*stateErrorBound;
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
    layout = struct("decisionCount", decisionCount, "planCount", planCount, ...
        "horizonSteps", count, "inputDimension", 2, "inputIndex", 1:planCount, ...
        "planIndex", 1:planCount, "relaxationIndex", planCount+1:decisionCount, ...
        "tailSteps", 0, "tailIndex", [], "relaxationCount", count);
    qp = struct("problemClass", "encounterPredictiveCbfClfSocp", ...
        "layout", layout, "geometry", geometry, "clf", clf, ...
        "Hessian", hessian, "linear", linear, "constant", constant, ...
        "inequalityMatrix", hardMatrix, "inequalityBound", bound, ...
        "physicalBound", physicalBound, "safetyRows", safetyRows, ...
        "requiredMargin", model.requiredMargin, "exitMargin", model.exitMargin, ...
        "equalityMatrix", zeros(0, decisionCount), "equalityBound", zeros(0, 1), ...
        "lowerBound", [lowerInput; zeros(count, 1)], "upperBound", [upperInput; inf(count, 1)], ...
        "certifiedInfeasible", any(bound(~any(hardMatrix, 2)) < 0));
    % One unit in each hard row's native units normalizes its margin.
    scale = ones(size(bound));
    scale(size(geometry.matrix, 1)+2*planCount+(1:count)) = 0;
    qp.barrier = struct("baseBound", bound, "scale", scale, ...
        "completionRows", completionRows);
    qp.stageProgram = avoidanceStageQp.build(qp,prediction,model);

end

function certificate = localClfCertificate(model)
% Continuous Riccati CLF certificate of the path-frame cruise error.
%
% The error is relative to the nonlinear cruise trim at the current road
% curvature. The station coordinate is cyclic at frozen curvature. The
% continuous Riccati solution P and gain K certify, for the
% unconstrained linearized error dynamics,
%
%   Vdot(e) = -e' (Q + K' R K) e <= -lambdaMin(W,P)*V(e).
%
% The configured fraction scales this certified rate in inverse seconds.
% The metric and trim remain frozen within each formulated horizon. Changes
% between frames are scheduled local certificates, not a common Lyapunov proof.
    persistent memoKey memoCertificate
    cfg = model.cfg;
    minimumBrakingRatio = cfg.actuation.brakingRatioMinimum;
    maximumBrakingRatio = cfg.actuation.brakingRatioMaximum;
    brakingRatioScale = max( ...
        abs(minimumBrakingRatio), abs(maximumBrakingRatio));
    key = struct( ...
        "curvature", laneGeometry.curvature(model.initialEgoState(1),model.lane), ...
        "accelerationBias", model.longitudinalAccelerationBias, ...
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
    operatingCfg = cfg;
    operatingCfg.referenceSpeed = key.referenceSpeed;
    [operatingState, operatingInput] = ltvBicycleModel.cruiseEquilibrium( ...
        key.curvature,operatingCfg,key.accelerationBias);
    if abs(operatingInput(2))>=1-sqrt(eps)
        error("collisionAvoidanceController:invalidCruiseOperatingPoint", ...
            "The certificate cruise trim is outside the differentiable tire domain.");
    end
    [continuousA, continuousB] = ltvBicycleModel.continuousMatrices( ...
        key.curvature,key.referenceSpeed,cfg,[],key.accelerationBias, ...
        struct("state",operatingState,"input",operatingInput));
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
        "operatingState", operatingState, ...
        "operatingInput", operatingInput, ...
        "operatingCurvature", key.curvature, "operatingAccelerationBias", key.accelerationBias, ...
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
