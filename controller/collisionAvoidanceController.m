function [command, predictedInput, planningProblem, certificate] = ...
        collisionAvoidanceController(egoState, targetEstimate, laneCenterline, cfg, controllerState)
%collisionAvoidanceController Encounter-scoped predictive CBF and sampled CLF.
% The fourth output is the accepted continuation. Supply it as the fifth
% input at the next sample. Targets require explicit finite motion/exit
% contracts; missing observations retain their active obligations. Controls
% are held for cfg.controller.sampleTime. No target forecast is appended.
    persistent previousCertificate
    if nargin == 1 && (ischar(egoState) || isstring(egoState))
        if ~isscalar(string(egoState)) || string(egoState) ~= "resetNominalTrajectory"
            error("collisionAvoidanceController:invalidAction", "Use resetNominalTrajectory.");
        end
        previousCertificate = [];
        command = []; predictedInput = []; planningProblem = []; certificate = [];
        return;
    end
    explicitState = nargin >= 5;
    if ~explicitState, controllerState = previousCertificate; end
    if nargin < 4, cfg = []; end
    if nargin < 3, laneCenterline = []; end
    if nargin < 2, targetEstimate = []; end
    timer = tic;
    cfg = localControllerConfiguration(cfg);
    [ego, lane, road, observations] = readPlanningInputs(egoState, targetEstimate, laneCenterline, cfg);
    model = localFiniteModel(ego, lane, road, cfg);
    identity = struct("configuration", rmfield(cfg, "solver"), "lane", lane, "road", road, ...
        "accelerationBias", ego.longitudinalAccelerationBias);
    incumbent = [];
    encounters = struct("key", {}, "contract", {}, "center", {}, "radius", {}, ...
        "time", {}, "halfLength", {}, "halfWidth", {}, "discharged", {}, "exitMargin", {});
    if ~isempty(controllerState)
        if ~isstruct(controllerState) || ~isscalar(controllerState) ...
                || ~isfield(controllerState, "version") || controllerState.version ~= 9
            error("collisionAvoidanceController:invalidStoredCertificate", ...
                "Admission requires empty state or a version-9 encounter certificate.");
        end
        if ~isequaln(identity, controllerState.identity)
            error("collisionAvoidanceController:changedExecutionContract", ...
                "Changed model, route or CLF data cannot inherit the stored continuation.");
        end
        if ~isfinite(model.stateTime)
            if any(~[controllerState.encounters.discharged])
                error("collisionAvoidanceController:missingStateTime", "Active encounters require a timestamp.");
            end
            model.stateTime = controllerState.stateTime+model.sampleTime;
        end
        tolerance = 128*eps(max(1, abs(model.stateTime)));
        if abs(model.stateTime-controllerState.stateTime-model.sampleTime) > tolerance ...
                || (~isempty(ego.heldActuatorInput) && any(abs(ego.heldActuatorInput-controllerState.appliedInput) ...
                > cfg.controller.shiftConsistencyTolerance))
            error("collisionAvoidanceController:executionContractViolation", ...
                "The certificate requires the scheduled sample and the previously issued held input.");
        end
        priorCheck = certifyAvoidancePlan(controllerState.qp, controllerState.prediction, model, controllerState.decision);
        if ~priorCheck.accepted || ~isequal(controllerState.plan(:), controllerState.decision(controllerState.qp.layout.planIndex)) ...
                || ~isequal(controllerState.appliedInput, controllerState.plan(:, 1))
            error("collisionAvoidanceController:invalidStoredCertificate", "The stored witness failed verification.");
        end
        predicted = reshape(pagemtimes(controllerState.prediction.egoStateMatrix(:, :, 2), controllerState.plan(:)), 6, 1) ...
            +controllerState.prediction.egoStateOffset(:, 2);
        [radius, consistent] = stateUncertainty.intersect(predicted, ...
            controllerState.prediction.egoStateErrorBound(:, 2), model.initialEgoState, model.initialFrenetErrorBound);
        if ~consistent
            error("collisionAvoidanceController:inconsistentObservation", ...
                "The ego observation is inconsistent with the executed certified tube.");
        end
        model.initialEgoState = predicted;
        model.initialFrenetErrorBound = radius;
        model.previousInput = controllerState.appliedInput;
        model.previousManeuver = controllerState.maneuver;
        model.requiredMargin = (1-cfg.encounter.barrierFraction)*controllerState.margin;
        incumbent = localTail(controllerState);
        encounters = controllerState.encounters;
        for index = 1:numel(encounters)
            match = find(string({observations.key}) == encounters(index).key, 1);
            observation = [];
            if ~isempty(match), observation = observations(match); end
            if encounters(index).discharged
                if ~isempty(observation)
                    localCheckDischargedObservation(encounters(index), observation, cfg);
                end
                encounters(index).time = model.stateTime;
            else
                encounters(index) = targetPrediction.advance(encounters(index), model.sampleTime, observation, lane, cfg);
            end
        end
    else
        if ~isfinite(model.stateTime)
            if ~isempty(observations)
                error("collisionAvoidanceController:missingStateTime", "Active encounters require a timestamp.");
            end
            model.stateTime = 0;
        end
    end
    newAdmission = false;
    for index = 1:numel(observations)
        match = find(string({encounters.key}) == observations(index).key, 1);
        if isempty(match)
            next = targetPrediction.admit(observations(index), model.stateTime, lane, cfg);
            next.exitMargin = targetPrediction.exitMargin(next, 0, cfg);
            next.discharged = next.exitMargin >= cfg.encounter.numericalMargin;
            encounters = [encounters; next]; %#ok<AGROW>
            newAdmission = true;
        end
    end
    if isempty(encounters)
        encounters = struct("key", {}, "contract", {}, "center", {}, "radius", {}, ...
            "time", {}, "halfLength", {}, "halfWidth", {}, "discharged", {}, "exitMargin", {});
    end
    model.encounters = encounters;
    active = any(~[encounters.discharged]);
    continuingEncounter = false;
    if ~isempty(controllerState)
        priorKeys = string({controllerState.encounters(~[controllerState.encounters.discharged]).key});
        activeKeys = string({encounters(~[encounters.discharged]).key});
        continuingEncounter = any(ismember(activeKeys, priorKeys));
    end
    if ~continuingEncounter
        model.requiredMargin = 0;
        if ~isempty(incumbent) && incumbent.prediction.stageCount == 0
            incumbent = [];
        end
    elseif ~isempty(incumbent)
        model.horizonSteps = incumbent.prediction.stageCount;
    end
    if model.horizonSteps == 0
        error("collisionAvoidanceController:uncertifiedEndpoint", "The retained deadline has no certified exit.");
    end
    [model.exitSteps, model.exitMargin] = localExitSchedule(model);
    preparationSeconds = toc(timer);
    schedule = [];
    if continuingEncounter && ~isempty(incumbent), schedule = incumbent.prediction.scheduleForStore; end
    prediction = ltvBicycleModel.finitePredict(model, schedule);
    predictionSeconds = toc(timer)-preparationSeconds;
    maneuvers = "track";
    if active, maneuvers = ["yield", "passLeft", "passRight"]; end
    best = [];
    solverCalls = 0;
    failures = strings(0, 1);
    formulationSeconds = 0;
    solveSeconds = 0;
    verificationSeconds = 0;
    for maneuver = maneuvers
        candidateModel = model;
        candidateModel.maneuver = maneuver;
        anchor = localAnchor(candidateModel, prediction, incumbent);
        try
            phase = tic;
            qp = formulateAvoidanceProblem(candidateModel, prediction, anchor);
            formulationSeconds = formulationSeconds+toc(phase);
            phase = tic;
            result = solveHardCbfClf(qp, cfg);
            solveSeconds = solveSeconds+toc(phase);
            phase = tic;
            solverCalls = solverCalls+result.solverCalls;
            check = certifyAvoidancePlan(qp, prediction, candidateModel, result.decision);
            if result.feasible && check.accepted && (isempty(best) || result.objectiveValue < best.result.objectiveValue)
                best = struct("qp", qp, "result", result, "check", check, "maneuver", maneuver);
            elseif ~check.accepted
                failures(end+1, 1) = maneuver+": "+strjoin(check.failedConditions, ","); %#ok<AGROW>
            end
            verificationSeconds = verificationSeconds+toc(phase);
        catch exception
            if any(string(exception.identifier) == ["collisionAvoidanceController:roadBoundaryCoverageGap", ...
                    "collisionAvoidanceController:unsupportedReferenceJump"])
                failures(end+1, 1) = maneuver+": "+string(exception.message); %#ok<AGROW>
            else
                rethrow(exception);
            end
        end
    end
    fallback = isempty(best);
    phase = tic;
    if fallback
        if isempty(incumbent) || newAdmission
            error("collisionAvoidanceController:noCertifiedContinuation", ...
                "No jointly certified continuation: %s.", strjoin(failures, "; "));
        end
        prediction = incumbent.prediction;
        qp = incumbent.qp;
        decision = incumbent.decision;
        check = certifyAvoidancePlan(qp, prediction, model, decision);
        if ~check.accepted
            error("collisionAvoidanceController:invalidStoredCertificate", "The truncated witness failed verification.");
        end
        maneuver = incumbent.maneuver;
        source = "conditionedStoredContinuation";
        margin = controllerState.margin;
    else
        qp = best.qp;
        decision = best.result.decision;
        check = best.check;
        maneuver = best.maneuver;
        source = "checkedOptimization";
        margin = check.margin;
    end
    predictedInput = reshape(decision(qp.layout.planIndex), 2, []);
    command = localCommand(predictedInput, model, prediction);
    predictedState = reshape(pagemtimes(prediction.egoStateMatrix, predictedInput(:)), 6, [])+prediction.egoStateOffset;
    certificate = struct("version", 9, "identity", identity, "stateTime", model.stateTime, ...
        "deadline", model.stateTime+prediction.stageCount*model.sampleTime, ...
        "remainingSteps", prediction.stageCount, "margin", margin, "maneuver", maneuver, ...
        "plan", predictedInput, "decision", decision, "qp", qp, "prediction", prediction, ...
        "predictedState", predictedState, "appliedInput", command.actuatorInput, ...
        "stateErrorBound", prediction.egoStateErrorBound, ...
        "encounters", encounters, "acceptance", check, "safetyScope", "heldIntervalsUntilCertifiedEncounterExit");
    metadata = struct("planCertified", check.accepted, "certificateSource", source, ...
        "fallbackUsed", fallback, "solverCallCount", solverCalls, "maneuver", maneuver, ...
        "maneuverCandidates", maneuvers, "carriedMargin", margin, "requiredMargin", model.requiredMargin, ...
        "horizonSteps", prediction.stageCount, "tailSteps", 0, "deadline", certificate.deadline, ...
        "activeTargetKeys", string({encounters(~[encounters.discharged]).key}), ...
        "dischargedTargetKeys", string({encounters([encounters.discharged]).key}), ...
        "clfRelaxation", decision(qp.layout.relaxationIndex), "clfDecayRate", qp.clf.decayRate, ...
        "collisionDiscretization", "sweptBernsteinCells", "acceptance", check, ...
        "hasTarget", active, "postSolveCertificationPerformed", true, "runtimeSeconds", toc(timer));
    metadata.runtime = struct("inputPreparationSeconds", preparationSeconds, ...
        "predictionSeconds", predictionSeconds, "formulationAndWitnessSeconds", formulationSeconds, ...
        "solveSeconds", solveSeconds, "acceptanceAndCommitSeconds", verificationSeconds+toc(phase), ...
        "diagnosticsSeconds", 0);
    metadata.solverAlgorithm = "Clarabel predictive CBF-CLF SOCP";
    if fallback, metadata.solverAlgorithm = "stored certified continuation"; end
    metadata.setMembershipUpdate = ~isempty(controllerState);
    metadata.certificateCompatible = ~isempty(incumbent);
    metadata.carriedWitnessFeasible = ~isempty(incumbent) && ~newAdmission;
    metadata.exactPredictionAssumptionsHold = false;
    trackingError = predictedState(2:6, :)-qp.clf.referenceStart ...
        -qp.clf.referenceRate*((0:prediction.stageCount)*model.sampleTime);
    metadata.clfValueProfile = sum(trackingError.*(qp.clf.lyapunovMatrix*trackingError), 1);
    metadata.clfInitialValue = metadata.clfValueProfile(1);
    metadata.jointObjectiveValue = 0.5*decision.'*qp.Hessian*decision+qp.linear.'*decision+qp.constant;
    metadata.clfRelaxationCost = model.sampleTime*cfg.clf.relaxationWeight*sum(metadata.clfRelaxation.^2);
    metadata.hardRowViolation = check.hardRowViolation;
    planningProblem = struct("problemClass", qp.problemClass, "qp", qp, "layout", qp.layout, ...
        "prediction", prediction, "model", model, "decision", decision, "plan", predictedInput(:), ...
        "inputPlan", predictedInput, "tailPlan", zeros(2, 0), "metadata", metadata);
    if ~explicitState, previousCertificate = certificate; end
end

function model = localFiniteModel(ego, lane, road, cfg)
    projection = laneGeometry.project(ego.position, lane);
    heading = atan2(sin(ego.yaw-projection.heading), cos(ego.yaw-projection.heading));
    [radius, chartValid] = stateUncertainty.toFrenet(ego.modelState, ego.stateErrorBound, lane);
    if any(ego.stateErrorBound) && (~chartValid || ~isfinite(ego.stateTime))
        error("collisionAvoidanceController:invalidUncertaintyChart", ...
            "Uncertain admission requires a timestamp and one invertible projection chart.");
    end
    previousInput = zeros(2, 1);
    if ~isempty(ego.heldActuatorInput), previousInput = ego.heldActuatorInput; end
    model = struct("encounterMode", true, "cfg", cfg, "lane", lane, "road", road, ...
        "stateTime", ego.stateTime, "sampleTime", cfg.controller.sampleTime, ...
        "horizonSteps", cfg.controller.horizonSteps, "referenceSpeed", cfg.referenceSpeed, ...
        "initialEgoState", [projection.station; projection.lateralPosition; heading; ego.modelState(4:6)], ...
        "initialFrenetErrorBound", radius, "longitudinalAccelerationBias", ego.longitudinalAccelerationBias, ...
        "previousInput", previousInput, "previousManeuver", "track", "requiredMargin", 0);
end

function [steps, margin] = localExitSchedule(model)
    steps = zeros(numel(model.encounters), 1);
    margin = inf;
    times = (1:model.horizonSteps)*model.sampleTime;
    for index = 1:numel(model.encounters)
        encounter = model.encounters(index);
        if encounter.discharged, continue; end
        margins = targetPrediction.exitMargin(encounter, times, model.cfg);
        covered = model.stateTime+times <= encounter.contract.validUntil+128*eps(max(1, abs(encounter.contract.validUntil)));
        step = find(covered & margins >= model.requiredMargin+model.cfg.encounter.numericalMargin, 1);
        if isempty(step)
            error("collisionAvoidanceController:noCertifiedExit", ...
                "Target %s has no certified exit within the retained deadline and contract validity.", encounter.key);
        end
        steps(index) = step;
        margin = min(margin, margins(step));
    end
end

function anchor = localAnchor(model, prediction, incumbent)
    if ~isempty(incumbent) && model.maneuver == incumbent.maneuver ...
            && numel(incumbent.plan) == prediction.planCount
        anchor = incumbent.plan(:);
        return;
    end
    inputs = reshape(prediction.referencePlan, 2, []);
    if model.maneuver == "passLeft" || model.maneuver == "passRight"
        direction = 1;
        if model.maneuver == "passRight", direction = -1; end
        count = prediction.stageCount;
        inputs(1, :) = inputs(1, :)+direction*0.02*sin(2*pi*(0:count-1)/max(1, count));
    elseif model.maneuver == "yield"
        inputs(2, :) = max(model.cfg.actuation.brakingRatioMinimum, inputs(2, :)-0.15);
    end
    anchor = inputs(:);
end

function localCheckDischargedObservation(encounter, observation, cfg)
    if ~isempty(observation.encounterContract) && ~isequaln(encounter.contract, observation.encounterContract)
        error("collisionAvoidanceController:changedEncounterContract", ...
            "A later encounter must use a new stable track identity and a new admission contract.");
    end
    current = encounter;
    current.center = [observation.position; observation.velocity; observation.acceleration; observation.yaw; observation.yawRate];
    current.radius = [observation.positionErrorBound; observation.velocityErrorBound; observation.accelerationErrorBound; ...
        observation.yawErrorBound; observation.yawRateErrorBound];
    current.halfLength = observation.length/2;
    current.halfWidth = observation.width/2;
    if targetPrediction.exitMargin(current, 0, cfg) < 0
        error("collisionAvoidanceController:exitRouteViolation", ...
            "The observation no longer certifies the discharged target's nonreturn route.");
    end
end

function tail = localTail(stored)
% Truncate the verified representation algebraically. Nothing is appended.
    tail = stored;
    prediction = stored.prediction;
    count = prediction.stageCount;
    oldPlanCount = 2*count;
    keep = [3:oldPlanCount, oldPlanCount+2:3*count];
    drop = [1, 2, oldPlanCount+1];
    fixed = stored.decision(drop);
    input = stored.plan(:, 1);
    prediction.stageCount = count-1;
    prediction.nodeCount = count;
    prediction.planCount = 2*(count-1);
    prediction.egoStateOffset = prediction.egoStateOffset(:, 2:end) ...
        +reshape(pagemtimes(prediction.egoStateMatrix(:, 1:2, 2:end), input), 6, []);
    prediction.egoStateMatrix = prediction.egoStateMatrix(:, 3:end, 2:end);
    prediction.egoStateErrorBound = prediction.egoStateErrorBound(:, 2:end);
    prediction.continuousA = prediction.continuousA(:, :, 2:end);
    prediction.continuousB = prediction.continuousB(:, :, 2:end);
    prediction.continuousC = prediction.continuousC(:, 2:end);
    prediction.stageMatrixA = prediction.stageMatrixA(:, :, 2:end);
    prediction.stageMatrixB = prediction.stageMatrixB(:, :, 2:end);
    prediction.stageAffine = prediction.stageAffine(:, 2:end);
    prediction.referencePlan = prediction.referencePlan(3:end);
    for field = ["speedProfile", "station", "curvature", "brakingRatio"]
        prediction.scheduleForStore.(field) = prediction.scheduleForStore.(field)(2:end);
    end
    prediction.scheduleSpeedProfile = prediction.scheduleSpeedProfile(2:end);
    prediction.scheduleCurvature = prediction.scheduleCurvature(2:end);
    prediction.scheduleBrakingRatio = prediction.scheduleBrakingRatio(2:end);
    cells = prediction.cells([prediction.cells.stage] > 1);
    keptCells = [prediction.cells.stage] > 1;
    for index = 1:numel(cells)
        cells(index).offset = cells(index).offset+reshape(pagemtimes(cells(index).map(:, 1:2, :), input), 6, []);
        cells(index).map = cells(index).map(:, 3:end, :);
        cells(index).endOffset = cells(index).endOffset+cells(index).endMap(:, 1:2)*input;
        cells(index).endMap = cells(index).endMap(:, 3:end);
        cells(index).stage = cells(index).stage-1;
        cells(index).start = cells(index).start-stored.identity.configuration.controller.sampleTime;
        cells(index).time = cells(index).time-stored.identity.configuration.controller.sampleTime;
    end
    prediction.cells = cells;
    qp = stored.qp;
    geometryRows = qp.geometry.stage > 1;
    inputRows = [false(2, 1); true(oldPlanCount-2, 1)];
    rows = [geometryRows; inputRows; inputRows; false; true(count-1, 1)];
    qp.physicalBound = qp.physicalBound(rows)-qp.inequalityMatrix(rows, drop)*fixed;
    qp.inequalityBound = qp.inequalityBound(rows)-qp.inequalityMatrix(rows, drop)*fixed;
    qp.inequalityMatrix = qp.inequalityMatrix(rows, keep);
    qp.safetyRows = qp.safetyRows(rows);
    qp.geometry.physicalBound = qp.geometry.physicalBound(geometryRows)-qp.geometry.matrix(geometryRows, 1:2)*input;
    qp.geometry.matrix = qp.geometry.matrix(geometryRows, 3:end);
    qp.geometry.stage = qp.geometry.stage(geometryRows)-1;
    qp.geometry.safety = qp.geometry.safety(geometryRows);
    qp.geometry.label = qp.geometry.label(geometryRows);
    qp.geometry.frames = qp.geometry.frames(keptCells);
    qp.geometry.normals = qp.geometry.normals(keptCells);
    constraints = qp.clf.constraints([qp.clf.constraints.stage] > 1);
    for index = 1:numel(constraints)
        constraints(index).offset = constraints(index).offset+constraints(index).map(:, drop)*fixed;
        constraints(index).map = constraints(index).map(:, keep);
        constraints(index).stage = constraints(index).stage-1;
    end
    qp.clf.constraints = constraints;
    qp.lowerBound = qp.lowerBound(keep);
    qp.upperBound = qp.upperBound(keep);
    qp.layout.planCount = 2*(count-1);
    qp.layout.horizonSteps = count-1;
    qp.layout.decisionCount = 3*(count-1);
    qp.layout.relaxationCount = count-1;
    qp.layout.planIndex = 1:2*(count-1);
    qp.layout.inputIndex = qp.layout.planIndex;
    qp.layout.relaxationIndex = 2*(count-1)+1:3*(count-1);
    qp.requiredMargin = stored.margin;
    qp.constant = qp.constant+qp.linear(drop).'*fixed+0.5*fixed.'*qp.Hessian(drop, drop)*fixed;
    qp.linear = qp.linear(keep)+qp.Hessian(keep, drop)*fixed;
    qp.Hessian = qp.Hessian(keep, keep);
    qp.equalityMatrix = zeros(0, numel(keep));
    qp.stageProgram = avoidanceStageQp(qp);
    qp.clf.referenceStart = qp.clf.referenceStart ...
        +qp.clf.referenceRate*stored.identity.configuration.controller.sampleTime;
    newState = stored.predictedState(2:6, 2)-qp.clf.referenceStart;
    qp.clf.initialValue = newState.'*qp.clf.lyapunovMatrix*newState;
    tail.plan = stored.plan(:, 2:end);
    tail.decision = stored.decision(keep);
    tail.qp = qp;
    tail.prediction = prediction;
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
