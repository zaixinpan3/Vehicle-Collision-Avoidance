function [command, predictedInput, planningProblem, certificate] = ...
        collisionAvoidanceController(egoState, targetEstimate, laneCenterline, cfg, controllerState)
%collisionAvoidanceController Encounter-scoped predictive CBF and sampled CLF.
% The fourth output is the accepted continuation. Supply it as the fifth
% input at the next sample. Targets require explicit finite motion/exit
% contracts; missing observations retain their active obligations. Controls
% are held for cfg.controller.sampleTime. Finite-sensing observations renew
% the nominal lookahead; issued controls require a fresh verified solve.
% Every issued command requires a verified solution from the current call.
% If no candidate supplies one, report failure without applying stored inputs.
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
    identity = struct("configuration", rmfield(cfg, "solver"), "lane", lane, ...
        "accelerationBias", ego.longitudinalAccelerationBias);
    incumbent = [];
    encounters = struct("key", {}, "contract", {}, "center", {}, "radius", {}, ...
        "time", {}, "halfLength", {}, "halfWidth", {}, "discharged", {}, "exitMargin", {},"nominalCenter",{});
    if ~isempty(controllerState)
        if ~isstruct(controllerState) || ~isscalar(controllerState) ...
                || ~isfield(controllerState, "version") || controllerState.version ~= 10
            error("collisionAvoidanceController:invalidStoredCertificate", ...
                "Admission requires empty state or a version-10 encounter certificate.");
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
        [radius, consistent] = stateUncertainty.intersect(model.initialEgoState,model.initialFrenetErrorBound, ...
            predicted,controllerState.prediction.egoStateErrorBound(:,2));
        if ~consistent
            error("collisionAvoidanceController:inconsistentObservation", ...
                "The ego observation is inconsistent with the executed certified tube.");
        end
        model.initialFrenetErrorBound = radius;
        model.previousInput = controllerState.appliedInput;
        model.previousManeuver = controllerState.maneuver;
        model.requiredMargin = (1-cfg.encounter.barrierFraction)*controllerState.margin;
        if isempty(controllerState.encounters) || all(arrayfun(@targetPrediction.isFiniteSensing,controllerState.encounters))
            incumbent = struct("plan",controllerState.plan(:,2:end),"maneuver",controllerState.maneuver, ...
                "prediction",struct("stageCount",controllerState.remainingSteps-1));
        else
            incumbent = localTail(controllerState);
        end
        encounters = controllerState.encounters;
        for index = 1:numel(encounters)
            match = find(string({observations.key}) == encounters(index).key, 1);
            observation = [];
            if ~isempty(match), observation = observations(match); end
            if targetPrediction.isFiniteSensing(encounters(index))
                if isempty(observation) && ego.completePerception
                    [reachable,reachableRadius] = targetPrediction.finiteFlow(encounters(index),model.sampleTime);
                    maximumRange = norm(reachable(1:2)-ego.position) ...
                        +norm(reachableRadius(1:2))+norm(ego.stateErrorBound(1:2));
                    if ~encounters(index).discharged && maximumRange<ego.perceptionRange
                        error("collisionAvoidanceController:inconsistentPerception", ...
                            "Complete perception cannot omit a target whose entire reachable set remains in range.");
                    end
                    encounters(index).discharged = true;
                    encounters(index).time = model.stateTime;
                    continue;
                elseif ~isempty(observation) && encounters(index).discharged
                    encounters(index) = targetPrediction.admit(observation,model.stateTime,lane,cfg);
                    continue;
                end
            end
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
    for index = 1:numel(observations)
        match = find(string({encounters.key}) == observations(index).key, 1);
        if isempty(match)
            next = targetPrediction.admit(observations(index), model.stateTime, lane, cfg);
            next.exitMargin = targetPrediction.exitMargin(next, 0, cfg);
            next.discharged = next.exitMargin >= cfg.encounter.numericalMargin;
            encounters = [encounters; next]; %#ok<AGROW>
        end
    end
    if isempty(encounters)
        encounters = struct("key", {}, "contract", {}, "center", {}, "radius", {}, ...
            "time", {}, "halfLength", {}, "halfWidth", {}, "discharged", {}, "exitMargin", {},"nominalCenter",{});
    end
    model.encounters = encounters;
    active = any(~[encounters.discharged]);
    continuingEncounter = false;
    if ~isempty(controllerState)
        priorKeys = string({controllerState.encounters(~[controllerState.encounters.discharged]).key});
        strict = ~reshape(arrayfun(@targetPrediction.isFiniteSensing,encounters),1,[]);
        continuingEncounter = any(ismember(string({encounters(strict & ~[encounters.discharged]).key}),priorKeys));
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
    if ~isempty(controllerState)
        model.linearizationStates = controllerState.predictedState(:,2:end);
        model.linearizationInputs = controllerState.plan(:,2:end);
        if isempty(model.linearizationInputs), model.linearizationInputs = model.previousInput; end
    end
    prediction = ltvBicycleModel.finitePredict(model, schedule);
    [model,prediction] = localPlanningWindow(model,prediction);
    [model.exitSteps,model.exitMargin] = localExitSchedule(model);
    predictionSeconds = toc(timer)-preparationSeconds;
    maneuvers = "track";
    if active, maneuvers = ["yield", "passLeft", "passRight"]; end
    retainManeuver = string(cfg.encounter.maneuverSelectionPolicy)=="retainFeasibleManeuver";
    if active && retainManeuver
        preferred = model.previousManeuver;
        if ~any(maneuvers==preferred)
            nearest = encounters(find(~[encounters.discharged],1));
            projection = laneGeometry.project(nearest.center(1:2),lane);
            preferred = "passRight";
            if projection.lateralPosition<model.initialEgoState(2), preferred = "passLeft"; end
        end
        maneuvers = [preferred,maneuvers(maneuvers~=preferred)];
    end
    best = [];
    solverCalls = 0;
    geometryRebuildCount = 0;
    nominalRefinementCount = 0;
    failures = strings(0, 1);
    formulationSeconds = 0;
    solveSeconds = 0;
    verificationSeconds = 0;
    for maneuver = maneuvers
        candidateModel = model;
        candidateModel.maneuver = maneuver;
        candidatePrediction = prediction;
        anchor = localAnchor(candidateModel, candidatePrediction, incumbent);
        if string(cfg.model.linearizationPolicy)~="cruise" && maneuver~=model.previousManeuver
            candidateModel.linearizationInputs = reshape(anchor,2,[]);
            candidatePrediction = ltvBicycleModel.finitePredict(candidateModel,[]);
        end
        rebuild = 0;refinement = 0;
        try
            while true
                phase = tic;
                [candidateModel,candidatePrediction,anchor] = ...
                    localPlanningWindow(candidateModel,candidatePrediction,anchor);
                [candidateModel.exitSteps,candidateModel.exitMargin] = localExitSchedule(candidateModel);
                qp = formulateAvoidanceProblem(candidateModel, candidatePrediction, anchor);
                formulationSeconds = formulationSeconds+toc(phase);
                phase = tic;
                [result,qp] = solveHardCbfClf(qp, cfg);
                solveSeconds = solveSeconds+toc(phase);
                phase = tic;
                solverCalls = solverCalls+result.solverCalls;
                check = certifyAvoidancePlan(qp, candidatePrediction, candidateModel, result.decision);
                if result.feasible && check.accepted
                    [nominal,nominalViolation] = localNominalLookahead(qp,candidateModel,result.decision);
                    if nominalViolation>0
                        if refinement>=cfg.encounter.maximumNominalRefinements || any(~isfinite(nominal),"all")
                            failures(end+1,1) = maneuver+": nonlinear nominal lookahead remains infeasible"; %#ok<AGROW>
                            break;
                        end
                        candidateModel.linearizationStates = nominal(:,1:end-1);
                        candidateModel.linearizationInputs = reshape(result.decision(qp.layout.planIndex),2,[]);
                        candidateModel.refineNominalLookahead = true;
                        candidatePrediction = ltvBicycleModel.finitePredict(candidateModel,[]);
                        anchor = result.decision(qp.layout.planIndex);
                        refinement = refinement+1;nominalRefinementCount = nominalRefinementCount+1;
                        verificationSeconds = verificationSeconds+toc(phase);
                        continue;
                    end
                    if isempty(best) || result.objectiveValue < best.result.objectiveValue
                        best = struct("qp",qp,"result",result,"check",check,"maneuver",maneuver, ...
                            "model",candidateModel,"prediction",candidatePrediction, ...
                            "nominalStates",nominal,"nominalViolation",nominalViolation);
                    end
                elseif ~result.feasible
                    failures(end+1, 1) = maneuver+": "+result.message; %#ok<AGROW>
                elseif ~check.accepted
                    failures(end+1, 1) = maneuver+": "+strjoin(check.failedConditions, ","); %#ok<AGROW>
                end
                verificationSeconds = verificationSeconds+toc(phase);
                if ~isempty(best) && best.maneuver==maneuver, break; end
                if result.exitFlag~=-2 || ~any(maneuver==["passLeft","passRight"]) ...
                        || rebuild==cfg.encounter.maximumGeometryRebuilds
                    break;
                end
                % A failed separating-plane inner approximation does not
                % prove physical infeasibility. Backtrack its steering seed
                % toward cruise and rebuild the same hard constraints.
                inputs = reshape(anchor,2,[]);
                reference = reshape(candidatePrediction.referencePlan,2,[]);
                if rebuild==0
                    inputs = reshape(localAnchor(candidateModel,candidatePrediction,[]),2,[]);
                else
                    inputs(1,:) = (inputs(1,:)+reference(1,:))/2;
                end
                anchor = inputs(:);
                if string(cfg.model.linearizationPolicy)~="cruise"
                    candidateModel.linearizationInputs = inputs;
                    candidatePrediction = ltvBicycleModel.finitePredict(candidateModel,[]);
                end
                geometryRebuildCount = geometryRebuildCount+1;
                rebuild = rebuild+1;
            end
            if retainManeuver && ~isempty(best), break; end
        catch exception
            if any(string(exception.identifier) == ["collisionAvoidanceController:roadBoundaryCoverageGap", ...
                    "collisionAvoidanceController:unsupportedReferenceJump"])
                failures(end+1, 1) = maneuver+": "+string(exception.message); %#ok<AGROW>
            else
                rethrow(exception);
            end
        end
    end
    phase = tic;
    if isempty(best)
        error("collisionAvoidanceController:noCertifiedContinuation", ...
            "Control failed: no verified solution at the current sample. %s.", ...
            strjoin(failures, "; "));
    end
    model = best.model;prediction = best.prediction;
    qp = best.qp;
    decision = best.result.decision;
    check = best.check;
    maneuver = best.maneuver;
    source = "checkedOptimization";
    margin = check.margin;
    predictedInput = reshape(decision(qp.layout.planIndex), 2, []);
    command = localCommand(predictedInput, model, prediction);
    predictedState = reshape(pagemtimes(prediction.egoStateMatrix, predictedInput(:)), 6, [])+prediction.egoStateOffset;
    certificate = struct("version", 10, "identity", identity, "stateTime", model.stateTime, ...
        "deadline", model.stateTime+prediction.stageCount*model.sampleTime, ...
        "remainingSteps", prediction.stageCount, "margin", margin, "maneuver", maneuver, ...
        "plan", predictedInput, "decision", decision, "qp", qp, "prediction", prediction, ...
        "predictedState", predictedState, "appliedInput", command.actuatorInput, ...
        "stateErrorBound", prediction.egoStateErrorBound, ...
        "encounters", encounters, "acceptance", check, "safetyScope", "executedIntervalWithNominalLookahead", ...
        "certifiedDuration",min(cfg.controller.certifiedSteps,prediction.stageCount)*model.sampleTime);
    if cfg.controller.certifiedSteps>=prediction.stageCount
        certificate.safetyScope = "uncertainFiniteHorizon";
    end
    metadata = struct("planCertified", check.accepted, "certificateSource", source, ...
        "fallbackUsed", false, "solverCallCount", solverCalls, ...
        "geometryRebuildCount",geometryRebuildCount, "nominalRefinementCount",nominalRefinementCount, ...
        "nominalLookaheadChecked",~isempty(best.nominalStates), ...
        "nominalLookaheadViolation",best.nominalViolation, ...
        "anticipationReserveFraction",qp.reserveFraction,"maneuver", maneuver, ...
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
    metadata.setMembershipUpdate = ~isempty(controllerState);
    metadata.certificateCompatible = ~isempty(incumbent);
    metadata.carriedWitnessFeasible = false;
    metadata.safetyScope = certificate.safetyScope;
    metadata.certifiedDuration = certificate.certifiedDuration;
    metadata.lookaheadDuration = prediction.stageCount*model.sampleTime;
    if cfg.controller.certifiedSteps<prediction.stageCount
        metadata.collisionDiscretization = "sweptExecutedIntervalsAndNominalChordLookahead";
    end
    metadata.recursiveFeasibilityClaimed = false;
    metadata.exactPredictionAssumptionsHold = false;
    metadata.tireForceConstraintScope = "nominalScheduledForceWithSeparatePlantResidual";
    metadata.executedContinuousGenerator = [prediction.continuousA(:,:,1), ...
        prediction.continuousB(:,:,1),prediction.continuousC(:,1)];
    metadata.executedResidualRateBound = prediction.modelErrorRateBound(:,1);
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
        if targetPrediction.isFiniteSensing(encounter)
            if model.stateTime+times(end)>encounter.contract.validUntil+128*eps(max(1,abs(encounter.contract.validUntil)))
                error("collisionAvoidanceController:expiredEncounterContract","The requested prediction exceeds available motion validity.");
            end
            steps(index) = model.horizonSteps;
            continue;
        end
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
    if ~isempty(incumbent) && model.maneuver == incumbent.maneuver
        inputs = reshape(prediction.referencePlan,2,[]);
        retained = min(size(inputs,2),size(incumbent.plan,2));
        inputs(:,1:retained) = incumbent.plan(:,1:retained);
        anchor = inputs(:);
        return;
    end
    inputs = reshape(prediction.referencePlan, 2, []);
    if model.maneuver == "passLeft" || model.maneuver == "passRight"
        direction = 1;
        if model.maneuver == "passRight", direction = -1; end
        count = prediction.stageCount;
        shape = sin(2*pi*(0:count-1)/max(1,count));
        perturbation = zeros(2,count);
        perturbation(1,:) = shape;
        nominal = reshape(pagemtimes(prediction.egoStateMatrix,inputs(:)),6,[])+prediction.egoStateOffset;
        response = reshape(pagemtimes(prediction.egoStateMatrix,perturbation(:)),6,[]);
        amplitude = 0.02;
        % Seed the selected side beyond the footprint at closest approach.
        % This only chooses separating planes; it adds no tracking objective.
        for encounter = model.encounters(:).'
            if encounter.discharged, continue; end
            distances = inf(1,count+1);
            lateral = zeros(1,count+1);
            for node = 1:count+1
                duration = (node-1)*model.sampleTime;
                if targetPrediction.isFiniteSensing(encounter)
                    center = targetPrediction.nominalFlow(encounter,duration);
                else
                    center = targetPrediction.finiteFlow(encounter,duration);
                end
                targetFrame = laneGeometry.project(center(1:2),model.lane);
                distances(node) = abs(nominal(1,node)-targetFrame.station);
                lateral(node) = targetFrame.lateralPosition;
            end
            [~,node] = min(distances);
            targetState = targetPrediction.nominalFlow(encounter,(node-1)*model.sampleTime);
            routeHeading = laneGeometry.project(targetState(1:2),model.lane).heading;
            lateralNormal = [-sin(routeHeading);cos(routeHeading)];
            clearance = model.cfg.vehicle.width/2 ...
                +targetPrediction.rectangleSupport(encounter.halfLength,encounter.halfWidth, ...
                    lateralNormal,targetState(7),0)+model.cfg.collision.clearanceMargin;
            required = clearance+direction*(lateral(node)-nominal(2,node));
            if abs(response(2,node))>sqrt(eps)
                amplitude = max(amplitude,required/abs(response(2,node)));
            end
        end
        amplitude = min(amplitude,model.cfg.model.frontWheelSteeringAngleMaximum);
        inputs(1,:) = inputs(1,:)+direction*amplitude*shape;
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
    rows = [rows;true(numel(qp.physicalBound)-numel(rows),1)];
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

function [model, prediction, anchor] = localPlanningWindow(model, prediction, anchor)
%localPlanningWindow Restrict prediction to complete sensed road cells.
% The maximum horizon is a requested lookahead, not permission to extrapolate
% a perceived boundary. No failed optimization or stored control is used here.
    cfg = model.cfg;
    if nargin<3,anchor = prediction.referencePlan;end
    count = prediction.stageCount;
    for index = 1:numel(prediction.cells)
        cell = prediction.cells(index);
        frame = laneGeometry.sweptCellFrame(model,cell,anchor);
        covered = true;
        for boundary = model.road.boundaries(:).'
            direction = boundary.longitudinalDirection;
            stations = direction.'*frame.tangent*[frame.stationLower,frame.stationUpper];
            extent = abs(direction.'*frame.lateral)*cfg.model.lateralDomainRadius ...
                +hypot(cfg.vehicle.length/2,cfg.vehicle.width/2) ...
                +abs(direction).'*frame.positionErrorBound;
            range = [min(stations)-extent,max(stations)+extent] ...
                +direction.'*(frame.origin-boundary.origin);
            covered = covered && range(1)>=boundary.parameterRange(1) ...
                && range(2)<=boundary.parameterRange(2);
        end
        if ~covered
            count = cell.stage-1;
            break;
        end
    end
    if count < 1
        error("collisionAvoidanceController:roadBoundaryCoverageGap", ...
            "Sensed road geometry cannot certify even the next held interval.");
    end
    if count < prediction.stageCount
        model.horizonSteps = count;
        prediction = ltvBicycleModel.finitePredict(model,[]);
        anchor = anchor(1:prediction.planCount);
        [model,prediction,anchor] = localPlanningWindow(model,prediction,anchor);
    end
end

function [states,violation] = localNominalLookahead(qp,model,decision)
    states = [];violation = 0;
    if model.cfg.controller.certifiedSteps>=model.horizonSteps ...
            || string(model.cfg.model.linearizationPolicy)=="cruise"
        return;
    end
    inputs = reshape(decision(qp.layout.planIndex),2,[]);
    states = ltvBicycleModel.nominalRollout(model,inputs);
    cfg = model.cfg;
    slipLimit = cfg.model.slipAngleMaximum(:);
    if isscalar(slipLimit), slipLimit = repmat(slipLimit,2,1); end
    for geometry = qp.geometry.local(:).'
        if geometry.stage<=model.cfg.controller.certifiedSteps,continue;end
        value = reshape(pagemtimes(geometry.nodeStateRows, ...
            reshape(states(:,geometry.stage:geometry.stage+1),6,1,2)),[],2) ...
            +reshape(pagemtimes(geometry.nodeInputRows,inputs(:,geometry.stage)),[],2)-geometry.nodeLimits;
        % Evaluate physical tire constraints on the nonlinear trajectory.
        % Reusing a force tangent at a different state can reject feasible
        % nonlinear forces and create a cycle of unnecessary refinements.
        for point = 1:2
            state = states(:,geometry.stage+point-1);
            input = inputs(:,geometry.stage);
            slip = atan2(state(5)+[cfg.vehicle.lf;-cfg.vehicle.lr]*state(6), ...
                max(state(4),cfg.model.scheduleSpeedFloor))-[input(1);0];
            value(geometry.nodeLabels=="tireSlip",point) = ...
                [slip;-slip]-[slipLimit;slipLimit]+cfg.encounter.numericalMargin;
        end
        violation = max(violation,max(value,[],"all"));
    end
end
