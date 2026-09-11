classdef hardEncounterBarrier
    %hardEncounterBarrier Preserve a complete hard-safe encounter witness.
    % Certificates retain the original affine inclusion, rows, absolute target
    % envelopes and numerical reserves. Only the executed prefix is fixed.
    % See HARD_PREDICTIVE_CBF.md for assumptions and the augmented-state CBF.

    methods (Static)
        function validateAdmission(ego, observations)
            if ~isfinite(ego.stateTime) || ~isfinite(ego.perceptionRange) ...
                    || ~ego.completePerception
                error("collisionAvoidanceController:invalidBarrierAdmission", ...
                    "Admission needs timestamped complete circular perception and a declared sensor radius.");
            end
            if any(arrayfun(@(target) isempty(target.predictionMotion), observations))
                error("collisionAvoidanceController:invalidBarrierAdmission", ...
                    "Retained perception exit requires finite-sensing motion bounds for every target.");
            end
        end

        function [matrix, bound] = completionRows(model, prediction, geometry)
        % A supporting halfspace certifies center-based circular sensor exit.
            frame = geometry.frames(end);
            positionMap = [frame.tangent, frame.lateral];
            stateMap = positionMap*prediction.egoStateMatrix(1:2, :, end);
            offset = frame.origin+positionMap*prediction.egoStateOffset(1:2, end);
            radius = abs(positionMap)*prediction.egoStateErrorBound(1:2, end) ...
                + frame.positionErrorBound;
            anchor = offset+stateMap*model.anchorPlan;
            count = numel(model.encounters);
            matrix = zeros(count, prediction.planCount);
            bound = zeros(count, 1);
            duration = prediction.stageCount*model.sampleTime;
            for index = 1:count
                encounter = model.encounters(index);
                [center, targetRadius] = targetPrediction.finiteFlow(encounter, duration);
                displacement = anchor-center(1:2);
                if norm(displacement) <= sqrt(eps)
                    error("collisionAvoidanceController:noCertifiedExit", ...
                        "The terminal anchor supplies no perception-exit direction.");
                end
                normal = displacement/norm(displacement);
                matrix(index, :) = -normal.'*stateMap;
                bound(index) = normal.'*(offset-center(1:2)) ...
                    -abs(normal).'*(radius+targetRadius(1:2)) ...
                    -model.perceptionRange-model.cfg.encounter.perceptionExitBuffer;
            end
        end

        function [stored, problem] = admit(stored, problem)
            stored.version = 16;
            stored.admissionTime = stored.stateTime;
            stored.consumedSteps = 0;
            stored.targetAdmissionSteps = zeros(numel(stored.encounters), 1);
            stored.encounterComplete = false;
            stored.witnessModel = problem.model;
            stored.safetyScope = "completeEncounterForDeclaredInclusion";
            stored.certifiedDuration = stored.remainingSteps*problem.model.sampleTime;
            problem.metadata = localMetadata(problem.metadata, stored, "checkedOptimization", ...
                problem.metadata.solverCallCount, false);
            stored.metadata = problem.metadata;
        end

        function [command, inputs, problem, stored] = advance(stored, ego, model, observations, identity, makeCommand, readmit)
            timer = tic;
            localValidateStored(stored, identity);
            h = model.sampleTime;
            consumed = stored.consumedSteps+1;
            count = stored.prediction.stageCount;
            expectedTime = stored.admissionTime+consumed*h;
            if abs(model.stateTime-expectedTime) > 128*eps(max(1, abs(expectedTime))) ...
                    || ~isfinite(model.stateTime) || ~isequal(ego.heldActuatorInput, stored.appliedInput)
                error("collisionAvoidanceController:executionContractViolation", ...
                    "Continuation requires the scheduled sample and the exact previously issued input.");
            end
            rootPlan = reshape(stored.decision(stored.qp.layout.planIndex), 2, []);
            if ~isequal(stored.plan, rootPlan(:, stored.consumedSteps+1:end)) ...
                    || ~isequal(stored.appliedInput, rootPlan(:, stored.consumedSteps+1))
                error("collisionAvoidanceController:invalidStoredCertificate", "The retained input witness changed.");
            end
            check = certifyAvoidancePlan(stored.qp, stored.prediction, model, stored.decision);
            if ~check.accepted || ~isequal(check.margin, stored.margin)
                error("collisionAvoidanceController:invalidStoredCertificate", "The retained certificate failed verification.");
            end
            predicted = stored.prediction.egoStateMatrix(:, :, consumed+1)*rootPlan(:) ...
                +stored.prediction.egoStateOffset(:, consumed+1);
            radius = stored.prediction.egoStateErrorBound(:, consumed+1);
            if any(model.initialEgoState-model.initialFrenetErrorBound > predicted+radius) ...
                    || any(model.initialEgoState+model.initialFrenetErrorBound < predicted-radius)
                error("collisionAvoidanceController:inconsistentObservation", ...
                    "The ego measurement contradicts the retained execution enclosure.");
            end
            exited = localCheckTargets(stored, ego, model, observations, consumed);
            newTargets = observations(~ismember(string({observations.key}), string({stored.encounters.key})));
            stored.consumedSteps = consumed;
            stored.stateTime = expectedTime;
            stored.remainingSteps = count-consumed;
            stored.encounterComplete = exited;
            if (isempty(stored.encounters) && (~isempty(newTargets) || consumed==count)) ...
                    || (exited && ~isempty(newTargets))
                validationSeconds=toc(timer);
                [command,inputs,problem,stored]=readmit();
                problem.metadata.runtime.priorCertificateValidationSeconds=validationSeconds;
                problem.metadata.runtimeSeconds=toc(timer);
                problem.metadata.jointAdmissionPerformed=~isempty(newTargets);
                problem.metadata.newlyAdmittedTargetKeys=string({newTargets.key});
                stored.metadata=problem.metadata;
                return;
            end
            if consumed==count && ~exited
                error("collisionAvoidanceController:unverifiedEncounterCompletion", ...
                    "Exhausting a witness is not evidence that its targets left perception.");
            end
            source = "retainedCertifiedWitness";
            calls = 0;
            attempted = false;
            jointAdmission = ~isempty(newTargets);
            jointCheck = struct("incumbentAccepted", false, "usedIncumbent", false);
            if jointAdmission
                if consumed == count
                    error("collisionAvoidanceController:jointAdmissionNotCertified", ...
                        "A new target cannot enter an exhausted retained deadline.");
                end
                hardEncounterBarrier.validateAdmission(ego, newTargets);
                [stored, check, calls, jointCheck] = localAdmitNewTargets(stored, model, newTargets, rootPlan);
                stored.encounterComplete = false;
                source = "checkedJointAdmission";
            elseif ~stored.encounterComplete
                % The initial matrices stay intact. No new normals, tubes,
                % terminal times, or numerical allowances replace the witness.
                qp = stored.qp;
                qp.requiredMargin = stored.margin;
                qp.inequalityBound = qp.barrier.baseBound-stored.margin*qp.barrier.scale;
                qp.stageProgram = updateAvoidanceStageBounds(qp);
                qp.stageProgram.fixedDecisionIndex = (1:2*consumed).';
                qp.stageProgram.fixedDecisionValue = rootPlan(:, 1:consumed);
                qp.stageProgram.fixedDecisionValue = qp.stageProgram.fixedDecisionValue(:);
                attempted = true;
                [result, candidateQp] = solveHardCbfClf(qp, model.cfg);
                calls = result.solverCalls;
                candidate = certifyAvoidancePlan(candidateQp, stored.prediction, model, result.decision);
                if result.feasible && candidate.accepted && candidate.margin >= stored.margin
                    stored.decision = result.decision;
                    qp = candidateQp;
                    check = candidate;
                    source = "checkedContinuationOptimization";
                end
                stored.qp = qp;
            else
                source = "verifiedPerceptionExit";
            end
            stored.margin = check.margin;
            stored.acceptance = check;
            rootPlan = reshape(stored.decision(stored.qp.layout.planIndex), 2, []);
            states = reshape(pagemtimes(stored.prediction.egoStateMatrix, rootPlan(:)), 6, []) ...
                +stored.prediction.egoStateOffset;
            stored.predictedState = states(:, consumed+1:end);
            stored.stateErrorBound = stored.prediction.egoStateErrorBound(:, consumed+1:end);
            stored.plan = rootPlan(:, consumed+1:end);
            stored.certifiedDuration = stored.remainingSteps*h;
            command = [];
            inputs = zeros(2, 0);
            stored.appliedInput = [];
            stored.scheduledInput = [];
            if ~stored.encounterComplete
                inputs = stored.plan;
                command = makeCommand(rootPlan, stored.witnessModel, stored.prediction, consumed+1);
                command.measurementTime = expectedTime;
                command.actuationTime = expectedTime;
                command.holdSeconds = h;
                stored.appliedInput = inputs(:, 1);
                stored.scheduledInput = inputs(:, 1);
            end
            metadata = localMetadata(stored.metadata, stored, source, calls, attempted);
            metadata.jointAdmissionPerformed = jointAdmission;
            metadata.newlyAdmittedTargetKeys = string({newTargets.key});
            if jointAdmission
                metadata.carriedWitnessFeasible = jointCheck.incumbentAccepted;
                metadata.certificateCompatible = false;
                metadata.fallbackUsed = jointCheck.usedIncumbent;
            end
            metadata.runtimeSeconds = toc(timer);
            metadata.runtime = struct("continuationSeconds", metadata.runtimeSeconds);
            metadata.requiredMargin = stored.qp.requiredMargin;
            if ~stored.encounterComplete
                metadata.executedContinuousGenerator = [stored.prediction.continuousA(:, :, consumed+1), ...
                    stored.prediction.continuousB(:, :, consumed+1), stored.prediction.continuousC(:, consumed+1)];
                metadata.executedResidualRateBound = stored.prediction.modelErrorRateBound(:, consumed+1);
            end
            stored.metadata = metadata;
            % qp/layout/decision refer to the immutable admission coordinates;
            % inputPlan and predictedState expose the unexecuted continuation.
            problem = struct("problemClass", stored.qp.problemClass, "qp", stored.qp, ...
                "layout", stored.qp.layout, "prediction", stored.prediction, "model", model, ...
                "decision", stored.decision, "plan", rootPlan(:), "inputPlan", inputs, ...
                "tailPlan", zeros(2, 0), "metadata", metadata);
        end
    end
end

function localValidateStored(stored, identity)
    required = ["version", "admissionTime", "consumedSteps", "encounterComplete", ...
        "witnessModel", "qp", "decision", "prediction", "metadata", "identity", "targetAdmissionSteps"];
    if ~isstruct(stored) || ~isscalar(stored) || ~all(isfield(stored, required)) ...
            || stored.version ~= 16 || stored.encounterComplete
        error("collisionAvoidanceController:invalidStoredCertificate", ...
            "Continuation requires an active version-16 hard encounter certificate.");
    end
    if ~isequaln(identity, stored.identity)
        error("collisionAvoidanceController:changedExecutionContract", ...
            "The retained witness requires unchanged model, road, route and perception range.");
    end
    validateattributes(stored.consumedSteps, {'double'}, ...
        {'scalar', 'integer', 'nonnegative', '<', stored.prediction.stageCount});
    validateattributes(stored.targetAdmissionSteps, {'double'}, ...
        {'integer', 'nonnegative', '<=', stored.consumedSteps, 'numel', numel(stored.encounters)});
    if stored.deadline ~= stored.admissionTime+stored.prediction.stageCount*stored.witnessModel.sampleTime
        error("collisionAvoidanceController:invalidStoredCertificate", "The retained deadline changed.");
    end
end

function exited = localCheckTargets(stored, ego, model, observations, consumed)
% The admission envelope is evaluated at absolute time; it is never renewed.
    exited = ~isempty(stored.encounters);
    if ~exited,return;end
    [egoLower,egoUpper]=localExecutedPositionIntersection(stored,ego,consumed);
    for index = 1:numel(stored.encounters)
        encounter = stored.encounters(index);
        targetElapsed = (consumed-stored.targetAdmissionSteps(index))*model.sampleTime;
        [center, radius] = targetPrediction.finiteFlow(encounter, targetElapsed);
        match = find(string({observations.key}) == encounter.key, 1);
        if ~isempty(match)
            measured = targetPrediction.admit(observations(match), model.stateTime, model.lane, model.cfg);
            if ~targetPrediction.isFiniteSensing(measured) ...
                    || ~isequal(measured.contract.jerkBound, encounter.contract.jerkBound) ...
                    || measured.contract.yawAccelerationBound ~= encounter.contract.yawAccelerationBound ...
                    || measured.halfLength ~= encounter.halfLength || measured.halfWidth ~= encounter.halfWidth
                error("collisionAvoidanceController:changedEncounterContract", "Target motion or extent changed.");
            end
            measured.center(7) = center(7)+atan2(sin(measured.center(7)-center(7)), cos(measured.center(7)-center(7)));
            if any(measured.center-measured.radius > center+radius) ...
                    || any(measured.center+measured.radius < center-radius)
                error("collisionAvoidanceController:inconsistentObservation", ...
                    "The target measurement contradicts the retained absolute-time envelope.");
            end
        end
        distance = norm(center(1:2)-ego.position);
        positionRadius = norm(radius(1:2))+norm(ego.stateErrorBound(1:2));
        if isempty(match) && ego.completePerception && distance+positionRadius < ego.perceptionRange
            error("collisionAvoidanceController:inconsistentPerception", ...
                "Complete perception omitted a target whose retained enclosure remains in range.");
        end
        separation=norm(max([egoLower-center(1:2)-radius(1:2), ...
            center(1:2)-radius(1:2)-egoUpper,zeros(2,1)],[],2));
        exited = exited && separation >= ego.perceptionRange+model.cfg.encounter.perceptionExitBuffer;
    end
end

function [lower,upper]=localExecutedPositionIntersection(stored,ego,consumed)
% Both valid enclosures contain the actual executed state. Intersect them so
% a coarse current measurement cannot erase the already proved terminal exit.
    prediction=stored.prediction;
    cellIndex=find([prediction.cells.stage]==consumed,1,'last');
    frame=stored.qp.geometry.frames(cellIndex);
    state=prediction.egoStateMatrix(:,:,consumed+1)*stored.decision(stored.qp.layout.planIndex) ...
        +prediction.egoStateOffset(:,consumed+1);
    positionMap=[frame.tangent,frame.lateral];
    center=frame.origin+positionMap*state(1:2);
    radius=abs(positionMap)*prediction.egoStateErrorBound(1:2,consumed+1)+frame.positionErrorBound;
    lower=max(center-radius,ego.position-ego.stateErrorBound(1:2));
    upper=min(center+radius,ego.position+ego.stateErrorBound(1:2));
    if any(lower>upper)
        error("collisionAvoidanceController:inconsistentObservation", ...
            "The current position measurement contradicts the retained world-position enclosure.");
    end
end

function [stored, check, calls, jointCheck] = localAdmitNewTargets(stored, model, observations, rootPlan)
% Joint admission changes the information state, never the old obligations.
% New target rows start at detection; old rows and the absolute deadline stay.
    cfg = model.cfg;
    prediction = stored.prediction;
    remaining = [prediction.cells.stage] > stored.consumedSteps;
    prediction.cells = prediction.cells(remaining);
    elapsed = stored.consumedSteps*model.sampleTime;
    for index = 1:numel(prediction.cells)
        prediction.cells(index).start = prediction.cells(index).start-elapsed;
        prediction.cells(index).time = prediction.cells(index).time-elapsed;
    end
    prediction.geometryAnchor = rootPlan(:);
    prediction.geometryFrames = stored.qp.geometry.frames(remaining);
    prediction.geometryNominal = cell(numel(prediction.cells), 1);
    for index = 1:numel(prediction.cells)
        tube = prediction.cells(index);
        prediction.geometryNominal{index} = reshape(pagemtimes(tube.map, rootPlan(:)), 6, [])+tube.offset;
    end
    jointModel = stored.witnessModel;
    jointModel.stateTime = stored.stateTime;
    jointModel.anchorPlan = rootPlan(:);
    jointModel.encounters = repmat(targetPrediction.admit(observations(1), ...
        stored.stateTime, model.lane, cfg), numel(observations), 1);
    for index = 1:numel(observations)
        jointModel.encounters(index) = targetPrediction.admit(observations(index), ...
            stored.stateTime, model.lane, cfg);
    end
    jointModel.exitSteps = repmat(stored.prediction.stageCount, numel(observations), 1);
    geometry = avoidanceSafetyGeometry(jointModel, prediction);
    collisionRows = startsWith(geometry.label, "collision:");
    % Only completionRows uses stageCount here; input maps retain admission
    % coordinates so the previously executed input prefix cannot change.
    prediction.stageCount = stored.remainingSteps;
    [exitMatrix, exitBound] = hardEncounterBarrier.completionRows(jointModel, prediction, geometry);
    matrix = [geometry.matrix(collisionRows, :); exitMatrix];
    bound = [geometry.physicalBound(collisionRows); exitBound];
    reserve = 2*cfg.encounter.numericalMargin*double(any(matrix ~= 0, 2));
    qp = stored.qp;
    oldCount = numel(qp.physicalBound);
    qp.inequalityMatrix = [qp.inequalityMatrix; matrix, zeros(numel(bound), qp.layout.relaxationCount)];
    qp.physicalBound = [qp.physicalBound; bound];
    qp.safetyRows = [qp.safetyRows; true(numel(bound), 1)];
    qp.barrier.baseBound = [qp.barrier.baseBound; bound-reserve];
    qp.barrier.scale = [qp.barrier.scale; ones(numel(bound), 1)];
    qp.barrier.completionRows = [qp.barrier.completionRows; ...
        oldCount+nnz(collisionRows)+(1:numel(exitBound)).'];
    qp.requiredMargin = 0;
    qp.inequalityBound = qp.barrier.baseBound;
    qp.certifiedInfeasible = any(qp.inequalityBound(~any(qp.inequalityMatrix, 2)) < 0);
    qp.stageProgram = avoidanceStageQp(qp);
    qp.stageProgram.fixedDecisionIndex = (1:2*stored.consumedSteps).';
    qp.stageProgram.fixedDecisionValue = rootPlan(:, 1:stored.consumedSteps);
    qp.stageProgram.fixedDecisionValue = qp.stageProgram.fixedDecisionValue(:);
    incumbent = certifyAvoidancePlan(qp, stored.prediction, model, stored.decision);
    jointCheck = struct("incumbentAccepted", incumbent.accepted, "usedIncumbent", false);
    [result, candidate] = solveHardCbfClf(qp, cfg);
    check = certifyAvoidancePlan(candidate, stored.prediction, model, result.decision);
    if result.feasible && check.accepted
        stored.qp = candidate;
        stored.decision = result.decision;
    elseif incumbent.accepted
        % The old controls are admissible only after checking every new row.
        stored.qp = qp;
        check = incumbent;
        jointCheck.usedIncumbent = true;
    else
        error("collisionAvoidanceController:jointAdmissionNotCertified", ...
            "No joint witness was certified for all retained and new targets before the unchanged deadline.");
    end
    stored.encounters = [stored.encounters(:); jointModel.encounters(:)];
    stored.targetAdmissionSteps = [stored.targetAdmissionSteps(:); ...
        repmat(stored.consumedSteps, numel(observations), 1)];
    calls = result.solverCalls;
end

function metadata = localMetadata(metadata, stored, source, calls, attempted)
    metadata.certificateSource = source;
    metadata.solverCallCount = calls;
    metadata.fallbackUsed = attempted && source == "retainedCertifiedWitness";
    metadata.carriedWitnessFeasible = stored.consumedSteps > 0;
    metadata.certificateCompatible = stored.consumedSteps > 0;
    metadata.carriedMargin = stored.margin;
    metadata.barrierValue = -stored.margin;
    metadata.barrierInterpretation = "storedWitnessLowerBound";
    metadata.horizonSteps = stored.remainingSteps;
    metadata.planningWindowSteps = stored.identity.configuration.controller.horizonSteps;
    metadata.certificateExtensionSteps = max(0,stored.remainingSteps-metadata.planningWindowSteps);
    metadata.deadline = stored.deadline;
    metadata.consumedSteps = stored.consumedSteps;
    metadata.encounterComplete = stored.encounterComplete;
    metadata.safetyScope = stored.safetyScope;
    metadata.certifiedDuration = stored.certifiedDuration;
    metadata.lookaheadDuration = stored.certifiedDuration;
    metadata.recursiveFeasibilityClaimed = ~isempty(stored.encounters);
    metadata.recursiveFeasibilityScope = "conditionalOnDeclaredInclusionExecutionAndJointAdmission";
    metadata.exactPredictionAssumptionsHold = false;
    metadata.physicalVehicleGuaranteeEstablished = false;
    metadata.newTargetAdmissionAssumption = "jointStateInCertifiableDomainAtFirstDetection";
    metadata.jointAdmissionPerformed = false;
    metadata.newlyAdmittedTargetKeys = strings(1, 0);
    metadata.planCertified = ~stored.encounterComplete;
    metadata.acceptance = stored.acceptance;
    metadata.hardRowViolation = stored.acceptance.hardRowViolation;
    metadata.clfRelaxation = stored.decision(stored.qp.layout.relaxationIndex(stored.consumedSteps+1:end));
    metadata.jointObjectiveValue = 0.5*stored.decision.'*stored.qp.Hessian*stored.decision ...
        +stored.qp.linear.'*stored.decision+stored.qp.constant;
    metadata.commandActuationTime = stored.stateTime;
    metadata.activeTargetKeys = string({stored.encounters.key});
    if stored.encounterComplete
        metadata.activeTargetKeys = strings(1, 0);
        metadata.dischargedTargetKeys = string({stored.encounters.key});
    end
end
