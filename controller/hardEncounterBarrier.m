classdef hardEncounterBarrier
    %hardEncounterBarrier Rolling safe MPC with a carried recursive-feasibility witness.
    % The plan accepted at one frame is carried, together with its own stage
    % generators, cell charts, separating normals and terminal rows, to the
    % next frame. There the shifted plan is verified on the conditioned
    % information set before any fresh optimization may replace it. The
    % analytic terminal law is the tail of that witness; it is commanded only
    % when no optimized stage remains in the carried plan.

    methods (Static)
        function [model,prediction,qp,result,check,timing] = plan(model)
            % Fresh search. Every seed is a convexification anchor only: the CLF
            % majorant is exact at its anchor and grows quadratically away from
            % it, so a verified plan from one seed is not the tie-break optimum
            % of another. The shifted and equilibrium seeds are therefore both
            % solved at each horizon and the verified plan with the smaller
            % (value, objective) pair is kept; the slowing seed is a rescue
            % tried only when neither verifies.
            cfg = model.cfg;
            timing = struct("predictionSeconds",0,"formulationSeconds",0, ...
                "solveSeconds",0,"verificationSeconds",0,"attempts",0);
            calls = 0;
            searchTimer = tic;
            shifted = isfield(model,"initializationPlan");
            seedCount = 2+double(shifted);
            lastOutcome = "no attempt";
            best = [];
            seedKind = 1;
            while true
                if timing.attempts>0 && isempty(best) && toc(searchTimer)>=cfg.solver.certificateSearchTimeLimit
                    error("collisionAvoidanceController:certificateSearchLimit", ...
                        "The search budget expired without a verified plan after %d attempts (last: %s); " ...
                        + "infeasibility is not established.",timing.attempts,lastOutcome);
                end
                timing.attempts = timing.attempts+1;
                trial = model;
                trial.exitSteps = zeros(numel(model.encounters),1);
                trial.exitMargin = inf;
                phase = tic;
                [schedule,anchor] = localPredictionSchedule(trial,seedKind);
                trial.linearizationInputs = reshape(anchor,2,[]);
                trialPrediction = ltvBicycleModel.finitePredict(trial,schedule);
                timing.predictionSeconds = timing.predictionSeconds+toc(phase);
                phase = tic;
                trialQp = formulateAvoidanceProblem(trial,trialPrediction,anchor);
                timing.formulationSeconds = timing.formulationSeconds+toc(phase);
                phase = tic;
                [trialResult,trialQp] = solveHardCbfClf.solve(trialQp,cfg);
                calls = calls+trialResult.solverCalls;
                timing.solveSeconds = timing.solveSeconds+toc(phase);
                phase = tic;
                trialCheck = solveHardCbfClf.certify(trialQp,trialPrediction,trial,trialResult.decision);
                timing.verificationSeconds = timing.verificationSeconds+toc(phase);
                if trialResult.feasible && trialCheck.accepted
                    objective = 0.5*trialResult.decision.'*trialQp.Hessian*trialResult.decision ...
                        +trialQp.linear.'*trialResult.decision+trialQp.constant;
                    key = [trialCheck.value,objective];
                    if isempty(best) || key(1)<best.key(1) || (key(1)==best.key(1) && key(2)<best.key(2))
                        best = struct("model",trial,"prediction",trialPrediction,"qp",trialQp, ...
                            "result",trialResult,"check",trialCheck,"key",key);
                    end
                else
                    if ~ismember(trialResult.exitFlag,[-2,0,-7])
                        localReject(trialResult,trialCheck,trialQp);
                    end
                    lastOutcome = trialResult.message+"; "+strjoin(trialCheck.failedConditions,",");
                end
                % The rescue seed (slowing geometry) runs only when no verified
                % plan exists at this horizon; a verified plan never waits for it.
                rescueSeed = seedKind+1==seedCount;
                if seedKind<seedCount && ~(rescueSeed && ~isempty(best))
                    seedKind = seedKind+1;
                    continue;
                end
                if ~isempty(best)
                    model = best.model;
                    prediction = best.prediction;
                    qp = best.qp;
                    result = best.result;
                    result.solverCalls = calls;
                    check = best.check;
                    return;
                end
                % Only a complete verified plan can authorize control. Extension
                % searches a larger admission domain; no prefix is executed.
                model.horizonSteps = model.horizonSteps+1;
                seedKind = 1;
            end
        end

        function validateAdmission(ego, observations, cfg)
            if ~isfinite(ego.stateTime) || numel(observations)~=1
                error("collisionAvoidanceController:invalidExactScene", ...
                    "The study requires a timestamp and exactly one persistent target at every frame.");
            end
            if any(cfg.model.ltvModelErrorRateBound~=0) ...
                    || any(cfg.model.plantModelResidualRateBound~=0)
                error("collisionAvoidanceController:nonexactStudyInput", ...
                    "The declared-plant study excludes process residuals; estimation error boxes are admitted.");
            end
            if cfg.model.speedMinimum~=0
                error("collisionAvoidanceController:invalidExactScene", ...
                    "The invariant slowing continuation requires the speed domain to include zero.");
            end
        end

        function [matrix,bound,terminal] = completionRows(model,prediction,~)
            cfg = model.cfg;
            if any(cfg.model.ltvModelErrorRateBound~=0) || any(cfg.model.plantModelResidualRateBound~=0) ...
                    || any(arrayfun(@(target) any(target.contract.jerkBound~=0) ...
                        || target.contract.yawAccelerationBound~=0,model.encounters))
                error("collisionAvoidanceController:nonexactStudyInput", ...
                    "The invariant terminal construction requires exact stage models and an exact target law.");
            end
            % The terminal node is evaluated through the exact held-input stage
            % transitions, not the Bernstein tubes: an exact initial state then
            % has an exactly zero terminal box, and the box of an uncertain
            % state is the interval hull of the exact affine chain.
            [finalMap,finalOffset] = localExactFinalMap(model,prediction);
            finalRadius = prediction.initialErrorBound(:,end);
            if isfield(model,"prescribedTerminal") && ~isempty(model.prescribedTerminal)
                terminal = model.prescribedTerminal;
            else
                anchor = finalOffset+finalMap*model.anchorPlan;
                terminal = localTerminalSet(model,prediction,anchor);
            end
            if any(finalRadius(4:6)~=0) && ~terminal.errorBudgetFinite
                error("collisionAvoidanceController:invalidTerminalModel", ...
                    "An uncertain terminal velocity needs passive road-load damping for its open-loop error budget.");
            end
            rows = terminal.stateRows;
            matrix = rows*finalMap;
            % The nominal enters through the signed rows; the box enters through
            % the error rows, which include the open-loop future excursion.
            bound = terminal.stateBound-rows*finalOffset-terminal.errorRows*finalRadius;
            % The first terminal input must satisfy slew relative to u_(N-1). The
            % law acts on the predicted nominal, so no box enters this bound.
            rate = model.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
            selected = isfinite(rate);
            lastInput = zeros(2,prediction.planCount);
            lastInput(:,end-1:end) = eye(2);
            changeMap = terminal.feedback*finalMap-lastInput;
            changeOffset = terminal.input+terminal.feedback*finalOffset;
            matrix = [matrix;changeMap(selected,:);-changeMap(selected,:)];
            bound = [bound;rate(selected)-changeOffset(selected);rate(selected)+changeOffset(selected)];
        end

        function [stored,problem] = admit(stored,problem)
            stored.version = 19;
            stored.encounterComplete = false;
            stored.witnessModel = problem.model;
            stored.safetyScope = "verifiedPredictionWithInvariantTerminalTail";
            stored.certifiedDuration = inf;
            stored.metadata = localMetadata(problem.metadata,stored);
            problem.metadata = stored.metadata;
        end

        function [model,candidate,initialization,original] = validateTransition(stored,ego,model,observations,identity)
        % Check the executed first hold, condition both information sets and
        % assemble the carried witness before any fresh problem is built.
            cfg = model.cfg;
            hardEncounterBarrier.validateAdmission(ego,observations,cfg);
            localValidateStored(stored,identity);
            expectedTime = stored.stateTime+model.sampleTime;
            if abs(model.stateTime-expectedTime)>128*eps(max(1,abs(expectedTime))) ...
                    || ~isequal(ego.heldActuatorInput,stored.appliedInput)
                error("collisionAvoidanceController:executionContractViolation", ...
                    "Replanning requires the next sample and the previously issued input.");
            end
            plan = stored.plan;
            if ~isequal(stored.appliedInput,plan(:,1)) || size(plan,2)~=max(1,stored.remainingSteps)
                error("collisionAvoidanceController:invalidStoredCertificate","The issued input witness changed.");
            end
            if stored.remainingSteps>0
                check = solveHardCbfClf.certify(stored.qp,stored.prediction,stored.witnessModel,stored.decision);
                if ~check.accepted || ~isequal(check.value,stored.value) ...
                        || ~isequal(reshape(stored.decision(stored.qp.layout.planIndex),2,[]),plan)
                    error("collisionAvoidanceController:invalidStoredCertificate","The previous witness failed verification.");
                end
            end
            % Ego conditioning: the true state lies in the published successor
            % box and in the measurement box, so it lies in their intersection.
            predictedCenter = stored.predictedState(:,2);
            predictedRadius = stored.stateErrorBound(:,2);
            [model.initialEgoState,model.initialFrenetErrorBound] = localConditionBox( ...
                predictedCenter,predictedRadius,model.initialEgoState,model.initialFrenetErrorBound);
            % The plant speed lies in the model domain by premise, so the
            % domain is a third set in the intersection; a measurement box
            % straddling rest therefore never yields a negative centre.
            [model.initialEgoState(4),model.initialFrenetErrorBound(4)] = localDomainBox( ...
                model.initialEgoState(4),model.initialFrenetErrorBound(4), ...
                cfg.model.speedMinimum,cfg.model.speedMaximum);
            % Target conditioning against the carried exact flow.
            measured = targetPrediction.admitExact(observations,model.stateTime,model.lane,cfg);
            original = stored.originalEncounter;
            if measured.key~=original.key || measured.halfLength~=original.halfLength ...
                    || measured.halfWidth~=original.halfWidth
                error("collisionAvoidanceController:changedEncounterContract", ...
                    "The same target and footprint must persist at every frame.");
            end
            model.encounters = targetPrediction.conditionExact(stored.encounters,model.sampleTime,measured);
            % The carried witness: the shifted plan with its own data.
            keep = stored.cellStage>=2;
            carriedValue = 0;
            if ~isempty(stored.stageViolation)
                carriedValue = max(0,stored.value-stored.stageViolation(1));
            end
            candidate = struct("stages",stored.stages(min(2,end+1):end),"inputs",plan(:,2:end), ...
                "frames",stored.cellFrames(keep),"normals",{stored.cellNormals(keep)}, ...
                "terminal",stored.terminal,"carriedValue",carriedValue, ...
                "previousValue",stored.value,"previousFirstViolation",localFirst(stored.stageViolation));
            initialization = zeros(2,0);
            if stored.remainingSteps>0
                % Performance convexification must not inherit a compulsory
                % brake from the hypothetical terminal continuation.
                initialization = [plan(:,2:end),plan(:,end)];
            end
        end

        function candidate = verifyCandidate(model,candidate)
        % Verify the shifted plan with its carried data on the conditioned set.
            timer = tic;
            stageCount = numel(candidate.stages);
            carried = model;
            carried.prescribedTerminal = candidate.terminal;
            carried.exitSteps = zeros(numel(model.encounters),1);
            carried.exitMargin = inf;
            if stageCount==0
                [carried,prediction,inputs,check] = localTerminalOnlyPlan(carried,candidate.terminal);
                candidate.verificationMethod = "terminalInvariance";
            else
                carried.horizonSteps = stageCount;
                carried.prescribedStages = candidate.stages;
                carried.linearizationInputs = candidate.inputs;
                inputs = candidate.inputs;
                schedule = localPrescribedSchedule(candidate.stages);
                prediction = ltvBicycleModel.finitePredict(carried,schedule);
                if numel(prediction.cells)~=numel(candidate.frames)
                    error("collisionAvoidanceController:invalidStoredCertificate", ...
                        "The carried cell charts do not match the carried stage models.");
                end
                prediction = localPrescribeGeometry(prediction,inputs(:),candidate.frames,candidate.normals,[]);
                check = [];
                candidate.verificationMethod = "carriedEnclosureRows";
            end
            carried.anchorPlan = inputs(:);
            qp = formulateAvoidanceProblem(carried,prediction,inputs(:));
            [rowCheck,decision] = solveHardCbfClf.certifyInputs(qp,prediction,carried,inputs);
            if isempty(check)
                check = rowCheck;
            else
                % Acceptance is the node-zero terminal membership; the one-stage
                % rows are reported for diagnostics and use the same majorants.
                check.rowsAccepted = rowCheck.accepted;
                check.rowFailedConditions = rowCheck.failedConditions;
                check.clfViolation = rowCheck.clfViolation;
                check.margin = rowCheck.margin;
                check.exitMargin = rowCheck.exitMargin;
                check.sweptClearanceMargin = rowCheck.sweptClearanceMargin;
            end
            candidate.model = carried;
            candidate.prediction = prediction;
            candidate.qp = qp;
            candidate.decision = decision;
            candidate.inputs = inputs;
            candidate.check = check;
            candidate.optimizedStages = stageCount;
            candidate.seconds = toc(timer);
        end

        function record = carriedData(prediction,qp,optimizedStages)
        % Per-stage generators and per-cell charts/normals of an accepted plan.
            stages = repmat(struct("continuousA",zeros(6),"continuousB",zeros(6,2),"continuousC",zeros(6,1), ...
                "speed",0,"curvature",0,"brakingRatio",0,"tireModel",[]),optimizedStages,1);
            for stage = 1:optimizedStages
                stages(stage).continuousA = prediction.continuousA(:,:,stage);
                stages(stage).continuousB = prediction.continuousB(:,:,stage);
                stages(stage).continuousC = prediction.continuousC(:,stage);
                stages(stage).speed = prediction.scheduleSpeedProfile(stage);
                stages(stage).curvature = prediction.scheduleCurvature(stage);
                stages(stage).brakingRatio = prediction.scheduleBrakingRatio(stage);
                stages(stage).tireModel = prediction.tireModels{stage};
            end
            cellStage = reshape([prediction.cells.stage],[],1);
            retained = cellStage<=optimizedStages;
            record = struct("stages",stages,"cellStage",cellStage(retained), ...
                "cellFrames",qp.geometry.frames(retained),"cellNormals",{qp.geometry.normals(retained)}, ...
                "terminal",qp.terminal);
        end

        function states = terminalFlow(terminal,initial,steps)
        % Absolute-time evaluation of the sampled nominal terminal closed loop.
            validateattributes(steps,{'double'},{'real','finite','nonnegative','integer'});
            states = repmat(initial,1,numel(steps));
            f = terminal.continuousA(5:6,5:6);
            for index = 1:numel(steps)
                attenuation = terminal.longitudinalRatio^steps(index);
                lateralFlow = expm(f*(steps(index)*terminal.sampleTime));
                states(1:3,index) = initial(1:3) ...
                    +terminal.stepMatrix(1:3,4)*((1-attenuation)/(1-terminal.longitudinalRatio))*initial(4) ...
                    +terminal.continuousA(1:3,5:6)*(f\((lateralFlow-eye(2))*initial(5:6)));
                states(4,index) = attenuation*initial(4);
                states(5:6,index) = lateralFlow*initial(5:6);
            end
        end

        function [accepted,margins] = terminalMembership(terminal,center,radius)
        % Node-level test of the robust terminal set for a nominal and its box.
            operations = 8;
            gamma = operations*eps/(1-operations*eps);
            allowance = gamma*(abs(terminal.stateBound)+abs(terminal.stateRows)*abs(center) ...
                +abs(terminal.errorRows)*abs(radius));
            margins = terminal.stateBound-terminal.stateRows*center-terminal.errorRows*radius-allowance;
            accepted = all(isfinite(margins)) && all(margins>=0);
        end
    end
end

function value = localFirst(values)
    value = 0;
    if ~isempty(values), value = values(1); end
end

function [finalMap,finalOffset] = localExactFinalMap(model,prediction)
% Affine map from the plan to the terminal node through the exact stage flows.
    finalMap = zeros(6,prediction.planCount);
    finalOffset = model.initialEgoState;
    for stage = 1:prediction.stageCount
        finalMap = prediction.stageMatrixA(:,:,stage)*finalMap;
        finalMap(:,2*stage-1:2*stage) = finalMap(:,2*stage-1:2*stage)+prediction.stageMatrixB(:,:,stage);
        finalOffset = prediction.stageMatrixA(:,:,stage)*finalOffset+prediction.stageAffine(:,stage);
    end
end

function [center,radius] = localConditionBox(predictedCenter,predictedRadius,measuredCenter,measuredRadius)
% Interval hull of the intersection of the successor box with the measurement box.
    difference = measuredCenter-predictedCenter;
    difference(3) = atan2(sin(difference(3)),cos(difference(3)));
    allowance = 256*eps*(1+abs(predictedCenter)+abs(measuredCenter)+predictedRadius+measuredRadius);
    lower = max(-predictedRadius,difference-measuredRadius);
    upper = min(predictedRadius,difference+measuredRadius);
    if any(lower>upper+allowance)
        error("collisionAvoidanceController:inconsistentObservation", ...
            "The ego measurement box does not intersect the published successor box.");
    end
    middle = (lower+upper)/2;
    lower = min(lower,middle);
    upper = max(upper,middle);
    center = predictedCenter+middle;
    radius = (upper-lower)/2;
end

function [center,radius] = localDomainBox(center,radius,lower,upper)
% Intersect a scalar box with a fixed interval (monotone under inclusion).
    allowance = 256*eps*(1+abs(center)+radius+abs(lower)+abs(upper));
    low = max(center-radius,lower);
    high = min(center+radius,upper);
    if low>high+allowance
        error("collisionAvoidanceController:inconsistentObservation", ...
            "The conditioned ego speed box does not meet the model domain.");
    end
    high = max(high,low);
    center = (low+high)/2;
    radius = (high-low)/2;
end

function schedule = localPrescribedSchedule(stages)
    speed = [stages.speed];
    curvature = [stages.curvature];
    schedule = struct("speedProfile",[speed,speed(end)],"station",zeros(1,numel(stages)+1), ...
        "curvature",[curvature,curvature(end)],"brakingRatio",[stages.brakingRatio]);
end

function prediction = localPrescribeGeometry(prediction,anchor,frames,normals,headingOverride)
% Reuse carried charts and normals; nominals come from the new tubes.
    prediction.geometryAnchor = anchor;
    prediction.geometryFrames = frames;
    nominal = cell(numel(prediction.cells),1);
    for index = 1:numel(prediction.cells)
        tube = prediction.cells(index);
        values = reshape(pagemtimes(tube.map,anchor),6,[])+tube.offset;
        if ~isempty(headingOverride)
            % Anchor the heading majorant where the terminal rows anchored it,
            % so the one-stage rows are implied by the terminal membership.
            values(3,:) = headingOverride;
        end
        nominal{index} = values;
    end
    prediction.geometryNominal = nominal;
    prediction.separationNormals = normals;
end

function [carried,prediction,inputs,check] = localTerminalOnlyPlan(carried,terminal)
% No optimized stage remains: the terminal law itself is the verified plan.
    center = carried.initialEgoState;
    radius = carried.initialFrenetErrorBound;
    [accepted,margins] = hardEncounterBarrier.terminalMembership(terminal,center,radius);
    inputs = terminal.input+terminal.feedback*center;
    stage = struct("continuousA",terminal.continuousA,"continuousB",terminal.continuousB, ...
        "continuousC",terminal.continuousC,"speed",0,"curvature",terminal.curvature, ...
        "brakingRatio",inputs(2),"tireModel",terminal.tireModel);
    carried.horizonSteps = 1;
    carried.prescribedStages = stage;
    carried.linearizationInputs = inputs;
    prediction = ltvBicycleModel.finitePredict(carried,localPrescribedSchedule(stage));
    frame = terminal.frame;
    frame.referenceHeadingErrorBound = frame.headingErrorBound;
    frames = repmat(frame,numel(prediction.cells),1);
    normals = repmat({terminal.targetNormals},numel(prediction.cells),1);
    prediction = localPrescribeGeometry(prediction,inputs(:),frames,normals,terminal.anchorHeading);
    check = struct("accepted",accepted,"failedConditions",strings(1,0), ...
        "hardRowViolation",max([0;-margins]),"clfViolation",-inf,"margin",min(margins), ...
        "sweptClearanceMargin",min(margins),"exitMargin",min(margins), ...
        "value",0,"stageViolation",0,"terminalMargins",margins);
    if ~accepted, check.failedConditions = "terminalMembership"; end
end

function terminal = localTerminalSet(model,prediction,anchor)
% Robust terminal set: pose rows with nominal and open-loop error budgets,
% velocity box, nonnegative nominal speed and all-future target separation.
    cfg = model.cfg;
    curvature = laneGeometry.curvature(anchor(1),model.lane);
    terminal = localTerminalDynamics(model,curvature);
    % Include the certified stopping excursion of nominal plus error in the
    % chart. A local linearization radius is not a required stopping position.
    excursion = max(terminal.poseExcursion(1,:),terminal.errorExcursion(1,:));
    terminalRadius = cfg.controller.stationTrustRadius+excursion*terminal.velocityLimit;
    frame = laneGeometry.frameBounds(model.lane,anchor(1),terminalRadius,cfg.model.lateralDomainRadius);
    data = struct("frame",[frame.origin;frame.tangent;frame.lateral;frame.heading; ...
        frame.positionErrorBound;frame.headingErrorBound;frame.stationLower;frame.stationUpper], ...
        "nominal",anchor,"targets",struct([]),"boundaries",model.road.boundaries, ...
        "settings",[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.model.headingDomainRadius; ...
            cfg.model.lateralDomainRadius;cfg.collision.clearanceMargin], ...
        "duration",0,"degree",3);
    roadRows = avoidanceSafetyGeometry.cellRows(data);
    poseRows = [eye(3);-eye(3);roadRows.state(:,1:3)];
    poseBound = [frame.stationUpper;cfg.model.lateralDomainRadius;cfg.model.headingDomainRadius; ...
        -frame.stationLower;cfg.model.lateralDomainRadius;cfg.model.headingDomainRadius;roadRows.bound];
    duration = prediction.stageCount*model.sampleTime;
    normals = zeros(2,numel(model.encounters));
    futureSupports = zeros(numel(model.encounters),1);
    for index = 1:numel(model.encounters)
        target = model.encounters(index);
        [center,radius] = targetPrediction.finiteFlow(target,duration);
        egoPosition = frame.origin+[frame.tangent,frame.lateral]*anchor(1:2);
        displacement = egoPosition-center(1:2);
        [normal,support] = localAdmissibleNormal(displacement,center,radius,frame,egoPosition);
        normalNorm = norm(normal)+64*eps*(1+norm(normal));
        if center(8)~=0 || radius(8)~=0
            bodySupport = hypot(target.halfLength,target.halfWidth)*normalNorm;
        else
            bodySupport = targetPrediction.rectangleSupport(target.halfLength,target.halfWidth, ...
                normal,center(7),radius(7));
        end
        [egoSupport,headingSlope] = targetPrediction.rectangleSupportMajorant( ...
            normal,frame.heading,cfg.vehicle.length/2,cfg.vehicle.width/2, ...
            anchor(3),cfg.model.headingDomainRadius+frame.headingErrorBound);
        poseRows = [poseRows;repmat(-normal.'*[frame.tangent,frame.lateral],numel(egoSupport),1),headingSlope]; %#ok<AGROW>
        collisionLimit = normal.'*frame.origin-support-bodySupport-egoSupport ...
            -cfg.collision.clearanceMargin*normalNorm-abs(normal).'*frame.positionErrorBound ...
            -abs(headingSlope)*frame.headingErrorBound;
        poseBound = [poseBound;collisionLimit]; %#ok<AGROW>
        normals(:,index) = normal;
        futureSupports(index) = support;
    end
    % A p + R |v| <= b is invariant for the terminal comparison dynamics.
    % Enumerate signs of the nominal velocity to obtain ordinary linear rows.
    signs = 2*double(dec2bin(0:7,3)-'0')-1;
    poseCount = size(poseRows,1);
    projectedFlow = poseRows*terminal.continuousA(1:3,4:6);
    % The nominal longitudinal velocity stays nonnegative under the braking
    % law, so a lower station bound needs no fictitious backward budget.
    growth = [max(projectedFlow(:,1),0),abs(projectedFlow(:,2:3))];
    budget = localBudget(growth,terminal.comparison);
    % The estimation error evolves open loop. Its budget is symmetric and it
    % dominates the nominal budget, which keeps membership monotone under
    % box inclusion.
    if terminal.errorBudgetFinite
        errorBudget = localBudget(abs(projectedFlow),terminal.errorComparison);
        if any(errorBudget<budget-1024*eps*(1+abs(budget)),"all")
            error("collisionAvoidanceController:invalidTerminalModel", ...
                "The open-loop error budget must dominate the nominal budget.");
        end
        errorBudget = max(errorBudget,budget);
    else
        errorBudget = budget;
    end
    rows = [repelem(poseRows,8,1),repelem(budget,8,1).*repmat(signs,poseCount,1)];
    limits = repelem(poseBound,8);
    errorRows = [repelem(abs(poseRows),8,1),repelem(errorBudget,8,1)];
    % Both signs of the velocity box and the nonnegative nominal speed.
    % The nominal longitudinal speed stays nonnegative by its braking law;
    % charging its radius keeps this row monotone under box inclusion too.
    rows = [rows;zeros(6,3),[eye(3);-eye(3)];0,0,0,-1,0,0];
    limits = [limits;terminal.velocityLimit;terminal.velocityLimit;0];
    errorRows = [errorRows;zeros(6,3),[eye(3);eye(3)];0,0,0,1,0,0];
    terminal.poseBudget = budget;
    terminal.poseErrorBudget = errorBudget;
    terminal.stateRows = rows;
    terminal.stateBound = limits;
    terminal.errorRows = errorRows;
    terminal.poseRows = poseRows;
    terminal.poseBound = poseBound;
    terminal.frame = frame;
    terminal.anchorHeading = anchor(3);
    terminal.anchorState = anchor;
    terminal.targetNormals = normals;
    terminal.futureTargetSupports = futureSupports;
end

function budget = localBudget(growth,comparison)
% Nonnegative row budgets R with R*C + growth <= 0, with a checked reserve.
    poseCount = size(growth,1);
    budget = growth/(-comparison);
    budget = budget+4096*eps*(1+norm(budget,inf))*ones(poseCount,1)*(ones(1,3)/(-comparison));
    residual = budget*comparison+growth;
    allowance = 128*eps*(abs(budget)*abs(comparison)+abs(growth));
    if any(budget<0,"all") || any(residual+allowance>0,"all")
        error("collisionAvoidanceController:invalidTerminalModel", ...
            "A terminal excursion budget failed verification.");
    end
end

function terminal = localTerminalDynamics(model,curvature)
    cfg = model.cfg;
    h = model.sampleTime;
    gain = modifiedFialaTire.accelerationGain(cfg);
    if model.longitudinalAccelerationBias~=0
        error("collisionAvoidanceController:invalidTerminalModel", ...
            "The invariant rest schedule requires zero independent acceleration bias.");
    end
    input = [0;-model.longitudinalAccelerationBias/gain];
    [a,b,c,tireModel] = ltvBicycleModel.continuousMatrices(curvature,0,cfg,input(2),model.longitudinalAccelerationBias);
    if any(a(:,1:3)~=0,"all") || any(a(5:6,4)~=0) || any(b(5:6,2)~=0) ...
            || norm(c+b*input,inf)>64*eps*(1+norm(c,inf))
        error("collisionAvoidanceController:invalidTerminalModel","The terminal scheduled rest structure is not valid.");
    end
    damping = -a(4,4);
    if damping==0, phi = h; else, phi = -expm1(-damping*h)/damping; end
    brakeGain = exp(-damping*h)*(-expm1(-h))/phi;
    feedback = zeros(2,6);
    feedback(2,4) = -brakeGain/gain;
    flow = expm(h*[a,b;zeros(2,8)]);
    step = flow(1:6,1:6)+flow(1:6,7:8)*feedback;
    rho = exp(-(damping+1)*h);
    % Open-loop error comparison and its braked nominal counterpart.
    errorComparison = abs(a(4:6,4:6));
    errorComparison(1:4:end) = diag(a(4:6,4:6));
    comparison = errorComparison;
    comparison(1,1) = comparison(1,1)-brakeGain;
    [errorDirection,errorHurwitz] = localComparisonDirection(errorComparison);
    if errorHurwitz
        direction = errorDirection;
    else
        [direction,hurwitz] = localComparisonDirection(comparison);
        if ~hurwitz
            error("collisionAvoidanceController:invalidTerminalModel","No contracting terminal velocity comparison exists.");
        end
    end
    if any(comparison*direction>=0) || ~(rho>0 && rho<1)
        error("collisionAvoidanceController:invalidTerminalModel","No contracting terminal velocity comparison exists.");
    end
    excursion = localExcursion(abs(a(1:3,4:6)),comparison);
    if errorHurwitz
        errorExcursion = localExcursion(abs(a(1:3,4:6)),errorComparison);
    else
        errorExcursion = excursion;
    end
    caps = [cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
    caps(1) = min(caps(1),(input(2)-cfg.actuation.brakingRatioMinimum)*gain/brakeGain);
    rate = h*cfg.model.brakingRatioRateMaximum;
    if isfinite(rate), caps(1) = min(caps(1),rate*gain/(brakeGain*(1-rho))); end
    slip = cfg.model.slipAngleMaximum(:);
    if isscalar(slip), slip = repmat(slip,2,1); end
    slipDirection = [direction(2)+cfg.vehicle.lf*direction(3); ...
        direction(2)+cfg.vehicle.lr*direction(3)]/cfg.model.scheduleSpeedFloor;
    % At zero scheduled speed the longitudinal and lateral velocity channels
    % are decoupled in both comparison matrices, so each block is scaled to
    % its own limits; a single scale would let the longitudinal cap starve
    % the lateral box. The joint contraction is verified below.
    longitudinalScale = caps(1)/direction(1);
    lateralScale = min([caps(2:3)./direction(2:3);slip./slipDirection]);
    if ~(longitudinalScale>0) || ~(lateralScale>0) ...
            || input(2)>cfg.actuation.brakingRatioMaximum || input(2)<cfg.actuation.brakingRatioMinimum
        error("collisionAvoidanceController:invalidTerminalModel","The terminal input and velocity domain are empty.");
    end
    velocityLimit = 0.99*[longitudinalScale*direction(1);lateralScale*direction(2:3)];
    reserve = 128*eps*(abs(comparison)*velocityLimit);
    if any(comparison*velocityLimit+reserve>=0) ...
            || (errorHurwitz && any(errorComparison*velocityLimit+128*eps*(abs(errorComparison)*velocityLimit)>=0))
        error("collisionAvoidanceController:invalidTerminalModel","The scaled terminal velocity box is not contracting.");
    end
    terminal = struct("continuousA",a,"continuousB",b,"continuousC",c,"tireModel",tireModel, ...
        "input",input,"feedback",feedback,"stepMatrix",step,"sampleTime",h, ...
        "longitudinalRatio",rho,"comparison",comparison,"errorComparison",errorComparison, ...
        "errorBudgetFinite",errorHurwitz,"poseExcursion",excursion,"errorExcursion",errorExcursion, ...
        "velocityLimit",velocityLimit,"curvature",curvature, ...
        "feedbackArgument","predictedNominalVelocity");
end

function [direction,hurwitz] = localComparisonDirection(comparison)
% A positive vector with C*q < 0 certifies that the Metzler matrix C is Hurwitz.
    hurwitz = false;
    direction = zeros(3,1);
    if rcond(-comparison)<=1e-12, return; end
    direction = (-comparison)\ones(3,1);
    reserve = 128*eps*(abs(comparison)*abs(direction));
    hurwitz = all(direction>0) && all(comparison*direction+reserve<0);
end

function excursion = localExcursion(growth,comparison)
    excursion = growth/(-comparison);
    reserve = 4096*eps*(1+norm(excursion,inf));
    excursion = excursion+reserve*ones(3,1)*(ones(1,3)/(-comparison));
    residual = excursion*comparison+growth;
    residualReserve = 128*eps*(abs(excursion)*abs(comparison)+abs(growth));
    if any(residual+residualReserve>0,"all")
        error("collisionAvoidanceController:invalidTerminalModel","The excursion comparison failed numerical verification.");
    end
end

function [normal,support] = localAdmissibleNormal(displacement,center,radius,frame,egoPosition)
% Choose a terminal separating normal whose all-future box support is finite.
% Candidates are the displacement-based proposal and the chart axes; among
% the admissible ones, keep the largest current clearance along the normal.
    proposal = localFutureNormal(displacement,center,frame.lateral);
    candidates = [proposal,frame.lateral,-frame.lateral,frame.tangent,-frame.tangent];
    best = -inf;
    normal = proposal;
    support = inf;
    for index = 1:size(candidates,2)
        direction = candidates(:,index)/norm(candidates(:,index));
        value = localFutureSupport(center,radius,direction);
        if ~isfinite(value), continue; end
        clearance = direction.'*egoPosition-value;
        if clearance>best
            best = clearance;
            normal = direction;
            support = value;
        end
    end
    if ~isfinite(support)
        error("collisionAvoidanceController:unboundedTargetSupport", ...
            "No terminal halfspace separates the ego from the target's entire future: its " ...
            + "velocity or acceleration box is not receding along any admissible direction.");
    end
end

function normal = localFutureNormal(displacement,center,lateral)
    normal = displacement;
    direction = center(5:6);
    if ~any(direction), direction = center(3:4); end
    if any(direction) && dot(normal,direction)>0
        normal = normal-direction*(dot(normal,direction)/dot(direction,direction));
    end
    if norm(normal)<1e-10*(1+norm(displacement))
        if any(direction)
            normal = [-direction(2);direction(1)];
            if dot(normal,lateral)<0, normal = -normal; end
        else
            normal = lateral;
        end
    end
    normal = normal/norm(normal);
    if any(direction) && dot(normal,direction)>=-1e-10*norm(direction)
        normal = normal-1e-9*direction/norm(direction);
        normal = normal/norm(normal);
    end
end

function support = localFutureSupport(center,radius,normal)
% Supremum over all future time of the box's directional position support.
% Upper coefficients enclose every member of the box; a derivative sign is
% never inferred by rounding a small positive value to zero.
    initialPosition = dot(normal,center(1:2))+abs(normal).'*radius(1:2);
    velocity = dot(normal,center(3:4))+abs(normal).'*radius(3:4);
    acceleration = 0.5*(dot(normal,center(5:6))+abs(normal).'*radius(5:6));
    errorAcceleration = 128*eps*(sum(abs(normal.*center(5:6)))/2+abs(normal).'*radius(5:6));
    errorVelocity = 128*eps*(sum(abs(normal.*center(3:4)))+abs(normal).'*radius(3:4));
    errorPosition = 128*eps*(sum(abs(normal.*center(1:2)))+abs(normal).'*radius(1:2));
    velocity = velocity+errorVelocity;
    acceleration = acceleration+errorAcceleration;
    support = initialPosition+errorPosition;
    if acceleration>0 || (acceleration==0 && velocity>0)
        support = inf;
    elseif acceleration<0 && velocity>0
        support = support-velocity^2/(4*acceleration);
    end
    support = support+256*eps*(1+abs(support)+norm(center(1:2)));
end

function [schedule,anchor] = localPredictionSchedule(model,seedKind)
    cfg = model.cfg;
    count = model.horizonSteps;
    speed = repmat(model.initialEgoState(4),1,count+1);
    shifted = isfield(model,"initializationPlan");
    brakingSeed = seedKind==2+double(shifted);
    if brakingSeed
        speed = linspace(speed(1),min(0.25,speed(1)),count+1);
    end
    station = model.initialEgoState(1)+[0,cumsum(model.sampleTime*(speed(1:end-1)+speed(2:end))/2)];
    curvature = arrayfun(@(s) laneGeometry.curvature(s,model.lane),station);
    inputs = zeros(2,count);
    prior = model.previousInput;
    % Fiala derivatives are singular at |beta|=1. Keep only the schedule
    % anchor inside that boundary; optimization retains the configured limits.
    lower = [-cfg.model.frontWheelSteeringAngleMaximum;max(-0.95,cfg.actuation.brakingRatioMinimum)];
    upper = [cfg.model.frontWheelSteeringAngleMaximum;min(0.95,cfg.actuation.brakingRatioMaximum)];
    change = model.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    for stage = 1:count
        stageCfg = cfg;
        stageCfg.referenceSpeed = speed(stage);
        [~,input] = ltvBicycleModel.cruiseEquilibrium(curvature(stage),stageCfg,model.longitudinalAccelerationBias);
        input(2) = input(2)+(speed(stage+1)-speed(stage))/(model.sampleTime*modifiedFialaTire.accelerationGain(cfg));
        if shifted && seedKind==1
            input = model.initializationPlan(:,min(stage,size(model.initializationPlan,2)));
        end
        inputs(:,stage) = min(max(input,max(lower,prior-change)),min(upper,prior+change));
        prior = inputs(:,stage);
    end
    schedule = struct("speedProfile",speed,"station",station,"curvature",curvature,"brakingRatio",inputs(2,:));
    anchor = inputs(:);
end

function localValidateStored(stored,identity)
    required = ["version","stateTime","appliedInput","plan","value","stageViolation","witnessModel","qp", ...
        "decision","prediction","predictedState","stateErrorBound","metadata","identity","encounters", ...
        "originalEncounter","stages","cellStage","cellFrames","cellNormals","terminal","remainingSteps"];
    if ~isstruct(stored) || ~isscalar(stored) || ~all(isfield(stored,required)) ...
            || stored.version~=19 || ~isfield(stored.terminal,"errorRows")
        error("collisionAvoidanceController:invalidStoredCertificate","Use a version-19 carried-witness certificate.");
    end
    if ~isequaln(identity,stored.identity)
        error("collisionAvoidanceController:changedExecutionContract", ...
            "The retained declared plant, road, route and physical limits must be unchanged.");
    end
end

function metadata = localMetadata(metadata,stored)
    metadata.fallbackUsed = false;
    metadata.carriedMargin = stored.margin;
    metadata.requiredMargin = stored.qp.requiredMargin;
    metadata.pcbfValue = stored.value;
    metadata.stageViolation = stored.stageViolation;
    metadata.barrierValue = stored.value;
    metadata.barrierInterpretation = "verifiedAccumulatedSafetyViolation";
    metadata.horizonSteps = stored.remainingSteps;
    metadata.planningWindowSteps = stored.identity.configuration.controller.horizonSteps;
    metadata.certificateExtensionSteps = max(0,stored.prediction.stageCount-metadata.planningWindowSteps);
    metadata.deadline = stored.deadline;
    metadata.encounterComplete = false;
    metadata.safetyScope = stored.safetyScope;
    metadata.certifiedDuration = inf;
    metadata.commandCertifiedDuration = stored.witnessModel.sampleTime;
    metadata.lookaheadDuration = stored.remainingSteps*stored.witnessModel.sampleTime;
    metadata.recursiveFeasibilityClaimed = true;
    metadata.recursiveFeasibilityScope = "declaredAffineStagePlant;exactTargetLaw;boundedEstimationError;conditionedInformationSets";
    metadata.indefiniteRecursiveFeasibilityClaimed = true;
    metadata.terminalContinuationCertified = true;
    metadata.terminalPolicyRole = "carriedWitnessTail";
    metadata.exactPredictionAssumptionsHold = true;
    metadata.physicalVehicleGuaranteeEstablished = false;
    metadata.jointAdmissionPerformed = false;
    metadata.newlyAdmittedTargetKeys = strings(1,0);
    metadata.planCertified = true;
    metadata.acceptance = stored.acceptance;
    metadata.hardRowViolation = stored.acceptance.hardRowViolation;
    metadata.activeTargetKeys = string({stored.encounters.key});
    metadata.dischargedTargetKeys = strings(1,0);
    metadata.commandActuationTime = stored.stateTime;
end

function localReject(result,check,qp)
    detail = "";
    if isfield(check,"violatedHardRows") && ~isempty(check.violatedHardRows)
        names = solveHardCbfClf.rowNames(qp);
        detail = "; violated hard rows: "+strjoin(unique(names(check.violatedHardRows)).',",");
    end
    error("collisionAvoidanceController:noCertifiedContinuation", ...
        "No verified plan was obtained: %s; %s%s.",result.message,strjoin(check.failedConditions,","),detail);
end
