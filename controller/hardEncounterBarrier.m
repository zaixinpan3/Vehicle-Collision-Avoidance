classdef hardEncounterBarrier
    %hardEncounterBarrier Rolling safe MPC with a carried recursive-feasibility witness.
    % The plan accepted at one frame is carried, together with its own stage
    % generators, cell charts, separating normals and terminal rows, to the
    % next frame. There the shifted plan is verified on the conditioned
    % information set before any fresh optimization may replace it. The
    % analytic terminal law is the tail of that witness; it is commanded only
    % when no optimized stage remains in the carried plan.

    methods (Static)
        function confirmation = admitConfirmation(ego,model)
            hardEncounterBarrier.confirmationObservation(ego,model.stateTime,[],true);
            range = ego.perception.range;
            target = model.encounters;
            minimum = hypot(model.cfg.vehicle.length,model.cfg.vehicle.width)/2 ...
                +hypot(target.halfLength,target.halfWidth)+model.cfg.collision.clearanceMargin;
            if range<=minimum
                error("collisionAvoidanceController:invalidConfirmationRegion", ...
                    "The confirmation range must exceed both body radii plus collision clearance.");
            end
            confirmation = struct("range",range,"reference","egoReferencePoint", ...
                "exitGeometry","entireTargetFootprint","confirmationDelay",0, ...
                "observationContract","currentCompleteObservationAtConfirmationSample");
            [direction,steps,passing] = localEncounterProposal(model,range);
            confirmation.exitDirection = direction;
            confirmation.searchHorizonSteps = steps;
            confirmation.passingRequired = passing;
        end

        function valid = confirmationObservation(ego,time,confirmation,required)
            p = ego.perception;
            valid = isstruct(p) && isscalar(p) ...
                && all(isfield(p,["time","range","completeWithinRange"]));
            if valid
                valid = isnumeric(p.time) && isscalar(p.time) && isfinite(p.time) ...
                    && abs(p.time-time)<=128*eps(max(1,abs(time))) ...
                    && isnumeric(p.range) && isscalar(p.range) && isfinite(p.range) && p.range>0 ...
                    && islogical(p.completeWithinRange) && isscalar(p.completeWithinRange) ...
                    && p.completeWithinRange;
            end
            if valid && ~isempty(confirmation) && p.range~=confirmation.range
                error("collisionAvoidanceController:changedConfirmationRegion", ...
                    "The current scan and the carried finite exit must use the same range.");
            end
            if required && ~valid
                error("collisionAvoidanceController:unconfirmedTargetDeparture", ...
                    "Finite encounter admission or release requires a current complete-within-range observation.");
            end
        end

        function outside = observedExterior(model,target,completion)
            z = model.initialEgoState;
            rho = model.initialFrenetErrorBound;
            frame = laneGeometry.frameBounds(model.lane,z(1), ...
                max(model.cfg.controller.stationTrustRadius,rho(1)),model.cfg.model.lateralDomainRadius);
            direction = localExitDirection(target.center,frame,z);
            if ~isempty(model.confirmation.exitDirection)
                direction = model.confirmation.exitDirection;
            end
            if nargin>2
                % At the certified final node, its carried chart/direction
                % preserve the proven exterior row under box conditioning.
                % A new direction need not be better for anisotropic boxes.
                frame = completion.frame;
                direction = completion.direction;
            end
            [row,bound] = localExitRow(model,target,target.center,target.radius,rho,frame,direction);
            allowance = 32*eps*(abs(bound)+abs(row)*abs(z));
            outside = row*z+allowance<=bound;
        end

        function requirePossibleAbsence(model,carried)
        % A complete scan is a set-valued observation too. An absent object
        % cannot be reconciled with a reachable reference box wholly inside
        % the declared region. Partial intersection needs no favorable reset.
            [center,radius] = targetPrediction.finiteFlow(carried,model.sampleTime);
            z = model.initialEgoState;
            rho = model.initialFrenetErrorBound;
            frame = laneGeometry.frameBounds(model.lane,z(1), ...
                max(model.cfg.controller.stationTrustRadius,rho(1)),model.cfg.model.lateralDomainRadius);
            map = [frame.tangent,frame.lateral];
            delta = center(1:2)-frame.origin-map*z(1:2);
            positionRadius = radius(1:2)+abs(map)*rho(1:2)+frame.positionErrorBound;
            upper = norm(delta)+norm(positionRadius);
            upper = upper+256*eps*(1+upper+norm(center(1:2))+norm(frame.origin));
            if upper<model.confirmation.range
                error("collisionAvoidanceController:inconsistentObservation", ...
                    "The complete scan reports absence while the carried target set is wholly inside its range.");
            end
        end

        function [matrix,bound,completion] = finiteCompletionRows(model,prediction,finalMap,finalOffset,frame)
            matrix = zeros(0,prediction.planCount);
            bound = zeros(0,1);
            deadline = model.stateTime+prediction.stageCount*model.sampleTime;
            completion = struct("active",false,"deadline",NaN,"direction",zeros(2,0), ...
                "frame",frame,"stateRow",zeros(0,6),"stateBound",zeros(0,1));
            if isempty(model.encounters), return; end
            if ~isfield(model,"confirmation") || isempty(model.confirmation)
                error("collisionAvoidanceController:missingConfirmationContract", ...
                    "A finite exit certificate requires the declared confirmation region.");
            end
            target = model.encounters;
            [center,radius] = targetPrediction.finiteFlow(target,prediction.stageCount*model.sampleTime);
            anchor = finalOffset+finalMap*model.anchorPlan;
            direction = localExitDirection(center,frame,anchor);
            if ~isempty(model.confirmation.exitDirection)
                direction = model.confirmation.exitDirection;
            end
            if isfield(model,"prescribedCompletion") && model.prescribedCompletion.active
                direction = model.prescribedCompletion.direction;
                frame = model.prescribedCompletion.frame;
            end
            [row,limit] = localExitRow(model,target,center,radius,prediction.initialErrorBound(:,end),frame,direction);
            matrix = row*finalMap;
            bound = limit-row*finalOffset;
            completion = struct("active",true,"deadline",deadline,"direction",direction, ...
                "frame",frame,"stateRow",row,"stateBound",limit);
        end

        function [model,prediction,qp,result,check,timing,failure] = plan(model)
            % Fresh search. Every seed is a convexification anchor only: the CLF
            % majorant is exact at its anchor and grows quadratically away from
            % it, so a verified plan from one seed is not the tie-break optimum
            % of another. The shifted seed is solved first unless the previous
            % accepted plan's tail decelerated, in which case the equilibrium
            % seed goes first; the second of the pair runs only when the first
            % plan's tail decelerates below the current speed (the lock-in
            % signature), and the verified plan with the smaller (value,
            % objective) pair is kept. The slowing seed is a rescue tried only
            % when nothing verifies. The horizon is scaled with speed; the
            % carried witness keeps its own length, so a shorter fresh horizon
            % never weakens the guarantee. With a witness available, no attempt
            % starts after the work deadline. Observed costs screen long
            % attempts, and the native solver receives the remaining budget.
            % Initial admission has its own search budget and no executable
            % prefix until a complete witness has been verified.
            % Failures are returned, not thrown, so their timing is reported.
            cfg = model.cfg;
            timing = struct("predictionSeconds",0,"formulationSeconds",0, ...
                "solveSeconds",0,"verificationSeconds",0,"attempts",0,"deadlineHit",false, ...
                "maximumAttemptSeconds",0,"lastAttemptStages",0);
            failure = "";
            prediction = [];qp = [];result = [];check = [];
            calls = 0;
            searchTimer = tic;
            shifted = isfield(model,"initializationPlan");
            if shifted
                seeds = [1,2,3];
                if isfield(model,"preferEquilibriumSeed") && model.preferEquilibriumSeed
                    seeds = [2,1,3];
                end
            else
                seeds = [1,4,5,2];
                if ~isempty(model.encounters) && model.confirmation.passingRequired
                    seeds = [4,5,1,2];
                end
            end
            lastOutcome = "no attempt";
            best = [];
            slot = 1;
            attemptSeconds = 0;
            deadlineGuarded = isfield(model,"frameTimer");
            speedRatio = min(1,max(0,model.initialEgoState(4)/max(model.referenceSpeed,eps)));
            model.horizonSteps = max(min(cfg.controller.minimumHorizonSteps,cfg.controller.horizonSteps), ...
                min(cfg.controller.horizonSteps,ceil(cfg.controller.horizonSteps*speedRatio)));
            if ~isempty(model.encounters)
                model.horizonSteps = max(model.horizonSteps,model.confirmation.searchHorizonSteps);
                [position,heading] = laneGeometry.fromFrenet(model.initialEgoState,model.lane);
                target = model.encounters;
                distance = avoidanceSafetyGeometry.rectangleDistance(position,heading,target.center(1:2),target.center(7), ...
                    [cfg.vehicle.length/2;cfg.vehicle.width/2;target.halfLength;target.halfWidth]);
                if distance<cfg.collision.clearanceMargin
                    failure = "collisionAvoidanceController:noCertifiedContinuation: " ...
                        +"The admitted state box contains an already unsafe nominal footprint pair.";
                    return;
                end
            end
            maximumSteps = inf;
            if ~isempty(model.encounters) && isfield(model,"exitDeadline")
                maximumSteps = round((model.exitDeadline-model.stateTime)/model.sampleTime);
                model.horizonSteps = min(model.horizonSteps,maximumSteps);
                if maximumSteps<1
                    failure = "collisionAvoidanceController:unconfirmedEncounterExit: " ...
                        +"The finite encounter deadline has no confirmed release.";
                    return;
                end
            end
            while true
                if timing.attempts>0 && isempty(best) && toc(searchTimer)>=cfg.solver.certificateSearchTimeLimit
                    failure = sprintf("collisionAvoidanceController:certificateSearchLimit: The search budget " ...
                        + "expired without a verified plan after %d attempts (last: %s); infeasibility is not established.", ...
                        timing.attempts,lastOutcome);
                    return;
                end
                estimate = attemptSeconds;
                if isfield(model,"previousAttemptSeconds") && model.horizonSteps>cfg.controller.horizonSteps
                    estimate = max(estimate,model.previousAttemptSeconds* ...
                        (model.horizonSteps/model.previousAttemptStages)^2);
                end
                if deadlineGuarded && toc(model.frameTimer)+estimate ...
                        +min(0.02,0.2*cfg.solver.frameDeadlineSeconds)>=cfg.solver.frameDeadlineSeconds
                    timing.deadlineHit = true;
                    if isempty(best)
                        failure = sprintf("collisionAvoidanceController:frameDeadline: The frame deadline of %.3g s " ...
                            + "leaves no room for another attempt after %d (last: %s); the carried witness is the command.", ...
                            cfg.solver.frameDeadlineSeconds,timing.attempts,lastOutcome);
                        return;
                    end
                    break;
                end
                attemptTimer = tic;
                timing.attempts = timing.attempts+1;
                seedKind = seeds(slot);
                trial = model;
                trial.exitSteps = zeros(numel(model.encounters),1);
                trial.exitMargin = inf;
                phase = tic;
                [schedule,anchor] = localPredictionSchedule(trial,seedKind);
                trial.linearizationInputs = reshape(anchor,2,[]);
                trialPrediction = ltvBicycleModel.finitePredict(trial,schedule);
                if ~isempty(trial.encounters) && ismember(seedKind,[4,5])
                    trialPrediction.separationNormals = avoidanceSafetyGeometry.passingNormals( ...
                        trial,trialPrediction,anchor,2*double(seedKind==5)-1);
                end
                timing.predictionSeconds = timing.predictionSeconds+toc(phase);
                phase = tic;
                trialQp = formulateAvoidanceProblem(trial,trialPrediction,anchor);
                timing.formulationSeconds = timing.formulationSeconds+toc(phase);
                phase = tic;
                solveCfg = cfg;
                solveCfg.solver.workTimer = searchTimer;
                solveCfg.solver.workTimeLimit = cfg.solver.certificateSearchTimeLimit;
                if ~shifted && isfinite(cfg.solver.certificateSearchTimeLimit)
                    % Reserve search time for the other normal families. A
                    % difficult convexification must not consume every trial.
                    elapsed = toc(searchTimer);
                    remaining = max(0,cfg.solver.certificateSearchTimeLimit-elapsed);
                    solveCfg.solver.workTimeLimit = elapsed+remaining/(numel(seeds)-slot+1);
                end
                if deadlineGuarded
                    solveCfg.solver.workTimer = model.frameTimer;
                    solveCfg.solver.workTimeLimit = cfg.solver.frameDeadlineSeconds ...
                        -min(0.02,0.2*cfg.solver.frameDeadlineSeconds);
                end
                [trialResult,trialQp] = solveHardCbfClf.solve(trialQp,solveCfg);
                calls = calls+trialResult.solverCalls;
                timing.solveSeconds = timing.solveSeconds+toc(phase);
                phase = tic;
                trialCheck = solveHardCbfClf.certify(trialQp,trialPrediction,trial,trialResult.decision);
                timing.verificationSeconds = timing.verificationSeconds+toc(phase);
                elapsed = toc(attemptTimer);
                if elapsed>=attemptSeconds
                    timing.lastAttemptStages = model.horizonSteps;
                end
                attemptSeconds = max(attemptSeconds,elapsed);
                timing.maximumAttemptSeconds = attemptSeconds;
                deceleratingTail = false;
                if trialResult.feasible && trialCheck.accepted
                    objective = 0.5*trialResult.decision.'*trialQp.Hessian*trialResult.decision ...
                        +trialQp.linear.'*trialResult.decision+trialQp.constant;
                    key = [trialCheck.value,objective];
                    if isempty(best) || key(1)<best.key(1) || (key(1)==best.key(1) && key(2)<best.key(2))
                        best = struct("model",trial,"prediction",trialPrediction,"qp",trialQp, ...
                            "result",trialResult,"check",trialCheck,"key",key);
                    end
                    plan = trialResult.decision(trialQp.layout.planIndex);
                    node = trialPrediction.egoStateOffset(:,end)+trialPrediction.egoStateMatrix(:,:,end)*plan;
                    deceleratingTail = node(4)<model.initialEgoState(4)-0.25 ...
                        && model.initialEgoState(4)<model.referenceSpeed;
                else
                    diagnosticValue = trialCheck.candidateAccepted && trialCheck.value>0;
                    if ~ismember(trialResult.exitFlag,[-2,0,2,-7]) && ~diagnosticValue
                        failure = localRejectMessage(trialResult,trialCheck,trialQp);
                        return;
                    end
                    lastOutcome = trialResult.message+"; "+strjoin(trialCheck.failedConditions,",");
                end
                if ~shifted && ~isempty(best), break; end
                nextIsRescue = slot+1==numel(seeds);
                secondOfPair = shifted && slot==1;
                if slot<numel(seeds) && ~(nextIsRescue && ~isempty(best)) ...
                        && ~(secondOfPair && ~isempty(best) && ~deceleratingTail)
                    slot = slot+1;
                    continue;
                end
                if ~isempty(best), break; end
                if model.horizonSteps>=maximumSteps
                    failure = "collisionAvoidanceController:noCertifiedContinuation: " ...
                        +"No fresh zero-violation finite completion was verified within the carried exit deadline.";
                    return;
                end
                % Only a complete verified plan can authorize control. Extension
                % searches a larger admission domain; no prefix is executed.
                model.horizonSteps = min(maximumSteps, ...
                    model.horizonSteps+max(1,ceil(0.2*model.horizonSteps)));
                slot = 1;
            end
            timing.deadlineHit = timing.deadlineHit || (deadlineGuarded ...
                && toc(model.frameTimer)>=cfg.solver.frameDeadlineSeconds);
            model = best.model;
            prediction = best.prediction;
            qp = best.qp;
            result = best.result;
            result.solverCalls = calls;
            check = best.check;
        end

        function validateAdmission(ego, observations, cfg)
            if ~isfinite(ego.stateTime) || numel(observations)>1
                error("collisionAvoidanceController:invalidExactScene", ...
                    "The study requires a timestamp and supports at most one visible target.");
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

        function [matrix,bound,terminal,completion] = completionRows(model,prediction,~)
            cfg = model.cfg;
            if any(cfg.model.ltvModelErrorRateBound~=0) || any(cfg.model.plantModelResidualRateBound~=0)
                error("collisionAvoidanceController:nonexactStudyInput", ...
                    "The invariant terminal construction requires zero ego process residuals.");
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
            % law acts on the predicted lower speed endpoint, so the fixed
            % propagated radius contributes to this affine bound.
            rate = model.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
            selected = isfinite(rate);
            lastInput = zeros(2,prediction.planCount);
            lastInput(:,end-1:end) = eye(2);
            changeMap = terminal.feedback*finalMap-lastInput;
            changeOffset = terminal.input+terminal.feedback*finalOffset+terminal.radiusFeedback*finalRadius;
            matrix = [matrix;changeMap(selected,:);-changeMap(selected,:)];
            bound = [bound;rate(selected)-changeOffset(selected);rate(selected)+changeOffset(selected)];
            [exitMatrix,exitBound,completion] = hardEncounterBarrier.finiteCompletionRows( ...
                model,prediction,finalMap,finalOffset,terminal.frame);
            matrix = [matrix;exitMatrix];
            bound = [bound;exitBound];
        end

        function [stored,problem] = admit(stored,problem)
            stored.version = 22;
            stored.encounterComplete = isempty(stored.encounters);
            stored.witnessModel = problem.model;
            stored.safetyScope = "finiteConfirmedEncounterThenInvariantRoadTail";
            stored.certifiedDuration = inf;
            if ~isempty(stored.encounters)
                stored.certifiedDuration = stored.completion.deadline-stored.stateTime;
            end
            stored.metadata = localMetadata(problem.metadata,stored);
            problem.metadata = stored.metadata;
        end

        function [model,candidate,initialization,original] = validateTransition(stored,ego,model,observations,identity)
        % Check the executed first hold, condition both information sets and
        % assemble the carried witness before any fresh problem is built. The
        % stored certificate describes an accepted plan of N stages of which
        % consumedStages have already been executed; the carried witness is
        % its remaining tail closed by the terminal law.
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
            consumed = stored.consumedStages;
            stageCount = size(plan,2);
            % horizonSteps counts the optimized stages of the stored plan (zero
            % for a terminal-law certificate, whose single column is the law's
            % input); remainingSteps counts those not yet consumed.
            if consumed+1>stageCount || ~isequal(stored.appliedInput,plan(:,consumed+1)) ...
                    || stored.remainingSteps~=stored.horizonSteps-consumed
                error("collisionAvoidanceController:invalidStoredCertificate","The issued input witness changed.");
            end
            if stored.remainingSteps>0 && (~isequal( ...
                    reshape(stored.decision(stored.qp.layout.planIndex),2,[]),plan) ...
                    || ~isequaln(stored.terminal,stored.qp.terminal) ...
                    || (~isempty(stored.encounters) && ~isequaln(stored.completion,stored.qp.completion)))
                error("collisionAvoidanceController:invalidStoredCertificate", ...
                    "The carried controls or completion data differ from the verified decision.");
            end
            rebuilt = string(cfg.solver.witnessVerification)=="rebuilt";
            if rebuilt && stored.remainingSteps>0
                check = solveHardCbfClf.certify(stored.qp,stored.prediction,stored.witnessModel,stored.decision);
                if ~check.accepted || ~isequal(check.value,stored.value) ...
                        || ~isequal(reshape(stored.decision(stored.qp.layout.planIndex),2,[]),plan)
                    error("collisionAvoidanceController:invalidStoredCertificate","The previous witness failed verification.");
                end
            end
            % Ego conditioning: the true state lies in the published successor
            % box and in the measurement box, so it lies in their intersection.
            % The certificate publishes its node boxes from the current frame
            % on, so column 2 is always the successor of the executed hold.
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
            % Inclusion of the conditioned box in the published successor box
            % is what transfers the stored verification (Lemma 1); it holds
            % by construction and is checked to eps-level allowance.
            difference = model.initialEgoState-predictedCenter;
            difference(3) = atan2(sin(difference(3)),cos(difference(3)));
            inclusionMargin = min(predictedRadius-abs(difference)-model.initialFrenetErrorBound);
            allowance = 1024*eps*(1+max(abs(predictedCenter))+max(predictedRadius));
            if inclusionMargin<-allowance
                error("collisionAvoidanceController:inconsistentObservation", ...
                    "The conditioned ego box is not inside the published successor box.");
            end
            % Target conditioning against the carried bounded reachable set.
            measured = targetPrediction.admitOnline(observations,model.stateTime,model.lane,cfg);
            original = stored.originalEncounter;
            model.confirmation = stored.confirmation;
            if isempty(stored.encounters) && ~isempty(measured)
                % New obligations need fresh admission. An observed exterior
                % object can be discharged immediately using its current box.
                model.encounters = measured;
                model.confirmation = hardEncounterBarrier.admitConfirmation(ego,model);
                if hardEncounterBarrier.observedExterior(model,measured)
                    measured = measured([]);
                end
            end
            model.targetSetChanged = isempty(measured)~=isempty(stored.encounters);
            if model.targetSetChanged && ~isempty(measured)
                model.encounters = measured;
                original = measured;
                candidate = [];
                initialization = zeros(2,0);
                return;
            end
            observationContract = model.confirmation;
            if isempty(stored.encounters), observationContract = []; end
            observationValid = hardEncounterBarrier.confirmationObservation( ...
                ego,model.stateTime,observationContract, ...
                isempty(measured) && ~isempty(stored.encounters));
            if isempty(measured) && ~isempty(stored.encounters)
                hardEncounterBarrier.requirePossibleAbsence(model,stored.encounters);
            end
            if ~isempty(measured) && (measured.key~=original.key || measured.halfLength~=original.halfLength ...
                    || measured.halfWidth~=original.halfWidth)
                error("collisionAvoidanceController:changedEncounterContract", ...
                    "The same target and footprint must persist at every frame.");
            end
            model.encounters = measured;
            if ~isempty(measured)
                model.encounters = targetPrediction.condition(stored.encounters,model.sampleTime,measured);
                model.motionBoundsIncreased = any(measured.contract.jerkBound>stored.encounters.contract.jerkBound) ...
                    || measured.contract.yawAccelerationBound>stored.encounters.contract.yawAccelerationBound;
            end
            if ~isempty(stored.encounters)
                atDeadline = model.stateTime>=stored.completion.deadline ...
                    -128*eps(max(1,abs(stored.completion.deadline)));
                exterior = false;
                if ~isempty(model.encounters) && observationValid
                    if atDeadline
                        exterior = hardEncounterBarrier.observedExterior(model,model.encounters,stored.completion);
                    else
                        exterior = hardEncounterBarrier.observedExterior(model,model.encounters);
                    end
                end
                if exterior
                    model.encounters = model.encounters([]);
                end
                model.targetSetChanged = isempty(model.encounters);
                if ~isempty(model.encounters)
                    if model.motionBoundsIncreased
                        % Past motion was conditioned against the old bounds.
                        % New active obligations need fresh admission; a
                        % confirmed departure needs only the road witness.
                        model.encounters.contract = measured.contract;
                        original = model.encounters;
                        candidate = [];
                        initialization = zeros(2,0);
                        return;
                    end
                    model.exitDeadline = stored.completion.deadline;
                    if atDeadline
                        error("collisionAvoidanceController:unconfirmedEncounterExit", ...
                            "The certified exit deadline requires current confirmed departure before road-only control.");
                    end
                end
            end
            % The carried witness: the remaining tail with its own data.
            keep = stored.cellStage>=consumed+2;
            carriedValue = max(0,stored.value-sum(stored.stageViolation(1:min(end,consumed+1))));
            tailViolation = stored.stageViolation(min(consumed+2,end+1):end);
            candidate = struct("stages",stored.stages(min(consumed+2,end+1):end),"inputs",plan(:,consumed+2:end), ...
                "frames",stored.cellFrames(keep),"normals",{stored.cellNormals(keep)}, ...
                "terminal",stored.terminal,"completion",stored.completion,"carriedValue",carriedValue, ...
                "previousValue",stored.value,"previousFirstViolation",localFirst(stored.stageViolation(consumed+1:end)), ...
                "consumed",consumed,"tailViolation",tailViolation,"storedAcceptance",stored.acceptance, ...
                "terminalCenter",predictedCenter,"terminalRadius",predictedRadius, ...
                "inclusionMargin",inclusionMargin,"rebuilt",rebuilt);
            if isempty(model.encounters)
                candidate.completion.active = false;
                candidate.normals = repmat({zeros(2,0)},numel(candidate.frames),1);
            end
            initialization = zeros(2,0);
            if isfield(stored,"tailDecelerating"), model.preferEquilibriumSeed = stored.tailDecelerating; end
            if stored.remainingSteps>0
                % Performance convexification must not inherit a compulsory
                % brake from the hypothetical terminal continuation.
                initialization = [plan(:,consumed+2:end),plan(:,end)];
            end
        end

        function candidate = transferCandidate(~,candidate)
        % Carry the stored verification to the conditioned box. Every carried
        % row is a monotone function of the box (Lemma 1) and the stored plan
        % was verified on a box containing the conditioned one, so the tail
        % remains a verified plan; only the terminal membership of a
        % zero-stage tail is re-evaluated (Proposition 2).
            timer = tic;
            stageCount = numel(candidate.stages);
            if stageCount==0
                [accepted,margins] = hardEncounterBarrier.terminalMembership(candidate.terminal, ...
                    candidate.terminalCenter,candidate.terminalRadius);
                candidate.check = struct("accepted",accepted,"failedConditions",strings(1,0), ...
                    "hardRowViolation",max([0;-margins]),"clfViolation",-inf,"margin",min(margins), ...
                    "sweptClearanceMargin",min(margins),"exitMargin",min(margins), ...
                    "value",0,"stageViolation",0,"terminalMargins",margins,"violatedHardRows",zeros(0,1));
                if ~accepted, candidate.check.failedConditions = "terminalMembership"; end
                candidate.verificationMethod = "terminalInvariance";
            else
                stored = candidate.storedAcceptance;
                candidate.check = struct("accepted",true,"failedConditions",strings(1,0), ...
                    "hardRowViolation",0,"clfViolation",stored.clfViolation,"margin",stored.margin, ...
                    "sweptClearanceMargin",stored.sweptClearanceMargin,"exitMargin",stored.exitMargin, ...
                    "value",candidate.carriedValue,"stageViolation",candidate.tailViolation, ...
                    "violatedHardRows",zeros(0,1),"inclusionMargin",candidate.inclusionMargin);
                candidate.verificationMethod = "inclusionTransfer";
            end
            candidate.optimizedStages = stageCount;
            candidate.check.candidateAccepted = candidate.check.accepted;
            candidate.check.safetyCertified = candidate.check.accepted && candidate.check.value==0;
            candidate.check.accepted = candidate.check.safetyCertified;
            candidate.seconds = toc(timer);
        end

        function step = terminalStep(terminal,center,radius,sampleTime)
        % One hold of the sampled terminal law from a node box: the exact
        % held-input flow of the declared rest model, its successor box as the
        % interval hull, and the affine generator that is executed.
            % Brake the smallest possible speed. The lower endpoint obeys
            % l+ = rho*l >= 0; the upper endpoint includes passive error decay.
            input = terminal.input+terminal.feedback*center+terminal.radiusFeedback*radius;
            generator = [terminal.continuousA,terminal.continuousB,terminal.continuousC];
            exact = expm(sampleTime*[generator;zeros(3,9)]);
            step = struct("input",input,"generator",generator, ...
                "successor",exact(1:6,:)*[center;input;1], ...
                "successorRadius",abs(exact(1:6,1:6))*radius, ...
                "stage",struct("continuousA",terminal.continuousA,"continuousB",terminal.continuousB, ...
                    "continuousC",terminal.continuousC,"speed",0,"curvature",terminal.curvature, ...
                    "brakingRatio",input(2),"tireModel",terminal.tireModel));
            % Analytic nonnegativity permits clipping roundoff below zero in
            % this successor interval; the upper endpoint is only enlarged.
            step.successor(4) = max(step.successor(4),step.successorRadius(4));
        end

        function candidate = verifyCandidate(model,candidate)
        % Verify the shifted plan with its carried data on the conditioned set.
            if isempty(candidate.stages)
                % The invariant terminal law is verified by membership in
                % both modes. Expanding its stiff rest generator into a fresh
                % Taylor tube adds no premise to that analytic certificate.
                candidate = hardEncounterBarrier.transferCandidate(model,candidate);
                return;
            end
            timer = tic;
            stageCount = numel(candidate.stages);
            carried = model;
            carried.prescribedTerminal = candidate.terminal;
            carried.prescribedCompletion = candidate.completion;
            carried.exitSteps = zeros(numel(model.encounters),1);
            carried.exitMargin = inf;
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
            prediction = localPrescribeGeometry(prediction,inputs(:),candidate.frames,candidate.normals);
            candidate.verificationMethod = "carriedEnclosureRows";
            carried.anchorPlan = inputs(:);
            carried.verificationOnly = true;
            qp = formulateAvoidanceProblem(carried,prediction,inputs(:));
            carried = rmfield(carried,"verificationOnly");
            [check,decision] = solveHardCbfClf.certifyInputs(qp,prediction,carried,inputs);
            candidate.model = carried;
            candidate.prediction = prediction;
            candidate.qp = qp;
            candidate.decision = decision;
            candidate.inputs = inputs;
            check.candidateAccepted = check.accepted;
            check.safetyCertified = check.accepted && check.value==0;
            check.accepted = check.safetyCertified;
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
            % Exact ordering of the stored floating-point endpoints needs no
            % dot-product allowance. Subtraction near rest is exact (Sterbenz).
            margins(end) = center(4)-radius(4);
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

function prediction = localPrescribeGeometry(prediction,anchor,frames,normals)
% Reuse carried charts and normals; nominals come from the new tubes.
    prediction.geometryAnchor = anchor;
    prediction.geometryFrames = frames;
    nominal = cell(numel(prediction.cells),1);
    for index = 1:numel(prediction.cells)
        tube = prediction.cells(index);
        values = reshape(pagemtimes(tube.map,anchor),6,[])+tube.offset;
        nominal{index} = values;
    end
    prediction.geometryNominal = nominal;
    prediction.separationNormals = normals;
end

function terminal = localTerminalSet(model,~,anchor)
% Robust road terminal set for braking from the lower speed endpoint.
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
        errorBudget = localBudget(abs(projectedFlow)+budget*terminal.errorInputCoupling,terminal.errorComparison);
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
    % Both signs of the velocity box and its nonnegative lower speed endpoint.
    % Radius feedback makes this last row invariant: l+ = rho*l.
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
    terminal.targetNormals = zeros(2,0);
    terminal.futureTargetSupports = zeros(0,1);
    terminal.targetIndependent = true;
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
% The rest dynamics depend on the configuration, the hold and the curvature
% only; the last result is reused while those are unchanged.
    persistent memoKey memoTerminal
    key = struct("curvature",curvature,"sampleTime",model.sampleTime, ...
        "bias",model.longitudinalAccelerationBias,"cfg",rmfield(model.cfg,"solver"));
    if ~isempty(memoKey) && isequaln(key,memoKey)
        terminal = memoTerminal;
        return;
    end
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
    radiusFeedback = -feedback;
    flow = expm(h*[a,b;zeros(2,8)]);
    step = flow(1:6,1:6)+flow(1:6,7:8)*feedback;
    rho = exp(-(damping+1)*h);
    % Open-loop error comparison and its braked nominal counterpart.
    errorComparison = abs(a(4:6,4:6));
    errorComparison(1:4:end) = diag(a(4:6,4:6));
    comparison = errorComparison;
    comparison(1,1) = comparison(1,1)-brakeGain;
    errorInputCoupling = zeros(3);
    errorInputCoupling(1,1) = brakeGain;
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
        errorExcursion = localExcursion(abs(a(1:3,4:6))+excursion*errorInputCoupling,errorComparison);
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
        "input",input,"feedback",feedback,"radiusFeedback",radiusFeedback,"stepMatrix",step,"sampleTime",h, ...
        "longitudinalRatio",rho,"comparison",comparison,"errorComparison",errorComparison, ...
        "errorBudgetFinite",errorHurwitz,"poseExcursion",excursion,"errorExcursion",errorExcursion, ...
        "velocityLimit",velocityLimit,"curvature",curvature, ...
        "errorInputCoupling",errorInputCoupling,"feedbackArgument","certifiedLowerSpeedEndpoint");
    memoKey = key;
    memoTerminal = terminal;
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

function [direction,steps,passing] = localEncounterProposal(model,range)
% Propose the departure side and a useful initial horizon for a known approach.
% These are search choices; finite exit and swept rows remain the certificate.
    cfg = model.cfg;
    direction = zeros(2,0);
    steps = cfg.controller.minimumHorizonSteps;
    passing = false;
    [position,heading] = laneGeometry.fromFrenet(model.initialEgoState,model.lane);
    frame = laneGeometry.frameBounds(model.lane,model.initialEgoState(1), ...
        cfg.controller.stationTrustRadius,cfg.model.lateralDomainRadius);
    target = model.encounters;
    relative = target.center(1:2)-position;
    egoVelocity = [cos(heading),-sin(heading);sin(heading),cos(heading)]*model.initialEgoState(4:5);
    velocity = target.center(3:4)-egoVelocity;
    if norm(relative)<=range
        velocity = target.center(3:4)-frame.tangent*max(model.initialEgoState(4),model.referenceSpeed);
    end
    speed = norm(velocity);
    if speed<1e-6 || relative.'*velocity>=0, return; end
    closestTime = -relative.'*velocity/speed^2;
    miss = norm(relative+closestTime*velocity);
    [~,radius] = targetPrediction.finiteFlow(target,closestTime);
    body = hypot(cfg.vehicle.length,cfg.vehicle.width)/2+hypot(target.halfLength,target.halfWidth) ...
        +cfg.collision.clearanceMargin;
    uncertainty = norm(radius(1:2))+norm(model.initialFrenetErrorBound(1:2));
    passing = miss<=body+uncertainty;
    approachesRegion = norm(relative)>range && miss<=range+hypot(target.halfLength,target.halfWidth)+uncertainty;
    if ~passing && ~approachesRegion, return; end
    direction = velocity/speed;
    duration = (range+hypot(target.halfLength,target.halfWidth)-direction.'*relative)/speed+0.5;
    block = max(1,ceil(cfg.controller.horizonSteps/2));
    % Bound the initial allocation only. The timed search may extend it;
    % exhausting work never authorizes release or an uncertified prefix.
    steps = min(4*cfg.controller.horizonSteps, ...
        max(steps,block*ceil(duration/(model.sampleTime*block))));
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
        "originalEncounter","stages","cellStage","cellFrames","cellNormals","terminal","remainingSteps", ...
        "consumedStages","acceptance","horizonSteps","completion","confirmation", ...
        "lastAttemptSeconds","lastAttemptStages"];
    if ~isstruct(stored) || ~isscalar(stored) || ~all(isfield(stored,required)) ...
            || stored.version~=22 || ~isfield(stored.terminal,"radiusFeedback")
        error("collisionAvoidanceController:invalidStoredCertificate","Use a version-22 finite-completion certificate.");
    end
    if ~isequaln(identity,stored.identity)
        error("collisionAvoidanceController:changedExecutionContract", ...
            "The retained declared plant, road, route and physical limits must be unchanged.");
    end
end

function metadata = localMetadata(metadata,stored)
    metadata.fallbackUsed = false;
    metadata.carriedMargin = stored.margin;
    metadata.requiredMargin = stored.witnessModel.requiredMargin;
    metadata.pcbfValue = stored.value;
    metadata.stageViolation = stored.stageViolation;
    metadata.barrierValue = stored.value;
    metadata.barrierInterpretation = "verifiedAccumulatedSafetyViolation";
    metadata.horizonSteps = stored.remainingSteps;
    metadata.planningWindowSteps = stored.identity.configuration.controller.horizonSteps;
    metadata.certificateExtensionSteps = max(0,stored.horizonSteps-metadata.planningWindowSteps);
    metadata.deadline = stored.deadline;
    metadata.encounterComplete = stored.encounterComplete;
    metadata.confirmedRelease = ~isempty(metadata.dischargedTargetKeys);
    metadata.roadTailCertified = stored.terminal.targetIndependent;
    metadata.targetCertifiedUntil = NaN;
    if ~isempty(stored.encounters), metadata.targetCertifiedUntil = stored.completion.deadline; end
    metadata.confirmationContract = stored.confirmation;
    metadata.safetyScope = stored.safetyScope;
    metadata.certifiedDuration = stored.certifiedDuration;
    metadata.commandCertifiedDuration = stored.witnessModel.sampleTime;
    metadata.lookaheadDuration = stored.remainingSteps*stored.witnessModel.sampleTime;
    metadata.recursiveFeasibilityClaimed = true;
    metadata.recursiveFeasibilityScope = "admittedEncounterUntilConfirmedExit;fixedOrSmallerMotionBounds;declaredAffineStagePlant;conditionedInformationSets;currentConfirmationObservation";
    metadata.indefiniteRecursiveFeasibilityClaimed = isempty(stored.encounters);
    metadata.terminalContinuationCertified = true;
    metadata.terminalPolicyRole = "carriedWitnessTail";
    metadata.physicalVehicleGuaranteeEstablished = false;
    metadata.planCertified = stored.acceptance.safetyCertified;
    metadata.safetyCertified = stored.acceptance.safetyCertified;
    metadata.candidateAccepted = stored.acceptance.candidateAccepted;
    metadata.acceptance = stored.acceptance;
    metadata.hardRowViolation = stored.acceptance.hardRowViolation;
    metadata.activeTargetKeys = string({stored.encounters.key});
    metadata.commandActuationTime = stored.stateTime;
end

function message = localRejectMessage(result,check,qp)
% Identifier-prefixed description of a hard solver failure.
    detail = "";
    if isfield(check,"violatedHardRows") && ~isempty(check.violatedHardRows)
        names = solveHardCbfClf.rowNames(qp);
        detail = "; violated hard rows: "+strjoin(unique(names(check.violatedHardRows)).',",");
    end
    message = sprintf("collisionAvoidanceController:noCertifiedContinuation: No verified plan was obtained: %s; %s%s.", ...
        result.message,strjoin(check.failedConditions,","),detail);
end

function direction = localExitDirection(center,frame,anchor)
    direction = center(1:2)-frame.origin-[frame.tangent,frame.lateral]*anchor(1:2);
    if norm(direction)<sqrt(eps), direction = frame.lateral; end
    direction = direction/norm(direction);
end

function [row,bound] = localExitRow(model,target,center,radius,egoRadius,frame,direction)
% Directional exterior membership is a convex inner approximation of the
% complement of the range ball. Charge both boxes, chart error and the entire
% target body. Norm scaling keeps the implication valid after normalization.
    row = [direction.'*[frame.tangent,frame.lateral],zeros(1,4)];
    body = targetPrediction.rectangleSupport(target.halfLength,target.halfWidth, ...
        -direction,center(7),radius(7))*norm(direction);
    distance = model.confirmation.range+model.cfg.encounter.numericalMargin;
    bound = direction.'*(center(1:2)-frame.origin)-abs(direction).'*radius(1:2) ...
        -abs(row)*egoRadius-abs(direction).'*frame.positionErrorBound ...
        -body-distance*norm(direction);
    bound = bound-256*eps*(1+abs(bound)+abs(direction).'*(abs(center(1:2))+abs(frame.origin)));
end
