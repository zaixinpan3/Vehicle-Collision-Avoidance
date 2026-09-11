classdef hardEncounterBarrier
    %hardEncounterBarrier Exact scheduled-model MPC with an invariant tail.
    % The admitted finite schedule is followed by the zero-speed scheduled
    % bicycle and its sampled braking law. Both are part of the exact plant
    % assumption; no nonlinear Fiala model-transfer claim is made.

    methods (Static)
        function [model,prediction,qp,result,check,timing] = plan(model)
            cfg = model.cfg;
            timing = struct("predictionSeconds",0,"formulationSeconds",0, ...
                "solveSeconds",0,"verificationSeconds",0,"attempts",0);
            calls = 0;
            searchTimer = tic;
            while true
                if timing.attempts>0 && toc(searchTimer)>=cfg.solver.certificateSearchTimeLimit
                    error("collisionAvoidanceController:certificateSearchLimit", ...
                        "The search budget expired without an invariant continuation; infeasibility is not established.");
                end
                timing.attempts = timing.attempts+1;
                model.exitSteps = zeros(numel(model.encounters),1);
                model.exitMargin = inf;
                phase = tic;
                [schedule,anchor] = localApproachSchedule(model);
                model.linearizationInputs = reshape(anchor,2,[]);
                prediction = ltvBicycleModel.finitePredict(model,schedule);
                timing.predictionSeconds = timing.predictionSeconds+toc(phase);
                phase = tic;
                qp = formulateAvoidanceProblem(model,prediction,anchor);
                timing.formulationSeconds = timing.formulationSeconds+toc(phase);
                phase = tic;
                [result,qp] = solveHardCbfClf.solve(qp,cfg);
                calls = calls+result.solverCalls;
                timing.solveSeconds = timing.solveSeconds+toc(phase);
                phase = tic;
                check = solveHardCbfClf.certify(qp,prediction,model,result.decision);
                timing.verificationSeconds = timing.verificationSeconds+toc(phase);
                if result.feasible && check.accepted
                    result.solverCalls = calls;
                    return;
                end
                if ~ismember(result.exitFlag,[-2,0,-7])
                    localReject(result,check);
                end
                % Only a complete terminal certificate can authorize control.
                % Extension searches a larger admission domain; no prefix-only
                % decision is executed and no repeated admission is assumed.
                model.horizonSteps = model.horizonSteps+1;
            end
        end

        function validateAdmission(ego, observations, cfg)
            if ~isfinite(ego.stateTime) || numel(observations)~=1
                error("collisionAvoidanceController:invalidExactScene", ...
                    "The study requires a timestamp and exactly one persistent target at every frame.");
            end
            if any(ego.stateErrorBound~=0) ...
                    || any(cfg.model.ltvModelErrorRateBound~=0) ...
                    || any(cfg.model.plantModelResidualRateBound~=0)
                error("collisionAvoidanceController:nonexactStudyInput", ...
                    "The exact scheduled-model study excludes ego measurement and process errors.");
            end
            if cfg.model.speedMinimum~=0
                error("collisionAvoidanceController:invalidExactScene", ...
                    "The invariant slowing continuation requires the speed domain to include zero.");
            end
        end

        function [matrix,bound,terminal] = completionRows(model,prediction,~)
            cfg = model.cfg;
            if any(model.initialFrenetErrorBound~=0) || any(cfg.model.ltvModelErrorRateBound~=0) ...
                    || any(cfg.model.plantModelResidualRateBound~=0) ...
                    || any(arrayfun(@(target) any(target.radius~=0) ...
                        || any(target.contract.jerkBound~=0) || target.contract.yawAccelerationBound~=0,model.encounters))
                error("collisionAvoidanceController:nonexactStudyInput", ...
                    "The invariant terminal construction requires the exact scheduled-model study.");
            end
            finalMap = prediction.egoStateMatrix(:,:,end);
            finalOffset = prediction.egoStateOffset(:,end);
            finalRadius = prediction.egoStateErrorBound(:,end);
            anchor = finalOffset+finalMap*model.anchorPlan;
            frame = laneGeometry.frameBounds(model.lane,anchor(1), ...
                cfg.controller.stationTrustRadius,cfg.model.lateralDomainRadius);
            curvature = laneGeometry.curvature(anchor(1),model.lane);
            terminal = localTerminalDynamics(model,curvature);
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
                center = targetPrediction.finiteFlow(target,duration);
                displacement = frame.origin+[frame.tangent,frame.lateral]*anchor(1:2)-center(1:2);
                normal = localFutureNormal(displacement,center,frame.lateral);
                support = localFutureSupport(target.center,normal,duration);
                normalNorm = norm(normal)+64*eps*(1+norm(normal));
                if center(8)==0
                    bodySupport = targetPrediction.rectangleSupport(target.halfLength,target.halfWidth,normal,center(7),0);
                else
                    bodySupport = hypot(target.halfLength,target.halfWidth)*normalNorm;
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
            % Ap + |A| M |v| <= b is invariant for the terminal comparison
            % dynamics. Enumerate signs to obtain ordinary hard linear rows.
            signs = 2*double(dec2bin(0:7,3)-'0')-1;
            poseCount = size(poseRows,1);
            rows = [repelem(poseRows,8,1), ...
                repelem(abs(poseRows)*terminal.poseExcursion,8,1).*repmat(signs,poseCount,1)];
            limits = repelem(poseBound,8);
            % Both signs of velocity and the nonnegative longitudinal domain.
            rows = [rows;zeros(6,3),[eye(3);-eye(3)];0,0,0,-1,0,0];
            limits = [limits;terminal.velocityLimit;terminal.velocityLimit;0];
            terminal.stateRows = rows;
            terminal.stateBound = limits;
            terminal.poseRows = poseRows;
            terminal.poseBound = poseBound;
            terminal.frame = frame;
            terminal.targetNormals = normals;
            terminal.futureTargetSupports = futureSupports;
            matrix = rows*finalMap;
            bound = limits-rows*finalOffset-abs(rows)*finalRadius;
            % The first terminal input must satisfy slew relative to u_(N-1).
            rate = model.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
            selected = isfinite(rate);
            lastInput = zeros(2,prediction.planCount);
            lastInput(:,end-1:end) = eye(2);
            changeMap = terminal.feedback*finalMap-lastInput;
            changeOffset = terminal.input+terminal.feedback*finalOffset;
            uncertainty = abs(terminal.feedback)*finalRadius;
            matrix = [matrix;changeMap(selected,:);-changeMap(selected,:)];
            bound = [bound;rate(selected)-changeOffset(selected)-uncertainty(selected); ...
                rate(selected)+changeOffset(selected)-uncertainty(selected)];
        end

        function [stored,problem] = admit(stored,problem)
            stored.version = 17;
            stored.admissionTime = stored.stateTime;
            stored.consumedSteps = 0;
            stored.encounterComplete = false;
            stored.witnessModel = problem.model;
            stored.safetyScope = "indefiniteExactScheduledModel";
            stored.certifiedDuration = inf;
            stored.metadata = localMetadata(problem.metadata,stored,"checkedOptimization", ...
                problem.metadata.solverCallCount,false);
            problem.metadata = stored.metadata;
        end

        function [command,inputs,problem,stored] = advance(stored,ego,model,observations,identity,makeCommand)
            timer = tic;
            hardEncounterBarrier.validateAdmission(ego,observations,model.cfg);
            localValidateStored(stored,identity);
            h = model.sampleTime;
            consumed = stored.consumedSteps+1;
            count = stored.prediction.stageCount;
            expectedTime = stored.admissionTime+consumed*h;
            if ~isfinite(model.stateTime) || abs(model.stateTime-expectedTime)>128*eps(max(1,abs(expectedTime))) ...
                    || ~isequal(ego.heldActuatorInput,stored.appliedInput)
                error("collisionAvoidanceController:executionContractViolation", ...
                    "Continuation requires the scheduled sample and the previously issued input.");
            end
            rootPlan = reshape(stored.decision(stored.qp.layout.planIndex),2,[]);
            if stored.consumedSteps<count
                expectedInput = rootPlan(:,stored.consumedSteps+1);
                planCompatible = isequal(stored.plan,rootPlan(:,stored.consumedSteps+1:end));
            else
                expectedInput = stored.qp.terminal.input+stored.qp.terminal.feedback*stored.predictedState(:,1);
                planCompatible = isequal(stored.plan(:,1),expectedInput);
            end
            if ~isequal(stored.appliedInput,expectedInput) || ~planCompatible
                error("collisionAvoidanceController:invalidStoredCertificate","The issued input witness changed.");
            end
            check = solveHardCbfClf.certify(stored.qp,stored.prediction,stored.witnessModel,stored.decision);
            if ~check.accepted || ~isequal(check.margin,stored.margin)
                error("collisionAvoidanceController:invalidStoredCertificate","The complete witness failed verification.");
            end
            [predicted,radius] = localExpectedState(stored,rootPlan,consumed);
            allowance = 256*eps*(1+abs(predicted)+abs(model.initialEgoState));
            if any(abs(model.initialEgoState-predicted)>radius+allowance)
                error("collisionAvoidanceController:inconsistentObservation", ...
                    "The ego state contradicts the retained exact scheduled dynamics.");
            end
            measured = targetPrediction.admitExact(observations,model.stateTime,model.lane,model.cfg);
            original = stored.encounters;
            if measured.key~=original.key || measured.halfLength~=original.halfLength ...
                    || measured.halfWidth~=original.halfWidth
                error("collisionAvoidanceController:changedEncounterContract", ...
                    "The same target and footprint must persist at every frame.");
            end
            [targetCenter,targetRadius] = targetPrediction.finiteFlow(original,expectedTime-original.time);
            measured.center(7) = targetCenter(7)+atan2(sin(measured.center(7)-targetCenter(7)), ...
                cos(measured.center(7)-targetCenter(7)));
            if any(abs(measured.center-targetCenter)>targetRadius+256*eps*(1+abs(targetCenter)))
                error("collisionAvoidanceController:inconsistentObservation", ...
                    "The target must follow its original absolute-time prediction exactly.");
            end
            stored.consumedSteps = consumed;
            stored.stateTime = expectedTime;
            source = "invariantTerminalContinuation";
            attempted = false;
            calls = 0;
            if consumed<count
                qp = stored.qp;
                qp.requiredMargin = stored.margin;
                qp.inequalityBound = qp.barrier.baseBound-stored.margin*qp.barrier.scale;
                qp.stageProgram = avoidanceStageQp.updateBounds(qp);
                qp.stageProgram.fixedDecisionIndex = (1:2*consumed).';
                qp.stageProgram.fixedDecisionValue = reshape(rootPlan(:,1:consumed),[],1);
                attempted = true;
                [result,candidateQp] = solveHardCbfClf.solve(qp,model.cfg);
                calls = result.solverCalls;
                candidate = solveHardCbfClf.certify(candidateQp,stored.prediction,stored.witnessModel,result.decision);
                source = "retainedCertifiedWitness";
                if result.feasible && candidate.accepted && candidate.margin>=stored.margin
                    stored.decision = result.decision;
                    qp = candidateQp;
                    check = candidate;
                    source = "checkedContinuationOptimization";
                end
                stored.qp = qp;
                rootPlan = reshape(stored.decision(stored.qp.layout.planIndex),2,[]);
                states = reshape(pagemtimes(stored.prediction.egoStateMatrix,rootPlan(:)),6,[]) ...
                    +stored.prediction.egoStateOffset;
                stored.predictedState = states(:,consumed+1:end);
                stored.stateErrorBound = stored.prediction.egoStateErrorBound(:,consumed+1:end);
                inputs = rootPlan(:,consumed+1:end);
                command = makeCommand(rootPlan,stored.witnessModel,stored.prediction,consumed+1);
                generator = [stored.prediction.continuousA(:,:,consumed+1), ...
                    stored.prediction.continuousB(:,:,consumed+1),stored.prediction.continuousC(:,consumed+1)];
            else
                terminal = stored.qp.terminal;
                % Use the exact measured state in the terminal feedback law.
                % Open-loop replay of a rounded terminal center would not
                % preserve the invariant set indefinitely.
                value = terminal.stateRows*model.initialEgoState;
                arithmetic = 32*eps*(abs(terminal.stateRows)*abs(model.initialEgoState)+abs(terminal.stateBound));
                if any(value+arithmetic>terminal.stateBound)
                    error("collisionAvoidanceController:inconsistentObservation", ...
                        "The measured state is outside the retained invariant terminal set.");
                end
                steps = 0:model.cfg.controller.horizonSteps;
                stored.predictedState = hardEncounterBarrier.terminalFlow(terminal,model.initialEgoState,steps);
                inputs = terminal.input+terminal.feedback*stored.predictedState(:,1:end-1);
                stored.stateErrorBound = zeros(size(stored.predictedState));
                commandPrediction = struct("egoStateMatrix",zeros(6,2,1), ...
                    "egoStateOffset",stored.predictedState(:,1),"scheduleSpeedProfile",0, ...
                    "scheduleCurvature",terminal.curvature,"scheduleBrakingRatio",terminal.input(2));
                command = makeCommand(inputs(:,1),stored.witnessModel,commandPrediction,1);
                generator = [terminal.continuousA,terminal.continuousB,terminal.continuousC];
            end
            stored.remainingSteps = max(0,count-consumed);
            stored.margin = check.margin;
            stored.acceptance = check;
            stored.plan = inputs;
            stored.appliedInput = inputs(:,1);
            stored.scheduledInput = inputs(:,1);
            command.measurementTime = expectedTime;
            command.actuationTime = expectedTime;
            command.holdSeconds = h;
            metadata = localMetadata(stored.metadata,stored,source,calls,attempted);
            metadata.executedContinuousGenerator = generator;
            metadata.executedResidualRateBound = zeros(6,1);
            metadata.runtimeSeconds = toc(timer);
            metadata.runtime = struct("continuationSeconds",metadata.runtimeSeconds);
            stored.metadata = metadata;
            model.encounters = stored.encounters;
            problem = struct("problemClass",stored.qp.problemClass,"qp",stored.qp, ...
                "layout",stored.qp.layout,"prediction",stored.prediction,"model",model, ...
                "decision",stored.decision,"plan",rootPlan(:),"inputPlan",inputs, ...
                "tailPlan",inputs(:,[]),"metadata",metadata);
        end

        function states = terminalFlow(terminal,initial,steps)
        % Absolute-time evaluation avoids repeated shifting of the certificate.
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
    [a,b,c] = ltvBicycleModel.continuousMatrices(curvature,0,cfg,input(2),model.longitudinalAccelerationBias);
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
    comparison = abs(a(4:6,4:6));
    comparison(1:4:end) = diag(a(4:6,4:6));
    comparison(1,1) = comparison(1,1)-brakeGain;
    direction = (-comparison)\ones(3,1);
    directionReserve = 128*eps*(abs(comparison)*abs(direction));
    if any(direction<=0) || any(comparison*direction+directionReserve>=0) || ~(rho>0 && rho<1)
        error("collisionAvoidanceController:invalidTerminalModel","No contracting terminal velocity comparison exists.");
    end
    excursion = abs(a(1:3,4:6))/(-comparison);
    reserve = 4096*eps*(1+norm(excursion,inf));
    excursion = excursion+reserve*ones(3,1)*(ones(1,3)/(-comparison));
    residual = excursion*comparison+abs(a(1:3,4:6));
    residualReserve = 128*eps*(abs(excursion)*abs(comparison)+abs(a(1:3,4:6)));
    if any(residual+residualReserve>0,"all")
        error("collisionAvoidanceController:invalidTerminalModel","The excursion comparison failed numerical verification.");
    end
    caps = [cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
    caps(1) = min(caps(1),(input(2)-cfg.actuation.brakingRatioMinimum)*gain/brakeGain);
    rate = h*cfg.model.brakingRatioRateMaximum;
    if isfinite(rate), caps(1) = min(caps(1),rate*gain/(brakeGain*(1-rho))); end
    slip = cfg.model.slipAngleMaximum(:);
    if isscalar(slip), slip = repmat(slip,2,1); end
    slipDirection = [direction(2)+cfg.vehicle.lf*direction(3); ...
        direction(2)+cfg.vehicle.lr*direction(3)]/cfg.model.scheduleSpeedFloor;
    scale = min([caps./direction;slip./slipDirection]);
    if ~(scale>0) || input(2)>cfg.actuation.brakingRatioMaximum || input(2)<cfg.actuation.brakingRatioMinimum
        error("collisionAvoidanceController:invalidTerminalModel","The terminal input and velocity domain are empty.");
    end
    terminal = struct("continuousA",a,"continuousB",b,"continuousC",c, ...
        "input",input,"feedback",feedback,"stepMatrix",step,"sampleTime",h, ...
        "longitudinalRatio",rho,"comparison",comparison,"poseExcursion",excursion, ...
        "velocityLimit",0.99*scale*direction,"curvature",curvature);
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

function support = localFutureSupport(center,normal,duration)
% Upper scalar coefficients enclose the exact absolute-time polynomial.
% A derivative sign is never inferred by rounding a small positive value to zero.
    acceleration = 0.5*dot(normal,center(5:6));
    initialVelocity = dot(normal,center(3:4));
    initialPosition = dot(normal,center(1:2));
    velocity = initialVelocity+2*acceleration*duration;
    support = initialPosition+initialVelocity*duration+acceleration*duration^2;
    errorAcceleration = 128*eps*sum(abs(normal.*center(5:6)))/2;
    errorVelocity = 128*eps*sum(abs(normal.*center(3:4)))+2*duration*errorAcceleration;
    errorPosition = 128*eps*sum(abs(normal.*center(1:2))) ...
        +duration*errorVelocity+duration^2*errorAcceleration;
    velocity = velocity+errorVelocity+128*eps*(abs(initialVelocity)+2*abs(acceleration)*duration);
    acceleration = acceleration+errorAcceleration;
    support = support+errorPosition+128*eps*(abs(initialPosition) ...
        +abs(initialVelocity)*duration+abs(acceleration)*duration^2);
    if acceleration>0 || (acceleration==0 && velocity>0)
        support = inf;
    elseif acceleration<0 && velocity>0
        support = support-velocity^2/(4*acceleration);
    end
    support = support+256*eps*(1+abs(support)+norm(center(1:2)));
end

function [schedule,anchor] = localApproachSchedule(model)
    cfg = model.cfg;
    count = model.horizonSteps;
    speed = linspace(model.initialEgoState(4),min(0.25,model.initialEgoState(4)),count+1);
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
        inputs(:,stage) = min(max(input,max(lower,prior-change)),min(upper,prior+change));
        prior = inputs(:,stage);
    end
    schedule = struct("speedProfile",speed,"station",station,"curvature",curvature,"brakingRatio",inputs(2,:));
    anchor = inputs(:);
end

function localValidateStored(stored,identity)
    required = ["version","admissionTime","consumedSteps","witnessModel","qp", ...
        "decision","prediction","metadata","identity","encounters"];
    if ~isstruct(stored) || ~isscalar(stored) || ~all(isfield(stored,required)) ...
            || stored.version~=17 || ~isfield(stored.qp,"terminal")
        error("collisionAvoidanceController:invalidStoredCertificate","Use a version-17 exact-model invariant certificate.");
    end
    if ~isequaln(identity,stored.identity)
        error("collisionAvoidanceController:changedExecutionContract", ...
            "The retained exact plant, road, route and physical limits must be unchanged.");
    end
    validateattributes(stored.consumedSteps,{'double'},{'scalar','integer','nonnegative'});
end

function [state,radius] = localExpectedState(stored,inputs,consumed)
    count = stored.prediction.stageCount;
    if consumed<=count
        state = stored.prediction.egoStateMatrix(:,:,consumed+1)*inputs(:)+stored.prediction.egoStateOffset(:,consumed+1);
        radius = stored.prediction.egoStateErrorBound(:,consumed+1);
    else
        state = stored.predictedState(:,2);
        radius = stored.stateErrorBound(:,2);
    end
end

function metadata = localMetadata(metadata,stored,source,calls,attempted)
    metadata.certificateSource = source;
    metadata.solverCallCount = calls;
    metadata.fallbackUsed = attempted && source=="retainedCertifiedWitness";
    metadata.carriedWitnessFeasible = stored.consumedSteps>0;
    metadata.certificateCompatible = stored.consumedSteps>0;
    metadata.carriedMargin = stored.margin;
    metadata.requiredMargin = stored.qp.requiredMargin;
    metadata.barrierValue = -stored.margin;
    metadata.barrierInterpretation = "storedWitnessLowerBound";
    metadata.horizonSteps = stored.remainingSteps;
    metadata.planningWindowSteps = stored.identity.configuration.controller.horizonSteps;
    metadata.certificateExtensionSteps = max(0,stored.prediction.stageCount-metadata.planningWindowSteps);
    metadata.deadline = stored.deadline;
    metadata.consumedSteps = stored.consumedSteps;
    metadata.encounterComplete = false;
    metadata.safetyScope = stored.safetyScope;
    metadata.certifiedDuration = inf;
    metadata.lookaheadDuration = stored.remainingSteps*stored.witnessModel.sampleTime;
    metadata.recursiveFeasibilityClaimed = true;
    metadata.recursiveFeasibilityScope = "indefiniteForExactRetainedScheduleAndInvariantTail";
    metadata.indefiniteRecursiveFeasibilityClaimed = true;
    metadata.terminalContinuationCertified = true;
    metadata.terminalActive = stored.consumedSteps>=stored.prediction.stageCount;
    metadata.exactPredictionAssumptionsHold = true;
    metadata.physicalVehicleGuaranteeEstablished = false;
    metadata.jointAdmissionPerformed = false;
    metadata.newlyAdmittedTargetKeys = strings(1,0);
    metadata.planCertified = true;
    metadata.acceptance = stored.acceptance;
    metadata.hardRowViolation = stored.acceptance.hardRowViolation;
    metadata.clfRelaxation = stored.decision(stored.qp.layout.relaxationIndex(min(stored.consumedSteps+1,numel(stored.qp.layout.relaxationIndex)+1):end));
    metadata.activeTargetKeys = string({stored.encounters.key});
    metadata.dischargedTargetKeys = strings(1,0);
    metadata.commandActuationTime = stored.stateTime;
end

function localReject(result,check)
    error("collisionAvoidanceController:noCertifiedContinuation", ...
        "No invariant continuation was certified: %s; %s.",result.message,strjoin(check.failedConditions,","));
end
