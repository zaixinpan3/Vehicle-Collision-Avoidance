classdef hardEncounterBarrier
    %hardEncounterBarrier Finite encounter completion and predictive continuation.
    % The terminal law is a prediction-side witness. It never issues a command.
    methods (Static)
        function [model,carry] = prepare(model,ego,observations,stored,identity)
            cfg = model.cfg;
            carry = [];
            model.encounters = struct("key",{});
            model.confirmation = [];
            model.carriedWitness = [];
            model.exitMargin = inf;
            model.exitSteps = zeros(0,1);
            model.dischargedTargetKeys = strings(1,0);
            if ~isempty(stored)
                if ~isstruct(stored) || ~isfield(stored,'version') || stored.version~=26
                    error('collisionAvoidanceController:invalidControllerState','Reset incompatible controller state.');
                end
                if ~isequal(stored.plan(:),stored.decision(stored.program.layout.planIndex)) ...
                        || ~isequal(stored.appliedInput,stored.plan(:,1)) ...
                        || ~isequaln(stored.terminal,stored.program.terminal)
                    error('collisionAvoidanceController:invalidStoredCertificate','The stored accepted plan was modified.');
                end
                if ~isequaln(identity,stored.identity)
                    error('collisionAvoidanceController:changedExecutionContract','The model, road and physical limits changed.');
                end
                if abs(model.stateTime-stored.stateTime-model.sampleTime)>1e-10 ...
                        || ~isequal(ego.heldActuatorInput,stored.appliedInput)
                    error('collisionAvoidanceController:executionContractViolation','The next timestamp and issued held input are required.');
                end
                [model.initialEgoState,model.initialFrenetErrorBound] = localConditionBox( ...
                    stored.predictedState(:,2),stored.stateErrorBound(:,2), ...
                    model.initialEgoState,model.initialFrenetErrorBound);
                model.previousInput = stored.appliedInput;
                model.confirmation = stored.confirmation;
            end
            measured = cell(1,numel(observations));
            for index = 1:numel(observations)
                measured{index} = targetPrediction.admitOnline(observations(index),model.stateTime,model.lane,cfg);
            end
            if isempty(measured),measured=struct("key",{});else,measured=[measured{:}];end
            oldTargets = struct("key",{});
            if ~isempty(stored),oldTargets=stored.encounters;end
            needsObservation = ~isempty(measured) || ~isempty(oldTargets);
            valid = hardEncounterBarrier.confirmationObservation(ego,model.stateTime,model.confirmation,needsObservation);
            if needsObservation && isempty(model.confirmation)
                model.confirmation = struct('range',ego.perception.range,'exitDirection',[], ...
                    'reference',"egoReferencePoint",'exitGeometry',"entireTargetFootprint", ...
                    'confirmationDelay',0,'observationContract',"currentCompleteObservationAtConfirmationSample");
                body = hypot(cfg.vehicle.length,cfg.vehicle.width)/2;
                for index = 1:numel(measured)
                    if ego.perception.range<=body+hypot(measured(index).halfLength,measured(index).halfWidth)+cfg.collision.clearanceMargin
                        error('collisionAvoidanceController:invalidConfirmationRegion','The perception range must exceed both footprints plus clearance.');
                    end
                end
            end
            changed = false;
            active = cell(1,numel(measured));
            for index = 1:numel(measured)
                target = measured(index);
                match = find(string({oldTargets.key})==target.key,1);
                if ~isempty(match)
                    prior = oldTargets(match);
                    if target.halfLength~=prior.halfLength || target.halfWidth~=prior.halfWidth
                        error('collisionAvoidanceController:changedEncounterContract','The carried target footprint changed.');
                    end
                    conditioned = targetPrediction.condition(prior,model.sampleTime,target);
                    increased = any(target.contract.jerkBound>prior.contract.jerkBound) ...
                        || target.contract.yawAccelerationBound>prior.contract.yawAccelerationBound;
                    conditioned.contract = target.contract;
                    target = conditioned;
                    changed = changed || increased;
                end
                if ~isempty(match) && stored.completion.active ...
                        && model.stateTime>=stored.completion.deadline-1e-10
                    completion = stored.completion;
                    completion.direction = completion.direction(:,match);
                    outside = valid && hardEncounterBarrier.observedExterior(model,target,completion);
                else
                    outside = valid && hardEncounterBarrier.observedExterior(model,target);
                end
                if outside
                    if ~isempty(match),model.dischargedTargetKeys(end+1)=target.key;end
                    continue;
                end
                changed = changed || isempty(match);
                active{index} = target;
            end
            for index = 1:numel(oldTargets)
                if ~any(string({measured.key})==oldTargets(index).key)
                    hardEncounterBarrier.requirePossibleAbsence(model,oldTargets(index));
                    model.dischargedTargetKeys(end+1)=oldTargets(index).key;
                end
            end
            model.encounters = [active{:}];
            if isempty(model.encounters),model.encounters=struct("key",{});end
            model.exitSteps = zeros(numel(model.encounters),1);
            if ~isempty(stored) && ~changed
                % Preserve the actual accepted generators, enclosures, charts,
                % normals and terminal set. No relinearization is a proof step.
                keep = [stored.prediction.cells.stage]>=2;
                carry = struct('inputs',stored.plan(:,2:end),'stages',stored.stages(2:end), ...
                    'cells',stored.prediction.cells(keep),'frames',stored.cellFrames(keep), ...
                    'normals',{stored.cellNormals(keep)},'terminal',stored.terminal, ...
                    'completion',stored.completion,'remainingSteps',size(stored.plan,2)-1, ...
                    'sourceTime',stored.stateTime,'feasibleByInclusion',true, ...
                    'initialCenter',model.initialEgoState,'initialRadius',model.initialFrenetErrorBound, ...
                    'program',stored.program,'prediction',stored.prediction, ...
                    'issuedInput',stored.appliedInput,'predictedCenter',stored.predictedState(:,2));
                if ~isempty(model.encounters) && stored.completion.active
                    model.exitDeadline = stored.completion.deadline;
                    if model.stateTime>=model.exitDeadline-1e-10
                        error('collisionAvoidanceController:unconfirmedEncounterExit','The finite deadline needs current confirmed release.');
                    end
                end
                if isempty(model.encounters),carry.completion.active=false;end
            end
            model.horizonSteps = max(cfg.controller.horizonSteps,cfg.controller.minimumHorizonSteps);
            model.passingRequired = false;
            for index = 1:numel(model.encounters)
                trial = model;trial.encounters=model.encounters(index);
                [~,steps,passing] = localEncounterProposal(trial,model.confirmation.range);
                model.horizonSteps = max(model.horizonSteps,steps);
                model.passingRequired = model.passingRequired || passing;
            end
            if isfield(model,'exitDeadline')
                model.horizonSteps = min(model.horizonSteps,round((model.exitDeadline-model.stateTime)/model.sampleTime));
            end
            if ~isempty(carry) && ~isempty(model.encounters) && carry.remainingSteps>0 ...
                    && isempty(model.dischargedTargetKeys)
                % Choose the inherited feasible family before the one solve.
                % This is a predictive optimization, never fallback execution.
                model.carriedWitness = carry;
                model.horizonSteps = carry.remainingSteps;
            end
            if ~isempty(stored)
                model.initializationPlan = [stored.plan(:,2:end),stored.plan(:,end)];
            end
        end

        function [prediction,anchor] = predict(model,cruise)
            count = model.horizonSteps;
            inputs = repmat(cruise.input,1,count);
            if isfield(model,'initializationPlan')
                retained = min(count,size(model.initializationPlan,2));
                inputs(:,1:retained)=model.initializationPlan(:,1:retained);
            end
            prescribed = model;
            prescribed.prescribedStages = repmat(cruise.stage,count,1);
            anchor = inputs(:);
            prediction = ltvBicycleModel.finitePredict(prescribed,[]);
            if model.passingRequired
                normals = cell(numel(prediction.cells),numel(model.encounters));
                side = 1;
                if model.initialEgoState(2)<-0.1,side=-1;end
                projection=laneGeometry.project(model.encounters(1).center(1:2),model.lane);
                if projection.lateralPosition>model.initialEgoState(2)+0.1,side=-1;end
                for index = 1:numel(model.encounters)
                    trial = model;trial.encounters=model.encounters(index);
                    normals(:,index)=avoidanceSafetyGeometry.passingNormals(trial,prediction,anchor,side);
                end
                prediction.separationNormals = cell(numel(prediction.cells),1);
                for index = 1:numel(prediction.cells)
                    prediction.separationNormals{index}=[normals{index,:}];
                end
            end
        end

        function [matrix,bound,completion] = finiteCompletionRows(model,prediction,finalMap,finalOffset,frame)
            count = numel(model.encounters);
            matrix = zeros(count,prediction.planCount);bound=zeros(count,1);
            directions=zeros(2,count);rows=zeros(count,6);limits=zeros(count,1);
            duration=prediction.stageCount*model.sampleTime;
            anchor=finalOffset+finalMap*model.anchorPlan;
            for index=1:count
                target=model.encounters(index);
                [center,radius]=targetPrediction.finiteFlow(target,duration);
                direction=localExitDirection(center,frame,anchor);
                trial=model;trial.encounters=target;
                [proposed,~,~]=localEncounterProposal(trial,model.confirmation.range);
                if ~isempty(proposed),direction=proposed;end
                [row,limit]=localExitRow(model,target,center,radius,prediction.initialErrorBound(:,end),frame,direction);
                matrix(index,:)=row*finalMap;bound(index)=limit-row*finalOffset;
                directions(:,index)=direction;rows(index,:)=row;limits(index)=limit;
            end
            completion=struct('active',count>0,'deadline',model.stateTime+duration, ...
                'direction',directions,'frame',frame,'stateRow',rows,'stateBound',limits);
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

        function step = terminalStep(terminal,center,radius,sampleTime)
        % One hold of the sampled terminal law from a node box: the exact
        % held-input flow of the declared rest model, its successor box as the
        % interval hull, and the hypothetical terminal affine generator.
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

function terminal = localTerminalSet(model,~,anchor)
% Robust road terminal set for braking from the lower speed endpoint.
    cfg = model.cfg;
    curvature = laneGeometry.curvature(anchor(1),model.lane);
    terminal = localTerminalDynamics(model,curvature);
    % Include the certified stopping excursion of nominal plus error in the
    % chart. A local linearization radius is not a required stopping position.
    excursion = max(terminal.poseExcursion(1,:),terminal.errorExcursion(1,:));
    % Chart size is a proposal; the invariant pose rows themselves enforce
    % its boundaries. Using the entire velocity domain here can demand road
    % data far behind the car even for a modest entry speed.
    terminalRadius = cfg.controller.stationTrustRadius+excursion*min( ...
        terminal.velocityLimit,abs(anchor(4:6))+model.initialFrenetErrorBound(4:6));
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
    % A fixed one-per-second brake unnecessarily excludes ordinary cruise
    % speeds when the acceleration gain is small. Reduce the gain so the
    % same invariant law covers the configured speed domain. The excursion
    % budget below grows with the longer stop, preserving road coverage.
    brakeGain = min(brakeGain,.98*(input(2)-cfg.actuation.brakingRatioMinimum)*gain/cfg.model.speedMaximum);
    feedback = zeros(2,6);
    feedback(2,4) = -brakeGain/gain;
    radiusFeedback = -feedback;
    flow = expm(h*[a,b;zeros(2,8)]);
    step = flow(1:6,1:6)+flow(1:6,7:8)*feedback;
    rho = exp(-damping*h)-brakeGain*phi;
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
