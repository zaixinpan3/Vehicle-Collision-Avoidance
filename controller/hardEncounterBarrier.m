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
            model.nominalSource = "cruiseInitialization";
            model.measurementContractChanged=false;
            model.measurementRadiusLimit = model.initialFrenetErrorBound;
            if isfield(model.lane,'referenceCurve') && ~laneGeometry.isVaryingReference(model.lane) ...
                    && model.lane.referenceCurve.curvature~=0
                curvature=abs(model.lane.referenceCurve.curvature);
                distance=norm(ego.stateErrorBound(1:2));
                % Declare a Frenet sensing contract from the current Cartesian
                % measurement box, without imposing a lateral state domain.
                radial=abs(1/model.lane.referenceCurve.curvature-model.initialEgoState(2))-2*distance;
                if distance>=radial
                    error('collisionAvoidanceController:invalidUncertaintyChart','The sensing bound crosses the reference center.');
                end
                angle=asin(distance/radial);
                model.measurementRadiusLimit(1:3)=[angle/curvature;distance;ego.stateErrorBound(3)+angle];
            end
            model.cruiseCertificate = [];
            model.exitMargin = inf;
            model.exitSteps = zeros(0,1);
            model.dischargedTargetKeys = strings(1,0);
            if ~isempty(stored)
                expectedVersion=41;
                if ~isstruct(stored) || ~isfield(stored,'version') || stored.version~=expectedVersion
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
                measuredLimit=model.measurementRadiusLimit;
                model.measurementContractChanged=any(model.initialFrenetErrorBound>stored.terminal.measurementRadiusLimit+1e-12);
                model.measurementRadiusLimit=stored.terminal.measurementRadiusLimit;
                if model.measurementContractChanged
                    model.measurementRadiusLimit=max(measuredLimit,model.measurementRadiusLimit);
                end
                if laneGeometry.isVaryingReference(model.lane)
                    model.cruiseCertificate = ltvBicycleModel.sampledCruise(model);
                else
                    model.cruiseCertificate = stored.program.cruiseCertificate;
                    if ~model.measurementContractChanged,model.permanentTerminal = stored.terminal;end
                end
                % Enlarged bounds require a NEW complete certificate. The
                % old prediction still conditions the actual successor, but
                % does not prove the changed future sensing contract feasible.
                if isfield(model.lane,'referenceCurve') && ~laneGeometry.isVaryingReference(model.lane) ...
                        && model.lane.referenceCurve.curvature~=0
                    period = 2*pi/abs(model.lane.referenceCurve.curvature);
                    model.initialEgoState(1) = model.initialEgoState(1)+period*round( ...
                        (stored.predictedState(1,2)-model.initialEgoState(1))/period);
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
                    if ego.perception.range<=body+hypot(measured(index).halfLength,measured(index).halfWidth)
                        error('collisionAvoidanceController:invalidConfirmationRegion','The perception range must exceed the sum of both footprint circumradii.');
                    end
                end
            end
            changed = model.measurementContractChanged;
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
                    completion.direction = completion.direction(:,completion.keys==target.key);
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
            % Departure proposals of this frame, reused by the completion rows.
            model.encounterProposals = cell(1,numel(model.encounters));
            for index = 1:numel(model.encounters)
                trial = model;trial.encounters=model.encounters(index);
                [direction,steps,~] = localEncounterProposal(trial,model.confirmation.range);
                model.encounterProposals{index} = direction;
                model.horizonSteps = max(model.horizonSteps,steps);
            end
            if isfield(model,'exitDeadline')
                model.horizonSteps = min(model.horizonSteps,round((model.exitDeadline-model.stateTime)/model.sampleTime));
            end
            if ~isempty(carry)
                % Choose the inherited feasible family before the one solve.
                % This is a predictive optimization, never fallback execution.
                model.carriedWitness = carry;
                model.horizonSteps = carry.remainingSteps;
            end
            if ~isempty(stored)
                model.initializationPlan = [stored.plan(:,2:end),stored.plan(:,end)];
                model.nominalSource = "shiftedPreviousSolution";
            elseif isempty(model.encounters)
                [model.horizonSteps,model.initializationPlan]=localCruiseAdmission(model);
            end
        end

        function [prediction,anchor] = predict(model,cruise)
            count = model.horizonSteps;
            inputs = repmat(cruise.input,1,count);
            prescribed = model;
            prescribed.prescribedStages = repmat(cruise.stage,count,1);
            scheduled=isfield(cruise,'scheduled') && cruise.scheduled;
            if scheduled
                referenceStates=zeros(6,count+1);referenceInputs=zeros(2,count);
                referenceMatrices=zeros(5,5,count+1);
                for index=1:count+1
                    reference=ltvBicycleModel.referenceAt(model,cruise.index+index-1);
                    referenceStates(:,index)=reference.state;
                    referenceMatrices(:,:,index)=reference.matrix;
                    if index<=count
                        prescribed.prescribedStages(index)=reference.stage;
                        inputs(:,index)=reference.input;
                        referenceInputs(:,index)=reference.input;
                    end
                end
            end
            if isfield(model,'initializationPlan')
                retained = min(count,size(model.initializationPlan,2));
                inputs(:,1:retained)=model.initializationPlan(:,1:retained);
            end
            anchor = inputs(:);
            prediction = ltvBicycleModel.finitePredict(prescribed,[]);
            if scheduled
                prediction.referenceStates=referenceStates;
                prediction.referenceInputs=referenceInputs;
                prediction.referenceMatrices=referenceMatrices;
                prediction.referencePhaseIndex=cruise.index;
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
                if isfield(frame,'positionMap')
                    position=laneGeometry.fromFrenet(anchor,model.lane);
                    relative=center(1:2)-position;
                    if norm(relative)>sqrt(eps),direction=relative/norm(relative);end
                end
                if isfield(model,'encounterProposals') && numel(model.encounterProposals)==count
                    proposed=model.encounterProposals{index};
                else
                    trial=model;trial.encounters=target;
                    [proposed,~,~]=localEncounterProposal(trial,model.confirmation.range);
                end
                if ~isempty(proposed) && ~isfield(frame,'positionMap')
                    direction=proposed;
                end
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
            if isfield(model.lane,'referenceCurve') && (laneGeometry.isVaryingReference(model.lane) ...
                    || model.lane.referenceCurve.curvature~=0)
                frame=laneGeometry.localPoseFrame(model.lane.referenceCurve,z(1:3),rho(1:3));
            else
                frame = laneGeometry.frameBounds(model.lane,z(1), ...
                    max(model.cfg.controller.stationTrustRadius,rho(1)),abs(z(2))+rho(2));
            end
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
            if isfield(frame,'domainCenter') && any(abs(z(1:3)-frame.domainCenter)+rho(1:3)>frame.domainRadius)
                outside=false;return;
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
                max(model.cfg.controller.stationTrustRadius,rho(1)),abs(z(2))+rho(2));
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

        function [matrix,bound,terminal,completion,cone] = completionRows(model,prediction,geometry)
        % The terminal modal set and its sampled feedback use the ONLINE generator.
            terminalModel=model;
            if isfield(prediction,'referencePhaseIndex')
                terminalModel.referencePhaseIndex=prediction.referencePhaseIndex+prediction.stageCount;
                if isfield(model,'terminalOptimization') && model.terminalOptimization
                    terminalModel.referencePhaseIndex=prediction.referencePhaseIndex;
                end
            end
            terminal = localTerminalSet(terminalModel);
            [finalMap,finalOffset] = localExactFinalMap(model,prediction);
            radius = prediction.initialErrorBound(:,end);
            modal=terminal.modalMatrix;
            stateIndex=terminal.stateIndex;modeCount=size(modal,1);
            cone.matrix=zeros(3*modeCount,prediction.planCount);cone.bound=zeros(3*modeCount,1);
            cone.sizes=3*ones(modeCount,1);
            for mode=1:modeCount
                rows=3*mode-2:3*mode;
                mapped=modal(mode,:)*finalMap(stateIndex,:);
                offset=modal(mode,:)*(finalOffset(stateIndex)-terminal.reference(stateIndex));
                cone.matrix(rows,:)=[zeros(1,prediction.planCount);-real(mapped);-imag(mapped)];
                cone.bound(rows)=[terminal.radius(mode)-terminal.reserve-abs(modal(mode,:))*radius(stateIndex); ...
                    real(offset);imag(offset)];
            end
            % Any conditioned terminal center lies in the certified modal set.
            % Its first feedback input must also honor the preceding input.
            last = zeros(2,prediction.planCount);last(:,end-1:end)=eye(2);
            deltaMap = terminal.feedback*finalMap-last;
            deltaOffset = terminal.input+terminal.feedback*(finalOffset-terminal.reference);
            rate = model.sampleTime*[model.cfg.model.frontWheelSteeringRateMaximum; ...
                model.cfg.model.brakingRatioRateMaximum];
            selected = isfinite(rate);
            support = abs(terminal.feedback)*radius;
            matrix = [deltaMap(selected,:);-deltaMap(selected,:)];
            bound = [rate(selected)-support(selected)-deltaOffset(selected); ...
                rate(selected)-support(selected)+deltaOffset(selected)];
            reach = repmat([model.cfg.model.frontWheelSteeringAngleMaximum; ...
                max(abs([model.cfg.actuation.brakingRatioMinimum,model.cfg.actuation.brakingRatioMaximum]))],prediction.stageCount,1);
            extent = abs(finalMap)*reach+radius;
            frame = laneGeometry.frameBounds(model.lane,finalOffset(1), ...
                extent(1),abs(finalOffset(2))+extent(2));
            domainCount=0;
            if ~isempty(model.encounters) && isfield(geometry.frames,'positionMap')
                frame=geometry.frames(end);
                directions=[eye(3);-eye(3)];
                domainMatrix=directions*finalMap(1:3,:);
                domainBound=[frame.domainRadius;frame.domainRadius] ...
                    +directions*(frame.domainCenter-finalOffset(1:3))-abs(directions)*radius(1:3);
                matrix=[matrix;domainMatrix];bound=[bound;domainBound];domainCount=6;
            end
            [exitMatrix,exitBound,completion] = hardEncounterBarrier.finiteCompletionRows( ...
                model,prediction,finalMap,finalOffset,frame);
            completion.keys = string({model.encounters.key});
            completion.domainRowCount=domainCount;
            matrix = [matrix;exitMatrix];bound=[bound;exitBound];
        end

        function terminal = terminalCertificate(model)
            terminal = localTerminalSet(model);
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
        % A hypothetical held feedback step, never an actuator fallback.
        % successorRadius is PRIOR to the next bounded measurement.
            input = terminal.input+terminal.feedback*(center-terminal.reference);
            generator = [terminal.continuousA,terminal.continuousB,terminal.continuousC];
            exact = expm(sampleTime*[generator;zeros(3,9)]);
            step = struct('input',input,'generator',generator, ...
                'successor',exact(1:6,:)*[center;input;1], ...
                'successorRadius',abs(exact(1:6,1:6))*radius,'stage',terminal.cruise.stage);
            if isfield(terminal,'scheduled') && terminal.scheduled
                nextModel=terminal.scheduleModel;
                nextModel.referencePhaseIndex=terminal.index+1;
                step.nextTerminal=localTerminalSet(nextModel);
            end
        end

        function states = terminalFlow(terminal,initial,steps)
        % Exact-state sampled terminal feedback for independent validation.
            validateattributes(steps,{'double'},{'real','finite','nonnegative','integer'});
            states = zeros(6,numel(steps));
            for index = 1:numel(steps)
                x=initial;
                currentTerminal=terminal;
                for stage=1:steps(index)
                    next=hardEncounterBarrier.terminalStep(currentTerminal,x,zeros(6,1),terminal.sampleTime);
                    x=next.successor;
                    if isfield(next,'nextTerminal'),currentTerminal=next.nextTerminal;end
                end
                states(:,index)=x;
            end
        end

        function [accepted,margins] = terminalMembership(terminal,center,radius)
        % Prior box inclusion in the terminal modal set, before conditioning.
            selected=terminal.stateIndex;
            margins = terminal.radius-abs(terminal.modalMatrix*(center(selected)-terminal.reference(selected))) ...
                -abs(terminal.modalMatrix)*radius(selected);
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

function terminal = localTerminalSet(model)
% An invariant information-state cruise set for the admitted affine plant.
% The sensor contract bounds EVERY future posterior measurement box. No
% favorable future reset is used anywhere in the finite open-loop witness.
    if laneGeometry.isVaryingReference(model.lane)
        terminal=localScheduledTerminalSet(model);
        return;
    end
    if isfield(model,'permanentTerminal'),terminal=model.permanentTerminal;return;end
    cfg=model.cfg;
    if ~isempty(model.cruiseCertificate)
        cruise=model.cruiseCertificate;
    else
        cruise=ltvBicycleModel.sampledCruise(model);
    end
    if ~isfield(model.lane,'referenceCurve') && ( ...
            any(abs(model.lane.tangent-model.lane.tangent(1,:))>1e-12,'all') ...
            || any(abs(model.lane.segmentCurvature)>1e-12))
        error('collisionAvoidanceController:unsupportedReferenceJump', ...
            'The recursive certificate requires a continuous straight or analytic constant-curvature reference.');
    end
    if ~isempty(model.road.boundaries)
        error('collisionAvoidanceController:optimizationFailed', ...
            'The recursive cruise certificate requires an unbounded road-free reference domain.');
    end
    persistent savedKey saved
    key={cruise,model.measurementRadiusLimit,rmfield(cfg,'solver'),cfg.solver.constraintTolerance};
    if ~isempty(savedKey) && isequaln(key,savedKey),terminal=saved;return;end
    gain=cruise.gain;
    [basis,eigenvalues]=eig(cruise.closedLoop);
    modal=basis\eye(5);contraction=abs(diag(eigenvalues));
    if rcond(basis)<1e-10 || any(contraction>=1)
        error('collisionAvoidanceController:invalidTerminalModel','No well-conditioned stable terminal modal basis exists.');
    end
    ref=cruise.state(2:6);trim=cruise.input;
    sampledDrift=cruise.transition(2:6,:)*[cruise.state;trim;1]-ref;
    cap=model.measurementRadiusLimit(2:6);
    inputLimit=[cfg.model.frontWheelSteeringAngleMaximum; ...
        max(abs([cfg.actuation.brakingRatioMinimum,cfg.actuation.brakingRatioMaximum]))];
    % The held terminal input is constant throughout the complete sample.
    % Only actuator amplitude/slew restricts the invariant modal certificate.
    inputRows=[eye(2);-eye(2)];
    rows=[zeros(4,5),inputRows];
    holdBound=[inputLimit(1);cfg.actuation.brakingRatioMaximum; ...
        inputLimit(1);-cfg.actuation.brakingRatioMinimum]-inputRows*trim;
    reserve=16*max(cfg.encounter.numericalMargin,cfg.solver.constraintTolerance);
    holdBound=holdBound-reserve*(1+abs(holdBound));
    feedbackRows=rows*[eye(5);-gain];
    support=abs(feedbackRows*basis);
    noise=abs(rows(:,6:7)*gain)*cap;
    rate=model.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    finite=isfinite(rate);
    change=eye(2)+gain*cruise.transition(2:6,7:8);
    slewSupport=abs(gain*(eye(5)-cruise.closedLoop)*basis);
    slewNoise=(abs(change*gain)+abs(gain))*cap+abs(gain*sampledDrift);
    disturbance=abs(modal*cruise.transition(2:6,7:8)*gain)*cap+abs(modal*sampledDrift);
    % Numerical eigensynthesis residual is included as a nonnegative modal
    % coupling; its certificate is checked componentwise below.
    transformed=modal*cruise.closedLoop*basis;
    comparison=abs(transformed)+4096*eps*(1+abs(modal)*abs(cruise.closedLoop)*abs(basis));
    base=(eye(5)-comparison)\(disturbance+2*reserve);
    available=[holdBound-noise;rate(finite)-slewNoise(finite)];
    shape=[support;slewSupport(finite,:)];
    direction=(eye(5)-comparison)\ones(5,1);
    direction=direction/max(direction);
    growth=shape*direction;
    active=growth>0;
    scale=min((available(active)-shape(active,:)*base)./growth(active));
    radius=base+.95*scale*direction;
    if any(radius<=0) || any(base<0) || any(direction<=0) || ~(scale>0) ...
            || any(comparison*radius+disturbance>radius-2*reserve) ...
            || any(shape*radius>available)
        error('collisionAvoidanceController:invalidTerminalModel', ...
            'The sensing and actuator contract has no certified modal terminal set.');
    end
    deviationRows=[rows(:,6:7);change(finite,:);-change(finite,:)];
    deviationBound=[holdBound-support*radius-noise; ...
        rate(finite)-slewSupport(finite,:)*radius-slewNoise(finite); ...
        rate(finite)-slewSupport(finite,:)*radius-slewNoise(finite)];
    terminal=struct('cruise',cruise,'reference',cruise.state,'input',trim,'stateIndex',2:6, ...
        'feedback',[zeros(2,1),-gain],'modalMatrix',modal,'modalBasis',basis,'radius',radius,'reserve',reserve, ...
        'measurementRadiusLimit',model.measurementRadiusLimit,'holdRows',rows,'holdBound',holdBound, ...
        'sampleTime',model.sampleTime,'contraction',contraction,'comparison',comparison, ...
        'disturbanceSupport',disturbance,'holdSupport',support,'holdNoise',noise, ...
        'deviationRows',deviationRows,'deviationBound',deviationBound, ...
        'continuousA',cruise.stage.continuousA,'continuousB',cruise.stage.continuousB, ...
        'continuousC',cruise.stage.continuousC,'targetIndependent',true, ...
        'scope',"boundedMeasurementRobustModalCruise",'sameOnlineGenerator',true);
    terminal.nextRadius=radius;
    terminal.nextModalMatrix=modal;
    terminal.nextReference=cruise.state;
    terminal.successorInputMap=modal*cruise.transition(2:6,7:8);
    savedKey=key;saved=terminal;
end

function terminal=localScheduledTerminalSet(model)
% A finite reference-indexed family with an invariant constant-curvature tail.
% Every transition and held phase domain is checked; a curvature grid is not
% treated as a certificate for an undeclared continuous parameter family.
    cfg=model.cfg;
    if isfield(model,'road') && ~isempty(model.road.boundaries)
        error('collisionAvoidanceController:optimizationFailed', ...
            'The scheduled terminal certificate requires an unbounded road-free reference domain.');
    end
    bank=ltvBicycleModel.referenceSchedule(model);
    if isfield(model,'referencePhaseIndex')
        index=model.referencePhaseIndex;
    else
        current=ltvBicycleModel.sampledCruise(model);index=current.index;
    end
    persistent savedKey saved
    % The bank stamp identifies the compiled schedule within this session.
    key={bank.stamp,model.measurementRadiusLimit,cfg.solver.constraintTolerance};
    if isempty(savedKey) || ~isequaln(savedKey,key)
        saved=localScheduledTerminalFamily(model,bank);
        savedKey=key;
    end
    selected=min(index,bank.tailIndex);
    terminal=saved{selected};
    reference=ltvBicycleModel.referenceAt(model,index);
    terminal.reference=reference.state;terminal.nextReference=reference.nextState;
    terminal.cruise=reference;terminal.index=index;
    % Keep only inputs needed to select the immutable successor certificate.
    terminal.scheduleModel=struct('cfg',cfg,'sampleTime',model.sampleTime,'lane',model.lane, ...
        'longitudinalAccelerationBias',model.longitudinalAccelerationBias, ...
        'measurementRadiusLimit',model.measurementRadiusLimit, ...
        'initialEgoState',reference.state,'stateTime',cfg.clf.referenceEpoch+(index-1)*model.sampleTime);
end

function terminals=localScheduledTerminalFamily(model,bank)
% Sparse offline feasibility synthesis. The online optimization still uses
% six modal SOCs, actuator rows and a short certified prediction horizon.
    cfg=model.cfg;count=bank.tailIndex;dimension=6;decisionCount=dimension*count+1;
    cap=model.measurementRadiusLimit(:);h=model.sampleTime;
    reserve=16*max(cfg.encounter.numericalMargin,cfg.solver.constraintTolerance);
    phaseRadius=cfg.encounter.referencePhaseRadius;
    scales=[min(2,phaseRadius);cfg.clf.lateralPositionErrorScale;cfg.clf.headingErrorScale; ...
        cfg.clf.speedErrorScale;cfg.clf.lateralVelocityErrorScale;cfg.clf.yawRateErrorScale];
    q=diag(1./scales.^2);
    inputWeight=diag([10*cfg.clf.frontWheelSteeringAngleWeight,cfg.clf.brakingRatioWeight]);
    gains=zeros(2,dimension,count);closed=zeros(dimension,dimension,count);
    for index=1:count
        reference=bank.certificates{index};f=reference.transition(1:6,1:6);g=reference.transition(1:6,7:8);
        gains(:,:,index)=dlqr(f,g,q,inputWeight);
        closed(:,:,index)=f-g*gains(:,:,index);
    end
    [basis,~]=eig(closed(:,:,1));modal=basis\eye(dimension);
    if rcond(basis)<1e-10
        error('collisionAvoidanceController:invalidTerminalModel', ...
            'The scheduled terminal modal coordinates are ill-conditioned.');
    end
    lower=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
    upper=[cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
    rate=h*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    finiteRate=isfinite(rate);inputRows=[eye(2);-eye(2)];
    matrixParts=cell(count,1);boundParts=cell(count,1);equalParts=cell(count,1);
    ingredients=cell(count,1);
    for index=1:count
        next=min(index+1,count);columns=(index-1)*dimension+(1:dimension);
        nextColumns=(next-1)*dimension+(1:dimension);
        reference=bank.certificates{index};nextReference=bank.certificates{next};
        gain=gains(:,:,index);nextGain=gains(:,:,next);
        g=reference.transition(1:6,7:8);fc=closed(:,:,index);
        drift=reference.transition(1:6,:)*[reference.state;reference.input;1]-reference.nextState;
        comparison=abs(modal*fc*basis)+4096*eps*(1+abs(modal)*abs(fc)*abs(basis));
        disturbance=abs(modal*drift)+abs(modal*g*gain)*cap;
        holdBound=[upper-reference.input;reference.input-lower];
        holdBound=holdBound-reserve*(1+abs(holdBound));
        holdSupport=abs(inputRows*gain*basis);holdNoise=abs(inputRows*gain)*cap;
        change=eye(2)+nextGain*g;
        slewSupport=abs((gain-nextGain*fc)*basis);
        slewNoise=abs(gain+nextGain*g*gain)*cap+abs(nextGain)*cap ...
            +abs(nextReference.input-reference.input-nextGain*drift);
        % This generous enclosure supplies Taylor arithmetic limits. Its
        % support rows below verify it; it is not a physical state limit.
        envelope=[phaseRadius;1000*ones(5,1)];
        stage=reference.stage;
        tube=stateUncertainty.heldInterval(stage.continuousA,[zeros(6),stage.continuousB], ...
            stage.continuousC,[eye(6),zeros(6,2)],reference.state,zeros(6,1),zeros(6,1), ...
            h,cfg.encounter.taylorOrder,[envelope;max(abs([lower,upper]),[],2)],zeros(6,1));
        coefficients=size(tube.offset,2);
        phaseSupport=zeros(coefficients,dimension);phaseBound=zeros(coefficients,1);
        phaseInput=zeros(coefficients,2);
        for coefficient=1:coefficients
            fraction=(coefficient-1)/(coefficients-1);
            phase=reference.state(1)+fraction*(reference.nextState(1)-reference.state(1));
            flowInput=tube.map(1,7:8,coefficient);
            flowState=tube.map(1,1:6,coefficient);
            driftHold=tube.offset(1,coefficient)+flowInput*reference.input-phase;
            phaseSupport(coefficient,:)=abs((flowState-flowInput*gain)*basis);
            phaseBound(coefficient)=phaseRadius-abs(driftHold)-abs(flowInput*gain)*cap ...
                -tube.radius(1,coefficient)-reserve;
            phaseInput(coefficient,:)=flowInput;
        end
        if isfinite(reference.lateralRegularityRadius)
            for coefficient=1:coefficients
                flowInput=tube.map(2,7:8,coefficient);
                flowState=tube.map(2,1:6,coefficient);
                driftHold=tube.offset(2,coefficient)+flowInput*reference.input;
                phaseSupport(end+1,:)=abs((flowState-flowInput*gain)*basis); %#ok<AGROW>
                phaseBound(end+1,1)=reference.lateralRegularityRadius-abs(driftHold) ...
                    -abs(flowInput*gain)*cap-tube.radius(2,coefficient)-reserve; %#ok<AGROW>
                phaseInput(end+1,:)=flowInput; %#ok<AGROW>
            end
        end
        localMatrix=[comparison;holdSupport;slewSupport(finiteRate,:);abs(basis); ...
            phaseSupport;-eye(dimension)];
        localBound=[-disturbance-2*reserve;holdBound-holdNoise-2*reserve; ...
            rate(finiteRate)-slewNoise(finiteRate)-2*reserve;envelope; ...
            phaseBound-2*reserve;zeros(dimension,1)];
        rows=sparse(size(localMatrix,1),decisionCount);rows(:,columns)=localMatrix;
        rows(1:dimension,nextColumns)=rows(1:dimension,nextColumns)-eye(dimension);
        rows(end-dimension+1:end,end)=1;
        matrixParts{index}=rows;boundParts{index}=localBound;
        pairs=sparse(0,decisionCount);
        for mode=1:dimension
            if all(abs(imag(basis(:,mode)))<1e-14),continue;end
            [distance,match]=min(sum(abs(basis-conj(basis(:,mode))).^2,1));
            if match>mode && distance<1e-16
                row=sparse(1,decisionCount);row(columns(mode))=1;row(columns(match))=-1;
                pairs=[pairs;row]; %#ok<AGROW>
            end
        end
        equalParts{index}=pairs;
        ingredients{index}=struct('comparison',comparison,'disturbance',disturbance, ...
            'holdBound',holdBound,'holdSupport',holdSupport,'holdNoise',holdNoise, ...
            'change',change,'slewSupport',slewSupport,'slewNoise',slewNoise, ...
            'phaseSupport',phaseSupport,'phaseBound',phaseBound,'phaseInput',phaseInput);
    end
    matrix=vertcat(matrixParts{:});bound=vertcat(boundParts{:});equal=vertcat(equalParts{:});
    objective=zeros(decisionCount,1);objective(end)=-1;
    options=optimoptions('linprog','Display','none','ConstraintTolerance',1e-9);
    [solution,~,flag]=linprog(objective,matrix,bound,equal,zeros(size(equal,1),1), ...
        [repmat(4*reserve,dimension*count,1);0],[],options);
    if flag<=0 || isempty(solution) || any(~isfinite(solution)) ...
            || max(matrix*solution-bound)>reserve/4 ...
            || any(abs(equal*solution)>reserve/4)
        error('collisionAvoidanceController:invalidTerminalModel', ...
            'No verified phase-indexed terminal family satisfies sensing, phase and actuator constraints.');
    end
    radii=reshape(solution(1:end-1),dimension,count);terminals=cell(count,1);
    for index=1:count
        next=min(index+1,count);reference=bank.certificates{index};data=ingredients{index};
        gain=gains(:,:,index);radius=radii(:,index);nextRadius=radii(:,next);
        phaseRoom=data.phaseBound-data.phaseSupport*radius;
        slewRoom=rate(finiteRate)-data.slewSupport(finiteRate,:)*radius-data.slewNoise(finiteRate);
        deviationRows=[inputRows;data.change(finiteRate,:);-data.change(finiteRate,:); ...
            data.phaseInput;-data.phaseInput];
        deviationBound=[data.holdBound-data.holdSupport*radius-data.holdNoise; ...
            slewRoom;slewRoom;phaseRoom;phaseRoom];
        if any(data.comparison*radius+data.disturbance>nextRadius-reserve) ...
                || any(deviationBound<reserve)
            error('collisionAvoidanceController:invalidTerminalModel', ...
                'The independently checked terminal family has insufficient numerical reserve.');
        end
        stage=reference.stage;
        terminals{index}=struct('cruise',reference,'reference',reference.state,'input',reference.input, ...
            'feedback',-gain,'stateIndex',1:6,'modalMatrix',modal,'modalBasis',basis,'radius',radius, ...
            'nextRadius',nextRadius,'nextModalMatrix',modal,'nextReference',reference.nextState, ...
            'successorInputMap',modal*reference.transition(1:6,7:8),'reserve',reserve, ...
            'measurementRadiusLimit',cap,'sampleTime',h,'comparison',data.comparison, ...
            'contraction',abs(eig(closed(:,:,index))),'disturbanceSupport',data.disturbance, ...
            'holdRows',[zeros(4,dimension),inputRows],'holdBound',data.holdBound, ...
            'holdSupport',data.holdSupport,'holdNoise',data.holdNoise, ...
            'deviationRows',deviationRows,'deviationBound',deviationBound, ...
            'continuousA',stage.continuousA,'continuousB',stage.continuousB,'continuousC',stage.continuousC, ...
            'targetIndependent',true,'sameOnlineGenerator',true,'scheduled',true,'index',index, ...
            'phaseRadius',phaseRadius,'scope',"boundedPhaseScheduledModalContinuation");
    end
end

function [count,inputs]=localCruiseAdmission(model)
% A bounded feedback rollout proposes sufficient recovery TIME, not a hard
% early-recovery requirement or a substitute actuator command.
    terminal=localTerminalSet(model);cfg=model.cfg;
    count=model.horizonSteps;maximum=4*count;
    inputs=zeros(2,maximum);x=model.initialEgoState;rho=model.initialFrenetErrorBound;
    previous=model.previousInput;cruise=terminal.cruise;feasibleCount=0;
    lower=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
    upper=[cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
    rate=model.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    for index=1:maximum
        input=terminal.input+terminal.feedback*(x-terminal.reference);
        input=min(max(input,max(lower,previous-rate)),min(upper,previous+rate));
        inputs(:,index)=input;previous=input;
        x=cruise.transition(1:6,:)*[x;input;1];
        rho=abs(cruise.transition(1:6,1:6))*rho;
        if isfield(terminal,'scheduled') && terminal.scheduled
            nextModel=terminal.scheduleModel;
            nextModel.referencePhaseIndex=terminal.index+1;
            if isfield(model,'referenceBank'),nextModel.referenceBank=model.referenceBank;end
            terminal=localTerminalSet(nextModel);cruise=terminal.cruise;
        end
        [~,margin]=hardEncounterBarrier.terminalMembership(terminal,x,rho);
        if all(margin>4*terminal.reserve)
            if index>=cfg.controller.minimumHorizonSteps,feasibleCount=index;end
            if index>=count,count=index;inputs=inputs(:,1:count);return;end
        end
    end
    count=maximum;if feasibleCount>0,count=feasibleCount;end
    inputs=inputs(:,1:count);
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
        cfg.controller.stationTrustRadius,abs(model.initialEgoState(2))+model.initialFrenetErrorBound(2));
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
    body = hypot(cfg.vehicle.length,cfg.vehicle.width)/2+hypot(target.halfLength,target.halfWidth);
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
    [pose,~]=laneGeometry.poseData(frame);
    direction = center(1:2)-pose(1:2)-reshape(pose(3:14),2,6)*anchor;
    if norm(direction)<sqrt(eps), direction = frame.lateral; end
    direction = direction/norm(direction);
end

function [row,bound] = localExitRow(model,target,center,radius,egoRadius,frame,direction)
% Directional exterior membership is a convex inner approximation of the
% complement of the range ball. Charge both boxes, chart error and the entire
% target body. Norm scaling keeps the implication valid after normalization.
    [pose,domain]=laneGeometry.poseData(frame);
    row = direction.'*reshape(pose(3:14),2,6);
    positionCharge=abs(direction).'*frame.positionErrorBound;
    if domain(1)>0,positionCharge=norm(direction)*pose(22);end
    body = targetPrediction.rectangleSupport(target.halfLength,target.halfWidth, ...
        -direction,center(7),radius(7))*norm(direction);
    distance = model.confirmation.range+model.cfg.encounter.numericalMargin;
    bound = direction.'*(center(1:2)-pose(1:2))-abs(direction).'*radius(1:2) ...
        -abs(row)*egoRadius-positionCharge ...
        -body-distance*norm(direction);
    bound = bound-256*eps*(1+abs(bound)+abs(direction).'*(abs(center(1:2))+abs(frame.origin)));
end
