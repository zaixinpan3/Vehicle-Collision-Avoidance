function [program,prediction,clf] = formulateAvoidanceProblem(model)
%formulateAvoidanceProblem Common convex base and separation certificates.
% Retain inherited occupied sets and directions, and delegate complete
% trajectory optimization with fixed-direction support bounds to the solver.
    carry=model.carriedWitness;
    if isempty(carry)
        [program,prediction,clf]=localFormulate(model);
        return;
    end
    if isempty(model.encounters)
        fresh=model;fresh.carriedWitness=[];
        fresh.horizonSteps=max(model.cfg.controller.horizonSteps,model.cfg.controller.minimumHorizonSteps);
        fresh.initializationPlan=localContinuation(model,fresh.horizonSteps);
        [candidate,predicted,candidateClf]=localFormulate(fresh);
        if localWitnessFeasible(candidate,candidate.feasibleWitness)
            program=candidate;prediction=predicted;clf=candidateClf;
            program.replacementContainsWitness=true;
            return;
        end
    end
    if carry.remainingSteps>0
        [program,prediction,clf]=localFormulate(model);
    else
        % The confirmed exit has occurred and true-state modal membership is
        % retained. Optimize a new hold with a proved feasible input.
        model.carriedWitness=[];model.horizonSteps=1;model.terminalOptimization=true;
        model.initializationPlan=localContinuation(model,1);
        [program,prediction,clf]=localFormulate(model);
    end
end

function inputs=localContinuation(model,count)
    carry=model.carriedWitness;
    cruise=model.cruiseCertificate;
    x=model.initialEgoState;inputs=zeros(2,count);
    retained=0;
    if ~isempty(carry),retained=min(count,size(carry.inputs,2));end
    for stage=1:count
        if isfield(cruise,'scheduled') && cruise.scheduled
            cruise=ltvBicycleModel.referenceAt(model,model.cruiseCertificate.index+stage-1);
        end
        if stage<=retained
            input=carry.inputs(:,stage);
        else
            if isfield(cruise,'scheduled') && cruise.scheduled
                terminalModel=model;terminalModel.referencePhaseIndex=cruise.index;
                terminal=hardEncounterBarrier.terminalCertificate(terminalModel);
                input=terminal.input+terminal.feedback*(x-terminal.reference);
            else
                input=cruise.input-cruise.gain*(x(2:6)-cruise.state(2:6));
            end
        end
        inputs(:,stage)=input;
        x=cruise.transition(1:6,:)*[x;input;1];
    end
end

function witness=localCompleteSlack(program,anchor)
    witness=[anchor;0];
    first=program.cones(2)+1;
    value=program.b(first:first+5)-program.A(first:first+5,:)*witness;
    witness(end)=max(0,norm(value(2:end))-value(1))+1;
end

function accepted=localWitnessFeasible(program,witness)
    value=program.b-program.A*witness;
    allowance=64*numel(witness)*eps*(1+abs(program.b)+abs(program.A)*abs(witness));
    count=program.cones(2);
    accepted=all(isfinite(value)) && all(value(1:count)>=allowance(1:count));
    for dimension=program.cones(3:end).'
        cone=value(count+(1:dimension));
        accepted=accepted && cone(1)>=norm(cone(2:end))+norm(allowance(count+(1:dimension)));
        count=count+dimension;
    end
end

function [program,prediction,clf] = localFormulate(model)
%formulateAvoidanceProblem Predictive hard safety with a soft sampled CLF.
% Optimize the complete input sequence and the first-hold CLF norm slack.
% The finite target exit and invariant road terminal set are hard constraints.
    cfg = model.cfg;
    cruise = model.cruiseCertificate;
    if isempty(cruise),cruise=ltvBicycleModel.sampledCruise(model);end
    model.cruiseCertificate=cruise;
    inherited = ~isempty(model.carriedWitness);
    dual=struct('available',false,'overlappingNodes',0, ...
        'minimumAnchorDistance',Inf,'normalSwitchCount',0, ...
        'used',false,'witnessPreserved',false);
    if inherited
        if ~laneGeometry.isVaryingReference(model.lane)
            cruise = model.carriedWitness.program.cruiseCertificate;
        end
        [prediction,geometry,matrix,physicalBound,bound,terminal,completion,anchor,labels,terminalCone] = localShift(model);
    else
        [prediction,anchor] = hardEncounterBarrier.predict(model,cruise);
        model.anchorPlan = anchor;
        if ~isempty(model.encounters)
            [frames,nominal]=laneGeometry.sweptCellFrames(model,prediction.cells,anchor);
            [normals,dual]=avoidanceSafetyGeometry.supportNormals(model,prediction,anchor);
            if ~dual.available
                error('collisionAvoidanceController:invalidSeparationNormal','A finite unit support direction is required.');
            end
            prediction.geometryAnchor=anchor;prediction.geometryFrames=frames;
            prediction.geometryNominal=nominal;prediction.separationNormals=normals;
            dual.used=true;
        end
        geometry = avoidanceSafetyGeometry.build(model,prediction);
        [terminalMatrix,terminalBound,terminal,completion,terminalCone] = ...
            hardEncounterBarrier.completionRows(model,prediction,geometry);
        count = prediction.stageCount;
        planCount = 2*count;
        lower = repmat([-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum],count,1);
        upper = repmat([cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum],count,1);
        rate = model.sampleTime*repmat([cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum],count,1);
        difference = eye(planCount)-diag(ones(planCount-2,1),-2);
        prior = [model.previousInput;zeros(planCount-2,1)];
        finiteRate = isfinite(rate);
        if isfield(model,'terminalOptimization') && model.terminalOptimization
            % This one-hold optimization uses the invariant policy only to
            % prove nonemptiness. Both actuator coordinates remain decisions.
            candidate=terminal.input+terminal.feedback*(model.initialEgoState-terminal.reference);
            matrix=terminal.deviationRows;
            physicalBound=terminal.deviationBound+matrix*candidate;
            labels=repmat("terminalHoldAndSlew",numel(physicalBound),1);
            matrix=[matrix;difference(finiteRate,:);-difference(finiteRate,:)];
            physicalBound=[physicalBound;rate(finiteRate)+prior(finiteRate);rate(finiteRate)-prior(finiteRate)];
            labels=[labels;repmat("slew",2*nnz(finiteRate),1)];
            mapped=terminal.modalMatrix*cruise.transition(terminal.stateIndex,7:8);
            nextRadius=terminal.radius;
            if isfield(terminal,'nextRadius')
                nextRadius=terminal.nextRadius;mapped=terminal.successorInputMap;
            end
            room=nextRadius-terminal.comparison*terminal.radius-terminal.disturbanceSupport;
            for mode=1:numel(terminal.radius)
                selected=3*mode-2:3*mode;
                terminalCone.matrix(selected,:)=[zeros(1,2);-real(mapped(mode,:));-imag(mapped(mode,:))];
                terminalCone.bound(selected)=[room(mode)-terminal.reserve; ...
                    -real(mapped(mode,:)*candidate);-imag(mapped(mode,:)*candidate)];
            end
            bound=physicalBound;
        else
            matrix = [geometry.matrix;eye(planCount);-eye(planCount); ...
                difference(finiteRate,:);-difference(finiteRate,:);terminalMatrix];
            physicalBound = [geometry.physicalBound;upper;-lower; ...
                rate(finiteRate)+prior(finiteRate);rate(finiteRate)-prior(finiteRate);terminalBound];
            labels=[geometry.label;repmat("actuator",2*planCount,1); ...
                repmat("slew",2*nnz(finiteRate),1); ...
                repmat("terminalEntry",numel(terminalBound)-numel(model.encounters)-completion.domainRowCount,1); ...
                repmat("terminalPoseDomain",completion.domainRowCount,1); ...
                "exit:"+string({model.encounters.key}).'];
            reach = max(abs(lower),abs(upper));
            scale = 1+abs(physicalBound)+abs(matrix)*reach;
            reserve = 4*max(cfg.encounter.numericalMargin,cfg.solver.constraintTolerance) ...
                *scale.*any(matrix~=0,2);
            bound = physicalBound-reserve;
        end
    end
    count = prediction.stageCount;
    planCount = 2*count;
    reach = repmat([cfg.model.frontWheelSteeringAngleMaximum; ...
        max(abs([cfg.actuation.brakingRatioMinimum,cfg.actuation.brakingRatioMaximum]))],count,1);
    safetyBound = bound;
    % Only the CLF has a slack column. Physical inequalities remain hard.
    physicalMatrix = [matrix,zeros(numel(bound),1)];
    bound = [bound;0];
    linearCount = numel(bound);
    currentRoot = chol(cruise.matrix);
    nextState=cruise.state;nextMatrix=cruise.matrix;
    if isfield(cruise,'nextState'),nextState=cruise.nextState;nextMatrix=cruise.nextMatrix;end
    root = chol(nextMatrix);
    trackingError = model.initialEgoState(2:6)-cruise.state(2:6);
    radius = model.initialFrenetErrorBound(2:6);
    contraction = 1-cruise.decayPerHold;
    middle = (contraction+cruise.contraction)/2;
    stateMap = cruise.transition(2:6,1:6);
    inputMap = cruise.transition(2:6,7:8);
    nominalOffset = stateMap*model.initialEgoState+cruise.transition(2:6,9)-nextState(2:6);
    inputRoot = root*inputMap;
    numeric = 100*cfg.solver.constraintTolerance*(1+norm(inputRoot,'fro')*norm(reach(1:2)));
    coneRadius = sqrt(middle)*norm(currentRoot*trackingError)+numeric;
    % The complete constraint matrix is assembled sparse from the dense
    % physical rows and the small slack, CLF-cone and terminal-cone blocks.
    tail = [zeros(1,planCount),-1;zeros(1,planCount),-1;-inputRoot,zeros(5,planCount-1); ...
        terminalCone.matrix,zeros(size(terminalCone.matrix,1),1)];
    matrix = [sparse(physicalMatrix);sparse(tail)];
    bound = [bound;coneRadius;root*nominalOffset;terminalCone.bound];
    disturbance = sqrt(middle)*norm(abs(currentRoot)*radius) ...
        +norm(abs(root*stateMap)*model.initialFrenetErrorBound)+2*numeric;
    clf = struct('cruise',cruise,'initialValue',trackingError.'*cruise.matrix*trackingError, ...
        'decayPerHold',cruise.decayPerHold, ...
        'disturbanceBound',contraction/(contraction-middle)*disturbance^2, ...
        'normDisturbance',disturbance,'youngFactor',contraction/(contraction-middle));
    % Only the five path/velocity errors enter performance; phase is a hard
    % model-alignment domain and a terminal certificate coordinate.
    objectiveMap=zeros(5*count,planCount);objectiveOffset=zeros(5*count,1);
    referenceStates=repmat(cruise.state,1,count+1);
    referenceInputs=repmat(cruise.input,1,count);
    referenceMatrices=repmat(cruise.matrix,1,1,count+1);
    if isfield(prediction,'referenceStates')
        referenceStates=prediction.referenceStates;referenceInputs=prediction.referenceInputs;
        referenceMatrices=prediction.referenceMatrices;
    end
    for stage=1:count
        rows=5*(stage-1)+(1:5);stageRoot=chol(referenceMatrices(:,:,stage+1));
        objectiveMap(rows,:)=stageRoot*prediction.egoStateMatrix(2:6,:,stage+1);
        objectiveOffset(rows)=stageRoot*(prediction.egoStateOffset(2:6,stage+1)-referenceStates(2:6,stage+1));
    end
    inputWeight=repmat([cfg.clf.frontWheelSteeringAngleWeight;cfg.clf.brakingRatioWeight],count,1);
    weight=spdiags(inputWeight,0,planCount,planCount);
    hessian=objectiveMap.'*objectiveMap+weight;
    linear=2*(objectiveMap.'*objectiveOffset-weight*referenceInputs(:));
    hessian=2*[hessian,zeros(planCount,1);zeros(1,planCount),cfg.clf.relaxationWeight];
    layout = struct('planIndex',1:planCount,'planCount',planCount,'horizonSteps',count, ...
        'decisionCount',planCount+1,'relaxationIndex',planCount+1);
    program = struct('P',sparse(hessian),'q',[linear;0],'A',matrix,'b',bound, ...
        'cones',[0;linearCount;6;terminalCone.sizes],'decisionRadius',reach, ...
        'obstacleCbfRowCount',nnz(startsWith(geometry.label,"collision:")), ...
        'geometry',geometry,'terminal',terminal,'completion',completion,'layout',layout, ...
        'clfNumericalReserve',numeric, ...
        'physicalMatrix',physicalMatrix,'physicalBound',physicalBound, ...
        'physicalLabels',labels,'terminalCone',terminalCone, ...
        'anchorPlan',anchor,'safetyBound',safetyBound,'inheritedPredictionFamily',inherited, ...
        'inheritedFeasibleFamily',inherited, ...
        'cruiseCertificate',cruise,'clf',clf, ...
        'prediction',prediction,'inputWeight',inputWeight, ...
        'referenceStates',referenceStates,'referenceInputs',referenceInputs,'referenceMatrices',referenceMatrices, ...
        'slackWeight',cfg.clf.relaxationWeight);
    program.terminalConePhysicalBound=terminalCone.bound;
    program.terminalConePhysicalBound(1:3:end)=program.terminalConePhysicalBound(1:3:end)+terminal.reserve;
    if inherited
        program.terminalConePhysicalBound=model.carriedWitness.program.terminalConePhysicalBound ...
            -model.carriedWitness.program.terminalCone.matrix(:,1:2)*model.carriedWitness.issuedInput;
    end
    program.replacementContainsWitness=false;
    program.terminalOptimization=isfield(model,'terminalOptimization') && model.terminalOptimization;
    program.feasibleWitness=localCompleteSlack(program,anchor);
    if inherited,program.inheritedWitness=program.feasibleWitness;end
    program.supportGeometry=dual;
    if ~isempty(model.encounters)
        program=avoidanceSafetyGeometry.jointProgram(program,model);
    end
    program.fluidReference=solveHardCbfClf.prepareFluidReference(program,model);
end

function [prediction,geometry,matrix,physicalBound,bound,terminal,completion,anchor,labels,terminalCone] = localShift(model)
% Eliminate the executed input from the accepted affine family. The old
% enclosures and numerical reserves are inherited without reconstruction.
% Conditioning only restricts the true state set, so it cannot invalidate
% these constraints. Rows independent of every remaining input concern the
% already certified fixed prefix and are removed with that prefix.
    carry = model.carriedWitness;
    old = carry.program;
    executed = carry.issuedInput;
    columns = 3:old.layout.planCount;
    matrix = old.physicalMatrix(:,columns);
    labels=old.physicalLabels;
    discharging=~isempty(model.dischargedTargetKeys);
    removed=false(numel(labels),1);
    if discharging
        removed=ismember(labels,"collision:"+model.dischargedTargetKeys) ...
            | ismember(labels,"exit:"+model.dischargedTargetKeys);
    end
    selected = any(matrix~=0,2) & ~removed;
    geometric=numel(old.geometry.label);
    selected(1:geometric)=old.geometry.stage>=2 & ~removed(1:geometric);
    physicalBound = old.physicalBound-old.physicalMatrix(:,1:2)*executed;
    bound = old.safetyBound-old.physicalMatrix(:,1:2)*executed;
    matrix = matrix(selected,:);physicalBound=physicalBound(selected);bound=bound(selected);
    labels=labels(selected);
    terminalCone=old.terminalCone;
    terminalCone.bound=terminalCone.bound-terminalCone.matrix(:,1:2)*executed;
    terminalCone.matrix=terminalCone.matrix(:,columns);
    terminal=carry.terminal;completion=carry.completion;anchor=carry.inputs(:);
    retained=~ismember(completion.keys,model.dischargedTargetKeys);
    completion.keys=completion.keys(retained);
    completion.direction=completion.direction(:,retained);
    completion.stateRow=completion.stateRow(retained,:);
    completion.stateBound=completion.stateBound(retained);
    completion.active=~isempty(completion.keys);
    prediction=carry.prediction;
    prediction.nominalInitialState=carry.predictedCenter;
    prediction.stageCount=prediction.stageCount-1;
    prediction.nodeCount=prediction.nodeCount-1;
    prediction.planCount=prediction.planCount-2;
    prediction.egoStateOffset=prediction.egoStateOffset(:,2:end)+reshape( ...
        pagemtimes(prediction.egoStateMatrix(:,1:2,2:end),executed),6,[]);
    prediction.egoStateMatrix=prediction.egoStateMatrix(:,columns,2:end);
    nodeFields={'initialErrorBound','egoStateErrorBound','domainErrorBound'};
    for index=1:numel(nodeFields)
        name=nodeFields{index};prediction.(name)=prediction.(name)(:,2:end);
    end
    stageFields={'continuousA','continuousB','stageMatrixA','stageMatrixB'};
    for index=1:numel(stageFields)
        name=stageFields{index};prediction.(name)=prediction.(name)(:,:,2:end);
    end
    stageFields={'continuousC','stageAffine','modelErrorRateBound','executionReserve'};
    for index=1:numel(stageFields)
        name=stageFields{index};prediction.(name)=prediction.(name)(:,2:end);
    end
    prediction.scheduleSpeedProfile=prediction.scheduleSpeedProfile(2:end);
    prediction.scheduleCurvature=prediction.scheduleCurvature(2:end);
    prediction.scheduleBrakingRatio=prediction.scheduleBrakingRatio(2:end);
    prediction.tireModels=prediction.tireModels(2:end);
    if isfield(prediction,'referencePhaseIndex')
        prediction.referencePhaseIndex=prediction.referencePhaseIndex+1;
        prediction.referenceStates=prediction.referenceStates(:,2:end);
        prediction.referenceInputs=prediction.referenceInputs(:,2:end);
        prediction.referenceMatrices=prediction.referenceMatrices(:,:,2:end);
    end
    keep=[prediction.cells.stage]>=2;
    prediction.cells=prediction.cells(keep);
    for index=1:numel(prediction.cells)
        tube=prediction.cells(index);
        tube.offset=tube.offset+reshape(pagemtimes(tube.map(:,1:2,:),executed),6,[]);
        tube.map=tube.map(:,columns,:);
        tube.endOffset=tube.endOffset+tube.endMap(:,1:2)*executed;
        tube.endMap=tube.endMap(:,columns);
        tube.stage=tube.stage-1;
        % Each certificate is one hold node. Reconstruct its clock from the
        % integer stage; repeated subtraction can create negative zero.
        tube.start=tube.stage*model.sampleTime;
        tube.time=tube.start+linspace(0,tube.duration,numel(tube.time));
        prediction.cells(index)=tube;
    end
    geometry=old.geometry;
    selected=geometry.stage>=2;
    if discharging
        selected=selected & ~ismember(geometry.label,"collision:"+model.dischargedTargetKeys);
    end
    geometry.physicalBound=geometry.physicalBound(selected)-geometry.matrix(selected,1:2)*executed;
    geometry.matrix=geometry.matrix(selected,columns);
    geometry.stage=geometry.stage(selected)-1;geometry.label=geometry.label(selected);
    geometry.safety=geometry.safety(selected);
    geometry.cellIndex=geometry.cellIndex(selected);
    geometry.cellIndex=geometry.cellIndex-min(geometry.cellIndex)+1;
    geometry.frames=geometry.frames(keep);geometry.normals=geometry.normals(keep);
    geometry.normals=cellfun(@(normal) normal(:,retained),geometry.normals,UniformOutput=false);
    geometry.local=geometry.local(keep);
    geometry.cellData=geometry.cellData(keep);
    for index=1:numel(geometry.local)
        local=geometry.local(index);
        if discharging
            rows=~ismember(local.nodeLabels,"collision:"+model.dischargedTargetKeys);
            expanded=repmat(rows,size(local.nodeStateRows,3),1);
            local.stateMatrix=local.stateMatrix(expanded,:);
            local.inputMatrix=local.inputMatrix(expanded,:);local.bound=local.bound(expanded);
            local.nodeStateRows=local.nodeStateRows(rows,:,:);
            local.nodeStartStateRows=local.nodeStartStateRows(rows,:,:);
            local.nodeInputRows=local.nodeInputRows(rows,:,:);
            local.nodeLimits=local.nodeLimits(rows,:);local.nodeLabels=local.nodeLabels(rows);
            geometry.cellData(index).targets=geometry.cellData(index).targets(retained);
            geometry.cellData(index).normals=geometry.cellData(index).normals(:,retained);
        end
        local.stage=local.stage-1;geometry.local(index)=local;
    end
end
