function [program,prediction,clf] = formulateAvoidanceProblem(model)
%formulateAvoidanceProblem Predictive hard safety with a soft sampled CLF.
% Optimize the complete input sequence and the first-hold CLF norm slack.
% The finite target exit and invariant road terminal set are hard constraints.
    cfg = model.cfg;
    cruise = ltvBicycleModel.sampledCruise(model);
    inherited = ~isempty(model.carriedWitness);
    if inherited
        cruise = model.carriedWitness.program.cruiseCertificate;
        [prediction,geometry,matrix,physicalBound,bound,terminal,completion,anchor] = localShift(model);
    else
        [prediction,anchor] = hardEncounterBarrier.predict(model,cruise);
        model.anchorPlan = anchor;
        geometry = avoidanceSafetyGeometry.build(model,prediction);
        [terminalMatrix,terminalBound,terminal,completion] = ...
            hardEncounterBarrier.completionRows(model,prediction,geometry);
        count = prediction.stageCount;
        planCount = 2*count;
        lower = repmat([-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum],count,1);
        upper = repmat([cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum],count,1);
        rate = model.sampleTime*repmat([cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum],count,1);
        difference = eye(planCount)-diag(ones(planCount-2,1),-2);
        prior = [model.previousInput;zeros(planCount-2,1)];
        finiteRate = isfinite(rate);
        matrix = [geometry.matrix;eye(planCount);-eye(planCount); ...
            difference(finiteRate,:);-difference(finiteRate,:);terminalMatrix];
        physicalBound = [geometry.physicalBound;upper;-lower; ...
            rate(finiteRate)+prior(finiteRate);rate(finiteRate)-prior(finiteRate);terminalBound];
        reach = max(abs(lower),abs(upper));
        scale = 1+abs(physicalBound)+abs(matrix)*reach;
        reserve = 4*max(cfg.encounter.numericalMargin,cfg.solver.constraintTolerance) ...
            *scale.*any(matrix~=0,2);
        bound = physicalBound-reserve;
    end
    count = prediction.stageCount;
    planCount = 2*count;
    reach = repmat([cfg.model.frontWheelSteeringAngleMaximum; ...
        max(abs([cfg.actuation.brakingRatioMinimum,cfg.actuation.brakingRatioMaximum]))],count,1);
    safetyBound = bound;
    % Only the CLF has a slack column. Physical inequalities remain hard.
    physicalMatrix = [matrix,zeros(numel(bound),1)];
    matrix = [physicalMatrix;zeros(1,planCount),-1];
    bound = [bound;0];
    linearCount = numel(bound);
    root = chol(cruise.matrix);
    trackingError = model.initialEgoState(2:6)-cruise.state(2:6);
    radius = model.initialFrenetErrorBound(2:6);
    contraction = 1-cruise.decayPerHold;
    middle = (contraction+cruise.contraction)/2;
    stateMap = cruise.transition(2:6,2:6);
    inputMap = cruise.transition(2:6,7:8);
    drift = cruise.transition(2:6,:)*[cruise.state;cruise.input;1]-cruise.state(2:6);
    nominalOffset = stateMap*trackingError-inputMap*cruise.input+drift;
    inputRoot = root*inputMap;
    numeric = 100*cfg.solver.constraintTolerance*(1+norm(inputRoot,'fro')*norm(reach(1:2)));
    coneRadius = sqrt(middle)*norm(root*trackingError)+numeric;
    matrix = [matrix;zeros(1,planCount),-1;-inputRoot,zeros(5,planCount-1)];
    bound = [bound;coneRadius;root*nominalOffset];
    disturbance = sqrt(middle)*norm(abs(root)*radius) ...
        +norm(abs(root*stateMap)*radius)+2*numeric;
    clf = struct('cruise',cruise,'initialValue',trackingError.'*cruise.matrix*trackingError, ...
        'decayPerHold',cruise.decayPerHold, ...
        'disturbanceBound',contraction/(contraction-middle)*disturbance^2, ...
        'normDisturbance',disturbance,'youngFactor',contraction/(contraction-middle));
    % Future state performance uses the same cruise state and P certificate.
    maps = reshape(permute(prediction.egoStateMatrix(2:6,:,2:end),[1,3,2]),[],planCount);
    offsets = prediction.egoStateOffset(2:6,2:end)-cruise.state(2:6);
    objectiveMap = kron(eye(count),root)*maps;
    objectiveOffset = reshape(root*offsets,[],1);
    weight = diag(repmat([cfg.clf.frontWheelSteeringAngleWeight;cfg.clf.brakingRatioWeight],count,1));
    trim = repmat(cruise.input,count,1);
    hessian = objectiveMap.'*objectiveMap+weight;
    linear = 2*(objectiveMap.'*objectiveOffset-weight*trim);
    % Keep the current CLF slack units and squared penalty unchanged.
    hessian = 2*blkdiag(hessian,cfg.clf.relaxationWeight);
    layout = struct('planIndex',1:planCount,'planCount',planCount,'horizonSteps',count, ...
        'decisionCount',planCount+1,'relaxationIndex',planCount+1);
    program = struct('P',sparse(hessian),'q',[linear;0],'A',sparse(matrix),'b',bound, ...
        'cones',[0;linearCount;6],'decisionRadius',reach, ...
        'obstacleCbfRowCount',nnz(startsWith(geometry.label,"collision:")), ...
        'geometry',geometry,'terminal',terminal,'completion',completion,'layout',layout, ...
        'physicalMatrix',physicalMatrix,'physicalBound',physicalBound, ...
        'anchorPlan',anchor,'safetyBound',safetyBound,'inheritedFeasibleFamily',inherited, ...
        'cruiseCertificate',cruise);
    if inherited
        program.inheritedWitness = [model.carriedWitness.inputs(:);0];
    end
end

function [prediction,geometry,matrix,physicalBound,bound,terminal,completion,anchor] = localShift(model)
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
    selected = any(matrix~=0,2);
    physicalBound = old.physicalBound-old.physicalMatrix(:,1:2)*executed;
    bound = old.safetyBound-old.physicalMatrix(:,1:2)*executed;
    matrix = matrix(selected,:);physicalBound=physicalBound(selected);bound=bound(selected);
    terminal=carry.terminal;completion=carry.completion;anchor=carry.inputs(:);
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
    keep=[prediction.cells.stage]>=2;
    prediction.cells=prediction.cells(keep);
    for index=1:numel(prediction.cells)
        tube=prediction.cells(index);
        tube.offset=tube.offset+reshape(pagemtimes(tube.map(:,1:2,:),executed),6,[]);
        tube.map=tube.map(:,columns,:);
        tube.endOffset=tube.endOffset+tube.endMap(:,1:2)*executed;
        tube.endMap=tube.endMap(:,columns);
        tube.stage=tube.stage-1;tube.start=tube.start-model.sampleTime;tube.time=tube.time-model.sampleTime;
        prediction.cells(index)=tube;
    end
    geometry=old.geometry;
    selected=geometry.stage>=2;
    geometry.physicalBound=geometry.physicalBound(selected)-geometry.matrix(selected,1:2)*executed;
    geometry.matrix=geometry.matrix(selected,columns);
    geometry.stage=geometry.stage(selected)-1;geometry.label=geometry.label(selected);
    geometry.safety=geometry.safety(selected);
    geometry.frames=geometry.frames(keep);geometry.normals=geometry.normals(keep);
end
