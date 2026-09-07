function program = avoidanceStageQp(qp,prediction,model)
%avoidanceStageQp Convert the finite predictive QCQP to sparse Lorentz cones.
% The native convention is A*z+s=b. The first cone is the (possibly empty)
% equality cone, the second is the nonnegative cone, and each remaining
% cone encodes a quadratic upper bound on one CLF control point.
    if nargin == 3
        program = localLiftedProgram(qp,prediction,model);
        return;
    end
    constraints = qp.clf.constraints;
    matrices = cell(numel(constraints), 1);
    bounds = cell(numel(constraints), 1);
    for index = 1:numel(constraints)
        constraint = constraints(index);
        tMap = -constraint.linear.'*constraint.map;
        slackIndex = qp.layout.relaxationIndex(constraint.stage);
        tMap(slackIndex) = tMap(slackIndex)+1;
        tOffset = -constraint.linear.'*constraint.offset-constraint.constant;
        matrices{index} = -[tMap; 2*constraint.root*constraint.map; tMap];
        bounds{index} = [tOffset+1; 2*constraint.root*constraint.offset; tOffset-1];
    end
    program = struct("P", sparse(triu((qp.Hessian+qp.Hessian.')/2)), "q", qp.linear, ...
        "A", sparse([qp.inequalityMatrix; vertcat(matrices{:})]), ...
        "b", [qp.inequalityBound; vertcat(bounds{:})], ...
        "cones", [0; numel(qp.inequalityBound); 10*ones(numel(constraints), 1)], ...
        "physicalDecisionCount", qp.layout.decisionCount);
end

function program = localLiftedProgram(qp,prediction,model)
%localLiftedProgram Sparse cell-state realization of the same finite SOCP.
% The independent checker still evaluates the condensed physical decisions.
    physicalCount = qp.layout.decisionCount;
    count = prediction.stageCount;
    cells = prediction.cells;
    cellCount = numel(cells);
    total = physicalCount+6*(cellCount+1);
    stateIndex = reshape(physicalCount+(1:6*(cellCount+1)),6,[]);
    dynamics = spalloc(6*(cellCount+1),total,60*cellCount+6);
    dynamics(1:6,stateIndex(:,1)) = speye(6);
    dynamicsBound = zeros(6*(cellCount+1),1);
    dynamicsBound(1:6) = model.initialEgoState;
    localRows = cell(cellCount,1);
    localBounds = cell(cellCount,1);
    rowStart = 0;
    for index = 1:cellCount
        tube = cells(index);
        inputIndex = 2*tube.stage-1:2*tube.stage;
        rows = 6*index+(1:6);
        dynamics(rows,stateIndex(:,index+1)) = speye(6);
        dynamics(rows,stateIndex(:,index)) = -tube.localStateMap(:,:,end);
        dynamics(rows,inputIndex) = -tube.localInputMap(:,:,end);
        dynamicsBound(rows) = tube.localOffset(:,end);
        geometry = qp.geometry.local(index);
        rowCount = numel(geometry.bound);
        block = spalloc(rowCount,total,8*rowCount);
        block(:,stateIndex(:,index)) = geometry.stateMatrix;
        block(:,inputIndex) = geometry.inputMatrix;
        selected = rowStart+(1:rowCount);
        localBounds{index} = geometry.bound+qp.inequalityBound(selected)-qp.geometry.physicalBound(selected);
        localRows{index} = block;
        rowStart = rowStart+rowCount;
    end
    hard = [vertcat(localRows{:}); ...
        sparse(qp.inequalityMatrix(rowStart+1:end,:)),sparse(size(qp.inequalityMatrix,1)-rowStart,total-physicalCount)];
    hardBound = [vertcat(localBounds{:});qp.inequalityBound(rowStart+1:end)];
    cones = qp.clf.constraints;
    coneRows = cell(numel(cones),1);
    coneBounds = cell(numel(cones),1);
    for index = 1:numel(cones)
        constraint = cones(index);
        tube = cells(constraint.cellIndex);
        point = constraint.pointIndex;
        inputIndex = 2*tube.stage-1:2*tube.stage;
        map = sparse(8,total);
        map(1:5,stateIndex(:,constraint.cellIndex)) = tube.localStateMap(2:6,:,point);
        map(1:5,inputIndex) = tube.localInputMap(2:6,:,point);
        map(6:7,inputIndex) = eye(2);
        errorOffset = tube.localOffset(2:6,point)-qp.clf.referenceStart ...
            -qp.clf.referenceRate*tube.time(point);
        offset = [errorOffset;zeros(2,1);constraint.offset(8)];
        tMap = -constraint.linear.'*map;
        tMap(qp.layout.relaxationIndex(tube.stage)) = tMap(qp.layout.relaxationIndex(tube.stage))+1;
        tOffset = -constraint.linear.'*offset-constraint.constant;
        coneRows{index} = -[tMap;2*constraint.root*map;tMap];
        coneBounds{index} = [tOffset+1;2*constraint.root*offset;tOffset-1];
    end
    cfg = model.cfg;
    h = model.sampleTime;
    planCount = qp.layout.planCount;
    inputWeight = repmat([cfg.clf.frontWheelSteeringAngleWeight;cfg.clf.brakingRatioWeight],count,1);
    difference = speye(planCount)-sparse(3:planCount,1:planCount-2,1,planCount,planCount);
    smoothWeight = cfg.encounter.inputRateWeight/h;
    hessian = sparse(total,total);
    hessian(1:planCount,1:planCount) = 2*h*spdiags(inputWeight,0,planCount,planCount) ...
        +2*smoothWeight*(difference.'*difference);
    slackIndex = qp.layout.relaxationIndex;
    hessian(slackIndex,slackIndex) = 2*h*cfg.clf.relaxationWeight*speye(count);
    linear = zeros(total,1);
    linear(1:planCount) = -2*h*inputWeight.*prediction.referencePlan ...
        -2*smoothWeight*difference.'*[model.previousInput;zeros(planCount-2,1)];
    scales = [cfg.clf.lateralPositionErrorScale;cfg.clf.headingErrorScale;cfg.clf.speedErrorScale; ...
        cfg.clf.lateralVelocityErrorScale;cfg.clf.yawRateErrorScale];
    weight = diag(1./scales.^2);
    firstCell = [true;diff([cells.stage].')~=0];
    for index = find(firstCell).'
        stage = cells(index).stage;
        rows = stateIndex(2:6,index);
        reference = qp.clf.referenceStart+qp.clf.referenceRate*(stage-1)*h;
        hessian(rows,rows) = 2*h*weight;
        linear(rows) = -2*h*weight*reference;
    end
    program = struct("P",triu(hessian),"q",linear, ...
        "A",[dynamics;hard;vertcat(coneRows{:})], ...
        "b",[dynamicsBound;hardBound;vertcat(coneBounds{:})], ...
        "cones",[size(dynamics,1);numel(hardBound);10*ones(numel(cones),1)], ...
        "physicalDecisionCount",physicalCount,"stateIndex",stateIndex);
end
