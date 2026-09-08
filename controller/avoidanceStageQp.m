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
    % Auxiliary states are deviations from the current anchor. Absolute
    % route station can be hundreds of metres while active clearances are
    % micrometres; leaving that translation in equality right-hand sides
    % needlessly degrades the solver's relative feasibility scaling.
    stateCenter = zeros(6,cellCount+1);
    stateCenter(:,1) = model.initialEgoState;
    dynamics = spalloc(6*(cellCount+1),total,60*cellCount+6);
    dynamics(1:6,stateIndex(:,1)) = speye(6);
    dynamicsBound = zeros(6*(cellCount+1),1);
    localRows = cell(cellCount,1);
    localBounds = cell(cellCount,1);
    rowStart = 0;
    for index = 1:cellCount
        tube = cells(index);
        stateCenter(:,index+1) = tube.endMap*model.anchorPlan+tube.endOffset;
        inputIndex = 2*tube.stage-1:2*tube.stage;
        rows = 6*index+(1:6);
        dynamics(rows,stateIndex(:,index+1)) = speye(6);
        dynamics(rows,stateIndex(:,index)) = -tube.localStateMap(:,:,end);
        dynamics(rows,inputIndex) = -tube.localInputMap(:,:,end);
        dynamicsBound(rows) = tube.localOffset(:,end) ...
            +tube.localStateMap(:,:,end)*stateCenter(:,index)-stateCenter(:,index+1);
        geometry = qp.geometry.local(index);
        rowCount = numel(geometry.bound);
        block = spalloc(rowCount,total,8*rowCount);
        block(:,stateIndex(:,index)) = geometry.stateMatrix;
        block(:,inputIndex) = geometry.inputMatrix;
        selected = rowStart+(1:rowCount);
        localBounds{index} = geometry.bound-geometry.stateMatrix*stateCenter(:,index) ...
            +qp.inequalityBound(selected)-qp.geometry.physicalBound(selected);
        if size(geometry.nodeStateRows,3)>1
            % Use the existing endpoint state directly. Substituting its
            % dynamics into every endpoint inequality unnecessarily makes
            % the sparse constraint rows dense in the preceding state.
            endpointState = geometry.nodeStateRows(:,:,end);
            endpointStart = geometry.nodeStartStateRows(:,:,end);
            endpointInput = geometry.nodeInputRows(:,:,end);
            endpointRows = rowCount-size(endpointState,1)+(1:size(endpointState,1));
            block(endpointRows,:) = 0;
            block(endpointRows,stateIndex(:,index)) = endpointStart;
            block(endpointRows,stateIndex(:,index+1)) = endpointState;
            block(endpointRows,inputIndex) = endpointInput;
            localBounds{index}(endpointRows) = geometry.bound(endpointRows) ...
                +endpointState*tube.localOffset(:,end) ...
                -endpointState*stateCenter(:,index+1)-endpointStart*stateCenter(:,index) ...
                +qp.inequalityBound(selected(endpointRows))-qp.geometry.physicalBound(selected(endpointRows));
        end
        localRows{index} = block;
        rowStart = rowStart+rowCount;
    end
    hard = [vertcat(localRows{:}); ...
        sparse(qp.inequalityMatrix(rowStart+1:end,:)),sparse(size(qp.inequalityMatrix,1)-rowStart,total-physicalCount)];
    hardBound = [vertcat(localBounds{:});qp.inequalityBound(rowStart+1:end)];
    % Input and slew rows below remain explicit. Their reachable box can
    % therefore prove geometric rows redundant for every admissible plan,
    % including the maximum optional reserve. Keep the complete unpruned
    % rows in qp for independent post-solve certification.
    cfg = model.cfg;
    stages = reshape(repelem(1:count,2),[],1);
    rates = repmat([cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum],count,1);
    prior = repmat(model.previousInput,count,1);
    lower = max(qp.lowerBound(1:qp.layout.planCount),prior-model.sampleTime*stages.*rates);
    upper = min(qp.upperBound(1:qp.layout.planCount),prior+model.sampleTime*stages.*rates);
    geometryMap = qp.inequalityMatrix(1:rowStart,1:qp.layout.planCount);
    maximum = max(geometryMap,0)*upper+min(geometryMap,0)*lower;
    slewSupport = zeros(rowStart,1);
    for input = 1:2
        columns = input:2:qp.layout.planCount;
        slewSupport = slewSupport+localSlewSupport(geometryMap(:,columns),lower(columns),upper(columns), ...
            model.previousInput(input),model.sampleTime*rates(input));
    end
    maximum = min(maximum,slewSupport);
    error = 64*(qp.layout.planCount+1)^2*eps*(1+abs(geometryMap)*max(abs(lower),abs(upper)) ...
        +abs(qp.inequalityBound(1:rowStart)));
    retained = maximum+error>qp.inequalityBound(1:rowStart) ...
        -qp.reserveFractionMaximum*qp.anticipationReserve(1:rowStart);
    if any(lower>upper),retained(:) = true;end
    inequalityIndices = [find(retained);(rowStart+1:numel(hardBound)).'];
    hard = hard(inequalityIndices,:);
    hardBound = hardBound(inequalityIndices);
    uniqueRows = localUndominatedRows(hard,hardBound, ...
        qp.anticipationReserve(inequalityIndices),qp.reserveFractionMaximum);
    inequalityIndices = inequalityIndices(uniqueRows);
    hard = hard(uniqueRows,:);hardBound = hardBound(uniqueRows);
    cones = qp.clf.constraints;
    coneRows = cell(numel(cones),1);
    coneBounds = cell(numel(cones),1);
    coneDimensions = 10*ones(numel(cones),1);
    quadratics = zeros(6,numel(cones));
    firstInterval = false(numel(cones),1);
    for index = 1:numel(cones)
        constraint = cones(index);
        tube = cells(constraint.cellIndex);
        if tube.stage==1
            % The initial state is fixed: every first-interval CLF
            % quadratic depends on only the two held inputs. A thin QR
            % preserves its quadratic exactly while reducing the Lorentz
            % cone from ten coordinates to four.
            inputMap = constraint.map(:,1:2);
            weightedMap = constraint.root*inputMap;
            weightedOffset = constraint.root*constraint.offset;
            [~,root] = qr(weightedMap,0);
            affine = 2*weightedOffset.'*weightedMap+constraint.linear.'*inputMap;
            constant = weightedOffset.'*weightedOffset ...
                +constraint.linear.'*constraint.offset+constraint.constant;
            hessian = weightedMap.'*weightedMap;
            quadratics(:,index) = [hessian(1,1);2*hessian(1,2);hessian(2,2);affine.';constant];
            firstInterval(index) = true;
            tMap = sparse(1,total);tMap(1:2) = -affine;
            tMap(qp.layout.relaxationIndex(1)) = 1;
            square = sparse(2,total);square(:,1:2) = 2*root;
            coneRows{index} = -[tMap;square;tMap];
            coneBounds{index} = [1-constant;zeros(2,1);-1-constant];
            coneDimensions(index) = 4;
            continue;
        end
        point = constraint.pointIndex;
        inputIndex = 2*tube.stage-1:2*tube.stage;
        map = sparse(8,total);
        map(1:5,stateIndex(:,constraint.cellIndex)) = tube.localStateMap(2:6,:,point);
        map(1:5,inputIndex) = tube.localInputMap(2:6,:,point);
        map(6:7,inputIndex) = eye(2);
        errorOffset = tube.localOffset(2:6,point) ...
            +tube.localStateMap(2:6,:,point)*stateCenter(:,constraint.cellIndex)-qp.clf.referenceStart ...
            -qp.clf.referenceRate*tube.time(point);
        offset = [errorOffset;zeros(2,1);constraint.offset(8)];
        tMap = -constraint.linear.'*map;
        tMap(qp.layout.relaxationIndex(tube.stage)) = tMap(qp.layout.relaxationIndex(tube.stage))+1;
        tOffset = -constraint.linear.'*offset-constraint.constant;
        coneRows{index} = -[tMap;2*constraint.root*map;tMap];
        coneBounds{index} = [tOffset+1;2*constraint.root*offset;tOffset-1];
    end
    coneIndices = localUndominatedClf(quadratics,firstInterval,lower(1:2),upper(1:2));
    coneRows = coneRows(coneIndices);coneBounds = coneBounds(coneIndices);
    coneDimensions = coneDimensions(coneIndices);
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
        linear(rows) = 2*h*weight*(stateCenter(2:6,index)-reference);
    end
    program = struct("P",triu(hessian),"q",linear, ...
        "A",[dynamics;hard;vertcat(coneRows{:})], ...
        "b",[dynamicsBound;hardBound;vertcat(coneBounds{:})], ...
        "cones",[size(dynamics,1);numel(hardBound);coneDimensions], ...
        "physicalDecisionCount",physicalCount,"stateIndex",stateIndex,"stateCenter",stateCenter, ...
        "inequalityIndices",inequalityIndices, ...
        "clfConstraintIndices",coneIndices, ...
        "inactiveSlackIndex",qp.layout.relaxationIndex(min(count,cfg.controller.certifiedSteps)+1:end));
end

function retained = localUndominatedRows(matrix,bound,reserve,maximumFraction)
% Identical left sides need only bounds not dominated over the reserve range.
% Comparing both endpoints suffices because each right side is affine in
% the common allocation fraction. Keep the complete rows in the outer qp.
    count = numel(bound);
    % A sparse row is identified exactly by its ordered nonzero columns
    % and values. Compare that compact representation instead of hundreds
    % of identically zero state columns; no quantization or hash is used.
    [column,row,value] = find(matrix.');
    nonzeros = accumarray(row,1,[count,1]);
    width = max([nonzeros;0]);
    signature = zeros(count,2*width+1);
    signature(:,1) = nonzeros;
    ordinal = (1:numel(value)).'-repelem(cumsum(nonzeros)-nonzeros,nonzeros);
    signature(sub2ind(size(signature),row,2*ordinal)) = column;
    signature(sub2ind(size(signature),row,2*ordinal+1)) = value;
    [~,~,group] = unique(signature,'rows');
    endpoint = bound-maximumFraction*reserve;
    ordered = sortrows([group,bound,endpoint,(1:count).'],[1,2,3]);
    first = [true;diff(ordered(:,1))~=0];
    representatives = ordered(first,4);
    candidate = representatives(group);
    guard = 128*eps*(1+abs(bound)+abs(endpoint)+abs(bound(candidate))+abs(endpoint(candidate)));
    dominated = bound>bound(candidate)+guard & endpoint>endpoint(candidate)+guard;
    identical = bound==bound(candidate) & endpoint==endpoint(candidate);
    retained = ~dominated & ~identical;
    retained(representatives) = true;
end

function indices = localUndominatedClf(coefficients,firstInterval,lower,upper)
% A common slack needs only quadratics not dominated over the admitted box.
% The center/radius expansion bounds a quadratic difference uniformly; this
% screens cones without changing the maximum required CLF relaxation.
    selected = find(firstInterval);
    retained = true(size(firstInterval));
    if numel(selected)<2 || any(lower>upper),indices = find(retained);return;end
    center = (lower+upper)/2;radius = (upper-lower)/2;
    basis = [center(1)^2;center(1)*center(2);center(2)^2;center;1];
    [~,dominant] = max(coefficients(:,selected).'*basis);
    dominant = selected(dominant);
    difference = coefficients(:,dominant)-coefficients(:,selected);
    gradient = [2*center(1)*difference(1,:)+center(2)*difference(2,:)+difference(4,:); ...
        center(1)*difference(2,:)+2*center(2)*difference(3,:)+difference(5,:)];
    minimum = difference.'*basis-abs(gradient).'*radius ...
        -abs(difference(1:3,:)).'*[radius(1)^2;radius(1)*radius(2);radius(2)^2];
    magnitude = max(abs(lower),abs(upper));
    basisBound = [magnitude(1)^2;prod(magnitude);magnitude(2)^2;magnitude;1];
    guard = 4096*eps*(1+(abs(coefficients(:,dominant))+abs(coefficients(:,selected))).'*basisBound);
    retained(selected(minimum>guard)) = false;
    retained(dominant) = true;
    indices = find(retained);
end

function support = localSlewSupport(coefficients,lower,upper,previous,increment)
% Dual-feasible support bounds from every anchor along a rate-limited chain.
% Expand all inputs about one anchor and bound the intervening increments;
% each resulting support is valid, hence their minimum is also valid.
    support = max(coefficients,0)*upper+min(coefficients,0)*lower;
    if ~isfinite(increment),return;end
    count = size(coefficients,2);
    prefix = cumsum(coefficients,2);
    suffix = flip(cumsum(flip(coefficients,2),2),2);
    total = prefix(:,end);
    before = [zeros(size(total)),cumsum(abs(prefix(:,1:count-1)),2)];
    after = flip(cumsum(flip([abs(suffix(:,2:count)),zeros(size(total))],2),2),2);
    anchored = max(total,0)*upper.'+min(total,0)*lower.'+increment*(before+after);
    initial = previous*total+increment*sum(abs(suffix),2);
    support = min([support,initial,anchored],[],2);
end
