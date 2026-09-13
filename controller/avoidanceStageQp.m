classdef avoidanceStageQp
    %avoidanceStageQp Sparse transcription and named bound updates.

    methods (Static)
        function program = build(qp,prediction,model)
        %avoidanceStageQp Build the sole sparse stage-local conic formulation.
        % Rebuilds retain the original prediction coordinates through explicit context.
            if nargin == 1
                prediction = qp.stageProgram.context.prediction;
                model = qp.stageProgram.context.model;
            end
            program = localLiftedProgram(qp,prediction,model);
            program.context = struct("prediction",prediction,"model",model);
            equalities = program.cones(1);
            mapped = numel(program.inequalityIndices);
            program.rowMap = struct("equality",(1:equalities).', ...
                "inequality",equalities+(1:mapped).', ...
                "violation",equalities+mapped+(1:program.violationCount).', ...
                "budget",equalities+mapped+program.violationCount+1);
            program.inequalityOffset = program.b(program.rowMap.inequality) ...
                -qp.inequalityBound(program.inequalityIndices);

        end

        function program = updateBounds(qp)
        %avoidanceStageQp.updateBounds Update hard RHS values without touching equalities.
        % The retained row reduction is valid for the entire carried-margin interval.
            program = qp.stageProgram;
            program.b(program.rowMap.inequality) = program.inequalityOffset ...
                +qp.inequalityBound(program.inequalityIndices);
        end
    end
end

function program = localLiftedProgram(qp,prediction,model)
%localLiftedProgram Sparse cell-state realization of the same finite SOCP.
% The independent checker still evaluates the condensed physical decisions.
% Every sparse block is assembled from triplet lists in one call; indexed
% assignment into large sparse matrices was the dominant formulation cost.
    physicalCount = qp.layout.decisionCount;
    count = prediction.stageCount;
    cells = prediction.cells;
    cellCount = numel(cells);
    stateCount = 6*(cellCount+1);
    % One nonnegative safety violation per stage follows the auxiliary
    % states. It relaxes only collision and road rows; every other row,
    % including the terminal set, remains hard.
    violationIndex = physicalCount+stateCount+(1:count);
    total = physicalCount+stateCount+count;
    stateIndex = reshape(physicalCount+(1:stateCount),6,[]);
    % Auxiliary states are deviations from the current anchor. Absolute
    % route station can be hundreds of metres while active clearances are
    % micrometres; leaving that translation in equality right-hand sides
    % needlessly degrades the solver's relative feasibility scaling.
    stateCenter = zeros(6,cellCount+1);
    stateCenter(:,1) = model.initialEgoState;
    dynamicsBound = zeros(6*(cellCount+1),1);
    dynamicsRow = cell(cellCount+1,1);dynamicsCol = cell(cellCount+1,1);dynamicsVal = cell(cellCount+1,1);
    dynamicsRow{1} = (1:6).';dynamicsCol{1} = stateIndex(:,1);dynamicsVal{1} = ones(6,1);
    blockRow = cell(cellCount,1);blockCol = cell(cellCount,1);blockVal = cell(cellCount,1);
    localBounds = cell(cellCount,1);
    rowStart = 0;
    for index = 1:cellCount
        tube = cells(index);
        stateCenter(:,index+1) = tube.endMap*model.anchorPlan+tube.endOffset;
        inputIndex = 2*tube.stage-1:2*tube.stage;
        rows = 6*index+(1:6);
        stateMap = tube.localStateMap(:,:,end);
        inputMap = tube.localInputMap(:,:,end);
        [rr,cc] = ndgrid(rows,stateIndex(:,index));
        [ir,ic] = ndgrid(rows,inputIndex);
        dynamicsRow{index+1} = [rows.';rr(:);ir(:)];
        dynamicsCol{index+1} = [stateIndex(:,index+1);cc(:);ic(:)];
        dynamicsVal{index+1} = [ones(6,1);-stateMap(:);-inputMap(:)];
        dynamicsBound(rows) = tube.localOffset(:,end) ...
            +stateMap*stateCenter(:,index)-stateCenter(:,index+1);
        geometry = qp.geometry.local(index);
        rowCount = numel(geometry.bound);
        selected = rowStart+(1:rowCount);
        relaxed = qp.safetyRows(selected);
        localBounds{index} = geometry.bound-geometry.stateMatrix*stateCenter(:,index) ...
            +qp.inequalityBound(selected)-qp.geometry.physicalBound(selected);
        normalRows = (1:rowCount).';
        endpointRows = zeros(0,1);
        if size(geometry.nodeStateRows,3)>1
            % Use the existing endpoint state directly. Substituting its
            % dynamics into every endpoint inequality unnecessarily makes
            % the sparse constraint rows dense in the preceding state.
            endpointState = geometry.nodeStateRows(:,:,end);
            endpointStart = geometry.nodeStartStateRows(:,:,end);
            endpointInput = geometry.nodeInputRows(:,:,end);
            endpointRows = (rowCount-size(endpointState,1)+(1:size(endpointState,1))).';
            normalRows = (1:rowCount-numel(endpointRows)).';
            localBounds{index}(endpointRows) = geometry.bound(endpointRows) ...
                +endpointState*tube.localOffset(:,end) ...
                -endpointState*stateCenter(:,index+1)-endpointStart*stateCenter(:,index) ...
                +qp.inequalityBound(selected(endpointRows))-qp.geometry.physicalBound(selected(endpointRows));
        end
        globalRows = rowStart+(1:rowCount).';
        stateBlock = geometry.stateMatrix(normalRows,:);
        inputBlock = geometry.inputMatrix(normalRows,:);
        [nr,nc] = ndgrid(globalRows(normalRows),stateIndex(:,index));
        [mr,mc] = ndgrid(globalRows(normalRows),inputIndex);
        rowList = cell(6,1);colList = cell(6,1);valList = cell(6,1);
        rowList{1} = nr(:);colList{1} = nc(:);valList{1} = stateBlock(:);
        rowList{2} = mr(:);colList{2} = mc(:);valList{2} = inputBlock(:);
        if ~isempty(endpointRows)
            [er,ec] = ndgrid(globalRows(endpointRows),stateIndex(:,index));
            [fr,fc] = ndgrid(globalRows(endpointRows),stateIndex(:,index+1));
            [gr,gc] = ndgrid(globalRows(endpointRows),inputIndex);
            rowList{3} = er(:);colList{3} = ec(:);valList{3} = endpointStart(:);
            rowList{4} = fr(:);colList{4} = fc(:);valList{4} = endpointState(:);
            rowList{5} = gr(:);colList{5} = gc(:);valList{5} = endpointInput(:);
        end
        relaxedRows = globalRows(relaxed);
        rowList{6} = relaxedRows;
        colList{6} = repmat(violationIndex(tube.stage),numel(relaxedRows),1);
        valList{6} = -ones(numel(relaxedRows),1);
        blockRow{index} = vertcat(rowList{:});blockCol{index} = vertcat(colList{:});blockVal{index} = vertcat(valList{:});
        rowStart = rowStart+rowCount;
    end
    dynamics = sparse(vertcat(dynamicsRow{:}),vertcat(dynamicsCol{:}),vertcat(dynamicsVal{:}),6*(cellCount+1),total);
    localMatrix = sparse(vertcat(blockRow{:}),vertcat(blockCol{:}),vertcat(blockVal{:}),rowStart,total);
    hard = [localMatrix; ...
        sparse(qp.inequalityMatrix(rowStart+1:end,:)),sparse(size(qp.inequalityMatrix,1)-rowStart,total-physicalCount)];
    hardBound = [vertcat(localBounds{:});qp.inequalityBound(rowStart+1:end)];
    % Input and slew rows below remain explicit. Their reachable box can
    % therefore prove geometric rows redundant for every admissible plan,
    % Keep the complete unpruned
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
    % A row omitted at admission must remain redundant for every margin
    % used by either stage, including the maximization LP.
    strongestBound = qp.barrier.baseBound(1:rowStart) ...
        -cfg.encounter.maximumCarriedMargin*qp.barrier.scale(1:rowStart);
    error = 64*(qp.layout.planCount+1)^2*eps*(1+abs(geometryMap)*max(abs(lower),abs(upper)) ...
        +abs(qp.inequalityBound(1:rowStart))+abs(strongestBound));
    retained = maximum+error>strongestBound;
    if any(lower>upper),retained(:) = true;end
    inequalityIndices = [find(retained);(rowStart+1:numel(hardBound)).'];
    hard = hard(inequalityIndices,:);
    hardBound = hardBound(inequalityIndices);
    uniqueRows = localUndominatedRows([hard,sparse(qp.barrier.scale(inequalityIndices))],hardBound);
    inequalityIndices = inequalityIndices(uniqueRows);
    hard = hard(uniqueRows,:);hardBound = hardBound(uniqueRows);
    % Nonnegative violations and their lexicographic budget row. The budget
    % is set by the performance stage to the value-stage optimum.
    nonnegative = sparse(1:count,violationIndex,-1,count,total);
    budget = sparse(ones(1,count),violationIndex,1,1,total);
    hard = [hard;nonnegative;budget];
    hardBound = [hardBound;zeros(count,1);0];
    cones = qp.clf.constraints;
    coneCount = numel(cones);
    coneRow = cell(coneCount,1);coneCol = cell(coneCount,1);coneVal = cell(coneCount,1);
    coneBounds = cell(coneCount,1);
    coneDimensions = 10*ones(coneCount,1);
    quadratics = zeros(6,coneCount);
    firstInterval = false(coneCount,1);
    relaxationIndex = qp.layout.relaxationIndex;
    for index = 1:coneCount
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
            % Rows (local 1..4): -tMap, -square, -tMap with
            % tMap = [-affine at 1:2, +1 at relaxation(1)], square = 2*root at 1:2.
            square = 2*root;
            coneRow{index} = [1;1;1;2;2;3;3;4;4;4];
            coneCol{index} = [1;2;relaxationIndex(1);1;2;1;2;1;2;relaxationIndex(1)];
            coneVal{index} = [affine(1);affine(2);-1;-square(1,1);-square(1,2);-square(2,1);-square(2,2); ...
                affine(1);affine(2);-1];
            coneBounds{index} = [1-constant;zeros(2,1);-1-constant];
            coneDimensions(index) = 4;
            continue;
        end
        point = constraint.pointIndex;
        inputIndex = 2*tube.stage-1:2*tube.stage;
        stateColumns = stateIndex(:,constraint.cellIndex);
        localState = tube.localStateMap(2:6,:,point);
        localInput = tube.localInputMap(2:6,:,point);
        errorOffset = tube.localOffset(2:6,point) ...
            +localState*stateCenter(:,constraint.cellIndex)-qp.clf.referenceStart ...
            -qp.clf.referenceRate*tube.time(point);
        offset = [errorOffset;zeros(2,1);constraint.offset(8)];
        % map (8 x total): rows 1:5 = [localState at states, localInput at
        % inputs], rows 6:7 = identity at inputs, row 8 = 0.
        linearState = constraint.linear(1:5).'*localState;
        linearInput = constraint.linear(1:5).'*localInput+constraint.linear(6:7).';
        squareState = 2*constraint.root(:,1:5)*localState;
        squareInput = 2*constraint.root(:,1:5)*localInput+2*constraint.root(:,6:7);
        % Rows: 1 = -tMap, 2:9 = -2*root*map, 10 = -tMap, where
        % tMap = -linear'*map + e_relaxation(stage).
        [sr,sc] = ndgrid((2:9).',stateColumns);
        [ir,ic] = ndgrid((2:9).',inputIndex);
        coneRow{index} = [ones(6,1);ones(2,1);1;sr(:);ir(:);10*ones(6,1);10*ones(2,1);10];
        coneCol{index} = [stateColumns;inputIndex.';relaxationIndex(tube.stage); ...
            sc(:);ic(:);stateColumns;inputIndex.';relaxationIndex(tube.stage)];
        coneVal{index} = [linearState.';linearInput.';-1;-squareState(:);-squareInput(:); ...
            linearState.';linearInput.';-1];
        tOffset = -constraint.linear.'*offset-constraint.constant;
        coneBounds{index} = [tOffset+1;2*constraint.root*offset;tOffset-1];
    end
    coneIndices = localUndominatedClf(quadratics,firstInterval,lower(1:2),upper(1:2));
    coneDimensions = coneDimensions(coneIndices);
    coneBounds = coneBounds(coneIndices);
    offsets = [0;cumsum(coneDimensions(1:end-1))];
    coneRowsAll = cell(numel(coneIndices),1);
    for slot = 1:numel(coneIndices)
        coneRowsAll{slot} = coneRow{coneIndices(slot)}+offsets(slot);
    end
    coneMatrix = sparse(vertcat(coneRowsAll{:}),vertcat(coneCol{coneIndices}),vertcat(coneVal{coneIndices}), ...
        sum(coneDimensions),total);
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
    operatingInput = repmat(qp.clf.certificate.operatingInput,count,1);
    linear(1:planCount) = -2*h*inputWeight.*operatingInput ...
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
    % The anchor plan with zero state deviation and zero slacks is the
    % reference point from which row generation seeds its working set.
    anchorPoint = zeros(total,1);
    anchorPoint(1:planCount) = model.anchorPlan;
    program = struct("P",triu(hessian),"q",linear, ...
        "A",[dynamics;hard;coneMatrix], ...
        "b",[dynamicsBound;hardBound;vertcat(coneBounds{:})], ...
        "cones",[size(dynamics,1);numel(hardBound);coneDimensions], ...
        "physicalDecisionCount",physicalCount,"stateIndex",stateIndex,"stateCenter",stateCenter, ...
        "inequalityIndices",inequalityIndices, ...
        "violationIndex",violationIndex,"violationCount",count, ...
        "clfConstraintIndices",coneIndices, ...
        "inactiveSlackIndex",zeros(1, 0), ...
        "generatedRowCount",numel(inequalityIndices),"anchorPoint",anchorPoint);
end

function retained = localUndominatedRows(matrix,bound)
% Keep the tightest bound for each exactly identical left side.
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
    endpoint = bound;
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
