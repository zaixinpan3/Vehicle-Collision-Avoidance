function program = avoidanceStageSocp(qp, prediction, rows, bound, stateNode, inputStage)
%avoidanceStageSocp Keep dynamics and constraints local in the conic solve.
% The additional variables are x_1,...,x_M. Eliminating their affine
% dynamics recovers the condensed program used by independent acceptance.
% The quadratic input objective is retained directly; the sole SOC is the
% exact first-step CLF. No horizon, constraint or objective is approximated.

    stages = prediction.stageCount;
    controls = qp.layout.planCount;
    physicalCount = qp.layout.decisionCount;
    count = physicalCount+6*stages;
    rowCount = size(rows, 1);
    initial = stateNode == 0;
    bound(initial) = bound(initial)-rows(initial, 1:6)*prediction.egoStateOffset(:, 1);
    [stateRow, component, value] = find(rows(:, 1:6));
    keep = stateNode(stateRow) > 0;
    stateRow = stateRow(keep);
    component = component(keep);
    value = value(keep);
    stateColumn = physicalCount+6*(stateNode(stateRow)-1)+component;
    [inputRow, component, inputValue] = find(rows(:, 7:8));
    inputColumn = 2*(inputStage(inputRow)-1)+component;
    inequality = sparse([stateRow; inputRow], [stateColumn; inputColumn], ...
        [value; inputValue], rowCount, count);

    % Triplet assembly avoids repeatedly reallocating a sparse matrix.
    dynamicRows = (1:6*stages).';
    nextColumns = physicalCount+dynamicRows;
    [index, stage, stateValue] = find(reshape(-prediction.stageMatrixA, 36, []));
    keep = stage > 1;
    index = index(keep);
    stage = stage(keep);
    stateValue = stateValue(keep);
    previousRows = 6*(stage-1)+mod(index-1, 6)+1;
    previousColumns = physicalCount+6*(stage-2)+floor((index-1)/6)+1;
    [index, stage, inputValue] = find(reshape(-prediction.stageMatrixB, 12, []));
    inputRows = 6*(stage-1)+mod(index-1, 6)+1;
    inputColumns = 2*(stage-1)+floor((index-1)/6)+1;
    terminalColumns = [physicalCount+6*(stages-1)+(4:6), controls-1:controls].';
    equality = sparse([dynamicRows; previousRows; inputRows; 6*stages+(1:5).'], ...
        [nextColumns; previousColumns; inputColumns; terminalColumns], ...
        [ones(6*stages, 1); stateValue; inputValue; ones(5, 1)], 6*stages+5, count);
    right = prediction.stageAffine;
    right(:, 1) = right(:, 1)+prediction.stageMatrixA(:, :, 1)*prediction.egoStateOffset(:, 1);
    right = [right(:); zeros(3, 1); qp.terminalInput];

    % Fixed final inputs are already equality rows. Avoid duplicating them
    % as zero-interior inequality slacks in the native conic solver.
    fixed = qp.lowerBound == qp.upperBound;
    upper = find(isfinite(qp.upperBound) & ~fixed);
    lower = find(isfinite(qp.lowerBound) & ~fixed);
    selectors = sparse((1:numel(upper)+numel(lower)).', [upper; lower], ...
        [ones(numel(upper), 1); -ones(numel(lower), 1)], ...
        numel(upper)+numel(lower), count);
    inequality = [inequality; selectors];
    bound = [bound; qp.upperBound(upper); -qp.lowerBound(lower)];

    clf = qp.clf;
    factor = chol(clf.lyapunovMatrix);
    error = clf.errorOffset(:, 1);
    available = error.'*(clf.lyapunovMatrix-clf.decreaseMatrix)*error;
    referenceOffset = clf.errorOffset(:, 2)-prediction.egoStateOffset(2:6, 2);
    cone = sparse(7, count);
    cone([1, 7], physicalCount) = -1.0;
    cone(2:6, physicalCount+(2:6)) = -2.0*factor;
    coneBound = [available+1.0; 2.0*factor*referenceOffset; available-1.0];
    hessian = blkdiag(sparse(triu(qp.Hessian)), sparse(6*stages, 6*stages));
    program = struct("P", hessian, "q", [qp.linear; zeros(6*stages, 1)], ...
        "A", [equality; inequality; cone], "b", [right; bound; coneBound], ...
        "cones", [size(equality, 1); size(inequality, 1); 7], ...
        "physicalDecisionCount", physicalCount, "stateIndex", physicalCount+(1:6*stages));
end
