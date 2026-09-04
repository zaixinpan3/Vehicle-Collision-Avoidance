function egoState = ltvBicycleRollout( ...
        stageMatrixA, stageMatrixB, stageAffine, initialState, ...
        inputPlan)
% ltvBicycleRollout Integrate the scheduled LTV stages over an input plan.
%
% x_{k+1} = A_k x_k + B_k u_k + c_k, evaluated stage by stage: the
% single place a state trajectory is produced from an input plan (the
% nominal rollout stage 1 and the CLF rows are evaluated on), O(N)
% where a condensed-map readout would be O(N^2).

    horizonSteps = size(stageMatrixA, 3);
    egoState = zeros(6, horizonSteps+1);
    egoState(:, 1) = initialState(:);
    for stageIdx = 1:horizonSteps
        egoState(:, stageIdx+1) = ...
            stageMatrixA(:, :, stageIdx)*egoState(:, stageIdx) ...
            + stageMatrixB(:, :, stageIdx)*inputPlan(:, stageIdx) ...
            + stageAffine(:, stageIdx);
    end
end
