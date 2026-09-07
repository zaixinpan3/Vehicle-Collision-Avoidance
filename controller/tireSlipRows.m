function [matrix, offset, stageRows, stageOffset] = tireSlipRows(prediction, model)
%tireSlipRows Hard slip-angle model domains, without axle friction limits.
% Coefficients act on [s,d,ePsi,vx,vy,r,deltaF,beta]. Each robust row charges
% the support of the same state box used by prediction and geometry.
    cfg = model.cfg;
    limit = double(cfg.model.slipAngleMaximum(:));
    if isscalar(limit)
        limit = repmat(limit, 2, 1);
    end
    if numel(limit) ~= 2 || ~isreal(limit) || any(~isfinite(limit)) ...
            || any(limit <= 0.0) || any(limit >= pi/2)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "model.slipAngleMaximum must contain positive limits below pi/2.");
    end
    stageCount = prediction.stageCount;
    controlCount = size(prediction.egoStateMatrix, 2);
    stageRows = zeros(4, 8, stageCount);
    stageOffset = -ones(4, stageCount);
    speed = max(prediction.scheduleSpeedProfile(1:stageCount), cfg.model.scheduleSpeedFloor);
    inverseSpeed = reshape(1.0./speed, 1, 1, []);
    signs = [1.0; -1.0; 1.0; -1.0]./repelem(limit, 2);
    stageRows(:, 5, :) = signs.*inverseSpeed;
    stageRows(:, 6, :) = signs.*[cfg.vehicle.lf; cfg.vehicle.lf; ...
        -cfg.vehicle.lr; -cfg.vehicle.lr].*inverseSpeed;
    stageRows(1:2, 7, :) = -signs(1:2).*ones(1, 1, stageCount);
    if isfield(prediction, "egoStateErrorBound")
        support = pagemtimes(abs(stageRows(:, 1:6, :)), ...
            reshape(prediction.egoStateErrorBound(:, 1:stageCount), 6, 1, []));
        stageOffset = stageOffset+reshape(support, 4, stageCount);
    end
    mapped = pagemtimes(stageRows(:, 1:6, :), ...
        prediction.egoStateMatrix(:, :, 1:stageCount));
    for stageIdx = 1:stageCount
        inputRange = 2*stageIdx-1:2*stageIdx;
        mapped(:, inputRange, stageIdx) = mapped(:, inputRange, stageIdx) ...
            + stageRows(:, 7:8, stageIdx);
    end
    matrix = reshape(permute(mapped, [1, 3, 2]), [], controlCount);
    mappedOffset = pagemtimes(stageRows(:, 1:6, :), ...
        reshape(prediction.egoStateOffset(:, 1:stageCount), 6, 1, []));
    offset = reshape(reshape(mappedOffset, 4, [])+stageOffset, [], 1);
end
