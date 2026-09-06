function certificate = readEstimationErrorCertificate(data, kind)
%readEstimationErrorCertificate Validate a current estimator enclosure.
% A published certificate takes precedence over legacy numeric aliases.
% Its timestamp must match the estimate; unavailable radii are never zeroed.

    certificate = [];
    if ~isfield(data, "controllerErrorBound")
        if isfield(data, "relativePositionErrorBound") ...
                || isfield(data, "egoYawErrorBound")
            error("collisionAvoidanceController:missingEstimatorBound", ...
                "Estimator output must include its controllerErrorBound certificate.");
        end
        return;
    end
    raw = data.controllerErrorBound;
    required = ["kind", "time", "bounds", "available", ...
        "source", "futurePredictionIncluded"];
    if ~isstruct(raw) || ~isscalar(raw) || ~all(isfield(raw, required)) ...
            || ~isscalar(string(raw.kind)) || string(raw.kind) ~= kind ...
            || ~isscalar(string(raw.source)) || strlength(string(raw.source)) == 0 ...
            || ~islogical(raw.available) || ~isscalar(raw.available) ...
            || ~isequal(raw.futurePredictionIncluded, false)
        error("collisionAvoidanceController:invalidEstimatorBound", ...
            "The current estimation-error certificate has an invalid schema or scope.");
    end
    if ~raw.available
        error("collisionAvoidanceController:unavailableEstimatorBound", ...
            "The estimator marked its state-time enclosure unavailable.");
    end
    count = 6+2*double(kind == "target-state-v1");
    if ~isnumeric(raw.bounds) || ~isreal(raw.bounds) ...
            || ~isvector(raw.bounds) || numel(raw.bounds) ~= count ...
            || any(~isfinite(raw.bounds)) || any(raw.bounds < 0.0)
        error("collisionAvoidanceController:invalidEstimatorBound", ...
            "Estimator bounds must be finite nonnegative component bounds.");
    end
    if ~isfield(data, "stateTime") || ~localFiniteTime(data.stateTime) ...
            || ~localFiniteTime(raw.time)
        error("collisionAvoidanceController:invalidEstimatorBound", ...
            "Both the estimate and its bound need finite state timestamps.");
    end
    tolerance = 128*eps(max([1.0, abs(data.stateTime), abs(raw.time)]));
    if abs(raw.time-data.stateTime) > tolerance
        error("collisionAvoidanceController:staleEstimatorBound", ...
            "The estimation bound and state timestamps do not match.");
    end
    certificate = raw;
    certificate.bounds = double(raw.bounds(:));
end

function valid = localFiniteTime(value)
    valid = isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end
