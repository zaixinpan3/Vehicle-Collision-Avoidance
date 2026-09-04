function certificate = certifySweptRectangleIntervals( ...
        egoPose, targetPose, halfDimensions, clearance, options)
% certifySweptRectangleIntervals Certify rectangle separation between nodes.
%
% egoPose and targetPose are 3-by-K arrays [x; y; yaw]. clearance is a
% scalar or one value for each of the K-1 intervals. Between adjacent
% nodes each centre and unwrapped yaw is interpreted as affine in time.
% The signed configuration distance is Lipschitz with constant
%
%   ||Delta p_E|| + rho_E |Delta psi_E|
%   + ||Delta p_T|| + rho_T |Delta psi_T|
%
% over one normalized interval, where rho is the footprint circumradius.
% Adaptive midpoint subdivision therefore certifies a complete interval
% whenever its midpoint distance exceeds the required clearance by the
% Lipschitz radius of that subinterval.  An interval that remains
% inconclusive at maxDepth is rejected conservatively; sampled safety is
% never promoted to a certificate.

    if nargin < 5 || isempty(options)
        options = struct();
    end
    options = localOptions(options);
    localValidateInputs(egoPose, targetPose, halfDimensions, clearance);
    intervalCount = size(egoPose, 2)-1;
    clearance = double(clearance(:).');
    if isscalar(clearance)
        clearance = repmat(clearance, 1, intervalCount);
    end
    intervalCertified = false(1, intervalCount);
    intervalMargin = inf(1, intervalCount);
    intervalDepth = zeros(1, intervalCount);
    failureReason = strings(1, intervalCount);

    egoYaw = unwrap(double(egoPose(3, :)));
    targetYaw = unwrap(double(targetPose(3, :)));
    egoPose = double(egoPose);
    targetPose = double(targetPose);
    egoPose(3, :) = egoYaw;
    targetPose(3, :) = targetYaw;
    for intervalIdx = 1:intervalCount
        segment = struct( ...
            "egoStart", egoPose(:, intervalIdx), ...
            "egoEnd", egoPose(:, intervalIdx+1), ...
            "targetStart", targetPose(:, intervalIdx), ...
            "targetEnd", targetPose(:, intervalIdx+1));
        [intervalCertified(intervalIdx), intervalMargin(intervalIdx), ...
            intervalDepth(intervalIdx), failureReason(intervalIdx)] = ...
            localCertifySegment(segment, halfDimensions, ...
                clearance(intervalIdx), options);
    end

    certificate = struct( ...
        "certified", all(intervalCertified), ...
        "intervalCertified", intervalCertified, ...
        "intervalMargin", intervalMargin, ...
        "minimumMargin", min([inf, intervalMargin]), ...
        "maximumDepth", max([0, intervalDepth]), ...
        "failureReason", failureReason);
end

function [certified, minimumMargin, maximumDepth, reason] = ...
        localCertifySegment(segment, halfDimensions, clearance, options)
    egoRadius = hypot(halfDimensions(1), halfDimensions(2));
    targetRadius = hypot(halfDimensions(3), halfDimensions(4));
    lipschitz = norm(segment.egoEnd(1:2)-segment.egoStart(1:2)) ...
        + egoRadius*abs(segment.egoEnd(3)-segment.egoStart(3)) ...
        + norm(segment.targetEnd(1:2)-segment.targetStart(1:2)) ...
        + targetRadius*abs(segment.targetEnd(3)-segment.targetStart(3));
    queue = [0.0, 1.0, 0.0];
    certified = true;
    minimumMargin = inf;
    maximumDepth = 0;
    reason = "";
    while ~isempty(queue)
        item = queue(end, :);
        queue(end, :) = [];
        low = item(1);
        high = item(2);
        depth = item(3);
        maximumDepth = max(maximumDepth, depth);
        middle = 0.5*(low+high);
        parameters = [low, middle, high];
        margins = zeros(size(parameters));
        for pointIdx = 1:numel(parameters)
            parameter = parameters(pointIdx);
            ego = (1.0-parameter)*segment.egoStart ...
                + parameter*segment.egoEnd;
            target = (1.0-parameter)*segment.targetStart ...
                + parameter*segment.targetEnd;
            distance = rectangleConfigurationDistance( ...
                ego(1:2), ego(3), target(1:2), target(3), halfDimensions);
            margins(pointIdx) = distance-clearance;
        end
        minimumMargin = min(minimumMargin, min(margins));
        if any(margins < -options.distanceTolerance)
            certified = false;
            reason = "sampledCollision";
            return;
        end
        radius = 0.5*lipschitz*(high-low);
        if margins(2) >= radius+options.distanceTolerance
            continue;
        end
        if depth >= options.maxDepth
            certified = false;
            reason = "unresolvedLipschitzBound";
            return;
        end
        nextDepth = depth+1.0;
        queue(end+1, :) = [low, middle, nextDepth]; %#ok<AGROW>
        queue(end+1, :) = [middle, high, nextDepth]; %#ok<AGROW>
    end
end

function options = localOptions(options)
    if ~isfield(options, "maxDepth")
        options.maxDepth = 12;
    end
    if ~isfield(options, "distanceTolerance")
        options.distanceTolerance = 1.0e-8;
    end
    if ~isnumeric(options.maxDepth) || ~isscalar(options.maxDepth) ...
            || options.maxDepth < 0.0 ...
            || options.maxDepth ~= round(options.maxDepth)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "The swept-collision maxDepth must be a nonnegative integer.");
    end
    if ~isnumeric(options.distanceTolerance) ...
            || ~isscalar(options.distanceTolerance) ...
            || ~isfinite(options.distanceTolerance) ...
            || options.distanceTolerance < 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "The swept-collision distanceTolerance must be nonnegative.");
    end
end

function localValidateInputs(egoPose, targetPose, halfDimensions, clearance)
    if ~isnumeric(egoPose) || ~isreal(egoPose) ...
            || size(egoPose, 1) ~= 3 || size(egoPose, 2) < 2 ...
            || any(~isfinite(egoPose), "all")
        error("collisionAvoidanceController:invalidInput", ...
            "egoPose must be a finite real 3-by-K array with K at least 2.");
    end
    if ~isnumeric(targetPose) || ~isreal(targetPose) ...
            || ~isequal(size(targetPose), size(egoPose)) ...
            || any(~isfinite(targetPose), "all")
        error("collisionAvoidanceController:invalidInput", ...
            "targetPose must be finite, real, and the same size as egoPose.");
    end
    if ~isnumeric(halfDimensions) || ~isreal(halfDimensions) ...
            || numel(halfDimensions) ~= 4 ...
            || any(~isfinite(halfDimensions)) ...
            || any(halfDimensions <= 0.0)
        error("collisionAvoidanceController:invalidInput", ...
            "halfDimensions must contain four positive finite values.");
    end
    intervalCount = size(egoPose, 2)-1;
    if ~isnumeric(clearance) || ~isreal(clearance) ...
            || ~(isscalar(clearance) || numel(clearance) == intervalCount) ...
            || any(~isfinite(clearance)) || any(clearance < 0.0)
        error("collisionAvoidanceController:invalidInput", ...
            "clearance must be a nonnegative finite scalar or contain " ...
            + "one value per interval.");
    end
end
