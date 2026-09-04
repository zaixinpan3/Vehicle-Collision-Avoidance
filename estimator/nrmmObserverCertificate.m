function certificate = nrmmObserverCertificate(stage)
% nrmmObserverCertificate Construct the triangular core ISS certificate.
%
% For chi = [|ePsi|; Wv; WT], the comparison dynamics have the positive
% lower-triangular form D+chi <= A*chi+d. Positive weights are selected by
% backward substitution so that w'*A = -[1,1,1]. The copositive function
% V = w'*chi therefore satisfies
%
%   D+V <= -(1/max(w))*V + w'*d.
%
% This explicit certificate replaces a generic Lyapunov-equation solve and
% displays every feedforward coupling directly.

    yawDecayRate = localPositive(stage, "yawDecayRate");
    velocityDecayRate = localPositive(stage, "velocityDecayRate");
    targetDecayRate = localPositive(stage, "targetDecayRate");
    velocityYawCoupling = localNonnegative( ...
        stage, "velocityYawCoupling");
    targetVelocityCoupling = localNonnegative( ...
        stage, "targetVelocityCoupling");
    input = localInput(stage);

    comparisonMatrix = [ ...
        -yawDecayRate, 0.0, 0.0; ...
        velocityYawCoupling, -velocityDecayRate, 0.0; ...
        0.0, targetVelocityCoupling, -targetDecayRate];
    targetWeight = 1.0/targetDecayRate;
    velocityWeight = ...
        (1.0+targetVelocityCoupling*targetWeight)/velocityDecayRate;
    yawWeight = ...
        (1.0+velocityYawCoupling*velocityWeight)/yawDecayRate;
    weights = [yawWeight; velocityWeight; targetWeight];
    weightedMatrix = weights.'*comparisonMatrix;
    residual = weightedMatrix+ones(1, 3);
    tolerance = 256.0*eps(max([1.0; abs(weights); ...
        abs(comparisonMatrix(:))]));
    if max(abs(residual), [], "all") > tolerance
        error("nrmmObserverCertificate:invalidWeights", ...
            "The backward weights must satisfy w'*A = -ones(1,3).");
    end

    certificate = struct( ...
        "type", "linear-copositive", ...
        "stateOrder", ["yaw"; "bodyVelocity"; "target"], ...
        "matrix", comparisonMatrix, ...
        "input", input, ...
        "weights", weights, ...
        "weightedMatrix", weightedMatrix, ...
        "identityResidual", residual, ...
        "decayRate", 1.0/max(weights), ...
        "inputProjection", weights.'*input, ...
        "weightMinimum", min(weights), ...
        "weightMaximum", max(weights));
end

function value = localPositive(stage, fieldName)
    value = localScalar(stage, fieldName);
    if value <= 0.0
        error("nrmmObserverCertificate:invalidStage", ...
            "stage.%s must be positive.", fieldName);
    end
end

function value = localNonnegative(stage, fieldName)
    value = localScalar(stage, fieldName);
    if value < 0.0
        error("nrmmObserverCertificate:invalidStage", ...
            "stage.%s must be nonnegative.", fieldName);
    end
end

function value = localScalar(stage, fieldName)
    if ~isstruct(stage) || ~isfield(stage, fieldName)
        error("nrmmObserverCertificate:invalidStage", ...
            "stage.%s is required.", fieldName);
    end
    value = double(stage.(fieldName));
    if ~isscalar(value) || ~isfinite(value)
        error("nrmmObserverCertificate:invalidStage", ...
            "stage.%s must be a finite scalar.", fieldName);
    end
end

function input = localInput(stage)
    if ~isfield(stage, "input")
        error("nrmmObserverCertificate:invalidStage", ...
            "stage.input is required.");
    end
    input = double(stage.input);
    if ~isequal(size(input), [3, 1]) || any(~isfinite(input)) ...
            || any(input < 0.0)
        error("nrmmObserverCertificate:invalidStage", ...
            "stage.input must be a finite nonnegative three-vector.");
    end
end
