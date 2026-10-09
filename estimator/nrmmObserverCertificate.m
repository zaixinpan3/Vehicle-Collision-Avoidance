function certificate = nrmmObserverCertificate(action, varargin)
% nrmmObserverCertificate Algebraic certificates of the cascaded observer.
%
% Three closed-form evaluations that the gain synthesis and the sampled
% bound recursion share. None of them calls a solver.
%
%   certificate = nrmmObserverCertificate("core", stage)
%       Triangular core ISS certificate. For chi = [Wv; WT], the comparison
%       dynamics have the positive lower-triangular form D+chi <= A*chi+d.
%       Positive weights are selected by backward substitution so that
%       w'*A = -[1,1]. The copositive function V = w'*chi therefore satisfies
%           D+V <= -(1/max(w))*V + w'*d.
%       This explicit certificate replaces a generic Lyapunov-equation solve
%       and displays every feedforward coupling directly. stage carries
%       velocityDecayRate, targetDecayRate, targetVelocityCoupling and the
%       nonnegative two-vector input.
%   certificate = nrmmObserverCertificate("velocity", gain, speedInterval, model, sensors)
%       Combined gyro forcing of the body-velocity stage, bounded before norms.
%       The same gyro error enters l*u in the innovation and -u*J*v in
%       propagation; C_omega bounds [v_y-k*l*zeta; k*l-v_x] with
%       |zeta| <= tan(b). Sensor fields may also contain certified effective
%       held-input error bounds. Returns disturbanceBound and ultimateBound.
%   certificate = nrmmObserverCertificate("targetLipschitz", domain)
%       Global Lipschitz bounds of the unchanged saturated NRMM map: for all
%       q1,q2,s1,s2 in R^2 the extension in nrmmTargetTrackerDerivative
%       satisfies |Phi_e(q1,s1)-Phi_e(q2,s2)| <= Lq*|q1-q2|+Ls*|s1-s2|.
%       The bounds are analytic over all radial and scalar saturation
%       regions, not a sampled Jacobian maximum; the acceleration Jacobian
%       retains its 2-by-2 directional structure. See OBSERVER_ISS_THEORY.md.

    switch string(action)
        case "core"
            certificate = localCoreCertificate(varargin{1});
        case "velocity"
            certificate = localVelocityDisturbanceBound( ...
                varargin{1}, varargin{2}, varargin{3}, varargin{4});
        case "targetLipschitz"
            certificate = localTargetLipschitz(varargin{1});
        otherwise
            error("nrmmObserverCertificate:invalidAction", ...
                "action must be 'core', 'velocity' or 'targetLipschitz'.");
    end
end

%% Triangular core ISS certificate

function certificate = localCoreCertificate(stage)
    velocityDecayRate = localPositive(stage, "velocityDecayRate");
    targetDecayRate = localPositive(stage, "targetDecayRate");
    targetVelocityCoupling = localNonnegative( ...
        stage, "targetVelocityCoupling");
    input = localInput(stage);

    comparisonMatrix = [ ...
        -velocityDecayRate, 0.0; ...
        targetVelocityCoupling, -targetDecayRate];
    targetWeight = 1.0/targetDecayRate;
    velocityWeight = ...
        (1.0+targetVelocityCoupling*targetWeight)/velocityDecayRate;
    weights = [velocityWeight; targetWeight];
    weightedMatrix = weights.'*comparisonMatrix;
    residual = weightedMatrix+ones(1, 2);
    tolerance = 256.0*eps(max([1.0; abs(weights); ...
        abs(comparisonMatrix(:))]));
    if max(abs(residual), [], "all") > tolerance
        error("nrmmObserverCertificate:invalidWeights", ...
            "The backward weights must satisfy w'*A = -ones(1,2).");
    end

    certificate = struct( ...
        "type", "linear-copositive", ...
        "stateOrder", ["bodyVelocity"; "target"], ...
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
    if ~isequal(size(input), [2, 1]) || any(~isfinite(input)) ...
            || any(input < 0.0)
        error("nrmmObserverCertificate:invalidStage", ...
            "stage.input must be a finite nonnegative two-vector.");
    end
end

%% Velocity-stage disturbance bound with the common gyro column

function certificate = localVelocityDisturbanceBound(gain, speedInterval, model, sensors)
    persistent cachedGeometry cachedCoefficients
    geometry = {gain,speedInterval,model};
    if ~isequaln(geometry,cachedGeometry)
        cachedCoefficients = localVelocityCoefficients(gain,speedInterval,model);
        cachedGeometry = geometry;
    end
    sensorFields = ["velocityNoiseMaximum","gyroscopeNoiseMaximum","accelerometerNoiseMaximum"];
    for field = sensorFields
        validateattributes(sensors.(field), {'double'}, {'real','finite','scalar','nonnegative'});
    end
    certificate = cachedCoefficients;
    forcing = sensors.accelerometerNoiseMaximum ...
        + certificate.speedCoefficient*sensors.velocityNoiseMaximum ...
        + certificate.gyroCoefficient*sensors.gyroscopeNoiseMaximum ...
        + certificate.mismatchCoefficient*model.singleTrackYawRateMismatchMaximum;
    certificate.disturbanceBound = forcing;
    certificate.ultimateBound = forcing/gain;
end

function certificate = localVelocityCoefficients(gain,speedInterval,model)
% Geometry and observer gain are fixed across changing held sensor bounds.
    validateattributes(gain, {'double'}, {'real','finite','scalar','positive'});
    validateattributes(speedInterval, {'double'}, {'real','finite','numel',2,'nonnegative'});
    if speedInterval(1) > speedInterval(2)
        error("nrmmObserverCertificate:invalidSpeedInterval", ...
            "The speed interval must be ordered.");
    end
    angle = model.sideslipDomainMaximum;
    distance = model.rearAxleDistance;
    validateattributes(angle, {'double'}, {'real','finite','scalar','>=',0,'<',pi/2});
    validateattributes(distance, {'double'}, {'real','finite','scalar','positive'});
    mismatch = model.singleTrackYawRateMismatchMaximum;
    validateattributes(mismatch, {'double'}, {'real','finite','scalar','nonnegative'});
    cosine = cos(angle);
    speed = speedInterval(:);
    gyro = max(hypot(speed*sin(angle)+gain*distance*tan(angle), ...
        gain*distance-speed*cosine));
    speedCoefficient = gain/cosine;
    mismatchCoefficient = gain*distance/cosine;
    certificate = struct("gyroCoefficient",gyro, ...
        "separatedGyroCoefficient",max(speed)+gain*distance/cosine, ...
        "speedCoefficient",speedCoefficient,"mismatchCoefficient",mismatchCoefficient, ...
        "disturbanceBound",0,"ultimateBound",0);
end

%% Global Lipschitz bounds of the saturated target map

function certificate = localTargetLipschitz(domain)
    fields = ["speedMinimum","speedMaximum","accelerationNormBound", ...
        "scalarAccelerationMaximum","yawRateMaximum"];
    for field = fields
        validateattributes(domain.(field),{'double'}, ...
            {'scalar','real','finite','positive'},"nrmmObserverCertificate",field);
    end
    minimumSpeed = domain.speedMinimum;
    maximumSpeed = domain.speedMaximum;
    if maximumSpeed <= minimumSpeed
        error("nrmmObserverCertificate:invalidTargetSpeedInterval", ...
            "The target speed maximum must exceed its positive minimum.");
    end
    accelerationNorm = domain.accelerationNormBound;
    scalarAcceleration = domain.scalarAccelerationMaximum;
    yawRate = domain.yawRateMaximum;

    accelerationVelocity = accelerationNorm/minimumSpeed;
    yawVelocity = accelerationNorm ...
        *max(1/minimumSpeed^2,2/maximumSpeed^2);
    weightedYawVelocity = accelerationNorm ...
        *max(1/minimumSpeed,2/maximumSpeed);
    phiVelocity = yawRate^2+2*yawRate*weightedYawVelocity ...
        +3*scalarAcceleration*yawRate/minimumSpeed ...
        +3*yawRate*accelerationVelocity ...
        +3*scalarAcceleration*yawVelocity;
    accelerationJacobianEnvelope = [0,2*yawRate; ...
        3*yawRate,3*scalarAcceleration/minimumSpeed];
    phiAcceleration = norm(accelerationJacobianEnvelope,2);
    certificate = struct( ...
        "scalarAccelerationVelocity",accelerationVelocity, ...
        "scalarAccelerationAcceleration",1, ...
        "yawRateVelocity",yawVelocity, ...
        "yawRateAcceleration",1/minimumSpeed, ...
        "velocityWeightedYawRateVelocity",weightedYawVelocity, ...
        "accelerationJacobianEnvelope",accelerationJacobianEnvelope, ...
        "phiVelocity",phiVelocity, ...
        "phiAcceleration",phiAcceleration, ...
        "phi",hypot(phiVelocity,phiAcceleration), ...
        "scope","global analytic bounds for the unchanged saturation extension");
end
