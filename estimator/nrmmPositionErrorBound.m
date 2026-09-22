function bound = nrmmPositionErrorBound(action, varargin)
% nrmmPositionErrorBound Maintain a deterministic sampled position enclosure.
% initialize(design,cfg,state,time,prior), measure(bound,input,state),
% advance(bound,input,before,after,firstDerivative,h), reset(bound,state).
% State fields: yaw, bodyVelocity, targetState (6-by-1), radarPredictor (2-by-1).
% Yaw centers the optional initial prior. Subsequent yaw bounds enclose the
% propagated/intersected orientation set about the actual numerical yaw estimate.
% Prior, when supplied, declares true initial error norm bounds in fields yaw,
% bodyVelocity and targetComponents (3-by-1). Otherwise use the physical domain.
% The enclosure concerns the actual accepted numerical states. Linear-path
% defects account for integration error, and predictor error is an additional
% comparison state. Guarantees are conditional on declared true motion/sensor
% bounds, in real arithmetic; floating-point guards are not interval arithmetic.

    switch string(action)
        case "initialize"
            bound = localInitialize(varargin{:});
        case "measure"
            bound = localMeasure(varargin{:});
        case "advance"
            bound = localAdvance(varargin{:});
        case "reset"
            bound = localReset(varargin{:});
        otherwise
            error("nrmmPositionErrorBound:unknownAction", "Unknown bound action.");
    end
end

function bound = localInitialize(design,cfg,state,time,prior)
    if nargin < 5
        prior = struct();
    end
    initialYaw = 0.0;
    if isfield(state,"yaw")
        initialYaw = state.yaw;
    end
    holdBounds = struct( ...
        "acceleration",localHoldBound(cfg,"accelerationNormMaximum"), ...
        "bodyAccelerationRate",localHoldBound(cfg,"bodyAccelerationRateMaximum"), ...
        "yawAcceleration",localHoldBound(cfg,"yawAccelerationMaximum"));
    bound = struct("design",design,"holdBounds",holdBounds,"time",time, ...
        "yaw",pi,"bodyVelocity",design.operatingDomain.egoSpeedMaximum+norm(state.bodyVelocity), ...
        "targetComponents",zeros(3,1),"targetLyapunov",0, ...
        "radarPredictor",0, ...
        "trueRangeMaximum",design.target.domain.relativePositionMaximum, ...
        "egoValid",true,"valid",true,"reason","declared-domain-prior", ...
        "lastRadarTime",NaN, ...
        "orientationSet",nrmmYawSet("initialize",initialYaw,pi), ...
        "lastDefect",struct(),"lastComparison",struct([]), ...
        "integrationErrorIncluded",true,"floatingPointVerified",false, ...
        "scope","conditional deterministic containment; no sampled exponential stability claim");
    duration = 2.0;
    if isfield(cfg.runtime,"targetHistoryDuration"), duration = cfg.runtime.targetHistoryDuration; end
    validateattributes(duration,{'double'},{'scalar','finite','positive'});
    bound.targetHistory = nrmmTargetHistory("initialize", ...
        design.target.domain,design.target.modelJerkMaximum,duration,holdBounds.yawAcceleration);
    bound = localReset(bound,state);
    if ~isempty(fieldnames(prior))
        validateattributes(prior.yaw,{'double'},{'real','scalar','finite','>=',0,'<=',pi});
        validateattributes(prior.bodyVelocity,{'double'},{'real','scalar','finite','nonnegative'});
        validateattributes(prior.targetComponents,{'double'}, ...
            {'real','finite','nonnegative','size',[3,1]});
        bound.yaw = min(bound.yaw,prior.yaw);
        bound.bodyVelocity = min(bound.bodyVelocity,prior.bodyVelocity);
        bound.targetComponents = min(bound.targetComponents,prior.targetComponents);
        bound.targetLyapunov = localMetricBound( ...
            bound.targetComponents,design);
        bound.radarPredictor = bound.targetComponents(1) ...
            + norm(state.radarPredictor-state.targetState(1:2));
        bound.trueRangeMaximum = min(bound.trueRangeMaximum, ...
            norm(state.targetState(1:2))+bound.targetComponents(1));
        bound.reason = "declared-initial-error-prior";
    end
    bound.orientationSet = nrmmYawSet("initialize",initialYaw,bound.yaw);
end

function bound = localReset(bound,state)
    domain = bound.design.target.domain;
    physical = state.targetState;
    bound.trueRangeMaximum = domain.relativePositionMaximum;
    bound.targetComponents = [domain.relativePositionMaximum; ...
        domain.speedMaximum;domain.accelerationNormBound] ...
        + [norm(physical(1:2));norm(physical(3:4));norm(physical(5:6))];
    bound.targetLyapunov = localMetricBound( ...
        bound.targetComponents,bound.design);
    bound.radarPredictor = domain.relativePositionMaximum ...
        + norm(state.radarPredictor);
    bound.valid = bound.egoValid;
    bound.reason = "declared-domain-prior";
    if ~bound.egoValid
        bound.reason = "ego-bound-invalid-reinitialize-runtime";
    end
    bound.lastRadarTime = NaN;
    if isfield(bound,"targetHistory")
        history = bound.targetHistory;
        bound.targetHistory = nrmmTargetHistory("initialize",domain, ...
            bound.design.target.modelJerkMaximum,history.duration,history.yawAccelerationMaximum);
    end
end

function bound = localMeasure(bound,input,state)
    if abs(bound.time-input.time) > 128*eps(max(1,abs(input.time)))
        error("nrmmPositionErrorBound:measurementTimeMismatch", ...
            "The measurement and current bound must have the same timestamp.");
    end
    design = bound.design;
    sensors = design.sensors;
    domain = design.operatingDomain;
    course = certifiedKinematicCourseCorrespondence(input.gnssVelocity,input.yawRate, ...
        design.yaw.courseModel.rearAxleDistance,sensors.velocityNoiseMaximum, ...
        sensors.gyroscopeNoiseMaximum, ...
        design.yaw.courseModel.singleTrackYawRateMismatchMaximum, ...
        design.yaw.courseModel.sideslipDomainMaximum);
    [velocityMeasurement,feasible] = nrmmKinematicVelocityMeasurement( ...
        input.gnssVelocity,input.yawRate,design);
    sensorConsistent = feasible.consistent ...
        && abs(input.yawRate) <= localGuard(domain.egoYawRateMaximum+sensors.gyroscopeNoiseMaximum) ...
        && norm(input.bodyAcceleration) <= localGuard(bound.holdBounds.acceleration+sensors.accelerometerNoiseMaximum);
    if ~sensorConsistent
        bound.egoValid = false;
        bound.valid = false;
        bound.reason = "ego-measurement-inconsistent-with-declared-bounds";
    end
    % A failed yaw intersection invalidates only orientation-based outputs.
    % An uninformative correspondence contributes S^1 and never a false center.
    measurementSet = nrmmYawSet("initialize",course.correspondence.heading, ...
        course.correspondence.radius);
    if ~sensorConsistent
        measurementSet.intervals = zeros(0,2);
    end
    bound.orientationSet = nrmmYawSet("intersect",bound.orientationSet,measurementSet);
    bound.yaw = nrmmYawSet("radiusAbout",bound.orientationSet,state.yaw);
    model = design.yaw.courseModel;
    % This scalar reconstruction error is used only for initial/measurement
    % containment; propagation below combines the common gyro column first.
    velocityNoise = (sensors.velocityNoiseMaximum+model.rearAxleDistance ...
        *(sensors.gyroscopeNoiseMaximum+model.singleTrackYawRateMismatchMaximum)) ...
        /cos(model.sideslipDomainMaximum);
    residual = norm(velocityMeasurement-state.bodyVelocity);
    if residual > localGuard(bound.bodyVelocity+velocityNoise)
        bound.egoValid = false;
        bound.valid = false;
        bound.reason = "velocity-measurement-inconsistent-with-declared-bounds";
    end
    bound.bodyVelocity = min(bound.bodyVelocity,localGuard(residual+velocityNoise));
    if input.radarDetectionAvailable
        position = input.radarRelativePosition(:);
        if isfield(input,"gnssPosition")
            bound.targetHistory = nrmmTargetHistory("sensor", ...
                bound.targetHistory,input,design);
        end
        residual = norm(position-state.targetState(1:2));
        if residual > localGuard(bound.targetComponents(1)+sensors.radarNoiseMaximum) ...
                || norm(position) > localGuard(bound.trueRangeMaximum+sensors.radarNoiseMaximum)
            bound.valid = false;
            bound.reason = "radar-measurement-inconsistent-with-prior";
        end
        bound.targetComponents(1) = min(bound.targetComponents(1), ...
            localGuard(residual+sensors.radarNoiseMaximum));
        bound.radarPredictor = localGuard(sensors.radarNoiseMaximum);
        bound.trueRangeMaximum = min(bound.trueRangeMaximum, ...
            localGuard(norm(position)+sensors.radarNoiseMaximum));
        bound.lastRadarTime = input.time;
        bound.targetLyapunov = min(bound.targetLyapunov, ...
            localMetricBound(bound.targetComponents,design));
    end
end

function bound = localAdvance(bound,input,before,after,first,step)
    validateattributes(step,{'double'},{'scalar','real','finite','positive'});
    design = bound.design;
    target = design.target;
    domain = design.operatingDomain;
    age = max(0,bound.time-input.time)+step;
    gyroError = domain.egoYawRateMaximum+abs(input.yawRate);
    if isfinite(bound.holdBounds.yawAcceleration)
        gyroError = min(gyroError,design.sensors.gyroscopeNoiseMaximum ...
            + bound.holdBounds.yawAcceleration*age);
    end
    defect = localDefect(before,after,first,input,design,step);
    velocityCap = domain.egoSpeedMaximum ...
        + max(norm(before.bodyVelocity),norm(after.bodyVelocity));
    bound.orientationSet = nrmmYawSet("propagate",bound.orientationSet, ...
        input.yawRate*step,gyroError*step);
    egoMatrix = 0;
    egoInput = 0;
    egoInitial = bound.bodyVelocity;
    accelerationEnvelope = bound.holdBounds.acceleration;
    if isfinite(bound.holdBounds.bodyAccelerationRate)
        accelerationEnvelope = min(accelerationEnvelope,norm(input.bodyAcceleration) ...
            + design.sensors.accelerometerNoiseMaximum+bound.holdBounds.bodyAccelerationRate*age);
    end
    if isfinite(accelerationEnvelope)
        accelerationError = bound.holdBounds.acceleration+norm(input.bodyAcceleration);
        if isfinite(bound.holdBounds.bodyAccelerationRate)
            accelerationError = min(accelerationError,design.sensors.accelerometerNoiseMaximum ...
                + bound.holdBounds.bodyAccelerationRate*age);
        end
        velocityError = min(domain.egoSpeedMaximum+norm(input.gnssVelocity), ...
            design.sensors.velocityNoiseMaximum+accelerationEnvelope*age);
        effectiveSensors = design.sensors;
        effectiveSensors.velocityNoiseMaximum = velocityError;
        effectiveSensors.gyroscopeNoiseMaximum = gyroError;
        effectiveSensors.accelerometerNoiseMaximum = accelerationError;
        velocityCertificate = nrmmVelocityDisturbanceBound(design.velocity.gain, ...
            [domain.egoSpeedMinimum,domain.egoSpeedMaximum], ...
            design.yaw.courseModel,effectiveSensors);
        egoMatrix = -design.velocity.gain;
        egoInput = velocityCertificate.disturbanceBound+defect.bodyVelocity;
    else
        % A sample acceleration does not bound intersample acceleration.
        % The true speed domain still gives a finite uniform velocity error.
        egoInitial = localGuard(velocityCap);
    end
    velocityPathMaximum = velocityCap;
    if isfinite(accelerationEnvelope)
        equilibrium = egoInput/design.velocity.gain;
        velocityPathMaximum = min(velocityPathMaximum, ...
            localGuard(max(bound.bodyVelocity,equilibrium)));
    end
    range = bound.trueRangeMaximum ...
        + (domain.egoSpeedMaximum+target.domain.speedMaximum)*step;
    h = [range;target.domain.speedMaximum/target.bandwidth; ...
        target.domain.accelerationNormBound/target.bandwidth^2];
    gyroCoefficient = sqrt(h.'*abs(target.lyapunovMatrix)*h);
    if input.radarDetectionAvailable
        matrix = zeros(3);
        matrix(1,1) = egoMatrix;
        matrix(2,1:3) = [design.coupling.targetVelocityCoupling, ...
            -target.lambda,target.disturbanceCoefficients.radar];
        matrix(3,1:2) = [1,target.componentConversion(2)];
        forcing = [egoInput;gyroCoefficient*gyroError ...
            + target.disturbanceCoefficients.modelJerk*target.modelJerkMaximum ...
            + localMetricBound(defect.target,design); ...
            range*gyroError+defect.radarPredictor];
        initial = [egoInitial;bound.targetLyapunov;bound.radarPredictor];
        next = localComparisonStep(matrix,forcing,initial,step);
        bound.targetLyapunov = next(2);
        components = target.componentConversion*next(2);
        predictor = next(3);
        mode = "radar-correction";
    else
        % No negative target decay is claimed when radar correction is off.
        matrix = zeros(5);
        matrix(1,1) = egoMatrix;
        matrix(2,[1,3]) = 1;
        matrix(3,4) = 1;
        matrix(4,3:4) = [target.lipschitzCertificate.phiVelocity, ...
            target.lipschitzCertificate.phiAcceleration];
        matrix(5,[1,3]) = 1;
        forcing = [egoInput;gyroError*[range;target.domain.speedMaximum; ...
            target.domain.accelerationNormBound]+defect.target ...
            + [0;0;target.modelJerkMaximum];range*gyroError+defect.radarPredictor];
        initial = [egoInitial;bound.targetComponents;bound.radarPredictor];
        next = localComparisonStep(matrix,forcing,initial,step);
        components = next(2:4);
        predictor = next(5);
        bound.targetLyapunov = localMetricBound(components,design);
        mode = "radar-dropout";
    end
    % Independent domain enclosures can only tighten valid error bounds.
    qCap = target.domain.speedMaximum+max(norm(before.targetState(3:4)), ...
        norm(after.targetState(3:4)));
    predictorCap = bound.radarPredictor+step*(qCap+velocityPathMaximum ...
        + range*gyroError+defect.radarPredictor);
    predictor = min(predictor,localGuard(predictorCap));
    physical = after.targetState;
    caps = [range+norm(physical(1:2));target.domain.speedMaximum+norm(physical(3:4)); ...
        target.domain.accelerationNormBound+norm(physical(5:6))];
    components = min(components,localGuard(caps));
    components(1) = min(components(1), ...
        localGuard(predictor+norm(after.radarPredictor-physical(1:2))));
    bound.targetComponents = components;
    bound.targetLyapunov = min(bound.targetLyapunov, ...
        localMetricBound(components,design));
    bound.radarPredictor = predictor;
    bound.trueRangeMaximum = localGuard(range);
    record = struct("matrix",matrix,"input",forcing,"initial",initial, ...
        "final",next,"step",step,"mode",mode);
    comparison = record;
    bound.yaw = nrmmYawSet("radiusAbout",bound.orientationSet,after.yaw);
    bound.bodyVelocity = min(next(1),localGuard(domain.egoSpeedMaximum+norm(after.bodyVelocity)));
    bound.time = bound.time+step;
    bound.lastDefect = defect;
    bound.lastComparison = comparison;
    bound.lastGyroscopeHoldError = gyroError;
    bound.usesAccelerationEnvelope = isfinite(accelerationEnvelope);
end

function defect = localDefect(before,after,first,input,design,step)
% A posteriori defect of the line joining two accepted numerical states.
% Affine target rows use their exact midpoint variation. Only Phi needs a
% global Lipschitz remainder. No RK4 order or unbounded fifth derivative is used.
    cross = [0,-1;1,0];
    deltaVelocity = after.bodyVelocity-before.bodyVelocity;
    velocityDelta = -input.yawRate*cross*deltaVelocity ...
        -design.velocity.gain*deltaVelocity;
    defect = struct("bodyVelocity",localGuard( ...
        norm(deltaVelocity/step-first.bodyVelocity-0.5*velocityDelta) ...
        + 0.5*norm(velocityDelta)), ...
        "target",zeros(3,1), ...
        "radarPredictor",0);
    delta = after.targetState-before.targetState;
    deltaPredictor = after.radarPredictor-before.radarPredictor;
    gains = double(input.radarDetectionAvailable)*design.target.innovationGains;
    innovationDelta = deltaPredictor-delta(1:2);
    linearDelta = [delta(3:4)-deltaVelocity-input.yawRate*cross*delta(1:2) ...
        + gains(1)*innovationDelta;delta(5:6)-input.yawRate*cross*delta(3:4) ...
        + gains(2)*innovationDelta;-input.yawRate*cross*delta(5:6) ...
        + gains(3)*innovationDelta];
    residual = delta/step-first.targetState-0.5*linearDelta;
    for component = 1:3
        rows = (2*component-1):(2*component);
        defect.target(component) = norm(residual(rows))+0.5*norm(linearDelta(rows));
    end
    defect.target(3) = defect.target(3) ...
        + design.target.lipschitzCertificate.phiVelocity*norm(delta(3:4)) ...
        + design.target.lipschitzCertificate.phiAcceleration*norm(delta(5:6));
    predictorDelta = delta(3:4)-deltaVelocity-input.yawRate*cross*deltaPredictor;
    defect.radarPredictor = localGuard( ...
        norm(deltaPredictor/step-first.radarPredictor-0.5*predictorDelta) ...
        + 0.5*norm(predictorDelta));
    defect.target = localGuard(defect.target);
end

function next = localComparisonStep(matrix,forcing,initial,step)
% Cache the state/input flow, which is independent of the changing forcing.
% The augmented identity integrates constant inputs without inverting H.
    persistent cachedMatrices cachedSteps cachedFlows
    count = numel(initial);
    if isempty(cachedMatrices)
        cachedMatrices = cell(0,1);cachedSteps = zeros(0,1);cachedFlows = cell(0,1);
    end
    selected = [];
    for index = 1:numel(cachedSteps)
        if step==cachedSteps(index) && isequal(matrix,cachedMatrices{index})
            selected = index;break;
        end
    end
    if isempty(selected)
        transition = expm(step*[matrix,eye(count);zeros(count,2*count)]);
        flow = transition(1:count,:);
        if numel(cachedSteps)==4
            cachedMatrices(1) = [];cachedSteps(1) = [];cachedFlows(1) = [];
        end
        cachedMatrices{end+1,1} = matrix;cachedSteps(end+1,1) = step;cachedFlows{end+1,1} = flow;
    else
        flow = cachedFlows{selected};
    end
    next = localGuard(max(0,flow*[initial;forcing]));
end

function value = localMetricBound(components,design)
    scaled = [1;1/design.target.bandwidth;1/design.target.bandwidth^2].*components;
    value = localGuard(sqrt(scaled.'*abs(design.target.lyapunovMatrix)*scaled));
end

function value = localHoldBound(cfg,name)
    value = Inf;
    if isfield(cfg.ego.domain,name)
        value = cfg.ego.domain.(name);
    end
    validateattributes(value,{'double'},{'real','scalar','nonnegative','nonnan'});
end

function value = localGuard(value)
% A conservative engineering roundoff guard, not directed interval arithmetic.
    finite = isfinite(value);
    value(finite) = value(finite)+256*eps(max(1,abs(value(finite))));
    value(isnan(value)) = Inf;
end
