function bound = nrmmPositionErrorBound(action, varargin)
% nrmmPositionErrorBound Maintain a deterministic sampled position enclosure.
% initialize(design,cfg,state,time,prior), measure(bound,input,state),
% advance(bound,input,before,after,firstDerivative,h), reset(bound,index,state).
% State fields: yaw, bodyVelocity, targetState (6-by-N), radarPredictor (2-by-N).
% Prior, when supplied, declares true initial error norm bounds in fields yaw,
% bodyVelocity and targetComponents (3-by-N). Otherwise use the physical domain.
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
    count = size(state.targetState,2);
    holdBounds = struct( ...
        "acceleration",localHoldBound(cfg,"accelerationNormMaximum"), ...
        "bodyAccelerationRate",localHoldBound(cfg,"bodyAccelerationRateMaximum"), ...
        "yawAcceleration",localHoldBound(cfg,"yawAccelerationMaximum"));
    bound = struct("design",design,"holdBounds",holdBounds,"time",time, ...
        "yaw",pi,"bodyVelocity",design.operatingDomain.egoSpeedMaximum+norm(state.bodyVelocity), ...
        "targetComponents",zeros(3,count),"targetLyapunov",zeros(1,count), ...
        "radarPredictor",zeros(1,count), ...
        "trueRangeMaximum",repmat(design.target.domain.relativePositionMaximum,1,count), ...
        "egoValid",true,"valid",true(1,count),"reason",repmat("declared-domain-prior",1,count), ...
        "lastRadarTime",NaN(1,count),"yawMeasurementRadius",pi,"yawHeading",0, ...
        "lastDefect",struct(),"lastComparison",struct([]), ...
        "integrationErrorIncluded",true,"floatingPointVerified",false, ...
        "scope","conditional deterministic containment; no sampled exponential stability claim");
    for index = 1:count
        bound = localReset(bound,index,state);
    end
    if ~isempty(fieldnames(prior))
        validateattributes(prior.yaw,{'double'},{'real','scalar','finite','>=',0,'<=',pi});
        validateattributes(prior.bodyVelocity,{'double'},{'real','scalar','finite','nonnegative'});
        validateattributes(prior.targetComponents,{'double'}, ...
            {'real','finite','nonnegative','size',[3,count]});
        bound.yaw = min(bound.yaw,prior.yaw);
        bound.bodyVelocity = min(bound.bodyVelocity,prior.bodyVelocity);
        bound.targetComponents = min(bound.targetComponents,prior.targetComponents);
        for index = 1:count
            bound.targetLyapunov(index) = localMetricBound( ...
                bound.targetComponents(:,index),design);
            bound.radarPredictor(index) = bound.targetComponents(1,index) ...
                + norm(state.radarPredictor(:,index)-state.targetState(1:2,index));
            bound.trueRangeMaximum(index) = min(bound.trueRangeMaximum(index), ...
                norm(state.targetState(1:2,index))+bound.targetComponents(1,index));
        end
        bound.reason(:) = "declared-initial-error-prior";
    end
end

function bound = localReset(bound,index,state)
    domain = bound.design.target.domain;
    physical = state.targetState(:,index);
    bound.trueRangeMaximum(index) = domain.relativePositionMaximum;
    bound.targetComponents(:,index) = [domain.relativePositionMaximum; ...
        domain.speedMaximum;domain.accelerationNormBound] ...
        + [norm(physical(1:2));norm(physical(3:4));norm(physical(5:6))];
    bound.targetLyapunov(index) = localMetricBound( ...
        bound.targetComponents(:,index),bound.design);
    bound.radarPredictor(index) = domain.relativePositionMaximum ...
        + norm(state.radarPredictor(:,index));
    bound.valid(index) = bound.egoValid;
    bound.reason(index) = "declared-domain-prior";
    if ~bound.egoValid
        bound.reason(index) = "ego-bound-invalid-reinitialize-runtime";
    end
    bound.lastRadarTime(index) = NaN;
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
    yawDistance = abs(localWrap(course.correspondence.heading-state.yaw));
    radius = course.correspondence.radius;
    sensorConsistent = course.correspondence.informative && course.boundsConsistent ...
        && norm(input.gnssVelocity) <= domain.egoSpeedMaximum+sensors.velocityNoiseMaximum ...
        && norm(input.gnssVelocity)+sensors.velocityNoiseMaximum >= domain.egoSpeedMinimum ...
        && abs(input.yawRate) <= domain.egoYawRateMaximum+sensors.gyroscopeNoiseMaximum ...
        && norm(input.bodyAcceleration) <= bound.holdBounds.acceleration+sensors.accelerometerNoiseMaximum;
    if ~sensorConsistent || yawDistance > localGuard(bound.yaw+radius)
        bound.egoValid = false;
        bound.valid(:) = false;
        bound.reason(:) = "ego-measurement-inconsistent-with-declared-bounds";
    end
    bound.yaw = min(bound.yaw,localGuard(yawDistance+radius));
    bound.yawMeasurementRadius = radius;
    bound.yawHeading = course.correspondence.heading;
    rotation = [cos(state.yaw),-sin(state.yaw);sin(state.yaw),cos(state.yaw)];
    velocityNoise = sensors.velocityNoiseMaximum ...
        + 2*domain.egoSpeedMaximum*sin(min(bound.yaw,pi)/2);
    residual = norm(rotation.'*input.gnssVelocity-state.bodyVelocity);
    if residual > localGuard(bound.bodyVelocity+velocityNoise)
        bound.egoValid = false;
        bound.valid(:) = false;
        bound.reason(:) = "velocity-measurement-inconsistent-with-declared-bounds";
    end
    bound.bodyVelocity = min(bound.bodyVelocity,localGuard(residual+velocityNoise));
    for index = 1:size(state.targetState,2)
        if input.radarDetectionAvailable(index)
            position = input.radarRelativePosition(index,:).';
            residual = norm(position-state.targetState(1:2,index));
            if residual > localGuard(bound.targetComponents(1,index)+sensors.radarNoiseMaximum) ...
                    || norm(position) > localGuard(bound.trueRangeMaximum(index)+sensors.radarNoiseMaximum)
                bound.valid(index) = false;
                bound.reason(index) = "radar-measurement-inconsistent-with-prior";
            end
            bound.targetComponents(1,index) = min(bound.targetComponents(1,index), ...
                localGuard(residual+sensors.radarNoiseMaximum));
            bound.radarPredictor(index) = localGuard(sensors.radarNoiseMaximum);
            bound.trueRangeMaximum(index) = min(bound.trueRangeMaximum(index), ...
                localGuard(norm(position)+sensors.radarNoiseMaximum));
            bound.lastRadarTime(index) = input.time;
            bound.targetLyapunov(index) = min(bound.targetLyapunov(index), ...
                localMetricBound(bound.targetComponents(:,index),design));
        end
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
    headingRadius = min(pi,bound.yawMeasurementRadius+domain.egoYawRateMaximum*age);
    yawPathBound = bound.yaw+domain.egoYawRateMaximum*step+abs(after.yaw-before.yaw);
    chartValid = yawPathBound+headingRadius < pi;
    egoMatrix = zeros(2);
    egoInput = zeros(2,1);
    egoInitial = [bound.yaw;bound.bodyVelocity];
    if chartValid
        egoMatrix(1,1) = -design.yaw.correctionBandwidth;
        egoInput(1) = gyroError+design.yaw.correctionBandwidth*headingRadius+defect.yaw;
    else
        % Circular distance is bounded without selecting a linear-error chart.
        egoInput(1) = domain.egoYawRateMaximum+abs(after.yaw-before.yaw)/step;
    end
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
        egoMatrix(2,:) = [design.velocity.gain*domain.egoSpeedMaximum,-design.velocity.gain];
        egoInput(2) = accelerationError+domain.egoSpeedMaximum*gyroError ...
            + design.velocity.gain*velocityError+defect.bodyVelocity;
    else
        % A sample acceleration does not bound intersample acceleration.
        % The true speed domain still gives a finite uniform velocity error.
        egoInitial(2) = localGuard(velocityCap);
    end
    velocityPathMaximum = velocityCap;
    if isfinite(accelerationEnvelope)
        equilibrium = domain.egoSpeedMaximum*min(pi,yawPathBound) ...
            +egoInput(2)/design.velocity.gain;
        velocityPathMaximum = min(velocityPathMaximum, ...
            localGuard(max(bound.bodyVelocity,equilibrium)));
    end
    comparison = struct([]);
    for index = 1:size(before.targetState,2)
        range = bound.trueRangeMaximum(index) ...
            + (domain.egoSpeedMaximum+target.domain.speedMaximum)*step;
        h = [target.bandwidth^2*range;target.bandwidth*target.domain.speedMaximum; ...
            target.domain.accelerationNormBound];
        gyroCoefficient = sqrt(h.'*abs(target.lyapunovMatrix)*h);
        if input.radarDetectionAvailable(index)
            matrix = zeros(4);
            matrix(1:2,1:2) = egoMatrix;
            matrix(3,2:4) = [design.coupling.targetVelocityCoupling, ...
                -target.lambda,target.disturbanceCoefficients.radar];
            matrix(4,2:3) = [1,target.componentConversion(2)];
            forcing = [egoInput;gyroCoefficient*gyroError ...
                + target.disturbanceCoefficients.modelJerk*target.modelJerkMaximum ...
                + localMetricBound(defect.target(:,index),design); ...
                range*gyroError+defect.radarPredictor(index)];
            initial = [egoInitial;bound.targetLyapunov(index);bound.radarPredictor(index)];
            next = localComparisonStep(matrix,forcing,initial,step);
            bound.targetLyapunov(index) = next(3);
            components = target.componentConversion*next(3);
            predictor = next(4);
            mode = "radar-correction";
        else
            % No negative target decay is claimed when radar correction is off.
            matrix = zeros(6);
            matrix(1:2,1:2) = egoMatrix;
            matrix(3,[2,4]) = 1;
            matrix(4,5) = 1;
            matrix(5,4:5) = [target.lipschitzCertificate.phiVelocity, ...
                target.lipschitzCertificate.phiAcceleration];
            matrix(6,[2,4]) = 1;
            forcing = [egoInput;gyroError*[range;target.domain.speedMaximum; ...
                target.domain.accelerationNormBound]+defect.target(:,index) ...
                + [0;0;target.modelJerkMaximum];range*gyroError+defect.radarPredictor(index)];
            initial = [egoInitial;bound.targetComponents(:,index);bound.radarPredictor(index)];
            next = localComparisonStep(matrix,forcing,initial,step);
            components = next(3:5);
            predictor = next(6);
            bound.targetLyapunov(index) = localMetricBound(components,design);
            mode = "radar-dropout";
        end
        % Independent domain enclosures can only tighten valid error bounds.
        qCap = target.domain.speedMaximum+max(norm(before.targetState(3:4,index)), ...
            norm(after.targetState(3:4,index)));
        predictorCap = bound.radarPredictor(index)+step*(qCap+velocityPathMaximum ...
            + range*gyroError+defect.radarPredictor(index));
        predictor = min(predictor,localGuard(predictorCap));
        physical = after.targetState(:,index);
        caps = [range+norm(physical(1:2));target.domain.speedMaximum+norm(physical(3:4)); ...
            target.domain.accelerationNormBound+norm(physical(5:6))];
        components = min(components,localGuard(caps));
        components(1) = min(components(1), ...
            localGuard(predictor+norm(after.radarPredictor(:,index)-physical(1:2))));
        bound.targetComponents(:,index) = components;
        bound.targetLyapunov(index) = min(bound.targetLyapunov(index), ...
            localMetricBound(components,design));
        bound.radarPredictor(index) = predictor;
        bound.trueRangeMaximum(index) = localGuard(range);
        record = struct("matrix",matrix,"input",forcing,"initial",initial, ...
            "final",next,"step",step,"mode",mode);
        comparison = [comparison;record]; %#ok<AGROW>
    end
    bound.yaw = min([pi,localGuard(yawPathBound),next(1)]);
    bound.bodyVelocity = min(next(2),localGuard(domain.egoSpeedMaximum+norm(after.bodyVelocity)));
    bound.time = bound.time+step;
    bound.lastDefect = defect;
    bound.lastComparison = comparison;
    bound.lastYawChartValid = chartValid;
    bound.lastGyroscopeHoldError = gyroError;
    bound.usesAccelerationEnvelope = isfinite(accelerationEnvelope);
end

function defect = localDefect(before,after,first,input,design,step)
% A posteriori defect of the line joining two accepted numerical states.
% Affine target rows use their exact midpoint variation. Only Phi needs a
% global Lipschitz remainder. No RK4 order or unbounded fifth derivative is used.
    cross = [0,-1;1,0];
    deltaYaw = after.yaw-before.yaw;
    deltaVelocity = after.bodyVelocity-before.bodyVelocity;
    yawGain = design.yaw.correctionBandwidth;
    innovation = (first.yaw-input.yawRate)/yawGain;
    if abs(innovation)+abs(deltaYaw) < pi
        yawVariation = yawGain*abs(deltaYaw);
    else
        yawVariation = 2*pi*yawGain;
    end
    defect = struct("yaw",localGuard(abs(deltaYaw/step-first.yaw)+yawVariation), ...
        "bodyVelocity",localGuard(norm(deltaVelocity/step-first.bodyVelocity) ...
        + (abs(input.yawRate)+design.velocity.gain)*norm(deltaVelocity) ...
        + design.velocity.gain*norm(input.gnssVelocity)*min(2,abs(deltaYaw))), ...
        "target",zeros(3,size(before.targetState,2)), ...
        "radarPredictor",zeros(1,size(before.targetState,2)));
    for index = 1:size(before.targetState,2)
        delta = after.targetState(:,index)-before.targetState(:,index);
        deltaPredictor = after.radarPredictor(:,index)-before.radarPredictor(:,index);
        gains = double(input.radarDetectionAvailable(index))*design.target.innovationGains;
        innovationDelta = deltaPredictor-delta(1:2);
        linearDelta = [delta(3:4)-deltaVelocity-input.yawRate*cross*delta(1:2) ...
            + gains(1)*innovationDelta;delta(5:6)-input.yawRate*cross*delta(3:4) ...
            + gains(2)*innovationDelta;-input.yawRate*cross*delta(5:6) ...
            + gains(3)*innovationDelta];
        residual = delta/step-first.targetState(:,index)-0.5*linearDelta;
        for component = 1:3
            rows = (2*component-1):(2*component);
            defect.target(component,index) = norm(residual(rows))+0.5*norm(linearDelta(rows));
        end
        defect.target(3,index) = defect.target(3,index) ...
            + design.target.lipschitzCertificate.phiVelocity*norm(delta(3:4)) ...
            + design.target.lipschitzCertificate.phiAcceleration*norm(delta(5:6));
        predictorDelta = delta(3:4)-deltaVelocity-input.yawRate*cross*deltaPredictor;
        defect.radarPredictor(index) = localGuard( ...
            norm(deltaPredictor/step-first.radarPredictor(:,index)-0.5*predictorDelta) ...
            + 0.5*norm(predictorDelta));
    end
    defect.target = localGuard(defect.target);
end

function next = localComparisonStep(matrix,forcing,initial,step)
% Augmentation integrates constant nonnegative forcing without inverting H.
    count = numel(initial);
    flow = expm(step*[matrix,forcing;zeros(1,count+1)]);
    next = localGuard(max(0,flow(1:count,:)*[initial;1]));
end

function value = localMetricBound(components,design)
    scaled = [design.target.bandwidth^2;design.target.bandwidth;1].*components;
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

function angle = localWrap(angle)
    angle = mod(angle+pi,2*pi)-pi;
end
