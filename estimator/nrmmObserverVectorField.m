function [first, second] = nrmmObserverVectorField(action, varargin)
% nrmmObserverVectorField Continuous observer equations and their RK4 interval.
%
% The eight-state core for one target consists of body velocity and [rho;q;s].
% Absolute yaw does not drive the body-frame core. The position-only GNSS/IMU
% filter supplies inertial velocity. Synchronization and error publication
% remain in the runtime and bound modules.
%
%   targetDerivative(state,ego,domain) evaluates the covariant NRMM extension.
%   targetJacobian(action,...) evaluates its Jacobian or analytic enclosure.
%   positionVelocity(action,...) updates the position/IMU filter and enclosure.
% Each is selected by the first action argument of this function.
%
%   [derivative, model] = nrmmObserverVectorField("derivative", estimate, measurement, design)
%       Core vector field: derivative.bodyVelocity, .position, .targetState and
%       model.inertialVelocity, .targetPlantDerivative.
%   [states, firstDerivatives] = nrmmObserverVectorField("integrate", state, measurement, design, step, count)
%       Fixed-step RK4 over `count` substeps of the packed 15-vector
%       [bodyVelocity; position; gnssPredictor; targetState; radarPredictor; yaw]
%       with held measurements. Every accepted substep and its first
%       derivative are returned so that the runtime can retain every
%       error-bound update and operating-domain audit. Point dynamics do not
%       depend on the separately propagated error bounds. This action is the
%       kernel compiled by scripts/buildNrmmObserverKernel.m.
%   derivative = nrmmObserverVectorField("yawDerivative", yaw, measuredRate, correspondence, design)
%       Parallel yaw observer: gyro propagation plus a wrapped correction
%       using a continuous rotation-vector innovation. It
%       supplies inertial outputs and never drives the body-frame core.
%   [velocity, feasible] = nrmmObserverVectorField("velocityMeasurement", gnssVelocity, yawRate, design, lateralVelocity)
%       Forward kinematic inverse y_v = [phi(M, l*u); l*u]. The lateral
%       measurement is deliberately not clipped: its error is exactly
%       l*(nOmega+dSt). The longitudinal floor keeps the map defined at
%       standstill and outside the raw square-root branch. No orientation
%       estimate is used. An optional finite lateralVelocity (the certified
%       force-balance center from nrmmEgoCourseGeometry) replaces l*u. The
%       optional feasibility result concerns C_m, before any orientation prior.

    switch string(action)
        case "targetDerivative"
            if nargout > 1
                [first, second] = localTargetDerivative(varargin{:});
            else
                first = localTargetDerivative(varargin{:});
                second = [];
            end
        case "targetJacobian"
            [first, second] = localTargetJacobian(varargin{:});
        case "positionVelocity"
            first = localPositionVelocity(varargin{:});
            second = [];
        case "derivative"
            [first, second] = localDerivative(varargin{1}, varargin{2}, varargin{3});
        case "integrate"
            [first, second] = localRk4Interval(varargin{1}, varargin{2}, ...
                varargin{3}, varargin{4}, varargin{5});
        case "yawDerivative"
            first = localYawDerivative(varargin{1}, varargin{2}, varargin{3}, varargin{4});
            second = [];
        case "velocityMeasurement"
            lateralVelocity = NaN;
            if numel(varargin) >= 4
                lateralVelocity = varargin{4};
            end
            if nargout > 1
                [first, second] = localVelocityMeasurement( ...
                    varargin{1}, varargin{2}, varargin{3}, lateralVelocity);
            else
                first = localVelocityMeasurement( ...
                    varargin{1}, varargin{2}, varargin{3}, lateralVelocity);
                second = [];
            end
        otherwise
            error("nrmmObserverVectorField:invalidAction", ...
                "Unknown observer vector-field action.");
    end
end

%% Core vector field

function [derivative, model] = localDerivative(estimate, measurement, design)
    bodyVelocity = localVectorField(estimate, "bodyVelocity", 2);
    position = localVectorField(estimate, "position", 2);
    targetState = localVectorField(estimate, "targetState", 6);

    yawRate = localScalarField(measurement, "yawRate");
    gnssVelocity = localVectorField(measurement, "gnssVelocity", 2);
    bodyAcceleration = localVectorField( ...
        measurement, "bodyAcceleration", 2);
    positionReference = localVectorField( ...
        measurement, "positionReference", 2);
    radarReference = localVectorField( ...
        measurement, "radarReference", 2);
    radarAvailable = localAvailability(measurement);

    planarCross = [0.0, -1.0; 1.0, 0.0];
    % The lateral component is the certified force-balance center when the
    % shared ego model and held input are available, else l*u (NaN).
    lateralVelocity = NaN;
    if isfield(measurement, "lateralVelocity")
        lateralVelocity = double(measurement.lateralVelocity);
    end
    velocityMeasurement = localVelocityMeasurement( ...
        gnssVelocity, yawRate, design, lateralVelocity);
    velocityInnovation = velocityMeasurement-bodyVelocity;
    bodyVelocityDerivative = bodyAcceleration ...
        - yawRate*planarCross*bodyVelocity ...
        + design.velocity.gain*velocityInnovation;

    inertialVelocity = gnssVelocity;
    positionDerivative = inertialVelocity ...
        + design.position.gain*(positionReference-position);

    ego = struct("bodyVelocity", bodyVelocity, "yawRate", yawRate);
    innovationGains = design.target.innovationGains;
    targetPlantDerivative = localTargetDerivative(targetState,ego,design.target.domain);
    targetDerivative = targetPlantDerivative;
    if radarAvailable
        radarInnovation = radarReference-targetState(1:2);
        targetDerivative = targetDerivative+[innovationGains(1)*radarInnovation; ...
            innovationGains(2)*radarInnovation;innovationGains(3)*radarInnovation];
    end

    derivative = struct( ...
        "bodyVelocity", bodyVelocityDerivative, ...
        "position", positionDerivative, ...
        "targetState", targetDerivative);
    model = struct( ...
        "inertialVelocity", inertialVelocity, ...
        "targetPlantDerivative", targetPlantDerivative);
end

function value = localScalarField(input, fieldName)
    if ~isstruct(input) || ~isfield(input, fieldName)
        error("nrmmObserverVectorField:missingField", ...
            "%s is required.", fieldName);
    end
    value = double(input.(fieldName));
    if ~isscalar(value) || ~isfinite(value)
        error("nrmmObserverVectorField:invalidField", ...
            "%s must be a finite scalar.", fieldName);
    end
end

function value = localVectorField(input, fieldName, rowCount)
    if ~isstruct(input) || ~isfield(input,fieldName)
        error("nrmmObserverVectorField:missingField","%s is required.",fieldName);
    end
    value=double(input.(fieldName));
    if ~isvector(value) || numel(value)~=rowCount || any(~isfinite(value))
        error("nrmmObserverVectorField:invalidField","%s must be a finite %d-vector.",fieldName,rowCount);
    end
    value=value(:);
end

function available = localAvailability(measurement)
    if ~isfield(measurement,"radarAvailable")
        error("nrmmObserverVectorField:missingField","radarAvailable is required.");
    end
    validateattributes(measurement.radarAvailable,{'logical'},{'scalar'});
    available=measurement.radarAvailable;
end

%% Direct forward-cone velocity measurement

function [velocity, feasible] = localVelocityMeasurement(gnssVelocity, yawRate, design, lateralVelocity)
    validateattributes(gnssVelocity, {'double'}, {'real','finite','size',[2,1]});
    validateattributes(yawRate, {'double'}, {'real','finite','scalar'});
    model = design.yaw.courseModel;
    speed = norm(gnssVelocity);
    lateral = model.rearAxleDistance*yawRate;
    if isfinite(lateralVelocity)
        lateral = lateralVelocity;
    end
    cosine = cos(model.sideslipDomainMaximum);
    velocity = [sqrt(max(speed^2-lateral^2, (cosine*speed)^2)); lateral];
    if nargout < 2
        return
    end
    sensors = design.sensors;
    domain = design.operatingDomain;
    speedLower = max(domain.egoSpeedMinimum, speed-sensors.velocityNoiseMaximum);
    speedUpper = min(domain.egoSpeedMaximum, speed+sensors.velocityNoiseMaximum);
    lateralError = model.rearAxleDistance*(sensors.gyroscopeNoiseMaximum ...
        + model.singleTrackYawRateMismatchMaximum);
    lateralMaximum = speedUpper*sin(model.sideslipDomainMaximum);
    lateralLower = max(-lateralMaximum, lateral-lateralError);
    lateralUpper = min(lateralMaximum, lateral+lateralError);
    tolerance = 256*eps(max([1,speed,abs(lateral),domain.egoSpeedMaximum]));
    feasible = struct("consistent", speedLower <= speedUpper+tolerance ...
        && lateralLower <= lateralUpper+tolerance, ...
        "speedInterval", [speedLower;speedUpper], ...
        "lateralInterval", [lateralLower;lateralUpper], ...
        "floorActive", abs(lateral) > speed*sin(model.sideslipDomainMaximum));
end

%% Parallel yaw observer

function derivative = localYawDerivative(yaw, measuredRate, correspondence, design)
    validateattributes(yaw, {'double'}, {'real','finite','scalar'});
    validateattributes(measuredRate, {'double'}, {'real','finite','scalar'});
    gain = design.yaw.correctionBandwidth;
    validateattributes(gain, {'double'}, {'real','finite','scalar','positive'});
    vector=correspondence.rotationVector;
    derivative=measuredRate+gain*(vector(2)*cos(yaw)-vector(1)*sin(yaw));
end

%% Held-measurement RK4 interval (compiled kernel)

function [states,firstDerivatives] = localRk4Interval(state,measurement,design,step,count)
    validateattributes(state,{'double'},{'size',[15,1],'finite','real'});
    states = zeros(15,count+1);
    firstDerivatives = zeros(15,count);
    states(:,1) = state;
    for index = 1:count
        first = localPackedDerivative(state,measurement,design);
        second = localPackedDerivative(state+0.5*step*first,measurement,design);
        third = localPackedDerivative(state+0.5*step*second,measurement,design);
        fourth = localPackedDerivative(state+step*third,measurement,design);
        state = state+(step/6)*(first+2*second+2*third+fourth);
        if any(~isfinite(state))
            error("onlineNrmmTrackingRuntime:nonfiniteObserverState", ...
                "The cascaded observer produced a nonfinite state.");
        end
        state(end) = mod(state(end)+pi,2*pi)-pi;
        states(:,index+1) = state;
        firstDerivatives(:,index) = first;
    end
end

function derivative = localPackedDerivative(state,measurement,design)
    targetState = state(7:12);
    targetOutputPredictor = state(13:14);
    observerEstimate = struct("bodyVelocity",state(1:2),"position",state(3:4), ...
        "targetState",targetState);
    vectorFieldInput = struct("yawRate",measurement.yawRate,"gnssVelocity",measurement.gnssVelocity, ...
        "bodyAcceleration",measurement.bodyAcceleration,"positionReference",state(5:6), ...
        "radarReference",targetOutputPredictor,"radarAvailable",measurement.radarDetectionAvailable, ...
        "lateralVelocity",measurement.lateralVelocity);
    [observerDerivative,observerModel] = localDerivative(observerEstimate,vectorFieldInput,design);
    planarCross = [0,-1;1,0];
    targetPredictorDerivative = observerModel.targetPlantDerivative(1:2,:) ...
        -measurement.yawRate*planarCross*(targetOutputPredictor-targetState(1:2,:));
    yawDerivative = localYawDerivative(state(end),measurement.yawRate,measurement.correspondence,design);
    derivative = [observerDerivative.bodyVelocity;observerDerivative.position;observerModel.inertialVelocity; ...
        observerDerivative.targetState(:);targetPredictorDerivative(:);yawDerivative];
end

function [stateDerivative, estimate] = localTargetDerivative( ...
        state, ego, domain)
% nrmmObserverVectorField targetDerivative Evaluate the measured-input NRMM target map.
%
% The target state is x_T = [rho; q; s] with
%
%   rho = R(psiE)'*(pC - pE)   radar relative position, ego frame (m),
%   q   = R(psiE)'*vC          target absolute velocity, ego frame (m/s),
%   s   = R(psiE)'*aC          target absolute acceleration, ego frame (m/s^2),
%
% ordered as state = [rhoX; rhoY; qX; qY; sX; sY]. This is a one-to-one
% coordinate transformation of the Sharma NRMM target model (constant
% scalar-acceleration, constant-sideslip kinematic single track); it is not
% a different target-motion assumption. The exact dynamics are
%
%   rhoDot = q - vE - omegaE*J*rho,
%   qDot   = s - omegaE*J*q,
%   sDot   = Phi(q, s) - omegaE*J*s,
%
% with Phi(q,s) = -Omega^2*q + 3*A*Omega*J*e(q), A = q'*s/|q|,
% Omega = (J*q)'*s/|q|^2, and e(q) = q/|q|. Only the measured or estimated
% ego body velocity and ego yaw rate enter; no ego jerk or ego angular
% acceleration is required, and no derivative of any measured input is
% assumed zero.
%
% Phi is evaluated through a globally Lipschitz saturation extension Phi_e
% that equals Phi above its positive inverse floor on the operating set
% {speedMinimum <= |q| <= speedMaximum, |s| <= accelerationNormBound,
% |A| <= scalarAccelerationMaximum, |Omega| <= yawRateMaximum} and is
% bounded and Lipschitz on all of R^4. When speedMinimum=0, the separate
% regularizationSpeedMinimum is positive. Below it Phi_e=(V/a)^4 Phi on
% admissible NRMM states; synthesis charges that discrepancy to the error
% bound. At rest the Cartesian state remains valid and yaw is unobservable.
%
% ego is a scalar struct with fields bodyVelocity (2-by-1, m/s) and
% yawRate (scalar, rad/s). domain is the certified target operating domain
% (see localTargetDomain for the required fields); pass
% design.target.domain from synthesizeNrmmObserverGains.
%
% estimate returns the inverse-transform reconstructions of the Sharma
% target states (speed, scalar acceleration, yaw rate, course, sideslip,
% relative heading) together with saturation and domain-validity flags.

    arguments
        state (6, 1) double {mustBeFinite}
        ego (1, 1) struct
        domain (1, 1) struct
    end

    [speedMinimum, speedMaximum, scalarAccelerationMaximum, ...
        yawRateMaximum, accelerationNormBound, rearAxleDistance, ...
        sideslipMaximum] = localTargetDomain(domain);
    bodyVelocity = double(ego.bodyVelocity(:));
    egoYawRate = double(ego.yawRate);

    planarCross = [0.0, -1.0; 1.0, 0.0];
    relativePosition = state(1:2);
    targetVelocity = state(3:4);
    targetAcceleration = state(5:6);

    % Radial and scalar saturation give bounded, globally Lipschitz factors
    % that equal the original Phi on admissible states above the inverse floor.
    targetSpeed = norm(targetVelocity);
    inverseSpeedMinimum = speedMinimum;
    if speedMinimum == 0 && isfield(domain,"regularizationSpeedMinimum")
        inverseSpeedMinimum = domain.regularizationSpeedMinimum;
    end
    speedFloor = max(targetSpeed, inverseSpeedMinimum);
    unitDirection = targetVelocity/speedFloor;
    speedScale = min(1.0, speedMaximum/max(targetSpeed, realmin));
    qSaturated = speedScale*targetVelocity;
    accelerationRaw = norm(targetAcceleration);
    accelerationScale = min(1.0, ...
        accelerationNormBound/max(accelerationRaw, realmin));
    sSaturated = accelerationScale*targetAcceleration;

    scalarAccelerationRaw = (qSaturated.'*sSaturated)/speedFloor;
    targetScalarAcceleration = min(max(scalarAccelerationRaw, ...
        -scalarAccelerationMaximum), scalarAccelerationMaximum);
    yawRateRaw = ((planarCross*qSaturated).'*sSaturated)/speedFloor^2;
    targetYawRate = min(max(yawRateRaw, ...
        -yawRateMaximum), yawRateMaximum);

    phiValue = -targetYawRate^2*qSaturated ...
        + 3.0*targetScalarAcceleration*targetYawRate ...
            * planarCross*unitDirection;

    stateDerivative = [ ...
        targetVelocity-bodyVelocity ...
            - egoYawRate*planarCross*relativePosition; ...
        targetAcceleration-egoYawRate*planarCross*targetVelocity; ...
        phiValue-egoYawRate*planarCross*targetAcceleration];

    if nargout < 2
        return
    end

    % Reconstruct diagnostics only when the caller requests the estimate.
    saturatedQuantities = strings(0, 1);
    if targetSpeed < speedMinimum
        saturatedQuantities(end+1, 1) = "targetSpeedBelowMinimum";
    end
    if speedScale < 1.0
        saturatedQuantities(end+1, 1) = "targetSpeedAboveMaximum";
    end
    if accelerationScale < 1.0
        saturatedQuantities(end+1, 1) = "targetAccelerationNorm";
    end
    if abs(scalarAccelerationRaw) > scalarAccelerationMaximum
        saturatedQuantities(end+1, 1) = "targetScalarAcceleration";
    end
    if abs(yawRateRaw) > yawRateMaximum
        saturatedQuantities(end+1, 1) = "targetYawRate";
    end
    saturationActive = ~isempty(saturatedQuantities);
    courseAngleEgoFrame = atan2(targetVelocity(2), targetVelocity(1));
    sideslipSine = min(max( ...
        rearAxleDistance*targetYawRate/speedFloor, ...
        -sin(sideslipMaximum)), sin(sideslipMaximum));
    targetSideslip = asin(sideslipSine);
    speedDomainValid = targetSpeed >= speedMinimum;

    estimate = struct( ...
        "relativePosition", relativePosition, ...
        "relativeVelocity", stateDerivative(1:2), ...
        "targetVelocity", targetVelocity, ...
        "targetAcceleration", targetAcceleration, ...
        "targetSpeed", targetSpeed, ...
        "targetScalarAcceleration", targetScalarAcceleration, ...
        "targetYawRate", targetYawRate, ...
        "targetCourseAngleEgoFrame", courseAngleEgoFrame, ...
        "targetSideslip", targetSideslip, ...
        "targetRearAxleDistance", rearAxleDistance, ...
        "targetRelativeHeading", courseAngleEgoFrame-targetSideslip, ...
        "phiValue", phiValue, ...
        "speedDomainValid", speedDomainValid, ...
        "regularizationActive", targetSpeed < inverseSpeedMinimum, ...
        "operatingDomainValid", speedDomainValid && ~saturationActive, ...
        "saturationActive", saturationActive, ...
        "saturatedQuantities", saturatedQuantities);
end

function [speedMinimum, speedMaximum, scalarAccelerationMaximum, ...
        yawRateMaximum, accelerationNormBound, rearAxleDistance, ...
        sideslipMaximum] = localTargetDomain(domain)
    requiredFields = {'speedMinimum', 'speedMaximum', ...
        'scalarAccelerationMaximum', 'yawRateMaximum', ...
        'accelerationNormBound', 'rearAxleDistance', 'sideslipMaximum'};
    for index = coder.unroll(1:numel(requiredFields))
        fieldName = requiredFields{index};
        if ~isfield(domain, fieldName) ...
                || ~isscalar(domain.(fieldName)) ...
                || ~isfinite(domain.(fieldName))
            error("nrmmTargetTrackerDerivative:invalidDomain", ...
                "domain.%s must be a finite scalar.", fieldName);
        end
    end
    speedMinimum = double(domain.speedMinimum);
    speedMaximum = double(domain.speedMaximum);
    scalarAccelerationMaximum = ...
        double(domain.scalarAccelerationMaximum);
    yawRateMaximum = double(domain.yawRateMaximum);
    accelerationNormBound = double(domain.accelerationNormBound);
    rearAxleDistance = double(domain.rearAxleDistance);
    sideslipMaximum = double(domain.sideslipMaximum);
    inverseSpeedMinimum = speedMinimum;
    if speedMinimum == 0 && isfield(domain,"regularizationSpeedMinimum")
        inverseSpeedMinimum = domain.regularizationSpeedMinimum;
    end
    if speedMinimum < 0.0 || speedMaximum <= speedMinimum ...
            || ~isscalar(inverseSpeedMinimum) || ~isfinite(inverseSpeedMinimum) ...
            || inverseSpeedMinimum <= 0 || inverseSpeedMinimum >= speedMaximum ...
            || rearAxleDistance <= 0.0
        error("nrmmTargetTrackerDerivative:invalidDomain", ...
            "The target speed domain must be nonnegative with a positive inverse floor below its maximum.");
    end
end

function [result,field] = localTargetJacobian(action,varargin)
% nrmmObserverVectorField targetJacobian Full vector Jacobians of the retained NRMM extension.
% enclosure(domain) returns a finite analytic convex enclosure of [DqPhi,DsPhi]
% in the q-aligned frame, covering every radial/scalar/acceleration saturation.
% Orthogonal equivariance and P kron I2 make it sufficient for all headings.
% The vertices are polynomial/rational Bernstein coefficients, not samples.
% evaluate(q,s,domain) returns the Cartesian Jacobian on a smooth branch.
% Its optional second output is Phi at the same point, reusing the clipped
% velocity, acceleration, and inverse-speed factors of the Jacobian.
% At a switching surface it selects a neighboring branch for diagnostics.
% See OBSERVER_ISS_THEORY.md for the global incremental matrix-LMI proof.

    field = zeros(2,1);
    switch string(action)
        case "enclosure"
            result = localEnclosure(localDomain(varargin{1}));
        case "evaluate"
            [result,field] = localJacobian(varargin{1},varargin{2},localDomain(varargin{3}));
        otherwise
            error("nrmmTargetJacobian:invalidAction","Use 'enclosure' or 'evaluate'.");
    end
end

function values = localDomain(domain)
    minimum = domain.speedMinimum;
    if minimum == 0
        minimum = domain.regularizationSpeedMinimum;
    end
    values = [minimum,domain.speedMaximum,domain.scalarAccelerationMaximum, ...
        domain.yawRateMaximum,domain.accelerationNormBound];
    validateattributes(values,{'double'},{'real','finite','positive','size',[1,5]});
    if values(1) >= values(2)
        error("nrmmTargetJacobian:invalidDomain","The inverse floor must be below the maximum speed.");
    end
end

function enclosure = localEnclosure(domain)
    persistent cachedDomain cachedEnclosure
    if isequal(domain,cachedDomain)
        enclosure = cachedEnclosure;
        return
    end
    minimum = domain(1); maximum = domain(2);
    acceleration = domain(3); yaw = domain(4); cap = domain(5);
    subdivisions = 8;
    groups = cell(2*3*subdivisions^3*12,1);
    groupCount = 0;
    cellCount = 0;
    regionCells = zeros(3,1);
    for region = 1:3
        radial = linspace(0,1,subdivisions+1);
        lateral = linspace(0,cap,subdivisions+1);
        if region == 1
            radial = [0,1]; % The inner Jacobian uses raw clipped arguments.
        elseif region == 2
            radial = linspace(minimum,maximum,subdivisions+1);
            lateral = linspace(0,cap/minimum,subdivisions+1);
        end
        longitudinal = linspace(-cap,cap,subdivisions+1);
        for accelerationMode = -1:1
            for yawMode = 0:1
                for projected = 0:1
                    polynomial = localPolynomial(region,accelerationMode,yawMode,projected,domain);
                    for radialIndex = 1:numel(radial)-1
                        for longitudinalIndex = 1:subdivisions
                            for lateralIndex = 1:subdivisions
                                box = [radial(radialIndex:radialIndex+1); ...
                                    longitudinal(longitudinalIndex:longitudinalIndex+1); ...
                                    lateral(lateralIndex:lateralIndex+1)];
                                box = localIntersect(box,region,accelerationMode,yawMode, ...
                                    projected,minimum,maximum,acceleration,yaw,cap);
                                if isempty(box), continue; end
                                vertices = localBernstein(polynomial,box);
                                groupCount = groupCount+1;
                                groups{groupCount} = vertices;
                                cellCount = cellCount+1;
                                regionCells(region) = regionCells(region)+1;
                                if projected || region == 1
                                    % The projection derivative has a scalar factor in [0,1].
                                    % The inverse-floor interior has the common factor (r/a)^2.
                                    vertices(5:8,:) = 0;
                                    groupCount = groupCount+1;
                                    groups{groupCount} = vertices;
                                end
                            end
                        end
                    end
                end
            end
        end
    end
    vertices = unique([zeros(8,1),groups{1:groupCount}].','rows').';
    [~,lowerIndices] = min(vertices,[],2);
    [~,upperIndices] = max(vertices,[],2);
    zeroIndex = find(all(vertices==0,1),1);
    enclosure = struct("vertices",reshape(vertices,2,4,[]), ...
        "initialIndices",unique([zeroIndex;lowerIndices;upperIndices]), ...
        "domain",domain,"subdivisions",subdivisions,"cellCount",cellCount, ...
        "regionCellCounts",regionCells,"vertexCount",size(vertices,2), ...
        "construction","global rational-Bernstein vector-Jacobian enclosure", ...
        "scope","all states of the retained extension, up to orthogonal conjugation", ...
        "floatingPointVerified",false);
    cachedDomain = domain;
    cachedEnclosure = enclosure;
end

function box = localIntersect(box,region,accelerationMode,yawMode,projected,a,b,acceleration,yaw,cap)
    radial = box(1,:);
    accelerationScale = [1,1];
    yawScale = [1/a,1/a];
    if region == 2
        yawScale = [1,1];
    elseif region == 3
        accelerationScale = radial;
        yawScale = radial.^2/b;
    end
    if accelerationMode == 0 && accelerationScale(1) > 0
        box(2,:) = [max(box(2,1),-acceleration/accelerationScale(1)), ...
            min(box(2,2),acceleration/accelerationScale(1))];
    elseif accelerationMode == 1
        box(2,1) = max(box(2,1),acceleration/accelerationScale(2));
    elseif accelerationMode == -1
        box(2,2) = min(box(2,2),-acceleration/accelerationScale(2));
    end
    if yawMode == 0 && yawScale(1) > 0
        box(3,2) = min(box(3,2),yaw/yawScale(1));
    elseif yawMode == 1
        box(3,1) = max(box(3,1),yaw/yawScale(2));
    end
    if any(box(:,1)>box(:,2))
        box = [];
        return
    end
    minimumX = max([0,box(2,1),-box(2,2)]);
    radialScale = 1;
    if region == 2, radialScale = radial(1); end
    tolerance = 256*eps(max(1,cap^2));
    if minimumX^2+(radialScale*box(3,1))^2 > cap^2+tolerance
        box = [];
        return
    end
    xCap = sqrt(max(0,cap^2-(radialScale*box(3,1))^2));
    box(2,:) = [max(box(2,1),-xCap),min(box(2,2),xCap)];
    box(3,2) = min(box(3,2),sqrt(max(0,cap^2-minimumX^2))/radialScale);
    if region == 2, radialScale = radial(2); end
    if any(box(:,1)>box(:,2)) || (projected && region ~= 1 ...
            && max(box(2,:).^2)+(radialScale*box(3,2))^2 < cap^2-tolerance)
        box = [];
    end
end

function polynomial = localPolynomial(region,accelerationMode,yawMode,projected,domain)
% Compact variables: (unused,u,a*v), (r,u,v), or (b/r,x,y).
% The middle-region polynomial is multiplied by the positive denominator r^2.
    a = domain(1); b = domain(2); acceleration = domain(3); yaw = domain(4); cap = domain(5);
    zero = zeros(6,4,4); one = zero; one(1,1,1) = 1;
    t = zero; t(2,1,1) = 1;
    x = zero; x(1,2,1) = 1;
    y = zero; y(1,1,2) = 1;
    multiply = @localProduct;
    tSquared = multiply(t,t);
    u = x; v = y/a;
    if region == 2
        v = y;
    elseif region == 3
        u = multiply(t,x); v = multiply(tSquared,y)/b;
    end
    chi = double(accelerationMode==0); eta = double(yawMode==0);
    alpha = accelerationMode*acceleration*one;
    if chi, alpha = u; end
    nu = yaw*one;
    if eta, nu = v; end
    nuSquared = multiply(nu,nu);
    alphaNu = multiply(alpha,nu);
    alphaU = multiply(alpha,u);
    nuV = multiply(nu,v);
    denominator = one;
    if region == 1
        q = {-(1+2*eta)*nuSquared, multiply(nu,2*eta*u-3*alpha)/a; ...
            3*(1+chi+eta)*alphaNu/a, -nuSquared+3*chi*nuV-3*eta*alphaU/a^2};
        g = {zero,-2*eta*nu;3*chi*nu,3*eta*alpha/a};
    elseif region == 2
        denominator = tSquared;
        q = {(2*eta-1)*multiply(nuSquared,tSquared), multiply(multiply(nu,t),2*eta*u-3*alpha); ...
            -3*eta*multiply(alphaNu,t),multiply(-nuSquared+3*chi*nuV,tSquared)-3*eta*alphaU};
        g = {zero,-2*eta*multiply(nu,tSquared); ...
            3*chi*multiply(nu,tSquared),3*eta*multiply(alpha,t)};
    else
        q = {4*eta*multiply(t,nuSquared),multiply(multiply(nu,t),2*eta*multiply(t,u)-3*alpha)/b; ...
            -3*(chi+2*eta)*multiply(alphaNu,t)/b, ...
            -multiply(t,nuSquared)+3*chi*nuV-3*eta*multiply(alphaU,tSquared)/b^2};
        g = {zero,-2*eta*multiply(nu,tSquared); ...
            3*chi*multiply(nu,t),3*eta*multiply(alpha,tSquared)/b};
    end
    if projected
        projectedY = y;
        if region == 2, projectedY = multiply(t,y); end
        k = {multiply(projectedY,projectedY)/cap^2,-multiply(x,projectedY)/cap^2; ...
            -multiply(x,projectedY)/cap^2,multiply(x,x)/cap^2};
        product = cell(2);
        for row = 1:2
            for column = 1:2
                product{row,column} = multiply(g{row,1},k{1,column}) ...
                    +multiply(g{row,2},k{2,column});
            end
        end
        g = product;
    end
    entries = [q,g];
    coefficients = cat(4,entries{:},denominator);
    [first,second,third] = ind2sub([6,4,4],find(any(reshape(coefficients,96,9)~=0,2)));
    polynomial = coefficients(1:max(first),1:max(second),1:max(third),:);
end

function product = localProduct(first,second)
    full = convn(first,second);
    product = full(1:6,1:4,1:4);
end

function vertices = localBernstein(polynomial,box)
    coefficients = polynomial;
    for dimension = 1:3
        degree = size(polynomial,dimension)-1;
        lower = box(dimension,1); width = box(dimension,2)-lower;
        transform = zeros(degree+1);
        for row = 0:degree
            for power = 0:degree
                for index = 0:min(row,power)
                    transform(row+1,power+1) = transform(row+1,power+1) ...
                        +nchoosek(power,index)*lower^(power-index)*width^index ...
                            *nchoosek(row,index)/nchoosek(degree,index);
                end
            end
        end
        order = [dimension,setdiff(1:4,dimension,'stable')];
        permuted = permute(coefficients,order);
        shape = size(permuted,1:4);
        permuted = reshape(transform*reshape(permuted,shape(1),[]),shape);
        coefficients = ipermute(permuted,order);
    end
    coefficients = reshape(coefficients,[],9).';
    if any(coefficients(9,:)<=0) || any(~isfinite(coefficients),'all')
        error("nrmmTargetJacobian:invalidBernsteinDenominator", ...
            "The rational enclosure requires positive finite denominator coefficients.");
    end
    vertices = coefficients(1:8,:)./coefficients(9,:);
end

function [jacobian,field] = localJacobian(q,s,domain)
    validateattributes(q,{'double'},{'real','finite','size',[2,1]});
    validateattributes(s,{'double'},{'real','finite','size',[2,1]});
    r = norm(q);
    if r == 0
        jacobian = zeros(2,4);
        field = zeros(2,1);
        return
    end
    a = domain(1); b = domain(2); acceleration = domain(3); yaw = domain(4); cap = domain(5);
    cross = [0,-1;1,0];
    direction = q/r;
    denominator = max(r,a);
    denominatorDerivative = zeros(1,2);
    directionDerivative = eye(2)/a;
    if r >= a
        denominatorDerivative = direction.';
        directionDerivative = (eye(2)-direction*direction.')/r;
    end
    velocity = min(1,b/r)*q;
    velocityDerivative = eye(2);
    if r >= b
        velocityDerivative = (b/r)*(eye(2)-direction*direction.');
    end
    projected = min(1,cap/max(norm(s),realmin))*s;
    accelerationDerivative = eye(2);
    if norm(s) >= cap
        accelerationDerivative = (cap/norm(s))*(eye(2)-(s*s.')/norm(s)^2);
    end
    rawAlpha = velocity.'*projected/denominator;
    rawNu = (cross*velocity).'*projected/denominator^2;
    alpha = min(max(rawAlpha,-acceleration),acceleration);
    nu = min(max(rawNu,-yaw),yaw);
    alphaDerivative = double(abs(rawAlpha)<acceleration)*[ ...
        projected.'*velocityDerivative/denominator-rawAlpha*denominatorDerivative/denominator, ...
        velocity.'*accelerationDerivative/denominator];
    nuDerivative = double(abs(rawNu)<yaw)*[ ...
        projected.'*cross*velocityDerivative/denominator^2-2*rawNu*denominatorDerivative/denominator, ...
        (cross*velocity).'*accelerationDerivative/denominator^2];
    crossDirection = cross*q/denominator;
    field = -nu^2*velocity+3*alpha*nu*crossDirection;
    jacobian = -nu^2*[velocityDerivative,zeros(2)] ...
        +(-2*nu*velocity+3*alpha*crossDirection)*nuDerivative ...
        +3*nu*crossDirection*alphaDerivative ...
        +3*alpha*nu*[cross*directionDerivative,zeros(2)];
end

function value = localPositionVelocity(action,varargin)
%nrmmObserverVectorField positionVelocity Position-only, IMU-aided inertial velocity observer.
% The four numerical states are a position and a velocity. GNSS supplies
% position only. Each signed enclosure column multiplies an independent
% two-dimensional unit ball; signs are retained through the filter dynamics.
    switch string(action)
        case "initialize"
            value=localInitialize(varargin{:});
        case "measure"
            value=localMeasure(varargin{:});
        case "predict"
            value=localPredict(varargin{:});
        case "pointMeasure"
            value=localPointMeasure(varargin{:});
        case "pointPredict"
            value=localPointPredict(varargin{:});
        otherwise
            error('nrmmPositionVelocity:invalidAction','Unknown action.');
    end
end

function filter=localInitialize(position,velocity,positionRadius,velocityRadius,dt,bandwidth)
    validateattributes(position,{'double'},{'vector','numel',2,'real','finite'});
    validateattributes(velocity,{'double'},{'vector','numel',2,'real','finite'});
    validateattributes(dt,{'double'},{'scalar','finite','positive'});
    validateattributes(bandwidth,{'double'},{'scalar','finite','positive'});
    validateattributes(positionRadius,{'double'},{'scalar','finite','nonnegative'});
    validateattributes(velocityRadius,{'double'},{'scalar','finite','nonnegative'});
    pole=exp(-bandwidth*dt);
    % Both posterior error poles equal pole; beta has units 1/s.
    gain=[1-pole^2;(1-pole)^2/dt];
    filter=struct('point',[position(:);velocity(:)],'gain',gain, ...
        'generator',diag([positionRadius,velocityRadius]), ...
        'samplePeriod',dt,'bandwidth',bandwidth, ...
        'time',0,'historyTime',zeros(1,0),'historyPosition',zeros(2,0), ...
        'historyRadius',zeros(1,0),'historyAcceleration',zeros(1,0), ...
        'kind',"position-only-imu-velocity-v1");
end

function filter=localMeasure(filter,position,positionRadius)
    % Position secants are an independent, bounded-noise velocity enclosure.
    % They remain informative when a global intersample jerk bound is loose.
    % Retain actual GNSS samples; deterministic noise is never averaged down.
    keep=filter.historyTime<filter.time & filter.historyTime>=filter.time-4/filter.bandwidth;
    filter.historyTime=[filter.historyTime(keep),filter.time];
    filter.historyPosition=[filter.historyPosition(:,keep),position(:)];
    filter.historyRadius=[filter.historyRadius(keep),positionRadius];
    filter.historyAcceleration=[filter.historyAcceleration(keep),Inf];
    if isfield(filter,'initialPositionPending') && filter.initialPositionPending
        % The runtime's initial position is only a numerical guess. Anchor
        % this new filter to the first actual GNSS sample before correcting
        % velocity; a single position cannot supply a velocity observation.
        filter.point(1:2)=position(:);
        filter.generator(1,:)=0;
        filter.generator=[filter.generator,[positionRadius;0]];
        radius=sum(abs(filter.generator),2);
        filter.positionRadius=radius(1);filter.velocityRadius=radius(2);
        filter.initialPositionPending=false;
        return;
    end
    filter.point=localPointMeasure(filter.point,position,filter.gain);
    correction=eye(2)-filter.gain*[1,0];
    filter.generator=[correction*filter.generator,filter.gain*positionRadius];
    radius=sum(abs(filter.generator),2);
    filter.positionRadius=radius(1);filter.velocityRadius=radius(2);
    available=filter.velocityRadius;maximum=0;
    for k=numel(filter.historyTime)-1:-1:1
        maximum=max(maximum,filter.historyAcceleration(k));
        age=filter.time-filter.historyTime(k);
        center=(position(:)-filter.historyPosition(:,k))/age;
        bound=(positionRadius+filter.historyRadius(k))/age+maximum*age/2;
        available=min(available,norm(center-filter.point(3:4))+bound);
    end
    if available<filter.velocityRadius
        % An intersection cannot be represented by scaling old signed
        % columns. Keep the position projection and take its Cartesian
        % product with the new velocity ball; this encloses the intersection.
        filter.generator(2,:)=0;filter.generator=[filter.generator,[0;available]];
        filter.velocityRadius=available;
    end
end

function filter=localPredict(filter,acceleration,yaw,yawRadius,design,holdBounds)
    dt=filter.samplePeriod;
    filter.point=localPointPredict(filter.point,acceleration,yaw,dt);
    amax=holdBounds.acceleration;
    if ~isempty(filter.historyAcceleration),filter.historyAcceleration(end)=amax;end
    filter.time=filter.time+dt;
    % Compare the held world acceleration with the actual interval. IMU
    % noise, yaw uncertainty and intersample acceleration variation are all
    % included. Without a jerk envelope use the declared acceleration domain.
    mismatch=amax+norm(acceleration);
    jerk=holdBounds.bodyAccelerationRate+design.operatingDomain.egoYawRateMaximum*amax;
    localMismatch=design.sensors.accelerometerNoiseMaximum ...
        +2*norm(acceleration)*sin(min(pi,yawRadius)/2)+jerk*dt;
    mismatch=min(mismatch,localMismatch);
    transition=[1,dt;0,1];
    if isfinite(mismatch)
        % The two integrals of a varying disturbance need not share a sign.
        forcing=diag([dt^2/2,dt])*mismatch;
    else
        % Finite speed domain, with no unjustified intersample acceleration
        % premise. This remains an enclosure, although usually very loose.
        vmax=design.operatingDomain.egoSpeedMaximum;
        forcing=diag([2*vmax*dt+norm(acceleration)*dt^2/2, ...
            2*vmax+norm(acceleration)*dt]);
    end
    filter.generator=[transition*filter.generator,forcing];
end

function point=localPointMeasure(point,position,gain)
    residual=position(:)-point(1:2);
    point=point+[gain(1)*residual;gain(2)*residual];
end

function point=localPointPredict(point,acceleration,yaw,dt)
    rotation=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
    a=rotation*acceleration(:);
    point=[point(1:2)+dt*point(3:4)+dt^2*a/2;point(3:4)+dt*a];
end
