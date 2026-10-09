function [first, second] = nrmmObserverVectorField(action, varargin)
% nrmmObserverVectorField Continuous observer equations and their RK4 interval.
%
% The eight-state core for one target consists of body velocity and [rho;q;s].
% Absolute yaw is used only by the output reconstruction, never by the core
% field. The independent inertial position output uses the GNSS velocity
% directly. Synchronization, orientation sets and error bounds remain outside.
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
%       toward the certified course heading when it is informative. It
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
                "action must be 'derivative', 'integrate', 'yawDerivative' or 'velocityMeasurement'.");
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
    targetPlantDerivative = nrmmTargetTrackerDerivative(targetState,ego,design.target.domain);
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
    derivative = measuredRate;
    if correspondence.informative
        heading = correspondence.heading;
        validateattributes(heading, {'double'}, {'real','finite','scalar'});
        innovation = mod(heading-yaw+pi,2*pi)-pi;
        derivative = derivative+gain*innovation;
    end
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
