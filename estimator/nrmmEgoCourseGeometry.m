function value = nrmmEgoCourseGeometry(action, varargin)
% nrmmEgoCourseGeometry Ego lateral-velocity measurement and course-heading correspondence.
%
%   course = nrmmEgoCourseGeometry("course", observerInput, design)
%       With a shared vehicle model (design.egoModel) and the held controller
%       input (observerInput.heldInput), the lateral body velocity is certified
%       by the force balance ("lateralVelocity" below). The sideslip interval
%       follows from sin(beta) = v_y/||v|| over the GNSS speed interval, and
%       the heading correspondence is angle(GNSS velocity) - beta. Without them
%       the kinematic single-track relation v_y = l*r with its declared
%       mismatch bound is used, exactly as before ("kinematicCourse" below).
%       Both branches return the course fields consumed by the runtime and the
%       bound recursion, plus lateralMeasurement = [center, lower, upper,
%       radius, consistent, source] for the body-velocity observer.
%
%   course = nrmmEgoCourseGeometry("kinematicCourse", gnssVelocity, yawRateMeasured, ...
%       rearAxleDistance, velocityErrorMaximum, yawRateErrorMaximum, ...
%       modelYawRateMismatchMaximum, sideslipDomainMaximum)
%       Certify yaw from course and gyro under the ego single-track relation
%       with bounded mismatch, dSt = omegaE - VE*sin(betaE)/lrE,
%       |dSt| <= modelYawRateMismatchMaximum. With u3 = omegaE+dOmega,
%       ||vm-vE|| <= velocityErrorMaximum and Vm = ||vm||, the online sideslip
%       estimate is betaHat = asin(lrE*u3/Vm). Whenever Vm exceeds the
%       velocity-error radius, the sine error obeys
%           |sin(betaE)-lrE*u3/Vm| <= lrE/(Vm-velocityErrorMaximum) *
%               (yawRateErrorMaximum+modelYawRateMismatchMaximum
%                +abs(u3)*velocityErrorMaximum/Vm).
%       Intersecting that interval with the declared principal-branch domain
%       |betaE| <= sideslipDomainMaximum < pi/2 and mapping its endpoints
%       through asin gives an exact pointwise bound on |betaE-betaHat|. The
%       returned rotation correspondence uses R(betaHat)*e1 as its measured
%       body vector, so its heading is angle(vm)-betaHat and its model-angle
%       radius is the certified sideslip-estimation error. The channel becomes
%       uninformative at low speed, for an inadmissible asin argument, for
%       inconsistent declared bounds, or when the resulting yaw arc is not
%       proper. Identical complete inputs are served from a cache.
%
%   correspondence = nrmmEgoCourseGeometry("rotation", inertialVector, bodyVector, ...
%       inertialErrorMaximum, bodyErrorMaximum, modelAngleMaximum)
%       Certify one planar rotation measurement. The ideal vectors satisfy
%       aI = R(psi)*bE. Bounded perturbations of the measured vectors give the
%       pseudo-heading h = angle(aIMeasured)-angle(bEMeasured) and the
%       certified radius r = delta(aIMeasured, epsilonI) + delta(bEMeasured,
%       epsilonE) + betaMaximum. The correspondence is informative only when
%       both uncertainty balls exclude the origin and the resulting circular
%       arc is proper (r < pi); otherwise radius is Inf and no yaw direction
%       is claimed.
%
%   lateral = nrmmEgoCourseGeometry("lateralVelocity", gnssVelocity, yawRate, ...
%       bodyAcceleration, heldInput, model, sensors, sideslipDomainMaximum)
%       Certified ego lateral body velocity from the lateral force balance at
%       the centre of mass,
%           m*a_y = Fx_f(b)*sin(d) + Fy_f(alpha_f)*cos(d) + Fy_r(alpha_r),
%           alpha_f = atan2(v_y + lf*r, v_x) - d,  alpha_r = atan2(v_y - lr*r, v_x),
%       with the same modified Fiala tires that the controller predicts with
%       and the held steering d and braking ratio b that the controller
%       applied. Each axle force is non-increasing in v_y, so the measured a_y
%       determines v_y; this replaces the kinematic no-slip relation, which
%       fails whenever the rear tire slips. The true quantities satisfy
%       |a_y - am_y| <= epsA, |r - u| <= epsW, ||v|| in [M - epsG, M + epsG],
%       the sideslip cone, and every cornering stiffness and friction
%       coefficient within the relative parameterUncertainty of the model
%       value. Each axle force is monotone in those quantities, so its
%       extremes over the box are attained at the corners; with Gmin, Gmax the
%       per-axle sums, every admissible v_y satisfies Gmin(v_y) <= m*(am_y +
%       epsA) and Gmax(v_y) >= m*(am_y - epsA), an interval [lower, upper]
%       found by bisection (empty when the measurements contradict the model
%       or the cone). The returned center solves the balance with the nominal
%       parameters and the measured values; radius = max(center-lower,
%       upper-center) certifies it. The midpoint is not used: near saturation
%       the interval is wide and asymmetric and its midpoint is biased. model
%       carries mass, lf, lr, gravity, corneringStiffness, frictionCoefficient
%       and parameterUncertainty; sensors carries velocityNoiseMaximum,
%       gyroscopeNoiseMaximum and accelerometerNoiseMaximum.

    switch string(action)
        case "course"
            value = localCourse(varargin{1}, varargin{2});
        case "kinematicCourse"
            value = localKinematicCourse(varargin{:});
        case "rotation"
            value = localRotationCorrespondence(varargin{:});
        case "lateralVelocity"
            value = localModelLateralVelocity(varargin{:});
        otherwise
            error("nrmmEgoCourseGeometry:invalidAction", ...
                "action must be 'course', 'kinematicCourse', 'rotation' or 'lateralVelocity'.");
    end
end

%% Course geometry with or without the shared vehicle model

function course = localCourse(observerInput, design)
    model = design.yaw.courseModel;
    sensors = design.sensors;
    useModel = isfield(design,"egoModel") && ~isempty(design.egoModel) ...
        && isfield(observerInput,"heldInput") && numel(observerInput.heldInput)==2 ...
        && all(isfinite(observerInput.heldInput));
    if ~useModel
        course = localKinematicCourse(observerInput.gnssVelocity, ...
            observerInput.yawRate,model.rearAxleDistance,sensors.velocityNoiseMaximum, ...
            sensors.gyroscopeNoiseMaximum,model.singleTrackYawRateMismatchMaximum, ...
            model.sideslipDomainMaximum);
        [~,feasible] = nrmmObserverVectorField("velocityMeasurement", ...
            observerInput.gnssVelocity,observerInput.yawRate,design);
        lateralError = model.rearAxleDistance*(sensors.gyroscopeNoiseMaximum ...
            +model.singleTrackYawRateMismatchMaximum);
        center = model.rearAxleDistance*observerInput.yawRate;
        course.lateralMeasurement = struct("center",center, ...
            "lower",center-lateralError,"upper",center+lateralError,"radius",lateralError, ...
            "consistent",feasible.consistent,"informative",true,"source","kinematic");
        return;
    end

    lateral = localModelLateralVelocity(observerInput.gnssVelocity,observerInput.yawRate, ...
        observerInput.bodyAcceleration,observerInput.heldInput,design.egoModel,sensors, ...
        model.sideslipDomainMaximum);
    measuredSpeed = norm(observerInput.gnssVelocity);
    speedLow = measuredSpeed-sensors.velocityNoiseMaximum;
    speedHigh = measuredSpeed+sensors.velocityNoiseMaximum;
    sineDomain = sin(model.sideslipDomainMaximum);
    estimatedSideslip = NaN;sideslipError = Inf;interval = [NaN;NaN];sineInterval = [NaN;NaN];
    sineError = Inf;consistent = lateral.consistent && speedLow > 0;
    if consistent
        % sin(beta) = v_y/||v|| over v_y in [lower, upper], ||v|| in [speedLow, speedHigh].
        candidates = [lateral.lower,lateral.upper]./[speedLow;speedHigh];
        sineInterval = [max(-sineDomain,min(candidates(:)));min(sineDomain,max(candidates(:)))];
        consistent = sineInterval(1) <= sineInterval(2)+1e-12;
        if consistent
            sineInterval = sort(sineInterval);
            estimatedSine = max(-sineDomain,min(sineDomain,lateral.center/measuredSpeed));
            estimatedSideslip = asin(estimatedSine);
            interval = asin(sineInterval);
            sideslipError = max(abs(interval-estimatedSideslip));
            sineError = max(abs(sineInterval-estimatedSine));
        end
    end
    if consistent && sideslipError < pi
        bodyDirection = [cos(estimatedSideslip);sin(estimatedSideslip)];
        correspondence = localRotationCorrespondence(observerInput.gnssVelocity, ...
            bodyDirection,sensors.velocityNoiseMaximum,0.0,sideslipError);
    else
        correspondence = localRotationCorrespondence(observerInput.gnssVelocity, ...
            [1;0],sensors.velocityNoiseMaximum,0.0,0.0);
        correspondence.radius = Inf;correspondence.informative = false;
        correspondence.rawRadius = Inf;correspondence.modelAngleMaximum = Inf;
    end
    course = struct("correspondence",correspondence,"measuredSpeed",measuredSpeed, ...
        "trueSpeedLowerBound",speedLow,"speedCertificateValid",speedLow > 0, ...
        "normalizedYawRate",lateral.center/max(measuredSpeed,eps), ...
        "estimatedSideslip",estimatedSideslip, ...
        "speedErrorContribution",abs(lateral.center)*sensors.velocityNoiseMaximum/max(measuredSpeed*speedLow,eps), ...
        "yawRateAndModelContribution",lateral.radius/max(speedLow,eps), ...
        "sineErrorMaximum",sineError,"trueSideslipSineInterval",sineInterval, ...
        "trueSideslipInterval",interval,"sideslipErrorMaximum",sideslipError, ...
        "inversionValid",lateral.consistent,"boundsConsistent",consistent, ...
        "rearAxleDistance",model.rearAxleDistance, ...
        "velocityErrorMaximum",sensors.velocityNoiseMaximum, ...
        "yawRateErrorMaximum",sensors.gyroscopeNoiseMaximum, ...
        "modelYawRateMismatchMaximum",NaN, ...
        "sideslipDomainMaximum",model.sideslipDomainMaximum);
    lateral.consistent = consistent;
    course.lateralMeasurement = lateral;
end

%% Certified kinematic course correspondence

function course = localKinematicCourse( ...
        gnssVelocity, yawRateMeasured, rearAxleDistance, ...
        velocityErrorMaximum, yawRateErrorMaximum, ...
        modelYawRateMismatchMaximum, sideslipDomainMaximum)
    % The synchronized runtime, measurement enclosure and history update
    % request the same held geometry. Cache only identical complete inputs;
    % a changed value or shape still takes the full validation path.
    persistent cachedInputs cachedCourse
    inputs = {gnssVelocity,yawRateMeasured,rearAxleDistance,velocityErrorMaximum, ...
        yawRateErrorMaximum,modelYawRateMismatchMaximum,sideslipDomainMaximum};
    if isequaln(inputs,cachedInputs)
        course = cachedCourse;
        return;
    end
    gnssVelocity = localPlanarVector(gnssVelocity, "gnssVelocity");
    yawRateMeasured = localFiniteScalar( ...
        yawRateMeasured, "yawRateMeasured");
    rearAxleDistance = localPositiveScalar( ...
        rearAxleDistance, "rearAxleDistance");
    velocityErrorMaximum = localNonnegativeScalar( ...
        velocityErrorMaximum, "velocityErrorMaximum");
    yawRateErrorMaximum = localNonnegativeScalar( ...
        yawRateErrorMaximum, "yawRateErrorMaximum");
    modelYawRateMismatchMaximum = localNonnegativeScalar( ...
        modelYawRateMismatchMaximum, ...
        "modelYawRateMismatchMaximum");
    sideslipDomainMaximum = localNonnegativeScalar( ...
        sideslipDomainMaximum, "sideslipDomainMaximum");
    if sideslipDomainMaximum >= 0.5*pi
        error("nrmmEgoCourseGeometry:invalidSideslipDomain", ...
            "sideslipDomainMaximum must be smaller than pi/2 radians.");
    end

    measuredSpeed = norm(gnssVelocity);
    trueSpeedLowerBound = measuredSpeed-velocityErrorMaximum;
    speedCertificateValid = trueSpeedLowerBound > 0.0;
    normalizedYawRate = NaN;
    estimatedSideslip = NaN;
    speedErrorContribution = Inf;
    yawRateAndModelContribution = Inf;
    sineErrorMaximum = Inf;
    trueSideslipSineInterval = [NaN; NaN];
    trueSideslipInterval = [NaN; NaN];
    sideslipErrorMaximum = Inf;
    inversionValid = false;
    boundsConsistent = false;

    if measuredSpeed > 0.0
        normalizedYawRate = rearAxleDistance*yawRateMeasured/measuredSpeed;
        inversionValid = abs(normalizedYawRate) <= 1.0;
    end

    if speedCertificateValid && inversionValid
        estimatedSideslip = asin(normalizedYawRate);
        speedErrorContribution = rearAxleDistance ...
            * abs(yawRateMeasured)*velocityErrorMaximum ...
            / (measuredSpeed*trueSpeedLowerBound);
        yawRateAndModelContribution = rearAxleDistance ...
            * (yawRateErrorMaximum+modelYawRateMismatchMaximum) ...
            / trueSpeedLowerBound;
        sineErrorMaximum = speedErrorContribution ...
            + yawRateAndModelContribution;
        sineDomainMaximum = sin(sideslipDomainMaximum);
        sineLower = max( ...
            -sineDomainMaximum, normalizedYawRate-sineErrorMaximum);
        sineUpper = min( ...
            sineDomainMaximum, normalizedYawRate+sineErrorMaximum);
        consistencyTolerance = 128.0*eps(max(1.0, ...
            max(abs([sineLower, sineUpper, normalizedYawRate]))));
        boundsConsistent = sineLower <= sineUpper+consistencyTolerance;
        if boundsConsistent
            if sineLower > sineUpper
                sineMidpoint = 0.5*(sineLower+sineUpper);
                sineLower = sineMidpoint;
                sineUpper = sineMidpoint;
            end
            trueSideslipSineInterval = [sineLower; sineUpper];
            trueSideslipInterval = asin(trueSideslipSineInterval);
            sideslipErrorMaximum = max(abs( ...
                trueSideslipInterval-estimatedSideslip));
        end
    end

    if isfinite(estimatedSideslip)
        measuredBodyDirection = [ ...
            cos(estimatedSideslip); sin(estimatedSideslip)];
    else
        measuredBodyDirection = [1.0; 0.0];
    end
    if boundsConsistent
        correspondence = localRotationCorrespondence( ...
            gnssVelocity, measuredBodyDirection, ...
            velocityErrorMaximum, 0.0, sideslipErrorMaximum);
    else
        correspondence = localRotationCorrespondence( ...
            gnssVelocity, measuredBodyDirection,velocityErrorMaximum,0.0,0.0);
        correspondence.radius = Inf;
        correspondence.informative = false;
        correspondence.rawRadius = Inf;
        correspondence.modelAngleMaximum = Inf;
    end

    course = struct( ...
        "correspondence", correspondence, ...
        "measuredSpeed", measuredSpeed, ...
        "trueSpeedLowerBound", trueSpeedLowerBound, ...
        "speedCertificateValid", speedCertificateValid, ...
        "normalizedYawRate", normalizedYawRate, ...
        "estimatedSideslip", estimatedSideslip, ...
        "speedErrorContribution", speedErrorContribution, ...
        "yawRateAndModelContribution", ...
            yawRateAndModelContribution, ...
        "sineErrorMaximum", sineErrorMaximum, ...
        "trueSideslipSineInterval", ...
            trueSideslipSineInterval, ...
        "trueSideslipInterval", trueSideslipInterval, ...
        "sideslipErrorMaximum", sideslipErrorMaximum, ...
        "inversionValid", inversionValid, ...
        "boundsConsistent", boundsConsistent, ...
        "rearAxleDistance", rearAxleDistance, ...
        "velocityErrorMaximum", velocityErrorMaximum, ...
        "yawRateErrorMaximum", yawRateErrorMaximum, ...
        "modelYawRateMismatchMaximum", ...
            modelYawRateMismatchMaximum, ...
        "sideslipDomainMaximum", sideslipDomainMaximum);
    cachedInputs = inputs;
    cachedCourse = course;
end

function vector = localPlanarVector(vector, name)
    vector = double(vector);
    if ~isequal(size(vector), [2, 1]) || any(~isfinite(vector))
        error("nrmmEgoCourseGeometry:invalidVector", ...
            "%s must be a finite two-vector.", name);
    end
end

function value = localFiniteScalar(value, name)
    value = double(value);
    if ~isscalar(value) || ~isfinite(value)
        error("nrmmEgoCourseGeometry:invalidScalar", ...
            "%s must be a finite scalar.", name);
    end
end

function value = localPositiveScalar(value, name)
    value = localFiniteScalar(value, name);
    if value <= 0.0
        error("nrmmEgoCourseGeometry:invalidPositive", ...
            "%s must be strictly positive.", name);
    end
end

function value = localNonnegativeScalar(value, name)
    value = localFiniteScalar(value, name);
    if value < 0.0
        error("nrmmEgoCourseGeometry:invalidNonnegative", ...
            "%s must be nonnegative.", name);
    end
end

%% Certified planar rotation correspondence

function correspondence = localRotationCorrespondence( ...
        inertialVector, bodyVector, inertialErrorMaximum, ...
        bodyErrorMaximum, modelAngleMaximum)
    inertialVector = localPlanarVector(inertialVector, "inertialVector");
    bodyVector = localPlanarVector(bodyVector, "bodyVector");
    inertialErrorMaximum = localNonnegativeScalar( ...
        inertialErrorMaximum, "inertialErrorMaximum");
    bodyErrorMaximum = localNonnegativeScalar( ...
        bodyErrorMaximum, "bodyErrorMaximum");
    modelAngleMaximum = localNonnegativeScalar( ...
        modelAngleMaximum, "modelAngleMaximum");
    if modelAngleMaximum >= pi
        error("nrmmEgoCourseGeometry:invalidModelAngle", ...
            "modelAngleMaximum must be smaller than pi radians.");
    end

    [inertialRadius, inertialInformative, inertialMagnitude] = ...
        localDirectionErrorRadius(inertialVector, inertialErrorMaximum);
    [bodyRadius, bodyInformative, bodyMagnitude] = ...
        localDirectionErrorRadius(bodyVector, bodyErrorMaximum);
    rotationCosine = dot(bodyVector, inertialVector);
    rotationSine = bodyVector(1)*inertialVector(2) ...
        - bodyVector(2)*inertialVector(1);
    heading = atan2(rotationSine, rotationCosine);
    rawRadius = inertialRadius+bodyRadius+modelAngleMaximum;
    informative = inertialInformative && bodyInformative ...
        && rawRadius < pi;
    if informative
        radius = rawRadius;
    else
        radius = Inf;
    end

    correspondence = struct( ...
        "heading", heading, ...
        "radius", radius, ...
        "informative", informative, ...
        "rawRadius", rawRadius, ...
        "inertialDirectionRadius", inertialRadius, ...
        "bodyDirectionRadius", bodyRadius, ...
        "modelAngleMaximum", modelAngleMaximum, ...
        "inertialMagnitude", inertialMagnitude, ...
        "bodyMagnitude", bodyMagnitude);
end

function [radius, informative, magnitude] = ...
        localDirectionErrorRadius(measuredVector, errorMaximum)
% localDirectionErrorRadius Exact tangent-cone direction-error radius.
%
% If z = x+d with ||d|| <= epsilon and ||z|| > epsilon, the true vector
% x lies inside the origin-centered tangent cone through the uncertainty
% ball around z, so the angle between z and x is at most
% asin(epsilon/||z||). When ||z|| <= epsilon, that ball contains the
% origin and the measurement certifies no direction.

    magnitude = norm(measuredVector);
    informative = magnitude > errorMaximum;
    if informative
        radius = asin(min(1.0, errorMaximum/magnitude));
    else
        radius = Inf;
    end
end

%% Force-balance lateral velocity with the shared vehicle model

function lateral = localModelLateralVelocity(gnssVelocity, yawRate, bodyAcceleration, heldInput, model, sensors, sideslipDomainMaximum)
    speed = norm(gnssVelocity);
    speedLow = speed-sensors.velocityNoiseMaximum;
    speedHigh = speed+sensors.velocityNoiseMaximum;
    sineDomain = sin(sideslipDomainMaximum);
    lateral = struct("center",NaN,"lower",-Inf,"upper",Inf,"radius",Inf, ...
        "consistent",true,"informative",false,"source","force-balance", ...
        "measuredSpeed",speed,"trueSpeedLowerBound",speedLow);
    if ~(speedLow > 0) || numel(heldInput) ~= 2 || any(~isfinite(heldInput)) || abs(heldInput(2)) > 1
        lateral.lower = -speedHigh*sineDomain;lateral.upper = speedHigh*sineDomain;
        lateral.center = 0;lateral.radius = speedHigh*sineDomain;
        return;
    end
    steering = heldInput(1);braking = heldInput(2);
    accelerationLow = model.mass*(bodyAcceleration(2)-sensors.accelerometerNoiseMaximum);
    accelerationHigh = model.mass*(bodyAcceleration(2)+sensors.accelerometerNoiseMaximum);
    rates = yawRate+[-1,1]*sensors.gyroscopeNoiseMaximum;
    spread = model.parameterUncertainty*[-1,1]+1;
    wheelbase = model.lf+model.lr;
    load = model.mass*model.gravity/wheelbase*[model.lr;model.lf];
    eta = sqrt(max(0,1-braking^2));
    % Corner grids: rate x speed x stiffness x friction (2x2x2x2 per axle).
    [rateGrid,speedIndex,stiffnessGrid,frictionGrid] = ndgrid(rates,[1,2],spread,spread);
    rateGrid = rateGrid(:).';speedIndex = speedIndex(:).';
    stiffnessGrid = stiffnessGrid(:).';frictionGrid = frictionGrid(:).';
    longitudinal = braking*model.frictionCoefficient(1)*spread*load(1)*sin(steering);
    longitudinalRange = [min(longitudinal),max(longitudinal)];

    left = -speedHigh*sineDomain;right = speedHigh*sineDomain;
    [lowAtLeft,highAtLeft] = localRange(left);[lowAtRight,highAtRight] = localRange(right);
    % lower: smallest v_y with Gmin(v_y) <= m*(a_y + epsA).
    if lowAtLeft <= accelerationHigh
        lower = left;
    elseif lowAtRight > accelerationHigh
        lower = Inf;
    else
        lower = localBisect(left,right,@(v) localMinimum(v) <= accelerationHigh);
    end
    % upper: largest v_y with Gmax(v_y) >= m*(a_y - epsA).
    if highAtRight >= accelerationLow
        upper = right;
    elseif highAtLeft < accelerationLow
        upper = -Inf;
    else
        upper = localBisect(left,right,@(v) localMaximum(v) < accelerationLow);
    end
    lateral.lower = lower;lateral.upper = upper;
    lateral.consistent = lower <= upper;
    if lateral.consistent
        nominal = localNominal(left,right);
        if ~(isfinite(nominal) && nominal >= lower && nominal <= upper)
            nominal = 0.5*(lower+upper);
        end
        lateral.center = nominal;
        lateral.radius = max(nominal-lower,upper-nominal);
        lateral.informative = isfinite(lateral.radius);
    else
        lateral.center = NaN;lateral.radius = Inf;
    end

    function value = localNominal(leftEnd,rightEnd)
        % Nominal parameters and measured values: G0(v) = m*am_y.
        target = model.mass*bodyAcceleration(2);
        g = @(v) localNominalForce(v);
        if g(leftEnd) < target || g(rightEnd) > target
            value = NaN;return;
        end
        value = localBisect(leftEnd,rightEnd,@(v) g(v) < target);
    end
    function force = localNominalForce(v)
        longitudinalSpeed = sqrt(max(speed^2-v^2,(speed*cos(sideslipDomainMaximum))^2));
        slips = atan2([v+model.lf*yawRate;v-model.lr*yawRate],longitudinalSpeed)-[steering;0];
        forces = localFiala(slips,model.corneringStiffness(:), ...
            model.frictionCoefficient(:).*load*eta);
        force = braking*model.frictionCoefficient(1)*load(1)*sin(steering) ...
            +forces(1)*cos(steering)+forces(2);
    end

    function value = localMinimum(v)
        [value,~] = localRange(v);
    end
    function value = localMaximum(v)
        [~,value] = localRange(v);
    end
    function [low,high] = localRange(v)
        % Per-axle extremes over the corner grid at lateral velocity v.
        % Admissible speeds keep v inside the cone; the search interval ends
        % exactly on the cone boundary at the largest speed.
        speeds = [min(speedHigh,max(speedLow,abs(v)/max(sineDomain,eps))),speedHigh];
        longitudinalSpeed = sqrt(max(speeds(speedIndex).^2-v^2,0));
        frontSlip = atan2(v+model.lf*rateGrid,longitudinalSpeed)-steering;
        rearSlip = atan2(v-model.lr*rateGrid,longitudinalSpeed);
        front = localFiala(frontSlip,model.corneringStiffness(1)*stiffnessGrid, ...
            model.frictionCoefficient(1)*frictionGrid*load(1)*eta)*cos(steering);
        rear = localFiala(rearSlip,model.corneringStiffness(2)*stiffnessGrid, ...
            model.frictionCoefficient(2)*frictionGrid*load(2)*eta);
        low = longitudinalRange(1)+min(front)+min(rear);
        high = longitudinalRange(2)+max(front)+max(rear);
    end
end

function force = localFiala(slip,stiffness,capacity)
% Modified Fiala lateral force, identical to modifiedFialaTire.evaluate.
    tangent = tan(slip);
    force = -capacity.*sign(slip);
    adhesion = capacity > 0 & abs(tangent) < 3*capacity./stiffness;
    q = stiffness(adhesion).*abs(tangent(adhesion))./(3*capacity(adhesion));
    force(adhesion) = -stiffness(adhesion).*tangent(adhesion).*(1-q+q.^2/3);
end

function value = localBisect(left,right,predicate)
% First point of [left,right] where the monotone predicate becomes true,
% to 1e-5 m/s; predicate(left) is false and predicate(right) is true.
    while right-left > 1e-5
        middle = 0.5*(left+right);
        if predicate(middle),right = middle;else,left = middle;end
    end
    value = right;
end
