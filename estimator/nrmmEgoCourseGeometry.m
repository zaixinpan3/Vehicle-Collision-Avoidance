function [value,jacobian,info] = nrmmEgoCourseGeometry(action, varargin)
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
%   point = nrmmEgoCourseGeometry("pointCourse", observerInput, design)
%       The identical numerical lateral measurement and rotation innovation,
%       without interval calculations. This supplies no certificate fields.
%   [value,J,info] = nrmmEgoCourseGeometry("pointMeasurement", input, design)
%       Body velocity and rotation innovation, with their 4-by-9 Jacobian in
%       [position(2), reconstructed velocity(2), acceleration(2), gyro,
%       steering, braking]. One point inverse supplies all smooth derivatives.
%       Singular branches use reported finite-step secants, not certificates.
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
%       or the cone). The returned center solves the regularized nominal
%       force inversion; radius = max(center-lower,
%       upper-center) certifies it. The midpoint is not used: near saturation
%       the interval is wide and asymmetric and its midpoint is biased. model
%       carries mass, lf, lr, gravity, corneringStiffness, frictionCoefficient
%       and parameterUncertainty; sensors carries velocityNoiseMaximum,
%       gyroscopeNoiseMaximum and accelerometerNoiseMaximum.

    switch string(action)
        case "course"
            value = localCourse(varargin{1}, varargin{2});
        case "pointCourse"
            value = localPointCourse(varargin{1},varargin{2});
        case "pointMeasurement"
            [value,jacobian,info] = localPointMeasurement(varargin{1},varargin{2});
        case "kinematicCourse"
            value = localKinematicCourse(varargin{:});
        case "rotation"
            value = localRotationCorrespondence(varargin{:});
        case "lateralVelocity"
            value = localModelLateralVelocity(varargin{:});
        otherwise
            error("nrmmEgoCourseGeometry:invalidAction", ...
                "Use 'course', 'pointCourse', 'pointMeasurement', 'kinematicCourse', 'rotation' or 'lateralVelocity'.");
    end
end

%% Course geometry with or without the shared vehicle model

function [value,jacobian,info]=localPointMeasurement(input,design)
    if isfield(input,'courseGeometry'),course=input.courseGeometry;
    else,course=localPointCourse(input,design);end
    lateral=course.lateralMeasurement.center;
    useModel=course.lateralMeasurement.source~="kinematic" && isfinite(lateral);
    if ~useModel,lateral=design.yaw.courseModel.rearAxleDistance*input.yawRate;end
    inertial=input.gnssVelocity;
    body=nrmmObserverVectorField('velocityMeasurement',inertial,input.yawRate,design,lateral);
    value=[body;course.correspondence.rotationVector];
    if nargout==1,return;end
    speed=norm(inertial);beta=design.yaw.courseModel.sideslipDomainMaximum;
    ds=zeros(1,9);dy=zeros(1,9);jacobian=zeros(4,9);fallback=false(1,9);
    info=struct('method',"implicitPointInverse",'branch',"kinematic", ...
        'secantColumns',false(1,9),'nonlinearCertificate',false,'inverseSlope',NaN);
    if speed==0
        % The norm and forward speed do not have a classical derivative here.
        fallback(3:9)=true;
    else
        ds(3:4)=inertial'/speed;
        if useModel
            model=design.egoModel;sensors=design.sensors;
            load=model.mass*model.gravity/(model.lf+model.lr)*[model.lr;model.lf];
            forceResolution=model.mass*sensors.accelerometerNoiseMaximum ...
                +model.parameterUncertainty*sum(model.frictionCoefficient(:).*load);
            denominator=max(speed*sin(beta),eps);regularization=forceResolution/denominator;
            if lateral<=-speed*sin(beta) || lateral>=speed*sin(beta)
                dy=sign(lateral)*sin(beta)*ds;info.branch="selectedClipEndpoint";
            else
                [~,gradient]=localNominalForceGradient(lateral,speed,input.yawRate, ...
                    input.heldInput,model,load,beta);
                slope=gradient(1)-regularization;info.inverseSlope=slope;
                if isfinite(slope) && abs(slope)>64*eps(max(model.corneringStiffness))
                    regularizationSpeed=0;
                    if speed*sin(beta)>eps,regularizationSpeed=-regularization/speed;end
                    dy=-(gradient(2)-regularizationSpeed*lateral)/slope*ds;
                    dy(6)=model.mass/slope;dy(7)=-gradient(3)/slope;
                    dy(8)=-gradient(4)/slope;dy(9)=-gradient(5)/slope;
                    info.branch="implicitInterior";
                    if ~isfinite(dy(9)),dy(9)=0;fallback(9)=true;end
                else
                    fallback(3:9)=true;info.branch="singularInverse";
                end
            end
        else
            dy(7)=design.yaw.courseModel.rearAxleDistance;
        end
        % Chain the forward-cone velocity map and normalized rotation vector.
        if speed^2-lateral^2>(cos(beta)*speed)^2
            db=[(speed*ds-lateral*dy)/body(1);dy];
        else
            db=[cos(beta)*ds;dy];
        end
        di=zeros(2,9);di(:,3:4)=eye(2);j=[0,-1;1,0];
        numerator=[body'*inertial;(j*body)'*inertial];
        dn=[inertial'*db+body'*di;inertial'*j*db+(j*body)'*di];
        scale=design.positionVelocity.bandwidth*design.sensors.positionNoiseMaximum;
        denominator=max(norm(body)*speed+scale^2,eps);dd=zeros(1,9);
        if norm(body)*speed+scale^2>eps
            dd=speed*(body'/norm(body))*db+norm(body)*ds;
        end
        jacobian=[db;dn/denominator-numerator*dd/denominator^2];
    end
    % Only exceptional coordinates need a secant. At |b|=1 use the physical
    % one-sided interval. sqrt(1-b^2) need not have a finite endpoint slope.
    steps=[0,0,1e-4,1e-4,1e-5,1e-5,1e-6,1e-5,1e-4];
    measurement=[input.gnssPosition;inertial;input.bodyAcceleration;input.yawRate;zeros(2,1)];
    if isfield(input,'heldInput') && numel(input.heldInput)==2,measurement(8:9)=input.heldInput;end
    for k=find(fallback)
        plus=measurement;minus=measurement;plus(k)=plus(k)+steps(k);minus(k)=minus(k)-steps(k);
        if k==9,plus(k)=min(1,plus(k));minus(k)=max(-1,minus(k));end
        jacobian(:,k)=(localPointMeasurement(localPerturbedInput(input,plus),design) ...
            -localPointMeasurement(localPerturbedInput(input,minus),design))/(plus(k)-minus(k));
    end
    info.secantColumns=fallback;
end

function input=localPerturbedInput(input,values)
    input.gnssPosition=values(1:2);input.gnssVelocity=values(3:4);
    input.bodyAcceleration=values(5:6);input.yawRate=values(7);
    if isfield(input,'heldInput') && numel(input.heldInput)==2,input.heldInput=values(8:9);end
    if isfield(input,'courseGeometry'),input=rmfield(input,'courseGeometry');end
end

function [force,gradient]=localNominalForceGradient(v,speed,rate,input,model,load,beta)
    % Partial derivatives at fixed v: columns [v,speed,rate,steering,braking].
    steering=input(1);braking=input(2);eta=sqrt(max(0,1-braking^2));
    vx=sqrt(max(speed^2-v^2,(speed*cos(beta))^2));
    dx=[-v/vx,speed/vx,0,0,0];
    if speed^2-v^2<=(speed*cos(beta))^2,dx=[0,cos(beta),0,0,0];end
    offsets=[model.lf;-model.lr];y=v+offsets*rate;
    dy=[ones(2,1),zeros(2,1),offsets,zeros(2,2)];
    slip=atan2(y,vx)-[steering;0];
    da=(vx*dy-y*dx)./(vx^2+y.^2);da(1,4)=da(1,4)-1;
    capacity=model.frictionCoefficient(:).*load;
    [fy,fa,fk]=localFiala(slip,model.corneringStiffness(:),capacity*eta);
    df=fa.*da;
    if eta>0,df(:,5)=df(:,5)-fk.*capacity*braking/eta;
    else,df(:,5)=NaN;end
    force=braking*capacity(1)*sin(steering)+fy(1)*cos(steering)+fy(2);
    gradient=cos(steering)*df(1,:)+df(2,:);
    gradient(4)=gradient(4)+braking*capacity(1)*cos(steering)-fy(1)*sin(steering);
    gradient(5)=gradient(5)+capacity(1)*sin(steering);
end

function course=localPointCourse(input,design)
    % The numerical observer map needs no inverse-interval or angle bounds.
    % Use exactly the full course map's point inversion and innovation. This
    % result deliberately omits certificate fields: it is not an enclosure.
    model=design.yaw.courseModel;
    useModel=isfield(design,'egoModel') && ~isempty(design.egoModel) ...
        && isfield(input,'heldInput') && numel(input.heldInput)==2 && all(isfinite(input.heldInput));
    if useModel
        lateral=localModelLateralVelocity(input.gnssVelocity,input.yawRate,input.bodyAcceleration, ...
            input.heldInput,design.egoModel,design.sensors,model.sideslipDomainMaximum,true);
    else
        lateral=struct('center',model.rearAxleDistance*input.yawRate,'source',"kinematic");
    end
    course=struct('lateralMeasurement',lateral,'correspondence',struct());
    course=localPointInnovation(course,input,design);
end

function course = localCourse(observerInput, design)
    model = design.yaw.courseModel;
    sensors = design.sensors;
    if isfield(observerInput,"velocityErrorMaximum")
        sensors.velocityNoiseMaximum=observerInput.velocityErrorMaximum;
        design.sensors=sensors;
    end
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
        course=localPointInnovation(course,observerInput,design);
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
    % A speed ball containing zero is uninformative for orientation, not an
    % inconsistent physical measurement.
    course.lateralMeasurement = lateral;
    course=localPointInnovation(course,observerInput,design);
end

%% Certified kinematic course correspondence

function course=localPointInnovation(course,input,design)
    lateral=course.lateralMeasurement.center;
    if ~isfinite(lateral),lateral=0;end
    body=nrmmObserverVectorField("velocityMeasurement",input.gnssVelocity,input.yawRate,design,lateral);
    inertial=input.gnssVelocity;
    scale=design.positionVelocity.bandwidth*design.sensors.positionNoiseMaximum;
    denominator=max(norm(body)*norm(inertial)+scale^2,eps);
    course.correspondence.rotationVector=[body.'*inertial; ...
        body(1)*inertial(2)-body(2)*inertial(1)]/denominator;
end

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
        "rotationVector",[rotationCosine;rotationSine] ...
            /max(inertialMagnitude*bodyMagnitude+inertialErrorMaximum^2+bodyErrorMaximum^2,eps), ...
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

function lateral = localModelLateralVelocity(gnssVelocity, yawRate, bodyAcceleration, heldInput, model, sensors, sideslipDomainMaximum,pointOnly)
    if nargin<8,pointOnly=false;end
    speed = norm(gnssVelocity);
    speedLow = speed-sensors.velocityNoiseMaximum;
    speedHigh = speed+sensors.velocityNoiseMaximum;
    sineDomain = sin(sideslipDomainMaximum);
    lateral = struct("center",NaN,"lower",-Inf,"upper",Inf,"radius",Inf, ...
        "consistent",true,"informative",false,"source","force-balance", ...
        "measuredSpeed",speed,"trueSpeedLowerBound",speedLow);
    steering = heldInput(1);braking = heldInput(2);
    accelerationLow = model.mass*(bodyAcceleration(2)-sensors.accelerometerNoiseMaximum);
    accelerationHigh = model.mass*(bodyAcceleration(2)+sensors.accelerometerNoiseMaximum);
    rates = yawRate+[-1,1]*sensors.gyroscopeNoiseMaximum;
    spread = model.parameterUncertainty*[-1,1]+1;
    wheelbase = model.lf+model.lr;
    load = model.mass*model.gravity/wheelbase*[model.lr;model.lf];
    eta = sqrt(max(0,1-braking^2));
    % The point inversion depends on measured values, not certificate width.
    nominal=localNominal(-speed*sineDomain,speed*sineDomain);
    if ~isfinite(nominal),nominal=0;end
    if pointOnly
        lateral=struct('center',nominal,'source',"force-balance");return;
    end
    if ~(speedLow > 0)
        lateral.lower=-speedHigh*sineDomain;lateral.upper=speedHigh*sineDomain;
        lateral.center=nominal;lateral.radius=max(abs([lateral.lower,lateral.upper]-nominal));
        return;
    end
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
        lateral.center = nominal;
        lateral.radius = max(abs([nominal-lower,upper-nominal]));
        lateral.informative = isfinite(lateral.radius);
    else
        lateral.center = nominal;lateral.radius = Inf;
    end

    function value = localNominal(leftEnd,rightEnd)
        % A saturated force map has no unique inverse. A monotone proximal
        % term gives the point reconstruction a finite slope, calibrated by
        % the declared IMU noise across the slip domain. This does not change
        % the vehicle force or the enclosing inverse interval above. Unlike
        % its interval midpoint, this point does not depend on the evolving
        % velocity certificate, so runtime and future observer updates agree.
        target = model.mass*bodyAcceleration(2);
        forceResolution=model.mass*sensors.accelerometerNoiseMaximum ...
            +model.parameterUncertainty*sum(model.frictionCoefficient(:).*load);
        regularization=forceResolution/max(speed*sineDomain,eps);
        g = @(v) localNominalForce(v)-regularization*v;
        if g(leftEnd) < target
            value = leftEnd;return;
        elseif g(rightEnd) > target
            value = rightEnd;return;
        end
        % Resolve the POINT inverse below the tangent perturbation scale.
        % The enclosing interval inverse keeps its coarser outward brackets.
        value = localBisect(leftEnd,rightEnd,@(v) g(v) < target,1e-10);
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

function [force,angleSlope,capacitySlope] = localFiala(slip,stiffness,capacity)
% Modified Fiala lateral force, identical to modifiedFialaTire.evaluate.
    tangent = tan(slip);
    force = -capacity.*sign(slip);
    adhesion = capacity > 0 & abs(tangent) < 3*capacity./stiffness;
    q = stiffness(adhesion).*abs(tangent(adhesion))./(3*capacity(adhesion));
    force(adhesion) = -stiffness(adhesion).*tangent(adhesion).*(1-q+q.^2/3);
    if nargout>1
        angleSlope=zeros(size(slip));capacitySlope=-sign(slip);
        angleSlope(adhesion)=-stiffness(adhesion).*(1+tangent(adhesion).^2).*(1-q).^2;
        capacitySlope(adhesion)=-stiffness(adhesion).*tangent(adhesion).*q ...
            .*(1-2*q/3)./capacity(adhesion);
    end
end

function value = localBisect(left,right,predicate,tolerance)
% First point of the bracket where the monotone predicate becomes true.
% The default 1e-5 m/s bracket is retained for interval inversion.
    if nargin<4,tolerance=1e-5;end
    while right-left > tolerance
        middle = 0.5*(left+right);
        if predicate(middle),right = middle;else,left = middle;end
    end
    value = right;
end
