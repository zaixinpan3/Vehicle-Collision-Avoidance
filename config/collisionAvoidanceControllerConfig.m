function cfg = collisionAvoidanceControllerConfig(userCfg)
%collisionAvoidanceControllerConfig Raw-sensor predictive controller and shared vehicle settings.
% Unknown fields are rejected so retired algorithm settings cannot silently
% change the interpretation of a controller run.
    if nargin<1,userCfg=[];end
    cfg=localDefaults();
    if ~isempty(userCfg)
        if ~isstruct(userCfg) || ~isscalar(userCfg)
            localInvalid('The override must be a scalar structure.');
        end
        cfg=localMerge(cfg,userCfg);
    end
    try
        localValidate(cfg);
    catch exception
        error('collisionAvoidanceController:invalidConfiguration','%s',exception.message);
    end
    cfg.actuation.brakingRatioMinimum=double(cfg.actuation.brakingRatioMinimum);
    cfg.actuation.brakingRatioMaximum=double(cfg.actuation.brakingRatioMaximum);
end

function cfg=localDefaults()
    cfg.referenceSpeed=15;
    % Raw-sensor compact nonlinear control. Noise bounds are Euclidean radii for
    % GNSS position, inertial body acceleration, gyro, and radar position.
    % Finite hypotheses and future noise cases are a deployment approximation.
    % Nominal simulation road-wheel limit: 2021 Pacifica Hybrid OEM data
    % give 3.13 lock-to-lock turns / 16.2 ratio, about 34.8 deg each side.
    % This is an equivalent-angle estimate, not an actuator calibration.
    % The online IPOPT iteration limit is nlpMaximumIterations.
    cfg.outputFeedback=struct('method',"ipopt",'nlpMaximumIterations',50, ...
        'nlpTolerance',1e-7,'nlpJit',true,'nlpCompilerOptimization',2,'nlpCompiler',"clang", ...
        'sensorPeriod',.0125, ...
        'noiseBounds',[.04;.03;.0015;.04], ...
        'horizonSeconds',[],'previewDecelerationMetersPerSecondSquared',4, ...
        'previewSettlingSeconds',.5,'horizonResolutionSeconds',1, ...
        'clearanceMeters',.1,'steeringMaximum',deg2rad(35),'steeringRateMaximum',Inf, ...
        'inputWeight',.1,'trackingWeights',[.02;.2;2;.1;.1], ...
        'maximumHypotheses',64, ...
        'maximumSolveSeconds',1,'initializationSolveSeconds',2, ...
        'controlNodes',24,'nearControlNodes',4,'minimumPreviewSeconds',3, ...
        'validationSubsteps',8); % Fixed integration/domain mesh; collision nodes are control boundaries.
    % Compact maximumSolveSeconds covers the whole warmed controller call.
    % Only initialization has a separate budget. Both are soft search deadlines.
    % The transverse penalty permits a passing detour while the fixed speed
    % reference continues to penalize waiting; the cruise path stays fixed.
    % A finite steering-rate bound requires an explicitly supplied actuator contract.
    % The encounter preview sets the horizon. Every collision stage has slack.
    % Road edges are deferred. Lateral excursion is an experiment stop rule.
    cfg.controller=struct('sampleTime',.05,'maximumHorizonSteps',Inf, ...
        'enforceRoadBoundaries',false,'jointAffinePrediction',true,'jointNonlinearPrediction',false, ...
        'reducedJointInformation',true, ...
        'jointPredictionSamplesPerHold',2,'jointPredictionInverseTolerance',.005, ...
        'jointPredictionRefreshSteps',2,'compactObserverPrediction',true, ...
        'jointGeneratorLimit',4096,'jointMomentPrediction',true);
    cfg.nonlinear=struct('integrationStep',.05);
    % One field rollout combines route attraction with predictive obstacle
    % gradients. Time-aligned exposure scales continuous target-transport and
    % separation gradients, without a passing-side input or sign selection.
    % There is no candidate bank or nominal/avoidance behavior switch.
    % Cruise feedback never overwrites longitudinal obstacle correction.
    % The parallel-wall acceleration guide anticipates transverse momentum;
    % it is not a hard terminal constraint or an execution certificate.
    cfg.planner=struct('radialRate',1,'tangentialRate',2,'wallRate',2, ...
        'lateralAccelerationFraction',.9,'guideHalfWidthMeters',9);
    % encounterRangeMeters is the perception radius: the estimator receives
    % the target only inside it, and outside it there is no collision risk.
    % The first certified exit ends this prediction's encounter. A fresh
    % observation after an actual exit starts a new collision problem.
    % Collision rows require nonoverlap after uncertainty tightening. There
    % is no extra fixed clearance in the optimizer or geometric guide.
    cfg.collision=struct('encounterRangeMeters',50);
    cfg.vehicle=struct('m',1650,'Iz',1700,'lf',1.4,'lr',1.65, ...
        'wheelbase',3.05,'length',4.8,'width',1.9,'rectangleOffset',[0;0],'gravity',9.81);
    cfg.tire=struct('corneringStiffness',[96000;96000],'frictionCoefficient',[.85;.85]);
    cfg.actuation=struct('brakingRatioMinimum',-1,'brakingRatioMaximum',1);
    cfg.roadLoad=struct('airDensity',1.225,'dragCoefficient',.30,'frontalArea',2.2, ...
        'rollingCoefficient',.01,'rollingSpeedCoefficient',0,'rollingQuarticCoefficient',0, ...
        'rollingTransitionSpeed',.5);
    % sideslipMaximum b is the sideslip cone |v_y| <= tan(b) v_x shared with the
    % estimator (its force-balance and course inversions assume it); the
    % controller enforces it on every predicted node above brakeHoldSpeed (at
    % rest the sideslip is undefined).
    % Its value is a chosen operating domain, not a universal observer limit.
    % Enlarging it requires revalidating the inverse and its error bounds.
    % The declared model (nonlinearBicycleModel.derivative) is the Fiala
    % bicycle above kinematicSpeedRange [m/s], a kinematic bicycle below it
    % and a smooth blend between, so that the vehicle can slow to rest:
    % scheduleSpeedFloor is the Fiala tire's own floor, below the range. In
    % the kinematic bicycle the yaw rate and lateral velocity relax to their
    % kinematic values with kinematicLagSeconds, and braking fades below
    % brakeHoldSpeed [m/s], so that it ends at rest; a braked vehicle below a
    % fifth of brakeHoldSpeed at the end of a hold is at rest exactly.
    cfg.model=struct('speedMinimum',0,'speedMaximum',18,'scheduleSpeedFloor',1, ...
        'kinematicSpeedRange',[2.5,4],'kinematicLagSeconds',.1,'brakeHoldSpeed',.25, ...
        'brakingRatioRateMaximum',Inf,'lateralVelocityMaximum',12,'yawRateMaximum',5, ...
        'sideslipMaximum',.25);
    % One continuous-time quadratic CLF V = e' P e with the soft condition
    %   Vdot <= -2 V / convergenceTimeConstantSeconds + slack.
    % The offline design enforces the continuous trim Lyapunov inequality.
    % A separate sampled certificate supports the cruise initializer below.
    % P and the gain K of the cruise controller u = u* + K e are synthesized offline
    % (scripts/synthesizeClfMatrices.m) and read from config/clfMatrices.json:
    % inside the certification box of transverse errors (|lateral| <=
    % certificationLateralMeters, |heading| <= certificationHeadingRadians,
    % |speed| <= certificationSpeedMetersPerSecond, |lateral velocity| <=
    % certificationLateralVelocityMetersPerSecond, |yaw rate| <=
    % certificationYawRateRadiansPerSecond) and with inputs within
    % certificationSteeringRadians and certificationBrakingRatio of the trim,
    % the largest sublevel set V <= 1 on which u = u* + K e contracts by rho
    % against every ray-averaged Jacobian of the sampled nonlinear model over
    % the set is certified (CONTROLLER_THEORY.md, Section 4). Nominal feedback supplies
    % the search reference; the terminal set itself does not use the CLF. The box only bounds the set;
    % its size is set by the certificate.
    cfg.clf=struct('lateralPositionErrorScale',.5,'headingErrorScale',.1, ...
        'speedErrorScale',.25,'lateralVelocityErrorScale',.5,'yawRateErrorScale',.2, ...
        'convergenceTimeConstantSeconds',4, ...
        'certificationLateralMeters',2,'certificationHeadingRadians',.2, ...
        'certificationSpeedMetersPerSecond',1,'certificationLateralVelocityMetersPerSecond',.3, ...
        'certificationYawRateRadiansPerSecond',.2, ...
        'certificationSteeringRadians',.075,'certificationBrakingRatio',.125);
    % The remaining nominalClf parameters guide initialization only.
    cfg.nominalClf=struct('lookaheadSeconds',1.5,'minimumLookaheadMeters',8,'courseGain',1.5, ...
        'yawRateGain',10,'lateralAccelerationFraction',.75,'frontForceFraction',.9, ...
        'speedGain',.5,'brakingRatioLimit',.35);
    % Tolerance used for actuator-bound interior points in shared model routines.
    cfg.solver=struct('constraintTolerance',1e-5);
    cfg.target=struct('defaultLength',4.8,'defaultWidth',1.9,'rearAxleDistance',1.6,'observerDomain',[]);
end

function base=localMerge(base,overrides)
    names=fieldnames(overrides);
    for index=1:numel(names)
        name=names{index};value=overrides.(name);
        if ~isfield(base,name),localInvalid('Unknown controller configuration field: %s.',name);end
        if isstruct(base.(name))
            if ~isstruct(value) || ~isscalar(value),localInvalid('%s must be a scalar structure.',name);end
            base.(name)=localMerge(base.(name),value);
        else
            base.(name)=value;
        end
    end
end

function localValidate(cfg)
    of=cfg.outputFeedback;
    assert(isscalar(string(of.method)) && string(of.method)=="ipopt", ...
        'outputFeedback.method must be ipopt.');
    validateattributes(of.controlNodes,{'double'},{'scalar','integer','finite','>=',6});
    validateattributes(of.nearControlNodes,{'double'},{'scalar','integer','finite','positive','<',of.controlNodes});
    validateattributes(of.nlpMaximumIterations,{'double'},{'scalar','integer','finite','positive'});
    validateattributes(of.nlpTolerance,{'double'},{'scalar','finite','positive'});
    validateattributes(of.nlpJit,{'logical'},{'scalar'});
    validateattributes(of.nlpCompilerOptimization,{'double'},{'scalar','integer','>=',0,'<=',3});
    assert(isscalar(string(of.nlpCompiler)) && any(string(of.nlpCompiler)==["gcc","clang","cc"]), ...
        'nlpCompiler must name gcc, clang or cc.');
    if ~isempty(of.horizonSeconds)
        validateattributes(of.horizonSeconds,{'double'},{'scalar','real','finite','positive'});
    end
    for name=["previewDecelerationMetersPerSecondSquared","horizonResolutionSeconds"]
        validateattributes(of.(name),{'double'},{'scalar','real','finite','positive'});
    end
    validateattributes(of.previewSettlingSeconds,{'double'},{'scalar','real','finite','nonnegative'});
    for name=["sensorPeriod","steeringMaximum"]
        validateattributes(of.(name),{'double'},{'scalar','real','finite','positive'});
    end
    validateattributes(of.steeringRateMaximum,{'double'},{'scalar','real','nonnan','positive'});
    validateattributes(of.maximumSolveSeconds,{'double'},{'scalar','real','positive'});
    validateattributes(of.initializationSolveSeconds,{'double'},{'scalar','real','finite','positive'});
    assert(of.steeringMaximum<pi/2,'steeringMaximum must be below pi/2.');
    validateattributes(of.maximumHypotheses,{'double'},{'scalar','integer','finite','positive'});
    validateattributes(of.minimumPreviewSeconds,{'double'},{'scalar','real','finite','positive'});
    validateattributes(of.validationSubsteps,{'double'},{'scalar','integer','finite','>=',4});
    validateattributes(of.clearanceMeters,{'double'},{'scalar','real','finite','nonnegative'});
    if of.clearanceMeters==0
        localInvalid('Compact PCBF planning requires a positive clearanceMeters.');
    end
    validateattributes(of.inputWeight,{'double'},{'scalar','real','finite','nonnegative'});
    validateattributes(of.noiseBounds,{'double'},{'size',[4,1],'real','finite','nonnegative'});
    validateattributes(of.trackingWeights,{'double'},{'size',[5,1],'real','finite','nonnegative'});
    validateattributes(cfg.referenceSpeed,{'double'},{'scalar','real','finite','positive'});
    validateattributes(cfg.controller.sampleTime,{'double'},{'scalar','real','finite','positive'});
    validateattributes(cfg.controller.enforceRoadBoundaries,{'logical'},{'scalar'});
    validateattributes(cfg.controller.maximumHorizonSteps,{'double'},{'scalar','real','positive'});
    if isfinite(cfg.controller.maximumHorizonSteps)
        validateattributes(cfg.controller.maximumHorizonSteps,{'double'},{'integer'});
    end
    validateattributes(cfg.nonlinear.integrationStep,{'double'},{'scalar','real','finite','positive'});
    validateattributes(cfg.controller.jointAffinePrediction,{'logical'},{'scalar'});
    validateattributes(cfg.controller.reducedJointInformation,{'logical'},{'scalar'});
    validateattributes(cfg.controller.jointPredictionSamplesPerHold,{'double'},{'scalar','integer','nonnegative','finite'});
    assert(mod(cfg.controller.jointPredictionSamplesPerHold,2)==0, ...
        'collisionAvoidanceControllerConfig:invalidPredictionSamples', ...
        'Use zero for the sensor clock, or an even count resolving the hold midpoint.');
    validateattributes(cfg.controller.jointPredictionInverseTolerance,{'double'},{'scalar','positive','finite'});
    validateattributes(cfg.controller.jointPredictionRefreshSteps,{'double'},{'scalar','integer','positive','finite'});
    validateattributes(cfg.controller.compactObserverPrediction,{'logical'},{'scalar'});
    validateattributes(cfg.controller.jointMomentPrediction,{'logical'},{'scalar'});
    validateattributes(cfg.controller.jointGeneratorLimit,{'double'},{'scalar','integer','>=',70,'finite'});
    validateattributes(cfg.controller.jointNonlinearPrediction,{'logical'},{'scalar'});
    for name=["radialRate","tangentialRate","wallRate"]
        validateattributes(cfg.planner.(name),{'double'},{'scalar','real','finite','positive'});
    end
    validateattributes(cfg.planner.lateralAccelerationFraction,{'double'},{'scalar','real','>',0,'<',1});
    validateattributes(cfg.planner.guideHalfWidthMeters,{'double'},{'scalar','real','positive'});
    validateattributes(cfg.collision.encounterRangeMeters,{'double'},{'scalar','real','finite','positive'});
    for name=["m","Iz","lf","lr","wheelbase","length","width","gravity"]
        validateattributes(cfg.vehicle.(name),{'double'},{'scalar','real','finite','positive'});
    end
    validateattributes(cfg.vehicle.rectangleOffset,{'double'},{'size',[2,1],'real','finite'});
    for name=["corneringStiffness","frictionCoefficient"]
        validateattributes(cfg.tire.(name),{'double'},{'size',[2,1],'real','finite','positive'});
    end
    for name=["brakingRatioMinimum","brakingRatioMaximum"]
        validateattributes(cfg.actuation.(name),{'numeric'},{'scalar','real','finite'});
    end
    lo=cfg.actuation.brakingRatioMinimum;hi=cfg.actuation.brakingRatioMaximum;
    if lo < -1 || hi > 1 || lo>=hi || lo>0 || hi<0
        localInvalid('Braking-ratio limits must satisfy -1 <= minimum <= 0 <= maximum <= 1 with positive width.');
    end
    for name=string(fieldnames(cfg.roadLoad)).'
        validateattributes(cfg.roadLoad.(name),{'double'},{'scalar','real','finite','nonnegative'});
    end
    if cfg.roadLoad.rollingTransitionSpeed<=0,localInvalid('Rolling transition speed must be positive.');end
    validateattributes(cfg.model.speedMinimum,{'double'},{'scalar','real','finite','nonnegative'});
    for name=["speedMaximum","scheduleSpeedFloor", ...
            "lateralVelocityMaximum","yawRateMaximum"]
        validateattributes(cfg.model.(name),{'double'},{'scalar','real','finite','positive'});
    end
    validateattributes(cfg.model.sideslipMaximum,{'double'},{'scalar','real','positive','<',pi/2});
    if cfg.model.speedMaximum<=max(cfg.model.speedMinimum,cfg.model.scheduleSpeedFloor)
        localInvalid('The positive-speed model domain must have positive width.');
    end
    validateattributes(cfg.model.brakingRatioRateMaximum,{'double'},{'scalar','real','positive'});
    validateattributes(cfg.model.kinematicSpeedRange,{'double'},{'size',[1,2],'real','finite','increasing'});
    if cfg.model.kinematicSpeedRange(1)<=cfg.model.scheduleSpeedFloor || cfg.model.kinematicSpeedRange(2)>=cfg.model.speedMaximum
        localInvalid('The kinematic speed range must lie above the Fiala floor and below the maximum speed.');
    end
    for name=["kinematicLagSeconds","brakeHoldSpeed"]
        validateattributes(cfg.model.(name),{'double'},{'scalar','real','finite','positive'});
    end
    if cfg.model.brakeHoldSpeed>=cfg.model.kinematicSpeedRange(1)
        localInvalid('Braking must fade inside the kinematic regime.');
    end
    for name=string(fieldnames(cfg.clf)).'
        validateattributes(cfg.clf.(name),{'double'},{'scalar','real','finite','positive'});
    end
    for name=string(fieldnames(cfg.nominalClf)).'
        validateattributes(cfg.nominalClf.(name),{'double'},{'scalar','real','finite','positive'});
    end
    for name=["lateralAccelerationFraction","frontForceFraction","brakingRatioLimit"]
        if cfg.nominalClf.(name)>=1,localInvalid('nominalClf.%s must lie in (0,1).',name);end
    end
    validateattributes(cfg.solver.constraintTolerance,{'double'},{'scalar','real','finite','positive'});
    domain=cfg.target.observerDomain;
    if ~isempty(domain)
        allowed=["speedMinimum","speedMaximum","scalarAccelerationMaximum","sideslipMaximum", ...
            "yawRateMaximum","accelerationNormBound"];
        if ~isstruct(domain) || ~isscalar(domain) || ~all(ismember(fieldnames(domain),allowed))
            localInvalid('target.observerDomain accepts only NRMM physical operating bounds.');
        end
        if isfield(domain,'sideslipMaximum')
            validateattributes(domain.sideslipMaximum,{'double'},{'scalar','positive','<=',pi/2});
        end
        for name=string(fieldnames(domain)).'
            validateattributes(domain.(name),{'double'},{'scalar','real','finite','nonnegative'});
        end
    end
    for name=["defaultLength","defaultWidth","rearAxleDistance"]
        validateattributes(cfg.target.(name),{'double'},{'scalar','real','finite','positive'});
    end
end

function localInvalid(varargin)
    error('collisionAvoidanceController:invalidConfiguration',varargin{:});
end
