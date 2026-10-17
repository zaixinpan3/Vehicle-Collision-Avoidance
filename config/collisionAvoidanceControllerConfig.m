function cfg = collisionAvoidanceControllerConfig(userCfg)
%collisionAvoidanceControllerConfig Two-stage PCBF/CLF parameters in SI units.
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
    % horizonSteps is the soft-collision prefix and the shortest horizon.
    % References extend to a terminal endpoint without a default duration cap.
    % Road edges are deferred; the path remains the cruise/CLF reference.
    cfg.controller=struct('sampleTime',.05,'horizonSteps',16,'maximumHorizonSteps',Inf, ...
        'enforceRoadBoundaries',false);
    % trustRadius scales local state/input corrections about the RTI reference.
    % The input trust scale adapts to the plan innovation: the scale whose
    % second-order prediction error equals trustInnovationMeters [m], within
    % [trustMinimumScale,trustMaximumScale]. trustRetention is the per-sample
    % retention of the estimated error coefficient; the scale grows by at most
    % 1/sqrt(trustRetention) per sample and shrinks at once.
    % Only a completed prediction updates this scale for the next sample.
    cfg.nonlinear=struct('integrationStep',.05,'trustRadius',.5, ...
        'trustInnovationMeters',.02,'trustInitialScale',1, ...
        'trustMinimumScale',1/16,'trustMaximumScale',1,'trustRetention',.5);
    % One RTI model per reference. Local primal infeasibility permits one
    % fresh reference and retry, sharing the original controller deadline.
    % Encounter search starts from horizonSteps and the current stopping time;
    % the terminal extension has no separate duration cap.
    cfg.rti=struct('settlingSeconds',.5);
    % Second-order circulation supplies initialization controls only. Each
    % candidate is rolled through the declared Fiala model. Contour offset
    % is a guidance preference, never an extra collision constraint.
    cfg.planner=struct('radialRate',1,'tangentialRate',2, ...
        'contourOffsetFraction',1,'circulationSpeedFractions',[1,.65], ...
        'releaseAttempts',2,'lateralAccelerationFraction',.9);
    % encounterRangeMeters is the perception radius: the estimator receives
    % the target only inside it, and outside it there is no collision risk.
    % The first certified exit ends this prediction's encounter. A fresh
    % observation after an actual exit starts a new collision problem.
    % Collision rows require nonoverlap after uncertainty tightening. There
    % is no extra fixed clearance in the optimizer or geometric guide.
    cfg.collision=struct('encounterRangeMeters',50);
    % Terminal set (controller/terminalSafeSet.m, TERMINAL_SAFE_SET.md): it
    % holds only mild endpoints, inside the sideslip cone and with the held
    % arc's lateral acceleration |r V| at most
    % lateralAccelerationMaximumMetersPerSecondSquared [m/s^2], where the ego
    % is taken to be a kinematic vehicle. Past the plan's endpoint the target
    % keeps its forecast and the ego brakes at
    % brakingDecelerationMetersPerSecondSquared [m/s^2] with the steering
    % (its sideslip and course curvature) held until a forward-only stop,
    % possibly inside a hold; the endpoint is terminal when it respects any
    % enabled road edges and clears the target until it leaves encounterRangeMeters (after
    % the stop, over the target's remaining track). Braking and the lateral
    % acceleration together must lie inside the tire friction
    % (frictionCoefficient times gravity). No encounter duration is preset.
    % Terminal sideslip is stricter than the estimator's operating cone:
    % a settled endpoint, rather than a large lateral drift with small yaw.
    cfg.terminal=struct('brakingDecelerationMetersPerSecondSquared',4, ...
        'lateralAccelerationMaximumMetersPerSecondSquared',2, ...
        'sideslipMaximumRadians',.05);
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
    % One quadratic CLF V = e' P e. Each sample requires
    %   V(next) <= rho V(now),  rho = exp(-2 sampleTime / convergenceTimeConstantSeconds),
    % so the error sqrt(V) shrinks by at least 1/e every time constant.
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
    % the set is certified (NOMINAL_CLF.md). Nominal feedback supplies
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
    % Both RTI stages share the remaining controller-call budget.
    % An in-flight factorization can overrun this soft wall-clock limit.
    % constraintTolerance and optimalityTolerance are the conic solver's
    % feasibility and gap tolerances; reducedTolerance is the precision
    % for an AlmostSolved result. Budget includes controller preprocessing;
    % the reserve leaves time for prediction and output assembly. Native
    % solves also count their construction time against the remaining budget.
    cfg.solver=struct('maxIterations',40,'timeLimitSeconds',.1,'completionReserveSeconds',.002, ...
        'feasibilityTolerance',1e-4,'constraintTolerance',1e-5, ...
        'optimalityTolerance',1e-5,'lexicographicTieTolerance',1e-3, ...
        'clfTieTolerance',1e-3,'reducedTolerance',1e-3);
    cfg.target=struct('defaultLength',4.8,'defaultWidth',1.9,'rearAxleDistance',1.6);
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
    validateattributes(cfg.referenceSpeed,{'double'},{'scalar','real','finite','positive'});
    validateattributes(cfg.controller.sampleTime,{'double'},{'scalar','real','finite','positive'});
    validateattributes(cfg.controller.horizonSteps,{'double'},{'scalar','real','finite','integer','positive'});
    validateattributes(cfg.controller.enforceRoadBoundaries,{'logical'},{'scalar'});
    validateattributes(cfg.controller.maximumHorizonSteps,{'double'},{'scalar','real','positive'});
    if isfinite(cfg.controller.maximumHorizonSteps)
        validateattributes(cfg.controller.maximumHorizonSteps,{'double'},{'integer'});
    end
    if cfg.controller.horizonSteps>cfg.controller.maximumHorizonSteps
        localInvalid('horizonSteps cannot exceed maximumHorizonSteps.');
    end
    for name=["integrationStep","trustRadius", ...
            "trustInnovationMeters","trustInitialScale","trustMinimumScale","trustMaximumScale"]
        validateattributes(cfg.nonlinear.(name),{'double'},{'scalar','real','finite','positive'});
    end
    validateattributes(cfg.nonlinear.trustRetention,{'double'},{'scalar','real','>',0,'<',1});
    if cfg.nonlinear.trustMinimumScale>cfg.nonlinear.trustInitialScale ...
            || cfg.nonlinear.trustInitialScale>cfg.nonlinear.trustMaximumScale
        localInvalid('Trust scales must satisfy minimum <= initial <= maximum.');
    end
    validateattributes(cfg.rti.settlingSeconds,{'double'},{'scalar','real','finite','positive'});
    for name=["radialRate","tangentialRate","contourOffsetFraction"]
        validateattributes(cfg.planner.(name),{'double'},{'scalar','real','finite','positive'});
    end
    validateattributes(cfg.planner.circulationSpeedFractions,{'double'}, ...
        {'row','nonempty','real','finite','positive','<=',1});
    validateattributes(cfg.planner.releaseAttempts,{'double'},{'scalar','integer','nonnegative','finite'});
    validateattributes(cfg.planner.lateralAccelerationFraction,{'double'},{'scalar','real','>',0,'<',1});
    validateattributes(cfg.collision.encounterRangeMeters,{'double'},{'scalar','real','finite','positive'});
    validateattributes(cfg.terminal.brakingDecelerationMetersPerSecondSquared,{'double'},{'scalar','real','finite','positive'});
    validateattributes(cfg.terminal.lateralAccelerationMaximumMetersPerSecondSquared,{'double'},{'scalar','real','finite','positive'});
    validateattributes(cfg.terminal.sideslipMaximumRadians,{'double'},{'scalar','real','finite','positive','<',pi/2});
    for name=["m","Iz","lf","lr","wheelbase","length","width","gravity"]
        validateattributes(cfg.vehicle.(name),{'double'},{'scalar','real','finite','positive'});
    end
    validateattributes(cfg.vehicle.rectangleOffset,{'double'},{'size',[2,1],'real','finite'});
    for name=["corneringStiffness","frictionCoefficient"]
        validateattributes(cfg.tire.(name),{'double'},{'size',[2,1],'real','finite','positive'});
    end
    if hypot(cfg.terminal.brakingDecelerationMetersPerSecondSquared, ...
            cfg.terminal.lateralAccelerationMaximumMetersPerSecondSquared)>=min(cfg.tire.frictionCoefficient)*cfg.vehicle.gravity
        localInvalid('The terminal braking rate and lateral acceleration must lie inside the tire friction.');
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
    if cfg.terminal.sideslipMaximumRadians>cfg.model.sideslipMaximum
        localInvalid('The terminal sideslip bound must lie inside the estimator operating cone.');
    end
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
    validateattributes(cfg.solver.maxIterations,{'double'},{'scalar','real','finite','integer','positive'});
    validateattributes(cfg.solver.timeLimitSeconds,{'double'},{'scalar','real','finite','positive'});
    validateattributes(cfg.solver.completionReserveSeconds,{'double'},{'scalar','real','finite','nonnegative'});
    for name=["feasibilityTolerance","constraintTolerance","optimalityTolerance","clfTieTolerance","reducedTolerance"]
        validateattributes(cfg.solver.(name),{'double'},{'scalar','real','finite','positive'});
    end
    if cfg.solver.reducedTolerance<max(cfg.solver.feasibilityTolerance,cfg.solver.constraintTolerance)
        localInvalid('solver.reducedTolerance cannot be tighter than the feasibility and constraint tolerances.');
    end
    validateattributes(cfg.solver.lexicographicTieTolerance,{'double'},{'scalar','real','finite','nonnegative'});
    for name=["defaultLength","defaultWidth","rearAxleDistance"]
        validateattributes(cfg.target.(name),{'double'},{'scalar','real','finite','positive'});
    end
end

function localInvalid(varargin)
    error('collisionAvoidanceController:invalidConfiguration',varargin{:});
end
