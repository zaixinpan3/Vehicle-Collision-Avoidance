function cfg = collisionAvoidanceControllerConfig(userCfg)
%collisionAvoidanceControllerConfig Switched PCBF/CLF parameters in SI units.
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
    % Encounter preview sizes the first reference; its first accepted length
    % stays fixed throughout that encounter.
    % Road edges are deferred. Lateral excursion is an experiment stop rule.
    cfg.controller=struct('sampleTime',.05,'horizonSteps',16,'maximumHorizonSteps',Inf, ...
        'enforceRoadBoundaries',false);
    cfg.nonlinear=struct('integrationStep',.05);
    % Plan once on target acquisition; later frames shift the state/input iterate.
    % Initial preview starts from horizonSteps and the current stopping time.
    % Later frames shift M-1 state/input stages and append one hold to retain M.
    % Failure is reported without horizon extension, replanning or another
    % local model.
    % Shared quadratic performance: nominal transverse/speed tracking,
    % actual input increments, and optional reference proximity (off by default).
    % PCBF minimizes safety slack first, then performance at that optimum.
    % Cruise minimizes the integrated continuous-CLF slack plus performance.
    % All three weights are soft costs; zero disables that component.
    % Trust budgets are one-hold numerical approximation tolerances in
    % [m;m;rad;m/s;m/s;rad/s], separate from observer uncertainty.
    cfg.rti=struct('settlingSeconds',.5,'referenceDeviationWeight',0, ...
        'nominalTrackingWeight',1,'inputVariationWeight',20, ...
        'trustRegion',struct('enabled',true,'errorBudget',[.04;.04;.01;.4;.4;.2]));
    % One field rollout combines route attraction with motion-dependent target
    % circulation, including longitudinal avoidance. No nominal-path forecast
    % selects a behavior. Cruise feedback never overwrites obstacle correction.
    % The parallel-wall acceleration guide anticipates transverse momentum;
    % it is not a hard RTI/terminal constraint or an execution certificate.
    cfg.planner=struct('radialRate',1,'tangentialRate',2,'wallRate',2, ...
        'lateralAccelerationFraction',.9,'guideHalfWidthMeters',9);
    % encounterRangeMeters is the perception radius: the estimator receives
    % the target only inside it, and outside it there is no collision risk.
    % The first certified exit ends this prediction's encounter. A fresh
    % observation after an actual exit starts a new collision problem.
    % Collision rows require nonoverlap after uncertainty tightening. There
    % is no extra fixed clearance in the optimizer or geometric guide.
    cfg.collision=struct('encounterRangeMeters',50);
    % Terminal set (controller/terminalSafeSet.m, CONTROLLER_THEORY.md, Section 5): it
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
        'sideslipMaximumRadians',.1);
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
    % One continuous-time quadratic CLF V = e' P e with the soft condition
    %   Vdot <= -2 V / convergenceTimeConstantSeconds + slack.
    % Integrate this condition over a held-input interval h, then RTI
    % linearizes V at the first predicted state. The resulting affine row is
    % V_aff(x+) <= exp(-alpha*h)*V(x) + (1-exp(-alpha*h))/alpha * slack.
    % The continuous design of P is unchanged. Zero affine slack alone is
    % not an exact nonlinear sampled or continuous stability certificate.
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
    % PCBF priority and reference refinement share the remaining call budget.
    % An in-flight factorization can overrun this soft wall-clock limit.
    % constraintTolerance and optimalityTolerance are the conic solver's
    % feasibility and gap tolerances; feasibilityTolerance is also used for
    % diagnostic interpretation and limits numerical input-bound projection.
    % reducedTolerance is the precision
    % for an AlmostSolved result. Budget includes controller preprocessing;
    % the reserve leaves time for output assembly. Native
    % solves also count their construction time against the remaining budget.
    cfg.solver=struct('maxIterations',40,'timeLimitSeconds',.1,'completionReserveSeconds',.002, ...
        'feasibilityTolerance',1e-4,'constraintTolerance',1e-5, ...
        'optimalityTolerance',1e-5,'lexicographicTieTolerance',1e-3, ...
        'reducedTolerance',1e-3);
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
    validateattributes(cfg.nonlinear.integrationStep,{'double'},{'scalar','real','finite','positive'});
    validateattributes(cfg.rti.settlingSeconds,{'double'},{'scalar','real','finite','positive'});
    validateattributes(cfg.rti.referenceDeviationWeight,{'double'},{'scalar','real','finite','nonnegative'});
    validateattributes(cfg.rti.nominalTrackingWeight,{'double'},{'scalar','real','finite','nonnegative'});
    validateattributes(cfg.rti.inputVariationWeight,{'double'},{'scalar','real','finite','nonnegative'});
    validateattributes(cfg.rti.trustRegion.enabled,{'logical'},{'scalar'});
    validateattributes(cfg.rti.trustRegion.errorBudget,{'double'},{'size',[6,1],'real','finite','positive'});
    for name=["radialRate","tangentialRate","wallRate"]
        validateattributes(cfg.planner.(name),{'double'},{'scalar','real','finite','positive'});
    end
    validateattributes(cfg.planner.lateralAccelerationFraction,{'double'},{'scalar','real','>',0,'<',1});
    validateattributes(cfg.planner.guideHalfWidthMeters,{'double'},{'scalar','real','positive'});
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
    for name=["feasibilityTolerance","constraintTolerance","optimalityTolerance","reducedTolerance"]
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
