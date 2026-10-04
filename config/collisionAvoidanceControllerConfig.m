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
    % horizonSteps carries safety slack and is the shortest horizon. The
    % horizon extends until the anchor reaches the terminal set, at most
    % maximumHorizonSteps; a plan that cannot reach it has no solution.
    cfg.controller=struct('sampleTime',.05,'horizonSteps',16,'maximumHorizonSteps',512);
    % trustRadius scales RTI state/input corrections about each fresh rollout.
    % The input trust scale adapts to the plan innovation: the scale whose
    % second-order prediction error equals trustInnovationMeters [m], within
    % [trustMinimumScale,trustMaximumScale]. trustRetention is the per-sample
    % retention of the estimated error coefficient; the scale grows by at most
    % 1/sqrt(trustRetention) per sample and shrinks at once.
    % A primary problem infeasible inside the trust region is re-solved with
    % the scale doubled up to trustMaximumScale; the estimate is not changed.
    cfg.nonlinear=struct('integrationStep',.05,'trustRadius',.5, ...
        'trustInnovationMeters',.02,'trustInitialScale',.125, ...
        'trustMinimumScale',1/16,'trustMaximumScale',1,'trustRetention',.5);
    % Potential-field seed parameters; these are not optimized actuator limits.
    cfg.initialization=struct('previewSeconds',3,'previewStepSeconds',.1, ...
        'clearancePaddingMeters',.7,'brakingRatioLimit',.3, ...
        'lateralAccelerationFraction',.98,'frontForceFraction',.98,'speedGain',1);
    % encounterRangeMeters is the perception radius: the estimator receives
    % the target only inside it, and outside it there is no collision risk.
    % Collision rows apply at nodes inside it, including a later re-entry.
    % The terminal set requires only a separating relative velocity (and the
    % road); with a constant relative velocity the distance then never
    % decreases again.
    % safetyMarginMeters is the clearance required at the sampled collision
    % rows (hold start and midpoint), not a continuous-time clearance bound.
    % terminalSeparatingMarginMetersPerSecond only places the horizon end
    % where the anchor separates at that speed; a terminal row active at the
    % anchor stalled the conic solver.
    cfg.collision=struct('safetyMarginMeters',0.20,'encounterRangeMeters',50, ...
        'terminalSeparatingMarginMetersPerSecond',.5);
    cfg.vehicle=struct('m',1650,'Iz',1700,'lf',1.4,'lr',1.65, ...
        'wheelbase',3.05,'length',4.8,'width',1.9,'rectangleOffset',[0;0],'gravity',9.81);
    cfg.tire=struct('corneringStiffness',[96000;96000],'frictionCoefficient',[.85;.85]);
    cfg.actuation=struct('brakingRatioMinimum',-1,'brakingRatioMaximum',1);
    cfg.roadLoad=struct('airDensity',1.225,'dragCoefficient',.30,'frontalArea',2.2, ...
        'rollingCoefficient',.01,'rollingSpeedCoefficient',0,'rollingQuarticCoefficient',0, ...
        'rollingTransitionSpeed',.5);
    cfg.model=struct('speedMinimum',0,'speedMaximum',18,'scheduleSpeedFloor',1, ...
        'brakingRatioRateMaximum',Inf,'lateralVelocityMaximum',12,'yawRateMaximum',5);
    cfg.clf=struct('lateralPositionErrorScale',.5,'headingErrorScale',.1, ...
        'speedErrorScale',.25,'lateralVelocityErrorScale',.5,'yawRateErrorScale',.2, ...
        'frontWheelSteeringAngleWeight',1,'brakingRatioWeight',1);
    % One analytic quadratic CLF; decreaseFraction multiplies e' Q e.
    % The remaining nominalClf parameters guide initialization only.
    cfg.nominalClf=struct('lookaheadSeconds',1.5,'minimumLookaheadMeters',8,'courseGain',1.5, ...
        'yawRateGain',10,'lateralAccelerationFraction',.75,'frontForceFraction',.9, ...
        'speedGain',.5,'brakingRatioLimit',.35,'decreaseFraction',.5);
    % The two convex solves share the remaining controller-call budget.
    % An in-flight factorization can overrun this soft wall-clock limit.
    cfg.solver=struct('maxIterations',400,'timeLimitSeconds',5, ...
        'feasibilityTolerance',1e-5,'constraintTolerance',1e-8, ...
        'optimalityTolerance',1e-7,'lexicographicTieTolerance',1e-4, ...
        'clfTieTolerance',1e-4);
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
    for name=["horizonSteps","maximumHorizonSteps"]
        validateattributes(cfg.controller.(name),{'double'},{'scalar','real','finite','integer','positive'});
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
    for name=string(fieldnames(cfg.initialization)).'
        validateattributes(cfg.initialization.(name),{'double'},{'scalar','real','finite','positive'});
    end
    if cfg.initialization.previewStepSeconds>cfg.initialization.previewSeconds
        localInvalid('Initialization preview step cannot exceed its duration.');
    end
    for name=["lateralAccelerationFraction","frontForceFraction","brakingRatioLimit"]
        if cfg.initialization.(name)>=1,localInvalid('initialization.%s must lie in (0,1).',name);end
    end
    validateattributes(cfg.collision.safetyMarginMeters,{'double'},{'scalar','real','finite','nonnegative'});
    validateattributes(cfg.collision.encounterRangeMeters,{'double'},{'scalar','real','finite','positive'});
    validateattributes(cfg.collision.terminalSeparatingMarginMetersPerSecond,{'double'},{'scalar','real','finite','nonnegative'});
    if cfg.collision.encounterRangeMeters<=cfg.collision.safetyMarginMeters
        localInvalid('The encounter range must exceed the collision safety margin.');
    end
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
    if cfg.model.speedMaximum<=max(cfg.model.speedMinimum,cfg.model.scheduleSpeedFloor)
        localInvalid('The positive-speed model domain must have positive width.');
    end
    validateattributes(cfg.model.brakingRatioRateMaximum,{'double'},{'scalar','real','positive'});
    for name=string(fieldnames(cfg.clf)).'
        validateattributes(cfg.clf.(name),{'double'},{'scalar','real','finite','positive'});
    end
    for name=string(fieldnames(cfg.nominalClf)).'
        validateattributes(cfg.nominalClf.(name),{'double'},{'scalar','real','finite','positive'});
    end
    for name=["lateralAccelerationFraction","frontForceFraction","brakingRatioLimit","decreaseFraction"]
        if cfg.nominalClf.(name)>=1,localInvalid('nominalClf.%s must lie in (0,1).',name);end
    end
    validateattributes(cfg.solver.maxIterations,{'double'},{'scalar','real','finite','integer','positive'});
    validateattributes(cfg.solver.timeLimitSeconds,{'double'},{'scalar','real','positive'});
    for name=["feasibilityTolerance","constraintTolerance","optimalityTolerance","clfTieTolerance"]
        validateattributes(cfg.solver.(name),{'double'},{'scalar','real','finite','positive'});
    end
    validateattributes(cfg.solver.lexicographicTieTolerance,{'double'},{'scalar','real','finite','nonnegative'});
    for name=["defaultLength","defaultWidth","rearAxleDistance"]
        validateattributes(cfg.target.(name),{'double'},{'scalar','real','finite','positive'});
    end
end

function localInvalid(varargin)
    error('collisionAvoidanceController:invalidConfiguration',varargin{:});
end
