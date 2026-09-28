function cfg = collisionAvoidanceControllerConfig(userCfg)
%collisionAvoidanceControllerConfig Joint PCBF/CLF/SCvx parameters in SI units.
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
    cfg.controller=struct('sampleTime',.05,'horizonSteps',16,'maximumHorizonSteps',512);
    cfg.nonlinear=struct('integrationStep',.01,'terminalRadius',.25, ...
        'maximumIterations',24,'trustRadius',.5, ...
        'recoveryHorizonSeconds',3,'clfDecay',.01);
    cfg.collision=struct('safetyMarginMeters',.10);
    cfg.vehicle=struct('m',1650,'Iz',1700,'lf',1.4,'lr',1.65, ...
        'wheelbase',3.05,'length',4.8,'width',1.9,'rectangleOffset',[0;0],'gravity',9.81);
    cfg.tire=struct('corneringStiffness',[96000;96000],'frictionCoefficient',[.85;.85]);
    cfg.actuation=struct('brakingRatioMinimum',-1,'brakingRatioMaximum',1);
    cfg.roadLoad=struct('airDensity',1.225,'dragCoefficient',.30,'frontalArea',2.2, ...
        'rollingCoefficient',.01,'rollingSpeedCoefficient',0,'rollingQuarticCoefficient',0, ...
        'rollingTransitionSpeed',.5);
    cfg.model=struct('speedMinimum',0,'speedMaximum',18,'scheduleSpeedFloor',1, ...
        'frontWheelSteeringAngleMaximum',deg2rad(40),'frontWheelSteeringRateMaximum',Inf, ...
        'brakingRatioRateMaximum',Inf,'lateralVelocityMaximum',12,'yawRateMaximum',5);
    cfg.clf=struct('lateralPositionErrorScale',.5,'headingErrorScale',.1, ...
        'speedErrorScale',.25,'lateralVelocityErrorScale',.5,'yawRateErrorScale',.2, ...
        'frontWheelSteeringAngleWeight',1,'brakingRatioWeight',1,'relaxationWeight',100);
    % The time limit stops SCvx between iterations. A running LP/QP may
    % overrun it; this is not a guaranteed wall-clock deadline.
    cfg.solver=struct('maxIterations',400,'timeLimitSeconds',5, ...
        'feasibilityTolerance',1e-5,'constraintTolerance',1e-8, ...
        'optimalityTolerance',1e-7,'lexicographicTieTolerance',1e-6);
    cfg.target=struct('defaultLength',4.8,'defaultWidth',1.9);
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
    for name=["integrationStep","terminalRadius","trustRadius","recoveryHorizonSeconds","clfDecay"]
        validateattributes(cfg.nonlinear.(name),{'double'},{'scalar','real','finite','positive'});
    end
    for name="maximumIterations"
        validateattributes(cfg.nonlinear.(name),{'double'},{'scalar','real','finite','integer','positive'});
    end
    if cfg.nonlinear.clfDecay>=1,localInvalid('CLF decay must lie in (0,1).');end
    validateattributes(cfg.collision.safetyMarginMeters,{'double'},{'scalar','real','finite','positive'});
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
    for name=["speedMaximum","scheduleSpeedFloor","frontWheelSteeringAngleMaximum", ...
            "lateralVelocityMaximum","yawRateMaximum"]
        validateattributes(cfg.model.(name),{'double'},{'scalar','real','finite','positive'});
    end
    if cfg.model.speedMaximum<=max(cfg.model.speedMinimum,cfg.model.scheduleSpeedFloor)
        localInvalid('The positive-speed model domain must have positive width.');
    end
    for name=["frontWheelSteeringRateMaximum","brakingRatioRateMaximum"]
        validateattributes(cfg.model.(name),{'double'},{'scalar','real','positive'});
    end
    for name=string(fieldnames(cfg.clf)).'
        validateattributes(cfg.clf.(name),{'double'},{'scalar','real','finite','positive'});
    end
    validateattributes(cfg.solver.maxIterations,{'double'},{'scalar','real','finite','integer','positive'});
    validateattributes(cfg.solver.timeLimitSeconds,{'double'},{'scalar','real','positive'});
    for name=["feasibilityTolerance","constraintTolerance","optimalityTolerance"]
        validateattributes(cfg.solver.(name),{'double'},{'scalar','real','finite','positive'});
    end
    validateattributes(cfg.solver.lexicographicTieTolerance,{'double'},{'scalar','real','finite','nonnegative'});
    for name=["defaultLength","defaultWidth"]
        validateattributes(cfg.target.(name),{'double'},{'scalar','real','finite','positive'});
    end
end

function localInvalid(varargin)
    error('collisionAvoidanceController:invalidConfiguration',varargin{:});
end
