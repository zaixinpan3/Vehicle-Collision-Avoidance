function [model,previousState] = ...
        debugControllerModel(egoState,targetEstimate,laneCenterline,cfg,previousState)
%collisionAvoidanceController Two-stage PCBF/CLF optimization; no fallback algorithm.
% Controller state retains a target prediction and a linearization trajectory.
% Every current target estimate updates the prediction; its A and beta are
% held constant within this frame, independently of proof-metadata availability.
% Passing [] as previousState starts a new problem and a new target epoch.
% Collision interaction is active inside the configured position radius.
% A complete current sensor scan with no target clears the previous forecast.
    lastState=[];
    if nargin==1 && (ischar(egoState) || isstring(egoState))
        if ~isscalar(string(egoState)) || string(egoState)~="resetNominalTrajectory"
            error('collisionAvoidanceController:invalidAction','Use resetNominalTrajectory.');
        end
        lastState=[];command=[];predictedInput=[];prediction=[];controllerState=[];return;
    end
    explicitState=nargin>=5;
    if ~explicitState,previousState=lastState;end
    if isstruct(previousState) && (~isfield(previousState,'version') || previousState.version~=77),previousState=[];end
    if nargin<4,cfg=[];end
    if nargin<3,laneCenterline=[];end
    if nargin<2,targetEstimate=[];end
    timer=tic;cfg=collisionAvoidanceControllerConfig(cfg);
    [ego,lane,road,observation]=readControllerInputs(egoState,targetEstimate,laneCenterline,cfg);
    if isempty(targetEstimate) && isfield(egoState,'targetEstimate'),targetEstimate=egoState.targetEstimate;end
    observed=predictiveSafetyGeometry.target(observation,targetEstimate,cfg);
    uncertainty=predictiveSafetyGeometry.observerUncertainty(ego,observation,observed);
    frame=predictiveSafetyGeometry.roadFrame(lane,road);
    context={cfg.vehicle,cfg.tire,cfg.roadLoad,cfg.model,cfg.actuation,cfg.collision, ...
        cfg.controller,cfg.referenceSpeed,cfg.clf,cfg.nominalClf,cfg.initialization, ...
        cfg.nonlinear.integrationStep,cfg.terminal,frame,lane};
    index=0;epochTime=ego.stateTime;targetEpoch=observed;
    if isstruct(previousState)
        if ~isequaln(context,previousState.context)
            error('collisionAvoidanceController:changedContinuationProblem', ...
                'Model, constraints or reference path changed. Pass an empty previousState to initialize a new problem.');
        end
        index=previousState.sampleIndex+1;epochTime=previousState.epochTime;targetEpoch=previousState.targetEpoch;
        expectedTime=epochTime+index*cfg.controller.sampleTime;
        if isfinite(ego.stateTime) && isfinite(epochTime) && abs(ego.stateTime-expectedTime)>1e-9*max(1,abs(expectedTime))
            error('collisionAvoidanceController:invalidSampleTime','Continuation requires consecutive absolute sample times.');
        end
    end
    if isempty(observed) && isfield(egoState,'perception')
        perception=egoState.perception;
        if isstruct(perception) && isscalar(perception) ...
                && all(isfield(perception,{'time','completeWithinRange'})) ...
                && isequal(perception.completeWithinRange,true) && isequal(perception.time,ego.stateTime)
            targetEpoch=[];
        end
    end
    q=predictiveSafetyGeometry.predictTarget(targetEpoch,index*cfg.controller.sampleTime);
    if ~isempty(observed) && ~isempty(q)
        observed(3)=q(3)+atan2(sin(observed(3)-q(3)),cos(observed(3)-q(3)));
    end
    if ~isempty(observed)
        % Reanchor the prediction at the current estimate without resetting
        % the ego input warm start. Cross-frame constancy is not required.
        targetEpoch=predictiveSafetyGeometry.predictTarget(observed,-index*cfg.controller.sampleTime);
        q=observed;
    end
    previous=zeros(2,1);
    if ~isempty(ego.heldActuatorInput),previous=ego.heldActuatorInput;
    elseif isstruct(previousState),previous=previousState.appliedInput;
    elseif isfinite(cfg.model.brakingRatioRateMaximum)
        error('collisionAvoidanceController:missingInputMemory','Finite slew limits require the previous applied input.');
    end
    jointState=ego.modelState;targetParameters=zeros(7,0);
    if ~isempty(q),jointState=[jointState;q(1:4)];targetParameters=q(5:11);end
    nominalReference=nonlinearBicycleModel.cruise(cfg,frame(4));
    window=cfg.terminal.horizonSeconds;h=cfg.controller.sampleTime;encounterStart=NaN;
    if ~isempty(q)
        encounterStart=index;
        if isstruct(previousState) && isfield(previousState,'encounterStart') && isfinite(previousState.encounterStart) ...
                && (index-previousState.encounterStart)*h<window-1e-9
            encounterStart=previousState.encounterStart;
        end
    end
    remaining=window;if isfinite(encounterStart),remaining=window-(index-encounterStart)*h;end
    model=struct('cfg',cfg,'initialState',ego.modelState,'jointState',jointState, ...
        'targetParameters',targetParameters,'previousInput',previous,'target',q,'targetEpoch',targetEpoch, ...
        'sampleIndex',index,'epochTime',epochTime,'lane',lane,'road',road,'frame',frame, ...
        'nominalReference',nominalReference,'stateTime',ego.stateTime, ...
        'encounterWindowSeconds',remaining, ...
        'uncertainty',uncertainty);
end

function command=localCommand(input,state,cfg)
    tire=modifiedFialaTire.parameters(cfg);force=modifiedFialaTire.longitudinalForce(input(2),cfg);
    slip=atan2([state(5)+cfg.vehicle.lf*state(6);state(5)-cfg.vehicle.lr*state(6)],state(4))-[input(1);0];
    lateral=modifiedFialaTire.evaluate(slip,input(2),cfg);dx=nonlinearBicycleModel.derivative(state,input,cfg);
    [road,~,components]=nonlinearBicycleModel.roadLoad(state(4),cfg);
    rolling=components.rollingResistanceForce*tire.staticNormalLoad/(cfg.vehicle.m*cfg.vehicle.gravity);
    command=struct('actuatorInput',input,'frontWheelSteeringAngle',input(1),'brakingRatio',input(2), ...
        'actuatorInputOrder',["frontWheelSteeringAngle","brakingRatio"], ...
        'longitudinalAcceleration',modifiedFialaTire.accelerationGain(cfg)*input(2), ...
        'bodyLongitudinalVelocityDerivative',dx(4),'lateralAcceleration',dx(5),'yawAcceleration',dx(6), ...
        'totalLongitudinalActuatorForce',sum(force),'totalLongitudinalTireForce',sum(force-rolling), ...
        'axleLongitudinalTireForce',force-rolling,'axleLongitudinalForce',force, ...
        'axleLateralForce',lateral,'axleNormalLoad',tire.staticNormalLoad,'tireSideslipAngle',slip, ...
        'aerodynamicResistanceForce',components.aerodynamicForce, ...
        'rollingResistanceForce',components.rollingResistanceForce,'totalRoadLoadForce',road);
end
