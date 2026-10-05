function study = studyEstimatorDomain()
%studyEstimatorDomain Which estimator domain premises can be computed, and their cost.
% Derives ego domain values from the controller's vehicle, tire and
% state-limit configuration and re-synthesizes the observer with them,
% reporting the design ultimate bounds. Nothing in the estimator is changed.
%   - yaw-rate maximum: the controller's enforced |r| limit;
%   - single-track mismatch |r - vy/lr| = vx |tan(alpha_r)| / lr, bounded
%     only while the rear tire stays below its Fiala saturation slip
%     alpha_sat = atan(3 mu Fz / C), giving vmax tan(alpha_sat) / lr;
%   - sideslip inversion domain: the rear saturation slip;
%   - acceleration norm: mu g plus road load; yaw acceleration: axle
%     friction forces over Iz.
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'config'),fullfile(root,'controller'),fullfile(root,'estimator'));
    addpath(genpath(fullfile(root,'solver','YALMIP')));addpath(genpath(fullfile(root,'solver','sedumi')));
    cfg=collisionAvoidanceControllerConfig();
    g=cfg.vehicle.gravity;m=cfg.vehicle.m;lf=cfg.vehicle.lf;lr=cfg.vehicle.lr;
    mu=cfg.tire.frictionCoefficient;stiffness=cfg.tire.corneringStiffness;
    load=m*g*[lr;lf]/(lf+lr);saturation=atan(3*mu.*load./stiffness);
    speedMaximum=cfg.model.speedMaximum;
    derived=struct('rearSaturationSlip',saturation(2),'frontSaturationSlip',saturation(1), ...
        'mismatchMaximum',speedMaximum*tan(saturation(2))/lr, ...
        'accelerationNormMaximum',sum(mu.*load)/m+.5*cfg.roadLoad.airDensity*cfg.roadLoad.dragCoefficient ...
            *cfg.roadLoad.frontalArea*speedMaximum^2/m+cfg.roadLoad.rollingCoefficient*g, ...
        'yawAccelerationMaximum',[lf,lr]*(mu.*load)/cfg.vehicle.Iz, ...
        'yawRateMaximum',cfg.model.yawRateMaximum,'speedInterval',[max(cfg.model.speedMinimum,cfg.model.scheduleSpeedFloor),speedMaximum]);
    base=estimatorControllerIntegrationConfig().observer;base.ego.yaw.rearAxleDistance=lr;
    variants={"declared (current)",base};
    v=base;v.ego.domain.yawRateMaximum=derived.yawRateMaximum;v.ego.domain.speedMinimum=derived.speedInterval(1);
    variants(end+1,:)={"yaw rate and speed from controller limits",v};
    v.ego.yaw.singleTrackYawRateMismatchMaximum=derived.mismatchMaximum;
    variants(end+1,:)={"+ mismatch from rear saturation",v};
    v.ego.yaw.sideslipDomainMaximum=derived.rearSaturationSlip;
    variants(end+1,:)={"+ sideslip domain = rear saturation slip",v};
    rows=struct('variant',{},'bodyVelocity',{},'position',{},'relativePosition',{},'targetVelocity',{},'targetAcceleration',{});
    for index=1:size(variants,1)
        design=synthesizeNrmmObserverGains(variants{index,2});u=design.ultimateBounds;
        rows(end+1)=struct('variant',variants{index,1},'bodyVelocity',u.bodyVelocity,'position',u.position, ...
            'relativePosition',u.relativePosition,'targetVelocity',u.targetVelocity,'targetAcceleration',u.targetAcceleration); %#ok<AGROW>
        fprintf('%-42s velocity %.3f m/s, position %.3f m, relative position %.3f m, target velocity %.3f m/s, target acceleration %.3f m/s^2\n', ...
            variants{index,1},u.bodyVelocity,u.position,u.relativePosition,u.targetVelocity,u.targetAcceleration);
    end
    study=struct('derived',derived,'ultimateBounds',rows);
end
