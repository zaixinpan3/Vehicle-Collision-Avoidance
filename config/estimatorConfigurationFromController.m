function estimator = estimatorConfigurationFromController(estimator, controller, targetContract)
%estimatorConfigurationFromController Derive the estimator's premises from the controller.
% The estimator and controller share one vehicle and one operating domain:
%   - ego speed interval and yaw-rate maximum: the controller's enforced
%     state limits (cfg.model);
%   - ego acceleration norm: tire friction plus road load at the maximum
%     speed; yaw acceleration: axle friction forces over Iz;
%   - sideslip cone |v_y| <= ||v|| sin(b): the controller's
%     model.sideslipMaximum, which the controller also enforces;
%   - ego vehicle model for the force-balance lateral measurement: mass,
%     axle distances and Fiala tires, with the estimator's declared relative
%     parameter uncertainty (observer.ego.model.parameterUncertainty);
%   - target domain: targetContract (collisionThreatContract) when given.
% Sensor bounds and the realization settings are left unchanged.
    arguments
        estimator (1,1) struct
        controller (1,1) struct
        targetContract struct = struct([])
    end
    vehicle = controller.vehicle;tire = controller.tire;
    load = vehicle.m*vehicle.gravity/(vehicle.lf+vehicle.lr)*[vehicle.lr;vehicle.lf];
    friction = tire.frictionCoefficient(:).*load;
    observer = estimator.observer;
    observer.ego.yaw.rearAxleDistance = vehicle.lr;
    observer.ego.domain.speedMinimum = max(controller.model.speedMinimum,controller.model.scheduleSpeedFloor);
    observer.ego.domain.speedMaximum = controller.model.speedMaximum;
    observer.ego.domain.yawRateMaximum = controller.model.yawRateMaximum;
    observer.ego.domain.accelerationNormMaximum = (sum(friction) ...
        +nonlinearBicycleModel.roadLoad(controller.model.speedMaximum,controller))/vehicle.m;
    observer.ego.domain.yawAccelerationMaximum = [vehicle.lf,vehicle.lr]*friction/vehicle.Iz;
    observer.ego.yaw.sideslipDomainMaximum = controller.model.sideslipMaximum;
    uncertainty = 0.05;
    if isfield(observer.ego,"model") && isfield(observer.ego.model,"parameterUncertainty")
        uncertainty = observer.ego.model.parameterUncertainty;
    end
    observer.ego.model = struct("mass",vehicle.m,"lf",vehicle.lf,"lr",vehicle.lr, ...
        "gravity",vehicle.gravity,"corneringStiffness",tire.corneringStiffness(:), ...
        "frictionCoefficient",tire.frictionCoefficient(:),"parameterUncertainty",uncertainty);
    if ~isempty(targetContract)
        for field = string(fieldnames(targetContract)).'
            observer.target.domain.(field) = targetContract.(field);
        end
    end
    estimator.observer = observer;
end
