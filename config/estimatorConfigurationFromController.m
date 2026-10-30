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
    % The observer bounds the velocity NORM; the vehicle constraint bounds
    % its longitudinal component. Include the allowed lateral component.
    lateralMaximum=min(controller.model.lateralVelocityMaximum, ...
        tan(controller.model.sideslipMaximum)*controller.model.speedMaximum);
    observer.ego.domain.speedMaximum = hypot(controller.model.speedMaximum,lateralMaximum);
    observer.ego.domain.yawRateMaximum = controller.model.yawRateMaximum;
    observer.ego.yaw.sideslipDomainMaximum = controller.model.sideslipMaximum;
    uncertainty = 0.05;
    if isfield(observer.ego,"model") && isfield(observer.ego.model,"parameterUncertainty")
        uncertainty = observer.ego.model.parameterUncertainty;
    end
    % These global fallback bounds cover the entire declared friction
    % parameter family, including parameters above the nominal calibration.
    observer.ego.domain.accelerationNormMaximum = ((1+uncertainty)*sum(friction) ...
        +nonlinearBicycleModel.roadLoad(controller.model.speedMaximum,controller))/vehicle.m;
    observer.ego.domain.yawAccelerationMaximum = (1+uncertainty)*[vehicle.lf,vehicle.lr]*friction/vehicle.Iz;
    observer.ego.model = struct("mass",vehicle.m,"lf",vehicle.lf,"lr",vehicle.lr, ...
        "gravity",vehicle.gravity,"corneringStiffness",tire.corneringStiffness(:), ...
        "frictionCoefficient",tire.frictionCoefficient(:),"parameterUncertainty",uncertainty);
    if ~isempty(targetContract)
        for field = string(fieldnames(targetContract)).'
            observer.target.domain.(field) = targetContract.(field);
        end
    end
    estimator.observer = observer;
    % Initialization may bound position-secant acceleration bias using the
    % same plant and known actuator history. This is not a velocity sensor.
    estimator.initialization.vehicleModel = controller;
end
