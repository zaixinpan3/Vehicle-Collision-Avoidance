function inputs=pcbfCruiseInputs(x,previous,curve,trimOffset,count,cfg)
    lane=struct('referenceCurve',curve);
    model=struct('cfg',cfg,'nominalReference',struct('state',[0;0;0;cfg.referenceSpeed;0;0], ...
        'curvature',curve.curvature));
    inputs=zeros(2,count);
    lower=[-cfg.outputFeedback.steeringMaximum;max(-1,cfg.actuation.brakingRatioMinimum)];
    upper=[cfg.outputFeedback.steeringMaximum;min(1,cfg.actuation.brakingRatioMaximum)];
    delta=cfg.controller.sampleTime*[cfg.outputFeedback.steeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    for i=1:count
        projection=predictiveSafetyGeometry.project(x(1:2),lane);
        acceleration=predictiveSafetyGeometry.routeAcceleration(x,projection,model);
        guided=nonlinearBicycleModel.accelerationGuidance(x,acceleration,previous,cfg,1,true,true);
        previous=max(lower,min(upper,max(previous-delta,min(previous+delta,guided+trimOffset))));
        inputs(:,i)=previous;
        if i<count,x=nonlinearBicycleModel.sample(x,previous,cfg);end
    end
end
