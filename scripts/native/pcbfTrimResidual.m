function [residual,offset]=pcbfTrimResidual(point,curvature,cfg)
    speed=cfg.referenceSpeed;beta=point(1);
    x=[0;0;-beta;speed*cos(beta);speed*sin(beta);speed*curvature];u=point(2:3);
    dx=nonlinearBicycleModel.derivative(x,u,cfg,curvature);
    residual=dx(4:6);
    offset=u-nonlinearBicycleModel.accelerationGuidance(x,[0;curvature*speed^2],u,cfg,1,true,true);
end
