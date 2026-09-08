function [force, slope, components] = longitudinalRoadLoad(speed, cfg)
%longitudinalRoadLoad Signed passive road load and its speed derivative.
% Flat road, still air, and a quasi-static equivalent rolling force are
% assumed. Positive force opposes forward travel. The smooth rolling sign
% preserves rest without applying a constant backward force at zero speed.
% Polynomial rolling coefficients have units 1, s/m, and (s/m)^4.
% Wheel slip, wheel inertia, camber, and dynamic normal-load effects are
% residual dynamics, not reproduced by this reduced road-load model.

    roadLoad = cfg.roadLoad;
    magnitude = abs(speed);
    direction = tanh(speed/roadLoad.rollingTransitionSpeed);
    rollingCoefficient = roadLoad.rollingCoefficient ...
        + roadLoad.rollingSpeedCoefficient*magnitude ...
        + roadLoad.rollingQuarticCoefficient*magnitude.^4;
    aerodynamicFactor = 0.5*roadLoad.airDensity ...
        * roadLoad.dragCoefficient*roadLoad.frontalArea;
    aerodynamic = aerodynamicFactor*speed.*magnitude;
    rollingScale = cfg.vehicle.m*cfg.vehicle.gravity;
    rolling = rollingScale*rollingCoefficient.*direction;
    force = aerodynamic+rolling;
    if nargout > 1
        slope = 2*aerodynamicFactor*magnitude ...
            + rollingScale*((roadLoad.rollingSpeedCoefficient ...
                + 4*roadLoad.rollingQuarticCoefficient*magnitude.^3) ...
                .*sign(speed).*direction ...
                + rollingCoefficient.*(1-direction.^2)/roadLoad.rollingTransitionSpeed);
    end
    if nargout > 2
        components = struct("aerodynamicForce", aerodynamic, ...
            "rollingResistanceForce", rolling);
    end
end
