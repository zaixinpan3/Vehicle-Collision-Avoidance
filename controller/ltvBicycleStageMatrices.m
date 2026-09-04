function [stateMatrix, inputMatrix, affineVector] = ...
        ltvBicycleStageMatrices(kappa, vBar, sampleTime, cfg)
% ltvBicycleStageMatrices Forward-Euler Frenet bicycle matrices at a schedule point.
%
% Closed-form linearization of the dynamic bicycle with linear
% cornering, written in PATH COORDINATES along the lane centerline -
% state [s; d; ePsi; vx; vy; r] with s the station, d the left-positive
% lateral offset and ePsi the heading error to the path tangent - about
% the schedule point (d = 0, ePsi = 0, vy = 0, vx = vBar, r = kappa*vBar)
% at the local curvature kappa, discretized by ONE FORWARD-EULER step:
%
%   A_d = I + Ts*A,   B_d = Ts*B,   c_d = Ts*c,   c = f(xhat) - A*xhat.
%
% Continuous model:
%   sdot    = (vx cos ePsi - vy sin ePsi)/(1 - kappa d)
%   ddot    = vx sin ePsi + vy cos ePsi
%   ePsidot = r - kappa sdot
%   vxdot   = a + vy r
%   vydot   = (Fyf + Fyr)/m - vx r
%   rdot    = (lf Fyf - lr Fyr)/Iz
%   Fyf = Cf (deltaF - (vy + lf r)/vBar),   Fyr = -Cr (vy - lr r)/vBar
%
% Input [deltaF; a] (Ge et al. 2022). The curvature is treated as
% locally constant at the schedule station of the stage (its variation
% along the horizon is carried node by node by the schedule). The Euler
% map IS the declared discrete model (design decision 2026-08-23); it is
% conditionally stable - sampleTime times (Cf+Cr)/(m*vBar) and
% (lf^2*Cf + lr^2*Cr)/(Iz*vBar) must stay below 2, an obligation the
% sample time and the schedule speed floor carry together. This is the
% single model source for the prediction and for the CLF Riccati
% synthesis (at kappa = 0).

    mass = cfg.vehicle.m;
    yawInertia = cfg.vehicle.Iz;
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;
    corneringStiffness = double(cfg.tire.corneringStiffness(:));
    if isscalar(corneringStiffness)
        corneringStiffness = repmat(corneringStiffness, 2, 1);
    end
    if numel(corneringStiffness) ~= 2 ...
            || any(~isfinite(corneringStiffness)) ...
            || any(corneringStiffness <= 0.0)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "tire.corneringStiffness must be positive and scalar or " ...
            + "contain front/rear values.");
    end
    corneringFront = corneringStiffness(1);
    corneringRear = corneringStiffness(2);
    rBar = kappa*vBar;

    continuousA = zeros(6, 6);
    continuousB = zeros(6, 2);
    continuousC = zeros(6, 1);
    % Path kinematics linearized at (d = 0, ePsi = 0, vy = 0, vx = vBar):
    % every affine term of these rows vanishes at the schedule point.
    continuousA(1, 4) = 1.0;
    continuousA(1, 2) = kappa*vBar;
    continuousA(2, 3) = vBar;
    continuousA(2, 5) = 1.0;
    continuousA(3, 6) = 1.0;
    continuousA(3, 4) = -kappa;
    continuousA(3, 2) = -kappa^2*vBar;
    % vxdot = a + vy*r, bilinear term frozen at (vy = 0, r = rBar).
    continuousA(4, 5) = rBar;
    continuousB(4, 2) = 1.0;
    % Lateral channel at the frozen speed.
    yawStiffness = (lf*corneringFront-lr*corneringRear)/vBar;
    lateralStiffness = (corneringFront+corneringRear)/vBar;
    continuousA(5, 4) = -rBar;
    continuousA(5, 5) = -lateralStiffness/mass;
    continuousA(5, 6) = -(yawStiffness/mass+vBar);
    continuousB(5, 1) = corneringFront/mass;
    continuousC(5) = rBar*vBar;
    continuousA(6, 5) = -yawStiffness/yawInertia;
    continuousA(6, 6) = -(lf^2*corneringFront ...
        + lr^2*corneringRear)/(vBar*yawInertia);
    continuousB(6, 1) = lf*corneringFront/yawInertia;

    stateMatrix = eye(6)+sampleTime*continuousA;
    inputMatrix = sampleTime*continuousB;
    affineVector = sampleTime*continuousC;
end
