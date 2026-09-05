function [stateMatrix, inputMatrix, affineVector] = ...
        ltvBicycleStageMatrices(kappa, vBar, sampleTime, cfg)
% ltvBicycleStageMatrices Exact held-input flow of a scheduled affine bicycle.
%
% Closed-form linearization of the dynamic bicycle with linear
% cornering regularized at a positive tire-speed floor, in path coordinates
% along the lane centerline -
% state [s; d; ePsi; vx; vy; r] with s the station, d the left-positive
% lateral offset and ePsi the heading error to the path tangent - about
% the schedule point (d = 0, ePsi = 0, vy = 0, vx = vBar, r = kappa*vBar)
% at the local curvature kappa. One block matrix exponential integrates
% the affine model with constant input over the sample. This is exact for
% that scheduled linearization, not for the nonlinear bicycle or plant.
%
% Continuous model:
%   sdot    = (vx cos ePsi - vy sin ePsi)/(1 - kappa d)
%   ddot    = vx sin ePsi + vy cos ePsi
%   ePsidot = r - kappa sdot
%   vxdot   = gamma*a + vy r, gamma = cfg.model.longitudinalInputGain
%   vydot   = (Fyf + Fyr)/m - vx r
%   rdot    = (lf Fyf - lr Fyr)/Iz
%   Fyf = Cf (deltaF - (vy + lf r)/vTire), Fyr = -Cr (vy - lr r)/vTire
%   vTire = max(vBar, cfg.model.scheduleSpeedFloor)
%
% Input [deltaF; a] uses commanded acceleration, with a fixed declared
% longitudinal effectiveness gain. The default gain is one. The curvature is treated as
% locally constant at the schedule station of the stage (its variation
% along the horizon is carried node by node by the schedule). Prediction,
% continuation and CLF Riccati synthesis use this same held-input flow.
% Position and heading can therefore depend on the new first input. The
% discretization does not impose a forward-Euler stiffness restriction.

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
    % At zero schedule speed the rest state is invariant. Only the tire
    % denominator is regularized; kinematic transport uses vBar itself.
    tireSpeed = max(vBar, cfg.model.scheduleSpeedFloor);
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
    % vxdot = gamma*a + vy*r, frozen at (vy = 0, r = rBar).
    continuousA(4, 5) = rBar;
    continuousB(4, 2) = cfg.model.longitudinalInputGain;
    % Lateral channel at the frozen speed.
    yawStiffness = (lf*corneringFront-lr*corneringRear)/tireSpeed;
    lateralStiffness = (corneringFront+corneringRear)/tireSpeed;
    continuousA(5, 4) = -rBar;
    continuousA(5, 5) = -lateralStiffness/mass;
    continuousA(5, 6) = -(yawStiffness/mass+vBar);
    continuousB(5, 1) = corneringFront/mass;
    continuousC(5) = rBar*vBar;
    continuousA(6, 5) = -yawStiffness/yawInertia;
    continuousA(6, 6) = -(lf^2*corneringFront ...
        + lr^2*corneringRear)/(tireSpeed*yawInertia);
    continuousB(6, 1) = lf*corneringFront/yawInertia;

    heldTransition = expm(sampleTime*[continuousA, continuousB, continuousC; ...
        zeros(3, 9)]);
    stateMatrix = heldTransition(1:6, 1:6);
    inputMatrix = heldTransition(1:6, 7:8);
    affineVector = heldTransition(1:6, 9);
end
