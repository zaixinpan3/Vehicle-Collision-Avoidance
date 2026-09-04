function certificate = terminalLateralCertificate(cfg, curvatureMaximum)
% terminalLateralCertificate Closed-form certificate of the tail's lateral band.
%
% During the braking tail the ego is steered by the lane-keeping law
% that holds the lateral offset it has at the terminal node and drives
% its heading error to zero. In ARCLENGTH sigma the kinematic bicycle's
% lateral error [Delta d; e] under the law
% delta = L (kappa - k_d Delta d - k_psi e) obeys the linear
% time-invariant system
%
%   Delta d' = e,   e' = -k_d Delta d - k_psi e,
%
% independent of speed: braking only sets how fast sigma runs, so the
% band is a property of the distance travelled, not of the speed
% profile. With k_d = omega^2 and k_psi = 2 omega (critically damped,
% omega = terminal.lateralBandwidth in 1/m) and the initial error
% [0; e_0], the response is
%
%   Delta d(sigma) = e_0 sigma exp(-omega sigma),
%   e(sigma)       = e_0 (1 - omega sigma) exp(-omega sigma),
%
% so the largest lateral excursion is |e_0| / (omega e), attained at
% sigma = 1/omega, and |e| <= |e_0| for every sigma >= 0
% (gammaHeading = 1). The band the rows charge is the SHIFT-CONSISTENT
% one: at the next sample the tail restarts from a point of this very
% response, with the new terminal offset d_N + Delta d(sigma_1) and the
% new heading e(sigma_1), and the shifted candidate must lie inside the
% band the new rows charge. The band about the current offset that
% contains every later band about a later offset is
%
%   gammaLateral = sup_{x > 0} x exp(-x) / (1 - (1 - x) exp(-x)) / omega
%               = 1 / (2 omega),
%
% the supremum being approached as x -> 0 (it is 1/e at x = 1). With
% |Delta d| <= gammaLateral |e_0| and |e| <= |e_0| the steering the law
% demands is |delta| <= L (kappa_max + omega (2 + omega gammaLateral)
% |e_0|) = L (kappa_max + 2.5 omega |e_0|), and the lateral
% acceleration it needs at speed v is v^2 |delta| / L. Holding that
% under terminal.lateralAccelerationMaximum at the model's speed
% maximum, and the steering inside its box, gives the HEADING BOUND
%
%   |e_0| <= min( a_y,max / v_max^2 - kappa_max,
%                 delta_max / L   - kappa_max ) / (2.5 omega),
%
% the terminal row on the heading error (net of the course
% contributions of the terminal lateral velocity and yaw-rate error,
% which the caller charges). The lateral acceleration budget is
% validated against the axle friction polygons together with the
% backup deceleration in collisionAvoidanceControllerConfig.
% yawTimeConstant is the yaw-rate mode's time constant of the dynamic
% bicycle at the speed maximum, Iz v / (lf^2 Cf + lr^2 Cr): the time
% over which a terminal yaw-rate error still turns the heading before
% the kinematic law is in charge. Everything here is a closed form of a
% two-state linear system; no optimization is solved and none is
% needed.

    omega = cfg.terminal.lateralBandwidth;
    gammaLateral = 1.0/(2.0*omega);
    gammaHeading = 1.0;
    steeringGain = omega*(2.0+omega*gammaLateral);
    speedMaximum = cfg.model.speedMaximum;
    wheelbase = cfg.vehicle.wheelbase;
    curvatureMaximum = abs(double(curvatureMaximum));
    lateralBudget = cfg.terminal.lateralAccelerationMaximum ...
        / speedMaximum^2-curvatureMaximum;
    steeringBudget = cfg.model.frontWheelSteeringAngleMaximum ...
        / wheelbase-curvatureMaximum;
    headingBound = min(lateralBudget, steeringBudget)/steeringGain;
    if ~(headingBound > 0.0)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "The terminal lateral certificate has no heading budget: " ...
            + "the route curvature %.4f 1/m uses up the lateral " ...
            + "acceleration (%.2f m/s^2 at %.1f m/s) or the steering " ...
            + "box of the tail's lane-keeping law.", curvatureMaximum, ...
            cfg.terminal.lateralAccelerationMaximum, speedMaximum);
    end
    corneringStiffness = double(cfg.tire.corneringStiffness(:));
    if isscalar(corneringStiffness)
        corneringStiffness = repmat(corneringStiffness, 2, 1);
    end
    yawTimeConstant = cfg.vehicle.Iz*speedMaximum ...
        / (cfg.vehicle.lf^2*corneringStiffness(1) ...
            + cfg.vehicle.lr^2*corneringStiffness(2));
    certificate = struct( ...
        "bandwidth", omega, ...
        "gammaLateral", gammaLateral, ...
        "gammaHeading", gammaHeading, ...
        "steeringGain", steeringGain, ...
        "curvatureMaximum", curvatureMaximum, ...
        "headingBound", headingBound, ...
        "yawTimeConstant", yawTimeConstant);
end
