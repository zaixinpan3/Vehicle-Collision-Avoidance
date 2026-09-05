function cfg = nrmmTrackingConfig()
% nrmmTrackingConfig Physical bounds and sampled NRMM estimator settings.
%
% Online estimation separates yaw information from body-relative tracking.
% The target nominal estimate comes from an exact constant-A/curvature window
% fit. Independent analytic outer enclosures use the physical domain, bounded
% sensor errors, and explicit model/intersample variation bounds. Sensor biases
% are compensated upstream. See estimator/OBSERVER_ISS_THEORY.md.

    cfg.runtime.samplePeriod = 0.02;                 % s
    cfg.runtime.integrationStepMaximum = 0.005;      % s, continuous-comparator only
    % The online estimator fits a constant-acceleration/curvature segment.
    % These are explicit information-versus-response choices, not ISS gains.
    cfg.window.duration = 0.8;                       % s
    cfg.window.minimumFitSpan = 0.12;                % s
    cfg.window.maximumIterations = 40;
    cfg.window.numericalAllowance = 1.0e-9;
    % Optional bounds valid THROUGHOUT each sample interval. Inf preserves
    % the original unrestricted intersample model; then domain-only motion
    % enclosures are used. A point sensor bound alone never bounds a hold.
    cfg.ego.intersample.accelerationMaximum = Inf;   % m/s^2
    cfg.ego.intersample.yawAccelerationMaximum = Inf; % rad/s^2
    % Zero retains the exact model. Nonzero rates enlarge the hard jerk
    % enclosure; the nominal fit still uses constant A and curvature.
    cfg.target.model.scalarAccelerationRateMaximum = 0.0; % m/s^3
    cfg.target.model.curvatureRateMaximum = 0.0;      % 1/(m s)

    %% Ego operating domain (Assumption 1)
    % The positive lower speed makes the corrected GNSS-course channel
    % uniformly informative. It is a physical operating-domain assumption,
    % not a yaw-gain tuning threshold.
    cfg.ego.domain.speedMinimum = 5.0;               % m/s   V_{E,min}
    cfg.ego.domain.speedMaximum = 20.0;              % m/s   Vbar_E
    cfg.ego.domain.yawRateMaximum = 0.30;            % rad/s omegabar_E

    %% Joint body-velocity and circular yaw measurement information
    % The course channel estimates side slip pointwise from the measured
    % yaw rate and GNSS speed using the kinematic single-track relation.
    % sideslipDomainMaximum only selects the invertible principal branch;
    % it is not added as a fixed course-yaw disturbance. The declared
    % yaw-rate mismatch bounds unmodeled rear-tire slip and other departures
    % from omegaE = VE*sin(betaE)/lrE.
    cfg.ego.yaw.rearAxleDistance = 1.45;             % m l_{r,E}
    cfg.ego.yaw.sideslipDomainMaximum = 0.05;        % rad betaDomain_E
    cfg.ego.yaw.singleTrackYawRateMismatchMaximum = 0.02; % rad/s dbar_st
    % The direction certificate uses the measured GNSS speed directly; no
    % reliability score or fixed validity threshold is configured.

    %% Target operating domain
    cfg.target.domain.speedMinimum = 10.0;           % m/s V_{C,min}
    cfg.target.domain.speedMaximum = 20.0;           % m/s Vbar_C
    cfg.target.domain.scalarAccelerationMaximum = 2.0; % m/s^2 Abar_C
    cfg.target.domain.sideslipMaximum = 0.015;       % rad beta_max
    cfg.target.domain.rearAxleDistance = 1.6;        % m l_r
    cfg.target.domain.relativePositionMaximum = 50.0; % m rhobar
    %% Deterministic sensor error bounds
    % Input contract: GNSS position/velocity are inertial-frame,
    % center-of-mass, lever-arm-compensated samples; IMU acceleration is
    % ego-body-frame at the center of mass, gravity- and lever-arm-
    % compensated; radar relative position is ego-body-frame from the ego
    % center of mass to the tracked target reference point, extrinsics-
    % compensated. Synchronization to one common sample period happens
    % upstream of the observer.
    cfg.measurement.gps.positionNoiseMaximum = 0.12; % m nbar_p
    cfg.measurement.gps.velocityNoiseMaximum = 0.02; % m/s nbar_v
    % Accelerometer and gyroscope biases are neglected: the deployed
    % inertial sensors are treated as unbiased, so each instrument
    % contributes one bounded noise term and neither observer carries a
    % bias state.
    cfg.measurement.imu.noiseMaximum = 0.05;         % m/s^2 nbar_a
    cfg.measurement.gyroscope.noiseMaximum = 0.002;  % rad/s nbar_g
    cfg.measurement.radar.noiseMaximum = 0.12;       % m nbar_R
end
