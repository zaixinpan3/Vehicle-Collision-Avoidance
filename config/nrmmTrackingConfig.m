function cfg = nrmmTrackingConfig()
% nrmmTrackingConfig Parameters for the cascaded measured-input NRMM observer.
%
% The observer is the cascade
%
%   GNSS/IMU/gyro -> ego observer -> NRMM target observer,
%
% consisting of a first-order GNSS acceleration filter, certified
% set-membership fusion of two yaw pseudo-headings, a single-bandwidth yaw
% observer driven by the measured yaw rate, a body-velocity observer, a
% GNSS position observer, and a high-gain target observer in the
% transformed NRMM coordinates [rho; q; s]. Inertial-sensor biases are
% neglected, so no observer carries a bias state and the cascade is
% strictly feedforward. The configuration declares only physical operating
% domains, deterministic sensor bounds, and realization timing.
% synthesizeNrmmObserverGains solves every filter/observer gain and returns
% the corresponding optimization and stability certificates.

    % Implementation-layer settings. The synthesized gains and certificates
    % are continuous-time and do not depend on these; the settings exist to
    % realize the designed observer faithfully. Measured on the noise-free
    % constant-velocity probe, the predictor-reset realization reproduces
    % the continuous-time design to below a micrometre at
    % omega_T*samplePeriod <= 0.75, degrades to centimetres near 0.9,
    % reaches metres at 1.05 and diverges above about 1.2. The synthesized
    % bandwidth is therefore reported with this product so the realization
    % can be checked independently of the continuous-time design.
    cfg.runtime.samplePeriod = 0.02;                 % s
    cfg.runtime.integrationStepMaximum = 0.005;      % s

    %% Ego operating domain (Assumption 1)
    % The positive lower speed makes the corrected GNSS-course channel
    % uniformly informative. It is a physical operating-domain assumption,
    % not a yaw-gain tuning threshold.
    cfg.ego.domain.speedMinimum = 5.0;               % m/s   V_{E,min}
    cfg.ego.domain.speedMaximum = 20.0;              % m/s   Vbar_E
    cfg.ego.domain.yawRateMaximum = 0.30;            % rad/s omegabar_E

    %% Certified yaw measurement (Secs. 7-8)
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

    %% Target operating domain (Assumption 2; Secs. 12-17)
    cfg.target.domain.speedMinimum = 10.0;           % m/s V_{C,min}
    cfg.target.domain.speedMaximum = 20.0;           % m/s Vbar_C
    cfg.target.domain.scalarAccelerationMaximum = 2.0; % m/s^2 Abar_C
    cfg.target.domain.sideslipMaximum = 0.015;       % rad beta_max
    cfg.target.domain.rearAxleDistance = 1.6;        % m l_r
    cfg.target.domain.relativePositionMaximum = 50.0; % m rhobar
    %% Deterministic sensor error bounds (Sec. 4)
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
