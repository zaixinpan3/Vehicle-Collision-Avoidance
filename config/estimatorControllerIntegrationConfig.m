function cfg = estimatorControllerIntegrationConfig()
% estimatorControllerIntegrationConfig Sensor and observer test settings.
%
% The synthetic sensor contract contains ego GNSS position and velocity,
% ego center-of-mass body acceleration, ego gyroscope yaw rate, and radar
% relative position. Vector noise maxima are Euclidean norm bounds; the
% gyroscope maximum is an absolute scalar bound.

    cfg.randomSeed = 20260728;
    cfg.noiseModel = "boundedUniform";

    cfg.sensor.gnss.positionNoiseMaximum = 0.04;       % m
    cfg.sensor.gnss.velocityNoiseMaximum = 0.05;       % m/s
    cfg.sensor.imu.accelerationNoiseMaximum = 0.03;   % m/s^2
    cfg.sensor.gyroscope.yawRateNoiseMaximum = 0.0015; % rad/s
    cfg.sensor.radar.positionNoiseMaximum = 0.04;     % m
    cfg.sensor.radar.rangeMaximum = 50.0;             % m, reference-position radius
    % The synthetic radar has complete azimuth coverage and no missed
    % detections inside this range. The estimator may coast an acquired
    % target internally for a bounded interval, but it publishes a target
    % state to the controller only while the current radar sample reports
    % that target. A track is retired after the coast timeout and must be
    % reacquired from radar before it can be published again.
    cfg.sensor.radar.targetPublicationPolicy = ...
        "currentRadarVisibilityRequiredInternalTrackMayCoast";
    cfg.sensor.radar.maximumTrackCoastDuration = 0.50; % s
    % The synthetic IMU reports ego-body-frame acceleration at the vehicle
    % center of mass, gravity-compensated.

    cfg.initialization.historyDuration = 0.50;        % s
    % One radar position cannot observe target velocity. A target is
    % acquired after this many consecutive detections (0.25 s at 80 Hz),
    % whose least-squares line initializes its velocity. A single detection
    % with the oncoming line-of-sight speed prior (15 m/s) started a
    % re-acquired 8-m/s target 7 m/s off; the tracker's transient then moved
    % the published forecast across the whole target contract and left the
    % controller without a solution.
    cfg.initialization.minimumRadarSamples = 20;
    % A detection window spanning at least this duration also initializes
    % the target acceleration (constant-acceleration least squares); a
    % shorter window initializes it to zero.
    cfg.initialization.accelerationFitMinimumDuration = 0.50; % s
    % This default supplies only the provisional speed magnitude; scenario
    % drivers replace it with their declared target speed. It is not
    % presented as a radar velocity measurement.
    cfg.initialization.targetSpeedPrior = 15.0;       % m/s
    observer = nrmmTrackingConfig();
    % The scalar gain SDPs use sample time to bound the one-sample
    % correction, and the predictor-reset realization also depends on it.
    % The 80 Hz observer rate keeps the target innovation well resolved
    % during the avoidance maneuvers. The adapter runs observer samples
    % between the separately configured controller instants.
    observer.runtime.samplePeriod = 0.0125;           % s
    observer.runtime.integrationStepMaximum = 0.0025; % s

    % Standalone defaults of the ego domain and of the kinematic
    % single-track branch. The closed-loop drivers replace the ego domain,
    % the rear-axle distance and the sideslip cone by the controller's values
    % and add its vehicle model (estimatorConfigurationFromController); the
    % lateral velocity is then measured by the force balance, and the
    % mismatch allowance below is used only where no held input exists (the
    % initialization history). The 0.25 allowance is a declared premise of
    % the kinematic branch, not a proven vehicle-model bound: avoidance
    % maneuvers at 15 m/s exceed it (report/ESTIMATOR_DOMAIN_20261005.tex).
    observer.ego.domain.speedMaximum = 25.0;          % m/s
    observer.ego.domain.yawRateMaximum = 0.75;        % rad/s
    observer.ego.yaw.rearAxleDistance = 1.45;         % m
    observer.ego.yaw.sideslipDomainMaximum = 0.12;    % rad
    observer.ego.yaw.singleTrackYawRateMismatchMaximum = 0.25; % rad/s

    % Standalone target operating domain. The closed-loop drivers replace
    % it by the hull of the scenario targets (scripts/collisionThreatContract).
    % The certified speed floor does not abort a run: the Lipschitz
    % extension keeps the observer defined, and the audit records dips.
    observer.target.domain.speedMinimum = 5.0;        % m/s
    observer.target.domain.speedMaximum = 20.0;       % m/s
    observer.target.domain.scalarAccelerationMaximum = 2.0; % m/s^2
    observer.target.domain.sideslipMaximum = 0.005;   % rad
    observer.target.domain.relativePositionMaximum = 50.0; % m
    % The normalized target direction is recovered from an observer LMI;
    % its physical bandwidth is then solved by the normalized certified-
    % state minimax problem. The resulting omega_T*T_s is reported for the
    % sampled-realization audit rather than configured as a gain.

    observer.measurement.gps.positionNoiseMaximum = ...
        cfg.sensor.gnss.positionNoiseMaximum;
    observer.measurement.gps.velocityNoiseMaximum = ...
        cfg.sensor.gnss.velocityNoiseMaximum;
    observer.measurement.imu.noiseMaximum = ...
        cfg.sensor.imu.accelerationNoiseMaximum;
    observer.measurement.gyroscope.noiseMaximum = ...
        cfg.sensor.gyroscope.yawRateNoiseMaximum;
    observer.measurement.radar.noiseMaximum = ...
        cfg.sensor.radar.positionNoiseMaximum;
    cfg.observer = observer;

    % Controller errors come from the online timestamped observer enclosure.
    % Sensor and true-motion bounds above are its premises; they are not
    % substitutes for the evolving estimation errors. No fixed state-error
    % override is applied by the adapter.

    % Vehicle parameters come from the controller configuration
    % (estimatorConfigurationFromController); this configuration holds no
    % duplicate vehicle geometry. Only the scenario target speed lives
    % here, as the default prior magnitude for the adapter's provisional
    % first-detection target state.
    cfg.vehicle.targetSpeed = 15.0;                  % m/s
end
