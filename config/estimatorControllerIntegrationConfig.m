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
    cfg.sensor.radar.rangeMaximum = 30.0;             % m
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
    % One radar position cannot observe target velocity. On the first
    % detection, publish a clearly identified provisional state using an
    % oncoming line-of-sight direction and the declared speed prior; the
    % continuous-gain output-predictor observer then corrects that prior.
    cfg.initialization.minimumRadarSamples = 1;
    % This default supplies only the provisional speed magnitude; scenario
    % drivers replace it with their declared target speed. It is not
    % presented as a radar velocity measurement.
    cfg.initialization.targetSpeedPrior = 15.0;       % m/s
    observer = nrmmTrackingConfig();
    % The scalar gain SDPs use sample time to bound the one-sample
    % correction, and the predictor-reset realization also depends on it.
    % The 80 Hz observer rate keeps the target innovation well resolved
    % during the aggressive avoidance maneuvers. The controller remains at
    % 10 Hz; its adapter runs observer samples between controller instants.
    observer.runtime.samplePeriod = 0.0125;           % s
    observer.runtime.integrationStepMaximum = 0.0025; % s

    % Ego operating domain covering the deliberately aggressive straight
    % and R=60 m avoidance maneuvers, not merely centerline cornering.
    observer.ego.domain.speedMaximum = 25.0;          % m/s
    observer.ego.domain.yawRateMaximum = 0.75;        % rad/s
    % The measured circular probe reached 0.0792 rad before the
    % controller's current QP failure; retain explicit inversion-domain
    % margin. The rear-axle distance is replaced by the loaded plant value
    % in runCenterlineCruiseScenario. The mismatch value is a declared
    % research-model bound, not an empirical PassVeh14DOF certification.
    observer.ego.yaw.rearAxleDistance = 1.45;         % m
    observer.ego.yaw.sideslipDomainMaximum = 0.12;    % rad
    observer.ego.yaw.singleTrackYawRateMismatchMaximum = 0.10; % rad/s

    % Target operating domain for the avoidance suites: straight-driving
    % targets between 10 and 15 m/s. The certified speed floor no longer
    % aborts a run - the Lipschitz extension keeps the observer defined
    % and the audit merely records transient dips during the ego's dodge.
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

    % Error radii published to the controller with every estimate, which
    % is what lets its collision and road rows be tightened. Without them
    % the controller receives six numbers indistinguishable from a truth
    % state and plans as if estimation were exact.
    %
    % These controller tightening values remain explicit engineering
    % assumptions from the scenario configuration. The observer design
    % separately provides continuous-time Lyapunov bounds, which are not
    % certified sampled inertial-frame controller error bounds.
    % A certified closed-loop integration requires propagating ego position,
    % yaw, and target correlations into that controller's uncertainty model.
    cfg.publishedErrorBound.egoPosition = 0.08;       % m
    cfg.publishedErrorBound.egoVelocity = 0.30;       % m/s
    cfg.publishedErrorBound.egoYaw = 0.03;            % rad
    cfg.publishedErrorBound.egoYawRate = 0.05;        % rad/s
    % Retained nominal planner margins, not certificates of the sampled high-gain
    % estimator. The adapter labels their source explicitly and publishes the
    % unfiltered NRMM position, velocity, and acceleration alongside them.
    cfg.publishedErrorBound.targetPosition = 0.15;    % m
    cfg.publishedErrorBound.targetVelocity = 0.32;    % m/s
    cfg.publishedErrorBound.targetYaw = 0.10;         % rad
    cfg.publishedErrorBound.targetYawRate = 0.10;     % rad/s

    % Controller vehicle parameters are copied from the loaded PassVeh14DOF
    % plant by the scenario drivers; this configuration deliberately holds
    % no duplicate vehicle geometry. Only the scenario target speed lives
    % here, as the default prior magnitude for the adapter's provisional
    % first-detection target state.
    cfg.vehicle.targetSpeed = 15.0;                  % m/s
end
