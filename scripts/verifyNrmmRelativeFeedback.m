function result = verifyNrmmRelativeFeedback(outputFile, horizonSteps)
%verifyNrmmRelativeFeedback Actual NRMM feedback in two target-speed worlds.
% Common initial software state and radar position; future target and ego
% positions separate while their relative separation is regulated. This
% straight-following probe does not certify arbitrary target maneuvers.
    arguments
        outputFile (1,1) string = ""
        horizonSteps (1,1) double {mustBeInteger,mustBePositive} = 512
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'scripts'),fullfile(root,'controller'), ...
        fullfile(root,'config'),fullfile(root,'estimator'));
    cfg = collisionAvoidanceControllerConfig();reference = nonlinearBicycleModel.cruise(cfg,0);
    estimator = estimatorConfigurationFromController(estimatorControllerIntegrationConfig(), ...
        cfg,collisionThreatContract(WindowSeconds=8));
    design = synthesizeNrmmObserverGains(estimator.observer);distance = 8;
    initial = struct('initialTime',0,'egoInitialPosition',[0;0],'egoInitialYaw',0, ...
        'egoInitialBodyVelocity',[15;0],'targetInitialState',[distance;0;15;0;0;0], ...
        'targetIdentifier','relative-probe');
    rootRuntime = onlineNrmmTrackingRuntime('initialize',estimator.observer,initial,design);
    % A common 0.25 s radar history is consistent with both +/-0.1 m/s
    % worlds: its required position noise is at most 0.025 m (<0.04 m).
    for index = 1:20
        time = rootRuntime.currentTime;x = reference.state;x(1) = 15*time;
        frame = localFrame(x,reference.input,cfg,time,[distance+15*time;0],zeros(9,1));
        rootRuntime = onlineNrmmTrackingRuntime('sample',rootRuntime,frame);
    end
    [~,a,b] = nonlinearBicycleModel.sample(reference.state,reference.input,cfg);
    gain = -dlqr(a,b,diag([1 4 20 1 1 2]),diag([20 1]));
    tire = modifiedFialaTire.parameters(cfg);shape = [cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    offset = [-.1,.1];finalEgo = zeros(1,2);finalTarget = zeros(1,2);
    peak = zeros(1,2);firstInputs = zeros(2,2);maximumInput = zeros(2,1);
    clearance = Inf;targetPositionError = 0;targetSpeedError = 0;
    unavailable = 0;uncontained = 0;rootSet = struct();
    for world = 1:2
        runtime = rootRuntime;start = runtime.currentTime;
        x = reference.state;x(1) = 15*start;trueSpeed = 15+offset(world);
        for index = 1:horizonSteps
            time = runtime.currentTime;target = [distance+15*start+trueSpeed*(time-start);0];
            noise = localNoise(estimator,4*(index-1)+1);
            if index==1,noise(:) = 0;end
            frame = localFrame(x,reference.input,cfg,time,target,noise);
            output = onlineNrmmTrackingRuntime('output',runtime,frame);
            tracked = output.targetEstimate;
            if world==1 && index==1,rootSet = tracked.predictionErrorSet;end
            estimatedSpeed = tracked.targetSpeed;
            estimatedRelative = output.egoPositionInertial-tracked.targetPositionInertial;
            deviation = [estimatedRelative+[distance;0];output.egoYaw; ...
                output.egoBodyVelocity(1)-estimatedSpeed;output.egoBodyVelocity(2);output.egoYawRate];
            trim = [0;nonlinearBicycleModel.roadLoad(estimatedSpeed,cfg)/sum(tire.longitudinalForceScale)];
            input = trim+gain*deviation;
            assert(abs(input(2))<=1,'Relative policy leaves the braking input domain.');
            maximumInput = max(maximumInput,abs(input));
            if index==1,firstInputs(:,world) = input;end
            targetPositionError = max(targetPositionError,norm(target-tracked.targetPositionInertial));
            targetSpeedError = max(targetSpeedError,abs(trueSpeed-estimatedSpeed));
            prediction = tracked.predictionErrorSet;
            if prediction.available
                uncontained = uncontained+(norm(target-tracked.targetPositionInertial)>prediction.positionRadius+1e-9 ...
                    || trueSpeed<prediction.speedInterval(1)-1e-9 || trueSpeed>prediction.speedInterval(2)+1e-9);
            else,unavailable = unavailable+1;
            end
            for sensor = 1:4
                time = runtime.currentTime;target = [distance+15*start+trueSpeed*(time-start);0];
                noise = localNoise(estimator,4*(index-1)+sensor);
                if index==1 && sensor==1,noise(:) = 0;end
                frame = localFrame(x,input,cfg,time,target,noise);
                runtime = onlineNrmmTrackingRuntime('sample',runtime,frame);
                x = nonlinearBicycleModel.sample(x,input,cfg,[],runtime.samplePeriod);
                target = [distance+15*start+trueSpeed*(runtime.currentTime-start);0];
                peak(world) = max(peak(world),norm(x(1:2)-target+[distance;0]));
                clearance = min(clearance,predictiveSafetyGeometry.rectangle(x(1:3),shape,[target;0],shape));
            end
        end
        finalEgo(world) = x(1);finalTarget(world) = target(1);
    end
    result = struct('scope',"twoStraightConstantSpeedWorldsWithActualNrmmRelativeFeedback", ...
        'nrmmObserverExecuted',true,'nonlinearRobustCertificate',false, ...
        'targetSpeedOffsetsMetersPerSecond',offset,'separationMeters',distance, ...
        'horizonSeconds',horizonSteps*cfg.controller.sampleTime,'firstInputs',firstInputs, ...
        'rootTargetPredictionSet',rootSet,'peakRelativeTrackingErrorMeters',peak, ...
        'targetPositionDiameterMeters',diff(finalTarget),'egoPositionDiameterMeters',diff(finalEgo), ...
        'terminalRelativePositionDiameterMeters',abs(diff(finalEgo-finalTarget)), ...
        'independentMarginalLongitudinalGapMeters',min(finalTarget)-max(finalEgo)-cfg.vehicle.length, ...
        'minimumSampledRectangleClearanceMeters',clearance, ...
        'maximumAbsoluteInput',maximumInput,'peakTargetPositionEstimationErrorMeters',targetPositionError, ...
        'peakTargetSpeedEstimationErrorMetersPerSecond',targetSpeedError, ...
        'unavailableTargetBoundSamples',unavailable,'uncontainedAvailableTargetBoundSamples',uncontained);
    fprintf('RELATIVE NRMM: absolute spread %.4f/%.4f m, relative spread %.5f m, peak %.4f m\n', ...
        result.targetPositionDiameterMeters,result.egoPositionDiameterMeters, ...
        result.terminalRelativePositionDiameterMeters,max(peak));
    if strlength(outputFile)>0
        file = fopen(outputFile,'w');assert(file>=0,'Cannot open result file.');
        cleanup = onCleanup(@()fclose(file));
        fprintf(file,'%s\n',jsonencode(result,PrettyPrint=true));
    end
end

function noise = localNoise(estimator,index)
    phase = .37*index;noise = zeros(9,1);
    amplitude = [estimator.sensor.gnss.positionNoiseMaximum; ...
        0;estimator.sensor.imu.accelerationNoiseMaximum];
    for channel = 1:3
        noise(2*channel-1:2*channel) = amplitude(channel)*[cos(channel*phase);sin(channel*phase)];
    end
    noise(7) = estimator.sensor.gyroscope.yawRateNoiseMaximum*sin(phase);
    noise(8:9) = estimator.sensor.radar.positionNoiseMaximum*[cos(phase);sin(phase)];
end

function frame = localFrame(x,input,cfg,time,target,noise)
    rotation = [cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
    position = x(1:2)+noise(1:2);velocity = rotation*x(4:5)+noise(3:4);
    derivative = nonlinearBicycleModel.derivative(x,input,cfg);
    acceleration = [derivative(4)-x(6)*x(5);derivative(5)+x(6)*x(4)]+noise(5:6);
    relative = rotation'*(target-x(1:2))+noise(8:9);
    frame = struct('time',time,'xGps',position(1),'yGps',position(2), ...
        'vxGps',velocity(1),'vyGps',velocity(2), ...
        'longitudinalAcceleration',acceleration(1),'lateralAcceleration',acceleration(2), ...
        'yawRateMeasured',x(6)+noise(7),'radarRelativePosition',relative, ...
        'radarDetectionAvailable',true,'heldSteeringAngle',input(1),'heldBrakingRatio',input(2));
end
