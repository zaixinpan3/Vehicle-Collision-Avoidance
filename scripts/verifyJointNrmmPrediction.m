function results = verifyJointNrmmPrediction(outputFile, options)
%verifyJointNrmmPrediction Probe feedback tubes with the actual NRMM runtime.
% The local joint model retains six physical ego states and five persistent
% ego observer states. GNSS predictors reset at each of four sensor samples;
% the target subsystem is executed but does not drive this ego-only policy.
% Numerical Jacobians and nonlinear replays are research evidence, not a
% uniform nonlinear invariant-set or recursive-feasibility certificate.
    arguments
        outputFile (1,1) string = ""
        options.Speeds (1,:) double = [8 15]
        options.HorizonSteps (1,1) double {mustBeInteger,mustBePositive} = 512
        options.WorldCount (1,1) double {mustBeInteger,mustBePositive} = 3
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'scripts'),fullfile(root,'controller'), ...
        fullfile(root,'config'),fullfile(root,'estimator'));
    cases = struct([]);
    for speed = options.Speeds
        cfg = collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));
        reference = nonlinearBicycleModel.cruise(cfg,0);
        estimator = estimatorConfigurationFromController( ...
            estimatorControllerIntegrationConfig(),cfg,collisionThreatContract(WindowSeconds=8));
        design = synthesizeNrmmObserverGains(estimator.observer);
        initial = struct('initialTime',0,'egoInitialPosition',[0;0], ...
            'egoInitialYaw',0,'egoInitialBodyVelocity',[speed;0], ...
            'targetInitialState',[20;0;speed;0;0;0],'targetIdentifier','joint-probe');
        runtime = onlineNrmmTrackingRuntime('initialize',estimator.observer,initial,design);
        [~,a,b] = nonlinearBicycleModel.sample(reference.state,reference.input,cfg);
        gain = -dlqr(a,b,diag([1 4 20 1 1 2]),diag([20 1]));
        % Warm the real point and bound updates without changing their priors.
        for index = 1:10
            x = localNominal(runtime.currentTime,reference);
            [~,runtime] = localHold(x,runtime,zeros(7,4),cfg,reference,gain,reference.input);
        end
        nominal = localNominal(runtime.currentTime,reference);
        frame = localFrame(nominal,reference.input,cfg,runtime.currentTime,zeros(7,1),0);
        published = onlineNrmmTrackingRuntime('output',runtime,frame);
        assert(published.controllerErrorBound.available);
        generator = published.controllerErrorBound.generator;
        amplitude = [repmat(estimator.sensor.gnss.positionNoiseMaximum,2,1); ...
            repmat(estimator.sensor.gnss.velocityNoiseMaximum,2,1); ...
            repmat(estimator.sensor.imu.accelerationNoiseMaximum,2,1); ...
            estimator.sensor.gyroscope.yawRateNoiseMaximum];
        % Steps exceed the lateral inverse's 1e-5 m/s bisection resolution.
        % Differentiating below that resolution produces spurious matrices.
        [joint,noise,defect] = localLinearize(runtime,cfg,reference,gain,amplitude,.1);
        [jointCheck,noiseCheck] = localLinearize(runtime,cfg,reference,gain,amplitude,.05);
        convergence = max(norm(joint-jointCheck,'fro')/max(1,norm(joint,'fro')), ...
            norm(noise-noiseCheck,'fro')/max(1,norm(noise,'fro')));
        % The software state is known at the root, with its small deterministic
        % nominal offset retained as a singleton enclosed by a symmetric column.
        initialAtSample = [generator,zeros(6,1);zeros(5,8), ...
            localObserverPoint(runtime)-nominal(1:5)];
        % The current input is common to the initial information branches.
        % Include its first-hold image before using future feedback images.
        fixedInput = reference.input+gain*[localObserverPoint(runtime)-nominal(1:5);0];
        [rootMatrix,rootNoise,rootDefect] = localLinearize(runtime,cfg,reference,gain,amplitude,.1,fixedInput);
        initialGenerator = [initialAtSample,rootMatrix*initialAtSample,rootNoise,rootDefect];
        inputProjection = gain*[zeros(5,6),eye(5);zeros(1,5),1,zeros(1,5)];
        directNoise = gain(:,6)*amplitude(7);
        tube = localInvariantTube(joint,[noise,defect],initialGenerator, ...
            inputProjection,directNoise);
        independent = localInvariantTube(a+b*gain,b*gain*generator,generator,gain,gain*generator);
        times = unique([0 1 3 5 options.HorizonSteps*cfg.controller.sampleTime]);
        open = zeros(6,numel(times));
        for index = 1:numel(times)
            open(:,index) = sum(abs(a^round(times(index)/cfg.controller.sampleTime)*generator),2);
        end
        replay = localReplay(runtime,cfg,reference,gain,estimator,options.HorizonSteps,options.WorldCount);
        row = struct('speedMetersPerSecond',speed,'currentObserverRadius',published.controllerStateErrorBound, ...
            'currentObserverGenerator',generator,'feedbackGain',gain, ...
            'jointSpectralRadius',max(abs(eig(joint))),'jointTransition',joint, ...
            'sensorNoiseMap',noise,'nominalHoldDefect',defect, ...
            'finiteDifferenceRelativeChange',convergence, ...
            'jointLinearTube',tube,'independentObserverErrorLinearTube',independent, ...
            'timeSeconds',times,'openLoopInitialRadius',open,'replay',replay, ...
            'maximumAbsoluteInputFromJointLinearTube',abs(reference.input)+tube.inputRadius);
        if isempty(cases),cases = row;else,cases(end+1) = row;end %#ok<AGROW>
        fprintf('JOINT speed %.0f: spectral %.6f, fixed position radii %.4f/%.4f m, replay %.4f/%.4f m\n', ...
            speed,row.jointSpectralRadius,tube.coordinateRadius(1:2),replay.closedLoopPeak(1:2));
    end
    results = struct('scope',"actualNrmmEgoUpdateLocalLinearInvariantTubeAndNonlinearReplays", ...
        'nrmmObserverExecuted',true,'productionControllerChanged',false, ...
        'nonlinearRobustCertificate',false,'roadBoundariesIncluded',false, ...
        'sampleTimeSeconds',cfg.controller.sampleTime, ...
        'observerSampleTimeSeconds',runtime.samplePeriod,'horizonSteps',options.HorizonSteps, ...
        'worldCount',options.WorldCount,'cases',cases);
    if strlength(outputFile)>0
        file = fopen(outputFile,'w');assert(file>=0,'Cannot open result file.');
        cleanup = onCleanup(@()fclose(file));
        fprintf(file,'%s\n',jsonencode(results,PrettyPrint=true));
    end
end

function [matrix,noise,defect] = localLinearize(runtime,cfg,reference,gain,amplitude,fraction,fixedInput)
    if nargin<7,fixedInput = [];end
    scale = [.04;.04;.005;.05;.05;.0015;.04;.04;.005;.05;.05];
    matrix = zeros(11);noise = zeros(11,28);
    defect = localMap(zeros(11,1),zeros(7,4),runtime,cfg,reference,gain,fixedInput);
    for index = 1:11
        delta = zeros(11,1);step = fraction*scale(index);delta(index) = step;
        matrix(:,index) = (localMap(delta,zeros(7,4),runtime,cfg,reference,gain,fixedInput) ...
            -localMap(-delta,zeros(7,4),runtime,cfg,reference,gain,fixedInput))/(2*step);
    end
    for index = 1:28
        delta = zeros(7,4);delta(index) = fraction*amplitude(mod(index-1,7)+1);
        noise(:,index) = (localMap(zeros(11,1),delta,runtime,cfg,reference,gain,fixedInput) ...
            -localMap(zeros(11,1),-delta,runtime,cfg,reference,gain,fixedInput))/(2*fraction);
    end
end

function state = localMap(state,noise,runtime,cfg,reference,gain,fixedInput)
    nominal = localNominal(runtime.currentTime,reference);
    x = nominal+state(1:6);point = nominal(1:5)+state(7:11);
    runtime.positionEstimate = point(1:2);runtime.yawEstimate = point(3);
    runtime.bodyVelocityEstimate = point(4:5);
    [x,runtime] = localHold(x,runtime,noise,cfg,reference,gain,fixedInput);
    nominal = localNominal(runtime.currentTime,reference);
    state = [x-nominal;localObserverPoint(runtime)-nominal(1:5)];
end

function [x,runtime,input] = localHold(x,runtime,noise,cfg,reference,gain,fixedInput,radarNoise)
    if nargin<7,fixedInput = [];end
    if nargin<8,radarNoise = zeros(2,4);end
    nominal = localNominal(runtime.currentTime,reference);
    estimate = [localObserverPoint(runtime);x(6)+noise(7,1)];
    input = reference.input+gain*(estimate-nominal);
    if ~isempty(fixedInput),input = fixedInput;end
    % Directly execute the declared policy; no saturation or post-solve edits.
    assert(abs(input(2))<=1,'Joint policy leaves the braking input domain.');
    for index = 1:4
        frame = localFrame(x,input,cfg,runtime.currentTime,noise(:,index),radarNoise(:,index));
        runtime = onlineNrmmTrackingRuntime('sample',runtime,frame);
        x = nonlinearBicycleModel.sample(x,input,cfg,[],runtime.samplePeriod);
    end
end

function frame = localFrame(x,input,cfg,time,noise,radarNoise)
    rotation = [cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
    position = x(1:2)+noise(1:2);velocity = rotation*x(4:5)+noise(3:4);
    derivative = nonlinearBicycleModel.derivative(x,input,cfg);
    acceleration = [derivative(4)-x(6)*x(5);derivative(5)+x(6)*x(4)]+noise(5:6);
    targetPosition = [20+cfg.referenceSpeed*time;0];
    relative = rotation'*(targetPosition-x(1:2))+radarNoise;
    frame = struct('time',time,'xGps',position(1),'yGps',position(2), ...
        'vxGps',velocity(1),'vyGps',velocity(2), ...
        'longitudinalAcceleration',acceleration(1),'lateralAcceleration',acceleration(2), ...
        'yawRateMeasured',x(6)+noise(7),'radarRelativePosition',relative, ...
        'radarDetectionAvailable',true,'heldSteeringAngle',input(1),'heldBrakingRatio',input(2));
end

function point = localObserverPoint(runtime)
    point = [runtime.positionEstimate;runtime.yawEstimate;runtime.bodyVelocityEstimate];
end

function nominal = localNominal(time,reference)
    nominal = reference.state;nominal(1) = nominal(1)+nominal(4)*time;
end

function tube = localInvariantTube(matrix,noise,initial,inputProjection,directNoise)
    dimension = size(matrix,1);metric = dlyap(matrix',eye(dimension));factor = chol(metric);
    contraction = norm(factor*matrix/factor,2);assert(contraction<1);
    count = 512;power = eye(dimension);coordinate = zeros(dimension,1);
    input = zeros(2,1);initialCoordinate = zeros(dimension,1);initialInput = zeros(2,1);
    for index = 1:count
        coordinate = coordinate+sum(abs(power*noise),2);
        input = input+sum(abs(inputProjection*power*noise),2);
        initialCoordinate = max(initialCoordinate,sum(abs(power*initial),2));
        initialInput = max(initialInput,sum(abs(inputProjection*power*initial),2));
        power = matrix*power;
    end
    % Triangle bounds avoid enumerating 2^28 sensor-box corners. They are
    % valid for the linear model; no outward-rounded arithmetic is used.
    noiseTail = sum(vecnorm(factor*power*noise))/(1-contraction);
    initialTail = sum(vecnorm(factor*power*initial));
    inverseMetric = metric\eye(dimension);shape = sqrt(diag(inverseMetric));
    inputShape = sqrt(diag(inputProjection*inverseMetric*inputProjection'));
    tube = struct('contraction',contraction,'seriesTerms',count, ...
        'coordinateRadius',coordinate+noiseTail*shape+max(initialCoordinate,initialTail*shape), ...
        'inputRadius',input+noiseTail*inputShape+max(initialInput,initialTail*inputShape) ...
            +sum(abs(directNoise),2),'noiseTailMetricRadius',noiseTail, ...
        'initialTailMetricRadius',initialTail,'nonlinearCertificate',false, ...
        'coordinateBoxItselfAssertedInvariant',false);
end

function replay = localReplay(rootRuntime,cfg,reference,gain,estimator,count,worldCount)
    closedPeak = zeros(6,1);openPeak = zeros(6,1);observerPeak = zeros(6,1);
    maximumInput = zeros(2,1);boundMaximum = zeros(6,1);boundMinimum = inf(6,1);
    unavailable = 0;uncontained = 0;sampleCount = 0;firstInputs = zeros(2,worldCount);
    traceTimes = unique([0,1,3,5,count*cfg.controller.sampleTime]);
    traceTimes = traceTimes(traceTimes<=count*cfg.controller.sampleTime+1e-9);
    closedTrace = zeros(6,numel(traceTimes));openTrace = closedTrace;
    amplitude = [estimator.sensor.gnss.positionNoiseMaximum; ...
        estimator.sensor.gnss.velocityNoiseMaximum;estimator.sensor.imu.accelerationNoiseMaximum];
    for world = 1:worldCount
        for mode = 1:2
            runtime = rootRuntime;start = runtime.currentTime;
            x = localNominal(start,reference)+[.02*(-1)^world;.02*(-1)^floor(world/2);0;.02*(-1)^world;0;0];
            deviation = abs(x-localNominal(start,reference));
            if mode==1
                closedPeak = max(closedPeak,deviation);closedTrace(:,1) = max(closedTrace(:,1),deviation);
            else
                openPeak = max(openPeak,deviation);openTrace(:,1) = max(openTrace(:,1),deviation);
            end
            for index = 1:count
                noise = zeros(7,4);radar = zeros(2,4);
                for sensor = 1:4
                    phase = .37*(4*(index-1)+sensor)+world;
                    if world==1,phase = .7;end % persistent admissible bias
                    for channel = 1:3
                        angle = phase*channel;
                        noise(2*channel-1:2*channel,sensor) = amplitude(channel)*[cos(angle);sin(angle)];
                    end
                    noise(7,sensor) = estimator.sensor.gyroscope.yawRateNoiseMaximum*sin(phase);
                    radar(:,sensor) = estimator.sensor.radar.positionNoiseMaximum*[cos(phase);sin(phase)];
                end
                if index==1,noise(7,1) = 0;end % same known current gyro/input
                frame = localFrame(x,reference.input,cfg,runtime.currentTime,noise(:,1),radar(:,1));
                output = onlineNrmmTrackingRuntime('output',runtime,frame);
                estimate = [output.egoPositionInertial;output.egoYaw;output.egoBodyVelocity;output.egoYawRate];
                estimationError = abs(x-estimate);estimationError(3) = abs(atan2(sin(x(3)-estimate(3)),cos(x(3)-estimate(3))));
                if mode==1
                    sampleCount = sampleCount+1;observerPeak = max(observerPeak,estimationError);
                    if output.controllerErrorBound.available
                        boundMaximum = max(boundMaximum,output.controllerStateErrorBound);
                        boundMinimum = min(boundMinimum,output.controllerStateErrorBound);
                        uncontained = uncontained+any(estimationError>output.controllerStateErrorBound+1e-9);
                    else,unavailable = unavailable+1;
                    end
                end
                fixed = [];if mode==2,fixed = reference.input;end
                [x,runtime,input] = localHold(x,runtime,noise,cfg,reference,gain,fixed,radar);
                deviation = abs(x-localNominal(runtime.currentTime,reference));
                at = find(abs(traceTimes-(runtime.currentTime-start))<1e-8);
                if mode==1
                    closedPeak = max(closedPeak,deviation);maximumInput = max(maximumInput,abs(input));
                    if index==1,firstInputs(:,world) = input;end
                    closedTrace(:,at) = max(closedTrace(:,at),deviation);
                else
                    openPeak = max(openPeak,deviation);openTrace(:,at) = max(openTrace(:,at),deviation);
                end
            end
        end
    end
    replay = struct('noisePatterns',"oneConstantBiasAndDeterministicRotatingNoiseBalls", ...
        'closedLoopPeak',closedPeak,'openLoopPeak',openPeak,'observerErrorPeak',observerPeak, ...
        'maximumAbsoluteInput',maximumInput,'firstInputs',firstInputs, ...
        'observerBoundMinimum',boundMinimum,'observerBoundMaximum',boundMaximum, ...
        'auditedControllerSamples',sampleCount,'unavailableObserverBoundSamples',unavailable, ...
        'uncontainedObserverBoundSamples',uncontained,'timeSeconds',traceTimes, ...
        'closedLoopTrace',closedTrace,'openLoopTrace',openTrace,'nonlinearCertificate',false);
end
