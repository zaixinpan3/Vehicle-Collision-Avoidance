function varargout = nrmmEstimatorControllerAdapter(action, varargin)
% nrmmEstimatorControllerAdapter Connect synthetic sensors to the NRMM.
%
% This adapter is intentionally one-way:
%   plant/target truth -> allowed synthetic sensors -> NRMM -> controller.
% Plant truth is used only to synthesize measurements and evaluate results.

    action = lower(string(action));
    switch action
        case "initialize"
            [varargout{1}, varargout{2}] = localInitialize(varargin{:});
        case "sample"
            [varargout{1}, varargout{2}, varargout{3}, ...
                varargout{4}, varargout{5}] = localSample(varargin{:});
        otherwise
            error("nrmmEstimatorControllerAdapter:invalidAction", ...
                "action must be 'initialize' or 'sample'.");
    end
end

function [context, initialization] = localInitialize( ...
        cfg, initialEgoTruth, targetStateFunction)
    localAddProjectPaths();
    localValidateConfiguration(cfg);
    if ~isa(targetStateFunction, "function_handle")
        error("nrmmEstimatorControllerAdapter:invalidTargetFunction", ...
            "targetStateFunction must be a function handle.");
    end

    samplePeriod = cfg.observer.runtime.samplePeriod;
    historyStepCount = round( ...
        cfg.initialization.historyDuration/samplePeriod);
    durationError = abs(historyStepCount*samplePeriod ...
        - cfg.initialization.historyDuration);
    if durationError > 100.0*eps(max( ...
            1.0, cfg.initialization.historyDuration))
        error("nrmmEstimatorControllerAdapter:invalidHistoryDuration", ...
            "The initialization history must use whole observer samples.");
    end
    historyTime = (-historyStepCount:0).' * samplePeriod;
    stream = RandStream("mt19937ar", Seed=cfg.randomSeed);
    frame = cell(numel(historyTime), 1);
    audit = cell(numel(historyTime), 1);
    egoTruth = cell(numel(historyTime), 1);
    for sampleIdx = 1:numel(historyTime)
        egoTruth{sampleIdx} = localHistoricalEgoState( ...
            initialEgoTruth, historyTime(sampleIdx));
        targetTruth = targetStateFunction( ...
            historyTime(sampleIdx), egoTruth{sampleIdx});
        bodyAcceleration = localNominalBodyAcceleration( ...
            egoTruth{sampleIdx});
        [frame{sampleIdx}, audit{sampleIdx}] = localSensorFrame( ...
            cfg, stream, historyTime(sampleIdx), ...
            egoTruth{sampleIdx}, bodyAcceleration, targetTruth);
        if isfield(initialEgoTruth,'heldActuatorInput')
            % The synthetic prehistory uses this same known held command.
            frame{sampleIdx}.heldSteeringAngle=initialEgoTruth.heldActuatorInput(1);
            frame{sampleIdx}.heldBrakingRatio=initialEgoTruth.heldActuatorInput(2);
        end
    end

    observerDesign = synthesizeNrmmObserverGains(cfg.observer);
    egoInitial = localInitialEgoEstimate(frame, observerDesign, cfg);
    [targetInitialState, targetAcquired, targetAcquisition] = ...
        localInitialTargetEstimate( ...
            historyTime, frame, egoInitial.position, ...
            egoInitial.yaw, cfg, observerDesign.target.domain.accelerationNormBound);
    options = struct( ...
        "initialTime", 0.0, ...
        "egoInitialPosition", egoInitial.position, ...
        "egoInitialYaw", egoInitial.yaw, ...
        "egoInitialBodyVelocity", egoInitial.bodyVelocity, ...
        "targetInitialState", targetInitialState, ...
        "targetIdentifier", "synthetic-radar-track-1", ...
        "inertialVelocityInitialization",struct("value",egoInitial.inertialVelocity, ...
        "radius",egoInitial.inertialVelocityRadius));
    if targetAcquired
        history = struct([]);
        for sampleIdx = 1:numel(frame)
            measurement = frame{sampleIdx};
            if ~measurement.radarDetectionAvailable, continue; end
            record = struct("time",measurement.time, ...
                "gnssPosition",[measurement.xGps;measurement.yGps], ...
                "gnssVelocity",egoInitial.inertialVelocity, ...
                "velocityErrorMaximum",egoInitial.historyVelocityRadius(sampleIdx), ...
                "yawRate",measurement.yawRateMeasured, ...
                "radarRelativePosition",measurement.radarRelativePosition, ...
                "radarDetectionAvailable",true);
            if isfield(measurement,'heldSteeringAngle')
                record.heldInput=[measurement.heldSteeringAngle;measurement.heldBrakingRatio];
                record.bodyAcceleration=[measurement.longitudinalAcceleration;measurement.lateralAcceleration];
            end
            record.yawAccelerationMaximum=egoInitial.historyEvidence.yawAccelerationMaximum(sampleIdx);
            history = [history;record]; %#ok<AGROW>
        end
        options.targetMeasurementHistory = history;
    end
    runtime = onlineNrmmTrackingRuntime( ...
        "initialize", cfg.observer, options, observerDesign);
    currentFrame = frame{end};
    if isfield(initialEgoTruth,'heldActuatorInput')
        % This is known actuator history, not a truth-state measurement.
        % The first IMU sample still belongs to the command already held.
        held=initialEgoTruth.heldActuatorInput(:);
        assert(numel(held)==2 && all(isfinite(held)));
        currentFrame.heldSteeringAngle=held(1);
        currentFrame.heldBrakingRatio=held(2);
        frame{end}=currentFrame;
    end
    currentOutput = onlineNrmmTrackingRuntime( ...
        "output", runtime, currentFrame);

    context = struct( ...
        "configuration", cfg, ...
        "observerDesign", observerDesign, ...
        "runtime", runtime, ...
        "randomStream", stream, ...
        "currentOutput", currentOutput, ...
        "targetStateFunction", targetStateFunction, ...
        "targetAcquired", targetAcquired, ...
        "targetAcquisition", targetAcquisition, ...
        "targetAcquisitionTime", ...
            localInitialAcquisitionTime(targetAcquired), ...
        "targetLastDetectionTime", ...
            localInitialAcquisitionTime(targetAcquired), ...
        "targetTrackStatus", ...
            localInitialTrackStatus(targetAcquired), ...
        "targetRetirementCount", 0, ...
        "targetLastRetirementTime", NaN, ...
        "initialFramePending", true, ...
        "initialFrame", currentFrame, ...
        "initialAudit", audit{end}, ...
        "previousControllerTime", 0.0, ...
        "previousControllerEgoTruth", initialEgoTruth, ...
        "previousObserverEgoTruth", initialEgoTruth, ...
        "observerSamplesSinceLastControllerTime", 0);
    initialization = struct( ...
        "historyTime", historyTime, ...
        "sensorFrame", {frame}, ...
        "measurementAudit", {audit}, ...
        "egoInitialPosition", egoInitial.position, ...
        "egoInitialYaw", egoInitial.yaw, ...
        "egoInitialBodyVelocity", egoInitial.bodyVelocity, ...
        "inertialVelocityRadius",egoInitial.inertialVelocityRadius, ...
        "velocityInitializationEvidence",egoInitial.historyEvidence, ...
        "targetInitialState", targetInitialState, ...
        "targetInitiallyAcquired", targetAcquired, ...
        "targetAcquisition", targetAcquisition, ...
        "observerDesign", observerDesign, ...
        "observerSamplePeriod", samplePeriod);
end

function [context, egoEstimate, targetEstimate, frame, audit] = ...
        localSample(context, time, egoTruth, targetTruth, truthPath)
    % truthPath (optional) is the plant's continuous trajectory over the hold
    % that ends at this controller time: sensor-time states, instantaneous
    % body accelerations a = R(psi)'*pddot, and the input held during it.
    % With it every sensor sample is taken from the true trajectory and
    % carries the held input; without it samples between controller times
    % are interpolated as before and carry no input.
    if nargin < 5,truthPath = [];end
    context = localAdvanceObserverToControllerTime( ...
        context, time, egoTruth, truthPath);
    tolerance = 64.0*eps(max( ...
        1.0, max(abs([time; context.runtime.currentTime]))));
    if abs(time-context.runtime.currentTime) > tolerance
        error("nrmmEstimatorControllerAdapter:offSampleGrid", ...
            "The controller and estimator sample times must remain aligned.");
    end

    if context.initialFramePending
        frame = context.initialFrame;
        audit = context.initialAudit;
        context.initialFramePending = false;
    else
        [pathAcceleration,heldInput] = localPathSample(truthPath, time);
        if isempty(pathAcceleration)
            bodyAcceleration = localSampledBodyAcceleration( ...
                context.previousObserverEgoTruth, egoTruth, ...
                context.configuration.observer.runtime.samplePeriod);
        else
            bodyAcceleration = pathAcceleration;
        end
        [frame, audit] = localSensorFrame( ...
            context.configuration, context.randomStream, time, ...
            egoTruth, bodyAcceleration, targetTruth, heldInput);
    end

    observerPoint = struct("egoPositionInertial",context.runtime.positionEstimate, ...
        "egoYaw",context.runtime.yawEstimate);
    [context, targetJustAcquired] = ...
        localUpdateTargetAcquisition( ...
            context, frame, observerPoint);
    context = localUpdateTargetLifecycle( ...
        context, frame, observerPoint);
    shareMeasurement = context.targetAcquired && ~targetJustAcquired;
    if shareMeasurement
        [nextRuntime,egoEstimate] = onlineNrmmTrackingRuntime("sample",context.runtime,frame);
    else
        egoEstimate = localCurrentEgoEstimate(context.runtime,frame);
    end
    if context.targetAcquired
        rawTargetEstimate = egoEstimate.targetEstimate;
        rawTargetEstimate = localLabelTargetEstimate( ...
            rawTargetEstimate, context.targetAcquisition);
        publicationPolicy = string( ...
            context.configuration.sensor.radar ...
                .targetPublicationPolicy);
        if publicationPolicy ...
                == "currentRadarVisibilityRequiredInternalTrackMayCoast" ...
                && frame.radarDetectionAvailable
            targetEstimate = rawTargetEstimate;
        else
            targetEstimate = struct([]);
        end
    else
        rawTargetEstimate = struct([]);
        targetEstimate = struct([]);
    end
    egoEstimate.rawNrmmTargetEstimate = rawTargetEstimate;
    egoEstimate.targetEstimate = targetEstimate;
    egoEstimate.targetTrackAcquired = context.targetAcquired;
    egoEstimate.targetEstimatePublished = ~isempty(targetEstimate);
    egoEstimate.targetPublicationPolicy = string( ...
        context.configuration.sensor.radar.targetPublicationPolicy);
    egoEstimate.targetAcquisitionTime = ...
        context.targetAcquisitionTime;
    egoEstimate.targetTrackStatus = context.targetTrackStatus;
    egoEstimate.targetTrackCoastAge = ...
        localTargetTrackCoastAge(context, frame.time);
    egoEstimate.targetMaximumCoastDuration = ...
        context.configuration.sensor.radar ...
            .maximumTrackCoastDuration;
    egoEstimate.targetTrackRetirementCount = ...
        context.targetRetirementCount;
    egoEstimate.targetLastRetirementTime = ...
        context.targetLastRetirementTime;
    egoEstimate.perception = struct("time",time, ...
        "range",context.configuration.sensor.radar.rangeMaximum,"completeWithinRange",true);
    truthState = [localEgoPosition(egoTruth);localEgoYaw(egoTruth); ...
        localBodyVelocity(egoTruth);localEgoYawRate(egoTruth)];
    audit.truthEnclosure = auditNrmmTruthEnclosure( ...
        egoEstimate,truthState,targetTruth,context.configuration.observer);
    if ~context.targetAcquired
        context = localAnchorDormantTarget( ...
            context, egoEstimate);
    end
    observerFrame = frame;
    if ~context.targetAcquired || targetJustAcquired
        observerFrame = localFrameWithoutRadar(frame);
    end
    % Retain the output actually published at this controller time. The
    % internal observer still advances all states/bounds to the next sensor
    % sample, but no consumer uses a second full future-time publication.
    context.currentOutput = egoEstimate;
    if shareMeasurement
        context.runtime = nextRuntime;
    else
        context.runtime = onlineNrmmTrackingRuntime("step",context.runtime,observerFrame);
    end
    context.previousControllerTime = time;
    context.previousControllerEgoTruth = egoTruth;
    context.previousObserverEgoTruth = egoTruth;
end

function context = localAdvanceObserverToControllerTime( ...
        context, controllerTime, controllerEgoTruth, truthPath)
% localAdvanceObserverToControllerTime Process buffered sensor-rate samples.
%
% The plant and controller are evaluated at 10 Hz, whereas the configured
% output-predictor realization runs at 80 Hz. At a controller instant all
% sensor samples from the completed plant interval are causally available.
% The harness reconstructs their ego states by interpolation and evaluates
% the declared analytic target trajectory at each sensor timestamp.

    samplePeriod = ...
        context.configuration.observer.runtime.samplePeriod;
    remainingTime = controllerTime-context.runtime.currentTime;
    tolerance = 100.0*eps(max(1.0, max(abs([ ...
        controllerTime; context.runtime.currentTime; samplePeriod]))));
    if remainingTime < -tolerance
        error("nrmmEstimatorControllerAdapter:nonmonotonicControllerTime", ...
            "Controller times must increase monotonically.");
    end
    missingSampleCount = round(max(0.0, remainingTime)/samplePeriod);
    if abs(remainingTime-missingSampleCount*samplePeriod) > tolerance
        error("nrmmEstimatorControllerAdapter:offObserverSampleGrid", ...
            "Controller times must lie on the observer sample grid.");
    end

    controllerInterval = ...
        controllerTime-context.previousControllerTime;
    if missingSampleCount > 0 && controllerInterval <= 0.0
        error("nrmmEstimatorControllerAdapter:invalidControllerInterval", ...
            "A positive controller interval is required for sensor interpolation.");
    end
    for sampleIdx = 1:missingSampleCount
        sampleTime = context.runtime.currentTime;
        [pathAcceleration,heldInput,pathState] = localPathSample(truthPath, sampleTime);
        if isempty(pathAcceleration)
            fraction = (sampleTime-context.previousControllerTime) ...
                / controllerInterval;
            intermediateEgoTruth = localInterpolateEgoState( ...
                context.previousControllerEgoTruth, controllerEgoTruth, ...
                fraction);
            bodyAcceleration = localSampledBodyAcceleration( ...
                context.previousObserverEgoTruth, intermediateEgoTruth, ...
                samplePeriod);
        else
            intermediateEgoTruth = localStateStruct(pathState);
            bodyAcceleration = pathAcceleration;
        end
        intermediateTargetTruth = context.targetStateFunction( ...
            sampleTime, intermediateEgoTruth);
        [frame, ~] = localSensorFrame( ...
            context.configuration, context.randomStream, sampleTime, ...
            intermediateEgoTruth, bodyAcceleration, ...
            intermediateTargetTruth, heldInput);
        % Track initialization uses only the current observer point. Full
        % output enclosures are published at controller times; every sensor
        % update still measures and propagates the complete bound state.
        intermediateEgoEstimate = struct("egoPositionInertial",context.runtime.positionEstimate, ...
            "egoYaw",context.runtime.yawEstimate);
        [context, targetJustAcquired] = ...
            localUpdateTargetAcquisition( ...
                context, frame, intermediateEgoEstimate);
        context = localUpdateTargetLifecycle( ...
            context, frame, intermediateEgoEstimate);
        observerFrame = frame;
        if ~context.targetAcquired || targetJustAcquired
            observerFrame = localFrameWithoutRadar(frame);
        end
        context.runtime = onlineNrmmTrackingRuntime("step",context.runtime,observerFrame);
        context.previousObserverEgoTruth = intermediateEgoTruth;
    end
    context.observerSamplesSinceLastControllerTime = ...
        missingSampleCount+1;
end

function [acceleration, heldInput, state] = localPathSample(truthPath, time)
% Sample the supplied continuous trajectory at a sensor time, if present.
    acceleration = [];heldInput = [];state = [];
    if isempty(truthPath),return;end
    tolerance = 1e-9*max(1,abs(time));
    index = find(abs(truthPath.times-time) <= tolerance,1);
    if isempty(index),return;end
    acceleration = truthPath.accelerations(:,index);
    heldInput = truthPath.input(:);
    state = truthPath.states(:,index);
end

function state = localStateStruct(x)
    state = struct("position",x(1:2),"yawAngle",x(3), ...
        "longitudinalVelocity",x(4),"lateralVelocity",x(5),"yawRate",x(6));
end

function state = localInterpolateEgoState( ...
        initialState, finalState, fraction)
    if ~isscalar(fraction) || ~isfinite(fraction) ...
            || fraction < 0.0 || fraction > 1.0
        error("nrmmEstimatorControllerAdapter:invalidInterpolation", ...
            "The sensor-time interpolation fraction must be in [0, 1].");
    end
    initialYaw = localEgoYaw(initialState);
    yawDifference = atan2( ...
        sin(localEgoYaw(finalState)-initialYaw), ...
        cos(localEgoYaw(finalState)-initialYaw));
    position = (1.0-fraction)*localEgoPosition(initialState) ...
        + fraction*localEgoPosition(finalState);
    bodyVelocity = (1.0-fraction)*localBodyVelocity(initialState) ...
        + fraction*localBodyVelocity(finalState);
    yawRate = (1.0-fraction)*localEgoYawRate(initialState) ...
        + fraction*localEgoYawRate(finalState);
    state = struct( ...
        "position", position, ...
        "yawAngle", initialYaw+fraction*yawDifference, ...
        "longitudinalVelocity", bodyVelocity(1), ...
        "lateralVelocity", bodyVelocity(2), ...
        "yawRate", yawRate);
end

function estimate = localCurrentEgoEstimate(runtime, frame)
% Tighten the timestamped enclosure with the current sensor frame. This
% read-only output does not integrate the observer or advance its state.
    estimate = onlineNrmmTrackingRuntime("output", runtime, frame);
end

function estimate = localLabelTargetEstimate(estimate, acquisition)
    estimate.velocityInitializationMethod = ...
        acquisition.velocityInitializationMethod;
    estimate.source = ...
        "NRMM multistage high-gain observer";
end

function egoInitial = localInitialEgoEstimate(frame, observerDesign, cfg)
% Seed velocity from the observed displacement, with its acceleration bias.
    latestFrame=frame{end};firstFrame=frame{1};
    position=[latestFrame.xGps;latestFrame.yGps];
    duration=latestFrame.time-firstFrame.time;
    inertialVelocity=(position-[firstFrame.xGps;firstFrame.yGps])/duration;
    radius=2*observerDesign.sensors.positionNoiseMaximum/duration ...
        +cfg.observer.ego.domain.accelerationNormMaximum*duration/2;
    radius=min(radius,observerDesign.operatingDomain.egoSpeedMaximum+norm(inertialVelocity));
    % Intersect the implications of every past displacement without changing
    % the point estimate. A short interval has more position-noise error; a
    % long one has more acceleration bias. No velocity truth is used.
    for index=2:numel(frame)-1
        duration=latestFrame.time-frame{index}.time;
        secant=(position-[frame{index}.xGps;frame{index}.yGps])/duration;
        candidate=norm(secant-inertialVelocity)+2*observerDesign.sensors.positionNoiseMaximum/duration ...
            +cfg.observer.ego.domain.accelerationNormMaximum*duration/2;
        radius=min(radius,candidate);
    end
    time=cellfun(@(sample)sample.time,frame);time=time(:);
    accelerationEnvelope=repmat(cfg.observer.ego.domain.accelerationNormMaximum,numel(time)-1,1);
    for refinement=1:3
        [accelerationEnvelope,historyEvidence]=localHistoryAcceleration( ...
            frame,observerDesign,cfg,inertialVelocity,radius,accelerationEnvelope);
        previousRadius=radius;
        for index=1:numel(frame)-1
            duration=time(end)-time(index);
            secant=(position-[frame{index}.xGps;frame{index}.yGps])/duration;
            offset=time(index:end)-time(index);
            weights=diff(offset.^2)/(2*duration);
            bias=accelerationEnvelope(index:end)'*weights;
            candidate=norm(secant-inertialVelocity)+2*observerDesign.sensors.positionNoiseMaximum/duration+bias;
            radius=min(radius,candidate);
        end
        if previousRadius-radius<.01*max(radius,eps),break;end
    end
    pastDrift=flipud(cumsum(flipud(accelerationEnvelope.*diff(time))));
    historyVelocityRadius=min(radius+[pastDrift;0], ...
        observerDesign.operatingDomain.egoSpeedMaximum+norm(inertialVelocity));
    course=nrmmEgoCourseGeometry("kinematicCourse",inertialVelocity, ...
        latestFrame.yawRateMeasured,observerDesign.yaw.courseModel.rearAxleDistance, ...
        radius,observerDesign.sensors.gyroscopeNoiseMaximum, ...
        observerDesign.yaw.courseModel.singleTrackYawRateMismatchMaximum, ...
        observerDesign.yaw.courseModel.sideslipDomainMaximum);
    yaw=course.correspondence.heading;
    bodyVelocity=nrmmObserverVectorField("velocityMeasurement", ...
        inertialVelocity,latestFrame.yawRateMeasured,observerDesign);
    egoInitial=struct("position",position,"yaw",yaw,"bodyVelocity",bodyVelocity, ...
        "inertialVelocity",inertialVelocity,"inertialVelocityRadius",radius, ...
        "historyVelocityRadius",historyVelocityRadius,"historyEvidence",historyEvidence);
end

function [envelope,evidence]=localHistoryAcceleration(frame,design,cfg,velocity,radius,priorEnvelope)
    count=numel(frame)-1;maximum=cfg.observer.ego.domain.accelerationNormMaximum;
    envelope=repmat(maximum,count,1);accepted=false(count,1);
    yawMaximum=repmat(cfg.observer.ego.domain.yawAccelerationMaximum,count+1,1);
    evidence=struct('method',"physicalAccelerationDomain",'acceptedHolds',0,'totalHolds',count, ...
        'yawAccelerationMaximum',yawMaximum);
    if ~isfield(cfg.initialization,'vehicleModel') || ~isfield(design,'egoModel'),return;end
    vehicle=cfg.initialization.vehicleModel;model=design.egoModel;
    parameters=nrmmSet(zeros(4,1),model.parameterUncertainty*eye(4));
    beta=design.yaw.courseModel.sideslipDomainMaximum;
    time=cellfun(@(sample)sample.time,frame);time=time(:);duration=time(end)-time(1);
    leftWeights=diff((time-time(1)).^2)/(2*duration);
    rightWeights=-diff((time(end)-time).^2)/(2*duration);
    intervalDrift=priorEnvelope.*diff(time);
    for index=1:count
        sample=frame{index};
        if ~isfield(sample,'heldSteeringAngle'),continue;end
        dt=frame{index+1}.time-sample.time;
        sensor=design.sensors;
        % The same complete displacement also encloses velocities inside
        % its time interval. This is tighter than propagating an endpoint
        % estimate backwards with the global maximum acceleration.
        averageBias=priorEnvelope(1:index-1)'*leftWeights(1:index-1) ...
            +priorEnvelope(index:end)'*rightWeights(index:end);
        sensor.velocityNoiseMaximum=min(radius+sum(intervalDrift(index:end)), ...
            2*sensor.positionNoiseMaximum/duration+averageBias);
        input=[sample.heldSteeringAngle;sample.heldBrakingRatio];
        acceleration=[sample.longitudinalAcceleration;sample.lateralAcceleration];
        speedLower=max(0,norm(velocity)-sensor.velocityNoiseMaximum);
        speedUpper=min(design.operatingDomain.egoSpeedMaximum,norm(velocity)+sensor.velocityNoiseMaximum);
        edges=linspace(speedLower,speedUpper,max(2,1+ceil((speedUpper-speedLower)/.5)));
        largest=0;largestYaw=0;complete=true;
        for cellIndex=1:numel(edges)-1
            interval=edges(cellIndex:cellIndex+1);cellSpeed=mean(interval);
            sensor.velocityNoiseMaximum=diff(interval)/2;
            lateral=nrmmEgoCourseGeometry('lateralVelocity',[cellSpeed;0],sample.yawRateMeasured, ...
                acceleration,input,model,sensor,beta);
            % Only a proven empty measurement intersection excludes a cell.
            if ~lateral.consistent,continue;end
            lower=[zeros(3,1);interval(1)*cos(beta);lateral.lower;sample.yawRateMeasured-sensor.gyroscopeNoiseMaximum];
            upper=[zeros(3,1);interval(2);lateral.upper;sample.yawRateMeasured+sensor.gyroscopeNoiseMaximum];
            image=nrmmJointEnclosure('accelerationHold',nrmmSet.range(lower,upper),nrmmSet(input),parameters,vehicle,dt);
            if ~image.valid,complete=false;break;end
            largest=max(largest,image.normMaximum);
            largestYaw=max(largestYaw,image.yawAccelerationMaximum);
        end
        if complete && largest>0
            envelope(index)=min(priorEnvelope(index),largest);accepted(index)=true;
            yawMaximum(index)=min(yawMaximum(index),largestYaw);
        end
    end
    evidence=struct('method',"knownHoldPicardAccelerationEnvelope",'acceptedHolds',sum(accepted), ...
        'totalHolds',count,'accelerationNormMaximum',envelope,'yawAccelerationMaximum',yawMaximum, ...
        'realArithmeticEnclosure',true);
end

function [targetState, acquired, acquisition] = ...
        localInitialTargetEstimate( ...
            historyTime, frame, egoPosition, egoYaw, cfg, accelerationBound)
    detectionAvailable = cellfun( ...
        @(value) logical(value.radarDetectionAvailable), frame);
    minimumSampleCount = cfg.initialization.minimumRadarSamples;
    lastUnavailableIdx = find( ...
        ~detectionAvailable, 1, "last");
    if isempty(lastUnavailableIdx)
        trailingDetectionIdx = (1:numel(frame)).';
    else
        trailingDetectionIdx = ...
            ((lastUnavailableIdx+1):numel(frame)).';
    end

    acquisition = struct( ...
        "minimumSampleCount", minimumSampleCount, ...
        "time", historyTime(trailingDetectionIdx), ...
        "radarRelativePosition", localRadarRows( ...
            frame, trailingDetectionIdx), ...
        "targetPositionInertial", ...
            localInitialTargetPositionRows( ...
                frame, trailingDetectionIdx, egoYaw), ...
        "velocityInitializationMethod", "", ...
        "consecutiveDetectionCount", ...
            numel(trailingDetectionIdx));
    if numel(trailingDetectionIdx) >= minimumSampleCount
        % Initialization may use every consecutive detection already
        % present in the declared history window. Delayed online
        % acquisition below remains causal and triggers as soon as the
        % configured minimum count is reached.
        selectedIdx = trailingDetectionIdx;
        detectionTime = historyTime(selectedIdx);
        targetPositionInertial = ...
            acquisition.targetPositionInertial;
        [targetState, ~] = localTargetStateFromInertialWindow( ...
            detectionTime, targetPositionInertial, ...
            egoPosition, egoYaw, cfg, accelerationBound);
        acquisition.velocityInitializationMethod = ...
            localTargetVelocityInitializationMethod( ...
                numel(detectionTime), ...
                detectionTime(end)-detectionTime(1), cfg);
        acquired = true;
    else
        targetState = localDormantTargetState(cfg);
        acquired = false;
    end
end

function radarPosition = localRadarRows(frame, sampleIdx)
    if isempty(sampleIdx)
        radarPosition = zeros(0, 2);
        return;
    end
    radarPosition = cell2mat(cellfun( ...
        @(value) value.radarRelativePosition, ...
        frame(sampleIdx), UniformOutput=false));
end

function targetPosition = localInitialTargetPositionRows( ...
        frame, sampleIdx, currentYaw)
% Anchor all radar poses to ONE current estimated body yaw and transport
% it with the measured gyro history. Differentiating short GNSS position
% windows to reconstruct each past heading amplifies position noise into
% spurious target velocity through the radar's long position lever arm.
    if isempty(sampleIdx)
        targetPosition = zeros(0, 2);
        return;
    end
    radarPosition = localRadarRows(frame, sampleIdx);
    targetPosition = NaN(numel(sampleIdx), 2);
    times = cellfun(@(value) value.time, frame(sampleIdx));
    rates=cellfun(@(value) value.yawRateMeasured,frame(sampleIdx));
    increments=.5*(rates(1:end-1)+rates(2:end)).*diff(times);
    yaw=currentYaw-[flipud(cumsum(flipud(increments(:))));0];
    for positionIdx = 1:numel(sampleIdx)
        frameIdx = sampleIdx(positionIdx);
        egoPosition = [ ...
            frame{frameIdx}.xGps; frame{frameIdx}.yGps];
        rotation = localRotation(yaw(positionIdx));
        targetPosition(positionIdx, :) = ...
            (egoPosition ...
                + rotation*radarPosition(positionIdx, :).').';
    end
end

function targetState = localDormantTargetState(cfg)
% localDormantTargetState Park a constant-velocity phantom ahead of range.
%
% In the transformed coordinates the target velocity q is the absolute
% target velocity in the ego frame, so a dormant oncoming phantom is
% simply q = [-targetSpeed; 0] with zero absolute acceleration; no ego
% state enters the prior.
    relativePosition = [ ...
        cfg.sensor.radar.rangeMaximum+10.0; 0.0];
    targetVelocityEgoFrame = [-cfg.vehicle.targetSpeed; 0.0];
    targetState = [ ...
        relativePosition; targetVelocityEgoFrame; zeros(2, 1)];
end

function [context, justAcquired] = ...
        localUpdateTargetAcquisition(context, frame, egoEstimate)
    justAcquired = false;
    if context.targetAcquired
        return;
    end

    acquisition = context.targetAcquisition;
    if frame.radarDetectionAvailable
        acquisition = localAppendRadarDetection( ...
            acquisition, frame, egoEstimate, ...
            context.configuration.observer.runtime.samplePeriod);
    else
        acquisition.time = zeros(0, 1);
        acquisition.radarRelativePosition = zeros(0, 2);
        acquisition.targetPositionInertial = zeros(0, 2);
        acquisition.velocityInitializationMethod = "";
        acquisition.consecutiveDetectionCount = 0;
    end
    context.targetAcquisition = acquisition;
    if acquisition.consecutiveDetectionCount ...
            < acquisition.minimumSampleCount
        return;
    end

    selectedIdx = ...
        (acquisition.consecutiveDetectionCount ...
            - acquisition.minimumSampleCount+1) ...
        :acquisition.consecutiveDetectionCount;
    [targetState, targetSpeed] = ...
        localTargetStateFromInertialWindow( ...
            acquisition.time(selectedIdx), ...
            acquisition.targetPositionInertial(selectedIdx, :), ...
            egoEstimate.egoPositionInertial, egoEstimate.egoYaw, ...
            context.configuration, ...
            context.observerDesign.target.domain.accelerationNormBound);
    acquisition.velocityInitializationMethod = ...
        localTargetVelocityInitializationMethod( ...
            numel(selectedIdx), ...
            acquisition.time(selectedIdx(end))-acquisition.time(selectedIdx(1)), ...
            context.configuration);
    context.targetAcquisition = acquisition;
    context.runtime = onlineNrmmTrackingRuntime("resetTarget", context.runtime, targetState);
    context.runtime.targetCertifiedSpeedDomainValid = true;
    context.runtime.minimumReconstructedTargetSpeed = targetSpeed;
    context.runtime.lastIntervalTargetCertifiedSpeedDomainValid = true;
    context.targetAcquired = true;
    context.targetAcquisitionTime = frame.time;
    justAcquired = true;
end

function context = localUpdateTargetLifecycle( ...
        context, frame, egoEstimate)
    if ~context.targetAcquired
        context.targetTrackStatus = "dormant";
        return;
    end
    if frame.radarDetectionAvailable
        context.targetLastDetectionTime = frame.time;
        context.targetTrackStatus = "confirmed";
        return;
    end

    coastAge = frame.time-context.targetLastDetectionTime;
    maximumCoastDuration = ...
        context.configuration.sensor.radar ...
            .maximumTrackCoastDuration;
    tolerance = 100.0*eps(max( ...
        1.0, max(abs([frame.time; ...
            context.targetLastDetectionTime; ...
            maximumCoastDuration]))));
    if coastAge <= maximumCoastDuration+tolerance
        context.targetTrackStatus = "coasting";
        return;
    end

    context.targetAcquired = false;
    context.targetTrackStatus = "dormant";
    context.targetRetirementCount = ...
        context.targetRetirementCount+1;
    context.targetLastRetirementTime = frame.time;
    context.targetLastDetectionTime = NaN;
    context.targetAcquisition = localEmptyTargetAcquisition( ...
        context.targetAcquisition.minimumSampleCount);
    context = localAnchorDormantTarget(context, egoEstimate);
end

function context = localAnchorDormantTarget(context, ~)
    targetState = localDormantTargetState(context.configuration);
    context.runtime = onlineNrmmTrackingRuntime("resetTarget", context.runtime, targetState);
    context.runtime.lastIntervalTargetCertifiedSpeedDomainValid = true;
    context.runtime.minimumReconstructedTargetSpeed = min( ...
        context.runtime.minimumReconstructedTargetSpeed, ...
        context.configuration.vehicle.targetSpeed);
end

function acquisition = localEmptyTargetAcquisition(minimumSampleCount)
    acquisition = struct( ...
        "minimumSampleCount", minimumSampleCount, ...
        "time", zeros(0, 1), ...
        "radarRelativePosition", zeros(0, 2), ...
        "targetPositionInertial", zeros(0, 2), ...
        "velocityInitializationMethod", "", ...
        "consecutiveDetectionCount", 0);
end

function age = localTargetTrackCoastAge(context, time)
    if context.targetTrackStatus == "coasting"
        age = time-context.targetLastDetectionTime;
    elseif context.targetTrackStatus == "confirmed"
        age = 0.0;
    else
        age = NaN;
    end
end

function acquisition = localAppendRadarDetection( ...
        acquisition, frame, egoEstimate, samplePeriod)
    rotation = localRotation(egoEstimate.egoYaw);
    targetPosition = ...
        (egoEstimate.egoPositionInertial ...
            + rotation*frame.radarRelativePosition(:)).';
    tolerance = 100.0*eps(max( ...
        1.0, max(abs([frame.time; acquisition.time(:)]))));
    if isempty(acquisition.time)
        acquisition.time = frame.time;
        acquisition.radarRelativePosition = ...
            frame.radarRelativePosition;
        acquisition.targetPositionInertial = targetPosition;
    elseif abs(frame.time-acquisition.time(end)) <= tolerance
        acquisition.time(end) = frame.time;
        acquisition.radarRelativePosition(end, :) = ...
            frame.radarRelativePosition;
        acquisition.targetPositionInertial(end, :) = ...
            targetPosition;
    elseif abs(frame.time-acquisition.time(end)-samplePeriod) ...
            <= tolerance
        acquisition.time(end+1, 1) = frame.time;
        acquisition.radarRelativePosition(end+1, :) = ...
            frame.radarRelativePosition;
        acquisition.targetPositionInertial(end+1, :) = ...
            targetPosition;
    else
        acquisition.time = frame.time;
        acquisition.radarRelativePosition = ...
            frame.radarRelativePosition;
        acquisition.targetPositionInertial = targetPosition;
    end
    acquisition.consecutiveDetectionCount = ...
        numel(acquisition.time);
end

function [targetState, reconstructedTargetSpeed] = ...
        localTargetStateFromInertialWindow( ...
            detectionTime, targetPositionInertial, ...
            egoPosition, egoYaw, cfg, accelerationBound)
% The transformed target state is [rho; q; s]: radar relative position,
% absolute target velocity in the ego frame, and absolute target
% acceleration in the ego frame. A multi-sample window supplies the fitted
% velocity; only a single detection uses the provisional velocity prior.
% A window spanning at least initialization.accelerationFitMinimumDuration
% is fitted with a constant acceleration as well (an NRMM target has
% constant A and beta, so its acceleration turns only slowly over the
% window). Its tangential and normal components are kept only where they
% exceed three standard errors of the fit (estimated from its residuals),
% so a straight or steady target keeps the constant-velocity zero rather
% than fitted noise; the result is clipped to the acceleration domain. A
% shorter window keeps the constant-velocity zero.
    if any(abs(diff(detectionTime) ...
            - cfg.observer.runtime.samplePeriod) ...
            > 100.0*eps(max(1.0, max(abs(detectionTime)))))
        error("nrmmEstimatorControllerAdapter:nonconsecutiveRadarHistory", ...
            "Target initialization requires consecutive radar samples.");
    end
    targetPosition = targetPositionInertial(end, :).';
    if isscalar(detectionTime)
        lineOfSight = targetPosition - egoPosition;
        lineOfSightNorm = norm(lineOfSight);
        if lineOfSightNorm ...
                <= 100.0*eps(max(1.0, lineOfSightNorm))
            error("nrmmEstimatorControllerAdapter:" ...
                    + "invalidTargetInitialization", ...
                "The radial oncoming velocity prior requires a " ...
                + "nonzero radar line of sight.");
        end
        targetVelocity = ...
            -cfg.initialization.targetSpeedPrior ...
                * lineOfSight/lineOfSightNorm;
        targetAcceleration = zeros(2, 1);
    else
        relativeTime = detectionTime-detectionTime(end);
        fitAcceleration = numel(detectionTime) >= 3 ...
            && -relativeTime(1) >= cfg.initialization.accelerationFitMinimumDuration ...
                - 100.0*eps(max(1.0, max(abs(detectionTime))));
        basis = [ones(size(relativeTime)), relativeTime];
        if fitAcceleration
            basis = [basis, relativeTime.^2/2];
        end
        targetModel = basis \ targetPositionInertial;
        targetAcceleration = zeros(2, 1);
        if fitAcceleration
            residual = targetPositionInertial-basis*targetModel;
            deviation = sqrt(sum(residual(:).^2)/(2*(numel(relativeTime)-3)));
            covariance = inv(basis.'*basis);
            standardError = deviation*sqrt(covariance(3, 3));
            tangent = targetModel(2, :).'/max(norm(targetModel(2, :)), eps);
            frame = [tangent, [-tangent(2); tangent(1)]];
            components = frame.'*targetModel(3, :).';
            components(abs(components) <= 3.0*standardError) = 0.0;
            targetAcceleration = frame*components;
            targetAcceleration = targetAcceleration ...
                *min(1.0, accelerationBound/max(norm(targetAcceleration), eps));
        end
        rawTargetVelocity = targetModel(2, :).';
        rawTargetSpeed = norm(rawTargetVelocity);
        if rawTargetSpeed ...
                < cfg.observer.target.domain.speedMinimum
            error("nrmmEstimatorControllerAdapter:" ...
                    + "invalidTargetInitialization", ...
                "Radar/GNSS samples reconstructed a target below the " ...
                + "configured NRMM speed domain.");
        end
        targetVelocity = rawTargetVelocity;
    end
    reconstructedTargetSpeed = norm(targetVelocity);
    rotationTranspose = localRotation(egoYaw).';
    relativePosition = rotationTranspose ...
        * (targetPosition-egoPosition);
    targetVelocityEgoFrame = rotationTranspose*targetVelocity;
    targetState = [ ...
        relativePosition; targetVelocityEgoFrame; ...
        rotationTranspose*targetAcceleration];
end

function method = localTargetVelocityInitializationMethod(sampleCount, duration, cfg)
    if sampleCount == 1
        method = "radialOncomingConfiguredSpeedPrior";
    elseif sampleCount >= 3 && duration >= cfg.initialization.accelerationFitMinimumDuration ...
            - 100.0*eps(max(1.0, duration))
        method = "radarPositionWindowAccelerationLeastSquares";
    else
        method = "radarPositionWindowVelocityLeastSquares";
    end
end

function time = localInitialAcquisitionTime(acquired)
    if acquired
        time = 0.0;
    else
        time = NaN;
    end
end

function status = localInitialTrackStatus(acquired)
    if acquired
        status = "confirmed";
    else
        status = "dormant";
    end
end

function frame = localFrameWithoutRadar(frame)
    frame.radarRelativePosition = [NaN, NaN];
    frame.radarDetectionAvailable = false;
end

function [frame, audit] = localSensorFrame( ...
        cfg, stream, time, egoTruth, bodyAcceleration, targetTruth, heldInput)
    if nargin < 7,heldInput = [];end
    unitNoise = localUnitNoise(stream, 9, cfg.noiseModel);
    % Vector sensor maxima are Euclidean radii, matching the observer proof.
    % Independent component draws occupy the inscribed square of each ball.
    unitNoise([1:4,6:9]) = unitNoise([1:4,6:9])/sqrt(2);
    position = localEgoPosition(egoTruth);
    yaw = localEgoYaw(egoTruth);
    yawRate = localEgoYawRate(egoTruth);
    targetPosition = localTargetPosition(targetTruth);
    relativeInertial = targetPosition-position;
    targetRange = norm(relativeInertial);
    rangeTolerance = 100.0*eps(max( ...
        1.0, cfg.sensor.radar.rangeMaximum));
    detected = targetRange <= cfg.sensor.radar.rangeMaximum ...
        + rangeTolerance;
    if detected
        radarRelativePosition = ...
            (localRotation(yaw).'*relativeInertial).' ...
            + cfg.sensor.radar.positionNoiseMaximum ...
                * unitNoise(6:7);
    else
        radarRelativePosition = [NaN, NaN];
    end

    frame = struct( ...
        "time", time, ...
        "xGps", position(1) ...
            + cfg.sensor.gnss.positionNoiseMaximum*unitNoise(1), ...
        "yGps", position(2) ...
            + cfg.sensor.gnss.positionNoiseMaximum*unitNoise(2), ...
        "longitudinalAcceleration", bodyAcceleration(1) ...
            + cfg.sensor.imu.accelerationNoiseMaximum*unitNoise(3), ...
        "lateralAcceleration", bodyAcceleration(2) ...
            + cfg.sensor.imu.accelerationNoiseMaximum*unitNoise(4), ...
        "yawRateMeasured", yawRate ...
            + cfg.sensor.gyroscope.yawRateNoiseMaximum*unitNoise(5), ...
        "radarRelativePosition", radarRelativePosition, ...
        "radarDetectionAvailable", detected);
    if numel(heldInput) == 2
        % The controller's held command is known exactly to the estimator.
        frame.heldSteeringAngle = heldInput(1);
        frame.heldBrakingRatio = heldInput(2);
    end
    audit = struct( ...
        "time", time, ...
        "targetRange", targetRange, ...
        "radarDetectionAvailable", detected, ...
        "radarRangeMaximum", cfg.sensor.radar.rangeMaximum);
    audit.sensorErrorNorms = [ ...
        norm([frame.xGps;frame.yGps]-position); ...
        norm([frame.longitudinalAcceleration;frame.lateralAcceleration]-bodyAcceleration); ...
        abs(frame.yawRateMeasured-yawRate)];
    audit.sensorErrorLimits = [cfg.sensor.gnss.positionNoiseMaximum; ...
        cfg.sensor.imu.accelerationNoiseMaximum; ...
        cfg.sensor.gyroscope.yawRateNoiseMaximum];
    if detected
        audit.sensorErrorNorms(end+1) = norm(radarRelativePosition.' ...
            -localRotation(yaw).'*relativeInertial);
        audit.sensorErrorLimits(end+1) = cfg.sensor.radar.positionNoiseMaximum;
    end
    audit.sampledSensorNoiseWithinBounds = all( ...
        audit.sensorErrorNorms<=audit.sensorErrorLimits+1e-10);
end

function unitNoise = localUnitNoise(stream, count, noiseModel)
    switch lower(string(noiseModel))
        case "none"
            unitNoise = zeros(1, count);
        case "boundeduniform"
            unitNoise = 2.0*rand(stream, 1, count)-1.0;
        otherwise
            error("nrmmEstimatorControllerAdapter:invalidNoiseModel", ...
                "noiseModel must be 'none' or 'boundedUniform'.");
    end
end

function acceleration = localSampledBodyAcceleration( ...
        previousState, currentState, samplePeriod)
% localSampledBodyAcceleration Preserve the sampled rotation correspondence.
%
% The harness interpolates yaw and yaw rate independently between plant
% outputs. Differencing body velocity and then adding the pointwise yaw-rate
% term does not, in general, equal the derivative of the GNSS inertial
% velocity synthesized from those same interpolants. Form the backward
% inertial-velocity difference first, then express that acceleration in the
% current body frame. Consequently the noiseless synthetic measurements
% satisfy aI = R(psi)*aE exactly at every sensor sample.
    previousInertialVelocity = localRotation( ...
        localEgoYaw(previousState))*localBodyVelocity(previousState);
    currentRotation = localRotation(localEgoYaw(currentState));
    currentInertialVelocity = ...
        currentRotation*localBodyVelocity(currentState);
    inertialAcceleration = (currentInertialVelocity ...
        - previousInertialVelocity)/samplePeriod;
    acceleration = currentRotation.'*inertialAcceleration;
end

function acceleration = localNominalBodyAcceleration(state)
    planarCross = [0.0, -1.0; 1.0, 0.0];
    acceleration = localEgoYawRate(state) ...
        * planarCross*localBodyVelocity(state);
end

function state = localHistoricalEgoState(initialState, time)
    position = localEgoPosition(initialState);
    yaw = localEgoYaw(initialState);
    yawRate = localEgoYawRate(initialState);
    bodyVelocity = localBodyVelocity(initialState);
    historicalYaw = yaw+yawRate*time;
    if abs(yawRate) <= 1.0e-10
        displacement = localRotation(yaw)*bodyVelocity*time;
    else
        terminalYaw = historicalYaw;
        displacement = [ ...
            bodyVelocity(1)*(sin(terminalYaw)-sin(yaw))/yawRate ...
                + bodyVelocity(2)*(cos(terminalYaw)-cos(yaw))/yawRate; ...
            bodyVelocity(1)*(cos(yaw)-cos(terminalYaw))/yawRate ...
                + bodyVelocity(2)*(sin(terminalYaw)-sin(yaw))/yawRate];
    end
    state = struct( ...
        "position", position+displacement, ...
        "yawAngle", historicalYaw, ...
        "longitudinalVelocity", bodyVelocity(1), ...
        "lateralVelocity", bodyVelocity(2), ...
        "yawRate", yawRate);
end

function position = localEgoPosition(state)
    if isfield(state, "position")
        position = double(state.position(:));
    else
        position = [double(state.positionX); double(state.positionY)];
    end
end

function yaw = localEgoYaw(state)
    if isfield(state, "yawAngle")
        yaw = double(state.yawAngle);
    else
        yaw = double(state.yaw);
    end
end

function yawRate = localEgoYawRate(state)
    yawRate = double(state.yawRate);
end

function velocity = localBodyVelocity(state)
    velocity = [ ...
        double(state.longitudinalVelocity); ...
        double(state.lateralVelocity)];
end

function position = localTargetPosition(target)
    if ~isstruct(target) || numel(target) ~= 1 ...
            || ~isfield(target, "targetPositionInertial")
        error("nrmmEstimatorControllerAdapter:invalidTargetTruth", ...
            "The integration test requires one target truth trajectory.");
    end
    position = double(target.targetPositionInertial(:));
    if numel(position) ~= 2 || any(~isfinite(position))
        error("nrmmEstimatorControllerAdapter:invalidTargetTruth", ...
            "targetPositionInertial must be a finite two-vector.");
    end
end

function localValidateConfiguration(cfg)
    required = ["randomSeed", "noiseModel", "sensor", ...
        "initialization", "observer", "vehicle"];
    if ~isstruct(cfg) || ~isscalar(cfg) || ~all(isfield(cfg, required))
        error("nrmmEstimatorControllerAdapter:invalidConfiguration", ...
            "cfg must come from estimatorControllerIntegrationConfig.");
    end
    if isfield(cfg, "publishedErrorBound")
        error("nrmmEstimatorControllerAdapter:fixedErrorBoundOverride", ...
            "Remove publishedErrorBound: controller bounds now come from the online estimator.");
    end
    publicationPolicyValid = ...
        isfield(cfg.sensor, "radar") ...
        && isfield(cfg.sensor.radar, "targetPublicationPolicy") ...
        && isscalar(string( ...
            cfg.sensor.radar.targetPublicationPolicy)) ...
        && string(cfg.sensor.radar.targetPublicationPolicy) ...
            == "currentRadarVisibilityRequiredInternalTrackMayCoast";
    trackCoastDurationValid = ...
        isfield(cfg.sensor.radar, "maximumTrackCoastDuration") ...
        && isscalar(cfg.sensor.radar.maximumTrackCoastDuration) ...
        && isfinite(cfg.sensor.radar.maximumTrackCoastDuration) ...
        && cfg.sensor.radar.maximumTrackCoastDuration >= 0.0;
    if cfg.observer.runtime.samplePeriod <= 0.0 ...
            || cfg.sensor.radar.rangeMaximum <= 0.0 ...
            || ~publicationPolicyValid ...
            || ~trackCoastDurationValid ...
            || cfg.initialization.minimumRadarSamples < 1 ...
            || ~isfield(cfg.initialization, "accelerationFitMinimumDuration") ...
            || ~(cfg.initialization.accelerationFitMinimumDuration > 0.0) ...
            || ~isfield(cfg.initialization, "targetSpeedPrior") ...
            || ~isfinite(cfg.initialization.targetSpeedPrior) ...
            || cfg.initialization.targetSpeedPrior ...
                < cfg.observer.target.domain.speedMinimum ...
            || ~isfield(cfg.vehicle, "targetSpeed") ...
            || ~isfinite(cfg.vehicle.targetSpeed) ...
            || cfg.vehicle.targetSpeed <= 0.0
        error("nrmmEstimatorControllerAdapter:invalidConfiguration", ...
            "Sample time, radar range/publication/coast policy, and " ...
            + "initialization count are invalid.");
    end
end

function rotation = localRotation(yaw)
    rotation = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
end

function localAddProjectPaths()
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(repositoryRoot, "config"));
    addpath(genpath(fullfile(repositoryRoot, "estimator")));
end
