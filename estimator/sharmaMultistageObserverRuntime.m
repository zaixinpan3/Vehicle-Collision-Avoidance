function varargout = sharmaMultistageObserverRuntime(action, varargin)
% sharmaMultistageObserverRuntime Run the Sharma (2026) multistage observer.
%
% Sampled realization of the observer designed by sharmaMultistageObserverDesign
% with the same action interface as onlineNrmmTrackingRuntime:
%
%   runtime = sharmaMultistageObserverRuntime("initialize", cfg, options, design)
%   [runtime, output] = sharmaMultistageObserverRuntime("step", runtime, frame)
%   output = sharmaMultistageObserverRuntime("output", runtime, frame)
%
% options carries egoInitialPosition (2-by-1), egoInitialYaw, egoInitialBodyVelocity
% (2-by-1) and targetInitialState ([rho; q; s], the comparator's coordinates);
% the companion-form target state [r; rDot; rDdot] is formed from it at the
% first frame using that frame's gyroscope and accelerometer samples.
%
% Each frame's accelerometer and gyroscope samples are held over the sample
% interval. The GPS position and radar samples are either propagated between
% frames with the observer's own velocity estimates (design.sampledRealization
% "outputPredictor", the realization the comparator uses) or held
% ("zeroOrderHold"). The three stages (ego chain eq. 28, yaw eq. 37, NRMM chain
% eq. 65) are advanced together by fixed-step RK4. A radar sample flagged
% unavailable removes the radar innovation for that interval. The output maps
% the companion state back to [rho; q; s] through eqs. (54)-(61) so that both
% observers are scored in the same coordinates.

    action = lower(string(action));
    switch action
        case "initialize"
            varargout{1} = localInitialize(varargin{:});
        case "step"
            [varargout{1}, varargout{2}] = localStep(varargin{:});
        case "output"
            varargout{1} = localOutput(varargin{1}, localInput(varargin{2}, varargin{1}));
        otherwise
            error("sharmaMultistageObserverRuntime:invalidAction", ...
                "action must be 'initialize', 'step' or 'output'.");
    end
end

function runtime = localInitialize(cfg, options, design)
    samplePeriod = cfg.runtime.samplePeriod;
    integrationStepMaximum = cfg.runtime.integrationStepMaximum;
    % The theta-scaled gains place poles at theta*normalizedPoles; fixed-step
    % RK4 needs step*|pole| below its stability radius, so the substep count
    % also honours a 0.5 rad per step limit on the fastest physical pole. The
    % step actually used is published so the comparison can report it.
    fastestRate = max(abs([design.ego.physicalPoles; design.target.physicalPoles]));
    substepCount = max([1, ceil(samplePeriod/integrationStepMaximum), ...
        ceil(samplePeriod*fastestRate/0.5)]);
    initialTime = 0.0;
    if isfield(options, "initialTime")
        initialTime = options.initialTime;
    end
    egoPosition = options.egoInitialPosition(:);
    egoYaw = options.egoInitialYaw;
    egoBodyVelocity = options.egoInitialBodyVelocity(:);
    inertialVelocity = localRotation(egoYaw)*egoBodyVelocity;
    % Acceleration states start at zero: the paper's chain estimates them
    % from GPS positions and has no accelerometer input.
    egoState = [egoPosition(1); inertialVelocity(1); 0.0; ...
        egoPosition(2); inertialVelocity(2); 0.0];
    runtime = struct( ...
        "design", design, ...
        "samplePeriod", samplePeriod, ...
        "integrationStep", samplePeriod/substepCount, ...
        "substepCount", substepCount, ...
        "currentTime", initialTime, ...
        "egoState", egoState, ...
        "yawEstimate", egoYaw, ...
        "targetState", NaN(6, 1), ...
        "pendingTargetInitialState", options.targetInitialState(:), ...
        "lastRadar", NaN(2, 1), ...
        "lastInput", struct(), ...
        "speedGuardCount", 0, ...
        "stepCount", 0, ...
        "diverged", false);
end

function [runtime, output] = localStep(runtime, frame)
    input = localInput(frame, runtime);
    if any(isnan(runtime.targetState))
        runtime.targetState = localCompanionFromTransformed( ...
            runtime.pendingTargetInitialState, runtime, input);
    end
    if input.radarAvailable
        runtime.lastRadar = input.radar;
    end
    % Output predictors reset to the fresh samples at every frame.
    radarPredictor = input.radar;
    if ~input.radarAvailable
        radarPredictor = runtime.targetState([1, 4]);
    end
    state = [runtime.egoState; runtime.yawEstimate; runtime.targetState; ...
        input.gpsPosition; radarPredictor];
    step = runtime.integrationStep;
    guardCount = 0;
    for substep = 1:runtime.substepCount
        [k1, g1] = localDerivative(state, input, runtime.design);
        [k2, g2] = localDerivative(state + 0.5*step*k1, input, runtime.design);
        [k3, g3] = localDerivative(state + 0.5*step*k2, input, runtime.design);
        [k4, g4] = localDerivative(state + step*k3, input, runtime.design);
        state = state + (step/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
        state(7) = localWrapToPi(state(7));
        guardCount = guardCount + double(any([g1, g2, g3, g4]));
    end
    if ~all(isfinite(state))
        runtime.diverged = true;
        state(~isfinite(state)) = NaN;
    end
    runtime.egoState = state(1:6);
    runtime.yawEstimate = state(7);
    runtime.targetState = state(8:13);
    runtime.speedGuardCount = runtime.speedGuardCount + guardCount;
    runtime.stepCount = runtime.stepCount + 1;
    runtime.currentTime = input.time + runtime.samplePeriod;
    runtime.lastInput = input;
    output = localOutput(runtime, input);
end

function [derivative, guardActive] = localDerivative(state, input, design)
    egoState = state(1:6);
    yaw = state(7);
    targetState = state(8:13);
    gpsPredictor = state(14:15);
    radarPredictor = state(16:17);
    psiEdot = input.yawRate;

    %% Ego chain, eqs. (26)-(28)
    nonlinearity = [-psiEdot*egoState(6); psiEdot*egoState(3)];   % f_E(u3, z)
    egoDerivative = design.lmi.systemMatrix*egoState + design.lmi.inputMatrix*nonlinearity ...
        + design.ego.gain*(gpsPredictor - design.lmi.outputMatrix*egoState);

    %% Yaw observer, eqs. (34)-(37)
    heading = localYawMeasurement(egoState, input, design.yaw);
    yawDerivative = psiEdot + design.yaw.gain*localWrapToPi(heading - yaw);

    %% NRMM chain, eqs. (64)-(65), driven by the ego estimates
    bodyVelocity = localRotation(yaw).'*egoState([2, 5]);
    egoInput = struct("bodyVelocity", bodyVelocity, ...
        "bodyAcceleration", input.bodyAcceleration, "yawRate", psiEdot);
    [targetDerivative, reconstruction] = sharmaNrmmCompanionDerivative( ...
        targetState, egoInput, design.variant, design.target.speedFloor);
    guardActive = reconstruction.speedGuardActive;
    if input.radarAvailable
        innovation = radarPredictor - design.lmi.outputMatrix*targetState;
        targetDerivative = targetDerivative + design.target.gain*innovation;
    end

    %% Sample predictors: the samples move with the estimated velocities
    if design.sampledRealization == "outputPredictor"
        gpsPredictorDerivative = egoState([2, 5]);
        radarPredictorDerivative = targetState([2, 5]);
    else
        gpsPredictorDerivative = zeros(2, 1);
        radarPredictorDerivative = zeros(2, 1);
    end
    derivative = [egoDerivative; yawDerivative; targetDerivative; ...
        gpsPredictorDerivative; radarPredictorDerivative];
end

function heading = localYawMeasurement(egoState, input, yawDesign)
% localYawMeasurement Switched course/acceleration heading of eqs. (34)-(37).
%
% The paper's rules select the acceleration-based heading y2 when the
% gyroscope or accelerometer magnitude exceeds its threshold and the course
% angle y1 otherwise. y2 is evaluated in signed form,
% atan2(a_body x a_inertial, a_body . a_inertial). This repairs both the
% missing normalization and the sign ambiguity of the printed eq. (34);
% it is not a literal implementation of that equation. When the estimated
% inertial acceleration is too small to define a
% direction, y1 is used instead.
    inertialVelocity = egoState([2, 5]);
    inertialAcceleration = egoState([3, 6]);
    bodyAcceleration = input.bodyAcceleration;
    useAcceleration = abs(input.yawRate) > yawDesign.yawRateThreshold ...
        || norm(bodyAcceleration) > yawDesign.accelerationThreshold;
    if useAcceleration && norm(inertialAcceleration) > 0.1*yawDesign.accelerationThreshold
        heading = atan2(bodyAcceleration(1)*inertialAcceleration(2) ...
                - bodyAcceleration(2)*inertialAcceleration(1), ...
            bodyAcceleration.'*inertialAcceleration);
    else
        heading = atan2(inertialVelocity(2), inertialVelocity(1));
    end
end

function companion = localCompanionFromTransformed(transformed, runtime, input)
% localCompanionFromTransformed Map [rho; q; s] to [r; rDot; rDdot], eqs. (54)-(58).
    planarCross = [0.0, -1.0; 1.0, 0.0];
    psiEdot = input.yawRate;
    bodyVelocity = localRotation(runtime.yawEstimate).'*runtime.egoState([2, 5]);
    bodyVelocityDot = input.bodyAcceleration - psiEdot*planarCross*bodyVelocity;
    rho = transformed(1:2);
    q = transformed(3:4);
    s = transformed(5:6);
    rDot = q - bodyVelocity - psiEdot*planarCross*rho;
    rDdot = s - psiEdot*planarCross*q - bodyVelocityDot - psiEdot*planarCross*rDot;
    companion = [rho(1); rDot(1); rDdot(1); rho(2); rDot(2); rDdot(2)];
end

function output = localOutput(runtime, input)
    design = runtime.design;
    egoState = runtime.egoState;
    yaw = runtime.yawEstimate;
    rotation = localRotation(yaw);
    inertialVelocity = egoState([2, 5]);
    bodyVelocity = rotation.'*inertialVelocity;
    psiEdot = input.yawRate;
    targetState = runtime.targetState;
    if any(isnan(targetState))
        targetState = localCompanionFromTransformed(runtime.pendingTargetInitialState, runtime, input);
    end
    egoInput = struct("bodyVelocity", bodyVelocity, ...
        "bodyAcceleration", input.bodyAcceleration, "yawRate", psiEdot);
    [~, reconstruction] = sharmaNrmmCompanionDerivative( ...
        targetState, egoInput, design.variant, design.target.speedFloor);
    rho = targetState([1, 4]);
    rDot = targetState([2, 5]);
    q = reconstruction.targetVelocity;
    s = reconstruction.zeta;                   % zeta = s exactly, eqs. (57)-(58)
    speed = reconstruction.targetSpeed;
    yawRate = reconstruction.targetYawRate;
    scalarAcceleration = (q.'*s)/max(speed, design.target.speedFloor);
    sideslipSine = min(max(design.target.domain.rearAxleDistance*yawRate ...
        /max(speed, design.target.speedFloor), -1.0), 1.0);
    sideslip = asin(sideslipSine);
    courseEgoFrame = atan2(q(2), q(1));
    targetEstimate = struct( ...
        "relativePosition", rho, ...
        "relativeVelocity", rDot, ...
        "targetVelocity", q, ...
        "targetAcceleration", s, ...
        "targetSpeed", speed, ...
        "targetScalarAcceleration", scalarAcceleration, ...
        "targetYawRate", yawRate, ...
        "targetCourseAngleEgoFrame", courseEgoFrame, ...
        "targetSideslip", sideslip, ...
        "targetPositionInertial", egoState([1, 4]) + rotation*rho, ...
        "targetVelocityInertial", rotation*q, ...
        "targetAccelerationInertial", rotation*s, ...
        "targetCourseAngleInertial", localWrapToPi(yaw + courseEgoFrame), ...
        "targetHeadingInertial", localWrapToPi(yaw + courseEgoFrame - sideslip), ...
        "companionState", targetState, ...
        "estimateTime", runtime.currentTime);
    output = struct( ...
        "time", runtime.currentTime, ...
        "stateTime", runtime.currentTime, ...
        "inputSampleTime", input.time, ...
        "egoState", egoState, ...
        "egoPositionInertial", egoState([1, 4]), ...
        "egoVelocityInertial", inertialVelocity, ...
        "egoAccelerationInertial", egoState([3, 6]), ...
        "egoBodyVelocity", bodyVelocity, ...
        "egoYaw", yaw, ...
        "egoYawRate", psiEdot, ...
        "egoSideslip", atan2(bodyVelocity(2), bodyVelocity(1)), ...
        "yawEstimator", "sharma-switched-course-acceleration", ...
        "radarDetectionAvailable", input.radarAvailable, ...
        "targetState", [rho; q; s], ...
        "targetEstimate", targetEstimate, ...
        "speedGuardCount", runtime.speedGuardCount, ...
        "diverged", runtime.diverged, ...
        "certificateScope", design.certificateScope);
    if runtime.diverged
        output.targetState(:) = NaN;
    end
    output.integrationStep = runtime.integrationStep;
end

function input = localInput(frame, runtime)
    radar = double(frame.radarRelativePosition(:));
    available = logical(frame.radarDetectionAvailable) && all(isfinite(radar));
    if ~available
        radar = runtime.lastRadar;
    end
    input = struct( ...
        "time", double(frame.time), ...
        "gpsPosition", [double(frame.xGps); double(frame.yGps)], ...
        "gpsVelocity", [double(frame.vxGps); double(frame.vyGps)], ...
        "bodyAcceleration", [double(frame.longitudinalAcceleration); ...
            double(frame.lateralAcceleration)], ...
        "yawRate", double(frame.yawRateMeasured), ...
        "radar", radar, ...
        "radarAvailable", available);
    tolerance = 64.0*eps(max(1.0, abs(runtime.currentTime)));
    if abs(input.time - runtime.currentTime) > tolerance
        error("sharmaMultistageObserverRuntime:offSampleGrid", ...
            "The frame time must equal the observer time.");
    end
end

function rotation = localRotation(angle)
    rotation = [cos(angle), -sin(angle); sin(angle), cos(angle)];
end

function value = localWrapToPi(value)
    value = mod(value + pi, 2.0*pi) - pi;
end
