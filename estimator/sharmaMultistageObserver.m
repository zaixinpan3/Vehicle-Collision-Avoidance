function varargout = sharmaMultistageObserver(action, varargin)
% sharmaMultistageObserver Reproduction of the Sharma (2026) multistage observer.
%
% Paired comparator for the cascaded measured-input observer of this
% repository: the gain design, the sampled runtime and the companion-form
% NRMM map of Sharma, Alai and Rajamani, "Simultaneous ego-vehicle state
% estimation and vehicle trajectory tracking using a multistage high gain
% observer", Transp. Res. Part C 182 (2026) 105411, Section 3.
%
%   design = sharmaMultistageObserver("design", cfg, Name=Value)
%       Gain design (see "Design" below).
%   runtime = sharmaMultistageObserver("initialize", cfg, options, design)
%   [runtime, output] = sharmaMultistageObserver("step", runtime, frame)
%   output = sharmaMultistageObserver("output", runtime, frame)
%       Sampled realization with the same action interface as
%       onlineNrmmTrackingRuntime (see "Runtime" below).
%   [derivative, reconstruction] = sharmaMultistageObserver("companionDerivative", state, input, variant, speedFloor)
%       The companion-form NRMM process map, eqs. (47)-(64) (see "Companion
%       form" below).
%
% Design
%   * Ego stage (Sec. 3.1): six-state companion chain z_E = [x; xDot; xDdot;
%     y; yDot; yDdot] driven by GPS position and the gyroscope, eqs. (26)-(28),
%     with Assumption 1 (zero ego jerk and zero yaw acceleration).
%   * Gain LMI (Theorem 1): F'*P + P*F - H'*R - R'*H <= -lambda*I, K = P\R',
%     L = T(theta)*K with T(theta) = blkdiag(diag([theta theta^2 theta^3]), ...)
%     and theta > theta0 = 2*gamma*lambdaMax(P)/lambda, eqs. (30)-(33). The
%     paper fixes no objective for the LMI; this design minimizes lambdaMax(P)
%     subject to P >= I and lambda = 1, which minimizes the required theta0.
%     The ego and target chains share (F, H), so they share (P, K).
%   * Ego Lipschitz constant gamma_E = max|psiEdot| (eq. 29); target Lipschitz
%     constant gamma_C from eq. (73) on the constant-yaw-rate hyperplanes,
%     maximized over the sign combinations of c1 = max|psiCdot| and
%     c2 = max|psiEdot|.
%   * Yaw stage (eqs. 34-39): psiHatDot = u3 + kt*(yt - psiHat). The H-infinity
%     LMI (39) has no minimizer in kt (mu decreases monotonically in kt), so
%     kt is set to the comparator's continuous yaw bandwidth log(20)/T_domain
%     and (39) is verified with its minimal mu for that kt.
%   Options
%     Variant       "published" | "corrected" (see "Companion form")
%     Theta         "certified" (theta = ThetaMargin*max(theta0, 1) as Theorem 1
%                   prescribes) or "matched" (theta scales the slowest normalized
%                   pole to the slowest physical pole of ReferenceDesign, so both
%                   designs share a convergence rate).
%     ReferenceDesign  synthesizeNrmmObserverGains output, required for "matched".
%     ThetaMargin   multiplicative margin above theta0 (default 1.1).
%     YawRateThreshold, AccelerationThreshold  switching thresholds u_th, a_th of
%                   eq. (37) (the paper gives no values; defaults 0.05 rad/s and
%                   0.5 m/s^2).
%     SpeedFloor    guard for the 1/V_C^2 division (default 1 m/s).
%     SampledRealization  "outputPredictor" (default) propagates each GPS and
%                   radar sample between frames with the observer's own velocity
%                   estimates, the same sampled realization the comparator uses;
%                   "zeroOrderHold" holds the raw sample, which biases the
%                   innovation by about half a sample of relative motion.
%   Requires YALMIP and SeDuMi in solver/ (same dependency as the comparator).
%
% Runtime
%   options carries egoInitialPosition (2-by-1), egoInitialYaw,
%   egoInitialBodyVelocity (2-by-1) and targetInitialState ([rho; q; s], the
%   comparator's coordinates); the companion-form target state [r; rDot; rDdot]
%   is formed from it at the first frame using that frame's gyroscope and
%   accelerometer samples. Each frame's accelerometer and gyroscope samples
%   are held over the sample interval. The GPS position and radar samples are
%   either propagated between frames with the observer's own velocity
%   estimates (design.sampledRealization "outputPredictor") or held
%   ("zeroOrderHold"). The three stages (ego chain eq. 28, yaw eq. 37, NRMM
%   chain eq. 65) are advanced together by fixed-step RK4. A radar sample
%   flagged unavailable removes the radar innovation for that interval. The
%   output maps the companion state back to [rho; q; s] through eqs. (54)-(61)
%   so that both observers are scored in the same coordinates.
%
% Companion form
%   state = [rx; rxDot; rxDdot; ry; ryDot; ryDdot] (paper eq. 47); the ego
%   inputs are input.bodyVelocity = [vx; vy], input.bodyAcceleration = [ax; ay]
%   and input.yawRate = psiEdot (paper u_C2, eq. 63). The returned derivative
%   is F*state + G*f_C2 with f_C2 = [f1; f2] from eqs. (62)-(63). variant
%   selects how the third derivative is evaluated:
%     "published" evaluates eqs. (62)-(63) exactly as printed.
%     "corrected" adds the ego body-velocity second-derivative term that the
%                 printed equations omit. Differentiating r'' = s - psiEdot*J*q
%                 - vDot - psiEdot*J*rDot once more under the paper's own
%                 Assumption 1 (aDot = 0, psiEddot = 0, so vDdot = -psiEdot*J*vDot,
%                 eqs. 52-53) gives r''' = published + psiEdot*J*vDot, i.e.
%                 f1 gains -psiEdot*vyDot and f2 gains +psiEdot*vxDot.
%   Both variants keep Assumption 1; neither models ego jerk or ego angular
%   acceleration. speedFloor guards the 1/V_C^2 division of eq. (59) when
%   observer peaking drives the reconstructed target speed toward zero; the
%   paper does not specify such a guard. reconstruction reports V_Cx, V_Cy,
%   zeta, psiCdot, A_Cx, A_Cy and whether the guard was active.

    action = lower(string(action));
    switch action
        case "design"
            varargout{1} = localDesign(varargin{:});
        case "initialize"
            varargout{1} = localInitialize(varargin{:});
        case "step"
            [varargout{1}, varargout{2}] = localStep(varargin{:});
        case "output"
            varargout{1} = localOutput(varargin{1}, localInput(varargin{2}, varargin{1}));
        case "companionderivative"
            [varargout{1:max(1, nargout)}] = localCompanionDerivative(varargin{:});
        otherwise
            error("sharmaMultistageObserver:invalidAction", ...
                "action must be 'design', 'initialize', 'step', 'output' or 'companionDerivative'.");
    end
end

%% Design (Section 3, Theorems 1 and 2)

function design = localDesign(cfg, options)
    arguments
        cfg (1, 1) struct
        options.Variant (1, 1) string ...
            {mustBeMember(options.Variant, ["published", "corrected"])} = "published"
        options.Theta (1, 1) string ...
            {mustBeMember(options.Theta, ["certified", "matched"])} = "certified"
        options.ReferenceDesign struct = struct()
        options.ThetaMargin (1, 1) double {mustBeGreaterThan(options.ThetaMargin, 1.0)} = 1.1
        options.YawRateThreshold (1, 1) double {mustBePositive} = 0.05
        options.AccelerationThreshold (1, 1) double {mustBePositive} = 0.5
        options.SpeedFloor (1, 1) double {mustBePositive} = 1.0
        options.SampledRealization (1, 1) string {mustBeMember( ...
            options.SampledRealization, ["outputPredictor", "zeroOrderHold"])} = "outputPredictor"
    end

    localPrepareSolver();
    chainMatrix = [0.0, 1.0, 0.0; 0.0, 0.0, 1.0; 0.0, 0.0, 0.0];
    systemMatrix = blkdiag(chainMatrix, chainMatrix);
    outputMatrix = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0; 0.0, 0.0, 0.0, 1.0, 0.0, 0.0];
    inputMatrix = [0.0, 0.0; 0.0, 0.0; 1.0, 0.0; 0.0, 0.0; 0.0, 0.0; 0.0, 1.0];
    lambda = 1.0;
    [lyapunovMatrix, gainSeed, lmiReport] = localSolveTheoremOneLmi( ...
        systemMatrix, outputMatrix, lambda);
    normalizedClosedLoop = systemMatrix - gainSeed*outputMatrix;
    normalizedPoles = eig(normalizedClosedLoop);
    slowestNormalizedRate = min(abs(real(normalizedPoles)));
    lambdaMaximum = max(eig(lyapunovMatrix));

    %% Operating domain and Lipschitz constants
    egoYawRateMaximum = cfg.ego.domain.yawRateMaximum;
    egoLipschitz = egoYawRateMaximum;                         % eq. (29)
    targetDomain = localTargetDomain(cfg);
    targetLipschitz = localTargetLipschitz(targetDomain.yawRateMaximum, egoYawRateMaximum);
    egoThetaThreshold = 2.0*egoLipschitz*lambdaMaximum/lambda;        % eq. (31)
    targetThetaThreshold = 2.0*targetLipschitz*lambdaMaximum/lambda;

    egoSpeedMaximum = cfg.ego.domain.speedMaximum;
    domainTransitTime = targetDomain.relativePositionMaximum ...
        /(egoSpeedMaximum + targetDomain.speedMaximum);
    referenceRates = struct("ego", NaN, "target", NaN);
    switch options.Theta
        case "certified"
            egoTheta = options.ThetaMargin*max(egoThetaThreshold, 1.0);
            targetTheta = options.ThetaMargin*max(targetThetaThreshold, 1.0);
        case "matched"
            if isempty(fieldnames(options.ReferenceDesign))
                error("sharmaMultistageObserver:missingReference", ...
                    "Theta=""matched"" requires ReferenceDesign.");
            end
            reference = options.ReferenceDesign;
            referenceRates.ego = reference.velocity.gain;
            referenceRates.target = reference.target.bandwidth ...
                *min(abs(real(reference.target.closedLoopPoles)));
            egoTheta = referenceRates.ego/slowestNormalizedRate;
            targetTheta = referenceRates.target/slowestNormalizedRate;
        otherwise
            error("sharmaMultistageObserver:invalidTheta", "Unknown Theta option.");
    end
    egoGain = localScaledGain(gainSeed, egoTheta);
    targetGain = localScaledGain(gainSeed, targetTheta);

    %% Yaw stage (eqs. 36-39)
    yawGain = log(20.0)/domainTransitTime;
    gyroscopeNoise = cfg.measurement.gyroscope.noiseMaximum;
    headingNoise = cfg.measurement.gps.velocityNoiseMaximum/max(cfg.ego.domain.speedMinimum, eps);
    noiseInput = [gyroscopeNoise, 0.0];                 % B in eq. (38)
    noiseOutput = [0.0, headingNoise];                  % D in eq. (38)
    hInfinity = localVerifyYawLmi(yawGain, noiseInput, noiseOutput);

    design = struct();
    design.construction = "Sharma et al. (2026) multistage high-gain observer, " ...
        + "companion-form ego and NRMM chains with T(theta) scaling";
    design.variant = options.Variant;
    design.thetaSelection = options.Theta;
    design.sampledRealization = options.SampledRealization;
    design.assumptions = struct( ...
        "zeroEgoJerk", true, "zeroEgoYawAcceleration", true, ...
        "constantTargetScalarAcceleration", true, "constantTargetSideslip", true, ...
        "yawEstimateEntersTargetInputs", true);
    design.lmi = struct("systemMatrix", systemMatrix, "outputMatrix", outputMatrix, ...
        "inputMatrix", inputMatrix, "lyapunovMatrix", lyapunovMatrix, ...
        "lyapunovMaximumEigenvalue", lambdaMaximum, "lambda", lambda, ...
        "gainSeed", gainSeed, "normalizedClosedLoop", normalizedClosedLoop, ...
        "normalizedPoles", normalizedPoles, "slowestNormalizedRate", slowestNormalizedRate, ...
        "objective", "minimize lambdaMax(P) subject to P >= I, lambda = 1", ...
        "report", lmiReport);
    design.ego = struct("lipschitz", egoLipschitz, "thetaThreshold", egoThetaThreshold, ...
        "theta", egoTheta, "gain", egoGain, ...
        "physicalPoles", egoTheta*normalizedPoles, ...
        "slowestPhysicalRate", egoTheta*slowestNormalizedRate, ...
        "measurement", "GPS position; gyroscope in the nonlinearity");
    design.yaw = struct("gain", yawGain, "yawRateThreshold", options.YawRateThreshold, ...
        "accelerationThreshold", options.AccelerationThreshold, ...
        "hInfinity", hInfinity, ...
        "method", "switched course/acceleration heading, eqs. (34)-(37), signed atan2 form", ...
        "courseModel", struct("rearAxleDistance", cfg.ego.yaw.rearAxleDistance));
    design.target = struct("lipschitz", targetLipschitz, "thetaThreshold", targetThetaThreshold, ...
        "theta", targetTheta, "gain", targetGain, ...
        "physicalPoles", targetTheta*normalizedPoles, ...
        "slowestPhysicalRate", targetTheta*slowestNormalizedRate, ...
        "innovationGains", targetGain([1, 2, 3], 1), ...
        "bandwidth", targetTheta, ...
        "speedFloor", options.SpeedFloor, ...
        "domain", targetDomain);
    design.referenceRates = referenceRates;
    design.gainSelection = struct("domainTransitTime", domainTransitTime, ...
        "thetaMargin", options.ThetaMargin);
    design.certificateScope = struct( ...
        "timeModel", "continuous high-gain observer (Gauthier-Kupka), Assumption 1", ...
        "lipschitzConstantScope", "constant-yaw-rate hyperplanes only (paper eq. 73)", ...
        "sampledImplementationCertified", false);
end

function [lyapunovMatrix, gainSeed, report] = localSolveTheoremOneLmi(systemMatrix, outputMatrix, lambda)
% localSolveTheoremOneLmi Solve eq. (30) with the smallest lambdaMax(P).
    stateCount = size(systemMatrix, 1);
    outputCount = size(outputMatrix, 1);
    lyapunovVariable = sdpvar(stateCount, stateCount);
    gainVariable = sdpvar(outputCount, stateCount, 'full');
    bound = sdpvar(1, 1);
    constraints = [lyapunovVariable >= eye(stateCount), ...
        lyapunovVariable <= bound*eye(stateCount), ...
        systemMatrix.'*lyapunovVariable + lyapunovVariable*systemMatrix ...
            - outputMatrix.'*gainVariable - gainVariable.'*outputMatrix <= -lambda*eye(stateCount)];
    settings = sdpsettings('solver', 'sedumi', 'verbose', 0);
    warningState = warning("off", "MATLAB:rankDeficientMatrix");
    restoreWarning = onCleanup(@() warning(warningState));
    diagnostics = optimize(constraints, bound, settings);
    if diagnostics.problem ~= 0
        error("sharmaMultistageObserver:lmiInfeasible", ...
            "Theorem 1 LMI did not solve: %s", yalmiperror(diagnostics.problem));
    end
    lyapunovMatrix = value(lyapunovVariable);
    lyapunovMatrix = (lyapunovMatrix + lyapunovMatrix.')/2.0;
    gainSeed = lyapunovMatrix\value(gainVariable).';                      % eq. (32)
    residual = systemMatrix.'*lyapunovMatrix + lyapunovMatrix*systemMatrix ...
        - outputMatrix.'*value(gainVariable) - value(gainVariable).'*outputMatrix ...
        + lambda*eye(stateCount);
    residualMargin = max(eig((residual + residual.')/2.0));
    if min(eig(lyapunovMatrix)) <= 0.0 || residualMargin > 1.0e-6
        error("sharmaMultistageObserver:invalidLmiSolution", ...
            "The recovered Theorem 1 solution violates the LMI (margin %.3g).", residualMargin);
    end
    report = struct("solver", "sedumi", "objectiveValue", value(bound), ...
        "residualMargin", residualMargin, "solverInfo", diagnostics.info);
end

function gain = localScaledGain(gainSeed, theta)
% localScaledGain Apply T(theta) of eq. (33) to the normalized gain.
    scaling = blkdiag(diag([theta, theta^2, theta^3]), diag([theta, theta^2, theta^3]));
    gain = scaling*gainSeed;
end

function gamma = localTargetLipschitz(targetYawRateMaximum, egoYawRateMaximum)
% localTargetLipschitz Evaluate eq. (73) over the sign combinations of c1, c2.
    gamma = 0.0;
    for c1 = [-targetYawRateMaximum, targetYawRateMaximum]
        for c2 = [-egoYawRateMaximum, egoYawRateMaximum]
            first = 2.0*c1^2 - 6.0*c1*c2 + 3.0*c2^2;
            third = 2.0*c1^2 - 3.0*c1*c2 + c2^2;
            value = sqrt(2.0*first^2 + 18.0*(c1 - c2)^2 + 2.0*c2^2*third^2);
            gamma = max(gamma, value);
        end
    end
end

function domain = localTargetDomain(cfg)
    speedMaximum = cfg.target.domain.speedMaximum;
    sideslipMaximum = cfg.target.domain.sideslipMaximum;
    rearAxleDistance = cfg.target.domain.rearAxleDistance;
    yawRateMaximum = speedMaximum*sin(sideslipMaximum)/rearAxleDistance;
    domain = struct( ...
        "speedMinimum", cfg.target.domain.speedMinimum, ...
        "speedMaximum", speedMaximum, ...
        "scalarAccelerationMaximum", cfg.target.domain.scalarAccelerationMaximum, ...
        "yawRateMaximum", yawRateMaximum, ...
        "accelerationNormBound", hypot(cfg.target.domain.scalarAccelerationMaximum, ...
            speedMaximum*yawRateMaximum), ...
        "sideslipMaximum", sideslipMaximum, ...
        "rearAxleDistance", rearAxleDistance, ...
        "relativePositionMaximum", cfg.target.domain.relativePositionMaximum);
end

function hInfinity = localVerifyYawLmi(gain, noiseInput, noiseOutput)
% localVerifyYawLmi Evaluate eq. (39) for the fixed gain with its minimal mu.
%
% With scalar P the LMI reads [-2*P*kt+1, P*(B-kt*D); (B-kt*D)'*P, -mu*I] <= 0.
% Its Schur complement gives mu >= P^2*|B-kt*D|^2/(2*P*kt-1) for P > 1/(2*kt);
% the minimum over P is at P = 1/kt, mu = |B-kt*D|^2/kt^2.
    mixedNoise = noiseInput - gain*noiseOutput;
    lyapunovScalar = 1.0/gain;
    mu = (norm(mixedNoise)/gain)^2;
    lmiMatrix = [-2.0*lyapunovScalar*gain + 1.0, lyapunovScalar*mixedNoise; ...
        lyapunovScalar*mixedNoise.', -mu*eye(2)];
    feasible = max(eig((lmiMatrix + lmiMatrix.')/2.0)) <= 1.0e-9;
    hInfinity = struct("lyapunovScalar", lyapunovScalar, "mu", mu, ...
        "noiseInput", noiseInput, "noiseOutput", noiseOutput, "feasible", feasible, ...
        "note", "mu decreases monotonically in kt, so eq. (39) alone fixes no gain");
end

function localPrepareSolver()
    if ~isempty(which("optimize")) && ~isempty(which("sedumi"))
        return
    end
    repoRoot = fileparts(fileparts(mfilename("fullpath")));
    solverRoot = fullfile(repoRoot, "solver");
    yalmipRoot = fullfile(solverRoot, "YALMIP");
    sedumiRoot = fullfile(solverRoot, "sedumi");
    if ~isfolder(yalmipRoot) || ~isfolder(sedumiRoot)
        error("sharmaMultistageObserver:missingLmiSolver", ...
            "Theorem 1 requires YALMIP and SeDuMi in %s.", solverRoot);
    end
    addpath(genpath(yalmipRoot));
    addpath(genpath(sedumiRoot));
end

%% Sampled runtime

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
    [targetDerivative, reconstruction] = localCompanionDerivative( ...
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
    [~, reconstruction] = localCompanionDerivative( ...
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
        error("sharmaMultistageObserver:offSampleGrid", ...
            "The frame time must equal the observer time.");
    end
end

function rotation = localRotation(angle)
    rotation = [cos(angle), -sin(angle); sin(angle), cos(angle)];
end

function value = localWrapToPi(value)
    value = mod(value + pi, 2.0*pi) - pi;
end

%% Companion-form NRMM process map, eqs. (47)-(64)

function [derivative, reconstruction] = localCompanionDerivative(state, input, variant, speedFloor)
    arguments
        state (6, 1) double {mustBeReal}
        input (1, 1) struct
        variant (1, 1) string {mustBeMember(variant, ["published", "corrected"])}
        speedFloor (1, 1) double {mustBeNonnegative}
    end

    vx = input.bodyVelocity(1);
    vy = input.bodyVelocity(2);
    ax = input.bodyAcceleration(1);
    ay = input.bodyAcceleration(2);
    psiEdot = input.yawRate;

    rx = state(1);
    rxDot = state(2);
    rxDdot = state(3);
    ry = state(4);
    ryDot = state(5);
    ryDdot = state(6);

    % Eqs. (50)-(51): body-frame derivative of the ego velocity components.
    vxDot = ax + vy*psiEdot;
    vyDot = ay - vx*psiEdot;
    % Eqs. (54)-(55): target absolute velocity in the ego frame.
    vCx = rxDot + vx - psiEdot*ry;
    vCy = ryDot + vy + psiEdot*rx;
    speedSquared = vCx^2 + vCy^2;
    guardActive = speedSquared < speedFloor^2;
    speedSquaredGuarded = max(speedSquared, speedFloor^2);
    % Eqs. (57)-(58).
    zetaX = rxDdot + vxDot - psiEdot*ryDot - vCy*psiEdot;
    zetaY = ryDdot + vyDot + psiEdot*rxDot + vCx*psiEdot;
    % Eq. (59).
    psiCdot = (zetaY*vCx - zetaX*vCy)/speedSquaredGuarded;
    % Eqs. (60)-(61).
    aCx = zetaX + vCy*psiCdot;
    aCy = zetaY - vCx*psiCdot;
    % Eqs. (62)-(63) as printed.
    relativeRate = psiCdot - psiEdot;
    f1 = -3.0*aCy*psiCdot + 2.0*psiEdot*aCy - vCx*relativeRate^2 + psiEdot*ryDdot;
    f2 = 3.0*aCx*psiCdot - 2.0*psiEdot*aCx - vCy*relativeRate^2 - psiEdot*rxDdot;
    if variant == "corrected"
        f1 = f1 - psiEdot*vyDot;
        f2 = f2 + psiEdot*vxDot;
    end

    derivative = [rxDot; rxDdot; f1; ryDot; ryDdot; f2];
    if nargout < 2
        return
    end
    reconstruction = struct( ...
        "targetVelocity", [vCx; vCy], ...
        "targetSpeed", sqrt(speedSquared), ...
        "zeta", [zetaX; zetaY], ...
        "targetYawRate", psiCdot, ...
        "targetAcceleration", [aCx; aCy], ...
        "egoBodyVelocityDerivative", [vxDot; vyDot], ...
        "speedGuardActive", guardActive);
end
