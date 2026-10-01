function design = sharmaMultistageObserverDesign(cfg, options)
% sharmaMultistageObserverDesign Design the Sharma (2026) multistage observer.
%
% Reproduces the gain design of Sharma, Alai and Rajamani, Transp. Res. Part C
% 182 (2026) 105411, Section 3, for use as a paired comparator:
%
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
%
% Options
%   Variant       "published" | "corrected" (see sharmaNrmmCompanionDerivative)
%   Theta         "certified" (theta = ThetaMargin*max(theta0, 1) as Theorem 1
%                 prescribes) or "matched" (theta scales the slowest normalized
%                 pole to the slowest physical pole of ReferenceDesign, so both
%                 designs share a convergence rate).
%   ReferenceDesign  synthesizeNrmmObserverGains output, required for "matched".
%   ThetaMargin   multiplicative margin above theta0 (default 1.1).
%   YawRateThreshold, AccelerationThreshold  switching thresholds u_th, a_th of
%                 eq. (37) (the paper gives no values; defaults 0.05 rad/s and
%                 0.5 m/s^2).
%   SpeedFloor    guard for the 1/V_C^2 division (default 1 m/s).
%   SampledRealization  "outputPredictor" (default) propagates each GPS and radar
%                 sample between frames with the observer's own velocity
%                 estimates, the same sampled realization the comparator uses;
%                 "zeroOrderHold" holds the raw sample, which biases the
%                 innovation by about half a sample of relative motion.
%
% Requires YALMIP and SeDuMi in solver/ (same dependency as the comparator).

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
                error("sharmaMultistageObserverDesign:missingReference", ...
                    "Theta=""matched"" requires ReferenceDesign.");
            end
            reference = options.ReferenceDesign;
            referenceRates.ego = reference.velocity.gain;
            referenceRates.target = reference.target.bandwidth ...
                *min(abs(real(reference.target.closedLoopPoles)));
            egoTheta = referenceRates.ego/slowestNormalizedRate;
            targetTheta = referenceRates.target/slowestNormalizedRate;
        otherwise
            error("sharmaMultistageObserverDesign:invalidTheta", "Unknown Theta option.");
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
        error("sharmaMultistageObserverDesign:lmiInfeasible", ...
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
        error("sharmaMultistageObserverDesign:invalidLmiSolution", ...
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
        error("sharmaMultistageObserverDesign:missingLmiSolver", ...
            "Theorem 1 requires YALMIP and SeDuMi in %s.", solverRoot);
    end
    addpath(genpath(yalmipRoot));
    addpath(genpath(sedumiRoot));
end
