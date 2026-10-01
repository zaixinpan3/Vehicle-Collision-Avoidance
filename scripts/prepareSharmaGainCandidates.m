function candidates = prepareSharmaGainCandidates(reference)
% prepareSharmaGainCandidates Empirical gain alternatives for a paper reproduction.
% The paper leaves its LMI objective, theta, yaw gain and switch thresholds
% unspecified. Explore two stable normalized gain shapes and three independent
% ego, target and yaw rates. The companion dynamics and measurements are kept.
% These candidates are not claimed to meet the nonlinear Theorem 1 threshold.

    arguments
        reference (1, 1) struct
    end
    candidates = repmat(reference, 1, 54);
    index = 0;
    for shape = ["lmi", "repeated"]
        for egoRate = [1.5, 3.0, 6.0]
            for targetRate = [1.5, 3.0, 6.0]
                for yawRate = [0.1, 1.0, 3.0]
                    index = index + 1;
                    design = reference.design;
                    if shape == "repeated"
                        seed = zeros(6, 2);
                        seed(1:3, 1) = [3; 3; 1];
                        seed(4:6, 2) = [3; 3; 1];
                        normalizedRate = 1.0;
                    else
                        seed = design.lmi.gainSeed;
                        normalizedRate = design.lmi.slowestNormalizedRate;
                    end
                    design.ego = localStage(design.ego, seed, egoRate/normalizedRate);
                    design.target = localStage(design.target, seed, targetRate/normalizedRate);
                    design.yaw.gain = yawRate;
                    design.yaw.hInfinity = struct("note", "Not evaluated for empirical candidate");
                    design.lmi = struct("systemMatrix", design.lmi.systemMatrix, ...
                        "outputMatrix", design.lmi.outputMatrix, ...
                        "inputMatrix", design.lmi.inputMatrix, "gainSeed", seed, ...
                        "note", "Stable normalized gain; nonlinear threshold not claimed");
                    design.thetaSelection = "empirical-training-grid";
                    design.variant = "published";
                    design.audit = struct("gainShape", shape, "egoRate", egoRate, ...
                        "targetRate", targetRate, "yawGain", yawRate, ...
                        "theoremOneClaimed", false);
                    design.certificateScope.timeModel = ...
                        "Empirical stable-linear-gain candidate; nonlinear certificate not claimed";
                    label = sprintf("%s_e%g_t%g_y%g", shape, egoRate, targetRate, yawRate);
                    candidates(index) = struct("name", string(label), ...
                        "runtime", @sharmaMultistageObserverRuntime, "design", design, ...
                        "description", "Sharma dynamics with training-selected gains: " + label);
                end
            end
        end
    end
end

function stage = localStage(stage, seed, theta)
    scale = diag(repmat([theta, theta^2, theta^3], 1, 2));
    stage.gain = scale*seed;
    matrix = blkdiag([0 1 0; 0 0 1; 0 0 0], [0 1 0; 0 0 1; 0 0 0]);
    output = [1 0 0 0 0 0; 0 0 0 1 0 0];
    stage.physicalPoles = eig(matrix - stage.gain*output);
    stage.slowestPhysicalRate = min(-real(stage.physicalPoles));
    stage.theta = theta;
    stage.thetaThreshold = NaN;
    if isfield(stage, "innovationGains")
        stage.innovationGains = stage.gain(1:3, 1);
        stage.bandwidth = theta;
    end
end
