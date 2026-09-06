function result = analyzeRobustOutputFeedbackDesign()
%analyzeRobustOutputFeedbackDesign Reproduce bounded research diagnostics.
% Run from the repository root after addpath('scripts'). No controller is
% changed or admitted. These algebraic examples are not vehicle safety tests.

    root = fileparts(fileparts(mfilename("fullpath")));
    previousPath = path;
    cleanup = onCleanup(@() path(previousPath));
    addpath(fullfile(root, "controller"), fullfile(root, "config"));
    cfg = collisionAvoidanceControllerConfig();
    sampleTime = 0.05;
    speeds = [0, 1, 5, 15];
    curvatures = [0, 1/400];
    domainRows = zeros(numel(speeds)*numel(curvatures), 4);
    row = 0;
    for curvature = curvatures
        for speed = speeds
            row = row+1;
            [stateMatrix, inputMatrix] = ltvBicycleStageMatrices( ...
                curvature, speed, sampleTime, cfg);
            domainRows(row, :) = [curvature, speed, ...
                rank(ctrb(stateMatrix, inputMatrix)), ...
                rank([eye(6)-stateMatrix, inputMatrix])];
        end
    end
    result.bicycle = array2table(domainRows, VariableNames= ...
        ["curvaturePerMeter", "speedMetersPerSecond", ...
        "controllabilityRank", "unitEigenvaluePbhRank"]);

    % True-minus-nominal error: d+ = (A+BK)d - BK e + w,
    % where e = true-minus-estimated state, and u = v + K(xhat-z).
    stateMatrix = [1, sampleTime; 0, 1];
    inputMatrix = 0.8*[0.5*sampleTime^2; sampleTime];
    feedback = -dlqr(stateMatrix, inputMatrix, eye(2), 1);
    closedMatrix = stateMatrix+inputMatrix*feedback;
    metric = dlyap(closedMatrix.', eye(2));
    metricFactor = chol(metric);
    contraction = norm(metricFactor*closedMatrix/metricFactor, 2);
    disturbanceRadius = [1e-4; 1e-3];
    initialRadius = [0.1; 0.05];
    corners = [-1, -1, 1, 1; -1, 1, -1, 1];
    metricRadius = max(vecnorm(metricFactor*(initialRadius.*corners)));
    boxRadius = initialRadius;
    stageCount = 200;
    generator = RandStream("mt19937ar", Seed=7);
    trialCount = 512;
    errorState = initialRadius.*(2*rand(generator, 2, trialCount)-1);
    maximumNormalizedError = 0;
    for stage = 1:stageCount
        time = (stage-1)*sampleTime;
        estimationRadius = [0.01; 0.005]+[0.09; 0.045]*exp(-time);
        forcingRadius = 0;
        for errorCorner = 1:4
            for disturbanceCorner = 1:4
                forcing = -inputMatrix*feedback*(estimationRadius.*corners(:, errorCorner)) ...
                    + disturbanceRadius.*corners(:, disturbanceCorner);
                forcingRadius = max(forcingRadius, norm(metricFactor*forcing));
            end
        end
        metricRadius = contraction*metricRadius+forcingRadius;
        boxRadius = abs(closedMatrix)*boxRadius ...
            + abs(inputMatrix*feedback)*estimationRadius+disturbanceRadius;
        estimationError = estimationRadius.*(2*rand(generator, 2, trialCount)-1);
        disturbance = disturbanceRadius.*(2*rand(generator, 2, trialCount)-1);
        errorState = closedMatrix*errorState-inputMatrix*feedback*estimationError+disturbance;
        maximumNormalizedError = max(maximumNormalizedError, ...
            max(vecnorm(metricFactor*errorState))/metricRadius);
    end
    componentRadius = metricRadius*sqrt(diag(metric\eye(2)));
    omittedEffect = -inputMatrix*feedback*[0; 0.05];
    result.longitudinal = struct("sampleTimeSeconds", sampleTime, ...
        "inputGain", 0.8, "feedback", feedback, ...
        "closedLoopSpectralRadius", max(abs(eig(closedMatrix))), ...
        "absoluteMatrixSpectralRadius", max(abs(eig(abs(closedMatrix)))), ...
        "metricContraction", contraction, "stages", stageCount, ...
        "randomSeed", 7, "sampledTrajectories", trialCount, ...
        "maximumSampledNormalizedError", maximumNormalizedError, ...
        "finalBoxRadius", boxRadius, "finalMetricComponentRadius", componentRadius, ...
        "omittedEstimationFeedbackEffect", omittedEffect);

    % Equal inertial headings do not imply equal Frenet heading errors.
    radius = 60;
    stationError = 0.12;
    result.coordinate = struct("roadRadiusMeters", radius, ...
        "stationErrorMeters", stationError, "inertialYawErrorRadians", 0, ...
        "frenetHeadingErrorRadians", stationError/radius);
    % A continuous acceleration disturbance also changes position in a hold.
    result.hold = struct("sampleTimeSeconds", sampleTime, ...
        "accelerationDisturbanceMetersPerSecondSquared", 1, ...
        "exactPositionErrorMeters", 0.5*sampleTime^2, ...
        "exactVelocityErrorMetersPerSecond", sampleTime, ...
        "componentwiseRateTimesStepPositionBoundMeters", 0);
    observer = nrmmTrackingConfig();
    integration = estimatorControllerIntegrationConfig();
    result.observer = struct("defaultMinimumEgoSpeedMetersPerSecond", ...
        observer.ego.domain.speedMinimum, "adapterMinimumEgoSpeedMetersPerSecond", ...
        integration.observer.ego.domain.speedMinimum, "terminalSpeedMetersPerSecond", 0);
    result.scope = "Algebraic diagnostics and sampled LTI containment only; no NRMM vehicle closed-loop certification";

    assert(all(domainRows(domainRows(:, 2) == 0, 4) < 6));
    assert(result.longitudinal.closedLoopSpectralRadius < 1);
    assert(result.longitudinal.absoluteMatrixSpectralRadius > 1);
    assert(contraction < 1 && maximumNormalizedError <= 1+1e-12);
    assert(result.coordinate.frenetHeadingErrorRadians > 0);
    assert(result.hold.exactPositionErrorMeters > ...
        result.hold.componentwiseRateTimesStepPositionBoundMeters);
    disp(result.bicycle);
    disp(result.longitudinal);
    disp(result.observer);
end
