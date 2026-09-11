function report = analyzeClfRecoveryMechanisms(sourceDirectory, outputDirectory)
%analyzeClfRecoveryMechanisms Replay saved trials and probe the optimal face.
% Reproduces the recorded commands with certificate memory, then compares
% fixed-state counterfactuals under the original hard constraints and zero
% CLF slack. Alternative plans are diagnostics, never applied to the plant.
% The selected lateral-derivative probe adds a constraint, not an input cost.

    arguments
        sourceDirectory (1,1) string
        outputDirectory (1,1) string
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'controller'), fullfile(root, 'config'), ...
        fullfile(root, 'solver', 'clarabel', 'matlab'));
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    previousThreads = maxNumCompThreads(1);
    cleanup = onCleanup(@() maxNumCompThreads(previousThreads));
    selectedTimes = [7.2, 8.0, 9.0, 9.7, 9.75, 9.8];
    records = cell(1, 2);
    cases = cell(1, 2);
    for sceneIndex = 1:2
        names = ["straight", "arc"];
        scene = names(sceneIndex);
        loaded = load(fullfile(sourceDirectory, scene+'_avoidance.mat'), 'result');
        trial = loaded.result;
        cfg = trial.controllerConfiguration;
        certificate = [];
        trace = cell(1, numel(trial.command));
        probes = cell(1, numel(selectedTimes));
        cases{sceneIndex} = cell(1, numel(selectedTimes));
        prepared = prepareCollisionAvoidanceController(trial.controllerEgoEstimate{1}, ...
            trial.perception.roadBoundaryFit{1}.roadGeometry, cfg);
        assert(prepared.allProbesCertified);
        for sample = 1:numel(trial.command)
            ego = trial.controllerEgoEstimate{sample};
            target = trial.targetEstimate{sample};
            road = trial.perception.roadBoundaryFit{sample}.roadGeometry;
            [command, ~, problem, certificate] = collisionAvoidanceController( ...
                ego, target, road, cfg, certificate);
            assert(isequal(command.actuatorInput, trial.command{sample}.actuatorInput), ...
                'analyzeClfRecoveryMechanisms:replayMismatch', 'Recorded input did not reproduce.');
            assert(problem.metadata.planCertified && ~problem.metadata.fallbackUsed);
            trace{sample} = localSample(problem, trial, sample, cfg);
            selected = find(abs(trial.controlTime(sample)-selectedTimes) < 1e-9, 1);
            if ~isempty(selected)
                [~, lane] = readPlanningInputs(ego, target, road, cfg);
                model = struct('cfg', cfg, 'lane', lane, ...
                    'hasTarget', ~isempty(target), 'egoHalfLength', cfg.vehicle.length/2, ...
                    'egoHalfWidth', cfg.vehicle.width/2, 'targetHalfLength', 0, ...
                    'targetHalfWidth', 0, 'targetHorizon', certificate.targetHorizon);
                if ~isempty(target)
                    model.targetHalfLength = target.targetLength/2;
                    model.targetHalfWidth = target.targetWidth/2;
                end
                probes{selected} = localProbe(problem, model, trace{sample});
                cases{sceneIndex}{selected} = struct('time', trial.controlTime(sample), ...
                    'problem', problem, 'model', model, 'probe', probes{selected});
            end
        end
        traceTable = struct2table([trace{:}]);
        writetable(traceTable, fullfile(outputDirectory, scene+'_trace.csv'));
        recovery = traceTable.timeSeconds >= 9-1e-9;
        records{sceneIndex} = struct('scene', scene, 'reproducedCommands', numel(trace), ...
            'probes', [probes{:}], 'recoveryCalls', nnz(recovery), ...
            'recoveryPredictedVIncreases', nnz(traceTable.predictedDeltaV(recovery) > 1e-10), ...
            'recoveryActualVIncreases', nnz(traceTable.actualDeltaV(recovery) > 1e-10), ...
            'recoveryLateralDerivativePositive', nnz(traceTable.lateralDerivative(recovery) > 1e-10), ...
            'maximumRecoveryPredictionLateralError', max(abs(traceTable.lateralModelResidual(recovery))), ...
            'maximumRecoveryPredictionHeadingError', max(abs(traceTable.headingModelResidual(recovery))));
        save(fullfile(outputDirectory, scene+'_cases.mat'), 'probes', 'traceTable', '-v7.3');
        fprintf('%s: reproduced %d commands; completed %d optimal-face probes.\n', scene, numel(trace), numel(probes));
    end
    report = struct('sourceDirectory', sourceDirectory, 'matlabVersion', string(version), ...
        'selectedTimes', selectedTimes, 'scenes', [records{:}], ...
        'scope', 'Recorded-input replay and fixed-state feasible-set probes; no altered-input plant run, controller tuning, or timing claim.');
    save(fullfile(outputDirectory, 'mechanisms.mat'), 'report', 'cases', '-v7.3');
    localWriteJson(fullfile(outputDirectory, 'mechanisms.json'), report);
end

function row = localSample(problem, trial, sample, cfg)
    clf = problem.qp.clf;
    prediction = problem.prediction;
    p = clf.lyapunovMatrix;
    state = prediction.egoStateOffset(:, 1);
    error = clf.errorOffset(:, 1);
    reference = state(2:6)-error;
    next = prediction.stageMatrixA(:, :, 1)*state ...
        + prediction.stageMatrixB(:, :, 1)*problem.decision(1:2) ...
        + prediction.stageAffine(:, 1);
    predictedError = next(2:6)-reference;
    actual = [trial.controlTracking.lateralError(sample+1); ...
        trial.controlTracking.headingError(sample+1); trial.controlState(sample+1, 4:6).'];
    actualError = actual-reference;
    [a, b, affine] = ltvBicycleModel.continuousMatrices( ...
        prediction.scheduleCurvature(1), prediction.scheduleSpeedProfile(1), cfg);
    derivative = a*state+b*problem.decision(1:2)+affine;
    lateral = [1, 2, 4, 5];
    assert(norm(p(3, lateral), inf) < 1e-12);
    delta = predictedError-error;
    v = error.'*p*error;
    predictedDelta = predictedError.'*p*predictedError-v;
    linearTerm = 2*error.'*p*delta;
    quadraticTerm = delta.'*p*delta;
    assert(abs(predictedDelta-linearTerm-quadraticTerm) < 1e-9);
    speedDerivative = 2*p(3, 3)*error(3)*derivative(4);
    lateralDerivative = 2*error(lateral).'*p(lateral, lateral)*derivative([2, 3, 5, 6]);
    assert(abs(speedDerivative+lateralDerivative-problem.metadata.clfDerivative) < 1e-9);
    row = struct('timeSeconds', trial.controlTime(sample), ...
        'referenceSpeed', problem.metadata.performanceReferenceSpeed, ...
        'lateralError', error(1), 'headingError', error(2), 'speedError', error(3), ...
        'totalV', v, 'lateralV', error(lateral).'*p(lateral, lateral)*error(lateral), ...
        'speedDerivative', speedDerivative, 'lateralDerivative', lateralDerivative, ...
        'requiredTotalDerivative', -clf.decayRate*v, ...
        'relaxation', problem.metadata.clfRelaxation, ...
        'predictedDeltaV', predictedDelta, 'actualDeltaV', actualError.'*p*actualError-v, ...
        'predictedNextLateralV', predictedError(lateral).'*p(lateral, lateral)*predictedError(lateral), ...
        'actualNextLateralV', actualError(lateral).'*p(lateral, lateral)*actualError(lateral), ...
        'predictedNextHeading', next(3), 'actualNextHeading', actual(2), ...
        'linearIncrement', linearTerm, 'quadraticIncrement', quadraticTerm, ...
        'instantaneousEulerTerm', cfg.controller.sampleTime*problem.metadata.clfDerivative, ...
        'lateralModelResidual', actual(1)-next(2), 'headingModelResidual', actual(2)-next(3), ...
        'steeringRadians', problem.decision(1), 'acceleration', problem.decision(2));
end

function probe = localProbe(problem, model, sample)
    qp = problem.qp;
    clf = qp.clf;
    prediction = problem.prediction;
    p = clf.lyapunovMatrix;
    state = prediction.egoStateOffset(:, 1);
    error = clf.errorOffset(:, 1);
    reference = state(2:6)-error;
    lateral = [1, 2, 4, 5];
    assert(sample.relaxation <= 1e-8, 'Selected point is not on the zero-slack optimal face.');
    [a, b, affine] = ltvBicycleModel.continuousMatrices( ...
        prediction.scheduleCurvature(1), prediction.scheduleSpeedProfile(1), model.cfg);
    gradient = 2*error(lateral).'*p(lateral, lateral);
    drift = a*state+affine;
    lateralInput = gradient*b([2, 3, 5, 6], :);
    lateralDrift = gradient*drift([2, 3, 5, 6]);
    matrix = [qp.inequalityMatrix; clf.inequalityMatrix];
    bound = [qp.inequalityBound; clf.inequalityBound];
    lower = qp.lowerBound;
    upper = qp.upperBound;
    lower(end) = 0;
    upper(end) = 0;
    options = optimoptions('linprog', 'Display', 'none', ...
        'ConstraintTolerance', 1e-9, 'OptimalityTolerance', 1e-9);
    directions = [1, 0; -1, 0; lateralInput];
    labels = ["minimumSteering", "maximumSteering", "minimumLateralDerivative", ...
        "independentLateralDecay", "minimumLateralDerivativeWithRelaxedTotalClf", ...
        "minimumPredictedLateralV"];
    alternatives = cell(1, numel(labels));
    for index = 1:numel(labels)
        cost = zeros(qp.layout.decisionCount, 1);
        extraMatrix = matrix;
        extraBound = bound;
        if index <= 3
            cost(1:2) = directions(index, :).';
        elseif index == 4
            row = zeros(1, qp.layout.decisionCount);
            row(1:2) = lateralInput;
            extraMatrix = [matrix; row];
            extraBound = [bound; -clf.decayRate*sample.lateralV-lateralDrift];
        elseif index == 5
            cost(1:2) = lateralInput.';
            extraMatrix = qp.inequalityMatrix;
            extraBound = qp.inequalityBound;
        end
        if index == 6
            [decision, flag] = localMinimumNextLateralValue(problem, reference, lateral);
        else
            [decision, ~, flag] = linprog(cost, extraMatrix, extraBound, ...
                qp.equalityMatrix, qp.equalityBound, lower, upper, options);
        end
        alternative = struct('label', labels(index), 'exitFlag', flag, ...
            'certified', false, 'input', [NaN; NaN], 'lateralDerivative', NaN, ...
            'totalDerivative', NaN, 'predictedNextV', NaN, 'predictedNextLateralV', NaN, ...
            'maximumResidual', NaN, 'clfRelaxation', NaN);
        if flag > 0
            if index == 5
                decision(end) = max(0, clf.lieDerivativeDrift ...
                    + clf.lieDerivativeInput*decision(1:2)+clf.decayRate*clf.initialValue);
            end
            check = solveHardCbfClf.certify(qp, prediction, model, decision);
            alternative.certified = check.accepted;
            alternative.input = decision(1:2);
            alternative.lateralDerivative = lateralDrift+lateralInput*decision(1:2);
            alternative.totalDerivative = clf.lieDerivativeDrift+clf.lieDerivativeInput*decision(1:2);
            alternative.clfRelaxation = decision(end);
            nextError = prediction.egoStateMatrix(2:6, :, 2)*decision(1:end-1) ...
                + prediction.egoStateOffset(2:6, 2)-reference;
            alternative.predictedNextV = nextError.'*p*nextError;
            alternative.predictedNextLateralV = nextError(lateral).'*p(lateral, lateral)*nextError(lateral);
            checkedUpper = upper;
            if index == 5, checkedUpper(end) = inf; end
            alternative.maximumResidual = max([0; extraMatrix*decision-extraBound; ...
                abs(qp.equalityMatrix*decision-qp.equalityBound); lower-decision; decision-checkedUpper]);
            assert(check.accepted && alternative.maximumResidual < 1e-6);
        end
        alternatives{index} = alternative;
    end
    referenceState = [0; reference];
    equilibriumResidual = a*referenceState+b*clf.equilibriumInput(:, 1)+affine;
    [referenceA, referenceB, referenceAffine] = ltvBicycleModel.continuousMatrices( ...
        prediction.scheduleCurvature(1), reference(3), model.cfg);
    referenceSpeedResidual = referenceA*referenceState ...
        + referenceB*clf.equilibriumInput(:, 1)+referenceAffine;
    slipIndices = find(qp.rowFamily == "tireSlip");
    rowsPerStage = numel(slipIndices)/prediction.stageCount;
    assert(rowsPerStage == floor(rowsPerStage));
    currentSlip = slipIndices(1:rowsPerStage);
    assert(max(abs(qp.inequalityMatrix(currentSlip, 3:end-1)), [], 'all') < 1e-13);
    [physicalInput, ~, physicalFlag] = linprog(lateralInput.', ...
        qp.inequalityMatrix(currentSlip, 1:2), qp.inequalityBound(currentSlip), ...
        [], [], qp.lowerBound(1:2), qp.upperBound(1:2), options);
    assert(physicalFlag > 0);
    physicalResidual = qp.inequalityMatrix(currentSlip, 1:2)*physicalInput ...
        - qp.inequalityBound(currentSlip);
    inverseP = p\eye(5);
    probe = struct('timeSeconds', sample.timeSeconds, 'recorded', sample, ...
        'decayRate', clf.decayRate, 'requiredLateralDerivative', -clf.decayRate*sample.lateralV, ...
        'alternatives', [alternatives{:}], 'reference', reference, ...
        'referenceStateDerivativeAtDiagnosticEquilibrium', equilibriumResidual, ...
        'referenceStateDerivativeAtReferenceSpeed', referenceSpeedResidual, ...
        'currentSlipRowCount', numel(currentSlip), ...
        'currentPhysicalActiveRowIndices', find(abs(physicalResidual) < 1e-7), ...
        'minimumLateralDerivativeWithOnlyCurrentPhysicalConstraints', lateralDrift+lateralInput*physicalInput, ...
        'currentPhysicalMinimizingInput', physicalInput, ...
        'sufficientVForLateralTolerance', 0.2^2/inverseP(1, 1), ...
        'sufficientVForHeadingTolerance', 0.02^2/inverseP(2, 2), ...
        'scheduleSpeed', prediction.scheduleSpeedProfile(1), 'scheduleCurvature', prediction.scheduleCurvature(1));
end

function [decision, flag] = localMinimumNextLateralValue(problem, reference, lateral)
% An optimal-face diagnostic in predicted state space, without an input target.
    qp = problem.qp;
    program = qp.stageProgram;
    stateMap = problem.prediction.egoStateMatrix(2:6, :, 2);
    stateOffset = problem.prediction.egoStateOffset(2:6, 2)-reference;
    map = [stateMap(lateral, :), zeros(numel(lateral), 1)];
    offset = stateOffset(lateral);
    weight = qp.clf.lyapunovMatrix(lateral, lateral);
    hessian = 2*map.'*weight*map;
    linear = 2*map.'*weight*offset;
    count = numel(program.q);
    physical = qp.layout.decisionCount;
    zeroSlack = sparse(1, physical, 1, 1, count);
    equalityCount = program.cones(1);
    matrix = [program.A(1:equalityCount, :); zeroSlack; program.A(equalityCount+1:end, :)];
    bound = [program.b(1:equalityCount); 0; program.b(equalityCount+1:end)];
    [point, information] = solveAvoidanceSocpMex( ...
        blkdiag(sparse(triu(hessian)), sparse(count-physical, count-physical)), ...
        [linear; zeros(count-physical, 1)], matrix, bound, ...
        program.cones+[1; 0], [1e-9; 1e-9; 400]);
    decision = point(1:physical);
    decision(end) = 0;
    flag = double(any(information.status == [1, 4]));
end

function localWriteJson(path, value)
    file = fopen(path, 'w');
    assert(file >= 0, 'Could not open diagnostic report.');
    cleanup = onCleanup(@() fclose(file));
    fprintf(file, '%s\n', jsonencode(value, PrettyPrint=true));
end
