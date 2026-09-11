function report = benchmarkControllerRuntime(trial, options)
%benchmarkControllerRuntime Replay recorded controller inputs with wall timing.
% Uses the original measurements, target acquisition, road fits and settings.
% Each pass resets certificate memory, then carries its own certified plan.
% This measures the controller, not perception or the simulation plant, and
% is not a closed-loop experiment. The second input is dimensionless beta;
% gross longitudinal acceleration is recorded separately in m/s^2.
% The first pass is reported separately;
% every subsequent sample is retained, including deadline misses. Explicit
% preparation precedes all timed online samples, including sample one. The
% requested thread limit is restored on return. Disable preparation and use
% the original thread count when measuring cold-start behavior.
%
%   data = load("straight_avoidance.mat");
%   report = benchmarkControllerRuntime(data.result, Repetitions=3);

    arguments
        trial (1,1) struct
        options.Repetitions (1,1) double {mustBeInteger, mustBePositive} = 3
        options.OutputDirectory (1,1) string = ""
        options.PrepareController (1,1) logical = true
        options.ComputationalThreads (1,1) double {mustBeInteger, mustBePositive} = 1
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root, "controller"), fullfile(root, "config"));
    previousThreads = maxNumCompThreads(options.ComputationalThreads);
    threadCleanup = onCleanup(@() maxNumCompThreads(previousThreads));
    count = numel(trial.command);
    assert(count > 0, "benchmarkControllerRuntime:emptyTrial", "No controller inputs were recorded.");
    rows = zeros(count*options.Repetitions, 15);
    algorithms = strings(size(rows, 1), 1);
    preparation = struct("performed", false, "elapsedSeconds", 0.0);
    if options.PrepareController
        preparation = prepareCollisionAvoidanceController( ...
            trial.controllerEgoEstimate{1}, ...
            trial.perception.roadBoundaryFit{1}.roadGeometry, trial.controllerConfiguration,trial.targetEstimate{1});
    end
    for repetition = 1:options.Repetitions
        certificate = [];
        for sample = 1:count
            ego = trial.controllerEgoEstimate{sample};
            target = trial.targetEstimate{sample};
            road = trial.perception.roadBoundaryFit{sample}.roadGeometry;
            timer = tic;
            [command, ~, problem, certificate] = collisionAvoidanceController( ...
                ego, target, road, trial.controllerConfiguration, certificate);
            elapsed = toc(timer);
            assert(problem.metadata.planCertified, ...
                "benchmarkControllerRuntime:uncertified", "Replay returned an uncertified plan.");
            phase = problem.metadata.runtime;
            index = (repetition-1)*count+sample;
            rows(index, :) = [repetition, sample, trial.controlTime(sample), elapsed, ...
                phase.inputPreparationSeconds, phase.predictionSeconds, ...
                phase.formulationAndWitnessSeconds, phase.solveSeconds, ...
                phase.acceptanceAndCommitSeconds, phase.diagnosticsSeconds, ...
                problem.metadata.solverCallCount, problem.metadata.fallbackUsed, ...
                command.actuatorInput.', command.longitudinalAcceleration];
            algorithms(index) = problem.metadata.solverAlgorithm;
        end
        fprintf("Replay %d/%d: median %.3f ms, maximum %.3f ms\n", repetition, ...
            options.Repetitions, 1000*median(rows(index-count+1:index, 4)), ...
            1000*max(rows(index-count+1:index, 4)));
    end
    samples = array2table(rows, VariableNames=["repetition", "sample", "simulationTime", ...
        "elapsedSeconds", "preparationSeconds", "predictionSeconds", ...
        "formulationSeconds", "solveSeconds", "acceptanceSeconds", "diagnosticsSeconds", ...
        "solverCalls", "fallbackUsed", "steeringRadians", "brakingRatio", ...
        "accelerationMetersPerSecondSquared"]);
    samples.algorithm = algorithms;
    samples.deadlineMiss = samples.elapsedSeconds > trial.controllerConfiguration.controller.sampleTime;
    recorded = [trial.command{:}];
    recordedAcceleration = [recorded.longitudinalAcceleration].';
    recorded = [recorded.actuatorInput];
    samples.steeringDifferenceRadians = samples.steeringRadians ...
        - repmat(recorded(1, :).', options.Repetitions, 1);
    samples.brakingRatioDifference = samples.brakingRatio ...
        - repmat(recorded(2, :).', options.Repetitions, 1);
    samples.accelerationDifferenceMetersPerSecondSquared = samples.accelerationMetersPerSecondSquared ...
        - repmat(recordedAcceleration, options.Repetitions, 1);
    summary = zeros(options.Repetitions, 9);
    for repetition = 1:options.Repetitions
        selected = samples.repetition == repetition;
        timing = samples.elapsedSeconds(selected);
        summary(repetition, :) = [repetition, numel(timing), median(timing), ...
            prctile(timing, 95), max(timing), nnz(samples.deadlineMiss(selected)), ...
            max(abs(samples.steeringDifferenceRadians(selected))), ...
            max(abs(samples.brakingRatioDifference(selected))), ...
            max(abs(samples.accelerationDifferenceMetersPerSecondSquared(selected)))];
    end
    summary = array2table(summary, VariableNames=["repetition", "samples", ...
        "medianSeconds", "percentile95Seconds", "maximumSeconds", "deadlineMisses", ...
        "maximumSteeringDifferenceRadians", "maximumBrakingRatioDifference", ...
        "maximumAccelerationDifferenceMetersPerSecondSquared"]);
    report = struct("samples", samples, "summary", summary, "matlabVersion", string(version), ...
        "computer", string(computer), "controllerPath", string(which("collisionAvoidanceController")), ...
        "sampleTime", trial.controllerConfiguration.controller.sampleTime, ...
        "preparation", preparation, "computationalThreads", maxNumCompThreads, ...
        "scope", "Recorded-input replay; first pass separate; no hard real-time guarantee");
    if strlength(options.OutputDirectory) > 0
        if ~isfolder(options.OutputDirectory), mkdir(options.OutputDirectory); end
        writetable(samples, fullfile(options.OutputDirectory, "runtime_samples.csv"));
        writetable(summary, fullfile(options.OutputDirectory, "runtime_summary.csv"));
        save(fullfile(options.OutputDirectory, "runtime_report.mat"), "report");
    end
end
