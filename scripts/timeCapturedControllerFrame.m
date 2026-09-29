function runs = timeCapturedControllerFrame(captureFile, instrumentedDirectory, outputDirectory)
%timeCapturedControllerFrame Measure isolated solver phases and convergence.
% Generate the instrumented solver copy with instrumentControllerFrameTiming.py
% first. Run one warmup, three fixed-work repeats and one deadline-free solve
% with the original iteration budget. This changes no production source.
    arguments
        captureFile (1,1) string
        instrumentedDirectory (1,1) string
        outputDirectory (1,1) string
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    previousPath = path;
    cleanup = onCleanup(@()path(previousPath));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    addpath(instrumentedDirectory,'-begin');
    assert(string(which('solvePredictiveControl')) == ...
        fullfile(instrumentedDirectory,'solvePredictiveControl.m'), ...
        'The isolated instrumented solver must precede production on the path.');
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    data = load(captureFile,'capture');
    capture = data.capture;
    records = cell(1,5);
    for index = 1:numel(records)
        cfg = capture.fixedWorkConfiguration;
        if index == 5
            cfg.nonlinear.maximumIterations = capture.configuration.nonlinear.maximumIterations;
        end
        timer = tic;
        [~,~,problem] = collisionAvoidanceController(capture.ego,capture.target, ...
            capture.road,cfg,capture.previousState);
        elapsed = toc(timer);
        trajectoryError = NaN;
        if index <= 4
            trajectoryError = max(abs(problem.inputTrajectory - ...
                capture.fixedWorkSolution.inputs),[],'all');
            assert(trajectoryError < 1e-9,'Instrumentation changed the recovered plan.');
            assert(problem.metadata.solverCallCount == capture.originalTrace.solverCalls, ...
                'Instrumentation changed the solver work.');
        end
        records{index} = struct('seconds',elapsed,'trajectoryError',trajectoryError, ...
            'search',problem.metadata.search,'safetySlack',problem.solution.safety, ...
            'hardResidual',problem.solution.hard,'clfSlack',problem.solution.clfSlack, ...
            'firstInput',problem.inputTrajectory(:,1));
        fprintf('Instrumented run %d: %.3f s, %d solver calls, termination %s\n', ...
            index,elapsed,problem.metadata.solverCallCount,problem.metadata.search.terminationReason);
    end
    runs = [records{:}];
    localWrite(fullfile(outputDirectory,'timed-runs.json'),runs);
    findings = checkcode(fullfile(instrumentedDirectory,'solvePredictiveControl.m'), ...
        '-config=factory','-id');
    localWrite(fullfile(outputDirectory,'instrumented-analysis.json'),findings);
    localWrite(fullfile(outputDirectory,'timing-helper-analysis.json'), ...
        checkcode(mfilename('fullpath'),'-config=factory','-id'));
end

function localWrite(filePath,value)
    file = fopen(filePath,'w');
    assert(file >= 0);
    cleanup = onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(value,PrettyPrint=true));
end
