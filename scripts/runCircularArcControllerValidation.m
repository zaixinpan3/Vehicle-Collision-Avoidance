function summary = runCircularArcControllerValidation(options)
%runCircularArcControllerValidation Exact-state circular-arc controller trials.
% Targets are stationary or move along inertial straight lines. The oncoming
% line is tangent to the arc at s=30 m; the crossing line is normal at s=15 m.
% Startup trials are discarded before independent 100 ms periodic trials.
% A failed periodic trial is repeated independently with an offline budget to
% distinguish certificate/search failure from deadline failure.
    arguments
        options.OutputDirectory (1,1) string
        options.SampleCount (1,1) double {mustBeInteger,mustBePositive} = 300
        options.Curvatures (1,:) double {mustBeFinite} = [.01,-.01,.02,-.02]
        options.Scenarios (1,:) string = ["cruise","stationary","oncoming","crossing"]
        options.WarmupCount (1,1) double {mustBeInteger,mustBeNonnegative} = 6
        options.InitialTrackingError (5,1) double {mustBeFinite} = zeros(5,1)
    end
    if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
    previousThreads = maxNumCompThreads(1);
    cleanup = onCleanup(@()maxNumCompThreads(previousThreads));
    summary = struct('matlabVersion',string(version),'computationalThreads',1, ...
        'sampleTime',.1,'referenceSpeed',8,'roadBoundariesEnabled',false, ...
        'estimatorEnabled',false,'seed',20260912,'startup',{{}},'trials',{{}}, ...
        'scope',"Exact declared affine plant and sensing; curved-reference trim; targets follow inertial straight lines; offline truth integration excluded from frame time");
    for repetition = 1:options.WarmupCount
        directory = fullfile(options.OutputDirectory,"startup-"+string(repetition));
        summary.startup{end+1} = localTrial("stationary",.01,2,5,directory);
    end
    for curvature = options.Curvatures
        for scenario = options.Scenarios
            directory = fullfile(options.OutputDirectory,sprintf('kappa-%g',curvature),"periodic");
            trial = localTrial(scenario,curvature,options.SampleCount,.1,directory,options.InitialTrackingError);
            if ~trial.runtimeQualified
                directory = fullfile(options.OutputDirectory,sprintf('kappa-%g',curvature),"diagnostic");
                trial.offlineDiagnostic = localTrial(scenario,curvature,options.SampleCount,5,directory,options.InitialTrackingError);
            end
            summary.trials{end+1} = trial;
            localSave(summary,options.OutputDirectory);
        end
    end
    summary.allRuntimeQualified = all(cellfun(@(trial)trial.runtimeQualified,summary.trials));
    localSave(summary,options.OutputDirectory);
end

function item = localTrial(scenario,curvature,count,deadline,directory,initialError)
    if nargin<6,initialError=zeros(5,1);end
    try
        report = runExactStateRecursiveFeasibilityScenario(Scenario=scenario,RoadCurvature=curvature, ...
            SampleCount=count,DeadlineSeconds=deadline,OutputDirectory=directory,InitialTrackingError=initialError);
    catch exception
        artifact = fullfile(directory,scenario+"-exact-state.mat");
        if ~isfile(artifact),rethrow(exception);end
        saved = load(artifact,'report');report = saved.report;
    end
    tail = report.time>=max(0,report.time(end)-2);
    item = struct('scenario',scenario,'curvature',curvature,'radiusMeters',1/abs(curvature), ...
        'initialTrackingError',initialError, ...
        'completed',report.completed,'passed',report.passed,'runtimeQualified',report.runtimeQualified, ...
        'executedHolds',report.executedHolds,'durationSeconds',report.time(end), ...
        'maximumFrameMilliseconds',1000*report.runtime.maximumSeconds, ...
        'medianFrameMilliseconds',1000*report.runtime.medianSeconds, ...
        'deadlineMisses',report.runtime.deadlineMisses, ...
        'minimumSampledSeparationMargin',report.minimumSampledSeparationMargin, ...
        'maximumLateralError',max(abs(report.trackingError(1,:))), ...
        'finalTrackingError',report.trackingError(:,end), ...
        'lastTwoSecondsMaximumError',max(abs(report.trackingError(:,tail)),[],2), ...
        'allIssuedCommandsCertified',all(report.hardCertificateVerified), ...
        'confirmedRelease',any(report.confirmedRelease), ...
        'terminalCommands',nnz(report.terminalCommands),'failureTime',report.failureTime, ...
        'failureIdentifier',report.failureIdentifier,'failureMessage',report.failureMessage, ...
        'artifact',fullfile(directory,scenario+"-exact-state.mat"));
end

function localSave(summary,directory)
    file = fopen(fullfile(directory,'arc-validation-summary.json'),'w');assert(file>=0);
    cleanup = onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(summary,PrettyPrint=true));
end
