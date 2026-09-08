function report = runStraightRealtimeValidation(options)
%runStraightRealtimeValidation Gate the straight physical experiments in order.
% Each complete online frame must finish within DeadlineSeconds. A failed
% solve or expired frame stops before its command reaches the plant. The
% joint experiment starts only after the controller-only experiment passes
% collision, road, completion, eventual-cruise and runtime checks.
% Offline compilation/preparation and simulated plant integration are not
% online computations. The controller model still uses a 0.05 s interval;
% this experiment does not model computation-induced actuation delay.
    arguments
        options.Duration (1,1) double {mustBePositive} = 30
        options.DeadlineSeconds (1,1) double {mustBePositive} = 0.1
        options.RandomSeed (1,1) double {mustBeInteger,mustBeNonnegative} = 20260907
        options.OutputDirectory (1,1) string = ""
        options.Progress (1,1) logical = true
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"config"));
    cfg = finiteSensingValidationConfig();
    estimator = estimatorControllerIntegrationConfig();
    estimator.randomSeed = options.RandomSeed;
    % One RK4 step per held sensor interval. The runtime still includes
    % the a posteriori integration defect in its complete error enclosure.
    estimator.observer.runtime.integrationStepMaximum = estimator.observer.runtime.samplePeriod;
    report = struct("passed",false,"controllerOnly",[],"joint",[], ...
        "deadlineSeconds",options.DeadlineSeconds,"randomSeed",options.RandomSeed, ...
        "scope","Straight PassVeh14DOF; all attempted online frames; no delay-in-the-loop or platform WCET proof");
    report.execution = struct("matlabVersion",string(version), ...
        "architecture",string(computer('arch')),"computationalThreads",maxNumCompThreads, ...
        "allowedCpuList","");
    if isfile('/proc/self/status')
        affinity = regexp(fileread('/proc/self/status'),'Cpus_allowed_list:\s*([^\n]+)','tokens','once');
        if ~isempty(affinity),report.execution.allowedCpuList = string(strtrim(affinity{1}));end
    end
    report.controllerOnly = localRun(false,cfg,estimator,options);
    localSave(report,options.OutputDirectory);
    if ~report.controllerOnly.passed || ~report.controllerOnly.runtime.deadlineMet
        return;
    end
    report.joint = localRun(true,cfg,estimator,options);
    report.passed = report.joint.passed && report.joint.runtime.deadlineMet;
    localSave(report,options.OutputDirectory);
end

function trial = localRun(estimated,cfg,estimator,options)
    trial = runOncomingVehicleAvoidanceScenario( ...
        Duration=options.Duration,CenterlineLengthAfter=1000, ...
        ReferenceSpeed=10,TargetSpeed=10,TargetInitialLongitudinalDistance=100, ...
        TargetLateralOffset=0.8,TargetWidth=2,UseStateEstimator=estimated, ...
        EstimatorConfiguration=estimator,ControllerConfiguration=cfg, ...
        DeadlineSeconds=options.DeadlineSeconds,EnforceRuntimeDeadline=true, ...
        Plot=false,Report=true,Progress=options.Progress);
end

function localSave(report,directory)
    if strlength(directory)==0,return;end
    if ~isfolder(directory),mkdir(directory);end
    save(fullfile(directory,"straight-realtime-validation.mat"),"report","-v7.3");
end
