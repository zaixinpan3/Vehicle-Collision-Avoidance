function summary=runOverlapAdmissionValidation(options)
%runOverlapAdmissionValidation Straight admission, recovery and frame timing.
% Warmup runs are saved separately, discard all states/commands, and do not
% advance the measured trials. Plant integration and truth audits are offline.
    arguments
        options.OutputDirectory (1,1) string
        options.SampleCount (1,1) double {mustBeInteger,mustBePositive} = 300
        options.WarmupCount (1,1) double {mustBeInteger,mustBeNonnegative} = 6
        options.IncludeEstimator (1,1) logical = true
    end
    if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
    previousThreads=maxNumCompThreads(1);
    cleanup=onCleanup(@()maxNumCompThreads(previousThreads));
    summary=struct('matlabVersion',string(version),'computationalThreads',1, ...
        'warmup',{{}},'controller',struct(),'joint',struct(), ...
        'scope',"Declared affine plant; complete periodic frames; startup saved separately; no reused commands");
    for repetition=1:options.WarmupCount
        directory=fullfile(options.OutputDirectory,"startup-"+string(repetition));
        summary.warmup{repetition}=localController("stationary",2,5,directory);
    end
    for scenario=["stationary","oncoming","crossing","cruise"]
        summary.controller.(scenario)=localController(scenario,options.SampleCount,.1, ...
            fullfile(options.OutputDirectory,"controller"));
    end
    qualified=all(structfun(@(item)item.runtimeQualified,summary.controller));
    if options.IncludeEstimator && qualified
        summary.joint.exactSensing=localJoint(options.SampleCount,.1,fullfile(options.OutputDirectory,"exact-sensing"),false);
        summary.joint.functional=localJoint(options.SampleCount,5,fullfile(options.OutputDirectory,"joint-functional"));
        % An independent reset/seed reproduces the same scene under the full
        % 100 ms gate. The functional run supplies no executable witness.
        summary.joint.periodic=localJoint(options.SampleCount,.1,fullfile(options.OutputDirectory,"joint-periodic"));
    else
        summary.joint.skipped=true;
        summary.joint.reason="Estimator disabled or controller runtime gate did not pass.";
    end
    file=fopen(fullfile(options.OutputDirectory,"validation-summary.json"),'w');assert(file>=0);
    fileCleanup=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(summary,PrettyPrint=true));
end

function item=localController(scenario,count,deadline,directory)
    failure="";
    try
        report=runExactStateRecursiveFeasibilityScenario(Scenario=scenario,SampleCount=count, ...
            DeadlineSeconds=deadline,OutputDirectory=directory);
    catch exception
        file=fullfile(directory,scenario+"-exact-state.mat");
        if ~isfile(file),rethrow(exception);end
        saved=load(file,'report');report=saved.report;failure=string(exception.message);
    end
    last=report.state(:,end);
    item=struct('completed',report.completed,'runtimeQualified',report.runtimeQualified, ...
        'executedHolds',report.executedHolds,'minimumSampledSeparationMargin',report.minimumSampledSeparationMargin, ...
        'maximumFrameMilliseconds',1000*report.runtime.maximumSeconds,'deadlineMisses',report.runtime.deadlineMisses, ...
        'finalTrackingError',last(2:6)-[0;0;8;0;0],'failure',failure,'artifact',fullfile(directory,scenario+"-exact-state.mat"));
end

function item=localJoint(count,deadline,directory,useEstimator)
    if nargin<4,useEstimator=true;end
    failure="";
    try
        report=runDeclaredPlantEstimatorControllerScenario(SampleCount=count,TargetInitialDistance=60, ...
            ReferenceSpeed=8,TargetLateralPosition=0,ConfirmationRange=16,DeadlineSeconds=deadline, ...
            Warmup=false,UseEstimator=useEstimator,OutputDirectory=directory);
    catch exception
        file=fullfile(directory,'joint-declared-plant.mat');
        if ~isfile(file),rethrow(exception);end
        saved=load(file,'report');report=saved.report;failure=string(exception.message);
    end
    item=struct('completed',report.completed,'executedHolds',report.executedHolds, ...
        'failureTime',report.failure.time,'maximumFrameMilliseconds',1000*max(report.frameSeconds), ...
        'maximumObserverMilliseconds',1000*max(report.observerSeconds), ...
        'deadlineMisses',nnz(report.frameSeconds>.1),'failure',failure, ...
        'artifact',fullfile(directory,'joint-declared-plant.mat'));
end
