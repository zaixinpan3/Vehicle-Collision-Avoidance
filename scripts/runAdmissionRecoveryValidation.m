function summary = runAdmissionRecoveryValidation(options)
%runAdmissionRecoveryValidation Compare safety and full-frame timing after admission repair.
% Diagnostic-budget runs and strict 50 ms runs are recorded separately. Failed
% attempts remain in the timing totals; no offline audit selects a command.
    arguments
        options.OutputDirectory (1,1) string
        options.SampleCount (1,1) double {mustBeInteger,mustBePositive} = 600
        options.Repetitions (1,1) double {mustBeInteger,mustBePositive} = 2
        options.WarmupCount (1,1) double {mustBeInteger,mustBeNonnegative} = 3
        options.Curvatures (1,:) double {mustBeFinite} = [0,.01]
        options.Scenarios (1,:) string = ["stationary","oncoming","crossing"]
        options.StrictDeadline (1,1) logical = true
    end
    if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
    summary=struct('matlabVersion',string(version),'sampleTimeSeconds',.05, ...
        'diagnosticBudgetSeconds',30,'seed',20260912,'options',options, ...
        'warmup',{{}},'diagnostic',{{}},'strict',{{}}, ...
        'scope',"Exact held affine plant and exact sensing; node certificates and independent sampled body-gap audit; measured maxima are not WCET");
    for repetition=1:options.WarmupCount
        folder=fullfile(options.OutputDirectory,"warmup-"+repetition);
        summary.warmup{end+1}=localTrial("crossing",.01,140,30,folder);
    end
    for repetition=1:options.Repetitions
        for curvature=options.Curvatures
            for scenario=options.Scenarios
                label=sprintf('repeat-%d-kappa-%g-%s',repetition,curvature,scenario);
                folder=fullfile(options.OutputDirectory,'diagnostic',label);
                summary.diagnostic{end+1}=localTrial(scenario,curvature,options.SampleCount,30,folder);
                localSave(summary,options.OutputDirectory);
            end
        end
    end
    if options.StrictDeadline
        for repetition=1:options.Repetitions
            folder=fullfile(options.OutputDirectory,'strict',"repeat-"+repetition);
            summary.strict{end+1}=localTrial("crossing",.01,options.SampleCount,.05,folder);
            localSave(summary,options.OutputDirectory);
        end
    end
    localSave(summary,options.OutputDirectory);
end

function item=localTrial(scenario,curvature,count,deadline,directory)
    artifact=fullfile(directory,scenario+"-exact-state.mat");
    try
        report=runExactStateRecursiveFeasibilityScenario(Scenario=scenario, ...
            RoadCurvature=curvature,SampleCount=count,SampleTime=.05, ...
            DeadlineSeconds=deadline,SearchTimeLimitSeconds=30,OutputDirectory=directory);
    catch exception
        if ~isfile(artifact),rethrow(exception);end
        saved=load(artifact,'report');report=saved.report;
    end
    isAdmission=~report.inheritedFeasibleFamily & report.obstacleCbfRowCount>0;
    isContinuation=report.inheritedFeasibleFamily;
    times=report.runtime.frameSeconds;
    issuedTimes=times(1:report.executedHolds);
    fullPlan=cellfun(@localFullPlan,report.admissionSearch);
    item=struct('scenario',scenario,'curvature',curvature,'budgetSeconds',deadline, ...
        'requestedHolds',count,'executedHolds',report.executedHolds,'completed',report.completed, ...
        'passed',report.passed,'runtimeQualified',report.runtimeQualified, ...
        'minimumSampledBodyGap',report.minimumSampledBodyGap, ...
        'minimumNodeBodyGap',report.minimumNodeBodyGap, ...
        'minimumSampledModelDomainMargin',report.minimumSampledModelDomainMargin, ...
        'maximumTrackingError',max(abs(report.trackingError),[],2), ...
        'allIssuedCommandsCertified',all(report.hardCertificateVerified), ...
        'confirmedRelease',any(report.confirmedRelease),'fullPlanAdmissions',nnz(fullPlan), ...
        'admissionFrameSeconds',issuedTimes(isAdmission), ...
        'continuationFrameSeconds',issuedTimes(isContinuation), ...
        'frameSeconds',times,'maximumFrameSeconds',report.runtime.maximumSeconds, ...
        'medianFrameSeconds',report.runtime.medianSeconds,'deadlineMisses',report.runtime.deadlineMisses, ...
        'failureIdentifier',report.failureIdentifier,'failureMessage',report.failureMessage, ...
        'failureTime',report.failureTime,'artifact',artifact);
end

function value=localFullPlan(search)
    value=isfield(search,'usedFullPlanAdmission') && search.usedFullPlanAdmission;
end

function localSave(summary,directory)
    handle=fopen(fullfile(directory,'admission-recovery-summary.json'),'w');assert(handle>=0);
    cleanup=onCleanup(@()fclose(handle));
    fprintf(handle,'%s\n',jsonencode(summary,PrettyPrint=true));
end
