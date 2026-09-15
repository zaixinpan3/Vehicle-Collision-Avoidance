function campaign = runStraightControllerRerun(options)
%runStraightControllerRerun Compare exact, bounded-noise, and actual NRMM trials.
% Runs the declared-affine-plant drivers sequentially. Failed admission stops
% its trial; the remaining independent trials still run. Timing includes
% failed and final unexecuted decisions. Overruns are measured, not applied
% as actuation delay. Generated artifacts go to OutputDirectory, not scripts.
    arguments
        options.SampleCount (1,1) double {mustBeInteger,mustBePositive} = 300
        options.Seed (1,1) double {mustBeInteger,mustBeNonnegative} = 20260914
        options.OutputDirectory (1,1) string = fullfile(tempdir,"straight-controller-rerun")
    end
    if ~isfolder(options.OutputDirectory), mkdir(options.OutputDirectory); end
    campaign = struct('options',options,'trials',{{}},'summary',struct([]), ...
        'scope',"Declared affine ego plant; sequential trials; measured computation is not applied as delay");
    for noisy = [false,true]
        for scene = ["stationary","oncoming","crossing"]
            label = "exact-"+scene;
            extra = {};
            if noisy
                label = "bounded-"+scene;
                extra = {'EgoErrorBound',[.05;.05;.005;.05;.02;.005], ...
                    'TargetErrorBound',[.1;.1;.1;.1;.05;.05;.01;.01], ...
                    'TargetJerkAmplitude',[.1;.1],'TargetYawAccelerationAmplitude',.05};
            end
            fprintf('\nStarting %s\n',label);
            directory=fullfile(options.OutputDirectory,label);
            try
                report=runExactStateRecursiveFeasibilityScenario('Scenario',scene, ...
                    'SampleCount',options.SampleCount,'Seed',options.Seed,'OutputDirectory',directory,extra{:});
            catch exception
                if ~startsWith(string(exception.identifier),'collisionAvoidanceController:'),rethrow(exception);end
                saved=load(fullfile(directory,scene+"-exact-state.mat"),'report');report=saved.report;
            end
            campaign = localAppend(campaign,label,report,false);
        end
    end
    for useEstimator = [false,true]
        label = "range-exact-oncoming";
        if useEstimator, label = "range-nrmm-oncoming"; end
        fprintf('\nStarting %s\n',label);
        directory=fullfile(options.OutputDirectory,label);
        try
            report=runDeclaredPlantEstimatorControllerScenario(SampleCount=options.SampleCount, ...
                Seed=options.Seed,UseEstimator=useEstimator,OutputDirectory=directory);
        catch exception
            if ~startsWith(string(exception.identifier),'collisionAvoidanceController:'),rethrow(exception);end
            saved=load(fullfile(directory,'joint-declared-plant.mat'),'report');report=saved.report;
        end
        campaign = localAppend(campaign,label,report,true);
    end
end

function campaign = localAppend(campaign,label,report,rangeDriver)
    row = struct('trial',label,'completed',report.completed,'driverSafetyPassed',NaN, ...
        'executedHolds',report.executedHolds,'frames',1,'finalTime',0, ...
        'minimumSeparationMargin',NaN,'minimumRoadMargin',NaN, ...
        'minimumModelDomainMargin',NaN,'truthContained',NaN, ...
        'finalSpeed',NaN,'finalLateralError',NaN,'finalHeadingError',NaN, ...
        'admissionSeconds',NaN,'medianSeconds',NaN,'maximumSeconds',NaN, ...
        'maximumSuccessorSeconds',NaN,'deadlineMisses',NaN,'terminalCommands',0, ...
        'releaseTime',NaN,'failureIdentifier',"",'failureMessage',"");
    if rangeDriver
        times = report.frameSeconds;
        row.minimumSeparationMargin = report.minimumSeparationMargin;
        row.minimumRoadMargin = report.minimumRoadMargin;
        row.failureIdentifier = report.failure.identifier;
        row.failureMessage = report.failure.message;
    else
        row.driverSafetyPassed = report.passed;
        row.failureIdentifier = report.failureIdentifier;
        row.failureMessage = report.failureMessage;
        if isfield(report,'runtime')
            times = report.runtime.frameSeconds;
            row.minimumSeparationMargin = report.minimumSampledSeparationMargin;
            row.minimumRoadMargin = report.minimumSampledRoadMargin;
            row.minimumModelDomainMargin = report.minimumSampledModelDomainMargin;
        else
            times = report.admissionSeconds;
        end
    end
    row.frames = numel(times);
    row.admissionSeconds = times(1);
    row.medianSeconds = median(times);
    row.maximumSeconds = max(times);
    row.deadlineMisses = nnz(times>0.1);
    if numel(times)>1, row.maximumSuccessorSeconds = max(times(2:end)); end
    if isfield(report,'state')
        row.finalSpeed = report.state(4,end);
        row.finalLateralError = report.state(2,end);
        row.finalHeadingError = report.state(3,end);
        row.finalTime = report.time(end);
    end
    campaign.trials{end+1} = report;
    if isempty(campaign.summary)
        campaign.summary = row;
    else
        campaign.summary(end+1) = row;
    end
    save(fullfile(campaign.options.OutputDirectory,'campaign.mat'),'campaign','-v7.3');
    writetable(struct2table(campaign.summary),fullfile(campaign.options.OutputDirectory,'summary.csv'));
    summary = struct('options',campaign.options,'scope',campaign.scope,'trials',campaign.summary);
    fid = fopen(fullfile(campaign.options.OutputDirectory,'summary.json'),'w');
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));
    fprintf('%s: completed=%d, holds=%d, maximum frame=%.6f s, misses=%d\n', ...
        label,row.completed,row.executedHolds,row.maximumSeconds,row.deadlineMisses);
end
