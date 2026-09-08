function report = analyzePipelineRuntime(jointResultPath,delayedResultPath,outputDirectory)
%analyzePipelineRuntime Attribute recorded frames and profile current replays.
% Historical joint timing and current delayed-controller timing retain their
% separate configurations. Replays apply no commands and are not a new joint
% closed-loop validation. Profiled durations are not deadline measurements.
    arguments
        jointResultPath (1,1) string
        delayedResultPath (1,1) string
        outputDirectory (1,1) string
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'estimator'), ...
        fullfile(root,'solver','bicycle'),fullfile(root,'solver','nrmm'));
    threads = maxNumCompThreads(1);
    cleanup = onCleanup(@() maxNumCompThreads(threads));
    status = profile('status');
    assert(string(status.ProfilerStatus)=="off", ...
        'analyzePipelineRuntime:profilerInUse','Stop the existing profile before this analysis.');
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    data = load(jointResultPath,'report');joint = data.report.joint;
    data = load(delayedResultPath,'report');delayed = data.report.controllerOnly;
    assert(~isempty(joint) && delayed.controllerConfiguration.controller.inputDelaySteps==1);
    historical = localRecordedFrames(joint);
    current = localRecordedFrames(delayed);
    writetable(historical,fullfile(outputDirectory,'historical-joint-frames.csv'));
    writetable(current,fullfile(outputDirectory,'current-delayed-controller-frames.csv'));
    report = struct('matlabVersion',string(version),'jointResultPath',jointResultPath, ...
        'delayedResultPath',delayedResultPath,'historicalJoint',localSummary(historical), ...
        'currentDelayedController',localSummary(current));
    report.controllerReplay = localControllerReplay(delayed,outputDirectory);
    report.observerReplay = localObserverReplay(joint,delayed.controllerConfiguration.controller.sampleTime,outputDirectory);
    report.scope = "Recorded frame attribution and current-code component replays; no controls applied, no new joint closed loop, no WCET claim";
    save(fullfile(outputDirectory,'pipeline-analysis.mat'),'report','historical','current','-v7.3');
    localJson(fullfile(outputDirectory,'pipeline-analysis.json'),report);
end

function rows = localRecordedFrames(trial)
    runtime = trial.runtime;attempts = trial.attempts;
    count = numel(runtime.frameSeconds);
    parts = zeros(count,5);solves = zeros(count,1);refinements = solves;
    names = ["inputPreparationSeconds","predictionSeconds","formulationAndWitnessSeconds", ...
        "solveSeconds","acceptanceAndCommitSeconds"];
    for frame = 1:count
        metadata = attempts.metadata{frame};
        if isempty(metadata),parts(frame,:) = NaN;continue;end
        for part = 1:numel(names),parts(frame,part) = metadata.runtime.(names(part));end
        solves(frame) = metadata.solverCallCount;
        refinements(frame) = metadata.nominalRefinementCount;
    end
    otherController = runtime.controllerSeconds-sum(parts,2);
    otherFrame = runtime.frameSeconds-runtime.controllerSeconds-runtime.observerSeconds-runtime.roadFitSeconds;
    rows = table(attempts.time,runtime.frameSeconds,runtime.observerSeconds,runtime.roadFitSeconds, ...
        runtime.controllerSeconds,parts(:,1),parts(:,2),parts(:,3),parts(:,4),parts(:,5), ...
        otherController,otherFrame,solves,refinements,attempts.targetVisible, ...
        'VariableNames',{'Time','Frame','Observer','RoadFit','Controller','InputPreparation', ...
        'InitialPrediction','Formulation','NumericalSolve','Acceptance','OtherController','OtherFrame', ...
        'SolverCalls','NominalRefinements','TargetVisible'});
end

function summary = localSummary(rows)
    names = string(rows.Properties.VariableNames(2:12));
    values = rows{:,2:12};
    [peak,frame] = max(rows.Frame);
    summary = struct('frameCount',height(rows),'componentNames',names, ...
        'meanSeconds',mean(values,1),'medianSeconds',median(values,1), ...
        'percentile95Seconds',prctile(values,95,1),'maximumSeconds',max(values,[],1), ...
        'peakFrameSeconds',peak,'peakFrameTime',rows.Time(frame),'peakFrame',table2struct(rows(frame,:)), ...
        'summedTimeShare',sum(values,1)/sum(rows.Frame), ...
        'negativeUnattributedControllerFrames',nnz(rows.OtherController < -1e-6));
    groups = {true(height(rows),1),rows.TargetVisible,~rows.TargetVisible};
    labels = ["all","targetVisible","targetAbsent"];
    summary.groups = struct();
    for group = 1:numel(groups)
        selected = rows(groups{group},:);
        summary.groups.(labels(group)) = struct('frames',height(selected), ...
            'meanSeconds',mean(selected{:,2:12},1),'medianSeconds',median(selected{:,2:12},1), ...
            'maximumSeconds',max(selected{:,2:12},[],1));
    end
end

function result = localControllerReplay(trial,directory)
    cfg = trial.controllerConfiguration;attempts = trial.attempts;
    [~,peak] = max(trial.runtime.frameSeconds);
    prepareCollisionAvoidancePipeline(attempts.controllerEgoEstimate{1}, ...
        attempts.roadPerception{1}.roadGeometry,cfg,struct());
    stored = [];beforePeak = [];timings = zeros(peak,1);difference = timings;
    for frame = 1:peak
        if frame==peak,beforePeak = stored;end
        timer = tic;
        [command,~,problem,stored] = collisionAvoidanceController( ...
            attempts.controllerEgoEstimate{frame},attempts.targetEstimate{frame}, ...
            attempts.roadPerception{frame}.roadGeometry,cfg,stored);
        timings(frame) = toc(timer);
        difference(frame) = norm(command.actuatorInput-attempts.computedCommand{frame}.actuatorInput,Inf);
        assert(difference(frame)==0,'analyzePipelineRuntime:replayMismatch', ...
            'The recorded scheduled command did not reproduce exactly.');
    end
    repeated = zeros(20,1);repeatParts = zeros(20,5);
    names = ["inputPreparationSeconds","predictionSeconds","formulationAndWitnessSeconds", ...
        "solveSeconds","acceptanceAndCommitSeconds"];
    for repetition = 1:numel(repeated)
        timer = tic;
        [command,~,problem] = collisionAvoidanceController(attempts.controllerEgoEstimate{peak}, ...
            attempts.targetEstimate{peak},attempts.roadPerception{peak}.roadGeometry,cfg,beforePeak);
        repeated(repetition) = toc(timer);
        assert(isequal(command.actuatorInput,attempts.computedCommand{peak}.actuatorInput));
        for part = 1:numel(names),repeatParts(repetition,part) = problem.metadata.runtime.(names(part));end
    end
    profile clear;profile on -timer real;
    profileCleanup = onCleanup(@() profile('off'));
    [command,~,profiledProblem] = collisionAvoidanceController(attempts.controllerEgoEstimate{peak}, ...
        attempts.targetEstimate{peak},attempts.roadPerception{peak}.roadGeometry,cfg,beforePeak);
    profile off;stats = profile('info');
    assert(isequal(command.actuatorInput,attempts.computedCommand{peak}.actuatorInput));
    profileRows = localProfileRows(stats);
    writetable(profileRows,fullfile(directory,'current-controller-profile.csv'));
    save(fullfile(directory,'current-controller-replay.mat'),'beforePeak','timings','difference', ...
        'repeated','repeatParts','stats','profiledProblem','-v7.3');
    writetable(table((1:20).',repeated,repeatParts(:,1),repeatParts(:,2),repeatParts(:,3),repeatParts(:,4),repeatParts(:,5), ...
        'VariableNames',{'Repetition','Controller','InputPreparation','InitialPrediction','Formulation','NumericalSolve','Acceptance'}), ...
        fullfile(directory,'current-controller-repeated.csv'));
    result = struct('recordedFrameTime',attempts.time(peak),'recordedControllerSeconds',trial.runtime.controllerSeconds(peak), ...
        'reproducedFrames',peak,'maximumCommandDifference',max(difference), ...
        'sequentialReplaySeconds',timings,'repetitionCount',numel(repeated), ...
        'repeatedMedianSeconds',median(repeated),'repeatedMinimumSeconds',min(repeated),'repeatedMaximumSeconds',max(repeated), ...
        'repeatComponentNames',names,'repeatComponentMedianSeconds',median(repeatParts,1), ...
        'profile',table2struct(profileRows(1:min(20,height(profileRows)),:)), ...
        'solverCalls',profiledProblem.metadata.solverCallCount,'nominalRefinements',profiledProblem.metadata.nominalRefinementCount, ...
        'scope',"Identical recorded inputs and checked plans outside the physical simulation harness; repeats do not replace the original failed frame");
end

function result = localObserverReplay(trial,period,directory)
    cfg = trial.estimator.configuration;
    cfg.observer.runtime.integrationStepMaximum = cfg.observer.runtime.samplePeriod;
    target = @(time,~) struct('targetPositionInertial',[100-10*time;0.8], ...
        'targetVelocityInertial',[-10;0],'targetAccelerationInertial',[0;0]);
    % These recorded ego states are prescribed inputs to an observer-only
    % replay. They are not trajectories produced by the delayed controller.
    time = (0:period:8).';
    states = interp1(trial.controlTime,trial.controlState,time,'linear');
    initial = localEgo(states(1,:));
    context = nrmmEstimatorControllerAdapter('initialize',cfg,initial,target);
    seconds = zeros(numel(time),1);visible = false(numel(time),1);samples = seconds;
    contexts = cell(3,1);profileTimes = zeros(3,1);profileEgo = cell(3,1);
    largest = zeros(3,1);acquired = false;
    outputs = cell(numel(time),1);
    for frame = 1:numel(time)
        ego = localEgo(states(frame,:));
        before = context;
        streamState = before.randomStream.State;
        timer = tic;
        [context,estimate,targets,~,audit] = nrmmEstimatorControllerAdapter( ...
            'sample',context,time(frame),ego,target(time(frame),[]));
        seconds(frame) = toc(timer);
        visible(frame) = ~isempty(targets);
        samples(frame) = context.observerSamplesSinceLastControllerTime;
        outputs{frame} = struct('estimate',estimate,'targets',targets,'audit',audit);
        group = 1+double(visible(frame));
        if visible(frame) && ~acquired,group = 3;acquired = true;end
        if seconds(frame)>largest(group)
            copiedStream = RandStream('mt19937ar','Seed',cfg.randomSeed);
            copiedStream.State = streamState;before.randomStream = copiedStream;
            contexts{group} = before;profileTimes(group) = time(frame);profileEgo{group} = ego;
            largest(group) = seconds(frame);
        end
    end
    profileResults = cell(3,1);profileTables = cell(3,1);
    labels = ["targetAbsent","targetTracking","firstDetection"];
    profileCleanup = onCleanup(@() profile('off'));
    for group = 1:3
        assert(~isempty(contexts{group}));
        profile clear;profile on -timer real;
        [~,estimate,targets,~,audit] = nrmmEstimatorControllerAdapter('sample',contexts{group}, ...
            profileTimes(group),profileEgo{group},target(profileTimes(group),[]));
        profile off;stats = profile('info');profileResults{group} = stats;
        profileTables{group} = localProfileRows(stats);
        selected = find(abs(time-profileTimes(group))<1e-9,1);
        assert(isequaln(estimate,outputs{selected}.estimate) && isequaln(targets,outputs{selected}.targets) ...
            && isequaln(audit,outputs{selected}.audit),'Observer profile must preserve the exact sensor stream.');
        writetable(profileTables{group},fullfile(directory,'observer-'+labels(group)+'-profile.csv'));
    end
    save(fullfile(directory,'observer-replay.mat'),'cfg','time','seconds','visible','samples','contexts', ...
        'profileTimes','profileEgo','profileResults','outputs','states','-v7.3');
    writetable(table(time,seconds,visible,samples),fullfile(directory,'observer-replay.csv'));
    result = struct('controlPeriodSeconds',period,'sensorPeriodSeconds',cfg.observer.runtime.samplePeriod, ...
        'integrationStepMaximum',cfg.observer.runtime.integrationStepMaximum,'frameCount',numel(time), ...
        'medianSeconds',median(seconds),'maximumSeconds',max(seconds),'profileTimes',profileTimes, ...
        'profileLabels',labels,'profiledSampleUnprofiledSeconds',largest, ...
        'visibleMedianSeconds',median(seconds(visible)),'absentMedianSeconds',median(seconds(~visible)), ...
        'profiles',struct(),'scope',"Observer-only prescribed historical trajectory at current 100 ms output cadence; no coupled controller timing or closed-loop claim");
    for group = 1:3
        rows = profileTables{group};result.profiles.(labels(group)) = table2struct(rows(1:min(20,height(rows)),:));
    end
end

function ego = localEgo(state)
    ego = struct('position',state(1:2).','yawAngle',state(3), ...
        'longitudinalVelocity',state(4),'lateralVelocity',state(5),'yawRate',state(6));
end

function rows = localProfileRows(stats)
    values = stats.FunctionTable;
    self = zeros(numel(values),1);
    for index = 1:numel(values)
        self(index) = values(index).TotalTime-sum([values(index).Children.TotalTime]);
    end
    rows = table(string({values.FunctionName}).',string({values.FileName}).', ...
        [values.NumCalls].',[values.TotalTime].',self, ...
        'VariableNames',{'Function','File','Calls','InclusiveSeconds','SelfSeconds'});
    rows = sortrows(rows,'SelfSeconds','descend');
end

function localJson(path,value)
    file = fopen(path,'w');cleanup = onCleanup(@() fclose(file));
    fprintf(file,'%s\n',jsonencode(value,PrettyPrint=true));
end
