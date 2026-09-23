function summary = runRecursiveSafetyValidation(options)
%runRecursiveSafetyValidation Reproduce the declared-plant recursion campaign.
% Reports mathematical certificate checks separately from measured deadlines.
    arguments
        options.OutputDirectory (1,1) string
        options.SampleCount (1,1) double {mustBePositive,mustBeInteger} = 600
        options.SampleTime (1,1) double {mustBeFinite,mustBePositive} = 0.05
        options.IncludeDeadlineRuns (1,1) logical = true
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'));
    if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
    cases=["stationary","oncoming","crossing","cruise","uncertainCrossing"];
    entries=cell(1,numel(cases));
    for index=1:numel(cases)
        scenario=cases(index);egoBound=zeros(6,1);targetBound=zeros(8,1);jerk=zeros(2,1);
        if scenario=="uncertainCrossing"
            scenario="crossing";egoBound=[.01;.01;.001;.01;.01;.001];
            targetBound=[.1;.1;.05;.05;.01;.01;.01;.01];jerk=[.02;.02];
        end
        folder=fullfile(options.OutputDirectory,cases(index));
        try
            result=runExactStateRecursiveFeasibilityScenario(Scenario=scenario, ...
                SampleCount=options.SampleCount,SampleTime=options.SampleTime, ...
                DeadlineSeconds=Inf,OutputDirectory=folder, ...
                EgoErrorBound=egoBound,TargetErrorBound=targetBound,TargetJerkAmplitude=jerk);
        catch exception
            file=fullfile(folder,scenario+"-exact-state.mat");
            if ~isfile(file),rethrow(exception);end
            saved=load(file,'report');result=saved.report;
        end
        entries{index}=localSummary(cases(index),result);
    end
    summary=struct('seed',20260912,'sampleTime',options.SampleTime,'roadBoundariesEnabled',false, ...
        'cases',[entries{:}],'deadlineCases',struct([]), ...
        'scope',"Declared held affine plant; numerical simulation is validation, not the recursion proof");
    if options.IncludeDeadlineRuns
        entries=cell(1,3);
        for index=1:3
            scenario=cases(index);
            folder=fullfile(options.OutputDirectory,sprintf('deadline-%gms',1000*options.SampleTime),scenario);
            try
                result=runExactStateRecursiveFeasibilityScenario(Scenario=scenario, ...
                    SampleCount=ceil(6/options.SampleTime),SampleTime=options.SampleTime, ...
                    DeadlineSeconds=options.SampleTime,OutputDirectory=folder);
            catch exception
                file=fullfile(folder,scenario+"-exact-state.mat");
                if ~isfile(file),rethrow(exception);end
                saved=load(file,'report');result=saved.report;
            end
            entries{index}=localSummary(scenario,result);
        end
        summary.deadlineCases=[entries{:}];
    end
    file=fullfile(options.OutputDirectory,'summary.json');
    stream=fopen(file,'w');cleanup=onCleanup(@() fclose(stream));
    fprintf(stream,'%s\n',jsonencode(summary,PrettyPrint=true));
end

function result=localSummary(name,report)
    count=report.executedHolds;times=1000*report.runtime.frameSeconds;
    error=nan(5,1);
    if count>0,error=report.state(2:6,count+1)-[0;0;8;0;0];end
    result=struct('name',name,'completed',report.completed,'passed',report.passed, ...
        'executedHolds',count,'failureTime',report.failureTime,'failureIdentifier',report.failureIdentifier, ...
        'failureMessage',report.failureMessage,'minimumSeparationMargin',report.minimumSampledSeparationMargin, ...
        'minimumModelDomainMargin',report.minimumSampledModelDomainMargin, ...
        'minimumTerminalMargin',min(report.terminalMembershipMargin), ...
        'maximumClfResidual',max(report.clfDissipationResidual), ...
        'inheritedFrames',nnz(report.inheritedFeasibleFamily), ...
        'confirmedReleaseFrames',find(report.confirmedRelease), ...
        'terminalCommands',nnz(report.terminalCommands),'finalTrackingError',error, ...
        'terminalOptimizations',nnz(report.terminalInvariantOptimization), ...
        'approximateSolutionsAccepted',nnz(report.approximateSolveAccepted), ...
        'medianFrameMs',median(times),'maximumFrameMs',max(times), ...
        'deadlineMisses',report.runtime.deadlineMisses,'deadlineSeconds',report.runtime.deadlineSeconds, ...
        'maximumWarmFrameMs',max(times(2:end)));
end
