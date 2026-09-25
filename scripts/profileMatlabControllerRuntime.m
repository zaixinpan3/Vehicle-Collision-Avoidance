function summary=profileMatlabControllerRuntime(directory,options)
%profileMatlabControllerRuntime Separate clean timing from diagnostic profiles.
% Run each mode in a fresh MATLAB -singleCompThread process. Campaign timings
% include measurement construction; fixture replays time the controller only.
% The original MATLAB controller and its existing Clarabel MEX remain active.
    arguments
        directory (1,1) string
        options.Mode (1,1) string {mustBeMember(options.Mode,["campaign","replay","profile","probes"])} = "campaign"
        options.SampleCount (1,1) double {mustBeInteger,mustBePositive} = 600
        options.CampaignRepetitions (1,1) double {mustBeInteger,mustBePositive} = 2
        options.Repetitions (1,1) double {mustBeInteger,mustBePositive} = 11
        options.Warmups (1,1) double {mustBeInteger,mustBeNonnegative} = 5
        options.Selection (1,1) string {mustBeMember(options.Selection,["standard","longest","initialization"])} = "standard"
        options.CampaignDirectory (1,1) string = ""
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'), ...
        fullfile(root,'scripts'),fullfile(root,'solver','clarabel','matlab'));
    originalThreads=maxNumCompThreads(1);cleanup=onCleanup(@()maxNumCompThreads(originalThreads));
    if ~isfolder(directory),mkdir(directory);end
    assert(strcmp(which('collisionAvoidanceController'),fullfile(root,'controller','collisionAvoidanceController.m')));
    profile off;
    if options.Mode=="campaign"
        runJointSupportCertificateValidation(OutputDirectory=fullfile(directory,'warmup'), ...
            SampleCount=120,Curvatures=[0,.01]);
        summary=struct('scope',"MATLAB full frames after warmup; admission retained", ...
            'matlabVersion',string(version),'trials',{{}});
        for repetition=1:options.CampaignRepetitions
            folder=fullfile(directory,sprintf('measured-%d',repetition));
            trial=runJointSupportCertificateValidation(OutputDirectory=folder, ...
                SampleCount=options.SampleCount,Curvatures=[0,.01]);
            summary.trials{end+1}=trial;
        end
        localWrite(fullfile(directory,'campaign.json'),summary);
        return;
    end
    if options.Mode=="replay"
        if options.Selection=="initialization"
            assert(strlength(options.CampaignDirectory)>0,'Specify the saved diagnostic campaign directory.');
            definitions=localInitializationDefinitions(options.CampaignDirectory);
        elseif options.Selection=="longest"
            assert(strlength(options.CampaignDirectory)>0,'Specify the saved diagnostic campaign directory.');
            definitions=localLongestDefinitions(options.CampaignDirectory);
        else
            definitions=localDefinitions(directory,options.CampaignRepetitions);
        end
        summary=struct('scope',"MATLAB controller-only warm replay; profiler never enabled",'fixtures',{{}});
        for index=1:numel(definitions)
            item=definitions{index};loaded=load(item.source,'report');report=loaded.report;
            stored=[];prefixDifference=0;
            for frame=1:item.frame-1
                [ego,target]=localScene(report,frame);
                [command,~,~,stored]=collisionAvoidanceController(ego,target,report.road,report.configuration,stored);
                prefixDifference=max(prefixDifference,max(abs(command.actuatorInput-report.input(:,frame))));
            end
            assert(prefixDifference<=1e-7,'profileMatlabControllerRuntime:prefixChanged','Prefix command changed.');
            [ego,target]=localScene(report,item.frame);
            fixture=struct('ego',ego,'target',target,'road',report.road,'cfg',report.configuration, ...
                'stored',stored,'expectedFailure',item.frame>report.executedHolds);
            for warmup=1:options.Warmups,localInvoke(fixture);end
            samples=cell(options.Repetitions,1);expected=[];
            for repetition=1:options.Repetitions
                [sample,decision]=localInvoke(fixture);
                if repetition==1,expected=decision;end
                assert(isequaln(decision,expected),'profileMatlabControllerRuntime:decisionChanged','Replay decision changed.');
                if ~fixture.expectedFailure
                    assert(max(abs(sample.input-report.input(:,item.frame)))<=1e-7);
                end
                samples{repetition}=sample;
            end
            fixture.expectedDecision=expected;
            fixtureFile=fullfile(directory,item.name+'-fixture.mat');
            save(fixtureFile,'fixture','-v7');
            summary.fixtures{end+1}=struct('definition',item,'file',fixtureFile, ...
                'prefixCommandDifference',prefixDifference,'samples',{samples});
            localWrite(fullfile(directory,'replay.json'),summary);
            fprintf('%s: clean median/max %.3f/%.3f ms\n',item.name, ...
                1000*median(cellfun(@(s)s.seconds,samples)),1000*max(cellfun(@(s)s.seconds,samples)));
        end
        return;
    end
    replay=jsondecode(fileread(fullfile(directory,'replay.json')));
    if options.Mode=="probes"
        addpath(fullfile(directory,'instrumented'),'-begin');
        clear collisionAvoidanceController solveHardCbfClf hardEncounterBarrier avoidanceStageQp
        assert(startsWith(which('collisionAvoidanceController'),fullfile(directory,'instrumented')));
    end
    summary=struct('scope',"Instrumented diagnostic times; not deadline measurements",'fixtures',{{}});
    for index=1:numel(replay.fixtures)
        item=replay.fixtures(index);loaded=load(item.file,'fixture');fixture=loaded.fixture;
        if options.Mode=="probes",matlabControllerProbe('reset');end
        for warmup=1:options.Warmups,localInvoke(fixture);end
        if options.Mode=="profile"
            profile clear;profile on;
            [sample,decision]=localInvoke(fixture);
            profile off;information=profile('info');
            save(fullfile(directory,string(item.definition.name)+'-profile.mat'),'information','sample');
            functions=localFunctions(information);
            entry=struct('definition',item.definition,'sample',sample,'functions',{functions});
        else
            samples=cell(options.Repetitions,1);
            for repetition=1:options.Repetitions
                matlabControllerProbe('reset');
                [sample,decision]=localInvoke(fixture);
                assert(isequaln(decision,fixture.expectedDecision),'Instrumented decision changed.');
                sample.probes=matlabControllerProbe('get');samples{repetition}=sample;
            end
            entry=struct('definition',item.definition,'samples',{samples});
        end
        assert(isequaln(decision,fixture.expectedDecision),'Profiled decision changed.');
        summary.fixtures{end+1}=entry;
        localWrite(fullfile(directory,options.Mode+'.json'),summary);
        fprintf('%s: %s collected; decision unchanged\n',item.definition.name,options.Mode);
    end
end

function definitions=localInitializationDefinitions(directory)
% Select the slowest fresh admission in each of the three deadline bottlenecks.
    files=dir(fullfile(directory,'diagnostic','*','*-exact-state.mat'));
    names=["straight-stationary","circular-stationary","circular-crossing"];
    definitions=cell(1,3);maximum=-inf(1,3);
    for index=1:numel(files)
        source=fullfile(files(index).folder,files(index).name);
        loaded=load(source,'report');report=loaded.report;
        selected=find([report.roadCurvature==0 && report.scenario=="stationary", ...
            report.roadCurvature~=0 && report.scenario=="stationary", ...
            report.roadCurvature~=0 && report.scenario=="crossing"]);
        if isempty(selected),continue;end
        for frame=1:report.executedHolds
            search=report.admissionSearch{frame};seconds=report.runtime.frameSeconds(frame);
            if ~isfield(search,'initialCertificateAngles') || report.inheritedFeasibleFamily(frame) ...
                    || seconds<=maximum(selected),continue;end
            definitions{selected}=struct('name',names(selected),'kind',"freshAdmission", ...
                'source',string(source),'frame',frame,'campaignSeconds',seconds, ...
                'originalPhase',report.runtimeBreakdown{frame});
            maximum(selected)=seconds;
        end
    end
    assert(all(isfinite(maximum)),'Three saved bottleneck admissions are required.');
end

function definitions=localLongestDefinitions(directory)
% Select measured maxima from saved successful frames, retaining original
% timing buckets. Warmup and strict-deadline trials are separate populations.
    files=dir(fullfile(directory,'diagnostic','*','*-exact-state.mat'));
    assert(~isempty(files),'No saved diagnostic reports found.');
    definitions=cell(1,2);maximum=[-Inf,-Inf];
    names=["measured-longest","circular-crossing-longest"];
    for index=1:numel(files)
        source=fullfile(files(index).folder,files(index).name);
        loaded=load(source,'report');report=loaded.report;
        [seconds,frame]=max(report.runtime.frameSeconds(1:report.executedHolds));
        eligible=[true,report.roadCurvature~=0 && startsWith(files(index).name,'crossing-')];
        for selected=find(eligible & seconds>maximum)
            phase=report.runtimeBreakdown{frame};
            residual=seconds-phase.inputPreparationSeconds-phase.formulationSeconds-phase.solveSeconds;
            definition=struct('name',names(selected),'kind',"measuredMaximum", ...
                'source',string(source),'frame',frame,'campaignSeconds',seconds, ...
                'originalPhase',phase,'originalOtherSeconds',residual, ...
                'originalSearch',report.admissionSearch{frame});
            definitions{selected}=definition;maximum(selected)=seconds;
        end
    end
    assert(all(isfinite(maximum)),'Both overall and circular-crossing maxima are required.');
end

function definitions=localDefinitions(directory,repetitions)
    definitions={};
    for curvature=[0,.01]
        for scenario=["stationary","oncoming","crossing"]
            name=sprintf('%s-%g',scenario,curvature);reports=cell(repetitions,1);sources=strings(repetitions,1);
            slowest=-Inf;slowFrame=0;slowTrial=0;
            for trial=1:repetitions
                sources(trial)=fullfile(directory,sprintf('measured-%d',trial),name,scenario+'-exact-state.mat');
                saved=load(sources(trial),'report');reports{trial}=saved.report;r=saved.report;
                active=localActive(r);
                selected=find(active & r.inheritedFeasibleFamily);
                if ~isempty(selected)
                    [value,at]=max(r.runtime.frameSeconds(selected));
                    if value>slowest,slowest=value;slowFrame=selected(at);slowTrial=trial;end
                end
            end
            report=reports{1};active=localActive(report);
            frame=find(active & ~report.inheritedFeasibleFamily,1);
            if isempty(frame),frame=report.executedHolds+1;end
            definitions{end+1}=struct('name',string(name)+'-admission','kind',"admission", ...
                'source',sources(1),'frame',frame,'campaignSeconds',report.runtime.frameSeconds(frame));
            if slowFrame>0
                definitions{end+1}=struct('name',string(name)+'-continuation','kind',"continuation", ...
                    'source',sources(slowTrial),'frame',slowFrame,'campaignSeconds',slowest);
            end
            if scenario=="stationary"
                selected=find(~active);frame=selected(round(numel(selected)/2));
                definitions{end+1}=struct('name',string(name)+'-cruise','kind',"cruise", ...
                    'source',sources(1),'frame',frame,'campaignSeconds',report.runtime.frameSeconds(frame));
            end
        end
    end
end

function active=localActive(report)
    active=false(1,report.executedHolds);
    for frame=1:report.executedHolds
        active(frame)=isfield(report.admissionSearch{frame},'initialCertificateAngles');
    end
end

function [sample,decision]=localInvoke(fixture)
    timer=tic;decision=[];
    try
        [command,~,problem]=collisionAvoidanceController(fixture.ego,fixture.target,fixture.road,fixture.cfg,fixture.stored);
        seconds=toc(timer);
    catch exception
        seconds=toc(timer);
        if ~fixture.expectedFailure,rethrow(exception);end
        assert(strcmp(exception.identifier,'collisionAvoidanceController:optimizationFailed'));
        sample=struct('seconds',seconds,'failed',true,'message',string(exception.message));return;
    end
    assert(~fixture.expectedFailure,'A previously failed admission unexpectedly succeeded.');
    decision=problem.decision;
    sample=struct('seconds',seconds,'failed',false,'phase',problem.metadata.runtime, ...
        'input',command.actuatorInput,'search',problem.metadata.admissionSearch, ...
        'horizonSteps',problem.metadata.horizonSteps,'certified',problem.metadata.planCertified);
end

function [ego,target]=localScene(report,frame)
    state=report.state(:,frame);time=(frame-1)*report.configuration.controller.sampleTime;
    if report.roadCurvature==0
        position=report.road.centerline(1,:).'+state(1:2);heading=state(3);
    else
        [position,heading]=laneGeometry.fromFrenet(state,report.road);
    end
    assert(~any(report.egoErrorBound) && ~any(report.targetErrorBound));
    ego=struct('position',position,'yaw',heading,'speed',state(4),'lateralVelocity',state(5), ...
        'yawRate',state(6),'stateTime',time,'controllerStateErrorBound',zeros(6,1), ...
        'perception',struct('time',time,'range',16,'completeWithinRange',true));
    if frame>1,ego.heldActuatorInput=report.input(:,frame-1);end
    % The recorded run's exact NRMM measurement (zero bounds draw zero noise).
    target=nrmmTargetMeasurement(report.targetMotion,time,zeros(8,1),RandStream('mt19937ar'));
end

function functions=localFunctions(information)
    data=information.FunctionTable;functions=cell(numel(data),1);
    for index=1:numel(data)
        children=0;
        if ~isempty(data(index).Children),children=sum([data(index).Children.TotalTime]);end
        functions{index}=struct('name',data(index).FunctionName,'file',data(index).FileName, ...
            'calls',data(index).NumCalls,'totalSeconds',data(index).TotalTime, ...
            'selfSeconds',data(index).TotalTime-children,'executedLines',data(index).ExecutedLines);
    end
end

function localWrite(file,value)
    fid=fopen(file,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end
