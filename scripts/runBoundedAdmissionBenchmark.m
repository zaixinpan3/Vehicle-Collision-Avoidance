function summary=runBoundedAdmissionBenchmark(options)
%runBoundedAdmissionBenchmark Warm desktop admission and continuation timing.
% Raw traces and summaries belong to the explicitly selected output folder.
% ReplayFixture optionally names a saved ego/target/road/cfg admission input.
% Only its admission-algorithm settings are replaced by current defaults.
    arguments
        options.OutputDirectory (1,1) string
        options.ReplayFixture (1,1) string = ""
        options.SampleCount (1,1) double {mustBePositive,mustBeInteger} = 300
        options.Repetitions (1,1) double {mustBePositive,mustBeInteger} = 2
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
    oldThreads=maxNumCompThreads(1);cleanup=onCleanup(@()maxNumCompThreads(oldThreads));
    profile off;
    summary=struct('method',"boundedJointSupport",'sampleCount',options.SampleCount, ...
        'repetitions',options.Repetitions,'replayFixture',options.ReplayFixture, ...
        'matlabVersion',string(version),'replays',{{}},'campaigns',{{}},'strict',{{}});
    curvatures=[0,.01,-.01,.02,-.02];
    runJointSupportCertificateValidation(OutputDirectory=fullfile(options.OutputDirectory,'warmup'), ...
        SampleCount=60,Curvatures=curvatures);
    if strlength(options.ReplayFixture)>0
        data=load(options.ReplayFixture,'ego','target','road','cfg');
        defaults=collisionAvoidanceControllerConfig();
        data.cfg.jointCertificate=defaults.jointCertificate;
        save(fullfile(options.OutputDirectory,'current-fixture.mat'),'-struct','data');
        for index=1:20,localReplay(data);end
        for index=1:30,summary.replays{end+1}=localReplay(data);end
        localWrite(summary,options.OutputDirectory);
    end
    for repetition=1:options.Repetitions
        summary.campaigns{end+1}=runJointSupportCertificateValidation( ...
            OutputDirectory=fullfile(options.OutputDirectory,sprintf('measured-%d',repetition)), ...
            SampleCount=options.SampleCount,Curvatures=curvatures);
        localWrite(summary,options.OutputDirectory);
    end
    for curvature=curvatures
        for scenario=["stationary","oncoming","crossing"]
            directory=fullfile(options.OutputDirectory,'strict',sprintf('%s-%g',scenario,curvature));
            try
                result=runExactStateRecursiveFeasibilityScenario(Scenario=scenario,RoadCurvature=curvature, ...
                    SampleCount=options.SampleCount,DeadlineSeconds=.1,SearchTimeLimitSeconds=.1, ...
                    OutputDirectory=directory);
            catch exception
                artifact=fullfile(directory,scenario+'-exact-state.mat');
                if ~isfile(artifact),rethrow(exception);end
                saved=load(artifact,'report');result=saved.report;
            end
            summary.strict{end+1}=struct('scenario',scenario,'curvature',curvature, ...
                'completed',result.completed,'passed',result.passed, ...
                'executedHolds',result.executedHolds,'maximumSeconds',result.runtime.maximumSeconds, ...
                'minimumSampledBodyGap',result.minimumSampledBodyGap, ...
                'failureTime',result.failureTime,'failureMessage',result.failureMessage);
            localWrite(summary,options.OutputDirectory);
        end
    end
end

function item=localReplay(data)
    timer=tic;
    try
        [command,~,problem]=collisionAvoidanceController(data.ego,data.target,data.road,data.cfg,[]);
        item=struct('seconds',toc(timer),'accepted',true,'command',command.actuatorInput, ...
            'calls',problem.metadata.solverCallCount,'search',problem.metadata.admissionSearch, ...
            'runtime',problem.metadata.runtime);
    catch exception
        item=struct('seconds',toc(timer),'accepted',false, ...
            'identifier',exception.identifier,'message',exception.message);
    end
end

function localWrite(summary,directory)
    file=fopen(fullfile(directory,'benchmark.json'),'w');assert(file>=0);
    cleanup=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(summary,PrettyPrint=true));
end
